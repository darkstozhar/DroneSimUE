#include "Sim/DroneTileStore.h"

#include "Async/Async.h"
#include "HAL/FileManager.h"
#include "HttpModule.h"
#include "HttpPath.h"
#include "HttpServerModule.h"
#include "HttpServerRequest.h"
#include "HttpServerResponse.h"
#include "IHttpRouter.h"
#include "Interfaces/IHttpRequest.h"
#include "Interfaces/IHttpResponse.h"
#include "Misc/ConfigCacheIni.h"
#include "Misc/DateTime.h"
#include "Misc/FileHelper.h"
#include "Misc/Guid.h"
#include "Misc/Paths.h"
#include "Misc/SecureHash.h"

DEFINE_LOG_CATEGORY_STATIC(LogDroneTiles, Log, All);

// Only touched on the game thread (request handler, HTTP completion, AsyncTask to the game thread).
struct FDroneTileStore::FState
{
    FString Root;
    TMap<FString, FString> Upstreams;
    FDroneTileStore::FStats Stats;
    // Contents of "no imagery here" tiles (Esri: a gray "Map data not yet available" JPEG).
    TSet<FString> PlaceholderHashes;
    TMap<FString, int32> SmallTileRepeats; // identical small downloaded tiles this run, by MD5
};

// Named (not anonymous) so these do not collide with other files in a unity build.
namespace DroneTileStoreDetail
{
using FWeakTileState = TWeakPtr<FDroneTileStore::FState, ESPMode::ThreadSafe>;
using FTileStatePtr = TSharedPtr<FDroneTileStore::FState, ESPMode::ThreadSafe>;

constexpr float UpstreamTimeoutS = 25.f;
// Esri World Imagery's "Map data not yet available" tile (256 x 256 JPEG, 2521 bytes),
// found 1096 times at level 19 in the store on 2026-10-21.
const TCHAR* const EsriPlaceholderMd5 = TEXT("f27d9de7f80c13501f470595e327aa6d");
// Byte-identical downloaded tiles this small, at this level or deeper, from this many
// places are taken as another such placeholder.
constexpr int32 SmallTileBytes = 4096;
constexpr int32 PlaceholderMinLevel = 14;
constexpr int32 PlaceholderRepeats = 4;
// A "no imagery here" marker is trusted this long; then the tile is asked for again.
constexpr double NoImageryMarkerDays = 60.0;

bool ReadTileNumber(const FHttpServerRequest& Request, const TCHAR* Name, int32& Out)
{
    const FString* Text = Request.QueryParams.Find(Name);
    if (Text == nullptr || Text->IsEmpty() || Text->Len() > 9)
    {
        return false;
    }
    for (const TCHAR Character : *Text)
    {
        if (Character < TEXT('0') || Character > TEXT('9'))
        {
            return false;
        }
    }
    Out = FCString::Atoi(**Text);
    return true;
}

// <root>/<source>/<z>/<x>/<y>: + ".tile" (the bytes as received, JPEG or PNG) or + ".none" (no imagery there).
FString TileBase(const FString& Root, const FString& SourceId, int32 Z, int32 X, int32 Y)
{
    return FPaths::Combine(Root, SourceId, FString::FromInt(Z), FString::FromInt(X), FString::FromInt(Y));
}

FString SignaturesFile(const FString& Root)
{
    return FPaths::Combine(Root, TEXT("placeholders.txt"));
}

bool IsPng(const TArray<uint8>& Bytes)
{
    return Bytes.Num() > 8 && Bytes[0] == 0x89 && Bytes[1] == 'P' && Bytes[2] == 'N' && Bytes[3] == 'G';
}

bool LooksLikeImage(const TArray<uint8>& Bytes)
{
    const bool bJpeg = Bytes.Num() > 8 && Bytes[0] == 0xFF && Bytes[1] == 0xD8;
    return bJpeg || IsPng(Bytes);
}

// Only small tiles can be placeholders; larger ones are not hashed.
FString SmallTileHash(const TArray<uint8>& Bytes)
{
    return (Bytes.Num() > 0 && Bytes.Num() <= SmallTileBytes) ? FMD5::HashBytes(Bytes.GetData(), static_cast<uint64>(Bytes.Num()))
                                                              : FString();
}

TUniquePtr<FHttpServerResponse> ImageResponse(TArray<uint8>&& Bytes)
{
    const FString ContentType = IsPng(Bytes) ? TEXT("image/png") : TEXT("image/jpeg");
    TUniquePtr<FHttpServerResponse> Response = FHttpServerResponse::Create(MoveTemp(Bytes), ContentType);
    // Kept here already: no second copy in Cesium's request cache.
    TArray<FString> NoStore;
    NoStore.Add(TEXT("no-store"));
    Response->Headers.Add(TEXT("Cache-Control"), MoveTemp(NoStore));
    return Response;
}

// Thread pool.
void SaveTile(const FString& Path, const TArray<uint8>& Bytes)
{
    IFileManager& Files = IFileManager::Get();
    Files.MakeDirectory(*FPaths::GetPath(Path), /*Tree*/ true);
    // Written under a temporary name first, so a crash never leaves half a tile.
    const FString Temporary = FString::Printf(TEXT("%s.%s.part"), *Path, *FGuid::NewGuid().ToString(EGuidFormats::Digits));
    if (FFileHelper::SaveArrayToFile(Bytes, *Temporary))
    {
        if (!Files.Move(*Path, *Temporary, /*bReplace*/ true, /*bEvenIfReadOnly*/ true))
        {
            Files.Delete(*Temporary, false, true, true);
        }
    }
}

// Thread pool: "no imagery here" instead of a stored placeholder.
void WriteNoImageryMarker(const FString& Base)
{
    IFileManager& Files = IFileManager::Get();
    Files.MakeDirectory(*FPaths::GetPath(Base), /*Tree*/ true);
    FFileHelper::SaveStringToFile(TEXT("no imagery"), *(Base + TEXT(".none")));
    Files.Delete(*(Base + TEXT(".tile")), false, true, true);
}

// 404: Cesium then shows the parent tile's imagery there (cesium-native
// QuadtreeRasterOverlayTileProvider falls back to the parent when a tile fails).
void AnswerNoImagery(const FTileStatePtr& StatePtr, const FHttpResultCallback& Done)
{
    if (StatePtr.IsValid())
    {
        ++StatePtr->Stats.NoImagery;
    }
    Done(FHttpServerResponse::Error(EHttpServerResponseCodes::NotFound));
}

bool IsPlaceholder(const FDroneTileStore::FState& TileState, const FString& Hash)
{
    return !Hash.IsEmpty() && TileState.PlaceholderHashes.Contains(Hash);
}

// A small tile downloaded identical from several places becomes a known placeholder.
bool LearnPlaceholder(FDroneTileStore::FState& TileState, const FString& Hash, int32 Z)
{
    if (Hash.IsEmpty() || Z < PlaceholderMinLevel)
    {
        return false;
    }
    int32& Repeats = TileState.SmallTileRepeats.FindOrAdd(Hash);
    ++Repeats;
    if (Repeats < PlaceholderRepeats)
    {
        return false;
    }
    TileState.SmallTileRepeats.Remove(Hash);
    TileState.PlaceholderHashes.Add(Hash);
    FFileHelper::SaveStringToFile(Hash + TEXT("\n"), *SignaturesFile(TileState.Root), FFileHelper::EEncodingOptions::AutoDetect,
                                  &IFileManager::Get(), FILEWRITE_Append);
    UE_LOG(LogDroneTiles, Display, TEXT("Tile store: content %s came %d times, treated as \"no imagery here\" from now on"),
           *Hash, PlaceholderRepeats);
    return true;
}

void NoteFailure(const FWeakTileState& WeakState, const FString& Url, int32 Code)
{
    const FTileStatePtr StatePtr = WeakState.Pin();
    if (!StatePtr.IsValid())
    {
        return;
    }
    ++StatePtr->Stats.Failed;
    const int64 Failed = StatePtr->Stats.Failed;
    if (Failed <= 10 || Failed % 100 == 0)
    {
        UE_LOG(LogDroneTiles, Warning, TEXT("Tile download failed (%lld so far): HTTP %d for %s"), Failed, Code, *Url);
    }
}

void Download(const FWeakTileState& WeakState, const FString& Base, const FString& Url, int32 Z, const FHttpResultCallback& Done)
{
    TSharedRef<IHttpRequest, ESPMode::ThreadSafe> Request = FHttpModule::Get().CreateRequest();
    Request->SetURL(Url);
    Request->SetVerb(TEXT("GET"));
    Request->SetHeader(TEXT("User-Agent"), TEXT("DroneSimUE"));
    Request->SetTimeout(UpstreamTimeoutS);
    Request->OnProcessRequestComplete().BindLambda(
        [WeakState, Base, Url, Z, Done](FHttpRequestPtr, FHttpResponsePtr Response, bool bConnected)
        {
            const FTileStatePtr StatePtr = WeakState.Pin();
            if (StatePtr.IsValid())
            {
                --StatePtr->Stats.Pending;
            }
            const int32 Code = Response.IsValid() ? Response->GetResponseCode() : 0;
            if (bConnected && Code == 404)
            {
                // No tile at this level (older Wayback releases often stop at 17 or 18): remembered.
                Async(EAsyncExecution::ThreadPool, [Base]() { WriteNoImageryMarker(Base); });
                AnswerNoImagery(StatePtr, Done);
                return;
            }
            if (!bConnected || !Response.IsValid() || Code != 200 || !LooksLikeImage(Response->GetContent()))
            {
                NoteFailure(WeakState, Url, Code);
                Done(FHttpServerResponse::Error(Code == 404 ? EHttpServerResponseCodes::NotFound
                                                            : EHttpServerResponseCodes::BadGateway));
                return;
            }
            TArray<uint8> Bytes = Response->GetContent();
            const FString Hash = SmallTileHash(Bytes);
            if (StatePtr.IsValid() && (IsPlaceholder(*StatePtr, Hash) || LearnPlaceholder(*StatePtr, Hash, Z)))
            {
                Async(EAsyncExecution::ThreadPool, [Base]() { WriteNoImageryMarker(Base); });
                AnswerNoImagery(StatePtr, Done);
                return;
            }
            if (StatePtr.IsValid())
            {
                ++StatePtr->Stats.Downloaded;
                StatePtr->Stats.DownloadedBytes += Bytes.Num();
            }
            TArray<uint8> ToSave = Bytes;
            Async(EAsyncExecution::ThreadPool,
                  [Base, SaveBytes = MoveTemp(ToSave)]() { SaveTile(Base + TEXT(".tile"), SaveBytes); });
            Done(ImageResponse(MoveTemp(Bytes)));
        });
    Request->ProcessRequest();
}

bool HandleTileRequest(const FWeakTileState& WeakState, const FHttpServerRequest& Request, const FHttpResultCallback& OnComplete)
{
    const FTileStatePtr StatePtr = WeakState.Pin();
    const FString* SourceId = Request.QueryParams.Find(TEXT("s"));
    const FString* Upstream = (StatePtr.IsValid() && SourceId != nullptr) ? StatePtr->Upstreams.Find(*SourceId) : nullptr;
    int32 Z = 0;
    int32 X = 0;
    int32 Y = 0;
    const bool bNumbers = ReadTileNumber(Request, TEXT("z"), Z) && ReadTileNumber(Request, TEXT("x"), X) &&
                          ReadTileNumber(Request, TEXT("y"), Y);
    if (Upstream == nullptr || !bNumbers || Z > 30 || X >= (1 << Z) || Y >= (1 << Z))
    {
        OnComplete(FHttpServerResponse::Error(EHttpServerResponseCodes::BadRequest));
        return true;
    }
    const FString Base = TileBase(StatePtr->Root, *SourceId, Z, X, Y);
    const FString Url = Upstream->Replace(TEXT("{z}"), *FString::FromInt(Z))
                            .Replace(TEXT("{x}"), *FString::FromInt(X))
                            .Replace(TEXT("{y}"), *FString::FromInt(Y));
    ++StatePtr->Stats.Pending;
    // The reference is only valid during this call; the copy answers later.
    const FHttpResultCallback Done = OnComplete;
    Async(EAsyncExecution::ThreadPool,
          [WeakState, Base, Url, Z, Done]()
          {
              TArray<uint8> Bytes;
              const bool bOnDisk =
                  FFileHelper::LoadFileToArray(Bytes, *(Base + TEXT(".tile")), FILEREAD_Silent) && LooksLikeImage(Bytes);
              const FString Hash = bOnDisk ? SmallTileHash(Bytes) : FString();
              bool bNoImageryMarked = false;
              if (!bOnDisk)
              {
                  const FDateTime Marked = IFileManager::Get().GetTimeStamp(*(Base + TEXT(".none")));
                  bNoImageryMarked = Marked != FDateTime::MinValue() && (FDateTime::UtcNow() - Marked).GetTotalDays() < NoImageryMarkerDays;
              }
              AsyncTask(ENamedThreads::GameThread,
                        [WeakState, Base, Url, Z, Done, bOnDisk, bNoImageryMarked, Hash, TileBytes = MoveTemp(Bytes)]() mutable
                        {
                            const FTileStatePtr Pinned = WeakState.Pin();
                            const bool bLoaded = bOnDisk || bNoImageryMarked;
                            if (bLoaded && Pinned.IsValid())
                            {
                                --Pinned->Stats.Pending;
                            }
                            if (bOnDisk && Pinned.IsValid() && IsPlaceholder(*Pinned, Hash))
                            {
                                // Stored before it was known as a placeholder: replaced by the marker.
                                Async(EAsyncExecution::ThreadPool, [Base]() { WriteNoImageryMarker(Base); });
                                AnswerNoImagery(Pinned, Done);
                                return;
                            }
                            if (bOnDisk)
                            {
                                if (Pinned.IsValid())
                                {
                                    ++Pinned->Stats.FromDisk;
                                }
                                Done(ImageResponse(MoveTemp(TileBytes)));
                                return;
                            }
                            if (bNoImageryMarked)
                            {
                                AnswerNoImagery(Pinned, Done);
                                return;
                            }
                            Download(WeakState, Base, Url, Z, Done);
                        });
          });
    return true;
}

// Cesium opens up to ~20 connections at once; the HTTP server's defaults
// (backlog 16, one new connection per frame) could refuse some of them.
void EnsureListenerConfig(int32 Port)
{
    const TCHAR* Section = TEXT("HTTPServer.Listeners");
    TArray<FString> Overrides;
    GConfig->GetArray(Section, TEXT("ListenerOverrides"), Overrides, GEngineIni);
    const FString PortKey = FString::Printf(TEXT("Port=%d"), Port);
    for (const FString& Entry : Overrides)
    {
        if (Entry.Contains(PortKey))
        {
            return; // set in Config/DefaultEngine.ini
        }
    }
    Overrides.Add(FString::Printf(TEXT("(Port=%d, ConnectionsBacklogSize=64, MaxConnectionsAcceptPerFrame=16)"), Port));
    GConfig->SetArray(Section, TEXT("ListenerOverrides"), Overrides, GEngineIni);
}
} // namespace DroneTileStoreDetail

FDroneTileStore::FDroneTileStore()
    : State(MakeShared<FState, ESPMode::ThreadSafe>())
{
}

FDroneTileStore::~FDroneTileStore()
{
    Stop();
}

bool FDroneTileStore::Start(int32 Port, const FString& RootDir)
{
    State->Root = RootDir;
    State->PlaceholderHashes.Add(DroneTileStoreDetail::EsriPlaceholderMd5);
    TArray<FString> Learned;
    if (FFileHelper::LoadFileToStringArray(Learned, *DroneTileStoreDetail::SignaturesFile(RootDir)))
    {
        for (const FString& Line : Learned)
        {
            const FString Hash = Line.TrimStartAndEnd();
            if (!Hash.IsEmpty())
            {
                State->PlaceholderHashes.Add(Hash);
            }
        }
    }
    if (Route.IsValid())
    {
        return true;
    }
    const int32 UsePort = FMath::Clamp(Port, 1024, 65535);
    DroneTileStoreDetail::EnsureListenerConfig(UsePort);
    FHttpServerModule& Server = FHttpServerModule::Get();
    // Listeners only bind once enabled; with them enabled GetHttpRouter binds at once and reports failure.
    Server.StartAllListeners();
    Router = Server.GetHttpRouter(static_cast<uint32>(UsePort), /*bFailOnBindFailure*/ true);
    if (!Router.IsValid())
    {
        UE_LOG(LogDroneTiles, Error, TEXT("Tile store: cannot listen on 127.0.0.1:%d (port in use?)"), UsePort);
        return false;
    }
    const DroneTileStoreDetail::FWeakTileState WeakState = State;
    Route = Router->BindRoute(FHttpPath(TEXT("/tile")), EHttpServerRequestVerbs::VERB_GET,
                              FHttpRequestHandler::CreateLambda(
                                  [WeakState](const FHttpServerRequest& Request, const FHttpResultCallback& OnComplete)
                                  { return DroneTileStoreDetail::HandleTileRequest(WeakState, Request, OnComplete); }));
    if (!Route.IsValid())
    {
        UE_LOG(LogDroneTiles, Error, TEXT("Tile store: /tile is already bound on port %d"), UsePort);
        Router.Reset();
        return false;
    }
    ListenPort = UsePort;
    UE_LOG(LogDroneTiles, Display, TEXT("Tile store: http://127.0.0.1:%d/tile, tiles kept in %s (%d placeholder signatures)"),
           ListenPort, *RootDir, State->PlaceholderHashes.Num());
    return true;
}

void FDroneTileStore::Stop()
{
    if (Router.IsValid() && Route.IsValid())
    {
        Router->UnbindRoute(Route);
    }
    Route.Reset();
    Router.Reset();
}

void FDroneTileStore::SetSource(const FString& SourceId, const FString& UpstreamTemplate)
{
    State->Upstreams.Add(SourceId, UpstreamTemplate);
}

FString FDroneTileStore::OverlayTemplateUrl(const FString& SourceId) const
{
    // {reverseY}: Cesium's y counted from the north, as in XYZ tile servers.
    return FString::Printf(TEXT("http://127.0.0.1:%d/tile?s=%s&z={z}&x={x}&y={reverseY}"), ListenPort, *SourceId);
}

FString FDroneTileStore::GetRootDir() const
{
    return State->Root;
}

FDroneTileStore::FStats FDroneTileStore::GetStats() const
{
    return State->Stats;
}
