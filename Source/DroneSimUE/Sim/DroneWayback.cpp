#include "Sim/DroneWayback.h"

#include "Dom/JsonObject.h"
#include "Dom/JsonValue.h"
#include "GenericPlatform/GenericPlatformHttp.h"
#include "HAL/FileManager.h"
#include "HttpModule.h"
#include "Interfaces/IHttpRequest.h"
#include "Interfaces/IHttpResponse.h"
#include "Misc/DateTime.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "Serialization/JsonWriter.h"

DEFINE_LOG_CATEGORY_STATIC(LogDroneWayback, Log, All);

struct FDroneWaybackRelease
{
    int32 Release = 0;
    FString Date;        // YYYY-MM-DD from the item title
    FString MetadataUrl; // metadata MapServer of this release
};

// Only touched on the game thread (callers and HTTP completions).
struct FDroneWayback::FState
{
    TArray<FDroneWaybackRelease> Releases; // newest first
    TArray<FDroneWaybackVersion> Versions; // newest first
    FString Status;
    FString PlaceKey;
    FString CacheDir;
    double Lat = 0.0;
    double Lon = 0.0;
    int32 TileX = 0;
    int32 TileY = 0;
    int32 Generation = 0; // a new place makes the running search stale
    int32 MetadataPending = 0;
};

// Named (not anonymous) so these do not collide with other files in a unity build.
namespace DroneWaybackDetail
{
using FStatePtr = TSharedPtr<FDroneWayback::FState, ESPMode::ThreadSafe>;
using FWeakState = TWeakPtr<FDroneWayback::FState, ESPMode::ThreadSafe>;

const TCHAR* const ConfigUrl = TEXT("https://s3-us-west-2.amazonaws.com/config.maptiles.arcgis.com/waybackconfig.json");
const TCHAR* const MapServerUrl = TEXT("https://wayback.maptiles.arcgis.com/arcgis/rest/services/World_Imagery/MapServer");
constexpr int32 ChangeLevel = 14;   // a change anywhere in this tile (about 1.6 km here) counts
constexpr int32 MetadataLevel = 17; // capture date of the image as shown at this level
constexpr double ConfigMaxAgeDays = 3.0;
constexpr double VersionsMaxAgeDays = 14.0;
constexpr float RequestTimeoutS = 20.f;
constexpr int32 MaxVersions = 200;

FStatePtr Current(const FWeakState& Weak, int32 Generation)
{
    FStatePtr Pinned = Weak.Pin();
    return (Pinned.IsValid() && Pinned->Generation == Generation) ? Pinned : FStatePtr();
}

// First "YYYY-MM-DD" in the text (item titles look like "World Imagery (Wayback 2014-02-20)").
FString FindIsoDate(const FString& Text)
{
    for (int32 Index = 0; Index + 10 <= Text.Len(); ++Index)
    {
        bool bMatch = true;
        for (int32 Offset = 0; Offset < 10 && bMatch; ++Offset)
        {
            const TCHAR Character = Text[Index + Offset];
            bMatch = (Offset == 4 || Offset == 7) ? Character == TEXT('-') : (Character >= TEXT('0') && Character <= TEXT('9'));
        }
        if (bMatch)
        {
            return Text.Mid(Index, 10);
        }
    }
    return FString();
}

void TileOf(double Lat, double Lon, int32 Z, int32& OutX, int32& OutY)
{
    const double Count = static_cast<double>(1 << Z);
    const double LatRad = FMath::DegreesToRadians(FMath::Clamp(Lat, -85.0, 85.0));
    const double X = (Lon + 180.0) / 360.0 * Count;
    const double Y = (1.0 - FMath::Loge(FMath::Tan(LatRad) + 1.0 / FMath::Cos(LatRad)) / UE_DOUBLE_PI) / 2.0 * Count;
    OutX = FMath::Clamp(FMath::FloorToInt(X), 0, (1 << Z) - 1);
    OutY = FMath::Clamp(FMath::FloorToInt(Y), 0, (1 << Z) - 1);
}

double FileAgeDays(const FString& Path)
{
    const FDateTime Stamp = IFileManager::Get().GetTimeStamp(*Path);
    return Stamp == FDateTime::MinValue() ? 1.0e9 : (FDateTime::UtcNow() - Stamp).GetTotalDays();
}

FString ConfigFile(const FString& CacheDir)
{
    return FPaths::Combine(CacheDir, TEXT("wayback"), TEXT("waybackconfig.json"));
}

FString VersionsFile(const FString& CacheDir, int32 X, int32 Y)
{
    return FPaths::Combine(CacheDir, TEXT("wayback"), FString::Printf(TEXT("versions_z%d_%d_%d.json"), ChangeLevel, X, Y));
}

TSharedPtr<FJsonObject> ParseObject(const FString& Text)
{
    TSharedPtr<FJsonObject> Object;
    const TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(Text);
    if (!FJsonSerializer::Deserialize(Reader, Object))
    {
        return nullptr;
    }
    return Object;
}

bool ParseConfig(const FString& Text, TArray<FDroneWaybackRelease>& Out)
{
    const TSharedPtr<FJsonObject> Object = ParseObject(Text);
    if (!Object.IsValid())
    {
        return false;
    }
    TArray<FDroneWaybackRelease> Parsed;
    for (const TPair<FString, TSharedPtr<FJsonValue>>& Pair : Object->Values)
    {
        const TSharedPtr<FJsonObject>* Item = nullptr;
        if (!Pair.Key.IsNumeric() || !Pair.Value.IsValid() || !Pair.Value->TryGetObject(Item) || Item == nullptr)
        {
            continue;
        }
        FString Title;
        FDroneWaybackRelease Entry;
        (*Item)->TryGetStringField(TEXT("itemTitle"), Title);
        (*Item)->TryGetStringField(TEXT("metadataLayerUrl"), Entry.MetadataUrl);
        Entry.Release = FCString::Atoi(*Pair.Key);
        Entry.Date = FindIsoDate(Title);
        if (Entry.Release > 0 && !Entry.Date.IsEmpty())
        {
            Parsed.Add(MoveTemp(Entry));
        }
    }
    if (Parsed.Num() == 0)
    {
        return false;
    }
    Parsed.Sort([](const FDroneWaybackRelease& A, const FDroneWaybackRelease& B)
                { return A.Date != B.Date ? A.Date > B.Date : A.Release > B.Release; });
    Out = MoveTemp(Parsed);
    return true;
}

bool LoadVersionsFile(const FString& Path, TArray<FDroneWaybackVersion>& Out)
{
    FString Text;
    if (!FFileHelper::LoadFileToString(Text, *Path))
    {
        return false;
    }
    const TSharedPtr<FJsonObject> Object = ParseObject(Text);
    const TArray<TSharedPtr<FJsonValue>>* Items = nullptr;
    if (!Object.IsValid() || !Object->TryGetArrayField(TEXT("versions"), Items) || Items == nullptr)
    {
        return false;
    }
    Out.Reset();
    for (const TSharedPtr<FJsonValue>& Value : *Items)
    {
        const TSharedPtr<FJsonObject>* Item = nullptr;
        if (!Value.IsValid() || !Value->TryGetObject(Item) || Item == nullptr)
        {
            continue;
        }
        FDroneWaybackVersion Version;
        double Release = 0.0;
        (*Item)->TryGetNumberField(TEXT("release"), Release);
        Version.Release = static_cast<int32>(Release);
        (*Item)->TryGetStringField(TEXT("release_date"), Version.ReleaseDate);
        (*Item)->TryGetStringField(TEXT("capture_date"), Version.CaptureDate);
        (*Item)->TryGetStringField(TEXT("provider"), Version.Provider);
        (*Item)->TryGetStringField(TEXT("sensor"), Version.Sensor);
        (*Item)->TryGetNumberField(TEXT("resolution_m"), Version.ResolutionM);
        if (Version.Release > 0)
        {
            Out.Add(MoveTemp(Version));
        }
    }
    return Out.Num() > 0;
}

void SaveVersionsFile(const FString& Path, const TArray<FDroneWaybackVersion>& Versions)
{
    TArray<TSharedPtr<FJsonValue>> Items;
    for (const FDroneWaybackVersion& Version : Versions)
    {
        const TSharedRef<FJsonObject> Item = MakeShared<FJsonObject>();
        Item->SetNumberField(TEXT("release"), Version.Release);
        Item->SetStringField(TEXT("release_date"), Version.ReleaseDate);
        Item->SetStringField(TEXT("capture_date"), Version.CaptureDate);
        Item->SetStringField(TEXT("provider"), Version.Provider);
        Item->SetStringField(TEXT("sensor"), Version.Sensor);
        Item->SetNumberField(TEXT("resolution_m"), Version.ResolutionM);
        Items.Add(MakeShared<FJsonValueObject>(Item));
    }
    const TSharedRef<FJsonObject> Root = MakeShared<FJsonObject>();
    Root->SetArrayField(TEXT("versions"), Items);
    FString Text;
    const TSharedRef<TJsonWriter<>> Writer = TJsonWriterFactory<>::Create(&Text);
    if (FJsonSerializer::Serialize(Root, Writer))
    {
        IFileManager::Get().MakeDirectory(*FPaths::GetPath(Path), /*Tree*/ true);
        FFileHelper::SaveStringToFile(Text, *Path);
    }
}

void HttpGet(const FString& Url, TFunction<void(bool bOk, int32 Code, const FString& Body)> Done)
{
    const TSharedRef<IHttpRequest, ESPMode::ThreadSafe> Request = FHttpModule::Get().CreateRequest();
    Request->SetURL(Url);
    Request->SetVerb(TEXT("GET"));
    Request->SetHeader(TEXT("User-Agent"), TEXT("DroneSimUE"));
    Request->SetTimeout(RequestTimeoutS);
    Request->OnProcessRequestComplete().BindLambda(
        [Callback = MoveTemp(Done)](FHttpRequestPtr, FHttpResponsePtr Response, bool bConnected)
        {
            const int32 Code = Response.IsValid() ? Response->GetResponseCode() : 0;
            Callback(bConnected && Response.IsValid() && Code == 200, Code,
                     Response.IsValid() ? Response->GetContentAsString() : FString());
        });
    Request->ProcessRequest();
}

void Finish(const FStatePtr& StatePtr)
{
    StatePtr->Status = FString::Printf(TEXT("%d different images here (newest first)"), StatePtr->Versions.Num());
    if (StatePtr->Versions.Num() > 0)
    {
        SaveVersionsFile(VersionsFile(StatePtr->CacheDir, StatePtr->TileX, StatePtr->TileY), StatePtr->Versions);
    }
    UE_LOG(LogDroneWayback, Display, TEXT("Wayback: %d different images at %.5f, %.5f"), StatePtr->Versions.Num(), StatePtr->Lat,
           StatePtr->Lon);
}

// When and by what each found image was taken (one request per image, all at once).
void RequestMetadata(const FWeakState& Weak, int32 Generation)
{
    const FStatePtr StatePtr = Current(Weak, Generation);
    if (!StatePtr.IsValid())
    {
        return;
    }
    const int32 Layer = FMath::Clamp(23 - MetadataLevel, 0, 13); // sub-layer 0 = level 23 ... 13 = level 10
    const FString Geometry =
        FString::Printf(TEXT("{\"spatialReference\":{\"wkid\":4326},\"x\":%.7f,\"y\":%.7f}"), StatePtr->Lon, StatePtr->Lat);
    StatePtr->MetadataPending = 0;
    for (const FDroneWaybackVersion& Version : StatePtr->Versions)
    {
        const int32 Release = Version.Release;
        const FDroneWaybackRelease* Entry =
            StatePtr->Releases.FindByPredicate([Release](const FDroneWaybackRelease& Candidate) { return Candidate.Release == Release; });
        if (Entry == nullptr || Entry->MetadataUrl.IsEmpty())
        {
            continue;
        }
        const FString Url = FString::Printf(TEXT("%s/%d/query?f=json&where=1%%3D1&outFields=SRC_DATE2,NICE_DESC,SRC_DESC,SAMP_RES")
                                                TEXT("&geometry=%s&returnGeometry=false&geometryType=esriGeometryPoint")
                                                TEXT("&spatialRel=esriSpatialRelIntersects"),
                                            *Entry->MetadataUrl, Layer, *FGenericPlatformHttp::UrlEncode(Geometry));
        ++StatePtr->MetadataPending;
        HttpGet(Url,
                [Weak, Generation, Release](bool bOk, int32, const FString& Body)
                {
                    const FStatePtr Pinned = Current(Weak, Generation);
                    if (!Pinned.IsValid())
                    {
                        return;
                    }
                    const TSharedPtr<FJsonObject> Object = bOk ? ParseObject(Body) : nullptr;
                    const TArray<TSharedPtr<FJsonValue>>* Features = nullptr;
                    FDroneWaybackVersion* Found = Pinned->Versions.FindByPredicate(
                        [Release](const FDroneWaybackVersion& Candidate) { return Candidate.Release == Release; });
                    if (Found != nullptr && Object.IsValid() && Object->TryGetArrayField(TEXT("features"), Features) &&
                        Features != nullptr && Features->Num() > 0)
                    {
                        const TSharedPtr<FJsonObject>* Feature = nullptr;
                        const TSharedPtr<FJsonObject>* Attributes = nullptr;
                        if ((*Features)[0].IsValid() && (*Features)[0]->TryGetObject(Feature) && Feature != nullptr &&
                            (*Feature)->TryGetObjectField(TEXT("attributes"), Attributes) && Attributes != nullptr)
                        {
                            double Milliseconds = 0.0;
                            if ((*Attributes)->TryGetNumberField(TEXT("SRC_DATE2"), Milliseconds) && Milliseconds > 0.0)
                            {
                                Found->CaptureDate =
                                    FDateTime::FromUnixTimestamp(static_cast<int64>(Milliseconds / 1000.0)).ToString(TEXT("%Y-%m-%d"));
                            }
                            (*Attributes)->TryGetStringField(TEXT("NICE_DESC"), Found->Provider);
                            (*Attributes)->TryGetStringField(TEXT("SRC_DESC"), Found->Sensor);
                            (*Attributes)->TryGetNumberField(TEXT("SAMP_RES"), Found->ResolutionM);
                        }
                    }
                    if (--Pinned->MetadataPending <= 0)
                    {
                        Finish(Pinned);
                    }
                });
    }
    if (StatePtr->MetadataPending <= 0)
    {
        Finish(StatePtr);
    }
}

// The tilemap answer for release R names the release whose image is shown there
// ("select"); all releases in between repeat it, so the next request goes to the
// release just before that one. "data": [0] means no older image exists.
void RequestTilemap(const FWeakState& Weak, int32 Generation, int32 Index)
{
    const FStatePtr StatePtr = Current(Weak, Generation);
    if (!StatePtr.IsValid() || !StatePtr->Releases.IsValidIndex(Index))
    {
        return;
    }
    const FString Url = FString::Printf(TEXT("%s/tilemap/%d/%d/%d/%d"), MapServerUrl, StatePtr->Releases[Index].Release, ChangeLevel,
                                        StatePtr->TileY, StatePtr->TileX);
    HttpGet(Url,
            [Weak, Generation, Index](bool bOk, int32 Code, const FString& Body)
            {
                const FStatePtr Pinned = Current(Weak, Generation);
                if (!Pinned.IsValid() || !Pinned->Releases.IsValidIndex(Index))
                {
                    return;
                }
                const TSharedPtr<FJsonObject> Object = bOk ? ParseObject(Body) : nullptr;
                const TArray<TSharedPtr<FJsonValue>>* Data = nullptr;
                if (!Object.IsValid() || !Object->TryGetArrayField(TEXT("data"), Data) || Data == nullptr)
                {
                    UE_LOG(LogDroneWayback, Warning, TEXT("Wayback: tilemap request failed (HTTP %d), %d images found so far"), Code,
                           Pinned->Versions.Num());
                    RequestMetadata(Weak, Generation);
                    return;
                }
                const bool bImage = Data->Num() > 0 && (*Data)[0].IsValid() && (*Data)[0]->AsNumber() > 0.0;
                if (!bImage)
                {
                    RequestMetadata(Weak, Generation);
                    return;
                }
                int32 Selected = Pinned->Releases[Index].Release;
                const TArray<TSharedPtr<FJsonValue>>* Select = nullptr;
                if (Object->TryGetArrayField(TEXT("select"), Select) && Select != nullptr && Select->Num() > 0 && (*Select)[0].IsValid())
                {
                    Selected = static_cast<int32>((*Select)[0]->AsNumber());
                }
                const int32 SelectedIndex = Pinned->Releases.IndexOfByPredicate(
                    [Selected](const FDroneWaybackRelease& Candidate) { return Candidate.Release == Selected; });
                if (SelectedIndex == INDEX_NONE)
                {
                    RequestMetadata(Weak, Generation);
                    return;
                }
                const bool bKnown = Pinned->Versions.ContainsByPredicate(
                    [Selected](const FDroneWaybackVersion& Candidate) { return Candidate.Release == Selected; });
                if (!bKnown)
                {
                    FDroneWaybackVersion Version;
                    Version.Release = Selected;
                    Version.ReleaseDate = Pinned->Releases[SelectedIndex].Date;
                    Pinned->Versions.Add(MoveTemp(Version));
                }
                Pinned->Status = FString::Printf(TEXT("Looking for the dates with different imagery here... %d found"),
                                                 Pinned->Versions.Num());
                const int32 Next = FMath::Max(SelectedIndex, Index) + 1;
                if (Next < Pinned->Releases.Num() && Pinned->Versions.Num() < MaxVersions)
                {
                    RequestTilemap(Weak, Generation, Next);
                }
                else
                {
                    RequestMetadata(Weak, Generation);
                }
            });
}

void StartSearch(const FWeakState& Weak, int32 Generation)
{
    const FStatePtr StatePtr = Current(Weak, Generation);
    if (!StatePtr.IsValid())
    {
        return;
    }
    StatePtr->Versions.Reset();
    StatePtr->Status = TEXT("Looking for the dates with different imagery here...");
    RequestTilemap(Weak, Generation, 0);
}

// The list of all releases: from the saved copy while it is fresh, else downloaded
// (an older saved copy is used when offline).
void LoadReleasesThenSearch(const FWeakState& Weak, int32 Generation)
{
    const FStatePtr StatePtr = Current(Weak, Generation);
    if (!StatePtr.IsValid())
    {
        return;
    }
    if (StatePtr->Releases.Num() > 0)
    {
        StartSearch(Weak, Generation);
        return;
    }
    const FString Path = ConfigFile(StatePtr->CacheDir);
    FString Saved;
    const bool bSaved = FFileHelper::LoadFileToString(Saved, *Path);
    if (bSaved && FileAgeDays(Path) < ConfigMaxAgeDays && ParseConfig(Saved, StatePtr->Releases))
    {
        StartSearch(Weak, Generation);
        return;
    }
    StatePtr->Status = TEXT("Loading the list of Wayback releases...");
    HttpGet(ConfigUrl,
            [Weak, Generation, Path, Saved, bSaved](bool bOk, int32 Code, const FString& Body)
            {
                const FStatePtr Pinned = Current(Weak, Generation);
                if (!Pinned.IsValid())
                {
                    return;
                }
                if (bOk && ParseConfig(Body, Pinned->Releases))
                {
                    IFileManager::Get().MakeDirectory(*FPaths::GetPath(Path), /*Tree*/ true);
                    FFileHelper::SaveStringToFile(Body, *Path);
                    UE_LOG(LogDroneWayback, Display, TEXT("Wayback: %d releases"), Pinned->Releases.Num());
                    StartSearch(Weak, Generation);
                    return;
                }
                if (bSaved && ParseConfig(Saved, Pinned->Releases))
                {
                    StartSearch(Weak, Generation);
                    return;
                }
                Pinned->Status = FString::Printf(TEXT("Cannot load the list of Wayback releases (HTTP %d; offline?)"), Code);
                Pinned->PlaceKey.Reset(); // asked again next time
                UE_LOG(LogDroneWayback, Warning, TEXT("%s"), *Pinned->Status);
            });
}
} // namespace DroneWaybackDetail

FString FDroneWaybackVersion::Label() const
{
    FString Text = CaptureDate.IsEmpty() ? FString::Printf(TEXT("release %s"), *ReleaseDate)
                                         : FString::Printf(TEXT("taken %s"), *CaptureDate);
    FString Source = Provider;
    if (!Sensor.IsEmpty() && Sensor != Provider)
    {
        Source += Source.IsEmpty() ? Sensor : TEXT(" ") + Sensor;
    }
    if (ResolutionM > 0.0)
    {
        Source += FString::Printf(TEXT("%s%.2g m"), Source.IsEmpty() ? TEXT("") : TEXT(" "), ResolutionM);
    }
    if (!Source.IsEmpty())
    {
        Text += TEXT(", ") + Source;
    }
    if (!CaptureDate.IsEmpty())
    {
        Text += FString::Printf(TEXT(" (release %s)"), *ReleaseDate);
    }
    return Text;
}

FDroneWayback::FDroneWayback()
    : State(MakeShared<FState, ESPMode::ThreadSafe>())
{
}

FDroneWayback::~FDroneWayback() = default;

void FDroneWayback::RequestVersions(double Lat, double Lon, const FString& CacheDir)
{
    int32 X = 0;
    int32 Y = 0;
    DroneWaybackDetail::TileOf(Lat, Lon, DroneWaybackDetail::ChangeLevel, X, Y);
    const FString Key = FString::Printf(TEXT("%d/%d"), X, Y);
    if (Key == State->PlaceKey && CacheDir == State->CacheDir)
    {
        return;
    }
    State->PlaceKey = Key;
    State->CacheDir = CacheDir;
    State->Lat = Lat;
    State->Lon = Lon;
    State->TileX = X;
    State->TileY = Y;
    ++State->Generation;
    State->Versions.Reset();
    State->MetadataPending = 0;
    const FString Path = DroneWaybackDetail::VersionsFile(CacheDir, X, Y);
    if (DroneWaybackDetail::FileAgeDays(Path) < DroneWaybackDetail::VersionsMaxAgeDays &&
        DroneWaybackDetail::LoadVersionsFile(Path, State->Versions))
    {
        State->Status = FString::Printf(TEXT("%d different images here (newest first, list saved earlier)"), State->Versions.Num());
        return;
    }
    DroneWaybackDetail::LoadReleasesThenSearch(State, State->Generation);
}

const TArray<FDroneWaybackVersion>& FDroneWayback::GetVersions() const
{
    return State->Versions;
}

FString FDroneWayback::GetStatus() const
{
    return State->Status;
}

FString FDroneWayback::TileUrlTemplate(int32 Release)
{
    // The itemURL pattern of waybackconfig.json ({level}/{row}/{col}), FString::Printf needs a literal.
    return FString::Printf(
        TEXT("https://wayback.maptiles.arcgis.com/arcgis/rest/services/World_Imagery/WMTS/1.0.0/default028mm/MapServer/tile/%d/{z}/{y}/{x}"),
        Release);
}
