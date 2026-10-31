// Imagery kept on disk: a loopback HTTP server (127.0.0.1) that a Cesium
// URL-template raster overlay asks for its tiles. A tile already on disk is
// served from there; any other tile is downloaded once from its source and
// kept (no expiry), so flying over an area again needs no internet for its
// imagery. Cesium's own disk cache cannot do that for the Bing imagery from
// Cesium ion: those tile addresses carry a key that changes every run.
#pragma once

#include "CoreMinimal.h"
#include "HttpRequestHandler.h" // HttpRouteHandle.h uses FHttpRequestHandler without including it
#include "HttpRouteHandle.h"

class IHttpRouter;

class FDroneTileStore
{
public:
    // This run's counters.
    struct FStats
    {
        int64 FromDisk = 0;
        int64 Downloaded = 0;
        int64 DownloadedBytes = 0;
        int64 Failed = 0;
        int64 NoImagery = 0; // "no imagery here" answered (Cesium uses the coarser tile there)
        int32 Pending = 0; // requests being answered now
    };

    FDroneTileStore();
    ~FDroneTileStore();
    FDroneTileStore(const FDroneTileStore&) = delete;
    FDroneTileStore& operator=(const FDroneTileStore&) = delete;

    // Listens on 127.0.0.1:Port (once; later calls only change the folder).
    // False if the port cannot be used.
    bool Start(int32 Port, const FString& RootDir);
    void Stop();
    bool IsRunning() const { return Route.IsValid(); }

    // Tiles of SourceId (also its folder name) come from UpstreamTemplate
    // ({z} {x} {y}, y = 0 at the north) when they are not on disk yet.
    void SetSource(const FString& SourceId, const FString& UpstreamTemplate);
    // TemplateUrl for a UCesiumUrlTemplateRasterOverlay (Web Mercator XYZ tiles).
    FString OverlayTemplateUrl(const FString& SourceId) const;

    FString GetRootDir() const;
    FStats GetStats() const;

    struct FState; // shared with the request callbacks (defined in the .cpp)

private:
    TSharedRef<FState, ESPMode::ThreadSafe> State;
    TSharedPtr<IHttpRouter> Router;
    FHttpRouteHandle Route;
    int32 ListenPort = 0;
};
