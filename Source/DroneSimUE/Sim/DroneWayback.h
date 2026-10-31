// Esri World Imagery Wayback: the archive of earlier World Imagery mosaics
// ("releases", one every few weeks since 2014). Most releases repeat the image
// of the previous one at a given place, so for one place this finds only the
// releases whose image there really differs, and when each was taken - the same
// way the Wayback app does it (@esri/wayback-core 1.1: a chain of "tilemap"
// requests, then a query of each release's metadata layer).
#pragma once

#include "CoreMinimal.h"

struct FDroneWaybackVersion
{
    int32 Release = 0;
    FString ReleaseDate; // when Esri published the mosaic (YYYY-MM-DD)
    FString CaptureDate; // when the image at the place was taken (YYYY-MM-DD); empty if unknown
    FString Provider;    // e.g. "Maxar"
    FString Sensor;      // e.g. "WV02"
    double ResolutionM = 0.0;

    // "taken 2015-10-25, Maxar WV02 0.5 m (release 2019-04-03)"
    FString Label() const;
};

class FDroneWayback
{
public:
    FDroneWayback();
    ~FDroneWayback();
    FDroneWayback(const FDroneWayback&) = delete;
    FDroneWayback& operator=(const FDroneWayback&) = delete;

    // Finds the different images at (Lat, Lon): asynchronous, once per place (a
    // ~1.6 km tile); the list is kept in CacheDir/wayback for later runs.
    void RequestVersions(double Lat, double Lon, const FString& CacheDir);
    // Newest first; grows while the search runs.
    const TArray<FDroneWaybackVersion>& GetVersions() const;
    FString GetStatus() const;

    // Tiles of one release for the tile store ({z} {x} {y}, y from the north).
    static FString TileUrlTemplate(int32 Release);

    struct FState; // shared with the request callbacks (defined in the .cpp)

private:
    TSharedRef<FState, ESPMode::ThreadSafe> State;
};
