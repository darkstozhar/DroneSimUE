// Telemetry sinks of the simulated drone: MAVLink 2 over UDP (like a real
// flight controller) and a per-frame CSV log (ground truth). The CSV header
// contains the columns DroneLocalization's CsvFlightData "generic" preset
// reads: time_s, lat, lon, alt_msl_m, alt_agl_m, heading_deg, pitch_deg, roll_deg.
#pragma once

#include "CoreMinimal.h"

class FSocket;
class FInternetAddr;
class FArchive;

struct FDroneTelemetrySample
{
    int64 Frame = 0;
    double TimeS = 0.0;     // simulation time since the route started (sum of frame times)
    double UnixTimeS = 0.0; // wall clock, UTC
    double Latitude = 0.0;
    double Longitude = 0.0;
    double EllipsoidHeightM = 0.0; // WGS84 ellipsoid (Cesium's height reference)
    double MslHeightM = 0.0;       // ellipsoid height minus the configured geoid undulation
    double RelativeAltM = 0.0;     // above the start point's ground
    double AglM = -1.0;            // above terrain under the drone; < 0 = unknown
    double HeadingDeg = 0.0;       // compass, clockwise from north (direction of the image top)
    double PitchDeg = 0.0;         // body, nose up positive
    double RollDeg = 0.0;          // body, right wing down positive
    double VelNorthMps = 0.0;
    double VelEastMps = 0.0;
    double VelDownMps = 0.0;
    double CameraPitchDeg = -90.0; // relative to the body, -90 = nadir
    double CameraYawDeg = 0.0;     // pan relative to the nose, + = to the right
    double CameraHFovDeg = 0.0;
};

class FDroneTelemetryOutput
{
public:
    ~FDroneTelemetryOutput();

    bool OpenMavlink(const FString& Host, int32 Port);
    bool OpenCsv(const FString& Path);
    void Close();

    void WriteCsv(const FDroneTelemetrySample& Sample);
    void SendMavlink(const FDroneTelemetrySample& Sample, bool bWithHeartbeat);

    // One row per video frame (frames.csv next to flight.mp4).
    bool OpenFramesCsv(const FString& Path);
    void WriteFrameRow(const FDroneTelemetrySample& Sample, int64 VideoFrame, double VideoTimeS);
    void CloseFramesCsv();

    // Image-ray ground hits (ground_points.csv next to flight.mp4): for every
    // video frame, rays through fixed pixels intersected with the rendered
    // Cesium terrain. DroneLocalization's anchors and per-frame ground truth
    // are built from these exact-geometry points (Tools/export_dronelocalization.py).
    bool OpenGroundCsv(const FString& Path);
    void WriteGroundRow(int64 VideoFrame, int32 Point, double U, double V, bool bHit, double Latitude,
                        double Longitude, double EllipsoidHeightM, double RangeM);
    void CloseGroundCsv();

    const FString& GetCsvPath() const { return CsvPath; }

private:
    void SendFrame(const uint8* Data, int32 Size);

    FSocket* Socket = nullptr;
    TSharedPtr<FInternetAddr> Destination;
    FArchive* Csv = nullptr;
    FString CsvPath;
    int32 RowsSinceFlush = 0;
    FArchive* FramesCsv = nullptr;
    int32 FrameRowsSinceFlush = 0;
    FArchive* GroundCsv = nullptr;
    int32 GroundRowsSinceFlush = 0;
    uint8 Sequence = 0;
};
