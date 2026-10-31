#include "Sim/DroneTelemetryOutput.h"

#include "HAL/FileManager.h"
#include "IPAddress.h"
#include "Misc/Paths.h"
#include "SocketSubsystem.h"
#include "Sockets.h"
#include "Sim/MavlinkLite.h"

DEFINE_LOG_CATEGORY_STATIC(LogDroneTelemetry, Log, All);

namespace
{
constexpr uint8 SysId = 1;
constexpr uint8 CompId = 1; // MAV_COMP_ID_AUTOPILOT1

int16 ToCmPerS(double Mps)
{
    return static_cast<int16>(FMath::Clamp(FMath::RoundToDouble(Mps * 100.0), -32767.0, 32767.0));
}

double WrapPi(double Rad)
{
    return FMath::UnwindRadians(Rad);
}
} // namespace

FDroneTelemetryOutput::~FDroneTelemetryOutput()
{
    Close();
}

bool FDroneTelemetryOutput::OpenMavlink(const FString& Host, int32 Port)
{
    ISocketSubsystem* Subsystem = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM);
    if (Subsystem == nullptr)
    {
        UE_LOG(LogDroneTelemetry, Error, TEXT("No socket subsystem; MAVLink output disabled"));
        return false;
    }
    Destination = Subsystem->CreateInternetAddr();
    bool bValidIp = false;
    Destination->SetIp(*Host, bValidIp);
    Destination->SetPort(Port);
    if (!bValidIp)
    {
        UE_LOG(LogDroneTelemetry, Error, TEXT("Invalid MAVLink host '%s'; MAVLink output disabled"), *Host);
        Destination.Reset();
        return false;
    }
    Socket = Subsystem->CreateSocket(NAME_DGram, TEXT("DroneSimMavlink"), false);
    if (Socket == nullptr)
    {
        UE_LOG(LogDroneTelemetry, Error, TEXT("Could not create the MAVLink UDP socket"));
        Destination.Reset();
        return false;
    }
    UE_LOG(LogDroneTelemetry, Display, TEXT("MAVLink 2 telemetry -> udp://%s:%d"), *Host, Port);
    return true;
}

bool FDroneTelemetryOutput::OpenCsv(const FString& Path)
{
    IFileManager::Get().MakeDirectory(*FPaths::GetPath(Path), true);
    Csv = IFileManager::Get().CreateFileWriter(*Path);
    if (Csv == nullptr)
    {
        UE_LOG(LogDroneTelemetry, Error, TEXT("Could not open %s"), *Path);
        return false;
    }
    CsvPath = Path;
    const FTCHARToUTF8 Header(
        TEXT("frame,time_s,unix_time_s,lat,lon,alt_ellipsoid_m,alt_msl_m,alt_rel_m,alt_agl_m,")
        TEXT("heading_deg,pitch_deg,roll_deg,vn_mps,ve_mps,vd_mps,camera_pitch_deg,hfov_deg,camera_yaw_deg\n"));
    Csv->Serialize(const_cast<void*>(static_cast<const void*>(Header.Get())), Header.Length());
    UE_LOG(LogDroneTelemetry, Display, TEXT("Ground-truth telemetry -> %s"), *Path);
    return true;
}

bool FDroneTelemetryOutput::OpenFramesCsv(const FString& Path)
{
    CloseFramesCsv();
    IFileManager::Get().MakeDirectory(*FPaths::GetPath(Path), true);
    FramesCsv = IFileManager::Get().CreateFileWriter(*Path);
    if (FramesCsv == nullptr)
    {
        UE_LOG(LogDroneTelemetry, Error, TEXT("Could not open %s"), *Path);
        return false;
    }
    // time_s is the video time of the frame (frame / fps), so this file and
    // the video can be paired directly (CsvFlightData "generic" preset).
    const FTCHARToUTF8 Header(
        TEXT("video_frame,time_s,sim_time_s,unix_time_s,lat,lon,alt_ellipsoid_m,alt_msl_m,alt_rel_m,alt_agl_m,")
        TEXT("heading_deg,pitch_deg,roll_deg,camera_pitch_deg,hfov_deg,camera_yaw_deg\n"));
    FramesCsv->Serialize(const_cast<void*>(static_cast<const void*>(Header.Get())), Header.Length());
    UE_LOG(LogDroneTelemetry, Display, TEXT("Per-video-frame ground truth -> %s"), *Path);
    return true;
}

void FDroneTelemetryOutput::WriteFrameRow(const FDroneTelemetrySample& S, int64 VideoFrame, double VideoTimeS)
{
    if (FramesCsv == nullptr)
    {
        return;
    }
    const FString Agl = S.AglM >= 0.0 ? FString::Printf(TEXT("%.3f"), S.AglM) : FString();
    const FString Line = FString::Printf(
        TEXT("%lld,%.6f,%.6f,%.6f,%.9f,%.9f,%.3f,%.3f,%.3f,%s,%.4f,%.4f,%.4f,%.3f,%.4f,%.3f\n"), VideoFrame, VideoTimeS,
        S.TimeS, S.UnixTimeS, S.Latitude, S.Longitude, S.EllipsoidHeightM, S.MslHeightM, S.RelativeAltM, *Agl,
        S.HeadingDeg, S.PitchDeg, S.RollDeg, S.CameraPitchDeg, S.CameraHFovDeg, S.CameraYawDeg);
    const FTCHARToUTF8 Utf8(*Line);
    FramesCsv->Serialize(const_cast<void*>(static_cast<const void*>(Utf8.Get())), Utf8.Length());
    if (++FrameRowsSinceFlush >= 30)
    {
        FramesCsv->Flush();
        FrameRowsSinceFlush = 0;
    }
}

void FDroneTelemetryOutput::CloseFramesCsv()
{
    if (FramesCsv != nullptr)
    {
        FramesCsv->Flush();
        FramesCsv->Close();
        delete FramesCsv;
        FramesCsv = nullptr;
    }
    FrameRowsSinceFlush = 0;
}

bool FDroneTelemetryOutput::OpenGroundCsv(const FString& Path)
{
    CloseGroundCsv();
    IFileManager::Get().MakeDirectory(*FPaths::GetPath(Path), true);
    GroundCsv = IFileManager::Get().CreateFileWriter(*Path);
    if (GroundCsv == nullptr)
    {
        UE_LOG(LogDroneTelemetry, Warning, TEXT("Cannot create %s"), *Path);
        return false;
    }
    // Long format: one row per (video frame, pixel). hit=0 rows keep the
    // pixel so the exporter can tell "ray missed" from "row missing".
    const FTCHARToUTF8 Header(TEXT("video_frame,point,u_px,v_px,hit,lat,lon,h_ellipsoid_m,range_m\n"));
    GroundCsv->Serialize(const_cast<void*>(static_cast<const void*>(Header.Get())), Header.Length());
    UE_LOG(LogDroneTelemetry, Display, TEXT("Image-ray ground points -> %s"), *Path);
    return true;
}

void FDroneTelemetryOutput::WriteGroundRow(int64 VideoFrame, int32 Point, double U, double V, bool bHit,
                                           double Latitude, double Longitude, double EllipsoidHeightM, double RangeM)
{
    if (GroundCsv == nullptr)
    {
        return;
    }
    const FString Line = bHit ? FString::Printf(TEXT("%lld,%d,%.3f,%.3f,1,%.9f,%.9f,%.3f,%.3f\n"), VideoFrame, Point, U,
                                                V, Latitude, Longitude, EllipsoidHeightM, RangeM)
                              : FString::Printf(TEXT("%lld,%d,%.3f,%.3f,0,,,,\n"), VideoFrame, Point, U, V);
    const FTCHARToUTF8 Utf8(*Line);
    GroundCsv->Serialize(const_cast<void*>(static_cast<const void*>(Utf8.Get())), Utf8.Length());
    if (++GroundRowsSinceFlush >= 270)
    {
        GroundCsv->Flush();
        GroundRowsSinceFlush = 0;
    }
}

void FDroneTelemetryOutput::CloseGroundCsv()
{
    if (GroundCsv != nullptr)
    {
        GroundCsv->Flush();
        GroundCsv->Close();
        delete GroundCsv;
        GroundCsv = nullptr;
    }
    GroundRowsSinceFlush = 0;
}

void FDroneTelemetryOutput::Close()
{
    CloseFramesCsv();
    CloseGroundCsv();
    if (Csv != nullptr)
    {
        Csv->Flush();
        Csv->Close();
        delete Csv;
        Csv = nullptr;
    }
    if (Socket != nullptr)
    {
        Socket->Close();
        if (ISocketSubsystem* Subsystem = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM))
        {
            Subsystem->DestroySocket(Socket);
        }
        Socket = nullptr;
    }
    Destination.Reset();
}

void FDroneTelemetryOutput::WriteCsv(const FDroneTelemetrySample& S)
{
    if (Csv == nullptr)
    {
        return;
    }
    // An empty alt_agl_m cell is read as NaN by CsvFlightData.
    const FString Agl = S.AglM >= 0.0 ? FString::Printf(TEXT("%.3f"), S.AglM) : FString();
    const FString Line = FString::Printf(
        TEXT("%lld,%.6f,%.6f,%.9f,%.9f,%.3f,%.3f,%.3f,%s,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.3f,%.4f,%.3f\n"),
        S.Frame, S.TimeS, S.UnixTimeS, S.Latitude, S.Longitude, S.EllipsoidHeightM, S.MslHeightM,
        S.RelativeAltM, *Agl, S.HeadingDeg, S.PitchDeg, S.RollDeg, S.VelNorthMps, S.VelEastMps,
        S.VelDownMps, S.CameraPitchDeg, S.CameraHFovDeg, S.CameraYawDeg);
    const FTCHARToUTF8 Utf8(*Line);
    Csv->Serialize(const_cast<void*>(static_cast<const void*>(Utf8.Get())), Utf8.Length());
    if (++RowsSinceFlush >= 30)
    {
        Csv->Flush();
        RowsSinceFlush = 0;
    }
}

void FDroneTelemetryOutput::SendFrame(const uint8* Data, int32 Size)
{
    int32 Sent = 0;
    Socket->SendTo(Data, Size, Sent, *Destination);
}

void FDroneTelemetryOutput::SendMavlink(const FDroneTelemetrySample& S, bool bWithHeartbeat)
{
    if (Socket == nullptr || !Destination.IsValid())
    {
        return;
    }
    uint8 Frame[MavlinkLite::MaxFrameBytes];
    const uint32 BootMs = static_cast<uint32>(FMath::Max(0.0, S.TimeS) * 1000.0);

    if (bWithHeartbeat)
    {
        SendFrame(Frame, static_cast<int32>(MavlinkLite::Heartbeat(Frame, Sequence++, SysId, CompId)));
        const uint64 UnixUsec = static_cast<uint64>(S.UnixTimeS * 1.0e6);
        SendFrame(Frame, static_cast<int32>(MavlinkLite::SystemTime(Frame, Sequence++, SysId, CompId, UnixUsec, BootMs)));
    }

    const double YawRad = WrapPi(FMath::DegreesToRadians(S.HeadingDeg));
    SendFrame(Frame, static_cast<int32>(MavlinkLite::Attitude(
                         Frame, Sequence++, SysId, CompId, BootMs, static_cast<float>(FMath::DegreesToRadians(S.RollDeg)),
                         static_cast<float>(FMath::DegreesToRadians(S.PitchDeg)), static_cast<float>(YawRad), 0.f, 0.f, 0.f)));

    const int32 LatE7 = static_cast<int32>(FMath::RoundToDouble(S.Latitude * 1.0e7));
    const int32 LonE7 = static_cast<int32>(FMath::RoundToDouble(S.Longitude * 1.0e7));
    const int32 AltMm = static_cast<int32>(FMath::RoundToDouble(S.MslHeightM * 1000.0));
    const int32 RelMm = static_cast<int32>(FMath::RoundToDouble(S.RelativeAltM * 1000.0));
    double HeadingWrapped = FMath::Fmod(S.HeadingDeg, 360.0);
    if (HeadingWrapped < 0.0)
    {
        HeadingWrapped += 360.0;
    }
    const uint16 HdgCdeg = static_cast<uint16>(FMath::Clamp(FMath::RoundToDouble(HeadingWrapped * 100.0), 0.0, 35999.0));
    SendFrame(Frame, static_cast<int32>(MavlinkLite::GlobalPositionInt(
                         Frame, Sequence++, SysId, CompId, BootMs, LatE7, LonE7, AltMm, RelMm, ToCmPerS(S.VelNorthMps),
                         ToCmPerS(S.VelEastMps), ToCmPerS(S.VelDownMps), HdgCdeg)));

    if (S.AglM >= 0.0)
    {
        // Height above the terrain, the way ArduPilot reports it.
        SendFrame(Frame, static_cast<int32>(MavlinkLite::TerrainReport(
                             Frame, Sequence++, SysId, CompId, LatE7, LonE7, static_cast<float>(S.MslHeightM - S.AglM),
                             static_cast<float>(S.AglM))));
    }

    const double GroundSpeed = FMath::Sqrt(S.VelNorthMps * S.VelNorthMps + S.VelEastMps * S.VelEastMps);
    SendFrame(Frame, static_cast<int32>(MavlinkLite::VfrHud(
                         Frame, Sequence++, SysId, CompId, static_cast<float>(GroundSpeed), static_cast<float>(GroundSpeed),
                         static_cast<float>(S.MslHeightM), static_cast<float>(-S.VelDownMps),
                         static_cast<int16>(static_cast<int32>(FMath::RoundToDouble(HeadingWrapped)) % 360), 50)));
}
