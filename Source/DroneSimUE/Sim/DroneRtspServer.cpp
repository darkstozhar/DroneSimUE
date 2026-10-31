#include "Sim/DroneRtspServer.h"

#include "HAL/PlatformMisc.h"
#include "HAL/PlatformTime.h"
#include "IPAddress.h"
#include "Misc/Paths.h"
#include "SocketSubsystem.h"
#include "Sockets.h"

DEFINE_LOG_CATEGORY_STATIC(LogDroneRtsp, Log, All);

namespace
{
// rtsp://host[:port][/path] -> host, port (554 if not given).
bool ParseRtspUrl(const FString& Url, FString& OutHost, int32& OutPort)
{
    FString Rest = Url.TrimStartAndEnd();
    if (!Rest.StartsWith(TEXT("rtsp://"), ESearchCase::IgnoreCase))
    {
        return false;
    }
    Rest.RightChopInline(7);
    int32 Slash = INDEX_NONE;
    if (Rest.FindChar(TEXT('/'), Slash))
    {
        Rest.LeftInline(Slash);
    }
    int32 At = INDEX_NONE;
    if (Rest.FindLastChar(TEXT('@'), At)) // user:pass@host
    {
        Rest.RightChopInline(At + 1);
    }
    OutPort = 554;
    int32 Colon = INDEX_NONE;
    if (Rest.FindLastChar(TEXT(':'), Colon))
    {
        OutPort = FCString::Atoi(*Rest.Mid(Colon + 1));
        Rest.LeftInline(Colon);
    }
    OutHost = Rest;
    return !OutHost.IsEmpty() && OutPort > 0 && OutPort < 65536;
}

// True if a TCP connection to Host:Port succeeds within TimeoutS.
bool IsTcpPortOpen(const FString& Host, int32 Port, double TimeoutS)
{
    ISocketSubsystem* Subsystem = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM);
    if (Subsystem == nullptr)
    {
        return false;
    }
    TSharedRef<FInternetAddr> Addr = Subsystem->CreateInternetAddr();
    bool bValidIp = false;
    Addr->SetIp(*Host, bValidIp);
    if (!bValidIp)
    {
        return false;
    }
    Addr->SetPort(Port);
    FSocket* Socket = Subsystem->CreateSocket(NAME_Stream, TEXT("DroneRtspProbe"), false);
    if (Socket == nullptr)
    {
        return false;
    }
    // Non-blocking connect: the socket becomes writable once connected; a
    // refused connection never does, so the wait just times out.
    Socket->SetNonBlocking(true);
    Socket->Connect(*Addr);
    const bool bConnected = Socket->Wait(ESocketWaitConditions::WaitForWrite, FTimespan::FromSeconds(TimeoutS)) &&
                            Socket->GetConnectionState() == SCS_Connected;
    Socket->Close();
    Subsystem->DestroySocket(Socket);
    return bConnected;
}
} // namespace

FDroneRtspServer::~FDroneRtspServer()
{
    Stop();
}

bool FDroneRtspServer::EnsureRunning(const FString& Url, const FString& ExePath, const FString& ServerLogFile, double TimeoutS)
{
    FString Host;
    int32 Port = 0;
    if (!ParseRtspUrl(Url, Host, Port))
    {
        UE_LOG(LogDroneRtsp, Error, TEXT("Not an rtsp:// URL: '%s'"), *Url);
        return false;
    }
    if (Host.Equals(TEXT("localhost"), ESearchCase::IgnoreCase))
    {
        Host = TEXT("127.0.0.1");
    }
    if (IsTcpPortOpen(Host, Port, 0.3))
    {
        UE_LOG(LogDroneRtsp, Display, TEXT("RTSP server already running on %s:%d"), *Host, Port);
        return true;
    }
    if (Host != TEXT("127.0.0.1"))
    {
        UE_LOG(LogDroneRtsp, Warning, TEXT("Nothing listens on %s:%d and it is not a local address; not starting a server."),
               *Host, Port);
        return false;
    }

    const FString Exe = ExePath.TrimStartAndEnd().TrimQuotes();
    if (Exe.IsEmpty())
    {
        return false;
    }
    if (!FPaths::FileExists(Exe))
    {
        UE_LOG(LogDroneRtsp, Error, TEXT("RTSP server not found: '%s' (set Rtsp Server Path on the drone)"), *Exe);
        return false;
    }
    const FString Dir = FPaths::GetPath(Exe);
    const FString Config = Dir / TEXT("mediamtx.yml");
    const FString Args = FPaths::FileExists(Config) ? FString::Printf(TEXT("\"%s\""), *Config) : FString();

    // mediamtx takes any setting from MTX_<NAME> environment variables; the
    // child inherits them. Without a console its stdout log would be lost.
    FPlatformMisc::SetEnvironmentVar(TEXT("MTX_LOGDESTINATIONS"), TEXT("file"));
    FPlatformMisc::SetEnvironmentVar(TEXT("MTX_LOGFILE"), *ServerLogFile);

    Stop();
    Process = FPlatformProcess::CreateProc(*Exe, *Args, /*bLaunchDetached*/ false, /*bLaunchHidden*/ true,
                                           /*bLaunchReallyHidden*/ true, nullptr, 0, *Dir, nullptr);
    if (!Process.IsValid())
    {
        UE_LOG(LogDroneRtsp, Error, TEXT("Could not start '%s'"), *Exe);
        return false;
    }
    bStartedHere = true;

    const double Deadline = FPlatformTime::Seconds() + TimeoutS;
    while (FPlatformTime::Seconds() < Deadline)
    {
        if (!FPlatformProcess::IsProcRunning(Process))
        {
            int32 Code = 0;
            FPlatformProcess::GetProcReturnCode(Process, &Code);
            UE_LOG(LogDroneRtsp, Error, TEXT("mediamtx exited with code %d; see %s"), Code, *ServerLogFile);
            Stop();
            return false;
        }
        if (IsTcpPortOpen(Host, Port, 0.25))
        {
            UE_LOG(LogDroneRtsp, Display, TEXT("Started %s (RTSP %s:%d, log %s)"), *Exe, *Host, Port, *ServerLogFile);
            return true;
        }
    }
    UE_LOG(LogDroneRtsp, Error, TEXT("mediamtx did not open %s:%d within %.0f s; see %s"), *Host, Port, TimeoutS, *ServerLogFile);
    Stop();
    return false;
}

void FDroneRtspServer::Stop()
{
    if (bStartedHere && Process.IsValid())
    {
        if (FPlatformProcess::IsProcRunning(Process))
        {
            FPlatformProcess::TerminateProc(Process, /*KillTree*/ true);
        }
        FPlatformProcess::CloseProc(Process);
        UE_LOG(LogDroneRtsp, Display, TEXT("RTSP server stopped"));
    }
    Process = FProcHandle();
    bStartedHere = false;
}
