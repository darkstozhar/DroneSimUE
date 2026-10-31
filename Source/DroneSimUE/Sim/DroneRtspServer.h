// Local RTSP server for the live video: starts mediamtx when nothing listens
// on the stream's port yet, and stops it again if it was started here.
#pragma once

#include "CoreMinimal.h"
#include "HAL/PlatformProcess.h"

class FDroneRtspServer
{
public:
    ~FDroneRtspServer();

    // Url = rtsp://host:port/path. Returns true when the port accepts TCP
    // connections, either because a server already runs there or because
    // ExePath (mediamtx.exe) was started and came up within TimeoutS.
    // A server is only started for a local host (127.0.0.1 / localhost).
    // mediamtx reads mediamtx.yml next to the exe; its log goes to ServerLogFile.
    bool EnsureRunning(const FString& Url, const FString& ExePath, const FString& ServerLogFile, double TimeoutS = 10.0);

    // Stops the server if EnsureRunning started it (a server that was already
    // running is left alone).
    void Stop();

private:
    FProcHandle Process;
    bool bStartedHere = false;
};
