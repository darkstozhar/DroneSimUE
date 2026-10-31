// Video output of the drone camera: frames rendered into a render target are
// read back from the GPU asynchronously and piped as raw BGRA into an ffmpeg
// process, which writes an mp4 file or publishes an RTSP stream.
#pragma once

#include "CoreMinimal.h"

class UTextureRenderTarget2D;

enum class EDroneVideoSink : uint8
{
    Mp4File, // OutputPath = file path (.mp4)
    Rtsp,    // OutputPath = rtsp://host:port/path (needs an RTSP server, e.g. mediamtx)
};

struct FDroneVideoConfig
{
    EDroneVideoSink Sink = EDroneVideoSink::Mp4File;
    FString FfmpegPath = TEXT("ffmpeg"); // name in PATH or full path to ffmpeg.exe
    FString Encoder = TEXT("h264_nvenc"); // or libx264 (CPU)
    FString OutputPath;
    int32 Width = 1280;
    int32 Height = 720;
    int32 Fps = 30;
    // Live: frames beyond this backlog are dropped. Record: the game thread
    // waits instead, so no frame is lost.
    int32 MaxQueuedFrames = 8;
};

class FDroneVideoOutput
{
public:
    ~FDroneVideoOutput();

    bool Start(const FDroneVideoConfig& Config);

    // Game thread, right after SceneCapture->CaptureScene() into Target.
    // Returns false if the frame was dropped (live backlog) or output is broken.
    bool SubmitFrame(UTextureRenderTarget2D* Target);

    // Sends every pending frame, closes ffmpeg's input and waits for it to
    // finish (the mp4 is only valid after this).
    void Stop();

    bool IsRunning() const;
    int64 GetSubmittedFrames() const { return Submitted; }
    int64 GetDroppedFrames() const { return Dropped; }

private:
    TSharedPtr<struct FDroneVideoShared, ESPMode::ThreadSafe> Shared;
    FDroneVideoConfig Config;
    int64 Submitted = 0;
    int64 Dropped = 0;
};
