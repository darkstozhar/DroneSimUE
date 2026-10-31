#include "Sim/DroneVideoOutput.h"

#include "Containers/Queue.h"
#include "Engine/TextureRenderTarget2D.h"
#include "HAL/PlatformProcess.h"
#include "HAL/PlatformTime.h"
#include "HAL/Runnable.h"
#include "HAL/RunnableThread.h"
#include "RHICommandList.h"
#include "RHIGPUReadback.h"
#include "RenderingThread.h"
#include "TextureResource.h"

#include <atomic>

DEFINE_LOG_CATEGORY_STATIC(LogDroneVideo, Log, All);

namespace
{
// GPU readbacks kept in flight before the render thread waits for the GPU.
constexpr int32 MaxInFlightReadbacks = 4;
} // namespace

// State shared by the game thread (submit/stop), the render thread
// (readbacks) and the writer thread (pipe to ffmpeg). Kept alive by shared
// pointers captured in render commands.
struct FDroneVideoShared
{
    int32 Width = 0;
    int32 Height = 0;

    // Render thread only.
    TArray<TUniquePtr<FRHIGPUTextureReadback>> Pending; // oldest first
    TArray<TUniquePtr<FRHIGPUTextureReadback>> Pool;

    // Render thread -> writer thread.
    TQueue<TArray<uint8>, EQueueMode::Spsc> Frames;
    std::atomic<int32> QueuedFrames{0};
    std::atomic<int64> WrittenFrames{0};
    std::atomic<bool> bStopRequested{false};
    std::atomic<bool> bBroken{false};

    void* StdinRead = nullptr;  // ffmpeg's stdin (inherited by the child)
    void* StdinWrite = nullptr; // ours
    void* ErrRead = nullptr;    // ours
    void* ErrWrite = nullptr;   // ffmpeg's stdout + stderr
    FProcHandle Process;
    FRunnableThread* Thread = nullptr;
    TUniquePtr<FRunnable> Writer;

    void RT_Drain(bool bForce)
    {
        while (Pending.Num() > 0)
        {
            FRHIGPUTextureReadback* Readback = Pending[0].Get();
            if (!bForce && !Readback->IsReady())
            {
                break; // keep the frame order
            }
            int32 RowPitchPixels = 0;
            const uint8* Src = static_cast<const uint8*>(Readback->Lock(RowPitchPixels));
            if (Src != nullptr)
            {
                if (RowPitchPixels >= Width)
                {
                    TArray<uint8> Frame;
                    Frame.SetNumUninitialized(Width * Height * 4);
                    for (int32 Y = 0; Y < Height; ++Y)
                    {
                        FMemory::Memcpy(Frame.GetData() + static_cast<int64>(Y) * Width * 4,
                                        Src + static_cast<int64>(Y) * RowPitchPixels * 4, Width * 4);
                    }
                    Frames.Enqueue(MoveTemp(Frame));
                    QueuedFrames.fetch_add(1);
                }
                Readback->Unlock();
            }
            Pool.Add(MoveTemp(Pending[0]));
            Pending.RemoveAt(0);
        }
    }
};

namespace
{
void PumpStderr(FDroneVideoShared& S)
{
    if (S.ErrRead == nullptr)
    {
        return;
    }
    const FString Text = FPlatformProcess::ReadPipe(S.ErrRead);
    if (Text.IsEmpty())
    {
        return;
    }
    TArray<FString> Lines;
    Text.ParseIntoArrayLines(Lines);
    for (const FString& Line : Lines)
    {
        UE_LOG(LogDroneVideo, Warning, TEXT("ffmpeg: %s"), *Line);
    }
}

class FDroneVideoWriter final : public FRunnable
{
public:
    explicit FDroneVideoWriter(FDroneVideoShared& InShared) : S(InShared) {}

    virtual uint32 Run() override
    {
        TArray<uint8> Frame;
        for (;;)
        {
            if (S.Frames.Dequeue(Frame))
            {
                S.QueuedFrames.fetch_sub(1);
                // Keep ffmpeg's stderr pipe drained: if it filled up, ffmpeg
                // would block on it while we block on its stdin.
                PumpStderr(S);
                if (!S.bBroken.load())
                {
                    if (WriteAll(Frame.GetData(), Frame.Num()))
                    {
                        S.WrittenFrames.fetch_add(1);
                    }
                    else
                    {
                        S.bBroken.store(true);
                        UE_LOG(LogDroneVideo, Error,
                               TEXT("Writing to ffmpeg failed (it has exited). See the ffmpeg messages above."));
                    }
                }
                continue;
            }
            PumpStderr(S);
            if (S.bStopRequested.load())
            {
                break;
            }
            FPlatformProcess::Sleep(0.002f);
        }
        return 0;
    }

private:
    bool WriteAll(const uint8* Data, int32 Size)
    {
        int32 Offset = 0;
        while (Offset < Size)
        {
            int32 Written = 0;
            if (!FPlatformProcess::WritePipe(S.StdinWrite, Data + Offset, Size - Offset, &Written) || Written <= 0)
            {
                return false;
            }
            Offset += Written;
        }
        return true;
    }

    FDroneVideoShared& S;
};

FString BuildFfmpegArgs(const FDroneVideoConfig& C)
{
    const FString Input = FString::Printf(
        TEXT("-hide_banner -loglevel warning -nostats -f rawvideo -pix_fmt bgra -s %dx%d -framerate %d -i -"), C.Width,
        C.Height, C.Fps);
    const bool bNvenc = C.Encoder.Contains(TEXT("nvenc"));
    if (C.Sink == EDroneVideoSink::Mp4File)
    {
        const FString Enc = bNvenc
            ? FString::Printf(TEXT("-c:v %s -preset p5 -rc vbr -cq 19 -b:v 0 -g %d"), *C.Encoder, C.Fps)
            : FString::Printf(TEXT("-c:v %s -preset veryfast -crf 18 -g %d"), *C.Encoder, C.Fps);
        return FString::Printf(TEXT("%s -an %s -pix_fmt yuv420p -movflags +faststart -y \"%s\""), *Input, *Enc,
                               *C.OutputPath);
    }
    const FString Enc = bNvenc
        ? FString::Printf(TEXT("-c:v %s -preset p1 -tune ll -zerolatency 1 -bf 0 -g %d -b:v 8M"), *C.Encoder, C.Fps)
        : FString::Printf(TEXT("-c:v %s -preset ultrafast -tune zerolatency -bf 0 -g %d -b:v 8M"), *C.Encoder, C.Fps);
    return FString::Printf(TEXT("%s -an %s -pix_fmt yuv420p -f rtsp -rtsp_transport tcp \"%s\""), *Input, *Enc,
                           *C.OutputPath);
}
} // namespace

FDroneVideoOutput::~FDroneVideoOutput()
{
    Stop();
}

bool FDroneVideoOutput::IsRunning() const
{
    return Shared.IsValid() && !Shared->bBroken.load();
}

bool FDroneVideoOutput::Start(const FDroneVideoConfig& InConfig)
{
    Stop();
    Config = InConfig;
    Submitted = 0;
    Dropped = 0;

    TSharedPtr<FDroneVideoShared, ESPMode::ThreadSafe> S = MakeShared<FDroneVideoShared, ESPMode::ThreadSafe>();
    S->Width = Config.Width;
    S->Height = Config.Height;

    if (!FPlatformProcess::CreatePipe(S->StdinRead, S->StdinWrite, /*bWritePipeLocal*/ true) ||
        !FPlatformProcess::CreatePipe(S->ErrRead, S->ErrWrite, /*bWritePipeLocal*/ false))
    {
        UE_LOG(LogDroneVideo, Error, TEXT("Could not create pipes for ffmpeg"));
        FPlatformProcess::ClosePipe(S->StdinRead, S->StdinWrite);
        FPlatformProcess::ClosePipe(S->ErrRead, S->ErrWrite);
        return false;
    }

    const FString Args = BuildFfmpegArgs(Config);
    // CreateProc must not get a quoted path.
    FString Exe = Config.FfmpegPath.TrimStartAndEnd().TrimQuotes();
    if (Exe.IsEmpty())
    {
        Exe = TEXT("ffmpeg");
    }
    S->Process = FPlatformProcess::CreateProc(*Exe, *Args, false, true, true, nullptr, 0, nullptr, S->ErrWrite,
                                              S->StdinRead, S->ErrWrite);
    if (!S->Process.IsValid())
    {
        UE_LOG(LogDroneVideo, Error,
               TEXT("Could not start ffmpeg ('%s'). Install ffmpeg and add it to PATH, or set Ffmpeg Path on the drone."),
               *Config.FfmpegPath);
        FPlatformProcess::ClosePipe(S->StdinRead, S->StdinWrite);
        FPlatformProcess::ClosePipe(S->ErrRead, S->ErrWrite);
        return false;
    }

    S->Writer = MakeUnique<FDroneVideoWriter>(*S);
    S->Thread = FRunnableThread::Create(S->Writer.Get(), TEXT("DroneVideoWriter"));
    Shared = S;
    UE_LOG(LogDroneVideo, Display, TEXT("Video %dx%d @ %d fps (%s) -> %s"), Config.Width, Config.Height, Config.Fps,
           *Config.Encoder, *Config.OutputPath);
    UE_LOG(LogDroneVideo, Log, TEXT("ffmpeg %s"), *Args);
    return true;
}

bool FDroneVideoOutput::SubmitFrame(UTextureRenderTarget2D* Target)
{
    if (!Shared.IsValid() || Target == nullptr || Shared->bBroken.load())
    {
        return false;
    }
    const int32 Limit = FMath::Max(Config.MaxQueuedFrames, 1);
    if (Shared->QueuedFrames.load() >= Limit)
    {
        if (Config.Sink == EDroneVideoSink::Rtsp)
        {
            ++Dropped; // live: keep up with real time
            return false;
        }
        // Recording: wait for the encoder instead of losing a frame.
        const double Deadline = FPlatformTime::Seconds() + 10.0;
        while (Shared->QueuedFrames.load() >= Limit && !Shared->bBroken.load() && FPlatformTime::Seconds() < Deadline)
        {
            FPlatformProcess::Sleep(0.001f);
        }
    }

    FTextureRenderTargetResource* Resource = Target->GameThread_GetRenderTargetResource();
    if (Resource == nullptr)
    {
        return false;
    }
    TSharedPtr<FDroneVideoShared, ESPMode::ThreadSafe> S = Shared;
    ENQUEUE_RENDER_COMMAND(DroneVideoReadback)(
        [S, Resource](FRHICommandListImmediate& RHICmdList)
        {
            S->RT_Drain(false);
            if (S->Pending.Num() >= MaxInFlightReadbacks)
            {
                RHICmdList.SubmitAndBlockUntilGPUIdle();
                S->RT_Drain(true);
            }
            FRHITexture* Texture = Resource->GetRenderTargetTexture().GetReference();
            if (Texture == nullptr)
            {
                return;
            }
            TUniquePtr<FRHIGPUTextureReadback> Readback =
                S->Pool.Num() > 0 ? S->Pool.Pop() : MakeUnique<FRHIGPUTextureReadback>(TEXT("DroneVideoReadback"));
            // EnqueueCopy expects the source in CopySrc; render targets rest in
            // SRVMask (same pattern as FRenderTarget::ReadFloat16Pixels).
            RHICmdList.Transition(FRHITransitionInfo(Texture, ERHIAccess::SRVMask, ERHIAccess::CopySrc));
            Readback->EnqueueCopy(RHICmdList, Texture);
            RHICmdList.Transition(FRHITransitionInfo(Texture, ERHIAccess::CopySrc, ERHIAccess::SRVMask));
            S->Pending.Add(MoveTemp(Readback));
        });
    ++Submitted;
    return true;
}

void FDroneVideoOutput::Stop()
{
    if (!Shared.IsValid())
    {
        return;
    }
    TSharedPtr<FDroneVideoShared, ESPMode::ThreadSafe> S = Shared;
    ENQUEUE_RENDER_COMMAND(DroneVideoFlush)(
        [S](FRHICommandListImmediate& RHICmdList)
        {
            if (S->Pending.Num() > 0)
            {
                RHICmdList.SubmitAndBlockUntilGPUIdle();
                S->RT_Drain(true);
            }
        });
    FlushRenderingCommands();

    S->bStopRequested.store(true);
    if (S->Thread != nullptr)
    {
        S->Thread->WaitForCompletion();
        delete S->Thread;
        S->Thread = nullptr;
    }

    // Closing ffmpeg's stdin is its end-of-input: it finishes encoding and
    // writes the mp4 index.
    FPlatformProcess::ClosePipe(S->StdinRead, S->StdinWrite);
    S->StdinRead = nullptr;
    S->StdinWrite = nullptr;
    const double Deadline = FPlatformTime::Seconds() + 20.0;
    while (FPlatformProcess::IsProcRunning(S->Process) && FPlatformTime::Seconds() < Deadline)
    {
        PumpStderr(*S);
        FPlatformProcess::Sleep(0.05f);
    }
    PumpStderr(*S);
    if (FPlatformProcess::IsProcRunning(S->Process))
    {
        UE_LOG(LogDroneVideo, Warning, TEXT("ffmpeg did not exit in time; terminating it."));
        FPlatformProcess::TerminateProc(S->Process, true);
    }
    int32 ReturnCode = 0;
    FPlatformProcess::GetProcReturnCode(S->Process, &ReturnCode);
    FPlatformProcess::CloseProc(S->Process);
    FPlatformProcess::ClosePipe(S->ErrRead, S->ErrWrite);
    S->ErrRead = nullptr;
    S->ErrWrite = nullptr;

    UE_LOG(LogDroneVideo, Display, TEXT("Video stopped: %lld frames submitted, %lld written, %lld dropped, ffmpeg exit code %d"),
           Submitted, S->WrittenFrames.load(), Dropped, ReturnCode);
    Shared.Reset();
}
