// Automatic performance recorder: from BeginPlay to EndPlay it writes
// Saved/Perf/<date-time>/ with
//   frames.csv   - every frame: frame / game / render / RHI / GPU times, the
//                  time of the drone's own Tick sections, GC, Cesium tile
//                  loading and the flight state (mode, speed, AGL);
//   system.csv   - 4 times a second: CPU (total, busiest core, this process),
//                  GPU (3D / copy / video-encode load, dedicated VRAM), RAM
//                  (used, available, hard page faults) and disk (busy %,
//                  read / write MB/s, queue, this process's I/O);
//   hitches.csv  - every frame over the hitch threshold with its context and
//                  the latest system sample;
//   summary.txt  - per flight mode: frame-time percentiles, hitch counts and
//                  what limited the slow frames.
// Tools/perf_report.py joins the files into a report after the run.
// The game thread only fills queues; a background thread samples the system
// and does all the file writing, so the monitor itself adds no disk stalls.
#pragma once

#include "CoreMinimal.h"
#include "Containers/Queue.h"
#include "HAL/CriticalSection.h"

class UWorld;
class FDronePerfSampler;
class FRunnableThread;

// Sections of ADroneRouteFlight::Tick timed separately.
enum class EDronePerfScope : uint8
{
    Input,         // menu actions, hotkeys
    View,          // ClaimView + UpdateCameraAim
    AroundCameras, // UpdateAroundCameras (preview / extra views)
    CesiumCameras, // PushCesiumCameras
    Markers,
    GroundTrace,
    Physics,       // AdvanceFpv / AdvanceManual / route
    Move,          // globe anchor + camera transform
    Telemetry,     // CSV + MAVLink + video frame capture
    Count
};

struct FDronePerfFrameContext
{
    const TCHAR* Mode = TEXT("");
    bool bMenuOpen = false;
    bool bCrashed = false;
    double SpeedMps = 0.0;
    double AglM = -1.0;
    double HeadingDeg = 0.0;
    double PitchDeg = 0.0;
    double RollDeg = 0.0;
    int32 VideoMode = 0; // 0 off, 1 record, 2 live
    int32 CesiumCameras = 0; // extra views Cesium selects tiles for (preload + around)
};

struct FDronePerfSystemSample
{
    double TimeS = 0.0;
    float CpuTotalPct = -1.f;
    float CpuMaxCorePct = -1.f;
    float ProcessCpuPct = -1.f; // of the whole machine
    float Gpu3DPct = -1.f;      // all processes
    float Gpu3DProcessPct = -1.f;
    float GpuCopyPct = -1.f;
    float GpuVideoEncodePct = -1.f;
    float VramDedicatedMB = -1.f;        // busiest adapter, all processes
    float VramProcessMB = -1.f;          // this process
    float RamUsedMB = -1.f;
    float RamAvailableMB = -1.f;
    float ProcessWorkingSetMB = -1.f;
    float HardFaultsPerS = -1.f; // Memory\Pages Input/sec
    float DiskBusyPct = -1.f;
    float DiskReadMBps = -1.f;
    float DiskWriteMBps = -1.f;
    float DiskQueue = -1.f;
    float ProcessIoReadMBps = -1.f;
    float ProcessIoWriteMBps = -1.f;
};

class FDronePerfMonitor
{
public:
    FDronePerfMonitor();
    ~FDronePerfMonitor();

    void Start(UWorld* World, double InHitchMs);
    void Stop();
    bool IsRunning() const { return bRunning; }
    const FString& GetDir() const { return Dir; }

    // Call first thing in Tick: closes the previous frame (its frame time and
    // engine thread times arrive one frame late anyway).
    void BeginFrame(UWorld* World, float DeltaSeconds, const FDronePerfFrameContext& Ctx);

    struct FScope
    {
        FScope(FDronePerfMonitor& InMon, EDronePerfScope InScope);
        ~FScope();
        FDronePerfMonitor& Mon;
        EDronePerfScope Scope;
        double StartS;
    };

    FDronePerfSystemSample LatestSystemSample() const;

private:
    friend class FDronePerfSampler;
    void OnPreGC();
    void OnPostGC();
    void WriteSummary();

    bool bRunning = false;
    FString Dir;
    double HitchMs = 50.0;
    double StartTimeS = 0.0;
    int64 FrameIndex = 0;
    double ScopeMs[(int32)EDronePerfScope::Count] = {};
    double GcStartS = 0.0;
    double GcMsThisFrame = 0.0;
    int32 GcCountThisFrame = 0;
    double FrameMsEma = 0.0;
    double LastFrameS = 0.0;
    double SinceTilesCheckS = 1e9;
    int32 TilesetCount = 0;
    float TilesLoadPct = 100.f; // lowest load progress over the tilesets
    FDelegateHandle PreGcHandle, PostGcHandle;

    // Per-mode statistics for summary.txt.
    struct FModeStats
    {
        TArray<float> FrameMs;
        int32 Hitches = 0;
        int32 HitchBound[5] = {}; // game, render, rhi, gpu, gc
        double SumGame = 0, SumRender = 0, SumGpu = 0;
        double HitchSumCpu = 0, HitchSumGpu3D = 0, HitchSumDisk = 0, HitchSumFaults = 0;
        int32 HitchSysSamples = 0;
        double Seconds = 0;
    };
    TMap<FString, FModeStats> Stats;

    // Lines for the writer thread.
    TQueue<FString, EQueueMode::Mpsc> FrameLines;
    TQueue<FString, EQueueMode::Mpsc> HitchLines;

    mutable FCriticalSection SampleLock;
    FDronePerfSystemSample Latest;

    FDronePerfSampler* Sampler = nullptr;
    FRunnableThread* Thread = nullptr;
};
