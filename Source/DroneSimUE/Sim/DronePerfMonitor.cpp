#include "Sim/DronePerfMonitor.h"

#include <atomic>

#include "Cesium3DTileset.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "GenericPlatform/GenericPlatformMisc.h"
#include "HAL/FileManager.h"
#include "HAL/PlatformMemory.h"
#include "HAL/PlatformMisc.h"
#include "HAL/PlatformProcess.h"
#include "HAL/PlatformTime.h"
#include "HAL/Runnable.h"
#include "HAL/RunnableThread.h"
#include "Misc/DateTime.h"
#include "Misc/Paths.h"
#include "RHI.h"
#include "RenderCore.h"
#include "UObject/UObjectGlobals.h"

#if PLATFORM_WINDOWS
#include "Windows/WindowsHWrapper.h"
#include "Windows/AllowWindowsPlatformTypes.h"
#include <pdh.h>
#include <pdhmsg.h>
#include "Windows/HideWindowsPlatformTypes.h"
#pragma comment(lib, "pdh.lib")
#endif

DEFINE_LOG_CATEGORY_STATIC(LogDronePerf, Log, All);

namespace
{
const TCHAR* const ScopeNames[(int32)EDronePerfScope::Count] = {
    TEXT("tick_input"), TEXT("tick_view"), TEXT("tick_around_cams"), TEXT("tick_cesium_cams"), TEXT("tick_markers"),
    TEXT("tick_ground_trace"), TEXT("tick_physics"), TEXT("tick_move"), TEXT("tick_telemetry_video")};

const TCHAR* const BoundNames[5] = {TEXT("game thread (CPU)"), TEXT("render thread (CPU)"), TEXT("RHI thread (CPU)"),
                                    TEXT("GPU"), TEXT("garbage collection")};

FString F1(double V) { return FString::Printf(TEXT("%.1f"), V); }
FString F2(double V) { return FString::Printf(TEXT("%.2f"), V); }

void WriteUtf8(FArchive* Ar, const FString& Text)
{
    if (Ar == nullptr)
    {
        return;
    }
    FTCHARToUTF8 Utf8(*Text);
    Ar->Serialize(const_cast<ANSICHAR*>(Utf8.Get()), Utf8.Length());
}

float Percentile(TArray<float> Sorted, double P)
{
    if (Sorted.Num() == 0)
    {
        return 0.f;
    }
    Sorted.Sort();
    const int32 I = FMath::Clamp(FMath::RoundToInt(P * (Sorted.Num() - 1)), 0, Sorted.Num() - 1);
    return Sorted[I];
}
} // namespace

// ---------------------------------------------------------------------------
// Background thread: system counters + all file writing.
class FDronePerfSampler : public FRunnable
{
public:
    explicit FDronePerfSampler(FDronePerfMonitor& InMon) : Mon(InMon) {}

    virtual bool Init() override
    {
        IFileManager& FM = IFileManager::Get();
        FramesAr = FM.CreateFileWriter(*(Mon.Dir / TEXT("frames.csv")), FILEWRITE_AllowRead);
        SystemAr = FM.CreateFileWriter(*(Mon.Dir / TEXT("system.csv")), FILEWRITE_AllowRead);
        HitchesAr = FM.CreateFileWriter(*(Mon.Dir / TEXT("hitches.csv")), FILEWRITE_AllowRead);
        WriteUtf8(SystemAr, TEXT("t_s,cpu_total_pct,cpu_max_core_pct,process_cpu_pct,gpu_3d_pct,gpu_3d_process_pct,gpu_copy_pct,")
                            TEXT("gpu_video_encode_pct,vram_dedicated_mb,vram_process_mb,ram_used_mb,ram_available_mb,")
                            TEXT("process_working_set_mb,hard_faults_per_s,disk_busy_pct,disk_read_mbps,disk_write_mbps,")
                            TEXT("disk_queue,process_io_read_mbps,process_io_write_mbps\n"));
        OpenCounters();
        return true;
    }

    virtual uint32 Run() override
    {
        double SinceFlushS = 0.0;
        double LastS = FPlatformTime::Seconds();
        while (!bStop)
        {
            FPlatformProcess::Sleep(0.25f);
            const double Now = FPlatformTime::Seconds();
            const double Dt = Now - LastS;
            LastS = Now;
            Sample(Now, Dt);
            Drain();
            SinceFlushS += Dt;
            if (SinceFlushS >= 1.0)
            {
                SinceFlushS = 0.0;
                FlushAll(); // a freeze or crash keeps everything up to the last second
            }
        }
        Drain();
        FlushAll();
        return 0;
    }

    virtual void Stop() override { bStop = true; }

    virtual void Exit() override
    {
        CloseCounters();
        delete FramesAr;
        delete SystemAr;
        delete HitchesAr;
        FramesAr = SystemAr = HitchesAr = nullptr;
    }

private:
    void Drain()
    {
        FString Line;
        while (Mon.FrameLines.Dequeue(Line))
        {
            WriteUtf8(FramesAr, Line);
        }
        while (Mon.HitchLines.Dequeue(Line))
        {
            WriteUtf8(HitchesAr, Line);
        }
    }

    void FlushAll()
    {
        if (FramesAr) FramesAr->Flush();
        if (SystemAr) SystemAr->Flush();
        if (HitchesAr) HitchesAr->Flush();
    }

    void Sample(double Now, double Dt)
    {
        FDronePerfSystemSample S;
        S.TimeS = Now - Mon.StartTimeS;

        const FPlatformMemoryStats Mem = FPlatformMemory::GetStats();
        S.RamAvailableMB = float(Mem.AvailablePhysical / (1024.0 * 1024.0));
        S.RamUsedMB = float((Mem.TotalPhysical - Mem.AvailablePhysical) / (1024.0 * 1024.0));
        S.ProcessWorkingSetMB = float(Mem.UsedPhysical / (1024.0 * 1024.0));

#if PLATFORM_WINDOWS
        // This process: CPU share of the machine and I/O.
        FILETIME Create, ExitT, Kernel, User;
        if (::GetProcessTimes(::GetCurrentProcess(), &Create, &ExitT, &Kernel, &User))
        {
            const uint64 Cpu100ns = (uint64(Kernel.dwHighDateTime) << 32 | Kernel.dwLowDateTime) +
                                    (uint64(User.dwHighDateTime) << 32 | User.dwLowDateTime);
            if (PrevCpu100ns != 0 && Dt > 0.0)
            {
                const double Cores = double(FMath::Max(1, FPlatformMisc::NumberOfCoresIncludingHyperthreads()));
                S.ProcessCpuPct = float(100.0 * (double(Cpu100ns - PrevCpu100ns) * 1e-7) / (Dt * Cores));
            }
            PrevCpu100ns = Cpu100ns;
        }
        IO_COUNTERS Io;
        if (::GetProcessIoCounters(::GetCurrentProcess(), &Io))
        {
            if (bHaveIo && Dt > 0.0)
            {
                S.ProcessIoReadMBps = float(double(Io.ReadTransferCount - PrevIoRead) / (1024.0 * 1024.0) / Dt);
                S.ProcessIoWriteMBps = float(double(Io.WriteTransferCount - PrevIoWrite) / (1024.0 * 1024.0) / Dt);
            }
            PrevIoRead = Io.ReadTransferCount;
            PrevIoWrite = Io.WriteTransferCount;
            bHaveIo = true;
        }

        if (Query != nullptr && ::PdhCollectQueryData(Query) == ERROR_SUCCESS && bPrimed)
        {
            // CPU: "_Total" and the busiest core.
            ForEach(CpuCounter, [&](const FString& Name, double V) {
                if (Name == TEXT("_Total")) S.CpuTotalPct = float(V);
                else S.CpuMaxCorePct = FMath::Max<float>(S.CpuMaxCorePct, V);
            });
            S.Gpu3DPct = SumOf(Gpu3DCounter, nullptr);
            S.Gpu3DProcessPct = SumOf(Gpu3DCounter, *PidTag);
            S.GpuCopyPct = SumOf(GpuCopyCounter, nullptr);
            S.GpuVideoEncodePct = SumOf(GpuEncodeCounter, nullptr);
            for (float* P : {&S.Gpu3DPct, &S.Gpu3DProcessPct, &S.GpuCopyPct, &S.GpuVideoEncodePct})
            {
                if (*P >= 0.f) *P = FMath::Min(*P, 100.f);
            }
            ForEach(VramCounter, [&](const FString&, double V) { S.VramDedicatedMB = FMath::Max<float>(S.VramDedicatedMB, V / (1024.0 * 1024.0)); });
            S.VramProcessMB = SumOf(VramProcessCounter, *PidTag);
            if (S.VramProcessMB >= 0.f) S.VramProcessMB = float(S.VramProcessMB / (1024.0 * 1024.0));
            S.HardFaultsPerS = Single(FaultsCounter);
            // Busiest physical disk (the _Total idle time averages the disks).
            ForEach(DiskIdleCounter, [&](const FString& Name, double V) {
                if (Name != TEXT("_Total")) S.DiskBusyPct = FMath::Max<float>(S.DiskBusyPct, FMath::Clamp(100.0 - V, 0.0, 100.0));
            });
            const float Rd = Single(DiskReadCounter), Wr = Single(DiskWriteCounter);
            S.DiskReadMBps = Rd >= 0.f ? float(Rd / (1024.0 * 1024.0)) : -1.f;
            S.DiskWriteMBps = Wr >= 0.f ? float(Wr / (1024.0 * 1024.0)) : -1.f;
            S.DiskQueue = Single(DiskQueueCounter);
        }
        bPrimed = true; // rate counters need two collections
#endif

        {
            FScopeLock Lock(&Mon.SampleLock);
            Mon.Latest = S;
        }
        const FString Line = FString::Printf(
            TEXT("%.3f,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n"), S.TimeS, *F1(S.CpuTotalPct),
            *F1(S.CpuMaxCorePct), *F1(S.ProcessCpuPct), *F1(S.Gpu3DPct), *F1(S.Gpu3DProcessPct), *F1(S.GpuCopyPct),
            *F1(S.GpuVideoEncodePct), *F1(S.VramDedicatedMB), *F1(S.VramProcessMB), *F1(S.RamUsedMB), *F1(S.RamAvailableMB),
            *F1(S.ProcessWorkingSetMB), *F1(S.HardFaultsPerS), *F1(S.DiskBusyPct), *F2(S.DiskReadMBps), *F2(S.DiskWriteMBps),
            *F2(S.DiskQueue), *F2(S.ProcessIoReadMBps), *F2(S.ProcessIoWriteMBps));
        WriteUtf8(SystemAr, Line);
    }

#if PLATFORM_WINDOWS
    void OpenCounters()
    {
        PidTag = FString::Printf(TEXT("pid_%u_"), FPlatformProcess::GetCurrentProcessId());
        if (::PdhOpenQueryW(nullptr, 0, &Query) != ERROR_SUCCESS)
        {
            Query = nullptr;
            UE_LOG(LogDronePerf, Warning, TEXT("PDH unavailable: system counters will be empty."));
            return;
        }
        // English names work on any Windows display language.
        auto Add = [this](const TCHAR* Path) -> PDH_HCOUNTER {
            PDH_HCOUNTER C = nullptr;
            if (::PdhAddEnglishCounterW(Query, Path, 0, &C) != ERROR_SUCCESS)
            {
                UE_LOG(LogDronePerf, Warning, TEXT("Counter not available: %s"), Path);
                return nullptr;
            }
            return C;
        };
        CpuCounter = Add(TEXT("\\Processor(*)\\% Processor Time"));
        Gpu3DCounter = Add(TEXT("\\GPU Engine(*engtype_3D)\\Utilization Percentage"));
        GpuCopyCounter = Add(TEXT("\\GPU Engine(*engtype_Copy)\\Utilization Percentage"));
        GpuEncodeCounter = Add(TEXT("\\GPU Engine(*engtype_VideoEncode)\\Utilization Percentage"));
        VramCounter = Add(TEXT("\\GPU Adapter Memory(*)\\Dedicated Usage"));
        VramProcessCounter = Add(TEXT("\\GPU Process Memory(*)\\Dedicated Usage"));
        FaultsCounter = Add(TEXT("\\Memory\\Pages Input/sec"));
        DiskIdleCounter = Add(TEXT("\\PhysicalDisk(*)\\% Idle Time"));
        DiskReadCounter = Add(TEXT("\\PhysicalDisk(_Total)\\Disk Read Bytes/sec"));
        DiskWriteCounter = Add(TEXT("\\PhysicalDisk(_Total)\\Disk Write Bytes/sec"));
        DiskQueueCounter = Add(TEXT("\\PhysicalDisk(_Total)\\Current Disk Queue Length"));
        ::PdhCollectQueryData(Query);
    }

    void CloseCounters()
    {
        if (Query != nullptr)
        {
            ::PdhCloseQuery(Query);
            Query = nullptr;
        }
    }

    template <typename Fn>
    void ForEach(PDH_HCOUNTER Counter, Fn&& Visit)
    {
        if (Counter == nullptr)
        {
            return;
        }
        DWORD Bytes = 0, Count = 0;
        PDH_STATUS St = ::PdhGetFormattedCounterArrayW(Counter, PDH_FMT_DOUBLE | PDH_FMT_NOCAP100, &Bytes, &Count, nullptr);
        if (St != PDH_MORE_DATA || Bytes == 0)
        {
            return;
        }
        Buffer.SetNumUninitialized(Bytes);
        PDH_FMT_COUNTERVALUE_ITEM_W* Items = reinterpret_cast<PDH_FMT_COUNTERVALUE_ITEM_W*>(Buffer.GetData());
        if (::PdhGetFormattedCounterArrayW(Counter, PDH_FMT_DOUBLE | PDH_FMT_NOCAP100, &Bytes, &Count, Items) != ERROR_SUCCESS)
        {
            return;
        }
        for (DWORD I = 0; I < Count; ++I)
        {
            if (Items[I].FmtValue.CStatus == PDH_CSTATUS_VALID_DATA || Items[I].FmtValue.CStatus == PDH_CSTATUS_NEW_DATA)
            {
                Visit(FString(Items[I].szName), Items[I].FmtValue.doubleValue);
            }
        }
    }

    // Sum over the instances, optionally only those whose name contains Tag.
    float SumOf(PDH_HCOUNTER Counter, const TCHAR* Tag)
    {
        bool bAny = false;
        double Sum = 0.0;
        ForEach(Counter, [&](const FString& Name, double V) {
            if (Tag == nullptr || Name.Contains(Tag))
            {
                Sum += V;
                bAny = true;
            }
        });
        return bAny ? float(Sum) : (Counter != nullptr ? 0.f : -1.f);
    }

    float Single(PDH_HCOUNTER Counter)
    {
        if (Counter == nullptr)
        {
            return -1.f;
        }
        PDH_FMT_COUNTERVALUE V;
        if (::PdhGetFormattedCounterValue(Counter, PDH_FMT_DOUBLE | PDH_FMT_NOCAP100, nullptr, &V) != ERROR_SUCCESS)
        {
            return -1.f;
        }
        return float(V.doubleValue);
    }

    PDH_HQUERY Query = nullptr;
    PDH_HCOUNTER CpuCounter = nullptr, Gpu3DCounter = nullptr, GpuCopyCounter = nullptr, GpuEncodeCounter = nullptr;
    PDH_HCOUNTER VramCounter = nullptr, VramProcessCounter = nullptr, FaultsCounter = nullptr;
    PDH_HCOUNTER DiskIdleCounter = nullptr, DiskReadCounter = nullptr, DiskWriteCounter = nullptr, DiskQueueCounter = nullptr;
    TArray<uint8> Buffer;
    FString PidTag;
    uint64 PrevCpu100ns = 0;
    uint64 PrevIoRead = 0, PrevIoWrite = 0;
    bool bHaveIo = false;
#else
    void OpenCounters() {}
    void CloseCounters() {}
#endif

    FDronePerfMonitor& Mon;
    FArchive* FramesAr = nullptr;
    FArchive* SystemAr = nullptr;
    FArchive* HitchesAr = nullptr;
    bool bPrimed = false;
    std::atomic<bool> bStop{false};
};

// ---------------------------------------------------------------------------
FDronePerfMonitor::FDronePerfMonitor() = default;

FDronePerfMonitor::~FDronePerfMonitor() { Stop(); }

void FDronePerfMonitor::Start(UWorld* World, double InHitchMs)
{
    if (bRunning)
    {
        return;
    }
    HitchMs = FMath::Max(InHitchMs, 5.0);
    Dir = FPaths::ConvertRelativePathToFull(FPaths::ProjectSavedDir() / TEXT("Perf") /
                                            FDateTime::Now().ToString(TEXT("%Y-%m-%d_%H-%M-%S")));
    IFileManager::Get().MakeDirectory(*Dir, /*Tree*/ true);
    StartTimeS = FPlatformTime::Seconds();
    LastFrameS = 0.0;
    FrameMsEma = 0.0;
    FrameIndex = 0;
    Stats.Reset();
    FrameLines.Empty();
    HitchLines.Empty();

    FString Header = TEXT("t_s,frame,mode,menu,frame_ms,game_ms,render_ms,rhi_ms,gpu_ms,gc_ms,gc_count");
    for (const TCHAR* N : ScopeNames)
    {
        Header += TEXT(",");
        Header += N;
    }
    Header += TEXT(",tilesets,cesium_cameras,tiles_load_pct,speed_mps,agl_m,heading_deg,pitch_deg,roll_deg,video,crashed,hitch\n");
    FrameLines.Enqueue(Header);
    HitchLines.Enqueue(TEXT("t_s,frame,mode,frame_ms,bound,game_ms,render_ms,rhi_ms,gpu_ms,gc_ms,slowest_tick_section,")
                       TEXT("slowest_tick_ms,tiles_load_pct,speed_mps,agl_m,cpu_total_pct,cpu_max_core_pct,gpu_3d_pct,")
                       TEXT("vram_dedicated_mb,ram_available_mb,hard_faults_per_s,disk_busy_pct,disk_read_mbps,process_io_read_mbps\n"));

    PreGcHandle = FCoreUObjectDelegates::GetPreGarbageCollectDelegate().AddRaw(this, &FDronePerfMonitor::OnPreGC);
    PostGcHandle = FCoreUObjectDelegates::GetPostGarbageCollect().AddRaw(this, &FDronePerfMonitor::OnPostGC);

    Sampler = new FDronePerfSampler(*this);
    Thread = FRunnableThread::Create(Sampler, TEXT("DronePerfSampler"), 0, TPri_BelowNormal);
    bRunning = true;
    UE_LOG(LogDronePerf, Display, TEXT("Performance monitor: %s (hitch > %.0f ms)"), *Dir, HitchMs);
}

void FDronePerfMonitor::Stop()
{
    if (!bRunning)
    {
        return;
    }
    bRunning = false;
    FCoreUObjectDelegates::GetPreGarbageCollectDelegate().Remove(PreGcHandle);
    FCoreUObjectDelegates::GetPostGarbageCollect().Remove(PostGcHandle);
    if (Thread != nullptr)
    {
        Thread->Kill(/*bShouldWait*/ true); // calls Sampler->Stop(), drains and closes the files
        delete Thread;
        Thread = nullptr;
    }
    delete Sampler;
    Sampler = nullptr;
    WriteSummary();
    UE_LOG(LogDronePerf, Display, TEXT("Performance report written to %s"), *Dir);
}

void FDronePerfMonitor::OnPreGC() { GcStartS = FPlatformTime::Seconds(); }

void FDronePerfMonitor::OnPostGC()
{
    if (GcStartS > 0.0)
    {
        GcMsThisFrame += (FPlatformTime::Seconds() - GcStartS) * 1000.0;
        ++GcCountThisFrame;
        GcStartS = 0.0;
    }
}

FDronePerfSystemSample FDronePerfMonitor::LatestSystemSample() const
{
    FScopeLock Lock(&SampleLock);
    return Latest;
}

FDronePerfMonitor::FScope::FScope(FDronePerfMonitor& InMon, EDronePerfScope InScope)
    : Mon(InMon), Scope(InScope), StartS(InMon.bRunning ? FPlatformTime::Seconds() : 0.0)
{
}

FDronePerfMonitor::FScope::~FScope()
{
    if (Mon.bRunning && StartS > 0.0)
    {
        Mon.ScopeMs[(int32)Scope] += (FPlatformTime::Seconds() - StartS) * 1000.0;
    }
}

void FDronePerfMonitor::BeginFrame(UWorld* World, float DeltaSeconds, const FDronePerfFrameContext& Ctx)
{
    if (!bRunning)
    {
        return;
    }
    const double Now = FPlatformTime::Seconds();
    const double T = Now - StartTimeS;
    // Wall-clock time between ticks: DeltaSeconds is fixed when recording
    // (fixed frame rate) and clamped by the engine, so it hides real stalls.
    const double FrameMs = LastFrameS > 0.0 ? (Now - LastFrameS) * 1000.0 : DeltaSeconds * 1000.0;
    LastFrameS = Now;
    const double GameMs = FPlatformTime::ToMilliseconds(GGameThreadTime);
    const double RenderMs = FPlatformTime::ToMilliseconds(GRenderThreadTime);
    const double RhiMs = FPlatformTime::ToMilliseconds(GRHIThreadTime);
    const double GpuMs = FPlatformTime::ToMilliseconds(RHIGetGPUFrameCycles(0));

    SinceTilesCheckS += DeltaSeconds;
    if (SinceTilesCheckS >= 0.25 && World != nullptr)
    {
        SinceTilesCheckS = 0.0;
        TilesetCount = 0;
        TilesLoadPct = 100.f;
        for (TActorIterator<ACesium3DTileset> It(World); It; ++It)
        {
            ++TilesetCount;
            TilesLoadPct = FMath::Min(TilesLoadPct, It->GetLoadProgress());
        }
    }

    // Slowest section of the drone's own Tick.
    int32 Slowest = 0;
    for (int32 I = 1; I < (int32)EDronePerfScope::Count; ++I)
    {
        if (ScopeMs[I] > ScopeMs[Slowest]) Slowest = I;
    }

    const FString ModeKey = Ctx.bMenuOpen ? FString(TEXT("MENU")) : FString(Ctx.Mode);
    const bool bWarmup = T < 3.0; // level load / first shaders
    // Hitch: over the absolute threshold, or a spike well above the recent average.
    const bool bSpike = FrameMs > 25.0 && FrameMs > 2.5 * FrameMsEma;
    const bool bHitch = !bWarmup && (FrameMs >= HitchMs || bSpike);
    if (!bHitch)
    {
        FrameMsEma = FrameMsEma <= 0.0 ? FrameMs : FrameMsEma + (FrameMs - FrameMsEma) * 0.05;
    }

    FString Line = FString::Printf(TEXT("%.4f,%lld,%s,%d,%s,%s,%s,%s,%s,%s,%d"), T, FrameIndex, *ModeKey, Ctx.bMenuOpen ? 1 : 0,
                                   *F2(FrameMs), *F2(GameMs), *F2(RenderMs), *F2(RhiMs), *F2(GpuMs), *F2(GcMsThisFrame),
                                   GcCountThisFrame);
    for (double Ms : ScopeMs)
    {
        Line += TEXT(",");
        Line += F2(Ms);
    }
    Line += FString::Printf(TEXT(",%d,%d,%s,%s,%s,%s,%s,%s,%d,%d,%d\n"), TilesetCount, Ctx.CesiumCameras, *F1(TilesLoadPct), *F1(Ctx.SpeedMps),
                            *F1(Ctx.AglM), *F1(Ctx.HeadingDeg), *F1(Ctx.PitchDeg), *F1(Ctx.RollDeg), Ctx.VideoMode,
                            Ctx.bCrashed ? 1 : 0, bHitch ? 1 : 0);
    FrameLines.Enqueue(MoveTemp(Line));

    if (!bWarmup)
    {
        FModeStats& M = Stats.FindOrAdd(ModeKey);
        M.FrameMs.Add(float(FrameMs));
        M.Seconds += FrameMs / 1000.0;
        M.SumGame += GameMs;
        M.SumRender += RenderMs;
        M.SumGpu += GpuMs;
        if (bHitch)
        {
            const double Parts[5] = {GameMs - GcMsThisFrame, RenderMs, RhiMs, GpuMs, GcMsThisFrame};
            int32 Bound = 0;
            for (int32 I = 1; I < 5; ++I)
            {
                if (Parts[I] > Parts[Bound]) Bound = I;
            }
            ++M.Hitches;
            ++M.HitchBound[Bound];
            const FDronePerfSystemSample S = LatestSystemSample();
            if (S.CpuTotalPct >= 0.f)
            {
                M.HitchSumCpu += S.CpuTotalPct;
                M.HitchSumGpu3D += FMath::Max(S.Gpu3DPct, 0.f);
                M.HitchSumDisk += FMath::Max(S.DiskBusyPct, 0.f);
                M.HitchSumFaults += FMath::Max(S.HardFaultsPerS, 0.f);
                ++M.HitchSysSamples;
            }
            HitchLines.Enqueue(FString::Printf(
                TEXT("%.4f,%lld,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n"), T, FrameIndex, *ModeKey,
                *F2(FrameMs), BoundNames[Bound], *F2(GameMs), *F2(RenderMs), *F2(RhiMs), *F2(GpuMs), *F2(GcMsThisFrame),
                ScopeNames[Slowest], *F2(ScopeMs[Slowest]), *F1(TilesLoadPct), *F1(Ctx.SpeedMps), *F1(Ctx.AglM),
                *F1(S.CpuTotalPct), *F1(S.CpuMaxCorePct), *F1(S.Gpu3DPct), *F1(S.VramDedicatedMB), *F1(S.RamAvailableMB),
                *F1(S.HardFaultsPerS), *F1(S.DiskBusyPct), *F2(S.DiskReadMBps), *F2(S.ProcessIoReadMBps)));
        }
    }

    ++FrameIndex;
    for (double& Ms : ScopeMs)
    {
        Ms = 0.0;
    }
    GcMsThisFrame = 0.0;
    GcCountThisFrame = 0;
}

void FDronePerfMonitor::WriteSummary()
{
    FString Out;
    Out += FString::Printf(TEXT("DroneSimUE performance summary  (%s)\nHitch = frame >= %.0f ms or > 2.5x the recent average (first 3 s skipped).\n")
                           TEXT("Details: frames.csv, system.csv, hitches.csv; report: python Tools/perf_report.py \"%s\"\n\n"),
                           *FDateTime::Now().ToString(), HitchMs, *Dir);
    for (const TPair<FString, FModeStats>& P : Stats)
    {
        const FModeStats& M = P.Value;
        const int32 N = M.FrameMs.Num();
        if (N == 0)
        {
            continue;
        }
        int32 Over33 = 0;
        for (float V : M.FrameMs)
        {
            Over33 += V > 33.4f ? 1 : 0;
        }
        Out += FString::Printf(TEXT("== %s: %d frames, %.0f s, avg %.1f FPS\n"), *P.Key, N, M.Seconds, N / FMath::Max(M.Seconds, 1e-3));
        Out += FString::Printf(TEXT("   frame ms  p50 %.1f  p95 %.1f  p99 %.1f  max %.1f   frames > 33 ms: %.1f%%\n"),
                               Percentile(M.FrameMs, 0.5), Percentile(M.FrameMs, 0.95), Percentile(M.FrameMs, 0.99),
                               Percentile(M.FrameMs, 1.0), 100.0 * Over33 / N);
        Out += FString::Printf(TEXT("   avg game %.1f ms, render %.1f ms, GPU %.1f ms\n"), M.SumGame / N, M.SumRender / N, M.SumGpu / N);
        Out += FString::Printf(TEXT("   hitches: %d (%.1f per minute)\n"), M.Hitches, M.Hitches / FMath::Max(M.Seconds / 60.0, 1e-3));
        for (int32 I = 0; I < 5; ++I)
        {
            if (M.HitchBound[I] > 0)
            {
                Out += FString::Printf(TEXT("     limited by %-22s %d\n"), BoundNames[I], M.HitchBound[I]);
            }
        }
        if (M.HitchSysSamples > 0)
        {
            Out += FString::Printf(TEXT("   during hitches (avg): CPU %.0f%%, GPU 3D %.0f%%, disk busy %.0f%%, hard faults %.0f/s\n"),
                                   M.HitchSumCpu / M.HitchSysSamples, M.HitchSumGpu3D / M.HitchSysSamples,
                                   M.HitchSumDisk / M.HitchSysSamples, M.HitchSumFaults / M.HitchSysSamples);
        }
        Out += TEXT("\n");
    }
    {
        FArchive* Ar = IFileManager::Get().CreateFileWriter(*(Dir / TEXT("summary.txt")));
        WriteUtf8(Ar, Out);
        delete Ar;
    }
}
