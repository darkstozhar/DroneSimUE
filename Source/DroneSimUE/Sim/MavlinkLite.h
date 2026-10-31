// Minimal MAVLink 2 encoder for the few messages the simulator sends.
// Plain C++ (no Unreal types) so it can be unit-tested outside the engine.
// Field order, message ids and CRC_EXTRA values follow the MAVLink "common"
// dialect (checked against pymavlink 2.4: HEARTBEAT 0/50, SYSTEM_TIME 2/137,
// ATTITUDE 30/39, GLOBAL_POSITION_INT 33/104, VFR_HUD 74/20, TERRAIN_REPORT 136/1).
// Assumes a little-endian host (x86-64 and ARM64 Windows/Android are).
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace MavlinkLite
{
constexpr size_t MaxFrameBytes = 12 + 255; // header 10 + payload + crc 2

inline void CrcAccumulate(uint8_t Byte, uint16_t& Crc)
{
    uint8_t Tmp = static_cast<uint8_t>(Byte ^ static_cast<uint8_t>(Crc & 0xFF));
    Tmp = static_cast<uint8_t>(Tmp ^ static_cast<uint8_t>(Tmp << 4));
    Crc = static_cast<uint16_t>((Crc >> 8) ^ (static_cast<uint16_t>(Tmp) << 8) ^
                                (static_cast<uint16_t>(Tmp) << 3) ^ (static_cast<uint16_t>(Tmp) >> 4));
}

// Little-endian payload builder.
class FPayload
{
public:
    template <typename T>
    void Put(T Value)
    {
        std::memcpy(Bytes + Size, &Value, sizeof(T));
        Size += sizeof(T);
    }
    uint8_t Bytes[255] = {};
    size_t Size = 0;
};

// Wraps a payload into a MAVLink 2 frame. Trailing zero bytes of the payload
// are truncated as the MAVLink 2 spec requires (at least one byte is kept).
// Returns the frame length written to Out (Out must hold MaxFrameBytes).
inline size_t Finalize(uint8_t* Out, uint32_t MsgId, uint8_t CrcExtra, const FPayload& Payload,
                       uint8_t Seq, uint8_t SysId, uint8_t CompId)
{
    size_t Len = Payload.Size;
    while (Len > 1 && Payload.Bytes[Len - 1] == 0)
    {
        --Len;
    }
    Out[0] = 0xFD;
    Out[1] = static_cast<uint8_t>(Len);
    Out[2] = 0; // incompat_flags
    Out[3] = 0; // compat_flags
    Out[4] = Seq;
    Out[5] = SysId;
    Out[6] = CompId;
    Out[7] = static_cast<uint8_t>(MsgId & 0xFF);
    Out[8] = static_cast<uint8_t>((MsgId >> 8) & 0xFF);
    Out[9] = static_cast<uint8_t>((MsgId >> 16) & 0xFF);
    std::memcpy(Out + 10, Payload.Bytes, Len);
    uint16_t Crc = 0xFFFF;
    for (size_t I = 1; I < 10 + Len; ++I)
    {
        CrcAccumulate(Out[I], Crc);
    }
    CrcAccumulate(CrcExtra, Crc);
    Out[10 + Len] = static_cast<uint8_t>(Crc & 0xFF);
    Out[11 + Len] = static_cast<uint8_t>(Crc >> 8);
    return 12 + Len;
}

// MAV_TYPE_QUADROTOR = 2, MAV_AUTOPILOT_GENERIC = 0,
// base_mode = CUSTOM_MODE_ENABLED | SAFETY_ARMED, MAV_STATE_ACTIVE = 4.
inline size_t Heartbeat(uint8_t* Out, uint8_t Seq, uint8_t SysId, uint8_t CompId)
{
    FPayload P;
    P.Put<uint32_t>(0);    // custom_mode
    P.Put<uint8_t>(2);     // type
    P.Put<uint8_t>(0);     // autopilot
    P.Put<uint8_t>(0x81);  // base_mode
    P.Put<uint8_t>(4);     // system_status
    P.Put<uint8_t>(3);     // mavlink_version
    return Finalize(Out, 0, 50, P, Seq, SysId, CompId);
}

inline size_t SystemTime(uint8_t* Out, uint8_t Seq, uint8_t SysId, uint8_t CompId,
                         uint64_t TimeUnixUsec, uint32_t TimeBootMs)
{
    FPayload P;
    P.Put<uint64_t>(TimeUnixUsec);
    P.Put<uint32_t>(TimeBootMs);
    return Finalize(Out, 2, 137, P, Seq, SysId, CompId);
}

// Angles in radians, NED body convention (yaw 0 = north, clockwise positive).
inline size_t Attitude(uint8_t* Out, uint8_t Seq, uint8_t SysId, uint8_t CompId, uint32_t TimeBootMs,
                       float Roll, float Pitch, float Yaw, float RollSpeed, float PitchSpeed, float YawSpeed)
{
    FPayload P;
    P.Put<uint32_t>(TimeBootMs);
    P.Put<float>(Roll);
    P.Put<float>(Pitch);
    P.Put<float>(Yaw);
    P.Put<float>(RollSpeed);
    P.Put<float>(PitchSpeed);
    P.Put<float>(YawSpeed);
    return Finalize(Out, 30, 39, P, Seq, SysId, CompId);
}

// Lat/lon in degE7, altitudes in mm (alt = MSL, relative = above home),
// velocities in cm/s (NED), heading in cdeg (UINT16_MAX = unknown).
inline size_t GlobalPositionInt(uint8_t* Out, uint8_t Seq, uint8_t SysId, uint8_t CompId,
                                uint32_t TimeBootMs, int32_t LatE7, int32_t LonE7, int32_t AltMm,
                                int32_t RelativeAltMm, int16_t Vx, int16_t Vy, int16_t Vz, uint16_t HdgCdeg)
{
    FPayload P;
    P.Put<uint32_t>(TimeBootMs);
    P.Put<int32_t>(LatE7);
    P.Put<int32_t>(LonE7);
    P.Put<int32_t>(AltMm);
    P.Put<int32_t>(RelativeAltMm);
    P.Put<int16_t>(Vx);
    P.Put<int16_t>(Vy);
    P.Put<int16_t>(Vz);
    P.Put<uint16_t>(HdgCdeg);
    return Finalize(Out, 33, 104, P, Seq, SysId, CompId);
}

inline size_t VfrHud(uint8_t* Out, uint8_t Seq, uint8_t SysId, uint8_t CompId, float Airspeed,
                     float Groundspeed, float AltMsl, float Climb, int16_t HeadingDeg, uint16_t ThrottlePct)
{
    FPayload P;
    P.Put<float>(Airspeed);
    P.Put<float>(Groundspeed);
    P.Put<float>(AltMsl);
    P.Put<float>(Climb);
    P.Put<int16_t>(HeadingDeg);
    P.Put<uint16_t>(ThrottlePct);
    return Finalize(Out, 74, 20, P, Seq, SysId, CompId);
}
// Terrain under the vehicle (as ArduPilot reports it): TerrainHeightM = terrain
// above MSL, CurrentHeightM = vehicle height above that terrain (AGL).
inline size_t TerrainReport(uint8_t* Out, uint8_t Seq, uint8_t SysId, uint8_t CompId, int32_t LatE7, int32_t LonE7,
                            float TerrainHeightM, float CurrentHeightM)
{
    FPayload P;
    P.Put<int32_t>(LatE7);
    P.Put<int32_t>(LonE7);
    P.Put<float>(TerrainHeightM);
    P.Put<float>(CurrentHeightM);
    P.Put<uint16_t>(0); // spacing
    P.Put<uint16_t>(0); // pending
    P.Put<uint16_t>(1); // loaded
    return Finalize(Out, 136, 1, P, Seq, SysId, CompId);
}
} // namespace MavlinkLite
