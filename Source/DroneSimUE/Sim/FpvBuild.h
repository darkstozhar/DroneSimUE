// FPV drone builder + component-level physics, the way a real FPV quad works:
//
//   sticks -> rate setpoint (Betaflight "Actual" rates)
//          -> gyro (PT1-filtered) -> rate PID with Betaflight gain units (P/I/D/FF, TPA, iterm reset)
//          -> quad-X mixer with airmode and dynamic idle
//          -> 4 ESC duty commands -> brushless motor model (KV, winding resistance, rotor+prop inertia,
//             prop torque, active braking) -> per-prop thrust and reaction torque
//          -> rigid body (inertia tensor from the parts, gyroscopic coupling, body and rotor drag)
//   battery: open-circuit voltage from the state of charge minus I*R sag, drained by the real current.
//
// So the parts change the flight: a heavier battery lowers the thrust/weight and the authority,
// a low-KV motor on a small prop is sluggish, a big prop on a small motor overloads it, a weak
// Li-ion pack sags on punch-outs, and the same PID tune that is crisp on a 5" is soft on a 10".
//
// Plain C++ (no Unreal types) so it can be tested outside the engine; uses FpvModel's math.
// Frames: world NED, body FRD (as FpvModel).
#pragma once

#include "FpvModel.h"

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace FpvBuild
{
using FpvModel::ClampTo;
using FpvModel::kGravity;
using FpvModel::kPi;
using FpvModel::Larger;
using FpvModel::MakeVec;
using FpvModel::Quat;
using FpvModel::Vec3;

constexpr double kAirDensity = 1.225;
constexpr double kMaxSubstepS = 0.0005; // 2 kHz physics / PID loop (Betaflight runs 4..8 kHz)

// --- Component catalog (typical market parts; numbers are representative, not one vendor's datasheet) --

struct Frame
{
    const char* Name;
    double WheelbaseMm;   // motor-to-motor diagonal
    double MassG;         // bare frame
    double MaxPropIn;     // largest prop that clears the arms / ducts
    double FrontAreaM2;   // CdA-ish frontal area, forward flight
    double TopAreaM2;     // CdA-ish area seen from above (vertical drag, prop wash)
    double BodySizeM;     // centre stack + battery box edge (for the inertia)
};

struct Motor
{
    const char* Name;
    double Kv;            // rpm per volt
    double MassG;
    double ResistanceOhm; // phase-to-phase winding resistance
    double MaxCurrentA;   // short-burst rating
    double NoLoadCurrentA;
    double BellRadiusM;   // rotor (bell) radius
};

struct Prop
{
    const char* Name;
    double DiameterIn;
    double PitchIn;
    int Blades;
    double MassG;
};

struct Battery
{
    const char* Name;
    int Cells;
    double CapacityMah;
    double MassG;
    double ResistanceMOhmPerCell; // internal resistance, warm pack
    double MaxCurrentA;           // honest continuous rating (not the label C)
    double EmptyCellV;            // open-circuit voltage at 0 % (LiPo 3.3, Li-ion 3.0)
};

struct FpvSystem
{
    const char* Name;
    double MassG; // camera + VTX + antenna (+ action camera)
};

inline const Frame kFrames[] = {
    {"3\" cinewhoop (ducted)", 140.0, 90.0, 3.0, 0.016, 0.030, 0.06},
    {"3.5\" freestyle", 150.0, 55.0, 3.5, 0.008, 0.016, 0.05},
    {"5\" freestyle", 225.0, 120.0, 5.1, 0.011, 0.025, 0.07},
    {"5\" race (light)", 210.0, 85.0, 5.1, 0.008, 0.020, 0.06},
    {"7\" long range", 295.0, 160.0, 7.0, 0.014, 0.040, 0.08},
    {"10\" heavy lifter", 420.0, 320.0, 10.0, 0.022, 0.080, 0.11},
};

inline const Motor kMotors[] = {
    {"1404 3800KV", 3800.0, 9.0, 0.180, 15.0, 0.5, 0.0095},
    {"2004 3000KV", 3000.0, 19.0, 0.100, 25.0, 0.7, 0.0125},
    {"2306 2450KV", 2450.0, 31.0, 0.065, 38.0, 1.1, 0.0145},
    {"2207 1960KV", 1960.0, 33.0, 0.070, 40.0, 1.0, 0.0145},
    {"2806.5 1300KV", 1300.0, 45.0, 0.080, 40.0, 0.9, 0.0170},
    {"3115 900KV", 900.0, 70.0, 0.075, 50.0, 1.0, 0.0195},
};

inline const Prop kProps[] = {
    {"3x3 tri-blade", 3.0, 3.0, 3, 1.6},
    {"3.5x2.8 tri-blade", 3.5, 2.8, 3, 2.2},
    {"5x4.3 tri-blade", 5.0, 4.3, 3, 4.3},
    {"5.1x3.1 tri-blade (efficient)", 5.1, 3.1, 3, 4.0},
    {"7x3.5 bi-blade", 7.0, 3.5, 2, 6.5},
    {"10x5 bi-blade", 10.0, 5.0, 2, 14.0},
};

inline const Battery kBatteries[] = {
    {"4S 650 mAh LiPo", 4, 650.0, 80.0, 9.0, 60.0, 3.3},
    {"4S 1300 mAh LiPo", 4, 1300.0, 160.0, 5.0, 110.0, 3.3},
    {"6S 1100 mAh LiPo", 6, 1100.0, 190.0, 6.0, 90.0, 3.3},
    {"6S 1500 mAh LiPo", 6, 1500.0, 250.0, 4.5, 120.0, 3.3},
    {"6S1P 4000 mAh Li-ion", 6, 4000.0, 420.0, 18.0, 30.0, 3.0},
    {"6S2P 8000 mAh Li-ion", 6, 8000.0, 820.0, 9.0, 60.0, 3.0},
};

inline const FpvSystem kFpvSystems[] = {
    {"Analog cam + VTX", 12.0},
    {"Digital HD (DJI / Walksnail class)", 30.0},
    {"Digital HD + GoPro", 150.0},
};

template <typename T, int N>
constexpr int CountOf(const T (&)[N])
{
    return N;
}

inline int Pick(int Index, int Count)
{
    return Index < 0 ? 0 : (Index >= Count ? Count - 1 : Index);
}

struct Selection
{
    int FrameIndex = 2;   // 5" freestyle
    int MotorIndex = 2;   // 2306 2450KV
    int PropIndex = 2;    // 5x4.3
    int BatteryIndex = 1; // 4S 1300
    int FpvIndex = 1;     // digital HD
};

// Ready-made drones (the builder's "Ready" list); user drones are saved next to them as JSON.
struct Preset
{
    const char* Name;
    const char* Description;
    Selection Parts;
    double PidMaster;
    double PidDamping;
};

inline const Preset kPresets[] = {
    {"5\" freestyle 4S", "The classic: 2306 2450KV, 5x4.3 tri-blades, 4S 1300. Punchy and agile.", {2, 2, 2, 1, 1}, 1.0, 1.0},
    {"5\" freestyle 6S", "Same frame on 6S with 1960KV motors: more top end, less sag.", {2, 3, 2, 3, 1}, 1.0, 1.0},
    {"5\" racer", "Light race frame, 6S 1100, analog video: the most power per gram.", {3, 3, 2, 2, 0}, 1.05, 1.0},
    {"5\" cinematic + GoPro", "Freestyle quad carrying an action camera: heavier, smoother.", {2, 2, 3, 1, 2}, 0.95, 1.15},
    {"3\" cinewhoop", "Ducted 3\" for close indoor/proximity shots: slow, stable, short flights.", {0, 0, 0, 0, 1}, 1.0, 1.1},
    {"3.5\" light freestyle", "Small and light, analog video: floaty, forgiving.", {1, 1, 1, 0, 0}, 1.0, 1.0},
    {"7\" long range Li-ion", "2806.5 1300KV, 7\" bi-blades, 6S 4000 Li-ion: 10+ minutes, sags on punch-outs.", {4, 4, 4, 4, 1}, 0.9, 1.25},
    {"10\" heavy lifter", "3115 900KV, 10\" props, 6S2P 8000 Li-ion + GoPro: long endurance, slow to react.", {5, 5, 5, 5, 2}, 0.75, 1.5},
};

// Betaflight 4.4/4.5 default PIDs for a 5" (same units as the configurator).
struct Pids
{
    double P[3] = {45.0, 47.0, 45.0};
    double I[3] = {80.0, 84.0, 80.0};
    double D[3] = {40.0, 46.0, 0.0};
    double F[3] = {120.0, 125.0, 120.0};
    double Master = 1.0;    // configurator "master multiplier" slider
    double DGain = 1.0;     // "damping" slider
    double TpaRate = 0.65;  // D attenuation at full throttle (Betaflight 4.3+: TPA on D)
    double TpaBreakpoint = 0.35;
    double DynamicIdle = 0.055; // lowest motor output when armed (airmode keeps authority at zero throttle)
};

// --- Derived physical model -------------------------------------------------------------------

struct Physical
{
    double MassKg = 0.6;
    double Inertia[3] = {0.002, 0.002, 0.004}; // kg m^2, body x y z
    double ArmM = 0.08;                        // motor offset along body x and y (quad X)
    // Prop: T = Ct rho n^2 D^4, Q = Cq rho n^2 D^5 (n in rev/s)
    double PropDiameterM = 0.127;
    double PropPitchM = 0.109;
    double Ct = 0.12;
    double Cq = 0.009;
    double RotorInertia = 8e-6; // motor bell + prop
    // Motor
    double KvRadPerVolt = 256.0; // rad/s per volt
    double ResistanceOhm = 0.065;
    double NoLoadTorqueNm = 0.004;
    double MotorMaxCurrentA = 38.0;
    // Battery
    int Cells = 4;
    double CapacityAh = 1.3;
    double PackResistanceOhm = 0.02;
    double EmptyCellV = 3.3;
    double BatteryMaxCurrentA = 110.0;
    double AvionicsCurrentA = 0.6;
    // Drag
    double CdA[3] = {0.011, 0.012, 0.025}; // m^2: body x (forward), y (side), z (top)
    double RotorDragK = 2e-4;              // N per (m/s) per (rev/s * m): blade flapping / H-force
};

inline double CellOcv(double Soc, double EmptyCellV)
{
    // Rough LiPo/Li-ion curve: flat middle, knee near empty.
    const double S = ClampTo(Soc, 0.0, 1.0);
    const double Span = 4.2 - EmptyCellV;
    const double Knee = std::pow(1.0 - S, 6.0);
    return EmptyCellV + Span * (0.18 + 0.82 * S) - Span * 0.18 * Knee;
}

inline Physical Derive(const Selection& Sel)
{
    const Frame& F = kFrames[Pick(Sel.FrameIndex, CountOf(kFrames))];
    const Motor& M = kMotors[Pick(Sel.MotorIndex, CountOf(kMotors))];
    const Prop& Pr = kProps[Pick(Sel.PropIndex, CountOf(kProps))];
    const Battery& B = kBatteries[Pick(Sel.BatteryIndex, CountOf(kBatteries))];
    const FpvSystem& V = kFpvSystems[Pick(Sel.FpvIndex, CountOf(kFpvSystems))];

    Physical P;
    const double StackG = 18.0, MiscG = 45.0; // FC/ESC stack, receiver, straps, wires, screws
    const double RotorG = M.MassG + Pr.MassG;
    const double CoreKg = (F.MassG + StackG + MiscG + V.MassG) / 1000.0;
    const double BattKg = B.MassG / 1000.0;
    P.MassKg = 4.0 * RotorG / 1000.0 + CoreKg + BattKg;

    const double R = 0.5 * F.WheelbaseMm / 1000.0;
    P.ArmM = R / std::sqrt(2.0);
    // Inertia: motors as point masses, core as a flat box, battery as a box on top.
    const double Mr = 4.0 * RotorG / 1000.0;
    const double S = F.BodySizeM;
    const double BattH = 0.03 + 0.00002 * B.MassG; // battery height above the centre
    const double Ixx = Mr * P.ArmM * P.ArmM + CoreKg * (S * S + 0.02 * 0.02) / 12.0 + BattKg * (S * S / 12.0 + BattH * BattH);
    const double Iyy = Ixx * 1.1; // a bit longer than wide
    const double Izz = Mr * R * R + (CoreKg + BattKg) * (2.0 * S * S) / 12.0;
    P.Inertia[0] = Ixx;
    P.Inertia[1] = Iyy;
    P.Inertia[2] = Izz;

    // Prop coefficients from the pitch/diameter ratio and the blade count (rough fit to UIUC/APC-style data).
    const double D = Pr.DiameterIn * 0.0254;
    const double PD = Pr.PitchIn / Pr.DiameterIn;
    const double BladeK = Pr.Blades == 2 ? 0.80 : (Pr.Blades >= 4 ? 1.15 : 1.0);
    P.PropDiameterM = D;
    P.PropPitchM = Pr.PitchIn * 0.0254;
    P.Ct = (0.085 + 0.055 * (PD - 0.5)) * BladeK;
    const double Cp = (0.032 + 0.045 * (PD - 0.5)) * BladeK;
    P.Cq = Cp / (2.0 * kPi);
    const double PropJ = (Pr.MassG / 1000.0) * (D * 0.5) * (D * 0.5) / 3.0;
    const double BellJ = 0.35 * (M.MassG / 1000.0) * M.BellRadiusM * M.BellRadiusM;
    P.RotorInertia = PropJ + BellJ;

    P.KvRadPerVolt = M.Kv * 2.0 * kPi / 60.0;
    P.ResistanceOhm = M.ResistanceOhm;
    P.NoLoadTorqueNm = M.NoLoadCurrentA / P.KvRadPerVolt;
    P.MotorMaxCurrentA = M.MaxCurrentA;

    P.Cells = B.Cells;
    P.CapacityAh = B.CapacityMah / 1000.0;
    P.PackResistanceOhm = B.Cells * B.ResistanceMOhmPerCell / 1000.0 + 0.004; // + leads, XT60
    P.EmptyCellV = B.EmptyCellV;
    P.BatteryMaxCurrentA = B.MaxCurrentA;

    P.CdA[0] = F.FrontAreaM2 + 0.00002 * B.MassG; // the battery sticks out
    P.CdA[1] = F.FrontAreaM2 * 1.1 + 0.00002 * B.MassG;
    P.CdA[2] = F.TopAreaM2;
    P.RotorDragK = 2e-4;
    return P;
}

// Prop speed (rad/s) where the motor at a constant voltage balances the prop torque.
inline double SteadyOmega(const Physical& P, double Volts)
{
    // V = w/Kv + R*I,  I = Kv*(Q + Q0),  Q = Cq rho D^5 (w/2pi)^2  ->  a w^2 + b w + c = 0
    const double D5 = std::pow(P.PropDiameterM, 5.0);
    const double A = P.ResistanceOhm * P.KvRadPerVolt * P.Cq * kAirDensity * D5 / (4.0 * kPi * kPi);
    const double Bc = 1.0 / P.KvRadPerVolt;
    const double C = P.ResistanceOhm * P.KvRadPerVolt * P.NoLoadTorqueNm - Volts;
    if (C >= 0.0)
    {
        return 0.0;
    }
    return (-Bc + std::sqrt(Bc * Bc - 4.0 * A * C)) / (2.0 * A);
}

inline double ThrustAt(const Physical& P, double Omega)
{
    const double N = Omega / (2.0 * kPi);
    return P.Ct * kAirDensity * N * N * std::pow(P.PropDiameterM, 4.0);
}

inline double TorqueAt(const Physical& P, double Omega)
{
    const double N = Omega / (2.0 * kPi);
    return P.Cq * kAirDensity * N * N * std::pow(P.PropDiameterM, 5.0);
}

inline double MotorCurrentAt(const Physical& P, double Omega)
{
    return (TorqueAt(P, Omega) + P.NoLoadTorqueNm) * P.KvRadPerVolt;
}

// Prop speed for a target thrust per motor.
inline double OmegaForThrust(const Physical& P, double ThrustN)
{
    const double N = std::sqrt(Larger(ThrustN, 0.0) / (P.Ct * kAirDensity * std::pow(P.PropDiameterM, 4.0)));
    return 2.0 * kPi * N;
}

struct Summary
{
    double MassG = 0.0;
    double ThrustToWeight = 0.0;   // static, full pack, with sag
    double HoverThrottle = 0.0;    // 0..1, full pack
    double HoverCurrentA = 0.0;    // whole quad
    double MaxCurrentA = 0.0;      // whole quad at full throttle
    double MotorMaxCurrentA = 0.0; // one motor at full throttle
    double HoverMinutes = 0.0;     // using 80 % of the pack
    double MaxSpeedKmh = 0.0;      // level-flight speed after a 10 s full-throttle run (Curves)
    double MaxRpm = 0.0;           // full throttle, full pack, static
    double MaxThrustKg = 0.0;      // all four motors, static
    bool bCanHover = true;
    std::string Warnings;
};

// Static full-throttle point with battery sag (a few fixed-point iterations).
inline void FullThrottlePoint(const Physical& P, double Soc, double& OutOmega, double& OutPackA, double& OutVolts)
{
    const double Ocv = CellOcv(Soc, P.EmptyCellV) * P.Cells;
    double Amps = 0.0, Omega = 0.0, Volts = Ocv;
    for (int I = 0; I < 30; ++I)
    {
        Volts = Larger(Ocv - Amps * P.PackResistanceOhm, 0.5 * Ocv);
        Omega = SteadyOmega(P, Volts);
        Amps = 0.5 * Amps + 0.5 * (4.0 * MotorCurrentAt(P, Omega) + P.AvionicsCurrentA);
    }
    OutOmega = Omega;
    OutPackA = Amps;
    OutVolts = Volts;
}

inline Summary Summarize(const Selection& Sel, const Physical& P)
{
    Summary Sm;
    Sm.MassG = P.MassKg * 1000.0;
    double OmegaMax = 0.0, PackA = 0.0, Volts = 0.0;
    FullThrottlePoint(P, 1.0, OmegaMax, PackA, Volts);
    const double MaxThrust = 4.0 * ThrustAt(P, OmegaMax);
    const double Weight = P.MassKg * kGravity;
    Sm.ThrustToWeight = MaxThrust / Weight;
    Sm.MaxCurrentA = PackA;
    Sm.MaxRpm = OmegaMax * 60.0 / (2.0 * kPi);
    Sm.MaxThrustKg = MaxThrust / kGravity;
    Sm.MotorMaxCurrentA = MotorCurrentAt(P, OmegaMax);

    const double OmegaHover = OmegaForThrust(P, Weight / 4.0);
    const double MotorA = MotorCurrentAt(P, OmegaHover);
    Sm.HoverCurrentA = 4.0 * MotorA + P.AvionicsCurrentA;
    const double Ocv = CellOcv(0.6, P.EmptyCellV) * P.Cells;
    const double PackV = Ocv - Sm.HoverCurrentA * P.PackResistanceOhm;
    const double MotorV = OmegaHover / P.KvRadPerVolt + MotorA * P.ResistanceOhm;
    const double Idle = Pids().DynamicIdle;
    Sm.HoverThrottle = (MotorV / Larger(PackV, 0.1) - Idle) / (1.0 - Idle); // stick position
    Sm.bCanHover = Sm.ThrustToWeight > 1.05;
    Sm.HoverMinutes = 0.8 * P.CapacityAh / Larger(Sm.HoverCurrentA, 0.1) * 60.0;

    // Top speed: tilt until the vertical thrust just holds the weight, then the drag balances.
    const double CosTilt = ClampTo(1.0 / Larger(Sm.ThrustToWeight, 1.0001), 0.05, 1.0);
    const double Horizontal = Weight * std::sqrt(1.0 - CosTilt * CosTilt) / CosTilt;
    const double DragArea = 0.5 * kAirDensity * (P.CdA[0] * CosTilt + P.CdA[2] * std::sqrt(1.0 - CosTilt * CosTilt));
    Sm.MaxSpeedKmh = std::sqrt(Horizontal / Larger(DragArea, 1e-4)) * 3.6 * 0.75; // prop thrust drops with speed

    const Frame& F = kFrames[Pick(Sel.FrameIndex, CountOf(kFrames))];
    const Prop& Pr = kProps[Pick(Sel.PropIndex, CountOf(kProps))];
    char Buffer[160];
    if (Pr.DiameterIn > F.MaxPropIn + 0.05)
    {
        std::snprintf(Buffer, sizeof(Buffer), "Prop %.1f\" does not fit the frame (max %.1f\"). ", Pr.DiameterIn, F.MaxPropIn);
        Sm.Warnings += Buffer;
    }
    if (Pr.DiameterIn < F.MaxPropIn - 1.6)
    {
        Sm.Warnings += "Props much smaller than the frame: low thrust, wobbly. ";
    }
    if (Sm.MotorMaxCurrentA > P.MotorMaxCurrentA)
    {
        std::snprintf(Buffer, sizeof(Buffer), "Motors overloaded: %.0f A at full throttle (rated %.0f A). ", Sm.MotorMaxCurrentA,
                      P.MotorMaxCurrentA);
        Sm.Warnings += Buffer;
    }
    if (Sm.MaxCurrentA > P.BatteryMaxCurrentA)
    {
        std::snprintf(Buffer, sizeof(Buffer), "Battery can't supply %.0f A (about %.0f A): heavy voltage sag. ", Sm.MaxCurrentA,
                      P.BatteryMaxCurrentA);
        Sm.Warnings += Buffer;
    }
    if (!Sm.bCanHover)
    {
        Sm.Warnings += "Can't lift itself. ";
    }
    else if (Sm.ThrustToWeight < 2.0)
    {
        Sm.Warnings += "Thrust/weight under 2: sluggish, little control in wind. ";
    }
    if (Sm.HoverThrottle > 0.7 && Sm.bCanHover)
    {
        Sm.Warnings += "Hovers above 70 % throttle: no headroom. ";
    }
    return Sm;
}

// --- Flight controller + plant ---------------------------------------------------------------

struct PhysState
{
    double MotorOmega[4] = {0.0, 0.0, 0.0, 0.0}; // rad/s
    double MotorCmd[4] = {0.0, 0.0, 0.0, 0.0};   // ESC duty 0..1
    double MotorCurrentA[4] = {0.0, 0.0, 0.0, 0.0};
    double UsedAh = 0.0;
    double PackVolts = 0.0; // under load
    double PackCurrentA = 0.0;
    double GyroF[3] = {0.0, 0.0, 0.0};   // filtered gyro, deg/s
    double DtermF[3] = {0.0, 0.0, 0.0};  // filtered gyro for the D term, deg/s
    double PrevDGyro[3] = {0.0, 0.0, 0.0};
    double PrevSetpoint[3] = {0.0, 0.0, 0.0};
    double FfF[3] = {0.0, 0.0, 0.0};
    double SpLp[3] = {0.0, 0.0, 0.0};
    double ITerm[3] = {0.0, 0.0, 0.0};
};

inline double Soc(const Physical& P, const PhysState& S)
{
    return ClampTo(1.0 - S.UsedAh / Larger(P.CapacityAh, 1e-3), 0.0, 1.0);
}

// Motor layout, Betaflight quad X order (1 rear-right, 2 front-right, 3 rear-left, 4 front-left), props in.
// Positions in body FRD (x forward, y right); Spin +1 = clockwise seen from above.
constexpr double kMotorX[4] = {-1.0, 1.0, -1.0, 1.0};
constexpr double kMotorY[4] = {1.0, 1.0, -1.0, -1.0};
constexpr double kMotorSpin[4] = {1.0, -1.0, -1.0, 1.0};
// Mixer: + roll (right wing down) -> left motors up; + pitch (nose up) -> front motors up;
// + yaw (nose right) -> counter-clockwise motors up (their reaction torque turns the frame clockwise).
constexpr double kMixRoll[4] = {-1.0, -1.0, 1.0, 1.0};
constexpr double kMixPitch[4] = {-1.0, 1.0, -1.0, 1.0};
constexpr double kMixYaw[4] = {-1.0, 1.0, 1.0, -1.0};

inline double Pt1Alpha(double CutoffHz, double Dt)
{
    const double Rc = 1.0 / (2.0 * kPi * CutoffHz);
    return Dt / (Rc + Dt);
}

inline Vec3 RotateInv(const Quat& Q, const Vec3& V)
{
    Quat C = Q;
    C.X = -C.X;
    C.Y = -C.Y;
    C.Z = -C.Z;
    return FpvModel::Rotate(C, V);
}

inline void ResetForPark(PhysState& S, const Physical& P)
{
    const double Used = S.UsedAh;
    S = PhysState();
    S.UsedAh = Used;
    S.PackVolts = CellOcv(Soc(P, S), P.EmptyCellV) * P.Cells;
}

// Spins the motors up to hover (for entering FPV mode in the air); returns the hover throttle.
inline double PrimeHover(PhysState& S, const Physical& P, FpvModel::State& Body)
{
    const double Omega = OmegaForThrust(P, P.MassKg * kGravity / 4.0);
    const double MotorA = MotorCurrentAt(P, Omega);
    const double PackA = 4.0 * MotorA + P.AvionicsCurrentA;
    S.PackVolts = CellOcv(Soc(P, S), P.EmptyCellV) * P.Cells - PackA * P.PackResistanceOhm;
    const double Duty = (Omega / P.KvRadPerVolt + MotorA * P.ResistanceOhm) / Larger(S.PackVolts, 0.1);
    for (int M = 0; M < 4; ++M)
    {
        S.MotorOmega[M] = Omega;
        S.MotorCmd[M] = Duty;
        S.MotorCurrentA[M] = MotorA;
    }
    S.PackCurrentA = PackA;
    Body.ThrustAccel = kGravity;
    return ClampTo((Duty - Pids().DynamicIdle) / (1.0 - Pids().DynamicIdle), 0.0, 1.0); // stick position
}

// One PID-loop + physics tick.
inline Vec3 SubStep(FpvModel::State& Body, PhysState& S, const FpvModel::Input& In, const FpvModel::Params& Rates,
                    const Physical& P, const Pids& Pid, double H)
{
    // 1) Setpoint (deg/s, FRD: stick forward = nose down = negative pitch rate).
    const double Sp[3] = {FpvModel::ActualRateDps(In.Roll, Rates.Roll), -FpvModel::ActualRateDps(In.Pitch, Rates.Pitch),
                          FpvModel::ActualRateDps(In.Yaw, Rates.Yaw)};
    // 2) Gyro: true body rate -> deg/s, PT1 lowpass (gyro LPF ~ 250 Hz, D-term LPF ~ 100 Hz).
    const double True[3] = {Body.Rate.X / FpvModel::kDegToRad, Body.Rate.Y / FpvModel::kDegToRad,
                            Body.Rate.Z / FpvModel::kDegToRad};
    const double GyroA = Pt1Alpha(250.0, H), DA = Pt1Alpha(100.0, H), FfA = Pt1Alpha(40.0, H);
    const double Throttle = ClampTo(In.Throttle, 0.0, 1.0);
    // TPA: less D at high throttle (props bite harder there).
    const double Tpa = Throttle > Pid.TpaBreakpoint
                           ? 1.0 - (1.0 - Pid.TpaRate) * (Throttle - Pid.TpaBreakpoint) / (1.0 - Pid.TpaBreakpoint)
                           : 1.0;
    const bool bResetI = Throttle < 0.03; // on the ground / throttle cut: no wind-up
    double Out[3];
    for (int A = 0; A < 3; ++A)
    {
        S.GyroF[A] += (True[A] - S.GyroF[A]) * GyroA;
        S.DtermF[A] += (S.GyroF[A] - S.DtermF[A]) * DA;
        const double Err = Sp[A] - S.GyroF[A];
        const double Pterm = Pid.Master * Pid.P[A] * 0.032029 * Err;
        // I-term relax (Betaflight iterm_relax, setpoint mode): high-pass the setpoint at 15 Hz;
        // while it moves faster than 40 deg/s the I term stops growing (no bounce-back after flips).
        const double SpRate = (Sp[A] - S.PrevSetpoint[A]) / H;
        S.SpLp[A] += (Sp[A] - S.SpLp[A]) * Pt1Alpha(15.0, H);
        const double Relax = Larger(0.0, 1.0 - std::fabs(Sp[A] - S.SpLp[A]) / 40.0);
        S.ITerm[A] += Pid.Master * Pid.I[A] * 0.244381 * Err * Relax * H;
        S.ITerm[A] = bResetI ? 0.0 : ClampTo(S.ITerm[A], -400.0, 400.0);
        const double DGyro = (S.DtermF[A] - S.PrevDGyro[A]) / H;
        S.PrevDGyro[A] = S.DtermF[A];
        const double Dterm = -Pid.Master * Pid.DGain * Pid.D[A] * 0.000529 * DGyro * Tpa;
        S.FfF[A] += (SpRate - S.FfF[A]) * FfA;
        S.PrevSetpoint[A] = Sp[A];
        const double Fterm = Pid.Master * Pid.F[A] * 0.013754 * S.FfF[A] * 0.001; // per-ms setpoint delta in BF
        const double Limit = A == 2 ? 400.0 : 500.0;
        Out[A] = ClampTo(Pterm + S.ITerm[A] + Dterm + Fterm, -Limit, Limit) / 1000.0;
    }
    // 3) Mixer with airmode: keep the full PID correction, shift it into [0, 1].
    double Mix[4], MinM = 1e9, MaxM = -1e9;
    for (int M = 0; M < 4; ++M)
    {
        Mix[M] = Out[0] * kMixRoll[M] + Out[1] * kMixPitch[M] + Out[2] * kMixYaw[M];
        MinM = Mix[M] < MinM ? Mix[M] : MinM;
        MaxM = Mix[M] > MaxM ? Mix[M] : MaxM;
    }
    const double Range = MaxM - MinM;
    if (Range > 1.0)
    {
        for (double& V : Mix)
        {
            V /= Range;
        }
        MinM /= Range;
        MaxM /= Range;
    }
    const double ThrLow = Larger(-MinM, 0.0), ThrHigh = 1.0 - MaxM;
    const double Thr = ThrLow > ThrHigh ? 0.5 * (ThrLow + ThrHigh) : ClampTo(Throttle, ThrLow, ThrHigh);
    // 4) ESC duty with dynamic idle; 5) motors.
    const double Ocv = CellOcv(Soc(P, S), P.EmptyCellV) * P.Cells;
    double PackA = P.AvionicsCurrentA;
    Vec3 Force = MakeVec(0.0, 0.0, 0.0), Torque = MakeVec(0.0, 0.0, 0.0);
    double TotalThrust = 0.0, SumN = 0.0;
    const double D4 = std::pow(P.PropDiameterM, 4.0);
    for (int M = 0; M < 4; ++M)
    {
        const double Duty = Pid.DynamicIdle + (1.0 - Pid.DynamicIdle) * ClampTo(Thr + Mix[M], 0.0, 1.0);
        S.MotorCmd[M] = Duty;
        const double Volts = Duty * S.PackVolts;
        double Amps = (Volts - S.MotorOmega[M] / P.KvRadPerVolt) / P.ResistanceOhm; // negative = active braking
        Amps = ClampTo(Amps, -0.5 * P.MotorMaxCurrentA, 1.5 * P.MotorMaxCurrentA);  // ESC current limit
        const double Q = TorqueAt(P, S.MotorOmega[M]);
        const double OmegaDot = (Amps / P.KvRadPerVolt - Q - P.NoLoadTorqueNm) / P.RotorInertia;
        S.MotorOmega[M] = Larger(S.MotorOmega[M] + OmegaDot * H, 0.0);
        S.MotorCurrentA[M] = Amps;
        PackA += Amps * Duty;
        const double N = S.MotorOmega[M] / (2.0 * kPi);
        const double T = P.Ct * kAirDensity * N * N * D4;
        TotalThrust += T;
        SumN += N;
        // Thrust along body up (-z); torque r x F.
        Torque.X += -kMotorY[M] * P.ArmM * T;
        Torque.Y += kMotorX[M] * P.ArmM * T;
        // Reaction of the prop drag and of spinning the rotor up (both push the frame the other way).
        Torque.Z += -kMotorSpin[M] * (Q + P.RotorInertia * OmegaDot);
    }
    Force.Z = -TotalThrust;
    S.PackCurrentA = Larger(PackA, 0.0);
    const double TargetVolts = Larger(Ocv - S.PackCurrentA * P.PackResistanceOhm, 0.3 * Ocv);
    S.PackVolts += (TargetVolts - S.PackVolts) * Pt1Alpha(30.0, H);
    S.UsedAh += S.PackCurrentA * H / 3600.0;

    // 6) Drag in the body frame: quadratic body drag + rotor H-force (linear, grows with prop speed).
    const Vec3 VB = RotateInv(Body.Att, Body.Vel);
    const double Speed = FpvModel::Length(VB);
    const double Rotor = P.RotorDragK * SumN * P.PropDiameterM;
    Force.X += -0.5 * kAirDensity * P.CdA[0] * Speed * VB.X - Rotor * VB.X;
    Force.Y += -0.5 * kAirDensity * P.CdA[1] * Speed * VB.Y - Rotor * VB.Y;
    Force.Z += -0.5 * kAirDensity * P.CdA[2] * Speed * VB.Z;

    // 7) Rigid body: w' = I^-1 (tau - w x Iw).
    const double* J = P.Inertia;
    const Vec3 W = Body.Rate;
    const Vec3 Jw = MakeVec(J[0] * W.X, J[1] * W.Y, J[2] * W.Z);
    const Vec3 Gyro = MakeVec(W.Y * Jw.Z - W.Z * Jw.Y, W.Z * Jw.X - W.X * Jw.Z, W.X * Jw.Y - W.Y * Jw.X);
    Body.Rate.X += (Torque.X - Gyro.X) / J[0] * H;
    Body.Rate.Y += (Torque.Y - Gyro.Y) / J[1] * H;
    Body.Rate.Z += (Torque.Z - Gyro.Z) / J[2] * H;
    const double Omega = FpvModel::Length(Body.Rate);
    if (Omega * H > 1e-12)
    {
        const Quat Turn = FpvModel::AxisAngle(Body.Rate.X / Omega, Body.Rate.Y / Omega, Body.Rate.Z / Omega, Omega * H);
        Body.Att = FpvModel::Normalized(FpvModel::Mul(Body.Att, Turn));
    }
    const Vec3 FW = FpvModel::Rotate(Body.Att, Force);
    Body.Vel.X += FW.X / P.MassKg * H;
    Body.Vel.Y += FW.Y / P.MassKg * H;
    Body.Vel.Z += (FW.Z / P.MassKg + kGravity) * H;
    Body.ThrustAccel = TotalThrust / P.MassKg;
    return MakeVec(Body.Vel.X * H, Body.Vel.Y * H, Body.Vel.Z * H);
}

// Advances by Dt in 0.5 ms ticks; returns the NED displacement in metres.
inline Vec3 Step(FpvModel::State& Body, PhysState& S, const FpvModel::Input& In, const FpvModel::Params& Rates,
                 const Physical& P, const Pids& Pid, double Dt)
{
    Vec3 Moved;
    if (!(Dt > 0.0))
    {
        return Moved;
    }
    if (S.PackVolts <= 0.0)
    {
        S.PackVolts = CellOcv(Soc(P, S), P.EmptyCellV) * P.Cells;
    }
    const int Steps = static_cast<int>(std::ceil(Larger(Dt, 0.0) / kMaxSubstepS));
    const double H = Dt / Steps;
    for (int I = 0; I < Steps; ++I)
    {
        const Vec3 D = SubStep(Body, S, In, Rates, P, Pid, H);
        Moved.X += D.X;
        Moved.Y += D.Y;
        Moved.Z += D.Z;
    }
    return Moved;
}
// --- Throttle curve (Betaflight thr_mid / thr_expo / throttle_limit "scale") --------------

inline double ThrottleCurve(double Stick, double Mid, double Expo, double LimitPct)
{
    const double T = ClampTo(Stick, 0.0, 1.0);
    const double M = ClampTo(Mid, 0.05, 0.95);
    const double E = ClampTo(Expo, 0.0, 1.0);
    const double Tmp = T - M;
    const double Range = Tmp > 0.0 ? 1.0 - M : M;
    const double Curved = M + Tmp * (1.0 - E + E * (Tmp * Tmp) / (Range * Range));
    return ClampTo(Curved, 0.0, 1.0) * ClampTo(LimitPct, 10.0, 100.0) / 100.0;
}

// Stick position that gives a throttle output (bisection; the curve is monotonic).
inline double ThrottleStickFor(double Output, double Mid, double Expo, double LimitPct)
{
    double Low = 0.0, High = 1.0;
    for (int I = 0; I < 40; ++I)
    {
        const double M = 0.5 * (Low + High);
        (ThrottleCurve(M, Mid, Expo, LimitPct) < Output ? Low : High) = M;
    }
    return 0.5 * (Low + High);
}

// --- Curves for the builder's graphs (like the drone pages of other FPV simulators) --------

struct Curve
{
    const char* Title;
    const char* XUnit;
    const char* YUnit;
    std::vector<double> X;
    std::vector<double> Y;
};

struct Curves
{
    Curve ThrustTime{"Thrust | Time", "s", "kg", {}, {}};        // full-throttle forward run: sag + airspeed eat the thrust
    Curve VelocityTime{"Velocity | Time", "s", "km/h", {}, {}};  // same run, level flight
    Curve DragVelocity{"Air drag | Velocity", "km/h", "N", {}, {}};
    Curve ThrustRpm{"Thrust | RPM", "rpm", "g", {}, {}};         // one motor, static
    Curve DropTime{"Drop speed | Time", "s", "km/h", {}, {}};    // motors off, falling flat
};

inline double DragForceN(const Physical& P, double SpeedMps, double CosTilt, double RotorRevS)
{
    const double SinTilt = std::sqrt(Larger(0.0, 1.0 - CosTilt * CosTilt));
    const double CdA = P.CdA[0] * CosTilt + P.CdA[2] * SinTilt; // a tilted quad shows its top to the wind
    return 0.5 * kAirDensity * CdA * SpeedMps * SpeedMps + P.RotorDragK * 4.0 * RotorRevS * P.PropDiameterM * SpeedMps;
}

inline Curves ComputeCurves(const Physical& P)
{
    Curves C;
    const double Weight = P.MassKg * kGravity;
    const double D4 = std::pow(P.PropDiameterM, 4.0);
    // Full-throttle forward run, 10 s: tilt set at the start so the thrust just holds the height.
    double Omega0 = 0.0, PackA = 0.0, Volts = 0.0;
    FullThrottlePoint(P, 1.0, Omega0, PackA, Volts);
    const double T0 = 4.0 * ThrustAt(P, Omega0);
    const double CosTilt = ClampTo(Weight / Larger(T0, 1e-6), 0.05, 1.0);
    const double SinTilt = std::sqrt(1.0 - CosTilt * CosTilt);
    double V = 0.0, UsedAh = 0.0;
    const double H = 0.01;
    for (int I = 0; I <= 1000; ++I)
    {
        const double Soc = ClampTo(1.0 - UsedAh / P.CapacityAh, 0.0, 1.0);
        double Omega = 0.0;
        FullThrottlePoint(P, Soc, Omega, PackA, Volts);
        const double N = Omega / (2.0 * kPi);
        // Prop thrust falls as the inflow (airspeed along the prop axis) nears the pitch speed.
        const double Inflow = V * SinTilt;
        const double PitchSpeed = Larger(P.PropPitchM * N, 1.0);
        const double T = 4.0 * P.Ct * kAirDensity * N * N * D4 * ClampTo(1.0 - Inflow / PitchSpeed, 0.0, 1.0);
        if (I % 10 == 0)
        {
            C.ThrustTime.X.push_back(I * H);
            C.ThrustTime.Y.push_back(T / kGravity);
            C.VelocityTime.X.push_back(I * H);
            C.VelocityTime.Y.push_back(V * 3.6);
        }
        V = Larger(0.0, V + (T * SinTilt - DragForceN(P, V, CosTilt, N)) / P.MassKg * H);
        UsedAh += PackA * H / 3600.0;
    }
    const double VMax = Larger(V, 1.0);
    for (int I = 0; I <= 60; ++I)
    {
        const double Vi = VMax * 1.2 * I / 60.0;
        C.DragVelocity.X.push_back(Vi * 3.6);
        C.DragVelocity.Y.push_back(DragForceN(P, Vi, CosTilt, Omega0 / (2.0 * kPi)));
    }
    for (int I = 0; I <= 60; ++I)
    {
        const double Omega = Omega0 * I / 60.0;
        C.ThrustRpm.X.push_back(Omega * 60.0 / (2.0 * kPi));
        C.ThrustRpm.Y.push_back(ThrustAt(P, Omega) / kGravity * 1000.0);
    }
    double Vd = 0.0;
    for (int I = 0; I <= 600; ++I)
    {
        if (I % 10 == 0)
        {
            C.DropTime.X.push_back(I * H);
            C.DropTime.Y.push_back(Vd * 3.6);
        }
        Vd += (kGravity - 0.5 * kAirDensity * (P.CdA[2] * 1.6) * Vd * Vd / P.MassKg) * H; // stopped props add drag
    }
    return C;
}
} // namespace FpvBuild
