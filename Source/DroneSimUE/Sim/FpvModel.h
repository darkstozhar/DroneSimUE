// FPV quadcopter in acro (rate) mode, like a real FPV drone with no flight
// assistance: the sticks command angular rates (Betaflight "Actual" rates),
// the attitude stays where the pilot leaves it, the throttle sets the thrust
// (no altitude hold), and gravity, momentum and air drag do the rest.
// Plain C++ (no Unreal types) so it can be tested outside the engine.
// Frames: world NED (north, east, down), body FRD (forward, right, down).
#pragma once

#include <cmath>

namespace FpvModel
{
constexpr double kGravity = 9.80665;
constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kMaxSubstepS = 0.004;

// Own helpers: windows.h may define min / max macros.
inline double ClampTo(double Value, double Low, double High)
{
    return Value < Low ? Low : (Value > High ? High : Value);
}

inline double Larger(double A, double B)
{
    return A > B ? A : B;
}

struct Vec3
{
    double X = 0.0;
    double Y = 0.0;
    double Z = 0.0;
};

inline Vec3 MakeVec(double X, double Y, double Z)
{
    Vec3 V;
    V.X = X;
    V.Y = Y;
    V.Z = Z;
    return V;
}

inline double Length(const Vec3& V)
{
    return std::sqrt(V.X * V.X + V.Y * V.Y + V.Z * V.Z);
}

struct Quat
{
    double W = 1.0;
    double X = 0.0;
    double Y = 0.0;
    double Z = 0.0;
};

inline Quat Mul(const Quat& A, const Quat& B)
{
    Quat R;
    R.W = A.W * B.W - A.X * B.X - A.Y * B.Y - A.Z * B.Z;
    R.X = A.W * B.X + A.X * B.W + A.Y * B.Z - A.Z * B.Y;
    R.Y = A.W * B.Y - A.X * B.Z + A.Y * B.W + A.Z * B.X;
    R.Z = A.W * B.Z + A.X * B.Y - A.Y * B.X + A.Z * B.W;
    return R;
}

inline Quat Normalized(const Quat& Q)
{
    const double N = std::sqrt(Q.W * Q.W + Q.X * Q.X + Q.Y * Q.Y + Q.Z * Q.Z);
    if (N < 1e-12)
    {
        return Quat();
    }
    Quat R;
    R.W = Q.W / N;
    R.X = Q.X / N;
    R.Y = Q.Y / N;
    R.Z = Q.Z / N;
    return R;
}

// Body vector -> world vector.
inline Vec3 Rotate(const Quat& Q, const Vec3& V)
{
    // v' = v + 2w (q x v) + 2 q x (q x v)
    const double TX = 2.0 * (Q.Y * V.Z - Q.Z * V.Y);
    const double TY = 2.0 * (Q.Z * V.X - Q.X * V.Z);
    const double TZ = 2.0 * (Q.X * V.Y - Q.Y * V.X);
    return MakeVec(V.X + Q.W * TX + (Q.Y * TZ - Q.Z * TY), V.Y + Q.W * TY + (Q.Z * TX - Q.X * TZ),
                   V.Z + Q.W * TZ + (Q.X * TY - Q.Y * TX));
}

inline Quat AxisAngle(double AX, double AY, double AZ, double AngleRad)
{
    const double S = std::sin(0.5 * AngleRad);
    Quat R;
    R.W = std::cos(0.5 * AngleRad);
    R.X = AX * S;
    R.Y = AY * S;
    R.Z = AZ * S;
    return R;
}

// Aerospace Z-Y-X order: yaw (clockwise from north), pitch (nose up +), roll (right wing down +).
inline Quat FromYawPitchRoll(double YawRad, double PitchRad, double RollRad)
{
    return Mul(Mul(AxisAngle(0.0, 0.0, 1.0, YawRad), AxisAngle(0.0, 1.0, 0.0, PitchRad)), AxisAngle(1.0, 0.0, 0.0, RollRad));
}

inline void ToYawPitchRoll(const Quat& Q, double& YawRad, double& PitchRad, double& RollRad)
{
    RollRad = std::atan2(2.0 * (Q.W * Q.X + Q.Y * Q.Z), 1.0 - 2.0 * (Q.X * Q.X + Q.Y * Q.Y));
    PitchRad = std::asin(ClampTo(2.0 * (Q.W * Q.Y - Q.Z * Q.X), -1.0, 1.0));
    YawRad = std::atan2(2.0 * (Q.W * Q.Z + Q.X * Q.Y), 1.0 - 2.0 * (Q.Y * Q.Y + Q.Z * Q.Z));
}

// Betaflight-style "Actual" rates: CenterDps near the centre, MaxDps at full stick, Expo 0..1.
struct Rates
{
    double CenterDps = 70.0;
    double MaxDps = 670.0;
    double Expo = 0.54;
};

inline double ActualRateDps(double Stick, const Rates& R)
{
    const double X = ClampTo(Stick, -1.0, 1.0);
    const double Magnitude = std::fabs(X);
    const double Expo = ClampTo(R.Expo, 0.0, 1.0);
    const double X5 = X * X * X * X * X;
    const double Curved = Magnitude * (X5 * Expo + X * (1.0 - Expo));
    return X * R.CenterDps + Larger(0.0, R.MaxDps - R.CenterDps) * Curved;
}

struct Params
{
    double ThrustToWeight = 6.0;      // full-throttle thrust / weight (5" freestyle quads: about 5..10)
    double MotorTimeConstantS = 0.03; // thrust response
    double RateTimeConstantS = 0.03;  // how fast the flight controller reaches the commanded rate
    double LinearDrag = 0.05;         // 1/s
    double QuadraticDrag = 0.035;     // 1/m: about 40 m/s top speed at full throttle with TWR 6
    Rates Roll;
    Rates Pitch;
    Rates Yaw;
};

// Sticks -1..1 (roll right +, pitch forward + = nose down, yaw right +), throttle 0..1.
struct Input
{
    double Throttle = 0.0;
    double Roll = 0.0;
    double Pitch = 0.0;
    double Yaw = 0.0;
};

struct State
{
    Vec3 Vel;                 // m/s, NED
    Quat Att;                 // body FRD -> NED
    Vec3 Rate;                // rad/s, body (roll p, pitch q, yaw r)
    double ThrustAccel = 0.0; // m/s^2 along body up (after the motor lag)
};

// Thrust grows with the square of the throttle (propeller thrust ~ rpm^2).
inline double ThrustAccelFor(double Throttle, const Params& P)
{
    const double T = ClampTo(Throttle, 0.0, 1.0);
    return Larger(P.ThrustToWeight, 0.0) * kGravity * T * T;
}

inline double HoverThrottle(const Params& P)
{
    return std::sqrt(1.0 / Larger(P.ThrustToWeight, 1.01));
}

// Advances the state by Dt (in substeps of at most 4 ms); returns the displacement in NED metres.
inline Vec3 Step(State& S, const Input& In, const Params& P, double Dt)
{
    Vec3 Moved;
    if (!(Dt > 0.0))
    {
        return Moved;
    }
    const int Steps = static_cast<int>(std::ceil(Dt / kMaxSubstepS));
    const double H = Dt / (Steps > 0 ? Steps : 1);
    const double RateAlpha = 1.0 - std::exp(-H / Larger(P.RateTimeConstantS, 1e-3));
    const double MotorAlpha = 1.0 - std::exp(-H / Larger(P.MotorTimeConstantS, 1e-3));
    // Stick forward = nose down = negative pitch rate in FRD.
    const Vec3 TargetRate = MakeVec(ActualRateDps(In.Roll, P.Roll) * kDegToRad, -ActualRateDps(In.Pitch, P.Pitch) * kDegToRad,
                                    ActualRateDps(In.Yaw, P.Yaw) * kDegToRad);
    const double TargetThrust = ThrustAccelFor(In.Throttle, P);
    for (int Index = 0; Index < (Steps > 0 ? Steps : 1); ++Index)
    {
        S.Rate.X += (TargetRate.X - S.Rate.X) * RateAlpha;
        S.Rate.Y += (TargetRate.Y - S.Rate.Y) * RateAlpha;
        S.Rate.Z += (TargetRate.Z - S.Rate.Z) * RateAlpha;
        const double Omega = Length(S.Rate);
        if (Omega * H > 1e-12)
        {
            const Quat Turn = AxisAngle(S.Rate.X / Omega, S.Rate.Y / Omega, S.Rate.Z / Omega, Omega * H);
            S.Att = Normalized(Mul(S.Att, Turn)); // body rates: rotate in the body frame
        }
        S.ThrustAccel += (TargetThrust - S.ThrustAccel) * MotorAlpha;
        const Vec3 Thrust = Rotate(S.Att, MakeVec(0.0, 0.0, -S.ThrustAccel));
        const double DragK = P.LinearDrag + P.QuadraticDrag * Length(S.Vel);
        S.Vel.X += (Thrust.X - DragK * S.Vel.X) * H;
        S.Vel.Y += (Thrust.Y - DragK * S.Vel.Y) * H;
        S.Vel.Z += (Thrust.Z + kGravity - DragK * S.Vel.Z) * H;
        Moved.X += S.Vel.X * H;
        Moved.Y += S.Vel.Y * H;
        Moved.Z += S.Vel.Z * H;
    }
    return Moved;
}

// Angle between the body's up axis and the world's up, in degrees (0 = level, 180 = upside down).
inline double TiltDeg(const Quat& Att)
{
    const Vec3 BodyUp = Rotate(Att, MakeVec(0.0, 0.0, -1.0));
    return std::acos(ClampTo(-BodyUp.Z, -1.0, 1.0)) / kDegToRad;
}
} // namespace FpvModel
