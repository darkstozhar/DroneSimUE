#include "Sim/DroneCompass.h"

#include "Fonts/FontMeasure.h"
#include "Framework/Application/SlateApplication.h"
#include "Rendering/DrawElements.h"
#include "Rendering/SlateRenderer.h"
#include "Styling/CoreStyle.h"
#include "Widgets/Layout/SBox.h"

namespace
{
constexpr float CompassWidth = 640.f;
constexpr float CompassHeight = 72.f;
constexpr double HalfSpanDeg = 90.0; // the tape shows +-90 degrees around the view direction

const FLinearColor TickColor(0.85f, 0.85f, 0.85f, 1.f);
const FLinearColor ViewColor(1.f, 1.f, 1.f, 1.f);
const FLinearColor NoseColor(1.f, 0.55f, 0.1f, 1.f);
const FLinearColor MoveColor(0.35f, 1.f, 0.45f, 1.f);
const FLinearColor TargetColor(0.35f, 0.85f, 1.f, 1.f);

double SignedDelta(double FromDeg, double ToDeg)
{
    double D = FMath::Fmod(ToDeg - FromDeg + 180.0, 360.0);
    if (D < 0.0)
    {
        D += 360.0;
    }
    return D - 180.0;
}

int32 Rounded360(double Deg)
{
    const int32 R = FMath::RoundToInt(Deg) % 360;
    return R < 0 ? R + 360 : R;
}

const TCHAR* CardinalName(int32 Deg)
{
    switch (Deg)
    {
    case 0: return TEXT("N");
    case 45: return TEXT("NE");
    case 90: return TEXT("E");
    case 135: return TEXT("SE");
    case 180: return TEXT("S");
    case 225: return TEXT("SW");
    case 270: return TEXT("W");
    case 315: return TEXT("NW");
    default: return nullptr;
    }
}
} // namespace

void SDroneCompass::Construct(const FArguments& InArgs)
{
    StateFn = InArgs._State;
    // Repainted every frame even if the viewport uses Slate invalidation.
    ForceVolatile(true);
}

FVector2D SDroneCompass::ComputeDesiredSize(float) const
{
    return FVector2D(CompassWidth, CompassHeight);
}

int32 SDroneCompass::OnPaint(const FPaintArgs& Args, const FGeometry& AllottedGeometry, const FSlateRect& MyCullingRect,
                             FSlateWindowElementList& OutDrawElements, int32 LayerId, const FWidgetStyle& InWidgetStyle,
                             bool bParentEnabled) const
{
    const FDroneCompassState S = StateFn ? StateFn() : FDroneCompassState();
    const FVector2f LocalSize = AllottedGeometry.GetLocalSize();
    const float W = LocalSize.X;
    const float PxPerDeg = W / static_cast<float>(2.0 * HalfSpanDeg);
    const float TapeBottom = 34.f;
    const float MarkerTop = 36.f;
    const float InfoTop = 53.f;
    const FSlateBrush* WhiteBrush = FCoreStyle::Get().GetBrush("WhiteBrush");
    const TSharedRef<FSlateFontMeasure> FontMeasure = FSlateApplication::Get().GetRenderer()->GetFontMeasureService();
    const FSlateFontInfo TickFont = FCoreStyle::GetDefaultFontStyle("Regular", 8);
    const FSlateFontInfo CardinalFont = FCoreStyle::GetDefaultFontStyle("Bold", 11);
    const FSlateFontInfo InfoFont = FCoreStyle::GetDefaultFontStyle("Bold", 10);

    // Background
    FSlateDrawElement::MakeBox(OutDrawElements, LayerId, AllottedGeometry.ToPaintGeometry(), WhiteBrush,
                               ESlateDrawEffect::None, FLinearColor(0.f, 0.f, 0.f, 0.5f));
    const int32 TapeLayer = LayerId + 1;
    const int32 MarkerLayer = LayerId + 2;

    auto XOf = [&](double BearingDeg)
    {
        return W * 0.5f + static_cast<float>(SignedDelta(S.ViewDeg, BearingDeg)) * PxPerDeg;
    };
    auto DrawPolyline = [&](const TArray<FVector2f>& Points, const FLinearColor& Color, float Thickness, int32 Layer)
    {
        FSlateDrawElement::MakeLines(OutDrawElements, Layer, AllottedGeometry.ToPaintGeometry(), Points,
                                     ESlateDrawEffect::None, Color, true, Thickness);
    };
    auto DrawLabel = [&](const FString& Label, float LabelLeft, float LabelTop, const FSlateFontInfo& Font,
                         const FLinearColor& Color, int32 Layer)
    {
        const FVector2f LabelSize = FontMeasure->Measure(Label, Font);
        FSlateDrawElement::MakeText(OutDrawElements, Layer,
                                    AllottedGeometry.ToPaintGeometry(LabelSize, FSlateLayoutTransform(FVector2f(LabelLeft, LabelTop))),
                                    Label, Font, ESlateDrawEffect::None, Color);
        return LabelSize.X;
    };
    auto LabelWidth = [&](const FString& Label, const FSlateFontInfo& Font)
    {
        const FVector2f LabelSize = FontMeasure->Measure(Label, Font);
        return LabelSize.X;
    };

    // Tape: a tick every 5 degrees, numbers every 15, letters every 45.
    const int32 FirstTick = FMath::FloorToInt((S.ViewDeg - HalfSpanDeg) / 5.0) * 5;
    for (int32 TickDeg = FirstTick; TickDeg <= FirstTick + static_cast<int32>(2.0 * HalfSpanDeg) + 5; TickDeg += 5)
    {
        const float X = XOf(static_cast<double>(TickDeg));
        if (X < 2.f || X > W - 2.f)
        {
            continue;
        }
        const int32 Deg = ((TickDeg % 360) + 360) % 360;
        const TCHAR* Cardinal = CardinalName(Deg);
        const float Length = Cardinal != nullptr ? 12.f : (Deg % 15 == 0 ? 9.f : 5.f);
        DrawPolyline({FVector2f(X, 0.f), FVector2f(X, Length)}, TickColor, 1.5f, TapeLayer);
        if (Cardinal != nullptr)
        {
            const FString Label(Cardinal);
            DrawLabel(Label, X - 0.5f * LabelWidth(Label, CardinalFont), 13.f, CardinalFont,
                      Deg == 0 ? FLinearColor(1.f, 0.35f, 0.3f) : ViewColor, TapeLayer);
        }
        else if (Deg % 15 == 0)
        {
            const FString Label = FString::FromInt(Deg);
            DrawLabel(Label, X - 0.5f * LabelWidth(Label, TickFont), 15.f, TickFont, TickColor, TapeLayer);
        }
    }
    // The view direction (top of the image).
    DrawPolyline({FVector2f(W * 0.5f, 0.f), FVector2f(W * 0.5f, TapeBottom)}, ViewColor, 2.5f, MarkerLayer);

    // A marker below the tape; off the tape it sticks to the edge as an arrow.
    auto DrawMarker = [&](double BearingDeg, const FLinearColor& Color, int32 Shape)
    {
        const float RawX = XOf(BearingDeg);
        const float Y = MarkerTop;
        if (RawX < 8.f || RawX > W - 8.f)
        {
            const bool bLeft = RawX < 8.f;
            const float X = bLeft ? 6.f : W - 6.f;
            const float Dx = bLeft ? 8.f : -8.f;
            DrawPolyline({FVector2f(X + Dx, Y), FVector2f(X, Y + 6.f), FVector2f(X + Dx, Y + 12.f)}, Color, 2.5f, MarkerLayer);
            return;
        }
        const float X = RawX;
        if (Shape == 0) // triangle: the nose
        {
            DrawPolyline({FVector2f(X, Y), FVector2f(X - 7.f, Y + 12.f), FVector2f(X + 7.f, Y + 12.f), FVector2f(X, Y)}, Color,
                         2.5f, MarkerLayer);
        }
        else if (Shape == 1) // chevron: where it goes
        {
            DrawPolyline({FVector2f(X - 7.f, Y + 11.f), FVector2f(X, Y + 2.f), FVector2f(X + 7.f, Y + 11.f)}, Color, 2.5f,
                         MarkerLayer);
        }
        else // diamond: the next waypoint
        {
            DrawPolyline({FVector2f(X, Y), FVector2f(X - 6.f, Y + 6.f), FVector2f(X, Y + 12.f), FVector2f(X + 6.f, Y + 6.f),
                          FVector2f(X, Y)},
                         Color, 2.f, MarkerLayer);
        }
    };
    if (S.bTarget)
    {
        DrawMarker(S.TargetDeg, TargetColor, 2);
    }
    if (S.bMoving)
    {
        DrawMarker(S.MoveDeg, MoveColor, 1);
    }
    DrawMarker(S.NoseDeg, NoseColor, 0);

    // Legend with the numbers, one colour per marker.
    TArray<TPair<FString, FLinearColor>> Parts;
    Parts.Emplace(S.CameraTiltDeg > -89.5 ? FString::Printf(TEXT("camera %d (tilt %.0f)"), Rounded360(S.ViewDeg), S.CameraTiltDeg)
                                          : FString::Printf(TEXT("camera %d"), Rounded360(S.ViewDeg)),
                  ViewColor);
    Parts.Emplace(FString::Printf(TEXT("nose %d"), Rounded360(S.NoseDeg)), NoseColor);
    if (S.bMoving)
    {
        Parts.Emplace(FString::Printf(TEXT("track %d  %.1f m/s"), Rounded360(S.MoveDeg), S.SpeedMps), MoveColor);
    }
    if (S.bTarget)
    {
        Parts.Emplace(FString::Printf(TEXT("waypoint %d  %.0f m"), Rounded360(S.TargetDeg), S.TargetDistanceM), TargetColor);
    }
    const float Gap = 18.f;
    float Total = -Gap;
    for (const TPair<FString, FLinearColor>& Part : Parts)
    {
        Total += LabelWidth(Part.Key, InfoFont) + Gap;
    }
    float InfoX = 0.5f * (W - Total);
    for (const TPair<FString, FLinearColor>& Part : Parts)
    {
        InfoX += DrawLabel(Part.Key, InfoX, InfoTop, InfoFont, Part.Value, MarkerLayer) + Gap;
    }
    return MarkerLayer;
}

TSharedRef<SWidget> MakeDroneCompass(TFunction<FDroneCompassState()> GetState)
{
    return SNew(SBox)
        .Visibility(EVisibility::HitTestInvisible)
        .HAlign(HAlign_Center)
        .VAlign(VAlign_Top)
        .Padding(FMargin(0.f, 8.f, 0.f, 0.f))
        [
            SNew(SDroneCompass)
            .State(MoveTemp(GetState))
        ];
}
