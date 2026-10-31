// Compass tape at the top of the viewport (not in the video): centred on the
// direction the top of the camera image points to, with markers for the
// drone's nose, its ground track and the next waypoint, so a panned camera is
// easy to tell from the flight direction.
#pragma once

#include "CoreMinimal.h"
#include "Templates/Function.h"
#include "Widgets/DeclarativeSyntaxSupport.h"
#include "Widgets/SLeafWidget.h"

// Compass bearings in degrees: 0 = north, clockwise.
struct FDroneCompassState
{
    double ViewDeg = 0.0;          // top of the camera image (body heading + camera pan)
    double CameraTiltDeg = -90.0;  // -90 = straight down
    double NoseDeg = 0.0;          // drone heading
    bool bMoving = false;
    double MoveDeg = 0.0;          // ground track
    double SpeedMps = 0.0;
    bool bTarget = false;          // flying a route / mission
    double TargetDeg = 0.0;        // next waypoint
    double TargetDistanceM = 0.0;
};

class SDroneCompass : public SLeafWidget
{
public:
    SLATE_BEGIN_ARGS(SDroneCompass) {}
    SLATE_ARGUMENT(TFunction<FDroneCompassState()>, State)
    SLATE_END_ARGS()

    void Construct(const FArguments& InArgs);

    virtual int32 OnPaint(const FPaintArgs& Args, const FGeometry& AllottedGeometry, const FSlateRect& MyCullingRect,
                          FSlateWindowElementList& OutDrawElements, int32 LayerId, const FWidgetStyle& InWidgetStyle,
                          bool bParentEnabled) const override;
    virtual FVector2D ComputeDesiredSize(float) const override;

private:
    TFunction<FDroneCompassState()> StateFn;
};

// The compass in a top-centre box for UGameViewportClient::AddViewportWidgetContent.
TSharedRef<SWidget> MakeDroneCompass(TFunction<FDroneCompassState()> GetState);

// Height taken by MakeDroneCompass at the top of the viewport (the status line goes below).
constexpr float DroneCompassTopSpace = 86.f;
