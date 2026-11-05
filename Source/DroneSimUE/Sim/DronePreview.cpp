#include "Sim/DronePreview.h"

#include "Components/PointLightComponent.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Engine/World.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Materials/MaterialInterface.h"
#include "UObject/ConstructorHelpers.h"

namespace
{
// Basic shapes are 100 cm: cube 100 x 100 x 100, cylinder 100 across and 100 high (axis Z).
FVector ShapeScale(const FVector& SizeCm)
{
    return SizeCm / 100.0;
}

const FLinearColor Carbon(0.025f, 0.025f, 0.03f);
const FLinearColor Alu(0.55f, 0.55f, 0.6f);
const FLinearColor PropOrange(1.f, 0.38f, 0.02f);
const FLinearColor BatteryYellow(0.95f, 0.72f, 0.08f);
const FLinearColor Dark(0.06f, 0.06f, 0.07f);
} // namespace

ADronePreview::ADronePreview()
{
    PrimaryActorTick.bCanEverTick = true;
    PrimaryActorTick.bTickEvenWhenPaused = true;

    Root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
    SetRootComponent(Root);
    Model = CreateDefaultSubobject<USceneComponent>(TEXT("Model"));
    Model->SetupAttachment(Root);

    Capture = CreateDefaultSubobject<USceneCaptureComponent2D>(TEXT("Capture"));
    Capture->SetupAttachment(Root);
    Capture->FOVAngle = 35.f;
    Capture->bCaptureEveryFrame = true;
    Capture->bCaptureOnMovement = false;
    Capture->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
    Capture->PrimitiveRenderMode = ESceneCapturePrimitiveRenderMode::PRM_UseShowOnlyList;
    // Only the drone: no sky, fog or clouds behind it.
    Capture->ShowFlags.SetAtmosphere(false);
    Capture->ShowFlags.SetFog(false);
    Capture->ShowFlags.SetVolumetricFog(false);
    Capture->ShowFlags.SetCloud(false);
    Capture->ShowFlags.SetMotionBlur(false);
    Capture->ShowFlags.SetBloom(false);
    // Studio lighting only: the parts and the two preview lights use lighting channel 1, the map's
    // sun stays on channel 0; no sky light / GI either, so the exposure can be fixed (EV100 = 0).
    Capture->ShowFlags.SetSkyLighting(false);
    Capture->ShowFlags.SetGlobalIllumination(false);
    Capture->ShowFlags.SetDynamicShadows(false);
    Capture->ShowFlags.SetAmbientOcclusion(false);
    Capture->PostProcessSettings.bOverride_AutoExposureMethod = true;
    Capture->PostProcessSettings.AutoExposureMethod = EAutoExposureMethod::AEM_Manual;
    Capture->PostProcessSettings.bOverride_AutoExposureApplyPhysicalCameraExposure = true;
    Capture->PostProcessSettings.AutoExposureApplyPhysicalCameraExposure = false;
    Capture->PostProcessSettings.bOverride_AutoExposureBias = true;
    Capture->PostProcessSettings.AutoExposureBias = 0.f;
    Capture->PostProcessBlendWeight = 1.f;

    KeyLight = CreateDefaultSubobject<UPointLightComponent>(TEXT("KeyLight"));
    KeyLight->SetupAttachment(Root);
    KeyLight->SetIntensityUnits(ELightUnits::Candelas);
    KeyLight->SetIntensity(5.f);
    KeyLight->SetAttenuationRadius(2000.f);
    KeyLight->SetCastShadows(false);
    KeyLight->LightingChannels.bChannel0 = false;
    KeyLight->LightingChannels.bChannel1 = true;
    FillLight = CreateDefaultSubobject<UPointLightComponent>(TEXT("FillLight"));
    FillLight->SetupAttachment(Root);
    FillLight->SetIntensityUnits(ELightUnits::Candelas);
    FillLight->SetIntensity(2.f);
    FillLight->SetLightColor(FLinearColor(0.75f, 0.85f, 1.f));
    FillLight->SetAttenuationRadius(2000.f);
    FillLight->SetCastShadows(false);
    FillLight->LightingChannels.bChannel0 = false;
    FillLight->LightingChannels.bChannel1 = true;

    static ConstructorHelpers::FObjectFinder<UStaticMesh> CubeFinder(TEXT("/Engine/BasicShapes/Cube.Cube"));
    static ConstructorHelpers::FObjectFinder<UStaticMesh> CylinderFinder(TEXT("/Engine/BasicShapes/Cylinder.Cylinder"));
    static ConstructorHelpers::FObjectFinder<UMaterialInterface> MaterialFinder(
        TEXT("/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial"));
    Cube = CubeFinder.Object;
    Cylinder = CylinderFinder.Object;
    BaseMaterial = MaterialFinder.Object;
}

ADronePreview* ADronePreview::SpawnIn(UWorld* World)
{
    if (World == nullptr)
    {
        return nullptr;
    }
    FActorSpawnParameters Params;
    Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
    Params.ObjectFlags |= RF_Transient;
    // 20 km up: away from the terrain; the capture shows only this actor anyway.
    ADronePreview* Preview =
        World->SpawnActor<ADronePreview>(ADronePreview::StaticClass(), FVector(0.0, 0.0, 2.0e6), FRotator::ZeroRotator, Params);
    if (Preview != nullptr)
    {
        Preview->Image = NewObject<UTextureRenderTarget2D>(Preview);
        Preview->Image->ClearColor = FLinearColor(0.035f, 0.037f, 0.045f, 1.f);
        Preview->Image->InitCustomFormat(960, 540, PF_B8G8R8A8, false);
        Preview->Image->UpdateResourceImmediate(true);
        Preview->Capture->TextureTarget = Preview->Image;
        Preview->Capture->ShowOnlyActors.Add(Preview);
    }
    return Preview;
}

void ADronePreview::ShowBuild(const FpvBuild::Selection& PartsIn)
{
    const bool bSame = bBuilt && PartsIn.FrameIndex == Shown.FrameIndex && PartsIn.MotorIndex == Shown.MotorIndex &&
                       PartsIn.PropIndex == Shown.PropIndex && PartsIn.BatteryIndex == Shown.BatteryIndex &&
                       PartsIn.FpvIndex == Shown.FpvIndex;
    if (!bSame)
    {
        Shown = PartsIn;
        Rebuild();
    }
}

void ADronePreview::Orbit(float DeltaYawDeg, float DeltaPitchDeg)
{
    bAutoTurn = false; // the user turns it now
    ModelYawDeg += DeltaYawDeg;
    ViewPitchDeg = FMath::Clamp(ViewPitchDeg + DeltaPitchDeg, -80.f, 20.f);
    PlaceCamera();
}

void ADronePreview::ResetView()
{
    bAutoTurn = true;
    ViewPitchDeg = -22.f;
    ZoomFactor = 1.f;
    PlaceCamera();
}

void ADronePreview::Zoom(float Factor)
{
    ZoomFactor = FMath::Clamp(ZoomFactor * Factor, 0.4f, 2.5f);
    PlaceCamera();
}

UStaticMeshComponent* ADronePreview::AddPart(UStaticMesh* Mesh, USceneComponent* Parent, const FVector& LocationCm,
                                             const FRotator& Rotation, const FVector& SizeCm, const FLinearColor& Color)
{
    UStaticMeshComponent* Part = NewObject<UStaticMeshComponent>(this);
    Part->SetStaticMesh(Mesh);
    Part->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    Part->SetCastShadow(false);
    Part->LightingChannels.bChannel0 = false;
    Part->LightingChannels.bChannel1 = true;
    Part->SetupAttachment(Parent);
    Part->SetRelativeLocation(LocationCm);
    Part->SetRelativeRotation(Rotation);
    Part->SetRelativeScale3D(ShapeScale(SizeCm));
    if (BaseMaterial != nullptr)
    {
        UMaterialInstanceDynamic* Mat = UMaterialInstanceDynamic::Create(BaseMaterial, Part);
        Mat->SetVectorParameterValue(TEXT("Color"), Color);
        Part->SetMaterial(0, Mat);
    }
    Part->RegisterComponent();
    Parts.Add(Part);
    return Part;
}

void ADronePreview::Rebuild()
{
    for (UStaticMeshComponent* Part : Parts)
    {
        if (Part != nullptr)
        {
            Part->DestroyComponent();
        }
    }
    Parts.Reset();
    for (USceneComponent* Rotor : Rotors)
    {
        if (Rotor != nullptr)
        {
            Rotor->DestroyComponent();
        }
    }
    Rotors.Reset();
    RotorSpin.Reset();
    bBuilt = true;

    using namespace FpvBuild;
    const Frame& F = kFrames[Pick(Shown.FrameIndex, CountOf(kFrames))];
    const Motor& M = kMotors[Pick(Shown.MotorIndex, CountOf(kMotors))];
    const Prop& Pr = kProps[Pick(Shown.PropIndex, CountOf(kProps))];
    const Battery& B = kBatteries[Pick(Shown.BatteryIndex, CountOf(kBatteries))];
    const int32 Video = Pick(Shown.FpvIndex, CountOf(kFpvSystems));
    const bool bDucted = Shown.FrameIndex == 0;

    if (ModelMesh != nullptr)
    {
        AddPart(ModelMesh, Model, FVector::ZeroVector, FRotator::ZeroRotator, FVector(100.0), FLinearColor::White);
        ModelSizeCm = 40.f;
        PlaceCamera();
        return;
    }

    // Sizes in cm.
    const double Reach = 0.05 * F.WheelbaseMm;                      // centre to motor
    const double BodyLen = F.BodySizeM * 100.0 * 1.7;
    const double BodyWid = F.BodySizeM * 100.0 * 0.62;
    const double ArmWid = 0.9 + 0.006 * F.WheelbaseMm;
    const double PlateT = 0.35 + 0.0012 * F.WheelbaseMm;
    const double BellR = M.BellRadiusM * 100.0;
    const double MotorH = BellR * 1.25;
    const double PropD = Pr.DiameterIn * 2.54;
    ModelSizeCm = static_cast<float>(2.0 * Reach + PropD);

    // Frame: bottom plate, top plate, standoffs, four arms at 45 deg (quad X).
    AddPart(Cube, Model, FVector(0, 0, 0), FRotator::ZeroRotator, FVector(BodyLen, BodyWid, PlateT), Carbon);
    AddPart(Cube, Model, FVector(0, 0, 3.0), FRotator::ZeroRotator, FVector(BodyLen * 0.92, BodyWid * 0.9, PlateT * 0.6), Carbon);
    for (int32 I = 0; I < 4; ++I)
    {
        const double Sx = (I % 2 == 0) ? 1.0 : -1.0, Sy = (I < 2) ? 1.0 : -1.0;
        AddPart(Cylinder, Model, FVector(Sx * BodyLen * 0.4, Sy * BodyWid * 0.4, 1.5), FRotator::ZeroRotator, FVector(0.5, 0.5, 3.0),
                Alu);
    }
    // FC/ESC stack between the plates.
    AddPart(Cube, Model, FVector(0, 0, 1.5), FRotator::ZeroRotator, FVector(3.6, 3.6, 1.6), FLinearColor(0.05f, 0.25f, 0.08f));

    // Motor order and spin as in FpvBuild (1 RR, 2 FR, 3 RL, 4 FL; props in).
    for (int32 I = 0; I < 4; ++I)
    {
        const FVector Pos(kMotorX[I] * Reach / FMath::Sqrt(2.0), kMotorY[I] * Reach / FMath::Sqrt(2.0), 0.0);
        const double YawDeg = FMath::RadiansToDegrees(FMath::Atan2(Pos.Y, Pos.X));
        AddPart(Cube, Model, Pos * 0.5, FRotator(0.0, YawDeg, 0.0), FVector(Reach, ArmWid, PlateT * 1.4), Carbon);
        // Motor: stator base + bell.
        AddPart(Cylinder, Model, Pos + FVector(0, 0, PlateT * 0.7 + MotorH * 0.5), FRotator::ZeroRotator,
                FVector(BellR * 2.0, BellR * 2.0, MotorH), FLinearColor(0.55f, 0.06f, 0.05f));
        AddPart(Cylinder, Model, Pos + FVector(0, 0, PlateT * 0.7 + MotorH + 0.25), FRotator::ZeroRotator,
                FVector(BellR * 1.4, BellR * 1.4, 0.5), Alu);
        // Rotor: hub + blades, spun in Tick.
        USceneComponent* Rotor = NewObject<USceneComponent>(this);
        Rotor->SetupAttachment(Model);
        Rotor->SetRelativeLocation(Pos + FVector(0, 0, PlateT * 0.7 + MotorH + 0.8));
        Rotor->RegisterComponent();
        Rotors.Add(Rotor);
        RotorSpin.Add(static_cast<float>(kMotorSpin[I]));
        AddPart(Cylinder, Rotor, FVector::ZeroVector, FRotator::ZeroRotator, FVector(1.4, 1.4, 1.0), PropOrange);
        for (int32 Blade = 0; Blade < Pr.Blades; ++Blade)
        {
            const double A = 360.0 * Blade / Pr.Blades;
            const FVector Out = FRotator(0.0, A, 0.0).RotateVector(FVector(PropD * 0.25, 0, 0));
            // Blade pitch: a few degrees of roll about its own long axis.
            AddPart(Cube, Rotor, Out, FRotator(0.0, A, 12.0 * -kMotorSpin[I]), FVector(PropD * 0.48, PropD * 0.085, 0.18),
                    PropOrange);
        }
        if (bDucted)
        {
            // Duct: a ring of short segments around the prop.
            const double DuctR = PropD * 0.5 + 0.4;
            for (int32 Seg = 0; Seg < 20; ++Seg)
            {
                const double A = 360.0 * Seg / 20.0;
                const FVector Offset = FRotator(0.0, A, 0.0).RotateVector(FVector(DuctR, 0, 0));
                AddPart(Cube, Model, Pos + Offset + FVector(0, 0, MotorH * 0.6), FRotator(0.0, A, 0.0),
                        FVector(0.35, 2.0 * PI * DuctR / 20.0 + 0.1, 3.0), Dark);
            }
        }
    }

    // Battery on the top plate: size from its mass (packs are about 2 g/cm3), strap across.
    const double Volume = B.MassG / 2.0;
    const double BattH = FMath::Pow(Volume / (2.3 * 1.2), 1.0 / 3.0);
    const FVector BattSize(BattH * 2.3, BattH * 1.2, BattH);
    const double BattZ = 3.0 + PlateT * 0.3 + BattH * 0.5;
    AddPart(Cube, Model, FVector(-BodyLen * 0.05, 0, BattZ), FRotator::ZeroRotator, BattSize,
            B.EmptyCellV < 3.2 ? FLinearColor(0.15f, 0.35f, 0.75f) : BatteryYellow);
    AddPart(Cube, Model, FVector(-BodyLen * 0.05, 0, BattZ), FRotator::ZeroRotator, BattSize + FVector(-BattSize.X * 0.75, 0.2, 0.2),
            Dark);

    // FPV camera at the front, tilted up 30 deg; digital systems get a VTX + antenna at the back.
    const FVector CamPos(BodyLen * 0.5 - 0.6, 0, 1.6);
    AddPart(Cube, Model, CamPos, FRotator(30.0, 0.0, 0.0), FVector(1.9, 1.9, 1.9), Dark);
    AddPart(Cylinder, Model, CamPos + FRotator(30.0, 0.0, 0.0).RotateVector(FVector(1.2, 0, 0)), FRotator(-60.0, 0.0, 0.0),
            FVector(1.4, 1.4, 0.6), FLinearColor(0.1f, 0.1f, 0.2f));
    if (Video >= 1)
    {
        AddPart(Cube, Model, FVector(-BodyLen * 0.4, 0, 1.5), FRotator::ZeroRotator, FVector(3.0, 3.0, 1.2), Dark);
    }
    AddPart(Cylinder, Model, FVector(-BodyLen * 0.5, 0, 4.0), FRotator(-35.0, 0.0, 0.0), FVector(0.4, 0.4, 7.0), Dark);
    if (Video >= 2)
    {
        // Action camera on top of the battery.
        const FVector GoPro(BodyLen * 0.25, 0, BattZ + BattH * 0.5 + 2.6);
        AddPart(Cube, Model, GoPro, FRotator(15.0, 0.0, 0.0), FVector(3.6, 6.2, 4.4), FLinearColor(0.02f, 0.02f, 0.02f));
        AddPart(Cylinder, Model, GoPro + FRotator(15.0, 0.0, 0.0).RotateVector(FVector(1.9, -1.4, 0.6)),
                FRotator(-75.0, 0.0, 0.0), FVector(2.2, 2.2, 0.8), FLinearColor(0.08f, 0.08f, 0.12f));
    }
    PlaceCamera();
}

void ADronePreview::PlaceCamera()
{
    const double Distance = ModelSizeCm * 2.2 * ZoomFactor;
    const FRotator View(ViewPitchDeg, 180.0, 0.0); // looking at the nose; the model turns
    const FVector Eye = -View.Vector() * Distance;
    Capture->SetRelativeLocationAndRotation(Eye, View);
    KeyLight->SetRelativeLocation(FVector(-Distance * 0.3, Distance * 0.6, Distance * 0.8));
    FillLight->SetRelativeLocation(FVector(Distance * 0.7, -Distance * 0.5, Distance * 0.2));
    KeyLight->SetAttenuationRadius(static_cast<float>(Distance * 4.0));
    FillLight->SetAttenuationRadius(static_cast<float>(Distance * 4.0));
    // About 5 lux on the model whatever the zoom (I = E * d^2); EV100 = 0 shows that as a good grey.
    const double DistanceM = Distance / 100.0;
    KeyLight->SetIntensity(static_cast<float>(5.0 * DistanceM * DistanceM));
    FillLight->SetIntensity(static_cast<float>(2.0 * DistanceM * DistanceM));
}

void ADronePreview::Tick(float DeltaSeconds)
{
    Super::Tick(DeltaSeconds);
    if (bAutoTurn)
    {
        ModelYawDeg += 18.f * DeltaSeconds;
    }
    Model->SetRelativeRotation(FRotator(0.0, ModelYawDeg, 0.0));
    for (int32 I = 0; I < Rotors.Num(); ++I)
    {
        if (Rotors[I] != nullptr)
        {
            Rotors[I]->AddRelativeRotation(FRotator(0.0, 900.0 * DeltaSeconds * RotorSpin[I], 0.0));
        }
    }
}
