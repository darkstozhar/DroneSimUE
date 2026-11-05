// 3D preview of the FPV drone in the builder: a model put together from the
// engine's basic shapes, sized from the chosen parts (frame wheelbase, motor
// bell, prop diameter and blade count, battery size, camera / GoPro), turning
// slowly with spinning props. A scene capture that sees only this actor draws
// it into a render target, which the builder page shows as an image.
//
// A real mesh can replace the generated one later: set ModelMesh on the actor
// (any static mesh, X forward, Z up, centred) and the generated parts are hidden.
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Sim/FpvBuild.h"

#include "DronePreview.generated.h"

class UMaterialInterface;
class USceneCaptureComponent2D;
class USceneComponent;
class UStaticMesh;
class UStaticMeshComponent;
class UTextureRenderTarget2D;
class UPointLightComponent;

UCLASS()
class ADronePreview : public AActor
{
    GENERATED_BODY()

public:
    ADronePreview();

    // Spawns the preview far above the map (only it is captured, so nothing else shows).
    static ADronePreview* SpawnIn(UWorld* World);

    // Rebuilds the model when the parts changed (cheap to call every frame).
    void ShowBuild(const FpvBuild::Selection& Parts);

    // Mouse drag in the image: turn the model (degrees) and zoom (factor).
    void Orbit(float DeltaYawDeg, float DeltaPitchDeg);
    void Zoom(float Factor);
    void ResetView(); // auto-turn again, default angle and zoom

    UTextureRenderTarget2D* GetImage() const { return Image; }

    virtual void Tick(float DeltaSeconds) override;

    // Optional: a real drone mesh instead of the generated one.
    UPROPERTY(EditAnywhere, Category = "Drone|Preview")
    TObjectPtr<UStaticMesh> ModelMesh;

private:
    UStaticMeshComponent* AddPart(UStaticMesh* Mesh, USceneComponent* Parent, const FVector& LocationCm, const FRotator& Rotation,
                                  const FVector& SizeCm, const FLinearColor& Color);
    void Rebuild();
    void PlaceCamera();

    UPROPERTY()
    TObjectPtr<USceneComponent> Root;

    UPROPERTY()
    TObjectPtr<USceneComponent> Model; // turns slowly

    UPROPERTY()
    TObjectPtr<USceneCaptureComponent2D> Capture;

    UPROPERTY()
    TObjectPtr<UPointLightComponent> KeyLight;

    UPROPERTY()
    TObjectPtr<UPointLightComponent> FillLight;

    UPROPERTY()
    TObjectPtr<UTextureRenderTarget2D> Image;

    UPROPERTY()
    TArray<TObjectPtr<UStaticMeshComponent>> Parts;

    UPROPERTY()
    TArray<TObjectPtr<USceneComponent>> Rotors; // spun in Tick

    UPROPERTY()
    TObjectPtr<UStaticMesh> Cube;

    UPROPERTY()
    TObjectPtr<UStaticMesh> Cylinder;

    UPROPERTY()
    TObjectPtr<UMaterialInterface> BaseMaterial;

    FpvBuild::Selection Shown;
    bool bBuilt = false;
    float ModelYawDeg = 30.f;
    float ViewPitchDeg = -22.f;
    float ZoomFactor = 1.f;
    bool bAutoTurn = true;
    float ModelSizeCm = 30.f;
    TArray<float> RotorSpin; // +1 clockwise, -1 counter-clockwise
};
