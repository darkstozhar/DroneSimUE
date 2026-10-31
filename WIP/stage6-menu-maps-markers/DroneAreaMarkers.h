// Shows a survey area in the world: a pole at every vertex and see-through
// walls along the edges. No collision and no shadows, so the drone flies
// through them and the ground trace ignores them; the drone hides this actor
// from its video capture unless asked otherwise. Uses engine content only
// (BasicShapes cube / cylinder and the translucent material of 3D widgets).
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "DroneAreaMarkers.generated.h"

class ACesiumGeoreference;
class UMaterialInterface;
class UStaticMesh;
class UStaticMeshComponent;
class UTexture2D;

UCLASS(NotPlaceable, Transient)
class DRONESIMUE_API ADroneAreaMarkers : public AActor
{
    GENERATED_BODY()

public:
    ADroneAreaMarkers();

    // Vertices as (X = latitude, Y = longitude); heights are WGS84 ellipsoid heights.
    void Rebuild(ACesiumGeoreference* Georeference, const TArray<FVector2D>& Vertices, double BottomHeightM,
                 double TopHeightM, bool bPoles, bool bWalls, float WallOpacity);
    void Clear();
    // False if the engine meshes / material could not be loaded (then nothing is shown).
    bool HasAssets() const;

private:
    void AddPart(UStaticMesh* Mesh, UMaterialInterface* Material, const FTransform& PartTransform);
    UMaterialInterface* MakeColorMaterial(const FLinearColor& Color);

    UPROPERTY()
    TObjectPtr<USceneComponent> Root;

    UPROPERTY()
    TObjectPtr<UStaticMesh> CubeMesh;

    UPROPERTY()
    TObjectPtr<UStaticMesh> CylinderMesh;

    UPROPERTY()
    TObjectPtr<UMaterialInterface> TranslucentMaterial;

    UPROPERTY()
    TObjectPtr<UTexture2D> WhiteTexture;

    UPROPERTY(Transient)
    TArray<TObjectPtr<UStaticMeshComponent>> Parts;
};
