#include "Sim/DroneAreaMarkers.h"

#include "CesiumGeoreference.h"
#include "Components/SceneComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Engine/Texture2D.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Materials/MaterialInterface.h"
#include "Math/RotationMatrix.h"
#include "UObject/ConstructorHelpers.h"

DEFINE_LOG_CATEGORY_STATIC(LogDroneMarkers, Log, All);

namespace
{
// BasicShapes are 100 x 100 x 100 units (1 m), centred on their pivot.
constexpr double ShapeSizeCm = 100.0;
constexpr double PoleDiameterCm = 600.0;   // 6 m: about 20 px from 500 m with the default camera
constexpr double WallThicknessCm = 100.0;  // 1 m
} // namespace

ADroneAreaMarkers::ADroneAreaMarkers()
{
    PrimaryActorTick.bCanEverTick = false;
    SetActorEnableCollision(false);

    Root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
    Root->SetMobility(EComponentMobility::Movable);
    SetRootComponent(Root);

    // Loaded by the class default object, so the cooker packages them.
    static ConstructorHelpers::FObjectFinder<UStaticMesh> CubeFinder(TEXT("/Engine/BasicShapes/Cube.Cube"));
    static ConstructorHelpers::FObjectFinder<UStaticMesh> CylinderFinder(TEXT("/Engine/BasicShapes/Cylinder.Cylinder"));
    // Unlit translucent material of UWidgetComponent: texture "SlateUI" tinted by
    // "TintColorAndOpacity" (its alpha = opacity).
    static ConstructorHelpers::FObjectFinder<UMaterialInterface> MaterialFinder(
        TEXT("/Engine/EngineMaterials/Widget3DPassThrough_Translucent"));
    static ConstructorHelpers::FObjectFinder<UTexture2D> WhiteFinder(TEXT("/Engine/EngineResources/WhiteSquareTexture"));
    CubeMesh = CubeFinder.Object;
    CylinderMesh = CylinderFinder.Object;
    TranslucentMaterial = MaterialFinder.Object;
    WhiteTexture = WhiteFinder.Object;
}

bool ADroneAreaMarkers::HasAssets() const
{
    return CubeMesh != nullptr && CylinderMesh != nullptr && TranslucentMaterial != nullptr;
}

void ADroneAreaMarkers::Clear()
{
    for (TObjectPtr<UStaticMeshComponent>& Part : Parts)
    {
        if (Part != nullptr)
        {
            Part->DestroyComponent();
        }
    }
    Parts.Reset();
}

UMaterialInterface* ADroneAreaMarkers::MakeColorMaterial(const FLinearColor& Color)
{
    if (TranslucentMaterial == nullptr)
    {
        return nullptr;
    }
    UMaterialInstanceDynamic* Instance = UMaterialInstanceDynamic::Create(TranslucentMaterial, this);
    if (WhiteTexture != nullptr)
    {
        Instance->SetTextureParameterValue(TEXT("SlateUI"), WhiteTexture);
    }
    Instance->SetVectorParameterValue(TEXT("TintColorAndOpacity"), Color);
    Instance->SetScalarParameterValue(TEXT("OpacityFromTexture"), 1.f);
    return Instance;
}

void ADroneAreaMarkers::AddPart(UStaticMesh* Mesh, UMaterialInterface* Material, const FTransform& PartTransform)
{
    UStaticMeshComponent* Part = NewObject<UStaticMeshComponent>(this);
    Part->SetMobility(EComponentMobility::Movable);
    Part->SetStaticMesh(Mesh);
    if (Material != nullptr)
    {
        Part->SetMaterial(0, Material);
    }
    Part->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    Part->SetCollisionResponseToAllChannels(ECR_Ignore);
    Part->SetGenerateOverlapEvents(false);
    Part->SetCastShadow(false);
    Part->SetCanEverAffectNavigation(false);
    Part->SetupAttachment(Root);
    Part->RegisterComponent();
    Part->SetWorldTransform(PartTransform);
    Parts.Add(Part);
}

void ADroneAreaMarkers::Rebuild(ACesiumGeoreference* Georeference, const TArray<FVector2D>& Vertices,
                                double BottomHeightM, double TopHeightM, bool bPoles, bool bWalls, float WallOpacity)
{
    Clear();
    if (Georeference == nullptr || Vertices.Num() == 0 || (!bPoles && !bWalls))
    {
        return;
    }
    if (!HasAssets())
    {
        UE_LOG(LogDroneMarkers, Warning, TEXT("Area markers: engine meshes / material not found, nothing shown."));
        return;
    }

    // Bottom and top of every vertex in world space: the up direction follows
    // the globe, so markers far from the georeference origin stand upright too.
    const FTransform GeoToWorld = Georeference->GetActorTransform();
    TArray<FVector> Bottoms, Tops;
    for (const FVector2D& Vertex : Vertices)
    {
        Bottoms.Add(GeoToWorld.TransformPosition(
            Georeference->TransformLongitudeLatitudeHeightPositionToUnreal(FVector(Vertex.Y, Vertex.X, BottomHeightM))));
        Tops.Add(GeoToWorld.TransformPosition(
            Georeference->TransformLongitudeLatitudeHeightPositionToUnreal(FVector(Vertex.Y, Vertex.X, TopHeightM))));
    }

    if (bPoles)
    {
        UMaterialInterface* PoleMaterial = MakeColorMaterial(FLinearColor(1.f, 0.45f, 0.05f, 0.9f));
        for (int32 Index = 0; Index < Vertices.Num(); ++Index)
        {
            const FVector Axis = Tops[Index] - Bottoms[Index];
            const FQuat Rotation = FRotationMatrix::MakeFromZ(Axis).ToQuat();
            const FVector Scale(PoleDiameterCm / ShapeSizeCm, PoleDiameterCm / ShapeSizeCm, Axis.Size() / ShapeSizeCm);
            AddPart(CylinderMesh, PoleMaterial, FTransform(Rotation, (Bottoms[Index] + Tops[Index]) * 0.5, Scale));
        }
    }

    if (bWalls && Vertices.Num() >= 2)
    {
        UMaterialInterface* WallMaterial =
            MakeColorMaterial(FLinearColor(0.15f, 0.75f, 1.f, FMath::Clamp(WallOpacity, 0.01f, 1.f)));
        // A closed ring for an area; a single wall for two points.
        const int32 EdgeCount = Vertices.Num() >= 3 ? Vertices.Num() : 1;
        for (int32 Index = 0; Index < EdgeCount; ++Index)
        {
            const int32 Next = (Index + 1) % Vertices.Num();
            const FVector Along = Bottoms[Next] - Bottoms[Index];
            const FVector Up = (Tops[Index] - Bottoms[Index]) + (Tops[Next] - Bottoms[Next]);
            const double LengthCm = Along.Size();
            if (LengthCm < 1.0)
            {
                continue;
            }
            const FQuat Rotation = FRotationMatrix::MakeFromXZ(Along, Up).ToQuat();
            const FVector Center = (Bottoms[Index] + Bottoms[Next] + Tops[Index] + Tops[Next]) * 0.25;
            const FVector Scale(LengthCm / ShapeSizeCm, WallThicknessCm / ShapeSizeCm, 0.5 * Up.Size() / ShapeSizeCm);
            AddPart(CubeMesh, WallMaterial, FTransform(Rotation, Center, Scale));
        }
    }
    UE_LOG(LogDroneMarkers, Display, TEXT("Area markers: %d vertices, %d parts"), Vertices.Num(), Parts.Num());
}
