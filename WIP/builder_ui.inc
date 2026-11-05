// --- Graphs (curves like the drone / rates pages of other FPV simulators) ------------------

namespace
{
struct FGraphSeries
{
    TArray<FVector2f> Points; // data units
    FLinearColor Color;
};

struct FGraphData
{
    FString Title;
    FString TopLabel;
    FString BottomLabel;
    TArray<FGraphSeries> Series;
    double XMin = 0.0;
    double XMax = 1.0;
    double YMin = 0.0;
    double YMax = 1.0;
    bool bAxesThroughZero = false;
};

const FLinearColor GraphOrange(1.f, 0.66f, 0.05f);
const FLinearColor GraphBlue(0.35f, 0.65f, 1.f);

class SFpvGraph : public SLeafWidget
{
public:
    SLATE_BEGIN_ARGS(SFpvGraph) {}
    SLATE_ARGUMENT(TFunction<FGraphData()>, Data)
    SLATE_END_ARGS()

    void Construct(const FArguments& InArgs)
    {
        DataFn = InArgs._Data;
    }

    virtual FVector2D ComputeDesiredSize(float) const override
    {
        return FVector2D(170.0, 130.0);
    }

    virtual int32 OnPaint(const FPaintArgs& Args, const FGeometry& AllottedGeometry, const FSlateRect& MyCullingRect,
                          FSlateWindowElementList& OutDrawElements, int32 LayerId, const FWidgetStyle& InWidgetStyle,
                          bool bParentEnabled) const override
    {
        const FGraphData D = DataFn ? DataFn() : FGraphData();
        const FVector2f Size = AllottedGeometry.GetLocalSize();
        const FSlateBrush* White = FCoreStyle::Get().GetBrush("WhiteBrush");
        const TSharedRef<FSlateFontMeasure> Measure = FSlateApplication::Get().GetRenderer()->GetFontMeasureService();
        const FSlateFontInfo TitleFont = FCoreStyle::GetDefaultFontStyle("Regular", 9);
        const FSlateFontInfo ValueFont = FCoreStyle::GetDefaultFontStyle("Bold", 11);

        FSlateDrawElement::MakeBox(OutDrawElements, LayerId, AllottedGeometry.ToPaintGeometry(), White, ESlateDrawEffect::None,
                                   FLinearColor(0.02f, 0.022f, 0.028f, 0.92f));
        const float Left = 8.f, Right = Size.X - 8.f, Top = 22.f, Bottom = Size.Y - 8.f;
        const double SpanX = FMath::Max(D.XMax - D.XMin, 1e-9), SpanY = FMath::Max(D.YMax - D.YMin, 1e-9);
        auto Map = [&](double X, double Y)
        {
            return FVector2f(Left + static_cast<float>((X - D.XMin) / SpanX) * (Right - Left),
                             Bottom - static_cast<float>((Y - D.YMin) / SpanY) * (Bottom - Top));
        };
        auto Lines = [&](const TArray<FVector2f>& Points, const FLinearColor& Color, float Thickness, int32 Layer)
        {
            FSlateDrawElement::MakeLines(OutDrawElements, Layer, AllottedGeometry.ToPaintGeometry(), Points, ESlateDrawEffect::None,
                                         Color, true, Thickness);
        };
        auto Label = [&](const FString& Text, float X, float Y, const FSlateFontInfo& Font, const FLinearColor& Color, bool bRight)
        {
            const FVector2f TextSize = Measure->Measure(Text, Font);
            const float X0 = bRight ? X - TextSize.X : X;
            FSlateDrawElement::MakeText(OutDrawElements, LayerId + 3,
                                        AllottedGeometry.ToPaintGeometry(TextSize, FSlateLayoutTransform(FVector2f(X0, Y))), Text, Font,
                                        ESlateDrawEffect::None, Color);
        };

        const FLinearColor AxisColor(0.55f, 0.55f, 0.6f, 0.8f);
        if (D.bAxesThroughZero)
        {
            Lines({Map(D.XMin, 0.0), Map(D.XMax, 0.0)}, AxisColor, 1.f, LayerId + 1);
            Lines({Map(0.0, D.YMin), Map(0.0, D.YMax)}, AxisColor, 1.f, LayerId + 1);
        }
        else
        {
            Lines({FVector2f(Left, Top), FVector2f(Left, Bottom), FVector2f(Right, Bottom)}, AxisColor, 1.f, LayerId + 1);
        }
        for (const FGraphSeries& S : D.Series)
        {
            TArray<FVector2f> Mapped;
            Mapped.Reserve(S.Points.Num());
            for (const FVector2f& Point : S.Points)
            {
                Mapped.Add(Map(Point.X, Point.Y));
            }
            if (Mapped.Num() >= 2)
            {
                Lines(Mapped, S.Color, 2.f, LayerId + 2);
            }
        }
        Label(D.Title, 8.f, 3.f, TitleFont, FLinearColor::White, false);
        Label(D.TopLabel, Left + 6.f, Top + 2.f, ValueFont, GraphOrange, false);
        Label(D.BottomLabel, Right - 2.f, Bottom - 18.f, TitleFont, FLinearColor(0.75f, 0.75f, 0.8f), true);
        return LayerId + 3;
    }

private:
    TFunction<FGraphData()> DataFn;
};

FGraphData CurveGraph(const FpvBuild::Curve& C)
{
    FGraphData D;
    D.Title = UTF8_TO_TCHAR(C.Title);
    FGraphSeries S;
    S.Color = GraphOrange;
    double MaxY = 0.0;
    for (size_t I = 0; I < C.X.size(); ++I)
    {
        S.Points.Add(FVector2f(static_cast<float>(C.X[I]), static_cast<float>(C.Y[I])));
        MaxY = FMath::Max(MaxY, C.Y[I]);
    }
    D.Series.Add(S);
    if (!C.X.empty())
    {
        D.XMin = C.X.front();
        D.XMax = C.X.back();
        D.TopLabel = FString::Printf(TEXT("%.3g %s"), MaxY, UTF8_TO_TCHAR(C.YUnit));
        D.BottomLabel = FString::Printf(TEXT("%.3g %s at %.3g %s"), C.Y.back(), UTF8_TO_TCHAR(C.YUnit), C.X.back(), UTF8_TO_TCHAR(C.XUnit));
    }
    D.YMax = FMath::Max(MaxY * 1.08, 1e-6);
    return D;
}

FpvBuild::Selection SelectionOf(const FDroneSimSettings& S)
{
    FpvBuild::Selection Sel;
    Sel.FrameIndex = S.FpvFrame;
    Sel.MotorIndex = S.FpvMotor;
    Sel.PropIndex = S.FpvProp;
    Sel.BatteryIndex = S.FpvBattery;
    Sel.FpvIndex = S.FpvCameraSystem;
    return Sel;
}

// Video camera presets: what flight.mp4 / frames.csv / the DroneLocalization export will carry.
struct FCameraPreset
{
    const TCHAR* Name;
    int32 Width;
    int32 Height;
    int32 Fps;
    double SurveyHFovDeg;
    double FpvHFovDeg;
    double FpvUptiltDeg;
};

const FCameraPreset CameraPresets[] = {
    {TEXT("Survey 720p"), 1280, 720, 30, 36.87, 110.0, 30.0}, // = FlightSimulator's camera (8.8 mm sensor, 13.2 mm lens)
    {TEXT("Survey 1080p"), 1920, 1080, 30, 36.87, 110.0, 30.0},
    {TEXT("Survey 4K"), 3840, 2160, 30, 36.87, 110.0, 30.0},
    {TEXT("FPV analog"), 640, 480, 30, 36.87, 118.0, 30.0},
    {TEXT("FPV digital"), 1920, 1080, 60, 36.87, 120.0, 25.0},
    {TEXT("Action cam"), 1920, 1080, 60, 36.87, 86.0, 10.0}, // linear (de-fished) mode
};

// Rates profiles (Betaflight "Actual" rates + throttle curve).
struct FRatesPreset
{
    const TCHAR* Name;
    double RpCenter, RpMax, RpExpo;
    double YawCenter, YawMax, YawExpo;
    double ThrMid, ThrExpo, ThrLimit;
};

const FRatesPreset RatesPresets[] = {
    {TEXT("Super smooth"), 40.0, 400.0, 0.30, 40.0, 350.0, 0.30, 0.50, 0.30, 100.0},
    {TEXT("Smooth"), 60.0, 550.0, 0.45, 60.0, 450.0, 0.45, 0.50, 0.20, 100.0},
    {TEXT("Medium"), 70.0, 670.0, 0.54, 70.0, 670.0, 0.54, 0.50, 0.00, 100.0},
    {TEXT("Fast"), 100.0, 850.0, 0.55, 100.0, 700.0, 0.50, 0.50, 0.00, 100.0},
    {TEXT("Race"), 200.0, 900.0, 0.25, 150.0, 600.0, 0.20, 0.40, 0.00, 100.0},
    {TEXT("Cinematic"), 30.0, 300.0, 0.20, 30.0, 200.0, 0.20, 0.35, 0.40, 85.0},
};

int32 RatesIndexByName(const FString& Name)
{
    for (int32 I = 0; I < UE_ARRAY_COUNT(RatesPresets); ++I)
    {
        if (Name == RatesPresets[I].Name)
        {
            return I;
        }
    }
    return -1;
}

FString RatesDir()
{
    return FPaths::ConvertRelativePathToFull(FPaths::ProjectSavedDir() / TEXT("Rates"));
}

TArray<FString> ListJsonNames(const FString& Dir)
{
    TArray<FString> Files;
    IFileManager::Get().FindFiles(Files, *(Dir / TEXT("*.json")), true, false);
    TArray<FString> Names;
    for (const FString& FoundFile : Files)
    {
        Names.Add(FPaths::GetBaseFilename(FoundFile));
    }
    Names.Sort();
    return Names;
}

TSharedRef<SWidget> ListButton(const FText& Label, TFunction<bool()> IsActive, TFunction<void()> OnClick)
{
    return SNew(SButton)
        .IsFocusable(false)
        .Text(Label)
        .ButtonColorAndOpacity_Lambda([IsActive]()
        {
            return FSlateColor(IsActive() ? FLinearColor(0.85f, 0.55f, 0.02f) : FLinearColor(0.3f, 0.3f, 0.3f));
        })
        .OnClicked_Lambda([OnClick]()
        {
            OnClick();
            return FReply::Handled();
        });
}

// One cell of the stats strip: small grey label over a big orange value.
TSharedRef<SWidget> StatCell(const TCHAR* LabelText, TFunction<FString()> Value)
{
    return SNew(SVerticalBox)
        + SVerticalBox::Slot().AutoHeight()
        [ SNew(STextBlock).Text(FText::FromString(LabelText)).Font(FCoreStyle::GetDefaultFontStyle("Regular", 9))
          .ColorAndOpacity(FLinearColor(0.7f, 0.7f, 0.75f)) ]
        + SVerticalBox::Slot().AutoHeight()
        [ SNew(STextBlock).Font(FCoreStyle::GetDefaultFontStyle("Bold", 13)).ColorAndOpacity(GraphOrange)
          .Text_Lambda([Value]() { return FText::FromString(Value()); }) ];
}
} // namespace

void SDroneSettingsPanel::RefreshBuildCache()
{
    const FpvBuild::Selection Sel = SelectionOf(Settings);
    const int64 Key = ((((static_cast<int64>(Sel.FrameIndex) * 64 + Sel.MotorIndex) * 64 + Sel.PropIndex) * 64 + Sel.BatteryIndex) * 64) +
                      Sel.FpvIndex;
    if (Key != BuildKey)
    {
        BuildKey = Key;
        const FpvBuild::Physical P = FpvBuild::Derive(Sel);
        BuildSummary = FpvBuild::Summarize(Sel, P);
        BuildCurves = FpvBuild::ComputeCurves(P);
    }
    if (ADronePreview* Model = Preview.Get())
    {
        Model->ShowBuild(Sel);
    }
}

TSharedRef<SWidget> SDroneSettingsPanel::BuilderPage()
{
    if (!PreviewBrush.IsValid())
    {
        PreviewBrush = MakeShared<FSlateBrush>();
        PreviewBrush->ImageSize = FVector2D(960.0, 540.0);
        PreviewBrush->DrawAs = ESlateBrushDrawType::Image;
    }
    TSharedRef<SVerticalBox> Ready = SNew(SVerticalBox);
    for (int32 Index = 0; Index < FpvBuild::CountOf(FpvBuild::kPresets); ++Index)
    {
        Ready->AddSlot().AutoHeight().Padding(FMargin(0.f, 1.f))
        [
            SNew(SBox)
            .ToolTipText(FText::FromString(UTF8_TO_TCHAR(FpvBuild::kPresets[Index].Description)))
            [
                ListButton(FText::FromString(UTF8_TO_TCHAR(FpvBuild::kPresets[Index].Name)),
                           [this, Index]() { return PresetIndexByName(Settings.FpvDroneName) == Index; },
                           [this, Index]() { SelectPresetDrone(Index); })
            ]
        ];
    }
    auto Graph = [this](TFunction<const FpvBuild::Curve&(const FpvBuild::Curves&)> Pick) -> TSharedRef<SWidget>
    {
        return SNew(SBox).HeightOverride(150.f)
        [
            SNew(SFpvGraph).Data([this, Pick]()
            {
                RefreshBuildCache();
                return CurveGraph(Pick(BuildCurves));
            })
        ];
    };
    auto Stat = [this](const TCHAR* LabelText, TFunction<FString(const FDroneSimSettings&, const FpvBuild::Summary&)> Fn)
    {
        return StatCell(LabelText, [this, Fn]()
        {
            RefreshBuildCache();
            return Fn(Settings, BuildSummary);
        });
    };
    using namespace FpvBuild;
    TSharedRef<SWidget> Page =
        SNew(SVerticalBox)
        + SVerticalBox::Slot().AutoHeight()[ PageTitle(Txt(TEXT("DRONE BUILDER"))) ]
        + SVerticalBox::Slot().FillHeight(1.f).Padding(FMargin(0.f, 8.f))
        [
            SNew(SHorizontalBox)
            // 1) The list of drones.
            + SHorizontalBox::Slot().FillWidth(0.2f).Padding(FMargin(0.f, 0.f, 10.f, 0.f))
            [
                SNew(SScrollBox)
                + SScrollBox::Slot()[ Header(Txt(TEXT("Ready-made drones"))) ]
                + SScrollBox::Slot().Padding(FMargin(0.f, 4.f, 0.f, 10.f))[ Ready ]
                + SScrollBox::Slot()[ Header(Txt(TEXT("My drones"))) ]
                + SScrollBox::Slot().Padding(FMargin(0.f, 4.f))[ SAssignNew(DroneListBox, SVerticalBox) ]
            ]
            // 2) 3D model, stats, graphs.
            + SHorizontalBox::Slot().FillWidth(0.47f).Padding(FMargin(0.f, 0.f, 10.f, 0.f))
            [
                SNew(SVerticalBox)
                + SVerticalBox::Slot().AutoHeight()
                [
                    SNew(SBorder)
                    .BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush"))
                    .BorderBackgroundColor(FLinearColor(0.035f, 0.037f, 0.045f))
                    .Padding(0.f)
                    .OnMouseButtonDown_Lambda([](const FGeometry&, const FPointerEvent&) { return FReply::Handled(); })
                    .OnMouseMove_Lambda([this](const FGeometry&, const FPointerEvent& Event)
                    {
                        if (Event.IsMouseButtonDown(EKeys::LeftMouseButton) && Preview.IsValid())
                        {
                            Preview->Orbit(Event.GetCursorDelta().X * 0.4f, -Event.GetCursorDelta().Y * 0.3f);
                        }
                        return FReply::Handled();
                    })
                    [
                        SNew(SBox).HeightOverride(380.f)
                        [ SNew(SImage).Image(PreviewBrush.Get()) ]
                    ]
                ]
                + SVerticalBox::Slot().AutoHeight().Padding(FMargin(0.f, 4.f))
                [
                    SNew(SHorizontalBox)
                    + SHorizontalBox::Slot().FillWidth(1.f).VAlign(VAlign_Center)
                    [ Hint(Txt(TEXT("Drag the model to turn it."))) ]
                    + SHorizontalBox::Slot().AutoWidth().Padding(2.f)
                    [ ActionButton(Txt(TEXT("Zoom +")), [this]() { if (Preview.IsValid()) { Preview->Zoom(0.85f); } }) ]
                    + SHorizontalBox::Slot().AutoWidth().Padding(2.f)
                    [ ActionButton(Txt(TEXT("Zoom -")), [this]() { if (Preview.IsValid()) { Preview->Zoom(1.18f); } }) ]
                    + SHorizontalBox::Slot().AutoWidth().Padding(2.f)
                    [ ActionButton(Txt(TEXT("Reset view")), [this]() { if (Preview.IsValid()) { Preview->ResetView(); } }) ]
                ]
                // Stats strip.
                + SVerticalBox::Slot().AutoHeight().Padding(FMargin(0.f, 4.f))
                [
                    SNew(SBorder)
                    .BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush"))
                    .BorderBackgroundColor(FLinearColor(0.9f, 0.9f, 0.92f, 0.08f))
                    .Padding(FMargin(10.f, 6.f))
                    [
                        SNew(SVerticalBox)
                        + SVerticalBox::Slot().AutoHeight()
                        [
                            SNew(SHorizontalBox)
                            + SHorizontalBox::Slot().FillWidth(1.f)[ Stat(TEXT("Props"), [](const FDroneSimSettings& S, const Summary&)
                            {
                                const Prop& Pr = kProps[Pick(S.FpvProp, CountOf(kProps))];
                                return FString::Printf(TEXT("%.1f x %.1f x%d"), Pr.DiameterIn, Pr.PitchIn, Pr.Blades);
                            }) ]
                            + SHorizontalBox::Slot().FillWidth(1.f)[ Stat(TEXT("Battery"), [](const FDroneSimSettings& S, const Summary&)
                            {
                                const Battery& B = kBatteries[Pick(S.FpvBattery, CountOf(kBatteries))];
                                return FString::Printf(TEXT("%dS %.0f mAh"), B.Cells, B.CapacityMah);
                            }) ]
                            + SHorizontalBox::Slot().FillWidth(1.f)[ Stat(TEXT("Motors, KV"), [](const FDroneSimSettings& S, const Summary&)
                            {
                                return FString::Printf(TEXT("%.0f"), kMotors[Pick(S.FpvMotor, CountOf(kMotors))].Kv);
                            }) ]
                            + SHorizontalBox::Slot().FillWidth(1.f)[ Stat(TEXT("Weight, kg"), [](const FDroneSimSettings&, const Summary& Sm)
                            {
                                return FString::Printf(TEXT("%.3f"), Sm.MassG / 1000.0);
                            }) ]
                            + SHorizontalBox::Slot().FillWidth(1.f)[ Stat(TEXT("Flight time (hover)"), [](const FDroneSimSettings&, const Summary& Sm)
                            {
                                return FString::Printf(TEXT("%.1f min"), Sm.HoverMinutes);
                            }) ]
                        ]
                        + SVerticalBox::Slot().AutoHeight().Padding(FMargin(0.f, 6.f, 0.f, 0.f))
                        [
                            SNew(SHorizontalBox)
                            + SHorizontalBox::Slot().FillWidth(1.f)[ Stat(TEXT("Max speed, km/h"), [this](const FDroneSimSettings&, const Summary&)
                            {
                                return BuildCurves.VelocityTime.Y.empty() ? FString(TEXT("-"))
                                                                          : FString::Printf(TEXT("%.0f"), BuildCurves.VelocityTime.Y.back());
                            }) ]
                            + SHorizontalBox::Slot().FillWidth(1.f)[ Stat(TEXT("Max RPM"), [](const FDroneSimSettings&, const Summary& Sm)
                            {
                                return FString::Printf(TEXT("%.0f"), Sm.MaxRpm);
                            }) ]
                            + SHorizontalBox::Slot().FillWidth(1.f)[ Stat(TEXT("Thrust, kg"), [](const FDroneSimSettings&, const Summary& Sm)
                            {
                                return FString::Printf(TEXT("%.2f"), Sm.MaxThrustKg);
                            }) ]
                            + SHorizontalBox::Slot().FillWidth(1.f)[ Stat(TEXT("Thrust / weight"), [](const FDroneSimSettings&, const Summary& Sm)
                            {
                                return FString::Printf(TEXT("%.1f"), Sm.ThrustToWeight);
                            }) ]
                            + SHorizontalBox::Slot().FillWidth(1.f)[ Stat(TEXT("Hover throttle"), [](const FDroneSimSettings&, const Summary& Sm)
                            {
                                return FString::Printf(TEXT("%.0f %%"), Sm.HoverThrottle * 100.0);
                            }) ]
                        ]
                    ]
                ]
                // Graphs.
                + SVerticalBox::Slot().AutoHeight().Padding(FMargin(0.f, 4.f))
                [
                    SNew(SHorizontalBox)
                    + SHorizontalBox::Slot().FillWidth(1.f).Padding(2.f)[ Graph([](const Curves& C) -> const Curve& { return C.DropTime; }) ]
                    + SHorizontalBox::Slot().FillWidth(1.f).Padding(2.f)[ Graph([](const Curves& C) -> const Curve& { return C.ThrustRpm; }) ]
                    + SHorizontalBox::Slot().FillWidth(1.f).Padding(2.f)[ Graph([](const Curves& C) -> const Curve& { return C.DragVelocity; }) ]
                    + SHorizontalBox::Slot().FillWidth(1.f).Padding(2.f)[ Graph([](const Curves& C) -> const Curve& { return C.VelocityTime; }) ]
                    + SHorizontalBox::Slot().FillWidth(1.f).Padding(2.f)[ Graph([](const Curves& C) -> const Curve& { return C.ThrustTime; }) ]
                ]
                + SVerticalBox::Slot().AutoHeight()
                [
                    SNew(STextBlock).Font(PanelFont(9)).AutoWrapText(true).ColorAndOpacity(FLinearColor(1.f, 0.55f, 0.3f))
                    .Text_Lambda([this]()
                    {
                        RefreshBuildCache();
                        return BuildSummary.Warnings.empty() ? FText::GetEmpty()
                                                             : FText::FromString(FString(TEXT("Warning: ")) + UTF8_TO_TCHAR(BuildSummary.Warnings.c_str()));
                    })
                ]
            ]
            // 3) Name, parts, camera.
            + SHorizontalBox::Slot().FillWidth(0.33f)
            [
                SNew(SScrollBox)
                + SScrollBox::Slot()[ TextRow(Txt(TEXT("Name")), &Settings.FpvDroneName) ]
                + SScrollBox::Slot().Padding(FMargin(0.f, 4.f))
                [
                    SNew(STextBlock).Font(PanelFont(9)).AutoWrapText(true).ColorAndOpacity(FLinearColor(0.7f, 0.7f, 0.7f))
                    .Text_Lambda([this]()
                    {
                        const int32 Index = PresetIndexByName(Settings.FpvDroneName);
                        if (Index < 0)
                        {
                            return Txt(TEXT("Your own build."));
                        }
                        const FpvBuild::Preset& P = FpvBuild::kPresets[Index];
                        FString Text = UTF8_TO_TCHAR(P.Description);
                        if (!SameParts(Settings, P))
                        {
                            Text += TEXT("  (modified: save it as your drone)");
                        }
                        return FText::FromString(Text);
                    })
                ]
                + SScrollBox::Slot()[ BuildSection() ]
                + SScrollBox::Slot()[ CameraSection() ]
                + SScrollBox::Slot().Padding(FMargin(0.f, 10.f, 0.f, 0.f))
                [
                    SNew(SHorizontalBox)
                    + SHorizontalBox::Slot().FillWidth(1.f).Padding(FMargin(0.f, 0.f, 4.f, 0.f))
                    [ ActionButton(Txt(TEXT("Save as my drone")), [this]() { SaveUserDrone(); }) ]
                    + SHorizontalBox::Slot().FillWidth(1.f).Padding(FMargin(4.f, 0.f, 0.f, 0.f))
                    [ ActionButton(Txt(TEXT("Delete my drone")), [this]() { DeleteUserDrone(); }) ]
                ]
                + SScrollBox::Slot().Padding(FMargin(0.f, 4.f))
                [
                    SNew(STextBlock).Font(PanelFont(9)).AutoWrapText(true).ColorAndOpacity(FLinearColor(0.7f, 0.7f, 0.7f))
                    .Text_Lambda([this]() { return FText::FromString(DroneNote); })
                ]
            ]
        ]
        + SVerticalBox::Slot().AutoHeight().Padding(FMargin(0.f, 6.f, 0.f, 0.f))
        [
            SNew(SHorizontalBox)
            + SHorizontalBox::Slot().FillWidth(1.f).Padding(FMargin(0.f, 0.f, 4.f, 0.f))
            [ BigButton(Txt(TEXT("Fly this drone (save)")), [this]() { LeaveBuilder(/*bSave*/ true); }) ]
            + SHorizontalBox::Slot().FillWidth(1.f).Padding(FMargin(4.f, 0.f, 0.f, 0.f))
            [ BigButton(Txt(TEXT("Cancel")), [this]() { LeaveBuilder(/*bSave*/ false); }) ]
        ];
    RefreshDroneList();
    return Page;
}

// Camera for the localizer: resolution, frame rate, field of view, and the pinhole intrinsics they give.
TSharedRef<SWidget> SDroneSettingsPanel::CameraSection()
{
    const FMargin RowPad(0.f, 2.f);
    TSharedRef<SHorizontalBox> Presets = SNew(SHorizontalBox);
    for (int32 Index = 0; Index < UE_ARRAY_COUNT(CameraPresets); ++Index)
    {
        Presets->AddSlot().FillWidth(1.f).Padding(1.f)
        [
            ListButton(FText::FromString(CameraPresets[Index].Name),
                       [this, Index]()
                       {
                           const FCameraPreset& C = CameraPresets[Index];
                           return Settings.CamWidthPx == C.Width && Settings.CamHeightPx == C.Height && Settings.CamFps == C.Fps &&
                                  FMath::IsNearlyEqual(Settings.FpvCameraFovDeg, C.FpvHFovDeg, 0.01);
                       },
                       [this, Index]()
                       {
                           const FCameraPreset& C = CameraPresets[Index];
                           Settings.CamWidthPx = C.Width;
                           Settings.CamHeightPx = C.Height;
                           Settings.CamFps = C.Fps;
                           Settings.CamSurveyHFovDeg = C.SurveyHFovDeg;
                           Settings.FpvCameraFovDeg = C.FpvHFovDeg;
                           Settings.FpvCameraUptiltDeg = C.FpvUptiltDeg;
                       })
        ];
    }
    return SNew(SVerticalBox)
        + SVerticalBox::Slot().AutoHeight().Padding(FMargin(0.f, 12.f, 0.f, 2.f))[ Header(Txt(TEXT("Camera (what the localizer gets)"))) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ Hint(Txt(TEXT("Goes into flight.mp4, frames.csv (hfov_deg) and the DroneLocalization export. The render is a pinhole camera: ")
                   TEXT("fx = fy = W / (2 tan(HFOV/2)), cx = W/2, cy = H/2, no lens distortion. Resolution and frame rate change between flights."))) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)[ Presets ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)[ IntRow(Txt(TEXT("Width, px")), &Settings.CamWidthPx, 64, 3840) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)[ IntRow(Txt(TEXT("Height, px")), &Settings.CamHeightPx, 64, 2160) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)[ IntRow(Txt(TEXT("Frame rate, fps (frame_step in DroneLocalization counts these)")), &Settings.CamFps, 1, 120) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ NumberRow(Txt(TEXT("Survey camera HFOV, deg (36.87 = FlightSimulator)")), &Settings.CamSurveyHFovDeg, 5.0, 170.0, 0.5, 2) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ NumberRow(Txt(TEXT("FPV camera HFOV, deg")), &Settings.FpvCameraFovDeg, 40.0, 170.0, 1.0, 1) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ NumberRow(Txt(TEXT("FPV camera uptilt, deg")), &Settings.FpvCameraUptiltDeg, -30.0, 80.0, 1.0, 0) ]
        + SVerticalBox::Slot().AutoHeight().Padding(FMargin(0.f, 4.f))
        [
            SNew(STextBlock).Font(FCoreStyle::GetDefaultFontStyle("Mono", 9)).AutoWrapText(true)
            .ColorAndOpacity(FLinearColor(0.85f, 0.95f, 1.f))
            .Text_Lambda([this]()
            {
                const double W = FMath::Max(Settings.CamWidthPx, 1), H = FMath::Max(Settings.CamHeightPx, 1);
                auto Line = [W, H](const TCHAR* Name, double HFovDeg)
                {
                    const double Fx = 0.5 * W / FMath::Tan(FMath::DegreesToRadians(HFovDeg) * 0.5);
                    const double VFovDeg = FMath::RadiansToDegrees(2.0 * FMath::Atan(0.5 * H / Fx));
                    const double FocalMm = Fx * 36.0 / W; // the export's nominal 36 mm sensor
                    return FString::Printf(TEXT("%s: fx = fy = %.1f px, cx = %.1f, cy = %.1f, VFOV %.1f deg, focal %.1f mm (36 mm sensor)"),
                                           Name, Fx, 0.5 * W, 0.5 * H, VFovDeg, FocalMm);
                };
                const double SurveyFx = 0.5 * W / FMath::Tan(FMath::DegreesToRadians(Settings.CamSurveyHFovDeg) * 0.5);
                const double GsdCm = 100.0 * 100.0 / SurveyFx; // nadir, 100 m above ground
                return FText::FromString(FString::Printf(TEXT("%dx%d @ %d fps, aspect %.3f\n%s\n  ground sample distance at 100 m nadir: %.1f cm/px\n%s"),
                                                         Settings.CamWidthPx, Settings.CamHeightPx, Settings.CamFps, W / H,
                                                         *Line(TEXT("Survey"), Settings.CamSurveyHFovDeg), GsdCm,
                                                         *Line(TEXT("FPV"), Settings.FpvCameraFovDeg)));
            })
        ];
}

// --- Rates and throttle page -----------------------------------------------------------------

void SDroneSettingsPanel::EnterRates()
{
    CommitChoices();
    SettingsBackup = Settings;
    RatesNote.Reset();
    RefreshRatesList();
    ActivePage = PageRates;
}

void SDroneSettingsPanel::LeaveRates(bool bSave)
{
    if (bSave)
    {
        Fire(EDroneMenuAction::ApplySettings);
    }
    else
    {
        const FDroneSimSettings Edited = Settings;
        Settings = SettingsBackup;
        DroneMaps::CopyMapFields(Edited, Settings);
        SyncChoices();
    }
    ActivePage = RootPage();
}

void SDroneSettingsPanel::SelectPresetRates(int32 Index)
{
    const FRatesPreset& R = RatesPresets[FMath::Clamp(Index, 0, static_cast<int32>(UE_ARRAY_COUNT(RatesPresets)) - 1)];
    Settings.FpvRollPitchCenterRateDps = R.RpCenter;
    Settings.FpvRollPitchMaxRateDps = R.RpMax;
    Settings.FpvRollPitchExpo = R.RpExpo;
    Settings.FpvYawCenterRateDps = R.YawCenter;
    Settings.FpvYawMaxRateDps = R.YawMax;
    Settings.FpvYawExpo = R.YawExpo;
    Settings.FpvThrottleMid = R.ThrMid;
    Settings.FpvThrottleExpo = R.ThrExpo;
    Settings.FpvThrottleLimitPct = R.ThrLimit;
    Settings.FpvRatesName = R.Name;
    RatesNote.Reset();
}

void SDroneSettingsPanel::SelectUserRates(const FString& Entry)
{
    FString Text;
    TSharedPtr<FJsonObject> O;
    if (!FFileHelper::LoadFileToString(Text, *(RatesDir() / (Entry + TEXT(".json")))) ||
        !FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Text), O) || !O.IsValid())
    {
        RatesNote = FString::Printf(TEXT("Could not read %s.json"), *Entry);
        return;
    }
    O->TryGetNumberField(TEXT("roll_pitch_center_dps"), Settings.FpvRollPitchCenterRateDps);
    O->TryGetNumberField(TEXT("roll_pitch_max_dps"), Settings.FpvRollPitchMaxRateDps);
    O->TryGetNumberField(TEXT("roll_pitch_expo"), Settings.FpvRollPitchExpo);
    O->TryGetNumberField(TEXT("yaw_center_dps"), Settings.FpvYawCenterRateDps);
    O->TryGetNumberField(TEXT("yaw_max_dps"), Settings.FpvYawMaxRateDps);
    O->TryGetNumberField(TEXT("yaw_expo"), Settings.FpvYawExpo);
    O->TryGetNumberField(TEXT("throttle_mid"), Settings.FpvThrottleMid);
    O->TryGetNumberField(TEXT("throttle_expo"), Settings.FpvThrottleExpo);
    O->TryGetNumberField(TEXT("throttle_limit_pct"), Settings.FpvThrottleLimitPct);
    if (!O->TryGetStringField(TEXT("name"), Settings.FpvRatesName) || Settings.FpvRatesName.IsEmpty())
    {
        Settings.FpvRatesName = Entry;
    }
    RatesNote = FString::Printf(TEXT("Loaded \"%s\"."), *Settings.FpvRatesName);
}

void SDroneSettingsPanel::SaveUserRates()
{
    FString Name = Settings.FpvRatesName.TrimStartAndEnd();
    if (Name.IsEmpty() || RatesIndexByName(Name) >= 0)
    {
        Name = Name.IsEmpty() ? FString(TEXT("My rates")) : Name + TEXT(" (mine)");
        Settings.FpvRatesName = Name;
    }
    TSharedRef<FJsonObject> O = MakeShared<FJsonObject>();
    O->SetStringField(TEXT("name"), Name);
    O->SetNumberField(TEXT("roll_pitch_center_dps"), Settings.FpvRollPitchCenterRateDps);
    O->SetNumberField(TEXT("roll_pitch_max_dps"), Settings.FpvRollPitchMaxRateDps);
    O->SetNumberField(TEXT("roll_pitch_expo"), Settings.FpvRollPitchExpo);
    O->SetNumberField(TEXT("yaw_center_dps"), Settings.FpvYawCenterRateDps);
    O->SetNumberField(TEXT("yaw_max_dps"), Settings.FpvYawMaxRateDps);
    O->SetNumberField(TEXT("yaw_expo"), Settings.FpvYawExpo);
    O->SetNumberField(TEXT("throttle_mid"), Settings.FpvThrottleMid);
    O->SetNumberField(TEXT("throttle_expo"), Settings.FpvThrottleExpo);
    O->SetNumberField(TEXT("throttle_limit_pct"), Settings.FpvThrottleLimitPct);
    FString Text;
    FJsonSerializer::Serialize(O, TJsonWriterFactory<>::Create(&Text));
    IFileManager::Get().MakeDirectory(*RatesDir(), true);
    const FString Path = RatesDir() / (DroneMaps::FileNameFor(Name) + TEXT(".json"));
    RatesNote = FFileHelper::SaveStringToFile(Text, *Path) ? FString::Printf(TEXT("Saved %s"), *Path)
                                                          : FString::Printf(TEXT("Could not write %s"), *Path);
    RefreshRatesList();
}

void SDroneSettingsPanel::DeleteUserRates()
{
    const FString Entry = DroneMaps::FileNameFor(Settings.FpvRatesName);
    const FString Path = RatesDir() / (Entry + TEXT(".json"));
    if (!IFileManager::Get().FileExists(*Path))
    {
        RatesNote = TEXT("Ready-made profiles can't be deleted; pick one of yours.");
        return;
    }
    RatesNote = IFileManager::Get().Delete(*Path, false, false, true) ? FString::Printf(TEXT("Deleted \"%s\"."), *Entry)
                                                                      : FString::Printf(TEXT("Could not delete %s"), *Path);
    RefreshRatesList();
}

void SDroneSettingsPanel::RefreshRatesList()
{
    if (!RatesListBox.IsValid())
    {
        return;
    }
    RatesListBox->ClearChildren();
    const TArray<FString> Names = ListJsonNames(RatesDir());
    if (Names.Num() == 0)
    {
        RatesListBox->AddSlot().AutoHeight()[ Hint(Txt(TEXT("None yet: change the rates and press \"Save as my profile\"."))) ];
    }
    for (const FString& Entry : Names)
    {
        RatesListBox->AddSlot().AutoHeight().Padding(FMargin(0.f, 1.f))
        [
            ListButton(FText::FromString(Entry), [this, Entry]() { return DroneMaps::FileNameFor(Settings.FpvRatesName) == Entry; },
                       [this, Entry]() { SelectUserRates(Entry); })
        ];
    }
}

TSharedRef<SWidget> SDroneSettingsPanel::RatesPage()
{
    const FMargin RowPad(0.f, 2.f);
    TSharedRef<SVerticalBox> Ready = SNew(SVerticalBox);
    for (int32 Index = 0; Index < UE_ARRAY_COUNT(RatesPresets); ++Index)
    {
        Ready->AddSlot().AutoHeight().Padding(FMargin(0.f, 1.f))
        [
            ListButton(FText::FromString(RatesPresets[Index].Name), [this, Index]() { return RatesIndexByName(Settings.FpvRatesName) == Index; },
                       [this, Index]() { SelectPresetRates(Index); })
        ];
    }
    auto RatesOf = [](double Center, double Max, double Expo)
    {
        FpvModel::Rates R;
        R.CenterDps = Center;
        R.MaxDps = Max;
        R.Expo = Expo;
        return R;
    };
    TSharedRef<SWidget> Page =
        SNew(SVerticalBox)
        + SVerticalBox::Slot().AutoHeight()[ PageTitle(Txt(TEXT("RATES AND THROTTLE"))) ]
        + SVerticalBox::Slot().FillHeight(1.f).Padding(FMargin(0.f, 8.f))
        [
            SNew(SHorizontalBox)
            // Profiles.
            + SHorizontalBox::Slot().FillWidth(0.24f).Padding(FMargin(0.f, 0.f, 12.f, 0.f))
            [
                SNew(SScrollBox)
                + SScrollBox::Slot()[ Header(Txt(TEXT("Rates profiles"))) ]
                + SScrollBox::Slot().Padding(FMargin(0.f, 4.f, 0.f, 10.f))[ Ready ]
                + SScrollBox::Slot()[ Header(Txt(TEXT("My profiles"))) ]
                + SScrollBox::Slot().Padding(FMargin(0.f, 4.f))[ SAssignNew(RatesListBox, SVerticalBox) ]
                + SScrollBox::Slot().Padding(FMargin(0.f, 10.f, 0.f, 2.f))[ TextRow(Txt(TEXT("Name")), &Settings.FpvRatesName) ]
                + SScrollBox::Slot().Padding(RowPad)[ ActionButton(Txt(TEXT("Save as my profile")), [this]() { SaveUserRates(); }) ]
                + SScrollBox::Slot().Padding(RowPad)[ ActionButton(Txt(TEXT("Delete my profile")), [this]() { DeleteUserRates(); }) ]
                + SScrollBox::Slot().Padding(RowPad)
                [
                    SNew(STextBlock).Font(PanelFont(9)).AutoWrapText(true).ColorAndOpacity(FLinearColor(0.7f, 0.7f, 0.7f))
                    .Text_Lambda([this]() { return FText::FromString(RatesNote); })
                ]
            ]
            // Rates.
            + SHorizontalBox::Slot().FillWidth(0.42f).Padding(FMargin(0.f, 0.f, 12.f, 0.f))
            [
                SNew(SScrollBox)
                + SScrollBox::Slot()[ Header(Txt(TEXT("Rates (Betaflight \"Actual\")"))) ]
                + SScrollBox::Slot().Padding(RowPad)
                [ Hint(Txt(TEXT("Centre = deg/s near the stick centre (sensitivity), max = deg/s at full stick, expo = how long the curve stays soft."))) ]
                + SScrollBox::Slot().Padding(RowPad)[ NumberRow(Txt(TEXT("Roll / pitch: centre, deg/s")), &Settings.FpvRollPitchCenterRateDps, 10.0, 2000.0, 5.0, 0) ]
                + SScrollBox::Slot().Padding(RowPad)[ NumberRow(Txt(TEXT("Roll / pitch: max, deg/s")), &Settings.FpvRollPitchMaxRateDps, 10.0, 2000.0, 10.0, 0) ]
                + SScrollBox::Slot().Padding(RowPad)[ NumberRow(Txt(TEXT("Roll / pitch: expo")), &Settings.FpvRollPitchExpo, 0.0, 1.0, 0.01, 2) ]
                + SScrollBox::Slot().Padding(RowPad)[ NumberRow(Txt(TEXT("Yaw: centre, deg/s")), &Settings.FpvYawCenterRateDps, 10.0, 2000.0, 5.0, 0) ]
                + SScrollBox::Slot().Padding(RowPad)[ NumberRow(Txt(TEXT("Yaw: max, deg/s")), &Settings.FpvYawMaxRateDps, 10.0, 2000.0, 10.0, 0) ]
                + SScrollBox::Slot().Padding(RowPad)[ NumberRow(Txt(TEXT("Yaw: expo")), &Settings.FpvYawExpo, 0.0, 1.0, 0.01, 2) ]
                + SScrollBox::Slot().Padding(FMargin(0.f, 8.f, 0.f, 0.f))
                [
                    SNew(SBox).HeightOverride(330.f)
                    [
                        SNew(SFpvGraph).Data([this, RatesOf]()
                        {
                            const FpvModel::Rates Rp = RatesOf(Settings.FpvRollPitchCenterRateDps, Settings.FpvRollPitchMaxRateDps, Settings.FpvRollPitchExpo);
                            const FpvModel::Rates Yw = RatesOf(Settings.FpvYawCenterRateDps, Settings.FpvYawMaxRateDps, Settings.FpvYawExpo);
                            FGraphData D;
                            D.Title = TEXT("Rate | Stick   (blue: roll / pitch, orange: yaw)");
                            FGraphSeries A, B;
                            A.Color = GraphBlue;
                            B.Color = GraphOrange;
                            for (int32 I = 0; I <= 100; ++I)
                            {
                                const double X = -1.0 + 2.0 * I / 100.0;
                                A.Points.Add(FVector2f(static_cast<float>(X), static_cast<float>(FpvModel::ActualRateDps(X, Rp))));
                                B.Points.Add(FVector2f(static_cast<float>(X), static_cast<float>(FpvModel::ActualRateDps(X, Yw))));
                            }
                            const double Peak = FMath::Max3(FpvModel::ActualRateDps(1.0, Rp), FpvModel::ActualRateDps(1.0, Yw), 1.0);
                            D.Series = {A, B};
                            D.XMin = -1.0;
                            D.XMax = 1.0;
                            D.YMin = -Peak * 1.05;
                            D.YMax = Peak * 1.05;
                            D.bAxesThroughZero = true;
                            D.TopLabel = FString::Printf(TEXT("%.0f / %.0f deg/s"), FpvModel::ActualRateDps(1.0, Rp), FpvModel::ActualRateDps(1.0, Yw));
                            D.BottomLabel = TEXT("stick -1 .. 1");
                            return D;
                        })
                    ]
                ]
            ]
            // Throttle.
            + SHorizontalBox::Slot().FillWidth(0.34f)
            [
                SNew(SScrollBox)
                + SScrollBox::Slot()[ Header(Txt(TEXT("Throttle"))) ]
                + SScrollBox::Slot().Padding(RowPad)
                [ Hint(Txt(TEXT("Like Betaflight thr_mid / thr_expo: the curve flattens around the mid point (finer hover); the limit scales the whole range."))) ]
                + SScrollBox::Slot().Padding(RowPad)[ NumberRow(Txt(TEXT("Throttle mid")), &Settings.FpvThrottleMid, 0.05, 0.95, 0.01, 2) ]
                + SScrollBox::Slot().Padding(RowPad)[ NumberRow(Txt(TEXT("Throttle expo")), &Settings.FpvThrottleExpo, 0.0, 1.0, 0.01, 2) ]
                + SScrollBox::Slot().Padding(RowPad)[ NumberRow(Txt(TEXT("Throttle limit, %")), &Settings.FpvThrottleLimitPct, 10.0, 100.0, 1.0, 0) ]
                + SScrollBox::Slot().Padding(FMargin(0.f, 8.f, 0.f, 0.f))
                [
                    SNew(SBox).HeightOverride(330.f)
                    [
                        SNew(SFpvGraph).Data([this]()
                        {
                            FGraphData D;
                            D.Title = TEXT("Throttle out | Stick");
                            FGraphSeries Linear, Curve;
                            Linear.Color = FLinearColor(0.4f, 0.4f, 0.45f);
                            Curve.Color = GraphOrange;
                            Linear.Points = {FVector2f(0.f, 0.f), FVector2f(1.f, 1.f)};
                            for (int32 I = 0; I <= 100; ++I)
                            {
                                const double X = I / 100.0;
                                Curve.Points.Add(FVector2f(static_cast<float>(X),
                                                           static_cast<float>(FpvBuild::ThrottleCurve(X, Settings.FpvThrottleMid, Settings.FpvThrottleExpo,
                                                                                                      Settings.FpvThrottleLimitPct))));
                            }
                            D.Series = {Linear, Curve};
                            D.YMax = 1.02;
                            D.TopLabel = FString::Printf(TEXT("max %.0f %%"), Settings.FpvThrottleLimitPct);
                            D.BottomLabel = TEXT("stick 0 .. 1");
                            return D;
                        })
                    ]
                ]
            ]
        ]
        + SVerticalBox::Slot().AutoHeight().Padding(FMargin(0.f, 6.f, 0.f, 0.f))
        [
            SNew(SHorizontalBox)
            + SHorizontalBox::Slot().FillWidth(1.f).Padding(FMargin(0.f, 0.f, 4.f, 0.f))
            [ BigButton(Txt(TEXT("Save")), [this]() { LeaveRates(/*bSave*/ true); }) ]
            + SHorizontalBox::Slot().FillWidth(1.f).Padding(FMargin(4.f, 0.f, 0.f, 0.f))
            [ BigButton(Txt(TEXT("Cancel")), [this]() { LeaveRates(/*bSave*/ false); }) ]
        ];
    RefreshRatesList();
    return Page;
}

