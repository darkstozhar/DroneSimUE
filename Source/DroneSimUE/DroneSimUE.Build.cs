// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

public class DroneSimUE : ModuleRules
{
	public DroneSimUE(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

		// Lets sources include module headers as "Sim/....h".
		PrivateIncludePaths.Add(ModuleDirectory);

		PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine", "InputCore", "CesiumRuntime" });

		// UDP sockets for the MAVLink telemetry output.
		PrivateDependencyModuleNames.AddRange(new string[] { "Sockets", "Networking" });

		// Video capture: GPU readback of the camera's render target.
		PrivateDependencyModuleNames.AddRange(new string[] { "RenderCore", "RHI" });

		// In-game settings panel and status line (Slate) and the settings file (JSON).
		PrivateDependencyModuleNames.AddRange(new string[] { "Slate", "SlateCore", "Json" });

		// Imagery kept on disk: loopback tile server (HTTPServer) and the downloads (HTTP).
		PrivateDependencyModuleNames.AddRange(new string[] { "HTTP", "HTTPServer" });

		// Uncomment if you are using Slate UI
		// PrivateDependencyModuleNames.AddRange(new string[] { "Slate", "SlateCore" });

		// Uncomment if you are using online features
		// PrivateDependencyModuleNames.Add("OnlineSubsystem");

		// To include OnlineSubsystemSteam, add it to the plugins section in your uproject file with the Enabled attribute set to true
	}
}
