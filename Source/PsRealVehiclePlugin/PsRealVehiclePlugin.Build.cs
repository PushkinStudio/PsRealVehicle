// Copyright 2016 Pushkin Studio. All Rights Reserved.

using System.IO;

namespace UnrealBuildTool.Rules
{
	public class PsRealVehiclePlugin : ModuleRules
	{
		public PsRealVehiclePlugin(ReadOnlyTargetRules Target) : base(Target)
		{
            PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;

            PrivateIncludePaths.AddRange(
				new string[] {
					"PsRealVehiclePlugin/Private",
					// ... add other private include paths required here ...
				});

			PublicDependencyModuleNames.AddRange(
				new string[]
				{
					"Core",
					"CoreUObject",
					"Engine",
					"PhysicsCore"
					// ... add other public dependencies that you statically link with here ...
				});

            PrivateDependencyModuleNames.AddRange(
                new string[]
				{
					"AnimGraphRuntime"
				});
		}
	}
}