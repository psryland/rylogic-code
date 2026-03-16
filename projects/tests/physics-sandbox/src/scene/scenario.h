#pragma once

namespace physics_sandbox
{
	// Test scenarios for validating physics behaviour progressively.
	// Each scenario configures two bodies with specific initial conditions.
	enum class EScenario : int
	{
		Sandbox = 0,          // Free-form sandbox (default)
		HeadOnEqualMass = 1,  // Two equal-mass boxes, head-on along X
		HeadOnDiffMass = 2,   // Mass 10 vs Mass 5, head-on along X
		StationaryTarget = 3, // Moving box hits stationary box
		OffCenter = 4,        // Off-center hit (induces rotation)
		Oblique = 5,          // Bodies approaching at an angle
	};

	inline char const* ScenarioName(EScenario s)
	{
		switch (s)
		{
			case EScenario::Sandbox:          return "Sandbox";
			case EScenario::HeadOnEqualMass:  return "Head-On Equal Mass";
			case EScenario::HeadOnDiffMass:   return "Head-On Diff Mass";
			case EScenario::StationaryTarget: return "Stationary Target";
			case EScenario::OffCenter:        return "Off-Center (Rotation)";
			case EScenario::Oblique:          return "Oblique Collision";
			default: return "Unknown";
		}
	}
}
