//************************************
// Game Systems
//  Copyright (c) Rylogic Ltd 2026
//************************************
// The stock particle effects: ready-made settings of the stock particle types for common effects, such as fire, rain and falling leaves.
// A preset gives everything needed to create one batch, one type and its emitters. Presets are starting points: callers copy the constants
// and emitter parameters, and change them to suit. See pr::view3d::ParticlePreset for the conventions that presets follow.
#pragma once
#include "pr/game-systems/forward.h"

namespace pr::game_systems::particles
{
	// Return the settings of the stock effect 'preset'. Every pointer in the result refers to static data.
	pr::view3d::ParticlePreset ParticlePresetGet(pr::view3d::EParticlePreset preset);
}
