//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/physics/forward.h"

namespace pr::physics
{
	// Return a recipe for every kernel the engine compiles at run time. Running them fills a shader cache so later engine construction does not invoke DXC.
	std::vector<ShaderRecipe> ShaderRecipes();

	// Add recipes for the sampled world-contact kernels.
	void AppendWorldContactShaderRecipes(std::vector<ShaderRecipe>& recipes);

	// Add the recipe for the water-force kernel.
	void AppendWaterShaderRecipes(std::vector<ShaderRecipe>& recipes);

	namespace atmosphere
	{
		// Add recipes for the atmosphere solver, tracer, probe and surface-wind kernels.
		void AppendAtmosphereShaderRecipes(std::vector<ShaderRecipe>& recipes);
	}
}
