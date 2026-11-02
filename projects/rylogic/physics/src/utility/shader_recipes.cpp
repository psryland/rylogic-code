//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "src/utility/shader_recipes.h"
#include "src/utility/gpu.h"

namespace pr::physics
{
	// Collect the recipes declared by each subsystem that compiles kernels at run time.
	std::vector<ShaderRecipe> ShaderRecipes()
	{
		// BoundsSorter and ContactSorter are the same sorter type with default tuning, so its kernels are listed once.
		static_assert(std::is_same_v<BoundsSorter, ContactSorter>);
		auto recipes = ContactSorter::ShaderRecipes();
		AppendWorldContactShaderRecipes(recipes);
		AppendWaterShaderRecipes(recipes);
		atmosphere::AppendAtmosphereShaderRecipes(recipes);
		return recipes;
	}
}
