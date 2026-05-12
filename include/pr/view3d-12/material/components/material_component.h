//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"

namespace pr::rdr12::materials
{
	// Concept for material component blocks addressable through Material::Component().
	template <typename T>
	concept ComponentType = requires
	{
		T::Id;
	};
}

