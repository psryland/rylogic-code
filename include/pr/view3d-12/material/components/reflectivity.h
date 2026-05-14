//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/view3d-12/material/components/material_component.h"

namespace pr::rdr12::materials
{
	// Reflectivity material state shared by environment-map and ray-tracing paths.
	struct Reflectivity
	{
		static constexpr RdrId Id = hash::HashCT("materials::Reflectivity");

		float m_rel_reflec = 1.0f; // Reflectivity relative to the instance reflectivity.
	};
	static_assert(ComponentType<Reflectivity>);
}

