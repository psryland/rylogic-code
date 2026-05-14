//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/view3d-12/material/components/material_component.h"

namespace pr::rdr12::materials
{
	// Surface shading options shared by lit render paths.
	struct Surface
	{
		static constexpr RdrId Id = hash::HashCT("materials::Surface");

		bool m_two_sided = false; // Flip back-facing normals instead of treating vertex normals as one-sided.
	};
	static_assert(ComponentType<Surface>);
}

