//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/view3d-12/material/components/material_component.h"

namespace pr::rdr12::materials
{
	// Surface orientation state shared by materials that support two-sided lighting.
	struct TwoSided
	{
		static constexpr RdrId Id = hash::HashCT("materials::TwoSided");

		bool m_enabled = false; // Flip back-facing normals instead of treating vertex normals as one-sided.
	};
	static_assert(ComponentType<TwoSided>);
}

