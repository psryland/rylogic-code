//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/view3d-12/material/components/material_component.h"

namespace pr::rdr12::materials
{
	// Optical material defaults used by ray-tracing paths until richer transparent material properties exist.
	struct Optics
	{
		static constexpr RdrId Id = hash::HashCT("materials::Optics");

		float m_transmission = 0.85f; // Fallback transmission when alpha implies transparency but no per-hit value is available.
		float m_ior = 1.5f;          // Default glass IOR.
		float m_thickness = 0.35f;   // Approximate glass thickness used by the caustic proof path.
	};
	static_assert(ComponentType<Optics>);
}

