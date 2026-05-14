//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/material/components/material_component.h"
#include "pr/view3d-12/shaders\shader.h"

namespace pr::rdr12::materials
{
	// Shader overlay component shared by materials that augment render-step default shaders.
	struct ShaderOverlays
	{
		static constexpr RdrId Id = hash::HashCT("materials::ShaderOverlays");

		// Shader stages that a material overlays onto a render-step default shader.
		struct Overlay
		{
			mutable ShaderPtr m_overlay = {};           // Individual stages that replace stages in the full shader.
			ERenderStep m_rdr_step = ERenderStep::Invalid; // The render step that the shader applies to.
			int pad = {};
		};

		vector<Overlay, 4, false> m_overlays = {}; // Overlays applied in order; later overlays win for repeated stages.

		// Add a shader overlay for 'step'. The shader is added to the end of the list of overlays, so later overlays win over earlier ones for repeated stages.
		void add(ERenderStep step, ShaderPtr overlay)
		{
			m_overlays.push_back(Overlay{ overlay, step });
		}
	};
	static_assert(ComponentType<ShaderOverlays>);
}

