//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/material/components/material_component.h"
#include "pr/view3d-12/material/components/texture_slot.h"
#include "pr/view3d-12/utility/pipe_state.h"

namespace pr::rdr12::materials
{
	// One world-space projection of a tileable slope map onto a surface.
	struct DetailNormalLayer
	{
		// The texture coordinates of a world position 'p' are u = dot(p.xyz, m_row_u.xyz) + m_row_u.w and v = dot(p.xyz, m_row_v.xyz) + m_row_v.w.
		// Animate a layer by changing the w offsets. 'm_height_scale' converts one unit of the map's height per texture unit into world height.
		v4 m_row_u = v4::XAxis();
		v4 m_row_v = v4::ZAxis();
		float m_height_scale = 0.0f;
	};

	// A set of detail-normal layers shared by every material copy that refers to it.
	struct DetailNormalLayers
	{
		static constexpr int MaxLayers = 4;

		std::array<DetailNormalLayer, MaxLayers> m_layers = {}; // Layers in use are [0, m_count).
		int m_count = 0;                                        // Number of layers in use, in [0, MaxLayers].

		// Replace the layers in use. 'layers' must contain at most MaxLayers entries with finite values.
		void Set(std::span<DetailNormalLayer const> layers);
	};

	// Detail normals for MaterialSimple. Each layer samples the same tileable slope map in a world-space projection, and the summed
	// height gradients tilt the interpolated surface normal. The map's red and green channels hold the height slope along u and v,
	// encoded as (slope + 1) / 2. Layers are shared by reference, so changing them updates every material copy without replacing materials.
	// While disabled, the owning material does not report this component and the stock pixel shaders are used.
	struct DetailNormals
	{
		static constexpr RdrId Id = hash::HashCT("materials::DetailNormals");

		TextureSlot m_tex = { {}, {}, {}, ETextureColourSpace::Linear, {} }; // Tileable slope map. Must have a texture and a sampler when enabled.
		std::shared_ptr<DetailNormalLayers> m_layers;                         // Shared layers. Non-null when enabled.
		bool m_enable = false;                                                // True when detail normals are applied.
	};
	static_assert(ComponentType<DetailNormals>);

	// Replace a stock simple-material forward pixel shader in 'desc' with its detail-normal variant. Throws for any other pixel shader.
	void ApplyDetailNormalsPixelShader(PipeStateDesc& desc);
}
