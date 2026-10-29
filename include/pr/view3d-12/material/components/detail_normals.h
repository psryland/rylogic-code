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
		// Smooth world noise scales the layer's height by a factor in [1 - m_weight_noise, 1 + m_weight_noise], which hides the tiling. The noise
		// has 'm_noise_frequency' cells per texture unit. It follows the orientation and scale of the layer's projection but not its offsets, so a
		// scrolling layer does not drag its noise, and a strength map's turn does not turn it.
		// Smooth world noise from the same cells also shifts the layer's texture coordinates by up to 'm_warp' texture units in u and v. This
		// bends the straight rows of repeated tiles, so the repetition does not line up across a large area. The normal is computed as if the
		// map were not warped, so keep m_warp * m_noise_frequency well below 1 to avoid a visible stretch of the map.
		v4 m_row_u = v4::XAxis();
		v4 m_row_v = v4::ZAxis();
		float m_height_scale = 0.0f;
		float m_weight_noise = 0.0f;    // In [0, 1].
		float m_noise_frequency = 0.0f; // Finite, and positive when m_weight_noise or m_warp is non-zero.
		float m_warp = 0.0f;            // Finite and non-negative.
	};

	// A grid of detail-normal strengths that varies the layers of every DetailNormalLayers set referring to it over the surface.
	// Each texel holds (x, y, z, w): x scales the height of every layer and y adds to the base slope variance; both are non-negative.
	// (z, w) = (cos a, sin a) is a unit vector that turns every layer's pattern by angle 'a' from its u direction toward its v direction,
	// about the world origin; (1, 0) leaves the layers unturned. Patterns are drawn only in 16 fixed directions, so an angle between two of them
	// cross-fades their patterns, and a changing angle fades between patterns instead of moving them. The texture coordinates of a world position 'p' are
	// u = dot(p.xyz, m_row_u.xyz) + m_row_u.w and v = dot(p.xyz, m_row_v.xyz) + m_row_v.w, where [0,1] spans the grid with texel (0,0) at
	// u,v = 0. Each fragment filters the texels bilinearly, including the angle vectors, and positions outside the grid use the nearest edge
	// texels. The fragment draws the two fixed directions either side of its filtered angle at its filtered strength, so neighbouring texels
	// more than one direction apart blend through the angles between them. The slopes of a cross-fade are scaled up so that it is as contrasted as one pattern. When 'm_visualise' is set, the shaded colour is replaced by the height strength,
	// from blue at 0 to red at 2 or more. Use on the renderer owner thread.
	struct DetailStrengthMap : RefCounted<DetailStrengthMap>
	{
		Texture2DPtr m_tex;
		SamplerPtr m_sam;
		std::unique_ptr<ResourceFactory> m_factory;
		int m_width;
		int m_height;
		v4 m_row_u;
		v4 m_row_v;
		bool m_visualise;

		// Create a 'width' x 'height' grid, each in [1,4096], with texels (1, 0, 1, 0) everywhere, which leaves the layers unchanged.
		DetailStrengthMap(Renderer& rdr, int width, int height);
		DetailStrengthMap(DetailStrengthMap const&) = delete;
		DetailStrengthMap& operator=(DetailStrengthMap const&) = delete;
		~DetailStrengthMap();

		// Set the world-space projection. Both rows must be finite, and their xyz parts must not be parallel so texel centres have world positions.
		void Projection(v4 const& row_u, v4 const& row_v);

		// Copy 'texels' (row-major, width * height; see the type comment for their domain) to the GPU. The copy is visible from the next render.
		void Upload(std::span<v4 const> texels);

		// Ref-counting clean up function
		static void RefCountZero(RefCounted<DetailStrengthMap>* doomed);
	};

	// A set of detail-normal layers shared by every material copy that refers to it.
	struct DetailNormalLayers
	{
		static constexpr int MaxLayers = 4;

		std::array<DetailNormalLayer, MaxLayers> m_layers = {}; // Layers in use are [0, m_count).
		int m_count = 0;                                        // Number of layers in use, in [0, MaxLayers].
		float m_base_slope_variance = 0.0f;                     // Mean square slope of detail finer than every layer. Non-negative.

		DetailStrengthMapPtr m_strength;                        // Optional strengths that vary the layers over the surface.

		// Replace the layers in use and the base slope variance. 'layers' must contain at most MaxLayers valid entries.
		void Set(std::span<DetailNormalLayer const> layers, float base_slope_variance);
	};

	// Detail normals for MaterialSimple. Each layer samples the same tileable slope map in a world-space projection, and the summed
	// height gradients tilt the interpolated surface normal. The map's red and green channels hold the height slope along u and v,
	// encoded as (slope + 1) / 2, and the blue channel holds (slope_u² + slope_v²) / 2. Mip filtering averages both, so where the map is
	// minified the shader recovers the slope variance the filtered normal has lost. That variance, plus the base slope variance, blurs and
	// weakens the environment reflection as a rough surface does. Layers are shared by reference, so changing them updates every material
	// copy without replacing materials.
	// While disabled, the owning material does not report this component and the stock pixel shaders are used.
	struct DetailNormals
	{
		static constexpr RdrId Id = hash::HashCT("materials::DetailNormals");

		TextureSlot m_tex = { {}, {}, {}, ETextureColourSpace::Linear, {} }; // Tileable slope map. Must have a texture and a sampler when enabled.
		std::shared_ptr<DetailNormalLayers> m_layers;                         // Shared layers. Non-null when enabled.
		bool m_enable = false;                                                // True when detail normals are applied.
	};
	static_assert(ComponentType<DetailNormals>);
}
