//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/render/render_step.h"
#include "pr/view3d-12/shaders/shader_smap.h"
#include "pr/view3d-12/lighting/shadow_view.h"

namespace pr::rdr12
{
	// Renders the depth of shadow casters into a shared shadow atlas for all shadow-casting lights.
	struct RenderSmap :RenderStep
	{
		// Notes:
		//  - The scene's shadow settings choose which lights have shadows. Each light has one or more shadow views (six for
		//    point lights), and each view is a square region of a single depth texture, the shadow atlas.
		//  - Views are chosen in 'Prepare' so that the forward pass can refer to them in the same frame.
		//  - Views are rendered in batches of up to 'ShadowViewBatchSize'. Each batch binds one viewport per view, and each
		//    element is drawn once with one instance per view that can see it. The vertex shader chooses the viewport.
		//  - Directional lights use cascades: views of increasing size that cover increasing depth ranges of the camera view.
		//    Directional views are orthographic with depth clamping, so casters in front of a view's near plane still cast shadows.
		//    Batches never mix clamped and unclamped views because depth clamping is part of the pipeline state.
		//  - With 'ShadowSettings::m_cache_views', a view is only re-rendered when a hash of its content changes. The hash covers
		//    the view transform, its atlas region, and every element the view can see. Elements drawn with a custom vertex shader
		//    (e.g. procedural geometry) can change without the hash changing, so views that see them are rendered every frame.
		//    Changes to texture content (alpha clipped casters) are not detected.

		using GfxCmdList = ::pr::compute::GfxCmdList;

	private:

		shaders::ShadowMap m_shader;      // The shader for this render step
		GfxCmdList m_cmd_list;            // The command list for this render step
		Texture2DPtr m_default_tex;       // Texture to use if a model has no diffuse texture
		SamplerPtr m_default_sam;         // Sampler to use if a model has no sampler
		Texture2DPtr m_atlas;             // The shadow atlas depth texture
		ShadowSettings m_settings;        // The shadow settings used to create the atlas and pipeline state
		ShadowViewSet m_views;            // The shadow views for the current frame
		ShadowViewCache m_cache;          // The content of the atlas regions rendered in earlier frames
		pr::vector<uint32_t> m_element_views; // Per draw list element, a bit mask of the views that can see it
		uint32_t m_dirty;                 // Bit mask of the views to render this frame
		std::unordered_set<Nugget const*> m_volatile; // Nuggets drawn with a custom vertex shader. Views that see them are rendered every frame

	public:

		explicit RenderSmap(Scene& scene);

		// Compile-time derived type
		inline static constexpr ERenderStep Id = ERenderStep::ShadowMap;

		// The shadow views for the current frame. Valid from 'Prepare' until the next frame.
		ShadowViewSet const& Views() const;

		// The shadow atlas depth texture
		Texture2D const* Atlas() const;

		// The shadow settings used for the current frame
		ShadowSettings const& Settings() const;

	private:

		// Choose the shadow views for the frame
		void Prepare(Frame& frame) override;

		// Perform the render step
		void Execute(Frame& frame) override;

		// Add model nuggets to the draw list for this render step
		void AddNuggets(BaseInstance const& inst, NuggetPtr nuggets, drawlist_t& drawlist) override;

		// Create the atlas texture and the pipeline state description for the current shadow settings
		void CreateAtlas(ShadowSettings const& settings);

		// Find the views that each element can see, and the views whose content has changed since they were last rendered
		void FindDirtyViews(std::span<BBox const> element_bounds);
	};
}
