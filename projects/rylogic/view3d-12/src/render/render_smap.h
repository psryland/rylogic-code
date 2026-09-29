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
		//  - Views are fitted to the bounds of all shadow casters. Wide scenes lose resolution; cascades are needed for those.

		using GfxCmdList = ::pr::compute::GfxCmdList;

	private:

		shaders::ShadowMap m_shader;      // The shader for this render step
		GfxCmdList m_cmd_list;            // The command list for this render step
		Texture2DPtr m_default_tex;       // Texture to use if a model has no diffuse texture
		SamplerPtr m_default_sam;         // Sampler to use if a model has no sampler
		Texture2DPtr m_atlas;             // The shadow atlas depth texture
		ShadowSettings m_settings;        // The shadow settings used to create the atlas and pipeline state
		ShadowViewSet m_views;            // The shadow views for the current frame
		pr::vector<BBox> m_element_bounds; // World space bounds of each draw list element. Invalid bounds mean "visible in all views"

	public:

		explicit RenderSmap(Scene& scene);

		// Compile-time derived type
		inline static constexpr ERenderStep Id = ERenderStep::ShadowMap;

		// The shadow views for the current frame. Valid from 'Prepare' until the next frame.
		ShadowViewSet const& Views() const;

		// The shadow atlas depth texture
		Texture2D const* Atlas() const;

		// The width and height of the shadow atlas (in pixels)
		int AtlasSize() const;

	private:

		// Choose the shadow views for the frame
		void Prepare(Frame& frame) override;

		// Perform the render step
		void Execute(Frame& frame) override;

		// Add model nuggets to the draw list for this render step
		void AddNuggets(BaseInstance const& inst, NuggetPtr nuggets, drawlist_t& drawlist) override;

		// Create the atlas texture and the pipeline state description for the current shadow settings
		void CreateAtlas(ShadowSettings const& settings);
	};
}
