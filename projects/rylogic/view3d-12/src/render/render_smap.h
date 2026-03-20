//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/render/render_step.h"
#include "pr/view3d-12/shaders/shader_smap.h"
#include "pr/view3d-12/resource/descriptor_store.h"
#include "pr/view3d-12/resource/descriptor.h"
#include "pr/view3d-12/utility/shadow_caster.h"

namespace pr::rdr12
{
	struct RenderSmap :RenderStep
	{
		// Algorithm:
		//  - Create a 2D colour texture. R=depth,G=colour?
		//  - Directional:
		//    - Create an orthographic projection that encloses everything the view can see plus everything between the light and the view.
		//    - Render the shadow map pass before the main render pass
		//    - Shade the scene using the smap
		//  - Spot:
		//    - Create a perspective projection that encloses everything the view can see plus everything between the light and the view.
		//    - Render the shadow map pass before the main render pass
		//    - Shade the scene using the smap
		//  - Point:
		//    - Create 6 perspective projections around the light.
		//    - ?? Use a Fibonacci sphere to map directions around the light to a 2D surface
		//    - ?? not sure
		//  - LiSPSM:
		//    - During the shadow map rendering pass, apply a perspective transform to the scene where the perspective
		//      view is perpendicular to the light direction.
		//    - During the main render, apply the perspective to the light lookup ray before sampling the smap.
		//
		// Notes:
		//  - This is an implementation of light space perspective shadow mapping (LiSPSM).
		//    (see Light Space Perspective Shadow Maps by Michael Wimmer, Daniel Scherzer and Werner Purgathofer)
		//    The main idea of perspective shadow mapping is to apply a perspective transformation
		//    to the scene before rendering it into the shadow map. In the original PSM algorithm
		//    the perspective transform was the same as the view projection, but that does weird
		//    things to the light direction. In LiSPSM, the projection is perpendicular to the light
		//    direction instead, with Zn and Zf clamped to the view frustum Zn,Zf.
		//  - The shadow map step handles generation of all shadow maps for all lights in the scene.
		//    It renders a shadow map for each shadow caster as a separate pass.
		//  - The smap face must be perpendicular to the light direction otherwise the smap texels
		//    are not isotropic and the shadow will be blocky in some places.
		//  - The shadow map is not a depth buffer. It's a colour buffer with depth encoded into it.

		using CasterCont = pr::vector<ShadowCaster, 4>;

	private:

		shaders::ShadowMap m_shader; // The shader for this render step
		GfxCmdList m_cmd_list;       // The command list for this render step
		Texture2DPtr m_default_tex;  // Texture to use if a model has no diffuse texture
		SamplerPtr m_default_sam;    // Sampler to use if a model has no sampler
		CasterCont m_casters;        // The light sources that cast shadows. This is the list of lights to create shadow maps for.
		int m_smap_size;             // Dimensions of the (square) 'smap' textures
		DXGI_FORMAT m_smap_format;   // The texture format of the smap textures
		BBox m_bbox_scene;           // The scene bounds of shadow casters

	public:

		RenderSmap(Scene& scene, Light const& light, int size = 1024, DXGI_FORMAT format = DXGI_FORMAT_R32_FLOAT);
		~RenderSmap();

		// Compile-time derived type
		inline static constexpr ERenderStep Id = ERenderStep::ShadowMap;

		// Add a shadow casting light source
		void AddLight(Light const& light);

		// The sources of light that cast shadows
		CasterCont const& Casters() const
		{
			return m_casters;
		}

	private:

		// Perform the render step
		void Execute(Frame& frame) override;

		// Add model nuggets to the draw list for this render step
		void AddNuggets(BaseInstance const& inst, NuggetPtr nuggets, drawlist_t& drawlist) override;

		// Call draw for a nugget
		void DrawNugget(Nugget const& nugget, PipeStateDesc& desc);
	};
}
