//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/render/render_step.h"
#include "pr/view3d-12/ray_tracing/ray_tracing_scene.h"

namespace pr::rdr12
{
	// Stub render step that owns the scene-level TLAS cache for future DXR rendering.
	struct RenderRayTracing :RenderStep
	{
	private:

		RayTracingScene m_ray_tracing;

	public:

		explicit RenderRayTracing(Scene& scene);

		// Compile-time derived type
		inline static constexpr ERenderStep Id = ERenderStep::RayTracing;

		// Return the latest TLAS build diagnostics.
		RayTracingSceneStats const& Stats() const;

	private:

		// Perform the render step.
		void Execute(Frame& frame) override;

		// Add model nuggets to the draw list for this render step.
		void AddNuggets(BaseInstance const& inst, NuggetPtr nuggets, drawlist_t& drawlist) override;
	};
}
