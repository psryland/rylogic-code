//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/render/render_step.h"
#include "pr/view3d-12/ray_tracing/ray_tracing_diagnostic.h"
#include "pr/view3d-12/ray_tracing/ray_tracing_reflections.h"
#include "pr/view3d-12/ray_tracing/ray_tracing_scene.h"

namespace pr::rdr12
{
	// Stub render step that owns the scene-level TLAS cache for future DXR rendering.
	struct RenderRayTracing :RenderStep
	{
	private:

		RayTracingScene m_ray_tracing;
		RayTracingDiagnostic m_diagnostic;
		RayTracingReflectionBuffer m_reflections;
		GfxCmdList m_cmd_list;
		int64_t m_prepared_frame;

	public:

		explicit RenderRayTracing(Scene& scene);
		~RenderRayTracing() override;

		// Compile-time derived type
		inline static constexpr ERenderStep Id = ERenderStep::RayTracing;

		// Return the latest TLAS build diagnostics.
		RayTracingSceneStats const& Stats() const;

		// Return the selected RT screen-space pass for this frame.
		ERayTracingScreenPass ScreenPass() const;

		// Prepare the raster reflection side-buffer before the forward opaque pass writes it.
		RayTracingReflectionBuffer* PrepareReflectionAttributes(Frame& frame);

	private:

		// Prepare RT resources that must be available before raster render steps execute.
		void Prepare(Frame& frame) override;

		// Perform the render step.
		void Execute(Frame& frame) override;

		// Add model nuggets to the draw list for this render step.
		void AddNuggets(BaseInstance const& inst, NuggetPtr nuggets, drawlist_t& drawlist) override;
	};
}
