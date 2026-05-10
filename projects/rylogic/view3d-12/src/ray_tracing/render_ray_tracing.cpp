//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#include "pr/view3d-12/ray_tracing/render_ray_tracing.h"
#include "pr/view3d-12/main/renderer.h"
#include "pr/view3d-12/scene/scene.h"
#include "pr/view3d-12/model/nugget.h"
#include "pr/view3d-12/resource/resource_factory.h"

namespace pr::rdr12
{
	// Create the stub ray tracing render step.
	RenderRayTracing::RenderRayTracing(Scene& scene)
		: RenderStep(Id, scene)
		, m_ray_tracing()
	{}

	// Return the latest TLAS build diagnostics.
	RayTracingSceneStats const& RenderRayTracing::Stats() const
	{
		return m_ray_tracing.Stats();
	}

	// Perform the render step.
	void RenderRayTracing::Execute(Frame&)
	{
		if (!rdr().RayTracing().Available())
			return;

		// Milestone 3 only proves TLAS maintenance. The DXR pipeline, shader table, and output UAV arrive in the diagnostic render milestone.
		ResourceFactory factory(rdr());
		m_ray_tracing.Build(factory, std::span<BaseInstance const* const>(scn().m_instances.data(), scn().m_instances.size()));
		factory.FlushToGpu(EGpuFlush::Block);
	}

	// Add model nuggets to the draw list for this render step.
	void RenderRayTracing::AddNuggets(BaseInstance const&, NuggetPtr, drawlist_t&)
	{
		// The TLAS is built from scene instances rather than per-nugget draw-list elements. Keeping this no-op avoids duplicating instance records.
	}
}
