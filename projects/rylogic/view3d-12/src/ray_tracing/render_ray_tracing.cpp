//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#include "pr/view3d-12/ray_tracing/render_ray_tracing.h"
#include "pr/view3d-12/main/frame.h"
#include "pr/view3d-12/main/renderer.h"
#include "pr/view3d-12/main/window.h"
#include "pr/view3d-12/scene/scene.h"
#include "pr/view3d-12/model/nugget.h"
#include "pr/view3d-12/resource/resource_factory.h"

namespace pr::rdr12
{
	namespace
	{
		// Return true when the internal DXR diagnostic visualisation is requested instead of the current lighting feature.
		bool RayTracingDiagnosticModeRequested()
		{
			auto value = std::array<wchar_t, 16>{};
			auto len = GetEnvironmentVariableW(L"RYLOGIC_VIEW3D_RAY_TRACING_DIAGNOSTIC", value.data(), s_cast<DWORD>(value.size()));
			return len != 0 && value[0] != L'0';
		}
	}

	// Create the stub ray tracing render step.
	RenderRayTracing::RenderRayTracing(Scene& scene)
		: RenderStep(Id, scene)
		, m_ray_tracing()
		, m_diagnostic()
	{}

	// Release ray tracing resources through the renderer's deferred-release queue.
	RenderRayTracing::~RenderRayTracing()
	{
		m_ray_tracing.DeferRelease(rdr());
		m_diagnostic.DeferRelease(rdr());
	}

	// Return the latest TLAS build diagnostics.
	RayTracingSceneStats const& RenderRayTracing::Stats() const
	{
		return m_ray_tracing.Stats();
	}

	// Perform the render step.
	void RenderRayTracing::Execute(Frame& frame)
	{
		if (!rdr().RayTracing().Available())
			return;

		ResourceFactory factory(rdr());
		m_ray_tracing.Build(factory, std::span<BaseInstance const* const>(scn().m_instances.data(), scn().m_instances.size()));
		if (!m_ray_tracing.Built())
		{
			factory.FlushToGpu(EGpuFlush::Block);
			return;
		}

		auto prepared = m_diagnostic.Prepare(factory, frame.bb_post().rt_size(), wnd().m_rt_props.Format);
		factory.FlushToGpu(EGpuFlush::Block);
		if (!prepared)
			return;

		// Present-list commands execute after View3D's alpha resolve, so the RT screen pass is not overwritten by the normal forward-composite path.
		auto heaps = { wnd().m_heap_view.get() };
		frame.m_present.SetDescriptorHeaps({ heaps.begin(), heaps.size() });
		auto pass = RayTracingDiagnosticModeRequested()
			? ERayTracingScreenPass::Diagnostic
			: ERayTracingScreenPass::HardShadows;
		m_diagnostic.Record(frame.m_present, frame, scn(), m_ray_tracing, pass, true);
	}

	// Add model nuggets to the draw list for this render step.
	void RenderRayTracing::AddNuggets(BaseInstance const&, NuggetPtr, drawlist_t&)
	{
		// The TLAS is built from scene instances rather than per-nugget draw-list elements. Keeping this no-op avoids duplicating instance records.
	}
}
