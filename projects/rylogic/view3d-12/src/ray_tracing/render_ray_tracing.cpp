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
		// Return true when an internal ray tracing feature toggle is enabled.
		bool RayTracingFeatureRequested(wchar_t const* name)
		{
			auto value = std::array<wchar_t, 16>{};
			auto len = GetEnvironmentVariableW(name, value.data(), s_cast<DWORD>(value.size()));
			return len != 0 && value[0] != L'0';
		}
	}

	// Create the stub ray tracing render step.
	RenderRayTracing::RenderRayTracing(Scene& scene)
		: RenderStep(Id, scene)
		, m_ray_tracing()
		, m_diagnostic()
		, m_reflections()
	{}

	// Release ray tracing resources through the renderer's deferred-release queue.
	RenderRayTracing::~RenderRayTracing()
	{
		m_ray_tracing.DeferRelease(rdr());
		m_diagnostic.DeferRelease(rdr());
		m_reflections.DeferRelease(rdr());
	}

	// Return the latest TLAS build diagnostics.
	RayTracingSceneStats const& RenderRayTracing::Stats() const
	{
		return m_ray_tracing.Stats();
	}

	// Return the selected RT screen-space pass for this frame.
	ERayTracingScreenPass RenderRayTracing::ScreenPass() const
	{
		return RayTracingFeatureRequested(L"RYLOGIC_VIEW3D_RAY_TRACING_DIAGNOSTIC") ? ERayTracingScreenPass::Diagnostic :
			RayTracingFeatureRequested(L"RYLOGIC_VIEW3D_RAY_TRACING_HARD_SHADOWS") ? ERayTracingScreenPass::HardShadows :
			ERayTracingScreenPass::Reflections;
	}

	// Prepare the raster reflection side-buffer before the forward opaque pass writes it.
	RayTracingReflectionBuffer* RenderRayTracing::PrepareReflectionAttributes(Frame& frame)
	{
		if (!rdr().RayTracing().Available() || ScreenPass() != ERayTracingScreenPass::Reflections)
			return nullptr;

		auto const size = frame.bb_main().rt_size();
		auto const multisamp = frame.bb_main().m_multisamp;
		if (!m_reflections.Matches(size, multisamp))
		{
			ResourceFactory factory(rdr());
			m_reflections.Prepare(factory, size, multisamp);
			factory.FlushToGpu(EGpuFlush::Block);
		}

		return m_reflections ? &m_reflections : nullptr;
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

		auto pass = ScreenPass();
		auto prepared = m_diagnostic.Prepare(factory, frame.bb_post().rt_size(), wnd().m_rt_props.Format);
		factory.FlushToGpu(EGpuFlush::Block);
		if (!prepared)
			return;

		// Reflections update the K-buffer's opaque base before the already-recorded alpha resolve composites sorted alpha layers over it.
		auto heaps = { wnd().m_heap_view.get() };
		if (pass == ERayTracingScreenPass::Reflections)
		{
			if (!m_reflections)
				return;

			frame.m_resolve.SetDescriptorHeaps({ heaps.begin(), heaps.size() });
			m_diagnostic.Record(frame.m_resolve, frame, scn(), m_ray_tracing, pass, &m_reflections, false);
			return;
		}

		// Present-list commands execute after View3D's alpha resolve, so diagnostic/shadow output is not overwritten by the normal forward-composite path.
		frame.m_present.SetDescriptorHeaps({ heaps.begin(), heaps.size() });
		m_diagnostic.Record(frame.m_present, frame, scn(), m_ray_tracing, pass, nullptr, true);
	}

	// Add model nuggets to the draw list for this render step.
	void RenderRayTracing::AddNuggets(BaseInstance const&, NuggetPtr, drawlist_t&)
	{
		// The TLAS is built from scene instances rather than per-nugget draw-list elements. Keeping this no-op avoids duplicating instance records.
	}
}
