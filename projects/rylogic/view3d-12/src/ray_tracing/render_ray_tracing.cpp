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
		, m_cmd_list(scene.d3d(), nullptr, "RenderRayTracing", EColours::BlueViolet)
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
		if (RayTracingFeatureRequested(L"RYLOGIC_VIEW3D_RAY_TRACING_DIAGNOSTIC"))
			return ERayTracingScreenPass::Diagnostic;
		if (RayTracingFeatureRequested(L"RYLOGIC_VIEW3D_RAY_TRACING_HARD_SHADOWS"))
			return ERayTracingScreenPass::HardShadows;
		if (RayTracingFeatureRequested(L"RYLOGIC_VIEW3D_RAY_TRACING_CAUSTICS"))
			return ERayTracingScreenPass::Caustics;

		switch (scn().RayTracingFeatures())
		{
			case ERayTracingFeature::None:
			{
				return ERayTracingScreenPass::None;
			}
			case ERayTracingFeature::Reflections:
			{
				return ERayTracingScreenPass::Reflections;
			}
			case ERayTracingFeature::Caustics:
			{
				return ERayTracingScreenPass::Caustics;
			}
			case ERayTracingFeature::All:
			{
				return ERayTracingScreenPass::ReflectionsAndCaustics;
			}
			default:
			{
				throw std::runtime_error("Unknown ray tracing feature flags");
			}
		}
	}

	// Prepare the raster reflection side-buffer before the forward opaque pass writes it.
	RayTracingReflectionBuffer* RenderRayTracing::PrepareReflectionAttributes(Frame& frame)
	{
		auto const pass = ScreenPass();
		if (!rdr().RayTracing().Available() || (pass != ERayTracingScreenPass::Reflections && pass != ERayTracingScreenPass::ReflectionsAndCaustics))
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
		auto pass = ScreenPass();
		if (!rdr().RayTracing().Available() || pass == ERayTracingScreenPass::None)
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

		// Reflections and caustics update the K-buffer's opaque base in the post-resolve stage. This keeps RT lighting between
		// AlphaKBuffer::CopyOpaqueBuffer and AlphaKBuffer::ResolveAlpha, so existing sorted alpha layers still composite over it.
		auto heaps = { wnd().m_heap_view.get() };
		if (pass == ERayTracingScreenPass::Reflections || pass == ERayTracingScreenPass::Caustics || pass == ERayTracingScreenPass::ReflectionsAndCaustics)
		{
			auto const use_reflections = pass == ERayTracingScreenPass::Reflections || pass == ERayTracingScreenPass::ReflectionsAndCaustics;
			if (use_reflections && !m_reflections)
				return;

			m_cmd_list.Reset(frame.m_cmd_alloc_pool.Get());
			frame.m_post.push_back(m_cmd_list);
			m_cmd_list.SetDescriptorHeaps({ heaps.begin(), heaps.size() });
			m_diagnostic.Record(m_cmd_list, frame, scn(), m_ray_tracing, pass, use_reflections ? &m_reflections : nullptr, false);
			m_cmd_list.Close();
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
