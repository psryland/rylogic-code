//*********************************************
// Renderer
//  Copyright (c) Rylogic Ltd 2012
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/render/render_step.h"
#include "pr/view3d-12/resource/gpu_descriptor_heap.h"
#include "pr/view3d-12/shaders/shader_ray_cast.h"
#include "pr/view3d-12/utility/ray_cast.h"

namespace pr::rdr12
{
	// Render step for performing ray casts
	struct RenderRayCast :RenderStep
	{
		// Notes:
		//  - This render step can be used in three main ways:
		//    1. Immediate ray casts: Call 'SetRays' followed by 'ExecuteImmediate' to perform a ray cast and read
		//       the results on the CPU immediately. This is the simplest way to use this render step, but it blocks
		//       the CPU until the GPU has completed the ray cast.
		//    2. Async ray casts: Call 'SetRays' followed by 'ExecuteAsync' to submit the ray cast to the GPU and
		//       return immediately. Results are reported via the callback passed to 'ExecuteAsync' when the GPU completes.
		//    3. As part of a render frame: Call 'SetRays'. After a scene render, the hit results will be reported back via
		//       the results callback supplied at construction.

	private:
		struct PendingResults
		{
			GpuTransferAllocation m_output; // The buffer containing the results of the ray casts
			uint64_t m_sync_point; // The sync point for the GPU work that will produce the results
			RayCastResultsOut m_cb; // The callback to invoke when the results are ready
		};

		vector<HitTestRay, 4>     m_rays;           // Rays to cast
		vector<HitTestResult, 4>  m_results;        // The results from the last ray cast
		RayCastFilter             m_include;        // A filter for instances to include for hit testing
		GfxCmdList                m_cmd_list;       // Command buffer
		GpuSync                   m_gsync;          //
		shaders::RayCast          m_shader;         // The ray cast shader
		D3DPtr<ID3D12Resource>    m_zero;           // A buffer of zeros used to reset the output counter
		D3DPtr<ID3D12Resource>    m_out;            // An unstructured buffer for the number of intercepts and the intercept data
		GpuReadbackBuffer         m_readback;       // A read back buffer for reading intercept data
		vector<PendingResults, 1> m_pending;        // Pending results waiting for the GPU to complete
		RayCastResultsOut         m_cb_results;     // Callback for results acquired during a scene render
		AutoSub                   m_sync_completed; // Subscription to the GPU sync completed event

	public:

		explicit RenderRayCast(Scene& scene, RayCastResultsOut results_cb);

		// Compile-time derived type
		inline static constexpr ERenderStep Id = ERenderStep::RayCast;

		// Set the rays to cast.
		// 'snap_mode' controls how point snapping is applied.
		// 'snap_distance', if the mode is 'perspective' then this is the ratio proportional to depth from the ray origin, otherwise it's in world units.
		// 'flags' controls what primitives snapping applies to.
		// 'filter' filters instances added to the render step (i.e. decides what's hit-able)
		void SetRays(std::span<HitTestRay const> rays, RayCastFilter include);

		// Perform the ray cast and read the results
		std::future<void> ExecuteImmediate(RayCastResultsOut out);

		// Submit the ray cast to the GPU and return immediately.
		// Results are reported via 'cb' when the GPU completes.
		void ExecuteAsync(RayCastResultsOut cb);

	private:

		// Add model nuggets to the draw list for this render step
		void AddNuggets(BaseInstance const& inst, NuggetPtr nuggets, drawlist_t& drawlist) override;

		// Submit the ray cast to the GPU and return immediately.
		void Execute(Frame& frame) override;

		// Step up the GPU call for the ray cast
		GpuTransferAllocation ExecuteCore();
	
		// Draw a single nugget
		void DrawNugget(Nugget const& nugget, PipeStateDesc& desc);

		// Process ray cast results from a readback buffer and invoke the callback
		void ProcessResults(GpuTransferAllocation& output, RayCastResultsOut cb);
	};
}
