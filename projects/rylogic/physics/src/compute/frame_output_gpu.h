//*********************************************
// Physics Engine — Gathered GPU Frame Output
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/physics/forward.h"
#include "src/utility/gpu.h"

namespace pr::physics
{
	// Byte offsets for one packed output containing a header, final bodies, and optional collision events.
	struct GpuFrameOutputLayout
	{
		int m_body_count;
		int m_event_capacity;
		int64_t m_body_offset;
		int64_t m_event_offset;
		int64_t m_readback_size;
		int64_t m_resource_size;
	};

	// Readback allocation and the exact layout used to populate it.
	struct GpuFrameOutputReadback
	{
		::pr::compute::GpuReadbackBuffer::Allocation m_allocation;
		GpuFrameOutputLayout m_layout;
	};

	// Owns the bounded frame event queue and gathers all selected outputs for one final GPU-to-CPU copy.
	struct GpuFrameOutput
	{
		Gpu& m_gpu;
		ComputeStep m_cs_prepare_substep;
		ComputeStep m_cs_append_events;
		ComputeStep m_cs_gather_bodies;
		D3DPtr<ID3D12CommandSignature> m_cmd_sig;
		D3DPtr<ID3D12Resource> m_r_output;
		D3DPtr<ID3D12Resource> m_r_substep_state;
		GpuFrameOutputLayout m_layout;
		int64_t m_capacity;

		explicit GpuFrameOutput(Gpu& gpu);

		// Reset aggregate state and establish the packed layout before any substep writes output.
		void BeginFrame(GpuJob& job, int body_count, int event_capacity, int substep_count);

		// Retain counters and optional resolved contacts before the next substep resets transient collision buffers.
		void CaptureSubstep(GpuJob& job, int max_pairs, int max_contacts, int substep_index, int substep_count, bool collect_events, ID3D12Resource* counters, ID3D12Resource* contacts, ID3D12Resource* contact_order, ID3D12Resource* contact_dispatch);

		// Gather final bodies and record the frame's sole GPU-to-CPU CopyBufferRegion.
		GpuFrameOutputReadback GatherAndReadback(GpuJob& job, int body_count, ID3D12Resource* bodies);

		// Access typed sections after the owning GPU job has completed.
		static GpuFrameOutputHeader const& Header(GpuFrameOutputReadback const& readback);
		static std::span<GpuRigidBody const> Bodies(GpuFrameOutputReadback const& readback);
		static std::span<GpuCollisionEvent const> Events(GpuFrameOutputReadback const& readback);

	private:

		// Grow resources before command recording references the current allocation.
		void ResizeBuffers(CmdList& cmd_list, int64_t capacity);
	};
}
