//*********************************************
// Physics Engine — Gathered GPU Frame Output
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/physics/forward.h"
#include "src/utility/gpu.h"

namespace pr::physics
{
	struct GpuArticulationMidpointOutput;
	struct GpuConstraintBreakOutput;
	struct GpuCoupledConstraintFailureOutput;
	struct GpuCoupledConstraintFailureState;

	// Byte offsets for one packed output containing state, events, break latches, and coupled rejection records.
	struct GpuFrameOutputLayout
	{
		int m_body_count;
		int m_event_capacity;
		int m_articulation_count;
		int m_position_count;
		int m_velocity_count;
		int m_constraint_break_count;
		int m_coupled_failure_count;
		int64_t m_body_offset;
		int64_t m_event_offset;
		int64_t m_articulation_offset;
		int64_t m_position_offset;
		int64_t m_velocity_offset;
		int64_t m_acceleration_offset;
		int64_t m_constraint_break_offset;
		int64_t m_coupled_failure_offset;
		int64_t m_readback_size;
		int64_t m_resource_size;
	};

	// Readback allocation and the exact layout used to populate it.
	struct GpuFrameOutputReadback
	{
		::pr::compute::GpuReadbackBuffer::Allocation m_allocation;
		GpuFrameOutputLayout m_layout;
	};

	// Observable packed-output storage and dispatch state for the current frame.
	struct GpuFrameOutputStats
	{
		int m_body_count;
		int m_event_capacity;
		int m_articulation_count;
		int m_constraint_break_count;
		int m_coupled_failure_count;
		int m_dispatch_count;
		int m_readback_count;
		int m_pad1;
		size_t m_logical_bytes;
		size_t m_allocated_feature_bytes;
		size_t m_readback_bytes;
	};

	// Owns the bounded frame event queue and gathers all selected outputs for one final GPU-to-CPU copy.
	struct GpuFrameOutput
	{
		Gpu& m_gpu;
		ComputeStep m_cs_prepare_substep;
		ComputeStep m_cs_compact_events;
		ComputeStep m_cs_append_events;
		ComputeStep m_cs_gather_bodies;
		ComputeStep m_cs_gather_articulations;
		D3DPtr<ID3D12CommandSignature> m_cmd_sig;
		D3DPtr<ID3D12Resource> m_r_output;
		D3DPtr<ID3D12Resource> m_r_substep_state;
		GpuFrameOutputLayout m_layout;
		int64_t m_capacity;
		bool m_substep_state_active;
		int m_dispatch_count;
		int m_readback_count;

		explicit GpuFrameOutput(Gpu& gpu);

		// Reset aggregate state and establish the packed layout before any substep writes output.
		void BeginFrame(GpuJob& job, int body_count, int event_capacity, int substep_count);

		// Reset aggregate state and append compact articulation sections to the legacy rigid/event layout.
		void BeginFrame(GpuJob& job, int body_count, int event_capacity, int substep_count, GpuArticulationMidpointOutput const& articulations);

		// Reset aggregate state and append optional articulation, break-latch, and coupled-failure sections.
		// 'capture_substep_state' is independent of the rigid output count because shaped articulation proxies can produce collision counters without public rigid bodies.
		void BeginFrame(GpuJob& job, int body_count, int event_capacity, int substep_count, GpuArticulationMidpointOutput const& articulations, GpuConstraintBreakOutput const& constraint_breaks, GpuCoupledConstraintFailureOutput const& coupled_failures, bool capture_substep_state);

		// Retain counters and optional resolved contacts before the next substep resets transient collision buffers.
		void CaptureSubstep(GpuJob& job, int max_pairs, int max_contacts, int substep_index, int substep_count, bool collect_events, ID3D12Resource* counters, ID3D12Resource* contacts, ID3D12Resource* contact_order, ID3D12Resource* contact_dispatch);

		// Gather final bodies and record the frame's sole GPU-to-CPU CopyBufferRegion.
		GpuFrameOutputReadback GatherAndReadback(GpuJob& job, int body_count, ID3D12Resource* bodies);

		// Gather final rigid and articulation state before recording the frame's sole GPU-to-CPU copy.
		GpuFrameOutputReadback GatherAndReadback(GpuJob& job, int body_count, ID3D12Resource* bodies, GpuArticulationMidpointOutput const& articulations);

		// Gather final rigid, articulation, break-latch, and coupled-failure state before the frame's sole GPU-to-CPU copy.
		GpuFrameOutputReadback GatherAndReadback(GpuJob& job, int body_count, ID3D12Resource* bodies, GpuArticulationMidpointOutput const& articulations, GpuConstraintBreakOutput const& constraint_breaks, GpuCoupledConstraintFailureOutput const& coupled_failures);

		// Access typed sections after the owning GPU job has completed.
		static GpuFrameOutputHeader const& Header(GpuFrameOutputReadback const& readback);
		static std::span<GpuRigidBody const> Bodies(GpuFrameOutputReadback const& readback);
		static std::span<GpuCollisionEvent const> Events(GpuFrameOutputReadback const& readback);
		static std::span<GpuArticulationFrameOutput const> Articulations(GpuFrameOutputReadback const& readback);
		static std::span<float const> Positions(GpuFrameOutputReadback const& readback);
		static std::span<float const> Velocities(GpuFrameOutputReadback const& readback);
		static std::span<float const> Accelerations(GpuFrameOutputReadback const& readback);
		static std::span<GpuConstraintBreakState const> ConstraintBreaks(GpuFrameOutputReadback const& readback);
		static std::span<GpuCoupledConstraintFailureState const> CoupledFailures(GpuFrameOutputReadback const& readback);

		// Return packed output usage, retained allocation, and work recorded for the current frame.
		GpuFrameOutputStats Stats() const;

	private:

		// Create the optional articulation gather pipeline only when a frame first supplies a reduced-coordinate forest.
		void EnsureArticulationGatherPipeline();

		// Grow resources before command recording references the current allocation.
		void ResizeBuffers(CmdList& cmd_list, int64_t capacity, bool requires_substep_state);
	};
}
