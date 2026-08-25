//*********************************************
// Physics Engine — Gathered GPU Frame Output
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "src/compute/frame_output_gpu.h"
#include "src/compute/articulation_midpoint_gpu.h"
#include "src/compute/physics_types.h"
#include "src/compute/shader_code.h"

namespace pr::physics
{
	using namespace ::pr::compute;

	namespace
	{
		// Keep every typed section naturally aligned while preserving the legacy rigid-only offsets.
		int64_t FrameOutputAlign(int64_t offset)
		{
			constexpr auto alignment = int64_t{16};
			return (offset + alignment - 1) & ~(alignment - 1);
		}
	}

	// Root constants shared by all frame-output kernels.
	struct alignas(16) cbFrameOutput
	{
		int max_pairs;
		int max_contacts;
		int event_capacity;
		int collect_events;
		int substep_index;
		int substep_count;
		int body_count;
		int pad0;
		int articulation_count;
		int position_count;
		int velocity_count;
		int pad1;
	};
	static_assert(sizeof(cbFrameOutput) == 48);

	// Register assignments mirror frame_output.hlsl.
	struct EReg
	{
		inline static constexpr auto Params = ECBufReg::b0;
		inline static constexpr auto Counters = EUAVReg::u0;
		inline static constexpr auto Contacts = EUAVReg::u1;
		inline static constexpr auto ContactOrder = EUAVReg::u2;
		inline static constexpr auto Header = EUAVReg::u3;
		inline static constexpr auto SubstepState = EUAVReg::u4;
		inline static constexpr auto Events = EUAVReg::u5;
		inline static constexpr auto Bodies = EUAVReg::u6;
		inline static constexpr auto OutputBodies = EUAVReg::u7;
		inline static constexpr auto Articulations = EUAVReg::u8;
		inline static constexpr auto ArticulationStates = EUAVReg::u9;
		inline static constexpr auto ArticulationPositions = EUAVReg::u10;
		inline static constexpr auto ArticulationVelocities = EUAVReg::u11;
		inline static constexpr auto ArticulationAccelerations = EUAVReg::u12;
		inline static constexpr auto OutputArticulations = EUAVReg::u13;
		inline static constexpr auto OutputPositions = EUAVReg::u14;
		inline static constexpr auto OutputVelocities = EUAVReg::u15;
		inline static constexpr auto OutputAccelerations = EUAVReg::u16;
	};

	GpuFrameOutput::GpuFrameOutput(Gpu& gpu)
		: m_gpu(gpu)
		, m_cs_prepare_substep()
		, m_cs_append_events()
		, m_cs_gather_bodies()
		, m_cs_gather_articulations()
		, m_cmd_sig()
		, m_r_output()
		, m_r_substep_state()
		, m_layout()
		, m_capacity()
	{
		// The serial reservation pass snapshots raw counters before the next substep resets them.
		{
			auto sig = RootSig(ERootSigFlags::ComputeOnly)
				.U32<cbFrameOutput>(EReg::Params)
				.UAV(EReg::Counters)
				.UAV(EReg::Header)
				.UAV(EReg::SubstepState)
				;

			m_cs_prepare_substep.m_sig = sig.Create(m_gpu, "Physics:PrepareSubstepOutputSig");
			m_cs_prepare_substep.m_pso = ComputePSO(m_cs_prepare_substep.m_sig.get(), shader_code::prepare_substep_output).Create(m_gpu, "Physics:PrepareSubstepOutputPSO");
		}

		// The event pass consumes the resolver's indirect dispatch so idle contact capacity is never scanned.
		{
			auto sig = RootSig(ERootSigFlags::ComputeOnly)
				.U32<cbFrameOutput>(EReg::Params)
				.UAV(EReg::Contacts)
				.UAV(EReg::ContactOrder)
				.UAV(EReg::SubstepState)
				.UAV(EReg::Events)
				;

			m_cs_append_events.m_sig = sig.Create(m_gpu, "Physics:AppendCollisionEventsSig");
			m_cs_append_events.m_pso = ComputePSO(m_cs_append_events.m_sig.get(), shader_code::append_collision_events).Create(m_gpu, "Physics:AppendCollisionEventsPSO");
		}

		// Final body gather writes directly into the body section of the packed output resource.
		{
			auto sig = RootSig(ERootSigFlags::ComputeOnly)
				.U32<cbFrameOutput>(EReg::Params)
				.UAV(EReg::Bodies)
				.UAV(EReg::OutputBodies)
				;

			m_cs_gather_bodies.m_sig = sig.Create(m_gpu, "Physics:GatherFrameBodiesSig");
			m_cs_gather_bodies.m_pso = ComputePSO(m_cs_gather_bodies.m_sig.get(), shader_code::gather_frame_bodies).Create(m_gpu, "Physics:GatherFrameBodiesPSO");
		}

		// Reuse the resolver's standard D3D12 dispatch-argument layout for event copies.
		auto arg = D3D12_INDIRECT_ARGUMENT_DESC{
			.Type = D3D12_INDIRECT_ARGUMENT_TYPE_DISPATCH,
		};
		auto desc = D3D12_COMMAND_SIGNATURE_DESC{
			.ByteStride = sizeof(D3D12_DISPATCH_ARGUMENTS),
			.NumArgumentDescs = 1,
			.pArgumentDescs = &arg,
		};
		Check(m_gpu->CreateCommandSignature(&desc, nullptr, __uuidof(ID3D12CommandSignature), reinterpret_cast<void**>(m_cmd_sig.address_of())));
	}

	// Create the optional articulation gather pipeline only when a frame first supplies a reduced-coordinate forest.
	void GpuFrameOutput::EnsureArticulationGatherPipeline()
	{
		if (m_cs_gather_articulations.m_pso != nullptr)
			return;

		auto sig = RootSig(ERootSigFlags::ComputeOnly)
			.U32<cbFrameOutput>(EReg::Params)
			.UAV(EReg::Articulations)
			.UAV(EReg::ArticulationStates)
			.UAV(EReg::ArticulationPositions)
			.UAV(EReg::ArticulationVelocities)
			.UAV(EReg::ArticulationAccelerations)
			.UAV(EReg::OutputArticulations)
			.UAV(EReg::OutputPositions)
			.UAV(EReg::OutputVelocities)
			.UAV(EReg::OutputAccelerations)
			;

		m_cs_gather_articulations.m_sig = sig.Create(m_gpu, "Physics:GatherFrameArticulationsSig");
		m_cs_gather_articulations.m_pso = ComputePSO(m_cs_gather_articulations.m_sig.get(), shader_code::gather_frame_articulations).Create(m_gpu, "Physics:GatherFrameArticulationsPSO");
	}

	// Grow resources before command recording references the current allocation.
	void GpuFrameOutput::ResizeBuffers(CmdList& cmd_list, int64_t capacity, bool requires_substep_state)
	{
		if (requires_substep_state && m_r_substep_state == nullptr)
			m_r_substep_state = m_gpu.CreateResource(ResDesc::Buf<GpuSubstepOutputState>(1, {}).usage(EUsage::UnorderedAccess), cmd_list, "Physics:SubstepOutputState");

		if (m_r_output != nullptr && capacity <= m_capacity)
			return;

		auto word_count = static_cast<int>((capacity + sizeof(uint32_t) - 1) / sizeof(uint32_t));
		m_r_output = m_gpu.CreateResource(ResDesc::Buf<uint32_t>(word_count, {}).usage(EUsage::UnorderedAccess), cmd_list, "Physics:FrameOutput");
		m_capacity = static_cast<int64_t>(word_count) * sizeof(uint32_t);
	}

	// Reset aggregate state and establish the packed layout before any substep writes output.
	void GpuFrameOutput::BeginFrame(GpuJob& job, int body_count, int event_capacity, int substep_count)
	{
		BeginFrame(job, body_count, event_capacity, substep_count, GpuArticulationMidpointOutput{});
	}

	// Reset aggregate state and establish rigid, event, and articulation sections before any substep writes output.
	void GpuFrameOutput::BeginFrame(GpuJob& job, int body_count, int event_capacity, int substep_count, GpuArticulationMidpointOutput const& articulations)
	{
		if (body_count < 0 || event_capacity < 0 || substep_count < 1)
			throw std::runtime_error("GPU frame output requires non-negative capacities and at least one substep");
		if (articulations.m_articulation_count < 0 || articulations.m_position_count < 0 || articulations.m_velocity_count < 0)
			throw std::runtime_error("GPU frame output articulation counts must be non-negative");
		if (articulations.m_articulation_count == 0 && (articulations.m_position_count != 0 || articulations.m_velocity_count != 0))
			throw std::runtime_error("GPU frame output cannot gather generalized state without articulations");

		auto body_offset = static_cast<int64_t>(sizeof(GpuFrameOutputHeader));
		auto event_offset = body_offset + static_cast<int64_t>(body_count) * static_cast<int64_t>(sizeof(GpuRigidBody));
		auto event_end = event_offset + static_cast<int64_t>(event_capacity) * static_cast<int64_t>(sizeof(GpuCollisionEvent));
		auto articulation_offset = FrameOutputAlign(event_end);
		auto position_offset = articulation_offset + static_cast<int64_t>(articulations.m_articulation_count) * static_cast<int64_t>(sizeof(GpuArticulationFrameOutput));
		auto velocity_offset = FrameOutputAlign(position_offset + static_cast<int64_t>(articulations.m_position_count) * static_cast<int64_t>(sizeof(float)));
		auto acceleration_offset = FrameOutputAlign(velocity_offset + static_cast<int64_t>(articulations.m_velocity_count) * static_cast<int64_t>(sizeof(float)));
		auto readback_size = acceleration_offset + static_cast<int64_t>(articulations.m_velocity_count) * static_cast<int64_t>(sizeof(float));
		auto resource_size = std::max(
			readback_size,
			event_offset + static_cast<int64_t>(std::max(1, event_capacity)) * static_cast<int64_t>(sizeof(GpuCollisionEvent)));

		// Empty generalized sections still bind root UAV descriptors, so keep their shared end address inside the resource.
		if (articulations.m_articulation_count != 0 && articulations.m_velocity_count == 0)
			resource_size = std::max(resource_size, readback_size + int64_t{64});

		m_layout = GpuFrameOutputLayout{
			.m_body_count = body_count,
			.m_event_capacity = event_capacity,
			.m_articulation_count = articulations.m_articulation_count,
			.m_position_count = articulations.m_position_count,
			.m_velocity_count = articulations.m_velocity_count,
			.m_pad0 = 0,
			.m_body_offset = body_offset,
			.m_event_offset = event_offset,
			.m_articulation_offset = articulation_offset,
			.m_position_offset = position_offset,
			.m_velocity_offset = velocity_offset,
			.m_acceleration_offset = acceleration_offset,
			.m_readback_size = readback_size,
			.m_resource_size = resource_size,
		};
		if (articulations.m_articulation_count != 0)
			EnsureArticulationGatherPipeline();
		ResizeBuffers(job.m_cmd_list, resource_size, body_count != 0);

		// Only the header needs clearing; logical counts make stale body/event storage unreachable.
		auto header = GpuFrameOutputHeader{
			.event_capacity = event_capacity,
			.pair_limit_substep = -1,
			.contact_limit_substep = -1,
			.event_overflow_substep = -1,
			.substep_count = substep_count,
		};
		auto upload = job.m_upload.Alloc<GpuFrameOutputHeader>(1);
		*upload.ptr<GpuFrameOutputHeader>() = header;

		job.m_barriers.Transition(m_r_output.get(), D3D12_RESOURCE_STATE_COPY_DEST).Commit();
		job.m_cmd_list.CopyBufferRegion(m_r_output.get(), 0, upload);
		job.m_barriers.Transition(m_r_output.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS).Commit();
	}

	// Retain counters and optional resolved contacts before the next substep resets transient collision buffers.
	void GpuFrameOutput::CaptureSubstep(GpuJob& job, int max_pairs, int max_contacts, int substep_index, int substep_count, bool collect_events, ID3D12Resource* counters, ID3D12Resource* contacts, ID3D12Resource* contact_order, ID3D12Resource* contact_dispatch)
	{
		if (m_r_output == nullptr || m_r_substep_state == nullptr)
			throw std::runtime_error("GPU frame output must begin before capturing a substep");

		auto cb = cbFrameOutput{
			.max_pairs = max_pairs,
			.max_contacts = max_contacts,
			.event_capacity = m_layout.m_event_capacity,
			.collect_events = collect_events ? 1 : 0,
			.substep_index = substep_index,
			.substep_count = substep_count,
			.body_count = m_layout.m_body_count,
		};

		// Snapshot raw counts and reserve an event range with one thread so substep ordering is stable.
		job.m_barriers.Transition(counters, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_output.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_substep_state.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Commit();

		job.m_cmd_list.SetPipelineState(m_cs_prepare_substep.m_pso.get());
		job.m_cmd_list.SetComputeRootSignature(m_cs_prepare_substep.m_sig.get());
		job.m_cmd_list.AddComputeRoot32BitConstants(cb);
		job.m_cmd_list.AddComputeRootUnorderedAccessView(counters->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_output->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_substep_state->GetGPUVirtualAddress());
		job.m_cmd_list.Dispatch(1, 1, 1);

		job.m_barriers.UAV(counters);
		job.m_barriers.UAV(m_r_substep_state.get());
		job.m_barriers.UAV(m_r_output.get());
		job.m_barriers.Commit();

		if (!collect_events)
			return;

		// Copy only generated contacts using the resolver's bounded indirect dispatch.
		job.m_barriers.Transition(contacts, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(contact_order, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(contact_dispatch, D3D12_RESOURCE_STATE_INDIRECT_ARGUMENT);
		job.m_barriers.Commit();

		job.m_cmd_list.SetPipelineState(m_cs_append_events.m_pso.get());
		job.m_cmd_list.SetComputeRootSignature(m_cs_append_events.m_sig.get());
		job.m_cmd_list.AddComputeRoot32BitConstants(cb);
		job.m_cmd_list.AddComputeRootUnorderedAccessView(contacts->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(contact_order->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_substep_state->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_output->GetGPUVirtualAddress() + m_layout.m_event_offset);
		job.m_cmd_list.ExecuteIndirect(m_cmd_sig.get(), 1, contact_dispatch);

		job.m_barriers.UAV(contacts);
		job.m_barriers.UAV(contact_order);
		job.m_barriers.UAV(m_r_substep_state.get());
		job.m_barriers.UAV(m_r_output.get());
		job.m_barriers.Commit();
	}

	// Gather final bodies and record the frame's sole GPU-to-CPU CopyBufferRegion.
	GpuFrameOutputReadback GpuFrameOutput::GatherAndReadback(GpuJob& job, int body_count, ID3D12Resource* bodies)
	{
		return GatherAndReadback(job, body_count, bodies, GpuArticulationMidpointOutput{});
	}

	// Gather final rigid and articulation state before recording the frame's sole GPU-to-CPU copy.
	GpuFrameOutputReadback GpuFrameOutput::GatherAndReadback(GpuJob& job, int body_count, ID3D12Resource* bodies, GpuArticulationMidpointOutput const& articulations)
	{
		if (body_count != m_layout.m_body_count)
			throw std::runtime_error("GPU frame output body count changed after BeginFrame");
		if (articulations.m_articulation_count != m_layout.m_articulation_count ||
			articulations.m_position_count != m_layout.m_position_count ||
			articulations.m_velocity_count != m_layout.m_velocity_count)
			throw std::runtime_error("GPU frame output articulation dimensions changed after BeginFrame");

		auto cb = cbFrameOutput{
			.body_count = body_count,
			.articulation_count = articulations.m_articulation_count,
			.position_count = articulations.m_position_count,
			.velocity_count = articulations.m_velocity_count,
		};

		// Copy final body state into the contiguous output without disturbing the retained event section.
		if (body_count != 0)
		{
			if (bodies == nullptr)
				throw std::runtime_error("GPU frame output requires a body resource for non-empty rigid state");

			job.m_barriers.Transition(bodies, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Transition(m_r_output.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Commit();

			job.m_cmd_list.SetPipelineState(m_cs_gather_bodies.m_pso.get());
			job.m_cmd_list.SetComputeRootSignature(m_cs_gather_bodies.m_sig.get());
			job.m_cmd_list.AddComputeRoot32BitConstants(cb);
			job.m_cmd_list.AddComputeRootUnorderedAccessView(bodies->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_output->GetGPUVirtualAddress() + m_layout.m_body_offset);
			job.m_cmd_list.Dispatch((body_count + FrameOutputThreadCount - 1) / FrameOutputThreadCount, 1, 1);
			job.m_barriers.UAV(m_r_output.get()).Commit();
		}

		// Compact final root/status and generalized state without copying topology or per-link ABA scratch.
		if (articulations.m_articulation_count != 0)
		{
			if (articulations.m_articulations == nullptr ||
				articulations.m_positions == nullptr ||
				articulations.m_velocities == nullptr ||
				articulations.m_accelerations == nullptr ||
				articulations.m_states == nullptr)
				throw std::runtime_error("GPU frame output articulation resources are incomplete");

			job.m_barriers.Transition(articulations.m_articulations, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Transition(articulations.m_positions, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Transition(articulations.m_velocities, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Transition(articulations.m_accelerations, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Transition(articulations.m_states, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Transition(m_r_output.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Commit();

			job.m_cmd_list.SetPipelineState(m_cs_gather_articulations.m_pso.get());
			job.m_cmd_list.SetComputeRootSignature(m_cs_gather_articulations.m_sig.get());
			job.m_cmd_list.AddComputeRoot32BitConstants(cb);
			job.m_cmd_list.AddComputeRootUnorderedAccessView(articulations.m_articulations->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(articulations.m_states->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(articulations.m_positions->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(articulations.m_velocities->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(articulations.m_accelerations->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_output->GetGPUVirtualAddress() + m_layout.m_articulation_offset);
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_output->GetGPUVirtualAddress() + m_layout.m_position_offset);
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_output->GetGPUVirtualAddress() + m_layout.m_velocity_offset);
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_output->GetGPUVirtualAddress() + m_layout.m_acceleration_offset);
			auto const item_count = std::max({articulations.m_articulation_count, articulations.m_position_count, articulations.m_velocity_count});
			job.m_cmd_list.Dispatch((item_count + FrameOutputThreadCount - 1) / FrameOutputThreadCount, 1, 1);
			job.m_barriers.UAV(m_r_output.get()).Commit();
		}

		// One contiguous copy is the only GPU-to-CPU transfer owned by the core frame pipeline.
		job.m_barriers.Transition(m_r_output.get(), D3D12_RESOURCE_STATE_COPY_SOURCE).Commit();
		auto allocation = job.m_readback.Alloc(m_layout.m_readback_size, alignof(GpuFrameOutputHeader));
		job.m_cmd_list.CopyBufferRegion(allocation, m_r_output.get(), 0);
		job.m_barriers.Transition(m_r_output.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS).Commit();

		return GpuFrameOutputReadback{
			.m_allocation = std::move(allocation),
			.m_layout = m_layout,
		};
	}

	// Access the aggregate header after GPU completion.
	GpuFrameOutputHeader const& GpuFrameOutput::Header(GpuFrameOutputReadback const& readback)
	{
		return *reinterpret_cast<GpuFrameOutputHeader const*>(readback.m_allocation.ptr<uint8_t>());
	}

	// Access final body records after GPU completion.
	std::span<GpuRigidBody const> GpuFrameOutput::Bodies(GpuFrameOutputReadback const& readback)
	{
		auto ptr = reinterpret_cast<GpuRigidBody const*>(readback.m_allocation.ptr<uint8_t>() + readback.m_layout.m_body_offset);
		return { ptr, static_cast<size_t>(readback.m_layout.m_body_count) };
	}

	// Access the bounded event storage; callers clamp the span to Header().event_count.
	std::span<GpuCollisionEvent const> GpuFrameOutput::Events(GpuFrameOutputReadback const& readback)
	{
		auto ptr = reinterpret_cast<GpuCollisionEvent const*>(readback.m_allocation.ptr<uint8_t>() + readback.m_layout.m_event_offset);
		return { ptr, static_cast<size_t>(readback.m_layout.m_event_capacity) };
	}

	// Access compact final root, identity, status, and convergence diagnostics.
	std::span<GpuArticulationFrameOutput const> GpuFrameOutput::Articulations(GpuFrameOutputReadback const& readback)
	{
		auto ptr = reinterpret_cast<GpuArticulationFrameOutput const*>(readback.m_allocation.ptr<uint8_t>() + readback.m_layout.m_articulation_offset);
		return { ptr, static_cast<size_t>(readback.m_layout.m_articulation_count) };
	}

	// Access final reduced joint positions in packed articulation order.
	std::span<float const> GpuFrameOutput::Positions(GpuFrameOutputReadback const& readback)
	{
		auto ptr = reinterpret_cast<float const*>(readback.m_allocation.ptr<uint8_t>() + readback.m_layout.m_position_offset);
		return { ptr, static_cast<size_t>(readback.m_layout.m_position_count) };
	}

	// Access final floating-root and reduced joint velocities in packed articulation order.
	std::span<float const> GpuFrameOutput::Velocities(GpuFrameOutputReadback const& readback)
	{
		auto ptr = reinterpret_cast<float const*>(readback.m_allocation.ptr<uint8_t>() + readback.m_layout.m_velocity_offset);
		return { ptr, static_cast<size_t>(readback.m_layout.m_velocity_count) };
	}

	// Access the final midpoint generalized accelerations corresponding to the velocity range.
	std::span<float const> GpuFrameOutput::Accelerations(GpuFrameOutputReadback const& readback)
	{
		auto ptr = reinterpret_cast<float const*>(readback.m_allocation.ptr<uint8_t>() + readback.m_layout.m_acceleration_offset);
		return { ptr, static_cast<size_t>(readback.m_layout.m_velocity_count) };
	}

	// Custom deleter implementation.
	void Deleter<GpuFrameOutput>::operator()(GpuFrameOutput* p) const
	{
		delete p;
	}
}
