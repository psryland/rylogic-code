//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "pr/physics/integrator/engine_config.h"
#include "src/compute/constraint_solver_gpu.h"
#include "src/compute/shader_code.h"

namespace pr::physics
{
	using namespace ::pr::compute;

	namespace
	{
		// Constant buffer shared by the compiler, colourer, warm-start, and block solver kernels.
		struct alignas(16) cbConstraintSolve
		{
			int block_count;
			int body_count;
			int colour;
			int position_iterations;
			float timestep;
			float relaxation;
			float position_relaxation;
			float position_beta;
			float max_position_speed;
			float regularization;
			float warm_start_factor;
			float warm_start_scale;
			int retain_current_impulses;
			int breakable_count;
			int break_substep_index;
			int pad2;
		};
		static_assert(sizeof(cbConstraintSolve) == 64);

		// Register assignments shared by every persistent-constraint compute entry point.
		struct EReg
		{
			inline static constexpr auto Params = ECBufReg::b0;
			inline static constexpr auto Bodies = EUAVReg::u0;
			inline static constexpr auto Endpoints = ESRVReg::t0;
			inline static constexpr auto Descriptors = ESRVReg::t1;
			inline static constexpr auto Blocks = EUAVReg::u1;
			inline static constexpr auto Rows = EUAVReg::u2;
			inline static constexpr auto Overflow = EUAVReg::u3;
			inline static constexpr auto PseudoVelocities = EUAVReg::u4;
			inline static constexpr auto BreakStates = EUAVReg::u5;
		};

		// Compare endpoint semantics while ignoring frame-local body-index changes caused only by submission order.
		bool EndpointChanged(GpuConstraintEndpoint const& lhs, GpuConstraintEndpoint const& rhs)
		{
			auto const semantic_flags = ~GpuConstraintEndpointFlags_ResetWarmStart;
			return
				(lhs.flags & semantic_flags) != (rhs.flags & semantic_flags) ||
				lhs.generation != rhs.generation ||
				lhs.break_force != rhs.break_force ||
				lhs.break_torque != rhs.break_torque;
		}
	}

	// Create pipeline state without allocating feature-dependent buffers.
	GpuConstraintSolver::GpuConstraintSolver(Gpu& gpu, EngineConfig const& config)
		: m_gpu(gpu)
		, m_config(config)
		, m_cs_compile()
		, m_cs_assign_colours()
		, m_cs_apply_warm_start()
		, m_cs_clear_pseudo_velocity()
		, m_cs_solve_position()
		, m_cs_apply_position()
		, m_cs_solve_velocity()
		, m_cs_detect_breakage()
		, m_r_endpoints()
		, m_r_descriptors()
		, m_r_blocks()
		, m_r_rows()
		, m_r_overflow()
		, m_r_pseudo_velocities()
		, m_r_break_states()
		, m_endpoint_shadow()
		, m_endpoint_identity_shadow()
		, m_descriptor_shadow()
		, m_source()
		, m_capacity()
		, m_body_capacity()
		, m_break_capacity()
		, m_slot_count()
		, m_active_count()
		, m_breakable_count()
		, m_previous_timestep()
		, m_frame_warm_start_scale()
		, m_retain_current_impulses()
		, m_break_substep_index(-1)
	{
		// Every entry point uses one root layout so phase changes only replace pipeline state.
		auto sig = RootSig(ERootSigFlags::ComputeOnly)
			.U32<cbConstraintSolve>(EReg::Params)
			.UAV(EReg::Bodies)
			.SRV(EReg::Endpoints)
			.SRV(EReg::Descriptors)
			.UAV(EReg::Blocks)
			.UAV(EReg::Rows)
			.UAV(EReg::Overflow)
			.UAV(EReg::PseudoVelocities)
			.UAV(EReg::BreakStates);
		auto const root_sig = sig.Create(m_gpu, "Physics:ConstraintSolverSig");

		auto compile_step = [&](ComputeStep& step, shader_code::ByteCode const& bytecode, char const* name)
		{
			auto const pso_name = FmtS("Physics:%sPSO", name);
			step.m_sig = root_sig;
			step.m_pso = ComputePSO(step.m_sig.get(), bytecode).Create(m_gpu, pso_name);
		};
		compile_step(m_cs_compile, shader_code::compile_constraints, "CompileConstraints");
		compile_step(m_cs_assign_colours, shader_code::assign_constraint_colours, "AssignConstraintColours");
		compile_step(m_cs_apply_warm_start, shader_code::apply_constraint_warm_start, "ApplyConstraintWarmStart");
		compile_step(m_cs_clear_pseudo_velocity, shader_code::clear_constraint_pseudo_velocity, "ClearConstraintPseudoVelocity");
		compile_step(m_cs_solve_position, shader_code::solve_constraint_position, "SolveConstraintPosition");
		compile_step(m_cs_apply_position, shader_code::apply_constraint_position, "ApplyConstraintPosition");
		compile_step(m_cs_solve_velocity, shader_code::solve_constraint_velocity, "SolveConstraintVelocity");
		compile_step(m_cs_detect_breakage, shader_code::detect_broken_constraints, "DetectBrokenConstraints");
	}

	// Create or grow feature-dependent buffers while preserving stable-slot capacity.
	bool GpuConstraintSolver::ResizeBuffers(CmdList& cmd_list, int capacity)
	{
		capacity = std::max(1, capacity);
		if (m_r_endpoints != nullptr && m_capacity >= capacity)
			return false;

		m_r_endpoints = m_gpu.CreateResource(ResDesc::Buf<GpuConstraintEndpoint>(capacity, {}), cmd_list, "Physics:ConstraintEndpoints");
		m_r_descriptors = m_gpu.CreateResource(ResDesc::Buf<GpuD6ConstraintDesc>(capacity, {}), cmd_list, "Physics:ConstraintDescriptors");
		m_r_blocks = m_gpu.CreateResource(ResDesc::Buf<GpuConstraintBlock>(capacity, {}).usage(EUsage::UnorderedAccess), cmd_list, "Physics:ConstraintBlocks");
		m_r_rows = m_gpu.CreateResource(ResDesc::Buf<GpuConstraintRow>(GpuConstraintRowsPerBlock * capacity, {}).usage(EUsage::UnorderedAccess), cmd_list, "Physics:ConstraintRows");
		m_r_overflow = m_gpu.CreateResource(ResDesc::Buf<uint32_t>(1, {}).usage(EUsage::UnorderedAccess), cmd_list, "Physics:ConstraintColourOverflow");
		m_capacity = capacity;
		return true;
	}

	// Create or grow shared rigid pseudo-twist storage for either independent or coupled position correction.
	void GpuConstraintSolver::EnsurePseudoVelocityStorage(CmdList& cmd_list, int body_count)
	{
		if (body_count < 0)
			throw std::invalid_argument("Constraint pseudo-velocity body count cannot be negative");
		if (m_r_pseudo_velocities != nullptr && m_body_capacity >= body_count)
			return;

		m_body_capacity = std::max(1, body_count);
		m_r_pseudo_velocities = m_gpu.CreateResource(ResDesc::Buf<GpuConstraintPseudoVelocity>(m_body_capacity, {}).usage(EUsage::UnorderedAccess), cmd_list, "Physics:ConstraintPseudoVelocities");
	}

	// Create or geometrically grow the optional per-slot overload latch.
	void GpuConstraintSolver::EnsureBreakStateStorage(CmdList& cmd_list, int slot_count)
	{
		if (slot_count < 0)
			throw std::invalid_argument("Constraint break-state slot count cannot be negative");
		if (m_r_break_states != nullptr && m_break_capacity >= slot_count)
			return;

		auto const doubled_capacity = m_break_capacity <= std::numeric_limits<int>::max() / 2
			? 2 * m_break_capacity
			: std::numeric_limits<int>::max();
		m_break_capacity = std::max(slot_count, std::max(1, doubled_capacity));
		m_r_break_states = m_gpu.CreateResource(ResDesc::Buf<GpuConstraintBreakState>(m_break_capacity, {}).usage(EUsage::UnorderedAccess), cmd_list, "Physics:ConstraintBreakStates");
	}

	// Upload shared frame-local endpoints and changed persistent descriptors, returning whether independent rigid work is active.
	bool GpuConstraintSolver::Upload(GpuJob& job, GpuConstraintUpload const& upload)
	{
		m_slot_count = static_cast<int>(upload.m_endpoints.size());
		m_active_count = static_cast<int>(upload.m_rigid_active_count);
		m_breakable_count = static_cast<int>(upload.m_breakable_count);
		if (m_slot_count == 0)
		{
			m_r_break_states = nullptr;
			m_break_capacity = 0;
			m_breakable_count = 0;
			m_source = upload.m_source;
			m_previous_timestep = 0.0f;
			m_frame_warm_start_scale = 0.0f;
			return false;
		}
		if (upload.m_descriptors.size() != upload.m_endpoints.size())
			throw std::invalid_argument("GPU constraint endpoint and descriptor slot counts must match");
		if (upload.m_endpoint_identities.size() != upload.m_endpoints.size())
			throw std::invalid_argument("GPU constraint endpoint identities and endpoint slot counts must match");
		if (m_breakable_count < 0 || m_breakable_count > m_slot_count)
			throw std::invalid_argument("GPU breakable constraint count is outside the stable-slot range");

		auto const resized = ResizeBuffers(job.m_cmd_list, m_slot_count);
		auto endpoints = upload.m_endpoints;
		auto descriptor_dirty = std::vector<bool>(m_slot_count, resized || m_descriptor_shadow.size() != upload.m_descriptors.size());
		auto const reset_all = resized || m_source != upload.m_source;
		auto reset_runtime = std::vector<bool>(m_slot_count, reset_all);

		// Mark only semantically changed stable slots for warm-start invalidation; packed-body reordering is harmless.
		for (int index = 0; index != m_slot_count; ++index)
		{
			auto const previously_enabled =
				!reset_all &&
				index < isize(m_endpoint_shadow) &&
				AllSet(m_endpoint_shadow[index].flags, GpuConstraintEndpointFlags_Enabled);
			auto const currently_enabled = AllSet(endpoints[index].flags, GpuConstraintEndpointFlags_Enabled);
			auto const descriptor_changed =
				resized ||
				index >= isize(m_descriptor_shadow) ||
				std::memcmp(&m_descriptor_shadow[index], &upload.m_descriptors[index], sizeof(GpuD6ConstraintDesc)) != 0;
			auto const endpoint_identity_changed =
				index >= isize(m_endpoint_identity_shadow) ||
				m_endpoint_identity_shadow[index] != upload.m_endpoint_identities[index];
			descriptor_dirty[index] = descriptor_dirty[index] || descriptor_changed;
			auto const endpoint_changed =
				reset_all ||
				index >= isize(m_endpoint_shadow) ||
				EndpointChanged(m_endpoint_shadow[index], endpoints[index]) ||
				endpoint_identity_changed;
			if (descriptor_changed || endpoint_changed)
				endpoints[index].flags |= GpuConstraintEndpointFlags_ResetWarmStart;

			// A newly active slot must not inherit a break latch or row state from an earlier occupant or an immediately repaired constraint.
			reset_runtime[index] = reset_runtime[index] || (currently_enabled && !previously_enabled);
		}

		// Clear only newly valid runtime ranges, while a source or capacity replacement resets every retained slot.
		auto const any_runtime_reset = std::ranges::find(reset_runtime, true) != reset_runtime.end();
		if (any_runtime_reset)
		{
			job.m_barriers.Transition(m_r_blocks.get(), D3D12_RESOURCE_STATE_COPY_DEST);
			job.m_barriers.Transition(m_r_rows.get(), D3D12_RESOURCE_STATE_COPY_DEST);
			job.m_barriers.Commit();

			for (int begin = 0; begin != m_slot_count;)
			{
				if (!reset_runtime[begin])
				{
					++begin;
					continue;
				}

				auto end = begin + 1;
				while (end != m_slot_count && reset_runtime[end])
					++end;

				auto const count = end - begin;
				auto block_upload = job.m_upload.Alloc<GpuConstraintBlock>(count);
				std::fill_n(block_upload.ptr<GpuConstraintBlock>(), count, GpuConstraintBlock{
					.body_idx_a = -1,
					.body_idx_b = -1,
					.colour = MaxColours,
				});
				job.m_cmd_list.CopyBufferRegion(m_r_blocks.get(), static_cast<uint64_t>(begin) * sizeof(GpuConstraintBlock), block_upload);

				auto const row_count = GpuConstraintRowsPerBlock * count;
				auto row_upload = job.m_upload.Alloc<GpuConstraintRow>(row_count);
				std::fill_n(row_upload.ptr<GpuConstraintRow>(), row_count, GpuConstraintRow{});
				job.m_cmd_list.CopyBufferRegion(m_r_rows.get(), static_cast<uint64_t>(GpuConstraintRowsPerBlock * begin) * sizeof(GpuConstraintRow), row_upload);
				begin = end;
			}

			job.m_barriers.Transition(m_r_blocks.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Transition(m_r_rows.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Commit();
		}

		// Endpoint indices are frame-local, so upload the complete compact endpoint stream every submitted constrained frame.
		auto const any_descriptor_dirty = std::ranges::find(descriptor_dirty, true) != descriptor_dirty.end();
		job.m_barriers.Transition(m_r_endpoints.get(), D3D12_RESOURCE_STATE_COPY_DEST);
		if (any_descriptor_dirty)
			job.m_barriers.Transition(m_r_descriptors.get(), D3D12_RESOURCE_STATE_COPY_DEST);
		job.m_barriers.Commit();
		{
			auto endpoint_upload = job.m_upload.Alloc<GpuConstraintEndpoint>(m_slot_count);
			memcpy(endpoint_upload.ptr<GpuConstraintEndpoint>(), endpoints.data(), endpoints.size() * sizeof(GpuConstraintEndpoint));
			job.m_cmd_list.CopyBufferRegion(m_r_endpoints.get(), 0, endpoint_upload);
		}

		// Merge adjacent changed descriptor slots so persistent parameter edits produce minimal upload copies.
		for (int begin = 0; begin != m_slot_count;)
		{
			if (!descriptor_dirty[begin])
			{
				++begin;
				continue;
			}

			auto end = begin + 1;
			while (end != m_slot_count && descriptor_dirty[end])
				++end;

			auto const count = end - begin;
			auto descriptor_upload = job.m_upload.Alloc<GpuD6ConstraintDesc>(count);
			memcpy(descriptor_upload.ptr<GpuD6ConstraintDesc>(), upload.m_descriptors.data() + begin, count * sizeof(GpuD6ConstraintDesc));
			job.m_cmd_list.CopyBufferRegion(m_r_descriptors.get(), static_cast<uint64_t>(begin) * sizeof(GpuD6ConstraintDesc), descriptor_upload);
			begin = end;
		}
		job.m_barriers.Transition(m_r_endpoints.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		if (any_descriptor_dirty)
			job.m_barriers.Transition(m_r_descriptors.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Commit();

		// Break latches are frame-local and exist only when at least one active descriptor has a finite threshold.
		if (m_breakable_count != 0)
		{
			EnsureBreakStateStorage(job.m_cmd_list, m_slot_count);
			auto break_upload = job.m_upload.Alloc<GpuConstraintBreakState>(m_slot_count);
			for (int index = 0; index != m_slot_count; ++index)
			{
				break_upload.ptr<GpuConstraintBreakState>()[index] = GpuConstraintBreakState{
					.generation = upload.m_endpoints[index].generation,
					.substep_index = -1,
				};
			}
			job.m_barriers.Transition(m_r_break_states.get(), D3D12_RESOURCE_STATE_COPY_DEST).Commit();
			job.m_cmd_list.CopyBufferRegion(m_r_break_states.get(), 0, break_upload);
			job.m_barriers.Transition(m_r_break_states.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS).Commit();
		}
		else
		{
			m_r_break_states = nullptr;
			m_break_capacity = 0;
		}

		m_endpoint_shadow = upload.m_endpoints;
		m_endpoint_identity_shadow = upload.m_endpoint_identities;
		m_descriptor_shadow = upload.m_descriptors;
		m_source = upload.m_source;
		return m_active_count != 0;
	}

	// Bind the common constraint root signature and resources for one compute step.
	void GpuConstraintSolver::Bind(GpuJob& job, ComputeStep& step, float timestep, int body_count, int colour, int position_iterations, D3DPtr<ID3D12Resource> bodies)
	{
		auto const constants = cbConstraintSolve{
			.block_count = m_slot_count,
			.body_count = body_count,
			.colour = colour,
			.position_iterations = position_iterations,
			.timestep = timestep,
			.relaxation = m_config.constraint_relaxation,
			.position_relaxation = m_config.constraint_position_relaxation,
			.position_beta = m_config.constraint_position_beta,
			.max_position_speed = m_config.constraint_max_position_speed,
			.regularization = m_config.constraint_regularization,
			.warm_start_factor = m_config.constraint_warm_start_factor,
			.warm_start_scale = m_frame_warm_start_scale,
			.retain_current_impulses = m_retain_current_impulses ? 1 : 0,
			.breakable_count = m_breakable_count,
			.break_substep_index = m_break_substep_index,
			.pad2 = 0,
		};
		job.m_cmd_list.SetPipelineState(step.m_pso.get());
		job.m_cmd_list.SetComputeRootSignature(step.m_sig.get());
		job.m_cmd_list.AddComputeRoot32BitConstants(constants);
		job.m_cmd_list.AddComputeRootUnorderedAccessView(bodies->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView(m_r_endpoints->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView(m_r_descriptors->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_blocks->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_rows->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_overflow->GetGPUVirtualAddress());
		auto* pseudo_velocities = m_r_pseudo_velocities != nullptr ? m_r_pseudo_velocities.get() : m_r_overflow.get();
		job.m_cmd_list.AddComputeRootUnorderedAccessView(pseudo_velocities->GetGPUVirtualAddress());
		auto* break_states = m_r_break_states != nullptr ? m_r_break_states.get() : m_r_overflow.get();
		job.m_cmd_list.AddComputeRootUnorderedAccessView(break_states->GetGPUVirtualAddress());
	}

	// Insert UAV ordering between dependent compiler, colouring, and solver passes.
	void GpuConstraintSolver::CommitUavBarriers(GpuJob& job, D3DPtr<ID3D12Resource> bodies)
	{
		job.m_barriers.UAV(bodies.get());
		job.m_barriers.UAV(m_r_blocks.get());
		job.m_barriers.UAV(m_r_rows.get());
		job.m_barriers.UAV(m_r_overflow.get());
		job.m_barriers.UAV(m_r_pseudo_velocities.get());
		job.m_barriers.Commit();
	}

	// Compile current world-space rows and graph-colour active blocks, optionally retaining already-applied current-frame impulses for a continuation sweep.
	void GpuConstraintSolver::Prepare(GpuJob& job, float timestep, int body_count, D3DPtr<ID3D12Resource> bodies, bool retain_current_impulses)
	{
		if (!Active())
			return;

		pix::BeginEvent(job.m_cmd_list.get(), 0xFF8A5FD3, "Physics::CompileConstraints");

		// Allocate split-correction state only after the optional lane has active work.
		EnsurePseudoVelocityStorage(job.m_cmd_list, body_count);

		// A continuation recompiles changed geometry but retains the accumulator that has already been applied to authoritative momentum.
		m_retain_current_impulses = retain_current_impulses;
		if (m_retain_current_impulses)
		{
			m_frame_warm_start_scale = 1.0f;
		}
		else
		{
			// Cross-frame warm starts are scaled only across ordinary timestep changes.
			m_frame_warm_start_scale = 0.0f;
			if (m_previous_timestep > 0.0f)
			{
				auto const timestep_ratio = timestep / m_previous_timestep;
				if (timestep_ratio >= 0.25f && timestep_ratio <= 4.0f)
					m_frame_warm_start_scale = m_config.constraint_warm_start_factor * timestep_ratio;
			}
			m_previous_timestep = timestep;
		}

		job.m_barriers.Transition(bodies.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_endpoints.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(m_r_descriptors.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(m_r_blocks.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_rows.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_overflow.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_pseudo_velocities.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Commit();

		auto const group_count = static_cast<UINT>((m_slot_count + ConstraintThreadCount - 1) / ConstraintThreadCount);
		Bind(job, m_cs_compile, timestep, body_count, 0, std::max(1, m_config.push_out_iterations), bodies);
		job.m_cmd_list.Dispatch(group_count, 1, 1);
		CommitUavBarriers(job, bodies);

		Bind(job, m_cs_assign_colours, timestep, body_count, 0, std::max(1, m_config.push_out_iterations), bodies);
		job.m_cmd_list.Dispatch(1, 1, 1);
		CommitUavBarriers(job, bodies);

		// Each constrained frame starts with zero pseudo velocity independently of physical momentum.
		Bind(job, m_cs_clear_pseudo_velocity, timestep, body_count, 0, std::max(1, m_config.push_out_iterations), bodies);
		job.m_cmd_list.Dispatch(static_cast<UINT>((body_count + ConstraintThreadCount - 1) / ConstraintThreadCount), 1, 1);
		CommitUavBarriers(job, bodies);
		pix::EndEvent(job.m_cmd_list.get());
	}

	// Apply retained physical impulses before the new velocity solve.
	void GpuConstraintSolver::ApplyWarmStart(GpuJob& job, float timestep, int body_count, D3DPtr<ID3D12Resource> bodies)
	{
		if (!Active() || m_retain_current_impulses || m_config.constraint_warm_start_factor <= 0.0f)
			return;

		auto const group_count = static_cast<UINT>((m_slot_count + ConstraintThreadCount - 1) / ConstraintThreadCount);
		for (int colour = 0; colour != MaxColours; ++colour)
		{
			Bind(job, m_cs_apply_warm_start, timestep, body_count, colour, std::max(1, m_config.push_out_iterations), bodies);
			job.m_cmd_list.Dispatch(group_count, 1, 1);
			CommitUavBarriers(job, bodies);
		}
	}

	// Execute one complete coloured split-position sweep without changing physical momentum.
	void GpuConstraintSolver::SolvePositionIteration(GpuJob& job, float timestep, int body_count, int position_iterations, D3DPtr<ID3D12Resource> bodies)
	{
		if (!Active() || position_iterations <= 0)
			return;

		auto const group_count = static_cast<UINT>((m_slot_count + ConstraintThreadCount - 1) / ConstraintThreadCount);
		for (int colour = 0; colour != MaxColours; ++colour)
		{
			Bind(job, m_cs_solve_position, timestep, body_count, colour, position_iterations, bodies);
			job.m_cmd_list.Dispatch(group_count, 1, 1);
			CommitUavBarriers(job, bodies);
		}
	}

	// Integrate converged split pseudo twists once while preserving physical momentum.
	void GpuConstraintSolver::ApplyPosition(GpuJob& job, float timestep, int body_count, int position_iterations, D3DPtr<ID3D12Resource> bodies)
	{
		if (!Active() || position_iterations <= 0 || body_count == 0)
			return;

		Bind(job, m_cs_apply_position, timestep, body_count, 0, position_iterations, bodies);
		job.m_cmd_list.Dispatch(static_cast<UINT>((body_count + ConstraintThreadCount - 1) / ConstraintThreadCount), 1, 1);
		CommitUavBarriers(job, bodies);
	}

	// Execute one complete coloured physical block-PGS sweep.
	void GpuConstraintSolver::SolveVelocityIteration(GpuJob& job, float timestep, int body_count, D3DPtr<ID3D12Resource> bodies)
	{
		if (!Active())
			return;

		auto const group_count = static_cast<UINT>((m_slot_count + ConstraintThreadCount - 1) / ConstraintThreadCount);
		for (int colour = 0; colour != MaxColours; ++colour)
		{
			Bind(job, m_cs_solve_velocity, timestep, body_count, colour, std::max(1, m_config.push_out_iterations), bodies);
			job.m_cmd_list.Dispatch(group_count, 1, 1);
			CommitUavBarriers(job, bodies);
		}
	}

	// Latch force or torque threshold crossings after one main or selective velocity solve.
	void GpuConstraintSolver::DetectBreakage(GpuJob& job, float timestep, int substep_index, int body_count, D3DPtr<ID3D12Resource> bodies)
	{
		if (m_breakable_count == 0)
			return;
		if (!(timestep > 0.0f) || !std::isfinite(timestep))
			throw std::invalid_argument("Constraint break detection requires a finite positive timestep");
		if (substep_index < 0)
			throw std::invalid_argument("Constraint break detection requires a non-negative substep index");
		if (m_r_break_states == nullptr)
			throw std::logic_error("Constraint break detection requires its optional state resource");

		// All six canonical impulses are already committed before this pass, so one lane can compute and latch each block's load independently.
		m_break_substep_index = substep_index;
		job.m_barriers.Transition(bodies.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_endpoints.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(m_r_blocks.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_rows.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_break_states.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Commit();

		Bind(job, m_cs_detect_breakage, timestep, body_count, 0, std::max(1, m_config.push_out_iterations), bodies);
		auto const group_count = static_cast<UINT>((m_slot_count + ConstraintThreadCount - 1) / ConstraintThreadCount);
		job.m_cmd_list.Dispatch(group_count, 1, 1);
		job.m_barriers.UAV(m_r_blocks.get());
		job.m_barriers.UAV(m_r_break_states.get());
		job.m_barriers.Commit();
		m_break_substep_index = -1;
	}

	// Return the optional full stable-slot break stream for the final frame gather.
	GpuConstraintBreakOutput GpuConstraintSolver::BreakOutput()
	{
		if (m_breakable_count != 0)
		{
			return GpuConstraintBreakOutput{
				.m_states = m_r_break_states.get(),
				.m_slot_count = m_slot_count,
			};
		}

		return {};
	}

	// Mirror completed breaks into the endpoint shadow so an immediate explicit repair invalidates stale GPU runtime state.
	void GpuConstraintSolver::AcknowledgeBreaks(std::span<GpuConstraintBreakState const> states)
	{
		if (states.empty())
			return;
		if (states.size() != m_endpoint_shadow.size())
			throw std::invalid_argument("Constraint break acknowledgement does not match the retained stable-slot stream");

		for (auto [state, index] : with_index(states))
		{
			if (!AllSet(state.flags, GpuConstraintBreakFlags_Broken))
				continue;
			if (state.generation != m_endpoint_shadow[index].generation)
				throw std::runtime_error("Constraint break acknowledgement generation does not match the uploaded endpoint");

			m_endpoint_shadow[index].flags = SetFlag(m_endpoint_shadow[index].flags, GpuConstraintEndpointFlags_Enabled, false);
		}
	}

	// Force the next upload to clear retained blocks after an abandoned or failed frame.
	void GpuConstraintSolver::InvalidateRuntimeState()
	{
		m_source = nullptr;
		m_previous_timestep = 0.0f;
		m_frame_warm_start_scale = 0.0f;
		m_retain_current_impulses = false;
		m_break_substep_index = -1;
		m_r_break_states = nullptr;
		m_break_capacity = 0;
		m_breakable_count = 0;
	}

	// True when the most recently uploaded set contains enabled persistent constraints.
	bool GpuConstraintSolver::Active() const
	{
		return m_active_count != 0;
	}

	// Invalidate retained frame timing when a step does not submit this optional solver lane.
	void GpuConstraintSolver::Deactivate()
	{
		m_active_count = 0;
		m_breakable_count = 0;
		m_previous_timestep = 0.0f;
		m_frame_warm_start_scale = 0.0f;
		m_retain_current_impulses = false;
		m_break_substep_index = -1;
		m_r_break_states = nullptr;
		m_break_capacity = 0;
	}

	// CPU-side testing: upload, solve, and read back bodies and runtime state in one GPU job.
	void GpuConstraintSolver::Solve(GpuJob& job, float timestep, GpuConstraintUpload const& upload, std::span<GpuRigidBody> bodies, std::span<GpuConstraintBlock> blocks, std::span<GpuConstraintRow> rows, std::span<GpuConstraintBreakState> break_states)
	{
		if (blocks.size() != 0 && blocks.size() < upload.m_endpoints.size())
			throw std::invalid_argument("Constraint block readback span is smaller than the stable-slot stream");
		if (rows.size() != 0 && rows.size() < GpuConstraintRowsPerBlock * upload.m_endpoints.size())
			throw std::invalid_argument("Constraint row readback span is smaller than the canonical stable-slot stream");
		if (break_states.size() != 0 && break_states.size() < upload.m_endpoints.size())
			throw std::invalid_argument("Constraint break-state readback span is smaller than the stable-slot stream");

		auto const body_count = static_cast<int>(bodies.size());
		auto r_bodies = m_gpu.CreateResource(ResDesc::Buf<GpuRigidBody>(std::max(1, body_count), {}).usage(EUsage::UnorderedAccess), job.m_cmd_list, "Physics:ConstraintTestBodies");
		job.m_barriers.Transition(r_bodies.get(), D3D12_RESOURCE_STATE_COPY_DEST);
		job.m_barriers.Commit();
		auto body_upload = job.m_upload.Alloc<GpuRigidBody>(std::max(1, body_count));
		memcpy(body_upload.ptr<GpuRigidBody>(), bodies.data(), bodies.size_bytes());
		job.m_cmd_list.CopyBufferRegion(r_bodies.get(), 0, body_upload);
		job.m_barriers.Transition(r_bodies.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Commit();

		if (Upload(job, upload))
		{
			Prepare(job, timestep, body_count, r_bodies);
			ApplyWarmStart(job, timestep, body_count, r_bodies);
			for (int iteration = 0; iteration != std::max(0, m_config.push_out_iterations); ++iteration)
				SolvePositionIteration(job, timestep, body_count, m_config.push_out_iterations, r_bodies);
			ApplyPosition(job, timestep, body_count, m_config.push_out_iterations, r_bodies);
			for (int iteration = 0; iteration != std::max(0, m_config.solver_iterations); ++iteration)
				SolveVelocityIteration(job, timestep, body_count, r_bodies);
			DetectBreakage(job, timestep, 0, body_count, r_bodies);
		}
		// Gather optional body, block, row, and break diagnostics into the same readback transaction.
		job.m_barriers.Transition(r_bodies.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
		if (!blocks.empty())
			job.m_barriers.Transition(m_r_blocks.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
		if (!rows.empty())
			job.m_barriers.Transition(m_r_rows.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
		if (!break_states.empty() && m_r_break_states != nullptr)
			job.m_barriers.Transition(m_r_break_states.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
		job.m_barriers.Commit();
		auto body_readback = job.m_readback.Alloc<GpuRigidBody>(std::max(1, body_count));
		job.m_cmd_list.CopyBufferRegion(body_readback, r_bodies.get(), 0);
		auto block_readback = ReadbackAlloc{};
		auto row_readback = ReadbackAlloc{};
		auto break_readback = ReadbackAlloc{};
		if (!blocks.empty())
		{
			block_readback = job.m_readback.Alloc<GpuConstraintBlock>(static_cast<int>(upload.m_endpoints.size()));
			job.m_cmd_list.CopyBufferRegion(block_readback, m_r_blocks.get(), 0);
		}
		if (!rows.empty())
		{
			row_readback = job.m_readback.Alloc<GpuConstraintRow>(GpuConstraintRowsPerBlock * static_cast<int>(upload.m_endpoints.size()));
			job.m_cmd_list.CopyBufferRegion(row_readback, m_r_rows.get(), 0);
		}
		if (!break_states.empty() && m_r_break_states != nullptr)
		{
			break_readback = job.m_readback.Alloc<GpuConstraintBreakState>(static_cast<int>(upload.m_endpoints.size()));
			job.m_cmd_list.CopyBufferRegion(break_readback, m_r_break_states.get(), 0);
		}
		job.Run();

		memcpy(bodies.data(), body_readback.ptr<GpuRigidBody>(), bodies.size_bytes());
		if (!blocks.empty())
			memcpy(blocks.data(), block_readback.ptr<GpuConstraintBlock>(), upload.m_endpoints.size() * sizeof(GpuConstraintBlock));
		if (!rows.empty())
			memcpy(rows.data(), row_readback.ptr<GpuConstraintRow>(), GpuConstraintRowsPerBlock * upload.m_endpoints.size() * sizeof(GpuConstraintRow));
		if (!break_states.empty())
		{
			if (m_r_break_states == nullptr)
				std::fill(break_states.begin(), break_states.end(), GpuConstraintBreakState{});
			else
				memcpy(break_states.data(), break_readback.ptr<GpuConstraintBreakState>(), upload.m_endpoints.size() * sizeof(GpuConstraintBreakState));
		}
	}

	// Custom deleter implementation keeps the incomplete type out of public Engine headers.
	void Deleter<GpuConstraintSolver>::operator()(GpuConstraintSolver* solver) const
	{
		delete solver;
	}
}
