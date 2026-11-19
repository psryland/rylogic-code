//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "pr/physics/integrator/engine_config.h"
#include "src/compute/constraint_solver_gpu.h"
#include "src/compute/gpu_capacity.h"
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
			int coupled_island_count;
			int pad0;
			int breakable_count;
			int break_substep_index;
			int schedule_begin;
			int schedule_count;
			uint32_t epoch_low;
			uint32_t epoch_high;
			int pad2;
		};
		static_assert(sizeof(cbConstraintSolve) == 80);

		// Register assignments shared by every persistent-constraint compute entry point.
		struct EReg
		{
			inline static constexpr auto Params = ECBufReg::b0;
			inline static constexpr auto Bodies = EUAVReg::u0;
			inline static constexpr auto Endpoints = ESRVReg::t0;
			inline static constexpr auto Descriptors = ESRVReg::t1;
			inline static constexpr auto Blocks = EUAVReg::u1;
			inline static constexpr auto Rows = EUAVReg::u2;
			inline static constexpr auto State = EUAVReg::u3;
			inline static constexpr auto PseudoVelocities = EUAVReg::u4;
			inline static constexpr auto BreakStates = EUAVReg::u5;
			inline static constexpr auto ColourDispatch = EUAVReg::u6;
			inline static constexpr auto SolveCache = EUAVReg::u7;
			inline static constexpr auto ColourSchedule = ESRVReg::t2;
			inline static constexpr auto ColourMasks = EUAVReg::u8;
			inline static constexpr auto RigidRecords = EUAVReg::u9;
			inline static constexpr auto TransactionResults = EUAVReg::u10;
			inline static constexpr auto Rejections = EUAVReg::u11;
			inline static constexpr auto CoupledAcceptance = EUAVReg::u12;
			inline static constexpr auto CoupledRows = EUAVReg::u13;
			inline static constexpr auto CoupledTopology = ESRVReg::t3;
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
		, m_cs_reset_colours()
		, m_cs_assign_colour_level()
		, m_cs_finish_colours()
		, m_cs_clear_pseudo_velocity()
		, m_cs_prepare_position_solve()
		, m_cs_solve_position()
		, m_cs_apply_position()
		, m_cs_prepare_velocity_solve()
		, m_cs_commit_velocity_transaction()
		, m_cs_detect_breakage()
		, m_cs_solve_position_fused()
		, m_r_endpoints()
		, m_r_descriptors()
		, m_r_blocks()
		, m_r_rows()
		, m_r_state()
		, m_r_pseudo_velocities()
		, m_r_break_states()
		, m_r_colour_dispatch()
		, m_r_solve_cache()
		, m_r_colour_schedule()
		, m_r_colour_masks()
		, m_r_rigid_records()
		, m_r_transaction_results()
		, m_transaction_epoch()
		, m_transaction_source()
		, m_transaction_capacity()
		, m_cmd_sig()
		, m_endpoint_shadow()
		, m_endpoint_identity_shadow()
		, m_descriptor_shadow()
		, m_invalidation_revisions()
		, m_body_degrees()
		, m_colour_schedule()
		, m_source()
		, m_capacity()
		, m_body_capacity()
		, m_break_capacity()
		, m_slot_count()
		, m_active_count()
		, m_breakable_count()
		, m_colour_count(MaxColours)
		, m_body_count()
		, m_dispatch_count()
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
			.UAV(EReg::State)
			.UAV(EReg::PseudoVelocities)
			.UAV(EReg::BreakStates)
			.UAV(EReg::ColourDispatch)
			.UAV(EReg::SolveCache)
			.SRV(EReg::ColourSchedule)
			.UAV(EReg::ColourMasks)
			.UAV(EReg::RigidRecords)
			.UAV(EReg::TransactionResults)
			.UAV(EReg::Rejections)
			.UAV(EReg::CoupledAcceptance)
			.UAV(EReg::CoupledRows)
			.SRV(EReg::CoupledTopology);
		auto const root_sig = sig.Create(m_gpu, "Physics:ConstraintSolverSig");

		auto compile_step = [&](ComputeStep& step, shader_code::ByteCode const& bytecode, char const* name)
		{
			auto const pso_name = FmtS("Physics:%sPSO", name);
			step.m_sig = root_sig;
			step.m_pso = ComputePSO(step.m_sig.get(), bytecode).Create(m_gpu, pso_name);
		};
		compile_step(m_cs_compile, shader_code::compile_constraints, "CompileConstraints");
		compile_step(m_cs_reset_colours, shader_code::reset_constraint_colours, "ResetConstraintColours");
		compile_step(m_cs_assign_colour_level, shader_code::assign_constraint_colour_level, "AssignConstraintColourLevel");
		compile_step(m_cs_finish_colours, shader_code::finish_constraint_colours, "FinishConstraintColours");
		compile_step(m_cs_clear_pseudo_velocity, shader_code::clear_constraint_pseudo_velocity, "ClearConstraintPseudoVelocity");
		compile_step(m_cs_prepare_position_solve, shader_code::prepare_constraint_position_solve, "PrepareConstraintPositionSolve");
		compile_step(m_cs_solve_position, shader_code::solve_constraint_position, "SolveConstraintPosition");
		compile_step(m_cs_apply_position, shader_code::apply_constraint_position, "ApplyConstraintPosition");
		compile_step(m_cs_prepare_velocity_solve, shader_code::prepare_constraint_velocity_solve, "PrepareConstraintVelocitySolve");
		compile_step(m_cs_commit_velocity_transaction, shader_code::commit_constraint_velocity_transaction, "CommitConstraintVelocityTransaction");
		compile_step(m_cs_detect_breakage, shader_code::detect_broken_constraints, "DetectBrokenConstraints");
		compile_step(m_cs_solve_position_fused, shader_code::solve_constraint_position_fused, "SolveConstraintPositionFused");

		// Colour sweeps read their per-colour group counts from the colouring pass output.
		D3D12_INDIRECT_ARGUMENT_DESC arg = {
			.Type = D3D12_INDIRECT_ARGUMENT_TYPE_DISPATCH
		};
		D3D12_COMMAND_SIGNATURE_DESC desc = {
			.ByteStride = sizeof(D3D12_DISPATCH_ARGUMENTS),
			.NumArgumentDescs = 1,
			.pArgumentDescs = &arg,
		};
		Check(m_gpu->CreateCommandSignature(&desc, nullptr, __uuidof(ID3D12CommandSignature), (void**)m_cmd_sig.address_of()));
	}

	// Create required upload buffers; Upload must finish initialization before publishing their capacity.
	bool GpuConstraintSolver::ResizeBuffers(GpuJob& job, int capacity)
	{
		// Dispatch arguments have fixed size, independent of retained slot capacity.
		if (m_r_colour_dispatch == nullptr)
			m_r_colour_dispatch = m_gpu.CreateResource(ResDesc::Buf<DispatchArguments>(MaxColours, {}).usage(EUsage::UnorderedAccess), job, "Physics:ConstraintColourDispatch");

		capacity = std::max(1, capacity);
		if (m_r_endpoints != nullptr && m_capacity >= capacity)
			return false;

		// A partial set is never reusable, even when retry requests fewer slots. Upload publishes capacity only after initialization.
		m_capacity = 0;
		for (auto const& resource : {m_r_endpoints, m_r_descriptors, m_r_blocks, m_r_rows, m_r_state, m_r_solve_cache, m_r_colour_schedule, m_r_colour_masks})
		{
			// Earlier commands may reference every displaced resource. KeepAlive allocation can fail before any member is replaced.
			if (resource != nullptr)
				job.m_keep_alive.Add(resource);
		}

		// Each returned resource immediately has an owner, including when a later allocation or upload fails.
		m_r_endpoints = m_gpu.CreateResource(ResDesc::Buf<GpuConstraintEndpoint>(capacity, {}), job, "Physics:ConstraintEndpoints");
		m_r_descriptors = m_gpu.CreateResource(ResDesc::Buf<GpuConstraintDesc>(capacity, {}), job, "Physics:ConstraintDescriptors");
		m_r_blocks = m_gpu.CreateResource(ResDesc::Buf<GpuConstraintBlock>(capacity, {}).usage(EUsage::UnorderedAccess), job, "Physics:ConstraintBlocks");
		m_r_rows = m_gpu.CreateResource(ResDesc::Buf<GpuConstraintRow>(GpuConstraintRowsPerBlock * capacity, {}).usage(EUsage::UnorderedAccess), job, "Physics:ConstraintRows");
		m_r_state = m_gpu.CreateResource(ResDesc::Buf<GpuConstraintSolverState>(1, {}).usage(EUsage::UnorderedAccess), job, "Physics:ConstraintSolverState");
		m_r_solve_cache = m_gpu.CreateResource(ResDesc::Buf<GpuConstraintSolveCache>(capacity, {}).usage(EUsage::UnorderedAccess), job, "Physics:ConstraintSolveCache");
		m_r_colour_schedule = m_gpu.CreateResource(ResDesc::Buf<GpuConstraintColourTask>(capacity, {}), job, "Physics:ConstraintColourSchedule");
		m_r_colour_masks = m_gpu.CreateResource(ResDesc::Buf<uint32_t>(2 * capacity, {}).usage(EUsage::UnorderedAccess), job, "Physics:ConstraintColourMasks");
		return true;
	}

	// Build the colouring schedule for 'endpoints' and upload it. The schedule buffer has one entry per stable slot.
	void GpuConstraintSolver::UploadColourSchedule(GpuJob& job, std::span<GpuConstraintEndpoint const> endpoints)
	{
		// The schedule depends only on the frame-local endpoint stream, so it is rebuilt with each upload.
		m_colour_schedule.Build(endpoints);
		auto const task_count = isize(m_colour_schedule.m_tasks);
		if (task_count == 0)
			return;

		// Copy the tasks; the level offsets stay on the CPU and become per-dispatch constants.
		job.m_barriers.Transition(m_r_colour_schedule.get(), D3D12_RESOURCE_STATE_COPY_DEST).Commit();
		auto schedule_upload = job.m_upload.Alloc<GpuConstraintColourTask>(task_count);
		memcpy(schedule_upload.ptr<GpuConstraintColourTask>(), m_colour_schedule.m_tasks.data(), task_count * sizeof(GpuConstraintColourTask));
		job.m_cmd_list.CopyBufferRegion(m_r_colour_schedule.get(), 0, schedule_upload);
		job.m_barriers.Transition(m_r_colour_schedule.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE).Commit();
	}

	// Create or grow shared rigid pseudo-twist storage for either independent or coupled position correction.
	void GpuConstraintSolver::EnsurePseudoVelocityStorage(GpuJob& job, int body_count)
	{
		if (body_count < 0)
			throw std::invalid_argument("Constraint pseudo-velocity body count cannot be negative");

		m_body_count = body_count;
		if (m_r_pseudo_velocities != nullptr && m_body_capacity >= body_count)
			return;

		// Keep the old handle/capacity coherent until replacement succeeds; a recorded prefix may still clear or use the old buffer.
		auto const capacity = GrowCapacity(body_count, m_body_capacity);
		if (m_r_pseudo_velocities != nullptr)
			job.m_keep_alive.Add(m_r_pseudo_velocities);

		// Failure leaves retry observing the old capacity, never a promise for an allocation that did not succeed.
		m_r_pseudo_velocities = m_gpu.CreateResource(ResDesc::Buf<GpuConstraintPseudoVelocity>(capacity, {}).usage(EUsage::UnorderedAccess), job, "Physics:ConstraintPseudoVelocities");
		m_body_capacity = capacity;
	}

	// Return adequately sized rigid pseudo storage for a shared position solve.
	D3DPtr<ID3D12Resource> GpuConstraintSolver::PseudoVelocityStorage(GpuJob& job, int body_count)
	{
		EnsurePseudoVelocityStorage(job, body_count);
		return m_r_pseudo_velocities;
	}

	// Create or geometrically grow the optional per-slot overload latch.
	void GpuConstraintSolver::EnsureBreakStateStorage(GpuJob& job, int slot_count)
	{
		if (slot_count < 0)
			throw std::invalid_argument("Constraint break-state slot count cannot be negative");
		if (m_r_break_states != nullptr && m_break_capacity >= slot_count)
			return;

		// Break storage follows the same single-resource replacement invariant as shared pseudo state.
		auto const capacity = GrowCapacity(slot_count, m_break_capacity);
		if (m_r_break_states != nullptr)
			job.m_keep_alive.Add(m_r_break_states);

		// Upload initializes every used latch on each visit, including a retry after interrupted initialization.
		m_r_break_states = m_gpu.CreateResource(ResDesc::Buf<GpuConstraintBreakState>(capacity, {}).usage(EUsage::UnorderedAccess), job, "Physics:ConstraintBreakStates");
		m_break_capacity = capacity;
	}

	// Upload shared frame-local endpoints and changed persistent descriptors, returning whether independent rigid work is active.
	bool GpuConstraintSolver::Upload(GpuJob& job, GpuConstraintUpload const& upload)
	{
		m_slot_count = static_cast<int>(upload.m_endpoints.size());
		m_active_count = static_cast<int>(upload.m_rigid_active_count);
		m_breakable_count = static_cast<int>(upload.m_breakable_count);
		m_body_count = 0;
		m_dispatch_count = 0;
		m_rejection_count = 0;
		if (m_slot_count == 0)
		{
			// An earlier recorded preparation may still reference the optional latch before this empty upload.
			if (m_r_break_states != nullptr)
				job.m_keep_alive.Add(m_r_break_states);

			m_r_break_states = nullptr;
			m_break_capacity = 0;
			m_breakable_count = 0;
			m_source = upload.m_source;
			m_transaction_source = nullptr;
			return false;
		}
		if (upload.m_descriptors.size() != upload.m_endpoints.size())
			throw std::invalid_argument("GPU constraint endpoint and descriptor slot counts must match");
		if (upload.m_endpoint_identities.size() != upload.m_endpoints.size())
			throw std::invalid_argument("GPU constraint endpoint identities and endpoint slot counts must match");
		if (m_breakable_count < 0 || m_breakable_count > m_slot_count)
			throw std::invalid_argument("GPU breakable constraint count is outside the stable-slot range");

		// Stable slots and frame-local coupled islands have disjoint addresses in one frame report.
		auto const report_count = upload.m_endpoints.size() + upload.m_coupled_islands.size();
		if (report_count > static_cast<size_t>(std::numeric_limits<int>::max()))
			throw std::overflow_error("Constraint rejection slot count exceeds the GPU index domain");

		if (m_rejection_capacity < static_cast<int>(report_count))
		{
			// Retain any recorded prefix before replacing storage.
			auto const capacity = GrowCapacity(static_cast<int>(report_count), m_rejection_capacity);
			if (m_r_frame_rejections != nullptr)
				job.m_keep_alive.Add(m_r_frame_rejections);

			m_r_frame_rejections = m_gpu.CreateResource(ResDesc::Buf<GpuConstraintRejectionState>(capacity, {}).usage(EUsage::UnorderedAccess), job, "Physics:ConstraintFrameRejections");
			m_rejection_capacity = capacity;
		}
		auto reports = job.m_upload.Alloc<GpuConstraintRejectionState>(static_cast<int>(report_count));
		std::fill_n(reports.ptr<GpuConstraintRejectionState>(), report_count, GpuConstraintRejectionState{.substep_index = -1});
		job.m_barriers.Transition(m_r_frame_rejections.get(), D3D12_RESOURCE_STATE_COPY_DEST).Commit();
		job.m_cmd_list.CopyBufferRegion(m_r_frame_rejections.get(), 0, reports);
		job.m_barriers.Transition(m_r_frame_rejections.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS).Commit();
		m_rejection_count = static_cast<int>(report_count);

		// Input clocks retain their high-water length even when another source has fewer slots.
		m_invalidation_revisions.resize(std::max(m_invalidation_revisions.size(), upload.m_endpoints.size()));
		auto const resized = ResizeBuffers(job, m_slot_count);
		m_colour_count = ColourBound(upload.m_endpoints);
		UploadColourSchedule(job, upload.m_endpoints);
		auto endpoints = upload.m_endpoints;
		auto descriptor_dirty = std::vector<bool>(m_slot_count, resized || m_descriptor_shadow.size() != upload.m_descriptors.size());
		auto const source_changed = m_source != upload.m_source;
		auto const reset_all = resized || source_changed;
		auto reset_runtime = std::vector<bool>(m_slot_count, reset_all);

		// Mark only semantically changed stable slots for warm-start invalidation; packed-body reordering is harmless.
		for (int index = 0; index != m_slot_count; ++index)
		{
			auto const previously_enabled =
				!source_changed &&
				index < isize(m_endpoint_shadow) &&
				AllSet(m_endpoint_shadow[index].flags, GpuConstraintEndpointFlags_Enabled);
			auto const currently_enabled = AllSet(endpoints[index].flags, GpuConstraintEndpointFlags_Enabled);
			auto const descriptor_changed =
				source_changed ||
				index >= isize(m_descriptor_shadow) ||
				std::memcmp(&m_descriptor_shadow[index], &upload.m_descriptors[index], sizeof(GpuConstraintDesc)) != 0;
			auto const endpoint_identity_changed =
				index >= isize(m_endpoint_identity_shadow) ||
				m_endpoint_identity_shadow[index] != upload.m_endpoint_identities[index];
			descriptor_dirty[index] = descriptor_dirty[index] || descriptor_changed;
			auto const endpoint_changed =
				source_changed ||
				index >= isize(m_endpoint_shadow) ||
				EndpointChanged(m_endpoint_shadow[index], endpoints[index]) ||
				endpoint_identity_changed;
			auto const invalidate = descriptor_changed || endpoint_changed || AllSet(endpoints[index].flags, GpuConstraintEndpointFlags_ResetWarmStart);
			if (resized || invalidate)
				endpoints[index].flags |= GpuConstraintEndpointFlags_ResetWarmStart;

			// Storage reset is not an input event. Semantic clocks survive both legacy and accepted-record allocation boundaries.
			auto& revision = m_invalidation_revisions[index];
			if (invalidate)
			{
				// Never alias a still-live receipt by wrapping its fixed-width representation.
				if (revision == GpuConstraintMaxInvalidationRevision)
					throw std::overflow_error("Constraint input invalidation revision exhausted");

				++revision;
			}
			endpoints[index].invalidation_revision = revision;

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
		job.m_barriers.Transition(m_r_state.get(), D3D12_RESOURCE_STATE_COPY_DEST);
		if (any_descriptor_dirty)
			job.m_barriers.Transition(m_r_descriptors.get(), D3D12_RESOURCE_STATE_COPY_DEST);
		job.m_barriers.Commit();
		{
			auto endpoint_upload = job.m_upload.Alloc<GpuConstraintEndpoint>(m_slot_count);
			memcpy(endpoint_upload.ptr<GpuConstraintEndpoint>(), endpoints.data(), endpoints.size() * sizeof(GpuConstraintEndpoint));
			job.m_cmd_list.CopyBufferRegion(m_r_endpoints.get(), 0, endpoint_upload);
		}

		// Reset both frame-local words once; later preparation and colouring must preserve all previously reported failures.
		auto state_upload = job.m_upload.Alloc<GpuConstraintSolverState>(1);
		state_upload.ptr<GpuConstraintSolverState>()[0] = GpuConstraintSolverState{};
		job.m_cmd_list.CopyBufferRegion(m_r_state.get(), 0, state_upload);

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
			auto descriptor_upload = job.m_upload.Alloc<GpuConstraintDesc>(count);
			memcpy(descriptor_upload.ptr<GpuConstraintDesc>(), upload.m_descriptors.data() + begin, count * sizeof(GpuConstraintDesc));
			job.m_cmd_list.CopyBufferRegion(m_r_descriptors.get(), static_cast<uint64_t>(begin) * sizeof(GpuConstraintDesc), descriptor_upload);
			begin = end;
		}
		job.m_barriers.Transition(m_r_endpoints.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(m_r_state.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		if (any_descriptor_dirty)
			job.m_barriers.Transition(m_r_descriptors.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Commit();

		// Break latches are frame-local and exist only when at least one active descriptor has a finite threshold.
		if (m_breakable_count != 0)
		{
			EnsureBreakStateStorage(job, m_slot_count);
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
			// Removing optional storage must also preserve any outstanding prefix that bound it.
			if (m_r_break_states != nullptr)
				job.m_keep_alive.Add(m_r_break_states);

			m_r_break_states = nullptr;
			m_break_capacity = 0;
		}

		m_endpoint_shadow = endpoints;
		m_endpoint_identity_shadow = upload.m_endpoint_identities;
		m_descriptor_shadow = upload.m_descriptors;
		m_source = upload.m_source;
		if (resized)
			m_capacity = std::max(1, m_slot_count);

		return m_active_count != 0;
	}

	// Bind the common constraint root signature and resources for one compute step.
	void GpuConstraintSolver::Bind(GpuJob& job, ComputeStep& step, float timestep, int body_count, int colour, int position_iterations, D3DPtr<ID3D12Resource> bodies, ID3D12Resource* colour_dispatch)
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
			.warm_start_factor = m_transaction_epoch.warm_start_factor,
			.coupled_island_count = 0,
			.pad0 = 0,
			.breakable_count = m_breakable_count,
			.break_substep_index = m_break_substep_index,
			.schedule_begin = 0,
			.schedule_count = isize(m_colour_schedule.m_tasks),
			.epoch_low = m_transaction_epoch.identity.x,
			.epoch_high = m_transaction_epoch.identity.y,
			.pad2 = 0,
		};
		job.m_cmd_list.SetPipelineState(step.m_pso.get());
		job.m_cmd_list.SetComputeRootSignature(step.m_sig.get());
		job.m_cmd_list.AddComputeRoot32BitConstants(constants);
		job.m_cmd_list.AddComputeRootUnorderedAccessView(bodies);
		job.m_cmd_list.AddComputeRootShaderResourceView(m_r_endpoints);
		job.m_cmd_list.AddComputeRootShaderResourceView(m_r_descriptors);
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_blocks);
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_rows);
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_state);
		auto* pseudo_velocities = m_r_pseudo_velocities != nullptr ? m_r_pseudo_velocities.get() : m_r_state.get();
		job.m_cmd_list.AddComputeRootUnorderedAccessView(pseudo_velocities);
		auto* break_states = m_r_break_states != nullptr ? m_r_break_states.get() : m_r_state.get();
		job.m_cmd_list.AddComputeRootUnorderedAccessView(break_states);
		job.m_cmd_list.AddComputeRootUnorderedAccessView(colour_dispatch);
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_solve_cache);
		job.m_cmd_list.AddComputeRootShaderResourceView(m_r_colour_schedule);
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_colour_masks);
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_rigid_records != nullptr ? m_r_rigid_records.get() : m_r_state.get());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_transaction_results != nullptr ? m_r_transaction_results.get() : m_r_state.get());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_frame_rejections);
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_state);
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_state);
		job.m_cmd_list.AddComputeRootShaderResourceView(m_r_endpoints);
	}

	// Insert UAV ordering between dependent compiler, colouring, and solver passes.
	void GpuConstraintSolver::CommitUavBarriers(GpuJob& job, D3DPtr<ID3D12Resource> bodies)
	{
		job.m_barriers.UAV(bodies.get());
		job.m_barriers.UAV(m_r_blocks.get());
		job.m_barriers.UAV(m_r_rows.get());
		job.m_barriers.UAV(m_r_state.get());
		job.m_barriers.UAV(m_r_pseudo_velocities.get());
		job.m_barriers.UAV(m_r_solve_cache.get());
		job.m_barriers.Commit();
	}

	// Return an upper bound on the number of colours that greedy colouring can use for 'endpoints'.
	int GpuConstraintSolver::ColourBound(std::span<GpuConstraintEndpoint const> endpoints)
	{
		// Count the slots that touch each body. Coupled slots are not coloured, and a world endpoint (a negative index) never limits a colour. Every
		// other body is counted as dynamic, which can only raise the bound.
		m_body_degrees.clear();
		auto count = [&](int body_idx)
		{
			// Grow the table to cover the body, then count the slot
			if (body_idx < 0)
				return;
			if (body_idx >= isize(m_body_degrees))
				m_body_degrees.resize(body_idx + 1, 0);

			++m_body_degrees[body_idx];
		};
		for (auto const& endpoint : endpoints)
		{
			// Coupled slots belong to the coupled solver
			if (AllSet(endpoint.flags, GpuConstraintEndpointFlags_Coupled))
				continue;

			count(endpoint.body_idx_a);
			count(endpoint.body_idx_b);
		}

		// Greedy colouring gives a slot the lowest colour that neither body has used yet. Before the slot is coloured, each body has used at most one
		// colour for each of its other slots, so the slot's colour index is at most the sum of those counts.
		auto bound = 1;
		auto degree = [&](int body_idx)
		{
			// A world endpoint has no other slots that could block a colour
			return body_idx >= 0 ? m_body_degrees[body_idx] - 1 : 0;
		};
		for (auto const& endpoint : endpoints)
		{
			// Coupled slots belong to the coupled solver
			if (AllSet(endpoint.flags, GpuConstraintEndpointFlags_Coupled))
				continue;

			bound = std::max(bound, degree(endpoint.body_idx_a) + degree(endpoint.body_idx_b) + 1);
		}
		return std::min(bound, int(MaxColours));
	}

	// Record one colour sweep in the established colour order. Small slot sets use the single-group fused step; larger sets use one indirect pass for
	// each colour that the colour bound allows.
	void GpuConstraintSolver::RecordColourSweep(GpuJob& job, ComputeStep& per_colour_step, ComputeStep& fused_step, float timestep, int body_count, int position_iterations, D3DPtr<ID3D12Resource> bodies)
	{
		// The slot count is exact on the CPU, so the form is chosen without a GPU readback. Both forms visit the colours in the same order.
		auto const fused = m_config.fused_colour_sweep_limit > 0 && m_slot_count <= m_config.fused_colour_sweep_limit;
		auto& step = fused ? fused_step : per_colour_step;

		// Bind once and flush pending transitions. Only the colour root constant changes between colours.
		Bind(job, step, timestep, body_count, 0, position_iterations, bodies, m_r_state.get());
		job.m_barriers.Commit();

		// Every colour has the same dependency set, so the UAV barriers are prepared once.
		auto const uav_barrier = [](ID3D12Resource* resource)
		{
			// UAV ordering does not need the state tracking done by BarrierBatch.
			return D3D12_RESOURCE_BARRIER{
				.Type = D3D12_RESOURCE_BARRIER_TYPE_UAV,
				.Flags = D3D12_RESOURCE_BARRIER_FLAG_NONE,
				.UAV = {.pResource = resource},
			};
		};
		auto const barriers = std::array{
			uav_barrier(bodies.get()),
			uav_barrier(m_r_blocks.get()),
			uav_barrier(m_r_rows.get()),
			uav_barrier(m_r_state.get()),
			uav_barrier(m_r_pseudo_velocities.get()),
			uav_barrier(m_r_solve_cache.get()),
		};

		// The fused kernel walks every occupied colour inside one group; the trailing barrier orders the whole sweep before later work.
		auto* cmd_list = job.m_cmd_list.get();
		if (fused)
		{
			cmd_list->Dispatch(1, 1, 1);
			cmd_list->ResourceBarrier(static_cast<UINT>(barriers.size()), barriers.data());
			++m_dispatch_count;
			return;
		}

		// Empty colours have zero groups. Occupied colours keep the full stable-slot grid, so the solve order and results are unchanged. Colours past
		// the bound are always empty, so they are not recorded. Serial fallback after colour exhaustion runs in colour zero, which is always recorded.
		job.m_cmd_list.Use(m_r_colour_dispatch.get());
		for (int colour = 0; colour != m_colour_count; ++colour)
		{
			cmd_list->SetComputeRoot32BitConstants(0, 1, &colour, offsetof(cbConstraintSolve, colour) / sizeof(uint32_t));
			cmd_list->ExecuteIndirect(m_cmd_sig.get(), 1, m_r_colour_dispatch.get(), colour * sizeof(DispatchArguments), nullptr, 0);
			cmd_list->ResourceBarrier(static_cast<UINT>(barriers.size()), barriers.data());
			++m_dispatch_count;
		}
	}

	// Record the colouring passes, which also write the per-colour indirect arguments that every later sweep in this frame reads.
	void GpuConstraintSolver::RecordColouring(GpuJob& job, float timestep, int body_count, D3DPtr<ID3D12Resource> bodies)
	{
		// Clear body masks, slot colours, and the overflow words. One thread covers one body and one slot.
		auto const groups = [](int count)
		{
			return static_cast<UINT>((std::max(1, count) + ConstraintThreadCount - 1) / ConstraintThreadCount);
		};
		auto const iterations = std::max(1, m_config.push_out_iterations);
		Bind(job, m_cs_reset_colours, timestep, body_count, 0, iterations, bodies, m_r_colour_dispatch.get());
		job.m_cmd_list.Dispatch(groups(std::max(body_count, m_slot_count)), 1, 1);
		++m_dispatch_count;
		CommitUavBarriers(job, bodies);

		// Colour the levels in order. Each level reads the masks written by earlier levels, so only those buffers need ordering between levels.
		auto const uav_barrier = [](ID3D12Resource* resource)
		{
			// UAV ordering does not need the state tracking done by BarrierBatch.
			return D3D12_RESOURCE_BARRIER{
				.Type = D3D12_RESOURCE_BARRIER_TYPE_UAV,
				.Flags = D3D12_RESOURCE_BARRIER_FLAG_NONE,
				.UAV = {.pResource = resource},
			};
		};
		auto const barriers = std::array{
			uav_barrier(m_r_blocks.get()),
			uav_barrier(m_r_state.get()),
			uav_barrier(m_r_colour_masks.get()),
		};
		Bind(job, m_cs_assign_colour_level, timestep, body_count, 0, iterations, bodies, m_r_colour_dispatch.get());
		job.m_barriers.Commit();
		auto* cmd_list = job.m_cmd_list.get();
		auto const& offsets = m_colour_schedule.m_level_offsets;
		for (int level = 0; level != m_colour_schedule.LevelCount(); ++level)
		{
			// Only the level range changes between dispatches.
			auto const range = std::array<int, 2>{ offsets[level], offsets[level + 1] - offsets[level] };
			cmd_list->SetComputeRoot32BitConstants(0, 2, range.data(), offsetof(cbConstraintSolve, schedule_begin) / sizeof(uint32_t));
			cmd_list->Dispatch(groups(range[1]), 1, 1);
			cmd_list->ResourceBarrier(static_cast<UINT>(barriers.size()), barriers.data());
			++m_dispatch_count;
		}

		// Publish the body masks and the indirect arguments. The arguments then become read-only for the rest of the frame.
		Bind(job, m_cs_finish_colours, timestep, body_count, 0, iterations, bodies, m_r_colour_dispatch.get());
		job.m_barriers.Commit();
		job.m_cmd_list.Dispatch(groups(isize(m_colour_schedule.m_tasks)), 1, 1);
		++m_dispatch_count;
		job.m_barriers.Transition(m_r_colour_dispatch.get(), D3D12_RESOURCE_STATE_INDIRECT_ARGUMENT);
		CommitUavBarriers(job, bodies);
	}

	// Compile current geometry without consuming invalidation or changing accepted physical ownership.
	void GpuConstraintSolver::CompileRows(GpuJob& job, float timestep, int body_count, D3DPtr<ID3D12Resource> bodies)
	{
		// Only the ordinary compiler writes independent rigid rows.
		if (!Active())
			return;

		pix::BeginEvent(job.m_cmd_list.get(), 0xFF8A5FD3, "Physics::CompileConstraints");
		m_body_count = body_count;
		job.m_barriers.Transition(bodies.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_endpoints.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(m_r_descriptors.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(m_r_blocks.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_rows.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_state.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_colour_dispatch.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_solve_cache.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_colour_masks.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_colour_schedule.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Commit();

		auto const group_count = static_cast<UINT>((m_slot_count + ConstraintThreadCount - 1) / ConstraintThreadCount);
		Bind(job, m_cs_compile, timestep, body_count, 0, std::max(1, m_config.push_out_iterations), bodies, m_r_state.get());
		job.m_cmd_list.Dispatch(group_count, 1, 1);
		++m_dispatch_count;
		CommitUavBarriers(job, bodies);

		pix::EndEvent(job.m_cmd_list.get());
	}

	// Initialize ordinary-owned position scratch before the first writer; shared owners initialize their own view instead.
	void GpuConstraintSolver::ClearPositionState(GpuJob& job, float timestep, int body_count, D3DPtr<ID3D12Resource> bodies)
	{
		// Empty ordinary work has no independent pseudo stream to initialize.
		if (!Active() || body_count == 0)
			return;

		EnsurePseudoVelocityStorage(job, body_count);
		job.m_barriers.Transition(m_r_pseudo_velocities.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS).Commit();
		Bind(job, m_cs_clear_pseudo_velocity, timestep, body_count, 0, std::max(1, m_config.push_out_iterations), bodies, m_r_state.get());
		job.m_cmd_list.Dispatch(static_cast<UINT>((body_count + ConstraintThreadCount - 1) / ConstraintThreadCount), 1, 1);
		++m_dispatch_count;
		CommitUavBarriers(job, bodies);
	}

	// Build each block's split-position response once, because transforms and rows are fixed during the position iterations.
	void GpuConstraintSolver::PreparePositionSolve(GpuJob& job, float timestep, int body_count, int position_iterations, D3DPtr<ID3D12Resource> bodies)
	{
		// Skip the build when no position sweep will read it.
		if (!Active() || position_iterations <= 0)
			return;

		// Colouring belongs to position sweeps; physical transactions use the uploaded dependency levels directly.
		RecordColouring(job, timestep, body_count, bodies);
		Bind(job, m_cs_prepare_position_solve, timestep, body_count, 0, position_iterations, bodies, m_r_state.get());
		job.m_barriers.Commit();
		job.m_cmd_list.Dispatch(static_cast<UINT>((m_slot_count + ConstraintThreadCount - 1) / ConstraintThreadCount), 1, 1);
		++m_dispatch_count;
		CommitUavBarriers(job, bodies);
	}

	// Execute one coloured split-position sweep against initialized shared pseudo velocity without clearing accumulated corrections.
	void GpuConstraintSolver::SolvePositionIteration(GpuJob& job, float timestep, int body_count, int position_iterations, D3DPtr<ID3D12Resource> bodies)
	{
		if (!Active() || position_iterations <= 0)
			return;

		RecordColourSweep(job, m_cs_solve_position, m_cs_solve_position_fused, timestep, body_count, position_iterations, bodies);
	}

	// Integrate converged split pseudo twists once while preserving physical momentum.
	void GpuConstraintSolver::ApplyPosition(GpuJob& job, float timestep, int body_count, int position_iterations, D3DPtr<ID3D12Resource> bodies)
	{
		if (!Active() || position_iterations <= 0 || body_count == 0)
			return;

		Bind(job, m_cs_apply_position, timestep, body_count, 0, position_iterations, bodies, m_r_state.get());
		job.m_cmd_list.Dispatch(static_cast<UINT>((body_count + ConstraintThreadCount - 1) / ConstraintThreadCount), 1, 1);
		++m_dispatch_count;
		CommitUavBarriers(job, bodies);
	}

	// Build each block's physical response once, because transforms and rows are fixed during the velocity iterations.
	void GpuConstraintSolver::PrepareVelocitySolve(GpuJob& job, float timestep, int body_count, D3DPtr<ID3D12Resource> bodies)
	{
		// The cache is only read by the velocity sweeps of an active lane.
		if (!Active())
			return;

		Bind(job, m_cs_prepare_velocity_solve, timestep, body_count, 0, std::max(1, m_config.push_out_iterations), bodies, m_r_state.get());
		job.m_barriers.Commit();
		job.m_cmd_list.Dispatch(static_cast<UINT>((m_slot_count + ConstraintThreadCount - 1) / ConstraintThreadCount), 1, 1);
		++m_dispatch_count;
		CommitUavBarriers(job, bodies);
	}

	// Prepare stable-membership rigid transactions without physically applying any provisional coefficient.
	void GpuConstraintSolver::PrepareVelocityTransactions(GpuJob& job, GpuConstraintEpoch epoch)
	{
		// Context failures are programmer/input errors, never non-terminal physical admission outcomes.
		if (!(epoch.timestep > 0.0f) || !std::isfinite(epoch.timestep) || !std::isfinite(epoch.warm_start_factor) || epoch.warm_start_factor < 0.0f)
			throw std::invalid_argument("Constraint transactions require a finite positive timestep and finite nonnegative warm factor");

		auto const continuing = m_transaction_capacity != 0 && All(m_transaction_epoch.identity == epoch.identity);
		if (continuing && (m_transaction_source != m_source || m_slot_count > m_transaction_capacity))
			throw std::logic_error("Constraint transaction context changed within a stable-membership epoch");

		// Lowered ordinary endpoints preserve ConstraintSet's no-self-connection domain; distinct blocks can still share bodies.
		for (auto const& endpoint : m_endpoint_shadow)
		{
			// Disabled and coupled slots do not enter this ordinary transaction owner.
			if (AllSet(endpoint.flags, GpuConstraintEndpointFlags_Enabled) && !AllSet(endpoint.flags, GpuConstraintEndpointFlags_Coupled) && endpoint.body_idx_a == endpoint.body_idx_b)
				throw std::invalid_argument("Ordinary constraint transactions require distinct endpoints");
		}

		// These records belong only to ordinary stable slots. Coupled-island target responses have a different owner and lifetime.
		auto const grow = m_transaction_capacity == 0 || m_transaction_capacity < m_slot_count;
		auto const reset = grow || m_transaction_source != m_source;
		auto const capacity = grow ? GrowCapacity(std::max(1, m_slot_count), m_transaction_capacity) : m_transaction_capacity;
		if (reset)
		{
			// Mark the set incomplete before any replacement allocation, including KeepAlive's own storage, can throw.
			m_transaction_capacity = 0;
			m_transaction_source = nullptr;

			// Earlier recorded work may still use the displaced buffers. Retain them before releasing either owner reference.
			if (m_r_rigid_records != nullptr)
				job.m_keep_alive.Add(m_r_rigid_records);

			if (m_r_transaction_results != nullptr)
				job.m_keep_alive.Add(m_r_transaction_results);

			// Returned resources stay owned here during the caller's recording retirement, even if the second allocation fails.
			// Any allocation/initialization failure leaves retry obliged to recreate both buffers.
			m_r_rigid_records = m_gpu.CreateResource(ResDesc::Buf<GpuConstraintRigidRecord>(capacity, {}).usage(EUsage::UnorderedAccess), job, "Physics:ConstraintRigidRecords");
			m_r_transaction_results = m_gpu.CreateResource(ResDesc::Buf<uint32_t>(capacity, {}).usage(EUsage::UnorderedAccess), job, "Physics:ConstraintTransactionResults");

			// A source/capacity replacement at a fresh epoch cannot inherit another owner's accepted coefficients.
			auto data = job.m_upload.Alloc<GpuConstraintRigidRecord>(capacity);
			std::fill_n(data.ptr<GpuConstraintRigidRecord>(), capacity, GpuConstraintRigidRecord{});
			job.m_barriers.Transition(m_r_rigid_records.get(), D3D12_RESOURCE_STATE_COPY_DEST).Commit();
			job.m_cmd_list.CopyBufferRegion(m_r_rigid_records.get(), 0, data);
		}

		// Zero iterations leave every result unattempted and retain the prior acceptance unchanged.
		auto results = job.m_upload.Alloc<uint32_t>(std::max(1, m_slot_count));
		std::fill_n(results.ptr<uint32_t>(), std::max(1, m_slot_count), 0xFFFFFFFFu);
		job.m_barriers.Transition(m_r_transaction_results.get(), D3D12_RESOURCE_STATE_COPY_DEST).Commit();
		job.m_cmd_list.CopyBufferRegion(m_r_transaction_results.get(), 0, results);
		job.m_barriers.Transition(m_r_rigid_records.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_transaction_results.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Commit();

		// Publish a usable set only after both resources and their initialization have been recorded successfully.
		m_transaction_epoch = epoch;
		m_transaction_source = m_source;
		m_transaction_capacity = capacity;
	}

	// Publish complete independent blocks in the same stable dependency order used to construct the ordinary colouring schedule.
	void GpuConstraintSolver::CommitVelocityTransactions(GpuJob& job, int body_count, D3DPtr<ID3D12Resource> bodies, int substep_index)
	{
		// The caller must begin this owner's transaction context before recording a solve.
		if (m_transaction_capacity < std::max(1, m_slot_count) || m_transaction_source != m_source)
			throw std::logic_error("Constraint transaction publication requires matching prepared owner storage");

		// Dependency levels include enabled slots with zero current rows, so null-response cache retirement is never skipped.
		m_break_substep_index = substep_index;
		job.m_barriers.Transition(m_r_frame_rejections.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		Bind(job, m_cs_commit_velocity_transaction, m_transaction_epoch.timestep, body_count, 0, 0, bodies, m_r_state.get());
		job.m_barriers.Commit();
		auto const& offsets = m_colour_schedule.m_level_offsets;
		for (int level = 0; level != m_colour_schedule.LevelCount(); ++level)
		{
			// Different blocks in one level share no body. A UAV barrier completes publication before any dependent block runs.
			auto const range = std::array{offsets[level], offsets[level + 1] - offsets[level]};
			job.m_cmd_list.get()->SetComputeRoot32BitConstants(0, 2, range.data(), offsetof(cbConstraintSolve, schedule_begin) / sizeof(uint32_t));
			job.m_cmd_list.Dispatch(static_cast<UINT>((range[1] + ConstraintThreadCount - 1) / ConstraintThreadCount), 1, 1);
			++m_dispatch_count;
			job.m_barriers.UAV(m_r_rigid_records.get());
			job.m_barriers.UAV(m_r_transaction_results.get());
			job.m_barriers.UAV(m_r_frame_rejections.get());
			CommitUavBarriers(job, bodies);
		}
	}

	// Return the prepared owner's accepted records and latest attempt results without recording work.
	GpuConstraintTransactionOutput GpuConstraintSolver::TransactionOutput()
	{
		// Partially allocated storage is not a readable owner result.
		if (m_transaction_capacity < std::max(1, m_slot_count) || m_transaction_source != m_source)
			throw std::logic_error("Constraint transaction output requires complete owner storage");

		// The owning solver retains these resources; consumers must finish before its next storage replacement.
		return {m_r_rigid_records.get(), m_r_transaction_results.get(), m_slot_count};
	}

	// Direct diagnostics call the same owner preparation and publication phases that the later Engine integration will dispatch.
	void GpuConstraintSolver::SolveTransactions(GpuJob& job, GpuConstraintEpoch epoch, GpuConstraintUpload const& upload, std::span<GpuRigidBody> bodies, std::span<GpuConstraintRigidRecord> records, std::span<uint32_t> results)
	{
		// Every owner result is mandatory: a rejected transaction must not be presented as an ordinary successful solve.
		if (upload.m_endpoints.empty() || records.size() != upload.m_endpoints.size() || results.size() != upload.m_endpoints.size())
			throw std::invalid_argument("Constraint transaction diagnostics require a nonempty slot stream and matching result/record spans");

		// Resource and readback owners outlive an interrupted recording; normal submission is outside the recording-only catch.
		auto r_bodies = D3DPtr<ID3D12Resource>{};
		auto body_readback = ReadbackAlloc{};
		auto record_readback = ReadbackAlloc{};
		auto result_readback = ReadbackAlloc{};
		auto state_readback = ReadbackAlloc{};
		try
		{
			// Upload the caller's actual current physical state, including any intervening writer's changes.
			auto const body_count = isize(bodies);
			r_bodies = m_gpu.CreateResource(ResDesc::Buf<GpuRigidBody>(std::max(1, body_count), {}).usage(EUsage::UnorderedAccess), job, "Physics:ConstraintTransactionTestBodies");
			auto data = job.m_upload.Alloc<GpuRigidBody>(std::max(1, body_count));
			std::fill_n(data.ptr<GpuRigidBody>(), std::max(1, body_count), GpuRigidBody{});
			std::copy(bodies.begin(), bodies.end(), data.ptr<GpuRigidBody>());
			job.m_barriers.Transition(r_bodies.get(), D3D12_RESOURCE_STATE_COPY_DEST).Commit();
			job.m_cmd_list.CopyBufferRegion(r_bodies.get(), 0, data);
			job.m_barriers.Transition(r_bodies.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS).Commit();
			Upload(job, upload);
			PrepareVelocityTransactions(job, epoch);
			CompileRows(job, epoch.timestep, body_count, r_bodies);
			PrepareVelocitySolve(job, epoch.timestep, body_count, r_bodies);
			for (int iteration = 0; iteration != std::max(0, m_config.solver_iterations); ++iteration)
			{
				// No separately published warm operation belongs in this sequence.
				CommitVelocityTransactions(job, body_count, r_bodies);
			}

			// Capture physical state and the authoritative GPU record after the actual publication dispatches.
			job.m_barriers.Transition(r_bodies.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
			job.m_barriers.Transition(m_r_rigid_records.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
			job.m_barriers.Transition(m_r_transaction_results.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
			job.m_barriers.Transition(m_r_state.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
			job.m_barriers.Commit();
			body_readback = job.m_readback.Alloc<GpuRigidBody>(std::max(1, body_count));
			record_readback = job.m_readback.Alloc<GpuConstraintRigidRecord>(m_slot_count);
			result_readback = job.m_readback.Alloc<uint32_t>(m_slot_count);
			state_readback = job.m_readback.Alloc<GpuConstraintSolverState>(1);
			job.m_cmd_list.CopyBufferRegion(body_readback, r_bodies.get(), 0);
			job.m_cmd_list.CopyBufferRegion(record_readback, m_r_rigid_records.get(), 0);
			job.m_cmd_list.CopyBufferRegion(result_readback, m_r_transaction_results.get(), 0);
			job.m_cmd_list.CopyBufferRegion(state_readback, m_r_state.get(), 0);
		}
		catch (...)
		{
			// Do not turn the original recording failure into a cleanup exception.
			auto error = std::current_exception();
			try
			{
				// Only a recorded, not submitted, prefix is retired here.
				job.RetireRecordedWork();
			}
			catch (std::exception const& cleanup_error)
			{
				// Both failures remain visible to the diagnostic caller.
				std::cerr << "Constraint transaction recording retirement failed: " << cleanup_error.what() << '\n';
			}
			std::rethrow_exception(error);
		}
		job.Run();
		std::copy_n(body_readback.ptr<GpuRigidBody>(), bodies.size(), bodies.data());
		std::copy_n(record_readback.ptr<GpuConstraintRigidRecord>(), records.size(), records.data());
		std::copy_n(result_readback.ptr<uint32_t>(), results.size(), results.data());

		// Existing compiler/factor/projection failures remain fatal; per-owner admission results remain separately inspectable.
		auto const failure = state_readback.ptr<GpuConstraintSolverState>()->failure_slot_plus_one;
		if (failure != 0u)
			throw std::runtime_error(FmtS("Constraint transaction numerical failure at slot %u", failure - 1u));
	}

	// Sample final accepted completed-substep loads. No further solve may follow disposition in this epoch.
	void GpuConstraintSolver::DetectBreakage(GpuJob& job, GpuConstraintEpoch epoch, int substep_index, GpuCoupledConstraintLoadOutput coupled)
	{
		if (m_breakable_count == 0)
			return;
		if (!(epoch.timestep > 0.0f) || !std::isfinite(epoch.timestep))
			throw std::invalid_argument("Constraint break detection requires a finite positive timestep");
		if (substep_index < 0)
			throw std::invalid_argument("Constraint break detection requires a non-negative substep index");
		if (m_r_break_states == nullptr)
			throw std::logic_error("Constraint break detection requires its optional state resource");

		// Accepted records own the loads; articulation-only frames need neither a rigid body stream nor ordinary acceptance storage.
		m_break_substep_index = substep_index;
		m_transaction_epoch = epoch;
		job.m_barriers.Transition(m_r_endpoints.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(m_r_blocks.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_rows.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_break_states.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		if (m_r_rigid_records != nullptr)
			job.m_barriers.Transition(m_r_rigid_records.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);

		if (coupled.m_island_count != 0)
		{
			job.m_barriers.Transition(coupled.m_acceptance, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Transition(coupled.m_rows, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Transition(coupled.m_topology, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		}
		job.m_barriers.Commit();

		// This kernel never reads bodies or pseudo state; bind the existing unused-root sentinel, not an invented rigid allocation.
		Bind(job, m_cs_detect_breakage, epoch.timestep, 0, 0, 0, m_r_state, m_r_state.get());
		if (coupled.m_island_count != 0)
		{
			job.m_cmd_list.get()->SetComputeRoot32BitConstants(0, 1, &coupled.m_island_count, offsetof(cbConstraintSolve, coupled_island_count) / sizeof(uint32_t));
			job.m_cmd_list.SetComputeRootUnorderedAccessView(16, coupled.m_acceptance);
			job.m_cmd_list.SetComputeRootUnorderedAccessView(17, coupled.m_rows);
			job.m_cmd_list.SetComputeRootShaderResourceView(18, coupled.m_topology);
		}
		auto const group_count = static_cast<UINT>((m_slot_count + ConstraintThreadCount - 1) / ConstraintThreadCount);
		job.m_cmd_list.Dispatch(group_count, 1, 1);
		++m_dispatch_count;
		job.m_barriers.UAV(m_r_blocks.get());
		job.m_barriers.UAV(m_r_break_states.get());
		job.m_barriers.Commit();
		m_break_substep_index = -1;
	}

	// Expose the single sparse report stream without transferring resource ownership.
	GpuConstraintRejectionOutput GpuConstraintSolver::RejectionOutput()
	{
		return {m_rejection_count != 0 ? m_r_frame_rejections.get() : nullptr, m_rejection_count};
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

	// Return the current nonempty upload's frame-local solver state, or null when no slot stream was submitted.
	ID3D12Resource* GpuConstraintSolver::FrameState()
	{
		return m_slot_count != 0 ? m_r_state.get() : nullptr;
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
		// Abandoned work cannot seed a later explicit transaction epoch.
		m_transaction_source = nullptr;
		m_source = nullptr;
		m_break_substep_index = -1;
		m_r_break_states = nullptr;
		m_break_capacity = 0;
		m_breakable_count = 0;
		m_body_count = 0;
		m_dispatch_count = 0;
	}

	// True when the most recently uploaded set contains enabled persistent constraints.
	bool GpuConstraintSolver::Active() const
	{
		return m_active_count != 0;
	}

	// Return current logical usage, retained capacities, and the most recent frame dispatch count.
	GpuConstraintSolverStats GpuConstraintSolver::Stats() const
	{
		auto const slot_bytes =
			sizeof(GpuConstraintEndpoint) +
			sizeof(GpuConstraintDesc) +
			sizeof(GpuConstraintBlock) +
			GpuConstraintRowsPerBlock * sizeof(GpuConstraintRow) +
			sizeof(GpuConstraintSolveCache) +
			sizeof(GpuConstraintColourTask) +
			2 * sizeof(uint32_t);
		auto const transaction_bytes = sizeof(GpuConstraintRigidRecord) + sizeof(uint32_t);
		auto const logical_bytes =
			static_cast<size_t>(m_slot_count) * slot_bytes +
			static_cast<size_t>(m_rejection_count) * sizeof(GpuConstraintRejectionState) +
			(m_r_rigid_records != nullptr ? static_cast<size_t>(m_slot_count) * transaction_bytes : 0) +
			(m_slot_count != 0 ? sizeof(GpuConstraintSolverState) : 0) +
			static_cast<size_t>(m_body_count) * sizeof(GpuConstraintPseudoVelocity) +
			(m_breakable_count != 0 ? static_cast<size_t>(m_slot_count) * sizeof(GpuConstraintBreakState) : 0);
		auto const allocated_bytes =
			static_cast<size_t>(m_capacity) * slot_bytes +
			static_cast<size_t>(m_rejection_capacity) * sizeof(GpuConstraintRejectionState) +
			static_cast<size_t>(m_transaction_capacity) * transaction_bytes +
			(m_r_state != nullptr ? sizeof(GpuConstraintSolverState) : 0) +
			static_cast<size_t>(m_body_capacity) * sizeof(GpuConstraintPseudoVelocity) +
			static_cast<size_t>(m_break_capacity) * sizeof(GpuConstraintBreakState);
		return GpuConstraintSolverStats{
			.m_slot_count = m_slot_count,
			.m_active_count = m_active_count,
			.m_breakable_count = m_breakable_count,
			.m_slot_capacity = m_capacity,
			.m_body_capacity = m_body_capacity,
			.m_break_capacity = m_break_capacity,
			.m_dispatch_count = m_dispatch_count,
			.m_pad0 = 0,
			.m_logical_bytes = logical_bytes,
			.m_allocated_feature_bytes = allocated_bytes,
		};
	}

	// Invalidate retained frame timing when a step does not submit this optional solver lane.
	void GpuConstraintSolver::Deactivate()
	{
		// A later reactivation starts with fresh owner cache provenance.
		m_transaction_source = nullptr;
		m_slot_count = 0;
		m_rejection_count = 0;
		m_active_count = 0;
		m_breakable_count = 0;
		m_break_substep_index = -1;
		m_r_break_states = nullptr;
		m_break_capacity = 0;
		m_body_count = 0;
		m_dispatch_count = 0;
	}

	// Complete the production position/velocity phases with separately observable accepted ownership and prepared geometry.
	GpuConstraintSolveResult GpuConstraintSolver::Solve(GpuJob& job, GpuConstraintEpoch epoch, GpuConstraintUpload const& upload, std::span<GpuRigidBody> bodies, std::span<GpuConstraintBlock> blocks, std::span<GpuConstraintRow> rows, std::span<GpuConstraintBreakState> break_states)
	{
		auto const timestep = epoch.timestep;
		if (blocks.size() != 0 && blocks.size() < upload.m_endpoints.size())
			throw std::invalid_argument("Constraint block readback span is smaller than the stable-slot stream");
		if (rows.size() != 0 && rows.size() < GpuConstraintRowsPerBlock * upload.m_endpoints.size())
			throw std::invalid_argument("Constraint row readback span is smaller than the canonical stable-slot stream");
		if (break_states.size() != 0 && break_states.size() < upload.m_endpoints.size())
			throw std::invalid_argument("Constraint break-state readback span is smaller than the stable-slot stream");

		// Optional destinations expose only the written prefix; an empty upload must not bind absent geometry resources or overwrite caller tails.
		if (!blocks.empty())
			blocks = blocks.first(upload.m_endpoints.size());
		if (!rows.empty())
			rows = rows.first(GpuConstraintRowsPerBlock * upload.m_endpoints.size());
		if (!break_states.empty())
			break_states = break_states.first(upload.m_endpoints.size());

		auto const body_count = static_cast<int>(bodies.size());
		auto r_bodies = m_gpu.CreateResource(ResDesc::Buf<GpuRigidBody>(std::max(1, body_count), {}).usage(EUsage::UnorderedAccess), job, "Physics:ConstraintTestBodies");
		job.m_barriers.Transition(r_bodies.get(), D3D12_RESOURCE_STATE_COPY_DEST);
		job.m_barriers.Commit();
		auto body_upload = job.m_upload.Alloc<GpuRigidBody>(std::max(1, body_count));
		memcpy(body_upload.ptr<GpuRigidBody>(), bodies.data(), bodies.size_bytes());
		job.m_cmd_list.CopyBufferRegion(r_bodies.get(), 0, body_upload);
		job.m_barriers.Transition(r_bodies.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Commit();

		auto result = GpuConstraintSolveResult{};
		Upload(job, upload);
		if (m_slot_count != 0)
		{
			// Disabled slots remain observable historical owners with unattempted results, not synthetic zero acceptances.
			result.m_records.resize(m_slot_count);
			result.m_results.resize(m_slot_count);
			PrepareVelocityTransactions(job, epoch);
			CompileRows(job, timestep, body_count, r_bodies);
			if (m_config.push_out_iterations > 0)
				ClearPositionState(job, timestep, body_count, r_bodies);

			PreparePositionSolve(job, timestep, body_count, m_config.push_out_iterations, r_bodies);
			for (int iteration = 0; iteration != std::max(0, m_config.push_out_iterations); ++iteration)
				SolvePositionIteration(job, timestep, body_count, m_config.push_out_iterations, r_bodies);
			ApplyPosition(job, timestep, body_count, m_config.push_out_iterations, r_bodies);
			if (m_config.push_out_iterations > 0 && m_config.solver_iterations > 0)
				CompileRows(job, timestep, body_count, r_bodies);

			PrepareVelocitySolve(job, timestep, body_count, r_bodies);
			for (int iteration = 0; iteration != std::max(0, m_config.solver_iterations); ++iteration)
				CommitVelocityTransactions(job, body_count, r_bodies);

			DetectBreakage(job, epoch, 0);
		}
		// Gather optional body, block, row, and break diagnostics into the same readback transaction.
		job.m_barriers.Transition(r_bodies.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
		if (!blocks.empty())
			job.m_barriers.Transition(m_r_blocks.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
		if (!rows.empty())
			job.m_barriers.Transition(m_r_rows.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
		if (!break_states.empty() && m_r_break_states != nullptr)
			job.m_barriers.Transition(m_r_break_states.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);

		if (m_slot_count != 0)
		{
			job.m_barriers.Transition(m_r_rigid_records.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
			job.m_barriers.Transition(m_r_transaction_results.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
			job.m_barriers.Transition(m_r_state.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
		}
		job.m_barriers.Commit();
		auto body_readback = job.m_readback.Alloc<GpuRigidBody>(std::max(1, body_count));
		job.m_cmd_list.CopyBufferRegion(body_readback, r_bodies.get(), 0);
		auto block_readback = ReadbackAlloc{};
		auto row_readback = ReadbackAlloc{};
		auto break_readback = ReadbackAlloc{};
		auto record_readback = ReadbackAlloc{};
		auto result_readback = ReadbackAlloc{};
		auto state_readback = ReadbackAlloc{};
		if (m_slot_count != 0)
		{
			record_readback = job.m_readback.Alloc<GpuConstraintRigidRecord>(m_slot_count);
			result_readback = job.m_readback.Alloc<uint32_t>(m_slot_count);
			state_readback = job.m_readback.Alloc<GpuConstraintSolverState>(1);
			job.m_cmd_list.CopyBufferRegion(record_readback, m_r_rigid_records.get(), 0);
			job.m_cmd_list.CopyBufferRegion(result_readback, m_r_transaction_results.get(), 0);
			job.m_cmd_list.CopyBufferRegion(state_readback, m_r_state.get(), 0);
		}
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

		// A completed diagnostic may contain admission rejection, but a fatal slot is never reported as a successful solve.
		if (m_slot_count != 0)
		{
			std::copy_n(record_readback.ptr<GpuConstraintRigidRecord>(), m_slot_count, result.m_records.data());
			std::copy_n(result_readback.ptr<uint32_t>(), m_slot_count, result.m_results.data());
			auto const failure = state_readback.ptr<GpuConstraintSolverState>()->failure_slot_plus_one;
			if (failure != 0u)
				throw std::runtime_error(FmtS("Constraint diagnostic numerical failure at slot %u", failure - 1u));
		}
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
		return result;
	}

	// Custom deleter implementation keeps the incomplete type out of public Engine headers.
	void Deleter<GpuConstraintSolver>::operator()(GpuConstraintSolver* solver) const
	{
		delete solver;
	}
}
