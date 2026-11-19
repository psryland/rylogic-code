//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "pr/common/cast.h"
#include "src/compute/coupled_constraint_velocity_gpu.h"
#include "src/compute/gpu_capacity.h"
#include "src/compute/shader_code.h"

namespace pr::physics
{
	using namespace ::pr::compute;

	namespace
	{
		// Root-register assignments shared with coupled_constraint_velocity.hlsl.
		struct EReg
		{
			inline static constexpr auto Params = ECBufReg::b0;
			inline static constexpr auto Bodies = EUAVReg::u0;
			inline static constexpr auto Endpoints = ESRVReg::t0;
			inline static constexpr auto CoupledEndpoints = ESRVReg::t1;
			inline static constexpr auto BlockTopology = ESRVReg::t2;
			inline static constexpr auto Targets = ESRVReg::t3;
			inline static constexpr auto Adjacency = ESRVReg::t4;
			inline static constexpr auto Preconditioners = ESRVReg::t5;
			inline static constexpr auto AbaScratch = ESRVReg::t6;
			inline static constexpr auto ArticulationIslands = ESRVReg::t7;
			inline static constexpr auto Islands = ESRVReg::t8;
			inline static constexpr auto IslandBlocks = ESRVReg::t9;
			inline static constexpr auto Blocks = EUAVReg::u1;
			inline static constexpr auto Rows = EUAVReg::u2;
			inline static constexpr auto Scratch = EUAVReg::u3;
			inline static constexpr auto Contributions = EUAVReg::u4;
			inline static constexpr auto TargetImpulses = EUAVReg::u5;
			inline static constexpr auto IslandStates = EUAVReg::u6;
			inline static constexpr auto LinkImpulses = EUAVReg::u7;
			inline static constexpr auto TreeSelection = EUAVReg::u8;
			inline static constexpr auto TreeResults = EUAVReg::u9;
			inline static constexpr auto IslandFailures = EUAVReg::u10;
			inline static constexpr auto ArticulationWork = EUAVReg::u11;
			inline static constexpr auto FrameFailures = EUAVReg::u12;
			inline static constexpr auto TransactionResults = EUAVReg::u13;
		};

		// Match the HLSL bounds and phase-mode constant buffer exactly.
		struct alignas(16) cbCoupledConstraintVelocity
		{
			int m_slot_count;
			int m_body_count;
			int m_target_count;
			int m_adjacency_count;
			int m_island_count;
			int m_articulation_range_count;
			int m_mobility_count;
			int m_work_count;
			int m_island_block_count;
			int m_phase;
			int m_attempt_index;
			int m_backtrack_limit;
			float m_relaxation;
			int m_substep_index;
			float m_pad1;
			float m_pad2;
		};
		static_assert(sizeof(cbCoupledConstraintVelocity) == 64);

		// The accounting signature occupies exactly the D3D12 limit: twelve constants and twenty-six root descriptors.
		struct alignas(16) cbCoupledConstraintTransaction
		{
			int m_slot_count;
			int m_body_count;
			int m_target_count;
			int m_island_count;
			int m_range_count;
			int m_articulation_count;
			int m_velocity_count;
			int m_link_count;
			GpuConstraintEpoch m_epoch;
		};
		static_assert(sizeof(cbCoupledConstraintTransaction) == 48);

		// A root CBV keeps the private work signature at 58 DWORDs, including its evaluator-only motion/error UAV.
		struct alignas(16) cbCoupledConstraintTrial
		{
			int m_slot_count;
			int m_body_count;
			int m_island_count;
			int m_range_count;
			int m_rigid_count;
			int m_velocity_count;
			int m_link_count;
			int m_mobility_count;
			int m_island_block_count;
			int m_attempt_index;
			int m_backtrack_limit;
			int m_reference_phase;
			GpuConstraintEpoch m_epoch;
		};
		static_assert(sizeof(cbCoupledConstraintTrial) == 64);

		// Copy an explicit primary-state range before any private solver writer runs.
		void CopyCoupledState(GpuJob& job, ID3D12Resource* destination, ID3D12Resource* source, uint64_t bytes)
		{
			// Empty logical streams have a binding sentinel but no readable physical elements.
			if (bytes == 0)
				return;

			job.m_barriers.Transition(source, D3D12_RESOURCE_STATE_COPY_SOURCE);
			job.m_barriers.Transition(destination, D3D12_RESOURCE_STATE_COPY_DEST);
			job.m_barriers.Commit();
			job.m_cmd_list.CopyBufferRegion(destination, 0, source, 0, bytes);
		}

		// Create and initialize one persistent accounting resource while preserving its displaced recorded uses.
		template <typename Type>
		void ReplaceCoupledTransactionBuffer(Gpu& gpu, GpuJob& job, D3DPtr<ID3D12Resource>& resource, int count, char const* name)
		{
			// The containing set is marked incomplete before this potentially allocating retention operation.
			if (resource != nullptr)
				job.m_keep_alive.Add(resource);

			resource = gpu.CreateResource(ResDesc::Buf<Type>(std::max(1, count), {}).usage(EUsage::UnorderedAccess), job, name);
			auto data = job.m_upload.Alloc<Type>(std::max(1, count));
			std::fill_n(data.ptr<Type>(), std::max(1, count), Type{});
			job.m_barriers.Transition(resource.get(), D3D12_RESOURCE_STATE_COPY_DEST).Commit();
			job.m_cmd_list.CopyBufferRegion(resource.get(), 0, data);
		}

		// Create or retain one typed resource using geometric high-water growth.
		template <typename Type>
		void EnsureCoupledVelocityBuffer(
			Gpu& gpu,
			GpuJob& job,
			D3DPtr<ID3D12Resource>& resource,
			int required_count,
			int& capacity,
			EUsage usage,
			char const* name)
		{
			if (required_count <= capacity)
				return;

			auto const grown_capacity = GrowCapacity(required_count, capacity);
			if (resource != nullptr)
				job.m_keep_alive.Add(resource);

			resource = gpu.CreateResource(ResDesc::Buf<Type>(grown_capacity, {}).usage(usage), job, name);
			capacity = grown_capacity;
		}

		// Upload one non-empty immutable typed stream and leave it ready for shader reads.
		template <typename Type>
		void UploadCoupledVelocityStream(GpuJob& job, ID3D12Resource* resource, std::span<Type const> values)
		{
			if (resource == nullptr || values.empty())
				throw std::invalid_argument("Coupled velocity topology streams must be non-empty");

			job.m_barriers.Transition(resource, D3D12_RESOURCE_STATE_COPY_DEST).Commit();
			auto allocation = job.m_upload.Alloc<Type>(isize(values));
			memcpy(allocation.ptr<Type>(), values.data(), values.size_bytes());
			job.m_cmd_list.CopyBufferRegion(resource, 0, allocation);
			job.m_barriers.Transition(resource, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE).Commit();
		}

		// Create and initialize one diagnostic input resource, retaining a one-element sentinel for empty body input.
		template <typename Type>
		D3DPtr<ID3D12Resource> CreateCoupledVelocityInput(
			Gpu& gpu,
			GpuJob& job,
			std::span<Type const> values,
			EUsage usage,
			D3D12_RESOURCE_STATES final_state,
			std::string_view name)
		{
			auto const count = std::max(1, isize(values));
			auto resource = gpu.CreateResource(ResDesc::Buf<Type>(count, {}).usage(usage), job, name);
			job.m_barriers.Transition(resource.get(), D3D12_RESOURCE_STATE_COPY_DEST).Commit();
			auto allocation = job.m_upload.Alloc<Type>(count);
			std::fill_n(allocation.ptr<Type>(), count, Type{});
			if (!values.empty())
				memcpy(allocation.ptr<Type>(), values.data(), values.size_bytes());
			job.m_cmd_list.CopyBufferRegion(resource.get(), 0, allocation);
			job.m_barriers.Transition(resource.get(), final_state).Commit();
			return resource;
		}

		// Return the exact dispatch width for a non-empty coupled work range.
		int CoupledVelocityThreadGroupCount(int item_count)
		{
			return (item_count + ConstraintThreadCount - 1) / ConstraintThreadCount;
		}
	}

	// Create all fixed pipeline state without allocating coupled-feature resources.
	GpuCoupledConstraintVelocity::GpuCoupledConstraintVelocity(
		GpuCoupledConstraintPrepare& prepare,
		GpuArticulationImpulseAba& impulse_aba,
		EngineConfig const& config)
		:m_gpu(prepare.m_gpu)
		,m_config(config)
		,m_prepare(prepare)
		,m_impulse_aba(impulse_aba)
		,m_cs_begin()
		, m_cs_build_warm_start()
		, m_cs_candidates()
		, m_cs_gather()
		, m_cs_select_trees()
		, m_cs_validate_trees()
		, m_cs_validate_warm_start()
		, m_cs_evaluate_merit()
		,m_cs_commit()
		,m_cs_finalize_islands()
		,m_r_block_topology()
		,m_r_targets()
		,m_r_adjacency()
		,m_r_articulation_islands()
		,m_r_islands()
		,m_r_island_blocks()
		,m_r_scratch()
		,m_r_contributions()
		,m_r_target_impulses()
		,m_r_island_states()
		,m_r_island_failures()
		,m_r_tree_selection()
		,m_r_tree_results()
		,m_source()
		,m_slot_count()
		,m_target_count()
		,m_adjacency_count()
		,m_island_count()
		,m_island_block_count()
		,m_articulation_range_count()
		,m_mobility_count()
		,m_stats()
	{
		auto root_sig = RootSig(ERootSigFlags::ComputeOnly)
			.U32<cbCoupledConstraintVelocity>(EReg::Params)
			.UAV(EReg::Bodies)
			.SRV(EReg::Endpoints)
			.SRV(EReg::CoupledEndpoints)
			.SRV(EReg::BlockTopology)
			.SRV(EReg::Targets)
			.SRV(EReg::Adjacency)
			.SRV(EReg::Preconditioners)
			.SRV(EReg::AbaScratch)
			.SRV(EReg::ArticulationIslands)
			.SRV(EReg::Islands)
			.SRV(EReg::IslandBlocks)
			.UAV(EReg::Blocks)
			.UAV(EReg::Rows)
			.UAV(EReg::Scratch)
			.UAV(EReg::Contributions)
			.UAV(EReg::TargetImpulses)
			.UAV(EReg::IslandStates)
			.UAV(EReg::LinkImpulses)
			.UAV(EReg::TreeSelection)
			.UAV(EReg::TreeResults)
			.UAV(EReg::IslandFailures)
			.UAV(EReg::ArticulationWork)
			.UAV(EReg::FrameFailures)
			.UAV(EReg::TransactionResults)
			.Create(m_gpu, "Physics:CoupledConstraintVelocitySig");
		m_cs_begin.m_sig = root_sig;
		m_cs_begin.m_pso = ComputePSO(root_sig.get(), shader_code::begin_coupled_velocity).Create(m_gpu, "Physics:BeginCoupledVelocityPSO");
		m_cs_build_warm_start.m_sig = root_sig;
		m_cs_build_warm_start.m_pso = ComputePSO(root_sig.get(), shader_code::build_coupled_velocity_warm_start).Create(m_gpu, "Physics:BuildCoupledVelocityWarmStartPSO");
		m_cs_candidates.m_sig = root_sig;
		m_cs_candidates.m_pso = ComputePSO(root_sig.get(), shader_code::build_coupled_velocity_candidates).Create(m_gpu, "Physics:BuildCoupledVelocityCandidatesPSO");
		m_cs_gather.m_sig = root_sig;
		m_cs_gather.m_pso = ComputePSO(root_sig.get(), shader_code::gather_coupled_velocity_targets).Create(m_gpu, "Physics:GatherCoupledVelocityTargetsPSO");
		m_cs_select_trees.m_sig = root_sig;
		m_cs_select_trees.m_pso = ComputePSO(root_sig.get(), shader_code::select_coupled_velocity_trees).Create(m_gpu, "Physics:SelectCoupledVelocityTreesPSO");
		m_cs_validate_trees.m_sig = root_sig;
		m_cs_validate_trees.m_pso = ComputePSO(root_sig.get(), shader_code::validate_coupled_velocity_trees).Create(m_gpu, "Physics:ValidateCoupledVelocityTreesPSO");
		m_cs_validate_warm_start.m_sig = root_sig;
		m_cs_validate_warm_start.m_pso = ComputePSO(root_sig.get(), shader_code::validate_coupled_velocity_warm_start).Create(m_gpu, "Physics:ValidateCoupledVelocityWarmStartPSO");
		m_cs_commit.m_sig = root_sig;
		m_cs_commit.m_pso = ComputePSO(root_sig.get(), shader_code::commit_coupled_velocity).Create(m_gpu, "Physics:CommitCoupledVelocityPSO");
		m_cs_finalize_islands.m_sig = root_sig;
		m_cs_finalize_islands.m_pso = ComputePSO(root_sig.get(), shader_code::finalize_coupled_velocity_islands).Create(m_gpu, "Physics:FinalizeCoupledVelocityIslandsPSO");

		// Accounting has separate live and private resources; numerical projection keeps its unchanged root layout.
		auto accounting = RootSig(ERootSigFlags::ComputeOnly).U32<cbCoupledConstraintTransaction>(ECBufReg::b0);
		for (int index = 0; index != 8; ++index)
		{
			// These contiguous register ranges match coupled_constraint_transaction.hlsl.
			accounting.SRV(static_cast<ESRVReg>(index));
		}
		for (int index = 0; index != 18; ++index)
		{
			// No descriptor aliases a private state buffer with its corresponding live state.
			accounting.UAV(static_cast<EUAVReg>(index));
		}
		auto accounting_sig = accounting.Create(m_gpu, "Physics:CoupledTransactionSig");
		auto create = [&](ComputeStep& step, shader_code::ByteCode const& code, char const* name)
		{
			// All accounting phases share the same owner binding.
			step.m_sig = accounting_sig;
			step.m_pso = ComputePSO(accounting_sig.get(), code).Create(m_gpu, name);
		};
		create(m_cs_transaction_recover, shader_code::recover_coupled_transaction, "Physics:RecoverCoupledTransaction");
		create(m_cs_transaction_select, shader_code::select_coupled_transaction_trees, "Physics:SelectCoupledTransactionTrees");
		create(m_cs_transaction_validate, shader_code::validate_coupled_transaction_trees, "Physics:ValidateCoupledTransactionTrees");
		create(m_cs_transaction_admit_seed, shader_code::admit_coupled_transaction_seed, "Physics:AdmitCoupledTransactionSeed");
		create(m_cs_transaction_seed, shader_code::seed_coupled_transaction, "Physics:SeedCoupledTransaction");
		create(m_cs_transaction_collect, shader_code::collect_coupled_transaction_solve, "Physics:CollectCoupledTransactionSolve");
		create(m_cs_transaction_stage, shader_code::stage_coupled_transaction, "Physics:StageCoupledTransaction");
		create(m_cs_transaction_publish, shader_code::publish_coupled_transaction, "Physics:PublishCoupledTransaction");

		// References remain compact; only the evaluator owns the extra 192-byte record per participating link.
		auto trial = RootSig(ERootSigFlags::ComputeOnly).CBuf(ECBufReg::b0);
		for (int index = 0; index != 16; ++index)
			trial.SRV(static_cast<ESRVReg>(index));

		for (int index = 0; index != 12; ++index)
			trial.UAV(static_cast<EUAVReg>(index));

		auto trial_sig = trial.Create(m_gpu, "Physics:CoupledTrialSig");
		auto create_trial = [&](ComputeStep& step, shader_code::ByteCode const& code, char const* name)
		{
			step.m_sig = trial_sig;
			step.m_pso = ComputePSO(trial_sig.get(), code).Create(m_gpu, name);
		};
		create_trial(m_cs_capture_reference, shader_code::capture_coupled_reference, "Physics:CaptureCoupledReference");
		create_trial(m_cs_choose_reference, shader_code::choose_coupled_reference, "Physics:ChooseCoupledReference");
		create_trial(m_cs_admit_initialization, shader_code::admit_coupled_initialization, "Physics:AdmitCoupledInitialization");
		create_trial(m_cs_restore_reference, shader_code::restore_coupled_reference, "Physics:RestoreCoupledReference");
		create_trial(m_cs_begin_trial, shader_code::begin_coupled_velocity_trial, "Physics:BeginCoupledVelocityTrial");
		create_trial(m_cs_evaluate_merit, shader_code::evaluate_coupled_velocity_merit, "Physics:EvaluateCoupledVelocityMerit");
	}

	// Upload deterministic persistent topology after matching constraint and articulation participation streams.
	bool GpuCoupledConstraintVelocity::Upload(GpuJob& job, GpuConstraintUpload const& upload)
	{
		m_sweep_profiles.clear();
		m_stats.m_dispatch_count = 0;
		if (upload.m_coupled_active_count == 0)
		{
			// Earlier commands in this recording may still reference the deactivated owner's resources.
			for (auto resource : {m_r_block_topology, m_r_targets, m_r_adjacency, m_r_articulation_islands, m_r_islands, m_r_island_blocks,
				m_r_scratch, m_r_contributions, m_r_target_impulses, m_r_island_states, m_r_island_failures,
				m_r_tree_selection, m_r_tree_results, m_r_transaction_acceptance, m_r_transaction_rows, m_r_rigid_response,
				m_r_velocity_response, m_r_transaction_results, m_r_work_bodies, m_r_work_velocities, m_r_work_aba_scratch,
				m_r_reference_momenta, m_r_reference_velocities, m_r_initial_coefficients, m_r_work_spans,
				m_r_work_indices, m_r_reference_choices, m_r_work_motion})
			{
				// Retain before dropping any member so an allocation failure leaves the existing owner intact.
				if (resource != nullptr)
					job.m_keep_alive.Add(resource);
			}
			ReleaseBuffers();
			return false;
		}
		if (
			m_prepare.m_source != upload.m_source ||
			m_prepare.m_slot_count != isize(upload.m_endpoints) ||
			m_prepare.m_active_count != s_cast<int>(upload.m_coupled_active_count))
			throw std::logic_error("Coupled velocity topology must follow matching coupled preparation upload");
		if (
			upload.m_coupled_block_topology.size() != upload.m_endpoints.size() ||
			upload.m_coupled_targets.empty() ||
			upload.m_coupled_target_adjacency.empty() ||
			upload.m_coupled_islands.empty() ||
			upload.m_coupled_island_blocks.empty() ||
			upload.m_coupled_articulation_islands.size() != upload.m_coupled_articulation_indices.size())
			throw std::invalid_argument("Coupled velocity upload requires complete deterministic topology");
		if (m_impulse_aba.m_mobility.Ranges().size() != upload.m_coupled_articulation_indices.size())
			throw std::logic_error("Coupled velocity topology must match prepared articulation participation");

		// Membership includes stable owners and packed response indices, but excludes poses, mobility values, and parameter edits.
		auto membership = std::vector<std::byte>{};
		auto append = [&](auto const& values)
		{
			// All packed stream structs are initialized by the validated producer.
			auto bytes = std::as_bytes(std::span(values));
			membership.insert(membership.end(), bytes.begin(), bytes.end());
		};
		append(upload.m_coupled_block_topology);
		append(upload.m_coupled_targets);
		append(upload.m_coupled_endpoints);
		append(upload.m_coupled_articulation_indices);
		append(upload.m_coupled_articulation_islands);
		append(m_impulse_aba.m_mobility.Ranges());
		auto const membership_changed = m_membership_source != upload.m_source || membership != m_membership_key || m_membership_identities != upload.m_endpoint_identities;
		if (membership_changed && m_membership_revision == std::numeric_limits<uint64_t>::max())
			throw std::overflow_error("Coupled membership revision exhausted");

		// Allocate both CPU keys before publishing either; a failed copy must not hide a membership change on retry.
		auto identities = upload.m_endpoint_identities;

		// Count and prefix compact owner ranges in linear time without changing the shared topology ABI.
		auto work_spans = std::vector<GpuCoupledConstraintWorkSpan>(upload.m_coupled_islands.size());
		for (auto const& target : upload.m_coupled_targets)
		{
			if (target.target_type == GpuCoupledConstraintTargetType_Rigid)
				++work_spans.at(target.island_idx).rigid_count;
		}
		for (auto island : upload.m_coupled_articulation_islands)
			++work_spans.at(island).tree_count;

		auto offset = uint32_t{};
		for (auto& span : work_spans)
		{
			span.rigid_offset = offset;
			offset += span.rigid_count;
		}
		auto rigid_count = s_cast<int>(offset);
		for (auto& span : work_spans)
		{
			span.tree_offset = offset;
			offset += span.tree_count;
		}
		auto work_indices = std::vector<uint32_t>(offset);
		auto cursors = work_spans;
		for (int index = 0; index != isize(upload.m_coupled_targets); ++index)
		{
			auto const& target = upload.m_coupled_targets[index];
			if (target.target_type == GpuCoupledConstraintTargetType_Rigid)
				work_indices[cursors[target.island_idx].rigid_offset++] = s_cast<uint32_t>(index);
		}
		for (int index = 0; index != isize(upload.m_coupled_articulation_islands); ++index)
			work_indices[cursors[upload.m_coupled_articulation_islands[index]].tree_offset++] = s_cast<uint32_t>(index);

		m_source = upload.m_source;
		m_slot_count = isize(upload.m_endpoints);
		m_target_count = isize(upload.m_coupled_targets);
		m_adjacency_count = isize(upload.m_coupled_target_adjacency);
		m_island_count = isize(upload.m_coupled_islands);
		m_island_block_count = isize(upload.m_coupled_island_blocks);
		m_articulation_range_count = isize(upload.m_coupled_articulation_indices);
		auto const ranges = m_impulse_aba.m_mobility.Ranges();
		m_mobility_count = ranges.empty() ? 0 : ranges.back().mobility_offset + ranges.back().link_count;
		if (m_mobility_count <= 0 || !m_impulse_aba.Prepare(job))
			throw std::logic_error("Coupled velocity upload requires prepared articulation impulse storage");
		ResizeBuffers(job);

		// Immutable topology is rebuilt frame-locally because rigid and articulation packing indices may change.
		UploadCoupledVelocityStream(job, m_r_block_topology.get(), std::span{upload.m_coupled_block_topology});
		UploadCoupledVelocityStream(job, m_r_targets.get(), std::span{upload.m_coupled_targets});
		UploadCoupledVelocityStream(job, m_r_adjacency.get(), std::span{upload.m_coupled_target_adjacency});
		UploadCoupledVelocityStream(job, m_r_articulation_islands.get(), std::span{upload.m_coupled_articulation_islands});
		UploadCoupledVelocityStream(job, m_r_islands.get(), std::span{upload.m_coupled_islands});
		UploadCoupledVelocityStream(job, m_r_island_blocks.get(), std::span{upload.m_coupled_island_blocks});

		m_stats.m_active_count = s_cast<int>(upload.m_coupled_active_count);
		m_stats.m_logical_bytes =
			static_cast<size_t>(m_slot_count) *
				(sizeof(GpuCoupledConstraintBlockTopology) + sizeof(GpuCoupledConstraintSolveScratch) + 2 * sizeof(GpuArticulationSpatialVector)) +
			static_cast<size_t>(m_target_count) *
				(sizeof(GpuCoupledConstraintTarget) + sizeof(GpuArticulationSpatialVector)) +
			static_cast<size_t>(m_adjacency_count) * sizeof(uint32_t) +
			static_cast<size_t>(m_island_count) *
				(sizeof(GpuCoupledConstraintIsland) + sizeof(GpuCoupledConstraintIslandState) + sizeof(uint32_t)) +
			static_cast<size_t>(m_island_block_count) * sizeof(uint32_t) +
			static_cast<size_t>(m_articulation_range_count) * 3 * sizeof(uint32_t);
		m_stats.m_allocated_feature_bytes =
			static_cast<size_t>(m_stats.m_slot_capacity) *
				(sizeof(GpuCoupledConstraintBlockTopology) + sizeof(GpuCoupledConstraintSolveScratch) + 2 * sizeof(GpuArticulationSpatialVector)) +
			static_cast<size_t>(m_stats.m_target_capacity) *
				(sizeof(GpuCoupledConstraintTarget) + sizeof(GpuArticulationSpatialVector)) +
			static_cast<size_t>(m_stats.m_adjacency_capacity) * sizeof(uint32_t) +
			static_cast<size_t>(m_stats.m_island_capacity) *
				(sizeof(GpuCoupledConstraintIsland) + sizeof(GpuCoupledConstraintIslandState) + sizeof(uint32_t)) +
			static_cast<size_t>(m_stats.m_island_block_capacity) * sizeof(uint32_t) +
			static_cast<size_t>(m_stats.m_articulation_range_capacity) * 3 * sizeof(uint32_t);

		// Do not publish a new membership stamp while any dependent upload can still fail.
		m_membership_key = std::move(membership);
		m_membership_identities = std::move(identities);
		m_membership_source = upload.m_source;
		m_work_spans = std::move(work_spans);
		m_work_indices = std::move(work_indices);
		m_rigid_work_count = rigid_count;
		if (membership_changed)
			++m_membership_revision;

		return true;
	}

	// Prepare persistent response/cache ownership without physically publishing provisional guesses.
	void GpuCoupledConstraintVelocity::PrepareVelocityTransactions(GpuJob& job, GpuConstraintEpoch epoch, int body_count)
	{
		// Epoch identity and component packing must remain stable for every current-substep response.
		auto& aba = m_impulse_aba.m_aba;
		if (m_source == nullptr || m_island_count <= 0 || body_count < 0)
			throw std::invalid_argument("Coupled transaction preparation requires active matching topology and a nonnegative body count");

		auto scale = 0.0f;
		if (!ConstraintStageGuessScale({}, epoch, scale))
			throw std::invalid_argument("Coupled transaction epoch requires a finite positive timestep and finite nonnegative warm factor");

		auto const changed = m_transaction_membership_revision != m_membership_revision ||
			m_transaction_body_count != body_count || m_transaction_velocity_count != aba.m_velocity_count || m_transaction_link_count != aba.m_link_count;
		if (m_transaction_membership_revision != 0 && All(m_transaction_epoch.identity == epoch.identity) && changed)
			throw std::logic_error("Coupled transaction membership or component packing changed within one epoch");

		if (!m_transaction_ready || changed)
		{
			// Any partial allocation or initialization forces recreation; old recorded users are retained by each replacement.
			m_transaction_ready = false;
			ReplaceCoupledTransactionBuffer<GpuConstraintAcceptance>(m_gpu, job, m_r_transaction_acceptance, m_island_count, "Physics:CoupledAcceptedEpochs");
			ReplaceCoupledTransactionBuffer<GpuConstraintAcceptedRows>(m_gpu, job, m_r_transaction_rows, m_slot_count, "Physics:CoupledAcceptedRows");
			ReplaceCoupledTransactionBuffer<GpuArticulationSpatialVector>(m_gpu, job, m_r_rigid_response, body_count, "Physics:CoupledRigidResponses");
			ReplaceCoupledTransactionBuffer<float>(m_gpu, job, m_r_velocity_response, aba.m_velocity_count, "Physics:CoupledVelocityResponses");
			ReplaceCoupledTransactionBuffer<uint32_t>(m_gpu, job, m_r_transaction_results, m_island_count, "Physics:CoupledTransactionResults");
			ReplaceCoupledTransactionBuffer<GpuRigidBody>(m_gpu, job, m_r_work_bodies, body_count, "Physics:CoupledWorkingBodies");
			ReplaceCoupledTransactionBuffer<float>(m_gpu, job, m_r_work_velocities, aba.m_velocity_count, "Physics:CoupledWorkingVelocities");
			ReplaceCoupledTransactionBuffer<GpuArticulationAbaScratch>(m_gpu, job, m_r_work_aba_scratch, aba.m_link_count, "Physics:CoupledWorkingAbaScratch");
			ReplaceCoupledTransactionBuffer<GpuCoupledConstraintMomentumReference>(m_gpu, job, m_r_reference_momenta, m_rigid_work_count, "Physics:CoupledMomentumReferences");
			ReplaceCoupledTransactionBuffer<GpuCoupledConstraintSpeedReference>(m_gpu, job, m_r_reference_velocities, m_impulse_aba.m_velocity_delta_count, "Physics:CoupledSpeedReferences");
			ReplaceCoupledTransactionBuffer<float>(m_gpu, job, m_r_initial_coefficients, GpuConstraintRowsPerBlock * m_slot_count, "Physics:CoupledInitialCoefficients");
			ReplaceCoupledTransactionBuffer<GpuCoupledConstraintWorkSpan>(m_gpu, job, m_r_work_spans, m_island_count, "Physics:CoupledWorkSpans");
			ReplaceCoupledTransactionBuffer<uint32_t>(m_gpu, job, m_r_work_indices, isize(m_work_indices), "Physics:CoupledWorkIndices");
			ReplaceCoupledTransactionBuffer<uint32_t>(m_gpu, job, m_r_reference_choices, m_island_count, "Physics:CoupledReferenceChoices");
			ReplaceCoupledTransactionBuffer<GpuCoupledConstraintWorkMotion>(m_gpu, job, m_r_work_motion, m_mobility_count, "Physics:CoupledWorkMotion");
			UploadCoupledVelocityStream(job, m_r_work_spans.get(), std::span<GpuCoupledConstraintWorkSpan const>{m_work_spans});
			UploadCoupledVelocityStream(job, m_r_work_indices.get(), std::span<uint32_t const>{m_work_indices});
		}

		// Preparation alone never accepts a warm cache or an owned response.
		auto results = job.m_upload.Alloc<uint32_t>(m_island_count);
		std::fill_n(results.ptr<uint32_t>(), m_island_count, 0xFFFFFFFFu);
		job.m_barriers.Transition(m_r_transaction_results.get(), D3D12_RESOURCE_STATE_COPY_DEST).Commit();
		job.m_cmd_list.CopyBufferRegion(m_r_transaction_results.get(), 0, results);
		m_transaction_epoch = epoch;
		m_transaction_membership_revision = m_membership_revision;
		m_transaction_body_count = body_count;
		m_transaction_velocity_count = aba.m_velocity_count;
		m_transaction_link_count = aba.m_link_count;
		m_transaction_ready = true;
	}

	// Attempt a replacement against private state; publication is a separate complete-island gate.
	void GpuCoupledConstraintVelocity::CommitVelocityTransactions(GpuJob& job, int body_count, ID3D12Resource* bodies, int substep_index)
	{
		// Count recorded numerical and ABA helper dispatches independently of GPU-authored evaluated attempts.
		auto const dispatch_begin = m_stats.m_dispatch_count + m_impulse_aba.Stats().m_dispatch_count;

		// A numerical attempt cannot silently consume stale packing or incomplete storage.
		auto& aba = m_impulse_aba.m_aba;
		if (!m_transaction_ready || m_transaction_membership_revision != m_membership_revision || body_count != m_transaction_body_count ||
			aba.m_velocity_count != m_transaction_velocity_count || aba.m_link_count != m_transaction_link_count || bodies == nullptr)
			throw std::logic_error("Coupled transaction commit requires matching prepared owner storage");

		if (!(m_config.constraint_coupled_relaxation > 0.0f) || m_config.constraint_coupled_relaxation > 1.0f || !std::isfinite(m_config.constraint_coupled_relaxation))
			throw std::invalid_argument("Coupled constraint relaxation must be finite and in (0,1]");

		if (m_config.constraint_coupled_backtrack_limit < 0 || m_config.constraint_coupled_backtrack_limit > 8)
			throw std::invalid_argument("Coupled constraint backtracking must be between zero and eight retries");

		// No contact or other owner may interleave between these copies and the final publication barrier.
		auto live = m_impulse_aba.VelocityBuffers();
		auto work = GpuArticulationVelocityBuffers{.m_velocities = m_r_work_velocities.get(), .m_scratch = m_r_work_aba_scratch.get()};
		CopyCoupledState(job, m_r_work_bodies.get(), bodies, static_cast<uint64_t>(body_count) * sizeof(GpuRigidBody));
		CopyCoupledState(job, work.m_scratch, live.m_scratch, static_cast<uint64_t>(aba.m_link_count) * sizeof(GpuArticulationAbaScratch));
		auto results = job.m_upload.Alloc<uint32_t>(m_island_count);
		std::fill_n(results.ptr<uint32_t>(), m_island_count, GpuConstraintStage_Accepted);
		job.m_barriers.Transition(m_r_transaction_results.get(), D3D12_RESOURCE_STATE_COPY_DEST).Commit();
		job.m_cmd_list.CopyBufferRegion(m_r_transaction_results.get(), 0, results);
		auto const work_count = std::max({m_slot_count, m_target_count, m_island_count, m_articulation_range_count, m_mobility_count});
		Dispatch(job, m_cs_begin, GpuCoupledConstraintPhase_Evaluate, 0, work_count, body_count, m_r_work_bodies.get(), work, -1);
		DispatchTrial(job, m_cs_capture_reference, body_count, bodies, 0);

		// Recover actual free state and compatible guesses, then admit its derived link velocities before solving.
		DispatchTransaction(job, m_cs_transaction_recover, body_count, bodies);
		DispatchTransaction(job, m_cs_transaction_select, body_count, bodies);
		m_impulse_aba.RefreshVelocities(job, m_r_tree_selection.get(), m_r_tree_results.get(), work);
		DispatchTransaction(job, m_cs_transaction_validate, body_count, bodies);
		DispatchTransaction(job, m_cs_transaction_select, body_count, bodies);

		// Keep actual current-epoch primary state/ledger separate from its recovered free reference.
		DispatchTrial(job, m_cs_capture_reference, body_count, bodies, 1);
		DispatchTrial(job, m_cs_choose_reference, body_count, bodies);

		// Gather provisional coefficients through the existing operators without calling the old published-warm API.
		Dispatch(job, m_cs_build_warm_start, GpuCoupledConstraintPhase_Evaluate, 0, m_slot_count, body_count, m_r_work_bodies.get(), work, -1);
		Dispatch(job, m_cs_gather, GpuCoupledConstraintPhase_Evaluate, 0, m_target_count, body_count, m_r_work_bodies.get(), work, -1);
		Dispatch(job, m_cs_select_trees, GpuCoupledConstraintPhase_Evaluate, 0, m_articulation_range_count, body_count, m_r_work_bodies.get(), work, -1);
		m_impulse_aba.EvaluateResponse(job, m_r_tree_selection.get(), m_r_tree_results.get());
		Dispatch(job, m_cs_validate_trees, GpuCoupledConstraintPhase_Evaluate, 0, m_articulation_range_count, body_count, m_r_work_bodies.get(), work, -1);
		Dispatch(job, m_cs_validate_warm_start, GpuCoupledConstraintPhase_Evaluate, 0, m_island_count, body_count, m_r_work_bodies.get(), work, -1);
		DispatchTransaction(job, m_cs_transaction_admit_seed, body_count, bodies);
		DispatchTransaction(job, m_cs_transaction_seed, body_count, bodies);
		DispatchTransaction(job, m_cs_transaction_stage, body_count, bodies);
		DispatchTransaction(job, m_cs_transaction_select, body_count, bodies);
		m_impulse_aba.RefreshVelocities(job, m_r_tree_selection.get(), m_r_tree_results.get(), work);
		DispatchTransaction(job, m_cs_transaction_validate, body_count, bodies);
		DispatchTransaction(job, m_cs_transaction_select, body_count, bodies);

		// Decline an uphill warm initialization before direction construction, then capture the one trusted numerical initial state.
		DispatchTrial(job, m_cs_admit_initialization, body_count, bodies);
		DispatchTrial(job, m_cs_restore_reference, body_count, bodies, 3);
		DispatchTransaction(job, m_cs_transaction_select, body_count, bodies);
		m_impulse_aba.RefreshVelocities(job, m_r_tree_selection.get(), m_r_tree_results.get(), work);
		DispatchTransaction(job, m_cs_transaction_validate, body_count, bodies);
		DispatchTransaction(job, m_cs_transaction_select, body_count, bodies);
		DispatchTrial(job, m_cs_capture_reference, body_count, bodies, 2);

		// Early rejection is reported before an unexecuted numerical sweep can be mistaken for measured backtracking.
		Dispatch(job, m_cs_finalize_islands, GpuCoupledConstraintPhase_Evaluate, 0, m_island_count, body_count, m_r_work_bodies.get(), work, substep_index);

		// One authoritative Pending-only trial loop stages and checks the actual publication candidate before accepting it.
		for (int attempt = 0; attempt != m_config.constraint_coupled_backtrack_limit + 1; ++attempt)
		{
			DispatchTrial(job, m_cs_begin_trial, body_count, bodies, 0, attempt);
			DispatchTrial(job, m_cs_restore_reference, body_count, bodies, 4, attempt);
			DispatchTransaction(job, m_cs_transaction_select, body_count, bodies);
			m_impulse_aba.RefreshVelocities(job, m_r_tree_selection.get(), m_r_tree_results.get(), work);
			DispatchTransaction(job, m_cs_transaction_validate, body_count, bodies);
			DispatchTransaction(job, m_cs_transaction_select, body_count, bodies);
			Dispatch(job, m_cs_candidates, GpuCoupledConstraintPhase_Evaluate, attempt, m_slot_count, body_count, m_r_work_bodies.get(), work, -1);
			Dispatch(job, m_cs_gather, GpuCoupledConstraintPhase_Evaluate, attempt, m_target_count, body_count, m_r_work_bodies.get(), work, -1);
			Dispatch(job, m_cs_select_trees, GpuCoupledConstraintPhase_Evaluate, attempt, m_articulation_range_count, body_count, m_r_work_bodies.get(), work, -1);
			m_impulse_aba.Evaluate(job, m_r_tree_selection.get(), m_r_tree_results.get(), work);
			Dispatch(job, m_cs_validate_trees, GpuCoupledConstraintPhase_Evaluate, attempt, m_articulation_range_count, body_count, m_r_work_bodies.get(), work, -1);
			Dispatch(job, m_cs_select_trees, GpuCoupledConstraintPhase_Evaluate, attempt, m_articulation_range_count, body_count, m_r_work_bodies.get(), work, -1);
			Dispatch(job, m_cs_commit, GpuCoupledConstraintPhase_Evaluate, attempt, work_count, body_count, m_r_work_bodies.get(), work, -1);
			m_impulse_aba.Commit(job, m_r_tree_selection.get(), m_r_tree_results.get(), work);
			DispatchTransaction(job, m_cs_transaction_stage, body_count, bodies);
			DispatchTransaction(job, m_cs_transaction_select, body_count, bodies);
			m_impulse_aba.RefreshVelocities(job, m_r_tree_selection.get(), m_r_tree_results.get(), work);
			DispatchTransaction(job, m_cs_transaction_validate, body_count, bodies);
			DispatchTrial(job, m_cs_evaluate_merit, body_count, bodies, 0, attempt);
		}
		DispatchTransaction(job, m_cs_transaction_collect, body_count, bodies);

		// Successful islands already hold their final staged bodies, complete-tree caches, and actual represented response ledger.
		DispatchTransaction(job, m_cs_transaction_publish, body_count, bodies);
		Dispatch(job, m_cs_finalize_islands, GpuCoupledConstraintPhase_CommitVelocity, m_config.constraint_coupled_backtrack_limit, m_island_count, body_count, bodies, live, substep_index);

		// Snapshot only diagnostics, with no extra submission or wait. Restore states before the next numerical consumer.
		if (pix::DetailEnabled())
		{
			auto profile = SweepProfile{
				.m_island_count = m_island_count,
				.m_recorded_dispatches = m_stats.m_dispatch_count + m_impulse_aba.Stats().m_dispatch_count - dispatch_begin,
				.m_states = job.m_readback.Alloc<GpuCoupledConstraintIslandState>(m_island_count),
				.m_results = job.m_readback.Alloc<uint32_t>(m_island_count),
			};
			job.m_barriers.Transition(m_r_island_states.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
			job.m_barriers.Transition(m_r_transaction_results.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
			job.m_barriers.Commit();
			job.m_cmd_list.CopyBufferRegion(profile.m_states, m_r_island_states.get(), 0);
			job.m_cmd_list.CopyBufferRegion(profile.m_results, m_r_transaction_results.get(), 0);
			job.m_barriers.Transition(m_r_island_states.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Transition(m_r_transaction_results.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Commit();
			m_sweep_profiles.push_back(std::move(profile));
		}
	}

	// Return the owner result without changing any resource or accepting provisional data.
	GpuCoupledConstraintTransactionOutput GpuCoupledConstraintVelocity::TransactionOutput()
	{
		// Partial or stale owner storage is not a usable publication result.
		if (!m_transaction_ready || m_transaction_membership_revision != m_membership_revision)
			throw std::logic_error("Coupled transaction output requires complete matching storage");

		return {m_r_transaction_acceptance.get(), m_r_transaction_rows.get(), m_r_rigid_response.get(), m_r_velocity_response.get(),
			m_r_transaction_results.get(), m_r_island_states.get(), m_prepare.m_constraints.m_r_rows.get(), m_prepare.m_constraints.m_r_blocks.get(), m_slot_count, m_island_count};
	}

	// Prepare all dependent production lanes and read one coupled sweep through exactly one GPU submission.
	GpuCoupledConstraintVelocityResult GpuCoupledConstraintVelocity::Solve(
		GpuJob& job,
		GpuConstraintEpoch epoch,
		GpuConstraintUpload const& constraint_upload,
		GpuArticulationUpload const& articulation_upload,
		std::span<GpuRigidBody const> bodies,
		std::span<GpuConstraintFrame const> link_to_world,
		std::span<GpuCoupledConstraintPreconditioner const> preconditioner_override,
		std::span<GpuConstraintRow const> prepared_row_override)
	{
		auto const timestep = epoch.timestep;
		if (!(timestep > 0.0f) || !std::isfinite(timestep))
			throw std::invalid_argument("Coupled velocity diagnostics require a finite positive timestep");
		if (link_to_world.size() != articulation_upload.m_links.size())
			throw std::invalid_argument("Coupled velocity link-frame count must match the packed articulation forest");

		auto& constraints = m_prepare.m_constraints;
		auto& mobility = m_impulse_aba.m_mobility;
		auto& aba = m_impulse_aba.m_aba;
		constraints.Upload(job, constraint_upload);
		if (!aba.Upload(job, articulation_upload))
			throw std::invalid_argument("Coupled velocity diagnostics require a non-empty articulation forest");
		if (!mobility.Upload(job, articulation_upload, constraint_upload.m_coupled_articulation_indices))
			throw std::invalid_argument("Coupled velocity diagnostics require participating articulation ranges");
		if (!m_prepare.Upload(job, constraint_upload))
			return {};
		if (!Upload(job, constraint_upload))
			return {};

		auto r_bodies = CreateCoupledVelocityInput(m_gpu, job, bodies, EUsage::UnorderedAccess, D3D12_RESOURCE_STATE_UNORDERED_ACCESS, "Physics:CoupledVelocityTestBodies");
		auto r_link_to_world = CreateCoupledVelocityInput(m_gpu, job, link_to_world, EUsage::Default, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE, "Physics:CoupledVelocityTestLinkFrames");
		mobility.Run(job);

		// Geometry supplies factors; transaction recovery explicitly rebuilds the velocity cache from private free state.
		m_prepare.Run(job, timestep, isize(bodies), r_bodies.get(), r_link_to_world.get(), mobility);
		PrepareVelocityTransactions(job, epoch, isize(bodies));
		if (!preconditioner_override.empty())
		{
			if (preconditioner_override.size() != constraint_upload.m_endpoints.size())
				throw std::invalid_argument("Coupled velocity preconditioner override must contain one stable-slot value");

			job.m_barriers.Transition(m_prepare.m_r_preconditioners.get(), D3D12_RESOURCE_STATE_COPY_DEST).Commit();
			auto allocation = job.m_upload.Alloc<GpuCoupledConstraintPreconditioner>(isize(preconditioner_override));
			memcpy(allocation.ptr<GpuCoupledConstraintPreconditioner>(), preconditioner_override.data(), preconditioner_override.size_bytes());
			job.m_cmd_list.CopyBufferRegion(m_prepare.m_r_preconditioners.get(), 0, allocation);
		}
		if (!prepared_row_override.empty())
		{
			if (prepared_row_override.size() != static_cast<size_t>(GpuConstraintRowsPerBlock * m_slot_count))
				throw std::invalid_argument("Coupled velocity prepared-row override must contain six rows per stable slot");

			job.m_barriers.Transition(constraints.m_r_rows.get(), D3D12_RESOURCE_STATE_COPY_DEST).Commit();
			auto allocation = job.m_upload.Alloc<GpuConstraintRow>(isize(prepared_row_override));
			memcpy(allocation.ptr<GpuConstraintRow>(), prepared_row_override.data(), prepared_row_override.size_bytes());
			job.m_cmd_list.CopyBufferRegion(constraints.m_r_rows.get(), 0, allocation);
		}
		// Production staging, admission, and publication are the only physical path used by this diagnostic.
		CommitVelocityTransactions(job, isize(bodies), r_bodies.get());

		// Capture all authoritative outputs and transaction diagnostics before the caller's single submission.
		auto result = GpuCoupledConstraintVelocityResult{};
		result.m_bodies.resize(bodies.size());
		result.m_rows.resize(static_cast<size_t>(GpuConstraintRowsPerBlock) * m_slot_count);
		result.m_articulation_velocities.resize(aba.m_velocity_count);
		result.m_articulation_accelerations.resize(aba.m_acceleration_count);
		result.m_articulation_scratch.resize(aba.m_link_count);
		result.m_articulation_dof_scratch.resize(aba.m_dof_count);
		result.m_island_states.resize(m_island_count);
		result.m_acceptance.resize(m_island_count);
		result.m_accepted_rows.resize(m_slot_count);
		result.m_results.resize(m_island_count);
		job.m_barriers.Transition(r_bodies.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
		job.m_barriers.Transition(constraints.m_r_rows.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
		if (aba.m_velocity_count != 0)
			job.m_barriers.Transition(aba.m_r_velocities.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
		if (aba.m_acceleration_count != 0)
			job.m_barriers.Transition(aba.m_r_accelerations.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
		job.m_barriers.Transition(aba.m_r_scratch.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
		if (aba.m_dof_count != 0)
			job.m_barriers.Transition(aba.m_r_dof_scratch.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);

		job.m_barriers.Transition(m_r_island_states.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
		job.m_barriers.Transition(m_r_transaction_acceptance.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
		job.m_barriers.Transition(m_r_transaction_rows.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
		job.m_barriers.Transition(m_r_transaction_results.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
		job.m_barriers.Commit();

		auto body_readback = ReadbackAlloc{};
		if (!bodies.empty())
			body_readback = job.m_readback.Alloc<GpuRigidBody>(isize(bodies));
		auto row_readback = job.m_readback.Alloc<GpuConstraintRow>(GpuConstraintRowsPerBlock * m_slot_count);
		auto velocity_readback = ReadbackAlloc{};
		if (aba.m_velocity_count != 0)
			velocity_readback = job.m_readback.Alloc<float>(aba.m_velocity_count);
		auto acceleration_readback = ReadbackAlloc{};
		if (aba.m_acceleration_count != 0)
			acceleration_readback = job.m_readback.Alloc<float>(aba.m_acceleration_count);
		auto scratch_readback = job.m_readback.Alloc<GpuArticulationAbaScratch>(aba.m_link_count);
		auto dof_readback = ReadbackAlloc{};
		if (aba.m_dof_count != 0)
			dof_readback = job.m_readback.Alloc<GpuArticulationAbaDofScratch>(aba.m_dof_count);

		auto island_readback = job.m_readback.Alloc<GpuCoupledConstraintIslandState>(m_island_count);
		auto acceptance_readback = job.m_readback.Alloc<GpuConstraintAcceptance>(m_island_count);
		auto accepted_rows_readback = job.m_readback.Alloc<GpuConstraintAcceptedRows>(m_slot_count);
		auto results_readback = job.m_readback.Alloc<uint32_t>(m_island_count);
		if (!bodies.empty())
			job.m_cmd_list.CopyBufferRegion(body_readback, r_bodies.get(), 0);
		job.m_cmd_list.CopyBufferRegion(row_readback, constraints.m_r_rows.get(), 0);
		if (aba.m_velocity_count != 0)
			job.m_cmd_list.CopyBufferRegion(velocity_readback, aba.m_r_velocities.get(), 0);
		if (aba.m_acceleration_count != 0)
			job.m_cmd_list.CopyBufferRegion(acceleration_readback, aba.m_r_accelerations.get(), 0);
		job.m_cmd_list.CopyBufferRegion(scratch_readback, aba.m_r_scratch.get(), 0);
		if (aba.m_dof_count != 0)
			job.m_cmd_list.CopyBufferRegion(dof_readback, aba.m_r_dof_scratch.get(), 0);

		job.m_cmd_list.CopyBufferRegion(island_readback, m_r_island_states.get(), 0);
		job.m_cmd_list.CopyBufferRegion(acceptance_readback, m_r_transaction_acceptance.get(), 0);
		job.m_cmd_list.CopyBufferRegion(accepted_rows_readback, m_r_transaction_rows.get(), 0);
		job.m_cmd_list.CopyBufferRegion(results_readback, m_r_transaction_results.get(), 0);
		job.Run();

		if (!bodies.empty())
			memcpy(result.m_bodies.data(), body_readback.ptr<GpuRigidBody>(), result.m_bodies.size() * sizeof(result.m_bodies[0]));
		memcpy(result.m_rows.data(), row_readback.ptr<GpuConstraintRow>(), result.m_rows.size() * sizeof(result.m_rows[0]));
		if (aba.m_velocity_count != 0)
			memcpy(result.m_articulation_velocities.data(), velocity_readback.ptr<float>(), result.m_articulation_velocities.size() * sizeof(result.m_articulation_velocities[0]));
		if (aba.m_acceleration_count != 0)
			memcpy(result.m_articulation_accelerations.data(), acceleration_readback.ptr<float>(), result.m_articulation_accelerations.size() * sizeof(result.m_articulation_accelerations[0]));
		memcpy(result.m_articulation_scratch.data(), scratch_readback.ptr<GpuArticulationAbaScratch>(), result.m_articulation_scratch.size() * sizeof(result.m_articulation_scratch[0]));
		if (aba.m_dof_count != 0)
			std::copy_n(dof_readback.ptr<GpuArticulationAbaDofScratch>(), result.m_articulation_dof_scratch.size(), result.m_articulation_dof_scratch.data());

		memcpy(result.m_island_states.data(), island_readback.ptr<GpuCoupledConstraintIslandState>(), result.m_island_states.size() * sizeof(result.m_island_states[0]));
		std::copy_n(acceptance_readback.ptr<GpuConstraintAcceptance>(), result.m_acceptance.size(), result.m_acceptance.data());
		std::copy_n(accepted_rows_readback.ptr<GpuConstraintAcceptedRows>(), result.m_accepted_rows.size(), result.m_accepted_rows.data());
		std::copy_n(results_readback.ptr<uint32_t>(), result.m_results.size(), result.m_results.data());
		return result;
	}

	// Return current logical usage, retained capacities, and most recent coupled dispatch count.
	GpuCoupledConstraintVelocityStats GpuCoupledConstraintVelocity::Stats() const
	{
		// Include actual allocated accounting storage even if allocation stopped before the set became usable.
		auto result = m_stats;
		for (auto const& resource : {m_r_transaction_acceptance, m_r_transaction_rows, m_r_rigid_response, m_r_velocity_response,
			m_r_transaction_results, m_r_work_bodies, m_r_work_velocities, m_r_work_aba_scratch,
			m_r_reference_momenta, m_r_reference_velocities, m_r_initial_coefficients, m_r_work_spans,
			m_r_work_indices, m_r_reference_choices, m_r_work_motion})
		{
			// Resource widths include binding sentinels and do not depend on advertised capacities.
			if (resource != nullptr)
				result.m_allocated_feature_bytes += resource->GetDesc().Width;
		}
		if (m_transaction_ready && m_transaction_membership_revision == m_membership_revision)
		{
			// Response payload, cache/provenance, result transport, and private working state are distinct costs.
			result.m_logical_bytes += static_cast<size_t>(m_slot_count) * sizeof(GpuConstraintAcceptedRows) +
				static_cast<size_t>(m_island_count) * (sizeof(GpuConstraintAcceptance) + sizeof(uint32_t)) +
				static_cast<size_t>(m_transaction_body_count) * (sizeof(GpuArticulationSpatialVector) + sizeof(GpuRigidBody)) +
				static_cast<size_t>(m_transaction_velocity_count) * 2 * sizeof(float) +
				static_cast<size_t>(m_transaction_link_count) * sizeof(GpuArticulationAbaScratch);
			result.m_logical_bytes += static_cast<size_t>(m_rigid_work_count) * sizeof(GpuCoupledConstraintMomentumReference) +
				static_cast<size_t>(m_impulse_aba.m_velocity_delta_count) * sizeof(GpuCoupledConstraintSpeedReference) +
				static_cast<size_t>(m_slot_count) * GpuConstraintRowsPerBlock * sizeof(float) +
				static_cast<size_t>(m_island_count) * (sizeof(GpuCoupledConstraintWorkSpan) + sizeof(uint32_t)) +
				m_work_indices.size() * sizeof(uint32_t) +
				static_cast<size_t>(m_mobility_count) * sizeof(GpuCoupledConstraintWorkMotion);
		}
		return result;
	}

	// Read acceptance and row ownership without exposing private work buffers to the break detector.
	GpuCoupledConstraintLoadOutput GpuCoupledConstraintVelocity::LoadOutput()
	{
		if (!m_transaction_ready)
			throw std::logic_error("Coupled load output requires prepared ownership");

		return GpuCoupledConstraintLoadOutput{
			.m_acceptance = m_r_transaction_acceptance.get(),
			.m_rows = m_r_transaction_rows.get(),
			.m_topology = m_r_block_topology.get(),
			.m_island_count = m_island_count,
		};
	}

	// Release every optional coupled velocity resource when no coupled work remains.
	void GpuCoupledConstraintVelocity::ReleaseBuffers()
	{
		m_sweep_profiles.clear();
		m_r_block_topology = nullptr;
		m_r_targets = nullptr;
		m_r_adjacency = nullptr;
		m_r_articulation_islands = nullptr;
		m_r_islands = nullptr;
		m_r_island_blocks = nullptr;
		m_r_scratch = nullptr;
		m_r_contributions = nullptr;
		m_r_target_impulses = nullptr;
		m_r_island_states = nullptr;
		m_r_island_failures = nullptr;
		m_r_tree_selection = nullptr;
		m_r_tree_results = nullptr;
		m_r_transaction_acceptance = nullptr;
		m_r_transaction_rows = nullptr;
		m_r_rigid_response = nullptr;
		m_r_velocity_response = nullptr;
		m_r_transaction_results = nullptr;
		m_r_work_bodies = nullptr;
		m_r_work_velocities = nullptr;
		m_r_work_aba_scratch = nullptr;
		m_r_reference_momenta = nullptr;
		m_r_reference_velocities = nullptr;
		m_r_initial_coefficients = nullptr;
		m_r_work_spans = nullptr;
		m_r_work_indices = nullptr;
		m_r_reference_choices = nullptr;
		m_r_work_motion = nullptr;
		m_work_spans.clear();
		m_work_indices.clear();
		m_rigid_work_count = 0;
		m_transaction_ready = false;
		m_membership_source = nullptr;
		m_membership_key.clear();
		m_membership_identities.clear();
		m_source = nullptr;
		m_slot_count = 0;
		m_target_count = 0;
		m_adjacency_count = 0;
		m_island_count = 0;
		m_island_block_count = 0;
		m_articulation_range_count = 0;
		m_mobility_count = 0;
		m_stats = {};
	}

	// Create or geometrically grow topology and detached transaction buffers.
	void GpuCoupledConstraintVelocity::ResizeBuffers(GpuJob& job)
	{
		// Resources sharing one logical index grow together so one capacity describes their retained cost exactly.
		if (m_slot_count > m_stats.m_slot_capacity)
		{
			// A partial set is never advertised as usable; each returned replacement remains a member during recording retirement.
			auto const capacity = GrowCapacity(m_slot_count, m_stats.m_slot_capacity);
			m_stats.m_slot_capacity = 0;
			for (auto resource : {m_r_block_topology, m_r_scratch, m_r_contributions})
			{
				// Retain all old resources before replacing the first one.
				if (resource != nullptr)
					job.m_keep_alive.Add(resource);
			}
			m_r_block_topology = m_gpu.CreateResource(ResDesc::Buf<GpuCoupledConstraintBlockTopology>(capacity, {}).usage(EUsage::Default), job, "Physics:CoupledVelocityBlockTopology");
			m_r_scratch = m_gpu.CreateResource(ResDesc::Buf<GpuCoupledConstraintSolveScratch>(capacity, {}).usage(EUsage::UnorderedAccess), job, "Physics:CoupledVelocityScratch");
			m_r_contributions = m_gpu.CreateResource(ResDesc::Buf<GpuArticulationSpatialVector>(2 * capacity, {}).usage(EUsage::UnorderedAccess), job, "Physics:CoupledVelocityContributions");
			m_stats.m_slot_capacity = capacity;
		}

		if (m_target_count > m_stats.m_target_capacity)
		{
			// Target topology and trial impulses share one complete allocation set.
			auto const capacity = GrowCapacity(m_target_count, m_stats.m_target_capacity);
			m_stats.m_target_capacity = 0;
			for (auto resource : {m_r_targets, m_r_target_impulses})
			{
				// Preserve any earlier recorded gather or publication.
				if (resource != nullptr)
					job.m_keep_alive.Add(resource);
			}
			m_r_targets = m_gpu.CreateResource(ResDesc::Buf<GpuCoupledConstraintTarget>(capacity, {}).usage(EUsage::Default), job, "Physics:CoupledVelocityTargets");
			m_r_target_impulses = m_gpu.CreateResource(ResDesc::Buf<GpuArticulationSpatialVector>(capacity, {}).usage(EUsage::UnorderedAccess), job, "Physics:CoupledVelocityTargetImpulses");
			m_stats.m_target_capacity = capacity;
		}

		EnsureCoupledVelocityBuffer<uint32_t>(m_gpu, job, m_r_adjacency, m_adjacency_count, m_stats.m_adjacency_capacity, EUsage::Default, "Physics:CoupledVelocityAdjacency");
		if (m_island_count > m_stats.m_island_capacity)
		{
			// Upload initializes immutable topology and frame failures on every successful call.
			auto const capacity = GrowCapacity(m_island_count, m_stats.m_island_capacity);
			m_stats.m_island_capacity = 0;
			for (auto resource : {m_r_islands, m_r_island_states, m_r_island_failures})
			{
				// Trial state is initialized by Begin, never recovered from an incomplete allocation.
				if (resource != nullptr)
					job.m_keep_alive.Add(resource);
			}
			m_r_islands = m_gpu.CreateResource(ResDesc::Buf<GpuCoupledConstraintIsland>(capacity, {}).usage(EUsage::Default), job, "Physics:CoupledVelocityIslands");
			m_r_island_states = m_gpu.CreateResource(ResDesc::Buf<GpuCoupledConstraintIslandState>(capacity, {}).usage(EUsage::UnorderedAccess), job, "Physics:CoupledVelocityIslandStates");
			m_r_island_failures = m_gpu.CreateResource(ResDesc::Buf<uint32_t>(capacity, {}).usage(EUsage::UnorderedAccess), job, "Physics:CoupledVelocityIslandFailures");
			m_stats.m_island_capacity = capacity;
		}
		EnsureCoupledVelocityBuffer<uint32_t>(m_gpu, job, m_r_island_blocks, m_island_block_count, m_stats.m_island_block_capacity, EUsage::Default, "Physics:CoupledVelocityIslandBlocks");

		if (m_articulation_range_count > m_stats.m_articulation_range_capacity)
		{
			// Selection and results use the same compact participation index as the topology mapping.
			auto const capacity = GrowCapacity(m_articulation_range_count, m_stats.m_articulation_range_capacity);
			m_stats.m_articulation_range_capacity = 0;
			for (auto resource : {m_r_articulation_islands, m_r_tree_selection, m_r_tree_results})
			{
				// Retire previous range users after their recorded commands complete.
				if (resource != nullptr)
					job.m_keep_alive.Add(resource);
			}
			m_r_articulation_islands = m_gpu.CreateResource(ResDesc::Buf<int>(capacity, {}).usage(EUsage::Default), job, "Physics:CoupledVelocityArticulationIslands");
			m_r_tree_selection = m_gpu.CreateResource(ResDesc::Buf<uint32_t>(capacity, {}).usage(EUsage::UnorderedAccess), job, "Physics:CoupledVelocityTreeSelection");
			m_r_tree_results = m_gpu.CreateResource(ResDesc::Buf<uint32_t>(capacity, {}).usage(EUsage::UnorderedAccess), job, "Physics:CoupledVelocityTreeResults");
			m_stats.m_articulation_range_capacity = capacity;
		}
	}

	// Bind one coupled phase against the complete stable resource layout.
	void GpuCoupledConstraintVelocity::Dispatch(
		GpuJob& job,
		ComputeStep& step,
		int phase,
		int attempt_index,
		int item_count,
		int body_count,
		ID3D12Resource* bodies,
		GpuArticulationVelocityBuffers state,
		int substep_index)
	{
		if (item_count <= 0)
			return;

		// A named pass includes its dependency barriers; PIX replay durations are not exclusive shader time.
		auto profile_scope = pix::DetailScope(job.m_cmd_list.get(), pix::DetailEnabled() ? DebugName(step.m_pso) : "");
		pix::DetailMarker(job.m_cmd_list.get(), "CoupledVelocity phase=%d attempt=%d substep=%d", phase, attempt_index, substep_index);
		PrepareResources(job, bodies, state);
		auto const constants = cbCoupledConstraintVelocity{
			.m_slot_count = m_slot_count,
			.m_body_count = body_count,
			.m_target_count = m_target_count,
			.m_adjacency_count = m_adjacency_count,
			.m_island_count = m_island_count,
			.m_articulation_range_count = m_articulation_range_count,
			.m_mobility_count = m_mobility_count,
			.m_work_count = std::max({m_slot_count, m_target_count, m_island_count, m_articulation_range_count, m_mobility_count}),
			.m_island_block_count = m_island_block_count,
			.m_phase = phase,
			.m_attempt_index = attempt_index,
			.m_backtrack_limit = m_config.constraint_coupled_backtrack_limit,
			.m_relaxation = m_config.constraint_coupled_relaxation,
			.m_substep_index = substep_index,
		};
		auto& constraints = m_prepare.m_constraints;
		job.m_cmd_list.SetPipelineState(step.m_pso.get());
		job.m_cmd_list.SetComputeRootSignature(step.m_sig.get());
		job.m_cmd_list.AddComputeRoot32BitConstants(constants);
		job.m_cmd_list.AddComputeRootUnorderedAccessView(bodies);
		job.m_cmd_list.AddComputeRootShaderResourceView(constraints.m_r_endpoints);
		job.m_cmd_list.AddComputeRootShaderResourceView(m_prepare.m_r_coupled_endpoints);
		job.m_cmd_list.AddComputeRootShaderResourceView(m_r_block_topology);
		job.m_cmd_list.AddComputeRootShaderResourceView(m_r_targets);
		job.m_cmd_list.AddComputeRootShaderResourceView(m_r_adjacency);
		job.m_cmd_list.AddComputeRootShaderResourceView(m_prepare.m_r_preconditioners);
		job.m_cmd_list.AddComputeRootShaderResourceView(state.m_scratch);
		job.m_cmd_list.AddComputeRootShaderResourceView(m_r_articulation_islands);
		job.m_cmd_list.AddComputeRootShaderResourceView(m_r_islands);
		job.m_cmd_list.AddComputeRootShaderResourceView(m_r_island_blocks);
		job.m_cmd_list.AddComputeRootUnorderedAccessView(constraints.m_r_blocks);
		job.m_cmd_list.AddComputeRootUnorderedAccessView(constraints.m_r_rows);
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_scratch);
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_contributions);
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_target_impulses);
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_island_states);
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_impulse_aba.LinkImpulses());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_tree_selection);
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_tree_results);
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_island_failures);
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_impulse_aba.Work());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(constraints.m_r_frame_rejections);
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_transaction_results != nullptr ? m_r_transaction_results.get() : constraints.m_r_state.get());
		job.m_cmd_list.Dispatch(CoupledVelocityThreadGroupCount(item_count), 1, 1);
		++m_stats.m_dispatch_count;
		CommitUavBarriers(job, bodies);
	}

	// Bind the fixed accounting layout against separate live and private primary state.
	void GpuCoupledConstraintVelocity::DispatchTransaction(GpuJob& job, ComputeStep& step, int body_count, ID3D12Resource* bodies)
	{
		// Retain the stable pipeline name for pass-level replay attribution.
		auto profile_scope = pix::DetailScope(job.m_cmd_list.get(), pix::DetailEnabled() ? DebugName(step.m_pso) : "");

		// Register order is shared by all accounting kernels; live and private state never share a resource.
		auto& constraints = m_prepare.m_constraints;
		auto& mobility = m_impulse_aba.m_mobility;
		auto& aba = m_impulse_aba.m_aba;
		auto live = m_impulse_aba.VelocityBuffers();
		auto const srvs = std::array{
			m_r_targets.get(), m_r_block_topology.get(), constraints.m_r_endpoints.get(), m_prepare.m_r_coupled_endpoints.get(),
			constraints.m_r_blocks.get(), mobility.m_r_ranges.get(), aba.m_r_articulations.get(), m_r_articulation_islands.get()};
		auto const uavs = std::array{
			m_r_work_bodies.get(), m_r_work_velocities.get(), m_r_work_aba_scratch.get(), constraints.m_r_rows.get(),
			m_r_transaction_rows.get(), m_r_transaction_acceptance.get(), m_r_rigid_response.get(), m_r_velocity_response.get(),
			m_r_transaction_results.get(), m_r_island_states.get(), m_r_tree_selection.get(), m_r_tree_results.get(),
			m_r_target_impulses.get(), (m_impulse_aba.m_velocity_delta_count != 0 ? m_impulse_aba.m_r_velocity_deltas : aba.m_r_uav_sentinel).get(),
			bodies, live.m_velocities, live.m_scratch, constraints.m_r_state.get()};
		for (auto resource : srvs)
		{
			// Numerical and accounting phases have different read/write views of the compiled block stream.
			job.m_barriers.Transition(resource, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		}
		for (auto resource : uavs)
		{
			// Publication is the only accounting phase that writes the live state.
			job.m_barriers.Transition(resource, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		}
		job.m_barriers.Commit();

		// Bounds are the actual packed stream counts, not retained capacities.
		auto const constants = cbCoupledConstraintTransaction{
			.m_slot_count = m_slot_count, .m_body_count = body_count, .m_target_count = m_target_count, .m_island_count = m_island_count,
			.m_range_count = m_articulation_range_count, .m_articulation_count = aba.m_articulation_count,
			.m_velocity_count = aba.m_velocity_count, .m_link_count = aba.m_link_count, .m_epoch = m_transaction_epoch};
		job.m_cmd_list.SetPipelineState(step.m_pso.get());
		job.m_cmd_list.SetComputeRootSignature(step.m_sig.get());
		job.m_cmd_list.AddComputeRoot32BitConstants(constants);
		for (auto resource : srvs)
		{
			// Root descriptors use this fixed declaration order.
			job.m_cmd_list.AddComputeRootShaderResourceView(resource);
		}
		for (auto resource : uavs)
		{
			// Working and accepted response resources remain separately bound.
			job.m_cmd_list.AddComputeRootUnorderedAccessView(resource);
		}
		job.m_cmd_list.Dispatch(CoupledVelocityThreadGroupCount(std::max({m_slot_count, m_target_count, m_island_count, m_articulation_range_count})), 1, 1);
		++m_stats.m_dispatch_count;

		// This barrier is the complete publication boundary; no later consumer observes a partially updated stamp or payload.
		for (auto resource : uavs)
		{
			// Read-only phases also order the same layout, avoiding phase-dependent lifetime assumptions.
			job.m_barriers.UAV(resource);
		}
		job.m_barriers.Commit();
	}

	// Bind one compact reference or complete-primary work phase without another working articulation forest.
	void GpuCoupledConstraintVelocity::DispatchTrial(GpuJob& job, ComputeStep& step, int body_count, ID3D12Resource* bodies, int phase, int attempt_index)
	{
		// This records every configured dispatch, including attempts whose GPU Pending guard performs no work.
		auto profile_scope = pix::DetailScope(job.m_cmd_list.get(), pix::DetailEnabled() ? DebugName(step.m_pso) : "");
		pix::DetailMarker(job.m_cmd_list.get(), "CoupledTrial phase=%d attempt=%d", phase, attempt_index);
		auto& constraints = m_prepare.m_constraints;
		auto& aba = m_impulse_aba.m_aba;
		auto live = m_impulse_aba.VelocityBuffers();
		auto const srvs = std::array{
			m_r_targets.get(), m_r_block_topology.get(), m_prepare.m_r_coupled_endpoints.get(),
			m_r_work_spans.get(), m_r_work_indices.get(), m_r_islands.get(), m_r_island_blocks.get(),
			m_impulse_aba.m_mobility.m_r_ranges.get(), aba.m_r_articulations.get(), aba.m_r_links.get(),
			m_r_transaction_rows.get(), m_r_transaction_acceptance.get(), bodies, live.m_velocities,
			(aba.m_dof_count != 0 ? aba.m_r_dof_scratch : aba.m_r_srv_sentinel).get(), constraints.m_r_blocks.get()};
		auto const uavs = std::array{
			m_r_work_bodies.get(), m_r_work_velocities.get(), m_r_work_aba_scratch.get(), constraints.m_r_rows.get(),
			m_r_reference_momenta.get(), m_r_reference_velocities.get(), m_r_initial_coefficients.get(), m_r_reference_choices.get(),
			m_r_island_states.get(), m_r_island_failures.get(), m_r_transaction_results.get(), m_r_work_motion.get()};
		for (auto resource : srvs)
			job.m_barriers.Transition(resource, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);

		for (auto resource : uavs)
			job.m_barriers.Transition(resource, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);

		job.m_barriers.Commit();
		auto const constants = cbCoupledConstraintTrial{
			.m_slot_count = m_slot_count, .m_body_count = body_count, .m_island_count = m_island_count, .m_range_count = m_articulation_range_count,
			.m_rigid_count = m_rigid_work_count, .m_velocity_count = m_impulse_aba.m_velocity_delta_count,
			.m_link_count = aba.m_link_count, .m_mobility_count = m_mobility_count,
			.m_island_block_count = m_island_block_count, .m_attempt_index = attempt_index,
			.m_backtrack_limit = m_config.constraint_coupled_backtrack_limit, .m_reference_phase = phase, .m_epoch = m_transaction_epoch};
		job.m_cmd_list.SetPipelineState(step.m_pso.get());
		job.m_cmd_list.SetComputeRootSignature(step.m_sig.get());
		job.m_cmd_list.AddComputeRootConstantBufferView(job.m_upload.Add(constants, D3D12_CONSTANT_BUFFER_DATA_PLACEMENT_ALIGNMENT, false));
		for (auto resource : srvs)
			job.m_cmd_list.AddComputeRootShaderResourceView(resource);

		for (auto resource : uavs)
			job.m_cmd_list.AddComputeRootUnorderedAccessView(resource);

		job.m_cmd_list.Dispatch(CoupledVelocityThreadGroupCount(m_island_count), 1, 1);
		++m_stats.m_dispatch_count;
		for (auto resource : uavs)
			job.m_barriers.UAV(resource);

		job.m_barriers.Commit();
	}

	// Transition the common coupled resources into the states required by a coupled shader phase.
	void GpuCoupledConstraintVelocity::PrepareResources(GpuJob& job, ID3D12Resource* bodies, GpuArticulationVelocityBuffers state)
	{
		auto& constraints = m_prepare.m_constraints;
		job.m_barriers.Transition(bodies, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(constraints.m_r_endpoints.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(m_prepare.m_r_coupled_endpoints.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(m_r_block_topology.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(m_r_targets.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(m_r_adjacency.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(m_prepare.m_r_preconditioners.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(state.m_scratch, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(m_r_articulation_islands.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(m_r_islands.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(m_r_island_blocks.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(constraints.m_r_blocks.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(constraints.m_r_rows.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_scratch.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_contributions.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_target_impulses.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_island_states.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_impulse_aba.LinkImpulses(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_tree_selection.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_tree_results.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_island_failures.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_impulse_aba.Work(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_prepare.m_constraints.m_r_frame_rejections.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		if (m_r_transaction_results != nullptr)
			job.m_barriers.Transition(m_r_transaction_results.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Commit();
	}

	// Order every mutable coupled resource before its next dependent phase.
	void GpuCoupledConstraintVelocity::CommitUavBarriers(GpuJob& job, ID3D12Resource* bodies)
	{
		auto& constraints = m_prepare.m_constraints;
		job.m_barriers.UAV(bodies);
		job.m_barriers.UAV(constraints.m_r_blocks.get());
		job.m_barriers.UAV(constraints.m_r_rows.get());
		job.m_barriers.UAV(m_r_scratch.get());
		job.m_barriers.UAV(m_r_contributions.get());
		job.m_barriers.UAV(m_r_target_impulses.get());
		job.m_barriers.UAV(m_r_island_states.get());
		job.m_barriers.UAV(m_impulse_aba.LinkImpulses());
		job.m_barriers.UAV(m_r_tree_selection.get());
		job.m_barriers.UAV(m_r_tree_results.get());
		job.m_barriers.UAV(m_r_island_failures.get());
		job.m_barriers.UAV(m_impulse_aba.Work());
		job.m_barriers.UAV(m_prepare.m_constraints.m_r_frame_rejections.get());
		job.m_barriers.Commit();
	}
}
