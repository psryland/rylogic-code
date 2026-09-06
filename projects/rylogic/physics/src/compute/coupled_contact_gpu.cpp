//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "pr/common/cast.h"
#include "src/compute/coupled_contact_gpu.h"
#include "src/compute/shader_code.h"

namespace pr::physics
{
	using namespace ::pr::compute;

	namespace
	{
		// Root-register assignments shared with coupled_contact.hlsl.
		struct EReg
		{
			inline static constexpr auto Params = ECBufReg::b0;
			inline static constexpr auto Counters = ESRVReg::t0;
			inline static constexpr auto Materials = ESRVReg::t1;
			inline static constexpr auto Links = ESRVReg::t2;
			inline static constexpr auto LinkToWorld = ESRVReg::t3;
			inline static constexpr auto Mobilities = ESRVReg::t4;
			inline static constexpr auto AbaScratch = ESRVReg::t5;
			inline static constexpr auto Ranges = ESRVReg::t6;
			inline static constexpr auto Bodies = EUAVReg::u0;
			inline static constexpr auto Contacts = EUAVReg::u1;
			inline static constexpr auto Blocks = EUAVReg::u2;
			inline static constexpr auto Scratch = EUAVReg::u3;
			inline static constexpr auto Contributions = EUAVReg::u4;
			inline static constexpr auto EndpointKeys = EUAVReg::u5;
			inline static constexpr auto EndpointOrder = EUAVReg::u6;
			inline static constexpr auto TargetImpulses = EUAVReg::u7;
			inline static constexpr auto ParticipantDegrees = EUAVReg::u8;
			inline static constexpr auto LinkImpulses = EUAVReg::u9;
			inline static constexpr auto TreeSelection = EUAVReg::u10;
			inline static constexpr auto TreeResults = EUAVReg::u11;
			inline static constexpr auto State = EUAVReg::u12;
			inline static constexpr auto RigidPseudo = EUAVReg::u13;
			inline static constexpr auto LinkPseudo = EUAVReg::u14;
			inline static constexpr auto GeneralizedPseudo = EUAVReg::u15;
			inline static constexpr auto ArticulationWork = EUAVReg::u16;
			inline static constexpr auto VelocityDeltas = EUAVReg::u17;
			inline static constexpr auto Articulations = EUAVReg::u18;
			inline static constexpr auto Positions = EUAVReg::u19;
		};

		// Match the HLSL bounds and solve-control constant buffer exactly.
		struct alignas(16) cbCoupledContact
		{
			int m_max_contacts;
			int m_body_count;
			int m_rigid_body_count;
			int m_articulation_count;
			int m_link_count;
			int m_participant_count;
			int m_target_count;
			int m_mobility_count;
			int m_articulation_range_count;
			int m_work_count;
			int m_phase;
			int m_velocity_delta_count;
			float m_relaxation;
			float m_restitution_scale;
			float m_dt;
			float m_position_slop;
			float m_position_beta;
			float m_max_position_speed;
			int m_position_iteration_index;
			float m_pad1;
		};
		static_assert(sizeof(cbCoupledContact) == 80);

		enum class ECoupledContactPhase
		{
			WarmStart = 0,
			Velocity = 1,
			Position = 2,
		};

		// Create or retain one typed UAV resource using geometric high-water growth.
		template <typename Type>
		void EnsureCoupledContactBuffer(Gpu& gpu, CmdList& cmd_list, D3DPtr<ID3D12Resource>& resource, int required_count, int& capacity, char const* name)
		{
			if (required_count <= capacity)
				return;

			auto const doubled_capacity = capacity <= std::numeric_limits<int>::max() / 2 ? 2 * capacity : std::numeric_limits<int>::max();
			capacity = std::max(required_count, std::max(1, doubled_capacity));
			resource = gpu.CreateResource(ResDesc::Buf<Type>(capacity, {}).usage(EUsage::UnorderedAccess), cmd_list, name);
		}

		// Return the exact dispatch width for a non-empty contact work range.
		int CoupledContactThreadGroupCount(int item_count)
		{
			return (item_count + ConstraintThreadCount - 1) / ConstraintThreadCount;
		}
	}

	// Create fixed pipeline state while leaving every contact-dependent resource unallocated.
	GpuCoupledContactSolver::GpuCoupledContactSolver(Gpu& gpu, GpuArticulationForceAba& aba, GpuArticulationLinkProxies& proxies, EngineConfig const& config)
		: m_gpu(gpu)
		, m_config(config)
		, m_aba(aba)
		, m_proxies(proxies)
		, m_mobility(aba)
		, m_impulse_aba(gpu, aba, m_mobility)
		, m_endpoint_sorter(gpu.m_gpu)
		, m_cs_clear()
		, m_cs_prepare()
		, m_cs_prepare_position()
		, m_cs_begin()
		, m_cs_build_warm_start()
		, m_cs_build_candidates()
		, m_cs_build_position_candidates()
		, m_cs_gather()
		, m_cs_validate_trees()
		, m_cs_validate_position_trees()
		, m_cs_select_trees()
		, m_cs_commit()
		, m_cs_commit_position_articulations()
		, m_cs_apply_position()
		, m_r_blocks()
		, m_r_scratch()
		, m_r_contributions()
		, m_r_endpoint_keys()
		, m_r_endpoint_order()
		, m_r_target_impulses()
		, m_r_participant_degrees()
		, m_r_tree_selection()
		, m_r_tree_results()
		, m_r_state()
		, m_r_rigid_pseudo()
		, m_r_link_pseudo()
		, m_r_generalized_pseudo()
		, m_r_uav_sentinel()
		, m_r_counters()
		, m_r_contacts()
		, m_r_bodies()
		, m_r_materials()
		, m_max_contacts()
		, m_body_count()
		, m_rigid_body_count()
		, m_articulation_count()
		, m_link_count()
		, m_target_count()
		, m_participant_count()
		, m_mobility_count()
		, m_velocity_delta_count()
		, m_work_count()
		, m_dt()
		, m_restitution_scale()
		, m_active()
		, m_position_active()
		, m_stats()
	{
		// The common layout deliberately occupies exactly 64 DWORDs: 20 constants, six SRVs, and sixteen UAVs.
		auto common_sig = RootSig(ERootSigFlags::ComputeOnly)
			.U32<cbCoupledContact>(EReg::Params)
			.SRV(EReg::Counters)
			.SRV(EReg::Materials)
			.SRV(EReg::Links)
			.SRV(EReg::LinkToWorld)
			.SRV(EReg::Mobilities)
			.SRV(EReg::AbaScratch)
			.UAV(EReg::Bodies)
			.UAV(EReg::Contacts)
			.UAV(EReg::Blocks)
			.UAV(EReg::Scratch)
			.UAV(EReg::Contributions)
			.UAV(EReg::EndpointKeys)
			.UAV(EReg::EndpointOrder)
			.UAV(EReg::TargetImpulses)
			.UAV(EReg::ParticipantDegrees)
			.UAV(EReg::LinkImpulses)
			.UAV(EReg::TreeSelection)
			.UAV(EReg::TreeResults)
			.UAV(EReg::State)
			.UAV(EReg::RigidPseudo)
			.UAV(EReg::LinkPseudo)
			.UAV(EReg::GeneralizedPseudo)
			.Create(m_gpu, "Physics:CoupledContactSig");

		auto compile_common = [&](ComputeStep& step, shader_code::ByteCode const& bytecode, char const* name)
		{
			step.m_sig = common_sig;
			step.m_pso = ComputePSO(common_sig.get(), bytecode).Create(m_gpu, name);
		};
		compile_common(m_cs_clear, shader_code::clear_coupled_contacts, "Physics:ClearCoupledContactsPSO");
		compile_common(m_cs_prepare, shader_code::prepare_coupled_contacts, "Physics:PrepareCoupledContactsPSO");
		compile_common(m_cs_prepare_position, shader_code::prepare_coupled_contact_position, "Physics:PrepareCoupledContactPositionPSO");
		compile_common(m_cs_begin, shader_code::begin_coupled_contact_transaction, "Physics:BeginCoupledContactTransactionPSO");
		compile_common(m_cs_build_warm_start, shader_code::build_coupled_contact_warm_start, "Physics:BuildCoupledContactWarmStartPSO");
		compile_common(m_cs_build_candidates, shader_code::build_coupled_contact_candidates, "Physics:BuildCoupledContactCandidatesPSO");
		compile_common(m_cs_build_position_candidates, shader_code::build_coupled_contact_position_candidates, "Physics:BuildCoupledContactPositionCandidatesPSO");
		compile_common(m_cs_gather, shader_code::gather_coupled_contact_targets, "Physics:GatherCoupledContactTargetsPSO");
		compile_common(m_cs_validate_trees, shader_code::validate_coupled_contact_trees, "Physics:ValidateCoupledContactTreesPSO");
		compile_common(m_cs_select_trees, shader_code::select_coupled_contact_trees, "Physics:SelectCoupledContactTreesPSO");
		compile_common(m_cs_commit, shader_code::commit_coupled_contacts, "Physics:CommitCoupledContactsPSO");

		// Complete-tree position validation and commit bind only canonical ranges and detached response streams.
		auto articulation_sig = RootSig(ERootSigFlags::ComputeOnly)
			.U32<cbCoupledContact>(EReg::Params)
			.SRV(EReg::Ranges)
			.UAV(EReg::TreeSelection)
			.UAV(EReg::TreeResults)
			.UAV(EReg::State)
			.UAV(EReg::ArticulationWork)
			.UAV(EReg::VelocityDeltas)
			.UAV(EReg::LinkPseudo)
			.UAV(EReg::GeneralizedPseudo)
			.UAV(EReg::Articulations)
			.Create(m_gpu, "Physics:CoupledContactPositionArticulationSig");
		m_cs_validate_position_trees.m_sig = articulation_sig;
		m_cs_validate_position_trees.m_pso = ComputePSO(articulation_sig.get(), shader_code::validate_coupled_contact_position_trees).Create(m_gpu, "Physics:ValidateCoupledContactPositionTreesPSO");
		m_cs_commit_position_articulations.m_sig = articulation_sig;
		m_cs_commit_position_articulations.m_pso = ComputePSO(articulation_sig.get(), shader_code::commit_coupled_contact_position_articulations).Create(m_gpu, "Physics:CommitCoupledContactPositionArticulationsPSO");

		// Final coordinate application cannot bind physical articulation velocity or momentum response buffers.
		auto apply_sig = RootSig(ERootSigFlags::ComputeOnly)
			.U32<cbCoupledContact>(EReg::Params)
			.SRV(EReg::Links)
			.SRV(EReg::Ranges)
			.UAV(EReg::Bodies)
			.UAV(EReg::RigidPseudo)
			.UAV(EReg::GeneralizedPseudo)
			.UAV(EReg::Articulations)
			.UAV(EReg::Positions)
			.Create(m_gpu, "Physics:CoupledContactPositionApplySig");
		m_cs_apply_position.m_sig = apply_sig;
		m_cs_apply_position.m_pso = ComputePSO(apply_sig.get(), shader_code::apply_coupled_contact_position).Create(m_gpu, "Physics:ApplyCoupledContactPositionPSO");
	}

	// Select every packed articulation for exact whole-tree contact response in canonical forest order.
	bool GpuCoupledContactSolver::Upload(GpuJob& job, GpuArticulationUpload const& upload)
	{
		m_stats.m_dispatch_count = 0;
		m_impulse_aba.ResetDispatchCount();
		if (upload.m_articulations.empty())
		{
			Deactivate();
			return false;
		}

		// Canonical all-tree participation makes compact mobility indices identical to global packed link indices.
		auto articulation_indices = std::vector<int>(upload.m_articulations.size());
		std::iota(articulation_indices.begin(), articulation_indices.end(), 0);
		if (!m_mobility.Upload(job, upload, articulation_indices))
			throw std::logic_error("Coupled contact mobility rejected a non-empty articulation forest");

		auto const ranges = m_mobility.Ranges();
		for (int articulation_idx = 0; articulation_idx != isize(ranges); ++articulation_idx)
		{
			auto const& range = ranges[articulation_idx];
			auto const& articulation = upload.m_articulations[articulation_idx];
			if (
				range.articulation_index != articulation_idx ||
				range.mobility_offset != articulation.link_offset ||
				range.link_count != articulation.link_count)
				throw std::logic_error("Coupled contact mobility must preserve global packed-link addressing");
		}

		m_articulation_count = isize(upload.m_articulations);
		m_link_count = isize(upload.m_links);
		m_mobility_count = m_link_count;
		m_active = true;
		return true;
	}

	// Release retained optional resources when the frame contains no shaped articulation links.
	void GpuCoupledContactSolver::Deactivate()
	{
		// Internal exact-self and impulse-ABA buffers are part of the optional contact feature and must not survive deactivation.
		m_endpoint_sorter.ReleaseBuffers();
		m_impulse_aba.ReleaseBuffers();
		m_mobility.ReleaseBuffers();
		m_r_blocks = nullptr;
		m_r_scratch = nullptr;
		m_r_contributions = nullptr;
		m_r_endpoint_keys = nullptr;
		m_r_endpoint_order = nullptr;
		m_r_target_impulses = nullptr;
		m_r_participant_degrees = nullptr;
		m_r_tree_selection = nullptr;
		m_r_tree_results = nullptr;
		m_r_state = nullptr;
		m_r_rigid_pseudo = nullptr;
		m_r_link_pseudo = nullptr;
		m_r_generalized_pseudo = nullptr;
		m_r_uav_sentinel = nullptr;
		m_r_counters = nullptr;
		m_r_contacts = nullptr;
		m_r_bodies = nullptr;
		m_r_materials = nullptr;
		m_max_contacts = 0;
		m_body_count = 0;
		m_rigid_body_count = 0;
		m_articulation_count = 0;
		m_link_count = 0;
		m_target_count = 0;
		m_participant_count = 0;
		m_mobility_count = 0;
		m_velocity_delta_count = 0;
		m_work_count = 0;
		m_dt = 0.0f;
		m_restitution_scale = 0.0f;
		m_active = false;
		m_position_active = false;
		m_stats = {};
	}

	// Build exact-self contact blocks and deterministic transient endpoint topology at the integrated configuration.
	void GpuCoupledContactSolver::PrepareVelocity(
		GpuJob& job,
		float dt,
		int body_count,
		int rigid_body_count,
		int max_contacts,
		D3DPtr<ID3D12Resource> counters,
		D3DPtr<ID3D12Resource> contacts,
		D3DPtr<ID3D12Resource> bodies,
		D3DPtr<ID3D12Resource> materials,
		float restitution_scale)
	{
		if (!m_active)
			return;
		if (!(dt > 0.0f) || !std::isfinite(dt))
			throw std::invalid_argument("Coupled contact preparation requires a finite positive timestep");
		if (rigid_body_count < 0 || body_count < rigid_body_count || body_count - rigid_body_count != m_link_count)
			throw std::invalid_argument("Coupled contact body suffix must contain exactly one proxy per packed articulation link");
		if (max_contacts <= 0 || max_contacts > std::numeric_limits<int>::max() / 2)
			throw std::invalid_argument("Coupled contact capacity must be positive and leave room for two endpoint entries per contact");
		if (counters == nullptr || contacts == nullptr || bodies == nullptr || materials == nullptr)
			throw std::invalid_argument("Coupled contact preparation requires collision, body, and material resources");
		if (!(m_config.constraint_coupled_relaxation > 0.0f) || m_config.constraint_coupled_relaxation > 1.0f || !std::isfinite(m_config.constraint_coupled_relaxation))
			throw std::invalid_argument("Coupled contact relaxation must be finite and in (0,1]");

		m_body_count = body_count;
		m_rigid_body_count = rigid_body_count;
		m_max_contacts = max_contacts;
		m_target_count = rigid_body_count + m_link_count;
		m_participant_count = rigid_body_count + m_articulation_count;
		m_velocity_delta_count = m_mobility.m_velocity_delta_count;
		m_work_count = std::max({1, m_max_contacts, m_target_count, m_participant_count, m_mobility_count, m_velocity_delta_count, m_articulation_count});
		m_dt = dt;
		m_restitution_scale = restitution_scale;
		m_position_active = false;
		m_r_counters = std::move(counters);
		m_r_contacts = std::move(contacts);
		m_r_bodies = std::move(bodies);
		m_r_materials = std::move(materials);
		ResizeBuffers(job.m_cmd_list);

		// Rebuild fixed-configuration factors before contact blocks query exact self-link response.
		m_mobility.Run(job);
		if (!m_impulse_aba.Prepare(job))
			throw std::logic_error("Coupled contact preparation requires articulation impulse storage");

		DispatchCommon(job, m_cs_clear, static_cast<int>(ECoupledContactPhase::Velocity), m_work_count);
		DispatchCommon(job, m_cs_prepare, static_cast<int>(ECoupledContactPhase::Velocity), m_max_contacts);

		// Stable radix order gives every target a deterministic contiguous endpoint segment reused by all outer sweeps.
		m_endpoint_sorter.Bind(job.m_cmd_list, 2 * m_max_contacts, m_r_endpoint_keys, m_r_endpoint_order);
		m_endpoint_sorter.Sort(job.m_cmd_list);
		m_stats.m_dispatch_count += m_endpoint_sorter.SortDispatchCount();
		job.m_barriers.UAV(m_r_endpoint_keys.get());
		job.m_barriers.UAV(m_r_endpoint_order.get());
		job.m_barriers.Commit();

		UpdateMemoryStats();
	}

	// Apply cached normal impulses through rigid endpoints and complete articulation trees as one finite transaction.
	void GpuCoupledContactSolver::ApplyWarmStart(GpuJob& job)
	{
		if (!m_active || !(m_config.warm_start_scale > 0.0f))
			return;
		RunTransaction(job, m_cs_build_warm_start, static_cast<int>(ECoupledContactPhase::WarmStart), -1);
	}

	// Execute one degree-damped block-Jacobi velocity iteration through a complete-tree ABA response.
	void GpuCoupledContactSolver::SolveVelocityIteration(GpuJob& job)
	{
		if (!m_active)
			return;
		RunTransaction(job, m_cs_build_candidates, static_cast<int>(ECoupledContactPhase::Velocity), -1);
	}

	// Clear contact-owned pseudo state before detached penetration correction.
	bool GpuCoupledContactSolver::PreparePosition(GpuJob& job, int iteration_count)
	{
		if (!m_active || iteration_count <= 0)
			return false;
		if (!(m_config.position_slop >= 0.0f) || !std::isfinite(m_config.position_slop))
			throw std::invalid_argument("Coupled contact position slop must be finite and non-negative");
		if (!(m_config.position_baumgarte >= 0.0f) || !std::isfinite(m_config.position_baumgarte))
			throw std::invalid_argument("Coupled contact position Baumgarte factor must be finite and non-negative");
		if (!(m_config.constraint_max_position_speed >= 0.0f) || !std::isfinite(m_config.constraint_max_position_speed))
			throw std::invalid_argument("Coupled contact maximum position speed must be finite and non-negative");

		EnsureCoupledContactBuffer<GpuConstraintPseudoVelocity>(m_gpu, job.m_cmd_list, m_r_rigid_pseudo, std::max(1, m_rigid_body_count), m_stats.m_rigid_pseudo_capacity, "Physics:CoupledContactRigidPseudo");
		EnsureCoupledContactBuffer<GpuArticulationSpatialVector>(m_gpu, job.m_cmd_list, m_r_link_pseudo, std::max(1, m_mobility_count), m_stats.m_link_pseudo_capacity, "Physics:CoupledContactLinkPseudo");
		EnsureCoupledContactBuffer<float>(m_gpu, job.m_cmd_list, m_r_generalized_pseudo, std::max(1, m_velocity_delta_count), m_stats.m_generalized_pseudo_capacity, "Physics:CoupledContactGeneralizedPseudo");
		m_position_active = true;
		UpdateMemoryStats();
		DispatchCommon(job, m_cs_prepare_position, static_cast<int>(ECoupledContactPhase::Position), m_work_count);
		return true;
	}

	// Execute one zero-based monotone detached penetration-correction iteration with acceleration confined to the initial sweep.
	void GpuCoupledContactSolver::SolvePositionIteration(GpuJob& job, int position_iteration_index)
	{
		if (!m_active)
			return;
		if (position_iteration_index < 0)
			throw std::invalid_argument("Coupled contact position iteration index must be non-negative");

		RunTransaction(job, m_cs_build_position_candidates, static_cast<int>(ECoupledContactPhase::Position), position_iteration_index);
	}

	// Apply converged pseudo state once to rigid transforms and articulation coordinates.
	void GpuCoupledContactSolver::ApplyPosition(GpuJob& job)
	{
		if (!m_active)
			return;
		DispatchApplyPosition(job);
	}

	// Return aggregate logical usage, retained storage, and dispatch cost for the complete current frame.
	GpuCoupledContactStats GpuCoupledContactSolver::Stats() const
	{
		auto stats = m_stats;
		if (!m_active)
			return stats;

		// Include the whole-tree preparation, detached impulse response, and deterministic endpoint sort owned by this optional lane.
		auto const mobility = m_mobility.Stats();
		auto const impulse = m_impulse_aba.Stats();
		auto const sort_bytes = m_endpoint_sorter.AllocatedBufferBytes();
		stats.m_dispatch_count += mobility.m_dispatch_count + impulse.m_dispatch_count;
		stats.m_logical_bytes += mobility.m_logical_bytes + impulse.m_logical_buffer_bytes + sort_bytes;
		stats.m_allocated_feature_bytes += mobility.m_allocated_feature_bytes + impulse.m_allocated_feature_bytes + sort_bytes;
		return stats;
	}

	// Create or geometrically grow all contact, endpoint, target, participant, and tree transaction buffers.
	void GpuCoupledContactSolver::ResizeBuffers(CmdList& cmd_list)
	{
		// Contact-indexed streams grow as one unit so endpoint offsets remain exactly 2*contact_idx across every retained resource.
		if (m_max_contacts > m_stats.m_contact_capacity)
		{
			auto const doubled_capacity = m_stats.m_contact_capacity <= std::numeric_limits<int>::max() / 2
				? 2 * m_stats.m_contact_capacity
				: std::numeric_limits<int>::max();
			m_stats.m_contact_capacity = std::max(m_max_contacts, std::max(1, doubled_capacity));
			auto const endpoint_capacity = 2 * m_stats.m_contact_capacity;
			m_r_blocks = m_gpu.CreateResource(ResDesc::Buf<GpuCoupledContactBlock>(m_stats.m_contact_capacity, {}).usage(EUsage::UnorderedAccess), cmd_list, "Physics:CoupledContactBlocks");
			m_r_scratch = m_gpu.CreateResource(ResDesc::Buf<GpuCoupledContactScratch>(m_stats.m_contact_capacity, {}).usage(EUsage::UnorderedAccess), cmd_list, "Physics:CoupledContactScratch");
			m_r_contributions = m_gpu.CreateResource(ResDesc::Buf<GpuArticulationSpatialVector>(endpoint_capacity, {}).usage(EUsage::UnorderedAccess), cmd_list, "Physics:CoupledContactContributions");
			m_r_endpoint_keys = m_gpu.CreateResource(ResDesc::Buf<uint32_t>(endpoint_capacity, {}).usage(EUsage::UnorderedAccess), cmd_list, "Physics:CoupledContactEndpointKeys");
			m_r_endpoint_order = m_gpu.CreateResource(ResDesc::Buf<uint32_t>(endpoint_capacity, {}).usage(EUsage::UnorderedAccess), cmd_list, "Physics:CoupledContactEndpointOrder");
		}

		EnsureCoupledContactBuffer<GpuArticulationSpatialVector>(m_gpu, cmd_list, m_r_target_impulses, m_target_count, m_stats.m_target_capacity, "Physics:CoupledContactTargetImpulses");
		EnsureCoupledContactBuffer<uint32_t>(m_gpu, cmd_list, m_r_participant_degrees, m_participant_count, m_stats.m_participant_capacity, "Physics:CoupledContactParticipantDegrees");
		EnsureCoupledContactBuffer<uint32_t>(m_gpu, cmd_list, m_r_tree_selection, m_articulation_count, m_stats.m_tree_capacity, "Physics:CoupledContactTreeSelection");
		if (m_r_tree_results == nullptr || m_r_tree_results->GetDesc().Width < static_cast<UINT64>(m_stats.m_tree_capacity) * sizeof(uint32_t))
			m_r_tree_results = m_gpu.CreateResource(ResDesc::Buf<uint32_t>(m_stats.m_tree_capacity, {}).usage(EUsage::UnorderedAccess), cmd_list, "Physics:CoupledContactTreeResults");
		auto state_capacity = m_r_state != nullptr ? 1 : 0;
		auto sentinel_capacity = m_r_uav_sentinel != nullptr ? 1 : 0;
		EnsureCoupledContactBuffer<GpuCoupledContactState>(m_gpu, cmd_list, m_r_state, 1, state_capacity, "Physics:CoupledContactState");
		EnsureCoupledContactBuffer<uint32_t>(m_gpu, cmd_list, m_r_uav_sentinel, 1, sentinel_capacity, "Physics:CoupledContactUavSentinel");
		UpdateMemoryStats();
	}

	// Refresh logical and retained byte counts after topology or optional position storage changes.
	void GpuCoupledContactSolver::UpdateMemoryStats()
	{
		m_stats.m_logical_bytes =
			static_cast<size_t>(m_max_contacts) * (
				sizeof(GpuCoupledContactBlock) +
				sizeof(GpuCoupledContactScratch) +
				2 * sizeof(GpuArticulationSpatialVector) +
				4 * sizeof(uint32_t)) +
			static_cast<size_t>(m_target_count) * sizeof(GpuArticulationSpatialVector) +
			static_cast<size_t>(m_participant_count) * sizeof(uint32_t) +
			static_cast<size_t>(m_articulation_count) * 2 * sizeof(uint32_t) +
			sizeof(GpuCoupledContactState);
		if (m_position_active)
		{
			m_stats.m_logical_bytes +=
				static_cast<size_t>(m_rigid_body_count) * sizeof(GpuConstraintPseudoVelocity) +
				static_cast<size_t>(m_mobility_count) * sizeof(GpuArticulationSpatialVector) +
				static_cast<size_t>(m_velocity_delta_count) * sizeof(float);
		}
		m_stats.m_allocated_feature_bytes =
			static_cast<size_t>(m_stats.m_contact_capacity) * (
				sizeof(GpuCoupledContactBlock) +
				sizeof(GpuCoupledContactScratch) +
				2 * sizeof(GpuArticulationSpatialVector) +
				4 * sizeof(uint32_t)) +
			static_cast<size_t>(m_stats.m_target_capacity) * sizeof(GpuArticulationSpatialVector) +
			static_cast<size_t>(m_stats.m_participant_capacity) * sizeof(uint32_t) +
			static_cast<size_t>(m_stats.m_tree_capacity) * 2 * sizeof(uint32_t) +
			static_cast<size_t>(m_stats.m_rigid_pseudo_capacity) * sizeof(GpuConstraintPseudoVelocity) +
			static_cast<size_t>(m_stats.m_link_pseudo_capacity) * sizeof(GpuArticulationSpatialVector) +
			static_cast<size_t>(m_stats.m_generalized_pseudo_capacity) * sizeof(float) +
			sizeof(GpuCoupledContactState) +
			sizeof(uint32_t);
	}

	// Run one warm-start, velocity, or position candidate through deterministic gather and detached ABA evaluation.
	void GpuCoupledContactSolver::RunTransaction(GpuJob& job, ComputeStep& build_step, int phase, int position_iteration_index)
	{
		if (
			m_r_counters == nullptr ||
			m_r_contacts == nullptr ||
			m_r_bodies == nullptr ||
			m_r_materials == nullptr ||
			m_impulse_aba.LinkImpulses() == nullptr)
			throw std::logic_error("Coupled contact transaction requires prepared frame resources");

		DispatchCommon(job, m_cs_begin, phase, m_work_count);
		DispatchCommon(job, build_step, phase, m_max_contacts, position_iteration_index);
		DispatchCommon(job, m_cs_gather, phase, m_max_contacts);
		m_impulse_aba.Evaluate(job, m_r_tree_selection.get(), m_r_tree_results.get());
		switch (static_cast<ECoupledContactPhase>(phase))
		{
			case ECoupledContactPhase::WarmStart:
			case ECoupledContactPhase::Velocity:
			{
				DispatchCommon(job, m_cs_validate_trees, phase, m_articulation_count);
				DispatchCommon(job, m_cs_select_trees, phase, m_articulation_count);
				DispatchCommon(job, m_cs_commit, phase, m_work_count);
				m_impulse_aba.Commit(job, m_r_tree_selection.get(), m_r_tree_results.get());
				break;
			}
			case ECoupledContactPhase::Position:
			{
				DispatchPositionArticulations(job, m_cs_validate_position_trees, phase);
				DispatchCommon(job, m_cs_select_trees, phase, m_articulation_count);
				DispatchCommon(job, m_cs_commit, phase, m_work_count);
				DispatchPositionArticulations(job, m_cs_commit_position_articulations, phase);
				break;
			}
			default:
			{
				throw std::invalid_argument("Unknown coupled contact transaction phase");
			}
		}
	}

	// Bind and dispatch one common contact phase over a non-empty logical work range.
	void GpuCoupledContactSolver::DispatchCommon(GpuJob& job, ComputeStep& step, int phase, int item_count, int position_iteration_index)
	{
		if (item_count <= 0)
			return;

		PrepareCommonResources(job);
		auto* rigid_pseudo = m_position_active && m_r_rigid_pseudo != nullptr ? m_r_rigid_pseudo.get() : m_r_uav_sentinel.get();
		auto* link_pseudo = m_position_active && m_r_link_pseudo != nullptr ? m_r_link_pseudo.get() : m_r_uav_sentinel.get();
		auto* generalized_pseudo = m_position_active && m_r_generalized_pseudo != nullptr ? m_r_generalized_pseudo.get() : m_r_uav_sentinel.get();
		auto const constants = cbCoupledContact{
			.m_max_contacts = m_max_contacts,
			.m_body_count = m_body_count,
			.m_rigid_body_count = m_rigid_body_count,
			.m_articulation_count = m_articulation_count,
			.m_link_count = m_link_count,
			.m_participant_count = m_participant_count,
			.m_target_count = m_target_count,
			.m_mobility_count = m_mobility_count,
			.m_articulation_range_count = m_articulation_count,
			.m_work_count = m_work_count,
			.m_phase = phase,
			.m_velocity_delta_count = m_velocity_delta_count,
			.m_relaxation = m_config.constraint_coupled_relaxation,
			.m_restitution_scale = m_restitution_scale,
			.m_dt = m_dt,
			.m_position_slop = m_config.position_slop,
			.m_position_beta = m_config.position_baumgarte,
			.m_max_position_speed = m_config.constraint_max_position_speed,
			.m_position_iteration_index = position_iteration_index,
			.m_pad1 = 0.0f,
		};
		job.m_cmd_list.SetPipelineState(step.m_pso.get());
		job.m_cmd_list.SetComputeRootSignature(step.m_sig.get());
		job.m_cmd_list.AddComputeRoot32BitConstants(constants);
		job.m_cmd_list.AddComputeRootShaderResourceView(m_r_counters->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView(m_r_materials->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView(m_aba.m_r_links->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView(m_proxies.m_r_link_to_world->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView(m_mobility.m_r_mobilities->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView(m_aba.m_r_scratch->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_bodies->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_contacts->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_blocks->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_scratch->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_contributions->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_endpoint_keys->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_endpoint_order->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_target_impulses->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_participant_degrees->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_impulse_aba.m_r_link_impulses->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_tree_selection->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_tree_results->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_state->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(rigid_pseudo->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(link_pseudo->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(generalized_pseudo->GetGPUVirtualAddress());
		job.m_cmd_list.Dispatch(CoupledContactThreadGroupCount(item_count), 1, 1);
		++m_stats.m_dispatch_count;
		CommitUavBarriers(job);
	}

	// Bind and dispatch one complete-tree position validation or pseudo-state commit phase.
	void GpuCoupledContactSolver::DispatchPositionArticulations(GpuJob& job, ComputeStep& step, int phase)
	{
		if (m_articulation_count <= 0)
			return;

		auto* velocity_deltas = m_impulse_aba.m_r_velocity_deltas != nullptr
			? m_impulse_aba.m_r_velocity_deltas.get()
			: m_r_uav_sentinel.get();
		job.m_barriers.Transition(m_mobility.m_r_ranges.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(m_r_tree_selection.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_tree_results.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_state.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_impulse_aba.m_r_work.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(velocity_deltas, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_link_pseudo.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_generalized_pseudo.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_aba.m_r_articulations.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Commit();

		auto const constants = cbCoupledContact{
			.m_max_contacts = m_max_contacts,
			.m_body_count = m_body_count,
			.m_rigid_body_count = m_rigid_body_count,
			.m_articulation_count = m_articulation_count,
			.m_link_count = m_link_count,
			.m_participant_count = m_participant_count,
			.m_target_count = m_target_count,
			.m_mobility_count = m_mobility_count,
			.m_articulation_range_count = m_articulation_count,
			.m_work_count = m_work_count,
			.m_phase = phase,
			.m_velocity_delta_count = m_velocity_delta_count,
			.m_relaxation = m_config.constraint_coupled_relaxation,
			.m_restitution_scale = m_restitution_scale,
			.m_dt = m_dt,
			.m_position_slop = m_config.position_slop,
			.m_position_beta = m_config.position_baumgarte,
			.m_max_position_speed = m_config.constraint_max_position_speed,
			.m_position_iteration_index = -1,
			.m_pad1 = 0.0f,
		};
		job.m_cmd_list.SetPipelineState(step.m_pso.get());
		job.m_cmd_list.SetComputeRootSignature(step.m_sig.get());
		job.m_cmd_list.AddComputeRoot32BitConstants(constants);
		job.m_cmd_list.AddComputeRootShaderResourceView(m_mobility.m_r_ranges->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_tree_selection->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_tree_results->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_state->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_impulse_aba.m_r_work->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(velocity_deltas->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_link_pseudo->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_generalized_pseudo->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_aba.m_r_articulations->GetGPUVirtualAddress());
		job.m_cmd_list.Dispatch(CoupledContactThreadGroupCount(m_articulation_count), 1, 1);
		++m_stats.m_dispatch_count;

		job.m_barriers.UAV(m_r_tree_selection.get());
		job.m_barriers.UAV(m_r_tree_results.get());
		job.m_barriers.UAV(m_r_state.get());
		job.m_barriers.UAV(m_impulse_aba.m_r_work.get());
		job.m_barriers.UAV(velocity_deltas);
		job.m_barriers.UAV(m_r_link_pseudo.get());
		job.m_barriers.UAV(m_r_generalized_pseudo.get());
		job.m_barriers.UAV(m_aba.m_r_articulations.get());
		job.m_barriers.Commit();
	}

	// Bind and dispatch final rigid and articulation coordinate integration.
	void GpuCoupledContactSolver::DispatchApplyPosition(GpuJob& job)
	{
		auto* positions = m_aba.m_r_positions != nullptr
			? m_aba.m_r_positions.get()
			: m_r_uav_sentinel.get();
		job.m_barriers.Transition(m_aba.m_r_links.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(m_mobility.m_r_ranges.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(m_r_bodies.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_rigid_pseudo.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_generalized_pseudo.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_aba.m_r_articulations.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(positions, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Commit();

		auto const constants = cbCoupledContact{
			.m_max_contacts = m_max_contacts,
			.m_body_count = m_body_count,
			.m_rigid_body_count = m_rigid_body_count,
			.m_articulation_count = m_articulation_count,
			.m_link_count = m_link_count,
			.m_participant_count = m_participant_count,
			.m_target_count = m_target_count,
			.m_mobility_count = m_mobility_count,
			.m_articulation_range_count = m_articulation_count,
			.m_work_count = m_work_count,
			.m_phase = static_cast<int>(ECoupledContactPhase::Position),
			.m_velocity_delta_count = m_velocity_delta_count,
			.m_relaxation = m_config.constraint_coupled_relaxation,
			.m_restitution_scale = m_restitution_scale,
			.m_dt = m_dt,
			.m_position_slop = m_config.position_slop,
			.m_position_beta = m_config.position_baumgarte,
			.m_max_position_speed = m_config.constraint_max_position_speed,
			.m_position_iteration_index = -1,
			.m_pad1 = 0.0f,
		};
		job.m_cmd_list.SetPipelineState(m_cs_apply_position.m_pso.get());
		job.m_cmd_list.SetComputeRootSignature(m_cs_apply_position.m_sig.get());
		job.m_cmd_list.AddComputeRoot32BitConstants(constants);
		job.m_cmd_list.AddComputeRootShaderResourceView(m_aba.m_r_links->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView(m_mobility.m_r_ranges->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_bodies->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_rigid_pseudo->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_generalized_pseudo->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_aba.m_r_articulations->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(positions->GetGPUVirtualAddress());
		job.m_cmd_list.Dispatch(CoupledContactThreadGroupCount(std::max(m_rigid_body_count, m_articulation_count)), 1, 1);
		++m_stats.m_dispatch_count;

		job.m_barriers.UAV(m_r_bodies.get());
		job.m_barriers.UAV(m_aba.m_r_articulations.get());
		job.m_barriers.UAV(positions);
		job.m_barriers.Commit();
	}

	// Transition the common contact and articulation resources into the states required by contact shaders.
	void GpuCoupledContactSolver::PrepareCommonResources(GpuJob& job)
	{
		auto* rigid_pseudo = m_position_active && m_r_rigid_pseudo != nullptr ? m_r_rigid_pseudo.get() : m_r_uav_sentinel.get();
		auto* link_pseudo = m_position_active && m_r_link_pseudo != nullptr ? m_r_link_pseudo.get() : m_r_uav_sentinel.get();
		auto* generalized_pseudo = m_position_active && m_r_generalized_pseudo != nullptr ? m_r_generalized_pseudo.get() : m_r_uav_sentinel.get();
		job.m_barriers.Transition(m_r_counters.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(m_r_materials.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(m_aba.m_r_links.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(m_proxies.m_r_link_to_world.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(m_mobility.m_r_mobilities.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(m_aba.m_r_scratch.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(m_r_bodies.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_contacts.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_blocks.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_scratch.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_contributions.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_endpoint_keys.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_endpoint_order.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_target_impulses.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_participant_degrees.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_impulse_aba.m_r_link_impulses.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_tree_selection.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_tree_results.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_state.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(rigid_pseudo, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(link_pseudo, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(generalized_pseudo, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Commit();
	}

	// Order every mutable resource before the next dependent contact or ABA phase.
	void GpuCoupledContactSolver::CommitUavBarriers(GpuJob& job)
	{
		auto* rigid_pseudo = m_position_active && m_r_rigid_pseudo != nullptr ? m_r_rigid_pseudo.get() : m_r_uav_sentinel.get();
		auto* link_pseudo = m_position_active && m_r_link_pseudo != nullptr ? m_r_link_pseudo.get() : m_r_uav_sentinel.get();
		auto* generalized_pseudo = m_position_active && m_r_generalized_pseudo != nullptr ? m_r_generalized_pseudo.get() : m_r_uav_sentinel.get();
		job.m_barriers.UAV(m_r_bodies.get());
		job.m_barriers.UAV(m_r_contacts.get());
		job.m_barriers.UAV(m_r_blocks.get());
		job.m_barriers.UAV(m_r_scratch.get());
		job.m_barriers.UAV(m_r_contributions.get());
		job.m_barriers.UAV(m_r_endpoint_keys.get());
		job.m_barriers.UAV(m_r_endpoint_order.get());
		job.m_barriers.UAV(m_r_target_impulses.get());
		job.m_barriers.UAV(m_r_participant_degrees.get());
		job.m_barriers.UAV(m_impulse_aba.m_r_link_impulses.get());
		job.m_barriers.UAV(m_r_tree_selection.get());
		job.m_barriers.UAV(m_r_tree_results.get());
		job.m_barriers.UAV(m_r_state.get());
		job.m_barriers.UAV(rigid_pseudo);
		job.m_barriers.UAV(link_pseudo);
		job.m_barriers.UAV(generalized_pseudo);
		job.m_barriers.Commit();
	}

	// Destroy lazily owned contact resources where the implementation type is complete.
	void Deleter<GpuCoupledContactSolver>::operator()(GpuCoupledContactSolver* solver) const
	{
		delete solver;
	}
}
