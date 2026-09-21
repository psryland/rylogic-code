//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2025
//*********************************************
#include "pr/physics/integrator/engine_config.h"
#include "src/compute/resolve_gpu.h"
#include "src/compute/constraint_solver_gpu.h"
#include "src/compute/coupled_constraint_solver_gpu.h"
#include "src/compute/coupled_contact_gpu.h"
#include "src/compute/physics_types.h"
#include "src/compute/shader_code.h"

namespace pr::physics
{
	using namespace ::pr::compute;

	// Constant buffer layout matching the HLSL cbResolve declaration.
	struct alignas(16) cbResolve
	{
		int max_contacts; // The max capacity of the contacts buffer
		int body_count;   // The number of bodies in the scene
		int colour;       // Current colour batch being processed (for CSResolve)
		int sort_capacity;

		int shock_iterations;
		float max_position_speed;
		int shock_padding1;
		float shock_alignment;

		float shock_min_strength;
		float dt;         // timestep in seconds
		float support_only;
		float support_alignment;

		float restitution_scale;
		float penetration_slop;
		float velocity_baumgarte;
		float deep_penetration_threshold;

		float deep_penetration_range;
		float deep_penetration_baumgarte_min;
		float deep_penetration_baumgarte_max;
		float bias_scale;

		float propagation_key_scale;
		float position_slop;
		float position_baumgarte;
		float position_correction_scale;

		float shock_decay;
		float contact_slop_scale;
		float support_contact_slop_scale;
		float warm_start_scale;

		int warm_start_capacity;
		int rigid_body_count;
		int warm_start_preloaded;
		int shared_position_state;
	};
	static_assert((sizeof(cbResolve) & 0xf) == 0);
	static_assert(sizeof(cbResolve::colour) == sizeof(uint32_t));
	static_assert(offsetof(cbResolve, colour) % sizeof(uint32_t) == 0);

	// Register assignments for the resolve root signature
	struct EReg
	{
		inline static constexpr auto Params         = ECBufReg::b0;
		inline static constexpr auto Counters       = ESRVReg::t0;
		inline static constexpr auto Materials      = ESRVReg::t1;
		inline static constexpr auto Bodies         = EUAVReg::u0;
		inline static constexpr auto Colours        = EUAVReg::u1;
		inline static constexpr auto Contacts       = EUAVReg::u2;
		inline static constexpr auto ContactTimes   = EUAVReg::u3;
		inline static constexpr auto ContactOrder   = EUAVReg::u4;
		inline static constexpr auto BodyContactHead = EUAVReg::u5;
		inline static constexpr auto ContactNextA   = EUAVReg::u6;
		inline static constexpr auto ContactNextB   = EUAVReg::u7;
		inline static constexpr auto WarmStartPrev  = EUAVReg::u8;
		inline static constexpr auto WarmStartCurr  = EUAVReg::u9;
		inline static constexpr auto PositionPseudo = EUAVReg::u10;
	};

	GpuResolver::GpuResolver(Gpu& gpu, EngineConfig const& config, IShaderCache* shader_cache)
		: m_gpu(gpu)
		, m_config(config)
		, m_contact_sorter(gpu.m_gpu, ContactSorter::TuningParams{}, shader_cache)
		, m_cs_compute_times()
		, m_cs_clear_shock_lists()
		, m_cs_seed_shock_priority()
		, m_cs_propagate_shock_priority()
		, m_cs_commit_shock_priority()
		, m_cs_finalize_shock_priority()
		, m_cs_assign_colours()
		, m_cs_warm_start_clear()
		, m_cs_load_warm_start()
		, m_cs_apply_warm_start()
		, m_cs_store_warm_start()
		, m_cs_position_solve()
		, m_cs_resolve()
		, m_cmd_sig()
		, m_r_materials()
		, m_r_colours()
		, m_r_contact_times()
		, m_r_contact_order()
		, m_r_body_contact_head()
		, m_r_contact_next_a()
		, m_r_contact_next_b()
		, m_r_warm_start_prev()
		, m_r_warm_start_curr()
		, m_max_materials()
		, m_max_contacts()
		, m_body_capacity()
		, m_warm_start_capacity()
		, m_reset_warm_start_cache(true)
		, m_materials_dirty(true)
	{
		// m_cs_compute_times: parallel, one thread per contact — writes collision_time + zeroes body colour_used
		{
			auto sig = RootSig(ERootSigFlags::ComputeOnly)
				.U32<cbResolve>(EReg::Params)
				.SRV(EReg::Counters)
				.UAV(EReg::Bodies)
				.UAV(EReg::Contacts)
				.UAV(EReg::ContactTimes)
				.UAV(EReg::ContactOrder)
				;

			m_cs_compute_times.m_sig = sig.Create(m_gpu, "Physics:ComputeTimesSig");
			m_cs_compute_times.m_pso = ComputePSO(m_cs_compute_times.m_sig.get(), shader_code::compute_collision_times).Create(m_gpu, "Physics:ComputeTimesPSO");
		}

		// Shock-priority passes: build dynamic body adjacency, propagate priority in parallel, and finalise sort keys.
		{
			auto compile_step = [&](ComputeStep& step, shader_code::ByteCode const& bytecode, char const* name)
			{
				std::string sig_name = FmtS("Physics:%sSig", name);
				std::string pso_name = FmtS("Physics:%sPSO", name);
				step.m_sig = RootSig(ERootSigFlags::ComputeOnly)
					.U32<cbResolve>(EReg::Params)
					.SRV(EReg::Counters)
					.UAV(EReg::Bodies)
					.UAV(EReg::Colours)
					.UAV(EReg::Contacts)
					.UAV(EReg::ContactTimes)
					.UAV(EReg::ContactOrder)
					.UAV(EReg::BodyContactHead)
					.UAV(EReg::ContactNextA)
					.UAV(EReg::ContactNextB)
					.Create(m_gpu, sig_name.c_str());
				step.m_pso = ComputePSO(step.m_sig.get(), bytecode).Create(m_gpu, pso_name.c_str());
			};

			compile_step(m_cs_clear_shock_lists, shader_code::clear_shock_lists, "ClearShockLists");
			compile_step(m_cs_seed_shock_priority, shader_code::seed_shock_priority, "SeedShockPriority");
			compile_step(m_cs_propagate_shock_priority, shader_code::propagate_shock_priority, "PropagateShockPriority");
			compile_step(m_cs_commit_shock_priority, shader_code::commit_shock_priority, "CommitShockPriority");
			compile_step(m_cs_finalize_shock_priority, shader_code::finalize_shock_priority, "FinalizeShockPriority");
		}

		// m_cs_assign_colours: serial, walks sorted contacts + assigns colours
		{
			auto sig = RootSig(ERootSigFlags::ComputeOnly)
				.U32<cbResolve>(EReg::Params)
				.SRV(EReg::Counters)
				.UAV(EReg::Bodies)
				.UAV(EReg::Colours)
				.UAV(EReg::Contacts)
				.UAV(EReg::ContactOrder);

			m_cs_assign_colours.m_sig = sig.Create(m_gpu, "Physics:AssignColoursSig");
			m_cs_assign_colours.m_pso = ComputePSO(m_cs_assign_colours.m_sig.get(), shader_code::assign_colours).Create(m_gpu, "Physics:AssignColoursPSO");
		}

		// Warm-start passes: clear cache, apply previous-frame impulses, then store this frame's final impulses.
		{
			auto compile_step = [&](ComputeStep& step, shader_code::ByteCode const& bytecode, char const* name)
			{
				std::string sig_name = FmtS("Physics:%sSig", name);
				std::string pso_name = FmtS("Physics:%sPSO", name);
				step.m_sig = RootSig(ERootSigFlags::ComputeOnly)
					.U32<cbResolve>(EReg::Params)
					.SRV(EReg::Counters)
					.UAV(EReg::Bodies)
					.UAV(EReg::Colours)
					.UAV(EReg::Contacts)
					.UAV(EReg::ContactOrder)
					.UAV(EReg::WarmStartPrev)
					.UAV(EReg::WarmStartCurr)
					.Create(m_gpu, sig_name.c_str());
				step.m_pso = ComputePSO(step.m_sig.get(), bytecode).Create(m_gpu, pso_name.c_str());
			};

			compile_step(m_cs_warm_start_clear, shader_code::warm_start_clear, "WarmStartClear");
			compile_step(m_cs_load_warm_start, shader_code::load_warm_start, "LoadWarmStart");
			compile_step(m_cs_apply_warm_start, shader_code::apply_warm_start, "ApplyWarmStart");
			compile_step(m_cs_store_warm_start, shader_code::store_warm_start, "StoreWarmStart");
		}

		// m_cs_position_solve
		{
			auto sig = RootSig(ERootSigFlags::ComputeOnly)
				.U32<cbResolve>(EReg::Params)
				.SRV(EReg::Counters)
				.UAV(EReg::Bodies)
				.UAV(EReg::Colours)
				.UAV(EReg::Contacts)
				.UAV(EReg::ContactOrder)
				.UAV(EReg::PositionPseudo);

			m_cs_position_solve.m_sig = sig.Create(m_gpu, "Physics:PositionSolveSig");
			m_cs_position_solve.m_pso = ComputePSO(m_cs_position_solve.m_sig.get(), shader_code::position_solve).Create(m_gpu, "Physics:PositionSolvePSO");
		}

		// m_cs_resolve
		{
			auto sig = RootSig(ERootSigFlags::ComputeOnly)
				.U32<cbResolve>(EReg::Params)
				.SRV(EReg::Counters)
				.SRV(EReg::Materials)
				.UAV(EReg::Bodies)
				.UAV(EReg::Colours)
				.UAV(EReg::Contacts)
				.UAV(EReg::ContactOrder);

			m_cs_resolve.m_sig = sig.Create(m_gpu, "Physics:ResolveSig");
			m_cs_resolve.m_pso = ComputePSO(m_cs_resolve.m_sig.get(), shader_code::resolve).Create(m_gpu, "Physics:ResolvePSO");
		}

		// Create a command signature for indirect dispatch
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

	// Create or grow GPU buffers for contacts and colour assignments.
	void GpuResolver::ResizeBuffers(CmdList& cmd_list, int body_count, int max_contacts, int max_materials)
	{
		body_count = std::max(1, body_count);
		max_contacts = std::max(1, max_contacts);
		max_materials = std::max(1, max_materials);
		auto warm_start_capacity = 1;
		while (warm_start_capacity < max_contacts * 2)
			warm_start_capacity <<= 1;

		if (m_r_materials == nullptr || max_materials > m_max_materials)
		{
			m_r_materials = m_gpu.CreateResource(ResDesc::Buf<GpuMaterial>(max_materials, {}), cmd_list, "Physics:Materials");
			m_max_materials = max_materials;
			m_materials_dirty = true;
		}
		if (m_r_colours == nullptr || m_max_contacts < max_contacts)
		{
			// Reserve one element beyond the sortable contact capacity for the frame-wide colour-overflow flag.
			m_r_colours = m_gpu.CreateResource(ResDesc::Buf<uint32_t>(max_contacts + 1, {}).usage(EUsage::UnorderedAccess), cmd_list, "Physics:ResolveColours");
			m_r_contact_times = m_gpu.CreateResource(ResDesc::Buf<float>(max_contacts, {}).usage(EUsage::UnorderedAccess), cmd_list, "Physics:ContactTimes");
			m_r_contact_order = m_gpu.CreateResource(ResDesc::Buf<uint32_t>(max_contacts, {}).usage(EUsage::UnorderedAccess), cmd_list, "Physics:ContactOrder");
			m_r_contact_next_a = m_gpu.CreateResource(ResDesc::Buf<uint32_t>(max_contacts, {}).usage(EUsage::UnorderedAccess), cmd_list, "Physics:ContactNextA");
			m_r_contact_next_b = m_gpu.CreateResource(ResDesc::Buf<uint32_t>(max_contacts, {}).usage(EUsage::UnorderedAccess), cmd_list, "Physics:ContactNextB");
			m_max_contacts = max_contacts;
		}
		if (m_r_body_contact_head == nullptr || m_body_capacity < body_count)
		{
			m_r_body_contact_head = m_gpu.CreateResource(ResDesc::Buf<uint32_t>(body_count, {}).usage(EUsage::UnorderedAccess), cmd_list, "Physics:BodyContactHead");
			m_body_capacity = body_count;
		}
		if (m_r_warm_start_prev == nullptr || m_warm_start_capacity < warm_start_capacity)
		{
			m_r_warm_start_prev = m_gpu.CreateResource(ResDesc::Buf<GpuWarmStartEntry>(warm_start_capacity, {}).usage(EUsage::UnorderedAccess), cmd_list, "Physics:WarmStartPrev");
			m_r_warm_start_curr = m_gpu.CreateResource(ResDesc::Buf<GpuWarmStartEntry>(warm_start_capacity, {}).usage(EUsage::UnorderedAccess), cmd_list, "Physics:WarmStartCurr");
			m_warm_start_capacity = warm_start_capacity;
			m_reset_warm_start_cache = true;
		}
	}

	// Resolve ordinary rigid contacts while retaining proxy-touching contacts for the coupled articulation lane.
	void GpuResolver::Resolve(GpuJob& job, float dt, int body_count, int rigid_body_count, int max_contacts,
		D3DPtr<ID3D12Resource> dispatch, D3DPtr<ID3D12Resource> counters, D3DPtr<ID3D12Resource> contacts, D3DPtr<ID3D12Resource> bodies,
		std::span<GpuMaterial const> materials, float bias_scale, int solver_iterations_, int push_out_iterations, float restitution_scale, bool support_only,
		GpuConstraintSolver* constraint_solver, GpuCoupledConstraintSolver* coupled_constraint_solver, GpuCoupledContactSolver* coupled_contact_solver,
		bool retain_constraint_impulses, int substep_index)
	{
		if (rigid_body_count < 0 || rigid_body_count > body_count)
			throw std::invalid_argument("GPU resolver rigid-body prefix is outside the submitted body range");

		auto material_count = static_cast<int>(materials.size());
		pix::BeginEvent(job.m_cmd_list.get(), 0xFF6799Ab, "Physics::Resolve");

		ResizeBuffers(job.m_cmd_list, body_count, max_contacts, material_count);

		auto const push_out_steps = std::max(0, push_out_iterations >= 0 ? push_out_iterations : m_config.push_out_iterations);
		auto const solver_iterations = std::max(0, solver_iterations_ >= 0 ? solver_iterations_ : m_config.solver_iterations);
		auto const position_correction_scale = push_out_steps != 0 ? 1.0f / push_out_steps : 0.0f;
		auto const priority_sort_enabled =
			m_config.contact_sort_propagation_scale > 0.0f &&
			m_config.contact_sort_shock_iterations > 0;

		cbResolve cb_resolve = {
			.max_contacts = max_contacts,
			.body_count = body_count,
			.colour = 0,
			.sort_capacity = m_max_contacts,
			.shock_iterations = m_config.contact_sort_shock_iterations,
			.max_position_speed = m_config.constraint_max_position_speed,
			.shock_padding1 = 0,
			.shock_alignment = m_config.contact_sort_shock_alignment,
			.shock_min_strength = m_config.contact_sort_shock_min_strength,
			.dt = dt,
			.support_only = support_only ? 1.0f : 0.0f,
			.support_alignment = m_config.selective_refresh_support_alignment,
			.restitution_scale = restitution_scale,
			.penetration_slop = m_config.penetration_slop,
			.velocity_baumgarte = m_config.velocity_baumgarte,
			.deep_penetration_threshold = m_config.deep_penetration_threshold,
			.deep_penetration_range = m_config.deep_penetration_range,
			.deep_penetration_baumgarte_min = m_config.deep_penetration_baumgarte_min,
			.deep_penetration_baumgarte_max = m_config.deep_penetration_baumgarte_max,
			.bias_scale = bias_scale,
			.propagation_key_scale = m_config.contact_sort_propagation_scale,
			.position_slop = m_config.position_slop,
			.position_baumgarte = m_config.position_baumgarte,
			.position_correction_scale = position_correction_scale,
			.shock_decay = m_config.contact_sort_shock_decay,
			.contact_slop_scale = m_config.contact_slop_scale,
			.support_contact_slop_scale = m_config.support_contact_slop_scale,
			.warm_start_scale = m_config.warm_start_scale,
			.warm_start_capacity = m_warm_start_capacity,
			.rigid_body_count = rigid_body_count,
			.warm_start_preloaded = coupled_contact_solver != nullptr ? 1 : 0,
			.shared_position_state = 0,
		};
		if (m_config.warm_start_scale <= 0.0f)
			m_reset_warm_start_cache = true;

		// Own the per-call resolver state explicitly so phase dependencies are visible without implicit lambda captures.
		struct ResolvePhases
		{
			GpuResolver& m_resolver;
			GpuJob& m_job;
			float m_dt;
			int m_body_count;
			int m_rigid_body_count;
			int m_max_contacts;
			int m_material_count;
			int m_push_out_steps;
			int m_solver_iterations;
			float m_restitution_scale;
			bool m_priority_sort_enabled;
			bool m_retain_constraint_impulses;
			int m_substep_index;
			D3DPtr<ID3D12Resource>& m_dispatch;
			D3DPtr<ID3D12Resource>& m_counters;
			D3DPtr<ID3D12Resource>& m_contacts;
			D3DPtr<ID3D12Resource>& m_bodies;
			std::span<GpuMaterial const> m_materials;
			GpuConstraintSolver* m_constraint_solver;
			GpuCoupledConstraintSolver* m_coupled_constraint_solver;
			GpuCoupledContactSolver* m_coupled_contact_solver;
			cbResolve& m_cb;

			// Record every resolver phase in dependency order.
			void Run()
			{
				PrepareMaterials();
				TransitionResources();
				ClearWarmStartCache();
				ComputeContactTimes();
				PropagateContactPriority();
				SortContacts();
				ColourContacts();
				PrepareSolverWork();
				ApplyWarmStart();

				// Contact-only frames keep their established order; coupled lanes need physical impulses before fixed-configuration correction.
				if (m_coupled_constraint_solver != nullptr || m_coupled_contact_solver != nullptr)
				{
					SolveVelocity();
					SolvePosition();
				}
				else
				{
					SolvePosition();
					SolveVelocity();
				}

				StoreWarmStart();
			}

			// Upload materials only after the CPU material map changes or the GPU buffer grows.
			void PrepareMaterials()
			{
				// Main and selective resolvers have separate GPU buffers, so each tracks this independently.
				if (!m_resolver.m_materials_dirty)
					return;

				m_job.m_barriers.Transition(m_resolver.m_r_materials.get(), D3D12_RESOURCE_STATE_COPY_DEST);
				m_job.m_barriers.Commit();

				auto mat_upload = m_job.m_upload.Alloc<GpuMaterial>(std::max(1, m_material_count));
				memcpy(mat_upload.ptr<GpuMaterial>(), m_materials.data(), m_material_count * sizeof(GpuMaterial));
				m_job.m_cmd_list.CopyBufferRegion(m_resolver.m_r_materials.get(), 0, mat_upload);

				m_job.m_barriers.Transition(m_resolver.m_r_materials.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
				m_job.m_barriers.Commit();
				m_resolver.m_materials_dirty = false;
			}

			// Put persistent resolver resources into the states expected by the compute phases.
			void TransitionResources()
			{
				// All transitions are committed together before any phase records root bindings.
				m_job.m_barriers.Transition(m_dispatch.get(), D3D12_RESOURCE_STATE_INDIRECT_ARGUMENT);
				m_job.m_barriers.Transition(m_counters.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
				m_job.m_barriers.Transition(m_resolver.m_r_materials.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
				m_job.m_barriers.Transition(m_bodies.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
				m_job.m_barriers.Transition(m_resolver.m_r_colours.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
				m_job.m_barriers.Transition(m_contacts.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
				m_job.m_barriers.Transition(m_resolver.m_r_contact_times.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
				m_job.m_barriers.Transition(m_resolver.m_r_contact_order.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
				m_job.m_barriers.Transition(m_resolver.m_r_body_contact_head.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
				m_job.m_barriers.Transition(m_resolver.m_r_contact_next_a.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
				m_job.m_barriers.Transition(m_resolver.m_r_contact_next_b.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
				m_job.m_barriers.Transition(m_resolver.m_r_warm_start_prev.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
				m_job.m_barriers.Transition(m_resolver.m_r_warm_start_curr.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
				m_job.m_barriers.Commit();
			}

			// Bind one warm-start compute step to the common resolver resources.
			void BindWarmStartStep(ComputeStep& step, ID3D12Resource* warm_start_curr)
			{
				// The selected current-cache resource distinguishes clear, load, apply, and store operations.
				m_job.m_cmd_list.SetPipelineState(step.m_pso.get());
				m_job.m_cmd_list.SetComputeRootSignature(step.m_sig.get());
				m_job.m_cmd_list.AddComputeRoot32BitConstants(m_cb);
				m_job.m_cmd_list.AddComputeRootShaderResourceView(m_counters->GetGPUVirtualAddress());
				m_job.m_cmd_list.AddComputeRootUnorderedAccessView(m_bodies->GetGPUVirtualAddress());
				m_job.m_cmd_list.AddComputeRootUnorderedAccessView(m_resolver.m_r_colours->GetGPUVirtualAddress());
				m_job.m_cmd_list.AddComputeRootUnorderedAccessView(m_contacts->GetGPUVirtualAddress());
				m_job.m_cmd_list.AddComputeRootUnorderedAccessView(m_resolver.m_r_contact_order->GetGPUVirtualAddress());
				m_job.m_cmd_list.AddComputeRootUnorderedAccessView(m_resolver.m_r_warm_start_prev->GetGPUVirtualAddress());
				m_job.m_cmd_list.AddComputeRootUnorderedAccessView(warm_start_curr->GetGPUVirtualAddress());
			}

			// Make warm-start writes visible to the next resolver phase.
			void CommitWarmStartBarriers()
			{
				// Every warm-start kernel can update bodies, contacts, and either cache role.
				m_job.m_barriers.UAV(m_bodies.get());
				m_job.m_barriers.UAV(m_contacts.get());
				m_job.m_barriers.UAV(m_resolver.m_r_warm_start_prev.get());
				m_job.m_barriers.UAV(m_resolver.m_r_warm_start_curr.get());
				m_job.m_barriers.Commit();
			}

			// Clear the current cache and initialise a newly allocated previous cache.
			void ClearWarmStartCache()
			{
				// The previous cache is cleared once so the first lookup is deterministic; the current cache is cleared every frame.
				auto const warm_start_group_count = static_cast<UINT>(std::max(1, (m_resolver.m_warm_start_capacity + ResolveThreadCount - 1) / ResolveThreadCount));
				if (m_resolver.m_reset_warm_start_cache)
				{
					BindWarmStartStep(m_resolver.m_cs_warm_start_clear, m_resolver.m_r_warm_start_prev.get());
					m_job.m_cmd_list.Dispatch(warm_start_group_count, 1, 1);
					CommitWarmStartBarriers();
					m_resolver.m_reset_warm_start_cache = false;
				}

				BindWarmStartStep(m_resolver.m_cs_warm_start_clear, m_resolver.m_r_warm_start_curr.get());
				m_job.m_cmd_list.Dispatch(warm_start_group_count, 1, 1);
				CommitWarmStartBarriers();
			}

			// Calculate contact sort keys and clear body colour masks.
			void ComputeContactTimes()
			{
				// Gravity-biased collision times provide the initial ordering before optional shock-priority propagation.
				m_job.m_cmd_list.SetPipelineState(m_resolver.m_cs_compute_times.m_pso.get());
				m_job.m_cmd_list.SetComputeRootSignature(m_resolver.m_cs_compute_times.m_sig.get());
				m_job.m_cmd_list.AddComputeRoot32BitConstants(m_cb);
				m_job.m_cmd_list.AddComputeRootShaderResourceView(m_counters->GetGPUVirtualAddress());
				m_job.m_cmd_list.AddComputeRootUnorderedAccessView(m_bodies->GetGPUVirtualAddress());
				m_job.m_cmd_list.AddComputeRootUnorderedAccessView(m_contacts->GetGPUVirtualAddress());
				m_job.m_cmd_list.AddComputeRootUnorderedAccessView(m_resolver.m_r_contact_times->GetGPUVirtualAddress());
				m_job.m_cmd_list.AddComputeRootUnorderedAccessView(m_resolver.m_r_contact_order->GetGPUVirtualAddress());

				// CSCalcResolveDispatch always records at least one group so colour masks and contact times are initialised for empty GPU counts.
				m_job.m_cmd_list.ExecuteIndirect(m_resolver.m_cmd_sig.get(), 1, m_dispatch.get());

				m_job.m_barriers.UAV(m_bodies.get());
				m_job.m_barriers.UAV(m_contacts.get());
				m_job.m_barriers.UAV(m_resolver.m_r_contact_times.get());
				m_job.m_barriers.UAV(m_resolver.m_r_contact_order.get());
				m_job.m_barriers.Commit();
			}

			// Bind one shock-priority compute step to the contact graph resources.
			void BindShockStep(ComputeStep& step)
			{
				// Every shock phase shares the same adjacency and sort-key layout.
				m_job.m_cmd_list.SetPipelineState(step.m_pso.get());
				m_job.m_cmd_list.SetComputeRootSignature(step.m_sig.get());
				m_job.m_cmd_list.AddComputeRoot32BitConstants(m_cb);
				m_job.m_cmd_list.AddComputeRootShaderResourceView(m_counters->GetGPUVirtualAddress());
				m_job.m_cmd_list.AddComputeRootUnorderedAccessView(m_bodies->GetGPUVirtualAddress());
				m_job.m_cmd_list.AddComputeRootUnorderedAccessView(m_resolver.m_r_colours->GetGPUVirtualAddress());
				m_job.m_cmd_list.AddComputeRootUnorderedAccessView(m_contacts->GetGPUVirtualAddress());
				m_job.m_cmd_list.AddComputeRootUnorderedAccessView(m_resolver.m_r_contact_times->GetGPUVirtualAddress());
				m_job.m_cmd_list.AddComputeRootUnorderedAccessView(m_resolver.m_r_contact_order->GetGPUVirtualAddress());
				m_job.m_cmd_list.AddComputeRootUnorderedAccessView(m_resolver.m_r_body_contact_head->GetGPUVirtualAddress());
				m_job.m_cmd_list.AddComputeRootUnorderedAccessView(m_resolver.m_r_contact_next_a->GetGPUVirtualAddress());
				m_job.m_cmd_list.AddComputeRootUnorderedAccessView(m_resolver.m_r_contact_next_b->GetGPUVirtualAddress());
			}

			// Make shock-priority graph writes visible to the next propagation phase.
			void CommitShockBarriers()
			{
				// Propagation reads the adjacency and priority values written by the preceding dispatch.
				m_job.m_barriers.UAV(m_resolver.m_r_colours.get());
				m_job.m_barriers.UAV(m_resolver.m_r_contact_times.get());
				m_job.m_barriers.UAV(m_resolver.m_r_contact_order.get());
				m_job.m_barriers.UAV(m_resolver.m_r_body_contact_head.get());
				m_job.m_barriers.UAV(m_resolver.m_r_contact_next_a.get());
				m_job.m_barriers.UAV(m_resolver.m_r_contact_next_b.get());
				m_job.m_barriers.Commit();
			}

			// Propagate contact support priority through the dynamic-body contact graph.
			void PropagateContactPriority()
			{
				// Disabled propagation leaves the original collision-time keys unchanged.
				if (!m_priority_sort_enabled)
					return;

				auto const body_group_count = static_cast<UINT>(std::max(1, (m_body_count + ResolveThreadCount - 1) / ResolveThreadCount));
				BindShockStep(m_resolver.m_cs_clear_shock_lists);
				m_job.m_cmd_list.Dispatch(body_group_count, 1, 1);
				CommitShockBarriers();

				BindShockStep(m_resolver.m_cs_seed_shock_priority);
				m_job.m_cmd_list.ExecuteIndirect(m_resolver.m_cmd_sig.get(), 1, m_dispatch.get());
				CommitShockBarriers();

				for (int iter = 0; iter != m_cb.shock_iterations; ++iter)
				{
					BindShockStep(m_resolver.m_cs_propagate_shock_priority);
					m_job.m_cmd_list.ExecuteIndirect(m_resolver.m_cmd_sig.get(), 1, m_dispatch.get());
					CommitShockBarriers();

					BindShockStep(m_resolver.m_cs_commit_shock_priority);
					m_job.m_cmd_list.ExecuteIndirect(m_resolver.m_cmd_sig.get(), 1, m_dispatch.get());
					CommitShockBarriers();
				}

				BindShockStep(m_resolver.m_cs_finalize_shock_priority);
				m_job.m_cmd_list.ExecuteIndirect(m_resolver.m_cmd_sig.get(), 1, m_dispatch.get());
				CommitShockBarriers();

				m_job.m_barriers.UAV(m_contacts.get());
				m_job.m_barriers.Commit();
			}

			// Sort contacts by their collision-time and propagated-priority key.
			void SortContacts()
			{
				// The payload preserves the original contact buffer while defining solver order separately.
				m_resolver.m_contact_sorter.Bind(m_job.m_cmd_list, m_resolver.m_max_contacts, m_resolver.m_r_contact_times, m_resolver.m_r_contact_order);
				m_resolver.m_contact_sorter.Sort(m_job.m_cmd_list);

				m_job.m_barriers.UAV(m_resolver.m_r_contact_times.get());
				m_job.m_barriers.UAV(m_resolver.m_r_contact_order.get());
				m_job.m_barriers.Commit();
			}

			// Assign graph colours to the sorted contacts.
			void ColourContacts()
			{
				// Contacts sharing a body receive different colours so each colour batch owns exclusive body writes.
				m_job.m_cmd_list.SetPipelineState(m_resolver.m_cs_assign_colours.m_pso.get());
				m_job.m_cmd_list.SetComputeRootSignature(m_resolver.m_cs_assign_colours.m_sig.get());
				m_job.m_cmd_list.AddComputeRoot32BitConstants(m_cb);
				m_job.m_cmd_list.AddComputeRootShaderResourceView(m_counters->GetGPUVirtualAddress());
				m_job.m_cmd_list.AddComputeRootUnorderedAccessView(m_bodies->GetGPUVirtualAddress());
				m_job.m_cmd_list.AddComputeRootUnorderedAccessView(m_resolver.m_r_colours->GetGPUVirtualAddress());
				m_job.m_cmd_list.AddComputeRootUnorderedAccessView(m_contacts->GetGPUVirtualAddress());
				m_job.m_cmd_list.AddComputeRootUnorderedAccessView(m_resolver.m_r_contact_order->GetGPUVirtualAddress());
				m_job.m_cmd_list.Dispatch(1, 1, 1);

				m_job.m_barriers.UAV(m_bodies.get());
				m_job.m_barriers.UAV(m_resolver.m_r_colours.get());
				m_job.m_barriers.Commit();
			}

			// Prepare persistent constraints and coupled contacts for their iterative solver phases.
			void PrepareSolverWork()
			{
				// Constraint blocks are compiled after integration has produced the current body transforms.
				if (m_constraint_solver != nullptr)
					m_constraint_solver->Prepare(m_job, m_dt, m_rigid_body_count, m_bodies, m_retain_constraint_impulses);
				if (m_coupled_constraint_solver != nullptr)
					m_coupled_constraint_solver->PrepareVelocity(m_job, m_dt, m_rigid_body_count, m_bodies.get(), m_retain_constraint_impulses);

				// Articulation contacts need every cache result before their topology pass; rigid-only frames use the fused load-and-apply path.
				if (m_resolver.m_config.warm_start_scale > 0.0f && m_coupled_contact_solver != nullptr)
				{
					BindWarmStartStep(m_resolver.m_cs_load_warm_start, m_resolver.m_r_warm_start_curr.get());
					m_job.m_cmd_list.ExecuteIndirect(m_resolver.m_cmd_sig.get(), 1, m_dispatch.get());
					CommitWarmStartBarriers();
				}

				// Exact articulation self-mobility and contact blocks depend on the collision frame and loaded warm-start accumulator.
				if (m_coupled_contact_solver != nullptr)
					m_coupled_contact_solver->PrepareVelocity(m_job, m_dt, m_body_count, m_rigid_body_count, m_max_contacts, m_counters, m_contacts, m_bodies, m_resolver.m_r_materials, m_restitution_scale);
			}

			// Apply retained physical impulses before iterative solving.
			void ApplyWarmStart()
			{
				// Cached support starts resting contacts close to the preceding frame's accepted solution.
				if (m_resolver.m_config.warm_start_scale > 0.0f)
				{
					BindWarmStartStep(m_resolver.m_cs_apply_warm_start, m_resolver.m_r_warm_start_curr.get());
					for (int colour = 0; colour != MaxColours; ++colour)
					{
						m_cb.colour = colour;
						m_job.m_cmd_list.SetComputeRoot32BitConstants(0, 1, &m_cb.colour, offsetof(cbResolve, colour) / sizeof(uint32_t));
						m_job.m_cmd_list.ExecuteIndirect(m_resolver.m_cmd_sig.get(), 1, m_dispatch.get());
						CommitWarmStartBarriers();
					}
					m_cb.colour = 0;
					if (m_coupled_contact_solver != nullptr)
						m_coupled_contact_solver->ApplyWarmStart(m_job);
				}
				if (!m_retain_constraint_impulses)
				{
					if (m_constraint_solver != nullptr)
						m_constraint_solver->ApplyWarmStart(m_job, m_dt, m_rigid_body_count, m_bodies);
					if (m_coupled_constraint_solver != nullptr)
						m_coupled_constraint_solver->ApplyWarmStart(m_job, m_rigid_body_count, m_bodies.get());
				}
			}

			// Bind the contact position solver after any constraint root-signature change.
			void BindPositionSolve(ID3D12Resource* pseudo_buffer)
			{
				// Contact-only solving aliases existing UAV scratch rather than allocating an unused pseudo-state sentinel.
				m_job.m_barriers.Transition(pseudo_buffer, D3D12_RESOURCE_STATE_UNORDERED_ACCESS).Commit();
				m_job.m_cmd_list.SetPipelineState(m_resolver.m_cs_position_solve.m_pso.get());
				m_job.m_cmd_list.SetComputeRootSignature(m_resolver.m_cs_position_solve.m_sig.get());
				m_job.m_cmd_list.AddComputeRoot32BitConstants(m_cb);
				m_job.m_cmd_list.AddComputeRootShaderResourceView(m_counters->GetGPUVirtualAddress());
				m_job.m_cmd_list.AddComputeRootUnorderedAccessView(m_bodies->GetGPUVirtualAddress());
				m_job.m_cmd_list.AddComputeRootUnorderedAccessView(m_resolver.m_r_colours->GetGPUVirtualAddress());
				m_job.m_cmd_list.AddComputeRootUnorderedAccessView(m_contacts->GetGPUVirtualAddress());
				m_job.m_cmd_list.AddComputeRootUnorderedAccessView(m_resolver.m_r_contact_order->GetGPUVirtualAddress());
				m_job.m_cmd_list.AddComputeRootUnorderedAccessView(pseudo_buffer->GetGPUVirtualAddress());
			}

			// Solve detached position correction without changing physical momentum.
			void SolvePosition()
			{
				// Every position lane writes shared pseudo-position state, including coupled-only frames.
				auto const has_constraint_work = m_constraint_solver != nullptr || m_coupled_constraint_solver != nullptr || m_coupled_contact_solver != nullptr;
				auto rigid_pseudo = D3DPtr<ID3D12Resource>{};
				if (has_constraint_work && m_push_out_steps > 0)
				{
					if (!(m_resolver.m_config.constraint_max_position_speed >= 0.0f) || !std::isfinite(m_resolver.m_config.constraint_max_position_speed))
						throw std::invalid_argument("Shared position correction requires a finite non-negative maximum speed");

					if (m_coupled_constraint_solver != nullptr)
						rigid_pseudo = m_coupled_constraint_solver->RigidPseudoVelocityStorage(m_job.m_cmd_list, m_rigid_body_count);
					else if (m_constraint_solver != nullptr)
						rigid_pseudo = m_constraint_solver->PseudoVelocityStorage(m_job.m_cmd_list, m_rigid_body_count);
				}

				// Contacts own complete-forest pseudo streams; compact joint work maps into the same streams for one coherent configuration.
				auto const coupled_contact_position_active =
					m_coupled_contact_solver != nullptr &&
					m_coupled_contact_solver->PreparePosition(m_job, m_push_out_steps, rigid_pseudo);
				auto const shared_pseudo = coupled_contact_position_active
					? m_coupled_contact_solver->PseudoState()
					: GpuPositionPseudoBuffers{};
				auto const coupled_position_active =
					m_coupled_constraint_solver != nullptr &&
					m_coupled_constraint_solver->PreparePosition(m_job, m_dt, m_rigid_body_count, m_bodies.get(), shared_pseudo, m_push_out_steps);
				if (coupled_contact_position_active)
					rigid_pseudo = shared_pseudo.m_rigid_velocities;

				m_cb.shared_position_state = rigid_pseudo != nullptr;
				if (m_push_out_steps != 0)
				{
					// Contact depths are stale collision-pass values, so each sweep applies only its share of the intended total correction.
					auto* pseudo_buffer = rigid_pseudo != nullptr ? rigid_pseudo.get() : m_resolver.m_r_colours.get();
					BindPositionSolve(pseudo_buffer);

					for (int iter = 0; iter != m_push_out_steps; ++iter)
					{
						// Constraint sweeps leave another root signature active, so every contact binding is restored before the next outer sweep.
						if (iter != 0 && has_constraint_work)
							BindPositionSolve(pseudo_buffer);

						for (int colour = 0; colour != MaxColours; ++colour)
						{
							// Each graph colour owns exclusive body writes; barriers expose pseudo-state updates to later colours and solvers.
							m_cb.colour = colour;
							m_job.m_cmd_list.SetComputeRoot32BitConstants(0, 1, &m_cb.colour, offsetof(cbResolve, colour) / sizeof(uint32_t));
							m_job.m_cmd_list.ExecuteIndirect(m_resolver.m_cmd_sig.get(), 1, m_dispatch.get());

							m_job.m_barriers.UAV(m_bodies.get());
							if (rigid_pseudo != nullptr)
								m_job.m_barriers.UAV(rigid_pseudo.get());

							m_job.m_barriers.Commit();
						}
						if (m_constraint_solver != nullptr)
							m_constraint_solver->SolvePositionIteration(m_job, m_dt, m_rigid_body_count, m_push_out_steps, m_bodies);
						if (coupled_position_active)
							m_coupled_constraint_solver->SolvePositionIteration(m_job, m_bodies.get(), m_substep_index);
						if (coupled_contact_position_active)
							m_coupled_contact_solver->SolvePositionIteration(m_job, iter);
					}
					m_cb.colour = 0;
				}

				// Apply shared pseudo state once through the owner that covers every participating articulation.
				if (coupled_contact_position_active)
					m_coupled_contact_solver->ApplyPosition(m_job);
				else if (coupled_position_active)
					m_coupled_constraint_solver->ApplyPosition(m_job, m_bodies.get());
				else if (m_constraint_solver != nullptr)
					m_constraint_solver->ApplyPosition(m_job, m_dt, m_rigid_body_count, m_push_out_steps, m_bodies);
			}

			// Bind the physical contact solver after any constraint root-signature change.
			void BindVelocitySolve()
			{
				// Restore every root constant and resource used by the contact table.
				m_job.m_cmd_list.SetPipelineState(m_resolver.m_cs_resolve.m_pso.get());
				m_job.m_cmd_list.SetComputeRootSignature(m_resolver.m_cs_resolve.m_sig.get());
				m_job.m_cmd_list.AddComputeRoot32BitConstants(m_cb);
				m_job.m_cmd_list.AddComputeRootShaderResourceView(m_counters->GetGPUVirtualAddress());
				m_job.m_cmd_list.AddComputeRootShaderResourceView(m_resolver.m_r_materials->GetGPUVirtualAddress());
				m_job.m_cmd_list.AddComputeRootUnorderedAccessView(m_bodies->GetGPUVirtualAddress());
				m_job.m_cmd_list.AddComputeRootUnorderedAccessView(m_resolver.m_r_colours->GetGPUVirtualAddress());
				m_job.m_cmd_list.AddComputeRootUnorderedAccessView(m_contacts->GetGPUVirtualAddress());
				m_job.m_cmd_list.AddComputeRootUnorderedAccessView(m_resolver.m_r_contact_order->GetGPUVirtualAddress());
			}

			// Solve physical contact impulses and body momentum.
			void SolveVelocity()
			{
				// Gauss-Seidel sweeps re-read momentum changed by earlier colours and constraints; CSResolve guards against energy injection.
				auto const has_constraint_work = m_constraint_solver != nullptr || m_coupled_constraint_solver != nullptr || m_coupled_contact_solver != nullptr;
				BindVelocitySolve();

				for (int iter = 0; iter != m_solver_iterations; ++iter)
				{
					// Constraint sweeps use another root layout, so rebind the contact table before the next outer sweep.
					if (iter != 0 && has_constraint_work)
						BindVelocitySolve();

					for (int colour = 0; colour != MaxColours; ++colour)
					{
						// Each graph colour owns exclusive body writes; barriers expose momentum changes to later colours and solvers.
						m_cb.colour = colour;
						m_job.m_cmd_list.SetComputeRoot32BitConstants(0, 1, &m_cb.colour, offsetof(cbResolve, colour) / sizeof(uint32_t));
						m_job.m_cmd_list.ExecuteIndirect(m_resolver.m_cmd_sig.get(), 1, m_dispatch.get());
						m_job.m_barriers.UAV(m_bodies.get());
						m_job.m_barriers.Commit();
					}
					if (m_constraint_solver != nullptr)
						m_constraint_solver->SolveVelocityIteration(m_job, m_dt, m_rigid_body_count, m_bodies);
					if (m_coupled_constraint_solver != nullptr)
						m_coupled_constraint_solver->SolveVelocityIteration(m_job, m_rigid_body_count, m_bodies.get(), m_substep_index);
					if (m_coupled_contact_solver != nullptr)
						m_coupled_contact_solver->SolveVelocityIteration(m_job);
				}
				m_cb.colour = 0;
			}

			// Persist accepted physical impulses for the next frame.
			void StoreWarmStart()
			{
				// Disabled warm-starting leaves both cache roles untouched after their required clear phase.
				if (m_resolver.m_config.warm_start_scale <= 0.0f)
					return;

				BindWarmStartStep(m_resolver.m_cs_store_warm_start, m_resolver.m_r_warm_start_curr.get());
				m_job.m_cmd_list.ExecuteIndirect(m_resolver.m_cmd_sig.get(), 1, m_dispatch.get());
				CommitWarmStartBarriers();
				std::swap(m_resolver.m_r_warm_start_prev, m_resolver.m_r_warm_start_curr);
			}
		};

		auto phases = ResolvePhases{
			.m_resolver = *this,
			.m_job = job,
			.m_dt = dt,
			.m_body_count = body_count,
			.m_rigid_body_count = rigid_body_count,
			.m_max_contacts = max_contacts,
			.m_material_count = material_count,
			.m_push_out_steps = push_out_steps,
			.m_solver_iterations = solver_iterations,
			.m_restitution_scale = restitution_scale,
			.m_priority_sort_enabled = priority_sort_enabled,
			.m_retain_constraint_impulses = retain_constraint_impulses,
			.m_substep_index = substep_index,
			.m_dispatch = dispatch,
			.m_counters = counters,
			.m_contacts = contacts,
			.m_bodies = bodies,
			.m_materials = materials,
			.m_constraint_solver = constraint_solver,
			.m_coupled_constraint_solver = coupled_constraint_solver,
			.m_coupled_contact_solver = coupled_contact_solver,
			.m_cb = cb_resolve,
		};
		phases.Run();

		pix::EndEvent(job.m_cmd_list.get());
	}

	// Mark the material buffer dirty so it is re-uploaded on the next resolve.
	void GpuResolver::MaterialsDirty()
	{
		m_materials_dirty = true;
	}

	// Discard retained contact impulses after rejected recorded work or a topology reset.
	void GpuResolver::InvalidateWarmStart()
	{
		m_reset_warm_start_cache = true;
	}

	// CPU-side testing: upload contacts and bodies, run graph colouring + resolve on GPU, readback bodies.
	void GpuResolver::Resolve(GpuJob& job, float dt, std::span<GpuResolveContact const> contacts, std::span<GpuRigidBody> bodies, std::span<GpuMaterial const> materials)
	{
		auto contact_count = static_cast<int>(contacts.size());
		auto body_count = static_cast<int>(bodies.size());
		if (contact_count == 0 || body_count == 0)
		{
			return;
		}

		// Create temporary GPU resources
		auto r_counters = m_gpu.CreateResource(ResDesc::Buf<GpuCollisionCounters>(1, {}), job.m_cmd_list, "Physics:TempCounters");
		auto r_contacts = m_gpu.CreateResource(ResDesc::Buf<GpuResolveContact>(contact_count, {}).usage(EUsage::UnorderedAccess), job.m_cmd_list, "Physics:TempContacts");
		auto r_bodies = m_gpu.CreateResource(ResDesc::Buf<GpuRigidBody>(body_count, {}).usage(EUsage::UnorderedAccess), job.m_cmd_list, "Physics:TempBodies");
		auto r_dispatch = m_gpu.CreateResource(ResDesc::Buf<D3D12_DISPATCH_ARGUMENTS>(1, {}).usage(EUsage::UnorderedAccess), job.m_cmd_list, "Physics:TempDispatch");

		// Upload counters
		{
			job.m_barriers.Transition(r_counters.get(), D3D12_RESOURCE_STATE_COPY_DEST);
			job.m_barriers.Commit();

			auto upload = job.m_upload.Alloc<GpuCollisionCounters>(1);
			*upload.ptr<GpuCollisionCounters>() = GpuCollisionCounters{
				.pair_count = 0,
				.contact_count = contact_count,
			};
			job.m_cmd_list.CopyBufferRegion(r_counters.get(), 0, upload);

			job.m_barriers.Transition(r_counters.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
			job.m_barriers.Commit();
		}

		// Upload contacts
		{
			job.m_barriers.Transition(r_contacts.get(), D3D12_RESOURCE_STATE_COPY_DEST);
			job.m_barriers.Commit();

			auto upload = job.m_upload.Alloc<GpuResolveContact>(contact_count);
			memcpy(upload.ptr<GpuResolveContact>(), contacts.data(), contact_count * sizeof(GpuResolveContact));
			job.m_cmd_list.CopyBufferRegion(r_contacts.get(), 0, upload);

			job.m_barriers.Transition(r_contacts.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
			job.m_barriers.Commit();
		}

		// Upload bodies
		{
			job.m_barriers.Transition(r_bodies.get(), D3D12_RESOURCE_STATE_COPY_DEST);
			job.m_barriers.Commit();

			auto upload = job.m_upload.Alloc<GpuRigidBody>(body_count);
			memcpy(upload.ptr<GpuRigidBody>(), bodies.data(), body_count * sizeof(GpuRigidBody));
			job.m_cmd_list.CopyBufferRegion(r_bodies.get(), 0, upload);

			job.m_barriers.Transition(r_bodies.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Commit();
		}

		// Upload dispatch args
		{
			job.m_barriers.Transition(r_dispatch.get(), D3D12_RESOURCE_STATE_COPY_DEST);
			job.m_barriers.Commit();

			auto upload = job.m_upload.Alloc<D3D12_DISPATCH_ARGUMENTS>(1);
			auto* args = upload.ptr<D3D12_DISPATCH_ARGUMENTS>();
			args->ThreadGroupCountX = static_cast<UINT>((contact_count + ResolveThreadCount - 1) / ResolveThreadCount);
			args->ThreadGroupCountY = 1;
			args->ThreadGroupCountZ = 1;
			job.m_cmd_list.CopyBufferRegion(r_dispatch.get(), 0, upload);

			job.m_barriers.Transition(r_dispatch.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Commit();
		}

		// Run the GPU resolve pipeline
		MaterialsDirty();
		Resolve(job, dt, body_count, body_count, contact_count, r_dispatch, r_counters, r_contacts, r_bodies, materials);

		// Readback bodies
		GpuReadbackBuffer::Allocation readback_bodies;
		{
			job.m_barriers.Transition(r_bodies.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
			job.m_barriers.Commit();

			readback_bodies = job.m_readback.Alloc<GpuRigidBody>(body_count);
			job.m_cmd_list.CopyBufferRegion(readback_bodies, r_bodies.get(), 0);

			job.m_barriers.Transition(r_bodies.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		}

		job.Run();

		memcpy(bodies.data(), readback_bodies.ptr<GpuRigidBody>(), body_count * sizeof(GpuRigidBody));
	}

	ID3D12Resource* GpuResolver::ContactOrder()
	{
		return m_r_contact_order.get();
	}

	// Readback bodies after GPU resolve (for CPU-side testing).
	void GpuResolver::Readback(GpuJob& job, D3DPtr<ID3D12Resource> r_bodies, std::span<GpuRigidBody> out_bodies)
	{
		auto body_count = static_cast<int>(out_bodies.size());
		if (body_count == 0)
			return;

		GpuReadbackBuffer::Allocation readback;

		{
			job.m_barriers.Transition(r_bodies.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
			job.m_barriers.Commit();

			readback = job.m_readback.Alloc<GpuRigidBody>(body_count);

			job.m_cmd_list.CopyBufferRegion(readback, r_bodies.get(), 0);
			job.m_barriers.Transition(r_bodies.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Commit();
		}

		// Execute the command list and wait for completion
		job.Run();

		// Read the results back to the CPU
		memcpy(out_bodies.data(), readback.ptr<GpuRigidBody>(), body_count * sizeof(GpuRigidBody));
	}

	// Custom deleter implementation (GpuResolver is complete here)
	void Deleter<GpuResolver>::operator()(GpuResolver* p) const
	{
		delete p;
	}
}
