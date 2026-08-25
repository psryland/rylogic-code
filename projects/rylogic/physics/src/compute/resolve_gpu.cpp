//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2025
//*********************************************
#include "pr/physics/integrator/engine_config.h"
#include "src/compute/resolve_gpu.h"
#include "src/compute/constraint_solver_gpu.h"
#include "src/compute/coupled_constraint_solver_gpu.h"
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
		int shock_padding0;
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
		int pad_i0;
		int pad_i1;
		int pad_i2;
	};
	static_assert((sizeof(cbResolve) & 0xf) == 0);

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
				.UAV(EReg::ContactOrder);

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

	// Resolve collisions on the GPU using graph-coloured batches.
	void GpuResolver::Resolve(GpuJob& job, float dt, int body_count, int max_contacts, D3DPtr<ID3D12Resource> dispatch, D3DPtr<ID3D12Resource> counters, D3DPtr<ID3D12Resource> contacts, D3DPtr<ID3D12Resource> bodies, std::span<GpuMaterial const> materials, float bias_scale, int solver_iterations_, int push_out_iterations, float restitution_scale, bool support_only, GpuConstraintSolver* constraint_solver, GpuCoupledConstraintSolver* coupled_constraint_solver, bool retain_constraint_impulses)
	{
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
			.shock_padding0 = 0,
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
			.pad_i0 = 0,
			.pad_i1 = 0,
			.pad_i2 = 0,
		};
		if (m_config.warm_start_scale <= 0.0f)
			m_reset_warm_start_cache = true;

		// Upload materials only when the CPU material map changes or the GPU buffer grows.
		// Main and selective resolvers have separate GPU buffers, so each tracks this independently.
		if (m_materials_dirty)
		{
			job.m_barriers.Transition(m_r_materials.get(), D3D12_RESOURCE_STATE_COPY_DEST);
			job.m_barriers.Commit();

			auto mat_upload = job.m_upload.Alloc<GpuMaterial>(std::max(1, material_count));
			memcpy(mat_upload.ptr<GpuMaterial>(), materials.data(), material_count * sizeof(GpuMaterial));
			job.m_cmd_list.CopyBufferRegion(m_r_materials.get(), 0, mat_upload);

			job.m_barriers.Transition(m_r_materials.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
			job.m_barriers.Commit();
			m_materials_dirty = false;
		}

		// Switch states for resources
		{
			job.m_barriers.Transition(dispatch.get(), D3D12_RESOURCE_STATE_INDIRECT_ARGUMENT);
			job.m_barriers.Transition(counters.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
			job.m_barriers.Transition(m_r_materials.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
			job.m_barriers.Transition(bodies.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Transition(m_r_colours.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Transition(contacts.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Transition(m_r_contact_times.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Transition(m_r_contact_order.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Transition(m_r_body_contact_head.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Transition(m_r_contact_next_a.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Transition(m_r_contact_next_b.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Transition(m_r_warm_start_prev.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Transition(m_r_warm_start_curr.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Commit();
		}

		auto bind_warm_start_step = [&](ComputeStep& step, ID3D12Resource* warm_start_curr)
		{
			job.m_cmd_list.SetPipelineState(step.m_pso.get());
			job.m_cmd_list.SetComputeRootSignature(step.m_sig.get());
			job.m_cmd_list.AddComputeRoot32BitConstants(cb_resolve);
			job.m_cmd_list.AddComputeRootShaderResourceView(counters->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(bodies->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_colours->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(contacts->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_contact_order->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_warm_start_prev->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(warm_start_curr->GetGPUVirtualAddress());
		};
		auto commit_warm_start_barriers = [&]
		{
			job.m_barriers.UAV(bodies.get());
			job.m_barriers.UAV(contacts.get());
			job.m_barriers.UAV(m_r_warm_start_prev.get());
			job.m_barriers.UAV(m_r_warm_start_curr.get());
			job.m_barriers.Commit();
		};

		// Clear the current cache every frame. Newly allocated previous caches are cleared once so the first lookup is deterministic.
		{
			auto const warm_start_group_count = static_cast<UINT>(std::max(1, (m_warm_start_capacity + ResolveThreadCount - 1) / ResolveThreadCount));
			if (m_reset_warm_start_cache)
			{
				bind_warm_start_step(m_cs_warm_start_clear, m_r_warm_start_prev.get());
				job.m_cmd_list.Dispatch(warm_start_group_count, 1, 1);
				commit_warm_start_barriers();
				m_reset_warm_start_cache = false;
			}

			bind_warm_start_step(m_cs_warm_start_clear, m_r_warm_start_curr.get());
			job.m_cmd_list.Dispatch(warm_start_group_count, 1, 1);
			commit_warm_start_barriers();
		}

		// Calculate the contact times (biased by gravity) and zero the body colour_used bitmasks.
		{
			job.m_cmd_list.SetPipelineState(m_cs_compute_times.m_pso.get());
			job.m_cmd_list.SetComputeRootSignature(m_cs_compute_times.m_sig.get());
			job.m_cmd_list.AddComputeRoot32BitConstants(cb_resolve);
			job.m_cmd_list.AddComputeRootShaderResourceView(counters->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(bodies->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(contacts->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_contact_times->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_contact_order->GetGPUVirtualAddress());

			// The collide shader's 'CSCalcResolveDispatch' function ensures that there will always be at least one thread
			// group dispatched, even if contact_count is 0. This ensures the 'colour_used' and contact times are always initialised.
			job.m_cmd_list.ExecuteIndirect(m_cmd_sig.get(), 1, dispatch.get());

			job.m_barriers.UAV(bodies.get());
			job.m_barriers.UAV(contacts.get());
			job.m_barriers.UAV(m_r_contact_times.get());
			job.m_barriers.UAV(m_r_contact_order.get());
			job.m_barriers.Commit();
		}

		// Propagate contact priority through the contact graph and fold it into the sort key.
		if (priority_sort_enabled)
		{
			auto bind_shock_step = [&](ComputeStep& step)
			{
				job.m_cmd_list.SetPipelineState(step.m_pso.get());
				job.m_cmd_list.SetComputeRootSignature(step.m_sig.get());
				job.m_cmd_list.AddComputeRoot32BitConstants(cb_resolve);
				job.m_cmd_list.AddComputeRootShaderResourceView(counters->GetGPUVirtualAddress());
				job.m_cmd_list.AddComputeRootUnorderedAccessView(bodies->GetGPUVirtualAddress());
				job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_colours->GetGPUVirtualAddress());
				job.m_cmd_list.AddComputeRootUnorderedAccessView(contacts->GetGPUVirtualAddress());
				job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_contact_times->GetGPUVirtualAddress());
				job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_contact_order->GetGPUVirtualAddress());
				job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_body_contact_head->GetGPUVirtualAddress());
				job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_contact_next_a->GetGPUVirtualAddress());
				job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_contact_next_b->GetGPUVirtualAddress());
			};
			auto commit_shock_barriers = [&]
			{
				job.m_barriers.UAV(m_r_colours.get());
				job.m_barriers.UAV(m_r_contact_times.get());
				job.m_barriers.UAV(m_r_contact_order.get());
				job.m_barriers.UAV(m_r_body_contact_head.get());
				job.m_barriers.UAV(m_r_contact_next_a.get());
				job.m_barriers.UAV(m_r_contact_next_b.get());
				job.m_barriers.Commit();
			};
			auto const body_group_count = static_cast<UINT>(std::max(1, (body_count + ResolveThreadCount - 1) / ResolveThreadCount));

			bind_shock_step(m_cs_clear_shock_lists);
			job.m_cmd_list.Dispatch(body_group_count, 1, 1);
			commit_shock_barriers();

			bind_shock_step(m_cs_seed_shock_priority);
			job.m_cmd_list.ExecuteIndirect(m_cmd_sig.get(), 1, dispatch.get());
			commit_shock_barriers();

			for (int iter = 0; iter != cb_resolve.shock_iterations; ++iter)
			{
				bind_shock_step(m_cs_propagate_shock_priority);
				job.m_cmd_list.ExecuteIndirect(m_cmd_sig.get(), 1, dispatch.get());
				commit_shock_barriers();

				bind_shock_step(m_cs_commit_shock_priority);
				job.m_cmd_list.ExecuteIndirect(m_cmd_sig.get(), 1, dispatch.get());
				commit_shock_barriers();
			}

			bind_shock_step(m_cs_finalize_shock_priority);
			job.m_cmd_list.ExecuteIndirect(m_cmd_sig.get(), 1, dispatch.get());
			commit_shock_barriers();

			job.m_barriers.UAV(contacts.get());
			job.m_barriers.Commit();
		}

		// Create the sorted contact order based on contact time
		{
			m_contact_sorter.Bind(job.m_cmd_list, m_max_contacts, m_r_contact_times, m_r_contact_order);
			m_contact_sorter.Sort(job.m_cmd_list);

			job.m_barriers.UAV(m_r_contact_times.get());
			job.m_barriers.UAV(m_r_contact_order.get());
			job.m_barriers.Commit();
		}

		// Greedy graph colouring on sorted contacts.
		{
			job.m_cmd_list.SetPipelineState(m_cs_assign_colours.m_pso.get());
			job.m_cmd_list.SetComputeRootSignature(m_cs_assign_colours.m_sig.get());
			job.m_cmd_list.AddComputeRoot32BitConstants(cb_resolve);
			job.m_cmd_list.AddComputeRootShaderResourceView(counters->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(bodies->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_colours->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(contacts->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_contact_order->GetGPUVirtualAddress());

			job.m_cmd_list.Dispatch(1, 1, 1);

			job.m_barriers.UAV(bodies.get());
			job.m_barriers.UAV(m_r_colours.get());
			job.m_barriers.Commit();
		}

		// Compile and independently colour persistent D6 blocks after integration has produced the current body transforms.
		if (constraint_solver != nullptr)
			constraint_solver->Prepare(job, dt, body_count, bodies, retain_constraint_impulses);
		if (coupled_constraint_solver != nullptr)
			coupled_constraint_solver->PrepareVelocity(job, dt, body_count, bodies.get(), retain_constraint_impulses);

		// Apply cached physical impulses before the iterative solves so resting stacks start close to last frame's support solution.
		if (m_config.warm_start_scale > 0.0f)
		{
			for (int colour = 0; colour != MaxColours; ++colour)
			{
				cb_resolve.colour = colour;
				bind_warm_start_step(m_cs_apply_warm_start, m_r_warm_start_curr.get());
				job.m_cmd_list.SetComputeRoot32BitConstants(0, cb_resolve);
				job.m_cmd_list.ExecuteIndirect(m_cmd_sig.get(), 1, dispatch.get());
				commit_warm_start_barriers();
			}
			cb_resolve.colour = 0;
		}
		if (!retain_constraint_impulses)
		{
			if (constraint_solver != nullptr)
				constraint_solver->ApplyWarmStart(job, dt, body_count, bodies);
			if (coupled_constraint_solver != nullptr)
				coupled_constraint_solver->ApplyWarmStart(job, body_count, bodies.get());
		}

		// Keep the long-established contact-only ordering unchanged while coupled rows use velocity-first fixed-configuration solving.
		auto const has_constraint_work = constraint_solver != nullptr || coupled_constraint_solver != nullptr;
		auto solve_position = [&]
		{
			auto const coupled_position_active =
				coupled_constraint_solver != nullptr &&
				coupled_constraint_solver->PreparePosition(job, dt, body_count, bodies.get());

			// Split position correction in colour batches.
			if (push_out_steps != 0)
			{
				// The contact depths are from the collision pass, so the correction is split across iterations rather than re-applying the full depth each sweep.
				auto bind_position_solve = [&]
				{
					job.m_cmd_list.SetPipelineState(m_cs_position_solve.m_pso.get());
					job.m_cmd_list.SetComputeRootSignature(m_cs_position_solve.m_sig.get());
					job.m_cmd_list.AddComputeRoot32BitConstants(cb_resolve);
					job.m_cmd_list.AddComputeRootShaderResourceView(counters->GetGPUVirtualAddress());
					job.m_cmd_list.AddComputeRootUnorderedAccessView(bodies->GetGPUVirtualAddress());
					job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_colours->GetGPUVirtualAddress());
					job.m_cmd_list.AddComputeRootUnorderedAccessView(contacts->GetGPUVirtualAddress());
					job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_contact_order->GetGPUVirtualAddress());
				};
				bind_position_solve();

				for (int iter = 0; iter != push_out_steps; ++iter)
				{
					// A preceding constraint sweep leaves its root signature active, so restore every contact root binding before resuming.
					if (iter != 0 && has_constraint_work)
						bind_position_solve();

					for (int colour = 0; colour != MaxColours; ++colour)
					{
						cb_resolve.colour = colour;
						job.m_cmd_list.SetComputeRoot32BitConstants(0, cb_resolve);
						job.m_cmd_list.ExecuteIndirect(m_cmd_sig.get(), 1, dispatch.get());

						job.m_barriers.UAV(bodies.get());
						job.m_barriers.Commit();
					}
					if (constraint_solver != nullptr)
						constraint_solver->SolvePositionIteration(job, dt, body_count, push_out_steps, bodies);
					if (coupled_position_active)
						coupled_constraint_solver->SolvePositionIteration(job, bodies.get());
				}
				cb_resolve.colour = 0;
			}

			// Coupled application owns the shared rigid pseudo stream, preventing independent correction from being integrated twice.
			if (coupled_position_active)
				coupled_constraint_solver->ApplyPosition(job, bodies.get());
			else if (constraint_solver != nullptr)
				constraint_solver->ApplyPosition(job, dt, body_count, push_out_steps, bodies);
		};

		auto solve_velocity = [&]
		{
			// Multiple solver iterations (Gauss-Seidel) allow stacked contacts to converge.
			// Each iteration sweeps all colour batches, re-reading body momenta updated by prior contacts.
			// The energy guard in CSResolve prevents energy injection across iterations.
			auto bind_velocity_solve = [&]
			{
				job.m_cmd_list.SetPipelineState(m_cs_resolve.m_pso.get());
				job.m_cmd_list.SetComputeRootSignature(m_cs_resolve.m_sig.get());
				job.m_cmd_list.AddComputeRoot32BitConstants(cb_resolve);
				job.m_cmd_list.AddComputeRootShaderResourceView(counters->GetGPUVirtualAddress());
				job.m_cmd_list.AddComputeRootShaderResourceView(m_r_materials->GetGPUVirtualAddress());
				job.m_cmd_list.AddComputeRootUnorderedAccessView(bodies->GetGPUVirtualAddress());
				job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_colours->GetGPUVirtualAddress());
				job.m_cmd_list.AddComputeRootUnorderedAccessView(contacts->GetGPUVirtualAddress());
				job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_contact_order->GetGPUVirtualAddress());
			};
			bind_velocity_solve();

			for (int iter = 0; iter != solver_iterations; ++iter)
			{
				// Constraint velocity sweeps use a separate root layout and require the contact table to be rebound on the next outer sweep.
				if (iter != 0 && has_constraint_work)
					bind_velocity_solve();

				for (int colour = 0; colour != MaxColours; ++colour)
				{
					cb_resolve.colour = colour;
					job.m_cmd_list.SetComputeRoot32BitConstants(0, cb_resolve);
					job.m_cmd_list.ExecuteIndirect(m_cmd_sig.get(), 1, dispatch.get());

					job.m_barriers.UAV(bodies.get());
					job.m_barriers.Commit();
				}
				if (constraint_solver != nullptr)
					constraint_solver->SolveVelocityIteration(job, dt, body_count, bodies);
				if (coupled_constraint_solver != nullptr)
					coupled_constraint_solver->SolveVelocityIteration(job, body_count, bodies.get());
			}
			cb_resolve.colour = 0;
		};

		if (coupled_constraint_solver != nullptr)
		{
			// Physical impulses establish the fixed-configuration state before any detached coordinate correction.
			solve_velocity();
			solve_position();
		}
		else
		{
			solve_position();
			solve_velocity();
		}

		// Persist accumulated physical impulses for the next frame and then swap the cache roles.
		if (m_config.warm_start_scale > 0.0f)
		{
			bind_warm_start_step(m_cs_store_warm_start, m_r_warm_start_curr.get());
			job.m_cmd_list.ExecuteIndirect(m_cmd_sig.get(), 1, dispatch.get());
			commit_warm_start_barriers();
			std::swap(m_r_warm_start_prev, m_r_warm_start_curr);
		}

		pix::EndEvent(job.m_cmd_list.get());
	}

	// Mark the material buffer dirty so it is re-uploaded on the next resolve.
	void GpuResolver::MaterialsDirty()
	{
		m_materials_dirty = true;
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
		Resolve(job, dt, body_count, contact_count, r_dispatch, r_counters, r_contacts, r_bodies, materials);

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
