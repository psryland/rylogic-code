//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2025
//*********************************************
#include "pr/physics/integrator/engine_config.h"
#include "src/compute/resolve_gpu.h"
#include "src/compute/physics_types.h"

namespace pr::physics
{
	using namespace pr::rdr12;

	// Constant buffer layout matching the HLSL cbResolve declaration.
	struct alignas(16) cbResolve
	{
		int g_max_contacts; // The max capacity of the contacts buffer
		int g_body_count;   // The number of bodies in the scene
		int g_colour;       // Current colour batch being processed (for CSResolve)
		int g_tgs_steps;    // Number of Temporal Gauss-Seidel substeps

		float g_dt;         // timestep in seconds
		float g_tgs_velocity_bias_max;
		float pad2;
		float pad3;

		float g_penetration_slop;
		float g_velocity_baumgarte;
		float g_deep_penetration_threshold;
		float g_deep_penetration_range;

		float g_deep_penetration_baumgarte_min;
		float g_deep_penetration_baumgarte_max;
		float pad4;
		float pad5;

		float g_position_slop;
		float g_position_baumgarte;
		float g_position_correction_scale;
		float pad6;
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
	};

	GpuResolver::GpuResolver(Gpu& gpu, EngineConfig const& config)
		: m_gpu(gpu)
		, m_config(config)
		, m_contact_sorter(gpu.m_gpu)
		, m_cs_compute_times()
		, m_cs_assign_colours()
		, m_cs_temporal_drift()
		, m_cs_position_solve()
		, m_cs_resolve()
		, m_cmd_sig()
		, m_r_materials()
		, m_r_colours()
		, m_r_contact_times()
		, m_r_contact_order()
		, m_max_materials()
		, m_max_contacts()
	{
		CompileShaders();

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

	// Compile the resolve compute shader from embedded resources.
	void GpuResolver::CompileShaders()
	{
		auto compiler = ShaderCompiler{}
			.Source(resource::Read<char>(L"src/compute/resolve.hlsl", L"TEXT"))
			.Includes({new ResourceIncludeHandler, true})
			.ShaderModel(L"cs_6_0")
			.Optimise();

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

			auto bytecode = compiler.EntryPoint(L"CSComputeCollisionTimes").Compile();

			m_cs_compute_times.m_sig = sig.Create(m_gpu, "Physics:ComputeTimesSig");
			m_cs_compute_times.m_pso = ComputePSO(m_cs_compute_times.m_sig.get(), bytecode).Create(m_gpu, "Physics:ComputeTimesPSO");
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

			auto bytecode = compiler.EntryPoint(L"CSAssignColours").Compile();

			m_cs_assign_colours.m_sig = sig.Create(m_gpu, "Physics:AssignColoursSig");
			m_cs_assign_colours.m_pso = ComputePSO(m_cs_assign_colours.m_sig.get(), bytecode).Create(m_gpu, "Physics:AssignColoursPSO");
		}

		// m_cs_temporal_drift
		{
			auto sig = RootSig(ERootSigFlags::ComputeOnly)
				.U32<cbResolve>(EReg::Params)
				.UAV(EReg::Bodies);

			auto bytecode = compiler.EntryPoint(L"CSTemporalDrift").Compile();

			m_cs_temporal_drift.m_sig = sig.Create(m_gpu, "Physics:TemporalDriftSig");
			m_cs_temporal_drift.m_pso = ComputePSO(m_cs_temporal_drift.m_sig.get(), bytecode).Create(m_gpu, "Physics:TemporalDriftPSO");
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

			auto bytecode = compiler.EntryPoint(L"CSPositionSolve").Compile();

			m_cs_position_solve.m_sig = sig.Create(m_gpu, "Physics:PositionSolveSig");
			m_cs_position_solve.m_pso = ComputePSO(m_cs_position_solve.m_sig.get(), bytecode).Create(m_gpu, "Physics:PositionSolvePSO");
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

			auto bytecode = compiler.EntryPoint(L"CSResolve").Compile();

			m_cs_resolve.m_sig = sig.Create(m_gpu, "Physics:ResolveSig");
			m_cs_resolve.m_pso = ComputePSO(m_cs_resolve.m_sig.get(), bytecode).Create(m_gpu, "Physics:ResolvePSO");
		}

	}

	// Create or grow GPU buffers for contacts and colour assignments.
	void GpuResolver::ResizeBuffers(CmdList& cmd_list, int max_contacts, int max_materials)
	{
		max_contacts = std::max(1, max_contacts);
		max_materials = std::max(1, max_materials);

		if (m_r_materials == nullptr || max_materials > m_max_materials)
		{
			m_r_materials = m_gpu.CreateResource(ResDesc::Buf<GpuMaterial>(max_materials, {}), cmd_list, "Physics:Materials");
			m_max_materials = max_materials;
		}
		if (m_r_colours == nullptr || m_max_contacts < max_contacts)
		{
			m_r_colours = m_gpu.CreateResource(ResDesc::Buf<uint32_t>(max_contacts, {}).usage(EUsage::UnorderedAccess), cmd_list, "Physics:ResolveColours");
			m_r_contact_times = m_gpu.CreateResource(ResDesc::Buf<float>(max_contacts, {}).usage(EUsage::UnorderedAccess), cmd_list, "Physics:ContactTimes");
			m_r_contact_order = m_gpu.CreateResource(ResDesc::Buf<uint32_t>(max_contacts, {}).usage(EUsage::UnorderedAccess), cmd_list, "Physics:ContactOrder");
			m_max_contacts = max_contacts;
		}
	}

	// Resolve collisions on the GPU using graph-coloured batches.
	void GpuResolver::Resolve(GpuJob& job, float dt, int body_count, int max_contacts, D3DPtr<ID3D12Resource> dispatch, D3DPtr<ID3D12Resource> counters, D3DPtr<ID3D12Resource> contacts, D3DPtr<ID3D12Resource> bodies, std::span<GpuMaterial const> materials)
	{
		auto material_count = static_cast<int>(materials.size());
		pix::BeginEvent(job.m_cmd_list.get(), 0xFF6799Ab, "Physics::Resolve");

		ResizeBuffers(job.m_cmd_list, max_contacts, material_count);

		assert(m_config.position_iterations >= 0);
		assert(m_config.tgs_steps >= 1);
		auto const position_iterations = std::max(0, m_config.position_iterations);
		auto const tgs_steps = std::max(1, m_config.tgs_steps);
		auto const position_correction_scale = position_iterations != 0 ? 1.0f / position_iterations : 0.0f;

		cbResolve cb_resolve = {
			.g_max_contacts = max_contacts,
			.g_body_count = body_count,
			.g_colour = 0,
			.g_tgs_steps = tgs_steps,
			.g_dt = dt,
			.g_tgs_velocity_bias_max = m_config.tgs_velocity_bias_max,
			.pad2 = 0,
			.pad3 = 0,
			.g_penetration_slop = m_config.penetration_slop,
			.g_velocity_baumgarte = m_config.velocity_baumgarte,
			.g_deep_penetration_threshold = m_config.deep_penetration_threshold,
			.g_deep_penetration_range = m_config.deep_penetration_range,
			.g_deep_penetration_baumgarte_min = m_config.deep_penetration_baumgarte_min,
			.g_deep_penetration_baumgarte_max = m_config.deep_penetration_baumgarte_max,
			.pad4 = 0,
			.pad5 = 0,
			.g_position_slop = m_config.position_slop,
			.g_position_baumgarte = m_config.position_baumgarte,
			.g_position_correction_scale = position_correction_scale,
			.pad6 = 0,
		};

		// Upload materials (small buffer, upload every frame for simplicity)
		{
			job.m_barriers.Transition(m_r_materials.get(), D3D12_RESOURCE_STATE_COPY_DEST);
			job.m_barriers.Commit();

			auto mat_upload = job.m_upload.Alloc<GpuMaterial>(std::max(1, material_count));
			memcpy(mat_upload.ptr<GpuMaterial>(), materials.data(), material_count * sizeof(GpuMaterial));
			job.m_cmd_list.CopyBufferRegion(m_r_materials.get(), 0, mat_upload);

			job.m_barriers.Transition(m_r_materials.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
			job.m_barriers.Commit();
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
			job.m_barriers.Commit();
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

			// The collide shader's 'CSCalcResolveDispatch' function ensures that there will always be at least
			// one thread group dispatched, even if contact_count is 0. This ensures the 'colour_used' and
			// contact times are always initialised.
			job.m_cmd_list.ExecuteIndirect(m_cmd_sig.get(), 1, dispatch.get());

			job.m_barriers.UAV(bodies.get());
			job.m_barriers.UAV(contacts.get());
			job.m_barriers.UAV(m_r_contact_times.get());
			job.m_barriers.UAV(m_r_contact_order.get());
			job.m_barriers.Commit();
		}

		// Create the sorted contact order based on contact time
		{
			m_contact_sorter.Bind(job.m_cmd_list, max_contacts, m_r_contact_times, m_r_contact_order);
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

		assert(m_config.solver_iterations >= 0);
		auto const solver_iterations = std::max(0, m_config.solver_iterations);
		auto const body_dispatch_count = static_cast<UINT>(std::max(1, (body_count + ResolveThreadCount - 1) / ResolveThreadCount));
		auto const tgs_dt = dt / static_cast<float>(tgs_steps);
		auto const position_iterations_per_tgs = position_iterations != 0 ? std::max(1, (position_iterations + tgs_steps - 1) / tgs_steps) : 0;
		auto const solver_iterations_per_tgs = solver_iterations != 0 ? std::max(1, (solver_iterations + tgs_steps - 1) / tgs_steps) : 0;

		// Temporal Gauss-Seidel starts from an approximate pre-integrated pose, then advances and solves in smaller
		// temporal slices while reusing the contact anchors from the collision pass.
		if (tgs_steps != 1)
		{
			cb_resolve.g_dt = -dt;
			job.m_cmd_list.SetPipelineState(m_cs_temporal_drift.m_pso.get());
			job.m_cmd_list.SetComputeRootSignature(m_cs_temporal_drift.m_sig.get());
			job.m_cmd_list.AddComputeRoot32BitConstants(cb_resolve);
			job.m_cmd_list.AddComputeRootUnorderedAccessView(bodies->GetGPUVirtualAddress());
			job.m_cmd_list.Dispatch(body_dispatch_count, 1, 1);
			job.m_barriers.UAV(bodies.get());
			job.m_barriers.Commit();
		}

		for (int tgs_step = 0; tgs_step != tgs_steps; ++tgs_step)
		{
			if (tgs_steps != 1)
			{
				cb_resolve.g_dt = tgs_dt;
				job.m_cmd_list.SetPipelineState(m_cs_temporal_drift.m_pso.get());
				job.m_cmd_list.SetComputeRootSignature(m_cs_temporal_drift.m_sig.get());
				job.m_cmd_list.AddComputeRoot32BitConstants(cb_resolve);
				job.m_cmd_list.AddComputeRootUnorderedAccessView(bodies->GetGPUVirtualAddress());
				job.m_cmd_list.Dispatch(body_dispatch_count, 1, 1);
				job.m_barriers.UAV(bodies.get());
				job.m_barriers.Commit();
			}

			cb_resolve.g_dt = tgs_dt;
			cb_resolve.g_colour = 0;

			// Split position correction in colour batches. TGS refreshes each contact's effective depth from current transforms, so the
			// correction is split across position iterations for this temporal slice rather than amortised across all TGS steps.
			if (position_iterations_per_tgs != 0)
			{
				job.m_cmd_list.SetPipelineState(m_cs_position_solve.m_pso.get());
				job.m_cmd_list.SetComputeRootSignature(m_cs_position_solve.m_sig.get());
				job.m_cmd_list.AddComputeRoot32BitConstants(cb_resolve);
				job.m_cmd_list.AddComputeRootShaderResourceView(counters->GetGPUVirtualAddress());
				job.m_cmd_list.AddComputeRootUnorderedAccessView(bodies->GetGPUVirtualAddress());
				job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_colours->GetGPUVirtualAddress());
				job.m_cmd_list.AddComputeRootUnorderedAccessView(contacts->GetGPUVirtualAddress());
				job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_contact_order->GetGPUVirtualAddress());

				for (int iter = 0; iter != position_iterations_per_tgs; ++iter)
				{
					for (int colour = 0; colour != MaxColours; ++colour)
					{
						cb_resolve.g_colour = colour;
						job.m_cmd_list.SetComputeRoot32BitConstants(0, cb_resolve);
						job.m_cmd_list.ExecuteIndirect(m_cmd_sig.get(), 1, dispatch.get());

						job.m_barriers.UAV(bodies.get());
						job.m_barriers.Commit();
					}
				}
				cb_resolve.g_colour = 0;
			}

			// Velocity resolve each colour batch.
			// Multiple solver iterations (Gauss-Seidel) allow stacked contacts to converge.
			// Each iteration sweeps all colour batches, re-reading body momenta updated by prior contacts.
			// The energy guard in CSResolve prevents energy injection across iterations.
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

				for (int iter = 0; iter != solver_iterations_per_tgs; ++iter)
				{
					for (int colour = 0; colour != MaxColours; ++colour)
					{
						cb_resolve.g_colour = colour;
						job.m_cmd_list.SetComputeRoot32BitConstants(0, cb_resolve);
						job.m_cmd_list.ExecuteIndirect(m_cmd_sig.get(), 1, dispatch.get());

						job.m_barriers.UAV(bodies.get());
						job.m_barriers.Commit();
					}
				}
				cb_resolve.g_colour = 0;
			}
		}

		pix::EndEvent(job.m_cmd_list.get());
	}

	// CPU-side testing: run the GPU contact-time shader and radix sorter, then read back the sorted contact order.
	void GpuResolver::SortContacts(GpuJob& job, float dt, std::span<GpuResolveContact const> contacts, std::span<GpuRigidBody> bodies, std::span<uint32_t> out_order, std::span<float> out_times)
	{
		auto contact_count = static_cast<int>(contacts.size());
		auto body_count = static_cast<int>(bodies.size());
		if (contact_count == 0 || body_count == 0)
			return;
		if (std::ssize(out_order) < contact_count || std::ssize(out_times) < contact_count)
			throw std::invalid_argument("SortContacts output buffers are too small");

		auto r_counters = m_gpu.CreateResource(ResDesc::Buf<GpuCollisionCounters>(1, {}), job.m_cmd_list, "Physics:SortTestCounters");
		auto sort_count = 16;
		while (sort_count < contact_count)
			sort_count *= 2;

		auto r_contacts = m_gpu.CreateResource(ResDesc::Buf<GpuResolveContact>(contact_count, {}).usage(EUsage::UnorderedAccess), job.m_cmd_list, "Physics:SortTestContacts");
		auto r_bodies = m_gpu.CreateResource(ResDesc::Buf<GpuRigidBody>(body_count, {}).usage(EUsage::UnorderedAccess), job.m_cmd_list, "Physics:SortTestBodies");
		auto r_dispatch = m_gpu.CreateResource(ResDesc::Buf<D3D12_DISPATCH_ARGUMENTS>(1, {}).usage(EUsage::UnorderedAccess), job.m_cmd_list, "Physics:SortTestDispatch");

		{
			job.m_barriers.Transition(r_counters.get(), D3D12_RESOURCE_STATE_COPY_DEST);
			job.m_barriers.Transition(r_contacts.get(), D3D12_RESOURCE_STATE_COPY_DEST);
			job.m_barriers.Transition(r_bodies.get(), D3D12_RESOURCE_STATE_COPY_DEST);
			job.m_barriers.Transition(r_dispatch.get(), D3D12_RESOURCE_STATE_COPY_DEST);
			job.m_barriers.Commit();

			auto counters_upload = job.m_upload.Alloc<GpuCollisionCounters>(1);
			*counters_upload.ptr<GpuCollisionCounters>() = GpuCollisionCounters{
				.pair_count = 0,
				.contact_count = contact_count,
			};
			job.m_cmd_list.CopyBufferRegion(r_counters.get(), 0, counters_upload);

			auto contacts_upload = job.m_upload.Alloc<GpuResolveContact>(contact_count);
			memcpy(contacts_upload.ptr<GpuResolveContact>(), contacts.data(), contact_count * sizeof(GpuResolveContact));
			job.m_cmd_list.CopyBufferRegion(r_contacts.get(), 0, contacts_upload);

			auto bodies_upload = job.m_upload.Alloc<GpuRigidBody>(body_count);
			memcpy(bodies_upload.ptr<GpuRigidBody>(), bodies.data(), body_count * sizeof(GpuRigidBody));
			job.m_cmd_list.CopyBufferRegion(r_bodies.get(), 0, bodies_upload);

			auto dispatch_upload = job.m_upload.Alloc<D3D12_DISPATCH_ARGUMENTS>(1);
			auto* args = dispatch_upload.ptr<D3D12_DISPATCH_ARGUMENTS>();
			args->ThreadGroupCountX = static_cast<UINT>((contact_count + ResolveThreadCount - 1) / ResolveThreadCount);
			args->ThreadGroupCountY = 1;
			args->ThreadGroupCountZ = 1;
			job.m_cmd_list.CopyBufferRegion(r_dispatch.get(), 0, dispatch_upload);
		}

		ResizeBuffers(job.m_cmd_list, sort_count, 1);

		auto cb_resolve = cbResolve{
			.g_max_contacts = sort_count,
			.g_body_count = body_count,
			.g_colour = 0,
			.g_tgs_steps = std::max(1, m_config.tgs_steps),
			.g_dt = dt,
			.g_tgs_velocity_bias_max = m_config.tgs_velocity_bias_max,
			.pad2 = 0,
			.pad3 = 0,
			.g_penetration_slop = m_config.penetration_slop,
			.g_velocity_baumgarte = m_config.velocity_baumgarte,
			.g_deep_penetration_threshold = m_config.deep_penetration_threshold,
			.g_deep_penetration_range = m_config.deep_penetration_range,
			.g_deep_penetration_baumgarte_min = m_config.deep_penetration_baumgarte_min,
			.g_deep_penetration_baumgarte_max = m_config.deep_penetration_baumgarte_max,
			.pad4 = 0,
			.pad5 = 0,
			.g_position_slop = m_config.position_slop,
			.g_position_baumgarte = m_config.position_baumgarte,
			.g_position_correction_scale = 1.0f,
			.pad6 = 0,
		};

		{
			job.m_barriers.Transition(r_dispatch.get(), D3D12_RESOURCE_STATE_INDIRECT_ARGUMENT);
			job.m_barriers.Transition(r_counters.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
			job.m_barriers.Transition(r_contacts.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Transition(r_bodies.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Transition(m_r_contact_times.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Transition(m_r_contact_order.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Commit();
		}
		{
			job.m_cmd_list.SetPipelineState(m_cs_compute_times.m_pso.get());
			job.m_cmd_list.SetComputeRootSignature(m_cs_compute_times.m_sig.get());
			job.m_cmd_list.AddComputeRoot32BitConstants(cb_resolve);
			job.m_cmd_list.AddComputeRootShaderResourceView(r_counters->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(r_bodies->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(r_contacts->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_contact_times->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_contact_order->GetGPUVirtualAddress());
			job.m_cmd_list.ExecuteIndirect(m_cmd_sig.get(), 1, r_dispatch.get());

			job.m_barriers.UAV(r_bodies.get());
			job.m_barriers.UAV(r_contacts.get());
			job.m_barriers.UAV(m_r_contact_times.get());
			job.m_barriers.UAV(m_r_contact_order.get());
			job.m_barriers.Commit();
		}
		{
			m_contact_sorter.Bind(job.m_cmd_list, sort_count, m_r_contact_times, m_r_contact_order);
			m_contact_sorter.Sort(job.m_cmd_list);

			job.m_barriers.UAV(m_r_contact_times.get());
			job.m_barriers.UAV(m_r_contact_order.get());
			job.m_barriers.Commit();
		}

		auto readback_times = ReadbackAlloc{};
		auto readback_order = ReadbackAlloc{};
		{
			job.m_barriers.Transition(m_r_contact_times.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
			job.m_barriers.Transition(m_r_contact_order.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
			job.m_barriers.Commit();

			readback_times = job.m_readback.Alloc<float>(contact_count);
			readback_order = job.m_readback.Alloc<uint32_t>(contact_count);
			job.m_cmd_list.CopyBufferRegion(readback_times, m_r_contact_times.get(), 0);
			job.m_cmd_list.CopyBufferRegion(readback_order, m_r_contact_order.get(), 0);

			job.m_barriers.Transition(m_r_contact_times.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Transition(m_r_contact_order.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Commit();
		}

		job.Run();

		memcpy(out_times.data(), readback_times.ptr<float>(), contact_count * sizeof(float));
		memcpy(out_order.data(), readback_order.ptr<uint32_t>(), contact_count * sizeof(uint32_t));
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










#if 0

	// Greedy graph colouring: assign colours so no two contacts sharing a body have the same colour.
	// Uses per-body colour tracking to avoid the O(n²) scan of all earlier contacts.
	// Complexity: O(n × k) where k is the max colour count (typically small, ~4-8).
	std::pair<std::vector<int>, int> GraphColourContacts(std::span<GpuResolveContact const> contacts)
	{
		auto n = static_cast<int>(contacts.size());
		std::vector<int> colours(n, -1);
		int max_colour = 0;

		// Map from body_index → bitset of colours already used by that body's contacts.
		// This lets us find conflicting colours in O(k) instead of scanning all earlier contacts.
		std::unordered_map<int, std::vector<bool>> body_used;

		for (int i = 0; i != n; ++i)
		{
			auto a = contacts[i].body_idx_a;
			auto b = contacts[i].body_idx_b;

			auto& ua = body_used[a];
			auto& ub = body_used[b];

			// Find the lowest colour not used by either body
			auto limit = static_cast<int>(std::max(ua.size(), ub.size()));
			int c = 0;
			for (; c < limit; ++c)
			{
				auto used_a = c < static_cast<int>(ua.size()) && ua[c];
				auto used_b = c < static_cast<int>(ub.size()) && ub[c];
				if (!used_a && !used_b)
					break;
			}

			colours[i] = c;
			max_colour = std::max(max_colour, c + 1);

			// Mark this colour as used for both bodies
			if (c >= static_cast<int>(ua.size())) ua.resize(c + 1, false);
			if (c >= static_cast<int>(ub.size())) ub.resize(c + 1, false);
			ua[c] = true;
			ub[c] = true;
		}

		return {colours, max_colour};
	}


		// Upload contacts
		{
			job.m_barriers.Transition(m_r_contacts.get(), D3D12_RESOURCE_STATE_COPY_DEST);
			job.m_barriers.Commit();

			auto upload = job.m_upload.Alloc<GpuResolveContact>(contact_count);
			memcpy(upload.ptr<GpuResolveContact>(), contacts.data(), contact_count * sizeof(GpuResolveContact));
			job.m_cmd_list.CopyBufferRegion(m_r_contacts.get(), 0, upload);

			job.m_barriers.Transition(m_r_contacts.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
			job.m_barriers.Commit();
		}

		// Upload colours (convert int→uint32_t for GPU)
		{
			job.m_barriers.Transition(m_r_colours.get(), D3D12_RESOURCE_STATE_COPY_DEST);
			job.m_barriers.Commit();

			auto upload = job.m_upload.Alloc<uint32_t>(contact_count);
			auto* dst = upload.ptr<uint32_t>();
			for (int i = 0; i != contact_count; ++i)
				dst[i] = static_cast<uint32_t>(colours[i]);
			job.m_cmd_list.CopyBufferRegion(m_r_colours.get(), 0, upload);

			job.m_barriers.Transition(m_r_colours.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
			job.m_barriers.Commit();
		}
#endif
