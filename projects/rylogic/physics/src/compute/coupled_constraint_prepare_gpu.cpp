//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "pr/physics/integrator/engine_config.h"
#include "src/compute/articulation_mobility_gpu.h"
#include "src/compute/coupled_constraint_prepare_gpu.h"
#include "src/compute/shader_code.h"

namespace pr::physics
{
	using namespace ::pr::compute;

	namespace
	{
		// Packed bounds and numerical controls shared with coupled_constraint_prepare.hlsl.
		struct alignas(16) cbCoupledConstraintPrepare
		{
			int slot_count;
			int body_count;
			int link_count;
			int mobility_count;
			float timestep;
			float regularization;
			float warm_start_scale;
			float pad0;
		};
		static_assert(sizeof(cbCoupledConstraintPrepare) == 32);

		// Root-register assignments for the coupled preparation kernel.
		struct EReg
		{
			inline static constexpr auto Params = ECBufReg::b0;
			inline static constexpr auto Bodies = EUAVReg::u0;
			inline static constexpr auto ConstraintEndpoints = ESRVReg::t0;
			inline static constexpr auto Descriptors = ESRVReg::t1;
			inline static constexpr auto CoupledEndpoints = ESRVReg::t2;
			inline static constexpr auto LinkToWorld = ESRVReg::t3;
			inline static constexpr auto Mobilities = ESRVReg::t4;
			inline static constexpr auto AbaScratch = ESRVReg::t5;
			inline static constexpr auto Blocks = EUAVReg::u1;
			inline static constexpr auto Rows = EUAVReg::u2;
			inline static constexpr auto Preconditioners = EUAVReg::u3;
		};

		// Create and initialize one diagnostic input buffer, retaining a valid one-element sentinel for empty rigid-body input.
		template <typename Type>
		D3DPtr<ID3D12Resource> CreateCoupledInputResource(Gpu& gpu, GpuJob& job, std::span<Type const> values, EUsage usage, D3D12_RESOURCE_STATES final_state, std::string_view name)
		{
			auto const count = std::max(1, isize(values));
			auto resource = gpu.CreateResource(ResDesc::Buf<Type>(count, {}).usage(usage), job.m_cmd_list, name);
			job.m_barriers.Transition(resource.get(), D3D12_RESOURCE_STATE_COPY_DEST).Commit();
			auto allocation = job.m_upload.Alloc<Type>(count);
			std::fill_n(allocation.ptr<Type>(), count, Type{});
			if (!values.empty())
				memcpy(allocation.ptr<Type>(), values.data(), values.size_bytes());
			job.m_cmd_list.CopyBufferRegion(resource.get(), 0, allocation);
			job.m_barriers.Transition(resource.get(), final_state).Commit();
			return resource;
		}
	}

	// Create pipeline state without allocating any coupled-feature buffers.
	GpuCoupledConstraintPrepare::GpuCoupledConstraintPrepare(GpuConstraintSolver& constraints, EngineConfig const& config)
		: m_gpu(constraints.m_gpu)
		, m_config(config)
		, m_constraints(constraints)
		, m_cs_prepare()
		, m_cs_prepare_position()
		, m_r_coupled_endpoints()
		, m_r_preconditioners()
		, m_source()
		, m_slot_count()
		, m_active_count()
		, m_previous_timestep()
		, m_frame_warm_start_scale()
		, m_stats()
	{
		auto root_sig = RootSig(ERootSigFlags::ComputeOnly)
			.U32<cbCoupledConstraintPrepare>(EReg::Params)
			.UAV(EReg::Bodies)
			.SRV(EReg::ConstraintEndpoints)
			.SRV(EReg::Descriptors)
			.SRV(EReg::CoupledEndpoints)
			.SRV(EReg::LinkToWorld)
			.SRV(EReg::Mobilities)
			.SRV(EReg::AbaScratch)
			.UAV(EReg::Blocks)
			.UAV(EReg::Rows)
			.UAV(EReg::Preconditioners)
			.Create(m_gpu, "Physics:CoupledConstraintPrepareSig");
		m_cs_prepare.m_sig = root_sig;
		m_cs_prepare.m_pso = ComputePSO(root_sig.get(), shader_code::prepare_coupled_constraints).Create(m_gpu, "Physics:PrepareCoupledConstraintsPSO");
		m_cs_prepare_position.m_sig = root_sig;
		m_cs_prepare_position.m_pso = ComputePSO(root_sig.get(), shader_code::prepare_coupled_position_preconditioners).Create(m_gpu, "Physics:PrepareCoupledPositionPreconditionersPSO");
	}

	// Replace physical preconditioners with exact hard-passive inverses immediately before coupled position iterations.
	void GpuCoupledConstraintPrepare::PreparePositionPreconditioners(GpuJob& job, int body_count, ID3D12Resource* bodies, ID3D12Resource* link_to_world, GpuArticulationMobility& mobility)
	{
		if (m_active_count == 0)
			return;
		if (mobility.m_mobility_count == 0 || mobility.m_r_mobilities == nullptr)
			throw std::logic_error("Coupled position preconditioning requires retained articulation mobilities");
		if (body_count < 0 || bodies == nullptr || link_to_world == nullptr)
			throw std::invalid_argument("Coupled position preconditioning requires valid rigid and link-frame streams");

		auto& aba = mobility.m_aba;
		job.m_barriers.Transition(bodies, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_constraints.m_r_endpoints.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(m_constraints.m_r_descriptors.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(m_r_coupled_endpoints.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(link_to_world, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(mobility.m_r_mobilities.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(aba.m_r_scratch.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(m_constraints.m_r_blocks.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_constraints.m_r_rows.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_preconditioners.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Commit();

		// The position pass reuses the preparation ABI because all response inputs remain at the same fixed configuration.
		auto const constants = cbCoupledConstraintPrepare{
			.slot_count = m_slot_count,
			.body_count = body_count,
			.link_count = aba.m_link_count,
			.mobility_count = mobility.m_mobility_count,
			.timestep = 1.0f,
			.regularization = m_config.constraint_regularization,
			.warm_start_scale = 0.0f,
			.pad0 = 0.0f,
		};
		job.m_cmd_list.SetPipelineState(m_cs_prepare_position.m_pso.get());
		job.m_cmd_list.SetComputeRootSignature(m_cs_prepare_position.m_sig.get());
		job.m_cmd_list.AddComputeRoot32BitConstants(constants);
		job.m_cmd_list.AddComputeRootUnorderedAccessView(bodies->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView(m_constraints.m_r_endpoints->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView(m_constraints.m_r_descriptors->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView(m_r_coupled_endpoints->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView(link_to_world->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView(mobility.m_r_mobilities->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView(aba.m_r_scratch->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_constraints.m_r_blocks->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_constraints.m_r_rows->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_preconditioners->GetGPUVirtualAddress());
		job.m_cmd_list.Dispatch(static_cast<UINT>((m_slot_count + ConstraintThreadCount - 1) / ConstraintThreadCount), 1, 1);
		job.m_barriers.UAV(m_constraints.m_r_blocks.get());
		job.m_barriers.UAV(m_r_preconditioners.get());
		job.m_barriers.Commit();
		++m_stats.m_dispatch_count;
	}

	// Release every coupled-owned resource when no coupled slot is active.
	void GpuCoupledConstraintPrepare::ReleaseBuffers()
	{
		m_r_coupled_endpoints = nullptr;
		m_r_preconditioners = nullptr;
		m_source = nullptr;
		m_slot_count = 0;
		m_active_count = 0;
		m_previous_timestep = 0.0f;
		m_frame_warm_start_scale = 0.0f;
		m_stats = {};
	}

	// Create or geometrically grow stable-slot metadata and preconditioner buffers.
	void GpuCoupledConstraintPrepare::ResizeBuffers(CmdList& cmd_list)
	{
		if (m_slot_count <= m_stats.m_slot_capacity)
			return;

		auto const doubled_capacity = m_stats.m_slot_capacity <= std::numeric_limits<int>::max() / 2
			? 2 * m_stats.m_slot_capacity
			: std::numeric_limits<int>::max();
		m_stats.m_slot_capacity = std::max(m_slot_count, std::max(1, doubled_capacity));
		m_r_coupled_endpoints = m_gpu.CreateResource(
			ResDesc::Buf<GpuCoupledConstraintEndpoint>(m_stats.m_slot_capacity, {}),
			cmd_list,
			"Physics:CoupledConstraintEndpoints");
		m_r_preconditioners = m_gpu.CreateResource(
			ResDesc::Buf<GpuCoupledConstraintPreconditioner>(m_stats.m_slot_capacity, {}).usage(EUsage::UnorderedAccess),
			cmd_list,
			"Physics:CoupledConstraintPreconditioners");
	}

	// Upload stable-slot link ownership after the shared constraint streams have been uploaded.
	bool GpuCoupledConstraintPrepare::Upload(GpuJob& job, GpuConstraintUpload const& upload)
	{
		m_stats.m_dispatch_count = 0;
		if (upload.m_coupled_active_count == 0)
		{
			ReleaseBuffers();
			return false;
		}
		if (upload.m_coupled_endpoints.size() != upload.m_endpoints.size())
			throw std::invalid_argument("Coupled endpoint metadata must preserve the shared stable-slot count");
		if (m_constraints.m_source != upload.m_source || m_constraints.m_slot_count != isize(upload.m_endpoints))
			throw std::logic_error("Coupled metadata must be uploaded after the matching shared constraint streams");

		if (m_source != upload.m_source)
		{
			m_previous_timestep = 0.0f;
			m_frame_warm_start_scale = 0.0f;
		}
		m_source = upload.m_source;
		m_slot_count = isize(upload.m_endpoints);
		m_active_count = s_cast<int>(upload.m_coupled_active_count);
		ResizeBuffers(job.m_cmd_list);

		// Link ownership changes with frame-local packing, so upload the complete stable-slot stream every active frame.
		job.m_barriers.Transition(m_r_coupled_endpoints.get(), D3D12_RESOURCE_STATE_COPY_DEST).Commit();
		auto allocation = job.m_upload.Alloc<GpuCoupledConstraintEndpoint>(m_slot_count);
		memcpy(allocation.ptr<GpuCoupledConstraintEndpoint>(), upload.m_coupled_endpoints.data(), upload.m_coupled_endpoints.size() * sizeof(upload.m_coupled_endpoints[0]));
		job.m_cmd_list.CopyBufferRegion(m_r_coupled_endpoints.get(), 0, allocation);
		job.m_barriers.Transition(m_r_coupled_endpoints.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE).Commit();

		m_stats.m_active_count = m_active_count;
		m_stats.m_logical_bytes =
			static_cast<size_t>(m_slot_count) *
			(sizeof(GpuCoupledConstraintEndpoint) + sizeof(GpuCoupledConstraintPreconditioner));
		m_stats.m_allocated_feature_bytes =
			static_cast<size_t>(m_stats.m_slot_capacity) *
			(sizeof(GpuCoupledConstraintEndpoint) + sizeof(GpuCoupledConstraintPreconditioner));
		return true;
	}

	// Compile link-coordinate rows and exact-self preconditioners from retained final-configuration mobility factors.
	void GpuCoupledConstraintPrepare::Run(GpuJob& job, float timestep, int body_count, ID3D12Resource* bodies, ID3D12Resource* link_to_world, GpuArticulationMobility& mobility)
	{
		if (m_active_count == 0)
			return;
		if (mobility.m_mobility_count == 0 || mobility.m_r_mobilities == nullptr)
			throw std::logic_error("Coupled constraint preparation requires retained articulation mobilities");

		Run(
			job,
			timestep,
			body_count,
			mobility.m_aba.m_link_count,
			mobility.m_mobility_count,
			bodies,
			link_to_world,
			mobility.m_r_mobilities.get(),
			mobility.m_aba.m_r_scratch.get());
	}

	// Record the preparation pass against explicit production or diagnostic resources.
	void GpuCoupledConstraintPrepare::Run(
		GpuJob& job,
		float timestep,
		int body_count,
		int link_count,
		int mobility_count,
		ID3D12Resource* bodies,
		ID3D12Resource* link_to_world,
		ID3D12Resource* mobilities,
		ID3D12Resource* aba_scratch)
	{
		m_stats.m_dispatch_count = 0;
		if (m_active_count == 0)
			return;
		if (!(timestep > 0.0f) || !std::isfinite(timestep))
			throw std::invalid_argument("Coupled constraint preparation requires a finite positive timestep");
		if (body_count < 0 || link_count <= 0 || mobility_count <= 0)
			throw std::invalid_argument("Coupled constraint preparation received invalid packed counts");
		if (bodies == nullptr || link_to_world == nullptr || mobilities == nullptr || aba_scratch == nullptr)
			throw std::invalid_argument("Coupled constraint preparation requires every bound resource");

		// Scale retained impulses only across ordinary timestep changes; large discontinuities intentionally cold-start.
		m_frame_warm_start_scale = 0.0f;
		if (m_previous_timestep > 0.0f)
		{
			auto const timestep_ratio = timestep / m_previous_timestep;
			if (timestep_ratio >= 0.25f && timestep_ratio <= 4.0f)
				m_frame_warm_start_scale = m_config.constraint_warm_start_factor * timestep_ratio;
		}
		m_previous_timestep = timestep;

		job.m_barriers.Transition(bodies, D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_constraints.m_r_endpoints.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(m_constraints.m_r_descriptors.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(m_r_coupled_endpoints.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(link_to_world, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(mobilities, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(aba_scratch, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(m_constraints.m_r_blocks.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_constraints.m_r_rows.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_preconditioners.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Commit();

		auto const constants = cbCoupledConstraintPrepare{
			.slot_count = m_slot_count,
			.body_count = body_count,
			.link_count = link_count,
			.mobility_count = mobility_count,
			.timestep = timestep,
			.regularization = m_config.constraint_regularization,
			.warm_start_scale = m_frame_warm_start_scale,
			.pad0 = 0.0f,
		};
		job.m_cmd_list.SetPipelineState(m_cs_prepare.m_pso.get());
		job.m_cmd_list.SetComputeRootSignature(m_cs_prepare.m_sig.get());
		job.m_cmd_list.AddComputeRoot32BitConstants(constants);
		job.m_cmd_list.AddComputeRootUnorderedAccessView(bodies->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView(m_constraints.m_r_endpoints->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView(m_constraints.m_r_descriptors->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView(m_r_coupled_endpoints->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView(link_to_world->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView(mobilities->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView(aba_scratch->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_constraints.m_r_blocks->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_constraints.m_r_rows->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_preconditioners->GetGPUVirtualAddress());
		job.m_cmd_list.Dispatch(static_cast<UINT>((m_slot_count + ConstraintThreadCount - 1) / ConstraintThreadCount), 1, 1);

		// Published rows, blocks, and inverses must become visible together before warm start or coupled sweeps consume them.
		job.m_barriers.UAV(m_constraints.m_r_blocks.get());
		job.m_barriers.UAV(m_constraints.m_r_rows.get());
		job.m_barriers.UAV(m_r_preconditioners.get());
		job.m_barriers.Commit();
		m_stats.m_dispatch_count = 1;
	}

	// Upload, execute, and read back focused diagnostics using exactly one GPU submission.
	GpuCoupledConstraintPrepareResult GpuCoupledConstraintPrepare::Solve(
		GpuJob& job,
		float timestep,
		GpuConstraintUpload const& upload,
		std::span<GpuRigidBody const> bodies,
		std::span<GpuConstraintFrame const> link_to_world,
		std::span<GpuArticulationSpatialMobility const> mobilities,
		std::span<GpuArticulationAbaScratch const> aba_scratch)
	{
		if (link_to_world.size() != aba_scratch.size())
			throw std::invalid_argument("Coupled diagnostic link frames and ABA scratch counts must match");

		m_constraints.Upload(job, upload);
		auto result = GpuCoupledConstraintPrepareResult{};
		if (!Upload(job, upload))
			return result;

		auto r_bodies = CreateCoupledInputResource(m_gpu, job, bodies, EUsage::UnorderedAccess, D3D12_RESOURCE_STATE_UNORDERED_ACCESS, "Physics:CoupledPrepareTestBodies");
		auto r_link_to_world = CreateCoupledInputResource(m_gpu, job, link_to_world, EUsage::Default, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE, "Physics:CoupledPrepareTestLinkFrames");
		auto r_mobilities = CreateCoupledInputResource(m_gpu, job, mobilities, EUsage::Default, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE, "Physics:CoupledPrepareTestMobilities");
		auto r_aba_scratch = CreateCoupledInputResource(m_gpu, job, aba_scratch, EUsage::Default, D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE, "Physics:CoupledPrepareTestAbaScratch");
		Run(
			job,
			timestep,
			isize(bodies),
			isize(link_to_world),
			isize(mobilities),
			r_bodies.get(),
			r_link_to_world.get(),
			r_mobilities.get(),
			r_aba_scratch.get());

		// Gather all stable-slot diagnostics in the caller's existing one-submission job.
		result.m_blocks.resize(m_slot_count);
		result.m_rows.resize(GpuConstraintRowsPerBlock * m_slot_count);
		result.m_preconditioners.resize(m_slot_count);
		job.m_barriers.Transition(m_constraints.m_r_blocks.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
		job.m_barriers.Transition(m_constraints.m_r_rows.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
		job.m_barriers.Transition(m_r_preconditioners.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
		job.m_barriers.Commit();
		auto block_readback = job.m_readback.Alloc<GpuConstraintBlock>(m_slot_count);
		auto row_readback = job.m_readback.Alloc<GpuConstraintRow>(GpuConstraintRowsPerBlock * m_slot_count);
		auto preconditioner_readback = job.m_readback.Alloc<GpuCoupledConstraintPreconditioner>(m_slot_count);
		job.m_cmd_list.CopyBufferRegion(block_readback, m_constraints.m_r_blocks.get(), 0);
		job.m_cmd_list.CopyBufferRegion(row_readback, m_constraints.m_r_rows.get(), 0);
		job.m_cmd_list.CopyBufferRegion(preconditioner_readback, m_r_preconditioners.get(), 0);
		job.Run();

		memcpy(result.m_blocks.data(), block_readback.ptr<GpuConstraintBlock>(), result.m_blocks.size() * sizeof(result.m_blocks[0]));
		memcpy(result.m_rows.data(), row_readback.ptr<GpuConstraintRow>(), result.m_rows.size() * sizeof(result.m_rows[0]));
		memcpy(result.m_preconditioners.data(), preconditioner_readback.ptr<GpuCoupledConstraintPreconditioner>(), result.m_preconditioners.size() * sizeof(result.m_preconditioners[0]));
		return result;
	}

	// Return current logical usage, retained capacity, and dispatch count.
	GpuCoupledConstraintPrepareStats const& GpuCoupledConstraintPrepare::Stats() const
	{
		return m_stats;
	}
}
