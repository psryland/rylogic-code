//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "src/compute/articulation_mobility_gpu.h"
#include "src/compute/shader_code.h"

namespace pr::physics
{
	using namespace ::pr::compute;

	namespace
	{
		// Active compact ranges and shared packed-buffer bounds.
		struct alignas(16) cbArticulationMobility
		{
			int participating_articulation_count;
			int articulation_count;
			int link_count;
			int mobility_count;
		};
		static_assert(sizeof(cbArticulationMobility) == 16);

		// Root-register assignments shared with articulation_mobility.hlsl.
		struct EReg
		{
			inline static constexpr auto Params = ECBufReg::b0;
			inline static constexpr auto Ranges = ESRVReg::t0;
			inline static constexpr auto Articulations = ESRVReg::t1;
			inline static constexpr auto Links = ESRVReg::t2;
			inline static constexpr auto Dofs = ESRVReg::t3;
			inline static constexpr auto Positions = ESRVReg::t4;
			inline static constexpr auto Velocities = ESRVReg::t5;
			inline static constexpr auto Forces = ESRVReg::t6;
			inline static constexpr auto ExternalForces = ESRVReg::t7;
			inline static constexpr auto Children = ESRVReg::t8;
			inline static constexpr auto Accelerations = EUAVReg::u0;
			inline static constexpr auto Scratch = EUAVReg::u1;
			inline static constexpr auto DofScratch = EUAVReg::u2;
			inline static constexpr auto JointMatrixScratch = EUAVReg::u3;
			inline static constexpr auto Mobilities = EUAVReg::u4;
		};

		// Create or geometrically grow one mobility-owned typed buffer.
		template <typename T> void EnsureMobilityBuffer(Gpu& gpu, CmdList& cmd_list, D3DPtr<ID3D12Resource>& resource, int count, int& capacity, EUsage usage, std::string_view name)
		{
			if (count <= capacity)
				return;

			auto const doubled_capacity = capacity <= std::numeric_limits<int>::max() / 2 ? 2 * capacity : std::numeric_limits<int>::max();
			capacity = std::max(count, std::max(1, doubled_capacity));
			resource = gpu.CreateResource(ResDesc::Buf<T>(capacity, {}).usage(usage), cmd_list, name);
		}

		// Return the exact group count for a non-empty participating-articulation range.
		int MobilityThreadGroupCount(int articulation_count)
		{
			return (articulation_count + ArticulationThreadCount - 1) / ArticulationThreadCount;
		}
	}

	// Return true when every participating link completed a non-singular factorization.
	bool GpuArticulationMobilityResult::AllValid() const
	{
		return std::ranges::all_of(m_solve_valid, [](int valid) { return valid != 0; });
	}

	// Return canonical compact output ranges for the selected articulation indices.
	std::vector<GpuArticulationMobilityRange> BuildGpuArticulationMobilityRanges(GpuArticulationUpload const& upload, std::span<int const> articulation_indices)
	{
		ValidateGpuArticulationUpload(upload);

		// Canonical ascending indices make output layout independent of constraint insertion order.
		auto canonical_indices = std::vector<int>{articulation_indices.begin(), articulation_indices.end()};
		for (auto articulation_index : canonical_indices)
		{
			if (articulation_index < 0 || articulation_index >= isize(upload.m_articulations))
				throw std::out_of_range("Articulation mobility participant index is out of range");
		}
		std::ranges::sort(canonical_indices);
		auto const unique_end = std::ranges::unique(canonical_indices).begin();
		canonical_indices.erase(unique_end, canonical_indices.end());

		// Every selected tree occupies one contiguous output range containing all of its links.
		auto ranges = std::vector<GpuArticulationMobilityRange>{};
		ranges.reserve(canonical_indices.size());
		auto mobility_offset = 0;
		for (auto articulation_index : canonical_indices)
		{
			auto const& articulation = upload.m_articulations[articulation_index];
			if (articulation.link_count > std::numeric_limits<int>::max() - mobility_offset)
				throw std::overflow_error("Articulation mobility link count exceeds the packed index range");

			ranges.push_back(GpuArticulationMobilityRange{
				.articulation_index = articulation_index,
				.mobility_offset = mobility_offset,
				.link_count = articulation.link_count,
				.pad0 = 0,
			});
			mobility_offset += articulation.link_count;
		}
		return ranges;
	}

	// Create the mobility pipeline without allocating articulation-dependent resources.
	GpuArticulationMobility::GpuArticulationMobility(GpuArticulationForceAba& aba)
		: m_aba(aba)
		, m_cs_prepare()
		, m_r_ranges()
		, m_r_mobilities()
		, m_ranges()
		, m_mobility_count()
		, m_stats()
	{
		auto root_sig = RootSig(ERootSigFlags::ComputeOnly)
			.U32<cbArticulationMobility>(EReg::Params)
			.SRV(EReg::Ranges)
			.SRV(EReg::Articulations)
			.SRV(EReg::Links)
			.SRV(EReg::Dofs)
			.SRV(EReg::Positions)
			.SRV(EReg::Velocities)
			.SRV(EReg::Forces)
			.SRV(EReg::ExternalForces)
			.SRV(EReg::Children)
			.UAV(EReg::Accelerations)
			.UAV(EReg::Scratch)
			.UAV(EReg::DofScratch)
			.UAV(EReg::JointMatrixScratch)
			.UAV(EReg::Mobilities)
			.Create(m_aba.m_gpu, "Physics:ArticulationMobilitySig");
		m_cs_prepare.m_sig = root_sig;
		m_cs_prepare.m_pso = ComputePSO(root_sig.get(), shader_code::articulation_prepare_mobility).Create(m_aba.m_gpu, "Physics:ArticulationPrepareMobilityPSO");
	}

	// Release every mobility-owned resource when no articulation participates.
	void GpuArticulationMobility::ReleaseBuffers()
	{
		m_r_ranges = nullptr;
		m_r_mobilities = nullptr;
		m_ranges.clear();
		m_mobility_count = 0;
		m_stats = {};
	}

	// Create or geometrically grow the range and compact mobility resources.
	void GpuArticulationMobility::ResizeBuffers(CmdList& cmd_list)
	{
		EnsureMobilityBuffer<GpuArticulationMobilityRange>(
			m_aba.m_gpu,
			cmd_list,
			m_r_ranges,
			isize(m_ranges),
			m_stats.m_range_capacity,
			EUsage::Default,
			"Physics:ArticulationMobilityRanges");
		EnsureMobilityBuffer<GpuArticulationSpatialMobility>(
			m_aba.m_gpu,
			cmd_list,
			m_r_mobilities,
			m_mobility_count,
			m_stats.m_mobility_capacity,
			EUsage::UnorderedAccess,
			"Physics:ArticulationLinkMobilities");
	}

	// Upload compact participating-tree ranges after the matching forest has been uploaded to the shared ABA owner.
	bool GpuArticulationMobility::Upload(GpuJob& job, GpuArticulationUpload const& upload, std::span<int const> articulation_indices)
	{
		m_stats.m_dispatch_count = 0;
		m_ranges = BuildGpuArticulationMobilityRanges(upload, articulation_indices);
		if (m_ranges.empty())
		{
			ReleaseBuffers();
			return false;
		}
		if (
			m_aba.m_articulation_count != isize(upload.m_articulations) ||
			m_aba.m_link_count != isize(upload.m_links) ||
			m_aba.m_dof_count != isize(upload.m_dofs))
			throw std::logic_error("Articulation mobility upload does not match the shared ABA forest");

		auto const& final_range = m_ranges.back();
		m_mobility_count = final_range.mobility_offset + final_range.link_count;
		ResizeBuffers(job.m_cmd_list);

		// Participating ranges are immutable for the frame while mobility output is regenerated per substep.
		job.m_barriers.Transition(m_r_ranges.get(), D3D12_RESOURCE_STATE_COPY_DEST).Commit();
		auto allocation = job.m_upload.Alloc<GpuArticulationMobilityRange>(isize(m_ranges));
		memcpy(allocation.ptr<GpuArticulationMobilityRange>(), m_ranges.data(), m_ranges.size() * sizeof(m_ranges[0]));
		job.m_cmd_list.CopyBufferRegion(m_r_ranges.get(), 0, allocation);
		job.m_barriers.Transition(m_r_ranges.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(m_r_mobilities.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Commit();

		m_stats.m_logical_bytes =
			m_ranges.size() * sizeof(GpuArticulationMobilityRange) +
			static_cast<size_t>(m_mobility_count) * sizeof(GpuArticulationSpatialMobility);
		m_stats.m_allocated_feature_bytes =
			static_cast<size_t>(m_stats.m_range_capacity) * sizeof(GpuArticulationMobilityRange) +
			static_cast<size_t>(m_stats.m_mobility_capacity) * sizeof(GpuArticulationSpatialMobility);
		return true;
	}

	// Rebuild final-configuration ABA factors and exact self-link mobilities for all participating trees.
	void GpuArticulationMobility::Run(GpuJob& job)
	{
		m_stats.m_dispatch_count = 0;
		if (m_ranges.empty())
			return;

		// The pass reads committed primary state and rewrites only shared factor scratch plus its compact mobility stream.
		job.m_barriers.Transition(m_r_ranges.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(m_aba.m_r_articulations.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(m_aba.m_r_links.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition((m_aba.m_dof_count != 0 ? m_aba.m_r_dofs : m_aba.m_r_srv_sentinel).get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition((m_aba.m_position_count != 0 ? m_aba.m_r_positions : m_aba.m_r_srv_sentinel).get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition((m_aba.m_velocity_count != 0 ? m_aba.m_r_velocities : m_aba.m_r_srv_sentinel).get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition((m_aba.m_force_count != 0 ? m_aba.m_r_forces : m_aba.m_r_srv_sentinel).get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition(m_aba.m_r_external_forces.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition((m_aba.m_child_count != 0 ? m_aba.m_r_children : m_aba.m_r_srv_sentinel).get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
		job.m_barriers.Transition((m_aba.m_acceleration_count != 0 ? m_aba.m_r_accelerations : m_aba.m_r_uav_sentinel).get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_aba.m_r_scratch.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition((m_aba.m_dof_count != 0 ? m_aba.m_r_dof_scratch : m_aba.m_r_uav_sentinel).get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition((m_aba.m_joint_matrix_count != 0 ? m_aba.m_r_joint_matrix_scratch : m_aba.m_r_uav_sentinel).get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Transition(m_r_mobilities.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
		job.m_barriers.Commit();

		auto const constants = cbArticulationMobility{
			.participating_articulation_count = isize(m_ranges),
			.articulation_count = m_aba.m_articulation_count,
			.link_count = m_aba.m_link_count,
			.mobility_count = m_mobility_count,
		};
		job.m_cmd_list.SetPipelineState(m_cs_prepare.m_pso.get());
		job.m_cmd_list.SetComputeRootSignature(m_cs_prepare.m_sig.get());
		job.m_cmd_list.AddComputeRoot32BitConstants(constants);
		job.m_cmd_list.AddComputeRootShaderResourceView(m_r_ranges->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView(m_aba.m_r_articulations->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView(m_aba.m_r_links->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView((m_aba.m_dof_count != 0 ? m_aba.m_r_dofs : m_aba.m_r_srv_sentinel)->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView((m_aba.m_position_count != 0 ? m_aba.m_r_positions : m_aba.m_r_srv_sentinel)->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView((m_aba.m_velocity_count != 0 ? m_aba.m_r_velocities : m_aba.m_r_srv_sentinel)->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView((m_aba.m_force_count != 0 ? m_aba.m_r_forces : m_aba.m_r_srv_sentinel)->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView(m_aba.m_r_external_forces->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootShaderResourceView((m_aba.m_child_count != 0 ? m_aba.m_r_children : m_aba.m_r_srv_sentinel)->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView((m_aba.m_acceleration_count != 0 ? m_aba.m_r_accelerations : m_aba.m_r_uav_sentinel)->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_aba.m_r_scratch->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView((m_aba.m_dof_count != 0 ? m_aba.m_r_dof_scratch : m_aba.m_r_uav_sentinel)->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView((m_aba.m_joint_matrix_count != 0 ? m_aba.m_r_joint_matrix_scratch : m_aba.m_r_uav_sentinel)->GetGPUVirtualAddress());
		job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_mobilities->GetGPUVirtualAddress());
		job.m_cmd_list.Dispatch(MobilityThreadGroupCount(isize(m_ranges)), 1, 1);
		++m_stats.m_dispatch_count;

		// Every downstream consumer observes complete factors and compact matrices from all participating trees.
		if (m_aba.m_acceleration_count != 0)
			job.m_barriers.UAV(m_aba.m_r_accelerations.get());
		job.m_barriers.UAV(m_aba.m_r_scratch.get());
		if (m_aba.m_dof_count != 0)
			job.m_barriers.UAV(m_aba.m_r_dof_scratch.get());
		if (m_aba.m_joint_matrix_count != 0)
			job.m_barriers.UAV(m_aba.m_r_joint_matrix_scratch.get());
		job.m_barriers.UAV(m_r_mobilities.get());
		job.m_barriers.Commit();
	}

	// Upload, execute, and read back focused diagnostics using exactly one GPU submission.
	GpuArticulationMobilityResult GpuArticulationMobility::Solve(GpuJob& job, GpuArticulationUpload const& upload, std::span<int const> articulation_indices)
	{
		auto result = GpuArticulationMobilityResult{};
		if (BuildGpuArticulationMobilityRanges(upload, articulation_indices).empty())
		{
			ReleaseBuffers();
			return result;
		}
		if (!m_aba.Upload(job, upload))
		{
			Upload(job, upload, {});
			return result;
		}
		if (!Upload(job, upload, articulation_indices))
			return result;

		Run(job);

		// Read compact mobility output and shared solve-valid flags without exposing other ABA scratch.
		job.m_barriers.Transition(m_r_mobilities.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
		job.m_barriers.Transition(m_aba.m_r_scratch.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
		job.m_barriers.Commit();
		auto mobility_readback = job.m_readback.Alloc<GpuArticulationSpatialMobility>(m_mobility_count);
		auto scratch_readback = job.m_readback.Alloc<GpuArticulationAbaScratch>(m_aba.m_link_count);
		job.m_cmd_list.CopyBufferRegion(mobility_readback, m_r_mobilities.get(), 0);
		job.m_cmd_list.CopyBufferRegion(scratch_readback, m_aba.m_r_scratch.get(), 0);
		job.Run();

		result.m_ranges = m_ranges;
		result.m_mobilities.assign(
			mobility_readback.ptr<GpuArticulationSpatialMobility>(),
			mobility_readback.ptr<GpuArticulationSpatialMobility>() + m_mobility_count);
		auto const* scratch = scratch_readback.ptr<GpuArticulationAbaScratch>();
		result.m_solve_valid.reserve(m_mobility_count);
		for (auto const& range : m_ranges)
		{
			auto const& articulation = upload.m_articulations[range.articulation_index];
			for (int link_offset = 0; link_offset != range.link_count; ++link_offset)
				result.m_solve_valid.push_back(scratch[articulation.link_offset + link_offset].solve_valid);
		}
		return result;
	}

	// Return the compact mobility resource for downstream coupled-block preparation.
	ID3D12Resource* GpuArticulationMobility::Mobilities()
	{
		return m_r_mobilities.get();
	}

	// Return canonical participating ranges and their compact mobility offsets.
	std::span<GpuArticulationMobilityRange const> GpuArticulationMobility::Ranges() const
	{
		return m_ranges;
	}

	// Return current logical usage, retained capacities, and the most recent dispatch count.
	GpuArticulationMobilityStats const& GpuArticulationMobility::Stats() const
	{
		return m_stats;
	}

	// Destroy lazily owned mobility resources where the implementation type is complete.
	void Deleter<GpuArticulationMobility>::operator()(GpuArticulationMobility* mobility) const
	{
		delete mobility;
	}
}
