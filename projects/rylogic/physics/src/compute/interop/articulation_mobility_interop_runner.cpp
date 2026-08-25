//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "src/compute/interop/articulation_mobility_runner.h"
#include "src/compute/articulation_mobility_gpu.h"

// Give this translation unit private emulated resource identities because several shader replays reuse the canonical ABA names.
#define g_aba g_mobility_aba
#define g_aba_articulations g_mobility_aba_articulations
#define g_aba_links g_mobility_aba_links
#define g_aba_dofs g_mobility_aba_dofs
#define g_aba_positions g_mobility_aba_positions
#define g_aba_velocities g_mobility_aba_velocities
#define g_aba_forces g_mobility_aba_forces
#define g_aba_external_forces g_mobility_aba_external_forces
#define g_aba_children g_mobility_aba_children
#define g_aba_level_links g_mobility_aba_level_links
#define g_aba_accelerations g_mobility_aba_accelerations
#define g_aba_scratch g_mobility_aba_scratch
#define g_aba_dof_scratch g_mobility_aba_dof_scratch
#define g_aba_inverse_joint_inertia g_mobility_aba_inverse_joint_inertia
#include "src/compute/articulation_mobility.hlsl"

namespace pr::physics
{
	namespace
	{
		// Return a const span suitable for assigning one emulated shader resource.
		template <typename T> std::span<T const> MobilitySpanOf(std::vector<T> const& values)
		{
			return std::span<T const>{values.data(), values.size()};
		}

		// Return the exact group count for a non-empty participating-articulation range.
		int MobilityThreadGroupCount(int articulation_count)
		{
			return (articulation_count + ArticulationThreadCount - 1) / ArticulationThreadCount;
		}
	}

	// Rebuild exact mobilities for the selected articulation indices.
	void ArticulationMobilityInteropRunner::Run(GpuArticulationUpload const& upload, std::span<int const> articulation_indices)
	{
		m_ranges = BuildGpuArticulationMobilityRanges(upload, articulation_indices);
		if (m_ranges.empty())
		{
			m_mobilities.clear();
			m_accelerations.clear();
			m_scratch.clear();
			m_dof_scratch.clear();
			m_joint_matrix_scratch.clear();
			return;
		}

		// Replay uses exact active dimensions and preserves shared ABA factor layout.
		auto const& final_range = m_ranges.back();
		auto const mobility_count = final_range.mobility_offset + final_range.link_count;
		m_mobilities.assign(mobility_count, GpuArticulationSpatialMobility{});
		m_accelerations.assign(upload.m_accelerations.size(), 0.0f);
		m_scratch.assign(upload.m_links.size(), GpuArticulationAbaScratch{});
		m_dof_scratch.assign(upload.m_dofs.size(), GpuArticulationAbaDofScratch{});
		m_joint_matrix_scratch.assign(upload.m_joint_matrix_scratch_count, 0.0f);

		// Bind immutable packed state and writable factor/output ranges exactly as the hardware pass does.
		g_mobility = cbArticulationMobility{
			.participating_articulation_count = isize(m_ranges),
			.articulation_count = isize(upload.m_articulations),
			.link_count = isize(upload.m_links),
			.mobility_count = mobility_count,
		};
		g_mobility_ranges.assign(MobilitySpanOf(m_ranges));
		g_aba_articulations.assign(MobilitySpanOf(upload.m_articulations));
		g_aba_links.assign(MobilitySpanOf(upload.m_links));
		g_aba_dofs.assign(MobilitySpanOf(upload.m_dofs));
		g_aba_positions.assign(MobilitySpanOf(upload.m_positions));
		g_aba_velocities.assign(MobilitySpanOf(upload.m_velocities));
		g_aba_forces.assign(MobilitySpanOf(upload.m_forces));
		g_aba_external_forces.assign(MobilitySpanOf(upload.m_external_forces));
		g_aba_children.assign(MobilitySpanOf(upload.m_children));
		g_aba_accelerations.assign(MobilitySpanOf(m_accelerations));
		g_aba_scratch.assign(MobilitySpanOf(m_scratch));
		g_aba_dof_scratch.assign(MobilitySpanOf(m_dof_scratch));
		g_aba_inverse_joint_inertia.assign(MobilitySpanOf(m_joint_matrix_scratch));
		g_link_mobilities.assign(MobilitySpanOf(m_mobilities));

		hlsl::GpuEmulator emulator(CSArticulationPrepareMobility, CSArticulationPrepareMobility_NumThreads);
		emulator.Dispatch({MobilityThreadGroupCount(isize(m_ranges)), 1, 1});

		m_mobilities.assign(g_link_mobilities.begin(), g_link_mobilities.end());
		m_accelerations.assign(g_aba_accelerations.begin(), g_aba_accelerations.end());
		m_scratch.assign(g_aba_scratch.begin(), g_aba_scratch.end());
		m_dof_scratch.assign(g_aba_dof_scratch.begin(), g_aba_dof_scratch.end());
		m_joint_matrix_scratch.assign(g_aba_inverse_joint_inertia.begin(), g_aba_inverse_joint_inertia.end());

		// Replay rejects singular factors instead of exposing regularized diagnostic mobilities.
		for (auto const& range : m_ranges)
		{
			auto const& articulation = upload.m_articulations[range.articulation_index];
			for (int link_offset = 1; link_offset != range.link_count; ++link_offset)
			{
				if (m_scratch[articulation.link_offset + link_offset].solve_valid == 0)
					throw std::runtime_error(std::format(
						"Articulation self-link mobility factorization failed for articulation {} link {}",
						range.articulation_index,
						link_offset));
			}
			if (m_scratch[articulation.link_offset].solve_valid == 0)
				throw std::runtime_error(std::format(
					"Articulation self-link root response failed for articulation {}",
					range.articulation_index));
		}
	}

#undef g_aba
#undef g_aba_articulations
#undef g_aba_links
#undef g_aba_dofs
#undef g_aba_positions
#undef g_aba_velocities
#undef g_aba_forces
#undef g_aba_external_forces
#undef g_aba_children
#undef g_aba_level_links
#undef g_aba_accelerations
#undef g_aba_scratch
#undef g_aba_dof_scratch
#undef g_aba_inverse_joint_inertia

	// Return canonical participating ranges and their compact output offsets.
	std::span<GpuArticulationMobilityRange const> ArticulationMobilityInteropRunner::Ranges() const
	{
		return m_ranges;
	}

	// Return packed exact self-link mobilities in canonical range order.
	std::span<GpuArticulationSpatialMobility const> ArticulationMobilityInteropRunner::Mobilities() const
	{
		return m_mobilities;
	}

	// Return shared ABA scratch after final-configuration factorization.
	std::span<GpuArticulationAbaScratch const> ArticulationMobilityInteropRunner::Scratch() const
	{
		return m_scratch;
	}

	// Return phase-reused generalized response scratch after factorization.
	std::span<float const> ArticulationMobilityInteropRunner::Accelerations() const
	{
		return m_accelerations;
	}

	// Return retained per-DOF motion subspaces and articulated columns.
	std::span<GpuArticulationAbaDofScratch const> ArticulationMobilityInteropRunner::DofScratch() const
	{
		return m_dof_scratch;
	}

	// Return retained packed inverse joint matrices.
	std::span<float const> ArticulationMobilityInteropRunner::InverseJointInertia() const
	{
		return m_joint_matrix_scratch;
	}
}
