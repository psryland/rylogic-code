//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "src/compute/interop/articulation_force_aba_runner.h"
#include "src/compute/articulation_force_aba.hlsl"

namespace pr::physics
{
	using namespace articulation_force_aba_detail;

	namespace
	{
		// Return a const span suitable for assigning one emulated shader resource.
		template <typename T> std::span<T const> ArticulationSpanOf(std::vector<T> const& values)
		{
			return std::span<T const>{values.data(), values.size()};
		}

		// Return the exact non-zero group count for one level or root dispatch.
		int ArticulationThreadGroupCount(int item_count)
		{
			return (item_count + ArticulationThreadCount - 1) / ArticulationThreadCount;
		}

		// Select one breadth-level range for the next emulated kernel dispatch.
		void SetArticulationLevelConstants(GpuArticulationLevel const& level, int articulation_count, int link_count)
		{
			g_aba = cbArticulationForceAba{
				.level_offset = level.link_offset,
				.level_count = level.link_count,
				.articulation_count = articulation_count,
				.link_count = link_count,
			};
		}
	}

	// Execute the same parent-owned level order intended for the future D3D12 host path.
	void ArticulationForceAbaInteropRunner::Run(GpuArticulationUpload& upload)
	{
		ValidateGpuArticulationUpload(upload);
		std::ranges::fill(upload.m_accelerations, 0.0f);

		// Empty optional input performs no emulated dispatch and retains no scratch allocation.
		if (upload.m_articulations.empty())
		{
			m_scratch = {};
			m_dof_scratch = {};
			m_joint_matrix_scratch = {};
			return;
		}

		// Allocate exactly one compact core per link, two spatial factors per DOF, and each active d-by-d inverse block.
		m_scratch.assign(upload.m_links.size(), GpuArticulationAbaScratch{});
		m_dof_scratch.assign(upload.m_dofs.size(), GpuArticulationAbaDofScratch{});
		m_joint_matrix_scratch.assign(upload.m_joint_matrix_scratch_count, 0.0f);

		// Bind immutable packed ABI ranges once; only schedule constants change between levels.
		g_aba_articulations.assign(ArticulationSpanOf(upload.m_articulations));
		g_aba_links.assign(ArticulationSpanOf(upload.m_links));
		g_aba_dofs.assign(ArticulationSpanOf(upload.m_dofs));
		g_aba_positions.assign(ArticulationSpanOf(upload.m_positions));
		g_aba_velocities.assign(ArticulationSpanOf(upload.m_velocities));
		g_aba_forces.assign(ArticulationSpanOf(upload.m_forces));
		g_aba_external_forces.assign(ArticulationSpanOf(upload.m_external_forces));
		g_aba_children.assign(ArticulationSpanOf(upload.m_children));
		g_aba_level_links.assign(ArticulationSpanOf(upload.m_level_links));
		g_aba_accelerations.assign(ArticulationSpanOf(upload.m_accelerations));
		g_aba_scratch.assign(ArticulationSpanOf(m_scratch));
		g_aba_dof_scratch.assign(ArticulationSpanOf(m_dof_scratch));
		g_aba_inverse_joint_inertia.assign(ArticulationSpanOf(m_joint_matrix_scratch));

		// Prepare transforms, motion subspaces, velocities, and physical bias in parent-before-child order.
		for (auto const& level : upload.m_levels)
		{
			if (level.link_count == 0)
				continue;

			SetArticulationLevelConstants(level, isize(upload.m_articulations), isize(upload.m_links));
			hlsl::GpuEmulator emulator(CSArticulationPrepare, CSArticulationPrepare_NumThreads);
			emulator.Dispatch({ArticulationThreadGroupCount(level.link_count), 1, 1});
		}

		// Each reverse-level parent lane reduces direct children in stable adjacency order.
		for (int level_index = isize(upload.m_levels); level_index-- != 0;)
		{
			auto const& level = upload.m_levels[level_index];
			if (level.link_count == 0)
				continue;

			SetArticulationLevelConstants(level, isize(upload.m_articulations), isize(upload.m_links));
			hlsl::GpuEmulator emulator(CSArticulationInwardDynamics, CSArticulationInwardDynamics_NumThreads);
			emulator.Dispatch({ArticulationThreadGroupCount(level.link_count), 1, 1});
		}

		// Roots are independent and select fixed zero acceleration or one six-dimensional solve.
		g_aba = cbArticulationForceAba{
			.level_offset = 0,
			.level_count = 0,
			.articulation_count = isize(upload.m_articulations),
			.link_count = isize(upload.m_links),
		};
		{
			hlsl::GpuEmulator emulator(CSArticulationRootDynamics, CSArticulationRootDynamics_NumThreads);
			emulator.Dispatch({ArticulationThreadGroupCount(isize(upload.m_articulations)), 1, 1});
		}

		// Recover non-root generalized and link accelerations in parent-before-child order.
		for (int level_index = 1; level_index != isize(upload.m_levels); ++level_index)
		{
			auto const& level = upload.m_levels[level_index];
			if (level.link_count == 0)
				continue;

			SetArticulationLevelConstants(level, isize(upload.m_articulations), isize(upload.m_links));
			hlsl::GpuEmulator emulator(CSArticulationOutwardDynamics, CSArticulationOutwardDynamics_NumThreads);
			emulator.Dispatch({ArticulationThreadGroupCount(level.link_count), 1, 1});
		}

		upload.m_accelerations.assign(g_aba_accelerations.begin(), g_aba_accelerations.end());
		m_scratch.assign(g_aba_scratch.begin(), g_aba_scratch.end());
		m_dof_scratch.assign(g_aba_dof_scratch.begin(), g_aba_dof_scratch.end());
		m_joint_matrix_scratch.assign(g_aba_inverse_joint_inertia.begin(), g_aba_inverse_joint_inertia.end());

		// Replay reports the same explicit failure as the CPU solver instead of exposing regularized diagnostic values.
		if (std::ranges::any_of(m_scratch, [](GpuArticulationAbaScratch const& scratch) { return scratch.solve_valid == 0; }))
			throw std::runtime_error("Articulation joint or root inertia is singular or not positive definite");
	}

	// Return one solved link-frame spatial acceleration from the phase-reused core scratch.
	GpuArticulationSpatialVector const& ArticulationForceAbaInteropRunner::LinkAcceleration(int link_index) const
	{
		return m_scratch.at(link_index).articulated_bias_or_acceleration;
	}

	// Return compact per-link state for focused phase-reuse and validity diagnostics.
	std::span<GpuArticulationAbaScratch const> ArticulationForceAbaInteropRunner::Scratch() const
	{
		return m_scratch;
	}

	// Return the two spatial factors retained for every generalized joint DOF.
	std::span<GpuArticulationAbaDofScratch const> ArticulationForceAbaInteropRunner::DofScratch() const
	{
		return m_dof_scratch;
	}

	// Return tightly packed active inverse joint-inertia blocks.
	std::span<float const> ArticulationForceAbaInteropRunner::JointMatrixScratch() const
	{
		return m_joint_matrix_scratch;
	}
}
