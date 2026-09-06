//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "src/compute/interop/articulation_midpoint_runner.h"
#include "src/compute/articulation_midpoint.hlsl"

namespace pr::physics
{
	namespace
	{
		// Return a const span suitable for assigning one emulated shader resource.
		template <typename T> std::span<T const> MidpointSpanOf(std::vector<T> const& values)
		{
			return std::span<T const>{values.data(), values.size()};
		}

		// Return the exact group count for a non-empty articulation range.
		int MidpointThreadGroupCount(int articulation_count)
		{
			return (articulation_count + ArticulationThreadCount - 1) / ArticulationThreadCount;
		}
	}

	// Execute the requested internal substeps and replace packed primary/output state.
	void ArticulationMidpointInteropRunner::Run(GpuArticulationUpload& upload, float dt, int substep_count)
	{
		ValidateGpuArticulationUpload(upload);
		if (!std::isfinite(dt) || dt < 0.0f)
			throw std::invalid_argument("Articulation midpoint replay timestep must be finite and non-negative");
		if (substep_count < 0)
			throw std::invalid_argument("Articulation midpoint replay substep count must be non-negative");

		// Empty optional input performs no emulated dispatch and retains no feature storage.
		if (upload.m_articulations.empty())
		{
			m_articulations = {};
			m_scratch = {};
			m_dof_scratch = {};
			m_joint_matrix_scratch = {};
			m_position_start = {};
			m_velocity_start = {};
			m_midpoint_velocity = {};
			m_states = {};
			return;
		}

		// Allocate the same dimension-scaled persistent ranges used by the production host.
		m_articulations = upload.m_articulations;
		m_scratch.assign(upload.m_links.size(), GpuArticulationAbaScratch{});
		m_dof_scratch.assign(upload.m_dofs.size(), GpuArticulationAbaDofScratch{});
		m_joint_matrix_scratch.assign(upload.m_joint_matrix_scratch_count, 0.0f);
		m_position_start.assign(upload.m_positions.size(), 0.0f);
		m_velocity_start.assign(upload.m_velocities.size(), 0.0f);
		m_midpoint_velocity.assign(upload.m_velocities.size(), 0.0f);
		m_states.clear();
		m_states.reserve(upload.m_articulations.size());
		for (auto const& articulation : upload.m_articulations)
		{
			m_states.push_back(GpuArticulationIntegrationState{
				.root_to_world_start = articulation.root_to_world,
				.status = GpuArticulationIntegrationStatus_Success,
				.iteration_count = 0,
				.residual = 0.0f,
				.pad = 0.0f,
			});
		}

		// Bind topology, immutable forces, writable primary state, canonical ABA scratch, and transaction scratch once.
		g_aba_links.assign(MidpointSpanOf(upload.m_links));
		g_aba_dofs.assign(MidpointSpanOf(upload.m_dofs));
		g_aba_forces.assign(MidpointSpanOf(upload.m_forces));
		g_aba_external_forces.assign(MidpointSpanOf(upload.m_external_forces));
		g_aba_children.assign(MidpointSpanOf(upload.m_children));
		g_aba_articulations.assign(MidpointSpanOf(m_articulations));
		g_aba_positions.assign(MidpointSpanOf(upload.m_positions));
		g_aba_velocities.assign(MidpointSpanOf(upload.m_velocities));
		g_aba_accelerations.assign(MidpointSpanOf(upload.m_accelerations));
		g_aba_scratch.assign(MidpointSpanOf(m_scratch));
		g_aba_dof_scratch.assign(MidpointSpanOf(m_dof_scratch));
		g_aba_inverse_joint_inertia.assign(MidpointSpanOf(m_joint_matrix_scratch));
		g_midpoint_position_start.assign(MidpointSpanOf(m_position_start));
		g_midpoint_velocity_start.assign(MidpointSpanOf(m_velocity_start));
		g_midpoint_velocity.assign(MidpointSpanOf(m_midpoint_velocity));
		g_midpoint_state.assign(MidpointSpanOf(m_states));

		// Every emulated dispatch advances one substep; sticky failure is handled inside each lane.
		g_midpoint = cbArticulationMidpoint{
			.dt = dt,
			.articulation_count = isize(upload.m_articulations),
			.link_count = isize(upload.m_links),
			.velocity_count = isize(upload.m_velocities),
		};
		for (int substep_index = 0; substep_index != substep_count; ++substep_index)
		{
			hlsl::GpuEmulator emulator(CSArticulationMidpoint, CSArticulationMidpoint_NumThreads);
			emulator.Dispatch({MidpointThreadGroupCount(isize(upload.m_articulations)), 1, 1});
		}

		// Recover the same typed result streams exposed by the production readback surface.
		m_articulations.assign(g_aba_articulations.begin(), g_aba_articulations.end());
		upload.m_positions.assign(g_aba_positions.begin(), g_aba_positions.end());
		upload.m_velocities.assign(g_aba_velocities.begin(), g_aba_velocities.end());
		upload.m_accelerations.assign(g_aba_accelerations.begin(), g_aba_accelerations.end());
		m_scratch.assign(g_aba_scratch.begin(), g_aba_scratch.end());
		m_dof_scratch.assign(g_aba_dof_scratch.begin(), g_aba_dof_scratch.end());
		m_joint_matrix_scratch.assign(g_aba_inverse_joint_inertia.begin(), g_aba_inverse_joint_inertia.end());
		m_position_start.assign(g_midpoint_position_start.begin(), g_midpoint_position_start.end());
		m_velocity_start.assign(g_midpoint_velocity_start.begin(), g_midpoint_velocity_start.end());
		m_midpoint_velocity.assign(g_midpoint_velocity.begin(), g_midpoint_velocity.end());
		m_states.assign(g_midpoint_state.begin(), g_midpoint_state.end());
	}

	// Return writable articulation records containing the integrated root frames.
	std::span<GpuArticulation const> ArticulationMidpointInteropRunner::Articulations() const
	{
		return m_articulations;
	}

	// Return explicit sticky status and fixed-point diagnostics for every articulation.
	std::span<GpuArticulationIntegrationState const> ArticulationMidpointInteropRunner::States() const
	{
		return m_states;
	}

	// Return solved link-frame spatial acceleration from phase-reused ABA scratch.
	GpuArticulationSpatialVector const& ArticulationMidpointInteropRunner::LinkAcceleration(int link_index) const
	{
		return m_scratch.at(link_index).articulated_bias_or_acceleration;
	}
}
