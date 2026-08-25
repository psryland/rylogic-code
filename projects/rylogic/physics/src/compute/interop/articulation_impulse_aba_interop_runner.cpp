//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "src/compute/interop/articulation_impulse_aba_runner.h"
#include "src/compute/interop/articulation_mobility_runner.h"

// Give this translation unit private emulated resource identities because shader replays reuse canonical ABA names.
#define g_impulse g_impulse_replay
#define g_mobility_ranges g_impulse_replay_mobility_ranges
#define g_aba_articulations g_impulse_replay_articulations
#define g_aba_links g_impulse_replay_links
#define g_aba_dofs g_impulse_replay_dofs
#define g_aba_positions g_impulse_replay_positions
#define g_aba_velocities g_impulse_replay_velocities
#define g_aba_forces g_impulse_replay_forces
#define g_aba_external_forces g_impulse_replay_external_forces
#define g_aba_children g_impulse_replay_children
#define g_aba_accelerations g_impulse_replay_accelerations
#define g_aba_scratch g_impulse_replay_scratch
#define g_aba_dof_scratch g_impulse_replay_dof_scratch
#define g_aba_inverse_joint_inertia g_impulse_replay_inverse_joint_inertia
#define g_link_mobilities g_impulse_replay_link_mobilities
#define g_link_impulses g_impulse_replay_link_impulses
#define g_impulse_work g_impulse_replay_work
#include "src/compute/articulation_impulse_aba.hlsl"

namespace pr::physics
{
	namespace
	{
		// Return a const span suitable for assigning one emulated shader resource.
		template <typename Type>
		std::span<Type const> ImpulseSpanOf(std::vector<Type> const& values)
		{
			return std::span<Type const>{values.data(), values.size()};
		}

		// Return the exact group count for a non-empty participating-articulation range.
		int ImpulseThreadGroupCount(int articulation_count)
		{
			return (articulation_count + ArticulationThreadCount - 1) / ArticulationThreadCount;
		}
	}

	// Rebuild participating-tree factors and apply one link-coordinate impulse per packed link.
	void ArticulationImpulseAbaInteropRunner::Run(
		GpuArticulationUpload const& upload,
		std::span<int const> articulation_indices,
		std::span<GpuArticulationSpatialVector const> link_impulses)
	{
		if (articulation_indices.empty())
		{
			if (!link_impulses.empty())
				throw std::invalid_argument("Articulation impulses require at least one participating articulation");
			m_upload = {};
			m_ranges.clear();
			m_mobilities.clear();
			m_velocities.clear();
			m_accelerations.clear();
			m_scratch.clear();
			m_dof_scratch.clear();
			m_inverse_joint_inertia.clear();
			m_work.clear();
			return;
		}
		// Reuse the canonical mobility replay so factor preparation cannot diverge between self mobility and impulse response.
		auto mobility = ArticulationMobilityInteropRunner{};
		mobility.Run(upload, articulation_indices);
		m_upload = upload;
		m_ranges.assign(mobility.Ranges().begin(), mobility.Ranges().end());
		m_mobilities.assign(mobility.Mobilities().begin(), mobility.Mobilities().end());
		m_velocities = upload.m_velocities;
		m_accelerations.assign(mobility.Accelerations().begin(), mobility.Accelerations().end());
		m_scratch.assign(mobility.Scratch().begin(), mobility.Scratch().end());
		m_dof_scratch.assign(mobility.DofScratch().begin(), mobility.DofScratch().end());
		m_inverse_joint_inertia.assign(mobility.InverseJointInertia().begin(), mobility.InverseJointInertia().end());
		m_work.assign(m_mobilities.size(), GpuArticulationSpatialVector{});
		Apply(link_impulses);
	}

	// Apply another simultaneous impulse through the retained fixed-configuration factors.
	void ArticulationImpulseAbaInteropRunner::Apply(std::span<GpuArticulationSpatialVector const> link_impulses)
	{
		if (m_ranges.empty())
			throw std::runtime_error("Articulation impulse replay has no retained participating factors");
		if (link_impulses.size() != m_mobilities.size())
			throw std::invalid_argument("Articulation impulse replay requires one entry per participating link");

		// Bind immutable packed input and writable response ranges exactly as the hardware pass does.
		g_impulse = cbArticulationImpulseAba{
			.participating_articulation_count = isize(m_ranges),
			.articulation_count = isize(m_upload.m_articulations),
			.link_count = isize(m_upload.m_links),
			.mobility_count = isize(m_work),
		};
		g_mobility_ranges.assign(ImpulseSpanOf(m_ranges));
		g_aba_articulations.assign(ImpulseSpanOf(m_upload.m_articulations));
		g_aba_links.assign(ImpulseSpanOf(m_upload.m_links));
		g_aba_dofs.assign(ImpulseSpanOf(m_upload.m_dofs));
		g_aba_positions.assign(ImpulseSpanOf(m_upload.m_positions));
		g_aba_velocities.assign(ImpulseSpanOf(m_velocities));
		g_aba_forces.assign(ImpulseSpanOf(m_upload.m_forces));
		g_aba_external_forces.assign(ImpulseSpanOf(m_upload.m_external_forces));
		g_aba_children.assign(ImpulseSpanOf(m_upload.m_children));
		g_link_mobilities.assign(ImpulseSpanOf(m_mobilities));
		g_link_impulses.assign(link_impulses);
		g_aba_accelerations.assign(ImpulseSpanOf(m_accelerations));
		g_aba_scratch.assign(ImpulseSpanOf(m_scratch));
		g_aba_dof_scratch.assign(ImpulseSpanOf(m_dof_scratch));
		g_aba_inverse_joint_inertia.assign(ImpulseSpanOf(m_inverse_joint_inertia));
		g_impulse_work.assign(ImpulseSpanOf(m_work));

		hlsl::GpuEmulator emulator(CSArticulationApplyImpulses, CSArticulationApplyImpulses_NumThreads);
		emulator.Dispatch({ImpulseThreadGroupCount(isize(m_ranges)), 1, 1});
		m_velocities.assign(g_aba_velocities.begin(), g_aba_velocities.end());
		m_accelerations.assign(g_aba_accelerations.begin(), g_aba_accelerations.end());
		m_scratch.assign(g_aba_scratch.begin(), g_aba_scratch.end());
		m_dof_scratch.assign(g_aba_dof_scratch.begin(), g_aba_dof_scratch.end());
		m_inverse_joint_inertia.assign(g_aba_inverse_joint_inertia.begin(), g_aba_inverse_joint_inertia.end());
		m_work.assign(g_impulse_work.begin(), g_impulse_work.end());
	}

#undef g_impulse
#undef g_mobility_ranges
#undef g_aba_articulations
#undef g_aba_links
#undef g_aba_dofs
#undef g_aba_positions
#undef g_aba_velocities
#undef g_aba_forces
#undef g_aba_external_forces
#undef g_aba_children
#undef g_aba_accelerations
#undef g_aba_scratch
#undef g_aba_dof_scratch
#undef g_aba_inverse_joint_inertia
#undef g_link_mobilities
#undef g_link_impulses
#undef g_impulse_work

	// Return committed packed generalized velocities after impulse application.
	std::span<float const> ArticulationImpulseAbaInteropRunner::Velocities() const
	{
		return m_velocities;
	}

	// Return retained ABA scratch containing committed cached link velocities.
	std::span<GpuArticulationAbaScratch const> ArticulationImpulseAbaInteropRunner::Scratch() const
	{
		return m_scratch;
	}
}
