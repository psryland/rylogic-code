//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "src/compute/interop/coupled_constraint_prepare_runner.h"

// Give this translation unit private emulated resource identities because shader replays share common constraint names.
#define g_coupled_prepare g_coupled_prepare_replay
#define g_coupled_bodies g_coupled_replay_bodies
#define g_coupled_constraint_endpoints g_coupled_replay_constraint_endpoints
#define g_coupled_descriptors g_coupled_replay_descriptors
#define g_coupled_link_endpoints g_coupled_replay_link_endpoints
#define g_coupled_link_to_world g_coupled_replay_link_to_world
#define g_coupled_link_mobilities g_coupled_replay_link_mobilities
#define g_coupled_aba_scratch g_coupled_replay_aba_scratch
#define g_coupled_blocks g_coupled_replay_blocks
#define g_coupled_rows g_coupled_replay_rows
#define g_coupled_preconditioners g_coupled_replay_preconditioners
#include "src/compute/coupled_constraint_prepare.hlsl"

namespace pr::physics
{
	namespace
	{
		// Return a const span suitable for assigning one emulated shader resource.
		template <typename Type>
		std::span<Type const> CoupledSpanOf(std::vector<Type> const& values)
		{
			return std::span<Type const>{values.data(), values.size()};
		}
	}

	// Compile one stable constraint stream using caller-provided final-configuration link factors.
	void CoupledConstraintPrepareInteropRunner::Run(
		float timestep,
		float regularization,
		float warm_start_scale,
		GpuConstraintUpload const& upload,
		std::span<GpuRigidBody const> bodies,
		std::span<GpuConstraintFrame const> link_to_world,
		std::span<GpuArticulationSpatialMobility const> mobilities,
		std::span<GpuArticulationAbaScratch const> aba_scratch,
		std::span<GpuConstraintBlock const> retained_blocks,
		std::span<GpuConstraintRow const> retained_rows)
	{
		if (upload.m_coupled_endpoints.size() != upload.m_endpoints.size())
			throw std::invalid_argument("Coupled replay requires stable-slot link metadata");
		if (link_to_world.size() != aba_scratch.size())
			throw std::invalid_argument("Coupled replay link frames and ABA scratch counts must match");
		if (!(timestep > 0.0f) || !std::isfinite(timestep))
			throw std::invalid_argument("Coupled replay requires a finite positive timestep");

		auto const slot_count = isize(upload.m_endpoints);
		if ((!retained_blocks.empty() || !retained_rows.empty()) &&
			(isize(retained_blocks) != slot_count || isize(retained_rows) != GpuConstraintRowsPerBlock * slot_count))
			throw std::invalid_argument("Coupled replay retained runtime streams do not match the stable-slot layout");

		m_bodies.assign(bodies.begin(), bodies.end());
		m_blocks.assign(retained_blocks.begin(), retained_blocks.end());
		m_rows.assign(retained_rows.begin(), retained_rows.end());
		m_blocks.resize(slot_count);
		m_rows.resize(GpuConstraintRowsPerBlock * slot_count);
		m_preconditioners.resize(slot_count);

		// Bind immutable packed input and retained stable-slot output exactly as the hardware pass does.
		g_coupled_prepare = cbCoupledConstraintPrepare{
			.slot_count = slot_count,
			.body_count = isize(bodies),
			.link_count = isize(link_to_world),
			.mobility_count = isize(mobilities),
			.timestep = timestep,
			.regularization = regularization,
			.warm_start_scale = warm_start_scale,
			.pad0 = 0.0f,
		};
		g_coupled_bodies.assign(CoupledSpanOf(m_bodies));
		g_coupled_constraint_endpoints.assign(std::span<GpuConstraintEndpoint const>{upload.m_endpoints});
		g_coupled_descriptors.assign(std::span<GpuD6ConstraintDesc const>{upload.m_descriptors});
		g_coupled_link_endpoints.assign(std::span<GpuCoupledConstraintEndpoint const>{upload.m_coupled_endpoints});
		g_coupled_link_to_world.assign(link_to_world);
		g_coupled_link_mobilities.assign(mobilities);
		g_coupled_aba_scratch.assign(aba_scratch);
		g_coupled_blocks.assign(CoupledSpanOf(m_blocks));
		g_coupled_rows.assign(CoupledSpanOf(m_rows));
		g_coupled_preconditioners.assign(CoupledSpanOf(m_preconditioners));

		auto const group_count = std::max(1, (slot_count + ConstraintThreadCount - 1) / ConstraintThreadCount);
		hlsl::GpuEmulator emulator(CSPrepareCoupledConstraints, CSPrepareCoupledConstraints_NumThreads);
		emulator.Dispatch({group_count, 1, 1});
		m_bodies.assign(g_coupled_bodies.begin(), g_coupled_bodies.end());
		m_blocks.assign(g_coupled_blocks.begin(), g_coupled_blocks.end());
		m_rows.assign(g_coupled_rows.begin(), g_coupled_rows.end());
		m_preconditioners.assign(g_coupled_preconditioners.begin(), g_coupled_preconditioners.end());
	}

	// Replace physical preconditioners with exact hard-passive inverses for detached position correction.
	void CoupledConstraintPrepareInteropRunner::PreparePositionPreconditioners()
	{
		if (m_blocks.empty())
			return;

		g_coupled_blocks.assign(CoupledSpanOf(m_blocks));
		g_coupled_rows.assign(CoupledSpanOf(m_rows));
		g_coupled_preconditioners.assign(CoupledSpanOf(m_preconditioners));
		auto const group_count = std::max(1, (isize(m_blocks) + ConstraintThreadCount - 1) / ConstraintThreadCount);
		hlsl::GpuEmulator emulator(CSPrepareCoupledPositionPreconditioners, CSPrepareCoupledPositionPreconditioners_NumThreads);
		emulator.Dispatch({group_count, 1, 1});
		m_blocks.assign(g_coupled_blocks.begin(), g_coupled_blocks.end());
		m_preconditioners.assign(g_coupled_preconditioners.begin(), g_coupled_preconditioners.end());
	}

	// Return compiled stable-slot runtime blocks.
	std::span<GpuConstraintBlock const> CoupledConstraintPrepareInteropRunner::Blocks() const
	{
		return m_blocks;
	}

	// Return six canonical runtime rows per stable slot.
	std::span<GpuConstraintRow const> CoupledConstraintPrepareInteropRunner::Rows() const
	{
		return m_rows;
	}

	// Return one packed exact-self inverse per stable slot.
	std::span<GpuCoupledConstraintPreconditioner const> CoupledConstraintPrepareInteropRunner::Preconditioners() const
	{
		return m_preconditioners;
	}
}

#undef g_coupled_prepare
#undef g_coupled_bodies
#undef g_coupled_constraint_endpoints
#undef g_coupled_descriptors
#undef g_coupled_link_endpoints
#undef g_coupled_link_to_world
#undef g_coupled_link_mobilities
#undef g_coupled_aba_scratch
#undef g_coupled_blocks
#undef g_coupled_rows
#undef g_coupled_preconditioners
