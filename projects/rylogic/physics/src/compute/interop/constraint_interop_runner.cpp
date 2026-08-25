//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "src/compute/interop/constraint_runner.h"
#include "src/compute/constraint_solver.hlsl"

namespace pr::physics
{
	namespace
	{
		// Return a const span suitable for assigning an emulated shader resource.
		template <typename T> std::span<T const> ConstraintSpanOf(std::vector<T> const& values)
		{
			return std::span<T const>{values.data(), values.size()};
		}

		// Return the dispatch group count for a fixed-width shader pass.
		int ConstraintThreadGroupCount(int item_count)
		{
			return std::max(1, (item_count + ConstraintThreadCount - 1) / ConstraintThreadCount);
		}

		// Mirror the shader constant-buffer layout for the current replay pass.
		cbConstraintSolver MakeConstraintConstants(CpuConstraintSolverConfig const& config, float dt, float previous_dt, int body_count, int slot_count, int colour = 0, int breakable_count = 0, int break_substep_index = -1)
		{
			auto warm_start_scale = 0.0f;
			if (previous_dt > 0.0f)
			{
				auto const timestep_ratio = dt / previous_dt;
				if (std::isfinite(timestep_ratio) && timestep_ratio >= 0.25f && timestep_ratio <= 4.0f)
					warm_start_scale = config.m_warm_start_factor * timestep_ratio;
			}
			return cbConstraintSolver{
				.slot_count = slot_count,
				.body_count = body_count,
				.colour = colour,
				.position_iterations = config.m_position_iterations,
				.timestep = dt,
				.relaxation = config.m_relaxation,
				.position_relaxation = config.m_position_relaxation,
				.position_beta = config.m_position_beta,
				.max_position_speed = config.m_max_position_speed,
				.regularization = config.m_regularization,
				.warm_start_factor = config.m_warm_start_factor,
				.warm_start_scale = warm_start_scale,
				.breakable_count = breakable_count,
				.break_substep_index = break_substep_index,
			};
		}
	}

	// Construct a replay runner with empty persistent runtime storage.
	ConstraintInteropRunner::ConstraintInteropRunner(CpuConstraintSolverConfig const& config)
		: m_config(config)
		, m_dt()
		, m_previous_dt()
		, m_body_count()
		, m_slot_count()
		, m_bodies()
		, m_endpoints()
		, m_descriptors()
		, m_blocks()
		, m_rows()
		, m_break_states()
		, m_pseudo_velocities()
		, m_colours()
		, m_colour_overflow(1, 0u)
	{
	}

	// Map every public constraint control onto the reference solver so parity tests exercise the production configuration.
	ConstraintInteropRunner::ConstraintInteropRunner(EngineConfig const& config)
		: ConstraintInteropRunner(CpuConstraintSolverConfig{
			.m_velocity_iterations = config.solver_iterations,
			.m_position_iterations = config.push_out_iterations,
			.m_relaxation = config.constraint_relaxation,
			.m_position_relaxation = config.constraint_position_relaxation,
			.m_position_beta = config.constraint_position_beta,
			.m_max_position_speed = config.constraint_max_position_speed,
			.m_regularization = config.constraint_regularization,
			.m_warm_start_factor = config.constraint_warm_start_factor,
		})
	{
	}

	// Execute the same pass and iteration order expected from the D3D12 host.
	void ConstraintInteropRunner::Run(ConstraintRunnerBuffers buffers)
	{
		Load(buffers);
		CompileConstraints();
		AssignColours();
		ApplyWarmStart();
		ClearPseudoVelocity();

		for (int iteration = 0; iteration != m_config.m_position_iterations; ++iteration)
			SolvePosition();
		ApplyPosition();
		for (int iteration = 0; iteration != m_config.m_velocity_iterations; ++iteration)
			SolveVelocity();
		DetectBreakage();

		Store(buffers);
		m_previous_dt = m_dt;
	}

	// Validate and copy frame inputs while preserving same-sized blocks and rows for warm starts.
	void ConstraintInteropRunner::Load(ConstraintRunnerBuffers buffers)
	{
		if (!std::isfinite(buffers.m_dt) || buffers.m_dt <= 0.0f)
			throw std::invalid_argument("ConstraintInteropRunner requires a finite positive timestep");
		if (buffers.m_endpoints.size() != buffers.m_descriptors.size())
			throw std::invalid_argument("Constraint endpoint and descriptor slot counts must match");
		if (m_config.m_velocity_iterations < 0 || m_config.m_position_iterations < 0)
			throw std::invalid_argument("Constraint iteration counts cannot be negative");

		m_dt = buffers.m_dt;
		m_body_count = static_cast<int>(buffers.m_bodies.size());
		m_slot_count = static_cast<int>(buffers.m_endpoints.size());
		m_bodies.assign(buffers.m_bodies.begin(), buffers.m_bodies.end());
		m_endpoints.assign(buffers.m_endpoints.begin(), buffers.m_endpoints.end());
		m_descriptors.assign(buffers.m_descriptors.begin(), buffers.m_descriptors.end());
		m_pseudo_velocities.assign(buffers.m_bodies.size(), GpuConstraintPseudoVelocity{});
		m_break_states.resize(buffers.m_endpoints.size());
		for (int index = 0; index != m_slot_count; ++index)
		{
			m_break_states[index] = GpuConstraintBreakState{
				.generation = m_endpoints[index].generation,
				.substep_index = -1,
			};
		}

		// A topology-size change has no stable row correspondence, so only that case reallocates retained state.
		if (m_blocks.size() != buffers.m_endpoints.size())
		{
			m_blocks.assign(buffers.m_endpoints.size(), GpuConstraintBlock{});
			m_rows.assign(GpuConstraintRowsPerBlock * buffers.m_endpoints.size(), GpuConstraintRow{});
			m_colours.assign(buffers.m_endpoints.size(), MaxColours);
			m_previous_dt = 0.0f;
		}
		m_colour_overflow[0] = 0u;
	}

	// Dispatch one body invocation to clear each per-frame pseudo twist.
	void ConstraintInteropRunner::ClearPseudoVelocity()
	{
		g = MakeConstraintConstants(m_config, m_dt, m_previous_dt, m_body_count, m_slot_count);
		g_constraint_bodies.assign(ConstraintSpanOf(m_bodies));
		g_pseudo_velocities.assign(ConstraintSpanOf(m_pseudo_velocities));

		hlsl::GpuEmulator emulator(CSClearConstraintPseudoVelocity, CSClearConstraintPseudoVelocity_NumThreads);
		emulator.Dispatch({ConstraintThreadGroupCount(m_body_count), 1, 1});
		m_pseudo_velocities.assign(g_pseudo_velocities.begin(), g_pseudo_velocities.end());
	}

	// Copy only body outputs because runtime rows remain owned by the runner.
	void ConstraintInteropRunner::Store(ConstraintRunnerBuffers buffers) const
	{
		if (buffers.m_bodies.size() != m_bodies.size())
			throw std::invalid_argument("ConstraintInteropRunner output body buffer size changed");

		std::copy(m_bodies.begin(), m_bodies.end(), buffers.m_bodies.begin());
	}

	// Dispatch one compiler invocation per stable slot.
	void ConstraintInteropRunner::CompileConstraints()
	{
		g = MakeConstraintConstants(m_config, m_dt, m_previous_dt, m_body_count, m_slot_count);
		g_constraint_bodies.assign(ConstraintSpanOf(m_bodies));
		g_endpoints.assign(ConstraintSpanOf(m_endpoints));
		g_descriptors.assign(ConstraintSpanOf(m_descriptors));
		g_blocks.assign(ConstraintSpanOf(m_blocks));
		g_rows.assign(ConstraintSpanOf(m_rows));

		hlsl::GpuEmulator emulator(CSCompileConstraints, CSCompileConstraints_NumThreads);
		emulator.Dispatch({ConstraintThreadGroupCount(m_slot_count), 1, 1});

		m_blocks.assign(g_blocks.begin(), g_blocks.end());
		m_rows.assign(g_rows.begin(), g_rows.end());
	}

	// Dispatch the serial deterministic colouring pass.
	void ConstraintInteropRunner::AssignColours()
	{
		g = MakeConstraintConstants(m_config, m_dt, m_previous_dt, m_body_count, m_slot_count);
		g_constraint_bodies.assign(ConstraintSpanOf(m_bodies));
		g_blocks.assign(ConstraintSpanOf(m_blocks));
		g_colour_overflow.assign(ConstraintSpanOf(m_colour_overflow));

		hlsl::GpuEmulator emulator(CSAssignConstraintColours, CSAssignConstraintColours_NumThreads);
		emulator.Dispatch({1, 1, 1});

		m_bodies.assign(g_constraint_bodies.begin(), g_constraint_bodies.end());
		m_blocks.assign(g_blocks.begin(), g_blocks.end());
		m_colour_overflow.assign(g_colour_overflow.begin(), g_colour_overflow.end());
		for (int slot_idx = 0; slot_idx != m_slot_count; ++slot_idx)
			m_colours[slot_idx] = m_blocks[slot_idx].colour;
	}

	// Dispatch every colour so the shader can select either parallel batches or the colour-zero serial fallback.
	void ConstraintInteropRunner::ApplyWarmStart()
	{
		for (int colour = 0; colour != MaxColours; ++colour)
		{
			g = MakeConstraintConstants(m_config, m_dt, m_previous_dt, m_body_count, m_slot_count, colour);
			g_constraint_bodies.assign(ConstraintSpanOf(m_bodies));
			g_blocks.assign(ConstraintSpanOf(m_blocks));
			g_rows.assign(ConstraintSpanOf(m_rows));
			g_colour_overflow.assign(ConstraintSpanOf(m_colour_overflow));

			hlsl::GpuEmulator emulator(CSApplyConstraintWarmStart, CSApplyConstraintWarmStart_NumThreads);
			emulator.Dispatch({ConstraintThreadGroupCount(m_slot_count), 1, 1});
			m_bodies.assign(g_constraint_bodies.begin(), g_constraint_bodies.end());
		}
	}

	// Execute one fixed-order coloured physical PGS sweep.
	void ConstraintInteropRunner::SolveVelocity()
	{
		for (int colour = 0; colour != MaxColours; ++colour)
		{
			g = MakeConstraintConstants(m_config, m_dt, m_previous_dt, m_body_count, m_slot_count, colour);
			g_constraint_bodies.assign(ConstraintSpanOf(m_bodies));
			g_blocks.assign(ConstraintSpanOf(m_blocks));
			g_rows.assign(ConstraintSpanOf(m_rows));
			g_colour_overflow.assign(ConstraintSpanOf(m_colour_overflow));

			hlsl::GpuEmulator emulator(CSSolveConstraintVelocity, CSSolveConstraintVelocity_NumThreads);
			emulator.Dispatch({ConstraintThreadGroupCount(m_slot_count), 1, 1});
			m_bodies.assign(g_constraint_bodies.begin(), g_constraint_bodies.end());
			m_rows.assign(g_rows.begin(), g_rows.end());
		}
	}

	// Latch force or torque threshold crossings from the committed canonical row impulses.
	void ConstraintInteropRunner::DetectBreakage()
	{
		auto const breakable_count = static_cast<int>(std::ranges::count_if(m_endpoints, [](GpuConstraintEndpoint const& endpoint)
		{
			return
				AllSet(endpoint.flags, GpuConstraintEndpointFlags_Enabled) &&
				(std::isfinite(endpoint.break_force) || std::isfinite(endpoint.break_torque));
		}));
		if (breakable_count == 0)
			return;

		g = MakeConstraintConstants(m_config, m_dt, m_previous_dt, m_body_count, m_slot_count, 0, breakable_count, 0);
		g_endpoints.assign(ConstraintSpanOf(m_endpoints));
		g_blocks.assign(ConstraintSpanOf(m_blocks));
		g_rows.assign(ConstraintSpanOf(m_rows));
		g_break_states.assign(ConstraintSpanOf(m_break_states));

		hlsl::GpuEmulator emulator(CSDetectBrokenConstraints, CSDetectBrokenConstraints_NumThreads);
		emulator.Dispatch({ConstraintThreadGroupCount(m_slot_count), 1, 1});
		m_blocks.assign(g_blocks.begin(), g_blocks.end());
		m_break_states.assign(g_break_states.begin(), g_break_states.end());
	}

	// Execute one fixed-order coloured split-position sweep.
	void ConstraintInteropRunner::SolvePosition()
	{
		for (int colour = 0; colour != MaxColours; ++colour)
		{
			g = MakeConstraintConstants(m_config, m_dt, m_previous_dt, m_body_count, m_slot_count, colour);
			g_constraint_bodies.assign(ConstraintSpanOf(m_bodies));
			g_blocks.assign(ConstraintSpanOf(m_blocks));
			g_rows.assign(ConstraintSpanOf(m_rows));
			g_colour_overflow.assign(ConstraintSpanOf(m_colour_overflow));
			g_pseudo_velocities.assign(ConstraintSpanOf(m_pseudo_velocities));

			hlsl::GpuEmulator emulator(CSSolveConstraintPosition, CSSolveConstraintPosition_NumThreads);
			emulator.Dispatch({ConstraintThreadGroupCount(m_slot_count), 1, 1});
			m_rows.assign(g_rows.begin(), g_rows.end());
			m_pseudo_velocities.assign(g_pseudo_velocities.begin(), g_pseudo_velocities.end());
		}
	}

	// Dispatch one body invocation to apply each converged pseudo twist once.
	void ConstraintInteropRunner::ApplyPosition()
	{
		g = MakeConstraintConstants(m_config, m_dt, m_previous_dt, m_body_count, m_slot_count);
		g_constraint_bodies.assign(ConstraintSpanOf(m_bodies));
		g_pseudo_velocities.assign(ConstraintSpanOf(m_pseudo_velocities));

		hlsl::GpuEmulator emulator(CSApplyConstraintPosition, CSApplyConstraintPosition_NumThreads);
		emulator.Dispatch({ConstraintThreadGroupCount(m_body_count), 1, 1});
		m_bodies.assign(g_constraint_bodies.begin(), g_constraint_bodies.end());
	}

	// Return stable-slot runtime blocks without exposing mutable ownership.
	std::span<GpuConstraintBlock const> ConstraintInteropRunner::Blocks() const
	{
		return m_blocks;
	}

	// Return fixed canonical runtime rows without exposing mutable ownership.
	std::span<GpuConstraintRow const> ConstraintInteropRunner::Rows() const
	{
		return m_rows;
	}

	// Return frame-local stable-slot overload latches.
	std::span<GpuConstraintBreakState const> ConstraintInteropRunner::BreakStates() const
	{
		return m_break_states;
	}

	// Return the colour assignment materialized by the latest colouring pass.
	std::span<uint32_t const> ConstraintInteropRunner::Colours() const
	{
		return m_colours;
	}

	// Return the dedicated one-element overflow buffer's state.
	bool ConstraintInteropRunner::ColourOverflow() const
	{
		return m_colour_overflow[0] != 0u;
	}
}
