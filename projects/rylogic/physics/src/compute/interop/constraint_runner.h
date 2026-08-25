//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/physics/integrator/engine_config.h"
#include "src/constraint/constraint_solver.h"
#include "src/compute/physics_types.h"

namespace pr::physics
{
	// Mutable body state and stable-slot descriptor inputs for one constraint frame.
	struct ConstraintRunnerBuffers
	{
		float m_dt;
		std::span<GpuRigidBody> m_bodies;
		std::span<GpuConstraintEndpoint const> m_endpoints;
		std::span<GpuD6ConstraintDesc const> m_descriptors;
	};

	// Common replay interface shared by the HLSL emulator and future hardware runner.
	struct IConstraintRunner
	{
		virtual ~IConstraintRunner() = default;

		// Compile and solve one complete constraint frame.
		virtual void Run(ConstraintRunnerBuffers buffers) = 0;
	};

	// Deterministic HLSL-as-C++ replay of the persistent D6 shader pipeline.
	struct ConstraintInteropRunner final : IConstraintRunner
	{
		// Construct a replay runner with the CPU reference solver's controls.
		explicit ConstraintInteropRunner(CpuConstraintSolverConfig const& config = {});

		// Construct a replay runner using the engine's iteration counts and reference defaults.
		explicit ConstraintInteropRunner(EngineConfig const& config);

		// Compile and solve one complete constraint frame while retaining runtime rows for warm starts.
		void Run(ConstraintRunnerBuffers buffers) override;

		// Load frame inputs without discarding same-sized persistent runtime storage.
		void Load(ConstraintRunnerBuffers buffers);

		// Store solved body state back to the caller.
		void Store(ConstraintRunnerBuffers buffers) const;

		// Execute the stable-slot GPU row compiler.
		void CompileConstraints();

		// Execute deterministic greedy graph colouring.
		void AssignColours();

		// Apply retained physical impulses for every colour.
		void ApplyWarmStart();

		// Reset per-body pseudo twists before split-position iterations.
		void ClearPseudoVelocity();

		// Execute one complete coloured velocity sweep.
		void SolveVelocity();

		// Execute one complete coloured split-position sweep.
		void SolvePosition();

		// Latch force or torque threshold crossings from the committed canonical row impulses.
		void DetectBreakage();

		// Integrate converged pseudo twists once without changing physical momentum.
		void ApplyPosition();

		// Return stable-slot runtime blocks.
		std::span<GpuConstraintBlock const> Blocks() const;

		// Return six canonical runtime rows per stable slot.
		std::span<GpuConstraintRow const> Rows() const;

		// Return frame-local stable-slot overload latches.
		std::span<GpuConstraintBreakState const> BreakStates() const;

		// Return stable-slot colour assignments.
		std::span<uint32_t const> Colours() const;

		// Return whether this frame selected coherent serial execution.
		bool ColourOverflow() const;

	private:

		CpuConstraintSolverConfig m_config;
		float m_dt;
		float m_previous_dt;
		int m_body_count;
		int m_slot_count;
		std::vector<GpuRigidBody> m_bodies;
		std::vector<GpuConstraintEndpoint> m_endpoints;
		std::vector<GpuD6ConstraintDesc> m_descriptors;
		std::vector<GpuConstraintBlock> m_blocks;
		std::vector<GpuConstraintRow> m_rows;
		std::vector<GpuConstraintBreakState> m_break_states;
		std::vector<GpuConstraintPseudoVelocity> m_pseudo_velocities;
		std::vector<uint32_t> m_colours;
		std::vector<uint32_t> m_colour_overflow;
	};
}
