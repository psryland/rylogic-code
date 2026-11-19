//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/physics/forward.h"
#include "src/constraint/constraint_gpu.h"

namespace pr::physics
{
	// Deterministic C++ replay of the coupled D6 row and exact-self preconditioner kernel.
	struct CoupledConstraintPrepareInteropRunner
	{
	private:

		std::vector<GpuRigidBody> m_bodies;
		std::vector<GpuConstraintBlock> m_blocks;
		std::vector<GpuConstraintRow> m_rows;
		std::vector<GpuCoupledConstraintPreconditioner> m_preconditioners;

	public:

		// Compile coupled slots at the supplied pose, preserving ordinary slots in the shared prepared streams.
		void Run(
			float timestep,
			float regularization,
			GpuConstraintUpload const& upload,
			std::span<GpuRigidBody const> bodies,
			std::span<GpuConstraintFrame const> link_to_world,
			std::span<GpuArticulationSpatialMobility const> mobilities,
			std::span<GpuArticulationAbaScratch const> aba_scratch,
			std::span<GpuConstraintBlock const> prepared_blocks = {},
			std::span<GpuConstraintRow const> prepared_rows = {});

		// Replace physical preconditioners with exact hard-passive inverses for detached position correction.
		void PreparePositionPreconditioners();

		// Return compiled stable-slot runtime blocks.
		std::span<GpuConstraintBlock const> Blocks() const;

		// Return six canonical runtime rows per stable slot.
		std::span<GpuConstraintRow const> Rows() const;

		// Return one packed exact-self inverse per stable slot.
		std::span<GpuCoupledConstraintPreconditioner const> Preconditioners() const;
	};
}
