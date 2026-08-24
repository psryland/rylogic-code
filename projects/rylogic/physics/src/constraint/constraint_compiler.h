//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/physics/constraint/constraint_set.h"

namespace pr::physics
{
	// Identifies whether a compiled scalar row constrains translation or rotation.
	enum class EConstraintRowKind
	{
		Linear,
		Angular,
	};

	// Stable-identity remap and body access for one packed simulation step.
	struct BodyRemap
	{
	private:

		std::vector<RigidBody const*> m_bodies;
		std::unordered_map<uint64_t, int> m_rigid_indices;

	public:

		// Build a remap and reject null pointers, invalid identities, or duplicates before submission.
		explicit BodyRemap(std::span<RigidBody* const> bodies);

		// Resolve a stable endpoint to its current packed index, using -1 for fixed world space.
		int Resolve(BodyRef body) const;

		// Return a remapped rigid body by current packed index.
		RigidBody const& Body(int index) const;

		// Return the number of remapped rigid bodies.
		int BodyCount() const;
	};

	// One deterministic scalar row compiled from an active D6 axis.
	struct CompiledConstraintRow
	{
		uint32_t m_block_index = 0;
		EConstraintRowKind m_kind = EConstraintRowKind::Linear;
		uint8_t m_axis = 0;
		EConstraintAxisMode m_mode = EConstraintAxisMode::Free;
		v8force m_jacobian_a = {};
		v8force m_jacobian_b = {};
		float m_position = 0.0f;
		Range<float> m_limits = {};
		float m_target_position = 0.0f;
		float m_target_velocity = 0.0f;
		float m_stiffness = 0.0f;
		float m_damping = 0.0f;
		float m_max_force = 0.0f;
	};

	// Header for one persistent constraint block and its contiguous scalar row range.
	struct CompiledConstraintBlock
	{
		ConstraintHandle m_source = {};
		int m_body_index_a = -1;
		int m_body_index_b = -1;
		uint32_t m_row_begin = 0;
		uint32_t m_row_count = 0;
		float m_break_force = 0.0f;
		float m_break_torque = 0.0f;
		bool m_collide_connected = false;
	};

	// Deterministic active block and scalar-row streams for CPU reference or later GPU upload.
	struct CompiledConstraintSet
	{
		std::vector<CompiledConstraintBlock> m_blocks;
		std::vector<CompiledConstraintRow> m_rows;
	};

	// Resolve endpoints and compile enabled D6 descriptors in stable slot and axis order.
	CompiledConstraintSet CompileConstraints(ConstraintSet const& constraints, BodyRemap const& remap);
}
