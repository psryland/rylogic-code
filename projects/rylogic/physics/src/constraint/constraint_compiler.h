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

	// Projection applied to the accumulated impulses of one solver block.
	enum class EConstraintProjection
	{
		Independent,
		FrictionCone,
	};

	// One stable endpoint resolved against the caller-owned objects submitted for the current step.
	struct CompiledConstraintEndpoint
	{
		EConstraintBodyType m_type = EConstraintBodyType::World;
		int m_rigid_index = -1;
		int m_articulation_index = -1;
		int m_link_index = -1;
		int m_packed_body_index = -1;
		LinkHandle m_link = {};

		// True when this endpoint is fixed world space.
		bool IsWorld() const
		{
			return m_type == EConstraintBodyType::World;
		}

		// True when this endpoint is an ordinary rigid body.
		bool IsRigid() const
		{
			return m_type == EConstraintBodyType::Rigid;
		}

		// True when this endpoint belongs to a reduced-coordinate articulation.
		bool IsLink() const
		{
			return m_type == EConstraintBodyType::ArticulationLink;
		}
	};

	// Stable-identity remap and body access for one packed simulation step.
	struct BodyRemap
	{
	private:

		std::vector<RigidBody*> m_bodies;
		std::vector<Articulation*> m_articulations;
		std::unordered_map<uint64_t, int> m_rigid_indices;
		std::unordered_map<uint64_t, int> m_articulation_indices;
		std::vector<int> m_articulation_link_offsets;

	public:

		// Build a remap and reject null pointers, invalid identities, duplicate objects, or malformed links before submission.
		explicit BodyRemap(std::span<RigidBody* const> bodies, std::span<Articulation* const> articulations = {});

		// Resolve a stable endpoint to its current object owner and packed proxy index.
		CompiledConstraintEndpoint ResolveEndpoint(BodyRef body) const;

		// Resolve a stable endpoint to its current packed index, using -1 for fixed world space.
		int Resolve(BodyRef body) const;

		// Return a remapped rigid body by current packed index.
		RigidBody const& Body(int index) const;

		// Return a mutable remapped rigid body by current packed index.
		RigidBody& MutableBody(int index) const;

		// Return a remapped articulation by current packed forest index.
		Articulation const& ArticulationBody(int index) const;

		// Return a mutable remapped articulation by current packed forest index.
		Articulation& MutableArticulation(int index) const;

		// Return the number of remapped rigid bodies.
		int BodyCount() const;

		// Return the number of remapped articulations.
		int ArticulationCount() const;
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
		CompiledConstraintEndpoint m_endpoint_a = {};
		CompiledConstraintEndpoint m_endpoint_b = {};
		uint32_t m_row_begin = 0;
		uint32_t m_row_count = 0;
		float m_break_force = 0.0f;
		float m_break_torque = 0.0f;
		float m_friction = 0.0f;
		EConstraintProjection m_projection = EConstraintProjection::Independent;
		bool m_collide_connected = false;
	};

	// Deterministic active block and scalar-row streams for CPU reference or later GPU upload.
	struct CompiledConstraintSet
	{
		ConstraintSet const* m_source = nullptr;
		uint64_t m_topology_revision = 0;
		uint64_t m_parameter_revision = 0;
		std::vector<CompiledConstraintBlock> m_blocks;
		std::vector<CompiledConstraintRow> m_rows;
	};

	// Resolve endpoints and compile enabled D6 descriptors in stable slot and axis order.
	CompiledConstraintSet CompileConstraints(ConstraintSet const& constraints, BodyRemap const& remap);
}
