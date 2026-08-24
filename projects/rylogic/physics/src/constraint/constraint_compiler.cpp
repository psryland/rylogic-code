//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "pr/physics/rigid_body/rigid_body.h"
#include "src/constraint/constraint_compiler.h"

namespace pr::physics
{
	namespace
	{
		// Return an endpoint's constraint frame in world space.
		m4x4 ConstraintToWorld(BodyFrame const& frame, int body_index, BodyRemap const& remap)
		{
			return body_index >= 0
				? remap.Body(body_index).O2W() * frame.m_constraint_to_body
				: frame.m_constraint_to_body;
		}

		// Return the endpoint Jacobian for linear speed along one world-space axis at an anchor point.
		v8force LinearJacobian(v4 axis_ws, v4 anchor_offset_ws, float sign)
		{
			return v8force{
				sign * Cross(anchor_offset_ws, axis_ws),
				sign * axis_ws,
			};
		}

		// Return the endpoint Jacobian for angular speed about one world-space axis.
		v8force AngularJacobian(v4 axis_ws, float sign)
		{
			return v8force{
				sign * axis_ws,
				v4::Zero(),
			};
		}

		// Return the endpoint anchor offset from the centre of mass used by the body's spatial velocity.
		v4 AnchorOffset(m4x4 const& constraint_to_world, int body_index, BodyRemap const& remap)
		{
			return body_index >= 0
				? (constraint_to_world.pos - remap.Body(body_index).CentreOfMassPositionWS()).w0()
				: v4::Zero();
		}

		// Append one non-free axis while preserving the canonical X, Y, Z order.
		void AppendRow(
			CompiledConstraintSet& compiled,
			EConstraintRowKind kind,
			uint8_t axis_index,
			ConstraintAxisDesc const& axis_desc,
			v4 axis_ws,
			v4 anchor_offset_a,
			v4 anchor_offset_b,
			v4 linear_error_ws,
			v4 angular_error_ws,
			int body_index_a,
			int body_index_b)
		{
			if (axis_desc.m_mode == EConstraintAxisMode::Free)
				return;

			auto jacobian_a = v8force::Zero();
			auto jacobian_b = v8force::Zero();
			auto position = 0.0f;
			switch (kind)
			{
				case EConstraintRowKind::Linear:
				{
					if (body_index_a >= 0)
						jacobian_a = LinearJacobian(axis_ws, anchor_offset_a, -1.0f);
					if (body_index_b >= 0)
						jacobian_b = LinearJacobian(axis_ws, anchor_offset_b, +1.0f);
					position = Dot3(linear_error_ws, axis_ws);
					break;
				}
				case EConstraintRowKind::Angular:
				{
					if (body_index_a >= 0)
						jacobian_a = AngularJacobian(axis_ws, -1.0f);
					if (body_index_b >= 0)
						jacobian_b = AngularJacobian(axis_ws, +1.0f);
					position = Dot3(angular_error_ws, axis_ws);
					break;
				}
				default:
				{
					throw std::invalid_argument("Unknown compiled constraint row kind");
				}
			}

			compiled.m_rows.push_back(CompiledConstraintRow{
				.m_block_index = s_cast<uint32_t>(compiled.m_blocks.size()),
				.m_kind = kind,
				.m_axis = axis_index,
				.m_mode = axis_desc.m_mode,
				.m_jacobian_a = jacobian_a,
				.m_jacobian_b = jacobian_b,
				.m_position = position,
				.m_limits = axis_desc.m_limits,
				.m_target_position = axis_desc.m_target_position,
				.m_target_velocity = axis_desc.m_target_velocity,
				.m_stiffness = axis_desc.m_stiffness,
				.m_damping = axis_desc.m_damping,
				.m_max_force = axis_desc.m_max_force,
			});
		}
	}

	// Build a remap and reject null pointers, invalid identities, or duplicates before submission.
	BodyRemap::BodyRemap(std::span<RigidBody* const> bodies)
		: m_bodies()
		, m_rigid_indices()
	{
		m_bodies.reserve(bodies.size());
		m_rigid_indices.reserve(bodies.size());
		for (int index = 0; index != isize(bodies); ++index)
		{
			auto* body = bodies[index];
			if (body == nullptr)
				throw std::invalid_argument("Body remap cannot contain a null rigid-body pointer");
			if (!body->Id())
				throw std::invalid_argument("Body remap cannot contain an invalid rigid-body identity");

			auto const [iter, inserted] = m_rigid_indices.emplace(body->Id().m_value, index);
			if (!inserted)
				throw std::invalid_argument("Body remap contains duplicate rigid-body identities");

			m_bodies.push_back(body);
		}
	}

	// Resolve a stable endpoint to its current packed index, using -1 for fixed world space.
	int BodyRemap::Resolve(BodyRef body) const
	{
		switch (body.m_type)
		{
			case EConstraintBodyType::World:
				{
					if (body.m_body_id)
						throw std::invalid_argument("World constraint endpoint cannot carry a rigid-body identity");
					return -1;
				}
			case EConstraintBodyType::Rigid:
				{
					auto const iter = m_rigid_indices.find(body.m_body_id.m_value);
					if (iter == m_rigid_indices.end())
						throw std::invalid_argument("Constraint endpoint rigid body is missing from the current step");
					return iter->second;
				}
			default:
				{
					throw std::invalid_argument("Unknown constraint endpoint type");
				}
		}
	}

	// Return a remapped rigid body by current packed index.
	RigidBody const& BodyRemap::Body(int index) const
	{
		if (index < 0 || index >= isize(m_bodies))
			throw std::out_of_range("Remapped rigid-body index is out of range");

		return *m_bodies[index];
	}

	// Return a mutable remapped rigid body by current packed index.
	RigidBody& BodyRemap::MutableBody(int index) const
	{
		if (index < 0 || index >= isize(m_bodies))
			throw std::out_of_range("Remapped rigid-body index is out of range");

		return *m_bodies[index];
	}

	// Return the number of remapped rigid bodies.
	int BodyRemap::BodyCount() const
	{
		return isize(m_bodies);
	}

	// Resolve endpoints and compile enabled D6 descriptors in stable slot and axis order.
	CompiledConstraintSet CompileConstraints(ConstraintSet const& constraints, BodyRemap const& remap)
	{
		auto compiled = CompiledConstraintSet{
			.m_source = &constraints,
			.m_topology_revision = constraints.TopologyRevision(),
			.m_parameter_revision = constraints.ParameterRevision(),
		};
		compiled.m_blocks.reserve(constraints.m_count);
		compiled.m_rows.reserve(6 * constraints.m_count);

		// Slot iteration, followed by linear and angular XYZ iteration, defines reproducible row identity.
		for (uint32_t slot_index = 0; slot_index != constraints.m_slots.size(); ++slot_index)
		{
			auto const& slot = constraints.m_slots[slot_index];
			if (!slot.m_occupied || !slot.m_desc.m_enabled)
				continue;

			auto const& desc = slot.m_desc;
			auto const body_index_a = remap.Resolve(desc.m_frame_a.m_body);
			auto const body_index_b = remap.Resolve(desc.m_frame_b.m_body);
			auto const constraint_to_world_a = ConstraintToWorld(desc.m_frame_a, body_index_a, remap);
			auto const constraint_to_world_b = ConstraintToWorld(desc.m_frame_b, body_index_b, remap);
			auto const axis_ws = std::array{
				constraint_to_world_a.x.w0(),
				constraint_to_world_a.y.w0(),
				constraint_to_world_a.z.w0(),
			};
			auto const linear_error_ws = (constraint_to_world_b.pos - constraint_to_world_a.pos).w0();
			auto const relative_rotation = constraint_to_world_b.rot * Transpose(constraint_to_world_a.rot);
			auto const [angular_error_axis, angular_error_angle] = AxisAngle(relative_rotation);
			auto const angular_error_ws = angular_error_angle * angular_error_axis.w0();
			auto const anchor_offset_a = AnchorOffset(constraint_to_world_a, body_index_a, remap);
			auto const anchor_offset_b = AnchorOffset(constraint_to_world_b, body_index_b, remap);

			auto const row_begin = s_cast<uint32_t>(compiled.m_rows.size());
			for (uint8_t axis_index = 0; axis_index != 3; ++axis_index)
				AppendRow(compiled, EConstraintRowKind::Linear, axis_index, desc.m_linear[axis_index], axis_ws[axis_index], anchor_offset_a, anchor_offset_b, linear_error_ws, angular_error_ws, body_index_a, body_index_b);
			for (uint8_t axis_index = 0; axis_index != 3; ++axis_index)
				AppendRow(compiled, EConstraintRowKind::Angular, axis_index, desc.m_angular[axis_index], axis_ws[axis_index], anchor_offset_a, anchor_offset_b, linear_error_ws, angular_error_ws, body_index_a, body_index_b);

			// An enabled all-free descriptor has no active solver work and therefore emits no block.
			if (compiled.m_rows.size() == row_begin)
				continue;

			compiled.m_blocks.push_back(CompiledConstraintBlock{
				.m_source = ConstraintHandle{
					.m_index = slot_index,
					.m_generation = slot.m_generation,
				},
				.m_body_index_a = body_index_a,
				.m_body_index_b = body_index_b,
				.m_row_begin = row_begin,
				.m_row_count = s_cast<uint32_t>(compiled.m_rows.size()) - row_begin,
				.m_break_force = desc.m_break_force,
				.m_break_torque = desc.m_break_torque,
				.m_collide_connected = desc.m_collide_connected,
			});
		}
		return compiled;
	}
}
