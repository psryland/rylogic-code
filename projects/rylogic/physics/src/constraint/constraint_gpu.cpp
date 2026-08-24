//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "pr/physics/rigid_body/rigid_body.h"
#include "src/constraint/constraint_gpu.h"

namespace pr::physics
{
	namespace
	{
		// Convert an endpoint-local matrix to the compact quaternion-and-position GPU representation.
		GpuConstraintFrame PackFrame(m4x4 const& constraint_to_body)
		{
			auto const rotation = ToQuat<quat>(constraint_to_body.rot);
			return GpuConstraintFrame{
				.rotation = float4{rotation.x, rotation.y, rotation.z, rotation.w},
				.position = constraint_to_body.pos,
			};
		}

		// Convert one public D6 axis descriptor without changing its canonical enum values.
		GpuConstraintAxisDesc PackAxis(ConstraintAxisDesc const& axis)
		{
			return GpuConstraintAxisDesc{
				.mode = static_cast<int>(axis.m_mode),
				.lower_limit = axis.m_limits.m_beg,
				.upper_limit = axis.m_limits.m_end,
				.target_position = axis.m_target_position,
				.target_velocity = axis.m_target_velocity,
				.stiffness = axis.m_stiffness,
				.damping = axis.m_damping,
				.max_force = axis.m_max_force,
			};
		}

		// Preserve linear XYZ followed by angular XYZ so runtime rows have stable canonical identities.
		GpuD6ConstraintDesc PackDescriptor(D6ConstraintDesc const& desc)
		{
			auto packed = GpuD6ConstraintDesc{
				.frame_a = PackFrame(desc.m_frame_a.m_constraint_to_body),
				.frame_b = PackFrame(desc.m_frame_b.m_constraint_to_body),
			};
			for (int axis = 0; axis != 3; ++axis)
			{
				packed.axes[axis] = PackAxis(desc.m_linear[axis]);
				packed.axes[3 + axis] = PackAxis(desc.m_angular[axis]);
			}
			return packed;
		}
	}

	// Pack persistent descriptors and resolve enabled endpoints into current frame-local rigid-body indices.
	GpuConstraintUpload PackGpuConstraints(ConstraintSet const& constraints, BodyRemap const& remap)
	{
		auto upload = GpuConstraintUpload{
			.m_topology_revision = constraints.m_topology_revision,
			.m_parameter_revision = constraints.m_parameter_revision,
		};
		upload.m_endpoints.resize(constraints.m_slots.size());
		upload.m_descriptors.resize(constraints.m_slots.size());

		// Slot-preserving output lets removed descriptors become disabled tombstones without changing any surviving row or warm-start identity.
		for (uint32_t slot_index = 0; slot_index != constraints.m_slots.size(); ++slot_index)
		{
			auto const& slot = constraints.m_slots[slot_index];
			if (!slot.m_occupied)
			{
				upload.m_endpoints[slot_index].generation = slot.m_generation;
				continue;
			}

			auto const& desc = slot.m_desc;
			upload.m_descriptors[slot_index] = PackDescriptor(desc);
			auto flags = desc.m_collide_connected ? GpuConstraintEndpointFlags_CollideConnected : GpuConstraintEndpointFlags_None;
			if (desc.m_enabled)
			{
				flags |= GpuConstraintEndpointFlags_Enabled;
				++upload.m_active_count;
			}

			// Disabled constraints deliberately do not require their bodies to be submitted until they are re-enabled.
			auto const body_idx_a = desc.m_enabled ? remap.Resolve(desc.m_frame_a.m_body) : -1;
			auto const body_idx_b = desc.m_enabled ? remap.Resolve(desc.m_frame_b.m_body) : -1;
			upload.m_endpoints[slot_index] = GpuConstraintEndpoint{
				.body_idx_a = body_idx_a,
				.body_idx_b = body_idx_b,
				.flags = flags,
				.generation = slot.m_generation,
				.break_force = desc.m_break_force,
				.break_torque = desc.m_break_torque,
			};
		}
		return upload;
	}
}
