//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "src/articulation/articulation_gpu_data.h"

namespace pr::physics
{
	namespace
	{
		// Preserve the public root-type semantics in the shader ABI and reject corrupted enum values.
		int PackRootType(EArticulationRootType root_type)
		{
			switch (root_type)
			{
				case EArticulationRootType::Fixed:
				{
					return GpuArticulationRootType_Fixed;
				}
				case EArticulationRootType::Floating:
				{
					return GpuArticulationRootType_Floating;
				}
				default:
				{
					throw std::invalid_argument("Articulation root type cannot be represented on the GPU");
				}
			}
		}

		// Preserve the public scalar-axis semantics in the packed float4 representation.
		GpuArticulationDof PackDof(ArticulationAxisDesc const& axis)
		{
			auto axis_type = 0;
			switch (axis.m_type)
			{
				case EArticulationAxisType::Revolute:
				{
					axis_type = GpuArticulationAxisType_Revolute;
					break;
				}
				case EArticulationAxisType::Prismatic:
				{
					axis_type = GpuArticulationAxisType_Prismatic;
					break;
				}
				default:
				{
					throw std::invalid_argument("Articulation axis type cannot be represented on the GPU");
				}
			}
			return GpuArticulationDof{
				.axis_and_type = float4{axis.m_axis.x, axis.m_axis.y, axis.m_axis.z, static_cast<float>(axis_type)},
			};
		}

		// Append one padded spatial vector in angular-then-linear generalized order.
		void AppendSpatial(std::vector<float>& values, v8motion motion)
		{
			values.insert(values.end(), {
				motion.ang.x,
				motion.ang.y,
				motion.ang.z,
				motion.lin.x,
				motion.lin.y,
				motion.lin.z,
			});
		}

		// Append one padded spatial force in angular-then-linear generalized order.
		void AppendSpatial(std::vector<float>& values, v8force force)
		{
			AppendSpatial(values, v8motion{force.ang, force.lin});
		}

		// Return one GPU spatial-force record without changing its link-frame convention.
		GpuFrameForce PackSpatialForce(v8force force)
		{
			return GpuFrameForce{
				.force_ang = force.ang,
				.force_lin = force.lin,
			};
		}

		// Return true when a non-negative packed subrange fits inside its owning stream.
		bool PackedRangeValid(int offset, int count, size_t size)
		{
			return offset >= 0 && count >= 0 && static_cast<uint64_t>(offset) + static_cast<uint64_t>(count) <= size;
		}

		// Return the generalized root width for one validated packed root-type encoding.
		int PackedRootVelocityCount(int root_type)
		{
			switch (root_type)
			{
				case GpuArticulationRootType_Fixed:
				{
					return 0;
				}
				case GpuArticulationRootType_Floating:
				{
					return 6;
				}
				default:
				{
					throw std::invalid_argument("GPU articulation root type is invalid");
				}
			}
		}
	}

	// Validate and flatten independent articulation trees into deterministic linear GPU ranges and traversal schedules.
	GpuArticulationUpload PackGpuArticulations(std::span<Articulation* const> articulations)
	{
		auto upload = GpuArticulationUpload{};
		auto identities = std::unordered_set<uint64_t>{};
		auto maximum_depth = 0;

		// Pack every tree in caller order while preserving each tree's parent-before-child link order.
		for (auto* articulation : articulations)
		{
			if (articulation == nullptr)
				throw std::invalid_argument("GPU articulation submissions cannot contain null pointers");

			auto const identity = articulation->Id().m_value;
			if (!identities.insert(identity).second)
				throw std::invalid_argument("GPU articulation submissions cannot contain duplicate identities");

			auto const articulation_index = isize(upload.m_articulations);
			auto const link_offset = isize(upload.m_links);
			auto const position_offset = isize(upload.m_positions);
			auto const velocity_offset = isize(upload.m_velocities);
			auto const dof_offset = isize(upload.m_dofs);
			auto const root_type = articulation->RootType();

			// Floating roots occupy the first six generalized velocity entries but have no reduced position coordinates.
			switch (root_type)
			{
				case EArticulationRootType::Fixed:
				{
					break;
				}
				case EArticulationRootType::Floating:
				{
					AppendSpatial(upload.m_velocities, articulation->RootVelocity());
					AppendSpatial(upload.m_forces, articulation->RootForce());
					AppendSpatial(upload.m_accelerations, articulation->RootAcceleration());
					break;
				}
				default:
				{
					throw std::invalid_argument("Articulation root type cannot be represented on the GPU");
				}
			}

			// Link descriptors retain global range offsets so shader traversals never need pointer chasing or per-tree rebasing.
			auto tree_maximum_depth = 0;
			for (int link_index = 0; link_index != articulation->LinkCount(); ++link_index)
			{
				auto const link_handle = articulation->LinkAt(link_index);
				auto const& link_desc = articulation->LinkDescription(link_handle);
				auto parent_link_index = -1;
				auto position_index = -1;
				auto velocity_index = -1;
				auto packed_dof_offset = -1;
				auto joint_matrix_offset = -1;
				auto dof_count = 0;
				auto depth = 0;
				auto joint_to_parent = m4x4::Identity();
				auto joint_to_child = m4x4::Identity();

				if (link_index == 0)
				{
					if (root_type == EArticulationRootType::Floating)
						velocity_index = velocity_offset;
				}
				else
				{
					auto const parent = articulation->Parent(link_handle);
					parent_link_index = link_offset + static_cast<int>(parent.m_index);
					depth = upload.m_links[parent_link_index].depth + 1;
					auto const& joint = articulation->JointDescription(link_handle);
					dof_count = joint.m_dof_count;
					position_index = isize(upload.m_positions);
					velocity_index = isize(upload.m_velocities);
					packed_dof_offset = isize(upload.m_dofs);
					joint_to_parent = joint.m_joint_to_parent;
					joint_to_child = joint.m_joint_to_child;

					// Assign the active inverse its exact scalar range; zero-DOF joints consume an empty range.
					joint_matrix_offset = upload.m_joint_matrix_scratch_count;
					upload.m_joint_matrix_scratch_count += dof_count * dof_count;

					// Generalized arrays and ordered screw axes share the same per-joint scalar order.
					auto const position = articulation->JointPosition(link_handle);
					auto const velocity = articulation->JointVelocity(link_handle);
					auto const force = articulation->JointForce(link_handle);
					auto const acceleration = articulation->JointAcceleration(link_handle);
					upload.m_positions.insert(upload.m_positions.end(), position.begin(), position.end());
					upload.m_velocities.insert(upload.m_velocities.end(), velocity.begin(), velocity.end());
					upload.m_forces.insert(upload.m_forces.end(), force.begin(), force.end());
					upload.m_accelerations.insert(upload.m_accelerations.end(), acceleration.begin(), acceleration.end());
					for (int axis_index = 0; axis_index != dof_count; ++axis_index)
						upload.m_dofs.push_back(PackDof(joint.m_axes[axis_index]));
				}

				upload.m_links.push_back(GpuArticulationLink{
					.parent_link_index = parent_link_index,
					.articulation_index = articulation_index,
					.position_offset = position_index,
					.velocity_offset = velocity_index,
					.dof_offset = packed_dof_offset,
					.dof_count = dof_count,
					.child_offset = 0,
					.child_count = 0,
					.proxy_body_index = -1,
					.depth = depth,
					.joint_matrix_offset = joint_matrix_offset,
					.pad1 = 0,
					.joint_to_parent = PackGpuTransform(joint_to_parent),
					.joint_to_child = PackGpuTransform(joint_to_child),
					.shape_to_link = PackGpuTransform(link_desc.m_shape_to_link),
					.inertia_diagonal = link_desc.m_inertia.m_diagonal,
					.inertia_products = link_desc.m_inertia.m_products,
					.inertia_com_and_mass = link_desc.m_inertia.m_com_and_mass,
				});
				upload.m_external_forces.push_back(PackSpatialForce(articulation->ExternalForce(link_handle)));
				tree_maximum_depth = std::max(tree_maximum_depth, depth);
			}

			upload.m_articulations.push_back(GpuArticulation{
				.identity_low = static_cast<uint32_t>(identity),
				.identity_high = static_cast<uint32_t>(identity >> 32),
				.link_offset = link_offset,
				.link_count = articulation->LinkCount(),
				.position_offset = position_offset,
				.position_count = isize(upload.m_positions) - position_offset,
				.velocity_offset = velocity_offset,
				.velocity_count = isize(upload.m_velocities) - velocity_offset,
				.dof_offset = dof_offset,
				.dof_count = isize(upload.m_dofs) - dof_offset,
				.root_type = PackRootType(root_type),
				.max_depth = tree_maximum_depth,
				.root_to_world = PackGpuTransform(articulation->RootToWorld()),
			});
			maximum_depth = std::max(maximum_depth, tree_maximum_depth);
		}

		// Count direct children first so a prefix sum can allocate one compact adjacency range per link.
		for (auto const& link : upload.m_links)
		{
			if (link.parent_link_index >= 0)
				++upload.m_links[link.parent_link_index].child_count;
		}
		auto child_count = 0;
		for (auto& link : upload.m_links)
		{
			link.child_offset = child_count;
			child_count += link.child_count;
		}
		upload.m_children.resize(child_count);

		// Fill each child range without changing the deterministic topological order inherited from the builder.
		auto child_cursor = std::vector<int>(upload.m_links.size());
		for (int link_index = 0; link_index != isize(upload.m_links); ++link_index)
			child_cursor[link_index] = upload.m_links[link_index].child_offset;
		for (int link_index = 0; link_index != isize(upload.m_links); ++link_index)
		{
			auto const parent_link_index = upload.m_links[link_index].parent_link_index;
			if (parent_link_index >= 0)
				upload.m_children[child_cursor[parent_link_index]++] = static_cast<uint32_t>(link_index);
		}

		// Count and prefix breadth levels so a deep chain still builds its traversal schedule in linear work.
		auto level_counts = std::vector<int>(upload.m_links.empty() ? 0 : maximum_depth + 1, 0);
		for (auto const& link : upload.m_links)
			++level_counts[link.depth];
		for (int depth = 0, level_offset = 0; depth != isize(level_counts); ++depth)
		{
			upload.m_levels.push_back(GpuArticulationLevel{
				.depth = depth,
				.link_offset = level_offset,
				.link_count = level_counts[depth],
				.pad0 = 0,
			});
			level_offset += level_counts[depth];
		}
		upload.m_level_links.resize(upload.m_links.size());

		// Stable per-level cursors preserve caller and builder order among independent links at the same depth.
		auto level_cursors = std::vector<int>(upload.m_levels.size());
		for (int depth = 0; depth != isize(upload.m_levels); ++depth)
			level_cursors[depth] = upload.m_levels[depth].link_offset;
		for (int link_index = 0; link_index != isize(upload.m_links); ++link_index)
			upload.m_level_links[level_cursors[upload.m_links[link_index].depth]++] = static_cast<uint32_t>(link_index);

		return upload;
	}

	// Pack one hidden force/collision proxy per link and assign its contiguous body-buffer index.
	std::vector<GpuRigidBody> PackGpuArticulationProxies(GpuArticulationUpload& upload, std::span<Articulation* const> articulations, std::span<int const> shape_ids, int first_body_index)
	{
		ValidateGpuArticulationUpload(upload);
		if (first_body_index < 0)
			throw std::invalid_argument("GPU articulation proxy body offset must be non-negative");
		if (shape_ids.size() != upload.m_links.size())
			throw std::invalid_argument("GPU articulation proxy shape ids must match the packed link count");
		if (articulations.size() != upload.m_articulations.size())
			throw std::invalid_argument("GPU articulation proxy sources must match the packed articulation count");

		auto proxies = std::vector<GpuRigidBody>{};
		proxies.reserve(upload.m_links.size());

		// Preserve the forest's deterministic articulation/link order so every proxy index is a direct packed-link offset.
		for (int articulation_index = 0; articulation_index != isize(articulations); ++articulation_index)
		{
			auto const* articulation = articulations[articulation_index];
			if (articulation == nullptr)
				throw std::invalid_argument("GPU articulation proxy sources cannot contain null pointers");

			auto const& packed_articulation = upload.m_articulations[articulation_index];
			for (int local_link_index = 0; local_link_index != articulation->LinkCount(); ++local_link_index)
			{
				auto const packed_link_index = packed_articulation.link_offset + local_link_index;
				auto const link = articulation->LinkAt(local_link_index);
				auto const& desc = articulation->LinkDescription(link);
				auto const proxy_body_index = first_body_index + packed_link_index;
				auto const proxy_to_world = articulation->LinkToWorld(link) * desc.m_shape_to_link;
				auto const link_to_proxy = InvertOrthonormal(desc.m_shape_to_link);
				auto const proxy_velocity = link_to_proxy * articulation->LinkVelocity(link);
				auto const proxy_com = (link_to_proxy * desc.m_inertia.CoM().w1()).w0();
				auto proxy_inertia = Rotate(desc.m_inertia, link_to_proxy.rot);
				proxy_inertia.CoM(proxy_com);

				// Rigid force modules consume world momentum measured at the centre of mass.
				auto const angular_velocity_proxy = proxy_velocity.ang;
				auto const linear_velocity_com_proxy = proxy_velocity.lin + Cross(angular_velocity_proxy, proxy_com);
				auto const momentum_ang_ws = proxy_to_world.rot * (proxy_inertia.Ic3x3() * angular_velocity_proxy);
				auto const momentum_lin_ws = proxy_to_world.rot * (proxy_inertia.Mass() * linear_velocity_com_proxy);
				auto const fixed_root = local_link_index == 0 && articulation->RootType() == EArticulationRootType::Fixed;
				auto const inertia_inv = fixed_root ? InertiaInv::Zero() : Invert(proxy_inertia);
				auto const gravity_ws = articulation->GravityWS(link);
				auto const gravity_force_ws = fixed_root ? v4{} : proxy_inertia.Mass() * gravity_ws;
				auto const shape_id = shape_ids[packed_link_index];
				auto const os_bbox = desc.m_shape != nullptr ? desc.m_shape->m_s2r * desc.m_shape->m_bbox : BBox::Zero();

				upload.m_links[packed_link_index].proxy_body_index = proxy_body_index;
				proxies.push_back(GpuRigidBody{
					.o2w = proxy_to_world,
					.momentum_ang = momentum_ang_ws,
					.momentum_lin = momentum_lin_ws,
					.force_ang = {},
					.force_lin = gravity_force_ws,
					.ws_gravity = gravity_ws,
					.inertia_inv_diagonal = inertia_inv.m_diagonal,
					.inertia_inv_products = inertia_inv.m_products,
					.os_com_and_invmass = v4{proxy_com.xyz, inertia_inv.InvMass()},
					.os_bbox = os_bbox,
					.state_flags = fixed_root ? static_cast<int>(ERigidBodyStateFlags::Static) : static_cast<int>(ERigidBodyStateFlags::None),
					.shape_id = shape_id,
					.colour_used = 0,
					.pad0 = 0,
					.sleep = GpuSleepData{
						.timer_s = 0.0f,
						.island_id = -1,
						.generation = 0,
						.flags = 0,
					},
				});
			}
		}
		return proxies;
	}

	// Reject malformed packed ranges and topology before shared replay or GPU kernels can index them.
	void ValidateGpuArticulationUpload(GpuArticulationUpload const& upload)
	{
		if (upload.m_velocities.size() != upload.m_forces.size() ||
			upload.m_velocities.size() != upload.m_accelerations.size())
			throw std::invalid_argument("GPU articulation generalized velocity, force, and acceleration ranges must match");
		if (upload.m_links.size() != upload.m_external_forces.size() ||
			upload.m_links.size() != upload.m_level_links.size())
			throw std::invalid_argument("GPU articulation link, wrench, and schedule ranges must match");
		if (upload.m_joint_matrix_scratch_count < 0)
			throw std::invalid_argument("GPU articulation matrix scratch count is invalid");

		// Empty forests have no partial ABI image because no dispatch may legally consume one.
		if (upload.m_articulations.empty())
		{
			if (!upload.m_links.empty() ||
				!upload.m_dofs.empty() ||
				!upload.m_positions.empty() ||
				!upload.m_velocities.empty() ||
				!upload.m_forces.empty() ||
				!upload.m_accelerations.empty() ||
				!upload.m_external_forces.empty() ||
				!upload.m_children.empty() ||
				!upload.m_levels.empty() ||
				!upload.m_level_links.empty() ||
				upload.m_joint_matrix_scratch_count != 0)
				throw std::invalid_argument("Empty GPU articulation forests cannot contain packed feature ranges");
			return;
		}

		// Breadth levels are contiguous, depth ordered, and cover every link exactly once.
		auto scheduled = std::vector<int>(upload.m_links.size(), 0);
		auto level_cursor = 0;
		for (int level_index = 0; level_index != isize(upload.m_levels); ++level_index)
		{
			auto const& level = upload.m_levels[level_index];
			if (level.depth != level_index ||
				level.link_offset != level_cursor ||
				level.link_count <= 0 ||
				!PackedRangeValid(level.link_offset, level.link_count, upload.m_level_links.size()))
				throw std::invalid_argument("GPU articulation breadth-level schedule is invalid");

			for (int offset = 0; offset != level.link_count; ++offset)
			{
				auto const link_index = upload.m_level_links[level.link_offset + offset];
				if (link_index >= upload.m_links.size() ||
					upload.m_links[link_index].depth != level.depth ||
					++scheduled[link_index] != 1)
					throw std::invalid_argument("GPU articulation schedule contains an invalid or duplicate link");
			}
			level_cursor += level.link_count;
		}
		if (level_cursor != isize(upload.m_level_links) ||
			std::ranges::find(scheduled, 1) == scheduled.end() ||
			std::ranges::any_of(scheduled, [](int count) { return count != 1; }))
			throw std::invalid_argument("GPU articulation schedule does not cover the packed forest exactly once");

		// Tree headers partition every packed stream and establish each root's generalized-state convention.
		auto link_cursor = 0;
		auto position_cursor = 0;
		auto velocity_cursor = 0;
		auto dof_cursor = 0;
		for (int articulation_index = 0; articulation_index != isize(upload.m_articulations); ++articulation_index)
		{
			auto const& articulation = upload.m_articulations[articulation_index];
			if (articulation.link_offset != link_cursor ||
				articulation.position_offset != position_cursor ||
				articulation.velocity_offset != velocity_cursor ||
				articulation.dof_offset != dof_cursor ||
				articulation.link_count <= 0 ||
				!PackedRangeValid(articulation.link_offset, articulation.link_count, upload.m_links.size()) ||
				!PackedRangeValid(articulation.position_offset, articulation.position_count, upload.m_positions.size()) ||
				!PackedRangeValid(articulation.velocity_offset, articulation.velocity_count, upload.m_velocities.size()) ||
				!PackedRangeValid(articulation.dof_offset, articulation.dof_count, upload.m_dofs.size()) ||
				articulation.position_count != articulation.dof_count)
				throw std::invalid_argument("GPU articulation header partition is invalid");

			auto const root_velocity_count = PackedRootVelocityCount(articulation.root_type);
			if (articulation.velocity_count != articulation.dof_count + root_velocity_count)
				throw std::invalid_argument("GPU articulation root type or generalized range is invalid");

			auto const& root = upload.m_links[articulation.link_offset];
			if (root.parent_link_index != -1 ||
				root.articulation_index != articulation_index ||
				root.depth != 0 ||
				root.position_offset != -1 ||
				root.dof_offset != -1 ||
				root.dof_count != 0 ||
				root.joint_matrix_offset != -1 ||
				root.velocity_offset != (root_velocity_count != 0 ? articulation.velocity_offset : -1))
				throw std::invalid_argument("GPU articulation root descriptor is invalid");

			link_cursor += articulation.link_count;
			position_cursor += articulation.position_count;
			velocity_cursor += articulation.velocity_count;
			dof_cursor += articulation.dof_count;
		}
		if (link_cursor != isize(upload.m_links) ||
			position_cursor != isize(upload.m_positions) ||
			velocity_cursor != isize(upload.m_velocities) ||
			dof_cursor != isize(upload.m_dofs))
			throw std::invalid_argument("GPU articulation headers do not partition their packed streams");

		// Link ranges preserve topological order, exact compact matrix packing, and complete direct-child adjacency.
		auto adjacency_seen = std::vector<int>(upload.m_links.size(), 0);
		auto matrix_cursor = 0;
		auto child_cursor = 0;
		auto joint_position_cursor = 0;
		auto joint_velocity_cursor = 0;
		auto joint_dof_cursor = 0;
		for (int link_index = 0; link_index != isize(upload.m_links); ++link_index)
		{
			auto const& link = upload.m_links[link_index];
			if (link.articulation_index < 0 ||
				link.articulation_index >= isize(upload.m_articulations) ||
				link.child_offset != child_cursor ||
				!PackedRangeValid(link.child_offset, link.child_count, upload.m_children.size()))
				throw std::invalid_argument("GPU articulation link ownership or child range is invalid");
			child_cursor += link.child_count;

			auto const& articulation = upload.m_articulations[link.articulation_index];
			if (!PackedRangeValid(articulation.link_offset, articulation.link_count, upload.m_links.size()) ||
				link_index < articulation.link_offset ||
				link_index >= articulation.link_offset + articulation.link_count)
				throw std::invalid_argument("GPU articulation link lies outside its owning tree");

			if (link.parent_link_index >= 0)
			{
				if (link.dof_count < 0 || link.dof_count > 6)
					throw std::invalid_argument("GPU articulation joint dimension is invalid");

				auto const matrix_count = link.dof_count * link.dof_count;
				if (link.parent_link_index >= link_index ||
					upload.m_links[link.parent_link_index].articulation_index != link.articulation_index ||
					upload.m_links[link.parent_link_index].depth + 1 != link.depth ||
					link.position_offset != joint_position_cursor ||
					link.velocity_offset != joint_velocity_cursor ||
					link.dof_offset != joint_dof_cursor ||
					!PackedRangeValid(link.position_offset, link.dof_count, upload.m_positions.size()) ||
					!PackedRangeValid(link.velocity_offset, link.dof_count, upload.m_velocities.size()) ||
					!PackedRangeValid(link.dof_offset, link.dof_count, upload.m_dofs.size()) ||
					link.joint_matrix_offset != matrix_cursor ||
					!PackedRangeValid(link.joint_matrix_offset, matrix_count, static_cast<size_t>(upload.m_joint_matrix_scratch_count)))
					throw std::invalid_argument("GPU articulation non-root link range is invalid");

				if (link.position_offset < articulation.position_offset ||
					link.position_offset + link.dof_count > articulation.position_offset + articulation.position_count ||
					link.velocity_offset < articulation.velocity_offset ||
					link.velocity_offset + link.dof_count > articulation.velocity_offset + articulation.velocity_count ||
					link.dof_offset < articulation.dof_offset ||
					link.dof_offset + link.dof_count > articulation.dof_offset + articulation.dof_count)
					throw std::invalid_argument("GPU articulation link generalized range crosses its owning tree");
				joint_position_cursor += link.dof_count;
				joint_velocity_cursor += link.dof_count;
				joint_dof_cursor += link.dof_count;
				matrix_cursor += matrix_count;
			}
			else
			{
				joint_position_cursor = articulation.position_offset;
				joint_velocity_cursor = articulation.velocity_offset + PackedRootVelocityCount(articulation.root_type);
				joint_dof_cursor = articulation.dof_offset;
			}

			for (int offset = 0; offset != link.child_count; ++offset)
			{
				auto const child_index = upload.m_children[link.child_offset + offset];
				if (child_index >= upload.m_links.size() ||
					upload.m_links[child_index].parent_link_index != link_index ||
					++adjacency_seen[child_index] != 1)
					throw std::invalid_argument("GPU articulation child adjacency is invalid");
			}
		}
		if (child_cursor != isize(upload.m_children) ||
			joint_position_cursor != isize(upload.m_positions) ||
			joint_velocity_cursor != isize(upload.m_velocities) ||
			joint_dof_cursor != isize(upload.m_dofs) ||
			matrix_cursor != upload.m_joint_matrix_scratch_count)
			throw std::invalid_argument("GPU articulation child, generalized, or inverse ranges are not tightly packed");
		for (int link_index = 0; link_index != isize(upload.m_links); ++link_index)
		{
			auto const is_root = upload.m_links[link_index].parent_link_index < 0;
			if (adjacency_seen[link_index] != (is_root ? 0 : 1))
				throw std::invalid_argument("GPU articulation child adjacency does not cover every non-root link exactly once");
		}
	}
}
