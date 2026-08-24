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
					.pad0 = 0,
					.pad1 = 0,
					.joint_to_parent = PackGpuTransform(joint_to_parent),
					.joint_to_child = PackGpuTransform(joint_to_child),
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
}
