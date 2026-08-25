//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/physics/physics.h"
#include "src/articulation/articulation_gpu_data.h"

namespace pr::physics::tests
{
	namespace
	{
		// Return asymmetric finite link mass properties so every compact inertia field is observable.
		ArticulationLinkDesc GpuDataLink(int seed)
		{
			auto const scale = static_cast<float>(seed + 1);
			return ArticulationLinkDesc{
				.m_inertia = Inertia::Box(
					v4{0.17f + 0.03f * scale, 0.29f + 0.02f * scale, 0.41f + 0.01f * scale, 0},
					0.8f + 0.4f * scale,
					v4{0.013f * scale, -0.009f * scale, 0.007f * scale, 0}),
			};
		}

		// Return a deterministic ordered joint with explicit attachment frames and up to two scalar axes.
		ArticulationJointDesc GpuDataJoint(int dof_count, int seed)
		{
			auto const scale = static_cast<float>(seed + 1);
			auto joint = ArticulationJointDesc::Fixed(
				m4x4::Transform(v4::ZAxis(), +0.05f * scale, v4{+0.21f * scale, -0.08f, +0.04f, 1}),
				m4x4::Transform(v4::XAxis(), -0.03f * scale, v4{-0.11f, +0.06f * scale, -0.09f, 1}));
			joint.m_dof_count = dof_count;
			joint.m_axes[0] = ArticulationAxisDesc{.m_type = EArticulationAxisType::Revolute, .m_axis = Normalise(v4{1, 2, -1, 0})};
			joint.m_axes[1] = ArticulationAxisDesc{.m_type = EArticulationAxisType::Prismatic, .m_axis = Normalise(v4{-1, 1, 2, 0})};
			for (int axis_index = 0; axis_index != dof_count; ++axis_index)
			{
				joint.m_initial_position[axis_index] = 0.12f * scale * static_cast<float>(axis_index + 1);
				joint.m_initial_velocity[axis_index] = -0.17f * scale + 0.09f * static_cast<float>(axis_index);
			}
			return joint;
		}

		// One articulation plus stable handles retained for setting and checking packed state.
		struct PackedFixture
		{
			Articulation m_articulation;
			std::vector<LinkHandle> m_links;
		};

		// Build a floating branching tree containing two-DOF, fixed, and one-DOF joints.
		PackedFixture BuildPackedFixture()
		{
			auto builder = ArticulationBuilder{};
			auto links = std::vector<LinkHandle>{};
			links.push_back(builder.AddFloatingRoot(
				GpuDataLink(0),
				m4x4::Transform(v4::YAxis(), 0.37f, v4{1.2f, -0.7f, 2.3f, 1}),
				v8motion{v4{0.31f, -0.27f, 0.19f, 0}, v4{-0.23f, 0.17f, 0.29f, 0}}));
			links.push_back(builder.AddLink(links[0], GpuDataJoint(2, 1), GpuDataLink(1)));
			links.push_back(builder.AddLink(links[1], GpuDataJoint(0, 2), GpuDataLink(2)));
			links.push_back(builder.AddLink(links[0], GpuDataJoint(1, 3), GpuDataLink(3)));
			return PackedFixture{
				.m_articulation = builder.Build(),
				.m_links = std::move(links),
			};
		}

		// Build a fixed chain with one scalar joint per non-root link.
		Articulation BuildChain(int link_count)
		{
			auto builder = ArticulationBuilder{};
			auto parent = builder.AddFixedRoot({});
			for (int link_index = 1; link_index != link_count; ++link_index)
				parent = builder.AddLink(parent, GpuDataJoint(1, link_index), GpuDataLink(link_index));

			return builder.Build();
		}

		// Require a packed float range to equal a public generalized range exactly.
		void ExpectRange(std::span<float const> packed, int offset, std::span<float const> expected)
		{
			PR_EXPECT(offset >= 0);
			PR_EXPECT(offset + isize(expected) <= isize(packed));
			for (int index = 0; index != isize(expected); ++index)
				PR_EXPECT(packed[offset + index] == expected[index]);
		}
	}

	PRUnitTestClass(ArticulationGpuDataTests)
	{
		// Preserve topology, generalized state, link wrenches, frames, axes, and compact inertias in the shared GPU ABI.
		PRUnitTestMethod(PackPreservesTreeState, Quick)
		{
			auto fixture = BuildPackedFixture();
			fixture.m_articulation.RootForce(v8force{v4{0.7f, -0.3f, 0.2f, 0}, v4{-0.4f, 0.6f, 0.1f, 0}});
			fixture.m_articulation.JointForce(fixture.m_links[1], std::array{+0.23f, -0.41f});
			fixture.m_articulation.JointForce(fixture.m_links[3], std::array{+0.37f});
			fixture.m_articulation.ExternalForce(fixture.m_links[0], v8force{v4{0.1f, 0.2f, 0.3f, 0}, v4{0.4f, 0.5f, 0.6f, 0}});
			fixture.m_articulation.ExternalForce(fixture.m_links[2], v8force{v4{-0.6f, 0.5f, -0.4f, 0}, v4{0.3f, -0.2f, 0.1f, 0}});
			fixture.m_articulation.ForwardDynamics();

			auto sources = std::array{&fixture.m_articulation};
			auto const upload = PackGpuArticulations(sources);
			auto const& header = upload.m_articulations[0];

			PR_EXPECT(upload.m_articulations.size() == 1);
			PR_EXPECT(upload.m_links.size() == 4);
			PR_EXPECT(upload.m_dofs.size() == 3);
			PR_EXPECT(upload.m_positions.size() == 3);
			PR_EXPECT(upload.m_velocities.size() == 9);
			PR_EXPECT(upload.m_forces.size() == 9);
			PR_EXPECT(upload.m_accelerations.size() == 9);
			PR_EXPECT(upload.m_external_forces.size() == 4);
			PR_EXPECT(header.link_offset == 0);
			PR_EXPECT(header.link_count == 4);
			PR_EXPECT(header.position_offset == 0);
			PR_EXPECT(header.position_count == 3);
			PR_EXPECT(header.velocity_offset == 0);
			PR_EXPECT(header.velocity_count == 9);
			PR_EXPECT(header.dof_offset == 0);
			PR_EXPECT(header.dof_count == 3);
			PR_EXPECT(header.root_type == GpuArticulationRootType_Floating);
			PR_EXPECT(header.max_depth == 2);
			PR_EXPECT(FEql(UnpackGpuTransform(header.root_to_world), fixture.m_articulation.RootToWorld()));

			// The root occupies six velocity entries while reduced joint arrays remain tightly concatenated after it.
			auto const root_velocity = fixture.m_articulation.RootVelocity();
			auto const root_force = fixture.m_articulation.RootForce();
			PR_EXPECT(upload.m_velocities[0] == root_velocity.ang.x);
			PR_EXPECT(upload.m_velocities[5] == root_velocity.lin.z);
			PR_EXPECT(upload.m_forces[0] == root_force.ang.x);
			PR_EXPECT(upload.m_forces[5] == root_force.lin.z);
			ExpectRange(upload.m_positions, upload.m_links[1].position_offset, fixture.m_articulation.JointPosition(fixture.m_links[1]));
			ExpectRange(upload.m_velocities, upload.m_links[1].velocity_offset, fixture.m_articulation.JointVelocity(fixture.m_links[1]));
			ExpectRange(upload.m_forces, upload.m_links[1].velocity_offset, fixture.m_articulation.JointForce(fixture.m_links[1]));
			ExpectRange(upload.m_accelerations, upload.m_links[3].velocity_offset, fixture.m_articulation.JointAcceleration(fixture.m_links[3]));

			// Parent indices are global packed indices and fixed joints retain valid empty scalar ranges.
			PR_EXPECT(upload.m_links[0].parent_link_index == -1);
			PR_EXPECT(upload.m_links[1].parent_link_index == 0);
			PR_EXPECT(upload.m_links[2].parent_link_index == 1);
			PR_EXPECT(upload.m_links[3].parent_link_index == 0);
			PR_EXPECT(upload.m_links[0].velocity_offset == 0);
			PR_EXPECT(upload.m_links[1].velocity_offset == 6);
			PR_EXPECT(upload.m_links[2].dof_count == 0);
			PR_EXPECT(upload.m_links[3].velocity_offset == 8);
			PR_EXPECT(FEql(UnpackGpuTransform(upload.m_links[1].joint_to_parent), fixture.m_articulation.JointDescription(fixture.m_links[1]).m_joint_to_parent));
			PR_EXPECT(FEql(UnpackGpuTransform(upload.m_links[1].joint_to_child), fixture.m_articulation.JointDescription(fixture.m_links[1]).m_joint_to_child));

			// Axis types use exact float encodings and compact inertia fields remain byte-for-byte compatible with the CPU descriptors.
			PR_EXPECT(upload.m_dofs[0].axis_and_type.w == static_cast<float>(GpuArticulationAxisType_Revolute));
			PR_EXPECT(upload.m_dofs[1].axis_and_type.w == static_cast<float>(GpuArticulationAxisType_Prismatic));
			PR_EXPECT(FEql(upload.m_dofs[0].axis_and_type.xyz, fixture.m_articulation.JointDescription(fixture.m_links[1]).m_axes[0].m_axis.xyz));
			PR_EXPECT(FEql(upload.m_links[3].inertia_diagonal, fixture.m_articulation.LinkDescription(fixture.m_links[3]).m_inertia.m_diagonal));
			PR_EXPECT(FEql(upload.m_links[3].inertia_products, fixture.m_articulation.LinkDescription(fixture.m_links[3]).m_inertia.m_products));
			PR_EXPECT(FEql(upload.m_links[3].inertia_com_and_mass, fixture.m_articulation.LinkDescription(fixture.m_links[3]).m_inertia.m_com_and_mass));
			PR_EXPECT(FEql(upload.m_external_forces[2].force_ang, fixture.m_articulation.ExternalForce(fixture.m_links[2]).ang));
			PR_EXPECT(FEql(upload.m_external_forces[2].force_lin, fixture.m_articulation.ExternalForce(fixture.m_links[2]).lin));
		}

		// Cover every forest link exactly once in deterministic breadth levels and compact direct-child ranges.
		PRUnitTestMethod(TraversalScheduleCoversForest, Quick)
		{
			auto chain = BuildChain(4);
			auto fixture = BuildPackedFixture();
			auto sources = std::array{&chain, &fixture.m_articulation};
			auto const upload = PackGpuArticulations(sources);

			PR_EXPECT(upload.m_levels.size() == 4);
			PR_EXPECT(upload.m_levels[0].link_count == 2);
			PR_EXPECT(upload.m_levels[1].link_count == 3);
			PR_EXPECT(upload.m_levels[2].link_count == 2);
			PR_EXPECT(upload.m_levels[3].link_count == 1);
			PR_EXPECT(upload.m_level_links.size() == upload.m_links.size());
			PR_EXPECT(upload.m_children.size() + upload.m_articulations.size() == upload.m_links.size());

			// Every scheduled index is unique, belongs to its advertised depth, and follows its parent.
			auto seen = std::vector<int>(upload.m_links.size(), 0);
			for (auto const& level : upload.m_levels)
			{
				for (int offset = 0; offset != level.link_count; ++offset)
				{
					auto const link_index = static_cast<int>(upload.m_level_links[level.link_offset + offset]);
					PR_EXPECT(link_index >= 0 && link_index < isize(upload.m_links));
					PR_EXPECT(upload.m_links[link_index].depth == level.depth);
					PR_EXPECT(++seen[link_index] == 1);
					auto const parent = upload.m_links[link_index].parent_link_index;
					if (parent >= 0)
						PR_EXPECT(upload.m_links[parent].depth + 1 == level.depth);
				}
			}
			for (auto count : seen)
				PR_EXPECT(count == 1);

			// Child adjacency is complete and every entry points back to the link that owns its range.
			for (int parent_index = 0; parent_index != isize(upload.m_links); ++parent_index)
			{
				auto const& parent = upload.m_links[parent_index];
				for (int offset = 0; offset != parent.child_count; ++offset)
				{
					auto const child_index = static_cast<int>(upload.m_children[parent.child_offset + offset]);
					PR_EXPECT(upload.m_links[child_index].parent_link_index == parent_index);
				}
			}
		}

		// Pack shaped and shape-less links into one contiguous hidden-body suffix while preserving frame, force, and ownership conventions.
		PRUnitTestMethod(PackArticulationLinkProxies, Quick)
		{
			auto shape = collision::ShapeSphere{0.35f};
			auto root_desc = GpuDataLink(0);
			root_desc.m_shape = collision::shape_cast(&shape);
			root_desc.m_shape_to_link = m4x4::Transform(v4::YAxis(), 0.21f, v4{0.04f, -0.03f, 0.07f, 1});
			auto child_desc = GpuDataLink(1);
			child_desc.m_shape_to_link = m4x4::Transform(v4::XAxis(), -0.17f, v4{-0.06f, 0.02f, 0.05f, 1});

			auto builder = ArticulationBuilder{};
			auto const root = builder.AddFixedRoot(root_desc, m4x4::Transform(v4::ZAxis(), 0.32f, v4{1.1f, -0.8f, 0.6f, 1}));
			auto const child = builder.AddLink(root, GpuDataJoint(1, 2), child_desc);
			auto articulation = builder.Build();
			auto const gravity_ws = v4{0.4f, -0.7f, -9.2f, 0};
			articulation.GravityWS(root, v4{0, 0, -9.8f, 0});
			articulation.GravityWS(child, gravity_ws);

			auto sources = std::array{&articulation};
			auto upload = PackGpuArticulations(sources);
			auto const shape_ids = std::array{17, -1};
			auto const proxies = PackGpuArticulationProxies(upload, sources, shape_ids, 5);

			PR_EXPECT(proxies.size() == 2);
			PR_EXPECT(upload.m_links[0].proxy_body_index == 5);
			PR_EXPECT(upload.m_links[1].proxy_body_index == 6);
			PR_EXPECT(proxies[0].shape_id == 17);
			PR_EXPECT(proxies[1].shape_id == -1);
			PR_EXPECT(AllSet(static_cast<ERigidBodyStateFlags>(proxies[0].state_flags), ERigidBodyStateFlags::Static));
			PR_EXPECT(!AllSet(static_cast<ERigidBodyStateFlags>(proxies[1].state_flags), ERigidBodyStateFlags::Static));
			PR_EXPECT(FEql(proxies[0].o2w, articulation.LinkToWorld(root) * root_desc.m_shape_to_link));
			PR_EXPECT(FEql(proxies[1].o2w, articulation.LinkToWorld(child) * child_desc.m_shape_to_link));
			PR_EXPECT(FEql(proxies[0].force_lin, v4{}));
			PR_EXPECT(FEql(proxies[1].force_lin, child_desc.m_inertia.Mass() * gravity_ws));
			PR_EXPECT(FEql(proxies[1].ws_gravity, gravity_ws));
			PR_EXPECT(FEql(proxies[1].os_bbox.m_centre, v4{0, 0, 0, 1}));
			PR_EXPECT(FEql(proxies[1].os_bbox.m_radius, v4{}));
			PR_EXPECT(FEql(UnpackGpuTransform(upload.m_links[1].shape_to_link), child_desc.m_shape_to_link));

			auto const wrong_shape_ids = std::array{17};
			PR_THROWS(PackGpuArticulationProxies(upload, sources, wrong_shape_ids, 5), std::exception);
			PR_THROWS(PackGpuArticulationProxies(upload, sources, shape_ids, -1), std::exception);
		}

		// Reject pointer and identity aliasing before any GPU resource can observe an ambiguous tree.
		PRUnitTestMethod(RejectsInvalidSubmission, Quick)
		{
			auto fixture = BuildPackedFixture();
			auto null_sources = std::array<Articulation*, 1>{nullptr};
			auto duplicate_sources = std::array{&fixture.m_articulation, &fixture.m_articulation};
			PR_THROWS(PackGpuArticulations(null_sources), std::exception);
			PR_THROWS(PackGpuArticulations(duplicate_sources), std::exception);

			auto const empty = PackGpuArticulations({});
			ValidateGpuArticulationUpload(empty);
			PR_EXPECT(empty.m_articulations.empty());
			PR_EXPECT(empty.m_links.empty());
			PR_EXPECT(empty.m_levels.empty());

			// Shared replay and hardware validation reject schedule, adjacency, and compact-matrix corruption before indexing.
			auto sources = std::array{&fixture.m_articulation};
			auto valid = PackGpuArticulations(sources);
			ValidateGpuArticulationUpload(valid);
			auto malformed_schedule = valid;
			malformed_schedule.m_level_links[0] = static_cast<uint32_t>(malformed_schedule.m_links.size());
			PR_THROWS(ValidateGpuArticulationUpload(malformed_schedule), std::exception);
			auto malformed_adjacency = valid;
			malformed_adjacency.m_children[0] = 0;
			PR_THROWS(ValidateGpuArticulationUpload(malformed_adjacency), std::exception);
			auto malformed_dimension = valid;
			malformed_dimension.m_links[1].dof_count = std::numeric_limits<int>::max();
			PR_THROWS(ValidateGpuArticulationUpload(malformed_dimension), std::exception);
			auto malformed_matrix = valid;
			++malformed_matrix.m_links[1].joint_matrix_offset;
			PR_THROWS(ValidateGpuArticulationUpload(malformed_matrix), std::exception);
		}
	};
}
#endif
