//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/physics/physics.h"
#include "src/constraint/constraint_gpu.h"

namespace pr::physics::tests
{
	namespace
	{
		// Build one translated child link for exercising the coupled constraint packing lane.
		std::pair<Articulation, LinkHandle> MakeConstraintGpuTree()
		{
			auto const link = ArticulationLinkDesc{
				.m_inertia = Inertia::Sphere(0.25f, 1.0f),
			};
			auto builder = ArticulationBuilder{};
			auto const root = builder.AddFloatingRoot(link);
			auto const child = builder.AddLink(root, ArticulationJointDesc::Revolute(v4::ZAxis(), m4x4::Translation(1.0f, 0.0f, 0.0f)), link);
			return {builder.Build(), child};
		}
	}

	// Focused checks for the persistent frame representation and its validation boundary.
	PRUnitTestClass(CanonicalConstraintFrameTests)
	{
		// Matrix-authored frames must retain the previous GPU bytes without a conversion during packing.
		PRUnitTestMethod(MatrixConstructionPreservesGpuFrames, Quick)
		{
			// Exercise all rotation-conversion branches, translation, and signed-zero position components.
			auto const matrices = std::array{
				m4x4::Identity(),
				m4x4::Translation(-0.0f, 2.0f, -3.0f),
				m4x4::Transform(v4::XAxis(), 2.8f, v4{1, 2, 3, 1}),
				m4x4::Transform(v4::YAxis(), 2.8f, v4{2, 3, 4, 1}),
				m4x4::Transform(v4::ZAxis(), 2.8f, v4{3, 4, 5, 1}),
			};
			auto body = RigidBody{};
			auto body_ptrs = std::array<RigidBody*, 1>{&body};
			for (auto const& matrix : matrices)
			{
				// Compare actual packed bytes with the original matrix packing operation for both endpoints.
				auto desc = D6ConstraintDesc{};
				desc.m_frame_a = BodyFrame::FromMatrix(BodyRef::Rigid(body), matrix);
				desc.m_frame_b = BodyFrame::FromMatrix(BodyRef::World(), matrix);
				desc.m_linear[0].m_mode = EConstraintAxisMode::Locked;
				auto constraints = ConstraintSet{};
				auto const handle = constraints.Add(desc);
				auto upload = PackGpuConstraints(constraints, BodyRemap(body_ptrs));
				auto const expected = PackGpuTransform(matrix);
				PR_EXPECT(memcmp(&upload.m_descriptors[handle.m_index].frame_a, &expected, sizeof(expected)) == 0);
				PR_EXPECT(memcmp(&upload.m_descriptors[handle.m_index].frame_b, &expected, sizeof(expected)) == 0);
				PR_EXPECT(FEqlAbsolute(m4x4::Transform(desc.m_frame_a.m_rotation, desc.m_frame_a.m_position), matrix, 1.0e-6f));

				// A valid directly authored quaternion is copied exactly, not converted through an intermediate matrix.
				desc.m_frame_a.m_rotation.xyzw = -desc.m_frame_a.m_rotation.xyzw;
				constraints.Update(handle, desc);
				upload = PackGpuConstraints(constraints, BodyRemap(body_ptrs));
				PR_EXPECT(memcmp(&upload.m_descriptors[handle.m_index].frame_a.rotation, &desc.m_frame_a.m_rotation, sizeof(quat)) == 0);
			}
		}

		// Invalid matrix inputs must fail before entering the quaternion conversion routine.
		PRUnitTestMethod(MatrixConstructionRejectsNonRigidInput, Quick)
		{
			// Cover finite non-rigid matrices as well as nonfinite rotation and translation.
			auto matrices = std::array<m4x4, 6>{};
			matrices.fill(m4x4::Identity());
			matrices[0].x.x = 2.0f;
			matrices[1].x.y = 0.1f;
			matrices[2].x.x = -1.0f;
			matrices[3].x.w = 0.1f;
			matrices[4].x.x = std::numeric_limits<float>::quiet_NaN();
			matrices[5].pos.x = std::numeric_limits<float>::infinity();
			for (auto const& matrix : matrices)
			{
				PR_THROWS(BodyFrame::FromMatrix(BodyRef::World(), matrix), std::invalid_argument);
			}
		}

		// Reused frames must not bypass changed-input validation or the batch transaction contract.
		PRUnitTestMethod(UpdatesValidateChangedFramesAtomically, Quick)
		{
			// Start with validated descriptors so target-only updates exercise the exact-frame reuse path.
			auto body = RigidBody{};
			auto desc = D6ConstraintDesc{};
			desc.m_frame_a.m_body = BodyRef::Rigid(body);
			desc.m_linear[0].m_mode = EConstraintAxisMode::Locked;
			auto constraints = ConstraintSet{};
			auto const handles = std::array{constraints.Add(desc), constraints.Add(desc)};
			desc.m_linear[0].m_target_position = 0.25f;
			constraints.Update(handles[0], desc);
			PR_EXPECT(constraints.Get(handles[0]).m_linear[0].m_target_position == 0.25f);
			constraints.ClearDirty();
			auto const topology = constraints.TopologyRevision();
			auto const parameters = constraints.ParameterRevision();

			// Even a one-ULP invalid homogeneous coordinate must be checked, not treated as an approximately unchanged frame.
			auto invalid = std::array<D6ConstraintDesc, 6>{};
			invalid.fill(desc);
			invalid[0].m_frame_a.m_rotation.w = 2.0f;
			invalid[1].m_frame_a.m_position.x = std::numeric_limits<float>::quiet_NaN();
			invalid[2].m_frame_b.m_position.w = std::nextafter(1.0f, 2.0f);
			invalid[3].m_frame_a.m_body.m_type = static_cast<EConstraintBodyType>(100);
			invalid[4].m_frame_a.m_body.m_body_id = {};
			invalid[5].m_angular[1].m_damping = -1.0f;
			auto changed = desc;
			changed.m_frame_a = BodyFrame::FromMatrix(desc.m_frame_a.m_body, m4x4::Transform(v4::YAxis(), 0.3f, v4{1, 2, 3, 1}));
			changed.m_linear[0].m_target_position = 0.5f;
			for (auto const& bad : invalid)
			{
				// Neither a new insertion nor a failed second batch entry may publish invalid state.
				PR_THROWS(constraints.Add(bad), std::invalid_argument);
				auto const batch = std::array{changed, bad};
				PR_THROWS(constraints.Update(handles, batch), std::invalid_argument);
				PR_EXPECT(constraints.Count() == 2);
				PR_EXPECT(constraints.TopologyRevision() == topology);
				PR_EXPECT(constraints.ParameterRevision() == parameters);
				PR_EXPECT(constraints.DirtyRanges().empty());
				PR_EXPECT(constraints.Get(handles[0]).m_linear[0].m_target_position == 0.25f);
				PR_EXPECT(All(constraints.Get(handles[0]).m_frame_a.m_position == v4::Origin()));
			}

			// Duplicate handles still validate every entry before committing the last descriptor.
			auto const duplicate = std::array{handles[0], handles[0]};
			auto const rejected = std::array{invalid[0], changed};
			PR_THROWS(constraints.Update(duplicate, rejected), std::invalid_argument);
			PR_EXPECT(constraints.ParameterRevision() == parameters);
			PR_EXPECT(constraints.DirtyRanges().empty());
			auto const accepted = std::array{desc, changed};
			constraints.Update(duplicate, accepted);
			auto const& stored = constraints.Get(handles[0]);
			PR_EXPECT(constraints.Contains(handles[0]));
			PR_EXPECT(constraints.TopologyRevision() == topology);
			PR_EXPECT(constraints.ParameterRevision() == parameters + 1);
			PR_EXPECT(stored.m_linear[0].m_target_position == 0.5f);
			PR_EXPECT(All(stored.m_frame_a.m_rotation.xyzw == changed.m_frame_a.m_rotation.xyzw));
			PR_EXPECT(All(stored.m_frame_a.m_position == changed.m_frame_a.m_position));
			auto const dirty = constraints.DirtyRanges();
			PR_EXPECT(dirty.size() == 1 && dirty[0].m_begin == handles[0].m_index && dirty[0].m_end == handles[0].m_index + 1);
		}
	};

	PRUnitTestClass(ConstraintGpuPackingTests)
	{
		// Preserve stable slots, canonical axis order, endpoint remapping, and compact transfer-layout contracts.
		PRUnitTestMethod(PacksSparsePersistentDescriptors, Quick)
		{
			PR_EXPECT(sizeof(GpuD6ConstraintDesc) == 256);
			PR_EXPECT(sizeof(GpuConstraintEndpoint) == 32);
			PR_EXPECT(sizeof(GpuCoupledConstraintBlockTopology) == 16);
			PR_EXPECT(sizeof(GpuCoupledConstraintTarget) == 32);
			PR_EXPECT(sizeof(GpuCoupledConstraintIsland) == 16);
			PR_EXPECT(sizeof(GpuConstraintBlock) == 32);
			PR_EXPECT(sizeof(GpuConstraintRow) == 96);

			auto body_a = RigidBody{};
			auto body_b = RigidBody{};
			auto desc = D6ConstraintDesc{};
			desc.m_frame_a.m_body = BodyRef::Rigid(body_a);
			desc.m_frame_b.m_body = BodyRef::Rigid(body_b);
			desc.m_frame_a = BodyFrame::FromMatrix(desc.m_frame_a.m_body, m4x4::Transform(v4::ZAxis(), 0.25f, v4{1, 2, 3, 1}));
			desc.m_linear[0].m_mode = EConstraintAxisMode::Locked;
			desc.m_linear[0].m_target_position = 0.5f;
			desc.m_angular[2].m_mode = EConstraintAxisMode::Driven;
			desc.m_angular[2].m_target_velocity = 2.0f;
			desc.m_angular[2].m_max_force = 7.0f;
			desc.m_collide_connected = true;

			auto constraints = ConstraintSet{};
			auto const removed = constraints.Add(desc);
			auto const live = constraints.Add(desc);
			constraints.Remove(removed);
			auto body_ptrs = std::array<RigidBody*, 2>{&body_b, &body_a};
			auto const upload = PackGpuConstraints(constraints, BodyRemap(body_ptrs));

			PR_EXPECT(upload.m_rigid_active_count == 1);
			PR_EXPECT(upload.m_coupled_active_count == 0);
			PR_EXPECT(upload.m_breakable_count == 0);
			PR_EXPECT(upload.m_coupled_endpoints.empty());
			PR_EXPECT(upload.m_coupled_articulation_indices.empty());
			PR_EXPECT(upload.m_endpoints.size() == 2);
			PR_EXPECT(upload.m_descriptors.size() == 2);
			PR_EXPECT(upload.m_endpoints[removed.m_index].flags == GpuConstraintEndpointFlags_None);
			PR_EXPECT(upload.m_endpoints[removed.m_index].generation != removed.m_generation);
			PR_EXPECT(upload.m_endpoints[live.m_index].body_idx_a == 1);
			PR_EXPECT(upload.m_endpoints[live.m_index].body_idx_b == 0);
			PR_EXPECT(AllSet(upload.m_endpoints[live.m_index].flags, GpuConstraintEndpointFlags_Enabled));
			PR_EXPECT(AllSet(upload.m_endpoints[live.m_index].flags, GpuConstraintEndpointFlags_CollideConnected));
			PR_EXPECT(upload.m_descriptors[live.m_index].axes[0].mode == GpuConstraintAxisMode_Locked);
			PR_EXPECT(upload.m_descriptors[live.m_index].axes[0].target_position == 0.5f);
			PR_EXPECT(upload.m_descriptors[live.m_index].axes[5].mode == GpuConstraintAxisMode_Driven);
			PR_EXPECT(upload.m_descriptors[live.m_index].axes[5].target_velocity == 2.0f);
			PR_EXPECT(upload.m_descriptors[live.m_index].axes[5].max_force == 7.0f);
		}

		// Request overload storage only while at least one active constraint has a finite force or torque threshold.
		PRUnitTestMethod(PacksBreakableCountWithoutChargingOrdinaryConstraints, Quick)
		{
			auto body = RigidBody{};
			auto ordinary = D6ConstraintDesc{};
			ordinary.m_frame_a.m_body = BodyRef::Rigid(body);
			ordinary.m_frame_b.m_body = BodyRef::World();
			ordinary.m_linear[0].m_mode = EConstraintAxisMode::Locked;
			auto breakable = ordinary;
			breakable.m_break_torque = 12.0f;
			auto disabled = breakable;
			disabled.m_enabled = false;
			auto constraints = ConstraintSet{};
			constraints.Add(ordinary);
			auto const breakable_handle = constraints.Add(breakable);
			constraints.Add(disabled);
			auto body_ptrs = std::array<RigidBody*, 1>{&body};

			auto upload = PackGpuConstraints(constraints, BodyRemap(body_ptrs));
			PR_EXPECT(upload.m_breakable_count == 1);

			constraints.SetEnabled(breakable_handle, false);
			upload = PackGpuConstraints(constraints, BodyRemap(body_ptrs));
			PR_EXPECT(upload.m_breakable_count == 0);
		}

		// Disabled descriptors retain their stable slot without requiring missing endpoint bodies in the submitted frame.
		PRUnitTestMethod(DisabledDescriptorsDoNotResolveEndpoints, Quick)
		{
			auto missing_body = RigidBody{};
			auto submitted_body = RigidBody{};
			auto desc = D6ConstraintDesc{};
			desc.m_frame_a.m_body = BodyRef::Rigid(missing_body);
			desc.m_frame_b.m_body = BodyRef::Rigid(submitted_body);
			desc.m_linear[0].m_mode = EConstraintAxisMode::Locked;
			desc.m_enabled = false;

			auto constraints = ConstraintSet{};
			auto const handle = constraints.Add(desc);
			auto body_ptrs = std::array<RigidBody*, 1>{&submitted_body};
			auto const upload = PackGpuConstraints(constraints, BodyRemap(body_ptrs));

			PR_EXPECT(upload.m_rigid_active_count == 0);
			PR_EXPECT(upload.m_coupled_active_count == 0);
			PR_EXPECT(upload.m_endpoints[handle.m_index].body_idx_a == -1);
			PR_EXPECT(upload.m_endpoints[handle.m_index].body_idx_b == -1);
			PR_EXPECT(!AllSet(upload.m_endpoints[handle.m_index].flags, GpuConstraintEndpointFlags_Enabled));
		}

		// Enabled all-free descriptors preserve their slot but do not activate or allocate the solver lane by themselves.
		PRUnitTestMethod(AllFreeDescriptorsHaveNoSolverWork, Quick)
		{
			auto body_a = RigidBody{};
			auto body_b = RigidBody{};
			auto desc = D6ConstraintDesc{};
			desc.m_frame_a.m_body = BodyRef::Rigid(body_a);
			desc.m_frame_b.m_body = BodyRef::Rigid(body_b);

			auto constraints = ConstraintSet{};
			auto const handle = constraints.Add(desc);
			auto body_ptrs = std::array<RigidBody*, 2>{&body_a, &body_b};
			auto const upload = PackGpuConstraints(constraints, BodyRemap(body_ptrs));

			PR_EXPECT(upload.m_rigid_active_count == 0);
			PR_EXPECT(upload.m_coupled_active_count == 0);
			PR_EXPECT(AllSet(upload.m_endpoints[handle.m_index].flags, GpuConstraintEndpointFlags_Enabled));
		}

		// Classify link constraints separately so hidden proxies can never enter the independent rigid-body solver lane.
		PRUnitTestMethod(ClassifiesArticulationLinkConstraints, Quick)
		{
			auto [articulation, child] = MakeConstraintGpuTree();
			auto desc = D6ConstraintDesc{};
			desc.m_frame_a.m_body = BodyRef::Link(articulation, child);
			desc.m_frame_b.m_body = BodyRef::World();
			desc.m_linear[0].m_mode = EConstraintAxisMode::Locked;
			auto constraints = ConstraintSet{};
			auto const handle = constraints.Add(desc);
			auto articulation_ptrs = std::array<Articulation*, 1>{&articulation};

			PR_EXPECT(HasCoupledConstraintWork(constraints));
			auto const upload = PackGpuConstraints(constraints, BodyRemap({}, articulation_ptrs));
			PR_EXPECT(upload.m_rigid_active_count == 0);
			PR_EXPECT(upload.m_coupled_active_count == 1);
			PR_EXPECT(upload.m_coupled_endpoints.size() == upload.m_endpoints.size());
			PR_EXPECT(upload.m_coupled_endpoints[handle.m_index].articulation_idx_a == 0);
			PR_EXPECT(upload.m_coupled_endpoints[handle.m_index].link_idx_a == 1);
			PR_EXPECT(upload.m_coupled_endpoints[handle.m_index].mobility_idx_a == 1);
			PR_EXPECT(upload.m_coupled_endpoints[handle.m_index].root_link_idx_a == 0);
			PR_EXPECT(upload.m_coupled_endpoints[handle.m_index].articulation_idx_b == -1);
			PR_EXPECT(upload.m_coupled_endpoints[handle.m_index].link_idx_b == -1);
			PR_EXPECT(upload.m_coupled_endpoints[handle.m_index].mobility_idx_b == -1);
			PR_EXPECT(upload.m_coupled_endpoints[handle.m_index].root_link_idx_b == -1);
			PR_EXPECT(upload.m_coupled_articulation_indices == std::vector<int>{0});
			PR_EXPECT(upload.m_endpoints[handle.m_index].body_idx_a == 1);
			PR_EXPECT(upload.m_endpoints[handle.m_index].body_idx_b == -1);
			PR_EXPECT(AllSet(upload.m_endpoints[handle.m_index].flags, GpuConstraintEndpointFlags_Enabled));
			PR_EXPECT(AllSet(upload.m_endpoints[handle.m_index].flags, GpuConstraintEndpointFlags_Coupled));

			constraints.SetEnabled(handle, false);
			PR_EXPECT(!HasCoupledConstraintWork(constraints));
		}

		// Pack stable target adjacency while keeping articulations attached through the same fixed body in independent islands.
		PRUnitTestMethod(PacksDeterministicCoupledTopology, Quick)
		{
			auto [articulation_a, child_a] = MakeConstraintGpuTree();
			auto [articulation_b, child_b] = MakeConstraintGpuTree();
			auto shape = collision::ShapeSphere{0.25f};
			auto dynamic_body = RigidBody{&shape, m4x4::Identity(), Inertia::Sphere(shape.m_radius, 1.0f)};
			auto fixed_body = RigidBody{&shape, m4x4::Identity(), Inertia::Infinite()};

			auto constraint = [](BodyRef body_a, BodyRef body_b)
			{
				auto desc = D6ConstraintDesc{};
				desc.m_frame_a.m_body = body_a;
				desc.m_frame_b.m_body = body_b;
				desc.m_linear[0].m_mode = EConstraintAxisMode::Locked;
				return desc;
			};
			auto constraints = ConstraintSet{};
			auto const articulation_dynamic = constraints.Add(constraint(BodyRef::Link(articulation_a, child_a), BodyRef::Rigid(dynamic_body)));
			auto const articulation_fixed = constraints.Add(constraint(BodyRef::Link(articulation_b, child_b), BodyRef::Rigid(fixed_body)));
			auto const repeated_link = constraints.Add(constraint(BodyRef::Link(articulation_a, child_a), BodyRef::Rigid(fixed_body)));
			auto body_ptrs = std::array<RigidBody*, 2>{&dynamic_body, &fixed_body};
			auto articulation_ptrs = std::array<Articulation*, 2>{&articulation_a, &articulation_b};
			auto const upload = PackGpuConstraints(constraints, BodyRemap(body_ptrs, articulation_ptrs));

			PR_EXPECT(upload.m_coupled_islands.size() == 2);
			PR_EXPECT(upload.m_coupled_articulation_indices == std::vector<int>({0, 1}));
			PR_EXPECT(upload.m_coupled_articulation_islands == std::vector<int>({0, 1}));
			PR_EXPECT(upload.m_coupled_block_topology[articulation_dynamic.m_index].island_idx == 0);
			PR_EXPECT(upload.m_coupled_block_topology[articulation_fixed.m_index].island_idx == 1);
			PR_EXPECT(upload.m_coupled_block_topology[repeated_link.m_index].island_idx == 0);
			PR_EXPECT(upload.m_coupled_island_blocks == std::vector<uint32_t>({
				articulation_dynamic.m_index,
				repeated_link.m_index,
				articulation_fixed.m_index,
			}));

			PR_EXPECT(upload.m_coupled_targets.size() == 3);
			PR_EXPECT(upload.m_coupled_targets[0].target_type == GpuCoupledConstraintTargetType_Link);
			PR_EXPECT(upload.m_coupled_targets[0].target_idx == 1);
			PR_EXPECT(upload.m_coupled_targets[0].island_idx == 0);
			PR_EXPECT(upload.m_coupled_targets[1].target_type == GpuCoupledConstraintTargetType_Rigid);
			PR_EXPECT(upload.m_coupled_targets[1].target_idx == 0);
			PR_EXPECT(upload.m_coupled_targets[2].target_type == GpuCoupledConstraintTargetType_Link);
			PR_EXPECT(upload.m_coupled_targets[2].target_idx == 3);
			PR_EXPECT(upload.m_coupled_targets[2].island_idx == 1);
			PR_EXPECT(upload.m_coupled_target_adjacency == std::vector<uint32_t>({
				2U * articulation_dynamic.m_index,
				2U * repeated_link.m_index,
				2U * articulation_dynamic.m_index + 1U,
				2U * articulation_fixed.m_index,
			}));
			PR_EXPECT(upload.m_coupled_block_topology[articulation_fixed.m_index].target_idx_b == -1);
			PR_EXPECT(upload.m_coupled_block_topology[repeated_link.m_index].target_idx_b == -1);
		}

		// Connected-body collision suppression is deduplicated into a compact canonical hash table.
		PRUnitTestMethod(PacksCollisionExclusions, Quick)
		{
			auto body_a = RigidBody{};
			auto body_b = RigidBody{};
			auto desc = D6ConstraintDesc{};
			desc.m_frame_a.m_body = BodyRef::Rigid(body_a);
			desc.m_frame_b.m_body = BodyRef::Rigid(body_b);
			desc.m_linear[0].m_mode = EConstraintAxisMode::Locked;

			auto constraints = ConstraintSet{};
			constraints.Add(desc);
			constraints.Add(desc);
			auto collide_desc = desc;
			collide_desc.m_collide_connected = true;
			constraints.Add(collide_desc);
			auto disabled_desc = desc;
			disabled_desc.m_enabled = false;
			constraints.Add(disabled_desc);

			auto body_ptrs = std::array<RigidBody*, 2>{&body_b, &body_a};
			auto const upload = PackGpuConstraints(constraints, BodyRemap(body_ptrs));
			auto const& exclusions = upload.m_collision_exclusions;

			PR_EXPECT(exclusions.m_count == 1);
			PR_EXPECT(exclusions.m_slots.size() == 4);
			PR_EXPECT(exclusions.Mask() == 3);
			PR_EXPECT(std::ranges::count_if(exclusions.m_slots, [](GpuCollisionExclusion const& entry)
			{
				return entry.body_idx_a_plus_one == 1 && entry.body_idx_b_plus_one == 2;
			}) == 1);
		}
	};
}
#endif
