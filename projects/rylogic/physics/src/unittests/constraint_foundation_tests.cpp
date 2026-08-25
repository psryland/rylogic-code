//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/physics/physics.h"
#include "src/constraint/constraint_compiler.h"

namespace pr::physics::tests
{
	namespace
	{
		// Return a minimally valid D6 descriptor between two endpoints.
		D6ConstraintDesc MakeD6(BodyRef body_a, BodyRef body_b)
		{
			auto desc = D6ConstraintDesc{};
			desc.m_frame_a.m_body = body_a;
			desc.m_frame_b.m_body = body_b;
			return desc;
		}

		// Return a hard-locked scalar axis.
		ConstraintAxisDesc LockedAxis()
		{
			auto axis = ConstraintAxisDesc{};
			axis.m_mode = EConstraintAxisMode::Locked;
			return axis;
		}

		// Require two floating-point values to agree within an absolute tolerance.
		void ExpectNear(float actual, float expected, float tolerance = 1.0e-5f)
		{
			PR_EXPECT(std::abs(actual - expected) <= tolerance);
		}

		// Require a spatial force vector to agree with its expected angular and linear components.
		void ExpectSpatial(v8force actual, v4 expected_angular, v4 expected_linear)
		{
			PR_EXPECT(FEqlAbsolute(actual.ang, expected_angular, 1.0e-5f));
			PR_EXPECT(FEqlAbsolute(actual.lin, expected_linear, 1.0e-5f));
		}

		// Build a two-link floating tree with a controllable root orientation and one translated child attachment.
		std::pair<Articulation, LinkHandle> MakeConstraintTree(float root_x, float root_angle = 0.0f)
		{
			auto const link = ArticulationLinkDesc{
				.m_inertia = Inertia::Sphere(0.25f, 1.0f),
			};
			auto builder = ArticulationBuilder{};
			auto const root = builder.AddFloatingRoot(link, m4x4::Transform(v4::ZAxis(), root_angle, v4{root_x, 0.0f, 0.0f, 1}));
			auto const child = builder.AddLink(root, ArticulationJointDesc::Revolute(v4::ZAxis(), m4x4::Translation(1.0f, 0.0f, 0.0f)), link);
			return {builder.Build(), child};
		}
	}

	PRUnitTestClass(ConstraintFoundationTests)
	{
		// Preserve body identity through value relocation and reject simultaneous duplicate identities.
		PRUnitTestMethod(BodyIdentitySurvivesRelocation, Quick)
		{
			auto bodies = std::vector<RigidBody>{};
			bodies.emplace_back();
			auto const first_id = bodies.front().Id();
			for (int index = 0; index != 64; ++index)
				bodies.emplace_back();

			PR_EXPECT(static_cast<bool>(first_id));
			PR_EXPECT(bodies.front().Id() == first_id);
			for (int lhs = 0; lhs != isize(bodies); ++lhs)
				for (int rhs = lhs + 1; rhs != isize(bodies); ++rhs)
					PR_EXPECT(bodies[lhs].Id() != bodies[rhs].Id());

			auto duplicate = bodies.front();
			auto duplicate_ptrs = std::array<RigidBody*, 2>{&bodies.front(), &duplicate};
			PR_THROWS((void)BodyRemap(duplicate_ptrs), std::exception);
		}

		// Reuse descriptor slots with a new generation and reject every operation through the stale handle.
		PRUnitTestMethod(ConstraintHandlesAreGenerational, Quick)
		{
			auto body_a = RigidBody{};
			auto body_b = RigidBody{};
			auto desc = MakeD6(BodyRef::Rigid(body_a), BodyRef::Rigid(body_b));
			desc.m_linear[0] = LockedAxis();
			auto constraints = ConstraintSet{};

			auto const original = constraints.Add(desc);
			PR_EXPECT(constraints.Contains(original));
			PR_EXPECT(constraints.Count() == 1);
			PR_EXPECT(constraints.CapacitySlots() == 1);

			constraints.Remove(original);
			PR_EXPECT(!constraints.Contains(original));
			PR_EXPECT(constraints.Count() == 0);
			PR_THROWS(constraints.Get(original), std::exception);
			PR_THROWS(constraints.Update(original, desc), std::exception);
			PR_THROWS(constraints.SetEnabled(original, false), std::exception);
			PR_THROWS(constraints.Remove(original), std::exception);

			auto const replacement = constraints.Add(desc);
			PR_EXPECT(replacement.m_index == original.m_index);
			PR_EXPECT(replacement.m_generation != original.m_generation);
			PR_EXPECT(constraints.Contains(replacement));
			PR_EXPECT(constraints.Count() == 1);
		}

		// Merge adjacent dirty slots while retaining separated update ranges and removed tombstones.
		PRUnitTestMethod(DirtyRangesTrackDescriptorUploads, Quick)
		{
			auto body_a = RigidBody{};
			auto body_b = RigidBody{};
			auto desc = MakeD6(BodyRef::Rigid(body_a), BodyRef::Rigid(body_b));
			desc.m_linear[0] = LockedAxis();
			auto constraints = ConstraintSet{};
			auto handles = std::array{
				constraints.Add(desc),
				constraints.Add(desc),
				constraints.Add(desc),
			};

			auto ranges = constraints.DirtyRanges();
			PR_EXPECT(ranges.size() == 1);
			PR_EXPECT(ranges[0].m_begin == 0 && ranges[0].m_end == 3);

			constraints.ClearDirty();
			desc.m_linear[0].m_target_position = 0.25f;
			constraints.Update(handles[0], desc);
			constraints.Update(handles[2], desc);
			ranges = constraints.DirtyRanges();
			PR_EXPECT(ranges.size() == 2);
			PR_EXPECT(ranges[0].m_begin == 0 && ranges[0].m_end == 1);
			PR_EXPECT(ranges[1].m_begin == 2 && ranges[1].m_end == 3);

			constraints.Update(handles[1], desc);
			ranges = constraints.DirtyRanges();
			PR_EXPECT(ranges.size() == 1);
			PR_EXPECT(ranges[0].m_begin == 0 && ranges[0].m_end == 3);

			constraints.ClearDirty();
			constraints.Remove(handles[1]);
			ranges = constraints.DirtyRanges();
			PR_EXPECT(ranges.size() == 1);
			PR_EXPECT(ranges[0].m_begin == 1 && ranges[0].m_end == 2);
		}

		// Emit active D6 rows in canonical axis order with correct anchor Jacobians and current position errors.
		PRUnitTestMethod(D6CompilerEmitsDeterministicRows, Quick)
		{
			auto body_a = RigidBody{};
			auto body_b = RigidBody{};
			auto const com_a = v4{0.0f, +0.25f, 0.0f, 0.0f};
			auto const com_b = v4{0.0f, -0.50f, 0.0f, 0.0f};
			body_a.SetMassProperties(Inertia::Sphere(1.0f, 1.0f, com_a), com_a);
			body_b.SetMassProperties(Inertia::Sphere(1.0f, 1.0f, com_b), com_b);
			body_a.O2W(m4x4::Identity());
			body_b.O2W(m4x4::Translation(2.0f, 0.0f, 0.0f));

			auto desc = MakeD6(BodyRef::Rigid(body_a), BodyRef::Rigid(body_b));
			desc.m_frame_a.m_constraint_to_body = m4x4::Translation(0.0f, 1.0f, 0.0f);
			desc.m_frame_b.m_constraint_to_body = m4x4::Transform(v4::XAxis(), 0.25f, v4::Origin());
			desc.m_linear[0] = LockedAxis();
			desc.m_linear[2].m_mode = EConstraintAxisMode::Limited;
			desc.m_linear[2].m_limits = Range<float>{-0.5f, +0.5f};
			desc.m_angular[0].m_mode = EConstraintAxisMode::Driven;
			desc.m_angular[0].m_target_velocity = 1.5f;
			desc.m_angular[0].m_max_force = 12.0f;
			desc.m_angular[1] = LockedAxis();

			auto constraints = ConstraintSet{};
			auto const handle = constraints.Add(desc);
			auto body_ptrs = std::array<RigidBody*, 2>{&body_a, &body_b};
			auto const compiled = CompileConstraints(constraints, BodyRemap(body_ptrs));

			PR_EXPECT(compiled.m_blocks.size() == 1);
			PR_EXPECT(compiled.m_rows.size() == 4);
			auto const& block = compiled.m_blocks[0];
			PR_EXPECT(block.m_source == handle);
			PR_EXPECT(block.m_endpoint_a.m_rigid_index == 0);
			PR_EXPECT(block.m_endpoint_b.m_rigid_index == 1);
			PR_EXPECT(block.m_row_begin == 0);
			PR_EXPECT(block.m_row_count == 4);

			auto const& linear_x = compiled.m_rows[0];
			PR_EXPECT(linear_x.m_kind == EConstraintRowKind::Linear);
			PR_EXPECT(linear_x.m_axis == 0);
			PR_EXPECT(linear_x.m_mode == EConstraintAxisMode::Locked);
			ExpectNear(linear_x.m_position, 2.0f);
			ExpectSpatial(linear_x.m_jacobian_a, v4{0, 0, +0.75f, 0}, v4{-1, 0, 0, 0});
			ExpectSpatial(linear_x.m_jacobian_b, v4{0, 0, -0.50f, 0}, v4{+1, 0, 0, 0});

			auto const& linear_z = compiled.m_rows[1];
			PR_EXPECT(linear_z.m_kind == EConstraintRowKind::Linear);
			PR_EXPECT(linear_z.m_axis == 2);
			PR_EXPECT(linear_z.m_mode == EConstraintAxisMode::Limited);
			ExpectNear(linear_z.m_position, 0.0f);
			ExpectSpatial(linear_z.m_jacobian_a, v4{-0.75f, 0, 0, 0}, v4{0, 0, -1, 0});
			ExpectSpatial(linear_z.m_jacobian_b, v4{+0.50f, 0, 0, 0}, v4{0, 0, +1, 0});

			auto const& angular_x = compiled.m_rows[2];
			PR_EXPECT(angular_x.m_kind == EConstraintRowKind::Angular);
			PR_EXPECT(angular_x.m_axis == 0);
			PR_EXPECT(angular_x.m_mode == EConstraintAxisMode::Driven);
			ExpectNear(angular_x.m_position, 0.25f);
			ExpectNear(angular_x.m_target_velocity, 1.5f);
			ExpectNear(angular_x.m_max_force, 12.0f);
			ExpectSpatial(angular_x.m_jacobian_a, v4{-1, 0, 0, 0}, v4::Zero());
			ExpectSpatial(angular_x.m_jacobian_b, v4{+1, 0, 0, 0}, v4::Zero());

			auto const& angular_y = compiled.m_rows[3];
			PR_EXPECT(angular_y.m_kind == EConstraintRowKind::Angular);
			PR_EXPECT(angular_y.m_axis == 1);
			PR_EXPECT(angular_y.m_mode == EConstraintAxisMode::Locked);
			ExpectNear(angular_y.m_position, 0.0f);
		}

		// Resolve stable endpoints to each frame's current packed indices after reorder and insertion.
		PRUnitTestMethod(EndpointRemapSurvivesBodyOrdering, Quick)
		{
			auto body_a = RigidBody{};
			auto body_b = RigidBody{};
			auto inserted = RigidBody{};
			auto desc = MakeD6(BodyRef::Rigid(body_a), BodyRef::Rigid(body_b));
			desc.m_linear[0] = LockedAxis();
			auto constraints = ConstraintSet{};
			constraints.Add(desc);

			auto original_order = std::array<RigidBody*, 2>{&body_a, &body_b};
			auto reordered = std::array<RigidBody*, 3>{&inserted, &body_b, &body_a};
			auto missing_endpoint = std::array<RigidBody*, 2>{&inserted, &body_a};
			auto const original = CompileConstraints(constraints, BodyRemap(original_order));
			auto const remapped = CompileConstraints(constraints, BodyRemap(reordered));

			PR_EXPECT(original.m_blocks[0].m_endpoint_a.m_rigid_index == 0);
			PR_EXPECT(original.m_blocks[0].m_endpoint_b.m_rigid_index == 1);
			PR_EXPECT(remapped.m_blocks[0].m_endpoint_a.m_rigid_index == 2);
			PR_EXPECT(remapped.m_blocks[0].m_endpoint_b.m_rigid_index == 1);
			PR_THROWS(CompileConstraints(constraints, BodyRemap(missing_endpoint)), std::exception);
		}

		// Compile named joint descriptors to the expected D6 row counts without introducing separate solver primitives.
		PRUnitTestMethod(NamedDescriptorsCompileToD6, Quick)
		{
			auto body_a = RigidBody{};
			auto body_b = RigidBody{};
			auto const frame_a = BodyFrame{.m_body = BodyRef::Rigid(body_a)};
			auto const frame_b = BodyFrame{.m_body = BodyRef::Rigid(body_b)};
			auto constraints = ConstraintSet{};
			auto const ball = constraints.Add(BallSocketConstraintDesc{.m_frame_a = frame_a, .m_frame_b = frame_b});
			auto const hinge = constraints.Add(HingeConstraintDesc{.m_frame_a = frame_a, .m_frame_b = frame_b});
			auto const slider = constraints.Add(SliderConstraintDesc{.m_frame_a = frame_a, .m_frame_b = frame_b});
			auto const weld = constraints.Add(WeldConstraintDesc{.m_frame_a = frame_a, .m_frame_b = frame_b});
			auto body_ptrs = std::array<RigidBody*, 2>{&body_a, &body_b};

			auto const compiled = CompileConstraints(constraints, BodyRemap(body_ptrs));

			PR_EXPECT(compiled.m_blocks.size() == 4);
			PR_EXPECT(compiled.m_rows.size() == 19);
			PR_EXPECT(compiled.m_blocks[0].m_source == ball && compiled.m_blocks[0].m_row_count == 3);
			PR_EXPECT(compiled.m_blocks[1].m_source == hinge && compiled.m_blocks[1].m_row_count == 5);
			PR_EXPECT(compiled.m_blocks[2].m_source == slider && compiled.m_blocks[2].m_row_count == 5);
			PR_EXPECT(compiled.m_blocks[3].m_source == weld && compiled.m_blocks[3].m_row_count == 6);
		}

		// Omit enabled all-free descriptors so unused persistent declarations create no active solver work.
		PRUnitTestMethod(AllFreeD6ProducesNoActiveBlock, Quick)
		{
			auto body_a = RigidBody{};
			auto body_b = RigidBody{};
			auto constraints = ConstraintSet{};
			constraints.Add(MakeD6(BodyRef::Rigid(body_a), BodyRef::Rigid(body_b)));
			auto body_ptrs = std::array<RigidBody*, 2>{&body_a, &body_b};

			auto const compiled = CompileConstraints(constraints, BodyRemap(body_ptrs));

			PR_EXPECT(compiled.m_blocks.empty());
			PR_EXPECT(compiled.m_rows.empty());
		}

		// Keep world Jacobians zero while compiling the dynamic endpoint normally.
		PRUnitTestMethod(WorldEndpointHasNoJacobian, Quick)
		{
			auto body = RigidBody{};
			auto desc = MakeD6(BodyRef::Rigid(body), BodyRef::World());
			desc.m_linear[0] = LockedAxis();
			desc.m_frame_b.m_constraint_to_body = m4x4::Translation(1.0f, 0.0f, 0.0f);
			auto constraints = ConstraintSet{};
			constraints.Add(desc);
			auto body_ptrs = std::array<RigidBody*, 1>{&body};

			auto const compiled = CompileConstraints(constraints, BodyRemap(body_ptrs));

			PR_EXPECT(compiled.m_blocks[0].m_endpoint_a.m_rigid_index == 0);
			PR_EXPECT(compiled.m_blocks[0].m_endpoint_b.IsWorld());
			ExpectSpatial(compiled.m_rows[0].m_jacobian_a, v4::Zero(), v4{-1, 0, 0, 0});
			ExpectSpatial(compiled.m_rows[0].m_jacobian_b, v4::Zero(), v4::Zero());
			ExpectNear(compiled.m_rows[0].m_position, 1.0f);
		}

		// Resolve stable articulation/link identities to their hidden-proxy slots and compile rows in link coordinates.
		PRUnitTestMethod(ArticulationLinkEndpointsCompileInLinkFrame, Quick)
		{
			auto [articulation, child] = MakeConstraintTree(10.0f, constants<float>::tau_by_4);
			auto desc = MakeD6(BodyRef::Link(articulation, child), BodyRef::World());
			desc.m_frame_a.m_constraint_to_body = m4x4::Translation(0.0f, 1.0f, 0.0f);
			desc.m_frame_b.m_constraint_to_body = m4x4::Translation(9.0f, 1.0f, 0.0f);
			desc.m_linear[0] = LockedAxis();
			auto constraints = ConstraintSet{};
			constraints.Add(desc);
			auto articulation_ptrs = std::array<Articulation*, 1>{&articulation};
			auto remap = BodyRemap({}, articulation_ptrs);

			auto const endpoint = remap.ResolveEndpoint(desc.m_frame_a.m_body);
			PR_EXPECT(endpoint.IsLink());
			PR_EXPECT(endpoint.m_articulation_index == 0);
			PR_EXPECT(endpoint.m_link_index == 1);
			PR_EXPECT(endpoint.m_packed_body_index == 1);
			PR_EXPECT(endpoint.m_link == child);

			auto const compiled = CompileConstraints(constraints, remap);
			PR_EXPECT(compiled.m_blocks.size() == 1);
			PR_EXPECT(compiled.m_blocks[0].m_endpoint_a.IsLink());
			PR_EXPECT(compiled.m_blocks[0].m_endpoint_b.IsWorld());
			PR_EXPECT(compiled.m_rows.size() == 1);
			ExpectNear(compiled.m_rows[0].m_position, 0.0f);
			ExpectSpatial(compiled.m_rows[0].m_jacobian_a, v4{0, 0, +1, 0}, v4{-1, 0, 0, 0});
			ExpectSpatial(compiled.m_rows[0].m_jacobian_b, v4::Zero(), v4::Zero());
		}

		// Reject missing, duplicate, stale, and cross-articulation link identities before compiling solver rows.
		PRUnitTestMethod(ArticulationLinkEndpointValidation, Quick)
		{
			auto [articulation_a, child_a] = MakeConstraintTree(0.0f);
			auto [articulation_b, child_b] = MakeConstraintTree(2.0f);
			PR_THROWS(BodyRef::Link(articulation_a, child_b), std::exception);

			auto const endpoint = BodyRef::Link(articulation_a, child_a);
			auto articulation_a_ptrs = std::array<Articulation*, 1>{&articulation_a};
			auto duplicate_ptrs = std::array<Articulation*, 2>{&articulation_a, &articulation_a};
			auto missing_remap = BodyRemap({}, std::span<Articulation* const>{});
			PR_THROWS(missing_remap.ResolveEndpoint(endpoint), std::exception);
			PR_THROWS((void)BodyRemap({}, duplicate_ptrs), std::exception);

			auto remap = BodyRemap({}, articulation_a_ptrs);
			auto stale = endpoint;
			++stale.m_link.m_generation;
			PR_THROWS(remap.ResolveEndpoint(stale), std::exception);
		}

		// Reject invalid endpoint topology, frames, axis modes, and position limits before persistent storage changes.
		PRUnitTestMethod(InvalidDescriptorsAreRejected, Quick)
		{
			auto body = RigidBody{};
			auto constraints = ConstraintSet{};
			auto world_to_world = MakeD6(BodyRef::World(), BodyRef::World());
			auto self = MakeD6(BodyRef::Rigid(body), BodyRef::Rigid(body));
			auto invalid_frame = MakeD6(BodyRef::Rigid(body), BodyRef::World());
			invalid_frame.m_frame_a.m_constraint_to_body.x.x = 2.0f;
			auto invalid_limit = MakeD6(BodyRef::Rigid(body), BodyRef::World());
			invalid_limit.m_linear[0].m_mode = EConstraintAxisMode::Limited;
			invalid_limit.m_linear[0].m_limits = Range<float>{+1.0f, -1.0f};
			auto invalid_mode = MakeD6(BodyRef::Rigid(body), BodyRef::World());
			invalid_mode.m_linear[0].m_mode = static_cast<EConstraintAxisMode>(100);

			PR_THROWS(constraints.Add(world_to_world), std::exception);
			PR_THROWS(constraints.Add(self), std::exception);
			PR_THROWS(constraints.Add(invalid_frame), std::exception);
			PR_THROWS(constraints.Add(invalid_limit), std::exception);
			PR_THROWS(constraints.Add(invalid_mode), std::exception);
			PR_EXPECT(constraints.Count() == 0);
		}
	};
}
#endif
