//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/physics/physics.h"
#include "src/unittests/shared_engine.h"

namespace pr::physics::tests
{
	namespace
	{
		// One fixed-root prismatic tree exposing a single translational generalized coordinate.
		struct EnginePrismaticTree
		{
			Articulation m_articulation;
			LinkHandle m_link;
		};

		// Return finite spherical mass properties for compact engine-level articulation fixtures.
		ArticulationLinkDesc CoupledEngineLink(float mass = 1.0f)
		{
			return ArticulationLinkDesc{
				.m_inertia = Inertia::Sphere(0.2f, mass),
			};
		}

		// Build a fixed-root tree whose moving link translates along one world-space cardinal axis.
		EnginePrismaticTree MakeEnginePrismaticTree(v4 axis, float root_position, float position)
		{
			auto builder = ArticulationBuilder{};
			auto const root = builder.AddFixedRoot(CoupledEngineLink(2.0f), m4x4::Translation(root_position * axis));
			auto joint = ArticulationJointDesc::Prismatic(axis);
			joint.m_initial_position[0] = position;
			auto const link = builder.AddLink(root, joint, CoupledEngineLink());
			return EnginePrismaticTree{
				.m_articulation = builder.Build(),
				.m_link = link,
			};
		}

		// Build one freely moving root link at a displaced world-space X coordinate.
		std::pair<Articulation, LinkHandle> MakeEngineFloatingRoot(float position_x)
		{
			auto builder = ArticulationBuilder{};
			auto const root = builder.AddFloatingRoot(CoupledEngineLink(), m4x4::Translation(position_x, 0.0f, 0.0f));
			return {builder.Build(), root};
		}

		// Return one hard linear row between arbitrary supported endpoints.
		D6ConstraintDesc CoupledEngineLinear(BodyRef endpoint_a, BodyRef endpoint_b, int axis = 0)
		{
			auto desc = D6ConstraintDesc{};
			desc.m_frame_a.m_body = endpoint_a;
			desc.m_frame_b.m_body = endpoint_b;
			desc.m_linear[axis].m_mode = EConstraintAxisMode::Locked;
			desc.m_linear[axis].m_max_force = 10000.0f;
			return desc;
		}

		// Configure deterministic constraint-only engine runs without sleeping or selective contact passes.
		void ConfigureCoupledEngine(Engine& engine)
		{
			ResetEngineForNextTest(engine);
			auto config = EngineConfig{};
			config.sleeping_enabled = false;
			config.selective_refresh_passes = 0;
			config.constraint_warm_start_factor = 0.0f;
			engine.Config(config);
		}

		// Build a fixed three-link chain whose spherical proxies all share one world-space centre.
		Articulation MakeOverlappingContactTree(collision::ShapeSphere const& shape, bool collide_parent, bool collide_self)
		{
			auto make_link = [&](float shape_offset_x)
			{
				auto link_desc = CoupledEngineLink();
				link_desc.m_shape = collision::shape_cast(&shape);
				link_desc.m_shape_to_link = m4x4::Translation(shape_offset_x, 0.0f, 0.0f);
				link_desc.m_collide_parent = collide_parent;
				link_desc.m_collide_self = collide_self;
				return link_desc;
			};

			auto builder = ArticulationBuilder{};
			auto const root = builder.AddFixedRoot(make_link(0.0f), m4x4::Identity());
			auto const child = builder.AddLink(root, ArticulationJointDesc::Fixed(), make_link(0.2f));
			builder.AddLink(child, ArticulationJointDesc::Fixed(), make_link(0.4f));
			return builder.Build();
		}
	}

	// Prove production Engine scheduling covers mixed and articulation-only coupled topologies.
	PRUnitTestClass(CoupledConstraintEngineTests)
	{
		// Correct a fixed-root generalized coordinate and an ordinary rigid transform without changing physical momentum.
		PRUnitTestMethod(MixedRigidAndFixedRootPositionCorrection, Quick)
		{
			auto tree = MakeEnginePrismaticTree(v4::XAxis(), 0.0f, 0.25f);
			auto shape = collision::ShapeSphere{0.2f};
			auto body = RigidBody{&shape, m4x4::Translation(0.9f, 0.0f, 0.0f), Inertia::Sphere(shape.m_radius, 1.0f)};
			auto constraints = ConstraintSet{};
			constraints.Add(CoupledEngineLinear(BodyRef::Link(tree.m_articulation, tree.m_link), BodyRef::Rigid(body)));
			auto body_ptrs = std::array<RigidBody*, 1>{&body};
			auto articulation_ptrs = std::array<Articulation*, 1>{&tree.m_articulation};
			auto const initial_error = Abs(tree.m_articulation.LinkToWorld(tree.m_link).pos.x - body.O2W().pos.x);
			auto& engine = SharedEngine();
			ConfigureCoupledEngine(engine);

			engine.Step(Engine::StepInput{
				.m_bodies = body_ptrs,
				.m_articulations = articulation_ptrs,
				.m_constraints = &constraints,
				.m_elapsed_seconds = 1.0f / 60.0f,
			});

			auto const final_error = Abs(tree.m_articulation.LinkToWorld(tree.m_link).pos.x - body.O2W().pos.x);
			PR_EXPECT(final_error < initial_error);
			PR_EXPECT(Abs(tree.m_articulation.JointVelocity(tree.m_link)[0]) < 1.0e-5f);
			PR_EXPECT(FEqlAbsolute(body.MomentumWS().ang, v4::Zero(), 1.0e-6f));
			PR_EXPECT(FEqlAbsolute(body.MomentumWS().lin, v4::Zero(), 1.0e-6f));
		}

		// Solve a link-to-link island with no ordinary rigid bodies while retaining one submission, wait, and readback.
		PRUnitTestMethod(ArticulationToArticulationRunsWithoutRigidBodies, Quick)
		{
			auto tree_a = MakeEnginePrismaticTree(v4::XAxis(), 0.0f, 0.2f);
			auto tree_b = MakeEnginePrismaticTree(v4::XAxis(), 1.0f, -0.2f);
			auto constraints = ConstraintSet{};
			constraints.Add(CoupledEngineLinear(
				BodyRef::Link(tree_a.m_articulation, tree_a.m_link),
				BodyRef::Link(tree_b.m_articulation, tree_b.m_link)));
			auto articulation_ptrs = std::array<Articulation*, 2>{&tree_a.m_articulation, &tree_b.m_articulation};
			auto const initial_error = Abs(tree_a.m_articulation.LinkToWorld(tree_a.m_link).pos.x - tree_b.m_articulation.LinkToWorld(tree_b.m_link).pos.x);
			auto& engine = SharedEngine();
			ConfigureCoupledEngine(engine);

			engine.Step(Engine::StepInput{
				.m_articulations = articulation_ptrs,
				.m_constraints = &constraints,
				.m_elapsed_seconds = 1.0f / 30.0f,
				.m_substep_count = 4,
			});

			auto const final_error = Abs(tree_a.m_articulation.LinkToWorld(tree_a.m_link).pos.x - tree_b.m_articulation.LinkToWorld(tree_b.m_link).pos.x);
			auto const& profile = engine.LastStepProfile();
			PR_EXPECT(final_error < initial_error);
			PR_EXPECT(profile.m_submission_count == 1);
			PR_EXPECT(profile.m_wait_count == 1);
			PR_EXPECT(profile.m_readback_copy_count == 1);
		}

		// Release every optional coupled stream on an inactive frame and rebuild it when the same persistent slot is re-enabled.
		PRUnitTestMethod(CoupledResourcesDeactivateAndReuse, Quick)
		{
			auto [articulation, root] = MakeEngineFloatingRoot(0.3f);
			auto constraints = ConstraintSet{};
			auto const handle = constraints.Add(CoupledEngineLinear(BodyRef::Link(articulation, root), BodyRef::World()));
			auto articulation_ptrs = std::array<Articulation*, 1>{&articulation};
			auto& engine = SharedEngine();
			ConfigureCoupledEngine(engine);

			engine.Step(Engine::StepInput{
				.m_articulations = articulation_ptrs,
				.m_constraints = &constraints,
				.m_elapsed_seconds = 1.0f / 60.0f,
			});
			PR_EXPECT(Abs(articulation.RootToWorld().pos.x) < 0.3f);

			// The disabled frame retains the public slot but must not retain or execute coupled feature resources.
			constraints.SetEnabled(handle, false);
			articulation.RootToWorld(m4x4::Translation(0.45f, 0.0f, 0.0f));
			engine.Step(Engine::StepInput{
				.m_articulations = articulation_ptrs,
				.m_constraints = &constraints,
				.m_elapsed_seconds = 1.0f / 60.0f,
			});
			PR_EXPECT(Abs(articulation.RootToWorld().pos.x - 0.45f) < 1.0e-5f);

			constraints.SetEnabled(handle, true);
			engine.Step(Engine::StepInput{
				.m_articulations = articulation_ptrs,
				.m_constraints = &constraints,
				.m_elapsed_seconds = 1.0f / 60.0f,
			});
			PR_EXPECT(Abs(articulation.RootToWorld().pos.x) < 0.45f);
		}

		// Keep a mixed coupled island coherent when selective contact cleanup changes its ordinary rigid endpoint.
		PRUnitTestMethod(SelectiveContactRefreshContinuesCoupledSolve, Quick)
		{
			auto tree = MakeEnginePrismaticTree(v4::ZAxis(), 0.0f, 0.6f);
			auto sphere_shape = collision::ShapeSphere{0.5f};
			auto ground_shape = collision::ShapeBox{v4{5.0f, 5.0f, 0.5f, 0.0f}};
			auto body = RigidBody{};
			body.Shape(collision::shape_cast(&sphere_shape), 1.0f);
			body.O2W(m4x4::Translation(0.0f, 0.0f, 0.6f));
			auto ground = RigidBody{};
			ground.Shape(collision::shape_cast(&ground_shape), Inertia::Infinite());
			ground.O2W(m4x4::Translation(0.0f, 0.0f, -0.5f));
			auto constraints = ConstraintSet{};
			auto constraint = CoupledEngineLinear(BodyRef::Link(tree.m_articulation, tree.m_link), BodyRef::Rigid(body), 2);
			constraint.m_collide_connected = true;
			constraints.Add(constraint);
			auto body_ptrs = std::array<RigidBody*, 2>{&body, &ground};
			auto articulation_ptrs = std::array<Articulation*, 1>{&tree.m_articulation};
			auto& engine = SharedEngine();
			ConfigureCoupledEngine(engine);
			auto config = engine.Config();
			config.selective_refresh_passes = 1;
			config.selective_refresh_position_iterations = 4;
			config.selective_refresh_depth_slop = 0.0f;
			config.selective_refresh_support_depth_slop = 0.0f;
			engine.Config(config);
			engine.Material(Material{
				.m_id = Material::DefaultID,
				.m_friction_static = 0.0f,
				.m_elasticity_norm = 0.0f,
			});

			// Stop on the first contact-bearing frame so a later full solve cannot conceal selective-pass drift.
			auto saw_contact = false;
			for (int frame = 0; frame != 60 && !saw_contact; ++frame)
			{
				body.ZeroForces();
				ground.ZeroForces();
				body.ApplyForceWS(v4{0.0f, 0.0f, -9.81f * body.Mass(), 0.0f}, v4::Zero(), body.O2W().rot * body.CentreOfMassOS());
				engine.Step(Engine::StepInput{
					.m_bodies = body_ptrs,
					.m_articulations = articulation_ptrs,
					.m_constraints = &constraints,
					.m_elapsed_seconds = 1.0f / 60.0f,
				});
				saw_contact = engine.LastCollisionStats().m_contact_count != 0;
			}
			auto const constraint_error = Abs(tree.m_articulation.LinkToWorld(tree.m_link).pos.z - body.O2W().pos.z);
			PR_EXPECT(saw_contact);
			PR_EXPECT(constraint_error < 0.002f);
		}

		// Admit shaped proxies to broadphase and narrowphase without exposing or resolving them through rigid-only paths.
		PRUnitTestMethod(ArticulationProxyContactsRemainCoupledOnly, Quick)
		{
			auto shape = collision::ShapeSphere{0.5f};
			auto link_desc = CoupledEngineLink();
			link_desc.m_shape = collision::shape_cast(&shape);
			auto builder = ArticulationBuilder{};
			auto const root = builder.AddFloatingRoot(link_desc, m4x4::Identity());
			auto articulation = builder.Build();
			auto body = RigidBody{&shape, m4x4::Translation(0.6f, 0.0f, 0.0f), Inertia::Sphere(shape.m_radius, 1.0f)};
			auto body_ptrs = std::array<RigidBody*, 1>{&body};
			auto articulation_ptrs = std::array<Articulation*, 1>{&articulation};
			auto const initial_body_transform = body.O2W();
			auto const initial_root_transform = articulation.RootToWorld();
			auto collision_event_count = 0;
			auto& engine = SharedEngine();
			ConfigureCoupledEngine(engine);
			engine.Collisions += [&](Engine&, std::span<RbContact const> contacts)
			{
				collision_event_count += isize(contacts);
			};

			engine.Step(Engine::StepInput{
				.m_bodies = body_ptrs,
				.m_articulations = articulation_ptrs,
				.m_elapsed_seconds = 1.0f / 60.0f,
			});

			PR_EXPECT(engine.LastCollisionStats().m_pair_count == 1);
			PR_EXPECT(engine.LastCollisionStats().m_contact_count != 0);
			PR_EXPECT(collision_event_count == 0);
			PR_EXPECT(FEqlAbsolute(body.O2W(), initial_body_transform, 1.0e-6f));
			PR_EXPECT(FEqlAbsolute(body.MomentumWS().ang, v4::Zero(), 1.0e-6f));
			PR_EXPECT(FEqlAbsolute(body.MomentumWS().lin, v4::Zero(), 1.0e-6f));
			PR_EXPECT(FEqlAbsolute(articulation.RootToWorld(), initial_root_transform, 1.0e-6f));
			PR_EXPECT(FEqlAbsolute(articulation.LinkVelocity(root).ang, v4::Zero(), 1.0e-6f));
			PR_EXPECT(FEqlAbsolute(articulation.LinkVelocity(root).lin, v4::Zero(), 1.0e-6f));
		}

		// Enforce adjacent and broader same-tree collision policies without growing exclusion storage quadratically.
		PRUnitTestMethod(ArticulationProxySelfCollisionPolicy, Quick)
		{
			auto shape = collision::ShapeSphere{0.5f};
			auto articulation_ptrs = std::array<Articulation*, 1>{};
			auto& engine = SharedEngine();

			// Default parent suppression leaves only the non-adjacent root-to-grandchild pair.
			auto default_tree = MakeOverlappingContactTree(shape, false, true);
			articulation_ptrs[0] = &default_tree;
			ConfigureCoupledEngine(engine);
			engine.Step(Engine::StepInput{
				.m_articulations = articulation_ptrs,
				.m_elapsed_seconds = 1.0f / 60.0f,
			});
			PR_EXPECT(engine.LastCollisionStats().m_pair_count == 1);
			PR_EXPECT(engine.LastCollisionStats().m_contact_count != 0);

			// Disabling same-tree collision on each link removes the remaining non-adjacent pair as well.
			auto filtered_tree = MakeOverlappingContactTree(shape, false, false);
			articulation_ptrs[0] = &filtered_tree;
			ConfigureCoupledEngine(engine);
			engine.Step(Engine::StepInput{
				.m_articulations = articulation_ptrs,
				.m_elapsed_seconds = 1.0f / 60.0f,
			});
			PR_EXPECT(engine.LastCollisionStats().m_pair_count == 0);
			PR_EXPECT(engine.LastCollisionStats().m_contact_count == 0);

			// Explicit adjacent collision restores all three pairs independently of the default policy.
			auto enabled_tree = MakeOverlappingContactTree(shape, true, true);
			articulation_ptrs[0] = &enabled_tree;
			ConfigureCoupledEngine(engine);
			engine.Step(Engine::StepInput{
				.m_articulations = articulation_ptrs,
				.m_elapsed_seconds = 1.0f / 60.0f,
			});
			PR_EXPECT(engine.LastCollisionStats().m_pair_count == 3);
			PR_EXPECT(engine.LastCollisionStats().m_contact_count != 0);
		}
	};
}
#endif
