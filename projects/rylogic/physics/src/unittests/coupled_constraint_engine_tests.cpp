//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/physics/physics.h"
#include "src/compute/physics_types.h"
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

		// Settled metrics from one high-degree articulation support run.
		struct HighDegreeSupportResult
		{
			float m_max_settled_penetration = 0.0f;
			float m_max_support_gap = 0.0f;
			float m_max_energy = 0.0f;
			float m_max_settled_energy = 0.0f;
			int m_max_contact_count = 0;
			bool m_all_finite = true;
			int m_submission_count = 0;
			int m_wait_count = 0;
			int m_readback_copy_count = 0;
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

		// Build one shaped floating root for transient articulation-contact acceptance cases.
		std::pair<Articulation, LinkHandle> MakeContactFloatingRoot(collision::Shape const& shape, float position_x, float velocity_x = 0.0f)
		{
			auto link_desc = CoupledEngineLink();
			link_desc.m_shape = &shape;
			auto builder = ArticulationBuilder{};
			auto const root = builder.AddFloatingRoot(link_desc, m4x4::Translation(position_x, 0.0f, 0.0f), v8motion{v4::Zero(), velocity_x * v4::XAxis()});
			return {builder.Build(), root};
		}

		// Return one hard linear row between arbitrary supported endpoints.
		ConstraintDesc CoupledEngineLinear(BodyRef endpoint_a, BodyRef endpoint_b, int axis = 0)
		{
			auto desc = ConstraintDesc{};
			desc.m_frame_a.m_body = endpoint_a;
			desc.m_frame_b.m_body = endpoint_b;
			desc.D6().m_linear[axis].m_mode = EConstraintAxisMode::Locked;
			desc.D6().m_linear[axis].m_max_force = 10000.0f;
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

		// Rest a high-degree floating tree on fixed geometry and report its sustained support quality.
		HighDegreeSupportResult RunHighDegreeSupport(int support_link_count, int velocity_iteration_count, int position_iteration_count)
		{
			if (support_link_count <= 0)
				throw std::invalid_argument("High-degree support requires at least one articulation link");

			auto sphere_shape = collision::ShapeSphere{0.25f};
			auto link_desc = ArticulationLinkDesc{
				.m_inertia = Inertia::Sphere(sphere_shape.m_radius, 1.0f),
				.m_shape = collision::shape_cast(&sphere_shape),
				.m_collide_self = false,
			};
			auto builder = ArticulationBuilder{};
			auto links = std::vector<LinkHandle>(support_link_count);
			auto link_count = 0;
			auto const root = builder.AddFloatingRoot(
				link_desc,
				m4x4::Translation(0.0f, 0.0f, 0.24f),
				v8motion{v4::Zero(), -2.0f * v4::ZAxis()});
			links[link_count++] = root;

			// Coincident fixed links create the coherent redundant-contact worst case at the requested participant degree.
			for (; link_count != isize(links); ++link_count)
				links[link_count] = builder.AddLink(root, ArticulationJointDesc::Fixed(), link_desc);

			auto articulation = builder.Build();
			for (auto const link : links)
				articulation.GravityWS(link, v4{0.0f, 0.0f, -9.81f, 0.0f});

			// Exercise a deliberately low fixed velocity budget while varying only the detached position sweep count.
			auto ground_shape = collision::ShapeBox{v4{8.0f, 8.0f, 1.0f, 0.0f}};
			auto ground = RigidBody{collision::shape_cast(&ground_shape), m4x4::Translation(0.0f, 0.0f, -0.5f), Inertia::Infinite()};
			auto body_ptrs = std::array<RigidBody*, 1>{&ground};
			auto articulation_ptrs = std::array<Articulation*, 1>{&articulation};
			auto& engine = SharedEngine();
			ConfigureCoupledEngine(engine);
			auto config = engine.Config();
			config.solver_iterations = velocity_iteration_count;
			config.push_out_iterations = position_iteration_count;
			engine.Config(config);
			engine.Material(Material{
				.m_id = Material::DefaultID,
				.m_friction_static = 0.0f,
				.m_elasticity_norm = 0.0f,
			});

			// Retain only aggregate metrics over the settled second half so the regression remains allocation-free inside the frame loop.
			auto result = HighDegreeSupportResult{};
			for (int frame = 0; frame != 180; ++frame)
			{
				engine.Step(Engine::StepInput{
					.m_bodies = body_ptrs,
					.m_articulations = articulation_ptrs,
					.m_elapsed_seconds = 1.0f / 60.0f,
				});

				auto penetration = 0.0f;
				for (auto const link : links)
				{
					auto const link_z = articulation.LinkToWorld(link).pos.z;
					result.m_all_finite &= std::isfinite(link_z);
					penetration = std::max(penetration, sphere_shape.m_radius - link_z);
					result.m_max_support_gap = std::max(result.m_max_support_gap, link_z - sphere_shape.m_radius);
				}
				auto const energy = articulation.KineticEnergy();
				result.m_all_finite &= std::isfinite(energy);
				result.m_max_energy = std::max(result.m_max_energy, energy);
				result.m_max_contact_count = std::max(result.m_max_contact_count, engine.LastCollisionStats().m_contact_count);
				if (frame >= 90)
				{
					result.m_max_settled_penetration = std::max(result.m_max_settled_penetration, penetration);
					result.m_max_settled_energy = std::max(result.m_max_settled_energy, energy);
				}
			}

			// Preserve the frame-level GPU contract while exposing the physical acceptance metrics.
			auto const& profile = engine.LastStepProfile();
			result.m_submission_count = profile.m_submission_count;
			result.m_wait_count = profile.m_wait_count;
			result.m_readback_copy_count = profile.m_readback_copy_count;
			return result;
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

		// Solve a link-to-link island with substep submissions but only one completion wait and readback.
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
			PR_EXPECT(profile.m_submission_count == 4);
			PR_EXPECT(profile.m_wait_count == 1);
			PR_EXPECT(profile.m_readback_copy_count == 1);
		}

		// Report every coupled dispatch recorded across internal substeps rather than only the final preparation pass.
		PRUnitTestMethod(CoupledDispatchDiagnosticsCoverCompleteFrame, Quick)
		{
			auto run = [](int substep_count)
			{
				auto tree_a = MakeEnginePrismaticTree(v4::XAxis(), 0.0f, 0.2f);
				auto tree_b = MakeEnginePrismaticTree(v4::XAxis(), 1.0f, -0.2f);
				auto constraints = ConstraintSet{};
				constraints.Add(CoupledEngineLinear(
					BodyRef::Link(tree_a.m_articulation, tree_a.m_link),
					BodyRef::Link(tree_b.m_articulation, tree_b.m_link)));
				auto articulation_ptrs = std::array<Articulation*, 2>{&tree_a.m_articulation, &tree_b.m_articulation};
				auto& engine = SharedEngine();
				ConfigureCoupledEngine(engine);
				engine.Step(Engine::StepInput{
					.m_articulations = articulation_ptrs,
					.m_constraints = &constraints,
					.m_elapsed_seconds = 1.0f / 30.0f,
					.m_substep_count = substep_count,
				});
				return engine.LastFeatureStats().m_coupled.m_resources.m_dispatch_count;
			};

			auto const single_substep_dispatches = run(1);
			auto const four_substep_dispatches = run(4);
			PR_EXPECT(single_substep_dispatches > 0);
			PR_EXPECT(four_substep_dispatches > single_substep_dispatches);
		}


		// A rejected frame must not expose the temporary wake-up required to stage a coupled constraint.
		PRUnitTestMethod(PreSubmitFailureRestoresSleepingArticulation, Extended)
		{
			auto tree = MakeEnginePrismaticTree(v4::XAxis(), 0.0f, 0.0f);
			tree.m_articulation.Sleep();
			auto constraints = ConstraintSet{};
			constraints.Add(CoupledEngineLinear(BodyRef::Link(tree.m_articulation, tree.m_link), BodyRef::World()));
			auto articulation_ptrs = std::array<Articulation*, 1>{&tree.m_articulation};
			auto& engine = SharedEngine();
			ResetEngineForNextTest(engine);
			engine.ExternalForces += [](Engine&, Engine::ExternalForceArgs const&)
			{
				throw std::runtime_error("Intentional coupled pre-submit failure");
			};

			PR_THROWS(engine.Step(Engine::StepInput{
				.m_articulations = articulation_ptrs,
				.m_constraints = &constraints,
				.m_elapsed_seconds = 1.0f / 60.0f,
			}), std::runtime_error);
			PR_EXPECT(tree.m_articulation.Sleeping());
		}

		// A throwing post-commit notification propagates without restoring the pre-step articulation sleep state over published motion.
		PRUnitTestMethod(PostCommitNotificationPreservesPublishedState, Extended)
		{
			auto tree = MakeEnginePrismaticTree(v4::XAxis(), 0.0f, 0.0f);
			tree.m_articulation.Sleep();
			auto shape = collision::ShapeSphere{0.2f};
			auto body = RigidBody{&shape, m4x4::Identity(), Inertia::Sphere(shape.m_radius, 1.0f)};
			body.VelocityWS(v8motion{v4::Zero(), 5.0f * v4::XAxis()});
			auto desc = CoupledEngineLinear(BodyRef::Link(tree.m_articulation, tree.m_link), BodyRef::Rigid(body));
			desc.m_break_force = 10.0f;
			auto constraints = ConstraintSet{};
			auto const handle = constraints.Add(desc);
			auto body_ptrs = std::array<RigidBody*, 1>{&body};
			auto articulation_ptrs = std::array<Articulation*, 1>{&tree.m_articulation};
			auto& engine = SharedEngine();
			ResetEngineForNextTest(engine);
			engine.ConstraintsBroken += [](auto&, auto)
			{
				throw std::runtime_error("Intentional post-commit notification failure");
			};

			PR_THROWS(engine.Step(Engine::StepInput{
				.m_bodies = body_ptrs,
				.m_articulations = articulation_ptrs,
				.m_constraints = &constraints,
				.m_elapsed_seconds = 1.0f / 60.0f,
			}), std::runtime_error);

			PR_EXPECT(constraints.IsBroken(handle));
			PR_EXPECT(!tree.m_articulation.Sleeping());
			PR_EXPECT(!engine.StepPending());

			// The consumed frame leaves the engine immediately reusable after the throwing observer is removed.
			engine.ConstraintsBroken.reset();
			engine.Step(Engine::StepInput{
				.m_bodies = body_ptrs,
				.m_articulations = articulation_ptrs,
				.m_constraints = &constraints,
				.m_elapsed_seconds = 1.0f / 60.0f,
			});
		}

		// Break an articulation-to-world row from committed generalized impulse without requiring an ordinary rigid body.
		PRUnitTestMethod(CoupledConstraintBreaksFromLinkLoad, Quick)
		{
			auto tree = MakeEnginePrismaticTree(v4::XAxis(), 0.0f, 0.0f);
			tree.m_articulation.JointVelocity(tree.m_link, std::array{5.0f});
			auto desc = CoupledEngineLinear(BodyRef::Link(tree.m_articulation, tree.m_link), BodyRef::World());
			desc.m_break_force = 10.0f;
			auto constraints = ConstraintSet{};
			auto const handle = constraints.Add(desc);
			auto articulation_ptrs = std::array<Articulation*, 1>{&tree.m_articulation};
			auto events = std::vector<ConstraintBreakEvent>{};
			auto& engine = SharedEngine();
			ConfigureCoupledEngine(engine);
			engine.ConstraintsBroken += [&](auto&, auto broken)
			{
				events.insert(events.end(), broken.begin(), broken.end());
			};

			engine.Step(Engine::StepInput{
				.m_articulations = articulation_ptrs,
				.m_constraints = &constraints,
				.m_elapsed_seconds = 1.0f / 30.0f,
				.m_substep_count = 4,
			});

			PR_EXPECT(constraints.IsBroken(handle));
			PR_EXPECT(events.size() == 1);
			PR_EXPECT(events[0].m_constraint == handle);
			PR_EXPECT(events[0].m_substep_index == 0);
			PR_EXPECT(events[0].m_force > desc.m_break_force);
			PR_EXPECT(engine.LastStepProfile().m_readback_copy_count == 1);
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

		// Keep a mixed island coherent when only the selective pass corrects an initially satisfied joint.
		PRUnitTestMethod(SelectiveContactRefreshContinuesCoupledSolve, Quick)
		{
			auto tree = MakeEnginePrismaticTree(v4::ZAxis(), 0.0f, 0.15f);
			auto sphere_shape = collision::ShapeSphere{0.5f};
			auto ground_shape = collision::ShapeBox{v4{5.0f, 5.0f, 0.5f, 0.0f}};
			auto body = RigidBody{};
			body.Shape(collision::shape_cast(&sphere_shape), 1.0f);
			body.O2W(m4x4::Translation(0.0f, 0.0f, 0.15f));
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
			config.push_out_iterations = 0;
			config.velocity_baumgarte = 0.0f;
			config.deep_penetration_baumgarte_min = 0.0f;
			config.deep_penetration_baumgarte_max = 0.0f;
			config.selective_refresh_passes = 1;
			config.selective_refresh_position_iterations = 4;
			config.selective_refresh_support_only = false;
			config.selective_refresh_depth_slop = 0.0f;
			config.selective_refresh_support_depth_slop = 0.0f;
			engine.Config(config);
			engine.Material(Material{
				.m_id = Material::DefaultID,
				.m_friction_static = 0.0f,
				.m_elasticity_norm = 0.0f,
			});

			// Depth-based selection needs no gravity; starting in contact and disabling main push-out isolates selective continuation.
			engine.Step(Engine::StepInput{
				.m_bodies = body_ptrs,
				.m_articulations = articulation_ptrs,
				.m_constraints = &constraints,
				.m_elapsed_seconds = 1.0f / 60.0f,
			});
			// The first completed residual admits the selective continuation on the following frame.
			engine.Step(Engine::StepInput{
				.m_bodies = body_ptrs,
				.m_articulations = articulation_ptrs,
				.m_constraints = &constraints,
				.m_elapsed_seconds = 1.0f / 60.0f,
			});
			auto const constraint_error = Abs(tree.m_articulation.LinkToWorld(tree.m_link).pos.z - body.O2W().pos.z);
			PR_EXPECT(engine.LastCollisionStats().m_contact_count != 0);
			PR_EXPECT(tree.m_articulation.LinkToWorld(tree.m_link).pos.z > 0.16f);
			PR_EXPECT(body.O2W().pos.z > 0.16f);
			PR_EXPECT(constraint_error < 0.002f);
		}

		// Transfer one proxy impact through the floating tree while keeping proxy contacts private to the coupled lane.
		PRUnitTestMethod(ArticulationProxyContactTransfersMomentumThroughWholeTree, Quick)
		{
			auto shape = collision::ShapeSphere{0.5f};
			auto link_desc = CoupledEngineLink();
			link_desc.m_shape = collision::shape_cast(&shape);
			auto builder = ArticulationBuilder{};
			auto const initial_speed = 2.0f;
			auto const root = builder.AddFloatingRoot(link_desc, m4x4::Identity(), v8motion{v4::Zero(), initial_speed * v4::XAxis()});
			auto articulation = builder.Build();
			auto body = RigidBody{&shape, m4x4::Translation(0.9f, 0.0f, 0.0f), Inertia::Sphere(shape.m_radius, 1.0f)};
			auto body_ptrs = std::array<RigidBody*, 1>{&body};
			auto articulation_ptrs = std::array<Articulation*, 1>{&articulation};
			auto collision_event_count = 0;
			auto& engine = SharedEngine();
			ConfigureCoupledEngine(engine);
			engine.Material(Material{
				.m_id = Material::DefaultID,
				.m_friction_static = 0.0f,
				.m_elasticity_norm = 0.0f,
			});
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
			PR_EXPECT(FEqlAbsolute(body.MomentumWS().ang, v4::Zero(), 1.0e-6f));
			PR_EXPECT(FEqlAbsolute(articulation.LinkVelocity(root).ang, v4::Zero(), 1.0e-6f));
			PR_EXPECT(body.MomentumWS().lin.x > 0.1f);
			PR_EXPECT(articulation.LinkVelocity(root).lin.x < initial_speed - 0.1f);
			PR_EXPECT(Abs(body.MomentumWS().lin.x + articulation.LinkVelocity(root).lin.x - initial_speed) < 1.0e-3f);
			auto const uncorrected_separation = 0.9f - initial_speed / 60.0f;
			auto const corrected_separation = body.O2W().pos.x - articulation.LinkToWorld(root).pos.x;
			PR_EXPECT(corrected_separation > uncorrected_separation + 0.01f);
			PR_EXPECT(engine.LastStepProfile().m_submission_count == 1);
			PR_EXPECT(engine.LastStepProfile().m_readback_copy_count == 1);
		}

		// Preserve a rigid impulse when its body index and contact index occupy different GPU groups.
		PRUnitTestMethod(RigidEndpointIndexDoesNotAliasContactIndex, Quick)
		{
			auto proxy_shape = collision::ShapeSphere{0.5f};
			auto filler_shape = collision::ShapeSphere{0.1f};
			auto [articulation, root] = MakeContactFloatingRoot(proxy_shape, 0.0f, 2.0f);
			auto bodies = std::vector<RigidBody>{};
			bodies.reserve(65);

			// Keep the first 64 ordinary bodies isolated so the only contact has contact index zero and rigid endpoint index 64.
			for (int index = 0; index != 64; ++index)
				bodies.emplace_back(&filler_shape, m4x4::Translation(100.0f + 2.0f * index, 0.0f, 0.0f), Inertia::Sphere(filler_shape.m_radius, 1.0f));

			bodies.emplace_back(&proxy_shape, m4x4::Translation(0.9f, 0.0f, 0.0f), Inertia::Sphere(proxy_shape.m_radius, 1.0f));
			auto body_ptrs = std::vector<RigidBody*>{};
			body_ptrs.reserve(bodies.size());
			for (auto& body : bodies)
				body_ptrs.push_back(&body);

			auto articulation_ptrs = std::array<Articulation*, 1>{&articulation};
			auto& engine = SharedEngine();
			ConfigureCoupledEngine(engine);
			engine.Material(Material{
				.m_id = Material::DefaultID,
				.m_friction_static = 0.0f,
				.m_elasticity_norm = 0.0f,
			});

			engine.Step(Engine::StepInput{
				.m_bodies = body_ptrs,
				.m_articulations = articulation_ptrs,
				.m_elapsed_seconds = 1.0f / 60.0f,
			});

			auto const rigid_momentum = bodies.back().MomentumWS().lin.x;
			auto const articulation_momentum = articulation.LinkVelocity(root).lin.x;
			PR_EXPECT(engine.LastCollisionStats().m_pair_count == 1);
			PR_EXPECT(engine.LastCollisionStats().m_contact_count != 0);
			PR_EXPECT(rigid_momentum > 0.1f);
			PR_EXPECT(Abs(rigid_momentum + articulation_momentum - 2.0f) < 1.0e-3f);
		}

		// Separate an initially resting floating tree from fixed geometry without changing physical momentum.
		PRUnitTestMethod(ArticulationProxyContactCorrectsPositionWithoutMomentum, Quick)
		{
			auto shape = collision::ShapeSphere{0.5f};
			auto link_desc = CoupledEngineLink();
			link_desc.m_shape = collision::shape_cast(&shape);
			auto builder = ArticulationBuilder{};
			auto const root = builder.AddFloatingRoot(link_desc);
			auto articulation = builder.Build();
			auto ground = RigidBody{&shape, m4x4::Translation(0.9f, 0.0f, 0.0f), Inertia::Infinite()};
			auto body_ptrs = std::array<RigidBody*, 1>{&ground};
			auto articulation_ptrs = std::array<Articulation*, 1>{&articulation};
			auto& engine = SharedEngine();
			ConfigureCoupledEngine(engine);
			engine.Material(Material{
				.m_id = Material::DefaultID,
				.m_friction_static = 0.0f,
				.m_elasticity_norm = 0.0f,
			});

			engine.Step(Engine::StepInput{
				.m_bodies = body_ptrs,
				.m_articulations = articulation_ptrs,
				.m_elapsed_seconds = 1.0f / 60.0f,
			});

			auto const separation = ground.O2W().pos.x - articulation.LinkToWorld(root).pos.x;
			PR_EXPECT(engine.LastCollisionStats().m_contact_count != 0);
			PR_EXPECT(separation > 0.91f);
			PR_EXPECT(FEqlAbsolute(ground.O2W().pos, v4{0.9f, 0.0f, 0.0f, 1.0f}, 1.0e-6f));
			PR_EXPECT(FEqlAbsolute(ground.MomentumWS().ang, v4::Zero(), 1.0e-6f));
			PR_EXPECT(FEqlAbsolute(ground.MomentumWS().lin, v4::Zero(), 1.0e-6f));
			PR_EXPECT(FEqlAbsolute(articulation.LinkVelocity(root).ang, v4::Zero(), 1.0e-6f));
			PR_EXPECT(FEqlAbsolute(articulation.LinkVelocity(root).lin, v4::Zero(), 1.0e-6f));
			PR_EXPECT(engine.LastStepProfile().m_submission_count == 1);
			PR_EXPECT(engine.LastStepProfile().m_readback_copy_count == 1);
		}

		// Keep sustained high-degree support bounded and passive for every practical detached position sweep count.
		PRUnitTestMethod(HighDegreeArticulationSupportRemainsStable, Extended)
		{
			for (auto const position_iteration_count : std::array{1, 2, 4})
			{
				auto const result = RunHighDegreeSupport(9, 4, position_iteration_count);
				log_stream() << std::format(
					"  [high-degree-support] velocity_iterations=4 position_iterations={} max_contacts={} max_penetration={:.6g} max_gap={:.6g} max_energy={:.6g} settled_energy={:.6g}\n",
					position_iteration_count,
					result.m_max_contact_count,
					result.m_max_settled_penetration,
					result.m_max_support_gap,
					result.m_max_energy,
					result.m_max_settled_energy);
				PR_EXPECT(result.m_all_finite);
				PR_EXPECT(result.m_max_contact_count >= 9);
				PR_EXPECT(result.m_max_settled_penetration < 0.01f);
				PR_EXPECT(result.m_max_support_gap < 0.005f);
				PR_EXPECT(result.m_max_energy < 0.01f);
				PR_EXPECT(result.m_max_settled_energy < 0.001f);
				PR_EXPECT(result.m_submission_count == 1);
				PR_EXPECT(result.m_wait_count == 1);
				PR_EXPECT(result.m_readback_copy_count == 1);
			}

			// A single legal velocity sweep must remain passive even when every coherent contact updates simultaneously.
			auto const single_sweep = RunHighDegreeSupport(9, 1, 1);
			log_stream() << std::format(
				"  [high-degree-support] velocity_iterations=1 position_iterations=1 max_contacts={} max_energy={:.6g}\n",
				single_sweep.m_max_contact_count,
				single_sweep.m_max_energy);
			PR_EXPECT(single_sweep.m_all_finite);
			PR_EXPECT(single_sweep.m_max_contact_count >= 9);
			PR_EXPECT(single_sweep.m_max_energy < 0.5f);
			PR_EXPECT(single_sweep.m_submission_count == 1);
			PR_EXPECT(single_sweep.m_wait_count == 1);
			PR_EXPECT(single_sweep.m_readback_copy_count == 1);

			// A degree-one-hundred coherent manifold must receive the same bounded geometric treatment as the practical support case.
			auto const hundred_contact = RunHighDegreeSupport(100, 4, 1);
			log_stream() << std::format(
				"  [high-degree-support] velocity_iterations=4 position_iterations=1 max_contacts={} max_gap={:.6g} max_energy={:.6g}\n",
				hundred_contact.m_max_contact_count,
				hundred_contact.m_max_support_gap,
				hundred_contact.m_max_energy);
			PR_EXPECT(hundred_contact.m_all_finite);
			PR_EXPECT(hundred_contact.m_max_contact_count >= 100);
			PR_EXPECT(hundred_contact.m_max_support_gap < 0.005f);
			PR_EXPECT(hundred_contact.m_max_energy < 0.01f);
			PR_EXPECT(hundred_contact.m_submission_count == 1);
			PR_EXPECT(hundred_contact.m_wait_count == 1);
			PR_EXPECT(hundred_contact.m_readback_copy_count == 1);
		}

		// Use conservative damping when a contact first requires correction after the fixed-manifold initial sweep.
		PRUnitTestMethod(LateActivatingPositionContactsUseConservativeDamping, Extended)
		{
			// Place nine shallow contacts opposite one deep contact so only the deep side contributes during the first position sweep.
			auto shape = collision::ShapeSphere{0.5f};
			auto link_desc = CoupledEngineLink();
			link_desc.m_shape = collision::shape_cast(&shape);
			auto builder = ArticulationBuilder{};
			auto const root = builder.AddFloatingRoot(link_desc);
			auto articulation = builder.Build();
			auto fixed_bodies = std::vector<RigidBody>{};
			fixed_bodies.reserve(10);
			for (int body_index = 0; body_index != 9; ++body_index)
				fixed_bodies.emplace_back(&shape, m4x4::Translation(-0.997f, 0.0f, 0.0f), Inertia::Infinite());

			fixed_bodies.emplace_back(&shape, m4x4::Translation(+0.9f, 0.0f, 0.0f), Inertia::Infinite());
			auto body_ptrs = std::vector<RigidBody*>{};
			body_ptrs.reserve(fixed_bodies.size());
			for (auto& body : fixed_bodies)
				body_ptrs.push_back(&body);

			auto articulation_ptrs = std::array<Articulation*, 1>{&articulation};

			// Two sweeps expose late activation while disabling physical impulses keeps the observed motion purely positional.
			auto& engine = SharedEngine();
			ConfigureCoupledEngine(engine);
			auto config = engine.Config();
			config.solver_iterations = 0;
			config.push_out_iterations = 2;
			config.constraint_max_position_speed = 100.0f;
			engine.Config(config);
			engine.Material(Material{
				.m_id = Material::DefaultID,
				.m_friction_static = 0.0f,
				.m_elasticity_norm = 0.0f,
			});
			engine.Step(Engine::StepInput{
				.m_bodies = body_ptrs,
				.m_articulations = articulation_ptrs,
				.m_elapsed_seconds = 1.0f / 60.0f,
			});

			// The shallow side may close its initial three-millimetre overlap but must not be over-corrected into a support gap.
			auto const root_x = articulation.LinkToWorld(root).pos.x;
			auto const shallow_support_gap = std::max(root_x - 0.003f, 0.0f);
			PR_EXPECT(engine.LastCollisionStats().m_contact_count >= 10);
			PR_EXPECT(shallow_support_gap < 0.005f);
			PR_EXPECT(FEqlAbsolute(articulation.LinkVelocity(root).ang, v4::Zero(), 1.0e-6f));
			PR_EXPECT(FEqlAbsolute(articulation.LinkVelocity(root).lin, v4::Zero(), 1.0e-6f));
			PR_EXPECT(engine.LastStepProfile().m_submission_count == 1);
			PR_EXPECT(engine.LastStepProfile().m_wait_count == 1);
			PR_EXPECT(engine.LastStepProfile().m_readback_copy_count == 1);
		}

		// Exchange momentum directly between two independent trees without introducing a rigid-body intermediary.
		PRUnitTestMethod(ArticulationToArticulationContactTransfersMomentum, Quick)
		{
			auto shape = collision::ShapeSphere{0.5f};
			auto [articulation_a, root_a] = MakeContactFloatingRoot(shape, 0.0f, 2.0f);
			auto [articulation_b, root_b] = MakeContactFloatingRoot(shape, 0.9f);
			auto articulation_ptrs = std::array<Articulation*, 2>{&articulation_a, &articulation_b};
			auto& engine = SharedEngine();
			ConfigureCoupledEngine(engine);
			engine.Material(Material{
				.m_id = Material::DefaultID,
				.m_friction_static = 0.0f,
				.m_elasticity_norm = 0.0f,
			});

			engine.Step(Engine::StepInput{
				.m_articulations = articulation_ptrs,
				.m_elapsed_seconds = 1.0f / 60.0f,
			});

			auto const velocity_a = articulation_a.LinkVelocity(root_a).lin.x;
			auto const velocity_b = articulation_b.LinkVelocity(root_b).lin.x;
			PR_EXPECT(engine.LastCollisionStats().m_pair_count == 1);
			PR_EXPECT(engine.LastCollisionStats().m_contact_count != 0);
			PR_EXPECT(velocity_a < 1.9f);
			PR_EXPECT(velocity_b > 0.1f);
			PR_EXPECT(Abs(velocity_a + velocity_b - 2.0f) < 1.0e-3f);
			PR_EXPECT(engine.LastStepProfile().m_submission_count == 1);
			PR_EXPECT(engine.LastStepProfile().m_readback_copy_count == 1);
		}

		// Drive a fixed-root prismatic coordinate from an external impact while preserving passive contact response.
		PRUnitTestMethod(FixedRootContactDrivesJointVelocityWithoutEnergyGain, Quick)
		{
			auto shape = collision::ShapeSphere{0.5f};
			auto link_desc = CoupledEngineLink();
			link_desc.m_shape = collision::shape_cast(&shape);
			auto builder = ArticulationBuilder{};
			auto const root = builder.AddFixedRoot(CoupledEngineLink(2.0f));
			auto const link = builder.AddLink(root, ArticulationJointDesc::Prismatic(v4::XAxis()), link_desc);
			auto articulation = builder.Build();
			auto body = RigidBody{&shape, m4x4::Translation(0.9f, 0.0f, 0.0f), Inertia::Sphere(shape.m_radius, 1.0f)};
			body.VelocityWS(v4::Zero(), -2.0f * v4::XAxis());
			auto body_ptrs = std::array<RigidBody*, 1>{&body};
			auto articulation_ptrs = std::array<Articulation*, 1>{&articulation};
			auto& engine = SharedEngine();
			ConfigureCoupledEngine(engine);
			engine.Material(Material{
				.m_id = Material::DefaultID,
				.m_friction_static = 0.0f,
				.m_elasticity_norm = 0.0f,
			});

			engine.Step(Engine::StepInput{
				.m_bodies = body_ptrs,
				.m_articulations = articulation_ptrs,
				.m_elapsed_seconds = 1.0f / 60.0f,
			});

			auto const body_speed = body.VelocityWS().lin.x;
			auto const joint_speed = articulation.JointVelocity(link)[0];
			auto const final_energy = 0.5f * (Sqr(body_speed) + Sqr(joint_speed));
			PR_EXPECT(engine.LastCollisionStats().m_contact_count != 0);
			PR_EXPECT(body_speed > -1.9f);
			PR_EXPECT(joint_speed < -0.1f);
			PR_EXPECT(final_energy <= 2.001f);
		}

		// Correct a non-adjacent same-tree overlap through a reduced coordinate while leaving physical velocity untouched.
		PRUnitTestMethod(SelfContactCorrectsFixedRootJointWithoutMomentum, Quick)
		{
			auto shape = collision::ShapeSphere{0.5f};
			auto root_desc = CoupledEngineLink(2.0f);
			root_desc.m_shape = collision::shape_cast(&shape);
			root_desc.m_collide_self = true;
			auto moving_desc = CoupledEngineLink();
			moving_desc.m_shape = collision::shape_cast(&shape);
			moving_desc.m_collide_self = true;
			auto joint = ArticulationJointDesc::Prismatic(v4::XAxis());
			joint.m_initial_position[0] = 0.9f;
			auto builder = ArticulationBuilder{};
			auto const root = builder.AddFixedRoot(root_desc);
			auto const middle = builder.AddLink(root, ArticulationJointDesc::Fixed(), CoupledEngineLink());
			auto const moving = builder.AddLink(middle, joint, moving_desc);
			auto articulation = builder.Build();
			auto articulation_ptrs = std::array<Articulation*, 1>{&articulation};
			auto& engine = SharedEngine();
			ConfigureCoupledEngine(engine);
			engine.Material(Material{
				.m_id = Material::DefaultID,
				.m_friction_static = 0.0f,
				.m_elasticity_norm = 0.0f,
			});

			engine.Step(Engine::StepInput{
				.m_articulations = articulation_ptrs,
				.m_elapsed_seconds = 1.0f / 60.0f,
			});

			PR_EXPECT(engine.LastCollisionStats().m_pair_count == 1);
			PR_EXPECT(engine.LastCollisionStats().m_contact_count != 0);
			PR_EXPECT(articulation.JointPosition(moving)[0] > 0.91f);
			PR_EXPECT(Abs(articulation.JointVelocity(moving)[0]) < 1.0e-6f);
		}

		// Keep equal-key endpoint reduction deterministic and passive across warm-started internal substeps and resource reuse.
		PRUnitTestMethod(EqualKeyContactReductionIsDeterministicAndPassive, Quick)
		{
			auto shape = collision::ShapeSphere{0.5f};
			auto& engine = SharedEngine();

			// Run a symmetric pair of contacts that reduce two endpoint contributions into the same articulation link.
			auto run_scenario = [&]
			{
				auto [articulation, root] = MakeContactFloatingRoot(shape, 0.0f);
				auto body_a = RigidBody{&shape, m4x4::Translation(-0.9f, 0.0f, 0.0f), Inertia::Sphere(shape.m_radius, 1.0f)};
				auto body_b = RigidBody{&shape, m4x4::Translation(+0.9f, 0.0f, 0.0f), Inertia::Sphere(shape.m_radius, 1.0f)};
				body_a.VelocityWS(v4::Zero(), +v4::XAxis());
				body_b.VelocityWS(v4::Zero(), -v4::XAxis());
				auto body_ptrs = std::array<RigidBody*, 2>{&body_a, &body_b};
				auto articulation_ptrs = std::array<Articulation*, 1>{&articulation};
				ConfigureCoupledEngine(engine);
				engine.Material(Material{
					.m_id = Material::DefaultID,
					.m_friction_static = 0.0f,
					.m_elasticity_norm = 0.0f,
				});

				engine.Step(Engine::StepInput{
					.m_bodies = body_ptrs,
					.m_articulations = articulation_ptrs,
					.m_elapsed_seconds = 1.0f / 30.0f,
					.m_substep_count = 4,
				});

				auto const velocity_a = body_a.VelocityWS().lin.x;
				auto const velocity_b = body_b.VelocityWS().lin.x;
				auto const root_velocity = articulation.LinkVelocity(root).lin.x;
				auto const final_energy = 0.5f * (Sqr(velocity_a) + Sqr(velocity_b) + Sqr(root_velocity));
				PR_EXPECT(final_energy <= 1.001f);
				PR_EXPECT(Abs(velocity_a + velocity_b + root_velocity) < 2.0e-3f);
				PR_EXPECT(engine.LastStepProfile().m_substep_count == 4);
				PR_EXPECT(engine.LastStepProfile().m_submission_count == 4);
				PR_EXPECT(engine.LastStepProfile().m_wait_count == 1);
				PR_EXPECT(engine.LastStepProfile().m_readback_copy_count == 1);
				return std::array{
					body_a.O2W().pos.x,
					body_b.O2W().pos.x,
					articulation.RootToWorld().pos.x,
					velocity_a,
					velocity_b,
					root_velocity,
				};
			};

			auto const result_a = run_scenario();
			auto const result_b = run_scenario();
			for (int index = 0; index != isize(result_a); ++index)
				PR_EXPECT(Abs(result_a[index] - result_b[index]) < 1.0e-6f);
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

			// Parent collision remains independent of broader self-collision and restores only the two tree edges.
			auto adjacent_only_tree = MakeOverlappingContactTree(shape, true, false);
			articulation_ptrs[0] = &adjacent_only_tree;
			ConfigureCoupledEngine(engine);
			engine.Step(Engine::StepInput{
				.m_articulations = articulation_ptrs,
				.m_elapsed_seconds = 1.0f / 60.0f,
			});
			PR_EXPECT(engine.LastCollisionStats().m_pair_count == 2);
			PR_EXPECT(engine.LastCollisionStats().m_contact_count != 0);

			// Enabling both policies admits every adjacent and non-adjacent pair.
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

	// Independent conservation and lifecycle examples for the reviewed integration boundaries.
	PRUnitTestClass(ConstraintReviewEngineTests)
	{
		// Unrepresentable rigid soft rows must reject the submitted frame instead of silently becoming hard or inactive rows.
		PRUnitTestMethod(RigidNumericalFailureRejectsFrameAndRecovers, Quick)
		{
			auto shape = collision::ShapeSphere{0.5f};
			auto body = RigidBody{&shape, m4x4::Translation(1.0f, 0.0f, 0.0f), Inertia::Sphere(shape.m_radius, 1.0f)};
			body.VelocityWS(v4::Zero(), v4::XAxis());
			auto desc = CoupledEngineLinear(BodyRef::World(), BodyRef::Rigid(body));
			desc.D6().m_linear[0].m_stiffness = 1.0e-36f;
			auto constraints = ConstraintSet{};
			auto handle = constraints.Add(desc);
			auto body_ptrs = std::array<RigidBody*, 1>{&body};
			auto input = Engine::StepInput{
				.m_bodies = body_ptrs,
				.m_constraints = &constraints,
				.m_elapsed_seconds = 1.0f / 30.0f,
				.m_substep_count = 2,
			};
			auto& engine = SharedEngine();
			ConfigureCoupledEngine(engine);

			// GPU prediction has advanced, but no caller-owned state may publish after the frame-sticky numerical failure.
			PR_THROWS(engine.Step(input), std::runtime_error);
			PR_EXPECT(engine.LastFeatureStats().m_failure.m_reason == EStepFailure::ConstraintNumerics);
			PR_EXPECT(engine.LastFeatureStats().m_failure.m_item_index == static_cast<int>(handle.m_index));
			PR_EXPECT(FEqlAbsolute(body.O2W().pos.x, 1.0f, 1.0e-7f));
			PR_EXPECT(FEqlAbsolute(body.VelocityWS().lin.x, 1.0f, 1.0e-7f));
			PR_EXPECT(engine.LastStepProfile().m_submission_count == 2);
			PR_EXPECT(engine.LastStepProfile().m_wait_count == 1);
			PR_EXPECT(engine.LastStepProfile().m_readback_copy_count == 1);

			// Corrected input starts a new frame and must not inherit the preceding failure latch.
			desc.D6().m_linear[0].m_stiffness = 0.01f;
			constraints.Update(handle, desc);
			engine.Step(input);
			PR_EXPECT(!engine.LastFeatureStats().m_failure.HasTerminalFailure());
			PR_EXPECT(body.O2W().pos.x > 1.0f);
		}


		// Saturating a driven row must leave the other coupled hard row satisfied.
		PRUnitTestMethod(BoundedCoupledBlockKeepsItsUnboundedRowSatisfied, Quick)
		{
			auto builder = ArticulationBuilder{};
			auto const root = builder.AddFloatingRoot(ArticulationLinkDesc{.m_inertia = Inertia{1.0f, 1.0f}}, m4x4::Identity(), v8motion{v4::Zero(), v4{-1.0f, -1.0f, 0.0f, 0.0f}});
			auto articulation = builder.Build();
			auto desc = ConstraintDesc{};
			desc.m_frame_a.m_constraint_to_body = m4x4::Translation(1.0f, -1.0f, 0.0f);
			desc.m_frame_b.m_body = BodyRef::Link(articulation, root);
			desc.m_frame_b.m_constraint_to_body = desc.m_frame_a.m_constraint_to_body;
			desc.D6().m_linear[0].m_mode = EConstraintAxisMode::Locked;
			desc.D6().m_linear[1].m_mode = EConstraintAxisMode::Driven;
			desc.D6().m_linear[1].m_max_force = 6.0f;
			auto constraints = ConstraintSet{};
			constraints.Add(desc);
			auto articulation_ptrs = std::array<Articulation*, 1>{&articulation};

			// The point response is [[2,1],[1,2]] and the driven impulse is capped at one tenth.
			auto& engine = SharedEngine();
			ConfigureCoupledEngine(engine);
			auto config = engine.Config();
			config.max_collision_pairs = 16;
			config.solver_iterations = 32;
			config.push_out_iterations = 0;
			engine.Config(config);
			engine.Step(Engine::StepInput{
				.m_articulations = articulation_ptrs,
				.m_constraints = &constraints,
				.m_elapsed_seconds = 1.0f / 60.0f,
			});

			// Fixing the saturated impulse and solving the remaining row gives impulses (0.45,0.1).
			auto const velocity = articulation.LinkVelocity(root);
			auto const point_velocity = velocity.lin + Cross(velocity.ang, v4{1.0f, -1.0f, 0.0f, 0.0f});
			PR_EXPECT(Abs(point_velocity.x) < 1.0e-4f);
			PR_EXPECT(Abs(point_velocity.y + 0.35f) < 1.0e-4f);
			PR_EXPECT(Abs(velocity.lin.y + 0.9f) < 1.0e-4f);
		}

		// A unilateral pseudo row must not distort the correction of another coupled hard row.
		PRUnitTestMethod(BoundedCoupledPositionPreservesFreeRowResponse, Quick)
		{
			auto builder = ArticulationBuilder{};
			auto const root = builder.AddFloatingRoot(ArticulationLinkDesc{.m_inertia = Inertia{1.0f, 1.0f}}, m4x4::Identity());
			auto articulation = builder.Build();
			auto desc = ConstraintDesc{};
			desc.m_frame_a.m_constraint_to_body = m4x4::Translation(2.0f, -0.9999f, 0.0f);
			desc.m_frame_b.m_body = BodyRef::Link(articulation, root);
			desc.m_frame_b.m_constraint_to_body = m4x4::Translation(1.0f, -1.0f, 0.0f);
			desc.D6().m_linear[0].m_mode = EConstraintAxisMode::Locked;
			desc.D6().m_linear[1].m_mode = EConstraintAxisMode::Limited;
			desc.D6().m_linear[1].m_limits = {0.0f, 1.0f};
			auto constraints = ConstraintSet{};
			constraints.Add(desc);
			auto articulation_ptrs = std::array<Articulation*, 1>{&articulation};
			auto& engine = SharedEngine();
			ConfigureCoupledEngine(engine);
			auto config = engine.Config();
			config.max_collision_pairs = 16;
			config.push_out_iterations = 1;
			config.constraint_coupled_relaxation = 1.0f;
			config.constraint_max_position_speed = 2.0f;
			engine.Config(config);

			// K=[[2,1],[1,2]], target X=2, and the lower-limit Y impulse cannot be negative, so the exact impulses are (1,0).
			auto const dt = 1.0f / 60.0f;
			engine.Step(Engine::StepInput{
				.m_articulations = articulation_ptrs,
				.m_constraints = &constraints,
				.m_elapsed_seconds = dt,
			});
			auto const expected_translation_x = dt * std::cos(0.5f * dt);
			PR_EXPECT(Abs(articulation.RootToWorld().pos.x - expected_translation_x) < 1.0e-4f);
			PR_EXPECT(Length(articulation.LinkVelocity(root).lin) < 1.0e-6f);
			PR_EXPECT(Length(articulation.LinkVelocity(root).ang) < 1.0e-6f);
			PR_EXPECT(!engine.LastFeatureStats().m_failure.HasTerminalFailure());
		}

		// A tangential slip must not determine the normal impulse of a frictionless contact.
		PRUnitTestMethod(OffCentreFrictionlessContactRemainsPassive, Quick)
		{
			auto shape = collision::ShapeSphere{0.1f};
			auto link_desc = ArticulationLinkDesc{
				.m_inertia = Inertia{1.0f, 1.0f},
				.m_shape = collision::shape_cast(&shape),
				.m_shape_to_link = m4x4::Translation(1.0f, 0.0f, -0.9f),
			};
			auto builder = ArticulationBuilder{};
			auto const root = builder.AddFloatingRoot(link_desc, m4x4::Translation(0.0f, 0.0f, 0.995f), v8motion{v4::Zero(), v4{10.0f, 0.0f, -1.0f, 0.0f}});
			auto articulation = builder.Build();
			auto ground_shape = collision::ShapeBox{v4{10.0f, 10.0f, 1.0f, 0.0f}};
			auto ground = RigidBody{&ground_shape, m4x4::Translation(0.0f, 0.0f, -0.5f), Inertia::Infinite()};
			auto body_ptrs = std::array<RigidBody*, 1>{&ground};
			auto articulation_ptrs = std::array<Articulation*, 1>{&articulation};

			// Isolate the velocity solve from gravity, cached impulses, and coordinate correction.
			auto& engine = SharedEngine();
			ConfigureCoupledEngine(engine);
			auto config = engine.Config();
			config.max_collision_pairs = 16;
			config.push_out_iterations = 0;
			config.warm_start_scale = 0.0f;
			engine.Config(config);
			engine.Material(Material{
				.m_id = Material::DefaultID,
				.m_friction_static = 0.0f,
				.m_elasticity_norm = 0.0f,
			});
			engine.Step(Engine::StepInput{
				.m_bodies = body_ptrs,
				.m_articulations = articulation_ptrs,
				.m_elapsed_seconds = 1.0f / 60.0f,
			});

			// Unit mass and inertia give normal mobility two and the exact stopping impulse one half.
			auto const velocity = articulation.LinkVelocity(root);
			auto const energy = 0.5f * (LengthSq(velocity.ang) + LengthSq(velocity.lin));
			PR_EXPECT(engine.LastCollisionStats().m_contact_count != 0);
			PR_EXPECT(energy <= 50.501f);
			PR_EXPECT(Abs(velocity.lin.x - 10.0f) < 1.0e-4f);
			PR_EXPECT(Abs(velocity.lin.z + 0.5f) < 0.005f);
			PR_EXPECT(Abs(velocity.ang.y + 0.5f) < 0.005f);
			PR_EXPECT(engine.LastStepProfile().m_readback_copy_count == 1);
		}

		// Coulomb friction requires normal support; pure sliding cannot manufacture a separating impulse.
		PRUnitTestMethod(FrictionDoesNotCreateNormalSupportFromPureSlip, Quick)
		{
			auto shape = collision::ShapeSphere{0.5f};
			auto builder = ArticulationBuilder{};
			auto const root = builder.AddFloatingRoot(ArticulationLinkDesc{.m_inertia = Inertia{1.0f, 1.0f}, .m_shape = collision::shape_cast(&shape)}, m4x4::Translation(0.0f, 0.0f, 0.495f), v8motion{v4::Zero(), 4.0f * v4::XAxis()});
			auto articulation = builder.Build();
			auto ground_shape = collision::ShapeBox{v4{10.0f, 10.0f, 1.0f, 0.0f}};
			auto ground = RigidBody{&ground_shape, m4x4::Translation(0.0f, 0.0f, -0.5f), Inertia::Infinite()};
			auto body_ptrs = std::array<RigidBody*, 1>{&ground};
			auto articulation_ptrs = std::array<Articulation*, 1>{&articulation};

			// No gravity or position correction supplies a normal impulse in this shallow contact.
			auto& engine = SharedEngine();
			ConfigureCoupledEngine(engine);
			auto config = engine.Config();
			config.max_collision_pairs = 16;
			config.push_out_iterations = 0;
			config.warm_start_scale = 0.0f;
			engine.Config(config);
			engine.Material(Material{
				.m_id = Material::DefaultID,
				.m_friction_static = 0.5f,
				.m_elasticity_norm = 0.0f,
			});
			engine.Step(Engine::StepInput{
				.m_bodies = body_ptrs,
				.m_articulations = articulation_ptrs,
				.m_elapsed_seconds = 1.0f / 60.0f,
			});

			// A joint cone-energy projection would incorrectly lift this body to obtain a friction budget.
			auto const velocity = articulation.LinkVelocity(root);
			PR_EXPECT(engine.LastCollisionStats().m_contact_count != 0);
			PR_EXPECT(Length(velocity.lin - 4.0f * v4::XAxis()) < 1.0e-4f);
			PR_EXPECT(Length(velocity.ang) < 1.0e-4f);
		}

		// Exercise collision discovery and optional later integration without any CPU wake between internal substeps.
		static void ExerciseSleepingCollision(int substep_count)
		{
			auto shape = collision::ShapeSphere{0.5f};
			auto [articulation, root] = MakeContactFloatingRoot(shape, 0.0f);
			articulation.Sleep();
			auto body = RigidBody{&shape, m4x4::Translation(1.01f, 0.0f, 0.0f), Inertia::Sphere(shape.m_radius, 1.0f)};
			body.VelocityWS(v4::Zero(), -2.0f * v4::XAxis());
			auto body_ptrs = std::array<RigidBody*, 1>{&body};
			auto articulation_ptrs = std::array<Articulation*, 1>{&articulation};

			// No external D6 constraint or force setter may pre-wake this tree and mask collision discovery.
			auto& engine = SharedEngine();
			ConfigureCoupledEngine(engine);
			auto config = engine.Config();
			config.max_collision_pairs = 16;
			config.sleeping_enabled = true;
			config.push_out_iterations = 0;
			config.warm_start_scale = 0.0f;
			engine.Config(config);
			engine.Material(Material{
				.m_id = Material::DefaultID,
				.m_friction_static = 0.0f,
				.m_elasticity_norm = 0.0f,
			});
			engine.Step(Engine::StepInput{
				.m_bodies = body_ptrs,
				.m_articulations = articulation_ptrs,
				.m_elapsed_seconds = static_cast<float>(substep_count) / 60.0f,
				.m_substep_count = substep_count,
			});

			// An inelastic collision shares the incoming unit mass's momentum with the complete tree.
			auto const tree_velocity = articulation.LinkVelocity(root).lin.x;
			PR_EXPECT(engine.LastCollisionStats().m_contact_count != 0);
			PR_EXPECT(!articulation.Sleeping());
			PR_EXPECT(tree_velocity < -0.2f);
			PR_EXPECT(body.VelocityWS().lin.x > -1.8f);
			PR_EXPECT(Abs(tree_velocity + body.MomentumWS().lin.x + 2.0f) < 0.005f);
			PR_EXPECT(engine.LastStepProfile().m_submission_count == substep_count);
			PR_EXPECT(engine.LastStepProfile().m_wait_count == 1);
			PR_EXPECT(engine.LastStepProfile().m_readback_copy_count == 1);
			if (substep_count > 1)
				PR_EXPECT(articulation.RootToWorld().pos.x < -0.01f);

		}

		// Sleeping reduces dynamics work without making the tree's collision geometry disappear.
		PRUnitTestMethod(IncomingBodyWakesSleepingArticulation, Quick)
		{
			ExerciseSleepingCollision(1);
		}

		// A collision wake must take effect before the next GPU-resident internal substep.
		PRUnitTestMethod(CollisionWakeIntegratesLaterInternalSubstep, Quick)
		{
			ExerciseSleepingCollision(2);
		}

		// A retained collider must not integrate gravity merely because another body keeps the frame active.
		PRUnitTestMethod(UntouchedSleepingColliderRetainsState, Quick)
		{
			auto shape = collision::ShapeSphere{0.5f};
			auto [articulation, root] = MakeContactFloatingRoot(shape, 0.0f);
			articulation.RootToWorld(m4x4::Translation(0.0f, 0.0f, 5.0f));
			articulation.GravityWS(root, -9.81f * v4::ZAxis());
			articulation.Sleep();
			auto body = RigidBody{&shape, m4x4::Translation(10.0f, 0.0f, 0.0f), Inertia::Sphere(shape.m_radius, 1.0f)};
			body.VelocityWS(v4::Zero(), v4::XAxis());
			auto body_ptrs = std::array<RigidBody*, 1>{&body};
			auto articulation_ptrs = std::array<Articulation*, 1>{&articulation};

			// Multiple substeps must continue to skip this tree while maintaining its collision proxy.
			auto& engine = SharedEngine();
			ConfigureCoupledEngine(engine);
			auto config = engine.Config();
			config.max_collision_pairs = 16;
			config.sleeping_enabled = true;
			engine.Config(config);
			engine.Step(Engine::StepInput{
				.m_bodies = body_ptrs,
				.m_articulations = articulation_ptrs,
				.m_elapsed_seconds = 3.0f / 60.0f,
				.m_substep_count = 3,
			});
			PR_EXPECT(articulation.Sleeping());
			PR_EXPECT(FEqlAbsolute(articulation.RootToWorld().pos, v4{0.0f, 0.0f, 5.0f, 1.0f}, 1.0e-7f));
			PR_EXPECT(Length(articulation.LinkVelocity(root).lin) == 0.0f);
			PR_EXPECT(engine.LastCollisionStats().m_contact_count == 0);
			PR_EXPECT(engine.LastStepProfile().m_submission_count == 3);
			PR_EXPECT(engine.LastStepProfile().m_wait_count == 1);
			PR_EXPECT(engine.LastStepProfile().m_readback_copy_count == 1);
		}

		// Cached impulses must not wake a resting tree when the current contact has no physical closing motion.
		PRUnitTestMethod(WarmStartDoesNotWakeSleepingArticulation, Quick)
		{
			auto shape = collision::ShapeSphere{0.5f};
			auto [articulation, root] = MakeContactFloatingRoot(shape, 0.0f);
			auto body = RigidBody{&shape, m4x4::Translation(1.01f, 0.0f, 0.0f), Inertia::Sphere(shape.m_radius, 1.0f)};
			body.VelocityWS(v4::Zero(), -2.0f * v4::XAxis());
			auto body_ptrs = std::array<RigidBody*, 1>{&body};
			auto articulation_ptrs = std::array<Articulation*, 1>{&articulation};
			auto input = Engine::StepInput{
				.m_bodies = body_ptrs,
				.m_articulations = articulation_ptrs,
				.m_elapsed_seconds = 1.0f / 60.0f,
			};

			// Populate a nonzero support cache before putting only the articulation to sleep.
			auto& engine = SharedEngine();
			ConfigureCoupledEngine(engine);
			auto config = engine.Config();
			config.max_collision_pairs = 16;
			config.sleeping_enabled = true;
			config.push_out_iterations = 0;
			config.warm_start_scale = 1.0f;
			engine.Config(config);
			engine.Material(Material{
				.m_id = Material::DefaultID,
				.m_friction_static = 0.0f,
				.m_elasticity_norm = 0.0f,
			});
			engine.Step(input);
			PR_EXPECT(articulation.LinkVelocity(root).lin.x < -0.2f);

			// Keep the pair colliding but remove all current closing motion and positional correction.
			articulation.RootToWorld(m4x4::Identity());
			articulation.Sleep();
			body.O2W(m4x4::Translation(0.99f, 0.0f, 0.0f));
			body.VelocityWS(v4::Zero(), v4::Zero());
			engine.Step(input);
			PR_EXPECT(engine.LastCollisionStats().m_contact_count != 0);
			PR_EXPECT(articulation.Sleeping());
			PR_EXPECT(Length(articulation.LinkVelocity(root).lin) == 0.0f);
			PR_EXPECT(Length(body.VelocityWS().lin) < 1.0e-6f);
		}

		// Reused contact storage must ignore inactive tails when the GPU-generated prefix grows, shrinks, and becomes empty.
		PRUnitTestMethod(ContactPrefixGrowthAndShrinkIgnoresStaleTails, Quick)
		{
			auto shape = collision::ShapeSphere{0.5f};
			auto [articulation, root] = MakeContactFloatingRoot(shape, 0.0f);
			auto directions = std::array<v4, 4>{v4::XAxis(), v4::YAxis(), -v4::XAxis(), -v4::YAxis()};
			auto bodies = std::array{RigidBody{}, RigidBody{}, RigidBody{}, RigidBody{}};
			auto body_ptrs = std::array<RigidBody*, 4>{};
			for (int idx = 0; idx != isize(bodies); ++idx)
			{
				bodies[idx].Shape(collision::shape_cast(&shape), 1.0f);
				body_ptrs[idx] = &bodies[idx];
			}
			auto articulation_ptrs = std::array<Articulation*, 1>{&articulation};
			auto& engine = SharedEngine();
			ConfigureCoupledEngine(engine);
			auto config = engine.Config();
			config.max_collision_pairs = 64;
			config.push_out_iterations = 0;
			config.warm_start_scale = 0.0f;
			engine.Config(config);
			engine.Material(Material{
				.m_id = Material::DefaultID,
				.m_friction_static = 0.0f,
				.m_elasticity_norm = 0.0f,
			});

			// Keep endpoint identities stable; the sparse frame uses the last body rather than a prefix of body indices.
			for (auto active_mask : std::array<unsigned, 5>{0u, 15u, 8u, 0u, 15u})
			{
				articulation.RootToWorld(m4x4::Identity());
				articulation.Sleep();
				articulation.Wake();
				auto expected_momentum = v4::Zero();
				auto expected_contacts = 0;
				for (int idx = 0; idx != isize(bodies); ++idx)
				{
					auto const active = (active_mask & (1u << idx)) != 0u;
					auto const position = active ? 1.01f * directions[idx] : v4{10.0f + 2.0f * idx, 0.0f, 0.0f, 0.0f};
					auto const velocity = active ? -2.0f * directions[idx] : v4::Zero();
					bodies[idx].O2W(m4x4::Translation(position));
					bodies[idx].VelocityWS(v4::Zero(), velocity);
					expected_momentum += velocity;
					expected_contacts += active ? 1 : 0;
				}
				engine.Step(Engine::StepInput{
					.m_bodies = body_ptrs,
					.m_articulations = articulation_ptrs,
					.m_elapsed_seconds = 1.0f / 60.0f,
				});

				// Inactive endpoints must remain untouched, and every active impulse must retain its equal-and-opposite response.
				auto total_momentum = articulation.LinkVelocity(root).lin;
				for (int idx = 0; idx != isize(bodies); ++idx)
				{
					total_momentum += bodies[idx].MomentumWS().lin;
					if ((active_mask & (1u << idx)) == 0u)
						PR_EXPECT(Length(bodies[idx].VelocityWS().lin) < 1.0e-6f);

				}
				PR_EXPECT(engine.LastCollisionStats().m_contact_count == expected_contacts);
				PR_EXPECT(FEqlAbsolute(total_momentum, expected_momentum, 0.005f));
				PR_EXPECT(engine.LastStepProfile().m_submission_count == 1);
				PR_EXPECT(engine.LastStepProfile().m_wait_count == 1);
				PR_EXPECT(engine.LastStepProfile().m_readback_copy_count == 1);
			}
		}

		// Exercise rigid-contact correction through either the main solve or selective continuation.
		static void ExerciseRigidJointContactPosition(bool selective, int position_iterations = 1)
		{
			auto contact_shape = collision::ShapeSphere{0.5f};
			auto follower_shape = collision::ShapeSphere{0.1f};
			auto ground_shape = collision::ShapeBox{v4{10.0f, 10.0f, 1.0f, 0.0f}};
			auto contact_body = RigidBody{&contact_shape, m4x4::Translation(0.0f, 0.0f, 0.4f), Inertia::Sphere(contact_shape.m_radius, 1.0f)};
			auto follower_body = RigidBody{&follower_shape, m4x4::Translation(0.0f, 0.0f, 1.4f), Inertia::Sphere(follower_shape.m_radius, 1.0f)};
			auto ground = RigidBody{&ground_shape, m4x4::Translation(0.0f, 0.0f, -0.5f), Inertia::Infinite()};
			auto desc = CoupledEngineLinear(BodyRef::Rigid(contact_body), BodyRef::Rigid(follower_body), 2);
			desc.m_frame_b.m_constraint_to_body = m4x4::Translation(0.0f, 0.0f, -1.0f);
			auto constraints = ConstraintSet{};
			constraints.Add(desc);
			auto body_ptrs = std::array<RigidBody*, 3>{&contact_body, &follower_body, &ground};
			auto& engine = SharedEngine();
			ConfigureCoupledEngine(engine);
			auto config = engine.Config();
			config.max_collision_pairs = 16;
			config.push_out_iterations = selective ? 0 : position_iterations;
			config.position_baumgarte = 1.0f;
			config.velocity_baumgarte = 0.0f;
			config.deep_penetration_baumgarte_min = 0.0f;
			config.deep_penetration_baumgarte_max = 0.0f;
			config.warm_start_scale = 0.0f;
			config.selective_refresh_passes = selective ? 1 : 0;
			config.selective_refresh_position_iterations = position_iterations;
			config.selective_refresh_support_only = false;
			config.selective_refresh_depth_slop = 0.0f;
			engine.Config(config);

			// One sweep exposes a late clear; additional sweeps require root restoration. Repeated input exposes stale pseudo state.
			auto first_position = 0.0f;
			for (int frame = 0; frame != (selective ? 3 : 2); ++frame)
			{
				contact_body.O2W(m4x4::Translation(0.0f, 0.0f, 0.4f));
				follower_body.O2W(m4x4::Translation(0.0f, 0.0f, 1.4f));
				engine.Step(Engine::StepInput{
					.m_bodies = body_ptrs,
					.m_constraints = &constraints,
					.m_elapsed_seconds = 1.0f / 60.0f,
				});
				if (selective && frame == 0)
					continue;

				PR_EXPECT(engine.LastCollisionStats().m_contact_count != 0);
				PR_EXPECT(contact_body.O2W().pos.z > 0.41f);
				PR_EXPECT(follower_body.O2W().pos.z > 1.41f);
				PR_EXPECT(Abs(follower_body.O2W().pos.z - contact_body.O2W().pos.z - 1.0f) < 1.0e-5f);
				PR_EXPECT(Length(contact_body.VelocityWS().lin) < 1.0e-6f);
				PR_EXPECT(Length(follower_body.VelocityWS().lin) < 1.0e-6f);
				if (frame == (selective ? 1 : 0))
					first_position = contact_body.O2W().pos.z;
				else
					PR_EXPECT(Abs(contact_body.O2W().pos.z - first_position) < 1.0e-6f);

			}
		}

		// The first rigid-joint sweep must retain contact correction already written into the common pseudo stream.
		PRUnitTestMethod(RigidJointSharesInitialContactCorrection, Quick)
		{
			ExerciseRigidJointContactPosition(false);
		}

		// Selective-only rigid correction initializes and clears the same shared state independently of main-pass iteration settings.
		PRUnitTestMethod(SelectiveRigidJointSharesInitialContactCorrection, Quick)
		{
			ExerciseRigidJointContactPosition(true);
		}

		// Contact correction must reach both initially coincident joint anchors regardless of which endpoint touches the ground.
		static void ExerciseConnectedContactPosition(bool articulated_contact)
		{
			auto shape = collision::ShapeSphere{0.5f};
			auto builder = ArticulationBuilder{};
			auto link_desc = CoupledEngineLink();
			link_desc.m_shape = articulated_contact ? collision::shape_cast(&shape) : nullptr;
			auto const root = builder.AddFloatingRoot(link_desc, m4x4::Translation(0.0f, 0.0f, articulated_contact ? 0.4f : 1.4f));
			auto articulation = builder.Build();
			auto body_shape = collision::ShapeSphere{articulated_contact ? 0.1f : 0.5f};
			auto body = RigidBody{&body_shape, m4x4::Translation(0.0f, 0.0f, articulated_contact ? 1.4f : 0.4f), Inertia::Sphere(body_shape.m_radius, 1.0f)};
			auto ground_shape = collision::ShapeBox{v4{10.0f, 10.0f, 1.0f, 0.0f}};
			auto ground = RigidBody{&ground_shape, m4x4::Translation(0.0f, 0.0f, -0.5f), Inertia::Infinite()};
			auto desc = CoupledEngineLinear(BodyRef::Link(articulation, root), BodyRef::Rigid(body), 2);
			desc.m_frame_b.m_constraint_to_body = m4x4::Translation(0.0f, 0.0f, articulated_contact ? -1.0f : 1.0f);
			auto constraints = ConstraintSet{};
			constraints.Add(desc);
			auto body_ptrs = std::array<RigidBody*, 2>{&body, &ground};

			// Prepend an unrelated tree so shared forest indices differ from the coupled solver's compact indices.
			auto unrelated_builder = ArticulationBuilder{};
			auto const unrelated_root = unrelated_builder.AddFloatingRoot(CoupledEngineLink(), m4x4::Translation(20.0f, 0.0f, 10.0f));
			auto unrelated = unrelated_builder.Build();
			auto articulation_ptrs = std::array<Articulation*, 2>{&unrelated, &articulation};

			// Use enough sweeps to expose missing feedback, with physical penetration bias disabled.
			auto& engine = SharedEngine();
			ConfigureCoupledEngine(engine);
			auto config = engine.Config();
			config.max_collision_pairs = 16;
			config.push_out_iterations = 32;
			config.warm_start_scale = 0.0f;
			config.velocity_baumgarte = 0.0f;
			config.deep_penetration_baumgarte_min = 0.0f;
			config.deep_penetration_baumgarte_max = 0.0f;
			engine.Config(config);
			engine.Material(Material{
				.m_id = Material::DefaultID,
				.m_friction_static = 0.0f,
				.m_elasticity_norm = 0.0f,
			});
			engine.Step(Engine::StepInput{
				.m_bodies = body_ptrs,
				.m_articulations = articulation_ptrs,
				.m_constraints = &constraints,
				.m_elapsed_seconds = 1.0f / 60.0f,
			});

			// Both endpoints must move together, and coordinate correction must remain detached from physical velocity.
			auto const contact_height = articulated_contact ? articulation.RootToWorld().pos.z : body.O2W().pos.z;
			auto const expected_gap = articulated_contact ? 1.0f : -1.0f;
			PR_EXPECT(contact_height > 0.41f);
			PR_EXPECT(Abs(body.O2W().pos.z - articulation.RootToWorld().pos.z - expected_gap) < 0.0005f);
			PR_EXPECT(Length(articulation.LinkVelocity(root).lin) < 1.0e-5f);
			PR_EXPECT(Length(body.VelocityWS().lin) < 1.0e-5f);
			PR_EXPECT(Abs(unrelated.RootToWorld().pos.z - 10.0f) < 1.0e-5f);
			PR_EXPECT(Length(unrelated.LinkVelocity(unrelated_root).lin) < 1.0e-5f);
		}

		// Articulation contact push-out and coupled joint correction must share detached state.
		PRUnitTestMethod(ContactPushOutPreservesConnectedPosition, Quick)
		{
			ExerciseConnectedContactPosition(true);
		}

		// Ordinary rigid contacts must also feed the detached state read by their articulation-coupled joints.
		PRUnitTestMethod(RigidContactPushOutPreservesConnectedPosition, Quick)
		{
			ExerciseConnectedContactPosition(false);
		}
	};

	// Contact constants must be restored after other solver phases replace the active root signature.
	PRUnitTestClass(ResolveRootRestoreTests)
	{
		// Multiple main-pass sweeps restore shared_position_state and every resource after rigid joint solving.
		PRUnitTestMethod(RigidJointRestoresMainContactRoots, Extended)
		{
			TestClass_ConstraintReviewEngineTests::ExerciseRigidJointContactPosition(false, 4);
		}

		// Selective continuation owns a separate resolver and must apply the same root restoration rule.
		PRUnitTestMethod(RigidJointRestoresSelectiveContactRoots, Extended)
		{
			TestClass_ConstraintReviewEngineTests::ExerciseRigidJointContactPosition(true, 4);
		}

		// Coupled and rigid contact phases share pseudo state but use different root layouts and forest indices.
		PRUnitTestMethod(CoupledJointRestoresSharedPositionRoots, Extended)
		{
			TestClass_ConstraintReviewEngineTests::ExerciseConnectedContactPosition(false);
			TestClass_ConstraintReviewEngineTests::ExerciseConnectedContactPosition(true);
		}
	};

	// A rigid-only maintenance frame must not consume another frame's optional diagnostic streams.
	PRUnitTestClass(ConstraintReviewLifecycleTests)
	{
		// Keep each failed-recording witness on a separate engine so its command list cannot affect another test.
		static void ExerciseSleepIslandRefresh(bool coupled)
		{
			auto engine = Engine{{}, &SharedTestShaderCache()};
			ConfigureCoupledEngine(engine);
			auto config = engine.Config();
			config.max_collision_pairs = 16;
			engine.Config(config);
			auto shape = collision::ShapeSphere{0.2f};
			auto body = RigidBody{&shape, m4x4::Identity(), Inertia::Sphere(shape.m_radius, 1.0f)};
			auto [articulation, root] = MakeEngineFloatingRoot(0.0f);
			auto desc = CoupledEngineLinear(BodyRef::World(), coupled ? BodyRef::Link(articulation, root) : BodyRef::Rigid(body));
			desc.m_break_force = coupled ? std::numeric_limits<float>::infinity() : 100.0f;
			auto constraints = ConstraintSet{};
			constraints.Add(desc);
			auto body_ptrs = std::array<RigidBody*, 1>{&body};
			auto articulation_ptrs = std::array<Articulation*, 1>{&articulation};
			auto input = Engine::StepInput{
				.m_bodies = body_ptrs,
				.m_constraints = &constraints,
				.m_elapsed_seconds = 1.0f / 60.0f,
			};
			if (coupled)
				input.m_articulations = articulation_ptrs;

			// The preceding frame leaves a nonempty break or coupled-failure output even though nothing failed.
			engine.Step(input);
			engine.UpdateSleepIslands(body_ptrs);
			PR_EXPECT(engine.LastStepProfile().m_submission_count == 1);
			PR_EXPECT(engine.LastStepProfile().m_readback_copy_count == 1);

			// Maintenance must also leave the retained constraint resources usable by the next simulation frame.
			engine.Step(input);
			PR_EXPECT(engine.LastStepProfile().m_submission_count == 1);
			PR_EXPECT(engine.LastStepProfile().m_readback_copy_count == 1);
		}

		// Break-state storage must not leak into a subsequent rigid-only sleep-island update.
		PRUnitTestMethod(SleepIslandRefreshAfterBreakableFrame, Quick)
		{
			ExerciseSleepIslandRefresh(false);
		}

		// Coupled-failure storage must not leak into a subsequent rigid-only sleep-island update.
		PRUnitTestMethod(SleepIslandRefreshAfterCoupledFrame, Quick)
		{
			ExerciseSleepIslandRefresh(true);
		}
	};
	// Exercise accepted-epoch and membership lifetime through completed public Engine frames.
	PRUnitTestClass(EngineMigrationLifetimeTests)
	{

		// Historical loads are not completed-substep samples; exact thresholds and repaired stable identities retain their existing semantics.
		PRUnitTestMethod(SkippedHistoricalLoadsAndStrictRepairThresholds, Extended)
		{
			auto shape = collision::ShapeSphere{0.2f};
			auto body = RigidBody{&shape, m4x4::Identity(), Inertia::Sphere(0.2f, 1.0f)};
			auto [articulation, root] = MakeEngineFloatingRoot(10.0f);
			auto desc = CoupledEngineLinear(BodyRef::World(), BodyRef::Rigid(body));
			desc.D6().m_linear[0].m_mode = EConstraintAxisMode::Driven;
			desc.D6().m_linear[0].m_target_velocity = 1.0f;
			desc.m_break_force = 16.0f;
			auto linked = CoupledEngineLinear(BodyRef::World(), BodyRef::Link(articulation, root));
			linked.D6().m_linear[0] = desc.D6().m_linear[0];
			linked.m_break_force = desc.m_break_force;
			auto constraints = ConstraintSet{};
			auto const ordinary_handle = constraints.Add(desc);
			auto const coupled_handle = constraints.Add(linked);
			auto bodies = std::array{&body};
			auto forest = std::array{&articulation};
			auto input = Engine::StepInput{.m_bodies = bodies, .m_articulations = forest, .m_constraints = &constraints, .m_elapsed_seconds = 1.0f / 16.0f};
			auto& engine = SharedEngine();
			ConfigureCoupledEngine(engine);
			auto config = engine.Config();
			config.solver_iterations = 1;
			config.push_out_iterations = 0;
			config.constraint_relaxation = 0.5f;
			config.constraint_coupled_relaxation = 0.5f;
			config.constraint_warm_start_factor = 1.0f;
			engine.Config(config);
			auto events = std::vector<ConstraintBreakEvent>{};
			engine.ConstraintsBroken += [&](Engine&, std::span<ConstraintBreakEvent const> values)
			{
				events.insert(events.end(), values.begin(), values.end());
			};
			engine.Step(input);
			PR_EXPECT(events.empty());

			// The old .5 impulse implies load8, but no acceptance in this new zero-sweep epoch may sample it against threshold4.
			desc.m_break_force = linked.m_break_force = 4.0f;
			constraints.Update(ordinary_handle, desc);
			constraints.Update(coupled_handle, linked);
			config.solver_iterations = 0;
			engine.Config(config);
			engine.Step(input);
			PR_EXPECT(events.empty());
			PR_EXPECT(!constraints.IsBroken(ordinary_handle) && !constraints.IsBroken(coupled_handle));
			PR_EXPECT(FEqlAbsolute(body.VelocityWS().lin.x, 0.5f, 1.0e-6f));
			PR_EXPECT(FEqlAbsolute(articulation.LinkVelocity(root).lin.x, 0.5f, 1.0e-6f));

			// Binary-exact h makes the next cold .25/h load exactly4: equality must not break.
			config.solver_iterations = 1;
			engine.Config(config);
			engine.Step(input);
			PR_EXPECT(events.empty());
			PR_EXPECT(FEqlAbsolute(body.VelocityWS().lin.x, 0.75f, 1.0e-6f));
			PR_EXPECT(FEqlAbsolute(articulation.LinkVelocity(root).lin.x, 0.75f, 1.0e-6f));
			desc.m_break_force = linked.m_break_force = 1.0f;
			constraints.Update(ordinary_handle, desc);
			constraints.Update(coupled_handle, linked);
			engine.Step(input);
			PR_EXPECT(events.size() == 2);
			PR_EXPECT(constraints.IsBroken(ordinary_handle) && constraints.IsBroken(coupled_handle));
			for (auto const& value : events)
			{
				PR_EXPECT(value.m_constraint == ordinary_handle || value.m_constraint == coupled_handle);
				PR_EXPECT(value.m_force == 2.0f);
				PR_EXPECT(value.m_substep_index == 0);
			}

			// Repair keeps the public generation, clears the disposition, and cold-starts the next exact equality load1.
			events.clear();
			constraints.Repair(ordinary_handle);
			constraints.Repair(coupled_handle);
			engine.Step(input);
			PR_EXPECT(events.empty());
			PR_EXPECT(!constraints.IsBroken(ordinary_handle) && !constraints.IsBroken(coupled_handle));
			PR_EXPECT(FEqlAbsolute(body.VelocityWS().lin.x, 0.9375f, 1.0e-6f));
			PR_EXPECT(FEqlAbsolute(articulation.LinkVelocity(root).lin.x, 0.9375f, 1.0e-6f));
		}

		// Fresh-frame repacking must preserve physical history and never redirect a retained response to an unrelated body or tree.
		PRUnitTestMethod(RepackedOwnersKeepTheirPhysicalIdentities, Extended)
		{
			auto shape = collision::ShapeSphere{0.2f};
			auto body = RigidBody{&shape, m4x4::Identity(), Inertia::Sphere(0.2f, 1.0f)};
			auto decoy = RigidBody{&shape, m4x4::Translation(100, 0, 0), Inertia::Sphere(0.2f, 1.0f)};
			decoy.VelocityWS(v8motion{v4::Zero(), 2.0f * v4::XAxis()});
			auto [articulation, root] = MakeEngineFloatingRoot(10.0f);
			auto builder = ArticulationBuilder{};
			auto const other_root = builder.AddFloatingRoot(CoupledEngineLink(), m4x4::Translation(200, 0, 0), v8motion{v4::Zero(), 3.0f * v4::XAxis()});
			auto other = builder.Build();
			auto desc = CoupledEngineLinear(BodyRef::World(), BodyRef::Rigid(body));
			desc.D6().m_linear[0].m_mode = EConstraintAxisMode::Driven;
			desc.D6().m_linear[0].m_target_velocity = 1.0f;
			auto linked = CoupledEngineLinear(BodyRef::World(), BodyRef::Link(articulation, root));
			linked.D6().m_linear[0] = desc.D6().m_linear[0];
			auto constraints = ConstraintSet{};
			constraints.Add(desc);
			constraints.Add(linked);
			auto first_bodies = std::array{&body};
			auto first_forest = std::array{&articulation};
			auto& engine = SharedEngine();
			ConfigureCoupledEngine(engine);
			auto config = engine.Config();
			config.solver_iterations = 1;
			config.push_out_iterations = 0;
			config.constraint_relaxation = 0.5f;
			config.constraint_coupled_relaxation = 0.5f;
			config.constraint_warm_start_factor = 1.0f;
			engine.Config(config);
			engine.Step(Engine::StepInput{.m_bodies = first_bodies, .m_articulations = first_forest, .m_constraints = &constraints, .m_elapsed_seconds = 1.0f / 60.0f});
			PR_EXPECT(FEqlAbsolute(body.VelocityWS().lin.x, 0.5f, 2.0e-5f));
			PR_EXPECT(FEqlAbsolute(articulation.LinkVelocity(root).lin.x, 0.5f, 2.0e-5f));

			// Ordinary identity receipts survive repacking; the coupled packed-response membership changes and deliberately starts cold.
			auto repacked_bodies = std::array{&decoy, &body};
			auto repacked_forest = std::array{&other, &articulation};
			engine.Step(Engine::StepInput{.m_bodies = repacked_bodies, .m_articulations = repacked_forest, .m_constraints = &constraints, .m_elapsed_seconds = 1.0f / 60.0f});
			printf("repacked owners: ordinary=%.9g coupled=%.9g unrelated-body=%.9g unrelated-root=%.9g\n",
				body.VelocityWS().lin.x, articulation.LinkVelocity(root).lin.x, decoy.VelocityWS().lin.x, other.LinkVelocity(other_root).lin.x);
			PR_EXPECT(FEqlAbsolute(body.VelocityWS().lin.x, 1.0f, 2.0e-5f));
			PR_EXPECT(FEqlAbsolute(articulation.LinkVelocity(root).lin.x, 0.75f, 2.0e-5f));
			PR_EXPECT(FEqlAbsolute(decoy.VelocityWS().lin.x, 2.0f, 2.0e-5f));
			PR_EXPECT(FEqlAbsolute(other.LinkVelocity(other_root).lin.x, 3.0f, 2.0e-5f));
			PR_EXPECT(!engine.LastFeatureStats().m_failure.HasTerminalFailure());
		}


		// Skipped persistent work retains accepted dt; disabling, removal and slot reuse must invalidate future guesses, not physical history.
		PRUnitTestMethod(SkippedWorkAndMembershipPreservePhysicalHistory, Extended)
		{
			auto [articulation, root] = MakeEngineFloatingRoot(10.0f);
			auto shape = collision::ShapeSphere{0.2f};
			auto body = RigidBody{&shape, m4x4::Identity(), Inertia::Sphere(shape.m_radius, 1.0f)};
			auto rigid_desc = CoupledEngineLinear(BodyRef::World(), BodyRef::Rigid(body));
			rigid_desc.D6().m_linear[0].m_mode = EConstraintAxisMode::Driven;
			rigid_desc.D6().m_linear[0].m_target_velocity = 1.0f;
			auto coupled_desc = CoupledEngineLinear(BodyRef::World(), BodyRef::Link(articulation, root));
			coupled_desc.D6().m_linear[0] = rigid_desc.D6().m_linear[0];
			auto constraints = ConstraintSet{};
			auto rigid_handle = constraints.Add(rigid_desc);
			auto coupled_handle = constraints.Add(coupled_desc);
			auto bodies = std::array{&body};
			auto forest = std::array{&articulation};
			auto input = Engine::StepInput{.m_bodies = bodies, .m_articulations = forest, .m_constraints = &constraints, .m_elapsed_seconds = 1.0f / 60.0f};
			auto& engine = SharedEngine();
			ConfigureCoupledEngine(engine);
			auto config = engine.Config();
			config.solver_iterations = 1;
			config.push_out_iterations = 0;
			config.constraint_relaxation = 0.5f;
			config.constraint_coupled_relaxation = 0.5f;
			config.constraint_warm_start_factor = 1.0f;
			engine.Config(config);

			// Both owners have unit translational response and no angular motion, so the root's local axes stay aligned with world space.
			auto CheckSpeed = [&](float expected)
			{
				PR_EXPECT(FEqlAbsolute(body.VelocityWS().lin.x, expected, 3.0e-5f));
				PR_EXPECT(FEqlAbsolute(articulation.LinkVelocity(root).lin.x, expected, 3.0e-5f));
				PR_EXPECT(!engine.LastFeatureStats().m_failure.HasTerminalFailure());
				printf("Engine owner lifetime: expected=%.9g rigid=%.9g coupled=%.9g\n", expected, body.VelocityWS().lin.x, articulation.LinkVelocity(root).lin.x);
			};
			engine.Step(input);
			CheckSpeed(0.5f);

			// This longer zero-sweep frame must neither publish a warm seed nor replace the accepted h with 2h.
			config.solver_iterations = 0;
			config.constraint_warm_start_factor = 2.0e38f;
			engine.Config(config);
			input.m_elapsed_seconds *= 2.0f;
			engine.Step(input);
			CheckSpeed(0.5f);
			config.solver_iterations = 1;
			config.constraint_warm_start_factor = 1.0f;
			engine.Config(config);
			input.m_elapsed_seconds *= 0.5f;
			engine.Step(input);
			CheckSpeed(1.0f);

			// Re-enabling is cold: stale .5 support would instead produce 1.25 under the same half-relaxed solve.
			constraints.SetEnabled(rigid_handle, false);
			constraints.SetEnabled(coupled_handle, false);
			engine.Step(input);
			CheckSpeed(1.0f);
			constraints.SetEnabled(rigid_handle, true);
			constraints.SetEnabled(coupled_handle, true);
			engine.Step(input);
			CheckSpeed(1.0f);

			// Create nonzero accepted coefficients again before destroying and reusing both public slots.
			rigid_desc.D6().m_linear[0].m_target_velocity = 2.0f;
			coupled_desc.D6().m_linear[0].m_target_velocity = 2.0f;
			constraints.Update(rigid_handle, rigid_desc);
			constraints.Update(coupled_handle, coupled_desc);
			engine.Step(input);
			CheckSpeed(1.5f);
			constraints.Remove(rigid_handle);
			constraints.Remove(coupled_handle);
			engine.Step(input);
			CheckSpeed(1.5f);
			auto const replacement_rigid = constraints.Add(rigid_desc);
			auto const replacement_coupled = constraints.Add(coupled_desc);
			PR_EXPECT(replacement_rigid != rigid_handle && replacement_coupled != coupled_handle);
			engine.Step(input);
			CheckSpeed(1.75f);
		}

		// An overloaded constituent becomes inactive without invalidating the uploaded island's surviving row in the next substep.
		PRUnitTestMethod(BrokenConstituentLeavesSurvivingIslandScheduled, Extended)
		{
			auto [articulation, root] = MakeEngineFloatingRoot(0.0f);
			auto desc_x = CoupledEngineLinear(BodyRef::World(), BodyRef::Link(articulation, root));
			desc_x.D6().m_linear[0].m_mode = EConstraintAxisMode::Driven;
			desc_x.D6().m_linear[0].m_target_velocity = 1.0f;
			desc_x.m_break_force = 24.0f;
			auto desc_y = CoupledEngineLinear(BodyRef::World(), BodyRef::Link(articulation, root), 1);
			desc_y.D6().m_linear[1].m_mode = EConstraintAxisMode::Driven;
			desc_y.D6().m_linear[1].m_target_velocity = 2.0f;
			auto constraints = ConstraintSet{};
			auto const x = constraints.Add(desc_x);
			auto const y = constraints.Add(desc_y);
			auto forest = std::array{&articulation};
			auto& engine = SharedEngine();
			ConfigureCoupledEngine(engine);
			auto config = engine.Config();
			config.solver_iterations = 1;
			config.push_out_iterations = 0;
			config.constraint_coupled_relaxation = 0.5f;
			config.constraint_warm_start_factor = 1.0f;
			engine.Config(config);
			auto broken = std::vector<ConstraintBreakEvent>{};
			engine.ConstraintsBroken += [&](Engine&, std::span<ConstraintBreakEvent const> events)
			{
				broken.insert(broken.end(), events.begin(), events.end());
			};
			engine.Step(Engine::StepInput{.m_articulations = forest, .m_constraints = &constraints, .m_elapsed_seconds = 1.0f / 30.0f, .m_substep_count = 2});

			// First accepted load is (.5,1)/h. Only X breaks; Y's next accepted total reaches speed2.
			auto const velocity = articulation.LinkVelocity(root).lin;
			printf("broken island constituent: vx=%.9g vy=%.9g events=%zu\n", velocity.x, velocity.y, broken.size());
			PR_EXPECT(constraints.IsBroken(x));
			PR_EXPECT(!constraints.IsBroken(y));
			PR_EXPECT(FEqlAbsolute(velocity.x, 0.5f, 3.0e-5f));
			PR_EXPECT(FEqlAbsolute(velocity.y, 2.0f, 3.0e-5f));
			PR_EXPECT(broken.size() == 1);
			if (!broken.empty())
			{
				PR_EXPECT(broken[0].m_constraint == x);
				PR_EXPECT(broken[0].m_substep_index == 0);
				PR_EXPECT(FEqlAbsolute(broken[0].m_force, 30.0f, 0.002f));
			}
		}
	};

	// Selective continuation must preserve earlier reports and replace, rather than peak-sample, transient accepted loads.
	PRUnitTestClass(EngineMigrationSelectiveTests)
	{
		// Changed unilateral membership distinguishes accepted replacement, rejected replacement and completed-substep load eligibility.
		PRUnitTestMethod(SelectivePublicationPreservesReportsAndSamplesFinalAcceptance, Extended)
		{
			for (auto const relaxation : {0.5f, 0.9f, 0.3f})
			{
				// An independent rigid overlap opens the real selective gate without adding contact impulses to the unshaped articulation.
				auto sphere = collision::ShapeSphere{0.5f};
				auto box = collision::ShapeBox{v4{5, 5, 0.5f, 0}};
				auto body = RigidBody{&sphere, m4x4::Translation(0, 0, 0.15f), Inertia::Sphere(0.5f, 1.0f)};
				auto ground = RigidBody{&box, m4x4::Translation(0, 0, -0.5f), Inertia::Infinite()};
				auto bodies = std::array{&body, &ground};
				auto& engine = SharedEngine();
				ConfigureCoupledEngine(engine);
				auto config = engine.Config();
				config.solver_iterations = 1;
				config.push_out_iterations = 0;
				config.velocity_baumgarte = 0.0f;
				config.deep_penetration_baumgarte_min = 0.0f;
				config.deep_penetration_baumgarte_max = 0.0f;
				config.warm_start_scale = 0.0f;
				config.constraint_coupled_relaxation = relaxation;
				config.constraint_coupled_backtrack_limit = 0;
				config.constraint_position_beta = 1.0f;
				config.constraint_position_relaxation = 0.5f;
				config.constraint_max_position_speed = 100.0f;
				config.selective_refresh_passes = 1;
				config.selective_refresh_solver_iterations = 1;
				config.selective_refresh_adaptive_body_limit = 0;
				config.selective_refresh_position_iterations = 1;
				config.selective_refresh_support_only = false;
				config.selective_refresh_depth_slop = 0.0f;
				config.selective_refresh_support_depth_slop = 0.0f;
				engine.Config(config);
				engine.Material(Material{.m_id = Material::DefaultID, .m_friction_static = 0.0f, .m_elasticity_norm = 0.0f});
				engine.Step(1.0f / 60.0f, bodies);
				PR_EXPECT(engine.LastCollisionStats().m_contact_count != 0);

				// Main position work crosses both upper limits. The third case activates ten opposite limits to reject the later replacement.
				auto const reject_selective = relaxation == 0.3f;
				auto builder = ArticulationBuilder{};
				auto const root = builder.AddFloatingRoot(CoupledEngineLink(), m4x4::Translation(0.1f, 0, 0), v8motion{v4::Zero(), v4::XAxis()});
				auto articulation = builder.Build();
				auto desc = CoupledEngineLinear(BodyRef::World(), BodyRef::Link(articulation, root));
				auto limited = desc;
				limited.D6().m_linear[0].m_mode = EConstraintAxisMode::Limited;
				limited.D6().m_linear[0].m_limits = {-100.0f, 0.0f};
				limited.m_break_force = reject_selective ? 12.0f : 20.0f;
				auto constraints = ConstraintSet{};
				auto const first = constraints.Add(limited);
				auto const second = constraints.Add(limited);
				for (int index = 0; index != (reject_selective ? 4 : 1); ++index)
					constraints.Add(desc);

				if (reject_selective)
				{
					auto lower = limited;
					lower.m_break_force = std::numeric_limits<float>::infinity();
					lower.D6().m_linear[0].m_limits = {0.0f, 100.0f};
					for (int index = 0; index != 10; ++index)
						constraints.Add(lower);
				}
				auto forest = std::array{&articulation};
				auto rejections = std::vector<ConstraintRejectionEvent>{};
				auto breaks = std::vector<ConstraintBreakEvent>{};
				engine.ConstraintRejections += [&](Engine&, std::span<ConstraintRejectionEvent const> events)
				{
					rejections.insert(rejections.end(), events.begin(), events.end());
				};
				engine.ConstraintsBroken += [&](Engine&, std::span<ConstraintBreakEvent const> events)
				{
					breaks.insert(breaks.end(), events.begin(), events.end());
				};
				config.push_out_iterations = 1;
				engine.Config(config);
				engine.Step(Engine::StepInput{.m_bodies = bodies, .m_articulations = forest, .m_constraints = &constraints, .m_elapsed_seconds = 1.0f / 60.0f});

				// The first two cases accept -.75 from a compatible -.5 guess or -.9 cold. The third must preserve its main accepted total -1.8.
				auto const accepted_main = relaxation == 0.5f;
				auto const expected_speed = reject_selective ? -0.8f : accepted_main ? 0.25f : 0.1f;
				auto const speed = articulation.LinkVelocity(root).lin.x;
				printf("selective ownership: relaxation=%.9g speed=%.9g x=%.9g passes=%d rejections=%zu breaks=%zu\n",
					relaxation, speed, articulation.RootToWorld().pos.x, engine.LastStepProfile().m_selective_refresh_pass_count, rejections.size(), breaks.size());
				PR_EXPECT(engine.LastStepProfile().m_selective_refresh_pass_count == 1);
				PR_EXPECT(articulation.RootToWorld().pos.x < -0.01f);
				PR_EXPECT(FEqlAbsolute(speed, expected_speed, 3.0e-5f));
				PR_EXPECT(constraints.IsBroken(first) == reject_selective);
				PR_EXPECT(constraints.IsBroken(second) == reject_selective);
				PR_EXPECT(breaks.size() == (reject_selective ? 2u : 0u));
				for (auto const& value : breaks)
				{
					// Accepted upper-limit impulses remain .3/h = 18 despite their now-inactive prepared geometry.
					PR_EXPECT(value.m_constraint == first || value.m_constraint == second);
					PR_EXPECT(value.m_substep_index == 0);
					PR_EXPECT(FEqlAbsolute(value.m_force, 18.0f, 0.002f));
				}
				PR_EXPECT(!engine.LastFeatureStats().m_failure.HasTerminalFailure());
				PR_EXPECT(engine.LastStepProfile().m_readback_copy_count == 1);
				PR_EXPECT(rejections.size() == (accepted_main ? 0u : 1u));
				if (!accepted_main && !rejections.empty())
				{
					// The report survives later scratch reuse. In the last case seed -.2 plus proposed delta .84 gives merit .1848.
					PR_EXPECT(rejections[0].m_owner == EConstraintRejectionOwner::CoupledIsland);
					PR_EXPECT(rejections[0].m_index == 0 && rejections[0].m_generation == 0);
					PR_EXPECT(rejections[0].m_phase == EConstraintRejectionPhase::Velocity);
					PR_EXPECT(rejections[0].m_substep_index == 0);
					PR_EXPECT(rejections[0].m_outcome == EConstraintRejectionOutcome::SolveRejected);
					PR_EXPECT(rejections[0].m_failure_flags == GpuCoupledConstraintFailure_Merit);
					PR_EXPECT(rejections[0].m_iteration_count == 1);
					PR_EXPECT(FEqlAbsolute(rejections[0].m_relaxation, relaxation, 1.0e-6f));
					PR_EXPECT(FEqlAbsolute(rejections[0].m_merit_change, reject_selective ? 0.1848f : 0.945f, 2.0e-5f));
				}
			}
		}
	};

	// Focused controls for the coordinated Engine ownership and public-boundary migration.
	PRUnitTestClass(EngineMigrationReportingTests)
	{
		// A zero-mobility coupled island reports bounded rejection without aborting publication or adding a readback.
		PRUnitTestMethod(CoupledNonConvergenceUsesFrameReadback, Extended)
		{
			auto builder = ArticulationBuilder{};
			auto const root = builder.AddFixedRoot(CoupledEngineLink());
			auto articulation = builder.Build();
			auto constraints = ConstraintSet{};
			constraints.Add(CoupledEngineLinear(BodyRef::Link(articulation, root), BodyRef::World()));
			auto articulation_ptrs = std::array<Articulation*, 1>{&articulation};
			auto failures = std::vector<ConstraintRejectionEvent>{};
			auto& engine = SharedEngine();
			ConfigureCoupledEngine(engine);
			engine.ConstraintRejections += [&](Engine&, std::span<ConstraintRejectionEvent const> events)
			{
				failures.insert(failures.end(), events.begin(), events.end());
			};

			engine.Step(Engine::StepInput{
				.m_articulations = articulation_ptrs,
				.m_constraints = &constraints,
				.m_elapsed_seconds = 1.0f / 60.0f,
				.m_substep_count = 2,
			});

			PR_EXPECT(failures.size() == 1);
			if (!failures.empty())
			{
				PR_EXPECT(failures[0].m_substep_index == 0);
				PR_EXPECT(failures[0].m_owner == EConstraintRejectionOwner::CoupledIsland);
				PR_EXPECT(failures[0].m_index == 0 && failures[0].m_generation == 0);
				PR_EXPECT(failures[0].m_outcome == EConstraintRejectionOutcome::SolveRejected);
				PR_EXPECT(failures[0].m_iteration_count == 0);
				PR_EXPECT(failures[0].m_failure_flags != 0);
			}
			auto const& stats = engine.LastFeatureStats();
			PR_EXPECT(!stats.m_failure.HasTerminalFailure());
			PR_EXPECT(stats.m_frame_output.m_constraint_rejection_slot_count == 2);
			PR_EXPECT(stats.m_frame_output.m_readback_count == 1);
			PR_EXPECT(engine.LastStepProfile().m_readback_copy_count == 1);
		}

		// Coupled soft-row arithmetic uses the same validity gate and reports rejected island work through the existing frame output.
		PRUnitTestMethod(CoupledNumericalFailureUsesFrameDiagnostics, Quick)
		{
			auto [articulation, root] = MakeEngineFloatingRoot(1.0f);
			auto desc = CoupledEngineLinear(BodyRef::World(), BodyRef::Link(articulation, root));
			desc.D6().m_linear[0].m_stiffness = 1.0e-36f;
			auto constraints = ConstraintSet{};
			constraints.Add(desc);
			auto articulation_ptrs = std::array<Articulation*, 1>{&articulation};
			auto& engine = SharedEngine();
			ConfigureCoupledEngine(engine);
			auto rejections = std::vector<ConstraintRejectionEvent>{};
			engine.ConstraintRejections += [&](Engine&, std::span<ConstraintRejectionEvent const> events)
			{
				rejections.insert(rejections.end(), events.begin(), events.end());
			};
			engine.Step(Engine::StepInput{
				.m_articulations = articulation_ptrs,
				.m_constraints = &constraints,
				.m_elapsed_seconds = 1.0f / 60.0f,
			});
			PR_EXPECT(!engine.LastFeatureStats().m_failure.HasTerminalFailure());
			PR_EXPECT(rejections.size() == 1);
			if (!rejections.empty())
			{
				PR_EXPECT(rejections[0].m_outcome == EConstraintRejectionOutcome::SolveRejected);
				PR_EXPECT(AllSet(rejections[0].m_failure_flags, GpuCoupledConstraintFailure_NonFinite));
			}
			PR_EXPECT(FEqlAbsolute(articulation.RootToWorld().pos.x, 1.0f, 1.0e-7f));
			PR_EXPECT(engine.LastStepProfile().m_submission_count == 1);
			PR_EXPECT(engine.LastStepProfile().m_wait_count == 1);
			PR_EXPECT(engine.LastStepProfile().m_readback_copy_count == 1);
		}
	};

	// Exercise actual published passive energy, independently of the local scalar merit diagnostic.
	PRUnitTestClass(PassiveMeritScaleTests)
	{
		// Configure one physical sweep without contact, positional, selective, force or drive work.
		static void ConfigurePassive(Engine& engine, float warm_factor)
		{
			ConfigureCoupledEngine(engine);
			auto config = engine.Config();
			config.solver_iterations = 1;
			config.push_out_iterations = 0;
			config.selective_refresh_passes = 0;
			config.velocity_baumgarte = 0.0f;
			config.position_baumgarte = 0.0f;
			config.constraint_warm_start_factor = warm_factor;
			config.warm_start_scale = 0.0f;
			config.constraint_coupled_relaxation = 0.9f;
			config.constraint_coupled_backtrack_limit = 4;
			engine.Config(config);
		}

		// Evaluate this unit-mass sphere fixture's actual kinetic energy using double intermediates.
		static double Energy(v8motion velocity)
		{
			auto const linear_squared = double(velocity.lin.x) * velocity.lin.x + double(velocity.lin.y) * velocity.lin.y + double(velocity.lin.z) * velocity.lin.z;
			auto const angular_squared = double(velocity.ang.x) * velocity.ang.x + double(velocity.ang.y) * velocity.ang.y + double(velocity.ang.z) * velocity.ang.z;
			return 0.5 * linear_squared + 0.5 * (2.0 / 5.0) * 0.2 * 0.2 * angular_squared;
		}

		// Passive duplicate rows must dissipate at contrasting speeds and active row counts with unchanged retry budgets.
		PRUnitTestMethod(ThreePassiveLocksAtContrastingSpeeds, Extended)
		{
			// N identical rows yield v'=(1-N*alpha)v; the first decreasing candidates use alpha .9, .45 and .225 respectively.
			for (auto const [row_count, expected_speed_ratio] : {std::pair{1, 0.1}, std::pair{3, -0.35}, std::pair{7, -0.575}})
			for (auto const speed : {1.0f, 1.0e-3f, 1.0e-6f})
			{
				auto builder = ArticulationBuilder{};
				auto const root = builder.AddFloatingRoot(CoupledEngineLink(), m4x4::Identity(), v8motion{v4::Zero(), speed * v4::XAxis()});
				auto articulation = builder.Build();
				articulation.GravityWS(root, v4::Zero());
				auto constraints = ConstraintSet{};
				for (int index = 0; index != row_count; ++index)
					constraints.Add(CoupledEngineLinear(BodyRef::World(), BodyRef::Link(articulation, root)));

				// Only one physical sweep runs; no force, contact, position, warm seed or selective visit can supply energy.
				auto& engine = SharedEngine();
				ConfigurePassive(engine, 0.0f);
				auto rejection_count = 0;
				engine.ConstraintRejections += [&](Engine&, std::span<ConstraintRejectionEvent const> events)
				{
					rejection_count += isize(events);
				};
				auto forest = std::array{&articulation};
				auto const before = articulation.RootVelocity();
				engine.Step(Engine::StepInput{.m_articulations = forest, .m_constraints = &constraints, .m_elapsed_seconds = 1.0f / 60.0f, .m_substep_count = 1});
				auto const after = articulation.RootVelocity();

				// Independent double-precision energy uses the unit mass and solid-sphere inertia, not the solver merit or Engine energy routine.
				auto const energy_before = Energy(before);
				auto const energy_after = Energy(after);
				auto const energy_ratio = energy_after / energy_before;
				auto const speed_ratio = double(after.lin.x) / before.lin.x;
				auto const first_impulse = -double(row_count) * 0.9 * double(before.lin.x);
				auto const first_work = first_impulse * double(before.lin.x) + 0.5 * first_impulse * first_impulse;
				auto const old_tolerance = 64.0 * std::numeric_limits<float>::epsilon() * (1.0 + std::abs(first_work));
				printf("passive-scale: rows=%d speed=%.17g before_x=%.17g after_x=%.17g speed_ratio=%.17g energy_before=%.17g energy_after=%.17g energy_change=%.17g energy_ratio=%.17g first_work=%.17g old_tolerance=%.17g engine_energy=%.17g rejections=%d terminal=%u contacts=%d\n",
					row_count, double(speed), double(before.lin.x), double(after.lin.x), speed_ratio, energy_before, energy_after, energy_after - energy_before, energy_ratio, first_work, old_tolerance, double(articulation.KineticEnergy()), rejection_count, static_cast<unsigned>(engine.LastFeatureStats().m_failure.m_reason), engine.LastCollisionStats().m_contact_count);

				// These actual-state expectations fail under the previous absolute energy allowance at both small scales.
				PR_EXPECT(energy_after < energy_before);
				PR_EXPECT(std::abs(speed_ratio - expected_speed_ratio) < 2.0e-5);
				PR_EXPECT(std::abs(energy_ratio - expected_speed_ratio * expected_speed_ratio) < 8.0e-5);
				PR_EXPECT(std::abs(double(articulation.KineticEnergy()) - energy_after) < 2.0e-6 * energy_before);
				PR_EXPECT(All(after.ang == v4::Zero()));
				PR_EXPECT(after.lin.y == 0.0f && after.lin.z == 0.0f);
				PR_EXPECT(rejection_count == 0);
				PR_EXPECT(!engine.LastFeatureStats().m_failure.HasTerminalFailure());
				PR_EXPECT(engine.LastCollisionStats().m_contact_count == 0);
				PR_EXPECT(engine.LastFeatureStats().m_coupled.m_constraint_count == row_count);
				PR_EXPECT(engine.LastFeatureStats().m_coupled.m_contact_capacity == 0);
				PR_EXPECT(engine.LastStepProfile().m_readback_copy_count == 1);
			}
		}

		// Track 600 consecutive published states without resetting the engine, state or owner between visits.
		PRUnitTestMethod(RepeatedPassiveLocksColdAndWarm, Extended)
		{
			// Run every requested scale/ownership case before asserting, so a surviving energy-growth witness remains complete.
			auto all_valid = true;
			auto all_bounded = true;
			for (auto const warm_factor : {0.0f, 1.0f})
			for (auto const speed : {1.0f, 1.0e-3f, 1.0e-6f, std::ldexp(1.0f, -100)})
			{
				// The additional FTZ-range case keeps the reviewed full-warm, half-relaxed three-row recurrence.
				auto const ftz_case = speed == std::ldexp(1.0f, -100);
				if (ftz_case && warm_factor == 0.0f)
					continue;

				auto builder = ArticulationBuilder{};
				auto const root = builder.AddFloatingRoot(CoupledEngineLink(), m4x4::Identity(), v8motion{v4::Zero(), speed * v4::XAxis()});
				auto articulation = builder.Build();
				articulation.GravityWS(root, v4::Zero());
				auto constraints = ConstraintSet{};
				for (int index = 0; index != 3; ++index)
					constraints.Add(CoupledEngineLinear(BodyRef::World(), BodyRef::Link(articulation, root)));

				auto& engine = SharedEngine();
				ConfigurePassive(engine, warm_factor);
				if (ftz_case)
				{
					auto config = engine.Config();
					config.constraint_coupled_relaxation = 0.45f;
					engine.Config(config);
				}
				printf("passive-case: warm=%.9g speed=%.17g relaxation=%.9g\n", double(warm_factor), double(speed), double(engine.Config().constraint_coupled_relaxation));
				auto rejection_count = 0;
				engine.ConstraintRejections += [&](Engine&, std::span<ConstraintRejectionEvent const> events)
				{
					rejection_count += isize(events);
				};
				auto forest = std::array{&articulation};
				auto const input = Engine::StepInput{.m_articulations = forest, .m_constraints = &constraints, .m_elapsed_seconds = 1.0f / 60.0f, .m_substep_count = 1};
				auto const initial_energy = Energy(articulation.RootVelocity());
				auto previous_energy = initial_energy;
				auto max_increase = 0.0;
				auto max_energy = initial_energy;
				auto positive_work = 0.0;
				auto visits = 0;
				for (; visits != 600; ++visits)
				{
					engine.Step(input);
					auto const velocity = articulation.RootVelocity();
					auto const energy = Energy(velocity);
					auto const increase = std::max(0.0, energy - previous_energy);
					max_energy = std::max(max_energy, energy);
					max_increase = std::max(max_increase, increase);
					positive_work += increase;
					printf("passive-visit: warm=%.9g speed=%.17g step=%d x=%.17g energy=%.17g change=%.17g rejections=%d terminal=%u\n",
						double(warm_factor), double(speed), visits + 1, double(velocity.lin.x), energy, energy - previous_energy, rejection_count, static_cast<unsigned>(engine.LastFeatureStats().m_failure.m_reason));
					previous_energy = energy;
					auto const valid = std::isfinite(energy) && All(velocity.ang == v4::Zero()) && velocity.lin.y == 0.0f && velocity.lin.z == 0.0f &&
						!engine.LastFeatureStats().m_failure.HasTerminalFailure() && engine.LastCollisionStats().m_contact_count == 0 &&
						engine.LastFeatureStats().m_coupled.m_constraint_count == 3 && engine.LastFeatureStats().m_coupled.m_contact_capacity == 0 &&
						engine.LastStepProfile().m_readback_copy_count == 1;
					all_valid &= valid;
					if (!valid)
						break;

				}

				// "Material" here is ten parts per million of initial energy, not a physical-unit allowance or an arbitrary-duration claim.
				printf("passive-repeated: warm=%.9g speed=%.17g visits=%d initial=%.17g final=%.17g max_energy=%.17g max_increase=%.17g positive_work=%.17g rejections=%d\n",
					double(warm_factor), double(speed), visits, initial_energy, previous_energy, max_energy, max_increase, positive_work, rejection_count);
				all_valid &= visits == 600;
				all_bounded &= previous_energy < initial_energy && max_energy <= initial_energy * (1.0 + 1.0e-5) && positive_work <= initial_energy * 1.0e-5;
			}
			PR_EXPECT(all_valid);
			PR_EXPECT(all_bounded);
		}
	};

	// Terminal abandonment does not reset GpuJob recording, so this class intentionally never submits again after abandonment.
	PRUnitTestClass(EngineMigrationAbandonTests)
	{
		// A real submitted frame can be abandoned without publishing its physical state or constraint events.
		PRUnitTestMethod(SubmittedAbandonPreservesCallerState, Extended)
		{
			auto shape = collision::ShapeSphere{0.2f};
			auto body = RigidBody{&shape, m4x4::Identity(), Inertia::Sphere(0.2f, 1.0f)};
			auto [articulation, root] = MakeEngineFloatingRoot(10.0f);
			auto ordinary = CoupledEngineLinear(BodyRef::World(), BodyRef::Rigid(body));
			ordinary.D6().m_linear[0].m_mode = EConstraintAxisMode::Driven;
			ordinary.D6().m_linear[0].m_target_velocity = 1.0f;
			auto coupled = CoupledEngineLinear(BodyRef::World(), BodyRef::Link(articulation, root));
			coupled.D6().m_linear[0] = ordinary.D6().m_linear[0];
			auto constraints = ConstraintSet{};
			constraints.Add(ordinary);
			constraints.Add(coupled);
			auto bodies = std::array{&body};
			auto forest = std::array{&articulation};
			auto input = Engine::StepInput{.m_bodies = bodies, .m_articulations = forest, .m_constraints = &constraints, .m_elapsed_seconds = 1.0f / 60.0f};
			auto& engine = SharedEngine();
			ConfigureCoupledEngine(engine);
			auto config = engine.Config();
			config.solver_iterations = 1;
			config.push_out_iterations = 0;
			config.constraint_relaxation = 0.5f;
			config.constraint_coupled_relaxation = 0.5f;
			config.constraint_warm_start_factor = 1.0f;
			engine.Config(config);
			engine.Step(input);
			PR_EXPECT(FEqlAbsolute(body.VelocityWS().lin.x, 0.5f, 2.0e-5f));
			PR_EXPECT(FEqlAbsolute(articulation.LinkVelocity(root).lin.x, 0.5f, 2.0e-5f));
			auto const body_before = PackDynamics(body, 0);
			auto const root_pose = articulation.RootToWorld();
			auto const root_velocity = articulation.LinkVelocity(root);
			auto event_count = 0;
			engine.ConstraintRejections += [&](Engine&, std::span<ConstraintRejectionEvent const> values)
			{
				event_count += isize(values);
			};
			engine.ConstraintsBroken += [&](Engine&, std::span<ConstraintBreakEvent const> values)
			{
				event_count += isize(values);
			};

			// The second frame has real persistent work, but waiting through terminal abandonment is not CPU publication.
			engine.BeginStep(input);
			PR_EXPECT(engine.StepPending());
			engine.AbandonStep();
			PR_EXPECT(!engine.StepPending());
			PR_EXPECT(event_count == 0);
			auto const body_after = PackDynamics(body, 0);
			auto const pose_after = articulation.RootToWorld();
			auto const velocity_after = articulation.LinkVelocity(root);
			PR_EXPECT(std::memcmp(&body_before, &body_after, sizeof(body_before)) == 0);
			PR_EXPECT(std::memcmp(&pose_after, &root_pose, sizeof(root_pose)) == 0);
			PR_EXPECT(std::memcmp(&velocity_after, &root_velocity, sizeof(root_velocity)) == 0);
		}
	};
	// Focused controls for the coordinated Engine owner and phase migration.
	PRUnitTestClass(EngineMigrationFailureTests)
	{
		// A naturally rejected gathered frame must not leave GPU-only accepted guesses usable by either persistent owner.
		PRUnitTestMethod(RejectedFrameInvalidatesBothOwnersBeforeReuse, Extended)
		{
			// Keep initial and rejected frames identical; only the recovery clock and warm factor vary.
			for (auto const scaled_recovery : {false, true})
			{
				auto shape = collision::ShapeSphere{0.2f};
				auto body = RigidBody{&shape, m4x4::Identity(), Inertia::Sphere(0.2f, 1.0f)};
				auto crowd = std::array{
					RigidBody{&shape, m4x4::Translation(100, 0, 0), Inertia::Sphere(0.2f, 1.0f)},
					RigidBody{&shape, m4x4::Translation(200, 0, 0), Inertia::Sphere(0.2f, 1.0f)},
					RigidBody{&shape, m4x4::Translation(300, 0, 0), Inertia::Sphere(0.2f, 1.0f)},
				};
				auto [articulation, root] = MakeEngineFloatingRoot(10.0f);
				auto ordinary = CoupledEngineLinear(BodyRef::World(), BodyRef::Rigid(body));
				ordinary.D6().m_linear[0].m_mode = EConstraintAxisMode::Driven;
				ordinary.D6().m_linear[0].m_target_velocity = 1.0f;
				auto coupled = CoupledEngineLinear(BodyRef::World(), BodyRef::Link(articulation, root));
				coupled.D6().m_linear[0] = ordinary.D6().m_linear[0];
				auto constraints = ConstraintSet{};
				constraints.Add(ordinary);
				constraints.Add(coupled);
				auto bodies = std::array{&body, &crowd[0], &crowd[1], &crowd[2]};
				auto forest = std::array{&articulation};
				auto input = Engine::StepInput{.m_bodies = bodies, .m_articulations = forest, .m_constraints = &constraints, .m_elapsed_seconds = 1.0f / 60.0f};
				auto& engine = SharedEngine();
				ConfigureCoupledEngine(engine);
				auto config = engine.Config();
				config.max_collision_pairs = 1;
				config.solver_iterations = 1;
				config.push_out_iterations = 0;
				config.velocity_baumgarte = 0.0f;
				config.deep_penetration_baumgarte_min = 0.0f;
				config.deep_penetration_baumgarte_max = 0.0f;
				config.warm_start_scale = 0.0f;
				config.constraint_relaxation = 0.5f;
				config.constraint_coupled_relaxation = 0.5f;
				config.constraint_warm_start_factor = 1.0f;
				engine.Config(config);
				engine.Step(input);
				PR_EXPECT(FEqlAbsolute(body.VelocityWS().lin.x, 0.5f, 2.0e-5f));
				PR_EXPECT(FEqlAbsolute(articulation.LinkVelocity(root).lin.x, 0.5f, 2.0e-5f));

				// Keep topology, packed counts and owner identities unchanged: only independent collision geometry exceeds the existing capacity.
				for (auto& value : crowd)
					value.O2W(m4x4::Translation(100, 0, 0));

				auto const body_before = PackDynamics(body, 0);
				auto const root_pose = articulation.RootToWorld();
				auto const root_velocity = articulation.LinkVelocity(root);
				PR_THROWS(engine.Step(input), std::runtime_error);
				PR_EXPECT(engine.LastFeatureStats().m_failure.m_reason == EStepFailure::CollisionPairCapacity);
				PR_EXPECT(!engine.StepPending());
				auto const body_after = PackDynamics(body, 0);
				auto const pose_after = articulation.RootToWorld();
				auto const velocity_after = articulation.LinkVelocity(root);
				PR_EXPECT(std::memcmp(&body_before, &body_after, sizeof(body_before)) == 0);
				PR_EXPECT(std::memcmp(&pose_after, &root_pose, sizeof(root_pose)) == 0);
				PR_EXPECT(std::memcmp(&velocity_after, &root_velocity, sizeof(root_velocity)) == 0);

				// Cold recovery reaches .75. With unchanged h, retained ordinary support instead reaches 1; shared row revisions mask coupled support.
				for (int index = 0; index != isize(crowd); ++index)
					crowd[index].O2W(m4x4::Translation(100.0f * (index + 1), 0, 0));

				if (scaled_recovery)
				{
					// Retained acceptance overflows its provisional scale before invalidated row receipts can discard guesses; fresh ownership has scale zero.
					input.m_elapsed_seconds *= 2.0f;
					config.constraint_warm_start_factor = 2.0e38f;
					engine.Config(config);
				}
				auto rejections = std::vector<ConstraintRejectionEvent>{};
				engine.ConstraintRejections += [&](Engine&, std::span<ConstraintRejectionEvent const> values)
				{
					rejections.insert(rejections.end(), values.begin(), values.end());
				};
				engine.Step(input);
				printf("rejected-frame recovery: scaled=%d ordinary=%.9g coupled=%.9g rejections=%zu\n", scaled_recovery, body.VelocityWS().lin.x, articulation.LinkVelocity(root).lin.x, rejections.size());
				for (auto const& event : rejections)
				{
					printf("recovery rejection: owner=%u index=%u outcome=%u flags=%u phase=%u substep=%d terminal=%u\n", static_cast<unsigned>(event.m_owner), event.m_index, static_cast<unsigned>(event.m_outcome), event.m_failure_flags, static_cast<unsigned>(event.m_phase), event.m_substep_index, static_cast<unsigned>(engine.LastFeatureStats().m_failure.m_reason));
				}
				PR_EXPECT(FEqlAbsolute(body.VelocityWS().lin.x, 0.75f, 2.0e-5f));
				PR_EXPECT(FEqlAbsolute(articulation.LinkVelocity(root).lin.x, 0.75f, 2.0e-5f));
				PR_EXPECT(rejections.empty());
				PR_EXPECT(!engine.LastFeatureStats().m_failure.HasTerminalFailure());
			}
		}

		// Unrepresentable finite soft-row coefficients retain the existing terminal slot failure and invalidate both unpublished owners.
		PRUnitTestMethod(FatalCompileFailureInvalidatesUnpublishedOwners, Extended)
		{
			auto shape = collision::ShapeSphere{0.2f};
			auto body = RigidBody{&shape, m4x4::Identity(), Inertia::Sphere(0.2f, 1.0f)};
			auto auxiliary = RigidBody{&shape, m4x4::Translation(100, 0, 0), Inertia::Sphere(0.2f, 1.0f)};
			auto [articulation, root] = MakeEngineFloatingRoot(10.0f);
			auto ordinary = CoupledEngineLinear(BodyRef::World(), BodyRef::Rigid(body));
			ordinary.D6().m_linear[0].m_mode = EConstraintAxisMode::Driven;
			ordinary.D6().m_linear[0].m_target_velocity = 1.0f;
			auto coupled = CoupledEngineLinear(BodyRef::World(), BodyRef::Link(articulation, root));
			coupled.D6().m_linear[0] = ordinary.D6().m_linear[0];
			auto soft = CoupledEngineLinear(BodyRef::World(), BodyRef::Rigid(auxiliary));
			soft.D6().m_linear[0].m_mode = EConstraintAxisMode::Driven;
			soft.D6().m_linear[0].m_damping = 1.0f;
			auto constraints = ConstraintSet{};
			constraints.Add(ordinary);
			constraints.Add(coupled);
			auto const soft_handle = constraints.Add(soft);
			auto bodies = std::array{&body, &auxiliary};
			auto forest = std::array{&articulation};
			auto input = Engine::StepInput{.m_bodies = bodies, .m_articulations = forest, .m_constraints = &constraints, .m_elapsed_seconds = 1.0f / 16.0f};
			auto& engine = SharedEngine();
			ConfigureCoupledEngine(engine);
			auto config = engine.Config();
			config.solver_iterations = 1;
			config.push_out_iterations = 0;
			config.constraint_relaxation = 0.5f;
			config.constraint_coupled_relaxation = 0.5f;
			config.constraint_warm_start_factor = 1.0f;
			engine.Config(config);
			engine.Step(input);
			PR_EXPECT(FEqlAbsolute(body.VelocityWS().lin.x, 0.5f, 2.0e-5f));
			PR_EXPECT(FEqlAbsolute(articulation.LinkVelocity(root).lin.x, 0.5f, 2.0e-5f));

			// Gamma = 1/(h*c) cannot be represented. Inputs, membership and h remain valid; no fault seam or device loss is used.
			soft.D6().m_linear[0].m_damping = 2.0e-38f;
			constraints.Update(soft_handle, soft);
			auto const body_before = PackDynamics(body, 0);
			auto const auxiliary_before = PackDynamics(auxiliary, 1);
			auto const root_pose = articulation.RootToWorld();
			auto const root_velocity = articulation.LinkVelocity(root);
			auto rejection_count = 0;
			engine.ConstraintRejections += [&](Engine&, std::span<ConstraintRejectionEvent const> values)
			{
				rejection_count += isize(values);
			};
			PR_THROWS(engine.Step(input), std::runtime_error);
			PR_EXPECT(engine.LastFeatureStats().m_failure.m_reason == EStepFailure::ConstraintNumerics);
			PR_EXPECT(engine.LastFeatureStats().m_failure.m_item_index == static_cast<int>(soft_handle.m_index));
			PR_EXPECT(rejection_count == 0);
			PR_EXPECT(!engine.StepPending());
			auto const body_after = PackDynamics(body, 0);
			auto const auxiliary_after = PackDynamics(auxiliary, 1);
			auto const pose_after = articulation.RootToWorld();
			auto const velocity_after = articulation.LinkVelocity(root);
			PR_EXPECT(std::memcmp(&body_before, &body_after, sizeof(body_before)) == 0);
			PR_EXPECT(std::memcmp(&auxiliary_before, &auxiliary_after, sizeof(auxiliary_before)) == 0);
			PR_EXPECT(std::memcmp(&pose_after, &root_pose, sizeof(root_pose)) == 0);
			PR_EXPECT(std::memcmp(&velocity_after, &root_velocity, sizeof(root_velocity)) == 0);

			// The recovered owners must start cold from the still-published speed .5, not from the failed frame's GPU-only caches.
			soft.D6().m_linear[0].m_damping = 1.0f;
			constraints.Update(soft_handle, soft);
			engine.Step(input);
			printf("fatal-frame recovery: ordinary=%.9g coupled=%.9g\n", body.VelocityWS().lin.x, articulation.LinkVelocity(root).lin.x);
			PR_EXPECT(FEqlAbsolute(body.VelocityWS().lin.x, 0.75f, 2.0e-5f));
			PR_EXPECT(FEqlAbsolute(articulation.LinkVelocity(root).lin.x, 0.75f, 2.0e-5f));
			PR_EXPECT(!engine.LastFeatureStats().m_failure.HasTerminalFailure());
		}
	};

}
#endif
