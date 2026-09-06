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

		// A zero-mobility coupled island reports bounded rejection without aborting publication or adding a readback.
		PRUnitTestMethod(CoupledNonConvergenceUsesFrameReadback, Extended)
		{
			auto builder = ArticulationBuilder{};
			auto const root = builder.AddFixedRoot(CoupledEngineLink());
			auto articulation = builder.Build();
			auto constraints = ConstraintSet{};
			constraints.Add(CoupledEngineLinear(BodyRef::Link(articulation, root), BodyRef::World()));
			auto articulation_ptrs = std::array<Articulation*, 1>{&articulation};
			auto failures = std::vector<CoupledConstraintFailureEvent>{};
			auto& engine = SharedEngine();
			ConfigureCoupledEngine(engine);
			engine.CoupledConstraintFailures += [&](Engine&, std::span<CoupledConstraintFailureEvent const> events)
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
				PR_EXPECT(failures[0].m_island_index == 0);
				PR_EXPECT(failures[0].m_iteration_count > 0);
				PR_EXPECT(failures[0].m_failure_flags != 0);
			}
			auto const& stats = engine.LastFeatureStats();
			PR_EXPECT(stats.m_failure.m_reason == EStepFailure::CoupledConstraintNonConvergence);
			PR_EXPECT(stats.m_frame_output.m_coupled_failure_count == 1);
			PR_EXPECT(stats.m_frame_output.m_readback_count == 1);
			PR_EXPECT(engine.LastStepProfile().m_readback_copy_count == 1);
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
				pr::unittests::TestFramework::out() << std::format(
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
			pr::unittests::TestFramework::out() << std::format(
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
			pr::unittests::TestFramework::out() << std::format(
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
				PR_EXPECT(engine.LastStepProfile().m_submission_count == 1);
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
}
#endif
