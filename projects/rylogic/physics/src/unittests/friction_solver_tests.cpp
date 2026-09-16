//*********************************************
// Physics Engine
// Copyright (C) Rylogic Ltd 2026
//*********************************************
#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/physics/physics.h"
#include "src/compute/interop/resolve_runner.h"
#include "src/unittests/shared_engine.h"

namespace pr::physics::tests
{
	// Exercise physical friction budgets independently of penetration bias and fresh normal closing.
	PRUnitTestClass(FrictionSolverTests)
	{
		// A warm-started support impulse must resist slip even when the normal constraint is already separating.
		PRUnitTestMethod(WarmNormalSupportCancelsSlip, Quick)
		{
			auto config = EngineConfig{};
			config.push_out_iterations = 0;
			config.velocity_baumgarte = 0;
			auto sphere = collision::ShapeSphere{0.5f};
			auto ground = collision::ShapeBox{v4{10, 10, 1, 0}};
			auto body = RigidBody{&sphere, m4x4::Translation(0, 0, 0.5f), Inertia::Sphere(0.5f, 1)};
			auto fixed = RigidBody{&ground, m4x4::Translation(0, 0, -0.5f), Inertia::Infinite()};
			body.VelocityWS(v4{0, 0.1f, 0, 0}, v4{-0.01f, 0, 0.001f, 0});
			auto bodies = std::array{PackDynamics(body, 0), PackDynamics(fixed, 1)};
			auto contacts = std::array{GpuResolveContact{
				.axis = -v4::ZAxis(),
				.contact_point = v4{0, 0, -0.5f, 1},
				.b2a = InvertOrthonormal(body.O2W()) * fixed.O2W(),
				.body_idx_a = 0, .body_idx_b = 1,
				.warmstart_impulse = v4{0, 0, -0.16f, 0},
			}};
			auto materials = std::array{GpuMaterial{.friction_static = 0.3f}};
			auto const energy_before = body.KineticEnergy();
			auto runner = ResolveInteropRunner{config};
			runner.Run({.m_dt = 1.0f / 60, .m_bodies = bodies, .m_contacts = contacts, .m_materials = materials});
			UnpackDynamics(bodies[0], body);

			// Neither fresh closing nor bias is available: only the already-applied normal support can fund this tangent impulse.
			auto const velocity = body.VelocityWS();
			PR_EXPECT(Abs(velocity.LinAt(v4{0, 0, -0.5f, 0}).x) < 1.0e-5f);
			PR_EXPECT(FEqlAbsolute(velocity.lin.z, 0.001f, 1.0e-6f));
			PR_EXPECT(body.KineticEnergy() <= energy_before + 1.0e-7f);
		}

		// Extra sweeps must not multiply the Coulomb allowance of a saturated sliding contact.
		PRUnitTestMethod(SlidingImpulseBudgetDoesNotGrowWithIterations, Quick)
		{
			auto momentum_after = std::array<float, 2>{};
			for (int run = 0; run != 2; ++run)
			{
				auto config = EngineConfig{};
				config.push_out_iterations = 0;
				config.velocity_baumgarte = 0;
				config.solver_iterations = run == 0 ? 1 : 12;
				auto sphere = collision::ShapeSphere{0.5f};
				auto ground = collision::ShapeBox{v4{10, 10, 1, 0}};
				auto body = RigidBody{&sphere, m4x4::Translation(0, 0, 0.5f), Inertia::Sphere(0.5f, 1)};
				auto fixed = RigidBody{&ground, m4x4::Translation(0, 0, -0.5f), Inertia::Infinite()};
				body.VelocityWS(v4::Zero(), v4{10, 0, 0, 0});
				auto bodies = std::array{PackDynamics(body, 0), PackDynamics(fixed, 1)};
				auto contacts = std::array{GpuResolveContact{
					.axis = -v4::ZAxis(),
					.contact_point = v4{0, 0, -0.5f, 1},
					.b2a = InvertOrthonormal(body.O2W()) * fixed.O2W(),
					.body_idx_a = 0, .body_idx_b = 1,
					.warmstart_impulse = v4{0, 0, -0.1f, 0},
				}};
				auto materials = std::array{GpuMaterial{.friction_static = 0.3f}};
				auto runner = ResolveInteropRunner{config};
				runner.Run({.m_dt = 1.0f / 60, .m_bodies = bodies, .m_contacts = contacts, .m_materials = materials});
				momentum_after[run] = bodies[0].momentum_lin.x;

				// The accumulated delta reaches, but cannot exceed, the one physical support budget.
				auto const limit = 0.1f * 0.3f / (1.000001f - 0.3f);
				PR_EXPECT(FEqlAbsolute(10 - momentum_after[run], limit, 2.0e-6f));
				PR_EXPECT(Length(contacts[0].friction_impulses[0]) <= limit + 1.0e-6f);
			}
			PR_EXPECT(FEqlAbsolute(momentum_after[0], momentum_after[1], 1.0e-6f));
		}

		// A broad face resists torsion using a shared physical budget, but penetration bias alone cannot create that budget.
		PRUnitTestMethod(ManifoldFrictionUsesPhysicalSupportNotBias, Quick)
		{
			for (auto const supported : {false, true})
			{
				auto config = EngineConfig{};
				config.push_out_iterations = 0;
				auto box = collision::ShapeBox{v4{1, 1, 1, 0}};
				auto ground = collision::ShapeBox{v4{10, 10, 1, 0}};
				auto body = RigidBody{&box, m4x4::Translation(0, 0, 0.5f), Inertia::Box(box.m_radius, 1)};
				auto fixed = RigidBody{&ground, m4x4::Translation(0, 0, -0.5f), Inertia::Infinite()};
				body.VelocityWS(v4::ZAxis(), v4::Zero());
				auto bodies = std::array{PackDynamics(body, 0), PackDynamics(fixed, 1)};
				auto contacts = std::array{GpuResolveContact{
					.axis = -v4::ZAxis(),
					.contact_point = v4{0, 0, -0.5f, 1},
					.manifold = {v4{-0.5f, -0.5f, -0.5f, 1}, v4{+0.5f, -0.5f, -0.5f, 1}, v4{+0.5f, +0.5f, -0.5f, 1}, v4{-0.5f, +0.5f, -0.5f, 1}},
					.b2a = InvertOrthonormal(body.O2W()) * fixed.O2W(),
					.body_idx_a = 0, .body_idx_b = 1,
					.depth = supported ? 0.0f : 0.1f,
					.feature = 4,
					.warmstart_impulse = supported ? v4{0, 0, -0.16f, 0} : v4::Zero(),
				}};
				auto materials = std::array{GpuMaterial{.friction_static = 0.3f}};
				auto runner = ResolveInteropRunner{config};
				runner.Run({.m_dt = 1.0f / 60, .m_bodies = bodies, .m_contacts = contacts, .m_materials = materials});
				UnpackDynamics(bodies[0], body);

				// All four point accumulators share the contact's normal allowance rather than each receiving a full friction cone.
				auto total_tangent = 0.0f;
				for (auto const& impulse : contacts[0].friction_impulses)
					total_tangent += Length(impulse);

				if (supported)
				{
					PR_EXPECT(body.VelocityWS().ang.z < 0.99f);
					PR_EXPECT(total_tangent <= 0.16f * 0.3f / (1.000001f - 0.3f) + 1.0e-5f);
				}
				else
				{
					PR_EXPECT(FEqlAbsolute(body.VelocityWS().ang.z, 1.0f, 1.0e-6f));
					PR_EXPECT(total_tangent == 0);
					PR_EXPECT(body.VelocityWS().lin.z > 0);
				}
			}
		}

		// Resolving an impact must retain the transform that generated the stored contact geometry.
		PRUnitTestMethod(CollisionTimeRewindPreservesGeneratingTransform, Quick)
		{
			// Shallow overlap and a closing velocity force a nonzero collision-time rewind without position correction or friction.
			auto config = EngineConfig{};
			config.push_out_iterations = 0;
			config.solver_iterations = 1;
			config.velocity_baumgarte = 0;
			auto sphere = collision::ShapeSphere{0.5f};
			auto ground = collision::ShapeBox{v4{10, 10, 1, 0}};
			auto body = RigidBody{&sphere, m4x4::Translation(0, 0, 0.495f), Inertia::Sphere(0.5f, 1)};
			auto fixed = RigidBody{&ground, m4x4::Translation(0, 0, -0.5f), Inertia::Infinite()};
			body.VelocityWS(v4::Zero(), -v4::ZAxis());
			auto const generating_b2a = InvertOrthonormal(body.O2W()) * fixed.O2W();
			auto bodies = std::array{PackDynamics(body, 0), PackDynamics(fixed, 1)};
			auto contacts = std::array{GpuResolveContact{
				.axis = -v4::ZAxis(),
				.contact_point = v4{0, 0, -0.4975f, 1},
				.b2a = generating_b2a,
				.body_idx_a = 0, .body_idx_b = 1,
				.depth = 0.005f,
			}};
			auto materials = std::array{GpuMaterial{}};
			auto runner = ResolveInteropRunner{config};
			runner.Run({.m_dt = 1.0f / 60, .m_bodies = bodies, .m_contacts = contacts, .m_materials = materials});

			// Require a real rewind and impulse writeback, then check that all four generating transform columns survive unchanged.
			PR_EXPECT(contacts[0].collision_time < -1.0e-6f);
			PR_EXPECT(FEqlAbsolute(bodies[0].momentum_lin.z, 0.0f, 1.0e-5f));
			for (int column = 0; column != 4; ++column)
				PR_EXPECT(FEqlAbsolute(contacts[0].b2a[column], generating_b2a[column], 1.0e-6f));
		}

		// Mixed rigid/articulation scheduling must spend only the accepted part of an ordinary contact's preloaded normal impulse.
		PRUnitTestMethod(MixedPreloadedWarmStartUsesAcceptedSupport, Extended)
		{
			// Seed the same 0.1 N s cache entry, then distinguish rejected, partially accepted, and fully accepted normal support.
			auto const normal_velocities = std::array{1.0f, -0.02f, -0.1f};
			auto const accepted_normal_impulses = std::array{0.0f, 0.04f, 0.1f};
			for (int run = 0; run != isize(normal_velocities); ++run)
			{
				auto& engine = SharedEngine();
				ResetEngineForNextTest(engine);
				engine.Terrain(std::nullopt);
				engine.CylindricalBoundary(std::nullopt);
				auto config = EngineConfig{};
				config.sleeping_enabled = false;
				config.selective_refresh_passes = 0;
				config.push_out_iterations = 0;
				config.solver_iterations = 1;
				config.velocity_baumgarte = 0;
				config.warm_start_scale = 1;
				engine.Config(config);
				engine.Material(Material{.m_friction_static = 0.3f, .m_elasticity_norm = 0});
				auto sphere = collision::ShapeSphere{0.5f};
				auto ground_shape = collision::ShapeBox{v4{10, 10, 1, 0}};
				auto body = RigidBody{&sphere, m4x4::Translation(0, 0, 0.45f), Inertia::Sphere(0.5f, 1)};
				auto ground = RigidBody{&ground_shape, m4x4::Translation(0, 0, -0.5f), Inertia::Infinite()};
				body.VelocityWS(v4::Zero(), v4{0, 0, -0.1f, 0});

				// A shaped floating root touching the same immovable floor activates the coupled lane and its all-contact preload dispatch.
				auto builder = ArticulationBuilder{};
				builder.AddFloatingRoot(ArticulationLinkDesc{.m_inertia = Inertia::Sphere(0.5f, 1), .m_shape = collision::shape_cast(&sphere)},
					m4x4::Translation(-3, 0, 0.45f), v8motion{v4::Zero(), v4{0, 0, -0.1f, 0}});
				auto articulation = builder.Build();
				auto bodies = std::array<RigidBody*, 2>{&body, &ground};
				auto articulations = std::array<Articulation*, 1>{&articulation};
				auto input = Engine::StepInput{.m_bodies = bodies, .m_articulations = articulations, .m_elapsed_seconds = 1.0f / 60, .m_substep_count = 1};

				// Retain the actual ordinary contact so a cache miss caused by changed local geometry cannot masquerade as a rejected preload.
				auto rigid_contact = std::optional<RbContact>{};
				engine.Collisions += [&](Engine&, std::span<RbContact const> contacts)
				{
					for (auto const& contact : contacts)
					{
						if ((contact.m_objA == &body && contact.m_objB == &ground) || (contact.m_objA == &ground && contact.m_objB == &body))
							rigid_contact = contact;
					}
				};
				engine.Step(input);
				PR_EXPECT(engine.LastCollisionStats().m_contact_count >= 2);
				PR_EXPECT(engine.LastFeatureStats().m_articulations.m_articulation_count == 1);
				PR_EXPECT(engine.LastFeatureStats().m_coupled.m_resources.m_dispatch_count > 0);
				PR_EXPECT(FEqlAbsolute(body.VelocityWS().lin.z, 0.0f, 1.0e-5f));
				auto const seed_contact = rigid_contact.value();
				auto const contact_o2w = body.O2W();

				// Offset only the fixture's initial pose to cancel integration drift; WarmStartKey hashes the quantised local contact point.
				auto const normal_velocity = normal_velocities[run];
				auto const accepted_normal = accepted_normal_impulses[run];
				auto const initial_velocity = v4{1, 0, normal_velocity, 0};
				auto initial_o2w = contact_o2w;
				initial_o2w.pos -= input.m_elapsed_seconds * initial_velocity;
				body.O2W(initial_o2w);
				body.VelocityWS(v4::Zero(), initial_velocity);
				rigid_contact.reset();
				engine.Step(input);
				PR_EXPECT(engine.LastCollisionStats().m_contact_count >= 2);
				PR_EXPECT(engine.LastFeatureStats().m_coupled.m_resources.m_dispatch_count > 0);
				auto const replay_contact = rigid_contact.value();
				PR_EXPECT(replay_contact.m_objA == seed_contact.m_objA && replay_contact.m_objB == seed_contact.m_objB);
				PR_EXPECT(replay_contact.m_child_idA == seed_contact.m_child_idA && replay_contact.m_child_idB == seed_contact.m_child_idB);
				PR_EXPECT(FEqlAbsolute(replay_contact.Point(), seed_contact.Point(), 1.0e-6f));
				PR_EXPECT(FEqlAbsolute(body.O2W().pos, contact_o2w.pos, 1.0e-6f));
				auto const velocity = body.VelocityWS();
				auto const friction_limit = accepted_normal * 0.3f / (1.000001f - 0.3f);
				printf("Mixed preload: initial vn=%g, expected accepted Jn=%g, final vx=%g, final vz=%g, angular speed=%g, local point delta=%g\n",
					normal_velocity, accepted_normal, velocity.lin.x, velocity.lin.z, Length(velocity.ang), Length(replay_contact.Point() - seed_contact.Point()));
				PR_EXPECT(FEqlAbsolute(velocity.lin.z, normal_velocity + accepted_normal, 2.0e-5f));
				PR_EXPECT(FEqlAbsolute(velocity.lin.x, 1 - friction_limit, 2.0e-5f));
				if (accepted_normal == 0)
				{
					PR_EXPECT(FEqlAbsolute(velocity.ang, v4::Zero(), 1.0e-5f));

					// A rejected preload must not poison the following cache; with ordinary iterations disabled, no fresh normal response is available.
					auto const cache_probe_velocity = v4{0, 0, -0.02f, 0};
					initial_o2w = contact_o2w;
					initial_o2w.pos -= input.m_elapsed_seconds * cache_probe_velocity;
					body.O2W(initial_o2w);
					body.VelocityWS(v4::Zero(), cache_probe_velocity);
					config.solver_iterations = 0;
					engine.Config(config);
					rigid_contact.reset();
					engine.Step(input);
					PR_EXPECT(engine.LastCollisionStats().m_contact_count >= 2);
					PR_EXPECT(FEqlAbsolute(rigid_contact.value().Point(), seed_contact.Point(), 1.0e-6f));
					PR_EXPECT(FEqlAbsolute(body.VelocityWS().lin.z, -0.02f, 2.0e-5f));
				}
				engine.Collisions.reset();
			}
		}

		// Exact torque must hold a sphere on a feasible incline with the default warm-start setting and production material.
		PRUnitTestMethod(DefaultWarmStartSphereHoldOnIncline, Extended)
		{
			auto& engine = SharedEngine();
			ResetEngineForNextTest(engine);
			engine.Terrain(std::nullopt);
			engine.CylindricalBoundary(std::nullopt);
			engine.Material(Material{.m_friction_static = 0.3f, .m_elasticity_norm = 0.05f, .m_elasticity_tang = 0, .m_elasticity_tors = 0, .m_density = 1});
			auto const angle = DegreesToRadians(10.0f);
			auto const normal = v4{-Sin(angle), 0, Cos(angle), 0};
			auto const uphill = v4{Cos(angle), 0, Sin(angle), 0};
			auto sphere = collision::ShapeSphere{0.5f};
			auto ground = collision::ShapeBox{v4{100, 100, 2, 0}};
			auto body = RigidBody{&sphere, m4x4::Translation(v4::Origin() + 0.495f * normal), Inertia::Sphere(0.5f, 1)};
			auto fixed = RigidBody{&ground, m4x4{uphill, v4::YAxis(), normal, v4::Origin() - normal}, Inertia::Infinite()};
			auto bodies = std::array<RigidBody*, 2>{&body, &fixed};
			auto const start = body.O2W().pos;
			auto const gravity = v4{0, 0, -9.81f, 0};
			auto const torque = v4{0, 0.5f * 9.81f * Sin(angle), 0, 0};
			auto max_late_speed = 0.0f;
			auto max_late_slip = 0.0f;
			for (int tick = 0; tick != 120; ++tick)
			{
				body.ApplyForceWS(v8force{torque, gravity});
				engine.Step(Engine::StepInput{.m_bodies = bodies, .m_elapsed_seconds = 1.0f / 60, .m_substep_count = 4, .m_time_s = tick / 60.0});
				auto const velocity = body.VelocityWS();
				if (tick >= 60)
				{
					max_late_speed = Max(max_late_speed, Length(velocity.lin));
					max_late_slip = Max(max_late_slip, Abs(Dot3(0.5f * Cross(velocity.ang, normal) - velocity.lin, uphill)));
				}
			}

			// Zero-feedback hold isolates the solver: the old normal-only warm-start path slides downhill and spins despite sufficient traction.
			printf("Exact hold: late speed=%g, late slip=%g, uphill displacement=%g\n", max_late_speed, max_late_slip, Dot3(body.O2W().pos - start, uphill));
			PR_EXPECT(max_late_speed < 0.05f);
			PR_EXPECT(max_late_slip < 0.05f);
			PR_EXPECT(Abs(Dot3(body.O2W().pos - start, uphill)) < 0.05f);
		}
	};
}
#endif
