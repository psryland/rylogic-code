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
			// The solver withdraws the part of that support that leaves the bodies separating, so the normal speed settles at zero.
			auto const velocity = body.VelocityWS();
			PR_EXPECT(Abs(velocity.LinAt(v4{0, 0, -0.5f, 0}).x) < 1.0e-5f);
			PR_EXPECT(FEqlAbsolute(velocity.lin.z, 0.0f, 1.0e-6f));
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

				// The solver iteration withdraws any accepted support that leaves the bodies separating, but never more than the accepted preload.
				auto const retained_normal = accepted_normal - Min(Max(normal_velocity + accepted_normal, 0.0f), accepted_normal);
				auto const friction_limit = retained_normal * 0.3f / (1.000001f - 0.3f);
				log_stream() << std::format("Mixed preload: initial vn={:g}, expected accepted Jn={:g}, retained Jn={:g}, final vx={:g}, final vz={:g}, angular speed={:g}, local point delta={:g}\n",
					normal_velocity, accepted_normal, retained_normal, velocity.lin.x, velocity.lin.z, Length(velocity.ang), Length(replay_contact.Point() - seed_contact.Point()));
				PR_EXPECT(FEqlAbsolute(velocity.lin.z, normal_velocity + retained_normal, 2.0e-5f));
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
			log_stream() << std::format("Exact hold: late speed={:g}, late slip={:g}, uphill displacement={:g}\n", max_late_speed, max_late_slip, Dot3(body.O2W().pos - start, uphill));
			PR_EXPECT(max_late_speed < 0.05f);
			PR_EXPECT(max_late_slip < 0.05f);
			PR_EXPECT(Abs(Dot3(body.O2W().pos - start, uphill)) < 0.05f);
		}
	};

	// Prepare the shared engine for a rolling resistance test: no terrain or boundary, and a grippy material with the given resistance coefficients.
	inline Engine& RollingResistanceEngine(float rolling_resistance, float spin_resistance)
	{
		// Friction is high enough that every test body rolls rather than slides
		auto& engine = SharedEngine();
		ResetEngineForNextTest(engine);
		engine.Terrain(std::nullopt);
		engine.CylindricalBoundary(std::nullopt);
		engine.Material(Material{
			.m_friction_static = 0.6f,
			.m_elasticity_norm = 0.05f,
			.m_density = 1,
			.m_rolling_resistance = rolling_resistance,
			.m_spin_resistance = spin_resistance,
		});
		return engine;
	}

	// Rolling and spin resistance must stop round bodies without preventing tall bodies from tipping.
	PRUnitTestClass(RollingResistanceTests)
	{
		// The accumulated angular impulse must oppose rolling and spin, and stay within 'coefficient * Jn * r'. Zero coefficients must change nothing.
		PRUnitTestMethod(AngularImpulseRespectsLimit, Quick)
		{
			for (auto const coefficient : {0.0f, 0.2f})
			{
				// A ball of radius 0.5 rolls along +X without slip and also spins about the contact normal, supported by a known normal impulse.
				auto config = EngineConfig{};
				config.push_out_iterations = 0;
				config.velocity_baumgarte = 0;
				auto const radius = 0.5f;
				auto const support = 0.1f;
				auto sphere = collision::ShapeSphere{radius};
				auto ground = collision::ShapeBox{v4{10, 10, 1, 0}};
				auto body = RigidBody{&sphere, m4x4::Translation(0, 0, radius), Inertia::Sphere(radius, 1)};
				auto fixed = RigidBody{&ground, m4x4::Translation(0, 0, -0.5f), Inertia::Infinite()};
				body.VelocityWS(v4{0, 2, 3, 0}, v4{1, 0, 0, 0});
				auto bodies = std::array{PackDynamics(body, 0), PackDynamics(fixed, 1)};
				auto contacts = std::array{GpuResolveContact{
					.axis = -v4::ZAxis(),
					.contact_point = v4{0, 0, -radius, 1},
					.b2a = InvertOrthonormal(body.O2W()) * fixed.O2W(),
					.body_idx_a = 0, .body_idx_b = 1,
					.warmstart_impulse = v4{0, 0, -support, 0},
				}};
				auto materials = std::array{GpuMaterial{.friction_static = 0.6f, .rolling_resistance = coefficient, .spin_resistance = coefficient}};
				auto const energy_before = body.KineticEnergy();
				auto runner = ResolveInteropRunner{config};
				runner.Run({.m_dt = 1.0f / 60, .m_bodies = bodies, .m_contacts = contacts, .m_materials = materials});
				UnpackDynamics(bodies[0], body);

				// Split the accumulated angular impulse into its spin (normal) and rolling (tangent) parts
				auto const angular_impulse = contacts[0].angular_impulse;
				auto const spin_impulse = Abs(angular_impulse.z);
				auto const roll_impulse = Length(v4{angular_impulse.x, angular_impulse.y, 0, 0});
				auto const velocity = body.VelocityWS();
				log_stream() << std::format("coefficient={:g}: roll impulse={:g} spin impulse={:g} ang=({:g},{:g},{:g})\n", coefficient, roll_impulse, spin_impulse, velocity.ang.x, velocity.ang.y, velocity.ang.z);
				if (coefficient == 0)
				{
					PR_EXPECT(Length(angular_impulse) == 0);
					PR_EXPECT(FEqlAbsolute(velocity.ang.z, 3.0f, 1.0e-5f));
				}
				else
				{
					// The ball's inertia about any axis is 0.4 * m * r^2 = 0.1, so a saturated impulse removes at most limit / 0.1 of the spin rate
					auto const limit = coefficient * support * radius;
					PR_EXPECT(roll_impulse > 0 && roll_impulse <= limit + 1.0e-6f);
					PR_EXPECT(spin_impulse > 0 && spin_impulse <= limit + 1.0e-6f);
					PR_EXPECT(velocity.ang.z < 3.0f && velocity.ang.z >= 3.0f - limit / 0.1f - 1.0e-5f);
					PR_EXPECT(body.KineticEnergy() < energy_before);
				}
			}
		}

		// A rolling ball and a rolling capsule on flat ground must stop and sleep, while without resistance they keep rolling.
		PRUnitTestMethod(RoundBodiesStopAndSleepOnFlatGround, Extended)
		{
			for (auto const coefficient : {0.0f, 0.1f})
			{
				auto& engine = RollingResistanceEngine(coefficient, coefficient);

				// Both bodies roll along +Y at 2 m/s without slip. The capsule lies along the X axis.
				auto const speed = 2.0f;
				auto sphere = collision::ShapeSphere{0.3f};
				auto capsule = collision::ShapeLine{1.6f, 0.25f, m4x4::Transform(v4::YAxis(), constants<float>::tau_by_4, v4::Origin())};
				auto ground_shape = collision::ShapeBox{v4{200, 200, 2, 0}};
				auto ball = RigidBody{&sphere, m4x4::Translation(-3, 0, 0.3f), Inertia::Sphere(0.3f, 2)};
				auto log = RigidBody{};
				log.Shape(collision::shape_cast(&capsule), 5.0f, false);
				log.O2W(m4x4::Translation(3, 0, 0.25f));
				auto ground = RigidBody{&ground_shape, m4x4::Translation(0, 0, -1), Inertia::Infinite()};
				ball.VelocityWS(v4{-speed / 0.3f, 0, 0, 0}, v4{0, speed, 0, 0});
				log.VelocityWS(v4{-speed / 0.25f, 0, 0, 0}, v4{0, speed, 0, 0});
				auto bodies = std::array<RigidBody*, 3>{&ball, &log, &ground};
				for (int tick = 0; tick != 300; ++tick)
				{
					ball.GravityWS(v4{0, 0, -9.81f, 0});
					log.GravityWS(v4{0, 0, -9.81f, 0});
					engine.Step(Engine::StepInput{.m_bodies = bodies, .m_elapsed_seconds = 1.0f / 60, .m_substep_count = 4, .m_time_s = tick / 60.0});
				}

				// Rolling resistance of 0.1 decelerates the ball at about 0.7 m/s^2, so both bodies stop within about 3 seconds and then sleep
				auto const ball_speed = Length(ball.VelocityWS().lin);
				auto const log_speed = Length(log.VelocityWS().lin);
				log_stream() << std::format("coefficient={:g}: ball speed={:g} sleeping={} travel={:g}; capsule speed={:g} sleeping={} travel={:g}\n",
					coefficient, ball_speed, ball.Sleeping(), ball.O2W().pos.y, log_speed, log.Sleeping(), log.O2W().pos.y);
				if (coefficient == 0)
				{
					PR_EXPECT(ball_speed > 1.0f);
					PR_EXPECT(log_speed > 1.0f);
				}
				else
				{
					PR_EXPECT(ball.Sleeping() && ball_speed < 0.01f);
					PR_EXPECT(log.Sleeping() && log_speed < 0.01f);
				}
			}
		}

		// A round body rests on a slope whose tangent is below the rolling resistance coefficient, and still rolls down a steeper slope.
		PRUnitTestMethod(SlopeThreshold, Extended)
		{
			// tan(4 deg) = 0.07 and tan(12 deg) = 0.21, either side of the 0.1 coefficient. The 4 degree case without resistance shows the slope does roll.
			auto const cases = std::array{std::pair{4.0f, 0.1f}, std::pair{12.0f, 0.1f}, std::pair{4.0f, 0.0f}};
			for (auto const& [slope_degrees, coefficient] : cases)
			{
				auto& engine = RollingResistanceEngine(coefficient, 0.0f);
				auto const angle = DegreesToRadians(slope_degrees);
				auto const normal = v4{-Sin(angle), 0, Cos(angle), 0};
				auto const uphill = v4{Cos(angle), 0, Sin(angle), 0};
				auto sphere = collision::ShapeSphere{0.5f};
				auto ground_shape = collision::ShapeBox{v4{100, 100, 2, 0}};
				auto ball = RigidBody{&sphere, m4x4::Translation(v4::Origin() + 0.495f * normal), Inertia::Sphere(0.5f, 1)};
				auto ground = RigidBody{&ground_shape, m4x4{uphill, v4::YAxis(), normal, v4::Origin() - normal}, Inertia::Infinite()};
				auto bodies = std::array<RigidBody*, 2>{&ball, &ground};
				auto const start = ball.O2W().pos;
				for (int tick = 0; tick != 120; ++tick)
				{
					ball.GravityWS(v4{0, 0, -9.81f, 0});
					engine.Step(Engine::StepInput{.m_bodies = bodies, .m_elapsed_seconds = 1.0f / 60, .m_substep_count = 4, .m_time_s = tick / 60.0});
				}

				// Free rolling covers about 1 m in 2 s on the 4 degree slope and about 1.5 m on the 12 degree slope with resistance
				auto const downhill_travel = -Dot3(ball.O2W().pos - start, uphill);
				auto const holds = coefficient > 0 && Tan(angle) < coefficient;
				log_stream() << std::format("slope={:g} coefficient={:g}: downhill travel={:g} speed={:g}\n", slope_degrees, coefficient, downhill_travel, Length(ball.VelocityWS().lin));
				PR_EXPECT(holds ? Abs(downhill_travel) < 0.05f : downhill_travel > 0.5f);
			}
		}

		// Spin resistance must stop a ball spinning in place about the contact normal.
		PRUnitTestMethod(SpinDecays, Extended)
		{
			for (auto const coefficient : {0.0f, 0.1f})
			{
				// Spin resistance of 0.1 on a 0.5 m ball gives an angular deceleration of 0.1 * g * r / (0.4 * r^2), about 4.9 rad/s^2
				auto& engine = RollingResistanceEngine(0.0f, coefficient);
				auto sphere = collision::ShapeSphere{0.5f};
				auto ground_shape = collision::ShapeBox{v4{100, 100, 2, 0}};
				auto ball = RigidBody{&sphere, m4x4::Translation(0, 0, 0.495f), Inertia::Sphere(0.5f, 1)};
				auto ground = RigidBody{&ground_shape, m4x4::Translation(0, 0, -1), Inertia::Infinite()};
				ball.VelocityWS(v4{0, 0, 10, 0}, v4::Zero());
				auto bodies = std::array<RigidBody*, 2>{&ball, &ground};
				for (int tick = 0; tick != 180; ++tick)
				{
					ball.GravityWS(v4{0, 0, -9.81f, 0});
					engine.Step(Engine::StepInput{.m_bodies = bodies, .m_elapsed_seconds = 1.0f / 60, .m_substep_count = 4, .m_time_s = tick / 60.0});
				}

				auto const spin = Abs(ball.VelocityWS().ang.z);
				log_stream() << std::format("spin coefficient={:g}: spin after 3 s={:g}\n", coefficient, spin);
				if (coefficient > 0)
					PR_EXPECT(spin < 0.1f);
			}
		}

		// Rolling resistance must not stop a tall box that is well past its tipping point from falling over.
		PRUnitTestMethod(TallBoxStillTips, Extended)
		{
			for (auto const coefficient : {0.0f, 0.05f})
			{
				// A 0.4 x 0.4 x 2 box tilted 25 degrees about X rests on one bottom edge, with its centre of mass well outside that edge
				auto& engine = RollingResistanceEngine(coefficient, coefficient);
				auto const angle = DegreesToRadians(25.0f);
				auto box = collision::ShapeBox{v4{0.4f, 0.4f, 2.0f, 0}};
				auto ground_shape = collision::ShapeBox{v4{100, 100, 2, 0}};
				auto body = RigidBody{&box, m4x4::Transform(v4::XAxis(), angle, v4{0, 0, Cos(angle) + 0.2f * Sin(angle) + 0.002f, 1}), Inertia::Box(box.m_radius, 10)};
				auto ground = RigidBody{&ground_shape, m4x4::Translation(0, 0, -1), Inertia::Infinite()};
				auto bodies = std::array<RigidBody*, 2>{&body, &ground};
				auto fall_tick = -1;
				for (int tick = 0; tick != 180; ++tick)
				{
					body.GravityWS(v4{0, 0, -9.81f, 0});
					engine.Step(Engine::StepInput{.m_bodies = bodies, .m_elapsed_seconds = 1.0f / 60, .m_substep_count = 4, .m_time_s = tick / 60.0});
					if (fall_tick < 0 && Abs(body.O2W().z.z) < 0.3f)
						fall_tick = tick;
				}

				// The box's long axis ends close to horizontal
				log_stream() << std::format("tipping coefficient={:g}: fall tick={} final axis z={:g}\n", coefficient, fall_tick, body.O2W().z.z);
				PR_EXPECT(fall_tick >= 0);
				PR_EXPECT(Abs(body.O2W().z.z) < 0.3f);
			}
		}
	};

	// A bounded horizontal velocity motor must carry a sliding capsule across a slope in every direction.
	PRUnitTestClass(SlopeVelocityMotorTests)
	{
		// The motor force far exceeds friction plus the downhill weight component, so each drive direction reaches its target on shallow and steep slopes.
		// The result must not depend on the solver iteration count, because extra iterations must not build up extra contact impulse.
		PRUnitTestMethod(UprightCapsuleReachesDriveVelocity, Extended)
		{
			auto& engine = SharedEngine();
			ResetEngineForNextTest(engine);
			engine.Terrain(std::nullopt);
			engine.CylindricalBoundary(std::nullopt);

			// An upright capsule with its lower cap at the body origin, matching a walking character.
			auto const radius = 0.25f;
			auto const height = 0.8f;
			auto const mass = 90.0f;
			auto const com_height = 0.5f * height;
			auto capsule = collision::ShapeLine{height - 2 * radius, radius, m4x4::Translation(0, 0, com_height)};
			auto const directions = std::array{v4{+1, 0, 0, 0}, v4{0, +1, 0, 0}, v4{-1, 0, 0, 0}, Normalise(v4{1, 1, 0, 0})};
			auto const cases = std::array{std::pair{13.0f, 8}, std::pair{13.0f, 32}, std::pair{35.0f, 8}};
			for (auto const& [slope_degrees, solver_iterations] : cases)
			{
				// A static box tilted about the world Y axis forms the slope, rising towards +X.
				auto const angle = DegreesToRadians(slope_degrees);
				auto const normal = v4{-Sin(angle), 0, Cos(angle), 0};
				auto const uphill = v4{Cos(angle), 0, Sin(angle), 0};
				auto ground_shape = collision::ShapeBox{v4{200, 200, 2, 0}};
				auto ground = RigidBody{&ground_shape, m4x4{uphill, v4::YAxis(), normal, v4::Origin() - normal}, Inertia::Infinite()};

				for (auto const speed : {1.5f, 2.5f})
				{
					for (auto const& direction : directions)
					{
						// Reset the engine with the iteration count under test.
						ResetEngineForNextTest(engine);
						engine.Material(Material{.m_friction_static = 0.3f, .m_elasticity_norm = 0.05f, .m_elasticity_tang = 0, .m_elasticity_tors = 0, .m_density = 1});
						auto config = EngineConfig{};
						config.solver_iterations = solver_iterations;
						engine.Config(config);

						// Rest the lower cap on the slope at the origin, upright, with no initial motion.
						auto body = RigidBody{};
						body.Shape(collision::shape_cast(&capsule), mass, false);
						body.O2W(m4x4::Translation(v4::Origin() + radius * normal - v4{0, 0, radius, 0}));
						auto bodies = std::array<RigidBody*, 2>{&body, &ground};

						// Posture constraint: world X/Y velocity motors at the centre of mass, free vertical motion, locked tilt and driven yaw.
						auto desc = D6ConstraintDesc{};
						desc.m_frame_b.m_body = BodyRef::Rigid(body);
						desc.m_frame_b = BodyFrame::FromMatrix(desc.m_frame_b.m_body, m4x4::Translation(0, 0, com_height));
						for (int axis = 0; axis != 2; ++axis)
						{
							desc.m_linear[axis].m_mode = EConstraintAxisMode::Driven;
							desc.m_linear[axis].m_target_velocity = speed * direction[axis];
							desc.m_linear[axis].m_max_force = 3000.0f;
						}
						desc.m_linear[2].m_mode = EConstraintAxisMode::Free;
						desc.m_angular[0].m_mode = EConstraintAxisMode::Locked;
						desc.m_angular[1].m_mode = EConstraintAxisMode::Locked;
						desc.m_angular[2].m_mode = EConstraintAxisMode::Driven;
						desc.m_angular[2].m_max_force = 400.0f;
						auto constraints = ConstraintSet{};
						constraints.Add(desc);

						// Measure the steady horizontal velocity error after the motor has had time to accelerate the body.
						auto max_error = 0.0f;
						auto min_speed = std::numeric_limits<float>::infinity();
						for (int tick = 0; tick != 90; ++tick)
						{
							body.GravityWS(v4{0, 0, -9.81f, 0});
							engine.Step(Engine::StepInput{.m_bodies = bodies, .m_constraints = &constraints, .m_elapsed_seconds = 1.0f / 60, .m_substep_count = 4, .m_time_s = tick / 60.0});
							auto const velocity = body.VelocityWS().lin;
							auto const horizontal = v4{velocity.x, velocity.y, 0, 0};
							if (tick >= 30)
							{
								max_error = Max(max_error, Length(horizontal - speed * direction));
								min_speed = Min(min_speed, Length(horizontal));
							}
						}
						log_stream() << std::format("slope motor slope={:g} iterations={} dir=({:g},{:g}) speed={:g}: max error={:.4f} min speed={:.4f}\n", slope_degrees, solver_iterations, direction.x, direction.y, speed, max_error, min_speed);
						PR_EXPECT(max_error < 0.05f);
					}
				}
			}
		}
	};
}
#endif
