//*********************************************
// Physics Engine Water Tests
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Engine-owned water forces. See engine_water.h.
#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/physics/physics.h"
#include "pr/physics/terrain/water/wave_spectrum.h"
#include "pr/physics/terrain/water/wind_sea.h"
#include "src/buoyancy/gpu_water.h"
#include "src/unittests/shared_engine.h"

namespace pr::physics::tests
{
	PRUnitTestClass(EngineWaterTests)
	{
		static constexpr float Gravity = 9.8f;
		static constexpr float Density = 1000.0f;
		static constexpr float FrameTime = 1.0f / 60.0f;

		// A still, flat water surface with the default force model.
		static WaterConfig FlatWater(double level)
		{
			// Only the surface differs from the defaults.
			return WaterConfig{
				.m_field = terrain::water::WaterField{ level },
				.m_density = Density,
				.m_gravity = Gravity,
			};
		}

		// Water that applies only pressure forces and, optionally, added mass. Drag and wave damping are off so analytic results hold.
		static WaterConfig PressureOnly(terrain::water::WaterField field, float added_mass_coefficient)
		{
			// Non-positive time-constants and zero coefficients disable their terms.
			return WaterConfig{
				.m_field = std::move(field),
				.m_density = Density,
				.m_gravity = Gravity,
				.m_linear_drag_time_constant_s = 0.0f,
				.m_angular_drag_time_constant_s = 0.0f,
				.m_quadratic_drag_coefficient = 0.0f,
				.m_tangential_drag_coefficient = 0.0f,
				.m_added_mass_coefficient = added_mass_coefficient,
				.m_radiation_damping = 0.0f,
				.m_wave_resistance = 0.0f,
			};
		}

		// Make the water hull of a free-standing body.
		static GpuWater::Hull HullOf(RigidBody const& body)
		{
			// A free-standing awake body with a shape always has a hull.
			auto hull = GpuWater::Hull{};
			PR_EXPECT(GpuWater::MakeHull(body, 0, hull));
			return hull;
		}

		// Step 'bodies' for 'frames' frames of 1/60 s with 'substeps' internal substeps, starting at frame 'first_frame', with each body under 'gravity_ws'.
		static void Run(Engine& engine, std::span<RigidBody*> bodies, int frames, int substeps, int first_frame = 0, v4 gravity_ws = v4{ 0, 0, -Gravity, 0 })
		{
			// Gravity persists on the bodies, so each frame only submits the step.
			for (auto* body : bodies)
				body->GravityWS(gravity_ws);

			for (int i = first_frame; i != first_frame + frames; ++i)
			{
				engine.BeginStep(Engine::StepInput{
					.m_bodies = bodies,
					.m_elapsed_seconds = FrameTime,
					.m_substep_count = substeps,
					.m_time_s = i * double(FrameTime),
				});
				engine.CompleteStep();
			}
		}
		static void Run(Engine& engine, RigidBody& body, int frames, int substeps, int first_frame = 0, v4 gravity_ws = v4{ 0, 0, -Gravity, 0 })
		{
			// Forward a single body as a one-element span.
			auto* bodies = &body;
			Run(engine, std::span<RigidBody*>{ &bodies, 1 }, frames, substeps, first_frame, gravity_ws);
		}

		// Remove the water from the shared engine without leaving its wake request pending for the next test.
		static void ClearWater(Engine& engine)
		{
			// Removing water wakes the bodies of the next nonempty step. Consume that wake with a throwaway body so a later test's
			// deliberately sleeping bodies are not woken by this test's environment change.
			engine.Water(std::nullopt);
			auto sphere = collision::ShapeSphere(0.5f);
			auto body = RigidBody{};
			body.Shape(collision::shape_cast(&sphere), 1.0f);
			Run(engine, body, 1, 1);
		}

		// Prepare the shared engine for a test that needs bodies to stay awake.
		static Engine& AwakeEngine()
		{
			// Sleeping would freeze bodies whose motion the test measures.
			auto& engine = SharedEngine();
			ResetEngineForNextTest(engine);
			auto config = engine.Config();
			config.sleeping_enabled = false;
			engine.Config(config);
			return engine;
		}

		// Return the depth below the centre of a floating sphere's displaced spherical cap, found by bisection.
		static float SphereDraft(float radius, float mass, float density)
		{
			// The cap of depth d holds pi*d^2*(3r - d)/3 of water, which must weigh as much as the sphere.
			auto const displaced = mass / density;
			auto lo = 0.0f;
			auto hi = 2.0f * radius;
			for (int i = 0; i != 60; ++i)
			{
				auto const d = 0.5f * (lo + hi);
				auto const cap = (constants<float>::tau / 2.0f) * d * d * (3.0f * radius - d) / 3.0f;
				(cap < displaced ? lo : hi) = d;
			}
			return 0.5f * (lo + hi);
		}

		PRUnitTestMethod(ConfigValidation, Quick)
		{
			// Densities and coefficients must be non-negative and finite, time-constants finite, and the spacing valid.
			auto config = FlatWater(0.0);
			config.Validate();

			auto bad = config;
			bad.m_density = -1.0f;
			PR_THROWS(bad.Validate(), std::invalid_argument);

			// The water's gravity must be finite and positive.
			for (auto gravity : { 0.0f, -Gravity, float(NAN), float(INFINITY) })
			{
				// Each bad gravity value is rejected.
				bad = config;
				bad.m_gravity = gravity;
				PR_THROWS(bad.Validate(), std::invalid_argument);
			}
			bad = config;
			bad.m_linear_drag_time_constant_s = NAN;
			PR_THROWS(bad.Validate(), std::invalid_argument);

			bad = config;
			bad.m_quadratic_drag_coefficient = NAN;
			PR_THROWS(bad.Validate(), std::invalid_argument);

			bad = config;
			bad.m_radiation_damping = -1.0f;
			PR_THROWS(bad.Validate(), std::invalid_argument);

			bad = config;
			bad.m_surface_spacing = 0.0f;
			PR_THROWS(bad.Validate(), std::exception);

			// Invalid configurations are rejected without changing the engine.
			auto& engine = SharedEngine();
			ResetEngineForNextTest(engine);
			bad = config;
			bad.m_added_mass_coefficient = -0.1f;
			PR_THROWS(engine.Water(bad), std::invalid_argument);
			PR_EXPECT(engine.Water() == nullptr);

			engine.Water(config);
			PR_EXPECT(engine.Water() != nullptr);
			ClearWater(engine);
			PR_EXPECT(engine.Water() == nullptr);
		}

		PRUnitTestMethod(DryCullIsConservative, Quick)
		{
			// A resting body well above the water is culled, one reaching the surface within the frame is not.
			auto sphere = collision::ShapeSphere(0.5f);
			auto body = RigidBody{};
			body.Shape(collision::shape_cast(&sphere), 1.0f);
			body.GravityWS(v4{ 0, 0, -Gravity, 0 });
			auto const still = terrain::water::WaterField{ 0.0 };

			body.O2W(m4x4::Translation(0, 0, 10));
			PR_EXPECT(!GpuWater::MayBeWet(HullOf(body), still, FrameTime));
			body.O2W(m4x4::Translation(0, 0, 0.51f));
			PR_EXPECT(GpuWater::MayBeWet(HullOf(body), still, FrameTime));
			body.O2W(m4x4::Translation(0, 0, 0.4f));
			PR_EXPECT(GpuWater::MayBeWet(HullOf(body), still, FrameTime));

			// A fast fall can reach the water within the frame.
			body.O2W(m4x4::Translation(0, 0, 2.0f));
			body.VelocityWS(v4::Zero(), v4{ 0, 0, -120.0f, 0 });
			PR_EXPECT(GpuWater::MayBeWet(HullOf(body), still, FrameTime));
		}

		PRUnitTestMethod(DryLandCullsBelowWaveCrests, Quick)
		{
			// Terrain at +5 on the east half and -50 on the west half, with waves that reach 0.9 m over deep water.
			auto const heights = std::array{ -50.0f, 5.0f, 5.0f, -50.0f, 5.0f, 5.0f };
			auto const bathymetry = std::make_shared<terrain::water::Bathymetry const>(terrain::v2d{ -100.0, -100.0 }, 100.0, 3, 2, heights);
			auto const wave = terrain::water::GerstnerWave(v2{ 1.0f, 0.0f }, 0.9f, 40.0f, 7.9f, 0.0f);
			auto field = terrain::water::WaterField{ 0.0, std::span{ &wave, 1 } };
			field.TerrainHeights(bathymetry);

			// A resting body 1.2 m above the still level is below the deep-water crest bound, but not over dry land.
			auto sphere = collision::ShapeSphere(0.5f);
			auto body = RigidBody{};
			body.Shape(collision::shape_cast(&sphere), 1.0f);
			body.GravityWS(v4{ 0, 0, -Gravity, 0 });
			body.O2W(m4x4::Translation(-100, 0, 1.2f));
			PR_EXPECT(GpuWater::MayBeWet(HullOf(body), field, FrameTime));
			body.O2W(m4x4::Translation(50, 0, 1.2f));
			PR_EXPECT(!GpuWater::MayBeWet(HullOf(body), field, FrameTime));
		}

		PRUnitTestMethod(WavesWakeSleepersInSurfaceBand, Quick)
		{
			// Sleeping bodies inside the band the waves can reach wake; ones well above or below it keep sleeping.
			auto sphere = collision::ShapeSphere(0.5f);
			auto in_band = RigidBody{};
			in_band.Shape(collision::shape_cast(&sphere), 1.0f);
			in_band.O2W(m4x4::Translation(0, 0, 0.3f));
			auto above = RigidBody{};
			above.Shape(collision::shape_cast(&sphere), 1.0f);
			above.O2W(m4x4::Translation(10, 0, 50.0f));
			auto below = RigidBody{};
			below.Shape(collision::shape_cast(&sphere), 1.0f);
			below.O2W(m4x4::Translation(-10, 0, -50.0f));

			// Start from still water and consume its environment-change wake.
			auto& engine = SharedEngine();
			ResetEngineForNextTest(engine);
			engine.Water(FlatWater(0.0));
			auto bodies = std::array<RigidBody*, 3>{ &in_band, &above, &below };
			auto step = [&]
			{
				// One frame without gravity so sleeping state is the only thing that changes.
				engine.BeginStep(Engine::StepInput{
					.m_bodies = std::span<RigidBody*>{ bodies },
					.m_elapsed_seconds = FrameTime,
					.m_substep_count = 1,
					.m_time_s = 0.0,
				});
				engine.CompleteStep();
			};
			step();

			// Replacing only the waves is not an environment change, so the bodies outside the band keep sleeping while the one in it wakes.
			in_band.Sleep();
			above.Sleep();
			below.Sleep();
			auto waves = FlatWater(0.0);
			auto const wave = terrain::water::GerstnerWave(v2{ 1.0f, 0.0f }, 0.5f, 20.0f, 5.6f, 0.0f);
			waves.m_field.Elements(std::span{ &wave, 1 });
			engine.Water(waves);
			step();
			ClearWater(engine);

			PR_EXPECT(!in_band.Sleeping());
			PR_EXPECT(above.Sleeping());
			PR_EXPECT(below.Sleeping());
		}

		PRUnitTestMethod(DryBodyMatchesNoWater, Quick)
		{
			// A body far above the water must follow exactly the same trajectory as without water.
			auto sphere = collision::ShapeSphere(0.5f);
			auto make = [&]
			{
				// Identical starting state for each run.
				auto body = RigidBody{};
				body.Shape(collision::shape_cast(&sphere), 1.0f);
				body.O2W(m4x4::Translation(0, 0, 100));
				body.VelocityWS(v4{ 0.5f, 0, 0, 0 }, v4{ 1, 0, 0, 0 });
				return body;
			};

			auto& engine = SharedEngine();
			ResetEngineForNextTest(engine);
			auto dry = make();
			Run(engine, dry, 10, 4);

			engine.Water(FlatWater(0.0));
			auto wet_world = make();
			Run(engine, wet_world, 10, 4);
			ClearWater(engine);

			PR_EXPECT(All(dry.O2W().pos == wet_world.O2W().pos));
			PR_EXPECT(All(dry.VelocityWS().lin == wet_world.VelocityWS().lin));
		}

		PRUnitTestMethod(SubmergedBoxBuoyancy, Quick)
		{
			// A fully submerged 1 m^3 box of 2000 kg feels a net force of 1000*g. Without added mass it sinks at g/2. Added mass Ca*rho*V
			// joins the box's inertia, so with Ca = 0.5 it sinks at 1000*g / 2500 = 0.4*g.
			auto box = collision::ShapeBox(v4{ 1, 1, 1, 0 });
			auto sink_speed = [&](float added_mass_coefficient)
			{
				// One second of sinking from rest, far below the surface.
				auto body = RigidBody{};
				body.Shape(collision::shape_cast(&box), 2.0f * Density);
				body.O2W(m4x4::Translation(0, 0, -20));

				auto& engine = AwakeEngine();
				engine.Water(PressureOnly(terrain::water::WaterField{ 0.0 }, added_mass_coefficient));
				Run(engine, body, 60, 4);
				ClearWater(engine);
				return body.VelocityWS().lin.z;
			};
			PR_EXPECT(FEqlRelative(sink_speed(0.0f), -0.5f * Gravity, 0.01f));
			PR_EXPECT(FEqlRelative(sink_speed(0.5f), -0.4f * Gravity, 0.01f));
		}

		PRUnitTestMethod(NeutralBoxStaysPut, Quick)
		{
			// A neutrally buoyant submerged box feels no net force.
			auto box = collision::ShapeBox(v4{ 1, 1, 1, 0 });
			auto body = RigidBody{};
			body.Shape(collision::shape_cast(&box), Density);
			body.O2W(m4x4::Translation(0, 0, -5));

			auto& engine = AwakeEngine();
			engine.Water(FlatWater(0.0));
			Run(engine, body, 60, 4);
			ClearWater(engine);

			PR_EXPECT(FEqlAbsolute(body.O2W().pos.z, -5.0f, 1e-3f));
			PR_EXPECT(FEqlAbsolute(body.VelocityWS().lin.z, 0.0f, 1e-3f));
		}

		PRUnitTestMethod(TiltedBodyGravityDoesNotMoveWaterSurface, Quick)
		{
			// A body whose own gravity is tilted 30 degrees from -Z, held just above the water and far from the origin, is dry.
			// The water surface is a height field along world Z, so the body's gravity must not change which points are under water.
			auto const tilt = 30.0f * constants<float>::tau / 360.0f;
			auto const gravity_ws = Gravity * v4{ 0, std::sin(tilt), -std::cos(tilt), 0 };
			auto sphere = collision::ShapeSphere(0.5f);
			auto make = [&]
			{
				// The sphere's lowest point is 0.1 m above the water, close enough that the host cull still sends it to the GPU.
				auto body = RigidBody{};
				body.Shape(collision::shape_cast(&sphere), 1.0f);
				body.O2W(m4x4::Translation(4, 299, -99.4f));
				return body;
			};

			// Run the same body without water and with water, recording the water force on the first frame.
			auto& engine = AwakeEngine();
			auto dry = make();
			Run(engine, dry, 1, 1, 0, gravity_ws);

			auto config = FlatWater(-100.0);
			config.m_enable_diagnostics = true;
			engine.Water(config);
			auto wet_world = make();
			PR_EXPECT(GpuWater::MayBeWet(HullOf(wet_world), config.m_field, FrameTime));
			Run(engine, wet_world, 1, 1, 0, gravity_ws);
			auto const* diag = engine.WaterDiagnosticsFor(wet_world);
			auto const volume = diag != nullptr ? diag->m_volume_m3 : 0.0f;
			auto const force = diag != nullptr ? diag->m_force_ws : v4::Zero();
			ClearWater(engine);

			// No water acts on the body, so it moves exactly as it does without water.
			PR_EXPECT(volume == 0.0f);
			PR_EXPECT(All(force == v4::Zero()));
			PR_EXPECT(All(dry.O2W().pos == wet_world.O2W().pos));
			PR_EXPECT(All(dry.VelocityWS().lin == wet_world.VelocityWS().lin));
		}

		PRUnitTestMethod(BuoyancyUsesWaterGravity, Quick)
		{
			// A fully submerged 1 m^3 box whose own gravity is tilted feels buoyancy of density * water gravity * volume along world +Z.
			// The water's gravity differs from the body's gravity, so the force shows which one the water uses.
			auto const water_gravity = 5.0f;
			auto const tilt = 30.0f * constants<float>::tau / 360.0f;
			auto const gravity_ws = Gravity * v4{ 0, std::sin(tilt), -std::cos(tilt), 0 };
			auto box = collision::ShapeBox(v4{ 1, 1, 1, 0 });
			auto body = RigidBody{};
			body.Shape(collision::shape_cast(&box), Density);
			body.O2W(m4x4::Translation(0, 0, -20));

			// One frame from rest, with only pressure forces so the result is the analytic buoyancy.
			auto& engine = AwakeEngine();
			auto config = PressureOnly(terrain::water::WaterField{ 0.0 }, 0.0f);
			config.m_gravity = water_gravity;
			config.m_enable_diagnostics = true;
			engine.Water(config);
			Run(engine, body, 1, 1, 0, gravity_ws);
			auto const* diag = engine.WaterDiagnosticsFor(body);
			PR_EXPECT(diag != nullptr);
			auto const force = diag != nullptr ? diag->m_force_ws : v4::Zero();
			auto const volume = diag != nullptr ? diag->m_volume_m3 : 0.0f;
			ClearWater(engine);

			PR_EXPECT(FEqlRelative(volume, 1.0f, 1e-3f));
			PR_EXPECT(FEqlRelative(force, v4{ 0, 0, Density * water_gravity, 0 }, 1e-3f));
		}

		PRUnitTestMethod(DeepBodyDampingIsDepthIndependent, Quick)
		{
			// A buoyant capsule (Petri's Actor) released spinning and tilted far below the surface rises steadily while its spin decays.
			// Its motion does not depend on the depth of water above it, because a fully submerged body's volume terms come from its own volume.
			auto capsule = collision::ShapeLine(0.55f, 0.125f);
			auto rise = [&](float depth)
			{
				// Three seconds of rising, with the drag settings used by Petri.
				auto body = RigidBody{};
				body.Shape(collision::shape_cast(&capsule), 25.0f);
				body.O2W(m4x4::Transform(v4::XAxis(), 0.5f, v4{ 0, 0, -depth, 1 }));
				body.VelocityWS(v4{ 3.0f, 0, 0, 0 }, v4{ 0, 2.0f, 0.5f, 0 });

				auto& engine = AwakeEngine();
				auto water = FlatWater(0.0);
				water.m_density = 1025.0f;
				water.m_linear_drag_time_constant_s = 0.5f;
				water.m_angular_drag_time_constant_s = 0.5f;
				water.m_quadratic_drag_coefficient = 0.5f;
				engine.Water(water);

				auto max_spin = 0.0f;
				auto max_speed = 0.0f;
				for (int i = 0; i != 180; ++i)
				{
					Run(engine, body, 1, 1, i);
					max_spin = std::max(max_spin, Length(body.VelocityWS().ang));
					max_speed = std::max(max_speed, Length(body.VelocityWS().lin));
				}
				ClearWater(engine);
				return std::tuple{ body.VelocityWS(), max_spin, max_speed };
			};
			auto [vel40, max_spin40, max_speed40] = rise(40.0f);
			auto [vel100, max_spin100, max_speed100] = rise(100.0f);

			// The spin only decays, the rise stays bounded, and the depth makes no difference.
			PR_EXPECT(max_spin40 <= 3.0f + 1e-3f);
			PR_EXPECT(Length(vel40.ang) < 0.3f);
			PR_EXPECT(max_speed40 < 2.5f);
			PR_EXPECT(FEqlAbsolute(max_spin40, max_spin100, 1e-2f));
			PR_EXPECT(FEqlAbsolute(max_speed40, max_speed100, 1e-2f));
			PR_EXPECT(FEqlAbsolute(vel40.ang, vel100.ang, 1e-2f));
			PR_EXPECT(FEqlAbsolute(vel40.lin, vel100.lin, 1e-2f));
		}

		PRUnitTestMethod(SphereFloatsAtAnalyticDraft, Quick)
		{
			// A sphere of half the water's density settles with its centre on the surface. Wave-radiation damping settles the bob without tuned drag.
			auto const radius = 0.5f;
			auto const volume = 2.0f / 3.0f * constants<float>::tau * radius * radius * radius;
			auto sphere = collision::ShapeSphere(radius);
			auto body = RigidBody{};
			body.Shape(collision::shape_cast(&sphere), 0.5f * Density * volume);
			body.O2W(m4x4::Translation(0, 0, 2.4f));

			auto& engine = AwakeEngine();
			engine.Water(FlatWater(2.0));
			Run(engine, body, 300, 4);
			ClearWater(engine);

			PR_EXPECT(FEqlAbsolute(body.O2W().pos.z, 2.0f, 0.01f));
			PR_EXPECT(FEqlAbsolute(body.VelocityWS().lin.z, 0.0f, 0.01f));
		}

		PRUnitTestMethod(DragNeverReversesMotion, Quick)
		{
			// Very strong drag stops relative motion without overshooting within one frame.
			auto box = collision::ShapeBox(v4{ 1, 1, 1, 0 });
			auto body = RigidBody{};
			body.Shape(collision::shape_cast(&box), Density);
			body.O2W(m4x4::Translation(0, 0, -5));
			body.VelocityWS(v4{ 0, 0, 5, 0 }, v4{ 20, 0, 0, 0 });

			auto& engine = AwakeEngine();
			auto config = FlatWater(0.0);
			config.m_linear_drag_time_constant_s = 1e-4f;
			config.m_angular_drag_time_constant_s = 1e-4f;
			config.m_quadratic_drag_coefficient = 1000.0f;
			config.m_tangential_drag_coefficient = 1000.0f;
			engine.Water(config);
			Run(engine, body, 1, 1);
			ClearWater(engine);

			auto const velocity = body.VelocityWS();
			PR_EXPECT(velocity.lin.x >= 0.0f && velocity.lin.x < 20.0f);
			PR_EXPECT(velocity.ang.z >= 0.0f && velocity.ang.z < 5.0f);
		}

		// A light hollow beach ball spinning at 20 rad/s about a horizontal axis is dropped 0.85 m onto calm sea water. The water must catch it
		// rather than throw it back up, and the wetted surface must take away most of its spin within a second or two. Water is not a collision
		// contact, so the ball's restitution plays no part in this.
		PRUnitTestMethod(SpinningBeachBallSettles, Quick)
		{
			auto const radius = 0.15f;
			auto const mass = 0.06f;
			auto const sea_density = 1025.0f;
			auto sphere = collision::ShapeSphere(radius);
			auto body = RigidBody{};
			body.Shape(collision::shape_cast(&sphere), mass, false, EMassDistribution::Shell);
			body.O2W(m4x4::Translation(0, 0, 1.0f));
			body.VelocityWS(v4{ 14.14f, 14.14f, 0, 0 }, v4::Zero());

			auto& engine = AwakeEngine();
			auto config = FlatWater(0.0);
			config.m_density = sea_density;
			engine.Water(config);

			// Record the lowest point after entry, the highest point after that, and the spin at one and five seconds.
			auto lowest = 1.0f;
			auto rebound = -1.0f;
			auto spin_1s = 0.0f;
			auto spin_5s = 0.0f;
			for (int i = 0; i != 300; ++i)
			{
				// Track the trough, then the highest point reached after it.
				Run(engine, body, 1, 1, i);
				auto const z = body.O2W().pos.z;
				if (z < lowest)
				{
					lowest = z;
					rebound = z;
				}
				rebound = std::max(rebound, z);

				if (i == 59)
					spin_1s = Length(body.VelocityWS().ang);
				if (i == 299)
					spin_5s = Length(body.VelocityWS().ang);
			}
			ClearWater(engine);

			// The ball floats at the analytic height with no rebound worth the name.
			auto const rest_z = radius - SphereDraft(radius, mass, sea_density);
			PR_EXPECT(lowest < rest_z);
			PR_EXPECT(rebound < rest_z + 0.03f);
			PR_EXPECT(FEqlAbsolute(body.O2W().pos.z, rest_z, 0.005f));
			PR_EXPECT(std::abs(body.VelocityWS().lin.z) < 0.02f);

			// The spin is mostly gone after a second and keeps falling. The residual spin is small enough that it can rise briefly while the ball
			// settles, so the decay is checked over several seconds.
			PR_EXPECT(spin_1s < 1.0f);
			PR_EXPECT(spin_5s < 0.8f * spin_1s);
		}

		// Water pressure acts along the surface normal, so on a sphere it passes through the centre and cannot make the sphere spin, even when a
		// steep wave tilts the surface across it.
		PRUnitTestMethod(PressureDoesNotTorqueSphereInWaves, Quick)
		{
			auto const radius = 0.5f;
			auto const volume = 2.0f / 3.0f * constants<float>::tau * radius * radius * radius;
			auto sphere = collision::ShapeSphere(radius);
			auto body = RigidBody{};
			body.Shape(collision::shape_cast(&sphere), 0.5f * Density * volume);

			auto& engine = AwakeEngine();
			auto const wave = terrain::water::GerstnerWave(v2{ 1.0f, 0.0f }, 0.4f, 6.0f, 3.2f, 0.8f);
			auto config = PressureOnly(terrain::water::WaterField{ 0.0, std::span{ &wave, 1 } }, 0.0f);
			config.m_enable_diagnostics = true;
			engine.Water(config);

			// Sample the torque across a wave period, with the sphere half in the water at different points on the wave.
			auto const scale = Density * Gravity * volume * radius;
			for (int i = 0; i != 12; ++i)
			{
				// Reset the sphere each frame so it stays where the wave surface is steepest.
				body.O2W(m4x4::Translation(0, 0, 0));
				body.VelocityWS(v4::Zero(), v4::Zero());
				Run(engine, body, 1, 1, i * 10);

				auto const* diag = engine.WaterDiagnosticsFor(body);
				PR_EXPECT(diag != nullptr);
				if (diag == nullptr)
					break;

				PR_EXPECT(diag->m_volume_m3 > 0.0f);
				PR_EXPECT(Length(diag->m_torque_ws) < 1e-3f * scale);
			}
			ClearWater(engine);
			PR_EXPECT(Length(body.VelocityWS().ang) < 1e-3f);
		}

		// Return the velocity of a neutrally buoyant submerged box after one second in 'field', moved only by water pressure and inertia.
		static v4 WaveDrivenVelocity(terrain::water::WaterField const& field, v4 position)
		{
			// One second of motion from rest. Drag is off so the pressure field alone moves the box.
			auto box = collision::ShapeBox(v4{ 2, 2, 1, 0 });
			auto body = RigidBody{};
			body.Shape(collision::shape_cast(&box), 4.0f * Density);
			body.O2W(m4x4::Translation(position));

			auto& engine = AwakeEngine();
			engine.Water(PressureOnly(field, 0.5f));
			Run(engine, body, 60, 1);
			ClearWater(engine);
			return body.VelocityWS().lin;
		}

		// Return 'elements' as plain waves, with their wind-sea band cleared.
		static std::vector<terrain::water::WaterFieldElement> Plain(std::span<terrain::water::WaterFieldElement const> elements)
		{
			// A cleared band makes each element a stored wave rather than a wind-sea member.
			auto plain = std::vector<terrain::water::WaterFieldElement>(elements.begin(), elements.end());
			for (auto& element : plain)
				element.info.y = 0;

			return plain;
		}

		PRUnitTestMethod(WindSeaUsesLocalWaves, Quick)
		{
			using namespace terrain::water;

			// Build the sea for a steady 10 m/s east wind over the origin.
			auto const spectrum = WaveSpectrum{};
			auto const bands = spectrum.Layout().m_bands;
			constexpr int speed_count = 81;
			auto table = std::vector<WindSeaTableRow>(static_cast<size_t>(bands) * speed_count);
			spectrum.SeaTable(10000.0f, 0.5f, table);
			auto elements = std::vector<WaterFieldElement>(static_cast<size_t>(spectrum.ComponentCount()));
			elements.resize(static_cast<size_t>(spectrum.Elements(table, 0.0f, elements)));
			auto const grid = WindSeaGrid{.origin = {-100.0f, -100.0f}, .cell_size = 200.0f, .speed_step = 0.5f, .dims = {2, 2}, .band_count = bands, .speed_count = speed_count, .level_count = bands};
			auto const winds = std::vector<v2>(static_cast<size_t>(4 * bands), v2{10.0f, 0.0f});
			auto const sea = std::make_shared<WindSea const>(grid, table, winds);

			// Keep the element with the largest local amplitude, so its wave force is clear of sampling noise.
			auto buffer = std::array<WaterFieldElement, MaxElementCount>{};
			auto element = WaterFieldElement{};
			auto local = WaterFieldElement{};
			for (auto const& candidate : elements)
			{
				// Localise each element on its own so the local copy is known to belong to it.
				auto field = WaterField(0.0, std::span{ &candidate, 1 });
				field.WindSea(sea);
				auto const locals = field.LocalElements(terrain::v2d{ 0.0, 0.0 }, buffer);
				if (locals.empty() || locals[0].wave.x <= local.wave.x)
					continue;

				element = candidate;
				local = locals[0];
			}
			PR_EXPECT(local.wave.x > 0.05f && local.wave.x < 0.5f * element.wave.x);

			// The wind sea moves the body as the local wave does, and not as the stored bound would.
			auto sea_field = WaterField(0.0, std::span{ &element, 1 });
			sea_field.WindSea(sea);
			auto const position = v4{ 0, 0, -5, 1 };
			auto const sea_velocity = WaveDrivenVelocity(sea_field, position);
			auto const plain_velocity = WaveDrivenVelocity(WaterField(0.0, Plain(std::span{ &local, 1 })), position);
			auto const bound_velocity = WaveDrivenVelocity(WaterField(0.0, Plain(std::span{ &element, 1 })), position);
			auto const separation = Length(bound_velocity - plain_velocity);
			PR_EXPECT(separation > 1e-2f);
			PR_EXPECT(Length(sea_velocity - plain_velocity) < 0.01f * separation);
		}

		PRUnitTestMethod(ShallowWaterUsesDepthCorrectedWaves, Quick)
		{
			using namespace terrain::water;

			// A long swell over a uniform 3 m deep shelf, where shoaling and the shallow-water velocity profile change the wave a lot.
			auto const heights = std::array{ -3.0f, -3.0f, -3.0f, -3.0f };
			auto const bathymetry = std::make_shared<Bathymetry const>(terrain::v2d{ -100.0, -100.0 }, 200.0, 2, 2, heights);
			auto const wave = SineWave(v2{ 1.0f, 0.0f }, 0.5f, 40.0f, 1.25f);
			auto shallow = WaterField(0.0, std::span{ &wave, 1 });
			shallow.TerrainHeights(bathymetry);

			// The depth-corrected wave at the body's position.
			auto buffer = std::array<WaterFieldElement, MaxElementCount>{};
			auto const locals = shallow.LocalElements(terrain::v2d{ 0.0, 0.0 }, buffer);
			PR_EXPECT(locals.size() == 1);
			if (locals.size() != 1)
				return;

			// The shelf moves the body as the corrected wave does, and not as the deep-water wave would.
			auto const position = v4{ 0, 0, -1.2f, 1 };
			auto const shallow_velocity = WaveDrivenVelocity(shallow, position);
			auto const corrected_velocity = WaveDrivenVelocity(WaterField(0.0, std::span{ locals.data(), 1 }), position);
			auto const deep_velocity = WaveDrivenVelocity(WaterField(0.0, std::span{ &wave, 1 }), position);
			auto const separation = Length(deep_velocity - corrected_velocity);
			PR_EXPECT(separation > 1e-2f);
			PR_EXPECT(Length(shallow_velocity - corrected_velocity) < 0.01f * separation);
		}
	};
}
#endif
