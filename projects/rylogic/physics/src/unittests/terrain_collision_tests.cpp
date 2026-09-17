//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#if PR_UNITTESTS
#include "src/unittests/forward.h"
#include "src/unittests/shared_gpu.h"
#include "pr/physics/integrator/engine.h"
#include "pr/physics/rigid_body/rigid_body.h"
#include "pr/physics/materials/material.h"
#include "pr/physics/surface/surface_sampling.h"
#include "pr/physics/shape/shape_builder.h"
#include "pr/physics/articulation/articulation.h"
#include "src/surface/gpu_world_contacts.h"
#include "src/collision/shape_cache.h"

namespace pr::physics::tests
{
	namespace
	{
		// Expose resting-island identity to fixtures without extending the production body API.
		struct SleepObservedBody : RigidBody
		{
			using RigidBody::RigidBody;

			// Return the persistent island assigned by the completed GPU frame.
			int SleepIsland() const
			{
				return m_sleep.m_island_id;
			}
		};

		// Return a canonical terrain source whose spatial bands are disabled and whose height is zero.
		terrain::landscape::BaselineSurface FlatTerrain()
		{
			auto config = terrain::landscape::BaselineSurfaceConfig{};
			config.m_regional_base.m_amplitude = config.m_region_selector.m_amplitude = config.m_region_uplift.m_amplitude = 0;
			config.m_domain_warp.m_amplitude_m = 0;
			config.m_plains.m_amplitude = config.m_hills.m_amplitude = config.m_mountains.m_amplitude = 0;
			config.m_uplift_height_m = config.m_mountain_base_height_m = 0;
			config.m_sea_level_bias_m = 0;

			// Cancel the fixed family datum so the reference plane is exactly at the origin.
			config.m_sea_level_bias_m = -terrain::landscape::BaselineSurface(config).Sample({0, 0}).m_height;
			return terrain::landscape::BaselineSurface(config);
		}

		// Return exact radial extent for the box, sphere and compound fixtures, independently of collision sampling.
		double RadialExtent(Shape const& shape, m4x4 const& root_to_world)
		{
			auto shape_to_world = root_to_world * shape.m_s2r;
			switch (shape.m_type)
			{
				case collision::EShape::Sphere:
				{
					auto const& sphere = shape_cast<collision::ShapeSphere>(shape);
					return std::hypot(double(shape_to_world.pos.x), double(shape_to_world.pos.y)) + sphere.m_radius;
				}
				case collision::EShape::Box:
				{
					auto const& box = shape_cast<collision::ShapeBox>(shape);
					auto extent = 0.0;
					for (int corner = 0; corner != 8; ++corner)
					{
						auto local = v4((corner & 1) ? box.m_radius.x : -box.m_radius.x, (corner & 2) ? box.m_radius.y : -box.m_radius.y, (corner & 4) ? box.m_radius.z : -box.m_radius.z, 1);
						auto point = shape_to_world * local;
						extent = std::max(extent, std::hypot(double(point.x), double(point.y)));
					}
					return extent;
				}
				case collision::EShape::Array:
				{
					auto const& array = shape_cast<collision::ShapeArray>(shape);
					auto extent = 0.0;
					for (auto child = array.begin(); child != array.end(); child = collision::next(child))
						extent = std::max(extent, RadialExtent(*child, root_to_world));

					return extent;
				}
				default: { throw std::runtime_error("Unsupported radial-extent reference fixture"); }
			}
		}
	} // namespace

	// Verify that ordinary rigid contacts use the completed motion to decide whether a sleeper wakes.
	PRUnitTestClass(RigidContactSleepTests)
	{
		// A contact below the existing sleep threshold preserves rest; a faster impact transfers motion and wakes.
		PRUnitTestMethod(ResolvedMotionControlsWake, Extended)
		{
			for (auto fast : {false, true})
			{
				// Isolate the contact from terrain and gravity.
				auto shape = collision::ShapeSphere(0.3f);
				auto body = RigidBody(&shape, m4x4::Translation(0, 0, 0), Inertia::Sphere(0.3f, 1));
				auto impactor = RigidBody(&shape, m4x4::Translation(0.599f, 0, 0), Inertia::Sphere(0.3f, 1));
				auto bodies = std::array{&body, &impactor};
				auto& gpu = SharedTestGpu();
				auto engine = Engine({}, nullptr, gpu, gpu.m_job.m_queue.get());
				engine.Material(Material{.m_friction_static = 0, .m_elasticity_norm = 0});
				body.Sleep();
				engine.UpdateSleepIslands(bodies);
				impactor.VelocityWS(v4::Zero(), v4(fast ? -2.0f : -0.02f, 0, 0, 0));
				engine.Step(1.0f / 240, bodies);

				// Check both sides of the unchanged threshold with a real solver impulse.
				std::printf("sleep_rigid_contact fast=%d sleeping=%d vx=%.6f\n", fast, body.Sleeping(), body.VelocityWS().lin.x);
				PR_EXPECT(body.Sleeping() == !fast);
				if (fast)
					PR_EXPECT(body.VelocityWS().lin.x < -0.25f);

				// Reject invalid state independently of the expected wake decision.
				PR_EXPECT(IsFinite(body.O2W().pos));
			}
		}
	};

	// Verify the infinite cylindrical world boundary through the production GPU solver and transactional readback.
	PRUnitTestClass(CylindricalBoundaryTests)
	{
		// Adding a fine wall plan must not change terrain sample density or rebuild unchanged plans on later frames.
		PRUnitTestMethod(IndependentSamplingDensity, Extended)
		{
			auto& gpu = SharedTestGpu();
			auto shape = collision::ShapeSphere(0.5f);
			auto cache = ShapeCache{};
			auto index = cache.GetOrAdd(shape);
			auto body = RigidBody(&shape, m4x4::Translation(0, 0, 20), Inertia::Sphere(0.5f, 1));
			auto bodies = std::array{PackDynamics(body, index)};
			auto world = GpuWorldContacts(gpu, FlatTerrain(), surface::DefaultSpacing, CylindricalBoundaryConfig{});
			world.Upload(gpu.m_job, cache, bodies, 1);
			gpu.m_job.Run();
			auto terrain_count = world.m_ranges[index].m_sample_count;
			auto wall_count = world.m_ranges[index + world.m_boundary_plan_offset].m_sample_count;
			PR_EXPECT(terrain_count == surface::BuildPlan(shape, 0.16f).m_count);
			PR_EXPECT(wall_count == surface::BuildPlan(shape, 0.05f).m_count);
			PR_EXPECT(terrain_count < wall_count);
			auto plans = world.m_plans.get();
			auto patches = world.m_patch_buffer.get();
			auto patch_data = world.m_patches.data();

			// Only the dynamic instance stream changes on an otherwise unchanged frame.
			cache.m_changed = false;
			body.O2W(m4x4::Translation(10, 0, 20));
			bodies[0] = PackDynamics(body, index);
			world.Upload(gpu.m_job, cache, bodies, 1);
			gpu.m_job.Run();
			PR_EXPECT(world.m_plans.get() == plans && world.m_patch_buffer.get() == patches && world.m_patches.data() == patch_data);
			auto terrain_only = GpuWorldContacts(gpu, FlatTerrain(), surface::DefaultSpacing);
			terrain_only.Upload(gpu.m_job, cache, bodies, 1);
			gpu.m_job.Run();
			PR_EXPECT(terrain_only.m_ranges[index].m_sample_count == terrain_count);
			std::printf("Cylinder density 1m sphere: terrain_spacing=0.16 samples=%u wall_spacing=0.05 samples=%u; cache reused\n", terrain_count, wall_count);
		}

		// Contacts at several azimuths retain the exact radial normal and tangent-plane depth at large coordinates.
		PRUnitTestMethod(AzimuthNormalsDepthAndHeight, Extended)
		{
			auto& gpu = SharedTestGpu();
			auto engine = Engine(EngineConfig{.solver_iterations = 0, .push_out_iterations = 0, .selective_refresh_passes = 0}, nullptr, gpu, gpu.m_job.m_queue.get());
			auto config = CylindricalBoundaryConfig{.m_centre_x = 12, .m_centre_y = -7, .m_material_id = 7};
			engine.CylindricalBoundary(config);
			auto shape = collision::ShapeSphere(0.3f);
			shape.m_base.m_material_id = 3;
			auto body = RigidBody(&shape, m4x4::Identity(), Inertia::Sphere(0.3f, 1));
			auto bodies = std::array{&body};
			for (auto angle : {0.0, 0.7, 1.9, 3.5, 5.4})
			{
				// The height is deliberately far above any plausible visible rim.
				auto radial = v4(static_cast<float>(std::cos(angle)), static_cast<float>(std::sin(angle)), 0, 0);
				body.O2W(m4x4::Translation(v4(12, -7, 5000, 1) + radial * 3999.72f));
				body.VelocityWS(v4::Zero(), v4::Zero());
				auto count = 0;
				auto subscription = engine.Collisions += [&](Engine&, std::span<RbContact const> contacts)
				{
					for (auto const& contact : contacts)
					{
						auto a2w = InvertOrthonormal(contact.m_b2a);
						auto normal = -(a2w * contact.m_axis);
						auto sample = a2w * contact.Point() - normal * (0.5f * contact.m_depth);
						auto dx = double(sample.x) - config.m_centre_x;
						auto dy = double(sample.y) - config.m_centre_y;
						auto distance = std::hypot(dx, dy);
						auto expected = v4(static_cast<float>(-dx / distance), static_cast<float>(-dy / distance), 0, 0);
						PR_EXPECT(Length(normal - expected) < 1e-4f);
						PR_EXPECT(std::abs(contact.m_depth - (distance - config.m_radius)) < 0.002);
						PR_EXPECT(Dot(normal, radial) < -0.999f);
						PR_EXPECT(contact.m_mat_idA == 3 && contact.m_mat_idB == 7);
						++count;
					}
				};
				engine.Step(0.00001f, bodies);
				PR_EXPECT(count > 0);
				PR_EXPECT(engine.LastStepProfile().m_terrain_gpu_ms == 0);
			}
		}

		// Capsules, thin lines, triangles, polytopes and points use physical leaf samples rather than a sphere-only collision path.
		PRUnitTestMethod(PrimitiveLeafContacts, Extended)
		{
			auto capsule = collision::ShapeLine(0.6f, 0.12f);
			auto thin = collision::ShapeLine(0.6f);
			auto point = collision::ShapeSphere(0);
			auto triangle = collision::ShapeTriangle(v4(-0.3f, -0.2f, 0, 1), v4(0.3f, -0.2f, 0, 1), v4(0, 0.3f, 0, 1));
			auto vertices = std::array{v4(-0.3f, -0.2f, -0.2f, 1), v4(0.3f, -0.2f, -0.2f, 1), v4(0, 0.3f, -0.2f, 1), v4(0, 0, 0.3f, 1)};
			auto poly = collision::BuildPolytopeFromPoints(vertices, m4x4::Identity(), 0, Shape::EFlags::None, 0);
			auto shapes = std::array<Shape const*, 5>{&capsule.m_base, &thin.m_base, &point.m_base, &triangle.m_base, &poly.as<collision::ShapePolytope>().m_base};
			auto& gpu = SharedTestGpu();
			auto engine = Engine(EngineConfig{.solver_iterations = 0, .push_out_iterations = 0, .selective_refresh_passes = 0}, nullptr, gpu, gpu.m_job.m_queue.get());
			engine.CylindricalBoundary(CylindricalBoundaryConfig{});
			for (auto shape : shapes)
			{
				// Place a rotated surface slightly across the wall without relying on its loose world bounding box.
				auto transform = m4x4::Transform(v4(0.4f, 0.3f, 0.2f, 0), v4(0, 0, 80, 1));
				auto plan = surface::BuildPlan(*shape, 0.05f);
				auto furthest = -std::numeric_limits<float>::max();
				for (uint32_t ordinal = 0; ordinal != plan.m_count; ++ordinal)
					furthest = std::max(furthest, (transform * shape->m_s2r * surface::EmitSurfaceSample(plan, ordinal).m_pos_local).x);

				// The unchanged solver output preserves contact geometry for the independent normal/depth checks.
				transform.pos.x = 4000 - furthest + 0.01f;
				auto body = RigidBody(shape, transform, Inertia::Sphere(1, 1));
				auto bodies = std::array{&body};
				auto count = 0;
				auto subscription = engine.Collisions += [&](Engine&, std::span<RbContact const> contacts)
				{
					for (auto const& contact : contacts)
					{
						auto normal = -(InvertOrthonormal(contact.m_b2a) * contact.m_axis);
						PR_EXPECT(normal.x < -0.999f);
						PR_EXPECT(contact.m_depth > 0 && contact.m_depth < 0.03f);
						++count;
					}
				};
				engine.Step(0.00001f, bodies);
				PR_EXPECT(count > 0);
				engine.ResetCaches();
			}
		}

		// Sustained radial forcing cannot escape; tangent motion and vertical freefall remain physical solver responses.
		PRUnitTestMethod(RadialContainmentSlidingAndFreefall, Extended)
		{
			auto& gpu = SharedTestGpu();
			auto engine = Engine({}, nullptr, gpu, gpu.m_job.m_queue.get());
			engine.CylindricalBoundary(CylindricalBoundaryConfig{});
			engine.Material(Material{.m_friction_static = 0, .m_elasticity_norm = 0});
			auto shape = collision::ShapeSphere(0.3f);
			auto body = RigidBody(&shape, m4x4::Translation(3999.65f, 0, 100), Inertia::Sphere(0.3f, 1));
			body.VelocityWS(v4::Zero(), v4(1, 1, -100, 0));
			auto bodies = std::array{&body};
			auto peak = 0.0;
			for (int frame = 0; frame != 120; ++frame)
			{
				body.GravityWS(v4(5, 0, -9.81f, 0));
				engine.Step(Engine::StepInput{.m_bodies = bodies, .m_elapsed_seconds = 1.0f / 60, .m_substep_count = 4});
				auto depth = std::hypot(double(body.O2W().pos.x), double(body.O2W().pos.y)) + 0.3 - 4000;
				peak = std::max(peak, depth);
				PR_EXPECT(depth < 0.01);
			}
			std::printf("Cylinder radius=4000 spacing=0.05 peak_sphere_penetration=%.9f tangent=%.6f z=%.6f\n", peak, body.O2W().pos.y, body.O2W().pos.z);
			PR_EXPECT(body.O2W().pos.y > 1);
			PR_EXPECT(body.O2W().pos.z < -90);
		}

		// A one-metre body approaching at 20 m/s stays within the centimetre-scale overlap gate under 240 Hz collision sampling.
		PRUnitTestMethod(FastOneMetreActor, Extended)
		{
			auto& gpu = SharedTestGpu();
			auto engine = Engine({}, nullptr, gpu, gpu.m_job.m_queue.get());
			engine.CylindricalBoundary(CylindricalBoundaryConfig{});
			engine.Material(Material{.m_friction_static = 0, .m_elasticity_norm = 0});
			auto shape = collision::ShapeSphere(0.5f);
			auto body = RigidBody(&shape, m4x4::Translation(3999.45f, 0, 20), Inertia::Sphere(0.5f, 1));
			body.VelocityWS(v4::Zero(), v4(20, 0, 0, 0));
			auto bodies = std::array{&body};
			auto peak = 0.0;
			for (int frame = 0; frame != 120; ++frame)
			{
				body.GravityWS(v4(5, 0, 0, 0));
				engine.Step(Engine::StepInput{.m_bodies = bodies, .m_elapsed_seconds = 1.0f / 240});
				auto depth = std::hypot(double(body.O2W().pos.x), double(body.O2W().pos.y)) + 0.5 - 4000;
				peak = std::max(peak, depth);
				PR_EXPECT(depth < 0.01);
			}
			std::printf("Cylinder 1m actor speed=20m/s dt=1/240 peak_postsubstep_penetration=%.9f\n", peak);
		}

		// Non-spherical leaves share both terrain and wall support.
		PRUnitTestMethod(BoxAndTerrain, Extended)
		{
			auto& gpu = SharedTestGpu();
			auto engine = Engine({}, nullptr, gpu, gpu.m_job.m_queue.get());
			engine.CylindricalBoundary(CylindricalBoundaryConfig{});
			engine.Terrain(FlatTerrain());
			engine.Material(Material{.m_friction_static = 0, .m_elasticity_norm = 0});
			auto box = collision::ShapeBox(v4(1, 1, 1, 0));
			auto body = RigidBody(&box, m4x4::Translation(3999.48f, 0, 0.51f), Inertia::Box(v4(0.5f, 0.5f, 0.5f, 0), 1));
			auto bodies = std::array{&body};
			auto wall = false, ground = false;
			auto peak = 0.0;
			auto subscription = engine.Collisions += [&](Engine&, std::span<RbContact const> contacts)
			{
				for (auto const& contact : contacts)
				{
					auto normal = -(InvertOrthonormal(contact.m_b2a) * contact.m_axis);
					wall |= normal.x < -0.9f;
					ground |= normal.z > 0.9f;
				}
			};
			for (int frame = 0; frame != 90; ++frame)
			{
				body.GravityWS(v4(3, 0, -9.81f, 0));
				engine.Step(Engine::StepInput{.m_bodies = bodies, .m_elapsed_seconds = 1.0f / 60, .m_substep_count = 4});
				auto depth = RadialExtent(box, body.O2W()) - 4000;
				peak = std::max(peak, depth);
				PR_EXPECT(depth < 0.01);
				PR_EXPECT(body.O2W().pos.z > 0.46f);
			}
			PR_EXPECT(wall && ground);
			PR_EXPECT(engine.LastStepProfile().m_terrain_gpu_ms > 0);
			std::printf("Cylinder 1m box simultaneous terrain+wall max_physical_surface_penetration=%.9f\n", peak);
		}

		// Transformed compound leaves and articulation proxies reach the same world-contact solver endpoint.
		PRUnitTestMethod(CompoundAndArticulation, Extended)
		{
			auto builder = ShapeBuilder{};
			builder.AddShape(collision::ShapeBox(v4(0.4f, 0.4f, 0.4f, 0), m4x4::Translation(-0.5f, 0, 0)));
			builder.AddShape(collision::ShapeSphere(0.2f, m4x4::Translation(0.5f, 0, 0)));
			auto storage = byte_data<16>{};
			auto mass = MassProperties{};
			auto centre = v4{};
			auto shape = builder.BuildShape(storage, mass, centre);
			auto& gpu = SharedTestGpu();
			auto engine = Engine({}, nullptr, gpu, gpu.m_job.m_queue.get());
			engine.CylindricalBoundary(CylindricalBoundaryConfig{});
			engine.Material(Material{.m_friction_static = 0, .m_elasticity_norm = 0});
			auto body = RigidBody(shape, m4x4::Translation(3999.05f, 0, 20), Inertia::Sphere(1, 1));
			auto bodies = std::array{&body};
			auto contact_count = 0;
			auto peak = 0.0;
			auto subscription = engine.Collisions += [&](Engine&, std::span<RbContact const> contacts)
			{
				contact_count += isize(contacts);
			};
			for (int frame = 0; frame != 60; ++frame)
			{
				body.GravityWS(v4(5, 0, 0, 0));
				engine.Step(Engine::StepInput{.m_bodies = bodies, .m_elapsed_seconds = 1.0f / 60, .m_substep_count = 4});
				auto depth = RadialExtent(*shape, body.O2W()) - 4000;
				peak = std::max(peak, depth);
				PR_EXPECT(depth < 0.01);
			}
			PR_EXPECT(contact_count > 0);
			std::printf("Cylinder compound max_physical_surface_penetration=%.9f\n", peak);

			// A floating articulation link has the same radial restriction without becoming a caller-owned rigid body.
			auto sphere = collision::ShapeSphere(0.3f);
			auto articulation_builder = ArticulationBuilder{};
			auto root = articulation_builder.AddFloatingRoot(ArticulationLinkDesc{.m_inertia = Inertia::Sphere(0.3f, 1), .m_shape = &sphere.m_base}, m4x4::Translation(3999.65f, 0, 20));
			auto articulation = articulation_builder.Build();
			auto articulations = std::array{&articulation};
			for (int frame = 0; frame != 60; ++frame)
			{
				articulation.GravityWS(root, v4(5, 0, 0, 0));
				engine.Step(Engine::StepInput{.m_articulations = articulations, .m_elapsed_seconds = 1.0f / 60, .m_substep_count = 4});
				PR_EXPECT(articulation.LinkToWorld(root).pos.x < 3999.75f);
			}
		}

		// Unchanged support stays asleep; explicit motion and source removal wake it without retaining a static endpoint.
		PRUnitTestMethod(SleepWakeAndCapacity, Extended)
		{
			auto& gpu = SharedTestGpu();
			auto engine = Engine({}, nullptr, gpu, gpu.m_job.m_queue.get());
			engine.CylindricalBoundary(CylindricalBoundaryConfig{});
			auto shape = collision::ShapeSphere(0.3f);
			auto body = RigidBody(&shape, m4x4::Translation(3999.7f, 0, 50), Inertia::Sphere(0.3f, 1));
			auto bodies = std::array{&body};
			engine.Step(0.001f, bodies);
			body.Sleep();
			engine.UpdateSleepIslands(bodies);
			auto before = body.O2W();
			for (int frame = 0; frame != 30; ++frame)
			{
				engine.Step(1.0f / 240, bodies);
				PR_EXPECT(body.Sleeping());
				PR_EXPECT(All(body.O2W() == before));
			}

			// An ordinary tangential impact must disturb the island while the wall still supplies radial support.
			auto impactor = RigidBody(&shape, m4x4::Translation(3999.7f, -0.599f, 50), Inertia::Sphere(0.3f, 1));
			impactor.VelocityWS(v4::Zero(), v4(0, 2, 0, 0));
			auto impact_bodies = std::array{&body, &impactor};
			engine.Step(1.0f / 240, impact_bodies);
			PR_EXPECT(!body.Sleeping());
			PR_EXPECT(body.VelocityWS().lin.y > 0.25f);

			// Direct motion and removal are also meaningful wake requests.
			body.VelocityWS(v4::Zero(), v4(-1, 0, 0, 0));
			engine.Step(1.0f / 240, bodies);
			PR_EXPECT(!body.Sleeping());
			body.Sleep();
			engine.CylindricalBoundary(std::nullopt);
			engine.Step(1.0f / 240, bodies);
			PR_EXPECT(!body.Sleeping());

			// Contact overflow cannot partially publish the body's wall response.
			auto box = collision::ShapeBox(v4(0.6f, 0.6f, 0.6f, 0));
			auto crowded = RigidBody(&box, m4x4::Translation(3999.72f, 0, 50), Inertia::Sphere(1, 1));
			auto crowded_bodies = std::array{&crowded};
			auto limited = Engine(EngineConfig{.max_collision_pairs = 1}, nullptr, gpu, gpu.m_job.m_queue.get());
			limited.CylindricalBoundary(CylindricalBoundaryConfig{});
			before = crowded.O2W();
			PR_THROWS(limited.Step(0.00001f, crowded_bodies), std::runtime_error);
			PR_EXPECT(All(crowded.O2W() == before));
		}

		// Invalid source geometry and mutations during pending work fail at the source/lifetime boundary.
		PRUnitTestMethod(ValidationAndAtomicFailure, Extended)
		{
			auto& gpu = SharedTestGpu();
			auto engine = Engine({}, nullptr, gpu, gpu.m_job.m_queue.get());
			engine.CylindricalBoundary(CylindricalBoundaryConfig{});
			auto invalid = CylindricalBoundaryConfig{};
			invalid.m_radius = std::numeric_limits<double>::infinity();
			PR_THROWS(engine.CylindricalBoundary(invalid), std::runtime_error);
			invalid = {};
			invalid.m_surface_spacing = 0;
			PR_THROWS(engine.CylindricalBoundary(invalid), std::runtime_error);
			auto shape = collision::ShapeSphere(0.3f);
			auto body = RigidBody(&shape, m4x4::Translation(3999.6f, 0, 1000), Inertia::Sphere(0.3f, 1));
			auto bodies = std::array{&body};
			engine.BeginStep(0.001f, bodies);
			PR_THROWS(engine.CylindricalBoundary(std::nullopt), std::runtime_error);
			engine.CompleteStep();

			// Removal must leave no retained wall endpoint or contacts.
			engine.CylindricalBoundary(std::nullopt);
			engine.Step(0.001f, bodies);
			PR_EXPECT(engine.LastCollisionStats().LastContactCount() == 0);
		}
	};

	// Contact geometry depends on the current pose, without speed, spin, timestep or overlap cutoffs.
	PRUnitTestClass(CylindricalBoundaryDiscreteTests)
	{
		// Fast translation and rolling far from the wall must keep their physical momentum.
		PRUnitTestMethod(DistantFastRolling, Extended)
		{
			auto& gpu = SharedTestGpu();
			auto engine = Engine({}, nullptr, gpu, gpu.m_job.m_queue.get());
			engine.CylindricalBoundary(CylindricalBoundaryConfig{});
			auto shape = collision::ShapeSphere(0.5f);
			auto body = RigidBody(&shape, m4x4::Translation(60, 80, 10000), Inertia::Sphere(0.5f, 1));
			body.VelocityWS(v4(0, 200, 0, 0), v4(100, 0, 0, 0));
			auto bodies = std::array{&body};
			for (int frame = 0; frame != 120; ++frame)
			{
				engine.Step(Engine::StepInput{.m_bodies = bodies, .m_elapsed_seconds = 1.0f / 60, .m_substep_count = 4});
				PR_EXPECT(FEql(body.VelocityWS().lin.x, 100.0f));
				PR_EXPECT(FEql(body.VelocityWS().ang.y, 200.0f));
			}
			PR_EXPECT(body.O2W().pos.x > 259);
		}

		// Discrete wall contacts must stop outward motion even when one step crosses metres beyond the boundary.
		PRUnitTestMethod(FastWallResponse, Extended)
		{
			for (auto dt : {1.0f / 240, 1.0f / 60, 0.1f})
			{
				auto& gpu = SharedTestGpu();
				auto engine = Engine({}, nullptr, gpu, gpu.m_job.m_queue.get());
				engine.CylindricalBoundary(CylindricalBoundaryConfig{});
				engine.Material(Material{.m_friction_static = 0, .m_elasticity_norm = 0});
				auto shape = collision::ShapeSphere(0.5f);
				auto body = RigidBody(&shape, m4x4::Translation(3998, 0, 10000), Inertia::Sphere(0.5f, 1));
				body.VelocityWS(v4::Zero(), v4(480, 0, 0, 0));
				auto bodies = std::array{&body};
				engine.Step(dt, bodies);
				PR_EXPECT(engine.LastCollisionStats().LastContactCount() > 0);
				PR_EXPECT(body.VelocityWS().lin.x < 1);

				// Resolve residual overlap through normal solver steps, not a caller-applied pose correction.
				for (int frame = 0; frame != 60; ++frame)
					engine.Step(dt, bodies);

				auto extent = RadialExtent(shape, body.O2W());
				PR_EXPECT(extent <= 4000.01);
				std::printf("Discrete wall speed=480 dt=%g final_extent=%g\n", dt, extent);
			}
		}

		// Fast spin and deep placement remain geometric contacts rather than exceptional operating conditions.
		PRUnitTestMethod(DeepSpinningContactGeometry, Extended)
		{
			for (auto spin : {0.0f, 200.0f})
			{
				auto& gpu = SharedTestGpu();
				auto engine = Engine(EngineConfig{.solver_iterations = 0, .push_out_iterations = 0, .selective_refresh_passes = 0}, nullptr, gpu, gpu.m_job.m_queue.get());
				engine.CylindricalBoundary(CylindricalBoundaryConfig{});
				auto shape = collision::ShapeSphere(0.5f);
				auto body = RigidBody(&shape, m4x4::Translation(4005, 0, 10000), Inertia::Sphere(0.5f, 1));
				body.VelocityWS(v4(0, spin, 0, 0), v4::Zero());
				auto bodies = std::array{&body};
				auto peak_depth = 0.0f;
				auto subscription = engine.Collisions += [&](Engine&, std::span<RbContact const> contacts)
				{
					// Check generated geometry independently of any solver convergence policy.
					for (auto const& contact : contacts)
					{
						auto normal = -(InvertOrthonormal(contact.m_b2a) * contact.m_axis);
						PR_EXPECT(normal.x < -0.999f && std::abs(normal.z) < 0.0001f);
						peak_depth = std::max(peak_depth, contact.m_depth);
					}
				};
				engine.Step(0.1f, bodies);
				PR_EXPECT(peak_depth > 5.49f && peak_depth <= 5.501f);
				PR_EXPECT(IsFinite(body.O2W().pos) && IsFinite(body.VelocityWS().ang));
			}
		}

		// Radial distance must not overflow its float square-root seed for a representable distant contact.
		PRUnitTestMethod(LargeRadialContact, Extended)
		{
			auto& gpu = SharedTestGpu();
			auto engine = Engine(EngineConfig{.solver_iterations = 0, .push_out_iterations = 0, .selective_refresh_passes = 0}, nullptr, gpu, gpu.m_job.m_queue.get());
			engine.CylindricalBoundary(CylindricalBoundaryConfig{});
			auto shape = collision::ShapeSphere(0.5f);
			auto body = RigidBody(&shape, m4x4::Translation(1.0e20f, 0, 0), Inertia::Sphere(0.5f, 1));
			auto bodies = std::array{&body};
			auto observed = false;
			auto subscription = engine.Collisions += [&](Engine&, std::span<RbContact const> contacts)
			{
				for (auto const& contact : contacts)
				{
					PR_EXPECT(std::isfinite(contact.m_depth) && contact.m_depth > 9.9e19f);
					observed = true;
				}
			};
			engine.Step(0.01f, bodies);
			PR_EXPECT(observed);
		}

		// A full rotation with an interior endpoint has no discrete contact; intermediate crossings require CCD.
		PRUnitTestMethod(OffsetRotationUsesCurrentPose, Extended)
		{
			auto& gpu = SharedTestGpu();
			auto engine = Engine({}, nullptr, gpu, gpu.m_job.m_queue.get());
			engine.CylindricalBoundary(CylindricalBoundaryConfig{});
			auto shape = collision::ShapeSphere(0.5f, m4x4::Translation(-3, 0, 0));
			auto body = RigidBody(&shape, m4x4::Translation(3998, 0, 10000), Inertia::Sphere(0.5f, 1));
			body.VelocityWS(v4(0, 0, 240 * math::constants<float>::tau, 0), v4::Zero());
			auto bodies = std::array{&body};
			auto before = body.O2W();
			PR_EXPECT(RadialExtent(shape, before) < 4000);
			engine.Step(Engine::StepInput{.m_bodies = bodies, .m_elapsed_seconds = 1.0f / 240});
			PR_EXPECT(RadialExtent(shape, body.O2W()) < 4000);
			PR_EXPECT(engine.LastCollisionStats().LastContactCount() == 0);
		}
	};

	// Exercise real wall friction without restricting the spin it generates.
	PRUnitTestClass(CylindricalBoundaryFrictionTests)
	{
		// Wall friction may transfer fast vertical motion into rotation; finite solver output must continue to publish.
		PRUnitTestMethod(FrictionDrivenSurfaceMotion, Extended)
		{
			auto& gpu = SharedTestGpu();
			for (auto degrees : {0.0, 45.0, 90.0, 180.0, 270.0})
			{
				auto angle = degrees * math::constants<double>::tau / 360;
				auto radial = v4(static_cast<float>(std::cos(angle)), static_cast<float>(std::sin(angle)), 0, 0);
				auto tangent = v4(-radial.y, radial.x, 0, 0);
				auto shape = collision::ShapeSphere(0.5f);
				auto body = RigidBody(&shape, m4x4::Translation(v4(0, 0, 10000, 1) + 3998.5f * radial), Inertia::Sphere(0.5f, 1));
				body.VelocityWS(v4::Zero(), 20 * radial + 3 * tangent + v4(0, 0, -100, 0));
				auto engine = Engine({}, nullptr, gpu, gpu.m_job.m_queue.get());
				engine.Terrain(terrain::landscape::BaselineSurface(terrain::landscape::BaselineSurfaceConfig{.m_seed = 42}), 0.16f);
				engine.CylindricalBoundary(CylindricalBoundaryConfig{});
				engine.Material(Material{.m_friction_static = 0.3f, .m_elasticity_norm = 0.05f, .m_elasticity_tang = 0, .m_elasticity_tors = 0, .m_density = 1});
				auto bodies = std::array{&body};
				auto peak = 0.0;
				auto peak_spin = 0.0f;
				for (int frame = 0; frame != 60; ++frame)
				{
					body.GravityWS(v4(0, 0, -9.81f, 0));
					engine.Step(Engine::StepInput{.m_bodies = bodies, .m_elapsed_seconds = 1.0f / 60, .m_substep_count = 4});
					PR_EXPECT(IsFinite(body.O2W().pos) && IsFinite(body.VelocityWS().lin) && IsFinite(body.VelocityWS().ang));
					peak = std::max(peak, RadialExtent(shape, body.O2W()) - 4000);
					peak_spin = std::max(peak_spin, Length(body.VelocityWS().ang));
					PR_EXPECT(peak < 0.01);
				}
				PR_EXPECT(peak_spin > 40);
				PR_EXPECT(Dot(body.VelocityWS().lin, radial) < 1);
				std::printf("Cylinder friction azimuth=%.0f completed=60 peak_spin=%g peak_overlap=%g\n", degrees, peak_spin, peak);
			}
		}
	};

	// Compare persistent resting support through ordinary collision and sampled terrain.
	PRUnitTestClass(TerrainSleepTests)
	{
		// A distant active body must not make unchanged static support wake an isolated sleeper.
		PRUnitTestMethod(IsolatedSleepingSupport, Extended)
		{
			auto retained = true;
			for (auto sampled : {false, true})
			{
				// Keep the engine active without disturbing the sleeping body's bounds.
				auto shape = collision::ShapeBox(v4(1, 1, 1, 0));
				auto ground_shape = collision::ShapeBox(v4(20, 20, 1, 0));
				auto body = RigidBody(&shape, m4x4::Translation(0, 0, 0.49f), Inertia::Box(v4(0.5f, 0.5f, 0.5f, 0), 1));
				auto distant = RigidBody(&shape, m4x4::Translation(100, 0, 10), Inertia::Sphere(1, 1));
				auto ground = RigidBody(&ground_shape, m4x4::Translation(0, 0, -0.5f), Inertia::Infinite());
				distant.NeverSleep(true);
				auto& gpu = SharedTestGpu();
				auto engine = Engine({}, nullptr, gpu, gpu.m_job.m_queue.get());
				if (sampled)
					engine.Terrain(FlatTerrain());

				// Consume the initial environment change, then initialise the caller-owned resting island.
				auto bodies = std::vector<RigidBody*>{&body, &distant};
				if (!sampled)
					bodies.push_back(&ground);

				// Initial source invalidation is not part of the resting-support check.
				engine.Step(0.00001f, bodies);
				body.O2W(m4x4::Translation(0, 0, 0.49f));
				body.Sleep();
				engine.UpdateSleepIslands(bodies);
				auto const before = body.O2W();
				auto sleeping_frames = 0;
				auto wakes = 0;
				auto was_sleeping = true;
				for (int frame = 0; frame != 120; ++frame)
				{
					body.GravityWS(v4(0, 0, -9.81f, 0));
					engine.Step(Engine::StepInput{.m_bodies = bodies, .m_elapsed_seconds = 1.0f / 60, .m_substep_count = 4});
					sleeping_frames += body.Sleeping();
					wakes += was_sleeping && !body.Sleeping();
					was_sleeping = body.Sleeping();
				}

				// Report the control and sampled outcomes before asserting so a failure retains both measurements.
				std::printf("sleep_isolated sampled=%d sleeping_frames=%d/120 wakes=%d displacement=%.8f\n", sampled, sleeping_frames, wakes, Length(body.O2W().pos - before.pos));
				retained = retained && sleeping_frames == 120 && All(body.O2W() == before);
			}
			PR_EXPECT(retained);
		}

		// Matched small stacks report sustained sleep, transitions, velocity, and wall cost over one minute.
		PRUnitTestMethod(MatchedClumps, Extended)
		{
			auto sustained = true;
			for (auto sampled : {false, true})
			{
				// Use identical geometry, material, initial poses, and stepping for both support representations.
				auto shape = collision::ShapeBox(v4(0.4f, 0.4f, 0.4f, 0));
				auto ground_shape = collision::ShapeBox(v4(20, 20, 1, 0));
				auto ground = RigidBody(&ground_shape, m4x4::Translation(0, 0, -0.5f), Inertia::Infinite());
				auto distant = RigidBody(&shape, m4x4::Translation(100, 0, 10), Inertia::Sphere(1, 1));
				distant.NeverSleep(true);
				auto clump = std::vector<SleepObservedBody>{};
				clump.reserve(16);
				for (int i = 0; i != 16; ++i)
					clump.emplace_back(&shape, m4x4::Translation((i % 2) * 0.4f, ((i / 2) % 2) * 0.4f, 0.3f + (i / 4) * 0.4f), Inertia::Box(v4(0.2f, 0.2f, 0.2f, 0), 1));

				// A remote active body prevents the all-asleep early exit from masking repeated support work.
				auto bodies = std::vector<RigidBody*>{};
				for (auto& body : clump)
					bodies.push_back(&body);

				// Ground and remote active endpoints are excluded from clump statistics.
				bodies.push_back(&distant);
				if (!sampled)
					bodies.push_back(&ground);

				// Both support representations use the same solver settings and material.
				auto& gpu = SharedTestGpu();
				auto engine = Engine({}, nullptr, gpu, gpu.m_job.m_queue.get());
				engine.Material(Material{.m_friction_static = 0.3f, .m_elasticity_norm = 0.05f});
				if (sampled)
					engine.Terrain(FlatTerrain());

				// Measure every frame, retaining ten-second windows rather than isolated end snapshots.
				auto was_sleeping = std::array<bool, 16>{};
				auto previous_islands = std::array<int, 16>{};
				auto sleeps = 0, wakes = 0, island_changes = 0, sleeping_sum = 0, minimum_sleeping = 16;
				auto max_linear = 0.0f, max_angular = 0.0f;
				auto physics_ms = 0.0;
				for (int frame = 0; frame != 3600; ++frame)
				{
					for (auto& body : clump)
						body.GravityWS(v4(0, 0, -9.81f, 0));

					// Time stepping without including the subsequent state inspection.
					auto start = std::chrono::steady_clock::now();
					engine.Step(Engine::StepInput{.m_bodies = bodies, .m_elapsed_seconds = 1.0f / 60, .m_substep_count = 4});
					physics_ms += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
					auto sleeping = 0;
					for (int i = 0; i != 16; ++i)
					{
						auto const& body = clump[i];
						PR_EXPECT(IsFinite(body.O2W().pos) && body.O2W().pos.z > 0.15f);
						sleeping += body.Sleeping();
						sleeps += !was_sleeping[i] && body.Sleeping();
						wakes += was_sleeping[i] && !body.Sleeping();
						island_changes += previous_islands[i] != body.SleepIsland();
						was_sleeping[i] = body.Sleeping();
						previous_islands[i] = body.SleepIsland();
						max_linear = std::max(max_linear, Length(body.VelocityWS().lin));
						max_angular = std::max(max_angular, Length(body.VelocityWS().ang));
					}
					sleeping_sum += sleeping;
					minimum_sleeping = std::min(minimum_sleeping, sleeping);
					if ((frame + 1) % 600 != 0)
						continue;

					// Transition counts expose repeatedly waking stacks even when the last frame happens to be asleep.
					auto top_z_min = std::min({clump[12].O2W().pos.z, clump[13].O2W().pos.z, clump[14].O2W().pos.z, clump[15].O2W().pos.z});
					std::printf("sleep_clump sampled=%d seconds=%d sleep_min=%d sleep_mean=%.4f sleep_final=%d sleeps=%d wakes=%d island_changes=%d lin_max=%.6f ang_max=%.6f top_z_min=%.6f physics_mean_ms=%.4f\n",
						sampled, (frame + 1) / 60, minimum_sleeping, sleeping_sum / 600.0, sleeping, sleeps, wakes, island_changes, max_linear, max_angular, top_z_min, physics_ms / 600);
					std::fflush(stdout);
					if (frame == 3599)
						sustained = sustained && minimum_sleeping == 16 && wakes == 0;

					// Each report owns a disjoint interval, including transient maxima and wall time.
					sleeps = wakes = island_changes = sleeping_sum = 0;
					minimum_sleeping = 16;
					max_linear = max_angular = 0;
					physics_ms = 0;
				}

				// Sleeping after a collapsed stack is not a support regression pass.
				for (int i = 0; i != 16; ++i)
					PR_EXPECT(clump[i].O2W().pos.z >= 0.2f + (i / 4) * 0.4f - 0.08f);
			}
			PR_EXPECT(sustained);
		}

		// An impact must retain terrain support for the disturbed island and an explicit load must still wake it.
		PRUnitTestMethod(DisturbedSupportAndForceWake, Extended)
		{
			// Establish a sleeping box and keep the impactor out of its bounds until the island exists.
			auto shape = collision::ShapeBox(v4(1, 1, 1, 0));
			auto body = RigidBody(&shape, m4x4::Translation(0, 0, 0.49f), Inertia::Box(v4(0.5f, 0.5f, 0.5f, 0), 1));
			auto impactor = RigidBody(&shape, m4x4::Translation(5, 0, 0.5f), Inertia::Box(v4(0.5f, 0.5f, 0.5f, 0), 1));
			auto bodies = std::array{&body, &impactor};
			auto& gpu = SharedTestGpu();
			auto engine = Engine({}, nullptr, gpu, gpu.m_job.m_queue.get());
			engine.Terrain(FlatTerrain());
			engine.Step(0.00001f, bodies);
			body.O2W(m4x4::Translation(0, 0, 0.49f));
			body.Sleep();
			engine.UpdateSleepIslands(bodies);

			// Resolve the first impacting substep with the sleeper's ground contacts already present.
			auto support_contacts = 0;
			auto subscription = engine.Collisions += [&](Engine&, std::span<RbContact const> contacts)
			{
				for (auto const& contact : contacts)
					support_contacts += contact.m_objA == &body && (body.O2W() * contact.m_axis).z < -0.9f;
			};
			impactor.O2W(m4x4::Translation(0.99f, 0, 0.5f));
			impactor.VelocityWS(v4::Zero(), v4(-2, 0, 0, 0));
			engine.Step(1.0f / 240, bodies);
			std::printf("sleep_impact support_contacts=%d sleeping=%d vx=%.6f z=%.6f\n", support_contacts, body.Sleeping(), body.VelocityWS().lin.x, body.O2W().pos.z);
			PR_EXPECT(support_contacts > 0 && !body.Sleeping() && body.VelocityWS().lin.x < -0.1f);
			PR_EXPECT(body.O2W().pos.z > 0.45f);

			// A caller-applied load bypasses sleep without weakening ordinary motion.
			body.Sleep();
			body.ApplyForceWS(v4(0, 0, 120, 0), v4::Zero());
			PR_EXPECT(!body.Sleeping());
			engine.Step(1.0f / 60, bodies);
			PR_EXPECT(body.VelocityWS().lin.z > 0.5f);
		}

		// Whole-tree sleep remains inert on terrain while a remote rigid body keeps the engine active.
		PRUnitTestMethod(SleepingArticulationSupport, Extended)
		{
			// Keep active rigid work separate from the sleeping tree.
			auto shape = collision::ShapeSphere(0.3f);
			auto builder = ArticulationBuilder{};
			auto root = builder.AddFloatingRoot(ArticulationLinkDesc{.m_inertia = Inertia::Sphere(0.3f, 1), .m_shape = &shape.m_base}, m4x4::Translation(0, 0, 0.29f));
			auto articulation = builder.Build();
			auto distant = RigidBody(&shape, m4x4::Translation(100, 0, 10), Inertia::Sphere(0.3f, 1));
			distant.NeverSleep(true);
			auto articulations = std::array{&articulation};
			auto bodies = std::array{&distant};
			auto& gpu = SharedTestGpu();
			auto engine = Engine({}, nullptr, gpu, gpu.m_job.m_queue.get());
			engine.Terrain(FlatTerrain());
			auto input = Engine::StepInput{.m_bodies = bodies, .m_articulations = articulations, .m_elapsed_seconds = 1.0f / 60, .m_substep_count = 4};
			engine.Step(input);
			articulation.Sleep();
			auto const before = articulation.LinkToWorld(root);

			// No terrain bias may change a sleeping tree's pose or restore link velocities.
			for (int frame = 0; frame != 120; ++frame)
			{
				engine.Step(input);
				PR_EXPECT(articulation.Sleeping());
				PR_EXPECT(All(articulation.LinkToWorld(root) == before));
			}
		}
	};

	// Verify shared surface coverage and terrain contacts through the engine's GPU path.
	PRUnitTestClass(TerrainCollisionTests)
	{
		// Capsule samples cover the rounded surface with balanced area; thin lines retain both endpoints.
		PRUnitTestMethod(LineSurfaceCoverage, Extended)
		{
			auto line = collision::ShapeLine(2, 0.3f);
			auto plan = surface::BuildPlan(line);
			auto area = 0.0;
			auto normal_sum = v4::Zero();

			// Compare emitted positions with the swept segment and integrate their represented area and normals.
			for (uint32_t i = 0; i != plan.m_count; ++i)
			{
				auto const sample = surface::EmitSurfaceSample(plan, i);
				auto axis = v4(0, 0, std::clamp(sample.m_pos_local.z, -1.0f, 1.0f), 1);
				PR_EXPECT(FEqlAbsolute(Length(sample.m_pos_local - axis), 0.3f, 1e-5f));
				area += sample.m_darea;
				normal_sum += sample.m_normal_local * sample.m_darea;
			}

			// The capsule's cylinder and sphere contributions form a closed, balanced surface.
			PR_EXPECT(std::abs(area - math::constants<double>::tau * (0.3 * 2 + 2 * 0.3 * 0.3)) < 1e-4);
			PR_EXPECT(Length(normal_sum) < 1e-4f);

			// Thin geometry retains deterministic endpoint-inclusive coverage at the default spacing.
			auto thin = surface::BuildPlan(collision::ShapeLine(2));
			PR_EXPECT(thin.m_count == 14);
			PR_EXPECT(surface::EmitSurfaceSample(thin, 0).m_pos_local.z == -1);
			PR_EXPECT(surface::EmitSurfaceSample(thin, thin.m_count - 1).m_pos_local.z == 1);
		}

		// A falling box settles with its lower face at the independently known flat terrain height.
		PRUnitTestMethod(FlatBoxGpuRest, Extended)
		{
			auto shape = collision::ShapeBox(v4(1, 1, 1, 0));
			auto body = RigidBody(&shape, m4x4::Translation(0, 0, 1), Inertia::Box(v4(0.5f, 0.5f, 0.5f, 0), 1));
			auto& gpu = SharedTestGpu();
			auto engine = Engine({}, nullptr, gpu, gpu.m_job.m_queue.get());
			engine.Material(Material{.m_friction_static = 0.5f, .m_elasticity_norm = 0});
			engine.Terrain(FlatTerrain());
			auto bodies = std::array{&body};

			// Reapply gravity each frame and reject nonfinite or deeply penetrating trajectories.
			for (int i = 0; i != 240; ++i)
			{
				body.GravityWS(v4(0, 0, -9.81f, 0));
				engine.Step(1.0f / 120, bodies);
				PR_EXPECT(IsFinite(body.O2W().pos));
				PR_EXPECT(body.O2W().pos.z > 0.45f);
			}

			// Check resting height rather than only checking that the fall was arrested.
			std::printf("Terrain flat box final z=%.8f\n", body.O2W().pos.z);
			PR_EXPECT(std::abs(body.O2W().pos.z - 0.5f) < 0.02f);
		}

		// Every physical primitive and a transformed compound uses the same GPU stream and static endpoint.
		PRUnitTestMethod(AllShapeContactsAndTransforms, Extended)
		{
			auto box = collision::ShapeBox(v4(0.6f, 0.4f, 0.3f, 0));
			auto sphere = collision::ShapeSphere(0.3f);
			auto line = collision::ShapeLine(0.6f, 0.12f);
			auto thin = collision::ShapeLine(0.6f);
			auto point = collision::ShapeSphere(0);
			auto degenerate = collision::ShapeTriangle(v4(-0.3f, 0, 0, 1), v4(0.3f, 0, 0, 1), v4(0, 0, 0, 1));
			auto zero_line = collision::ShapeLine(0);
			auto triangle = collision::ShapeTriangle(v4(-0.3f, -0.2f, 0, 1), v4(0.3f, -0.2f, 0, 1), v4(0, 0.3f, 0, 1));
			auto vertices = std::array{v4(-0.3f, -0.2f, -0.2f, 1), v4(0.3f, -0.2f, -0.2f, 1), v4(0, 0.3f, -0.2f, 1), v4(0, 0, 0.3f, 1)};
			auto poly = collision::BuildPolytopeFromPoints(vertices, m4x4::Identity(), 0, Shape::EFlags::None, 0);

			// Use displaced leaves to expose missing or repeated child transforms.
			auto compound_builder = ShapeBuilder{};
			auto child = box;
			child.m_base.m_s2r = m4x4::Translation(-0.3f, 0.2f, 0.1f);
			compound_builder.AddShape(child);
			auto child_sphere = sphere;
			child_sphere.m_base.m_s2r = m4x4::Translation(0.4f, 0, 0);
			compound_builder.AddShape(child_sphere);
			auto compound_data = byte_data<16>{};
			auto mass = MassProperties{};
			auto centre = v4{};
			auto compound = compound_builder.BuildShape(compound_data, mass, centre);
			auto shapes = std::array<Shape const*, 10>{&box.m_base, &sphere.m_base, &line.m_base, &thin.m_base, &triangle.m_base, &poly.as<collision::ShapePolytope>().m_base, compound, &point.m_base, &degenerate.m_base, &zero_line.m_base};

			// Disable solver motion so contact geometry can be checked at the chosen pose.
			auto& gpu = SharedTestGpu();
			auto engine = Engine(EngineConfig{.solver_iterations = 0, .push_out_iterations = 0, .selective_refresh_passes = 0}, nullptr, gpu, gpu.m_job.m_queue.get());
			engine.Terrain(FlatTerrain());
			auto contact_count = 0;

			// Flat terrain must yield downward body-A normals and the intended shallow penetration.
			auto subscription = engine.Collisions += [&](Engine&, std::span<RbContact const> contacts)
			{
				for (auto const& contact : contacts)
				{
					++contact_count;
					auto const normal = contact.m_objA->O2W() * contact.m_axis;
					PR_EXPECT(normal.z < -0.999f);
					PR_EXPECT(contact.m_depth > 0 && contact.m_depth < 0.03f);
				}
			};

			// Place each rotated shape from its lowest shared surface sample, independent of its bounding box.
			for (auto shape : shapes)
			{
				auto transform = m4x4::Transform(v4(0.21f, 0.17f, 0.13f, 0), v4(2, 3, 0, 1));
				auto lower = std::numeric_limits<float>::max();

				// Measure leaf samples in world space using their stored shape-to-root transforms once.
				auto measure = [&](auto&& self, Shape const& primitive) -> void
				{
					switch (primitive.m_type)
					{
						case collision::EShape::Array:
						{
							auto const& array = shape_cast<collision::ShapeArray>(primitive);
							for (auto c = array.begin(); c != array.end(); c = collision::next(c))
								self(self, *c);
							break;
						}
						default:
						{
							auto const plan = surface::BuildPlan(primitive);
							for (uint32_t i = 0; i != plan.m_count; ++i)
								lower = std::min(lower, (transform * primitive.m_s2r * surface::EmitSurfaceSample(plan, i).m_pos_local).z);
							break;
						}
					}
				};

				// Advance a nearly stationary frame and require at least one terrain contact for every shape.
				measure(measure, *shape);
				transform.pos.z = -lower - 0.01f;
				auto body = RigidBody(shape, transform, Inertia::Sphere(0.3f, 1));
				auto bodies = std::array{&body};
				contact_count = 0;
				engine.Step(0.00001f, bodies);
				PR_EXPECT(contact_count != 0);
				std::printf("Terrain primitive %d contacts=%d\n", static_cast<int>(shape->m_type), contact_count);
				engine.ResetCaches();
			}
		}

		// Clearing an already absent source is cache maintenance, not an environmental change that wakes sleepers.
		PRUnitTestMethod(AbsentTerrainDoesNotWakeBodies, Extended)
		{
			auto shape = collision::ShapeSphere(0.3f);
			auto body = RigidBody(&shape, m4x4::Translation(0, 0, 3), Inertia::Sphere(0.3f, 1));
			auto& gpu = SharedTestGpu();
			auto engine = Engine({}, nullptr, gpu, gpu.m_job.m_queue.get());
			auto bodies = std::array{&body};

			// Clearing an absent source must preserve an already sleeping body's state.
			engine.Step(0.01f, bodies);
			body.Sleeping(true);
			engine.Terrain(std::nullopt);
			engine.Step(0.01f, bodies);
			PR_EXPECT(body.Sleeping());
			PR_EXPECT(body.O2W().pos.z == 3);

			// Irrelevant static geometry must not require a representable surface plan or lie in the terrain domain.
			auto huge = collision::ShapeBox(v4(1e6f, 1e6f, 1e6f, 0));
			auto ground = RigidBody(&huge, m4x4::Translation(1e7f, 0, 0), Inertia::Infinite());
			auto mixed = std::array{&body, &ground};
			engine.Terrain(FlatTerrain());
			engine.Step(0.01f, mixed);
			PR_EXPECT(engine.LastCollisionStats().LastContactCount() == 0);

			// The same shape must be validated once a mass change makes it participate.
			ground.SetMassProperties(Inertia::Sphere(1, 1));
			PR_THROWS(engine.Step(0.01f, mixed), std::runtime_error);
		}

		// With positional bias disabled, the physical impulse has a known angular response and energy loss.
		PRUnitTestMethod(ImpulseLeverArmAndCapacity, Extended)
		{
			auto point = collision::ShapeSphere(0, m4x4::Translation(0.2f, 0, 0));
			auto body = RigidBody(&point, m4x4::Identity(), Inertia::Sphere(1, 1));
			body.VelocityWS(v4::Zero(), v4(0, 0, -1, 0));
			auto& gpu = SharedTestGpu();
			auto engine = Engine(EngineConfig{.solver_iterations = 1, .push_out_iterations = 0, .velocity_baumgarte = 0, .selective_refresh_passes = 0}, nullptr, gpu, gpu.m_job.m_queue.get());
			engine.Material(Material{.m_friction_static = 0, .m_elasticity_norm = 0});
			engine.Terrain(FlatTerrain());
			auto bodies = std::array{&body};
			engine.Step(0.00001f, bodies);

			// Compare the off-centre impulse with the analytical effective mass and require non-increasing energy.
			auto const impulse = 1.0f / (1 + 0.2f * 0.2f / 0.4f);
			std::printf("Terrain impulse vz=%.8f wy=%.8f expected_j=%.8f energy=%.8f\n", body.VelocityWS().lin.z, body.VelocityWS().ang.y, impulse, body.KineticEnergy());
			PR_EXPECT(std::abs(body.VelocityWS().lin.z - (-1 + impulse)) < 2e-4f);
			PR_EXPECT(std::abs(body.VelocityWS().ang.y + 0.2f * impulse / 0.4f) < 2e-4f);
			PR_EXPECT(body.KineticEnergy() <= 0.5f);
			PR_EXPECT(engine.LastStepProfile().m_terrain_gpu_ms > 0);

			// Overflow is an explicit rejected frame, not a partially published terrain manifold.
			auto box = collision::ShapeBox(v4(1, 1, 1, 0));
			auto crowded = RigidBody(&box, m4x4::Translation(0, 0, 0.4f), Inertia::Sphere(1, 1));
			auto before = crowded.O2W();
			auto limited = Engine(EngineConfig{.max_collision_pairs = 1}, nullptr, gpu, gpu.m_job.m_queue.get());
			limited.Terrain(FlatTerrain());
			auto limited_bodies = std::array{&crowded};
			PR_THROWS(limited.Step(0.00001f, limited_bodies), std::runtime_error);
			PR_EXPECT(FEqlAbsolute(crowded.O2W().pos, before.pos, 1e-7f));
		}

		// Opposite sides of a hollow retain independent normals, and penetrating depth is the local tangent-plane distance.
		PRUnitTestMethod(HollowNormalsAndDepth, Extended)
		{
			auto config = FlatTerrain().Config();
			config.m_regional_base = {0.8, 4, 1, 2, 0.5};
			auto source = terrain::landscape::BaselineSurface(config);
			auto centre = terrain::v2d{0, 0};
			auto best = std::numeric_limits<double>::max();

			// Locate a hollow with a coarse search, then refine its centre without relying on GPU contact results.
			for (int x = -16; x != 17; ++x)
				for (int y = -16; y != 17; ++y)
				{
					auto xy = terrain::v2d{x * 0.25, y * 0.25};
					auto h = source.Sample(xy).m_height;
					if (h < best)
					{
						centre = xy;
						best = h;
					}
				}
			for (auto step : {0.1, 0.025, 0.005, 0.001})
			{
				auto origin = centre;
				for (int x = -2; x != 3; ++x)
					for (int y = -2; y != 3; ++y)
					{
						auto xy = origin + terrain::v2d{x * step, y * step};
						auto h = source.Sample(xy).m_height;
						if (h < best)
						{
							centre = xy;
							best = h;
						}
					}
			}

			// Span both sides of the hollow with a shallowly penetrating box and disable solver motion.
			auto shape = collision::ShapeBox(v4(0.6f, 0.6f, 0.6f, 0));
			auto body = RigidBody(&shape, m4x4::Translation(static_cast<float>(centre.x), static_cast<float>(centre.y), static_cast<float>(best + 0.28)), Inertia::Sphere(0.3f, 1));
			auto& gpu = SharedTestGpu();
			auto engine = Engine(EngineConfig{.solver_iterations = 0, .push_out_iterations = 0, .selective_refresh_passes = 0}, nullptr, gpu, gpu.m_job.m_queue.get());
			engine.Terrain(source);
			auto min_x = 1.0f, max_x = -1.0f;
			auto count = 0;

			// Reconstruct each original sample from the contact midpoint and check the independent FP64 field.
			auto subscription = engine.Collisions += [&](Engine&, std::span<RbContact const> contacts)
			{
				for (auto const& contact : contacts)
				{
					auto a2w = InvertOrthonormal(contact.m_b2a);
					auto normal = -(a2w * contact.m_axis);
					auto sample = a2w * contact.Point() - normal * (0.5f * contact.m_depth);
					auto field = source.Sample({sample.x, sample.y});
					auto expected_normal = Normalise(v4(static_cast<float>(-field.m_gradient_xy.x), static_cast<float>(-field.m_gradient_xy.y), 1, 0));
					PR_EXPECT(Length(normal - expected_normal) < 1e-4f);
					PR_EXPECT(std::abs(contact.m_depth - (field.m_height - sample.z) * expected_normal.z) < 1e-4);
					min_x = std::min(min_x, normal.x);
					max_x = std::max(max_x, normal.x);
					++count;
				}
			};

			// Require simultaneous constraints from both horizontal slope directions.
			auto bodies = std::array{&body};
			engine.Step(0.00001f, bodies);
			std::printf("Terrain hollow contacts=%d normal_x=[%.6f,%.6f]\n", count, min_x, max_x);
			PR_EXPECT(count >= 4 && min_x < -0.02f && max_x > 0.02f);
		}

		// Discrete substeps catch this bounded fast fall, while source changes respect pending frames and wake resting bodies.
		PRUnitTestMethod(SubstepsLifecycleAndFailures, Extended)
		{
			auto shape = collision::ShapeSphere(0.3f);
			auto body = RigidBody(&shape, m4x4::Translation(0, 0, 3), Inertia::Sphere(0.3f, 1));
			body.VelocityWS(v4::Zero(), v4(0, 0, -20, 0));
			auto& gpu = SharedTestGpu();
			auto engine = Engine({}, nullptr, gpu, gpu.m_job.m_queue.get());
			engine.Material(Material{.m_friction_static = 0.5f, .m_elasticity_norm = 0});
			engine.Terrain(FlatTerrain());
			auto bodies = std::array{&body};

			// Check a fast downward trajectory with eight discrete substeps per frame.
			for (int i = 0; i != 60; ++i)
			{
				body.GravityWS(v4(0, 0, -9.81f, 0));
				engine.Step(Engine::StepInput{.m_bodies = bodies, .m_elapsed_seconds = 1.0f / 60, .m_substep_count = 8});
				PR_EXPECT(body.O2W().pos.z > 0.27f);
			}
			PR_EXPECT(std::abs(body.O2W().pos.z - 0.3f) < 0.02f);

			// Source mutation is forbidden while a frame is pending, and invalid spacing is rejected.
			engine.BeginStep(0.001f, bodies);
			PR_THROWS(engine.Terrain(std::nullopt), std::runtime_error);
			engine.CompleteStep();
			PR_THROWS(engine.Terrain(FlatTerrain(), 0), std::runtime_error);

			// Removing resting support wakes the body on the next submitted frame.
			body.Sleeping(true);
			engine.Terrain(std::nullopt);
			body.GravityWS(v4(0, 0, -9.81f, 0));
			auto z = body.O2W().pos.z;
			engine.Step(0.1f, bodies);
			PR_EXPECT(!body.Sleeping());

			// GravityWS does not accumulate forces while asleep; the caller supplies it again after the environment wakes the body.
			body.GravityWS(v4(0, 0, -9.81f, 0));
			engine.Step(0.1f, bodies);
			PR_EXPECT(body.O2W().pos.z < z - 0.01f);

			// Above-terrain rejection must still validate the horizontal query domain.
			engine.Terrain(FlatTerrain());
			body.O2W(m4x4::Translation(0, 0, 500));
			engine.Step(0.00001f, bodies);
			PR_EXPECT(engine.LastCollisionStats().LastContactCount() == 0);
			body.O2W(m4x4::Translation(2.0e6f, 0, 500));
			auto rejected = false;
			try
			{
				engine.Step(0.00001f, bodies);
			}
			catch (std::runtime_error const& error)
			{
				auto message = std::string_view(error.what());
				PR_EXPECT(message.find("configured terrain query domain") != std::string_view::npos);
				PR_EXPECT(message.find("body=0") != std::string_view::npos);
				rejected = true;
			}
			PR_EXPECT(rejected);
		}

		// A moving articulation link reaches the same terrain pass and its existing coupled solver.
		PRUnitTestMethod(ArticulationTerrainContact, Extended)
		{
			auto shape = collision::ShapeSphere(0.3f);
			auto builder = ArticulationBuilder{};
			auto root = builder.AddFloatingRoot(ArticulationLinkDesc{.m_inertia = Inertia::Sphere(0.3f, 1), .m_shape = &shape.m_base}, m4x4::Translation(0, 0, 0.5f));
			auto articulation = builder.Build();
			auto articulations = std::array{&articulation};
			auto& gpu = SharedTestGpu();
			auto engine = Engine({}, nullptr, gpu, gpu.m_job.m_queue.get());
			engine.Material(Material{.m_friction_static = 0.5f, .m_elasticity_norm = 0});
			engine.Terrain(FlatTerrain());

			// Exercise terrain contact through the floating link's existing coupled solver.
			for (int i = 0; i != 120; ++i)
			{
				articulation.GravityWS(root, v4(0, 0, -9.81f, 0));
				engine.Step(Engine::StepInput{.m_articulations = articulations, .m_elapsed_seconds = 1.0f / 120});
				PR_EXPECT(articulation.LinkToWorld(root).pos.z > 0.27f);
			}

			// The spherical link settles one radius above the reference plane.
			std::printf("Terrain articulation final z=%.8f\n", articulation.LinkToWorld(root).pos.z);
			PR_EXPECT(std::abs(articulation.LinkToWorld(root).pos.z - 0.3f) < 0.02f);
		}
	};
}
#endif
