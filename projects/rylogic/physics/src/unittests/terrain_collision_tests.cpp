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

namespace pr::physics::tests
{
	namespace
	{
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
	} // namespace

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
			PR_THROWS(engine.Step(0.00001f, bodies), std::runtime_error);
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
