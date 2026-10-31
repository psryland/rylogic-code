//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Unit tests for the deterministic CPU buoyancy sampler (the GPU buoyancy reference oracle).
// These tests are pure-CPU (no GPU pipeline) so they can run via:
//   physics-sandbox.exe -unittest BuoyancySamplerTests
#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/physics/buoyancy/buoyancy_sampler.h"
#include "pr/physics/shape/shape_builder.h"
#include "src/buoyancy/buoyancy_analytical.h"

namespace pr::physics::tests
{
	using namespace pr::collision;
	using namespace pr::physics::buoyancy;

	namespace
	{
		// A gravity-frame analytic water field for tests: height = level + a*u + b*v measured along
		// 'up', with constant gradient (a,b) and an optional uniform fluid velocity.
		struct TestField
		{
			float m_level = 0.0f;
			float m_a = 0.0f;
			float m_b = 0.0f;
			v4 m_velocity = v4::Zero();

			float Height(v2 uv) const { return m_level + m_a * uv.x + m_b * uv.y; }
			v2 Slope(v2) const { return v2{m_a, m_b}; }
			v4 PressureGradient(v2, float) const { return v4{m_a, m_b, 0.0f, 0.0f}; }
			v4 Velocity(v4) const { return m_velocity; }
		};

		// Van der Corput radical inverse of 'index' in 'base', giving a low-discrepancy value in [0, 1).
		float RadicalInverse(uint32_t index, uint32_t base)
		{
			// Mirror the base-'base' digits of 'index' about the radix point.
			auto result = 0.0;
			auto scale = 1.0 / base;
			for (; index != 0; index /= base, scale /= base)
				result += (index % base) * scale;

			return static_cast<float>(result);
		}

		// Analytic submerged volume of a ball of radius R centred at the origin below the plane z<=h.
		float BallVolumeBelow(float R, float h)
		{
			if (h <= -R) return 0.0f;
			if (h >= +R) return (4.0f / 3.0f) * static_cast<float>(math::constants<double>::tau_by_2) * R * R * R;
			return static_cast<float>(math::constants<double>::tau_by_2) * (R * R * h - (h * h * h) / 3.0f + (2.0f / 3.0f) * R * R * R);
		}
	}

	PRUnitTestClass(BuoyancySamplerTests)
	{
		// Pin face-local corners, exact face quadrature, cell spacing, and all incident sharp normals.
		PRUnitTestMethod(FeaturePreservingBoxSurface, Extended)
		{
			auto box = ShapeBox(v4{2, 4, 0.5f, 0});
			auto const spacing = 0.2f;
			auto const plan = surface::BuildPlan(box.m_base, spacing);
			auto normal_sum = v4::Zero();
			auto moment = v4::Zero();
			auto total_area = 0.0;
			auto begin = 0u;
			for (auto const& patch : plan.m_patches)
			{
				// Rectangular cells obey the diagonal bound, not merely a spacing bound along one axis.
				PR_EXPECT(std::hypot(Length(patch.m_u) / patch.m_nu, Length(patch.m_v) / patch.m_nv) <= spacing * 1.00001f);
				auto face_area = 0.0;
				auto const count = patch.m_sample_end - begin;
				for (auto i = 0u; i != count; ++i)
				{
					auto const sample = surface::EmitSurfaceSample(plan, begin + i);
					auto const again = surface::EmitSurfaceSample(plan, begin + i);
					PR_EXPECT(All(sample.m_pos_local == again.m_pos_local) && sample.m_darea == again.m_darea);
					PR_EXPECT(All(sample.m_normal_local == patch.m_normal));
					PR_EXPECT(sample.m_darea > 0.0f);
					face_area += sample.m_darea;
					normal_sum += sample.m_normal_local * sample.m_darea;
					moment += sample.m_pos_local.w0() * sample.m_darea;
				}
				PR_EXPECT(std::abs(face_area - patch.m_measure) < 2e-6 * patch.m_measure);
				total_area += face_area;
				begin = patch.m_sample_end;
			}
			PR_EXPECT(std::abs(total_area - 22.0) < 1e-5);
			PR_EXPECT(Length(normal_sum) < 1e-4f && Length(moment) < 1e-4f);

			// Every real corner has exactly the three incident face contributions, never a diagonal averaged normal.
			for (int corner = 0; corner != 8; ++corner)
			{
				auto const position = v4{(corner & 1) ? 1.0f : -1.0f, (corner & 2) ? 2.0f : -2.0f, (corner & 4) ? 0.25f : -0.25f, 1};
				auto mask = 0;
				auto count = 0;
				for (uint32_t i = 0; i != plan.m_count; ++i)
				{
					auto const sample = surface::EmitSurfaceSample(plan, i);
					if (!FEqlAbsolute(sample.m_pos_local, position, 1e-6f))
						continue;

					++count;
					for (int axis = 0; axis != 3; ++axis)
					{
						if (sample.m_normal_local[axis] != 0.0f)
						{
							mask |= 1 << axis;
							PR_EXPECT(sample.m_normal_local[axis] * position[axis] > 0.0f);
						}
					}
				}
				PR_EXPECT(count == 3 && mask == 7);
			}
		}

		// Cubed-sphere quadrature covers every face without poles, conserves area, and respects antipodal symmetry.
		PRUnitTestMethod(SphereSurfaceCoverageAndArea, Extended)
		{
			auto sphere = ShapeSphere(1.0f);
			auto const plan = surface::BuildPlan(sphere.m_base);
			PR_EXPECT(plan.m_count == 2166);
			auto area = 0.0;
			auto normal_sum = v4::Zero();
			auto samples = std::vector<v4>{};
			for (uint32_t i = 0; i != plan.m_count; ++i)
			{
				auto const sample = surface::EmitSurfaceSample(plan, i);
				PR_EXPECT(sample.m_darea > 0.0f);
				PR_EXPECT(FEqlAbsolute(Length(sample.m_pos_local.w0()), 1.0f, 2e-6f));
				PR_EXPECT(All(sample.m_pos_local.w0() == sample.m_normal_local));
				area += sample.m_darea;
				normal_sum += sample.m_normal_local * sample.m_darea;
				samples.push_back(sample.m_pos_local);
			}
			PR_EXPECT(std::abs(area - 4.0 * math::constants<double>::tau_by_2) < 2e-5);
			PR_EXPECT(Length(normal_sum) < 2e-5f);

			// Independent low-discrepancy probes include all orientations rather than reusing the cube lattice.
			for (uint32_t i = 1; i != 513; ++i)
			{
				auto const z = 1.0f - 2.0f * RadicalInverse(i, 2);
				auto const phi = math::constants<float>::tau * RadicalInverse(i, 3);
				auto const rho = std::sqrt(1.0f - z * z);
				auto const point = v4{rho * std::cos(phi), rho * std::sin(phi), z, 1};
				auto distance_sq = 4.0f;
				for (auto const& sample : samples)
					distance_sq = std::min(distance_sq, LengthSq(point - sample));

				PR_EXPECT(2.0f * std::asin(0.5f * std::sqrt(distance_sq)) <= surface::DefaultSpacing * 0.5001f);
			}
		}

		// Triangle strips preserve corners, first moments, and bounded coverage even at high aspect ratio.
		PRUnitTestMethod(TriangleSurfaceQuadrature, Extended)
		{
			for (auto const extent : {v2{2, 1}, v2{10, 0.01f}, v2{1, 0.7f}})
			{
				auto triangle = ShapeTriangle(v4{-extent.x, 0, 0, 1}, v4{extent.x, 0, 0, 1}, v4{extent.x * 0.8f, extent.y, 0, 1});
				auto const plan = surface::BuildPlan(triangle.m_base, 0.2f);
				auto area = 0.0;
				auto moment = v4::Zero();
				auto samples = std::vector<v4>{};
				for (uint32_t i = 0; i != plan.m_count; ++i)
				{
					auto const sample = surface::EmitSurfaceSample(plan, i);
					PR_EXPECT(sample.m_darea > 0.0f);
					PR_EXPECT(FEqlAbsolute(sample.m_normal_local, triangle.normal(), 1e-6f));
					area += sample.m_darea;
					moment += sample.m_pos_local.w0() * sample.m_darea;
					samples.push_back(sample.m_pos_local);
				}
				auto const expected_area = extent.x * extent.y;
				PR_EXPECT(std::abs(area - expected_area) < expected_area * 2e-5);
				auto const centroid = ((triangle.m_v.x + triangle.m_v.y + triangle.m_v.z) / 3.0f).w0();
				PR_EXPECT(FEqlAbsolute(moment / static_cast<float>(area), centroid, 2e-5f));
				PR_EXPECT(plan.m_count < 10000);

				// Probe corners, all three edges, and the interior in independent barycentric coordinates.
				for (int i = 0; i != 21; ++i)
				{
					for (int j = 0; j != 21 - i; ++j)
					{
						auto const point = (triangle.m_v.x * (1.0f - (i + j) / 20.0f) + triangle.m_v.y * (i / 20.0f) + triangle.m_v.z * (j / 20.0f)).w1();
						auto nearest = std::numeric_limits<float>::max();
						for (auto const& sample : samples)
							nearest = std::min(nearest, LengthSq(point - sample));

						auto const corner = (i == 0 && j == 0) || i == 20 || j == 20;
						PR_EXPECT(nearest <= (corner ? 1e-10f : 0.010001f));
					}
				}
			}

			// Exactly collinear triangles have no represented area and do not fabricate weighted features.
			auto degenerate = ShapeTriangle(v4{-1, 0, 0, 1}, v4{1, 0, 0, 1}, v4{0, 0, 0, 1});
			auto degenerate_plan = surface::BuildPlan(degenerate.m_base);
			PR_EXPECT(degenerate_plan.m_count == 14);
			PR_EXPECT(surface::EmitSurfaceSample(degenerate_plan, 0).m_darea == 0);
		}

		// Surface-only polytopes require no interior tetrahedra and retain outward balance after flattening.
		PRUnitTestMethod(PolytopeSurfaceWithoutVolumeGeometry, Extended)
		{
			v4 points[] = {v4{1,0,0,1}, v4{-1,0,0,1}, v4{0,1,0,1}, v4{0,-1,0,1}, v4{0,0,1,1}, v4{0,0,-1,1}};
			auto buffer = BuildPolytopeFromPoints(points, m4x4::Identity(), 0, Shape::EFlags::None, 0);
			auto const& poly = buffer.as<ShapePolytope>();
			PR_EXPECT(poly.m_tet_count == 0);
			auto const plan = surface::BuildPlan(poly.m_base, 0.2f);
			auto area = 0.0;
			auto balance = v4::Zero();
			for (uint32_t i = 0; i != plan.m_count; ++i)
			{
				auto const sample = surface::EmitSurfaceSample(plan, i);
				area += sample.m_darea;
				balance += sample.m_normal_local * sample.m_darea;
				PR_EXPECT(Dot3(sample.m_pos_local, sample.m_normal_local) > 0.0f);
			}
			PR_EXPECT(std::abs(area - 4.0 * std::sqrt(3.0)) < 1e-5);
			PR_EXPECT(Length(balance) < 2e-5f);

			// Each polytope corner retains every incident face normal, not only one averaged direction.
			for (int f = 0; f != poly.m_face_count; ++f)
			{
				auto const& face = poly.face(f);
				for (int corner = 0; corner != 3; ++corner)
				{
					auto found = false;
					for (uint32_t i = 0; i != plan.m_count; ++i)
					{
						auto const sample = surface::EmitSurfaceSample(plan, i);
						found |= FEqlAbsolute(sample.m_pos_local, poly.vertex(face.m_index[corner]), 2e-6f) &&
							FEqlAbsolute(sample.m_normal_local, face.m_plane.direction(), 2e-6f);
					}
					PR_EXPECT(found);
				}
			}

			// A slender polytope keeps area-proportional strip counts and the same per-cell spacing guarantee.
			for (auto& point : points)
			{
				point.x *= 4.0f;
				point.z *= 0.03f;
			}
			auto slender_buffer = BuildPolytopeFromPoints(points, m4x4::Identity(), 0, Shape::EFlags::None, 0);
			auto const& slender = slender_buffer.as<ShapePolytope>();
			auto const slender_plan = surface::BuildPlan(slender.m_base, 0.2f);
			auto expected_area = 0.0;
			for (int f = 0; f != slender.m_face_count; ++f)
			{
				auto const& face = slender.face(f);
				expected_area += 0.5 * Length(Cross(slender.vertex(face.m_index[1]) - slender.vertex(face.m_index[0]), slender.vertex(face.m_index[2]) - slender.vertex(face.m_index[0])));
			}
			area = 0.0;
			for (uint32_t i = 0; i != slender_plan.m_count; ++i)
				area += surface::EmitSurfaceSample(slender_plan, i).m_darea;

			PR_EXPECT(std::abs(area - expected_area) < expected_area * 2e-5);
			PR_EXPECT(slender_plan.m_count < 20000);
			for (auto const& patch : slender_plan.m_patches)
			{
				PR_EXPECT(std::hypot(Length(patch.m_u), Length(patch.m_v) * patch.m_taper) <= 0.10001f);
				PR_EXPECT(Length(patch.m_v) / patch.m_nv <= 0.10001f);
			}
		}

		// Reject invalid, numerically indistinguishable, and overflow-sized plans rather than silently reducing counts.
		PRUnitTestMethod(SurfacePlanValidation, Extended)
		{
			auto box = ShapeBox(v4{2, 2, 2, 0});
			PR_THROWS(surface::BuildPlan(box.m_base, 0.0f), std::runtime_error);
			PR_THROWS(surface::BuildPlan(box.m_base, -1.0f), std::runtime_error);
			PR_THROWS(surface::BuildPlan(box.m_base, std::numeric_limits<float>::infinity()), std::runtime_error);
			PR_THROWS(surface::BuildPlan(box.m_base, std::numeric_limits<float>::quiet_NaN()), std::runtime_error);
			PR_THROWS(surface::BuildPlan(box.m_base, 1e-12f), std::runtime_error);
			PR_THROWS(surface::BuildPlan(box.m_base, 1e-4f), std::runtime_error);
			auto const plan = surface::BuildPlan(box.m_base);
			PR_THROWS(surface::EmitSurfaceSample(plan, plan.m_count), std::runtime_error);

			// Invalidate a legitimately constructed box to exercise the sampler's exception seam, not the constructor assertion.
			auto invalid_box = ShapeBox(v4{2, 2, 2, 0});
			invalid_box.m_radius.z = 0.0f;
			PR_THROWS(surface::BuildPlan(invalid_box.m_base), std::runtime_error);
			auto sphere = ShapeSphere(0.0f);
			auto point_plan = surface::BuildPlan(sphere.m_base);
			PR_EXPECT(point_plan.m_count == 1);
			PR_EXPECT(surface::EmitSurfaceSample(point_plan, 0).m_darea == 0);
			sphere.m_radius = -1.0f;
			PR_THROWS(surface::BuildPlan(sphere.m_base), std::runtime_error);
			sphere.m_radius = std::numeric_limits<float>::quiet_NaN();
			PR_THROWS(surface::BuildPlan(sphere.m_base), std::runtime_error);
			sphere.m_radius = 1e-20f;
			PR_THROWS(surface::BuildPlan(sphere.m_base), std::runtime_error);
		}

		// Known full/half-wet drag checks area, first moments and symmetry, and the water-column volume is independent of resolution.
		PRUnitTestMethod(SurfaceForceTorqueAndVolumePreservation, Extended)
		{
			// Keep the exact half-face reference between grid rows rather than on a wet/dry sample boundary.
			auto box = ShapeBox(v4{2, 2, 2, 0});
			auto body = BodyState{.m_gravity_ws = v4{0,0,-9.81f,0}, .m_vel_lin_ws = v4::XAxis()};
			auto cfg = SamplerConfig{.m_quadratic_drag_coefficient = 1.0f, .m_surface_spacing = 0.1f};
			auto const full = SampleHull(box.m_base, body, WaterFrame{}, TestField{.m_level = 10}, cfg);
			auto const half = SampleHull(box.m_base, body, WaterFrame{}, TestField{}, cfg);
			PR_EXPECT(FEqlAbsolute(full.m_drag_force_ws, v4{-2000,0,0,0}, 0.05f));
			PR_EXPECT(Length(full.m_drag_torque_ws) < 0.01f);
			PR_EXPECT(FEqlAbsolute(half.m_drag_force_ws, v4{-1000,0,0,0}, 0.05f));
			PR_EXPECT(FEqlAbsolute(half.m_drag_torque_ws, v4{0,500,0,0}, 1.0f));

			// The default's 18 intervals put a row exactly on the waterline. Partial cell wetness counts that row as half wet,
			// which recovers the exact half-face drag.
			cfg.m_surface_spacing = surface::DefaultSpacing;
			auto const boundary = SampleHull(box.m_base, body, WaterFrame{}, TestField{}, cfg);
			PR_EXPECT(FEqlAbsolute(boundary.m_drag_force_ws, v4{-1000,0,0,0}, 0.05f));
			PR_EXPECT(FEqlAbsolute(boundary.m_drag_torque_ws, v4{0,500,0,0}, 1.0f));

			// Sphere windward-normal drag integrates n_x cubed, not the projected disk with a constant normal.
			auto sphere = ShapeSphere(1.0f);
			auto const ball = SampleHull(sphere.m_base, body, WaterFrame{}, TestField{.m_level = 10}, cfg);
			PR_EXPECT(FEqlRelative(ball.m_drag_force_ws.x, -250.0f * math::constants<float>::tau_by_2, 0.002f));
			PR_EXPECT(std::abs(ball.m_drag_force_ws.y) < 0.01f && std::abs(ball.m_drag_force_ws.z) < 0.01f);
			PR_EXPECT(Length(ball.m_drag_torque_ws) < 0.01f);

			// The half-submerged box's wet volume is its flat bottom face times its depth, so it is exact at any resolution.
			// Lift and damping follow the volume, and the symmetric layout keeps the torques near zero.
			cfg.m_quadratic_drag_coefficient = 0.0f;
			cfg.m_linear_drag_time_constant_s = 2.0f;
			cfg.m_angular_drag_time_constant_s = 3.0f;
			body.m_omega_ws = v4{0.1f, 0.2f, -0.3f, 0};
			auto const coarse = SampleHull(box.m_base, body, WaterFrame{}, TestField{}, cfg);
			cfg.m_surface_spacing = 0.037f;
			auto const fine = SampleHull(box.m_base, body, WaterFrame{}, TestField{}, cfg);
			PR_EXPECT(FEqlRelative(coarse.m_volume_m3, 4.0f, 1e-4f) && FEqlRelative(fine.m_volume_m3, 4.0f, 1e-4f));
			PR_EXPECT(FEqlRelative(coarse.m_buoyancy_force_ws, fine.m_buoyancy_force_ws, 1e-4f));
			PR_EXPECT(FEqlAbsolute(coarse.m_buoyancy_torque_ws, fine.m_buoyancy_torque_ws, 1.0f));
			PR_EXPECT(FEqlRelative(coarse.m_drag_force_ws, fine.m_drag_force_ws, 1e-3f));
			PR_EXPECT(FEqlRelative(coarse.m_drag_torque_ws, fine.m_drag_torque_ws, 0.02f));
		}

		// Buoyancy plans keep the configured spacing for large primitives and refine small primitives to the minimum sample count.
		PRUnitTestMethod(SurfacePlanMinimumSamples, Extended)
		{
			// A large box already has enough samples, so its buoyancy plan is the plain surface plan.
			auto box = ShapeBox(v4{2.0f, 2.0f, 1.0f, 0.0f});
			PR_EXPECT(surface::BuildPlan(box.m_base, DefaultSurfaceSpacing).m_count >= MinSurfaceSamplesPerPrimitive);
			PR_EXPECT(BuildSurfacePlan(box.m_base, DefaultSurfaceSpacing).m_count == surface::BuildPlan(box.m_base, DefaultSurfaceSpacing).m_count);

			// Small primitives of every type are refined until they reach the minimum.
			auto sphere = ShapeSphere(0.15f);
			auto small_box = ShapeBox(v4{0.1f, 0.2f, 0.05f, 0.0f});
			auto capsule = ShapeLine(0.2f, 0.05f);
			auto triangle = ShapeTriangle(v4{0, 0, 0, 1}, v4{0.1f, 0, 0, 1}, v4{0, 0.1f, 0, 1});
			for (auto const* shape : {&sphere.m_base, &small_box.m_base, &capsule.m_base, &triangle.m_base})
			{
				PR_EXPECT(surface::BuildPlan(*shape, DefaultSurfaceSpacing).m_count < MinSurfaceSamplesPerPrimitive);
				PR_EXPECT(BuildSurfacePlan(*shape, DefaultSurfaceSpacing).m_count >= MinSurfaceSamplesPerPrimitive);
			}

			// A zero-area primitive has no pressure to resolve, so it is not refined.
			auto thin = ShapeLine(0.2f);
			PR_EXPECT(BuildSurfacePlan(thin.m_base, DefaultSurfaceSpacing).m_count == surface::BuildPlan(thin.m_base, DefaultSurfaceSpacing).m_count);
		}

		// A light hollow ball (0.06 kg, r = 0.15 m) floats with a 1.14 cm draft. Only a few cells near the lowest point are wet, but the
		// sampled lift must still float the ball close to the analytic height, without a false lateral force or torque.
		PRUnitTestMethod(ShallowDraftSphereLift, Extended)
		{
			auto const radius = 0.15f;
			auto const mass = 0.06f;
			auto const weight = mass * 9.81f;
			auto sphere = ShapeSphere(radius);

			// Return the sampled hull result with the ball's lowest point 'draft' below the calm water plane.
			auto const sample_at_draft = [&](float draft)
			{
				auto body = BodyState{};
				body.m_gravity_ws = v4{0.0f, 0.0f, -9.81f, 0.0f};
				body.m_o2w = m4x4::Translation(0.0f, 0.0f, radius - draft);
				return SampleHull(sphere.m_base, body, WaterFrame{}, TestField{}, SamplerConfig{});
			};

			// Solve the analytic draft (cap volume = mass/rho) and the sampled equilibrium draft (lift = weight) by bisection.
			auto analytic_lo = 0.0f, analytic_hi = radius;
			auto sampled_lo = 0.0f, sampled_hi = radius;
			for (int i = 0; i != 40; ++i)
			{
				auto const analytic_mid = 0.5f * (analytic_lo + analytic_hi);
				(BallVolumeBelow(radius, analytic_mid - radius) < mass / 1000.0f ? analytic_lo : analytic_hi) = analytic_mid;

				auto const sampled_mid = 0.5f * (sampled_lo + sampled_hi);
				(sample_at_draft(sampled_mid).m_buoyancy_force_ws.z < weight ? sampled_lo : sampled_hi) = sampled_mid;
			}
			auto const analytic_draft = 0.5f * (analytic_lo + analytic_hi);
			auto const sampled_draft = 0.5f * (sampled_lo + sampled_hi);
			PR_EXPECT(FEqlAbsolute(analytic_draft, 0.0114f, 0.0002f));
			PR_EXPECT(FEqlAbsolute(sampled_draft, analytic_draft, 0.002f));

			// At the analytic height the symmetric wet cells must cancel laterally and produce no torque.
			auto const r = sample_at_draft(analytic_draft);
			PR_EXPECT(r.m_valid);
			PR_EXPECT(r.m_buoyancy_force_ws.z > 0.0f);
			PR_EXPECT(std::abs(r.m_buoyancy_force_ws.x) < 1e-3f && std::abs(r.m_buoyancy_force_ws.y) < 1e-3f);
			PR_EXPECT(Length(r.m_buoyancy_torque_ws) < 1e-3f * weight * radius);
		}

		// Fully-submerged box, flat water: buoyancy force is exactly rho*g*V*up (all samples wet, zero
		// gradient), volume is exact, and the centre of buoyancy is at the box centre.
		PRUnitTestMethod(BoxFullySubmergedFlatWater, Extended)
		{
			auto const half = v4{1.0f, 1.0f, 0.5f, 0.0f};
			auto box = ShapeBox(half * 2.0f);

			auto body = BodyState{};
			body.m_gravity_ws = v4{0.0f, 0.0f, -9.81f, 0.0f};

			auto const frame = WaterFrame{}; // up=+Z, ref=origin
			auto const water = TestField{.m_level = 10.0f}; // well above the box
			auto const cfg = SamplerConfig{.m_fluid_density = 1000.0f};

			auto const r = SampleHull(box.m_base, body, frame, water, cfg);

			auto const volume = 8.0f * half.x * half.y * half.z; // 4 m^3
			PR_EXPECT(r.m_valid);
			PR_EXPECT(FEqlRelative(r.m_volume_m3, volume, 1e-3f)); // float-summed over N samples (not bit-exact)
			PR_EXPECT(FEqlRelative(r.m_buoyancy_force_ws, v4{0.0f, 0.0f, 1000.0f * 9.81f * volume, 0.0f}, 1e-3f));
			PR_EXPECT(FEqlAbsolute(r.m_centre_buoyancy_ws.w0(), v4::Zero(), 0.02f));
			PR_EXPECT(Length(r.m_buoyancy_torque_ws) < 20.0f); // symmetric => ~zero (sampling noise floor)
		}

		// A collision-shape pose and its centre of mass remain independent so offset articulation links report torque about the physical mass centre.
		PRUnitTestMethod(OffsetCentreOfMass, Extended)
		{
			auto box = ShapeBox(v4{2.0f, 2.0f, 2.0f, 0.0f});
			auto body = BodyState{
				.m_centre_of_mass_os = v4{1.0f, 0.0f, 0.0f, 0.0f},
				.m_gravity_ws = v4{0.0f, 0.0f, -9.81f, 0.0f},
			};
			auto const water = TestField{.m_level = 10.0f};
			auto const cfg = SamplerConfig{.m_fluid_density = 1000.0f};

			// The symmetric pressure centre remains at the shape origin while its upward force acts one metre left of the centre of mass.
			auto const result = SampleHull(box.m_base, body, WaterFrame{}, water, cfg);
			PR_EXPECT(result.m_valid);
			PR_EXPECT(FEqlAbsolute(result.m_centre_buoyancy_ws.w0(), v4::Zero(), 0.02f));
			PR_EXPECT(FEqlRelative(result.m_buoyancy_torque_ws.y, result.m_buoyancy_force_ws.z, 0.01f));
		}

		// Partially-submerged box: the water-column volume and centre of buoyancy match the analytic clipped-box result.
		PRUnitTestMethod(BoxPartialVsAnalytic, Extended)
		{
			auto const half = v4{1.0f, 1.0f, 0.5f, 0.0f};
			auto box = ShapeBox(half * 2.0f);

			auto const z = 0.0f; // box centred on the water plane -> half submerged
			auto body = BodyState{};
			body.m_o2w = m4x4::Translation(0.0f, 0.0f, z);
			body.m_gravity_ws = v4{0.0f, 0.0f, -9.81f, 0.0f};

			auto const frame = WaterFrame{};
			auto const water = TestField{};
			auto const cfg = SamplerConfig{.m_fluid_density = 1000.0f};

			auto const r = SampleHull(box.m_base, body, frame, water, cfg);

			auto const analytic = SubmergedBoxVolumeCentroid(body.m_o2w, half, 0.0f);
			PR_EXPECT(analytic.m_valid && r.m_valid);
			PR_EXPECT(FEqlRelative(r.m_volume_m3, analytic.m_volume_m3, 0.03f));
			PR_EXPECT(FEqlAbsolute(r.m_centre_buoyancy_ws.w0(), analytic.m_centroid_ws.w0(), 0.03f));
		}

		// Fully-submerged sphere: exact volume + force; partially-submerged sphere converges to the
		// analytic spherical-cap volume.
		PRUnitTestMethod(SphereVolume, Extended)
		{
			auto sphere = ShapeSphere(1.0f);
			auto const full = (4.0f / 3.0f) * static_cast<float>(math::constants<double>::tau_by_2);

			// Fully submerged
			{
				auto body = BodyState{};
				body.m_gravity_ws = v4{0.0f, 0.0f, -9.81f, 0.0f};
				auto const water = TestField{.m_level = 10.0f};
				auto const cfg = SamplerConfig{.m_fluid_density = 1000.0f};
				auto const r = SampleHull(sphere.m_base, body, WaterFrame{}, water, cfg);
				PR_EXPECT(FEqlRelative(r.m_volume_m3, full, 1e-3f)); // float-summed over N samples
				PR_EXPECT(FEqlRelative(r.m_buoyancy_force_ws, v4{0.0f, 0.0f, 1000.0f * 9.81f * full, 0.0f}, 1e-3f));
			}

			// Partially submerged (water at z = 0.3)
			{
				auto body = BodyState{};
				body.m_gravity_ws = v4{0.0f, 0.0f, -9.81f, 0.0f};
				auto const water = TestField{.m_level = 0.3f};
				auto const cfg = SamplerConfig{.m_fluid_density = 1000.0f};
				auto const r = SampleHull(sphere.m_base, body, WaterFrame{}, water, cfg);
				PR_EXPECT(FEqlRelative(r.m_volume_m3, BallVolumeBelow(1.0f, 0.3f), 0.03f));
			}
		}

		// Two overlapping boxes: the sibling cull removes the internal surfaces, so the water columns give the union volume, not
		// the sum of the two volumes.
		PRUnitTestMethod(OverlappingBoxesUnionVolume, Extended)
		{
			// Two boxes, half-extents (1,0.5,0.5), centred at +/-0.5 along X. X spans [-1.5,1.5]=3,
			// Y/Z cross-section 1x1 => union volume = 3. Each box volume = 2, sum = 4, overlap = 1.
			ShapeBuilder sb;
			sb.AddShape(ShapeBox(v4{2.0f, 1.0f, 1.0f, 0.0f}, m4x4::Translation(+0.5f, 0.0f, 0.0f)));
			sb.AddShape(ShapeBox(v4{2.0f, 1.0f, 1.0f, 0.0f}, m4x4::Translation(-0.5f, 0.0f, 0.0f)));

			byte_data<16> data;
			MassProperties mp;
			v4 model_to_com;
			auto* hull = sb.BuildShape(data, mp, model_to_com);
			PR_EXPECT(hull != nullptr && hull->m_type == EShape::Array);
			PR_EXPECT(FEqlAbsolute(mp.m_centre_of_mass.w0(), v4::Zero(), 1e-3f)); // symmetric => CoM at origin

			auto body = BodyState{};
			body.m_gravity_ws = v4{0.0f, 0.0f, -9.81f, 0.0f};
			auto const water = TestField{.m_level = 10.0f};
			auto const cfg = SamplerConfig{.m_fluid_density = 1000.0f};

			auto const r = SampleHull(*hull, body, WaterFrame{}, water, cfg);
			PR_EXPECT(r.m_valid);
			PR_EXPECT(FEqlRelative(r.m_volume_m3, 3.0f, 0.03f)); // union, not 4
		}

		// Gravity along -Y (up = +Y): a fully-submerged box must report the full volume and a buoyancy
		// force purely along +up. This guards against any world-Z assumption leaking into the wet test
		// or the FK force. A non-zero gradient field is also checked to exercise the lifted slope term.
		PRUnitTestMethod(NonZGravityFrame, Extended)
		{
			auto box = ShapeBox(v4{2.0f, 2.0f, 1.0f, 0.0f});
			auto const volume = 2.0f * 2.0f * 1.0f;

			auto body = BodyState{};
			body.m_gravity_ws = v4{0.0f, -9.81f, 0.0f, 0.0f}; // gravity along -Y
			auto const frame = WaterFrame::FromGravity(body.m_gravity_ws, v4::Origin());
			PR_EXPECT(FEqlAbsolute(frame.m_up, v4{0.0f, 1.0f, 0.0f, 0.0f}, 1e-5f));

			auto const cfg = SamplerConfig{.m_fluid_density = 1000.0f};

			// Flat field (fully submerged): force is exactly rho*g*V*up.
			{
				auto const water = TestField{.m_level = 10.0f};
				auto const r = SampleHull(box.m_base, body, frame, water, cfg);
				PR_EXPECT(FEqlRelative(r.m_volume_m3, volume, 1e-3f)); // float-summed over N samples
				PR_EXPECT(FEqlRelative(r.m_buoyancy_force_ws, (1000.0f * 9.81f * volume) * frame.m_up, 1e-3f));
			}

			// Sloped field along t0: force = rho*g*V*(up - a*t0).
			{
				auto const a = 0.1f;
				auto const water = TestField{.m_level = 10.0f, .m_a = a};
				auto const r = SampleHull(box.m_base, body, frame, water, cfg);
				auto const expected = (1000.0f * 9.81f * volume) * (frame.m_up - a * frame.m_t0);
				PR_EXPECT(FEqlRelative(r.m_buoyancy_force_ws, expected, 1e-3f));
			}
		}

		// Fully-dry hull: no submerged volume => invalid result, zero forces.
		PRUnitTestMethod(FullyDryHull, Extended)
		{
			auto box = ShapeBox(v4{1.0f, 1.0f, 1.0f, 0.0f});
			auto body = BodyState{};
			body.m_o2w = m4x4::Translation(0.0f, 0.0f, 10.0f); // high above water
			body.m_gravity_ws = v4{0.0f, 0.0f, -9.81f, 0.0f};
			auto const water = TestField{}; // level 0
			auto const cfg = SamplerConfig{.m_fluid_density = 1000.0f};

			auto const r = SampleHull(box.m_base, body, WaterFrame{}, water, cfg);
			PR_EXPECT(!r.m_valid);
			PR_EXPECT(FEqlAbsolute(r.m_buoyancy_force_ws, v4::Zero(), 1e-6f));
			PR_EXPECT(FEqlAbsolute(r.m_drag_force_ws, v4::Zero(), 1e-6f));
		}

		// Quadratic drag on a translating fully-submerged box: only the leading (+X) face contributes
		// (v_n > 0); the drag force opposes motion with magnitude ~ 0.5*rho*Cd*A_front*v^2.
		PRUnitTestMethod(QuadraticDragTranslation, Extended)
		{
			auto const half = v4{0.5f, 0.5f, 0.5f, 0.0f};
			auto box = ShapeBox(half * 2.0f);
			auto const a_front = (2.0f * half.y) * (2.0f * half.z); // +X face area = 1

			auto body = BodyState{};
			body.m_gravity_ws = v4{0.0f, 0.0f, -9.81f, 0.0f};
			body.m_vel_lin_ws = v4{1.0f, 0.0f, 0.0f, 0.0f};

			auto const water = TestField{.m_level = 10.0f}; // fully submerged
			auto const cfg = SamplerConfig
			{
				.m_fluid_density = 1000.0f,
				.m_linear_drag_time_constant_s = 0.0f, // linear drag off
				.m_quadratic_drag_coefficient = 1.0f, // Cd = 1
			};

			auto const r = SampleHull(box.m_base, body, WaterFrame{}, water, cfg);

			auto const expected_fx = -0.5f * 1000.0f * 1.0f * a_front * 1.0f; // v^2 = 1
			PR_EXPECT(FEqlRelative(r.m_drag_force_ws.x, expected_fx, 0.06f));
			PR_EXPECT(std::abs(r.m_drag_force_ws.y) < 0.05f * std::abs(expected_fx));
			PR_EXPECT(std::abs(r.m_drag_force_ws.z) < 0.05f * std::abs(expected_fx));
		}

		// Tangential drag on a translating fully submerged unit box acts on the four faces parallel to
		// motion. Their combined area is 4 m^2, giving F_x = -0.5*rho*Ct*A_tangent*|v_t|*v_t.
		PRUnitTestMethod(TangentialDragTranslation, Extended)
		{
			auto box = ShapeBox(v4{1.0f, 1.0f, 1.0f, 0.0f});
			auto body = BodyState{};
			body.m_gravity_ws = v4{0.0f, 0.0f, -9.81f, 0.0f};
			body.m_vel_lin_ws = v4{1.0f, 0.0f, 0.0f, 0.0f};

			auto const water = TestField{.m_level = 10.0f};
			auto const cfg = SamplerConfig{
				.m_fluid_density = 1000.0f,
				.m_linear_drag_time_constant_s = 0.0f,
				.m_quadratic_drag_coefficient = 0.0f,
				.m_tangential_drag_coefficient = 0.1f,
			};

			auto const r = SampleHull(box.m_base, body, WaterFrame{}, water, cfg);
			auto const tangent_area = 4.0f;
			auto const expected_fx =
				-0.5f *
				cfg.m_fluid_density *
				cfg.m_tangential_drag_coefficient *
				tangent_area;

			PR_EXPECT(FEqlRelative(r.m_drag_force_ws.x, expected_fx, 0.06f));
			PR_EXPECT(std::abs(r.m_drag_force_ws.y) < 0.05f * std::abs(expected_fx));
			PR_EXPECT(std::abs(r.m_drag_force_ws.z) < 0.05f * std::abs(expected_fx));
			PR_EXPECT(FEqlAbsolute(r.m_drag_torque_ws, v4::Zero(), std::abs(expected_fx) * 0.05f));
		}

		// The optional debug collector records every sample classification and the per-primitive
		// accepted buoyancy partials, without changing the physical result.
		PRUnitTestMethod(DebugCollector, Extended)
		{
			// Single fully-submerged box: every surface sample is wet and owned by primitive 0; the
			// summed per-primitive partials reconstruct the total buoyancy force exactly.
			{
				auto box = ShapeBox(v4{2.0f, 2.0f, 1.0f, 0.0f});
				auto body = BodyState{};
				body.m_gravity_ws = v4{0.0f, 0.0f, -9.81f, 0.0f};
				auto const water = TestField{.m_level = 10.0f};
				auto const cfg = SamplerConfig{.m_fluid_density = 1000.0f}; // drag off

				auto dbg = SampleDebug{};
				auto const r = SampleHull(box.m_base, body, WaterFrame{}, water, cfg, &dbg);

				// Per-primitive accumulators sized to the single primitive.
				PR_EXPECT(dbg.m_prim_buoy_force_ws.size() == 1u);

				// Count classifications.
				auto surf_active = 0, surf_dry = 0, surf_culled = 0;
				for (auto const& s : dbg.m_samples)
				{
					switch (s.m_kind)
					{
						case ESampleKind::SurfaceActive: ++surf_active; break;
						case ESampleKind::SurfaceDry: ++surf_dry; break;
						case ESampleKind::SurfaceCulled: ++surf_culled; break;
						default: throw std::runtime_error("Unknown sample kind");
					}
				}

				// Fully submerged single box: all surface samples are wet and active, none dry or culled.
				PR_EXPECT(surf_active == static_cast<int>(BuildSurfacePlan(box.m_base, cfg.m_surface_spacing).m_count));
				PR_EXPECT(surf_dry == 0);
				PR_EXPECT(surf_culled == 0);

				// Summed per-primitive partials reconstruct the total buoyancy force.
				PR_EXPECT(FEqlRelative(dbg.m_prim_buoy_force_ws[0], r.m_buoyancy_force_ws, 1e-4f));

				// Per-primitive wet centre matches the diagnostic centre of buoyancy.
				PR_EXPECT(FEqlAbsolute(dbg.PrimWetCentre(0).w0(), r.m_centre_buoyancy_ws.w0(), 1e-3f));
			}

			// Overlapping boxes: the sibling cull produces SurfaceCulled records, and the
			// surviving union force still equals the aggregate result.
			{
				ShapeBuilder sb;
				sb.AddShape(ShapeBox(v4{2.0f, 1.0f, 1.0f, 0.0f}, m4x4::Translation(+0.5f, 0.0f, 0.0f)));
				sb.AddShape(ShapeBox(v4{2.0f, 1.0f, 1.0f, 0.0f}, m4x4::Translation(-0.5f, 0.0f, 0.0f)));

				byte_data<16> data;
				MassProperties mp;
				v4 model_to_com;
				auto* hull = sb.BuildShape(data, mp, model_to_com);

				auto body = BodyState{};
				body.m_gravity_ws = v4{0.0f, 0.0f, -9.81f, 0.0f};
				auto const water = TestField{.m_level = 10.0f};
				auto const cfg = SamplerConfig{.m_fluid_density = 1000.0f};

				auto dbg = SampleDebug{};
				auto const r = SampleHull(*hull, body, WaterFrame{}, water, cfg, &dbg);

				PR_EXPECT(dbg.m_prim_buoy_force_ws.size() == 2u);

				auto surf_culled = 0;
				for (auto const& s : dbg.m_samples)
				{
					if (s.m_kind == ESampleKind::SurfaceCulled)
						++surf_culled;
				}

				// The boxes overlap, so the cull must reject the internal surface samples.
				PR_EXPECT(surf_culled > 0);

				// Summed per-primitive partials reconstruct the total union buoyancy force. The aggregate
				// is one running sum while the partials are summed per-primitive, so the float addition
				// order differs - allow float-summation noise.
				auto const sum = dbg.m_prim_buoy_force_ws[0] + dbg.m_prim_buoy_force_ws[1];
				PR_EXPECT(FEqlRelative(sum, r.m_buoyancy_force_ws, 1e-3f));
			}

			// Half-submerged box: both wet and dry surface samples are recorded.
			{
				auto box = ShapeBox(v4{2.0f, 2.0f, 2.0f, 0.0f});
				auto body = BodyState{};
				body.m_gravity_ws = v4{0.0f, 0.0f, -9.81f, 0.0f}; // box centred on z=0 water plane
				auto const water = TestField{};
				auto const cfg = SamplerConfig{.m_fluid_density = 1000.0f};

				auto dbg = SampleDebug{};
				SampleHull(box.m_base, body, WaterFrame{}, water, cfg, &dbg);

				auto surf_wet = 0, surf_dry = 0;
				for (auto const& s : dbg.m_samples)
				{
					if (s.m_kind == ESampleKind::SurfaceActive) ++surf_wet;
					if (s.m_kind == ESampleKind::SurfaceDry) ++surf_dry;
				}
				PR_EXPECT(surf_wet > 0);
				PR_EXPECT(surf_dry > 0);
			}
		}
	};
}
#endif
