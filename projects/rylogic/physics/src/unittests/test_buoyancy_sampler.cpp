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

	void ForceLink_BuoyancySampler() {}

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
			v2 PressureGradient(v2, float) const { return v2{m_a, m_b}; }
			v4 Velocity(v4) const { return m_velocity; }
		};

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
		// Pin the low-discrepancy sequence + hash so future GPU implementations can match exactly.
		PRUnitTestMethod(GoldenSampleGeneration)
		{
			// Radical inverse known values.
			PR_EXPECT(FEqlAbsolute(RadicalInverse(1, 2), 0.5f, 1e-6f));
			PR_EXPECT(FEqlAbsolute(RadicalInverse(2, 2), 0.25f, 1e-6f));
			PR_EXPECT(FEqlAbsolute(RadicalInverse(3, 2), 0.75f, 1e-6f));
			PR_EXPECT(FEqlAbsolute(RadicalInverse(1, 3), 1.0f / 3.0f, 1e-6f));
			PR_EXPECT(FEqlAbsolute(RadicalInverse(1, 5), 0.2f, 1e-6f));
			PR_EXPECT(FEqlAbsolute(RadicalInverse(5, 5), 0.04f, 1e-6f));

			// Sample index is deterministic, >= 1, and bounded by the offset window.
			auto const idx0 = SampleIndex(7, 0, 0);
			PR_EXPECT(idx0 == SampleIndex(7, 0, 0));
			PR_EXPECT(idx0 >= 1u);
			PR_EXPECT(SampleIndex(7, 0, 3) == idx0 + 3u);
			PR_EXPECT(idx0 < 1u + 4096u);

			// Hash is deterministic and distinguishes distinct inputs.
			PR_EXPECT(HashU32(123) == HashU32(123));
			PR_EXPECT(HashU32(1) != HashU32(2));
		}

		// Fully-submerged box, flat water: buoyancy force is exactly rho*g*V*up (all samples wet, zero
		// gradient), volume is exact, and the centre of buoyancy is at the box centre.
		PRUnitTestMethod(BoxFullySubmergedFlatWater)
		{
			auto const half = v4{1.0f, 1.0f, 0.5f, 0.0f};
			auto box = ShapeBox(half * 2.0f);

			auto body = BodyState{};
			body.m_gravity_ws = v4{0.0f, 0.0f, -9.81f, 0.0f};

			auto const frame = WaterFrame{}; // up=+Z, ref=origin
			auto const water = TestField{.m_level = 10.0f}; // well above the box
			auto const cfg = SamplerConfig{.m_fluid_density = 1000.0f};

			auto const r = SampleHull(box.m_base, 1, body, frame, water, cfg, 20000, 0);

			auto const volume = 8.0f * half.x * half.y * half.z; // 4 m^3
			PR_EXPECT(r.m_valid);
			PR_EXPECT(FEqlRelative(r.m_volume_m3, volume, 1e-3f)); // float-summed over N samples (not bit-exact)
			PR_EXPECT(FEqlRelative(r.m_buoyancy_force_ws, v4{0.0f, 0.0f, 1000.0f * 9.81f * volume, 0.0f}, 1e-3f));
			PR_EXPECT(FEqlAbsolute(r.m_centre_buoyancy_ws.w0(), v4::Zero(), 0.02f));
			PR_EXPECT(Length(r.m_buoyancy_torque_ws) < 20.0f); // symmetric => ~zero (sampling noise floor)
		}

		// Partially-submerged box: sampled volume and centre of buoyancy converge to the analytic
		// clipped-box result (with Monte-Carlo tolerance for the waterline discontinuity).
		PRUnitTestMethod(BoxPartialVsAnalytic)
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

			auto const r = SampleHull(box.m_base, 2, body, frame, water, cfg, 40000, 0);

			auto const analytic = SubmergedBoxVolumeCentroid(body.m_o2w, half, 0.0f);
			PR_EXPECT(analytic.m_valid && r.m_valid);
			PR_EXPECT(FEqlRelative(r.m_volume_m3, analytic.m_volume_m3, 0.03f));
			PR_EXPECT(FEqlAbsolute(r.m_centre_buoyancy_ws.w0(), analytic.m_centroid_ws.w0(), 0.03f));
		}

		// Fully-submerged sphere: exact volume + force; partially-submerged sphere converges to the
		// analytic spherical-cap volume.
		PRUnitTestMethod(SphereVolume)
		{
			auto sphere = ShapeSphere(1.0f);
			auto const full = (4.0f / 3.0f) * static_cast<float>(math::constants<double>::tau_by_2);

			// Fully submerged
			{
				auto body = BodyState{};
				body.m_gravity_ws = v4{0.0f, 0.0f, -9.81f, 0.0f};
				auto const water = TestField{.m_level = 10.0f};
				auto const cfg = SamplerConfig{.m_fluid_density = 1000.0f};
				auto const r = SampleHull(sphere.m_base, 3, body, WaterFrame{}, water, cfg, 20000, 0);
				PR_EXPECT(FEqlRelative(r.m_volume_m3, full, 1e-3f)); // float-summed over N samples
				PR_EXPECT(FEqlRelative(r.m_buoyancy_force_ws, v4{0.0f, 0.0f, 1000.0f * 9.81f * full, 0.0f}, 1e-3f));
			}

			// Partially submerged (water at z = 0.3)
			{
				auto body = BodyState{};
				body.m_gravity_ws = v4{0.0f, 0.0f, -9.81f, 0.0f};
				auto const water = TestField{.m_level = 0.3f};
				auto const cfg = SamplerConfig{.m_fluid_density = 1000.0f};
				auto const r = SampleHull(sphere.m_base, 4, body, WaterFrame{}, water, cfg, 40000, 0);
				PR_EXPECT(FEqlRelative(r.m_volume_m3, BallVolumeBelow(1.0f, 0.3f), 0.03f));
			}
		}

		// Two overlapping boxes: the volume pass deduplicates the overlap region (union volume, not
		// the sum of the two volumes). This exercises the lowest-index-sibling cull.
		PRUnitTestMethod(OverlappingBoxesUnionVolume)
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

			auto const r = SampleHull(*hull, 5, body, WaterFrame{}, water, cfg, 60000, 0);
			PR_EXPECT(r.m_valid);
			PR_EXPECT(FEqlRelative(r.m_volume_m3, 3.0f, 0.03f)); // union, not 4
		}

		// Gravity along -Y (up = +Y): a fully-submerged box must report the full volume and a buoyancy
		// force purely along +up. This guards against any world-Z assumption leaking into the wet test
		// or the FK force. A non-zero gradient field is also checked to exercise the lifted slope term.
		PRUnitTestMethod(NonZGravityFrame)
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
				auto const r = SampleHull(box.m_base, 6, body, frame, water, cfg, 20000, 0);
				PR_EXPECT(FEqlRelative(r.m_volume_m3, volume, 1e-3f)); // float-summed over N samples
				PR_EXPECT(FEqlRelative(r.m_buoyancy_force_ws, (1000.0f * 9.81f * volume) * frame.m_up, 1e-3f));
			}

			// Sloped field along t0: force = rho*g*V*(up - a*t0).
			{
				auto const a = 0.1f;
				auto const water = TestField{.m_level = 10.0f, .m_a = a};
				auto const r = SampleHull(box.m_base, 7, body, frame, water, cfg, 20000, 0);
				auto const expected = (1000.0f * 9.81f * volume) * (frame.m_up - a * frame.m_t0);
				PR_EXPECT(FEqlRelative(r.m_buoyancy_force_ws, expected, 1e-3f));
			}
		}

		// Fully-dry hull: no submerged volume => invalid result, zero forces.
		PRUnitTestMethod(FullyDryHull)
		{
			auto box = ShapeBox(v4{1.0f, 1.0f, 1.0f, 0.0f});
			auto body = BodyState{};
			body.m_o2w = m4x4::Translation(0.0f, 0.0f, 10.0f); // high above water
			body.m_gravity_ws = v4{0.0f, 0.0f, -9.81f, 0.0f};
			auto const water = TestField{}; // level 0
			auto const cfg = SamplerConfig{.m_fluid_density = 1000.0f};

			auto const r = SampleHull(box.m_base, 8, body, WaterFrame{}, water, cfg, 8000, 8000);
			PR_EXPECT(!r.m_valid);
			PR_EXPECT(FEqlAbsolute(r.m_buoyancy_force_ws, v4::Zero(), 1e-6f));
			PR_EXPECT(FEqlAbsolute(r.m_drag_force_ws, v4::Zero(), 1e-6f));
		}

		// Quadratic drag on a translating fully-submerged box: only the leading (+X) face contributes
		// (v_n > 0); the drag force opposes motion with magnitude ~ 0.5*rho*Cd*A_front*v^2.
		PRUnitTestMethod(QuadraticDragTranslation)
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
				.m_drag_time_constant_s = 0.0f,       // linear drag off
				.m_quadratic_drag_coefficient = 1.0f, // Cd = 1
			};

			auto const r = SampleHull(box.m_base, 9, body, WaterFrame{}, water, cfg, 0, 24000);

			auto const expected_fx = -0.5f * 1000.0f * 1.0f * a_front * 1.0f; // v^2 = 1
			PR_EXPECT(FEqlRelative(r.m_drag_force_ws.x, expected_fx, 0.06f));
			PR_EXPECT(std::abs(r.m_drag_force_ws.y) < 0.05f * std::abs(expected_fx));
			PR_EXPECT(std::abs(r.m_drag_force_ws.z) < 0.05f * std::abs(expected_fx));
		}

		// Tangential drag on a translating fully submerged unit box acts on the four faces parallel to
		// motion. Their combined area is 4 m^2, giving F_x = -0.5*rho*Ct*A_tangent*|v_t|*v_t.
		PRUnitTestMethod(TangentialDragTranslation)
		{
			auto box = ShapeBox(v4{1.0f, 1.0f, 1.0f, 0.0f});
			auto body = BodyState{};
			body.m_gravity_ws = v4{0.0f, 0.0f, -9.81f, 0.0f};
			body.m_vel_lin_ws = v4{1.0f, 0.0f, 0.0f, 0.0f};

			auto const water = TestField{.m_level = 10.0f};
			auto const cfg = SamplerConfig{
				.m_fluid_density = 1000.0f,
				.m_drag_time_constant_s = 0.0f,
				.m_quadratic_drag_coefficient = 0.0f,
				.m_tangential_drag_coefficient = 0.1f,
			};

			auto const r = SampleHull(box.m_base, 10, body, WaterFrame{}, water, cfg, 0, 24000);
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
		// accepted buoyancy partials, without changing the physical result. It also forces the surface
		// pass to run with drag disabled so surface classifications are still captured.
		PRUnitTestMethod(DebugCollector)
		{
			// Single fully-submerged box: every volume sample is wet and owned by primitive 0; the
			// summed per-primitive partials reconstruct the total buoyancy force exactly.
			{
				auto box = ShapeBox(v4{2.0f, 2.0f, 1.0f, 0.0f});
				auto body = BodyState{};
				body.m_gravity_ws = v4{0.0f, 0.0f, -9.81f, 0.0f};
				auto const water = TestField{.m_level = 10.0f};
				auto const cfg = SamplerConfig{.m_fluid_density = 1000.0f}; // drag off

				auto dbg = SampleDebug{};
				auto const r = SampleHull(box.m_base, 11, body, WaterFrame{}, water, cfg, 8000, 8000, &dbg);

				// Per-primitive accumulators sized to the single primitive.
				PR_EXPECT(dbg.m_prim_buoy_force_ws.size() == 1u);

				// Count classifications.
				auto vol_wet = 0, vol_dry = 0, vol_culled = 0, surf_active = 0, surf_dry = 0;
				for (auto const& s : dbg.m_samples)
				{
					switch (s.m_kind)
					{
						case ESampleKind::VolumeWet: ++vol_wet; break;
						case ESampleKind::VolumeDry: ++vol_dry; break;
						case ESampleKind::VolumeCulled: ++vol_culled; break;
						case ESampleKind::SurfaceActive: ++surf_active; break;
						case ESampleKind::SurfaceDry: ++surf_dry; break;
						case ESampleKind::SurfaceCulled: break;
					}
				}

				// Fully submerged single box: all volume samples wet, none dry/culled.
				PR_EXPECT(vol_wet == 8000);
				PR_EXPECT(vol_dry == 0);
				PR_EXPECT(vol_culled == 0);

				// Surface pass ran despite drag being off (debug gate); all surface samples are wet+active.
				PR_EXPECT(surf_active == 8000);
				PR_EXPECT(surf_dry == 0);

				// Summed per-primitive partials reconstruct the total buoyancy force.
				PR_EXPECT(FEqlRelative(dbg.m_prim_buoy_force_ws[0], r.m_buoyancy_force_ws, 1e-4f));

				// Per-primitive wet centre matches the diagnostic centre of buoyancy.
				PR_EXPECT(FEqlAbsolute(dbg.PrimWetCentre(0).w0(), r.m_centre_buoyancy_ws.w0(), 1e-3f));
			}

			// Overlapping boxes: the lowest-index-sibling cull produces VolumeCulled records, and the
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
				auto const r = SampleHull(*hull, 12, body, WaterFrame{}, water, cfg, 20000, 0, &dbg);

				PR_EXPECT(dbg.m_prim_buoy_force_ws.size() == 2u);

				auto vol_culled = 0;
				for (auto const& s : dbg.m_samples)
					if (s.m_kind == ESampleKind::VolumeCulled)
						++vol_culled;

				// The boxes overlap, so the cull must reject some second-primitive samples.
				PR_EXPECT(vol_culled > 0);

				// Summed per-primitive partials reconstruct the total union buoyancy force. The aggregate
				// is one running sum while the partials are summed per-primitive, so the float addition
				// order differs - allow float-summation noise over the ~20k samples.
				auto const sum = dbg.m_prim_buoy_force_ws[0] + dbg.m_prim_buoy_force_ws[1];
				PR_EXPECT(FEqlRelative(sum, r.m_buoyancy_force_ws, 1e-3f));
			}

			// Half-submerged box: both wet and dry volume samples are recorded.
			{
				auto box = ShapeBox(v4{2.0f, 2.0f, 2.0f, 0.0f});
				auto body = BodyState{};
				body.m_gravity_ws = v4{0.0f, 0.0f, -9.81f, 0.0f}; // box centred on z=0 water plane
				auto const water = TestField{};
				auto const cfg = SamplerConfig{.m_fluid_density = 1000.0f};

				auto dbg = SampleDebug{};
				SampleHull(box.m_base, 13, body, WaterFrame{}, water, cfg, 8000, 0, &dbg);

				auto vol_wet = 0, vol_dry = 0;
				for (auto const& s : dbg.m_samples)
				{
					if (s.m_kind == ESampleKind::VolumeWet) ++vol_wet;
					if (s.m_kind == ESampleKind::VolumeDry) ++vol_dry;
				}
				PR_EXPECT(vol_wet > 0);
				PR_EXPECT(vol_dry > 0);
			}
		}
	};
}
#endif
