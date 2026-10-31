//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Deterministic CPU buoyancy sampler.
//
// This is the reference oracle for the GPU buoyancy pass (gpu_buoyancy.hlsl). Surface emission compiles the shared
// surface_sampling.hlsli implementation and uses the same per-sample weights, cull rules, and force terms, so the GPU
// kernels can be validated against it.
//
// Concept:
//  - Buoyancy is a SURFACE integral of water pressure. Each primitive's surface is covered by samples
//    (point, outward normal, area dA), and each wet sample feels the hydrostatic pressure at its depth
//    along its inward normal. A partly wet cell uses the mean depth of its wet part, so forces change
//    smoothly as cells cross the waterline. This needs no surface reconstruction (no CSG / seam / cap
//    clipping) and handles arbitrary overlapping, possibly non-convex composites.
//  - The vertical water column between each wet sample and the water surface has signed volume
//    -n.up * (depth * wet area). The columns sum to the submerged volume, so they carry the volume
//    terms: the wave's lateral pressure force, linear/angular volume damping, and the wet volume and
//    centre of buoyancy diagnostics.
//  - Quadratic form drag and tangential surface drag act on the same wet surface samples.
//  - Overlap is deduplicated by sibling-cull: a surface sample lies on the union boundary iff its
//    exterior side is outside every OTHER primitive, so it is dropped if any other sibling contains the
//    slightly outward-offset point (or strictly contains the sample). This avoids counting
//    internal/embedded surfaces and correctly drops shared faces of abutting primitives. Coplanar
//    faces of overlapping primitives are kept only by the lowest-index primitive.
//  - Every sample carries its own area weight; reductions are weighted sums, never count x constant,
//    because different primitives have different sample densities.
//
// Gravity frame (no baked-in gravity direction):
//  The water surface is a height field along the local "up" axis = -normalize(gravity), NOT world
//  Z. All wet/dry tests, the pressure force, and the drag velocity are expressed against a
//  WaterFrame {up, t0, t1, ref}; world-Z flat water is just the special case up=+Z,t0=+X,t1=+Y.
//
// The water surface is supplied as a template parameter so this header does not depend on the
// physics engine, and so gravity-frame water fields can be plugged in for tests. A water field
// type must provide:
//    float Height(v2 uv) const;                         // signed height along 'up' at planar coords (u,v), relative to 'ref'
//    v2    Slope(v2 uv) const;                          // surface slope (dh/du, dh/dv) at planar coords (u,v)
//    v4    PressureGradient(v2 uv, float gravity) const;// dimensionless dynamic pressure gradient (-a_fluid/g) along (t0, t1, up)
//    v4    Velocity(v4 pos_ws) const;                   // world-space fluid velocity (w = 0)
#pragma once
#include <vector>
#include <cstdint>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <algorithm>
#include "pr/collision/shape.h"
#include "pr/collision/shape_box.h"
#include "pr/collision/shape_sphere.h"
#include "pr/collision/shape_line.h"
#include "pr/collision/shape_triangle.h"
#include "pr/collision/shape_polytope.h"
#include "pr/collision/shape_array.h"
#include "pr/physics/surface/surface_sampling.h"
#include "pr/physics/buoyancy/buoyancy_primitives.h"

namespace pr::physics::buoyancy
{
	// Tunable fluid parameters used by the sampler. Mirrors the runtime GpuBuoyancy::Config fields
	// the sampler cares about, but kept independent so this header has no engine dependency.
	struct SamplerConfig
	{
		float m_fluid_density = 1000.0f;            // kg/m^3
		float m_linear_drag_time_constant_s = 0.0f; // translational water-column drag e-fold time; <= 0 disables it
		float m_angular_drag_time_constant_s = 0.0f; // rotational water-column drag e-fold time; <= 0 disables it
		float m_quadratic_drag_coefficient = 0.0f;  // form-drag Cd; <= 0 disables quadratic drag
		float m_tangential_drag_coefficient = 0.0f; // surface-shear Ct; <= 0 disables tangential drag
		float m_added_mass_coefficient = 0.0f;      // Ca; the wave force becomes the Morison inertia force (1 + Ca)*rho*dV*a_fluid
		float m_surface_spacing = DefaultSurfaceSpacing; // maximum surface cell diameter in shape-local units (see BuildSurfacePlan)
	};

	// Rigid-body kinematics needed to place samples in world space and evaluate point velocities.
	struct BodyState
	{
		m4x4 m_o2w = m4x4::Identity();   // collision-shape root -> world
		v4 m_centre_of_mass_os = v4::Zero(); // centre of mass relative to the collision-shape root
		v4 m_gravity_ws = v4::Zero();    // world-space gravity for this body (w = 0)
		v4 m_vel_lin_ws = v4::Zero();    // linear velocity of the centre of mass (w = 0)
		v4 m_omega_ws = v4::Zero();      // angular velocity (w = 0)
	};

	// Gravity-aligned water frame. 'up' is the local opposite-of-gravity direction; (t0,t1) span the
	// plane perpendicular to 'up'; 'ref' is a point on the still-water surface. The height field is
	// parameterised over (t0,t1) and measured along 'up'.
	struct WaterFrame
	{
		v4 m_up = v4::ZAxis();
		v4 m_t0 = v4::XAxis();
		v4 m_t1 = v4::YAxis();
		v4 m_ref = v4::Origin();

		// Build a frame from a gravity vector. 'up' = -normalize(gravity); tangents are arbitrary but
		// orthonormal. Falls back to world axes when gravity is ~zero.
		static WaterFrame FromGravity(v4 gravity_ws, v4 ref)
		{
			auto frame = WaterFrame{};
			frame.m_ref = ref.w1();

			auto const g_len = Length(gravity_ws.w0());
			frame.m_up = g_len > math::constants<float>::tiny ? Normalise(-gravity_ws.w0()) : v4::ZAxis();
			frame.m_t0 = Normalise(Perpendicular(frame.m_up));
			frame.m_t1 = Cross(frame.m_up, frame.m_t0);
			return frame;
		}
	};

	// Aggregated buoyancy + drag result for a hull (one rigid body's collision shape).
	struct HullResult
	{
		bool m_valid = false;             // true when any submerged volume was sampled
		float m_volume_m3 = 0.0f;         // submerged union volume, summed from the water columns (diagnostic)
		v4 m_centre_buoyancy_ws = v4::Zero();
		v4 m_buoyancy_force_ws = v4::Zero();
		v4 m_buoyancy_torque_ws = v4::Zero();
		v4 m_drag_force_ws = v4::Zero();
		v4 m_drag_torque_ws = v4::Zero();
	};

	// Classification of a single sample emitted during SampleHull, recorded only when a debug
	// collector is supplied. Mirrors the exact cull/wet branches so a visualiser can render the
	// real sampler decisions rather than an independent re-derivation.
	enum class ESampleKind
	{
		SurfaceActive,   // surface sample on the wetted union boundary, contributes buoyancy and drag
		SurfaceDry,      // surface sample above the water surface
		SurfaceCulled,   // surface sample interior to another primitive (not on the union boundary)
	};

	// One recorded sample for debug visualisation: world-space position, world-space outward
	// normal, owning primitive index and classification.
	struct SampleDebugRecord
	{
		v4 m_pos_ws = v4::Origin();
		v4 m_normal_ws = v4::Zero();
		int m_prim_index = 0;
		ESampleKind m_kind = ESampleKind::SurfaceDry;
	};

	// Optional out-parameter collector for SampleHull. When passed, every sample's classification is
	// recorded and per-primitive accepted buoyancy partials are accumulated, so a visualiser can draw
	// the sample cloud and per-primitive force arrows. Data-only: no render/LDraw dependencies here.
	struct SampleDebug
	{
		std::vector<SampleDebugRecord> m_samples;   // every classified surface sample
		std::vector<v4> m_prim_buoy_force_ws;       // per-primitive accepted buoyancy force
		std::vector<v4> m_prim_wet_moment_ws;       // per-primitive sum(column_centroid_ws * dV) over wet samples
		std::vector<float> m_prim_wet_volume;       // per-primitive signed water-column volume

		// Clear the sample list and (re)size the per-primitive accumulators for a fresh hull pass.
		void Reset(size_t prim_count)
		{
			m_samples.clear();
			m_prim_buoy_force_ws.assign(prim_count, v4::Zero());
			m_prim_wet_moment_ws.assign(prim_count, v4::Zero());
			m_prim_wet_volume.assign(prim_count, 0.0f);
		}

		// World-space wet centroid of a primitive (moment / volume), or the origin when fully dry.
		v4 PrimWetCentre(size_t k) const
		{
			return (k < m_prim_wet_volume.size() && m_prim_wet_volume[k] > 0.0f)
				? (m_prim_wet_moment_ws[k] / m_prim_wet_volume[k]).w1()
				: v4::Origin();
		}
	};

	//
	// Inside tests (shape-local space). 'eps' is a small slack so boundary samples register.
	//

	// Return true if 'p_local' is inside (or within 'eps' of) the convex primitive.
	inline bool ContainsLocal(collision::Shape const& shape, v4 p_local, float eps)
	{
		using namespace collision;
		switch (shape.m_type)
		{
			case EShape::Box:
			{
				auto const& box = shape_cast<ShapeBox>(shape);
				return
					std::abs(p_local.x) <= box.m_radius.x + eps &&
					std::abs(p_local.y) <= box.m_radius.y + eps &&
					std::abs(p_local.z) <= box.m_radius.z + eps;
			}
			case EShape::Sphere:
			{
				auto const& sph = shape_cast<ShapeSphere>(shape);
				auto const r = sph.m_radius + eps;
				return LengthSq(p_local.w0()) <= r * r;
			}
			case EShape::Line:
			{
				// Inside a capsule means within 'radius' of the nearest point on its Z-axis segment.
				auto const& line = shape_cast<ShapeLine>(shape);
				auto const z = std::clamp(p_local.z, -line.m_hlength, line.m_hlength);
				auto const d = v4{ p_local.x, p_local.y, p_local.z - z, 0.0f };
				auto const r = line.m_radius + eps;
				return LengthSq(d) <= r * r;
			}
			case EShape::Polytope:
			{
				// Convex hull with outward face planes: inside iff on the negative side of every plane.
				auto const& poly = shape_cast<ShapePolytope>(shape);
				for (int f = 0; f != poly.m_face_count; ++f)
				{
					if (Distance(poly.face(f).m_plane, p_local.w1()) > eps)
						return false;
				}
				return poly.m_face_count != 0;
			}
			case EShape::Triangle:
			{
				// Zero-volume: never "contains" a point for sibling-cull purposes.
				return false;
			}
			default:
			{
				return false;
			}
		}
	}

	//
	// Helpers
	//

	// Collect the convex primitive children of a hull. A non-Array shape is treated as a single
	// primitive. The primitive index is the child order (the sibling-cull priority).
	inline std::vector<collision::Shape const*> CollectPrimitives(collision::Shape const& hull)
	{
		using namespace collision;
		auto prims = std::vector<Shape const*>{};
		if (hull.m_type == EShape::Array)
		{
			auto const& arr = shape_cast<ShapeArray>(hull);
			for (Shape const* s = arr.begin(), *e = arr.end(); s != e; s = next(s))
				prims.push_back(s);
		}
		else
		{
			prims.push_back(&hull);
		}
		return prims;
	}

	//
	// Top-level sampler
	//

	// Sample buoyancy + drag for a hull. 'water' is any type satisfying the water-field concept
	// documented at the top of this file. Returns aggregated world-space force/torque + diagnostics.
	// When 'debug' is non-null, every sample's classification and the per-primitive accepted buoyancy
	// partials are recorded for visualisation; the physical result is unchanged whether debug is set or not.
	template <typename TWater>
	inline HullResult SampleHull(
		collision::Shape const& hull,
		BodyState const& body,
		WaterFrame const& frame,
		TWater const& water,
		SamplerConfig const& cfg,
		SampleDebug* debug = nullptr)
	{
		using namespace collision;

		// Validate the resolution before doing any work.
		surface::ValidateSpacing(cfg.m_surface_spacing);
		auto result = HullResult{};

		auto const prims = CollectPrimitives(hull);
		if (debug)
			debug->Reset(prims.size());

		// Without gravity there is no "up" for the water and no pressure, so the GPU pass applies nothing.
		auto const g_mag = Length(body.m_gravity_ws.w0());
		if (prims.empty() || !(g_mag > math::constants<float>::tiny))
			return result;

		// Pre-compute per-primitive transforms (shape-local <-> COM-root) and surface plans.
		auto s2r = std::vector<m4x4>(prims.size());
		auto r2s = std::vector<m4x4>(prims.size());
		auto surface_plans = std::vector<surface::Plan>(prims.size());
		for (size_t k = 0; k != prims.size(); ++k)
		{
			s2r[k] = prims[k]->m_s2r;
			r2s[k] = InvertOrthonormal(prims[k]->m_s2r);
			surface_plans[k] = BuildSurfacePlan(*prims[k], cfg.m_surface_spacing);
		}

		auto const up = frame.m_up;
		auto const t0 = frame.m_t0;
		auto const t1 = frame.m_t1;
		auto const ref = frame.m_ref;
		auto const com_ws = body.m_o2w * body.m_centre_of_mass_os.w1();
		auto const rho_g = cfg.m_fluid_density * g_mag;
		auto const ca = cfg.m_added_mass_coefficient;

		// A scale-relative epsilon for inside tests so boundary samples register without leaking.
		auto bbox = CalcBBox(hull);
		auto const extent = MaxElement(bbox.m_radius.w0());
		auto const eps = std::max(1e-6f, extent * 1e-5f);

		auto wet_volume = 0.0f;
		auto wet_moment = v4::Zero();
		auto buoy_force = v4::Zero();
		auto buoy_torque = v4::Zero();
		auto drag_force = v4::Zero();
		auto drag_torque = v4::Zero();
		auto const c_linear = cfg.m_linear_drag_time_constant_s > 0.0f ? cfg.m_fluid_density / cfg.m_linear_drag_time_constant_s : 0.0f;
		auto const c_angular = cfg.m_angular_drag_time_constant_s > 0.0f ? cfg.m_fluid_density / cfg.m_angular_drag_time_constant_s : 0.0f;
		auto const w_horizontal = body.m_omega_ws - Dot3(body.m_omega_ws, up) * up;

		// Integrate pressure, water-column terms, and drag over the wetted union boundary,
		// deduplicated by the any-other-sibling exterior-side rule.
		for (size_t k = 0; k != prims.size(); ++k)
		{
			auto const& plan = surface_plans[k];
			for (uint32_t i = 0; i != plan.m_count; ++i)
			{
				auto const sample = surface::EmitSurfaceSample(plan, i);
				auto const p_root = s2r[k] * sample.m_pos_local;
				auto const n_root = Normalise((s2r[k] * sample.m_normal_local).w0(), v4::Zero());

				// A surface sample is on the union boundary iff its exterior side is outside every
				// other primitive. Cull if any other sibling contains the slightly-outward point or
				// strictly contains the sample itself. The outward point is 'eps' outside a coplanar
				// sibling face, so lower-index siblings use a wider slack: coplanar faces are then kept
				// by exactly one primitive (the lowest index) rather than by all or none.
				auto const probe_root = p_root + n_root * eps;
				auto culled = false;
				for (size_t j = 0; j != prims.size() && !culled; ++j)
				{
					if (j == k)
						continue;

					culled =
						ContainsLocal(*prims[j], r2s[j] * probe_root, j < k ? 2.0f * eps : 0.5f * eps) ||
						ContainsLocal(*prims[j], r2s[j] * p_root, -eps);
				}
				if (culled)
				{
					if (debug)
						debug->m_samples.push_back({ (body.m_o2w * p_root).w1(), Normalise((body.m_o2w * n_root).w0(), v4::Zero()), static_cast<int>(k), ESampleKind::SurfaceCulled });
					continue;
				}

				auto const sample_ws = body.m_o2w * p_root;
				auto const normal_ws = Normalise((body.m_o2w * n_root).w0(), v4::Zero());

				// Each sample stands for a surface cell, so forces act on the cell's submerged area. This keeps the forces
				// continuous as the body moves or rotates, even when the wetted region is smaller than one cell.
				auto const uv = v2{Dot3(sample_ws - ref, t0), Dot3(sample_ws - ref, t1)};
				auto const signed_height = Dot3(sample_ws - ref, up);
				auto const n_up = Dot3(normal_ws, up);
				auto const cell_height = SurfaceCellHeight(sample.m_darea, n_up);
				auto const h = water.Height(uv);
				auto const wet_fraction = CellWetFraction(signed_height, h, cell_height);
				auto const wet_weight = sample.m_darea * wet_fraction;
				if (wet_weight <= 0.0f)
				{
					if (debug)
						debug->m_samples.push_back({ sample_ws.w1(), normal_ws, static_cast<int>(k), ESampleKind::SurfaceDry });
					continue; // dry
				}

				// Hydrostatic pressure acts along the inward normal on the wet part of the cell. A partly wet cell spans depths
				// (depth +/- cell_height/2), so its wet part has mean depth (depth + cell_height/2)/2.
				auto const r = sample_ws - com_ws;
				auto const depth = h - signed_height;
				auto const wet_depth = wet_fraction < 1.0f ? 0.5f * (depth + 0.5f * cell_height) : depth;
				auto const depth_area = wet_weight * wet_depth;

				// The vertical water column between the sample and the surface has signed volume -n.up * depth_area.
				// The columns sum to the wet volume, so they carry the volume terms.
				auto const dvol = -n_up * depth_area;
				auto const r_col = r + (0.5f * wet_depth) * up;
				auto const grad = water.PressureGradient(uv, g_mag);
				auto const grad_ws = grad.x * t0 + grad.y * t1 + grad.z * up;
				auto const grad_up = Dot3(grad_ws, up);
				auto const slope = water.Slope(uv);
				auto const slope_ws = slope.x * t0 + slope.y * t1;

				// The vertical pressure force comes from the sampled normals, scaled by the wave's vertical acceleration.
				// The horizontal force comes from the columns: the Froude-Krylov force (1 + Ca) * grad on each column's volume.
				// The torque uses the sampled pressure plus the column correction for the difference between the gradient and
				// the slope (see CSBuoyancySurfaceSamples in gpu_buoyancy.hlsl).
				auto const dF_pressure = -(rho_g * (1.0f - (1.0f + ca) * grad_up) * depth_area) * normal_ws;
				auto const grad_lat = grad_ws - grad_up * up;
				auto const dF_lateral = -(rho_g * dvol * (1.0f + ca)) * grad_lat;
				auto const dF_column = dF_lateral + (rho_g * dvol) * slope_ws;
				auto const dF_buoy = Dot3(dF_pressure, up) * up + dF_lateral;
				buoy_force += dF_buoy;
				buoy_torque += Cross(r, dF_pressure) + Cross(r_col, dF_column);
				wet_volume += dvol;
				wet_moment += (com_ws + r_col).w0() * dvol;

				// Volume damping of the column. Translation is resisted relative to the water; rotation uses the column's
				// exact second moment.
				auto const water_velocity = water.Velocity(sample_ws);
				if (c_linear > 0.0f || c_angular > 0.0f)
				{
					auto const d_lin = c_linear * dvol;
					auto const d_ang = c_angular * dvol;
					auto const f_lin = -d_lin * (body.m_vel_lin_ws - water_velocity);
					auto const f_ang = -d_ang * Cross(body.m_omega_ws, r_col);
					auto const t_ang = Cross(r_col, f_ang) - (d_ang * wet_depth * wet_depth / 12.0f) * w_horizontal;
					drag_force += f_lin + f_ang;
					drag_torque += Cross(r_col, f_lin) + t_ang;
				}

				// Quadratic form drag acts only on faces moving outward into the fluid.
				auto const v_point = body.m_vel_lin_ws + Cross(body.m_omega_ws, r);
				auto const v_rel = v_point - water_velocity;
				auto const v_n = Dot3(v_rel, normal_ws);
				auto dF = v4::Zero();
				if (cfg.m_quadratic_drag_coefficient > 0.0f && v_n > 0.0f)
					dF += (-0.5f * cfg.m_fluid_density * cfg.m_quadratic_drag_coefficient * wet_weight * v_n * v_n) * normal_ws;

				// Tangential drag opposes the complete in-plane relative velocity. The |v_t| * v_t
				// form is quadratic in speed while remaining continuous through zero.
				if (cfg.m_tangential_drag_coefficient > 0.0f)
				{
					auto const v_t = v_rel - v_n * normal_ws;
					auto const speed_t = Length(v_t);
					dF += (-0.5f * cfg.m_fluid_density * cfg.m_tangential_drag_coefficient * wet_weight * speed_t) * v_t;
				}

				drag_force += dF;
				drag_torque += Cross(r, dF);

				if (debug)
				{
					debug->m_samples.push_back({ sample_ws.w1(), normal_ws, static_cast<int>(k), ESampleKind::SurfaceActive });
					debug->m_prim_buoy_force_ws[k] += dF_buoy;
					debug->m_prim_wet_moment_ws[k] += (com_ws + r_col).w0() * dvol;
					debug->m_prim_wet_volume[k] += dvol;
				}
			}
		}

		result.m_valid = wet_volume > 0.0f;
		result.m_volume_m3 = wet_volume;
		result.m_centre_buoyancy_ws = result.m_valid ? (wet_moment / wet_volume).w1() : v4::Zero();
		result.m_buoyancy_force_ws = buoy_force.w0();
		result.m_buoyancy_torque_ws = buoy_torque.w0();
		result.m_drag_force_ws = drag_force.w0();
		result.m_drag_torque_ws = drag_torque.w0();
		return result;
	}
}
