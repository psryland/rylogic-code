//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Deterministic CPU buoyancy sampler.
//
// This is the reference oracle for the GPU buoyancy sampler. It mirrors the planned GPU
// algorithm exactly (same hash, same low-discrepancy sequence, same per-sample weights, same
// cull rules) so the GPU compute kernels can be validated against it to single-precision noise.
//
// Concept (locked design):
//  - Buoyancy is a VOLUME integral. The submerged union of a body's convex primitives is sampled
//    volumetrically; each accepted sample contributes a Froude-Krylov pressure-gradient force and
//    a per-sample torque. This needs no surface reconstruction (no CSG / seam / cap clipping) and
//    handles arbitrary overlapping, possibly non-convex composites.
//  - Linear damping is a VOLUME integral over accepted wet samples. This gives equal-density bodies
//    the configured velocity time constant independently of scale.
//  - Quadratic drag is a SURFACE integral. Per-primitive surface samples emit (point, normal, dA)
//    tuples and contribute independent windward-normal form drag and tangential surface drag.
//  - Overlap is deduplicated by sibling-cull, with DIFFERENT rules for the two passes:
//      Volume : a sample is owned by the LOWEST-index primitive that contains it, so a volume
//               sample is dropped if any LOWER-index sibling contains it (unbiased union volume).
//      Surface: a surface sample lies on the union boundary iff its exterior side is outside every
//               OTHER primitive, so it is dropped if any other sibling contains the slightly
//               outward-offset point (or strictly contains the sample). This avoids counting
//               internal/embedded surfaces and correctly drops shared faces of abutting primitives.
//  - Every sample carries its own weight (dV or dA); reductions are weighted sums, never
//    count x constant, because different primitives have different sample densities.
//
// Gravity frame (no baked-in gravity direction):
//  The water surface is a height field along the local "up" axis = -normalize(gravity), NOT world
//  Z. All wet/dry tests, the FK slope force, and the drag velocity are expressed against a
//  WaterFrame {up, t0, t1, ref}; world-Z flat water is just the special case up=+Z,t0=+X,t1=+Y.
//
// The water surface is supplied as a template parameter so this header does not depend on the
// physics engine, and so gravity-frame water fields can be plugged in for tests. A water field
// type must provide:
//    float Height(v2 uv) const;                         // signed height along 'up' at planar coords (u,v), relative to 'ref'
//    v2    PressureGradient(v2 uv, float gravity) const;// dimensionless lateral pressure gradient
//    v4    Velocity(v4 pos_ws) const;                   // world-space fluid velocity (w = 0)
#pragma once
#include <vector>
#include <cstdint>
#include <cmath>
#include <algorithm>
#include "pr/collision/shape.h"
#include "pr/collision/shape_box.h"
#include "pr/collision/shape_sphere.h"
#include "pr/collision/shape_triangle.h"
#include "pr/collision/shape_polytope.h"
#include "pr/collision/shape_array.h"

namespace pr::physics::buoyancy
{
	// Tunable fluid parameters used by the sampler. Mirrors the runtime GpuBuoyancy::Config fields
	// the sampler cares about, but kept independent so this header has no engine dependency.
	struct SamplerConfig
	{
		float m_fluid_density = 1000.0f;            // kg/m^3
		float m_drag_time_constant_s = 0.0f;        // linear drag e-fold time; <= 0 disables linear drag
		float m_quadratic_drag_coefficient = 0.0f;  // form-drag Cd; <= 0 disables quadratic drag
		float m_tangential_drag_coefficient = 0.0f; // surface-shear Ct; <= 0 disables tangential drag
	};

	// Rigid-body kinematics needed to place samples in world space and evaluate point velocities.
	struct BodyState
	{
		m4x4 m_o2w = m4x4::Identity();   // centre-of-mass-root -> world (shapes are authored in COM-root space)
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
		float m_volume_m3 = 0.0f;         // submerged union volume (diagnostic)
		v4 m_centre_buoyancy_ws = v4::Zero();
		v4 m_buoyancy_force_ws = v4::Zero();
		v4 m_buoyancy_torque_ws = v4::Zero();
		v4 m_drag_force_ws = v4::Zero();
		v4 m_drag_torque_ws = v4::Zero();
	};

	// A volume sample in shape-local space with its volume weight.
	struct VolumeSample
	{
		v4 m_pos_local = v4::Origin();
		float m_dvol = 0.0f;
	};

	// A surface sample in shape-local space with its outward normal and area weight.
	struct SurfaceSample
	{
		v4 m_pos_local = v4::Origin();
		v4 m_normal_local = v4::Zero();
		float m_darea = 0.0f;
	};

	// Classification of a single sample emitted during SampleHull, recorded only when a debug
	// collector is supplied. Mirrors the exact cull/wet branches so a visualiser can render the
	// real sampler decisions rather than an independent re-derivation.
	enum class ESampleKind
	{
		VolumeWet,       // volume sample inside the wetted union, contributes buoyancy
		VolumeDry,       // volume sample above the water surface
		VolumeCulled,    // volume sample owned by a lower-index sibling primitive
		SurfaceActive,   // surface sample on the wetted union boundary, contributes drag
		SurfaceDry,      // surface sample above the water surface
		SurfaceCulled,   // surface sample interior to another primitive (not on the union boundary)
	};

	// One recorded sample for debug visualisation: world-space position, world-space outward
	// normal (zero for volume samples), owning primitive index and classification.
	struct SampleDebugRecord
	{
		v4 m_pos_ws = v4::Origin();
		v4 m_normal_ws = v4::Zero();
		int m_prim_index = 0;
		ESampleKind m_kind = ESampleKind::VolumeDry;
	};

	// Optional out-parameter collector for SampleHull. When passed, every sample's classification is
	// recorded and per-primitive accepted buoyancy partials are accumulated, so a visualiser can draw
	// the sample cloud and per-primitive force arrows. Data-only: no render/LDraw dependencies here.
	struct SampleDebug
	{
		std::vector<SampleDebugRecord> m_samples;   // every classified sample (volume + surface)
		std::vector<v4> m_prim_buoy_force_ws;       // per-primitive accepted buoyancy force
		std::vector<v4> m_prim_wet_moment_ws;       // per-primitive sum(sample_ws * dV) over wet samples
		std::vector<float> m_prim_wet_volume;       // per-primitive accepted wet volume

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
	// Deterministic low-discrepancy sample generation (mirrorable in HLSL)
	//

	// 32-bit integer hash (Wang/lowbias style). Only used to derive a bounded, reproducible Halton
	// index offset per primitive so different primitives sample different windows of the sequence.
	inline uint32_t HashU32(uint32_t x)
	{
		x ^= x >> 16;
		x *= 0x7feb352du;
		x ^= x >> 15;
		x *= 0x846ca68bu;
		x ^= x >> 16;
		return x;
	}

	// Combine two 32-bit hashes into one.
	inline uint32_t HashCombine(uint32_t a, uint32_t b)
	{
		return HashU32(a ^ (b + 0x9e3779b9u + (a << 6) + (a >> 2)));
	}

	// Van der Corput / Halton radical inverse of 'index' in 'base'. Float accumulation matches the
	// natural HLSL implementation. 'index' is expected to be small (offset is bounded below) so the
	// float precision of the inverse stays high.
	inline float RadicalInverse(uint32_t index, uint32_t base)
	{
		auto const inv_base = 1.0f / static_cast<float>(base);
		auto f = inv_base;
		auto result = 0.0f;
		for (; index != 0; index /= base, f *= inv_base)
			result += static_cast<float>(index % base) * f;

		return result;
	}

	// Per-primitive Halton index for the i'th sample (0-based). The bounded hash offset keeps the
	// index small (good float precision) while decorrelating primitives; +1 skips the 0 endpoint.
	inline uint32_t SampleIndex(uint32_t hull_id, int primitive_index, int i)
	{
		auto const offset = HashCombine(hull_id, static_cast<uint32_t>(primitive_index)) % 4096u;
		return 1u + offset + static_cast<uint32_t>(i);
	}

	//
	// Per-primitive measures (used to allocate samples and to weight each sample)
	//

	// Return the volume of a single convex primitive (shape-local). Triangles are zero-volume.
	inline float PrimitiveVolume(collision::Shape const& shape)
	{
		using namespace collision;
		switch (shape.m_type)
		{
			case EShape::Box:
			{
				auto const& box = shape_cast<ShapeBox>(shape);
				return 8.0f * box.m_radius.x * box.m_radius.y * box.m_radius.z;
			}
			case EShape::Sphere:
			{
				auto const& sph = shape_cast<ShapeSphere>(shape);
				return (4.0f / 3.0f) * static_cast<float>(math::constants<double>::tau_by_2) * sph.m_radius * sph.m_radius * sph.m_radius;
			}
			case EShape::Polytope:
			{
				auto const& poly = shape_cast<ShapePolytope>(shape);
				if (poly.m_tet_count == 0 || poly.m_volume_vert_count == 0)
					return CalcVolume(poly);

				auto vol = 0.0f;
				for (int t = 0; t != poly.m_tet_count; ++t)
				{
					auto const& tet = poly.tet(t);
					auto const a = poly.volume_vertex(tet.m_corner[0]);
					auto const b = poly.volume_vertex(tet.m_corner[1]);
					auto const c = poly.volume_vertex(tet.m_corner[2]);
					auto const d = poly.volume_vertex(tet.m_corner[3]);
					vol += std::abs(Dot3(a - d, Cross(b - d, c - d))) / 6.0f;
				}
				return vol;
			}
			case EShape::Triangle:
			{
				return 0.0f;
			}
			default:
			{
				return 0.0f;
			}
		}
	}

	// Precomputed volume-sampling acceleration table for one primitive. Built once per primitive per
	// SampleHull call and reused across that primitive's (potentially thousands of) volume samples,
	// replacing the previous design that recomputed the total volume and re-scanned the tet list for
	// every individual sample. For polytopes it holds the cumulative tet-volume CDF (m_tet_cdf[t] =
	// sum of tet volumes [0..t], so m_tet_cdf.back() == total) which a sample picks from by binary
	// search. For analytic primitives (box/sphere/triangle) only m_total is meaningful.
	struct VolumeSampleTable
	{
		std::vector<float> m_tet_cdf; // cumulative tet volumes; empty for non-polytopes
		float m_total = 0.0f;         // total primitive volume (CDF back, or analytic volume)
	};

	// Build the volume-sampling table for one primitive. For polytopes this walks the tets once,
	// hoisting the volume-vert / tet base pointers out of the loop so the ShapePolytope blob-offset
	// chain (vert_beg -> ... -> volume_vert_beg / tet_beg) is resolved a single time rather than on
	// every vertex fetch. The cumulative sums are accumulated in tet order so m_total is bit-identical
	// to PrimitiveVolume() and the GPU oracle's per-tet running sum, preserving sample parity.
	inline VolumeSampleTable BuildVolumeSampleTable(collision::Shape const& shape)
	{
		using namespace collision;
		auto table = VolumeSampleTable{};
		if (shape.m_type != EShape::Polytope)
		{
			table.m_total = PrimitiveVolume(shape);
			return table;
		}

		auto const& poly = shape_cast<ShapePolytope>(shape);
		auto const* vv = poly.volume_vert_beg();
		auto const* tets = poly.tet_beg();
		table.m_tet_cdf.resize(static_cast<size_t>(poly.m_tet_count));
		auto accum = 0.0f;
		for (int t = 0; t != poly.m_tet_count; ++t)
		{
			auto const& tet = tets[t];
			auto const a = vv[tet.m_corner[0]];
			auto const b = vv[tet.m_corner[1]];
			auto const c = vv[tet.m_corner[2]];
			auto const d = vv[tet.m_corner[3]];
			accum += std::abs(Dot3(a - d, Cross(b - d, c - d))) / 6.0f;
			table.m_tet_cdf[static_cast<size_t>(t)] = accum;
		}
		table.m_total = accum;
		return table;
	}

	// Return the surface area of a single convex primitive (shape-local).
	inline float PrimitiveArea(collision::Shape const& shape)
	{
		using namespace collision;
		switch (shape.m_type)
		{
			case EShape::Box:
			{
				auto const& box = shape_cast<ShapeBox>(shape);
				auto const ax = 2.0f * box.m_radius.x;
				auto const ay = 2.0f * box.m_radius.y;
				auto const az = 2.0f * box.m_radius.z;
				return 2.0f * (ax * ay + ay * az + az * ax);
			}
			case EShape::Sphere:
			{
				auto const& sph = shape_cast<ShapeSphere>(shape);
				return 2.0f * static_cast<float>(math::constants<double>::tau) * sph.m_radius * sph.m_radius;
			}
			case EShape::Polytope:
			{
				auto const& poly = shape_cast<ShapePolytope>(shape);
				auto area = 0.0f;
				for (int f = 0; f != poly.m_face_count; ++f)
				{
					auto const& face = poly.face(f);
					auto const a = poly.vertex(face.m_index[0]);
					auto const b = poly.vertex(face.m_index[1]);
					auto const c = poly.vertex(face.m_index[2]);
					area += 0.5f * Length(Cross(b - a, c - a));
				}
				return area;
			}
			case EShape::Triangle:
			{
				auto const& tri = shape_cast<ShapeTriangle>(shape);
				return 0.5f * Length(Cross(tri.m_v.y - tri.m_v.x, tri.m_v.z - tri.m_v.x));
			}
			default:
			{
				return 0.0f;
			}
		}
	}

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
	// Per-primitive sample emission
	//

	// Emit the i'th low-discrepancy volume sample for a primitive, weighted by 'dvol' (= measure/N).
	// 'vtable' is the primitive's precomputed VolumeSampleTable (built once by the caller); for
	// polytopes it supplies the total volume and the cumulative tet CDF so this routine does O(log
	// tets) work per sample instead of the previous O(tets) total-recompute + linear scan.
	inline VolumeSample EmitVolumeSample(collision::Shape const& shape, uint32_t index, float dvol, VolumeSampleTable const& vtable)
	{
		using namespace collision;
		switch (shape.m_type)
		{
			case EShape::Box:
			{
				auto const& box = shape_cast<ShapeBox>(shape);
				auto const u = RadicalInverse(index, 2);
				auto const v = RadicalInverse(index, 3);
				auto const w = RadicalInverse(index, 5);
				auto const p = v4{(2.0f * u - 1.0f) * box.m_radius.x, (2.0f * v - 1.0f) * box.m_radius.y, (2.0f * w - 1.0f) * box.m_radius.z, 1.0f};
				return VolumeSample{p, dvol};
			}
			case EShape::Sphere:
			{
				auto const& sph = shape_cast<ShapeSphere>(shape);
				auto const a = RadicalInverse(index, 2); // radius via cube-root for uniform density in the ball
				auto const b = RadicalInverse(index, 3); // cos(theta) mapped to [-1,1]
				auto const c = RadicalInverse(index, 5); // azimuth
				auto const r = sph.m_radius * std::cbrt(a);
				auto const z = 1.0f - 2.0f * b;
				auto const rho = std::sqrt(std::max(0.0f, 1.0f - z * z));
				auto const phi = static_cast<float>(math::constants<double>::tau) * c;
				auto const dir = v4{rho * std::cos(phi), rho * std::sin(phi), z, 0.0f};
				return VolumeSample{(dir * r).w1(), dvol};
			}
			case EShape::Polytope:
			{
				auto const& poly = shape_cast<ShapePolytope>(shape);
				auto const pick = RadicalInverse(index, 2);
				auto const s_in = RadicalInverse(index, 3);
				auto const t_in = RadicalInverse(index, 5);
				auto const u_in = RadicalInverse(index, 7);

				// Hoist the blob-offset base pointers so the ShapePolytope accessor chain is resolved
				// once for this sample rather than on each of the eight volume_vertex fetches below.
				auto const* vv = poly.volume_vert_beg();
				auto const* tets = poly.tet_beg();

				// Pick a tetrahedron with probability proportional to its volume. With p_tet =
				// V_tet/V_total, a constant per-sample weight dvol = V_total/N is the unbiased estimator.
				// The cumulative CDF is monotonic, so a binary search finds the first tet whose running
				// sum reaches the target - identical selection to the old linear scan (the CDF entries
				// are the same running sums, in the same order) but O(log tets) instead of O(tets).
				auto const& cdf = vtable.m_tet_cdf;
				auto const target = pick * vtable.m_total;
				auto const it = std::lower_bound(cdf.begin(), cdf.end(), target);
				auto const chosen = it != cdf.end() ? static_cast<int>(it - cdf.begin()) : poly.m_tet_count - 1;

				// Uniform barycentric point in the chosen tet (Rocchini & Cignoni fold).
				auto s = s_in, t = t_in, u = u_in;
				if (s + t > 1.0f) { s = 1.0f - s; t = 1.0f - t; }
				if (t + u > 1.0f) { auto const tmp = u; u = 1.0f - s - t; t = 1.0f - tmp; }
				else if (s + t + u > 1.0f) { auto const tmp = u; u = s + t + u - 1.0f; s = 1.0f - t - tmp; }
				auto const aw = 1.0f - s - t - u;

				auto const& tet = tets[chosen];
				auto const A = vv[tet.m_corner[0]];
				auto const B = vv[tet.m_corner[1]];
				auto const C = vv[tet.m_corner[2]];
				auto const D = vv[tet.m_corner[3]];
				auto const p = (A * aw + B * s + C * t + D * u);
				return VolumeSample{p.w1(), dvol};
			}
			default:
			{
				return VolumeSample{v4::Origin(), 0.0f};
			}
		}
	}

	// Emit the i'th low-discrepancy surface sample for a primitive, weighted by 'darea' (= area/N).
	inline SurfaceSample EmitSurfaceSample(collision::Shape const& shape, uint32_t index, float darea)
	{
		using namespace collision;
		switch (shape.m_type)
		{
			case EShape::Box:
			{
				auto const& box = shape_cast<ShapeBox>(shape);
				auto const hx = box.m_radius.x, hy = box.m_radius.y, hz = box.m_radius.z;

				// Area CDF over the 6 faces (constant darea avoids tiny faces being starved of samples).
				float const face_area[6] =
				{
					4.0f * hy * hz, 4.0f * hy * hz, // +X, -X
					4.0f * hx * hz, 4.0f * hx * hz, // +Y, -Y
					4.0f * hx * hy, 4.0f * hx * hy, // +Z, -Z
				};
				auto sum = 0.0f;
				for (auto fa : face_area) sum += fa;

				auto const pick = RadicalInverse(index, 2) * sum;
				auto const ca = RadicalInverse(index, 3);
				auto const cb = RadicalInverse(index, 5);
				auto accum = 0.0f;
				auto face = 0;
				for (int f = 0; f != 6; ++f)
				{
					accum += face_area[f];
					face = f;
					if (accum >= pick)
						break;
				}

				auto const sa = 2.0f * ca - 1.0f;
				auto const sb = 2.0f * cb - 1.0f;
				switch (face)
				{
					case 0: return SurfaceSample{v4{+hx, sa * hy, sb * hz, 1.0f}, v4{+1, 0, 0, 0}, darea};
					case 1: return SurfaceSample{v4{-hx, sa * hy, sb * hz, 1.0f}, v4{-1, 0, 0, 0}, darea};
					case 2: return SurfaceSample{v4{sa * hx, +hy, sb * hz, 1.0f}, v4{0, +1, 0, 0}, darea};
					case 3: return SurfaceSample{v4{sa * hx, -hy, sb * hz, 1.0f}, v4{0, -1, 0, 0}, darea};
					case 4: return SurfaceSample{v4{sa * hx, sb * hy, +hz, 1.0f}, v4{0, 0, +1, 0}, darea};
					default: return SurfaceSample{v4{sa * hx, sb * hy, -hz, 1.0f}, v4{0, 0, -1, 0}, darea};
				}
			}
			case EShape::Sphere:
			{
				auto const& sph = shape_cast<ShapeSphere>(shape);
				auto const a = RadicalInverse(index, 2);
				auto const b = RadicalInverse(index, 3);
				auto const z = 1.0f - 2.0f * a;
				auto const rho = std::sqrt(std::max(0.0f, 1.0f - z * z));
				auto const phi = static_cast<float>(math::constants<double>::tau) * b;
				auto const dir = v4{rho * std::cos(phi), rho * std::sin(phi), z, 0.0f};
				return SurfaceSample{(dir * sph.m_radius).w1(), dir, darea};
			}
			case EShape::Polytope:
			{
				auto const& poly = shape_cast<ShapePolytope>(shape);
				auto const total = PrimitiveArea(shape);
				auto const pick = RadicalInverse(index, 2) * total;
				auto const u1 = RadicalInverse(index, 3);
				auto const u2 = RadicalInverse(index, 5);

				auto accum = 0.0f;
				auto chosen = 0;
				for (int f = 0; f != poly.m_face_count; ++f)
				{
					auto const& face = poly.face(f);
					auto const a = poly.vertex(face.m_index[0]);
					auto const b = poly.vertex(face.m_index[1]);
					auto const c = poly.vertex(face.m_index[2]);
					accum += 0.5f * Length(Cross(b - a, c - a));
					chosen = f;
					if (accum >= pick)
						break;
				}

				auto const& face = poly.face(chosen);
				auto const A = poly.vertex(face.m_index[0]);
				auto const B = poly.vertex(face.m_index[1]);
				auto const C = poly.vertex(face.m_index[2]);
				auto const su = std::sqrt(u1);
				auto const w0 = 1.0f - su;
				auto const w1 = su * (1.0f - u2);
				auto const w2 = su * u2;
				auto const p = (A * w0 + B * w1 + C * w2);
				auto const n = Normalise(face.m_plane.direction(), v4::Zero());
				return SurfaceSample{p.w1(), n, darea};
			}
			case EShape::Triangle:
			{
				auto const& tri = shape_cast<ShapeTriangle>(shape);
				auto const u1 = RadicalInverse(index, 2);
				auto const u2 = RadicalInverse(index, 3);
				auto const su = std::sqrt(u1);
				auto const w0 = 1.0f - su;
				auto const w1 = su * (1.0f - u2);
				auto const w2 = su * u2;
				auto const p = (tri.m_v.x * w0 + tri.m_v.y * w1 + tri.m_v.z * w2);
				return SurfaceSample{p.w1(), tri.m_v.w.w0(), darea};
			}
			default:
			{
				return SurfaceSample{v4::Origin(), v4::Zero(), 0.0f};
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

	// Distribute 'total' samples across primitives proportional to 'measure' (volume or area).
	// Primitives with zero measure get zero samples; positive-measure primitives get at least one.
	inline std::vector<int> DistributeCounts(std::vector<float> const& measure, int total)
	{
		auto counts = std::vector<int>(measure.size(), 0);
		auto sum = 0.0f;
		for (auto m : measure) sum += m;
		if (sum <= 0.0f || total <= 0)
			return counts;

		for (size_t i = 0; i != measure.size(); ++i)
		{
			if (measure[i] <= 0.0f)
				continue;

			auto const n = static_cast<int>(std::lround(total * (measure[i] / sum)));
			counts[i] = std::max(1, n);
		}
		return counts;
	}

	//
	// Top-level sampler
	//

	// Sample buoyancy + drag for a hull. 'water' is any type satisfying the water-field concept
	// documented at the top of this file. Returns aggregated world-space force/torque + diagnostics.
	// When 'debug' is non-null, every sample's classification and the per-primitive accepted buoyancy
	// partials are recorded for visualisation; the physical result is unchanged whether debug is set or
	// not. Note that when debug is non-null the surface pass runs even with drag disabled (so surface
	// classifications are still recorded), but it contributes zero drag because the coefficients are zero.
	template <typename TWater>
	inline HullResult SampleHull(
		collision::Shape const& hull,
		uint32_t hull_id,
		BodyState const& body,
		WaterFrame const& frame,
		TWater const& water,
		SamplerConfig const& cfg,
		int volume_samples_total,
		int surface_samples_total,
		SampleDebug* debug = nullptr)
	{
		using namespace collision;

		auto result = HullResult{};

		auto const prims = CollectPrimitives(hull);
		if (prims.empty())
		{
			if (debug)
				debug->Reset(0);
			return result;
		}

		if (debug)
			debug->Reset(prims.size());

		// Pre-compute per-primitive transforms (shape-local <-> COM-root) and measures.
		auto s2r = std::vector<m4x4>(prims.size());
		auto r2s = std::vector<m4x4>(prims.size());
		auto volume = std::vector<float>(prims.size(), 0.0f);
		auto area = std::vector<float>(prims.size(), 0.0f);
		auto vol_tables = std::vector<VolumeSampleTable>(prims.size());
		for (size_t k = 0; k != prims.size(); ++k)
		{
			s2r[k] = prims[k]->m_s2r;
			r2s[k] = InvertOrthonormal(prims[k]->m_s2r);
			vol_tables[k] = BuildVolumeSampleTable(*prims[k]);
			volume[k] = vol_tables[k].m_total;
			area[k] = PrimitiveArea(*prims[k]);
		}

		auto const vol_counts = DistributeCounts(volume, volume_samples_total);
		auto const surf_counts = DistributeCounts(area, surface_samples_total);

		auto const up = frame.m_up;
		auto const t0 = frame.m_t0;
		auto const t1 = frame.m_t1;
		auto const ref = frame.m_ref;
		auto const com_ws = body.m_o2w.pos;
		auto const g_mag = Length(body.m_gravity_ws.w0());

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
		auto const c_lin = cfg.m_drag_time_constant_s > 0.0f ? cfg.m_fluid_density / cfg.m_drag_time_constant_s : 0.0f;

		// Volume pass: Froude-Krylov pressure-gradient force and linear damping over the submerged
		// union, deduplicated by the lowest-index-sibling rule.
		if (g_mag > math::constants<float>::tiny)
		{
			for (size_t k = 0; k != prims.size(); ++k)
			{
				if (vol_counts[k] == 0)
					continue;

				auto const dvol = volume[k] / static_cast<float>(vol_counts[k]);
				for (int i = 0; i != vol_counts[k]; ++i)
				{
					auto const sample = EmitVolumeSample(*prims[k], SampleIndex(hull_id, static_cast<int>(k), i), dvol, vol_tables[k]);
					auto const p_root = s2r[k] * sample.m_pos_local;

					// Lowest-index-sibling cull: skip if any lower-index primitive owns this volume.
					auto culled = false;
					for (size_t j = 0; j != k && !culled; ++j)
						culled = ContainsLocal(*prims[j], r2s[j] * p_root, eps);
					if (culled)
					{
						if (debug)
							debug->m_samples.push_back({ (body.m_o2w * p_root).w1(), v4::Zero(), static_cast<int>(k), ESampleKind::VolumeCulled });
						continue;
					}

					auto const sample_ws = body.m_o2w * p_root;
					auto const signed_height = Dot3(sample_ws - ref, up);
					auto const u = Dot3(sample_ws - ref, t0);
					auto const v = Dot3(sample_ws - ref, t1);
					auto const h = water.Height(v2{u, v});
					if (signed_height >= h)
					{
						if (debug)
							debug->m_samples.push_back({ sample_ws.w1(), v4::Zero(), static_cast<int>(k), ESampleKind::VolumeDry });
						continue; // dry
					}

					auto const grad = water.PressureGradient(v2{u, v}, g_mag);
					auto const grad_ws = grad.x * t0 + grad.y * t1;
					auto const dF = (cfg.m_fluid_density * g_mag * sample.m_dvol) * (up - grad_ws);

					buoy_force += dF;
					buoy_torque += Cross(sample_ws - com_ws, dF);
					wet_volume += sample.m_dvol;
					wet_moment += sample_ws * sample.m_dvol;

					// Weighting damping by wet volume gives force units and makes acceleration depend on
					// body density rather than its linear dimensions.
					if (c_lin > 0.0f)
					{
						auto const r = sample_ws - com_ws;
						auto const v_point = body.m_vel_lin_ws + Cross(body.m_omega_ws, r);
						auto const v_rel = v_point - water.Velocity(sample_ws);
						auto const drag_dF = (-c_lin * sample.m_dvol) * v_rel;
						drag_force += drag_dF;
						drag_torque += Cross(r, drag_dF);
					}

					if (debug)
					{
						debug->m_samples.push_back({ sample_ws.w1(), v4::Zero(), static_cast<int>(k), ESampleKind::VolumeWet });
						debug->m_prim_buoy_force_ws[k] += dF;
						debug->m_prim_wet_moment_ws[k] += sample_ws * sample.m_dvol;
						debug->m_prim_wet_volume[k] += sample.m_dvol;
					}
				}
			}
		}

		// Surface pass: normal form drag and tangential surface drag over the wetted union boundary,
		// deduplicated by the any-other-sibling exterior-side rule.
		auto const have_drag =
			cfg.m_quadratic_drag_coefficient > 0.0f ||
			cfg.m_tangential_drag_coefficient > 0.0f;
		if (have_drag || debug != nullptr)
		{
			for (size_t k = 0; k != prims.size(); ++k)
			{
				if (surf_counts[k] == 0)
					continue;

				auto const darea = area[k] / static_cast<float>(surf_counts[k]);
				for (int i = 0; i != surf_counts[k]; ++i)
				{
					auto const sample = EmitSurfaceSample(*prims[k], SampleIndex(hull_id, static_cast<int>(k), i), darea);
					auto const p_root = s2r[k] * sample.m_pos_local;
					auto const n_root = Normalise((s2r[k] * sample.m_normal_local).w0(), v4::Zero());

					// A surface sample is on the union boundary iff its exterior side is outside every
					// other primitive. Cull if any other sibling contains the slightly-outward point or
					// strictly contains the sample itself.
					auto const probe_root = p_root + n_root * eps;
					auto culled = false;
					for (size_t j = 0; j != prims.size() && !culled; ++j)
					{
						if (j == k)
							continue;
						culled =
							ContainsLocal(*prims[j], r2s[j] * probe_root, eps) ||
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

					auto const u = Dot3(sample_ws - ref, t0);
					auto const v = Dot3(sample_ws - ref, t1);
					if (Dot3(sample_ws - ref, up) >= water.Height(v2{u, v}))
					{
						if (debug)
							debug->m_samples.push_back({ sample_ws.w1(), normal_ws, static_cast<int>(k), ESampleKind::SurfaceDry });
						continue; // dry
					}

					auto const v_point = body.m_vel_lin_ws + Cross(body.m_omega_ws, sample_ws - com_ws);
					auto const v_rel = v_point - water.Velocity(sample_ws);
					auto const v_n = Dot3(v_rel, normal_ws);

					auto dF = v4::Zero();
					if (cfg.m_quadratic_drag_coefficient > 0.0f && v_n > 0.0f)
						dF += (-0.5f * cfg.m_fluid_density * cfg.m_quadratic_drag_coefficient * sample.m_darea * v_n * v_n) * normal_ws;

					// Tangential drag opposes the complete in-plane relative velocity. The |v_t| * v_t
					// form is quadratic in speed while remaining continuous through zero.
					if (cfg.m_tangential_drag_coefficient > 0.0f)
					{
						auto const v_t = v_rel - v_n * normal_ws;
						auto const speed_t = Length(v_t);
						dF += (-0.5f * cfg.m_fluid_density * cfg.m_tangential_drag_coefficient * sample.m_darea * speed_t) * v_t;
					}

					drag_force += dF;
					drag_torque += Cross(sample_ws - com_ws, dF);

					if (debug)
						debug->m_samples.push_back({ sample_ws.w1(), normal_ws, static_cast<int>(k), ESampleKind::SurfaceActive });
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
