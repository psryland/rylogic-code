//************************************
// Physics Engine
//  Copyright (c) Rylogic Ltd 2026
//************************************
// GPU sampled-composite buoyancy: per-primitive volume/surface sample emission.
//
// This module is the HLSL mirror of the deterministic CPU oracle in
// include/pr/physics/buoyancy/buoyancy_sampler.h. Every function here must reproduce the oracle's
// arithmetic to single-precision noise so the GPU kernels can be validated against the CPU sampler
// (Tier-2 parity tests). Keep the two files in lock-step: the hash sequence, the radical-inverse
// bases, the per-primitive sample maps and the tet/face CDF scans are all parity-critical.
//
// Binding-agnostic: the geometry-reading functions take the composite-hull StructuredBuffers as
// parameters (resolved at compile time by the including kernel TU). The kernel declares the buffers
// at whatever registers it needs and passes them through; this module pins no registers. Tet and
// face vertex indices are stored RELATIVE to each primitive's own block, so callers add the
// primitive's m_volume_vert_ofs (tets) or m_vert_ofs (faces) when indexing the shared arrays.
#ifndef PR_PHYSICS_BUOYANCY_SAMPLER_HLSLI
#define PR_PHYSICS_BUOYANCY_SAMPLER_HLSLI

#include "pr/hlsl/core.hlsli"
#include "pr/hlsl/vector.hlsli"
#include "pr/hlsl/interop.hlsli"

// Dual HLSL/C++ compilation. When compiled as C++ (interop.h supplies float4/StructuredBuffer/etc.)
// the whole module lives in pr::physics so the oracle and parity tests can call it directly; the
// 'using namespace hlsl' in pr/physics/forward.h makes the hlsl type/intrinsic names visible here,
// matching the established collision.hlsli pattern.
#ifdef __cplusplus
namespace pr::physics {
#endif

// Primitive type values mirror pr::physics::buoyancy::EPrimitiveType. Fixed so the host can upload
// them verbatim and this module can switch on them.
static const int BUOY_PRIM_BOX = 0;
static const int BUOY_PRIM_SPHERE = 1;
static const int BUOY_PRIM_POLYTOPE = 2;
static const int BUOY_PRIM_TRIANGLE = 3;

// Matches pr::math::constants<float>::tiny (1.00000007e-05f). Used as the zero-length guard so this
// module reproduces the oracle's Normalise(v, fallback) threshold exactly.
static const float BUOY_TINY = 1.00000007e-05f;

// HLSL mirror of pr::physics::buoyancy::GpuPrimitive (128-byte GPU ABI). The kernel reads an array
// of these; the layout (offsets/counts) is documented in buoyancy_primitives.h.
struct BuoyPrimitive
{
	// Shape-local -> centre-of-mass-root transform. Each HLSL row = one C++ column vector (matches
	// the GpuRigidBody.o2w upload convention), so mul(float4(p,1), m_s2r) transforms a shape-local
	// point into COM-root space, mirroring the C++ 'm_s2r * p'.
	row_major float4x4 m_s2r;

	int m_type;             // EPrimitiveType
	int m_sibling_index;    // child order (sibling-cull priority)
	int m_vert_ofs;         // surface verts block start (in the shared verts buffer)
	int m_vert_count;
	int m_volume_vert_ofs;  // interior tet verts block start (in the shared volume-verts buffer)
	int m_volume_vert_count;
	int m_tet_ofs;          // tets block start (in the shared tets buffer)
	int m_tet_count;
	int m_face_ofs;         // face planes / face verts block start (in the shared face buffers)
	int m_face_count;
	int m_pad0;
	int m_pad1;

	float4 m_params;        // box: half-extents in xyz; sphere: radius in x; else unused
};

//
// Deterministic hash + low-discrepancy sequence (mirror of buoyancy_sampler.h)
//

// 32-bit integer hash (Wang/lowbias variant) - identical mixing constants to the oracle's HashU32.
odr uint BuoyHashU32(uint x)
{
	x ^= x >> 16;
	x *= 0x7feb352du;
	x ^= x >> 15;
	x *= 0x846ca68bu;
	x ^= x >> 16;
	return x;
}

// Combine two 32-bit hashes into one (mirror of HashCombine).
odr uint BuoyHashCombine(uint a, uint b)
{
	return BuoyHashU32(a ^ (b + 0x9e3779b9u + (a << 6) + (a >> 2)));
}

// Van der Corput / Halton radical inverse of 'index' in 'base'. Float accumulation order matches the
// oracle so the two sequences agree bit-for-bit for the small indices used here.
odr float BuoyRadicalInverse(uint index, uint base)
{
	float inv_base = 1.0f / (float)base;
	float f = inv_base;
	float result = 0.0f;
	for (; index != 0; index /= base, f *= inv_base)
		result += (float)(index % base) * f;

	return result;
}

// Per-primitive Halton index for the i'th sample (0-based). The bounded hash offset keeps the index
// small (good float precision) while decorrelating primitives; +1 skips the 0 endpoint.
odr uint BuoySampleIndex(uint hull_id, int primitive_index, int i)
{
	uint offset = BuoyHashCombine(hull_id, (uint)primitive_index) % 4096u;
	return 1u + offset + (uint)i;
}

// Normalise a 3-vector, returning zero when its length is at or below BUOY_TINY. Mirrors the
// oracle's Normalise(v, v4::Zero()) so transformed/face normals degrade identically.
odr float3 BuoyNormaliseOrZero(float3 v)
{
	float len = length(v);
	return len > BUOY_TINY ? v / len : float3(0, 0, 0);
}

// Conservative support interval of a primitive along the world-space 'up' axis, expressed as the
// min/max of dot(point, up) over the primitive in world space. Only box and sphere have a cheap
// closed-form support; polytope/triangle return false so callers fall back to per-sample sampling.
// 's2w' is the shape-local -> world transform and is assumed rigid (rotation + translation, no
// scale), matching the rest of this module's InvertOrthonormal usage. Used by the flat-water
// fully-dry fast path to skip primitives that lie entirely above the water surface.
odr bool BuoySupportAlongUp(in_(BuoyPrimitive) prim, float4x4 s2w, float3 up, out float lo, out float hi)
{
	// World position of the shape origin (box/sphere centre); project onto 'up'.
	float3 centre_ws = mul(float4(0.0f, 0.0f, 0.0f, 1.0f), s2w).xyz;
	float c = dot(centre_ws, up);
	switch (prim.m_type)
	{
		case BUOY_PRIM_BOX:
		{
			// Half-extents along each shape axis; the support radius along 'up' is the sum of each
			// world axis projected onto 'up' scaled by its half-extent.
			float3 ax = mul(float4(1.0f, 0.0f, 0.0f, 0.0f), s2w).xyz;
			float3 ay = mul(float4(0.0f, 1.0f, 0.0f, 0.0f), s2w).xyz;
			float3 az = mul(float4(0.0f, 0.0f, 1.0f, 0.0f), s2w).xyz;
			float r = abs(dot(ax, up)) * prim.m_params.x + abs(dot(ay, up)) * prim.m_params.y + abs(dot(az, up)) * prim.m_params.z;
			lo = c - r;
			hi = c + r;
			return true;
		}
		case BUOY_PRIM_SPHERE:
		{
			// A rigid transform preserves the radius, so the support is centre +/- R along any axis.
			float radius = prim.m_params.x;
			lo = c - radius;
			hi = c + radius;
			return true;
		}
		default:
		{
			lo = 0.0f;
			hi = 0.0f;
			return false;
		}
	}
}

//
// Per-primitive measures (allocate samples + weight each sample). Shape-local space.
//

// Volume of a single convex primitive. Triangles are zero-volume.
odr float BuoyPrimitiveVolume(in_(BuoyPrimitive) prim, in_(StructuredBuffer<float4>) volume_verts, in_(StructuredBuffer<int4>) tets)
{
	switch (prim.m_type)
	{
		case BUOY_PRIM_BOX:
		{
			return 8.0f * prim.m_params.x * prim.m_params.y * prim.m_params.z;
		}
		case BUOY_PRIM_SPHERE:
		{
			// (4/3)*pi*r^3 - 'tau*0.5f' is pi to float precision, matching the oracle's tau_by_2.
			float r = prim.m_params.x;
			return (4.0f / 3.0f) * (tau * 0.5f) * r * r * r;
		}
		case BUOY_PRIM_POLYTOPE:
		{
			float vol = 0.0f;
			for (int t = 0; t != prim.m_tet_count; ++t)
			{
				int4 tet = tets[prim.m_tet_ofs + t];
				float3 a = volume_verts[prim.m_volume_vert_ofs + tet.x].xyz;
				float3 b = volume_verts[prim.m_volume_vert_ofs + tet.y].xyz;
				float3 c = volume_verts[prim.m_volume_vert_ofs + tet.z].xyz;
				float3 d = volume_verts[prim.m_volume_vert_ofs + tet.w].xyz;
				vol += abs(dot(a - d, cross(b - d, c - d))) / 6.0f;
			}
			return vol;
		}
		default:
		{
			return 0.0f;
		}
	}
}

// Surface area of a single convex primitive. Shape-local.
odr float BuoyPrimitiveArea(in_(BuoyPrimitive) prim, in_(StructuredBuffer<float4>) verts, in_(StructuredBuffer<int4>) face_verts)
{
	switch (prim.m_type)
	{
		case BUOY_PRIM_BOX:
		{
			float ax = 2.0f * prim.m_params.x;
			float ay = 2.0f * prim.m_params.y;
			float az = 2.0f * prim.m_params.z;
			return 2.0f * (ax * ay + ay * az + az * ax);
		}
		case BUOY_PRIM_SPHERE:
		{
			float r = prim.m_params.x;
			return 2.0f * tau * r * r;
		}
		case BUOY_PRIM_POLYTOPE:
		{
			float area = 0.0f;
			for (int f = 0; f != prim.m_face_count; ++f)
			{
				int4 fv = face_verts[prim.m_face_ofs + f];
				float3 a = verts[prim.m_vert_ofs + fv.x].xyz;
				float3 b = verts[prim.m_vert_ofs + fv.y].xyz;
				float3 c = verts[prim.m_vert_ofs + fv.z].xyz;
				area += 0.5f * length(cross(b - a, c - a));
			}
			return area;
		}
		case BUOY_PRIM_TRIANGLE:
		{
			float3 a = verts[prim.m_vert_ofs + 0].xyz;
			float3 b = verts[prim.m_vert_ofs + 1].xyz;
			float3 c = verts[prim.m_vert_ofs + 2].xyz;
			return 0.5f * length(cross(b - a, c - a));
		}
		default:
		{
			return 0.0f;
		}
	}
}

//
// Inside test (shape-local space). 'eps' is a small slack so boundary samples register.
//

// True if 'p_local' is inside (or within 'eps' of) the convex primitive. Triangles never contain.
odr bool BuoyContainsLocal(in_(BuoyPrimitive) prim, float3 p_local, float eps, in_(StructuredBuffer<float4>) face_planes)
{
	switch (prim.m_type)
	{
		case BUOY_PRIM_BOX:
		{
			return
				abs(p_local.x) <= prim.m_params.x + eps &&
				abs(p_local.y) <= prim.m_params.y + eps &&
				abs(p_local.z) <= prim.m_params.z + eps;
		}
		case BUOY_PRIM_SPHERE:
		{
			float r = prim.m_params.x + eps;
			return dot(p_local, p_local) <= r * r;
		}
		case BUOY_PRIM_POLYTOPE:
		{
			// Convex hull with outward face planes: inside iff on the negative side of every plane.
			// Plane is stored (dir.xyz, dist.w); signed distance = dot(dir, p) + dist.
			for (int f = 0; f != prim.m_face_count; ++f)
			{
				float4 pl = face_planes[prim.m_face_ofs + f];
				if (dot(pl.xyz, p_local) + pl.w > eps)
					return false;
			}
			return prim.m_face_count != 0;
		}
		default:
		{
			// Triangle (zero-volume) never contains a point for sibling-cull purposes.
			return false;
		}
	}
}

//
// Sibling-cull helpers (phase 9). The volume and surface passes deduplicate overlapping primitives
// with DIFFERENT rules, mirroring SampleHull in buoyancy_sampler.h:
//  * Volume  -> lowest-index-sibling owns the overlap: cull if ANY lower-index sibling (j < k)
//               contains the sample. Gives an unbiased union volume.
//  * Surface -> a sample is on the union boundary iff its slightly-outward probe is outside EVERY
//               other sibling (j != k): cull if any other sibling contains the +eps probe OR
//               strictly (-eps) contains the sample itself.
// 'prim_base' is the absolute start of the hull's primitive block in the shared primitive buffer,
// 'prim_count' the number of siblings, and 'k' the local sibling index (0..prim_count-1). Points are
// in COM-root space; each sibling's r2s = InvertOrthonormal(m_s2r) maps the root point into that
// sibling's shape-local space for the contains test.
//

// Transform a COM-root point into a primitive's shape-local space (orthonormal inverse of m_s2r).
odr float3 BuoyRootToLocal(in_(BuoyPrimitive) prim, float3 p_root)
{
	float4x4 r2s = InvertOrthonormal(prim.m_s2r);
	return mul(float4(p_root, 1.0f), r2s).xyz;
}

// Volume cull: true if any lower-index sibling (j < k) contains 'p_root'. Mirrors the SampleHull
// lowest-index-sibling rule (j != 0..k-1, ContainsLocal(prim_j, r2s_j * p_root, +eps)).
odr bool BuoyIsInsideAnyLowerSibling(in_(StructuredBuffer<BuoyPrimitive>) prims, int prim_base, int k, float3 p_root, float eps, in_(StructuredBuffer<float4>) face_planes)
{
	for (int j = 0; j != k; ++j)
	{
		BuoyPrimitive sib = prims[prim_base + j];
		if (BuoyContainsLocal(sib, BuoyRootToLocal(sib, p_root), eps, face_planes))
			return true;
	}
	return false;
}

// Surface cull: true if any other sibling (j != k) hides this surface sample from the union
// boundary. Mirrors the SampleHull any-other-sibling rule: cull if a sibling contains the slightly
// outward probe (p_root + n_root*eps) OR strictly contains the sample itself (-eps). 'n_root' is the
// outward sample normal in COM-root space.
odr bool BuoyIsInsideAnyOtherSibling(in_(StructuredBuffer<BuoyPrimitive>) prims, int prim_base, int prim_count, int k, float3 p_root, float3 n_root, float eps, in_(StructuredBuffer<float4>) face_planes)
{
	float3 probe_root = p_root + n_root * eps;
	for (int j = 0; j != prim_count; ++j)
	{
		if (j == k)
			continue;

		BuoyPrimitive sib = prims[prim_base + j];
		if (BuoyContainsLocal(sib, BuoyRootToLocal(sib, probe_root), eps, face_planes) ||
			BuoyContainsLocal(sib, BuoyRootToLocal(sib, p_root), -eps, face_planes))
			return true;
	}
	return false;
}

//
// (surface) so reductions accumulate weighted sums (different primitives have different densities).
//

// Emit the i'th low-discrepancy volume sample for a primitive, weighted by 'dvol' (= measure/N).
// 'pos_local' is returned with w=1. Mirrors EmitVolumeSample.
odr void BuoyEmitVolumeSample(in_(BuoyPrimitive) prim, uint index, float dvol,
	in_(StructuredBuffer<float4>) volume_verts, in_(StructuredBuffer<int4>) tets,
	out_(float4) pos_local, out_(float) weight)
{
	weight = dvol;
	switch (prim.m_type)
	{
		case BUOY_PRIM_BOX:
		{
			float u = BuoyRadicalInverse(index, 2);
			float v = BuoyRadicalInverse(index, 3);
			float w = BuoyRadicalInverse(index, 5);
			pos_local = float4((2.0f * u - 1.0f) * prim.m_params.x, (2.0f * v - 1.0f) * prim.m_params.y, (2.0f * w - 1.0f) * prim.m_params.z, 1.0f);
			return;
		}
		case BUOY_PRIM_SPHERE:
		{
			float a = BuoyRadicalInverse(index, 2); // radius via cube-root for uniform density in the ball
			float b = BuoyRadicalInverse(index, 3); // cos(theta) mapped to [-1,1]
			float c = BuoyRadicalInverse(index, 5); // azimuth

			// PARITY NOTE: the oracle uses std::cbrt(a); HLSL has no cbrt so this uses pow(a,1/3).
			// These are not bit-identical and can flip a sphere sample's wet/dry state right at the
			// waterline. This is the known sphere parity risk; it does NOT affect box parity (the
			// phase-11 gate). Reconcile (match the oracle to this approximation, or use looser sphere
			// tolerances) when sphere parity tests are written.
			float r = prim.m_params.x * pow(a, 1.0f / 3.0f);
			float z = 1.0f - 2.0f * b;
			float rho = sqrt(max(0.0f, 1.0f - z * z));
			float phi = tau * c;
			float3 dir = float3(rho * cos(phi), rho * sin(phi), z);
			pos_local = float4(dir * r, 1.0f);
			return;
		}
		case BUOY_PRIM_POLYTOPE:
		{
			float pick = BuoyRadicalInverse(index, 2);
			float s_in = BuoyRadicalInverse(index, 3);
			float t_in = BuoyRadicalInverse(index, 5);
			float u_in = BuoyRadicalInverse(index, 7);

			// Pick a tetrahedron with probability proportional to its volume (inline volume-CDF scan).
			float total = BuoyPrimitiveVolume(prim, volume_verts, tets);
			float target = pick * total;
			int chosen = 0;
			float accum = 0.0f;
			for (int tt = 0; tt != prim.m_tet_count; ++tt)
			{
				int4 tet = tets[prim.m_tet_ofs + tt];
				float3 a = volume_verts[prim.m_volume_vert_ofs + tet.x].xyz;
				float3 b = volume_verts[prim.m_volume_vert_ofs + tet.y].xyz;
				float3 c = volume_verts[prim.m_volume_vert_ofs + tet.z].xyz;
				float3 d = volume_verts[prim.m_volume_vert_ofs + tet.w].xyz;
				accum += abs(dot(a - d, cross(b - d, c - d))) / 6.0f;
				chosen = tt;
				if (accum >= target)
					break;
			}

			// Uniform barycentric point in the chosen tet (Rocchini & Cignoni fold).
			float s = s_in, t = t_in, u = u_in;
			if (s + t > 1.0f) { s = 1.0f - s; t = 1.0f - t; }
			if (t + u > 1.0f) { float tmp = u; u = 1.0f - s - t; t = 1.0f - tmp; }
			else if (s + t + u > 1.0f) { float tmp = u; u = s + t + u - 1.0f; s = 1.0f - t - tmp; }
			float aw = 1.0f - s - t - u;

			int4 ct = tets[prim.m_tet_ofs + chosen];
			float3 A = volume_verts[prim.m_volume_vert_ofs + ct.x].xyz;
			float3 B = volume_verts[prim.m_volume_vert_ofs + ct.y].xyz;
			float3 C = volume_verts[prim.m_volume_vert_ofs + ct.z].xyz;
			float3 D = volume_verts[prim.m_volume_vert_ofs + ct.w].xyz;
			float3 p = A * aw + B * s + C * t + D * u;
			pos_local = float4(p, 1.0f);
			return;
		}
		default:
		{
			pos_local = float4(0, 0, 0, 1);
			weight = 0.0f;
			return;
		}
	}
}

// Emit the i'th low-discrepancy surface sample for a primitive, weighted by 'darea' (= area/N).
// 'pos_local' (w=1) and 'normal_local' (w=0) are shape-local. Mirrors EmitSurfaceSample.
odr void BuoyEmitSurfaceSample(in_(BuoyPrimitive) prim, uint index, float darea,
	in_(StructuredBuffer<float4>) verts, in_(StructuredBuffer<float4>) face_planes, in_(StructuredBuffer<int4>) face_verts,
	out_(float4) pos_local, out_(float4) normal_local, out_(float) weight)
{
	weight = darea;
	switch (prim.m_type)
	{
		case BUOY_PRIM_BOX:
		{
			float hx = prim.m_params.x, hy = prim.m_params.y, hz = prim.m_params.z;

			// Area CDF over the 6 faces (constant darea avoids tiny faces being starved of samples).
			float face_area[6] =
			{
				4.0f * hy * hz, 4.0f * hy * hz, // +X, -X
				4.0f * hx * hz, 4.0f * hx * hz, // +Y, -Y
				4.0f * hx * hy, 4.0f * hx * hy, // +Z, -Z
			};
			float sum = 0.0f;
			for (int s = 0; s != 6; ++s) sum += face_area[s];

			float pick = BuoyRadicalInverse(index, 2) * sum;
			float ca = BuoyRadicalInverse(index, 3);
			float cb = BuoyRadicalInverse(index, 5);
			float accum = 0.0f;
			int face = 0;
			for (int f = 0; f != 6; ++f)
			{
				accum += face_area[f];
				face = f;
				if (accum >= pick)
					break;
			}

			float sa = 2.0f * ca - 1.0f;
			float sb = 2.0f * cb - 1.0f;
			switch (face)
			{
				case 0:  { pos_local = float4(+hx, sa * hy, sb * hz, 1.0f); normal_local = float4(+1, 0, 0, 0); return; }
				case 1:  { pos_local = float4(-hx, sa * hy, sb * hz, 1.0f); normal_local = float4(-1, 0, 0, 0); return; }
				case 2:  { pos_local = float4(sa * hx, +hy, sb * hz, 1.0f); normal_local = float4(0, +1, 0, 0); return; }
				case 3:  { pos_local = float4(sa * hx, -hy, sb * hz, 1.0f); normal_local = float4(0, -1, 0, 0); return; }
				case 4:  { pos_local = float4(sa * hx, sb * hy, +hz, 1.0f); normal_local = float4(0, 0, +1, 0); return; }
				default: { pos_local = float4(sa * hx, sb * hy, -hz, 1.0f); normal_local = float4(0, 0, -1, 0); return; }
			}
		}
		case BUOY_PRIM_SPHERE:
		{
			float r = prim.m_params.x;
			float a = BuoyRadicalInverse(index, 2);
			float b = BuoyRadicalInverse(index, 3);
			float z = 1.0f - 2.0f * a;
			float rho = sqrt(max(0.0f, 1.0f - z * z));
			float phi = tau * b;
			float3 dir = float3(rho * cos(phi), rho * sin(phi), z);
			pos_local = float4(dir * r, 1.0f);
			normal_local = float4(dir, 0.0f);
			return;
		}
		case BUOY_PRIM_POLYTOPE:
		{
			float total = BuoyPrimitiveArea(prim, verts, face_verts);
			float pick = BuoyRadicalInverse(index, 2) * total;
			float u1 = BuoyRadicalInverse(index, 3);
			float u2 = BuoyRadicalInverse(index, 5);

			float accum = 0.0f;
			int chosen = 0;
			for (int f = 0; f != prim.m_face_count; ++f)
			{
				int4 fv = face_verts[prim.m_face_ofs + f];
				float3 a = verts[prim.m_vert_ofs + fv.x].xyz;
				float3 b = verts[prim.m_vert_ofs + fv.y].xyz;
				float3 c = verts[prim.m_vert_ofs + fv.z].xyz;
				accum += 0.5f * length(cross(b - a, c - a));
				chosen = f;
				if (accum >= pick)
					break;
			}

			int4 cf = face_verts[prim.m_face_ofs + chosen];
			float3 A = verts[prim.m_vert_ofs + cf.x].xyz;
			float3 B = verts[prim.m_vert_ofs + cf.y].xyz;
			float3 C = verts[prim.m_vert_ofs + cf.z].xyz;
			float su = sqrt(u1);
			float w0 = 1.0f - su;
			float w1 = su * (1.0f - u2);
			float w2 = su * u2;
			float3 p = A * w0 + B * w1 + C * w2;
			float3 n = BuoyNormaliseOrZero(face_planes[prim.m_face_ofs + chosen].xyz);
			pos_local = float4(p, 1.0f);
			normal_local = float4(n, 0.0f);
			return;
		}
		case BUOY_PRIM_TRIANGLE:
		{
			float3 a = verts[prim.m_vert_ofs + 0].xyz;
			float3 b = verts[prim.m_vert_ofs + 1].xyz;
			float3 c = verts[prim.m_vert_ofs + 2].xyz;
			float u1 = BuoyRadicalInverse(index, 2);
			float u2 = BuoyRadicalInverse(index, 3);
			float su = sqrt(u1);
			float w0 = 1.0f - su;
			float w1 = su * (1.0f - u2);
			float w2 = su * u2;
			float3 p = a * w0 + b * w1 + c * w2;

			// PARITY NOTE: the oracle reads a precomputed triangle normal (ShapeTriangle::m_v.w); the
			// GPU model only stores the three corners, so the normal is recomputed here. This matches
			// the oracle to float noise only if that stored normal was the (b-a)x(c-a) winding normal.
			// If exact triangle parity is needed later, store the normal in the GpuPrimitive ABI.
			float3 n = BuoyNormaliseOrZero(cross(b - a, c - a));
			pos_local = float4(p, 1.0f);
			normal_local = float4(n, 0.0f);
			return;
		}
		default:
		{
			pos_local = float4(0, 0, 0, 1);
			normal_local = float4(0, 0, 0, 0);
			weight = 0.0f;
			return;
		}
	}
}

#ifdef __cplusplus
} // namespace pr::physics
#endif

#endif // PR_PHYSICS_BUOYANCY_SAMPLER_HLSLI
