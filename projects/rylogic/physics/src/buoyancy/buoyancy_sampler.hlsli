//************************************
// Physics Engine
//  Copyright (c) Rylogic Ltd 2026
//************************************
// GPU sampled-composite buoyancy: per-primitive containment, support and surface-cell helpers.
//
// This module is the HLSL mirror of the deterministic CPU oracle in
// include/pr/physics/buoyancy/buoyancy_sampler.h. Every function here must reproduce the oracle's
// arithmetic to single-precision noise so the GPU kernels can be validated against the CPU sampler.
// Surface emission is shared directly through pr/physics/surface/surface_sampling.hlsli.
//
// Binding-agnostic: the geometry-reading functions take the composite-hull StructuredBuffers as
// parameters (resolved at compile time by the including kernel TU). The kernel declares the buffers
// at whatever registers it needs and passes them through; this module pins no registers.
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
static const int BUOY_PRIM_CAPSULE = 4;
static const int BUOY_PRIM_CYLINDER = 5;

// Matches pr::math::constants<float>::tiny (1.00000007e-05f). Used as the zero-length guard so this
// module reproduces the oracle's Normalise(v, fallback) threshold exactly.
static const float BUOY_TINY = 1.00000007e-05f;

// HLSL mirror of pr::physics::buoyancy::GpuPrimitive (112-byte GPU ABI). The kernel reads an array
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
	int m_face_ofs;         // face planes / face verts block start (in the shared face buffers)
	int m_face_count;
	int m_pad0;
	int m_pad1;

	float4 m_params;        // box: half-extents in xyz; sphere: radius in x; capsule/cylinder: radius in x, half-length in y; else unused
};

// Normalise a 3-vector, returning zero when its length is at or below BUOY_TINY. Mirrors the
// oracle's Normalise(v, v4::Zero()) so transformed/face normals degrade identically.
odr float3 BuoyNormaliseOrZero(float3 v)
{
	float len = length(v);
	return len > BUOY_TINY ? v / len : float3(0, 0, 0);
}

// Conservative support interval of a primitive along the world-space 'up' axis, expressed as the
// min/max of dot(point, up) over the primitive in world space. Only box, sphere, capsule and cylinder have a cheap
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
		case BUOY_PRIM_CAPSULE:
		{
			// The segment end points extend along the shape Z axis; the radius adds equally in every direction.
			float3 az = mul(float4(0.0f, 0.0f, 1.0f, 0.0f), s2w).xyz;
			float r = abs(dot(az, up)) * prim.m_params.y + prim.m_params.x;
			lo = c - r;
			hi = c + r;
			return true;
		}
		case BUOY_PRIM_CYLINDER:
		{
			// The caps contribute the half-length along the shape Z axis, and the rim contributes the radius times the sine of the axis tilt.
			float3 az = mul(float4(0.0f, 0.0f, 1.0f, 0.0f), s2w).xyz;
			float cos_tilt = dot(az, up);
			float r = abs(cos_tilt) * prim.m_params.y + sqrt(max(0.0f, 1.0f - cos_tilt * cos_tilt)) * prim.m_params.x;
			lo = c - r;
			hi = c + r;
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
// Surface-cell wetting helpers. Each surface sample stands for a cell, which wets gradually as it crosses the waterline.
//

// Return the submerged fraction of a sample cell centred 'signed_height' along up, given the water height at the same planar
// position and the cell's vertical extent. Mirrors buoyancy::CellWetFraction.
odr float BuoyWetFraction(float signed_height, float water_height, float cell_height)
{
	return saturate((water_height - signed_height) / max(cell_height, 1e-6f) + 0.5f);
}

// Return the vertical extent of a surface cell with area 'darea' whose unit normal has component 'normal_dot_up' along up.
// Mirrors buoyancy::SurfaceCellHeight.
odr float BuoySurfaceCellHeight(float darea, float normal_dot_up)
{
	return sqrt(darea * max(0.0f, 1.0f - normal_dot_up * normal_dot_up));
}

#ifdef __cplusplus
} // namespace pr::physics
#endif

#endif // PR_PHYSICS_BUOYANCY_SAMPLER_HLSLI
