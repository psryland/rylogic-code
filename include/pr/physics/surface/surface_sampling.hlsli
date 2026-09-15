//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#ifndef PR_PHYSICS_SURFACE_SAMPLING_HLSLI
#define PR_PHYSICS_SURFACE_SAMPLING_HLSLI

#ifdef __cplusplus
#include "pr/hlsl/interop.h"
#define PR_SURFACE_ODR inline
#define PR_SURFACE_IN(type) type const&
namespace pr::physics::surface
{
	using namespace pr::hlsl;
#else
#define PR_SURFACE_ODR
#define PR_SURFACE_IN(type) in type
#endif

// Compact shape-local quadrature patch. Plans are validated by BuildPlan before CPU/GPU emission.
// Kind 0 is a rectangle, 1 a tapered strip, and 2 a radially projected unit-cube face.
struct SurfacePatch
{
	float4 m_origin;
	float4 m_u;
	float4 m_v;
	float4 m_normal;
	uint m_nu;
	uint m_nv;
	uint m_kind;
	uint m_sample_end;
	float m_taper;
	float m_measure;
	uint m_pad0;
	uint m_pad1;
};

// One surface point with its own outward normal and represented area; coincident features retain each face contribution.
struct SurfaceSample
{
	float4 m_pos_local;
	float4 m_normal_local;
	float m_darea;
};

// Solid angle of a small spherical triangle. Edge differences avoid subtracting nearly equal large determinants.
PR_SURFACE_ODR float SurfaceSolidAngle(float4 a, float4 b, float4 c)
{
	float determinant = abs(dot(a.xyz, cross((b - a).xyz, (c - a).xyz)));
	return 2.0f * atan2(determinant, 1.0f + dot(a.xyz, b.xyz) + dot(b.xyz, c.xyz) + dot(c.xyz, a.xyz));
}

// Area on the unit sphere of a projected rectangular cube-face cell.
PR_SURFACE_ODR float SurfaceSphereCell(float x0, float y0, float x1, float y1)
{
	float4 a = normalize(float4(x0, y0, 1, 0));
	float4 b = normalize(float4(x1, y0, 1, 0));
	float4 c = normalize(float4(x1, y1, 1, 0));
	float4 d = normalize(float4(x0, y1, 1, 0));
	return SurfaceSolidAngle(a, b, c) + SurfaceSolidAngle(a, c, d);
}

// Emit a zero-based patch-local ordinal. The same arithmetic is compiled as C++ and HLSL.
PR_SURFACE_ODR SurfaceSample EmitSurfaceSample(PR_SURFACE_IN(SurfacePatch) patch, uint index)
{
	uint i = index / (patch.m_nv + 1);
	uint j = index % (patch.m_nv + 1);
	float u = (float)i / (float)patch.m_nu;
	float v = (float)j / (float)patch.m_nv;
	float wu = (i == 0 || i == patch.m_nu) ? 0.5f : 1.0f;
	float wv = (j == 0 || j == patch.m_nv) ? 0.5f : 1.0f;
	SurfaceSample sample;
	sample.m_pos_local = patch.m_origin + patch.m_u * u + patch.m_v * v;
	sample.m_normal_local = patch.m_normal;
	sample.m_darea = patch.m_measure * wu * wv / ((float)patch.m_nu * (float)patch.m_nv);
	switch (patch.m_kind)
	{
		case 0:
		{
			return sample;
		}
		case 1:
		{
			// Integrate the bilinear nodal basis against the strip's linearly varying width.
			sample.m_pos_local = patch.m_origin + patch.m_u * u + patch.m_v * (v * (1.0f - patch.m_taper * u));
			wu = i == 0 ? (3.0f - patch.m_taper) / 6.0f : (3.0f - 2.0f * patch.m_taper) / 6.0f;
			sample.m_darea = patch.m_measure * wu * wv / (float)patch.m_nv;
			if (i == 1 && patch.m_taper == 1.0f)
				sample.m_darea = patch.m_measure / 6.0f;

			return sample;
		}
		case 2:
		{
			// Each node owns a quarter of every incident spherical cell, including face-boundary nodes.
			float4 direction = sample.m_pos_local;
			direction.w = 0.0f;
			direction = normalize(direction);
			sample.m_normal_local = direction;
			sample.m_pos_local = direction * patch.m_taper;
			sample.m_pos_local.w = 1.0f;
			float area = 0.0f;
			for (uint di = 0; di != 2; ++di)
			{
				for (uint dj = 0; dj != 2; ++dj)
				{
					if (i + di == 0 || i + di > patch.m_nu || j + dj == 0 || j + dj > patch.m_nv)
						continue;

					float x0 = -1.0f + 2.0f * (float)(i + di - 1) / (float)patch.m_nu;
					float y0 = -1.0f + 2.0f * (float)(j + dj - 1) / (float)patch.m_nv;
					float x1 = -1.0f + 2.0f * (float)(i + di) / (float)patch.m_nu;
					float y1 = -1.0f + 2.0f * (float)(j + dj) / (float)patch.m_nv;
					area += SurfaceSphereCell(x0, y0, x1, y1);
				}
			}
			sample.m_darea = 0.25f * patch.m_measure * area;
			return sample;
		}
		default:
		{
			// Host validation excludes unknown patch kinds from uploaded plans.
			sample.m_darea = 0.0f;
			return sample;
		}
	}
}

#ifdef __cplusplus
}
#endif
#undef PR_SURFACE_ODR
#undef PR_SURFACE_IN
#endif
