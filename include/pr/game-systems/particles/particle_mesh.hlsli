//************************************
// Game Systems
//  Copyright (c) Rylogic Ltd 2026
//************************************
// The stock mesh particle batch: each record draws one copy of a small mesh, such as a leaf, a wood chip or a bird, placed, turned and scaled
// by the record. The mesh is shared by every record of the batch and is uploaded once, as the batch's vertex data.
//
// A batch is drawn by the stock mesh vertex shader. Any particle type whose update kernel writes ParticleMeshRecords can add itself to the batch.
// Mesh positions are in units of the record's size, so a mesh within a ball of radius r draws within a ball of radius r * size around the record's
// position. Types use that to write conservative particle bounds. Normals light the mesh, and UVs sample the batch texture, if it has one.
#ifndef PR_GAME_SYSTEMS_PARTICLE_MESH_HLSLI
#define PR_GAME_SYSTEMS_PARTICLE_MESH_HLSLI
#include "pr/game-systems/particles/particles.hlsli"

#ifdef __cplusplus
namespace pr::game_systems::particles
{
	using namespace pr::hlsl;
#endif

// What the mesh vertex shader draws for one particle.
struct ParticleMeshRecord
{
	float4 pos_size;     // xyz = Position of the mesh origin. w = Scale of the mesh.
	float4 orientation;  // Unit quaternion (x, y, z, w) that turns the mesh.
	uint colour;         // 0xAARRGGBB. Multiplies the batch texture, if any.
	uint pad0;
	uint pad1;
	uint pad2;
};

// One vertex of the shared mesh. The UV is split across the two float3 rows so the vertex packs into 32 bytes.
struct ParticleMeshVertex
{
	float3 pos;   // Position, in units of the record's size.
	float u;      // Texture coordinate u.
	float3 norm;  // Unit normal.
	float v;      // Texture coordinate v.
};

#ifndef __cplusplus

// Rotate 'v' by the unit quaternion 'q'.
float3 ParticleRotate(float4 q, float3 v)
{
	return v + 2.0f * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}

// Return the quaternion that applies 'b' then 'a'.
float4 ParticleQuatMul(float4 a, float4 b)
{
	return float4(a.w * b.xyz + b.w * a.xyz + cross(a.xyz, b.xyz), a.w * b.w - dot(a.xyz, b.xyz));
}

// Return the quaternion that turns by 'angle' radians about the unit 'axis'.
float4 ParticleQuatAxisAngle(float3 axis, float angle)
{
	// A quaternion holds the sine and cosine of half the angle.
	float sine;
	float cosine;
	sincos(0.5f * angle, sine, cosine);
	return float4(axis * sine, cosine);
}

// Return the shortest-arc quaternion that turns +z onto the unit vector 'n'.
float4 ParticleQuatFromZ(float3 n)
{
	// The half-way vector between z and n gives the rotation directly. Turning z onto -z has no unique axis, so turn about x.
	if (n.z < -0.9999f)
		return float4(1.0f, 0.0f, 0.0f, 0.0f);

	return normalize(float4(-n.y, n.x, 0.0f, 1.0f + n.z));
}

// Return a uniformly random unit vector from two random values in [0, 1).
float3 ParticleRandomDirection(float2 u)
{
	// A uniform height on the sphere and a uniform angle around it give a uniform direction.
	float z = 2.0f * u.x - 1.0f;
	float r = sqrt(saturate(1.0f - z * z));
	float phi = 6.283185307179586f * u.y;
	return float3(r * cos(phi), r * sin(phi), z);
}

// Return the stock scale ramp of a mesh particle at 'age' of 'lifetime': growing from zero over 'grow' seconds after birth and shrinking to zero over
// 'shrink' seconds before death. A zero time means no ramp at that end.
float ParticleMeshScaleRamp(float age, float lifetime, float grow, float shrink)
{
	// Each end ramps linearly. The ramps multiply, so a life shorter than both ramps never reaches full scale.
	float scale_in = grow > 0.0f ? saturate(age / grow) : 1.0f;
	float scale_out = shrink > 0.0f ? saturate((lifetime - age) / shrink) : 1.0f;
	return scale_in * scale_out;
}

#endif

#ifdef __cplusplus
	static_assert(sizeof(ParticleMeshRecord) == 48);
	static_assert(sizeof(ParticleMeshVertex) == 32);
}
#endif
#endif
