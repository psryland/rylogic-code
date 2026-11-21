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

#endif

#ifdef __cplusplus
	static_assert(sizeof(ParticleMeshRecord) == 48);
	static_assert(sizeof(ParticleMeshVertex) == 32);
}
#endif
#endif
