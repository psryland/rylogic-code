//*********************************************
// Physics Engine — GPU Sort and Sweep Compute Shader
//  Copyright (C) Rylogic Ltd 2025
//*********************************************
// This shader performs the 'sweep' step of a sort and sweep.
// It takes a sorted index list, performs AABB overlap tests,
// and outputs a list of potential collision pairs.
#include "pr/hlsl/core.hlsli"
#include "pr/hlsl/spatial_algebra.hlsli"

// Shader Resources
StructuredBuffer<RigidBodyDynamics> g_bodies : register(t0);
StructuredBuffer<int> g_aabb_idx : register(t1);
StructuredBuffer<int> g_collision_pairs : register(u0);

// Shader parameters
cbuffer cbSweep : register(b0)
{
	int g_body_count;
	int g_pad0;
	int g_pad1;
	int g_pad2;
};

[numthreads(64, 1, 1)]
void CSSweep(int3 dtid : SV_DispatchThreadID)
{
	int idx = dtid.x;
	if (idx >= g_body_count)
		return;

	
}