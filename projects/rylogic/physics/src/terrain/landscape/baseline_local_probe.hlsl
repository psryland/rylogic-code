//*********************************************
// Physics Terrain local evaluator stage-consumer proof
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "pr/physics/terrain/landscape/baseline_surface_local.hlsli"

ByteAddressBuffer g_recipe : register(t0);
StructuredBuffer<double2> g_positions : register(t1);
RWStructuredBuffer<BaselineResult> g_results : register(u0);
cbuffer Counts : register(b0) { uint g_count; };

// Sample the local evaluator at single-precision local positions and widen its result into the shared result layout.
[numthreads(32, 1, 1)]
void CSBaselineLocalProbe(uint3 dispatch_id : SV_DispatchThreadID)
{
	// The test supplies float-representable local positions, so the narrowing here is exact.
	if (dispatch_id.x >= g_count)
		return;

	float4 weights;
	BaselineLocalResult sample = BaselineEvaluateLocal(g_recipe, 0, (float2)g_positions[dispatch_id.x], weights);
	BaselineResult result;
	result.m_height = sample.m_height;
	result.m_dx = sample.m_dx;
	result.m_dy = sample.m_dy;
	result.m_material_id = sample.m_material_id;
	result.m_status = sample.m_status;
	g_results[dispatch_id.x] = result;
}

// A vertex-stage consumer with no double-precision inputs or outputs, so its DXIL must not require double-precision support.
float4 VSBaselineLocalProbe(uint vertex_id : SV_VertexID) : SV_Position
{
	// Derive a local position from the vertex ID so no double-precision resource is read.
	float2 local = float2(vertex_id & 0xFFFFu, vertex_id >> 16) * 0.25f;
	float4 weights;
	BaselineLocalResult sample = BaselineEvaluateLocal(g_recipe, 0, local, weights);
	return float4(local, sample.m_height, sample.m_status == 0 ? weights.x + 1 : 0);
}


// Sample the local evaluator with the decal blob applied, for comparison against the decorated CPU surface.
ByteAddressBuffer g_decals : register(t2);
[numthreads(32, 1, 1)]
void CSBaselineLocalDecalProbe(uint3 dispatch_id : SV_DispatchThreadID)
{
	// Same as CSBaselineLocalProbe, with the decals applied to the field.
	if (dispatch_id.x >= g_count)
		return;

	float4 weights;
	float2 local = (float2)g_positions[dispatch_id.x];
	BaselineLocalResult sample = BaselineEvaluateLocal(g_recipe, 0, local, weights);
	BaselineApplyDecalsLocal(g_decals, 0, local, sample);
	BaselineResult result;
	result.m_height = sample.m_height;
	result.m_dx = sample.m_dx;
	result.m_dy = sample.m_dy;
	result.m_material_id = sample.m_material_id;
	result.m_status = sample.m_status;
	g_results[dispatch_id.x] = result;
}