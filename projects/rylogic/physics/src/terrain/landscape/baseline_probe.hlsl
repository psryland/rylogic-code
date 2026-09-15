//*********************************************
// Physics Terrain stage-consumer proof
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "pr/physics/terrain/landscape/baseline_surface.hlsli"

StructuredBuffer<BaselineRecipe> g_recipe : register(t0);
StructuredBuffer<double2> g_positions : register(t1);
RWStructuredBuffer<BaselineResult> g_results : register(u0);
cbuffer Counts : register(b0) { uint g_count; };

// Exercise the public evaluator using explicitly bounded input and output views.
[numthreads(32, 1, 1)]
void CSBaselineProbe(uint3 dispatch_id : SV_DispatchThreadID)
{
	if (dispatch_id.x >= g_count)
		return;

	g_results[dispatch_id.x] = BaselineEvaluate(g_recipe[0], g_positions[dispatch_id.x]);
}

// Return the consumer's actual GPU unit normal, packing XYZ into the three result scalars.
[numthreads(32, 1, 1)]
void CSBaselineNormalProbe(uint3 dispatch_id : SV_DispatchThreadID)
{
	if (dispatch_id.x >= g_count)
		return;

	BaselineResult result = BaselineEvaluate(g_recipe[0], g_positions[dispatch_id.x]);
#if PR_TERRAIN_FP32
	float3 normal = normalize(float3(-(float)result.m_dx, -(float)result.m_dy, 1));
#else
	double3 normal = double3(-result.m_dx, -result.m_dy, 1) * BaselineReciprocalSqrt(1 + result.m_dx * result.m_dx + result.m_dy * result.m_dy);
#endif
	result.m_height = normal.x;
	result.m_dx = normal.y;
	result.m_dy = normal.z;
	g_results[dispatch_id.x] = result;
}

// Probe double-precision root refinement over the full exponent range.
[numthreads(32, 1, 1)]
void CSRootPrecisionProbe(uint3 dispatch_id : SV_DispatchThreadID)
{
	if (dispatch_id.x >= g_count)
		return;

	double value = g_positions[dispatch_id.x].x;
	BaselineResult result;
	result.m_height = BaselineSqrt(value);
	result.m_dx = BaselineReciprocalSqrt(value);
	result.m_dy = 0;
	result.m_status = NoiseFinite(result.m_height) && NoiseFinite(result.m_dx) ? 0 : 2;
	result.m_material_id = result.m_status == 0 ? 0 : -1;
	g_results[dispatch_id.x] = result;
}

// Isolate cancellation-sensitive easing arithmetic from terrain composition.
[numthreads(32, 1, 1)]
void CSNoisePrecisionProbe(uint3 dispatch_id : SV_DispatchThreadID)
{
	if (dispatch_id.x >= g_count)
		return;

	NoiseReal t = (NoiseReal)g_positions[dispatch_id.x].x;
	BaselineResult result;
	result.m_height = NoiseFade(t);
	result.m_dx = NoiseFadeDerivative(t);
	result.m_dy = 0;
	result.m_material_id = 0;
	result.m_status = 0;
	g_results[dispatch_id.x] = result;
}

// Compile an ordinary vertex-stage consumer of exactly the same evaluator (no compute dispatch dependency).
float4 VSBaselineProbe(uint vertex_id : SV_VertexID) : SV_Position
{
	double2 xy = g_positions[vertex_id];
	BaselineResult sample = BaselineEvaluate(g_recipe[0], xy);
	if (sample.m_status != 0)
		return float4(0, 0, 0, 0);

	// Rasterizer positions are floats; conversion happens only after the selected-precision terrain query.
	return float4((float)xy.x, (float)xy.y, (float)sample.m_height, 1);
}
