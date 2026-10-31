#include "pr/hlsl/interop.hlsli"
#include "pr/view3d-12/shaders/population.hlsli"

cbuffer PopulationProducerConstants : register(VIEW3D_POPULATION_PRODUCER_CONSTANTS_REGISTER)
{
	// x = instances to write, y = live count to publish for block 0, z = grid columns, w = tint (0xAARRGGBB).
	uint4 g_count;

	// xyz = position of instance 0, w = grid spacing.
	float4 g_origin;
}

RWStructuredBuffer<View3DPopulationInstance> g_instances : register(VIEW3D_POPULATION_PRODUCER_INSTANCES_REGISTER);
RWStructuredBuffer<uint> g_live_counts : register(VIEW3D_POPULATION_PRODUCER_LIVE_COUNTS_REGISTER);
RWByteAddressBuffer g_payload : register(VIEW3D_POPULATION_PRODUCER_PAYLOAD_REGISTER);

// Place instances on a grid and publish block 0's live count. The instance and live counts may deliberately exceed the population capacities.
numthreads(CSMain, 64, 1, 1)
void CSMain(uint index : SV_DispatchThreadID)
{
	// Thread 0 publishes the live count and touches the payload, which may be a null view.
	if (index == 0)
	{
		g_live_counts[0] = g_count.y;
		g_payload.Store(0, g_count.x);
	}

	// Threads past the instance count belong to the partial final group.
	if (index >= g_count.x)
		return;

	// Place the instance on its grid cell with an identity rotation.
	float3 pos = g_origin.xyz + g_origin.w * float3(index % g_count.z, index / g_count.z, 0);
	View3DPopulationInstance inst;
	inst.o2w[0] = float4(1, 0, 0, pos.x);
	inst.o2w[1] = float4(0, 1, 0, pos.y);
	inst.o2w[2] = float4(0, 0, 1, pos.z);
	inst.info = uint4(g_count.w, VIEW3D_POPULATION_FLAG_ENABLED, 0, 0);
	g_instances[index] = inst;
}
