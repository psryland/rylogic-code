//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
// Copies a rendered environment map face and stores each texel's distance from the capture position in a separate distance texture.
#include "view3d-12/src/shaders/hlsl/types.hlsli"

struct CBufEnvMapDistance
{
	row_major float4x4 s2c; // Screen (normalised device coordinates) to camera space for the face camera
	float4 info;            // x = face size in pixels, y = distance scale 'S'
};

// Resources
ConstantBuffer<CBufEnvMapDistance> g_cb : register(b0);
Texture2D<float4> g_colour : register(t0);   // The rendered face
Texture2D<float> g_depth : register(t1);     // The face's depth buffer, with 1 at the far plane
RWTexture2D<float4> g_face : register(u0);   // Mip 0 of the face texture that receives the colour
RWTexture2D<float> g_distance : register(u1); // Mip 0 of the face texture that receives the distance

// Compute shader entry point
numthreads(CSEnvMapDistance, 8, 8, 1)
void CSEnvMapDistance(uint3 DTid : SV_DispatchThreadID)
{
	// Threads beyond the face edge have no texel
	uint size = (uint)g_cb.info.x;
	if (DTid.x >= size || DTid.y >= size)
		return;

	// Unproject the depth to a camera-space position. Its length is the distance from the capture position, because the face camera is at
	// that position. Distance 'd' is stored as 'd / (d + S)', which gives most precision near 'S' and maps the far plane (nothing rendered) to 1.
	float depth = g_depth[DTid.xy];
	float a = 1.0f;
	if (depth < 1.0f)
	{
		float2 uv = (DTid.xy + 0.5f) / size;
		float4 cs = mul(float4(uv.x * 2.0f - 1.0f, 1.0f - uv.y * 2.0f, depth, 1.0f), g_cb.s2c);
		float d = length(cs.xyz / cs.w);
		a = d / (d + g_cb.info.y);
	}

	// Reflections treat the captured environment as opaque, so the colour's alpha is always 1
	g_face[DTid.xy] = float4(g_colour[DTid.xy].rgb, 1.0f);
	g_distance[DTid.xy] = a;
}
