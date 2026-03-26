//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#include "view3d-12/src/shaders/hlsl/types.hlsli"

struct CBufGenMips
{
	float2 texel_size; // 1.0 / destination dimension
};

// Resources
ConstantBuffer<CBufGenMips> g_mmap : register(b0);
Texture2D<float4> g_src_texture : register(t0); // Texture to compute mips for
RWTexture2D<float4> g_dst_texture : register(u0); // Output texture containing mips
SamplerState g_src_sampler : register(s0);


void CSMipMapGenerator(uint3 DTid : SV_DispatchThreadID)
{
	// DTid is the thread ID * the values from numthreads above and in this case correspond to the pixels location in number of pixels.
	// As a result 'tex_coords' (in 0-1 range) will point at the center between the 4 pixels used for the mip-map.
	float2 tex_coords = g_mmap.texel_size * (DTid.xy + 0.5);

	// The samplers linear interpolation will mix the four pixel values to the new pixels color
	float4 col = g_src_texture.SampleLevel(g_src_sampler, tex_coords, 0);

	//Write the final color into the destination texture.
	g_dst_texture[DTid.xy] = col;
}

// Computer shader
#ifdef PR_RDR_CSHADER_mipmap_generator
numthreads(main, 8, 8, 1)
void main(uint3 DTid : SV_DispatchThreadID)
{
	CSMipMapGenerator(DTid);
}
#endif
