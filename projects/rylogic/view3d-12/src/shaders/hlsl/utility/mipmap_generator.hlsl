//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#include "view3d-12/src/shaders/hlsl/types.hlsli"

// Texture to compute mips for
Texture2D<float4> m_src_texture : reg(t0);
SamplerState m_src_sampler : reg(s0);

// Output texture containing mips
RWTexture2D<float4> m_dst_texture : reg(u0);

// Constants
cbuffer CBufGenMips : reg(b0)
{
	float2 TexelSize; // 1.0 / destination dimension
}

void CSMipMapGenerator(uint3 DTid : SV_DispatchThreadID)
{
	// DTid is the thread ID * the values from numthreads above and in this case correspond to the pixels location in number of pixels.
	// As a result 'tex_coords' (in 0-1 range) will point at the center between the 4 pixels used for the mip-map.
	float2 tex_coords = TexelSize * (DTid.xy + 0.5);

	// The samplers linear interpolation will mix the four pixel values to the new pixels color
	float4 col = m_src_texture.SampleLevel(m_src_sampler, tex_coords, 0);

	//Write the final color into the destination texture.
	m_dst_texture[DTid.xy] = col;
}

// Computer shader
#ifdef PR_RDR_CSHADER_mipmap_generator
numthreads(main, 8, 8, 1)
void main(uint3 DTid : SV_DispatchThreadID)
{
	CSMipMapGenerator(DTid);
}
#endif
