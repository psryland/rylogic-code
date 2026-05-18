//***********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//***********************************************
#ifndef PR_VIEW3D_SHADER_COLOUR_SPACE_HLSLI
#define PR_VIEW3D_SHADER_COLOUR_SPACE_HLSLI

// Convert sRGB-encoded colour channels to linear values.
float3 SrgbToLinear(float3 srgb)
{
	float3 x = saturate(srgb);
	float3 low = x / 12.92f;
	float3 high = pow((x + 0.055f) / 1.055f, 2.4f);
	return lerp(low, high, step(0.04045f, x));
}

// Convert an sRGB-encoded colour to linear RGB, leaving alpha unchanged.
float4 SrgbToLinear(float4 srgb)
{
	return float4(SrgbToLinear(srgb.rgb), srgb.a);
}

#endif

