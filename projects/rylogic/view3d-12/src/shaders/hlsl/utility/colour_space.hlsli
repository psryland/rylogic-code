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

// Convert linear colour channels to sRGB-encoded values.
float3 LinearToSrgb(float3 lin)
{
	float3 x = saturate(lin);
	float3 low = x * 12.92f;
	float3 high = 1.055f * pow(x, 1.0f / 2.4f) - 0.055f;
	return lerp(low, high, step(0.0031308f, x));
}

// Convert a linear colour to sRGB-encoded RGB, leaving alpha unchanged.
float4 LinearToSrgb(float4 lin)
{
	return float4(LinearToSrgb(lin.rgb), lin.a);
}

// Return a well-mixed 32-bit hash of 'v'.
uint DitherHash(uint v)
{
	uint state = v * 747796405u + 2891336453u;
	uint word = ((state >> ((state >> 28u) + 4u)) ^ state) * 277803737u;
	return (word >> 22u) ^ word;
}

// Return noise in [-1, 1] with a triangular distribution. The value is fixed for each 'pixel' and 'seed' so still images do not flicker.
float DitherNoise(uint2 pixel, uint seed)
{
	// The sum of two independent uniform values has a triangular distribution. This makes the average error of the
	// dithered result independent of the colour value, so no faint banding pattern remains.
	uint h0 = DitherHash(pixel.x + DitherHash(pixel.y + DitherHash(seed)));
	uint h1 = DitherHash(h0);
	float u0 = float(h0 >> 8) * (1.0f / 16777216.0f);
	float u1 = float(h1 >> 8) * (1.0f / 16777216.0f);
	return u0 + u1 - 1.0f;
}

// Return the sRGB-encoded offset that dithers one pixel before it is stored with 8 bits per channel.
// 'amount' scales the noise: 1 gives a peak offset of one 8-bit step, and 0 disables dithering.
float DitherOffsetSrgb8(uint2 pixel, float amount, uint seed)
{
	return DitherNoise(pixel, seed) * (amount / 255.0f);
}

// Add noise to a linear colour so its 8-bit sRGB encoding shows noise instead of visible colour bands.
// See 'DitherOffsetSrgb8' for 'amount'. The noise is added in sRGB-encoded space, where the 8-bit steps are evenly spaced.
float3 DitherSrgb8(float3 colour, uint2 pixel, float amount, uint seed)
{
	// Leave the colour unchanged when dithering is disabled so exact colours remain exact.
	if (amount <= 0.0f)
		return colour;

	return SrgbToLinear(LinearToSrgb(colour) + DitherOffsetSrgb8(pixel, amount, seed));
}

#endif

