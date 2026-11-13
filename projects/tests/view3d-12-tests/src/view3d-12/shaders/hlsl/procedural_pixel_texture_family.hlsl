// Forward pixel family fixture: replaces the stock colour with the centre texel of procedural texture slot 2.
#include "pr/view3d-12/shaders/forward_pixel.hlsli"

Texture2D<float4> g_textures[VIEW3D_PROCEDURAL_TEXTURE_COUNT] : register(VIEW3D_PROCEDURAL_TEXTURE_REGISTER);
SamplerState g_texture_sampler : register(VIEW3D_PROCEDURAL_SAMPLER_REGISTER);

// Output the slot 2 texel so the image proves which slot is bound, and that unbound slots read zero.
float4 TextureShade(inout PSIn In, bool is_front_face)
{
	// Keep the stock alpha so opaque and transparent materials still choose their own entry points.
	float4 diff = ForwardShade(In, is_front_face).diff;
	diff.rgb = g_textures[2].SampleLevel(g_texture_sampler, float2(0.5, 0.5), 0).rgb;
	return diff;
}

// Generate the three forward pixel entry points.
VIEW3D_FORWARD_PIXEL_ENTRY_POINTS(Texture, TextureShade)
