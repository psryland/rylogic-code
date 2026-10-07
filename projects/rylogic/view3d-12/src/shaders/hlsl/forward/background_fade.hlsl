//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
// Far clip fade to the background. See scene/far_clip_fade.md for the rendering contract.

// The main depth buffer after the opaque scene. It is always a multisampled resource, even at 1x.
Texture2DMS<float> g_depth :register(t0);

// Root constants for the fade pass.
cbuffer CBufBackgroundFade :register(b0)
{
	// Terms that convert a depth buffer value 'd' to camera-forward depth: -(x*d + y) / (z*d + w).
	float4 g_depth_to_view;

	// x/y = camera-forward depth interval over which the scene fades to the background.
	float4 g_fade_range;

	// The linear clear colour, used when the scene has no background objects.
	float4 g_clear_colour;
};

struct PSIn_BackgroundFade
{
	float4 ss_vert :SV_Position;
};

// A triangle that covers the whole viewport.
PSIn_BackgroundFade VSBackgroundFade(uint vid :SV_VertexID)
{
	// Generate the corners (-1,1), (3,1), (-1,-3) from the vertex index.
	PSIn_BackgroundFade Out = (PSIn_BackgroundFade)0;
	float2 uv = float2((vid << 1) & 2, vid & 2);
	Out.ss_vert = float4(uv * float2(2, -2) + float2(-1, 1), 0, 1);
	return Out;
}

// Return the background weight of one depth sample, from 0 in front of the fade interval to 1 beyond it.
float BackgroundFade(float4 ss_vert, uint sample_index)
{
	// Convert the stored depth to camera-forward depth.
	float d = g_depth.Load(int2(ss_vert.xy), sample_index);
	float view_depth = -(g_depth_to_view.x * d + g_depth_to_view.y) / (g_depth_to_view.z * d + g_depth_to_view.w);
	return smoothstep(g_fade_range.x, g_fade_range.y, view_depth);
}

// Store the opaque weight (1 - fade) in destination alpha. The background objects that follow blend by this weight.
// Every sample is written, so the background blend never depends on alpha from the opaque shaders.
// Reading 'SV_SampleIndex' runs the shader once per sample, so each MSAA sample gets the weight of its own depth.
float4 PSBackgroundFadeWeight(PSIn_BackgroundFade In, uint sample_index :SV_SampleIndex) :SV_Target
{
	// Only the alpha channel is written.
	float fade = BackgroundFade(In.ss_vert, sample_index);
	return float4(0, 0, 0, 1.0f - fade);
}

// Blend the clear colour over the faded samples directly, because there are no background objects to draw.
float4 PSBackgroundFadeClear(PSIn_BackgroundFade In, uint sample_index :SV_SampleIndex) :SV_Target
{
	// The blend state uses source alpha as the fade weight. Samples in front of the fade interval are unchanged, so skip their writes.
	float fade = BackgroundFade(In.ss_vert, sample_index);
	clip(fade - 1.0f / 1024.0f);
	return float4(g_clear_colour.rgb, fade);
}
