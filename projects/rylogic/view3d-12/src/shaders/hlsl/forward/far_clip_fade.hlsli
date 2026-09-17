// Shared forward-fragment partitioning; scene/far_clip_fade.md owns the rendering contract.
#ifndef PR_VIEW3D_FAR_CLIP_FADE_HLSLI
#define PR_VIEW3D_FAR_CLIP_FADE_HLSLI

// Return camera-forward depth, matching the camera projection rather than radial distance.
float FarClipForwardDepth(float4 ws_vert)
{
	return -mul(ws_vert, g_frame.cam.w2c).z;
}

// Return a smooth opacity ramp that reaches zero before hardware clipping.
float FarClipOpacity(float depth)
{
	return 1.0f - smoothstep(g_nugget.far_clip_fade.x, g_nugget.far_clip_fade.y, depth);
}

// Keep fully opaque coverage in the depth-writing pass; the remaining coverage belongs to alpha collect.
void ClipFarFadeOpaque(float4 ws_vert)
{
	if (g_nugget.far_clip_fade.z != 0.0f && FarClipForwardDepth(ws_vert) > g_nugget.far_clip_fade.x)
		discard;
}

// Reject non-contributing coverage before re-shading opaque geometry in the alpha pass.
void ClipFarFadeCollect(float4 ws_vert)
{
	if (g_nugget.far_clip_fade.z == 0.0f)
		return;

	float depth = FarClipForwardDepth(ws_vert);
	if (depth >= g_nugget.far_clip_fade.y || (g_nugget.far_clip_fade.z == 1.0f && depth <= g_nugget.far_clip_fade.x))
		discard;
}

// Preserve source opacity for blended materials and full coverage for surviving opaque/cutout fragments.
float4 ApplyFarFadeAlpha(float4 ws_vert, float4 colour)
{
	if (g_nugget.far_clip_fade.z == 0.0f)
		return colour;

	// Opaque and surviving cutout fragments were fully covering, regardless of their unused output alpha.
	if (g_nugget.far_clip_fade.z == 1.0f)
		colour.a = 1.0f;

	colour.a *= FarClipOpacity(FarClipForwardDepth(ws_vert));
	return colour;
}
#endif
