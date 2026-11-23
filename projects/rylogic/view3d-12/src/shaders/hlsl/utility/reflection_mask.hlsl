//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
// Masks the pixels of the reflection's mirror view that no surface on the reflection plane can show (see reflection_plane.hlsl).
// The mirror view's pixel at (x, y) holds the reflection seen at the main view's pixel (x, y) / scale relative to the viewport, where 'scale' is
// the mirror view's resolution as a fraction of the viewport's, because a point on the plane is its own mirror image. A main view pixel can show the plane only when the plane is in front of the visible surface. The camera is above
// the plane, so that is the case exactly when the visible surface is below the plane, or when the pixel shows the sky below the plane's horizon.
//
// The mask draws one full-screen triangle into the mirror view's depth after the depth is cleared and before the scene is drawn. It writes the
// nearest depth to every pixel that no nearby main view pixel shows the plane at, so the depth test rejects the scene's fragments there. Pixels
// near the plane's outline stay open too, because waves bend the reflected rays and read reflection texels a short distance away.
#include "view3d-12/src/shaders/hlsl/types.hlsli"

struct CBufReflectionMask
{
	row_major float4x4 s2w; // The main view's normalised device coordinates to world space
	float4 plane;           // xyz = unit normal, w = distance of the plane from the origin along the normal
	float4 cam;             // xyz = world-space camera position of the main view, w unused
	float4 ndc_map;         // Viewport-relative main view pixel position p maps to normalised device position (p.x * x + y, p.y * z + w)
	float4 load;            // xy = offset from a viewport-relative main view pixel to the main depth pixel, z = margin in main view pixels, w = main view pixels per mirror pixel
	float4 bounds;          // xy = the lowest main depth pixel of the viewport, zw = the highest
};

// Resources
ConstantBuffer<CBufReflectionMask> g_cb : register(b0);
Texture2DMS<float> g_depth : register(t0); // The main view's opaque depth. Depth is reversed, with 0 at the far plane

// Directions to the tested neighbours of a pixel, in units of the margin
static const float2 ReflectionMaskTaps[9] =
{
	float2(0, 0),
	float2(1, 0), float2(-1, 0), float2(0, 1), float2(0, -1),
	float2(0.7071f, 0.7071f), float2(-0.7071f, 0.7071f), float2(0.7071f, -0.7071f), float2(-0.7071f, -0.7071f),
};

// True when the main view pixel at viewport-relative position 'p' shows the reflection plane
bool ShowsReflectionPlane(float2 p)
{
	// Find the main view pixel, clamped to the viewport, and its normalised device position
	int2 pix = clamp((int2)floor(p + g_cb.load.xy), (int2)g_cb.bounds.xy, (int2)g_cb.bounds.zw);
	float2 q = (float2)pix - g_cb.load.xy + 0.5f;
	float2 ndc = float2(q.x * g_cb.ndc_map.x + g_cb.ndc_map.y, q.y * g_cb.ndc_map.z + g_cb.ndc_map.w);
	float depth = g_depth.Load(pix, 0);
	float3 n = g_cb.plane.xyz;

	// A visible surface below the plane is seen through it
	if (depth > 0.0f)
	{
		float4 ws = mul(float4(ndc, depth, 1.0f), g_cb.s2w);
		return dot(n, ws.xyz / ws.w) - g_cb.plane.w < 0.0f;
	}

	// The sky is seen through the plane when the view ray points below the horizon. The far point of the ray, in homogeneous form, gives the
	// ray's direction from the camera even when it is at infinity.
	float4 far_pt = mul(float4(ndc, 0.0f, 1.0f), g_cb.s2w);
	float3 dir = (far_pt.xyz - g_cb.cam.xyz * far_pt.w) * (far_pt.w < 0.0f ? -1.0f : 1.0f);
	return dot(dir, n) < 0.0f;
}

// Cover the whole viewport at the near plane. Depth is reversed, so the nearest depth is 1.
float4 VSReflectionMask(uint vid : SV_VertexID) : SV_Position
{
	// Generate the triangle from the vertex index
	float2 t = float2((vid << 1) & 2, vid & 2);
	return float4(t * float2(2, -2) + float2(-1, 1), 1, 1);
}

// Leave open the pixels that show the plane or are near one that does. All others keep the triangle's depth.
void PSReflectionMask(float4 ss : SV_Position)
{
	// Test the main view pixel under this mirror pixel and its neighbours at the margin's distance
	float2 p = ss.xy * g_cb.load.w;
	[unroll] for (int i = 0; i != 9; ++i)
	{
		// Discarding leaves the cleared depth, so the scene draws there
		if (ShowsReflectionPlane(p + ReflectionMaskTaps[i] * g_cb.load.z))
			discard;
	}
}
