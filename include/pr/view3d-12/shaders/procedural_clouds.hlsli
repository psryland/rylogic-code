//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
// Cloud shadows for procedural Forward shaders. The stock pixel shaders shade every directional light from the sky's sun by the clouds per pixel.
// A procedural shader can instead find the cloud shadow once per vertex, pass it to its own pixel shader (e.g. in 'View3DForwardVertexOut.idx0'),
// and apply it to the sun there. The cloud field varies slowly, so per-vertex shadows look the same on all but very large triangles.
// Include this in a Forward stage shader only. It declares the sky's cloud textures and sampler at the registers the Forward step binds them to.
#ifndef PR_VIEW3D_PROCEDURAL_CLOUDS_HLSLI
#define PR_VIEW3D_PROCEDURAL_CLOUDS_HLSLI
#include "pr/view3d-12/shaders/procedural_vertex.hlsli"
#include "view3d-12/src/shaders/hlsl/sky/cloud_field.hlsli"

// The fraction of direct sunlight that passes through the sky's clouds to the world-space position 'ws_pos', in [0,1].
// It is 1 when the scene has no cloud shadows. Apply it to a light in proportion to 'View3DSunAlignment'.
float View3DSunCloudShadow(View3DForwardFrame frame, float3 ws_pos)
{
	// The cloud field is centred on the camera.
	return CloudSunShadow(frame.clouds, ws_pos, frame.cam.c2w[3].xyz);
}

// How much a light in the world-space direction 'ws_to_light' (normalised, toward the light) is the sky's sun, in [0,1].
// The light's visibility through the clouds is 'lerp(1, View3DSunCloudShadow(...), View3DSunAlignment(...))'.
float View3DSunAlignment(View3DForwardFrame frame, float3 ws_to_light)
{
	// Only light from the sun is shaded by its clouds.
	return CloudSunAlignment(frame.clouds, ws_to_light);
}

#endif
