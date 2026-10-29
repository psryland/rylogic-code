// Detail-normal forward pixel family fixture: stock detail-normal shading with its colour channels rotated.
#include "pr/view3d-12/shaders/forward_pixel_detail.hlsli"

// Rotate the detail-normal shaded colour so the output proves both the detail tilt and the caller's pixel stage ran.
float4 RotateDetailShade(inout PSIn In, bool is_front_face)
{
	// The stock detail shading applies the layers' tilt before lighting.
	float4 diff = DetailShade(In, is_front_face);
	diff.rgb = diff.gbr;
	return diff;
}

// Generate the three forward pixel entry points.
VIEW3D_FORWARD_PIXEL_ENTRY_POINTS(RotateDetail, RotateDetailShade)
