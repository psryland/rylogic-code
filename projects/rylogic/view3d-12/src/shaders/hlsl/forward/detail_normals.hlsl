// Opt-in simple-material variants that tilt the interpolated normal with world-projected detail-normal layers.
// The stock entry points shade the perturbed fragment unchanged, so materials without detail normals pay nothing.
#include "pr/view3d-12/shaders/forward_pixel_detail.hlsli"

// Generate the forward pixel family for detail-normal materials.
VIEW3D_FORWARD_PIXEL_ENTRY_POINTS(ForwardDetail, DetailShade)
