// A square grid of vertices displaced by a travelling wave. All geometry is generated from the logical vertex index.
#include "pr/view3d-12/shaders/procedural_vertex.hlsli"

// Must match 'WaveGridConstants' in demos_procedural.cpp
struct WaveGridConstants
{
	float4 wave;      // x = amplitude, y = spatial frequency (radians per unit), z = angular speed (radians per second), w = time (seconds)
	float4 grid;      // x = vertices per side, y = side length
	float4 colour_lo; // Colour at the wave troughs
	float4 colour_hi; // Colour at the wave crests
};

ConstantBuffer<View3DForwardFrame> g_frame : register(VIEW3D_FORWARD_FRAME_REGISTER);
ConstantBuffer<View3DElementIndex> g_element : register(VIEW3D_FORWARD_ELEMENT_INDEX_REGISTER);
StructuredBuffer<View3DForwardElement> g_elements : register(VIEW3D_FORWARD_ELEMENTS_REGISTER);
static const View3DForwardElement g_nugget = g_elements[g_element.index];
ConstantBuffer<WaveGridConstants> g_wave : register(VIEW3D_PROCEDURAL_FORWARD_CONSTANTS_REGISTER);

// Generate one grid vertex from its index
View3DForwardVertexOut VSMain(uint vertex_id : SV_VertexID)
{
	// Vertices are numbered row by row. Map the index to a position in the XY plane, centred on the origin.
	uint n = (uint)g_wave.grid.x;
	float side = g_wave.grid.y;
	float2 uv = float2(vertex_id % n, vertex_id / n) / (n - 1);
	float2 xy = (uv - 0.5f) * side;

	// The height is the product of two waves travelling along X and Y, which gives a moving pattern of peaks.
	// The normal comes from the exact partial derivatives of the height, so lighting matches the surface.
	float amp = g_wave.wave.x;
	float k = g_wave.wave.y;
	float phase = g_wave.wave.z * g_wave.wave.w;
	float ax = k * xy.x + phase;
	float ay = 0.7f * k * xy.y + 0.8f * phase;
	float z = amp * sin(ax) * cos(ay);
	float dz_dx = amp * k * cos(ax) * cos(ay);
	float dz_dy = -amp * 0.7f * k * sin(ax) * sin(ay);
	float4 ms_norm = float4(normalize(float3(-dz_dx, -dz_dy, 1)), 0);

	// Blend the colour from trough to crest
	float t = amp > 0 ? saturate(0.5f + 0.5f * z / amp) : 0.5f;
	float4 colour = lerp(g_wave.colour_lo, g_wave.colour_hi, t);
	return View3DProceduralForwardVertex(float4(xy, z, 1), ms_norm, colour, uv, float2(0, 0), g_frame, g_nugget);
}
