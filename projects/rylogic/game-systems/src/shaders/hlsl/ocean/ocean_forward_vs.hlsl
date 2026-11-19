//************************************
// Ocean Surface Forward Vertex
//  Copyright (c) Rylogic Ltd 2026
//************************************
// Generates the world-view water surface from vertex IDs alone.
// The vertex IDs address a square lattice that is mapped to a disk around the camera. Vertex spacing grows with distance from the camera,
// so the triangle density is highest where the waves are seen closest. The disk reaches the far side of the domain, and vertices beyond the
// domain wall are pulled back onto it. Wave maths runs only for wet vertices within the wave radius of the camera whose triangles may be in view;
// every other vertex is flat.
// Vertex IDs after the lattice form an opaque curtain from the surface's edge down to the domain floor, so the water closes against the wall.
#include "pr/view3d-12/shaders/procedural_vertex.hlsli"
#include "pr/game-systems/ocean/ocean_surface.hlsli"

ConstantBuffer<View3DForwardFrame> g_frame : register(VIEW3D_FORWARD_FRAME_REGISTER);
ConstantBuffer<View3DElementIndex> g_element : register(VIEW3D_FORWARD_ELEMENT_INDEX_REGISTER);
StructuredBuffer<View3DForwardElement> g_elements : register(VIEW3D_FORWARD_ELEMENTS_REGISTER);
static const View3DForwardElement g_nugget = g_elements[g_element.index];
ConstantBuffer<OceanConstants> g_ocean : register(VIEW3D_PROCEDURAL_FORWARD_CONSTANTS_REGISTER);

// Wind-sea wave elements, bathymetry heights, the wind-sea table and winds, and the foam history. See OceanConstants for the layout.
ByteAddressBuffer g_ocean_buffer : register(VIEW3D_PROCEDURAL_BUFFER0_REGISTER);
#include "pr/game-systems/ocean/ocean_buffer.hlsli"

// Map a square point to a disk while keeping concentric squares as concentric circles, so the lattice stays connected without a polar singularity.
float2 ConcentricSquareToDisk(float2 square_pos)
{
	// The centre maps to itself; elsewhere the larger coordinate sets the radius and the smaller sets the angle within its octant.
	if (all(abs(square_pos) < 1.0e-6f))
		return float2(0.0f, 0.0f);

	float radius;
	float angle;
	if (abs(square_pos.x) > abs(square_pos.y))
	{
		radius = square_pos.x;
		angle = 0.7853981633974483f * square_pos.y / square_pos.x;
	}
	else
	{
		radius = square_pos.y;
		angle = 1.5707963267948966f - 0.7853981633974483f * square_pos.x / square_pos.y;
	}

	float sine;
	float cosine;
	sincos(angle, sine, cosine);
	return radius * float2(cosine, sine);
}

// Return true when no point of the model-space box centred on 'centre', with half sizes 'half_size' along the model axes, can be seen in the view.
// The test is exact for the box itself; it never reports a visible box as hidden.
bool OceanBoxHidden(float3 centre, float3 half_size)
{
	// Transform the centre point and the three half-size axes to clip space. The box is an affine image of a cube, so its clip-space corners
	// are the centre plus or minus each transformed axis.
	float4 c = mul(View3DProceduralWorldPosition(float4(centre, 1.0f), g_nugget.m2o, g_nugget.o2w), g_frame.cam.w2s);
	float4 ax = mul(View3DProceduralWorldPosition(float4(half_size.x, 0.0f, 0.0f, 0.0f), g_nugget.m2o, g_nugget.o2w), g_frame.cam.w2s);
	float4 ay = mul(View3DProceduralWorldPosition(float4(0.0f, half_size.y, 0.0f, 0.0f), g_nugget.m2o, g_nugget.o2w), g_frame.cam.w2s);
	float4 az = mul(View3DProceduralWorldPosition(float4(0.0f, 0.0f, half_size.z, 0.0f), g_nugget.m2o, g_nugget.o2w), g_frame.cam.w2s);

	// A visible point has -w <= x <= w, -w <= y <= w, and 0 <= z <= w in clip space. Each clip plane value is linear in the clip position, so
	// over the whole box it is at most the centre's value plus the sizes of the axes' values. The box is hidden if any plane rejects all of it.
	// This holds whatever the depth direction, and for boxes behind the camera.
	float3 c0 = float3(c.w + c.x, c.w - c.x, c.w + c.y);
	float3 c1 = float3(c.w - c.y, c.z, c.w - c.z);
	float3 r0 = abs(float3(ax.w + ax.x, ax.w - ax.x, ax.w + ax.y)) + abs(float3(ay.w + ay.x, ay.w - ay.x, ay.w + ay.y)) + abs(float3(az.w + az.x, az.w - az.x, az.w + az.y));
	float3 r1 = abs(float3(ax.w - ax.y, ax.z, ax.w - ax.z)) + abs(float3(ay.w - ay.y, ay.z, ay.w - ay.z)) + abs(float3(az.w - az.y, az.z, az.w - az.z));
	return any(c0 + r0 < 0.0f) || any(c1 + r1 < 0.0f);
}

// Generate one vertex of the opaque curtain that closes the water against the domain wall. 'curtain_id' counts from the first curtain vertex.
// Even IDs are on the wave surface and odd IDs are on the domain floor. Segments crowd towards the wall nearest the camera, where they are seen largest.
View3DForwardVertexOut CurtainVertex(uint curtain_id, float central_extent, float warp_power, float half_cells)
{
	// Place the segment on the wall. 'u' runs over [-1, 1) around the wall, starting and ending opposite the camera.
	float3 camera = g_ocean.camera_time.xyz;
	float domain_radius = g_ocean.level_radii.y;
	float u = 2.0f * float(curtain_id >> 1) / float(OceanCurtainSegments) - 1.0f;
	float nearest_angle = any(abs(camera.xy) > 1.0e-6f) ? atan2(camera.y, camera.x) : 0.0f;
	float sine;
	float cosine;
	sincos(nearest_angle + 3.141592653589793f * u * abs(u), sine, cosine);
	float2 wall_dir = float2(cosine, sine);
	float2 world_xy = domain_radius * wall_dir;

	// The top follows the waves exactly as the surface's edge does, while the bottom rests on the domain floor.
	float distance = length(world_xy - camera.xy);
	float spacing = OceanSpacingAtDistance(distance, central_extent, g_ocean.level_radii.z, warp_power, half_cells);
	OceanWaveSample water = OceanSample(world_xy, distance, spacing);
	float top = g_ocean.level_radii.x + water.surface.displacement_foam.z;
	float height = (curtain_id & 1) == 0 ? top : min(g_ocean.mesh.w, top);

	// The curtain faces into the domain and has the water's colour without its transparency.
	float4 position = float4(world_xy, height, 1.0f);
	float4 colour = float4(g_ocean.colour.rgb, 1.0f);
	return View3DProceduralForwardVertex(position, float4(-wall_dir, 0.0f, 0.0f), colour, float2(0, 0), float2(0, 0), g_frame, g_nugget);
}

// Generate one water surface or curtain vertex for the water pixel shaders (see ocean_forward_ps.hlsli). Surface vertices carry their crest steepness in tex0.x,
// their crest depth fade in tex0.y, and their lattice spacing (m) in idx0.y (see OceanWaveSample).
View3DForwardVertexOut Main(uint vertex_id : SV_VertexID)
{
	// The lattice mapping is shared by the surface and the curtain, so both find the same waves at the wall.
	float3 camera = g_ocean.camera_time.xyz;
	float level = g_ocean.level_radii.x;
	float domain_radius = g_ocean.level_radii.y;
	float outer_radius = g_ocean.level_radii.z;
	uint side = g_ocean.layout.x;
	float half_cells = 0.5f * float(side - 1);
	float central_extent = OceanCentralExtent(g_ocean.mesh.x, half_cells, outer_radius);
	float warp_power = OceanWarpPower(g_ocean.mesh.y, camera.z - level, outer_radius);
	if (vertex_id >= side * side)
		return CurtainVertex(vertex_id - side * side, central_extent, warp_power, half_cells);

	// Decode the lattice position and map it to the unit disk.
	uint grid_y = vertex_id / side;
	uint grid_x = vertex_id - grid_y * side;
	float2 disk_pos = ConcentricSquareToDisk((float2(grid_x, grid_y) - half_cells) / half_cells);
	float unit_radius = length(disk_pos);

	// Warp the radius so rings start at the minimum spacing near the camera and spread out towards the outer radius.
	float radius = OceanRingRadius(unit_radius, central_extent, outer_radius, warp_power);
	float2 world_xy = camera.xy + (unit_radius > 1.0e-6f ? disk_pos * (radius / unit_radius) : float2(0.0f, 0.0f));
	float spacing = OceanRingSpacing(unit_radius, central_extent, outer_radius, warp_power, half_cells);

	// Bound the distance to every lattice neighbour, before any collapse onto the wall. Neighbours are at most one lattice step further out,
	// where the ring spacing is 's'. Each is within 's' radially and about 1.6 's' around the ring, so three times 's' covers both with room
	// to spare. Collapsing onto the wall never moves two vertices further apart, so the bound still holds after it.
	float neighbour_reach = 3.0f * OceanRingSpacing(unit_radius + 1.0f / half_cells, central_extent, outer_radius, warp_power, half_cells);

	// Keep the surface inside the domain wall. Vertices beyond it collapse onto the wall, where their triangles have no area. They take the
	// spacing and distance of their wall position, so they match the top of the curtain there.
	float distance = radius;
	float centre_distance = length(world_xy);
	if (centre_distance > domain_radius)
	{
		// Measure the collapsed position from the camera as the curtain does.
		world_xy *= domain_radius / centre_distance;
		distance = length(world_xy - camera.xy);
		spacing = OceanSpacingAtDistance(distance, central_extent, outer_radius, warp_power, half_cells);
	}

	// Skip the wave maths for vertices whose triangles are all out of view. The box holds the vertex and its neighbours at every height the
	// waves can reach, so it holds every triangle that uses this vertex. A hidden vertex stays flat, which changes nothing that is drawn.
	if (OceanBoxHidden(float3(world_xy, level), float3(neighbour_reach, neighbour_reach, g_ocean.waves.y)))
		return View3DProceduralForwardVertex(float4(world_xy, level, 1.0f), float4(0.0f, 0.0f, 1.0f, 0.0f), g_ocean.colour, float2(0, 0), float2(0, 0), g_frame, g_nugget);

	// Raise the vertex to the physics wave height; it never moves sideways, so the drawn height at each position is the height bodies feel.
	OceanWaveSample water = OceanSample(world_xy, distance, spacing);
	float4 position = float4(world_xy, level + water.surface.displacement_foam.z, 1.0f);
	float3 normal = normalize(float3(water.surface.normal_delta.xy, 1.0f));

	// Water reflects more at grazing angles, so the surface becomes more opaque towards the horizon.
	float3 to_camera = normalize(camera - position.xyz);
	float fresnel = pow(1.0f - saturate(abs(dot(to_camera, normal))), 5.0f);
	float4 colour = g_ocean.colour;
	colour.a = lerp(colour.a, 1.0f, fresnel);
	return View3DProceduralForwardVertex(position, float4(normal, 0.0f), colour, float2(water.crest_steepness, water.crest_depth_fade), float2(0.0f, spacing), g_frame, g_nugget);
}