//************************************
// Lost at Sea
//  Copyright (c) Rylogic Ltd 2024
//************************************
// Ocean vertex shader.
// Reconstructs world-space vertex positions from the indexed draw's vertex IDs,
// applies Gerstner wave displacement, and computes analytical normals.
//
// Vertex IDs identify a regular row-major Cartesian lattice. The shader maps the square lattice
// to a disk before applying distance-dependent spacing, avoiding the polar centre singularity.
#include "pr/hlsl/interop.hlsli"
#include "view3d-12/src/shaders/hlsl/forward/forward_cbuf.hlsli"
#include "view3d-12/src/shaders/hlsl/forward/kbuffer.hlsli"
#include "src/world/ocean/shaders/ocean_cbuf.hlsli"
#include "src/world/water/shaders/water_field.hlsli"
#include "pr/hlsl/camera.hlsli"

#ifdef __cplusplus
namespace las
{
	using namespace pr::hlsl;
#endif

ConstantBuffer<CBufFrame> resource(g_frame, b0);
ConstantBuffer<CBufNugget> resource(g_nugget, b1);
ConstantBuffer<CBufOcean> resource(g_ocean, b3);

// Environment map cubemap (bound by forward render step when scene env map is set)
TextureCube<float4> resource(m_envmap_texture, t1);
SamplerState resource(m_envmap_sampler, s1);
Texture2DMS<float> resource(g_opaque_depth, t6);
RasterizerOrderedTexture2D<uint4> resource(g_alpha_colour, u0);
RasterizerOrderedTexture2D<uint4> resource(g_alpha_depth, u1);

struct PSOut
{
	float4 diff semantic(SV_TARGET);
};

// Map a square point to a disk while preserving concentric square boundaries. This keeps the regular grid connected and gives the centre vertex bounded valence.
float2 ConcentricSquareToDisk(float2 square_pos)
{
	if (all(abs(square_pos) < 1.0e-6))
		return float2(0.0, 0.0);

	float radius;
	float angle;
	if (abs(square_pos.x) > abs(square_pos.y))
	{
		radius = square_pos.x;
		angle = 0.7853981633974483 * square_pos.y / square_pos.x;
	}
	else
	{
		radius = square_pos.y;
		angle = 1.5707963267948966 - 0.7853981633974483 * square_pos.x / square_pos.y;
	}

	float sine;
	float cosine;
	sincos(angle, sine, cosine);
	return radius * float2(cosine, sine);
}

// Expand disk radius from the configured minimum central cell size to the outer ocean radius. At altitude the power tends toward one for more uniform screen coverage.
float WarpedGridRadius(float unit_radius, float camera_height)
{
	float outer_radius = g_ocean.mesh_config.x;
	float half_cell_count = 0.5 * (g_ocean.mesh_config.y - 1.0);
	float central_extent = min(g_ocean.mesh_config.z * half_cell_count, outer_radius);
	float altitude_blend = saturate(abs(camera_height) / outer_radius);
	float warp_power = lerp(g_ocean.mesh_config.w, 1.0, altitude_blend);
	return central_extent * unit_radius + (outer_radius - central_extent) * pow(unit_radius, warp_power);
}

// Evaluate all active elements at a world-space position.
WaterFieldSample EvaluateWaterField(float2 world_xy, float time)
{
	WaterFieldSample sample = WaterFieldSampleZero();
	for (int i = 0; i != g_ocean.water_field_count; ++i)
	{
		AccumulateWaterFieldElement(g_ocean.water_field[i], world_xy, time, sample);
	}
	sample.displacement_foam.w = saturate(sample.displacement_foam.w);
	return sample;
}

// Procedural sky colour (fallback when no cubemap is available)
float3 ProceduralSky(float3 dir)
{
	float3 horizon = float3(0.75, 0.85, 0.95);
	float3 zenith  = float3(0.25, 0.45, 0.80);
	float t = saturate(dir.z);
	float3 sky = lerp(horizon, zenith, t * t);

	float sun_dot = saturate(dot(dir, g_ocean.sun_direction.xyz));
	float sun_disc = pow(sun_dot, 512.0);
	float sun_glow = pow(sun_dot, 8.0) * 0.3;
	sky += g_ocean.sun_colour.rgb * (sun_disc + sun_glow);

	return sky;
}

// Sample reflection from env map cubemap or fall back to procedural sky
float3 SampleReflection(float3 dir)
{
	if (g_ocean.has_env_map)
		return m_envmap_texture.Sample(m_envmap_sampler, dir).rgb;
	return ProceduralSky(dir);
}

// Schlick's Fresnel approximation
float FresnelSchlick(float cos_theta, float f0)
{
	return f0 + (1.0 - f0) * pow(saturate(1.0 - cos_theta), 5.0);
}

// Generate and displace one indexed warped-grid vertex.
PSIn VSOcean(uint vertex_id semantic(SV_VertexID))
{
	PSIn Out = (PSIn)0;

	float outer = g_ocean.mesh_config.x;
	float cam_height = g_ocean.camera_pos_time.z;
	float time = g_ocean.camera_pos_time.w;
	float2 cam_xy = g_ocean.camera_pos_time.xy;

	// View3D draws with BaseVertexLocation zero, so an indexed draw reports the model-relative index as SV_VertexID.
	uint grid_vertex_count = uint(g_ocean.mesh_config.y);
	uint grid_y = vertex_id / grid_vertex_count;
	uint grid_x = vertex_id - grid_y * grid_vertex_count;
	float half_cell_count = 0.5 * float(grid_vertex_count - 1);
	float2 square_pos = (float2(grid_x, grid_y) - half_cell_count) / half_cell_count;
	float2 disk_pos = ConcentricSquareToDisk(square_pos);
	float unit_radius = length(disk_pos);
	float radius = WarpedGridRadius(unit_radius, cam_height);
	float2 local_xy = unit_radius > 1.0e-6 ? disk_pos * (radius / unit_radius) : float2(0.0, 0.0);
	float2 world_xy = cam_xy + local_xy;

	// Fade wave displacement to zero near the outer edge so the near ocean smoothly flattens to z=0, matching the distant ocean patches.
	float fade_start = outer * 0.7;
	float wave_fade = 1.0 - saturate((radius - fade_start) / (outer - fade_start));

	// Apply Gerstner displacement and analytical normals, scaled by the shared outer-edge fade.
	WaterFieldSample water = EvaluateWaterField(world_xy, time);
	float4 disp = water.displacement_foam;
	float3 ws_pos = float3(
		world_xy.x + disp.x * wave_fade,
		world_xy.y + disp.y * wave_fade,
		disp.z * wave_fade
	);

	float3 n_waves = normalize(float3(0, 0, 1) + water.normal_delta.xyz);
	float3 n = normalize(lerp(float3(0, 0, 1), n_waves, wave_fade));
	Out.ws_norm = float4(n, 0);
	Out.diff = float4(0, 0, 0, disp.w * wave_fade);

	// Camera-relative rendering: subtract camera XY to keep geometry near the origin.
	// The camera Z offset is handled by the view matrix (w2c) since
	// the camera is at (0, 0, cam_height) in render space.
	float3 cam_rel = ws_pos - float3(cam_xy, 0);
	float4 os_vert = float4(cam_rel, 1);

	// Transform using standard forward matrices
	Out.ws_vert = mul(os_vert, g_nugget.o2w);
	Out.ss_vert = mul(os_vert, g_nugget.o2s);

	return Out;

}

// Calculate the near-ocean colour for the alpha collect pass.
float4 OceanColour(PSIn In)
{
	// Surface normal and view direction
	float3 N = normalize(In.ws_norm.xyz);
	float3 V = normalize(g_frame.cam.c2w[3].xyz - In.ws_vert.xyz);
	float NdotV = saturate(dot(N, V));
	float foam_factor = In.diff.a;

	// --- Fresnel reflectance ---
	float fresnel = FresnelSchlick(NdotV, g_ocean.fresnel_f0);

	// --- Reflection ---
	float3 R = reflect(-V, N);
	float3 reflection = SampleReflection(R);

	// --- Refraction / water body colour ---
	// Approximate depth: steeper viewing angles see deeper water
	float depth_factor = saturate(1.0 - NdotV);
	float3 refraction = lerp(g_ocean.colour_shallow.rgb, g_ocean.colour_deep.rgb, depth_factor);

	// --- Subsurface scattering ---
	// Light passing through thin wave crests when backlit by the sun
	float3 L = g_ocean.sun_direction.xyz;
	float sss_dot = saturate(dot(V, -L));
	float wave_height = saturate(In.ws_vert.z * 0.5 + 0.5); // Normalise wave height around 0
	float sss = pow(sss_dot, 4.0) * wave_height * g_ocean.sss_strength;
	float3 sss_colour = g_ocean.colour_shallow.rgb * 1.5; // Brighter turquoise for SSS

	// --- Specular (sun glint) ---
	float3 H = normalize(L + V);
	float NdotH = saturate(dot(N, H));
	float specular = pow(NdotH, g_ocean.specular_power) * fresnel;
	float3 spec_colour = g_ocean.sun_colour.rgb * specular;

	// --- Combine ---
	float3 colour = lerp(refraction, reflection, fresnel);
	colour += sss * sss_colour;
	colour += spec_colour;

	// --- Foam ---
	// Foam appears at wave crests where the Gerstner Jacobian drops below 1
	colour = lerp(colour, g_ocean.colour_foam.rgb, foam_factor * 0.8);

	// --- Basic ambient from scene lighting ---
	float ambient = 0.15;
	float diffuse_light = saturate(dot(N, L)) * 0.5;
	colour *= (ambient + diffuse_light + 0.5); // Brighten overall to avoid too-dark water

	// --- Transparency ---
	// Fresnel reflection is always visible; water absorption increases with viewing angle.
	// Looking down: mostly transparent (see terrain below). At horizon: opaque/reflective.
	float alpha = fresnel + (1.0 - fresnel) * (1.0 - g_ocean.water_transparency * NdotV);

	return float4(colour, alpha);
}

// Pixel shader for K-buffer alpha collection.
void PSOcean(PSIn In)
{
	float4 diff = OceanColour(In);
	clip(diff.a - (1.0f / 255.0f));

	uint2 pix = uint2(In.ss_vert.xy);
	uint width, height, sample_count;
	g_opaque_depth.GetDimensions(width, height, sample_count);

	float opaque_depth = 1.0f;
	for (uint sample = 0; sample != sample_count; ++sample)
		opaque_depth = min(opaque_depth, g_opaque_depth.Load(pix, sample));
	if (In.ss_vert.z >= opaque_depth)
		discard;

	float view_z = -mul(In.ws_vert, g_frame.cam.w2c).z;
	uint depth = PackDepthKey(view_z, ClipPlanes(g_frame.cam.c2s), uint(g_nugget.flags.w));
	uint colour = PackRGBA8(diff);

	uint4 alpha_colour = g_alpha_colour[pix];
	uint4 alpha_depth = g_alpha_depth[pix];
	InsertKBufferLayer(alpha_colour, alpha_depth, colour, depth);
	g_alpha_colour[pix] = alpha_colour;
	g_alpha_depth[pix] = alpha_depth;
}

#ifdef __cplusplus
}
#endif
