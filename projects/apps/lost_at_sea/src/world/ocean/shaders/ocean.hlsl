//************************************
// Lost at Sea
//  Copyright (c) Rylogic Ltd 2024
//************************************
// Ocean vertex shader.
// Reconstructs world-space vertex positions from encoded ring/segment data,
// applies Gerstner wave displacement, and computes analytical normals.
//
// Vertex encoding:
//   m_vert.xy = unit-circle direction (cos θ, sin θ) for the segment
//   m_vert.z  = normalised ring index t ∈ [0, 1]
//   m_vert.w  = 1
//   Centre vertex: xy = (0, 0), z = -1 (sentinel)
#include "pr/hlsl/interop.hlsli"
#include "view3d-12/src/shaders/hlsl/forward/forward_cbuf.hlsli"
#include "view3d-12/src/shaders/hlsl/forward/kbuffer.hlsli"
#include "src/world/ocean/shaders/ocean_cbuf.hlsli"
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

// Compute ring radius with log-to-linear blend based on camera height.
// Clamps to enforce minimum ring spacing, capping point density near the camera.
float RingRadius(float t, float camera_height, float inner, float outer)
{
	float num_rings = g_ocean.mesh_config.z;
	float min_spacing = g_ocean.mesh_config.w;

	// Logarithmic: r = inner * exp(log(outer/inner) * t)
	float log_ratio = log(outer / inner);
	float r_log = inner * exp(log_ratio * t);

	// Linear: r = inner + (outer - inner) * t
	float r_lin = inner + (outer - inner) * t;

	// Blend: 0 = fully logarithmic (near surface), 1 = fully linear (high altitude)
	float h = abs(camera_height);
	float blend = saturate(h / outer);

	float r = lerp(r_log, r_lin, blend);

	// Enforce minimum ring spacing to cap point density near camera
	float ring_idx = t * (num_rings - 1);
	float r_min = inner + min_spacing * ring_idx;
	return max(r, r_min);
}

// Apply Gerstner wave displacement to a world-space position
// Returns: xyz = displaced position, w = foam factor (Jacobian > 1 indicates foam)
float4 GerstnerDisplace(float2 world_xy, float time)
{
	float dx = 0, dy = 0, dz = 0;
	float jacobian = 1.0;

	for (int i = 0; i < g_ocean.wave_count; ++i)
	{
		float2 dir = g_ocean.wave_dirs[i].xy;
		float amplitude  = g_ocean.wave_params[i].x;
		float wavelength = g_ocean.wave_params[i].y;
		float speed      = g_ocean.wave_params[i].z;
		float steepness  = g_ocean.wave_params[i].w;

		float k = 6.283185307 / wavelength; // tau / wavelength
		float freq = k * speed;
		float phase = k * dot(dir, world_xy) - freq * time;
		float c = cos(phase);
		float s = sin(phase);

		dx -= steepness * amplitude * dir.x * c;
		dy -= steepness * amplitude * dir.y * c;
		dz += amplitude * s;

		// Jacobian term for foam detection
		jacobian -= steepness * k * amplitude * s;
	}

	return float4(dx, dy, dz, saturate(1.0 - jacobian));
}

// Compute analytical Gerstner wave normal
float3 GerstnerNormal(float2 world_xy, float time)
{
	float nx = 0, ny = 0, nz = 1;

	for (int i = 0; i < g_ocean.wave_count; ++i)
	{
		float2 dir = g_ocean.wave_dirs[i].xy;
		float amplitude  = g_ocean.wave_params[i].x;
		float wavelength = g_ocean.wave_params[i].y;
		float speed      = g_ocean.wave_params[i].z;
		float steepness  = g_ocean.wave_params[i].w;

		float k = 6.283185307 / wavelength;
		float freq = k * speed;
		float phase = k * dot(dir, world_xy) - freq * time;
		float c = cos(phase);
		float s = sin(phase);

		nx -= dir.x * k * amplitude * c;
		ny -= dir.y * k * amplitude * c;
		nz -= steepness * k * amplitude * s;
	}

	return normalize(float3(nx, ny, nz));
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

// Displace verts of the input mesh by the waves
PSIn VSOcean(VSIn In)
{
	PSIn Out = (PSIn)0;

	float inner = g_ocean.mesh_config.x;
	float outer = g_ocean.mesh_config.y;
	float cam_height = g_ocean.camera_pos_time.z;
	float time = g_ocean.camera_pos_time.w;
	float2 cam_xy = g_ocean.camera_pos_time.xy;

	float3 ws_pos;

	// Centre vertex sentinel: z == -1
	if (In.vert.z < 0)
	{
		// Centre vertex is at the camera XY position, displaced by waves
		float4 disp = GerstnerDisplace(cam_xy, time);
		ws_pos = float3(cam_xy.x + disp.x, cam_xy.y + disp.y, disp.z);

		float3 n = GerstnerNormal(cam_xy, time);
		Out.ws_norm = float4(n, 0);
		Out.diff = float4(0, 0, 0, disp.w); // foam factor in alpha
	}
	else
	{
		// Ring vertex: decode direction and ring parameter
		float2 dir = In.vert.xy;    // unit-circle direction
		float t = In.vert.z;        // normalised ring index [0, 1]

		float r = RingRadius(t, cam_height, inner, outer);
		float2 local_xy = r * dir;
		float2 world_xy = cam_xy + local_xy;

		// Fade wave displacement to zero near the outer edge so the near ocean
		// smoothly flattens to z=0, matching the distant ocean patches.
		float fade_start = outer * 0.7; // Begin fading at 70% of outer radius
		float wave_fade = 1.0 - saturate((r - fade_start) / (outer - fade_start));

		// Apply Gerstner displacement, scaled by fade factor
		float4 disp = GerstnerDisplace(world_xy, time);
		ws_pos = float3(
			world_xy.x + disp.x * wave_fade,
			world_xy.y + disp.y * wave_fade,
			disp.z * wave_fade
		);

		float3 n_waves = GerstnerNormal(world_xy, time);
		float3 n_flat = float3(0, 0, 1);
		float3 n = normalize(lerp(n_flat, n_waves, wave_fade));
		Out.ws_norm = float4(n, 0);
		Out.diff = float4(0, 0, 0, disp.w * wave_fade); // foam also fades
	}

	// Camera-relative rendering: subtract camera XY to keep geometry near the origin.
	// The camera Z offset is handled by the view matrix (w2c) since
	// the camera is at (0, 0, cam_height) in render space.
	float3 cam_rel = ws_pos - float3(cam_xy, 0);
	float4 os_vert = float4(cam_rel, 1);

	// Transform using standard forward matrices
	Out.ws_vert = mul(os_vert, g_nugget.o2w);
	Out.ss_vert = mul(os_vert, g_nugget.o2s);

	// Pass through texture coordinates (unused for now, but available for detail maps)
	Out.tex0 = In.tex0;
	Out.idx0 = In.idx0;

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
