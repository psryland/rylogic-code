//************************************
// Lost at Sea
//  Copyright (c) Rylogic Ltd 2025
//************************************
// Distant ocean shader.
// Simple flat z=0 ocean patches beyond the near Gerstner ocean.
// VS: grid → world at sea level via m_o2w. PS: Fresnel + distance fog.
#include "view3d-12/src/shaders/hlsl/forward/forward_cbuf.hlsli"
#include "src/world/ocean/shaders/distant_ocean_cbuf.hlsli"

// Environment map cubemap (bound by forward render step when scene env map is set)
TextureCube<float4> m_envmap_texture :reg(t1);
SamplerState        m_envmap_sampler :reg(s1);

struct PSOut
{
	float4 diff :SV_TARGET;
};

// Sample reflection from env map cubemap or fall back to procedural sky
float3 SampleReflectionDistant(float3 dir, float3 sun_dir, float3 sun_col)
{
	if (m_has_env_map)
		return m_envmap_texture.Sample(m_envmap_sampler, dir).rgb;

	// Fallback procedural sky
	float3 horizon = float3(0.75, 0.85, 0.95);
	float3 zenith  = float3(0.25, 0.45, 0.80);
	float t = saturate(dir.z);
	float3 sky = lerp(horizon, zenith, t * t);
	float sun_dot = saturate(dot(dir, sun_dir));
	sky += sun_col * pow(sun_dot, 8.0) * 0.3;
	return sky;
}

// Vertex shader: flat ocean at z=0
PSIn VSDistantOcean(VSIn In)
{
	PSIn Out = (PSIn)0;

	// Transform grid vertex [0,1] to world space via m_o2w
	float4 world_pos = mul(float4(In.vert.xy, 0, 1), m_o2w);
	world_pos.z = 0.0; // Flat ocean at sea level

	Out.ws_vert = world_pos;
	Out.ws_norm = float4(0, 0, 1, 0);
	Out.ss_vert = mul(world_pos, m_cam.m_w2s);
	Out.diff = float4(0, 0, 0, 1);
	Out.tex0 = In.tex0;
	Out.idx0 = In.idx0;

	return Out;
}

// Pixel shader: ocean colour + Fresnel reflection + distance fog
PSOut PSDistantOcean(PSIn In)
{
	PSOut Out = (PSOut)0;

	float3 N = float3(0, 0, 1);
	float3 V = normalize(m_cam.m_c2w[3].xyz - In.ws_vert.xyz);
	float3 L = m_sun_direction.xyz;

	float NdotV = saturate(dot(N, V));
	float NdotL = saturate(dot(N, L));

	// Schlick Fresnel
	float fresnel = 0.02 + 0.98 * pow(saturate(1.0 - NdotV), 5.0);

	// Reflection
	float3 R = reflect(-V, N);
	float3 reflection = SampleReflectionDistant(R, L, m_sun_colour.rgb);

	// Water body colour (depth approximation from view angle)
	float depth_factor = saturate(1.0 - NdotV);
	float3 water = lerp(m_colour_shallow.rgb, m_colour_deep.rgb, depth_factor);

	// Combine reflection and water
	float3 colour = lerp(water, reflection, fresnel);

	// Ambient + diffuse lighting (matches near ocean formula for seamless transition)
	colour *= (0.15 + NdotL * 0.5 + 0.5);

	// Sun specular glint
	float3 H = normalize(L + V);
	float spec = pow(saturate(dot(N, H)), 256.0) * fresnel;
	colour += m_sun_colour.rgb * spec;

	// Distance fog toward horizon colour
	float dist = length(In.ws_vert.xy - m_camera_pos.xy);
	float fog = saturate((dist - m_fog_params.x) / max(m_fog_params.y - m_fog_params.x, 1.0));
	colour = lerp(colour, m_fog_colour.rgb, fog * fog);

	Out.diff = float4(colour, 1.0);
	return Out;
}
