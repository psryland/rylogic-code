//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
// Shader for forward rendering
#include "view3d-12/src/shaders/hlsl/types.hlsli"
#include "view3d-12/src/shaders/hlsl/forward/forward_cbuf.hlsli"

// Texture2D /w sampler
Texture2D<float4> m_texture0 :reg(t0);
SamplerState      m_sampler0 :reg(s0);

// Environment map
TextureCube<float4> m_envmap_texture :reg(t1);
SamplerState        m_envmap_sampler :reg(s1);

// Shadow map
Texture2D<float2> m_smap_texture[MaxShadowMaps] :reg(t2);
SamplerComparisonState m_smap_sampler           :reg(s2);

// Projected textures
Texture2D<float4> m_proj_texture[MaxProjectedTextures] :reg(t3);
SamplerState      m_proj_sampler[MaxProjectedTextures] :reg(s3);

// Skinned Meshes
StructuredBuffer<Mat4x4> m_pose : reg(t4);
StructuredBuffer<Skinfluence> m_skin : reg(t5);

#include "view3d-12/src/shaders/hlsl/lighting/phong_lighting.hlsli"
#include "view3d-12/src/shaders/hlsl/shadow/shadow_cast.hlsli"
#include "view3d-12/src/shaders/hlsl/skinned/skinned.hlsli"
#include "view3d-12/src/shaders/hlsl/utility/env_map.hlsli"

// PS output format
struct PSOut
{
	float4 diff :SV_TARGET;
};

// Default VS
PSIn VSDefault(VSIn In)
{
	PSIn Out = (PSIn)0;

	// Transform
	float4 os_vert = mul(In.vert, m_m2o);
	float4 os_norm = mul(In.norm, m_m2o);
	
	if (IsSkinned(m_flags))
	{
		os_vert = SkinVertex(m_pose, m_skin[In.idx0.x], os_vert);
		os_norm = SkinNormal(m_pose, m_skin[In.idx0.x], os_norm);
	}

	Out.ws_vert = mul(os_vert, m_o2w);
	Out.ws_norm = mul(os_norm, m_n2w);
	Out.ss_vert = mul(os_vert, m_o2s);

	// Tinting
	Out.diff = m_tint;

	// Per Vertex colour
	Out.diff = In.diff * Out.diff;

	// Texture2D (with transform)
	Out.tex0 = mul(float4(In.tex0, 0, 1), m_tex2surf0).xy;

	// Copy the source instance index
	Out.idx0 = In.idx0;
	
	return Out;
}

// Default PS
PSOut PSDefault(PSIn In)
{
	// Notes:
	//  - 'ss_vert:SV_Position' in the pixel shader already has x,y,z divided by w (w unchanged)
	//  - Models without normals can still use 'HasNormals' as true. For this case, and normal to
	//    the light source is used.

	PSOut Out = (PSOut)0;

	// Tinting
	Out.diff = In.diff;

	// Transform
	if (HasNormals(m_flags))
	{
		// If the normal is (0,0,0), use a vector to the light source
		In.ws_norm =
			dot(In.ws_norm, In.ws_norm) != 0 ? normalize(In.ws_norm) :
			DirectionalLight(m_global_light) ? -m_global_light.m_ws_direction :
			PointLight(m_global_light)       ? normalize(m_global_light.m_ws_position - In.ws_vert) :
			SpotLight(m_global_light)        ? normalize(m_global_light.m_ws_position - In.ws_vert) :
			float4(0,0,0,0);
	}

	// Texture2D (with transform)
	if (HasTex0(m_flags))
	{
		if (EnvMapProj(m_flags))
		{
			float3 dir = mul(In.ws_vert, m_tex2surf0).xyz;
			Out.diff = m_envmap_texture.Sample(m_envmap_sampler, dir);
		}
		else
		{
			float4 texel = m_texture0.Sample(m_sampler0, In.tex0);
			Out.diff = texel * Out.diff;
		}
	}

	// Env Map
	if (HasEnvMap(m_flags) && HasNormals(m_flags))
		Out.diff = EnvironmentMap(m_env_map, In.ws_vert, In.ws_norm, m_cam.m_c2w[3], Out.diff);

	// Shadows
	float light_visible = 1.0f;
	if (ShadowMapCount(m_shadow) != 0)
		light_visible = LightVisibility(m_shadow, In.ws_vert);

	// Lighting
	if (HasNormals(m_flags))
		Out.diff = Illuminate(m_global_light, In.ws_vert, In.ws_norm, m_cam.m_c2w[3], light_visible, Out.diff);

	// If not alpha blending, clip alpha pixels
	if (!HasAlpha(m_flags))
		clip(Out.diff.a - 0.5);

	return Out;
}

PSOut PSRadialFade(PSIn In)
{
	PSOut Out = PSDefault(In);

	// Fade pixels radially from 'centre'
	float4 centre = any(m_fade_centre) ? m_fade_centre : m_cam.m_c2w[3];
	float4 radial = In.ws_vert - centre;
	float radius =
		m_fade_type == 0 ? length(radial) : // Spherical
		m_fade_type == 1 ? length(radial - dot(radial, m_cam.m_c2w[1]) * m_cam.m_c2w[1]) : // Cylindrical
		0;

	// Lerp to alpha = 0 based on distance
	float frac = smoothstep(m_fade_radius[0], m_fade_radius[1], radius);
	Out.diff.a = lerp(Out.diff.a, 0, frac);
	return Out;
}

// Main vertex shader
#ifdef PR_RDR_VSHADER_forward
PSIn main(VSIn In)
{
	return VSDefault(In);
}
#endif

// Main pixel shader
#ifdef PR_RDR_PSHADER_forward
PSOut main(PSIn In)
{
	return PSDefault(In);
}
#endif

// Main pixel shader
#ifdef PR_RDR_PSHADER_forward_radial_fade
PSOut main(PSIn In)
{
	return PSRadialFade(In);
}
#endif