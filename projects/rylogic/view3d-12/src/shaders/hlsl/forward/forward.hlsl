//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
// Shader for forward rendering
#include "view3d-12/src/shaders/hlsl/types.hlsli"
#include "view3d-12/src/shaders/hlsl/forward/forward_cbuf.hlsli"

static const int AlphaModeOpaque = 0;
static const int AlphaModeMask = 1;
static const int AlphaModeBlend = 2;

// Constant buffers
ConstantBuffer<CBufFrame> g_frame : register(b0);
ConstantBuffer<CBufNugget> g_nugget : register(b1);
ConstantBuffer<CBufFade> g_fade: register(b2);
ConstantBuffer<CBufPbrSurface> g_pbr : register(b4);

// Texture2D /w sampler
Texture2D<float4> g_texture0 :register(t0);
SamplerState      g_sampler0 :register(s0);

// Environment map
TextureCube<float4> g_envmap_texture :register(t1);
SamplerState        g_envmap_sampler :register(s1);

// Shadow map
Texture2D<float2> g_smap_texture[MaxShadowMaps] :register(t2);
SamplerComparisonState g_smap_sampler           :register(s2);

// Projected textures
Texture2D<float4> g_proj_texture[MaxProjectedTextures] :register(t3);
SamplerState      g_proj_sampler[MaxProjectedTextures] :register(s3);

// Opaque depth
Texture2DMS<float> g_opaque_depth : register(t6);

// Alpha sorting
RasterizerOrderedTexture2D<uint4> g_alpha_colour :register(u0);
RasterizerOrderedTexture2D<uint4> g_alpha_depth  :register(u1);
RasterizerOrderedTexture2D<uint4> g_alpha_rt_attrs :register(u2);

#include "view3d-12/src/shaders/hlsl/forward/kbuffer.hlsli"
#include "view3d-12/src/shaders/hlsl/lighting/phong_lighting.hlsli"
#include "view3d-12/src/shaders/hlsl/lighting/pbr.hlsli"
#include "view3d-12/src/shaders/hlsl/shadow/shadow_cast.hlsli"
#include "view3d-12/src/shaders/hlsl/ray_tracing/ray_tracing.hlsli"
#include "view3d-12/src/shaders/hlsl/utility/colour_space.hlsli"
#include "view3d-12/src/shaders/hlsl/utility/env_map.hlsli"
#include "pr/hlsl/camera.hlsli"

// PS output format
struct PSOut
{
	float4 diff :SV_TARGET;
};
struct PSReflectionOut
{
	float4 diff :SV_TARGET0;
	float4 reflection_attrs :SV_TARGET1;
};

// Return the world-space surface normal used by the forward material path.
float4 ResolveWorldNormal(PSIn In, bool is_front_face)
{
	if (!HasNormals(g_nugget.flags))
		return float4(0, 0, 0, 0);

	float4 norm =
		dot(In.ws_norm, In.ws_norm) != 0 ? normalize(In.ws_norm) :
		DirectionalLight(g_frame.global_light) ? -g_frame.global_light.ws_direction :
		PointLight(g_frame.global_light)       ? normalize(g_frame.global_light.ws_position - In.ws_vert) :
		SpotLight(g_frame.global_light)        ? normalize(g_frame.global_light.ws_position - In.ws_vert) :
		float4(0, 0, 0, 0);

	if (TwoSided(g_nugget.flags) && !is_front_face)
		norm = -norm;

	return norm;
}

// Forward VS
PSIn VSForward(VSIn In)
{
	PSIn Out = (PSIn)0;

	// Transform
	float4 os_vert = mul(In.vert, g_nugget.m2o);
	float4 os_norm = mul(In.norm, g_nugget.m2o);
	
	Out.ws_vert = mul(os_vert, g_nugget.o2w);
	Out.ws_norm = mul(os_norm, g_nugget.n2w);
	Out.ss_vert = mul(os_vert, g_nugget.o2s);

	// Tinting
	Out.diff = g_nugget.tint;

	// Per Vertex colour
	Out.diff = In.diff * Out.diff;

	// Texture2D (with transform)
	Out.tex0 = mul(float4(In.tex0, 0, 1), g_nugget.tex2surf0).xy;

	// Copy the source instance index
	Out.idx0 = In.idx0;
	
	return Out;
}

// Forward PS
PSOut PSForward(PSIn In, bool is_front_face : SV_IsFrontFace)
{
	// Notes:
	//  - 'ss_vert:SV_Position' in the pixel shader already has x,y,z divided by w (w unchanged)
	//  - Models without normals can still use 'HasNormals' as true. For this case, and normal to
	//    the light source is used.

	PSOut Out = (PSOut)0;

	// Tinting
	Out.diff = In.diff;

	// Transform
	if (HasNormals(g_nugget.flags))
	{
		// If the normal is (0,0,0), use a vector to the light source
		In.ws_norm = ResolveWorldNormal(In, is_front_face);
	}

	// Texture2D (with transform)
	if (HasTex0(g_nugget.flags))
	{
		if (EnvMapProj(g_nugget.flags))
		{
			float3 dir = mul(In.ws_vert, g_nugget.tex2surf0).xyz;
			Out.diff = g_envmap_texture.Sample(g_envmap_sampler, dir);
		}
		else
		{
			float4 texel = g_texture0.Sample(g_sampler0, In.tex0);
			Out.diff = texel * Out.diff;
		}
	}

	// Env Map
	if (HasEnvMap(g_nugget.flags) && HasNormals(g_nugget.flags))
		Out.diff = EnvironmentMap(g_frame.env_map, In.ws_vert, In.ws_norm, g_frame.cam.c2w[3], Out.diff);

	// Shadows
	float light_visible = 1.0f;
	if (ShadowMapCount(g_frame.shadow) != 0)
		light_visible = LightVisibility(g_frame.shadow, In.ws_vert);

	// Lighting
	if (HasNormals(g_nugget.flags))
		Out.diff = Illuminate(g_frame.global_light, In.ws_vert, In.ws_norm, g_frame.cam.c2w[3], light_visible, Out.diff);

	// If not alpha blending, clip alpha pixels
	if (!HasAlpha(g_nugget.flags))
		clip(Out.diff.a - 0.5);

	return Out;
}

// Direct-lighting PBR pixel shader path.
PSOut PSForwardPbr(PSIn In, bool is_front_face : SV_IsFrontFace)
{
	PSOut Out = (PSOut)0;

	// Resolve the scalar and texture-backed material inputs into a linear base colour.
	float4 base_colour = g_pbr.base_colour * In.diff;
	if (HasTex0(g_nugget.flags))
	{
		float4 tex_colour = g_texture0.Sample(g_sampler0, In.tex0);
		if (AnySet(g_pbr.texture_flags, PbrTextureFlag_BaseColourSrgb))
			tex_colour = SrgbToLinear(tex_colour);

		base_colour *= tex_colour;
	}

	// Apply the material alpha mode before lighting so masked surfaces can discard early.
	float alpha = base_colour.a;
	if (g_pbr.alpha_mode == AlphaModeMask)
		clip(alpha - g_pbr.alpha_cutoff);
	else if (g_pbr.alpha_mode == AlphaModeOpaque)
		alpha = 1.0f;

	// Convert material parameters to the ranges expected by the PBR lighting equations.
	float3 albedo = saturate(base_colour.rgb);
	float3 emissive = g_pbr.emissive.rgb;
	float metallic = saturate(g_pbr.metallic);
	float roughness = clamp(g_pbr.roughness, 0.04f, 1.0f);

	// Without a usable normal there is no surface orientation, so leave the material effectively unlit.
	if (!HasNormals(g_nugget.flags))
	{
		Out.diff = float4(saturate(albedo + emissive), alpha);
		return Out;
	}

	float3 normal = ResolveWorldNormal(In, is_front_face).xyz;
	if (dot(normal, normal) == 0.0f)
	{
		Out.diff = float4(saturate(albedo + emissive), alpha);
		return Out;
	}

	// Evaluate direct PBR lighting using the shared PBR material lighting helper.
	normal = normalize(normal);
	float3 view = normalize(g_frame.cam.c2w[3].xyz - In.ws_vert.xyz);
	float light_visible = ShadowMapCount(g_frame.shadow) != 0
		? LightVisibility(g_frame.shadow, In.ws_vert)
		: 1.0f;
	float3 colour = PbrIlluminate(g_frame.global_light, In.ws_vert.xyz, normal, view, light_visible, albedo, metallic, roughness, emissive);

	Out.diff = float4(saturate(colour), alpha);
	return Out;
}

// Collect one transparent fragment into the forward alpha K-buffer.
void CollectAlphaLayer(PSIn In, float4 diff, bool is_front_face)
{
	clip(diff.a - (1.0f / 255.0f));

	// Reject transparent fragments hidden behind the opaque pass.
	uint2 pix = uint2(In.ss_vert.xy);
	uint width, height, sample_count;
	g_opaque_depth.GetDimensions(width, height, sample_count);

	float opaque_depth = 1.0f;
	for (uint sample = 0; sample != sample_count; ++sample)
		opaque_depth = min(opaque_depth, g_opaque_depth.Load(pix, sample));
	if (In.ss_vert.z >= opaque_depth)
		discard;

	// Pack view-space depth, colour, and optional RT sidecar metadata for later resolve.
	float view_z = -mul(In.ws_vert, g_frame.cam.w2c).z;
	uint depth = PackDepthKey(view_z, ClipPlanes(g_frame.cam.c2s), uint(g_nugget.flags.w));
	uint colour = PackRGBA8(diff);
	uint rt_attrs = AlphaRtAttributes(In, diff, is_front_face);

	// Insert the transparent layer into the rasterizer-ordered K-buffer.
	uint4 alpha_colour = g_alpha_colour[pix];
	uint4 alpha_depth = g_alpha_depth[pix];
	uint4 alpha_rt_attrs = g_alpha_rt_attrs[pix];
	InsertKBufferLayer(alpha_colour, alpha_depth, alpha_rt_attrs, colour, depth, rt_attrs);
	g_alpha_colour[pix] = alpha_colour;
	g_alpha_depth[pix] = alpha_depth;
	g_alpha_rt_attrs[pix] = alpha_rt_attrs;
}

// Collect transparent simple-material fragments into the alpha K-buffer.
void PSForwardAlphaCollect(PSIn In, bool is_front_face : SV_IsFrontFace)
{
	CollectAlphaLayer(In, PSForward(In, is_front_face).diff, is_front_face);
}

// Collect transparent PBR fragments into the alpha K-buffer.
void PSForwardPbrAlphaCollect(PSIn In, bool is_front_face : SV_IsFrontFace)
{
	CollectAlphaLayer(In, PSForwardPbr(In, is_front_face).diff, is_front_face);
}

// Forward pass that also writes reflection attributes for RT reflections.
PSReflectionOut PSForwardReflectionAttrs(PSIn In, bool is_front_face : SV_IsFrontFace)
{
	PSReflectionOut Out = (PSReflectionOut)0;
	Out.diff = PSForward(In, is_front_face).diff;
	Out.reflection_attrs = ReflectionAttributes(In, Out.diff, is_front_face);
	return Out;
}

// PBR forward pass that also writes reflection attributes for RT reflections.
PSReflectionOut PSForwardPbrReflectionAttrs(PSIn In, bool is_front_face : SV_IsFrontFace)
{
	PSReflectionOut Out = (PSReflectionOut)0;
	Out.diff = PSForwardPbr(In, is_front_face).diff;
	Out.reflection_attrs = PbrReflectionAttributes(In, Out.diff, is_front_face);
	return Out;
}

// Forward radial fade pass.
PSOut PSForwardRadialFade(PSIn In, bool is_front_face : SV_IsFrontFace)
{
	PSOut Out = PSForward(In, is_front_face);

	// Fade pixels radially from 'centre'
	float4 centre = any(g_fade.fade_centre) ? g_fade.fade_centre : g_frame.cam.c2w[3];
	float4 radial = In.ws_vert - centre;
	float radius =
		g_fade.fade_type == 0 ? length(radial) : // Spherical
		g_fade.fade_type == 1 ? length(radial - dot(radial, g_frame.cam.c2w[1]) * g_frame.cam.c2w[1]) : // Cylindrical
		0;

	// Lerp to alpha = 0 based on distance
	float frac = smoothstep(g_fade.fade_radius[0], g_fade.fade_radius[1], radius);
	Out.diff.a = lerp(Out.diff.a, 0, frac);
	return Out;
}
