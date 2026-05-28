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

// Textures /w samplers
Texture2D<float4> g_base_texture :register(t0);
SamplerState      g_base_sampler :register(s0);

// Environment map
TextureCube<float4> g_envmap_texture :register(t1);
SamplerState        g_envmap_sampler :register(s1);

// Shadow map
Texture2D<float2> g_smap_texture[MaxShadowMaps] :register(t2);
SamplerComparisonState g_smap_sampler           :register(s2);

// Projected textures
Texture2D<float4> g_proj_texture[MaxProjectedTextures] :register(t3);
SamplerState      g_proj_sampler[MaxProjectedTextures] :register(s3);

// PBR material texture slots.
Texture2D<float4> g_metallic_texture  :register(t4);
Texture2D<float4> g_roughness_texture :register(t5);

// Opaque depth
Texture2DMS<float> g_opaque_depth : register(t6);

// PBR material texture slots continued after the shared forward resources.
Texture2D<float4> g_emissive_texture  :register(t7);
StructuredBuffer<float2> g_tex1 :register(t8);
StructuredBuffer<float2> g_tex2 :register(t9);
StructuredBuffer<float2> g_tex3 :register(t10);
StructuredBuffer<float2> g_tex4 :register(t11);
Texture2D<float4> g_normal_texture    :register(t12);
SamplerState      g_metallic_sampler  :register(s4);
SamplerState      g_roughness_sampler :register(s5);
SamplerState      g_emissive_sampler  :register(s6);
SamplerState      g_normal_sampler    :register(s7);

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

// Return the common pixel-shader input fields from a standard pixel-shader input.
PSIn ToPSIn(PSIn In)
{
	return In;
}

// Return the common pixel-shader input fields from an optional texture-coordinate input.
PSIn ToPSIn(PSInTexN In)
{
	PSIn Out = (PSIn)0;
	Out.ss_vert = In.ss_vert;
	Out.ws_vert = In.ws_vert;
	Out.ws_norm = In.ws_norm;
	Out.diff = In.diff;
	Out.tex0 = In.tex0;
	Out.idx0 = In.idx0;
	return Out;
}

// Return the transformed texture-coordinate value for a shader lane.
float2 PbrTextureUV(PSIn In, int texcoord)
{
	return In.tex0;
}

// Return the transformed texture-coordinate value for a shader lane.
float2 PbrTextureUV(PSInTexN In, int texcoord)
{
	switch (texcoord)
	{
		case 1: return In.tex1;
		case 2: return In.tex2;
		case 3: return In.tex3;
		case 4: return In.tex4;
		default: return In.tex0;
	}
}

// Return one channel from a texture sample using the channel index stored in the material constants.
float SelectTextureChannel(float4 sample, int channel)
{
	return
		channel == 0 ? sample.r :
		channel == 1 ? sample.g :
		channel == 2 ? sample.b :
		channel == 3 ? sample.a :
		sample.r;
}

// Apply a material texture-coordinate transform before sampling a texture slot.
float2 TransformPbrUV(float2 uv, TexXForm transform)
{
	float3 uv1 = float3(uv, 1);
	return float2(dot(uv1, transform.m_x.xyz), dot(uv1, transform.m_y.xyz));
}

// Return the transformed texture-coordinate value for a material texture slot.
template <typename TIn>
float2 PbrSlotUV(TIn In, int texcoord, TexXForm transform)
{
	return TransformPbrUV(PbrTextureUV(In, texcoord), transform);
}

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

// Apply the material normal-map sample to a world-space geometric normal.
float3 PerturbWorldNormal(PSIn In, float3 normal, float2 normal_uv)
{
	float2 map_xy = g_normal_texture.Sample(g_normal_sampler, normal_uv).xy * 2.0f - 1.0f;
	map_xy *= g_pbr.normal_scale;
	float3 map_normal = normalize(float3(map_xy, sqrt(saturate(1.0f - dot(map_xy, map_xy)))));

	float3 dp1 = ddx(In.ws_vert.xyz);
	float3 dp2 = ddy(In.ws_vert.xyz);
	float2 duv1 = ddx(normal_uv);
	float2 duv2 = ddy(normal_uv);
	float3 dp2_perp = cross(dp2, normal);
	float3 dp1_perp = cross(normal, dp1);
	float3 tangent = dp2_perp * duv1.x + dp1_perp * duv2.x;
	float3 bitangent = dp2_perp * duv1.y + dp1_perp * duv2.y;
	float frame_len = max(dot(tangent, tangent), dot(bitangent, bitangent));
	if (frame_len < TINY)
		return normal;

	float inv_frame_len = rsqrt(frame_len);
	return normalize(
		tangent * (map_normal.x * inv_frame_len) +
		bitangent * (map_normal.y * inv_frame_len) +
		normal * map_normal.z);
}

// Return the PBR surface normal after applying any usable normal-map texture.
float3 ResolvePbrWorldNormal(PSIn In, bool is_front_face, float2 normal_uv)
{
	float3 normal = ResolveWorldNormal(In, is_front_face).xyz;
	if (dot(normal, normal) == 0.0f)
		return normal;

	normal = normalize(normal);
	if (AnySet(g_pbr.texture_flags, PbrTextureFlag_HasNormalMap))
		normal = PerturbWorldNormal(In, normal, normal_uv);

	return normal;
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

// Forward VS variant that interpolates optional texture-coordinate lanes for PBR material textures.
PSInTexN VSForwardTexN(VSIn In, uint vertex_id : SV_VertexID)
{
	PSIn base = VSForward(In);

	PSInTexN Out = (PSInTexN)0;
	Out.ss_vert = base.ss_vert;
	Out.ws_vert = base.ws_vert;
	Out.ws_norm = base.ws_norm;
	Out.diff = base.diff;
	Out.tex0 = base.tex0;
	Out.idx0 = base.idx0;

	// Optional streams follow the model vertex-buffer order. SV_VertexID is already the model-buffer index for both indexed and non-indexed draws.
	if (g_pbr.texcoord_count > 0)
		Out.tex1 = g_tex1[vertex_id];

	if (g_pbr.texcoord_count > 1)
		Out.tex2 = g_tex2[vertex_id];

	if (g_pbr.texcoord_count > 2)
		Out.tex3 = g_tex3[vertex_id];

	if (g_pbr.texcoord_count > 3)
		Out.tex4 = g_tex4[vertex_id];

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
			float4 texel = g_base_texture.Sample(g_base_sampler, In.tex0);
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

// Direct-lighting PBR pixel shader path using the supplied UV coordinates for each texture slot.
PSOut PSForwardPbrSampledUV(PSIn In, bool is_front_face, float2 base_uv, float2 metallic_uv, float2 roughness_uv, float2 emissive_uv, float2 normal_uv)
{
	PSOut Out = (PSOut)0;

	// Resolve the scalar and texture-backed material inputs into a linear base colour.
	float4 base_colour = g_pbr.base_colour * In.diff;
	if (AnySet(g_pbr.texture_flags, PbrTextureFlag_HasBaseColourMap))
	{
		float4 tex_colour = g_base_texture.Sample(g_base_sampler, base_uv);
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
	float metallic = saturate(g_pbr.metallic);
	float roughness = clamp(g_pbr.roughness, 0.04f, 1.0f);
	float3 emissive = g_pbr.emissive.rgb;
	if (AnySet(g_pbr.texture_flags, PbrTextureFlag_HasMetallicMap))
		metallic *= SelectTextureChannel(g_metallic_texture.Sample(g_metallic_sampler, metallic_uv), g_pbr.metallic_channel);
	if (AnySet(g_pbr.texture_flags, PbrTextureFlag_HasRoughnessMap))
		roughness *= SelectTextureChannel(g_roughness_texture.Sample(g_roughness_sampler, roughness_uv), g_pbr.roughness_channel);
	if (AnySet(g_pbr.texture_flags, PbrTextureFlag_HasEmissiveMap))
	{
		float4 emissive_tex = g_emissive_texture.Sample(g_emissive_sampler, emissive_uv);
		if (AnySet(g_pbr.texture_flags, PbrTextureFlag_EmissiveSrgb))
			emissive_tex = SrgbToLinear(emissive_tex);

		emissive *= emissive_tex.rgb;
	}

	metallic = saturate(metallic);
	roughness = clamp(roughness, 0.04f, 1.0f);

	// Without a usable normal there is no surface orientation, so leave the material effectively unlit.
	if (!HasNormals(g_nugget.flags))
	{
		Out.diff = float4(saturate(albedo + emissive), alpha);
		return Out;
	}

	float3 normal = ResolvePbrWorldNormal(In, is_front_face, normal_uv);
	if (dot(normal, normal) == 0.0f)
	{
		Out.diff = float4(saturate(albedo + emissive), alpha);
		return Out;
	}

	// Evaluate direct PBR lighting using the shared PBR material lighting helper.
	float3 view = normalize(g_frame.cam.c2w[3].xyz - In.ws_vert.xyz);
	float light_visible = ShadowMapCount(g_frame.shadow) != 0
		? LightVisibility(g_frame.shadow, In.ws_vert)
		: 1.0f;
	float3 colour = PbrIlluminate(g_frame.global_light, In.ws_vert.xyz, normal, view, light_visible, albedo, metallic, roughness, emissive);

	Out.diff = float4(saturate(colour), alpha);
	return Out;
}

// Direct-lighting PBR pixel shader path shared by concrete fast-path and optional texture-coordinate entry points.
template <typename TIn>
PSOut PSForwardPbrImpl(TIn In, bool is_front_face)
{
	return PSForwardPbrSampledUV(
		ToPSIn(In),
		is_front_face,
		PbrSlotUV(In, g_pbr.base_colour_texcoord, g_pbr.base_colour_uv_transform),
		PbrSlotUV(In, g_pbr.metallic_texcoord, g_pbr.metallic_uv_transform),
		PbrSlotUV(In, g_pbr.roughness_texcoord, g_pbr.roughness_uv_transform),
		PbrSlotUV(In, g_pbr.emissive_texcoord, g_pbr.emissive_uv_transform),
		PbrSlotUV(In, g_pbr.normal_texcoord, g_pbr.normal_uv_transform));
}

// Direct-lighting PBR pixel shader path using TEXCOORD_0 for all texture slots.
PSOut PSForwardPbr(PSIn In, bool is_front_face : SV_IsFrontFace)
{
	return PSForwardPbrImpl(In, is_front_face);
}

// Direct-lighting PBR pixel shader path with optional texture-coordinate lanes.
PSOut PSForwardPbrTexN(PSInTexN In, bool is_front_face : SV_IsFrontFace)
{
	return PSForwardPbrImpl(In, is_front_face);
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

	// Pack view-space depth, colour, and optional RT side-buffer metadata for later resolve.
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
	CollectAlphaLayer(In, PSForwardPbrImpl(In, is_front_face).diff, is_front_face);
}

// Collect transparent PBR fragments with optional texture-coordinate lanes into the alpha K-buffer.
void PSForwardPbrTexNAlphaCollect(PSInTexN In, bool is_front_face : SV_IsFrontFace)
{
	CollectAlphaLayer(ToPSIn(In), PSForwardPbrImpl(In, is_front_face).diff, is_front_face);
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
	Out.diff = PSForwardPbrImpl(In, is_front_face).diff;
	Out.reflection_attrs = PbrReflectionAttributes(
		In,
		Out.diff,
		PbrSlotUV(In, g_pbr.metallic_texcoord, g_pbr.metallic_uv_transform),
		ResolvePbrWorldNormal(In, is_front_face, PbrSlotUV(In, g_pbr.normal_texcoord, g_pbr.normal_uv_transform)));
	return Out;
}

// PBR forward pass with optional texture-coordinate lanes that also writes reflection attributes for RT reflections.
PSReflectionOut PSForwardPbrTexNReflectionAttrs(PSInTexN In, bool is_front_face : SV_IsFrontFace)
{
	PSIn base = ToPSIn(In);
	PSReflectionOut Out = (PSReflectionOut)0;
	Out.diff = PSForwardPbrImpl(In, is_front_face).diff;
	Out.reflection_attrs = PbrReflectionAttributes(
		base,
		Out.diff,
		PbrSlotUV(In, g_pbr.metallic_texcoord, g_pbr.metallic_uv_transform),
		ResolvePbrWorldNormal(base, is_front_face, PbrSlotUV(In, g_pbr.normal_texcoord, g_pbr.normal_uv_transform)));
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
