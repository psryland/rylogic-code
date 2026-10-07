//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
// PBR forward pixel families: caller-written forward pixel shaders that reuse the stock PBR material shading and outputs.
//
// This is the PBR equivalent of 'forward_pixel.hlsli'; see that header for what a forward pixel family is. Write the shading function once and
// expand VIEW3D_FORWARD_PBR_PIXEL_ENTRY_POINTS to generate the family:
//
//   #include "pr/view3d-12/shaders/forward_pixel_pbr.hlsli"
//   float4 MyShade(inout PSIn In, bool is_front_face)
//   {
//       float4 diff = ForwardShadePbr(In, is_front_face).diff;  // The stock PBR shading, in linear colour before dithering.
//       ...                                                     // Modify the colour, or modify 'In' before calling ForwardShadePbr.
//       return diff;
//   }
//   VIEW3D_FORWARD_PBR_PIXEL_ENTRY_POINTS(My, MyShade)
//
// This generates PSMy, PSMyReflectionAttrs and PSMyAlphaCollect.
// Compile each with '-T ps_6_6 -HV 2021' and the Rylogic native include directory on the include path, then pass the bytecode to
// ShaderOptions::m_procedural.m_forward_pixel in that order with 'm_forward_pixel_model' set to EForwardPixelModel::Pbr.
// Changes made to 'In' by the shading function are used for the reflection attributes.
//
// A PBR family is used only by PBR materials that read all textures through TEXCOORD_0. Drawing it with a simple material, or with a PBR material
// that uses extra texture-coordinate streams, fails at render time.
//
// Resources are as described in 'forward_pixel.hlsli'. The PBR material constants ('g_pbr') and textures are also declared by this header.
//
// The stock shaders and their structures are internal to View3D and may change with any package version.
#ifndef PR_VIEW3D_FORWARD_PIXEL_PBR_HLSLI
#define PR_VIEW3D_FORWARD_PIXEL_PBR_HLSLI
#include "pr/view3d-12/shaders/procedural_vertex.hlsli"
#include "view3d-12/src/shaders/hlsl/forward/forward.hlsl"

// Shade a PBR fragment with the stock PBR forward shading. The colour is linear and not yet dithered.
PSOut ForwardShadePbr(PSIn In, bool is_front_face)
{
	// All PBR texture slots read TEXCOORD_0 in this family.
	return PSForwardPbrImpl(In, is_front_face);
}

// Return the RT reflection attributes for a PBR fragment with final colour 'diff'.
float4 ForwardPbrReflectionAttrs(PSIn In, float4 diff, bool is_front_face)
{
	// Reflections use the final material normal, including normal maps and procedural perturbation.
	float3 normal = ResolvePbrMaterialWorldNormal(In, is_front_face, PbrSlotUV(In, g_pbr.normal_texcoord, g_pbr.normal_uv_transform));
	return PbrReflectionAttributes(In, diff, PbrSlotUV(In, g_pbr.metallic_texcoord, g_pbr.metallic_uv_transform), normal);
}

// Return the alpha K-buffer RT attributes for a transparent PBR fragment with final colour 'diff'.
uint ForwardPbrAlphaRtAttrs(PSIn In, float4 diff, bool is_front_face)
{
	// Alpha reflections use the final material normal, as the opaque reflection attributes do.
	float3 normal = ResolvePbrMaterialWorldNormal(In, is_front_face, PbrSlotUV(In, g_pbr.normal_texcoord, g_pbr.normal_uv_transform));
	return AlphaRtAttributesFromNormal(diff, normal, g_nugget.env_reflectivity);
}

// Generate the three PBR forward pixel entry points 'PS<Prefix>...' around 'Shade', a function 'float4 Shade(inout PSIn In, bool is_front_face)'
// that returns the linear fragment colour before output dithering.
#define VIEW3D_FORWARD_PBR_PIXEL_ENTRY_POINTS(Prefix, Shade)\
	PSOut PS##Prefix(PSIn In, bool is_front_face : SV_IsFrontFace)\
	{\
		PSOut Out = (PSOut)0;\
		Out.diff = DitherOutput(Shade(In, is_front_face), In.ss_vert);\
		return Out;\
	}\
	PSReflectionOut PS##Prefix##ReflectionAttrs(PSIn In, bool is_front_face : SV_IsFrontFace)\
	{\
		PSReflectionOut Out = (PSReflectionOut)0;\
		Out.diff = Shade(In, is_front_face);\
		Out.reflection_attrs = ForwardPbrReflectionAttrs(In, Out.diff, is_front_face);\
		Out.diff = DitherOutput(Out.diff, In.ss_vert);\
		return Out;\
	}\
	void PS##Prefix##AlphaCollect(PSIn In, bool is_front_face : SV_IsFrontFace)\
	{\
		float4 diff = Shade(In, is_front_face);\
		CollectAlphaLayer(In, diff, ForwardPbrAlphaRtAttrs(In, diff, is_front_face));\
	}

#endif
