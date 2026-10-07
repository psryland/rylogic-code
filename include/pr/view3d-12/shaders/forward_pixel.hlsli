//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
// Forward pixel families: caller-written forward pixel shaders that reuse the stock simple-material shading and outputs.
//
// A forward pass draws the same material through several pixel-shader entry points, one per output contract: opaque, opaque with reflection
// attributes, transparent K-buffer collection, and the far-clip-fade version of each. A family is the six entry points that share one shading
// function. Write the shading function once and expand VIEW3D_FORWARD_PIXEL_ENTRY_POINTS to generate the family:
//
//   #include "pr/view3d-12/shaders/forward_pixel.hlsli"
//   float4 MyShade(inout PSIn In, bool is_front_face)
//   {
//       float4 diff = ForwardShade(In, is_front_face).diff;   // The stock simple-material shading, in linear colour before dithering.
//       ...                                                   // Modify the colour, or modify 'In' before calling ForwardShade.
//       return diff;
//   }
//   VIEW3D_FORWARD_PIXEL_ENTRY_POINTS(My, MyShade)
//
// This generates PSMy, PSMyReflectionAttrs, PSMyAlphaCollect, PSMyFarFade, PSMyFarFadeReflectionAttrs and PSMyFarFadeAlphaCollect.
// Compile each with '-T ps_6_6 -HV 2021' and the Rylogic native include directory on the include path, then pass the bytecode to
// ShaderOptions::m_procedural.m_forward_pixel in that order with 'm_forward_pixel_model' set to EForwardPixelModel::Simple.
// Changes made to 'In' by the shading function are used for the reflection attributes. For PBR materials, see 'forward_pixel_pbr.hlsli'.
//
// Resources:
//  - The stock forward resources are declared by this header (for example 'g_frame', 'g_nugget', 'g_lights').
//  - The procedural constants (VIEW3D_PROCEDURAL_FORWARD_CONSTANTS_REGISTER) and optional buffer (VIEW3D_PROCEDURAL_BUFFER_REGISTER)
//    are visible to the pixel shader as well as the vertex shader. Declare them in the caller's shader as for the vertex shader.
//  - b7 and t12 are reserved by the stock shaders and must not be declared.
//
// The stock shaders and their structures are internal to View3D and may change with any package version.
#ifndef PR_VIEW3D_FORWARD_PIXEL_HLSLI
#define PR_VIEW3D_FORWARD_PIXEL_HLSLI
#include "pr/view3d-12/shaders/procedural_vertex.hlsli"
#include "view3d-12/src/shaders/hlsl/forward/forward.hlsl"
#include "view3d-12/src/shaders/hlsl/forward/far_clip_fade.hlsli"

// Generate the six forward pixel entry points 'PS<Prefix>...' around 'Shade', a function 'float4 Shade(inout PSIn In, bool is_front_face)'
// that returns the linear fragment colour before output dithering.
#define VIEW3D_FORWARD_PIXEL_ENTRY_POINTS(Prefix, Shade)\
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
		Out.reflection_attrs = ReflectionAttributes(In, Out.diff, is_front_face);\
		Out.diff = DitherOutput(Out.diff, In.ss_vert);\
		return Out;\
	}\
	void PS##Prefix##AlphaCollect(PSIn In, bool is_front_face : SV_IsFrontFace)\
	{\
		float4 diff = Shade(In, is_front_face);\
		CollectAlphaLayer(In, diff, AlphaRtAttributes(In, diff, is_front_face));\
	}\
	PSOut PS##Prefix##FarFade(PSIn In, bool is_front_face : SV_IsFrontFace)\
	{\
		ClipFarFadeOpaque(In.ws_vert);\
		return PS##Prefix(In, is_front_face);\
	}\
	PSReflectionOut PS##Prefix##FarFadeReflectionAttrs(PSIn In, bool is_front_face : SV_IsFrontFace)\
	{\
		ClipFarFadeOpaque(In.ws_vert);\
		return PS##Prefix##ReflectionAttrs(In, is_front_face);\
	}\
	void PS##Prefix##FarFadeAlphaCollect(PSIn In, bool is_front_face : SV_IsFrontFace)\
	{\
		ClipFarFadeCollect(In.ws_vert);\
		float4 diff = Shade(In, is_front_face);\
		CollectAlphaLayer(In, ApplyFarFadeAlpha(In.ws_vert, diff), AlphaRtAttributes(In, diff, is_front_face));\
	}

#endif
