// Opt-in output variants share stock shading without adding work to disabled pixel shaders.
#include "view3d-12/src/shaders/hlsl/forward/forward.hlsl"
#include "view3d-12/src/shaders/hlsl/forward/far_clip_fade.hlsli"

// Draw a fixed black diagnostic edge without writing auxiliary render targets.
PSOut PSWire(PSIn In)
{
	PSOut Out = (PSOut)0;
	Out.diff = float4(0, 0, 0, 1);
	return Out;
}

// Draw only the depth-writing portion of a faded opaque diagnostic edge.
PSOut PSFarFadeWire(PSIn In)
{
	ClipFarFadeOpaque(In.ws_vert);
	return PSWire(In);
}

// Replace the matching faded layer colour instead of collecting a second translucent fragment.
void PSFarFadeWireCollect(PSIn In)
{
	ClipFarFadeCollect(In.ws_vert);
	OverlayAlphaLayer(In, float3(0, 0, 0));
}

// Preserve depth-writing simple coverage up to the start of the fade.
PSOut PSFarFade(PSIn In, bool is_front_face : SV_IsFrontFace)
{
	ClipFarFadeOpaque(In.ws_vert);
	return PSForward(In, is_front_face);
}

// Preserve depth-writing PBR coverage up to the start of the fade.
PSOut PSFarFadePbr(PSIn In, bool is_front_face : SV_IsFrontFace)
{
	ClipFarFadeOpaque(In.ws_vert);
	return PSForwardPbr(In, is_front_face);
}

// Preserve depth-writing multi-UV PBR coverage up to the start of the fade.
PSOut PSFarFadePbrTexN(PSInTexN In, bool is_front_face : SV_IsFrontFace)
{
	ClipFarFadeOpaque(In.ws_vert);
	return PSForwardPbrTexN(In, is_front_face);
}

// Collect fading simple coverage with the ordinary sorted transparent layers.
void PSFarFadeAlphaCollect(PSIn In, bool is_front_face : SV_IsFrontFace)
{
	ClipFarFadeCollect(In.ws_vert);
	CollectAlphaLayer(In, ApplyFarFadeAlpha(In.ws_vert, PSForward(In, is_front_face).diff), is_front_face);
}

// Collect fading PBR coverage after applying the material's alpha mask.
void PSFarFadePbrAlphaCollect(PSIn In, bool is_front_face : SV_IsFrontFace)
{
	ClipFarFadeCollect(In.ws_vert);
	CollectAlphaLayer(In, ApplyFarFadeAlpha(In.ws_vert, PSForwardPbrImpl(In, is_front_face).diff), is_front_face);
}

// Collect fading multi-UV PBR coverage after applying the material's alpha mask.
void PSFarFadePbrTexNAlphaCollect(PSInTexN In, bool is_front_face : SV_IsFrontFace)
{
	ClipFarFadeCollect(In.ws_vert);
	CollectAlphaLayer(ToPSIn(In), ApplyFarFadeAlpha(In.ws_vert, PSForwardPbrImpl(In, is_front_face).diff), is_front_face);
}

// Keep reflection attributes only for the surviving depth-writing simple coverage.
PSReflectionOut PSFarFadeReflectionAttrs(PSIn In, bool is_front_face : SV_IsFrontFace)
{
	ClipFarFadeOpaque(In.ws_vert);
	return PSForwardReflectionAttrs(In, is_front_face);
}

// Keep reflection attributes only for the surviving depth-writing PBR coverage.
PSReflectionOut PSFarFadePbrReflectionAttrs(PSIn In, bool is_front_face : SV_IsFrontFace)
{
	ClipFarFadeOpaque(In.ws_vert);
	return PSForwardPbrReflectionAttrs(In, is_front_face);
}

// Keep reflection attributes only for the surviving depth-writing multi-UV PBR coverage.
PSReflectionOut PSFarFadePbrTexNReflectionAttrs(PSInTexN In, bool is_front_face : SV_IsFrontFace)
{
	ClipFarFadeOpaque(In.ws_vert);
	return PSForwardPbrTexNReflectionAttrs(In, is_front_face);
}
