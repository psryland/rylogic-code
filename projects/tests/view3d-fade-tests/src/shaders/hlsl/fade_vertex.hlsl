#include "view3d-12/src/shaders/hlsl/forward/forward_cbuf.hlsli"
ConstantBuffer<CBufFrame> g_frame : register(b0);
ConstantBuffer<CBufNugget> g_nugget : register(b1);

// Deform geometry in world space while preserving the stock forward pixel-input contract.
PSIn VSMain(VSIn input)
{
	PSIn output = (PSIn)0;
	output.ws_vert = mul(mul(input.vert, g_nugget.m2o), g_nugget.o2w);
	output.ws_vert.z -= 4.5f;
	output.ss_vert = mul(output.ws_vert, g_frame.cam.w2s);
	output.ws_norm = mul(input.norm, g_nugget.n2w);
	output.diff = input.diff * g_nugget.tint;
	output.tex0 = input.tex0;
	output.idx0 = input.idx0;
	return output;
}

// Deliberately unsupported custom pixel output, used only to verify explicit rejection.
float4 PSMain(PSIn input) : SV_Target
{
	return input.diff;
}
