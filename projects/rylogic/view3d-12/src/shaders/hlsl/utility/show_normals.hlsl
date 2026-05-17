//***********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2010
//***********************************************
#include "view3d-12/src/shaders/hlsl/types.hlsli"
#include "view3d-12/src/shaders/hlsl/forward/forward_cbuf.hlsli"

// Constant buffers
ConstantBuffer<CBufFrame> g_frame : register(b0);
ConstantBuffer<CBufDiag> g_diag : register(b3);

// Converts point geometry into normal vectors
[maxvertexcount(2)]
void GSShowNormals(point PSIn In[1], inout LineStream<PSIn> OutStream)
{
	PSIn Out = In[0];
	Out.diff = g_diag.colour;
	Out.ws_norm = float4(0, 0, 0, 0);
	
	Out.ws_vert = In[0].ws_vert;
	Out.ss_vert = mul(Out.ws_vert, g_frame.cam.w2s);
	OutStream.Append(Out);

	Out.ws_vert = In[0].ws_vert + g_diag.length * In[0].ws_norm;
	Out.ss_vert = mul(Out.ws_vert, g_frame.cam.w2s);
	OutStream.Append(Out);

	OutStream.RestartStrip();
}
