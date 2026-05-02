//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
// Helpers for the 1x alpha K-buffer.
#ifndef PR_VIEW3D_SHADER_KBUFFER_HLSLI
#define PR_VIEW3D_SHADER_KBUFFER_HLSLI

static const uint KBufferDepthMask = 0x00FFFFFFu;
static const uint KBufferOiaMask   = 0xFF000000u;
static const uint KBufferDepthFar  = 0x00FFFFFFu;

uint PackRGBA8(float4 colour)
{
	uint4 c = uint4(round(saturate(colour) * 255.0f));
	return c.r | (c.g << 8) | (c.b << 16) | (c.a << 24);
}

float4 UnpackRGBA8(uint colour)
{
	uint4 c = uint4(colour, colour >> 8, colour >> 16, colour >> 24) & 0xFFu;
	return float4(c) / 255.0f;
}

uint PackDepth24(float view_z, float2 near_far)
{
	float t = saturate((view_z - near_far.x) / max(near_far.y - near_far.x, 1e-6f));
	return uint(round(t * float(KBufferDepthFar)));
}

uint DepthOf(uint packed_depth)
{
	return packed_depth & KBufferDepthMask;
}

uint WithDepth(uint packed_depth, uint depth)
{
	return (packed_depth & KBufferOiaMask) | (depth & KBufferDepthMask);
}

uint OiaChannel(uint packed_depth)
{
	return packed_depth >> 24;
}

uint WithOiaChannel(uint packed_depth, uint channel)
{
	return (packed_depth & KBufferDepthMask) | ((channel & 0xFFu) << 24);
}

uint4 PackOia(float4 oia)
{
	uint4 c = uint4(round(saturate(oia) * 255.0f));
	return c;
}

float4 UnpackOia(uint4 alpha_depth)
{
	return float4(OiaChannel(alpha_depth.x), OiaChannel(alpha_depth.y), OiaChannel(alpha_depth.z), OiaChannel(alpha_depth.w)) / 255.0f;
}

uint4 StoreOia(uint4 alpha_depth, float4 oia)
{
	uint4 c = PackOia(oia);
	alpha_depth.x = WithOiaChannel(alpha_depth.x, c.r);
	alpha_depth.y = WithOiaChannel(alpha_depth.y, c.g);
	alpha_depth.z = WithOiaChannel(alpha_depth.z, c.b);
	alpha_depth.w = WithOiaChannel(alpha_depth.w, c.a);
	return alpha_depth;
}

float4 AccumulateOia(float4 oia, float4 colour)
{
	float a = saturate(colour.a);
	float one_minus_a = 1.0f - oia.a;
	oia.rgb += colour.rgb * a * one_minus_a;
	oia.a += a * one_minus_a;
	return saturate(oia);
}

void AddOverflow(inout uint4 alpha_depth, uint colour)
{
	float4 oia = UnpackOia(alpha_depth);
	oia = AccumulateOia(oia, UnpackRGBA8(colour));
	alpha_depth = StoreOia(alpha_depth, oia);
}

void InsertKBufferLayer(inout uint4 alpha_colour, inout uint4 alpha_depth, uint colour, uint depth)
{
	uint4 d = uint4(DepthOf(alpha_depth.x), DepthOf(alpha_depth.y), DepthOf(alpha_depth.z), DepthOf(alpha_depth.w));
	if (depth >= d.w)
	{
		AddOverflow(alpha_depth, colour);
		return;
	}

	if (d.w != KBufferDepthFar)
		AddOverflow(alpha_depth, alpha_colour.w);

	if (depth < d.x)
	{
		alpha_colour.w = alpha_colour.z;
		alpha_colour.z = alpha_colour.y;
		alpha_colour.y = alpha_colour.x;
		alpha_colour.x = colour;
		alpha_depth.w = WithDepth(alpha_depth.w, d.z);
		alpha_depth.z = WithDepth(alpha_depth.z, d.y);
		alpha_depth.y = WithDepth(alpha_depth.y, d.x);
		alpha_depth.x = WithDepth(alpha_depth.x, depth);
		return;
	}
	if (depth < d.y)
	{
		alpha_colour.w = alpha_colour.z;
		alpha_colour.z = alpha_colour.y;
		alpha_colour.y = colour;
		alpha_depth.w = WithDepth(alpha_depth.w, d.z);
		alpha_depth.z = WithDepth(alpha_depth.z, d.y);
		alpha_depth.y = WithDepth(alpha_depth.y, depth);
		return;
	}
	if (depth < d.z)
	{
		alpha_colour.w = alpha_colour.z;
		alpha_colour.z = colour;
		alpha_depth.w = WithDepth(alpha_depth.w, d.z);
		alpha_depth.z = WithDepth(alpha_depth.z, depth);
		return;
	}

	alpha_colour.w = colour;
	alpha_depth.w = WithDepth(alpha_depth.w, depth);
}

#endif
