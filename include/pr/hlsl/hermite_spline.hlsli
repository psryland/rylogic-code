//*********************************************
// HLSL
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#ifndef PR_HLSL_HERMITE_SPLINE_HLSLI
#define PR_HLSL_HERMITE_SPLINE_HLSLI

struct HermiteSpline
{
	float4x4 m_coeff;
};

static const float4x4 Hermite = {
	{+1.f, +0.f, +0.f, +0.f},
	{+0.f, +1.f, +0.f, +0.f},
	{-3.f, -2.f, +3.f, -1.f},
	{+2.f, +1.f, -2.f, +1.f},
};

HermiteSpline HermiteSpline_Create(float3 x0, float3 v0, float3 x1, float3 v1)
{
	HermiteSpline hs;
	hs.m_coeff = mul(Hermite, float4x4(
		float4(x0, 0),
		float4(v0, 0),
		float4(x1, 0),
		float4(v1, 0))
	);
	return hs;
}
float3 HermiteSpline_Position(HermiteSpline hs, float t)
{
	t = saturate(t);
	float4 p = mul(float4(1, t, t * t, t * t * t), hs.m_coeff);
	return p.xyz;
}
float3 HermiteSpline_Velocity(HermiteSpline hs, float t)
{
	t = saturate(t);
	float4 p = mul(float4(0, 1, 2 * t, 3 * t * t), hs.m_coeff);
	return p.xyz;
}
float3 HermiteSpline_Acceleration(HermiteSpline hs, float t)
{
	t = saturate(t);
	float4 p = mul(float4(0, 0, 2, 6 * t), hs.m_coeff);
	return p.xyz;
}
float3 HermiteSpline_Jolt(HermiteSpline hs)
{
	float4 p = mul(float4(0, 0, 0, 6), hs.m_coeff);
	return p.xyz;
}

#endif
