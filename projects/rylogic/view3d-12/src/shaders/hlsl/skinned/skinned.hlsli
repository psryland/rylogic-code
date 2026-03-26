//***********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2010
//***********************************************
#ifndef PR_VIEW3D_SHADER_SKINNED_HLSLI
#define PR_VIEW3D_SHADER_SKINNED_HLSLI
#include "view3d-12/src/shaders/hlsl/types.hlsli"

// Skin 'vert'. 'vert' is a vertex in model space when in the rest pose
float4 SkinVertex(in StructuredBuffer<Mat4x4> pose, in Skinfluence influence, in float4 os_vert)
{
	int4 idx_lo = (influence.bones >>  0) & 0xFFFF;
	int4 idx_hi = (influence.bones >> 16) & 0xFFFF;
	float4 wgt_lo = ((influence.weights >>  0) & 0xFFFF) / 65535.0f;
	float4 wgt_hi = ((influence.weights >> 16) & 0xFFFF) / 65535.0f;

	float4 skinned_vert = float4(0, 0, 0, 1);
	skinned_vert.xyz += mul(os_vert, pose[idx_lo.x].m).xyz * wgt_lo.x;
	skinned_vert.xyz += mul(os_vert, pose[idx_hi.x].m).xyz * wgt_hi.x;
	skinned_vert.xyz += mul(os_vert, pose[idx_lo.y].m).xyz * wgt_lo.y;
	skinned_vert.xyz += mul(os_vert, pose[idx_hi.y].m).xyz * wgt_hi.y;
	skinned_vert.xyz += mul(os_vert, pose[idx_lo.z].m).xyz * wgt_lo.z;
	skinned_vert.xyz += mul(os_vert, pose[idx_hi.z].m).xyz * wgt_hi.z;
	skinned_vert.xyz += mul(os_vert, pose[idx_lo.w].m).xyz * wgt_lo.w;
	skinned_vert.xyz += mul(os_vert, pose[idx_hi.w].m).xyz * wgt_hi.w;
	return skinned_vert;
}

// Skin 'norm'. 'norm' is a normal in model space when in the rest pose
float4 SkinNormal(in StructuredBuffer<Mat4x4> pose, in Skinfluence influence, in float4 os_norm)
{
	int4 idx_lo = (influence.bones >>  0) & 0xFFFF;
	int4 idx_hi = (influence.bones >> 16) & 0xFFFF;
	float4 wgt_lo = ((influence.weights >>  0) & 0xFFFF) / 65535.0f;
	float4 wgt_hi = ((influence.weights >> 16) & 0xFFFF) / 65535.0f;

	float4 skinned_norm = float4(0, 0, 0, 0);
	skinned_norm.xyz += mul(os_norm, pose[idx_lo.x].m).xyz * wgt_lo.x;
	skinned_norm.xyz += mul(os_norm, pose[idx_hi.x].m).xyz * wgt_hi.x;
	skinned_norm.xyz += mul(os_norm, pose[idx_lo.y].m).xyz * wgt_lo.y;
	skinned_norm.xyz += mul(os_norm, pose[idx_hi.y].m).xyz * wgt_hi.y;
	skinned_norm.xyz += mul(os_norm, pose[idx_lo.z].m).xyz * wgt_lo.z;
	skinned_norm.xyz += mul(os_norm, pose[idx_hi.z].m).xyz * wgt_hi.z;
	skinned_norm.xyz += mul(os_norm, pose[idx_lo.w].m).xyz * wgt_lo.w;
	skinned_norm.xyz += mul(os_norm, pose[idx_hi.w].m).xyz * wgt_hi.w;
	return skinned_norm;
}

#endif

