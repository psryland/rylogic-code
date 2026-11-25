//************************************
// Game Systems
//  Copyright (c) Rylogic Ltd 2026
//************************************
// Shared types for the fork lightning bolt shaders.
// This file is included from both HLSL and C++ source.
#ifndef PR_GAME_SYSTEMS_LIGHTNING_CBUF_HLSLI
#define PR_GAME_SYSTEMS_LIGHTNING_CBUF_HLSLI
#include "pr/hlsl/interop.hlsli"

#ifdef __cplusplus
namespace pr::game_systems::sky
{
	using namespace pr::hlsl;
#endif

// Lightning bolt constant buffer. Bound to b3.
// Each vertex of a bolt carries (arrival time, brightness, width, clarity) in its colour. See LightningNode.
struct CBufLightning //:reg(b3)
{
	// The render target size in pixels (xy), and the number of pixels per world unit at clip w = 1 (z). A length at clip w covers z / w pixels.
	float4 screen_dim;

	// The colour of the bolt's glow (rgb). The core of the channel is drawn whiter than this.
	float4 colour;

	// Seconds since the strike started, and the time when the leader reaches the ground and stops growing.
	float age;
	float leader_time;

	// The bolt's brightness (see LightningLight::m_glow) and the progress of its fade, in [0,1].
	float glow;
	float fade;

	// The smallest half width of the main channel's core, in pixels, so distant bolts stay visible. Dimmer branches get a smaller minimum,
	// in proportion to their brightness. The minimum narrows with the channel as the strike fades, but never below 'thin_pixels'.
	float min_pixels;
	float thin_pixels;
	float pad1;
	float pad2;
};

#ifdef __cplusplus
}
#endif
#endif
