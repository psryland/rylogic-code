//************************************
// Physics Sandbox
//  Copyright (c) Rylogic Ltd 2026
//************************************
// Shared water-visual constant-buffer layout.
#ifndef PHYSICS_SANDBOX_WATER_VISUAL_CBUF_HLSLI
#define PHYSICS_SANDBOX_WATER_VISUAL_CBUF_HLSLI
#include "pr/hlsl/interop.hlsli"
#include "pr/physics/terrain/water/water_field_types.hlsli"

#ifdef __cplusplus
namespace physics_sandbox
{
	using namespace pr::hlsl;
	using namespace pr::physics::terrain::water::shared;
#endif

// The physics water-field elements, uploaded unchanged so the visual surface matches buoyancy sampling.
struct CBufWaterVisual
{
	WaterFieldElement m_elements[WaterFieldMaxElementCount];
	int m_element_count;
	float m_time_s;
	float m_water_level;
	float m_pad;
};

#ifdef __cplusplus
}
#endif
#endif
