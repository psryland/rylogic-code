//************************************
// Lost at Sea
//  Copyright (c) Rylogic Ltd 2026
//************************************
// GPU data shared by every consumer of the LaS water field.
#ifndef LAS_WATER_FIELD_TYPES_HLSLI
#define LAS_WATER_FIELD_TYPES_HLSLI
#include "pr/hlsl/interop.hlsli"

#ifdef __cplusplus
namespace las::water
{
	using namespace pr::hlsl;
#endif

static const int MaxWaterFieldElementCount = 64;
static const int WaterFieldElementTypeGerstnerWave = 1;
static const int WaterFieldElementTypeStoneDrop = 2;

// Fixed-stride water-field input. The interpretation of the payload depends on info.x:
//   GerstnerWave: position.xy = direction; wave = amplitude, wavelength, speed, steepness.
//   StoneDrop: reserved for the finite radial-wave implementation.
#ifdef __cplusplus
struct alignas(16) WaterFieldElement
#else
struct WaterFieldElement
#endif
{
	int4 info;
	float4 position;
	float4 wave;
	float4 timing;
};

#ifdef __cplusplus
}
#endif
#endif
