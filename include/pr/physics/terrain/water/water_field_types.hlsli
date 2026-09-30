//*********************************************
// Physics Terrain Water
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Fixed-stride water-field data shared by CPU sampling, GPU buoyancy, and water-surface rendering.
#ifndef PR_PHYSICS_WATER_FIELD_TYPES_HLSLI
#define PR_PHYSICS_WATER_FIELD_TYPES_HLSLI
#include "pr/hlsl/interop.hlsli"

#ifdef __cplusplus
namespace pr::physics::terrain::water::shared
{
	using namespace pr::hlsl;
#endif

// Upper bound on the elements in one water field. Consumers can size constant buffers from it.
static const int WaterFieldMaxElementCount = 64;

// Element types stored in WaterFieldElement::info.x. Zero is an inactive element that contributes nothing.
static const int WaterFieldElementNone = 0;
static const int WaterFieldElementSineWave = 1;
static const int WaterFieldElementGerstnerWave = 2;
static const int WaterFieldElementRadialPacket = 3;

// One water-field element. It adds to a still-water level, and info.x selects how the payload is read:
//   SineWave:     position.xy = unit direction, position.z = phase offset (rad); wave = amplitude (m), wavelength (m), angular frequency (rad/s), unused.
//                 Height is A*sin(k*dot(d, xy) + omega*t + phase).
//   GerstnerWave: position.xy = unit direction, position.z = phase offset (rad); wave = amplitude (m), wavelength (m), phase speed (m/s), steepness [0,1].
//                 Height is A*sin(k*dot(d, xy) - k*c*t + phase). Steepness moves rendered vertices sideways but does not change the sampled height.
//   RadialPacket: position.xy = source; wave = amplitude (m), wavelength (m), packet half-width (m), propagation speed (m/s);
//                 timing = age (s), lifetime (s), attack time (s), radial attenuation scale (m).
// Unused fields must be zero.
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
	static_assert(sizeof(WaterFieldElement) == 64 && alignof(WaterFieldElement) == 16);
	static_assert(std::is_trivially_copyable_v<WaterFieldElement>);
}
#endif
#endif
