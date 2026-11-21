//*********************************************
// Game Systems Particle Tests
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Layouts of the test chip particle type, shared by its kernels, its vertex shader and the tests. Chips are small tumbling boxes, like wood
// chips, with their own render record and vertex shader. See particle_chip_cs.hlsl and particle_chip_vs.hlsl.
#ifndef GAME_SYSTEMS_TESTS_PARTICLE_CHIP_HLSLI
#define GAME_SYSTEMS_TESTS_PARTICLE_CHIP_HLSLI
#include "pr/game-systems/particles/particle_sprite.hlsli"

#ifdef __cplusplus
#include <array>
namespace game_systems_tests
{
	using namespace pr::hlsl;
#endif

// The number of logical vertices and indices of a chip: four vertices and two triangles for each of the box's six faces.
static const uint ParticleChipVertexCount = 24;
static const uint ParticleChipIndexCount = 36;

// The parameters of one chip emitter. 'colour' is 0xAARRGGBB.
struct ParticleChipEmitter
{
	float4 position;  // xyz = Where the first chip of the emitter is born. w = Chip length.
	float4 velocity;  // xyz = Mean birth velocity (m/s). w = Radius of the ball around 'xyz' that birth velocities are spread over.
	float spacing;    // Distance along x between the birth positions of consecutive chips of the emitter.
	float spin;       // Rotation rate (radians/s) about each chip's random axis.
	float lifetime;   // Life of every chip (s).
	uint colour;      // Colour of every chip.
};

// The constants of the chip type.
struct ParticleChipConstants
{
	float4 gravity;  // xyz = Constant acceleration (m/s²). w unused.
};

// The state of one chip. Its position and orientation are functions of its age, so only the age changes.
struct ParticleChipState
{
	float3 p0;
	float age;
	float3 v0;
	float lifetime;
	float3 axis;
	float spin;
	uint colour;
	float size;
	uint pad0;
	uint pad1;
};

// The render record of one chip.
struct ParticleChipRecord
{
	float4 pos_size;     // xyz = Centre. w = Length.
	float4 orientation;  // Unit quaternion (xyz, w).
	uint colour;         // 0xAARRGGBB.
	uint pad0;
	uint pad1;
	uint pad2;
};

// The constants of a chip batch, seen by the chip vertex shader.
struct ParticleChipDrawConstants
{
	float width;      // Chip width as a fraction of its length.
	float thickness;  // Chip thickness as a fraction of its length.
	float pad0;
	float pad1;
};

#ifdef __cplusplus
	static_assert(sizeof(ParticleChipEmitter) == 48);
	static_assert(sizeof(ParticleChipConstants) == 16);
	static_assert(sizeof(ParticleChipState) == 64);
	static_assert(sizeof(ParticleChipRecord) == 48);
	static_assert(sizeof(ParticleChipDrawConstants) == 16);

	// The logical vertex ids drawn for each chip: two triangles over the four corners of each face. See particle_chip_vs.hlsl for the corner order.
	constexpr std::array<uint32_t, ParticleChipIndexCount> ParticleChipIndices()
	{
		// Corners go around each face, so each face is a fan from its first corner.
		auto indices = std::array<uint32_t, ParticleChipIndexCount>{};
		for (auto face = 0U; face != 6U; ++face)
		{
			// Six indices per face.
			auto const v = 4U * face;
			auto const quad = std::array<uint32_t, 6>{ v + 0, v + 1, v + 2, v + 0, v + 2, v + 3 };
			for (auto i = 0U; i != 6U; ++i)
				indices[6 * face + i] = quad[i];
		}
		return indices;
	}
}
#endif
#endif
