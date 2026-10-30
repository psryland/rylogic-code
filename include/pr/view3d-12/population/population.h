//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
// Native mirror of the population contract in "pr/view3d-12/shaders/population.hlsli". See that file for the meaning of each field.
#pragma once
#include "pr/view3d-12/forward.h"

namespace pr::rdr12
{
	// The maximum number of LODs in a population prototype. Matches VIEW3D_POPULATION_MAX_LODS.
	inline constexpr int PopulationMaxLods = 8;

	// Flags of a population instance. Matches VIEW3D_POPULATION_FLAG_*.
	enum class EPopulationInstanceFlags : uint32_t
	{
		None = 0,
		Enabled = 1 << 0,
		_flags_enum = 0,
	};

	// One placement of a population prototype. Matches 'View3DPopulationInstance'.
	struct PopulationInstance
	{
		// Rows of the affine instance-to-world transform: 'world[i] = Dot(o2w[i], p.w1())'.
		v4 o2w[3];

		// Multiplies the vertex colour.
		Colour32 tint;

		// See EPopulationInstanceFlags.
		EPopulationInstanceFlags flags;

		// Caller payload index, interpreted only by caller shaders.
		uint32_t payload;

		// Must be zero.
		uint32_t reserved;
	};
	static_assert(sizeof(PopulationInstance) == 64, "Must match View3DPopulationInstance");

	// Make an instance record from an affine instance-to-world transform. 'o2w.w' must be (0,0,0,1) in its 'w' components.
	inline PopulationInstance MakePopulationInstance(m4x4 const& o2w, Colour32 tint, EPopulationInstanceFlags flags, uint32_t payload)
	{
		// The record holds the rows of the transform, so the shader can use dot products.
		assert(o2w.x.w == 0 && o2w.y.w == 0 && o2w.z.w == 0 && o2w.w.w == 1 && "Population instance transforms must be affine");
		return PopulationInstance{
			.o2w = {
				v4{ o2w.x.x, o2w.y.x, o2w.z.x, o2w.w.x },
				v4{ o2w.x.y, o2w.y.y, o2w.z.y, o2w.w.y },
				v4{ o2w.x.z, o2w.y.z, o2w.z.z, o2w.w.z },
			},
			.tint = tint,
			.flags = flags,
			.payload = payload,
			.reserved = 0,
		};
	}

	// Dither word fields. Match VIEW3D_POPULATION_DITHER_*.
	inline constexpr uint32_t PopulationDitherCoverageMask = 0x0000FFFFu;
	inline constexpr int PopulationDitherSeedShift = 16;
	inline constexpr uint32_t PopulationDitherSeedMask = 0x3FFFu;
	inline constexpr uint32_t PopulationDitherActive = 0x40000000u;
	inline constexpr uint32_t PopulationDitherComplement = 0x80000000u;

	// Byte offsets of the per-view list header. Match VIEW3D_POPULATION_HEADER_*.
	inline constexpr int PopulationHeaderCounts = 0;
	inline constexpr int PopulationHeaderOffsets = 32;
	inline constexpr int PopulationHeaderCursors = 64;
	inline constexpr int PopulationHeaderDiag = 96;
	inline constexpr int PopulationHeaderSize = 128;

	// One entry of a per-view visible list. Matches 'View3DPopulationEntry'.
	struct PopulationEntry
	{
		uint32_t source;
		uint32_t dither;
	};
	static_assert(sizeof(PopulationEntry) == 8, "Must match View3DPopulationEntry");

	// Per-draw constants of a population draw. Matches 'View3DPopulationDraw'.
	struct PopulationDraw
	{
		uint32_t lod;
		uint32_t list_base;
		uint32_t pad0;
		uint32_t pad1;
	};
	static_assert(sizeof(PopulationDraw) == 16, "Must match View3DPopulationDraw");
}
