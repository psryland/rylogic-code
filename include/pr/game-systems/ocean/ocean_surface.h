//************************************
// Game Systems
//  Copyright (c) Rylogic Ltd 2026
//************************************
// A wind-driven ocean surface with foam, drawn as one procedural View3D object whose vertices are generated on the GPU.
// The surface is a camera-centred lattice clipped to a circular domain, with an opaque curtain hanging from its edge to the domain floor.
// Waves follow a wind sea (see pr/physics/terrain/water/wind_sea.hlsli) and are corrected for the depth over a bathymetry grid.
// The CPU copy of the wave state lets callers sample the drawn surface height exactly (see Height).
#pragma once
#include "pr/game-systems/forward.h"

namespace pr::game_systems::ocean
{
	// One ocean surface. It holds the fixed layout needed to pack each frame's constants, and the CPU copies of the wave state.
	// The View3D object is created by Create and owned by the caller; the methods that take an object require the object that Create returned.
	struct OceanSurface
	{
		virtual ~OceanSurface() = default;

		// Validate 'options' and create the surface and its procedural View3D object, returned in 'out_object'.
		static std::unique_ptr<OceanSurface> Create(pr::view3d::OceanOptions const& options, char const* name, pr::view3d::Object& out_object);

		// Build the shared index list for surfaces with 'vertices_per_side' vertices per side, so creating them later does not. Any thread may call this.
		static void PrepareLattice(int32_t vertices_per_side);

		// Return the detail map (pr::view3d::OceanDetailMapSize^2 RGBA8 pixels, row-major) that OceanOptions requires. It depends on nothing else,
		// so it can be baked ahead of time on any thread.
		static std::vector<uint32_t> BakeDetailMap();

		// Pack one frame's state into the constants and hand them to the object.
		virtual void Update(pr::view3d::Object object, pr::view3d::OceanFrame const& frame) = 0;

		// Replace the wind-sea data of 'object', laid out by the surface's fixed grid. Null elements keep the current elements and table.
		virtual void SetWindSea(pr::view3d::Object object, pr::view3d::OceanWindSea const& wind_sea) = 0;

		// Draw the ripple strength as colour instead of the shaded surface. Takes effect from the next Update.
		virtual void ShowRippleStrength(bool show) = 0;

		// Draw or hide the small-scale ripple normals, and the foam and lip. Takes effect from the next Update.
		virtual void SetEffects(bool ripples, bool foam) = 0;

		// Validate and store the foam. Takes effect from the next Update.
		virtual void SetFoam(pr::view3d::OceanFoam const& foam) = 0;

		// Bind the ground source that the pixel shaders read for the shore foam and lip, or the surface's own buffer when 'buffer' is null.
		// 'params' fill the 'shore_source' constants. Takes effect from the next Update.
		virtual void SetShoreSource(pr::view3d::Object object, ID3D12Resource* buffer, std::span<uint32_t const, 12> params) = 0;

		// Return the height (m) of the drawn surface at the world position 'x_m', 'y_m' in the last frame passed to Update.
		// Positions beyond the domain radius take the height at the domain edge, where the surface ends.
		virtual double Height(double x_m, double y_m) const = 0;
	};
}
