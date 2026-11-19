//************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//************************************
// The renderer's contract with a sky whose clouds shade the scene.
#pragma once
#include "pr/view3d-12/forward.h"

namespace pr::rdr12
{
	// A sky whose clouds shade the scene's sun light and whose low cloud the renderer follows between frames.
	// Give it to the scene for the current frame with 'Scene::CloudSky'. The renderer only reads it while rendering that frame.
	struct ICloudSky
	{
		// Get the cloud field for cloud shadows on the scene, and the sky's texture table (cube, weather map, cloud noise) that the cloud field reads.
		// Returns false, with 'clouds' still filled, when the sky casts no cloud shadows.
		virtual bool CloudShadows(shaders::CloudConstants& clouds, Descriptor (&textures)[3]) const = 0;

		// The total distance the low cloud has moved with the wind, in world units in the sky frame. Views use it to follow the cloud between frames.
		virtual std::array<double, 2> LowCloudDrift() const = 0;

	protected:

		// Owners destroy the concrete sky, never through this interface.
		~ICloudSky() = default;
	};
}
