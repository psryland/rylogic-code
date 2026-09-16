#pragma once
#include "src/forward.h"

namespace physics_sandbox
{
	// Optionally reduce sleeping-object opacity while preserving RGB and already lower source opacity.
	inline Colour32 DisplayColour(Colour32 colour, bool sleeping, bool sleeping_transparency)
	{
		if (sleeping_transparency && sleeping && colour.a > 0x30)
			colour.a = 0x30;

		return colour;
	}
}
