#pragma once
#include "src/forward.h"

namespace physics_sandbox::procedural_sky
{
	// A simple description of a procedural sky.
	// The sky is rendered into the 6 faces of a cube map as a sum of a horizon-to-zenith
	// gradient, a procedural "ground" band below the horizon plane, and a soft sun disc.
	// All colours are linear (post sRGB-decode). Alpha is ignored — output is BGRA8 opaque.
	struct SkyDesc
	{
		// Sky gradient (top of sphere = zenith, equator = horizon, bottom = ground).
		Colour m_zenith   = Colour(0.20f, 0.45f, 0.85f, 1.0f);
		Colour m_horizon  = Colour(0.85f, 0.90f, 0.98f, 1.0f);
		Colour m_ground   = Colour(0.18f, 0.22f, 0.18f, 1.0f);

		// Sun direction in world space (points TOWARD the sun; e.g. mostly up with a tilt for an apparent late-morning sun).
		// Need not be normalised — the generator normalises before use; a zero vector disables the sun.
		v4 m_sun_dir      = v4(0.4f, -0.3f, 0.85f, 0.0f);
		Colour m_sun_col  = Colour(1.0f, 0.96f, 0.85f, 1.0f);

		// Cosine of the sun disc angular radius (1 = point, smaller = wider disc).
		// 0.9995 ~ 1.8 degrees (similar to the real sun); we use a slightly larger 0.999 ~ 2.6 degrees by default
		// so the disc is more visible in low-resolution cube faces and obvious in reflections.
		float m_sun_core_cos     = 0.999f;

		// Cosine of the sun halo angular radius (gives a soft falloff around the disc).
		float m_sun_halo_cos     = 0.985f;

		// Up-axis used for the sky gradient. Defaults to world Z-up (sandbox convention).
		v4 m_up                  = v4(0.0f, 0.0f, 1.0f, 0.0f);
	};

	// Generate the six face images of a procedural sky cube map.
	// The returned vector owns the pixel data; faces are in DX cube-map order (+X, -X, +Y, -Y, +Z, -Z),
	// BGRA8 format, square, with edge length `face_size` (typical: 128-256).
	std::vector<::pr::compute::ImageWithData> GenerateSkyFaces(int face_size, SkyDesc const& desc);
}
