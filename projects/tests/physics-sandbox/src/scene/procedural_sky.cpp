#include "src/scene/procedural_sky.h"

namespace physics_sandbox::procedural_sky
{
	namespace
	{
		// Map a face index + (u,v) in [-1,+1] to a world-space direction.
		// Face order follows the DX cube-map convention: 0=+X, 1=-X, 2=+Y, 3=-Y, 4=+Z, 5=-Z.
		// The world up axis is +Y inside the cube-map convention; callers can interpret the
		// returned direction in whatever coordinate frame they want (we use it as-is and then
		// dot it against `desc.m_up` to get the elevation).
		v4 FaceDirection(int face, float u, float v)
		{
			switch (face)
			{
				case 0: return Normalise(v4(+1.0f, -v,   -u, 0.0f)); // +X
				case 1: return Normalise(v4(-1.0f, -v,   +u, 0.0f)); // -X
				case 2: return Normalise(v4(+u,   +1.0f, +v, 0.0f)); // +Y
				case 3: return Normalise(v4(+u,   -1.0f, -v, 0.0f)); // -Y
				case 4: return Normalise(v4(+u,   -v,   +1.0f, 0.0f)); // +Z
				case 5: return Normalise(v4(-u,   -v,   -1.0f, 0.0f)); // -Z
				default: throw std::runtime_error("Cube map face index out of range");
			}
		}

		// Evaluate the procedural sky function for a single direction.
		// Returns a linear-space colour. Elevation t is the dot product of dir with the up axis,
		// in [-1, +1] (1=zenith, 0=horizon, -1=nadir).
		Colour EvaluateSky(v4 const& dir, SkyDesc const& desc, v4 const& sun_dir, bool sun_enabled)
		{
			auto const t = Dot3(dir, desc.m_up);

			// Sky gradient: zenith -> horizon -> ground.
			Colour sky{};
			if (t >= 0.0f)
			{
				// Above the horizon: smoothly blend horizon -> zenith with a soft curve
				// (1 - (1-t)^2) keeps the zenith band concentrated near the top.
				auto const k = 1.0f - (1.0f - t) * (1.0f - t);
				sky.r = desc.m_horizon.r * (1.0f - k) + desc.m_zenith.r * k;
				sky.g = desc.m_horizon.g * (1.0f - k) + desc.m_zenith.g * k;
				sky.b = desc.m_horizon.b * (1.0f - k) + desc.m_zenith.b * k;
				sky.a = 1.0f;
			}
			else
			{
				// Below the horizon: smoothly blend horizon -> ground with a soft curve
				// (1 - (1+t)^2) concentrates the ground band toward the nadir.
				auto const k = 1.0f - (1.0f + t) * (1.0f + t);
				sky.r = desc.m_horizon.r * (1.0f - k) + desc.m_ground.r * k;
				sky.g = desc.m_horizon.g * (1.0f - k) + desc.m_ground.g * k;
				sky.b = desc.m_horizon.b * (1.0f - k) + desc.m_ground.b * k;
				sky.a = 1.0f;
			}

			// Sun disc + halo: additive contribution wherever the ray points near the sun.
			// Two thresholds give a bright core and a softer surrounding glow.
			if (sun_enabled)
			{
				auto const cos_a = Dot3(dir, sun_dir);
				if (cos_a > desc.m_sun_halo_cos)
				{
					auto const halo = (cos_a - desc.m_sun_halo_cos) / std::max(1e-6f, 1.0f - desc.m_sun_halo_cos);
					auto const core = cos_a > desc.m_sun_core_cos
						? 1.0f
						: 0.0f;
					auto const sun_intensity = core * 0.85f + halo * halo * 0.3f;
					sky.r += desc.m_sun_col.r * sun_intensity;
					sky.g += desc.m_sun_col.g * sun_intensity;
					sky.b += desc.m_sun_col.b * sun_intensity;
				}
			}

			// Clamp to displayable range (sun core can saturate but BGRA8 has no headroom).
			sky.r = std::clamp(sky.r, 0.0f, 1.0f);
			sky.g = std::clamp(sky.g, 0.0f, 1.0f);
			sky.b = std::clamp(sky.b, 0.0f, 1.0f);
			return sky;
		}
	}

	std::vector<::pr::compute::ImageWithData> GenerateSkyFaces(int face_size, SkyDesc const& desc)
	{
		if (face_size <= 0)
			throw std::runtime_error("Sky face size must be positive");

		// Pre-normalise the sun direction once. Treat near-zero magnitudes as "no sun".
		auto sun_dir = v4::Zero();
		auto sun_enabled = false;
		if (LengthSq(desc.m_sun_dir) > 1e-8f)
		{
			sun_dir = Normalise(desc.m_sun_dir);
			sun_dir.w = 0.0f;
			sun_enabled = true;
		}

		// Allocate 6 face images. BGRA8 matches the default ImageWithData format and the path the
		// existing texture loaders use for stock textures.
		std::vector<::pr::compute::ImageWithData> faces;
		faces.reserve(6);

		auto const pixels_per_face = static_cast<size_t>(face_size) * static_cast<size_t>(face_size);
		auto const stride = sizeof(uint32_t);

		for (int face = 0; face != 6; ++face)
		{
			// Allocate per-face pixel storage; ImageWithData takes ownership via shared_ptr.
			std::shared_ptr<uint8_t[]> bits(new uint8_t[pixels_per_face * stride]);
			auto* px = reinterpret_cast<uint32_t*>(bits.get());

			for (int y = 0; y != face_size; ++y)
			{
				// Map texel centre to (u,v) in [-1, +1]. The +0.5 ensures we sample the centre of each texel.
				auto const v_coord = (2.0f * (static_cast<float>(y) + 0.5f) / static_cast<float>(face_size)) - 1.0f;
				for (int x = 0; x != face_size; ++x)
				{
					auto const u_coord = (2.0f * (static_cast<float>(x) + 0.5f) / static_cast<float>(face_size)) - 1.0f;
					auto const dir = FaceDirection(face, u_coord, v_coord);
					auto const col = EvaluateSky(dir, desc, sun_dir, sun_enabled);

					// Encode linear -> sRGB BGRA8 (matches DXGI_FORMAT_B8G8R8A8_UNORM with our existing pipeline assumption).
					auto const packed = Colour32(col);
					px[y * face_size + x] = packed.argb;
				}
			}

			faces.emplace_back(face_size, face_size, bits, DXGI_FORMAT_B8G8R8A8_UNORM);
		}

		return faces;
	}
}
