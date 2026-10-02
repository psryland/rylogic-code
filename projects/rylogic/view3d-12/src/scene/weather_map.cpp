//************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2025
//************************************
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/scene/weather_map.h"
#include "pr/view3d-12/main/renderer.h"
#include "pr/view3d-12/resource/resource_factory.h"
#include "pr/view3d-12/texture/texture_desc.h"
#include "pr/view3d-12/texture/texture_2d.h"
#include "view3d-12/src/shaders/hlsl/sky/procedural_sky_cbuf.hlsli"

namespace pr::rdr12
{
	namespace
	{
		// Hermite smoothing matching HLSL 'smoothstep', so the CPU cover query agrees with the shader.
		float SmoothStep(float lo, float hi, float x)
		{
			// Normalise into [0,1] before applying the cubic.
			auto t = std::clamp((x - lo) / (hi - lo), 0.0f, 1.0f);
			return t * t * (3.0f - 2.0f * t);
		}

		// Return a repeatable value in [0,1] for an integer lattice point.
		float LatticeValue(int x, int y, uint32_t seed)
		{
			// Mix the coordinates and seed so neighbouring lattice points are uncorrelated.
			auto h = s_cast<uint32_t>(x) * 0x8DA6B343u ^ s_cast<uint32_t>(y) * 0xD8163841u ^ seed * 0xCB1AB31Fu;
			h ^= h >> 13;
			h *= 0x5BD1E995u;
			h ^= h >> 15;
			return s_cast<float>(h & 0xFFFFFF) / s_cast<float>(0xFFFFFF);
		}

		// Smoothly interpolated lattice noise in [0,1] with features about one unit in size.
		float ValueNoise(v2 p, uint32_t seed)
		{
			// Interpolate the four surrounding lattice values with smoothed weights to avoid visible grid lines.
			auto x0 = std::floor(p.x);
			auto y0 = std::floor(p.y);
			auto ix = s_cast<int>(x0);
			auto iy = s_cast<int>(y0);
			auto fx = SmoothStep(0.0f, 1.0f, p.x - x0);
			auto fy = SmoothStep(0.0f, 1.0f, p.y - y0);
			auto a = Lerp(LatticeValue(ix + 0, iy + 0, seed), LatticeValue(ix + 1, iy + 0, seed), fx);
			auto b = Lerp(LatticeValue(ix + 0, iy + 1, seed), LatticeValue(ix + 1, iy + 1, seed), fx);
			return Lerp(a, b, fy);
		}
	}

	// Create the CPU grid and a matching GPU texture holding zero cover.
	WeatherMap::WeatherMap(Renderer& rdr, int width, int height, v2 area_min, v2 area_max)
		: m_rdr(&rdr)
		, m_width(width)
		, m_height(height)
		, m_area_min()
		, m_area_max()
		, m_cover(s_cast<size_t>(std::max(width, 0)) * s_cast<size_t>(std::max(height, 0)), 0.0f)
		, m_tex()
		, m_factory(new ResourceFactory(rdr))
	{
		// Bilinear filtering needs at least two texels in each direction.
		if (width < 2 || height < 2 || width > 4096 || height > 4096)
			throw std::invalid_argument("Weather map dimensions must be in [2,4096]");

		Area(area_min, area_max);

		// The factory persists because a temporary factory blocks until the GPU finishes when it is destroyed.
		auto image = ::pr::compute::Image(m_width, m_height, m_cover.data(), DXGI_FORMAT_R32_FLOAT);
		m_tex = m_factory->CreateTexture2D(TextureDesc(AutoId, ResDesc::Tex2D(image, 1)).name("WeatherMap"));
		m_factory->FlushToGpu(EGpuFlush::Async);
	}

	// Release the texture through the renderer so frames still in flight remain valid.
	WeatherMap::~WeatherMap()
	{
		// The factory destructor waits for any queued upload to finish.
		m_tex = nullptr;
		m_factory.reset();
	}

	// Move the area that the grid covers.
	void WeatherMap::Area(v2 area_min, v2 area_max)
	{
		// A degenerate or non-finite area has no valid mapping from world to grid.
		if (!std::isfinite(area_min.x) || !std::isfinite(area_min.y) || !std::isfinite(area_max.x) || !std::isfinite(area_max.y) ||
			!(area_max.x > area_min.x) || !(area_max.y > area_min.y))
			throw std::invalid_argument("Weather map area must be finite with max > min");

		m_area_min = area_min;
		m_area_max = area_max;
	}

	// Blend each cell toward 'cover' by a per-position weight.
	template <typename WeightFn> void WeatherMap::Blend(float cover, WeightFn weight)
	{
		// Out-of-range cover has no meaning to the sky.
		if (!(cover >= 0 && cover <= 1))
			throw std::invalid_argument("Weather cover must be in [0,1]");

		// Evaluate the brush at each texel centre in world space.
		auto cell = (m_area_max - m_area_min) / v2(s_cast<float>(m_width), s_cast<float>(m_height));
		for (int y = 0; y != m_height; ++y)
		{
			for (int x = 0; x != m_width; ++x)
			{
				auto p = m_area_min + cell * v2(x + 0.5f, y + 0.5f);
				auto& c = m_cover[s_cast<size_t>(y) * m_width + x];
				c = Lerp(c, cover, std::clamp(weight(p), 0.0f, 1.0f));
			}
		}
	}

	// Set every cell to 'cover'.
	void WeatherMap::Fill(float cover)
	{
		// Out-of-range cover has no meaning to the sky.
		if (!(cover >= 0 && cover <= 1))
			throw std::invalid_argument("Weather cover must be in [0,1]");

		std::fill(m_cover.begin(), m_cover.end(), cover);
	}

	// Blend toward 'cover' within a circle.
	void WeatherMap::AddStormCell(v2 centre, float radius, float cover)
	{
		// Validate before editing so a rejected brush leaves the grid unchanged.
		if (!std::isfinite(centre.x) || !std::isfinite(centre.y) || !(radius > 0) || !std::isfinite(radius))
			throw std::invalid_argument("Storm cell centre must be finite and its radius positive");

		Blend(cover, [&](v2 p)
		{
			// Solid core with a soft edge so cells merge smoothly when they overlap.
			return 1.0f - SmoothStep(0.4f * radius, radius, Length(p - centre));
		});
	}

	// Blend toward 'cover' behind a straight front.
	void WeatherMap::AddFront(v2 point, v2 travel_direction, float width, float cover)
	{
		// Validate before editing so a rejected brush leaves the grid unchanged.
		auto len = Length(travel_direction);
		if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(len) || len <= 0 || !(width >= 0) || !std::isfinite(width))
			throw std::invalid_argument("Weather front requires a finite point, a nonzero direction, and a finite nonnegative width");

		auto normal = travel_direction / len;
		Blend(cover, [&](v2 p)
		{
			// Signed distance ahead of the front; the covered side is behind it.
			auto d = Dot(p - point, normal);
			return width > 0 ? 1.0f - SmoothStep(-0.5f * width, 0.5f * width, d) : (d < 0 ? 1.0f : 0.0f);
		});
	}

	// Add smooth random variation to the cover.
	void WeatherMap::AddNoise(float scale, float amplitude, uint32_t seed)
	{
		// Validate before editing so a rejected brush leaves the grid unchanged.
		if (!(scale > 0) || !std::isfinite(scale) || !std::isfinite(amplitude))
			throw std::invalid_argument("Weather noise requires a positive scale and a finite amplitude");

		// Three octaves give large variations with some smaller detail; the sum is normalised to [-1,1].
		auto cell = (m_area_max - m_area_min) / v2(s_cast<float>(m_width), s_cast<float>(m_height));
		for (int y = 0; y != m_height; ++y)
		{
			for (int x = 0; x != m_width; ++x)
			{
				// Sample in world space so the pattern does not depend on the grid resolution.
				auto p = m_area_min + cell * v2(x + 0.5f, y + 0.5f);
				auto n = 0.0f;
				auto norm = 0.0f;
				auto amp = 1.0f;
				auto freq = 1.0f / scale;
				for (int o = 0; o != 3; ++o)
				{
					n += amp * ValueNoise(p * freq, seed + o);
					norm += amp;
					amp *= 0.5f;
					freq *= 2.0f;
				}
				auto& c = m_cover[s_cast<size_t>(y) * m_width + x];
				c = std::clamp(c + amplitude * (2.0f * n / norm - 1.0f), 0.0f, 1.0f);
			}
		}
	}

	// Return the cover at 'position', matching the shader's sampling.
	float WeatherMap::CoverAt(v2 position, float default_cover) const
	{
		// Normalised position within the area.
		auto uv = (position - m_area_min) / (m_area_max - m_area_min);

		// Bilinear filter between texel centres, clamped at the edges like the shader.
		auto fx = std::clamp(uv.x * m_width - 0.5f, 0.0f, s_cast<float>(m_width - 1));
		auto fy = std::clamp(uv.y * m_height - 0.5f, 0.0f, s_cast<float>(m_height - 1));
		auto x0 = std::min(s_cast<int>(fx), m_width - 2);
		auto y0 = std::min(s_cast<int>(fy), m_height - 2);
		auto tx = fx - x0;
		auto ty = fy - y0;
		auto at = [&](int x, int y) { return m_cover[s_cast<size_t>(y) * m_width + x]; };
		auto map_cover = Lerp(Lerp(at(x0, y0), at(x0 + 1, y0), tx), Lerp(at(x0, y0 + 1), at(x0 + 1, y0 + 1), tx), ty);

		// Fade to the default cover near and beyond the edges so the map has no visible boundary in the sky.
		auto edge = std::min({uv.x, 1.0f - uv.x, uv.y, 1.0f - uv.y});
		return Lerp(default_cover, map_cover, SmoothStep(0.0f, PR_SKY_WEATHER_EDGE_FADE, edge));
	}

	// Copy the CPU grid to the GPU texture.
	void WeatherMap::Upload()
	{
		// Queue on the graphics queue so the copy is ordered before later frames that sample it.
		auto image = ::pr::compute::Image(m_width, m_height, m_cover.data(), DXGI_FORMAT_R32_FLOAT);
		::pr::compute::UpdateSubresourceScope map(m_factory->CmdList(), m_factory->UploadBuffer(), m_tex->m_res.get(), 0, 0, 1, D3D12_TEXTURE_DATA_PLACEMENT_ALIGNMENT);
		map.Write(image);
		map.Commit(::pr::compute::EFinalState::Restore);
		m_factory->FlushToGpu(EGpuFlush::Async);
	}

	// Ref-counting clean up function
	void WeatherMap::RefCountZero(RefCounted<WeatherMap>* doomed)
	{
		delete static_cast<WeatherMap*>(doomed);
	}

	// Throw if 'weather' is null
	void Validate(WeatherMap const* weather)
	{
		if (weather == nullptr)
			throw std::runtime_error("Weather map pointer is null");
	}
}
