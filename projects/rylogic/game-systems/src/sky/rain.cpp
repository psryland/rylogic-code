//************************************
// Game Systems
//  Copyright (c) Rylogic Ltd 2026
//************************************
#include "pr/game-systems/forward.h"
#include "pr/game-systems/sky/rain.h"
#include "game-systems/src/shaders/hlsl/sky/rain_cbuf.hlsli"

#ifdef NDEBUG
#define PR_GAME_SYSTEMS_SHADER_COMPILED_DIR(file) PR_STRINGISE(game-systems/src/shaders/hlsl/compiled/release/##file)
#else
#define PR_GAME_SYSTEMS_SHADER_COMPILED_DIR(file) PR_STRINGISE(game-systems/src/shaders/hlsl/compiled/debug/##file)
#endif

namespace pr::game_systems
{
	namespace shader_code
	{
		// The rain's build-time shader bytecode
		namespace compiled
		{
			#include PR_GAME_SYSTEMS_SHADER_COMPILED_DIR(rain_vs.h)
			#include PR_GAME_SYSTEMS_SHADER_COMPILED_DIR(rain_ps.h)
			#include PR_GAME_SYSTEMS_SHADER_COMPILED_DIR(rain_weighted_ps.h)
		}
		static compute::ByteCode const rain_vs(compiled::rain_vs);
		static compute::ByteCode const rain_ps(compiled::rain_ps);
		static compute::ByteCode const rain_weighted_ps(compiled::rain_weighted_ps);
	}

	namespace
	{
		static_assert(Rain::LayerCount == PR_RAIN_LAYERS && Rain::SpeedClasses == PR_RAIN_SPEED_CLASSES);

		// The side length of the largest tile. The whole drop field repeats with this period.
		constexpr double FieldPeriod = Rain::TileSize * Rain::TileRatio * Rain::TileRatio * Rain::TileRatio;
		static_assert(Rain::LayerCount == 4, "FieldPeriod assumes four layers");

		// Drop thresholds stop short of 1 by the shader's fade range, so every drop is fully shown at an intensity of 1. See rain.hlsl.
		constexpr float ThresholdRange = 0.95f;

		// The longest drawn streak, in world units. It stops very fast cameras turning drops into long lines across the view.
		constexpr float MaxStreakLength = 1.5f;

		// The smallest drawn streak width, in pixels. Thinner drops are drawn this wide but fainter, so they do not flicker.
		constexpr float MinPixels = 1.5f;

		// The distances over which the smallest layer fades in. Drops nearer than this would be large, fast streaks across the view.
		constexpr float NearFadeStart = 0.4f;
		constexpr float NearFadeEnd = 1.2f;

		// The fraction of a tile's side length at which its layer starts and finishes fading out. The next layer fades in over the same distances.
		constexpr float FarFadeStart = 0.35f;
		constexpr float FarFadeEnd = 0.5f;

		// A small deterministic random number generator, so the drop pattern is the same every run.
		struct DropRandom
		{
			uint64_t m_state;

			// Uniform in [0,1).
			float Next()
			{
				// SplitMix64: mix a stepped counter into 64 well-distributed bits, and keep the top 24 bits for an exact float.
				m_state += 0x9E3779B97F4A7C15ull;
				auto z = m_state;
				z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
				z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
				z = z ^ (z >> 31);
				return s_cast<float>(z >> 40) * (1.0f / 16777216.0f);
			}
		};

		// The fractional part of 'x', in [0,1).
		double Frac(double x)
		{
			// Rounding can give exactly 1 for tiny negative values, which wraps to 0.
			auto f = x - std::floor(x);
			return f < 1.0 ? f : 0.0;
		}

		// True if all of 'v's xyz components are finite.
		bool IsFinite3(v4 v)
		{
			return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
		}
	}

	// Replaces the forward vertex stage and the pixel stages that transparent nuggets use, using the overlay constant buffer slot.
	struct RainShader : Shader
	{
		sky::CBufRain m_cbuf;

		// How far each speed class has moved since the rain was created, wrapped to the field period so it never loses precision.
		std::array<std::array<double, 3>, Rain::SpeedClasses> m_drift;

		// No rain until updated.
		explicit RainShader(Renderer& rdr)
			: Shader(rdr)
			, m_cbuf()
			, m_drift()
		{
			// Each layer's tile, the number of smallest-layer drops one of its drops stands for, and where it fades in.
			for (int k = 0; k != Rain::LayerCount; ++k)
			{
				auto size = s_cast<float>(Rain::LayerSize(k));
				auto ratio = size / s_cast<float>(Rain::TileSize);
				auto fade_start = k == 0 ? NearFadeStart : FarFadeStart * s_cast<float>(Rain::LayerSize(k - 1));
				auto fade_end = k == 0 ? NearFadeEnd : FarFadeEnd * s_cast<float>(Rain::LayerSize(k - 1));
				m_cbuf.layers[k] = v4(size, ratio * ratio * ratio, fade_start, fade_end);
			}
			m_cbuf.min_pixels = MinPixels;
			m_cbuf.max_length = MaxStreakLength;

			// The pixel stage is chosen per forward sub-pass: weighted transparency normally, or the stock alpha blend without transparency buffers.
			m_code = ShaderCode{
				.VS = shader_code::rain_vs,
				.PS = rdr12::shader_code::none,
				.DS = rdr12::shader_code::none,
				.HS = rdr12::shader_code::none,
				.GS = rdr12::shader_code::none,
				.CS = rdr12::shader_code::none,
			};
			m_pixel_family = ForwardPixelFamily{
				.m_code = { shader_code::rain_ps, rdr12::shader_code::none, rdr12::shader_code::none, shader_code::rain_weighted_ps },
				.m_replaces = &rdr12::shader_code::forward_family,
			};
		}

		// Bind the rain's constants for the camera of this draw. Drops are placed relative to this camera, so every view sees the same field.
		void SetupElement(ID3D12GraphicsCommandList* cmd_list, GpuUploadBuffer& upload, Scene const& scene, CameraTransforms const& cam, DrawListElement const*) override
		{
			// The camera-to-screen projection's X scale converts a width at clip w = 1 to a fraction of half the screen width.
			auto size = To<v2>(scene.wnd().RenderTargetSize());
			auto c2w = InvertOrthonormal(cam.m_w2c);
			auto c2s = cam.m_w2s * c2w;
			m_cbuf.w2c = cam.m_w2c;
			m_cbuf.c2s = c2s;
			m_cbuf.cam_pos = c2w.pos;
			m_cbuf.screen_dim = v4(size.x, size.y, c2s.x.x * size.x * 0.5f, 0);

			// A drop's place in its tile is its drift relative to the camera. Work in double and wrap to the field period first, so large world
			// positions keep their precision. The camera's position wraps by the same period because every tile size divides it.
			double cam_pos[3] = { std::fmod(s_cast<double>(c2w.pos.x), FieldPeriod), std::fmod(s_cast<double>(c2w.pos.y), FieldPeriod), std::fmod(s_cast<double>(c2w.pos.z), FieldPeriod) };
			for (int k = 0; k != Rain::LayerCount; ++k)
			{
				auto tile = Rain::LayerSize(k);
				for (int c = 0; c != Rain::SpeedClasses; ++c)
				{
					auto const& drift = m_drift[c];
					m_cbuf.offset[k * Rain::SpeedClasses + c] = v4(
						s_cast<float>(Frac((drift[0] - cam_pos[0]) / tile)),
						s_cast<float>(Frac((drift[1] - cam_pos[1]) / tile)),
						s_cast<float>(Frac((drift[2] - cam_pos[2]) / tile)),
						0);
				}
			}

			auto gpu_address = upload.Add(m_cbuf, D3D12_CONSTANT_BUFFER_DATA_PLACEMENT_ALIGNMENT, true).VirtualAddress();
			cmd_list->SetGraphicsRootConstantBufferView(static_cast<UINT>(shaders::fwd::ERootParam::CBufScreenSpace), gpu_address);
		}
	};

	// One quad per drop, for every layer. The vertices carry fixed random data; the shader places them each frame.
	Rain::Rain(Renderer& rdr)
		: m_inst()
		, m_shader()
		, m_last_time()
	{
		// Each drop has a random place in its tile, a random intensity threshold, and a speed class that cycles so each class has the same share.
		constexpr int drop_count = LayerCount * DropsPerLayer;
		static_assert(s_cast<int64_t>(drop_count) * 4 < 0xFFFFFFFFll);
		ModelGenerator::Buffers<Vert> buf;
		buf.Reset(drop_count * 4, 0, 0, sizeof(uint32_t));
		buf.m_icont.reserve<uint32_t>(s_cast<size_t>(drop_count) * 6);
		DropRandom rng{ .m_state = 0x5241494E5241494Eull };
		for (int layer = 0; layer != LayerCount; ++layer)
		{
			for (int i = 0; i != DropsPerLayer; ++i)
			{
				// The four corners share the drop's data and differ only in which corner they are.
				auto pos = v4(rng.Next(), rng.Next(), rng.Next(), 1.0f);
				auto threshold = rng.Next() * ThresholdRange;
				auto slot = s_cast<float>(layer * SpeedClasses + i % SpeedClasses);
				auto base = (layer * DropsPerLayer + i) * 4;
				static float const corners[4][2] = { {0, -1}, {0, +1}, {1, -1}, {1, +1} };
				for (int c = 0; c != 4; ++c)
				{
					auto& v = buf.m_vcont[base + c];
					v.m_vert = pos;
					v.m_diff = Colour(threshold, corners[c][0], corners[c][1], slot);
					v.m_norm = v4::Zero();
					v.m_tex0 = v2::Zero();
					v.m_idx0 = iv2::Zero();
				}

				// Two triangles from tail to head.
				buf.m_icont.push_back(s_cast<uint32_t>(base + 0));
				buf.m_icont.push_back(s_cast<uint32_t>(base + 1));
				buf.m_icont.push_back(s_cast<uint32_t>(base + 2));
				buf.m_icont.push_back(s_cast<uint32_t>(base + 2));
				buf.m_icont.push_back(s_cast<uint32_t>(base + 1));
				buf.m_icont.push_back(s_cast<uint32_t>(base + 3));
			}
		}

		// The drops are placed around whichever camera draws them, so the bounds only describe the field's extent around the model origin.
		auto reach = s_cast<float>(0.5 * FieldPeriod);
		buf.m_bbox = BBox(v4::Origin(), v4(reach, reach, reach, 0));

		// The material retains the shader, so its pointer remains valid for the lifetime of the model.
		// Streaks are transparent, so they draw with the other transparent surfaces and blend correctly over water. They are not sorted with the
		// K-buffer layers, because there are far too many to sort, so they draw in the weighted (order-independent) transparency group.
		auto shdr = Shader::Create<RainShader>(rdr);
		m_shader = shdr.get();
		buf.m_ncont.push_back(
			NuggetDesc(ETopo::TriList, EGeom::Vert | EGeom::Colr)
				.flags(ENuggetFlag::ShadowCastExclude | ENuggetFlag::GeometryHasAlpha | ENuggetFlag::AlphaWeighted)
				.mat([&](MaterialSimple& m) {
					m.use_shader_overlay(ERenderStep::RenderForward, shdr);
				})
			);

		ResourceFactory factory(rdr);
		ModelGenerator::Cache cache{buf};
		m_inst.m_model = ModelGenerator::Create<Vert>(factory, cache);
		m_inst.m_i2w = m4x4::Identity();
		factory.FlushToGpu(EGpuFlush::Block);
	}

	// Move the drops by the time since the last update, and pass the settings to the shader.
	void Rain::Update(RainSettings const& settings)
	{
		// Check everything before changing anything, so invalid settings leave the rain as it was.
		auto fall = settings.m_fall_velocity;
		fall.w = 0;
		auto fall_sq = LengthSq(fall);
		if (!(settings.m_intensity >= 0 && settings.m_intensity <= 1))
			throw std::invalid_argument("Rain requires an intensity in [0,1]");
		if (!IsFinite3(settings.m_wind) || !IsFinite3(settings.m_camera_velocity) || !std::isfinite(fall_sq) || fall_sq <= 0)
			throw std::invalid_argument("Rain requires a finite wind and camera velocity, and a finite nonzero fall velocity");
		if (!IsFinite3(settings.m_colour) || settings.m_colour.x < 0 || settings.m_colour.y < 0 || settings.m_colour.z < 0 || !(settings.m_opacity >= 0 && settings.m_opacity <= 1))
			throw std::invalid_argument("Rain requires a finite nonnegative colour and an opacity in [0,1]");
		if (!IsFinite3(settings.m_clip_plane) || !std::isfinite(settings.m_clip_plane.w))
			throw std::invalid_argument("Rain requires a finite clip plane");
		if (!(settings.m_streak_time > 0) || !std::isfinite(settings.m_streak_time) || !(settings.m_drop_width > 0) || !std::isfinite(settings.m_drop_width))
			throw std::invalid_argument("Rain requires a finite positive streak time and drop width");
		if (!std::isfinite(settings.m_time))
			throw std::invalid_argument("Rain requires a finite time");

		// Each speed class drifts with the wind plus its own fall velocity. Drift is kept in double and wrapped to the field period, which
		// leaves the drops where they are because the field repeats with that period.
		auto dt = m_last_time ? settings.m_time - *m_last_time : 0.0;
		m_last_time = settings.m_time;
		auto& cbuf = m_shader->m_cbuf;
		for (int c = 0; c != SpeedClasses; ++c)
		{
			auto velocity = settings.m_wind + fall * SpeedScale[c];
			velocity.w = 0;
			auto& drift = m_shader->m_drift[c];
			for (int a = 0; a != 3; ++a)
				drift[a] = std::fmod(drift[a] + s_cast<double>(velocity[a]) * dt, FieldPeriod);

			cbuf.velocity[c] = velocity - settings.m_camera_velocity;
			cbuf.velocity[c].w = 0;
		}

		// Heavier rain has slightly larger drops as well as more of them.
		cbuf.colour = v4(settings.m_colour.x, settings.m_colour.y, settings.m_colour.z, settings.m_opacity);
		cbuf.clip_plane = settings.m_clip_plane;
		cbuf.intensity = settings.m_intensity;
		cbuf.streak_time = settings.m_streak_time;
		cbuf.half_width = 0.5f * settings.m_drop_width * Lerp(0.6f, 1.0f, settings.m_intensity);
		cbuf.ref_length = std::sqrt(fall_sq) * settings.m_streak_time;
	}

	// Each layer's tile is 'TileRatio' times larger than the one before.
	double Rain::LayerSize(int layer)
	{
		assert(layer >= 0 && layer < LayerCount);
		auto size = TileSize;
		for (int k = 0; k != layer; ++k)
			size *= TileRatio;

		return size;
	}
}
