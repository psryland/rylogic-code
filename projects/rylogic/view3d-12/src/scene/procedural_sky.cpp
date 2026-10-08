//************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2025
//************************************
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/scene/procedural_sky.h"
#include "pr/view3d-12/scene/weather_map.h"
#include "pr/view3d-12/texture/texture_2d.h"
#include "pr/view3d-12/texture/texture_desc.h"
#include "pr/view3d-12/shaders/shader_forward.h"
#include "pr/view3d-12/model/model_generator.h"
#include "pr/view3d-12/model/vertex_layout.h"
#include "pr/view3d-12/material/material_simple.h"
#include "pr/view3d-12/resource/resource_factory.h"
#include "view3d-12/src/shaders/common.h"
#include "view3d-12/src/shaders/hlsl/sky/procedural_sky_cbuf.hlsli"

namespace pr::rdr12
{
	namespace
	{
		// Scramble a lattice point and seed into a well-mixed 32-bit value.
		uint32_t LatticeHash(int x, int y, uint32_t seed)
		{
			// Multiply by large odd constants and fold the high bits down so neighbouring cells are uncorrelated.
			auto h = s_cast<uint32_t>(x) * 0x8DA6B343u ^ s_cast<uint32_t>(y) * 0xD8163841u ^ seed * 0x165667B1u;
			h ^= h >> 13;
			h *= 0x5BD1E995u;
			h ^= h >> 15;
			return h;
		}

		// Smooth gradient noise that repeats every 'period' lattice cells in x and y. Values lie in about [-0.7, 0.7] with a mean of 0.
		struct PeriodicNoise
		{
			int m_period;
			std::vector<v2> m_gradients;

			// Pick one random unit gradient per lattice point of one period.
			PeriodicNoise(int period, uint32_t seed)
				: m_period(period)
				, m_gradients(s_cast<size_t>(period) * period)
			{
				for (int y = 0; y != period; ++y)
				{
					for (int x = 0; x != period; ++x)
					{
						// Hash bits map to an angle, which gives uniformly distributed directions.
						auto angle = (LatticeHash(x, y, seed) & 0xFFFFu) * (6.2831853f / 65536.0f);
						m_gradients[s_cast<size_t>(y) * period + x] = v2(std::cos(angle), std::sin(angle));
					}
				}
			}

			// Noise at 'p', in lattice cells.
			float operator()(v2 p) const
			{
				// Blend the ramps from the four surrounding lattice points with a smooth fade, wrapping the lattice so the pattern repeats.
				auto ix = s_cast<int>(std::floor(p.x));
				auto iy = s_cast<int>(std::floor(p.y));
				auto f = v2(p.x - ix, p.y - iy);
				auto fade = [](float t)
				{
					return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);
				};
				auto ramp = [&](int dx, int dy)
				{
					auto x = ((ix + dx) % m_period + m_period) % m_period;
					auto y = ((iy + dy) % m_period + m_period) % m_period;
					auto g = m_gradients[s_cast<size_t>(y) * m_period + x];
					return g.x * (f.x - dx) + g.y * (f.y - dy);
				};
				auto u = fade(f.x);
				auto v = fade(f.y);
				return Lerp(Lerp(ramp(0, 0), ramp(1, 0), u), Lerp(ramp(0, 1), ramp(1, 1), u), v);
			}
		};

		// Create the tileable cloud noise texture, with mips. Channels R and G are independent broad noise that sets the cloud masses.
		// Channels B and A are independent heaped noise, where each octave is folded into rounded lumps with creases between them.
		// Each channel is normalised to a mean of 0.5 and a spread of 0.17, so cover thresholds in the shader have a consistent meaning.
		Texture2DPtr CreateCloudNoise(ResourceFactory& factory)
		{
			// Channel recipes: lattice cells per tile for the first octave, octave count, amplitude gain per octave, and whether to fold into lumps.
			struct Recipe
			{
				int cells;
				int octaves;
				float gain;
				bool heaped;
			};
			constexpr int size = PR_SKY_CLOUD_NOISE_SIZE;
			Recipe const recipes[] = {
				{ .cells = 8, .octaves = 3, .gain = 0.45f, .heaped = false },
				{ .cells = 8, .octaves = 3, .gain = 0.45f, .heaped = false },
				{ .cells = 16, .octaves = 4, .gain = 0.5f, .heaped = true },
				{ .cells = 16, .octaves = 4, .gain = 0.5f, .heaped = true },
			};

			std::vector<uint32_t> texels(s_cast<size_t>(size) * size);
			std::vector<float> values(texels.size());
			for (int ch = 0; ch != 4; ++ch)
			{
				// Sum the octaves. Doubling the lattice cells per octave keeps every octave periodic over one tile.
				auto const& recipe = recipes[ch];
				std::fill(values.begin(), values.end(), 0.0f);
				auto amp = 1.0f;
				for (int o = 0; o != recipe.octaves; ++o)
				{
					auto cells = recipe.cells << o;
					PeriodicNoise noise(cells, s_cast<uint32_t>(ch * 16 + o + 1));
					auto scale = s_cast<float>(cells) / size;
					for (int y = 0; y != size; ++y)
					{
						for (int x = 0; x != size; ++x)
						{
							// Folding 1 - n^2 turns each octave into rounded bulges that meet in soft creases.
							auto n = noise(v2(x + 0.5f, y + 0.5f) * scale);
							if (recipe.heaped)
							{
								auto k = std::clamp(n / 0.7f, -1.0f, 1.0f);
								n = 1.0f - k * k;
							}
							values[s_cast<size_t>(y) * size + x] += amp * n;
						}
					}
					amp *= recipe.gain;
				}

				// Normalise to a known mean and spread, then pack as 8-bit unsigned normalised values.
				auto mean = 0.0;
				auto mean_sq = 0.0;
				for (auto v : values)
				{
					mean += v;
					mean_sq += s_cast<double>(v) * v;
				}
				mean /= values.size();
				auto spread = std::sqrt(std::max(mean_sq / values.size() - mean * mean, 1e-12));
				for (size_t i = 0; i != values.size(); ++i)
				{
					auto v = std::clamp(0.5 + 0.17 * (values[i] - mean) / spread, 0.0, 1.0);
					texels[i] |= s_cast<uint32_t>(std::lround(v * 255.0)) << (8 * ch);
				}
			}

			// A full mip chain lets distant cloud read pre-filtered noise instead of shimmering.
			auto image = ::pr::compute::Image(size, size, texels.data(), DXGI_FORMAT_R8G8B8A8_UNORM);
			return factory.CreateTexture2D(TextureDesc(AutoId, ResDesc::Tex2D(image, 0)).name("ProceduralSkyCloudNoise"));
		}
	}

	// Replaces the forward vertex/pixel stages while using its existing overlay constant-buffer slot.
	struct ProceduralSkyShader : Shader
	{
		sky::CBufProceduralSky m_cbuf;
		TextureCubePtr m_background;
		WeatherMapPtr m_weather;
		Texture2DPtr m_cloud_noise;
		::pr::compute::Descriptor m_null_cube;
		::pr::compute::Descriptor m_null_weather;

		// Use the renderer's build-time shader bytecode and default daylight parameters.
		explicit ProceduralSkyShader(Renderer& rdr)
			: Shader(rdr)
			, m_cbuf{
				.sun_direction = Normalise(v4(0.5f, 0.3f, 0.8f, 0)),
				.sun_colour = v4(1.0f, 0.95f, 0.85f, 1),
				.sun_intensity = 1.0f,
				.blend_weight = 1.0f,
				.world_to_sky = m4x4::Identity(),
				.world_to_cube = m4x4::Identity(),
			}
		{
			static_assert(sizeof(m_cbuf) == 320);
			m_code.VS = shader_code::procedural_sky_vs;
			m_code.PS = shader_code::procedural_sky_ps;

			// Bind valid null descriptors even when the atmosphere has no source cubemap or weather map.
			ResourceStore::Access store(rdr);
			auto cube_desc = D3D12_SHADER_RESOURCE_VIEW_DESC{
				.Format = DXGI_FORMAT_R8G8B8A8_UNORM,
				.ViewDimension = D3D12_SRV_DIMENSION_TEXTURECUBE,
				.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING,
				.TextureCube = { .MostDetailedMip = 0, .MipLevels = 1, .ResourceMinLODClamp = 0 },
			};
			auto weather_desc = D3D12_SHADER_RESOURCE_VIEW_DESC{
				.Format = DXGI_FORMAT_R32_FLOAT,
				.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2D,
				.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING,
				.Texture2D = { .MostDetailedMip = 0, .MipLevels = 1, .PlaneSlice = 0, .ResourceMinLODClamp = 0 },
			};
			m_null_cube = store.Descriptors().Create(nullptr, cube_desc);
			m_null_weather = store.Descriptors().Create(nullptr, weather_desc);
		}

		// Return the owned null views to the store; the texture members release their sources independently.
		~ProceduralSkyShader() override
		{
			ResourceStore::Access store(rdr());
			store.Descriptors().Release(m_null_cube);
			store.Descriptors().Release(m_null_weather);
		}

		// Bind sky constants and the background without replacing the shared material or reflection descriptors.
		void SetupElement(ID3D12GraphicsCommandList* cmd_list, GpuUploadBuffer& upload, Scene const& scene, CameraTransforms const&, DrawListElement const* dle) override
		{
			if (dle == nullptr)
				return;

			// Read the weather area now, so moving the area takes effect without re-binding the map.
			m_cbuf.has_weather = m_weather ? 1.0f : 0.0f;
			m_cbuf.weather_area = m_weather
				? v4(m_weather->m_area_min.x, m_weather->m_area_min.y, 1.0f / (m_weather->m_area_max.x - m_weather->m_area_min.x), 1.0f / (m_weather->m_area_max.y - m_weather->m_area_min.y))
				: v4(0, 0, 1, 1);
			auto gpu_address = upload.Add(m_cbuf, D3D12_CONSTANT_BUFFER_DATA_PLACEMENT_ALIGNMENT, true);
			cmd_list->SetGraphicsRootConstantBufferView(static_cast<UINT>(shaders::fwd::ERootParam::CBufScreenSpace), gpu_address);

			// The cube, the weather map and the cloud noise are a contiguous descriptor table.
			::pr::compute::Descriptor const descriptors[] = {
				m_background ? m_background->m_srv : m_null_cube,
				m_weather ? m_weather->m_tex->m_srv : m_null_weather,
				m_cloud_noise->m_srv,
			};
			auto table = scene.wnd().m_heap_view.Add(descriptors);
			cmd_list->SetGraphicsRootDescriptorTable(static_cast<UINT>(shaders::fwd::ERootParam::SkyTexture), table);
		}
	};

	// Build one persistent background triangle whose shader derives world directions from the camera.
	ProceduralSky::ProceduralSky(Renderer& rdr)
		: m_inst()
		, m_shader()
		, m_last_time()
		, m_cloud_offset()
		, m_cloud_evolve()
	{
		// A full-screen triangle covers perspective and orthographic views without cube seams or clip-distance dependence.
		rdr12::ModelGenerator::Buffers<Vert> buf;
		buf.Reset(3, 0, 0, sizeof(uint16_t));
		static v4 const verts[] = {
			v4(-1, -1, 1, 1),
			v4(-1, +3, 1, 1),
			v4(+3, -1, 1, 1),
		};
		for (int i = 0; i != 3; ++i)
		{
			auto& v = buf.m_vcont[i];
			v.m_vert = verts[i];
			v.m_diff = Colour(1.0f, 1.0f, 1.0f, 1.0f);
			v.m_norm = v4::Zero();
			v.m_tex0 = v2::Zero();
			v.m_idx0 = iv2::Zero();
			buf.m_icont.push_back(s_cast<uint16_t>(i));
		}
		buf.m_bbox = BBox(v4::Origin(), v4(1, 1, 1, 0));

		// The material retains the shader, so its pointer remains valid for the lifetime of the model.
		auto shdr = Shader::Create<ProceduralSkyShader>(rdr);
		m_shader = shdr.get();

		buf.m_ncont.push_back(
			NuggetDesc(ETopo::TriList, EGeom::Vert | EGeom::Colr)
				.flags(ENuggetFlag::ShadowCastExclude)
				.pso<EPipeState::CullMode>(D3D12_CULL_MODE_NONE)
				.pso<EPipeState::DepthWriteMask>(D3D12_DEPTH_WRITE_MASK_ZERO)
				.pso<EPipeState::DepthFunc>(D3D12_COMPARISON_FUNC_GREATER_EQUAL)
				.mat([&](MaterialSimple& m) {
					m.use_shader_overlay(ERenderStep::RenderForward, shdr);
				})
			);

		// Upload geometry and the cloud noise once; subsequent frames change only sky constants and the optional source binding.
		auto colour = Colour32White;
		auto opts = ModelGenerator::CreateOptions().colours({ &colour, 1 });
		ResourceFactory factory(rdr);
		ModelGenerator::Cache cache{buf};
		m_inst.m_model = ModelGenerator::Create<Vert>(factory, cache, &opts);
		m_inst.m_i2w = m4x4::Identity();
		m_inst.m_sko.Group(ESortGroup::Skybox);
		shdr->m_cloud_noise = CreateCloudNoise(factory);
		factory.FlushToGpu(EGpuFlush::Block);
	}

	// Reject invalid values before modifying the live shader so failed updates leave the previous sky intact.
	void ProceduralSky::Update(ProceduralSkySettings const& settings)
	{
		auto sun_direction = settings.m_sun_direction;
		auto sun_colour = settings.m_sun_colour;
		sun_direction.w = 0;
		auto length_sq = LengthSq(sun_direction);
		if (!std::isfinite(length_sq) || length_sq <= 0 ||
			!std::isfinite(sun_colour.x) || !std::isfinite(sun_colour.y) || !std::isfinite(sun_colour.z) ||
			sun_colour.x < 0 || sun_colour.y < 0 || sun_colour.z < 0 ||
			!std::isfinite(settings.m_sun_intensity) || settings.m_sun_intensity < 0)
			throw std::invalid_argument("Procedural sky requires a finite nonzero sun direction and finite nonnegative colour/intensity");
		if (!(settings.m_cloud_cover >= 0 && settings.m_cloud_cover <= 1) ||
			!(settings.m_wind_speed >= 0) || !std::isfinite(settings.m_wind_speed) ||
			!std::isfinite(settings.m_wind_direction) || !std::isfinite(settings.m_time))
			throw std::invalid_argument("Procedural sky requires cloud cover in [0,1], a finite nonnegative wind speed, and a finite wind direction and time");
		static_assert(ProceduralSkySettings::LightningMax == PR_SKY_LIGHTNING_MAX);
		for (auto const& flash : settings.m_lightning)
		{
			// Unused flashes have zero brightness. A visible flash needs a finite position and a positive radius.
			if (!std::isfinite(flash.w) || flash.w < 0 || (flash.w > 0 && (!std::isfinite(flash.x) || !std::isfinite(flash.y) || !std::isfinite(flash.z) || flash.z <= 0)))
				throw std::invalid_argument("Procedural sky lightning requires a finite nonnegative brightness, and a finite position and positive radius when visible");
		}

		// Move the clouds by the wind over the elapsed time. The first update, and time going backwards, do not move them.
		auto dt = m_last_time ? std::max(settings.m_time - *m_last_time, 0.0) : 0.0;
		m_last_time = settings.m_time;
		v4 const layers[] = { v4(PR_SKY_CLOUD_LAYER0), v4(PR_SKY_CLOUD_LAYER1), v4(PR_SKY_CLOUD_LAYER2) };
		for (int i = 0; i != 3; ++i)
		{
			// Offsets are in noise tiles of the wind-aligned frame, so the wind moves only the along-wind component.
			// They are wrapped at the noise period, so they stay precise over long run times.
			auto& offset = m_cloud_offset[i];
			auto const& layer = layers[i];
			auto period = s_cast<double>(PR_SKY_CLOUD_PERIOD);
			offset.x = s_cast<float>(std::fmod(offset.x + settings.m_wind_speed * layer.w * dt / layer.y, period));

			// Shapes change slowly in still air, and faster in wind: a quarter of an evolution unit per tile travelled.
			auto evolve_rate = PR_SKY_CLOUD_EVOLVE_RATE + 0.25 * settings.m_wind_speed * layer.w / std::min(layer.y, layer.z);
			m_cloud_evolve[i] = s_cast<float>(std::fmod(m_cloud_evolve[i] + evolve_rate * dt, s_cast<double>(PR_SKY_CLOUD_EVOLVE_PERIOD)));
		}

		m_shader->m_cbuf.sun_direction = Normalise(sun_direction);
		m_shader->m_cbuf.sun_colour = sun_colour;
		m_shader->m_cbuf.sun_intensity = settings.m_sun_intensity;
		m_shader->m_cbuf.cloud_cover = settings.m_cloud_cover;
		m_shader->m_cbuf.time = s_cast<float>(std::fmod(settings.m_time, s_cast<double>(PR_SKY_TIME_PERIOD)));
		m_shader->m_cbuf.cloud_offset01 = v4(m_cloud_offset[0].x, m_cloud_offset[0].y, m_cloud_offset[1].x, m_cloud_offset[1].y);
		m_shader->m_cbuf.cloud_offset2 = m_cloud_offset[2];
		m_shader->m_cbuf.wind_direction = s_cast<float>(std::fmod(settings.m_wind_direction, constants<double>::tau));
		m_shader->m_cbuf.cloud_evolve = v4(m_cloud_evolve[0], m_cloud_evolve[1], m_cloud_evolve[2], 0);
		m_shader->m_cbuf.hidden_cloud_layers = settings.m_hidden_cloud_layers;
		for (int i = 0; i != PR_SKY_LIGHTNING_MAX; ++i)
			m_shader->m_cbuf.lightning[i] = settings.m_lightning[i].w > 0 ? settings.m_lightning[i] : v4::Zero();
	}

	// Retain the weather map; the shader reads its area and texture each frame.
	void ProceduralSky::Weather(WeatherMapPtr weather)
	{
		if (weather && weather->m_rdr != &m_shader->rdr())
			throw std::invalid_argument("The weather map must belong to the same renderer as the sky");

		m_shader->m_weather = std::move(weather);
	}

	// The shader handles the far plane, while the instance remains at the camera for scene bookkeeping.
	void ProceduralSky::AddToScene(Scene& scene)
	{
		if (!m_inst.m_model)
			return;

		// Keep the model independent of the application's world scale and draw distance.
		m_inst.m_i2w = m4x4::Identity();
		m_inst.m_i2w.pos = scene.m_cam.CameraToWorld().pos;
		scene.AddInstance(m_inst);
	}

	// Validate first so a rejected update cannot partly replace the source or its direction mapping.
	void ProceduralSky::Blend(TextureCubePtr background, float weight, m4x4 const& world_to_sky, m4x4 const& world_to_background)
	{
		if (!std::isfinite(weight) || weight < 0 || weight > 1 || (!background && weight != 1))
			throw std::invalid_argument("Sky blend weight must be in [0,1], or 1 when no cubemap is supplied");

		// Only direction rotations are accepted; translation and scale belong to the caller's scene geometry.
		auto valid_rotation = [](m4x4 const& matrix)
		{
			for (auto column : {matrix.x, matrix.y, matrix.z, matrix.pos})
			{
				for (auto value : {column.x, column.y, column.z, column.w})
				{
					if (!std::isfinite(value))
						return false;
				}
			}
			return IsOrthonormal(matrix.rot, 0.0001f) && matrix.x.w == 0 && matrix.y.w == 0 && matrix.z.w == 0 &&
				matrix.pos.x == 0 && matrix.pos.y == 0 && matrix.pos.z == 0 && matrix.pos.w == 1;
		};

		// A cube map orientation may also be a mirror (see TextureCube::m_cube2w), so its handedness is normalised before the rotation check
		auto valid_cube_orientation = [&](m4x4 const& matrix)
		{
			auto rotation = matrix;
			if (Triple(rotation.x, rotation.y, rotation.z) < 0)
				rotation.z = -rotation.z;

			return valid_rotation(rotation);
		};
		if (!valid_rotation(world_to_sky) || !valid_rotation(world_to_background) ||
			(background && (!valid_cube_orientation(background->m_cube2w) || &background->rdr() != &m_shader->rdr())))
			throw std::invalid_argument("Sky direction transforms must be finite rotations, the cubemap orientation must be orthonormal, and the cubemap must belong to this renderer");

		// Retain the native texture independently of the caller's wrapper; update only constants during a fade.
		m_shader->m_cbuf.blend_weight = weight;
		m_shader->m_cbuf.world_to_sky = world_to_sky;
		m_shader->m_cbuf.world_to_cube = background ? Transpose3x3(background->m_cube2w) * world_to_background : m4x4::Identity();
		m_shader->m_background = std::move(background);
	}
}
