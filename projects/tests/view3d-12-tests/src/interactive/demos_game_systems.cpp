//*********************************************
// View3d-12 Tests
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Demos of the game-systems library: GPU-resident particles.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <format>
#include <ranges>
#include <stdexcept>
#include <string>
#include <vector>
#include "pr/common/flags_enum.h"
#include "pr/hlsl/interop.h"
#include "pr/gui/wingui.h"
#include "pr/view3d-12/view3d-dll.h"
#include "pr/game-systems/particles/particle_sprite.hlsli"
#include "pr/game-systems/particles/particle_stepped_sprite.hlsli"
#include "pr/game-systems/particles/particle_mesh.hlsli"
#include "pr/game-systems/particles/particle_debris.hlsli"
#include "pr/physics/terrain/water/water_field_types.hlsli"
#include "../game-systems/shaders/hlsl/particle_chip.hlsli"
#include "demo.h"
#include "controls_panel.h"

namespace view3d_test
{
	namespace
	{
		namespace api = pr::view3d;
		using ParticleSpriteEmitter = pr::game_systems::particles::ParticleSpriteEmitter;
		using ParticleSpriteConstants = pr::game_systems::particles::ParticleSpriteConstants;

		namespace compiled
		{
			// The shaders of the test chip type. See particle_chip.hlsli.
			#ifdef NDEBUG
			#include "../game-systems/shaders/hlsl/compiled/release/particle_chip_emit_cs.h"
			#include "../game-systems/shaders/hlsl/compiled/release/particle_chip_update_cs.h"
			#include "../game-systems/shaders/hlsl/compiled/release/particle_chip_vs.h"
			#else
			#include "../game-systems/shaders/hlsl/compiled/debug/particle_chip_emit_cs.h"
			#include "../game-systems/shaders/hlsl/compiled/debug/particle_chip_update_cs.h"
			#include "../game-systems/shaders/hlsl/compiled/debug/particle_chip_vs.h"
			#endif
		}

		// Sprite particle effects that all share one particle type, so every sprite draws in one draw call. Wood chips are a caller-defined
		// type with its own render record and vertex shader, drawn opaque by a second batch. The time scale shows that particles are exact in
		// slow motion and stop completely when paused. The stress grid adds many emitters to fill the pool.
		struct ParticlesDemo :IDemo
		{
			static constexpr int Capacity = 1 << 18;
			static constexpr int MaxEmitters = 2048;
			static constexpr int MaxGridSide = 32;
			static constexpr float GridRate = 100.0f;
			static constexpr int ChipCapacity = 4096;
			static constexpr int LeafCapacity = 1024;
			static constexpr int SteppedCapacity = 1 << 14;
			static constexpr float WallX = 20.0f;

			// One continuous effect: its emitter id, its parameters, and its rate before the rate scale.
			struct Effect
			{
				uint64_t m_id;
				ParticleSpriteEmitter m_params;
				float m_rate;
			};

			DemoContext m_ctx;
			demo::SceneObjects m_scene;
			api::ParticleSystem m_system;
			int m_type;
			std::vector<Effect> m_effects;
			std::vector<uint64_t> m_grid;
			uint64_t m_burst;
			int m_chip_type;
			uint64_t m_chips;
			int m_stepped_type;
			uint64_t m_stepped_sparks;
			pr::gui::Label* m_stats_label;
			double m_stats_timer;
			float m_time_scale;
			float m_rate_scale;
			float m_wind;

			explicit ParticlesDemo(DemoContext const& ctx)
				: m_ctx(ctx)
				, m_scene(ctx.m_window)
				, m_system()
				, m_type(-1)
				, m_effects()
				, m_grid()
				, m_burst()
				, m_chip_type(-1)
				, m_chips()
				, m_stepped_type(-1)
				, m_stepped_sparks()
				, m_stats_label()
				, m_stats_timer()
				, m_time_scale(1.0f)
				, m_rate_scale(1.0f)
				, m_wind(2.0f)
			{
				// A floor for reference, viewed from the side, and a wall that stepped sparks bounce off.
				m_scene.Add("*Plane floor FF404850 { *Data {60 60} *AxisId {+3} }");
				m_scene.Add(std::format("*Box wall FF607080 {{ *Data {{0.4 24 6}} *o2w {{*pos {{{} 0 3}}}} }}", WallX + 0.2f).c_str());
				View3D_CameraPositionSet(ctx.m_window, { 0, -28, 9, 1 }, { 0, 0, 4, 1 }, { 0, 0, 1, 0 });

				// One sprite type for every effect, drawn by one batch. The fade keeps births and deaths soft.
				m_system = View3D_ParticleSystemCreate("DemoParticles");
				if (m_system == nullptr)
					throw std::runtime_error("Particle system creation failed");

				auto sim = api::ParticleSimShaders{};
				auto draw = api::ParticleDrawShaders{};
				if (!View3D_ParticleSpriteShaders(&sim, &draw))
					throw std::runtime_error("Particle sprite shaders unavailable");

				// Weighted transparency blends overlapping soft sprites, such as smoke, smoothly in any order. The K-buffer keeps only the nearest layers.
				auto batch_options = api::ParticleBatchOptions
				{
					.m_struct_size = sizeof(api::ParticleBatchOptions),
					.m_version = api::ParticleBatchOptions::CurrentVersion,
					.m_shaders = draw,
					.m_capacity = Capacity + SteppedCapacity,
					.m_record_stride = sizeof(pr::game_systems::particles::ParticleSpriteRecord),
					.m_constants = nullptr,
					.m_constants_size = 0,
					.m_blend = api::EParticleBlend::Weighted,
					.m_bbox = api::BBox{ {0, 0, 20, 1}, {60, 60, 30, 0} },
				};
				auto batch = -1;
				auto object = api::Object{};
				if (!View3D_ParticleBatchCreate(m_system, batch_options, "DemoSprites", &batch, &object))
					throw std::runtime_error("Particle batch creation failed");

				m_scene.Add(object);

				// Fade sprites out where they cut through the ground, so smoke and dust show no hard intersection lines.
				View3D_ObjectSoftDepthSet(object, 1.0f, nullptr);

				// Sprites fade in over the first 5% of their lives and out over the last 30%.
				auto constants = ParticleSpriteConstants
				{
					.key_count = 4,
					.keys = { { 0.0f, 1.0f, 0x00FFFFFF }, { 0.05f, 1.0f, 0xFFFFFFFF }, { 0.7f, 1.0f, 0xFFFFFFFF }, { 1.0f, 1.0f, 0x00FFFFFF } },
				};
				auto options = api::ParticleTypeOptions
				{
					.m_struct_size = sizeof(api::ParticleTypeOptions),
					.m_version = api::ParticleTypeOptions::CurrentVersion,
					.m_shaders = sim,
					.m_batch = batch,
					.m_capacity = Capacity,
					.m_max_emitters = MaxEmitters,
					.m_state_stride = sizeof(pr::game_systems::particles::ParticleSpriteState),
					.m_record_stride = sizeof(pr::game_systems::particles::ParticleSpriteRecord),
					.m_params_stride = sizeof(ParticleSpriteEmitter),
					.m_constants = &constants,
					.m_constants_size = sizeof(constants),
				};
				if (!View3D_ParticleTypeCreate(m_system, options, "DemoSprites", &m_type))
					throw std::runtime_error("Particle type creation failed");

				// Sparks: a fountain thrown up and pulled down by gravity.
				AddEffect(600.0f, Params({ -8, 0, 0.2f, 0.1f }, { 0, 0, 9, 3 }, { 0, 0, -9.8f }, { 1.2f, 2.0f }, { 0.15f, 0.05f }, 0xFFFFD040, 0x00FF4000));

				// Smoke: slow puffs that drift downwind and grow.
				AddEffect(30.0f, Params({ 0, 0, 0.5f, 0.5f }, { 0.6f, 0, 1.5f, 0.4f }, { 0.3f, 0, 0.2f }, { 4.0f, 6.0f }, { 0.6f, 3.0f }, 0x80606060, 0x00909090));

				// Fire: short-lived flames that rise and shrink.
				AddEffect(300.0f, Params({ 8, 0, 0.3f, 0.6f }, { 0, 0, 2, 0.6f }, { 0, 0, 1.5f }, { 0.5f, 1.0f }, { 0.8f, 0.2f }, 0xFFFFC040, 0x00C02000));

				// Bursts: an explosion that only emits when asked.
				m_burst = Create(0.0f, Params({ 0, 0, 5, 0.2f }, { 0, 0, 0, 8 }, { 0, 0, -9.8f }, { 1.0f, 2.5f }, { 0.25f, 0.1f }, 0xFFFFFFFF, 0x00FF8000));

				// Wood chips: tumbling opaque boxes thrown from a chopped tree.
				CreateChips();

				// Leaves: textured cards tumbling down from a canopy, drawn by the stock mesh batch.
				CreateLeaves();

				// Stepped sprites: sparks that bounce off the floor and wall, and snow carried by the wind. They share the sprite batch.
				CreateStepped(batch);

				// Demo controls.
				auto& ui = ctx.m_controls;
				ui.AddLabel(L"Sprites are one particle type in one batch, drawn with one draw call. Wood chips are a custom type with their own record and vertex shader, in a second, opaque batch. Leaves reuse the chip simulation, drawn as textured cards by the stock mesh batch. Stepped sprites (bouncing sparks and snow) share the sprite batch, and move through the environment's gravity and wind, bouncing off its planes. Particle state lives on the GPU.");
				ui.AddSlider(L"Time scale (0 = pause)", 0, 2, m_time_scale, [this](float v)
				{
					m_time_scale = v;
				}, 200);
				ui.AddSlider(L"Effect rate scale", 0, 4, m_rate_scale, [this](float v)
				{
					m_rate_scale = v;
					for (auto const& effect : m_effects)
						Update(effect.m_id, effect.m_rate * m_rate_scale, effect.m_params);
				}, 40);
				ui.AddCheckBox(L"Weighted transparency (off = K-buffer)", true, [object](bool on)
				{
					View3D_ObjectNuggetFlagsSet(object, api::ENuggetFlag::AlphaWeighted, on ? TRUE : FALSE, nullptr, 0);
				});
				ui.AddButton(L"Burst 5000", [this]
				{
					View3D_ParticleEmitterBurst(m_system, m_burst, 5000);
				});
				ui.AddButton(L"Chop (60 wood chips)", [this]
				{
					View3D_ParticleEmitterBurst(m_system, m_chips, 60);
				});
				ui.AddButton(L"Burst 500 bouncing sparks", [this]
				{
					View3D_ParticleEmitterBurst(m_system, m_stepped_sparks, 500);
				});
				ui.AddSlider(L"Stress grid side", 0, MaxGridSide, 0, [this](float v)
				{
					SetGridSide(static_cast<int>(std::lround(v)));
				}, MaxGridSide);
				ui.AddSlider(L"Wind (m/s along x)", -10, 10, m_wind, [this](float v)
				{
					m_wind = v;
					SetEnvironment();
				}, 40);
				ui.AddCheckBox(L"Gusty wind grid", false, [this](bool on)
				{
					SetWindGrid(on);
				});
				ui.AddButton(L"Reset", [this]
				{
					View3D_ParticleSystemReset(m_system);
				});
				m_stats_label = ui.AddLabel(L"\n\n\n\n\n");
			}
			~ParticlesDemo()
			{
				// The batch's object stays in the scene until the scene is destroyed, and keeps its own references to the particle buffers.
				View3D_ParticleSystemDestroy(m_system);
			}

			// Return sprite emitter parameters.
			static ParticleSpriteEmitter Params(api::Vec4 position, api::Vec4 velocity, api::Vec4 accel, api::Vec2 lifetime, api::Vec2 size, uint32_t colour0, uint32_t colour1)
			{
				// Colours are 0xAARRGGBB.
				auto params = ParticleSpriteEmitter{};
				params.position = { position.x, position.y, position.z, position.w };
				params.velocity = { velocity.x, velocity.y, velocity.z, velocity.w };
				params.accel = { accel.x, accel.y, accel.z, 0.0f };
				params.lifetime = { lifetime.x, lifetime.y };
				params.size = { size.x, size.y };
				params.colour0 = colour0;
				params.colour1 = colour1;
				return params;
			}

			// Create an emitter with 'rate' and 'params'.
			uint64_t Create(float rate, ParticleSpriteEmitter const& params, uint32_t seed = 1)
			{
				// Errors are reported through the error callback, and the emitter is then unusable.
				auto settings = api::ParticleEmitter{ .m_rate = rate, .m_seed = seed, .m_active = TRUE };
				auto id = uint64_t{};
				if (!View3D_ParticleEmitterCreate(m_system, m_type, settings, &params, sizeof(params), &id))
					throw std::runtime_error("Particle emitter creation failed");

				return id;
			}

			// Replace the rate and parameters of emitter 'id'.
			void Update(uint64_t id, float rate, ParticleSpriteEmitter const& params)
			{
				// Rates only change, so the seed is unchanged.
				auto settings = api::ParticleEmitter{ .m_rate = rate, .m_seed = 1, .m_active = TRUE };
				View3D_ParticleEmitterUpdate(m_system, id, settings, &params, sizeof(params));
			}

			// Add the wood chip batch, type and emitter.
			void CreateChips()
			{
				// The chip vertex shader draws a box per record. The batch constants set the chips' proportions.
				using namespace game_systems_tests;
				static constexpr auto indices = ParticleChipIndices();
				auto draw_constants = ParticleChipDrawConstants{ .width = 0.4f, .thickness = 0.12f };
				auto batch_options = api::ParticleBatchOptions
				{
					.m_struct_size = sizeof(api::ParticleBatchOptions),
					.m_version = api::ParticleBatchOptions::CurrentVersion,
					.m_shaders =
					{
						.m_vertex = { compiled::particle_chip_vs, sizeof(compiled::particle_chip_vs) },
						.m_indices = indices.data(),
						.m_index_count = static_cast<int>(indices.size()),
						.m_vertex_count = static_cast<int>(ParticleChipVertexCount),
						.m_geom = api::EGeom::Vert | api::EGeom::Colr | api::EGeom::Norm,
					},
					.m_capacity = ChipCapacity,
					.m_record_stride = sizeof(ParticleChipRecord),
					.m_constants = &draw_constants,
					.m_constants_size = sizeof(draw_constants),
					.m_blend = api::EParticleBlend::Opaque,
					.m_bbox = api::BBox{ {0, 0, 20, 1}, {60, 60, 30, 0} },
				};
				auto batch = -1;
				auto object = api::Object{};
				if (!View3D_ParticleBatchCreate(m_system, batch_options, "DemoChips", &batch, &object))
					throw std::runtime_error("Particle chip batch creation failed");

				m_scene.Add(object);

				// Chips fall under gravity. Without collision they fall through the floor, and die soon after.
				auto constants = ParticleChipConstants{ .gravity = { 0, 0, -9.8f, 0 } };
				auto options = api::ParticleTypeOptions
				{
					.m_struct_size = sizeof(api::ParticleTypeOptions),
					.m_version = api::ParticleTypeOptions::CurrentVersion,
					.m_shaders =
					{
						.m_emit = { compiled::particle_chip_emit_cs, sizeof(compiled::particle_chip_emit_cs) },
						.m_update = { compiled::particle_chip_update_cs, sizeof(compiled::particle_chip_update_cs) },
					},
					.m_batch = batch,
					.m_capacity = ChipCapacity,
					.m_max_emitters = 4,
					.m_state_stride = sizeof(ParticleChipState),
					.m_record_stride = sizeof(ParticleChipRecord),
					.m_params_stride = sizeof(ParticleChipEmitter),
					.m_constants = &constants,
					.m_constants_size = sizeof(constants),
				};
				if (!View3D_ParticleTypeCreate(m_system, options, "DemoChips", &m_chip_type))
					throw std::runtime_error("Particle chip type creation failed");

				// Chips fly out sideways and up from the side of a trunk, only when chopped.
				auto params = ParticleChipEmitter
				{
					.position = { 4.0f, 3.0f, 1.0f, 0.25f },
					.velocity = { -1.5f, -1.0f, 3.5f, 2.0f },
					.spacing = 0.0f,
					.spin = 12.0f,
					.lifetime = 1.5f,
					.colour = 0xFFC8A060,
				};
				auto settings = api::ParticleEmitter{ .m_rate = 0.0f, .m_seed = 7, .m_active = TRUE };
				if (!View3D_ParticleEmitterCreate(m_system, m_chip_type, settings, &params, sizeof(params), &m_chips))
					throw std::runtime_error("Particle chip emitter creation failed");
			}

			// Add the falling leaves: the chip simulation drawn by the stock mesh batch as textured, two-sided leaf cards.
			void CreateLeaves()
			{
				// A leaf card in the xy plane, with a back face of reversed winding so tumbling leaves are seen from both sides.
				using namespace game_systems_tests;
				using ParticleMeshVertex = pr::game_systems::particles::ParticleMeshVertex;
				static constexpr auto card = std::array<ParticleMeshVertex, 8>
				{{
					{ .pos = { -0.5f, -0.5f, 0.0f }, .u = 0.0f, .norm = { 0.0f, 0.0f, +1.0f }, .v = 1.0f },
					{ .pos = { +0.5f, -0.5f, 0.0f }, .u = 1.0f, .norm = { 0.0f, 0.0f, +1.0f }, .v = 1.0f },
					{ .pos = { +0.5f, +0.5f, 0.0f }, .u = 1.0f, .norm = { 0.0f, 0.0f, +1.0f }, .v = 0.0f },
					{ .pos = { -0.5f, +0.5f, 0.0f }, .u = 0.0f, .norm = { 0.0f, 0.0f, +1.0f }, .v = 0.0f },
					{ .pos = { -0.5f, -0.5f, 0.0f }, .u = 0.0f, .norm = { 0.0f, 0.0f, -1.0f }, .v = 1.0f },
					{ .pos = { +0.5f, -0.5f, 0.0f }, .u = 1.0f, .norm = { 0.0f, 0.0f, -1.0f }, .v = 1.0f },
					{ .pos = { +0.5f, +0.5f, 0.0f }, .u = 1.0f, .norm = { 0.0f, 0.0f, -1.0f }, .v = 0.0f },
					{ .pos = { -0.5f, +0.5f, 0.0f }, .u = 0.0f, .norm = { 0.0f, 0.0f, -1.0f }, .v = 0.0f },
				}};
				static constexpr auto card_indices = std::array<uint32_t, 12>{ 0, 1, 2, 0, 2, 3, 4, 6, 5, 4, 7, 6 };

				// A white leaf shape in the alpha channel: an ellipse pointed at both ends of its diagonal. The opaque batch cuts out alpha below one half.
				constexpr int LeafTexSize = 32;
				auto texels = std::vector<uint32_t>(LeafTexSize * LeafTexSize);
				for (auto y = 0; y != LeafTexSize; ++y)
				{
					for (auto x = 0; x != LeafTexSize; ++x)
					{
						// Distance from the centre in coordinates along and across the diagonal, with the width narrowing towards the tips.
						auto u = (x + 0.5f) / LeafTexSize * 2.0f - 1.0f;
						auto v = (y + 0.5f) / LeafTexSize * 2.0f - 1.0f;
						auto along = (u + v) * 0.7071f;
						auto across = (u - v) * 0.7071f;
						auto half_width = 0.45f * (1.0f - along * along);
						auto inside = std::abs(along) < 0.95f && std::abs(across) < half_width;
						texels[y * LeafTexSize + x] = inside ? 0xFFFFFFFFU : 0x00FFFFFFU;
					}
				}
				auto texture_options = api::TextureOptions{};
				texture_options.m_format = DXGI_FORMAT_R8G8B8A8_UNORM;
				texture_options.m_resource_state = D3D12_RESOURCE_STATE_ALL_SHADER_RESOURCE;
				texture_options.m_mips = 1;
				texture_options.m_multisamp = { 1, 0 };
				texture_options.m_t2s = { {1,0,0,0}, {0,1,0,0}, {0,0,1,0}, {0,0,0,1} };
				texture_options.m_has_alpha = TRUE;
				texture_options.m_dbg_name = "DemoLeaf";
				auto texture = api::TexturePtr(View3D_TextureCreate(LeafTexSize, LeafTexSize, texels.data(), texels.size() * sizeof(uint32_t), texture_options));
				if (texture == nullptr)
					throw std::runtime_error("Leaf texture creation failed");

				// The stock mesh draw with the leaf card as its shared mesh. The batch keeps its own reference to the texture.
				auto draw = api::ParticleDrawShaders{};
				if (!View3D_ParticleMeshShaders(&draw))
					throw std::runtime_error("Particle mesh shaders unavailable");

				draw.m_indices = card_indices.data();
				draw.m_index_count = static_cast<int>(card_indices.size());
				draw.m_vertex_count = static_cast<int>(card.size());
				draw.m_vertex_data = card.data();
				auto batch_options = api::ParticleBatchOptions
				{
					.m_struct_size = sizeof(api::ParticleBatchOptions),
					.m_version = api::ParticleBatchOptions::CurrentVersion,
					.m_shaders = draw,
					.m_capacity = LeafCapacity,
					.m_record_stride = sizeof(ParticleChipRecord),
					.m_constants = nullptr,
					.m_constants_size = 0,
					.m_blend = api::EParticleBlend::Opaque,
					.m_bbox = api::BBox{ {0, 0, 20, 1}, {60, 60, 30, 0} },
					.m_texture = texture.get(),
					.m_sampler = View3D_SamplerCreateStock(api::EStockSampler::LinearClamp),
				};
				auto batch = -1;
				auto object = api::Object{};
				if (!View3D_ParticleBatchCreate(m_system, batch_options, "DemoLeaves", &batch, &object))
					throw std::runtime_error("Particle leaf batch creation failed");

				m_scene.Add(object);

				// Leaves drift down slowly. The chip simulation writes the mesh record layout, so its records are drawn as leaf cards.
				auto constants = ParticleChipConstants{ .gravity = { 0, 0, -0.6f, 0 } };
				auto options = api::ParticleTypeOptions
				{
					.m_struct_size = sizeof(api::ParticleTypeOptions),
					.m_version = api::ParticleTypeOptions::CurrentVersion,
					.m_shaders =
					{
						.m_emit = { compiled::particle_chip_emit_cs, sizeof(compiled::particle_chip_emit_cs) },
						.m_update = { compiled::particle_chip_update_cs, sizeof(compiled::particle_chip_update_cs) },
					},
					.m_batch = batch,
					.m_capacity = LeafCapacity,
					.m_max_emitters = 4,
					.m_state_stride = sizeof(ParticleChipState),
					.m_record_stride = sizeof(ParticleChipRecord),
					.m_params_stride = sizeof(ParticleChipEmitter),
					.m_constants = &constants,
					.m_constants_size = sizeof(constants),
				};
				auto type = -1;
				if (!View3D_ParticleTypeCreate(m_system, options, "DemoLeaves", &type))
					throw std::runtime_error("Particle leaf type creation failed");

				// Leaves fall continuously from a canopy, drifting downwind and tumbling.
				auto params = ParticleChipEmitter
				{
					.position = { -4.0f, 8.0f, 9.0f, 0.5f },
					.velocity = { 0.6f, 0.0f, 0.0f, 0.8f },
					.spacing = 0.0f,
					.spin = 2.5f,
					.lifetime = 14.0f,
					.colour = 0xFF70A030,
				};
				auto settings = api::ParticleEmitter{ .m_rate = 12.0f, .m_seed = 11, .m_active = TRUE };
				auto id = uint64_t{};
				if (!View3D_ParticleEmitterCreate(m_system, type, settings, &params, sizeof(params), &id))
					throw std::runtime_error("Particle leaf emitter creation failed");
			}

			// Add a continuous effect.
			void AddEffect(float rate, ParticleSpriteEmitter const& params)
			{
				// Effects keep their unscaled rate so the rate scale can be changed freely.
				m_effects.push_back(Effect{ .m_id = Create(rate * m_rate_scale, params), .m_params = params, .m_rate = rate });
			}

			// Add the stepped sprite type to the sprite 'batch', with bouncing sparks and wind-blown snow.
			void CreateStepped(int batch)
			{
				// Stepped sprites write the sprite record, so they share the sprite batch and its single draw call.
				using namespace pr::game_systems::particles;
				auto sim = api::ParticleSimShaders{};
				auto draw = api::ParticleDrawShaders{};
				if (!View3D_ParticleSteppedSpriteShaders(&sim, &draw))
					throw std::runtime_error("Particle stepped sprite shaders unavailable");

				auto constants = ParticleSpriteConstants{};
				auto options = api::ParticleTypeOptions
				{
					.m_struct_size = sizeof(api::ParticleTypeOptions),
					.m_version = api::ParticleTypeOptions::CurrentVersion,
					.m_shaders = sim,
					.m_batch = batch,
					.m_capacity = SteppedCapacity,
					.m_max_emitters = 4,
					.m_state_stride = sizeof(ParticleSteppedSpriteState),
					.m_record_stride = sizeof(ParticleSpriteRecord),
					.m_params_stride = sizeof(ParticleSteppedSpriteEmitter),
					.m_constants = &constants,
					.m_constants_size = sizeof(constants),
				};
				if (!View3D_ParticleTypeCreate(m_system, options, "DemoStepped", &m_stepped_type))
					throw std::runtime_error("Particle stepped type creation failed");

				// The floor, and the face of the wall, keep particles in z >= 0 and x <= WallX.
				auto planes = std::array<api::ParticlePlane, 2>
				{{
					{ .m_plane = { 0, 0, +1, 0 }, .m_restitution = 0.45f, .m_damping = 0.15f },
					{ .m_plane = { -1, 0, 0, -WallX }, .m_restitution = 0.6f, .m_damping = 0.1f },
				}};
				auto collision_set = -1;
				if (!View3D_ParticleCollisionSetCreate(m_system, planes.data(), static_cast<int>(planes.size()), &collision_set))
					throw std::runtime_error("Particle collision set creation failed");

				SetEnvironment();

				// Sparks: thrown towards the wall, falling under gravity and bouncing off the wall and the floor. No drag, so the wind does not move them.
				auto sparks = ParticleSteppedSpriteEmitter
				{
					.position = { 14.0f, -4.0f, 3.0f, 0.1f },
					.velocity = { 6.0f, 0.0f, 5.0f, 2.5f },
					.accel = { 0.0f, 0.0f, 0.0f, 0.0f },
					.lifetime = { 2.5f, 4.0f },
					.size = { 0.15f, 0.05f },
					.colour0 = 0xFFFFF0A0,
					.colour1 = 0x40FF4000,
					.radius = 0.05f,
					.collision_set = collision_set,
					.gravity = 1.0f,
				};
				auto spark_settings = api::ParticleEmitter{ .m_rate = 120.0f, .m_seed = 13, .m_active = TRUE };
				if (!View3D_ParticleEmitterCreate(m_system, m_stepped_type, spark_settings, &sparks, sizeof(sparks), &m_stepped_sparks))
					throw std::runtime_error("Particle spark emitter creation failed");

				// Snow: an upward acceleration cancels most of gravity, and strong drag pulls the flakes to the wind's speed, so they drift with it.
				// Flakes stop dead on the floor and lie there until they die.
				auto snow = ParticleSteppedSpriteEmitter
				{
					.position = { -6.0f, 6.0f, 16.0f, 12.0f },
					.velocity = { 0.0f, 0.0f, -1.0f, 0.3f },
					.accel = { 0.0f, 0.0f, 8.8f, 1.0f },
					.lifetime = { 16.0f, 20.0f },
					.size = { 0.12f, 0.12f },
					.colour0 = 0xFFFFFFFF,
					.colour1 = 0x60FFFFFF,
					.radius = 0.06f,
					.collision_set = collision_set,
					.gravity = 1.0f,
				};
				auto snow_settings = api::ParticleEmitter{ .m_rate = 250.0f, .m_seed = 17, .m_active = TRUE };
				auto snow_id = uint64_t{};
				if (!View3D_ParticleEmitterCreate(m_system, m_stepped_type, snow_settings, &snow, sizeof(snow), &snow_id))
					throw std::runtime_error("Particle snow emitter creation failed");
			}

			// Set gravity and the uniform wind for the stepped particles.
			void SetEnvironment()
			{
				// Four contacts per step is enough for a particle in the corner between the floor and the wall.
				auto environment = api::ParticleEnvironment
				{
					.m_gravity = { 0, 0, -9.8f, 0 },
					.m_wind = { m_wind, 0, 0, 0 },
					.m_collision_iterations = 4,
				};
				View3D_ParticleEnvironmentSet(m_system, environment);
			}

			// Add or remove a gusty wind grid over the scene. Its wind adds to the uniform wind.
			void SetWindGrid(bool on)
			{
				// A 4 x 4 grid of 15 m cells over the floor, with gusts that cross the scene in alternating directions.
				if (!on)
				{
					View3D_ParticleWindGridSet(m_system, { 0, 0 }, 1.0f, 0, 0, nullptr);
					return;
				}

				auto wind = std::array<api::Vec2, 16>{};
				for (auto j = 0; j != 4; ++j)
				{
					// Each row blows the opposite way along y to its neighbours, and the middle columns blow harder along x.
					for (auto i = 0; i != 4; ++i)
						wind[j * 4 + i] = { (i == 1 || i == 2) ? 4.0f : -2.0f, (j % 2) ? 5.0f : -5.0f };
				}
				View3D_ParticleWindGridSet(m_system, { -30.0f, -30.0f }, 15.0f, 4, 4, wind.data());
			}

			// Show a square grid of 'side' x 'side' small emitters above the floor.
			void SetGridSide(int side)
			{
				// Rebuild the grid, so each emitter's colour and position follow from its place in the grid.
				for (auto id : m_grid)
					View3D_ParticleEmitterDelete(m_system, id);

				m_grid.clear();
				for (auto j = 0; j != side; ++j)
				{
					for (auto i = 0; i != side; ++i)
					{
						// Emitters spread over a 40 m square, each a small blue fountain.
						auto x = side > 1 ? -20.0f + 40.0f * i / (side - 1) : 0.0f;
						auto y = side > 1 ? -20.0f + 40.0f * j / (side - 1) + 15.0f : 15.0f;
						auto params = Params({ x, y, 0.1f, 0.05f }, { 0, 0, 3, 1 }, { 0, 0, -2.0f }, { 2.0f, 3.0f }, { 0.12f, 0.06f }, 0xFF40A0FF, 0x0020FFC0);
						m_grid.push_back(Create(GridRate, params, static_cast<uint32_t>(m_grid.size() + 2)));
					}
				}
			}

			// Advance the particles by the scaled frame time, and refresh the statistics twice a second.
			void Step(double dt) override
			{
				// Long frames are clamped so that a stall does not emit a flood of particles.
				View3D_ParticleSystemStep(m_system, std::min(dt, 0.1) * m_time_scale);

				// Reading the statistics waits for the GPU, so it is throttled.
				m_stats_timer += dt;
				if (m_stats_timer < 0.5)
					return;

				m_stats_timer = 0;
				auto stats = api::ParticleStats{};
				auto stepped = api::ParticleStats{};
				if (!View3D_ParticleStats(m_system, m_type, &stats) || !View3D_ParticleStats(m_system, m_stepped_type, &stepped))
					return;

				m_stats_label->Text(std::format(
					L"Live: {} of {}\nFree: {}\nEmitters: {}\nRequested: {}, dropped: {}\nStepped live: {} of {}, contacts: {}, overlaps: {}, exhausted: {}",
					stats.m_live, Capacity, stats.m_free, m_effects.size() + 1 + m_grid.size(), stats.m_requested, stats.m_dropped,
					stepped.m_live, SteppedCapacity, stepped.m_contacts, stepped.m_overlaps, stepped.m_exhausted).c_str());
			}
		};

		// A gallery of the stock particle effects, in a row along x. Each effect has its own batch and type, built only from its preset.
		struct ParticlePresetsDemo :IDemo
		{
			static constexpr int Capacity = 8192;
			static constexpr float Spacing = 8.0f;

			// One effect of the gallery: its preset, its type, its emitter, and the emitter's parameters.
			struct Effect
			{
				api::EParticlePreset m_preset;
				api::ParticlePreset m_settings;
				int m_type;
				uint64_t m_emitter;
				std::vector<uint8_t> m_params;
			};

			DemoContext m_ctx;
			demo::SceneObjects m_scene;
			api::ParticleSystem m_system;
			std::vector<Effect> m_effects;
			pr::gui::Label* m_stats_label;
			double m_stats_timer;
			float m_time_scale;
			float m_rate_scale;
			float m_wind;

			explicit ParticlePresetsDemo(DemoContext const& ctx)
				: m_ctx(ctx)
				, m_scene(ctx.m_window)
				, m_system()
				, m_effects()
				, m_stats_label()
				, m_stats_timer()
				, m_time_scale(1.0f)
				, m_rate_scale(1.0f)
				, m_wind(2.0f)
			{
				// A floor for reference, viewed from the side, with a post under each effect.
				m_scene.Add("*Plane floor FF404850 { *Data {100 60} *AxisId {+3} }");
				View3D_CameraPositionSet(ctx.m_window, { 0, -40, 10, 1 }, { 0, 0, 5, 1 }, { 0, 0, 1, 0 });

				m_system = View3D_ParticleSystemCreate("DemoPresets");
				if (m_system == nullptr)
					throw std::runtime_error("Particle system creation failed");

				// The floor keeps particles in z >= 0. Presets collide with nothing until given a collision set.
				auto floor = api::ParticlePlane{ .m_plane = { 0, 0, +1, 0 }, .m_restitution = 0.3f, .m_damping = 0.3f };
				auto collision_set = -1;
				if (!View3D_ParticleCollisionSetCreate(m_system, &floor, 1, &collision_set))
					throw std::runtime_error("Particle collision set creation failed");

				SetEnvironment();

				// The effects, left to right. Rain and snow fall over a smaller area than their presets so they stay in their own place.
				constexpr api::EParticlePreset presets[] =
				{
					api::EParticlePreset::Fire, api::EParticlePreset::Sparks, api::EParticlePreset::Smoke, api::EParticlePreset::Dust, api::EParticlePreset::Rain,
					api::EParticlePreset::Snow, api::EParticlePreset::Leaves, api::EParticlePreset::WoodChips, api::EParticlePreset::Marker, api::EParticlePreset::Birds,
				};
				for (auto preset : presets)
				{
					// Each effect sits above its own place on the floor.
					auto x = (static_cast<float>(m_effects.size()) - 4.5f) * Spacing;
					auto pos = api::Vec4{ x, 0.0f, 0.1f, 0.0f };
					switch (preset)
					{
						case api::EParticlePreset::Rain:
						case api::EParticlePreset::Snow:
						{
							pos = { x, 0.0f, 12.0f, 3.0f };
							break;
						}
						case api::EParticlePreset::Leaves:
						{
							pos = { x, 0.0f, 8.0f, 1.0f };
							break;
						}
						case api::EParticlePreset::Marker:
						{
							pos = { x, 0.0f, 0.5f, 0.0f };
							break;
						}
						case api::EParticlePreset::Birds:
						{
							pos = { x, 10.0f, 20.0f, 5.0f };
							break;
						}
						default:
						{
							pos.w = -1.0f;
							break;
						}
					}
					m_effects.push_back(CreateEffect(preset, pos, collision_set));
				}

				// Demo controls.
				auto& ui = ctx.m_controls;
				ui.AddLabel(L"The stock particle effects, left to right: fire, sparks, smoke, dust, rain, snow, leaves, wood chips, a beacon marker, and a flock of birds. Each is created only from its preset (View3D_ParticlePresetGet), with its position set and a floor to land on.");
				ui.AddSlider(L"Time scale (0 = pause)", 0, 2, m_time_scale, [this](float v)
				{
					m_time_scale = v;
				}, 200);
				ui.AddSlider(L"Effect rate scale", 0, 4, m_rate_scale, [this](float v)
				{
					m_rate_scale = v;
					for (auto const& effect : m_effects)
						UpdateRate(effect);
				}, 40);
				ui.AddSlider(L"Wind (m/s along x)", -10, 10, m_wind, [this](float v)
				{
					m_wind = v;
					SetEnvironment();
				}, 40);
				ui.AddButton(L"Burst all", [this]
				{
					for (auto const& effect : m_effects)
						Burst(effect);
				});
				ui.AddButton(L"Burst sparks", [this]
				{
					Burst(Find(api::EParticlePreset::Sparks));
				});
				ui.AddButton(L"Burst dust", [this]
				{
					Burst(Find(api::EParticlePreset::Dust));
				});
				ui.AddButton(L"Chop (wood chips)", [this]
				{
					Burst(Find(api::EParticlePreset::WoodChips));
				});
				ui.AddButton(L"Reset", [this]
				{
					View3D_ParticleSystemReset(m_system);
				});
				m_stats_label = ui.AddLabel(L"\n\n\n\n\n\n\n\n\n\n");
			}
			~ParticlePresetsDemo()
			{
				// The batches' objects stay in the scene until the scene is destroyed, and keep their own references to the particle buffers.
				View3D_ParticleSystemDestroy(m_system);
			}

			// Create the batch, type and emitter of 'preset' at 'pos'. A negative 'pos.w' keeps the preset's spread of births.
			Effect CreateEffect(api::EParticlePreset preset, api::Vec4 pos, int collision_set)
			{
				// The preset supplies every setting. Only the emitter's position and collision set change.
				using namespace pr::game_systems::particles;
				auto settings = api::ParticlePreset{};
				if (!View3D_ParticlePresetGet(preset, &settings))
					throw std::runtime_error("Particle preset unavailable");

				// Bounds cover the effect's place, including birds circling wide of the flock's centre.
				auto batch_options = api::ParticleBatchOptions
				{
					.m_struct_size = sizeof(api::ParticleBatchOptions),
					.m_version = api::ParticleBatchOptions::CurrentVersion,
					.m_shaders = settings.m_draw,
					.m_capacity = Capacity,
					.m_record_stride = settings.m_record_stride,
					.m_constants = nullptr,
					.m_constants_size = 0,
					.m_blend = settings.m_blend,
					.m_bbox = api::BBox{ {pos.x, pos.y, 15, 1}, {40, 40, 25, 0} },
				};
				auto batch = -1;
				auto object = api::Object{};
				if (!View3D_ParticleBatchCreate(m_system, batch_options, "PresetBatch", &batch, &object))
					throw std::runtime_error("Particle batch creation failed");

				m_scene.Add(object);
				if (settings.m_blend == api::EParticleBlend::Weighted)
					View3D_ObjectSoftDepthSet(object, 1.0f, nullptr);

				auto type_options = api::ParticleTypeOptions
				{
					.m_struct_size = sizeof(api::ParticleTypeOptions),
					.m_version = api::ParticleTypeOptions::CurrentVersion,
					.m_shaders = settings.m_sim,
					.m_batch = batch,
					.m_capacity = Capacity,
					.m_max_emitters = 4,
					.m_state_stride = settings.m_state_stride,
					.m_record_stride = settings.m_record_stride,
					.m_params_stride = settings.m_params_stride,
					.m_constants = settings.m_constants,
					.m_constants_size = settings.m_constants_size,
				};
				auto type = -1;
				if (!View3D_ParticleTypeCreate(m_system, type_options, "PresetType", &type))
					throw std::runtime_error("Particle type creation failed");

				// Every emitter parameter structure starts with its position. Stepped sprites and debris also have a collision set.
				auto params = std::vector<uint8_t>(static_cast<size_t>(settings.m_params_stride));
				std::memcpy(params.data(), settings.m_params, params.size());
				auto& position = *reinterpret_cast<pr::hlsl::float4*>(params.data());
				position = { pos.x, pos.y, pos.z, pos.w >= 0.0f ? pos.w : position.w };
				switch (preset)
				{
					case api::EParticlePreset::Fire:
					case api::EParticlePreset::Sparks:
					case api::EParticlePreset::Smoke:
					case api::EParticlePreset::Dust:
					case api::EParticlePreset::Rain:
					case api::EParticlePreset::Snow:
					{
						reinterpret_cast<ParticleSteppedSpriteEmitter*>(params.data())->collision_set = collision_set;
						break;
					}
					case api::EParticlePreset::Leaves:
					case api::EParticlePreset::WoodChips:
					{
						reinterpret_cast<ParticleDebrisEmitter*>(params.data())->collision_set = collision_set;
						break;
					}
					case api::EParticlePreset::Birds:
					case api::EParticlePreset::Marker:
					{
						break;
					}
					default:
					{
						throw std::invalid_argument("Unknown particle preset");
					}
				}

				auto effect = Effect{ .m_preset = preset, .m_settings = settings, .m_type = type, .m_emitter = 0, .m_params = std::move(params) };
				auto emitter = api::ParticleEmitter{ .m_rate = settings.m_rate * m_rate_scale, .m_seed = static_cast<uint32_t>(m_effects.size() + 1), .m_active = TRUE };
				if (!View3D_ParticleEmitterCreate(m_system, type, emitter, effect.m_params.data(), effect.m_params.size(), &effect.m_emitter))
					throw std::runtime_error("Particle emitter creation failed");

				// Burst effects start with one burst, so the gallery is not empty.
				Burst(effect);
				return effect;
			}

			// Return the effect of 'preset'.
			Effect const& Find(api::EParticlePreset preset) const
			{
				// Every preset has one effect.
				auto iter = std::ranges::find(m_effects, preset, &Effect::m_preset);
				if (iter == m_effects.end())
					throw std::runtime_error("No effect for the preset");

				return *iter;
			}

			// Emit the suggested burst of 'effect'.
			void Burst(Effect const& effect)
			{
				// Continuous effects suggest no burst.
				if (effect.m_settings.m_burst != 0)
					View3D_ParticleEmitterBurst(m_system, effect.m_emitter, static_cast<uint32_t>(effect.m_settings.m_burst));
			}

			// Apply the rate scale to the continuous rate of 'effect'.
			void UpdateRate(Effect const& effect)
			{
				// The parameters are unchanged.
				auto settings = api::ParticleEmitter{ .m_rate = effect.m_settings.m_rate * m_rate_scale, .m_seed = 1, .m_active = TRUE };
				View3D_ParticleEmitterUpdate(m_system, effect.m_emitter, settings, effect.m_params.data(), effect.m_params.size());
			}

			// Set gravity and the uniform wind.
			void SetEnvironment()
			{
				// One contact per step is enough on a flat floor.
				auto environment = api::ParticleEnvironment
				{
					.m_gravity = { 0, 0, -9.8f, 0 },
					.m_wind = { m_wind, 0, 0, 0 },
					.m_collision_iterations = 2,
				};
				View3D_ParticleEnvironmentSet(m_system, environment);
			}

			// Advance the particles by the scaled frame time, and refresh the statistics twice a second.
			void Step(double dt) override
			{
				// Long frames are clamped so that a stall does not emit a flood of particles.
				View3D_ParticleSystemStep(m_system, std::min(dt, 0.1) * m_time_scale);

				// Reading the statistics waits for the GPU, so it is throttled.
				m_stats_timer += dt;
				if (m_stats_timer < 0.5)
					return;

				m_stats_timer = 0;
				auto text = std::wstring{};
				for (auto const& effect : m_effects)
				{
					// One line per effect, in gallery order.
					auto stats = api::ParticleStats{};
					if (!View3D_ParticleStats(m_system, effect.m_type, &stats))
						return;

					text += std::format(L"{}: live {}, contacts {}\n", Name(effect.m_preset), stats.m_live, stats.m_contacts);
				}
				m_stats_label->Text(text.c_str());
			}

			// The display name of 'preset'.
			static wchar_t const* Name(api::EParticlePreset preset)
			{
				// Names match the enum.
				switch (preset)
				{
					case api::EParticlePreset::Fire: return L"Fire";
					case api::EParticlePreset::Sparks: return L"Sparks";
					case api::EParticlePreset::Smoke: return L"Smoke";
					case api::EParticlePreset::Dust: return L"Dust";
					case api::EParticlePreset::Rain: return L"Rain";
					case api::EParticlePreset::Snow: return L"Snow";
					case api::EParticlePreset::Leaves: return L"Leaves";
					case api::EParticlePreset::Birds: return L"Birds";
					case api::EParticlePreset::WoodChips: return L"Wood chips";
					case api::EParticlePreset::Marker: return L"Marker";
					default: throw std::invalid_argument("Unknown particle preset");
				}
			}
		};

		// Spray and froth on a wind-driven ocean. Both presets are born only on breaking crests, which they find by sampling the ocean that the
		// system's water binding names, so they follow the waves without any CPU readback. The ocean is updated before each particle step.
		struct OceanSprayDemo :IDemo
		{
			static constexpr int Capacity = 1 << 15;
			static constexpr float MaxWind = 20.0f;

			DemoContext m_ctx;
			demo::SceneObjects m_scene;
			api::Ocean m_ocean;
			api::Object m_ocean_object;
			api::ParticleSystem m_system;
			int m_spray_type;
			int m_froth_type;
			pr::gui::Label* m_stats_label;
			double m_stats_timer;
			double m_wave_time;
			float m_time_scale;
			float m_wind;

			explicit OceanSprayDemo(DemoContext const& ctx)
				: m_ctx(ctx)
				, m_scene(ctx.m_window)
				, m_ocean()
				, m_ocean_object()
				, m_system()
				, m_spray_type(-1)
				, m_froth_type(-1)
				, m_stats_label()
				, m_stats_timer()
				, m_wave_time(100.0)
				, m_time_scale(1.0f)
				, m_wind(14.0f)
			{
				// Look across the sea from a few metres above it, downwind.
				View3D_CameraPositionSet(ctx.m_window, { -40, -40, 12, 1 }, { 0, 0, 0, 1 }, { 0, 0, 1, 0 });
				CreateOcean();

				// The particles share the ocean's space, and sample its waves through the water binding.
				m_system = View3D_ParticleSystemCreate("DemoOceanSpray");
				if (m_system == nullptr)
					throw std::runtime_error("Particle system creation failed");
				if (!View3D_ParticleWaterSet(m_system, m_ocean, m_ocean_object))
					throw std::runtime_error("Particle water set failed");

				SetEnvironment();
				m_spray_type = CreateEffect(api::EParticlePreset::Spray, 0.5f);
				m_froth_type = CreateEffect(api::EParticlePreset::Froth, 0.0f);

				// Demo controls.
				auto& ui = ctx.m_controls;
				ui.AddLabel(L"Ocean spray and froth, each created only from its preset (View3D_ParticlePresetGet). Both are born only on breaking crests, found on the GPU by sampling the ocean bound with View3D_ParticleWaterSet. Spray dies when it falls back into the sea; froth rides the waves.");
				ui.AddSlider(L"Time scale (0 = pause)", 0, 2, m_time_scale, [this](float v)
				{
					m_time_scale = v;
				}, 200);
				ui.AddSlider(L"Wind (m/s along x)", 0, MaxWind, m_wind, [this](float v)
				{
					m_wind = v;
					SetWind();
					SetEnvironment();
				}, 40);
				ui.AddButton(L"Reset", [this]
				{
					View3D_ParticleSystemReset(m_system);
				});
				m_stats_label = ui.AddLabel(L"\n\n");
			}
			~OceanSprayDemo()
			{
				// The particle system samples the ocean, so it goes first. The scene deletes both objects afterwards.
				View3D_ParticleSystemDestroy(m_system);
				View3D_OceanDestroy(m_ocean);
			}

			// The wind blowing over every node of the ocean's 2 x 2 wind grid.
			std::array<float, 8> Winds() const
			{
				// One level per node.
				return { m_wind, 0.0f, m_wind, 0.0f, m_wind, 0.0f, m_wind, 0.0f };
			}

			// Create a deep ocean of one wind-sea band of wave trains travelling roughly along +x, which grow with the wind up to 'MaxWind'.
			void CreateOcean()
			{
				// Gerstner trains spread about the wind direction, longest first.
				using namespace pr::physics::terrain::water::shared;
				struct Train { float angle, wavelength, phase; };
				constexpr Train trains[] = { {0.0f, 40.0f, 0.0f}, {0.3f, 27.0f, 1.3f}, {-0.35f, 19.0f, 2.9f}, {0.6f, 13.0f, 4.1f}, {-0.5f, 9.0f, 5.3f}, {0.15f, 6.0f, 0.7f} };
				constexpr auto exponent = 2.0f;
				auto elements = std::vector<WaterFieldElement>{};
				auto alignment = 0.0f;
				for (auto const& train : trains)
				{
					// The amplitude and sharpness are bounds; the table scales them with the wind.
					auto e = WaterFieldElement{};
					e.info = { WaterFieldElementGerstnerWave, 1, 0, 0 };
					e.position = { std::cos(train.angle), std::sin(train.angle), train.phase, 0.0f };
					e.wave = { 1.0f, train.wavelength, std::sqrt(9.81f * train.wavelength / 6.2831853f), 0.0f };
					e.timing = { 0.7f, 0.0f, 0.0f, 0.0f };
					elements.push_back(e);
					alignment += std::pow(std::cos(train.angle / 2.0f), 4.0f * exponent);
				}

				// Flat water in still air, and steep, sharp crests at the strongest wind.
				auto const table = std::array<float, 8>{ 0.0f, 0.0f, 0.0f, 0.0f, 1.2f, exponent, 0.7f, alignment };
				auto const winds = Winds();
				auto const depths = std::array<float, 4>{ -200.0f, -200.0f, -200.0f, -200.0f };
				auto detail = std::vector<uint32_t>(api::OceanDetailMapSize * api::OceanDetailMapSize);
				if (!View3D_OceanDetailMapBake(detail.data(), static_cast<int>(detail.size())))
					throw std::runtime_error("Ocean detail map bake failed");

				auto options = api::OceanOptions
				{
					.m_wind_sea_grid = { .m_origin_x_m = -500, .m_origin_y_m = -500, .m_cell_m = 1000, .m_speed_step_mps = MaxWind, .m_width = 2, .m_height = 2, .m_band_count = 1, .m_speed_count = 2, .m_level_count = 1 },
					.m_wind_sea = { .m_elements = elements.data(), .m_table = table.data(), .m_winds = winds.data(), .m_element_count = static_cast<int32_t>(elements.size()) },
					.m_bathymetry_heights = depths.data(),
					.m_bathymetry_origin_x_m = -500,
					.m_bathymetry_origin_y_m = -500,
					.m_bathymetry_cell_m = 1000,
					.m_bathymetry_width = 2,
					.m_bathymetry_height = 2,
					.m_vertices_per_side = 513,
					.m_radius_m = 450,
					.m_half_height_m = 20,
					.m_floor_height_m = -200,
					.m_detail_map = detail.data(),
				};
				m_ocean = View3D_OceanCreate(options, "DemoOcean", &m_ocean_object);
				if (m_ocean == nullptr)
					throw std::runtime_error("Ocean creation failed");

				m_scene.Add(m_ocean_object);

				// Crest foam over deep water, so the foam drawn on the surface matches where spray and froth are born.
				auto foam = api::OceanFoam
				{
					.m_shore_foam_depth_m = 1, .m_edge_softness = 0.1, .m_crest_strength = 1, .m_crest_threshold = 0.5, .m_tile_m = 4, .m_bump_m = 0.01,
					.m_radius_m = 100, .m_fade_m = 10, .m_lip_height_m = 0.5, .m_lip_strength = 0, .m_crest_range = 0.5, .m_trail_time_s = 4, .m_crest_depth_m = 1, .m_churn_mps = 0.1,
				};
				View3D_OceanSetFoam(m_ocean, foam);
				View3D_OceanSetEffects(m_ocean, TRUE, TRUE);
			}

			// Create the batch, type and continuous emitter of 'preset' over the sea in view, and return the type. A positive 'soft_depth'
			// fades the batch's sprites where they meet the water.
			int CreateEffect(api::EParticlePreset preset, float soft_depth)
			{
				// The preset supplies every setting. Only the emitter's area changes.
				auto settings = api::ParticlePreset{};
				if (!View3D_ParticlePresetGet(preset, &settings))
					throw std::runtime_error("Particle preset unavailable");

				auto batch_options = api::ParticleBatchOptions
				{
					.m_struct_size = sizeof(api::ParticleBatchOptions),
					.m_version = api::ParticleBatchOptions::CurrentVersion,
					.m_shaders = settings.m_draw,
					.m_capacity = Capacity,
					.m_record_stride = settings.m_record_stride,
					.m_constants = nullptr,
					.m_constants_size = 0,
					.m_blend = settings.m_blend,
					.m_bbox = api::BBox{ {0, 0, 0, 1}, {100, 100, 20, 0} },
				};
				auto batch = -1;
				auto object = api::Object{};
				if (!View3D_ParticleBatchCreate(m_system, batch_options, "OceanBatch", &batch, &object))
					throw std::runtime_error("Particle batch creation failed");

				m_scene.Add(object);
				if (soft_depth > 0.0f)
					View3D_ObjectSoftDepthSet(object, soft_depth, nullptr);

				auto type_options = api::ParticleTypeOptions
				{
					.m_struct_size = sizeof(api::ParticleTypeOptions),
					.m_version = api::ParticleTypeOptions::CurrentVersion,
					.m_shaders = settings.m_sim,
					.m_batch = batch,
					.m_capacity = Capacity,
					.m_max_emitters = 4,
					.m_state_stride = settings.m_state_stride,
					.m_record_stride = settings.m_record_stride,
					.m_params_stride = settings.m_params_stride,
					.m_constants = settings.m_constants,
					.m_constants_size = settings.m_constants_size,
				};
				auto type = -1;
				if (!View3D_ParticleTypeCreate(m_system, type_options, "OceanType", &type))
					throw std::runtime_error("Particle type creation failed");

				// Both emitters' parameters start with the disc of sea that they search for crests.
				auto params = std::vector<uint8_t>(static_cast<size_t>(settings.m_params_stride));
				std::memcpy(params.data(), settings.m_params, params.size());
				*reinterpret_cast<pr::hlsl::float4*>(params.data()) = { 0.0f, 0.0f, 0.0f, 50.0f };

				auto emitter = api::ParticleEmitter{ .m_rate = settings.m_rate, .m_seed = static_cast<uint32_t>(type + 1), .m_active = TRUE };
				auto id = uint64_t{};
				if (!View3D_ParticleEmitterCreate(m_system, type, emitter, params.data(), params.size(), &id))
					throw std::runtime_error("Particle emitter creation failed");

				return type;
			}

			// Blow the wind over the whole sea. The waves follow it from the next update.
			void SetWind()
			{
				// The elements and table are kept.
				auto const winds = Winds();
				auto wind_sea = api::OceanWindSea{ .m_elements = nullptr, .m_table = nullptr, .m_winds = winds.data(), .m_element_count = 0 };
				View3D_OceanSetWindSea(m_ocean, m_ocean_object, wind_sea);
			}

			// Set gravity, and the same wind that drives the waves, which carries spray and drifts froth.
			void SetEnvironment()
			{
				// Spray and froth do not collide with planes.
				auto environment = api::ParticleEnvironment
				{
					.m_gravity = { 0, 0, -9.8f, 0 },
					.m_wind = { m_wind, 0, 0, 0 },
					.m_collision_iterations = 0,
				};
				View3D_ParticleEnvironmentSet(m_system, environment);
			}

			// Advance the waves and then the particles, which sample the waves' new frame, and refresh the statistics twice a second.
			void Step(double dt) override
			{
				// Long frames are clamped so that a stall does not emit a flood of particles. The waves and particles share one clock.
				auto const step = std::min(dt, 0.1) * m_time_scale;
				m_wave_time += step;
				auto const c2w = View3D_CameraToWorldGet(m_ctx.m_window);
				auto frame = api::OceanFrame
				{
					.m_camera_x_m = c2w.w.x, .m_camera_y_m = c2w.w.y, .m_camera_z_m = c2w.w.z, .m_wave_time_s = m_wave_time, .m_level_m = 0, .m_wave_radius_m = 300,
					.m_min_spacing_m = 0.1, .m_warp_power = 2, .m_breaking_ratio = 0.8, .m_shore_sharpness = 0, .m_colour_argb = 0xE0103848, .m_depth_effects = 1,
				};
				View3D_OceanUpdate(m_ocean, m_ocean_object, frame);
				View3D_ParticleSystemStep(m_system, step);

				// Reading the statistics waits for the GPU, so it is throttled.
				m_stats_timer += dt;
				if (m_stats_timer < 0.5)
					return;

				m_stats_timer = 0;
				auto spray = api::ParticleStats{};
				auto froth = api::ParticleStats{};
				if (!View3D_ParticleStats(m_system, m_spray_type, &spray) || !View3D_ParticleStats(m_system, m_froth_type, &froth))
					return;

				m_stats_label->Text(std::format(L"Spray: live {}\nFroth: live {}", spray.m_live, froth.m_live).c_str());
			}
		};
	}

	// Demo factories
	std::unique_ptr<IDemo> CreateParticlesDemo(DemoContext const& ctx)
	{
		return std::make_unique<ParticlesDemo>(ctx);
	}
	std::unique_ptr<IDemo> CreateParticlePresetsDemo(DemoContext const& ctx)
	{
		return std::make_unique<ParticlePresetsDemo>(ctx);
	}
	std::unique_ptr<IDemo> CreateOceanSprayDemo(DemoContext const& ctx)
	{
		return std::make_unique<OceanSprayDemo>(ctx);
	}
}
