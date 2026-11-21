//*********************************************
// View3d-12 Tests
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Demos of the game-systems library: GPU-resident particles.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <format>
#include <stdexcept>
#include <vector>
#include "pr/hlsl/interop.h"
#include "pr/gui/wingui.h"
#include "pr/view3d-12/view3d-dll.h"
#include "pr/game-systems/particles/particle_sprite.hlsli"
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
			pr::gui::Label* m_stats_label;
			double m_stats_timer;
			float m_time_scale;
			float m_rate_scale;

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
				, m_stats_label()
				, m_stats_timer()
				, m_time_scale(1.0f)
				, m_rate_scale(1.0f)
			{
				// A floor for reference, viewed from the side.
				m_scene.Add("*Plane floor FF404850 { *Data {60 60} *AxisId {+3} }");
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
					.m_capacity = Capacity,
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

				auto constants = ParticleSpriteConstants{ .fade_in = 0.05f, .fade_out = 0.3f };
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

				// Demo controls.
				auto& ui = ctx.m_controls;
				ui.AddLabel(L"Sprites are one particle type in one batch, drawn with one draw call. Wood chips are a custom type with their own record and vertex shader, in a second, opaque batch. Particle state lives on the GPU.");
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
				ui.AddSlider(L"Stress grid side", 0, MaxGridSide, 0, [this](float v)
				{
					SetGridSide(static_cast<int>(std::lround(v)));
				}, MaxGridSide);
				ui.AddButton(L"Reset", [this]
				{
					View3D_ParticleSystemReset(m_system);
				});
				m_stats_label = ui.AddLabel(L"\n\n\n\n");
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

			// Add a continuous effect.
			void AddEffect(float rate, ParticleSpriteEmitter const& params)
			{
				// Effects keep their unscaled rate so the rate scale can be changed freely.
				m_effects.push_back(Effect{ .m_id = Create(rate * m_rate_scale, params), .m_params = params, .m_rate = rate });
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
				if (!View3D_ParticleStats(m_system, m_type, &stats))
					return;

				m_stats_label->Text(std::format(
					L"Live: {} of {}\nFree: {}\nEmitters: {}\nRequested: {}, dropped: {}",
					stats.m_live, Capacity, stats.m_free, m_effects.size() + 1 + m_grid.size(), stats.m_requested, stats.m_dropped).c_str());
			}
		};
	}

	// Demo factories
	std::unique_ptr<IDemo> CreateParticlesDemo(DemoContext const& ctx)
	{
		return std::make_unique<ParticlesDemo>(ctx);
	}
}
