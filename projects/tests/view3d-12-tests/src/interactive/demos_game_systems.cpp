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
#include "demo.h"
#include "controls_panel.h"

namespace view3d_test
{
	namespace
	{
		namespace api = pr::view3d;
		using ParticleSpriteEmitter = pr::game_systems::particles::ParticleSpriteEmitter;
		using ParticleSpriteConstants = pr::game_systems::particles::ParticleSpriteConstants;

		// Sprite particle effects that all share one particle type, so every particle draws in one draw call. The time scale shows that
		// particles are exact in slow motion and stop completely when paused. The stress grid adds many emitters to fill the pool.
		struct ParticlesDemo :IDemo
		{
			static constexpr int Capacity = 1 << 18;
			static constexpr int MaxEmitters = 2048;
			static constexpr int MaxGridSide = 32;
			static constexpr float GridRate = 100.0f;

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
				, m_stats_label()
				, m_stats_timer()
				, m_time_scale(1.0f)
				, m_rate_scale(1.0f)
			{
				// A floor for reference, viewed from the side.
				m_scene.Add("*Plane floor FF404850 { *Data {60 60} *AxisId {+3} }");
				View3D_CameraPositionSet(ctx.m_window, { 0, -28, 9, 1 }, { 0, 0, 4, 1 }, { 0, 0, 1, 0 });

				// One sprite type for every effect. The fade keeps births and deaths soft.
				m_system = View3D_ParticleSystemCreate("DemoParticles");
				if (m_system == nullptr)
					throw std::runtime_error("Particle system creation failed");

				auto shaders = api::ParticleTypeShaders{};
				if (!View3D_ParticleSpriteShaders(&shaders))
					throw std::runtime_error("Particle sprite shaders unavailable");

				auto constants = ParticleSpriteConstants{ .fade_in = 0.05f, .fade_out = 0.3f };
				auto options = api::ParticleTypeOptions
				{
					.m_struct_size = sizeof(api::ParticleTypeOptions),
					.m_version = api::ParticleTypeOptions::CurrentVersion,
					.m_shaders = shaders,
					.m_capacity = Capacity,
					.m_max_emitters = MaxEmitters,
					.m_state_stride = sizeof(pr::game_systems::particles::ParticleSpriteState),
					.m_record_stride = sizeof(pr::game_systems::particles::ParticleSpriteRecord),
					.m_params_stride = sizeof(ParticleSpriteEmitter),
					.m_constants = &constants,
					.m_constants_size = sizeof(constants),
					.m_bbox = api::BBox{ {0, 0, 20, 1}, {60, 60, 30, 0} },
				};
				auto object = api::Object{};
				if (!View3D_ParticleTypeCreate(m_system, options, "DemoSprites", &m_type, &object))
					throw std::runtime_error("Particle type creation failed");

				m_scene.Add(object);

				// Weighted transparency blends overlapping soft sprites, such as smoke, smoothly in any order. The K-buffer keeps only the nearest layers.
				View3D_ObjectNuggetFlagsSet(object, api::ENuggetFlag::AlphaWeighted, TRUE, nullptr, 0);
				// Sparks: a fountain thrown up and pulled down by gravity.
				AddEffect(600.0f, Params({ -8, 0, 0.2f, 0.1f }, { 0, 0, 9, 3 }, { 0, 0, -9.8f }, { 1.2f, 2.0f }, { 0.15f, 0.05f }, 0xFFFFD040, 0x00FF4000));

				// Smoke: slow puffs that drift downwind and grow.
				AddEffect(30.0f, Params({ 0, 0, 0.5f, 0.5f }, { 0.6f, 0, 1.5f, 0.4f }, { 0.3f, 0, 0.2f }, { 4.0f, 6.0f }, { 0.6f, 3.0f }, 0x80606060, 0x00909090));

				// Fire: short-lived flames that rise and shrink.
				AddEffect(300.0f, Params({ 8, 0, 0.3f, 0.6f }, { 0, 0, 2, 0.6f }, { 0, 0, 1.5f }, { 0.5f, 1.0f }, { 0.8f, 0.2f }, 0xFFFFC040, 0x00C02000));

				// Bursts: an explosion that only emits when asked.
				m_burst = Create(0.0f, Params({ 0, 0, 5, 0.2f }, { 0, 0, 0, 8 }, { 0, 0, -9.8f }, { 1.0f, 2.5f }, { 0.25f, 0.1f }, 0xFFFFFFFF, 0x00FF8000));

				// Demo controls.
				auto& ui = ctx.m_controls;
				ui.AddLabel(L"All particles are one particle type, drawn with one draw call. Particle state lives on the GPU.");
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
				// The type's object stays in the scene until the scene is destroyed, and keeps its own references to the particle buffers.
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
