//*********************************************
// Game Systems Particle Tests
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Render tests of the GPU-resident particle system through the View3D ABI, using the stock sprite type.
// The fixture's orthographic camera looks down -z with a 100x100 view, so world x maps to pixel column (x + 50) * 128 / 100.
#include "../view3d-12/forward.h"
#include "../view3d-12/render_test_support.h"
#include "pr/game-systems/particles/particle_sprite.hlsli"

namespace render_tests
{
	using ParticleSpriteEmitter = pr::game_systems::particles::ParticleSpriteEmitter;
	using ParticleSpriteConstants = pr::game_systems::particles::ParticleSpriteConstants;

	// Sprites sit at this depth in front of the camera.
	constexpr float ParticleDepth = 10.0f;

	// A zero step is a pause that does no work, so bursts are born by a short positive step. Burst particles are born at the end of a step, aged zero.
	constexpr double BirthStep = 1.0 / 1024.0;

	// Return the pixel column of world 'x'.
	static int PixelX(float x)
	{
		// The view is 100 units wide over the image.
		return static_cast<int>((x + 50.0f) * ImageSize / 100.0f);
	}

	// Average linear red in a small square around world (x, 0).
	static float RedAt(std::vector<unsigned char> const& image, float x)
	{
		// A 3x3 sample tolerates rasterisation differences at the sprite centre.
		auto px = PixelX(x);
		return RegionRed(image, px - 1, px + 2, ImageSize / 2 - 1, ImageSize / 2 + 2);
	}

	// Own a particle system with one sprite type, and destroy the system before the fixture shuts View3D down.
	struct SpriteParticles
	{
		Fixture& m_fixture;
		api::ParticleSystem m_system = nullptr;
		api::Object m_object = nullptr;
		int m_type = -1;

		// Create the system and one sprite type of 'capacity' particles, drawn in the fixture's window.
		SpriteParticles(Fixture& fixture, int capacity, int max_emitters = 64)
			: m_fixture(fixture)
		{
			// The stock sprite shaders and layouts.
			m_system = View3D_ParticleSystemCreate("TestParticles");
			Require(m_system != nullptr, "Particle system creation failed");
			auto shaders = api::ParticleTypeShaders{};
			Require(View3D_ParticleSpriteShaders(&shaders) != FALSE, "Particle sprite shaders unavailable");

			// No fading, so a sprite's centre has its full colour for its whole life.
			auto constants = ParticleSpriteConstants{};
			auto options = api::ParticleTypeOptions
			{
				.m_struct_size = sizeof(api::ParticleTypeOptions),
				.m_version = api::ParticleTypeOptions::CurrentVersion,
				.m_shaders = shaders,
				.m_capacity = capacity,
				.m_max_emitters = max_emitters,
				.m_state_stride = sizeof(pr::game_systems::particles::ParticleSpriteState),
				.m_record_stride = sizeof(pr::game_systems::particles::ParticleSpriteRecord),
				.m_params_stride = sizeof(ParticleSpriteEmitter),
				.m_constants = &constants,
				.m_constants_size = sizeof(constants),
				.m_bbox = api::BBox{ {0, 0, 0, 1}, {100, 100, 100, 0} },
			};
			View3D_ParticleTypeCreate(m_system, options, "TestSprites", &m_type, &m_object);
			m_fixture.CheckErrors();

			// The fixture owns the object, which keeps its own references to the particle buffers.
			m_fixture.m_objects.push_back(m_object);
			View3D_WindowAddObject(m_fixture.m_window, m_object);
			m_fixture.CheckErrors();
		}
		~SpriteParticles()
		{
			// The renderer must still exist when the system releases its GPU resources.
			View3D_ParticleSystemDestroy(m_system);
		}

		// Create an emitter of red sprites at world (x, 0) with velocity 'vx' along x.
		uint64_t Emitter(float x, float rate = 0.0f, float vx = 0.0f, float lifetime = 100.0f, uint32_t seed = 1)
		{
			// Birth positions and velocities are exact, so every sprite of the emitter follows the same path.
			auto params = ParticleSpriteEmitter{};
			params.position = { x, 0.0f, -ParticleDepth, 0.0f };
			params.velocity = { vx, 0.0f, 0.0f, 0.0f };
			params.accel = { 0.0f, 0.0f, 0.0f, 0.0f };
			params.lifetime = { lifetime, lifetime };
			params.size = { 8.0f, 8.0f };
			params.colour0 = 0xFFFF0000U;
			params.colour1 = 0xFFFF0000U;

			auto settings = api::ParticleEmitter{ .m_rate = rate, .m_seed = seed, .m_active = TRUE };
			auto id = uint64_t{};
			Require(View3D_ParticleEmitterCreate(m_system, m_type, settings, &params, sizeof(params), &id) != FALSE, "Particle emitter creation failed");
			return id;
		}

		// Request 'count' particles from 'emitter' at the end of the next step.
		void Burst(uint64_t emitter, uint32_t count)
		{
			// Bursts are admitted by the next step.
			Require(View3D_ParticleEmitterBurst(m_system, emitter, count) != FALSE, "Particle burst failed");
		}

		// Advance the simulation by 'dt' seconds.
		void Step(double dt)
		{
			// Errors surface immediately rather than through a later frame.
			Require(View3D_ParticleSystemStep(m_system, dt) != FALSE, "Particle step failed");
			m_fixture.CheckErrors();
		}

		// Restart the simulation with no live particles.
		void Reset()
		{
			// Emitters survive a reset.
			Require(View3D_ParticleSystemReset(m_system) != FALSE, "Particle reset failed");
			m_fixture.CheckErrors();
		}

		// Read the type's pool usage after the last step.
		api::ParticleStats Stats()
		{
			// Stats are read back from the GPU.
			auto stats = api::ParticleStats{};
			Require(View3D_ParticleStats(m_system, m_type, &stats) != FALSE, "Particle stats failed");
			return stats;
		}
	};

	// Many emitters' particles all draw through the type's one object, and pausing changes nothing.
	void ParticleRenderTests()
	{
		auto fixture = Fixture(1);
		{
			auto particles = SpriteParticles(fixture, 1024);

			// One sprite from each of eight emitters spaced along x.
			constexpr int Count = 8;
			for (auto i = 0; i != Count; ++i)
				particles.Burst(particles.Emitter(-35.0f + 10.0f * i), 1);

			particles.Step(BirthStep);
			auto stats = particles.Stats();
			Require(stats.m_live == Count, "Every burst particle should be live");
			Require(stats.m_requested == Count && stats.m_admitted == Count && stats.m_dropped == 0, "Burst counters are wrong");
			Require(stats.m_free == 1024 - Count, "Free count is wrong");

			// Each sprite's centre is red, and the gaps between sprites are darker.
			auto image = fixture.Image();
			for (auto i = 0; i != Count; ++i)
			{
				auto x = -35.0f + 10.0f * i;
				Require(RedAt(image, x) > 0.25f, "Sprite centre is not drawn");
				Require(RedAt(image, x + 5.0f) < RedAt(image, x), "Sprite edge is not softer than its centre");
			}
			Require(RedAt(image, 45.0f) < 0.01f, "Pixels outside every sprite should be background");

			// Zero-length steps are a pause: no particles move, are born, or die.
			for (auto i = 0; i != 5; ++i)
				particles.Step(0.0);

			Require(ImageDifference(image, fixture.Image()) == 0, "Pausing changed the image");
			auto paused = particles.Stats();
			Require(paused.m_live == Count && paused.m_requested == Count, "Pausing changed the particle counts");

			// Reset removes every particle.
			particles.Reset();
			particles.Step(0.0);
			Require(particles.Stats().m_live == 0, "Reset left live particles");
			for (auto i = 0; i != Count; ++i)
				Require(RedAt(fixture.Image(), -35.0f + 10.0f * i) < 0.01f, "Reset left sprites in the image");
		}
		fixture.CheckErrors();
		fixture.CheckDebugLayer();
	}

	// Particles reach the same place in one long step as in many short ones, so slow motion is exact.
	void ParticleTimeStepTests()
	{
		auto fixture = Fixture(1);
		{
			auto particles = SpriteParticles(fixture, 64);
			auto emitter = particles.Emitter(-30.0f, 0.0f, 10.0f);

			// One step of two seconds moves the sprite from x = -30 to x = -10.
			particles.Burst(emitter, 1);
			particles.Step(BirthStep);
			Require(RedAt(fixture.Image(), -30.0f) > 0.25f, "Sprite not drawn at its birth position");
			particles.Step(2.0);
			auto image_long = fixture.Image();
			Require(RedAt(image_long, -10.0f) > 0.25f, "Sprite not drawn at its two second position");
			Require(RedAt(image_long, -30.0f) < 0.01f, "Sprite still drawn at its birth position");

			// Forty steps of 0.05 seconds reach the same image.
			particles.Reset();
			particles.Burst(emitter, 1);
			particles.Step(BirthStep);
			for (auto i = 0; i != 40; ++i)
				particles.Step(0.05);

			Require(ImageDifference(image_long, fixture.Image()) == 0, "Short steps reached a different image");

			// Continuous emission makes the same number of particles at any tick rate.
			particles.Reset();
			auto stream = particles.Emitter(20.0f, 100.0f, 0.0f, 100.0f, 2);
			particles.Step(0.5);
			auto one_step = particles.Stats().m_requested;
			particles.Reset();
			for (auto i = 0; i != 10; ++i)
				particles.Step(0.05);

			auto many_steps = particles.Stats().m_requested;
			Require(one_step >= 49 && one_step <= 50, "Continuous emission made the wrong number of particles");
			Require(one_step == many_steps, "Continuous emission depends on the tick rate");
			Require(View3D_ParticleEmitterDelete(particles.m_system, stream) != FALSE, "Particle emitter delete failed");

			// Particles die when their age reaches their lifetime.
			particles.Reset();
			particles.Burst(particles.Emitter(0.0f, 0.0f, 0.0f, 1.0f, 3), 4);
			particles.Step(BirthStep);
			Require(particles.Stats().m_live == 4, "Short-lived particles were not born");
			particles.Step(0.5);
			Require(particles.Stats().m_live == 4, "Particles died before their lifetime");
			particles.Step(0.6);
			Require(particles.Stats().m_live == 0, "Particles outlived their lifetime");
			Require(particles.Stats().m_free == 64, "Dead particles were not freed");
		}
		fixture.CheckErrors();
		fixture.CheckDebugLayer();
	}

	// A type draws all of its emitters' particles with one indirect draw, so the draw count depends on the number of types, not emitters.
	void ParticleDrawCountTests()
	{
		auto fixture = Fixture(1);
		{
			// Read the draw counts of a newly rendered frame.
			auto draw_stats = [&]
			{
				// Rendering the image records a frame, which resets and fills the window's counts.
				fixture.Image();
				auto stats = api::DrawStats{};
				Require(View3D_WindowDrawStats(fixture.m_window, stats) != FALSE, "Draw stats failed");
				return stats;
			};

			// One emitter of one type.
			auto sparks = SpriteParticles(fixture, 4096, 256);
			sparks.Burst(sparks.Emitter(-40.0f), 4);
			sparks.Step(BirthStep);
			auto one = draw_stats();
			Require(one.m_forward_indirect == 1, "One particle type should be one indirect draw");

			// Many more emitters of the same type add particles but no draws.
			for (auto i = 1; i != 256; ++i)
				sparks.Burst(sparks.Emitter(-40.0f + 0.3f * i, 0.0f, 0.0f, 100.0f, static_cast<uint32_t>(i + 1)), 4);

			sparks.Step(BirthStep);
			Require(sparks.Stats().m_live == 256 * 4, "Every emitter's particles should be live");
			auto many = draw_stats();
			Require(many.m_forward == one.m_forward && many.m_forward_indirect == one.m_forward_indirect, "More emitters changed the draw count");

			// A second type is one more draw.
			auto smoke = SpriteParticles(fixture, 64);
			smoke.Burst(smoke.Emitter(20.0f), 4);
			smoke.Step(BirthStep);
			auto two = draw_stats();
			Require(two.m_forward == one.m_forward + 1 && two.m_forward_indirect == 2, "A second type should add one indirect draw");
		}
		fixture.CheckErrors();
		fixture.CheckDebugLayer();
	}

	// Measure how the GPU cost of tiny sprite particles grows with the live count, at 1920x1080 with 4x MSAA in a perspective view.
	// Each frame is stepped by a fixed 1/60 s, rendered and then waited for, so the GPU work of consecutive frames does not overlap. The step is recorded
	// on the same queue as the frame, so the wait covers it. The simulation cost is the frame time of frames that step without drawing the particles,
	// less that of frames that neither step nor draw. The draw cost is the frame time of frames that step and draw, less that of frames that only step.
	// Batches of the three kinds alternate so that GPU clock changes affect each equally, and medians reject occasional slow frames.
	void ParticleCostTests()
	{
		constexpr int Width = 1920;
		constexpr int Height = 1080;
		constexpr int GridSide = 8;
		constexpr int EmitterCount = GridSide * GridSide;
		constexpr float MeanLifetime = 2.0f;
		constexpr double FrameStep = 1.0 / 60.0;
		constexpr int Capacity = (1 << 20) + (1 << 17);

		// A perspective view of a 40 m square from 45 m away, so the particle cloud covers much of the screen.
		std::cout << std::format("Particle cost at {}x{}, MSAA 4, weighted transparency", Width, Height) << std::endl;
		auto fixture = Fixture(4);
		View3D_WindowBackBufferSizeSet(fixture.m_window, SIZE{ Width, Height }, FALSE);
		View3D_WindowViewportSet(fixture.m_window, api::Viewport{ 0, 0, Width, Height, 0, 1, Width, Height });
		View3D_CameraOrthographicSet(fixture.m_window, FALSE);
		View3D_CameraFovSet(fixture.m_window, api::Vec2{ 1.6f, 0.9f });
		View3D_CameraClipPlanesSet(fixture.m_window, 0.1f, 200, api::EClipPlanes::Both);
		View3D_CameraPositionSet(fixture.m_window, api::Vec4{ 0, -45, 20, 1 }, api::Vec4{ 0, 0, 5, 1 }, api::Vec4{ 0, 0, 1, 0 });
		{
			// One sprite type for every emitter, with the blend mode that smoke and other soft sprites use.
			auto particles = SpriteParticles(fixture, Capacity, EmitterCount);
			View3D_ObjectNuggetFlagsSet(particles.m_object, api::ENuggetFlag::AlphaWeighted, TRUE, nullptr, 0);

			// Tiny sparks thrown up from a grid of emitters over a 40 m square and pulled back down.
			auto emitters = std::vector<std::pair<uint64_t, ParticleSpriteEmitter>>{};
			for (auto j = 0; j != GridSide; ++j)
			{
				for (auto i = 0; i != GridSide; ++i)
				{
					// Emission starts when 'set_live' gives the emitters a rate.
					auto params = ParticleSpriteEmitter{};
					params.position = { -20.0f + 40.0f * i / (GridSide - 1), -20.0f + 40.0f * j / (GridSide - 1), 0.1f, 0.5f };
					params.velocity = { 0.0f, 0.0f, 6.0f, 2.0f };
					params.accel = { 0.0f, 0.0f, -5.0f, 0.0f };
					params.lifetime = { MeanLifetime - 0.5f, MeanLifetime + 0.5f };
					params.size = { 0.06f, 0.02f };
					params.colour0 = 0xFFFFD040U;
					params.colour1 = 0x80FF4000U;

					auto settings = api::ParticleEmitter{ .m_rate = 0.0f, .m_seed = static_cast<uint32_t>(emitters.size() + 1), .m_active = TRUE };
					auto id = uint64_t{};
					Require(View3D_ParticleEmitterCreate(particles.m_system, particles.m_type, settings, &params, sizeof(params), &id) != FALSE, "Particle emitter creation failed");
					emitters.emplace_back(id, params);
				}
			}

			// Set every emitter's rate so that the pool holds about 'live' particles once births and deaths balance.
			auto set_live = [&](int live)
			{
				// The live count of a continuous emitter is its rate times the mean lifetime.
				auto seed = uint32_t{ 1 };
				for (auto const& [id, params] : emitters)
				{
					auto settings = api::ParticleEmitter{ .m_rate = live / (EmitterCount * MeanLifetime), .m_seed = seed++, .m_active = TRUE };
					Require(View3D_ParticleEmitterUpdate(particles.m_system, id, settings, &params, sizeof(params)) != FALSE, "Particle emitter update failed");
				}
			};

			// Render 'count' frames, waiting for each one, and return the frame times in milliseconds.
			auto frame_times = [&](int count, bool step, bool draw)
			{
				// Removing the particles' object from the window removes only their draw, so the batch kinds differ by exactly one cost.
				if (draw)
					View3D_WindowAddObject(fixture.m_window, particles.m_object);
				else
					View3D_WindowRemoveObject(fixture.m_window, particles.m_object);

				auto times = std::vector<double>{};
				for (auto i = 0; i != count; ++i)
				{
					// Time the CPU recording plus the GPU execution of the step and the frame.
					auto t0 = std::chrono::steady_clock::now();
					if (step)
						particles.Step(FrameStep);

					View3D_WindowRender(fixture.m_window);
					View3D_WindowGSyncWait(fixture.m_window);
					auto t1 = std::chrono::steady_clock::now();
					times.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());
				}
				return times;
			};
			auto median = [](std::vector<double> times)
			{
				std::ranges::sort(times);
				return times[times.size() / 2];
			};

			// Measure each live count.
			std::cout << std::format("  {:>8} {:>10} {:>10} {:>10}", "live", "frame ms", "sim ms", "draw ms") << std::endl;
			for (auto live : { 0, 1 << 10, 1 << 12, 1 << 14, 1 << 16, 1 << 18, 1 << 20 })
			{
				// Start from an empty pool, and step until births and deaths balance, which takes the longest lifetime.
				particles.Reset();
				set_live(live);
				frame_times(static_cast<int>((MeanLifetime + 0.5) / FrameStep) + 30, true, true);

				// Alternate batches of frames that neither step nor draw, that only step, and that step and draw.
				auto empty = std::vector<double>{};
				auto sim = std::vector<double>{};
				auto full = std::vector<double>{};
				for (auto round = 0; round != 10; ++round)
				{
					std::ranges::copy(frame_times(20, false, false), std::back_inserter(empty));
					std::ranges::copy(frame_times(20, true, false), std::back_inserter(sim));
					std::ranges::copy(frame_times(20, true, true), std::back_inserter(full));
				}

				// Every request must fit in the pool, or the live count would not be the one asked for.
				auto stats = particles.Stats();
				Require(stats.m_dropped == 0, "The cost pool dropped particles");
				std::cout << std::format("  {:>8} {:>10.3f} {:>10.3f} {:>10.3f}", stats.m_live, median(full), median(sim) - median(empty), median(full) - median(sim)) << std::endl;
			}
		}
		fixture.CheckErrors();
		fixture.CheckDebugLayer();
	}

	// Requests beyond the pool's capacity are dropped and counted.
	void ParticleCapacityTests()
	{
		auto fixture = Fixture(1);
		{
			auto particles = SpriteParticles(fixture, 64);
			auto emitter = particles.Emitter(0.0f);

			// A burst larger than the pool fills it and drops the rest.
			particles.Burst(emitter, 100);
			particles.Step(BirthStep);
			auto stats = particles.Stats();
			Require(stats.m_live == 64 && stats.m_free == 0, "The pool was not filled");
			Require(stats.m_requested == 100 && stats.m_admitted == 64 && stats.m_dropped == 36, "Dropped particles were not counted");

			// A full pool admits nothing more.
			particles.Burst(emitter, 10);
			particles.Step(BirthStep);
			stats = particles.Stats();
			Require(stats.m_live == 64, "A full pool admitted particles");
			Require(stats.m_requested == 110 && stats.m_admitted == 64 && stats.m_dropped == 46, "Counters did not accumulate");
		}
		fixture.CheckErrors();
		fixture.CheckDebugLayer();
	}
}

namespace pr::unittests::game_systems
{
	PRUnitTest(GameSystems_ParticleRender, Extended | Serial)
	{
		render_tests::ParticleRenderTests();
	}
	PRUnitTest(GameSystems_ParticleTimeStep, Extended | Serial)
	{
		render_tests::ParticleTimeStepTests();
	}
	PRUnitTest(GameSystems_ParticleCapacity, Extended | Serial)
	{
		render_tests::ParticleCapacityTests();
	}
	PRUnitTest(GameSystems_ParticleDrawCount, Extended | Serial)
	{
		render_tests::ParticleDrawCountTests();
	}
	PRUnitTest(GameSystems_ParticleCost, Stress | Serial)
	{
		render_tests::ParticleCostTests();
	}
}
