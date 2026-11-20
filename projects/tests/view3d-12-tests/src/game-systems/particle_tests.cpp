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
}
