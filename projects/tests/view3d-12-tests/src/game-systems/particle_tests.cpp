//*********************************************
// Game Systems Particle Tests
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Render tests of the GPU-resident particle system through the View3D ABI, using the stock sprite type and a test type of its own kernels.
// The fixture's orthographic camera looks down -z with a 100x100 view, so world x maps to pixel column (x + 50) * 128 / 100.
#include "../view3d-12/forward.h"
#include "../view3d-12/render_test_support.h"
#include "pr/game-systems/particles/particle_sprite.hlsli"
#include "pr/game-systems/particles/particle_stepped_sprite.hlsli"
#include "pr/game-systems/particles/particle_mesh.hlsli"
#include "pr/game-systems/particles/particle_debris.hlsli"
#include "pr/game-systems/particles/particle_spray.hlsli"
#include "pr/game-systems/particles/particle_froth.hlsli"
#include "pr/physics/terrain/water/water_field_types.hlsli"
#include "shaders/hlsl/particle_marker.hlsli"
#include "shaders/hlsl/particle_chip.hlsli"

namespace render_tests
{
	namespace compiled
	{
		// The shaders of the test marker and chip types.
		#ifdef NDEBUG
		#include "shaders/hlsl/compiled/release/particle_marker_emit_cs.h"
		#include "shaders/hlsl/compiled/release/particle_marker_update_cs.h"
		#include "shaders/hlsl/compiled/release/particle_chip_emit_cs.h"
		#include "shaders/hlsl/compiled/release/particle_chip_update_cs.h"
		#include "shaders/hlsl/compiled/release/particle_chip_vs.h"
		#else
		#include "shaders/hlsl/compiled/debug/particle_marker_emit_cs.h"
		#include "shaders/hlsl/compiled/debug/particle_marker_update_cs.h"
		#include "shaders/hlsl/compiled/debug/particle_chip_emit_cs.h"
		#include "shaders/hlsl/compiled/debug/particle_chip_update_cs.h"
		#include "shaders/hlsl/compiled/debug/particle_chip_vs.h"
		#endif
	}

	using ParticleSpriteEmitter = pr::game_systems::particles::ParticleSpriteEmitter;
	using ParticleSpriteConstants = pr::game_systems::particles::ParticleSpriteConstants;
	using ParticleMarkerEmitter = game_systems_tests::ParticleMarkerEmitter;
	using ParticleMarkerConstants = game_systems_tests::ParticleMarkerConstants;

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

	// Own a particle system with one sprite batch and one sprite type, and destroy the system before the fixture shuts View3D down.
	struct SpriteParticles
	{
		Fixture& m_fixture;
		api::ParticleSystem m_system = nullptr;
		api::Object m_object = nullptr;
		int m_batch = -1;
		int m_type = -1;

		// Create the system, a sprite batch of 'batch_capacity' records (or 'capacity' when zero), and a sprite type of 'capacity' particles in it.
		SpriteParticles(Fixture& fixture, int capacity, int max_emitters = 64, api::EParticleBlend blend = api::EParticleBlend::Sorted, int batch_capacity = 0)
			: m_fixture(fixture)
		{
			// The system, its first batch, and the stock sprite type drawn by that batch.
			m_system = View3D_ParticleSystemCreate("TestParticles");
			Require(m_system != nullptr, "Particle system creation failed");
			m_batch = BatchCreate(batch_capacity != 0 ? batch_capacity : capacity, blend, m_object);
			m_type = SpriteTypeCreate(m_batch, capacity, max_emitters);
		}
		~SpriteParticles()
		{
			// The renderer must still exist when the system releases its GPU resources.
			View3D_ParticleSystemDestroy(m_system);
		}

		// Add a batch drawn by the stock sprite vertex shader to the fixture's window, returning its index and its object in 'out_object'.
		int BatchCreate(int capacity, api::EParticleBlend blend, api::Object& out_object)
		{
			// The stock sprite draw shaders, with no batch constants.
			auto sim = api::ParticleSimShaders{};
			auto draw = api::ParticleDrawShaders{};
			Require(View3D_ParticleSpriteShaders(&sim, &draw) != FALSE, "Particle sprite shaders unavailable");
			return BatchCreate(draw, sizeof(pr::game_systems::particles::ParticleSpriteRecord), nullptr, 0, capacity, blend, out_object);
		}

		// Add a batch of 'capacity' records of 'record_stride' bytes, drawn by 'draw' with the batch constants 'constants', to the fixture's window.
		// Returns the batch index, and its object in 'out_object'.
		int BatchCreate(api::ParticleDrawShaders const& draw, int record_stride, void const* constants, size_t constants_size, int capacity, api::EParticleBlend blend, api::Object& out_object)
		{
			// The bounds cover the whole view, so the object is never culled.
			auto options = api::ParticleBatchOptions
			{
				.m_struct_size = sizeof(api::ParticleBatchOptions),
				.m_version = api::ParticleBatchOptions::CurrentVersion,
				.m_shaders = draw,
				.m_capacity = capacity,
				.m_record_stride = record_stride,
				.m_constants = constants,
				.m_constants_size = constants_size,
				.m_blend = blend,
				.m_bbox = api::BBox{ {0, 0, 0, 1}, {100, 100, 100, 0} },
			};
			auto batch = -1;
			View3D_ParticleBatchCreate(m_system, options, "TestBatch", &batch, &out_object);
			m_fixture.CheckErrors();

			// The fixture owns the object, which keeps its own references to the particle buffers.
			m_fixture.m_objects.push_back(out_object);
			View3D_WindowAddObject(m_fixture.m_window, out_object);
			m_fixture.CheckErrors();
			return batch;
		}

		// Add a stock sprite type of 'capacity' particles with 'constants', drawn by 'batch'. The default constants have no curve, flicker or
		// streak, so a sprite's centre has its full colour for its whole life.
		int SpriteTypeCreate(int batch, int capacity, int max_emitters, ParticleSpriteConstants const& constants = {})
		{
			// The stock sprite simulation shaders.
			auto sim = api::ParticleSimShaders{};
			auto draw = api::ParticleDrawShaders{};
			Require(View3D_ParticleSpriteShaders(&sim, &draw) != FALSE, "Particle sprite shaders unavailable");
			return TypeCreate(api::ParticleTypeOptions
			{
				.m_struct_size = sizeof(api::ParticleTypeOptions),
				.m_version = api::ParticleTypeOptions::CurrentVersion,
				.m_shaders = sim,
				.m_batch = batch,
				.m_capacity = capacity,
				.m_max_emitters = max_emitters,
				.m_state_stride = sizeof(pr::game_systems::particles::ParticleSpriteState),
				.m_record_stride = sizeof(pr::game_systems::particles::ParticleSpriteRecord),
				.m_params_stride = sizeof(ParticleSpriteEmitter),
				.m_constants = &constants,
				.m_constants_size = sizeof(constants),
			}, "TestSprites");
		}

		// Add a particle type, returning its index.
		int TypeCreate(api::ParticleTypeOptions const& options, char const* name)
		{
			// Option errors are reported through the fixture's error callback.
			auto type = -1;
			View3D_ParticleTypeCreate(m_system, options, name, &type);
			m_fixture.CheckErrors();
			return type;
		}

		// Create an emitter of 'type' with the type's parameters 'params'.
		uint64_t EmitterCreate(int type, void const* params, size_t params_size, float rate, uint32_t seed)
		{
			// Emitters start active.
			auto settings = api::ParticleEmitter{ .m_rate = rate, .m_seed = seed, .m_active = TRUE };
			auto id = uint64_t{};
			Require(View3D_ParticleEmitterCreate(m_system, type, settings, params, params_size, &id) != FALSE, "Particle emitter creation failed");
			return id;
		}

		// Create an emitter of red sprites of the sprite type at world (x, 0) with velocity 'vx' along x.
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
			return EmitterCreate(m_type, &params, sizeof(params), rate, seed);
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

		// Add a stock stepped sprite type of 'capacity' particles, drawn by 'batch'.
		int SteppedTypeCreate(int batch, int capacity, int max_emitters = 16)
		{
			// Stepped sprites share the sprite record and constants, so they draw with the sprite batch.
			auto sim = api::ParticleSimShaders{};
			auto draw = api::ParticleDrawShaders{};
			Require(View3D_ParticleSteppedSpriteShaders(&sim, &draw) != FALSE, "Particle stepped sprite shaders unavailable");
			auto constants = ParticleSpriteConstants{};
			return TypeCreate(api::ParticleTypeOptions
			{
				.m_struct_size = sizeof(api::ParticleTypeOptions),
				.m_version = api::ParticleTypeOptions::CurrentVersion,
				.m_shaders = sim,
				.m_batch = batch,
				.m_capacity = capacity,
				.m_max_emitters = max_emitters,
				.m_state_stride = sizeof(pr::game_systems::particles::ParticleSteppedSpriteState),
				.m_record_stride = sizeof(pr::game_systems::particles::ParticleSpriteRecord),
				.m_params_stride = sizeof(pr::game_systems::particles::ParticleSteppedSpriteEmitter),
				.m_constants = &constants,
				.m_constants_size = sizeof(constants),
			}, "TestSteppedSprites");
		}

		// Create an emitter of red stepped sprites of 'type' at world (x, 0), with velocity 'vx' along x, acceleration 'ax' along x, and 'drag'.
		// Its particles have collision 'radius' and bounce off 'collision_set'.
		uint64_t SteppedEmitter(int type, float x, float vx, int collision_set, float radius = 1.0f, float ax = 0.0f, float drag = 0.0f)
		{
			// Birth positions and velocities are exact, so every particle of the emitter follows the same path.
			auto params = pr::game_systems::particles::ParticleSteppedSpriteEmitter{};
			params.position = { x, 0.0f, -ParticleDepth, 0.0f };
			params.velocity = { vx, 0.0f, 0.0f, 0.0f };
			params.accel = { ax, 0.0f, 0.0f, drag };
			params.lifetime = { 100.0f, 100.0f };
			params.size = { 8.0f, 8.0f };
			params.colour0 = 0xFFFF0000U;
			params.colour1 = 0xFFFF0000U;
			params.radius = radius;
			params.collision_set = collision_set;
			params.gravity = 1.0f;
			return EmitterCreate(type, &params, sizeof(params), 0.0f, 1);
		}

		// Set the environment's gravity and uniform wind along x, and its collision budget.
		void Environment(float gravity_x, float wind_x, int collision_iterations = 4)
		{
			// The tests move particles along x only, so the image shows their positions.
			auto env = api::ParticleEnvironment
			{
				.m_gravity = { gravity_x, 0, 0, 0 },
				.m_wind = { wind_x, 0, 0, 0 },
				.m_collision_iterations = collision_iterations,
			};
			Require(View3D_ParticleEnvironmentSet(m_system, env) != FALSE, "Particle environment set failed");
		}

		// Create a collision set of 'planes', returning its index.
		int CollisionSet(std::initializer_list<api::ParticlePlane> planes)
		{
			// An empty set is valid.
			auto set = -1;
			Require(View3D_ParticleCollisionSetCreate(m_system, planes.begin(), static_cast<int>(planes.size()), &set) != FALSE, "Particle collision set create failed");
			return set;
		}

		// Read the pool usage of 'type' after the last step.
		api::ParticleStats Stats(int type = -1)
		{
			// Stats are read back from the GPU. The default is the sprite type.
			auto stats = api::ParticleStats{};
			Require(View3D_ParticleStats(m_system, type != -1 ? type : m_type, &stats) != FALSE, "Particle stats failed");
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

	// A batch draws all of its types' and emitters' particles with one indirect draw, so the draw count depends on the number of batches only.
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

			// One emitter of one type in one batch. The batch has room for a second type.
			auto sparks = SpriteParticles(fixture, 4096, 256, api::EParticleBlend::Sorted, 4096 + 64);
			sparks.Burst(sparks.Emitter(-40.0f), 4);
			sparks.Step(BirthStep);
			auto one = draw_stats();
			Require(one.m_forward_indirect == 1, "One particle batch should be one indirect draw");

			// Many more emitters of the same type add particles but no draws.
			for (auto i = 1; i != 256; ++i)
				sparks.Burst(sparks.Emitter(-40.0f + 0.3f * i, 0.0f, 0.0f, 100.0f, static_cast<uint32_t>(i + 1)), 4);

			sparks.Step(BirthStep);
			Require(sparks.Stats().m_live == 256 * 4, "Every emitter's particles should be live");
			auto many = draw_stats();
			Require(many.m_forward == one.m_forward && many.m_forward_indirect == one.m_forward_indirect, "More emitters changed the draw count");

			// A second type in the same batch adds particles but no draws.
			auto embers = sparks.SpriteTypeCreate(sparks.m_batch, 64, 4);
			auto params = ParticleSpriteEmitter{};
			params.position = { 20.0f, 0.0f, -ParticleDepth, 0.0f };
			params.lifetime = { 100.0f, 100.0f };
			params.size = { 8.0f, 8.0f };
			params.colour0 = 0xFFFF0000U;
			params.colour1 = 0xFFFF0000U;
			Require(View3D_ParticleEmitterBurst(sparks.m_system, sparks.EmitterCreate(embers, &params, sizeof(params), 0.0f, 1), 4) != FALSE, "Particle burst failed");
			sparks.Step(BirthStep);
			Require(sparks.Stats(embers).m_live == 4, "The second type's particles should be live");
			auto shared = draw_stats();
			Require(shared.m_forward == one.m_forward && shared.m_forward_indirect == 1, "A second type in the same batch changed the draw count");

			// A second batch is one more draw.
			auto object = api::Object{};
			auto batch = sparks.BatchCreate(64, api::EParticleBlend::Weighted, object);
			auto smoke = sparks.SpriteTypeCreate(batch, 64, 4);
			params.position = { 30.0f, 0.0f, -ParticleDepth, 0.0f };
			Require(View3D_ParticleEmitterBurst(sparks.m_system, sparks.EmitterCreate(smoke, &params, sizeof(params), 0.0f, 1), 4) != FALSE, "Particle burst failed");
			sparks.Step(BirthStep);
			auto two = draw_stats();
			Require(two.m_forward == one.m_forward + 1 && two.m_forward_indirect == 2, "A second batch should add one indirect draw");
		}
		fixture.CheckErrors();
		fixture.CheckDebugLayer();
	}

	// The renderer culls each particle by its bounds, so particles whose sprites reach into the view are drawn and the rest are skipped.
	void ParticleCullTests()
	{
		auto fixture = Fixture(1);
		{
			// A sprite in the view, a sprite whose centre is just left of the view but whose edge reaches into it, and many far outside the view.
			auto particles = SpriteParticles(fixture, 1024, 128);
			particles.Burst(particles.Emitter(0.0f), 1);
			particles.Burst(particles.Emitter(-52.0f), 1);
			for (auto i = 0; i != 64; ++i)
				particles.Burst(particles.Emitter(200.0f + 4.0f * i, 0.0f, 0.0f, 100.0f, static_cast<uint32_t>(i + 2)), 4);

			particles.Step(BirthStep);
			Require(particles.Stats().m_live == 2 + 64 * 4, "Every burst particle should be live");

			// The visible sprite is drawn, and so is the visible part of the sprite whose centre is outside the view.
			auto image = fixture.Image();
			Require(RedAt(image, 0.0f) > 0.25f, "The sprite in the view is not drawn");
			Require(RegionRed(image, 0, 2, ImageSize / 2 - 1, ImageSize / 2 + 2) > 0.02f, "A sprite reaching into the view from outside it was culled");
			Require(RedAt(image, 25.0f) < 0.01f, "Pixels outside every sprite should be background");

			// Moving the camera onto the far sprites draws them. The camera keeps its orientation and moves along x only.
			auto moved = api::Mat4x4{ {1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {248, 0, 0, 1} };
			View3D_CameraToWorldSet(fixture.m_window, moved);
			Require(RedAt(fixture.Image(), 0.0f) > 0.25f, "Sprites outside the first view are not drawn when the camera moves to them");
		}
		fixture.CheckErrors();
		fixture.CheckDebugLayer();
	}

	// Types with different states, emitter parameters, constants and kernels share a batch when they write the same render record.
	void ParticleBatchTests()
	{
		auto fixture = Fixture(1);
		{
			// A sprite type and a marker type in one batch.
			auto particles = SpriteParticles(fixture, 64, 4, api::EParticleBlend::Sorted, 64 + 16);
			auto constants = ParticleMarkerConstants{ .size = 6.0f };
			auto markers = particles.TypeCreate(api::ParticleTypeOptions
			{
				.m_struct_size = sizeof(api::ParticleTypeOptions),
				.m_version = api::ParticleTypeOptions::CurrentVersion,
				.m_shaders =
				{
					.m_emit = { compiled::particle_marker_emit_cs, sizeof(compiled::particle_marker_emit_cs) },
					.m_update = { compiled::particle_marker_update_cs, sizeof(compiled::particle_marker_update_cs) },
				},
				.m_batch = particles.m_batch,
				.m_capacity = 16,
				.m_max_emitters = 4,
				.m_state_stride = sizeof(game_systems_tests::ParticleMarkerState),
				.m_record_stride = sizeof(pr::game_systems::particles::ParticleSpriteRecord),
				.m_params_stride = sizeof(ParticleMarkerEmitter),
				.m_constants = &constants,
				.m_constants_size = sizeof(constants),
			}, "TestMarkers");

			// Two red sprites on the left, and three red markers spaced 10 apart on the right, so no two particles overlap.
			particles.Burst(particles.Emitter(-35.0f), 1);
			particles.Burst(particles.Emitter(-25.0f, 0.0f, 0.0f, 100.0f, 2), 1);
			auto params = ParticleMarkerEmitter{ .position = { 5.0f, 0.0f, -ParticleDepth }, .spacing = 10.0f, .colour = 0xFFFF0000U };
			particles.Burst(particles.EmitterCreate(markers, &params, sizeof(params), 0.0f, 1), 3);
			particles.Step(BirthStep);

			// Each type counts only its own particles.
			auto sprite_stats = particles.Stats();
			auto marker_stats = particles.Stats(markers);
			Require(sprite_stats.m_live == 2 && sprite_stats.m_free == 62, "Sprite type counts are wrong");
			Require(marker_stats.m_live == 3 && marker_stats.m_free == 13, "Marker type counts are wrong");

			// Every particle of both types is drawn, by one indirect draw.
			auto image = fixture.Image();
			for (auto x : { -35.0f, -25.0f, 5.0f, 15.0f, 25.0f })
				Require(RedAt(image, x) > 0.25f, "A particle of the shared batch is not drawn");

			Require(RedAt(image, 40.0f) < 0.01f, "Pixels outside every particle should be background");
			auto stats = api::DrawStats{};
			Require(View3D_WindowDrawStats(fixture.m_window, stats) != FALSE, "Draw stats failed");
			Require(stats.m_forward_indirect == 1, "A shared batch should be one indirect draw");

			// Markers never die, and sprites die at their lifetime, independently of each other.
			particles.Step(200.0);
			Require(particles.Stats().m_live == 0 && particles.Stats(markers).m_live == 3, "Types did not age independently");
			image = fixture.Image();
			Require(RedAt(image, -35.0f) < 0.01f && RedAt(image, 15.0f) > 0.25f, "Dead sprites drawn or live markers missing");

			// Reset empties every type of the batch.
			particles.Reset();
			particles.Step(0.0);
			Require(particles.Stats(markers).m_live == 0, "Reset left live markers");
			Require(RedAt(fixture.Image(), 15.0f) < 0.01f, "Reset left markers in the image");
		}
		fixture.CheckErrors();
		fixture.CheckDebugLayer();
	}

	// A caller-defined type with its own render record, vertex shader and batch constants, drawn opaque by a batch of its own.
	void ParticleCustomRecordTests()
	{
		auto fixture = Fixture(1);
		{
			using namespace game_systems_tests;

			// An opaque chip batch next to the helper's sprite batch. Chips are half as wide as they are long.
			static constexpr auto indices = ParticleChipIndices();
			auto particles = SpriteParticles(fixture, 16, 4);
			auto draw = api::ParticleDrawShaders
			{
				.m_vertex = { compiled::particle_chip_vs, sizeof(compiled::particle_chip_vs) },
				.m_indices = indices.data(),
				.m_index_count = static_cast<int>(indices.size()),
				.m_vertex_count = static_cast<int>(ParticleChipVertexCount),
				.m_geom = api::EGeom::Vert | api::EGeom::Colr | api::EGeom::Norm,
			};
			auto draw_constants = ParticleChipDrawConstants{ .width = 0.5f, .thickness = 0.25f };
			auto object = api::Object{};
			auto batch = particles.BatchCreate(draw, sizeof(ParticleChipRecord), &draw_constants, sizeof(draw_constants), 16, api::EParticleBlend::Opaque, object);

			// A chip type with no gravity, so chips stay where they are born.
			auto constants = ParticleChipConstants{};
			auto chips = particles.TypeCreate(api::ParticleTypeOptions
			{
				.m_struct_size = sizeof(api::ParticleTypeOptions),
				.m_version = api::ParticleTypeOptions::CurrentVersion,
				.m_shaders =
				{
					.m_emit = { compiled::particle_chip_emit_cs, sizeof(compiled::particle_chip_emit_cs) },
					.m_update = { compiled::particle_chip_update_cs, sizeof(compiled::particle_chip_update_cs) },
				},
				.m_batch = batch,
				.m_capacity = 16,
				.m_max_emitters = 4,
				.m_state_stride = sizeof(ParticleChipState),
				.m_record_stride = sizeof(ParticleChipRecord),
				.m_params_stride = sizeof(ParticleChipEmitter),
				.m_constants = &constants,
				.m_constants_size = sizeof(constants),
			}, "TestChips");

			// Three unturned red chips, 8 long and 4 wide, spaced 10 apart.
			auto params = ParticleChipEmitter
			{
				.position = { 5.0f, 0.0f, -ParticleDepth, 8.0f },
				.velocity = { 0.0f, 0.0f, 0.0f, 0.0f },
				.spacing = 10.0f,
				.spin = 0.0f,
				.lifetime = 100.0f,
				.colour = 0xFFFF0000U,
			};
			auto emitter = particles.EmitterCreate(chips, &params, sizeof(params), 0.0f, 1);
			particles.Burst(emitter, 3);
			particles.Step(BirthStep);
			Require(particles.Stats(chips).m_live == 3, "Chip count is wrong");

			// Each chip covers its length along x, and the batch constants limit its width along y.
			auto image = fixture.Image();
			for (auto x : { 5.0f, 15.0f, 25.0f, 5.0f + 3.0f, 5.0f - 3.0f })
				Require(RedAt(image, x) > 0.25f, "A chip is not drawn over its length");

			auto px = PixelX(5.0f);
			auto in_y = static_cast<int>(1.5f * ImageSize / 100.0f);
			auto off_y = static_cast<int>(3.0f * ImageSize / 100.0f);
			Require(RegionRed(image, px - 1, px + 2, ImageSize / 2 + in_y - 1, ImageSize / 2 + in_y + 1) > 0.25f, "A chip is narrower than its batch constants say");
			Require(RegionRed(image, px - 1, px + 2, ImageSize / 2 + off_y, ImageSize / 2 + off_y + 2) < 0.01f, "A chip is wider than its batch constants allow");
			Require(RedAt(image, -25.0f) < 0.01f, "Pixels outside every chip should be background");

			// The sprite batch and the chip batch are one indirect draw each.
			auto stats = api::DrawStats{};
			Require(View3D_WindowDrawStats(fixture.m_window, stats) != FALSE, "Draw stats failed");
			Require(stats.m_forward_indirect == 2, "Each batch should be one indirect draw");

			// Chips die at their lifetime.
			particles.Step(200.0);
			Require(particles.Stats(chips).m_live == 0, "Chips outlived their lifetime");
			Require(RedAt(fixture.Image(), 5.0f) < 0.01f, "Dead chips are drawn");
		}
		fixture.CheckErrors();
		fixture.CheckDebugLayer();
	}

	// The stock mesh batch draws a caller's shared mesh for each record, textured by the batch texture. Chips write the mesh record layout, so the
	// test chip type is drawn by it.
	void ParticleMeshTests()
	{
		auto fixture = Fixture(1);
		{
			using namespace game_systems_tests;
			using ParticleMeshVertex = pr::game_systems::particles::ParticleMeshVertex;
			static_assert(sizeof(ParticleChipRecord) == sizeof(pr::game_systems::particles::ParticleMeshRecord));

			// A unit card facing +z, so the camera sees it face on. 'u' runs along x and 'v' down y.
			static constexpr auto card = std::array<ParticleMeshVertex, 4>
			{{
				{ .pos = { -0.5f, -0.5f, 0.0f }, .u = 0.0f, .norm = { 0.0f, 0.0f, 1.0f }, .v = 1.0f },
				{ .pos = { +0.5f, -0.5f, 0.0f }, .u = 1.0f, .norm = { 0.0f, 0.0f, 1.0f }, .v = 1.0f },
				{ .pos = { +0.5f, +0.5f, 0.0f }, .u = 1.0f, .norm = { 0.0f, 0.0f, 1.0f }, .v = 0.0f },
				{ .pos = { -0.5f, +0.5f, 0.0f }, .u = 0.0f, .norm = { 0.0f, 0.0f, 1.0f }, .v = 0.0f },
			}};
			static constexpr auto card_indices = std::array<uint32_t, 6>{ 0, 1, 2, 0, 2, 3 };

			// A two-texel texture: the left texel is transparent and the right one is opaque red. Point sampling keeps the halves sharp.
			auto texture_options = api::TextureOptions{};
			texture_options.m_format = DXGI_FORMAT_R8G8B8A8_UNORM;
			texture_options.m_resource_state = D3D12_RESOURCE_STATE_ALL_SHADER_RESOURCE;
			texture_options.m_mips = 1;
			texture_options.m_multisamp = { 1, 0 };
			texture_options.m_t2s = { {1,0,0,0}, {0,1,0,0}, {0,0,1,0}, {0,0,0,1} };
			auto const texels = std::array<uint32_t, 2>{ 0x00000000U, 0xFF0000FFU };
			auto texture = api::TexturePtr(View3D_TextureCreate(2, 1, texels.data(), sizeof(texels), texture_options));
			auto sampler = View3D_SamplerCreateStock(api::EStockSampler::PointClamp);
			fixture.CheckErrors();
			Require(texture != nullptr && sampler != nullptr, "Mesh texture creation failed");

			// The stock mesh draw with the card as its shared mesh, in an opaque batch next to the helper's sprite batch.
			auto particles = SpriteParticles(fixture, 16, 4);
			auto draw = api::ParticleDrawShaders{};
			Require(View3D_ParticleMeshShaders(&draw) != FALSE, "Particle mesh shaders unavailable");
			draw.m_indices = card_indices.data();
			draw.m_index_count = static_cast<int>(card_indices.size());
			draw.m_vertex_count = static_cast<int>(card.size());
			draw.m_vertex_data = card.data();
			auto options = api::ParticleBatchOptions
			{
				.m_struct_size = sizeof(api::ParticleBatchOptions),
				.m_version = api::ParticleBatchOptions::CurrentVersion,
				.m_shaders = draw,
				.m_capacity = 16,
				.m_record_stride = sizeof(ParticleChipRecord),
				.m_constants = nullptr,
				.m_constants_size = 0,
				.m_blend = api::EParticleBlend::Opaque,
				.m_bbox = api::BBox{ {0, 0, 0, 1}, {100, 100, 100, 0} },
				.m_texture = texture.get(),
				.m_sampler = sampler,
			};

			// Return true if creating a batch with 'opts' fails. Failures are reported as errors, which the fixture collects.
			auto rejected = [&](api::ParticleBatchOptions const& opts)
			{
				auto batch = -1;
				auto object = api::Object{};
				auto ok = View3D_ParticleBatchCreate(particles.m_system, opts, "Invalid", &batch, &object);
				fixture.m_errors.clear();
				return ok == FALSE;
			};

			// Vertex data needs a valid stride, a stride needs vertex data, and every draw generates positions.
			auto bad_stride = options;
			bad_stride.m_shaders.m_vertex_stride = 30;
			Require(rejected(bad_stride), "A vertex data stride that is not a multiple of 4 was accepted");
			auto no_data = options;
			no_data.m_shaders.m_vertex_data = nullptr;
			Require(rejected(no_data), "A vertex data stride without vertex data was accepted");
			auto no_vert = options;
			no_vert.m_shaders.m_geom = api::EGeom::Colr;
			Require(rejected(no_vert), "A draw without positions was accepted");

			auto object = api::Object{};
			auto batch = -1;
			View3D_ParticleBatchCreate(particles.m_system, options, "TestMeshes", &batch, &object);
			fixture.CheckErrors();
			fixture.m_objects.push_back(object);
			View3D_WindowAddObject(fixture.m_window, object);

			// Two unturned white chips, 8 long, spaced 12 apart, drawn as cards by the mesh batch.
			auto constants = ParticleChipConstants{};
			auto chips = particles.TypeCreate(api::ParticleTypeOptions
			{
				.m_struct_size = sizeof(api::ParticleTypeOptions),
				.m_version = api::ParticleTypeOptions::CurrentVersion,
				.m_shaders =
				{
					.m_emit = { compiled::particle_chip_emit_cs, sizeof(compiled::particle_chip_emit_cs) },
					.m_update = { compiled::particle_chip_update_cs, sizeof(compiled::particle_chip_update_cs) },
				},
				.m_batch = batch,
				.m_capacity = 16,
				.m_max_emitters = 4,
				.m_state_stride = sizeof(ParticleChipState),
				.m_record_stride = sizeof(ParticleChipRecord),
				.m_params_stride = sizeof(ParticleChipEmitter),
				.m_constants = &constants,
				.m_constants_size = sizeof(constants),
			}, "TestMeshChips");
			auto params = ParticleChipEmitter
			{
				.position = { 5.0f, 0.0f, -ParticleDepth, 8.0f },
				.velocity = { 0.0f, 0.0f, 0.0f, 0.0f },
				.spacing = 12.0f,
				.spin = 0.0f,
				.lifetime = 100.0f,
				.colour = 0xFFFFFFFFU,
			};
			auto emitter = particles.EmitterCreate(chips, &params, sizeof(params), 0.0f, 1);
			particles.Burst(emitter, 2);
			particles.Step(BirthStep);
			Require(particles.Stats(chips).m_live == 2, "Mesh chip count is wrong");

			// Each card is red over its right half, where the texture is opaque. Its left half is cut out by the transparent texel, and the cards are
			// no wider than their size.
			auto image = fixture.Image();
			for (auto x : { 7.0f, 19.0f })
				Require(RedAt(image, x) > 0.1f, "A mesh card does not show the batch texture");
			for (auto x : { 3.0f, 15.0f })
				Require(RedAt(image, x) < 0.01f, "A transparent texel of a mesh card is drawn");
			for (auto x : { 11.0f, 23.0f })
				Require(RedAt(image, x) < 0.01f, "A mesh card is larger than its size");

			// The sprite batch and the mesh batch are one indirect draw each.
			auto stats = api::DrawStats{};
			Require(View3D_WindowDrawStats(fixture.m_window, stats) != FALSE, "Draw stats failed");
			Require(stats.m_forward_indirect == 2, "Each batch should be one indirect draw");
		}
		fixture.CheckErrors();
		fixture.CheckDebugLayer();
	}

	// A batch rejects a type whose record stride differs from its own, or whose capacity would overflow it.
	void ParticleBatchValidationTests()
	{
		auto fixture = Fixture(1);
		{
			auto particles = SpriteParticles(fixture, 64, 4, api::EParticleBlend::Sorted, 80);
			auto sim = api::ParticleSimShaders{};
			auto draw = api::ParticleDrawShaders{};
			Require(View3D_ParticleSpriteShaders(&sim, &draw) != FALSE, "Particle sprite shaders unavailable");
			auto constants = ParticleSpriteConstants{};
			auto options = api::ParticleTypeOptions
			{
				.m_struct_size = sizeof(api::ParticleTypeOptions),
				.m_version = api::ParticleTypeOptions::CurrentVersion,
				.m_shaders = sim,
				.m_batch = particles.m_batch,
				.m_capacity = 16,
				.m_max_emitters = 4,
				.m_state_stride = sizeof(pr::game_systems::particles::ParticleSpriteState),
				.m_record_stride = sizeof(pr::game_systems::particles::ParticleSpriteRecord),
				.m_params_stride = sizeof(ParticleSpriteEmitter),
				.m_constants = &constants,
				.m_constants_size = sizeof(constants),
			};

			// Return true if creating a type with 'opts' fails.
			auto rejected = [&](api::ParticleTypeOptions const& opts)
			{
				// Failures are reported as errors, which the fixture collects.
				auto type = -1;
				auto ok = View3D_ParticleTypeCreate(particles.m_system, opts, "Invalid", &type);
				fixture.m_errors.clear();
				return ok == FALSE;
			};

			// The batch holds 80 records and already has 64; a stride mismatch or 17 more records is rejected, and exactly 16 more is accepted.
			auto bad_stride = options;
			bad_stride.m_record_stride += 4;
			Require(rejected(bad_stride), "A record stride mismatch was accepted");
			auto too_many = options;
			too_many.m_capacity = 17;
			Require(rejected(too_many), "A batch capacity overflow was accepted");
			Require(!rejected(options), "A type that fits the batch was rejected");
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
			auto particles = SpriteParticles(fixture, Capacity, EmitterCount, api::EParticleBlend::Weighted);

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

			// Point the camera at, or away from, the particle cloud. Off-screen particles cost only their vertex work, which is what culling would save.
			auto look = [&](bool at_cloud)
			{
				// Turning around keeps the camera in place, so only the visibility of the particles changes.
				auto target = at_cloud ? api::Vec4{ 0, 0, 5, 1 } : api::Vec4{ 0, -90, 35, 1 };
				View3D_CameraPositionSet(fixture.m_window, api::Vec4{ 0, -45, 20, 1 }, target, api::Vec4{ 0, 0, 1, 0 });
			};

			// Measure each live count.
			std::cout << std::format("  {:>8} {:>10} {:>10} {:>10} {:>14}", "live", "frame ms", "sim ms", "draw ms", "off-screen ms") << std::endl;
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
				auto away = std::vector<double>{};
				for (auto round = 0; round != 10; ++round)
				{
					std::ranges::copy(frame_times(20, false, false), std::back_inserter(empty));
					std::ranges::copy(frame_times(20, true, false), std::back_inserter(sim));
					std::ranges::copy(frame_times(20, true, true), std::back_inserter(full));
					look(false);
					std::ranges::copy(frame_times(20, true, true), std::back_inserter(away));
					look(true);
				}

				// Every request must fit in the pool, or the live count would not be the one asked for.
				auto stats = particles.Stats();
				Require(stats.m_dropped == 0, "The cost pool dropped particles");
				std::cout << std::format("  {:>8} {:>10.3f} {:>10.3f} {:>10.3f} {:>14.3f}", stats.m_live, median(full), median(sim) - median(empty), median(full) - median(sim), median(away) - median(sim)) << std::endl;
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

	// Stepped particles bounce off collision planes, with counted contacts, overlaps and exhausted budgets.
	void ParticleCollisionTests()
	{
		auto fixture = Fixture(1);
		{
			auto particles = SpriteParticles(fixture, 16, 4, api::EParticleBlend::Sorted, 80);
			auto type = particles.SteppedTypeCreate(particles.m_batch, 64);

			// A wall at x = 0 that keeps particles in x <= 0. Particles of radius 1 touch it at x = -1.
			auto wall = particles.CollisionSet({ api::ParticlePlane{ .m_plane = { -1, 0, 0, 0 }, .m_restitution = 1.0f, .m_damping = 0.0f } });

			// A particle moving at 10 m/s from x = -30 reaches the wall after 2.9 s and returns 11 m in the remaining 1.1 s, whatever the tick length.
			auto bouncer = particles.SteppedEmitter(type, -30.0f, 10.0f, wall);
			for (auto ticks : { 1, 80 })
			{
				// Born at the end of the birth step, then moved for 4 s in 'ticks' equal ticks.
				particles.Reset();
				particles.Burst(bouncer, 1);
				particles.Step(BirthStep);
				for (auto i = 0; i != ticks; ++i)
					particles.Step(4.0 / ticks);

				auto image = fixture.Image();
				auto stats = particles.Stats(type);
				Require(RedAt(image, -12.0f) > 0.25f, "The particle did not bounce off the wall");
				Require(RedAt(image, -1.0f) < 0.01f, "The particle stuck to the wall");
				Require(stats.m_contacts == 1 && stats.m_overlaps == 0 && stats.m_exhausted == 0, "The bounce was not counted once");
			}

			// Restitution scales the normal speed after a contact, so half restitution returns only 5.5 m.
			api::ParticlePlane half = { .m_plane = { -1, 0, 0, 0 }, .m_restitution = 0.5f, .m_damping = 0.0f };
			Require(View3D_ParticleCollisionSetUpdate(particles.m_system, wall, &half, 1) != FALSE, "Particle collision set update failed");
			particles.Reset();
			particles.Burst(bouncer, 1);
			particles.Step(BirthStep);
			particles.Step(4.0);
			Require(RedAt(fixture.Image(), -6.5f) > 0.25f, "Restitution did not scale the bounce");
			Require(View3D_ParticleEmitterDelete(particles.m_system, bouncer) != FALSE, "Particle emitter delete failed");

			// A particle resting on the wall and moving away from it neither overlaps nor contacts it.
			particles.Reset();
			particles.Burst(particles.SteppedEmitter(type, -1.0f, -10.0f, wall), 1);
			particles.Step(BirthStep);
			particles.Step(2.0);
			auto stats = particles.Stats(type);
			Require(RedAt(fixture.Image(), -21.0f) > 0.25f, "The particle moving away from the wall did not move freely");
			Require(stats.m_contacts == 0 && stats.m_overlaps == 0, "A particle moving away from the wall was collided");

			// A particle born inside the wall's allowed distance is pushed out to touch it, and counted as an overlap.
			particles.Reset();
			particles.Burst(particles.SteppedEmitter(type, -0.5f, 0.0f, wall, 4.0f), 1);
			particles.Step(BirthStep);
			stats = particles.Stats(type);
			Require(RedAt(fixture.Image(), -4.0f) > 0.25f, "The overlapping particle was not pushed out of the wall");
			Require(stats.m_overlaps == 1 && stats.m_contacts == 0, "The initial overlap was not counted");

			// A channel between walls at x = 0 and x = -4 leaves particle centres the range [-3, -1].
			auto channel = particles.CollisionSet(
			{
				api::ParticlePlane{ .m_plane = { -1, 0, 0, 0 }, .m_restitution = 1.0f, .m_damping = 0.0f },
				api::ParticlePlane{ .m_plane = { 1, 0, 0, -4 }, .m_restitution = 1.0f, .m_damping = 0.0f },
			});
			auto rattler = particles.SteppedEmitter(type, -2.0f, 100.0f, channel);

			// A particle crossing the channel 50 times in one tick makes 50 contacts within a large budget, ending 1 m past its last contact.
			particles.Environment(0.0f, 0.0f, 64);
			particles.Reset();
			particles.Burst(rattler, 1);
			particles.Step(BirthStep);
			particles.Step(1.0);
			stats = particles.Stats(type);
			Require(stats.m_contacts == 50 && stats.m_exhausted == 0, "Multiple contacts in one tick were not all resolved");
			Require(RedAt(fixture.Image(), -2.0f) > 0.25f, "The particle left the channel");

			// A small budget stops the particle at its last contact for the rest of the tick, and counts the exhausted budget.
			particles.Environment(0.0f, 0.0f, 4);
			particles.Reset();
			particles.Burst(rattler, 1);
			particles.Step(BirthStep);
			particles.Step(1.0);
			stats = particles.Stats(type);
			Require(stats.m_contacts == 4 && stats.m_exhausted == 1, "The exhausted collision budget was not counted");
			Require(RedAt(fixture.Image(), -3.0f) > 0.25f, "The particle did not stop at its last contact");

			// A deleted set has no planes, so its particles pass through where the wall was.
			Require(View3D_ParticleCollisionSetDelete(particles.m_system, wall) != FALSE, "Particle collision set delete failed");
			particles.Reset();
			particles.Burst(particles.SteppedEmitter(type, -30.0f, 10.0f, wall), 1);
			particles.Step(BirthStep);
			particles.Step(4.0);
			Require(RedAt(fixture.Image(), 10.0f) > 0.25f, "A deleted collision set still collided");
			Require(particles.Stats(type).m_contacts == 0, "A deleted collision set counted contacts");

			// Return true if the result 'ok' is a failure, clearing the error it reported.
			auto rejected = [&](BOOL ok)
			{
				// Failures are reported as errors, which the fixture collects.
				fixture.m_errors.clear();
				return ok == FALSE;
			};
			api::ParticlePlane skewed = { .m_plane = { 1, 1, 0, 0 }, .m_restitution = 1.0f, .m_damping = 0.0f };
			api::ParticlePlane bouncy = { .m_plane = { 1, 0, 0, 0 }, .m_restitution = 1.5f, .m_damping = 0.0f };
			auto set = -1;
			Require(rejected(View3D_ParticleCollisionSetCreate(particles.m_system, &skewed, 1, &set)), "A plane without a unit normal was accepted");
			Require(rejected(View3D_ParticleCollisionSetCreate(particles.m_system, &bouncy, 1, &set)), "A restitution above one was accepted");
			Require(rejected(View3D_ParticleCollisionSetDelete(particles.m_system, wall)), "A collision set was deleted twice");
			Require(rejected(View3D_ParticleEnvironmentSet(particles.m_system, api::ParticleEnvironment{ .m_collision_iterations = -1 })), "A negative collision budget was accepted");

			// Deleted indices are reused by the next set.
			Require(particles.CollisionSet({}) == wall, "A deleted collision set index was not reused");
		}
		fixture.CheckErrors();
		fixture.CheckDebugLayer();
	}

	// Gravity, wind and drag are integrated exactly, so stepped particles reach the same place at any tick rate.
	void ParticleEnvironmentTests()
	{
		auto fixture = Fixture(1);
		{
			auto particles = SpriteParticles(fixture, 16, 4, api::EParticleBlend::Sorted, 80);
			auto type = particles.SteppedTypeCreate(particles.m_batch, 64);

			// With gravity 2, wind 5 and drag 0.5 along x, a particle at rest at x = -30 reaches
			//   x = -30 + 5t - 5(1 - e^(-kt))/k + 2(t - (1 - e^(-kt))/k)/k = -9.56 after t = 4 s.
			particles.Environment(2.0f, 5.0f);
			auto emitter = particles.SteppedEmitter(type, -30.0f, 0.0f, -1, 1.0f, 0.0f, 0.5f);
			auto images = std::vector<std::vector<unsigned char>>{};
			for (auto ticks : { 1, 80 })
			{
				// Born at the end of the birth step, then moved for 4 s in 'ticks' equal ticks.
				particles.Reset();
				particles.Burst(emitter, 1);
				particles.Step(BirthStep);
				for (auto i = 0; i != ticks; ++i)
					particles.Step(4.0 / ticks);

				images.push_back(fixture.Image());
				Require(RedAt(images.back(), -9.56f) > 0.25f, "The particle did not follow the exact motion");
			}
			Require(ImageDifference(images[0], images[1]) == 0, "The tick rate changed the particle's position");
			Require(View3D_ParticleEmitterDelete(particles.m_system, emitter) != FALSE, "Particle emitter delete failed");

			// A wind grid of 25 m columns blows at 0 m/s in the left column (centre x = -25) and 20 m/s in the right one (centre x = 0).
			// Strong drag makes particles move with the wind, which is sampled at the start of each tick and clamped beyond the outer centres.
			particles.Environment(0.0f, 0.0f);
			auto wind = std::array<api::Vec2, 4>{ api::Vec2{ 0, 0 }, api::Vec2{ 20, 0 }, api::Vec2{ 0, 0 }, api::Vec2{ 20, 0 } };
			Require(View3D_ParticleWindGridSet(particles.m_system, { -37.5f, -25.0f }, 25.0f, 2, 2, wind.data()) != FALSE, "Particle wind grid set failed");
			particles.Reset();
			particles.Burst(particles.SteppedEmitter(type, -12.5f, 0.0f, -1, 1.0f, 0.0f, 100.0f), 1);
			particles.Burst(particles.SteppedEmitter(type, -40.0f, 0.0f, -1, 1.0f, 0.0f, 100.0f), 1);
			particles.Step(BirthStep);
			particles.Step(1.0);
			auto image = fixture.Image();
			Require(RedAt(image, -2.6f) > 0.25f, "The wind grid was not interpolated between column centres");
			Require(RedAt(image, -12.5f) < 0.01f, "The particle in the interpolated wind did not move");
			Require(RedAt(image, -40.0f) > 0.25f, "The particle in still air beyond the grid moved");

			// Clearing the grid leaves still air, where drag stops the moving particle almost at once.
			Require(View3D_ParticleWindGridSet(particles.m_system, { 0, 0 }, 0.0f, 0, 0, nullptr) != FALSE, "Particle wind grid clear failed");
			particles.Step(1.0);
			image = fixture.Image();
			Require(RedAt(image, -2.5f) > 0.25f, "The particle kept moving in still air");
			Require(RedAt(image, -40.0f) > 0.25f, "The particle at rest moved in still air");
		}
		fixture.CheckErrors();
		fixture.CheckDebugLayer();
	}

	// The sprite curve sets colour by age, and the streak stretches a sprite back along its velocity.
	void ParticleSpriteCurveTests()
	{
		auto fixture = Fixture(1);
		{
			// The batch has room for the default sprite type and the two types below.
			auto particles = SpriteParticles(fixture, 16, 64, api::EParticleBlend::Sorted, 24);

			// Create an emitter of red sprites of 'type' at world (x, 0) moving at 'vx', living 10 s.
			auto emitter = [&](int type, float x, float vx)
			{
				// Small sprites, so the streak is clearly longer than the sprite.
				auto params = ParticleSpriteEmitter{};
				params.position = { x, 0.0f, -ParticleDepth, 0.0f };
				params.velocity = { vx, 0.0f, 0.0f, 0.0f };
				params.lifetime = { 10.0f, 10.0f };
				params.size = { 4.0f, 4.0f };
				params.colour0 = 0xFFFF0000U;
				params.colour1 = 0xFFFF0000U;
				return particles.EmitterCreate(type, &params, sizeof(params), 0.0f, 1);
			};

			// Keys at equal 't' make a step, so the sprite is red for the first half of its life and black after.
			auto curve = ParticleSpriteConstants{ .key_count = 3 };
			curve.keys[0] = { .t = 0.0f, .size = 1.0f, .colour = 0xFFFFFFFFU };
			curve.keys[1] = { .t = 0.5f, .size = 1.0f, .colour = 0xFFFFFFFFU };
			curve.keys[2] = { .t = 0.5f, .size = 1.0f, .colour = 0xFF000000U };
			auto curved = particles.SpriteTypeCreate(particles.m_batch, 4, 4, curve);
			particles.Burst(emitter(curved, -20.0f, 0.0f), 1);
			particles.Step(BirthStep);
			particles.Step(3.0);
			Require(RedAt(fixture.Image(), -20.0f) > 0.25f, "The sprite was not its full colour before the step in its curve");
			particles.Step(4.0);
			Require(RedAt(fixture.Image(), -20.0f) < 0.01f, "The sprite was not black after the step in its curve");

			// A sprite moving at 10 m/s with a 1 s streak spans 10 m behind its centre, as a soft ellipse that is most opaque halfway along the streak.
			// After 2 s its centre is at x = 20, so it covers x = 8 to 22 with its most opaque point at x = 15. A round sprite covers only its centre.
			particles.Reset();
			auto streaked = particles.SpriteTypeCreate(particles.m_batch, 4, 4, ParticleSpriteConstants{ .streak = 1.0f });
			particles.Burst(emitter(streaked, 0.0f, 10.0f), 1);
			particles.Burst(emitter(particles.m_type, -30.0f, 10.0f), 1);
			particles.Step(BirthStep);
			particles.Step(2.0);
			auto image = fixture.Image();
			Require(RedAt(image, 17.0f) > 0.25f && RedAt(image, 13.0f) > 0.25f, "The streaked sprite was not stretched behind its centre");
			Require(RedAt(image, 24.0f) < 0.01f, "The streaked sprite was stretched ahead of its centre");
			Require(RedAt(image, -10.0f) > 0.25f && RedAt(image, -16.0f) < 0.01f, "The round sprite was stretched");

			// Under perspective, a streak follows the particle's motion on screen. A camera at the origin looks down -z with a field of view half as wide
			// as its distance. A sprite 1.5 m to the right moves straight at the camera from 20 m to 5 m away in 1 s, so its 1 s streak runs from 20 m
			// to 5 m deep, which is along the row through the screen centre from 0.15 to 0.6 of the half-width, centred at 0.375. A streak that ignored
			// depth would look like a round sprite at 0.24.
			auto ndc_px = [](float ndc)
			{
				// Pixel column of a horizontal screen position in [-1, 1].
				return static_cast<int>((ndc + 1.0f) * 0.5f * ImageSize);
			};
			auto red_at_ndc = [&](std::vector<unsigned char> const& img, float ndc, int dy)
			{
				// Average red in a 3x3 square, 'dy' rows from the centre row.
				auto px = ndc_px(ndc);
				return RegionRed(img, px - 1, px + 2, ImageSize / 2 + dy - 1, ImageSize / 2 + dy + 2);
			};
			View3D_CameraOrthographicSet(fixture.m_window, FALSE);
			View3D_CameraFovSet(fixture.m_window, api::Vec2{ 2.0f * std::atan(0.5f), 2.0f * std::atan(0.5f) });
			View3D_CameraClipPlanesSet(fixture.m_window, 0.1f, 200, api::EClipPlanes::Both);
			View3D_CameraPositionSet(fixture.m_window, api::Vec4{ 0, 0, 0, 1 }, api::Vec4{ 0, 0, -10, 1 }, api::Vec4{ 0, 1, 0, 0 });
			particles.Reset();
			auto params = ParticleSpriteEmitter{};
			params.position = { 1.5f, 0.0f, -20.0f, 0.0f };
			params.velocity = { 0.0f, 0.0f, 15.0f, 0.0f };
			params.lifetime = { 10.0f, 10.0f };
			params.size = { 1.0f, 1.0f };
			params.colour0 = 0xFFFF0000U;
			params.colour1 = 0xFFFF0000U;
			particles.Burst(particles.EmitterCreate(streaked, &params, sizeof(params), 0.0f, 1), 1);
			particles.Step(BirthStep);
			particles.Step(1.0);
			image = fixture.Image();
			Require(red_at_ndc(image, 0.375f, 0) > 0.25f && red_at_ndc(image, 0.47f, 0) > 0.25f, "The perspective streak did not follow the motion on screen");
			Require(red_at_ndc(image, 0.7f, 0) < 0.01f, "The perspective streak was stretched ahead of its head");
			Require(red_at_ndc(image, 0.375f, 12) < 0.01f && red_at_ndc(image, 0.375f, -12) < 0.01f, "The perspective streak was stretched across its motion");
		}
		fixture.CheckErrors();
		fixture.CheckDebugLayer();
	}

	// A stock effect built only from its preset, in its own batch and type.
	struct PresetEffect
	{
		api::EParticlePreset m_preset;
		api::ParticlePreset m_settings;
		int m_type;
	};

	// Create the batch and type of 'preset' in 'particles'.
	static PresetEffect PresetCreate(SpriteParticles& particles, api::EParticlePreset preset, int capacity)
	{
		// The preset supplies every setting of the batch and the type.
		auto settings = api::ParticlePreset{};
		Require(View3D_ParticlePresetGet(preset, &settings) != FALSE, "Particle preset unavailable");
		auto object = api::Object{};
		auto batch = particles.BatchCreate(settings.m_draw, settings.m_record_stride, nullptr, 0, capacity, settings.m_blend, object);
		auto type = particles.TypeCreate(api::ParticleTypeOptions
		{
			.m_struct_size = sizeof(api::ParticleTypeOptions),
			.m_version = api::ParticleTypeOptions::CurrentVersion,
			.m_shaders = settings.m_sim,
			.m_batch = batch,
			.m_capacity = capacity,
			.m_max_emitters = 4,
			.m_state_stride = settings.m_state_stride,
			.m_record_stride = settings.m_record_stride,
			.m_params_stride = settings.m_params_stride,
			.m_constants = settings.m_constants,
			.m_constants_size = settings.m_constants_size,
		}, "PresetType");
		return PresetEffect{ .m_preset = preset, .m_settings = settings, .m_type = type };
	}

	// Create an emitter of 'effect' at the origin with the preset's rate, that collides with 'collision_set'. A positive 'spread' replaces the
	// preset's spread of births.
	static uint64_t PresetEmitter(SpriteParticles& particles, PresetEffect const& effect, int collision_set, float spread = -1.0f)
	{
		// Every emitter parameter structure starts with its position. Stepped sprites and debris also have a collision set.
		using namespace pr::game_systems::particles;
		auto params = std::vector<uint8_t>(static_cast<size_t>(effect.m_settings.m_params_stride));
		std::memcpy(params.data(), effect.m_settings.m_params, params.size());
		if (spread >= 0.0f)
			reinterpret_cast<pr::hlsl::float4*>(params.data())->w = spread;

		switch (effect.m_preset)
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
			case api::EParticlePreset::Spray:
			case api::EParticlePreset::Froth:
			{
				break;
			}
			default:
			{
				throw std::invalid_argument("Unknown particle preset");
			}
		}
		return particles.EmitterCreate(effect.m_type, params.data(), params.size(), effect.m_settings.m_rate, 1);
	}

	// Every stock effect runs and draws from its preset alone, and moves the way its effect should. Collision planes act as sensors: a plane
	// counts the contacts of particles that reach it, so its contact count shows which way the particles went.
	void ParticlePresetTests()
	{
		auto fixture = Fixture(1);
		{
			auto particles = SpriteParticles(fixture, 16);
			using EParticlePreset = api::EParticlePreset;

			// Gravity down and a wind of 5 m/s along +x.
			auto env = api::ParticleEnvironment{ .m_gravity = { 0, 0, -9.8f, 0 }, .m_wind = { 5, 0, 0, 0 }, .m_collision_iterations = 4 };
			Require(View3D_ParticleEnvironmentSet(particles.m_system, env) != FALSE, "Particle environment set failed");

			// Return a sensor plane that keeps particles on the side of 'normal' at 'distance' from the origin.
			auto sensor = [&](float nx, float nz, float distance)
			{
				// No bounce, so a particle that reaches the plane stays on it.
				return particles.CollisionSet({ api::ParticlePlane{ .m_plane = { nx, 0, nz, -distance }, .m_restitution = 0.0f, .m_damping = 0.0f } });
			};

			// One effect and emitter, colliding with 'collision_set'.
			struct Case
			{
				PresetEffect m_effect;
				uint64_t m_emitter;
			};
			auto create = [&](EParticlePreset preset, int collision_set, float spread = -1.0f)
			{
				// Each case has its own type, so its statistics are its own.
				auto effect = PresetCreate(particles, preset, 4096);
				auto emitter = PresetEmitter(particles, effect, collision_set, spread);
				if (effect.m_settings.m_burst != 0)
					particles.Burst(emitter, static_cast<uint32_t>(effect.m_settings.m_burst));

				return Case{ effect, emitter };
			};

			// Fire and smoke rise to a ceiling and never reach a floor. Sparks are thrown up and fall back below where they started.
			auto fire_up = create(EParticlePreset::Fire, sensor(0, -1, 0.4f));
			auto fire_down = create(EParticlePreset::Fire, sensor(0, +1, 0.4f));
			auto smoke_up = create(EParticlePreset::Smoke, sensor(0, -1, 0.8f));
			auto smoke_down = create(EParticlePreset::Smoke, sensor(0, +1, 0.8f));
			auto sparks = create(EParticlePreset::Sparks, sensor(0, +1, 0.5f));

			// Rain and snow fall onto a floor close below their emitters and die there, so far fewer are alive than their lifetimes allow.
			auto rain = create(EParticlePreset::Rain, sensor(0, +1, 2.0f), 0.5f);
			auto snow = create(EParticlePreset::Snow, sensor(0, +1, 1.0f), 0.5f);

			// Leaves blow downwind and not upwind. Wood chips fly up and land on the floor.
			auto leaves_downwind = create(EParticlePreset::Leaves, sensor(-1, 0, 1.5f));
			auto leaves_upwind = create(EParticlePreset::Leaves, sensor(+1, 0, 1.5f));
			auto chips = create(EParticlePreset::WoodChips, sensor(0, +1, 0.3f));

			// Dust, birds and markers collide with nothing.
			auto dust = create(EParticlePreset::Dust, -1);
			auto birds = create(EParticlePreset::Birds, -1);
			auto marker = create(EParticlePreset::Marker, -1);

			// Run for 3 s at 20 ticks per second, then draw every effect.
			particles.Step(BirthStep);
			for (auto i = 0; i != 60; ++i)
				particles.Step(0.05);

			Require(!fixture.Image().empty(), "The effects were not drawn");
			auto stats = [&](Case const& c)
			{
				// Statistics of the case's type.
				return particles.Stats(c.m_effect.m_type);
			};
			Require(stats(fire_up).m_contacts > 0 && stats(fire_down).m_contacts == 0, "Fire did not rise");
			Require(stats(smoke_up).m_contacts > 0 && stats(smoke_down).m_contacts == 0, "Smoke did not rise");
			Require(stats(sparks).m_contacts > 0, "Sparks did not fall");
			Require(stats(rain).m_contacts > 0 && stats(rain).m_live < 500, "Rain did not die where it landed");
			Require(stats(snow).m_contacts > 0 && stats(snow).m_live < 1000, "Snow did not die where it landed");
			Require(stats(leaves_downwind).m_contacts > 0 && stats(leaves_upwind).m_contacts == 0, "Leaves did not blow downwind");
			Require(stats(chips).m_contacts > 0, "Wood chips did not land");
			Require(stats(dust).m_live == static_cast<uint32_t>(dust.m_effect.m_settings.m_burst), "The dust burst did not live on");
			Require(stats(birds).m_live == static_cast<uint32_t>(birds.m_effect.m_settings.m_burst), "The flock did not live on");
			Require(stats(marker).m_live > 0, "The marker emitted no pulses");

			// An unknown preset is reported as an error.
			auto settings = api::ParticlePreset{};
			Require(View3D_ParticlePresetGet(static_cast<EParticlePreset>(12), &settings) == FALSE, "An unknown preset was accepted");
			Require(!fixture.m_errors.empty(), "An unknown preset reported no error");
			fixture.m_errors.clear();
		}
		fixture.CheckErrors();
		fixture.CheckDebugLayer();
	}

	// Own an ocean of one wind-sea band of three wave trains travelling roughly along +x, over deep water, and its View3D object.
	// The object is not drawn; particles only sample it.
	struct TestOcean
	{
		Fixture& m_fixture;
		api::Ocean m_ocean = nullptr;
		api::Object m_object = nullptr;
		double m_time = 10.0;

		// The wind blowing over every node; zero for a calm sea.
		static std::array<float, 8> Winds(float wind_x)
		{
			// Four nodes of one level each.
			return { wind_x, 0.0f, wind_x, 0.0f, wind_x, 0.0f, wind_x, 0.0f };
		}

		// Create the ocean with a wind of 'wind_x' m/s along x.
		TestOcean(Fixture& fixture, float wind_x)
			: m_fixture(fixture)
		{
			// Three Gerstner trains in band 0, with amplitude and sharpness bounds that the table's rows stay within.
			using namespace pr::physics::terrain::water::shared;
			auto element = [](float angle, float wavelength, float phase)
			{
				auto e = WaterFieldElement{};
				e.info = { WaterFieldElementGerstnerWave, 1, 0, 0 };
				e.position = { std::cos(angle), std::sin(angle), phase, 0.0f };
				e.wave = { 1.0f, wavelength, std::sqrt(9.81f * wavelength / 6.2831853f), 0.0f };
				e.timing = { 0.6f, 0.0f, 0.0f, 0.0f };
				return e;
			};
			auto const elements = std::array<WaterFieldElement, 3>{ element(0.0f, 12.0f, 0.0f), element(0.35f, 9.0f, 1.3f), element(-0.3f, 7.0f, 2.9f) };

			// No waves in still air, and steep, sharp crests downwind at 20 m/s.
			auto const table = std::array<float, 8>{ 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 2.0f, 0.6f, 3.0f };
			auto const winds = Winds(wind_x);
			auto const depths = std::array<float, 4>{ -200.0f, -200.0f, -200.0f, -200.0f };
			auto detail = std::vector<uint32_t>(api::OceanDetailMapSize * api::OceanDetailMapSize);
			Require(View3D_OceanDetailMapBake(detail.data(), static_cast<int>(detail.size())) != FALSE, "Ocean detail map bake failed");

			auto options = api::OceanOptions
			{
				.m_wind_sea_grid = { .m_origin_x_m = -500, .m_origin_y_m = -500, .m_cell_m = 1000, .m_speed_step_mps = 20, .m_width = 2, .m_height = 2, .m_band_count = 1, .m_speed_count = 2, .m_level_count = 1 },
				.m_wind_sea = { .m_elements = elements.data(), .m_table = table.data(), .m_winds = winds.data(), .m_element_count = static_cast<int32_t>(elements.size()) },
				.m_bathymetry_heights = depths.data(),
				.m_bathymetry_origin_x_m = -500,
				.m_bathymetry_origin_y_m = -500,
				.m_bathymetry_cell_m = 1000,
				.m_bathymetry_width = 2,
				.m_bathymetry_height = 2,
				.m_vertices_per_side = 129,
				.m_radius_m = 400,
				.m_half_height_m = 20,
				.m_floor_height_m = -200,
				.m_detail_map = detail.data(),
			};
			m_ocean = View3D_OceanCreate(options, "TestOcean", &m_object);
			m_fixture.CheckErrors();
			Require(m_ocean != nullptr && m_object != nullptr, "Ocean creation failed");
			m_fixture.m_objects.push_back(m_object);

			// Foam starts on crests a little steeper than a single train's, so only crests where trains meet foam.
			auto foam = api::OceanFoam
			{
				.m_shore_foam_depth_m = 1, .m_edge_softness = 0.1, .m_crest_strength = 1, .m_crest_threshold = 0.5, .m_tile_m = 4, .m_bump_m = 0.01,
				.m_radius_m = 100, .m_fade_m = 10, .m_lip_height_m = 0.5, .m_lip_strength = 0, .m_crest_range = 0.5, .m_trail_time_s = 4, .m_crest_depth_m = 1, .m_churn_mps = 0.1,
			};
			Require(View3D_OceanSetFoam(m_ocean, foam) != FALSE, "Ocean foam set failed");
			Update(0.0);
		}
		~TestOcean()
		{
			// The fixture deletes the object.
			View3D_OceanDestroy(m_ocean);
		}

		// Advance the wave clock by 'dt' and set the ocean's frame, with the camera above the origin.
		void Update(double dt)
		{
			// Particles sample the frame set here, so it is set before each particle step.
			m_time += dt;
			auto frame = api::OceanFrame
			{
				.m_camera_x_m = 0, .m_camera_y_m = 0, .m_camera_z_m = 30, .m_wave_time_s = m_time, .m_level_m = 0, .m_wave_radius_m = 300,
				.m_min_spacing_m = 0.25, .m_warp_power = 2, .m_breaking_ratio = 0.8, .m_shore_sharpness = 0,
				.m_rain_intensity = 0, .m_rain_ring_radius_m = 0, .m_rain_ring_wavelength_m = 0, .m_rain_ring_slope = 0, .m_colour_argb = 0xFF204060, .m_depth_effects = 1,
			};
			Require(View3D_OceanUpdate(m_ocean, m_object, frame) != FALSE, "Ocean update failed");
			m_fixture.CheckErrors();
		}

		// Blow a wind of 'wind_x' m/s along x over the whole sea, from the next update.
		void Wind(float wind_x)
		{
			// The elements and table are kept.
			auto const winds = Winds(wind_x);
			auto wind_sea = api::OceanWindSea{ .m_elements = nullptr, .m_table = nullptr, .m_winds = winds.data(), .m_element_count = 0 };
			Require(View3D_OceanSetWindSea(m_ocean, m_object, wind_sea) != FALSE, "Ocean wind set failed");
		}
	};

	// Spray is born all over a bound water surface where the wind blows, and froth only where the ocean draws strong crest foam. Spray droplets die when they fall back
	// into the water, and froth rides the water for its whole life, but dies when the water is removed.
	void ParticleWaterTests()
	{
		auto fixture = Fixture(1);
		{
			// The ocean outlives the particle system, which samples it.
			auto ocean = TestOcean(fixture, 20.0f);
			auto particles = SpriteParticles(fixture, 16);
			auto env = api::ParticleEnvironment{ .m_gravity = { 0, 0, -9.8f, 0 }, .m_wind = { 5, 0, 0, 0 }, .m_collision_iterations = 4 };
			Require(View3D_ParticleEnvironmentSet(particles.m_system, env) != FALSE, "Particle environment set failed");

			// Step the ocean and then the particles, as a game frame does.
			auto step = [&](double dt, int count)
			{
				for (auto i = 0; i != count; ++i)
				{
					ocean.Update(dt);
					particles.Step(dt);
				}
			};

			// Emitters over the sea near the camera, born only by bursts, with lifetimes long enough that only the water ends them. The spray's wind
			// range puts the 5 m/s wind half way up it, so about half its births make droplets.
			using namespace pr::game_systems::particles;
			auto spray = PresetCreate(particles, api::EParticlePreset::Spray, 4096);
			auto spray_params = *static_cast<ParticleSprayEmitter const*>(spray.m_settings.m_params);
			spray_params.area = { 0.0f, 0.0f, 0.0f, 15.0f };
			spray_params.lifetime = { 100.0f, 100.0f };
			spray_params.wind_range = { 2.0f, 8.0f };
			auto spray_emitter = particles.EmitterCreate(spray.m_type, &spray_params, sizeof(spray_params), 0.0f, 1);
			auto froth = PresetCreate(particles, api::EParticlePreset::Froth, 4096);
			auto froth_params = *static_cast<ParticleFrothEmitter const*>(froth.m_settings.m_params);
			froth_params.area = { 0.0f, 0.0f, 0.0f, 15.0f };
			froth_params.lifetime = { 100.0f, 100.0f };
			froth_params.linger = 1e6f;
			auto froth_emitter = particles.EmitterCreate(froth.m_type, &froth_params, sizeof(froth_params), 0.0f, 2);

			// Froth with the preset's short linger, which fades once the crests that formed it move on.
			auto fading = PresetCreate(particles, api::EParticlePreset::Froth, 4096);
			auto fading_params = froth_params;
			fading_params.linger = static_cast<ParticleFrothEmitter const*>(fading.m_settings.m_params)->linger;
			auto fading_emitter = particles.EmitterCreate(fading.m_type, &fading_params, sizeof(fading_params), 0.0f, 3);

			// Burst all effects, and return how many spray and froth particles survive their first tick: spray born where the wind tears it from the
			// sea, and froth born on a crest.
			auto burst = [&]
			{
				// A failed birth lives for its birth step only.
				particles.Burst(spray_emitter, 1000);
				particles.Burst(froth_emitter, 1000);
				particles.Burst(fading_emitter, 1000);
				step(BirthStep, 1);
				step(0.05, 1);
				return std::pair{ particles.Stats(spray.m_type).m_live, particles.Stats(froth.m_type).m_live };
			};

			// Without water, nothing is born.
			auto [dry_spray, dry_froth] = burst();
			Require(dry_spray == 0 && dry_froth == 0, "Spray or froth was born without water");

			// On a rough sea, about half the spray births make droplets, and some froth births find strong crest foam, but not every point has it.
			Require(View3D_ParticleWaterSet(particles.m_system, ocean.m_ocean, ocean.m_object) != FALSE, "Particle water set failed");
			auto [rough_spray, rough_froth] = burst();
			Require(rough_spray > 250 && rough_spray < 750, std::format("Spray births were not thinned by the wind ({} of 1000)", rough_spray).c_str());
			Require(rough_froth > 20 && rough_froth < 1000, std::format("Froth was not born on crests only ({} of 1000)", rough_froth).c_str());
			Require(!fixture.Image().empty(), "Spray and froth were not drawn");

			// Spray falls back into the water within a few seconds and dies there. Froth rides the waves for its whole life, unless the crests that
			// formed it move on, when it fades and dies.
			auto fading_born = particles.Stats(fading.m_type).m_live;
			Require(fading_born > 20, std::format("Fading froth was not born on crests ({} of 1000)", fading_born).c_str());

			// Step for the longest flight of a droplet launched straight up at the top speed from the greatest lift, plus a second for the waves.
			// The wind is level, so drag slows the rise but also limits the fall to 'g / drag'. A fall from rest covers at least '(g/k)(t - 1/k)'
			// in time 't', which bounds the time down from the highest point.
			auto spray_g = 9.8f * spray_params.gravity;
			auto spray_k = spray_params.launch.w;
			auto spray_up = spray_params.launch.x + spray_params.launch.y;
			auto spray_peak = spray_params.lift + spray_up * spray_up / (2.0f * spray_g);
			auto spray_fall = spray_k > 0.0f ? spray_peak * spray_k / spray_g + 1.0f / spray_k : std::sqrt(2.0f * spray_peak / spray_g);
			auto spray_flight = spray_up / spray_g + spray_fall;
			step(0.05, static_cast<int>((spray_flight + 1.0f) / 0.05f));
			Require(particles.Stats(spray.m_type).m_live == 0, "Spray did not die in the water");
			Require(particles.Stats(froth.m_type).m_live == rough_froth, "Froth did not live on the water");

			// Faded puffs are removed once their presence decays below 0.02, about four lingers after the foam under them has gone.
			step(0.05, static_cast<int>(5.0f * fading_params.linger / 0.05f));
			Require(particles.Stats(fading.m_type).m_live < fading_born / 4, std::format("Froth did not fade behind the passing crests ({} of {})", particles.Stats(fading.m_type).m_live, fading_born).c_str());

			// Removing the water ends the froth.
			Require(View3D_ParticleWaterSet(particles.m_system, nullptr, nullptr) != FALSE, "Particle water clear failed");
			step(0.05, 1);
			Require(particles.Stats(froth.m_type).m_live == 0, "Froth outlived the water");

			// A calm sea has no crest foam, and still air tears no spray from it.
			ocean.Wind(0.0f);
			auto calm_env = env;
			calm_env.m_wind = { 0, 0, 0, 0 };
			Require(View3D_ParticleEnvironmentSet(particles.m_system, calm_env) != FALSE, "Particle environment set failed");
			Require(View3D_ParticleWaterSet(particles.m_system, ocean.m_ocean, ocean.m_object) != FALSE, "Particle water set failed");
			auto [calm_spray, calm_froth] = burst();
			Require(calm_spray == 0 && calm_froth == 0, "Spray or froth was born on a calm sea");

			// An ocean without its object is a caller error.
			Require(View3D_ParticleWaterSet(particles.m_system, ocean.m_ocean, nullptr) == FALSE, "An ocean without its object was accepted");
			Require(!fixture.m_errors.empty(), "An ocean without its object reported no error");
			fixture.m_errors.clear();
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
	PRUnitTest(GameSystems_ParticleCull, Extended | Serial)
	{
		render_tests::ParticleCullTests();
	}
	PRUnitTest(GameSystems_ParticleBatch, Extended | Serial)
	{
		render_tests::ParticleBatchTests();
		render_tests::ParticleBatchValidationTests();
	}
	PRUnitTest(GameSystems_ParticleCustomRecord, Extended | Serial)
	{
		render_tests::ParticleCustomRecordTests();
	}
	PRUnitTest(GameSystems_ParticleMesh, Extended | Serial)
	{
		render_tests::ParticleMeshTests();
	}
	PRUnitTest(GameSystems_ParticleCollision, Extended | Serial)
	{
		render_tests::ParticleCollisionTests();
	}
	PRUnitTest(GameSystems_ParticleEnvironment, Extended | Serial)
	{
		render_tests::ParticleEnvironmentTests();
	}
	PRUnitTest(GameSystems_ParticleSpriteCurve, Extended | Serial)
	{
		render_tests::ParticleSpriteCurveTests();
	}
	PRUnitTest(GameSystems_ParticlePresets, Extended | Serial)
	{
		render_tests::ParticlePresetTests();
	}
	PRUnitTest(GameSystems_ParticleWater, Extended | Serial)
	{
		render_tests::ParticleWaterTests();
	}
	PRUnitTest(GameSystems_ParticleCost, Stress | Serial)
	{
		render_tests::ParticleCostTests();
	}
}
