//*********************************************
// Game Systems Particle Tests
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Render tests of the GPU-resident particle system through the View3D ABI, using the stock sprite type and a test type of its own kernels.
// The fixture's orthographic camera looks down -z with a 100x100 view, so world x maps to pixel column (x + 50) * 128 / 100.
#include "../view3d-12/forward.h"
#include "../view3d-12/render_test_support.h"
#include "pr/game-systems/particles/particle_sprite.hlsli"
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

		// Add a stock sprite type of 'capacity' particles, drawn by 'batch'.
		int SpriteTypeCreate(int batch, int capacity, int max_emitters)
		{
			// No fading, so a sprite's centre has its full colour for its whole life.
			auto sim = api::ParticleSimShaders{};
			auto draw = api::ParticleDrawShaders{};
			Require(View3D_ParticleSpriteShaders(&sim, &draw) != FALSE, "Particle sprite shaders unavailable");
			auto constants = ParticleSpriteConstants{};
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
	PRUnitTest(GameSystems_ParticleBatch, Extended | Serial)
	{
		render_tests::ParticleBatchTests();
		render_tests::ParticleBatchValidationTests();
	}
	PRUnitTest(GameSystems_ParticleCustomRecord, Extended | Serial)
	{
		render_tests::ParticleCustomRecordTests();
	}
	PRUnitTest(GameSystems_ParticleCost, Stress | Serial)
	{
		render_tests::ParticleCostTests();
	}
}
