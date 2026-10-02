//*********************************************
// Physics Engine Atmosphere Tests
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/physics/physics.h"
#include "src/utility/gpu.h"

namespace pr::physics::tests
{
	using namespace pr::physics::atmosphere;

	PRUnitTestClass(AtmosphereTests)
	{
		// Build a small stable default configuration for focused solver tests.
		static AtmosphereConfig Config(iv3 cells = iv3{ 16, 16, 8 })
		{
			// Use metre-scale cells and a mild reference lapse so buoyancy is easy to observe in short runs.
			return AtmosphereConfig{
				.m_grid = AtmosphereGrid{ .m_cell_count = cells, .m_origin = v4::Zero(), .m_dx = 1.0f, .m_lid_z = static_cast<float>(cells.z), .m_first_layer_thickness = 1.0f, .m_layer_stretch_power = 1.0f },
				.m_boundaries = AtmosphereBoundaries{},
				.m_reference = AtmosphereReferenceProfile{ .m_temperature_at_origin = 288.0f, .m_lapse_rate = -0.001f, .m_min_temperature = 250.0f },
				.m_gravity = 9.8f,
				.m_floor_exchange_rate = 0.5f,
				.m_lid_temperature = 284.0f,
				.m_lid_relaxation_rate = 0.05f,
				.m_pressure_vcycles = 3,
				.m_pressure_pre_smooth = 16,
				.m_pressure_post_smooth = 16,
				.m_pressure_coarse_smooth = 128,
			};
		}

		// Run 'steps' solver steps and return the final staggered field.
		static AtmosphereState Run(AtmosphereSolver& solver, GpuJob& job, int steps, float dt, AtmosphereStepSources const& sources)
		{
			// Each iteration records into the same job and runs synchronously to keep readback lifetimes simple in tests.
			for (int i = 0; i != steps; ++i)
			{
				solver.Step(job, dt, sources);
				job.Run();
			}
			return solver.ReadBack(job);
		}

		// Create a full staggered field at rest with reference-profile temperatures.
		static AtmosphereState ReferenceState(AtmosphereConfig const& config)
		{
			// The MAC arrays are sized from the grid helper so tests cover the public layout contract.
			auto const& grid = config.m_grid;
			auto state = AtmosphereState{
				.m_u_faces = std::vector<float>(grid.UFaceCount(), 0.0f),
				.m_v_faces = std::vector<float>(grid.VFaceCount(), 0.0f),
				.m_w_faces = std::vector<float>(grid.WFaceCount(), 0.0f),
				.m_temperature = std::vector<float>(grid.CellCount(), 0.0f),
				.m_pressure = std::vector<float>(grid.CellCount(), 0.0f),
			};
			for (int z = 0; z != grid.m_cell_count.z; ++z)
			{
				// Fill one horizontal layer at a time.
				for (int y = 0; y != grid.m_cell_count.y; ++y)
				{
					// Rows use contiguous x cells.
					for (int x = 0; x != grid.m_cell_count.x; ++x)
						state.m_temperature[grid.CellIndex(iv3{ x, y, z })] = config.m_reference.Temperature(grid.CellCentre(iv3{ x, y, z }).z);
				}
			}
			return state;
		}

		// Return the mean temperature for one horizontal layer.
		static float LayerMeanTemperature(AtmosphereConfig const& config, AtmosphereState const& state, int z)
		{
			// Layer means make the plume test independent of where the warm cap spreads laterally.
			auto sum = 0.0;
			for (int y = 0; y != config.m_grid.m_cell_count.y; ++y)
			{
				// Rows use contiguous x cells.
				for (int x = 0; x != config.m_grid.m_cell_count.x; ++x)
					sum += state.m_temperature[config.m_grid.CellIndex(iv3{ x, y, z })];
			}
			return static_cast<float>(sum / (config.m_grid.m_cell_count.x * config.m_grid.m_cell_count.y));
		}

		PRUnitTestMethod(ConfigValidation, Quick)
		{
			// Invalid dimensions, spacing, rates, pressure counts, V-cycle smoothing counts, and boundary enum values are rejected at the API boundary.
			auto config = Config();
			config.Validate();

			auto bad = config;
			bad.m_grid.m_cell_count.x = 1;
			PR_THROWS(bad.Validate(), std::invalid_argument);

			bad = config;
			bad.m_grid.m_dx = 0.0f;
			PR_THROWS(bad.Validate(), std::invalid_argument);

			bad = config;
			bad.m_reference.m_temperature_at_origin = -1.0f;
			PR_THROWS(bad.Validate(), std::invalid_argument);

			bad = config;
			bad.m_floor_exchange_rate = -0.1f;
			PR_THROWS(bad.Validate(), std::invalid_argument);

			bad = config;
			bad.m_pressure_vcycles = -1;
			PR_THROWS(bad.Validate(), std::invalid_argument);

			bad = config;
			bad.m_pressure_coarse_smooth = -1;
			PR_THROWS(bad.Validate(), std::invalid_argument);

			bad = config;
			bad.m_boundaries.m_x_min = static_cast<EAtmosphereBoundary>(99);
			PR_THROWS(bad.Validate(), std::invalid_argument);
		}

		PRUnitTestMethod(RestStability, Quick)
		{
			// A reference-profile atmosphere with no forcing should stay near rest.
			auto gpu = Gpu{};
			auto job = GpuJob{ gpu.m_gpu, "AtmosphereTests.Rest", 0xFF00AAFF, 1 };
			auto solver = AtmosphereSolver{ gpu, Config() };
			auto state = Run(solver, job, 8, 0.05f, AtmosphereStepSources{});
			auto stats = solver.Stats(state);
			std::printf("Atmosphere rest max_speed %.6f rms_div %.6f max_div %.6f\n", stats.m_max_speed, stats.m_rms_divergence, stats.m_max_divergence);
			PR_EXPECT(stats.m_max_speed < 0.02f);
			PR_EXPECT(stats.m_rms_divergence < 0.002f);
		}

		PRUnitTestMethod(WarmPlume, Quick)
		{
			// A warm sphere near the floor should drive upward motion and warm the lid layer.
			auto gpu = Gpu{};
			auto job = GpuJob{ gpu.m_gpu, "AtmosphereTests.WarmPlume", 0xFF00AAFF, 1 };
			auto config = Config();
			config.m_lid_relaxation_rate = 0.0f;
			auto solver = AtmosphereSolver{ gpu, config };
			auto initial = solver.ReadBack(job);
			auto const upper_layer = config.m_grid.m_cell_count.z - 2;
			auto initial_upper = LayerMeanTemperature(config, initial, upper_layer);
			auto source = AtmosphereHeatSource{ .m_centre = v4{ 8.0f, 8.0f, 1.5f, 1.0f }, .m_radius = 3.0f, .m_heating_rate = 60.0f, .m_target_temperature = 0.0f, .m_relaxation_rate = 0.0f };
			auto sources = AtmosphereStepSources{ .m_heat_sources = std::span{ &source, 1 }, .m_uniform_floor_temperature = 288.0f };
			auto state = Run(solver, job, 120, 0.05f, sources);
			auto stats = solver.Stats(state);
			auto final_upper = LayerMeanTemperature(config, state, upper_layer);
			std::printf("Atmosphere plume peak_w %.6f upper_warming %.6f\n", stats.m_peak_vertical_velocity, final_upper - initial_upper);
			PR_EXPECT(stats.m_peak_vertical_velocity > 0.05f);
			PR_EXPECT(final_upper > initial_upper + 0.01f);
		}

		PRUnitTestMethod(ColdDrainage, Quick)
		{
			// A cold block on the left produces dense air that spreads along the floor toward the warmer right side in this box fixture.
			auto gpu = Gpu{};
			auto job = GpuJob{ gpu.m_gpu, "AtmosphereTests.ColdDrainage", 0xFF00AAFF, 1 };
			auto solver = AtmosphereSolver{ gpu, Config() };
			auto source = AtmosphereHeatSource{ .m_centre = v4{ 3.0f, 8.0f, 1.5f, 1.0f }, .m_radius = 3.0f, .m_heating_rate = -80.0f, .m_target_temperature = 0.0f, .m_relaxation_rate = 0.0f };
			auto node = AtmospherePressureNode{ .m_centre = v2{ -20.0f, 8.0f }, .m_strength = -10.0f, .m_radius = 30.0f, .m_lifetime = 100.0f };
			auto sources = AtmosphereStepSources{ .m_heat_sources = std::span{ &source, 1 }, .m_uniform_floor_temperature = 288.0f };
			auto state = Run(solver, job, 20, 0.05f, sources);
			auto cells = solver.CellStates(state);
			auto vx = cells[solver.Config().m_grid.CellIndex(iv3{ 4, 8, 0 })].m_velocity.x;
			std::printf("Atmosphere drainage floor_vx %.6f\n", vx);
			PR_EXPECT(vx > 0.01f);
		}

		PRUnitTestMethod(DivergenceProjection, Quick)
		{
			// The multigrid projection should reduce a smooth expanding MAC field by at least two orders of magnitude.
			auto gpu = Gpu{};
			auto job = GpuJob{ gpu.m_gpu, "AtmosphereTests.Divergence", 0xFF00AAFF, 1 };
			auto config = Config();
			config.m_pressure_vcycles = 3;
			config.m_pressure_pre_smooth = 256;
			config.m_pressure_post_smooth = 256;
			config.m_pressure_coarse_smooth = 512;
			auto solver = AtmosphereSolver{ gpu, config };
			auto state = ReferenceState(config);
			auto const& grid = config.m_grid;
			for (int z = 0; z != grid.m_cell_count.z; ++z)
			{
				// Fill one layer at a time with an outward horizontal face field that is compatible with solid walls.
				for (int y = 0; y != grid.m_cell_count.y; ++y)
				{
					// Interior faces carry an expanding velocity, wall faces remain zero.
					for (int x = 1; x != grid.m_cell_count.x; ++x)
						state.m_u_faces[grid.UFaceIndex(iv3{ x, y, z })] = (x - 0.5f * grid.m_cell_count.x) * 0.1f;
				}
				for (int y = 1; y != grid.m_cell_count.y; ++y)
				{
					// Interior faces carry an expanding velocity, wall faces remain zero.
					for (int x = 0; x != grid.m_cell_count.x; ++x)
						state.m_v_faces[grid.VFaceIndex(iv3{ x, y, z })] = (y - 0.5f * grid.m_cell_count.y) * 0.1f;
				}
			}
			auto before_stats = solver.Stats(state);
			solver.UploadState(job, state);
			job.Run();
			auto after_state = Run(solver, job, 1, 0.0f, AtmosphereStepSources{});
			auto after_stats = solver.Stats(after_state);
			std::printf("Atmosphere divergence V-cycles %d rms %.9f -> %.9f max %.9f -> %.9f\n", config.m_pressure_vcycles, before_stats.m_rms_divergence, after_stats.m_rms_divergence, before_stats.m_max_divergence, after_stats.m_max_divergence);
			PR_EXPECT(after_stats.m_rms_divergence < before_stats.m_rms_divergence * 0.01f);
			PR_EXPECT(after_stats.m_max_divergence < before_stats.m_max_divergence * 0.01f);
		}

		PRUnitTestMethod(SharedGpuStepCost, Quick)
		{
			// The solver records into a job that shares the same compute device and queue wrapper used by physics GPU work.
			auto gpu = Gpu{};
			auto job = GpuJob{ gpu.m_gpu, "AtmosphereTests.SharedGpu", 0xFF00AAFF, 1 };
			auto const measure = [&](iv3 cells)
			{
				// Exclude first-run shader work by constructing the solver and running one warm-up step before timing.
				auto config = Config(cells);
				config.m_pressure_vcycles = 3;
				config.m_pressure_pre_smooth = 8;
				config.m_pressure_post_smooth = 8;
				config.m_pressure_coarse_smooth = 96;
				auto solver = AtmosphereSolver{ gpu, config };
				solver.Step(job, 0.025f, AtmosphereStepSources{});
				job.Run();
				auto const steps = 5;
				auto const beg = std::chrono::steady_clock::now();
				for (int i = 0; i != steps; ++i)
				{
					solver.Step(job, 0.025f, AtmosphereStepSources{});
					job.Run();
				}
				auto const end = std::chrono::steady_clock::now();
				return std::chrono::duration<double, std::milli>(end - beg).count() / steps;
			};
			auto const small_ms = measure(iv3{ 64, 64, 8 });
			auto const large_ms = measure(iv3{ 250, 250, 16 });
			std::printf("Atmosphere shared GPU average wall step: 64x64x8 %.3f ms, 250x250x16 %.3f ms\n", small_ms, large_ms);
			PR_EXPECT(small_ms >= 0.0);
			PR_EXPECT(large_ms >= 0.0);
		}
	};


	PRUnitTestClass(AtmosphereTracerTests)
	{
		// Build a small flat-domain configuration for focused tracer tests.
		static AtmosphereConfig Config()
		{
			// A uniform grid keeps expected motion simple and independent of terrain metrics.
			return AtmosphereConfig{
				.m_grid = AtmosphereGrid{ .m_cell_count = iv3{ 8, 8, 4 }, .m_origin = v4::Zero(), .m_dx = 1.0f, .m_lid_z = 4.0f, .m_first_layer_thickness = 1.0f, .m_layer_stretch_power = 1.0f },
				.m_boundaries = AtmosphereBoundaries{ .m_x_min = EAtmosphereBoundary::Open, .m_x_max = EAtmosphereBoundary::Open, .m_y_min = EAtmosphereBoundary::Open, .m_y_max = EAtmosphereBoundary::Open },
				.m_reference = AtmosphereReferenceProfile{ .m_temperature_at_origin = 280.0f, .m_lapse_rate = 0.0f, .m_min_temperature = 200.0f },
				.m_pressure_vcycles = 1,
				.m_pressure_pre_smooth = 1,
				.m_pressure_post_smooth = 1,
				.m_pressure_coarse_smooth = 1,
			};
		}

		// Return true when a tracer particle is inside the configured domain.
		static bool Inside(AtmosphereConfig const& config, AtmosphereTracerParticle const& particle)
		{
			// The fixture has a flat floor, so simple box tests match the shader domain test.
			auto const& grid = config.m_grid;
			return particle.m_position.x >= grid.m_origin.x && particle.m_position.x <= grid.m_origin.x + grid.m_cell_count.x * grid.m_dx
				&& particle.m_position.y >= grid.m_origin.y && particle.m_position.y <= grid.m_origin.y + grid.m_cell_count.y * grid.m_dx
				&& particle.m_position.z >= grid.m_origin.z && particle.m_position.z <= grid.m_lid_z;
		}

		PRUnitTestMethod(UniformWindAdvectsAndRespawnsDeterministically, Quick)
		{
			// A uniform MAC wind should move every non-respawned tracer by the same distance, while short-lived tracers respawn inside the domain deterministically.
			auto gpu = Gpu{};
			auto job = GpuJob{ gpu.m_gpu, "AtmosphereTracerTests.UniformWind", 0xFF00AAFF, 1 };
			auto config = Config();
			auto solver = AtmosphereSolver{ gpu, config };
			auto state = AtmosphereState{
				.m_u_faces = std::vector<float>(config.m_grid.UFaceCount(), 1.0f),
				.m_v_faces = std::vector<float>(config.m_grid.VFaceCount(), 0.0f),
				.m_w_faces = std::vector<float>(config.m_grid.WFaceCount(), 0.0f),
				.m_temperature = std::vector<float>(config.m_grid.CellCount(), 300.0f),
				.m_pressure = std::vector<float>(config.m_grid.CellCount(), 0.0f),
			};
			solver.UploadState(job, state);
			job.Run();

			auto tracers = AtmosphereTracers{ solver, gpu, AtmosphereTracerConfig{ .m_particle_count = 64, .m_seed = 12345u, .m_max_age = 0.04f } };
			auto initial = tracers.ReadBack(job);
			tracers.Advect(job, 0.025f);
			auto moved = tracers.ReadBack(job);
			auto moved_count = 0;
			for (int i = 0; i != isize(moved); ++i)
			{
				PR_EXPECT(Inside(config, moved[i]));
				PR_EXPECT(std::abs(moved[i].m_temperature - 300.0f) < 1.0e-4f);
				if (moved[i].m_age > 0.0f)
				{
					PR_EXPECT(std::abs((moved[i].m_position.x - initial[i].m_position.x) - 0.025f) < 2.0e-3f);
					++moved_count;
				}
			}
			PR_EXPECT(moved_count > 0);

			tracers.Advect(job, 0.05f);
			auto respawned = tracers.ReadBack(job);
			for (auto const& particle : respawned)
			{
				PR_EXPECT(Inside(config, particle));
				PR_EXPECT(particle.m_age == 0.0f);
			}

			auto tracers_again = AtmosphereTracers{ solver, gpu, AtmosphereTracerConfig{ .m_particle_count = 64, .m_seed = 12345u, .m_max_age = 0.04f } };
			auto initial_again = tracers_again.ReadBack(job);
			for (int i = 0; i != isize(initial); ++i)
			{
				PR_EXPECT(Length(initial[i].m_position - initial_again[i].m_position) < 1.0e-5f);
			}
		}

		PRUnitTestMethod(StretchedSolidBoxWithForcing, Quick)
		{
			// A closed box with stretched layers, heat and pressure forcing, and thousands of tracers should keep every tracer inside the domain.
			auto gpu = Gpu{};
			auto job = GpuJob{ gpu.m_gpu, "AtmosphereTracerTests.SolidBox", 0xFF00AAFF, 1 };
			auto const solid = EAtmosphereBoundary::Solid;
			auto config = AtmosphereConfig{
				.m_grid = AtmosphereGrid{ .m_cell_count = iv3{ 64, 64, 8 }, .m_origin = v4{ -640.0f, -640.0f, 0.0f, 1.0f }, .m_dx = 20.0f, .m_lid_z = 360.0f, .m_first_layer_thickness = 8.0f, .m_layer_stretch_power = 0.72f },
				.m_boundaries = AtmosphereBoundaries{ .m_x_min = solid, .m_x_max = solid, .m_y_min = solid, .m_y_max = solid, .m_z_min = solid, .m_z_max = solid },
				.m_reference = AtmosphereReferenceProfile{ .m_temperature_at_origin = 288.0f, .m_lapse_rate = -0.004f, .m_min_temperature = 250.0f },
			};
			auto solver = AtmosphereSolver{ gpu, config };
			auto tracers = AtmosphereTracers{ solver, gpu, AtmosphereTracerConfig{ .m_particle_count = 4096, .m_seed = 12648430u, .m_max_age = 26.0f } };
			auto particles = tracers.ReadBack(job);

			auto heat = AtmosphereHeatSource{ .m_centre = v4{ 0.0f, -80.0f, 28.0f, 1.0f }, .m_radius = 95.0f, .m_heating_rate = 28.0f, .m_target_temperature = 306.0f, .m_relaxation_rate = 0.08f };
			auto nodes = std::array{
				AtmospherePressureNode{ .m_centre = v2{ -260.0f, -120.0f }, .m_strength = 24.0f, .m_radius = 170.0f, .m_lifetime = 180.0f },
				AtmospherePressureNode{ .m_centre = v2{ 260.0f, 160.0f }, .m_strength = -22.0f, .m_radius = 160.0f, .m_lifetime = 180.0f },
			};
			auto sources = AtmosphereStepSources{ .m_heat_sources = std::span{ &heat, 1 }, .m_pressure_nodes = nodes };
			for (int i = 0; i != 10; ++i)
			{
				// Each step submits solver and tracer work together, as an interactive caller would.
				solver.Step(job, 1.0f / 60.0f, sources);
				tracers.Advect(job, 1.0f / 60.0f);
				particles = tracers.ReadBack(job);
			}
			for (auto const& particle : particles)
				PR_EXPECT(Inside(config, particle));
		}

		PRUnitTestMethod(ReservoirWindCarriesTracers, Quick)
		{
			// A small wind tunnel with open west/east edges should carry tracers downwind at roughly the reservoir wind speed.
			auto gpu = Gpu{};
			auto job = GpuJob{ gpu.m_gpu, "AtmosphereTracerTests.WindTunnel", 0xFF00AAFF, 1 };
			auto const open = EAtmosphereBoundary::Open;
			auto const solid = EAtmosphereBoundary::Solid;
			auto config = AtmosphereConfig{
				.m_grid = AtmosphereGrid{ .m_cell_count = iv3{ 5, 5, 3 }, .m_origin = v4{ -50.0f, -50.0f, 0.0f, 1.0f }, .m_dx = 20.0f, .m_lid_z = 60.0f, .m_first_layer_thickness = 20.0f, .m_layer_stretch_power = 1.0f },
				.m_boundaries = AtmosphereBoundaries{ .m_x_min = open, .m_x_max = open, .m_y_min = solid, .m_y_max = solid, .m_z_min = solid, .m_z_max = solid },
				.m_reference = AtmosphereReferenceProfile{ .m_temperature_at_origin = 288.0f, .m_lapse_rate = -0.004f, .m_min_temperature = 250.0f },
			};
			auto solver = AtmosphereSolver{ gpu, config };
			auto tracers = AtmosphereTracers{ solver, gpu, AtmosphereTracerConfig{ .m_particle_count = 512, .m_seed = 12648430u, .m_max_age = 30.0f } };
			auto const sources = AtmosphereStepSources{ .m_reservoir_wind = v4{ 5.0f, 0.0f, 0.0f, 0.0f } };
			auto const dt = 1.0f / 60.0f;

			// Let the reservoir wind fill the tunnel before measuring.
			for (int i = 0; i != 300; ++i)
			{
				// Advance the solver and tracers together, as the sandbox does.
				solver.Step(job, dt, sources);
				tracers.Advect(job, dt);
			}
			auto const before = tracers.ReadBack(job);

			// Advance one second, then compare drift and respawn positions.
			for (int i = 0; i != 60; ++i)
			{
				// Same per-step order as the spin-up.
				solver.Step(job, dt, sources);
				tracers.Advect(job, dt);
			}
			auto const after = tracers.ReadBack(job);
			auto drift = 0.0f;
			auto count = 0;
			auto respawned = 0;
			for (int i = 0; i != isize(after); ++i)
			{
				// Respawned tracers must have re-entered through the west inflow face, so they can be at most one second of wind downstream of it.
				PR_EXPECT(Inside(config, after[i]));
				if (after[i].m_age < before[i].m_age)
				{
					PR_EXPECT(after[i].m_position.x < config.m_grid.m_origin.x + 6.0f);
					++respawned;
					continue;
				}

				// Measure drift only for tracers that start in the western half, so none reach the outflow edge.
				if (before[i].m_position.x >= 0.0f)
					continue;

				drift += after[i].m_position.x - before[i].m_position.x;
				++count;
			}
			PR_EXPECT(respawned > 0);
			PR_EXPECT(count > 50);
			auto const mean_speed = drift / std::max(count, 1);
			std::printf("Atmosphere wind tunnel tracer speed %f m/s target 5.000000 count %d\n", mean_speed, count);
			PR_EXPECT(mean_speed > 3.5f && mean_speed < 6.5f);
		}
	};

	PRUnitTestClass(AtmosphereTerrainTests)
	{
		// Return the large terrain-following fixture used by the domain-scale tests.
		static AtmosphereConfig Config(iv3 cells, std::vector<float> floors)
		{
			// These tests use a 1/15 s step with the configured V-cycle solver.
			return AtmosphereConfig{
				.m_grid = AtmosphereGrid{ .m_cell_count = cells, .m_origin = v4{ -0.5f * cells.x * 32.0f, -0.5f * cells.y * 32.0f, 0.0f, 1.0f }, .m_dx = 32.0f, .m_lid_z = 1500.0f, .m_first_layer_thickness = 5.0f, .m_layer_stretch_power = 0.58f, .m_floor_heights = std::move(floors) },
				.m_boundaries = AtmosphereBoundaries{ .m_x_min = EAtmosphereBoundary::Open, .m_x_max = EAtmosphereBoundary::Open, .m_y_min = EAtmosphereBoundary::Open, .m_y_max = EAtmosphereBoundary::Open, .m_z_min = EAtmosphereBoundary::Solid, .m_z_max = EAtmosphereBoundary::Solid },
				.m_reference = AtmosphereReferenceProfile{ .m_temperature_at_origin = 288.0f, .m_lapse_rate = -0.004f, .m_min_temperature = 250.0f },
				.m_gravity = 9.80665f,
				.m_floor_exchange_rate = 0.0f,
				.m_lid_temperature = 282.0f,
				.m_lid_relaxation_rate = 0.0f,
				.m_surface_forcing_height = 600.0f,
				.m_pressure_vcycles = 3,
				.m_pressure_pre_smooth = 12,
				.m_pressure_post_smooth = 12,
				.m_pressure_coarse_smooth = 128,
			};
		}

		// Build the 600 m high, roughly 1 km wide Gaussian mountain fixture.
		static std::vector<float> Mountain(iv3 cells, float height = 600.0f, float width = 500.0f)
		{
			// The width is the Gaussian scale in metres, giving a visually about-one-kilometre mountain across the steep central slopes.
			auto floors = std::vector<float>(cells.x * cells.y, 0.0f);
			for (int y = 0; y != cells.y; ++y)
			{
				// Rows share the same northing offset.
				for (int x = 0; x != cells.x; ++x)
				{
					// Cell centres are measured in metres relative to the domain centre.
					auto const px = (x + 0.5f - 0.5f * cells.x) * 32.0f;
					auto const py = (y + 0.5f - 0.5f * cells.y) * 32.0f;
					floors[y * cells.x + x] = height * std::exp(-(px * px + py * py) / (width * width));
				}
			}
			return floors;
		}

		// Run a synchronous sequence and return the final readback.
		static AtmosphereState Run(AtmosphereSolver& solver, GpuJob& job, int steps, float dt, AtmosphereStepSources const& sources)
		{
			// Synchronous execution gives deterministic diagnostics and keeps temporary upload memory alive until each step completes.
			for (int i = 0; i != steps; ++i)
			{
				// Each solver step is complete before the next one records.
				solver.Step(job, dt, sources);
				job.Run();
			}
			return solver.ReadBack(job);
		}

		// Return the average cell-centred velocity in a small box.
		static v4 MeanVelocity(AtmosphereSolver const& solver, AtmosphereState const& state, iv3 lo, iv3 hi)
		{
			// Box means reduce grid-cell noise in long GPU runs.
			auto const cells = solver.CellStates(state);
			auto sum = v4::Zero();
			auto count = 0;
			for (int z = lo.z; z != hi.z; ++z)
			{
				// Scan every requested layer.
				for (int y = lo.y; y != hi.y; ++y)
				{
					// Scan every requested row.
					for (int x = lo.x; x != hi.x; ++x)
					{
						// Accumulate cell-centred velocity samples.
						sum += cells[solver.Config().m_grid.CellIndex(iv3{ x, y, z })].m_velocity;
						++count;
					}
				}
			}
			return sum / static_cast<float>(std::max(1, count));
		}

		// Return the maximum cell-centred speed in the selected box.
		static float MaxSpeed(AtmosphereSolver const& solver, AtmosphereState const& state, iv3 lo, iv3 hi)
		{
			// Maximum speed checks catch unstable cells even when the average flow looks plausible.
			auto const cells = solver.CellStates(state);
			auto result = 0.0f;
			for (int z = lo.z; z != hi.z; ++z)
			{
				// Scan every requested layer.
				for (int y = lo.y; y != hi.y; ++y)
				{
					// Scan every requested row.
					for (int x = lo.x; x != hi.x; ++x)
					{
						// Track the largest velocity magnitude.
						result = std::max(result, Length(cells[solver.Config().m_grid.CellIndex(iv3{ x, y, z })].m_velocity.xyz));
					}
				}
			}
			return result;
		}

		// Return the volume-weighted heat proxy for a state.
		static double Heat(AtmosphereConfig const& config, AtmosphereState const& state)
		{
			// Density is constant in the Boussinesq solver, so temperature times volume is the conserved heat proxy.
			auto const& grid = config.m_grid;
			auto sum = 0.0;
			for (int z = 0; z != grid.m_cell_count.z; ++z)
			{
				// Accumulate one physical layer at a time.
				for (int y = 0; y != grid.m_cell_count.y; ++y)
				{
					// Cell volumes differ by column because the terrain-following layers have different heights.
					for (int x = 0; x != grid.m_cell_count.x; ++x)
					{
						// Horizontal area is constant for every column.
						auto const column = iv2{ x, y };
						auto const volume = grid.m_dx * grid.m_dx * grid.CellHeight(column, z);
						sum += state.m_temperature[grid.CellIndex(iv3{ x, y, z })] * volume;
					}
				}
			}
			return sum;
		}

		PRUnitTestMethod(FalseWindOverMountain, Quick)
		{
			// Still, stably stratified air over a steep mountain should not create material false wind from the terrain-following pressure metric.
			auto const cells = iv3{ 96, 96, 16 };
			auto gpu = Gpu{};
			auto job = GpuJob{ gpu.m_gpu, "AtmosphereTerrainTests.FalseWind", 0xFF00AAFF, 1 };
			auto solver = AtmosphereSolver{ gpu, Config(cells, Mountain(cells)) };
			auto state = Run(solver, job, 900, 1.0f / 15.0f, AtmosphereStepSources{});
			auto stats = solver.Stats(state);
			std::printf("Atmosphere false wind max_speed %.6f m/s threshold 0.100000 rms_div %.9f\n", stats.m_max_speed, stats.m_rms_divergence);
			PR_EXPECT(stats.m_max_speed < 0.1f);
		}

		PRUnitTestMethod(FlowAroundOrOverMountain, Quick)
		{
			// A 5 m/s reservoir wind should remain bounded and show terrain-induced acceleration or deflection around the mountain fixture.
			auto const cells = iv3{ 96, 96, 16 };
			auto gpu = Gpu{};
			auto job = GpuJob{ gpu.m_gpu, "AtmosphereTerrainTests.FlowMountain", 0xFF00AAFF, 1 };
			auto solver = AtmosphereSolver{ gpu, Config(cells, Mountain(cells)) };
			auto sources = AtmosphereStepSources{ .m_reservoir_wind = v4{ 5.0f, 0.0f, 0.0f, 0.0f } };
			auto state = Run(solver, job, 450, 1.0f / 15.0f, sources);
			auto stats = solver.Stats(state);
			auto upstream = MeanVelocity(solver, state, iv3{ 8, 44, 1 }, iv3{ 16, 52, 4 });
			auto crest = MeanVelocity(solver, state, iv3{ 46, 46, 2 }, iv3{ 50, 50, 6 });
			auto north_flank = MeanVelocity(solver, state, iv3{ 44, 58, 1 }, iv3{ 52, 66, 4 });
			auto south_flank = MeanVelocity(solver, state, iv3{ 44, 30, 1 }, iv3{ 52, 38, 4 });
			auto deflection = std::abs(north_flank.y) + std::abs(south_flank.y);
			auto mountain_effect = std::max(deflection, Length(upstream.xyz) - Length(crest.xyz));
			auto flank_sign = north_flank.y - south_flank.y;
			std::printf("Atmosphere mountain flow upstream %.6f crest %.6f crest_w %.6f terrain_effect %.6f threshold 0.050000 flank_sign %.6f max_speed %.6f threshold %.6f\n", Length(upstream.xyz), Length(crest.xyz), crest.z, mountain_effect, flank_sign, stats.m_max_speed, 16.0f);
			PR_EXPECT(mountain_effect > 0.05f);
			PR_EXPECT(crest.z > 0.015f || deflection > 0.005f);
			PR_EXPECT(stats.m_max_speed < 16.0f);
		}

		PRUnitTestMethod(ColdDrainageSlope, Quick)
		{
			// A cold floor on a real 15 degree slope should produce downslope near-floor flow within one simulated minute.
			auto const cells = iv3{ 64, 32, 16 };
			auto floors = std::vector<float>(cells.x * cells.y, 0.0f);
			auto const slope = std::tan(15.0f * pr::math::constants<float>::tau_by_360);
			for (int y = 0; y != cells.y; ++y)
			{
				// Rows share the same east-west slope.
				for (int x = 0; x != cells.x; ++x)
					floors[y * cells.x + x] = (x - 0.5f * cells.x) * 32.0f * slope;
			}
			auto gpu = Gpu{};
			auto job = GpuJob{ gpu.m_gpu, "AtmosphereTerrainTests.Drainage", 0xFF00AAFF, 1 };
			auto config = Config(cells, floors);
			config.m_floor_exchange_rate = 0.25f;
			auto solver = AtmosphereSolver{ gpu, config };
			auto cold = std::vector<float>(cells.x * cells.y, 278.0f);
			auto state = Run(solver, job, 900, 1.0f / 15.0f, AtmosphereStepSources{ .m_floor_temperatures = cold, .m_reservoir_temperature_offset = 6.0f });
			auto downslope = -MeanVelocity(solver, state, iv3{ 24, 12, 0 }, iv3{ 40, 20, 3 }).x;
			std::printf("Atmosphere cold drainage downslope %.6f m/s threshold 0.200000\n", downslope);
			PR_EXPECT(downslope > 0.2f);
		}

		PRUnitTestMethod(FloorHeightRemap, Quick)
		{
			// Raising part of the floor from 0 m to 20 m should remap only changed columns and preserve heat in the remaining air volume.
			auto const cells = iv3{ 48, 32, 16 };
			auto floors = std::vector<float>(cells.x * cells.y, 0.0f);
			for (int y = 0; y != cells.y; ++y)
			{
				// Build a sloped floor with high ground and low basins.
				for (int x = 0; x != cells.x; ++x)
					floors[y * cells.x + x] = -50.0f + 250.0f * static_cast<float>(x) / static_cast<float>(cells.x - 1);
			}
			auto gpu = Gpu{};
			auto job = GpuJob{ gpu.m_gpu, "AtmosphereTerrainTests.Remap", 0xFF00AAFF, 1 };
			auto config = Config(cells, floors);
			auto solver = AtmosphereSolver{ gpu, config };
			auto before = solver.ReadBack(job);
			auto before_heat = Heat(solver.Config(), before);
			auto raised = floors;
			for (auto& floor_height : raised)
			{
				// Low columns use the raised surface as the new floor.
				floor_height = std::max(floor_height, 20.0f);
			}
			solver.RemapFloors(job, raised);
			job.Run();
			auto after = solver.ReadBack(job);
			auto after_heat = Heat(solver.Config(), after);
			auto unchanged_equal = true;
			auto changed_geometry = false;
			for (int y = 0; y != cells.y; ++y)
			{
				// Compare unchanged high-ground columns exactly.
				for (int x = 0; x != cells.x; ++x)
				{
					// The remap contract promises bit-identical values only where the floor is unchanged.
					auto const changed = raised[y * cells.x + x] != floors[y * cells.x + x];
					changed_geometry = changed_geometry || (changed && solver.Config().m_grid.FloorHeight(iv2{ x, y }) == raised[y * cells.x + x]);
					if (changed)
						continue;

					for (int z = 0; z != cells.z; ++z)
					{
						// Cell-centred temperature and every adjacent face component must remain untouched.
						unchanged_equal = unchanged_equal && before.m_temperature[config.m_grid.CellIndex(iv3{ x, y, z })] == after.m_temperature[solver.Config().m_grid.CellIndex(iv3{ x, y, z })];
						unchanged_equal = unchanged_equal && before.m_u_faces[config.m_grid.UFaceIndex(iv3{ x, y, z })] == after.m_u_faces[solver.Config().m_grid.UFaceIndex(iv3{ x, y, z })];
						unchanged_equal = unchanged_equal && before.m_u_faces[config.m_grid.UFaceIndex(iv3{ x + 1, y, z })] == after.m_u_faces[solver.Config().m_grid.UFaceIndex(iv3{ x + 1, y, z })];
						unchanged_equal = unchanged_equal && before.m_v_faces[config.m_grid.VFaceIndex(iv3{ x, y, z })] == after.m_v_faces[solver.Config().m_grid.VFaceIndex(iv3{ x, y, z })];
						unchanged_equal = unchanged_equal && before.m_v_faces[config.m_grid.VFaceIndex(iv3{ x, y + 1, z })] == after.m_v_faces[solver.Config().m_grid.VFaceIndex(iv3{ x, y + 1, z })];
						unchanged_equal = unchanged_equal && before.m_w_faces[config.m_grid.WFaceIndex(iv3{ x, y, z })] == after.m_w_faces[solver.Config().m_grid.WFaceIndex(iv3{ x, y, z })];
						unchanged_equal = unchanged_equal && before.m_w_faces[config.m_grid.WFaceIndex(iv3{ x, y, z + 1 })] == after.m_w_faces[solver.Config().m_grid.WFaceIndex(iv3{ x, y, z + 1 })];
					}
				}
			}
			auto const relative_heat_change = std::abs(after_heat - before_heat) / std::max(1.0, std::abs(before_heat));
			std::printf("Atmosphere floor remap unchanged %d changed_geometry %d heat_relative_change %.9f threshold 0.020000\n", unchanged_equal ? 1 : 0, changed_geometry ? 1 : 0, relative_heat_change);
			PR_EXPECT(unchanged_equal);
			PR_EXPECT(changed_geometry);
			PR_EXPECT(relative_heat_change < 0.02);
		}

		PRUnitTestMethod(OpenEdgesReservoirWind, Quick)
		{
			// Open edges should admit a 5 m/s reservoir wind without overshoot or material divergence after one simulated minute.
			auto const cells = iv3{ 96, 96, 16 };
			auto gpu = Gpu{};
			auto job = GpuJob{ gpu.m_gpu, "AtmosphereTerrainTests.OpenEdges", 0xFF00AAFF, 1 };
			auto solver = AtmosphereSolver{ gpu, Config(cells, std::vector<float>(cells.x * cells.y, 0.0f)) };
			auto state = Run(solver, job, 900, 1.0f / 15.0f, AtmosphereStepSources{ .m_reservoir_wind = v4{ 5.0f, 0.0f, 0.0f, 0.0f } });
			auto stats = solver.Stats(state);
			auto interior = MeanVelocity(solver, state, iv3{ 32, 32, 1 }, iv3{ 64, 64, 8 });
			std::printf("Atmosphere open edges interior %.6f m/s target 5.000000 max_speed %.6f threshold 7.500000 rms_div %.9f threshold 0.005000\n", interior.x, stats.m_max_speed, stats.m_rms_divergence);
			PR_EXPECT(std::abs(interior.x - 5.0f) < 0.5f);
			PR_EXPECT(stats.m_max_speed < 7.5f);
			PR_EXPECT(stats.m_rms_divergence < 0.005f);
		}

		PRUnitTestMethod(PressureNodesDeterministicAndSigns, Quick)
		{
			// Node state is ordinary data; evolving a copied saved state by the same dt must produce identical fields.
			auto a = AtmospherePressureForcingState::Create(1234u, 6, 4000.0f, 0.4f, 2.0f);
			auto b = a;
			a.Evolve(10.0f, 4000.0f, 0.4f, 2.0f);
			b.Evolve(10.0f, 4000.0f, 0.4f, 2.0f);
			auto same = a.m_time_s == b.m_time_s && a.m_nodes.size() == b.m_nodes.size();
			for (int i = 0; same && i != isize(a.m_nodes); ++i)
			{
				// Bitwise equality is expected because evolution uses only saved scalar state.
				same = memcmp(&a.m_nodes[i], &b.m_nodes[i], sizeof(AtmospherePressureNode)) == 0;
			}

			// A high node pushes air away and warms reservoir inflow; a low node pulls air inward and cools it.
			auto const cells = iv3{ 64, 64, 16 };
			auto gpu = Gpu{};
			auto job = GpuJob{ gpu.m_gpu, "AtmosphereTerrainTests.NodeSigns", 0xFF00AAFF, 1 };
			auto high = AtmospherePressureNode{ .m_centre = v2{ 0.0f, 0.0f }, .m_strength = 60.0f, .m_radius = 700.0f, .m_lifetime = 100.0f, .m_temperature_offset = 4.0f };
			auto high_solver = AtmosphereSolver{ gpu, Config(cells, std::vector<float>(cells.x * cells.y, 0.0f)) };
			auto high_state = Run(high_solver, job, 20, 1.0f / 15.0f, AtmosphereStepSources{ .m_pressure_nodes = std::span{ &high, 1 } });
			auto low = high;
			low.m_strength = -high.m_strength;
			low.m_temperature_offset = -high.m_temperature_offset;
			auto low_solver = AtmosphereSolver{ gpu, Config(cells, std::vector<float>(cells.x * cells.y, 0.0f)) };
			auto low_state = Run(low_solver, job, 20, 1.0f / 15.0f, AtmosphereStepSources{ .m_pressure_nodes = std::span{ &low, 1 } });
			auto high_west = MeanVelocity(high_solver, high_state, iv3{ 20, 30, 0 }, iv3{ 24, 34, 2 }).x;
			auto high_east = MeanVelocity(high_solver, high_state, iv3{ 40, 30, 0 }, iv3{ 44, 34, 2 }).x;
			auto low_west = MeanVelocity(low_solver, low_state, iv3{ 20, 30, 0 }, iv3{ 24, 34, 2 }).x;
			auto low_east = MeanVelocity(low_solver, low_state, iv3{ 40, 30, 0 }, iv3{ 44, 34, 2 }).x;
			auto high_temp_solver = AtmosphereSolver{ gpu, Config(cells, std::vector<float>(cells.x * cells.y, 0.0f)) };
			auto high_temp_state = Run(high_temp_solver, job, 20, 1.0f / 15.0f, AtmosphereStepSources{ .m_pressure_nodes = std::span{ &high, 1 }, .m_reservoir_wind = v4{ 5.0f, 0.0f, 0.0f, 0.0f } });
			auto low_temp_solver = AtmosphereSolver{ gpu, Config(cells, std::vector<float>(cells.x * cells.y, 0.0f)) };
			auto low_temp_state = Run(low_temp_solver, job, 20, 1.0f / 15.0f, AtmosphereStepSources{ .m_pressure_nodes = std::span{ &low, 1 }, .m_reservoir_wind = v4{ 5.0f, 0.0f, 0.0f, 0.0f } });
			auto high_temp = high_temp_state.m_temperature[high_temp_solver.Config().m_grid.CellIndex(iv3{ 0, 32, 0 })] - high_temp_solver.Config().m_reference.Temperature(high_temp_solver.Config().m_grid.CellCentre(iv3{ 0, 32, 0 }).z);
			auto low_temp = low_temp_state.m_temperature[low_temp_solver.Config().m_grid.CellIndex(iv3{ 0, 32, 0 })] - low_temp_solver.Config().m_reference.Temperature(low_temp_solver.Config().m_grid.CellCentre(iv3{ 0, 32, 0 }).z);
			std::printf("Atmosphere pressure nodes deterministic %d high_west %.6f high_east %.6f low_west %.6f low_east %.6f high_inflow_dt %.6f low_inflow_dt %.6f\n", same ? 1 : 0, high_west, high_east, low_west, low_east, high_temp, low_temp);
			PR_EXPECT(same);
			PR_EXPECT(high_west < 0.0f && high_east > 0.0f);
			PR_EXPECT(low_west > 0.0f && low_east < 0.0f);
			PR_EXPECT(high_temp > 0.0f && low_temp < 0.0f);
		}

		PRUnitTestMethod(RuntimeDivergenceReduction, Quick)
		{
			// The target 250x250x16 terrain grid should remove long-wave divergence that stationary smoothers cannot reach at this scale.
			auto const cells = iv3{ 250, 250, 16 };
			auto floors = std::vector<float>(cells.x * cells.y, 0.0f);
			auto gpu = Gpu{};
			auto job = GpuJob{ gpu.m_gpu, "AtmosphereTerrainTests.RuntimeDivergence", 0xFF00AAFF, 1 };
			auto config = Config(cells, floors);
			config.m_boundaries.m_x_min = EAtmosphereBoundary::Solid;
			config.m_boundaries.m_x_max = EAtmosphereBoundary::Solid;
			config.m_boundaries.m_y_min = EAtmosphereBoundary::Solid;
			config.m_boundaries.m_y_max = EAtmosphereBoundary::Solid;
			config.m_pressure_pre_smooth = 64;
			config.m_pressure_post_smooth = 64;
			config.m_pressure_coarse_smooth = 256;
			auto solver = AtmosphereSolver{ gpu, config };
			auto const& grid = solver.Config().m_grid;
			auto state = AtmosphereState{
				.m_u_faces = std::vector<float>(grid.UFaceCount(), 0.0f),
				.m_v_faces = std::vector<float>(grid.VFaceCount(), 0.0f),
				.m_w_faces = std::vector<float>(grid.WFaceCount(), 0.0f),
				.m_temperature = std::vector<float>(grid.CellCount(), 0.0f),
				.m_pressure = std::vector<float>(grid.CellCount(), 0.0f),
			};
			auto potential = std::vector<float>(cells.x * cells.y, 0.0f);
			for (int y = 0; y != cells.y; ++y)
			{
				// Use a smooth Neumann pressure mode so the target pressure is representable by the solid side conditions.
				for (int x = 0; x != cells.x; ++x)
					potential[y * cells.x + x] = 1000.0f * std::cos(pr::constants<float>::tau_by_2 * static_cast<float>(x + 0.5f) / static_cast<float>(cells.x)) * std::cos(pr::constants<float>::tau_by_2 * static_cast<float>(y + 0.5f) / static_cast<float>(cells.y));
			}
			for (int z = 0; z != cells.z; ++z)
			{
				// A domain-scale pressure-gradient wind exposes the long wavelengths that made the old pressure solve stall, while staying compatible with solid side boundaries.
				for (int y = 0; y != cells.y; ++y)
				{
					for (int x = 0; x != cells.x; ++x)
						state.m_temperature[grid.CellIndex(iv3{ x, y, z })] = solver.Config().m_reference.Temperature(grid.CellCentre(iv3{ x, y, z }).z);

					for (int x = 0; x != cells.x + 1; ++x)
					{
						if (x != 0 && x != cells.x)
							state.m_u_faces[grid.UFaceIndex(iv3{ x, y, z })] = (potential[y * cells.x + x] - potential[y * cells.x + x - 1]) / grid.m_dx;
					}
				}
				for (int y = 0; y != cells.y + 1; ++y)
				{
					// The paired Y gradient gives the V-cycle a genuinely two-dimensional low-frequency mode to remove.
					for (int x = 0; x != cells.x; ++x)
					{
						if (y != 0 && y != cells.y)
							state.m_v_faces[grid.VFaceIndex(iv3{ x, y, z })] = (potential[y * cells.x + x] - potential[(y - 1) * cells.x + x]) / grid.m_dx;
					}
				}
			}
			auto before_stats = solver.Stats(state);
			solver.UploadState(job, state);
			job.Run();
			solver.Step(job, 0.0f, AtmosphereStepSources{});
			job.Run();
			auto after = solver.ReadBack(job);
			auto after_stats = solver.Stats(after);
			auto const relative = after_stats.m_rms_divergence / std::max(before_stats.m_rms_divergence, 1.0e-7f);
			std::printf("Atmosphere runtime divergence rms_before %.9f rms_after %.9f relative %.6f threshold 0.010000 max_speed %.6f\n", before_stats.m_rms_divergence, after_stats.m_rms_divergence, relative, after_stats.m_max_speed);
			PR_EXPECT(relative < 0.01f);
			PR_EXPECT(after_stats.m_rms_divergence < 0.01f);
		}
	};

}
#endif
