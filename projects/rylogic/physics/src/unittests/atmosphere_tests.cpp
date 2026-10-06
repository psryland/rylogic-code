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

	// Return outside air with the same wind and temperature offset beside every boundary column of 'grid'.
	static std::vector<AtmosphereOutsideAir> UniformOutsideAir(AtmosphereGrid const& grid, v2 wind, float temperature_offset = 0.0f)
	{
		// Every sample position gets the same air.
		return grid.BuildOutsideAir([=](v2)
		{
			return AtmosphereOutsideAir{ .m_wind = wind, .m_temperature_offset = temperature_offset };
		});
	}

	PRUnitTestClass(AtmosphereTests)
	{
		// Build a small stable default configuration for focused solver tests.
		static AtmosphereConfig Config(iv3 cells = iv3{ 16, 16, 8 })
		{
			// Use metre-scale cells and a mild reference lapse so buoyancy is easy to observe in short runs.
			return AtmosphereConfig{
				.m_grid = AtmosphereGrid{ .m_cell_count = cells, .m_origin = v4::Zero(), .m_dx = 1.0f, .m_lid_z = static_cast<float>(cells.z), .m_first_layer_thickness = 1.0f },
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

			bad = config;
			bad.m_wall_drag.m_z_min = -0.01f;
			PR_THROWS(bad.Validate(), std::invalid_argument);

			bad = config;
			bad.m_boundaries.m_x_min = EAtmosphereBoundary::Open;
			bad.m_wall_drag.m_x_min = 0.01f;
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

		PRUnitTestMethod(LargeOpenGridProjectionConverges, Quick)
		{
			// Deep, odd-sized multigrid hierarchies with open sides must keep the coarse levels' zero-pressure boundary at the fine domain edge.
			// A misplaced coarse boundary over-corrects the largest-scale pressure mode, so repeated projections of the same field grow without bound.
			auto gpu = Gpu{};
			auto job = GpuJob{ gpu.m_gpu, "AtmosphereTests.LargeOpenGrid", 0xFF00AAFF, 1 };
			auto const n = 136;
			auto const open = EAtmosphereBoundary::Open;
			auto config = AtmosphereConfig{
				.m_grid = AtmosphereGrid{ .m_cell_count = iv3{ n, n, 12 }, .m_origin = v4{ -16.0f * n, -16.0f * n, -100.0f, 1.0f }, .m_dx = 32.0f, .m_lid_z = 1500.0f, .m_first_layer_thickness = 5.0f },
				.m_boundaries = AtmosphereBoundaries{ .m_x_min = open, .m_x_max = open, .m_y_min = open, .m_y_max = open },
			};
			config.m_open_edge_band = 8;
			auto solver = AtmosphereSolver{ gpu, config };

			// Fill the interior horizontal faces with deterministic noise so every pressure scale has divergence to remove.
			auto const& grid = config.m_grid;
			auto state = solver.ReadBack(job);
			auto rng = 12345u;
			auto noise = [&]
			{
				// A small linear congruential generator in [-0.5, 0.5).
				rng = rng * 1664525u + 1013904223u;
				return static_cast<float>(rng >> 8) / 16777216.0f - 0.5f;
			};
			for (int z = 0; z != grid.m_cell_count.z; ++z)
			{
				// Fill one layer of U and V faces, skipping the boundary faces.
				for (int y = 0; y != grid.m_cell_count.y; ++y)
				{
					for (int x = 1; x != grid.m_cell_count.x; ++x)
						state.m_u_faces[grid.UFaceIndex(iv3{ x, y, z })] = noise();
				}
				for (int y = 1; y != grid.m_cell_count.y; ++y)
				{
					for (int x = 0; x != grid.m_cell_count.x; ++x)
						state.m_v_faces[grid.VFaceIndex(iv3{ x, y, z })] = noise();
				}
			}
			auto initial = solver.Stats(state);
			solver.UploadState(job, state);
			job.Run();

			// Zero-length steps only project the field, so divergence must fall and speed must stay near its initial size.
			auto stats = solver.Stats(Run(solver, job, 8, 0.0f, AtmosphereStepSources{}));
			std::printf("Atmosphere large open grid rms_div %.6f -> %.6f max_speed %.6f -> %.6f\n", initial.m_rms_divergence, stats.m_rms_divergence, initial.m_max_speed, stats.m_max_speed);
			PR_EXPECT(stats.m_rms_divergence < 0.05f * initial.m_rms_divergence);
			PR_EXPECT(stats.m_max_speed < 2.0f * initial.m_max_speed);
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
				// Exclude first-run shader work by constructing the solver and running one warm-up step before timing. The default solver settings are timed.
				auto config = Config(cells);
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
				.m_grid = AtmosphereGrid{ .m_cell_count = iv3{ 8, 8, 4 }, .m_origin = v4::Zero(), .m_dx = 1.0f, .m_lid_z = 4.0f, .m_first_layer_thickness = 1.0f },
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

		PRUnitTestMethod(GroundDensityConcentratesTracersLow, Quick)
		{
			// Densities 10 at the floor, 1 at a fifth of the height, and 0.1 above sum to 1.18. That places 0.775/1.18 of new tracers
			// below a tenth of the column and 0.08/1.18 above a fifth.
			auto gpu = Gpu{};
			auto job = GpuJob{ gpu.m_gpu, "AtmosphereTracerTests.GroundDensity", 0xFF00AAFF, 1 };
			auto config = Config();
			auto solver = AtmosphereSolver{ gpu, config };
			auto tracers = AtmosphereTracers{ solver, gpu, AtmosphereTracerConfig{ .m_particle_count = 16384, .m_seed = 777u, .m_max_age = 10.0f, .m_ground_density = 10.0f, .m_break_density = 1.0f, .m_upper_density = 0.1f, .m_break_height = 0.2f } };
			auto particles = tracers.ReadBack(job);

			// Count tracers by height band. The fixture floor is at zero, so height over the lid is the column fraction.
			auto low = 0;
			auto high = 0;
			for (auto const& particle : particles)
			{
				// Each tracer must still start inside the domain.
				PR_EXPECT(Inside(config, particle));
				auto const s = particle.m_position.z / config.m_grid.m_lid_z;
				low += s < 0.1f ? 1 : 0;
				high += s > 0.2f ? 1 : 0;
			}
			auto const low_share = static_cast<float>(low) / isize(particles);
			auto const high_share = static_cast<float>(high) / isize(particles);
			std::printf("Atmosphere tracer ground density low %.4f (%.4f) high %.4f (%.4f)\n", low_share, 0.775f / 1.18f, high_share, 0.08f / 1.18f);
			PR_EXPECT(std::abs(low_share - 0.775f / 1.18f) < 0.02f);
			PR_EXPECT(std::abs(high_share - 0.08f / 1.18f) < 0.01f);
		}

		PRUnitTestMethod(StretchedSolidBoxWithHeat, Quick)
		{
			// A closed box with stretched layers, a heat source, and thousands of tracers should keep every tracer inside the domain.
			auto gpu = Gpu{};
			auto job = GpuJob{ gpu.m_gpu, "AtmosphereTracerTests.SolidBox", 0xFF00AAFF, 1 };
			auto const solid = EAtmosphereBoundary::Solid;
			auto config = AtmosphereConfig{
				.m_grid = AtmosphereGrid{ .m_cell_count = iv3{ 64, 64, 8 }, .m_origin = v4{ -640.0f, -640.0f, 0.0f, 1.0f }, .m_dx = 20.0f, .m_lid_z = 360.0f, .m_first_layer_thickness = 8.0f },
				.m_boundaries = AtmosphereBoundaries{ .m_x_min = solid, .m_x_max = solid, .m_y_min = solid, .m_y_max = solid, .m_z_min = solid, .m_z_max = solid },
				.m_reference = AtmosphereReferenceProfile{ .m_temperature_at_origin = 288.0f, .m_lapse_rate = -0.004f, .m_min_temperature = 250.0f },
			};
			auto solver = AtmosphereSolver{ gpu, config };
			auto tracers = AtmosphereTracers{ solver, gpu, AtmosphereTracerConfig{ .m_particle_count = 4096, .m_seed = 12648430u, .m_max_age = 26.0f } };
			auto particles = tracers.ReadBack(job);

			auto heat = AtmosphereHeatSource{ .m_centre = v4{ 0.0f, -80.0f, 28.0f, 1.0f }, .m_radius = 95.0f, .m_heating_rate = 28.0f, .m_target_temperature = 306.0f, .m_relaxation_rate = 0.08f };
			auto sources = AtmosphereStepSources{ .m_heat_sources = std::span{ &heat, 1 } };
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

		PRUnitTestMethod(OutsideWindCarriesTracers, Quick)
		{
			// A small wind tunnel with open west/east edges should carry tracers downwind at roughly the outside wind speed.
			auto gpu = Gpu{};
			auto job = GpuJob{ gpu.m_gpu, "AtmosphereTracerTests.WindTunnel", 0xFF00AAFF, 1 };
			auto const open = EAtmosphereBoundary::Open;
			auto const solid = EAtmosphereBoundary::Solid;
			auto config = AtmosphereConfig{
				.m_grid = AtmosphereGrid{ .m_cell_count = iv3{ 5, 5, 3 }, .m_origin = v4{ -50.0f, -50.0f, 0.0f, 1.0f }, .m_dx = 20.0f, .m_lid_z = 60.0f, .m_first_layer_thickness = 20.0f },
				.m_boundaries = AtmosphereBoundaries{ .m_x_min = open, .m_x_max = open, .m_y_min = solid, .m_y_max = solid, .m_z_min = solid, .m_z_max = solid },
				.m_reference = AtmosphereReferenceProfile{ .m_temperature_at_origin = 288.0f, .m_lapse_rate = -0.004f, .m_min_temperature = 250.0f },
			};
			auto solver = AtmosphereSolver{ gpu, config };
			// Initial ages are staggered across the lifetime, so use a lifetime long enough that no tracer expires by age and every respawn is an inflow respawn.
			auto tracers = AtmosphereTracers{ solver, gpu, AtmosphereTracerConfig{ .m_particle_count = 512, .m_seed = 12648430u, .m_max_age = 1.0e6f } };
			auto const outside_air = UniformOutsideAir(config.m_grid, v2{ 5.0f, 0.0f });
			auto const sources = AtmosphereStepSources{ .m_outside_air = outside_air };
			auto const dt = 1.0f / 60.0f;

			// Let the outside wind fill the tunnel before measuring.
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

		PRUnitTestMethod(OpposingOutsideWindDrivesCounterFlow, Quick)
		{
			// Opposing outside winds on the open ends of a tunnel should drive west-to-east flow in the southern half and east-to-west flow in the northern half.
			auto gpu = Gpu{};
			auto job = GpuJob{ gpu.m_gpu, "AtmosphereTracerTests.CounterFlow", 0xFF00AAFF, 1 };
			auto const open = EAtmosphereBoundary::Open;
			auto const solid = EAtmosphereBoundary::Solid;
			auto config = AtmosphereConfig{
				.m_grid = AtmosphereGrid{ .m_cell_count = iv3{ 5, 5, 3 }, .m_origin = v4{ -50.0f, -50.0f, 0.0f, 1.0f }, .m_dx = 20.0f, .m_lid_z = 60.0f, .m_first_layer_thickness = 20.0f },
				.m_boundaries = AtmosphereBoundaries{ .m_x_min = open, .m_x_max = open, .m_y_min = solid, .m_y_max = solid, .m_z_min = solid, .m_z_max = solid },
				.m_reference = AtmosphereReferenceProfile{ .m_temperature_at_origin = 288.0f, .m_lapse_rate = -0.004f, .m_min_temperature = 250.0f },
			};
			auto solver = AtmosphereSolver{ gpu, config };
			// Initial ages are staggered across the lifetime, so use a lifetime long enough that no tracer expires by age and every respawn is an inflow respawn.
			auto tracers = AtmosphereTracers{ solver, gpu, AtmosphereTracerConfig{ .m_particle_count = 512, .m_seed = 12648430u, .m_max_age = 1.0e6f } };
			auto const outside_air = config.m_grid.BuildOutsideAir([](v2 pos)
			{
				// The outside air south of the tunnel axis blows east, and north of it blows west.
				return AtmosphereOutsideAir{ .m_wind = v2{ pos.y < 0.0f ? +5.0f : -5.0f, 0.0f } };
			});
			auto const sources = AtmosphereStepSources{ .m_outside_air = outside_air };
			auto const dt = 1.0f / 60.0f;

			// Let the flow develop before measuring.
			for (int i = 0; i != 600; ++i)
			{
				// Advance the solver and tracers together, as the sandbox does.
				solver.Step(job, dt, sources);
				tracers.Advect(job, dt);
			}
			auto const before = tracers.ReadBack(job);

			// Advance one second, then compare drift in each half.
			for (int i = 0; i != 60; ++i)
			{
				// Same per-step order as the spin-up.
				solver.Step(job, dt, sources);
				tracers.Advect(job, dt);
			}
			auto const after = tracers.ReadBack(job);
			auto south_drift = 0.0f;
			auto north_drift = 0.0f;
			auto south_count = 0;
			auto north_count = 0;
			for (int i = 0; i != isize(after); ++i)
			{
				// Skip respawned tracers and the middle row, where the two flows shear against each other.
				PR_EXPECT(Inside(config, after[i]));
				if (after[i].m_age < before[i].m_age)
					continue;

				auto const drift = after[i].m_position.x - before[i].m_position.x;
				if (before[i].m_position.y < -10.0f)
				{
					south_drift += drift;
					++south_count;
				}
				else if (before[i].m_position.y > 10.0f)
				{
					north_drift += drift;
					++north_count;
				}
			}
			PR_EXPECT(south_count > 50 && north_count > 50);
			auto const south_speed = south_drift / std::max(south_count, 1);
			auto const north_speed = north_drift / std::max(north_count, 1);
			std::printf("Atmosphere counter-flow tracer speed south %f m/s north %f m/s\n", south_speed, north_speed);
			PR_EXPECT(south_speed > 1.0f);
			PR_EXPECT(north_speed < -1.0f);
		}
	};

	PRUnitTestClass(AtmosphereTerrainTests)
	{
		// Return the large terrain-following fixture used by the domain-scale tests.
		static AtmosphereConfig Config(iv3 cells, std::vector<float> floors)
		{
			// These tests use a 1/15 s step with the default V-cycle settings, so they also check that the defaults are good enough for real terrain.
			return AtmosphereConfig{
				.m_grid = AtmosphereGrid{ .m_cell_count = cells, .m_origin = v4{ -0.5f * cells.x * 32.0f, -0.5f * cells.y * 32.0f, 0.0f, 1.0f }, .m_dx = 32.0f, .m_lid_z = 1500.0f, .m_first_layer_thickness = 5.0f, .m_floor_heights = std::move(floors) },
				.m_boundaries = AtmosphereBoundaries{ .m_x_min = EAtmosphereBoundary::Open, .m_x_max = EAtmosphereBoundary::Open, .m_y_min = EAtmosphereBoundary::Open, .m_y_max = EAtmosphereBoundary::Open, .m_z_min = EAtmosphereBoundary::Solid, .m_z_max = EAtmosphereBoundary::Solid },
				.m_reference = AtmosphereReferenceProfile{ .m_temperature_at_origin = 288.0f, .m_lapse_rate = -0.004f, .m_min_temperature = 250.0f },
				.m_gravity = 9.80665f,
				.m_floor_exchange_rate = 0.0f,
				.m_lid_temperature = 282.0f,
				.m_lid_relaxation_rate = 0.0f,
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
			// A 5 m/s outside wind should remain bounded and show terrain-induced acceleration or deflection around the mountain fixture.
			auto const cells = iv3{ 96, 96, 16 };
			auto gpu = Gpu{};
			auto job = GpuJob{ gpu.m_gpu, "AtmosphereTerrainTests.FlowMountain", 0xFF00AAFF, 1 };
			auto solver = AtmosphereSolver{ gpu, Config(cells, Mountain(cells)) };
			auto const outside_air = UniformOutsideAir(solver.Config().m_grid, v2{ 5.0f, 0.0f });
			auto sources = AtmosphereStepSources{ .m_outside_air = outside_air };
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

		PRUnitTestMethod(WindClimbsRamp, Quick)
		{
			// Wind blowing up a 10% ramp cannot pass through the ground, so air near the floor must rise at about the wind speed times the slope.
			auto const cells = iv3{ 64, 32, 16 };
			auto const slope = 0.1f;
			auto floors = std::vector<float>(cells.x * cells.y, 0.0f);
			for (int y = 0; y != cells.y; ++y)
			{
				// The ramp rises from x = 16 to x = 48 and is flat on either side.
				for (int x = 0; x != cells.x; ++x)
					floors[y * cells.x + x] = std::clamp(x - 16.0f, 0.0f, 32.0f) * 32.0f * slope;
			}

			// Solid side walls stop air escaping sideways around the ramp, and a neutral lapse rate removes buoyancy so only the ground's kinematics lift the air.
			auto gpu = Gpu{};
			auto job = GpuJob{ gpu.m_gpu, "AtmosphereTerrainTests.Ramp", 0xFF00AAFF, 1 };
			auto config = Config(cells, floors);
			config.m_boundaries.m_y_min = EAtmosphereBoundary::Solid;
			config.m_boundaries.m_y_max = EAtmosphereBoundary::Solid;
			config.m_reference.m_lapse_rate = -config.m_gravity / 1004.5f;
			auto solver = AtmosphereSolver{ gpu, config };

			// Start from the outside wind everywhere so a short run measures the developed flow rather than the spin-up from rest.
			auto const wind = 10.0f;
			auto const outside_air = UniformOutsideAir(solver.Config().m_grid, v2{ wind, 0.0f });
			auto initial = solver.ReadBack(job);
			for (auto& u : initial.m_u_faces)
				u = wind;

			solver.UploadState(job, initial);
			job.Run();
			auto state = Run(solver, job, 150, 1.0f / 15.0f, AtmosphereStepSources{ .m_outside_air = outside_air });

			// The lowest layers lie between 0 and about 300 m above the ground, where the rise is expected to fade from u * slope towards zero at the lid.
			auto const ramp = MeanVelocity(solver, state, iv3{ 24, 8, 0 }, iv3{ 40, 24, 2 });
			auto const stats = solver.Stats(state);
			std::printf("Atmosphere ramp u %.6f w %.6f floor_w %.6f max_speed %.6f\n", ramp.x, ramp.z, ramp.x * slope, stats.m_max_speed);
			PR_EXPECT(ramp.x > 0.5f * wind);
			PR_EXPECT(ramp.z > 0.3f * ramp.x * slope);
			PR_EXPECT(ramp.z < 1.0f * ramp.x * slope);
			PR_EXPECT(stats.m_max_speed < 2.0f * wind);
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
			auto state = Run(solver, job, 900, 1.0f / 15.0f, AtmosphereStepSources{ .m_floor_temperatures = cold });
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

		PRUnitTestMethod(OpenEdgesOutsideWind, Quick)
		{
			// Open edges should admit a 5 m/s outside wind without overshoot or material divergence after one simulated minute.
			auto const cells = iv3{ 96, 96, 16 };
			auto gpu = Gpu{};
			auto job = GpuJob{ gpu.m_gpu, "AtmosphereTerrainTests.OpenEdges", 0xFF00AAFF, 1 };
			auto config = Config(cells, std::vector<float>(cells.x * cells.y, 0.0f));

			// A sponge reaching the domain centre drives the whole interior toward the outside wind within the test time.
			config.m_open_edge_band = cells.x / 2;
			auto solver = AtmosphereSolver{ gpu, config };
			auto const outside_air = UniformOutsideAir(solver.Config().m_grid, v2{ 5.0f, 0.0f });
			auto state = Run(solver, job, 900, 1.0f / 15.0f, AtmosphereStepSources{ .m_outside_air = outside_air });
			auto stats = solver.Stats(state);
			auto interior = MeanVelocity(solver, state, iv3{ 32, 32, 1 }, iv3{ 64, 64, 8 });
			std::printf("Atmosphere open edges interior %.6f m/s target 5.000000 max_speed %.6f threshold 7.500000 rms_div %.9f threshold 0.005000\n", interior.x, stats.m_max_speed, stats.m_rms_divergence);
			PR_EXPECT(std::abs(interior.x - 5.0f) < 0.5f);
			PR_EXPECT(stats.m_max_speed < 7.5f);
			PR_EXPECT(stats.m_rms_divergence < 0.005f);
		}

		PRUnitTestMethod(FlowAroundSolidCylinder, Quick)
		{
			// Columns whose floor reaches the lid are solid. Wind should pass around them without crossing any of their faces, speeding up
			// beside the obstacle and slowing in its wake, while the projection stays divergence-free.
			auto const cells = iv3{ 64, 32, 8 };
			auto const centre = v2{ 20.5f, 16.0f };
			auto const radius = 4.0f;
			auto config = Config(cells, std::vector<float>(cells.x * cells.y, 0.0f));
			auto solid = std::vector<uint8_t>(cells.x * cells.y, 0);
			for (int y = 0; y != cells.y; ++y)
			{
				// Mark columns whose centre lies inside the cylinder and raise their floor to the lid.
				for (int x = 0; x != cells.x; ++x)
				{
					// Distances are measured in cells from the cylinder axis.
					auto const d = v2{ x + 0.5f, y + 0.5f } - centre;
					if (LengthSq(d) > radius * radius)
						continue;

					solid[y * cells.x + x] = 1;
					config.m_grid.m_floor_heights[y * cells.x + x] = config.m_grid.m_lid_z;
				}
			}
			config.m_boundaries.m_y_min = EAtmosphereBoundary::Solid;
			config.m_boundaries.m_y_max = EAtmosphereBoundary::Solid;
			auto gpu = Gpu{};
			auto job = GpuJob{ gpu.m_gpu, "AtmosphereTerrainTests.SolidCylinder", 0xFF00AAFF, 1 };
			auto solver = AtmosphereSolver{ gpu, config };
			auto const& grid = solver.Config().m_grid;
			auto const outside_air = UniformOutsideAir(grid, v2{ 5.0f, 0.0f });
			auto state = Run(solver, job, 450, 1.0f / 15.0f, AtmosphereStepSources{ .m_outside_air = outside_air });
			auto stats = solver.Stats(state);

			// Every face of a solid column must carry no flow.
			auto max_wall_flux = 0.0f;
			for (int y = 0; y != cells.y; ++y)
			{
				// Only solid columns are checked; their six face families are all walls.
				for (int x = 0; x != cells.x; ++x)
				{
					// Air columns are not walls.
					if (solid[y * cells.x + x] == 0)
						continue;

					for (int z = 0; z != cells.z; ++z)
					{
						// Check the x, y and z faces bounding this cell.
						max_wall_flux = std::max(max_wall_flux, std::abs(state.m_u_faces[grid.UFaceIndex(iv3{ x, y, z })]));
						max_wall_flux = std::max(max_wall_flux, std::abs(state.m_u_faces[grid.UFaceIndex(iv3{ x + 1, y, z })]));
						max_wall_flux = std::max(max_wall_flux, std::abs(state.m_v_faces[grid.VFaceIndex(iv3{ x, y, z })]));
						max_wall_flux = std::max(max_wall_flux, std::abs(state.m_v_faces[grid.VFaceIndex(iv3{ x, y + 1, z })]));
						max_wall_flux = std::max(max_wall_flux, std::abs(state.m_w_faces[grid.WFaceIndex(iv3{ x, y, z })]));
						max_wall_flux = std::max(max_wall_flux, std::abs(state.m_w_faces[grid.WFaceIndex(iv3{ x, y, z + 1 })]));
					}
				}
			}

			// Compare the air beside the cylinder with the air in its wake.
			auto const flank = 0.5f * (MeanVelocity(solver, state, iv3{ 19, 21, 0 }, iv3{ 23, 24, 8 }).x + MeanVelocity(solver, state, iv3{ 19, 8, 0 }, iv3{ 23, 11, 8 }).x);
			auto const wake = MeanVelocity(solver, state, iv3{ 25, 14, 0 }, iv3{ 28, 18, 8 }).x;
			std::printf("Atmosphere solid cylinder wall_flux %.9f flank %.6f wake %.6f max_speed %.6f threshold 10.000000 rms_div %.9f threshold 0.005000\n", max_wall_flux, flank, wake, stats.m_max_speed, stats.m_rms_divergence);
			PR_EXPECT(max_wall_flux == 0.0f);
			PR_EXPECT(flank > 5.0f);
			PR_EXPECT(wake < flank - 1.0f);
			PR_EXPECT(stats.m_max_speed < 10.0f);
			PR_EXPECT(stats.m_rms_divergence < 0.005f);
		}

		PRUnitTestMethod(FloorDragSlowsNearFloorFlow, Quick)
		{
			// Floor drag should slow the wind in the lowest layer, leave the upper layers close to the outside wind, and have no effect when zero.
			// Thin, even layers make the drag on the 12.5 m bottom layer strong enough to measure over a short run.
			auto const cells = iv3{ 48, 16, 8 };
			auto config = Config(cells, std::vector<float>(cells.x * cells.y, 0.0f));
			config.m_boundaries.m_y_min = EAtmosphereBoundary::Solid;
			config.m_boundaries.m_y_max = EAtmosphereBoundary::Solid;
			config.m_grid.m_lid_z = 100.0f;
			config.m_grid.m_first_layer_thickness = 12.5f;
			config.m_open_edge_band = 4;
			auto dragged_config = config;
			dragged_config.m_wall_drag.m_z_min = 0.05f;
			auto gpu = Gpu{};
			auto job = GpuJob{ gpu.m_gpu, "AtmosphereTerrainTests.FloorDrag", 0xFF00AAFF, 1 };
			auto free_solver = AtmosphereSolver{ gpu, config };
			auto dragged_solver = AtmosphereSolver{ gpu, dragged_config };
			auto const outside_air = UniformOutsideAir(free_solver.Config().m_grid, v2{ 5.0f, 0.0f });
			auto const free_state = Run(free_solver, job, 300, 1.0f / 15.0f, AtmosphereStepSources{ .m_outside_air = outside_air });
			auto const dragged_state = Run(dragged_solver, job, 300, 1.0f / 15.0f, AtmosphereStepSources{ .m_outside_air = outside_air });

			// Compare the bottom and top layers in the interior, away from the sponge bands.
			auto const lo = iv3{ 16, 4, 0 };
			auto const hi = iv3{ 32, 12, 1 };
			auto const free_floor = MeanVelocity(free_solver, free_state, lo, hi).x;
			auto const dragged_floor = MeanVelocity(dragged_solver, dragged_state, lo, hi).x;
			auto const dragged_top = MeanVelocity(dragged_solver, dragged_state, iv3{ lo.x, lo.y, cells.z - 1 }, iv3{ hi.x, hi.y, cells.z }).x;
			auto const stats = dragged_solver.Stats(dragged_state);
			std::printf("Atmosphere floor drag free_floor %.6f dragged_floor %.6f dragged_top %.6f max_speed %.6f rms_div %.9f\n", free_floor, dragged_floor, dragged_top, stats.m_max_speed, stats.m_rms_divergence);
			PR_EXPECT(std::abs(free_floor - 5.0f) < 0.25f);
			PR_EXPECT(dragged_floor > 0.0f);
			PR_EXPECT(dragged_floor < free_floor - 1.0f);
			PR_EXPECT(dragged_top > dragged_floor + 1.0f);
			PR_EXPECT(stats.m_max_speed < 7.5f);
			PR_EXPECT(stats.m_rms_divergence < 0.005f);
		}

		PRUnitTestMethod(VerticalViscositySpreadsFloorDrag, Quick)
		{
			// Vertical viscosity should carry the floor drag up into the layers above, and in return speed up the bottom layer.
			// Uses the same thin, even layers as FloorDragSlowsNearFloorFlow so the effect is measurable over a short run.
			auto const cells = iv3{ 48, 16, 8 };
			auto config = Config(cells, std::vector<float>(cells.x * cells.y, 0.0f));
			config.m_boundaries.m_y_min = EAtmosphereBoundary::Solid;
			config.m_boundaries.m_y_max = EAtmosphereBoundary::Solid;
			config.m_grid.m_lid_z = 100.0f;
			config.m_grid.m_first_layer_thickness = 12.5f;
			config.m_open_edge_band = 4;
			config.m_wall_drag.m_z_min = 0.05f;
			auto mixed_config = config;
			mixed_config.m_vertical_viscosity = 50.0f;
			auto gpu = Gpu{};
			auto job = GpuJob{ gpu.m_gpu, "AtmosphereTerrainTests.VerticalViscosity", 0xFF00AAFF, 1 };
			auto unmixed_solver = AtmosphereSolver{ gpu, config };
			auto mixed_solver = AtmosphereSolver{ gpu, mixed_config };
			auto const outside_air = UniformOutsideAir(unmixed_solver.Config().m_grid, v2{ 5.0f, 0.0f });
			auto const unmixed_state = Run(unmixed_solver, job, 300, 1.0f / 15.0f, AtmosphereStepSources{ .m_outside_air = outside_air });
			auto const mixed_state = Run(mixed_solver, job, 300, 1.0f / 15.0f, AtmosphereStepSources{ .m_outside_air = outside_air });

			// Compare the bottom layer and the layer just above it in the interior, away from the sponge bands.
			auto const lo = iv3{ 16, 4, 0 };
			auto const hi = iv3{ 32, 12, 1 };
			auto const above_lo = iv3{ lo.x, lo.y, 1 };
			auto const above_hi = iv3{ hi.x, hi.y, 2 };
			auto const unmixed_floor = MeanVelocity(unmixed_solver, unmixed_state, lo, hi).x;
			auto const mixed_floor = MeanVelocity(mixed_solver, mixed_state, lo, hi).x;
			auto const unmixed_above = MeanVelocity(unmixed_solver, unmixed_state, above_lo, above_hi).x;
			auto const mixed_above = MeanVelocity(mixed_solver, mixed_state, above_lo, above_hi).x;
			auto const stats = mixed_solver.Stats(mixed_state);
			std::printf("Atmosphere vertical viscosity unmixed_floor %.6f mixed_floor %.6f unmixed_above %.6f mixed_above %.6f max_speed %.6f rms_div %.9f\n", unmixed_floor, mixed_floor, unmixed_above, mixed_above, stats.m_max_speed, stats.m_rms_divergence);
			PR_EXPECT(mixed_floor > unmixed_floor + 0.2f);
			PR_EXPECT(mixed_above < unmixed_above - 0.2f);
			PR_EXPECT(stats.m_max_speed < 7.5f);
			PR_EXPECT(stats.m_rms_divergence < 0.005f);
		}

		PRUnitTestMethod(OutsideAirTemperatureInflow, Quick)
		{
			// Warm outside air should warm the inflow edge and cold outside air should cool it, relative to the reference profile.
			auto const cells = iv3{ 64, 64, 16 };
			auto gpu = Gpu{};
			auto job = GpuJob{ gpu.m_gpu, "AtmosphereTerrainTests.OutsideTemperature", 0xFF00AAFF, 1 };
			auto warm_solver = AtmosphereSolver{ gpu, Config(cells, std::vector<float>(cells.x * cells.y, 0.0f)) };
			auto const& grid = warm_solver.Config().m_grid;
			auto const warm_air = UniformOutsideAir(grid, v2{ 5.0f, 0.0f }, +4.0f);
			auto const warm_state = Run(warm_solver, job, 20, 1.0f / 15.0f, AtmosphereStepSources{ .m_outside_air = warm_air });
			auto cold_solver = AtmosphereSolver{ gpu, Config(cells, std::vector<float>(cells.x * cells.y, 0.0f)) };
			auto const cold_air = UniformOutsideAir(grid, v2{ 5.0f, 0.0f }, -4.0f);
			auto const cold_state = Run(cold_solver, job, 20, 1.0f / 15.0f, AtmosphereStepSources{ .m_outside_air = cold_air });

			// Compare the west inflow edge cell with the reference temperature at its height.
			auto const edge = iv3{ 0, 32, 0 };
			auto const ref_temp = warm_solver.Config().m_reference.Temperature(grid.CellCentre(edge).z);
			auto const warm_dt = warm_state.m_temperature[grid.CellIndex(edge)] - ref_temp;
			auto const cold_dt = cold_state.m_temperature[grid.CellIndex(edge)] - ref_temp;
			std::printf("Atmosphere outside air inflow warm_dt %.6f cold_dt %.6f\n", warm_dt, cold_dt);
			PR_EXPECT(warm_dt > 1.0f);
			PR_EXPECT(cold_dt < -1.0f);
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
