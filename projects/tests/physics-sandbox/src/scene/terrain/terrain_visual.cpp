//************************************
// Physics Sandbox
//  Copyright (c) Rylogic Ltd 2026
//************************************
#include "src/scene/terrain/terrain_visual.h"
#include "src/scene/scene.h"

namespace physics_sandbox
{
	namespace
	{
		using Clock = std::chrono::steady_clock;

		struct Row
		{
			double m_y = 0.0;
			std::vector<double> m_x = {};
			std::vector<uint32_t> m_vertex_index = {};
		};

		// Return the elapsed time in milliseconds between two steady-clock points.
		double ElapsedMs(Clock::time_point beg, Clock::time_point end)
		{
			return std::chrono::duration<double, std::milli>(end - beg).count();
		}

		// Clamp a scalar into the unit interval.
		double Clamp01(double value)
		{
			return std::clamp(value, 0.0, 1.0);
		}

		// Blend two display colours in straight RGBA space.
		Colour32 LerpColour(Colour32 lhs, Colour32 rhs, double t)
		{
			auto channel = [=](int a, int b)
			{
				return static_cast<int>(std::lround(a + (b - a) * Clamp01(t)));
			};
			return Colour32(channel(lhs.r, rhs.r), channel(lhs.g, rhs.g), channel(lhs.b, rhs.b), channel(lhs.a, rhs.a));
		}

		// Convert one terrain sample into the requested diagnostic display colour.
		Colour32 VertexColour(pr::physics::terrain::SurfaceSample const& sample, scene_loader::ETerrainDisplayMode display)
		{
			switch (display)
			{
				case scene_loader::ETerrainDisplayMode::Neutral:
				{
					return Colour32(0xFFC8C8C8U);
				}
				case scene_loader::ETerrainDisplayMode::Elevation:
				{
					if (sample.m_height <= 0.0)
						return LerpColour(Colour32(0xFF274C9BU), Colour32(0xFF6AA5D7U), Clamp01((sample.m_height + 120.0) / 120.0));
					if (sample.m_height <= 120.0)
						return LerpColour(Colour32(0xFFC2B280U), Colour32(0xFF6E9B4BU), Clamp01(sample.m_height / 120.0));
					if (sample.m_height <= 260.0)
						return LerpColour(Colour32(0xFF6E9B4BU), Colour32(0xFF8D7E67U), Clamp01((sample.m_height - 120.0) / 140.0));
					return LerpColour(Colour32(0xFF8D7E67U), Colour32(0xFFD9D9D9U), Clamp01((sample.m_height - 260.0) / 220.0));
				}
				case scene_loader::ETerrainDisplayMode::Slope:
				{
					auto const slope = Length(sample.m_gradient_xy);
					return LerpColour(Colour32(0xFFDBE6D1U), Colour32(0xFF6F5846U), Clamp01(slope / 1.6));
				}
				default:
				{
					throw std::runtime_error("Unknown terrain display mode");
				}
			}
		}

		// Require the sandbox mesh settings to be finite, positive, and bounded before allocation.
		void Validate(scene_loader::TerrainDesc const& terrain)
		{
			if (!std::isfinite(terrain.radius_m) || terrain.radius_m <= 0.0)
				throw std::invalid_argument("Terrain radius must be finite and greater than zero");
			if (terrain.intervals < 4 || terrain.intervals > 4096)
				throw std::invalid_argument("Terrain intervals must be in the range [4, 4096]");
			if (!std::isfinite(terrain.centre_xy.x) || !std::isfinite(terrain.centre_xy.y))
				throw std::invalid_argument("Terrain centre coordinates must be finite");
		}

		// Measure point and four-point batch workloads without retaining any caller-owned sampling buffers.
		void MeasureSampling(pr::physics::terrain::landscape::BaselineSurface const& surface, scene_loader::TerrainDesc const& terrain, double spacing_m, TerrainVisual::Metrics& metrics)
		{
			constexpr auto PointSampleCount = 8192;
			constexpr auto BatchCount = 2048;
			constexpr auto BatchWidth = 4;
			auto rng = std::mt19937(terrain.surface.m_seed ^ 0x71EAA123u);
			auto radius = std::uniform_real_distribution<double>(0.0, std::max(terrain.radius_m - spacing_m, 0.0));
			auto angle = std::uniform_real_distribution<double>(0.0, math::constants<double>::tau);
			auto points = std::vector<pr::physics::terrain::v2d>{};
			points.reserve(PointSampleCount);
			for (auto index = 0; index != PointSampleCount; ++index)
			{
				auto const r = radius(rng);
				auto const a = angle(rng);
				points.push_back(terrain.centre_xy + pr::physics::terrain::v2d{r * std::cos(a), r * std::sin(a)});
			}

			auto point_samples = std::vector<pr::physics::terrain::SurfaceSample>(points.size());
			auto point_beg = Clock::now();
			surface.Sample(points, point_samples);
			auto const point_end = Clock::now();

			// Nearby clusters exercise the batch boundary with wheel-like four-point workloads that can cross noise-cell boundaries.
			auto batch_positions = std::vector<pr::physics::terrain::v2d>{};
			batch_positions.reserve(BatchCount * BatchWidth);
			auto const cluster_offset = 0.51 * spacing_m;
			for (auto index = 0; index != BatchCount; ++index)
			{
				auto const r = radius(rng);
				auto const a = angle(rng);
				auto const centre = terrain.centre_xy + pr::physics::terrain::v2d{r * std::cos(a), r * std::sin(a)};
				batch_positions.push_back(centre + pr::physics::terrain::v2d{-cluster_offset, -cluster_offset});
				batch_positions.push_back(centre + pr::physics::terrain::v2d{+cluster_offset, -cluster_offset});
				batch_positions.push_back(centre + pr::physics::terrain::v2d{-cluster_offset, +cluster_offset});
				batch_positions.push_back(centre + pr::physics::terrain::v2d{+cluster_offset, +cluster_offset});
			}

			auto batch_samples = std::vector<pr::physics::terrain::SurfaceSample>(batch_positions.size());
			auto batch_beg = Clock::now();
			for (auto batch_index = 0; batch_index != BatchCount; ++batch_index)
			{
				auto const offset = static_cast<size_t>(batch_index * BatchWidth);
				surface.Sample(std::span(batch_positions).subspan(offset, BatchWidth), std::span(batch_samples).subspan(offset, BatchWidth));
			}
			auto const batch_end = Clock::now();

			auto scalar_beg = Clock::now();
			for (auto const& position : batch_positions)
				(void)surface.Sample(position);
			auto const scalar_end = Clock::now();

			metrics.m_point_sample_count = PointSampleCount;
			metrics.m_batch_count = BatchCount;
			metrics.m_batch_width = BatchWidth;
			metrics.m_point_sampling_ms = ElapsedMs(point_beg, point_end);
			metrics.m_batch_sampling_ms = ElapsedMs(batch_beg, batch_end);
			metrics.m_batch_scalar_ms = ElapsedMs(scalar_beg, scalar_end);
			metrics.m_peak_scratch_bytes = std::max(metrics.m_peak_scratch_bytes, points.capacity() * sizeof(pr::physics::terrain::v2d) + point_samples.capacity() * sizeof(pr::physics::terrain::SurfaceSample) + batch_positions.capacity() * sizeof(pr::physics::terrain::v2d) + batch_samples.capacity() * sizeof(pr::physics::terrain::SurfaceSample));
		}

		// Build one row of the clipped grid by keeping interior grid vertices and exact circle intersections on the row ends.
		Row BuildRow(scene_loader::TerrainDesc const& terrain, int row_index, double spacing_m)
		{
			auto row = Row{};
			row.m_y = -terrain.radius_m + row_index * spacing_m;
			auto const extent_sq = std::max(0.0, terrain.radius_m * terrain.radius_m - row.m_y * row.m_y);
			auto const extent = std::sqrt(extent_sq);
			auto const left = -extent;
			auto const right = +extent;
			auto constexpr Eps = 1.0e-9;

			row.m_x.reserve(static_cast<size_t>(terrain.intervals) + 2);
			row.m_x.push_back(left);
			for (auto column = 0; column != terrain.intervals + 1; ++column)
			{
				auto const x = -terrain.radius_m + column * spacing_m;
				if (x > left + Eps && x < right - Eps)
					row.m_x.push_back(x);
			}
			if (right > left + Eps)
				row.m_x.push_back(right);

			return row;
		}

		// Add one prepared terrain vertex while preserving the authoritative terrain evaluation in double precision.
		uint32_t AddVertex(TerrainVisual::PreparedMesh& mesh, pr::physics::terrain::landscape::BaselineSurface const& surface, scene_loader::TerrainDesc const& terrain, double x_local, double y_local)
		{
			auto const world_xy = terrain.centre_xy + pr::physics::terrain::v2d{x_local, y_local};
			auto const sample = surface.Sample(world_xy);
			auto const normal = sample.Normal();
			auto const world_pos = pr::physics::terrain::v4d{world_xy.x, world_xy.y, sample.m_height, 1.0};
			auto const local_pos = pr::physics::terrain::v4d{x_local, y_local, sample.m_height, 1.0};
			auto vert = rdr12::Vert{};
			vert.m_vert = v4(static_cast<float>(local_pos.x), static_cast<float>(local_pos.y), static_cast<float>(local_pos.z), 1.0f);
			vert.m_diff = VertexColour(sample, terrain.display);
			vert.m_norm = v4(static_cast<float>(normal.x), static_cast<float>(normal.y), static_cast<float>(normal.z), 0.0f);
			vert.m_tex0 = v2(static_cast<float>((x_local / (2.0 * terrain.radius_m)) + 0.5), static_cast<float>((y_local / (2.0 * terrain.radius_m)) + 0.5));
			vert.m_idx0 = iv2::Zero();
			mesh.m_vertices.push_back(vert);
			Grow(mesh.m_local_bounds, vert.m_vert);
			Grow(mesh.m_world_bounds, world_pos);
			return static_cast<uint32_t>(mesh.m_vertices.size() - 1);
		}

		// Connect two neighbouring monotonic rows with consistently wound triangles.
		void StitchRows(Row const& lower, Row const& upper, std::vector<uint32_t>& indices)
		{
			auto i = size_t{};
			auto j = size_t{};
			while (i + 1 < lower.m_vertex_index.size() || j + 1 < upper.m_vertex_index.size())
			{
				if (i + 1 == lower.m_vertex_index.size())
				{
					indices.push_back(lower.m_vertex_index[i]);
					indices.push_back(upper.m_vertex_index[j + 1]);
					indices.push_back(upper.m_vertex_index[j]);
					++j;
					continue;
				}

				if (j + 1 == upper.m_vertex_index.size())
				{
					indices.push_back(lower.m_vertex_index[i]);
					indices.push_back(lower.m_vertex_index[i + 1]);
					indices.push_back(upper.m_vertex_index[j]);
					++i;
					continue;
				}

				if (lower.m_x[i + 1] <= upper.m_x[j + 1])
				{
					indices.push_back(lower.m_vertex_index[i]);
					indices.push_back(lower.m_vertex_index[i + 1]);
					indices.push_back(upper.m_vertex_index[j]);
					++i;
				}
				else
				{
					indices.push_back(lower.m_vertex_index[i]);
					indices.push_back(upper.m_vertex_index[j + 1]);
					indices.push_back(upper.m_vertex_index[j]);
					++j;
				}
			}
		}
	}

	// Prepare the CPU mesh in a renderer-independent form.
	TerrainVisual::PreparedMesh TerrainVisual::PrepareMesh(pr::physics::terrain::landscape::BaselineSurface const& surface, scene_loader::TerrainDesc const& terrain)
	{
		Validate(terrain);

		auto mesh = PreparedMesh{};
		mesh.m_desc = terrain;
		mesh.m_origin_ws = pr::physics::terrain::v4d{terrain.centre_xy.x, terrain.centre_xy.y, 0.0, 1.0};
		auto const spacing_m = (2.0 * terrain.radius_m) / terrain.intervals;
		MeasureSampling(surface, terrain, spacing_m, mesh.m_metrics);

		// Build the clipped row layout first so allocation sizes are known before vertex generation begins.
		auto const mesh_beg = Clock::now();
		auto rows = std::vector<Row>{};
		rows.reserve(static_cast<size_t>(terrain.intervals) + 1);
		for (auto row_index = 0; row_index != terrain.intervals + 1; ++row_index)
			rows.push_back(BuildRow(terrain, row_index, spacing_m));

		auto vertex_count = size_t{};
		auto triangle_count = size_t{};
		for (auto row_index = size_t{}; row_index != rows.size(); ++row_index)
		{
			vertex_count += rows[row_index].m_x.size();
			if (row_index + 1 != rows.size())
				triangle_count += rows[row_index].m_x.size() + rows[row_index + 1].m_x.size() - 2;
		}
		if (vertex_count == 0 || triangle_count == 0)
			throw std::runtime_error("Terrain mesh generation produced no geometry");
		if (vertex_count > std::numeric_limits<uint32_t>::max())
			throw std::runtime_error("Terrain mesh requires more than 32-bit vertex indices");
		if (vertex_count > std::numeric_limits<size_t>::max() / sizeof(rdr12::Vert))
			throw std::overflow_error("Terrain mesh vertex allocation would overflow");
		if (triangle_count > std::numeric_limits<size_t>::max() / 3 || triangle_count * 3 > std::numeric_limits<size_t>::max() / sizeof(uint32_t))
			throw std::overflow_error("Terrain mesh index allocation would overflow");

		// Sample the authoritative terrain function once per retained vertex after the row topology is fixed.
		mesh.m_vertices.reserve(vertex_count);
		mesh.m_indices.reserve(triangle_count * 3);
		for (auto& row : rows)
		{
			row.m_vertex_index.reserve(row.m_x.size());
			for (auto const x_local : row.m_x)
				row.m_vertex_index.push_back(AddVertex(mesh, surface, terrain, x_local, row.m_y));
		}
		for (auto row_index = size_t{}; row_index + 1 != rows.size(); ++row_index)
			StitchRows(rows[row_index], rows[row_index + 1], mesh.m_indices);

		mesh.m_metrics.m_mesh_generation_ms = ElapsedMs(mesh_beg, Clock::now());
		mesh.m_metrics.m_vertex_count = mesh.m_vertices.size();
		mesh.m_metrics.m_triangle_count = mesh.m_indices.size() / 3;
		mesh.m_metrics.m_retained_cpu_bytes = mesh.m_vertices.capacity() * sizeof(rdr12::Vert) + mesh.m_indices.capacity() * sizeof(uint32_t);
		mesh.m_metrics.m_peak_scratch_bytes = std::max(mesh.m_metrics.m_peak_scratch_bytes, rows.capacity() * sizeof(Row));
		for (auto const& row : rows)
			mesh.m_metrics.m_peak_scratch_bytes += row.m_x.capacity() * sizeof(double) + row.m_vertex_index.capacity() * sizeof(uint32_t);
		return mesh;
	}

	// Create the renderer resources for a previously prepared CPU mesh.
	TerrainVisual::TerrainVisual(rdr12::Renderer& rdr, PreparedMesh prepared_mesh)
		: m_inst()
		, m_mesh(std::move(prepared_mesh))
	{
		auto buffers = rdr12::ModelGenerator::Buffers<rdr12::Vert>{};
		buffers.Reset(static_cast<int>(m_mesh.m_vertices.size()), 0, 0, sizeof(uint32_t));
		for (auto index = size_t{}; index != m_mesh.m_vertices.size(); ++index)
			buffers.m_vcont[index] = m_mesh.m_vertices[index];
		for (auto const index : m_mesh.m_indices)
			buffers.m_icont.push_back(index);
		buffers.m_bbox = m_mesh.m_local_bounds;
		buffers.m_name = "Terrain";
		buffers.m_ncont.push_back(rdr12::NuggetDesc(rdr12::ETopo::TriList, rdr12::EGeom::Vert | rdr12::EGeom::Colr | rdr12::EGeom::Norm | rdr12::EGeom::Tex0));

		auto factory = rdr12::ResourceFactory(rdr);
		auto cache = rdr12::ModelGenerator::Cache{buffers};
		m_inst.m_model = rdr12::ModelGenerator::Create<rdr12::Vert>(factory, cache);
		m_inst.m_i2w = m4x4::Translation(static_cast<float>(m_mesh.m_origin_ws.x), static_cast<float>(m_mesh.m_origin_ws.y), static_cast<float>(m_mesh.m_origin_ws.z));
	}

	// Add the visual to the scene when renderer resources are available.
	void TerrainVisual::AddToScene(rdr12::Scene& scene)
	{
		scene.AddInstance(m_inst);
	}

	#if PR_UNITTESTS
	namespace tests
	{
		namespace
		{
			// Return a small deterministic terrain scene for mesh-shape tests.
			scene_loader::TerrainDesc TestTerrain(scene_loader::ETerrainDisplayMode display = scene_loader::ETerrainDisplayMode::Neutral, int intervals = 32)
			{
				auto terrain = scene_loader::TerrainDesc{};
				terrain.surface.m_seed = 99u;
				terrain.radius_m = 128.0;
				terrain.intervals = intervals;
				terrain.display = display;
				return terrain;
			}
		}

		PRUnitTestClass(TerrainVisualTests)
		{
			PRUnitTestMethod(PreparesCleanDiskTopologyAndBounds, Quick)
			{
				auto const surface = pr::physics::terrain::landscape::BaselineSurface();
				auto const mesh = TerrainVisual::PrepareMesh(surface, TestTerrain());
				PR_EXPECT(!mesh.m_vertices.empty());
				PR_EXPECT(mesh.m_indices.size() % 3 == 0);
				PR_EXPECT(mesh.m_metrics.m_vertex_count == mesh.m_vertices.size());
				PR_EXPECT(mesh.m_metrics.m_triangle_count == mesh.m_indices.size() / 3);

				auto boundary_found = false;
				for (auto const& vert : mesh.m_vertices)
				{
					auto const radius = std::sqrt(static_cast<double>(vert.m_vert.x) * vert.m_vert.x + static_cast<double>(vert.m_vert.y) * vert.m_vert.y);
					PR_EXPECT(radius <= mesh.m_desc.radius_m + 1.0e-4);
					if (std::abs(radius - mesh.m_desc.radius_m) < 1.0e-6)
						boundary_found = true;
				}
				PR_EXPECT(boundary_found);

				for (auto index = size_t{}; index != mesh.m_indices.size(); index += 3)
				{
					auto const a = mesh.m_vertices[mesh.m_indices[index + 0]].m_vert;
					auto const b = mesh.m_vertices[mesh.m_indices[index + 1]].m_vert;
					auto const c = mesh.m_vertices[mesh.m_indices[index + 2]].m_vert;
					auto const winding = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
					PR_EXPECT(winding > 0.0f);
				}
			}

			PRUnitTestMethod(SceneLoadingBuildsTerrainWithoutRenderer, Quick)
			{
				auto scene_desc = scene_loader::SceneDesc{};
				scene_desc.terrain = TestTerrain(scene_loader::ETerrainDisplayMode::Elevation, 24);
				auto scene = Scene(nullptr);
				scene.LoadScene(scene_desc);
				PR_EXPECT(scene.m_terrain_surface != nullptr);
				PR_EXPECT(scene.m_terrain_mesh.has_value());
				PR_EXPECT(!scene.m_terrain_gfx);
				PR_EXPECT(scene.m_terrain_mesh->m_metrics.m_vertex_count != 0);
				PR_EXPECT(scene.m_terrain_surface->Sample(scene_desc.terrain->centre_xy + pr::physics::terrain::v2d{12.5, -9.5}).Normal().z > 0.0);
			}
		};
	}
	#endif
}
