//************************************
// Physics Sandbox
//  Copyright (c) Rylogic Ltd 2026
//************************************
#include "src/scene/atmosphere_visual.h"

namespace physics_sandbox
{
	namespace
	{
		// Return an ARGB colour on a blue-white-red temperature ramp.
		uint32_t TemperatureColour(float temperature, float min_temperature, float max_temperature, int alpha = 255)
		{
			// Clamp to the configured display range so outliers remain visible without changing the legend every frame.
			auto const t = std::clamp((temperature - min_temperature) / std::max(max_temperature - min_temperature, 0.001f), 0.0f, 1.0f);
			auto const r = static_cast<int>(255.0f * t);
			auto const b = static_cast<int>(255.0f * (1.0f - t));
			auto const g = static_cast<int>(96.0f * (1.0f - std::abs(2.0f * t - 1.0f)));
			return (static_cast<uint32_t>(alpha) << 24) | (static_cast<uint32_t>(r) << 16) | (static_cast<uint32_t>(g) << 8) | static_cast<uint32_t>(b);
		}

		// Append one box outline to a line builder.
		void AddBoxLines(ldraw::LdrLine& lines, v4 const& min_corner, v4 const& max_corner, uint32_t colour)
		{
			// The domain edge list is small and explicit, which keeps the diagnostic shape readable.
			auto const p000 = v4{min_corner.x, min_corner.y, min_corner.z, 1.0f};
			auto const p100 = v4{max_corner.x, min_corner.y, min_corner.z, 1.0f};
			auto const p010 = v4{min_corner.x, max_corner.y, min_corner.z, 1.0f};
			auto const p110 = v4{max_corner.x, max_corner.y, min_corner.z, 1.0f};
			auto const p001 = v4{min_corner.x, min_corner.y, max_corner.z, 1.0f};
			auto const p101 = v4{max_corner.x, min_corner.y, max_corner.z, 1.0f};
			auto const p011 = v4{min_corner.x, max_corner.y, max_corner.z, 1.0f};
			auto const p111 = v4{max_corner.x, max_corner.y, max_corner.z, 1.0f};
			lines.line(p000, p100, colour).line(p100, p110, colour).line(p110, p010, colour).line(p010, p000, colour);
			lines.line(p001, p101, colour).line(p101, p111, colour).line(p111, p011, colour).line(p011, p001, colour);
			lines.line(p000, p001, colour).line(p100, p101, colour).line(p110, p111, colour).line(p010, p011, colour);
		}
	}

	// Create the solver, tracer set, and initial diagnostic geometry.
	AtmosphereVisual::AtmosphereVisual(ID3D12Device4* device, rdr12::Renderer& rdr, ::pr::compute::shader_cache::IShaderCache& shader_cache, scene_loader::AtmosphereDesc desc)
		: m_gpu(device)
		, m_solver(m_gpu, desc.m_config, &shader_cache)
		, m_tracers()
		, m_desc(std::move(desc))
		, m_particles()
		, m_rdr(rdr)
		, m_gfx()
		, m_gfx_stale(true)
		, m_show_grid(m_desc.m_visual.m_show_grid)
		, m_show_particles(m_desc.m_visual.m_show_particles)
		, m_show_pressure_nodes(m_desc.m_visual.m_show_pressure_nodes)
	{
		// Tracers are optional so diagnostic-only atmosphere scenes can omit particle cost.
		if (m_desc.m_tracers.m_particle_count > 0)
		{
			m_tracers = std::make_unique<physics::atmosphere::AtmosphereTracers>(m_solver, m_gpu, m_desc.m_tracers, &shader_cache);
			m_particles = m_tracers->ReadBack(m_gpu.m_job);
		}
	}

	// Advance the atmosphere solver and refresh CPU-visible tracer state.
	void AtmosphereVisual::Step(float dt)
	{
		// Evolve the pressure nodes with the solver's own rules so drift, decay, and respawn match what a game would see.
		// Evolve treats the domain as centred on the world origin, so use the half-width of the grid as its radius.
		auto const& grid = m_desc.m_config.m_grid;
		auto const domain_radius = 0.5f * grid.m_dx * static_cast<float>(std::max(grid.m_cell_count.x, grid.m_cell_count.y));
		m_desc.m_forcing.Evolve(dt, domain_radius, m_desc.m_forcing_pressure_scale, m_desc.m_forcing_temperature_scale);
		auto const sources = physics::atmosphere::AtmosphereStepSources{
			.m_heat_sources = m_desc.m_heat_sources,
			.m_pressure_nodes = m_desc.m_forcing.m_nodes,
			.m_reservoir_wind = m_desc.m_reservoir_wind,
			.m_reservoir_temperature_offset = m_desc.m_reservoir_temperature_offset,
		};

		// Run the solver and tracers in submission order, then read back the particles for the CPU LDraw overlay.
		m_solver.Step(m_gpu.m_job, dt, sources);
		if (m_tracers != nullptr)
		{
			// The tracer readback also submits the solver work recorded above.
			m_tracers->Advect(m_gpu.m_job, dt);
			m_particles = m_tracers->ReadBack(m_gpu.m_job);
		}
		else
		{
			// Without tracers the recorded solver work still needs submitting.
			m_gpu.m_job.Run();
		}
		m_gfx_stale = true;
	}

	// Add current atmosphere diagnostics to the render scene.
	void AtmosphereVisual::AddToScene(rdr12::Scene& scene)
	{
		// Rebuild here, after the scene has cleared its drawlists, because releasing a model that is still in a drawlist is an error.
		// A single parsed LDraw object owns all atmosphere diagnostics for straightforward toggling.
		if (m_gfx_stale)
		{
			// Replace the overlay with one built from the latest readback and toggle state.
			RebuildGfx();
			m_gfx_stale = false;
		}
		if (m_gfx)
			m_gfx->AddToScene(scene);
	}

	// Toggle the floor/lid/domain grid overlay.
	void AtmosphereVisual::ShowGrid(bool show)
	{
		// Mark stale so the next render shows the change, even while the simulation is paused.
		m_show_grid = show;
		m_gfx_stale = true;
	}

	// Toggle tracer particle rendering.
	void AtmosphereVisual::ShowParticles(bool show)
	{
		// Mark stale so the next render shows the change, even while the simulation is paused.
		m_show_particles = show;
		m_gfx_stale = true;
	}

	// Toggle pressure-node rendering.
	void AtmosphereVisual::ShowPressureNodes(bool show)
	{
		// Mark stale so the next render shows the change, even while the simulation is paused.
		m_show_pressure_nodes = show;
		m_gfx_stale = true;
	}

	// Return true when the grid overlay is enabled.
	bool AtmosphereVisual::ShowGrid() const
	{
		// The UI uses this to keep menu checks in sync.
		return m_show_grid;
	}

	// Return true when tracer particle rendering is enabled.
	bool AtmosphereVisual::ShowParticles() const
	{
		// The UI uses this to keep menu checks in sync.
		return m_show_particles;
	}

	// Return true when pressure-node rendering is enabled.
	bool AtmosphereVisual::ShowPressureNodes() const
	{
		// The UI uses this to keep menu checks in sync.
		return m_show_pressure_nodes;
	}

	// Rebuild the LDraw diagnostic overlay from the latest CPU-visible state.
	void AtmosphereVisual::RebuildGfx()
	{
		// All diagnostics are regenerated together because the particle cloud already changes every frame.
		ldraw::Builder ldr;
		auto& group = ldr.Group("atmosphere_visual", 0xFFFFFFFFU);
		auto const& grid = m_desc.m_config.m_grid;
		auto const min_corner = grid.m_origin;
		auto const max_corner = v4{ grid.m_origin.x + grid.m_cell_count.x * grid.m_dx, grid.m_origin.y + grid.m_cell_count.y * grid.m_dx, grid.m_lid_z, 1.0f };

		if (m_show_grid)
		{
			// Draw the domain box and readable floor/lid lattices with automatic line decimation.
			auto& lines = group.Line("grid", 0x80404040U).width(1.0f).per_item_colour();
			AddBoxLines(lines, min_corner, max_corner, 0xFF202020U);
			auto const stride_x = std::max(1, grid.m_cell_count.x / std::max(m_desc.m_visual.m_grid_line_limit, 1));
			auto const stride_y = std::max(1, grid.m_cell_count.y / std::max(m_desc.m_visual.m_grid_line_limit, 1));
			for (int x = 0; x <= grid.m_cell_count.x; x += stride_x)
			{
				auto const wx = grid.m_origin.x + x * grid.m_dx;
				lines.line(v4{wx, min_corner.y, min_corner.z, 1.0f}, v4{wx, max_corner.y, min_corner.z, 1.0f}, 0x60303030U);
				lines.line(v4{wx, min_corner.y, max_corner.z, 1.0f}, v4{wx, max_corner.y, max_corner.z, 1.0f}, 0x60303030U);
			}
			for (int y = 0; y <= grid.m_cell_count.y; y += stride_y)
			{
				auto const wy = grid.m_origin.y + y * grid.m_dx;
				lines.line(v4{min_corner.x, wy, min_corner.z, 1.0f}, v4{max_corner.x, wy, min_corner.z, 1.0f}, 0x60303030U);
				lines.line(v4{min_corner.x, wy, max_corner.z, 1.0f}, v4{max_corner.x, wy, max_corner.z, 1.0f}, 0x60303030U);
			}
		}

		if (m_show_particles && !m_particles.empty())
		{
			// Point sprites show sampled air temperature for each tracer.
			auto& points = group.Point("tracers", 0xFFFFFFFFU).size(m_desc.m_visual.m_particle_size).style(ldraw::seri::PointStyle{"Circle"}).depth(false);
			for (auto const& particle : m_particles)
				points.pt(particle.m_position, TemperatureColour(particle.m_temperature, m_desc.m_visual.m_min_temperature, m_desc.m_visual.m_max_temperature));
		}

		if (m_show_pressure_nodes)
		{
			// Transparent spheres show the Gaussian influence radius, with opacity scaled by strength relative to the strongest node.
			// Spheres sit a quarter of the way up the domain because the forcing is a column-wide surface with no height of its own.
			auto const& nodes = m_desc.m_forcing.m_nodes;
			auto max_strength = 1.0f;
			for (auto const& node : nodes)
				max_strength = std::max(max_strength, std::abs(node.m_strength));

			// Draw each node as a sphere plus a sign ring.
			for (auto const& node : nodes)
			{
				// Colour by the air temperature the node imposes: reference surface temperature plus the outside and node offsets.
				auto const alpha = 48 + static_cast<int>(96.0f * std::clamp(std::abs(node.m_strength) / max_strength, 0.0f, 1.0f));
				auto const node_temperature = m_desc.m_config.m_reference.Temperature(0.0f) + m_desc.m_reservoir_temperature_offset + node.m_temperature_offset;
				auto const colour = TemperatureColour(node_temperature, m_desc.m_visual.m_min_temperature, m_desc.m_visual.m_max_temperature, alpha);
				auto const centre = v4{node.m_centre.x, node.m_centre.y, grid.m_origin.z + 0.25f * (grid.m_lid_z - grid.m_origin.z), 1.0f};
				group.Sphere("pressure_node", colour).sphere(node.m_radius).pos(centre).solid(true);

				// Highs get a white horizontal ring and lows a black vertical ring, so the sign reads at a glance.
				auto const is_high = node.m_strength >= 0.0f;
				auto const ring_axis = is_high ? v4{0, 1, 0, 0} : v4{0, 0, 1, 0};
				auto& ring = group.Line("pressure_node_ring", is_high ? 0xFFFFFFFFU : 0xFF000000U).width(2.0f);
				constexpr int RingSegments = 48;
				for (int i = 0; i != RingSegments; ++i)
				{
					// Each segment joins two consecutive points on the ring in the x/ring_axis plane.
					auto const a0 = constants<float>::tau * float(i) / float(RingSegments);
					auto const a1 = constants<float>::tau * float(i + 1) / float(RingSegments);
					auto const p0 = centre + node.m_radius * (std::cos(a0) * v4{1, 0, 0, 0} + std::sin(a0) * ring_axis);
					auto const p1 = centre + node.m_radius * (std::cos(a1) * v4{1, 0, 0, 0} + std::sin(a1) * ring_axis);
					ring.line(p0, p1);
				}
			}
		}

		if (m_desc.m_visual.m_show_heat_sources)
		{
			// Heat sources are wire spheres so their radius can be compared with nearby tracer motion.
			for (auto const& source : m_desc.m_heat_sources)
				group.Sphere("heat_source", 0xFFFFA000U).sphere(source.m_radius).pos(source.m_centre).wireframe(true).solid(false);
		}

		auto result = rdr12::ldraw::Parse(m_rdr, ldr.ToBinary());
		m_gfx = !result.m_objects.empty() ? result.m_objects.front() : nullptr;
	}
}
