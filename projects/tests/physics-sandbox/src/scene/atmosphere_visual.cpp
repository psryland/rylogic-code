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

		// Return an ARGB colour on a blue-cyan-green-yellow-red speed ramp, for 't' in [0, 1].
		uint32_t SpeedColour(float t)
		{
			// Four equal segments between five key colours give clear bands for slow, medium and fast air.
			static constexpr uint32_t keys[] = { 0xFF2040FFU, 0xFF00D0FFU, 0xFF20E040U, 0xFFFFE000U, 0xFFFF2010U };
			auto const s = std::clamp(t, 0.0f, 1.0f) * 4.0f;
			auto const i = std::min(static_cast<int>(s), 3);
			auto const f = s - static_cast<float>(i);
			auto const lerp_channel = [&](int shift)
			{
				// Blend one 8-bit channel between the two keys of this segment.
				auto const a = static_cast<float>((keys[i] >> shift) & 0xFF);
				auto const b = static_cast<float>((keys[i + 1] >> shift) & 0xFF);
				return static_cast<uint32_t>(a + f * (b - a) + 0.5f) << shift;
			};
			return 0xFF000000U | lerp_channel(16) | lerp_channel(8) | lerp_channel(0);
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
		, m_pending()
		, m_unstepped_time(0.0f)
		, m_rdr(rdr)
		, m_gfx()
		, m_tracer_gfx()
		, m_tracer_palette()
		, m_gfx_stale(true)
		, m_tracers_stale(false)
		, m_show_grid(m_desc.m_visual.m_show_grid)
		, m_show_particles(m_desc.m_visual.m_show_particles)
	{
		// Tracers are optional so diagnostic-only atmosphere scenes can omit particle cost.
		if (m_desc.m_tracers.m_particle_count > 0)
		{
			m_tracers = std::make_unique<physics::atmosphere::AtmosphereTracers>(m_solver, m_gpu, m_desc.m_tracers, &shader_cache);
			m_particles = m_tracers->ReadBack(m_gpu.m_job);
			CreateTracerGfx();
			m_tracers_stale = true;
		}
	}

	// Wait for any climate step still in flight before the GPU resources are released.
	AtmosphereVisual::~AtmosphereVisual()
	{
		// The solver and tracer buffers must outlive GPU work that references them.
		if (m_pending)
			m_gpu.m_job.Abandon(m_pending);
	}

	// Advance simulated time, collecting finished climate steps and submitting new ones at the fixed climate rate.
	void AtmosphereVisual::Step(float dt)
	{
		// Collect the step in flight only once the GPU has finished it, so the frame never waits on the climate.
		auto& job = m_gpu.m_job;
		if (m_pending)
		{
			// Keep showing the previous particles while the GPU is still busy.
			if (job.m_gsync.CompletedSyncPoint() < m_pending.m_sync_point)
				return;

			job.Complete(m_pending);
			if (m_tracers != nullptr)
			{
				// The read back recorded with the step is now valid.
				m_particles = m_tracers->ResolveReadBack();
				m_tracers_stale = true;
			}
		}

		// Submit one climate step per elapsed period. Time beyond one pending period is dropped, so a slow GPU slows the climate
		// rather than building an ever-growing backlog.
		auto const step_period = 1.0f / m_desc.m_step_rate;
		m_unstepped_time = std::min(m_unstepped_time + dt, 2.0f * step_period);
		if (m_unstepped_time < step_period)
			return;

		m_unstepped_time -= step_period;

		// The scene's heat sources and outside air are static, so the same inputs drive every step.
		auto const sources = physics::atmosphere::AtmosphereStepSources{
			.m_heat_sources = m_desc.m_heat_sources,
			.m_outside_air = m_desc.m_outside_air,
		};

		// Record the solver, tracers, and particle copy in submission order, then submit without waiting.
		m_solver.Step(job, step_period, sources);
		if (m_tracers != nullptr)
		{
			// The particle copy is collected when this submission completes.
			m_tracers->Advect(job, step_period);
			m_tracers->RecordReadBack(job);
		}
		m_pending = job.Submit();
	}

	// Add current atmosphere diagnostics to the render scene.
	void AtmosphereVisual::AddToScene(rdr12::Scene& scene)
	{
		// Rebuild here, after the scene has cleared its drawlists, because releasing a model that is still in a drawlist is an error.
		// The static overlay only changes when a toggle changes.
		if (m_gfx_stale)
		{
			// Replace the overlay with one built from the current toggle state.
			RebuildGfx();
			m_gfx_stale = false;
		}
		if (m_gfx)
			m_gfx->AddToScene(scene);

		// Tracer vertices are refreshed in place, and only when visible, because rebuilding a large point model every frame is expensive.
		if (m_show_particles && m_tracer_gfx)
		{
			// Upload the latest readback before adding the model to the drawlist.
			if (m_tracers_stale)
			{
				UpdateTracerGfx();
				m_tracers_stale = false;
			}
			m_tracer_gfx->AddToScene(scene);
		}
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
		// The tracer model persists, so the next AddToScene simply includes or omits it.
		m_show_particles = show;
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

	// Rebuild the static LDraw diagnostic overlay from the current toggle state.
	void AtmosphereVisual::RebuildGfx()
	{
		// Grid and heat sources share one object for straightforward toggling. Tracers live in 'm_tracer_gfx'.
		ldraw::Builder ldr;
		auto& group = ldr.Group("atmosphere_visual", 0xFFFFFFFFU);
		auto const& grid = m_desc.m_config.m_grid;
		auto const min_corner = grid.m_origin;
		auto const max_corner = v4{ grid.m_origin.x + grid.m_cell_count.x * grid.m_dx, grid.m_origin.y + grid.m_cell_count.y * grid.m_dx, grid.m_lid_z, 1.0f };

		if (m_show_grid)
		{
			// Draw the domain box and readable floor/lid lattices with automatic line decimation. A flat floor lattice would cut through terrain, so terrain floors only get the lid lattice.
			auto& lines = group.Line("grid", 0x80404040U).width(1.0f).per_item_colour();
			AddBoxLines(lines, min_corner, max_corner, 0xFF202020U);
			auto const draw_floor = !m_desc.m_terrain_floor;
			auto const stride_x = std::max(1, grid.m_cell_count.x / std::max(m_desc.m_visual.m_grid_line_limit, 1));
			auto const stride_y = std::max(1, grid.m_cell_count.y / std::max(m_desc.m_visual.m_grid_line_limit, 1));
			for (int x = 0; x <= grid.m_cell_count.x; x += stride_x)
			{
				// One line across the lid, and the floor when it is flat.
				auto const wx = grid.m_origin.x + x * grid.m_dx;
				if (draw_floor)
					lines.line(v4{wx, min_corner.y, min_corner.z, 1.0f}, v4{wx, max_corner.y, min_corner.z, 1.0f}, 0x60303030U);

				lines.line(v4{wx, min_corner.y, max_corner.z, 1.0f}, v4{wx, max_corner.y, max_corner.z, 1.0f}, 0x60303030U);
			}
			for (int y = 0; y <= grid.m_cell_count.y; y += stride_y)
			{
				// One line across the lid, and the floor when it is flat.
				auto const wy = grid.m_origin.y + y * grid.m_dx;
				if (draw_floor)
					lines.line(v4{min_corner.x, wy, min_corner.z, 1.0f}, v4{max_corner.x, wy, min_corner.z, 1.0f}, 0x60303030U);

				lines.line(v4{min_corner.x, wy, max_corner.z, 1.0f}, v4{max_corner.x, wy, max_corner.z, 1.0f}, 0x60303030U);
			}
		}

		if (m_desc.m_visual.m_show_heat_sources)
		{
			// Heat sources are wire spheres so their radius can be compared with nearby tracer motion.
			for (auto const& source : m_desc.m_heat_sources)
				group.Sphere("heat_source", 0xFFFFA000U).sphere(source.m_radius).pos(source.m_centre).wireframe(true).solid(false);
		}

		if (m_desc.m_visual.m_show_obstacles)
		{
			// Obstacles are semi-transparent so tracers passing behind them stay visible. LDraw cylinders are centred and lie along Z.
			auto const height = grid.m_lid_z - grid.m_origin.z;
			for (auto const& cylinder : m_desc.m_cylinders)
				group.Cylinder("obstacle", 0x80A0A0A0U).cylinder(height, cylinder.m_radius).facets(1, 40).pos(v4{ cylinder.m_centre.x, cylinder.m_centre.y, grid.m_origin.z + 0.5f * height, 1.0f });
		}

		auto result = rdr12::ldraw::Parse(m_rdr, ldr.ToBinary());
		m_gfx = !result.m_objects.empty() ? result.m_objects.front() : nullptr;
	}

	// Create the tracer point sprite model sized for the tracer set.
	void AtmosphereVisual::CreateTracerGfx()
	{
		// Create the point list first; the vertex colours are filled from the palette by UpdateTracerGfx below.
		ldraw::Builder ldr;
		auto& points = ldr.Point("atmosphere_tracers", 0xFFFFFFFFU).size(m_desc.m_visual.m_particle_size).style(ldraw::seri::PointStyle{"Circle"}).depth(false);
		for (auto const& particle : m_particles)
			points.pt(particle.m_position, 0xFFFFFFFFU);

		auto result = rdr12::ldraw::Parse(m_rdr, ldr.ToBinary());
		m_tracer_gfx = !result.m_objects.empty() ? result.m_objects.front() : nullptr;
		if (m_tracer_gfx == nullptr || m_tracer_gfx->m_model == nullptr || m_tracer_gfx->m_model->m_vcount != isize(m_particles))
			throw std::runtime_error("Atmosphere tracer model must have one vertex per tracer");

		// Precompute the colour ramp so the per-frame vertex update is a table lookup per tracer.
		for (int i = 0; i != isize(m_tracer_palette); ++i)
		{
			auto const t = static_cast<float>(i) / static_cast<float>(isize(m_tracer_palette) - 1);
			switch (m_desc.m_visual.m_colour_by)
			{
				case scene_loader::EAtmosphereColourBy::Temperature:
				{
					auto const temperature = m_desc.m_visual.m_min_temperature + t * (m_desc.m_visual.m_max_temperature - m_desc.m_visual.m_min_temperature);
					m_tracer_palette[i] = Colour(Colour32(TemperatureColour(temperature, m_desc.m_visual.m_min_temperature, m_desc.m_visual.m_max_temperature)));
					break;
				}
				case scene_loader::EAtmosphereColourBy::Speed:
				{
					m_tracer_palette[i] = Colour(Colour32(SpeedColour(t)));
					break;
				}
				default:
				{
					throw std::runtime_error("Unknown atmosphere tracer colour mode");
				}
			}
		}

		// Tracers can move anywhere in the domain, so bound the model by the domain rather than the initial particle positions.
		auto const& grid = m_desc.m_config.m_grid;
		auto const min_corner = grid.m_origin;
		auto const max_corner = v4{ grid.m_origin.x + grid.m_cell_count.x * grid.m_dx, grid.m_origin.y + grid.m_cell_count.y * grid.m_dx, grid.m_lid_z, 1.0f };
		m_tracer_gfx->m_model->m_bbox = BBox((min_corner + max_corner) * 0.5f, (max_corner - min_corner) * 0.5f);
	}

	// Overwrite the tracer model's vertices with the latest particle readback.
	void AtmosphereVisual::UpdateTracerGfx()
	{
		// The tracer count is fixed at creation, so the vertex buffer size always matches the readback.
		auto& model = *m_tracer_gfx->m_model.get();
		rdr12::ResourceFactory factory(m_rdr);
		auto update = model.UpdateVertices(factory.CmdList(), factory.UploadBuffer(), { 0, isize(m_particles) });
		auto vout = update.ptr<rdr12::Vert>();

		// Map the coloured property to a palette index. The loop avoids helper calls because this runs for every tracer every frame, including in Debug builds.
		auto const palette_max = isize(m_tracer_palette) - 1;
		auto field = &physics::atmosphere::AtmosphereTracerParticle::m_temperature;
		auto min_value = 0.0f;
		auto max_value = 1.0f;
		switch (m_desc.m_visual.m_colour_by)
		{
			case scene_loader::EAtmosphereColourBy::Temperature:
			{
				field = &physics::atmosphere::AtmosphereTracerParticle::m_temperature;
				min_value = m_desc.m_visual.m_min_temperature;
				max_value = m_desc.m_visual.m_max_temperature;
				break;
			}
			case scene_loader::EAtmosphereColourBy::Speed:
			{
				field = &physics::atmosphere::AtmosphereTracerParticle::m_speed;
				min_value = m_desc.m_visual.m_min_speed;
				max_value = m_desc.m_visual.m_max_speed;
				break;
			}
			default:
			{
				throw std::runtime_error("Unknown atmosphere tracer colour mode");
			}
		}
		auto const palette_scale = palette_max / std::max(max_value - min_value, 0.001f);
		auto vert = rdr12::Vert{ .m_vert = v4::Origin(), .m_diff = {}, .m_norm = v4::Zero(), .m_tex0 = v2::Zero(), .m_idx0 = iv2::Zero() };
		auto const* particle = m_particles.data();
		for (auto const* end = particle + m_particles.size(); particle != end; ++particle)
		{
			// Point sprites only use position and colour. The template vertex keeps the unused fields defined.
			auto index = static_cast<int>((particle->*field - min_value) * palette_scale);
			index = index < 0 ? 0 : index > palette_max ? palette_max : index;
			vert.m_vert.x = particle->m_position.x;
			vert.m_vert.y = particle->m_position.y;
			vert.m_vert.z = particle->m_position.z;
			vert.m_diff = m_tracer_palette[index];
			*vout++ = vert;
		}
		update.Commit();
	}
}
