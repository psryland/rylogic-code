//*********************************************
// Physics Engine
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
#include "physics/src/dll/atmosphere_record.h"

namespace pr::physics
{
	namespace
	{
		// Convert only an explicit native input rejection, never an exception from possibly partial work.
		ApiRejection InvalidInput(atmosphere::AtmosphereInputCheck check)
		{
			if (!check.Accepted())
				return {EStatus::InvalidArgument, check.m_rejection};

			return {};
		}
	}

	AtmosphereRecord::AtmosphereRecord(ID3D12Device4* device, atmosphere::AtmosphereConfig config, atmosphere::AtmosphereTracerConfig const& tracers, IShaderCache* shader_cache)
		: m_gpu(device)
		, m_solver(m_gpu, std::move(config), shader_cache)
		, m_tracers()
		, m_probes(m_solver, m_gpu, shader_cache)
		, m_surface_wind(m_solver, m_gpu, shader_cache)
		, m_published_slot(-1)
		, m_pending()
		, m_probe_samples()
		, m_surface_wind_field()
		, m_failed(false)
	{
		// Field-only atmospheres allocate no tracer particles.
		if (tracers.m_particle_count > 0)
		{
			m_tracers = std::make_unique<atmosphere::AtmosphereTracers>(m_solver, m_gpu, tracers, shader_cache);
			m_published_slot = m_tracers->CurrentSlot();
		}
	}
	AtmosphereRecord::~AtmosphereRecord()
	{
		// Finish submitted work before solver members unwind, without recording, replay or observer callbacks.
		m_gpu.m_job.QuiesceForDestruction();
	}

	void AtmosphereRecord::BeginStep(AtmosphereStepDesc const& step)
	{
		Call([&]() -> ApiRejection
		{
			// Reject ABI-only invalid inputs and unavailable tracer capacity before recording any part of the step.
			if (Pending())
				return {EStatus::StepPending, "An atmosphere step is already in flight"};

			if (step.heat_source_count < 0 || (step.heat_source_count != 0) != (step.heat_sources != nullptr))
				return {EStatus::InvalidArgument, "Atmosphere heat source pointer and count disagree"};

			auto const heat_in = std::span{step.heat_sources, static_cast<size_t>(step.heat_source_count)};
			if (!std::ranges::all_of(heat_in, [](AtmosphereHeatSource const& h) { return h.centre.w == 1.0f; }))
				return {EStatus::InvalidArgument, "Atmosphere heat source centre is not a point"};

			if (step.probe_count < 0 || step.probe_count > atmosphere::AtmosphereProbes::MaxPoints || (step.probe_count != 0) != (step.probes != nullptr))
				return {EStatus::InvalidArgument, "Atmosphere probe pointer and count disagree, or the count exceeds the limit"};

			if (!std::isfinite(step.surface_wind_height) || step.surface_wind_height < 0.0f)
				return {EStatus::InvalidArgument, "Atmosphere surface wind height must be finite and not negative"};

			auto const probes_in = std::span{step.probes, static_cast<size_t>(step.probe_count)};
			if (!std::ranges::all_of(probes_in, [](Vector4 const& p) { return p.w == 1.0f && std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z); }))
				return {EStatus::InvalidArgument, "Invalid atmosphere probe point"};

			if (m_tracers && !m_tracers->HasFreeSlot())
				return {EStatus::InvalidArgument, "Every atmosphere tracer slot is held; release a held slot before stepping"};

			// Convert inputs before the native solver's final input check and recording boundary.
			auto heat_sources = std::vector<atmosphere::AtmosphereHeatSource>{};
			heat_sources.reserve(heat_in.size());
			for (auto const& h : heat_in)
			{
				heat_sources.push_back(atmosphere::AtmosphereHeatSource{
					.m_centre = v4{h.centre.x, h.centre.y, h.centre.z, h.centre.w},
					.m_radius = h.radius,
					.m_heating_rate = h.heating_rate,
					.m_target_temperature = h.target_temperature,
					.m_relaxation_rate = h.relaxation_rate,
				});
			}
			auto probes = std::vector<v4>{};
			probes.reserve(probes_in.size());
			for (auto const& p : probes_in)
				probes.push_back(v4{p.x, p.y, p.z, p.w});

			auto& job = m_gpu.m_job;
			if (auto const rejection = InvalidInput(m_solver.Step(job, step.dt, heat_sources)); rejection.Rejected())
				return rejection;

			// From here onward any exception may leave partial work, so Call permanently fails the owner.
			if (m_tracers)
				m_tracers->Advect(job, step.dt);

			m_probes.Record(job, probes);
			if (step.surface_wind_height > 0.0f)
				m_surface_wind.Record(job, step.surface_wind_height);

			m_pending = job.Submit();
			return {};
		});
	}
	bool AtmosphereRecord::PollStep()
	{
		auto idle = false;
		Call([&]() -> ApiRejection
		{
			if (Pending() && m_gpu.m_job.m_gsync.CompletedSyncPoint() >= m_pending.m_sync_point)
				FinishStep();

			idle = !Pending();
			return {};
		});
		return idle;
	}
	void AtmosphereRecord::CompleteStep()
	{
		Call([&]() -> ApiRejection
		{
			if (!Pending())
				return {EStatus::NoStepPending, "No atmosphere step is in flight"};

			FinishStep();
			return {};
		});
	}
	void AtmosphereRecord::CopyProbes(AtmosphereProbeSample* samples, std::uint32_t capacity, std::uint32_t& required)
	{
		Call([&]() -> ApiRejection
		{
			// Published CPU samples remain independent of an in-flight step. Capacity checks never start GPU work.
			auto const count = static_cast<std::uint32_t>(m_probe_samples.size());
			required = count;
			if (capacity < count || (count != 0 && samples == nullptr))
				return {EStatus::BufferTooSmall, "Probe sample buffer is too small"};

			for (auto i = std::uint32_t{}; i != count; ++i)
			{
				auto const& p = m_probe_samples[i];
				samples[i] = AtmosphereProbeSample{
					.velocity = {p.m_velocity.x, p.m_velocity.y, p.m_velocity.z, 0.0f},
					.temperature = p.m_temperature,
					.inside = p.m_inside ? 1 : 0,
				};
			}
			return {};
		});
	}
	void AtmosphereRecord::CopySurfaceWind(Vector2* wind, std::uint32_t capacity, std::uint32_t& required)
	{
		Call([&]() -> ApiRejection
		{
			// Sizing and copying published CPU output cannot interfere with an in-flight GPU step.
			auto const count = static_cast<std::uint32_t>(m_surface_wind_field.size());
			required = count;
			if (capacity < count || (count != 0 && wind == nullptr))
				return {EStatus::BufferTooSmall, "Surface wind buffer is too small"};

			for (auto i = std::uint32_t{}; i != count; ++i)
				wind[i] = Vector2{m_surface_wind_field[i].x, m_surface_wind_field[i].y};

			return {};
		});
	}
	void AtmosphereRecord::SetFloors(float const* floor_heights, std::int32_t count)
	{
		Call([&]() -> ApiRejection
		{
			if (Pending())
				return {EStatus::StepPending, "Atmosphere floors cannot change while a step is in flight"};

			if (count < 0 || (count != 0) != (floor_heights != nullptr))
				return {EStatus::InvalidArgument, "Atmosphere floor pointer and count disagree"};

			// Native validation precedes readback and its sync observers. An exception is not an input rejection.
			auto& job = m_gpu.m_job;
			if (auto const rejection = InvalidInput(m_solver.RemapFloors(job, {floor_heights, static_cast<size_t>(count)})); rejection.Rejected())
				return rejection;

			job.Run();
			return {};
		});
	}
	void AtmosphereRecord::SetFloorTemperatures(float const* floor_temperatures, std::int32_t count)
	{
		Call([&]() -> ApiRejection
		{
			if (count < 0 || (count != 0) != (floor_temperatures != nullptr))
				return {EStatus::InvalidArgument, "Atmosphere floor temperature pointer and count disagree"};

			return InvalidInput(m_solver.SetFloorTemperatures({floor_temperatures, static_cast<size_t>(count)}));
		});
	}
	void AtmosphereRecord::SetOutsideAir(AtmosphereOutsideAir const* outside_air, std::int32_t count)
	{
		Call([&]() -> ApiRejection
		{
			if (count < 0 || (count != 0) != (outside_air != nullptr))
				return {EStatus::InvalidArgument, "Atmosphere outside air pointer and count disagree"};

			auto const air_in = std::span{outside_air, static_cast<size_t>(count)};
			if (!std::ranges::all_of(air_in, [](AtmosphereOutsideAir const& a) { return a.reserved == 0; }))
				return {EStatus::InvalidArgument, "Invalid atmosphere outside air reserved field"};

			// Native validation checks every column before replacing any pending value.
			auto air = std::vector<atmosphere::AtmosphereOutsideAir>{};
			air.reserve(air_in.size());
			for (auto const& a : air_in)
				air.push_back(atmosphere::AtmosphereOutsideAir{.m_wind = v2{a.wind.x, a.wind.y}, .m_temperature_offset = a.temperature_offset});

			return InvalidInput(m_solver.SetOutsideAir(air));
		});
	}
	void AtmosphereRecord::CopyTracers(AtmosphereTracerParticle* particles, std::uint32_t capacity, std::uint32_t& required)
	{
		Call([&]() -> ApiRejection
		{
			if (Pending())
				return {EStatus::StepPending, "Atmosphere tracers cannot be read while a step is in flight"};

			auto const count = m_tracers ? static_cast<std::uint32_t>(m_tracers->Config().m_particle_count) : 0U;
			required = count;
			if (capacity < count || (count != 0 && particles == nullptr))
				return {EStatus::BufferTooSmall, "Tracer buffer is too small"};

			if (count == 0)
				return {};

			// Readback uses the owner's queue only after capacity and pending-state checks succeed.
			auto const source = m_tracers->ReadBack(m_gpu.m_job);
			for (auto i = std::uint32_t{}; i != count; ++i)
			{
				auto const& p = source[i];
				particles[i] = AtmosphereTracerParticle{
					.position = {p.m_position.x, p.m_position.y, p.m_position.z, 1.0f},
					.temperature = p.m_temperature,
					.age = p.m_age,
					.speed = p.m_speed,
					.reserved = 0.0f,
				};
			}
			return {};
		});
	}
	void AtmosphereRecord::AcquireTracers(AtmosphereTracerSlot& slot)
	{
		Call([&]() -> ApiRejection
		{
			if (!m_tracers)
				return {EStatus::InvalidArgument, "Atmosphere has no tracers"};

			// The published slot can be held while another slot is being written.
			auto const index = m_published_slot;
			auto* resource = m_tracers->Slot(index);
			m_tracers->Hold(index);
			resource->AddRef();
			slot = AtmosphereTracerSlot{
				.resource = resource,
				.slot = index,
				.count = static_cast<std::uint32_t>(m_tracers->Config().m_particle_count),
				.stride = static_cast<std::uint32_t>(sizeof(AtmosphereTracerParticle)),
				.reserved = 0,
			};
			return {};
		});
	}
	void AtmosphereRecord::ReleaseTracers(std::int32_t slot, ID3D12Fence* fence, std::uint64_t value)
	{
		Call([&]() -> ApiRejection
		{
			if (!m_tracers)
				return {EStatus::InvalidArgument, "Atmosphere has no tracers"};

			// Native release retains the fence before dropping the hold.
			return InvalidInput(m_tracers->Release(slot, fence, value));
		});
	}
	void AtmosphereRecord::CopyCellStates(AtmosphereCellState* cells, std::uint32_t capacity, std::uint32_t& required, AtmosphereStats* stats)
	{
		Call([&]() -> ApiRejection
		{
			if (Pending())
				return {EStatus::StepPending, "Atmosphere cells cannot be read while a step is in flight"};

			auto const count = static_cast<std::uint32_t>(m_solver.Config().m_grid.CellCount());
			required = count;
			if (capacity < count || cells == nullptr)
				return {EStatus::BufferTooSmall, "Cell buffer is too small"};

			// Convert the idle field; any readback or conversion failure makes the owner unusable.
			auto const state = m_solver.ReadBack(m_gpu.m_job);
			auto const states = m_solver.CellStates(state);
			for (auto i = std::uint32_t{}; i != count; ++i)
			{
				auto const& c = states[i];
				cells[i] = AtmosphereCellState{.velocity = {c.m_velocity.x, c.m_velocity.y, c.m_velocity.z, 0.0f}, .temperature = c.m_temperature};
			}
			if (stats != nullptr)
			{
				auto const st = m_solver.Stats(state);
				stats->max_speed = st.m_max_speed;
				stats->max_divergence = st.m_max_divergence;
				stats->rms_divergence = st.m_rms_divergence;
				stats->mean_top_temperature = st.m_mean_top_temperature;
				stats->peak_vertical_velocity = st.m_peak_vertical_velocity;
				stats->reserved = 0;
			}
			return {};
		});
	}
	bool AtmosphereRecord::Pending() const
	{
		return static_cast<bool>(m_pending);
	}
	void AtmosphereRecord::FinishStep()
	{
		// A failure after handle consumption may leave mixed outputs; the enclosing Call then prevents all later access.
		m_gpu.m_job.Complete(m_pending);
		if (m_tracers)
			m_published_slot = m_tracers->CurrentSlot();

		// Every completed step replaces both output collections, including when it produced no samples.
		m_probe_samples = m_probes.Resolve();
		m_surface_wind_field = m_surface_wind.Resolve();
	}
}
