//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2016
//*********************************************
#include "physics/src/dll/atmosphere_record.h"

namespace pr::physics
{
	namespace
	{
		// Convert a native atmosphere call's returned input check into the ABI call's rejection. A rejected native call changed no state and
		// did no work, so the atmosphere stays usable. Exceptions from the native call are failures and are not converted.
		ApiRejection InvalidInput(atmosphere::AtmosphereInputCheck check)
		{
			// The native message is a static string.
			if (!check.Accepted())
				return ApiRejection{ .m_status = EStatus::InvalidArgument, .m_message = check.m_rejection };

			return {};
		}
	}

	// Create the solver and, when 'tracers' has particles, the tracers.
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
		// Tracers are optional so field-only atmospheres pay no particle cost.
		if (tracers.m_particle_count > 0)
		{
			m_tracers = std::make_unique<atmosphere::AtmosphereTracers>(m_solver, m_gpu, tracers, shader_cache);
			m_published_slot = m_tracers->CurrentSlot();
		}
	}

	// Run the recorded initialization after ownership is established.
	void AtmosphereRecord::Initialise()
	{
		// The registry, rather than a constructor stack frame, retains every submitted resource if this fails.
		Call([&]() -> ApiRejection
		{
			// Initialization failures have the same terminal state as failures in later work.
			m_gpu.m_job.Run();
			return {};
		});
	}

	// Retire the job before releasing resources or pending readback handles.
	HRESULT AtmosphereRecord::TryClose(DWORD timeout) noexcept
	{
		// Closure is terminal even when the caller needs to retry the wait.
		m_failed = true;
		auto const result = m_gpu.m_job.TryClose(timeout);
		if (result == S_OK)
			m_pending = {};

		return result;
	}

	// Record and submit one step.
	void AtmosphereRecord::BeginStep(AtmosphereStepDesc const& s)
	{
		Call([&]() -> ApiRejection
		{
			// Reject the request before anything is recorded, so a rejected step leaves the atmosphere usable. These checks do not allocate.
			if (Pending())
				return ApiRejection{ .m_status = EStatus::StepPending, .m_message = "An atmosphere step is already in flight" };
			if (s.heat_source_count < 0 || (s.heat_source_count != 0) != (s.heat_sources != nullptr))
				return ApiRejection{ .m_status = EStatus::InvalidArgument, .m_message = "Atmosphere heat source pointer and count disagree" };
			auto const heat_in = std::span{ s.heat_sources, static_cast<size_t>(s.heat_source_count) };
			if (!std::ranges::all_of(heat_in, [](AtmosphereHeatSource const& h) { return h.centre.w == 1.0f; }))
				return ApiRejection{ .m_status = EStatus::InvalidArgument, .m_message = "Atmosphere heat source centre is not a point" };
			if (s.probe_count < 0 || s.probe_count > atmosphere::AtmosphereProbes::MaxPoints || (s.probe_count != 0) != (s.probes != nullptr))
				return ApiRejection{ .m_status = EStatus::InvalidArgument, .m_message = "Atmosphere probe pointer and count disagree, or the count exceeds the limit" };
			if (!std::isfinite(s.surface_wind_height) || s.surface_wind_height < 0.0f)
				return ApiRejection{ .m_status = EStatus::InvalidArgument, .m_message = "Atmosphere surface wind height must be finite and not negative" };
			auto const probes_in = std::span{ s.probes, static_cast<size_t>(s.probe_count) };
			if (!std::ranges::all_of(probes_in, [](Vector4 const& p) { return p.w == 1.0f && std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z); }))
				return ApiRejection{ .m_status = EStatus::InvalidArgument, .m_message = "Invalid atmosphere probe point" };
			if (m_tracers && !m_tracers->HasFreeSlot())
				return ApiRejection{ .m_status = EStatus::InvalidArgument, .m_message = "Every atmosphere tracer slot is held; release a held slot before stepping" };

			// Convert the inputs to the solver's layouts.
			auto heat_sources = std::vector<atmosphere::AtmosphereHeatSource>{};
			heat_sources.reserve(heat_in.size());
			for (auto const& h : heat_in)
			{
				heat_sources.push_back(atmosphere::AtmosphereHeatSource{
					.m_centre = v4{ h.centre.x, h.centre.y, h.centre.z, h.centre.w },
					.m_radius = h.radius,
					.m_heating_rate = h.heating_rate,
					.m_target_temperature = h.target_temperature,
					.m_relaxation_rate = h.relaxation_rate,
				});
			}
			auto probes = std::vector<v4>{};
			probes.reserve(probes_in.size());
			for (auto const& p : probes_in)
				probes.push_back(v4{ p.x, p.y, p.z, p.w });

			// Record the step, tracer advection, probe samples, and surface wind. The finishing call collects the copies. The solver's own step
			// checks are the last rejection; after them, every failure leaves a partly recorded step.
			auto& job = m_gpu.m_job;
			if (auto const rejection = InvalidInput(m_solver.Step(job, s.dt, heat_sources)); rejection.Rejected())
				return rejection;

			if (m_tracers)
				m_tracers->Advect(job, s.dt);

			m_probes.Record(job, probes);
			if (s.surface_wind_height > 0.0f)
				m_surface_wind.Record(job, s.surface_wind_height);

			m_pending = job.Submit();
			return {};
		});
	}

	// Finish the step in flight if the GPU has executed it.
	bool AtmosphereRecord::PollStep()
	{
		auto idle = false;
		Call([&]() -> ApiRejection
		{
			// Finishing a step that the GPU has already executed does not wait.
			if (Pending() && m_gpu.m_job.m_gsync.CompletedSyncPoint() >= m_pending.m_sync_point)
				FinishStep();

			idle = !Pending();
			return {};
		});
		return idle;
	}

	// Wait for and finish the step in flight.
	void AtmosphereRecord::CompleteStep()
	{
		Call([&]() -> ApiRejection
		{
			// There is nothing to wait for without a submitted step.
			if (!Pending())
				return ApiRejection{ .m_status = EStatus::NoStepPending, .m_message = "No atmosphere step is in flight" };

			FinishStep();
			return {};
		});
	}

	// Copy the probe samples of the last finished step.
	void AtmosphereRecord::CopyProbes(AtmosphereProbeSample* samples, std::uint32_t capacity, std::uint32_t& required)
	{
		Call([&]() -> ApiRejection
		{
			// The published samples are CPU memory, so copying them never touches the GPU or the step in flight.
			// Report the size first so a sizing call succeeds with no buffer.
			auto const count = static_cast<std::uint32_t>(m_probe_samples.size());
			required = count;
			if (capacity < count || (count != 0 && samples == nullptr))
				return ApiRejection{ .m_status = EStatus::BufferTooSmall, .m_message = "Probe sample buffer is too small" };

			// Convert to the ABI layout.
			for (auto i = std::uint32_t{}; i != count; ++i)
			{
				auto const& p = m_probe_samples[i];
				samples[i] = AtmosphereProbeSample{
					.velocity = { p.m_velocity.x, p.m_velocity.y, p.m_velocity.z, 0.0f },
					.temperature = p.m_temperature,
					.inside = p.m_inside ? 1 : 0,
				};
			}
			return {};
		});
	}

	// Copy the surface wind of the last finished step.
	void AtmosphereRecord::CopySurfaceWind(Vector2* wind, std::uint32_t capacity, std::uint32_t& required)
	{
		Call([&]() -> ApiRejection
		{
			// The published wind is CPU memory, so copying it never touches the GPU or the step in flight.
			// Report the size first so a sizing call succeeds with no buffer.
			auto const count = static_cast<std::uint32_t>(m_surface_wind_field.size());
			required = count;
			if (capacity < count || (count != 0 && wind == nullptr))
				return ApiRejection{ .m_status = EStatus::BufferTooSmall, .m_message = "Surface wind buffer is too small" };

			// Convert to the ABI layout.
			for (auto i = std::uint32_t{}; i != count; ++i)
				wind[i] = Vector2{ m_surface_wind_field[i].x, m_surface_wind_field[i].y };
			return {};
		});
	}

	// Change the column floors and remap the field.
	void AtmosphereRecord::SetFloors(float const* floor_heights, std::int32_t count)
	{
		Call([&]() -> ApiRejection
		{
			// Reject the request before the field is read back or changed.
			if (Pending())
				return ApiRejection{ .m_status = EStatus::StepPending, .m_message = "Atmosphere floors cannot change while a step is in flight" };
			if (count < 0 || (count != 0) != (floor_heights != nullptr))
				return ApiRejection{ .m_status = EStatus::InvalidArgument, .m_message = "Atmosphere floor pointer and count disagree" };

			// Remapping records GPU work, which is run to completion here so the next step starts from the new layers. The solver rejects
			// invalid floors before it reads back the field. Its read back runs the job's sync observers, so anything they throw is a failure.
			auto const floors = std::span{ floor_heights, static_cast<size_t>(count) };
			auto& job = m_gpu.m_job;
			if (auto const rejection = InvalidInput(m_solver.RemapFloors(job, floors)); rejection.Rejected())
				return rejection;

			job.Run();
			return {};
		});
	}

	// Set the floor temperature under each column.
	void AtmosphereRecord::SetFloorTemperatures(float const* floor_temperatures, std::int32_t count)
	{
		Call([&]() -> ApiRejection
		{
			// The solver keeps a CPU copy that the next step uploads, so this is allowed while a step is in flight.
			if (count < 0 || (count != 0) != (floor_temperatures != nullptr))
				return ApiRejection{ .m_status = EStatus::InvalidArgument, .m_message = "Atmosphere floor temperature pointer and count disagree" };

			// The solver rejects invalid values without changing the stored ones.
			auto const temperatures = std::span{ floor_temperatures, static_cast<size_t>(count) };
			return InvalidInput(m_solver.SetFloorTemperatures(temperatures));
		});
	}

	// Set the air outside the open faces.
	void AtmosphereRecord::SetOutsideAir(AtmosphereOutsideAir const* outside_air, std::int32_t count)
	{
		Call([&]() -> ApiRejection
		{
			// The solver keeps a CPU copy that the next step uploads, so this is allowed while a step is in flight.
			// Reject the ABI layout's own invalid fields before converting.
			if (count < 0 || (count != 0) != (outside_air != nullptr))
				return ApiRejection{ .m_status = EStatus::InvalidArgument, .m_message = "Atmosphere outside air pointer and count disagree" };
			auto const air_in = std::span{ outside_air, static_cast<size_t>(count) };
			if (!std::ranges::all_of(air_in, [](AtmosphereOutsideAir const& a) { return a.reserved == 0; }))
				return ApiRejection{ .m_status = EStatus::InvalidArgument, .m_message = "Invalid atmosphere outside air reserved field" };

			// Convert to the solver's layout. The solver checks the whole input before its pending values change.
			auto air = std::vector<atmosphere::AtmosphereOutsideAir>{};
			air.reserve(air_in.size());
			for (auto const& a : air_in)
				air.push_back(atmosphere::AtmosphereOutsideAir{ .m_wind = v2{ a.wind.x, a.wind.y }, .m_temperature_offset = a.temperature_offset });

			return InvalidInput(m_solver.SetOutsideAir(air));
		});
	}

	// Read back all tracer particles.
	void AtmosphereRecord::CopyTracers(AtmosphereTracerParticle* particles, std::uint32_t capacity, std::uint32_t& required)
	{
		Call([&]() -> ApiRejection
		{
			// The read back uses the atmosphere's queue, so no step may be in flight.
			if (Pending())
				return ApiRejection{ .m_status = EStatus::StepPending, .m_message = "Atmosphere tracers cannot be read while a step is in flight" };

			// Report the size before the read back so a sizing call does not touch the GPU.
			auto const count = m_tracers ? static_cast<std::uint32_t>(m_tracers->Config().m_particle_count) : 0U;
			required = count;
			if (capacity < count || (count != 0 && particles == nullptr))
				return ApiRejection{ .m_status = EStatus::BufferTooSmall, .m_message = "Tracer buffer is too small" };
			if (count == 0)
				return {};

			// No step is in flight, so the current slot is the published one.
			auto const source = m_tracers->ReadBack(m_gpu.m_job);
			for (auto i = std::uint32_t{}; i != count; ++i)
			{
				auto const& p = source[i];
				particles[i] = AtmosphereTracerParticle{
					.position = { p.m_position.x, p.m_position.y, p.m_position.z, 1.0f },
					.temperature = p.m_temperature,
					.age = p.m_age,
					.speed = p.m_speed,
					.reserved = 0.0f,
				};
			}
			return {};
		});
	}

	// Hold the published tracer slot and return an owned COM reference to its buffer.
	void AtmosphereRecord::AcquireTracers(AtmosphereTracerSlot& slot)
	{
		Call([&]() -> ApiRejection
		{
			// Holding the published slot is safe while a step is in flight, because that step reads it and writes another slot.
			if (!m_tracers)
				return ApiRejection{ .m_status = EStatus::InvalidArgument, .m_message = "Atmosphere has no tracers" };

			// Hold before handing out the reference, so the hold and the COM reference are both owned by the caller on return.
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

	// Release one hold on tracer 'slot'.
	void AtmosphereRecord::ReleaseTracers(std::int32_t slot, ID3D12Fence* fence, std::uint64_t value)
	{
		Call([&]() -> ApiRejection
		{
			// A release must match a hold. The next step that writes the slot makes its queue wait for the fence, so no CPU wait is needed here.
			// The tracers reject an out-of-range or unheld slot without changing any hold.
			if (!m_tracers)
				return ApiRejection{ .m_status = EStatus::InvalidArgument, .m_message = "Atmosphere has no tracers" };

			return InvalidInput(m_tracers->Release(slot, fence, value));
		});
	}

	// Read back the field as cell-centre samples, with optional stats.
	void AtmosphereRecord::CopyCellStates(AtmosphereCellState* cells, std::uint32_t capacity, std::uint32_t& required, AtmosphereStats* stats)
	{
		Call([&]() -> ApiRejection
		{
			// The read back uses the atmosphere's queue, so no step may be in flight.
			if (Pending())
				return ApiRejection{ .m_status = EStatus::StepPending, .m_message = "Atmosphere cells cannot be read while a step is in flight" };

			// Report the size before the read back so a sizing call does not touch the GPU.
			auto const count = static_cast<std::uint32_t>(m_solver.Config().m_grid.CellCount());
			required = count;
			if (capacity < count || cells == nullptr)
				return ApiRejection{ .m_status = EStatus::BufferTooSmall, .m_message = "Cell buffer is too small" };

			// Read the staggered field and convert it to cell-centre samples.
			auto const state = m_solver.ReadBack(m_gpu.m_job);
			auto const states = m_solver.CellStates(state);
			for (auto i = std::uint32_t{}; i != count; ++i)
			{
				auto const& c = states[i];
				cells[i] = AtmosphereCellState{
					.velocity = { c.m_velocity.x, c.m_velocity.y, c.m_velocity.z, 0.0f },
					.temperature = c.m_temperature,
				};
			}
			if (stats != nullptr)
			{
				// The caller has checked the stats header.
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

	// True while a submitted step has not been finished.
	bool AtmosphereRecord::Pending() const
	{
		// 'Complete' clears the handle once the GPU has finished the step.
		return static_cast<bool>(m_pending);
	}

	// Wait for the submitted step, then publish the tracer slot, probe samples, and surface wind it wrote.
	void AtmosphereRecord::FinishStep()
	{
		// The step's outputs are published only after its GPU work has finished. A failure after 'Complete' can leave the outputs partly
		// published, which is why the caller marks the atmosphere failed rather than reporting the mix as a finished step.
		m_gpu.m_job.Complete(m_pending);
		if (m_tracers)
			m_published_slot = m_tracers->CurrentSlot();

		// Every step replaces its probes and surface wind, possibly with none, so stale samples never describe a later field.
		m_probe_samples = m_probes.Resolve();
		m_surface_wind_field = m_surface_wind.Resolve();
	}
}
