//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "src/compute/interop/resolve_runner.h"
#include "src/compute/resolve.hlsl"

namespace pr::physics
{
	namespace
	{
		template <typename T> std::span<T const> SpanOf(std::vector<T> const& values)
		{
			return std::span<T const>{values.data(), values.size()};
		}

		int ThreadGroupCount(int item_count, int thread_count)
		{
			return std::max(1, (item_count + thread_count - 1) / thread_count);
		}

		float PositionCorrectionScale(EngineConfig const& config)
		{
			return config.position_iterations != 0 ? 1.0f / std::max(1, config.position_iterations) : 0.0f;
		}

		cbResolve MakeConstants(EngineConfig const& config, float dt, int body_count, int max_contacts, int colour = 0)
		{
			return cbResolve{
				.max_contacts = max_contacts,
				.body_count = body_count,
				.colour = colour,
				.tgs_steps = std::max(1, config.tgs_steps),
				.dt = dt,
				.tgs_velocity_bias_max = config.tgs_velocity_bias_max,
				.pad2 = 0,
				.pad3 = 0,
				.penetration_slop = config.penetration_slop,
				.velocity_baumgarte = config.velocity_baumgarte,
				.deep_penetration_threshold = config.deep_penetration_threshold,
				.deep_penetration_range = config.deep_penetration_range,
				.deep_penetration_baumgarte_min = config.deep_penetration_baumgarte_min,
				.deep_penetration_baumgarte_max = config.deep_penetration_baumgarte_max,
				.pad4 = 0,
				.pad5 = 0,
				.position_slop = config.position_slop,
				.position_baumgarte = config.position_baumgarte,
				.position_correction_scale = PositionCorrectionScale(config),
				.pad6 = 0,
			};
		}
	}

	ResolveInteropRunner::ResolveInteropRunner(EngineConfig const& config)
		: m_config(config)
		, m_dt()
		, m_body_count()
		, m_max_contacts()
		, m_counters()
		, m_bodies()
		, m_contacts()
		, m_materials()
		, m_colours()
		, m_contact_order()
		, m_contact_times()
	{
	}

	void ResolveInteropRunner::Run(ResolveRunnerBuffers buffers)
	{
		Load(buffers);

		ComputeCollisionTimes();
		SortContacts();
		AssignColours();

		auto const position_iterations = std::max(0, m_config.position_iterations);
		auto const solver_iterations = std::max(0, m_config.solver_iterations);
		auto const tgs_steps = std::max(1, m_config.tgs_steps);
		auto const tgs_dt = m_dt / static_cast<float>(tgs_steps);
		auto const position_iterations_per_tgs = position_iterations != 0 ? std::max(1, (position_iterations + tgs_steps - 1) / tgs_steps) : 0;
		auto const solver_iterations_per_tgs = solver_iterations != 0 ? std::max(1, (solver_iterations + tgs_steps - 1) / tgs_steps) : 0;
		if (tgs_steps != 1)
			TemporalDrift(-m_dt);

		for (int tgs_step = 0; tgs_step != tgs_steps; ++tgs_step)
		{
			if (tgs_steps != 1)
				TemporalDrift(tgs_dt);

			for (int iter = 0; iter != position_iterations_per_tgs; ++iter)
			{
				for (int colour = 0; colour != MaxColours; ++colour)
					PositionSolve(tgs_dt, colour);
			}

			for (int iter = 0; iter != solver_iterations_per_tgs; ++iter)
			{
				for (int colour = 0; colour != MaxColours; ++colour)
					ResolveVelocity(tgs_dt, colour);
			}
		}

		Store(buffers);
	}

	void ResolveInteropRunner::Load(ResolveRunnerBuffers buffers)
	{
		if (m_config.position_iterations < 0)
			throw std::invalid_argument("ResolveInteropRunner requires non-negative position_iterations");
		if (m_config.solver_iterations < 0)
			throw std::invalid_argument("ResolveInteropRunner requires non-negative solver_iterations");
		if (m_config.tgs_steps < 1)
			throw std::invalid_argument("ResolveInteropRunner requires positive tgs_steps");
		if (buffers.m_contacts.size() != 0 && buffers.m_bodies.size() == 0)
			throw std::invalid_argument("ResolveInteropRunner requires bodies when contacts are supplied");
		if (buffers.m_contacts.size() != 0 && buffers.m_materials.size() == 0)
			throw std::invalid_argument("ResolveInteropRunner requires materials when contacts are supplied");

		m_dt = buffers.m_dt;
		m_body_count = static_cast<int>(buffers.m_bodies.size());
		m_max_contacts = std::max(1, static_cast<int>(buffers.m_contacts.size()));
		m_counters = {GpuCollisionCounters{
			.pair_count = 0,
			.contact_count = static_cast<int>(buffers.m_contacts.size()),
		}};

		m_bodies.assign(buffers.m_bodies.begin(), buffers.m_bodies.end());
		m_contacts.assign(buffers.m_contacts.begin(), buffers.m_contacts.end());
		m_materials.assign(buffers.m_materials.begin(), buffers.m_materials.end());
		m_colours.assign(m_max_contacts, 0);
		m_contact_order.resize(m_max_contacts);
		m_contact_times.assign(m_max_contacts, 1e30f);
		std::iota(m_contact_order.begin(), m_contact_order.end(), 0u);
	}

	void ResolveInteropRunner::Store(ResolveRunnerBuffers buffers) const
	{
		if (buffers.m_bodies.size() != m_bodies.size())
			throw std::invalid_argument("ResolveInteropRunner output body buffer size changed");
		if (buffers.m_contacts.size() != m_contacts.size())
			throw std::invalid_argument("ResolveInteropRunner output contact buffer size changed");

		std::copy(m_bodies.begin(), m_bodies.end(), buffers.m_bodies.begin());
		std::copy(m_contacts.begin(), m_contacts.end(), buffers.m_contacts.begin());
	}

	void ResolveInteropRunner::ComputeCollisionTimes()
	{
		g = MakeConstants(m_config, m_dt, m_body_count, m_max_contacts);
		g_counters.assign(SpanOf(m_counters));
		g_bodies.assign(SpanOf(m_bodies));
		g_contacts.assign(SpanOf(m_contacts));
		g_contact_times.assign(SpanOf(m_contact_times));
		g_contact_order.assign(SpanOf(m_contact_order));

		hlsl::GpuEmulator emu(CSComputeCollisionTimes, CSComputeCollisionTimes_NumThreads);
		emu.Dispatch({ThreadGroupCount(m_counters[0].contact_count, ResolveThreadCount), 1, 1});

		m_bodies.assign(g_bodies.begin(), g_bodies.end());
		m_contacts.assign(g_contacts.begin(), g_contacts.end());
		m_contact_times.assign(g_contact_times.begin(), g_contact_times.end());
		m_contact_order.assign(g_contact_order.begin(), g_contact_order.end());
	}

	void ResolveInteropRunner::SortContacts()
	{
		std::stable_sort(m_contact_order.begin(), m_contact_order.end(), [this](uint32_t lhs, uint32_t rhs)
		{
			return m_contact_times[lhs] < m_contact_times[rhs];
		});
	}

	void ResolveInteropRunner::AssignColours()
	{
		g = MakeConstants(m_config, m_dt, m_body_count, m_max_contacts);
		g_counters.assign(SpanOf(m_counters));
		g_bodies.assign(SpanOf(m_bodies));
		g_colours.assign(SpanOf(m_colours));
		g_contacts.assign(SpanOf(m_contacts));
		g_contact_order.assign(SpanOf(m_contact_order));

		hlsl::GpuEmulator emu(CSAssignColours, CSAssignColours_NumThreads);
		emu.Dispatch({1, 1, 1});

		m_bodies.assign(g_bodies.begin(), g_bodies.end());
		m_colours.assign(g_colours.begin(), g_colours.end());
	}

	void ResolveInteropRunner::TemporalDrift(float dt)
	{
		g = MakeConstants(m_config, dt, m_body_count, m_max_contacts);
		g_bodies.assign(SpanOf(m_bodies));

		hlsl::GpuEmulator emu(CSTemporalDrift, CSTemporalDrift_NumThreads);
		emu.Dispatch({ThreadGroupCount(m_body_count, ResolveThreadCount), 1, 1});

		m_bodies.assign(g_bodies.begin(), g_bodies.end());
	}

	void ResolveInteropRunner::PositionSolve(float dt, int colour)
	{
		g = MakeConstants(m_config, dt, m_body_count, m_max_contacts, colour);
		g_counters.assign(SpanOf(m_counters));
		g_bodies.assign(SpanOf(m_bodies));
		g_colours.assign(SpanOf(m_colours));
		g_contacts.assign(SpanOf(m_contacts));
		g_contact_order.assign(SpanOf(m_contact_order));

		hlsl::GpuEmulator emu(CSPositionSolve, CSPositionSolve_NumThreads);
		emu.Dispatch({ThreadGroupCount(m_counters[0].contact_count, ResolveThreadCount), 1, 1});

		m_bodies.assign(g_bodies.begin(), g_bodies.end());
	}

	void ResolveInteropRunner::ResolveVelocity(float dt, int colour)
	{
		g = MakeConstants(m_config, dt, m_body_count, m_max_contacts, colour);
		g_counters.assign(SpanOf(m_counters));
		g_materials.assign(SpanOf(m_materials));
		g_bodies.assign(SpanOf(m_bodies));
		g_colours.assign(SpanOf(m_colours));
		g_contacts.assign(SpanOf(m_contacts));
		g_contact_order.assign(SpanOf(m_contact_order));

		hlsl::GpuEmulator emu(CSResolve, CSResolve_NumThreads);
		emu.Dispatch({ThreadGroupCount(m_counters[0].contact_count, ResolveThreadCount), 1, 1});

		m_bodies.assign(g_bodies.begin(), g_bodies.end());
	}

	std::span<uint32_t const> ResolveInteropRunner::Colours() const
	{
		return m_colours;
	}

	std::span<uint32_t const> ResolveInteropRunner::ContactOrder() const
	{
		return m_contact_order;
	}

	std::span<float const> ResolveInteropRunner::ContactTimes() const
	{
		return m_contact_times;
	}
}
