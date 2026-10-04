//***********************************************************
// Profiler
//  Copyright (C) Rylogic Ltd 2007
//***********************************************************
#pragma once
#include <string>
#include <string_view>
#include <chrono>
#include <format>
#include <windows.h>

namespace pr::profile
{
	// Simple object for timing a block of code and writing the output to the Output window
	struct TimeThis
	{
		using clock_t = std::chrono::high_resolution_clock;
		using duration_t = clock_t::duration;
		using time_point_t = clock_t::time_point;

		std::string m_message;
		time_point_t m_start;
		duration_t m_time;

		TimeThis()
			:m_message()
			,m_start()
			,m_time()
		{}
		explicit TimeThis(std::string_view message)
			:TimeThis()
		{
			// Note: you don't need to call Start/Stop if you use this as an RAII object.
			Start(message);
		}
		~TimeThis()
		{
			Stop();
		}
		TimeThis& Start(std::string_view message)
		{
			m_message = message;
			m_time = duration_t::zero();
			m_start = clock_t::now();
			return *this;
		}
		TimeThis& Stop()
		{
			m_time = clock_t::now() - m_start;
			return *this;
		}
		TimeThis const& Display(bool progress = false) const
		{
			// The can display output before calling Stop()
			auto lineend = progress ? '\r' : '\n';
			auto time = m_time != duration_t::zero() ? m_time : clock_t::now() - m_start;
			OutputDebugStringA(std::format("{} {}{}", m_message, time, lineend).c_str());
			return *this;
		}
	};
}
