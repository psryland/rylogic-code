//*****************************************************************************************
// Message Loop
//  Copyright (c) Rylogic Ltd 2012
//*****************************************************************************************
#pragma once
#include <string>
#include <string_view>
#include <span>
#include <vector>
#include <exception>
#include <functional>
#include <algorithm>
#include <memory>
#include <chrono>
#include <thread>
#include <format>

#include <Windows.h>

namespace pr::gui
{
	// PR_CODE_SYNC_BEGIN(MessageLoop, source_of_truth)

	// An interface for types that need to handle messages from the message loop before TranslateMessage is called.
	// Typically these are dialog windows or windows with keyboard accelerators that need to call 'IsDialogMessage'
	// or 'TranslateAccelerator'
	struct IMessageFilter
	{
		virtual ~IMessageFilter() = default;

		// Implementers should return true to halt processing of the message.
		// Typically, if you're just observing messages as they go past, return false.
		// If you're a dialog return the result of IsDialogMessage()
		// If you're a window with accelerators, return the result of TranslateAccelerator()
		virtual bool TranslateMessage(MSG&)
		{
			return false;
		}
	};

	// Message loop that also manages and runs a priority queue of loops 
	struct MessageLoop :IMessageFilter
	{
		// Notes:
		//  - For event driven applications, don't add any loops. Then the message loop will just pump messages as normal.
		//  - Fixed step loops run at exactly the requested rate, accumulating time and catching up if behind.
		//  - Variable step loops run whenever possible, receiving the actual wall-clock elapsed time.
		//  - Fixed step loops have priority over variable step loops when both are due.

		using clock_t = std::chrono::steady_clock;
		using time_point_t = clock_t::time_point;
		using duration_t = clock_t::duration;
		using step_func_t = std::function<void(double)>; // receives elapsed time in seconds

		// The maximum number of fixed-step catch-up iterations before skipping ahead (death spiral protection)
		static constexpr int MaxCatchUpSteps = 4;

		// Compact history of frame rates 
		union FpsHistory
		{
			enum { Length = sizeof(uint64_t) };

			uint64_t u64;
			uint8_t ms_per_frame[sizeof(uint64_t)];
			void Add(double fps)
			{
				u64 <<= 8;
				ms_per_frame[0] = static_cast<uint8_t>(std::clamp(1000.0 / fps, 0.0, 255.0));
			}
			template <std::floating_point S> void Fps(std::span<S, Length> out_fps) const
			{
				// Output fps from newest to oldest
				for (size_t i = 0; i != sizeof(u64); ++i)
					out_fps[i] = ms_per_frame[i] > 0 ? S(1000.0 / ms_per_frame[i]) : S(0.0);
			}
		};

		// Controls for the message loop behaviour
		struct Config
		{
			// The number of window messages to process between calls to StepLoops.
			// Simulation apps would typically set this to 1.
			int msgs_per_loop = 1000;
		};

	private:

		// A loop represents a process that should be run at a given rate
		struct Loop
		{
			step_func_t m_step;       // The function to call to step the loop
			duration_t m_interval;    // The (minimum) time between steps
			time_point_t m_last_time; // The time this loop was last stepped
			time_point_t m_next_due;  // When this loop is next due to be stepped
			FpsHistory m_fps_history; // Ring buffer of recent FPS values
			bool m_variable;          // Variable step rate (true = run as fast as possible, false = fixed step)

			Loop(step_func_t step, duration_t interval, bool variable)
				: m_step(step)
				, m_interval(interval)
				, m_last_time()
				, m_next_due()
				, m_fps_history()
				, m_variable(variable)
			{}
		};

		using LoopCont = std::vector<Loop>;
		using Filters = std::vector<IMessageFilter*>;

		LoopCont m_loop;          // The loops to execute
		Filters m_filters;        // Message filters to process messages before TranslateMessage is called
		time_point_t m_clock0;    // The time when 'Run' was called
		bool m_quit_pending;      // Set when PostQuitMessage has been called, enables aggressive drain

	public:

		// Configurable parameters for the message loop
		Config m_config;

		MessageLoop()
			: m_loop()
			, m_filters()
			, m_clock0()
			, m_config()
			, m_quit_pending(false)
		{
			m_filters.push_back(this);
		}
		virtual ~MessageLoop() = default;

		// Access FPS history for a loop by index
		FpsHistory const& LoopFps(int loop_index) const
		{
			static FpsHistory null = {};
			if (loop_index < 0 || loop_index >= ssize(m_loop)) return null;
			return m_loop[loop_index].m_fps_history;
		}

		// Return the running time since 'Run()' was called (in seconds)
		double Clock() const
		{
			return std::chrono::duration<double>(clock_t::now() - m_clock0).count();
		}

		// Add a loop to be stepped by this message pump.
		// 'fps' is the target frame rate. 'variable' true means run as fast as possible (fps is minimum rate).
		void AddLoop(double fps, bool variable, step_func_t step)
		{
			auto interval = std::chrono::duration_cast<duration_t>(std::chrono::duration<double>(1.0 / fps));
			m_loop.emplace_back(std::move(step), interval, variable);
		}

		// Add/Remove an instance that needs to handle messages before TranslateMessage is called
		void AddMessageFilter(IMessageFilter& filter)
		{
			m_filters.insert(--std::end(m_filters), &filter);
		}
		void RemoveMessageFilter(IMessageFilter& filter)
		{
			m_filters.erase(std::remove(std::begin(m_filters), std::end(m_filters), &filter), std::end(m_filters));
		}

		// Run the thread message pump while maintaining the desired loop rates
		virtual int Run()
		{
			// Initialise clocks
			m_clock0 = clock_t::now();
			auto now = m_clock0;
			for (auto& loop : m_loop)
			{
				loop.m_last_time = now;
				loop.m_next_due = now;
			}

			// Run the message pump loop
			for (;;)
			{
				// Step any pending loops and get the time till the next loop is due
				auto timeout = StepLoops();

				// Pump any queued messages, waiting up to 'timeout' for new ones
				auto exit_code = Pump(timeout);
				if (exit_code)
					return *exit_code;
			}
		}

		// Pump messages. Returns null or an exit code if a WM_QUIT message was pumped
		std::optional<int> Pump(DWORD timeout_ms = 0)
		{
			MSG msg = {};

			// Wait for messages or until timeout (efficient idle, no busy-spin)
			::MsgWaitForMultipleObjects(0, nullptr, FALSE, timeout_ms, QS_ALLPOSTMESSAGE | QS_ALLINPUT | QS_ALLEVENTS);
			for (int msgs_per_loop = m_config.msgs_per_loop; msgs_per_loop-- != 0 && ::PeekMessageW(&msg, 0, 0, 0, PM_REMOVE); )
			{
				if (msg.message == WM_QUIT)
					return static_cast<int>(msg.wParam);

				HandleMessage(msg);
			}

			// WM_QUIT has the lowest priority in the Windows message queue. It is only
			// synthesized by PeekMessage/GetMessage when no other messages are pending.
			// If any source continuously generates messages (timers, paint, posted messages
			// from background threads), WM_QUIT can be starved indefinitely.
			// Check explicitly by draining all remaining messages in a second pass.
			if (m_quit_pending)
			{
				while (::PeekMessageW(&msg, nullptr, 0, 0, PM_REMOVE))
				{
					if (msg.message == WM_QUIT)
						return static_cast<int>(msg.wParam);
					// Discard non-quit messages during shutdown drain
				}
			}
			return std::nullopt;
		}

		// Post WM_QUIT to the thread message queue and enable aggressive message
		// draining so that WM_QUIT (which has the lowest priority) is not starved
		// by continuously generated messages from timers, paint, or background threads.
		void RequestQuit(int exit_code = 0)
		{
			m_quit_pending = true;
			::PostQuitMessage(exit_code);
		}

		// Call 'Step' on all loops that are pending. Returns the time in milliseconds until the next loop is due.
		DWORD StepLoops()
		{
			if (m_loop.empty())
				return INFINITE;

			auto now = clock_t::now();

			// Step fixed-rate loops first (they have priority), then variable-rate loops
			for (int pass = 0; pass != 2; ++pass)
			{
				for (auto& loop : m_loop)
				{
					// Pass 0 = fixed loops, Pass 1 = variable loops
					if (loop.m_variable != (pass == 1))
						continue;

					if (loop.m_variable)
					{
						// Variable step: run whenever due, with the actual elapsed wall-clock time
						if (now < loop.m_next_due)
							continue;

						auto elapsed = std::chrono::duration<double>(now - loop.m_last_time).count();
						loop.m_step(elapsed);
						loop.m_last_time = now;
						loop.m_next_due = now + loop.m_interval;
						loop.m_fps_history.Add(elapsed > 0 ? (1.0 / elapsed) : 0.0f);
					}
					else
					{
						// Fixed step: run at exactly the requested rate, catching up if behind
						auto stepped = false;
						for (int catch_up = 0; catch_up != MaxCatchUpSteps && loop.m_next_due <= now; ++catch_up)
						{
							auto dt = std::chrono::duration<double>(loop.m_interval).count();
							loop.m_step(dt);
							loop.m_next_due += loop.m_interval;
							stepped = true;
						}
						if (stepped)
						{
							auto elapsed = max(now - loop.m_last_time, loop.m_interval);
							loop.m_fps_history.Add(1.0 / std::chrono::duration<double>(elapsed).count());
						}

						// Death spiral protection: if still behind, skip ahead
						if (loop.m_next_due < now)
							loop.m_next_due = now;

						loop.m_last_time = now;
					}
				}
			}

			// Calculate time until the next loop is due
			auto next_due = m_loop[0].m_next_due;
			for (auto const& loop : m_loop)
			{
				if (loop.m_next_due < next_due)
					next_due = loop.m_next_due;
			}

			now = clock_t::now();
			if (next_due <= now)
				return 0;

			auto wait = std::chrono::duration_cast<std::chrono::milliseconds>(next_due - now).count();
			return static_cast<DWORD>(std::max(0LL, wait));
		}

	protected:

		// Pass the message to each filter. The last filter is this message loop which always handles the message.
		void HandleMessage(MSG& msg)
		{
			for (auto filter : m_filters)
				if (filter->TranslateMessage(msg))
					break;
		}

		// The message loop is always the last filter in the chain
		virtual bool TranslateMessage(MSG& msg) override
		{
			::TranslateMessage(&msg);
			::DispatchMessageW(&msg);
			return true;
		}
	};

	// PR_CODE_SYNC_END()
}
