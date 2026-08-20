#pragma once

#include <mutex>
#include <thread>
#include <condition_variable>
#include <chrono>
#include <stdexcept>
#include <cassert>

namespace pr::threads
{
	// Mix-in class for pause thread functionality
	// Note: this is not a suspend/resume mechanism. The class that inherits this functionality
	// needs to call 'TestPause()' from a main loop in order for the pause to occur.
	struct PauseThread
	{
		// Notes:
		// Why this isn't a reference counted pause:
		// Consider a reference counted pause implementation. We could have a
		// 'm_pause_requests' variable and track pause/unpause calls. However,
		// nested pause/unpause calls would have to be made from the same thread,
		// otherwise there would be a race condition on the thread's pause state.
		// Assuming the pause/unpause calls are all on the same thread, the Pause
		// function blocks until the thread has switched to the requested state.
		// Nested Pause() calls could be handled by testing for 'already paused'
		// but the meaning of nested Unpause() calls is hard to define. If code
		// calls 'Unpause()' it wants the thread to unpause, not maybe unpause
		// if the matching number of unpause to pause calls have been made.
		// Support for nesting is still useful however. The behaviour of nested
		// pause/unpause calls has to be: (ignoring timeouts)
		//   pause()   -> thread pauses, returns true
		//   pause()   -> returns true, already paused
		//   unpause() -> thread un-pauses, return true
		//   unpause() -> returns true, already unpaused

		mutable std::mutex      m_mutex_pause;
		std::condition_variable m_cv_pause;
		std::thread::id         m_pause_request_threadid;
		std::atomic_bool        m_pause_request;
		bool                    m_force_unpause;
		bool                    m_paused;

		PauseThread(bool init_pause_request = false)
			:m_mutex_pause()
			,m_cv_pause()
			,m_pause_request_threadid()
			,m_pause_request()
			,m_force_unpause(false)
			,m_paused(false)
		{
			m_pause_request_threadid = std::this_thread::get_id();
			m_pause_request = init_pause_request;
		}

		// Test if the associated thread is paused
		// This method should be called from an external thread context
		bool IsPaused() const
		{
			std::unique_lock<std::mutex> lock(m_mutex_pause);
			return m_paused;
		}

		// Request the thread to pause/unpause and wait for 'wait_ms' milliseconds for it to happen.
		// Returns true if the thread was paused/unpaused within the timeout period.
		// This method should be called from an external thread context
		bool Pause(bool pause, int wait_ms = ~0)
		{
			std::unique_lock<std::mutex> lock(m_mutex_pause);

			// If ForceUnpause() has been called then pausing is disabled.
			// Return true if 'unpause' was requested, or false if pause was requested but denied.
			if (m_force_unpause)
				return !pause;

			// If pause/unpause is requested but the thread is not yet paused/unpaused,
			// assert that the call is from the same thread. If the actual thread state
			// matches the requested state, we allow a different thread to call pause/unpause.
			if (m_pause_request != m_paused && m_pause_request_threadid != std::this_thread::get_id())
				throw std::runtime_error("Cross thread pause request made");

			// Record the request, and the thread that made it
			m_pause_request = pause;
			m_pause_request_threadid = std::this_thread::get_id();

			// Notify the pause condition variable whenever 'm_pause_request' possibly changes
			m_cv_pause.notify_all();

			// Tests the block condition. Returns true to unblock
			auto test_condition = [&]
			{
				return
					(m_paused == pause) || // The thread has paused/unpaused as requested
					(m_force_unpause);     // ForceUnpause() has been called
			};

			// Wait till paused/unpaused
			return wait_ms == ~0
				? (m_cv_pause.wait(lock, test_condition), true)
				: m_cv_pause.wait_for(lock, std::chrono::milliseconds(wait_ms), test_condition);
		}

		// Unpause the thread and force it to say unpaused.
		// Once 'ForceUnpause()' is called the thread can never be paused again.
		// Used for thread shutdown.
		void ForceUnpause()
		{
			std::unique_lock<std::mutex> lock(m_mutex_pause);
			m_force_unpause = true;
			m_pause_request = false;
			m_cv_pause.notify_all();
		}

		// Returns true if the thread has been signalled to pause
		// This method would typically be called from the thread context that is to be paused
		bool IsPauseRequested() const
		{
			std::unique_lock<std::mutex> lock(m_mutex_pause);
			return m_pause_request;
		}

		// Checks for pause requests and pauses the current thread until the requests == 0
		// Returns true so that this method can be used in conditionals
		// This method should be called from the thread context that is to be paused
		bool TestPaused()
		{
			// Test outside of the lock for performance reasons
			if (!m_pause_request)
				return true;

			std::unique_lock<std::mutex> lock(m_mutex_pause);

			// Retest if pause requested to prevent the race condition.
			if (!m_pause_request)
				return true;

			// Notify that the thread is paused
			m_paused = true;
			m_cv_pause.notify_all();

			// Block till unpaused
			m_cv_pause.wait(lock, [&]{ return !m_pause_request || m_force_unpause; });

			// Notify that the thread is unpaused
			m_paused = false;
			m_cv_pause.notify_all();
			return true;
		}
	};
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include <atomic>
#include "pr/threads/name_thread.h"
namespace pr::threads
{
	namespace unittests::threads
	{
		struct PauseTestWorker :private PauseThread
		{
			std::atomic_bool m_exit_signalled;
			std::atomic_int m_count;

			PauseTestWorker()
				:m_exit_signalled()
				,m_count()
			{}

			using PauseThread::Pause;
			using PauseThread::IsPaused;

			void Main()
			{
				pr::threads::SetCurrentThreadName("PauseTestWorker");
				for (;!m_exit_signalled && TestPaused(); ++m_count)
					std::this_thread::yield();
			}
		};
	}
	PRUnitTest(PauseThreadTests, Quick)
	{
		using namespace unittests::threads;
		
		PauseTestWorker worker;
		std::thread thread([&]
		{
			worker.Main();
		});

		PR_EXPECT(thread.joinable());
		PR_EXPECT(!worker.IsPaused());

		worker.Pause(true);

		PR_EXPECT(worker.IsPaused());

		int count = worker.m_count;
		std::this_thread::sleep_for(std::chrono::milliseconds(10));

		PR_EXPECT(worker.IsPaused());
		PR_EXPECT(worker.m_count == count);

		worker.Pause(false);

		PR_EXPECT(!worker.IsPaused());

		for (;worker.m_count == count; std::this_thread::yield()) {}

		PR_EXPECT(worker.m_count > count);

		worker.m_exit_signalled = true;
		thread.join();
	}
}
#endif
