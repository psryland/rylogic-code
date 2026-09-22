//*********************************************
// Message Loop Tests
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "pr/common/unittests.h"
#include "pr/gui/message_loop.h"

namespace pr::unittests
{
	PRUnitTestClass(MessageLoopTests)
	{
		PRUnitTestMethod(QuitStopsScheduledCallbacks, Quick)
		{
			// Run the quit path on its owning thread so the thread-local message queue remains valid.
			auto callback_called = false;
			auto exit_code = std::optional<int>{};
			auto worker = std::thread([&]
				{
					// Request shutdown before scheduling work and require the queue to surface that exact exit code.
					auto loop = gui::MessageLoop{};

					// Create this thread's message queue before requesting its quit message.
					loop.Pump();
					loop.AddLoop(60.0, false, [&](double)
						{
							callback_called = true;
						});
					loop.RequestQuit(7);

					PR_EXPECT(loop.StepLoops() == gui::MessageLoop::duration_t::max());
					exit_code = loop.Pump();
				});
			worker.join();

			PR_EXPECT(!callback_called);
			PR_EXPECT(exit_code && *exit_code == 7);
		}

		PRUnitTestMethod(SubMillisecondWaitDoesNotPoll, Quick)
		{
			// Measure on an isolated message-queue thread so unrelated application windows cannot wake the deadline.
			auto elapsed = gui::MessageLoop::duration_t{};
			auto worker = std::thread([&]
				{
					// Create an empty thread message queue before measuring the timer-backed wait.
					auto loop = gui::MessageLoop{};
					loop.Pump();

					auto start = gui::MessageLoop::clock_t::now();
					loop.Pump(std::chrono::microseconds(500));
					elapsed = gui::MessageLoop::clock_t::now() - start;
				});
			worker.join();

			PR_EXPECT(elapsed >= std::chrono::microseconds(100));
			PR_EXPECT(elapsed < std::chrono::milliseconds(250));
		}

		PRUnitTestMethod(DeadlineWaitIsInterruptedByMessage, Quick)
		{
			// Publish the worker's queue identity before interrupting its long timer-backed wait.
			auto thread_id = std::atomic<DWORD>{};
			auto queue_ready = std::atomic<bool>{};
			auto elapsed = gui::MessageLoop::duration_t{};
			auto worker = std::thread([&]
				{
					// Create the queue before exposing the thread ID so PostThreadMessage cannot race queue creation.
					auto loop = gui::MessageLoop{};
					loop.Pump();
					thread_id.store(::GetCurrentThreadId(), std::memory_order_release);
					queue_ready.store(true, std::memory_order_release);

					auto start = gui::MessageLoop::clock_t::now();
					loop.Pump(std::chrono::seconds(1));
					elapsed = gui::MessageLoop::clock_t::now() - start;
				});
			while (!queue_ready.load(std::memory_order_acquire))
				std::this_thread::yield();

			PR_EXPECT(::PostThreadMessageW(thread_id.load(std::memory_order_acquire), WM_APP, 0, 0) != FALSE);
			worker.join();

			PR_EXPECT(elapsed < std::chrono::milliseconds(500));
		}
	};
}
