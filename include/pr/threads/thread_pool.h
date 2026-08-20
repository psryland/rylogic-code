//*********************************************************************
// Concurrent Queue
//  Copyright (c) Rylogic Ltd 2011
//*********************************************************************
// Thread safe producer/consumer queue
// See unit tests for usage
#pragma once
#include <concepts>
#include <thread>
#include <atomic>
#include <mutex>
#include <future>
#include <type_traits>
#include <condition_variable>
#include <concurrent_queue.h>

#include "pr/threads/name_thread.h"

namespace pr::threads
{
	class ThreadPool
	{
		using task_queue_t = concurrency::concurrent_queue<std::function<void()>>;
		using thread_cont_t = std::vector<std::thread>;
			
		thread_cont_t m_threads;
		task_queue_t m_tasks;
		std::condition_variable m_cv_task_added;
		std::condition_variable m_cv_task_complete;
		std::atomic_int m_tasks_pending;
		mutable std::mutex m_mutex_tasks;
		std::atomic_bool m_shutdown;

	public:
		ThreadPool(const uint32_t num_threads = std::thread::hardware_concurrency()) // Max # of threads the system supports
			:m_threads()
			,m_tasks()
			,m_cv_task_added()
			,m_cv_task_complete()
			,m_tasks_pending()
			,m_mutex_tasks()
			,m_shutdown()
		{
			for (auto i = 0U; i != num_threads; ++i)
				m_threads.push_back(std::thread(&ThreadPool::ThreadMain, this));
		}
		~ThreadPool()
		{
			// 'm_shutdown' is part of the predicate that worker threads wait on via 'm_cv_task_added'
			// (see 'ThreadMain'). The predicate must only change while 'm_mutex_tasks' is held: a
			// worker can otherwise observe the old value of 'm_shutdown' with the lock held, and then
			// have this thread flip it and call 'notify_all' before the worker's 'wait' call has
			// actually unlocked and registered with the condition variable. That notification would
			// then be missed, leaving the worker blocked forever and this destructor hung on 'join'.
			{
				std::lock_guard<std::mutex> lock(m_mutex_tasks);
				m_shutdown = true;
			}
			m_cv_task_added.notify_all();

			for (auto& thread : m_threads)
			{
				if (thread.joinable())
					thread.join();
			}
		}

		// The number of tasks currently queued (roughly)
		size_t TaskCountUnsafe() const
		{
			return m_tasks.unsafe_size();
		}

		// Queue a task with no return value
		void QueueTask(std::function<void()>&& task)
		{
			// 'm_tasks_pending' and the queue contents are the predicate state for 'm_cv_task_complete'
			// and 'm_cv_task_added' respectively (see 'WaitAll' and 'ThreadMain'). Both must change while
			// 'm_mutex_tasks' is held, otherwise a waiter could check the predicate (false, lock held),
			// then this thread could update the state and call 'notify' before the waiter's 'wait' call
			// unlocks and registers with the condition variable - a missed wakeup that leaves the waiter
			// blocked until some unrelated notification happens to come along, or forever.
			{
				std::lock_guard<std::mutex> lock(m_mutex_tasks);
				++m_tasks_pending;
				m_tasks.push(std::move(task));
			}
			m_cv_task_added.notify_one();
		}

		// Wait for all queued tasks to complete
		void WaitAll()
		{
			std::unique_lock<std::mutex> lock(m_mutex_tasks);
			m_cv_task_complete.wait(lock, [&] { return m_tasks_pending == 0; });
		}

	private:

		void ThreadMain()
		{
			SetCurrentThreadName("ThreadPool Worker");

			for (;;)
			{
				// Declared fresh each iteration (rather than once outside the loop) so that a
				// failed 'try_pop' below reliably leaves 'task' empty. 'concurrent_queue::try_pop'
				// only writes its output on success, so a shared, reused 'task' would otherwise
				// retain the previous iteration's callable, letting the '!task' guards below
				// mistake it for a genuinely pending task and re-run it instead of popping the
				// next queued task.
				std::function<void()> task;

				// If there are no tasks, wait for a signal
				if (!m_tasks.try_pop(task))
				{
					std::unique_lock<std::mutex> lock(m_mutex_tasks);
					
					// Wait for tasks or shutdown. Don't pop in predicate to avoid losing tasks on shutdown.
					m_cv_task_added.wait(lock, [&] { return !m_tasks.empty() || m_shutdown; });
					
					// Check shutdown first, but only exit if no tasks remain
					if (m_shutdown && !m_tasks.try_pop(task))
						return;
					
					// If we didn't pop above (not shutdown), try to pop now
					if (!task && !m_tasks.try_pop(task))
						continue;
				}

				// Execute the task
				task();

				// Signal task completed. The decrement must happen while 'm_mutex_tasks' is held for
				// the same reason as in 'QueueTask': 'm_tasks_pending' is the predicate state for
				// 'm_cv_task_complete' (see 'WaitAll'), and changing it without the lock held would
				// allow a waiter's predicate check to race with this update, missing the notification.
				{
					std::lock_guard<std::mutex> lock(m_mutex_tasks);
					--m_tasks_pending;
					assert(m_tasks_pending >= 0);
				}
				m_cv_task_complete.notify_all();
			}
		}
	};
}

#if PR_UNITTESTS
#include <chrono>
#include "pr/common/unittests.h"
namespace pr::threads
{
	PRUnitTest(ThreadPoolTests, Stress)
	{
		using namespace std::chrono_literals;

		// 'pool' is heap-allocated via 'shared_ptr' and shared with the detached thread below so
		// that, if a regression reintroduces the missed-wakeup bug and 'WaitAll' hangs, the pool
		// object stays alive for that thread instead of being destroyed while still in use.
		auto pool = std::make_shared<ThreadPool>();
		auto count = std::make_shared<std::atomic_int>(0);

		for (int i = 0; i != 10; ++i)
		{
			pool->QueueTask([count]
			{
				++*count;
				std::this_thread::sleep_for(std::chrono::milliseconds(100));
				++*count;
			});
		}

		// Run 'WaitAll' on a detached thread and signal completion via a promise. Unlike a
		// 'std::async' future, a 'std::promise'-backed future doesn't block in its destructor, so a
		// timed-out wait here reports a test failure without risking a further hang (see
		// 'ThreadPoolMissedWakeupStressTest' for the race this guards against).
		auto done = std::make_shared<std::promise<void>>();
		auto done_future = done->get_future();
		std::thread([pool, done] { pool->WaitAll(); done->set_value(); }).detach();

		PR_EXPECT(done_future.wait_for(5s) == std::future_status::ready);
		PR_EXPECT(*count == 20);

	//	auto result = pool.QueueTaskR([] { std::this_thread::sleep_for(std::chrono::milliseconds(100)); return 42; });
	//	PR_EXPECT(result.get() == 42);
	}

	// Regression test for a bug where a completed task could be re-run: the worker loop kept a
	// single 'task' variable across iterations, and 'concurrent_queue::try_pop' leaves its output
	// unchanged when the queue is empty. That meant a failed pop after a task finished left the
	// just-run task sitting in the loop variable, which then bypassed the "do we have work" guard
	// and got executed again instead of the next queued task.
	PRUnitTest(ThreadPoolStaleTaskReplayTest, Quick)
	{
		// A single worker thread makes the interleaving deterministic: there's exactly one thread
		// that can dequeue/execute tasks, so the only variable is timing between task 'A' finishing
		// and task 'B' being queued from this thread.
		ThreadPool pool(1);
		std::atomic_int count_a = 0;
		std::atomic_int count_b = 0;
		std::atomic_bool a_ran = false;

		pool.QueueTask([&]
		{
			++count_a;
			a_ran = true;
		});

		// Wait for 'A' to have run at least once.
		while (!a_ran)
			std::this_thread::yield();

		// Widen the race window: give the worker thread a chance to loop back to the (now empty)
		// queue and hit the failed 'try_pop' before 'B' is queued. This is not required for
		// correctness after the fix (the outcome is deterministic regardless of timing), it only
		// increases the chance of catching a regression that reintroduces the stale-task bug.
		std::this_thread::sleep_for(std::chrono::milliseconds(20));

		pool.QueueTask([&]
		{
			++count_b;
		});

		pool.WaitAll();
		PR_EXPECT(count_a == 1);
		PR_EXPECT(count_b == 1);
	}

	// Regression test for a missed-wakeup race: the state that condition-variable predicates depend
	// on ('m_tasks_pending' and the task queue contents) must only change while 'm_mutex_tasks' is
	// held. Otherwise, a waiter in 'WaitAll' can check its predicate (false, lock held), and then have
	// 'QueueTask' or the worker's completion update race ahead and notify before the waiter's 'wait'
	// call has actually unlocked and registered with the condition variable - a missed notification
	// that leaves 'WaitAll' blocked forever. Queuing tasks from several producer threads concurrently
	// with workers draining the queue, repeated many times, makes that race window likely to be hit
	// if it still exists.
	PRUnitTest(ThreadPoolMissedWakeupStressTest, Stress)
	{
		using namespace std::chrono_literals;

		for (int rep = 0; rep != 200; ++rep)
		{
			// Heap-allocated and shared with the detached 'WaitAll' thread below so that, if this
			// regresses and 'WaitAll' hangs, the pool stays alive for that thread (and is
			// intentionally never destroyed) instead of the test blocking to tear it down.
			auto pool = std::make_shared<ThreadPool>(4);
			auto count = std::make_shared<std::atomic_int>(0);

			std::vector<std::thread> producers;
			for (int p = 0; p != 4; ++p)
			{
				producers.push_back(std::thread([pool, count]
				{
					for (int i = 0; i != 25; ++i)
						pool->QueueTask([count] { ++*count; });
				}));
			}
			for (auto& producer : producers)
				producer.join();

			// See 'ThreadPoolTests' for why a promise-backed future (rather than 'std::async') is
			// used here: its destructor doesn't block, so a timed-out wait can't itself hang.
			auto done = std::make_shared<std::promise<void>>();
			auto done_future = done->get_future();
			std::thread([pool, done] { pool->WaitAll(); done->set_value(); }).detach();

			auto status = done_future.wait_for(5s);
			PR_EXPECT(status == std::future_status::ready);
			if (status != std::future_status::ready)
				break; // Regression detected: stop repeating rather than leak an unbounded number of stuck threads.

			PR_EXPECT(*count == 100);
		}
	}

	// Regression test for a missed-wakeup race in pool shutdown: 'm_shutdown' must only change while
	// 'm_mutex_tasks' is held, otherwise a worker parked in 'wait' on 'm_cv_task_added' can miss the
	// shutdown notification and never return, leaving the destructor's 'join' calls blocked forever.
	// Constructing and immediately destroying pools with idle workers (no tasks queued, so workers are
	// parked in 'wait' when shutdown happens), repeated many times, makes that race window likely to
	// be hit if it still exists.
	PRUnitTest(ThreadPoolShutdownStressTest, Stress)
	{
		using namespace std::chrono_literals;

		for (int rep = 0; rep != 200; ++rep)
		{
			// The pool is constructed and destroyed entirely within the detached thread, so a hung
			// destructor (were the bug to regress) leaks only that thread rather than risking
			// use of a pool object whose enclosing test scope has already returned. 'pool' is
			// scoped to end before 'set_value' is called, so the promise is only fulfilled once
			// the destructor (and its worker 'join' calls) has actually completed - otherwise a
			// hung destructor would go undetected, since 'set_value' would already have fired.
			auto done = std::make_shared<std::promise<void>>();
			auto done_future = done->get_future();
			std::thread([done]
			{
				{
					ThreadPool pool(4);
					std::this_thread::sleep_for(1ms); // Give workers a chance to reach 'wait' before destruction.
				}
				done->set_value();
			}).detach();

			auto status = done_future.wait_for(5s);
			PR_EXPECT(status == std::future_status::ready);
			if (status != std::future_status::ready)
				break; // Regression detected: stop repeating rather than leak an unbounded number of stuck threads.
		}
	}
}
#endif
