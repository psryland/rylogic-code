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
			m_shutdown = true;
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
			++m_tasks_pending;
			m_tasks.push(std::move(task));
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

				// Signal task completed
				--m_tasks_pending;
				assert(m_tasks_pending >= 0);
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
	PRUnitTest(ThreadPoolTests)
	{
#if 0 // There is a race condition in here somewhere. This test sometimes never ends
		ThreadPool pool;
		std::atomic_int count = 0;

		for (int i = 0; i != 10; ++i)
		{
			pool.QueueTask([&]
			{
				++count;
				std::this_thread::sleep_for(std::chrono::milliseconds(100));
				++count;
			});
		}

		pool.WaitAll();
		PR_EXPECT(count == 20);

	//	auto result = pool.QueueTaskR([] { std::this_thread::sleep_for(std::chrono::milliseconds(100)); return 42; });
	//	PR_EXPECT(result.get() == 42);
#endif
	}

	// Regression test for a bug where a completed task could be re-run: the worker loop kept a
	// single 'task' variable across iterations, and 'concurrent_queue::try_pop' leaves its output
	// unchanged when the queue is empty. That meant a failed pop after a task finished left the
	// just-run task sitting in the loop variable, which then bypassed the "do we have work" guard
	// and got executed again instead of the next queued task.
	PRUnitTest(ThreadPoolStaleTaskReplayTest)
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
}
#endif
