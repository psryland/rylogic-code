//************************************************************************
// Task Graph
//  Copyright (c) Rylogic Ltd 2025
//************************************************************************
// A coroutine-based task graph for running interdependent tasks across
// multiple threads. Tasks signal completion (broadcast) and wait for
// other tasks' signals using co_await.
//
// Usage:
//   enum class TaskId : int { AI, Physics, Render, Count };
//
//   pr::task_graph::Graph<TaskId> graph;
//
//   graph.Add(TaskId::AI, [&](auto ctx) -> pr::task_graph::Task {
//       DoPathfinding();
//       co_return; // automatically signals TaskId::AI
//   });
//
//   graph.Add(TaskId::Physics, [&](auto ctx) -> pr::task_graph::Task {
//       DoBroadphase();
//       co_await ctx.Wait(TaskId::AI);
//       DoNarrowphase();
//       co_return; // automatically signals TaskId::Physics
//   });
//
//   graph.Run();   // blocks until all tasks complete
//   graph.Reset(); // ready for next frame
//
#pragma once
#include <cstdint>
#include <atomic>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <coroutine>
#include <deque>
#include <vector>
#include <functional>
#include <exception>
#include <cassert>
#include <type_traits>
#include <memory>
#include <format>

namespace pr::task_graph
{
	// Forward declarations
	struct Task;
	template <typename TaskId> struct Graph;
	template <typename TaskId> struct Context;

	// ─ WorkerPool ─────────────────────────────────────────────────────
	struct WorkerPool
	{
		// Notes:
		//  - A simple thread pool that resumes coroutine handles.
		WorkerPool(int thread_count = 0)
			: m_queue()
			, m_mutex()
			, m_cv_work()
			, m_cv_idle()
			, m_threads()
			, m_active()
			, m_shutdown(false)
		{
			if (thread_count <= 0)
				thread_count = static_cast<int>(std::thread::hardware_concurrency());
			if (thread_count <= 0)
				thread_count = 1;

			m_threads.reserve(thread_count);
			for (int i = 0; i != thread_count; ++i)
			{
				m_threads.emplace_back([this]
				{
					for (;;)
					{
						std::coroutine_handle<> job;
						{
							auto lock = std::unique_lock(m_mutex);
							m_cv_work.wait(lock, [this] { return !m_queue.empty() || m_shutdown; });
							if (m_shutdown && m_queue.empty())
								break;

							job = m_queue.front();
							m_queue.pop_front();
							++m_active;
						}
						job.resume();
						{
							auto lock = std::unique_lock(m_mutex);
							--m_active;
							if (m_queue.empty() && m_active == 0)
								m_cv_idle.notify_all();
						}
					}
				});
			}
		}
		WorkerPool(WorkerPool const&) = delete;
		WorkerPool& operator =(WorkerPool const&) = delete;
		~WorkerPool()
		{
			{
				auto lock = std::unique_lock(m_mutex);
				m_shutdown = true;
			}
			m_cv_work.notify_all();
			for (auto& t : m_threads)
			{
				if (t.joinable())
					t.join();
			}
		}

		// Enqueue a coroutine handle to be resumed by a worker thread
		void Enqueue(std::coroutine_handle<> handle)
		{
			{
				auto lock = std::unique_lock(m_mutex);
				m_queue.push_back(handle);
			}
			m_cv_work.notify_one();
		}

		// Wait until every queued or running coroutine resume has returned to the worker pool.
		void WaitIdle()
		{
			auto lock = std::unique_lock(m_mutex);
			m_cv_idle.wait(lock, [this] { return m_queue.empty() && m_active == 0; });
		}

		// Get the number of worker threads
		int ThreadCount() const
		{
			return static_cast<int>(m_threads.size());
		}

	private:

		std::deque<std::coroutine_handle<>> m_queue;
		std::mutex m_mutex;
		std::condition_variable m_cv_work;
		std::condition_variable m_cv_idle;
		std::vector<std::thread> m_threads;
		int m_active;
		bool m_shutdown;
	};

	// ── SignalState ────────────────────────────────────────────────────
	struct SignalState
	{
		// Notes:
		//  - Per-signal atomic state: tracks whether a signal has been raised and maintains a list of coroutines waiting for it.

		// Intrusive node for the waiter list
		struct Waiter
		{
			std::coroutine_handle<> handle;
			Waiter* next;
		};

		SignalState()
			: m_mutex()
			, m_signaled(false)
			, m_waiters(nullptr)
		{}
		SignalState(SignalState const&) = delete;
		SignalState& operator =(SignalState const&) = delete;

		// Check if the signal has been raised
		bool IsSignaled() const
		{
			return m_signaled.load(std::memory_order_acquire);
		}

		// Raise the signal and enqueue all waiters to the pool.
		// Returns the number of waiters resumed.
		int Raise(WorkerPool& pool)
		{
			// Set the signal flag first so any future Wait() sees it immediately
			m_signaled.store(true, std::memory_order_release);

			// Steal the waiter list
			Waiter* list = nullptr;
			{
				auto lock = std::unique_lock(m_mutex);
				list = m_waiters;
				m_waiters = nullptr;
			}

			// Enqueue all waiters to the thread pool
			int count = 0;
			for (auto* w = list; w != nullptr; )
			{
				auto* next = w->next;
				pool.Enqueue(w->handle);
				delete w;
				++count;
				w = next;
			}
			return count;
		}

		// Add a waiter. Returns true if the waiter was added (signal not yet raised),
		// or false if the signal is already raised (caller should not suspend).
		bool AddWaiter(std::coroutine_handle<> handle)
		{
			auto lock = std::unique_lock(m_mutex);

			// Double-check under lock
			if (m_signaled.load(std::memory_order_acquire))
				return false;

			auto* node = new Waiter{ handle, m_waiters };
			m_waiters = node;
			return true;
		}

		// Reset for reuse
		void Reset()
		{
			m_signaled.store(false, std::memory_order_release);

			// Any remaining waiters are stale — clean them up
			auto lock = std::unique_lock(m_mutex);
			for (auto* w = m_waiters; w != nullptr; )
			{
				auto* next = w->next;
				delete w;
				w = next;
			}
			m_waiters = nullptr;
		}

	private:

		std::mutex m_mutex;
		std::atomic<bool> m_signaled;
		Waiter* m_waiters;
	};

	// ── Task ───────────────────────────────────────────────────────────
	struct Task
	{
		// Notes:
		//  - Coroutine return type for task graph tasks.

		struct promise_type
		{
			// The signal to broadcast when this task completes (set by Graph::Add)
			std::function<void(promise_type&)> m_on_complete;

			// Exception captured from the coroutine
			std::exception_ptr m_exception;

			// Extends the lifetime of the lambda closure passed to Graph::Add().
			// Lambda coroutines access captures through the closure's implicit 'this' pointer,
			// which is stored in the coroutine frame. Since initial_suspend is suspend_always,
			// the coroutine body doesn't run during Add(), so the lambda must be kept alive
			// until the coroutine completes to prevent dangling access to captures.
			std::shared_ptr<void> m_keep_alive;

			// Get the Task object from the promise
			Task get_return_object()
			{
				return Task{ std::coroutine_handle<promise_type>::from_promise(*this) };
			}

			// Tasks start suspended — Graph::Run() resumes them
			std::suspend_always initial_suspend() noexcept
			{
				return {};
			}

			// On final suspend, collect exceptions and broadcast the completion signal
			auto final_suspend() noexcept
			{
				struct FinalAwaiter
				{
					bool await_ready() noexcept { return false; }
					void await_suspend(std::coroutine_handle<promise_type> h) noexcept
					{
						auto& promise = h.promise();
						if (promise.m_on_complete)
							promise.m_on_complete(promise);
					}
					void await_resume() noexcept {}
				};
				return FinalAwaiter{};
			}

			// No return value from the coroutine
			void return_void() {}

			// Capture exceptions to propagate to Graph::Run()
			void unhandled_exception()
			{
				m_exception = std::current_exception();
			}
		};

		using handle_type = std::coroutine_handle<promise_type>;

		Task(handle_type h)
			: m_handle(h)
		{}
		Task(Task&& rhs) noexcept
			: m_handle(rhs.m_handle)
		{
			rhs.m_handle = nullptr;
		}
		Task(Task const&) = delete;
		Task& operator =(Task&& rhs) noexcept
		{
			if (this != &rhs)
			{
				if (m_handle) m_handle.destroy();
				m_handle = rhs.m_handle;
				rhs.m_handle = nullptr;
			}
			return *this;
		}
		Task& operator =(Task const&) = delete;
		~Task()
		{
			if (m_handle)
				m_handle.destroy();
		}

		handle_type m_handle;
	};

	// ── Context ────────────────────────────────────────────────────────
	template <typename TaskId>
	struct Context
	{
		// Notes:
		//  - Passed to each task coroutine. Provides Wait() and Signal() methods.

		Graph<TaskId>* m_graph;

		explicit Context(Graph<TaskId>* graph)
			: m_graph(graph)
		{}

		// co_await ctx.Wait(id) — suspend until the signal is raised
		auto Wait(TaskId id)
		{
			struct WaitAwaiter
			{
				Graph<TaskId>* graph;
				TaskId id;

				bool await_ready() const noexcept
				{
					return graph->GetSignal(id).IsSignaled();
				}
				bool await_suspend(std::coroutine_handle<> h)
				{
					// Try to add as a waiter. Returns false if already signaled.
					return graph->GetSignal(id).AddWaiter(h);
				}
				void await_resume() const noexcept {}
			};
			return WaitAwaiter{ m_graph, id };
		}

		// co_await ctx.Signal(id) — broadcast a signal mid-task and continue
		auto Signal(TaskId id)
		{
			struct SignalAwaiter
			{
				Graph<TaskId>* graph;
				TaskId id;

				bool await_ready() const noexcept { return false; }
				void await_suspend(std::coroutine_handle<> h)
				{
					graph->GetSignal(id).Raise(graph->Pool());
					graph->Pool().Enqueue(h); // re-enqueue self to continue
				}
				void await_resume() const noexcept {}
			};
			return SignalAwaiter{ m_graph, id };
		}
	};

	// ── Graph ──────────────────────────────────────────────────────────
	template <typename TaskId>
	struct Graph
	{
		// Notes:
		//  - The main task graph. Owns the thread pool, signal state, and tasks.

		static_assert(std::is_enum_v<TaskId> || std::is_integral_v<TaskId>, "TaskId must be an enum or integral type");

		// Construct with optional thread count and signal count.
		// 'max_signals' defaults to TaskId::Count if it exists, otherwise must be provided.
		template <typename TId = TaskId>
		explicit Graph(int thread_count = 0, int max_signals = static_cast<int>(TId::Count))
			: m_signals(max_signals)
			, m_tasks()
			, m_pending(0)
			, m_mutex()
			, m_cv_done()
			, m_exceptions()
			, m_started()
			, m_pool(thread_count)
		{}
		Graph(Graph const&) = delete;
		Graph& operator =(Graph const&) = delete;

		// Add a task. The callable receives a Context<TaskId>& and must return Task.
		template <typename Fn>
		void Add(TaskId id, Fn&& fn)
		{
			if (m_started)
				throw std::runtime_error("Graph::Add called after Graph::Start");

			// Pin the callable on the heap so it outlives the coroutine.
			auto pinned = std::make_shared<std::decay_t<Fn>>(std::forward<Fn>(fn));

			auto ctx = Context<TaskId>(this);
			auto task = (*pinned)(ctx);

			// Keep the lambda alive in the promise until the coroutine frame is destroyed
			task.m_handle.promise().m_keep_alive = pinned;

			// Set the completion callback: collect exceptions, signal the task's ID, and decrement pending count
			task.m_handle.promise().m_on_complete = [this, id](Task::promise_type& promise)
			{
				if (promise.m_exception)
				{
					auto lock = std::lock_guard(m_mutex);
					m_exceptions.push_back(promise.m_exception);
				}
				m_signals[static_cast<int>(id)].Raise(m_pool);
				DecrementPending();
			};

			m_tasks.push_back(std::move(task));
		}

		// Start running all currently-added tasks without waiting for them to complete.
		void Start()
		{
			if (m_started)
				throw std::runtime_error("Graph::Start called while the graph is already running");

			m_started = true;
			m_pending.store(static_cast<int>(m_tasks.size()), std::memory_order_release);

			// Schedule all tasks onto the worker pool
			for (auto& task : m_tasks)
				m_pool.Enqueue(task.m_handle);
		}

		// Wait for all started tasks to complete and return the first captured exception, or null on success.
		[[nodiscard]] std::exception_ptr Wait() noexcept
		{
			try
			{
				if (!m_started)
					throw std::runtime_error("Graph::Wait called before Graph::Start");

				// Block until all tasks have completed
				{
					auto lock = std::unique_lock(m_mutex);
					m_cv_done.wait(lock, [this] { return m_pending.load(std::memory_order_acquire) == 0; });
				}

				// Ensure the worker thread has returned from the final coroutine resume before Reset() can destroy completed coroutine frames.
				m_pool.WaitIdle();

				// Return the first captured exception after draining the graph so callers can cleanly reset or signal follow-on work before rethrowing.
				if (!m_exceptions.empty())
				{
					auto ex = m_exceptions.front();
					m_exceptions.clear();
					return ex;
				}

				return {};
			}
			catch (...)
			{
				return std::current_exception();
			}
		}

		// Run all tasks to completion. Blocks until done.
		void Run()
		{
			if (m_tasks.empty())
				return;

			Start();
			if (auto ex = Wait())
				std::rethrow_exception(ex);
		}

		// Raise a graph signal from outside a task.
		void Signal(TaskId id)
		{
			GetSignal(id).Raise(m_pool);
		}

		// Reset the graph for reuse (e.g. next frame).
		// Clears signal state and removes completed tasks.
		void Reset()
		{
			if (m_started && m_pending.load(std::memory_order_acquire) != 0)
				throw std::runtime_error("Graph::Reset called while tasks are pending");

			m_pool.WaitIdle();

			// Destroy completed task coroutines
			m_tasks.clear();

			// Reset all signals
			for (auto& sig : m_signals)
				sig.Reset();

			m_pending.store(0, std::memory_order_release);
			m_exceptions.clear();
			m_started = false;
		}

		// Access the signal state for a given ID
		SignalState& GetSignal(TaskId id)
		{
			auto idx = static_cast<int>(id);
			assert(idx >= 0 && idx < static_cast<int>(m_signals.size()));
			return m_signals[idx];
		}

		// Access the thread pool
		WorkerPool& Pool()
		{
			return m_pool;
		}

		// Number of worker threads
		int ThreadCount() const
		{
			return m_pool.ThreadCount();
		}

	private:

		// Decrement the pending task count and notify if all tasks are done
		void DecrementPending()
		{
			auto prev = m_pending.fetch_sub(1, std::memory_order_acq_rel);
			if (prev == 1)
			{
				auto lock = std::unique_lock(m_mutex);
				m_cv_done.notify_all();
			}
		}

		std::vector<SignalState> m_signals;
		std::vector<Task> m_tasks;
		std::atomic<int> m_pending;
		std::mutex m_mutex;
		std::condition_variable m_cv_done;
		std::vector<std::exception_ptr> m_exceptions;
		bool m_started;

		// Declared last so it is destroyed first. Members are destroyed in reverse declaration order, and
		// a task's completion callback (see Add()) can touch every member above via 'this' from a worker
		// thread. Tearing any of them down while a worker thread might still resume a task coroutine
		// (e.g. if the graph is destroyed without a preceding Wait()/Reset() to quiescence) is a
		// use-after-free: coroutine_handle::destroy() requires the coroutine to be suspended, not actively
		// executing. Destroying the pool first shuts down and joins every worker thread, so no thread can
		// still be resuming a task by the time the containers above are torn down.
		WorkerPool m_pool;
	};
}

// ── Unit Tests ─────────────────────────────────────────────────────
#if PR_UNITTESTS
#include "pr/common/unittests.h"

namespace pr::task_graph::unittests
{
	enum class TestId : int
	{
		A, B, C, D, E, F, G, H,
		PhaseOne, // for mid-task signal test
		External, // for external signal tests
		Count,
	};

	PRUnitTest(TaskGraphBasicParallel, Quick)
	{
		// Independent tasks run in parallel without dependencies
		std::atomic<int> sum = 0;

		Graph<TestId> graph(4);
		graph.Add(TestId::A, [&](auto&) -> Task { sum += 1; co_return; });
		graph.Add(TestId::B, [&](auto&) -> Task { sum += 2; co_return; });
		graph.Add(TestId::C, [&](auto&) -> Task { sum += 4; co_return; });
		graph.Add(TestId::D, [&](auto&) -> Task { sum += 8; co_return; });
		graph.Run();

		PR_EXPECT(sum == 15);
	}

	PRUnitTest(TaskGraphDependency, Quick)
	{
		// Task A depends on Task B — A must see B's result
		std::atomic<int> order = 0;
		std::atomic<int> a_saw = 0;

		Graph<TestId> graph(2);

		graph.Add(TestId::B, [&](auto&) -> Task {
			std::this_thread::sleep_for(std::chrono::milliseconds(50));
			order.store(1, std::memory_order_release);
			co_return;
		});

		graph.Add(TestId::A, [&](auto ctx) -> Task {
			co_await ctx.Wait(TestId::B);
			a_saw.store(order.load(std::memory_order_acquire));
			co_return;
		});

		graph.Run();

		PR_EXPECT(a_saw == 1);
	}

	PRUnitTest(TaskGraphFanOut, Quick)
	{
		// Multiple tasks wait on the same signal (broadcast)
		std::atomic<int> count = 0;

		Graph<TestId> graph(4);

		graph.Add(TestId::A, [&](auto&) -> Task {
			std::this_thread::sleep_for(std::chrono::milliseconds(30));
			co_return;
		});

		// B, C, D all wait for A
		graph.Add(TestId::B, [&](auto ctx) -> Task { co_await ctx.Wait(TestId::A); count += 1; co_return; });
		graph.Add(TestId::C, [&](auto ctx) -> Task { co_await ctx.Wait(TestId::A); count += 1; co_return; });
		graph.Add(TestId::D, [&](auto ctx) -> Task { co_await ctx.Wait(TestId::A); count += 1; co_return; });

		graph.Run();

		PR_EXPECT(count == 3);
	}

	PRUnitTest(TaskGraphFanIn, Quick)
	{
		// One task waits on multiple signals
		std::atomic<int> sum = 0;

		Graph<TestId> graph(4);

		graph.Add(TestId::A, [&](auto&) -> Task { sum += 1; co_return; });
		graph.Add(TestId::B, [&](auto&) -> Task { sum += 2; co_return; });
		graph.Add(TestId::C, [&](auto&) -> Task { sum += 4; co_return; });

		graph.Add(TestId::D, [&](auto ctx) -> Task {
			co_await ctx.Wait(TestId::A);
			co_await ctx.Wait(TestId::B);
			co_await ctx.Wait(TestId::C);
			PR_EXPECT(sum >= 7);
			co_return;
		});

		graph.Run();

		PR_EXPECT(sum == 7);
	}

	PRUnitTest(TaskGraphMidTaskSignal, Quick)
	{
		// A task signals an intermediate phase before completing
		std::atomic<int> phase_value = 0;
		std::atomic<int> final_value = 0;

		Graph<TestId> graph(2);

		graph.Add(TestId::A, [&](auto ctx) -> Task {
			phase_value.store(10, std::memory_order_release);
			co_await ctx.Signal(TestId::PhaseOne);
			std::this_thread::sleep_for(std::chrono::milliseconds(30));
			final_value.store(20, std::memory_order_release);
			co_return;
		});

		graph.Add(TestId::B, [&](auto ctx) -> Task {
			co_await ctx.Wait(TestId::PhaseOne);
			// Should see phase_value but final_value may not be set yet
			PR_EXPECT(phase_value.load(std::memory_order_acquire) == 10);
			co_return;
		});

		graph.Run();

		PR_EXPECT(final_value == 20);
	}

	PRUnitTest(TaskGraphStartWait, Quick)
	{
		// Start and Wait allow callers to run owner-thread work while graph tasks are in flight.
		std::atomic<int> sum = 0;

		Graph<TestId> graph(2);
		graph.Add(TestId::A, [&](auto&) -> Task
		{
			sum += 1;
			co_return;
		});
		graph.Add(TestId::B, [&](auto&) -> Task
		{
			sum += 2;
			co_return;
		});

		graph.Start();
		PR_EXPECT(!graph.Wait());

		PR_EXPECT(sum == 3);
	}

	PRUnitTest(TaskGraphStartWaitNoTasks, Quick)
	{
		// Empty graph cycles are valid for callers that build task lists conditionally.
		Graph<TestId> graph(2);

		graph.Start();
		PR_EXPECT(!graph.Wait());
		graph.Reset();
	}

	PRUnitTest(TaskGraphExternalSignalAfterStart, Quick)
	{
		// External signals release tasks that are waiting for work completed outside the worker pool.
		std::atomic<int> value = 0;

		Graph<TestId> graph(2);
		graph.Add(TestId::A, [&](auto ctx) -> Task
		{
			co_await ctx.Wait(TestId::External);
			value.store(1, std::memory_order_release);
			co_return;
		});

		graph.Start();
		graph.Signal(TestId::External);
		PR_EXPECT(!graph.Wait());

		PR_EXPECT(value.load(std::memory_order_acquire) == 1);
	}

	PRUnitTest(TaskGraphExternalSignalBeforeStart, Quick)
	{
		// Signals raised before Start are remembered for tasks that later wait on them in the same graph cycle.
		std::atomic<int> value = 0;

		Graph<TestId> graph(2);
		graph.Signal(TestId::External);
		graph.Add(TestId::A, [&](auto ctx) -> Task
		{
			co_await ctx.Wait(TestId::External);
			value.store(1, std::memory_order_release);
			co_return;
		});

		graph.Start();
		PR_EXPECT(!graph.Wait());

		PR_EXPECT(value.load(std::memory_order_acquire) == 1);
	}

	PRUnitTest(TaskGraphResetAndRerun, Quick)
	{
		// Per-frame reuse: run, reset, run again
		std::atomic<int> counter = 0;

		Graph<TestId> graph(2);

		for (int frame = 0; frame != 3; ++frame)
		{
			graph.Add(TestId::A, [&](auto&) -> Task { counter += 1; co_return; });
			graph.Add(TestId::B, [&](auto ctx) -> Task {
				co_await ctx.Wait(TestId::A);
				counter += 1;
				co_return;
			});

			graph.Run();
			graph.Reset();
		}

		PR_EXPECT(counter == 6); // 2 tasks × 3 frames
	}

	PRUnitTest(TaskGraphException, Quick)
	{
		// Exception in a task propagates to Run()
		Graph<TestId> graph(2);

		graph.Add(TestId::A, [&](auto&) -> Task {
			throw std::runtime_error("task failed");
			co_return;
		});

		bool caught = false;
		try
		{
			graph.Run();
		}
		catch (std::runtime_error const& e)
		{
			caught = true;
			PR_EXPECT(std::string(e.what()) == "task failed");
		}
		PR_EXPECT(caught);
	}

	PRUnitTest(TaskGraphWaitReturnsException, Quick)
	{
		// Wait() drains the graph and returns task failures so callers can decide when to rethrow.
		Graph<TestId> graph(2);

		graph.Add(TestId::A, [&](auto&) -> Task {
			throw std::runtime_error("task failed");
			co_return;
		});

		graph.Start();
		auto ex = graph.Wait();

		bool caught = false;
		try
		{
			PR_EXPECT(ex != nullptr);
			if (ex)
			{
				std::rethrow_exception(ex);
			}
		}
		catch (std::runtime_error const& e)
		{
			caught = true;
			PR_EXPECT(std::string(e.what()) == "task failed");
		}
		PR_EXPECT(caught);
	}

	PRUnitTest(TaskGraphSingleThread, Quick)
	{
		// Verify it works with a single worker thread
		std::atomic<int> sum = 0;

		Graph<TestId> graph(1);

		graph.Add(TestId::A, [&](auto&) -> Task { sum += 1; co_return; });
		graph.Add(TestId::B, [&](auto ctx) -> Task {
			co_await ctx.Wait(TestId::A);
			sum += 2;
			co_return;
		});
		graph.Add(TestId::C, [&](auto ctx) -> Task {
			co_await ctx.Wait(TestId::B);
			sum += 4;
			co_return;
		});

		graph.Run();

		PR_EXPECT(sum == 7);
	}

	PRUnitTest(TaskGraphDestructWhileTaskInFlight, Quick)
	{
		// Destroying the graph must not tear down task/signal state while a worker thread is still
		// actively resuming a task coroutine. This starts a task, confirms it is running on a worker
		// thread, and then destroys the graph *without* calling Wait() first. The task is parked on a
		// plain (non-coroutine) gate inside the coroutine body, so from the coroutine machine's point of
		// view it is not suspended and its frame must not be destroyed while the gate blocks it. If the
		// graph's destructor tears down its task/signal containers before the worker pool has fully shut
		// down and joined, this races the pool thread destroying/using the same coroutine frame; if it
		// shuts the pool down (and so joins the worker thread) first, the destructor simply blocks until
		// the gate is released and then tears down safely. A releaser thread frees the gate shortly after
		// being started so the destructor call below can only return once the task has actually finished.
		std::mutex gate_mutex;
		std::condition_variable gate_cv;
		bool started = false;
		bool release = false;
		bool completed = false;

		auto graph = std::make_unique<Graph<TestId>>(1);
		graph->Add(TestId::A, [&](auto&) -> Task
		{
			{
				auto lock = std::unique_lock(gate_mutex);
				started = true;
				gate_cv.notify_all();
				gate_cv.wait(lock, [&] { return release; });
			}
			completed = true;
			co_return;
		});

		graph->Start();

		// Wait for the task to be confirmed running on the worker thread before destroying the graph.
		{
			auto lock = std::unique_lock(gate_mutex);
			gate_cv.wait(lock, [&] { return started; });
		}

		// Release the gate from another thread shortly after the graph destructor begins tearing down,
		// so the destructor can only return once the in-flight task has actually finished.
		auto releaser = std::thread([&]
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(30));
			auto lock = std::unique_lock(gate_mutex);
			release = true;
			gate_cv.notify_all();
		});

		graph.reset(); // ~Graph() must block here until the worker pool has fully shut down and joined.
		releaser.join();

		PR_EXPECT(completed);
	}
}
#endif
