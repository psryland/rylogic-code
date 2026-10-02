//************************************************************************
// Coroutines
//  Copyright (c) Rylogic Ltd 2024
//************************************************************************
// Task<T>      - An eagerly started, awaitable coroutine that produces a 'T' (or nothing).
// Generator<T> - A lazily evaluated sequence produced by 'co_yield'.
// Scheduler    - A pool of worker threads that coroutines can move to.
//
// Unlike C#, 'co_await' does not change threads. A coroutine runs on the thread that started or
// resumed it until it explicitly moves with 'SwitchToWorkerThread' or 'SwitchToThread'. When an
// awaited Task completes, its awaiter continues on the thread that completed the Task.
#pragma once
#include <cassert>
#include <cstdint>
#include <algorithm>
#include <exception>
#include <optional>
#include <type_traits>
#include <utility>
#include <memory>
#include <vector>
#include <deque>
#include <atomic>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <coroutine>
#include <iterator>
#include <format>
#include <stdexcept>

#include "pr/threads/name_thread.h"

namespace pr::coroutine
{
	// A pool of worker threads that resume coroutines.
	struct Scheduler
	{
		// Notes:
		// - The scheduler is a singleton, but it must be instantiated manually early in the life of
		//   the program. Scheduler instantiations replace the previous scheduler and restore it when
		//   destructed.
		// - The reason for doing it this way is so that the scheduler is constructed/destructed within
		//   the normal scope of a program rather than at static initialization/destruction time.
		// - Destruction blocks until all scheduled work has run, including work scheduled by that work.
		//   Coroutines that are suspended waiting for something other than the scheduler are not tracked,
		//   so callers must make sure those have finished before destroying the scheduler.

	private:

		// A worker thread with its own queue of coroutines to resume
		struct Worker
		{
			Scheduler& m_owner;
			std::mutex m_mutex;
			std::condition_variable m_cv_queued;
			std::deque<std::coroutine_handle<>> m_queue;
			bool m_shutdown;
			std::thread m_thread;

			explicit Worker(Scheduler& owner)
				: m_owner(owner)
				, m_mutex()
				, m_cv_queued()
				, m_queue()
				, m_shutdown()
				, m_thread([this] { Run(); })
			{
			}
			Worker(Worker&&) = delete;
			Worker(Worker const&) = delete;
			Worker& operator =(Worker&&) = delete;
			Worker& operator =(Worker const&) = delete;
			~Worker()
			{
				// Ask the thread to exit once its queue is empty
				{
					std::lock_guard lock(m_mutex);
					m_shutdown = true;
				}
				m_cv_queued.notify_all();
				m_thread.join();
			}

			// Queue a coroutine to be resumed on this worker thread
			void Enqueue(std::coroutine_handle<> coroutine)
			{
				// Add to the queue and wake the thread
				{
					std::lock_guard lock(m_mutex);
					m_queue.push_back(coroutine);
				}
				m_cv_queued.notify_one();
			}

		private:

			// The worker thread main loop
			void Run()
			{
				// Name the thread to help debugging
				threads::SetCurrentThreadName(std::format("Worker({})", std::this_thread::get_id()));

				// Resume queued coroutines until shutdown is requested and the queue is empty
				for (;;)
				{
					// Take the next job. Remaining jobs are still run after shutdown is requested
					// because dropping them would leave their coroutines suspended forever.
					std::coroutine_handle<> job;
					{
						std::unique_lock lock(m_mutex);
						m_cv_queued.wait(lock, [this] { return !m_queue.empty() || m_shutdown; });
						if (m_queue.empty())
							break;

						job = m_queue.front();
						m_queue.pop_front();
					}

					// Run the job up to its next suspension point, or to completion
					job.resume();
					m_owner.JobDone();
				}
			}
		};

		// Singleton instance
		inline static Scheduler* s_instance = nullptr;
		Scheduler* m_prev_scheduler;

		// The number of scheduled jobs that have not finished running.
		// Declared before 'm_workers' so that it outlives the worker threads.
		std::atomic<int64_t> m_pending;

		// Round-robin counter used to spread work over the workers
		std::atomic<uint32_t> m_next_worker;

		// The worker threads
		std::vector<std::unique_ptr<Worker>> m_workers;

	public:

		// Create a scheduler with 'threads' worker threads ('threads' must be > 0)
		explicit Scheduler(int threads = DefaultThreadCount())
			: m_prev_scheduler(s_instance)
			, m_pending()
			, m_next_worker()
			, m_workers()
		{
			// Start the worker threads, then make this the current scheduler
			assert(threads > 0 && "A scheduler needs at least one worker thread");
			m_workers.reserve(threads);
			for (int i = 0; i != threads; ++i)
				m_workers.push_back(std::make_unique<Worker>(*this));

			s_instance = this;
		}
		Scheduler(Scheduler&&) = delete;
		Scheduler(Scheduler const&) = delete;
		Scheduler& operator =(Scheduler&&) = delete;
		Scheduler& operator =(Scheduler const&) = delete;
		~Scheduler()
		{
			// Wait for all scheduled work to finish. The workers are joined when 'm_workers' is destroyed.
			for (auto pending = m_pending.load(); pending != 0; pending = m_pending.load())
				m_pending.wait(pending);

			assert(s_instance == this && "Schedulers must be destroyed in reverse order of construction");
			s_instance = m_prev_scheduler;
		}

		// The default number of worker threads
		static int DefaultThreadCount()
		{
			// 'hardware_concurrency' returns 0 when the value is unknown
			return std::max(1, static_cast<int>(std::thread::hardware_concurrency()));
		}

		// Queue a coroutine to be resumed on a worker thread.
		// Use 'thread_id = {}' to run on any worker thread. Throws if 'thread_id' is not a worker of this scheduler.
		void Schedule(std::coroutine_handle<> coroutine, std::thread::id thread_id = {})
		{
			// Choose the worker before counting the job, so that an invalid thread id doesn't leave a pending count behind
			auto& worker = thread_id != std::thread::id{} ? FindWorker(thread_id) : *m_workers[m_next_worker++ % m_workers.size()];
			++m_pending;
			worker.Enqueue(coroutine);
		}

		// Get the singleton instance of the scheduler
		static Scheduler& instance()
		{
			// The caller must create a scheduler before using it
			assert(s_instance != nullptr && "No scheduler has been created");
			return *s_instance;
		}

	private:

		// Find the worker that owns 'thread_id'
		Worker& FindWorker(std::thread::id thread_id) const
		{
			// Linear search, the number of workers is small
			for (auto& worker : m_workers)
			{
				if (worker->m_thread.get_id() == thread_id)
					return *worker;
			}
			throw std::runtime_error("Thread ID is not a worker thread of this scheduler");
		}

		// Called by workers after each job has run
		void JobDone()
		{
			// Wake the destructor when the last job finishes
			if (--m_pending == 0)
				m_pending.notify_all();
		}
	};

	template <typename T = void> struct Task;

	namespace impl
	{
		// Promise state shared by all Task types.
		template <typename Derived>
		struct TaskPromiseBase
		{
			// Notes:
			// - The coroutine frame (which contains this promise) is co-owned by the Task returned to the caller
			//   and by the running coroutine. Whichever releases last destroys the frame. This allows a Task to be
			//   dropped while its coroutine is still running, without any allocation beyond the frame itself.
			// - 'm_state' is the hand-off between the coroutine completing and another coroutine awaiting it.
			//   Its value is one of:
			//     nullptr          - Running, and nothing is awaiting it.
			//     CompletedState() - Completed. The result or exception is available.
			//     anything else    - The address of the coroutine that is awaiting this one.
			//   Both sides use atomic read-modify-write operations, so exactly one of them resumes the awaiter.
			// - Only one coroutine can 'co_await' a Task. Any number of threads can block in 'Wait'.

			std::atomic<void*> m_state = nullptr;
			std::atomic<int> m_refs = 2;
			std::exception_ptr m_exception = {};

			TaskPromiseBase() = default;
			TaskPromiseBase(TaskPromiseBase&&) = delete;
			TaskPromiseBase(TaskPromiseBase const&) = delete;
			TaskPromiseBase& operator=(TaskPromiseBase&&) = delete;
			TaskPromiseBase& operator=(TaskPromiseBase const&) = delete;

			// The value of 'm_state' once the coroutine has completed. No coroutine can await itself, so this can't be an awaiter address.
			void* CompletedState() const noexcept
			{
				// Use the address of this promise as a unique marker
				return const_cast<TaskPromiseBase*>(this);
			}

			// The coroutine handle of the coroutine that this promise belongs to
			std::coroutine_handle<Derived> Coroutine() noexcept
			{
				// The promise lives in the coroutine frame, so the handle can be found from it
				return std::coroutine_handle<Derived>::from_promise(static_cast<Derived&>(*this));
			}

			// Create the Task that is returned to the caller of the coroutine function
			auto get_return_object() noexcept
			{
				// The Task takes one of the two initial references
				return typename Derived::task_type(&static_cast<Derived&>(*this));
			}

			// Tasks start running as soon as the coroutine function is called
			std::suspend_never initial_suspend() const noexcept
			{
				// Run the body immediately on the calling thread
				return {};
			}

			// Called when the coroutine body completes, normally or with an exception
			auto final_suspend() noexcept
			{
				// The frame must stay suspended at the final suspend point until both owners have released it
				struct FinalAwaiter
				{
					bool await_ready() const noexcept
					{
						// Always suspend so the frame can't be destroyed while still running
						return false;
					}
					std::coroutine_handle<> await_suspend(std::coroutine_handle<Derived> self) const noexcept
					{
						// Publish completion and take the awaiter, if there is one. Release ordering makes the result
						// visible to threads that observe the completed state.
						auto& promise = self.promise();
						auto awaiter = promise.m_state.exchange(promise.CompletedState(), std::memory_order_acq_rel);
						promise.m_state.notify_all();

						// Release the running coroutine's reference. This may destroy the frame, so the promise
						// (and this awaiter, which lives in the frame) must not be used after this point.
						promise.Release();

						// Continue the awaiting coroutine on this thread, if there is one
						return awaiter != nullptr ? std::coroutine_handle<>::from_address(awaiter) : std::noop_coroutine();
					}
					void await_resume() const noexcept
					{
						// Never resumed, a completed coroutine is only destroyed
						assert(false && "A completed coroutine should not be resumed");
					}
				};
				return FinalAwaiter{};
			}

			// Capture exceptions that escape the coroutine body so they can be rethrown to the consumer of the result
			void unhandled_exception() noexcept
			{
				// Stored until the result is read
				m_exception = std::current_exception();
			}

			// True once the coroutine has completed
			bool IsDone() const noexcept
			{
				// Acquire ordering makes the result written by the coroutine visible
				return m_state.load(std::memory_order_acquire) == CompletedState();
			}

			// Block the calling thread until the coroutine has completed
			void Wait() const noexcept
			{
				// 'm_state' can change from nullptr to an awaiter address before completion, so wait again until it's the completed state
				for (auto state = m_state.load(std::memory_order_acquire); state != CompletedState(); state = m_state.load(std::memory_order_acquire))
					m_state.wait(state, std::memory_order_acquire);
			}

			// Register 'awaiter' to be resumed when the coroutine completes. Returns false if it has already completed.
			bool SetAwaiter(std::coroutine_handle<> awaiter) noexcept
			{
				// Succeeds only if the coroutine is still running and nothing else is awaiting it
				void* expected = nullptr;
				if (m_state.compare_exchange_strong(expected, awaiter.address(), std::memory_order_acq_rel, std::memory_order_acquire))
					return true;

				assert(expected == CompletedState() && "A Task can only be awaited by one coroutine");
				return false;
			}

			// Rethrow the exception from the coroutine body, if there was one. Only valid once the coroutine has completed.
			void RethrowIfFailed() const
			{
				// Rethrow a copy of the stored exception, so it can be rethrown again
				if (m_exception)
					std::rethrow_exception(m_exception);
			}

			// Add an owner of the coroutine frame
			void AddRef() noexcept
			{
				// Ordering isn't needed to add a reference to something the caller already owns
				m_refs.fetch_add(1, std::memory_order_relaxed);
			}

			// Remove an owner of the coroutine frame, destroying the frame when there are no owners left
			void Release() noexcept
			{
				// Acquire/release ordering makes every owner's use of the frame happen before its destruction
				if (m_refs.fetch_sub(1, std::memory_order_acq_rel) == 1)
					Coroutine().destroy();
			}
		};

		// The promise type for Task<T>
		template <typename T>
		struct TaskPromise : TaskPromiseBase<TaskPromise<T>>
		{
			using task_type = Task<T>;
			std::optional<T> m_value;

			// Called by 'co_return value'
			template <typename U = T> requires std::convertible_to<U&&, T>
			void return_value(U&& value)
			{
				// Construct in place, 'T' doesn't need to be default constructible
				m_value.emplace(std::forward<U>(value));
			}

			// The result of the coroutine. Only valid once the coroutine has completed.
			T& Result()
			{
				// Rethrow failures in the context of the consumer
				this->RethrowIfFailed();
				return *m_value;
			}
		};
		template <>
		struct TaskPromise<void> : TaskPromiseBase<TaskPromise<void>>
		{
			using task_type = Task<void>;

			// Called by 'co_return' or falling off the end of the coroutine body
			void return_void() noexcept
			{
				// Nothing to store
			}

			// The result of the coroutine. Only valid once the coroutine has completed.
			void Result() const
			{
				// Rethrow failures in the context of the consumer
				this->RethrowIfFailed();
			}
		};
	}

	// The return type of an eagerly started coroutine that produces a 'T' (or nothing).
	template <typename T>
	struct Task final
	{
		// Notes:
		// - The coroutine starts running when the coroutine function is called. It runs on the calling thread until
		//   it first suspends (e.g. at 'co_await SwitchToWorkerThread()'), and then the Task is returned to the caller.
		// - Task is move-only. It co-owns the coroutine frame with the running coroutine (see TaskPromiseBase).
		//   Dropping a Task does not cancel the coroutine. It keeps running, and any exception it throws is lost.
		// - A Task can be awaited ('co_await task') by one coroutine. Awaiting an lvalue Task gives a reference to
		//   the result, which is valid while the Task exists. Awaiting an rvalue Task moves the result out.
		// - 'Wait' and 'Result' block the calling thread. Blocking a worker thread on a Task that needs that same
		//   worker to make progress will deadlock.
		// - Results and exceptions are delivered by 'co_await' and 'Result'. Exceptions are rethrown each time the result is read.
		static_assert(!std::is_reference_v<T>, "Task results can't be references, use a pointer or std::reference_wrapper instead");

		using promise_type = impl::TaskPromise<T>;
		friend impl::TaskPromiseBase<promise_type>;

	private:

		promise_type* m_promise;

		// Created by the promise, taking one of its initial references
		explicit Task(promise_type* promise) noexcept
			: m_promise(promise)
		{
		}

		// Awaiter for 'co_await task'. 'Move' selects whether the result is moved out or returned by reference.
		template <bool Move>
		struct Awaiter
		{
			promise_type* m_promise;

			bool await_ready() const noexcept
			{
				// Don't suspend if the result is already available
				return m_promise->IsDone();
			}
			bool await_suspend(std::coroutine_handle<> awaiter) const noexcept
			{
				// If the coroutine completed in the meantime, resume the awaiter immediately
				return m_promise->SetAwaiter(awaiter);
			}
			decltype(auto) await_resume() const
			{
				// Deliver the result, or rethrow the coroutine's exception
				if constexpr (Move && !std::is_void_v<T>)
					return T(std::move(m_promise->Result()));
				else
					return m_promise->Result();
			}
		};

		// Awaiter for 'co_await task.Completion()'. Resumes when the coroutine completes, without reading the result.
		struct CompletionAwaiter : Awaiter<false>
		{
			void await_resume() const noexcept
			{
				// The result, or exception, is left in the Task
			}
		};

	public:

		// An empty Task, not associated with any coroutine
		Task() noexcept
			: m_promise()
		{
		}
		Task(Task&& rhs) noexcept
			: m_promise(std::exchange(rhs.m_promise, nullptr))
		{
		}
		Task(Task const&) = delete;
		Task& operator =(Task&& rhs) noexcept
		{
			// Release the current coroutine (if any) and take ownership of 'rhs'
			if (this != &rhs)
			{
				Task old(std::move(*this));
				m_promise = std::exchange(rhs.m_promise, nullptr);
			}
			return *this;
		}
		Task& operator =(Task const&) = delete;
		~Task()
		{
			// Release this Task's ownership of the coroutine frame
			if (m_promise)
				m_promise->Release();
		}

		// True if this Task is associated with a coroutine
		explicit operator bool() const noexcept
		{
			// Default constructed and moved-from Tasks are empty
			return m_promise != nullptr;
		}

		// True if the coroutine has completed (normally or with an exception). Does not block.
		bool IsDone() const noexcept
		{
			// Only valid on non-empty Tasks
			assert(m_promise && "Task is empty");
			return m_promise->IsDone();
		}

		// Block the calling thread until the coroutine completes. Does not throw if the coroutine failed.
		void Wait() const noexcept
		{
			// Only valid on non-empty Tasks
			assert(m_promise && "Task is empty");
			m_promise->Wait();
		}

		// Block the calling thread until the coroutine completes, then return its result or rethrow its exception.
		// The returned reference is valid while this Task exists.
		std::add_lvalue_reference_t<T> Result() const&
		{
			// Wait for completion before reading the result
			Wait();
			return m_promise->Result();
		}

		// Block the calling thread until the coroutine completes, then move out its result or rethrow its exception.
		T Result() &&
		{
			// Wait for completion before reading the result
			Wait();
			if constexpr (!std::is_void_v<T>)
				return T(std::move(m_promise->Result()));
			else
				return m_promise->Result();
		}

		// 'co_await task' resumes with a reference to the result once the coroutine completes
		Awaiter<false> operator co_await() const& noexcept
		{
			// Only valid on non-empty Tasks
			assert(m_promise && "Task is empty");
			return { m_promise };
		}

		// 'co_await Function()' resumes with the moved result once the coroutine completes
		Awaiter<true> operator co_await() const&& noexcept
		{
			// Only valid on non-empty Tasks
			assert(m_promise && "Task is empty");
			return { m_promise };
		}

		// An awaitable that resumes once the coroutine completes, without reading the result or rethrowing its exception
		CompletionAwaiter Completion() const noexcept
		{
			// Only valid on non-empty Tasks
			assert(m_promise && "Task is empty");
			return { { m_promise } };
		}
	};

	// The return type of a coroutine that lazily produces a sequence of values using 'co_yield'.
	template <typename T>
	struct Generator final
	{
		// Notes:
		// - The coroutine doesn't run until 'begin()' is called, and then runs on the calling thread
		//   up to each 'co_yield'. 'begin()' should only be called once.
		// - Iterators refer to the yielded object inside the coroutine, so values are not copied unless the caller copies them.
		// - Exceptions thrown by the coroutine are rethrown from 'begin()' or 'operator++'. The sequence ends after an exception.
		// - Generators are synchronous, so 'co_await' isn't allowed in the coroutine body.
		static_assert(!std::is_reference_v<T>, "Generator values can't be references");

		struct promise_type;
		using handle_type = std::coroutine_handle<promise_type>;

		struct promise_type
		{
			T const* m_value = nullptr;
			std::exception_ptr m_exception = {};

			Generator get_return_object() noexcept
			{
				// The Generator owns the coroutine frame
				return Generator(handle_type::from_promise(*this));
			}
			std::suspend_always initial_suspend() const noexcept
			{
				// Don't run until the first value is requested
				return {};
			}
			std::suspend_always final_suspend() const noexcept
			{
				// Stay suspended so the Generator can detect the end of the sequence and destroy the frame
				return {};
			}
			void unhandled_exception() noexcept
			{
				// Stored and rethrown in the context of the iterating thread
				m_exception = std::current_exception();
			}
			void return_void() noexcept
			{
				// The end of the sequence
			}
			std::suspend_always yield_value(T const& value) noexcept
			{
				// The yielded object (including a temporary) lives until the coroutine is resumed, so a pointer to it is enough
				m_value = std::addressof(value);
				return {};
			}
			template <typename U>
			std::suspend_never await_transform(U&&) = delete;
		};

		// Input iterator over the generated sequence
		class iterator
		{
			friend struct Generator;
			handle_type m_handle;

			explicit iterator(handle_type handle) noexcept
				: m_handle(handle)
			{
			}

		public:

			using iterator_concept = std::input_iterator_tag;
			using difference_type = std::ptrdiff_t;
			using value_type = T;

			iterator() noexcept
				: m_handle()
			{
			}
			T const& operator *() const noexcept
			{
				// The value from the most recent 'co_yield'
				return *m_handle.promise().m_value;
			}
			iterator& operator ++()
			{
				// Run the coroutine to the next 'co_yield' or to the end
				Advance(m_handle);
				return *this;
			}
			void operator ++(int)
			{
				// Input iterators can't return the previous position
				++*this;
			}
			friend bool operator == (iterator const& it, std::default_sentinel_t) noexcept
			{
				// The sequence ends when the coroutine completes
				return !it.m_handle || it.m_handle.done();
			}
		};

	private:

		handle_type m_handle;

		explicit Generator(handle_type handle) noexcept
			: m_handle(handle)
		{
		}

		// Resume the coroutine to the next 'co_yield' or to completion, rethrowing any exception from it
		static void Advance(handle_type handle)
		{
			// An exception ends the coroutine, so the iterator compares equal to 'end()' afterwards
			handle.resume();
			if (handle.done() && handle.promise().m_exception)
				std::rethrow_exception(handle.promise().m_exception);
		}

	public:

		Generator(Generator&& rhs) noexcept
			: m_handle(std::exchange(rhs.m_handle, nullptr))
		{
		}
		Generator(Generator const&) = delete;
		Generator& operator =(Generator&& rhs) noexcept
		{
			// Destroy the current coroutine (if any) and take ownership of 'rhs'
			if (this != &rhs)
			{
				Generator old(std::move(*this));
				m_handle = std::exchange(rhs.m_handle, nullptr);
			}
			return *this;
		}
		Generator& operator =(Generator const&) = delete;
		~Generator()
		{
			// Destroying a suspended coroutine destroys its frame and any locals still alive in it
			if (m_handle)
				m_handle.destroy();
		}

		// True if this Generator owns a coroutine
		explicit operator bool() const noexcept
		{
			// Moved-from Generators are empty
			return m_handle != nullptr;
		}

		// Start the sequence. Runs the coroutine to the first 'co_yield'.
		iterator begin()
		{
			// Only valid on a non-empty Generator that hasn't been started
			assert(m_handle && "Generator is empty");
			Advance(m_handle);
			return iterator(m_handle);
		}

		// The end of the sequence
		std::default_sentinel_t end() const noexcept
		{
			// Iterators compare equal to the sentinel once the coroutine completes
			return {};
		}
	};

	// An awaitable that moves the awaiting coroutine to the worker thread 'thread_id' of the current scheduler.
	// Use 'thread_id = {}' for any worker thread. Throws, at the 'co_await', if 'thread_id' is not a worker thread.
	inline auto SwitchToThread(std::thread::id thread_id)
	{
		// The awaiting coroutine suspends and is resumed by the chosen worker thread
		struct Awaiter
		{
			std::thread::id m_thread_id;
			bool await_ready() const noexcept
			{
				// No need to switch if already on the requested thread
				return std::this_thread::get_id() == m_thread_id;
			}
			void await_suspend(std::coroutine_handle<> awaiter) const
			{
				// The coroutine may resume on the worker before this returns, so nothing can be used after scheduling it
				Scheduler::instance().Schedule(awaiter, m_thread_id);
			}
			void await_resume() const noexcept
			{
				// Nothing to return
			}
		};
		return Awaiter{ thread_id };
	}

	// An awaitable that moves the awaiting coroutine to any worker thread of the current scheduler
	inline auto SwitchToWorkerThread()
	{
		// Any worker will do
		return SwitchToThread({});
	}

	// Wait for all of 'tasks' to complete. The tasks must outlive the returned Task.
	// If any task failed, the exception of the first failed task (in argument order) is rethrown after all have completed.
	template <typename... T>
	Task<> WhenAll(Task<T> const&... tasks)
	{
		// Wait for every task, even after a failure, so that none are still running when this completes
		(co_await tasks.Completion(), ...);

		// Rethrow the first failure, if any. The explicit 'co_return' keeps this a coroutine when 'tasks' is empty.
		(tasks.Result(), ...);
		co_return;
	}
}

#if PR_UNITTESTS
#include <chrono>
#include <functional>
#include "pr/common/unittests.h"
namespace pr::coroutine
{
	namespace tests
	{
		using namespace std::chrono_literals;

		// Poll 'pred' until it is true or 'timeout' expires, so a failing test reports instead of hanging
		inline bool WaitUntil(std::function<bool()> pred, std::chrono::milliseconds timeout = 5000ms)
		{
			// Poll without sleeping, because a sleep is rounded up to the OS timer resolution
			auto end = std::chrono::steady_clock::now() + timeout;
			for (; !pred(); std::this_thread::yield())
			{
				if (std::chrono::steady_clock::now() > end)
					return false;
			}
			return true;
		}

		// Counts its live (non-moved-from) instances. Used as a coroutine parameter to detect when a frame is destroyed.
		struct FrameProbe
		{
			std::atomic_int* m_live;

			explicit FrameProbe(std::atomic_int& live)
				: m_live(&live)
			{
				++live;
			}
			FrameProbe(FrameProbe&& rhs) noexcept
				: m_live(std::exchange(rhs.m_live, nullptr))
			{
			}
			~FrameProbe()
			{
				// Only the owning instance counts
				if (m_live)
					--*m_live;
			}
		};

		Task<int> ReturnSync(int value)
		{
			co_return value;
		}
		int Boom()
		{
			throw std::runtime_error("boom");
		}
		Task<> ReturnVoidSync(int& counter)
		{
			++counter;
			co_return;
		}
		Task<std::thread::id> WorkerThreadId()
		{
			co_await SwitchToWorkerThread();
			co_return std::this_thread::get_id();
		}
		Task<int> WorkerValue(int value)
		{
			co_await SwitchToWorkerThread();
			co_return value;
		}
		Task<float> WorkerFloat(std::chrono::milliseconds delay)
		{
			co_await SwitchToWorkerThread();
			std::this_thread::sleep_for(delay);
			co_return 6.28f;
		}
		Task<int> AwaitOtherType(std::chrono::milliseconds delay)
		{
			// A Task<int> awaiting a Task<float>, completing after the awaiter suspends
			auto value = co_await WorkerFloat(delay);
			co_return value == 6.28f ? 1 : 0;
		}
		Task<int> AwaitChild(int value)
		{
			// Races the child completing on a worker with this coroutine suspending
			co_return co_await WorkerValue(value) + 1;
		}
		Task<int> AwaitCompleted()
		{
			// The awaited task has already completed, so this doesn't suspend
			auto task = ReturnSync(41);
			auto& value = co_await task;
			co_return value + 1;
		}
		Task<int> Throws(bool on_worker)
		{
			if (on_worker)
				co_await SwitchToWorkerThread();

			co_return Boom();
		}
		Task<int> CatchesChild()
		{
			// Exceptions from an awaited task are rethrown at the 'co_await'
			try
			{
				co_await Throws(true);
				co_return 0;
			}
			catch (std::runtime_error const&)
			{
			}
			co_return 1;
		}
		Task<std::unique_ptr<int>> MakeUnique(int value)
		{
			co_await SwitchToWorkerThread();
			co_return std::make_unique<int>(value);
		}
		Task<int> AwaitMoveOnly()
		{
			// Awaiting an rvalue Task moves the result out
			auto ptr = co_await MakeUnique(5);
			co_return *ptr;
		}
		Task<> HoldUntil(FrameProbe, std::atomic_bool& release)
		{
			co_await SwitchToWorkerThread();
			release.wait(false);
		}
		Task<int> Rendezvous(std::atomic_int& arrived, int count, int value)
		{
			// Only completes if all 'count' tasks are running at the same time
			co_await SwitchToWorkerThread();
			++arrived;
			if (!WaitUntil([&] { return arrived.load() == count; }))
				throw std::runtime_error("tasks did not run concurrently");

			co_return value;
		}
		Task<int> JobAsync(std::atomic_int& arrived)
		{
			co_await SwitchToWorkerThread();
			auto t0 = Rendezvous(arrived, 3, 1 << 0);
			auto t1 = Rendezvous(arrived, 3, 1 << 1);
			auto t2 = Rendezvous(arrived, 3, 1 << 2);
			co_await WhenAll(t0, t1, t2);
			co_return t0.Result() + t1.Result() + t2.Result();
		}
		Task<int> WhenAllWithFailure(std::atomic_int& completed)
		{
			auto t0 = Throws(true);
			auto t1 = WorkerFloat(50ms);
			auto t2 = WorkerValue(3);
			try
			{
				co_await WhenAll(t0, t1, t2);
			}
			catch (std::runtime_error const&)
			{
				completed = int(t0.IsDone()) + int(t1.IsDone()) + int(t2.IsDone());
				co_return 1;
			}
			co_return 0;
		}
		Task<bool> SwitchBackToThread()
		{
			// Leave a worker and return to it specifically
			co_await SwitchToWorkerThread();
			auto id = std::this_thread::get_id();
			for (int i = 0; i != 4; ++i)
				co_await SwitchToWorkerThread();

			co_await SwitchToThread(id);
			co_return std::this_thread::get_id() == id;
		}
		Task<> SwitchToNonWorker(std::thread::id thread_id)
		{
			co_await SwitchToWorkerThread();
			co_await SwitchToThread(thread_id);
		}
		Task<> Increment(std::atomic_int& counter)
		{
			co_await SwitchToWorkerThread();
			++counter;
		}
		Generator<int> Fibonacci(int n)
		{
			int a = 0, b = 1;
			for (int i = 0; i != n; ++i)
			{
				co_yield a;

				auto next = a + b;
				a = b;
				b = next;
			}
		}
		Generator<std::string> Words(FrameProbe, bool fail)
		{
			co_yield "one";
			co_yield std::string("two");
			if (fail)
				throw std::runtime_error("boom");

			co_yield "three";
		}
	}

	PRUnitTestClass(CoroutineTests)
	{
		PRUnitTestMethod(SynchronousCompletion, Quick)
		{
			using namespace tests;
			Scheduler scheduler(1);

			// A coroutine that never suspends completes before the Task is returned
			auto task = ReturnSync(42);
			PR_EXPECT(task.IsDone());
			PR_EXPECT(task.Result() == 42);

			int counter = 0;
			auto void_task = ReturnVoidSync(counter);
			PR_EXPECT(void_task.IsDone());
			PR_EXPECT(counter == 1);
			void_task.Result();
		}
		PRUnitTestMethod(WorkerThread, Quick)
		{
			using namespace tests;
			Scheduler scheduler(2);

			// Code after 'SwitchToWorkerThread' runs on a worker, and blocking for the result stays on this thread
			auto main_id = std::this_thread::get_id();
			auto worker_id = WorkerThreadId().Result();
			PR_EXPECT(worker_id != main_id);
			PR_EXPECT(std::this_thread::get_id() == main_id);
		}
		PRUnitTestMethod(AwaitingTasks, Quick)
		{
			using namespace tests;
			Scheduler scheduler(2);

			// Await a task of a different result type that completes after the awaiter suspends
			{
				auto task = AwaitOtherType(50ms);
				PR_EXPECT(WaitUntil([&] { return task.IsDone(); }));
				PR_EXPECT(task.Result() == 1);
			}

			// Await a task that has already completed
			PR_EXPECT(AwaitCompleted().Result() == 42);

			// Exercise the race between a child completing and its parent suspending.
			// Every parent must be resumed exactly once with the child's result.
			for (int i = 0; i != 2000; ++i)
			{
				auto task = AwaitChild(i);
				if (!WaitUntil([&] { return task.IsDone(); }))
				{
					PR_EXPECT(false);
					break;
				}
				PR_EXPECT(task.Result() == i + 1);
			}
		}
		PRUnitTestMethod(Exceptions, Quick)
		{
			using namespace tests;
			Scheduler scheduler(1);

			// Exceptions are rethrown by 'Result', every time it is called, but not by 'Wait'
			auto sync = Throws(false);
			PR_EXPECT(sync.IsDone());
			sync.Wait();
			PR_THROWS(sync.Result(), std::runtime_error);
			PR_THROWS(sync.Result(), std::runtime_error);

			auto async = Throws(true);
			async.Wait();
			PR_THROWS(async.Result(), std::runtime_error);

			// Exceptions are rethrown at 'co_await'
			PR_EXPECT(CatchesChild().Result() == 1);
		}
		PRUnitTestMethod(MoveOnlyResults, Quick)
		{
			using namespace tests;
			Scheduler scheduler(1);

			// 'Result' on an rvalue Task moves the result out
			auto ptr = MakeUnique(3).Result();
			PR_EXPECT(ptr && *ptr == 3);

			// 'Result' on an lvalue Task returns a reference to the result
			auto task = MakeUnique(4);
			PR_EXPECT(*task.Result() == 4);
			PR_EXPECT(task.Result() != nullptr);

			// Awaiting an rvalue Task moves the result out
			PR_EXPECT(AwaitMoveOnly().Result() == 5);
		}
		PRUnitTestMethod(Ownership, Quick)
		{
			using namespace tests;
			Scheduler scheduler(1);
			std::atomic_int live = 0;

			// Dropping a Task doesn't stop its coroutine. The frame is destroyed when the coroutine completes.
			{
				std::atomic_bool release = false;
				{
					auto task = HoldUntil(FrameProbe(live), release);
					PR_EXPECT(live == 1);
				}
				PR_EXPECT(live == 1);

				release = true;
				release.notify_all();
				PR_EXPECT(WaitUntil([&] { return live == 0; }));
			}

			// A Task that outlives its coroutine destroys the frame when the Task is destroyed
			{
				std::atomic_bool release = true;
				{
					auto task = HoldUntil(FrameProbe(live), release);
					task.Wait();
					PR_EXPECT(live == 1);
				}
				PR_EXPECT(live == 0);
			}

			// Move construction and assignment transfer ownership, and assignment releases the previous coroutine
			{
				std::atomic_bool release = true;
				auto a = HoldUntil(FrameProbe(live), release);
				a.Wait();
				auto b = std::move(a);
				PR_EXPECT(!a);
				PR_EXPECT(b && b.IsDone());
				PR_EXPECT(live == 1);

				b = HoldUntil(FrameProbe(live), release);
				b.Wait();
				PR_EXPECT(live == 1);

				b = Task<>{};
				PR_EXPECT(!b);
				PR_EXPECT(live == 0);
			}
		}
		PRUnitTestMethod(WhenAllTasks, Quick)
		{
			using namespace tests;
			Scheduler scheduler(4);

			// The tasks run concurrently, and the results are available after WhenAll
			std::atomic_int arrived = 0;
			PR_EXPECT(JobAsync(arrived).Result() == 0b111);

			// A failure is rethrown only after all of the tasks have completed
			std::atomic_int completed = 0;
			PR_EXPECT(WhenAllWithFailure(completed).Result() == 1);
			PR_EXPECT(completed == 3);
		}
		PRUnitTestMethod(SwitchThreads, Quick)
		{
			using namespace tests;
			Scheduler scheduler(3);

			// Switch to a specific worker thread
			PR_EXPECT(SwitchBackToThread().Result());

			// Switching to a thread that isn't a worker throws at the 'co_await'
			PR_THROWS(SwitchToNonWorker(std::this_thread::get_id()).Result(), std::runtime_error);
		}
		PRUnitTestMethod(SchedulerLifetime, Quick)
		{
			using namespace tests;

			// Schedulers nest, restoring the previous instance on destruction
			Scheduler outer(1);
			PR_EXPECT(&Scheduler::instance() == &outer);
			{
				// Destroying a scheduler runs all scheduled work first
				std::atomic_int counter = 0;
				{
					Scheduler inner(2);
					PR_EXPECT(&Scheduler::instance() == &inner);
					for (int i = 0; i != 100; ++i)
						Increment(counter);
				}
				PR_EXPECT(counter == 100);
			}
			PR_EXPECT(&Scheduler::instance() == &outer);
		}
		PRUnitTestMethod(Generators, Quick)
		{
			using namespace tests;

			// Values are produced lazily, in order
			{
				int fib[] = { 0, 1, 1, 2, 3, 5, 8, 13, 21, 34 }, i = 0;
				for (auto f : Fibonacci(10))
				{
					PR_EXPECT(i != 10 && f == fib[i]);
					++i;
				}
				PR_EXPECT(i == 10);
			}

			// An empty sequence
			{
				auto count = 0;
				for ([[maybe_unused]] auto f : Fibonacci(0))
					++count;

				PR_EXPECT(count == 0);
			}

			// Exceptions are rethrown during iteration, and end the sequence
			std::atomic_int live = 0;
			{
				auto words = Words(FrameProbe(live), true);
				auto it = words.begin();
				PR_EXPECT(*it == "one");
				++it;
				PR_EXPECT(*it == "two");
				PR_THROWS(++it, std::runtime_error);
				PR_EXPECT(it == words.end());
			}
			PR_EXPECT(live == 0);

			// Stopping early destroys the coroutine frame
			{
				auto words = Words(FrameProbe(live), false);
				auto it = words.begin();
				PR_EXPECT(*it == "one");
				PR_EXPECT(live == 1);
			}
			PR_EXPECT(live == 0);

			// Moving transfers ownership
			{
				auto a = Words(FrameProbe(live), false);
				auto b = std::move(a);
				std::vector<std::string> words;
				for (auto const& word : b)
					words.push_back(word);

				PR_EXPECT(!a);
				PR_EXPECT((words == std::vector<std::string>{ "one", "two", "three" }));
			}
			PR_EXPECT(live == 0);
		}
	};
}
#endif
