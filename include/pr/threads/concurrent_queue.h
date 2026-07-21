//*********************************************************************
// Concurrent Queue
//  Copyright (c) Rylogic Ltd 2011
//*********************************************************************
// Thread safe producer/consumer queue
// See unit tests for usage
#pragma once
#include <deque>
#include <thread>
#include <mutex>
#include <chrono>
#include <condition_variable>

namespace pr::threads
{
	// Base class for concurrent queues.
	// Allows the mutex to be provided externally.
	struct ConcurrentQueueBase
	{
		using MLock = std::unique_lock<std::mutex>;

	protected:

		std::mutex& m_mutex;
		std::condition_variable m_cv_added;
		std::condition_variable m_cv_empty;
		bool m_last;

		ConcurrentQueueBase(std::mutex& mutex)
			:m_mutex(mutex)
			,m_cv_added()
			,m_cv_empty()
			,m_last(false)
		{}

		ConcurrentQueueBase(ConcurrentQueueBase const&) = delete;
		ConcurrentQueueBase& operator=(ConcurrentQueueBase const&) = delete;

		// Call this after the last item has been added to the queue.
		// Queueing anything after 'm_last' has been set throws an exception
		void LastAdded()
		{
			MLock lock(m_mutex);
			m_last = true;
			m_cv_added.notify_all();
			m_cv_empty.notify_all();
		}
	};

	// Concurrent queue implementation.
	// Caller provides the mutex on which the queue is synchronised
	template <typename T> struct ConcurrentQueue2 :ConcurrentQueueBase
	{
	protected:

		std::deque<T> m_queue;

	public:

		explicit ConcurrentQueue2(std::mutex& mutex)
			:ConcurrentQueueBase(mutex)
		{}
		ConcurrentQueue2(ConcurrentQueue2 const&) = delete;
		ConcurrentQueue2& operator=(ConcurrentQueue2 const&) = delete;

		// A scope object for locking the queue
		// Allows enumeration methods while locked
		// Use:
		//   ConcurrentQueue<Blah> queue;
		//   ...
		//   {
		//      ConcurrentQueue<Blah>::Lock lock(queue);
		//      // use 'lock' like a container
		//   }
		class Lock
		{
			ConcurrentQueue2<T>& m_owner;
			ConcurrentQueueBase::MLock m_lock;

		public:

			std::deque<T>& m_queue;

			explicit Lock(ConcurrentQueue2<T>& queue)
				:m_owner(queue)
				,m_lock(m_owner.m_mutex)
				,m_queue(m_owner.m_queue)
			{}
			Lock(Lock const&) = delete;
			Lock& operator=(Lock const&) = delete;
		};

		// Tests if the LastAdded flag is set and the queue is empty
		bool Exhausted() const
		{
			MLock lock(m_mutex);
			return m_last && m_queue.empty();
		}

		// Call this after the last item has been added to the queue.
		// Queueing anything after 'm_last' has been set throws an exception
		void LastAdded()
		{
			ConcurrentQueueBase::LastAdded();
		}

		// Dequeue blocks until data is available in the queue
		// Returns true if an item was dequeued, or false if not (due to timeout or LastAdded())
		template <typename Pred> bool Dequeue(T& item, MLock& lock, Pred pred, int timeout_ms)
		{
			// Notify before we block. Waiting threads won't see 'm_queue'
			// as empty unless we actually wait (which releases the lock)
			if (m_queue.empty())
				m_cv_empty.notify_all();

			// Wait for an item to dequeue
			if (timeout_ms == ~0)
				m_cv_added.wait(lock, [&]{ return !m_queue.empty() || m_last || pred(); });
			else
				m_cv_added.wait_for(lock, std::chrono::milliseconds(timeout_ms), [&]{ return !m_queue.empty() || m_last || pred(); });

			// Timeout or last added
			if (m_queue.empty())
				return false;

			// Pop the queued item
			item = m_queue.front();
			m_queue.pop_front();

			// Wake any thread blocked in 'Flush()' if this pop just made the queue empty. The
			// notify above only covers the case where 'Dequeue' is entered with an already-empty
			// queue; it doesn't cover the transition to empty caused by this pop, so without this,
			// a 'Flush()' call that observed the queue non-empty and started waiting on
			// 'm_cv_empty' would never be woken.
			if (m_queue.empty())
				m_cv_empty.notify_all();

			return true;
		}
		template <typename Pred> bool Dequeue(T& item, MLock& lock, Pred pred)
		{
			return Dequeue(item, lock, pred, ~0);
		}
		bool Dequeue(T& item, MLock& lock, int timeout_ms)
		{
			return Dequeue(item, lock, []{ return false; }, timeout_ms);
		}
		bool Dequeue(T& item, MLock& lock)
		{
			return Dequeue(item, lock, ~0);
		}
		template <typename Pred> bool Dequeue(T& item, Pred pred, int timeout_ms)
		{
			MLock lock(m_mutex);
			return Dequeue(item, lock, pred, timeout_ms);
		}
		template <typename Pred> bool Dequeue(T& item, Pred pred)
		{
			return Dequeue(item, pred, ~0);
		}
		bool Dequeue(T& item, int timeout_ms)
		{
			MLock lock(m_mutex);
			return Dequeue(item, lock, []{ return false; }, timeout_ms);
		}
		bool Dequeue(T& item)
		{
			return Dequeue(item, []{ return false; }, ~0);
		}

		// Add something to the queue
		void Enqueue(T&& item, MLock&)
		{
			m_queue.push_back(std::move(item));
			m_cv_added.notify_one();
		}
		void Enqueue(T&& item)
		{
			MLock lock(m_mutex);
			Enqueue(std::forward<T>(item), lock);
		}
		void Enqueue(T const& item, MLock&)
		{
			m_queue.push_back(item);
			m_cv_added.notify_one();
		}
		void Enqueue(T const& item)
		{
			MLock lock(m_mutex);
			Enqueue(item, lock);
		}

		// Block until the queue is empty
		// WARNING: don't assume this means the consumer has finished
		// processing the last item removed from the queue.
		void Flush()
		{
			MLock lock(m_mutex);
			m_cv_empty.wait(lock, [&]{ return m_queue.empty(); });
		}

		// Pulse 'm_cv_added' to cause any thread waiting in 'Dequeue' to wake up and test its sleep condition
		void Signal()
		{
			m_cv_added.notify_all();
		}
	};

	// Concurrent queue that provides it's own mutex
	template <typename T> struct ConcurrentQueue :ConcurrentQueue2<T>
	{
		std::mutex m_mutex;
		ConcurrentQueue()
			:ConcurrentQueue2<T>(m_mutex)
		{}
	};
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/common/fmt.h"
#include <string>
#include <algorithm>
#include <future>

namespace pr::threads
{
	namespace unittests::threads
	{
		struct Item
		{
			std::string m_str;
			Item(){}
			Item(char const* name, int idx) :m_str(pr::Fmt("%s%d",name,idx)) {}
		};

		void Produce(char const* name, pr::threads::ConcurrentQueue<Item>& queue)
		{
			for (int i = 0; i != 10; ++i)
				queue.Enqueue(Item(name, i));
		}
		void Consume(pr::threads::ConcurrentQueue<Item>& queue, std::vector<std::string>& items)
		{
			Item item;
			while (queue.Dequeue(item))
				items.push_back(item.m_str);
		}
	}
	PRUnitTest(ConcurrentQueueTests)
	{
		using namespace unittests::threads;

		ConcurrentQueue<Item> queue;
		std::vector<std::string> items;

		std::thread t0(Produce, "t0_", std::ref(queue));
		std::thread t1(Produce, "t1_", std::ref(queue));
		std::thread t2(Produce, "t2_", std::ref(queue));

		t0.join();
		t1.join();
		{
			threads::ConcurrentQueue<Item>::Lock lock(queue);
			auto size = lock.m_queue.size() + items.size();
			PR_EXPECT(size >= 20 && size <= 30); // since t0,t1 have finished
		}

		// Start consuming
		std::thread t3(Consume, std::ref(queue), std::ref(items));

		// Finish adding
		t2.join();
		queue.LastAdded();

		// Finish consuming
		t3.join();

		PR_EXPECT(items.size() == 30U);
		std::sort(begin(items),end(items));
		for (auto i = 0U; i != items.size(); ++i)
			PR_EXPECT(items[i] == std::format("t{}_{}", i/10, i%10));
	}

	// Regression test: a 'Flush()' call that observes the queue non-empty and starts waiting must
	// be woken when a single 'Dequeue()' call pops the last item and empties the queue. Without a
	// notify for that pop-to-empty transition, 'Flush()' would depend on a spurious wakeup and
	// could block forever.
	PRUnitTest(ConcurrentQueueFlushAfterSingleDequeueTests)
	{
		ConcurrentQueue<int> queue;
		queue.Enqueue(42);

		// Hold the queue's mutex directly so the flush thread is guaranteed to block trying to
		// acquire it, then release it only once the flush thread has had time to reach that point.
		// This forces the flush thread to observe the queue as non-empty before the item below is
		// dequeued, reproducing the interleaving that exposes the lost-wakeup bug.
		queue.m_mutex.lock();
		auto flush = std::async(std::launch::async, [&] { queue.Flush(); });
		std::this_thread::sleep_for(std::chrono::milliseconds(50));
		queue.m_mutex.unlock();

		// Give the flush thread time to acquire the now-free mutex, observe the queue as
		// non-empty, and start waiting on the empty-queue condition before dequeuing the only item.
		std::this_thread::sleep_for(std::chrono::milliseconds(50));

		int item = 0;
		PR_EXPECT(queue.Dequeue(item));
		PR_EXPECT(item == 42);

		// 'Flush()' should wake up promptly now that the queue is empty. Bound the wait so a
		// regression fails the test instead of hanging the whole test run.
		auto woke = flush.wait_for(std::chrono::seconds(2)) == std::future_status::ready;
		if (!woke)
		{
			// Unblock the stuck flush thread so the process can still exit cleanly.
			queue.LastAdded();
			flush.wait();
		}
		PR_EXPECT(woke);
	}
}
#endif