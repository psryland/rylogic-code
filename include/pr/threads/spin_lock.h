//*********************************************************************
// Concurrent Queue
//  Copyright (c) Rylogic Ltd 2011
//*********************************************************************

#pragma once
#include <atomic>
#include <thread>
#include <mutex>

#define PR_STACKTRACE 0
#if PR_STACKTRACE
#include <string>
#include <unordered_map>
#include "pr/common/fmt.h"
#include "pr/win32/stackdump.h"
#endif

namespace pr
{
	namespace threads
	{
		// Use with std::lock_guard<SpinLock>. Recursive: lock()/try_lock() succeed immediately when
		// called again by the thread that already owns the lock, and the lock is only released once
		// unlock() has been called once for each successful lock()/try_lock() call on that thread.
		class SpinLock
		{
			std::atomic_bool m_flag;
			std::atomic<std::thread::id> m_owner; // Must be atomic to avoid data race with concurrent lock/unlock
			int m_depth; // Recursion depth. Only ever read or written by the thread that currently owns the lock.
			#if PR_STACKTRACE
			std::string m_stack; // call stack when last locked
			#endif
		
			bool try_lock_internal()
			{
				// exchange() returns the previous value, so the lock is
				// acquired when the previous value was false.
				if (m_flag.exchange(true) != 0)
					return false;

				// Save the lock owner and reset the recursion depth to the outer-most level
				m_owner.store(std::this_thread::get_id());
				m_depth = 1;

				// Record the call stack when the lock is acquired
				#if PR_STACKTRACE
				m_stack.resize(0);
				pr::StackDump([&](std::string sym, std::string file, int line)
				{
					m_stack.append(pr::FmtS("%s(%d): %s\n", file.c_str(), line, sym.c_str()));
				});
				#endif

				return true;
			}

			SpinLock(SpinLock const&);
			SpinLock& operator=(SpinLock const&);

		public:
			SpinLock()
				:m_flag()
				,m_owner()
				,m_depth()
			{}

			void lock()
			{
				// Already locked by this thread? Re-entrant acquisition, so just record the extra level.
				if (std::this_thread::get_id() == m_owner.load())
				{
					++m_depth;
					return;
				}

				// Spin lock. This works because exchange() returns the previous
				// value, the loop exits when the previous value was false.
				for (;!try_lock_internal();)
					std::this_thread::yield();
			}

			bool try_lock()
			{
				// Already locked by this thread? Re-entrant acquisition, so just record the extra level.
				if (std::this_thread::get_id() == m_owner.load())
				{
					++m_depth;
					return true;
				}

				// exchange() returns the previous value, so the lock is
				// acquired when the previous value was false.
				return try_lock_internal();
			}

			void unlock()
			{
				// Only release ownership once every nested lock()/try_lock() call on this thread has
				// been matched by an unlock(), otherwise an inner unlock() would expose an outer
				// critical section that is still in progress on this thread.
				if (--m_depth != 0)
					return;

				m_owner.store(std::thread::id());
				m_flag.exchange(false);
			}
		};
	}
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::threads
{
	namespace unittests::spinlock
	{
		struct Thing
		{
			typedef pr::threads::SpinLock SpinLock;

			SpinLock m_flag;
			int m_count;
			std::atomic_int m_calls;

			Thing()
				:m_flag()
				,m_count()
				,m_calls()
			{}
			void Spam()
			{
				std::lock_guard<SpinLock> lock(m_flag);
				m_count = m_count + 1;
				m_count = m_count - 2;
				m_count = m_count + 1;
				++m_calls;
			}
		};
	}
	PRUnitTest(SpinLockTests)
	{
		using namespace unittests::spinlock;

		Thing thing;
		std::atomic_bool exit(false);

		std::thread thd1([&]
		{
			while (!exit)
				thing.Spam();
		});
		std::thread thd2([&]
		{
			while (!exit)
				thing.Spam();
		});
		std::thread thd3([&]
		{
			while (!exit)
				thing.Spam();
		});

		while (thing.m_calls.load() < 100)
			std::this_thread::sleep_for(std::chrono::milliseconds(10));

		exit.exchange(true);
		thd1.join();
		thd2.join();
		thd3.join();

		PR_EXPECT(thing.m_count == 0);
		PR_EXPECT(thing.m_calls >= 100);
	}
	PRUnitTest(SpinLockRecursionTests)
	{
		// Same-thread re-entrant locking must not release the lock until the outer-most unlock.
		SpinLock lock;
		std::atomic_bool outer_locked(false);
		std::atomic_bool contender_saw_locked(false);
		std::atomic_bool contender_done(false);

		// A second thread that reports whether the lock is still held once the owning
		// thread signals it has completed a nested lock/unlock pair within the outer scope.
		std::thread contender([&]
		{
			while (!outer_locked.load())
				std::this_thread::yield();

			contender_saw_locked.store(!lock.try_lock());
			contender_done.store(true);
		});

		{
			std::lock_guard<SpinLock> outer(lock);
			{
				// Recursive re-entry from the owning thread; this must not release the outer lock.
				std::lock_guard<SpinLock> inner(lock);
			}

			outer_locked.store(true);
			while (!contender_done.load())
				std::this_thread::yield();
		}
		contender.join();

		// The nested unlock must not have exposed the outer critical section.
		PR_EXPECT(contender_saw_locked.load());

		// After the outer scope exits, the lock must be fully released.
		PR_EXPECT(lock.try_lock());
		lock.unlock();
	}
	PRUnitTest(SpinLockRecursionDepthTests)
	{
		// Multiple levels of recursion must all be unwound before the lock is released.
		// try_lock() from the owning thread always succeeds (it is the same recursive re-entry),
		// so ownership is probed here from a second thread instead, for which try_lock() only
		// succeeds once the outer-most owning thread has fully unwound its recursion. Results are
		// captured via atomics and asserted on the main thread; PR_EXPECT throws on failure, and an
		// exception escaping a std::thread function would call std::terminate.
		SpinLock lock;
		lock.lock();
		lock.lock();
		lock.lock();

		lock.unlock();
		lock.unlock();

		std::atomic_bool probe1_locked_out(false);
		std::thread probe1([&] { probe1_locked_out.store(!lock.try_lock()); });
		probe1.join();
		PR_EXPECT(probe1_locked_out.load());

		lock.unlock();

		std::atomic_bool probe2_acquired(false);
		std::thread probe2([&]
		{
			probe2_acquired.store(lock.try_lock());
			if (probe2_acquired.load())
				lock.unlock();
		});
		probe2.join();
		PR_EXPECT(probe2_acquired.load());
	}
}
#endif
