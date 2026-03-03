//*****************************************************************
// Async Wrap
//  Copyright (c) Rylogic Ltd 2024
//*****************************************************************
#pragma once
#include <mutex>

namespace pr
{
	template <typename TObject, typename TMutex = std::mutex>
	class AsyncWrap
	{
		TObject m_obj;
		mutable TMutex m_mutex;

	public:

		template <typename TObj>
		class Lock
		{
			std::lock_guard<TMutex> m_guard;
			TObj& m_obj;

		public:

			Lock(TMutex& mutex, TObj& obj)
				: m_guard(mutex)
				, m_obj(obj)
			{}
			Lock(Lock&&) = default;
			Lock(Lock const&) = delete;
			Lock& operator=(Lock&&) = default;
			Lock& operator=(Lock const&) = delete;

			TObj& get() const
			{
				return m_obj;
			}
			TObj* operator->() const
			{
				return &get();
			}
			TObj& operator*() const
			{
				return get();
			}

			// Allows the 'declared in an if statement' idiom
			explicit operator bool() const
			{
				return true;
			}
		};

		Lock<TObject const> lock() const
		{
			return Lock<TObject const>(m_mutex, m_obj);
		}
		Lock<TObject> lock()
		{
			return Lock<TObject>(m_mutex, m_obj);
		}
	};
}

#if PR_UNITTESTS
#include <vector>
#include "pr/common/unittests.h"
namespace pr::common
{
	PRUnitTest(AsyncWrapTests)
	{
		using async_vector_t = AsyncWrap<std::vector<int>>;

		async_vector_t avec;
		if (auto vec = avec.lock())
		{
			vec->push_back(1);
			vec->push_back(2);
			vec->push_back(3);
		}

		if (auto vec = avec.lock())
		{
			PR_EXPECT(vec.get().size() == 3);
			PR_EXPECT(vec->size() == 3);
		}
	}
}
#endif
