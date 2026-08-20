//*****************************************************************
// Async Wrap
//  Copyright (c) Rylogic Ltd 2024
//*****************************************************************
#pragma once
#include <mutex>

namespace pr
{
	template <typename TObject>
	class AsyncWrap
	{
		TObject m_obj;
		mutable std::mutex m_mutex;

	public:

		template <typename TObj>
		class Lock
		{
			std::lock_guard<std::mutex> m_guard;
			TObj& m_obj;

		public:

			Lock(std::mutex& mutex, TObj& obj)
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
	PRUnitTest(AsyncWrapTests, Quick)
	{
		using async_vector_t = AsyncWrap<std::vector<int>>;

		async_vector_t avec;
		if (auto lock = avec.lock())
		{
			auto& vec = lock.get();
			vec.push_back(1);
			vec.push_back(2);
			vec.push_back(3);
		}

		if (auto lock = avec.lock())
		{
			PR_EXPECT(lock.get().size() == 3);
			PR_EXPECT(lock->size() == 3);
		}
	}
}
#endif
