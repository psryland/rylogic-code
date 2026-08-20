//*********************************************
// Value smart pointer
//  Copyright (c) Rylogic Ltd 2020
//*********************************************
#pragma once
#include <memory>

namespace pr
{
	template <typename T, typename Deleter = std::default_delete<T>>
	class value_ptr
	{
		// Notes:
		//  - Simple implementation of a smart pointer with value semantics

	public:

		using pointer = T*;
		using element_type = T;
		using deleter_type = Deleter;

	private:

		T* m_ptr;

	public:

		value_ptr() noexcept
			:m_ptr()
		{
		}
		value_ptr(nullptr_t) noexcept
			:m_ptr()
		{
		}
		explicit value_ptr(T* rhs) noexcept
			:m_ptr(rhs)
		{
		}
		value_ptr(value_ptr&& rhs) noexcept
			:m_ptr(rhs.m_ptr)
		{
			rhs.m_ptr = nullptr;
		}
		value_ptr(value_ptr const& rhs)
			:m_ptr()
		{
			if (rhs.m_ptr)
				m_ptr = new T(*rhs.m_ptr);
		}
		~value_ptr() noexcept
		{
			reset(nullptr);
		}
		value_ptr& operator =(value_ptr&& rhs) noexcept
		{
			if (&rhs == this) return *this;
			std::swap(m_ptr, rhs.m_ptr);
			return *this;
		}
		value_ptr& operator =(value_ptr const& rhs)
		{
			if (&rhs == this) return *this;
			value_ptr p(rhs);
			std::swap(m_ptr, p.m_ptr);
			return *this;
		}
		T& operator*() const noexcept
		{
			return *get();
		}
		T* operator->() const noexcept
		{
			return get();
		}
		void reset(T* ptr) noexcept
		{
			get_deleter()(m_ptr);
			m_ptr = ptr;
		}
		T* get() const noexcept
		{
			return m_ptr;
		}
		T* release() noexcept
		{
			auto p = m_ptr;
			m_ptr = nullptr;
			return p;
		}
		deleter_type get_deleter() const noexcept
		{
			return deleter_type();
		}
		explicit operator bool() const
		{
			return m_ptr != nullptr;
		}

		// Comparisons must inspect the live owned pointer, not copies of the wrapper.
		friend bool operator == (value_ptr const& lhs, value_ptr const& rhs)
		{
			return lhs.m_ptr == rhs.m_ptr;
		}
		friend bool operator != (value_ptr const& lhs, value_ptr const& rhs)
		{
			return lhs.m_ptr != rhs.m_ptr;
		}
		friend bool operator <  (value_ptr const& lhs, value_ptr const& rhs)
		{
			return lhs.m_ptr < rhs.m_ptr;
		}
		friend bool operator <= (value_ptr const& lhs, value_ptr const& rhs)
		{
			return lhs.m_ptr <= rhs.m_ptr;
		}
		friend bool operator >  (value_ptr const& lhs, value_ptr const& rhs)
		{
			return lhs.m_ptr > rhs.m_ptr;
		}
		friend bool operator >= (value_ptr const& lhs, value_ptr const& rhs)
		{
			return lhs.m_ptr >= rhs.m_ptr;
		}
	};
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::common
{
	PRUnitTestClass(TestValuePtr)
	{
		struct Thing
		{
			int m_i;
			Thing(int i) :m_i(i) {}
		};

		// Null and self-comparisons must keep working once comparisons stop copying wrappers.
		PRUnitTestMethod(NullAndReflexive, Quick)
		{
			value_ptr<Thing> empty;
			PR_EXPECT(empty == nullptr);
			PR_EXPECT(!(empty != nullptr));
			PR_EXPECT(empty <= empty);
			PR_EXPECT(empty >= empty);
			PR_EXPECT(!(empty < empty));
			PR_EXPECT(!(empty > empty));

			value_ptr<Thing> value(new Thing(1));
			PR_EXPECT(value == value);
			PR_EXPECT(!(value != value));
			PR_EXPECT(value <= value);
			PR_EXPECT(value >= value);
			PR_EXPECT(!(value < value));
			PR_EXPECT(!(value > value));
		}

		// Distinct pointers only have a defined ordering when they refer to the same array.
		PRUnitTestMethod(DistinctAndOrdering, Quick)
		{
			// Keep the pointers non-owning so the ordering check can use stack storage in one array.
			struct NoDelete
			{
				void operator()(Thing*) const noexcept
				{
				}
			};

			Thing storage[2] =
			{
				Thing(2),
				Thing(3),
			};
			value_ptr<Thing, NoDelete> left(&storage[0]);
			value_ptr<Thing, NoDelete> right(&storage[1]);
			PR_EXPECT(left != right);
			PR_EXPECT(!(left == right));
			PR_EXPECT(left < right);
			PR_EXPECT(right > left);
			PR_EXPECT(left <= right);
			PR_EXPECT(right >= left);
		}

		// Copying must clone the pointee while keeping identity-based comparisons stable.
		PRUnitTestMethod(CopyIsolation, Quick)
		{
			value_ptr<Thing> value(new Thing(4));
			auto copied = value;
			PR_EXPECT(copied != value);
			PR_EXPECT(copied->m_i == value->m_i);
			PR_EXPECT(value == value);
			PR_EXPECT(copied == copied);

			copied->m_i = 9;
			PR_EXPECT(value->m_i == 4);
			PR_EXPECT(copied->m_i == 9);
		}
	};
}
#endif