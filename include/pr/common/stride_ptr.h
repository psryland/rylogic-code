//*********************************************
// Value smart pointer
//  Copyright (c) Rylogic Ltd 2020
//*********************************************
// An iterator that steps in 'stride' increments
#pragma once
#include <type_traits>
#include <span>

namespace pr
{
	template <typename Type>
	struct stride_ptr
	{
		// Notes:
		//  - Don't make stride a template parameter, because often the type definition will be
		//    stride_ptr<X const, sizeof(SomeType)>. This requires 'SomeType' to be fully defined making
		//    stride_ptr harder to use in a header.

		using voidptr_t = std::conditional_t<std::is_const_v<Type>, void const*, void*>;
		using byteptr_t = std::conditional_t<std::is_const_v<Type>, unsigned char const*, unsigned char*>;
		using value_type = std::conditional_t<std::is_same_v<Type, void>, unsigned char, Type>;
		
		using pointer = value_type*;
		using reference = value_type&;
		using difference_type = typename std::pointer_traits<pointer>::difference_type;
		using iterator_category = std::random_access_iterator_tag;
		using stride_t = union
		{
			voidptr_t vptr;
			byteptr_t bptr;
			pointer type;
		};

		// Pointer and stride
		stride_t m_ptr;
		int m_stride;

		stride_ptr()
			:m_ptr()
			,m_stride()
		{}
		stride_ptr(voidptr_t ptr, int stride)
			:m_ptr(ptr)
			,m_stride(stride)
		{}
		pointer operator ->() const
		{
			return m_ptr.type;
		}
		reference operator *() const
		{
			return *m_ptr.type;
		}
		stride_ptr& operator ++()
		{
			m_ptr.bptr += m_stride;
			return *this;
		}
		stride_ptr& operator --()
		{
			m_ptr.bptr -= m_stride;
			return *this;
		}
		stride_ptr operator ++(int)
		{
			stride_ptr i(*this);
			m_ptr.bptr += m_stride;
			return i;
		}
		stride_ptr operator --(int)
		{
			stride_ptr i(*this);
			m_ptr.bptr -= m_stride;
			return i;
		}
		reference operator[](difference_type i) const
		{
			return *stride_t(m_ptr.bptr + i * m_stride).type;
		}

		// Implicit conversion to const
		operator stride_ptr<Type const>() const
		{
			return stride_ptr<Type const>(m_ptr.vptr, m_stride);
		}

		// Implicit conversion to normal pointer
		operator pointer () const
		{
			return m_ptr.type;
		}

		// Explicit conversion to bool
		explicit operator bool() const
		{
			return m_ptr.vptr != nullptr;
		}

		// Comparison operators
		friend bool operator == (stride_ptr lhs, stride_ptr rhs)
		{
			return lhs.m_ptr.vptr == rhs.m_ptr.vptr;
		}
		friend bool operator != (stride_ptr lhs, stride_ptr rhs)
		{
			return lhs.m_ptr.vptr != rhs.m_ptr.vptr;
		}
		friend bool operator <  (stride_ptr lhs, stride_ptr rhs)
		{
			return lhs.m_ptr.vptr < rhs.m_ptr.vptr;
		}
		friend bool operator <= (stride_ptr lhs, stride_ptr rhs)
		{
			return lhs.m_ptr.vptr <= rhs.m_ptr.vptr;
		}
		friend bool operator >  (stride_ptr lhs, stride_ptr rhs)
		{
			return lhs.m_ptr.vptr > rhs.m_ptr.vptr;
		}
		friend bool operator >= (stride_ptr lhs, stride_ptr rhs)
		{
			return lhs.m_ptr.vptr >= rhs.m_ptr.vptr;
		}

		// Arithmetic
		friend stride_ptr& operator += (stride_ptr& lhs, difference_type rhs)
		{
			lhs.m_ptr.bptr += rhs * lhs.m_stride;
			return lhs;
		}
		friend stride_ptr& operator -= (stride_ptr& lhs, difference_type rhs)
		{
			lhs.m_ptr.bptr -= rhs * lhs.m_stride;
			return lhs;
		}
		friend stride_ptr operator + (stride_ptr lhs, difference_type rhs)
		{
			auto ptr = lhs;
			return ptr += rhs;
		}
		friend stride_ptr operator - (stride_ptr lhs, difference_type rhs)
		{
			auto ptr = lhs;
			return ptr -= rhs;
		}
		friend difference_type operator - (stride_ptr lhs, stride_ptr rhs)
		{
			if (lhs.m_stride != rhs.m_stride)
				throw std::runtime_error("Stride pointers have different stride values");

			return (lhs.m_ptr.bptr - rhs.m_ptr.bptr) / lhs.m_stride;
		}
	};

	// A range given by stride pointers
	template <typename Type>
	struct stride_range
	{
		using stride_ptr_t = stride_ptr<Type>;
		using pointer = typename stride_ptr_t::pointer;

		stride_ptr_t m_beg;
		stride_ptr_t m_end;

		template <typename T>
		stride_range(std::span<T> span, size_t ofs)
			: m_beg(reinterpret_cast<stride_ptr_t::byteptr_t>(span.data()) + ofs, sizeof(T))
			, m_end(m_beg + span.size())
		{}
		template <typename T, int S>
		stride_range(T (&arr)[S], size_t ofs)
			: m_beg(reinterpret_cast<stride_ptr_t::byteptr_t>(&arr[0]) + ofs, sizeof(T))
			, m_end(m_beg + S)
		{}

		stride_ptr_t begin() const
		{
			return m_beg;
		}
		stride_ptr_t end() const
		{
			return m_end;
		}
	};
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::common
{
	namespace stride_ptr_tests
	{
		using byte_t = unsigned char;
		struct Thing
		{
			int m_int;
			float m_float;
			byte_t m_byte;

			inline static int count = 0;
			
			Thing()
				:m_int(count++)
				,m_float(static_cast<float>(m_int))
				,m_byte(static_cast<byte_t>(m_int & 0xff))
			{}
		};
	}
	PRUnitTest(StridePtrTests, Quick)
	{
		using namespace stride_ptr_tests;
		Thing arr[300];

		byte_t i = 0;
		auto range = stride_range<byte_t>(arr, offsetof(Thing, m_byte));
		for (auto x : range)
		{
			PR_EXPECT(x == i);
			++i;
		}
	}
}
#endif
