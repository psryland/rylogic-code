//******************************************
// byte_data
//  Copyright (c) Oct 2003 Paul Ryland
//******************************************
#pragma once
#include <cstdint>
#include <cstring>
#include <vector>
#include <span>
#include <concepts>
#include <initializer_list>
#include <sstream>
#include <stdexcept>
#include <malloc.h>
#include <intrin.h>

namespace pr
{
	// A dynamically allocating container of bytes with alignment.
	// Loosely like 'std::vector<unsigned char>' except the buffer is aligned.
	// Note: 'Type' is not a class template parameter here so that the contents
	// of the container can be interpreted as different types as needed.
	// Also note: this container is intended as a byte bucket, so don't expect
	// constructors/destructors/etc to be called on types you add to this container.
	template <int Alignment = 4> struct byte_data
	{
		using value_type = std::byte;
		union
		{
			value_type* m_ptr;

			// Debugging helper pointers
			char*    m_str;
			wchar_t* m_wstr;
			uint8_t  (*m_u8)[128];
			uint16_t (*m_u16)[64];
			uint32_t (*m_u32)[32];
			uint64_t (*m_u64)[16];
		};
		size_t m_size;
		size_t m_capacity;

		byte_data()
			: m_ptr(nullptr)
			, m_size(0)
			, m_capacity(0)
		{
		}
		byte_data(byte_data&& rhs) noexcept
			: m_ptr(rhs.m_ptr)
			, m_size(rhs.m_size)
			, m_capacity(rhs.m_capacity)
		{
			rhs.m_ptr = nullptr;
			rhs.m_size = 0;
			rhs.m_capacity = 0;
		}
		byte_data(byte_data const& rhs)
			: m_ptr(nullptr)
			, m_size(0)
			, m_capacity(0)
		{
			set_capacity(rhs.m_capacity);
			if (m_ptr && rhs.m_ptr) memcpy(m_ptr, rhs.m_ptr, rhs.m_size);
			m_size = rhs.m_size;
		}
		explicit byte_data(size_t initial_size_in_bytes)
			:byte_data()
		{
			resize(initial_size_in_bytes);
		}
		byte_data(std::span<std::byte const> data)
			:byte_data()
		{
			append(data);
		}
		template <typename Type> requires std::is_trivially_copyable_v<Type>
		byte_data(std::span<Type const> data)
			:byte_data()
		{
			append(data);
		}
		template <typename citer, typename Type = std::iterator_traits<citer>::value_type>
		byte_data(citer first, citer last)
			:byte_data()
		{
			// Example use:
			//'   std::ifstream infile(filepath, std::ios::binary);
			//'   buf = byte_data{std::istreambuf_iterator<char>(infile), std::istreambuf_iterator<char>()};
			static_assert(std::is_trivially_copyable_v<Type>);
			for (; first != last; ++first)
				push_back<Type>(*first);
		}
		~byte_data()
		{
			set_capacity(0);
		}

		byte_data& operator = (byte_data&& rhs) noexcept
		{
			if (this != &rhs)
			{
				std::swap(m_ptr, rhs.m_ptr);
				std::swap(m_size, rhs.m_size);
				std::swap(m_capacity, rhs.m_capacity);
			}
			return *this;
		}
		byte_data& operator = (byte_data const& rhs)
		{
			if (this != &rhs)
			{
				set_capacity(rhs.m_capacity);
				memcpy(m_ptr, rhs.m_ptr, rhs.m_size);
				m_size = rhs.m_size;
			}
			return *this;
		}

		// Release memory the container has allocated.
		void clear()
		{
			set_capacity(0);
		}

		// Return true if the container is empty
		bool empty() const
		{
			return m_size == 0;
		}

		// Return the size of the data in the container in multiples of 'Type'
		template <typename Type> size_t size() const
		{
			return m_size / sizeof(Type);
		}
		size_t size() const
		{
			return m_size;
		}

		// The reserved space
		template <typename Type> size_t capacity() const
		{
			return m_capacity / sizeof(Type);
		}
		size_t capacity() const
		{
			return m_capacity;
		}

		// Resize the container to contain 'new_size' multiples of 'Type'
		template <typename Type> void resize(size_t new_size)
		{
			static_assert(std::is_trivially_copyable_v<Type>);
			resize(new_size * sizeof(Type));
		}
		template <typename Type> void resize(size_t new_size, Type fill)
		{
			static_assert(std::is_trivially_copyable_v<Type>);
			auto n = std::max<ptrdiff_t>(0, new_size - size<Type>());
			resize<Type>(new_size);
			for (auto ptr = end<Type>() - n; ptr != end<Type>(); ++ptr)
				memcpy(ptr, &fill, sizeof(Type));
		}
		void resize(size_t new_size)
		{
			ensure_capacity(new_size);
			m_size = new_size;
		}
		void resize(size_t new_size, value_type fill)
		{
			auto n = std::max<ptrdiff_t>(0, new_size - size());
			resize(new_size);
			memset(end() - n, static_cast<int>(fill), n);
		}

		// Pre-allocate space
		template <typename Type> void reserve(size_t new_capacity)
		{
			static_assert(std::is_trivially_copyable_v<Type>);
			reserve(new_capacity * sizeof(Type));
		}
		void reserve(size_t new_capacity)
		{
			// Don't use ensure_capacity because the caller is specifically setting the capacity
			if (new_capacity > m_capacity)
				set_capacity(new_capacity);
		}

		// Grow the container so that the size is a multiple of 'alignment'
		void pad_to(int alignment)
		{
			auto rem = size() % alignment;
			if (rem != 0)
				resize(size() + (alignment - rem));
		}

		// Add 'type' to the end of the container
		byte_data& push_back(std::span<value_type const> data)
		{
			// Copying from a sub range, allow for invalidation on resize
			if (inside(data.data()))
			{
				auto ofs = data.data() - begin();

				ensure_capacity(m_size + data.size());
				memcpy(m_ptr + m_size, m_ptr + ofs, data.size());
				m_size += data.size();
			}
			else
			{
				ensure_capacity(m_size + data.size());
				memcpy(m_ptr + m_size, data.data(), data.size());
				m_size += data.size();
			}
			return *this;
		}
		byte_data& push_back(std::string_view str)
		{
			return push_back({ byte_ptr(str.data()), str.size() });
		}
		template <typename Type> requires std::is_trivially_copyable_v<Type> byte_data& push_back(Type const& type)
		{
			// No 'std::span<Type const>' overload because it's ambiguous with this method. Use insert or append
			return push_back({ byte_ptr(&type), sizeof(Type) });
		}
		template <typename Type> requires std::is_trivially_copyable_v<Type> byte_data& push_back()
		{
			return push_back<Type>(Type{});
		}

		// Append the contents of 'rhs' to this container
		byte_data& append(byte_data const& rhs)
		{
			push_back(rhs);
			return *this;
		}
		byte_data& append(std::span<value_type const> data)
		{
			push_back(data);
			return *this;
		}
		template <typename Type> requires std::is_trivially_copyable_v<Type> byte_data& append(std::span<Type const> data)
		{
			// No overload for 'Type const&' because it's ambiguous with this method
			push_back({ byte_ptr(data.data()), data.size() * sizeof(Type) });
			return *this;
		}
		template <typename Type> requires std::is_trivially_copyable_v<Type> byte_data& append(std::initializer_list<Type const> data)
		{
			auto count = static_cast<size_t>(data.end() - data.begin());
			push_back({ byte_ptr(data.begin()), count * sizeof(Type) });
			return *this;
		}

		// Insert bytes into this container
		void insert(ptrdiff_t ofs, std::span<value_type const> data)
		{
			// Notes:
			//  Cannot add a template overload for inserting an array of 'Type' because
			//  the method signature is ambiguous.
			if (ofs < 0 || ofs > s_cast<ptrdiff_t>(m_size))
				throw std::out_of_range("Offset position out of range");

			// Insert from a sub range
			if (inside(data.data()))
			{
				// The sub range may span the insertion point
				auto b = data.data() - begin(); // The offset to the start of the sub range
				auto n = std::clamp<ptrdiff_t>(ofs - b, 0, data.size()); // The number of bytes before the insertion point

				make_hole(ofs, data.size());

				// Copy bytes from before and after the insertion point
				auto ins = m_ptr + ofs;
				if (n != 0)
					memcpy(ins, m_ptr + b, n);
				if (n != static_cast<ptrdiff_t>(data.size()))
					memcpy(ins + n, ins + data.size(), data.size() - n);
			}
			else
			{
				make_hole(ofs, data.size());

				// Insert the bytes from 'data'
				auto ins = m_ptr + ofs;
				memcpy(ins, data.data(), data.size());
			}
			m_size += data.size();
		}
		template <typename Type> requires std::is_trivially_copyable_v<Type> void insert(ptrdiff_t ofs, std::span<Type const> data)
		{
			insert(ofs * sizeof(Type), { byte_ptr(data.data()), data.size() * sizeof(Type) });
		}

		// Overwrite bytes at 'ofs'
		void overwrite(ptrdiff_t ofs, std::span<value_type const> data)
		{
			if (ofs < 0 || ofs > s_cast<ptrdiff_t>(m_size))
				throw std::out_of_range("Offset position out of range");

			// Overwrite from a sub range
			if (inside(data.data()))
			{
				// Record the offset into the buffer before resizing
				auto b = data.data() - begin();
				if (size() - ofs < data.size())
					resize(ofs + data.size());

				memmove(m_ptr + ofs, m_ptr + b, data.size());
			}
			else
			{
				if (size() - ofs < data.size())
					resize(ofs + data.size());
			
				memcpy(m_ptr + ofs, data.data(), data.size());
			}
		}

		// Return a pointer to the start of the contained range
		template <typename Type> Type const* begin() const
		{
			return reinterpret_cast<Type const*>(m_ptr);
		}
		template <typename Type> Type* begin()
		{
			return reinterpret_cast<Type*>(m_ptr);
		}
		value_type const* begin() const
		{
			return reinterpret_cast<value_type const*>(m_ptr);
		}
		value_type* begin()
		{
			return reinterpret_cast<value_type*>(m_ptr);
		}

		// Return a pointer to the end of the contained range
		template <typename Type> Type const* end() const
		{
			return begin<Type>() + size<Type>(); // note: end() - begin() is always an integer multiple of 'sizeof(Type)'
		}
		template <typename Type> Type* end()
		{
			return begin<Type>() + size<Type>(); // note: end() - begin() is always an integer multiple of 'sizeof(Type)'
		}
		value_type const* end() const
		{
			return begin() + size();
		}
		value_type* end()
		{
			return begin() + size();
		}

		// Return a const pointer to the start of the contained range
		template <typename Type> Type const* cbegin() const
		{
			return reinterpret_cast<Type const*>(m_ptr);
		}
		value_type const* cbegin() const
		{
			return reinterpret_cast<value_type const*>(m_ptr);
		}

		// Return a const pointer to the end of the contained range
		template <typename Type> Type const* cend() const
		{
			return cbegin<Type>() + size<Type>(); // note: end() - begin() is always an integer multiple of 'Type'
		}
		value_type const* cend() const
		{
			return cbegin() + size();
		}

		// Return the buffer interpreted as 'Type'
		template <typename Type> Type const& as() const
		{
			return *checked_ptr<Type>(0);
		}
		value_type const& as() const
		{
			return *begin();
		}
		template <typename Type> Type& as()
		{
			return *checked_ptr<Type>(0);
		}
		value_type& as()
		{
			return *begin();
		}

		// Indexed addressing in the buffer interpreted as an array of 'Type'
		template <typename Type> Type const& at(size_t index) const
		{
			if (index >= size<Type>())
				throw std::out_of_range("Index out of range");

			return *checked_ptr<Type>(index * sizeof(Type));
		}
		value_type const& at(size_t index) const
		{
			return begin()[index];
		}
		template <typename Type> Type& at(size_t index)
		{
			if (index >= size<Type>())
				throw std::out_of_range("Index out of range");

			return *checked_ptr<Type>(index * sizeof(Type));
		}
		value_type& at(size_t index)
		{
			return begin()[index];
		}

		// Return a reference to 'Type' at a byte offset into the container.
		// The offset must land on an address that satisfies alignof(Type).
		template <typename Type> Type const& at_byte_ofs(size_t byte_ofs) const
		{
			return *checked_ptr<Type>(byte_ofs);
		}
		template <typename Type> Type& at_byte_ofs(size_t byte_ofs)
		{
			return *checked_ptr<Type>(byte_ofs);
		}

		// Array access to bytes
		value_type const& operator[](size_t index) const
		{
			return at(index);
		}
		value_type& operator[](size_t index)
		{
			return at(index);
		}

		// A pointer to the data
		template <typename Type> Type const* data() const
		{
			return begin<Type>();
		}
		template <typename Type> Type* data()
		{
			return begin<Type>();
		}
		value_type const* data() const
		{
			return begin();
		}
		value_type* data()
		{
			return begin();
		}

		// A typed pointer to the data starting at the given byte offset.
		// This applies the same bounds and alignment checks as 'at_byte_ofs'.
		template <typename Type> Type const* data_at(size_t byte_ofs) const
		{
			return checked_ptr<Type>(byte_ofs);
		}
		template <typename Type> Type* data_at(size_t byte_ofs)
		{
			return checked_ptr<Type>(byte_ofs);
		}

		// Façade access
		std::span<value_type const> span() const
		{
			return { m_ptr, m_size };
		}
		std::span<value_type> span()
		{
			return { m_ptr, m_size };
		}
		template <typename Type> std::span<Type const> span() const
		{
			return std::span<Type const>(data<Type>(), size<Type>());
		}
		template <typename Type> std::span<Type> span()
		{
			return std::span<Type>(data<Type>(), size<Type>());
		}

		// Streaming access
		template <typename Type> Type read(size_t& ofs) const
		{
			static_assert(std::is_trivially_copyable_v<Type>);
			if (ofs + sizeof(Type) > size())
				throw std::out_of_range("read attempt beyond buffer end");

			Type value{};
			std::memcpy(&value, m_ptr + ofs, sizeof(Type));
			ofs += sizeof(Type);
			return value;
		}

		// implicit conversions
		template <typename Type> operator std::span<Type const>() const
		{
			return span<Type const>();
		}
		operator std::span<value_type const>() const
		{
			return span<std::byte const>();
		}

	private:

		// Resolve a typed access after validating the byte range and the target type's alignment.
		// Byte-oriented callers should stay on the raw byte APIs when they do not need typed access.
		template <typename Type> Type const* checked_ptr(size_t byte_ofs) const
		{
			if (byte_ofs > m_size || sizeof(Type) > m_size - byte_ofs)
				throw std::out_of_range("Offset position out of range");

			auto ptr = m_ptr + byte_ofs;
			if ((reinterpret_cast<std::uintptr_t>(ptr) % alignof(Type)) != 0)
				throw std::runtime_error("Type access is misaligned");

			return reinterpret_cast<Type const*>(ptr);
		}
		template <typename Type> Type* checked_ptr(size_t byte_ofs)
		{
			return const_cast<Type*>(static_cast<byte_data const&>(*this).template checked_ptr<Type>(byte_ofs));
		}

		// Convert a void pointer to a byte pointer
		static value_type const* byte_ptr(void const* ptr)
		{
			return static_cast<value_type const*>(ptr);
		}
		static value_type* byte_ptr(void* ptr)
		{
			return static_cast<value_type*>(ptr);
		}

		// Test a pointer for being within the data range of this container
		bool inside(value_type const* ptr) const
		{
			// A block of memory should not partially span our internal allocation.
			// That would be undefined behaviour
			return ptr >= begin() && ptr < end();
		}

		// Make a hole at 'ofs' of size 'size'
		void make_hole(ptrdiff_t ofs, size_t size)
		{
			ensure_capacity(m_size + size);
			auto ins = m_ptr + ofs;        // The insertion point of the hole
			auto rem = m_size - ofs;       // The number of remaining bytes after 'ofs'
			memmove(ins + size, ins, rem); // Move the bytes after the insertion point to the end
		}

		// Grow the allocation to at least 'required_size'
		void ensure_capacity(size_t required_size)
		{
			if (m_capacity >= required_size) return;
			set_capacity(required_size * 3 / 2);
		}

		// Grow/Shrink the allocation size of the container
		void set_capacity(size_t capacity)
		{
			static_assert(((Alignment - 1) & Alignment) == 0, "Alignment should be a power of two");
			
			// Round up to the alignment size.
			// Setting the capacity smaller than the size, truncates the data.
			auto new_capacity = pad(capacity);
			auto new_size = std::min(m_size, capacity);
			if (m_capacity == new_capacity)
				return;

			// Allocate a new buffer with the desired capacity
			value_type* ptr = nullptr;
			if (new_capacity != 0)
			{
				ptr = static_cast<value_type*>(_aligned_malloc(new_capacity, Alignment));
				if (ptr == nullptr)
					throw std::bad_alloc();

				// Copy the current buffer contents to the new location
				if (m_ptr != nullptr)
					memcpy(ptr, m_ptr, new_size);
			}

			// Release the old buffer
			if (m_ptr != nullptr)
				_aligned_free(m_ptr);

			// Point to the new buffer
			m_capacity = new_capacity;
			m_size = new_size;
			m_ptr = ptr;
		}

		// Pad 'n' out to an alignment boundary
		static size_t pad(size_t n)
		{
			static_assert(((Alignment - 1) & Alignment) == 0, "Alignment should be a power of two");
			return n + (~(n - 1) & (Alignment - 1));
		}
	};

	// An pointer for moving over bytes interpreted as other types
	struct byte_data_cptr
	{
		using value_type = std::byte;
		value_type const* m_beg;
		value_type const* m_end;
		
		byte_data_cptr()
			: byte_data_cptr(std::span<value_type>{})
		{}
		template <int N> byte_data_cptr(value_type const (&data)[N])
			: byte_data_cptr(&data[0], &data[0] + N)
		{}
		byte_data_cptr(std::span<value_type const> data)
			: m_beg(data.data())
			, m_end(data.data() + data.size())
		{}

		// Interpret the current 'm_beg' pointer as 'Type'
		template <typename Type> Type const& as() const
		{
			auto size = m_beg == nullptr ? size_t(0) : static_cast<size_t>(m_end - m_beg);
			if (sizeof(Type) > size)
				throw std::out_of_range("buffer overrun");

			if ((reinterpret_cast<std::uintptr_t>(m_beg) % alignof(Type)) != 0)
				throw std::runtime_error("Type access is misaligned");

			return *reinterpret_cast<Type const*>(m_beg);
		}
		
		// Advance the pointer by the size of 'Type'
		template <typename Type> Type read(int count = 1)
		{
			static_assert(std::is_trivially_copyable_v<Type>);
			auto size = m_beg == nullptr ? size_t(0) : static_cast<size_t>(m_end - m_beg);
			if (count < 0 || size < static_cast<size_t>(count) * sizeof(Type))
				throw std::out_of_range("buffer overrun");

			Type value{};
			std::memcpy(&value, m_beg, sizeof(Type));
			m_beg += sizeof(Type);
			return value;
		}

		// Read bytes to align to 'alignment'
		void align_to(int alignment)
		{
			auto rem = (m_beg - (value_type const*)0) % alignment;
			if (rem != 0)
				read<std::byte>(static_cast<int>(alignment - rem));
		}

		// True while there is data left
		explicit operator bool() const
		{
			return m_beg != m_end;
		}
	};
	struct byte_data_mptr
	{
		using value_type = std::byte;
		value_type* m_beg;
		value_type* m_end;
		
		byte_data_mptr()
			:byte_data_mptr(std::span<value_type>{})
		{}
		template <int N> byte_data_mptr(value_type (&data)[N])
			: byte_data_mptr(&data[0], &data[0] + N)
		{}
		byte_data_mptr(std::span<value_type> data)
			: m_beg(data.data())
			, m_end(data.data() + data.size())
		{}

		// Interpret the current 'm_beg' pointer as 'Type'
		template <typename Type> Type& as() const
		{
			auto size = m_beg == nullptr ? size_t(0) : static_cast<size_t>(m_end - m_beg);
			if (sizeof(Type) > size)
				throw std::out_of_range("buffer overrun");

			if ((reinterpret_cast<std::uintptr_t>(m_beg) % alignof(Type)) != 0)
				throw std::runtime_error("Type access is misaligned");

			return *reinterpret_cast<Type*>(m_beg);
		}
		
		// Advance the pointer by the size of 'Type'
		template <typename Type> void write(Type const& value, int count = 1)
		{
			static_assert(std::is_trivially_copyable_v<Type>);
			auto size = m_beg == nullptr ? size_t(0) : static_cast<size_t>(m_end - m_beg);
			if (count < 0 || size < static_cast<size_t>(count) * sizeof(Type))
				throw std::out_of_range("buffer overrun");

			for (; count-- != 0;)
			{
				std::memcpy(m_beg, &value, sizeof(Type));
				m_beg += sizeof(Type);
			}
		}

		// Read bytes to align to 'alignment'
		void align_to(int alignment)
		{
			auto rem = (m_beg - (value_type const*)0) % alignment;
			if (rem != 0)
				write<std::byte>({}, static_cast<int>(alignment - rem));
		}

		// Convertible to const
		operator byte_data_cptr() const
		{
			return byte_data_cptr({ m_beg, static_cast<size_t>(m_end - m_beg) });
		}
	};

	// Handy typedef of a vector of bytes
	using bytes_t = std::vector<std::byte>;

	// Append data to a byte container
	template <typename T> inline bytes_t& AppendData(bytes_t& data, const T& object)
	{
		data.insert(data.end(), reinterpret_cast<std::byte const*>(&object), reinterpret_cast<std::byte const*>(&object + 1));
		return data;
	}
	template <> inline bytes_t& AppendData(bytes_t& data, bytes_t const& more_data)
	{
		data.insert(data.end(), more_data.begin(), more_data.end());
		return data;
	}
	inline bytes_t& AppendData(bytes_t& data, std::span<std::byte const> more_data)
	{
		data.insert(std::end(data), more_data.begin(), more_data.end());
		return data;
	}

	// Deprecated names
	template <int Alignment = 4> using ByteData = byte_data<Alignment>;
	using ByteDataCPtr = byte_data_cptr;
	using ByteDataMPtr = byte_data_mptr;
	using ByteCont = bytes_t;
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::container
{
	PRUnitTest(ByteDataTests)
	{
		{ // Constructors
			byte_data buf0;
			PR_EXPECT(buf0.data() == nullptr);
			PR_EXPECT(buf0.capacity() == 0U);
			PR_EXPECT(buf0.size() == 0U);

			byte_data buf1;
			buf1.push_back(0);
			buf1.push_back(1);
			buf1.push_back(2);
			PR_EXPECT(buf1.capacity<int>() >= 3U);
			PR_EXPECT(buf1.size<int>() == 3U);
			PR_EXPECT(buf1.at<int>(0) == 0);
			PR_EXPECT(buf1.at<int>(1) == 1);
			PR_EXPECT(buf1.at<int>(2) == 2);

			// Move constructor
			byte_data buf2(std::move(buf1));
			PR_EXPECT(buf2.capacity<int>() >= 3U);
			PR_EXPECT(buf2.size<int>() == 3U);
			PR_EXPECT(buf2.at<int>(0) == 0);
			PR_EXPECT(buf2.at<int>(1) == 1);
			PR_EXPECT(buf2.at<int>(2) == 2);

			// Copy constructor
			byte_data buf3(buf2);
			PR_EXPECT(buf2.capacity<int>() >= 3U);
			PR_EXPECT(buf3.capacity<int>() == buf2.capacity<int>());
			PR_EXPECT(buf3.size<int>() == buf2.size<int>());
			PR_EXPECT(buf3.at<int>(0) == buf2.at<int>(0));
			PR_EXPECT(buf3.at<int>(1) == buf2.at<int>(1));
			PR_EXPECT(buf3.at<int>(2) == buf2.at<int>(2));
		}
		{ // Assignment
			byte_data buf0;
			buf0.push_back<short>(0);
			buf0.push_back<short>(1);
			buf0.push_back<short>(2);
			PR_EXPECT(buf0.capacity<short>() >= 3U);
			PR_EXPECT(buf0.size<short>() == 3U);

			byte_data buf1;
			buf1 = buf0;
			PR_EXPECT(buf1.capacity<short>() >= 3U);
			PR_EXPECT(buf1.size<short>() == 3U);
			PR_EXPECT(buf1.at<short>(0) == 0);
			PR_EXPECT(buf1.at<short>(1) == 1);
			PR_EXPECT(buf1.at<short>(2) == 2);

			byte_data buf2;
			buf2 = std::move(buf0);
			PR_EXPECT(buf2.capacity<short>() >= 3U);
			PR_EXPECT(buf2.size<short>() == 3U);
			PR_EXPECT(buf2.at<short>(0) == 0);
			PR_EXPECT(buf2.at<short>(1) == 1);
			PR_EXPECT(buf2.at<short>(2) == 2);
		}
		{ // Methods
			byte_data buf0;
			buf0.push_back<char>('A');
			buf0.push_back<short>(0x5555);
			buf0.push_back<char>('B');
			buf0.push_back<int>(42);

			// size/capacity
			PR_EXPECT(buf0.capacity() >= 8U);
			PR_EXPECT(buf0.size() == 8U);

			// clear
			byte_data buf1 = buf0;
			PR_EXPECT(buf1.capacity() == buf0.capacity());
			PR_EXPECT(buf1.size() == buf0.size());
			buf1.clear();
			PR_EXPECT(buf1.capacity() == 0U);
			PR_EXPECT(buf1.size() == 0U);

			// empty
			PR_EXPECT(buf1.empty());

			// resize
			buf1 = buf0;
			buf1.resize<int>(1);
			PR_EXPECT(buf1.capacity() >= 4U);
			PR_EXPECT(buf1.size() == 4U);
			PR_EXPECT(buf1.at<char>(0) == 'A');
			PR_EXPECT(buf1.at<char>(1) == 0x55);
			PR_EXPECT(buf1.at<char>(2) == 0x55);
			PR_EXPECT(buf1.at<char>(3) == 'B');
			buf1.resize<char>(3);
			PR_EXPECT(buf1.capacity() >= 3U);
			PR_EXPECT(buf1.size() == 3U);
			PR_EXPECT(buf1.at<char>(0) == 'A');
			PR_EXPECT(buf1.at<char>(1) == 0x55);
			PR_EXPECT(buf1.at<char>(2) == 0x55);
			buf1.resize<char>(8, static_cast<char>(0xAA));
			PR_EXPECT(buf1.capacity() >= 8U);
			PR_EXPECT(buf1.size() == 8U);
			PR_EXPECT(buf1.at<char>(0) == 'A');
			PR_EXPECT(buf1.at<char>(1) == static_cast<char>(0x55));
			PR_EXPECT(buf1.at<char>(2) == static_cast<char>(0x55));
			PR_EXPECT(buf1.at<char>(3) == static_cast<char>(0xAA));
			PR_EXPECT(buf1.at<char>(4) == static_cast<char>(0xAA));
			PR_EXPECT(buf1.at<char>(5) == static_cast<char>(0xAA));
			PR_EXPECT(buf1.at<char>(6) == static_cast<char>(0xAA));
			PR_EXPECT(buf1.at<char>(7) == static_cast<char>(0xAA));

			// reserve
			buf1.reserve(16);
			PR_EXPECT(buf1.capacity() >= 16U);
			buf1.reserve<int>(16);
			PR_EXPECT(buf1.capacity<int>() >= 16U);
			PR_EXPECT(buf1.capacity() >= 16U * sizeof(int));

			// align_to
			buf1.resize(21);
			PR_EXPECT(buf1.size() == 21);
			buf1.pad_to(16);
			PR_EXPECT(buf1.size() == 32);
			buf1.resize(0);
			buf1.pad_to(4);
			PR_EXPECT(buf1.size() == 0);

			// append
			buf1 = buf0;
			buf1.append(buf0);
			PR_EXPECT(buf1.size() == 2 * buf0.size());
			for (int i = 0, iend = int(buf1.size()); i != iend; ++i)
				PR_EXPECT(buf1[i] == buf0[i % buf0.size()]);
			buf1.clear();
			buf1.append({ 0, 1, 2, 3 });
			PR_EXPECT(buf1.size<int>() == 4U);
			PR_EXPECT(buf1.at<int>(0) == 0);
			PR_EXPECT(buf1.at<int>(1) == 1);
			PR_EXPECT(buf1.at<int>(2) == 2);
			PR_EXPECT(buf1.at<int>(3) == 3);
			buf1.clear();
			buf1.append(buf0.span().subspan(4, 4));
			PR_EXPECT(buf1.size<int>() == 1U);
			PR_EXPECT(buf1.at<uint8_t>(0) == 42);
			PR_EXPECT(buf1.at<uint8_t>(1) == 0);
			PR_EXPECT(buf1.at<uint8_t>(2) == 0);
			PR_EXPECT(buf1.at<uint8_t>(3) == 0);
			PR_EXPECT(buf1.at<int>(0) == 42);
			
			// insert
			buf1 = buf0;
			buf1.insert(1, buf0);
			PR_EXPECT(buf1.size() == 2 * buf0.size());
			PR_EXPECT(memcmp(&buf1[0], &buf0[0], 1) == 0);
			PR_EXPECT(memcmp(&buf1[1], &buf0[0], buf0.size()) == 0);
			PR_EXPECT(memcmp(&buf1[1+buf0.size()], &buf0[1], buf0.size()-1) == 0);
			buf1 = buf0;
			buf1.insert(1, { buf1.data(), 3 }); // subrange insert
			PR_EXPECT(buf1.size() == buf0.size() + 3U);
			PR_EXPECT(buf1[0] == buf0[0]);
			PR_EXPECT(buf1[1] == buf0[0]);
			PR_EXPECT(buf1[2] == buf0[1]);
			PR_EXPECT(buf1[3] == buf0[2]);
			PR_EXPECT(buf1[4] == buf0[1]);
			PR_EXPECT(buf1[5] == buf0[2]);

			// push_back
			buf1 = buf0;
			buf1.push_back<int>(123);
			PR_EXPECT(buf1.at<int>(0) == 0x42555541);
			PR_EXPECT(buf1.at<int>(1) == 42);
			PR_EXPECT(buf1.at<int>(2) == 123);
			buf1.push_back({ buf1.data(), 4 });
			PR_EXPECT(buf1.at<int>(3) == 0x42555541);

			// Overwrite
			buf1 = buf0;
			buf1.overwrite(2, buf0.span().subspan(0, 8));
			PR_EXPECT(buf1.size() == 2 + buf0.size());
			for (int i = 0; i != 2; ++i) { PR_EXPECT(buf1[i + 0] == buf0[i]); }
			for (int i = 0; i != isize(buf0); ++i) { PR_EXPECT(buf1[i + 2] == buf0[i]); }
			buf1 = buf0;
			buf1.overwrite(6, buf1.span().subspan(0, 4)); // subrange overwrite
			PR_EXPECT(buf1.size() == 10);
			for (int i = 0; i != 6; ++i) { PR_EXPECT(buf1[i + 0] == buf0[i]); }
			for (int i = 0; i != 4; ++i) { PR_EXPECT(buf1[i + 6] == buf0[i]); }
		}
		{ // Access
			struct alignas(64) OverAligned
			{
				std::uint64_t m_value;
			};

			byte_data buf0;
			buf0.append({0, 1, 2, 3});
			PR_EXPECT(buf0.size<int>() == 4U);
			
			// as
			auto& arr0 = buf0.as<int[4]>();
			PR_EXPECT(arr0[0] == 0);
			PR_EXPECT(arr0[1] == 1);
			PR_EXPECT(arr0[2] == 2);
			PR_EXPECT(arr0[3] == 3);

			// at_byte_ofs
			auto v1 = buf0.at_byte_ofs<int>(0);
			PR_EXPECT(v1 == 0);

			// const typed access
			auto const& cbuf0 = buf0;
			auto const v2 = cbuf0.template at_byte_ofs<int>(0);
			PR_EXPECT(v2 == 0);

			auto s = buf0.span<int>();
			PR_EXPECT(s.size() == 4U);
			PR_EXPECT(s[0] == 0);
			PR_EXPECT(s[1] == 1);
			PR_EXPECT(s[2] == 2);
			PR_EXPECT(s[3] == 3);

			// over-aligned access
			byte_data<64> buf2;
			buf2.push_back(OverAligned{ 0x1122334455667788ULL });
			PR_EXPECT((reinterpret_cast<std::uintptr_t>(buf2.data()) % alignof(OverAligned)) == 0);
			PR_EXPECT(buf2.at_byte_ofs<OverAligned>(0).m_value == 0x1122334455667788ULL);
			PR_EXPECT(buf2.data_at<OverAligned>(0)->m_value == 0x1122334455667788ULL);

			auto& aligned_ref = buf2.at_byte_ofs<OverAligned>(0);
			aligned_ref.m_value = 0x8877665544332211ULL;
			PR_EXPECT(buf2.at_byte_ofs<OverAligned>(0).m_value == 0x8877665544332211ULL);

			auto const& cbuf2 = buf2;
			PR_EXPECT(cbuf2.at_byte_ofs<OverAligned>(0).m_value == 0x8877665544332211ULL);
			PR_EXPECT(cbuf2.data_at<OverAligned>(0)->m_value == 0x8877665544332211ULL);

			// Check the two failure modes independently so the test documents both the size and alignment contracts.
			bool threw = false;
			try
			{
				(void)buf2.at_byte_ofs<OverAligned>(1);
			}
			catch (...)
			{
				threw = true;
			}
			PR_EXPECT(threw);

			byte_data<64> buf3;
			buf3.resize(sizeof(OverAligned) + 1);
			threw = false;
			try
			{
				(void)buf3.at_byte_ofs<OverAligned>(1);
			}
			catch (...)
			{
				threw = true;
			}
			PR_EXPECT(threw);

			std::span<std::byte const> misaligned_const{ buf2.data() + 1, sizeof(OverAligned) };
			byte_data_cptr cptr(misaligned_const);
			threw = false;
			try
			{
				(void)cptr.as<OverAligned>();
			}
			catch (...)
			{
				threw = true;
			}
			PR_EXPECT(threw);

			std::span<std::byte> misaligned_mut{ buf2.data() + 1, sizeof(OverAligned) };
			byte_data_mptr mptr(misaligned_mut);
			threw = false;
			try
			{
				(void)mptr.as<OverAligned>();
			}
			catch (...)
			{
				threw = true;
			}
			PR_EXPECT(threw);
		}
		{ // Stream
			byte_data buf0;
			buf0.push_back<char>('A');
			buf0.push_back<short>(0x5555);
			buf0.push_back<char>('B');
			buf0.push_back<int>(42);

			size_t ofs = 0;
			PR_EXPECT(buf0.read<char>(ofs) == 'A');
			PR_EXPECT(buf0.read<short>(ofs) == 0x5555);
			PR_EXPECT(buf0.read<char>(ofs) == 'B');
			PR_EXPECT(buf0.read<int>(ofs) == 42);
		}
	}
}
#endif
