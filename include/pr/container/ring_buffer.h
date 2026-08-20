//******************************************
// Ring Buffer
//  Copyright (c) Rylogic Ltd 2015
//******************************************
#pragma once
#include <span>
#include <cstdint>
#include <cassert>
#include <concepts>
#include <xutility>

namespace pr
{
	template <typename T> concept StorageType = requires(T t)
	{
		typename T::value_type;
		{ std::ssize(t) } -> std::convertible_to<ptrdiff_t>;
		{ t[0] } -> std::convertible_to<typename T::value_type&>;
	};

	template <StorageType TStore>
	struct RingBuffer
	{
		// Notes:
		//  The max number of items that can be stored is 'std::ssize(m_data) - 1' since head == tail means empty
		using value_type = typename TStore::value_type;

		TStore m_data;
		int64_t m_head; // index of where to add the next item or one past the last valid data
		int64_t m_tail; // index of the first item of valid data

		RingBuffer()
			: m_data()
			, m_head()
			, m_tail()
		{}
		RingBuffer(int64_t capacity) requires (requires() { TStore(capacity); })
			: m_data(capacity)
			, m_head()
			, m_tail()
		{}
		RingBuffer(TStore&& store)
			: m_data(std::move(store))
			, m_head()
			, m_tail()
		{}
	
		// Empty the ring buffer
		void Reset()
		{
			m_head = 0;
			m_tail = 0;
		}

		// Returns the amount of free space in the ring buffer
		int64_t FreeSpace() const
		{
			auto const size = std::ssize(m_data);
			if (size == 0)
				return 0;

			return m_tail - m_head + int64_t(m_tail <= m_head) * size - 1;
		}
	
		// Returns the number of items in the buffer
		int64_t Count() const
		{
			auto const size = std::ssize(m_data);
			if (size == 0)
				return 0;

			return m_head - m_tail + int64_t(m_tail > m_head) * size;
		}

		// Read a single item from the ring buffer. Faster than calling Read.
		// Returns false if there was insufficient data and the ring and 'data' are unchanged.
		bool Read(value_type& data)
		{
			if (Count() == 0)
				return false;

			data = m_data[m_tail];
			if (++m_tail == std::ssize(m_data)) m_tail = 0;
			return true;
		}

		// Write a single item to the ring buffer. Faster than calling Write()
		// Returns false if there was insufficient space and the ring is unchanged.
		bool Write(value_type const& data)
		{
			if (FreeSpace() == 0)
				return false;

			m_data[m_head] = data;
			if (++m_head == std::ssize(m_data)) m_head = 0;
			return true;
		}
	
		// Read data from the ring buffer into 'data'
		// Calling this function removes the data from the ring buffer
		// [in] 'data' - is a linear buffer to copy the data to
		// Returns true if 'length' bytes were removed from the ring buffer and copied to 'data'
		// Returns false if there was insufficient data and the ring and 'data' are unchanged.
		bool Read(std::span<value_type> data)
		{
			// Check there is enough data in the ring buffer.
			auto length = std::ssize(data);
			if (length == 0)
				return true;

			if (length > Count())
				return false;

			auto size = std::ssize(m_data) - m_tail;
			auto const* ibuf = &m_data[m_tail];
			auto* obuf = data.data();

			// Copy data from the ring buffer to 'data'
			if (length < size)
			{
				m_tail += length;
				for (; length-- != 0;) *obuf++ = *ibuf++;
			}
			else
			{
				length -= size;
				m_tail = length;
				for (; size-- != 0;) *obuf++ = *ibuf++;
				ibuf = &m_data[0];
				for (; length-- != 0;) *obuf++ = *ibuf++;
			}
			return true;
		}

		// Write data into the ring buffer
		// [in] data - the pointer to the array of data being written to the ring buffer.
		// Returns true if the data was written to the ring.
		// Returns false if there was insufficient space and the ring is unchanged.
		bool Write(std::span<value_type const> data)
		{
			// Check there is enough free space in the buffer
			auto length = std::ssize(data);
			if (length == 0)
				return true;

			if (length > FreeSpace())
				return false;

			auto size = std::ssize(m_data) - m_head;
			auto const* ibuf = data.data();
			auto* obuf = &m_data[m_head];

			// Copy data from 'data' to the ring buffer
			if (length < size)
			{
				m_head += length;
				for (; length-- != 0;) *obuf++ = *ibuf++;
			}
			else
			{
				length -= size;
				m_head = length;
				for (; size-- != 0;) *obuf++ = *ibuf++;
				obuf = &m_data[0];
				for (; length-- != 0;) *obuf++ = *ibuf++;
			}
			return true;
		}
	
		// Write data into the ring buffer overwriting the head if there is not enough free space
		// [in] data - the pointer to the array of data being written to the ring buffer.
		void Overwrite(std::span<value_type const> data)
		{
			auto length = std::ssize(data);
			if (length == 0)
				return;

			auto const size = std::ssize(m_data);
			if (size == 0)
				return;

			// If 'length' is greater than the number of bytes the ring buffer can hold,
			// only write the last 'capacity - 1' bytes from 'data' into the ring buffer
			// (the earlier bytes would be overwritten anyway)
			auto const* ibuf = data.data();
			if (length >= size)
			{
				ibuf += length - (size - 1);
				length = (size - 1);
			}

			// Shift the tail so that the ring buffer now has the needed free space
			if (length > FreeSpace())
			{
				m_tail = m_head + length;
				for (; m_tail >= size;) m_tail -= size;
			}

			// Do a normal 'Write' now there's room
			Write({ ibuf, static_cast<size_t>(length) });
		}
	
		// Deletes a maximum of 'length' items from an end of the ring buffer
		// [in] 'length' - the number of bytes to delete
		// [in] 'tail' - true if we want to delete items from the tail, false if from the head
		// If Count() <= 'length' then the ring buffer is emptied and head/tail reset to 0
		void Delete(int64_t length, bool tail)
		{
			if (Count() <= length)
			{
				Reset();
			}
			else
			{
				if (tail)
				{
					m_tail += length;
					for (; m_tail >= std::ssize(m_data);) m_tail -= std::ssize(m_data);
				}
				else
				{
					m_head += std::ssize(m_data) - length;
					for (; m_head >= std::ssize(m_data);) m_head -= std::ssize(m_data);
				}
			}
		}

		// Returns the range of contiguous data items in the ring buffer starting from 'tail+offset'
		// This can be used to 'memcpy' data out of the ring buffer.
		std::span<value_type const> Peek(int64_t offset) const
		{
			if (Count() == 0)
			{
				// Empty rings report no data regardless of the requested offset.
				assert(offset == 0);
				return {};
			}

			// 'offset' must be within the data range.
			assert(offset >= 0 && offset <= Count());

			// Not wrapped
			if (m_tail <= m_head)
			{
				auto first = static_cast<size_t>(m_tail + offset);
				return { &m_data[first], static_cast<size_t>(m_head) - first };
			}

			// Wrapped, front block
			if (offset < std::ssize(m_data) - m_tail)
			{
				auto first = static_cast<size_t>(m_tail + offset);
				return { &m_data[first], m_data.size() - first };
			}

			// Wrapped, back block
			{
				offset -= std::ssize(m_data) - m_tail;
				return { &m_data[offset], static_cast<size_t>(m_head - offset) };
			}
		}
	};
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include <vector>
namespace pr::container
{
	PRUnitTest(RingBufferTests, Quick)
	{
		const int BufSize = 10;
		struct GuardedBuffer
		{
			uint8_t guard0 = 0xAA;
			uint8_t bytes[BufSize + 1] = {};
			uint8_t guard1 = 0xBB;
		} g;

		RingBuffer<std::span<uint8_t>> ring(g.bytes);
		
		{// Zero-capacity storage
			uint8_t data[] = {0, 1, 2, 3};
			uint8_t value = 0xAB;

			RingBuffer<std::span<uint8_t>> empty_span(std::span<uint8_t>{});
			PR_EXPECT(empty_span.Count() == 0);
			PR_EXPECT(empty_span.FreeSpace() == 0);
			PR_EXPECT(!empty_span.Read(value) && value == 0xAB);
			PR_EXPECT(!empty_span.Write(value));
			PR_EXPECT(!empty_span.Read({ data, _countof(data) }));
			PR_EXPECT(!empty_span.Write({ data, _countof(data) }));
			empty_span.Overwrite({ data, _countof(data) });
			PR_EXPECT(empty_span.Count() == 0);
			PR_EXPECT(empty_span.FreeSpace() == 0);
			PR_EXPECT(empty_span.Peek(0).size() == 0);

			RingBuffer<std::vector<uint8_t>> empty_vec(0);
			value = 0xCD;
			PR_EXPECT(empty_vec.Count() == 0);
			PR_EXPECT(empty_vec.FreeSpace() == 0);
			PR_EXPECT(!empty_vec.Read(value) && value == 0xCD);
			PR_EXPECT(!empty_vec.Write(value));
			PR_EXPECT(!empty_vec.Read({ data, _countof(data) }));
			PR_EXPECT(!empty_vec.Write({ data, _countof(data) }));
			empty_vec.Overwrite({ data, _countof(data) });
			PR_EXPECT(empty_vec.Count() == 0);
			PR_EXPECT(empty_vec.FreeSpace() == 0);
			PR_EXPECT(empty_vec.Peek(0).size() == 0);
		}

		// Repeat the tests to exercise wrapping
		for (int i = 0; i != 20; ++i)
		{
			PR_EXPECT(g.guard0 == 0xAA);
			PR_EXPECT(g.guard1 == 0xBB);
			PR_EXPECT(ring.Count() == 0);
			PR_EXPECT(ring.FreeSpace() == BufSize);

			{// Read/Write single items
				uint8_t data = 0xAB;
				PR_EXPECT(!ring.Read(data) && data == 0xAB);
				PR_EXPECT(ring.Count() == 0);
				PR_EXPECT(ring.FreeSpace() == BufSize);

				PR_EXPECT(ring.Write(data));
				PR_EXPECT(ring.Count() == 1);
				PR_EXPECT(ring.FreeSpace() == BufSize - 1);

				data = 0;
				PR_EXPECT(ring.Read(data) && data == 0xAB);
				PR_EXPECT(ring.Count() == 0);
				PR_EXPECT(ring.FreeSpace() == BufSize);
			}

			{// Read/Write arrays of items
				uint8_t data[] = {0, 1, 2, 3, 4};
				PR_EXPECT(!ring.Read({ data, _countof(data) }));
				PR_EXPECT(ring.Count() == 0);
				PR_EXPECT(ring.FreeSpace() == BufSize);
				for (int j = 0; j != _countof(data); ++j)
					PR_EXPECT(data[j] == j);

				PR_EXPECT(ring.Write({ data, _countof(data) }));
				PR_EXPECT(ring.Count() == _countof(data));
				PR_EXPECT(ring.FreeSpace() == BufSize - _countof(data));

				memset(data, 0, sizeof(data));
				PR_EXPECT(ring.Read({ data, _countof(data) }));
				PR_EXPECT(ring.Count() == 0);
				PR_EXPECT(ring.FreeSpace() == BufSize);
				for (int j = 0; j != _countof(data); ++j)
					PR_EXPECT(data[j] == j);
			}

			{// Read/Overwrite
				uint8_t data[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0xA, 0xB, 0xC};
				ring.Overwrite({ data, _countof(data) });
				PR_EXPECT(ring.Count() == BufSize);
				PR_EXPECT(ring.FreeSpace() == 0);

				memset(data, 0, sizeof(data));
				PR_EXPECT(ring.Read({ data, BufSize }));
				PR_EXPECT(ring.Count() == 0);
				PR_EXPECT(ring.FreeSpace() == BufSize);
				for (int j = 0; j != BufSize; ++j)
					PR_EXPECT(data[j] == j + (_countof(data)-BufSize));
			}

			{// Peek
				static_assert(BufSize == 10); // This test relies on this
				uint8_t const* STR = (uint8_t const*)"ABCDEFGH";
				uint8_t data[12] = {0};

				ring.Reset();
				PR_EXPECT(ring.Write({ STR, 8 }));
				PR_EXPECT(ring.Count() == 8);

				// Peek for non-wrapped data
				auto range = ring.Peek(4);
				PR_EXPECT(strncmp((char const*)range.data(), "EFGH", 4) == 0);
				PR_EXPECT(range.size() == 4);

				// Remove 6 bytes and add 6. Should contain "GHABCDEF" with head at '6' and tail at '4' now.
				PR_EXPECT(ring.Read({ data, 6 }));
				PR_EXPECT(ring.Write({ STR, 6 }));
				PR_EXPECT(ring.Count() == 8);
				PR_EXPECT(ring.m_head < ring.m_tail); // confirm wrapped
			
				// Peek for wrapped data
				range = ring.Peek(1);  // Contiguous block from 'head + 1'
				PR_EXPECT(strncmp((char const*)range.data(), "HABC", 4) == 0);
				PR_EXPECT(range.size() == 4);

				range = ring.Peek(5); // Contiguous block from 'head + 4' (wrapped)
				PR_EXPECT(strncmp((char const*)range.data(), "DEF", 3) == 0);
				PR_EXPECT(range.size() == 3);

				// Read the remaining data
				PR_EXPECT(ring.Read({ data, 8 }));
				PR_EXPECT(ring.Count() == 0);
			
				// peek for empty
				range = ring.Peek(0); // Contiguous block from 'head + 0'
				PR_EXPECT(range.size() == 0);
			}
		}
	}
}

#endif