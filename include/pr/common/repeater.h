//*******************************************************************************************
// Repeater
//  Copyright (c) Rylogic Ltd 2009
//*******************************************************************************************
#pragma once
#include <iterator>
#include <span>

namespace pr
{
	// An iterator wrapper that returns N items from an iterator that points to M items
	template <typename TItem, std::input_iterator TCIter>
	struct Repeater
	{
		// Step size is (m_source_count-1) / (m_output_count-1)
		// e.g. source = 3 |                 |                 |
		//      output = 7 |     |     |     |     |     |     |
		//      output = 6 |      |      |      |      |       |

		TCIter m_source;       // The source iterator
		int    m_source_count; // The number of items available through 'm_source'
		int    m_source_reads; // The number of items read from 'm_source' so far. Remember we're always 2 ahead.
		int    m_output_count; // The total number of items to return
		int    m_accum;        // An accumulator for stepping
		TItem  m_default;      // A default item to use when 'm_source_count == 0'
		TItem  m_curr;         // The value of the current item
		TItem  m_next;         // The value of the next item
		TItem  m_item;         // The value of the next item
		bool   m_smooth;       // Whether to linearly interpolate between items
		
		// 'source' is the source iterator
		// 'source_count' is the number of values available through 'source'
		// 'output_count' is the number items to be output from the iterator
		// 'def' is the value to return when 'source_count' is 0
		Repeater(TCIter source, int source_count, int output_count, bool smooth, TItem def)
			: m_source(source)
			, m_source_count(source_count)
			, m_source_reads(0)
			, m_output_count(output_count)
			, m_accum(0)
			, m_default(def)
			, m_curr(Next())
			, m_next(Next())
			, m_item(m_curr)
			, m_smooth(smooth)
		{}
		TItem operator*() const
		{
			return m_item;
		}
		Repeater& operator ++()
		{
			if (m_output_count <= 1 || m_source_count == 0)
				return *this;

			// Think of this as a integer line rendering algorithm. The slope is (source_count-1) / (output_count-1).
			// Every step along "output" we add the Y amount (source_count-1) to the accumulator, moving up one unit
			// whenever the accumulator passes (output_count-1).
			auto source_count = m_source_count - int(m_smooth);
			auto output_count = m_output_count - int(m_smooth);

			m_accum += source_count;
			for (; m_accum >= output_count; )
			{
				m_accum -= output_count;
				m_curr = m_next;
				m_next = Next();
			}
			m_item = m_smooth
				? TItem(m_curr + (m_next - m_curr) * (1.0f * m_accum / output_count))
				: m_curr;

			return *this;
		}
		Repeater operator ++(int)
		{
			Repeater r = *this;
			++(*this);
			return r;
		}
		TItem Next()
		{
			if (m_source_reads != m_source_count)
			{
				++m_source_reads;
				return *m_source++;
			}
			return m_source_count != 0 ? m_curr : m_default; // Only return default if 'm_source_count == 0'
		}
	};

	// Helper method for returning a point sampling repeater
	// 'source' is the source iterator
	// 'source_count' is the number of available items pointed to by 'source'
	// 'output_count' is the number of times the iterator will be incremented
	// 'def' is the value to return when 'source_count == 0'
	template <typename TItem>
	auto CreateRepeater(int output_count, TItem def)
	{
		return Repeater<TItem, TItem const*>(nullptr, 0, output_count, false, def);
	}
	template <typename TItem>
	auto CreateRepeater(std::span<TItem const> source, int output_count, TItem def)
	{
		return Repeater<TItem, TItem const*>(source.data(), static_cast<int>(source.size()), output_count, false, def);
	}
	template <typename TItem, std::input_iterator TCIter>
	auto CreateRepeater(TCIter source, int source_count, int output_count, TItem def)
	{
		return Repeater<TItem, TCIter>(source, source_count, output_count, false, def);
	}

	// Helper method for returning a linearly interpolating repeater.
	// 'source' is the source iterator
	// 'source_count' is the number of available items pointed to by 'source'
	// 'output_count' is the number of times the iterator will be incremented
	// 'def' is the value to return when 'source_count == 0'
	template <typename TItem>
	auto CreateLerpRepeater(int output_count, TItem def)
	{
		return Repeater<TItem, TItem const*>(nullptr, 0, output_count, true, def);
	}
	template <typename TItem>
	auto CreateLerpRepeater(std::span<TItem const> source, int output_count, TItem def)
	{
		return Repeater<TItem, TItem const*>(source.data(), static_cast<int>(source.size()), output_count, true, def);
	}
	template <typename TItem, std::input_iterator TCIter>
	auto CreateLerpRepeater(TCIter source, int source_count, int output_count, TItem def)
	{
		return Repeater<TItem, TCIter>(source, source_count, output_count, true, def);
	}
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::common
{
	PRUnitTest(RepeaterTests)
	{
		{
			std::vector<int> vec = { 0, 1, 2 };
			auto rep = CreateRepeater<int>(vec, 6, -1);
			PR_EXPECT(*(  rep) == 0); // First value defined after construction
			PR_EXPECT(*(++rep) == 0);
			PR_EXPECT(*(++rep) == 1);
			PR_EXPECT(*(++rep) == 1);
			PR_EXPECT(*(++rep) == 2);
			PR_EXPECT(*(++rep) == 2);
			PR_EXPECT(*(++rep) == 2); // Last value repeats indefinitely
			PR_EXPECT(*(++rep) == 2);
		}
		{
			std::vector<float> vec = { 0, 1, 2 };
			auto rep = CreateRepeater<float>(vec, 6, -1.0f);
			PR_EXPECT(*(  rep) == 0); // First value defined after construction
			PR_EXPECT(*(++rep) == 0);
			PR_EXPECT(*(++rep) == 1);
			PR_EXPECT(*(++rep) == 1);
			PR_EXPECT(*(++rep) == 2);
			PR_EXPECT(*(++rep) == 2);
			PR_EXPECT(*(++rep) == 2); // Last value repeats indefinitely
			PR_EXPECT(*(++rep) == 2);
		}
		{
			std::vector<float> vec = { 0.0f, 0.5f, 1.0f };
			auto rep = CreateLerpRepeater<float>(vec, 6, -1.0f);
			PR_EXPECT(*rep++ == 0.0f);
			PR_EXPECT(*rep++ == 0.2f);
			PR_EXPECT(*rep++ == 0.4f);
			PR_EXPECT(*rep++ == 0.6f);
			PR_EXPECT(*rep++ == 0.8f);
			PR_EXPECT(*rep++ == 1.0f);
			PR_EXPECT(*rep++ == 1.0f);
			PR_EXPECT(*rep++ == 1.0f);
		}
		{
			std::vector<float> vec = { 0.0f, 1.0f };
			auto rep = CreateLerpRepeater<float>(vec, 6, -1.0f);
			PR_EXPECT(*rep++ == 0.0f);
			PR_EXPECT(*rep++ == 0.2f);
			PR_EXPECT(*rep++ == 0.4f);
			PR_EXPECT(*rep++ == 0.6f);
			PR_EXPECT(*rep++ == 0.8f);
			PR_EXPECT(*rep++ == 1.0f);
		}
		{
			auto rep = CreateRepeater<float>(1, -1.0f);
			PR_EXPECT(*rep++ == -1.0f); // Always get default value if no source
			PR_EXPECT(*rep++ == -1.0f);
		}
		{
			auto rep = CreateLerpRepeater<float>(4, 2.0f);
			PR_EXPECT(*rep++ == 2.0f); // Always get default value if no source
			PR_EXPECT(*rep++ == 2.0f);
			PR_EXPECT(*rep++ == 2.0f);
			PR_EXPECT(*rep++ == 2.0f);
			PR_EXPECT(*rep++ == 2.0f);
		}
		{
			float f = 1.0f;
			auto rep = CreateLerpRepeater<float>(&f, 1, 4, -1.0f);
			PR_EXPECT(*rep++ == 1.0f);
			PR_EXPECT(*rep++ == 1.0f);
			PR_EXPECT(*rep++ == 1.0f);
			PR_EXPECT(*rep++ == 1.0f);
			PR_EXPECT(*rep++ == 1.0f); // only returns default if no data is provided
		}
		{
			std::vector<int> ints = { 0, 1, 2, 3, 4 };
			auto rep = CreateLerpRepeater<int>(ints, 3, -1);
			PR_EXPECT(*rep++ == 0);
			PR_EXPECT(*rep++ == 2);
			PR_EXPECT(*rep++ == 4);
		}
		{
			constexpr int count = 3;
			std::vector<int> ints = { 0, 1, 2, 3 };
			auto rep = CreateRepeater<int>(ints, count * isize(ints), -1);
			for (int i = 0; i != isize(ints); ++i)
			{
				for (int j = 0; j != count; ++j)
				{
					auto val = *rep;
					PR_EXPECT(val == i);
					++rep;
				}
			}
		}
	}
}
#endif
