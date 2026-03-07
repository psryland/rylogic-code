//******************************************
// Ring Buffer
//  Copyright (c) Rylogic Ltd 2007
//******************************************
#pragma once
#include <cassert>
#include "pr/math/math.h"

namespace pr
{
	// Wrapper for a container with array access operators 
	// to allow access as a ring buffer.
	template <typename TIter> struct Ring
	{
		TIter m_first;
		int m_count;
		int m_offset;

		Ring() = default;
		Ring(TIter first, TIter last, int offset = 0)
			:m_first(first)
			,m_count(static_cast<int>(last - first))
			,m_offset(offset)
		{}

		// Negative indices access from the end and loop backwards through the container
		auto operator[](int i) const -> decltype(*m_first)
		{
			auto j = Wrap(i + m_offset, 0, m_count);
			return *(m_first + j);
		}
		auto operator[](int i) -> decltype(*m_first)
		{
			auto j = Wrap(i + m_offset, 0, m_count);
			return *(m_first + j);
		}

		// Set the offset to an absolute position
		void offset(int ofs)
		{
			m_offset = Wrap(ofs, 0, m_count);
		}

		// Shift the position of '0' in the ring. So, shift(1) means what was (*this)[1] is now (*this)[0]. Doesn't move the range within the buffer.
		void shift(int by)
		{
			m_offset = Wrap(m_offset + by, 0, m_count);
		}
	};

	// Create ring buffer access to a range
	template <typename Iter> Ring<Iter> MakeRing(Iter first, Iter last, int offset = 0)
	{
		return Ring<Iter>(first, last, offset);
	}
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::container
{
	PRUnitTest(RingTests)
	{
		int buf[5] = {};
		auto rbuf = MakeRing(&buf[1], &buf[1] + 3);

		rbuf[0] = 1; PR_EXPECT(buf[1] == 1);
		rbuf[1] = 2; PR_EXPECT(buf[2] == 2);
		rbuf[2] = 3; PR_EXPECT(buf[3] == 3);
		rbuf[3] = 4; PR_EXPECT(buf[1] == 4);
		rbuf[4] = 5; PR_EXPECT(buf[2] == 5);
		PR_EXPECT(buf[0] == 0);
		PR_EXPECT(buf[4] == 0);

		rbuf[-0] = -1; PR_EXPECT(buf[1] == -1);
		rbuf[-1] = -2; PR_EXPECT(buf[3] == -2);
		rbuf[-2] = -3; PR_EXPECT(buf[2] == -3);
		rbuf[-3] = -4; PR_EXPECT(buf[1] == -4);
		rbuf[-4] = -5; PR_EXPECT(buf[3] == -5);
		PR_EXPECT(buf[0] == 0);
		PR_EXPECT(buf[4] == 0);

		rbuf.shift(4);
		rbuf[0] = 1; PR_EXPECT(buf[2] == 1);
		rbuf[1] = 2; PR_EXPECT(buf[3] == 2);
		rbuf[2] = 3; PR_EXPECT(buf[1] == 3);
		rbuf[3] = 4; PR_EXPECT(buf[2] == 4);
		rbuf[4] = 5; PR_EXPECT(buf[3] == 5);
		PR_EXPECT(buf[0] == 0);
		PR_EXPECT(buf[4] == 0);

		rbuf.offset(0);
		rbuf.shift(-4);
		rbuf[-0] = -1; PR_EXPECT(buf[3] == -1);
		rbuf[-1] = -2; PR_EXPECT(buf[2] == -2);
		rbuf[-2] = -3; PR_EXPECT(buf[1] == -3);
		rbuf[-3] = -4; PR_EXPECT(buf[3] == -4);
		rbuf[-4] = -5; PR_EXPECT(buf[2] == -5);
		PR_EXPECT(buf[0] == 0);
		PR_EXPECT(buf[4] == 0);
	}
}
#endif
