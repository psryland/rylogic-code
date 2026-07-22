//*****************************************************************************
// Maths library
//  Copyright (c) Rylogic Ltd 2002
//*****************************************************************************
#pragma once

#if PR_UNITTESTS
namespace pr::math::tests
{
	// Pointer-pair range with random-access iterators but no range-level operator[].
	template <typename T>
	struct PtrRange
	{
		T const* m_first;
		T const* m_last;

		T const* begin() const
		{
			return m_first;
		}
		T const* end() const
		{
			return m_last;
		}
	};
}
#endif
