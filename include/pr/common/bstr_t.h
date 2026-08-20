//*************************************************************************************************
// BSTR
// Copyright (c) Rylogic Ltd 2017
//*************************************************************************************************
#pragma once

#include <string>
#include <algorithm>
#include <wtypes.h>

namespace pr
{
	// An RAII wrapper for BSTR
	struct BSTR_t
	{
		// Notes:
		//  There is a macro named 'bstr_t' in one of the com headers... :(
		//  BSTR's are allocated on a special heap that retains the length of the allocation.
		//  This means BSTR's can contain embedded '\0'.
		//  BSTR's are wchar_t strings

		BSTR m_str;
		bool m_own;

		// Duplicate a BSTR using its explicit length so embedded '\0' code units survive the copy.
		static BSTR Duplicate(BSTR str)
		{
			if (str == nullptr)
				return nullptr;

			return SysAllocStringLen(str, SysStringLen(str));
		}

		BSTR_t()
			:m_str()
			,m_own(true)
		{}
		BSTR_t(BSTR str, bool own)
			:m_str(str)
			,m_own(own)
		{}
		BSTR_t(BSTR_t const& rhs)
			:m_str(Duplicate(rhs.m_str))
			,m_own(true)
		{}
		BSTR_t(BSTR_t&& rhs) noexcept
			:m_str(rhs.m_str)
			,m_own(rhs.m_own)
		{
			rhs.m_str = nullptr;
			rhs.m_own = false;
		}
		~BSTR_t()
		{
			if (m_str && m_own)
				SysFreeString(m_str);
		}

		// Assignment
		BSTR_t& operator = (BSTR_t const& rhs)
		{
			if (&rhs == this) return *this;
			if (m_str && m_own) SysFreeString(m_str);
			m_str = Duplicate(rhs.m_str);
			m_own = true;
			return *this;
		}
		BSTR_t& operator = (BSTR_t&& rhs) noexcept
		{
			if (&rhs == this) return *this;
			std::swap(m_str, rhs.m_str);
			std::swap(m_own, rhs.m_own);
			return *this;
		}

		// Access as raw BSTR
		operator BSTR const&() const
		{
			return m_str;
		}
		operator BSTR&()
		{
			return m_str;
		}

		// Convert to wstring
		std::wstring wstr() const
		{
			return m_str ? std::wstring(m_str, SysStringLen(m_str)) : L"";
		}

		// std::string interface
		size_t size() const
		{
			return m_str ? SysStringLen(m_str) : 0U;
		}
	};
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"

namespace pr::unittests
{
	// Copy construction should keep every code unit because BSTR length is explicit.
	PRUnitTest(BstrCopyConstructionPreservesEmbeddedNul, Quick)
	{
		using namespace pr;

		wchar_t const src[] = { L'a', L'\0', L'b' };
		UINT const src_len = static_cast<UINT>(sizeof(src) / sizeof(src[0]));

		BSTR_t original(SysAllocStringLen(src, src_len), true);
		BSTR_t copy(original);

		PR_EXPECT(SysStringLen(copy.m_str) == src_len);
		for (UINT i = 0; i != src_len; ++i)
			PR_EXPECT(copy.m_str[i] == src[i]);
	}

	// Copy assignment and self-assignment should leave the full BSTR content intact.
	PRUnitTest(BstrCopyAssignmentPreservesEmbeddedNul, Quick)
	{
		using namespace pr;

		wchar_t const src[] = { L'a', L'\0', L'b' };
		UINT const src_len = static_cast<UINT>(sizeof(src) / sizeof(src[0]));

		BSTR_t original(SysAllocStringLen(src, src_len), true);
		BSTR_t copy;

		copy = original;
		PR_EXPECT(SysStringLen(copy.m_str) == src_len);
		for (UINT i = 0; i != src_len; ++i)
			PR_EXPECT(copy.m_str[i] == src[i]);

		copy = copy;
		PR_EXPECT(SysStringLen(copy.m_str) == src_len);
		for (UINT i = 0; i != src_len; ++i)
			PR_EXPECT(copy.m_str[i] == src[i]);
	}

	// Move, attach, and null cases should remain length-safe and keep ownership state predictable.
	PRUnitTest(BstrMoveAttachAndNullBehaviour, Quick)
	{
		using namespace pr;

		wchar_t const src[] = { L'a', L'\0', L'b' };
		UINT const src_len = static_cast<UINT>(sizeof(src) / sizeof(src[0]));

		BSTR raw = SysAllocStringLen(src, src_len);
		BSTR_t attached(raw, false);
		BSTR_t moved(std::move(attached));

		PR_EXPECT(SysStringLen(moved.m_str) == src_len);
		for (UINT i = 0; i != src_len; ++i)
			PR_EXPECT(moved.m_str[i] == src[i]);
		PR_EXPECT(attached.m_str == nullptr);
		PR_EXPECT(attached.m_own == false);

		BSTR_t null_value;
		BSTR_t null_copy(null_value);
		PR_EXPECT(null_copy.m_str == nullptr);

		BSTR_t null_assigned;
		null_assigned = null_value;
		PR_EXPECT(null_assigned.m_str == nullptr);

		SysFreeString(raw);
	}
}
#endif
