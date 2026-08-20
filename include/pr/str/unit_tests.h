//**********************************
// String Core
//  Copyright (c) Rylogic Ltd 2015
//**********************************
#pragma once

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/str/string_core.h"
#include "pr/str/char8.h"
namespace pr::str
{
	PRUnitTestClass(StringCoreTests)
	{
		// Notes:
		//  - The encoding of this file is expected to be UTF-8 so characters like
		//    "±" appear to the compiler as: ...,",-62,-79,",... in literal strings.
		//    Depending on the string type, the compiler turns each of these bytes
		//    into the character type, so: u"±" becomes char16_t[3] == {-62,-79,0}.
		//  - C# uses utf-16 for its strings and "".Length returns the array size,
		//    not the number of characters.

		PRUnitTestMethod(NarrowString, Quick)
		{
			using namespace unittests;
			using namespace std::string_view_literals;
			{
				auto r = Narrow(L"");
				PR_EXPECT(r.size() == 0U);
			}
			{
				auto r = Narrow(L"Ab3");
				PR_EXPECT(r.size() == 3U);
				PR_EXPECT(r == "Ab3");
			}
			{
				auto r = Narrow(u8"±1"sv);
				PR_EXPECT(r.size() == 3U);
				PR_EXPECT(UTEqual(r.c_str(), { 0xc2_c8, 0xb1_c8, 49_c8, 0_c8 }));
			}
			{
				char const s[] = { 0xe4_ch, 0xbd_ch, 0xa0_ch, 0xe5_ch, 0xa5_ch, 0xbd_ch, 0_ch }; // 'ni hao' in utf-8
				auto r = Narrow(s);
				PR_EXPECT(r.size() == 6U);
				PR_EXPECT(UTEqual(r.c_str(), {-28, -67, -96, -27, -91, -67, 0}));
			}
			{
				wchar_t const s[] = { 0x4f60, 0x597d, 0 }; // 'ni hao' in utf-16
				auto r = Narrow(s);
				PR_EXPECT(r.size() == 6U);
				PR_EXPECT(UTEqual(r.c_str(), {-28, -67, -96, -27, -91, -67, 0}));
			}
			{
				wchar_t const s[] = L"z\u00df\u6c34\U0001f34c";
				auto r = Narrow(s);
				PR_EXPECT(r.size() == 10U);
				PR_EXPECT(UTEqual(r.c_str(), {0x7A_ch, 0xC3_ch, 0x9F_ch, 0xE6_ch, 0xB0_ch, 0xB4_ch, 0xF0_ch, 0x9F_ch, 0x8D_ch, 0x8C_ch, 0x00_ch}));
			}
			{
				char8_t const s[] = u8"zß水🍌";
				auto r = Narrow(s);
				PR_EXPECT(r.size() == 10U);
				PR_EXPECT(UTEqual(r.c_str(), {0x7A_c8, 0xC3_c8, 0x9F_c8, 0xE6_c8, 0xB0_c8, 0xB4_c8, 0xF0_c8, 0x9F_c8, 0x8D_c8, 0x8C_c8, 0x00_c8}));
			}
		}
		PRUnitTestMethod(WidenString, Quick)
		{
			using namespace unittests;
			using namespace std::string_view_literals;
			{
				auto r = Widen("");
				PR_EXPECT(r.size() == 0U);
			}
			{
				auto r = Widen("Ab3");
				PR_EXPECT(r.size() == 3U);
				PR_EXPECT(UTEqual(r.c_str(), { 'A', 'b', '3', '\0' }));
			}
			{
				auto r = Widen(u8"±1"sv);
				PR_EXPECT(r.size() == 2U);
				PR_EXPECT(UTEqual(r.c_str(), { 177, 49, 0 }));
			}
			{
				char const s[] = { 0xe4_ch, 0xbd_ch, 0xa0_ch, 0xe5_ch, 0xa5_ch, 0xbd_ch, 0_ch }; // 'ni hao' in utf-8
				auto r = Widen(s);
				PR_EXPECT(r.size() == 2U);
				PR_EXPECT(UTEqual(r.c_str(), {0x4f60, 0x597d, 0}));
			}
			{
				wchar_t const s[] = { 0x4f60, 0x597d, 0 }; // 'ni hao' in utf-16
				auto r = Widen(s);
				PR_EXPECT(r.size() == 2U);
				PR_EXPECT(UTEqual(r.c_str(), {0x4f60, 0x597d, 0}));
			}
			{
				wchar_t const s[] = L"z\u00df\u6c34\U0001f34c";
				auto r = Widen(s);
				PR_EXPECT(r.size() == 5U);
				PR_EXPECT(UTEqual(r.c_str(), {0x007a, 0x00df, 0x6c34, 0xd83c, 0xdf4c, 0}));
			}
			{
				char8_t const s[] = u8"zß水🍌";
				auto r = Widen(s);
				PR_EXPECT(r.size() == 5U);
				PR_EXPECT(UTEqual(r.c_str(), {0x007a, 0x00df, 0x6c34, 0xd83c, 0xdf4c, 0}));
			}
		}
		PRUnitTestMethod(ConvertUtf, Quick)
		{
			using namespace unittests;

			// Supplementary-plane scalars must encode to UTF-16 surrogate pairs, while
			// invalid scalars stay unrepresentable and fall back to the replacement code.
			{
				char32_t const cp[] = { 0x10FFFE, 0 };
				auto r = convert_utf<char32_t, char16_t>::convert<std::u16string>(std::u32string_view(cp, 1));
				PR_EXPECT(r.size() == 2U);
				PR_EXPECT(UTEqual(r.c_str(), { 0xDBFF, 0xDFFE, 0 }));
			}
			{
				char32_t const cp[] = { 0x10FFFF, 0 };
				auto r = convert_utf<char32_t, char16_t>::convert<std::u16string>(std::u32string_view(cp, 1));
				PR_EXPECT(r.size() == 2U);
				PR_EXPECT(UTEqual(r.c_str(), { 0xDBFF, 0xDFFF, 0 }));
			}
			{
				char32_t const cp[] = { 0x110000, 0 };
				auto r = convert_utf<char32_t, char16_t>::convert<std::u16string>(std::u32string_view(cp, 1));
				PR_EXPECT(r.size() == 1U);
				PR_EXPECT(UTEqual(r.c_str(), { 0x005F, 0 }));
			}
			{
				char32_t const cp[] = { 0xD800, 0 };
				auto r = convert_utf<char32_t, char16_t>::convert<std::u16string>(std::u32string_view(cp, 1));
				PR_EXPECT(r.size() == 1U);
				PR_EXPECT(UTEqual(r.c_str(), { 0x005F, 0 }));
			}
		}
		PRUnitTestMethod(Empty, Quick)
		{
			char const* aptr = "full";
			char            aarr[] = "";
			std::string     astr = "";
			wchar_t const* wptr = L"";
			wchar_t         warr[] = L"full";
			std::wstring    wstr = L"full";

			PR_EXPECT(!Empty(aptr));
			PR_EXPECT(Empty(aarr));
			PR_EXPECT(Empty(astr));
			PR_EXPECT(Empty(wptr));
			PR_EXPECT(!Empty(warr));
			PR_EXPECT(!Empty(wstr));
		}
		PRUnitTestMethod(Size, Quick)
		{
			char const* aptr = "length7";
			char            aarr[] = "length7";
			std::string     astr = "length7";
			wchar_t const* wptr = L"length7";
			wchar_t         warr[] = L"length7";
			std::wstring    wstr = L"length7";

			PR_EXPECT(Size(aptr) == size_t(7));
			PR_EXPECT(Size(aarr) == size_t(7));
			PR_EXPECT(Size(astr) == size_t(7));
			PR_EXPECT(Size(wptr) == size_t(7));
			PR_EXPECT(Size(warr) == size_t(7));
			PR_EXPECT(Size(wstr) == size_t(7));
		}
		PRUnitTestMethod(Range, Quick)
		{
			char const* aptr = "range";
			char            aarr[] = "range";
			std::string     astr = "range";
			wchar_t const* wptr = L"range";
			wchar_t         warr[] = L"range";
			std::wstring    wstr = L"range";

			PR_EXPECT(*Begin(aptr) == 'r' && *(End(aptr) - 1) == 'e');
			PR_EXPECT(*Begin(aarr) == 'r' && *(End(aarr) - 1) == 'e');
			PR_EXPECT(*Begin(astr) == 'r' && *(End(astr) - 1) == 'e');
			PR_EXPECT(*Begin(wptr) == L'r' && *(End(wptr) - 1) == L'e');
			PR_EXPECT(*Begin(warr) == L'r' && *(End(warr) - 1) == L'e');
			PR_EXPECT(*Begin(wstr) == L'r' && *(End(wstr) - 1) == L'e');
		}
		PRUnitTestMethod(Equal, Quick)
		{
			char const* aptr = "equal";
			char            aarr[] = "equal";
			std::string     astr = "equal";
			wchar_t const* wptr = L"equal";
			wchar_t         warr[] = L"equal";
			std::wstring    wstr = L"equal";

			PR_EXPECT(Equal(aptr, aptr) && Equal(aptr, aarr) && Equal(aptr, astr) && Equal(aptr, wptr) && Equal(aptr, warr) && Equal(aptr, wstr));
			PR_EXPECT(Equal(aarr, aptr) && Equal(aarr, aarr) && Equal(aarr, astr) && Equal(aarr, wptr) && Equal(aarr, warr) && Equal(aarr, wstr));
			PR_EXPECT(Equal(astr, aptr) && Equal(astr, aarr) && Equal(astr, astr) && Equal(astr, wptr) && Equal(astr, warr) && Equal(astr, wstr));
			PR_EXPECT(Equal(wptr, aptr) && Equal(wptr, aarr) && Equal(wptr, astr) && Equal(wptr, wptr) && Equal(wptr, warr) && Equal(wptr, wstr));
			PR_EXPECT(Equal(warr, aptr) && Equal(warr, aarr) && Equal(warr, astr) && Equal(warr, wptr) && Equal(warr, warr) && Equal(warr, wstr));
			PR_EXPECT(Equal(wstr, aptr) && Equal(wstr, aarr) && Equal(wstr, astr) && Equal(wstr, wptr) && Equal(wstr, warr) && Equal(wstr, wstr));
			PR_EXPECT(!Equal(aptr, "equal!"));
			PR_EXPECT(!Equal(aarr, "equal!"));
			PR_EXPECT(!Equal(astr, "equal!"));
			PR_EXPECT(!Equal(wptr, "equal!"));
			PR_EXPECT(!Equal(warr, "equal!"));
			PR_EXPECT(!Equal(wstr, "equal!"));
		}
		PRUnitTestMethod(EqualI, Quick)
		{
			char const* aptr = "Equal";
			char            aarr[] = "eQual";
			std::string     astr = "eqUal";
			wchar_t const* wptr = L"equAl";
			wchar_t         warr[] = L"equaL";
			std::wstring    wstr = L"EQUAL";

			PR_EXPECT(EqualI(aptr, aptr) && EqualI(aptr, aarr) && EqualI(aptr, astr) && EqualI(aptr, wptr) && EqualI(aptr, warr) && EqualI(aptr, wstr));
			PR_EXPECT(EqualI(aarr, aptr) && EqualI(aarr, aarr) && EqualI(aarr, astr) && EqualI(aarr, wptr) && EqualI(aarr, warr) && EqualI(aarr, wstr));
			PR_EXPECT(EqualI(astr, aptr) && EqualI(astr, aarr) && EqualI(astr, astr) && EqualI(astr, wptr) && EqualI(astr, warr) && EqualI(astr, wstr));
			PR_EXPECT(EqualI(wptr, aptr) && EqualI(wptr, aarr) && EqualI(wptr, astr) && EqualI(wptr, wptr) && EqualI(wptr, warr) && EqualI(wptr, wstr));
			PR_EXPECT(EqualI(warr, aptr) && EqualI(warr, aarr) && EqualI(warr, astr) && EqualI(warr, wptr) && EqualI(warr, warr) && EqualI(warr, wstr));
			PR_EXPECT(EqualI(wstr, aptr) && EqualI(wstr, aarr) && EqualI(wstr, astr) && EqualI(wstr, wptr) && EqualI(wstr, warr) && EqualI(wstr, wstr));
			PR_EXPECT(!EqualI(aptr, "equal!"));
			PR_EXPECT(!EqualI(aarr, "equal!"));
			PR_EXPECT(!EqualI(astr, "equal!"));
			PR_EXPECT(!EqualI(wptr, "equal!"));
			PR_EXPECT(!EqualI(warr, "equal!"));
			PR_EXPECT(!EqualI(wstr, "equal!"));
		}
		PRUnitTestMethod(EqualN, Quick)
		{
			char const* aptr = "equal1";
			char            aarr[] = "equal2";
			std::string     astr = "equal3";
			wchar_t const* wptr = L"equal4";
			wchar_t         warr[] = L"equal5";
			std::wstring    wstr = L"equal6";

			PR_EXPECT(EqualN(aptr, aptr, 5) && EqualN(aptr, aarr, 5) && EqualN(aptr, astr, 5) && EqualN(aptr, wptr, 5) && EqualN(aptr, warr, 5) && EqualN(aptr, wstr, 5));
			PR_EXPECT(EqualN(aarr, aptr, 5) && EqualN(aarr, aarr, 5) && EqualN(aarr, astr, 5) && EqualN(aarr, wptr, 5) && EqualN(aarr, warr, 5) && EqualN(aarr, wstr, 5));
			PR_EXPECT(EqualN(astr, aptr, 5) && EqualN(astr, aarr, 5) && EqualN(astr, astr, 5) && EqualN(astr, wptr, 5) && EqualN(astr, warr, 5) && EqualN(astr, wstr, 5));
			PR_EXPECT(EqualN(wptr, aptr, 5) && EqualN(wptr, aarr, 5) && EqualN(wptr, astr, 5) && EqualN(wptr, wptr, 5) && EqualN(wptr, warr, 5) && EqualN(wptr, wstr, 5));
			PR_EXPECT(EqualN(warr, aptr, 5) && EqualN(warr, aarr, 5) && EqualN(warr, astr, 5) && EqualN(warr, wptr, 5) && EqualN(warr, warr, 5) && EqualN(warr, wstr, 5));
			PR_EXPECT(EqualN(wstr, aptr, 5) && EqualN(wstr, aarr, 5) && EqualN(wstr, astr, 5) && EqualN(wstr, wptr, 5) && EqualN(wstr, warr, 5) && EqualN(wstr, wstr, 5));
			PR_EXPECT(!EqualN(aptr, "equal!", 6));
			PR_EXPECT(!EqualN(aarr, "equal!", 6));
			PR_EXPECT(!EqualN(astr, "equal!", 6));
			PR_EXPECT(!EqualN(wptr, "equal!", 6));
			PR_EXPECT(!EqualN(warr, "equal!", 6));
			PR_EXPECT(!EqualN(wstr, "equal!", 6));
		}
		PRUnitTestMethod(EqualNI, Quick)
		{
			char const* aptr = "Equal1";
			char            aarr[] = "eQual2";
			std::string     astr = "eqUal3";
			wchar_t const* wptr = L"equAl4";
			wchar_t         warr[] = L"equaL5";
			std::wstring    wstr = L"EQUAL6";

			PR_EXPECT(EqualNI(aptr, aptr, 5) && EqualNI(aptr, aarr, 5) && EqualNI(aptr, astr, 5) && EqualNI(aptr, wptr, 5) && EqualNI(aptr, warr, 5) && EqualNI(aptr, wstr, 5));
			PR_EXPECT(EqualNI(aarr, aptr, 5) && EqualNI(aarr, aarr, 5) && EqualNI(aarr, astr, 5) && EqualNI(aarr, wptr, 5) && EqualNI(aarr, warr, 5) && EqualNI(aarr, wstr, 5));
			PR_EXPECT(EqualNI(astr, aptr, 5) && EqualNI(astr, aarr, 5) && EqualNI(astr, astr, 5) && EqualNI(astr, wptr, 5) && EqualNI(astr, warr, 5) && EqualNI(astr, wstr, 5));
			PR_EXPECT(EqualNI(wptr, aptr, 5) && EqualNI(wptr, aarr, 5) && EqualNI(wptr, astr, 5) && EqualNI(wptr, wptr, 5) && EqualNI(wptr, warr, 5) && EqualNI(wptr, wstr, 5));
			PR_EXPECT(EqualNI(warr, aptr, 5) && EqualNI(warr, aarr, 5) && EqualNI(warr, astr, 5) && EqualNI(warr, wptr, 5) && EqualNI(warr, warr, 5) && EqualNI(warr, wstr, 5));
			PR_EXPECT(EqualNI(wstr, aptr, 5) && EqualNI(wstr, aarr, 5) && EqualNI(wstr, astr, 5) && EqualNI(wstr, wptr, 5) && EqualNI(wstr, warr, 5) && EqualNI(wstr, wstr, 5));
			PR_EXPECT(!EqualNI(aptr, "equal!", 6));
			PR_EXPECT(!EqualNI(aarr, "equal!", 6));
			PR_EXPECT(!EqualNI(astr, "equal!", 6));
			PR_EXPECT(!EqualNI(wptr, "equal!", 6));
			PR_EXPECT(!EqualNI(warr, "equal!", 6));
			PR_EXPECT(!EqualNI(wstr, "equal!", 6));
		}
		PRUnitTestMethod(FindChar, Quick)
		{
			char const* aptr = "find char";
			char            aarr[] = "find char";
			std::string     astr = "find char";
			wchar_t const* wptr = L"find char";
			wchar_t         warr[] = L"find char";
			std::wstring    wstr = L"find char";

			PR_EXPECT(*FindChar(aptr, 'i') == 'i' && *FindChar(aptr, 'b') == 0);
			PR_EXPECT(*FindChar(aarr, L'i') == 'i' && *FindChar(aarr, L'b') == 0);
			PR_EXPECT(*FindChar(astr, 'i') == 'i' && *FindChar(astr, 'b') == 0);
			PR_EXPECT(*FindChar(wptr, 'i') == L'i' && *FindChar(wptr, L'b') == 0);
			PR_EXPECT(*FindChar(warr, L'i') == L'i' && *FindChar(warr, 'b') == 0);
			PR_EXPECT(*FindChar(wstr, 'i') == L'i' && *FindChar(wstr, L'b') == 0);
		}
		PRUnitTestMethod(FindCharN, Quick)
		{
			char const* aptr = "find char";
			char            aarr[] = "find char";
			std::string     astr = "find char";
			wchar_t const* wptr = L"find char";
			wchar_t         warr[] = L"find char";
			std::wstring    wstr = L"find char";

			PR_EXPECT(*FindChar(aptr, 'i', 2) == 'i' && *FindChar(aptr, 'c', 4) == ' ');
			PR_EXPECT(*FindChar(aarr, L'i', 2) == 'i' && *FindChar(aarr, L'c', 4) == ' ');
			PR_EXPECT(*FindChar(astr, 'i', 2) == 'i' && *FindChar(astr, 'c', 4) == ' ');
			PR_EXPECT(*FindChar(wptr, 'i', 2) == L'i' && *FindChar(wptr, L'c', 4) == ' ');
			PR_EXPECT(*FindChar(warr, L'i', 2) == L'i' && *FindChar(warr, 'c', 4) == ' ');
			PR_EXPECT(*FindChar(wstr, 'i', 2) == L'i' && *FindChar(wstr, L'c', 4) == ' ');
		}
		PRUnitTestMethod(FindStr, Quick)
		{
			char const* aptr = "find in str";
			char            aarr[] = "find in str";
			std::string     astr = "find in str";
			wchar_t const* wptr = L"find in str";
			wchar_t         warr[] = L"find in str";
			std::wstring    wstr = L"find in str";

			PR_EXPECT(*FindStr(aptr, "str") == 's' && FindStr(aptr, "bob") == End(aptr));
			PR_EXPECT(*FindStr(aarr, L"str") == 's' && FindStr(aarr, L"bob") == End(aarr));
			PR_EXPECT(*FindStr(astr, "str") == 's' && FindStr(astr, "bob") == End(astr));
			PR_EXPECT(*FindStr(wptr, "str") == L's' && FindStr(wptr, L"bob") == End(wptr));
			PR_EXPECT(*FindStr(warr, L"str") == L's' && FindStr(warr, "bob") == End(warr));
			PR_EXPECT(*FindStr(wstr, "str") == L's' && FindStr(wstr, L"bob") == End(wstr));

			PR_EXPECT(FindStr(aptr + 2, aptr + 9, "in") - Begin(aptr) == 5);
			PR_EXPECT(FindStr(wptr + 2, wptr + 9, "in") - Begin(wptr) == 5);
		}
		PRUnitTestMethod(FindFirst, Quick)
		{
			//                         0123456789
			char const* aptr = "find first";
			char            aarr[] = "find first";
			std::string     astr = "find first";
			wchar_t const* wptr = L"find first";
			wchar_t         warr[] = L"find first";
			std::wstring    wstr = L"find first";

			PR_EXPECT(FindFirst(aptr, [](char    ch) { return ch == 'i'; }) == &aptr[0] + 1);
			PR_EXPECT(FindFirst(aarr, [](char    ch) { return ch == 'i'; }) == &aarr[0] + 1);
			PR_EXPECT(FindFirst(astr, [](char    ch) { return ch == 'i'; }) == &astr[0] + 1);
			PR_EXPECT(FindFirst(wptr, [](wchar_t ch) { return ch == L'i'; }) == &wptr[0] + 1);
			PR_EXPECT(FindFirst(warr, [](wchar_t ch) { return ch == L'i'; }) == &warr[0] + 1);
			PR_EXPECT(FindFirst(wstr, [](wchar_t ch) { return ch == L'i'; }) == &wstr[0] + 1);

			PR_EXPECT(FindFirst(aptr, [](char    ch) { return ch == 'x'; }) == &aptr[0] + 10);
			PR_EXPECT(FindFirst(aarr, [](char    ch) { return ch == 'x'; }) == &aarr[0] + 10);
			PR_EXPECT(FindFirst(astr, [](char    ch) { return ch == 'x'; }) == &astr[0] + 10);
			PR_EXPECT(FindFirst(wptr, [](wchar_t ch) { return ch == L'x'; }) == &wptr[0] + 10);
			PR_EXPECT(FindFirst(warr, [](wchar_t ch) { return ch == L'x'; }) == &warr[0] + 10);
			PR_EXPECT(FindFirst(wstr, [](wchar_t ch) { return ch == L'x'; }) == &wstr[0] + 10);

			PR_EXPECT(FindFirst(&aptr[0] + 2, &aptr[0] + 8, [](char ch) { return ch == 'i'; }) == &aptr[0] + 6);
			PR_EXPECT(FindFirst(&aptr[0] + 2, &aptr[0] + 8, [](char ch) { return ch == 't'; }) == &aptr[0] + 8);

			PR_EXPECT(FindFirst(aptr, 2, 6, [](char ch) { return ch == 'i'; }) == &aptr[0] + 6);
			PR_EXPECT(FindFirst(aptr, 2, 6, [](char ch) { return ch == 't'; }) == &aptr[0] + 8);
		}
		PRUnitTestMethod(FindLast, Quick)
		{
			//                         0123456789
			char const* aptr = "find flast";
			char            aarr[] = "find flast";
			std::string     astr = "find flast";
			wchar_t const* wptr = L"find flast";
			wchar_t         warr[] = L"find flast";
			std::wstring    wstr = L"find flast";

			PR_EXPECT(FindLast(aptr, [](char    ch) { return ch == 'f'; }) == &aptr[0] + 6);
			PR_EXPECT(FindLast(aarr, [](char    ch) { return ch == 'f'; }) == &aarr[0] + 6);
			PR_EXPECT(FindLast(astr, [](char    ch) { return ch == 'f'; }) == &astr[0] + 6);
			PR_EXPECT(FindLast(wptr, [](wchar_t ch) { return ch == L'f'; }) == &wptr[0] + 6);
			PR_EXPECT(FindLast(warr, [](wchar_t ch) { return ch == L'f'; }) == &warr[0] + 6);
			PR_EXPECT(FindLast(wstr, [](wchar_t ch) { return ch == L'f'; }) == &wstr[0] + 6);

			PR_EXPECT(FindLast(aptr, [](char    ch) { return ch == 'x'; }) == &aptr[0] + 0);
			PR_EXPECT(FindLast(aarr, [](char    ch) { return ch == 'x'; }) == &aarr[0] + 0);
			PR_EXPECT(FindLast(astr, [](char    ch) { return ch == 'x'; }) == &astr[0] + 0);
			PR_EXPECT(FindLast(wptr, [](wchar_t ch) { return ch == L'x'; }) == &wptr[0] + 0);
			PR_EXPECT(FindLast(warr, [](wchar_t ch) { return ch == L'x'; }) == &warr[0] + 0);
			PR_EXPECT(FindLast(wstr, [](wchar_t ch) { return ch == L'x'; }) == &wstr[0] + 0);

			PR_EXPECT(FindLast(&aptr[0] + 2, &aptr[0] + 8, [](char ch) { return ch == 'f'; }) == &aptr[0] + 6);
			PR_EXPECT(FindLast(&aptr[0] + 2, &aptr[0] + 8, [](char ch) { return ch == 't'; }) == &aptr[0] + 2);

			PR_EXPECT(FindLast(aptr, 2, 6, [](char ch) { return ch == 'f'; }) == &aptr[0] + 6);
			PR_EXPECT(FindLast(aptr, 2, 6, [](char ch) { return ch == 't'; }) == &aptr[0] + 2);
		}
		PRUnitTestMethod(FindFirstOf, Quick)
		{
			//                      0123456
			char         aarr[] = "AaAaAa";
			wchar_t      warr[] = L"AaAaAa";
			std::string  astr = "AaAaAa";
			std::wstring wstr = L"AaAaAa";

			PR_EXPECT(FindFirstOf(aarr, "A") == &aarr[0] + 0);
			PR_EXPECT(FindFirstOf(warr, "a") == &warr[0] + 1);
			PR_EXPECT(FindFirstOf(astr, "B") == &astr[0] + 6);
			PR_EXPECT(FindFirstOf(wstr, "B") == &wstr[0] + 6);
		}
		PRUnitTestMethod(FindLastOf, Quick)
		{
			//                      0123456
			char         aarr[] = "AaAaAa";
			wchar_t      warr[] = L"AaAaaa";
			std::string  astr = "AaAaaa";
			std::wstring wstr = L"Aaaaaa";
			PR_EXPECT(FindLastOf(aarr, L"A") == &aarr[0] + 5);
			PR_EXPECT(FindLastOf(warr, L"A") == &warr[0] + 3);
			PR_EXPECT(FindLastOf(astr, L"B") == &astr[0]);
			PR_EXPECT(FindLastOf(wstr, L"B") == &wstr[0]);
		}
		PRUnitTestMethod(FindFirstNotOf, Quick)
		{
			//                      01234567890123
			char         aarr[] = "junk_str_junk";
			wchar_t      warr[] = L"junk_str_junk";
			std::string  astr = "junk_str_junk";
			std::wstring wstr = L"junk_str_junk";
			PR_EXPECT(FindFirstNotOf(aarr, "_knuj") == &aarr[0] + 5);
			PR_EXPECT(FindFirstNotOf(warr, "_knuj") == &warr[0] + 5);
			PR_EXPECT(FindFirstNotOf(astr, "_knujstr") == &astr[0] + 13);
			PR_EXPECT(FindFirstNotOf(wstr, "_knujstr") == &wstr[0] + 13);
		}
		PRUnitTestMethod(FindLastNotOf, Quick)
		{
			//                      01234567890123
			char         aarr[] = "junk_str_junk";
			wchar_t      warr[] = L"junk_str_junk";
			std::string  astr = "junk_str_junk";
			std::wstring wstr = L"junk_str_junk";
			PR_EXPECT(FindLastNotOf(aarr, "_knuj") == &aarr[0] + 8);
			PR_EXPECT(FindLastNotOf(warr, "_knuj") == &warr[0] + 8);
			PR_EXPECT(FindLastNotOf(astr, "_knujstr") == &astr[0] + 0);
			PR_EXPECT(FindLastNotOf(wstr, "_knujstr") == &wstr[0] + 0);
		}
		PRUnitTestMethod(Resize, Quick)
		{
			char            aarr[] = { 'a','a','a','a' };
			wchar_t         warr[] = { L'a',L'a',L'a',L'a' };
			std::string     astr = "aaaa";
			std::wstring    wstr = L"aaaa";

			Resize(aarr, 2); PR_EXPECT(Equal(aarr, "aa"));
			Resize(warr, 2); PR_EXPECT(Equal(warr, "aa"));
			Resize(astr, 2); PR_EXPECT(Equal(astr, "aa"));
			Resize(wstr, 2); PR_EXPECT(Equal(wstr, "aa"));

			Resize(aarr, 3, 'b'); PR_EXPECT(Equal(aarr, "aab"));
			Resize(warr, 3, 'b'); PR_EXPECT(Equal(warr, "aab"));
			Resize(astr, 3, 'b'); PR_EXPECT(Equal(astr, "aab"));
			Resize(wstr, 3, 'b'); PR_EXPECT(Equal(wstr, "aab"));
		}
		PRUnitTestMethod(Append, Quick)
		{
			char         aarr[5] = {};
			wchar_t      warr[5] = {};
			std::string  astr;
			std::wstring wstr;

			Append(aarr, 'a'); Append(aarr, L'b'); Append(aarr, 'c'); PR_EXPECT(Equal(aarr, "abc"));
			Append(warr, 'a'); Append(warr, L'b'); Append(warr, 'c'); PR_EXPECT(Equal(warr, "abc"));
			Append(astr, 'a'); Append(astr, L'b'); Append(astr, 'c'); PR_EXPECT(Equal(astr, "abc"));
			Append(wstr, 'a'); Append(wstr, L'b'); Append(wstr, 'c'); PR_EXPECT(Equal(wstr, "abc"));
		}
		PRUnitTestMethod(AppendString, Quick)
		{
			char         aarr[7] = {};
			wchar_t      warr[7] = {};
			std::string  astr;
			std::wstring wstr;

			Append(aarr, "abc"); Append(aarr, L"def"); PR_EXPECT(Equal(aarr, "abcdef"));
			Append(warr, "abc"); Append(warr, L"def"); PR_EXPECT(Equal(warr, "abcdef"));
			Append(astr, "abc"); Append(astr, L"def"); PR_EXPECT(Equal(astr, "abcdef"));
			Append(wstr, "abc"); Append(wstr, L"def"); PR_EXPECT(Equal(wstr, "abcdef"));
		}
		PRUnitTestMethod(Assign, Quick)
		{
			char const* asrc = "string";
			wchar_t const* wsrc = L"string";

			char            aarr[5];
			wchar_t         warr[5];
			std::string     astr;
			std::wstring    wstr;

			Assign(aarr, asrc, asrc + 3); PR_EXPECT(Equal(aarr, "str"));
			Assign(aarr, wsrc, wsrc + 3); PR_EXPECT(Equal(aarr, "str"));
			Assign(warr, asrc, asrc + 3); PR_EXPECT(Equal(warr, "str"));
			Assign(warr, wsrc, wsrc + 3); PR_EXPECT(Equal(warr, "str"));
			Assign(astr, asrc, asrc + 3); PR_EXPECT(Equal(astr, "str"));
			Assign(astr, wsrc, wsrc + 3); PR_EXPECT(Equal(astr, "str"));
			Assign(wstr, asrc, asrc + 3); PR_EXPECT(Equal(wstr, "str"));
			Assign(wstr, wsrc, wsrc + 3); PR_EXPECT(Equal(wstr, "str"));

			Assign(aarr, 2, 2, asrc, asrc + 3); PR_EXPECT(Equal(aarr, "stst"));
			Assign(aarr, 2, 2, wsrc, wsrc + 3); PR_EXPECT(Equal(aarr, "stst"));
			Assign(warr, 2, 2, asrc, asrc + 3); PR_EXPECT(Equal(warr, "stst"));
			Assign(warr, 2, 2, wsrc, wsrc + 3); PR_EXPECT(Equal(warr, "stst"));
			Assign(astr, 2, 2, asrc, asrc + 3); PR_EXPECT(Equal(astr, "stst"));
			Assign(astr, 2, 2, wsrc, wsrc + 3); PR_EXPECT(Equal(astr, "stst"));
			Assign(wstr, 2, 2, asrc, asrc + 3); PR_EXPECT(Equal(wstr, "stst"));
			Assign(wstr, 2, 2, wsrc, wsrc + 3); PR_EXPECT(Equal(wstr, "stst"));

			Assign(astr, 2, ~size_t(), asrc, asrc + 5); PR_EXPECT(Equal(astr, "ststrin"));
			Assign(astr, 2, ~size_t(), wsrc, wsrc + 5); PR_EXPECT(Equal(astr, "ststrin"));
			Assign(wstr, 2, ~size_t(), asrc, asrc + 5); PR_EXPECT(Equal(wstr, "ststrin"));
			Assign(wstr, 2, ~size_t(), wsrc, wsrc + 5); PR_EXPECT(Equal(wstr, "ststrin"));

			Assign(astr, 2, ~size_t(), "ab"); PR_EXPECT(Equal(astr, "stab"));
			Assign(astr, 2, ~size_t(), "ab"); PR_EXPECT(Equal(astr, "stab"));
			Assign(wstr, 2, ~size_t(), "ab"); PR_EXPECT(Equal(wstr, "stab"));
			Assign(wstr, 2, ~size_t(), "ab"); PR_EXPECT(Equal(wstr, "stab"));

			Assign(astr, "done"); PR_EXPECT(Equal(astr, "done"));
			Assign(astr, "done"); PR_EXPECT(Equal(astr, "done"));
			Assign(wstr, "done"); PR_EXPECT(Equal(wstr, "done"));
			Assign(wstr, "done"); PR_EXPECT(Equal(wstr, "done"));
		}
		PRUnitTestMethod(UpperCase, Quick)
		{
			char         aarr[5] = "CaSe";
			wchar_t      warr[5] = L"CaSe";
			std::string  astr = "CaSe";
			std::wstring wstr = L"CaSe";

			PR_EXPECT(Equal(UpperCaseC(aarr), L"CASE") && Equal(aarr, "CaSe"));
			PR_EXPECT(Equal(UpperCase(warr), L"CASE") && Equal(warr, "CASE"));
			PR_EXPECT(Equal(UpperCase(astr), L"CASE") && Equal(astr, "CASE"));
			PR_EXPECT(Equal(UpperCaseC(wstr), L"CASE") && Equal(wstr, "CaSe"));
		}
		PRUnitTestMethod(LowerCase, Quick)
		{
			char         aarr[5] = "CaSe";
			wchar_t      warr[5] = L"CaSe";
			std::string  astr = "CaSe";
			std::wstring wstr = L"CaSe";

			PR_EXPECT(Equal(LowerCaseC(aarr), L"case") && Equal(aarr, "CaSe"));
			PR_EXPECT(Equal(LowerCase(warr), L"case") && Equal(warr, "case"));
			PR_EXPECT(Equal(LowerCase(astr), L"case") && Equal(astr, "case"));
			PR_EXPECT(Equal(LowerCaseC(wstr), L"case") && Equal(wstr, "CaSe"));
		}
		PRUnitTestMethod(SubStr, Quick)
		{
			char    asrc[] = "SubstringExtract";
			wchar_t wsrc[] = L"SubstringExtract";

			char         aarr[10] = {};
			wchar_t      warr[10] = {};
			std::string  astr;
			std::wstring wstr;

			SubStr(asrc, 3, 6, aarr); PR_EXPECT(Equal(aarr, "string"));
			SubStr(asrc, 3, 6, warr); PR_EXPECT(Equal(warr, "string"));
			SubStr(asrc, 3, 6, astr); PR_EXPECT(Equal(astr, "string"));
			SubStr(asrc, 3, 6, wstr); PR_EXPECT(Equal(wstr, "string"));

			SubStr(wsrc, 3, 6, aarr); PR_EXPECT(Equal(aarr, "string"));
			SubStr(wsrc, 3, 6, warr); PR_EXPECT(Equal(warr, "string"));
			SubStr(wsrc, 3, 6, astr); PR_EXPECT(Equal(astr, "string"));
			SubStr(wsrc, 3, 6, wstr); PR_EXPECT(Equal(wstr, "string"));
		}
		PRUnitTestMethod(Split, Quick)
		{
			char    astr[] = "1,,2,3,4";
			wchar_t wstr[] = L"1,,2,3,4";
			char    res[][2] = { "1","","2","3","4" };
			int i;

			std::vector<std::string> abuf;
			Split(astr, L",", [&](auto sub, int)
			{
				abuf.push_back(std::string(sub));
			});
			i = 0; for (auto& s : abuf)
				PR_EXPECT(Equal(s, res[i++]));

			std::vector<std::wstring> wbuf;
			Split(wstr, ",", [&](auto sub, int)
			{
				wbuf.push_back(std::wstring(sub));
			});
			i = 0; for (auto& s : wbuf)
				PR_EXPECT(Equal(s, res[i++]));
		}
		PRUnitTestMethod(Trim, Quick)
		{
			char            aarr[] = " \t,trim\n";
			std::string     astr = " \t,trim\n";
			wchar_t         warr[] = L" \t,trim\n";
			std::wstring    wstr = L" \t,trim\n";
			auto aws = IsWhiteSpace<char>;
			auto wws = IsWhiteSpace<wchar_t>;

			PR_EXPECT(Equal(Trim(aarr, aws, true, true), ",trim"));
			PR_EXPECT(Equal(Trim(astr, aws, true, true), ",trim"));
			PR_EXPECT(Equal(Trim(warr, wws, true, true), ",trim"));
			PR_EXPECT(Equal(Trim(wstr, wws, true, true), ",trim"));

			PR_EXPECT(Equal(Trim(" \t,trim\n", aws, true, false), ",trim\n"));
			PR_EXPECT(Equal(Trim(" \t,trim\n", aws, true, false), ",trim\n"));
			PR_EXPECT(Equal(Trim(L" \t,trim\n", wws, true, false), ",trim\n"));
			PR_EXPECT(Equal(Trim(L" \t,trim\n", wws, true, false), ",trim\n"));

			PR_EXPECT(Equal(Trim(" \t,trim\n", aws, false, true), " \t,trim"));
			PR_EXPECT(Equal(Trim(" \t,trim\n", aws, false, true), " \t,trim"));
			PR_EXPECT(Equal(Trim(L" \t,trim\n", wws, false, true), " \t,trim"));
			PR_EXPECT(Equal(Trim(L" \t,trim\n", wws, false, true), " \t,trim"));

			PR_EXPECT(Equal(TrimChars(" \t,trim\n", " \t,\n", true, true), "trim"));
			PR_EXPECT(Equal(TrimChars(" \t,trim\n", L" \t,\n", true, true), L"trim"));
			PR_EXPECT(Equal(TrimChars(L" \t,trim\n", " \t,\n", true, false), "trim\n"));
			PR_EXPECT(Equal(TrimChars(L" \t,trim\n", L" \t,\n", false, true), L" \t,trim"));

			PR_EXPECT(Equal(Trim(" \t ", aws, false, true), ""));
		}
		PRUnitTestMethod(ConvertUTFTests, Quick)
		{
			using namespace unittests;
			using namespace std::string_view_literals;
			{// utf-8 to utf-32
				auto str = char8_ptr(u8"zß水🍌");
				auto out = convert_utf<char8_t, char32_t>::convert<std::u32string>(str);
				PR_EXPECT(out == U"zß水🍌");
			}
			{// ASCII to ASCII
				std::string s0 = "abc";
				auto r0 = convert_utf<char, char>::convert<std::string>(s0);
				auto eql = Equal<std::string, std::string>(s0, r0);
				PR_EXPECT(eql);

				std::wstring s1 = L"abc";
				auto r1 = convert_utf<wchar_t, char>::convert<std::string>(s1);
				eql = Equal(s1, r1);
				PR_EXPECT(eql);
			}
			{// utf-16 to utf-8
				char16_t s[] = u"\u00b1\U0001f34c";
				auto r = convert_utf<char16_t, char>::convert<std::string>(s);
				PR_EXPECT(r.size() == 6U);
				PR_EXPECT(UTEqual(r.c_str(), { -62, -79, -16, -97, -115, -116, 0 }));
			}
			{// utf-32 to utf-16
				char32_t s[] = U"\u00b1\U0001f34c\U0001f4a9";
				auto r = convert_utf<char32_t, wchar_t>::convert<std::wstring>(s, L'?');
				PR_EXPECT(r.size() == 5U);
				PR_EXPECT(UTEqual(r.c_str(), L"±🍌💩"));
			}
			{//ucs2le to utf-8
				wchar_t const s[] = { 0x4f60, 0x597d, 0 }; // 'ni hao'
				auto r = convert_utf<wchar_t, char>::convert<std::string>(s);
				PR_EXPECT(r.size() == 6U);
				PR_EXPECT(UTEqual(r.c_str(), { 0xe4_ch, 0xbd_ch, 0xa0_ch, 0xe5_ch, 0xa5_ch, 0xbd_ch, 0_ch }));
			}
		}
	};
}
#endif

