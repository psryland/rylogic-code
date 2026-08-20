//******************************************
// pr::string<>
//  Copyright (c) Rylogic Ltd 2008
//******************************************
#pragma once

// A string class that correctly handles character encoding.
// Not a replacement for std::string, inspired by 'copper spice string'

#pragma intrinsic(memcmp, memcpy, memset, strcmp)
#include <stdint.h>
#include <type_traits>
//#include <memory>
//#include <string>
//#include <sstream>
//#include <stdexcept>
//#include <algorithm>
//#include <utility>
//#include <cwchar>
//#include <cassert>
//#include <functional>
//#include <memory>

namespace pr::str::experimental
{
	// String encodings
	namespace encoding
	{
		struct utf8
		{
			using storage_t = char8_t;

			// Encode 'code_point' into 'storage'
			template <typename Out>
			static void encode(char32_t ch, Out out)
			{
				if (ch < 0x80)
				{
					out(static_cast<storage_t>(ch));
					return;
				}
				if (ch < 0x0800)
				{
					out(static_cast<storage_t>(((ch >> 6) & 0x1F) | 0xC0));
					out(static_cast<storage_t>(((ch >> 0) & 0x3F) | 0x80));
					return;
				}
				if (ch < 0x10000)
				{
					out(static_cast<storage_t>(((ch >> 12) & 0x0F) | 0xE0));
					out(static_cast<storage_t>(((ch >>  6) & 0x3F) | 0x80));
					out(static_cast<storage_t>(((ch >>  0) & 0x3F) | 0x80));
					return;
				}
				out(static_cast<storage_t>(((ch >> 18) & 0x07) | 0xF0));
				out(static_cast<storage_t>(((ch >> 12) & 0x3F) | 0x80));
				out(static_cast<storage_t>(((ch >>  6) & 0x3F) | 0x80));
				out(static_cast<storage_t>(((ch >>  0) & 0x3F) | 0x80));
				return;
			}

			#if 0
			// Advance 'iter' in a string of utf-8
			template <typename Iterator>
			static Iterator advance(Iterator iter, Iterator iend, int count)
			{
				for(; iter != iend && count != 0, ++iter)
				{
					auto value = *iter;
					count -= value < 0x80 || value >= 0xC0; // ASCII or the start of an encoded char
				}

				// Advance to the next code boundary
				for (; iter != iend && *iter >= 0x80 && *iter < 0xC0, ++iter)
				{}

				return iter;
			}

			// Measure the distance between two string positions
			template <typename Iterator>
			static int distance(Iterator iter_begin, Iterator iter_end)
			{
				int count = 0;
				for (auto iter = iter_begin; iter != iter_end; ++iter)
				{
					auto value = *iter;
					count += value < 0x80 || value >= 0xC0;
				}
				return count;
			}

			// Insert characters into 'str'
			template <typename Container>
			static typename Container::const_iterator insert(Container& str, typename Container::const_iterator iter, char32_t c, int count = 1)
			{
				auto value = c;//.unicode();
				for (int x = 0; x < count; ++x)
				{
					if (value <= 0x007F)
					{
						iter = str.insert(iter, static_cast<storage_unit_t>(value));

					}
					else if (value <= 0x07FF)
					{
						iter = str.insert(iter, ((value) & 0x3F) | 0x80);
						iter = str.insert(iter, ((value >> 6) & 0x1F) | 0xC0);

					}
					else if (value <= 0xFFFF)
					{
						iter = str.insert(iter, ((value) & 0x3F) | 0x80);
						iter = str.insert(iter, ((value >> 6) & 0x3F) | 0x80);
						iter = str.insert(iter, ((value >> 12) & 0x0F) | 0xE0);

					}
					else
					{
						iter = str.insert(iter, ((value) & 0x3F) | 0x80);
						iter = str.insert(iter, ((value >> 6) & 0x3F) | 0x80);
						iter = str.insert(iter, ((value >> 12) & 0x3F) | 0x80);
						iter = str.insert(iter, ((value >> 18) & 0x07) | 0xF0);

					}
				}

				return iter;
			}

			// 
			static int walk(int len, std::vector<storage_unit_t>::const_iterator iter)
			{
				int retval = 0;

				if (len >= 0)
				{
					// walk forward
					for (int x = 0; x < len; ++x)
					{
						auto count = NumOfBytes(*iter);
						iter += count;
						retval += count;
					}
				}
				else
				{
					// walk backwards
					for (int x = 0; x > len; --x)
					{
						while (true)
						{
							--iter;
							--retval;

							uint8_t value = *iter;
							if ((value & 0xC0) != 0x80)
							{
								// at the beginning of a char
								break;
							}
						}
					}
				}

				return retval;
			}

			// Convert the bytes starting at 'iter' to a character code
			static char32_t GetCodePoint(std::vector<storage_unit_t>::const_iterator iter)
			{
				if ((*iter & 0x80) == 0)
				{
					return *iter;
				}
				if ((*iter & 0xE0) == 0xC0)
				{
					char32_t value = 0;
					value |= (iter[0] & 0x1F) << 6;
					value |= (iter[1] & 0x3F);
					return value;
				}
				if ((*iter & 0xF0) == 0xE0)
				{
					char32_t value = 0;
					value |= (iter[0] & 0x0F) << 12;
					value |= (iter[1] & 0x3F) << 6;
					value |= (iter[2] & 0x3F);
					return value;
				}

				char32_t value = (iter[0] & 0x07) << 18;
				value |= (iter[1] & 0x3F) << 12;
				value |= (iter[2] & 0x3F) << 6;
				value |= (iter[3] & 0x3F);
				return value;
			}

			static int NumOfBytes(storage_unit_t value)
			{
				if ((value & 0x80) == 0)
					return 1;
				if ((value & 0xE0) == 0xC0)
					return 2;
				if ((value & 0xF0) == 0xE0)
					return 3;
				if ((value & 0xF8) == 0xF0)
					return 4;
				return 1;
			}
			#endif
		};
		#if 0
		struct utf16
		{
			using storage_unit_t = uint16_t;

			// Advance an iterator into a string utf-16
			template <typename Iterator>
			static Iterator advance(Iterator iter_begin, Iterator iter_end, int count)
			{
				storage_unit_t value = 0;

				auto iter = iter_begin;
				for (; iter != iter_end && count != 0; ++iter)
				{
					value = *iter;
					if (value < 0xDC00 || value > 0xDFFF)
						--count;
				}

				if (value >= 0xD800 && value <= 0xDBFF)
				{
					++iter;
				}

				return iter;
			}

			// Measure the distance between two string positions
			template <typename Iterator>
			static int distance(Iterator iter_begin, Iterator iter_end)
			{
				int retval = 0;

				for (auto iter = iter_begin; iter != iter_end; ++iter)
				{
					storage_unit_t value = *iter;

					// not a surrogate
					if (value < 0xDC00 || value > 0xDFFF)
						++retval;
				}

				return retval;
			}

			template <typename Container>
			static typename Container::const_iterator insert(Container& str1, typename Container::const_iterator iter, char32_t c, int count = 1)
			{
				uint32_t value = c.unicode();

				for (int x = 0; x < count; ++x)
				{

					if ((value <= 0xD7FF) || ((value >= 0xE000) && (value <= 0xFFFF)))
					{
						iter = str1.insert(iter, value);

					}
					else
					{
						value -= 0x010000;

						iter = str1.insert(iter, ((value) & 0x03FF) + 0xDC00);
						iter = str1.insert(iter, ((value >> 10) & 0x03FF) + 0xD800);
					}

				}

				return iter;
			}

			static int walk(int len, std::vector<storage_unit_t>::const_iterator iter)
			{
				int retval = 0;
				int count = 0;

				if (len >= 0)
				{
					// walk forward

					for (int x = 0; x < len; ++x)
					{
						uint16_t value = *iter;

						count = NumOfBytes(value);
						iter += count;

						retval += count;
					}

				}
				else
				{
					// walk backwards
					for (int x = 0; x > len; --x)
					{
						while (true)
						{
							--iter;
							--retval;

							uint16_t value = *iter;
							if ((value & 0xFC00) != 0xDC00)
							{
								// at the beginning of a char
								break;
							}
						}

						// inside of the for loop
					}
				}

				return retval;
			}

			// Get the code point at 'iter'
			static char32_t GetCodePoint(std::vector<storage_unit_t>::const_iterator iter)
			{
				if ((*iter & 0xFC00) != 0xD800)
					return *iter;

				char32_t value = 0;
				value |= (iter[0] & 0x03FF) << 10;
				value |= (iter[1] & 0x03FF);
				value |= 0x010000;
				return value;
			}

		private:

			// 
			static int NumOfBytes(uint16_t value)
			{
				if ((value & 0xFC00) == 0xD800)
					return 2;

				return 1;
			}
		};
		#endif
	}
	template <typename Encoding, typename Allocator = std::allocator<typename Encoding::storage_t>>
	struct string
	{
		// Notes:
		//  - Storage is a vector of 'char_t'
		//  - Interface is a vector of 'char32_t'
		//  - Stored string is always null terminated.

		using value_type      = char32_t;
		using size_type       = std::ptrdiff_t;
		using difference_type = std::ptrdiff_t;
		using char_t          = typename Encoding::storage_t;

		//using const_iterator         = CsStringIterator<E, A>;
		//using iterator               = CsStringIterator<E, A>;
		//using const_reverse_iterator = CsStringReverseIterator<const_iterator>;
		//using reverse_iterator       = CsStringReverseIterator<iterator>;
	
	private:
		using str_t = typename std::vector<char_t, Allocator>;
		//using str_iter = typename std::vector<storage_type, Allocator>::const_iterator;

		str_t m_string;

	public:

		string()
			:m_string(1, 0)
		{}
		explicit string(Allocator const& a)
			:m_string(1, 0, a)
		{}

		// Copy
		string(string const& rhs) = default;
		string(string const& rhs, Allocator const& a)
			:m_string(rhs.m_string, a)
		{}

		// Move
		string(string&& rhs) = default;
		string(string&& rhs, Allocator const& a)
			:m_string(std::move(rhs.m_string), a)
		{}

		// From char_t const*. Stops at the first '\0'
		string(char_t const* rhs, Allocator const& a = Allocator())
			:m_string(a)
		{
			if (rhs != nullptr)
			{
				for (auto c = rhs; *c != 0; ++c)
					m_string.push_back(*c);
			}
			m_string.push_back(0);
		}

		// N from char_t const*. Does not stop early at '\0'
		string(char_t const* rhs, size_type size, Allocator const& a = Allocator())
			:m_string(a)
		{
			if (rhs != nullptr)
			{
				for (size_type x = 0; x != size; ++x)
					m_string.push_back(rhs[x]);
			}
			m_string.push_back(0);
		}

		// From an array of char_t. Stops at the first '\0'
		template <int N>
		string(char_t const (&rhs)[N], Allocator const& a = Allocator())
			:m_string(a)
		{
			for (auto* c = &rhs[0]; *c != 0; ++c)
				m_string.push_back(*c);
			m_string.push_back(0);
		}

		// N from an array of char_t. Does not stop early at '\0'
		template <int N>
		string(char_t const (&rhs)[N], size_type size, Allocator const& a = Allocator())
			:m_string(a)
		{
			for (size_type x = 0; x != size; ++x)
				m_string.push_back(rhs[x]);
			m_string.push_back(0);
		}

		// From fixed width code points
		string(char32_t const* rhs, Allocator const& a = Allocator())
			:m_string(a)
		{
			for (auto c = rhs; *c != 0; ++c)
				Encoding::encode(*c, [&](char_t b) { m_string.push_back(b); });
			m_string.push_back(0);
		}

		// From substring
		string(string const& rhs, size_type start, Allocator const& a = Allocator())
			:m_string(1, 0, a)
		{
			auto iter_beg = rhs.cbegin();
			auto iter_end = rhs.cend();

			// Find the position of 'start' in 'rhs'
			auto s = iter_beg;
			for (size_type i = 0; i != start && s != iter_end; ++i, ++s)
			{}

			// Add the sub string
			append(s, iter_end);
		}

		// From substring
		string(string const& rhs, size_type start, size_type count, Allocator const& a = Allocator())
			:m_string(1, 0, a)
		{
			auto iter_beg = rhs.cbegin();
			auto iter_end = rhs.cend();

			// Find the position of 'start' in 'rhs'
			auto s = iter_beg;
			for (size_type i = 0; i != start && s != iter_end; ++i, ++s)
			{}

			// Find the position of 'start + count' in rhs
			auto e = s;
			for (size_type i = 0; i != count && e != iter_end; ++i, ++e)
			{}

			// Add the sub string
			append(s, e);
		}

		// From single code point
		string(size_type count, char32_t c, Allocator const& a = Allocator())
			:m_string(a)
		{
			Encoding::encode(c, [&](char_t b) { m_string.push_back(b); });
			auto len = m_string.size();
			--count;

			throw std::runtime_error("not implemented");
			//m_string.resize();
			//for (; count-- != 0; )
			//{
			//}

		}

		#if 0
		template <typename E, typename A>
		template <typename U, typename>
		CsBasicString<E, A>::CsBasicString(CsBasicStringView<U> str, const A &a)
		   : CsBasicString(str.begin(), str.end(), a)
		{
		   static_assert(std::is_base_of<CsBasicString<E,A>, U>::value,
			  "Unable to construct a CsBasicString using a CsBasicStringView, encoding E is "
			  "incompatible with the encoding for U");
		}

		template <typename E, typename A>
		template <typename U,  typename>
		CsBasicString<E, A>::CsBasicString(CsBasicStringView<U> str, size_type indexStart, size_type size, const A &a)
		   : m_string(1, 0, a)
		{
		 static_assert(std::is_base_of<CsBasicString<E,A>, U>::value,
			  "Unable to construct a CsBasicString using a CsBasicStringView, encoding E is "
			  "incompatible with the encoding for U");

		   typename U::const_iterator iter_begin = str.cbegin();
		   typename U::const_iterator iter_end;

		   for (size_type i = 0; i < indexStart && iter_begin != str.cend(); ++i)  {
			  ++iter_begin;
		   }

		   if (iter_begin == str.cend()) {
			  // indexStart > length
			  return;
		   }

		   if (size >= 0) {
			  iter_end = iter_begin;

			  for (size_type i = 0; i < size && iter_end != str.cend(); ++i)  {
				 ++iter_end;
			  }

		   } else {
			  iter_end = str.cend();

		   }

		   append(iter_begin, iter_end);
		}

		template <typename E, typename A>
		template <typename Iterator>
		CsBasicString<E, A>::CsBasicString(Iterator begin, Iterator end, const A &a)
		   : m_string(1, 0, a)
		{
		   for (Iterator item = begin; item != end; ++item) {
			  E::insert(m_string, m_string.end() - 1, *item);
		   }
		}

		template <typename E, typename A>
		CsBasicString<E, A>::CsBasicString(const_iterator begin, const_iterator end, const A &a)
   : m_string(begin.codePointBegin(), end.codePointBegin(), a)
{
   m_string.push_back(0);
}
		#endif
	};


	using u8string = string<encoding::utf8>;
}


#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::str
{
	PRUnitTest(String2Test, Quick)
	{
		using namespace pr::str::experimental;

		{// Test 1
			u8string str = char8_ptr(u8"zß水🍌");
		}

		#if 0


void test_2()
{
   printf("\n** Unit Test 2\n");

   // CsChar internationalization test

   // unicode 00 42, data type char
   CsString::CsChar c0   = 'B';

   // unicode 00 BF, data type maybe char or int, compile will return the value
   CsString::CsChar c127 = '¿';
   CsString::CsChar u127 = UCHAR('¿');

   // unicode 21 B4, data type int or a compile error, not safe
   CsString::CsChar c256 = '↴';

   // unicode 21 B4, data type char32_t, guaranteed to be the proper unicode value
   CsString::CsChar u256 = UCHAR('↴');

   // unicode 01 D1 60, data type int or a compile error, not safe
   CsString::CsChar cX = '𝅘𝅥𝅮';

   // unicode 01 D1 60, data type char32_t, guaranteed to be the proper unicode value
   CsString::CsChar uX = UCHAR('𝅘𝅥𝅮');

   printf("\n");
   printf("Char B    %08x \n", c0.unicode());

   printf("Char ¿    %08x    (implementation defined, unsafe) \n", c127.unicode());
   printf("Char ¿    %08x    (unicode literal)  \n", u127.unicode());

   printf("Char ↴    %08x    (implementation defined, unsafe) \n", c256.unicode());
   printf("Char ↴    %08x    (unicode literal)  \n", u256.unicode());

   printf("Char 𝅘𝅥𝅮    %08x    (implementation defined, unsafe) \n", cX.unicode());
   printf("Char 𝅘𝅥𝅮    %08x    (unicode literal)  \n", uX.unicode());

   printf("\n");

   // not safe
   CsString::CsString str("↴");

   printf("\nIn test 2B CsString() is passed a multi-byte string literal.\n"
            "This is not safe since the constructor will assume the data is Latin-1, \n"
            "which in this case is not true. Must be passed as a UCHAR.\n");

   printf("\nString literal ↴ : %s   (mangled, 86 is non printable)", str.constData());
   printf("\nUnicode value    : %x  %x  %x \n", str[0].unicode(), str[1].unicode(), str[2].unicode() );

   for (auto c = str.constData(); *c != '\0'; ++c)  {
      printf("\nRaw numeric value stored in the buffer : %02x", *c);
   }

   printf("\n");


   //
   CsString::CsString str2(U"ABCD↴");

   printf("\nIn test 2C CsString() is passed a UTF-32 string literal with a UTF \n"
            "specifier. This calls the constructor which takes a const char32_t *\n");

   printf("\nUTF-32 string literal ABCD↴ : %s", str2.constData());

   printf("\n\n");
}

void test_3()
{
   bool ok = true;
   printf("\n** Unit Test 3\n");

   {
      CsString::CsString str;
      str.append('A');
      str.append(':');
      str.append(' ');
      str.append(UCHAR('¿'));
      str.append(' ');
      str.append(' ');
      str.append(' ');
      str.append('B');
      str.append(':');
      str.append(' ');
      str.append(UCHAR('↴'));
      str.append(' ');
      str.append(' ');
      str.append(' ');
      str.append('C');
      str.append(':');
      str.append(' ');
      str.append(UCHAR('𝅘𝅥𝅮'));

      printf("\nA: (2 bytes) upside down question mark \nB: (3 bytes) rightwards arrow with corner downwards \n"
               "C: (4 bytes) musical symbol eighth note \n%s\n", str.constData());
   }

   printf("\n");

   if (ok) {
      printf("End Unit Test Three - PASSED\n\n");

   } else {
      printf("End Unit Test Three - Failed\n\n");
      g_unitTest = false;

   }
}

void test_4()
{
   bool ok = true;
   printf("\n** Unit Test 4\n");

   static_assert (std::is_move_constructible<CsString::CsString>::value, "Unable to move CsString");

   // part a
   CsString::CsString str1("A wacky fox and sizeable pig jumped halfway over a blue moon.");
   printf("\nConstructor passed a string literal: %s", str1.constData());

   CsString::CsString str2(str1);
   printf("\nCopy Construct String 1 to String 2: %s", str2.constData());

   CsString::CsString str3(std::move(str1));
   printf("\nMove Construct String 1 to String 3: %s", str3.constData());

   // part b
   const char *tmp = "This is a string literal assigned to a 'const char *'";
   CsString::CsString str4(tmp);
   printf("\n\nConstructor passed a const char *  : %s\n", str4.constData());

   printf("\n");

   if (ok) {
      printf("End Unit Test Four - PASSED\n\n");

   } else {
      printf("End Unit Test Fout - Failed\n\n");
      g_unitTest = false;

   }
}

void test_5()
{
   bool ok = true;
   printf("\n** Unit Test 5\n");

   CsString::CsString str1(5, UCHAR('↴'));
   printf("\nConstructed string with 5 copies of the same 3 byte character: %s", str1.constData());

   std::vector<CsString::CsChar> v = {'G', 'I', 'N', 'G', 'E', 'R' };
   CsString::CsString str2(v.begin(), v.begin() + 3);
   printf("\nConstructed string from vector GINGER, use iterator to copy first 3 elements: %s\n", str2.constData());

   if (str2 != "GIN") {
      ok = false;
   }

   // find
   // To dream t he impossi ble dream
   // 0123456789 0123456789 012345678

   CsString::CsString str3("To dream the impossible dream");

   int x1 = str3.find(CsString::CsString("dream"));
   int x2 = str3.find("dream", 10);
   int x3 = str3.find_first_not_of('s', 17);

   printf("\nGiven the following string              : %s", str3.constData());
   printf("\nStart at index  0, find 'dream'         : %d", x1);
   printf("\nStart at index 10, find 'dream'         : %d", x2);
   printf("\nStart at index 17, find first not of 's': %d", x3);

   printf("\n\n");

   if (x1 != 3 || x2 != 24) {
      ok = false;
   }

   if (ok) {
      printf("End Unit Test Five - PASSED\n\n");

   } else {
      printf("End Unit Test Five - Failed\n\n");
      g_unitTest = false;

   }
}

void test_6()
{
   bool ok = true;
   printf("\n** Unit Test 6\n");

   CsString::CsString str1("Ending character is 3 bytes ");
   str1.append(UCHAR('↴'));

   CsString::CsString str2;
   str2 = str1;

   printf("\nAssign String 1 to String 2: %s", str2.constData());

   CsString::CsString str3;
   str3 = std::move(str1);

   printf("\nMove   String 1 to String 3: %s\n", str3.constData());

   if (str2 != str3) {
      ok = false;
   }

   // insert tests
   str3.insert(6, 2, UCHAR('↵'));
   printf("\nInsert  2 left arrows  at the 7th character: %s", str3.constData());

   str3.insert(6, " [string literal] ");
   printf("\nInsert  string literal at the 7th character: %s", str3.constData());


   // replace
   CsString::CsString tmp = " [string literal] xx ";
   int len = tmp.size();

   str3.replace(6, len, " { new string text } ");
   printf("\nReplace string literal at the 7th character: %s", str3.constData());


   printf("\n\n");

   if (ok) {
      printf("End Unit Test Six - PASSED\n\n");

   } else {
      printf("End Unit Test Six - Failed\n\n");
      g_unitTest = false;

   }
}

void test_7()
{
   bool ok = true;
   printf("\n** Unit Test 7\n");

   CsString::CsString str1("ABCD");
   str1.append(UCHAR('↴'));            // 3 bytes
   str1.append(UCHAR('¿'));            // 2 bytes
   str1.append('E');
   str1.append(UCHAR('𝅘𝅥𝅮'));            // 4 bytes, unicode 01 D1 60
   str1.append('F');

   printf("\nOriginal String: %s", str1.constData());
   printf("\n");


   // 2
   CsString::CsString::const_iterator iter2 = str1.end();

   if (iter2 == str1.begin()) {
      // do nothing

   } else {
      --iter2;

      while (true) {
         const CsString::CsChar c = *iter2;

         CsString::CsString tmp(1, c);
         printf("\nWalk backwards: %s", tmp.constData() );

         if (iter2 == str1.begin()) {
            break;
         }

         --iter2;
      }
   }

   printf("\n");


   // 3
   CsString::CsString str3 = str1.substr(3, 4);
   printf("\nSubstring beginning at 3, length 4: %s", str3.constData());

   printf("\n");


   // 4
   CsString::CsString::const_iterator iter4 = str1.begin();
   int eraseCnt = 0;

   while (! str1.empty())  {
      CsString::CsString tmp(1, *iter4);

      iter4 = str1.erase(iter4);
      ++eraseCnt;

      printf("\nErase %s element: %s", tmp.constData(), str1.constData());
   }

   printf("\n\n");

   if (eraseCnt != 9) {
      ok = false;
   }

   if (ok) {
      printf("End Unit Test Seven - PASSED\n\n");

   } else {
      printf("End Unit Test Seven - Failed\n\n");
      g_unitTest = false;

   }
}

void test_8()
{
   bool ok = true;
   printf("\n** Unit Test 8\n");

   // 1
   CsString::CsString str1("ABCD");
   str1.append(UCHAR('↴'));
   printf("\nOriginal String (↴ is 3 bytes): %s\n", str1.constData());

   // 2
   printf("\nString - size storage    : %d",   str1.size_storage());
   printf("\nString - size code points: %d",   str1.size_codePoints());
   printf("\nString - size            : %d",   str1.size());
   printf("\nString - length          : %d\n", str1.length());

   if (str1.size_codePoints() != 5 ) {
      ok = false;
   }

   // 3
   CsString::CsString::const_iterator iter = str1.begin();
   iter = iter + 2;

   CsString::CsString str2(iter, str1.end());
   printf("\nCopy original string from begin() + 2: %s\n", str2.constData());

   if (str2 != CsString::CsString("CD") + UCHAR('↴')) {
      ok = false;
   }

   // 4
   CsString::CsString str4 = str1.substr(3, 2);
   printf("\nSubstring beginning at 3, length 2: %s", str4.constData());

   printf("\n\n");


   if (ok) {
      printf("End Unit Test Eight - PASSED\n\n");

   } else {
      printf("End Unit Test Eight - Failed\n\n");
      g_unitTest = false;

   }
}

void test_9()
{
   bool ok = true;
   printf("\n** Unit Test 9\n");

   CsString::CsString str1("ABCD (");
   str1.append(UCHAR('↴'));                  // 21b4
   str1.append(UCHAR(')'));

   CsString::CsString str2("ABCD (");
   str2.append(UCHAR('↵'));                  // 21b5
   str2.append(UCHAR(')'));

   printf("\nString 1  (unicode 21b4):  %s",   str1.constData());
   printf("\nString 2  (unicode 21b5):  %s\n", str2.constData());

   // 1
   printf("\nTest if String 1 == String 2:  ");

   if (str1 == str2) {
      printf("Test == is true");
      ok = false;

   }  else  {
      printf("Test == is false");

   }

   // 2
   printf("\nTest if String 1 != String 2:  ");

   if (str1 != str2) {
      printf("Test != is true");

   } else {
      printf("Test != is false");
      ok = false;
   }

   // 3
   printf("\nTest if String 1 >  String 2:  ");

   if (str1 > str2) {
      printf("Test >  is true");
      ok = false;

   } else {
      printf("Test >  is false");

   }

   // 4
   printf("\nTest if String 1 <  String 2:  ");

   if (str1 < str2) {
      printf("Test <  is true");

   } else {
      printf("Test <  is false");
      ok = false;

   }

   printf("\n\n");

   if (ok) {
      printf("End Unit Test Nine - PASSED\n\n");

   } else {
      printf("End Unit Test Nine - Failed\n\n");
      g_unitTest = false;

   }
}

void test_10()
{
   bool ok = true;
   printf("\n** Unit Test 10\n");

   CsString::CsString_utf16 str1("ABCD");
   str1.append(UCHAR('↴'));            // 2 bytes     21 B4
   str1.append(UCHAR('¿'));            // 2 bytes        BF
   str1.append('E');
   str1.append(UCHAR('𝅘𝅥𝅮'));            // 4 bytes  01 D1 60
   str1.append('F');

   // 1
   CsString::CsString str2;
   convert(str1, str2);

   printf("\nString Constructed using UTF-16 then convert() to UTF-8 : %s", str2.constData());

   CsString::CsChar c = str1[7];
   printf("\nCsChar 7 (unicode 01 d1 60) : %08x", c.unicode());

   printf("\n\n");

   if (c != UCHAR('𝅘𝅥𝅮')) {
      ok = false;
   }

   if (ok) {
      printf("End Unit Test Ten - PASSED\n\n");

   } else {
      printf("End Unit Test Ten - Failed\n\n");
      g_unitTest = false;

   }
}

void test_11()
{
   bool ok = true;
   printf("\n** Unit Test 11\n");

   CsString::CsString str1("ABCD");
   str1.append(UCHAR('↴'));            // 3 bytes     21 B4
   printf("\nOriginal string : %s", str1.constData());

   str1.resize(8, UCHAR('¿'));
   printf("\nResize to 8, passing ¿ : %s", str1.constData());

   str1.resize(7);
   printf("\nResize new string to 7 : %s", str1.constData());

   str1.resize(3);
   printf("\nResize new string to 3 : %s", str1.constData());

   printf("\n\n");

   if (str1 != "ABC") {
      ok = false;
   }

   if (ok) {
      printf("End Unit Test Eleven - PASSED\n\n");

   } else {
      printf("End Unit Test Eleven - Failed\n\n");
      g_unitTest = false;

   }
}

void test_12()
{
   bool ok = true;
   printf("\n** Unit Test 12\n");

   CsString::CsString str1("ABCD");
   str1.append(UCHAR('↴'));            // 3 bytes     21 B4
   printf("\nOriginal string : %s", str1.constData());

   // find_first_not_of
   int pos = str1.find_first_not_of("DAZB");
   printf("\nFind first not of 'DAZB' no start pos : %d", pos);

   pos = str1.find_first_not_of("B↴", 1);
   printf("\nFind first not of 'B↴'   start pos 1  : %d", pos);

   printf("\n");

   // find_last_not_of
   pos = str1.find_last_not_of("E", 3);
   printf("\nFind last not of 'E'     start pos 3  : %d", pos);

   printf("\n");

   // rfind
   pos = str1.rfind('D');
   printf("\nReverse find 'D' no start pos         : %d", pos);

   pos = str1.rfind('D', 3);
   printf("\nReverse find 'D' start pos 3          : %d", pos);

   pos = str1.rfind('A', 9);
   printf("\nReverse find 'A' start pos 9          : %d", pos);

   printf("\n\n");

   if (ok) {
      printf("End Unit Test Twelve - PASSED\n\n");

   } else {
      printf("End Unit Test Twelve - Failed\n\n");
      g_unitTest = false;

   }
}

#endif
	}
}
#endif