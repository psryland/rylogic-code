#pragma once
#include <cstdint>
#include <stdexcept>

namespace pr::str::utf8
{
	using code_point_t = unsigned int;

	// Control bit helpers
	constexpr bool Continuation(char c)
	{
		return (c & 0xC0) == 0x80;
	}
	constexpr bool _1Byte(char c)
	{
		return (c & 0x80) == 0x00;
	}
	constexpr bool _2Byte(char c)
	{
		return (c & 0xE0) == 0xC0;
	}
	constexpr bool _3Byte(char c)
	{
		return (c & 0xF0) == 0xE0;
	}
	constexpr bool _4Byte(char c)
	{
		return (c & 0xF8) == 0xF0;
	}

	// Returns the number of bytes expected for a unicode character starting with 'c'
	constexpr int ByteLength(char c)
	{
		return
			_4Byte(c) ? 4 :
			_3Byte(c) ? 3 :
			_2Byte(c) ? 2 :
			_1Byte(c) ? 1 :
			0; // Invalid utf-8 character
	}

	// Boolean test for utf-8 characters
	constexpr bool IsMultibyte(char c)
	{
		return ByteLength(c) > 1;
	}

	// Convert utf-8 bytes into a code point.
	inline code_point_t CodePoint(char const*& ptr, char const* end)
	{
		int len;
		if (ptr == end || (len = ByteLength(*ptr)) == 0)
			throw std::runtime_error("Invalid unicode character");
		if (end - ptr < len)
			throw std::runtime_error("Incomplete unicode character");
		if (len == 1)
			return *ptr;

		code_point_t code = *ptr++ & (0x7F >> len);
		for (--len; len != 0; --len)
		{
			if (!Continuation(*ptr)) throw std::runtime_error("Invalid unicode character");
			code = (code << 6) | (*ptr++ & 0x3F);
		}

		return code;
	}
	inline code_point_t CodePoint(std::string_view str)
	{
		auto ptr = str.data();
		auto end = str.data() + str.size();
		return CodePoint(ptr, end);
	}

	// Write a unicode code point into a utf-8 string. Returns the number of bytes written.
	inline int Write(code_point_t code_point, char* ptr, char const* end)
	{
		if (code_point < 0x80 && end - ptr >= 1)
		{
			*ptr++ = static_cast<char>(code_point);
			return 1;
		}
		if (code_point < 0x800 && end - ptr >= 2)
		{
			*ptr++ = static_cast<char>(0xC0 | ((code_point >> 6) & 0x1F));
			*ptr++ = static_cast<char>(0x80 | ((code_point >> 0) & 0x3F));
			return 2;
		}
		if (code_point < 0x10000 && end - ptr >= 3)
		{
			*ptr++ = static_cast<char>(0xE0 | ((code_point >> 12) & 0x0F));
			*ptr++ = static_cast<char>(0x80 | ((code_point >> 6) & 0x3F));
			*ptr++ = static_cast<char>(0x80 | ((code_point >> 0) & 0x3F));
			return 3;
		}
		if (code_point < 0x110000 && end - ptr >= 4)
		{
			*ptr++ = static_cast<char>(0xF0 | ((code_point >> 18) & 0x07));
			*ptr++ = static_cast<char>(0x80 | ((code_point >> 12) & 0x3F));
			*ptr++ = static_cast<char>(0x80 | ((code_point >> 6) & 0x3F));
			*ptr++ = static_cast<char>(0x80 | ((code_point >> 0) & 0x3F));
			return 4;
		}
		throw std::runtime_error("Invalid unicode character");
	}
	inline void Write(code_point_t code_point, std::string& str)
	{
		auto size = str.size();
		str.resize(size + 4);
		str.resize(Write(code_point, str.data() + size, str.data() + str.size()));
	}
	inline std::string Write(code_point_t code_point)
	{
		std::string str;
		Write(code_point, str);
		return str;
	}

	// Convert a code point into an escaped string using the '\u' or '\U' format
	inline void Escape(code_point_t code_point, std::string& out)
	{
		constexpr char hex[] = "0123456789ABCDEF";

		auto size = out.size();
		out.resize(size + 2 + (code_point > 0xFFFF ? 8 : 4));
		auto ptr = out.data() + size;

		*ptr++ = '\\';
		*ptr++ = code_point > 0xFFFF ? 'U' : 'u';

		auto count = code_point > 0xFFFF ? 8 : 4;
		for (; count-- != 0; )
			*ptr++ = hex[(code_point >> (count << 2)) & 0xf];
	}
	inline std::string Escape(code_point_t code_point)
	{
		std::string out;
		Escape(code_point, out);
		return out;
	}

	// Convert an escaped unicode code point into a code point
	inline code_point_t Unescape(std::string_view str)
	{
		auto ptr = str.data();
		auto end = str.data() + str.size();

		if (end - ptr < 2 || ptr[0] != '\\' || (ptr[1] != 'u' && ptr[1] != 'U'))
			throw std::runtime_error("Invalid unicode escape sequence");

		auto len = ptr[1] == 'U' ? 8 : 4;
		if (end - ptr < 2 + len)
			throw std::runtime_error("Incomplete unicode escape sequence");

		ptr += 2;

		constexpr static auto Nibble = [](char c)
		{
			if (c >= '0' && c <= '9') return c - '0';
			if (c >= 'A' && c <= 'F') return c - 'A' + 10;
			if (c >= 'a' && c <= 'f') return c - 'a' + 10;
			throw std::runtime_error("Invalid hex character");
		};

		code_point_t code = 0;
		for (auto pow = 1U << ((len - 1) * 4); len-- != 0; pow >>= 4)
			code |= Nibble(*ptr++) * pow;

		return code;
	}
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::str
{
	PRUnitTest(Utf8Tests, Quick)
	{
		char const poo[] = u8"\U0001f4a9";
		char const banana[] = u8"\U0001f34c";
		char const check[] = u8"\u2714";
		char const cross[] = u8"\u2717";

		PR_EXPECT(utf8::CodePoint(poo) == 0x1f4a9);
		PR_EXPECT(utf8::CodePoint(banana) == 0x1f34c);
		PR_EXPECT(utf8::CodePoint(check) == 0x2714);
		PR_EXPECT(utf8::CodePoint(cross) == 0x2717);

		PR_EXPECT(utf8::Write(0x1f4a9) == poo);
		PR_EXPECT(utf8::Write(0x1f34c) == banana);
		PR_EXPECT(utf8::Write(0x2714) == check);
		PR_EXPECT(utf8::Write(0x2717) == cross);

		PR_EXPECT(utf8::Escape(0x1f4a9) == "\\U0001F4A9");
		PR_EXPECT(utf8::Escape(0x1f34c) == "\\U0001F34C");
		PR_EXPECT(utf8::Escape(0x2714) == "\\u2714");
		PR_EXPECT(utf8::Escape(0x2717) == "\\u2717");

		PR_EXPECT(utf8::Unescape("\\U0001F4A9") == 0x1f4a9);
		PR_EXPECT(utf8::Unescape("\\U0001F34C") == 0x1f34c);
		PR_EXPECT(utf8::Unescape("\\u2714") == 0x2714);
		PR_EXPECT(utf8::Unescape("\\u2717") == 0x2717);
	}
}
#endif
