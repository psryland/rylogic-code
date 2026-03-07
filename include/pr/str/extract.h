//**********************************
// Extract
//  Copyright (c) Rylogic Ltd 2015
//**********************************
#pragma once
#include <type_traits>
#include <cerrno>
#include "pr/common/number.h"
#include "pr/common/flags_enum.h"
#include "pr/str/string_core.h"
#include "pr/str/string.h"

namespace pr::str
{
	// Notes:
	//  - Functions ending with 'C' don't advance the character source pointer.
	//  - 'errno' is used for 'wcstoul', etc conversions. 'errno' is thread local so is thread safe

	#pragma region Extract Utility Functions

	// A helper type that takes a pointer to a char stream and outputs a 'wchar_t' stream
	template <typename Ptr> struct wchar_ptr
	{
		Ptr& m_ptr;
		wchar_ptr(Ptr& ptr) :m_ptr(ptr) {}
		wchar_ptr(wchar_ptr const&) = delete;
		wchar_ptr& operator =(wchar_ptr const&) = delete;
		wchar_t    operator *() const       { return wchar_t(*m_ptr); }
		wchar_t    operator [](int i) const { return wchar_t(m_ptr[i]); }
		wchar_ptr& operator ++()            { ++m_ptr; return *this; }
		wchar_ptr  operator ++(int)         { auto this_ = *this; ++m_ptr; return this_; }
		wchar_ptr& operator +=(int count)   { m_ptr += count; return *this; }
	};

	// A string wrapper that provides a null-termination interface
	template <CharType Char> struct basic_stringz_t
	{
		Char const* m_ptr;
		Char const* m_end;
		inline static Char const m_null = 0;

		basic_stringz_t(Char const* str)
			:m_ptr(str)
			,m_end(nullptr)
		{}
		basic_stringz_t(std::basic_string_view<Char> str)
			:m_ptr(str.data())
			,m_end(str.data() + str.size())
		{}
		basic_stringz_t(std::basic_string<Char> const& str)
			:m_ptr(str.data())
			,m_end(str.data() + str.size())
		{}

		Char operator *() const
		{
			return *m_ptr;
		}
		basic_stringz_t& operator ++()
		{
			if (++m_ptr == m_end) m_ptr = &m_null;
			return *this;
		}
		basic_stringz_t operator ++(int)
		{
			auto me = *this;
			++*this;
			return me;
		}
		basic_stringz_t& operator +=(int n)
		{
			if ((m_ptr += n) == m_end) m_ptr = &m_null;
			return *this;
		}
	};
	using stringz_t = basic_stringz_t<char>;
	using wstringz_t = basic_stringz_t<wchar_t>;

	// Return the underlying char type from a pointer-like stream of characters
	template <typename Ptr>
	using char_type_t = std::decay_t<decltype(*std::declval<Ptr>())>;

	// Advance 'src' while 'pred' is true
	// Returns true if the function returned due to 'pred' returning false, false if *src == 0
	template <typename Ptr, typename Pred>
	inline bool Advance(Ptr& src, Pred pred)
	{
		// Find the first non-delimiter
		for (; *src && pred(*src); ++src) {}
		return *src != 0;
	}

	// Advance 'src' to the next delimiter character
	// Returns false if *src == 0
	template <typename Ptr, CharType Char = char_type_t<Ptr>>
	inline bool AdvanceToDelim(Ptr& src, Char const* delim = nullptr)
	{
		// Advance while *src does not point to a delimiter
		delim = Delim(delim);
		return Advance(src, [=](auto ch){ return *FindChar(delim, ch) == 0; });
	}

	// Advance 'src' to the next non-delimiter character
	// Returns false if *src == 0
	template <typename Ptr, CharType Char = char_type_t<Ptr>>
	inline bool AdvanceToNonDelim(Ptr& src, Char const* delim = nullptr)
	{
		// Advance while *src points to a delimiter
		delim = Delim(delim);
		return Advance(src, [=](Char ch){ return *FindChar(delim, ch) != 0; });
	}

	// Used to filter the accepted characters when extracting number strings
	enum class ENumType
	{
		Int = 1 << 0,
		FP  = 1 << 1,
		Any = Int|FP,
		_flags_enum = 0,
	};

	// Buffer characters for a number (real or int) from 'src'
	// Format: [delim][{+|-}][0[{x|X|b|B}]][digits][.digits][{d|D|e|E|p|P}[{+|-}]digits][U][L][L]
	// [out] 'str' = the extracted value
	// [in] 'src' = the forward only input stream
	// [in] 'radix' = the base of the number to read.
	// [in] 'type' = the number style to read.
	// [in] 'delim' = token delimiter characters.
	template <StringType Str, typename Ptr, CharType Char = char_type_t<Ptr>>
	void BufferNumber(Str& str, Ptr& src, int& radix, ENumType type = ENumType::Any, Char const* delim = nullptr)
	{
		// Notes:
		//  - This duplicates the BufferNumber function in pr::script :-/
		//    The pr::script version does not consume characters but instead buffers them within
		//    the source. I don't want pr::str to depend on pr::script and I don't want to change
		//    the behaviour of the pr::script version, so duplication is the only option.

		delim = Delim(delim);

		// Find the first non-delimiter
		if (!AdvanceToNonDelim(src, delim))
			return;

		auto len = Size(str);
		auto initial_len = len;

		constexpr auto digit = [](Char ch)
		{
			if (ch >= '0' && ch <= '9') return ch - '0';
			if (ch >= 'a' && ch <= 'z') return 10 + ch - 'a';
			if (ch >= 'A' && ch <= 'Z') return 10 + ch - 'A';
			return std::numeric_limits<int>::max();
		};

		// FP numbers can be in decimal or hex, but not anything else...
		auto allow_fp = AllSet(type, ENumType::FP);
		allow_fp &= radix == 0 || radix == 10 || radix == 16;
		auto fp = false;

		// Look for the optional sign character
		// Ideally we'd prefer not to advance 'src' passed the '+' or '-' if the next
		// character is not the start of a number. However doing so means 'src' can't
		// be a forward only input stream. Therefore, I'm pushing the responsibility
		// back to the caller, they need to check that if *src is a '+' or '-' then
		// the following char is a decimal digit.
		if (*src == '+' || *src == '-')
		{
			Append(str, *src, len);
			++src;
		}

		// If the first digit is zero, then the number may have a radix prefix.
		// '0x', '0b', or '0o' must have at least one digit following the prefix.
		// Added 'o' for octal, in addition to standard C literal syntax.
		if (*src == '0')
		{
			++src;

			// True if the radix prefix was consumed
			auto had_radix_prefix = false;

			// If 'radix' is 10 or 0 then the radix prefix overwrites radix.
			if (radix == 0 || radix == 10)
			{
				if      (*src == 'x' || *src == 'X') { radix = 16; ++src; had_radix_prefix = true; }
				else if (*src == 'o' || *src == 'O') { radix =  8; ++src; had_radix_prefix = true; allow_fp = false; }
				else if (*src == 'b' || *src == 'B') { radix =  2; ++src; had_radix_prefix = true; allow_fp = false; }
			}
			// If 'radix' is not 10 or 0, then radix is not changed but the optional prefix must still be allowed for.
			else
			{
				if      (radix == 16 && (*src == 'x' || *src == 'X')) { ++src; had_radix_prefix = true; }
				else if (radix ==  8 && (*src == 'o' || *src == 'O')) { ++src; had_radix_prefix = true; }
				else if (radix ==  2 && (*src == 'b' || *src == 'B')) { ++src; had_radix_prefix = true; }
			}

			// If a radix prefix was consumed, add characters to the output str to suit the 'strtol' still functions.
			if (had_radix_prefix)
			{
				// The prefix for hex and octal are needed for odd-ball numbers like '0x1.FEp1'
				if (digit(*src) >= radix)
				{
					len = initial_len;
					return;
				}
				else if (radix == 16)
				{
					Append(str, '0', len);
					Append(str, 'x', len);
				}
				else if (radix == 8)
				{
					Append(str, '0', len);
				}
			}
			else
			{
				// If no radix is given and cannot be determined from the radix prefix,
				// then assume octal (for conformance with C).
				// Note: 09.1 is an invalid octal number but a valid FP number.
				//       019 is an invalid octal number when FP is not allowed.
				if (radix == 0)
					radix = 8;

				// Add '0' to the string because there was no prefix and we consumed a '0'.
				Append(str, '0', len);
			}
		}

		// Default radix to decimal
		if (radix == 0)
			radix = 10;

		// Read digits up to a delimiter, decimal point, or digit >= radix.
		auto intg_len = initial_len; // the length of 'str' when we first assumed a FP number.
		for (; *src; ++src)
		{
			// If 'd' is a valid number, given 'radix', then append it.
			auto d = digit(*src);
			if (d < radix)
			{
				Append(str, *src, len);
				continue;
			}

			// If the number cannot be floating point, then we've reached the end.
			if (!allow_fp)
				break;

			// Decimal point means float
			if (*src == '.')
			{
				Append(str, *src, len);
				intg_len = initial_len;
				fp = true;
				continue;
			}

			// If 'radix' is 8 and 'allow_fp' is true, then we're assuming octal without a radix prefix.
			// In this case, the number might be a float (e.g. 09.1), but only if a decimal point is found.
			if (!fp && radix == 8 && d < 10)
			{
				if (intg_len == initial_len) intg_len = len; // Record the length of the valid integer
				Append(str, *src, len);
				continue;
			}

			// Delimiter found
			break;
		}

		// If no decimal point is found, then reset the length to the length of the valid integer.
		// If a decimal point is found, change the radix (if necessary)
		if (!fp && intg_len != initial_len)
			len = intg_len;
		if (fp && radix == 8)
			radix = 10;

		// Read an optional exponent. Note '123e4' will have fp == false because it has no '.'
		if (allow_fp && (*src == 'e' || *src == 'E' || *src == 'd' || *src == 'D' || (radix == 16 && (*src == 'p' || *src == 'P'))))
		{
			Append(str, *src, len);
			++src;

			// Read the optional exponent sign
			if (*src == '+' || *src == '-')
			{
				Append(str, *src, len);
				++src;
			}

			// Read decimal digits up to a delimiter, or suffix
			// For hex floats, the exponent is still a decimal number.
			for (; IsDecDigit(*src); ++src)
				Append(str, *src, len);

			fp = true;
		}

		// Read the optional number suffixes
		if (allow_fp && (*src == 'f' || *src == 'F'))
		{
			fp = true;
			++src;
		}
		if (!fp && (*src == 'u' || *src == 'U'))
		{
			++src;
		}
		if (!fp && (*src == 'l' || *src == 'L'))
		{
			++src;
			auto ll = *src == 'l' || *src == 'L';
			if (ll) ++src;
		}

		Resize(str, len);
	}

	#pragma endregion

	#pragma region Extract Line

	// Extract a contiguous block of characters up to (and possibly including) a new line character
	template <StringType Str, typename Ptr, CharType Char = char_type_t<Ptr>>
	bool ExtractLine(Str& line, Ptr& src, bool inc_cr, Char const* newline = nullptr)
	{
		if (newline == nullptr) newline = PR_STRLITERAL(Char,"\n");
		auto len = Size(line);

		for (;*src && *FindChar(newline, *src) == 0; Append(line, *src, len), ++src) {}
		if (*src && inc_cr) { Append(line, *src, len); ++src; }
		return true;
	}
	template <StringType Str, typename Ptr, CharType Char = char_type_t<Ptr>>
	inline bool ExtractLineC(Str& line, Ptr src, bool inc_cr, Char const* newline = nullptr)
	{
		return ExtractLine(line, src, inc_cr, newline);
	}

	#pragma endregion

	#pragma region Extract Token

	// Extract a contiguous block of non-delimiter characters from 'src'
	template <StringType Str, typename Ptr, CharType Char = char_type_t<Ptr>>
	bool ExtractToken(Str& token, Ptr& src, Char const* delim = nullptr)
	{
		delim = Delim(delim);

		// Find the first non-delimiter
		if (!AdvanceToNonDelim(src, delim))
			return false;

		// Copy up to the next delimiter
		auto len = Size(token);

		for (Append(token, *src, len), ++src; *src && *FindChar(delim, *src) == 0; Append(token, *src, len), ++src) {}
		return true;
	}
	template <StringType Str, typename Ptr, CharType Char = char_type_t<Ptr>>
	inline bool ExtractTokenC(Str& token, Ptr src, Char const* delim = nullptr)
	{
		return ExtractToken(token, src, delim);
	}

	#pragma endregion

	#pragma region Extract Identifier

	// Extract a contiguous block of identifier characters from 'src' incrementing 'src'
	template <StringType Str, typename Ptr, CharType Char = char_type_t<Ptr>>
	bool ExtractIdentifier(Str& id, Ptr& src, Char const* delim = nullptr, bool incl_dot = false)
	{
		delim = Delim(delim);

		// Find the first non-delimiter
		if (!AdvanceToNonDelim(src, delim))
			return false;

		// If the first non-delimiter is not a valid identifier character, then we can't extract an identifier
		if (!IsIdentifier(*src, true, incl_dot))
			return false;

		// Copy up to the first non-identifier character
		size_t len = Size(id);

		for (Append(id, *src, len), ++src; *src && IsIdentifier(*src, false, incl_dot); Append(id, *src, len), ++src) {}
		return true;
	}
	template <StringType Str, typename Ptr, CharType Char = char_type_t<Ptr>>
	inline bool ExtractIdentifierC(Str& id, Ptr src, Char const* delim = nullptr, bool incl_dot = false)
	{
		return ExtractIdentifier(id, src, delim, incl_dot);
	}

	#pragma endregion

	#pragma region Extract String

	// Extract a quoted (") string
	// if 'escape' is not 0, it is treated as the escape character
	// if 'quote' is not nullptr, it is treated as the accepted quote characters
	template <StringType Str, typename Ptr, CharType Char = char_type_t<Ptr>>
	bool ExtractString(Str& str, Ptr& src, Char escape, Char const* quotes, Char const* delim = nullptr)
	{
		delim = Delim(delim);

		// Find the first non-delimiter
		if (!AdvanceToNonDelim(src, delim))
			return false;

		// Set the accepted quote characters
		if (quotes == nullptr)
		{
			static Char const default_quotes[] = {'\"', '\'', 0};
			quotes = default_quotes;
		}

		// If the next character is not an acceptable quote, then this isn't a string
		auto quote = *src;
		if (FindChar(quotes, quote) != 0) ++src; else return false;

		auto len = Size(str);

		// Copy the string
		if (escape != 0)
		{
			// Copy to the closing ", allowing for the escape character
			Unescape<Char> unesc;
			for (; *src && (unesc.WithinEscapeSequence() || *src != quote); ++src)
				unesc.Translate(*src, str, len);
		}
		else
		{
			// Copy to the next "
			for (; *src && *src != quote; ++src)
				Append(str, *src, len);
		}

		// If the string doesn't end with a ", then it's not a valid string
		if (*src == quote) ++src; else return false;
		return true;
	}
	template <StringType Str, typename Ptr, CharType Char = char_type_t<Ptr>>
	inline bool ExtractString(Str& str, Ptr& src, Char escape, std::nullptr_t quotes, Char const* delim = nullptr)
	{
		return ExtractString(str, src, escape, static_cast<Char const*>(quotes), delim);
	}
	template <StringType Str, typename Ptr, CharType Char = char_type_t<Ptr>>
	inline bool ExtractString(Str& str, Ptr& src, Char const* delim = nullptr)
	{
		return ExtractString(str, src, Char(0), nullptr, delim);
	}
	template <StringType Str, typename Ptr, CharType Char = char_type_t<Ptr>>
	inline bool ExtractStringC(Str& str, Ptr src, Char const* delim = nullptr)
	{
		return ExtractString(str, src, Char(0), nullptr, delim);
	}
	template <StringType Str, typename Ptr, CharType Char = char_type_t<Ptr>>
	inline bool ExtractStringC(Str& str, Ptr src, Char escape, std::nullptr_t quotes, Char const* delim = nullptr)
	{
		return ExtractString(str, src, escape, static_cast<Char const*>(quotes), delim);
	}
	template <StringType Str, typename Ptr, CharType Char = char_type_t<Ptr>>
	inline bool ExtractStringC(Str& str, Ptr src, Char escape, Char const* quotes, Char const* delim = nullptr)
	{
		return ExtractString(str, src, escape, quotes, delim);
	}

	#pragma endregion

	#pragma region Extract Bool

	// Extract a boolean from 'src'
	// Expects 'src' to point to a string of the following form:
	// [delim]{0|1|true|false}
	// The first character that does not fit this form stops the scan.
	// '0','1' must be followed by a non-identifier character
	// 'true', 'false' can have any case
	template <typename Bool, typename Ptr, CharType Char = char_type_t<Ptr>>
	bool ExtractBool(Bool& bool_, Ptr& src, Char const* delim = nullptr)
	{
		delim = Delim(delim);

		// Find the first non-delimiter
		if (!AdvanceToNonDelim(src, delim))
			return false;

		// Convert a char to a lower case 'wchar_t'
		auto lwr = [](Char ch){ return char_traits<wchar_t>::lwr(wchar_t(ch)); };

		// Extract the boolean
		switch (lwr(*src))
		{
		default : return false;
		case '0': bool_ = static_cast<Bool>(false); return !IsIdentifier(*++src, false);
		case '1': bool_ = static_cast<Bool>(true ); return !IsIdentifier(*++src, false);
		case 't': bool_ = static_cast<Bool>(true ); return lwr(*++src) == 'r' && lwr(*++src) == 'u' && lwr(*++src) == 'e'                       && !IsIdentifier(*++src, false);
		case 'f': bool_ = static_cast<Bool>(false); return lwr(*++src) == 'a' && lwr(*++src) == 'l' && lwr(*++src) == 's' && lwr(*++src) == 'e' && !IsIdentifier(*++src, false);
		}
	}
	template <typename Bool, typename Ptr, CharType Char = char_type_t<Ptr>>
	inline bool ExtractBoolC(Bool& bool_, Ptr src, Char const* delim = nullptr)
	{
		return ExtractBool(bool_, src, delim);
	}
	template <typename Bool, typename Ptr, CharType Char = char_type_t<Ptr>>
	inline bool ExtractBoolArray(Bool* bool_, size_t count, Ptr& src, Char const* delim = nullptr)
	{
		while (count--) if (!ExtractBool(*bool_++, src, delim)) return false;
		return true;
	}
	template <typename Bool, typename Ptr, CharType Char = char_type_t<Ptr>>
	inline bool ExtractBoolArrayC(Bool* bool_, size_t count, Ptr src, Char const* delim = nullptr)
	{
		return ExtractBoolArray(bool_, count, src, delim);
	}

	#pragma endregion

	#pragma region Extract Int

	// Extract an integral number from 'src' (basically 'strtol')
	// Expects 'src' to point to a string of the following form:
	// [delim] [{+|-}][0[{x|X|b|B}]][digits]
	// The first character that does not fit this form stops the scan.
	// If 'radix' is between 2 and 36, then it is used as the base of the number.
	// If 'radix' is 0, the initial characters of the string are used to determine the base.
	// If the first character is 0 and the second character is not 'x' or 'X', the string is interpreted as an octal integer;
	// otherwise, it is interpreted as a decimal number. If the first character is '0' and the second character is 'x' or 'X',
	// the string is interpreted as a hexadecimal integer. If the first character is '1' through '9', the string is interpreted
	// as a decimal integer. The letters 'a' through 'z' (or 'A' through 'Z') are assigned the values 10 through 35; only letters
	// whose assigned values are less than 'radix' are permitted.
	template <typename Int, typename Ptr, CharType Char = char_type_t<Ptr>>
	bool ExtractInt(Int& intg, int radix, Ptr& src, Char const* delim = nullptr)
	{
		string<Char, 256> str = {};
		BufferNumber(str, src, radix, ENumType::Int, delim);
		if (str.empty()) return false;

		errno = 0;
		Char const* end;
		intg = std::is_unsigned_v<Int>
			? static_cast<Int>(char_traits<Char>::strtoui64(str.c_str(), &end, radix))
			: static_cast<Int>(char_traits<Char>::strtoi64(str.c_str(), &end, radix));

		// Check all of the string was used in the conversion and there wasn't an overflow
		return static_cast<size_t>(end - str.c_str()) == str.size() && errno != ERANGE;
	}
	template <typename Int, typename Ptr, CharType Char = char_type_t<Ptr>>
	inline bool ExtractIntC(Int& intg, int radix, Ptr src, Char const* delim = nullptr)
	{
		return ExtractInt(intg, radix, src, delim);
	}
	template <typename Int, typename Ptr, CharType Char = char_type_t<Ptr>>
	inline bool ExtractIntArray(Int* intg, size_t count, int radix, Ptr& src, Char const* delim = nullptr)
	{
		while (count--) if (!ExtractInt(*intg++, radix, src, delim)) return false;
		return true;
	}
	template <typename Int, typename Ptr, CharType Char = char_type_t<Ptr>>
	inline bool ExtractIntArrayC(Int* intg, size_t count, int radix, Ptr src, Char const* delim = nullptr)
	{
		return ExtractIntArray(intg, count, radix, src, delim); 
	}

	#pragma endregion

	#pragma region Extract Real

	// Extract a floating point number from 'src'
	// Expects 'src' to point to a string of the following form:
	// [delim] [{+|-}][digits][.digits][{d|D|e|E}[{+|-}]digits]
	// The first character that does not fit this form stops the scan.
	// If no digits appear before the '.' character, at least one must appear after the '.' character.
	// The decimal digits can be followed by an exponent, which consists of an introductory letter (d, D, e, or E) and an optionally signed integer.
	// If neither an exponent part nor a '.' character appears, a '.' character is assumed to follow the last digit in the string.
	template <typename Real, typename Ptr, CharType Char = char_type_t<Ptr>>
	bool ExtractReal(Real& real, Ptr& src, Char const* delim = nullptr)
	{
		int radix = 10;
		pr::string<Char, 256> str = {};
		BufferNumber(str, src, radix, ENumType::FP, delim);
		if (str.empty())
		{
			// Convert a char to a lower case 'wchar_t'
			auto lwr = [](Char ch){ return char_traits<wchar_t>::lwr(wchar_t(ch)); };

			// Check for 'nan'
			if (lwr(*src) == 'n' && lwr(*++src) == 'a' && lwr(*++src) == 'n')
			{
				real = std::numeric_limits<Real>::quiet_NaN();
				return true;
			}

			auto sign =
				*src == '-' ? (++src, -1) :
				*src == '+' ? (++src, +1) :
				+1;

			// Check for '(+/-)inf'
			if (lwr(*src) == 'i' && lwr(*++src) == 'n' && lwr(*++src) == 'f')
			{
				real = sign * std::numeric_limits<Real>::infinity();
				return true;
			}

			return false;
		}

		errno = 0;
		Char const* end;
		real = static_cast<Real>(char_traits<Char>::strtod(str.c_str(), &end));

		// Check all of the string was used in the conversion and there wasn't an overflow
		return static_cast<size_t>(end - str.c_str()) == str.size() && errno != ERANGE;
	}
	template <typename Real, typename Ptr, CharType Char = char_type_t<Ptr>>
	inline bool ExtractRealC(Real& real, Ptr src, Char const* delim = nullptr)
	{
		return ExtractReal(real, src, delim);
	}
	template <typename Real, typename Ptr, CharType Char = char_type_t<Ptr>>
	inline bool ExtractRealArray(Real* real, size_t count, Ptr& src, Char const* delim = nullptr)
	{
		while (count--) if (!ExtractReal(*real++, src, delim)) return false;
		return true;
	}
	template <typename Real, typename Ptr, CharType Char = char_type_t<Ptr>> 
	inline bool ExtractRealArrayC(Real* real, size_t count, Ptr src, Char const* delim = nullptr)
	{
		return ExtractRealArray(real, count, src, delim);
	}

	#pragma endregion

	#pragma region Extract Number

	// Extract a number (real or int) from 'src'
	// Format: [delim][{+|-}][0[{x|X|b|B}]][digits][.digits][{d|D|e|E}[{+|-}]digits][U][L][L]
	// [out] 'num' = the extracted value
	// [in] 'radix' = the base of the number to read.
	// [in] 'type' = the number style to read.
	// [in] 'src' = the forward only input stream
	// [in] 'delim' = token delimiter characters.
	// Returns false if a valid number could not be read, or 'str' is too small
	template <typename Ptr, CharType Char = char_type_t<Ptr>>
	bool ExtractNumber(Number& num, Ptr& src, int radix = 0, Char const* delim = nullptr)
	{
		pr::string<Char, 256> str = {};
		BufferNumber(str, src, radix, ENumType::Any, delim);
		if (str.empty()) return false;

		errno = 0;
		Char const* end;
		num = Number::From(str.c_str(), &end, radix);

		// Check all of the string was used in the conversion and there wasn't an overflow
		return static_cast<size_t>(end - str.c_str()) == str.size() && errno != ERANGE;
	}
	template <typename Ptr, CharType Char = char_type_t<Ptr>>
	inline bool ExtractNumberC(Number& num, Ptr src, int radix = 0, Char const* delim = nullptr)
	{
		return ExtractNumber(num, src, radix, delim);
	}
	template <typename Real, typename Ptr, CharType Char = char_type_t<Ptr>>
	inline bool ExtractNumberArray(Real* real, size_t count, Ptr& src, int radix = 0, Char const* delim = nullptr)
	{
		while (count--) if (!ExtractNumber(*real++, src, radix, delim)) return false;
		return true;
	}
	template <typename Real, typename Ptr, CharType Char = char_type_t<Ptr>>
	inline bool ExtractNumberArrayC(Number* num, size_t count, Ptr src, int radix = 0, Char const* delim = nullptr)
	{
		return ExtractNumberArray(num, count, src, radix, delim);
	}

	#pragma endregion

	#pragma region Extract Enum

	// This is basically a convenience wrapper around ExtractInt
	template <typename TEnum, typename Ptr, CharType Char = char_type_t<Ptr>>
	bool ExtractEnumValue(TEnum& enum_, int radix, Ptr& src, Char const* delim = nullptr)
	{
		std::underlying_type_t<TEnum> val;
		if (!ExtractInt(val, radix, src, delim)) return false;
		enum_ = static_cast<TEnum>(val);
		return true;
	}
	template <typename TEnum, typename Ptr, CharType Char = char_type_t<Ptr>>
	inline bool ExtractEnumValueC(TEnum& enum_, int radix, Ptr src, Char const* delim = nullptr)
	{
		return ExtractEnum(enum_, radix, src, delim);
	}

	// Extracts an enum by its string name. For use with 'PR_ENUM' defined enums
	template <typename TEnum, typename Ptr, CharType Char = char_type_t<Ptr>>
	inline bool ExtractEnum(TEnum& enum_, Ptr& src, Char const* delim = nullptr)
	{
		Char val[512] = {};
		if (!ExtractIdentifier(val, src, delim)) return false;
		return Enum<TEnum>::TryParse(enum_, std::basic_string_view<Char>{ &val[0] }, false);
	}
	template <typename TEnum, typename Ptr, CharType Char = char_type_t<Ptr>>
	inline bool ExtractEnumC(TEnum& enum_, Ptr src, Char const* delim = nullptr)
	{
		return ExtractEnum(enum_, src, delim);
	}

	#pragma endregion

	#pragma region Extract Section

	template <StringType Str, typename Ptr, CharType Char = char_type_t<Ptr>>
	bool ExtractSection(Str& section, Ptr& src, Char const* delim = nullptr)
	{
		delim = Delim(delim);

		// Don't call this unless 'src' is pointing at a '{'
		if (*src != '{')
			return false;

		auto len = Size(section);

		for (int nest = 0; *src;)
		{
			if (*src == L'\"')
			{
				Char quote[] = { *src, '\0' };
				Append(section, quote[0], len);
				if (!ExtractString(section, src, Char('\\'), quote, delim)) return false;
				Append(section, quote[0], len = Size(section));
				continue;
			}
			nest += (*src == L'{') ? 1 : 0;
			nest -= (*src == L'}') ? 1 : 0;
			if (nest == 0) break;
			Append(section, *src, len);
			++src;
		}

		if (*src != '}')
			return false;

		Append(section, *src, len);
		++src;

		return true;
	}

	#pragma endregion
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::str
{
	PRUnitTest(ExtractTests)
	{
		{// Line
			wchar_t const* src = L"abcefg\nhijk\nlmnop", *s;
			char         aarr[16] = {};
			wchar_t      warr[16] = {};
			std::string  astr;
			std::wstring wstr;

			s = src;
			PR_EXPECT(ExtractLine(aarr, s, false) && Equal(aarr, "abcefg") && *s == L'\n');
			PR_EXPECT(ExtractLineC(aarr, ++s, true) && Equal(aarr, "abcefghijk\n") && *s == L'h');

			s = src;
			PR_EXPECT(ExtractLine(warr, s, false) && Equal(warr, "abcefg") && *s == L'\n');
			PR_EXPECT(ExtractLineC(warr, ++s, true) && Equal(warr, "abcefghijk\n") && *s == L'h');

			s = src;
			PR_EXPECT(ExtractLine(astr, s, false) && Equal(astr, "abcefg") && *s == L'\n');
			PR_EXPECT(ExtractLineC(astr, ++s, true) && Equal(astr, "abcefghijk\n") && *s == L'h');

			s = src;
			PR_EXPECT(ExtractLine(wstr, s, false) && Equal(wstr, "abcefg") && *s == L'\n');
			PR_EXPECT(ExtractLineC(wstr, ++s, true) && Equal(wstr, "abcefghijk\n") && *s == L'h');
		}
		{// Token
			using namespace pr::str;

			wchar_t const* src = L"token1 token2:token3", *s;
			char         aarr[16] = {};
			wchar_t      warr[16] = {};
			std::string  astr;
			std::wstring wstr;

			s = src;
			PR_EXPECT(ExtractToken(aarr, s) && Equal(aarr, "token1") && *s == L' ');
			PR_EXPECT(ExtractTokenC(aarr, ++s, L" \n:") && Equal(aarr, "token1token2") && *s == L't');

			s = src;
			PR_EXPECT(ExtractToken(warr, s) && Equal(warr, "token1") && *s == L' ');
			PR_EXPECT(ExtractTokenC(warr, ++s, L" \n:") && Equal(warr, "token1token2") && *s == L't');

			s = src;
			PR_EXPECT(ExtractToken(astr, s) && Equal(astr, "token1") && *s == L' ');
			PR_EXPECT(ExtractTokenC(astr, ++s, L" \n:") && Equal(astr, "token1token2") && *s == L't');

			s = src;
			PR_EXPECT(ExtractToken(wstr, s) && Equal(wstr, "token1") && *s == L' ');
			PR_EXPECT(ExtractTokenC(wstr, ++s, L" \n:") && Equal(wstr, "token1token2") && *s == L't');
		}
		{// Identifier
			using namespace pr::str;

			wchar_t const* src = L"_ident ident2:token3", *s;
			char         aarr[16] = {};
			wchar_t      warr[16] = {};
			std::string  astr;
			std::wstring wstr;

			s = src;
			PR_EXPECT(ExtractIdentifier(aarr, s) && Equal(aarr, "_ident") && *s == L' ');
			PR_EXPECT(ExtractIdentifierC(aarr, ++s) && Equal(aarr, "_identident2") && *s == L'i');

			s = src;
			PR_EXPECT(ExtractIdentifier(warr, s) && Equal(warr, "_ident") && *s == L' ');
			PR_EXPECT(ExtractIdentifierC(warr, ++s) && Equal(warr, "_identident2") && *s == L'i');

			s = src;
			PR_EXPECT(ExtractIdentifier(astr, s) && Equal(astr, "_ident") && *s == L' ');
			PR_EXPECT(ExtractIdentifierC(astr, ++s) && Equal(astr, "_identident2") && *s == L'i');

			s = src;
			PR_EXPECT(ExtractIdentifier(wstr, s) && Equal(wstr, "_ident") && *s == L' ');
			PR_EXPECT(ExtractIdentifierC(wstr, ++s) && Equal(wstr, "_identident2") && *s == L'i');
		}
		{// String
			using namespace pr::str;

			wchar_t const* src = LR"("string1" "str\"i\\ng2")", *s;
			char         aarr[20] = {};
			wchar_t      warr[20] = {};
			std::string  astr;
			std::wstring wstr;

			s = src;
			PR_EXPECT(ExtractString(aarr, s, L'\\', nullptr) && Equal(aarr, "string1") && *s == L' ');
			PR_EXPECT(ExtractStringC(aarr, ++s, L'\\', nullptr) && Equal(aarr, "string1str\"i\\ng2") && *s == L'"');

			s = src;
			PR_EXPECT(ExtractString(warr, s) && Equal(warr, "string1") && *s == L' ');
			PR_EXPECT(ExtractStringC(warr, ++s) && Equal(warr, "string1str\\") && *s == L'"');

			s = src;
			PR_EXPECT(ExtractString(astr, s, L'\\', nullptr) && Equal(astr, "string1") && *s == L' ');
			PR_EXPECT(ExtractStringC(astr, ++s, L'\\', nullptr) && Equal(astr, "string1str\"i\\ng2") && *s == L'"');

			s = src;
			PR_EXPECT(ExtractString(wstr, s) && Equal(wstr, "string1") && *s == L' ');
			PR_EXPECT(ExtractStringC(wstr, ++s) && Equal(wstr, "string1str\\") && *s == L'"');
		}
		{// Bool
			using namespace pr::str;

			char  src[] = "true false 1", *s = src;
			bool  bbool = 0;
			int   ibool = 0;
			float fbool = 0;

			PR_EXPECT(ExtractBool(bbool, s) && bbool);
			PR_EXPECT(ExtractBool(ibool, s) && ibool == 0);
			PR_EXPECT(ExtractBool(fbool, s) && fbool == 1.0f);
		}
		{// Int
			using namespace pr::str;

			char  c = 0;      unsigned char  uc = 0;
			short s = 0;      unsigned short us = 0;
			int   i = 0;      unsigned int   ui = 0;
			long  l = 0;      unsigned long  ul = 0;
			long long ll = 0; unsigned long long ull = 0;
			float  f = 0;     double d = 0;
			{
				char src[] = "0";
				PR_EXPECT(ExtractIntC(i,  0, src) && i == 0);
				PR_EXPECT(ExtractIntC(i,  8, src) && i == 0);
				PR_EXPECT(ExtractIntC(i, 10, src) && i == 0);
				PR_EXPECT(ExtractIntC(i, 16, src) && i == 0);
			}
			{
				char src[] = "\n -1.14 ";
				PR_EXPECT(ExtractIntC(c  ,10 ,src) && c   == (char)-1);
				PR_EXPECT(ExtractIntC(uc ,10 ,src) && uc  == (unsigned char)0xff);
				PR_EXPECT(ExtractIntC(s  ,10 ,src) && s   == (short)-1);
				PR_EXPECT(ExtractIntC(us ,10 ,src) && us  == (unsigned short)0xffff);
				PR_EXPECT(ExtractIntC(i  ,10 ,src) && i   == (int)-1);
				PR_EXPECT(ExtractIntC(ui ,10 ,src) && ui  == (unsigned int)0xffffffff);
				PR_EXPECT(ExtractIntC(l  ,10 ,src) && l   == (long)-1);
				PR_EXPECT(ExtractIntC(ul ,10 ,src) && ul  == (unsigned long)0xffffffff);
				PR_EXPECT(ExtractIntC(ll ,10 ,src) && ll  == (long long)-1);
				PR_EXPECT(ExtractIntC(ull,10 ,src) && ull == (unsigned long long)0xffffffffffffffffL);
				PR_EXPECT(ExtractIntC(f  ,10 ,src) && f   == (float)-1.0f);
				PR_EXPECT(ExtractIntC(d  ,10 ,src) && d   == (double)-1.0);
			}
			{
				char src[] = "0x1abcZ", *ptr = src;
				PR_EXPECT(ExtractInt(i,0,ptr) && i == 0x1abc);
				PR_EXPECT(*ptr == 'Z');
			}
			{
				char src[] = "0xdeadBeaf";
				PR_EXPECT(ExtractIntC(ll, 16, src) && ll == 0xdeadBeaf);
			}
			{
				char src[] = "0BFF0000";
				PR_EXPECT(ExtractIntC(ll, 16, src) && ll == 0x0BFF0000);
			}
		}
		{// Real
			using namespace pr::str;

			float f = 0; double d = 0; int i = 0;
			{
				char src[] = "\n 6.28 ";
				PR_EXPECT(ExtractRealC(f ,src) && FEql(f, 6.28f));
				PR_EXPECT(ExtractRealC(d ,src) && FEql(d, 6.28));
				PR_EXPECT(ExtractRealC(i ,src) && i == 6);
			}
			{
				char src[] = "-1.25e-4Z", *ptr = src;
				PR_EXPECT(ExtractReal(d, ptr) && d == -1.25e-4);
				PR_EXPECT(*ptr == 'Z');
			}
		}
		{// Arrays
			using namespace pr::str;

			{// Bool Array
				char src[] = "\n true 1 TRUE ";
				float f[3] = {0,0,0};
				PR_EXPECT(ExtractBoolArrayC(f, 3, src));
				PR_EXPECT(f[0] == 1.0f);
				PR_EXPECT(f[1] == 1.0f);
				PR_EXPECT(f[2] == 1.0f);
			}
			{// Int Array
				char src[] = "\n \t3  1 \n -2\t ";
				int i[3] = {0,0,0};
				unsigned int u[3] = {0,0,0};
				float        f[3] = {0,0,0};
				double       d[3] = {0,0,0};
				PR_EXPECT(ExtractIntArrayC(i, 3, 10, src) && i[0] == 3 && i[1] == 1 && i[2] == -2);
				PR_EXPECT(ExtractIntArrayC(u, 3, 10, src) && i[0] == 3 && i[1] == 1 && i[2] == -2);
				PR_EXPECT(ExtractIntArrayC(f, 3, 10, src) && FEql(f[0], 3.f) && FEql(f[1], 1.f) && FEql(f[2], -2.f));
				PR_EXPECT(ExtractIntArrayC(d, 3, 10, src) && FEql(d[0], 3.0) && FEql(d[1], 1.0) && FEql(d[2], -2.0));
			}
			{// Real Array
				char src[] = "\n 6.28\t6.28e0\n-6.28 ";
				float  f[3] = {0,0,0};
				double d[3] = {0,0,0};
				int    i[3] = {0,0,0};
				PR_EXPECT(ExtractRealArrayC(f, 3, src) && FEql(f[0], 6.28f) && FEql(f[1], 6.28f) && FEql(f[2], -6.28f));
				PR_EXPECT(ExtractRealArrayC(d, 3, src) && FEql(d[0], 6.280) && FEql(d[1], 6.280) && FEql(d[2], -6.280));
				PR_EXPECT(ExtractRealArrayC(i, 3, src) && i[0] == 6 && i[1] == 6 && i[2] == -6);
			}
		}
		{// Number
			using namespace pr::str;
				
			Number num;
			PR_EXPECT(ExtractNumberC(num, "0"       ) && num.ll() == 0);
			PR_EXPECT(ExtractNumberC(num, "+0"      ) && num.ll() == 0);
			PR_EXPECT(ExtractNumberC(num, "-0"      ) && num.ll() == 0);
			PR_EXPECT(ExtractNumberC(num, "+.0f"    ) && FEql(num.db(), +0.0));
			PR_EXPECT(ExtractNumberC(num, "-.1f"    ) && FEql(num.db(), -0.1));
			PR_EXPECT(ExtractNumberC(num, "1F"      ) && FEql(num.db(), 1.0 ));
			PR_EXPECT(ExtractNumberC(num, "3E-05F"  ) && FEql(num.db(), 3E-05));
			PR_EXPECT(ExtractNumberC(num, "12three" ) && num.ll() == 12);
			PR_EXPECT(ExtractNumberC(num, "0x123"   ) && num.ll() == 0x123);
			PR_EXPECT(ExtractNumberC(num, "0x123ULL") && num.ul() == 0x123ULL);
			PR_EXPECT(ExtractNumberC(num, "0x123L"  ) && num.ll() == 0x123LL);
			PR_EXPECT(ExtractNumberC(num, "0x123LL" ) && num.ll() == 0x123LL);

			PR_EXPECT(ExtractNumberC(num, "0b101010") && num.ll() == 0b101010LL);
			PR_EXPECT(ExtractNumberC(num, "0923.0"  ) && FEql(num.db(), 0923.0));
			PR_EXPECT(ExtractNumberC(num, "0199"    ) && num.ll() == 01); // because it's octal
			PR_EXPECT(ExtractNumberC(num, "0199", 10) && num.ll() == 199);
			PR_EXPECT(ExtractNumberC(num, "0x1.FEp1") && FEql(num.db(), 0x1.FEp1));

			PR_EXPECT(!ExtractNumberC(num, "0x.0"));
			PR_EXPECT(!ExtractNumberC(num, ".x0"));
			PR_EXPECT(!ExtractNumberC(num, "-x.0"));

			char    src0[] =  "-3.24e-39f";
			wchar_t src1[] = L"0x123abcUL";
			char    src2[] =  "01234567";
			wchar_t src3[] = L"-34567L";
		
			PR_EXPECT(ExtractNumberC(num, src0) && num.m_type == Number::EType::FP && FEql(num.db(), -3.24e-39));
			PR_EXPECT(ExtractNumberC(num, src1) && num.m_type == Number::EType::Int && num.ul() == 0x123abcULL);
			PR_EXPECT(ExtractNumberC(num, src2) && num.m_type == Number::EType::Int && num.ll() == 01234567);
			PR_EXPECT(ExtractNumberC(num, src3) && num.m_type == Number::EType::Int && num.ll() == -34567LL);
		}
		{// Null terminated string
			std::string str = "6.28 Ident";
			stringz_t sz = str;

			double val0;
			PR_EXPECT(ExtractReal(val0, sz) && FEql(val0, 6.28));
			PR_EXPECT(*sz == ' ');

			std::string ident;
			PR_EXPECT(ExtractIdentifier(ident, sz) && ident == "Ident");
			PR_EXPECT(*sz == '\0');
		}
		{// Sections
			std::string str = "{ stuff { nested } \"} string\" }end";
			stringz_t sz = str;

			std::string section;
			PR_EXPECT(ExtractSection(section, sz) && section == "{ stuff { nested } \"} string\" }");
			PR_EXPECT(*sz == 'e');
		}
	}
}
#endif
