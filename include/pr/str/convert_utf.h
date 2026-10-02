//**********************************
// UTF convert
//  Copyright (c) Rylogic Ltd 2019
//**********************************
#pragma once
#include <type_traits>
#include <string_view>
#include <stdexcept>

namespace pr::str
{
	// A sane implementation of codecvt
	template <typename InChar, typename OuChar>
	struct convert_utf
	{
		#if !defined(__cpp_char8_t)
		using char8_t = unsigned char;
		#endif

		// Notes:
		//  - 'InChar' and 'OutChar' are assumed to be multi-byte unicode character units.
		//  - char8_t/char == utf-8
		//  - char16_t/wchar_t == utf-16
		//  - char32_t == utf-32
		//
		// Usage:
		//  convert_utf<char8_t, char16_t> cvt;
		//  for (char8_t const* c = &u8str[0]; *c != 0; ++c)
		//     cvt(*c, [&](char16_t const* s, char16_t const* e){ outstr.insert(end(outstr), s, e); });
		using in_char_t = InChar;
		using ou_char_t = OuChar;
		enum { ok, partial, error };

		// [2: number of in_char_t needed to complete the code point, 30: buffered code point value so far]
		// For UTF-8 input, bits [28,30) hold the sequence length minus one, and bits [0,21) hold the value.
		// The state is zero between whole code points.
		char32_t m_ibuf;

		convert_utf()
			:m_ibuf()
		{}

		// Convert text from 'in_char_t' to 'ou_char_t'.
		// 'Out' is a function that receives the converted (multi-byte) characters:
		//  void Out(out_char_t const* b, out_char_t const* e) {}
		// 'out' is only called on whole code points. This means converting from utf8 to utf8 is
		// not a no-op, it can be used to validate the encoding of a sequence of utf8 text.
		// Malformed input (e.g. overlong or surrogate UTF-8 forms, or unpaired UTF-16 surrogates) is an 'error'.
		// Input values that are not Unicode scalar values (surrogates or values above U+10FFFF) are output as 'dflt'.
		// Returns 'ok', 'partial', or 'error'.
		template <typename Out>
		int operator()(in_char_t c, Out out, ou_char_t dflt = '_')
		{
			// State layout helpers. See 'm_ibuf'.
			static constexpr char32_t shft = 30;
			static constexpr char32_t mask = (1 << shft) - 1;
			static constexpr char32_t len_shft = 28;
			static constexpr char32_t value_mask = (1 << 21) - 1;
			auto Count = [](char32_t x) { return (x >> shft) & 0b11; };
			auto IsScalar = [](char32_t x) { return x < 0xD800u || (x >= 0xE000u && x <= 0x10FFFFu); };

			// Convert from 'utf-8' to 'char32_t'
			if constexpr (std::is_same_v<in_char_t, char> || std::is_same_v<in_char_t, char8_t>)
			{
				// Ensure the code unit is unsigned
				char8_t ch = static_cast<char8_t>(c);

				// Single byte sequence
				if (ch < 0b1000'0000u)
				{
					// Encoding error
					if (Count(m_ibuf) != 0)
					{
						m_ibuf = 0;
						return error;
					}
					m_ibuf = ch;
				}
				// Lead byte. The number of continuation bytes is recorded twice: once as the count still needed,
				// and once as the sequence length, so that overlong forms can be detected when the code point is complete.
				else if (ch >= 0b1100'0000u)
				{
					// Encoding error
					if (Count(m_ibuf) != 0)
					{
						m_ibuf = 0;
						return error;
					}
					// Lead byte of 2-byte sequence
					else if (ch < 0b1110'0000u)
					{
						m_ibuf = (1 << shft) | (1 << len_shft) | (ch & 0b0001'1111u);
					}
					// Lead byte of 3-byte sequence
					else if (ch < 0b1111'0000u)
					{
						m_ibuf = (2 << shft) | (2 << len_shft) | (ch & 0b0000'1111u);
					}
					// Lead byte of 4-byte sequence
					else if (ch < 0b1111'1000u)
					{
						m_ibuf = (3 << shft) | (3 << len_shft) | (ch & 0b0000'0111u);
					}
					// Invalid UTF-8 code unit
					else
					{
						return error;
					}
				}
				// Trailing byte
				else
				{
					char32_t count = Count(m_ibuf);

					// Encoding error
					if (count == 0)
					{
						m_ibuf = 0;
						return error;
					}
					// Append trailing byte
					else
					{
						m_ibuf = ((count - 1) << shft) | (m_ibuf & (0b11u << len_shft)) | ((m_ibuf & value_mask) << 6) | (ch & 0b0011'1111u);
					}
				}

				// Once complete, reject values that use more bytes than needed, and values that are not Unicode scalar values.
				// UTF-8 cannot legally encode either, so these are encoding errors rather than unrepresentable values.
				if (Count(m_ibuf) == 0)
				{
					static constexpr char32_t min_value[] = { 0, 0x80u, 0x800u, 0x10000u };
					auto len = (m_ibuf >> len_shft) & 0b11u;
					m_ibuf &= value_mask;
					if (m_ibuf < min_value[len] || !IsScalar(m_ibuf))
					{
						m_ibuf = 0;
						return error;
					}
				}
			}

			// Convert from 'utf-16' to 'char32_t'
			if constexpr (std::is_same_v<in_char_t, wchar_t> || std::is_same_v<in_char_t, char16_t>)
			{
				// Ensure the code unit is unsigned
				char16_t ch = static_cast<char16_t>(c);

				// Not a surrogate
				if (ch < 0xD800u || ch >= 0xE000u)
				{
					// Encoding error - unpaired hi surrogate
					if (Count(m_ibuf) != 0)
					{
						m_ibuf = 0;
						return error;
					}
					m_ibuf = ch;
				}
				// Hi surrogate. A lo surrogate must follow.
				else if (ch < 0xDC00u)
				{
					// Encoding error - two successive hi surrogates
					if (Count(m_ibuf) != 0)
					{
						m_ibuf = 0;
						return error;
					}
					m_ibuf = (1u << shft) | ((ch - 0xD800u) << 10);
				}
				// Lo surrogate. Only valid after a hi surrogate.
				else
				{
					// Encoding error - unpaired lo surrogate
					if (Count(m_ibuf) == 0)
					{
						m_ibuf = 0;
						return error;
					}
					m_ibuf = ((m_ibuf & mask) | (ch - 0xDC00u)) + 0x10000u;
				}
			}

			// Convert from 'utf-32' to 'char32_t'
			if constexpr (std::is_same_v<in_char_t, char32_t>)
			{
				// Encoding error
				if (Count(c) != 0)
				{
					m_ibuf = 0;
					return error;
				}
				m_ibuf = c;
			}

			// If there are still code units required, wait for more data
			if (Count(m_ibuf) != 0)
				return partial;

			// Values that are not Unicode scalar values cannot be encoded in any UTF form
			if (!IsScalar(m_ibuf))
			{
				out(&dflt, &dflt + 1);
				m_ibuf = 0;
				return ok;
			}

			// Convert from 'char32_t' to 'utf-8'
			if constexpr (std::is_same_v<ou_char_t, char> || std::is_same_v<ou_char_t, char8_t>)
			{
				ou_char_t obuf[4] = {};
				if (m_ibuf < 0x80u)
				{
					obuf[0] = static_cast<ou_char_t>(m_ibuf & 0xFF);
					out(&obuf[0], &obuf[0] + 1);
				}
				else if (m_ibuf < 0x0800u)
				{
					obuf[0] = static_cast<ou_char_t>(((m_ibuf >> 6) & 0x1F) | 0xC0);
					obuf[1] = static_cast<ou_char_t>(((m_ibuf >> 0) & 0x3F) | 0x80);
					out(&obuf[0], &obuf[0] + 2);
				}
				else if (m_ibuf < 0x10000u)
				{
					obuf[0] = static_cast<ou_char_t>(((m_ibuf >> 12) & 0x0F) | 0xE0);
					obuf[1] = static_cast<ou_char_t>(((m_ibuf >>  6) & 0x3F) | 0x80);
					obuf[2] = static_cast<ou_char_t>(((m_ibuf >>  0) & 0x3F) | 0x80);
					out(&obuf[0], &obuf[0] + 3);
				}
				else
				{
					obuf[0] = static_cast<ou_char_t>(((m_ibuf >> 18) & 0x07) | 0xF0);
					obuf[1] = static_cast<ou_char_t>(((m_ibuf >> 12) & 0x3F) | 0x80);
					obuf[2] = static_cast<ou_char_t>(((m_ibuf >>  6) & 0x3F) | 0x80);
					obuf[3] = static_cast<ou_char_t>(((m_ibuf >>  0) & 0x3F) | 0x80);
					out(&obuf[0], &obuf[0] + 4);
				}
			}

			// Convert from 'char32_t' to 'utf-16'
			if constexpr (std::is_same_v<ou_char_t, wchar_t> || std::is_same_v<ou_char_t, char16_t>)
			{
				ou_char_t obuf[2] = {};
				if (m_ibuf < 0x10000u)
				{
					obuf[0] = static_cast<ou_char_t>(m_ibuf & 0xFFFFu);
					out(&obuf[0], &obuf[0] + 1);
				}
				else
				{
					m_ibuf -= 0x010000u;
					obuf[0] = static_cast<ou_char_t>(((m_ibuf >> 10) & 0x03FFu) + 0xD800u);
					obuf[1] = static_cast<ou_char_t>(((m_ibuf >>  0) & 0x03FFu) + 0xDC00u);
					out(&obuf[0], &obuf[0] + 2);
				}
			}

			// Convert from 'char32_t' to 'utf-32'
			if constexpr (std::is_same_v<ou_char_t, char32_t>)
			{
				out(&m_ibuf, &m_ibuf + 1);
			}

			m_ibuf = 0;
			return ok;
		}

		// Convert a string.
		// 'Out' is a function that receives the converted (multi-byte) characters:
		//  void Out(out_char_t const* b, out_char_t const* e) {}
		// 'out' is only called on whole code points. This means converting from utf8 to utf8 is
		// not a no-op, it can be used to validate the encoding of a sequence of utf8 text.
		// Returns 'ok', 'partial', or 'error'.
		template <typename Out>
		int operator()(std::basic_string_view<InChar> istr, Out out, ou_char_t dflt = '_')
		{
			int result = ok;
			for (auto ch : istr)
			{
				result = (*this)(ch, out, dflt);
				if (result == error)
					break;
			}
			return result;
		}

		// Convert a whole string and append it to 'out'.
		// Throws if the encoding is invalid or 'istr' ends part way through a code point.
		template <typename StrOut>
		StrOut& conv(std::basic_string_view<InChar> istr, StrOut& out, ou_char_t dflt = '_')
		{
			// Convert each code unit, appending whole code points to 'out'
			auto append = [&](ou_char_t const* s, ou_char_t const* e)
			{
				for (; s != e; ++s)
					out.push_back(*s);
			};
			for (auto ch : istr)
			{
				if ((*this)(ch, append, dflt) == error)
					throw std::runtime_error("Invalid character encoding");
			}

			// The state is only zero between whole code points, so anything left over is an incomplete sequence
			if (m_ibuf != 0)
			{
				m_ibuf = 0;
				throw std::runtime_error("Incomplete character encoding");
			}
			return out;
		}
		template <typename StrOut>
		StrOut conv(std::basic_string_view<InChar> istr, ou_char_t dflt = '_')
		{
			StrOut out;
			return conv(istr, out, dflt);
		}

		// Convert a string to a 'StrOut'. See 'conv'.
		template <typename StrOut>
		static StrOut& convert(std::basic_string_view<InChar> istr, StrOut& out, ou_char_t dflt = '_')
		{
			convert_utf<InChar,OuChar> cvt;
			return cvt.conv<StrOut>(istr, out, dflt);
		}
		template <typename StrOut>
		static StrOut convert(std::basic_string_view<InChar> istr, ou_char_t dflt = '_')
		{
			StrOut out;
			return convert(istr, out, dflt);
		}
	};
}
