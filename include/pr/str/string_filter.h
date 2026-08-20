//**********************************
// String Util
//  Copyright (c) Rylogic Ltd 2015
//**********************************
// Utility string functions that operate on:
//   std::string, std::wstring, pr::string<>, etc
//   char[], wchar_t[], etc
//   char*, wchar_t*, etc
// Note: char-array strings are not handled as special cases because there is no
// guarantee that the entire buffer is filled by the string, the null terminator
// may be midway through the buffer

#pragma once

#include <cmath>
#include <array>
#include <type_traits>
#include "pr/str/string_core.h"

namespace pr::str
{
	// A helper class for recognising literal strings in a stream of characters.
	struct InLiteral
	{
		// Notes:
		//  - Literal strings are closed automatically by newline characters. Higher level
		//    logic handles the unmatched quote character. This is needed for parsing inactive
		//    code blocks in the preprocessor, which ignores unclosed literal strings/characters.
		//  - Escape sequences don't have to be for single characters (i.e. unicode sequences) but
		//    that doesn't matter here because we only care about escaped quote characters.
		//  - 'm_in_literal' is the last reported state of "WithinLiteral".
		//  - 'm_in_literal_state' is an internal variable used to track the state changes

		// Flags for controlling the behaviour of the InLiteral class
		enum class EFlags
		{
			None = 0,

			// Expected escape sequences in the string
			Escaped = 1 << 0,

			// 'WithinLiteral' returns false for the initial and final quote characters
			ExcludeQuotes = 1 << 1,

			// New line characters end literal strings
			SingleLineStrings = 1 << 2,

			_flags_enum = 0,
		};
		friend constexpr EFlags operator | (EFlags lhs, EFlags rhs)
		{
			using ut = std::underlying_type_t<InLiteral::EFlags>;
			return static_cast<InLiteral::EFlags>(static_cast<ut>(lhs) | static_cast<ut>(rhs));
		}

		EFlags m_flags;
		int m_escape_character;
		int m_quote_character;
		bool m_in_literal_state;
		bool m_in_literal;
		bool m_escape;

		explicit InLiteral(EFlags flags = EFlags::Escaped, int escape_character = '\\') noexcept
			:m_flags(flags)
			,m_escape_character(escape_character)
			,m_quote_character()
			,m_in_literal_state(false)
			,m_in_literal(false)
			,m_escape(false)
		{}

		// True while within a string or character literal
		bool IsWithinLiteral() const
		{
			return m_in_literal;
		}

		// True while within an escape sequence
		bool IsWithinEscape() const
		{
			return m_escape;
		}

		// Consider the next character in the stream 'ch'.
		// Returns true if currently within a string/character literal
		template <CharType Char>
		bool WithinLiteral(Char ch) noexcept
		{
			if (m_in_literal_state)
			{
				if (m_escape)
				{
					// If escaped, then still within the literal
					m_escape = false;
					m_in_literal = true;
					return m_in_literal;
				}
				else if (ch == m_quote_character)
				{
					m_in_literal_state = false;
					m_in_literal = !Has(m_flags, EFlags::ExcludeQuotes); // terminating quote can be part of the literal
					return m_in_literal;
				}
				else if (ch == '\n' && Has(m_flags, EFlags::SingleLineStrings))
				{
					m_in_literal_state = false;
					m_in_literal = false; // terminating '\n' is not part of the literal
					return m_in_literal;
				}
				else
				{
					m_escape = (ch == m_escape_character) && Has(m_flags, EFlags::Escaped);
					m_in_literal = true;
					return m_in_literal;
				}
			}
			else if (ch == '\"' || ch == '\'')
			{
				m_quote_character = static_cast<char>(ch);
				m_in_literal_state = true;
				m_escape = false;
				m_in_literal = !Has(m_flags, EFlags::ExcludeQuotes); // first quote can be part of the literal
				return m_in_literal;
			}
			else
			{
				m_in_literal = false;
				return m_in_literal;
			}
		}
	
		// Helper for flags
		constexpr bool Has(EFlags lhs, EFlags rhs)
		{
			return (static_cast<int>(lhs) & static_cast<int>(rhs)) != 0;
		}
	};

	// A helper class for recognising line or block comments in a stream of characters.
	struct InComment
	{
		enum class EType { None = 0, Line = 1, Block = 2 };
		static int const LineContinuation = '\\';
		struct Patterns
		{
			std::wstring m_line_comment;
			std::wstring m_line_end;
			std::wstring m_block_beg;
			std::wstring m_block_end;

			Patterns(
				std::wstring_view line_comment = L"//",
				std::wstring_view line_end = L"\n",
				std::wstring_view block_beg = L"/*",
				std::wstring_view block_end = L"*/")
				:m_line_comment(line_comment)
				,m_line_end(line_end)
				,m_block_beg(block_beg)
				,m_block_end(block_end)
			{
				if (m_line_comment.empty() != m_line_end.empty())
					throw std::runtime_error("If line comments are detected, both start and end markers are required");
				if (m_block_beg.empty() != m_block_end.empty())
					throw std::runtime_error("If block comments are detected, both start and end markers are required");
			}
		};

		Patterns m_pat;
		InLiteral m_lit;
		EType m_comment;
		bool m_in_comment;
		bool m_escape;
		int m_emit;

		explicit InComment(Patterns pat = Patterns(), InLiteral::EFlags literal_flags = InLiteral::EFlags::Escaped | InLiteral::EFlags::SingleLineStrings)
			:m_pat(pat)
			,m_lit(literal_flags)
			,m_comment(EType::None)
			,m_in_comment()
			,m_escape()
			,m_emit()
		{}

		// True if the current state is "within comment"
		bool IsWithinComment() const
		{
			return m_in_comment;
		}

		// Processes the current character in 'src'.
		// Returns true if currently within a string/character literal
		template <typename TSrc> bool WithinComment(TSrc& src)
		{
			// This function requires 'src' because we need to look ahead
			// to say whether we're in a comment.

			switch (m_comment)
			{
			case EType::None:
				{
					if (m_lit.WithinLiteral(*src))
					{
					}
					else if (m_emit == 0 && !m_pat.m_line_comment.empty() && Match(src, m_pat.m_line_comment))
					{
						m_comment = EType::Line;
						m_emit = static_cast<int>(m_pat.m_line_comment.size());
						m_escape = false;
					}
					else if (m_emit == 0 && !m_pat.m_block_beg.empty() && Match(src, m_pat.m_block_beg))
					{
						m_comment = EType::Block;
						m_emit = static_cast<int>(m_pat.m_block_beg.size());
					}
					break;
				}
			case EType::Line:
				{
					if (*src == '\0')
					{
						m_comment = EType::None;
						m_emit = 0;
					}
					else if (m_emit == 0 && !m_escape && Match(src, m_pat.m_line_end))
					{
						m_comment = EType::None;
						m_emit = 0; // line comments don't include the line end
					}
					m_escape = *src == LineContinuation;
					break;
				}
			case EType::Block:
				{
					if (*src == '\0')
					{
						m_comment = EType::Block;
						m_emit = 0;
					}
					else if (m_emit == 0 && Match(src, m_pat.m_block_end))
					{
						m_comment = EType::None;
						m_emit = static_cast<int>(m_pat.m_block_end.size());
					}
					break;
				}
			default:
				{
					throw std::runtime_error("Unknown comment state");
				}
			}

			m_in_comment = m_comment != EType::None || m_emit != 0;
			m_emit -= static_cast<int>(m_emit != 0);
			return m_in_comment;
		}

	private:

		// True if 'src' starts with 'pattern'
		template <typename TSrc> static bool Match(TSrc& src, std::wstring_view pattern)
		{
			int i = 0, iend = static_cast<int>(pattern.size());
			for (; i != iend && src[i] == pattern[i]; ++i) {}
			return i == iend;
		}
	};

	// A helper class for escaping a string to a C-string
	template <CharType Char>
	struct Escape
	{
		// 'Char' is assumed to be a multi-byte encoding, either utf-8 (char8_t or char) or utf-16 (char16_t or wchar)
		enum class EEscSeq { None, Hex2, Octal3, Unicode4, Unicode8 };
		static constexpr int HighBit = 1 << (8*sizeof(Char) -  1);
		static constexpr int MaxLiteralLength = 8;
		using char_t = Char;

		char_t m_escape_character;
		char_t m_buf[MaxLiteralLength + 1];
		int m_buf_count;

		explicit Escape(char_t escape_character = '\\')
			:m_escape_character(escape_character)
			,m_buf()
			,m_buf_count()
		{}

		// Append characters to 'out' after translating escape sequences
		template <StringType Str>
		void Translate(char_t ch, Str& out, size_t& len)
		{
			switch (ch)
			{
			case '\a': Append(out, "\\a", len); break;
			case '\b': Append(out, "\\b", len); break;
			case '\f': Append(out, "\\f", len); break;
			case '\n': Append(out, "\\n", len); break;
			case '\r': Append(out, "\\r", len); break;
			case '\t': Append(out, "\\t", len); break;
			case '\v': Append(out, "\\v", len); break;
			case '\?': Append(out, "\\?", len); break;
			case '\'': Append(out, "\\\'", len); break;
			case '\"': Append(out, "\\\"", len); break;
			case '\\': Append(out, "\\\\", len); break;
			default:
				{
					// If not an encoded character
					if ((HighBit & ch) == 0)
					{
						Append(out, ch, len);
					}
					else
					{
						using converter_t = convert_utf<char_t, char32_t>;
						converter_t cvt;

						// Buffer character code units until we have a complete code point.
						m_buf[m_buf_count++] = ch;

						// Attempt to convert to utf-32... if incomplete, wait for more data
						char32_t c32;
						auto istr = std::basic_string_view<char_t>(&m_buf[0], m_buf_count);
						auto r = cvt(istr, [&](auto s, auto) { c32 = *s; });
						if (r == converter_t::error)
							throw std::runtime_error("Unicode encoding error");
						if (r == converter_t::partial)
							break;

						// Emit the canonical escape width so '\u' stays 4 hex digits and '\U' stays 8.
						char_t code[9] = {};
						auto const target_width = c32 <= 0xFFFF ? 4U : 8U;
						if (c32 <= 0xFFFF)
						{
							char_traits<char_t>::uitostr(c32, &code[0], _countof(code), 16);
							Append(out, "\\u", len);
						}
						else
						{
							char_traits<char_t>::uitostr(c32, &code[0], _countof(code), 16);
							Append(out, "\\U", len);
						}
						for (size_t i = char_traits<char_t>::length(code); i < target_width; ++i)
						{
							Append(out, '0', len);
						}
						Append(out, code, len);
						break;
					}
				}
			}
		}
		template <StringType Str>
		void Translate(Char ch, Str& out)
		{
			auto len = Size(out);
			Translate(ch, out, len);
		}
	};

	// A helper class for translating a C-string to unescaped characters
	template <CharType Char>
	struct Unescape
	{
		// 'Char' is assumed to be a multi-byte encoding, either utf-8 (char8_t or char) or utf-16 (char16_t or wchar)
		enum class EEscSeq { None, Hex2, Octal3, Unicode4, Unicode8 };
		static int const MaxLiteralLength = 8;
		using char_t = Char;

		char_t m_escape_character;
		char_t m_buf[MaxLiteralLength + 1];
		int m_buf_count;
		EEscSeq m_seq;
		bool m_escape;

		explicit Unescape(char_t escape_character = '\\')
			:m_escape_character(escape_character)
			,m_buf()
			,m_buf_count()
			,m_seq(EEscSeq::None)
			,m_escape(false)
		{}

		// True while in an escape sequence. (Used externally when looping over characters between quotes)
		bool WithinEscapeSequence() const
		{
			return m_escape;
		}

		// Append characters to 'out' after translating escape sequences
		template <StringType Str>
		void Translate(char_t ch, Str& out, size_t& len)
		{
			for (; m_escape; )
			{
				// Unicode character in hex, e.g. '\x1234'
				if (ch == 'x' || ch == 'u' || ch == 'U' || m_seq == EEscSeq::Hex2 || m_seq == EEscSeq::Unicode4 || m_seq == EEscSeq::Unicode8)
				{
					// This is the start of a hex unicode character sequence
					if (m_seq == EEscSeq::None)
					{
						assert(m_buf_count == 0 && "Buffer should have been reset to 0 at the end of the last code");
						m_seq =
							ch == 'x' ? EEscSeq::Hex2 :
							ch == 'u' ? EEscSeq::Unicode4 :
							ch == 'U' ? EEscSeq::Unicode8 :
							throw std::runtime_error("Unknown unicode sequence identifier");
						return;
					}

					// Buffer the hex digit
					if (IsHexDigit(ch))
						m_buf[m_buf_count++] = ch;
					else
						throw std::runtime_error("Invalid hex digit in character code");

					// End of the sequence?
					if ((m_seq == EEscSeq::Hex2     && m_buf_count == 2) ||
						(m_seq == EEscSeq::Unicode4 && m_buf_count == 4) ||
						(m_seq == EEscSeq::Unicode8 && m_buf_count == 8))
					{
						// Terminate the string in 'm_buf'
						m_buf[m_buf_count] = 0;

						// Convert the character code to an integer
						char32_t code = static_cast<char32_t>(char_traits<char_t>::strtoul(&m_buf[0], nullptr, 16));
						auto istr = std::basic_string_view<char32_t>(&code, 1);

						// Convert from char32_t to to utf-8/utf-16 based on 'char_t'
						convert_utf<char32_t, char_t> cvt;
						cvt(istr, [&](auto s, auto e){ for (; s != e; ++s) Append(out, *s, len); });

						// Leave the 'escaped' state
						m_escape = false;
						m_seq = EEscSeq::None;
						m_buf_count = 0;
					}
					return;
				}

				// Unicode character in octal, e.g. '\012'
				if (ch == '0' || ch == '1' || ch == '2' || ch == '3' || m_seq == EEscSeq::Octal3)
				{
					// This is the start of the octal unicode character
					if (m_seq == EEscSeq::None)
					{
						// The first octal digit is part of the code
						assert(m_buf_count == 0 && "Buffer should have been reset to 0 at the end of the last code");
						m_buf[m_buf_count++] = ch;
						m_seq = EEscSeq::Octal3;
						return;
					}

					// Buffer the octal digit
					if (IsOctDigit(ch))
						m_buf[m_buf_count++] = ch;
					else
						throw std::runtime_error("Invalid octal digit in character code");

					// End of the sequence?
					if (m_seq == EEscSeq::Octal3 && m_buf_count == 3)
					{
						// Terminate the string in 'm_buf'
						m_buf[m_buf_count] = 0;

						// Convert the character code to an integer value
						char32_t code = static_cast<char32_t>(char_traits<char_t>::strtoul(&m_buf[0], nullptr, 8));
						auto istr = std::basic_string_view<char32_t>(&code, 1);

						// Convert from char32_t to to utf-8/utf-16 based on 'char_t'
						convert_utf<char32_t, char_t> cvt;
						cvt(istr, [&](auto s, auto e) { for (; s != e; ++s) Append(out, *s, len); });

						// Leave the 'escaped' state
						m_escape = false;
						m_seq = EEscSeq::None;
						m_buf_count = 0;
					}
					return;
				}

				// Single character escape sequence
				m_escape = false;
				switch (ch)
				{
				case 'a':  Append(out, '\a', len); return;
				case 'b':  Append(out, '\b', len); return;
				case 'f':  Append(out, '\f', len); return;
				case 'n':  Append(out, '\n', len); return;
				case 'r':  Append(out, '\r', len); return;
				case 't':  Append(out, '\t', len); return;
				case 'v':  Append(out, '\v', len); return;
				case '?':  Append(out, '\?', len); return;
				case '\'': Append(out, '\'', len); return;
				case '\"': Append(out, '\"', len); return;
				case '\\': Append(out, '\\', len); return;
				default: throw std::runtime_error("Unknown escape sequence");
				}
			}
			
			// 'ch' is not part of an escape sequence
			if (ch == m_escape_character)
				m_escape = true;
			else
				Append(out, ch, len);
		}
		template <StringType Str>
		void Translate(char_t ch, Str& out)
		{
			auto len = Size(out);
			Translate(ch, out, len);
		}
	};
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::str
{
	PRUnitTestClass(StringFilterTests)
	{
		PRUnitTestMethod(InLiteral, Quick)
		{
			{
				// Escaped quotes are ignored
				char const* src = " \"\\\"\" ";
				bool const exp[] = {0,1,1,1,1,0};

				InLiteral lit;
				for (int i = 0; *src != 0; ++i, ++src)
				{
					if (lit.WithinLiteral(*src) == exp[i]) continue;
					PR_EXPECT(lit.WithinLiteral(*src) == exp[i]);
				}
				PR_EXPECT(*src == '\0');
			}
			{
				// Escape sequences are not always 1 character, but it doesn't
				// matter because we only care about escaped quotes.
				char const* src = " \"\\xB1\" ";
				bool const exp[] = {0,0,1,1,1,1,0,0};

				// Don't include the quotes
				InLiteral lit(InLiteral::EFlags::Escaped | InLiteral::EFlags::ExcludeQuotes);
				for (int i = 0; *src != 0; ++i, ++src)
				{
					if (lit.WithinLiteral(*src) == exp[i]) continue;
					PR_EXPECT(lit.WithinLiteral(*src) == exp[i]);
				}
				PR_EXPECT(*src == '\0');
			}
			{
				// Literals must match " to " and ' to '
				char const* src = "\"'\" '\"' ";
				bool const exp[] = {1,1,1,0,1,1,1,0};

				InLiteral lit;
				for (int i = 0; *src != 0; ++i, ++src)
				{
					if (lit.WithinLiteral(*src) == exp[i]) continue;
					PR_EXPECT(lit.WithinLiteral(*src) == exp[i]);
				}
				PR_EXPECT(*src == '\0');
			}
			{
				// Literals *are* closed by '\n'
				char const* src = "\" '\n ";
				bool const exp[] = {1,1,1,0,0};

				InLiteral lit(InLiteral::EFlags::Escaped | InLiteral::EFlags::SingleLineStrings);
				for (int i = 0; *src != 0; ++i, ++src)
				{
					if (lit.WithinLiteral(*src) == exp[i]) continue;
					PR_EXPECT(lit.WithinLiteral(*src) == exp[i]);
				}
				PR_EXPECT(*src == '\0');
			}
			{
				// Literals are not closed by EOS
				char const* src = "\" ";
				bool const exp[] = {1,1};

				InLiteral lit;
				for (int i = 0; *src != 0; ++i, ++src)
				{
					if (lit.WithinLiteral(*src) == exp[i]) continue;
					PR_EXPECT(lit.WithinLiteral(*src) == exp[i]);
				}
				PR_EXPECT(lit.WithinLiteral(*src));
				PR_EXPECT(*src == '\0');
			}
		}
		PRUnitTestMethod(InComment, Quick)
		{
			{
				// Simple block comment
				char const* src = " /**/ ";
				bool const exp[] = {0,1,1,1,1,0};

				InComment com;
				for (int i = 0; *src != 0; ++i, ++src)
				{
					if (com.WithinComment(src) == exp[i]) continue;
					PR_EXPECT(com.WithinComment(src) == exp[i]);
				}
				PR_EXPECT(*src == '\0');
			}
			{
				// No substring matching within block comment markers
				char const* src = "/*/*/ /**/*/";
				bool const exp[] = {1,1,1,1,1,0,1,1,1,1,0,0,0};

				InComment com;
				for (int i = 0; *src != 0; ++i, ++src)
				{
					if (com.WithinComment(src) == exp[i]) continue;
					PR_EXPECT(com.WithinComment(src) == exp[i]);
				}
				PR_EXPECT(*src == '\0');
			}
			{
				// Line comment ends at unescaped new line (exclusive)
				char const* src = " // \\\n \n ";
				bool const exp[] = {0,1,1,1,1,1,1,0,0,0};

				InComment com;
				for (int i = 0; *src != 0; ++i, ++src)
				{
					if (com.WithinComment(src) == exp[i]) continue;
					PR_EXPECT(com.WithinComment(src) == exp[i]);
				}
				PR_EXPECT(*src == '\0');
			}
			{
				// Line comment ends at EOS
				char const* src = " // ";
				bool const exp[] = {0,1,1,1,0};

				InComment com;
				for (int i = 0; *src != 0; ++i, ++src)
				{
					if (com.WithinComment(src) == exp[i]) continue;
					PR_EXPECT(com.WithinComment(src) == exp[i]);
				}
				PR_EXPECT(*src == '\0');
			}
			{
				// No comments within literal strings
				char const* src = " \"// /* */\" ";
				bool const exp[] = {0,0,0,0,0,0,0,0,0,0,0,0};

				InComment com;
				for (int i = 0; *src != 0; ++i, ++src)
				{
					if (com.WithinComment(src) == exp[i]) continue;
					PR_EXPECT(com.WithinComment(src) == exp[i]);
				}
				PR_EXPECT(*src == '\0');
			}
			{
				// Ignore literal strings within comments
				char const* src = " /* \" */ // \" \n ";
				bool const exp[] = {0,1,1,1,1,1,1,1,0,1,1,1,1,1,0,0};

				InComment com;
				for (int i = 0; *src != 0; ++i, ++src)
				{
					if (com.WithinComment(src) == exp[i]) continue;
					PR_EXPECT(com.WithinComment(src) == exp[i]);
				}
				PR_EXPECT(*src =='\0');
			}
		}
		PRUnitTestMethod(Escape, Quick)
		{
			// Cover the Unicode width bug at the code-point formatting boundary.
			// '\u' escapes must pad to 4 digits and '\U' escapes must pad to 8.
			auto test_escape = [&](std::u8string const& src, char const* expected)
			{
				std::string out;
				size_t len = 0;

				Escape<char> esc;
				for (auto ch : src)
					esc.Translate(ch, out, len);

				PR_EXPECT(out == expected);
			};

			std::u8string str = char8_ptr(u8"abc\123\u00b1\a\b\f\n\r\t\v\\\"\'\?");
			std::string res = "abcS\\u00b1\\a\\b\\f\\n\\r\\t\\v\\\\\\\"\\\'\\?";
			std::string out;

			size_t len = 0;
			Escape<char> esc;
			for (auto ch : str)
				esc.Translate(ch, out, len);

			PR_EXPECT(out == res);
			// Overlong UTF-8 is only used here to reach the 1-digit hex formatting path.
			test_escape(std::u8string{ static_cast<char8_t>(0xC0), static_cast<char8_t>(0x8F) }, "\\u000f");
			test_escape(std::u8string(u8"\u0123"), "\\u0123");
			test_escape(std::u8string(u8"\u1234"), "\\u1234");
			test_escape(std::u8string(u8"\U0001F4A9"), "\\U0001f4a9");
		}
		PRUnitTestMethod(Unescape, Quick)
		{
			std::string str = "abc\\123\\u00b1\\a\\b\\f\\n\\r\\t\\v\\\\\\\"\\\'\\?";
			std::u8string res = char8_ptr(u8"abc\123\u00b1\a\b\f\n\r\t\v\\\"\'\?");
			std::u8string out;

			size_t len = 0;
			Unescape<char> esc;
			for (auto ch : str)
				esc.Translate(ch, out, len);

			PR_EXPECT(out == res);
		}
	};
}
#endif
