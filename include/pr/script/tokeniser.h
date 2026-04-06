//**********************************
// Script
//  Copyright (c) Rylogic Ltd 2015
//**********************************

#pragma once

#include "pr/script/forward.h"
#include "pr/script/fail_policy.h"
#include "pr/script/token.h"
#include "pr/script/script_core.h"

namespace pr::script
{
	// An interface/base class for a source of tokens
	struct TokenSrc
	{
		virtual ~TokenSrc() {}
			
		// Pointer-like interface
		virtual Token const& operator * () const = 0;
		virtual TokenSrc& operator ++() = 0;
		TokenSrc& operator += (int n)
		{
			for (;n--;) ++*this;
			return *this;
		}
	};
		
	// C Tokeniser
	struct Tokeniser :TokenSrc
	{
		Src&  m_src;  // The character stream to read from
		Token m_tok;  // The token last read from the stream

		Tokeniser(Src& src)
			:m_src(src)
			,m_tok()
		{
			seek();
		}
		Tokeniser(Tokeniser&& rhs) noexcept
			:TokenSrc(std::move(rhs))
			,m_src(rhs.m_src)
			,m_tok(std::move(rhs.m_tok))
		{}
		Tokeniser(Tokeniser const& rhs)
			:TokenSrc(rhs)
			,m_src(rhs.m_src)
			,m_tok(rhs.m_tok)
		{}

		// Pointer-like interface
		Token const& operator * () const override
		{
			return m_tok;
		}
		TokenSrc& operator ++() override
		{
			seek();
			return *this;
		}

	private:

		// Advance to the next token to output
		void seek()
		{
			auto& src = m_src;

			// Line space does not generate tokens
			EatLineSpace(src, 0, 0);
			switch (*src)
			{
				case '\0':
				{
					m_tok = Token(EToken::EndOfStream);
					break;
				}
				case '\n':
				{
					m_tok = Token(ESymbol::NewLine, src.Location().Line()); // save the index of the line following this '\n'
					++src;
					break;
				}
				case '_':
				case 'a': case 'b': case 'c': case 'd': case 'e': case 'f': case 'g': case 'h': case 'i':
				case 'j': case 'k': case 'l': case 'm': case 'n': case 'o': case 'p': case 'q': case 'r':
				case 's': case 't': case 'u': case 'v': case 'w': case 'x': case 'y': case 'z':
				case 'A': case 'B': case 'C': case 'D': case 'E': case 'F': case 'G': case 'H': case 'I':
				case 'J': case 'K':           case 'M': case 'N': case 'O': case 'P': case 'Q': case 'R':
				case 'S': case 'T': case 'U': case 'V': case 'W': case 'X': case 'Y': case 'Z':
				tokeniser_extract_identitier: // extract an identifier that might be a keyword
				{
					// Read the identifier
					auto len = 0;
					BufferIdentifier(src, 0, &len);

					// If the identifier is a keyword, create a keyword token
					auto hash = src.Hash(0, len);
					if (Enum<EKeyword>::IsValue(hash))
					{
						m_tok = Token(s_cast<EKeyword>(hash));
						src += len;
					}
					// otherwise, it's an identifier token.
					else
					{
						m_tok = Token(EToken::Identifier, src.ReadN(len), s_cast<int64_t>(hash));
					}
					break;
				}
				case 'L': // might be a char literal, string literal, or an identifier
				{
					if (src[1] == '\'' || src[1] == '\"') goto tokeniser_extract_str_literal;
					goto tokeniser_extract_identitier;
				}
				case '0': case '1': case '2': case '3': case '4':
				case '5': case '6': case '7': case '8': case '9':
				tokeniser_extract_constant: // Extract a numeric constant
				{
					Number num;
					if (!str::ExtractNumber(num, src))
						throw ScriptException(EResult::SyntaxError, src.Location(), "Invalid numeric constant");

					m_tok = num.m_type == Number::EType::FP ? Token(EConstant::FloatingPoint, num.db()) : Token(EConstant::Integral, num.ll());
					break;
				}
				case '\'':
				case '\"':
				tokeniser_extract_str_literal: // Extract a literal c-string (possibly prefixed with 'L')
				{
					auto is_wide = *src == 'L' ? ++src, true : false;
					auto is_char = *src == '\'';

					string_t str;
					if (!str::ExtractString(str, src, L'\\')) throw ScriptException(EResult::SyntaxError, src.Location(), "Invalid literal constant");
					if (is_char) m_tok = Token(EConstant::Integral, int64_t(str[0])); // char literals are actually integral constants
					else if (is_wide) m_tok = Token(EConstant::WStringLiteral, str);
					else              m_tok = Token(EConstant::StringLiteral, str);
					break;
				}
				case '.':
				{
					if (src[1] == '.' && src[2] == '.') { m_tok = Token(ESymbol::Ellipsis); src += 3; break; }
					if (str::IsDigit(src[1])) goto tokeniser_extract_constant; // '.' can be the start of a number
					m_tok = Token(ESymbol::Dot); ++src;
					break;
				}
				case '<':
				{
					if (src[1] == '<' && src[2] == '=') { m_tok = Token(ESymbol::ShiftLAssign); src += 3; break; }
					if (src[1] == '<') { m_tok = Token(ESymbol::ShiftL); src += 2; break; }
					if (src[1] == '=') { m_tok = Token(ESymbol::LessEql); src += 2; break; }
					m_tok = Token(ESymbol::LessThan); ++src;
					break;
				}
				case '>':
				{
					if (src[1] == '>' && src[2] == '=') { m_tok = Token(ESymbol::ShiftRAssign); src += 3; break; }
					if (src[1] == '>') { m_tok = Token(ESymbol::ShiftR); src += 2; break; }
					if (src[1] == '=') { m_tok = Token(ESymbol::GtrEql); src += 2; break; }
					m_tok = Token(ESymbol::GtrThan); ++src;
					break;
				}
				case '&':
				{
					if (src[1] == '&') { m_tok = Token(ESymbol::LogicalAnd); src += 2; break; }
					if (src[1] == '=') { m_tok = Token(ESymbol::BitAndAssign); src += 2; break; }
					m_tok = Token(ESymbol::AddressOf); ++src;
					break;
				}
				case '|':
				{
					if (src[1] == '|') { m_tok = Token(ESymbol::LogicalOr); src += 2; break; }
					if (src[1] == '=') { m_tok = Token(ESymbol::BitOrAssign); src += 2; break; }
					m_tok = Token(ESymbol::BitOr); ++src;
					break;
				}
				case '^':
				{
					if (src[1] == '=') { m_tok = Token(ESymbol::BitXorAssign); src += 2; break; }
					m_tok = Token(ESymbol::BitXor); ++src;
					break;
				}
				case '!':
				{
					if (src[1] == '=') { m_tok = Token(ESymbol::NotEqual); src += 2; break; }
					m_tok = Token(ESymbol::Not); ++src;
					break;
				}
				case '=':
				{
					if (src[1] == '=') { m_tok = Token(ESymbol::Equal); src += 2; break; }
					m_tok = Token(ESymbol::Assign); ++src;
					break;
				}
				case '+':
				{
					if (src[1] == '+') { m_tok = Token(ESymbol::Increment); src += 2; break; }
					if (src[1] == '=') { m_tok = Token(ESymbol::AddAssign); src += 2; break; }
					m_tok = Token(ESymbol::Plus); ++src;
					break;
				}
				case '-':
				{
					if (src[1] == '-') { m_tok = Token(ESymbol::Decrement); src += 2; break; }
					if (src[1] == '=') { m_tok = Token(ESymbol::SubAssign); src += 2; break; }
					m_tok = Token(ESymbol::Minus); ++src;
					break;
				}
				case '*':
				{
					if (src[1] == '=') { m_tok = Token(ESymbol::MulAssign); src += 2; break; }
					m_tok = Token(ESymbol::Ptr); ++src;
					break;
				}
				case '%':
				{
					if (src[1] == '=') { m_tok = Token(ESymbol::ModAssign); src += 2; break; }
					m_tok = Token(ESymbol::Modulus); ++src;
					break;
				}
				case '/':
				{
					if (src[1] == '=') { m_tok = Token(ESymbol::DivAssign); src += 2; break; }
					m_tok = Token(ESymbol::Divide); ++src;
					break;
				}
				case '(': m_tok = Token(ESymbol::ParenthOpen); ++src; break;
				case ')': m_tok = Token(ESymbol::ParenthClose); ++src; break;
				case '[': m_tok = Token(ESymbol::BracketOpen); ++src; break;
				case ']': m_tok = Token(ESymbol::BracketClose); ++src; break;
				case '{': m_tok = Token(ESymbol::BraceOpen); ++src; break;
				case '}': m_tok = Token(ESymbol::BraceClose); ++src; break;
				case ',': m_tok = Token(ESymbol::Comma); ++src; break;
				case ';': m_tok = Token(ESymbol::SemiColon); ++src; break;
				case ':': m_tok = Token(ESymbol::Colon); ++src; break;
				case '?': m_tok = Token(ESymbol::Conditional); ++src; break;
				case '~': m_tok = Token(ESymbol::Complement); ++src; break;
				case '#': m_tok = Token(ESymbol::Hash); ++src; break;
				case '$': m_tok = Token(ESymbol::Dollar); ++src; break;
				case '@': m_tok = Token(ESymbol::At); ++src; break;
				default:
				{
					throw ScriptException(EResult::SyntaxError, src.Location(), "Tokeniser failed to understand code starting here");
				}
			}
		}
	};
}

