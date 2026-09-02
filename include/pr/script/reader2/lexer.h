//**********************************
// Script
//  Copyright (c) Rylogic Ltd 2015
//**********************************
// Reader2 (pr::script::v2) - UTF-8 native lexer.
//
// Style Guidance:
//  - This mirrors 'pr/script/tokeniser.h::Tokeniser::seek()' exactly (same character
//    dispatch, same symbol/keyword/constant recognition) but reads from a UTF-8 byte
//    stream rather than a 'wchar_t' 'Src', and reuses the legacy 'EToken'/'EKeyword'/
//    'ESymbol'/'EConstant' enumerations so token classification hashes match exactly.
//  - Comment stripping and line-continuation joining have already happened upstream
//    (in 'FilterCursor', see 'preprocessor.h'), so this lexer does not need to
//    recognise '//' or '/* */' itself.
#pragma once
#include <string>
#include <string_view>
#include "pr/script/forward.h"
#include "pr/script/location.h"
#include "pr/script/fail_policy.h"
#include "pr/script/script_core.h"
#include "pr/common/number.h"
#include "pr/str/extract.h"
#include "pr/str/string_core.h"
#include "pr/script/reader2/token.h"
#include "pr/script/reader2/preprocessor.h"

namespace pr::script::v2
{
	// Converts a fully-preprocessed UTF-8 character stream into a sequence of
	// 'Token's. 'Src' must satisfy the 'Ptr' concept used throughout reader2:
	// 'operator*'/'operator[]'/'operator++'/'operator+='/'Location()'/'AtEnd()'.
	// The normal instantiation is over 'Preprocessor', but the lexer can also run
	// directly over a 'Cursor' when preprocessing is not wanted.
	template <typename Src>
	class Lexer
	{
		Src* m_src;   // The character stream to read from (non-owning).
		Token m_tok;  // The token last read from the stream.

	public:

		// Construct a lexer over 'src', which must outlive this lexer, and read
		// the first token so '*lexer' is valid immediately.
		explicit Lexer(Src& src)
			: m_src(&src)
			, m_tok()
		{
			Seek();
		}

		// Pointer-like interface.
		Token const& operator *() const
		{
			return m_tok;
		}
		Lexer& operator ++()
		{
			Seek();
			return *this;
		}
		bool AtEnd() const
		{
			return m_tok.m_type == EToken::EndOfStream;
		}

	private:

		// Advance to the next token. Horizontal/vertical whitespace between tokens
		// does not itself produce a token; a run of blank lines/space is simply
		// skipped before dispatching on the next significant character.
		void Seek()
		{
			auto& src = *m_src;

			EatLineSpace(src, 0, 0);
			auto loc = src.Location();
			auto c = *src;

			// Dispatch purely on the first character, mirroring the legacy
			// tokeniser's character-driven switch so error locations and token
			// boundaries line up with the old reader.
			switch (Classify(c))
			{
				case ECharClass::End:
				{
					m_tok = Token(EToken::EndOfStream, {}, loc);
					break;
				}
				case ECharClass::NewLine:
				{
					m_tok = Token(EToken::Symbol, "\n", loc);
					m_tok.m_symbol = ESymbol::NewLine;
					++src;
					break;
				}
				case ECharClass::IdentStart:
				{
					ReadIdentifierOrKeyword(loc);
					break;
				}
				case ECharClass::Digit:
				{
					ReadNumber(loc);
					break;
				}
				case ECharClass::Quote:
				{
					ReadStringOrChar(loc, false);
					break;
				}
				case ECharClass::Symbol:
				{
					ReadSymbol(loc);
					break;
				}
				default:
				{
					throw ScriptException(EResult::SyntaxError, loc, "Lexer failed to understand code starting here");
				}
			}
		}

		// Broad classification of the first byte of the next token, used only to
		// select which extraction routine below applies.
		enum class ECharClass { End, NewLine, IdentStart, Digit, Quote, Symbol, Unknown };
		static ECharClass Classify(char c)
		{
			if (c == 0) return ECharClass::End;
			if (c == '\n') return ECharClass::NewLine;
			if (c == '_' || str::IsIdentifier(c, true)) return ECharClass::IdentStart;
			if (str::IsDigit(c)) return ECharClass::Digit;
			if (c == '\'' || c == '\"') return ECharClass::Quote;
			if (c == 'L') return ECharClass::IdentStart; // resolved to a literal prefix in 'ReadIdentifierOrKeyword' if followed by a quote.
			if (c == '.') return ECharClass::Symbol; // may be a number; resolved in 'ReadSymbol'.
			if (std::string_view("<>&|^!=+-*%/()[]{},;:?~#$@").find(c) != std::string_view::npos) return ECharClass::Symbol;
			return ECharClass::Unknown;
		}

		// Read an identifier, keyword, or the 'L' prefix of a wide string/char literal.
		void ReadIdentifierOrKeyword(Loc const& loc)
		{
			auto& src = *m_src;

			// 'L' immediately followed by a quote is a wide string/char literal
			// prefix, not an identifier - matches the legacy tokeniser's special case.
			if (*src == 'L' && (src[1] == '\'' || src[1] == '\"'))
			{
				++src;
				ReadStringOrChar(loc, true);
				return;
			}

			std::string id;
			if (!str::ExtractIdentifier(id, src))
				throw ScriptException(EResult::SyntaxError, loc, "Invalid identifier");

			// Keywords hash-match the ASCII-only legacy keyword set; identifiers do not.
			auto hash = pr::hash::Hash(std::string_view(id));
			if (pr::Enum<EKeyword>::IsValue(hash))
			{
				m_tok = Token(EToken::Keyword, id, loc);
				m_tok.m_keyword = s_cast<EKeyword>(hash);
			}
			else
			{
				m_tok = Token(EToken::Identifier, id, loc);
			}
		}

		// Read a numeric constant (integer or floating point), using the same
		// generic 'Number' extraction as the legacy tokeniser.
		void ReadNumber(Loc const& loc)
		{
			auto& src = *m_src;

			Number num;
			if (!str::ExtractNumber(num, src))
				throw ScriptException(EResult::SyntaxError, loc, "Invalid numeric constant");

			if (num.m_type == Number::EType::FP)
			{
				m_tok = Token(EToken::Constant, {}, loc);
				m_tok.m_constant = EConstant::FloatingPoint;
				m_tok.m_real_value = num.db();
			}
			else
			{
				m_tok = Token(EToken::Constant, {}, loc);
				m_tok.m_constant = EConstant::Integral;
				m_tok.m_int_value = num.ll();
			}
		}

		// Read a quoted string or character literal (optionally 'L'-prefixed,
		// already consumed by the caller when 'is_wide' is true).
		void ReadStringOrChar(Loc const& loc, bool is_wide)
		{
			auto& src = *m_src;
			auto is_char = *src == '\'';

			std::string str;
			if (!str::ExtractString(str, src, '\\'))
				throw ScriptException(EResult::SyntaxError, loc, "Invalid literal constant");

			if (is_char)
			{
				// Char literals are integral constants of the first decoded byte,
				// matching the legacy tokeniser (single-byte chars only; reader2
				// does not attempt to decode a UTF-8 char literal to a code point).
				m_tok = Token(EToken::Constant, str, loc);
				m_tok.m_constant = EConstant::Integral;
				m_tok.m_int_value = str.empty() ? 0 : static_cast<uint8_t>(str[0]);
			}
			else
			{
				m_tok = Token(EToken::Constant, str, loc);
				m_tok.m_constant = is_wide ? EConstant::WStringLiteral : EConstant::StringLiteral;
			}
		}

		// Read an operator/punctuation symbol, choosing the longest match first
		// (e.g. '<<=' before '<<' before '<'), exactly as the legacy tokeniser does.
		void ReadSymbol(Loc const& loc)
		{
			auto& src = *m_src;
			auto c = *src;

			ESymbol sym; int len;
			switch (c)
			{
				case '.':
				{
					if (src[1] == '.' && src[2] == '.') { sym = ESymbol::Ellipsis; len = 3; break; }
					if (str::IsDigit(src[1])) { ReadNumber(loc); return; } // '.' can start a number
					sym = ESymbol::Dot; len = 1; break;
				}
				case '<':
				{
					if (src[1] == '<' && src[2] == '=') { sym = ESymbol::ShiftLAssign; len = 3; break; }
					if (src[1] == '<') { sym = ESymbol::ShiftL; len = 2; break; }
					if (src[1] == '=') { sym = ESymbol::LessEql; len = 2; break; }
					sym = ESymbol::LessThan; len = 1; break;
				}
				case '>':
				{
					if (src[1] == '>' && src[2] == '=') { sym = ESymbol::ShiftRAssign; len = 3; break; }
					if (src[1] == '>') { sym = ESymbol::ShiftR; len = 2; break; }
					if (src[1] == '=') { sym = ESymbol::GtrEql; len = 2; break; }
					sym = ESymbol::GtrThan; len = 1; break;
				}
				case '&':
				{
					if (src[1] == '&') { sym = ESymbol::LogicalAnd; len = 2; break; }
					if (src[1] == '=') { sym = ESymbol::BitAndAssign; len = 2; break; }
					sym = ESymbol::AddressOf; len = 1; break;
				}
				case '|':
				{
					if (src[1] == '|') { sym = ESymbol::LogicalOr; len = 2; break; }
					if (src[1] == '=') { sym = ESymbol::BitOrAssign; len = 2; break; }
					sym = ESymbol::BitOr; len = 1; break;
				}
				case '^':
				{
					if (src[1] == '=') { sym = ESymbol::BitXorAssign; len = 2; break; }
					sym = ESymbol::BitXor; len = 1; break;
				}
				case '!':
				{
					if (src[1] == '=') { sym = ESymbol::NotEqual; len = 2; break; }
					sym = ESymbol::Not; len = 1; break;
				}
				case '=':
				{
					if (src[1] == '=') { sym = ESymbol::Equal; len = 2; break; }
					sym = ESymbol::Assign; len = 1; break;
				}
				case '+':
				{
					if (src[1] == '+') { sym = ESymbol::Increment; len = 2; break; }
					if (src[1] == '=') { sym = ESymbol::AddAssign; len = 2; break; }
					sym = ESymbol::Plus; len = 1; break;
				}
				case '-':
				{
					if (src[1] == '-') { sym = ESymbol::Decrement; len = 2; break; }
					if (src[1] == '=') { sym = ESymbol::SubAssign; len = 2; break; }
					sym = ESymbol::Minus; len = 1; break;
				}
				case '*':
				{
					if (src[1] == '=') { sym = ESymbol::MulAssign; len = 2; break; }
					sym = ESymbol::Ptr; len = 1; break;
				}
				case '%':
				{
					if (src[1] == '=') { sym = ESymbol::ModAssign; len = 2; break; }
					sym = ESymbol::Modulus; len = 1; break;
				}
				case '/':
				{
					if (src[1] == '=') { sym = ESymbol::DivAssign; len = 2; break; }
					sym = ESymbol::Divide; len = 1; break;
				}
				case '(': sym = ESymbol::ParenthOpen; len = 1; break;
				case ')': sym = ESymbol::ParenthClose; len = 1; break;
				case '[': sym = ESymbol::BracketOpen; len = 1; break;
				case ']': sym = ESymbol::BracketClose; len = 1; break;
				case '{': sym = ESymbol::BraceOpen; len = 1; break;
				case '}': sym = ESymbol::BraceClose; len = 1; break;
				case ',': sym = ESymbol::Comma; len = 1; break;
				case ';': sym = ESymbol::SemiColon; len = 1; break;
				case ':': sym = ESymbol::Colon; len = 1; break;
				case '?': sym = ESymbol::Conditional; len = 1; break;
				case '~': sym = ESymbol::Complement; len = 1; break;
				case '#': sym = ESymbol::Hash; len = 1; break;
				case '$': sym = ESymbol::Dollar; len = 1; break;
				case '@': sym = ESymbol::At; len = 1; break;
				default:
				{
					throw ScriptException(EResult::SyntaxError, loc, "Lexer failed to understand code starting here");
				}
			}

			std::string text;
			for (auto i = 0; i != len; ++i, ++src)
				text.push_back(*src);

			m_tok = Token(EToken::Symbol, text, loc);
			m_tok.m_symbol = sym;
		}
	};
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::script::v2::testing
{
	// Drain 'src' as tokens, formatting each as "<type>:<text>" joined by '|', for
	// compact assertions in the tests below.
	template <typename Src>
	inline std::string DrainTokens(Src& src)
	{
		std::string out;
		for (Lexer<Src> lex(src); !lex.AtEnd(); ++lex)
		{
			if (!out.empty())
				out.push_back('|');

			auto const& tok = *lex;
			switch (tok.m_type)
			{
				case EToken::Identifier: out += "Id:" + tok.m_text; break;
				case EToken::Keyword: out += "Kw:" + tok.m_text; break;
				case EToken::Symbol: out += "Sym:" + tok.m_text; break;
				case EToken::Constant:
				{
					switch (tok.m_constant)
					{
						case EConstant::Integral: out += "Int:" + std::to_string(tok.m_int_value); break;
						case EConstant::FloatingPoint: out += "Flt:" + std::to_string(tok.m_real_value); break;
						case EConstant::StringLiteral: out += "Str:" + tok.m_text; break;
						case EConstant::WStringLiteral: out += "WStr:" + tok.m_text; break;
						case EConstant::Invalid:
						default: out += "Const:?"; break;
					}
					break;
				}
				case EToken::Invalid:
				case EToken::EndOfStream:
				default: out += "?"; break;
			}
		}
		return out;
	}

	PRUnitTestClass(Reader2LexerTests)
	{

		// Identifiers, keywords, and symbols are recognised, with keywords matching
		// the shared 'EKeyword' hash set used by the legacy tokeniser.
		PRUnitTestMethod(IdentifiersKeywordsSymbols, Quick)
		{
			Preprocessor pp("int x = foo + 1;");
			auto out = DrainTokens(pp);
			PR_EXPECT(out == "Kw:int|Id:x|Sym:=|Id:foo|Sym:+|Int:1|Sym:;");
		}

		// Numeric constants: integer and floating point.
		PRUnitTestMethod(NumericConstants, Quick)
		{
			Preprocessor pp("123 4.5 0x1F");
			auto out = DrainTokens(pp);
			PR_EXPECT(out == "Int:123|Flt:4.500000|Int:31");
		}

		// String and char literals.
		PRUnitTestMethod(StringAndCharLiterals, Quick)
		{
			Preprocessor pp("\"hello\" 'a'");
			auto out = DrainTokens(pp);
			PR_EXPECT(out == "Str:hello|Int:97");
		}

		// Multi-character symbols choose the longest match.
		PRUnitTestMethod(LongestSymbolMatch, Quick)
		{
			Preprocessor pp("a<<=b a<<b a<=b");
			auto out = DrainTokens(pp);
			PR_EXPECT(out == "Id:a|Sym:<<=|Id:b|Id:a|Sym:<<|Id:b|Id:a|Sym:<=|Id:b");
		}

		// Tokens are correctly produced across an input that spans multiple
		// underlying transport blocks (forces 'Cursor::Refill'/'Compact' paths).
		PRUnitTestMethod(SpansMultipleBlocks, Quick)
		{
			std::string big(BlockSize * 2 + 7, ' ');
			big += "trailing_identifier";
			Preprocessor pp(big);
			auto out = DrainTokens(pp);
			PR_EXPECT(out == "Id:trailing_identifier");
		}
	};
}
#endif
