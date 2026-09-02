//**********************************
// Script
//  Copyright (c) Rylogic Ltd 2015
//**********************************
// Reader2 (pr::script::v2) - lexical tokens.
#pragma once
#include <string>
#include <cstdint>
#include "pr/script/forward.h"
#include "pr/script/location.h"

namespace pr::script::v2
{
	// A single lexical token produced by the reader2 lexer.
	//
	// Reuses the legacy 'EToken'/'EKeyword'/'ESymbol'/'EConstant' enumerations from
	// 'pr/script/forward.h' rather than declaring a parallel set, so that keyword and
	// symbol hash values remain identical between the old and new readers - this is
	// required for the "matching error code + location" compatibility contract, since
	// error messages and diagnostics may reference these values.
	struct Token
	{
		// The token's general classification.
		EToken m_type = EToken::Invalid;

		// The raw UTF-8 text of the token, exactly as it appeared in the (post
		// preprocessing) source. For string/char constants this excludes the quotes
		// and has escape sequences already resolved.
		std::string m_text;

		// The location of the first byte of this token in the original source.
		Loc m_loc;

		// Populated when 'm_type == EToken::Keyword': the recognised keyword and its
		// case-appropriate hash (see 'HashKeyword').
		EKeyword m_keyword = EKeyword::Invalid;

		// Populated when 'm_type == EToken::Symbol': the recognised symbol.
		ESymbol m_symbol = ESymbol::Invalid;

		// Populated when 'm_type == EToken::Constant': which kind of literal this is.
		EConstant m_constant = EConstant::Invalid;

		// Populated when 'm_constant' is 'Integral': the parsed value.
		int64_t m_int_value = 0;

		// Populated when 'm_constant' is 'FloatingPoint': the parsed value.
		double m_real_value = 0.0;

		Token() = default;
		Token(EToken type, std::string text, Loc const& loc)
			: m_type(type)
			, m_text(std::move(text))
			, m_loc(loc)
		{}
	};
}
