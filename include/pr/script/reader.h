//**********************************
// Script
//  Copyright (c) Rylogic Ltd 2015
//**********************************
// UTF-8 script reader.
//
// Style Guidance:
//  - This header aggregates the reader stack (input -> cursor -> token/lexer ->
//    macros/preprocessor -> reader) in dependency order.
//  - Conformance tests live in a sibling header,
//    'pr/script/reader/conformance_tests.h', included below only when PR_UNITTESTS is
//    enabled, so ordinary translation units do not pull in file-backed test fixtures.
#pragma once
#include "pr/script/reader/input.h"
#include "pr/script/reader/cursor.h"
#include "pr/script/reader/token.h"
#include "pr/script/reader/lexer.h"
#include "pr/script/reader/macros.h"
#include "pr/script/reader/preprocessor.h"
#include "pr/script/reader/reader.h"

namespace pr::script
{
	// Public UTF-8 script reader facade.
	using Reader = reader::Reader;
}

#if PR_UNITTESTS
#include "pr/script/reader/conformance_tests.h"
#endif
