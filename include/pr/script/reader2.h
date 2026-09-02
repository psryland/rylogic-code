//**********************************
// Script
//  Copyright (c) Rylogic Ltd 2015
//**********************************
// Reader2 (pr::script::v2) - umbrella header for the UTF-8 native script reader.
//
// Style Guidance:
//  - This header aggregates the whole reader2 stack (input -> cursor -> token/
//    lexer -> macros/preprocessor -> reader) in dependency order, mirroring how
//    'pr/script/script.h' aggregates the legacy library. It is intentionally NOT
//    included by 'script.h': reader2 is an opt-in sibling to the legacy reader,
//    not a replacement wired into the existing umbrella, so that the legacy
//    'pr::script::Reader' remains completely unaffected by this addition.
//  - Only include this header (or the individual 'pr/script/reader2/*' headers)
//    from code that has explicitly opted in to the new reader. The old
//    'pr::script::Reader' (pr/script/reader.h) is unmodified and continues to
//    work exactly as before.
//  - Conformance and differential tests for this reader live in a sibling header,
//    'pr/script/reader2/conformance_tests.h', included below only when PR_UNITTESTS is
//    enabled, so ordinary (non-test) translation units that include this header never
//    pull in the legacy reader, 'pr::maths', or the on-disk fixture tree those tests use.
#pragma once
#include "pr/script/reader2/input.h"
#include "pr/script/reader2/cursor.h"
#include "pr/script/reader2/token.h"
#include "pr/script/reader2/lexer.h"
#include "pr/script/reader2/macros.h"
#include "pr/script/reader2/preprocessor.h"
#include "pr/script/reader2/reader.h"

#if PR_UNITTESTS
#include "pr/script/reader2/conformance_tests.h"
#endif
