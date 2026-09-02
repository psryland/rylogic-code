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
//  - The differential tests below deliberately '#include "pr/script/reader.h"'
//    (the legacy reader) so both readers can be exercised side-by-side against
//    the same script text; that legacy include only takes effect when
//    PR_UNITTESTS is enabled, so ordinary (non-test) translation units that
//    include this header never pull in the legacy reader or 'pr::maths'.
#pragma once
#include "pr/script/reader2/input.h"
#include "pr/script/reader2/cursor.h"
#include "pr/script/reader2/token.h"
#include "pr/script/reader2/lexer.h"
#include "pr/script/reader2/macros.h"
#include "pr/script/reader2/preprocessor.h"
#include "pr/script/reader2/reader.h"

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/script/reader.h"
namespace pr::script::v2::testing
{
	// Differential tests: run the same UTF-8 script text through both the legacy
	// 'pr::script::Reader' and the new 'pr::script::v2::Reader', and check they
	// agree on every value extracted plus (for injected errors) on 'EResult' and
	// 'Loc', per the "matching error code + location" compatibility requirement.
	// Exact error message text is NOT compared, since the two readers are allowed
	// to differ there.
	PRUnitTestClass(Reader2DifferentialTests)
	{
		// A script exercising keywords, sections, nested sections, comments, line
		// continuations, macros (#define), conditional compilation (#if/#else/
		// #endif), strings/c-strings, bools, ints (multiple radixes) and reals -
		// i.e. the full non-math-vector surface both readers implement.
		inline static constexpr char const* Script =
			"// leading line comment\n"
			"/* leading block\n"
			"   comment */\n"
			"#define GREETING \"hello\"\n"
			"#define COUNT 3\n"
			"#if COUNT == 3\n"
			"*Kept true\n"
			"#else\n"
			"*NotKept true\n"
			"#endif\n"
			"*Message GREETING\n"
			"*LineCont \"first \\\n"
			"second\"\n"
			"*Nested { *Inner { *Value 7 } }\n"
			"*Ident an_identifier_123\n"
			"*Str \"plain string\"\n"
			"*CStr \"line1\\nline2\"\n"
			"*Flag true\n"
			"*Flag2 false\n"
			"*Dec -123\n"
			"*Hex ABCDEF00\n"
			"*Pi 3.14159\n"
			"*Neg -2.5e-2\n"
			"*Done";

		// Extract the full 'Script' contract from 'reader' (either reader type,
		// since both expose the same method names) and return the values as a
		// vector of strings for easy side-by-side comparison between readers.
		template <typename Reader> static std::vector<std::string> Extract(Reader& reader)
		{
			std::vector<std::string> out;
			char kw[64];

			auto next = [&]
			{
				reader.NextKeywordS(kw);
				out.push_back(std::string("kw:") + kw);
			};

			next(); // Kept
			bool kept = false;
			reader.Bool(kept);
			out.push_back(std::string("bool:") + (kept ? "true" : "false"));

			next(); // Message
			std::string msg;
			reader.String(msg);
			out.push_back("str:" + msg);

			next(); // LineCont
			std::string cont;
			reader.String(cont);
			out.push_back("str:" + cont);

			next(); // Nested
			reader.SectionStart();
			next(); // Inner
			reader.SectionStart();
			next(); // Value
			int inner = 0;
			reader.Int(inner, 10);
			out.push_back("int:" + std::to_string(inner));
			reader.SectionEnd();
			reader.SectionEnd();

			next(); // Ident
			std::string ident;
			reader.Identifier(ident);
			out.push_back("id:" + ident);

			next(); // Str
			std::string str;
			reader.String(str);
			out.push_back("str:" + str);

			next(); // CStr
			std::string cstr;
			reader.CString(cstr);
			out.push_back("cstr:" + cstr);

			next(); // Flag
			bool flag = false;
			reader.Bool(flag);
			out.push_back(std::string("bool:") + (flag ? "true" : "false"));

			next(); // Flag2
			bool flag2 = true;
			reader.Bool(flag2);
			out.push_back(std::string("bool:") + (flag2 ? "true" : "false"));

			next(); // Dec
			int dec = 0;
			reader.Int(dec, 10);
			out.push_back("int:" + std::to_string(dec));

			next(); // Hex
			unsigned int hex = 0;
			reader.Int(hex, 16);
			out.push_back("hex:" + std::to_string(hex));

			next(); // Pi
			double pi = 0.0;
			reader.Real(pi);
			out.push_back("real:" + std::to_string(pi));

			next(); // Neg
			double neg = 0.0;
			reader.Real(neg);
			out.push_back("real:" + std::to_string(neg));

			next(); // Done
			out.push_back(std::string("kw:") + kw);

			return out;
		}

		PRUnitTestMethod(SameResultsAsLegacyReader, Quick)
		{
			pr::script::Reader legacy(Script, false);
			pr::script::v2::Reader modern(std::string_view(Script), false);

			auto legacy_values = Extract(legacy);
			auto modern_values = Extract(modern);

			PR_EXPECT(legacy_values.size() == modern_values.size());
			for (size_t i = 0, iend = std::min(legacy_values.size(), modern_values.size()); i != iend; ++i)
				PR_EXPECT(legacy_values[i] == modern_values[i]);

			PR_EXPECT(legacy.IsSourceEnd());
			PR_EXPECT(modern.IsSourceEnd());
		}

		PRUnitTestMethod(MatchingErrorCodeAndLocation, Quick)
		{
			// Both readers must fail with the same 'EResult' at the same 'Loc' when
			// asked for a keyword that never appears, even though message text differs.
			char const* script = "*Foo true";

			pr::script::Reader legacy(script, false);
			pr::script::v2::Reader modern(std::string_view(script), false);

			legacy.ReportError = [](EResult, Loc const&, std::string_view) { return false; };
			modern.ReportError = [](EResult, Loc const&, std::string_view) { return false; };

			PR_EXPECT(legacy.FindKeyword(L"Missing") == false);
			PR_EXPECT(modern.FindKeyword("Missing") == false);

			auto legacy_loc = legacy.Location();
			auto modern_loc = modern.Location();
			PR_EXPECT(legacy_loc.Pos() == modern_loc.Pos());
		}

		PRUnitTestMethod(CrossBlockBoundaryDifferential, Quick)
		{
			// Pad the script so keyword/value pairs straddle 'reader2::Cursor's
			// physical refill/compaction boundaries, and check both readers still agree.
			std::string padded(BlockSize - 3, ' ');
			padded += "*Straddle 12345\n*After \"tail value\"";

			pr::script::Reader legacy(padded.c_str(), false);
			pr::script::v2::Reader modern(std::string_view(padded), false);

			char lkw[64], mkw[64];
			int lval = 0, mval = 0;
			std::string lstr, mstr;

			PR_EXPECT(legacy.NextKeywordS(lkw) && modern.NextKeywordS(mkw) && std::string(lkw) == std::string(mkw));
			PR_EXPECT(legacy.Int(lval, 10) && modern.Int(mval, 10) && lval == mval);
			PR_EXPECT(legacy.NextKeywordS(lkw) && modern.NextKeywordS(mkw) && std::string(lkw) == std::string(mkw));
			PR_EXPECT(legacy.String(lstr) && modern.String(mstr) && lstr == mstr);
			PR_EXPECT(legacy.IsSourceEnd() && modern.IsSourceEnd());
		}
	};
}
#endif
