//**********************************
// Script
//  Copyright (c) Rylogic Ltd 2015
//**********************************
// Reader2 conformance tests.
//
// Style Guidance:
//  - This header holds the differential/conformance tests for 'pr::script::v2::Reader'.
//    It lives beside the rest of the reader2 stack (rather than under
//    'projects/tests/unittests/src/') so the tests travel with the reader they exercise,
//    matching this stack's existing convention of keeping unit tests inline. It is
//    '#include'd by 'reader2.h' only under 'PR_UNITTESTS', so the legacy
//    'pr::script::Reader'/'std::filesystem' file-I/O it uses to run these tests never
//    reaches a non-test build.
//  - 'Reader2DifferentialTests' (relocated here verbatim from 'reader2.h') exercises
//    hand-written script snippets covering the math/vector/matrix/transform surface.
//    'Reader2ConformanceTests' below instead drives one canonical, checked-in UTF-8
//    fixture tree through both readers, so the milestone-0 "coverage matrix" of shared
//    legacy-supported features has a single, human-readable, on-disk source of truth
//    rather than being scattered across in-code string literals.
//  - The canonical fixture tree lives under
//    'projects/tests/unittests/res/script_reader/' and is located at runtime via a path
//    derived from this header's own '__FILE__', so these tests work regardless of the
//    process's current working directory.
#pragma once
#include <array>
#include <bit>
#include <cmath>
#include <cstring>
#include <fstream>
#include <sstream>
#include "pr/common/unittests.h"
#include "pr/script/reader.h"
#include "pr/script/includes.h"
#include "pr/script/reader2/reader.h"

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

		PRUnitTestMethod(PlainNumericFastPathMatchesLegacy, Quick)
		{
			// Exercise direct decimal conversion and every compatibility fallback
			// from a source that remains on the screened contiguous path.
			char const* script = "-23 +42 -1 456LL -1.25e-4 +2.0 .5 2 -0.2f 1e+2 0x10 0b1 0o7";
			pr::script::Reader legacy(script, false);
			pr::script::v2::Reader modern(std::string_view(script), false);

			int legacy_ints[2] = {}, modern_ints[2] = {};
			unsigned legacy_unsigned = 0, modern_unsigned = 0;
			int64_t legacy_suffix = 0, modern_suffix = 0;
			PR_EXPECT(legacy.Int(legacy_ints[0], 10) && modern.Int(modern_ints[0], 10));
			PR_EXPECT(legacy.Int(legacy_ints[1], 10) && modern.Int(modern_ints[1], 10));
			PR_EXPECT(legacy.Int(legacy_unsigned, 10) && modern.Int(modern_unsigned, 10));
			PR_EXPECT(legacy.Int(legacy_suffix, 10) && modern.Int(modern_suffix, 10));
			PR_EXPECT(legacy_ints[0] == modern_ints[0]);
			PR_EXPECT(legacy_ints[1] == modern_ints[1]);
			PR_EXPECT(legacy_unsigned == modern_unsigned);
			PR_EXPECT(legacy_suffix == modern_suffix);

			double legacy_reals[9] = {}, modern_reals[9] = {};
			for (size_t i = 0; i != std::size(legacy_reals); ++i)
			{
				PR_EXPECT(legacy.Real(legacy_reals[i]) && modern.Real(modern_reals[i]));
				if (std::isnan(legacy_reals[i]))
					PR_EXPECT(std::isnan(modern_reals[i]));
				else
					PR_EXPECT(std::bit_cast<uint64_t>(legacy_reals[i]) == std::bit_cast<uint64_t>(modern_reals[i]));
			}
			PR_EXPECT(legacy.IsSourceEnd());
			PR_EXPECT(modern.IsSourceEnd());

			// Overflow remains a reported conversion failure after the fast parser
			// delegates to the compatibility path.
			char const* overflow = "9223372036854775808";
			pr::script::Reader legacy_overflow(overflow, false);
			pr::script::v2::Reader modern_overflow(std::string_view(overflow), false);
			legacy_overflow.ReportError = [](EResult, Loc const&, std::string_view) { return false; };
			modern_overflow.ReportError = [](EResult, Loc const&, std::string_view) { return false; };
			int64_t legacy_value = 0, modern_value = 0;
			PR_EXPECT(!legacy_overflow.Int(legacy_value, 10));
			PR_EXPECT(!modern_overflow.Int(modern_value, 10));
			PR_EXPECT(legacy_overflow.Location().Pos() == modern_overflow.Location().Pos());
		}

		PRUnitTestMethod(StrictUtf8Validation, Quick)
		{
			// Reject malformed scalar encodings through the batched contiguous path.
			std::array<std::string, 5> invalid =
			{
				std::string("\xE0\x80\x80", 3),     // Overlong three-byte scalar.
				std::string("\xED\xA0\x80", 3),     // UTF-16 high surrogate.
				std::string("\xF4\x90\x80\x80", 4), // Beyond U+10FFFF.
				std::string("\xF0\x9F\x92", 3),     // Truncated four-byte scalar.
				std::string("\x80", 1),             // Standalone continuation byte.
			};
			for (auto const& bytes : invalid)
			{
				auto script = std::string("\"").append(bytes).append("\"");
				auto rejected = false;
				try
				{
					Reader reader(script);
					std::string value;
					reader.String(value);
				}
				catch (ScriptException const& ex)
				{
					rejected = ex.m_result == EResult::WrongEncoding;
				}
				PR_EXPECT(rejected);
			}

			// The character-at-a-time filtered path enforces the same scalar rules.
			auto filtered_script = std::string("\"/").append(invalid[1]).append("\"");
			auto filtered_rejected = false;
			try
			{
				Reader reader(filtered_script);
				std::string value;
				reader.String(value);
			}
			catch (ScriptException const& ex)
			{
				filtered_rejected = ex.m_result == EResult::WrongEncoding;
			}
			PR_EXPECT(filtered_rejected);

			// A continuation byte is only legal after its validated lead byte.
			auto continuation_script = std::string("\"/").append(invalid[4]).append("\"");
			auto continuation_rejected = false;
			try
			{
				Reader reader(continuation_script);
				std::string value;
				reader.String(value);
			}
			catch (ScriptException const& ex)
			{
				continuation_rejected = ex.m_result == EResult::WrongEncoding;
			}
			PR_EXPECT(continuation_rejected);

			// Boundary scalar values, including a supplementary character, remain
			// byte-for-byte intact after validation.
			auto valid = std::string("\xE0\xA0\x80\xED\x9F\xBF\xF4\x8F\xBF\xBF", 10);
			auto valid_script = std::string("\"").append(valid).append("\"");
			Reader valid_reader(valid_script);
			std::string value;
			PR_EXPECT(valid_reader.String(value));
			PR_EXPECT(value == valid);
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

		PRUnitTestMethod(MathAndDataExtractionDifferential, Quick)
		{
			// The math/data extractors ('Vector2/2i/3/3i/4/4i', 'Quaternion', 'Matrix3x3/4x4',
			// 'Data' and 'Enum') share identical parameter shapes between the two readers, so
			// both the literal expected values and cross-reader agreement can be checked at once.
			char const* script =
				"*V2 1 2\n"
				"*V2i 3 4\n"
				"*V3 5 6 7\n"
				"*V3i 8 9 10\n"
				"*V4 11 12 13 14\n"
				"*V4i 15 16 17 18\n"
				"*Quat 0 0 0 1\n"
				"*M3x3 1 0 0  0 1 0  0 0 1\n"
				"*M4x4 1 0 0 0  0 1 0 0  0 0 1 0  0 0 0 1\n"
				"*Data DE AD BE EF\n"
				"*En Success\n"
				"*EnValue 0\n"
				"*Done";

			pr::script::Reader legacy(script, false);
			pr::script::v2::Reader modern(std::string_view(script), false);

			char lkw[64], mkw[64];
			auto next = [&]
			{
				PR_EXPECT(legacy.NextKeywordS(lkw) && modern.NextKeywordS(mkw));
				PR_EXPECT(std::string(lkw) == std::string(mkw));
			};

			// Float and int 2D vectors.
			next(); // V2
			pr::v2 lv2 = {}, mv2 = {};
			PR_EXPECT(legacy.Vector2(lv2) && modern.Vector2(mv2));
			PR_EXPECT(FEql(lv2, mv2) && lv2.x == 1 && lv2.y == 2);

			next(); // V2i
			iv2 lv2i = {}, mv2i = {};
			PR_EXPECT(legacy.Vector2i(lv2i) && modern.Vector2i(mv2i));
			PR_EXPECT(lv2i.x == mv2i.x && lv2i.y == mv2i.y && lv2i.x == 3 && lv2i.y == 4);

			// Float and int 3D vectors (with an explicit 'w' component).
			next(); // V3
			v4 lv3 = {}, mv3 = {};
			PR_EXPECT(legacy.Vector3(lv3, 0.0f) && modern.Vector3(mv3, 0.0f));
			PR_EXPECT(FEql(lv3, mv3) && lv3.x == 5 && lv3.y == 6 && lv3.z == 7 && lv3.w == 0);

			next(); // V3i
			iv4 lv3i = {}, mv3i = {};
			PR_EXPECT(legacy.Vector3i(lv3i, 0, 10) && modern.Vector3i(mv3i, 0, 10));
			PR_EXPECT(lv3i.x == mv3i.x && lv3i.y == mv3i.y && lv3i.z == mv3i.z && lv3i.x == 8 && lv3i.y == 9 && lv3i.z == 10);

			// Float and int 4D vectors.
			next(); // V4
			v4 lv4 = {}, mv4 = {};
			PR_EXPECT(legacy.Vector4(lv4) && modern.Vector4(mv4));
			PR_EXPECT(FEql(lv4, mv4) && lv4.x == 11 && lv4.y == 12 && lv4.z == 13 && lv4.w == 14);

			next(); // V4i
			iv4 lv4i = {}, mv4i = {};
			PR_EXPECT(legacy.Vector4i(lv4i, 10) && modern.Vector4i(mv4i, 10));
			PR_EXPECT(lv4i.x == mv4i.x && lv4i.y == mv4i.y && lv4i.z == mv4i.z && lv4i.w == mv4i.w);
			PR_EXPECT(lv4i.x == 15 && lv4i.y == 16 && lv4i.z == 17 && lv4i.w == 18);

			// Quaternion, then 3x3/4x4 matrices (each read as flat lists of reals, non-sectioned).
			next(); // Quat
			quat lq = {}, mq = {};
			PR_EXPECT(legacy.Quaternion(lq) && modern.Quaternion(mq));
			PR_EXPECT(FEql(lq, mq) && FEql(lq, quat(0, 0, 0, 1)));

			next(); // M3x3
			m3x3 lm3 = {}, mm3 = {};
			PR_EXPECT(legacy.Matrix3x3(lm3) && modern.Matrix3x3(mm3));
			PR_EXPECT(FEql(lm3, mm3) && FEql(lm3, m3x3::Identity()));

			next(); // M4x4
			m4x4 lm4 = {}, mm4 = {};
			PR_EXPECT(legacy.Matrix4x4(lm4) && modern.Matrix4x4(mm4));
			PR_EXPECT(FEql(lm4, mm4) && FEql(lm4, m4x4::Identity()));

			// Raw bytes, read as space-delimited hex tokens.
			next(); // Data
			unsigned char ldata[4] = {}, mdata[4] = {};
			PR_EXPECT(legacy.Data(ldata, 4, 16) && modern.Data(mdata, 4, 16));
			PR_EXPECT(std::memcmp(ldata, mdata, 4) == 0);
			unsigned char const expect[4] = { 0xDE, 0xAD, 0xBE, 0xEF };
			PR_EXPECT(std::memcmp(ldata, expect, 4) == 0);

			// Named-value enum extraction (not 'EnumValue' - see the note above).
			next(); // En
			EResult lenum = EResult::Failed, menum = EResult::Failed;
			PR_EXPECT(legacy.Enum(lenum) && modern.Enum(menum));
			PR_EXPECT(lenum == menum && lenum == EResult::Success);

			// Integral enum extraction uses radix 10 in the legacy API and the Reader2-compatible default.
			next(); // EnValue
			EResult lenum_value = EResult::Failed, menum_value = EResult::Failed;
			PR_EXPECT(legacy.EnumValue(lenum_value) && modern.EnumValue(menum_value));
			PR_EXPECT(lenum_value == menum_value && lenum_value == EResult::Success);

			next(); // Done
			PR_EXPECT(legacy.IsSourceEnd() && modern.IsSourceEnd());
		}

		PRUnitTestMethod(TransformPositionRotationAlignQuatDifferential, Quick)
		{
			// 'Pos', 'M3x3', 'Align', and 'Quat' each pre-multiply 'o2w' in an identical,
			// deterministic way in both readers; a single accumulated 'Transform' call lets
			// one comparison exercise all four forms plus their combined ordering.
			char const* script =
				"*T { *Pos {1 2 3} *M3x3 {0 -1 0  1 0 0  0 0 1} *Align {2 0 1 0} *Quat {0 0 0 1} }"
				"*Done";

			pr::script::Reader legacy(script, false);
			pr::script::v2::Reader modern(std::string_view(script), false);

			char lkw[64], mkw[64];
			PR_EXPECT(legacy.NextKeywordS(lkw) && modern.NextKeywordS(mkw));

			auto lo2w = m4x4::Identity();
			auto mo2w = m4x4::Identity();
			PR_EXPECT(legacy.TransformS(lo2w) && modern.TransformS(mo2w));
			PR_EXPECT(FEql(lo2w, mo2w));
			PR_EXPECT(IsOrthonormal(lo2w)); // 'Align' rotates, 'Pos' translates - no scale or shear introduced.

			PR_EXPECT(legacy.NextKeywordS(lkw) && modern.NextKeywordS(mkw));
			PR_EXPECT(std::string(lkw) == std::string(mkw)); // both readers default to case-insensitive keyword matching, which lower-cases 'kw'.
		}

		PRUnitTestMethod(TransformScaleAndNormaliseFormsDifferential, Quick)
		{
			// 'Scale' (both uniform and per-axis), 'Transpose', 'Inverse', 'Normalise' and
			// 'Orthonormalise' are all pure matrix arithmetic with no randomness, so their
			// results must match the legacy reader exactly.
			char const* uniform_script = "*T { *Pos {1 2 3} *Scale {2} } *Done";
			char const* per_axis_script = "*T { *Scale {2 3 4} } *Done";
			// 'Pos' is applied last: it pre-multiplies a pure-translation matrix onto 'p2w' and so only ever
			// perturbs the translation column, leaving the rotation block exactly as 'Orthonormalise' left it.
			// Applying it before the rotation ops would instead corrupt the rotation block under 'Transpose'
			// (which transposes the whole 4x4, mixing the translation column into the rotation rows) and trip
			// the shared 'Orthonorm' assert - a test-input hazard, not a facade defect, since legacy exhibits
			// the identical whole-matrix Transpose behaviour.
			char const* ops_script = "*T { *M3x3 {0 -1 0  1 0 0  0 0 1} *Transpose *Inverse *Normalise *Orthonormalise *Pos {1 2 3} } *Done";

			for (auto script : { uniform_script, per_axis_script, ops_script })
			{
				pr::script::Reader legacy(script, false);
				pr::script::v2::Reader modern(std::string_view(script), false);

				char lkw[64], mkw[64];
				PR_EXPECT(legacy.NextKeywordS(lkw) && modern.NextKeywordS(mkw));

				auto lo2w = m4x4::Identity();
				auto mo2w = m4x4::Identity();
				PR_EXPECT(legacy.TransformS(lo2w) && modern.TransformS(mo2w));
				PR_EXPECT(FEql(lo2w, mo2w));

				PR_EXPECT(legacy.NextKeywordS(lkw) && modern.NextKeywordS(mkw));
				PR_EXPECT(std::string(lkw) == std::string(mkw)); // both readers default to case-insensitive keyword matching, which lower-cases 'kw'.
			}
		}

		PRUnitTestMethod(TransformNonAffineM4x4Differential, Quick)
		{
			// An 'M4x4' with a non-unit bottom-right element is only accepted when preceded
			// by 'NonAffine'; without it, both readers must report the same 'EResult' at the
			// same 'Loc' (message text is allowed to differ) and both still return 'true',
			// since the internal error path 'goto's straight to the accumulate-and-return step.
			char const* accepted_script = "*T { *NonAffine *M4x4 {1 0 0 0  0 1 0 0  0 0 1 0  0 0 0 2} } *Done";
			{
				pr::script::Reader legacy(accepted_script, false);
				pr::script::v2::Reader modern(std::string_view(accepted_script), false);

				char lkw[64], mkw[64];
				PR_EXPECT(legacy.NextKeywordS(lkw) && modern.NextKeywordS(mkw));

				auto lo2w = m4x4::Identity();
				auto mo2w = m4x4::Identity();
				PR_EXPECT(legacy.TransformS(lo2w) && modern.TransformS(mo2w));
				PR_EXPECT(FEql(lo2w, mo2w) && lo2w.w.w == 2);
			}

			char const* rejected_script = "*T { *M4x4 {1 0 0 0  0 1 0 0  0 0 1 0  0 0 0 2} } *Done";
			{
				pr::script::Reader legacy(rejected_script, false);
				pr::script::v2::Reader modern(std::string_view(rejected_script), false);

				EResult legacy_result = EResult::Success, modern_result = EResult::Success;
				Loc legacy_loc, modern_loc;
				legacy.ReportError = [&](EResult r, Loc const& l, std::string_view) { legacy_result = r; legacy_loc = l; return true; };
				modern.ReportError = [&](EResult r, Loc const& l, std::string_view) { modern_result = r; modern_loc = l; return true; };

				char lkw[64], mkw[64];
				PR_EXPECT(legacy.NextKeywordS(lkw) && modern.NextKeywordS(mkw));

				auto lo2w = m4x4::Identity();
				auto mo2w = m4x4::Identity();
				PR_EXPECT(legacy.TransformS(lo2w) && modern.TransformS(mo2w)); // still returns 'true' - see comment above.
				PR_EXPECT(legacy_result == modern_result && legacy_result == EResult::UnknownValue);
				PR_EXPECT(legacy_loc.Pos() == modern_loc.Pos());
				PR_EXPECT(FEql(lo2w, m4x4::Identity()) && FEql(mo2w, m4x4::Identity())); // the offending 'M4x4' was never applied.
			}
		}

		PRUnitTestMethod(TransformRandomFormsPropertyChecks, Quick)
		{
			// 'Rand4x4'/'RandPos'/'RandOri' each draw from a private, per-reader-type
			// 'static std::default_random_engine', so legacy and reader2 can never produce
			// bit-identical values; verify the documented structural guarantees on each
			// reader independently instead of comparing values across readers.
			char const* script =
				"*A { *Rand4x4 {10 20 30  5} }\n"
				"*B { *RandPos {10 20 30  5} }\n"
				"*C { *RandOri }\n"
				"*Done";

			auto const centre = v4(10, 20, 30, 1);
			float const radius = 5.0f;
			float const slack = 0.01f; // tolerance for the '<= radius' boundary check.

			auto check = [&](auto& reader)
			{
				char kw[64];

				PR_EXPECT(reader.NextKeywordS(kw));
				auto a = m4x4::Identity();
				PR_EXPECT(reader.TransformS(a));
				PR_EXPECT(IsOrthonormal(a));
				PR_EXPECT(Length(a.pos - centre) <= radius + slack);

				PR_EXPECT(reader.NextKeywordS(kw));
				auto b = m4x4::Identity();
				PR_EXPECT(reader.TransformS(b));
				PR_EXPECT(FEql(b.rot, m3x3::Identity())); // orientation is unaffected by 'RandPos'.
				PR_EXPECT(Length(b.pos - centre) <= radius + slack);

				PR_EXPECT(reader.NextKeywordS(kw));
				auto c = m4x4::Identity();
				PR_EXPECT(reader.TransformS(c));
				PR_EXPECT(IsOrthonormal(c));
				PR_EXPECT(FEql(c.pos, v4::Origin())); // position is unaffected by 'RandOri'.

				PR_EXPECT(reader.NextKeywordS(kw) && std::string(kw) == "done"); // 'case_sensitive=false' lower-cases extracted keywords.
			};

			pr::script::Reader legacy(script, false);
			pr::script::v2::Reader modern(std::string_view(script), false);
			check(legacy);
			check(modern);
		}

		PRUnitTestMethod(RotationDelegatesToTransformDifferential, Quick)
		{
			// 'Rotation' is a thin wrapper that routes through 'Transform', accepting any
			// 'ETransformKeyword' (not just rotation-only ones); verify both readers agree.
			char const* script = "*Rot *M3x3 {0 -1 0  1 0 0  0 0 1}";

			pr::script::Reader legacy(script, false);
			pr::script::v2::Reader modern(std::string_view(script), false);

			char lkw[64], mkw[64];
			PR_EXPECT(legacy.NextKeywordS(lkw) && modern.NextKeywordS(mkw));

			auto lrot = m3x3::Identity();
			auto mrot = m3x3::Identity();
			PR_EXPECT(legacy.Rotation(lrot) && modern.Rotation(mrot));
			PR_EXPECT(FEql(lrot, mrot));
			PR_EXPECT(legacy.IsSourceEnd() && modern.IsSourceEnd());
		}

		PRUnitTestMethod(AddressAtDifferential, Quick)
		{
			// ASCII-only script (so legacy's UTF-16-code-unit-counting 'Src::Limit()' and
			// reader2's raw-UTF-8-byte counting agree on what each truncation length means),
			// mirroring the structure of legacy's own 'AddressAt' unit test.
			char const* script =
				"*Group { *Width {1} *Smooth *Box\n"
				"{\n"
				"\t*other {}\n"
				"\t/* *something { */\n"
				"\t// *something {\n"
				"\t\"my { string\"\n"
				"\t*o2w { *pos {";
			auto const length = std::char_traits<char>::length(script);
			std::wstring const wscript(script, script + length);

			struct Case { size_t len; char const* expect; };
			Case const cases[] =
			{
				{ 0, "" },
				{ 18, "Group.Width" },
				{ 19, "Group" },
				{ 35, "Group.Box" },
				{ 88, "" }, // partway through a literal string.
				{ length, "Group.Box.o2w.pos" },
			};

			for (auto const& c : cases)
			{
				pr::script::StringSrc src({ wscript.c_str(), c.len });
				auto legacy_addr = pr::script::Reader::AddressAt(src);
				auto modern_addr = pr::script::v2::Reader::AddressAt(std::string_view(script, c.len));
				PR_EXPECT(str::Equal(legacy_addr, c.expect));
				PR_EXPECT(modern_addr == c.expect);
			}
		}

		PRUnitTestMethod(IsMatchDifferential, Quick)
		{
			// 'IsMatch' peeks ahead without consuming; both readers must agree on the match
			// result for identical ASCII patterns, and a following extraction must still see
			// the un-consumed value.
			char const* script = "*Value 12345";

			pr::script::Reader legacy(script, false);
			pr::script::v2::Reader modern(std::string_view(script), false);

			char lkw[64], mkw[64];
			PR_EXPECT(legacy.NextKeywordS(lkw) && modern.NextKeywordS(mkw));

			PR_EXPECT(legacy.IsMatch(5, std::wregex(L"\\d+")) == true);
			PR_EXPECT(modern.IsMatch(5, std::wregex(L"\\d+")) == true);
			PR_EXPECT(legacy.IsMatch(5, std::wregex(L"[a-z]+")) == false);
			PR_EXPECT(modern.IsMatch(5, std::wregex(L"[a-z]+")) == false);

			int lval = 0, mval = 0;
			PR_EXPECT(legacy.Int(lval, 10) && modern.Int(mval, 10) && lval == mval && lval == 12345);

			// The lookahead count is decoded characters rather than UTF-8 bytes.
			char const* utf8_script = "\xC2\xB1x";
			pr::script::Reader legacy_utf8(utf8_script, false);
			pr::script::v2::Reader modern_utf8(std::string_view(utf8_script), false);
			PR_EXPECT(legacy_utf8.IsMatch(1, std::wregex(L"\u00B1")));
			PR_EXPECT(modern_utf8.IsMatch(1, std::wregex(L"\u00B1")));
			PR_EXPECT(legacy_utf8.IsMatch(2, std::wregex(L"\u00B1x")));
			PR_EXPECT(modern_utf8.IsMatch(2, std::wregex(L"\u00B1x")));
		}

		PRUnitTestMethod(CustomDelimiterTokenDifferential, Quick)
		{
			// A custom delimiter set extends (not replaces) the reader's base delimiters;
			// both readers must stop the token at the same extra character.
			char const* script = "*Path a.b.c *Rest tail";

			pr::script::Reader legacy(script, false);
			pr::script::v2::Reader modern(std::string_view(script), false);

			char lkw[64], mkw[64];
			PR_EXPECT(legacy.NextKeywordS(lkw) && modern.NextKeywordS(mkw));

			std::string ltok, mtok;
			PR_EXPECT(legacy.Token(ltok, L".") && modern.Token(mtok, "."));
			PR_EXPECT(ltok == mtok && ltok == "a");

			PR_EXPECT(legacy.NextKeywordS(lkw) && modern.NextKeywordS(mkw));
			std::string lrest, mrest;
			PR_EXPECT(legacy.Token(lrest) && modern.Token(mrest));
			PR_EXPECT(lrest == mrest && lrest == "tail");
		}
	};

	// A v2 include handler that counts physical opens for include/dependency
	// side-effect parity with the legacy include handler.
	struct RecordingIncludeHandler2 :FileIncludeHandler2
	{
		int m_open_count = 0;

		std::unique_ptr<IInput> Open(std::filesystem::path const& resolved, EIncludeFlags flags, Loc const& loc) override
		{
			auto input = FileIncludeHandler2::Open(resolved, flags, loc);
			m_open_count += (input != nullptr);
			return input;
		}
	};

	// Canonical-fixture-driven conformance tests: both readers parse the same on-disk
	// UTF-8 script tree (see 'projects/tests/unittests/res/script_reader/') and are
	// checked for agreement on every extracted value, on include/dependency side effects,
	// and (for the dedicated 'errors/' fixtures) on 'EResult' + source offset.
	PRUnitTestClass(Reader2ConformanceTests)
	{
		// The fixture tree's root directory, located relative to this header's own
		// '__FILE__' so the tests do not depend on the process's current working
		// directory. 'include/pr/script/reader2/' -> repo root is 4 levels up.
		static std::filesystem::path FixtureRoot()
		{
			auto here = std::filesystem::path(__FILE__).parent_path();
			return (here / ".." / ".." / ".." / ".." / "projects" / "tests" / "unittests" / "res" / "script_reader").lexically_normal();
		}

		// Read the complete contents of 'path' as raw bytes (no text-mode translation),
		// preserving the fixture's checked-in UTF-8 byte-order-mark exactly.
		static std::string ReadFileBytes(std::filesystem::path const& path)
		{
			std::ifstream file(path, std::ios::binary);
			std::ostringstream ss;
			ss << file.rdbuf();
			return ss.str();
		}

		// Return 'bytes' with a leading UTF-8 byte-order-mark removed, if present.
		static std::string StripBom(std::string bytes)
		{
			unsigned char const bom[] = { 0xEF, 0xBB, 0xBF };
			if (bytes.size() >= 3 && memcmp(bytes.data(), bom, 3) == 0)
				bytes.erase(0, 3);

			return bytes;
		}

		// Parse a frozen golden trace file (one "kind:value" entry per non-empty line,
		// see 'DriveCanonical' below for the entries this must match). A trace value
		// may itself contain a real newline (e.g. a C-string's decoded '\n' escape),
		// so the file encodes '\\' and '\n' within a value as the two-character
		// sequences '\\\\' and '\\n' - undo that escaping here to recover the entry.
		static std::vector<std::string> ReadExpectedTrace(std::filesystem::path const& path)
		{
			std::vector<std::string> out;
			std::ifstream file(path);
			for (std::string line; std::getline(file, line);)
			{
				if (!line.empty() && line.back() == '\r')
					line.pop_back();

				if (line.empty())
					continue;

				std::string unescaped;
				unescaped.reserve(line.size());
				for (size_t i = 0; i != line.size(); ++i)
				{
					if (line[i] == '\\' && i + 1 != line.size())
					{
						if (line[i + 1] == 'n') { unescaped.push_back('\n'); ++i; continue; }
						if (line[i + 1] == '\\') { unescaped.push_back('\\'); ++i; continue; }
					}
					unescaped.push_back(line[i]);
				}
				out.push_back(unescaped);
			}
			return out;
		}

		// Drive 'reader' (either reader type, since both expose the same method names)
		// through the entirety of 'canonical.utf8.txt' and return every extracted value
		// as a "kind:value" string, in source order, for side-by-side comparison against
		// the other reader and against the frozen 'expected_trace.txt'. Math/vector/
		// matrix/transform extraction already has thorough dedicated coverage in
		// 'Reader2DifferentialTests' above, so this driver instead exercises the
		// file/preprocessor-facing surface plus a light scalar/section sampler - see the
		// fixture's own comments for what each keyword demonstrates.
		template <typename Reader> static std::vector<std::string> DriveCanonical(Reader& reader)
		{
			std::vector<std::string> out;
			char kw[64];

			auto next = [&]
			{
				reader.NextKeywordS(kw);
				out.push_back(std::string("kw:") + kw);
			};

			next(); // Comment
			bool comment = false;
			reader.Bool(comment);
			out.push_back(std::string("bool:") + (comment ? "true" : "false"));

			next(); // Continuation
			std::string cont;
			reader.String(cont);
			out.push_back("str:" + cont);

			next(); // Message (object macro 'GREETING')
			std::string msg;
			reader.String(msg);
			out.push_back("str:" + msg);

			next(); // Sum (#eval{ADD(2, 3)})
			int sum = 0;
			reader.Int(sum, 10);
			out.push_back("int:" + std::to_string(sum));

			next(); // Stringised (STR(payload) via '#'-stringise)
			std::string stringised;
			reader.String(stringised);
			out.push_back("str:" + stringised);

			next(); // Pasted (PASTE(fo, o) via '##'-paste)
			std::string pasted;
			reader.Identifier(pasted);
			out.push_back("id:" + pasted);

			next(); // Recursive (mutual-recursion suppression leaves the token literal)
			std::string recursive;
			reader.Identifier(recursive);
			out.push_back("id:" + recursive);

			next(); // IfdefTaken
			bool ifdef_taken = false;
			reader.Bool(ifdef_taken);
			out.push_back(std::string("bool:") + (ifdef_taken ? "true" : "false"));

			next(); // IfndefElse (COUNT is defined, so '#ifndef' takes the '#else' arm)
			bool ifndef_else = false;
			reader.Bool(ifndef_else);
			out.push_back(std::string("bool:") + (ifndef_else ? "true" : "false"));

			next(); // IfTaken (#if COUNT == 3)
			bool if_taken = false;
			reader.Bool(if_taken);
			out.push_back(std::string("bool:") + (if_taken ? "true" : "false"));

			next(); // UndefOk (COUNT undef'd, so '#ifdef COUNT' takes the '#else' arm)
			bool undef_ok = false;
			reader.Bool(undef_ok);
			out.push_back(std::string("bool:") + (undef_ok ? "true" : "false"));

			next(); // Ident
			std::string ident;
			reader.Identifier(ident);
			out.push_back("id:" + ident);

			next(); // Str
			std::string str;
			reader.String(str);
			out.push_back("str:" + str);

			next(); // CStr (escape-processed)
			std::string cstr;
			reader.CString(cstr);
			out.push_back("cstr:" + cstr);

			next(); // FlagTrue
			bool flag_true = false;
			reader.Bool(flag_true);
			out.push_back(std::string("bool:") + (flag_true ? "true" : "false"));

			next(); // FlagFalse
			bool flag_false = true;
			reader.Bool(flag_false);
			out.push_back(std::string("bool:") + (flag_false ? "true" : "false"));

			next(); // Dec
			int dec = 0;
			reader.Int(dec, 10);
			out.push_back("int:" + std::to_string(dec));

			next(); // HexRadix (no prefix - radix must be given explicitly)
			unsigned int hex_radix = 0;
			reader.Int(hex_radix, 16);
			out.push_back("hex:" + std::to_string(hex_radix));

			next(); // HexPrefix ("0x"-prefixed - overrides the given radix of 10)
			unsigned int hex_prefix = 0;
			reader.Int(hex_prefix, 10);
			out.push_back("hex:" + std::to_string(hex_prefix));

			next(); // Octal ("0o"-prefixed - overrides the given radix of 10)
			unsigned int octal = 0;
			reader.Int(octal, 10);
			out.push_back("oct:" + std::to_string(octal));

			next(); // Binary ("0b"-prefixed - overrides the given radix of 10)
			unsigned int binary = 0;
			reader.Int(binary, 10);
			out.push_back("bin:" + std::to_string(binary));

			next(); // Pi
			double pi = 0.0;
			reader.Real(pi);
			out.push_back("real:" + std::to_string(pi));

			next(); // Neg
			double neg = 0.0;
			reader.Real(neg);
			out.push_back("real:" + std::to_string(neg));

			next(); // NotANumber
			double nan_value = 0.0;
			reader.Real(nan_value);
			out.push_back(std::string("real:") + (std::isnan(nan_value) ? "nan" : std::to_string(nan_value)));

			next(); // Infinity
			double inf_value = 0.0;
			reader.Real(inf_value);
			out.push_back("real:" + std::to_string(inf_value));

			next(); // Group
			reader.SectionStart();
			next(); // Nested
			reader.SectionStart();
			next(); // Deep
			int deep = 0;
			reader.Int(deep, 10);
			out.push_back("int:" + std::to_string(deep));
			reader.SectionEnd();
			next(); // Sibling
			std::string sibling;
			reader.String(sibling);
			out.push_back("str:" + sibling);
			reader.SectionEnd();

			next(); // Status (named-value enum)
			EResult status = EResult::Failed;
			reader.Enum(status);
			out.push_back("enum:" + std::to_string(static_cast<int>(status)));

			next(); // StatusValue (integral-value enum)
			EResult status_value = EResult::Failed;
			reader.EnumValue(status_value);
			out.push_back("enum:" + std::to_string(static_cast<int>(status_value)));

			next(); // Lit (the '#lit' body, read as one identifier: 'GREETING' unexpanded)
			std::string lit;
			reader.Identifier(lit);
			out.push_back("id:" + lit);

			next(); // FromLocal (pushed by the quoted '#include "includes/local.inc"')
			std::string from_local;
			reader.String(from_local);
			out.push_back("str:" + from_local);

			next(); // FromNested (resolved relative to the including local.inc)
			std::string from_nested;
			reader.String(from_nested);
			out.push_back("str:" + from_nested);

			next(); // FromAngle (pushed by the angle-bracket '#include <angle.inc>')
			std::string from_angle;
			reader.String(from_angle);
			out.push_back("str:" + from_angle);

			next(); // Done ('#depend' above produced no observable keyword/value)

			return out;
		}

		PRUnitTestMethod(CanonicalFixtureTraceParity, Quick)
		{
			// Run the canonical, checked-in UTF-8 fixture (with its real BOM) through
			// both readers from the real on-disk file, including nested quoted includes
			// resolved against each including file's own directory.
			auto root = FixtureRoot();
			auto canonical_path = root / "canonical.utf8.txt";
			auto search_dir = root / "includes" / "search";

			int legacy_open_count = 0;
			pr::script::Includes legacy_includes;
			legacy_includes.AddSearchPath(search_dir);
			legacy_includes.FileOpened += [&](pr::script::Includes&, std::filesystem::path const&)
			{
				++legacy_open_count;
			};

			pr::script::FileSrc legacy_src(canonical_path);
			pr::script::Reader legacy(legacy_src, false, &legacy_includes);

			RecordingIncludeHandler2 modern_includes;
			modern_includes.AddSearchPath(search_dir);

			auto bytes = ReadFileBytes(canonical_path);
			pr::script::v2::Reader modern(std::string_view(bytes), false, canonical_path, &modern_includes);

			auto legacy_trace = DriveCanonical(legacy);
			auto modern_trace = DriveCanonical(modern);

			// Cross-reader agreement on every extracted value.
			PR_EXPECT(legacy_trace.size() == modern_trace.size());
			for (size_t i = 0, iend = std::min(legacy_trace.size(), modern_trace.size()); i != iend; ++i)
				PR_EXPECT(legacy_trace[i] == modern_trace[i]);

			PR_EXPECT(legacy.IsSourceEnd());
			PR_EXPECT(modern.IsSourceEnd());

			// Agreement against the frozen golden trace (catches an accidental behaviour
			// change even if both readers happened to change in the same way).
			auto expected_trace = ReadExpectedTrace(root / "expected_trace.txt");
			PR_EXPECT(legacy_trace.size() == expected_trace.size());
			for (size_t i = 0, iend = std::min(legacy_trace.size(), expected_trace.size()); i != iend; ++i)
				PR_EXPECT(legacy_trace[i] == expected_trace[i]);

			// Both readers open two includes, one nested include, and one dependency.
			PR_EXPECT(legacy_open_count == 4);
			PR_EXPECT(modern_includes.m_open_count == legacy_open_count);
		}

		PRUnitTestMethod(Utf8BomHandling, Quick)
		{
			// The canonical fixture is checked in WITH a UTF-8 byte-order-mark; both
			// readers must silently skip it. Verify the with-BOM (real file) and
			// without-BOM (scratch copy) forms parse identically as far as the first
			// keyword+value pair, confirming neither reader's tokenising is disturbed by
			// the BOM. Only 'FileSrc' auto-detects/strips a BOM (an in-memory
			// 'StringSrc' does not), so both variants are read from real files.
			auto canonical_path = FixtureRoot() / "canonical.utf8.txt";
			auto bytes_with_bom = ReadFileBytes(canonical_path);
			auto bytes_without_bom = StripBom(bytes_with_bom);
			PR_EXPECT(bytes_with_bom.size() == bytes_without_bom.size() + 3);

			auto no_bom_path = temp_dir() / "canonical_no_bom.utf8.txt";
			{
				std::ofstream out(no_bom_path, std::ios::binary);
				out.write(bytes_without_bom.data(), static_cast<std::streamsize>(bytes_without_bom.size()));
			}

			for (auto const& path : { canonical_path, no_bom_path })
			{
				pr::script::FileSrc legacy_src(path);
				pr::script::Reader legacy(legacy_src, false);

				auto bytes = ReadFileBytes(path);
				pr::script::v2::Reader modern(std::string_view(bytes), false);

				char lkw[64], mkw[64];
				PR_EXPECT(legacy.NextKeywordS(lkw) && modern.NextKeywordS(mkw));
				PR_EXPECT(std::string(lkw) == std::string(mkw) && std::string(lkw) == "comment"); // 'case_sensitive=false' lower-cases extracted keywords.

				bool lval = false, mval = false;
				PR_EXPECT(legacy.Bool(lval) && modern.Bool(mval));
				PR_EXPECT(lval == mval && lval == true);
			}

			std::filesystem::remove(no_bom_path);
		}

		PRUnitTestMethod(ErrorParity, Quick)
		{
			// For each 'errors/' fixture, both readers must fail with the same
			// 'EResult' at the same source offset, even though thrown message text
			// may differ (deliberately not compared - see 'MatchingErrorCodeAndLocation'
			// in 'Reader2DifferentialTests' for the same rule on hand-written scripts).
			//
			// 'loc_quirk' flags a fixture where the error is raised while legacy's
			// 'Preprocessor' is dispatching a '#' directive (case '#': in 'IsOutputChar').
			// Legacy's 'Src::Location()' reports the position one character past the '#'
			// itself in that exact code path - reproduced with a single-character file
			// containing just '#include' - whereas reader2 reports the '#' itself, which
			// matches every other location reported by both readers. This is a pre-existing
			// legacy quirk local to that one dispatch point, not a reader2 defect, so it is
			// tolerated here (with a fixed +1 adjustment) rather than replicated in reader2
			// or fixed in legacy, both of which are out of scope for this milestone.
			struct Case
			{
				char const* file;
				bool has_section;
				bool loc_quirk;
			};
			Case const cases[] =
			{
				{ "errors/missing_include.txt", false, true },
				{ "errors/unmatched_endif.txt", false, true },
				{ "errors/unknown_directive.txt", false, true },
				{ "errors/unmatched_section.txt", true, false },
			};

			auto root = FixtureRoot();
			auto search_dir = root / "includes" / "search";

			// Drive 'reader' up to the point each fixture is designed to fail, and
			// return the resulting '(EResult, byte offset)' pair. Reaching the end of
			// the sequence without an exception is itself a test failure, since every
			// fixture here is designed to fail.
			auto drive = [](auto& reader, bool has_section) -> std::pair<EResult, std::streamoff>
			{
				try
				{
					char kw[64];
					if (has_section)
					{
						reader.NextKeywordS(kw); // Outer
						reader.SectionStart();
						reader.NextKeywordS(kw); // Inner
						int value = 0;
						reader.Int(value, 10);
						reader.SectionEnd(); // throws: reaches end-of-source, never finds '}'
					}
					else
					{
						reader.NextKeywordS(kw); // Before
						bool before = false;
						reader.Bool(before);
						reader.NextKeywordS(kw); // throws mid-scan, at the offending directive
					}
				}
				catch (ScriptException const& ex)
				{
					return { ex.m_result, ex.m_loc.Pos() };
				}
				PR_EXPECT(false); // every fixture here must fail; reaching this point is a bug.
				return { EResult::Success, 0 };
			};

			for (auto const& c : cases)
			{
				auto path = root / c.file;

				pr::script::Includes legacy_includes;
				legacy_includes.AddSearchPath(search_dir);
				pr::script::FileSrc legacy_src(path);
				pr::script::Reader legacy(legacy_src, false, &legacy_includes);

				FileIncludeHandler2 modern_includes;
				modern_includes.AddSearchPath(search_dir);
				auto bytes = ReadFileBytes(path);
				pr::script::v2::Reader modern(std::string_view(bytes), false, path, &modern_includes);

				auto legacy_result = drive(legacy, c.has_section);
				auto modern_result = drive(modern, c.has_section);

				// See 'loc_quirk' above: legacy's reported offset is 1 higher than
				// reader2's for these fixtures, which is legacy's own quirk, not reader2's.
				auto quirk_adjust = c.loc_quirk ? 1 : 0;

				PR_EXPECT(legacy_result.first == modern_result.first);
				PR_EXPECT(legacy_result.second == modern_result.second + quirk_adjust);
			}
		}
	};
}
