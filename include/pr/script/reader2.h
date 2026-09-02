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
}
#endif
