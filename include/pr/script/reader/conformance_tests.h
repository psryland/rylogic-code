//**********************************
// Script
//  Copyright (c) Rylogic Ltd 2015
//**********************************
// Reader conformance tests.
//
// Style Guidance:
//  - This header holds fixture-driven tests for 'pr::script::Reader'.
//    It lives beside the rest of the reader stack (rather than under
//    'projects/tests/unittests/src/') so the tests travel with the reader they exercise,
//    matching this stack's existing convention of keeping unit tests inline. It is
//    '#include'd by 'reader.h' only under 'PR_UNITTESTS'.
//  - 'ReaderConformanceTests' drives one canonical, checked-in UTF-8 fixture tree so
//    preprocessor and extraction coverage has a human-readable source of truth.
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
#include "pr/script/reader/reader.h"

namespace pr::script::reader::testing
{
	struct RecordingIncludeHandler :FileIncludeHandler
	{
		int m_open_count = 0;

		std::unique_ptr<IInput> Open(std::filesystem::path const& resolved, EIncludeFlags flags, Loc const& loc) override
		{
			auto input = FileIncludeHandler::Open(resolved, flags, loc);
			m_open_count += (input != nullptr);
			return input;
		}
	};

	// Canonical-fixture-driven conformance tests for extraction, includes, preprocessing,
	// byte-order marks, and error reporting.
	PRUnitTestClass(ReaderConformanceTests)
	{
		// The fixture tree's root directory, located relative to this header's own
		// '__FILE__' so the tests do not depend on the process's current working
		// directory. 'include/pr/script/reader/' -> repo root is 4 levels up.
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

		// Drive the complete canonical fixture and return each extracted value as a stable
		// "kind:value" entry for comparison with the checked-in trace.
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

		PRUnitTestMethod(CanonicalFixtureTrace, Quick)
		{
			// Parse the checked-in UTF-8 fixture with nested local and search-path includes.
			auto root = FixtureRoot();
			auto canonical_path = root / "canonical.utf8.txt";
			auto search_dir = root / "includes" / "search";

			RecordingIncludeHandler includes;
			includes.AddSearchPath(search_dir);
			auto bytes = ReadFileBytes(canonical_path);
			pr::script::Reader reader(std::string_view(bytes), false, canonical_path, &includes);
			auto trace = DriveCanonical(reader);

			// The semantic trace and include side effects are the stable fixture contract.
			auto expected_trace = ReadExpectedTrace(root / "expected_trace.txt");
			PR_EXPECT(trace.size() == expected_trace.size());
			for (size_t i = 0, iend = std::min(trace.size(), expected_trace.size()); i != iend; ++i)
				PR_EXPECT(trace[i] == expected_trace[i]);

			PR_EXPECT(reader.IsSourceEnd());
			PR_EXPECT(includes.m_open_count == 4);
		}

		PRUnitTestMethod(Utf8BomHandling, Quick)
		{
			// A leading UTF-8 byte-order mark must not change the first keyword or value.
			auto canonical_path = FixtureRoot() / "canonical.utf8.txt";
			auto bytes_with_bom = ReadFileBytes(canonical_path);
			auto bytes_without_bom = StripBom(bytes_with_bom);
			PR_EXPECT(bytes_with_bom.size() == bytes_without_bom.size() + 3);

			for (auto const& bytes : { bytes_with_bom, bytes_without_bom })
			{
				pr::script::Reader reader(std::string_view(bytes), false);
				char keyword[64];
				PR_EXPECT(reader.NextKeywordS(keyword));
				PR_EXPECT(std::string_view(keyword) == "comment");

				bool value = false;
				PR_EXPECT(reader.Bool(value));
				PR_EXPECT(value);
			}
		}

		PRUnitTestMethod(ErrorReporting, Quick)
		{
			// Each invalid fixture must report its expected error category and source identity.
			struct Case
			{
				char const* file;
				bool has_section;
				EResult result;
			};
			Case const cases[] =
			{
				{ "errors/missing_include.txt", false, EResult::MissingInclude },
				{ "errors/unmatched_endif.txt", false, EResult::UnmatchedPreprocessorDirective },
				{ "errors/unknown_directive.txt", false, EResult::UnknownPreprocessorCommand },
				{ "errors/unmatched_section.txt", true, EResult::TokenNotFound },
			};

			auto root = FixtureRoot();
			auto search_dir = root / "includes" / "search";

			// Drive each reader to the operation that the fixture makes invalid.
			auto drive = [](auto& reader, bool has_section) -> ScriptException
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
					return ex;
				}
				PR_EXPECT(false); // every fixture here must fail; reaching this point is a bug.
				return ScriptException(EResult::Success, {}, "fixture did not fail");
			};

			for (auto const& c : cases)
			{
				auto path = root / c.file;

				FileIncludeHandler includes;
				includes.AddSearchPath(search_dir);
				auto bytes = ReadFileBytes(path);
				pr::script::Reader reader(std::string_view(bytes), false, path, &includes);
				auto error = drive(reader, c.has_section);

				PR_EXPECT(error.m_result == c.result);
				PR_EXPECT(error.m_loc.Line() >= 1);
				PR_EXPECT(error.m_loc.Col() >= 1);
			}
		}
	};
}
