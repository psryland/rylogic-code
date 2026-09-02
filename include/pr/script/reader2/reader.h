//**********************************
// Script
//  Copyright (c) Rylogic Ltd 2015
//**********************************
// Reader2 (pr::script::v2) - the UTF-8 native replacement for 'pr::script::Reader'.
//
// Style Guidance:
//  - This is the top-level facade of the reader2 stack: it wraps a 'Preprocessor'
//    (itself wrapping a 'Cursor') and exposes the same keyword/section/value
//    extraction vocabulary as the legacy 'pr::script::Reader', so that call sites
//    can be ported by swapping the type name. Extraction is implemented by
//    delegating to the same char-generic 'pr::str::Extract*' family used by the
//    legacy reader (via 'm_pp', which satisfies the 'Ptr' concept), so parsing
//    rules for numbers/strings/bools stay bit-for-bit identical between readers.
//  - Deliberately NOT supported: constructing from the legacy virtual 'Src'
//    hierarchy, or from a 'wchar_t const*' source. Reader2 only accepts UTF-8
//    bytes (an in-memory buffer or a binary-mode 'std::istream'); converting a
//    legacy wide-character or other-encoding source into UTF-8 is the caller's
//    responsibility (or an integration concern outside this library).
//  - The heavy math-vector/matrix extraction methods (Vector2/3/4, Vector2i/3i/4i,
//    Quaternion, Matrix3x3/4x4, Rotation, Transform, Data, EnumValue, Enum) are an
//    explicit, documented gap: porting them would require pulling in the full
//    'pr::maths' dependency graph, which is out of scope for this milestone.
#pragma once
#include <string>
#include <string_view>
#include <functional>
#include <filesystem>
#include <istream>
#include "pr/script/forward.h"
#include "pr/script/location.h"
#include "pr/script/fail_policy.h"
#include "pr/script/script_core.h"
#include "pr/str/extract.h"
#include "pr/str/string_core.h"
#include "pr/str/string_filter.h"
#include "pr/common/hash.h"
#include "pr/script/reader2/input.h"
#include "pr/script/reader2/preprocessor.h"

namespace pr::script::v2
{
	// A UTF-8 native, pull-based script reader. Reads keywords, sections, and
	// scalar/array values from a preprocessed character stream, reporting errors
	// via the same 'EResult' codes and 'Loc' locations as 'pr::script::Reader' so
	// that callers observe compatible failures (exact message text may differ).
	class Reader
	{
		// The preprocessed character stream this reader extracts values from. Owns
		// (via composition) the whole reader2 pipeline: 'Cursor' -> 'FilterCursor'
		// -> macro/directive expansion.
		Preprocessor m_pp;

		// The set of characters treated as inter-value whitespace/separators.
		std::string m_delim;

		// The text of the most recently recognised keyword (set by 'NextKeywordS'/
		// 'NextKeywordH'), lower-cased unless 'm_case_sensitive' is set.
		std::string m_last_keyword;

		// Whether keyword text/hashing is case-sensitive.
		bool m_case_sensitive;

	public:

		// Notes:
		//  - The extract functions come in four forms:
		//      Thing Thing();
		//      Thing ThingS();
		//      bool Thing(Thing&);
		//      bool ThingS(Thing&);
		//    All functions read one instance of type 'Thing' from the script.
		//    The postfix 'S' means 'within a section'. e.g. { "value" }
		//    If an error occurs in the first two forms, ReportError is called. If ReportError doesn't throw, then a default instance is returned.
		//    If an error occurs in the second two forms, ReportError is called. If ReportError doesn't throw, then false is returned.

		// Construct a reader over a caller-owned UTF-8 memory buffer, which must
		// outlive this reader. 'filepath' is used only for 'Loc' reporting and
		// resolving relative '#include's.
		explicit Reader(std::string_view utf8_src, bool case_sensitive = false, std::filesystem::path filepath = {}, IIncludeHandler2* inc = nullptr)
			: m_pp(utf8_src, std::move(filepath), inc)
			, m_delim(" \t\r\n\v,;")
			, m_last_keyword()
			, m_case_sensitive(case_sensitive)
			, ReportError(DefaultErrorHandler)
		{}

		// Construct a reader that pulls UTF-8 bytes in bulk from a binary-mode
		// stream, which must outlive this reader.
		explicit Reader(std::istream& utf8_stream, bool case_sensitive = false, std::filesystem::path filepath = {}, IIncludeHandler2* inc = nullptr)
			: m_pp(std::make_unique<StreamInput>(utf8_stream, std::move(filepath)), inc)
			, m_delim(" \t\r\n\v,;")
			, m_last_keyword()
			, m_case_sensitive(case_sensitive)
			, ReportError(DefaultErrorHandler)
		{}

		// Readers own a non-copyable, non-movable 'Preprocessor', so they are
		// themselves non-copyable and non-movable; construct in place instead.
		Reader(Reader const&) = delete;
		Reader& operator =(Reader const&) = delete;

		// Allow override of error handling. Called whenever an extraction fails;
		// if it returns without throwing, the failing call reports failure instead.
		std::function<bool(EResult, Loc const&, std::string_view)> ReportError;
		static bool DefaultErrorHandler(EResult result, Loc const& loc, std::string_view msg)
		{
			throw ScriptException(result, loc, msg);
		}

		// Access the underlying preprocessed character stream.
		Preprocessor const& Source() const noexcept
		{
			return m_pp;
		}
		Preprocessor& Source() noexcept
		{
			return m_pp;
		}

		// Return the current source location.
		Loc Location() noexcept
		{
			return m_pp.Location();
		}

		// Access the include handler resolving '#include' directives.
		IIncludeHandler2& Includes() noexcept
		{
			return m_pp.Includes();
		}

		// Access the macro table backing '#define'/'#undef' and macro expansion.
		IMacroHandler& Macros() noexcept
		{
			return m_pp.Macros();
		}

		// Register a handler used to execute '#embedded(lang) ... #end' blocks.
		void EmbeddedLookup(std::function<IEmbeddedCode2*(std::string_view)> lookup)
		{
			m_pp.EmbeddedLookup(std::move(lookup));
		}

		// Get/Set delimiter characters.
		std::string_view Delimiters() const noexcept
		{
			return m_delim;
		}
		void Delimiters(std::string_view delim)
		{
			m_delim = delim;
		}

		// Get/Set case sensitive keywords on/off.
		bool CaseSensitive() const noexcept
		{
			return m_case_sensitive;
		}
		void CaseSensitive(bool cs) noexcept
		{
			m_case_sensitive = cs;
		}

		// Return the hash of a keyword using the current reader settings. Uses the
		// same runtime FNV hash as macro/keyword recognition elsewhere in reader2,
		// which agrees with the compile-time hash used to define 'EKeyword' values
		// for ASCII keyword text.
		int HashKeyword(std::string_view keyword) const
		{
			return m_case_sensitive
				? static_cast<int>(pr::hash::Hash(keyword))
				: static_cast<int>(pr::hash::HashI(keyword.data(), keyword.data() + keyword.size()));
		}

		// Return true if the end of the source has been reached.
		bool IsSourceEnd()
		{
			auto& src = m_pp;
			EatDelimiters(src, m_delim);
			return *src == 0;
		}

		// Return true if the next token is a keyword.
		bool IsKeyword()
		{
			auto& src = m_pp;
			EatDelimiters(src, m_delim);
			return *src == '*';
		}

		// Returns true if the next non-whitespace character is the start/end of a section.
		bool IsSectionStart()
		{
			auto& src = m_pp;
			EatDelimiters(src, m_delim);
			return *src == '{';
		}
		bool IsSectionEnd()
		{
			auto& src = m_pp;
			EatDelimiters(src, m_delim);
			return *src == '}';
		}

		// Return true if the next token is not a keyword, the section end, or the end of the source.
		bool IsValue()
		{
			return !IsKeyword() && !IsSectionEnd() && !IsSourceEnd();
		}

		// Move to the start/end of a section and then one past it.
		bool SectionStart()
		{
			auto& src = m_pp;
			if (IsSectionStart()) { ++src; return true; }
			return ReportError(EResult::TokenNotFound, Location(), "expected '{'");
		}
		bool SectionEnd()
		{
			auto& src = m_pp;
			if (IsSectionEnd()) { ++src; return true; }
			return ReportError(EResult::TokenNotFound, Location(), "expected '}'");
		}

		// Move to the start of the next line.
		bool NewLine()
		{
			auto& src = m_pp;
			EatLine(src, 0, 0, true);
			return *src != 0;
		}

		// Advance the source to the next '{' within the current scope.
		// If true is returned, the current position should be a section start character.
		// If false, then the current position will be '*', '}', or the end of the stream.
		bool FindSectionStart()
		{
			auto& src = m_pp;
			for (; *src && *src != '{' && *src != '}' && *src != '*';)
			{
				if (*src == '\"') { EatLiteral(src); continue; }
				++src;
			}
			return *src == '{';
		}

		// Advance the source to the end of the current section.
		// On return the current position should be the section end character
		// or the end of the input stream (if called from file scope).
		bool FindSectionEnd()
		{
			auto& src = m_pp;
			for (int nest = IsSectionStart() ? 0 : 1; *src;)
			{
				if (*src == '\"') { EatLiteral(src); continue; }
				nest += (*src == '{') ? 1 : 0;
				nest -= (*src == '}') ? 1 : 0;
				if (nest == 0) break;
				++src;
			}
			return *src == '}';
		}

		// Scans forward until a keyword identifier is found within the current scope.
		// Non-keyword tokens are skipped. If a section is found it is skipped.
		// If a keyword is found, the source is positioned at the next character after the keyword.
		// Returns true if a keyword is found, false otherwise.
		template <typename StrType> bool NextKeywordS(StrType& kw)
		{
			auto& src = m_pp;
			for (; *src && *src != '}' && *src != '*';)
			{
				if (*src == '\"') { EatLiteral(src); continue; }
				if (*src == '{') { src += FindSectionEnd(); continue; }
				++src;
			}
			if (*src == '*') ++src; else return false;
			str::Resize(kw, 0);
			if (!str::ExtractIdentifier(kw, src, m_delim)) return false;
			if (!m_case_sensitive) str::LowerCase(kw);
			m_last_keyword.assign(str::BeginC(kw), str::EndC(kw));
			return true;
		}

		// As above except the hash of the keyword is returned instead (converted to an enum value).
		template <typename Enum> bool NextKeywordH(Enum& enum_kw)
		{
			std::string kw;
			if (!NextKeywordS(kw)) return false;
			enum_kw = static_cast<Enum>(HashKeyword(kw));
			return true;
		}

		// As above except an error is reported if the next token is not a keyword.
		int NextKeywordH()
		{
			int kw = 0;
			if (!NextKeywordH(kw)) ReportError(EResult::TokenNotFound, Location(), "keyword expected");
			return kw;
		}
		template <typename Enum> Enum NextKeywordH()
		{
			return static_cast<Enum>(NextKeywordH());
		}

		// Return the last keyword read from the stream.
		std::string const& LastKeyword() const
		{
			return m_last_keyword;
		}

		// Scans forward until a keyword matching 'named_kw' is found within the current scope.
		// Returns false if the named keyword is not found, true if it is.
		bool FindKeyword(std::string_view named_kw)
		{
			int kw_hashed = 0;
			auto const named_kw_hashed = HashKeyword(named_kw);
			for (; NextKeywordH(kw_hashed) && kw_hashed != named_kw_hashed;) {}
			return named_kw_hashed == kw_hashed;
		}

		// Scans forward until a keyword matching 'named_kw' is found within the current scope.
		// Calls ReportError if not found.
		Reader& Keyword(std::string_view named_kw)
		{
			if (!FindKeyword(named_kw))
				ReportError(EResult::KeywordNotFound, Location(), std::string("keyword '").append(named_kw).append("' expected"));

			return *this;
		}

		// Extract a token from the source. A token is a contiguous block of non-separator characters.
		template <typename StrType> StrType Token()
		{
			StrType token;
			return Token(token) ? token : StrType();
		}
		template <typename StrType> StrType TokenS()
		{
			StrType token;
			return TokenS(token) ? token : StrType{};
		}
		template <typename StrType> bool Token(StrType& token)
		{
			auto& src = m_pp;
			str::Resize(token, 0);
			return str::ExtractToken(token, src, m_delim) || ReportError(EResult::TokenNotFound, Location(), "token expected");
		}
		template <typename StrType> bool TokenS(StrType& token)
		{
			return SectionStart() && Token(token) && SectionEnd();
		}

		// Read an identifier from the source. An identifier is one of (A-Z,a-z,'_') followed by (A-Z,a-z,'_',0-9) in a contiguous block.
		template <typename StrType> StrType Identifier()
		{
			StrType word;
			return Identifier(word) ? word : StrType{};
		}
		template <typename StrType> StrType IdentifierS()
		{
			StrType word;
			return IdentifierS(word) ? word : StrType{};
		}
		template <typename StrType> bool Identifier(StrType& word)
		{
			auto& src = m_pp;
			str::Resize(word, 0);
			return str::ExtractIdentifier(word, src, m_delim) || ReportError(EResult::TokenNotFound, Location(), "{identifier} expected");
		}
		template <typename StrType> bool IdentifierS(StrType& word)
		{
			return SectionStart() && Identifier(word) && SectionEnd();
		}

		// Extract identifiers from the source separated by 'sep'.
		template <typename StrType> bool Identifiers(char, StrType& word)
		{
			auto& src = m_pp;
			str::Resize(word, 0);
			return str::ExtractIdentifier(word, src, m_delim) || ReportError(EResult::TokenNotFound, Location(), "identifier expected");
		}
		template <typename StrType, typename... StrTypes> bool Identifiers(char sep, StrType& word, StrTypes&&... words)
		{
			auto& src = m_pp;
			str::Resize(word, 0);
			if (!str::ExtractIdentifier(word, src, m_delim)) return ReportError(EResult::TokenNotFound, Location(), "identifier expected");
			if (*src == sep) ++src; else return ReportError(EResult::TokenNotFound, Location(), "identifier separator expected");
			return Identifiers(sep, std::forward<StrTypes>(words)...);
		}
		template <typename StrType, typename... StrTypes> bool IdentifiersS(char sep, StrType& word, StrTypes&&... words)
		{
			return SectionStart() && Identifiers(sep, word, std::forward<StrTypes>(words)...) && SectionEnd();
		}

		// Extract a string from the source. A string is a sequence of characters between quotes.
		template <typename StrType> StrType String()
		{
			StrType string;
			return String(string) ? string : StrType{};
		}
		template <typename StrType> StrType StringS()
		{
			StrType string;
			return StringS(string) ? string : StrType{};
		}
		template <typename StrType> bool String(StrType& string)
		{
			auto& src = m_pp;
			str::Resize(string, 0);
			if (str::ExtractString<StrType>(string, src, m_delim))
			{
				str::ProcessIndentedNewlines(string);
				return true;
			}
			return ReportError(EResult::TokenNotFound, Location(), "string expected");
		}
		template <typename StrType> bool StringS(StrType& string)
		{
			return SectionStart() && String(string) && SectionEnd();
		}

		// Extract a C-style string from the source.
		template <typename StrType> StrType CString()
		{
			StrType cstring;
			return CString(cstring) ? cstring : StrType{};
		}
		template <typename StrType> StrType CStringS()
		{
			StrType cstring;
			return CStringS(cstring) ? cstring : StrType{};
		}
		template <typename StrType> bool CString(StrType& cstring)
		{
			auto& src = m_pp;
			str::Resize(cstring, 0);
			return str::ExtractString<StrType>(cstring, src, '\\', {}, m_delim) || ReportError(EResult::TokenNotFound, Location(), "'cstring' expected");
		}
		template <typename StrType> bool CStringS(StrType& cstring)
		{
			return SectionStart() && CString(cstring) && SectionEnd();
		}

		// Extract a filepath string from the source.
		std::filesystem::path Filepath()
		{
			std::filesystem::path filepath;
			return Filepath(filepath) ? filepath : std::filesystem::path{};
		}
		std::filesystem::path FilepathS()
		{
			std::filesystem::path filepath;
			return FilepathS(filepath) ? filepath : std::filesystem::path{};
		}
		bool Filepath(std::filesystem::path& path)
		{
			std::string s;
			if (!String(s)) return ReportError(EResult::InvalidString, Location(), "'filepath' string expected");

			// Resolve relative to the include search path, same as the legacy reader.
			path = s;
			path = Includes().ResolveInclude(path, EIncludeFlags::IncludeLocalDir | EIncludeFlags::IgnoreMissing, Location());
			return true;
		}
		bool FilepathS(std::filesystem::path& path)
		{
			return SectionStart() && Filepath(path) && SectionEnd();
		}

		// Extract a bool from the source.
		bool Bool()
		{
			bool bool_;
			return Bool(bool_) ? bool_ : false;
		}
		bool BoolS()
		{
			bool bool_;
			return BoolS(bool_) ? bool_ : false;
		}
		bool Bool(bool& bool_)
		{
			auto& src = m_pp;
			return str::ExtractBool(bool_, src, m_delim) || ReportError(EResult::TokenNotFound, Location(), "bool expected");
		}
		bool BoolS(bool& bool_)
		{
			return SectionStart() && Bool(bool_) && SectionEnd();
		}
		bool Bool(bool* bools, size_t num_bools)
		{
			for (; num_bools && Bool(*bools); --num_bools, ++bools) {}
			return num_bools == 0;
		}
		bool BoolS(bool* bools, size_t num_bools)
		{
			return SectionStart() && Bool(bools, num_bools) && SectionEnd();
		}

		// Extract an integral type from the source.
		template <typename TInt> TInt Int(int radix)
		{
			TInt int_;
			return Int(int_, radix) ? int_ : TInt{};
		}
		template <typename TInt> TInt IntS(int radix)
		{
			TInt int_;
			return IntS(int_, radix) ? int_ : TInt{};
		}
		template <typename TInt> bool Int(TInt& int_, int radix)
		{
			auto& src = m_pp;
			return str::ExtractInt(int_, radix, src, m_delim) || ReportError(EResult::TokenNotFound, Location(), "integral value expected");
		}
		template <typename TInt> bool IntS(TInt& int_, int radix)
		{
			return SectionStart() && Int(int_, radix) && SectionEnd();
		}
		template <typename TInt> bool Int(TInt* ints, size_t num_ints, int radix)
		{
			for (; num_ints && Int(*ints, radix); --num_ints, ++ints) {}
			return num_ints == 0;
		}
		template <typename TInt> bool IntS(TInt* ints, size_t num_ints, int radix)
		{
			return SectionStart() && Int(ints, num_ints, radix) && SectionEnd();
		}

		// Extract a real from the source.
		template <typename TReal> TReal Real()
		{
			TReal real_;
			return Real(real_) ? real_ : TReal{};
		}
		template <typename TReal> TReal RealS()
		{
			TReal real_;
			return RealS(real_) ? real_ : TReal{};
		}
		template <typename TReal> bool Real(TReal& real_)
		{
			auto& src = m_pp;
			return str::ExtractReal(real_, src, m_delim) || ReportError(EResult::TokenNotFound, Location(), "real expected");
		}
		template <typename TReal> bool RealS(TReal& real_)
		{
			return SectionStart() && Real(real_) && SectionEnd();
		}
		template <typename TReal> bool Real(TReal* reals, size_t num_reals)
		{
			for (; num_reals && Real(*reals); --num_reals, ++reals) {}
			return num_reals == 0;
		}
		template <typename TReal> bool RealS(TReal* reals, size_t num_reals)
		{
			return SectionStart() && Real(reals, num_reals) && SectionEnd();
		}

		// Extract a complete section as a preprocessed string. Note: to embed
		// arbitrary text in a script use '#lit'/'#end' and then 'Section()'.
		template <typename String> String Section(bool include_braces)
		{
			String s;
			return Section(s, include_braces) ? s : String{};
		}
		template <typename String> bool Section(String& str, bool include_braces)
		{
			// Do not resize 'str' to 0 here, that's the caller's decision.
			auto& src = m_pp;
			pr::str::InLiteral lit;
			if (IsSectionStart()) ++src; else return ReportError(EResult::TokenNotFound, Location(), "expected '{'");
			if (include_braces) pr::str::Append(str, '{');
			for (int nest = 1; *src; ++src)
			{
				// While within a string/character literal, '{'/'}' characters don't affect nesting.
				if (lit.WithinLiteral(*src)) { pr::str::Append(str, *src); continue; }
				nest += int(*src == '{');
				nest -= int(*src == '}');
				if (nest == 0) break;
				pr::str::Append(str, *src);
			}
			if (include_braces) pr::str::Append(str, '}');
			if (IsSectionEnd()) ++src; else return ReportError(EResult::TokenNotFound, Location(), "expected '}'");
			return true;
		}
	};
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::script::v2::testing
{
	PRUnitTestClass(Reader2ReaderTests)
	{
		// A gold-reference-style script covering the core (non-math) extraction
		// facade, deliberately mirroring the legacy 'ReaderTests::Script' subset
		// that doesn't require 'pr::maths' types.
		inline static constexpr char const* Script =
			"#define NUM 23\n"
			"*Identifier ident\n"
			"*String \"simple string\"\n"
			"*CString \"C:\\\\Path\\\\Filename.txt\"\n"
			"*Bool true\n"
			"*Intg -NUM\n"
			"*Intg16 ABCDEF00\n"
			"*Real -2.3e+3\n"
			"*BoolArray 1 0 true false\n"
			"*IntArray -3 2 +1 -0\n"
			"*RealArray 2.3 -1.0e-1 2 -0.2\n"
			"*Section {*SubSection { *Data \n NUM \"With a }\\\"string\\\"{ in it\" }}    \n"
			"*Token 123token\n"
			"*LastThing";

		PRUnitTestMethod(BasicExtractMethods, Quick)
		{
			char kw[50];
			int hashed_kw = 0;
			std::string str;
			bool bval = false, barray[4];
			int ival = 0, iarray[4];
			unsigned int uival = 0;
			float fval = 0.0f, farray[4];

			Reader reader(Script, true);
			PR_EXPECT(reader.CaseSensitive());
			PR_EXPECT(reader.NextKeywordS(kw) && std::string(kw) == "Identifier");
			PR_EXPECT(reader.Identifier(str) && str == "ident");
			PR_EXPECT(reader.NextKeywordS(kw) && std::string(kw) == "String");
			PR_EXPECT(reader.String(str) && str == "simple string");
			PR_EXPECT(reader.NextKeywordH(hashed_kw) && hashed_kw == reader.HashKeyword("CString"));
			PR_EXPECT(reader.CString(str) && str == "C:\\Path\\Filename.txt");
			PR_EXPECT(reader.NextKeywordS(kw) && std::string(kw) == "Bool");
			PR_EXPECT(reader.Bool(bval) && bval == true);
			PR_EXPECT(reader.NextKeywordS(kw) && std::string(kw) == "Intg");
			PR_EXPECT(reader.Int(ival, 10) && ival == -23);
			PR_EXPECT(reader.NextKeywordS(kw) && std::string(kw) == "Intg16");
			PR_EXPECT(reader.Int(uival, 16) && uival == 0xABCDEF00);
			PR_EXPECT(reader.NextKeywordS(kw) && std::string(kw) == "Real");
			PR_EXPECT(reader.Real(fval) && fval == -2.3e+3f);
			PR_EXPECT(reader.NextKeywordS(kw) && std::string(kw) == "BoolArray");
			PR_EXPECT(reader.Bool(barray, 4));
			PR_EXPECT(barray[0]);
			PR_EXPECT(!barray[1]);
			PR_EXPECT(barray[2]);
			PR_EXPECT(!barray[3]);
			PR_EXPECT(reader.NextKeywordS(kw) && std::string(kw) == "IntArray");
			PR_EXPECT(reader.Int(iarray, 4, 10));
			PR_EXPECT(iarray[0] == -3);
			PR_EXPECT(iarray[1] == +2);
			PR_EXPECT(iarray[2] == +1);
			PR_EXPECT(iarray[3] == -0);
			PR_EXPECT(reader.NextKeywordS(kw) && std::string(kw) == "RealArray");
			PR_EXPECT(reader.Real(farray, 4));
			PR_EXPECT(farray[0] == 2.3f);
			PR_EXPECT(farray[1] == -1.0e-1f);
			PR_EXPECT(farray[2] == +2.0f);
			PR_EXPECT(farray[3] == -0.2f);
			PR_EXPECT(reader.FindKeyword("Section")); str.resize(0);
			PR_EXPECT(reader.Section(str, false) && str == "*SubSection { *Data \n 23 \"With a }\\\"string\\\"{ in it\" }");
			PR_EXPECT(reader.NextKeywordS(kw) && std::string(kw) == "Token");
			PR_EXPECT(reader.Token(str) && str == "123token");
			PR_EXPECT(reader.NextKeywordS(kw) && std::string(kw) == "LastThing");
			PR_EXPECT(reader.IsKeyword() == false);
			PR_EXPECT(reader.IsSectionStart() == false);
			PR_EXPECT(reader.IsSectionEnd() == false);
			PR_EXPECT(reader.IsSourceEnd());
		}
		PRUnitTestMethod(DotDelimitedIdentifiers, Quick)
		{
			char const* s =
				"A.B\n"
				"a.b.c\n"
				"A.B.C.D\n"
				;
			std::string s0, s1, s2, s3;

			Reader reader(s);
			reader.Identifiers('.', s0, s1);       PR_EXPECT(s0 == "A" && s1 == "B");
			reader.Identifiers('.', s0, s1, s2);    PR_EXPECT(s0 == "a" && s1 == "b" && s2 == "c");
			reader.Identifiers('.', s0, s1, s2, s3); PR_EXPECT(s0 == "A" && s1 == "B" && s2 == "C" && s3 == "D");
		}
		PRUnitTestMethod(FilepathExtraction, Quick)
		{
			// With no include handler, resolution degrades to the default 'NoIncludes2'
			// handler, matching the legacy reader's default 'NoIncludes' behaviour: the
			// call succeeds (because 'IgnoreMissing' is set) but yields an empty path
			// rather than throwing 'EResult::IncludesNotSupported'.
			Reader reader("\"foo/bar.txt\"");
			auto path = reader.Filepath();
			PR_EXPECT(path.empty());
		}
		PRUnitTestMethod(SpansMultipleBlocks, Quick)
		{
			// Pad the script well past 'BlockSize' so extraction exercises 'Cursor'
			// refill/compaction boundaries part-way through a keyword/value pair.
			std::string script(BlockSize * 2 + 5, ' ');
			script += "*Value 42";

			Reader reader(script);
			char kw[50];
			int ival = 0;
			PR_EXPECT(reader.NextKeywordS(kw) && std::string(kw) == "value");
			PR_EXPECT(reader.Int(ival, 10) && ival == 42);
			PR_EXPECT(reader.IsSourceEnd());
		}
	};
}
#endif
