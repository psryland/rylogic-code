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
//  - The math-vector/matrix/transform extraction methods (Vector2/3/4, Vector2i/3i/4i,
//    Quaternion, Matrix3x3/4x4, Rotation, Transform, Data, EnumValue, Enum) mirror the
//    legacy signatures using 'pr::maths' types directly; they are plain compositions of
//    'Real'/'Int' (or, for 'Transform', of those plus the shared 'pr::maths' functions),
//    so parsing rules stay identical to the legacy reader. 'AddressAt' takes a
//    'std::string_view' rather than a legacy 'Src&', since reader2 has no shareable
//    positioned-source abstraction to construct a nested reader over.
#pragma once
#include <string>
#include <string_view>
#include <functional>
#include <filesystem>
#include <istream>
#include <regex>
#include <random>
#include <vector>
#include "pr/math/math.h"
#include "pr/script/forward.h"
#include "pr/script/location.h"
#include "pr/script/fail_policy.h"
#include "pr/script/script_core.h"
#include "pr/str/extract.h"
#include "pr/str/char8.h"
#include "pr/str/string_core.h"
#include "pr/str/string_filter.h"
#include "pr/common/hash.h"
#include "pr/script/reader2/input.h"
#include "pr/script/reader2/preprocessor.h"

namespace pr::script::v2
{
	// Convert UTF-8 script text to a filesystem path without using the process code page.
	inline std::filesystem::path PathFromUtf8(std::string_view utf8)
	{
		return std::filesystem::path(std::u8string(pr::char8_ptr(utf8.data()), utf8.size()));
	}

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

		// Whether the caller supplied an include handler that can resolve extracted filepaths.
		bool m_has_includes;

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
			, m_has_includes(inc != nullptr)
			, ReportError(DefaultErrorHandler)
		{}

		// Construct a reader that pulls UTF-8 bytes in bulk from a binary-mode
		// stream, which must outlive this reader.
		explicit Reader(std::istream& utf8_stream, bool case_sensitive = false, std::filesystem::path filepath = {}, IIncludeHandler2* inc = nullptr)
			: m_pp(std::make_unique<StreamInput>(utf8_stream, std::move(filepath)), inc)
			, m_delim(" \t\r\n\v,;")
			, m_last_keyword()
			, m_case_sensitive(case_sensitive)
			, m_has_includes(inc != nullptr)
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

		// Buffer the next 'n' decoded characters without consuming them and test them against a wide regex.
		bool IsMatch(int n, std::wregex const& pattern)
		{
			auto& src = m_pp;
			EatDelimiters(src, m_delim);

			// Preserve complete UTF-8 sequences so 'n' has the same decoded-character meaning as the legacy reader.
			std::string utf8;
			for (size_t offset = 0, chars = 0; chars != static_cast<size_t>(n);)
			{
				auto byte = static_cast<unsigned char>(src[offset]);
				if (byte == 0)
					break;

				utf8.push_back(static_cast<char>(byte));
				++offset;
				if ((byte & 0xC0) != 0x80)
					++chars;

				// A decoded character includes all continuation bytes following its lead byte.
				if (chars == static_cast<size_t>(n))
				{
					for (; (static_cast<unsigned char>(src[offset]) & 0xC0) == 0x80; ++offset)
						utf8.push_back(src[offset]);
				}
			}
			return std::regex_match(pr::Widen(utf8), pattern);
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

		// As above, but additionally splitting on 'delim' (beyond the reader's normal 'Delimiters()').
		// Unlike legacy's 'wchar_t const*' overload, 'delim' is UTF-8 bytes.
		template <typename StrType> StrType Token(std::string_view delim)
		{
			StrType token;
			return Token(token, delim) ? token : StrType();
		}
		template <typename StrType> StrType TokenS(std::string_view delim)
		{
			StrType token;
			return TokenS(token, delim) ? token : StrType{};
		}
		template <typename StrType> bool Token(StrType& token, std::string_view delim)
		{
			auto& src = m_pp;
			str::Resize(token, 0);
			return str::ExtractToken(token, src, std::string(m_delim).append(delim)) || ReportError(EResult::TokenNotFound, Location(), "token expected");
		}
		template <typename StrType> bool TokenS(StrType& token, std::string_view delim)
		{
			return SectionStart() && Token(token, delim) && SectionEnd();
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

			// Interpret the source spelling as UTF-8 before optionally resolving it through the configured include handler.
			path = PathFromUtf8(s);
			if (m_has_includes)
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

		// Extract an enum value (its underlying integral representation) from the source.
		template <typename TEnum> TEnum EnumValue(int radix = 10)
		{
			TEnum enum_;
			return EnumValue(enum_, radix) ? enum_ : TEnum{};
		}
		template <typename TEnum> TEnum EnumValueS(int radix = 10)
		{
			TEnum enum_;
			return EnumValueS(enum_, radix) ? enum_ : TEnum{};
		}
		template <typename TEnum> bool EnumValue(TEnum& enum_, int radix = 10)
		{
			auto& src = m_pp;
			return str::ExtractEnumValue(enum_, radix, src, m_delim) || ReportError(EResult::TokenNotFound, Location(), "enum integral value expected");
		}
		template <typename TEnum> bool EnumValueS(TEnum& enum_, int radix = 10)
		{
			return SectionStart() && EnumValue(enum_, radix) && SectionEnd();
		}

		// Extract an enum identifier (its member name) from the source.
		template <typename TEnum> TEnum Enum()
		{
			TEnum enum_;
			return Enum(enum_) ? enum_ : TEnum{};
		}
		template <typename TEnum> TEnum EnumS()
		{
			TEnum enum_;
			return EnumS(enum_) ? enum_ : TEnum{};
		}
		template <typename TEnum> bool Enum(TEnum& enum_)
		{
			auto& src = m_pp;
			return str::ExtractEnum(enum_, src, m_delim) || ReportError(EResult::TokenNotFound, Location(), "enum member string name expected");
		}
		template <typename TEnum> bool EnumS(TEnum& enum_)
		{
			return SectionStart() && Enum(enum_) && SectionEnd();
		}

		// Extract a 2D real vector from the source.
		pr::v2 Vector2()
		{
			pr::v2 vector;
			return Vector2(vector) ? vector : pr::v2{};
		}
		pr::v2 Vector2S()
		{
			pr::v2 vector;
			return Vector2S(vector) ? vector : pr::v2{};
		}
		bool Vector2(pr::v2& vector)
		{
			return Real(vector.x) && Real(vector.y);
		}
		bool Vector2S(pr::v2& vector)
		{
			return SectionStart() && Vector2(vector) && SectionEnd();
		}

		// Extract a 2D integer vector from the source.
		iv2 Vector2i(int radix = 10)
		{
			iv2 vector;
			return Vector2i(vector, radix) ? vector : iv2{};
		}
		iv2 Vector2iS(int radix = 10)
		{
			iv2 vector;
			return Vector2iS(vector, radix) ? vector : iv2{};
		}
		bool Vector2i(iv2& vector, int radix = 10)
		{
			return Int(vector.x, radix) && Int(vector.y, radix);
		}
		bool Vector2iS(iv2& vector, int radix = 10)
		{
			return SectionStart() && Vector2i(vector, radix) && SectionEnd();
		}

		// Extract a 3D real vector from the source. 'w' fills in the unread 4th component.
		v4 Vector3(float w)
		{
			v4 vector;
			return Vector3(vector, w) ? vector : v4{};
		}
		v4 Vector3S(float w)
		{
			v4 vector;
			return Vector3S(vector, w) ? vector : v4{};
		}
		bool Vector3(v4& vector, float w)
		{
			vector.w = w;
			return Real(vector.x) && Real(vector.y) && Real(vector.z);
		}
		bool Vector3S(v4& vector, float w)
		{
			return SectionStart() && Vector3(vector, w) && SectionEnd();
		}

		// Extract a 3D integer vector from the source. 'w' fills in the unread 4th component.
		iv4 Vector3i(int w, int radix = 10)
		{
			iv4 vector;
			return Vector3i(vector, w, radix) ? vector : iv4{};
		}
		iv4 Vector3iS(int w, int radix = 10)
		{
			iv4 vector;
			return Vector3iS(vector, w, radix) ? vector : iv4{};
		}
		bool Vector3i(iv4& vector, int w, int radix = 10)
		{
			vector.w = w;
			return Int(vector.x, radix) && Int(vector.y, radix) && Int(vector.z, radix);
		}
		bool Vector3iS(iv4& vector, int w, int radix = 10)
		{
			return SectionStart() && Vector3i(vector, w, radix) && SectionEnd();
		}

		// Extract a 4D real vector from the source.
		v4 Vector4()
		{
			v4 vector;
			return Vector4(vector) ? vector : v4{};
		}
		v4 Vector4S()
		{
			v4 vector;
			return Vector4S(vector) ? vector : v4{};
		}
		bool Vector4(v4& vector)
		{
			return Real(vector.x) && Real(vector.y) && Real(vector.z) && Real(vector.w);
		}
		bool Vector4S(v4& vector)
		{
			return SectionStart() && Vector4(vector) && SectionEnd();
		}

		// Extract a 4D integer vector from the source.
		iv4 Vector4i(int radix = 10)
		{
			iv4 vector;
			return Vector4i(vector, radix) ? vector : iv4{};
		}
		iv4 Vector4iS(int radix = 10)
		{
			iv4 vector;
			return Vector4iS(vector, radix) ? vector : iv4{};
		}
		bool Vector4i(iv4& vector, int radix = 10)
		{
			return Int(vector.x, radix) && Int(vector.y, radix) && Int(vector.z, radix) && Int(vector.w, radix);
		}
		bool Vector4iS(iv4& vector, int radix = 10)
		{
			return SectionStart() && Vector4i(vector, radix) && SectionEnd();
		}

		// Extract a quaternion from the source.
		quat Quaternion()
		{
			quat quaternion;
			return Quaternion(quaternion) ? quaternion : quat{};
		}
		quat QuaternionS()
		{
			quat quaternion;
			return QuaternionS(quaternion) ? quaternion : quat{};
		}
		bool Quaternion(quat& quaternion)
		{
			return Real(quaternion.x) && Real(quaternion.y) && Real(quaternion.z) && Real(quaternion.w);
		}
		bool QuaternionS(quat& quaternion)
		{
			return SectionStart() && Quaternion(quaternion) && SectionEnd();
		}

		// Extract a 3x3 matrix from the source.
		m3x3 Matrix3x3()
		{
			m3x3 transform;
			return Matrix3x3(transform) ? transform : m3x3{};
		}
		m3x3 Matrix3x3S()
		{
			m3x3 transform;
			return Matrix3x3S(transform) ? transform : m3x3{};
		}
		bool Matrix3x3(m3x3& transform)
		{
			return Vector3(transform.x4, 0) && Vector3(transform.y4, 0) && Vector3(transform.z4, 0);
		}
		bool Matrix3x3S(m3x3& transform)
		{
			return SectionStart() && Matrix3x3(transform) && SectionEnd();
		}

		// Extract a 4x4 matrix from the source.
		m4x4 Matrix4x4()
		{
			m4x4 transform;
			return Matrix4x4(transform) ? transform : m4x4{};
		}
		m4x4 Matrix4x4S()
		{
			m4x4 transform;
			return Matrix4x4S(transform) ? transform : m4x4{};
		}
		bool Matrix4x4(m4x4& transform)
		{
			return Vector4(transform.x) && Vector4(transform.y) && Vector4(transform.z) && Vector4(transform.w);
		}
		bool Matrix4x4S(m4x4& transform)
		{
			return SectionStart() && Matrix4x4(transform) && SectionEnd();
		}

		// Extract a byte array from the source.
		std::vector<uint8_t> Data(size_t length, int radix = 16)
		{
			std::vector<uint8_t> data(length);
			return Data(data.data(), length, radix) ? std::move(data) : std::vector<uint8_t>{};
		}
		std::vector<uint8_t> DataS(size_t length, int radix = 16)
		{
			std::vector<uint8_t> data(length);
			return DataS(data.data(), length, radix) ? std::move(data) : std::vector<uint8_t>{};
		}
		bool Data(void* data, size_t length, int radix = 16)
		{
			return Int(static_cast<uint8_t*>(data), length, radix);
		}
		bool DataS(void* data, size_t length, int radix = 16)
		{
			return SectionStart() && Data(data, length, radix) && SectionEnd();
		}

		// Extract a rotation, pre-multiplying it onto 'rot'. 'rot' must already be a valid
		// (finite) rotation, since the result is 'read_rotation * rot', not a replacement.
		m3x3 Rotation()
		{
			auto rot = m3x3::Identity();
			return Rotation(rot) ? rot : m3x3::Identity();
		}
		m3x3 RotationS()
		{
			auto rot = m3x3::Identity();
			return RotationS(rot) ? rot : m3x3::Identity();
		}
		bool Rotation(m3x3& rot)
		{
			pr_assert(IsFinite(rot) && "A valid 'rot' must be passed to this function as it pre-multiplies the transform with the one read from the script");

			// Route through 'Transform', which accepts any 'ETransformKeyword', not just rotation ones.
			auto o2w = m4x4{rot, v4::Origin()};
			return Transform(o2w) ? (rot = o2w.rot, true) : false;
		}
		bool RotationS(m3x3& rot)
		{
			return SectionStart() && Rotation(rot) && SectionEnd();
		}

		// Extract a transform description accumulatively, pre-multiplying it onto 'o2w'. 'o2w' must
		// already be a valid (finite) transform, since the result is 'read_transform * o2w', not a
		// replacement; pass 'm4x4::Identity()' to build a transform from scratch.
		m4x4 Transform()
		{
			auto o2w = m4x4::Identity();
			return Transform(o2w) ? o2w : m4x4{};
		}
		m4x4 TransformS()
		{
			auto o2w = m4x4::Identity();
			return TransformS(o2w) ? o2w : m4x4{};
		}
		bool Transform(m4x4& o2w)
		{
			pr_assert(IsFinite(o2w) && "A valid 'o2w' must be passed to this function as it pre-multiplies the transform with the one read from the script");
			auto p2w = m4x4::Identity();
			auto affine = IsAffine(o2w);
			static std::default_random_engine rng;

			// Parse one transform keyword at a time, pre-multiplying 'p2w' by each, until a
			// non-transform keyword (or the source end) is reached.
			for (ETransformKeyword kw; NextKeywordH(kw);)
			{
				switch (kw)
				{
					case ETransformKeyword::NonAffine:
					{
						// A following 'M4x4' is allowed to be non-affine (e.g. contain a projection).
						affine = false;
						break;
					}
					case ETransformKeyword::M4x4:
					{
						auto m = m4x4::Identity();
						Matrix4x4S(m);
						if (affine && m.w.w != 1)
						{
							// Reporting is deferred to the switch's 'default' arm's error path; jump straight there.
							ReportError(EResult::UnknownValue, Location(), "Specify 'NonAffine' if M4x4 is intentionally non-affine.");
							goto transform_parse_done;
						}
						p2w = m * p2w;
						break;
					}
					case ETransformKeyword::M3x3:
					{
						auto m = m4x4::Identity();
						Matrix3x3S(m.rot);
						p2w = m * p2w;
						break;
					}
					case ETransformKeyword::Pos:
					{
						auto m = m4x4::Identity();
						Vector3S(m.pos, 1.0f);
						p2w = m * p2w;
						break;
					}
					case ETransformKeyword::Align:
					{
						// {axis_id, direction}: rotate the world axis 'axis_id' (\xc2\xb1""1/\xc2\xb1""2/\xc2\xb1""3, i.e. \xc2\xb1""X/\xc2\xb1""Y/\xc2\xb1""Z) to point along 'direction'.
						int axis_id;
						v4 direction;
						SectionStart();
						Int(axis_id, 10);
						Vector3(direction, 0.0f);
						SectionEnd();

						v4 axis = AxisId(axis_id);
						if (LengthSq(axis) == 0)
						{
							ReportError(EResult::UnknownValue, Location(), "axis_id must one of \xc2\xb1""1, \xc2\xb1""2, \xc2\xb1""3");
							goto transform_parse_done;
						}

						p2w = m4x4::Transform(axis, direction, v4::Origin()) * p2w;
						break;
					}
					case ETransformKeyword::Quat:
					{
						quat q;
						Vector4S(q.xyzw);
						p2w = m4x4::Transform(q, v4::Origin()) * p2w;
						break;
					}
					case ETransformKeyword::QuatPos:
					{
						v4 p;
						quat q;
						SectionStart();
						Vector4(q.xyzw);
						Vector3(p, 1.0f);
						SectionEnd();
						p2w = m4x4::Transform(q, p) * p2w;
						break;
					}
					case ETransformKeyword::Rand4x4:
					{
						// {centre, radius}: a uniformly random orientation and a random position within 'radius' of 'centre'.
						float radius;
						v4 centre;
						SectionStart();
						Vector3(centre, 1.0f);
						Real(radius);
						SectionEnd();
						auto rot = Random<m3x3>(rng);
						auto pos = Random<v4>(rng, centre, radius);
						p2w = m4x4{rot, pos} * p2w;
						break;
					}
					case ETransformKeyword::RandPos:
					{
						// {centre, radius}: a random position within 'radius' of 'centre'; orientation is unchanged.
						float radius;
						v4 centre;
						SectionStart();
						Vector3(centre, 1.0f);
						Real(radius);
						SectionEnd();
						p2w = m4x4::Translation(Random<v4>(rng, centre, radius).w1()) * p2w;
						break;
					}
					case ETransformKeyword::RandOri:
					{
						auto m = m4x4(Random<m3x3>(rng), v4::Origin());
						p2w = m * p2w;
						break;
					}
					case ETransformKeyword::Euler:
					{
						v4 angles;
						Vector3S(angles, 0.0f);
						p2w = m4x4::TransformDeg(angles.x, angles.y, angles.z, v4::Origin()) * p2w;
						break;
					}
					case ETransformKeyword::Scale:
					{
						// {s} scales uniformly; {sx, sy, sz} scales per-axis.
						v4 scale;
						SectionStart();
						Real(scale.x);
						if (IsSectionEnd())
						{
							scale.z = scale.y = scale.x;
						}
						else
						{
							Real(scale.y);
							Real(scale.z);
						}
						SectionEnd();
						p2w = m4x4::Scale(scale.x, scale.y, scale.z, v4::Origin()) * p2w;
						break;
					}
					case ETransformKeyword::Transpose:
					{
						p2w = Transpose(p2w);
						break;
					}
					case ETransformKeyword::Inverse:
					{
						p2w = IsOrthonormal(p2w) ? InvertOrthonormal(p2w) : Invert(p2w);
						break;
					}
					case ETransformKeyword::Normalise:
					{
						p2w.x = Normalise(p2w.x);
						p2w.y = Normalise(p2w.y);
						p2w.z = Normalise(p2w.z);
						break;
					}
					case ETransformKeyword::Orthonormalise:
					{
						p2w = Orthonorm(p2w);
						break;
					}
					default:
					{
						// An unrecognised transform keyword ends the accumulative parse; 'ReportError's
						// default handler throws, matching the "switch on enum -> default throws" convention,
						// while a caller-supplied handler that merely records the error can still recover here.
						ReportError(EResult::UnknownToken, Location(), std::string(m_last_keyword).append(" is not a valid Transform keyword"));
						goto transform_parse_done;
					}
				}
			}
			transform_parse_done:

			// Pre-multiply the object-to-world transform by the accumulated parsed transform.
			o2w = p2w * o2w;
			return true;
		}
		bool TransformS(m4x4& o2w)
		{
			return SectionStart() && Transform(o2w) && SectionEnd();
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

		// Allow extension methods, e.g: 'template <> bool Reader::Extract<MyType>(MyType& my_type) { return Int(my_type.field, 10); }'.
		template <typename Type> Type Extract()
		{
			Type type;
			return Extract(type) ? std::move(type) : Type{};
		}
		template <typename Type> Type ExtractS()
		{
			Type type;
			return ExtractS(type) ? std::move(type) : Type{};
		}
		template <typename Type> bool Extract(Type&)
		{
			static_assert(dependent_false<Type>, "Extract method not implemented for this type");
		}
		template <typename Type> bool ExtractS(Type& type)
		{
			// Parses 'type' via 'Extract' once inside the section (not via itself, which would recurse forever).
			return SectionStart() && Extract(type) && SectionEnd();
		}

		// Return the dot-delimited keyword "address" (e.g. "Group.Box.o2w.pos") for the position at
		// the end of 'utf8_up_to_cursor'. Unlike legacy's 'Reader::AddressAt(Src&)', which reads from
		// a shared, already-positioned 'Src', this takes the UTF-8 text truncated to the cursor
		// position (the caller's equivalent of legacy's 'Src::Limit()') and parses it with a private,
		// disposable reader. Returns an empty string if the truncated script doesn't parse cleanly.
		static std::string AddressAt(std::string_view utf8_up_to_cursor)
		{
			// The format of the returned address is: "keyword.keyword.keyword..."
			// e.g. for:
			//   *Group { *Width {1} *Smooth *Box
			//   {
			//       *other {}
			//       /* *something { */
			//       // *something {
			//       "my { string"
			//       *o2w { *pos { <-- address should be: Group.Box.o2w.pos

			// Use a case-sensitive reader so the reported address matches the source's case.
			Reader reader(utf8_up_to_cursor, true);

			std::string path, kw;
			try
			{
				for (; !reader.IsSourceEnd();)
				{
					// Find the next keyword in the current scope.
					if (reader.NextKeywordS(kw))
					{
						// A keyword followed by a section start extends the address while inside that section.
						if (reader.FindSectionStart())
						{
							path.append(path.empty() ? "" : ".").append(kw);
							reader.SectionStart();
						}
					}
					else if (reader.IsSectionEnd())
					{
						// Reaching the end of a scope means the cursor isn't within it; pop the last keyword.
						for (; !path.empty() && path.back() != '.'; path.pop_back()) {}
						if (!path.empty()) path.pop_back();
						reader.SectionEnd();
					}
				}
			}
			catch (std::exception const&)
			{
				// If the truncated script contains errors, the accumulated path can't be trusted.
				path.clear();
			}
			return path;
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
			// Filepath extraction preserves the source spelling when no include resolver is configured.
			Reader reader("\"foo/bar.txt\" \"unicode-\xC2\xB1.txt\"");
			PR_EXPECT(reader.Filepath() == PathFromUtf8("foo/bar.txt"));
			PR_EXPECT(reader.Filepath() == PathFromUtf8("unicode-\xC2\xB1.txt"));
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
		PRUnitTestMethod(EnumValueRadixParsing, Quick)
		{
			// Reader2-only: the legacy 'EnumValue' overload calls 'str::ExtractEnumValue'
			// with only 3 of its 4 required arguments, so it fails to compile if ever
			// instantiated - a latent bug in the untouched legacy header, not something to
			// replicate here. 'EnumValue' reads a raw integer at the given radix and blindly
			// casts it to the enum type, with no membership validation (unlike 'Enum', which
			// parses a member name via 'str::ExtractEnum').
			char const* script = "*Hex A\n*Bin 101\n*OutOfRange 999";

			Reader reader(script, false);
			char kw[64];

			PR_EXPECT(reader.NextKeywordS(kw));
			auto hex_val = reader.EnumValue<EResult>(16);
			PR_EXPECT(static_cast<int>(hex_val) == 0xA);

			PR_EXPECT(reader.NextKeywordS(kw));
			auto bin_val = reader.EnumValue<EResult>(2);
			PR_EXPECT(static_cast<int>(bin_val) == 5);

			PR_EXPECT(reader.NextKeywordS(kw));
			EResult out_of_range = EResult::Success;
			PR_EXPECT(reader.EnumValue(out_of_range, 10)); // succeeds even though 999 names no member.
			PR_EXPECT(static_cast<int>(out_of_range) == 999);
		}
		PRUnitTestMethod(AddressAtUtf8MultibyteBoundaries, Quick)
		{
			// Reader2-only: 'AddressAt' is a fresh, disposable-reader-based implementation
			// (see its declaration comment above) rather than a port of legacy's algorithm,
			// so exact byte-offset parity with legacy is only meaningful for pure-ASCII
			// scripts (covered by 'AddressAtDifferential' in 'reader2.h'). This test instead
			// checks reader2's own handling of a truncation landing mid multi-byte UTF-8
			// character.
			//
			// Byte layout (23 bytes total); '\xF0\x9F\x92\xA9' is one 4-byte UTF-8 codepoint
			// (U+1F4A9) occupying indices 16-19, with the closing quote at index 20:
			//   *Group { *Note "????" }
			//   0123456789
			char const* script = "*Group { *Note \"\xF0\x9F\x92\xA9\" }";

			PR_EXPECT(Reader::AddressAt(std::string_view(script, 9)) == "Group");  // before '*Note' is even scanned.
			PR_EXPECT(Reader::AddressAt(std::string_view(script, 21)) == "Group"); // literal parses cleanly; '*Note' isn't followed by '{'.
			PR_EXPECT(Reader::AddressAt(std::string_view(script, 23)) == "");      // 'Group's section closes, popping the path.
			PR_EXPECT(Reader::AddressAt(std::string_view(script, 18)) == "");      // truncated mid-codepoint: 'EatLiteral' throws, caught, path cleared.
		}
	};
}
#endif
