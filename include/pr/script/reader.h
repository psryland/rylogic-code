//**********************************
// Script
//  Copyright (c) Rylogic Ltd 2015
//**********************************
#pragma once
#include "pr/math/math.h"
#include "pr/script/forward.h"
#include "pr/script/script_core.h"
#include "pr/script/filter.h"
#include "pr/script/preprocessor.h"
#include "pr/script/fail_policy.h"

namespace pr::script
{
	struct Reader
	{
		// Notes:
		//  - The extract functions come in four forms:
		//      Thing Thing();
		//      Thing ThingS();
		//      bool Thing(Thing&);
		//      bool ThingS(Thing&);
		//    All functions read one instance of type 'Thing' from the script.
		//    The postfix 'S' means 'within a section'. e.g. { "value" }
		//    If an error occurs in the first two forms, ReportError is called. If ReportError doesn't throw, then a default instance is returned.
		//    If an error occurs in the second two forms,  ReportError is called. If ReportError doesn't throw, then false is returned.

	private:
		Preprocessor m_pp;
		string_t m_delim;
		string_t m_last_keyword;
		bool m_case_sensitive;

	public:

		Reader(bool case_sensitive = false, IIncludeHandler* inc = nullptr, EmbeddedCodeFactory emb = nullptr) noexcept
			:m_pp(inc, emb)
			,m_delim(L" \t\r\n\v,;")
			,m_last_keyword()
			,m_case_sensitive(case_sensitive)
			,ReportError(DefaultErrorHandler)
		{}
		Reader(Src& src, bool case_sensitive = false, IIncludeHandler* inc = nullptr, EmbeddedCodeFactory emb = nullptr)
			:m_pp(src, inc, emb)
			,m_delim(L" \t\r\n\v,;")
			,m_last_keyword()
			,m_case_sensitive(case_sensitive)
			,ReportError(DefaultErrorHandler)
		{}
		Reader(Src* src, bool delete_on_pop, bool case_sensitive = false, IIncludeHandler* inc = nullptr, EmbeddedCodeFactory emb = nullptr)
			:m_pp(src, delete_on_pop, inc, emb)
			,m_delim(L" \t\r\n\v,;")
			,m_last_keyword()
			,m_case_sensitive(case_sensitive)
			,ReportError(DefaultErrorHandler)
		{}
		Reader(char const* src, bool case_sensitive = false, IIncludeHandler* inc = nullptr, EmbeddedCodeFactory emb = nullptr)
			:m_pp(src, inc, emb)
			,m_delim(L" \t\r\n\v,;")
			,m_last_keyword()
			,m_case_sensitive(case_sensitive)
			,ReportError(DefaultErrorHandler)
		{}
		Reader(wchar_t const* src, bool case_sensitive = false, IIncludeHandler* inc = nullptr, EmbeddedCodeFactory emb = nullptr)
			:m_pp(src, inc, emb)
			,m_delim(L" \t\r\n\v,;")
			,m_last_keyword()
			,m_case_sensitive(case_sensitive)
			,ReportError(DefaultErrorHandler)
		{}

		// Allow override of error handling
		std::function<bool(EResult, Loc const&, std::string_view)> ReportError;
		static bool DefaultErrorHandler(EResult result, Loc const& loc, std::string_view msg)
		{
			throw ScriptException(result, loc, msg);
		}

		// Access the underlying source
		Src const& Source() const noexcept
		{
			return m_pp;
		}
		Src& Source() noexcept
		{
			return m_pp;
		}

		// Return the current source location
		Loc Location() const noexcept
		{
			return m_pp.Location();
		}

		// Access the include handler
		IIncludeHandler& Includes() const noexcept
		{
			return *m_pp.Includes;
		}

		// Access the macro handler
		IMacroHandler& Macros() const noexcept
		{
			return *m_pp.Macros;
		}

		// Get/Set delimiter characters
		wchar_t const* Delimiters() const noexcept
		{
			return m_delim.c_str();
		}
		void Delimiters(wchar_t const* delim) noexcept
		{
			m_delim = delim;
		}

		// Get/Set case sensitive keywords on/off
		bool CaseSensitive() const noexcept
		{
			return m_case_sensitive;
		}
		void CaseSensitive(bool cs) noexcept
		{
			m_case_sensitive = cs;
		}

		// Return the hash of a keyword using the current reader settings
		int HashKeyword(wchar_t const* keyword) const
		{
			return script::HashKeyword(keyword, m_case_sensitive);
		}

		// Return true if the end of the source has been reached
		bool IsSourceEnd()
		{
			auto& src = m_pp;
			EatDelimiters(src, m_delim.c_str());
			return *src == 0;
		}

		// Return true if the next token is a keyword
		bool IsKeyword()
		{
			auto& src = m_pp;
			EatDelimiters(src, m_delim.c_str());
			return *src == '*';
		}

		// Returns true if the next non-whitespace character is the start/end of a section
		bool IsSectionStart()
		{
			auto& src = m_pp;
			EatDelimiters(src, m_delim.c_str());
			return *src == L'{';
		}
		bool IsSectionEnd()
		{
			auto& src = m_pp;
			EatDelimiters(src, m_delim.c_str());
			return *src == L'}';
		}

		// Return true if the next token is not a keyword, the section end, or the end of the source
		bool IsValue()
		{
			return !IsKeyword() && !IsSectionEnd() && !IsSourceEnd();
		}

		// Buffer the next 'n' characters and test them against the given pattern.
		bool IsMatch(int n, std::wregex pattern)
		{
			auto& src = m_pp;
			EatDelimiters(src, m_delim.c_str());
			auto const len = src.ReadAhead(n);
			return std::regex_match(string_t(src.Buffer(0, len)), pattern);
		}

		// Move to the start/end of a section and then one past it
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

		// Move to the start of the next line
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
			for (;*src && *src != '{' && *src != '}' && *src != '*';)
			{
				if (*src == '\"')
				{
					EatLiteral(src);
					continue;
				}
				++src;
			}
			return *src == '{';
		}

		// Advance the source to the end of the current section
		// On return the current position should be the section end character
		// or the end of the input stream (if called from file scope).
		bool FindSectionEnd()
		{
			auto& src = m_pp;
			for (int nest = IsSectionStart() ? 0 : 1; *src;)
			{
				if (*src == L'\"') { EatLiteral(src); continue; }
				nest += (*src == L'{') ? 1 : 0;
				nest -= (*src == L'}') ? 1 : 0;
				if (nest == 0) break;
				++src;
			}
			return *src == L'}';
		}

		// Scans forward until a keyword identifier is found within the current scope.
		// Non-keyword tokens are skipped. If a section is found it is skipped.
		// If a keyword is found, the source is positioned at the next character after the keyword.
		// Returns true if a keyword is found, false otherwise.
		template <typename StrType> bool NextKeywordS(StrType& kw)
		{
			// Note:
			//  Typical control flow:
			//      reader.SectionStart();
			//      for (;!reader.IsSectionEnd();)
			//      {
			//      	if (!reader.IsKeyword())
			//      	{
			//      		reader.StringC(...);
			//      	}
			//      	else if (char kw[20]; reader.NextKeyword(kw))
			//      	{
			//      		if (str::EqualI(kw, "Option1"))
			//      		{
			//      			//...
			//      			continue;
			//      		}
			//      	}
			//      }
			//      reader.SectionEnd();

			auto& src = m_pp;
			for (;*src && *src != L'}' && *src != L'*';)
			{
				if (*src == L'\"') { EatLiteral(src); continue; }
				if (*src == L'{')  { src += FindSectionEnd(); continue; }
				++src;
			}
			if (*src == L'*') ++src; else return false;
			str::Resize(kw, 0);
			if (!str::ExtractIdentifier(kw, src, m_delim.c_str())) return false;
			if (!m_case_sensitive) str::LowerCase(kw);
			m_last_keyword = pr::Widen(kw);
			return true;
		}

		// As above except the hash of the keyword is returned instead (converted to an enum value)
		template <typename Enum> bool NextKeywordH(Enum& enum_kw)
		{
			string_t kw;
			if (!NextKeywordS(kw)) return false;
			m_last_keyword = kw;
			enum_kw = Enum(HashKeyword(kw.c_str()));
			return true;
		}

		// As above except an error is reported if the next token is not a keyword
		int NextKeywordH()
		{
			int kw = 0;
			if (!NextKeywordH(kw)) ReportError(EResult::TokenNotFound, Location(), "keyword expected");
			return kw;
		}
		template <typename Enum> Enum NextKeywordH()
		{
			return Enum(NextKeywordH());
		}

		// Return the last keyword reader from the stream
		string_t LastKeyword() const
		{
			return m_last_keyword;
		}

		// Scans forward until a keyword matching 'named_kw' is found within the current scope.
		// Returns false if the named keyword is not found, true if it is
		bool FindKeyword(wchar_t const* named_kw)
		{
			int kw_hashed = 0;
			auto const named_kw_hashed = HashKeyword(named_kw);
			for (; NextKeywordH(kw_hashed) && kw_hashed != named_kw_hashed; ) {}
			return named_kw_hashed == kw_hashed;
		}

		// Scans forward until a keyword matching 'named_kw' is found within the current scope.
		// Calls ReportError if not found.
		Reader& Keyword(wchar_t const* named_kw)
		{
			FindKeyword(named_kw) || ReportError(EResult::KeywordNotFound, Location(), FmtS("keyword '%S' expected", named_kw));
			return *this;
		}

		// Extract a token from the source.
		// A token is a contiguous block of non-separator characters
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
			return str::ExtractToken(token, src, m_delim.c_str()) || ReportError(EResult::TokenNotFound, Location(), "token expected");
		}
		template <typename StrType> bool TokenS(StrType& token)
		{
			return SectionStart() && Token(token) && SectionEnd();
		}

		// Extract a token using additional delimiters
		template <typename StrType> StrType Token(wchar_t const* delim)
		{
			StrType token;
			return Token(token, delim) ? token : StrType();
		}
		template <typename StrType> StrType TokenS(wchar_t const* delim)
		{
			StrType token;
			return TokenS(token, delim) ? token : StrType{};
		}
		template <typename StrType> bool Token(StrType& token, wchar_t const* delim)
		{
			auto& src = m_pp;
			str::Resize(token, 0);
			return str::ExtractToken(token, src, string_t(m_delim).append(delim).c_str()) || ReportError(EResult::TokenNotFound, Location(), "token expected");
		}
		template <typename StrType> bool TokenS(StrType& token, wchar_t const* delim)
		{
			return SectionStart() && Token(token, delim) && SectionEnd();
		}

		// Read an identifier from the source. An identifier is one of (A-Z,a-z,'_') followed by (A-Z,a-z,'_',0-9) in a contiguous block
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
			return str::ExtractIdentifier(word, src, m_delim.c_str()) || ReportError(EResult::TokenNotFound, Location(), "{identifier} expected");
		}
		template <typename StrType> bool IdentifierS(StrType& word)
		{
			return SectionStart() && Identifier(word) && SectionEnd();
		}

		// Extract identifiers from the source separated by 'sep'
		template <typename StrType> bool Identifiers(char, StrType& word)
		{
			auto& src = m_pp;
			str::Resize(word, 0);
			return str::ExtractIdentifier(word, src, m_delim.c_str()) || ReportError(EResult::TokenNotFound, Location(), "identifier expected");
		}
		template <typename StrType, typename... StrTypes> bool Identifiers(char sep, StrType& word, StrTypes&&... words)
		{
			auto& src = m_pp;
			str::Resize(word, 0);
			if (!str::ExtractIdentifier(word, src, m_delim.c_str())) return ReportError(EResult::TokenNotFound, Location(), "identifier expected");
			if (*src == sep) ++src; else return ReportError(EResult::TokenNotFound, Location(), "identifier separator expected");
			return Identifiers(sep, std::forward<StrTypes>(words)...);
		}
		template <typename StrType, typename... StrTypes> bool IdentifiersS(char sep, StrType& word, StrTypes&&... words)
		{
			return SectionStart() && Identifiers(sep,word,std::forward<StrTypes>(words)...) && SectionEnd();
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
			if (str::ExtractString<StrType>(string, src, m_delim.c_str()))
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
			return str::ExtractString<StrType>(cstring, src, L'\\', nullptr, m_delim.c_str()) || ReportError(EResult::TokenNotFound, Location(), "'cstring' expected");
		}
		template <typename StrType> bool CStringS(StrType& cstring)
		{
			return SectionStart() && CString(cstring) && SectionEnd();
		}

		// Extract a filepath string from the source
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
			std::wstring s;
			if (!String(s)) return ReportError(EResult::InvalidString, Location(), "'filepath' string expected");

			// Apply path resolution to the filepath
			path = s;
			if (m_pp.Includes != nullptr)
				path = m_pp.Includes->ResolveInclude(path, EIncludeFlags::IncludeLocalDir | EIncludeFlags::IgnoreMissing);

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
			return str::ExtractBool(bool_, src, m_delim.c_str()) || ReportError(EResult::TokenNotFound, Location(), "bool expected");
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
			return str::ExtractInt(int_, radix, src, m_delim.c_str()) || ReportError(EResult::TokenNotFound, Location(), "integral value expected");
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
			return str::ExtractReal(real_, src, m_delim.c_str()) || ReportError(EResult::TokenNotFound, Location(), "real expected");
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

		// Extract an enum value from the source.
		template <typename TEnum> TEnum EnumValue()
		{
			TEnum enum_;
			return EnumValue(enum_) ? enum_ : TEnum{};
		}
		template <typename TEnum> TEnum EnumValueS()
		{
			TEnum enum_;
			return EnumValueS(enum_) ? enum_ : TEnum{};
		}
		template <typename TEnum> bool EnumValue(TEnum& enum_)
		{
			auto& src = m_pp;
			return str::ExtractEnumValue(enum_, src, m_delim.c_str()) || ReportError(EResult::TokenNotFound, Location(), "enum integral value expected");
		}
		template <typename TEnum> bool EnumValueS(TEnum& enum_)
		{
			return SectionStart() && EnumValue(enum_) && SectionEnd();
		}

		// Extract an enum identifier from the source.
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
			return str::ExtractEnum(enum_, src, m_delim.c_str()) || ReportError(EResult::TokenNotFound, Location(), "enum member string name expected");
		}
		template <typename TEnum> bool EnumS(TEnum& enum_)
		{
			return SectionStart() && Enum(enum_) && SectionEnd();
		}

		// Extract a 2D real vector from the source
		v2 Vector2()
		{
			v2 vector;
			return Vector2(vector) ? vector : v2{};
		}
		v2 Vector2S()
		{
			v2 vector;
			return Vector2S(vector) ? vector : v2{};
		}
		bool Vector2(v2& vector)
		{
			return Real(vector.x) && Real(vector.y);
		}
		bool Vector2S(v2& vector)
		{
			return SectionStart() && Vector2(vector) && SectionEnd();
		}

		// Extract a 2D integer vector from the source
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

		// Extract a 3D real vector from the source
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

		// Extract a 3D integer vector from the source
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

		// Extract a 4D real vector from the source
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

		// Extract a 4D integer vector from the source
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

		// Extract a quaternion from the source
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

		// Extract a 3x3 matrix from the source
		m3x4 Matrix3x3()
		{
			m3x4 transform;
			return Matrix3x3(transform) ? transform : m3x4{};
		}
		m3x4 Matrix3x3S()
		{
			m3x4 transform;
			return Matrix3x3S(transform) ? transform : m3x4{};
		}
		bool Matrix3x3(m3x4& transform)
		{
			return Vector3(transform.x, 0) && Vector3(transform.y, 0) && Vector3(transform.z, 0);
		}
		bool Matrix3x3S(m3x4& transform)
		{
			return SectionStart() && Matrix3x3(transform) && SectionEnd();
		}

		// Extract a 4x4 matrix from the source
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

		// Extract a byte array
		vector<uint8_t> Data(size_t length, int radix = 16)
		{
			vector<uint8_t> data(length);
			return Data(data.data(), length, radix) ? std::move(data) : vector<uint8_t>{};
		}
		vector<uint8_t> DataS(size_t length, int radix = 16)
		{
			vector<uint8_t> data(length);
			return DataS(data.data(), length, radix) ? std::move(data) : vector<uint8_t>{};
		}
		bool Data(void* data, size_t length, int radix = 16)
		{
			return Int(static_cast<uint8_t*>(data), length, radix);
		}
		bool DataS(void* data, size_t length, int radix = 16)
		{
			return SectionStart() && Data(data, length, radix) && SectionEnd();
		}

		// Extract a transform description. 'rot' should be a valid initial transform
		m3x4 Rotation()
		{
			m3x4 rot = m3x4::Identity();
			return Rotation(rot) ? rot : m3x4::Identity();
		}
		m3x4 RotationS()
		{
			m3x4 rot = m3x4::Identity();
			return RotationS(rot) ? rot : m3x4::Identity();
		}
		bool Rotation(m3x4& rot)
		{
			assert(IsFinite(rot) && "A valid 'rot' must be passed to this function as it pre-multiplies the transform with the one read from the script");
			auto o2w = m4x4{ rot, v4::Origin() };
			return Transform(o2w) ? (rot = o2w.rot, true) : false;
		}
		bool RotationS(m3x4& o2w)
		{
			return SectionStart() && Rotation(o2w) && SectionEnd();
		}

		// Extract a transform description accumulatively. 'o2w' should be a valid initial transform.
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
			assert(IsFinite(o2w) && "A valid 'o2w' must be passed to this function as it pre-multiplies the transform with the one read from the script");
			auto p2w = m4x4::Identity();
			auto affine = IsAffine(o2w);
			static std::default_random_engine rng;

			// Parse the transform
			for (ETransformKeyword kw; NextKeywordH(kw);)
			{
				if (kw == ETransformKeyword::NonAffine)
				{
					affine = false;
					continue;
				}
				if (kw == ETransformKeyword::M4x4)
				{
					auto m = m4x4::Identity();
					Matrix4x4S(m);
					if (affine && m.w.w != 1)
					{
						ReportError(EResult::UnknownValue, Location(), "Specify 'NonAffine' if M4x4 is intentionally non-affine.");
						break;
					}
					p2w = m * p2w;
					continue;
				}
				if (kw == ETransformKeyword::M3x3)
				{
					auto m = m4x4::Identity();
					Matrix3x3S(m.rot);
					p2w = m * p2w;
					continue;
				}
				if (kw == ETransformKeyword::Pos)
				{
					auto m = m4x4::Identity();
					Vector3S(m.pos, 1.0f);
					p2w = m * p2w;
					continue;
				}
				if (kw == ETransformKeyword::Align)
				{
					int axis_id;
					v4 direction;
					SectionStart();
					Int(axis_id, 10);
					Vector3(direction, 0.0f);
					SectionEnd();

					v4 axis = AxisId(axis_id);
					if (axis == v4::Zero())
					{
						ReportError(EResult::UnknownValue, Location(), "axis_id must one of \xc2\xb1""1, \xc2\xb1""2, \xc2\xb1""3");
						break;
					}

					p2w = m4x4::Transform(axis, direction, v4::Origin()) * p2w;
					continue;
				}
				if (kw == ETransformKeyword::Quat)
				{
					quat q;
					Vector4S(q.xyzw);
					p2w = m4x4::Transform(q, v4::Origin()) * p2w;
					continue;
				}
				if (kw == ETransformKeyword::QuatPos)
				{
					v4 p;
					quat q;
					SectionStart();
					Vector4(q.xyzw);
					Vector3(p, 1.0f);
					SectionEnd();
					p2w = m4x4::Transform(q, p) * p2w;
					continue;
				}
				if (kw == ETransformKeyword::Rand4x4)
				{
					float radius;
					v4 centre;
					SectionStart();
					Vector3(centre, 1.0f);
					Real(radius);
					SectionEnd();
					auto rot = Random<m3x4>(rng);
					auto pos = Random<v4>(rng, centre, radius);
					p2w = m4x4{rot, pos} * p2w;
					continue;
				}
				if (kw == ETransformKeyword::RandPos)
				{
					float radius;
					v4 centre;
					SectionStart();
					Vector3(centre, 1.0f);
					Real(radius);
					SectionEnd();
					p2w = m4x4::Translation(Random<v4>(rng, centre, radius).w1()) * p2w;
					continue;
				}
				if (kw == ETransformKeyword::RandOri)
				{
					auto m = m4x4(Random<m3x4>(rng), v4::Origin());
					p2w = m * p2w;
					continue;
				}
				if (kw == ETransformKeyword::Euler)
				{
					v4 angles;
					Vector3S(angles, 0.0f);
					p2w = m4x4::TransformDeg(angles.x, angles.y, angles.z, v4::Origin()) * p2w;
					continue;
				}
				if (kw == ETransformKeyword::Scale)
				{
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
					continue;
				}
				if (kw == ETransformKeyword::Transpose)
				{
					p2w = Transpose(p2w);
					continue;
				}
				if (kw == ETransformKeyword::Inverse)
				{
					p2w = IsOrthonormal(p2w) ? InvertAffine(p2w) : Invert(p2w);
					continue;
				}
				if (kw == ETransformKeyword::Normalise)
				{
					p2w.x = Normalise(p2w.x);
					p2w.y = Normalise(p2w.y);
					p2w.z = Normalise(p2w.z);
					continue;
				}
				if (kw == ETransformKeyword::Orthonormalise)
				{
					p2w = Orthonorm(p2w);
					continue;
				}
				ReportError(script::EResult::UnknownToken, Location(), Fmt("%S is not a valid Transform keyword", m_last_keyword.c_str()));
				break;
			}

			// Pre-multiply the object to world transform
			o2w = p2w * o2w;
			return true;
		}
		bool TransformS(m4x4& o2w)
		{
			return SectionStart() && Transform(o2w) && SectionEnd();
		}

		// Extract a complete section as a preprocessed string. Note: To embed arbitrary text in a script use #lit/#end and then Section()
		template <typename String> String Section(bool include_braces)
		{
			String s;
			return Section(s, include_braces) ? s : String{};
		}
		template <typename String> bool Section(String& str, bool include_braces)
		{
			// Do not str.resize(0) here, that's the callers decision
			auto& src = m_pp;
			InLiteral lit;
			if (IsSectionStart()) ++src; else return ReportError(EResult::TokenNotFound, Location(), "expected '{'");
			if (include_braces) str::Append(str, L'{');
			for (int nest = 1; *src; ++src)
			{
				// If we're in a string/character literal, then ignore any '{''}' characters
				if (lit.WithinLiteral(*src)) { str::Append(str, *src); continue; }
				nest += int(*src == '{');
				nest -= int(*src == '}');
				if (nest == 0) break;
				str::Append(str, *src);
			}
			if (include_braces) str::Append(str, '}');
			if (IsSectionEnd()) ++src; else return ReportError(EResult::TokenNotFound, Location(), "expected '}'");
			return true;
		}

		// Allow extension methods. e.g: template <> bool script::Reader::Extract<MyType>(MyType& my_type) { return Int(my_type.int, 10); // etc }
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
		template <typename Type> bool Extract(Type& type)
		{
			static_assert(dependent_false<Type>, "Extract method not implemented for this type");
		}
		template <typename Type> bool ExtractS(Type& type)
		{
			return SectionStart() && ExtractS(type) && SectionEnd();
		}

		// Return the hierarchy "address" for the position at the end of 'src'.
		static string_t AddressAt(Src& src)
		{
			// The format of the returned address is: "keyword.keyword.keyword..."
			// e.g. example
			//   *Group { *Width {1} *Smooth *Box
			//   {
			//       *other {}
			//       /* *something { */
			//       // *something {
			//       "my { string"
			//       *o2w { *pos { <-- Address should be: Group.Box.o2w.pos
			// Notes:
			//  - Use Src::Limit() to set the maximum number of characters to read from 'src'.
			//    This allows for UTF8 rather than the underlying available data.

			// Use a case sensitive reader so that the reported address matches the case.
			NoIncludes inc;
			Reader reader(src, true, &inc);

			string_t path, kw;
			try
			{
				for (; !reader.IsSourceEnd(); )
				{
					// Find the next keyword in the current scope
					if (reader.NextKeywordS(kw))
					{
						// Look for a section start
						if (reader.FindSectionStart())
						{
							// Add to the path while within this section
							path.append(path.empty() ? L"" : L".").append(kw);
							reader.SectionStart();
						}
					}
					else
					{
						// If we've reached the end of the scope, pop that last keyword
						// from the path since 'position' is not within this scope.
						if (reader.IsSectionEnd())
						{
							for (; !path.empty() && path.back() != '.'; path.pop_back()) {}
							if (!path.empty()) path.pop_back();
							reader.SectionEnd();
						}
					}
				}
			}
			catch (std::exception const&)
			{
				// If the script contains errors, we can't be sure that 'path' is correct.
				// Return an empty path, rather than hoping that the path is right.
				path.resize(0);
			}
			return path;
		}
	};
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::script
{
	PRUnitTestClass(ReaderTests)
	{
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
			"*Vector3 1.0 2.0 3.0\n"
			"*Vector4 4.0 3.0 2.0 1.0\n"
			"*Quaternion 0.0 -1.0 -2.0 -3.0\n"
			"*M3x3 1.0 0.0 0.0  0.0 1.0 0.0  0.0 0.0 1.0\n"
			"*M4x4 1.0 0.0 0.0 0.0  0.0 1.0 0.0 0.0  0.0 0.0 1.0 0.0  0.0 0.0 0.0 1.0\n"
			"*Data 41 42 43 44 45 46 47 48 49 4A 4B 4C 4D 4E 4F 00\n"
			"*Junk\n"
			"*Section {*SubSection { *Data \n NUM \"With a }\\\"string\\\"{ in it\" }}    \n"
			"*Section {*SubSection { *Data \n NUM \"With a }\\\"string\\\"{ in it\" }}    \n"
			"*Token 123token\n"
			"*LastThing";

		PRUnitTestMethod(BasicExtractMethods)
		{
			char kw[50];
			int hashed_kw = 0;
			std::string str;
			bool bval = false, barray[4];
			int ival = 0, iarray[4];
			unsigned int uival = 0;
			float fval = 0.0f, farray[4];
			pr::v4 vec = pr::v4::Zero();
			pr::quat q = pr::quat::Identity();
			pr::m3x4 mat3;
			pr::m4x4 mat4;

			Reader reader(Script, true);
			PR_EXPECT(reader.CaseSensitive());
			PR_EXPECT(reader.NextKeywordS(kw) && std::string(kw) == "Identifier");
			PR_EXPECT(reader.Identifier(str) && str == "ident");
			PR_EXPECT(reader.NextKeywordS(kw) && std::string(kw) == "String" );
			PR_EXPECT(reader.String(str) && str == "simple string");
			PR_EXPECT(reader.NextKeywordH(hashed_kw) && hashed_kw == reader.HashKeyword(L"CString"));
			PR_EXPECT(reader.CString(str) && str == "C:\\Path\\Filename.txt");
			PR_EXPECT(reader.NextKeywordS(kw) && std::string(kw) == "Bool");
			PR_EXPECT(reader.Bool(bval) && bval == true);
			PR_EXPECT(reader.NextKeywordS(kw) && std::string(kw) == "Intg");
			PR_EXPECT(reader.Int(ival, 10) && ival == -23);
			PR_EXPECT(reader.NextKeywordS(kw) && std::string(kw) == "Intg16" );
			PR_EXPECT(reader.Int(uival, 16) && uival == 0xABCDEF00);
			PR_EXPECT(reader.NextKeywordS(kw) && std::string(kw) == "Real" );
			PR_EXPECT(reader.Real(fval) && fval == -2.3e+3);
			PR_EXPECT(reader.NextKeywordS(kw) && std::string(kw) == "BoolArray" );
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
			PR_EXPECT(reader.NextKeywordS(kw) && std::string(kw) == "Vector3");
			PR_EXPECT(reader.Vector3(vec,-1.0f) && FEql(vec, v4(1.0f, 2.0f, 3.0f,-1.0f)));
			PR_EXPECT(reader.NextKeywordS(kw) && std::string(kw) == "Vector4");
			PR_EXPECT(reader.Vector4(vec) && FEql(vec, v4(4.0f, 3.0f, 2.0f, 1.0f)));
			PR_EXPECT(reader.NextKeywordS(kw) && std::string(kw) == "Quaternion");
			PR_EXPECT(reader.Quaternion(q) && FEql(q, quat(0.0f, -1.0f, -2.0f, -3.0f)));
			PR_EXPECT(reader.NextKeywordS(kw) && std::string(kw) == "M3x3");
			PR_EXPECT(reader.Matrix3x3(mat3) && FEql(mat3, m3x4::Identity()));
			PR_EXPECT(reader.NextKeywordS(kw) && std::string(kw) == "M4x4");
			PR_EXPECT(reader.Matrix4x4(mat4) && FEql(mat4, m4x4::Identity()));
			PR_EXPECT(reader.NextKeywordS(kw) && std::string(kw) == "Data");
			PR_EXPECT(reader.Data(kw, 16) && std::string(kw) == "ABCDEFGHIJKLMNO");
			PR_EXPECT(reader.FindKeyword(L"Section")); str.resize(0);
			PR_EXPECT(reader.Section(str, false) && str == "*SubSection { *Data \n 23 \"With a }\\\"string\\\"{ in it\" }");
			PR_EXPECT(reader.FindKeyword(L"Section")); str.resize(0);
			PR_EXPECT(reader.NextKeywordS(kw) && std::string(kw) == "Token");
			PR_EXPECT(reader.Token(str) && str == "123token");
			PR_EXPECT(reader.NextKeywordS(kw) && std::string(kw) == "LastThing");
			PR_EXPECT(reader.IsKeyword() == false);
			PR_EXPECT(reader.IsSectionStart() == false);
			PR_EXPECT(reader.IsSectionEnd() == false);
			PR_EXPECT(reader.IsSourceEnd());
		}
		PRUnitTestMethod(DotDelimitedIdentifiers)
		{
			char const* s =
				"A.B\n"
				"a.b.c\n"
				"A.B.C.D\n"
				;
			std::string s0,s1,s2,s3;

			Reader reader(s);
			reader.Identifiers('.',s0,s1);        PR_EXPECT(s0 == "A" && s1 == "B");
			reader.Identifiers('.',s0,s1,s2);     PR_EXPECT(s0 == "a" && s1 == "b" && s2 == "c");
			reader.Identifiers('.',s0,s1,s2,s3);  PR_EXPECT(s0 == "A" && s1 == "B" && s2 == "C" && s3 == "D");
		}
		PRUnitTestMethod(AddressAt)
		{
			wchar_t const* str0 = L""
				L"*Group { *Width {1} *Smooth *Box\n" //33
				L"{\n" //35
				L"	*other {}\n" // 46
				L"	/* *something { */\n" //66
				L"	// *something {\n" //83
				L"	\"my { string\"\n" //98
				L"	*o2w { *pos {"; // 112
			{
				StringSrc src({ str0, 0 });
				PR_EXPECT(str::Equal(Reader::AddressAt(src), ""));
			}
			{
				StringSrc src({ str0, 18 });
				PR_EXPECT(str::Equal(Reader::AddressAt(src), "Group.Width"));
			}
			{
				StringSrc src({ str0, 19 });
				PR_EXPECT(str::Equal(Reader::AddressAt(src), "Group"));
			}
			{
				StringSrc src({ str0, 35 });
				PR_EXPECT(str::Equal(Reader::AddressAt(src), "Group.Box"));
			}
			{
				StringSrc src({ str0, 88 });
				PR_EXPECT(str::Equal(Reader::AddressAt(src), "")); // because partway through a literal string
			}
			{
				StringSrc src(str0);
				PR_EXPECT(str::Equal(Reader::AddressAt(src), "Group.Box.o2w.pos"));
			}

			auto const u8str1 = u8"*One { \"💩🍌\" \"💩🍌\" }";
			char const* str1 = reinterpret_cast<char const*>(&u8str1[0]);
			{
				StringSrc src(str1); src.Limit(6);
				PR_EXPECT(str::Equal(Reader::AddressAt(src), "One"));
			}
			{
				StringSrc src(str1); src.Limit(9);
				PR_EXPECT(str::Equal(Reader::AddressAt(src), ""));
			}
			{
				StringSrc src(str1); src.Limit(11);
				PR_EXPECT(str::Equal(Reader::AddressAt(src), "One"));
			}
			{
				StringSrc src(str1); src.Limit(14);
				PR_EXPECT(str::Equal(Reader::AddressAt(src), ""));
			}
			{
				StringSrc src(str1); src.Limit(16);
				PR_EXPECT(str::Equal(Reader::AddressAt(src), "One"));
			}
		}
	};
}
#endif