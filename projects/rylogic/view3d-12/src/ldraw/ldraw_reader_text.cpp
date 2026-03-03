//********************************
// Ldraw Script Text Serialiser
//  Copyright (c) Rylogic Ltd 2025
//********************************
#include "pr/view3d-12/ldraw/ldraw_reader_text.h"
#include "pr/script/script.h"

namespace pr::rdr12::ldraw
{
	// Convert opaque storage to a typed reference
	template <typename T, int N, typename B = std::conditional_t<std::is_const_v<T>, std::byte const, std::byte>>
	inline T& as(B (&storage)[N])
	{
		static_assert(sizeof(T) <= sizeof(storage));
		return reinterpret_cast<T&>(storage);
	}

	TextReader::TextReader(std::istream& stream, std::filesystem::path src_filepath, EEncoding enc, ReportErrorCB report_error_cb, ParseProgressCB progress_cb, IPathResolver const& resolver)
		: IReader(report_error_cb, progress_cb, resolver)
		, m_src()
		, m_pp()
		, m_location()
		, m_keyword()
		, m_delim(L" \t\r\n\v,;")
		, m_section_level()
		, m_nest_level()
	{
		static_assert(sizeof(m_src) >= sizeof(script::StreamSrc<char>));
		static_assert(sizeof(m_pp) >= sizeof(script::Preprocessor));
		new (m_src) script::StreamSrc<char>(stream, enc, script::Loc(src_filepath));
		new (m_pp) script::Preprocessor(as<script::Src>(m_src), nullptr, nullptr, nullptr);
	}
	TextReader::TextReader(std::wistream& stream, std::filesystem::path src_filepath, EEncoding enc, ReportErrorCB report_error_cb, ParseProgressCB progress_cb, IPathResolver const& resolver)
		: IReader(report_error_cb, progress_cb, resolver)
		, m_src()
		, m_pp()
		, m_location()
		, m_keyword()
		, m_delim(L" \t\r\n\v,;")
		, m_section_level()
		, m_nest_level()
	{
		static_assert(sizeof(m_src) >= sizeof(script::StreamSrc<wchar_t>));
		static_assert(sizeof(m_pp) >= sizeof(script::Preprocessor));
		new (m_src) script::StreamSrc<wchar_t>(stream, enc, script::Loc(src_filepath));
		new (m_pp) script::Preprocessor(as<script::Src>(m_src), nullptr, nullptr, nullptr);
	}
	TextReader::~TextReader()
	{
		as<script::Preprocessor>(m_pp).~Preprocessor();
		as<script::Src>(m_src).~Src();
	}

	// Return the current location in the source
	Location const& TextReader::Loc() const
	{
		auto loc = as<script::Preprocessor const>(m_pp).Location();
		m_location.m_filepath = loc.Filepath();
		m_location.m_filesize = loc.FileSize();
		m_location.m_offset = loc.Pos();
		m_location.m_column = loc.Col();
		m_location.m_line = loc.Line();
		return m_location;
	}

	// Move into a nested section
	void TextReader::PushSection()
	{
		auto& pp = as<script::Preprocessor>(m_pp);
		script::EatDelimiters(pp, m_delim.c_str());
		if (*pp != '{')
		{
			ReportError(EParseError::NotFound, Loc(), "section start expected");
			str::AdvanceToDelim(pp, m_delim.c_str());
			return;
		}

		++m_section_level;
		++m_nest_level;
		++pp;
	}

	// Leave the current nested section
	void TextReader::PopSection()
	{
		auto& pp = as<script::Preprocessor>(m_pp);
		script::EatDelimiters(pp, m_delim.c_str());
		if (*pp != '}')
		{
			ReportError(EParseError::NotFound, Loc(), "section end expected");
			str::AdvanceToDelim(pp, m_delim.c_str());
			return;
		}

		--m_section_level;
		--m_nest_level;
		++pp;
	}

	// True when the current position has reached the end of the current section
	bool TextReader::IsSectionEnd()
	{
		auto& pp = as<script::Preprocessor>(m_pp);
		m_nest_level += *pp == '{';
		pp += *pp == '{';
		script::EatDelimiters(pp, m_delim.c_str());
		return *pp == '}' || *pp == 0;
	}

	// True when the source is exhausted
	bool TextReader::IsSourceEnd()
	{
		auto& pp = as<script::Preprocessor>(m_pp);
		m_nest_level += *pp == '{';
		pp += *pp == '{';
		EatDelimiters(pp, m_delim.c_str());
		return *pp == 0;
	}

	// Get the next keyword within the current section.
	// Returns false if at the end of the section
	bool TextReader::NextKeywordImpl(int& kw)
	{
		auto& pp = as<script::Preprocessor>(m_pp);

		// Skip to the next keyword, but don't go beyond the current section level.
		for (;*pp && *pp != '*';)
		{
			if (*pp == '\"') { script::EatLiteral(pp, pp.Location()); continue; }
			if (*pp == '{')  { script::EatSection(pp, pp.Location()); continue; }
			if (*pp == '}')  { if (m_nest_level > m_section_level) --m_nest_level; else break; }
			++pp;
		}
		if (*pp == '*') ++pp; else return false;
			
		// Read the keyword
		wstring32 keyword;
		if (!str::ExtractIdentifier(keyword, pp, m_delim.c_str())) return false;

		// Convert the keyword to an integer
		m_keyword = Narrow(keyword);
		kw = HashI(m_keyword.c_str());

		// Handle an optional name and colour after the keyword
		wstring32 tokens[2]; int tcount = 0;
		script::EatDelimiters(pp, m_delim.c_str());
		if (*pp != '{') tcount += str::ExtractToken(tokens[0], pp, m_delim.c_str()) ? 1 : 0;
		script::EatDelimiters(pp, m_delim.c_str());
		if (*pp != '{') tcount += str::ExtractToken(tokens[1], pp, m_delim.c_str()) ? 1 : 0;
		script::EatDelimiters(pp, m_delim.c_str());
		if (*pp != '{')
		{
			ReportError(EParseError::UnexpectedToken, Loc(), "expected '{'");
			str::AdvanceToDelim(pp, m_delim.c_str());
			return {};
		}
		
		// Insert pseudo tokens for name and colour
		pp.Buffer(0, 1);
		for (; tcount--; )
		{
			auto& tok = tokens[tcount];

			// colour
			if (tok.size() == 8 && all(tok, std::iswxdigit))
				pp.Buffer().insert(1ULL, tok.insert(0, L"*Colour {").append(L"}"));
			
			// name
			else if (!tok.empty() && all(tok, [](auto ch) { return std::iswalnum(ch) || ch == '_'; }))
				pp.Buffer().insert(1ULL, tok.insert(0, L"*Name {").append(L"}"));
		}

		// At this stage we don't know if the following '{...}' is a data section or nested section.
		// Increment/decrement 'm_nest_level' whenever a '{' or '}' is consumed.
		// Increment/decrement 'm_section_level' whenever PushSection or PopSection is called.
		return true;
	}

	// Read an identifier from the current section. Leading '10xxxxxx' bytes are the length (in bytes). Default length is the full section
	string32 TextReader::IdentifierImpl(bool incl_dot)
	{
		auto& pp = as<script::Preprocessor>(m_pp);
		m_nest_level += *pp == '{';
		pp += *pp == '{';

		wstring32 str = {};
		if (!str::ExtractIdentifier(str, pp, m_delim.c_str(), incl_dot))
		{
			ReportError(EParseError::InvalidValue, Loc(), "identifier expected");
			str::AdvanceToDelim(pp, m_delim.c_str());
			return {};
		}
		return Narrow(str);
	}

	// Read a UTF-8 string from the current section. Leading '10xxxxxx' bytes are the length (in bytes). Default length is the full section
	string32 TextReader::StringImpl(char escape_char)
	{
		auto& pp = as<script::Preprocessor>(m_pp);
		m_nest_level += *pp == '{';
		pp += *pp == '{';

		wstring32 str = {};
		if (!str::ExtractString(str, pp, wchar_t(escape_char), nullptr, m_delim.c_str()))
		{
			ReportError(EParseError::InvalidValue, Loc(), "string expected");
			str::AdvanceToDelim(pp, m_delim.c_str());
			return {};
		}
		str::ProcessIndentedNewlines(str);
		return Narrow(str);
	}

	// Read an integral value from the current section
	int64_t TextReader::IntImpl(int, int radix)
	{
		auto& pp = as<script::Preprocessor>(m_pp);
		m_nest_level += *pp == '{';
		pp += *pp == '{';

		int64_t int_ = {};
		if (!str::ExtractInt(int_, radix, pp, m_delim.c_str()))
		{
			ReportError(EParseError::InvalidValue, Loc(), "integer value expected");
			str::AdvanceToDelim(pp, m_delim.c_str());
			return {};
		}
		return int_;
	}

	// Read a floating point value from the current section
	double TextReader::RealImpl(int)
	{
		auto& pp = as<script::Preprocessor>(m_pp);
		m_nest_level += *pp == '{';
		pp += *pp == '{';

		double real_ = {};
		if (!str::ExtractReal(real_, pp, m_delim.c_str()))
		{
			ReportError(EParseError::InvalidValue, Loc(), "real value expected");
			str::AdvanceToDelim(pp, m_delim.c_str());
			return {};
		}
		if (std::isnan(real_))
		{
			ReportError(EParseError::InvalidValue, Loc(), "real value is Not-a-Number");
			str::AdvanceToDelim(pp, m_delim.c_str());
			return {};
		}
		if (!std::isfinite(real_))
		{
			ReportError(EParseError::InvalidValue, Loc(), "real value is not finite");
			str::AdvanceToDelim(pp, m_delim.c_str());
			return {};
		}
		return real_;
	}

	// Read an enum value from the current section
	int64_t TextReader::EnumImpl(int, ParseEnumIdentCB parse)
	{
		auto& pp = as<script::Preprocessor>(m_pp);
		m_nest_level += *pp == '{';
		pp += *pp == '{';

		string32 ident = {};
		if (!str::ExtractIdentifier(ident, pp, m_delim.c_str()))
		{
			ReportError(EParseError::InvalidValue, Loc(), "enum identifier value expected");
			str::AdvanceToDelim(pp, m_delim.c_str());
			return {};
		}
		return parse(ident);
	}

	// Read a boolean value from the current section
	bool TextReader::BoolImpl()
	{
		auto& pp = as<script::Preprocessor>(m_pp);
		m_nest_level += *pp == '{';
		pp += *pp == '{';

		bool bool_ = {};
		if (!str::ExtractBool(bool_, pp, m_delim.c_str()))
		{
			ReportError(EParseError::InvalidValue, Loc(), "boolean value expected");
			str::AdvanceToDelim(pp, m_delim.c_str());
			return {};
		}
		return bool_;
	}
}