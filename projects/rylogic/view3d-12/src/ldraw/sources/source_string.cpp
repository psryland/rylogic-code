//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#include "view3d-12/src/ldraw/sources/source_string.h"
#include "pr/view3d-12/ldraw/ldraw_reader_text.h"
#include "pr/view3d-12/ldraw/ldraw_parsing.h"

namespace pr::rdr12::ldraw
{
	template <typename Char>
	SourceString<Char>::SourceString(Guid const* context_id, std::basic_string_view<Char> script, EEncoding enc, PathResolver const& includes)
		: SourceBase(context_id)
		, m_script(script)
		, m_includes(includes)
		, m_encoding(enc)
	{
		// Add the directory of the included file to the paths
		m_includes.FileOpened = [this](auto&, filepath_t const& fp)
		{
			m_includes.LocalDir(fp.parent_path());
			m_filepaths.push_back(fp.lexically_normal());
		};
	}

	// Regenerate the output from the source
	template <typename Char>
	ParseResult SourceString<Char>::ReadSource(Renderer& rdr, std::stop_token stop_token)
	{
		m_errors.resize(0);
		m_filepaths.resize(0);
		m_includes.LocalDir("");

		// Borrow UTF-8 strings directly and convert other encodings once.
		auto utf8 = std::string{};
		auto source = std::string_view{};
		if constexpr (std::is_same_v<Char, char>)
		{
			source = m_script;
			if (m_encoding != EEncoding::utf8)
			{
				utf8 = ToUtf8Source(source, m_encoding);
				source = utf8;
			}
		}
		else
		{
			auto bytes = std::string_view(reinterpret_cast<char const*>(m_script.data()), m_script.size() * sizeof(Char));
			utf8 = ToUtf8Source(bytes, EEncoding::utf16_le);
			source = utf8;
		}

		TextReader reader(source, std::filesystem::path{}, { this, OnReportError }, { this, OnProgress }, m_includes);
		return Parse(rdr, reader, m_context_id, std::move(stop_token));
	}

	template struct SourceString<char>;
	template struct SourceString<wchar_t>;
}
