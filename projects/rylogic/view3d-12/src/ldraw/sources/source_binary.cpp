//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#include "view3d-12/src/ldraw/sources/source_binary.h"
#include "pr/view3d-12/ldraw/ldraw_reader_binary.h"

namespace pr::rdr12::ldraw
{
	SourceBinary::SourceBinary(Guid const* context_id, std::span<std::byte const> data)
		: SourceBase(context_id)
		, m_script(data)
	{
	}

	// Regenerate the output from the source
	ParseResult SourceBinary::ReadSource(Renderer& rdr, std::stop_token stop_token)
	{
		m_errors.resize(0);
		m_filepaths.resize(0);

		mem_istream<char> src{ m_script.data(), m_script.size() };
		BinaryReader reader(src, {}, { this, OnReportError }, { this, OnProgress });
		return Parse(rdr, reader, m_context_id, std::move(stop_token));
	}
}
