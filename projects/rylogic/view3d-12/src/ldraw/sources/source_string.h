//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"
#include "view3d-12/src/ldraw/sources/source_base.h"

namespace pr::rdr12::ldraw
{
	template <typename Char>
	struct SourceString : SourceBase
	{
		std::basic_string<Char> m_script; // The script source
		PathResolver m_includes;          // Include paths to use with this file
		EEncoding m_encoding;             // The text encoding of the string

		SourceString(Guid const* context_id, std::basic_string_view<Char> script, EEncoding enc, PathResolver const& includes);

	protected:

		// Regenerate the output from the source
		ParseResult ReadSource(Renderer& rdr, std::stop_token stop_token) override;
	};
}
