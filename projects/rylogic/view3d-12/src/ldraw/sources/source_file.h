//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"
#include "view3d-12/src/ldraw/sources/source_base.h"

namespace pr::rdr12::ldraw
{
	// An Ldr script source
	struct SourceFile : SourceBase
	{
		filepath_t   m_filepath;    // The filepath of the source (if there is one)
		PathResolver m_includes;    // Include paths to use with this file
		EEncoding    m_encoding;    // The file encoding
		bool         m_text_format; // True if the file is a text based format

		SourceFile(Guid const* context_id, filepath_t const& filepath, EEncoding enc, PathResolver const& includes);

	protected:

		// Regenerate the output from the source
		ParseResult ReadSource(Renderer& rdr, std::stop_token stop_token) override;
	};
}
