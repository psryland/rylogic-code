//********************************
// Ldraw Script Text Serialiser
//  Copyright (c) Rylogic Ltd 2025
//********************************
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/ldraw/ldraw.h"
#include "pr/view3d-12/ldraw/ldraw_serialiser.h"
#include "pr/view3d-12/ldraw/ldraw_parsing.h"

namespace pr::rdr12::ldraw
{
	struct TextReader : IReader
	{
		alignas(8) std::byte m_src[224];
		alignas(8) std::byte m_pp[896];
		mutable Location m_location;
		string32 m_keyword;
		wstring32 m_delim;
		int m_section_level;
		int m_nest_level;

		TextReader(std::istream& stream, std::filesystem::path src_filepath, EEncoding enc = EEncoding::utf8, ReportErrorCB report_error_cb = nullptr, ParseProgressCB progress_cb = nullptr, IPathResolver const& resolver = NoIncludes::Instance());
		TextReader(std::wistream& stream, std::filesystem::path src_filepath, EEncoding enc = EEncoding::utf16_le, ReportErrorCB report_error_cb = nullptr, ParseProgressCB progress_cb = nullptr, IPathResolver const& resolver = NoIncludes::Instance());
		TextReader(TextReader&&) = delete;
		TextReader(TextReader const&) = delete;
		TextReader& operator=(TextReader&&) = delete;
		TextReader& operator=(TextReader const&) = delete;
		~TextReader();

		// Return the current location in the source
		virtual Location const& Loc() const override;

		// Move into a nested section
		virtual void PushSection() override;

		// Leave the current nested section
		virtual void PopSection() override;

		// True when the current position has reached the end of the current section
		virtual bool IsSectionEnd() override;

		// True when the source is exhausted
		virtual bool IsSourceEnd() override;

		// Get the next keyword within the current section.
		// Returns false if at the end of the section
		virtual bool NextKeywordImpl(int& kw) override;

		// Read an identifier from the current section. Leading '10xxxxxx' bytes are the length (in bytes). Default length is the full section
		virtual string32 IdentifierImpl(bool incl_dot) override;

		// Read a utf8 string from the current section. Leading '10xxxxxx' bytes are the length (in bytes). Default length is the full section
		virtual string32 StringImpl(char escape_char) override;

		// Read an integral value from the current section
		virtual int64_t IntImpl(int byte_count, int radix) override;

		// Read a floating point value from the current section
		virtual double RealImpl(int byte_count) override;

		// Read an enum value from the current section
		virtual int64_t EnumImpl(int byte_count, ParseEnumIdentCB parse) override;

		// Read a boolean value from the current section
		virtual bool BoolImpl() override;
	};
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/view3d-12/ldraw/ldraw_writer_text.h"
namespace pr::rdr12::ldraw
{
	PRUnitTest(LDrawTextSerialiserTests)
	{
		std::ostringstream strm;
		TextWriter::Write(strm, EKeyword::Point, "TestPoints", 0xFF00FF00, [&]
		{
			TextWriter::Write(strm, EKeyword::Data, v3(1, 1, 1), v3(2, 2, 2), v3(3, 3, 3));
			TextWriter::Write(strm, EKeyword::Line, "TestLines", 0xFF0000FF, [&]
			{
				TextWriter::Write(strm, EKeyword::Data, v3(-1, -1, 0), v3(1, 1, 0), v3(-1, 1, 0), v3(1, -1, 0));
			});
			TextWriter::Write(strm, EKeyword::Sphere, "TestSphere", 0xFFFF0000, [&]
			{
				TextWriter::Write(strm, EKeyword::Data, 1.0f);
			});
			TextWriter::Write(strm, EKeyword::Custom, [&]
			{
				std::string_view s = "ShortString";
				TextWriter::Write(strm, EKeyword::Name, StringWithLength{ s }, StringWithLength{ s });
			});
		});

		auto data = strm.str();
		PR_EXPECT(data.size() != 0);

		#if 0
		{
			std::ofstream ofile(temp_dir / "ldraw_test.lbr");
			ofile.write(data.data(), data.size());
		}
		#endif
	}
}
#endif
