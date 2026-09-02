//********************************
// Ldraw Script Text Reader2
//  Copyright (c) Rylogic Ltd 2026
//********************************
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/ldraw/ldraw.h"
#include "pr/view3d-12/ldraw/ldraw_parsing.h"

namespace pr::rdr12::ldraw
{
	// Convert caller-owned LDraw source bytes to the UTF-8 contract required by Reader2.
	std::string ToUtf8Source(std::string_view source, EEncoding encoding, std::filesystem::path const& filepath = {});

	// Adapts the UTF-8-native script Reader2 to the streamed LDraw parser interface.
	struct TextReader2 : IReader
	{
		struct Impl;
		std::unique_ptr<Impl> m_impl;

		TextReader2(std::string_view utf8_source, std::filesystem::path src_filepath = {}, ReportErrorCB report_error_cb = nullptr, ParseProgressCB progress_cb = nullptr, IPathResolver const& resolver = NoIncludes::Instance());
		TextReader2(std::string_view utf8_source, Location const& source_location, ReportErrorCB report_error_cb = nullptr, ParseProgressCB progress_cb = nullptr, IPathResolver const& resolver = NoIncludes::Instance());
		TextReader2(std::istream& utf8_stream, std::filesystem::path src_filepath = {}, ReportErrorCB report_error_cb = nullptr, ParseProgressCB progress_cb = nullptr, IPathResolver const& resolver = NoIncludes::Instance());
		TextReader2(TextReader2&&) = delete;
		TextReader2(TextReader2 const&) = delete;
		TextReader2& operator=(TextReader2&&) = delete;
		TextReader2& operator=(TextReader2 const&) = delete;
		~TextReader2();

		// Return the current location in the source.
		Location const& Loc() const override;

		// Move into a nested section.
		void PushSection() override;

		// Leave the current nested section.
		void PopSection() override;

		// True when the current position has reached the end of the current section.
		bool IsSectionEnd() override;

		// True when the source is exhausted.
		bool IsSourceEnd() override;

		// Get the next keyword within the current section.
		bool NextKeywordImpl(int& kw) override;

		// Read an identifier from the current section.
		string32 IdentifierImpl(bool incl_dot) override;

		// Read a UTF-8 string from the current section.
		string32 StringImpl(char escape_char) override;

		// Read an integral value from the current section.
		int64_t IntImpl(int byte_count, int radix) override;

		// Read a dense sequence of integral values without repeated interface dispatch.
		void IntsImpl(int byte_count, std::span<int64_t> values, int radix) override;

		// Read a floating point value from the current section.
		double RealImpl(int byte_count) override;

		// Read a dense sequence of floating-point values without repeated interface dispatch.
		void RealsImpl(int byte_count, std::span<double> values) override;

		// Read an enum value from the current section.
		int64_t EnumImpl(int byte_count, ParseEnumIdentCB parse) override;

		// Read a boolean value from the current section.
		bool BoolImpl() override;

		// Return the most recently read keyword for diagnostics.
		string32 LastKeywordString() const override;
	};
}

#if PR_UNITTESTS
#include <sstream>
#include "pr/common/unittests.h"
namespace pr::rdr12::ldraw::tests
{
	PRUnitTestClass(LDrawTextReader2Tests)
	{
		PRUnitTestMethod(CompactObjectHeader, Quick)
		{
			auto source = std::string("*Point Point_A FF123456 { *Data {1 2 3} } *Box FFABCDEF Box_B { *Data {4 5 6} }");
			TextReader2 reader(source);

			EKeyword keyword;
			PR_EXPECT(reader.NextKeyword(keyword) && keyword == EKeyword::Point);
			{
				auto section = reader.SectionScope();
				PR_EXPECT(reader.NextKeyword(keyword) && keyword == EKeyword::Name);
				PR_EXPECT(reader.Identifier<string32>() == "Point_A");
				PR_EXPECT(reader.NextKeyword(keyword) && keyword == EKeyword::Colour);
				PR_EXPECT(reader.Int<uint32_t>(16) == 0xFF123456);
				PR_EXPECT(reader.NextKeyword(keyword));
				PR_EXPECT(keyword == EKeyword::Data);
				PR_EXPECT(All(reader.Vector3f() == v3(1, 2, 3)));
			}
			PR_EXPECT(reader.NextKeyword(keyword) && keyword == EKeyword::Box);
			{
				auto section = reader.SectionScope();
				PR_EXPECT(reader.NextKeyword(keyword) && keyword == EKeyword::Colour);
				PR_EXPECT(reader.Int<uint32_t>(16) == 0xFFABCDEF);
				PR_EXPECT(reader.NextKeyword(keyword) && keyword == EKeyword::Name);
				PR_EXPECT(reader.Identifier<string32>() == "Box_B");
				PR_EXPECT(reader.NextKeyword(keyword));
				PR_EXPECT(keyword == EKeyword::Data);
				PR_EXPECT(All(reader.Vector3f() == v3(4, 5, 6)));
			}
			PR_EXPECT(!reader.NextKeyword(keyword));
		}
		PRUnitTestMethod(IncludeBridge, Quick)
		{
			PathResolver resolver;
			resolver.AddString("child", "*Point Child FFFFFFFF { *Data {7 8 9} }");
			auto source = std::string("#include \"child\"\n");
			TextReader2 reader(source, "root.ldr", nullptr, nullptr, resolver);

			EKeyword keyword;
			PR_EXPECT(reader.NextKeyword(keyword) && keyword == EKeyword::Point);
			{
				auto section = reader.SectionScope();
				PR_EXPECT(reader.NextKeyword(keyword) && keyword == EKeyword::Name);
				PR_EXPECT(reader.Identifier<string32>() == "Child");
				PR_EXPECT(reader.NextKeyword(keyword) && keyword == EKeyword::Colour);
				PR_EXPECT(reader.Int<uint32_t>(16) == 0xFFFFFFFF);
				PR_EXPECT(reader.NextKeyword(keyword) && keyword == EKeyword::Data);
				PR_EXPECT(All(reader.Vector3f() == v3(7, 8, 9)));
			}
		}
		PRUnitTestMethod(SourceEncodingConversion, Quick)
		{
			auto utf16_le = std::string("\xFF\xFE\x2A\x00\x54\x00\x65\x00\x78\x00\x74\x00\x20\x00\x7B\x00\x22\x00\x3D\xD8\x00\xDE\x22\x00\x7D\x00", 26);
			auto converted = ToUtf8Source(utf16_le, EEncoding::auto_detect, "unicode.ldr");
			PR_EXPECT(converted == "*Text {\"\xF0\x9F\x98\x80\"}");

			auto extended = std::string("\xE9", 1);
			PR_EXPECT(ToUtf8Source(extended, EEncoding::ascii_extended) == "\xC3\xA9");
		}
		PRUnitTestMethod(StreamSourceLocation, Quick)
		{
			auto filepath = std::filesystem::path("snapshot.ldr");
			auto stream = filesys::FileSnapshotStream(filepath, "xx*Point {}", 2);
			TextReader2 reader(stream);

			auto const& loc = reader.Loc();
			PR_EXPECT(loc.m_filepath == filepath);
			PR_EXPECT(loc.m_filesize == 11);
			PR_EXPECT(loc.m_offset == 2);

			auto source = std::string("*Point {}");
			auto source_location = Location{ .m_filesize = 40, .m_offset = 12, .m_column = 3, .m_line = 2 };
			TextReader2 memory_reader(source, source_location);
			auto const& memory_loc = memory_reader.Loc();
			PR_EXPECT(memory_loc.m_filepath.empty());
			PR_EXPECT(memory_loc.m_filesize == 40);
			PR_EXPECT(memory_loc.m_offset == 12);
			PR_EXPECT(memory_loc.m_column == 3);
			PR_EXPECT(memory_loc.m_line == 2);
		}
		PRUnitTestMethod(SourceEndLocation, Quick)
		{
			auto source = std::string("*Point {}");
			auto filepath = std::filesystem::path("finished.ldr");
			TextReader2 reader(source, filepath);
			EKeyword keyword;
			PR_EXPECT(reader.NextKeyword(keyword));
			{
				auto section = reader.SectionScope();
			}
			PR_EXPECT(reader.IsSourceEnd());

			auto const& loc = reader.Loc();
			PR_EXPECT(loc.m_filepath == filepath);
			PR_EXPECT(loc.m_filesize == static_cast<int64_t>(source.size()));
			PR_EXPECT(loc.m_offset == static_cast<int64_t>(source.size()));
		}
	};
}
#endif
