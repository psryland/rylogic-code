//********************************
// Ldraw Script Binary Serialiser
//  Copyright (c) Rylogic Ltd 2025
//********************************
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/ldraw/ldraw.h"
#include "pr/view3d-12/ldraw/ldraw_parsing.h"
#include "pr/view3d-12/ldraw/ldraw_serialiser.h"

namespace pr::rdr12::ldraw
{
	struct BinaryReader : IReader
	{
		// Byte offsets from the start of the stream for the range of the data in the current section (excludes the header)
		using SectionSpan = struct { int64_t m_beg, m_end; };
		using SectionStack = pr::vector<SectionSpan>;
		using istream_t = std::basic_istream<char>; // 'char' to support ifstream

		istream_t& m_src;            // The input byte stream
		int64_t m_pos;               // The number of bytes read from the stream so far, or the index of the next byte to read (same thing)
		SectionStack m_section;      // A stack of section headers. back() == top == current section.
		mutable Location m_location; // Source location description

		BinaryReader(istream_t& src, std::filesystem::path src_filepath, ReportErrorCB report_error_cb = nullptr, ParseProgressCB progress_cb = nullptr, IPathResolver const& resolver = PathResolver::Instance())
			: IReader(report_error_cb, progress_cb, resolver)
			, m_src(src)
			, m_pos()
			, m_section({ {0, std::numeric_limits<int64_t>::max()} })
			, m_location({ src_filepath, ReadSourceLength(src_filepath) })
		{
			PushSection();
		}
		BinaryReader(BinaryReader&&) = delete;
		BinaryReader(BinaryReader const&) = delete;
		BinaryReader& operator=(BinaryReader&&) = delete;
		BinaryReader& operator=(BinaryReader const&) = delete;

		// Return the current location in the source
		virtual Location const& Loc() const override
		{
			m_location.m_offset = m_pos;
			return m_location;
		}

		// Move into a nested section
		virtual void PushSection() override
		{
			// The current top of the stack becomes the parent
			m_section.push_back({ m_pos, m_pos });
		}

		// Leave the current nested section
		virtual void PopSection() override
		{
			m_section.pop_back();

			// Should always have the dummy "global" parent section and a 'current' section
			assert(m_section.size() >= 2);
		}

		// True when the current position has reached the end of the current section
		virtual bool IsSectionEnd() override
		{
			return m_pos == m_section.back().m_end;
		}

		// True when the source is exhausted
		virtual bool IsSourceEnd() override
		{
			return m_src.eof();
		}

		// Get the next keyword within the current section. Returns false if at the end of the section
		virtual bool NextKeywordImpl(int& kw) override
		{
			// Out of data
			if (m_src.eof())
				return false;

			// The top of the stack is the last section read at the current nesting level
			// The next on the stack is the parent level.
			auto& last = m_section[m_section.size() - 1];
			auto& parent = m_section[m_section.size() - 2];

			// Seek to the end of the current section.
			m_src.seekg(last.m_end - m_pos, std::ios::cur).peek(); // Peek to test for EOF
			m_pos = last.m_end;
			if (m_pos < 0)
				throw std::runtime_error("Corrupt ldraw data");

			// If this is the end of the parent section then there are no more sections at this level.
			if (m_pos == parent.m_end || m_src.eof())
				return false;

			// Read the next section header at this level
			SectionHeader header;
			Read(&header, sizeof(header));
			kw = static_cast<int>(header.m_keyword);

			// Replace the top of the stack
			m_section.back() = { m_pos, m_pos + header.m_size };
			return true;
		}
		
		// Read an identifier from the current section. Leading '10xxxxxx' bytes are the length (in bytes). Default length is the full section
		virtual string32 IdentifierImpl(bool) override
		{
			auto length = ReadLengthBytes();

			// Read the string 
			string32 str(length, 0);
			Read(str.data(), str.size());
			return str;
		}

		// Read a utf8 string from the current section.
		virtual string32 StringImpl(char) override
		{
			auto length = ReadLengthBytes();

			// Read the string 
			string32 str(length, 0);
			Read(str.data(), str.size());
			return str;
		}

		// Read an integral value from the current section
		virtual int64_t IntImpl(int byte_count, int = 0) override
		{
			switch (byte_count)
			{
				case 1: { int8_t v; Read(&v, sizeof(v)); return v; }
				case 2: { int16_t v; Read(&v, sizeof(v)); return v; }
				case 4: { int32_t v; Read(&v, sizeof(v)); return v; }
				case 8: { int64_t v; Read(&v, sizeof(v)); return v; }
				default: throw std::runtime_error("Invalid byte count");
			}
		}

		// Read a floating point value from the current section
		virtual double RealImpl(int byte_count) override
		{
			auto value = IntImpl(byte_count, 0);
			switch (byte_count)
			{
				case 2: return reinterpret_cast<half_t const&>(value);
				case 4: return reinterpret_cast<float const&>(value);
				case 8: return reinterpret_cast<double const&>(value);
				default: throw std::runtime_error("Invalid byte count");
			}
		}
		
		// Read an enum value from the current section
		virtual int64_t EnumImpl(int byte_count, ParseEnumIdentCB) override
		{
			return IntImpl(byte_count);
		}

		// Read a boolean value from the current section
		virtual bool BoolImpl() override
		{
			return IntImpl(1, 0) != 0;
		}

	private:

		// Read 'size' bytes into 'buf'
		void Read(void* buf, int64_t size)
		{
			if (size < 0)
				throw std::runtime_error("Invalid read size");

			if (!m_src.read(char_ptr(buf), static_cast<size_t>(size)).good())
			{
				ReportError(EParseError::DataMissing, Loc(), "Read failed");
				memset(buf, 0, static_cast<size_t>(size));
			}
			m_pos += size;
		}

		// Read bytes with pattern '10xxxxxx'. These are invalid UTF-8 characters which I'm using for length information
		size_t ReadLengthBytes()
		{
			size_t length = 0;
			for (;;)
			{
				auto ch = m_src.peek();
				if (ch == std::char_traits<char>::eof() || (ch & 0xC0) != 0x80)
					break;

				length = (length << 6) + (ch & 0b00111111);
				m_src.read(char_ptr(&ch), 1);
				++m_pos;
			}

			// If length is zero, infer the length from the section length
			if (length == 0)
			{
				auto& header = m_section.back();
				length = static_cast<size_t>(header.m_end - m_pos);
			}

			return length;
		}

		// Read the size of the input file/source
		static int64_t ReadSourceLength(std::filesystem::path src_filepath)
		{
			if (!src_filepath.empty() && std::filesystem::exists(src_filepath))
				return std::filesystem::file_size(src_filepath);
			
			return 0; // Unknown
		}
	};
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/view3d-12/ldraw/ldraw_writer_binary.h"
namespace pr::rdr12::ldraw
{
	PRUnitTest(LDrawBinarySerialiserTests)
	{
		std::vector<char> data;

		pr::mem_ostream<char> strm(data);
		BinaryWriter::Write(strm, EKeyword::Point, [&]
		{
			BinaryWriter::Write(strm, EKeyword::Name, "TestPoints");
			BinaryWriter::Write(strm, EKeyword::Colour, 0xFF00FF00);
			BinaryWriter::Write(strm, EKeyword::Data, v3(1,1,1), v3(2,2,2), v3(3,3,3));
			BinaryWriter::Write(strm, EKeyword::Line, [&]
			{
				BinaryWriter::Write(strm, EKeyword::Name, "TestLines");
				BinaryWriter::Write(strm, EKeyword::Colour, 0xFF0000FF);
				BinaryWriter::Write(strm, EKeyword::Data, v3(-1,-1,0), v3(1,1,0), v3(-1,1,0), v3(1,-1,0));
			});
			BinaryWriter::Write(strm, EKeyword::Sphere, [&]
			{
				BinaryWriter::Write(strm, EKeyword::Name, "TestSphere");
				BinaryWriter::Write(strm, EKeyword::Colour, 0xFFFF0000);
				BinaryWriter::Write(strm, EKeyword::Data, 1.0f);
			});
			BinaryWriter::Write(strm, EKeyword::Custom, [&]
			{
				std::string_view s = "ShortString";
				BinaryWriter::Write(strm, EKeyword::Name, StringWithLength{ s }, StringWithLength{ s });
			});
		});

		PR_EXPECT(data.size() != 0);

		#if 1
		{
			std::ofstream ofile(temp_dir() / "ldraw_test.lbr", std::ios::binary);
			ofile.write(data.data(), data.size());
		}
		#endif

		mem_istream<char> src(data);
		BinaryReader reader(src, {});

		PR_EXPECT(reader.Loc().m_offset == 0);
		
		EKeyword kw;
		PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Point);
		{
			auto points = reader.SectionScope();
			PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Name);
			PR_EXPECT(reader.Identifier<string32>() == "TestPoints");

			PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Colour);
			PR_EXPECT(reader.Int<uint32_t>() == 0xFF00FF00);

			PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Data);
			PR_EXPECT(reader.Vector3f().w1() == v4(1,1,1,1));
			PR_EXPECT(reader.Vector3f().w1() == v4(2,2,2,1));
			PR_EXPECT(reader.Vector3f().w1() == v4(3,3,3,1));

			PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Line); // Skip Line
			
			PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Sphere);
			{
				auto sphere = reader.SectionScope();
				PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Name);
				PR_EXPECT(reader.Identifier<string32>() == "TestSphere");

				PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Colour);
				PR_EXPECT(reader.Int<uint32_t>() == 0xFFFF0000);

				PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Data);
				PR_EXPECT(reader.Real<float>() == 1.0f);
			}

			PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Custom);
			{
				auto sphere = reader.SectionScope();
				PR_EXPECT(reader.NextKeyword(kw) && kw == EKeyword::Name);
				PR_EXPECT(reader.String<string32>() == "ShortString");
				PR_EXPECT(reader.String<string32>() == "ShortString");
			}

			PR_EXPECT(!reader.NextKeyword(kw));
			PR_EXPECT(reader.IsSectionEnd());
		}

		PR_EXPECT(reader.Loc().m_offset == isize(data));
	}
}
#endif
