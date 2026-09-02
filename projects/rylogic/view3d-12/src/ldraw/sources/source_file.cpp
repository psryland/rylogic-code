//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#include "view3d-12/src/ldraw/sources/source_file.h"
#include "pr/view3d-12/ldraw/ldraw_reader_text.h"
#include "pr/view3d-12/ldraw/ldraw_reader_text2.h"
#include "pr/view3d-12/ldraw/ldraw_reader_binary.h"
#include "pr/view3d-12/ldraw/ldraw_parsing.h"
#include "pr/view3d-12/ldraw/ldraw_svg.h"

namespace pr::rdr12::ldraw
{
	namespace
	{
		// Reader2 is the production default; the environment override provides a one-point diagnostic rollback.
		bool UseReader2()
		{
			wchar_t value[16] = {};
			auto length = GetEnvironmentVariableW(L"RYLOGIC_LDRAW_TEXT_READER", value, _countof(value));
			return length == 0 || _wcsicmp(value, L"legacy") != 0;
		}
	}

	SourceFile::SourceFile(Guid const* context_id, filepath_t const& filepath, EEncoding enc, PathResolver const& includes)
		: SourceBase(context_id)
		, m_filepath(filepath.lexically_normal())
		, m_includes(includes)
		, m_encoding(enc)
		, m_text_format()
	{
		m_name = m_filepath.filename().string();
		m_context_id = context_id ? *context_id : ContextIdFromFilepath(m_filepath);

		m_includes.FileOpened = [this](auto&, filepath_t const& fp)
		{
			// Add the directory of the included file to the paths
			m_includes.LocalDir(fp.parent_path());
			m_filepaths.push_back(fp.lexically_normal());
		};
	}

	// Regenerate the output from the source
	ParseResult SourceFile::ReadSource(Renderer& rdr, std::stop_token stop_token)
	{
		m_errors.resize(0);
		m_filepaths.resize(0);

		if (!std::filesystem::exists(m_filepath))
		{
			m_errors.push_back(ParseErrorEventArgs{ std::format("File '{}' not found", m_filepath.string()), EParseError::DataMissing, {} });
			return {};
		}

		m_includes.LocalDir("");
		m_includes.FileOpened(m_includes, m_filepath);

		// Keep text-reader selection in one place so every text-backed source follows the same diagnostic override.
		auto parse_text = [&](std::string_view source, std::filesystem::path const& filepath, EEncoding encoding = EEncoding::utf8)
		{
			auto bom_size = 0;
			if (encoding == EEncoding::auto_detect)
				encoding = filesys::DetectFileEncoding(std::span(source.data(), source.size()), bom_size);
			else if (encoding == EEncoding::utf8)
				bom_size = source.size() >= 3 &&
					static_cast<unsigned char>(source[0]) == 0xEF &&
					static_cast<unsigned char>(source[1]) == 0xBB &&
					static_cast<unsigned char>(source[2]) == 0xBF
					? 3
					: 0;
			else if (source.size() >= 2)
				bom_size =
					(encoding == EEncoding::utf16_le && static_cast<unsigned char>(source[0]) == 0xFF && static_cast<unsigned char>(source[1]) == 0xFE) ||
					(encoding == EEncoding::utf16_be && static_cast<unsigned char>(source[0]) == 0xFE && static_cast<unsigned char>(source[1]) == 0xFF)
					? 2
					: 0;

			if (UseReader2())
			{
				auto utf8 = std::string{};
				auto utf8_source = source;
				switch (encoding)
				{
					case EEncoding::utf8:
					{
						break;
					}
					case EEncoding::ascii:
					case EEncoding::ascii_extended:
					case EEncoding::utf16_le:
					case EEncoding::utf16_be:
					{
						utf8 = ToUtf8Source(source, encoding, filepath);
						utf8_source = utf8;
						break;
					}
					default:
					{
						throw std::runtime_error(std::format("Unsupported file encoding: {}", int(encoding)));
					}
				}

				auto location = Location
				{
					.m_filepath = filepath,
					.m_filesize = static_cast<int64_t>(source.size()),
					.m_offset = bom_size,
					.m_column = 1,
					.m_line = 1,
				};
				TextReader2 reader(utf8_source, location, { this, OnReportError }, { this, OnProgress }, m_includes);
				return Parse(rdr, reader, m_context_id, stop_token);
			}

			auto stream = filesys::FileSnapshotStream(filepath, std::string(source), static_cast<size_t>(bom_size));
			TextReader reader(stream, {}, encoding, { this, OnReportError }, { this, OnProgress }, m_includes);
			return Parse(rdr, reader, m_context_id, stop_token);
		};

		// Handle based on file extension
		auto extn = m_filepath.extension().string();
		switch (HashI(extn.c_str()))
		{
			// LDR = Text ldr script file
			case HashI(".ldr"):
			{
				auto snapshot = filesys::FileSnapshot(m_filepath);
				m_text_format = true;

				// Normalize the stable snapshot once at the LDraw source boundary.
				return parse_text(snapshot.str(), m_filepath, m_encoding);
			}

			// BDR = Binary ldr script file
			case HashI(".bdr"):
			{
				auto snapshot = filesys::FileSnapshot(m_filepath);
				m_text_format = false;

				// Parse the ldr script file
				auto src = filesys::FileSnapshotStream(std::move(snapshot));
				ldraw::BinaryReader reader(src, {}, { this, OnReportError }, { this, OnProgress }, m_includes);
				return Parse(rdr, reader, m_context_id, stop_token);
			}

			// SVG = Scalable Vector Graphics, translated to LDraw script
			case HashI(".svg"):
			{
				auto snapshot = filesys::FileSnapshot(m_filepath);
				auto ldr_script = pr::ldraw::svg::Read(snapshot.str()).ToString();
				m_text_format = true;

				return parse_text(ldr_script, m_filepath);
			}

			// P3D = My custom binary model file format
			// STL = "StereoLithography" model files (binary and text)
			// 3DS = 3D Studio Max model files (binary and text)
			// FBX/GLTF/GLB = Model file formats loaded via optional hot-loaded dlls
			case HashI(".p3d"):
			case HashI(".stl"):
			case HashI(".3ds"):
			{
				auto ldr_script = std::format("*Model {{ *FilePath {{\"{}\"}} }}", m_filepath.string());
				m_text_format = false;

				return parse_text(ldr_script, m_filepath);
			}

			// FBX/GLTF/GLB can contain animation data. Direct file opening should expose it when present without treating static models as errors.
			case HashI(".fbx"):
			case HashI(".gltf"):
			case HashI(".glb"):
			{
				auto ldr_script = std::format("*Model {{ *FilePath {{\"{}\"}} *Animation {{}} }}", m_filepath.string());
				m_text_format = false;

				return parse_text(ldr_script, m_filepath);
			}

			// CSV data, create a chart to graph the data
			case HashI(".csv"):
			{
				auto ldr_script = std::format("*Chart {{ *FilePath {{\"{}\"}} }}", m_filepath.string());
				m_text_format = true;

				return parse_text(ldr_script, m_filepath);
			}

			// Unknown file type
			default:
			{
				throw std::runtime_error(std::format("Unknown file type: {}", extn));
			}
		}
	}
}
