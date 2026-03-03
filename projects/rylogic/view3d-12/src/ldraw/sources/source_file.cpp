//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#include "view3d-12/src/ldraw/sources/source_file.h"
#include "pr/view3d-12/ldraw/ldraw_reader_text.h"
#include "pr/view3d-12/ldraw/ldraw_reader_binary.h"
#include "pr/view3d-12/ldraw/ldraw_parsing.h"

namespace pr::rdr12::ldraw
{
	SourceFile::SourceFile(Guid const* context_id, filepath_t const& filepath, EEncoding enc, PathResolver const& includes)
		: SourceBase(context_id)
		, m_filepath(filepath.lexically_normal())
		, m_includes(includes)
		, m_encoding(enc != EEncoding::auto_detect ? enc : filesys::DetectFileEncoding(m_filepath))
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

		// Handle based on file extension
		auto extn = m_filepath.extension().string();
		switch (HashI(extn.c_str()))
		{
			// LDR = Text ldr script file
			case HashI(".ldr"):
			{
				filesys::LockFile lock(m_filepath, 10, 5000);
				m_text_format = true;

				// Parse the ldr script text file
				switch (m_encoding)
				{
					case EEncoding::utf16_le:
					case EEncoding::utf16_be:
					{
						std::wifstream src(m_filepath);
						TextReader reader(src, m_filepath, m_encoding, { this, OnReportError }, { this, OnProgress }, m_includes);
						return Parse(rdr, reader, m_context_id, stop_token);
					}
					case EEncoding::ascii:
					case EEncoding::utf8:
					{
						std::ifstream src(m_filepath);
						TextReader reader(src, m_filepath, m_encoding, { this, OnReportError }, { this, OnProgress }, m_includes);
						return Parse(rdr, reader, m_context_id, stop_token);
					}
					default:
					{
						throw std::runtime_error(std::format("Unsupported file encoding: {}", int(m_encoding)));
					}
				}
			}

			// BDR = Binary ldr script file
			case HashI(".bdr"):
			{
				filesys::LockFile lock(m_filepath, 10, 5000);
				m_text_format = false;

				// Parse the ldr script file
				std::ifstream src(m_filepath, std::ios::binary);
				ldraw::BinaryReader reader(src, m_filepath, { this, OnReportError }, { this, OnProgress }, m_includes);
				return Parse(rdr, reader, m_context_id, stop_token);
			}

			// P3D = My custom binary model file format
			// STL = "StereoLithography" model files (binary and text)
			// 3DS = 3D Studio Max model files (binary and text)
			case HashI(".p3d"):
			case HashI(".stl"):
			case HashI(".3ds"):
			case HashI(".fbx"):
			{
				auto ldr_script = std::format("*Model {{ *FilePath {{\"{}\"}} *Animation{{}} }}", m_filepath.string());
				m_text_format = false;

				mem_istream<char> src{ ldr_script, 0 };
				TextReader reader(src, {}, EEncoding::utf8, { this, OnReportError }, { this, OnProgress }, m_includes);
				return Parse(rdr, reader, m_context_id, stop_token);
			}

			// CSV data, create a chart to graph the data
			case HashI(".csv"):
			{
				auto ldr_script = std::format("*Chart {{ *FilePath {{\"{}\"}} }}", m_filepath.string());
				m_text_format = true;

				mem_istream<char> src{ ldr_script, 0 };
				TextReader reader(src, {}, EEncoding::utf8, { this, OnReportError }, { this, OnProgress }, m_includes);
				return Parse(rdr, reader, m_context_id, stop_token);
			}

			// Unknown file type
			default:
			{
				throw std::runtime_error(std::format("Unknown file type: {}", extn));
			}
		}
	}
}
