//**********************************
// Script Reader Benchmark
//  Copyright (c) Rylogic Ltd 2015
//**********************************
// Templated parse-and-checksum drivers, one per workload, each instantiated once per reader backend.
//
// Style Guidance:
//  - Every 'Drive*' function is a template parameterised on the concrete reader type so that no
//    runtime backend branch exists inside the parsing loop: 'if constexpr' in 'MakeReader' below is
//    resolved entirely at compile time, so 'DriveX<pr::script::Reader>' and
//    'DriveX<pr::script::v2::Reader>' are two independent, monomorphic functions.
//  - Each function performs exactly one full parse of its input buffer and folds every extracted
//    value through 'Checksum::Add', matching the fixture shape written by the corresponding
//    generator in 'workload_generators.h'.
//  - Legacy 'pr::script::Reader::Keyword' takes a 'wchar_t const*' while 'pr::script::v2::Reader::Keyword'
//    takes a 'std::string_view'; 'ExpectKeyword' below is the one place that difference is bridged,
//    since every keyword name used here is plain ASCII and so widens losslessly.
#pragma once
#include "forward.h"
#include "checksum.h"
#include "workload_generators.h"

namespace pr::script_bench
{
	// The physical source used to construct each reader inside the timed region.
	enum class EInputMode
	{
		Memory,
		Stream,
		File,
	};

	// A generated fixture and its optional materialized file representation.
	struct BenchmarkSource
	{
		std::string const& m_text;
		std::filesystem::path const& m_filepath;
		std::filesystem::path const& m_include_root;
		EInputMode m_input_mode;
		uint64_t m_total_bytes;
	};

	// Result of driving one full parse: a black-box checksum, the number of items extracted, and the
	// input size, used both for cross-backend correctness comparison and for throughput reporting.
	struct ParseResult
	{
		Checksum m_checksum;
		uint64_t m_items;
		uint64_t m_bytes;
	};

	// Owns the transport and legacy source objects that must outlive one timed reader parse.
	template <typename ReaderType>
	class ReaderOwner;

	// Owns a legacy reader and the selected byte source feeding it.
	template <>
	class ReaderOwner<pr::script::Reader>
	{
		std::unique_ptr<pr::script::Includes> m_includes;
		std::unique_ptr<pr::mem_istream<char>> m_stream;
		std::unique_ptr<pr::script::StreamSrc<char>> m_stream_src;
		std::unique_ptr<pr::script::FileSrc> m_file_src;
		std::unique_ptr<pr::script::Reader> m_reader;

	public:

		// Construct the legacy reader through the selected memory, stream, or file source.
		explicit ReaderOwner(BenchmarkSource const& source)
		{
			// Configure production file-include resolution only for the include-tree workload.
			if (!source.m_include_root.empty())
			{
				m_includes = std::make_unique<pr::script::Includes>(pr::script::EIncludeTypes::Files);
				m_includes->SearchPaths({ source.m_include_root });
			}

			switch (source.m_input_mode)
			{
				case EInputMode::Memory:
				{
					m_reader = std::make_unique<pr::script::Reader>(source.m_text.c_str(), false, m_includes.get());
					break;
				}
				case EInputMode::Stream:
				{
					m_stream = std::make_unique<pr::mem_istream<char>>(std::string_view(source.m_text), 0);
					m_stream_src = std::make_unique<pr::script::StreamSrc<char>>(*m_stream, pr::EEncoding::utf8);
					m_reader = std::make_unique<pr::script::Reader>(*m_stream_src, false, m_includes.get());
					break;
				}
				case EInputMode::File:
				{
					m_file_src = std::make_unique<pr::script::FileSrc>(source.m_filepath, pr::EEncoding::utf8);
					m_reader = std::make_unique<pr::script::Reader>(*m_file_src, false, m_includes.get());
					break;
				}
				default:
				{
					throw std::invalid_argument("unknown EInputMode value");
				}
			}
		}

		// Return the constructed reader.
		pr::script::Reader& Reader()
		{
			return *m_reader;
		}
	};

	// Owns a Reader2 reader and the selected stream object feeding it.
	template <>
	class ReaderOwner<pr::script::v2::Reader>
	{
		std::unique_ptr<pr::script::v2::FileIncludeHandler2> m_includes;
		std::unique_ptr<pr::mem_istream<char>> m_memory_stream;
		std::unique_ptr<std::ifstream> m_file_stream;
		std::unique_ptr<pr::script::v2::Reader> m_reader;

	public:

		// Construct Reader2 through the selected memory, stream, or file source.
		explicit ReaderOwner(BenchmarkSource const& source)
		{
			// Configure Reader2's UTF-8 file include handler only when the fixture owns an include tree.
			if (!source.m_include_root.empty())
			{
				m_includes = std::make_unique<pr::script::v2::FileIncludeHandler2>();
				m_includes->AddSearchPath(source.m_include_root);
			}

			switch (source.m_input_mode)
			{
				case EInputMode::Memory:
				{
					m_reader = std::make_unique<pr::script::v2::Reader>(std::string_view(source.m_text), false, source.m_filepath, m_includes.get());
					break;
				}
				case EInputMode::Stream:
				{
					m_memory_stream = std::make_unique<pr::mem_istream<char>>(std::string_view(source.m_text), 0);
					m_reader = std::make_unique<pr::script::v2::Reader>(*m_memory_stream, false, source.m_filepath, m_includes.get());
					break;
				}
				case EInputMode::File:
				{
					m_file_stream = std::make_unique<std::ifstream>(source.m_filepath, std::ios::binary);
					if (!m_file_stream->is_open())
						throw std::runtime_error("failed to open benchmark fixture: " + source.m_filepath.string());

					m_reader = std::make_unique<pr::script::v2::Reader>(*m_file_stream, false, source.m_filepath, m_includes.get());
					break;
				}
				default:
				{
					throw std::invalid_argument("unknown EInputMode value");
				}
			}
		}

		// Return the constructed reader.
		pr::script::v2::Reader& Reader()
		{
			return *m_reader;
		}
	};

	// Scans forward to the named keyword, bridging the legacy/Reader2 'Keyword' signature difference.
	template <typename ReaderType>
	void ExpectKeyword(ReaderType& reader, char const* name)
	{
		if constexpr (std::is_same_v<ReaderType, pr::script::Reader>)
		{
			std::wstring wname(name, name + std::strlen(name)); // ASCII-only names widen losslessly.
			reader.Keyword(wname.c_str());
		}
		else
		{
			reader.Keyword(std::string_view(name));
		}
	}

	// Number-heavy: reads the Verts/Normals/UVs/Indices sections written by 'GenerateNumberHeavy'.
	template <typename ReaderType>
	ParseResult DriveNumberHeavy(BenchmarkSource const& source, SizeParams const& p)
	{
		auto owner = ReaderOwner<ReaderType>(source);
		auto& reader = owner.Reader();
		Checksum cs;
		uint64_t items = 0;

		ExpectKeyword(reader, "Mesh");
		reader.SectionStart();

		ExpectKeyword(reader, "Verts");
		reader.SectionStart();
		for (int i = 0; i != p.m_mesh_vertices; ++i)
		{
			pr::v4 v = {};
			reader.Vector3(v, 1.0f);
			cs.Add(v.x); cs.Add(v.y); cs.Add(v.z);
			++items;
		}
		reader.SectionEnd();

		ExpectKeyword(reader, "Normals");
		reader.SectionStart();
		for (int i = 0; i != p.m_mesh_vertices; ++i)
		{
			pr::v4 v = {};
			reader.Vector3(v, 0.0f);
			cs.Add(v.x); cs.Add(v.y); cs.Add(v.z);
			++items;
		}
		reader.SectionEnd();

		ExpectKeyword(reader, "UVs");
		reader.SectionStart();
		for (int i = 0; i != p.m_mesh_vertices; ++i)
		{
			pr::v2 uv = {};
			reader.Vector2(uv);
			cs.Add(uv.x); cs.Add(uv.y);
			++items;
		}
		reader.SectionEnd();

		ExpectKeyword(reader, "Indices");
		reader.SectionStart();
		for (int i = 0; i != p.m_mesh_triangles; ++i)
		{
			int a = 0, b = 0, c = 0;
			reader.Int(a, 10);
			reader.Int(b, 10);
			reader.Int(c, 10);
			cs.Add(int64_t(a)); cs.Add(int64_t(b)); cs.Add(int64_t(c));
			++items;
		}
		reader.SectionEnd();

		reader.SectionEnd(); // Mesh
		return ParseResult{ cs, items, source.m_total_bytes };
	}

	// Identifier-heavy (both empty- and populated-macro variants): reads the flat identifier list
	// written by 'GenerateIdentifierHeavy'. The extraction sequence is identical for both variants;
	// only the fixture text (and therefore the reader's macro-table state) differs between them.
	template <typename ReaderType>
	ParseResult DriveIdentifierHeavy(BenchmarkSource const& source, SizeParams const& p)
	{
		auto owner = ReaderOwner<ReaderType>(source);
		auto& reader = owner.Reader();
		Checksum cs;
		uint64_t items = 0;

		ExpectKeyword(reader, "Idents");
		reader.SectionStart();
		for (int i = 0; i != p.m_ident_count; ++i)
		{
			std::string ident;
			reader.Identifier(ident);
			cs.Add(std::string_view(ident));
			++items;
		}
		reader.SectionEnd();

		return ParseResult{ cs, items, source.m_total_bytes };
	}

	// Strings/non-ASCII: reads the quoted-string list written by 'GenerateStringsUtf8'. Legacy 'Reader'
	// decodes source bytes into 'wchar_t' internally, so extracting into a narrow 'std::string' would
	// truncate every non-ASCII code point instead of re-encoding it; extracting into 'std::wstring' and
	// then narrowing via 'pr::Narrow' round-trips UTF-8 correctly and matches Reader2's native
	// UTF-8 'std::string' output, so both backends are checksummed against the same decoded text.
	template <typename ReaderType>
	ParseResult DriveStringsUtf8(BenchmarkSource const& source, SizeParams const& p)
	{
		auto owner = ReaderOwner<ReaderType>(source);
		auto& reader = owner.Reader();
		Checksum cs;
		uint64_t items = 0;

		ExpectKeyword(reader, "Strings");
		reader.SectionStart();
		for (int i = 0; i != p.m_string_count; ++i)
		{
			if constexpr (std::is_same_v<ReaderType, pr::script::Reader>)
			{
				std::wstring w;
				reader.String(w);
				cs.Add(std::string_view(pr::Narrow(w)));
			}
			else
			{
				std::string s;
				reader.String(s);
				cs.Add(std::string_view(s));
			}
			++items;
		}
		reader.SectionEnd();

		return ParseResult{ cs, items, source.m_total_bytes };
	}

	// Directive/macro-heavy: reads the Flags/Values sections written by 'GenerateDirectiveMacroHeavy'.
	template <typename ReaderType>
	ParseResult DriveDirectiveMacroHeavy(BenchmarkSource const& source, SizeParams const& p)
	{
		constexpr int FlagToggleCount = 16; // Must match 'GenerateDirectiveMacroHeavy'.

		auto owner = ReaderOwner<ReaderType>(source);
		auto& reader = owner.Reader();
		Checksum cs;
		uint64_t items = 0;

		ExpectKeyword(reader, "Flags");
		reader.SectionStart();
		for (int i = 0; i != FlagToggleCount; ++i)
		{
			ExpectKeyword(reader, "Flag");
			bool flag = false;
			reader.Bool(flag);
			cs.Add(flag);
			++items;
		}
		reader.SectionEnd();

		ExpectKeyword(reader, "Values");
		reader.SectionStart();
		for (int i = 0; i != p.m_macro_value_count; ++i)
		{
			int v = 0;
			reader.Int(v, 10);
			cs.Add(int64_t(v));
			++items;
		}
		reader.SectionEnd();

		return ParseResult{ cs, items, source.m_total_bytes };
	}

	// Token-boundary adversary: reads the mixed-kind, comment-padded sequence written by
	// 'GenerateTokenBoundaryAdversary', cycling through int/real/string/identifier extraction in the
	// same fixed order the generator used to decide what to emit.
	template <typename ReaderType>
	ParseResult DriveTokenBoundaryAdversary(BenchmarkSource const& source, SizeParams const& p)
	{
		auto owner = ReaderOwner<ReaderType>(source);
		auto& reader = owner.Reader();
		Checksum cs;
		uint64_t items = 0;

		ExpectKeyword(reader, "Boundary");
		reader.SectionStart();
		for (int i = 0; i != p.m_boundary_item_count; ++i)
		{
			switch (i % 4)
			{
				case 0:
				{
					int v = 0;
					reader.Int(v, 10);
					cs.Add(int64_t(v));
					break;
				}
				case 1:
				{
					double v = 0;
					reader.Real(v);
					cs.Add(v);
					break;
				}
				case 2:
				{
					std::string s;
					reader.String(s);
					cs.Add(std::string_view(s));
					break;
				}
				case 3:
				{
					std::string ident;
					reader.Identifier(ident);
					cs.Add(std::string_view(ident));
					break;
				}
				default:
				{
					throw std::logic_error("unreachable: i % 4 is always in [0, 4)");
				}
			}
			++items;
		}
		reader.SectionEnd();

		return ParseResult{ cs, items, source.m_total_bytes };
	}

	// Include-tree: read the values emitted depth-first across the generated physical source tree.
	template <typename ReaderType>
	ParseResult DriveIncludeTree(BenchmarkSource const& source, SizeParams const& p)
	{
		auto owner = ReaderOwner<ReaderType>(source);
		auto& reader = owner.Reader();
		auto cs = Checksum{};
		auto items = uint64_t{};

		ExpectKeyword(reader, "Includes");
		reader.SectionStart();
		for (int index = 0, count = p.m_include_file_count * p.m_include_value_count; index != count; ++index)
		{
			auto value = int{};
			reader.Int(value, 10);
			cs.Add(int64_t(value));
			++items;
		}
		reader.SectionEnd();
		if (!reader.IsSourceEnd())
			throw std::runtime_error("include-tree fixture contains unexpected trailing data");

		return ParseResult{ cs, items, source.m_total_bytes };
	}
}
