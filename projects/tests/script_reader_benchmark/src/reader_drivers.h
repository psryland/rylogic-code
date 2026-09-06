//**********************************
// Script Reader Benchmark
//  Copyright (c) Rylogic Ltd 2015
//**********************************
// Parse-and-checksum drivers for the supported workloads.
//
// Style Guidance:
//  - Each function performs exactly one full parse of its input buffer and folds every extracted
//    value through 'Checksum::Add', matching the fixture shape written by the corresponding
//    generator in 'workload_generators.h'.
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

	// Result of driving one full parse: a black-box checksum, the number of items extracted, and the input size.
	struct ParseResult
	{
		Checksum m_checksum;
		uint64_t m_items;
		uint64_t m_bytes;
	};

	// Owns the reader and any stream object that must outlive one timed parse.
	class ReaderOwner
	{
		std::unique_ptr<pr::script::reader::FileIncludeHandler> m_includes;
		std::unique_ptr<pr::mem_istream<char>> m_memory_stream;
		std::unique_ptr<std::ifstream> m_file_stream;
		std::unique_ptr<pr::script::Reader> m_reader;

	public:

		// Construct the reader through the selected memory, stream, or file source.
		explicit ReaderOwner(BenchmarkSource const& source)
		{
			// Configure UTF-8 file includes only when the fixture owns an include tree.
			if (!source.m_include_root.empty())
			{
				m_includes = std::make_unique<pr::script::reader::FileIncludeHandler>();
				m_includes->AddSearchPath(source.m_include_root);
			}

			switch (source.m_input_mode)
			{
				case EInputMode::Memory:
				{
					m_reader = std::make_unique<pr::script::Reader>(std::string_view(source.m_text), false, source.m_filepath, m_includes.get());
					break;
				}
				case EInputMode::Stream:
				{
					m_memory_stream = std::make_unique<pr::mem_istream<char>>(std::string_view(source.m_text), 0);
					m_reader = std::make_unique<pr::script::Reader>(*m_memory_stream, false, source.m_filepath, m_includes.get());
					break;
				}
				case EInputMode::File:
				{
					m_file_stream = std::make_unique<std::ifstream>(source.m_filepath, std::ios::binary);
					if (!m_file_stream->is_open())
						throw std::runtime_error("failed to open benchmark fixture: " + source.m_filepath.string());

					m_reader = std::make_unique<pr::script::Reader>(*m_file_stream, false, source.m_filepath, m_includes.get());
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

	// Scan forward to the named keyword.
	void ExpectKeyword(pr::script::Reader& reader, char const* name)
	{
		reader.Keyword(std::string_view(name));
	}

	// Number-heavy: reads the Verts/Normals/UVs/Indices sections written by 'GenerateNumberHeavy'.
	ParseResult DriveNumberHeavy(BenchmarkSource const& source, SizeParams const& p)
	{
		auto owner = ReaderOwner(source);
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
	ParseResult DriveIdentifierHeavy(BenchmarkSource const& source, SizeParams const& p)
	{
		auto owner = ReaderOwner(source);
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

	// Strings/non-ASCII: read and checksum the UTF-8 quoted-string list.
	ParseResult DriveStringsUtf8(BenchmarkSource const& source, SizeParams const& p)
	{
		auto owner = ReaderOwner(source);
		auto& reader = owner.Reader();
		Checksum cs;
		uint64_t items = 0;

		ExpectKeyword(reader, "Strings");
		reader.SectionStart();
		for (int i = 0; i != p.m_string_count; ++i)
		{
			auto string = std::string{};
			reader.String(string);
			cs.Add(std::string_view(string));
			++items;
		}
		reader.SectionEnd();

		return ParseResult{ cs, items, source.m_total_bytes };
	}

	// Directive/macro-heavy: reads the Flags/Values sections written by 'GenerateDirectiveMacroHeavy'.
	ParseResult DriveDirectiveMacroHeavy(BenchmarkSource const& source, SizeParams const& p)
	{
		constexpr int FlagToggleCount = 16; // Must match 'GenerateDirectiveMacroHeavy'.

		auto owner = ReaderOwner(source);
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
	ParseResult DriveTokenBoundaryAdversary(BenchmarkSource const& source, SizeParams const& p)
	{
		auto owner = ReaderOwner(source);
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
	ParseResult DriveIncludeTree(BenchmarkSource const& source, SizeParams const& p)
	{
		auto owner = ReaderOwner(source);
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
