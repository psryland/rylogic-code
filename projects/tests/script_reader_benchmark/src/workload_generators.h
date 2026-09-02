//**********************************
// Script Reader Benchmark
//  Copyright (c) Rylogic Ltd 2015
//**********************************
// Deterministic, memory-only fixture generators for each benchmark workload.
//
// Style Guidance:
//  - Every generator here is pure and seed-driven: given the same 'SizeParams' and seed it must
//    produce byte-identical output on every run, so the benchmark's own fixtures never introduce
//    nondeterminism into the comparison between backends.
//  - Generation happens entirely outside the timed region (see main.cpp), so these functions are
//    free to be simple and unoptimized; only the parsing side of the benchmark is measured.
//  - Output is plain LDraw-style script text (leading '*Keyword' sections, '{'/'}' blocks) built
//    directly as UTF-8 bytes, matching what both 'pr::script::Reader' and 'pr::script::v2::Reader'
//    already parse identically per the existing differential/conformance tests.
#pragma once
#include "forward.h"

namespace pr::script_bench
{
	// Which named workload to generate/drive.
	enum class EWorkload
	{
		NumberHeavy,
		IdentEmpty,
		IdentPopulated,
		Strings,
		Macros,
		Boundary,
	};

	// Fixture size selection.
	enum class ESize
	{
		Small,
		Large,
	};

	// Item counts controlling every workload's fixture size. One instance is selected per 'ESize'.
	struct SizeParams
	{
		int m_mesh_vertices;        // NumberHeavy: vertex count (each of Verts/Normals/UVs)
		int m_mesh_triangles;       // NumberHeavy: triangle count (3 indices each)
		int m_ident_count;          // IdentEmpty/IdentPopulated: identifier count
		int m_macro_populate_count; // IdentPopulated: unrelated, never-referenced macros defined up front
		int m_string_count;         // Strings: string literal count
		int m_macro_define_count;   // Macros: distinct numeric object-macros defined
		int m_macro_value_count;    // Macros: values read back, each resolved through one of the macros
		int m_boundary_item_count;  // Boundary: mixed-kind token count
	};

	// Returns the fixed size parameters for 'size'. Values are chosen so the default CLI configuration
	// (small size, default warmups/reps, all workloads) completes in roughly a minute end to end.
	inline SizeParams GetSizeParams(ESize size)
	{
		switch (size)
		{
			case ESize::Small:
			{
				return SizeParams
				{
					.m_mesh_vertices = 6000,
					.m_mesh_triangles = 4000,
					.m_ident_count = 30000,
					.m_macro_populate_count = 2000,
					.m_string_count = 12000,
					.m_macro_define_count = 64,
					.m_macro_value_count = 40000,
					.m_boundary_item_count = 15000,
				};
			}
			case ESize::Large:
			{
				return SizeParams
				{
					.m_mesh_vertices = 60000,
					.m_mesh_triangles = 40000,
					.m_ident_count = 300000,
					.m_macro_populate_count = 20000,
					.m_string_count = 120000,
					.m_macro_define_count = 128,
					.m_macro_value_count = 400000,
					.m_boundary_item_count = 150000,
				};
			}
			default:
			{
				throw std::invalid_argument("unknown ESize value");
			}
		}
	}

	// Appends a decimal integer followed by a separating space.
	inline void AppendInt(std::string& out, long long v)
	{
		char buf[32];
		auto n = std::snprintf(buf, sizeof(buf), "%lld ", v);
		out.append(buf, size_t(n));
	}

	// Appends 'v' zero-padded to 'width' digits, with no trailing separator (used to build names).
	inline void AppendPadded(std::string& out, int v, int width)
	{
		char buf[32];
		auto n = std::snprintf(buf, sizeof(buf), "%0*d", width, v);
		out.append(buf, size_t(n));
	}

	// Appends a fixed-precision real followed by a separating space.
	inline void AppendReal(std::string& out, double v)
	{
		char buf[64];
		auto n = std::snprintf(buf, sizeof(buf), "%.6f ", v);
		out.append(buf, size_t(n));
	}

	// Generates an LDraw-like mesh: dense Vector3/Vector2 vertex data plus Int index triples.
	// This is the number-heavy workload; both real and integer extraction paths dominate its cost.
	inline std::string GenerateNumberHeavy(SizeParams const& p, uint64_t seed)
	{
		std::mt19937_64 rng(seed);
		std::uniform_real_distribution<double> pos(-500.0, 500.0);
		std::uniform_real_distribution<double> unit(-1.0, 1.0);
		std::uniform_real_distribution<double> uv(0.0, 1.0);
		std::uniform_int_distribution<int> idx(0, p.m_mesh_vertices - 1);

		std::string out;
		out.reserve(size_t(p.m_mesh_vertices) * 90 + size_t(p.m_mesh_triangles) * 24 + 256);

		out += "*Mesh\n{\n\t*Verts\n\t{\n";
		for (int i = 0; i != p.m_mesh_vertices; ++i)
		{
			out += "\t\t";
			AppendReal(out, pos(rng));
			AppendReal(out, pos(rng));
			AppendReal(out, pos(rng));
			out += '\n';
		}
		out += "\t}\n\t*Normals\n\t{\n";
		for (int i = 0; i != p.m_mesh_vertices; ++i)
		{
			out += "\t\t";
			AppendReal(out, unit(rng));
			AppendReal(out, unit(rng));
			AppendReal(out, unit(rng));
			out += '\n';
		}
		out += "\t}\n\t*UVs\n\t{\n";
		for (int i = 0; i != p.m_mesh_vertices; ++i)
		{
			out += "\t\t";
			AppendReal(out, uv(rng));
			AppendReal(out, uv(rng));
			out += '\n';
		}
		out += "\t}\n\t*Indices\n\t{\n";
		for (int i = 0; i != p.m_mesh_triangles; ++i)
		{
			out += "\t\t";
			AppendInt(out, idx(rng));
			AppendInt(out, idx(rng));
			AppendInt(out, idx(rng));
			out += '\n';
		}
		out += "\t}\n}\n";
		return out;
	}

	// Generates a section of many bare identifiers, optionally preceded by a large number of unrelated,
	// never-referenced object-macro definitions. Both variants drive the same extraction sequence, so
	// the only cost difference measured is how each backend's macro lookup behaves once "populated".
	inline std::string GenerateIdentifierHeavy(SizeParams const& p, uint64_t seed, bool populated)
	{
		(void)seed; // No randomisation needed: identifier names are a plain enumerated sequence.
		std::string out;
		out.reserve(size_t(p.m_ident_count) * 12 + size_t(p.m_macro_populate_count) * 24 + 256);

		if (populated)
		{
			for (int i = 0; i != p.m_macro_populate_count; ++i)
			{
				out += "#define MDEF";
				AppendPadded(out, i, 6);
				out += ' ';
				AppendInt(out, i);
				out += '\n';
			}
		}

		out += "*Idents\n{\n";
		for (int i = 0; i != p.m_ident_count; ++i)
		{
			out += "ident";
			AppendPadded(out, i, 6);
			out += ' ';
		}
		out += "\n}\n";
		return out;
	}

	// Generates a section of quoted string literals drawn from a fixed pool of ASCII and non-ASCII
	// (accented, CJK, Cyrillic) UTF-8 phrases, exercising the string/UTF-8 extraction path.
	// Note: astral-plane characters (e.g. emoji, which require a UTF-16 surrogate pair) are deliberately
	// excluded from the pool. Legacy 'pr::script::Src' (script_core.h) decodes UTF-8 one 'char16_t' per
	// 'Read()' call via 'std::mbrtoc16' and does not loop to emit the trailing low surrogate for 4-byte
	// UTF-8 sequences, so astral characters fail to round-trip; this is a pre-existing decode limitation
	// in the legacy reader, not a bug in this benchmark or in Reader2, and is out of scope to fix here.
	inline std::string GenerateStringsUtf8(SizeParams const& p, uint64_t seed)
	{
		static char const* s_ascii[] =
		{
			"alpha", "bravo", "charlie test", "delta-echo", "foxtrot golf hotel",
		};
		static char const* s_utf8[] =
		{
			"h\xc3\xa9llo w\xc3\xb6rld",             // "héllo wörld"
			"\xe6\x97\xa5\xe6\x9c\xac\xe8\xaa\x9e",  // "日本語"
			"\xc3\x91" "and\xc3\xba caf\xc3\xa9",     // "Ñandú café"
			"\xd0\x9a\xd0\xb8\xd1\x80\xd0\xb8\xd0\xbb\xd0\xbb\xd0\xb8\xd1\x86\xd0\xb0", // "Кириллица"
			"\xe6\xb7\xb7\xe5\x90\x88 mixed ASCII 12345", // "混合 mixed ASCII 12345"
		};

		std::mt19937_64 rng(seed);
		std::uniform_int_distribution<int> pick_pool(0, 1);
		std::uniform_int_distribution<int> pick_ascii(0, int(std::size(s_ascii)) - 1);
		std::uniform_int_distribution<int> pick_utf8(0, int(std::size(s_utf8)) - 1);

		std::string out;
		out.reserve(size_t(p.m_string_count) * 32 + 64);
		out += "*Strings\n{\n";
		for (int i = 0; i != p.m_string_count; ++i)
		{
			// A comma separates each literal so consecutive quoted strings are not joined into a single
			// string by the reader's literal-string-concatenation feature (adjacent strings separated only
			// by whitespace are deliberately merged into one string, matching C/C++ string literal joining).
			auto const* s = pick_pool(rng) != 0 ? s_utf8[pick_utf8(rng)] : s_ascii[pick_ascii(rng)];
			out += '"';
			out += s;
			out += "\",\n";
		}
		out += "}\n";
		return out;
	}

	// Generates a preamble of numeric object-macros, a fixed number of '#ifdef'/'#else'/'#endif'
	// toggles each guarding one boolean flag, and a large run of values each referencing one of the
	// preamble macros by name. This is the directive/macro-heavy workload.
	inline std::string GenerateDirectiveMacroHeavy(SizeParams const& p, uint64_t seed)
	{
		constexpr int FlagToggleCount = 16; // Fixed regardless of size so item counts stay deterministic.

		std::mt19937_64 rng(seed);
		std::uniform_int_distribution<int> pick_macro(0, p.m_macro_define_count - 1);
		std::uniform_int_distribution<int> pick_bool(0, 1);

		std::string out;
		out.reserve(size_t(p.m_macro_define_count) * 24 + size_t(p.m_macro_value_count) * 14 + FlagToggleCount * 64 + 256);

		// Preamble macros: each replacement text is a single numeric literal, referenced many times below.
		for (int i = 0; i != p.m_macro_define_count; ++i)
		{
			out += "#define MVAL";
			AppendPadded(out, i, 4);
			out += ' ';
			AppendInt(out, i * 7 + 3);
			out += '\n';
		}

		// A fixed number of conditional-compilation toggles, each guarding one boolean flag value.
		out += "*Flags\n{\n";
		for (int i = 0; i != FlagToggleCount; ++i)
		{
			if (pick_bool(rng) != 0)
			{
				out += "#define TOGGLE";
				AppendPadded(out, i, 2);
				out += '\n';
			}
			out += "#ifdef TOGGLE";
			AppendPadded(out, i, 2);
			out += "\n\t*Flag true\n#else\n\t*Flag false\n#endif\n";
		}
		out += "}\n";

		// A large run of numeric values, each resolved through one of the preamble macros.
		out += "*Values\n{\n";
		for (int i = 0; i != p.m_macro_value_count; ++i)
		{
			out += "MVAL";
			AppendPadded(out, pick_macro(rng), 4);
			out += ' ';
		}
		out += "\n}\n";
		return out;
	}

	// Generates a section of mixed-kind tokens (int/real/string/identifier, cycling in that fixed
	// order) separated by line comments of randomised length. The random padding statistically spreads
	// token start offsets across the whole 4096-byte block-read boundary over enough volume; this is a
	// statistical, not exhaustive, boundary adversary (see the benchmark's reported design limitations).
	inline std::string GenerateTokenBoundaryAdversary(SizeParams const& p, uint64_t seed)
	{
		std::mt19937_64 rng(seed);
		std::uniform_int_distribution<int> pad_len(0, 137); // Deliberately not a power of two.
		std::uniform_int_distribution<int> ident_len(1, 24);
		std::uniform_int_distribution<int> az(0, 25);

		std::string out;
		out.reserve(size_t(p.m_boundary_item_count) * 48 + 256);
		out += "*Boundary\n{\n";
		for (int i = 0; i != p.m_boundary_item_count; ++i)
		{
			// Variable-length comment padding shifts every following token to a pseudo-random byte offset.
			out += "//";
			for (int c = 0, n = pad_len(rng); c != n; ++c)
				out += 'x';
			out += '\n';

			switch (i % 4)
			{
				case 0:
				{
					AppendInt(out, i * 31 - 500);
					break;
				}
				case 1:
				{
					AppendReal(out, double(i) * 0.125 - 250.0);
					break;
				}
				case 2:
				{
					out += "\"tok";
					AppendInt(out, i);
					out.pop_back(); // AppendInt adds a trailing separator space we don't want inside the quotes.
					out += "\"";
					break;
				}
				case 3:
				{
					out += "id";
					for (int c = 0, n = ident_len(rng); c != n; ++c)
						out += char('a' + az(rng));
					break;
				}
				default:
				{
					throw std::logic_error("unreachable: i % 4 is always in [0, 4)");
				}
			}
			out += '\n';
		}
		out += "}\n";
		return out;
	}
}
