//********************************
// STL CAD Model file format
//  Copyright (c) Rylogic Ltd 2018
//********************************
#pragma once
#include <string>
#include <vector>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <memory>
#include <exception>
#include <cassert>
#include "pr/common/cast.h"
#include "pr/common/fmt.h"
#include "pr/gfx/colour.h"
#include "pr/common/range.h"
#include "pr/common/scope.h"
#include "pr/math/math.h"
#include "pr/str/extract.h"
#include "pr/script/script_core.h"

namespace pr::geometry::stl
{
	// STL files come in two variants; binary and text.
	// There is poor standardisation for the format though.
	// Simply, the file should start with 'solid' if it is an ASCII file,
	// or an 80-character string header that doesn't start with 'solid' if it's
	// a binary file. SolidWorks however uses 'solid' for both.
	// I'm using this strategy:
	//   Read 80 bytes, if the header contains a '\n' character, assume text.
	//   If binary,
	//      FSeek to byte 81,
	//      Read the 4-byte little endian triangle count,
	//      Read triangles till done
	//   If ASCII,
	//      FSeek to one past the '\n' character.
	//
	//  Use binary file mode for ASCII files as well
	//
	using u8 = unsigned char;       static_assert(sizeof(u8) == 1, "1byte u8 expected");
	using u16 = unsigned short;     static_assert(sizeof(u16) == 2, "2byte u16 expected");
	using u32 = unsigned long;      static_assert(sizeof(u32) == 4, "4byte u32 expected");
	using s64 = long long;          static_assert(sizeof(s64) == 8, "8byte s64 expected");
	using u64 = unsigned long long; static_assert(sizeof(u64) == 8, "8byte u64 expected");

	enum class EFormatVariant
	{
		// Basic vendor independent STL format
		Standard = 0,

		// VisCAM, SolidView format
		// u16 flags represent a colour in R5G5B5X1 format. If X1 is 0 then the colour should be ignored.
		VisCAM = 1,
		SolidView = 2,

		// Materialise Magics format
		// u16 flags represent a colour in B5G5R5X1 format. X1 is 0 if the colour should be used for the face, 1 if the per-object colour should be used.
		MaterialiseMagics = 3,
	};

	struct Vec3
	{
		float x,y,z;
		v4 w0() const { return v4{x,y,z,0}; }
		v4 w1() const { return v4{x,y,z,1}; }
	};
	struct Facet
	{
		Vec3 norm;
		Vec3 vert[3];
		u16 flags;
	};
	struct Model
	{
		char m_header[81];        // Header string (+1 for a terminator)
		std::vector<v4>  m_verts; // Vertices
		std::vector<v4>  m_norms; // Vertex normals (one per face)
		std::vector<u16> m_flags; // Colour flags, (one per face)
	};

	// Options for parsing STL files
	struct Options
	{
		Options()
			:m_variant(EFormatVariant::Standard)
			,m_calculate_normals(false)
			,m_per_face_flags(true)
		{}
		EFormatVariant m_variant; // Which specific variant of the STL format to expect
		bool m_calculate_normals; // Calculate the normals from the triangle winding order, ignoring the normals in the source
		bool m_per_face_flags;    // True if each facet has an associated u16 for flags
	};

	// Helpers for reading/writing from/to an istream-like source
	// Specialise these for non std::istream/std::ostream's
	template <typename TSrc> struct Src
	{
		static s64  TellPos(TSrc& src)          { return static_cast<s64>(src.tellg()); }
		static bool SeekAbs(TSrc& src, s64 pos) { return static_cast<bool>(src.seekg(pos)); }

		// Read an array
		template <typename TOut> static void Read(TSrc& src, TOut* out, size_t count, bool allow_partial = false)
		{
			if (src.read(char_ptr(out), count * sizeof(TOut))) return;
			if (!allow_partial && count != 0) throw std::exception("partial read of input stream");
			if (src.eof()) src.clear();// read pass the eof sets the failbit which causes tellg to return -1
		}

		// Write an array
		template <typename TIn> static void Write(TSrc& dst, TIn const* in, size_t count)
		{
			if (dst.write(char_ptr(in), count * sizeof(TIn))) return;
			throw std::exception("partial write of output stream");
		}
	};

	// Convert a u16 to a colour using R5G5B5X1
	inline Colour32 ToColour(u16 flags, bool rgb_order)
	{
		auto R = (flags & (0b11111 <<  0)) / 31.0f;
		auto G = (flags & (0b11111 <<  5)) / 31.0f;
		auto B = (flags & (0b11111 << 10)) / 31.0f;
		return rgb_order ? Colour32(R, G, B, 1.0f) : Colour32(B, G, R, 1.0f);
	}

	// Add a facet to the model
	inline void AddFacet(Model& model, Facet const& facet, Options const& opts)
	{
		auto v0 = facet.vert[0].w1();
		auto v1 = facet.vert[1].w1();
		auto v2 = facet.vert[2].w1();
		auto n = opts.m_calculate_normals
			? Normalise(Cross(v1 - v0, v2 - v1))
			: facet.norm.w0();

		model.m_norms.push_back(n);
		model.m_verts.push_back(v0);
		model.m_verts.push_back(v1);
		model.m_verts.push_back(v2);
		if (opts.m_per_face_flags)
			model.m_flags.push_back(facet.flags);
	}

	#pragma region Read

	// Read an array
	template <typename TOut, typename TSrc> inline void Read(TSrc& src, TOut* out, size_t count)
	{
		Src<TSrc>::Read(src, out, count);
	}

	// Read a single type
	template <typename TOut, typename TSrc> inline TOut Read(TSrc& src)
	{
		TOut out;
		Read(src, &out, 1);
		return out;
	}

	// Read the model data from an STL file
	template <typename TSrc, typename TModelOut>
	void Read(TSrc& src, Options opts, TModelOut out)
	{
		// Restore the src position on return
		auto reset_stream = Scope<u64>(
			[&]{ return Src<TSrc>::TellPos(src); },
			[&](u64 start){ Src<TSrc>::SeekAbs(src, start); }
		);

		Model model = {};

		// Read the header (ensure null termination)
		Src<TSrc>::Read(src, &model.m_header[0], _countof(model.m_header) - 1, true);
		auto fpos = Src<TSrc>::TellPos(src);

		// Decide whether the model is ASCII or binary by looking for a new line character in the header
		auto new_line = std::find(&model.m_header[0], &model.m_header[0] + fpos, '\n');
		auto is_ascii = *new_line == '\n';
		if (is_ascii)
		{
			using namespace pr::str;
			opts.m_per_face_flags = false;

			// Handle <CR><LF>
			if (new_line > &model.m_header[0] && *(new_line - 1) == '\r')
				--new_line;

			// Seek to the start of the data
			auto header_length = s64(new_line - &model.m_header[0]);
			memset(&model.m_header[header_length], 0, size_t(_countof(model.m_header) - header_length));
			if (!Src<TSrc>::SeekAbs(src, header_length))
				return;

			// Create a pointer-like interface to 'src'
			struct SrcPtr
			{
				TSrc& m_src;
				char m_buf[1024];
				char* m_ptr;
				char* m_end;
				s64 m_ofs;
				
				SrcPtr(TSrc& src)
					:m_src(src)
					,m_ptr()
					,m_end()
					,m_ofs()
				{
					fill();
				}
				SrcPtr(SrcPtr const&) = delete;
				SrcPtr& operator ++()
				{
					if (m_ptr == m_end) return *this;
					if (++m_ptr == m_end) fill();
					return *this;
				}
				char operator *() const
				{
					return m_ptr != m_end ? *m_ptr : 0;
				}
				void fill()
				{
					m_ofs = Src<TSrc>::TellPos(m_src);
					Src<TSrc>::Read(m_src, &m_buf[0], _countof(m_buf), true);
					auto avail = Src<TSrc>::TellPos(m_src) - m_ofs;
					m_ptr = &m_buf[0];
					m_end = m_ptr + avail;
				}
				s64 file_offset() const
				{
					return m_ofs;
				}
			};

			// Read the triangles
			for (SrcPtr ptr(src);;)
			{
				// Read the next identifier.
				char word[32] = {};
				if (!ExtractIdentifier(word, ptr))
					break;

				if (Equal(word, "facet"))
				{
					// Extract the facet and add it to the model
					Facet facet = {};

					// Parse the facet
					if (!ExtractIdentifier(word, ptr) || !Equal(word, "normal"))
						throw new std::runtime_error(pr::FmtS("File format error. Expected 'normal' to appear after 'facet' at file offset %ll", ptr.file_offset()));

					if (!ExtractReal(facet.norm.x, ptr) ||
						!ExtractReal(facet.norm.y, ptr) ||
						!ExtractReal(facet.norm.z, ptr))
						throw new std::runtime_error(pr::FmtS("File format error. Expected 3 float values to follow 'normal' at file offset %ll", ptr.file_offset()));

					if (!ExtractIdentifier(word, ptr) || !Equal(word, "outer") ||
						!ExtractIdentifier(word, ptr) || !Equal(word, "loop"))
						throw new std::runtime_error(pr::FmtS("File format error. Expected 'outer loop' to follow the facet normal at file offset %ll", ptr.file_offset()));

					for (int k = 0; k != 3; ++k)
					{
						if (!ExtractIdentifier(word, ptr) || !Equal(word, "vertex") ||
							!ExtractReal(facet.vert[k].x, ptr) ||
							!ExtractReal(facet.vert[k].y, ptr) ||
							!ExtractReal(facet.vert[k].z, ptr))
							throw new std::runtime_error(pr::FmtS("File format error. Expected 'vectex' followed by 3 float values at file offset %ll", ptr.file_offset()));
					}

					if (!ExtractIdentifier(word, ptr) || !Equal(word, "endloop"))
						throw new std::runtime_error(pr::FmtS("File format error. Expected 'endloop' to follow the facet vertex data at file offset %ll", ptr.file_offset()));

					if (!ExtractIdentifier(word, ptr) || !Equal(word, "endfacet"))
						throw new std::runtime_error(pr::FmtS("File format error. Expected 'endfacet' at file offset %ll", ptr.file_offset()));

					// Add the facet to the model
					AddFacet(model, facet, opts);
				}
				else if(Equal(word, "endsolid"))
				{
					break;
				}
				else
				{
					throw new std::runtime_error(pr::FmtS("File format error. Unknown key word at file offset %ll", ptr.file_offset()));
				}
			}
		}
		else
		{
			// Seek to the start of the data
			if (!Src<TSrc>::SeekAbs(src, 80))
				return;

			// Read the number of facets
			auto face_count = Read<u32>(src);
			auto vert_count = face_count * 3;

			model.m_verts.reserve(vert_count);
			model.m_norms.reserve(face_count);
			model.m_flags.reserve(opts.m_per_face_flags ? face_count : 0);

			// Read the triangles
			for (u32 f = 0; f != face_count; ++f)
			{
				Facet facet = {};
				facet.norm = Read<Vec3>(src);
				facet.vert[0] = Read<Vec3>(src);
				facet.vert[1] = Read<Vec3>(src);
				facet.vert[2] = Read<Vec3>(src);
				facet.flags = opts.m_per_face_flags ? Read<u16>(src) : 0;
				AddFacet(model, facet, opts);
			}
		}

		// Output the model
		out(std::move(model));
	}

	#pragma endregion
}