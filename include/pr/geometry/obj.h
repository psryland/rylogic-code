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
//#include "pr/gfx/colour.h"
//#include "pr/common/range.h"
#include "pr/common/scope.h"
//#include "pr/math/math.h"
#include "pr/str/extract.h"
#include "pr/script/script_core.h"

namespace pr::geometry::obj
{
	// See: http://paulbourke.net/dataformats/mtl/
	// See: https://en.wikipedia.org/wiki/Wavefront_.obj_file
	// There are lots of variations of .OBJ files and I'm too lazy to try support
	// all variants. Add support for special cases as needed.

	enum class EIlluminationModel
	{
		ColourOn_AmbientOff                = 0,  // Color on and Ambient off
		ColourOn_AmbientOn                 = 1,  // Color on and Ambient on
		HighlightOn                        = 2,  // Highlight on
		ReflectionOn_RayTraceOn            = 3,  // Reflection on and Ray trace on
		TransparencyOn_RayTraceOn          = 4,  // Transparency: Glass on, Reflection: Ray trace on
		FresnelOn_RayTrace_On              = 5,  // Reflection: Fresnel on and Ray trace on
		RefractionOn_FresnelOff_RayTraceOn = 6,  // Transparency: Refraction on, Reflection: Fresnel off and Ray trace on
		RefractionOn_FresnelOn_RayTraceOn  = 7,  // Transparency: Refraction on, Reflection: Fresnel on and Ray trace on
		ReflectionOn_RayTraceOff           = 8,  // Reflection on and Ray trace off
		GlassOn_RayTraceOff                = 9,  // Transparency: Glass on, Reflection: Ray trace off
		CastShadowsOntoInvisibleSurfaces   = 10, // Casts shadows onto invisible surfaces
	};
	//struct Vec3
	//{
	//	float x,y,z;
	//	v4 w0() const { return v4{x,y,z,0}; }
	//	v4 w1() const { return v4{x,y,z,1}; }
	//};
	//struct Facet
	//{
	//	Vec3 norm;
	//	Vec3 vert[3];
	//	u16 flags;
	//};
	struct Material
	{
		std::string m_name;
		float m_alpha;
		float m_ambient[3];
		float m_diffuse[3];
		float m_specular[3];
		float m_emissive[3];
		float m_transmissive[3];
		float m_spec_power;
		float m_refraction;
		float m_sharpness;
		std::filesystem::path m_tex_ambient;
		std::filesystem::path m_tex_diffuse;
		std::filesystem::path m_tex_specular;
		std::filesystem::path m_tex_spec_power;
		std::filesystem::path m_tex_alpha;
		std::filesystem::path m_tex_bump;
		EIlluminationModel m_illum;
	};
	struct SubModel
	{
		std::string m_mat_name;
		std::vector<int> m_indices;
	};
	struct Model
	{
		std::vector <v4>       m_verts; // Vertices
		std::vector <v4>       m_norms; // Vertex normals (one per face)
		std::vector <v2>       m_uvs;   // Texture coords
		std::vector <Material> m_mats;  // Materials
	};
	static const char* Delim = " ";

	// Options for parsing OBJ files
	struct Options
	{
		Options()
		{}
	};

	// Helpers for reading/writing from/to an istream-like source
	// Specialise these for non std::istream/std::ostream's
	template <typename TSrc> struct Src
	{
		static int64_t TellPos(TSrc& src)
		{
			return static_cast<int64_t>(src.tellg());
		}
		static bool SeekAbs(TSrc& src, int64_t pos)
		{
			return static_cast<bool>(src.seekg(pos));
		}

		// Read a line up to and including the newline character. 'line' should
		// not include the newline character but it should be read from the stream.
		// Returns true if 1 or more characters are read, false if not.
		template <typename TOut> static bool Read(TSrc& src, TOut& line)
		{
			return std::getline(src, line).good();
		}

		// Write a line
		template <typename TIn> static void Write(TSrc& dst, TIn const& line)
		{
			if (dst.write(line.c_str(), line.size())) return;
			throw std::exception("partial write of output stream");
		}
	};

	//// Convert a u16 to a colour using R5G5B5X1
	//inline Colour32 ToColour(u16 flags, bool rgb_order)
	//{
	//	auto R = (flags & (0b11111 <<  0)) / 31.0f;
	//	auto G = (flags & (0b11111 <<  5)) / 31.0f;
	//	auto B = (flags & (0b11111 << 10)) / 31.0f;
	//	return rgb_order ? Colour32(R, G, B, 1.0f) : Colour32(B, G, R, 1.0f);
	//}

	#pragma region Read

	namespace impl
	{
		// Parse a colour line
		inline bool ReadColour(float(&colour)[3], char const* ptr)
		{
			// R is required. G and B are optional
			if (!str::ExtractReal(colour[0], ptr, Delim)) return false;
			if (!str::ExtractReal(colour[1], ptr, Delim)) colour[1] = colour[0];
			if (!str::ExtractReal(colour[2], ptr, Delim)) colour[2] = colour[0];
			return true;
		}

		// Parse a texture map line
		inline bool ReadTexture(std::filesystem::path& map_filepath, char const* ptr)
		{
			std::wstring fp;
			if (!str::ExtractToken(fp, ptr, Delim)) return false;
			map_filepath = fp;
			return true;
		}
	}

	// Read material definitions from an OBJ '.mtl' stream
	template <typename TSrc, typename TMatOut>
	void ReadMaterials(TSrc& src, TMatOut out)
	{
		auto mat = Material{};
		bool new_mat = false;

		std::string line, id;
		for (int line_number = 1; Src<TSrc>::Read(src, line); ++line_number)
		{
			auto ptr = line.c_str();

			// Valid lines start with identifier tags
			if (!str::ExtractIdentifier(id, ptr, Delim))
			{
				// Blank lines signal the end of a material definition
				if (new_mat && *ptr == 0)
				{
					out(mat);
					mat = Material{};
					new_mat = false;
				}
				continue;
			}

			// Parse each line of the material file
			try
			{
				switch (id[0])
				{
				case 'n':
					{
						// Start of a new material definition
						if (str::Equal(id, "newmtl"))
						{
							if (!str::ExtractToken(mat.m_name, ptr, Delim)) throw std::runtime_error("Material name not found"); // Material names can be any characters except spaces
							new_mat = true;
							continue;
						}
						break;
					}
				case 'd':
					{
						if (str::Equal(id, "d"))
						{
							if (!str::ExtractReal(mat.m_alpha, ptr, Delim)) throw std::runtime_error("Invalid 'dissolved' definition");
							continue;
						}
						break;
					}
				case 'i':
					{
						if (str::Equal(id, "illum"))
						{
							int model;
							if (!str::ExtractInt(model, 10, ptr, Delim)) throw std::runtime_error("Invalid illumination model definition");
							mat.m_illum = static_cast<EIlluminationModel>(model);
							continue;
						}
						break;
					}
				case 'm':
					{
						if (str::Equal(id, "map_Ka"))
						{
							if (!impl::ReadTexture(mat.m_tex_alpha, ptr)) throw std::runtime_error("Invalid ambient texture map definition");
							continue;
						}
						if (str::Equal(id, "map_Kd"))
						{
							if (!impl::ReadTexture(mat.m_tex_diffuse, ptr)) throw std::runtime_error("Invalid diffuse texture map definition");
							continue;
						}
						if (str::Equal(id, "map_Ks"))
						{
							if (!impl::ReadTexture(mat.m_tex_specular, ptr)) throw std::runtime_error("Invalid specular texture map definition");
							continue;
						}
						if (str::Equal(id, "map_Ns"))
						{
							if (!impl::ReadTexture(mat.m_tex_spec_power, ptr)) throw std::runtime_error("Invalid specular power texture map definition");
							continue;
						}
						if (str::Equal(id, "map_d"))
						{
							if (!impl::ReadTexture(mat.m_tex_alpha, ptr)) throw std::runtime_error("Invalid alpha map definition");
							continue;
						}
						if (str::Equal(id, "map_bump"))
						{
							if (!impl::ReadTexture(mat.m_tex_bump, ptr)) throw std::runtime_error("Invalid bump map definition");
							continue;
						}
						break;
					}
				case 's':
					{
						if (str::Equal(id, "sharpness"))
						{
							if (!str::ExtractReal(mat.m_sharpness, ptr, Delim)) throw std::runtime_error("Invalid sharpness definition");
							continue;
						}
						break;
					}
				case 'K':
					{
						// Ambient colour
						if (str::Equal(id, "Ka"))
						{
							if (!impl::ReadColour(mat.m_ambient, ptr)) throw std::runtime_error("Invalid ambient colour definition");
							continue;
						}

						// Diffuse colour
						if (str::Equal(id, "Kd"))
						{
							if (!impl::ReadColour(mat.m_diffuse, ptr)) throw std::runtime_error("Invalid diffuse colour definition");
							continue;
						}

						// Specular colour
						if (str::Equal(id, "Ks"))
						{
							if (!impl::ReadColour(mat.m_specular, ptr)) throw std::runtime_error("Invalid specular colour definition");
							continue;
						}

						// Emissive colour
						if (str::Equal(id, "Ke"))
						{
							if (!impl::ReadColour(mat.m_emissive, ptr)) throw std::runtime_error("Invalid emissive colour definition");
							continue;
						}
						break;
					}
				case 'N':
					{
						if (str::Equal(id, "Ns"))
						{
							if (!str::ExtractReal(mat.m_spec_power, ptr, Delim)) throw std::runtime_error("Invalid specular power definition");
							continue;
						}
						if (str::Equal(id, "Ni"))
						{
							if (!str::ExtractReal(mat.m_refraction, ptr, Delim)) throw std::runtime_error("Invalid optical density definition");
							continue;
						}
						break;
					}
				case 'T':
					{
						if (str::Equal(id, "Tf"))
						{
							if (!impl::ReadColour(mat.m_transmissive, ptr)) throw std::runtime_error("Invalid transmissive colour definition");
							continue;
						}
						if (str::Equal(id, "Tr"))
						{
							if (!str::ExtractReal(mat.m_alpha, ptr, Delim)) throw std::runtime_error("Invalid transparency definition");
							mat.m_alpha = 1.0f - mat.m_alpha;
							continue;
							
						}
						break;
					}
				}
				throw std::runtime_error(Fmt("Unsupported tag '%s'", id.c_str()));
			}
			catch (std::exception const& ex)
			{
				throw std::runtime_error(Fmt("%s. Line: %d", ex.what(), line_number)); 
			}
		}
	}

	// Read the model data from an OBJ stream
	template <typename TSrc, typename TModelOut>
	void Read(TSrc& src, Options opts, TModelOut out)
	{
		(void)opts;
		// OBJ files are a newline delimited list of model data.
		// Each line has the form: {tag} {data...} where {tag} is:
		//    v = vertex
		//    vn = normal
		//    vt = tex coord
		//    vp = parameter space vertex
		//    f = face
		//    l = line
		//    g = group
		//    o = object definition
		//    mtllib = material definition external filepath
		//    usemtl = use material for the following geometry
		Model model;

		std::string line, id;
		for (int line_number = 1; Src<TSrc>::Read(src, line); ++line_number)
		{
			auto ptr = line.c_str();

			// Valid lines start with identifier tags
			if (!str::ExtractIdentifier(id, ptr, Delim))
			{
				continue;
			}

			// Each 'usemtl' is the start of a new nugget

			// Parse each line of the model file
			try
			{
				switch (id[0])
				{
				case 'm':
					{
						// Material definition file
						if (str::Equal(id, "mtllib"))
						{
							try
							{
								str::AdvanceToNonDelim(ptr, Delim);

								std::ifstream mat(ptr);
								ReadMaterials(mat, [] (auto const&) {});
								continue;
							}
							catch (std::exception const& ex)
							{
								throw std::runtime_error(Fmt("%s. File: '%s'", ex.what(), ptr));
							}
						}
						break;
					}
				case 'v':
					{
						break;
					}
				case 'f':
					{
						break;
					}
				case 'g':
				case 'o':
					{
						break;
					}
				}
				throw std::runtime_error(Fmt("Unsupported tag '%s'", id.c_str()));
			}
			catch (std::exception const& ex)
			{
				throw std::runtime_error(Fmt("%s. Line %d", ex.what(), line_number));
			}
		}

		// Output the model
		out(std::move(model));
	}

	#pragma endregion
}