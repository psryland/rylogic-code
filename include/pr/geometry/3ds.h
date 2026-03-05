//********************************
// 3DS file data
//  Copyright (c) Rylogic Ltd 2014
//********************************
#pragma once
#include <string>
#include <iostream>
#include <deque>
#include <memory>
#include <stdexcept>
#include <cstdint>
#include <cassert>
#include "pr/macros/enum.h"
#include "pr/common/cast.h"
#include "pr/common/fmt.h"
#include "pr/common/scope.h"
#include "pr/gfx/colour.h"
#include "pr/maths/maths.h"
#include "pr/geometry/common.h"
#include "pr/geometry/triangle.h"

// See: http://www.the-labs.com/Blender/3DS-details.html
namespace pr::geometry::max_3ds
{
	using u8 = uint8_t;  static_assert(sizeof(u8)  == 1);
	using u16 = uint16_t; static_assert(sizeof(u16) == 2);
	using u32 = uint32_t; static_assert(sizeof(u32) == 4);
	using u64 = uint64_t; static_assert(sizeof(u64) == 8);

	#pragma region Chunk Ids
	enum class EChunkId :u16
	{
		#define PR_ENUM(x)\
		x(Null                      ,= 0x0000) /* Null chunk id */\
		x(ColorF                    ,= 0x0010) /* float red, grn, blu */\
		x(Color24                   ,= 0x0011) /* char red, grn, blu */\
		x(LinColor24                ,= 0x0012) /* char red, grn, blu */\
		x(LinColorF                 ,= 0x0013) /* float red, grn, blu */\
		x(IntPercentage             ,= 0x0030) /* short percentage */\
		x(FloatPercentage           ,= 0x0031) /* float percentage */\
		\
		/* Basic File Layout: */\
		x(Main                      ,= 0x4d4d) /* Main Chunk */\
		x(M3DVersion                ,= 0x0002) /* ├─ M3D Version */\
		x(MasterScale               ,= 0x0100) /* ├─ Master Scale */\
		x(M3DEditor                 ,= 0x3D3D) /* ├─ 3D Editor Chunk */\
		x(MeshVersion               ,= 0x3D3E) /* │  ├─ Mesh Version */\
		x(ObjectBlock               ,= 0x4000) /* │  ├─ Object Block */\
		x(TriangularMesh            ,= 0x4100) /* │  │  ├─ Triangular Mesh */\
		x(VerticesList              ,= 0x4110) /* │  │  │  ├─ Vertices List */\
		x(FacesDescription          ,= 0x4120) /* │  │  │  ├─ Faces Description */\
		x(MaterialGroup             ,= 0x4130) /* │  │  │  │  ├─ Faces using Material */\
		x(SmoothingGroupList        ,= 0x4150) /* │  │  │  │  └─ Smoothing Group List */\
		x(TexVertList               ,= 0x4140) /* │  │  │  ├─ Mapping Coordinates List */\
		x(MeshMatrix                ,= 0x4160) /* │  │  │  └─ Local Coordinates System */\
		x(Light                     ,= 0x4600) /* │  │  ├─ Light */\
		x(Spotlight                 ,= 0x4610) /* │  │  │  └─ Spotlight */\
		x(Camera                    ,= 0x4700) /* │  │  └─ Camera */\
		x(MaterialBlock             ,= 0xAFFF) /* │  └─ Material Block */\
		x(MaterialName              ,= 0xA000) /* │     ├─ Material Name */\
		x(MatAmbientColor           ,= 0xA010) /* │     ├─ Ambient Color */\
		x(MatDiffuseColor           ,= 0xA020) /* │     ├─ Diffuse Color */\
		x(MatSpecularColor          ,= 0xA030) /* │     ├─ Specular Color */\
		x(MatShininess              ,= 0xA040) /* │     ├─ Shininess Percentage */\
		x(MatShininess2             ,= 0xA041) /* │     ├─ Shininess2 Percentage */\
		x(MatShininess3             ,= 0xA042) /* │     ├─ Shininess3 Percentage */\
		x(MatTransparency           ,= 0xA050) /* │     ├─ Transparency Percentage */\
		x(TextureMap1               ,= 0xA200) /* │     ├─ Texture Map 1 (see Map sub chunks) */\
		x(SpecularMap               ,= 0xA204) /* │     ├─ Specular Map */\
		x(OpacityMap                ,= 0xA210) /* │     ├─ Opacity Map */\
		x(ReflectionMap             ,= 0xA220) /* │     ├─ Reflection Map */\
		x(BumpMap                   ,= 0xA230) /* │     └─ Bump Map */\
		x(KeyframerChunk            ,= 0xB000) /* └─ Keyframer Chunk */\
		x(MeshInformationBlock      ,= 0xB002) /*    ├─ Mesh Information Block */\
		x(SpotLightInformationBlock ,= 0xB007) /*    ├─ Spot Light Information Block */\
		x(Frames                    ,= 0xB008) /*    └─ Frames (Start and End) */\
		x(ObjectName                ,= 0xB010) /*       ├─  Object Name */\
		x(ObjectPivotPoint          ,= 0xB013) /*       ├─  Object Pivot Point */\
		x(PositionTrack             ,= 0xB020) /*       ├─  Position Track */\
		x(RotationTrack             ,= 0xB021) /*       ├─  Rotation Track */\
		x(ScaleTrack                ,= 0xB022) /*       ├─  Scale Track */\
		x(HierarchyPosition         ,= 0xB030) /*       └─  Hierarchy Position */\
		\
		/* Map sub chunks */\
		x(MapFilename               ,= 0xA300) /* Map Filename */\
		x(MapTiling                 ,= 0xA351) /* Map Tiling */\
		\
		/* Others */\
		x(MatXPFall                 ,= 0xa052) /* */\
		x(MatRefBlur                ,= 0xa053) /* */\
		x(MatSELF_ILLUM             ,= 0xA080) /* */\
		x(MatTWO_SIDE               ,= 0xA081) /* */\
		x(MatDECAL                  ,= 0xA082) /* */\
		x(MatADDITIVE               ,= 0xA083) /* */\
		x(MatSELF_ILPCT             ,= 0xA084) /* */\
		x(MatWIRE                   ,= 0xA085) /* */\
		x(MatSUPERSMP               ,= 0xA086) /* */\
		x(MatWIRESIZE               ,= 0xA087) /* */\
		x(MatFACEMAP                ,= 0xA088) /* */\
		x(MatXPFALLIN               ,= 0xA08a) /* */\
		x(MatPHONGSOFT              ,= 0xA08c) /* */\
		x(MatWIREABS                ,= 0xA08e) /* */\
		x(MatSHADING                ,= 0xA100) /* */\
		x(MAT_USE_XPFALL            ,= 0xA240) /* */\
		x(MAT_USE_REFBLUR           ,= 0xA250) /* */\
		x(MapBumpPercent            ,= 0xA252) /* */\
		x(MatACUBIC                 ,= 0xA310) /* */\
		x(MatSXP_TEXT_DATA          ,= 0xA320) /* */\
		x(MatSXP_TEXT2_DATA         ,= 0xA321) /* */\
		x(MatSXP_OPAC_DATA          ,= 0xA322) /* */\
		x(MatSXP_BUMP_DATA          ,= 0xA324) /* */\
		x(MatSXP_SPEC_DATA          ,= 0xA325) /* */\
		x(MatSXP_SHIN_DATA          ,= 0xA326) /* */\
		x(MatSXP_SELFI_DATA         ,= 0xA328) /* */\
		x(MatSXP_TEXT_MASKDATA      ,= 0xA32a) /* */\
		x(MatSXP_TEXT2_MASKDATA     ,= 0xA32c) /* */\
		x(MatSXP_OPAC_MASKDATA      ,= 0xA32e) /* */\
		x(MatSXP_BUMP_MASKDATA      ,= 0xA330) /* */\
		x(MatSXP_SPEC_MASKDATA      ,= 0xA332) /* */\
		x(MatSXP_SHIN_MASKDATA      ,= 0xA334) /* */\
		x(MatSXP_SELFI_MASKDATA     ,= 0xA336) /* */\
		x(MatSXP_REFL_MASKDATA      ,= 0xA338) /* */\
		x(MatTEX2MAP                ,= 0xA33a) /* */\
		x(MatSHINMAP                ,= 0xA33c) /* */\
		x(MatSELFIMAP               ,= 0xA33d) /* */\
		x(MatTEXMASK                ,= 0xA33e) /* */\
		x(MatTEX2MASK               ,= 0xA340) /* */\
		x(MatOPACMASK               ,= 0xA342) /* */\
		x(MatBUMPMASK               ,= 0xA344) /* */\
		x(MatSHINMASK               ,= 0xA346) /* */\
		x(MatSPECMASK               ,= 0xA348) /* */\
		x(MatSELFIMASK              ,= 0xA34a) /* */\
		x(MatReflMask               ,= 0xA34c) /* */\
		x(MatMAP_TILINGOLD          ,= 0xA350) /* */\
		x(MatMapTEXBLUR_OLD         ,= 0xA352) /* */\
		x(MatMapTEXBLUR             ,= 0xA353) /* */\
		x(MatMapUSCALE              ,= 0xA354) /* */\
		x(MatMapVSCALE              ,= 0xA356) /* */\
		x(MatMapUOFFSET             ,= 0xA358) /* */\
		x(MatMapVOFFSET             ,= 0xA35a) /* */\
		x(MatMapANG                 ,= 0xA35c) /* */\
		x(MatMapCOL1                ,= 0xA360) /* */\
		x(MatMapCOL2                ,= 0xA362) /* */\
		x(MatMapRCOL                ,= 0xA364) /* */\
		x(MatMapGCOL                ,= 0xA366) /* */\
		x(MatMapBCOL                ,= 0xA368) /* */
		PR_ENUM_MEMBERS2(PR_ENUM)
	};
	PR_ENUM_REFLECTION2(EChunkId, PR_ENUM);
	#undef PR_ENUM
	#pragma endregion

	#pragma pack(push, 1)
	struct ChunkHeader
	{
		EChunkId id;
		u32 length;
	};
	static_assert(sizeof(ChunkHeader) == 6, "Incorrect chunk header size");
	#pragma pack(pop)

	struct Texture
	{
		std::string m_filepath; // Filepath
		u16         m_tiling;   // Clamp, Wrap, etc

		Texture()
			:m_filepath()
			,m_tiling()
		{}
	};
	struct Material
	{
		std::string m_name;              // The name of the material
		Colour m_ambient;            // Object ambient colour
		Colour m_diffuse;            // Object diffuse colour
		Colour m_specular;           // Object specular colour
		std::vector<Texture> m_textures; // 

		Material()
			:m_name()
			,m_ambient(ColourBlack)
			,m_diffuse(ColourWhite)
			,m_specular(ColourZero)
			,m_textures()
		{}
	};
	struct Face
	{
		u16 m_idx[3]; // three indices per face
		u16 m_flags; // one value per face
	};
	struct MaterialGroup
	{
		std::string m_name;      // The name of the material used by this group
		std::vector<u16> m_face; // The indices of the faces that use the material
	};
	struct TriMesh
	{
		m4x4 m_o2p;
		std::vector<v3> m_vert;
		std::vector<v2> m_uv;
		std::vector<Face> m_face;
		std::vector<MaterialGroup> m_matgroup;
		std::vector<u32> m_smoothing_groups;

		TriMesh()
			:m_o2p()
			,m_vert()
			,m_uv()
			,m_face()
			,m_matgroup()
			,m_smoothing_groups()
		{}
	};
	struct Object
	{
		std::string m_name;
		TriMesh m_mesh;

		Object()
			:m_name()
			,m_mesh()
		{}
	};

	// Helpers for reading from a stream source
	// Specialise these for non std::istream's
	template <typename TSrc> struct Src
	{
		static u64  TellPos(TSrc& src)
		{
			return static_cast<u64>(src.tellg());
		}
		static bool SeekAbs(TSrc& src, u64 pos)
		{
			return static_cast<bool>(src.seekg(pos));
		}
	
		// Read an array
		template <typename TOut> static void Read(TSrc& src, TOut* out, size_t count)
		{
			if (src.read(char_ptr(out), count * sizeof(TOut))) return;
			throw std::runtime_error("partial read of input stream");
		}
	};

	namespace impl
	{
		// Read an array
		template <typename TOut, typename TSrc>
		inline void Read(TSrc& src, TOut* out, size_t count)
		{
			Src<TSrc>::Read(src, out, count);
		}

		// Read a single type
		template <typename TOut, typename TSrc>
		inline TOut Read(TSrc& src)
		{
			TOut out;
			Read(src, &out, 1);
			return out;
		}

		// Generic chunk reading function
		// 'src' should point to a sub chunk
		// 'len' is the remaining bytes in the parent chunk from 'src' to the end of the parent chunk
		// 'func' is called back with the chunk header, it should return true to stop reading.
		template <typename TSrc, typename Func>
		void ReadChunks(TSrc& src, u32 len, Func func, u32* len_out = nullptr)
		{
			while (len != 0)
			{
				auto start = Src<TSrc>::TellPos(src);

				// Read the chunk header
				auto hdr = Read<ChunkHeader>(src);
				if (hdr.length <= len) len -= hdr.length; else throw std::runtime_error(FmtS("invalid chunk found at offset 0x%X", start));
				u32 data_len = hdr.length - sizeof(ChunkHeader);

				// Parse the chunk
				if (func(hdr, src, data_len))
				{
					if (len_out) *len_out = len;
					return;
				}

				// Seek to the next chunk
				Src<TSrc>::SeekAbs(src, start + hdr.length);
			}
			if (len_out) *len_out = 0;
		}

		//// Peek at a type
		//template <typename TOut, typename TSrc>
		//inline TOut Peek(TSrc& src)
		//{
		//	TOut out = Read<TOut>(src);
		//	if (Src<TSrc>::SeekRel(src, -int(sizeof(TOut)))) return out;
		//	throw std::runtime_error("seek failed on input stream");
		//}

		// Search from the current stream position to the next instance of chunk 'id'.
		// Assumes 'src' is positioned at a chunk header within a parent chunk.
		// 'len' is the number of bytes until the end of the parent chunk.
		// 'len_out' is an output of the length until the end of the parent chunk on return.
		// If 'next' is true and 'src' currently points to an 'id' chunk, then seeks to the next instance of 'id'
		// Returns the found chunk header with the current position of 'src' set immediately after it.
		template <typename TSrc>
		ChunkHeader Find(EChunkId id, TSrc& src, u32 len, u32* len_out = nullptr, bool next = false)
		{
			ChunkHeader chunk;
			ReadChunks(src, len, [&](ChunkHeader hdr, TSrc&, u32)
			{
				// If this is the chunk we're looking for return false to say "done"
				if (hdr.id == id && !next)
				{
					chunk = hdr;
					return true;
				}
				next = false;
				return false;
			}, len_out);
			return chunk;
		}

		// Read a null terminated string from a chunk.
		// Assumes 'src' points to the start of the string.
		template <typename TSrc>
		std::string ReadCStr(TSrc& src, u32 len, u32* len_out = nullptr)
		{
			std::string str;
			for (char c; len-- != 0 && (c = Read<char>(src)) != 0;)
				str.push_back(c);
				
			if (len_out) *len_out = len;
			return str;
		}

		// Read a colour from 'src'
		// Assumes 'src' points to a colour chunk header
		template <typename TSrc>
		Colour ReadColour(TSrc& src, u32)
		{
			auto hdr = Read<ChunkHeader>(src);
			switch (hdr.id) {
			default: throw std::runtime_error(FmtS("Unknown chunk id: %4.4x. Expected a colour chunk", hdr.id));
			case EChunkId::ColorF: //float red, grn, blu;
			case EChunkId::LinColorF: //float red, grn, blu;
				{
					float rgb[3];
					Read(src, rgb, 3);
					return Colour(rgb[0], rgb[1], rgb[2], 1.0f);
				}
			case EChunkId::Color24: // char red, grn, blu;
			case EChunkId::LinColor24: // char red, grn, blu;
				{
					u8 rgb[3];
					Read(src, rgb, 3);
					return Colour(rgb[0], rgb[1], rgb[2], 255);
				}
			}
		}

		// Read a texture from 'src'
		// Assumes 'src' points to a sub chunk within a TextureMap1, BumpMap, or ReflectionMap chunk
		template <typename TSrc>
		Texture ReadTexture(TSrc& src, u32 len)
		{
			Texture tex;
			ReadChunks(src, len, [&](ChunkHeader hdr, TSrc& src, u32 data_len)
			{
				switch (hdr.id)
				{
				case EChunkId::MapFilename:
					{
						tex.m_filepath = ReadCStr(src, data_len);
						break;
					}
				case EChunkId::MapTiling:
					{
						tex.m_tiling = Read<u16>(src);
						break;
					}
				}
				return false;
			});
			return tex;
		}

		// Read a material from 'src'
		// Assumes 'src' points to a sub chunk within a MaterialBlock chunk
		template <typename TSrc>
		Material ReadMaterial(TSrc& src, u32 len)
		{
			Material mat;
			ReadChunks(src, len, [&](ChunkHeader hdr, TSrc& src, u32 data_len)
			{
				switch (hdr.id)
				{
				case EChunkId::MaterialName:
					{
						mat.m_name = ReadCStr(src, data_len);
						break;
					}
				case EChunkId::MatAmbientColor:
					{
						mat.m_ambient = ReadColour(src, data_len);
						break;
					}
				case EChunkId::MatDiffuseColor:
					{
						mat.m_diffuse = ReadColour(src, data_len);
						break;
					}
				case EChunkId::MatSpecularColor:
					{
						mat.m_specular = ReadColour(src, data_len);
						break;
					}
				case EChunkId::TextureMap1:
					{
						mat.m_textures.push_back(ReadTexture(src, data_len));
						break;
					}
				}
				return false;
			});
			return mat;
		}

		// Read a face list description from 'src'
		// Assumes 'src' points just past the FacesDescription chunk header
		template <typename TSrc>
		void ReadFaceList(TSrc& src, TriMesh& mesh, u32 len)
		{
			size_t count = Read<u16>(src);
			auto face_data_size = sizeof(u16) + count * sizeof(Face);
			if (len >= face_data_size) len -= u32(face_data_size); else throw std::runtime_error("invalid face list data");

			// Read the face indices
			mesh.m_face.resize(count);
			Read(src, mesh.m_face.data(), count);

			ReadChunks(src, len, [&](ChunkHeader hdr, TSrc& src, u32 data_len)
			{
				switch (hdr.id)
				{
				case EChunkId::MaterialGroup:
					{
						MaterialGroup mgrp;
						mgrp.m_name = ReadCStr(src, data_len, &data_len);

						auto count = Read<u16>(src);
						mgrp.m_face.resize(count);
						Read(src, mgrp.m_face.data(), count);

						mesh.m_matgroup.emplace_back(std::move(mgrp));
						break;
					}
				case EChunkId::SmoothingGroupList:
					{
						mesh.m_smoothing_groups.resize(data_len/sizeof(u32));
						Read(src, mesh.m_smoothing_groups.data(), mesh.m_smoothing_groups.size());
						break;
					}
				}
				return false;
			});
		}

		// Read a trimesh from 'src'
		// Assumes 'src' points to the first sub chunk of a tri mesh chunk
		template <typename TSrc>
		TriMesh ReadTriMesh(TSrc& src, u32 len)
		{
			TriMesh mesh;
			ReadChunks(src, len, [&](ChunkHeader hdr, TSrc& src, u32 data_len)
			{
				switch (hdr.id)
				{
				case EChunkId::VerticesList:
					{
						size_t count = Read<u16>(src);
						mesh.m_vert.resize(count);
						Read(src, mesh.m_vert.data(), count);
						break;
					}
				case EChunkId::TexVertList:
					{
						size_t count = Read<u16>(src);
						mesh.m_uv.resize(count);
						Read(src, mesh.m_uv.data(), count);
						break;
					}
				case EChunkId::MeshMatrix:
					{
						float matrix[4][3];
						Read(src, &matrix, 1);
						mesh.m_o2p.x = v4(matrix[0], 0.0f);
						mesh.m_o2p.y = v4(matrix[1], 0.0f);
						mesh.m_o2p.z = v4(matrix[2], 0.0f);
						mesh.m_o2p.w = v4(matrix[3], 1.0f);
						break;
					}
				case EChunkId::FacesDescription:
					{
						ReadFaceList(src, mesh, data_len);
						break;
					}
				}
				return false;
			});
			return mesh;
		}

		// Read an object from 'src'
		template <typename TSrc>
		Object ReadObject(TSrc& src, u32 len)
		{
			Object obj;
			obj.m_name = ReadCStr(src, len, &len);
			ReadChunks(src, len, [&](ChunkHeader hdr, TSrc& src, u32 data_len)
			{
				switch (hdr.id)
				{
				case EChunkId::TriangularMesh:
					{
						obj.m_mesh = ReadTriMesh(src, data_len);
						break;
					}
				}
				return false;
			});
			return obj;
		}
	}

	// Extract the materials in the given 3DS stream
	// 'mat_out' should return true to stop searching (i.e. material found!)
	template <typename TSrc, typename TMatObj> void ReadMaterials(TSrc& src, TMatObj mat_out)
	{
		using namespace pr::geometry::max_3ds;

		// Restore the src position on return because typically we read the file in two
		// passes. It's convenient to allow the caller to use the same source.
		auto start = Src<TSrc>::TellPos(src);
		auto reset_stream = Scope<void>([]{}, [&]{ Src<TSrc>::SeekAbs(src, start); });

		// Check that this is actually a 3DS stream
		auto main = impl::Read<ChunkHeader>(src);
		if (main.id != EChunkId::Main)
			throw std::runtime_error("Source is not a 3ds stream");

		// Find the M3DEditor sub chunk
		auto editor = impl::Find(EChunkId::M3DEditor, src, main.length - sizeof(ChunkHeader));
		if (editor.id != EChunkId::M3DEditor)
			return; // Source contains no editor data

		// Read the materials
		impl::ReadChunks(src, editor.length - sizeof(ChunkHeader), [&](ChunkHeader hdr, TSrc& src, u32 data_len)
		{
			switch (hdr.id)
			{
			case EChunkId::MaterialBlock:
				if (mat_out(impl::ReadMaterial(src, data_len)))
					return true;
				break;
			}
			return false;
		});
	}

	// Extract the objects from a 3DS stream
	// 'obj_out' should return true to stop searching (i.e. object found)
	template <typename TSrc, typename TObjOut> void ReadObjects(TSrc& src, TObjOut obj_out)
	{
		using namespace pr::geometry::max_3ds;

		// Restore the src position on return
		auto start = Src<TSrc>::TellPos(src);
		auto reset_stream = Scope<void>([]{}, [&]{ Src<TSrc>::SeekAbs(src, start); });

		// Check that this is actually a 3DS stream
		auto main = impl::Read<ChunkHeader>(src);
		if (main.id != EChunkId::Main)
			throw std::runtime_error("Source is not a 3ds model");

		// Find the M3DEditor sub chunk
		auto editor = impl::Find(EChunkId::M3DEditor, src, main.length - sizeof(ChunkHeader));
		if (editor.id != EChunkId::M3DEditor)
			return; // Source contains no editor data

		// Read the objects
		impl::ReadChunks(src, editor.length - sizeof(ChunkHeader), [&](ChunkHeader hdr, TSrc& src, u32 data_len)
		{
			switch (hdr.id)
			{
			case EChunkId::ObjectBlock:
				if (obj_out(impl::ReadObject(src, data_len)))
					return true;
				break;
			}
			return false;
		});
	}

	// Given a 3DS model object, generate verts/indices for a renderer model
	// 'TOut' is an object with callback methods for received model data.
	template <typename TMatLookup, typename VOut, typename IOut, typename NOut>
	void CreateModel(max_3ds::Object const& obj, TMatLookup mats, VOut vout, IOut iout, NOut nout)
	{
		// Validate 'obj'
		if (!obj.m_mesh.m_uv.empty() && obj.m_mesh.m_vert.size() != obj.m_mesh.m_uv.size())
			throw std::runtime_error("invalid 3DS object. Number of UVs != number of verts");
		if (obj.m_mesh.m_face.size() != obj.m_mesh.m_smoothing_groups.size())
			throw std::runtime_error("invalid 3DS object. Number of faces != number of smoothing groups");

		// Can't just output the verts directly.
		// In a max model verts can have multiple normals.
		// Create one of these 'Verts' per unique model vert.
		struct Vert
		{
			v4       m_norm;       // The accumulated vertex normal
			Colour   m_col;        // The material colour for this vert
			uint32_t m_smooth;     // The smoothing group bits
			Vert*    m_next;       // Another copy of this vert with different smoothing group
			uint16_t m_orig_index; // The index into the original obj.m_mesh.m_vert container
			uint16_t m_new_index;  // The index of this vert in the 'verts' container

			Vert(uint16_t orig_index, uint16_t new_index, v4 norm, Colour const& col, uint32_t sg)
				:m_norm(norm)
				,m_col(col)
				,m_smooth(sg)
				,m_next()
				,m_orig_index(orig_index)
				,m_new_index(new_index)
			{}
		};
		struct Verts
		{
			std::deque<Vert, pr::aligned_alloc<Vert>> m_data;

			auto begin() const
			{
				return m_data.begin();
			}
			auto end() const
			{
				return m_data.end();
			}
			void push_back(Vert&& vert)
			{
				m_data.emplace_back(std::move(vert));
			}

			// Returns the new index for the vert in 'cont'
			uint16_t add(uint16_t idx, v4 norm, Colour const& col, uint32_t sg)
			{
				auto& vert = m_data.at(idx);

				// If the smoothing group intersects, accumulate 'norm'
				// and return the vertex index of this vert
				if ((sg == 0 && vert.m_smooth == 0) || (sg & vert.m_smooth) != 0 || (vert.m_norm == v4::Zero()))
				{
					vert.m_norm += norm;
					vert.m_col = col;
					vert.m_smooth |= sg;
					return vert.m_new_index;
				}

				// Otherwise if we have a 'next' try that vert
				if (vert.m_next != nullptr)
					return add(vert.m_next->m_new_index, norm, col, sg);

				// Otherwise, create a new Vert and add it to the linked list
				auto new_index = s_cast<uint16_t>(m_data.size());
				m_data.emplace_back(vert.m_orig_index, new_index, norm, col, sg);
				vert.m_next = &m_data.back();
				return new_index;
			}
		};
		Verts verts;

		// Initialise 'verts'
		for (uint16_t i = 0, iend = s_cast<uint16_t>(obj.m_mesh.m_vert.size()); i != iend; ++i)
			verts.push_back(Vert(i, i, v4::Zero(), ColourWhite, 0));

		// Loop over material groups, each material group is a nugget
		auto vrange = Range<size_t>::Zero();
		auto irange = Range<size_t>::Zero();
		for (auto const& mgrp : obj.m_mesh.m_matgroup)
		{
			// Ignore material groups that aren't used in the model
			if (mgrp.m_face.empty())
				continue;

			// Find the material
			auto const& mat = mats(mgrp.m_name);
			auto topo = ETopo::TriList;
			auto geom = EGeom::Vert | EGeom::Colr | EGeom::Norm | (!mat.m_textures.empty() ? EGeom::Tex0 : EGeom::None);

			// Write out each face that belongs to this group
			irange.m_beg = irange.m_end;
			vrange = Range<size_t>::Reset();
			for (auto const& face_idx : mgrp.m_face)
			{
				// Get the face and it's smoothing group
				auto const& face = obj.m_mesh.m_face[face_idx];
				auto sg = obj.m_mesh.m_smoothing_groups[face_idx];

				// Calculate the weighted normal for this face at each vertex
				// Reason weights are needed: consider the (+x,+y,+z) corner of a box, the normal should point out along (1,1,1)
				// but if one box face has two triangles, while the others have one, this wouldn't be true without weight values.
				// Could use face area as the weight, it's cheaper but doesn't quite give the correct result.
				auto v0 = obj.m_mesh.m_vert[face.m_idx[0]].w1();
				auto v1 = obj.m_mesh.m_vert[face.m_idx[1]].w1();
				auto v2 = obj.m_mesh.m_vert[face.m_idx[2]].w1();
				auto e0 = v1 - v0;
				auto e1 = v2 - v1;
				auto cx = Cross(e0, e1);
				auto norm = Normalise(cx, v4::Zero());
				auto angles = TriangleAngles(v0, v1, v2);

				// Get the final vertex indices for the face
				auto i0 = verts.add(face.m_idx[0], angles.x * norm, mat.m_diffuse, sg);
				auto i1 = verts.add(face.m_idx[1], angles.y * norm, mat.m_diffuse, sg);
				auto i2 = verts.add(face.m_idx[2], angles.z * norm, mat.m_diffuse, sg);

				vrange.grow(i0);
				vrange.grow(i1);
				vrange.grow(i2);
				irange.m_end += 3;

				// Write out face indices
				iout(i0, i1, i2);
			}

			// Output a nugget for this material group
			nout(topo, geom, mat, vrange, irange);
		}

		// Write out the verts including their normals
		for (auto const& vert : verts)
		{
			auto p = obj.m_mesh.m_vert[vert.m_orig_index].w1();
			auto c = vert.m_col;
			auto n = Normalise(vert.m_norm, v4::Zero());
			auto t = !obj.m_mesh.m_uv.empty() ? obj.m_mesh.m_uv[vert.m_orig_index] : v2Zero;
			vout(p, c, n, t);
		}
	}
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::geometry
{
	PRUnitTest(Geometry3dsTests)
	{
		//using namespace pr::geometry;
		//std::ifstream ifile("\\dump\\test2.3ds", std::ifstream::binary);
		//max_3ds::ReadMaterials(ifile, [](max_3ds::Material&&){ return false; });
		//max_3ds::ReadObjects(ifile, [](max_3ds::Object&&){ return false; });
	}
}
#endif
