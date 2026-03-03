//********************************
// PR3D Model file format
//  Copyright (c) Rylogic Ltd 2014
//********************************
// Binary 3d model format
#pragma once
#include <cstdint>
#include <string>
#include <vector>
#include <iostream>
#include <memory>
#include <exception>
#include <cassert>
#include "pr/macros/enum.h"
#include "pr/common/cast.h"
#include "pr/common/fmt.h"
#include "pr/common/scope.h"
#include "pr/common/compress.h"
#include "pr/common/range.h"
#include "pr/maths/maths.h"
#include "pr/gfx/colour.h"
#include "pr/geometry/common.h"
#include "pr/geometry/index_buffer.h"
#include "pr/geometry/utility.h"
#include "pr/container/byte_data.h"

namespace pr::geometry::p3d
{
	// Notes:
	//  - The original design goal for P3D was load speed. Models where stored in a format that could be
	//    directly 'memcpy'd into gfx memory. This results in very large p3d files, however, and doesn't give
	//    much load speed benefit. The idea was that models could be zipped if compression was needed. Data
	//    aware compression is way better than zip, and unzip is pretty slow so the original idea didn't work
	//    that well.
	//  - In this version, the binary format is small on disk, but easily map-able to renderer models. There
	//    is also options for highly effective data-aware compression.
	//  - The in-memory version is decompressed but still fairly memory efficient.
	//  - This means 'memcpy' can't be used to initialise a DX model.
	//  - To examine a file without fully loading all the data, use a 'p3d::ChunkIndex'.
	//  - Use order: Vert, Colour, Norm, UV for consistency.
	//
	// Format:
	//  A mesh is separated into vertex data and index data. Each nugget is a collection of faces that use one
	//  material. Vertex data is stratified into positions, colours, normals, and texture coords. The buffers
	//  can have any length, but it is assumed indices are shared across all buffers. The C,N,T buffers use mod
	//  to produce values up to the length of the positions buffer. Typically the lengths of the C,N,T buffers
	//  will be N, 1, or 0, where 'N' is the length of the vertex position buffer.
	//    i.e. This means a mesh cannot have some verts with normals and some without. Either all verts have
	//    normals or none.
	//  Although there will be some redundancy with vertex position data, it's the only option for fast loading.
	//
	// TODO:
	//  - Add structures for animation data (skeletons, skinning weights, and bone-tracks)

	struct ChunkHeader;
	struct ChunkIndex;
	static constexpr uint32_t Version = 0x00010101U;
	static constexpr uint32_t NoIndex = ~0U;
	ChunkIndex const& NullChunk();

	enum class EChunkId
	{
		#define PR_ENUM(x)\
		x(Null                  ,= 0x00000000)/* Null chunk                                                                                      */\
		x(Str                   ,= 0x00000001)/* utf-8 string (u32 length, length * [u8])                                                        */\
		x(Main                  ,= 0x44335250)/* PR3D File type indicator                                                                        */\
		x(FileVersion           ,= 0x00000100)/* ├─ File Version                                                                                 */\
		x(Scene                 ,= 0x00001000)/* └─ Scene                                                                                        */\
		x(Materials             ,= 0x00002000)/*    ├─ Materials                                                                                 */\
		x(Material              ,= 0x00002100)/*    │  └─ Material                                                                               */\
		x(DiffuseColour         ,= 0x00002110)/*    │     ├─ Diffuse Colour                                                                      */\
		x(Texture               ,= 0x00002120)/*    │     └─ Texture (Str filepath, u8 type, u8 addr_mode, u16 flags)                            */\
		x(Meshes                ,= 0x00003000)/*    └─ Meshes                                                                                    */\
		x(Mesh                  ,= 0x00003100)/*       ├─ Mesh (can be nested)                                                                   */\
		x(MeshName              ,= 0x00003101)/*       │  ├─ Name (cstr)                                                                         */\
		x(MeshBBox              ,= 0x00003102)/*       │  ├─ Bounding box (BBox)                                                                 */\
		x(MeshTransform         ,= 0x00003103)/*       │  ├─ Mesh to Parent Transform (m4x4)                                                     */\
		x(MeshVerts             ,= 0x00003300)/*       │  ├─ Vertex positions (u32 count, u16 format, u16 stride, count * [stride])              */\
		x(MeshNorms             ,= 0x00003310)/*       │  ├─ Vertex normals   (u32 count, u16 format, u16 stride, count * [stride])              */\
		x(MeshColours           ,= 0x00003320)/*       │  ├─ Vertex colours   (u32 count, u16 format, u16 stride, count * [stride])              */\
		x(MeshUVs               ,= 0x00003330)/*       │  ├─ Vertex UVs       (u32 count, u16 format, u16 stride, count * [float2])              */\
		x(MeshNugget            ,= 0x00004000)/*       │  └─ Nugget (topo, geom)                                                                 */\
		x(MeshMatId             ,= 0x00004001)/*       │     ├─ Material id (cstr)                                                               */\
		x(MeshVIdx              ,= 0x00004010)/*       │     └─ Vert indices   (u32 count, u8 format, u8 idx_flags, u16 stride, count * [stride] */\
		x(MeshInstance          ,= 0x00003050)/*       └─ MeshInstance (can be nested), contains mesh name, o2p transform chunk                  */
		PR_ENUM_MEMBERS2(PR_ENUM)
	};
	PR_ENUM_REFLECTION2(EChunkId, PR_ENUM);
	static_assert(sizeof(EChunkId) == sizeof(uint32_t), "Chunk Ids must be 4 bytes");
	#undef PR_ENUM

	#pragma region Flags
	namespace Flags
	{
		constexpr int VertsOfs = 0;
		constexpr int NormsOfs = 4;
		constexpr int ColoursOfs = 8;
		constexpr int UVsOfs = 12;
		constexpr int IndexOfs = 16;
		constexpr uint32_t Mask = 0b1111;
	}
	enum class EVertFormat
	{
		// Use 32-bit floats for position data (default).
		// Size/Vert = 12 bytes (float[3])
		Verts32Bit = 0,

		// Use 16-bit floats for position data.
		// Size/Vert = 6 bytes (half_t[3])
		Verts16Bit = 1,
	};
	enum class ENormFormat
	{
		// Use 32-bit floats for normal data (default)
		// Size/Norm = 12 bytes (float[3])
		Norms32Bit = 0,

		// Use 16-bit floats for normal data
		// Size/Norm = 6 bytes (half[3])
		Norms16Bit = 1,

		// Pack each normal into 32bits. 
		// Size/Norm = 4 bytes (uint32_t)
		NormsPack32 = 2,

	};
	enum class EColourFormat
	{
		// Use 32-bit AARRGGBB colours (default)
		// Size/Colour = 4 bytes (uint32_t)
		Colours32Bit = 0,
	};
	enum class EUVFormat
	{
		// Use 32-bit floats for UV data
		// Size/UV = 8 bytes (float[2])
		UVs32Bit = 0,

		// Use 16-bit floats for UV data
		// Size/UV = 4 bytes (half[2])
		UVs16Bit = 1,
	};
	enum class EIndexFormat
	{
		// Don't convert indices, use the input stride
		IdxSrc = 0,

		// Use 32-bit integers for index data
		Idx32Bit = 1,

		// Use 16-bit integers for index data
		Idx16Bit = 2,

		// Use 8-bit integers for index data
		Idx8Bit = 3,

		// Use variable length integers for index data
		IdxNBit = 4,
	};
	enum class EFlags :uint32_t
	{
		None = 0,

		// Vertex flags
		Verts32Bit = s_cast<int>(EVertFormat::Verts32Bit) << Flags::VertsOfs,
		Verts16Bit = s_cast<int>(EVertFormat::Verts16Bit) << Flags::VertsOfs,

		// Normals flags
		Norms32Bit = s_cast<int>(ENormFormat::Norms32Bit) << Flags::NormsOfs,
		Norms16Bit = s_cast<int>(ENormFormat::Norms16Bit) << Flags::NormsOfs,
		NormsPack32 = s_cast<int>(ENormFormat::NormsPack32) << Flags::NormsOfs,

		// Colours flags
		Colours32Bit = s_cast<int>(EColourFormat::Colours32Bit) << Flags::ColoursOfs,

		// TexCoord flags
		UVs32Bit = s_cast<int>(EUVFormat::UVs32Bit) << Flags::UVsOfs,
		UVs16Bit = s_cast<int>(EUVFormat::UVs16Bit) << Flags::UVsOfs,

		// Index data flags
		IdxSrc = s_cast<int>(EIndexFormat::IdxSrc) << Flags::IndexOfs,
		Idx32Bit = s_cast<int>(EIndexFormat::Idx32Bit) << Flags::IndexOfs,
		Idx16Bit = s_cast<int>(EIndexFormat::Idx16Bit) << Flags::IndexOfs,
		Idx8Bit = s_cast<int>(EIndexFormat::Idx8Bit) << Flags::IndexOfs,
		IdxNBit = s_cast<int>(EIndexFormat::IdxNBit) << Flags::IndexOfs,

		// Standard combinations
		Default = Verts32Bit | Norms32Bit | Colours32Bit | UVs32Bit | IdxSrc,
		Compressed1 = Verts32Bit | Norms16Bit | Colours32Bit | UVs16Bit | Idx16Bit,
		CompressedMax = Verts16Bit | NormsPack32 | Colours32Bit | UVs16Bit | IdxNBit,

		_flags_enum = 0,
	};
	#pragma endregion

	#pragma region Support
	template <typename T, typename Base> struct Cont :Base
	{
		// Notes:
		//  - Simple container with moduluo operator [] and a default value when empty.
		//  - Saves having to test for count != 0 when accessing contents.

		std::vector<T> m_cont;
		Cont()
			:m_cont()
		{}
		explicit Cont(size_t initial_size)
			:m_cont(initial_size)
		{}
		size_t size() const
		{
			return m_cont.size();
		}
		void reserve(size_t count)
		{
			m_cont.reserve(s_cast<size_t>(count));
		}
		void resize(size_t count)
		{
			resize(count, T{});
		}
		void resize(size_t count, T item)
		{
			m_cont.resize(count, item);
		}
		void assign(std::initializer_list<T> list)
		{
			m_cont.assign(list);
		}
		void push_back(T const& t)
		{
			m_cont.push_back(t);
		}
		//void emplace_back(T&& t)
		//{
		//	m_cont.emplace_back(std::forward<T>(t));
		//}
		template<class... Args> void emplace_back(Args&&... args)
		{
			m_cont.emplace_back(std::forward<Args>(args)...);
		}
		T const& back() const
		{
			if (size() == 0) return Base::Default;
			return m_cont.back();
		}
		T const& operator[](int i) const
		{
			if (size() == 0) return Base::Default;
			return m_cont[i % size()];
		}
		T const* data() const
		{
			return m_cont.data();
		}
		T const* begin() const
		{
			return m_cont.data();
		}
		T const* end() const
		{
			return m_cont.data() + m_cont.size();
		}
		T& back()
		{
			if (size() == 0) throw std::runtime_error("container is empty");
			return m_cont.back();
		}
		T& operator [] (int i)
		{
			if (size() == 0) throw std::runtime_error("container is empty"); // you need to use a const& to the container
			return m_cont[i % size()];
		}
		T* data()
		{
			return m_cont.data();
		}
		T* begin()
		{
			return m_cont.data();
		}
		T* end()
		{
			return m_cont.data() + m_cont.size();
		}
	};
	struct VBase { static constexpr v4 Default = v4::Origin(); };
	struct CBase { static constexpr Colour32 Default = Colour32White; };
	struct NBase { static constexpr v4 Default = v4::Zero(); };
	struct TBase { static constexpr v2 Default = v2::Zero(); };
	using VCont = Cont<v4, VBase>;
	using CCont = Cont<Colour32, CBase>;
	using NCont = Cont<v4, NBase>;
	using TCont = Cont<v2, TBase>;
	using IdxBuf = pr::geometry::IdxBuf;

	struct Str16;
	struct Nugget;
	struct Mesh;
	struct Material;
	struct Bone;
	using StrCont = std::vector<Str16>;
	using BoneCont = std::vector<Bone>;
	using Nuggets = std::vector<Nugget>;
	using MatCont = std::vector<Material>;
	using MeshCont = std::vector<Mesh>;
	#pragma endregion

	#pragma region P3D File
	struct FatVert
	{
		// Notes:
		//  - This vertex is intended to be compatible with pr::rdr::Vert.
		v4     m_vert;
		Colour m_diff;
		v4     m_norm;
		v2     m_tex0;
		v2     pad;

		FatVert() = default;
		FatVert(v4 p, Colour c, v4 n, v2 t)
			: m_vert(p)
			, m_diff(c)
			, m_norm(n)
			, m_tex0(t)
			, pad()
		{}

		friend v4     GetP(FatVert const& vert) { return vert.m_vert; }
		friend Colour GetC(FatVert const& vert) { return vert.m_diff; }
		friend v4     GetN(FatVert const& vert) { return vert.m_norm; }
		friend v2     GetT(FatVert const& vert) { return vert.m_tex0; }
	};
	struct Str16
	{
		char str[16];

		Str16(std::string_view s = {})
			:str()
		{
			*this = s;
		}
		Str16& operator = (std::string_view s)
		{
			memset(str, 0, sizeof(str));
			memcpy(&str[0], s.data(), std::min(s.size(), sizeof(str)));
			return *this;
		}
		operator std::string_view() const
		{
			auto slen = strnlen(&str[0], _countof(str));
			return std::string_view(&str[0], slen);
		}
		friend bool operator == (Str16 const& lhs, Str16 const& rhs)
		{
			return strncmp(lhs.str, rhs.str, sizeof(Str16)) == 0;
		}
		friend bool operator != (Str16 const& lhs, Str16 const& rhs)
		{
			return !(lhs == rhs);
		}
	};
	struct Texture
	{
		enum class EType
		{
			Unknown       = 0,
			Diffuse       = 1, // Diffuse colour per texel
			AlphaMap      = 2, // Transparency per texel
			ReflectionMap = 3, // Reflectivity per texel
			NormalMap     = 4, // Surface normal per texel (tangent space)
			Bump          = 5, // Scalar displacement per texel
			Displacement  = 6, // Vec3 displacement per texel
		};
		enum class EAddrMode // D3D11_TEXTURE_ADDRESS_MODE
		{
			Wrap       = 1, // D3D11_TEXTURE_ADDRESS_WRAP
			Mirror     = 2, // D3D11_TEXTURE_ADDRESS_MIRROR
			Clamp      = 3, // D3D11_TEXTURE_ADDRESS_CLAMP
			Border     = 4, // D3D11_TEXTURE_ADDRESS_BORDER
			MirrorOnce = 5, // D3D11_TEXTURE_ADDRESS_MIRROR_ONCE
		};
		enum class EFlags
		{
			None = 0,
			Alpha = 1 << 0,
			_flags_enum = 0,
		};

		// UTF-8 filepath or string identifier for looking up the texture
		std::string m_filepath;

		// Texture type
		EType m_type;

		// How the texture is to be mapped
		EAddrMode m_addr_mode;

		// Texture boolean properties
		EFlags m_flags;

		explicit Texture(std::string_view filepath = {}, EType type = EType::Diffuse, EAddrMode addr = EAddrMode::Wrap, EFlags flags = EFlags::None)
			:m_filepath(filepath)
			,m_type(type)
			,m_addr_mode(addr)
			,m_flags(flags)
		{}
	};
	struct Material
	{
		using TexCont = std::vector<Texture>;

		// A unique name/guid for the material.
		// The id is always 16 bytes, pad with zeros if you
		// use a string rather than a guid
		Str16 m_id;

		// Object diffuse colour
		Colour m_diffuse;

		// Diffuse textures
		TexCont m_textures;

		Material() = default;
		Material(std::string_view name, Colour const& diff_colour)
			:m_id(name)
			,m_diffuse(diff_colour)
			,m_textures()
		{}
	};
	struct Bone
	{
		m4x4 m_o2p;
	};
	struct Skeleton
	{
		// A tree of named bones
		BoneCont m_bones;
		StrCont m_names;
	};
	struct Rig
	{
		// Groups of verts that are attached to bones
	};
	struct Nugget
	{
		// Notes:
		//  - 'm_mat' is a string id not an index because ids are more reliable if the
		//     model is modified. Indices need fixing if materials get added/removed.
		//  - 'm_vidx' is the main face data. The other buffers, that aren't empty,
		//     are repeated modulo to generate the same number of indices as 'm_vidx'
		ETopo m_topo;  // Geometry topology
		EGeom m_geom;  // Geometry valid data
		Str16 m_mat;   // Material id
		IdxBuf m_vidx; // Vertex indices for faces/lines/points/tetras/etc

		// Constructor
		Nugget() = default;
		Nugget(ETopo topo, EGeom geom, std::string_view mat_id = {}, int idx_stride = sizeof(uint32_t))
			:m_topo(topo)
			,m_geom(geom)
			,m_mat(mat_id)
			,m_vidx(idx_stride)
		{}

		// The number of indices in the nugget
		size_t icount() const
		{
			return m_vidx.size();
		}

		// The stride of the contained indices
		int stride() const
		{
			return m_vidx.stride();
		}

		// Vertex/Index range
		Range<int> vrange() const
		{
			Range<int> r = Range<int>::Reset();
			for (auto idx : m_vidx.span_as<int>())
				r.grow(idx);

			return r;
		}
		Range<int> irange() const
		{
			return Range<int>{0, isize(m_vidx)};
		}

		// Iteration access to the nugget indices
		template <typename TOut = uint32_t> auto indices() const
		{
			return m_vidx.span_as<TOut>();
		}
	};
	struct Mesh
	{
		// Notes:
		//  - A complex model consists of multiple meshes (e.g. a car would have separate meshes
		//    for the body and the wheels (instances?))
		//  - A mesh can contain 1 or more nuggets which are divisions based on material (e.g. the
		//    wheel would have a nugget for the rim, and a nugget for the rubber tyre)
		//  - Nuggets don't need to all have the same topology.
		//  - There is only one transform per mesh. Nuggets don't have transforms.
		//  - The bounding box encloses the mesh. Nuggets don't have bounding boxes.
		struct fat_vert_iter_t
		{
			using proxy = struct proxy_ { FatVert x; FatVert const* operator -> () const { return &x; } };

			Mesh const* m_mesh;
			int m_idx;

			fat_vert_iter_t(Mesh const* mesh, int idx)
				:m_mesh(mesh)
				,m_idx(idx)
			{}
			FatVert operator *() const
			{
				return FatVert
				{
					m_mesh->m_vert[m_idx],
					Colour(m_mesh->m_diff[m_idx]),
					m_mesh->m_norm[m_idx],
					m_mesh->m_tex0[m_idx]
				};
			}
			proxy operator ->() const
			{
				return proxy{**this};
			}
			fat_vert_iter_t& operator ++()
			{
				++m_idx;
				return *this;
			}
			friend bool operator == (fat_vert_iter_t const& lhs, fat_vert_iter_t const& rhs)
			{
				return lhs.m_mesh == rhs.m_mesh && lhs.m_idx == rhs.m_idx;
			}
			friend bool operator != (fat_vert_iter_t const& lhs, fat_vert_iter_t const& rhs)
			{
				return !(lhs == rhs);
			}
			friend bool operator < (fat_vert_iter_t const& lhs, fat_vert_iter_t const& rhs)
			{
				if (lhs.m_mesh != rhs.m_mesh) throw std::runtime_error("Iterators are not from the same mesh");
				return lhs.m_idx < rhs.m_idx;
			}
		};
		struct fat_vert_span_t
		{
			Mesh const* m_mesh;
			fat_vert_span_t(Mesh const* mesh)
				:m_mesh(mesh)
			{}

			// Iterate over vertex indices
			fat_vert_iter_t begin() const
			{
				return fat_vert_iter_t{m_mesh, 0};
			}
			fat_vert_iter_t end() const
			{
				return fat_vert_iter_t{m_mesh, s_cast<int>(m_mesh->vcount())};
			}
		};

		// A name for the model
		std::string m_name;

		// Vertex data
		//  - The nuggets contain indices into the 'm_verts' buffer.
		//  - The same index is also used to access the C,N,T buffers using modulus if needed.
		VCont m_vert;
		CCont m_diff;
		NCont m_norm;
		TCont m_tex0;

		// Index data
		Nuggets m_nugget;

		// Mesh bounding box
		BBox m_bbox;

		// Mesh to parent transform
		m4x4 m_o2p;

		// Child meshes
		MeshCont m_children;

		// Construct
		explicit Mesh(std::string_view name = {})
			:m_name(name)
			,m_vert()
			,m_diff()
			,m_norm()
			,m_tex0()
			,m_nugget()
			,m_bbox(BBox::Reset())
			,m_o2p(m4x4::Identity())
		{}

		// The length of the vertex buffer
		size_t vcount() const
		{
			return m_vert.size();
		}

		// The sum of indices of all nuggets
		size_t icount() const
		{
			size_t count = 0;
			for (auto& nug : nuggets()) count += nug.icount();
			return count;
		}

		// The number of nuggets in the mesh
		size_t ncount() const
		{
			return m_nugget.size();
		}

		// The vertex data geometry type. Nuggets can have geometry types with less bits than this
		EGeom geom() const
		{
			// Even if the diff, norm, and tex0 buffers do not have the same number
			// of elements as 'm_vert', the accessor uses modulo index which has the
			// effect of looking like full geometry data.
			return
				(m_vert.size() != 0 ? EGeom::Vert : EGeom::None) |
				(m_diff.size() != 0 ? EGeom::Colr : EGeom::None) |
				(m_norm.size() != 0 ? EGeom::Norm : EGeom::None) |
				(m_tex0.size() != 0 ? EGeom::Tex0 : EGeom::None);
		}

		// Iteration access to the nuggets
		Nuggets const& nuggets() const
		{
			return m_nugget;
		}

		// Iteration access to the verts as 'fat verts'
		fat_vert_span_t fat_verts() const
		{
			return fat_vert_span_t{this};
		}

		// Add 'fvert' to the vert containers
		void add_vert(FatVert const& fvert)
		{
			// Grow a container if not repeatedly adding the same element or the default element
			auto add_to = [](auto& cont, auto const& elem, size_t vcount)
			{
				using ContType = std::decay_t<decltype(cont)>;

				// 2 or more unique elements, assume all are unique
				if (cont.size() > 1)
				{
					cont.push_back(elem);
					return;
				}

				// Different to the first element, fill to 'vcount' and add the new element
				if (cont.size() == 1 && cont[0] != elem)
				{
					cont.resize(vcount - 1, cont[0]);
					cont.push_back(elem);
					return;
				}
				
				// Not equal to the default elem, fill to 'vcount' and add the new element
				if (elem != ContType::Default)
				{
					cont.resize(vcount - 1, ContType::Default);
					cont.push_back(elem);
					return;
				}
			};

			// Verts are always unique
			m_vert.push_back(m_bbox.Grow(fvert.m_vert));
			add_to(m_diff, fvert.m_diff.argb(), vcount());
			add_to(m_norm, fvert.m_norm, vcount());
			add_to(m_tex0, fvert.m_tex0, vcount());
		}

		// Add a nugget to the mesh
		void add_nugget(Nugget const& nugget)
		{
			m_nugget.push_back(nugget);
		}
	};
	struct Scene
	{
		MatCont m_materials;
		MeshCont m_meshes;

		Scene()
			:m_materials()
			,m_meshes()
		{}
	};
	struct File
	{
		uint32_t m_version;
		Scene m_scene;

		File()
			:m_version(Version)
			,m_scene()
		{}
	};
	#pragma endregion

	#pragma region Stream traits
	// Traits for a stream-like source
	template <typename Stream> struct traits
	{
		// Move the read/write position in the stream
		static int64_t tellg(Stream& src)
		{
			return static_cast<int64_t>(src.tellg());
		}
		static int64_t tellp(Stream& src)
		{
			return static_cast<int64_t>(src.tellp());
		}
		static bool seekg(Stream& src, int64_t pos)
		{
			return static_cast<bool>(src.seekg(pos));
		}
		static bool seekp(Stream& src, int64_t pos)
		{
			return static_cast<bool>(src.seekp(pos));
		}

		// True if still ok for reading
		static bool good(Stream& src)
		{
			// Check if there is more data available. Eof is not signalled
			// until and attempt to read past the eof is made.
			src.peek();
			return src.good();
		}

		// Read an array
		template <typename TOut> static void read(Stream& src, TOut* out, size_t count)
		{
			if (src.read(char_ptr(out), count * sizeof(TOut))) return;
			throw std::runtime_error("partial read of input stream");
		}

		// Write an array
		template <typename TIn> static void write(Stream& dst, TIn const* in, size_t count)
		{
			if (dst.write(char_ptr(in), count * sizeof(TIn))) return;
			throw std::runtime_error("partial write of output stream");
		}

		// RAII stream position perserver
		struct SavePos
		{
			Stream* m_src;
			int64_t m_pos;

			~SavePos()
			{
				if (m_src == nullptr) return;
				seekg(*m_src, m_pos);
			}
			SavePos()
				:m_src()
				,m_pos()
			{}
			SavePos(Stream& src)
				:m_src(&src)
				,m_pos(tellg(src))
			{}
			SavePos(SavePos&& rhs)
				:SavePos()
			{
				std::swap(m_src, rhs.m_src);
				std::swap(m_pos, rhs.m_pos);
			}
			SavePos(SavePos const&) = delete;
			SavePos& operator =(SavePos&&) = delete;
			SavePos& operator =(SavePos const&) = delete;
		};

		// RAII scope for the stream get pointer
		static SavePos saveg(Stream& src)
		{
			return SavePos(src);
		}
	};
	#pragma endregion

	// Chunk header (8-bytes)
	struct ChunkHeader
	{
		// Notes:
		//  - 'm_length' includes the size of the ChunkHeader

		EChunkId m_id;
		uint32_t m_length;

		ChunkHeader() = default;
		ChunkHeader(EChunkId id, size_t payload)
			:m_id(id)
			, m_length(s_cast<uint32_t>(sizeof(ChunkHeader) + payload))
		{
		}

		// True if null equal to the Null chunk
		explicit operator bool() const
		{
			return m_id != EChunkId::Null;
		}

		// The size (in bytes) of the chunk payload
		constexpr uint32_t payload() const
		{
			assert(m_length >= sizeof(ChunkHeader));
			return m_length - sizeof(ChunkHeader);
		}
	};
	static_assert(sizeof(ChunkHeader) == 8, "Incorrect chunk header size");

	// Used to build an index of a p3d file without having to load all of the data into memory.
	struct ChunkIndex :ChunkHeader
	{
		// Notes:
		//  - 'payload' in these constructors should *not* include the ChunkHeader size
		using Cont = std::vector<ChunkIndex>;
		Cont m_chunks;

		ChunkIndex(EChunkId id, size_t payload)
			:ChunkHeader(id, payload)
			,m_chunks()
		{}
		ChunkIndex(EChunkId id, size_t payload, std::initializer_list<ChunkIndex> chunks)
			:ChunkIndex(id, payload)
		{
			for (auto& c : chunks)
				add(c);
		}
		void add(ChunkIndex const& chunk)
		{
			m_chunks.push_back(chunk);
			m_length += chunk.m_length;
		}
		ChunkIndex const& operator[](EChunkId id) const
		{
			for (auto& c : m_chunks)
				if (c.m_id == id)
					return c;

			throw std::runtime_error(Fmt("Child chunk '%8X' not a member of chunk '%8X'", id, m_id));
		}
		ChunkIndex const& find(std::initializer_list<EChunkId> chunk_id) const
		{
			// Search down the tree for a chunk
			auto const* b = chunk_id.begin();
			auto const* e = chunk_id.end();

			auto const* chunks = &m_chunks;
			for (; b != e; ++b)
			{
				for (auto& c : *chunks)
				{
					if (c.m_id != *b) continue;
					if ((b + 1) == e) return c;
					chunks = &c.m_chunks;
					break;
				}
			}
			return NullChunk();
		}
	};

	// Static null chunk
	inline ChunkIndex const& NullChunk()
	{
		static ChunkIndex nullchunk(EChunkId::Null, 0);
		return nullchunk;
	}

	// Preserve the stream get pointer
	template <typename TSrc> typename traits<TSrc>::SavePos SaveG(TSrc& src)
	{
		return traits<TSrc>::saveg(src);
	}

	// Chunk reading/searching function.
	// 'src' should point to data after a chunk header (or the start of a stream).
	// 'len' is the remaining bytes from 'src' to the end of the parent chunk or stream (can use ~0U to search to the end of the stream).
	// 'len_out' is an output of the length until the end of the parent chunk on return.
	// 'func' is called with the found ChunkHeader and with 'src' positioned at the start of the data for the found chunk.
	// 'func' signature is:   bool Func(ChunkHeader hdr, TSrc& src).
	// 'func' can be a boolean predicate or a parsing function. Return true to stop searching.
	template <typename TSrc, typename Func>
	ChunkHeader Find(TSrc& src, uint32_t len, Func func, uint32_t* len_out = nullptr)
	{
		for (; len != 0 && traits<TSrc>::good(src); )
		{
			auto start = traits<TSrc>::tellg(src);

			// Read the chunk header
			ChunkHeader hdr;
			traits<TSrc>::read(src, &hdr, 1);
			if (hdr.m_length <= len)
				len -= hdr.m_length;
			else
				throw std::runtime_error(Fmt("invalid chunk found at offset 0x%llx", start));

			try
			{
				// Callback with the chunk
				if (func(hdr, src))
				{
					// Update the remaining length
					if (len_out)
						*len_out = len;

					return hdr;
				}
			}
			catch (std::exception& ex)
			{
				// This is a slicing assignment, but that is actually what we want
				ex = std::exception(FmtS("%s\n  %s (%d)", ex.what(), EChunkId_::ToStringA(hdr.m_id), hdr.m_id));
				throw; // Preserves original exception type
			}

			// Seek to the next chunk
			traits<TSrc>::seekg(src, start + hdr.m_length);
		}

		// Update the remaining length
		if (len_out)
			*len_out = 0;

		return ChunkHeader{};
	}

	// Search from the current stream position to the next instance of chunk 'id'.
	// Assumes 'src' is positioned at a chunk header within a parent chunk.
	// 'len' is the number of bytes until the end of the parent chunk.
	// 'len_out' is an output of the length until the end of the parent chunk on return.
	// If 'next' is true and 'src' currently points to an 'id' chunk, then seeks to the next instance of 'id'
	// Returns the found chunk header with the current position of 'src' set immediately after it.
	template <typename TSrc>
	ChunkHeader Find(TSrc& src, uint32_t len, EChunkId id, uint32_t* len_out = nullptr, bool next = false)
	{
		auto chunk = ChunkHeader{};
		Find(src, len, [&](ChunkHeader hdr, TSrc&)
		{
			// If this is the chunk we're looking for return false to say "done"
			if (hdr.m_id == id && !next)
			{
				chunk = hdr;
				return true;
			}

			next = false;
			return false;
		}, len_out);
		return chunk;
	}

	// Search from the current stream position to the nested chunk described by the list.
	// Finds the first matching chunk Id at each level.
	// 'src' is assume to be pointed to a chunk header.
	template <typename TSrc>
	ChunkHeader Find(TSrc& src, uint32_t len, std::initializer_list<EChunkId> chunk_id)
	{
		auto hdr = ChunkHeader{};
		for (auto id : chunk_id)
		{
			hdr = Find(src, len, id);
			if (hdr.m_id != id)
			{
				// Special case the Main chunk, if it's missing assume 'src' is not a p3d stream and throw
				if (id == EChunkId::Main) throw std::runtime_error("Source is not a p3d stream");
				return ChunkHeader{};
			}
			len = hdr.payload();
		}
		return hdr;
	}

	#pragma region Write

	// Notes:
	//  - Each Write function returns the size (in bytes) added to 'out'
	//  - To write out only part of a File, delete the parts in a temporary copy of the file.

	// Write 'hdr' at 'offset', preserving the current output position in 'src'
	template <typename TOut> void UpdateHeader(TOut& out, int64_t offset, ChunkHeader hdr)
	{
		if ((hdr.m_length & 0b11) != 0)
			throw std::runtime_error("Chunk size is not aligned to 4 bytes");

		auto pos = traits<TOut>::tellp(out);
		traits<TOut>::seekp(out, offset);
		traits<TOut>::write(out, &hdr, 1);
		traits<TOut>::seekp(out, pos);
	}

	// Write an array
	template <typename TIn, typename TOut> inline uint32_t Write(TOut& out, TIn const* in, size_t count)
	{
		traits<TOut>::write(out, in, count);
		return s_cast<uint32_t>(count * sizeof(*in));
	}

	// Write bytes to 'out' to pad 'hdr' to a uint32_t boundary
	template <typename TOut> uint32_t PadToU32(TOut& out, uint32_t chunk_size)
	{
		return Write(out, "\0\0\0\0", Pad<uint32_t>(chunk_size, s_cast<int>(sizeof(uint32_t))));
	}

	// Write an array of type 'TIn' and an array of type 'TAs'
	template <typename TAs, typename TIn, typename TOut> inline uint32_t WriteCast(TOut& out, TIn const* in, size_t count)
	{
		if constexpr (std::is_same_v<TIn, TOut>)
		{
			// No type conversion, direct copy
			return Write(out, in, count);
		}
		else
		{
			// Convert from 'TIn' to 'TAs'
			byte_data<> buf(count * sizeof(TAs));

			auto pin = in;
			auto pas = buf.data<TAs>();
			for (auto n = count; n-- != 0;)
				*pas++ = s_cast<TAs>(*pin++);

			return Write(out, buf.data<TAs>(), buf.size<TAs>());
		}
	}

	// Write a single POD type
	template <typename TIn, typename TOut> inline uint32_t Write(TOut& out, TIn const& in)
	{
		return Write(out, &in, 1);
	}

	// Write a string not within a chunk. Note: not padded
	template <typename TOut> uint32_t WriteStr(TOut& out, std::string_view str)
	{
		// Don't combine these because the order of evaluating 'a' and 'b' in 'a + b' is undefined
		uint32_t len = 0;
		len += Write(out, s_cast<uint32_t>(str.size())); // String length;
		len += Write(out, str.data(), str.size());       // String data
		return len;
	}

	// Write a string to 'out'.
	template <typename TOut> uint32_t WriteStr(TOut& out, EChunkId chunk_id, std::string_view str)
	{
		auto offset = traits<TOut>::tellp(out);

		// String chunk header
		ChunkHeader hdr(chunk_id, 0U);
		Write<ChunkHeader>(out, hdr);
		
		// String
		hdr.m_length += WriteStr(out, str);

		// Chunk padding
		hdr.m_length += PadToU32(out, hdr.m_length);

		UpdateHeader(out, offset, hdr);
		return hdr.m_length;
	}

	// Write a texture to 'out'
	template <typename TOut> uint32_t WriteTexture(TOut& out, Texture const& tex)
	{
		auto offset = traits<TOut>::tellp(out);

		// Texture chunk header
		ChunkHeader hdr(EChunkId::Texture, 0U);
		Write<ChunkHeader>(out, hdr);

		// Texture filepath
		hdr.m_length += WriteStr(out, tex.m_filepath);

		// Texture type
		hdr.m_length += Write(out, s_cast<uint8_t>(tex.m_type));

		// Texture address mode
		hdr.m_length += Write(out, s_cast<uint8_t>(tex.m_addr_mode));

		// Texture flags
		hdr.m_length += Write(out, s_cast<uint16_t>(tex.m_flags));

		// Chunk padding
		hdr.m_length += PadToU32(out, hdr.m_length);

		UpdateHeader(out, offset, hdr);
		return hdr.m_length;
	}

	// Write a diffuse colour chunk to 'out'
	template <typename TOut> uint32_t WriteColour(TOut& out, EChunkId chunk_id, Colour const& colour)
	{
		ChunkHeader hdr(chunk_id, sizeof(Colour));
		Write<ChunkHeader>(out, hdr);
		Write<Colour>(out, colour);
		return hdr.m_length;
	}

	// Write a material to 'out'
	template <typename TOut> uint32_t WriteMaterial(TOut& out, Material const& mat)
	{
		auto offset = traits<TOut>::tellp(out);

		// Material chunk header
		ChunkHeader hdr(EChunkId::Material, 0U);
		Write<ChunkHeader>(out, hdr);

		// Material name (exactly 16 chars, no need for length first)
		hdr.m_length += Write(out, &mat.m_id.str[0], sizeof(mat.m_id.str));

		// Diffuse colour
		hdr.m_length += WriteColour(out, EChunkId::DiffuseColour, mat.m_diffuse);

		// Textures
		for (auto& tex : mat.m_textures)
			hdr.m_length += WriteTexture(out, tex);

		UpdateHeader(out, offset, hdr);
		return hdr.m_length;
	}

	// Write a collection of materials to 'out'
	template <typename TOut> uint32_t WriteMaterials(TOut& out, std::span<Material const> mats)
	{
		if (mats.empty())
			return 0;

		auto offset = traits<TOut>::tellp(out);

		// Materials chunk header
		ChunkHeader hdr(EChunkId::Materials, 0U);
		Write<ChunkHeader>(out, hdr);

		// Material data
		for (auto& mat : mats)
			hdr.m_length += WriteMaterial(out, mat);

		UpdateHeader(out, offset, hdr);
		return hdr.m_length;
	}

	// Write a bounding box to 'out'
	template <typename TOut> uint32_t WriteMeshBBox(TOut& out, BBox const& bbox)
	{
		if (bbox == BBox::Reset())
			return 0;

		if (bbox.m_radius.x < 0 ||
			bbox.m_radius.y < 0 ||
			bbox.m_radius.z < 0)
			throw std::runtime_error("Writing an invalid bounding box into p3d");

		ChunkHeader hdr(EChunkId::MeshBBox, sizeof(BBox));
		Write<ChunkHeader>(out, hdr);
		Write<BBox>(out, bbox);
		return hdr.m_length;
	}

	// Write a mesh to parent transform to 'out'
	template <typename TOut> uint32_t WriteMeshTransform(TOut& out, m4x4 const& o2p)
	{
		if (o2p == m4x4::Identity())
			return 0;

		ChunkHeader hdr(EChunkId::MeshTransform, sizeof(m4x4));
		Write<ChunkHeader>(out, hdr);
		Write<m4x4>(out, o2p);
		return hdr.m_length;
	}

	// Write vertices to 'out'
	template <typename TOut> uint32_t WriteVertices(TOut& out, VCont const& verts, EFlags flags)
	{
		if (verts.size() == 0)
			return 0;

		auto offset = traits<TOut>::tellp(out);

		ChunkHeader hdr(EChunkId::MeshVerts, 0U);
		Write<ChunkHeader>(out, hdr);

		// Count
		hdr.m_length += Write(out, s_cast<uint32_t>(verts.size()));

		// Format
		auto fmt = s_cast<EVertFormat>((int(flags) >> Flags::VertsOfs) & Flags::Mask);
		hdr.m_length += Write(out, s_cast<uint16_t>(fmt));

		// Vertex data
		switch (fmt)
		{
		case EVertFormat::Verts32Bit:
			{
				// Stride
				hdr.m_length += Write<uint16_t>(out, sizeof(float[3]));

				// Use 32bit floats for position data
				byte_data<> buf(verts.size() * sizeof(float[3]));
				auto ptr = buf.data<float>();
				for (auto& v : verts)
				{
					*ptr++ = v.x;
					*ptr++ = v.y;
					*ptr++ = v.z;
				}

				hdr.m_length += Write(out, buf.data(), buf.size());
				break;
			}
		case EVertFormat::Verts16Bit:
			{
				// Stride
				hdr.m_length += Write<uint16_t>(out, sizeof(half_t[3]));

				// Use 16bit floats for position data
				byte_data<> buf(verts.size() * sizeof(half_t[3]));
				auto ptr = buf.data<uint16_t>();
				for (auto& v : verts)
				{
					auto h = F32toF16(v);
					*ptr++ = h.x;
					*ptr++ = h.y;
					*ptr++ = h.z;
				}

				hdr.m_length += Write(out, buf.data(), buf.size());
				break;
			}
		default:
			{
				throw std::runtime_error("Unknown vertex storage flags");
			}
		}

		// Chunk padding
		hdr.m_length += PadToU32(out, hdr.m_length);

		UpdateHeader(out, offset, hdr);
		return hdr.m_length;
	}

	// Write vertex colours to 'out'
	template <typename TOut> uint32_t WriteColours(TOut& out, CCont const& colours, EFlags flags)
	{
		if (colours.size() == 0)
			return 0;

		auto offset = traits<TOut>::tellp(out);

		ChunkHeader hdr(EChunkId::MeshColours, 0U);
		Write<ChunkHeader>(out, hdr);

		// Count
		hdr.m_length += Write(out, s_cast<uint32_t>(colours.size()));

		// Format
		auto fmt = s_cast<EColourFormat>((int(flags) >> Flags::ColoursOfs) & Flags::Mask);
		hdr.m_length += Write(out, s_cast<uint16_t>(fmt));

		// Vertex colour data
		switch (fmt)
		{
		case EColourFormat::Colours32Bit:
			{
				// Stride
				hdr.m_length += Write<uint16_t>(out, sizeof(uint32_t));

				// Use AARRGGBB 32-bit colour values
				byte_data<> buf(colours.size() * sizeof(Colour32));
				auto ptr = buf.data<Colour32>();
				for (auto& c : colours)
					*ptr++ = c;

				hdr.m_length += Write(out, buf.data(), buf.size());
				break;
			}
		default:
			{
				throw std::runtime_error("Unknown vertex colours storage flag");
			}
		}
		
		// Chunk padding
		hdr.m_length += PadToU32(out, hdr.m_length);

		UpdateHeader(out, offset, hdr);
		return hdr.m_length;
	}

	// Write vertex normals to 'out'
	template <typename TOut> uint32_t WriteNormals(TOut& out, NCont const& norms, EFlags flags)
	{
		if (norms.size() == 0)
			return 0;

		auto offset = traits<TOut>::tellp(out);

		ChunkHeader hdr(EChunkId::MeshNorms, 0U);
		Write<ChunkHeader>(out, hdr);

		// Count
		hdr.m_length += Write(out, s_cast<uint32_t>(norms.size()));
				
		// Format
		auto fmt = s_cast<ENormFormat>((int(flags) >> Flags::NormsOfs) & Flags::Mask);
		hdr.m_length += Write(out, s_cast<uint16_t>(fmt));

		// Vertex data
		switch (fmt)
		{
		case ENormFormat::Norms32Bit:
			{
				// Stride
				hdr.m_length += Write<uint16_t>(out, sizeof(float[3]));

				// Use 32bit floats for normals
				byte_data<> buf(norms.size() * sizeof(float[3]));
				auto ptr = buf.data<float>();
				for (auto& n : norms)
				{
					*ptr++ = n.x;
					*ptr++ = n.y;
					*ptr++ = n.z;
				}

				hdr.m_length += Write(out, buf.data(), buf.size());
				break;
			}
		case ENormFormat::Norms16Bit:
			{
				// Stride
				hdr.m_length += Write<uint16_t>(out, sizeof(half_t[3]));

				// Use 16bit floats for normals
				byte_data<> buf(norms.size() * sizeof(half_t[3]));
				auto ptr = buf.data<half_t>();
				for (auto& n : norms)
				{
					auto h = F32toF16(n);
					*ptr++ = h.x;
					*ptr++ = h.y;
					*ptr++ = h.z;
				}

				hdr.m_length += Write(out, buf.data(), buf.size());
				break;
			}
		case ENormFormat::NormsPack32:
			{
				// Stride
				hdr.m_length += Write<uint16_t>(out, sizeof(uint32_t));

				// Pack normals into 32bits
				byte_data<> buf(norms.size() * sizeof(uint32_t));
				auto ptr = buf.data<uint32_t>();
				for (auto& n : norms)
					*ptr++ = Norm32bit::Compress(n);

				hdr.m_length += Write(out, buf.data(), buf.size());
				break;
			}
		default:
			{
				throw std::runtime_error("Unknown normals storage flag");
			}
		}

		// Chunk padding
		hdr.m_length += PadToU32(out, hdr.m_length);

		UpdateHeader(out, offset, hdr);
		return hdr.m_length;
	}

	// Write texture coordinates to 'out'
	template <typename TOut> uint32_t WriteTexCoords(TOut& out, TCont const& uvs, EFlags flags)
	{
		if (uvs.size() == 0)
			return 0;

		auto offset = traits<TOut>::tellp(out);

		ChunkHeader hdr(EChunkId::MeshUVs, 0U);
		Write<ChunkHeader>(out, hdr);

		// Count
		hdr.m_length += Write(out, s_cast<uint32_t>(uvs.size()));

		// Format
		auto fmt = s_cast<EUVFormat>((int(flags) >> Flags::UVsOfs) & Flags::Mask);
		hdr.m_length += Write(out, s_cast<uint16_t>(fmt));

		// Texture coords
		switch (fmt)
		{
		case EUVFormat::UVs32Bit:
			{
				// Stride
				hdr.m_length += Write<uint16_t>(out, sizeof(float[2]));

				// Use 32-bit float values
				byte_data<> buf(uvs.size() * sizeof(float[2]));
				auto ptr = buf.data<float>();
				for (auto& u : uvs)
				{
					*ptr++ = u.x;
					*ptr++ = u.y;
				}

				hdr.m_length += Write(out, buf.data(), buf.size());
				break;
			}
		case EUVFormat::UVs16Bit:
			{
				// Stride
				hdr.m_length += Write<uint16_t>(out, sizeof(half_t[2]));

				// Use 16-bit float values
				byte_data<> buf(uvs.size() * sizeof(half_t[2]));
				auto ptr = buf.data<half_t>();
				for (auto& u : uvs)
				{
					auto h = F32toF16(v4{u, 0, 0});
					*ptr++ = h.x;
					*ptr++ = h.y;
				}

				hdr.m_length += Write(out, buf.data(), buf.size());
				break;
			}
		default:
			{
				throw std::runtime_error("Unknown texture coordinates storage flag");
			}
		}

		// Chunk padding
		hdr.m_length += PadToU32(out, hdr.m_length);

		UpdateHeader(out, offset, hdr);
		return hdr.m_length;
	}

	// Write index data to 'out'
	template <typename TOut, typename Idx> uint32_t WriteIndices(TOut& out, IdxBuf const& idx, EFlags flags)
	{
		// Note:
		//  - 'Idx' is the data type of the values in 'idx'
		//  - 'flags' controls the type of indices that are written to 'out'.
		//  - 'chunkid' labels the type of indices being written.

		auto offset = traits<TOut>::tellp(out);

		ChunkHeader hdr(EChunkId::MeshVIdx, 0U);
		Write<ChunkHeader>(out, hdr);

		// If the format is 'IdxSrc', set 'fmt' to match 'Idx'
		auto fmt = s_cast<EIndexFormat>((int(flags) >> Flags::IndexOfs) & Flags::Mask);
		fmt = fmt != EIndexFormat::IdxSrc ? fmt :
			std::is_same_v<Idx, uint32_t> ? EIndexFormat::Idx32Bit :
			std::is_same_v<Idx, uint16_t> ? EIndexFormat::Idx16Bit :
			std::is_same_v<Idx, uint8_t > ? EIndexFormat::Idx8Bit :
			throw std::runtime_error("Unsupported index stride");

		// Count
		hdr.m_length += Write(out, s_cast<uint32_t>(idx.size()));

		// Format
		hdr.m_length += Write(out, s_cast<uint16_t>(fmt));

		// Index data
		switch (fmt)
		{
			case EIndexFormat::Idx32Bit:
			{
				// Stride (of written indices. Possibly different to idx.m_stride)
				hdr.m_length += Write<uint16_t>(out, sizeof(uint32_t));

				// Convert from 'Idx' to uint32_t
				hdr.m_length += WriteCast<uint32_t>(out, idx.data<Idx>(), idx.size());
				break;
			}
			case EIndexFormat::Idx16Bit:
			{
				// Stride (of written indices. Possibly different to idx.m_stride)
				hdr.m_length += Write<uint16_t>(out, sizeof(uint16_t));

				// Convert from 'Idx' to uint16_t
				hdr.m_length += WriteCast<uint16_t>(out, idx.data<Idx>(), idx.size());
				break;
			}
			case EIndexFormat::Idx8Bit:
			{
				// Stride (of written indices. Possibly different to idx.m_stride)
				hdr.m_length += Write<uint16_t>(out, sizeof(uint8_t));

				// Convert from 'Idx' to uint8_t
				hdr.m_length += WriteCast<uint8_t>(out, idx.data<Idx>(), idx.size());
				break;
			}
			case EIndexFormat::IdxNBit:
			{
				// Stride (of written indices *after decompression* == idx.m_stride)
				hdr.m_length += Write<uint16_t>(out, sizeof(Idx));

				// Use ZigZag encoded variable length integers (like protobuf)
				byte_data<> buf;
				buf.reserve(idx.size() * 3 / 2);

				// Fill 'buf' with variable length indices
				int64_t prev = 0;
				for (auto i : idx.span_as<Idx>())
				{
					// Get the delta from the previous index
					auto delta = i - prev;

					// ZigZag encode to prevent negative 2s compliment numbers using lots of space
					constexpr int BitWidth = sizeof(Idx) * 8;
					auto zz = static_cast<uint64_t>((delta << 1) ^ (delta >> (BitWidth - 1)));

					// Variable length int encode
					for (; zz > 127; zz >>= 7)
						buf.push_back(s_cast<uint8_t>(0x80 | (zz & 0x7F)));
					buf.push_back(s_cast<uint8_t>(zz));

					// Update the new previous value
					prev += delta;
				}

				hdr.m_length += Write(out, buf.data(), buf.size());
				break;
			}
			default:
			{
				throw std::runtime_error("Unknown texture coordinates storage flag");
			}
		}

		// Chunk padding
		hdr.m_length += PadToU32(out, hdr.m_length);

		UpdateHeader(out, offset, hdr);
		return hdr.m_length;
	}
	template <typename TOut> inline uint32_t WriteIndices(TOut& out, IdxBuf const& idx, EFlags flags)
	{
		// Convert from runtime 'stride' to compile time index type
		switch (idx.stride())
		{
			case 4: return WriteIndices<TOut, uint32_t>(out, idx, flags); break;
			case 2: return WriteIndices<TOut, uint16_t>(out, idx, flags); break;
			case 1: return WriteIndices<TOut, uint8_t>(out, idx, flags); break;
			default: throw std::runtime_error(FmtS("Unsupported index stride: %d", idx.stride()));
		}
	}

	// Write a mesh nugget to 'out'
	template <typename TOut> uint32_t WriteNugget(TOut& out, Nugget const& nug, EFlags flags)
	{
		auto offset = traits<TOut>::tellp(out);

		// Mesh chunk header
		ChunkHeader hdr(EChunkId::MeshNugget, 0U);
		Write<ChunkHeader>(out, hdr);

		// Mesh topology
		hdr.m_length += Write<uint16_t>(out, s_cast<uint16_t>(nug.m_topo));

		// Mesh geometry
		hdr.m_length += Write<uint16_t>(out, s_cast<uint16_t>(nug.m_geom));

		// Material id
		hdr.m_length += WriteStr(out, EChunkId::MeshMatId, nug.m_mat);

		// Face/Line/Tetra/etc indices
		hdr.m_length += WriteIndices(out, nug.m_vidx, flags);
		
		UpdateHeader(out, offset, hdr);
		return hdr.m_length;
	}

	// Write a mesh to 'out'
	template <typename TOut> uint32_t WriteMesh(TOut& out, Mesh const& mesh, EFlags flags)
	{
		auto offset = traits<TOut>::tellp(out);

		// Mesh chunk header
		ChunkHeader hdr(EChunkId::Mesh, 0U);
		Write<ChunkHeader>(out, hdr);

		// Mesh name
		hdr.m_length += WriteStr(out, EChunkId::MeshName, mesh.m_name);

		// Mesh bounding box
		hdr.m_length += WriteMeshBBox(out, mesh.m_bbox);

		// Mesh to parent transform
		hdr.m_length += WriteMeshTransform(out, mesh.m_o2p);

		// Vertex data
		hdr.m_length += WriteVertices(out, mesh.m_vert, flags);

		// Colour data
		hdr.m_length += WriteColours(out, mesh.m_diff, flags);

		// Normals data
		hdr.m_length += WriteNormals(out, mesh.m_norm, flags);

		// UV data
		hdr.m_length += WriteTexCoords(out, mesh.m_tex0, flags);

		// Write each nugget
		for (auto const& nugget : mesh.m_nugget)
			hdr.m_length += WriteNugget(out, nugget, flags);

		UpdateHeader(out, offset, hdr);
		return hdr.m_length;
	}

	// Write a collection of meshes to 'out'
	template <typename TOut> uint32_t WriteMeshes(TOut& out, std::span<Mesh const> meshs, EFlags flags)
	{
		auto offset = traits<TOut>::tellp(out);

		// Meshes chunk header
		ChunkHeader hdr(EChunkId::Meshes, 0U);
		Write<ChunkHeader>(out, hdr);

		// Mesh data
		for (auto& mesh : meshs)
			hdr.m_length += WriteMesh(out, mesh, flags);

		UpdateHeader(out, offset, hdr);
		return hdr.m_length;
	}

	// Write a scene to 'out'
	template <typename TOut> uint32_t WriteScene(TOut& out, Scene const& scene, EFlags flags)
	{
		auto offset = traits<TOut>::tellp(out);

		ChunkHeader hdr(EChunkId::Scene, 0U);
		Write<ChunkHeader>(out, hdr);

		// Scene materials
		hdr.m_length += WriteMaterials(out, scene.m_materials);

		// Scene meshes
		hdr.m_length += WriteMeshes(out, scene.m_meshes, flags);

		UpdateHeader(out, offset, hdr);
		return hdr.m_length;
	}

	// Write the file version chunk
	template <typename TOut> uint32_t WriteVersion(TOut& out, uint32_t version)
	{
		ChunkHeader hdr(EChunkId::FileVersion, sizeof(uint32_t));
		Write<ChunkHeader>(out, hdr);
		Write<uint32_t>(out, version);
		return hdr.m_length;
	}

	// Write the p3d file to an ostream-like output.
	template <typename TOut> uint32_t Write(TOut& out, File const& file, EFlags flags = EFlags::Default)
	{
		// Notes:
		//  - Cannot use forward iteration only because some chunk sizes are not known ahead of time.
		//  - The chunk sizes in 'index' are ignored/overwritten, compressed chunks will have smaller sizes.

		auto offset = traits<TOut>::tellp(out);

		// Write a proxy file chunk header. The length will be filled in at the end once known.
		ChunkHeader hdr(EChunkId::Main, 0U);
		Write<ChunkHeader>(out, hdr);

		// Write the file version
		hdr.m_length += WriteVersion(out, file.m_version);

		// Write the scene
		hdr.m_length += WriteScene(out, file.m_scene, flags);

		UpdateHeader(out, offset, hdr);
		return hdr.m_length;
	}

	#pragma endregion

	#pragma region Read

	// Notes:
	//  - All of these Read functions assume 'src' points to the start
	//    of the chunk data of the corresponding chunk type.
	//  - Backwards compatibility is only needed in the Read functions.

	// Read an array
	template <typename TOut, typename TSrc> inline void Read(TSrc& src, TOut* out, size_t count)
	{
		traits<TSrc>::read(src, out, count);
	}

	// Read an array with element transforming.
	// 'TIn' is the data type of the elements to read.
	// 'count' is the number of times to call 'out'
	// 'stride' is the size in bytes consumed with each call to 'out'
	// 'out' is an output function used to consume the read elements.
	template <typename TIn, typename TOut, typename TSrc> inline void Read(TSrc& src, size_t count, size_t stride, TOut out)
	{
		// Example:
		//  count = 3, stride = 12 bytes, sizeof(TIn) = 4 bytes
		//  'element' is the unit consumed by the 'out' callback.
		//    => element size in units of TIn = stride / sizeof(TIn) = 3 TIn/element

		constexpr int PageSizeBytes = 0x10000;

		if (stride > PageSizeBytes)
			throw std::runtime_error("Stride value is too large for local page buffer.");
		if ((stride % sizeof(TIn)) != 0)
			throw std::runtime_error("Stride value must be a multiple of the size of the input elements");

		// Local buffer
		std::vector<TIn> page(PageSizeBytes / sizeof(TIn));
		auto const page_max = PageSizeBytes / stride; // the number of whole elements that fit in 'page'
		auto const elem_size = stride / sizeof(TIn);  // the element size in units of 'Tin'

		for (; count != 0; )
		{
			// The number of 'TIn's to read
			auto n = elem_size * std::min(count, page_max);
			Read(src, &page[0], n);

			auto const* pbeg = &page[0];
			auto const* pend = &page[0] + n;
			for (auto p = pbeg; p != pend; p += elem_size, --count)
				out(p);
		}
	}

	// Read a single type
	template <typename TOut, typename TSrc> inline TOut Read(TSrc& src)
	{
		TOut out;
		Read(src, &out, 1);
		return out;
	}

	// Read a string. 'src' is assumed to point to the start of a EChunkId::CStr chunk data
	template <typename TSrc, typename TStr = std::string> TStr ReadStr(TSrc& src, uint32_t len)
	{
		// Read the string length
		auto count = Read<uint32_t>(src);
		len -= sizeof(uint32_t);

		// Read the string data
		TStr str(count, '\0');
		Read(src, str.data(), str.size());
		return str;
	}

	// Read a texture. 'src' is assumed to point to the start of EChunkId::Texture chunk data
	template <typename TSrc> Texture ReadTexture(TSrc& src, uint32_t len)
	{
		Texture tex;
	
		// Texture filepath length
		auto flen = Read<uint32_t>(src);
		len -= sizeof(uint32_t);

		// Texture filepath
		tex.m_filepath.resize(flen);
		Read(src, tex.m_filepath.data(), tex.m_filepath.size());
		len -= s_cast<uint32_t>(tex.m_filepath.size());

		// Texture type
		tex.m_type = s_cast<Texture::EType>(Read<uint8_t>(src));

		// Texture address mode
		tex.m_addr_mode = s_cast<Texture::EAddrMode>(Read<uint8_t>(src));

		// Texture flags
		tex.m_flags = s_cast<Texture::EFlags>(Read<uint16_t>(src));

		return tex;
	}

	// Read a material. 'src' is assumed to point to the start of EChunkId::Material chunk data
	template <typename TSrc> Material ReadMaterial(TSrc& src, uint32_t len)
	{
		Material mat;

		Read(src, mat.m_id.str, sizeof(Str16));
		len -= sizeof(Str16);

		Find(src, len, [&](ChunkHeader hdr, TSrc& src)
		{
			switch (hdr.m_id)
			{
			case EChunkId::DiffuseColour:
				{
					mat.m_diffuse = Read<Colour>(src);
					break;
				}
			case EChunkId::Texture:
				{
					mat.m_textures.emplace_back(ReadTexture(src, hdr.payload()));
					break;
				}
			}
			return false;
		});
		return mat;
	}

	// Fill a container of verts. 'src' is assumed to point to the start of EChunkId::MeshVerts chunk data
	template <typename TSrc> VCont ReadMeshVerts(TSrc& src, uint32_t len)
	{
		VCont cont;

		// Read the count
		auto count = Read<uint32_t>(src);
		len -= sizeof(uint32_t);

		// Read the format
		auto fmt = s_cast<EVertFormat>(Read<uint16_t>(src));
		len -= sizeof(uint16_t);

		// Read the stride
		auto stride = Read<uint16_t>(src);
		len -= sizeof(uint16_t);

		// Integrity check - remember data may be padded
		if (count * stride <= len)
			cont.resize(count);
		else
			throw std::runtime_error(Fmt("Vertex list count is invalid. Count is %d, data available for %d.", count, len / stride));

		// Read the vertex data into memory. Inflate to v4
		auto ptr = cont.data();
		switch (fmt)
		{
		case EVertFormat::Verts32Bit:
			{
				Read<float>(src, count, stride, [&](float const* p) { *ptr++ = v4{p[0], p[1], p[2], 1.0f}; });
				break;
			}
		case EVertFormat::Verts16Bit:
			{
				Read<half_t>(src, count, stride, [&](half_t const* p) { *ptr++ = F16toF32(Half4{p[0], p[1], p[2], 1.0_hf}); });
				break;
			}
		default:
			{
				throw std::runtime_error("Unsupported mesh vertex format");
			}
		}

		return cont;
	}

	// Fill a container of colours. 'src' is assumed to point to the start of EChunkId::MeshColours chunk data
	template <typename TSrc> CCont ReadMeshColours(TSrc& src, uint32_t len)
	{
		CCont cont;

		// Read the count
		auto count = Read<uint32_t>(src);
		len -= sizeof(uint32_t);

		// Read the format
		auto fmt = s_cast<EColourFormat>(Read<uint16_t>(src));
		len -= sizeof(uint16_t);

		// Read the stride
		auto stride = Read<uint16_t>(src);
		len -= sizeof(uint16_t);

		// Integrity check - remember data may be padded
		if (count * stride <= len)
			cont.resize(count);
		else
			throw std::runtime_error(Fmt("Colours list count is invalid. Count is %d, data available for %d.", count, len / stride));

		// Read the vertex colour data into memory. Inflate to Colour32
		auto ptr = cont.data();
		switch (fmt)
		{
		case EColourFormat::Colours32Bit:
			{
				Read<uint32_t>(src, count, stride, [&](uint32_t const* p) { *ptr++ = Colour32{p[0]}; });
				break;
			}
		default:
			{
				throw std::runtime_error("Unsupported mesh vertex colour format");
			}
		}

		return cont;
	}

	// Fill a container of normals. 'src' is assumed to point to the start of EChunkId::MeshNorms chunk data
	template <typename TSrc> NCont ReadMeshNorms(TSrc& src, uint32_t len)
	{
		NCont cont;

		// Read the count
		auto count = Read<uint32_t>(src);
		len -= sizeof(uint32_t);

		// Read the format
		auto fmt = s_cast<ENormFormat>(Read<uint16_t>(src));
		len -= sizeof(uint16_t);

		// Read the stride
		auto stride = Read<uint16_t>(src);
		len -= sizeof(uint16_t);

		// Integrity check - remember data may be padded
		if (count * stride <= len)
			cont.resize(count);
		else
			throw std::runtime_error(Fmt("Normals list count is invalid. Count is %d, data available for %d.", count, len / stride));

		// Read the normals data into memory. Inflate to v4
		auto ptr = cont.data();
		switch (fmt)
		{
		case ENormFormat::Norms32Bit:
			{
				Read<float>(src, count, stride, [&](float const* p) { *ptr++ = v4{p[0], p[1], p[2], 0.0f}; });
				break;
			}
		case ENormFormat::Norms16Bit:
			{
				Read<half_t>(src, count, stride, [&](half_t const* p) { *ptr++ = F16toF32(Half4{p[0], p[1], p[2], 0.0_hf}); });
				break;
			}
		case ENormFormat::NormsPack32:
			{
				Read<uint32_t>(src, count, stride, [&](uint32_t const* p) { *ptr++ = Norm32bit::Decompress(p[0]); });
				break;
			}
		default:
			{
				throw std::runtime_error("Unsupported mesh normals format");
			}
		}

		return cont;
	}

	// Fill a container of UVs. 'src' is assumed to point to the start of EChunkId::MeshUVs chunk data
	template <typename TSrc> TCont ReadMeshUVs(TSrc& src, uint32_t len)
	{
		TCont cont;

		// Read the count
		auto count = Read<uint32_t>(src);
		len -= sizeof(uint32_t);

		// Read the format
		auto fmt = s_cast<EUVFormat>(Read<uint16_t>(src));
		len -= sizeof(uint16_t);

		// Read the stride
		auto stride = Read<uint16_t>(src);
		len -= sizeof(uint16_t);

		// Integrity check - remember data may be padded
		if (count * stride <= len)
			cont.resize(count);
		else
			throw std::runtime_error(Fmt("Texture UVs list count is invalid. Count is %d, data available for %d.", count, len / stride));

		// Read the texture coord data into memory. Inflate to v2
		auto ptr = cont.data();
		switch (fmt)
		{
			case EUVFormat::UVs32Bit:
			{
				Read<float>(src, count, stride, [&](float const* p) { *ptr++ = v2{ p[0], p[1] }; });
				break;
			}
			case EUVFormat::UVs16Bit:
			{
				Read<half_t>(src, count, stride, [&](half_t const* p) { *ptr++ = F16toF32(Half4{ p[0], p[1], 0.0_hf, 0.0_hf }).xy; });
				break;
			}
			default:
			{
				throw std::runtime_error("Unsupported mesh UV format");
			}
		}

		return cont;
	}

	// Fill a container of indices. 'src' is assumed to point to the start of EChunkId::Mesh?Idx chunk data
	template <typename TSrc> IdxBuf ReadIndices(TSrc& src, uint32_t len)
	{
		IdxBuf cont;

		// Read the count
		auto count = Read<uint32_t>(src);
		len -= sizeof(uint32_t);

		// Read the format
		auto fmt = s_cast<EIndexFormat>(Read<uint16_t>(src));
		len -= sizeof(uint16_t);

		// Read the stride
		auto stride = Read<uint16_t>(src);
		len -= sizeof(uint16_t);

		// Integrity check
		if (count * stride > len && fmt != EIndexFormat::IdxNBit)
			throw std::runtime_error(Fmt("Indices buffer count is invalid. Count is %d, data available for %d.", count, len / stride));

		// Read the index data into memory
		switch (fmt)
		{
			case EIndexFormat::Idx32Bit:
			{
				cont.resize(count, stride);
				Read(src, cont.data<uint32_t>(), cont.size());
				break;
			}
			case EIndexFormat::Idx16Bit:
			{
				cont.resize(count, stride);
				Read(src, cont.data<uint16_t>(), cont.size());
				break;
			}
			case EIndexFormat::Idx8Bit:
			{
				cont.resize(count, stride);
				Read(src, cont.data<uint8_t>(), cont.size());
				break;
			}
			case EIndexFormat::IdxNBit:
			{
				// For IdxNBit, the stride value is the size of each decompressed index,
				// *not* the per-element size of the data in 'src' (like it is for other chunks).
				cont.reserve(count, stride);
				cont.resize(0, stride);

				// Read compressed indices into a local buffer
				byte_data<> buf(len);
				Read(src, buf.data(), buf.size());

				// Decompress from 'buf' into 'cont'
				// Note, that 'buf' contains padding, so the loop needs to stop when 'count' indices are read.
				int64_t prev = 0;
				auto const* p = buf.data<uint8_t>();
				auto const* pend = p + buf.size<uint8_t>();
				for (int i = 0; i != s_cast<int>(count) && p != pend; ++i, ++p)
				{
					int s = 0;
					uint64_t zz = 0;
					for (; p != pend && (*p & 0x80); ++p, s += 7)
						zz |= static_cast<uint64_t>(*p & 0x7F) << s;
					if (p != pend)
						zz |= static_cast<uint64_t>(*p & 0x7F) << s;

					// ZigZag decode
					auto delta = static_cast<int64_t>((zz & 1) ? (zz >> 1) ^ -1 : (zz >> 1));

					// Get the index value from the delta (only works for little endian!)
					auto idx = prev + delta;
					cont.push_back(idx);

					prev += delta;
				}

				// Integrity check
				if (cont.size() != count)
					throw std::runtime_error(Fmt("Index buffer count is invalid. Count is %d, %d indices provided.", count, static_cast<uint32_t>(cont.size() / stride)));

				break;
			}
			default:
			{
				throw std::runtime_error("Unsupported index buffer format");
			}
		}

		return cont;
	}

	// Read a mesh nugget. 'src' is assumed to point to the start of EChunkId::MeshNugget chunk data
	template <typename TSrc> Nugget ReadMeshNugget(TSrc& src, uint32_t len)
	{
		Nugget nugget;

		// Read the mesh topology
		nugget.m_topo = static_cast<ETopo>(Read<uint16_t>(src));
		len -= sizeof(uint16_t);

		// Read the mesh geometry
		nugget.m_geom = static_cast<EGeom>(Read<uint16_t>(src));
		len -= sizeof(uint16_t);

		// Read the child chunks
		Find(src, len, [&](ChunkHeader hdr, TSrc& src)
		{
			switch (hdr.m_id)
			{
				case EChunkId::MeshMatId:
				{
					// Read the material id
					auto id = ReadStr(src, hdr.payload());
					nugget.m_mat = id;
					break;
				}
				case EChunkId::MeshVIdx:
				{
					nugget.m_vidx = ReadIndices(src, hdr.payload());
					break;
				}
			}
			return false;
		});

		return nugget;
	}

	// Read a mesh. 'src' is assumed to point to the start of EChunkId::Mesh chunk data
	template <typename TSrc> Mesh ReadMesh(TSrc& src, uint32_t len)
	{
		Mesh mesh;
		Find(src, len, [&](ChunkHeader hdr, TSrc& src)
		{
			switch (hdr.m_id)
			{
				case EChunkId::MeshName:
				{
					mesh.m_name = ReadStr(src, hdr.payload());
					break;
				}
				case EChunkId::MeshBBox:
				{
					mesh.m_bbox = Read<BBox>(src);
					break;
				}
				case EChunkId::MeshTransform:
				{
					mesh.m_o2p = Read<m4x4>(src);
					break;
				}
				case EChunkId::MeshVerts:
				{
					mesh.m_vert = ReadMeshVerts(src, hdr.payload());
					break;
				}
				case EChunkId::MeshColours:
				{
					mesh.m_diff = ReadMeshColours(src, hdr.payload());
					break;
				}
				case EChunkId::MeshNorms:
				{
					mesh.m_norm = ReadMeshNorms(src, hdr.payload());
					break;
				}
				case EChunkId::MeshUVs:
				{
					mesh.m_tex0 = ReadMeshUVs(src, hdr.payload());
					break;
				}
				case EChunkId::MeshNugget:
				{
					mesh.m_nugget.emplace_back(ReadMeshNugget(src, hdr.payload()));
					break;
				}
				case EChunkId::Mesh:
				{
					auto child = ReadMesh(src, hdr.payload());
					mesh.m_children.emplace_back(std::move(child));
					break;
				}
			}
			return false;
		});
		return mesh;
	}

	// Fill a container of materials. 'src' is assumed to point to the start of EChunkId::Materials chunk data
	template <typename TSrc> MatCont ReadSceneMaterials(TSrc& src, uint32_t len)
	{
		MatCont mats;
		Find(src, len, [&](ChunkHeader hdr, TSrc& src)
		{
			switch (hdr.m_id)
			{
				case EChunkId::Material:
					mats.emplace_back(ReadMaterial(src, hdr.payload()));
					break;
			}
			return false;
		});
		return mats;
	}

	// Fill a container of meshes. 'src' is assumed to point to the start of EChunkId::Meshes chunk data
	template <typename TSrc> MeshCont ReadSceneMeshes(TSrc& src, uint32_t len)
	{
		MeshCont meshes;
		Find(src, len, [&](ChunkHeader hdr, TSrc& src)
		{
			switch (hdr.m_id)
			{
				case EChunkId::Mesh:
					meshes.emplace_back(ReadMesh(src, hdr.payload()));
					break;
			}
			return false;
		});
		return meshes;
	}

	// Read a scene. 'src' is assumed to point to the start of EChunkId::Scene chunk data
	template <typename TSrc> Scene ReadScene(TSrc& src, uint32_t len)
	{
		Scene scene;
		Find(src, len, [&](ChunkHeader hdr, TSrc& src)
			{
				switch (hdr.m_id)
				{
				case EChunkId::Materials:
					{
						scene.m_materials = ReadSceneMaterials(src, hdr.payload());
						break;
					}
				case EChunkId::Meshes:
					{
						scene.m_meshes = ReadSceneMeshes(src, hdr.payload());
						break;
					}
				}
				return false;
			});
		return scene;
	}

	// Read a p3d::File into memory from an istream-like source. Uses forward iteration only.
	template <typename TSrc> File Read(TSrc& src)
	{
		File file;

		// Check that this is actually a P3D stream
		auto main = Read<ChunkHeader>(src);
		if (main.m_id != EChunkId::Main)
			throw std::runtime_error("Source is not a p3d stream");

		// Read the sub chunks
		Find(src, main.payload(), [&](ChunkHeader hdr, TSrc& src)
			{
				switch (hdr.m_id)
				{
				case EChunkId::FileVersion:
					file.m_version = Read<uint32_t>(src);
					break;
				case EChunkId::Scene:
					file.m_scene = ReadScene(src, hdr.payload());
					break;
				}
				return false;
			});

		return file;
	}

	#pragma endregion

	#pragma region Utility

	// Extract the materials in the given P3D stream
	template <typename TSrc, typename TOut> void ExtractMaterials(TSrc& src, TOut out)
	{
		// Restore the src position on return
		auto save = SaveG(src);

		// Find the materials chunk
		auto materials = Find(src, ~0U, {EChunkId::Main, EChunkId::Scene, EChunkId::Materials});
		if (materials.m_id == p3d::EChunkId::Materials)
		{
			Find(src, materials.payload(), [&](p3d::ChunkHeader hdr, std::istream& src)
				{
					// Extract the material. 'out' returns true to stop
					if (hdr.m_id != EChunkId::Material) return false;
					return out(ReadMaterial(src, hdr.payload()));
				});
		}
	}

	// Extract the meshes from a P3D stream
	template <typename TSrc, typename TOut> void ExtractMeshes(TSrc& src, TOut out)
	{
		// Restore the src position on return
		auto save = SaveG(src);

		// Find the meshes chunk
		auto meshes = Find(src, ~0U, {EChunkId::Main, EChunkId::Scene, EChunkId::Meshes});
		if (meshes.m_id == EChunkId::Meshes)
		{
			// Read the meshes
			Find(src, meshes.payload(), [&](ChunkHeader hdr, TSrc& src)
				{
					// Extract the mesh. 'out' returns true to stop
					if (hdr.m_id != EChunkId::Mesh) return false;
					return out(ReadMesh(src, hdr.payload()));
				});
		}
	}

	// Write the p3d file as code
	template <typename TOut> void WriteAsCode(TOut& out, File const& file, char const* indent = "")
	{
		std::string ind = indent;
		for (auto& mesh : file.m_scene.m_meshes)
		{
			if (mesh.vcount() == 0)
				continue;

			// Mesh name
			out << "// " << mesh.m_name << "\n";

			// Write the model vertices
			out << ind << "#pragma region Verts\n";
			out << ind << "static pr::rdr::Vert const verts[] =\n";
			out << ind << "{\n";
			ind.push_back('\t');
			for (auto const& vert : mesh.fat_verts())
			{
				auto p = vert.m_vert;
				auto c = vert.m_diff;
				auto n = vert.m_norm;
				auto t = vert.m_tex0;
				out << ind
					<< "{"
					<< Fmt("{%+ff, %+ff, %+ff, 1.0f}, ", p.x, p.y, p.z)
					<< Fmt("{%+ff, %+ff, %+ff, %+ff}, ", c.r, c.g, c.b, c.a)
					<< Fmt("{%+ff, %+ff, %+ff, 0.0f}, ", n.x, n.y, n.z)
					<< Fmt("{%+ff, %+ff}", t.x, t.y)
					<< "},\n";
			}
			ind.pop_back();
			out << ind << "};\n";
			out << ind << "#pragma endregion\n";

			// Write the model indices
			out << ind << "#pragma region Indices\n";
			out << ind << "static " << (mesh.vcount() < 0x10000 ? "uint16_t" : "uint32_t") << " const idxs[] =\n";
			out << ind << "{\n";
			ind.push_back('\t');
			for (auto& nug : mesh.m_nugget)
			{
				out << ind << "// nugget " << (&nug - mesh.m_nugget.data()) << "\n";

				auto i = 0;
				for (auto idx : nug.m_vidx.span_as<int>())
				{
					out << (i == 0 ? ind : "");
					out << idx << ", ";
					out << (i == 31 ? "\n" : " ");
					++i %= 32;
				}
				out << (i != 0 ? "\n" : "");
			}
			ind.pop_back();
			out << ind << "};\n";
			out << ind << "#pragma endregion\n";

			// Write the model nuggets
			out << ind << "#pragma region Nuggets\n";
			out << ind << "static pr::rdr::NuggetProps const nuggets[] =\n";
			out << ind << "{\n";
			ind.push_back('\t');
			size_t ibeg = 0;
			for (auto& nug : mesh.m_nugget)
			{
				auto vrange = nug.vrange();
				out << ind 
					<< "pr::rdr::NuggetProps{"
					<< "pr::rdr::ETopo{" << s_cast<int>(nug.m_topo) << "}, "
					<< "pr::rdr::EGeom{" << s_cast<int>(nug.m_geom) << "}, "
					<< "nullptr, "
					<< "pr::rdr::Range{" << vrange.m_beg << "," << vrange.m_end << "}, "
					<< "pr::rdr::Range{" << ibeg << "," << ibeg + nug.icount() << "}"
					<< "},\n";

				ibeg += nug.icount();
			}
			ind.pop_back();
			out << ind << "};\n";
			out << ind << "#pragma endregion\n";

			// Write the bbox
			out << ind << "#pragma region BoundingBox\n";
			out << ind << "static pr::BBox const bbox =\n";
			out << ind << "{\n";
			ind.push_back('\t');
			out << ind << Fmt("{%+ff, %+ff, %+ff, 1.0f},\n", mesh.m_bbox.m_centre.x, mesh.m_bbox.m_centre.y, mesh.m_bbox.m_centre.z);
			out << ind << Fmt("{%+ff, %+ff, %+ff, 0.0f},\n", mesh.m_bbox.m_radius.x, mesh.m_bbox.m_radius.y, mesh.m_bbox.m_radius.z);
			ind.pop_back();
			out << ind << "}\n";
			out << ind << "#pragma endregion\n";
		}
	}
	
	// Write the p3d file as ldr script
	template <typename TOut> void WriteAsScript(TOut& out, File const& file, char const* indent = "")
	{
		auto ind = std::string{ indent };

		out << ind << "*Group {\n";
		ind.push_back('\t');

		// Add a *Mesh for each mesh in the scene
		for (auto& mesh : file.m_scene.m_meshes)
		{
			// No geometry in the mesh, skip...
			if (mesh.m_nugget.empty())
				continue;

			// Mesh
			out << ind << "*Mesh " << mesh.m_name << " {\n";
			ind.push_back('\t');

			// Verts
			if (mesh.m_vert.size() != 0)
			{
				out << ind << "*Verts {\n";
				ind.push_back('\t');

				for (int i = 0, iend = s_cast<int>(mesh.vcount()); i != iend; ++i)
				{
					auto& vert = mesh.m_vert[i];
					out << ind << FmtS("%f %f %f\n", vert.x, vert.y, vert.z);
				}

				ind.pop_back();
				out << ind << "}\n";
			}

			// Colours
			if (mesh.m_diff.size() != 0)
			{
				out << ind << "*Colours {\n";
				ind.push_back('\t');

				for (int i = 0, iend = s_cast<int>(mesh.vcount()); i != iend; ++i)
				{
					auto& diff = mesh.m_diff[i];
					out << ind << FmtS("%8.8X\n", diff.argb);
				}

				ind.pop_back();
				out << ind << "}\n";
			}

			// Normals
			if (mesh.m_norm.size() != 0)
			{
				out << ind << "*Normals {\n";
				ind.push_back('\t');

				for (int i = 0, iend = s_cast<int>(mesh.vcount()); i != iend; ++i)
				{
					auto& norm = mesh.m_norm[i];
					out << ind << FmtS("%f %f %f\n", norm.x, norm.y, norm.z);
				}

				ind.pop_back();
				out << ind << "}\n";
			}

			// UVs
			if (mesh.m_tex0.size() != 0)
			{
				out << ind << "*TexCoords {\n";
				ind.push_back('\t');

				for (int i = 0, iend = s_cast<int>(mesh.vcount()); i != iend; ++i)
				{
					auto& uv = mesh.m_tex0[i];
					out << ind << FmtS("%f %f\n", uv.x, uv.y);
				}

				ind.pop_back();
				out << ind << "}\n";
			}

			// Nuggets
			for (auto const& nug : mesh.m_nugget)
			{
				switch (nug.m_topo)
				{
					case ETopo::LineList:  out << ind << "*LineList {\n"; break;
					case ETopo::LineStrip: out << ind << "*LineStrip {\n"; break;
					case ETopo::TriList:   out << ind << "*TriList {\n"; break;
					case ETopo::TriStrip:  out << ind << "*TriStrip {\n"; break;
					default: throw std::runtime_error("Unsupported topology type");
				}
				ind.push_back('\t');

				// Indices
				auto i = 0U;
				for (auto const& vi : nug.m_vidx.span_as<int>())
				{
					out << (i == 0 ? ind : "");
					out << vi;
					out << (i == 15 ? "\n" : " ");
					++i %= 16;
				}
				out << (i != 0 ? "\n" : "");
				ind.pop_back();
				out << ind << "}\n";
			}

			ind.pop_back();
			out << ind << "}\n";
		}

		ind.pop_back();
		out << ind << "}\n";
	}

	#pragma endregion
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::geometry
{
	PRUnitTest(P3dTests)
	{
		using namespace pr::geometry::p3d;

		//{
		//	std::ifstream ifile("S:\\dump\\sketchup exports\\foot.p3d", std::ios::binary);
		//	std::ofstream ofile("P:\\dump\\foot.p3d", std::ios::binary);
		//	auto m = p3d::Read(ifile);
		//	p3d::Write(ofile, m, p3d::EFlags::Default);
		//}

		// Create a text p3d file
		File file;
		{
			Texture tex;
			tex.m_filepath = "filepath";
			tex.m_type = Texture::EType::Diffuse;
			tex.m_addr_mode = Texture::EAddrMode::Wrap;
			tex.m_flags = Texture::EFlags::None;

			Material mat{ "mat01", ColourWhite };
			mat.m_textures.push_back(tex);

			p3d::Mesh mesh{ "tri" };
			mesh.m_bbox = BBox{ v4Origin, v4{1, 3, 0, 0} };
			mesh.m_vert.assign({
				v4{-1, -1, 0, 1},
				v4{+1, -1, 0, 1},
				v4{+0, +2, 0, 1},
				});
			mesh.m_diff.assign({
				Colour32Red,
				Colour32Green,
				Colour32Blue,
				});
			mesh.m_norm.assign({
				v4{0, 0, 1, 0},
				});
			mesh.m_tex0.assign({
				v2{0.0f, 1.0f},
				v2{0.5f, 0.0f},
				v2{1.0f, 1.0f},
				});

			Nugget nug{ ETopo::TriList, EGeom::All, "mat01", sizeof(uint16_t) };
			nug.m_vidx.append<uint16_t>({ 0, 1, 2 });
			mesh.m_nugget.emplace_back(std::move(nug));

			file.m_version = Version;
			file.m_scene.m_materials.push_back(mat);
			file.m_scene.m_meshes.push_back(mesh);
		}

		// Write the file to a file stream
		//{
		//	std::ofstream buf("\\dump\\test.p3d", std::ofstream::binary);
		//	auto size = Write(buf, file, EFlags::Default);
		//	PR_EXPECT(size_t(buf.tellp()) == size_t(size));
		//}

		// Read the file from a stream and compare contents
		{
			// Write the file to a stream
			std::stringstream buf(std::ios_base::in | std::ios_base::out | std::ios_base::binary);
			{
				auto size = Write(buf, file, EFlags::Default);
				PR_EXPECT(size_t(buf.tellp()) == size_t(size));
			}

			//*
			File cmp;
			try { cmp = std::move(Read(buf)); }
			catch (std::exception const& ex)
			{
				OutputDebugStringA(ex.what());
			}

			PR_EXPECT(cmp.m_version == file.m_version);
			PR_EXPECT(cmp.m_scene.m_materials.size() == file.m_scene.m_materials.size());
			PR_EXPECT(cmp.m_scene.m_meshes.size() == file.m_scene.m_meshes.size());
			for (size_t i = 0; i != file.m_scene.m_materials.size(); ++i)
			{
				auto& m0 = cmp.m_scene.m_materials[i];
				auto& m1 = file.m_scene.m_materials[i];
				PR_EXPECT(pr::str::Equal(m0.m_id.str, m1.m_id.str));
				PR_EXPECT(m0.m_diffuse == m1.m_diffuse);
				PR_EXPECT(m0.m_textures.size() == m1.m_textures.size());
				for (size_t j = 0; j != m1.m_textures.size(); ++j)
				{
					auto& t0 = m0.m_textures[j];
					auto& t1 = m1.m_textures[j];
					PR_EXPECT(t0.m_filepath == t1.m_filepath);
					PR_EXPECT(t0.m_type == t1.m_type);
					PR_EXPECT(t0.m_addr_mode == t1.m_addr_mode);
					PR_EXPECT(t0.m_flags == t1.m_flags);
				}
			}
			for (size_t i = 0; i != file.m_scene.m_meshes.size(); ++i)
			{
				auto& m0 = cmp.m_scene.m_meshes[i];
				auto& m1 = file.m_scene.m_meshes[i];
				PR_EXPECT(m0.m_name == m1.m_name);
				PR_EXPECT(m0.m_vert.size() == m1.m_vert.size());
				PR_EXPECT(m0.m_diff.size() == m1.m_diff.size());
				PR_EXPECT(m0.m_norm.size() == m1.m_norm.size());
				PR_EXPECT(m0.m_tex0.size() == m1.m_tex0.size());
				PR_EXPECT(m0.m_nugget.size() == m1.m_nugget.size());
				for (int j = 0; j != (int)m1.m_vert.size(); ++j)
				{
					auto& v0 = m0.m_vert[j];
					auto& v1 = m1.m_vert[j];
					PR_EXPECT(v0 == v1);
				}
				for (int j = 0; j != (int)m1.m_diff.size(); ++j)
				{
					auto& c0 = m0.m_diff[j];
					auto& c1 = m1.m_diff[j];
					PR_EXPECT(c0 == c1);
				}
				for (int j = 0; j != (int)m1.m_norm.size(); ++j)
				{
					auto& n0 = m0.m_norm[j];
					auto& n1 = m1.m_norm[j];
					PR_EXPECT(n0 == n1);
				}
				for (int j = 0; j != (int)m1.m_tex0.size(); ++j)
				{
					auto& t0 = m0.m_tex0[j];
					auto& t1 = m1.m_tex0[j];
					PR_EXPECT(t0 == t1);
				}
				for (int j = 0; j != (int)m1.m_nugget.size(); ++j)
				{
					auto& n0 = m0.m_nugget[j];
					auto& n1 = m1.m_nugget[j];
					PR_EXPECT(n0.m_topo == n1.m_topo);
					PR_EXPECT(n0.m_geom == n1.m_geom);
					PR_EXPECT(n0.m_mat == n1.m_mat);
					PR_EXPECT(n0.m_vidx.size() == n1.m_vidx.size());
					PR_EXPECT(n0.m_vidx.stride() == n1.m_vidx.stride());
					for (int k = 0, kend = s_cast<int>(n1.icount()); k != kend; ++k)
					{
						auto i0 = n0.m_vidx[k];
						auto i1 = n1.m_vidx[k];
						PR_EXPECT(i0 == i1);
					}
				}
			}
			//*/
		}

		// Write the file as C++ code
		{
			std::stringstream script;
			//std::ofstream script("\\dump\\test.h");
			WriteAsCode(script, file);
		}

		// Write the file as a script
		{
			std::stringstream script;
			//std::ofstream script("\\dump\\test.ldr");
			WriteAsScript(script, file);
		}

		// IdxBuf iteration test
		{
			IdxBuf vidx{ sizeof(uint16_t) };
			vidx.append<uint16_t>({ 0, 1, 2, 3 });

			int i = 0;
			for (auto j : vidx.span_as<short>())
				PR_EXPECT(j == (short)i++);
		}

		// Fat vertex iteration
		{
			auto const& mesh = file.m_scene.m_meshes[0];

			std::vector<FatVert> fat;
			for (auto const& v : mesh.fat_verts())
				fat.push_back(v);

			PR_EXPECT(fat[0].m_vert == v4(-1, -1, 0, 1));
			PR_EXPECT(fat[1].m_diff == Colour(Colour32Green));
			PR_EXPECT(fat[2].m_norm == v4(0, 0, 1, 0));
			PR_EXPECT(fat[1].m_tex0 == v2(0.5f, 0.0f));
			PR_EXPECT(fat[2].pad == v2(0.0f, 0.0f));
		}
		//std::ifstream ifile("\\dump\\test2.3ds", std::ifstream::binary);
		//Read3DSMaterials(ifile, [](max_3ds::Material&&){});
		//Read3DSModels(ifile, [](max_3ds::Object&&){});
	}
}
#endif