//********************************
// glTF Model file format
//  Copyright (c) Rylogic Ltd 2025
//********************************
// Notes:
//  - glTF scenes are a hierarchy of nodes. The scene's root nodes are the entry points.
//    Meshes, skins, cameras, and lights are attached to nodes.
//  - All cgltf types are hidden within the dll.
//  - To avoid making this a build dependency, this header will dynamically load 'gltf.dll' as needed.
//  - glTF uses a right-handed coordinate system with Y-up, and meters as the unit of length.
//  - glTF supports both .gltf (JSON) and .glb (binary) file formats.

#pragma once
#include <memory>
#include <string>
#include <string_view>
#include "pr/common/range.h"
#include "pr/common/flags_enum.h"
#include "pr/maths/maths.h"
#include "pr/gfx/colour.h"
#include "pr/geometry/common.h"
#include "pr/win32/win32.h"
// No cgltf types here

namespace pr::geometry::gltf
{
	static constexpr uint32_t NoId = ~0UL;
	static constexpr int NoIndex = -1;

	// Opaque types
	struct SceneData;
	struct Context;

	// Parts of a glTF Scene
	using EParts = ESceneParts;

	// Interpolation modes
	enum class EInterpolation
	{
		Step = 0,
		Linear = 1,
		CubicSpline = 2,
	};

	// Error handling
	struct ErrorHandler
	{
		using FuncCB = void(*)(void*, char const* msg, size_t len);

		void* m_ctx;
		FuncCB m_cb;

		void operator()(std::string_view message) const
		{
			if (m_cb) m_cb(m_ctx, message.data(), message.size());
			else throw std::runtime_error(std::string(message));
		}
	};

	// Scene load options
	struct LoadOptions
	{
		// Preferences
		bool ignore_geometry;    // Do not load geometry data (vertices, indices, etc)
		bool ignore_animation;   // Do not load animation data
		bool ignore_all_content; // Do not load any content

		// Generate vertex normals for meshes that are missing normals.
		bool generate_missing_normals;

		// Filename hint for resolving external buffer files when loading from a stream.
		// Provide the original file path so that relative .bin URIs can be resolved.
		std::string_view filename;
	};

	// Metadata in the scene
	struct SceneProps
	{
		// Scene objects available
		int m_material_count = 0;
		int m_mesh_count = 0;
		int m_skin_count = 0;
		int m_animation_count = 0;
		int m_node_count = 0;
	};

	// Options for parsing glTF files
	struct ReadOptions
	{
		// Parts of the scene to read
		EParts m_parts = EParts::All;

		// The subset of meshes to load. Empty means load all. Returning true means load
		std::function<bool(std::string_view)> m_mesh_filter = {};

		// The subset of skeletons to load. Empty means load all. Returning true means load
		std::function<bool(std::string_view)> m_skel_filter = {};

		// The subset of animations to load. Empty means load all. Returning true means load
		std::function<bool(std::string_view)> m_anim_filter = {};

		// Progress callback
		std::function<bool(int64_t step, int64_t total, char const* message, int nest)> m_progress = {};
	};

	// Options for outputting the glTF scene dump
	struct DumpOptions
	{
		// Parts of the scene to dump
		EParts m_parts = EParts::All;

		// The number to cap output of arrays at
		int m_summary_length = 10;
	};

	// Model types
	struct Vert
	{
		v4 m_vert = v4::Origin();
		Colour m_colr = ColourWhite;
		v4 m_norm = v4::Zero();
		v2 m_tex0 = v2::Zero();
		iv2 m_idx0 = {};

		friend bool operator == (Vert const& lhs, Vert const& rhs)
		{
			return std::memcmp(&lhs, &rhs, sizeof(Vert)) == 0;
		}
	};
	struct Nugget
	{
		uint32_t m_mat_id = NoId;
		ETopo m_topo = ETopo::TriList;
		EGeom m_geom = EGeom::Vert;
		Range<int64_t> m_vrange = Range<int64_t>::Reset();
		Range<int64_t> m_irange = Range<int64_t>::Reset();
	};
	struct Material
	{
		uint32_t m_mat_id = NoId;
		std::string_view m_name = {};
		Colour m_ambient = ColourBlack;
		Colour m_diffuse = ColourWhite;
		Colour m_specular = ColourZero;
		std::string_view m_tex_diff = {};
	};
	struct Skin
	{
		uint32_t m_skel_id = NoId;         // The skeleton that this skin is based on
		std::span<int const> m_offsets;    // Index offset to the first influence for each vertex
		std::span<uint32_t const> m_bones; // The Ids of the bones that influence a vertex
		std::span<float const> m_weights;  // The influence weights

		// The number of vertices influenced by this skin
		int vert_count() const
		{
			return static_cast<int>(ssize(m_offsets) - 1);
		}
		int influence_count(int vidx) const
		{
			return static_cast<int>(m_offsets[vidx+1] - m_offsets[vidx]);
		}
		explicit operator bool() const
		{
			return !m_offsets.empty() && m_offsets.back() != 0;
		}
	};
	struct Skeleton
	{
		uint32_t m_skel_id = NoId;                 // Unique skeleton Id
		std::string_view m_name;                   // Skeleton name
		std::span<uint32_t const> m_bone_ids;      // Bone unique ids (first is the root bone)
		std::span<std::string const> m_bone_names; // Bone names
		std::span<m4x4 const> m_o2bp;              // Inverse of the bind-pose to root-object-space transform for each bone
		std::span<int const> m_hierarchy;          // Hierarchy levels. level == 0 are root bones.

		// The number of bones in this skeleton
		int size() const
		{
			return isize(m_bone_ids);
		}

		// Create a lookup table from bone id to bone index
		std::unordered_map<uint32_t, int> BoneIndexMap() const
		{
			std::unordered_map<uint32_t, int> map;
			map.reserve(s_cast<size_t>(size()));
			for (auto const& id : m_bone_ids) map[id] = s_cast<int>(&id - m_bone_ids.data());
			return map;
		}
	};
	struct Animation
	{
		// Notes:
		//  - Bone transform data are stored interleaved for each frame, e.g.,
		//    m_rotation: [frame0:(bone0,bone1,bone2,..)][frame1:(bone0,bone1,bone2,..)][...
		//    m_position: [frame0:(bone0,bone1,bone2,..)][frame1:(bone0,bone1,bone2,..)][...
		//    m_scale:    [frame0:(bone0,bone1,bone2,..)][frame1:(bone0,bone1,bone2,..)][...
		//   This is because it's more cache friendly to have all data for a frame local in memory.
		uint32_t m_skel_id = NoId;            // The skeleton that these tracks should match
		double m_duration;                    // The length (in seconds) of the animation
		double m_frame_rate;                  // The native frame rate of the animation
		std::string_view m_name;              // Animation name
		std::span<uint16_t const> m_bone_map; // The bone id for each track. Length = bone count.
		std::span<quat const> m_rotation;     // Frames of bone rotations
		std::span<v3 const> m_position;       // Frames of bone positions
		std::span<v3 const> m_scale;          // Frames of bone scales
	};
	struct Mesh
	{
		uint32_t m_mesh_id = NoId;
		std::string_view m_name;
		std::span<Vert const> m_vbuf;
		std::span<int const> m_ibuf;
		std::span<Nugget const> m_nbuf;
		Skin m_skin;
		BBox m_bbox;
	};
	struct MeshTree
	{
		m4x4 m_o2p;              // The node to parent transform
		std::string_view m_name; // Node of the mesh instance
		uint32_t m_mesh_id;      // The previously created mesh
		int m_level;             // The node hierarchy level
	};

	// Output interface for 'Read'
	struct IReadOutput
	{
		virtual ~IReadOutput() = default;
		
		// Create a user-side mesh from 'mesh' and return an opaque handle to it (or null)
		virtual void CreateMesh(gltf::Mesh const& mesh, std::span<gltf::Material const> materials) { (void)mesh, materials; }

		// Create a model from a hierarchy of mesh instances. Return true to continue
		virtual void CreateModel(std::span<gltf::MeshTree const> mesh_tree) { (void)mesh_tree; }

		// Create a skeleton from a hierarchy of bone instances. Return true to continue
		virtual void CreateSkeleton(gltf::Skeleton const& skel) { (void)skel; }

		// Create an animation. Return true to continue
		virtual bool CreateAnimation(gltf::Animation const& anim) { (void)anim; return false; }
	};

	// Dynamically loaded glTF dll
	class Gltf
	{
		// Gltf is private only, only friends can access the API functions.
		friend struct Scene;
		friend struct Mesh;
		friend struct Material;
		friend struct Skin;

		// Dll module handle
		HMODULE m_module;

		#define PR_GLTF_API(x)\
		x(       , Initialise , Context* (__stdcall*)(ErrorHandler error_cb))\
		x(       , Release    , void (__stdcall*)(Context* ctx))\
		x(Scene_ , LoadFile   , SceneData* (__stdcall*)(Context& ctx, char const* filepath, LoadOptions const& opts))\
		x(Scene_ , Load       , SceneData* (__stdcall*)(Context& ctx, std::istream& src, LoadOptions const& opts))\
		x(Scene_ , Read       , void (__stdcall*)(Context& ctx, SceneData& scene, ReadOptions const& options, IReadOutput& out))\
		x(Scene_ , Dump       , void (__stdcall*)(Context& ctx, SceneData const& scene, DumpOptions const& options, std::ostream& out))
		#define PR_GLTF_FUNCTION_MEMBERS(prefix, name, function_type) using prefix##name##Fn = function_type; prefix##name##Fn prefix##name = {};
		PR_GLTF_API(PR_GLTF_FUNCTION_MEMBERS)
		#undef PR_GLTF_FUNCTION_MEMBERS

		Gltf()
			: m_module(win32::LoadDll<struct GltfDll>("gltf.dll"))
		{
			#pragma warning(push)
			#pragma warning(disable: 4191) // 'reinterpret_cast': unsafe conversion from 'FARPROC' to function pointer
			#define PR_GLTF_GET_PROC_ADDRESS(prefix, name, function_type) prefix##name = reinterpret_cast<prefix##name##Fn>(GetProcAddress(m_module, "Gltf_" #prefix #name));
			PR_GLTF_API(PR_GLTF_GET_PROC_ADDRESS)
			#undef PR_GLTF_GET_PROC_ADDRESS
			#pragma warning(pop)
		}

		// Singleton Instance
		static Gltf& get() { static Gltf s_this; return s_this; }
		static void StaticChecks();
	};

	// A loaded glTF scene
	struct Scene
	{
		// The dll context
		Context* m_ctx;

		// The loaded scene
		SceneData* m_scene;

		// Load from a stream
		Scene(std::istream& src, LoadOptions const& opts = {}, ErrorHandler error_cb = {})
			: m_ctx(Gltf::get().Initialise(error_cb))
			, m_scene(Gltf::get().Scene_Load(*m_ctx, src, opts))
		{}

		// Load from a file path
		Scene(char const* filepath, LoadOptions const& opts = {}, ErrorHandler error_cb = {})
			: m_ctx(Gltf::get().Initialise(error_cb))
			, m_scene(Gltf::get().Scene_LoadFile(*m_ctx, filepath, opts))
		{}

		Scene(Scene&& rhs) noexcept
			: m_ctx()
			, m_scene()
		{
			std::swap(m_ctx, rhs.m_ctx);
			std::swap(m_scene, rhs.m_scene);
		}
		Scene(Scene const&) = delete;
		Scene& operator=(Scene&& rhs) noexcept
		{
			std::swap(m_ctx, rhs.m_ctx);
			std::swap(m_scene, rhs.m_scene);
			return *this;
		}
		Scene& operator=(Scene const&) = delete;
		~Scene()
		{
			if (m_ctx)
				Gltf::get().Release(m_ctx);
		}

		// Emit meshes/skeletons/etc
		void Read(IReadOutput& out, ReadOptions const& options)
		{
			Gltf::get().Scene_Read(*m_ctx, *m_scene, options, out);
		}

		// Read the full scene into memory
		void Dump(std::ostream& out, DumpOptions const& options)
		{
			Gltf::get().Scene_Dump(*m_ctx, *m_scene, options, out);
		}
	};
}
