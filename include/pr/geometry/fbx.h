//********************************
// FBX Model file format
//  Copyright (c) Rylogic Ltd 2018
//********************************
// Notes:
//  - FBX scenes are a hierarchy of ufbx_nodes. Under the Root node are trees of nodes representing
//    meshes, skeletons, lights, cameras, etc. These trees are serialised depth-first into arrays.
//    e.g.,
//             A
//           /   \
//          B     C
//        / | \   |
//       D  E  F  G
//    Serialised as: A0 B1 D2 E2 F2 C1 G2
//    Children = all nodes to the right with level > the current.
//  - All ufbx types are hidden within the dll.
//  - To avoid making this a build dependency, this header will dynamically load 'fbx.dll' as needed.
//
// Blender Export Settings:
//  - If exporting from blender use:
//    include:
//       Object Types: Mesh, Armature,
//    Transform:
//        Scale: 1.0
//        Apply Settings: All Local
//        Forward: -Z Forward
//        Up: Y Up
//        Apply Unit: Yes
//        Use Space Transform: Yes
//        Apply Transform: No
//    Geometry:
//        Whatever you like
//    Armature:
//        Primary: Y Axis
//        Secondary: X Axis
//        Armature FBXNode Type: Null
//        Only Deform Bones: No
//        Add Leaf Bones: No

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
// No ufbx types here

namespace pr::geometry::fbx
{
	static constexpr uint32_t NoId = ~0UL;
	static constexpr int NoIndex = -1;
	using TimeRange = Range<double>;

	// Opaque types
	struct SceneData;
	struct Context;

	// Parts of an FBX Scene
	using EParts = ESceneParts;

	// Fbx File Formats
	enum class EFormat
	{
		Binary,
		Ascii,
	};

	// Axis systems the scene can be converted to
	enum class ECoordAxis
	{
		PosX,
		NegX,
		PosY,
		NegY,
		PosZ,
		NegZ,
		Unknown,
	};

	// Animation channels
	enum class EAnimChannel
	{
		None = 0,
		Rotation = 1 << 0,
		Position = 1 << 1,
		Scale = 1 << 2,
		_flags_enum = 0,
	};

	// Interpolation modes
	enum class EInterpolation
	{
		Constant = 0,
		Linear = 1,
		Cubic = 2,
	};

	// Specify how unit / coordinate system conversion should be performed.
	enum class ESpaceConversion
	{
		// Notes:
		//  Affects how `ufbx_load_opts.target_axes` and `ufbx_load_opts.target_unit_meters` work,
		//  has no effect if neither is specified.

		// Store the space conversion transform in the root node.
		// Sets `ufbx_node.local_transform` of the root node.
		TransformRoot,

		// Perform the conversion by using "adjust" transforms.
		// Compensates for the transforms using `ufbx_node.adjust_pre_rotation` and
		// `ufbx_node.adjust_pre_scale`. You don't need to account for these unless
		// you are manually building transforms from `ufbx_props`.
		AdjustTransforms,

		// Perform the conversion by scaling geometry in addition to adjusting transforms.
		// Compensates transforms like `UFBX_SPACE_CONVERSION_ADJUST_TRANSFORMS` but
		// applies scaling to geometry as well.
		ModifyGeometry,
	};

	// How to handle FBX transform pivots.
	enum class EPivotHandling
	{
		// Take pivots into account when computing the transform.
		Retain,

		// Translate objects to be located at their pivot.
		// NOTE: Only applied if rotation and scaling pivots are equal.
		// NOTE: Results in geometric translation. Use `ufbx_geometry_transform_handling`
		// to interpret these in a standard scene graph.
		AdjustToPivot,

		// Translate objects to be located at their rotation pivot.
		// NOTE: Results in geometric translation. Use `ufbx_geometry_transform_handling`
		// to interpret these in a standard scene graph.
		// NOTE: By default the original transforms of empties are not retained when using this,
		// use `ufbx_load_opts.pivot_handling_retain_empties` to prevent adjusting these pivots.
		AdjustToRotationPivot,
	};

	// How to handle FBX node geometry transforms.
	enum class EGeometryTransformHandling
	{
		// Notes:
		//  FBX nodes can have "geometry transforms" that affect only the attached meshes,
		//  but not the children. This is not allowed in many scene representations so
		//  ufbx provides some ways to simplify them.
		//  Geometry transforms can also be used to transform any other attributes such
		//  as lights or cameras.

		// Preserve the geometry transforms as-is.
		// To be correct for all files you have to use `ufbx_node.geometry_transform`,
		// `ufbx_node.geometry_to_node`, or `ufbx_node.geometry_to_world` to compensate
		// for any potential geometry transforms.
		Preserve,

		// Add helper nodes between the nodes and geometry where needed.
		// The created nodes have `ufbx_node.is_geometry_transform_helper` set and are
		// named `ufbx_load_opts.geometry_transform_helper_name`.
		HelperNodes,

		// Modify the geometry of meshes attached to nodes with geometry transforms.
		// Will add helper nodes like `UFBX_GEOMETRY_TRANSFORM_HANDLING_HELPER_NODES` if
		// necessary, for example if there are multiple instances of the same mesh with
		// geometry transforms.
		ModifyGeometry,

		// Modify the geometry of meshes attached to nodes with geometry transforms.
		// NOTE: This will not work correctly for instanced geometry.
		ModifyGeometryNoFallback,
	};

	// Axis used to mirror transformations for handedness conversion.
	enum class EMirrorAxis
	{
		None,
		X,
		Y,
		Z,
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

	// Representation of a coordinate system
	struct CoordAxes
	{
		ECoordAxis right;
		ECoordAxis up;
		ECoordAxis front; // means out of the screen direction
	};

	// A rotation, translation, scale transform
	struct Transform
	{
		quat rotation;
		v3 translation;
		v3 scale;
	};

	// Scene load options
	struct LoadOptions
	{
		#if 0
		ufbx_allocator_opts temp_allocator;   // < Allocator used during loading
		ufbx_allocator_opts result_allocator; // < Allocator used for the final scene
		ufbx_thread_opts thread_opts;         // < Threading options
		#endif

		// Preferences
		bool ignore_geometry;    // < Do not load geometry datsa (vertices, indices, etc)
		bool ignore_animation;   // < Do not load animation curves
		bool ignore_embedded;    // < Do not load embedded content
		bool ignore_all_content; // < Do not load any content (geometry, animation, embedded)

		bool evaluate_skinning; // < Evaluate skinning (see ufbx_mesh.skinned_vertices)
		bool evaluate_caches;   // < Evaluate vertex caches (see ufbx_mesh.skinned_vertices)

		// Try to open external files referenced by the main file automatically.
		// Applies to geometry caches and .mtl files for OBJ.
		// NOTE: This may be risky for untrusted data as the input files may contain
		// references to arbitrary paths in the filesystem.
		// NOTE: This only applies to files *implicitly* referenced by the scene, if
		// you request additional files via eg. `ufbx_load_opts.obj_mtl_path` they
		// are still loaded.
		// NOTE: Will fail loading if any external files are not found by default, use
		// `ufbx_load_opts.ignore_missing_external_files` to suppress this, in this case
		// you can find the errors at `ufbx_metadata.warnings[]` as `UFBX_WARNING_MISSING_EXTERNAL_FILE`.
		bool load_external_files;

		// Don't fail loading if external files are not found.
		bool ignore_missing_external_files;

		// Don't compute `ufbx_skin_deformer` `vertices` and `weights` arrays saving
		// a bit of memory and time if not needed
		bool skip_skin_vertices;

		// Skip computing `ufbx_mesh.material_parts[]` and `ufbx_mesh.face_group_parts[]`.
		bool skip_mesh_parts;

		// Clean-up skin weights by removing negative, zero and NAN weights.
		bool clean_skin_weights;

		// Read Blender materials as PBR values.
		// Blender converts PBR materials to legacy FBX Phong materials in a deterministic way.
		// If this setting is enabled, such materials will be read as `UFBX_SHADER_BLENDER_PHONG`,
		// which means ufbx will be able to parse roughness and metallic textures.
		bool use_blender_pbr_material;

		// Don't adjust reading the FBX file depending on the detected exporter
		bool disable_quirks;

		// Don't allow partially broken FBX files to load
		bool strict;

		// Force ASCII parsing to use a single thread.
		// The multi-threaded ASCII parsing is slightly more lenient as it ignores
		// the self-reported size of ASCII arrays, that threaded parsing depends on.
		bool force_single_thread_ascii_parsing;

		#if 0
		// UNSAFE: If enabled allows using unsafe options that may fundamentally
		// break the API guarantees.
		ufbx_unsafe bool allow_unsafe;

		// Specify how to handle broken indices.
		ufbx_index_error_handling index_error_handling;
		#endif

		// Connect related elements even if they are broken. If `false` (default)
		// `ufbx_skin_cluster` with a missing `bone` field are _not_ included in
		// the `ufbx_skin_deformer.clusters[]` array for example.
		bool connect_broken_elements;

		// Allow nodes that are not connected in any way to the root. Conversely if
		// disabled, all lone nodes will be parented under `ufbx_scene.root_node`.
		bool allow_nodes_out_of_root;

		// Allow meshes with no vertex position attribute.
		// NOTE: If this is set `ufbx_mesh.vertex_position.exists` may be `false`.
		bool allow_missing_vertex_position;

		// Allow faces with zero indices.
		bool allow_empty_faces;

		// Generate vertex normals for a meshes that are missing normals.
		// You can see if the normals have been generated from `ufbx_mesh.generated_normals`.
		bool generate_missing_normals;

		// Ignore `open_file_cb` when loading the main file.
		bool open_main_file_with_default;

		// Path separator character, defaults to '\' on Windows and '/' otherwise.
		char path_separator;

		// Maximum depth of the node hirerachy.
		// Will fail with `UFBX_ERROR_NODE_DEPTH_LIMIT` if a node is deeper than this limit.
		// NOTE: The default of 0 allows arbitrarily deep hierarchies. Be careful if using
		// recursive algorithms without setting this limit.
		uint32_t node_depth_limit;

		// Estimated file size for progress reporting
		uint64_t file_size_estimate;

		// Buffer size in bytes to use for reading from files or IO callbacks
		size_t read_buffer_size;

		// Filename to use as a base for relative file paths if not specified using
		// `ufbx_load_file()`. Use `length = SIZE_MAX` for NULL-terminated strings.
		// `raw_filename` will be derived from this if empty.
		std::string_view filename;

		#if 0
		// Raw non-UTF8 filename. Does not support NULL termination.
		// `filename` will be derived from this if empty.
		ufbx_blob raw_filename;

		// Progress reporting
		ufbx_progress_cb progress_cb;
		uint64_t progress_interval_hint; // < Bytes between progress report calls

		// External file callbacks (defaults to stdio.h)
		ufbx_open_file_cb open_file_cb;
		#endif

		// How to handle geometry transforms in the nodes. See `ufbx_geometry_transform_handling` for an explanation.
		EGeometryTransformHandling geometry_transform_handling;

		#if 0
		// How to handle unconventional transform inherit modes. See `ufbx_inherit_mode_handling` for an explanation.
		ufbx_inherit_mode_handling inherit_mode_handling;
		#endif

		// How to perform space conversion by `target_axes` and `target_unit_meters`. See `ufbx_space_conversion` for an explanation.
		ESpaceConversion space_conversion;
		
		// How to handle pivots. See `ufbx_pivot_handling` for an explanation.
		EPivotHandling pivot_handling;

		// Retain the original transforms of empties when converting pivots.
		bool pivot_handling_retain_empties;

		// Axis used to mirror for conversion between left-handed and right-handed coordinates.
		EMirrorAxis handedness_conversion_axis;

		// Do not change winding of faces when converting handedness.
		bool handedness_conversion_retain_winding;

		// Reverse winding of all faces.
		// If `handedness_conversion_retain_winding` is not specified, mirrored meshes
		// will retain their original winding.
		bool reverse_winding;

		// Apply an implicit root transformation to match axes.
		// Used if `ufbx_coordinate_axes_valid(target_axes)`.
		CoordAxes target_axes;

		// Scale the scene so that one world-space unit is `target_unit_meters` meters.
		// By default units are not scaled.
		double target_unit_meters;

		// Target space for camera.
		// By default FBX cameras point towards the positive X axis.
		// Used if `ufbx_coordinate_axes_valid(target_camera_axes)`.
		CoordAxes target_camera_axes;

		// Target space for directed lights.
		// By default FBX lights point towards the negative Y axis.
		// Used if `ufbx_coordinate_axes_valid(target_light_axes)`.
		CoordAxes target_light_axes;

		#if 0
		// Name for dummy geometry transform helper nodes.
		// See `UFBX_GEOMETRY_TRANSFORM_HANDLING_HELPER_NODES`.
		ufbx_string geometry_transform_helper_name;

		// Name for dummy scale helper nodes.
		// See `UFBX_INHERIT_MODE_HANDLING_HELPER_NODES`.
		ufbx_string scale_helper_name;
		#endif

		// Normalize vertex normals.
		bool normalize_normals;

		// Normalize tangents and bitangents.
		bool normalize_tangents;

		// Override for the root transform
		bool use_root_transform;
		Transform root_transform;

		// Animation keyframe clamp threshold, only applies to specific interpolation modes.
		double key_clamp_threshold;

		#if 0
		// Specify how to handle Unicode errors in strings.
		ufbx_unicode_error_handling unicode_error_handling;

		// Retain the 'W' component of mesh normal/tangent/bitangent.
		// See `ufbx_vertex_attrib.values_w`.
		bool retain_vertex_attrib_w;

		// Retain the raw document structure using `ufbx_dom_node`.
		bool retain_dom;

		// Force a specific file format instead of detecting it.
		ufbx_file_format file_format;

		// How far to read into the file to determine the file format.
		// Default: 16kB
		size_t file_format_lookahead;

		// Do not attempt to detect file format from file content.
		bool no_format_from_content;

		// Do not attempt to detect file format from filename extension.
		// ufbx primarily detects file format from the file header,
		// this is just used as a fallback.
		bool no_format_from_extension;

		// (.obj) Try to find .mtl file with matching filename as the .obj file.
		// Used if the file specified `mtllib` line is not found, eg. for a file called
		// `model.obj` that contains the line `usemtl materials.mtl`, ufbx would first
		// try to open `materials.mtl` and if that fails it tries to open `model.mtl`.
		bool obj_search_mtl_by_filename;

		// (.obj) Don't split geometry into meshes by object.
		bool obj_merge_objects;

		// (.obj) Don't split geometry into meshes by groups.
		bool obj_merge_groups;

		// (.obj) Force splitting groups even on object boundaries.
		bool obj_split_groups;

		// (.obj) Path to the .mtl file.
		// Use `length = SIZE_MAX` for NULL-terminated strings.
		// NOTE: This is used _instead_ of the one in the file even if not found
		// and sidesteps `load_external_files` as it's _explicitly_ requested.
		ufbx_string obj_mtl_path;

		// (.obj) Data for the .mtl file.
		ufbx_blob obj_mtl_data;

		// The world unit in meters that .obj files are assumed to be in.
		// .obj files do not define the working units. By default the unit scale
		// is read as zero, and no unit conversion is performed.
		ufbx_real obj_unit_meters;

		// Coordinate space .obj files are assumed to be in.
		// .obj files do not define the coordinate space they use. By default no
		// coordinate space is assumed and no conversion is performed.
		ufbx_coordinate_axes obj_axes;
		#endif
	};

	// Metadata in the scene
	struct SceneProps
	{
		// The number of animations in the scene
		int m_animation_stack_count = 0;

		// The animation frame rate
		double m_frame_rate = 0;

		// Scene objects available (i.e. in the scene, but not necessarily loaded)
		int m_material_available = 0;
		int m_meshes_available = 0;
		int m_skeletons_available = 0;
		int m_animations_available = 0;

		// Scene object counts (loaded scene objects)
		int m_material_count = 0;
		int m_mesh_count = 0;
		int m_skeleton_count = 0;
		int m_animation_count = 0;

		// Names of the root mesh nodes
		std::span<std::string const> m_mesh_names;

		// Names of the root bone nodes
		std::span<std::string const> m_skel_names;
	};

	// Options for parsing FBXfiles
	struct ReadOptions
	{
		// Parts of the scene to read
		EParts m_parts = EParts::All;

		// The animation frame range to read
		pr::Range<int> m_frame_range = { 0, std::numeric_limits<int>::max() };

		// The subset of meshes to load. Empty means load all. Returning true means load
		std::function<bool(std::string_view)> m_mesh_filter = {};

		// The subset of skeletons to load. Empty means load all. Returning true means load
		std::function<bool(std::string_view)> m_skel_filter = {};

		// The subset of animations to load. Empty means load all. Returning true means load
		std::function<bool(std::string_view)> m_anim_filter = {};

		// Progress callback
		std::function<bool(int64_t step, int64_t total, char const* message, int nest)> m_progress = {};
	};

	// Options for outputting the FBX scene dump
	struct DumpOptions
	{
		// Parts of the scene to dump
		EParts m_parts = EParts::All;

		// The number to cap output of arrays at
		int m_summary_length = 10;

		// Run triangulation on meshes before outputting them
		bool m_triangulate_meshes = false;
	};

	// Model types
	struct Vert
	{
		v4 m_vert = {};
		Colour m_colr = {};
		v4 m_norm = {};
		v2 m_tex0 = {};
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
		std::string_view m_name;              // Animation "Take" name
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
		virtual void CreateMesh(fbx::Mesh const& mesh, std::span<fbx::Material const> materials) { (void)mesh, materials; }

		// Create a model from a hierarchy of mesh instances. Return true to continue
		virtual void CreateModel(std::span<fbx::MeshTree const> mesh_tree) { (void)mesh_tree; }

		// Create a skeleton from a hierarchy of bone instances. Return true to continue
		virtual void CreateSkeleton(fbx::Skeleton const& skel) { (void)skel; }

		// Create an animation. Return true to continue
		virtual bool CreateAnimation(fbx::Animation const& anim) { (void)anim; return false; }
	};

	// Dynamically loaded FBX dll
	class Fbx
	{
		// Fbx is private only, only friends can access the API functions.
		friend struct Scene;
		friend struct Mesh;
		friend struct Material;
		friend struct Skin;

		// Dll module handle
		HMODULE m_module;

		#define PR_FBX_API(x)\
		x(       , Initialise, Context* (__stdcall*)(ErrorHandler error_cb))\
		x(       , Release, void (__stdcall*)(Context* ctx))\
		x(Scene_ , Load, SceneData* (__stdcall *)(Context& ctx, std::istream& src, LoadOptions const& opts))\
		x(Scene_ , Read, void (__stdcall *)(Context& ctx, SceneData& scene, ReadOptions const& options, IReadOutput& out))\
		x(Scene_ , Dump, void (__stdcall *)(Context& ctx, SceneData const& scene, DumpOptions const& options, std::ostream& out))
		#define PR_FBX_FUNCTION_MEMBERS(prefix, name, function_type) using prefix##name##Fn = function_type; prefix##name##Fn prefix##name = {};
		PR_FBX_API(PR_FBX_FUNCTION_MEMBERS)
		#undef PR_FBX_FUNCTION_MEMBERS

		Fbx()
			: m_module(win32::LoadDll<struct FbxDll>("fbx.dll"))
		{
			#pragma warning(push)
			#pragma warning(disable: 4191) // 'reinterpret_cast': unsafe conversion from 'FARPROC' to function pointer
			#define PR_FBX_GET_PROC_ADDRESS(prefix, name, function_type) prefix##name = reinterpret_cast<prefix##name##Fn>(GetProcAddress(m_module, "Fbx_" #prefix #name));
			PR_FBX_API(PR_FBX_GET_PROC_ADDRESS)
			#undef PR_FBX_GET_PROC_ADDRESS
			#pragma warning(pop)
		}

		// Singleton Instance
		static Fbx& get() { static Fbx s_this; return s_this; }
		static void StaticChecks();
	};

	// A loaded FBX scene
	struct Scene
	{
		// The dll context
		Context* m_ctx;

		// The loaded scene
		SceneData* m_scene;

		// Remember to open streams in binary mode!
		Scene(std::istream& src, LoadOptions const& opts = {}, ErrorHandler error_cb = {})
			: m_ctx(Fbx::get().Initialise(error_cb))
			, m_scene(Fbx::get().Scene_Load(*m_ctx, src, opts))
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
				Fbx::get().Release(m_ctx);
		}

		// Emit meshes/skeletons/etc
		void Read(IReadOutput& out, ReadOptions const& options)
		{
			Fbx::get().Scene_Read(*m_ctx, *m_scene, options, out);
		}

		// Read the full scene into memory
		void Dump(std::ostream& out, DumpOptions const& options)
		{
			Fbx::get().Scene_Dump(*m_ctx, *m_scene, options, out);
		}
	};
}
