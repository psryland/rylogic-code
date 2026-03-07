//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
// This is the dll API header.
#pragma once

#ifdef VIEW3D_EXPORTS
#define VIEW3D_API __declspec(dllexport)
#else
#define VIEW3D_API __declspec(dllimport)
#endif

// ** No dependencies except standard includes **
#include <span>
#include <memory>
#include <cstdint>
#include <functional>
#include <type_traits>
#include <windows.h>
#include <d3d12.h>

namespace pr
{
	// Forward declare types
	namespace rdr12
	{
		struct Context;
		struct V3dWindow;
		struct Texture2D;
		struct TextureCube;
		struct Sampler;
		struct Shader;

		namespace ldraw
		{
			struct LdrObject;
			struct LdrGizmo;
		}
	}

	namespace view3d
	{
		using DllHandle = unsigned char const*;
		using Object = rdr12::ldraw::LdrObject*;
		using Gizmo = rdr12::ldraw::LdrGizmo*;
		using Texture = rdr12::Texture2D*;
		using CubeMap = rdr12::TextureCube*;
		using Sampler = rdr12::Sampler*;
		using Shader = rdr12::Shader*;
		using Window = rdr12::V3dWindow*;
		using Colour = unsigned int;

		#pragma region Enumerations
		enum class EResult :int
		{
			Success,
			Failed,
		};
		enum class EFillMode :int
		{
			Default   = 0,
			Points    = 1,
			Wireframe = D3D12_FILL_MODE::D3D12_FILL_MODE_WIREFRAME,
			Solid     = D3D12_FILL_MODE::D3D12_FILL_MODE_SOLID,
			SolidWire = 4,
		};
		enum class ECullMode :int
		{
			Default = 0,
			None    = D3D12_CULL_MODE::D3D12_CULL_MODE_NONE,
			Front   = D3D12_CULL_MODE::D3D12_CULL_MODE_FRONT,
			Back    = D3D12_CULL_MODE::D3D12_CULL_MODE_BACK,
		};
		enum class EGeom :int // rdr12::EGeom
		{
			Unknown = 0,
			Vert    = 1 << 0, // Object space 3D position
			Colr    = 1 << 1, // Diffuse base colour
			Norm    = 1 << 2, // Object space 3D normal
			Tex0    = 1 << 3, // Diffuse texture
			_flags_enum = 0,
		};
		enum class ETopo :int // rdr12::ETopo
		{
			Invalid      = D3D_PRIMITIVE_TOPOLOGY::D3D_PRIMITIVE_TOPOLOGY_UNDEFINED,
			PointList    = D3D_PRIMITIVE_TOPOLOGY::D3D_PRIMITIVE_TOPOLOGY_POINTLIST,
			LineList     = D3D_PRIMITIVE_TOPOLOGY::D3D_PRIMITIVE_TOPOLOGY_LINELIST,
			LineStrip    = D3D_PRIMITIVE_TOPOLOGY::D3D_PRIMITIVE_TOPOLOGY_LINESTRIP,
			TriList      = D3D_PRIMITIVE_TOPOLOGY::D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST,
			TriStrip     = D3D_PRIMITIVE_TOPOLOGY::D3D_PRIMITIVE_TOPOLOGY_TRIANGLESTRIP,
			LineListAdj  = D3D_PRIMITIVE_TOPOLOGY::D3D_PRIMITIVE_TOPOLOGY_LINELIST_ADJ,
			LineStripAdj = D3D_PRIMITIVE_TOPOLOGY::D3D_PRIMITIVE_TOPOLOGY_LINESTRIP_ADJ,
			TriListAdj   = D3D_PRIMITIVE_TOPOLOGY::D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST_ADJ,
			TriStripAdj  = D3D_PRIMITIVE_TOPOLOGY::D3D_PRIMITIVE_TOPOLOGY_TRIANGLESTRIP_ADJ,
		};
		enum class ERenderStep :int
		{
			Invalid = 0,
			ForwardRender,
			GBuffer,
			DSLighting,
			ShadowMap,
			RayCast,
		};
		enum class ENuggetFlag :int // rdr12::ENuggetFlag
		{
			None = 0,

			// Exclude this nugget when rendering a model
			Hidden = 1 << 0,

			// Set if the geometry data for the nugget contains alpha colours
			GeometryHasAlpha = 1 << 1,

			// Set if the tint colour contains alpha
			TintHasAlpha = 1 << 2,

			// Set if the diffuse texture contains alpha (and we want alpha blending, not just thresholding)
			TexDiffuseHasAlpha = 1 << 3,

			// Excluded from shadow map render steps
			ShadowCastExclude = 1 << 4,

			// Can overlap with other nuggets.
			// Set this flag to true if you want to add a nugget that overlaps the range
			// of an existing nugget. For simple models, overlapping nugget ranges is
			// usually an error, but in advanced cases it isn't.
			RangesCanOverlap = 1 << 5,

			_flags_enum = 0,
		};
		enum class EStockTexture :int // rdr12::EStockTexture
		{
			Invalid = 0,
			Black,
			White,
			Gray,
			Checker,
			Checker2,
			Checker3,
			WhiteDot,
			WhiteSpot,
			WhiteTriangle,
			EnvMapProjection,
		};
		enum class EStockSampler : int // rdr12::EStockSampler
		{
			Invalid = 0,
			PointClamp,
			PointWrap,
			LinearClamp,
			LinearWrap,
			AnisotropicClamp,
			AnisotropicWrap,
		};
		enum class EStockShader : int // rdr12::EStockShader
		{
			Invalid = 0,
			Forward,
			FwdShaderVS,
			FwdShaderPS,

			// Radial fade params:
			//  *Type {Spherical|Cylindrical}
			//  *Radius {min,max}
			//  *Centre {x,y,z} (optional, defaults to camera position)
			//  *Absolute (optional, default false) - True if 'radius' is absolute, false if 'radius' should be scaled by the focus distance
			FwdRadialFadePS,

			GBufferVS,
			GBufferPS,
			DSLightingVS,
			DSLightingPS,
			ShadowMapVS,
			ShadowMapPS,

			// Point sprite params: *PointSize {w,h} *Depth {true|false}
			PointSpritesGS,

			// Thick line params: *LineWidth {width}
			ThickLineListGS,

			// Thick line params: *LineWidth {width}
			ThickLineStripGS,

			// Arrow params: *Size {size}
			ArrowHeadGS,

			ShowNormalsGS,
		};
		enum class ELight :int
		{
			Ambient,
			Directional,
			Point,
			Spot
		};
		enum class EAnimCommand : int
		{
			Reset, // Reset the 'time' value
			Play,  // Run continuously using 'time' as the step size, or real time if 'time' == 0
			Stop,  // Stop at the current time.
			Step,  // Step by 'time' (can be positive or negative)
		};
		enum class EGizmoMode :int
		{
			Translate,
			Rotate,
			Scale,
		};
		enum class EGizmoState :int // ELdrGizmoEvent 
		{
			StartManip,
			Moving,
			Commit,
			Revert,
		};
		enum class EStockObject
		{
			None = 0,
			FocusPoint = 1 << 0,
			OriginPoint = 1 << 1,
			SelectionBox = 1 << 2,
			_flags_enum = 0,
		};
		enum class ENavOp :int // camera::ENavOp
		{
			None      = 0,
			Translate = 1 << 0,
			Rotate    = 1 << 1,
			Zoom      = 1 << 2,
			_flags_enum = 0,
		};
		enum class ECameraLockMask :int // camera::ELockMask
		{
			None           = 0,
			TransX         = 1 << 0,
			TransY         = 1 << 1,
			TransZ         = 1 << 2,
			RotX           = 1 << 3,
			RotY           = 1 << 4,
			RotZ           = 1 << 5,
			Zoom           = 1 << 6,
			CameraRelative = 1 << 7,
			Translation    = TransX | TransY | TransZ,
			Rotation       = RotX | RotY | RotZ,
			All            = (1 << 7) - 1, // Not including camera relative
		};
		enum class EClipPlanes
		{
			None = 0,
			Near = 1 << 0,
			Far = 1 << 1,
			CameraRelative = 1 << 2,
			Both = Near | Far,
			_flags_enum = 0,
		};
		enum class EColourOp :int // rdr12::EColourOp
		{
			Overwrite,
			Add,
			Subtract,
			Multiply,
			Lerp,
		};
		enum class ELdrFlags :int // sync with 'ELdrFlags'
		{
			// Notes:
			//  - Flags are for a single object only. Don't set the recursively.
			//    Instead use the
			None = 0,

			// The object is hidden
			Hidden = 1 << 0,

			// The object is filled in wireframe mode
			Wireframe = 1 << 1,

			// Render the object without testing against the depth buffer
			NoZTest = 1 << 2,

			// Render the object without effecting the depth buffer
			NoZWrite = 1 << 3,

			// The object has normals shown
			Normals = 1 << 4,

			// The object to world transform is not an affine transform
			NonAffine = 1 << 5,

			// Set when an instance is "selected". The meaning of 'selected' is up to the application
			Selected = 1 << 8,

			// Doesn't contribute to the bounding box
			BBoxExclude = 1 << 9,

			// Should not be included when determining the bounds of a scene.
			SceneBoundsExclude = 1 << 10,

			// Ignored for hit test ray casts
			HitTestExclude = 1 << 11,

			// Doesn't cast a shadow
			ShadowCastExclude = 1 << 12,

			// True if the object has animation data.
			Animated = 1 << 13,

			// Indicates invalidated flags that need to be refreshed
			Invalidated = 1 << 31,

			// Bitwise operators
			_flags_enum = 0,
		};
		enum class EBBoxFlags :int // sync with 'EBBoxFlags'
		{
			None = 0,

			// Include child objects when calculating the bounding box
			IncludeChildren = 1 << 0,

			// Bitwise operators
			_flags_enum = 0,
		};
		enum class EUpdateObject :int // Flags for partial update of a model
		{
			None         = 0,
			Name         = 1 << 0,
			Model        = 1 << 1,
			Transform    = 1 << 2,
			Children     = 1 << 3,
			Colour       = 1 << 4,
			GroupColour  = 1 << 5,
			Reflectivity = 1 << 6,
			Flags        = 1 << 7,
			Animation    = 1 << 8,
			All          = 0x1FF,
			_flags_enum = 0,
		};
		enum class ESortGroup :int // rdr12::ESortGroup
		{
			Min = 0,               // The minimum sort group value
			PreOpaques = 63,       // 
			Default = 64,          // Make opaques the middle group
			Skybox,                // Sky-box after opaques
			PostOpaques,           // 
			PreAlpha = Default+16, // Last group before the alpha groups
			AlphaBack,             // 
			AlphaFront,            // 
			PostAlpha,             // First group after the alpha groups
			Max = 127,             // The maximum sort group value
	
			// Arithmetic operators supported
			_arith_enum = 0,
		};
		enum class ESceneBounds :int
		{
			All,
			Selected,
			Visible,
		};
		enum class ESourcesChangedReason :int
		{
			NewData,
			Reload,
			Removal,
		};
		enum class ESceneChanged :int
		{
			ObjectsAdded,
			ObjectsRemoved,
			GizmoAdded,
			GizmoRemoved,
		};
		enum class ESnapMode :int
		{
			NoSnap = 0,
			Verts = 1 << 0,
			Edges = 1 << 1,
			Faces = 1 << 2,
			Perspective = 1 << 8,
			All = Faces | Edges | Verts,
			AllPerspective = All | Perspective,
			_flags_enum = 0,
		};
		enum class ESnapType :int
		{
			NoSnap,
			Vert,
			EdgeMiddle,
			FaceCentre,
			Edge,
			Face,
		};
		enum class HitTestRayId : int
		{
			None = 0,
		};
		enum class EStreamingState :int
		{
			Disconnected,
			Listening,
			Connected,
		};
		enum class ESettings :int
		{
			// Upper 2-bytes = category
			// Lower 2-bytes = specific property that changed.
			None = 0,

			General                     = 1 << 16,
			General_FocusPointVisible   = General | 1 << 0,
			General_OriginPointVisible  = General | 1 << 1,
			General_SelectionBoxVisible = General | 1 << 2,
			General_FocusPointSize      = General | 1 << 3,
			General_OriginPointSize     = General | 1 << 4,
			General_SelectionBox        = General | 1 << 5,

			Scene                  = 1 << 17,
			Scene_BackgroundColour = Scene | 1 << 0,
			Scene_Multisampling    = Scene | 1 << 1,
			Scene_FillMode         = Scene | 1 << 2,
			Scene_CullMode         = Scene | 1 << 3,
			Scene_Viewport         = Scene | 1 << 4,
			Scene_EnvMap           = Scene | 1 << 5,

			Camera              = 1 << 18,
			Camera_Position     = Camera | 1 << 0,
			Camera_FocusDist    = Camera | 1 << 1,
			Camera_Orthographic = Camera | 1 << 2,
			Camera_Aspect       = Camera | 1 << 3,
			Camera_Fov          = Camera | 1 << 4,
			Camera_ClipPlanes   = Camera | 1 << 5,
			Camera_LockMask     = Camera | 1 << 6,
			Camera_AlignAxis    = Camera | 1 << 7,

			Lighting           = 1 << 19,
			Lighting_Type      = Lighting | 1 << 0,
			Lighting_Position  = Lighting | 1 << 1,
			Lighting_Direction = Lighting | 1 << 2,
			Lighting_Colour    = Lighting | 1 << 3,
			Lighting_Range     = Lighting | 1 << 4,
			Lighting_Shadows   = Lighting | 1 << 5,
			Lighting_All       = Lighting | Lighting_Type | Lighting_Position | Lighting_Direction | Lighting_Colour | Lighting_Range | Lighting_Shadows,

			Diagnostics                    = 1 << 20,
			Diagnostics_BBoxesVisible      = Diagnostics | 1 << 0,
			Diagnostics_NormalsLength      = Diagnostics | 1 << 1,
			Diagnostics_NormalsColour      = Diagnostics | 1 << 2,
			Diagnostics_FillModePointsSize = Diagnostics | 1 << 3,

			_flags_enum = 0,
		};
		#pragma endregion

		#pragma region Callbacks
		template <typename FuncType>
		struct Callback
		{
			// Usage: { &thing, [](void* ctx, args...){ return type_ptr<Thing>(ctx)->Do(args); } }
			using FuncCB = FuncType;
			using CtxPtr = union { void const* cp; void* p; };

			CtxPtr m_ctx = {};
			FuncCB m_cb = {};

			template <typename... Args>
			auto operator()(Args&&... args) const
			{
				return m_cb(m_ctx.p, std::forward<Args>(args)...);
			}
			explicit operator bool() const
			{
				return m_cb != nullptr;
			}
			friend bool operator == (Callback lhs, Callback rhs)
			{
				return lhs.m_cb == rhs.m_cb && lhs.m_ctx.cp == rhs.m_ctx.cp;
			}
		};
		using ReportErrorCB = Callback<void(__stdcall*)(void* ctx, char const* msg, char const* filepath, int line, int64_t pos)>;
		#pragma endregion

		#pragma region Structures
		struct Vec2
		{
			float x, y;
		};
		struct Vec4
		{
			float x, y, z, w;
		};
		struct Mat4x4
		{
			Vec4 x, y, z, w;
		};
		struct BBox
		{
			Vec4 centre;
			Vec4 radius;
		};
		struct VICount
		{
			int m_vcount;
			int m_icount;
		};
		struct Vertex
		{
			Vec4 pos;
			Vec4 norm;
			Vec2 tex;
			Colour col;
			uint32_t pad;
		};
		struct Nugget
		{
			struct Shader
			{
				view3d::Shader m_shader; // The shader instance to use
				ERenderStep m_rdr_step;  // The render step that the shader applies to
				int pad;
			};
			std::span<Shader const> shader_span() const
			{
				size_t count = 0;
				for (; m_shaders[count].m_rdr_step != ERenderStep::Invalid; ++count) {}
				return { m_shaders, count };
			}

			ETopo       m_topo;         // Model topology
			EGeom       m_geom;         // Model geometry type
			int         m_v0, m_v1;     // Vertex buffer range. Set to 0,0 to mean the whole buffer. Use 1,1 if you want to say 0-span
			int         m_i0, m_i1;     // Index buffer range. Set to 0,0 to mean the whole buffer. Use 1,1 if you want to say 0-span
			Texture     m_tex_diffuse;  // Diffuse texture
			Sampler     m_sam_diffuse;  // Sampler for the diffuse texture 
			Shader      m_shaders[8];   // Array of shader overrides, length uses sentinel 'm_rdr_step == Invalid (0)'
			ENuggetFlag m_nflags;       // Nugget flags
			ECullMode   m_cull_mode;    // Face culling mode
			EFillMode   m_fill_mode;    // Model fill mode
			Colour      m_tint;         // Tint colour
			float       m_rel_reflec;   // How reflective this nugget is relative to the over all model;
		};
		struct MultiSamp
		{
			int m_count;
			int m_quality;
		};
		struct Light
		{
			Vec4 m_position;
			Vec4 m_direction;
			ELight m_type;
			Colour m_ambient;
			Colour m_diffuse;
			Colour m_specular;
			float m_specular_power;
			float m_range;
			float m_falloff;
			float m_inner_angle;
			float m_outer_angle;
			float m_cast_shadow;
			BOOL m_cam_relative;
			BOOL m_on;
		};
		struct TextureOptions
		{
			Mat4x4                m_t2s;
			DXGI_FORMAT           m_format;
			int                   m_mips;
			D3D12_RESOURCE_FLAGS  m_usage;
			D3D12_RESOURCE_STATES m_resource_state;
			D3D12_CLEAR_VALUE     m_clear_value;
			MultiSamp             m_multisamp;
			UINT                  m_colour_key;
			BOOL                  m_has_alpha;
			char const*           m_dbg_name;
		};
		struct CubeMapOptions
		{
			Mat4x4      m_cube2w;
			char const* m_dbg_name;
		};
		struct SamplerOptions
		{
			D3D12_FILTER               m_filter;
			D3D12_TEXTURE_ADDRESS_MODE m_addrU;
			D3D12_TEXTURE_ADDRESS_MODE m_addrV;
			D3D12_TEXTURE_ADDRESS_MODE m_addrW;
			char const*                m_dbg_name;
		};
		struct ShaderOptions
		{
			// todo
		};
		struct WindowOptions
		{
			ReportErrorCB m_error_cb;
			Colour        m_background_colour;
			BOOL          m_allow_alt_enter;
			BOOL          m_gdi_compatible_backbuffer;
			BOOL          m_xr_support;
			int           m_multisampling;
			char const*   m_dbg_name;

			WindowOptions()
				: m_error_cb()
				, m_background_colour(0xFF808080)
				, m_allow_alt_enter(true)
				, m_gdi_compatible_backbuffer(false)
				, m_xr_support(false)
				, m_multisampling(1)
				, m_dbg_name()
			{
			}
			WindowOptions& error_cb(ReportErrorCB error_cb)
			{
				m_error_cb = error_cb;
				return *this;
			}
			WindowOptions& back_colour(Colour colour)
			{
				m_background_colour = colour;
				return *this;
			}
			WindowOptions& alt_enter(bool enable = true)
			{
				m_allow_alt_enter = enable;
				return *this;
			}
			WindowOptions& gdi_compat(bool enable = true)
			{
				m_gdi_compatible_backbuffer = enable;
				return *this;
			}
			WindowOptions& xr_support(bool enable = true)
			{
				m_xr_support = enable;
				return *this;
			}
			WindowOptions& multisamp(int samples)
			{
				m_multisampling = samples;
				return *this;
			}
			WindowOptions& name(char const* dbg_name)
			{
				m_dbg_name = dbg_name;
				return *this;
			}
		};
		struct HitTestRay
		{
			// The world space origin and direction of the ray (normalisation not required)
			Vec4 m_ws_origin = {};
			Vec4 m_ws_direction = {};

			// Snap mode and distance. If snap_mode includes ESnapMode::Perspective, then the snap distance scales with distance from the origin
			ESnapMode m_snap_mode = ESnapMode::AllPerspective; // Snap behaviour
			float m_snap_distance = 0; // Snap distance: 'snap_dist = Perspectvie ? snap_distance * depth : snap_distance'

			// User provided id for the ray
			HitTestRayId m_id = HitTestRayId::None;
			int pad = 0;
		};
		struct HitTestResult
		{
			// The origin and direction of the cast ray (in world space)
			Vec4 m_ws_ray_origin;
			Vec4 m_ws_ray_direction;

			// The intercept point and surface normal (in world space)
			Vec4 m_ws_intercept;
			Vec4 m_ws_normal;

			// The object that was hit (or null)
			Object m_obj;

			// The distance from ray origin to hit point
			float m_distance;

			// The index/id of the input ray
			int m_ray_index;
			int m_ray_id;

			// How the hit point was snapped (if at all)
			ESnapType m_snap_type;

			int pad0;
			int pad1;

			// != 0 if this is a hit
			bool IsHit() const { return m_obj != nullptr; }
		};
		struct Viewport
		{
			float m_x;         // (x,y,x+width,y+width) is in back buffer pixels, *NOT* window DIP.
			float m_y;         // Typically the back buffer is the same size as the true screen pixels
			float m_width;     // Typically the BB width
			float m_height;    // Typically the BB height
			float m_min_depth; // Typically 0.0f
			float m_max_depth; // Typically 1.0f
			int m_screen_w;    // The screen width in DIP
			int m_screen_h;    // The screen height in DIP
		};
		struct Includes
		{
			char const* m_include_paths; // A comma or semicolon separated list of search directories
			HMODULE     m_modules[16];   // An array of binary modules that contain resources. '0' means 'this' module
			int         m_module_count;  // The number of valid module values in 'm_modules'
			// (ToDo) A string lookup table
		};
		struct SceneChanged
		{
			// How the scene was changed
			ESceneChanged m_change_type;

			// An array of the context ids that changed
			GUID const* m_ctx_ids;

			// The length of the 'm_ctx_ids' array
			int m_count;

			// Pointer to the object that changed (for single object changes only)
			Object m_object;
		};
		struct AnimEvent
		{
			// The state change type
			EAnimCommand m_command;

			// The current animation clock value
			double m_clock;
		};
		struct ImageInfo
		{
			uint64_t    m_width;
			uint32_t    m_height;
			uint16_t    m_depth;
			uint16_t    m_mips;
			DXGI_FORMAT m_format;
			uint32_t    m_image_file_format; // D3DXIMAGE_FILEFORMAT
		};
		struct BackBuffer
		{
			ID3D12Resource* m_render_target;
			ID3D12Resource* m_depth_stencil;
			SIZE m_dim;
		};
		struct SourceInfo
		{
			// The name of the source
			char const* m_name;

			// The file associated with the source (can be null)
			wchar_t const* m_filepath;

			// The context id associated with the source
			GUID m_context_id;

			// The number of objects in this source
			int m_object_count;

			// True if the source is a text format, false if binary
			int m_text_format;	
		};
		#pragma endregion

		// Callbacks
		using GuidPredCB = Callback<bool(__stdcall*)(void* ctx, GUID const&)>;
		using SettingsChangedCB = Callback<void(__stdcall *)(void* ctx, Window window, ESettings setting)>;
		using ParsingProgressCB = Callback<void(__stdcall *)(void* ctx, GUID const& context_id, char const* filepath, long long file_offset, long long file_size, BOOL complete, BOOL& cancel)>;
		using SourcesChangedCB = Callback<void(__stdcall *)(void* ctx, ESourcesChangedReason reason, GUID const* ids, int count, BOOL before)>;
		using EnumGuidsCB = Callback<bool(__stdcall *)(void* ctx, GUID const& context_id)>;
		using EnumObjectsCB = Callback<bool(__stdcall *)(void* ctx, Object object)>;
		using AddCompleteCB = Callback<void(__stdcall *)(void* ctx, GUID const& context_id, BOOL before)>;
		using InvalidatedCB = Callback<void(__stdcall *)(void* ctx, Window window)>;
		using RenderingCB = Callback<void(__stdcall *)(void* ctx, Window window)>;
		using SceneChangedCB = Callback<void(__stdcall *)(void* ctx, Window window, SceneChanged const&)>;
		using AnimationCB = Callback<void(__stdcall *)(void* ctx, Window window, EAnimCommand command, double clock)>;
		using HitTestAsyncCB = Callback<void(__stdcall*)(void* ctx, Window window, HitTestResult const* results, int count)>;
		using GizmoMovedCB = Callback<void(__stdcall *)(void* ctx, Gizmo gizmo, EGizmoState state)>;
		using AddNuggetCB = Callback<void(__stdcall*)(void* ctx, Nugget const& nugget)>;
		using EditObjectCB = Callback<VICount(__stdcall*)(void* ctx,
			int vcount,               // The maximum size of 'verts'
			int icount,               // The maximum size of 'indices'
			Vertex* verts,            // The vert buffer to be filled
			uint16_t* indices,        // The index buffer to be filled
			AddNuggetCB out_nugget)>; // Callback for adding a nugget
	}
}

// C linkage for API functions
extern "C"
{
	// Dll Context ****************************
	
	// Initialise calls are reference counted and must be matched with Shutdown calls
	// 'initialise_error_cb' is used to report dll initialisation errors only (i.e. it isn't stored)
	// Note: this function is not thread safe, avoid race calls
	VIEW3D_API pr::view3d::DllHandle __stdcall View3D_Initialise(pr::view3d::ReportErrorCB global_error_cb);
	VIEW3D_API void __stdcall View3D_Shutdown(pr::view3d::DllHandle context);

	// This error callback is called for errors that are associated with the dll (rather than with a window).
	VIEW3D_API void __stdcall View3D_GlobalErrorCBSet(pr::view3d::ReportErrorCB error_cb, BOOL add);

	// Set the callback for progress events when script sources are loaded or updated
	VIEW3D_API void __stdcall View3D_ParsingProgressCBSet(pr::view3d::ParsingProgressCB progress_cb, BOOL add);
	
	// Set the callback that is called when the sources are reloaded
	VIEW3D_API void __stdcall View3D_SourcesChangedCBSet(pr::view3d::SourcesChangedCB sources_changed_cb, BOOL add);

	// Return the context id for objects created from 'filepath' (if filepath is an existing source)
	VIEW3D_API GUID __stdcall View3D_ContextIdFromFilepath(char const* filepath);

	// Data Sources ***************************

	// Add an ldr script source. This will create all objects with context id 'context_id' (if given, otherwise an id will be created). Concurrent calls are thread safe.
	VIEW3D_API GUID __stdcall View3D_LoadScriptFromString(char const* ldr_script, GUID const* context_id, pr::view3d::Includes const* includes, pr::view3d::AddCompleteCB on_add_cb);
	VIEW3D_API GUID __stdcall View3D_LoadScriptFromFile(char const* ldr_file, GUID const* context_id, pr::view3d::Includes const* includes, pr::view3d::AddCompleteCB on_add_cb);

	// Cancel an in-progress load operation
	VIEW3D_API void __stdcall View3D_CancelLoad(GUID const* context_id);

	// Enumerate all sources in the store
	VIEW3D_API void __stdcall View3D_EnumSources(pr::view3d::EnumGuidsCB enum_guid_cb);

	// Reload objects from the source associated with 'context_id'
	VIEW3D_API void __stdcall View3D_SourceReload(GUID const& context_id);

	// Delete all objects and remove the source associated with 'context_id'
	VIEW3D_API void __stdcall View3D_SourceDelete(GUID const& context_id);

	// Get information about a source
	VIEW3D_API pr::view3d::SourceInfo __stdcall View3D_SourceInfo(GUID const& context_id);

	// Get/Set the name of a source
	VIEW3D_API BSTR __stdcall View3D_SourceNameGetBStr(GUID const& context_id);
	VIEW3D_API char const* __stdcall View3D_SourceNameGet(GUID const& context_id);
	VIEW3D_API void __stdcall View3D_SourceNameSet(GUID const& context_id, char const* name);

	// Reload script sources. This will delete all objects associated with the script sources then reload the files creating new objects with the same context ids.
	VIEW3D_API void __stdcall View3D_ReloadScriptSources();

	// Delete all objects and object sources
	VIEW3D_API void __stdcall View3D_DeleteAllObjects();

	// Delete all objects matching (or not matching) a context id
	VIEW3D_API void __stdcall View3D_DeleteById(pr::view3d::GuidPredCB pred);

	// Delete all objects not displayed in any windows
	VIEW3D_API void __stdcall View3D_DeleteUnused(pr::view3d::GuidPredCB pred);

	// Poll for changed script sources and reload any that have changed
	VIEW3D_API void __stdcall View3D_CheckForChangedSources();

	// Enable/Disable streaming script sources.
	VIEW3D_API pr::view3d::EStreamingState __stdcall View3D_StreamingState();
	VIEW3D_API void __stdcall View3D_StreamingEnable(BOOL enable, int port);

	// Windows ********************************

	// Create/Destroy a window
	VIEW3D_API pr::view3d::Window __stdcall View3D_WindowCreate(HWND hwnd, pr::view3d::WindowOptions const& opts);
	VIEW3D_API void __stdcall View3D_WindowDestroy(pr::view3d::Window window);
	
	// Add/Remove a window error callback. Note: The callback function can be called in a worker thread context.
	VIEW3D_API void __stdcall View3D_WindowErrorCBSet(pr::view3d::Window window, pr::view3d::ReportErrorCB error_cb, BOOL add);

	// Get/Set the window settings (as ldr script string)
	VIEW3D_API BSTR __stdcall View3D_WindowSettingsGetBStr(pr::view3d::Window window);
	VIEW3D_API char const* __stdcall View3D_WindowSettingsGet(pr::view3d::Window window);
	VIEW3D_API void __stdcall View3D_WindowSettingsSet(pr::view3d::Window window, char const* settings);

	// Get/Set the dimensions of the render target. Note: Not equal to window size for non-96 dpi screens!
	// In set, if 'width' and 'height' are zero, the RT is resized to the associated window automatically.
	VIEW3D_API SIZE __stdcall View3D_WindowBackBufferSizeGet(pr::view3d::Window window);
	VIEW3D_API void __stdcall View3D_WindowBackBufferSizeSet(pr::view3d::Window window, SIZE size, BOOL force_recreate);

	// Get/Set the window viewport (and clipping area)
	VIEW3D_API pr::view3d::Viewport __stdcall View3D_WindowViewportGet(pr::view3d::Window window);
	VIEW3D_API void __stdcall View3D_WindowViewportSet(pr::view3d::Window window, pr::view3d::Viewport const& vp);

	// Set a notification handler for when a window setting changes
	VIEW3D_API void __stdcall View3D_WindowSettingsChangedCB(pr::view3d::Window window, pr::view3d::SettingsChangedCB settings_changed_cb, BOOL add);

	// Add/Remove a callback that is called when the collection of objects associated with 'window' changes
	VIEW3D_API void __stdcall View3D_WindowSceneChangedCB(pr::view3d::Window window, pr::view3d::SceneChangedCB scene_changed_cb, BOOL add);

	// Add/Remove a callback that is called just prior to rendering the window
	VIEW3D_API void __stdcall View3D_WindowRenderingCB(pr::view3d::Window window, pr::view3d::RenderingCB rendering_cb, BOOL add);

	// Add/Remove an object to/from a window
	VIEW3D_API void __stdcall View3D_WindowAddObject(pr::view3d::Window window, pr::view3d::Object object);
	VIEW3D_API void __stdcall View3D_WindowRemoveObject(pr::view3d::Window window, pr::view3d::Object object);

	// Add/Remove a gizmo to/from a window
	VIEW3D_API void __stdcall View3D_WindowAddGizmo(pr::view3d::Window window, pr::view3d::Gizmo giz);
	VIEW3D_API void __stdcall View3D_WindowRemoveGizmo(pr::view3d::Window window, pr::view3d::Gizmo giz);

	// Add/Remove objects by context id. This function can be used to add all objects either in, or not in 'context_ids'
	VIEW3D_API void __stdcall View3D_WindowAddObjectsById(pr::view3d::Window window, pr::view3d::GuidPredCB pred);
	VIEW3D_API void __stdcall View3D_WindowRemoveObjectsById(pr::view3d::Window window, pr::view3d::GuidPredCB pred);

	// Remove all objects 'window'
	VIEW3D_API void __stdcall View3D_WindowRemoveAllObjects(pr::view3d::Window window);

	// Enumerate the GUIDs associated with 'window'
	VIEW3D_API void __stdcall View3D_WindowEnumGuids(pr::view3d::Window window, pr::view3d::EnumGuidsCB enum_guids_cb);

	// Enumerate the objects associated with 'window'
	VIEW3D_API void __stdcall View3D_WindowEnumObjects(pr::view3d::Window window, pr::view3d::EnumObjectsCB enum_objects_cb);
	VIEW3D_API void __stdcall View3D_WindowEnumObjectsById(pr::view3d::Window window, pr::view3d::EnumObjectsCB enum_objects_cb, pr::view3d::GuidPredCB pred);

	// Return true if 'object' is among 'window's objects
	VIEW3D_API BOOL __stdcall View3D_WindowHasObject(pr::view3d::Window window, pr::view3d::Object object, BOOL search_children);

	// Return the number of objects assigned to 'window'
	VIEW3D_API int __stdcall View3D_WindowObjectCount(pr::view3d::Window window);

	// Return the bounds of a scene
	VIEW3D_API pr::view3d::BBox __stdcall View3D_WindowSceneBounds(pr::view3d::Window window, pr::view3d::ESceneBounds bounds, int except_count, GUID const* except);

	// Render the window
	VIEW3D_API void __stdcall View3D_WindowRender(pr::view3d::Window window);

	// Wait for any previous frames to complete rendering within the GPU
	VIEW3D_API void __stdcall View3D_WindowGSyncWait(pr::view3d::Window window);

	// Replace the swap chain buffers
	VIEW3D_API void __stdcall View3D_WindowCustomSwapChain(pr::view3d::Window window, int count, pr::view3d::Texture* targets);

	// Get the MSAA back buffer (render target + depth stencil)
	VIEW3D_API pr::view3d::BackBuffer __stdcall View3D_WindowRenderTargetGet(pr::view3d::Window window);

	// Signal the window is invalidated. This does not automatically trigger rendering. Use InvalidatedCB.
	VIEW3D_API void __stdcall View3D_WindowInvalidate(pr::view3d::Window window, BOOL erase);
	VIEW3D_API void __stdcall View3D_WindowInvalidateRect(pr::view3d::Window window, RECT const& rect, BOOL erase);

	// Register a callback for when the window is invalidated. This can be used to render in response to invalidation, rather than rendering on a polling cycle.
	VIEW3D_API void __stdcall View3D_WindowInvalidatedCB(pr::view3d::Window window, pr::view3d::InvalidatedCB invalidated_cb, BOOL add);

	// Clear the 'invalidated' state of the window.
	VIEW3D_API void __stdcall View3D_WindowValidate(pr::view3d::Window window);

	// Get/Set the window background colour
	VIEW3D_API unsigned int __stdcall View3D_WindowBackgroundColourGet(pr::view3d::Window window);
	VIEW3D_API void __stdcall View3D_WindowBackgroundColourSet(pr::view3d::Window window, unsigned int argb);

	// Get/Set the fill mode for the window
	VIEW3D_API pr::view3d::EFillMode __stdcall View3D_WindowFillModeGet(pr::view3d::Window window);
	VIEW3D_API void __stdcall View3D_WindowFillModeSet(pr::view3d::Window window, pr::view3d::EFillMode mode);

	// Get/Set the cull mode for a faces in window
	VIEW3D_API pr::view3d::ECullMode __stdcall View3D_WindowCullModeGet(pr::view3d::Window window);
	VIEW3D_API void __stdcall View3D_WindowCullModeSet(pr::view3d::Window window, pr::view3d::ECullMode mode);

	// Get/Set the multi-sampling mode for a window
	VIEW3D_API int __stdcall View3D_MultiSamplingGet(pr::view3d::Window window);
	VIEW3D_API void __stdcall View3D_MultiSamplingSet(pr::view3d::Window window, int multisampling);

	// Control animation
	VIEW3D_API void __stdcall View3D_WindowAnimControl(pr::view3d::Window window, pr::view3d::EAnimCommand command, double time);

	// Set the callback for animation events
	VIEW3D_API void __stdcall View3D_WindowAnimEventCBSet(pr::view3d::Window window, pr::view3d::AnimationCB anim_cb, BOOL add);

	// Get/Set the animation time
	VIEW3D_API BOOL __stdcall View3D_WindowAnimating(pr::view3d::Window window);
	VIEW3D_API double __stdcall View3D_WindowAnimTimeGet(pr::view3d::Window window);
	VIEW3D_API void __stdcall View3D_WindowAnimTimeSet(pr::view3d::Window window, double time_s);

	// Return the DPI of the monitor that 'window' is displayed on
	VIEW3D_API pr::view3d::Vec2 __stdcall View3D_WindowDpiScale(pr::view3d::Window window);

	// Set the global environment map for the window
	VIEW3D_API void __stdcall View3D_WindowEnvMapSet(pr::view3d::Window window, pr::view3d::CubeMap env_map);

	// Enable/Disable the depth buffer
	VIEW3D_API BOOL __stdcall View3D_DepthBufferEnabledGet(pr::view3d::Window window);
	VIEW3D_API void __stdcall View3D_DepthBufferEnabledSet(pr::view3d::Window window, BOOL enabled);

	// Cast a ray into the scene, returning information about what it hit.
	VIEW3D_API void __stdcall View3D_WindowHitTestObjects(pr::view3d::Window window, pr::view3d::HitTestRay const* rays, pr::view3d::HitTestResult* hits, int ray_count, pr::view3d::Object const* objects, int object_count);
	VIEW3D_API void __stdcall View3D_WindowHitTestByCtx(pr::view3d::Window window, pr::view3d::HitTestRay const* rays, pr::view3d::HitTestResult* hits, int ray_count, pr::view3d::GuidPredCB pred);

	// Trigger execution of the async hit test rays. Submits GPU work and returns immediately.
	// Results are reported via the callback registered with View3D_HitTestAsyncCBSet.
	VIEW3D_API void __stdcall View3D_HitTestAsync(pr::view3d::Window window);

	// Add/Remove a callback for receiving async hit test results
	VIEW3D_API void __stdcall View3D_HitTestAsyncCBSet(pr::view3d::Window window, pr::view3d::HitTestAsyncCB cb, BOOL add);

	// Add/Update/Remove an async hit test ray.
	// Returns 'HitTestRayId::None' if no more rays can be added.
	// Returns 'id' if the ray was updated/removed successfully, otherwise returns HitTestRayId::None.
	// Use 'ray == nullptr' to remove a ray.
	VIEW3D_API pr::view3d::HitTestRayId __stdcall View3D_HitTestRayUpdate(pr::view3d::Window window, pr::view3d::HitTestRayId id, pr::view3d::HitTestRay const* ray);

	// Camera *********************************

	// Position the camera and focus distance
	VIEW3D_API void __stdcall View3D_CameraPositionSet(pr::view3d::Window window, pr::view3d::Vec4 position, pr::view3d::Vec4 lookat, pr::view3d::Vec4 up);

	// Get/Set the current camera to world transform
	VIEW3D_API pr::view3d::Mat4x4 __stdcall View3D_CameraToWorldGet(pr::view3d::Window window);
	VIEW3D_API void __stdcall View3D_CameraToWorldSet(pr::view3d::Window window, pr::view3d::Mat4x4 const& c2w);

	// Move the camera to a position that can see the whole scene. Set 'dist' to 0 to preserve the FoV, or a distance to set the FoV
	VIEW3D_API void __stdcall View3D_ResetView(pr::view3d::Window window, pr::view3d::Vec4 forward, pr::view3d::Vec4 up, float dist, BOOL preserve_aspect, BOOL commit);

	// Reset the camera to view a bbox. Set 'dist' to 0 to preserve the FoV, or a distance to set the FoV
	VIEW3D_API void __stdcall View3D_ResetViewBBox(pr::view3d::Window window, pr::view3d::BBox bbox, pr::view3d::Vec4 forward, pr::view3d::Vec4 up, float dist, BOOL preserve_aspect, BOOL commit);

	// Enable/Disable orthographic projection
	VIEW3D_API BOOL __stdcall View3D_CameraOrthographicGet(pr::view3d::Window window);
	VIEW3D_API void __stdcall View3D_CameraOrthographicSet(pr::view3d::Window window, BOOL on);

	// Get/Set the distance to the camera focus point
	VIEW3D_API float __stdcall View3D_CameraFocusDistanceGet(pr::view3d::Window window);
	VIEW3D_API void __stdcall View3D_CameraFocusDistanceSet(pr::view3d::Window window, float dist);

	// Get/Set the camera focus point position
	VIEW3D_API pr::view3d::Vec4 __stdcall View3D_CameraFocusPointGet(pr::view3d::Window window);
	VIEW3D_API void __stdcall View3D_CameraFocusPointSet(pr::view3d::Window window, pr::view3d::Vec4 position);

	// Get/Set bounds on the camera focus point position
	VIEW3D_API pr::view3d::BBox __stdcall View3D_CameraFocusBoundsGet(pr::view3d::Window window);
	VIEW3D_API void __stdcall View3D_CameraFocusBoundsSet(pr::view3d::Window window, pr::view3d::BBox bounds);

	// Get/Set the aspect ratio for the camera field of view
	VIEW3D_API float __stdcall View3D_CameraAspectGet(pr::view3d::Window window);
	VIEW3D_API void __stdcall View3D_CameraAspectSet(pr::view3d::Window window, float aspect);
	
	// Get/Set both the X and Y fields of view (i.e. set the aspect ratio)
	VIEW3D_API pr::view3d::Vec2 __stdcall View3D_CameraFovGet(pr::view3d::Window window);
	VIEW3D_API void __stdcall View3D_CameraFovSet(pr::view3d::Window window, pr::view3d::Vec2 fov);

	// Adjust the FocusDist, FovX, and FovY so that the average FOV equals 'fov'
	VIEW3D_API void __stdcall View3D_CameraBalanceFov(pr::view3d::Window window, float fov);

	// Get/Set (using fov and focus distance) the size of the perpendicular area visible to the camera at 'dist' (in world space). Use 'focus_dist != 0' to set a specific focus distance
	VIEW3D_API pr::view3d::Vec2 __stdcall View3D_CameraViewRectAtDistanceGet(pr::view3d::Window window, float dist);
	VIEW3D_API void __stdcall View3D_CameraViewRectAtDistanceSet(pr::view3d::Window window, pr::view3d::Vec2 rect, float focus_dist);

	// Get/Set the near and far clip planes for the camera
	VIEW3D_API pr::view3d::Vec2 __stdcall View3D_CameraClipPlanesGet(pr::view3d::Window window, pr::view3d::EClipPlanes flags);
	VIEW3D_API void __stdcall View3D_CameraClipPlanesSet(pr::view3d::Window window, float near_, float far_, pr::view3d::EClipPlanes flags);

	// Get/Set the scene camera lock mask
	VIEW3D_API pr::view3d::ECameraLockMask __stdcall View3D_CameraLockMaskGet(pr::view3d::Window window);
	VIEW3D_API void __stdcall View3D_CameraLockMaskSet(pr::view3d::Window window, pr::view3d::ECameraLockMask mask);
	
	// Get/Set the camera align axis
	VIEW3D_API pr::view3d::Vec4 __stdcall View3D_CameraAlignAxisGet(pr::view3d::Window window);
	VIEW3D_API void __stdcall View3D_CameraAlignAxisSet(pr::view3d::Window window, pr::view3d::Vec4 axis);
	
	// Reset to the default zoom
	VIEW3D_API void __stdcall View3D_CameraResetZoom(pr::view3d::Window window);

	// Get/Set the FOV zoom
	VIEW3D_API float __stdcall View3D_CameraZoomGet(pr::view3d::Window window);
	VIEW3D_API void __stdcall View3D_CameraZoomSet(pr::view3d::Window window, float zoom);

	// Commit the current O2W position as the reference position
	VIEW3D_API void __stdcall View3D_CameraCommit(pr::view3d::Window window);

	// Navigation *****************************

	// Direct movement of the camera
	VIEW3D_API BOOL __stdcall View3D_Navigate(pr::view3d::Window window, float dx, float dy, float dz);

	// Move the scene camera using the mouse
	VIEW3D_API BOOL __stdcall View3D_MouseNavigate(pr::view3d::Window window, pr::view3d::Vec2 ss_pos, pr::view3d::ENavOp nav_op, BOOL nav_start_or_end);
	VIEW3D_API BOOL __stdcall View3D_MouseNavigateZ(pr::view3d::Window window, pr::view3d::Vec2 ss_pos, float delta, BOOL along_ray);

	// Convert an MK_ macro to a default navigation operation
	VIEW3D_API pr::view3d::ENavOp __stdcall View3D_MouseBtnToNavOp(int mk);
	
	// Convert lengths in pixels (X and Y) into lengths in normalised screen space
	VIEW3D_API pr::view3d::Vec2 __stdcall View3D_PixelsToNSS(pr::view3d::Window window, pr::view3d::Vec2 pixels);

	// Convert a point between 'window' screen space and normalised screen space
	VIEW3D_API pr::view3d::Vec2 __stdcall View3D_SSPointToNSSPoint(pr::view3d::Window window, pr::view3d::Vec2 screen);
	VIEW3D_API pr::view3d::Vec2 __stdcall View3D_NSSPointToSSPoint(pr::view3d::Window window, pr::view3d::Vec2 nss_point);

	// Convert a point between world space and normalised screen space.
	// The x,y components of 'screen' should be in normalised screen space, i.e. (-1,-1)->(1,1)
	// The z component should be the world space distance from the camera
	VIEW3D_API pr::view3d::Vec4 __stdcall View3D_NSSPointToWSPoint(pr::view3d::Window window, pr::view3d::Vec4 screen);
	VIEW3D_API pr::view3d::Vec4 __stdcall View3D_WSPointToNSSPoint(pr::view3d::Window window, pr::view3d::Vec4 world);

	// Return a point and direction in world space corresponding to a normalised screen space point.
	// The x,y components of 'screen' should be in normalised screen space, i.e. (-1,-1)->(1,1)
	// The z component should be the world space distance from the camera
	VIEW3D_API void __stdcall View3D_NSSPointToWSRay(pr::view3d::Window window, pr::view3d::Vec4 screen, pr::view3d::Vec4& ws_point, pr::view3d::Vec4& ws_direction);

	// Return a point and direction in world space corresponding to a 'window' screen space point.
	// 'screen' should be in pixels
	VIEW3D_API void __stdcall View3D_SSPointToWSRay(pr::view3d::Window window, pr::view3d::Vec2 screen, pr::view3d::Vec4& ws_point, pr::view3d::Vec4& ws_direction);

	// Lights *********************************

	// Get/Set the properties of the global light
	VIEW3D_API pr::view3d::Light __stdcall View3D_LightPropertiesGet(pr::view3d::Window window);
	VIEW3D_API void __stdcall View3D_LightPropertiesSet(pr::view3d::Window window, pr::view3d::Light const& light);
	
	// Set the global light source for a window
	VIEW3D_API void __stdcall View3D_LightSource(pr::view3d::Window window, pr::view3d::Vec4 position, pr::view3d::Vec4 direction, BOOL camera_relative);

	// Objects ********************************

	// Notes:
	// 'name' parameter on object get/set functions:
	//   If 'name' is null, then the state of the root object is returned
	//   If 'name' begins with '#' then the remainder of the name is treated as a regular expression

	// Create an object from provided buffers
	VIEW3D_API pr::view3d::Object __stdcall View3D_ObjectCreate(char const* name, pr::view3d::Colour colour, int vcount, int icount, int ncount, pr::view3d::Vertex const* verts, UINT16 const* indices, pr::view3d::Nugget const* nuggets, GUID const& context_id);

	// Create an graphics object from ldr script, either a string or a file 
	VIEW3D_API pr::view3d::Object __stdcall View3D_ObjectCreateLdrW(wchar_t const* ldr_script, BOOL file, GUID const* context_id, pr::view3d::Includes const* includes);
	VIEW3D_API pr::view3d::Object __stdcall View3D_ObjectCreateLdrA(char const* ldr_script, BOOL file, GUID const* context_id, pr::view3d::Includes const* includes);

	// Load a p3d model file as a view3d object
	VIEW3D_API pr::view3d::Object __stdcall View3D_ObjectCreateP3DFile(char const* name, pr::view3d::Colour colour, char const* p3d_filepath, GUID const* context_id);

	// Load a p3d model in memory as a view3d object
	VIEW3D_API pr::view3d::Object __stdcall View3D_ObjectCreateP3DStream(char const* name, pr::view3d::Colour colour, size_t size, void const* p3d_data, GUID const* context_id);

	// Create an ldr object using a callback to populate the model data.
	VIEW3D_API pr::view3d::Object __stdcall View3D_ObjectCreateWithCallback(char const* name, pr::view3d::Colour colour, int vcount, int icount, int ncount, pr::view3d::EditObjectCB edit_cb, GUID const& context_id);
	VIEW3D_API void __stdcall View3D_ObjectEdit(pr::view3d::Object object, pr::view3d::EditObjectCB edit_cb);

	// Replace the model and all child objects of 'obj' with the results of 'ldr_script'
	VIEW3D_API void __stdcall View3D_ObjectUpdate(pr::view3d::Object object, wchar_t const* ldr_script, pr::view3d::EUpdateObject flags);

	// Delete an object, freeing its resources
	VIEW3D_API void __stdcall View3D_ObjectDelete(pr::view3d::Object object);

	// Create an instance of 'obj'
	VIEW3D_API pr::view3d::Object __stdcall View3D_ObjectCreateInstance(pr::view3d::Object object);

	// Return the context id that this object belongs to
	VIEW3D_API GUID __stdcall View3D_ObjectContextIdGet (pr::view3d::Object object);

	// Return the root object of 'object'(possibly itself)
	VIEW3D_API pr::view3d::Object __stdcall View3D_ObjectGetRoot(pr::view3d::Object object);

	// Return the immediate parent of 'object'
	VIEW3D_API pr::view3d::Object __stdcall View3D_ObjectGetParent(pr::view3d::Object object);

	// Return a child object of 'object'
	VIEW3D_API pr::view3d::Object __stdcall View3D_ObjectGetChildByName(pr::view3d::Object object, char const* name);
	VIEW3D_API pr::view3d::Object __stdcall View3D_ObjectGetChildByIndex(pr::view3d::Object object, int index);

	// Return the number of child objects of 'object'
	VIEW3D_API int __stdcall View3D_ObjectChildCount (pr::view3d::Object object);

	// Enumerate the child objects of 'object'. (Not recursive)
	VIEW3D_API void __stdcall View3D_ObjectEnumChildren (pr::view3d::Object object, pr::view3d::EnumObjectsCB enum_objects_cb);

	// Get/Set the name of 'object'
	VIEW3D_API BSTR __stdcall View3D_ObjectNameGetBStr(pr::view3d::Object object);
	VIEW3D_API char const* __stdcall View3D_ObjectNameGet(pr::view3d::Object object);
	VIEW3D_API void __stdcall View3D_ObjectNameSet(pr::view3d::Object object, char const* name);

	// Get the type of 'object'
	VIEW3D_API BSTR __stdcall View3D_ObjectTypeGetBStr(pr::view3d::Object object);
	VIEW3D_API char const* __stdcall View3D_ObjectTypeGet(pr::view3d::Object object);

	// Get/Set the current or base colour of an object(the first object to match 'name') (See LdrObject::Apply)
	VIEW3D_API pr::view3d::Colour __stdcall View3D_ObjectColourGet(pr::view3d::Object object, BOOL base_colour, char const* name);
	VIEW3D_API void __stdcall View3D_ObjectColourSet(pr::view3d::Object object, BOOL base_colour, pr::view3d::Colour colour, char const* name, pr::view3d::EColourOp op, float op_value);
	
	// Reset the object colour back to its default
	VIEW3D_API void __stdcall View3D_ObjectResetColour(pr::view3d::Object object, char const* name);

	// Get/Set the object's o2w transform
	VIEW3D_API pr::view3d::Mat4x4 __stdcall View3D_ObjectO2WGet(pr::view3d::Object object, char const* name);
	VIEW3D_API void __stdcall View3D_ObjectO2WSet(pr::view3d::Object object, pr::view3d::Mat4x4 const& o2w, char const* name);

	// Get/Set the object to parent transform for an object.
	// This is the object to world transform for objects without parents.
	// Note: In "*Box b { 1 1 1 *o2w{*pos{1 2 3}} }" setting this transform overwrites the "*o2w{*pos{1 2 3}}".
	VIEW3D_API pr::view3d::Mat4x4 __stdcall View3D_ObjectO2PGet(pr::view3d::Object object, char const* name);
	VIEW3D_API void __stdcall View3D_ObjectO2PSet(pr::view3d::Object object, pr::view3d::Mat4x4 const& o2p, char const* name);

	// Get/Set the animation time to apply to 'object'
	VIEW3D_API float __stdcall View3D_ObjectAnimTimeGet(pr::view3d::Object object, char const* name);
	VIEW3D_API void __stdcall View3D_ObjectAnimTimeSet(pr::view3d::Object object, float time_s, char const* name);

	// Return the model space bounding box for 'object'
	VIEW3D_API pr::view3d::BBox __stdcall View3D_ObjectBBoxMS(pr::view3d::Object object, pr::view3d::EBBoxFlags bbox_flags);

	// Get/Set the object visibility. See LdrObject::Apply for docs on the format of 'name'
	VIEW3D_API BOOL __stdcall View3D_ObjectVisibilityGet(pr::view3d::Object object, char const* name);
	VIEW3D_API void __stdcall View3D_ObjectVisibilitySet(pr::view3d::Object object, BOOL visible, char const* name);

	// Get/Set wireframe mode for an object (the first object to match 'name'). (See LdrObject::Apply)
	VIEW3D_API BOOL __stdcall View3D_ObjectWireframeGet(pr::view3d::Object object, char const* name);
	VIEW3D_API void __stdcall View3D_ObjectWireframeSet(pr::view3d::Object object, BOOL wireframe, char const* name);

	// Get/Set the object flags. See LdrObject::Apply for docs on the format of 'name'
	VIEW3D_API pr::view3d::ELdrFlags __stdcall View3D_ObjectFlagsGet(pr::view3d::Object object, char const* name);
	VIEW3D_API void __stdcall View3D_ObjectFlagsSet(pr::view3d::Object object, pr::view3d::ELdrFlags flags, BOOL state, char const* name);

	// Get/Set the reflectivity of an object (the first object to match 'name') (See LdrObject::Apply)
	VIEW3D_API float __stdcall View3D_ObjectReflectivityGet(pr::view3d::Object object, char const* name);
	VIEW3D_API void __stdcall View3D_ObjectReflectivitySet(pr::view3d::Object object, float reflectivity, char const* name);

	// Get/Set the sort group for the object or its children. (See LdrObject::Apply)
	VIEW3D_API pr::view3d::ESortGroup __stdcall View3D_ObjectSortGroupGet(pr::view3d::Object object, char const* name);
	VIEW3D_API void __stdcall View3D_ObjectSortGroupSet(pr::view3d::Object object, pr::view3d::ESortGroup group, char const* name);

	// Get/Set 'show normals' mode for an object (the first object to match 'name') (See LdrObject::Apply)
	VIEW3D_API BOOL __stdcall View3D_ObjectNormalsGet(pr::view3d::Object object, char const* name);
	VIEW3D_API void __stdcall View3D_ObjectNormalsSet(pr::view3d::Object object, BOOL show, char const* name);

	// Set the texture/sampler for all nuggets of 'object' or its children. (See LdrObject::Apply)
	VIEW3D_API void __stdcall View3D_ObjectSetTexture(pr::view3d::Object object, pr::view3d::Texture tex, char const* name);
	VIEW3D_API void __stdcall View3D_ObjectSetSampler(pr::view3d::Object object, pr::view3d::Sampler sam, char const* name);

	// Get/Set the nugget flags on an object or its children (See LdrObject::Apply)
	VIEW3D_API pr::view3d::ENuggetFlag __stdcall View3D_ObjectNuggetFlagsGet(pr::view3d::Object object, char const* name, int index);
	VIEW3D_API void __stdcall View3D_ObjectNuggetFlagsSet(pr::view3d::Object object, pr::view3d::ENuggetFlag flags, BOOL state, char const* name, int index);

	// Get/Set the tint colour for a nugget within the model of an object or its children. (See LdrObject::Apply)
	VIEW3D_API pr::view3d::Colour __stdcall View3D_ObjectNuggetTintGet(pr::view3d::Object object, char const* name, int index);
	VIEW3D_API void __stdcall View3D_ObjectNuggetTintSet(pr::view3d::Object object, pr::view3d::Colour colour, char const* name, int index);

	// Materials ******************************

	// Create a texture from data in memory.
	// Set 'data' to nullptr to leave the texture uninitialised, if not null then data must point to width x height pixel data
	// of the size appropriate for the given format. e.g. 'uint32_t px_data[width * height]' for D3DFMT_A8R8G8B8
	// Note: careful with stride, 'data' is expected to have the appropriate stride for BytesPerPixel(format) * width
	VIEW3D_API pr::view3d::Texture __stdcall View3D_TextureCreate(int width, int height, void const* data, size_t data_size, pr::view3d::TextureOptions const& options);

	// Create one of the stock textures
	VIEW3D_API pr::view3d::Texture __stdcall View3D_TextureCreateStock(pr::view3d::EStockTexture stock_texture);

	// Load a texture from file, embedded resource, or stock assets. Specify width == 0, height == 0 to use the dimensions of the file
	VIEW3D_API pr::view3d::Texture __stdcall View3D_TextureCreateFromUri(char const* resource, int width, int height, pr::view3d::TextureOptions const& options);

	// Load a cube map from file, embedded resource, or stock assets. Specify width == 0, height == 0 to use the dimensions of the file
	VIEW3D_API pr::view3d::CubeMap __stdcall View3D_CubeMapCreateFromUri(char const* resource, pr::view3d::CubeMapOptions const& options);

	// Create a texture sampler
	VIEW3D_API pr::view3d::Sampler __stdcall View3D_SamplerCreate(pr::view3d::SamplerOptions const& options);

	// Create one of the stock samplers
	VIEW3D_API pr::view3d::Sampler __stdcall View3D_SamplerCreateStock(pr::view3d::EStockSampler stock_sampler);

	// Create a shader
	VIEW3D_API pr::view3d::Shader __stdcall View3D_ShaderCreate(pr::view3d::ShaderOptions const& options);

	// Create one of the stock shaders
	VIEW3D_API pr::view3d::Shader __stdcall View3D_ShaderCreateStock(pr::view3d::EStockShader stock_shader, char const* config);

	// Read the properties of an existing texture
	VIEW3D_API pr::view3d::ImageInfo __stdcall View3D_TextureGetInfo(pr::view3d::Texture tex);

	// Read the properties of an image file
	VIEW3D_API pr::view3d::EResult __stdcall View3D_TextureGetInfoFromFile(char const* tex_filepath, pr::view3d::ImageInfo& info);

	// Release a reference to a resources
	VIEW3D_API void __stdcall View3D_TextureRelease(pr::view3d::Texture tex);
	VIEW3D_API void __stdcall View3D_CubeMapRelease(pr::view3d::CubeMap tex);
	VIEW3D_API void __stdcall View3D_SamplerRelease(pr::view3d::Sampler sam);
	VIEW3D_API void __stdcall View3D_ShaderRelease(pr::view3d::Shader shdr);

	// Resize this texture to 'size'
	VIEW3D_API void __stdcall View3D_TextureResize(pr::view3d::Texture tex, uint64_t width, uint32_t height, uint16_t depth_or_array_len);

	// Return the ref count of 'tex'
	VIEW3D_API ULONG __stdcall View3D_TextureRefCount(pr::view3d::Texture tex);

	// Get/Set the private data associated with 'guid' for 'tex'
	VIEW3D_API void __stdcall View3d_TexturePrivateDataGet(pr::view3d::Texture tex, GUID const& guid, UINT& size, void* data);
	VIEW3D_API void __stdcall View3d_TexturePrivateDataSet(pr::view3d::Texture tex, GUID const& guid, UINT size, void const* data);
	VIEW3D_API void __stdcall View3d_TexturePrivateDataIFSet(pr::view3d::Texture tex, GUID const& guid, IUnknown* pointer);

	// Resolve a MSAA texture into a non-MSAA texture
	VIEW3D_API void __stdcall View3D_TextureResolveAA(pr::view3d::Texture dst, pr::view3d::Texture src);

	// Gizmos *********************************

	// Create the 3D manipulation gizmo
	VIEW3D_API pr::view3d::Gizmo __stdcall View3D_GizmoCreate(pr::view3d::EGizmoMode mode, pr::view3d::Mat4x4 const& o2w);

	// Delete a 3D manipulation gizmo
	VIEW3D_API void __stdcall View3D_GizmoDelete(pr::view3d::Gizmo gizmo);

	// Attach/Detach callbacks that are called when the gizmo moves
	VIEW3D_API void __stdcall View3D_GizmoMovedCBSet(pr::view3d::Gizmo gizmo, pr::view3d::GizmoMovedCB moved_cb, BOOL add);

	// Attach/Detach an object to the gizmo that will be moved as the gizmo moves
	VIEW3D_API void __stdcall View3D_GizmoAttach(pr::view3d::Gizmo gizmo, pr::view3d::Object obj);
	VIEW3D_API void __stdcall View3D_GizmoDetach(pr::view3d::Gizmo gizmo, pr::view3d::Object obj);

	// Get/Set the scale factor for the gizmo
	VIEW3D_API float __stdcall View3D_GizmoScaleGet(pr::view3d::Gizmo gizmo);
	VIEW3D_API void __stdcall View3D_GizmoScaleSet(pr::view3d::Gizmo gizmo, float scale);

	// Get/Set the current mode of the gizmo
	VIEW3D_API pr::view3d::EGizmoMode __stdcall View3D_GizmoModeGet(pr::view3d::Gizmo gizmo);
	VIEW3D_API void __stdcall View3D_GizmoModeSet(pr::view3d::Gizmo gizmo, pr::view3d::EGizmoMode mode);

	// Get/Set the object to world transform for the gizmo
	VIEW3D_API pr::view3d::Mat4x4 __stdcall View3D_GizmoO2WGet(pr::view3d::Gizmo gizmo);
	VIEW3D_API void __stdcall View3D_GizmoO2WSet(pr::view3d::Gizmo gizmo, pr::view3d::Mat4x4 const& o2w);

	// Get the offset transform that represents the difference between the gizmo's transform at the start of manipulation and now.
	VIEW3D_API pr::view3d::Mat4x4 __stdcall View3D_GizmoOffsetGet(pr::view3d::Gizmo gizmo);

	// Get/Set whether the gizmo is active to mouse interaction
	VIEW3D_API BOOL __stdcall View3D_GizmoEnabledGet(pr::view3d::Gizmo gizmo);
	VIEW3D_API void __stdcall View3D_GizmoEnabledSet(pr::view3d::Gizmo gizmo, BOOL enabled);

	// Returns true while manipulation is in progress
	VIEW3D_API BOOL __stdcall View3D_GizmoManipulating(pr::view3d::Gizmo gizmo);

	// Diagnostics ****************************

	// Get/Set whether object bounding boxes are visible
	VIEW3D_API BOOL __stdcall View3D_DiagBBoxesVisibleGet(pr::view3d::Window window);
	VIEW3D_API void __stdcall View3D_DiagBBoxesVisibleSet(pr::view3d::Window window, BOOL visible);

	// Get/Set the length of the vertex normals
	VIEW3D_API float __stdcall View3D_DiagNormalsLengthGet(pr::view3d::Window window);
	VIEW3D_API void __stdcall View3D_DiagNormalsLengthSet(pr::view3d::Window window, float length);

	// Get/Set the colour of the vertex normals
	VIEW3D_API pr::view3d::Colour __stdcall View3D_DiagNormalsColourGet(pr::view3d::Window window);
	VIEW3D_API void __stdcall View3D_DiagNormalsColourSet(pr::view3d::Window window, pr::view3d::Colour colour);

	VIEW3D_API pr::view3d::Vec2 __stdcall View3D_DiagFillModePointsSizeGet(pr::view3d::Window window);
	VIEW3D_API void __stdcall View3D_DiagFillModePointsSizeSet(pr::view3d::Window window, pr::view3d::Vec2 size);

	// Miscellaneous **************************

	// Create a render target texture on a D3D9 device. Intended for WPF D3DImage
	VIEW3D_API pr::view3d::Texture __stdcall View3D_CreateDx9RenderTarget(HWND hwnd, UINT width, UINT height, pr::view3d::TextureOptions const& options, HANDLE* shared_handle);

	// Create a Texture instance from a shared d3d resource (created on a different d3d device)
	VIEW3D_API pr::view3d::Texture __stdcall View3D_CreateTextureFromSharedResource(IUnknown* shared_resource, pr::view3d::TextureOptions const& options);

	// Return the supported MSAA quality for the given multi-sampling count
	VIEW3D_API int __stdcall View3D_MSAAQuality(int count, DXGI_FORMAT format);

	// Get/Set the visibility of one or more stock objects (focus point, origin, selection box, etc)
	VIEW3D_API BOOL __stdcall View3D_StockObjectVisibleGet(pr::view3d::Window window, pr::view3d::EStockObject stock_objects);
	VIEW3D_API void __stdcall View3D_StockObjectVisibleSet(pr::view3d::Window window, pr::view3d::EStockObject stock_objects, BOOL show);

	// Get/Set the size of the focus point
	VIEW3D_API float __stdcall View3D_FocusPointSizeGet(pr::view3d::Window window);
	VIEW3D_API void __stdcall View3D_FocusPointSizeSet(pr::view3d::Window window, float size);

	// Get/Set the size of the origin point
	VIEW3D_API float __stdcall View3D_OriginPointSizeGet(pr::view3d::Window window);
	VIEW3D_API void __stdcall View3D_OriginPointSizeSet(pr::view3d::Window window, float size);

	// Get/Set the position and size of the selection box
	VIEW3D_API void __stdcall View3D_SelectionBoxGet(pr::view3d::Window window, pr::view3d::BBox& bbox, pr::view3d::Mat4x4& o2w);
	VIEW3D_API void __stdcall View3D_SelectionBoxSet(pr::view3d::Window window, pr::view3d::BBox const& bbox, pr::view3d::Mat4x4 const& o2w);

	// Set the selection box to encompass all selected objects
	VIEW3D_API void __stdcall View3D_SelectionBoxFitToSelected(pr::view3d::Window window);

	// Create/Delete the demo scene in the given window
	VIEW3D_API GUID __stdcall View3D_DemoSceneCreateText(pr::view3d::Window window);
	VIEW3D_API GUID __stdcall View3D_DemoSceneCreateBinary(pr::view3d::Window window);
	VIEW3D_API void __stdcall View3D_DemoSceneDelete();

	// Show a window containing the demo script
	VIEW3D_API void __stdcall View3D_DemoScriptShow(pr::view3d::Window window);

	// Return the example Ldr script as a BSTR
	VIEW3D_API BSTR __stdcall View3D_ExampleScriptBStr();

	// Return the auto complete templates as a BSTR
	VIEW3D_API BSTR __stdcall View3D_AutoCompleteTemplatesBStr();

	// Return the hierarchy "address" for a position in an ldr script file.
	VIEW3D_API BSTR __stdcall View3D_ObjectAddressAt(wchar_t const* ldr_script, int64_t position);

	// Parse a transform description using the Ldr script syntax
	VIEW3D_API pr::view3d::Mat4x4 __stdcall View3D_ParseLdrTransform(char const* ldr_script);

	// Handle standard keyboard shortcuts. 'key_code' should be a standard VK_ key code with modifiers included in the hi word. See 'EKeyCodes'. 'ss_point' is the mouse position in screen space (pixels)
	VIEW3D_API BOOL __stdcall View3D_TranslateKey(pr::view3d::Window window, int key_code, pr::view3d::Vec2 ss_point);

	// Return the reference count of a COM interface
	VIEW3D_API ULONG __stdcall View3D_RefCount(IUnknown* pointer);

	// Tools **********************************

	// Show/Hide the object manager tool
	VIEW3D_API BOOL __stdcall View3D_ObjectManagerVisibleGet(pr::view3d::Window window);
	VIEW3D_API void __stdcall View3D_ObjectManagerVisibleSet(pr::view3d::Window window, BOOL show);

	// Show/Hide the script editor tool
	VIEW3D_API BOOL __stdcall View3D_ScriptEditorVisibleGet(pr::view3d::Window window);
	VIEW3D_API void __stdcall View3D_ScriptEditorVisibleSet(pr::view3d::Window window, BOOL show);

	// Show/Hide the measurement tool
	VIEW3D_API BOOL __stdcall View3D_MeasureToolVisibleGet(pr::view3d::Window window);
	VIEW3D_API void __stdcall View3D_MeasureToolVisibleSet(pr::view3d::Window window, BOOL show);

	// Show/Hide the angle measurement tool
	VIEW3D_API BOOL __stdcall View3D_AngleToolVisibleGet(pr::view3d::Window window);
	VIEW3D_API void __stdcall View3D_AngleToolVisibleSet(pr::view3d::Window window, BOOL show);

	// Show/Hide the lighting controls UI
	VIEW3D_API void __stdcall View3D_LightingControlsUI(pr::view3d::Window window, BOOL show);

#if 0
	// Ldr Editor Ctrl
	VIEW3D_API HWND __stdcall View3D_LdrEditorCreate          (HWND parent);
	VIEW3D_API void __stdcall View3D_LdrEditorDestroy         (HWND hwnd);
	VIEW3D_API void __stdcall View3D_LdrEditorCtrlInit        (HWND scintilla_control, BOOL dark);
#endif
}

namespace pr::view3d
{
	namespace impl
	{
		struct Deleter
		{
			void operator()(Object p) noexcept { if (p) View3D_ObjectDelete(p); }
			void operator()(Gizmo p) noexcept { if (p) View3D_GizmoDelete(p); }
			void operator()(Texture p) noexcept { if (p) View3D_TextureRelease(p); }
			void operator()(CubeMap p) noexcept { if (p) View3D_CubeMapRelease(p); }
			void operator()(Sampler p) noexcept { if (p) View3D_SamplerRelease(p); }
			void operator()(Shader p) noexcept { if (p) View3D_ShaderRelease(p); }
			void operator()(Window p) noexcept { if (p) View3D_WindowDestroy(p); }
		};
	}

	using ObjectPtr  = std::unique_ptr<rdr12::ldraw::LdrObject, impl::Deleter>;
	using GizmoPtr   = std::unique_ptr<rdr12::ldraw::LdrGizmo, impl::Deleter>;
	using TexturePtr = std::unique_ptr<rdr12::Texture2D, impl::Deleter>;
	using CubeMapPtr = std::unique_ptr<rdr12::TextureCube, impl::Deleter>;
	using SamplerPtr = std::unique_ptr<rdr12::Sampler, impl::Deleter>;
	using ShaderPtr  = std::unique_ptr<rdr12::Shader, impl::Deleter>;
	using WindowPtr  = std::unique_ptr<rdr12::V3dWindow, impl::Deleter>;
}