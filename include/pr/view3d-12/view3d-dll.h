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
#include <string_view>
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
		struct PopulationPrototype;
		struct Population;
		namespace materials
		{
			struct DetailStrengthMap;
		}
		namespace ldraw
		{
			struct LdrObject;
			struct LdrGizmo;
		}
	}
	namespace game_systems
	{
		struct WeatherMap;
		namespace ocean
		{
			struct OceanSurface;
		}
		namespace atmosphere
		{
			struct AtmosphereRecord;
		}
		namespace particles
		{
			struct ParticleSystem;
		}
	}

	namespace view3d
	{
		using DllHandle = std::byte const*;
		using Object = rdr12::ldraw::LdrObject*;
		using Gizmo = rdr12::ldraw::LdrGizmo*;
		using Texture = rdr12::Texture2D*;
		using CubeMap = rdr12::TextureCube*;
		using Sampler = rdr12::Sampler*;
		using Shader = rdr12::Shader*;
		using WeatherMap = game_systems::WeatherMap*;
		using Ocean = game_systems::ocean::OceanSurface*;
		using Atmosphere = game_systems::atmosphere::AtmosphereRecord*;
		using ParticleSystem = game_systems::particles::ParticleSystem*;
		using PopulationPrototype = rdr12::PopulationPrototype*;
		using Population = rdr12::Population*;
		using DetailStrengthMap = rdr12::materials::DetailStrengthMap*;
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
			ShadowMap,
			RayCast,
		};
		enum class ENuggetFlag :int // rdr12::ENuggetFlag
		{
			// PR_CODE_SYNC_BEGIN(view3d::ENuggetFlag, source_of_truth)
			None = 0,

			// Exclude this nugget when rendering a model
			Hidden = 1 << 0,

			// Set if the geometry data for the nugget contains alpha colours
			GeometryHasAlpha = 1 << 1,

			// Force alpha blending for this nugget. Use this when alpha can be supplied by runtime state outside the material.
			AlphaBlend = 1 << 2,

			// Excluded from shadow map render steps
			ShadowCastExclude = 1 << 4,

			// Can overlap with other nuggets.
			// Set this flag to true if you want to add a nugget that overlaps the range
			// of an existing nugget. For simple models, overlapping nugget ranges is
			// usually an error, but in advanced cases it isn't.
			RangesCanOverlap = 1 << 5,

			// Draw alpha with weighted blended transparency instead of the sorted K-buffer layers. Weighted fragments have no per-pixel order among
			// themselves, so this suits soft, overlapping effects like smoke, where many layers would overflow the K-buffer.
			AlphaWeighted = 1 << 6,

			_flags_enum = 0,
			// PR_CODE_SYNC_END()
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
		// Representative visual starting points for procedural surface parameters.
		enum class EProceduralSurfacePreset :int
		{
			Soil,
			Grass,
			Sand,
			Rock,
			Snow,
		};
		// Coordinate frame used by procedural surface evaluation.
		enum class EProceduralCoordinateSpace :int
		{
			Object,
			World,
		};
		// How a normal map's samples are interpreted (see View3D_ObjectNuggetNormalMapSet).
		enum class ENormalMapSpace :int
		{
			Tangent,
			Model,
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
		// Defines how indexed vertex IDs are interpreted by an extended object.
		enum class EVertexSource :int
		{
			Buffer,             // Indices address physical vertex-buffer elements.
			ProceduralVertexId, // U32 indices are logical SV_VertexID values decoded by caller vertex shaders.
			GpuGeneratedBuffer, // A compute shader fills the canonical buffered vertices once during creation.
		};
		enum class EAOQuality :int // rdr12::EAOQuality
		{
			Low,
			Medium,
			High,
		};
		enum class EAOResolution :int // rdr12::EAOResolution
		{
			Half,
			Full,
		};
		enum class ELight :int
		{
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
			// PR_CODE_SYNC_BEGIN(view3d::ELdrFlags, source_of_truth)

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

			// Not rendered into captured environment maps (EnvMapCapture and the environment map probe)
			EnvMapCaptureExclude = 1 << 15,

			// Indicates invalidated flags that need to be refreshed
			Invalidated = 1 << 31,

			// Bitwise operators
			_flags_enum = 0,

			// PR_CODE_SYNC_END()
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
			GroupTint    = 1 << 5,
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
			PreAlpha = Default+16, // Last group before the alpha group
			Alpha,                 // Alpha nuggets, collected into the K-buffer
			AlphaWeighted,         // Alpha nuggets drawn with weighted blended transparency (see ENuggetFlag::AlphaWeighted)
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
		// The event that triggered the store change
		enum class EStoreChangeInitiator
		{
			// PR_CODE_SYNC_BEGIN(pr::rdr12::ldraw::EStoreChangeInitiator)

			// A new source (e.g. file, string, stream, etc) was added to the store
			NewSource,

			// A source was removed from the store
			SourceRemoved,

			// Existing sources refreshed their data
			Reload,

			// More data from a streaming source has arrived
			StreamData,

			// Objects were removed from a context, but the context itself was not deleted.
			// The objects will have been removed from any windows before the store change event.
			// There is not "before" event for this initiator type.
			ObjectsDeleted,

			// PR_CODE_SYNC_END();
		};

		// Flags describing how the store was changed
		enum class EStoreChangeFlags
		{
			// PR_CODE_SYNC_BEGIN(pr::rdr12::ldraw::EStoreChangeFlags)

			None = 0,

			// One or more objects were removed
			ObjectsAdded = 1 << 0,

			// One or more objects were removed
			ObjectsRemoved = 1 << 1,

			// A new context id was added to the store
			ContextIdAdded = 1 << 2,

			// A context id was removed from the store
			ContextIdRemoved = 1 << 3,

			_flags_enum = 0,

			// PR_CODE_SYNC_END();
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
			// PR_CODE_SYNC_BEGIN(view3d::ESettings, source_of_truth)

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

			Rendering            = 1 << 21,
			Rendering_RayTracing = Rendering | 1 << 0,
			Rendering_PostEffects = Rendering | 1 << 2,
			Rendering_Stereo     = Rendering | 1 << 3,

			_flags_enum = 0,

			// PR_CODE_SYNC_END()
		};
		#pragma endregion

		#pragma region API Compatibility
		enum class ERayTracingTier :int
		{
			NotSupported,
			Tier1_0,
			Tier1_1,
			Unknown,
		};
		enum class ERayTracingFeature :int
		{
			None        = 0,
			Reflections = 1 << 0,
			Caustics    = 1 << 1,
			All         = (1 << 0) | (1 << 1),
			_flags_enum = 0,
		};
		struct RayTracingProps
		{
			ERayTracingFeature m_features;
			int m_max_reflection_bounces;
		};
		struct StereoProps
		{
			// True to render side-by-side stereo: the left eye view in the left half of the output, the right eye view in the right half.
			BOOL m_enabled;

			// The distance between the eyes, in millimetres. Zero means automatic: one thirtieth of the convergence distance.
			float m_eye_separation_mm;

			// The length of one scene unit, in millimetres. Converts 'm_eye_separation_mm' into scene units. Zero means 1000 (scene units are metres).
			float m_scene_unit_mm;

			// The distance from the camera to the plane that appears at screen depth (zero disparity), in scene units. Zero means the camera focus distance.
			float m_convergence;
		};
		struct RayTracingInfo
		{
			BOOL m_requested;
			BOOL m_hardware_supported;
			BOOL m_available;
			BOOL m_enabled;
			ERayTracingTier m_tier;
		};
		struct StrView
		{
			char const* ptr = nullptr;
			size_t size = 0;

			operator std::string_view() const { return { ptr, size }; }
			friend bool operator==(StrView lhs, StrView rhs) { return std::string_view{ lhs.ptr, lhs.size } == std::string_view{ rhs.ptr, rhs.size }; }
			friend bool operator!=(StrView lhs, StrView rhs) { return !(lhs == rhs); }
		};
		struct WStrView
		{
			wchar_t const* ptr = nullptr;
			size_t size = 0;

			operator std::wstring_view() const { return { ptr, size }; }
			friend bool operator==(WStrView lhs, WStrView rhs) { return std::wstring_view{ lhs.ptr, lhs.size } == std::wstring_view{ rhs.ptr, rhs.size }; }
			friend bool operator!=(WStrView lhs, WStrView rhs) { return !(lhs == rhs); }
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

		// Procedural sky state. The sky is Z-up. 'm_sun_direction' points toward the sun; the finite sun colour and intensity must be nonnegative.
		// 'm_cloud_cover' in [0,1] is the default cover (0 = clear, 0.5 = scattered white cloud, 1 = dark overcast), used where no weather map applies.
		// 'm_wind_speed' (>= 0, world units per second) and 'm_wind_direction' (radians from +X toward +Y) move the clouds; lower layers move faster.
		// Direction changes are smooth: the clouds keep their positions, and their streaks turn by cross-fading to clouds aligned with the new direction.
		// 'm_time' is the caller's absolute time in seconds; clouds advance by the change in time between updates, and time may not go backwards.
		// 'm_hidden_cloud_layers' is a bit mask: bit i hides cloud layer i (0 = low, 1 = mid, 2 = cirrus). Zero shows all layers.
		// 'm_lightning' are flashes inside the cloud: xy = position in the sky frame, z = radius (> 0), w = finite brightness (>= 0, 0 = no flash).
		// 'm_cloud_shadow_strength' in [0,1] scales how much the clouds shade the scene's directional light that follows the sun (0 = no cloud shadows).
		// 'm_moon_intensity' (finite, >= 0) is the moon's brightness: 0 hides the moon, 1 is a full moon on a clear night.
		// 'm_moon_direction' points toward the moon. 'm_moon_light_direction' points from the moon toward the light that lights it, which sets the phase:
		// equal to m_moon_direction is a new moon, equal to -m_moon_direction is a full moon. Both must be finite and nonzero when the moon is shown.
		// 'm_sky_to_stars' rotates sky-frame directions into the star frame (Z = north celestial pole, X = right ascension 0h). Must be a rotation.
		struct ProceduralSkySettings
		{
			Vec4 m_sun_direction = { 0.5f, 0.3f, 0.8f, 0.0f };
			Vec4 m_sun_colour = { 1.0f, 0.95f, 0.85f, 1.0f };
			float m_sun_intensity = 1.0f;
			float m_cloud_cover = 0.0f;
			float m_wind_speed = 0.0f;
			float m_wind_direction = 0.0f;
			double m_time = 0.0;
			uint32_t m_hidden_cloud_layers = 0;
			Vec4 m_lightning[4] = {};
			float m_cloud_shadow_strength = 1.0f;
			float m_moon_intensity = 0.0f;
			Vec4 m_moon_direction = { -0.5f, -0.3f, 0.5f, 0.0f };
			Vec4 m_moon_light_direction = { 0.5f, 0.3f, -0.5f, 0.0f };
			Mat4x4 m_sky_to_stars = { {1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1} };
		};

		// Sun, moon and star state from the Sun–Earth–Moon model, in the procedural sky's frame (+Z up, +Y north, +X east). See CelestialState
		// in pr/game-systems/sky/celestial_clock.h. The directions and rotation can be copied directly into ProceduralSkySettings.
		struct CelestialState
		{
			Vec4 m_sun_direction;
			Vec4 m_moon_direction;
			Vec4 m_moon_light_direction;
			Mat4x4 m_sky_to_stars;
			float m_lit_fraction;
		};


		// Whole-screen underwater post effect. Colours are sRGB ARGB; alpha is ignored.
		// Requires visibility > 0 (positive infinity disables the fog), finite distortion amplitude >= 0, frequency > 0, and speed >= 0.
		// 'm_surface' is a world-space plane with its normal pointing out of the water; fog applies only below it.
		// It must be finite, and either zero (the whole view is in water) or have a non-zero normal.
		// 'm_fade_depth' (finite, >= 0) is the depth below the surface over which the effect fades in; zero gives a sharp waterline.
		// 'm_waterline_offsets' (finite) are the real surface's heights above 'm_surface' at an 8x8 grid on the near plane, row-major from the
		// top-left corner of the viewport. The waterline follows the interpolated heights. See UnderwaterProps in post_processing.h.
		struct UnderwaterProps
		{
			BOOL m_enabled = FALSE;
			Colour m_tint = 0xFFA6D9F2U;
			Colour m_fog_colour = 0xFF0A384DU;
			float m_visibility = 40.0f;
			float m_distortion_amplitude = 0.002f;
			float m_distortion_frequency = 6.0f;
			float m_distortion_speed = 0.25f;
			Vec4 m_surface = {};
			float m_fade_depth = 0.0f;
			float m_waterline_offsets[64] = {};
		};

		// Screen-space ambient occlusion of opaque surfaces. Requires finite 'm_radius' > 0 (world units), 'm_intensity' >= 0, 'm_falloff' in (0,1],
		// and 'm_max_screen_radius' > 0 (fraction of the viewport height). See AmbientOcclusionProps in post_processing.h.
		struct AmbientOcclusionProps
		{
			BOOL m_enabled = FALSE;
			float m_radius = 0.5f;
			float m_intensity = 1.0f;
			float m_falloff = 0.6f;
			float m_max_screen_radius = 0.1f;
			EAOQuality m_quality = EAOQuality::Medium;
			EAOResolution m_resolution = EAOResolution::Half;
			BOOL m_debug_view = FALSE;
		};

		// Parameters for a UV-free GPU procedural surface applied to a PBR material.
		struct ProceduralSurface
		{
			// Field order is part of the native/managed ABI contract.

			EProceduralCoordinateSpace m_coordinate_space;
			uint32_t m_seed;
			float m_feature_scale;
			float m_pad0;
			Vec4 m_coordinate_origin;
			Vec4 m_axis_scale;
			Colour m_colour0;
			Colour m_colour1;
			Colour m_colour2;
			Colour m_colour3;
			float m_normal_strength;
			float m_roughness_min;
			float m_roughness_max;
			float m_detail;
			float m_warp;
			float m_near_feature_scale;
			float m_near_normal_strength;
			float m_near_fade_start;
			float m_near_fade_end;
			float m_pad1;
			float m_pad2;
			float m_pad3;
		};

		// One world-space projection of a detail-normal slope map. See View3D_ObjectDetailNormalLayersSet.
		struct DetailNormalLayer
		{
			// Field order is part of the native/managed ABI contract.

			Vec4 m_row_u;         // u = dot(world_pos.xyz, m_row_u.xyz) + m_row_u.w
			Vec4 m_row_v;         // v = dot(world_pos.xyz, m_row_v.xyz) + m_row_v.w
			float m_height_scale;    // World height per unit of map height, per texture unit of slope
			float m_weight_noise;    // In [0, 1]. Smooth world noise scales the height by a factor in [1 - m_weight_noise, 1 + m_weight_noise]
			float m_noise_frequency; // Finite, and positive when m_weight_noise or m_warp is non-zero. Noise cells per texture unit; ignores row offsets
			float m_warp;            // Finite and non-negative. Noise shifts the texture coordinates by up to this many texture units
			float m_pad0;
		};
		struct BBox
		{
			Vec4 centre;
			Vec4 radius;
		};

		// Flags of a population instance. Matches VIEW3D_POPULATION_FLAG_* in 'pr/view3d-12/shaders/population.hlsli'.
		enum class EPopulationInstanceFlags : uint32_t
		{
			None = 0,
			Enabled = 1 << 0, // Only enabled instances are drawn.
			Faded = 1 << 1, // Dithered by the instance's own unorm16 coverage in bits 16-31, instead of by its prototype's fade bands.
			FadeComplement = 1 << 2, // A 'Faded' instance keeps the pixels that its coverage would discard. See View3D_PopulationDitherWord.
			_flags_enum = 0,
		};

		// One placement of a population prototype. Matches 'View3DPopulationInstance' in 'pr/view3d-12/shaders/population.hlsli'.
		struct PopulationInstance
		{
			Vec4 m_o2w[3];                      // Rows of the affine instance-to-world transform: world[i] = dot(m_o2w[i], (p,1)).
			Colour m_tint;                      // sRGB ARGB bytes with linear alpha; multiplies vertex colour after decoding RGB.
			EPopulationInstanceFlags m_flags;
			uint32_t m_payload;                 // Caller payload index, interpreted only by caller shaders.
			uint32_t m_prototype;               // The index of the instance's prototype in the population. Instances with an invalid index are not drawn.
		};

		// One level of detail of a population prototype. See View3D_PopulationPrototypeCreate.
		struct PopulationLod
		{
			Object m_object;      // Supplies the LOD's model and its instance material override, if any. Child objects are not used.
			float m_min_size;     // The minimum projected radius, in pixels, of the prototype bounding sphere at which this LOD is drawn.
			float m_transition;   // The dithered transition band above 'm_min_size', as a fraction of 'm_min_size'. Zero is an abrupt switch.
		};

		// A camera distance band [m_start, m_start + m_width] in which population instances fade in or out. Zero start and width means unused.
		struct PopulationFade
		{
			float m_start; // The distance at which the band begins.
			float m_width; // The width of the band. Zero is an abrupt switch at 'm_start'.
		};

		// Counters of a renderer's pipeline state cache. See View3D_WindowPipelineStats.
		struct PipelineStats
		{
			int64_t m_created;        // Pipelines created while recording draws. Each creation can stall the frame that draws it.
			double m_created_ms;      // The total time spent creating pipelines while recording draws.
			double m_created_max_ms;  // The longest single creation while recording draws.
			int64_t m_prewarmed;      // Pipelines created ahead of use by View3D_WindowPrewarmPipelines.
			double m_prewarmed_ms;    // The total worker time spent creating prewarmed pipelines.
			int64_t m_failed;         // Prewarm creations that failed. The failure is reported again if the state is drawn.
			int64_t m_pending;        // Prewarm requests that are queued or being created.
			int64_t m_count;          // The number of cached pipelines.
		};

		// The nugget draws recorded by the forward passes of a window's latest frame, summed over its views and scenes. See View3D_WindowDrawStats.
		struct DrawStats
		{
			int64_t m_forward;          // All forward nugget draw commands, direct and indirect.
			int64_t m_forward_indirect; // The forward draws whose arguments are read from a GPU buffer (populations and GPU-written draw arguments).
		};

		// Diagnostics of a population. See View3D_PopulationStats.
		struct PopulationStats
		{
			// Current CPU state.
			int64_t m_instance_capacity;   // The number of instance slots.
			int64_t m_block_capacity;      // The number of block slots.
			int64_t m_active_blocks;       // Blocks that are set and not removed.
			int64_t m_admitted_instances;  // The sum of the active blocks' capacities: the most instances that can be live.
			int64_t m_resident_bytes;      // The size of the instance, live count, payload, and prototype buffers.
			int64_t m_scratch_bytes;       // The size of all per-view classification buffers.
			int64_t m_scratch_high_water;  // The largest value 'm_scratch_bytes' has had.
			int64_t m_upload_bytes;        // The total bytes copied to the GPU by prototype, instance, block, and payload updates.
			int64_t m_producer_dispatches; // The total number of producer dispatches recorded.

			// The latest classification of each view that the GPU has completed, summed over 'm_views' views.
			int64_t m_views;                 // The number of views with completed counters.
			int64_t m_blocks_rejected;       // Active blocks skipped by the block test (frustum or draw distance).
			int64_t m_blocks_overflow;       // Visible blocks not drawn because a view exceeded 65535 visible blocks.
			int64_t m_instances_tested;      // Live instance slots tested by the GPU.
			int64_t m_entries_emitted;       // Visible list entries written. An instance in a LOD transition writes two.
			int64_t m_entries_per_lod[8];    // Visible list entries of each LOD.
			int64_t m_indirect_draws;        // Indirect draws: one per (LOD, nugget) pair.
			int64_t m_live_count_violations; // Visible blocks whose live count exceeded their capacity. The live count is clamped.
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
			EGeom       m_geom;         // Surface attributes: supplied vertex fields for Buffer, shader-generated outputs for ProceduralVertexId.
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
			Colour m_diffuse;
			Colour m_specular;
			float m_specular_power;
			float m_intensity;
			float m_range;
			float m_falloff;
			float m_inner_angle;
			float m_outer_angle;
			float m_cast_shadow;
			int   m_shadow_cascades;   // Directional lights only: the number of shadow cascades. Zero means use the scene shadow settings
			int   m_shadow_resolution; // Directional lights only: requested size of each shadow cascade view (in pixels). Zero means use the scene shadow settings
			float m_shadow_distance;   // Directional lights only: distance from the camera beyond which the light casts no shadows. Zero means use the scene shadow settings
			float m_shadow_fade_start; // Directional lights only: distance from the camera at which shadows start to fade out. Zero means 90% of the shadow distance
			BOOL m_cam_relative;
			BOOL m_on;
		};
		struct ShadowSettings
		{
			int   m_atlas_size;             // Width and height of the square shadow atlas (in pixels). Must be a power of two
			int   m_directional_resolution; // Requested size of each directional light cascade view (in pixels)
			int   m_far_cascade_resolution; // Requested size of each directional light cascade after the first (in pixels). Zero means the same as 'm_directional_resolution'
			int   m_spot_resolution;          // Largest size of a spot light shadow view (in pixels)
			int   m_point_resolution;       // Largest size of each of the six point light shadow views (in pixels)
			int   m_max_shadow_lights;      // The maximum number of lights that cast shadows. Zero disables shadows
			int   m_cascade_count;          // The number of cascades for directional lights, in [1,4]
			float m_shadow_distance;        // Distance from the camera beyond which directional lights cast no shadows. Shadows fade out over the last 10% of this distance. Zero means fit to the shadow casters
			float m_cascade_split_blend;    // Cascade split distribution in [0,1]. 0 = even spacing, 1 = logarithmic spacing
			float m_first_cascade_distance; // Distance from the camera to the far end of the first cascade. Zero means the first cascade is spaced like the others
			int   m_filter_size;             // Width of the shadow edge filter (in shadow texels). Either 5 or 7
			int   m_depth_bias;             // Constant depth bias (in units of the smallest depth step)
			float m_slope_bias;             // Depth bias scaled by the depth slope of each triangle
			float m_normal_bias;            // Receiver offset along the surface normal (in shadow texels)
			BOOL  m_cache_views;            // Re-render shadow views only when their content changes
			int   m_far_cascade_refresh_interval; // Frames between refreshes of each directional cascade after the first. View and content changes wait for the cascade's turn. Until then the cascade shows its last rendered view, or no shadow if it has none. Zero or one means every frame
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
		// Hardware stage selected by the shader creation descriptor.
		enum class EShaderStage : int
		{
			Vertex,
			Pixel,
			Geometry,
			Hull,
			Domain,
			Compute,
		};
		// The stock forward material shading that a procedural forward pixel family is built on and replaces.
		enum class EForwardPixelModel : int
		{
			Simple,       // Stock simple-material shading. See 'pr/view3d-12/shaders/forward_pixel.hlsli'.
			Pbr,          // Stock PBR shading without extra texture-coordinate streams. See 'pr/view3d-12/shaders/forward_pixel_pbr.hlsli'.
			SimpleDetail, // Stock simple-material shading with detail normals. See 'pr/view3d-12/shaders/forward_pixel_detail.hlsli'.
		};
		// Caller-owned shader bytecode, copied when the shader is created.
		struct ShaderByteCode
		{
			void const* m_bytecode;
			size_t m_size;
		};
		// Binding recipe for a procedural shader overlay on a stock raster pass.
		struct ProceduralBinding
		{
			static constexpr size_t ConstantsSize = 1024;
			static constexpr size_t MaxBufferSize = 64 * 1024 * 1024;
			static constexpr size_t ForwardPixelCount = 4;
			static constexpr int BufferSlotCount = 2;
			static constexpr int TextureSlotCount = 4;

			ERenderStep m_rdr_step;
			void const* m_constants;
			size_t m_constants_size;
			void const* m_buffer;    // Optional immutable data, copied once and bound as a raw root SRV (ByteAddressBuffer, no bounds) at VIEW3D_PROCEDURAL_BUFFER0_REGISTER. Null for none.
			size_t m_buffer_size;    // Size of 'm_buffer' in bytes. A non-zero multiple of 4 up to MaxBufferSize when 'm_buffer' is not null, otherwise 0.

			// Forward pixel family that replaces the stock forward pixel shaders of the 'm_forward_pixel_model' material model. The first three entries are all null,
			// or all valid, and only for ForwardRender. The AlphaWeighted entry is optional; without it the shader cannot draw ENuggetFlag::AlphaWeighted nuggets.
			// Optional for the Vertex stage and required for the Pixel stage. Materials of other models reject the family.
			// Order: Opaque, ReflectionAttrs, AlphaCollect, AlphaWeighted. See 'pr/view3d-12/shaders/forward_pixel.hlsli'.
			EForwardPixelModel m_forward_pixel_model;
			ShaderByteCode m_forward_pixel[ForwardPixelCount];
		};
		// Versioned shader creation descriptor. Only Vertex and Pixel with ProceduralBinding are currently implemented. See View3D_ShaderCreate.
		// Callers supply readable descriptor storage and buffers matching this header and runtime; size/version are not a global ABI handshake.
		struct ShaderOptions
		{
			static constexpr int CurrentVersion = 9;
			static constexpr size_t MaxByteCodeSize = 1024 * 1024;

			int m_struct_size;
			int m_version;
			EShaderStage m_stage;
			void const* m_bytecode;
			size_t m_bytecode_size;
			char const* m_dbg_name;
			ProceduralBinding m_procedural;
		};
		// Versioned extended-object descriptor with an explicit vertex source and authoritative model-space bounds.
		struct ObjectCreateOptions
		{
			static constexpr int CurrentVersion = 2;

			int m_struct_size;
			int m_version;
			EVertexSource m_vertex_source;  // Selects supplied buffered vertices, procedural logical IDs, or a one-time generated vertex buffer.
			int m_vcount_logical;           // Positive logical ID limit for ProceduralVertexId; every U32 index must be below this count.
			BBox m_bbox;                    // Required finite bounds when positions are not supplied through the public vertex array.
			void const* m_compute_bytecode; // GpuGeneratedBuffer compute bytecode using b0, u0 RWStructuredBuffer<View3DVertex>, and t0 when input is supplied.
			size_t m_compute_bytecode_size;
			void const* m_constants;        // GpuGeneratedBuffer immutable constants copied for the dispatch.
			size_t m_constants_size;
			int m_thread_group_size_x;      // GpuGeneratedBuffer numthreads X dimension used to derive the dispatch count.
			void const* m_input;            // Optional GpuGeneratedBuffer input copied to a GPU buffer and bound as a root SRV at t0 for the dispatch only.
			size_t m_input_size;            // Size of 'm_input' in bytes. A nonzero multiple of 4 when 'm_input' is supplied, otherwise zero.
		};
		// The fixed layout of an ocean's wind sea: waves whose amplitude and crest sharpness follow the wind at each place. Wind node (i, j)
		// is at origin + cell * (i, j) and holds 'm_level_count' wind vectors; the table holds 'm_speed_count' rows per band at multiples of
		// 'm_speed_step_mps'. See pr/physics/terrain/water/wind_sea.hlsli for the full contract.
		struct OceanWindSeaGrid
		{
			double m_origin_x_m;
			double m_origin_y_m;
			double m_cell_m;         // Node spacing (m). Positive.
			double m_speed_step_mps; // Wind speed between table rows (m/s). Positive.
			int32_t m_width;         // Nodes along x and y, each at least 2.
			int32_t m_height;
			int32_t m_band_count;    // Wavelength bands, in [1, 5].
			int32_t m_speed_count;   // Table rows per band, at least 2.
			int32_t m_level_count;   // Wind vectors per node, in [1, band_count].
			int32_t m_reserved;
		};

		// The data of an ocean's wind sea, laid out by its OceanWindSeaGrid.
		struct OceanWindSea
		{
			// 'm_element_count' (at most 40) Gerstner wave elements of 64 bytes each, in the shared physics water-field layout. Wind-sea elements
			// name their band in info.y (see WaterFieldWindSeaBand); their amplitude and crest sharpness come from the table and the local wind.
			// Null keeps the current elements and table.
			void const* m_elements;

			// band_count * speed_count rows of four floats (see WindSeaTableRow), indexed [band * speed_count + row]. Required with 'm_elements',
			// and otherwise ignored.
			float const* m_table;

			// width * height * level_count (x, y) downwind vectors (m/s), indexed [(j * width + i) * level_count + level]. Required.
			float const* m_winds;
			int32_t m_element_count;
			int32_t m_reserved;
		};

		// Fixed inputs of an ocean surface. World positions are metres, Z up, with the ocean's circular domain centred on the origin.
		struct OceanOptions
		{
			// The wind-sea layout, which is fixed for the ocean's lifetime, and its initial data, which requires elements.
			// View3D_OceanSetWindSea changes the data, so changing weather never needs a new ocean.
			OceanWindSeaGrid m_wind_sea_grid;
			OceanWindSea m_wind_sea;

			// Ground heights at 'm_bathymetry_width' x 'm_bathymetry_height' nodes, row-major. Node (i, j) is at origin + cell * (i, j).
			// Waves are corrected for the depth below the water level, and vertices over ground above the reach of the swash stay flat.
			float const* m_bathymetry_heights;
			double m_bathymetry_origin_x_m;
			double m_bathymetry_origin_y_m;
			double m_bathymetry_cell_m;
			int32_t m_bathymetry_width;
			int32_t m_bathymetry_height;

			// Lattice vertices per side (odd, in [3, 4097]). The surface has this many squared vertices; the triangle density is highest at the camera.
			int32_t m_vertices_per_side;

			// The surface stays inside this circle about the origin.
			double m_radius_m;

			// Half the vertical extent of the culling bounds, about z = 0. It must cover every water level plus the highest wave.
			double m_half_height_m;

			// Height of the domain floor. An opaque curtain hangs from the surface's edge down to this height, so the water meets the domain wall
			// without showing what lies beyond it.
			double m_floor_height_m;

			// The detail map baked by View3D_OceanDetailMapBake: OceanDetailMapSize x OceanDetailMapSize RGBA8 pixels, row-major. Required.
			uint32_t const* m_detail_map;

			// Optional replacements for the three forward pixel shaders (opaque, opaque reflection attributes, and alpha collect), for example to
			// supply the ground height for the shore foam (see pr/game-systems/ocean/ocean_forward_ps.hlsli). Null bytecode uses the default shader,
			// which has no shore. Either all three or none are given.
			ShaderByteCode m_forward_pixel[3];
		};
		static constexpr int OceanDetailMapSize = 512;

		// Per-frame state of an ocean.
		struct OceanFrame
		{
			double m_camera_x_m;
			double m_camera_y_m;
			double m_camera_z_m;
			double m_wave_time_s;        // The wave clock, wrapped by the caller into the spectrum's repeat period.
			double m_level_m;            // Still-water level.
			double m_wave_radius_m;      // Waves fade to flat water at this distance from the camera.
			double m_min_spacing_m;      // Vertex spacing at the camera.
			double m_warp_power;         // Growth of the vertex spacing with distance (1 is uniform).
			double m_breaking_ratio;     // Waves break at this height-to-depth ratio.
			double m_shore_sharpness;    // Crest sharpness that waves reach at the breaking height, in [0, WaterFieldMaxCrestSharpness]. See WaterFieldShoreSharpness.
			uint32_t m_colour_argb;      // Surface tint. Alpha is the opacity when viewed from directly above.
			uint32_t m_depth_effects;    // Non-zero to apply shoaling and breaking.
		};

		// The white foam drawn on an ocean, and the dark lip of the water at the waterline. All foam is one foam texture, revealed by a foam
		// amount: shore foam where the water laps the ground, crest foam on breaking wave crests, and the trails that crests leave behind as they
		// pass, which fade where they formed. Shore foam and the lip are drawn only within 'm_radius_m' of the camera. Every value is finite and
		// non-negative, and the shore foam depth, softness, crest threshold, tile size, lip height, radius, fade, crest range, and trail time
		// are positive. A zero strength turns that effect off.
		struct OceanFoam
		{
			double m_shore_foam_depth_m; // Vertical water depth (m) at which the shore foam fades out. Shallower water foams more, up to full cover at the waterline.
			double m_edge_softness;      // Blur of the foam's broken edge, as a fraction of the foam pattern's range. Small values give crisp patches.
			double m_crest_strength;     // Crest foam amount on the sharpest breaking crests. 0 turns crest foam and its trails off.
			double m_crest_threshold;    // Crest steepness at which crest foam starts: the summed steepest-face slopes of the waves near their crests.
			double m_tile_m;             // Size (m) of one repeat of the foam texture. It holds 5 of the foam's largest holes across.
			double m_bump_m;             // Height (m) of the foam's relief, which tilts its normal.
			double m_radius_m;           // Distance (m) from the camera beyond which there is no shore foam or lip.
			double m_fade_m;             // Distance (m) inside the radius over which the shore foam and lip fade out. A fade longer than the radius starts at the camera.
			double m_lip_height_m;       // Vertical water depth (m) over which the water darkens towards the ground, like a rounded lip in shadow.
			double m_lip_strength;       // Darkness of the lip at the waterline: 0 for none, and 1 for black.
			double m_crest_range;        // Crest steepness above the threshold at which the crest foam reaches the crest strength.
			double m_trail_time_s;       // Time (s) for a fully covered trail behind a crest to fade away.
			double m_crest_depth_m;      // Still-water depth (m) at which crest foam is gone. It fades in up to twice this depth, so shallow water near the shore shows only shore foam.
			double m_churn_mps;          // Rough speed (m/s) at which the foam pattern wanders in place.
		};

		// The result of an atmosphere function. On failure, View3D_AtmosphereLastError describes it.
		enum class EAtmosphereStatus :int32_t
		{
			Success = 0,
			InvalidArgument = 1,
			StepPending = 2,
			NoStepPending = 3,
			BufferTooSmall = 4,
			DeviceRemoved = 5,
			InternalError = 6,
			Timeout = 7,
		};

		// Boundary condition of one outside face of an atmosphere domain.
		enum class EAtmosphereBoundary :int32_t
		{
			Solid = 0,
			Open = 1,
		};

		// Indices of the six outside faces of an atmosphere domain, used by 'AtmosphereDesc::m_boundaries' and 'AtmosphereDesc::m_wall_drag'.
		enum class EAtmosphereSide :int32_t
		{
			XMin = 0,
			XMax = 1,
			YMin = 2,
			YMax = 3,
			ZMin = 4,
			ZMax = 5,
		};

		// A terrain-following GPU air solver and its optional tracer particles. Distances are metres, temperatures are kelvin, and rates are 1/s.
		// The domain has 'm_cell_count.x * m_cell_count.y' columns of 'm_cell_count.z' layers (each count above one, at most 32 layers) with square
		// columns of size 'm_dx' starting at 'm_origin'; the w components of 'm_cell_count' and 'm_origin' are ignored. Each column runs from its floor
		// to 'm_lid_z'. 'm_floor_heights' is null for a flat floor at 'm_origin.z', or one height per column in row-major order (x fastest); a floor at
		// or above the lid makes the column solid. 'm_active_columns' is null when every column is active, or one byte per column (non-zero = active).
		// The arrays are copied. 'm_boundaries' and 'm_wall_drag' are indexed by EAtmosphereSide; drag is a quadratic coefficient in [0, 1] and only
		// solid sides may have drag. The reference profile 'm_reference_temperature + m_lapse_rate * (z - m_origin.z)', limited below by
		// 'm_min_temperature', sets the initial air at rest. Moving air changes temperature by 'm_adiabatic_lapse_rate' (K/m, normally negative) per
		// metre that it rises; the air at rest is stable when 'm_lapse_rate' is greater than 'm_adiabatic_lapse_rate'.
		// 'm_tracer_count' particles follow the wind for diagnostics; zero disables them. Tracer heights are spread with the relative densities
		// 'm_tracer_ground_density' at the floor, 'm_tracer_break_density' at the column fraction 'm_tracer_break_height', and
		// 'm_tracer_upper_density' above it. The pressure and stability fields match 'game_systems::atmosphere::AtmosphereConfig', which documents them.
		struct AtmosphereDesc
		{
			int32_t m_cell_count[4];
			Vec4 m_origin;
			float m_dx, m_lid_z, m_first_layer_thickness;
			float const* m_floor_heights;
			uint8_t const* m_active_columns;
			EAtmosphereBoundary m_boundaries[6];
			float m_wall_drag[6];
			float m_reference_temperature, m_lapse_rate, m_adiabatic_lapse_rate, m_min_temperature;
			float m_gravity, m_floor_exchange_rate, m_lid_temperature, m_lid_relaxation_rate;
			int32_t m_pressure_vcycles, m_pressure_pre_smooth, m_pressure_post_smooth, m_pressure_coarse_smooth, m_open_edge_band;
			float m_vorticity_confinement, m_vertical_viscosity;
			int32_t m_tracer_count;
			uint32_t m_tracer_seed;
			float m_tracer_max_age, m_tracer_ground_density, m_tracer_break_density, m_tracer_upper_density, m_tracer_break_height;
		};
		static_assert(sizeof(AtmosphereDesc) == 200);

		// A sphere at 'm_centre' (m, w = 1) with 'm_radius' (m) that heats air at 'm_heating_rate' (K/s) and relaxes it towards
		// 'm_target_temperature' at 'm_relaxation_rate' (1/s).
		struct AtmosphereHeatSource
		{
			Vec4 m_centre;
			float m_radius;
			float m_heating_rate, m_target_temperature, m_relaxation_rate;
		};
		static_assert(sizeof(AtmosphereHeatSource) == 32);

		// Air outside one column: its horizontal wind (m/s) and its temperature relative to the reference profile (K). 'm_reserved' must be zero.
		struct AtmosphereOutsideAir
		{
			Vec2 m_wind;
			float m_temperature_offset;
			float m_reserved;
		};
		static_assert(sizeof(AtmosphereOutsideAir) == 16);

		// The air at one probe point after a step: velocity (m/s, w = 0) and temperature (K). 'm_inside' is 1 when the point is in the air of the
		// domain, otherwise 0 with zero velocity and temperature.
		struct AtmosphereProbeSample
		{
			Vec4 m_velocity;
			float m_temperature;
			int32_t m_inside;
		};
		static_assert(sizeof(AtmosphereProbeSample) == 24);

		// The inputs to one atmosphere step of 'm_dt' seconds; the heat sources are copied and apply to this step only. At most 64 heat sources.
		// The air is sampled at the 'm_probe_count' world-space probe points (m, w = 1; at most 64) once the step has updated it; read the samples with
		// View3D_AtmosphereProbesCopy. When 'm_surface_wind_height' is positive, the horizontal wind at that height (m) above the floor of every column
		// is also sampled once the step has updated it; read it with View3D_AtmosphereSurfaceWindCopy. Zero skips the surface wind.
		struct AtmosphereStepDesc
		{
			float m_dt;
			int32_t m_heat_source_count;
			AtmosphereHeatSource const* m_heat_sources;
			int32_t m_probe_count;
			float m_surface_wind_height;
			Vec4 const* m_probes;
		};
		static_assert(sizeof(AtmosphereStepDesc) == 32);

		// One atmosphere tracer particle: world position (m, w = 1), air temperature (K), age (s), the air speed that last moved it (m/s), and padding.
		// GPU tracer slots store particles in this 32-byte layout, which HLSL readers of a slot buffer use.
		struct AtmosphereTracerParticle
		{
			Vec4 m_position;
			float m_temperature, m_age, m_speed;
			float m_reserved;
		};
		static_assert(sizeof(AtmosphereTracerParticle) == 32);

		// A held GPU tracer slot. 'm_resource' is an owned ID3D12Resource COM reference that the caller releases exactly once with IUnknown::Release.
		// It is a buffer of 'm_count' AtmosphereTracerParticle records, 'm_stride' bytes apart, on the atmosphere's device.
		struct AtmosphereTracerSlot
		{
			void* m_resource;
			int32_t m_slot;
			uint32_t m_count;
			uint32_t m_stride;
			uint32_t m_reserved;
		};
		static_assert(sizeof(AtmosphereTracerSlot) == 24);

		// The air at one cell centre: velocity (m/s, w = 0, averaged from the cell faces) and temperature (K).
		struct AtmosphereCellState
		{
			Vec4 m_velocity;
			float m_temperature;
		};
		static_assert(sizeof(AtmosphereCellState) == 20);

		// Simple diagnostics of a completed atmosphere field.
		struct AtmosphereStats
		{
			float m_max_speed, m_max_divergence, m_rms_divergence, m_mean_top_temperature, m_peak_vertical_velocity;
			int32_t m_reserved;
		};
		static_assert(sizeof(AtmosphereStats) == 24);

		// The simulation shaders of a particle type. See pr/game-systems/particles/particle_kernels.hlsli. The bytecode must be a compiled compute shader
		// built on particle_kernels.hlsli, so it matches the system's root signature. Other bytecode is a caller error: the D3D12 debug layer reports it,
		// and otherwise type creation fails with an error naming the kernel.
		struct ParticleSimShaders
		{
			ShaderByteCode m_emit;      // Compute shader with the entry point 'ParticleEmitCS'.
			ShaderByteCode m_update;    // Compute shader with the entry point 'ParticleUpdateCS'.
		};

		// The drawing shaders of a particle batch, and the index list that its vertex shader draws for each render record. Optional per-vertex data,
		// such as a small mesh shared by every record, is copied to the GPU once when the batch is created and bound at VIEW3D_PROCEDURAL_BUFFER1_REGISTER.
		struct ParticleDrawShaders
		{
			ShaderByteCode m_vertex;    // Procedural forward vertex shader. It reads the records at VIEW3D_PROCEDURAL_BUFFER0_REGISTER, one per instance.
			uint32_t const* m_indices;  // Triangle list of logical vertex ids drawn for each record.
			int m_index_count;          // Number of 'm_indices'. A positive multiple of 3.
			int m_vertex_count;         // One more than the largest logical vertex id in 'm_indices'.
			EGeom m_geom;               // The surface outputs that 'm_vertex' generates. Must include Vert. Norm enables lighting, and Tex0 enables the batch texture.
			void const* m_vertex_data;  // Optional data for each of the 'm_vertex_count' logical vertices. Null for none.
			int m_vertex_stride;        // Bytes of 'm_vertex_data' per logical vertex. A multiple of 4 in [4, 2048] when 'm_vertex_data' is set, otherwise 0.
		};

		// How a particle batch blends with the scene.
		enum class EParticleBlend :int
		{
			// Alpha fragments sorted per pixel with the scene's other transparent geometry. Suits sparse particles that must sort exactly.
			Sorted = 0,

			// Weighted blended transparency (see ENuggetFlag::AlphaWeighted). Suits dense, soft, overlapping particles such as smoke and dust.
			Weighted = 1,

			// Opaque, depth-tested and depth-written. Suits solid particles such as chips and leaves.
			Opaque = 2,
		};

		// Fixed inputs of a particle batch. A batch is one View3D object that draws the render records of every particle type added to it with one
		// indirect draw. Types with different particle states share a batch when they write the same render record.
		struct ParticleBatchOptions
		{
			static constexpr int CurrentVersion = 1;

			int m_struct_size;
			int m_version;
			ParticleDrawShaders m_shaders;
			int m_capacity;              // Most records drawn at once. At least the sum of the capacities of the batch's types. In [1, 4194240].
			int m_record_stride;         // Bytes per render record (sizeof ParticleRecord). A multiple of 4 in [4, 2048].
			void const* m_constants;     // Optional constants seen by the vertex shader as its procedural constants. Null for none.
			size_t m_constants_size;     // Zero when 'm_constants' is null, otherwise a multiple of 4 in [4, ProceduralBinding::ConstantsSize].
			EParticleBlend m_blend;
			BBox m_bbox;                 // Bounds of every particle, in the object's space, for culling.
			Texture m_texture;           // Optional diffuse texture, sampled at the vertex shader's UVs when 'm_shaders.m_geom' includes Tex0. Null for none.
			Sampler m_sampler;           // Sampler of 'm_texture'. Null for the default.
		};

		// Fixed inputs of a particle type. A type is a pool of particles that share their state layout and simulation shaders. Its particles are
		// drawn by the batch it is added to.
		struct ParticleTypeOptions
		{
			static constexpr int CurrentVersion = 2;

			int m_struct_size;
			int m_version;
			ParticleSimShaders m_shaders;
			int m_batch;                 // The batch that draws the type's particles.
			int m_capacity;              // Most particles alive at once. In [1, 4194240].
			int m_max_emitters;          // Most emitters alive at once. In [1, 65535].
			int m_state_stride;          // Bytes per particle state (sizeof ParticleState). A multiple of 4 in [4, 2048].
			int m_record_stride;         // Bytes per render record (sizeof ParticleRecord). Equal to the batch's record stride.
			int m_params_stride;         // Bytes per emitter's parameters (sizeof EmitterParams). A multiple of 4 in [4, 2048].
			void const* m_constants;     // The type's constants (TypeConstants), seen by its kernels at b1.
			size_t m_constants_size;     // A multiple of 4 in [4, 65536].
		};

		// The system-owned settings of a particle emitter. The caller's parameters are passed separately.
		struct ParticleEmitter
		{
			float m_rate;     // Particles per simulated second. Finite and non-negative.
			uint32_t m_seed;  // Random seed; emitters with equal seeds and parameters make equal particles.
			BOOL m_active;    // Inactive emitters make no particles, and discard bursts requested while inactive.
		};
		static_assert(sizeof(ParticleEmitter) == 12);

		// The pool usage of a particle type after its last step. Counts of particles requested, admitted, and dropped, and of plane collisions, accumulate
		// from the last reset. See pr/game-systems/particles/particle_environment.hlsli for the collision counts.
		struct ParticleStats
		{
			uint32_t m_free;
			uint32_t m_live;
			uint32_t m_requested;
			uint32_t m_admitted;
			uint32_t m_dropped;
			uint32_t m_contacts;   // Plane contacts resolved.
			uint32_t m_overlaps;   // Particles pushed out of planes that they started inside.
			uint32_t m_exhausted;  // Particles stopped because their contact budget was spent.
		};
		static_assert(sizeof(ParticleStats) == 32);

		// The environment that stepped particle types move in, in the space of the particles' positions. See pr/game-systems/particles/particle_environment.hlsli.
		struct ParticleEnvironment
		{
			Vec4 m_gravity;                // xyz = Acceleration (m/s²) of every stepped particle. w unused. Finite.
			Vec4 m_wind;                   // xyz = Uniform wind velocity (m/s), added to the wind grid. w unused. Finite.
			int m_collision_iterations;    // Most plane contacts resolved per particle per step. In [0, 64].
		};
		static_assert(sizeof(ParticleEnvironment) == 36);

		// A static plane that stepped particles collide with. Particles are allowed in the half-space dot(n, p) >= d.
		struct ParticlePlane
		{
			Vec4 m_plane;         // xyz = Unit normal n, pointing into the allowed half-space. w = Offset d. Finite.
			float m_restitution;  // Fraction of the normal velocity kept after a contact. In [0, 1].
			float m_damping;      // Fraction of the tangential velocity removed by a contact. In [0, 1].
		};
		static_assert(sizeof(ParticlePlane) == 24);

		// The stock particle effects. See pr/game-systems/particles/particle_presets.h.
		enum class EParticlePreset :int
		{
			Fire = 0,       // Flickering flames that rise and shrink. Stepped sprites.
			Sparks = 1,     // Streaked sparks thrown up and falling under gravity. Stepped sprites.
			Smoke = 2,      // Slow grey puffs that rise, grow and drift with the wind. Stepped sprites.
			Dust = 3,       // A burst of dust that billows out, rises a little and settles. Stepped sprites.
			Rain = 4,       // Streaked drops that fall with the wind and die where they land. Stepped sprites.
			Snow = 5,       // Flakes that drift down with the wind and die where they land. Stepped sprites.
			Leaves = 6,     // Leaves that flutter and tumble down with the wind, and settle where they land. Debris.
			Birds = 7,      // A distant flock of birds circling, flapping and gliding. Birds.
			WoodChips = 8,  // A burst of tumbling wood chips, such as from chopping a tree. Debris.
			Marker = 9,     // Pulses of an expanding ring, such as a beacon. Markers.
			Spray = 10,     // Droplets thrown up from breaking wave crests. Needs a bound water surface. Spray.
			Froth = 11,     // Foam patches that form on breaking wave crests and ride the waves. Needs a bound water surface. Froth.
		};

		// A ready-made particle effect: the settings of one batch, one type and its emitters. Every pointer refers to static data.
		// Presets work in metres with z up, and their emitter parameters are placed at the origin: every emitter parameter structure starts with
		// a float4 whose xyz is the emitter's position, which the caller sets. Presets collide with nothing; the caller can set the collision set
		// of the types that have one (stepped sprites and debris).
		struct ParticlePreset
		{
			ParticleSimShaders m_sim;    // The type's simulation shaders.
			ParticleDrawShaders m_draw;  // The batch's drawing shaders, with the effect's mesh where it has one.
			EParticleBlend m_blend;      // How the batch blends with the scene.
			int m_record_stride;         // Bytes per render record, for the batch and the type.
			int m_state_stride;          // Bytes per particle state, for the type.
			int m_params_stride;         // Bytes of 'm_params'.
			void const* m_constants;     // The type's constants.
			size_t m_constants_size;     // Bytes of 'm_constants'.
			void const* m_params;        // The emitter parameters of the effect, at the origin.
			float m_rate;                // Suggested emitter rate (particles per second). Zero for effects that are bursts.
			int m_burst;                 // Suggested burst size. Zero for continuous effects.
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
		using StoreChangeCB = Callback<void(__stdcall *)(void* ctx, EStoreChangeInitiator initiator, EStoreChangeFlags flags, GUID const* ids, int count, BOOL before)>;
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

		// Supplies the bytes of an image named by an identifier held in a model file.
		// Model formats that reference textures by id rather than embedding them use this so several
		// models can share one copy of an image. An id must name the same image bytes for the lifetime of
		// the renderer, because a texture already loaded for an id is reused without calling the resolver.
		// Return null to report that the id is unknown, in which case the importer treats the id as a file
		// path instead. The returned bytes must stay valid until the model creation call that requested them returns.
		using ResolveTextureCB = Callback<uint8_t const*(__stdcall*)(void* ctx, char const* texture_id, size_t* size)>;

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
	
	// Initialise calls are reference counted and must be matched with Shutdown calls.
	// The first Initialise call binds View3D to the calling thread. Later Initialise calls, and all Shutdown calls, must be made on that thread until
	// the last Shutdown call; calls from other threads fail. 'global_error_cb' receives initialisation errors and, until the matching Shutdown call,
	// errors associated with the dll (see View3D_GlobalErrorCBSet).
	VIEW3D_API pr::view3d::DllHandle __stdcall View3D_Initialise(pr::view3d::ReportErrorCB global_error_cb);
	VIEW3D_API void __stdcall View3D_Shutdown(pr::view3d::DllHandle context);

	// Acquire an owned ID3D12Device COM reference for lifetime-safe interop with independent subsystems.
	// The caller must release the returned interface exactly once using IUnknown::Release.
	VIEW3D_API void* __stdcall View3D_DeviceLeaseAcquire(pr::view3d::DllHandle context);

	// Acquire an owned ID3D12Fence COM reference to the fence that the graphics queue signals at sync points. Other queues on the same device
	// can wait on it to order their work after graphics work. The caller must release the returned interface exactly once using IUnknown::Release.
	VIEW3D_API void* __stdcall View3D_GfxQueueFenceAcquire(pr::view3d::DllHandle context);

	// Signal the graphics queue fence after all graphics work submitted so far, including every frame already rendered, and return the fence
	// value that marks that point. Does not wait. The values increase with each call.
	VIEW3D_API uint64_t __stdcall View3D_GfxQueueSyncPointAdd(pr::view3d::DllHandle context);

	// Return the D3D12 device-removal reason, or S_OK while the device remains usable.
	VIEW3D_API HRESULT __stdcall View3D_DeviceRemovedReasonGet(pr::view3d::DllHandle context);

	// Return a bounded DRED report after device removal, or an empty view while the device remains usable.
	VIEW3D_API pr::view3d::StrView __stdcall View3D_DeviceRemovedReportGet(pr::view3d::DllHandle context);

	// This error callback is called for errors that are associated with the dll (rather than with a window).
	VIEW3D_API void __stdcall View3D_GlobalErrorCBSet(pr::view3d::ReportErrorCB error_cb, BOOL add);

	// Set the callback for progress events when script sources are loaded or updated
	VIEW3D_API void __stdcall View3D_ParsingProgressCBSet(pr::view3d::ParsingProgressCB progress_cb, BOOL add);
	
	// Set the callback that is called when the sources/object store is changed/reloaded
	VIEW3D_API void __stdcall View3D_StoreChangeCBSet(pr::view3d::StoreChangeCB store_change_cb, BOOL add);

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
	VIEW3D_API pr::view3d::StrView __stdcall View3D_SourceNameGet(GUID const& context_id);
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
	VIEW3D_API pr::view3d::StrView __stdcall View3D_WindowSettingsGet(pr::view3d::Window window);
	VIEW3D_API void __stdcall View3D_WindowSettingsSet(pr::view3d::Window window, char const* settings);

	// Query and enable/disable the per-window ray tracing render step.
	VIEW3D_API pr::view3d::RayTracingInfo __stdcall View3D_WindowRayTracingInfoGet(pr::view3d::Window window);
	VIEW3D_API BOOL __stdcall View3D_WindowRayTracingEnabledGet(pr::view3d::Window window);
	VIEW3D_API void __stdcall View3D_WindowRayTracingEnabledSet(pr::view3d::Window window, BOOL enable);
	VIEW3D_API pr::view3d::RayTracingProps __stdcall View3D_RayTracingPropertiesGet(pr::view3d::Window window);
	VIEW3D_API void __stdcall View3D_RayTracingPropertiesSet(pr::view3d::Window window, pr::view3d::RayTracingProps const& props);

	// Get/Set the side-by-side stereo settings. Stereo uses two eye views composed side by side into one output.
	VIEW3D_API pr::view3d::StereoProps __stdcall View3D_StereoGet(pr::view3d::Window window);
	VIEW3D_API void __stdcall View3D_StereoSet(pr::view3d::Window window, pr::view3d::StereoProps const& props);

	// Show or hide the stereo cursor. The cursor is drawn on the zero-disparity plane at the view-local position of 'ss_point' (in 'window' screen space).
	// Applications hide the OS cursor while stereo is enabled and forward mouse positions here.
	VIEW3D_API void __stdcall View3D_StereoCursorSet(pr::view3d::Window window, BOOL visible, pr::view3d::Vec2 ss_point);

	// Get/Set the per-window underwater post effect properties
	VIEW3D_API pr::view3d::UnderwaterProps __stdcall View3D_PostEffectUnderwaterGet(pr::view3d::Window window);
	VIEW3D_API BOOL __stdcall View3D_PostEffectUnderwaterSet(pr::view3d::Window window, pr::view3d::UnderwaterProps const& props);
	
	// Get/Set the per-window ambient occlusion properties
	VIEW3D_API pr::view3d::AmbientOcclusionProps __stdcall View3D_AmbientOcclusionGet(pr::view3d::Window window);
	VIEW3D_API BOOL __stdcall View3D_AmbientOcclusionSet(pr::view3d::Window window, pr::view3d::AmbientOcclusionProps const& props);

	// Get/Set the dimensions of the render target. Note: Not equal to window size for non-96 dpi screens!
	// In set, 'width' and 'height' must both be greater than zero. A zero-area window has no back buffer, so callers skip such resizes.
	VIEW3D_API SIZE __stdcall View3D_WindowBackBufferSizeGet(pr::view3d::Window window);
	VIEW3D_API void __stdcall View3D_WindowBackBufferSizeSet(pr::view3d::Window window, SIZE size, BOOL force_recreate);

	// Get/Set whether the swap chain owns an output in DXGI exclusive fullscreen mode
	VIEW3D_API BOOL __stdcall View3D_WindowFullScreenGet(pr::view3d::Window window);
	VIEW3D_API void __stdcall View3D_WindowFullScreenSet(pr::view3d::Window window, BOOL fullscreen);

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

	// Add/Remove a population to/from a window. The window keeps its own reference. Adding a population that is already in the window does nothing.
	VIEW3D_API void __stdcall View3D_WindowAddPopulation(pr::view3d::Window window, pr::view3d::Population population);
	VIEW3D_API void __stdcall View3D_WindowRemovePopulation(pr::view3d::Window window, pr::view3d::Population population);

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

	// Render the window and execute the frame on the GPU without showing it. The next frame renders over the same back buffer.
	// Use it to pay the first-use costs of new content (pipeline states, driver work) before the frames that show it.
	VIEW3D_API void __stdcall View3D_WindowRenderHidden(pr::view3d::Window window);

	// Start creating the pipeline states that drawing the window's objects would use, including hidden objects, on worker threads.
	// Returns the number of states requested, excluding those already created or requested. Poll View3D_WindowPipelineStats until 'm_pending' is zero
	// to find when they are ready. Returns -1 on error.
	VIEW3D_API int __stdcall View3D_WindowPrewarmPipelines(pr::view3d::Window window);

	// Get the counters of the pipeline state cache of the window's renderer, shared by all windows of the renderer
	VIEW3D_API BOOL __stdcall View3D_WindowPipelineStats(pr::view3d::Window window, pr::view3d::PipelineStats& stats);

	// Get the counts of the draws recorded by the window's latest frame
	VIEW3D_API BOOL __stdcall View3D_WindowDrawStats(pr::view3d::Window window, pr::view3d::DrawStats& stats);

	// Wait for any previous frames to complete rendering within the GPU
	VIEW3D_API void __stdcall View3D_WindowGSyncWait(pr::view3d::Window window);

	// Replace the swap chain buffers
	VIEW3D_API void __stdcall View3D_WindowCustomSwapChain(pr::view3d::Window window, int count, pr::view3d::Texture* targets);

	// Get the MSAA back buffer (render target + depth stencil)
	VIEW3D_API pr::view3d::BackBuffer __stdcall View3D_WindowRenderTargetGet(pr::view3d::Window window);

	// Borrow the last submitted final colour target (depth is null), including transparency and overlays, in PRESENT state.
	// Call WindowGSyncWait before reading; do not render, resize, replace the swap chain, or destroy the window while using it.
	VIEW3D_API pr::view3d::BackBuffer __stdcall View3D_WindowFrameOutputGet(pr::view3d::Window window);

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

	// Get/Set the output dither noise amplitude for a window, in 8-bit sRGB steps. Zero disables dithering.
	// Dithering hides colour banding in smooth gradients at the cost of fine, fixed noise.
	VIEW3D_API float __stdcall View3D_DitherAmountGet(pr::view3d::Window window);
	VIEW3D_API void __stdcall View3D_DitherAmountSet(pr::view3d::Window window, float amount);

	// Control animation
	VIEW3D_API void __stdcall View3D_WindowAnimControl(pr::view3d::Window window, pr::view3d::EAnimCommand command, double time);

	// Set the callback for animation events
	VIEW3D_API void __stdcall View3D_WindowAnimEventCBSet(pr::view3d::Window window, pr::view3d::AnimationCB anim_cb, BOOL add);

	// Get/Set the animation time
	VIEW3D_API BOOL __stdcall View3D_WindowAnimating(pr::view3d::Window window);
	VIEW3D_API double __stdcall View3D_WindowAnimTimeGet(pr::view3d::Window window);
	VIEW3D_API void __stdcall View3D_WindowAnimTimeSet(pr::view3d::Window window, double time_s);
	VIEW3D_API double __stdcall View3D_WindowAnimationDuration(pr::view3d::Window window);

	// Return the DPI of the monitor that 'window' is displayed on
	VIEW3D_API pr::view3d::Vec2 __stdcall View3D_WindowDpiScale(pr::view3d::Window window);

	// Return an auto-reset event that is signalled when the GPU has released the back buffer that the next render of 'window' draws into,
	// so starting that render will not block waiting for the GPU or the display. Each render clears and re-arms the event. Wait on it before
	// rendering to keep that wait outside the render call, e.g. in a message loop. The window owns the handle; do not close it.
	VIEW3D_API HANDLE __stdcall View3D_WindowFrameReadyEvent(pr::view3d::Window window);

	// Return the host time, in seconds, that the latest render of 'window' spent blocked waiting for the GPU to release its back buffer.
	// This is zero when the caller waited for the frame-ready event before rendering.
	VIEW3D_API double __stdcall View3D_WindowFrameWaitSeconds(pr::view3d::Window window);

	// Set the global environment map for the window
	VIEW3D_API void __stdcall View3D_WindowEnvMapSet(pr::view3d::Window window, pr::view3d::CubeMap env_map);

	// Render the window's objects, lights, and shadows into 'env_map', centred at 'position'. 'env_map' must be a cube map created by View3D_CubeMapCreate.
	// The capture replaces the cube's contents and orientation transform. It does not assign the cube to the window. Capture resources are cached on the window,
	// so repeated captures at the same face size are much cheaper than the first. Objects with ELdrFlags::EnvMapCaptureExclude are not captured.
	VIEW3D_API void __stdcall View3D_WindowEnvMapCapture(pr::view3d::Window window, pr::view3d::CubeMap env_map, pr::view3d::Vec4 position);

	// Size the window's time-sliced environment map probe with 'face_size' pixels per face edge, or release it with 0, and set whether it is enabled. Sizing creates
	// the probe's resources, so a probe sized while disabled can be enabled later without that cost. While enabled, the probe owns the window's environment map.
	// Each View3D_WindowEnvMapProbeUpdate renders one face, and every six updates complete a new cube that fades in over the previous one. Enabling restarts the
	// capture. Setting an environment map with View3D_WindowEnvMapSet disables the probe but keeps its resources.
	VIEW3D_API void __stdcall View3D_WindowEnvMapProbeSet(pr::view3d::Window window, int face_size, BOOL enabled);

	// Render the next face of the window's environment map probe. 'position' is sampled when the first face of each cube is rendered. Call once per frame before rendering.
	VIEW3D_API void __stdcall View3D_WindowEnvMapProbeUpdate(pr::view3d::Window window, pr::view3d::Vec4 position);

	// Get/Set the world-space bounds of the captured geometry that reflections correct for parallax. Within the bounds, reflections march along the
	// reflected ray to find the captured surface it meets; anything beyond them stays infinitely distant. An invalid box (the default) uses direction-only
	// lookups. Bounds must be finite. Changes affect cubes captured afterwards.
	VIEW3D_API pr::view3d::BBox __stdcall View3D_WindowEnvMapParallaxBoundsGet(pr::view3d::Window window);
	VIEW3D_API void __stdcall View3D_WindowEnvMapParallaxBoundsSet(pr::view3d::Window window, pr::view3d::BBox bounds);

	// Get/Set the world-space plane whose surfaces reflect the rendered scene, as (unit normal xyz, distance w along the normal). Shaders that opt in
	// reflect the opaque scene visible on screen and fall back to the environment map where it has no data. Zero (the default) disables it.
	VIEW3D_API pr::view3d::Vec4 __stdcall View3D_WindowReflectionPlaneGet(pr::view3d::Window window);
	VIEW3D_API void __stdcall View3D_WindowReflectionPlaneSet(pr::view3d::Window window, pr::view3d::Vec4 plane);

	// Get/Set the bounding radius below which populations are left out of the reflection's mirror view. The mirror view is drawn at low
	// resolution, where small objects add cost but little detail. Zero (the default) draws them all.
	VIEW3D_API float __stdcall View3D_WindowReflectionMinPopulationRadiusGet(pr::view3d::Window window);
	VIEW3D_API void __stdcall View3D_WindowReflectionMinPopulationRadiusSet(pr::view3d::Window window, float radius);

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

	// Enable / disable the native flight-camera controller. When enabled the window
	// captures Raw Input (keyboard + mouse + first connected gamepad) and drives the
	// scene camera in FPS / flight-sim style. Cursor visibility and clipping are the
	// host's responsibility - this API only controls the camera-update side.
	VIEW3D_API void __stdcall View3D_CameraFlightControl(pr::view3d::Window window, BOOL enable);
	VIEW3D_API BOOL __stdcall View3D_CameraFlightControlIsEnabled(pr::view3d::Window window);

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

	// Convert a point between 'window' screen space and normalised screen space.
	// With a multi-view layout (e.g. stereo), normalised screen space is local to the view under 'screen', and NSS points map into the first view.
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

	// Notes:
	//  - A window has a list of scene lights. Light 0 is conventionally the main light. New windows have one directional light.
	//  - At most 64 lights shade a frame, counting scene lights first, then light source objects. Lights that are off do not count.
	//  - Ambient light is a scene-wide colour, not a light.

	// The number of scene lights in 'window'
	VIEW3D_API int __stdcall View3D_LightCount(pr::view3d::Window window);

	// Get/Set the properties of the scene light at 'index'. 'index' must be in [0, View3D_LightCount).
	VIEW3D_API pr::view3d::Light __stdcall View3D_LightGet(pr::view3d::Window window, int index);
	VIEW3D_API void __stdcall View3D_LightSet(pr::view3d::Window window, int index, pr::view3d::Light const& light);

	// Add a scene light and return its index
	VIEW3D_API int __stdcall View3D_LightAdd(pr::view3d::Window window, pr::view3d::Light const& light);

	// Remove the scene light at 'index'. Later lights move down one index.
	VIEW3D_API void __stdcall View3D_LightRemove(pr::view3d::Window window, int index);

	// Get/Set the scene-wide ambient light colour
	VIEW3D_API pr::view3d::Colour __stdcall View3D_AmbientGet(pr::view3d::Window window);
	VIEW3D_API void __stdcall View3D_AmbientSet(pr::view3d::Window window, pr::view3d::Colour ambient);

	// Get/Set the scene-wide shadow settings. A light casts shadows when its 'm_cast_shadow' is greater than zero.
	// At most 'm_max_shadow_lights' lights cast shadows, chosen in light order, and all shadow views share one atlas.
	VIEW3D_API pr::view3d::ShadowSettings __stdcall View3D_ShadowSettingsGet(pr::view3d::Window window);
	VIEW3D_API void __stdcall View3D_ShadowSettingsSet(pr::view3d::Window window, pr::view3d::ShadowSettings const& settings);

	// Get/Set the shadow settings used when rendering environment map faces (View3D_WindowEnvMapCapture and the environment map probe).
	// Shadow cascades are fitted to each face's view, so every captured face renders its own shadow maps. Defaults to the default shadow settings.
	VIEW3D_API pr::view3d::ShadowSettings __stdcall View3D_EnvMapCaptureShadowsGet(pr::view3d::Window window);
	VIEW3D_API void __stdcall View3D_EnvMapCaptureShadowsSet(pr::view3d::Window window, pr::view3d::ShadowSettings const& settings);

	// Get/Set whether environment map faces (View3D_WindowEnvMapCapture and the environment map probe) render only objects in the 'Skybox' sort group.
	// A sky-only map has no scene geometry, so it never shows stale or low-resolution copies of nearby objects, and its faces are cheap to render. Defaults to false.
	VIEW3D_API BOOL __stdcall View3D_EnvMapCaptureSkyOnlyGet(pr::view3d::Window window);
	VIEW3D_API void __stdcall View3D_EnvMapCaptureSkyOnlySet(pr::view3d::Window window, BOOL sky_only);

	// Objects ********************************

	// Notes:
	// 'name' parameter on object get/set functions:
	//   If 'name' is null, then the state of the root object is returned
	//   If 'name' begins with '#' then the remainder of the name is treated as a regular expression

	// Create an object from provided buffers
	VIEW3D_API pr::view3d::Object __stdcall View3D_ObjectCreate(char const* name, pr::view3d::Colour colour, int vcount, int icount, int ncount, pr::view3d::Vertex const* verts, UINT16 const* indices, pr::view3d::Nugget const* nuggets, GUID const& context_id);

	// Create a 32-bit indexed object from supplied vertices, a procedural logical-ID domain, or a one-time GPU-generated canonical vertex buffer.
	// ProceduralVertexId objects use one physical placeholder vertex, while each U32 index is a logical SV_VertexID below options.m_vcount_logical.
	// GpuGeneratedBuffer uses vcount as the output count, requires verts == nullptr, and dispatches the supplied b0/u0 compute contract once.
	// Nugget vertex ranges always address the physical buffer: use [0,1) for the one-placeholder case, never the logical domain.
	// Procedural nugget geometry requires Vert and may declare Colr, Norm, and Tex0 generated by its shaders; Norm opts into stock lighting.
	// Generated-buffer compute shaders write the public View3DVertex HLSL layout; other inputs use one complete Vertex record.
	// Multiple nuggets share the logical domain and bounds but may partition the physical index buffer, for example [0,surface_icount) and [surface_icount,icount).
	VIEW3D_API pr::view3d::Object __stdcall View3D_ObjectCreateU32(char const* name, pr::view3d::Colour colour, int vcount, int icount, int ncount, pr::view3d::Vertex const* verts, UINT32 const* indices, pr::view3d::Nugget const* nuggets, pr::view3d::ObjectCreateOptions const& options, GUID const& context_id);

	// Re-run a compute vertex generator over the vertex buffer of an object created with EVertexSource::GpuGeneratedBuffer (the object's own model only).
	// Uses the same b0/u0 contract as creation. u0 holds the current vertices, so the shader may update records in place.
	// The work is submitted to the graphics queue without waiting, so later rendering and readbacks on that queue see the new vertices.
	VIEW3D_API void __stdcall View3D_ObjectGpuGenerate(pr::view3d::Object object, void const* compute_bytecode, size_t compute_bytecode_size, void const* constants, size_t constants_size, int thread_group_size_x);

	// Replace the ProceduralBinding::ConstantsSize constants of every procedural shader on the object's own nuggets. Later frames use the new values.
	// Shaders are shared by reference, so other objects that use the same shader handles also see the change.
	VIEW3D_API void __stdcall View3D_ObjectProceduralConstantsSet(pr::view3d::Object object, void const* constants, size_t constants_size);

	// Replace the buffer at 'slot' (an ID3D12Resource, or null to unbind) of every procedural shader on the object's own nuggets. 'slot' is in
	// [0, ProceduralBinding::BufferSlotCount). Later frames bind the new buffer at VIEW3D_PROCEDURAL_BUFFER<slot>_REGISTER. Slot 0 is the object's own
	// buffer, updated by View3D_ObjectProceduralBufferWrite and View3D_ObjectProceduralBufferDispatch; other slots typically share a buffer owned by
	// another object (see View3D_ObjectProceduralBufferGet). Binding the buffer that is already bound has no effect. The shaders take their own
	// references; the caller keeps its reference. The caller keeps the buffer contents valid until the frames that use them have finished on the graphics
	// queue (see View3D_GfxQueueSyncPointAdd). The buffer must rest in D3D12_RESOURCE_STATE_ALL_SHADER_RESOURCE whenever the graphics queue is not
	// updating it. Shaders are shared by reference, so other objects that use the same shader handles also see the change. A shader that declares a
	// slot must not be drawn while no buffer is bound there.
	VIEW3D_API void __stdcall View3D_ObjectProceduralBufferSet(pr::view3d::Object object, int slot, void* buffer);

	// Return the buffer at 'slot' of the first procedural shader on the object's own nuggets, or null if there is none. The ID3D12Resource pointer is
	// borrowed, not AddRef'd: it stays valid only while it remains bound, so pass it straight to View3D_ObjectProceduralBufferSet or AddRef it.
	VIEW3D_API void* __stdcall View3D_ObjectProceduralBufferGet(pr::view3d::Object object, int slot);

	// Replace the texture at 'slot' (or null to unbind) of every procedural shader with a forward pixel family on the object's own nuggets. 'slot' is in
	// [0, ProceduralBinding::TextureSlotCount). Later frames bind the texture at element 'slot' of the array at VIEW3D_PROCEDURAL_TEXTURE_REGISTER, where it
	// is sampled with the stock anisotropic wrapping sampler at VIEW3D_PROCEDURAL_SAMPLER_REGISTER. Unbound slots read zero. The shaders take their own
	// references; the caller keeps its reference. Throws if the object has no procedural shader with a forward pixel family. Shaders are shared by
	// reference, so other objects that use the same shader handles also see the change.
	VIEW3D_API void __stdcall View3D_ObjectProceduralTextureSet(pr::view3d::Object object, int slot, pr::view3d::Texture tex);

	// Overwrite 'data_size' bytes at byte 'offset' of each distinct slot 0 buffer bound to the procedural shaders on the object's own nuggets. The copy runs
	// on the graphics queue after the frames already submitted and before later frames, so no frame sees a partial write. 'offset' and 'data_size'
	// must be multiples of 4, with 'data_size' non-zero, inside every such buffer. Other queues must not use the buffers while this work is in flight.
	VIEW3D_API void __stdcall View3D_ObjectProceduralBufferWrite(pr::view3d::Object object, size_t offset, void const* data, size_t data_size);

	// Run a caller-compiled compute shader once over each distinct slot 0 buffer bound to the procedural shaders on the object's own nuggets. The shader sees
	// a copy of the 1 to 65536 'constants' bytes at b0 and the whole buffer as a RWByteAddressBuffer at u0. Like View3D_ObjectProceduralBufferWrite,
	// the dispatch runs on the graphics queue after the frames already submitted and before later frames. Each buffer must have been created with
	// D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS, and every group count must be positive. The pipeline for the most recent bytecode is cached per shader.
	VIEW3D_API void __stdcall View3D_ObjectProceduralBufferDispatch(pr::view3d::Object object, void const* cs_bytecode, size_t cs_bytecode_size, void const* constants, size_t constants_size, int groups_x, int groups_y, int groups_z);

	// Create a buffer of 'size' bytes on the renderer's device, for binding to procedural shaders with View3D_ObjectProceduralBufferSet. The first
	// 'init_size' bytes are copied from 'init_data', and the rest are unspecified. Sizes are multiples of 4, with 'size' non-zero and at most
	// ProceduralBinding::MaxBufferSize. Non-zero 'unordered_access' allows View3D_ObjectProceduralBufferDispatch. The buffer rests in
	// D3D12_RESOURCE_STATE_ALL_SHADER_RESOURCE. Returns an owned ID3D12Resource reference, which the caller releases exactly once with IUnknown::Release.
	VIEW3D_API void* __stdcall View3D_ProceduralBufferCreate(size_t size, void const* init_data, size_t init_size, BOOL unordered_access, char const* name);

	// Create an graphics object from ldr script, either a string or a file 
	VIEW3D_API pr::view3d::Object __stdcall View3D_ObjectCreateLdrW(wchar_t const* ldr_script, BOOL file, GUID const* context_id, pr::view3d::Includes const* includes);
	VIEW3D_API pr::view3d::Object __stdcall View3D_ObjectCreateLdrA(char const* ldr_script, BOOL file, GUID const* context_id, pr::view3d::Includes const* includes);

	// Create an graphics object from binary ldr script
	VIEW3D_API pr::view3d::Object __stdcall View3D_ObjectCreateLdrB(void const* binary, size_t size, GUID const* context_id);

	// Load a p3d model file as a view3d object. 'tex_resolver' may be empty when the model's textures are plain file paths.
	VIEW3D_API pr::view3d::Object __stdcall View3D_ObjectCreateP3DFile(char const* name, pr::view3d::Colour colour, char const* p3d_filepath, pr::view3d::ResolveTextureCB tex_resolver, GUID const* context_id);

	// Load a p3d model in memory as a view3d object. 'tex_resolver' may be empty when the model's textures are plain file paths.
	VIEW3D_API pr::view3d::Object __stdcall View3D_ObjectCreateP3DStream(char const* name, pr::view3d::Colour colour, size_t size, void const* p3d_data, pr::view3d::ResolveTextureCB tex_resolver, GUID const* context_id);

	// Create a background that shows 'cube_map' behind all scene geometry, centred on the camera at the far plane.
	// The sky retains the cube map and uses its orientation (CubeMapOptions::m_cube2w). Object transforms are ignored. Destroy using View3D_ObjectDelete.
	VIEW3D_API pr::view3d::Object __stdcall View3D_ObjectCreateSkybox(char const* name, pr::view3d::CubeMap cube_map, GUID const* context_id);

	// Create a Z-up GPU atmosphere with clouds and stars. See ProceduralSkySettings for the parameter contract.
	// Destroy using View3D_ObjectDelete. This object has no reflection cube map and ignores object transforms.
	VIEW3D_API pr::view3d::Object __stdcall View3D_ObjectCreateProceduralSky(char const* name, pr::view3d::ProceduralSkySettings const& settings, GUID const* context_id);

	// Update an atmosphere on its render owner thread. Returns FALSE on invalid parameters or a non-sky object, without changing the previous sky.
	VIEW3D_API BOOL __stdcall View3D_ObjectUpdateProceduralSky(pr::view3d::Object object, pr::view3d::ProceduralSkySettings const& settings);

	// Set the weather map that varies cloud cover across the sky, or null to use only the default cover. The sky retains the map.
	// Returns FALSE on a non-sky object, without changing the previous weather.
	VIEW3D_API BOOL __stdcall View3D_ObjectProceduralSkyWeatherSet(pr::view3d::Object object, pr::view3d::WeatherMap weather);

	// Retain and blend a source cube map (0=cubemap, 1=atmosphere). Direction transforms must be finite rotations.
	// The cubemap's own orientation is composed with world_to_background. Null background requires weight 1. FALSE leaves the sky unchanged.
	VIEW3D_API BOOL __stdcall View3D_ObjectBlendProceduralSky(pr::view3d::Object object, pr::view3d::CubeMap background, float weight, pr::view3d::Mat4x4 const& world_to_sky, pr::view3d::Mat4x4 const& world_to_background);

	// Place the sun, moon and stars for an observer at 'latitude' (degrees in [-90, 90], negative is south), 'day' days since midnight at the start of
	// 1 January (the fraction is the time of day), and 'moon_age' days since the last new moon. Any finite day and moon age are valid because both repeat.
	// See CelestialClock in pr/game-systems/sky/celestial_clock.h. Needs no View3D context.
	VIEW3D_API void __stdcall View3D_CelestialEvaluate(float latitude, double day, double moon_age, pr::view3d::CelestialState& state);

	// Create a 'width' x 'height' grid (each in [2,4096]) of cloud cover over the sky-frame XY area [area_min, area_max), initially clear.
	// Cover fades to the sky's default cover over the outer 10% of the area. Edits apply to the CPU copy; call View3D_WeatherMapUpload to show them.
	// Release using View3D_WeatherMapRelease. Returns null on failure.
	VIEW3D_API pr::view3d::WeatherMap __stdcall View3D_WeatherMapCreate(int width, int height, pr::view3d::Vec2 area_min, pr::view3d::Vec2 area_max);
	VIEW3D_API void __stdcall View3D_WeatherMapRelease(pr::view3d::WeatherMap weather);

	// Move the area the grid covers (e.g. to follow the camera). The grid contents are unchanged.
	VIEW3D_API void __stdcall View3D_WeatherMapAreaSet(pr::view3d::WeatherMap weather, pr::view3d::Vec2 area_min, pr::view3d::Vec2 area_max);

	// Brushes. Each blends cells toward 'cover' in [0,1]. A storm cell is solid inside 40% of 'radius' and fades to nothing at 'radius'.
	// A front covers the half-plane behind 'point', where 'travel_direction' points from the covered side to the clear side, with a soft edge 'width' wide.
	// Noise adds smooth variation in [-amplitude, +amplitude] with feature size 'scale', clamped to [0,1].
	VIEW3D_API void __stdcall View3D_WeatherMapFill(pr::view3d::WeatherMap weather, float cover);
	VIEW3D_API void __stdcall View3D_WeatherMapAddStormCell(pr::view3d::WeatherMap weather, pr::view3d::Vec2 centre, float radius, float cover);
	VIEW3D_API void __stdcall View3D_WeatherMapAddFront(pr::view3d::WeatherMap weather, pr::view3d::Vec2 point, pr::view3d::Vec2 travel_direction, float width, float cover);
	VIEW3D_API void __stdcall View3D_WeatherMapAddNoise(pr::view3d::WeatherMap weather, float scale, float amplitude, uint32_t seed);

	// Return the cover at 'position', faded to 'default_cover' toward the area edges, matching what the sky renders. Returns 'default_cover' on failure.
	VIEW3D_API float __stdcall View3D_WeatherMapCoverAt(pr::view3d::WeatherMap weather, pr::view3d::Vec2 position, float default_cover);

	// Copy the CPU grid to the GPU so later frames show the edits.
	VIEW3D_API void __stdcall View3D_WeatherMapUpload(pr::view3d::WeatherMap weather);

	// Ocean ******************************
	// An ocean is a wind-driven wave surface with foam, drawn as one procedural View3D object whose vertices are generated on the GPU. The ocean
	// handle keeps the fixed layout needed to pack each frame's constants, and its own copies of the wind sea and bathymetry for View3D_OceanHeight.
	// The object casts no shadow and has no ray-cast shader, so callers exclude it from hit tests. See pr/game-systems/ocean/ocean_surface.h.

	// Create an ocean and its View3D object. The options' arrays are copied, so they can be released afterwards. Returns null on failure, with
	// 'out_object' null. Destroy the ocean with View3D_OceanDestroy and the object with View3D_ObjectDelete; each outlives the other safely.
	VIEW3D_API pr::view3d::Ocean __stdcall View3D_OceanCreate(pr::view3d::OceanOptions const& options, char const* name, pr::view3d::Object* out_object);
	VIEW3D_API void __stdcall View3D_OceanDestroy(pr::view3d::Ocean ocean);

	// Build the triangle list shared by oceans with 'vertices_per_side' vertices per side, so View3D_OceanCreate does not build it. The list
	// depends on nothing else and is kept for the life of the process. Optional; any thread may call it, without a View3D context.
	VIEW3D_API BOOL __stdcall View3D_OceanLatticePrepare(int vertices_per_side);

	// Bake the ocean detail map, a tileable map of ripple slopes and foam, into 'out_pixels' (OceanDetailMapSize^2 RGBA8 pixels, row-major).
	// The map depends on nothing else and uses only the CPU, so it can be baked ahead of time on any thread, without a View3D context.
	VIEW3D_API BOOL __stdcall View3D_OceanDetailMapBake(uint32_t* out_pixels, int pixel_count);

	// Set the per-frame state of 'object', which must have been created with 'ocean'.
	VIEW3D_API BOOL __stdcall View3D_OceanUpdate(pr::view3d::Ocean ocean, pr::view3d::Object object, pr::view3d::OceanFrame const& frame);

	// Replace the wind-sea data of 'object' (created with 'ocean'), laid out by the grid the ocean was created with. The data is copied, and
	// later frames draw it; View3D_OceanHeight uses it from the next View3D_OceanUpdate.
	VIEW3D_API BOOL __stdcall View3D_OceanSetWindSea(pr::view3d::Ocean ocean, pr::view3d::Object object, pr::view3d::OceanWindSea const& wind_sea);

	// Non-zero 'ripples' draws the small-scale ripple normals on the surface; zero shows only the wave normals. Non-zero 'foam' draws the foam and
	// the dark lip at the waterline; zero draws neither and stops the foam trails, which start afresh when foam is turned back on. Both are on for a
	// new ocean. Takes effect from the next update.
	VIEW3D_API BOOL __stdcall View3D_OceanSetEffects(pr::view3d::Ocean ocean, BOOL ripples, BOOL foam);

	// Non-zero 'show' draws the ripple strength of the local wind as colour instead of the shaded water: blue for calm, green for a fresh breeze
	// (8 m/s), and red for twice its roughness or more. Off for a new ocean. Takes effect from the next update.
	VIEW3D_API BOOL __stdcall View3D_OceanShowRippleStrength(pr::view3d::Ocean ocean, BOOL show);

	// Set the foam of 'ocean'. Oceans start with a default foam. Takes effect from the next View3D_OceanUpdate. Foam is drawn only; it does not
	// change the surface height.
	VIEW3D_API BOOL __stdcall View3D_OceanSetFoam(pr::view3d::Ocean ocean, pr::view3d::OceanFoam const& foam);

	// Set the ground source that the ocean's pixel shaders read for the shore foam and the waterline lip of 'object' (created with 'ocean').
	// 'buffer' (an ID3D12Resource, typically borrowed from another object with View3D_ObjectProceduralBufferGet) is bound at procedural buffer
	// slot 1, and the 12 'params' are copied into the 'shore_source' constants (see OceanShoreHeight in pr/game-systems/ocean/ocean_forward_ps.hlsli).
	// A null 'buffer' binds the ocean's own buffer and zeroes the params. Oceans start with no ground source. Takes effect from the next update.
	VIEW3D_API BOOL __stdcall View3D_OceanShoreSourceSet(pr::view3d::Ocean ocean, pr::view3d::Object object, void* buffer, uint32_t const* params);

	// Return in 'out_height_m' the height of the drawn surface at the finite world position 'x_m', 'y_m', for the last frame passed to
	// View3D_OceanUpdate, which must have been called. The waves are evaluated on the CPU exactly as the vertex shader draws them.
	// Positions beyond the ocean's radius take the height at its edge.
	VIEW3D_API BOOL __stdcall View3D_OceanHeight(pr::view3d::Ocean ocean, double x_m, double y_m, double& out_height_m);

	// Particles ******************************
	// A particle system owns pools of GPU-resident particles. Each particle type is one pool. Each batch is one View3D object that draws the
	// particles of all of its types with one indirect draw, whose instance count the GPU writes. The CPU only uploads emitter changes; particles are
	// born, updated, and die on the GPU, in steps of simulated time, so they move correctly at any step rate and in slow motion.
	// See pr/game-systems/particles/particle_system.h.

	// Create an empty particle system on the renderer. Returns null on failure. Destroy it with View3D_ParticleSystemDestroy.
	VIEW3D_API pr::view3d::ParticleSystem __stdcall View3D_ParticleSystemCreate(char const* name);
	VIEW3D_API void __stdcall View3D_ParticleSystemDestroy(pr::view3d::ParticleSystem system);

	// Fill 'out_sim' and 'out_draw' with the shaders of the stock sprite type and its batch (see pr/game-systems/particles/particle_sprite.hlsli).
	// The data is static.
	VIEW3D_API BOOL __stdcall View3D_ParticleSpriteShaders(pr::view3d::ParticleSimShaders* out_sim, pr::view3d::ParticleDrawShaders* out_draw);

	// Fill 'out_draw' with the stock mesh batch's vertex shader and its outputs (see pr/game-systems/particles/particle_mesh.hlsli). The caller
	// supplies the mesh: its ParticleMeshVertex array as the vertex data, and its triangle list as the indices. The data is static.
	VIEW3D_API BOOL __stdcall View3D_ParticleMeshShaders(pr::view3d::ParticleDrawShaders* out_draw);

	// Fill 'out_sim' and 'out_draw' with the shaders of the stock stepped sprite type (see pr/game-systems/particles/particle_stepped_sprite.hlsli).
	// Its particles draw with the stock sprite batch. The data is static.
	VIEW3D_API BOOL __stdcall View3D_ParticleSteppedSpriteShaders(pr::view3d::ParticleSimShaders* out_sim, pr::view3d::ParticleDrawShaders* out_draw);

	// Fill 'out_preset' with the settings of a stock particle effect (see pr/game-systems/particles/particle_presets.h). The data is static.
	VIEW3D_API BOOL __stdcall View3D_ParticlePresetGet(pr::view3d::EParticlePreset preset, pr::view3d::ParticlePreset* out_preset);

	// Set the gravity, uniform wind and contact budget that stepped particle types move in, from the next step. The default is no gravity, no wind,
	// and 4 contacts.
	VIEW3D_API BOOL __stdcall View3D_ParticleEnvironmentSet(pr::view3d::ParticleSystem system, pr::view3d::ParticleEnvironment const& environment);

	// Set the wind grid, from the next step: 'cells_x' by 'cells_y' columns of horizontal wind velocity, row-major, with the minimum corner at 'origin'
	// in the XY plane and square cells of side 'cell_size'. The data is copied. Zero columns removes the grid.
	VIEW3D_API BOOL __stdcall View3D_ParticleWindGridSet(pr::view3d::ParticleSystem system, pr::view3d::Vec2 origin, float cell_size, int cells_x, int cells_y, pr::view3d::Vec2 const* wind);

	// Add a collision set of 'count' planes, returning its index in 'out_set'. Emitters of stepped types name the set that their particles collide with.
	// Indices of deleted sets are reused, so emitters must stop naming a set before it is deleted.
	VIEW3D_API BOOL __stdcall View3D_ParticleCollisionSetCreate(pr::view3d::ParticleSystem system, pr::view3d::ParticlePlane const* planes, int count, int* out_set);

	// Replace the planes of a collision set, from the next step.
	VIEW3D_API BOOL __stdcall View3D_ParticleCollisionSetUpdate(pr::view3d::ParticleSystem system, int set, pr::view3d::ParticlePlane const* planes, int count);

	// Remove a collision set. Particles that still name it collide with nothing.
	VIEW3D_API BOOL __stdcall View3D_ParticleCollisionSetDelete(pr::view3d::ParticleSystem system, int set);

	// Bind the water surface that particle types such as spray and froth sample (see pr/game-systems/particles/particle_water.hlsli), from the next step.
	// 'object' must be the View3D object of 'ocean'. Both must outlive the binding, and View3D_OceanUpdate must be called before each step so particles
	// see the frame being drawn. A null 'ocean' removes the binding, and 'object' is then ignored.
	VIEW3D_API BOOL __stdcall View3D_ParticleWaterSet(pr::view3d::ParticleSystem system, pr::view3d::Ocean ocean, pr::view3d::Object object);

	// Add a particle batch to 'system', returning its index in 'out_batch' and its View3D object in 'out_object'. The options' data is copied.
	// Delete the object with View3D_ObjectDelete. An object that outlives its system keeps drawing the particles of the system's last step.
	VIEW3D_API BOOL __stdcall View3D_ParticleBatchCreate(pr::view3d::ParticleSystem system, pr::view3d::ParticleBatchOptions const& options, char const* name, int* out_batch, pr::view3d::Object* out_object);

	// Add a particle type to 'system', drawn by the batch 'options.m_batch', returning its index in 'out_type'. The options' data is copied.
	VIEW3D_API BOOL __stdcall View3D_ParticleTypeCreate(pr::view3d::ParticleSystem system, pr::view3d::ParticleTypeOptions const& options, char const* name, int* out_type);

	// Add an emitter of particles of 'type', returning its id in 'out_emitter'. 'params' is 'params_size' bytes, the type's parameter stride.
	VIEW3D_API BOOL __stdcall View3D_ParticleEmitterCreate(pr::view3d::ParticleSystem system, int type, pr::view3d::ParticleEmitter const& emitter, void const* params, size_t params_size, uint64_t* out_emitter);

	// Replace the settings and parameters of an emitter, from the next step. Its particles already alive are unchanged.
	VIEW3D_API BOOL __stdcall View3D_ParticleEmitterUpdate(pr::view3d::ParticleSystem system, uint64_t emitter, pr::view3d::ParticleEmitter const& settings, void const* params, size_t params_size);

	// Ask an active emitter for 'count' extra particles, born at the end of the next step.
	VIEW3D_API BOOL __stdcall View3D_ParticleEmitterBurst(pr::view3d::ParticleSystem system, uint64_t emitter, uint32_t count);

	// Remove an emitter. Its particles already alive live out their lives.
	VIEW3D_API BOOL __stdcall View3D_ParticleEmitterDelete(pr::view3d::ParticleSystem system, uint64_t emitter);

	// Advance every particle by 'dt' simulated seconds (finite, non-negative). Zero does nothing, so a paused game draws its particles unchanged.
	VIEW3D_API BOOL __stdcall View3D_ParticleSystemStep(pr::view3d::ParticleSystem system, double dt);

	// Remove every particle and restart every emitter's emission. Emitters are kept.
	VIEW3D_API BOOL __stdcall View3D_ParticleSystemReset(pr::view3d::ParticleSystem system);

	// Read the pool usage of 'type'. Waits for the GPU, so use it for tests and diagnostics only.
	VIEW3D_API BOOL __stdcall View3D_ParticleStats(pr::view3d::ParticleSystem system, int type, pr::view3d::ParticleStats* out_stats);

	// Atmospheres ******************************
	// A terrain-following GPU air solver. Each atmosphere runs on the caller's D3D12 device with its own compute queue, and steps independently of
	// any other simulation. Atmosphere functions do not use the renderer and are not serialised by View3D; callers must not call functions on one
	// atmosphere from more than one thread at a time. Every function returns an EAtmosphereStatus; on failure, View3D_AtmosphereLastError describes
	// the most recent failure on the calling thread.

	// Copy the description of the calling thread's most recent atmosphere failure, as a null-terminated UTF-8 string, into 'buffer'.
	// 'required' receives the size including the terminator; the text is truncated when 'capacity' is smaller. 'buffer' may be null when 'capacity' is zero.
	VIEW3D_API pr::view3d::EAtmosphereStatus __stdcall View3D_AtmosphereLastError(char* buffer, uint32_t capacity, uint32_t* required);

	// Create an atmosphere on the ID3D12Device 'd3d12_device'. 'shader_cache_dir' is the directory of compiled kernels, or null to compile without
	// caching. Creation compiles the solver kernels and blocks until the air is at rest on the reference profile.
	VIEW3D_API pr::view3d::EAtmosphereStatus __stdcall View3D_AtmosphereCreate(void* d3d12_device, pr::view3d::AtmosphereDesc const* desc, wchar_t const* shader_cache_dir, pr::view3d::Atmosphere* atmosphere);

	// Destroy an atmosphere after waiting up to 'timeout_ms' for its GPU work. Fails with Timeout, leaving the handle valid, when the GPU work does not finish in time.
	VIEW3D_API pr::view3d::EAtmosphereStatus __stdcall View3D_AtmosphereDestroy(pr::view3d::Atmosphere atmosphere, uint32_t timeout_ms);

	// Submit one step, and tracer advection when tracers exist, without waiting for the GPU. Fails with StepPending while a step is in flight.
	VIEW3D_API pr::view3d::EAtmosphereStatus __stdcall View3D_AtmosphereBeginStep(pr::view3d::Atmosphere atmosphere, pr::view3d::AtmosphereStepDesc const* step);

	// Finish the step in flight if the GPU has completed it, without waiting. 'idle' is set to 1 when no step is in flight after the call.
	VIEW3D_API pr::view3d::EAtmosphereStatus __stdcall View3D_AtmospherePollStep(pr::view3d::Atmosphere atmosphere, int32_t* idle);

	// Copy the probe samples of the last finished step, in the order of its probes. 'required' is the sample count, which is zero before the
	// first finished step and after a step without probes. Does not wait, and may be called while a step is in flight.
	VIEW3D_API pr::view3d::EAtmosphereStatus __stdcall View3D_AtmosphereProbesCopy(pr::view3d::Atmosphere atmosphere, pr::view3d::AtmosphereProbeSample* samples, uint32_t capacity, uint32_t* required);

	// Copy the surface wind of the last finished step: the horizontal wind (m/s) at the requested height above each column floor, one entry per column
	// in the order of AtmosphereDesc's columns. Inactive columns report their outside air's wind; solid columns report zero. 'required' is the column
	// count, or zero before the first finished step and after a step that did not sample the surface wind. Does not wait, and may be called while
	// a step is in flight.
	VIEW3D_API pr::view3d::EAtmosphereStatus __stdcall View3D_AtmosphereSurfaceWindCopy(pr::view3d::Atmosphere atmosphere, pr::view3d::Vec2* wind, uint32_t capacity, uint32_t* required);

	// Wait for and finish the step in flight. Fails with NoStepPending when there is none.
	VIEW3D_API pr::view3d::EAtmosphereStatus __stdcall View3D_AtmosphereCompleteStep(pr::view3d::Atmosphere atmosphere);

	// Change the column floor heights (one per column, as in AtmosphereDesc) and remap the air in changed columns. Blocks; requires no step in flight.
	VIEW3D_API pr::view3d::EAtmosphereStatus __stdcall View3D_AtmosphereFloorsSet(pr::view3d::Atmosphere atmosphere, float const* floor_heights, int32_t count);

	// Set the floor temperature under each column (K, one per column in the floor-height order). The lowest layer relaxes toward it at the
	// descriptor's 'm_floor_exchange_rate'. Initially the reference temperature at each column's floor. The values are copied and used from the next step.
	VIEW3D_API pr::view3d::EAtmosphereStatus __stdcall View3D_AtmosphereFloorTemperaturesSet(pr::view3d::Atmosphere atmosphere, float const* floor_temperatures, int32_t count);

	// Set the air outside the open faces (one entry per column in row-major floor-height order). Initially calm air at the reference temperature.
	// The values are copied and used from the next step.
	VIEW3D_API pr::view3d::EAtmosphereStatus __stdcall View3D_AtmosphereOutsideAirSet(pr::view3d::Atmosphere atmosphere, pr::view3d::AtmosphereOutsideAir const* outside_air, int32_t count);

	// Read the tracer particles from the last finished step, or from creation, back from the GPU. 'required' is the particle count.
	// Blocks; requires no step in flight. For display, prefer reading the GPU buffers directly with View3D_AtmosphereTracersAcquire.
	VIEW3D_API pr::view3d::EAtmosphereStatus __stdcall View3D_AtmosphereTracersCopy(pr::view3d::Atmosphere atmosphere, pr::view3d::AtmosphereTracerParticle* particles, uint32_t capacity, uint32_t* required);

	// Hold the GPU buffer of the tracer particles from the last finished step, or from creation, so that later steps do not write it. Tracers
	// advance through a small ring of buffers, so at most two slots can be held at once; a step fails while it has no free buffer to write.
	// The buffer may be read on any queue of the atmosphere's device until the matching View3D_AtmosphereTracersRelease. Fails if the atmosphere has no tracers.
	VIEW3D_API pr::view3d::EAtmosphereStatus __stdcall View3D_AtmosphereTracersAcquire(pr::view3d::Atmosphere atmosphere, pr::view3d::AtmosphereTracerSlot* slot);

	// Release one hold on 'slot'. 'fence' is an ID3D12Fence that reaches 'value' when the caller's GPU reads of the slot have finished, or null when
	// they already have. The step that next writes the slot waits for the fence on the GPU, so this call never blocks.
	VIEW3D_API pr::view3d::EAtmosphereStatus __stdcall View3D_AtmosphereTracersRelease(pr::view3d::Atmosphere atmosphere, int32_t slot, void* fence, uint64_t value);

	// Read the whole field back from the GPU and copy the cell-centre states, packed by layer, then row, then column (x fastest).
	// 'required' is the cell count and 'stats' is optional. Blocks; requires no step in flight.
	VIEW3D_API pr::view3d::EAtmosphereStatus __stdcall View3D_AtmosphereCellStatesCopy(pr::view3d::Atmosphere atmosphere, pr::view3d::AtmosphereCellState* cells, uint32_t capacity, uint32_t* required, pr::view3d::AtmosphereStats* stats);

	// Compile every atmosphere kernel into the shader cache directory 'shader_cache_dir' using 'thread_count' worker threads, so later atmosphere
	// creation does not invoke the shader compiler. Blocks until done. 'progress' is optional; it is called on the calling thread with the number of
	// completed and total kernels, and returning zero cancels the remaining work (the call still returns Success).
	VIEW3D_API pr::view3d::EAtmosphereStatus __stdcall View3D_AtmosphereShaderPrecompile(wchar_t const* shader_cache_dir, int thread_count, int(__stdcall* progress)(void* ctx, int completed, int total), void* ctx);

	// Populations ******************************
	// A population draws many placements of one prototype with GPU instancing. Instances live in GPU buffers and are grouped into caller-owned blocks.
	// Population LOD materials must be opaque. Forward and shadow vertex shaders are either the stock shaders, or procedural shaders written for populations:
	// the LOD object is created with EVertexSource::ProceduralVertexId, and its shaders place each vertex at its instance using the population bindings
	// (see population.hlsli). Shadow views that see a population with a procedural shadow vertex shader are rendered every frame, because its output can
	// change without any change to the population. LOD nuggets with ENuggetFlag::ShadowCastExclude are not drawn into shadow views, and a population
	// whose LOD nuggets all exclude shadow casting is not a shadow caster.

	// Create a prototype from 'lod_count' LODs, ordered from most to least detailed. Each LOD draws the model of its object. The prototype keeps its own
	// reference to the models, so the objects may be deleted afterwards. LOD thresholds must strictly decrease, and transition bands must not overlap.
	// 'draw_distance' is the maximum camera distance of a drawn instance's bounding sphere; zero is unlimited. 'fade_in' and 'fade_out' are camera
	// distance bands, measured to the instance origin, in which instances fade in or out with a dither. Instances nearer than a used fade-in band or
	// further than a used fade-out band are not drawn. Equal fade-in and fade-out bands on two prototypes hand over instances with the same origin
	// without gaps or double coverage. The fade-in band must end before the fade-out band starts, and when either is used, the instance origin must lie
	// inside the bounding sphere of the LOD models. Release using View3D_PopulationPrototypeRelease. Returns null on failure.
	VIEW3D_API pr::view3d::PopulationPrototype __stdcall View3D_PopulationPrototypeCreate(pr::view3d::PopulationLod const* lods, int lod_count, float draw_distance, pr::view3d::PopulationFade fade_in, pr::view3d::PopulationFade fade_out);
	VIEW3D_API void __stdcall View3D_PopulationPrototypeRelease(pr::view3d::PopulationPrototype prototype);

	// Create a population of the 'prototype_count' prototypes in 'prototypes', with fixed capacities. Each instance selects its prototype by index, and the
	// whole population is classified once per view, so grouping many prototypes in one population is cheaper than one population per prototype.
	// GPU resources are created here, so later updates do not allocate. 'payload_size' is the size in bytes of an optional caller payload buffer for
	// custom shaders; zero means no payload, and it must be a multiple of 4. 'sort_group' orders the population's draws relative to the scene's other
	// draws, as for objects; drawing near opaque populations before the terrain and sky lets depth testing skip the pixels that they hide.
	// Release using View3D_PopulationRelease. Returns null on failure.
	VIEW3D_API pr::view3d::Population __stdcall View3D_PopulationCreate(pr::view3d::PopulationPrototype const* prototypes, int prototype_count, int instance_capacity, int block_capacity, int payload_size, pr::view3d::ESortGroup sort_group);
	VIEW3D_API void __stdcall View3D_PopulationRelease(pr::view3d::Population population);

	// Define block 'index' as the instance slots [first, first + capacity), of which the first 'live_count' are drawn. 'bbox' is a world-space bound that
	// must contain the bounding sphere of every live instance. Blocks outside the view are skipped without testing their instances.
	VIEW3D_API void __stdcall View3D_PopulationBlockSet(pr::view3d::Population population, int index, pr::view3d::BBox bbox, int first, int capacity, int live_count);

	// Stop drawing block 'index'.
	VIEW3D_API void __stdcall View3D_PopulationBlockRemove(pr::view3d::Population population, int index);

	// Draw a fraction 'coverage', in [0,1], of the pixels of every instance with a screen-door dither, in every view. One draws instances in full, and
	// zero draws nothing. Use it to dissolve a whole population in or out. Within LOD transitions and fade bands, each entry keeps 'coverage' of its own
	// pixels, so complementary entries still never cover a pixel twice.
	VIEW3D_API void __stdcall View3D_PopulationCoverageSet(pr::view3d::Population population, float coverage);

	// Return the screen-door dither word that an ordinary object at 'origin' (its object-to-world translation) uses to draw 'coverage' of its pixels.
	// An instance at the same translation with the 'Faded' instance flag and the same coverage draws the complementary pixels when 'complement' is
	// false for the object and set on the instance (see 'pr/view3d-12/shaders/population.hlsli'). Use it to cross-fade an object with an instance.
	VIEW3D_API uint32_t __stdcall View3D_PopulationDitherWord(pr::view3d::Vec4 origin, float coverage, BOOL complement);

	// Copy 'count' instance records into the slots [first, first + count). The data is copied during the call and shown from the next frame.
	VIEW3D_API void __stdcall View3D_PopulationInstancesUpdate(pr::view3d::Population population, int first, pr::view3d::PopulationInstance const* instances, int count);

	// Copy 'size' bytes into the payload buffer at byte offset 'ofs'. The data is copied during the call and shown from the next frame.
	VIEW3D_API void __stdcall View3D_PopulationPayloadUpdate(pr::view3d::Population population, int64_t ofs, void const* data, int64_t size);

	// Generate population content on the GPU. Queues one dispatch of the caller-compiled compute shader 'cs_bytecode', which sees a copy of the 1 to 65536
	// 'constants' bytes and writes instances, block live counts, and payload through the producer bindings in 'pr/view3d-12/shaders/population.hlsli'.
	// Dispatches and other updates run at the start of the next frame in the order they were queued, before any render step reads the population, and
	// nothing is read back to the CPU. Blocks still come from View3D_PopulationBlockSet, whose bounds must contain every instance the producer makes live.
	// Writes outside the buffers are discarded. The pipeline for the most recent bytecode is cached per population.
	VIEW3D_API void __stdcall View3D_PopulationProduce(pr::view3d::Population population, void const* cs_bytecode, size_t cs_bytecode_size, void const* constants, size_t constants_size, int groups_x, int groups_y, int groups_z);

	// Read the population's diagnostics. Classification counters are copied back from the GPU asynchronously, so they describe the latest classification
	// of each view that the GPU has already completed, typically a few frames old. The call never waits for the GPU. Returns false on failure.
	VIEW3D_API BOOL __stdcall View3D_PopulationStats(pr::view3d::Population population, pr::view3d::PopulationStats& stats);

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

	// Return true if 'object' is a decendant of 'parent'
	VIEW3D_API BOOL __stdcall View3D_ObjectIsDescendant(pr::view3d::Object object, pr::view3d::Object parent);

	// Return a child object of 'object'
	VIEW3D_API pr::view3d::Object __stdcall View3D_ObjectGetChildByName(pr::view3d::Object object, char const* name);
	VIEW3D_API pr::view3d::Object __stdcall View3D_ObjectGetChildByIndex(pr::view3d::Object object, int index);

	// Return the number of child objects of 'object'
	VIEW3D_API int __stdcall View3D_ObjectChildCount (pr::view3d::Object object);

	// Enumerate the child objects of 'object'. (Not recursive)
	VIEW3D_API void __stdcall View3D_ObjectEnumChildren (pr::view3d::Object object, pr::view3d::EnumObjectsCB enum_objects_cb);

	// Get/Set the name of 'object'
	VIEW3D_API pr::view3d::StrView __stdcall View3D_ObjectNameGet(pr::view3d::Object object);
	VIEW3D_API void __stdcall View3D_ObjectNameSet(pr::view3d::Object object, char const* name);

	// Get the type of 'object'
	VIEW3D_API pr::view3d::StrView __stdcall View3D_ObjectTypeGet(pr::view3d::Object object);

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
	VIEW3D_API double __stdcall View3D_ObjectAnimTimeGet(pr::view3d::Object object, char const* name);
	VIEW3D_API void __stdcall View3D_ObjectAnimTimeSet(pr::view3d::Object object, double time_s, char const* name);

	// Drive the skinned model of 'object' (or children matching 'name') from caller-published bone poses.
	// Rendering interpolates between the two most recent snapshots using the object's animation time, and holds the pose outside that interval.
	VIEW3D_API void __stdcall View3D_ObjectPoseSnapshotsCreate(pr::view3d::Object object, char const* name);

	// Publish a bone pose at 'time_s' (in object animation time, increasing with each call). 'bones' are bone-to-parent transforms without scale, in skeleton bone order.
	VIEW3D_API void __stdcall View3D_ObjectPoseSnapshotPublish(pr::view3d::Object object, pr::view3d::Mat4x4 const* bones, int bone_count, double time_s, char const* name);

	// Convert 'bones', bone-to-parent transforms in skeleton bone order, to object-space skinning matrices for the skeleton of the first skinned model in
	// 'object' (or children matching 'name'). 'palette_out' receives 'bone_count' matrices, which is the layout of a population skin-pose palette.
	VIEW3D_API void __stdcall View3D_ObjectSkinningPalette(pr::view3d::Object object, pr::view3d::Mat4x4 const* bones, int bone_count, pr::view3d::Mat4x4* palette_out, char const* name);

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
	// In the 'Nugget' functions, 'index' selects a nugget of each matched model in the order its nuggets were created.
	VIEW3D_API pr::view3d::ENuggetFlag __stdcall View3D_ObjectNuggetFlagsGet(pr::view3d::Object object, char const* name, int index);
	VIEW3D_API void __stdcall View3D_ObjectNuggetFlagsSet(pr::view3d::Object object, pr::view3d::ENuggetFlag flags, BOOL state, char const* name, int index);

	// Get/Set the tint colour for a nugget within the model of an object or its children. (See LdrObject::Apply)
	VIEW3D_API pr::view3d::Colour __stdcall View3D_ObjectNuggetTintGet(pr::view3d::Object object, char const* name, int index);
	VIEW3D_API void __stdcall View3D_ObjectNuggetTintSet(pr::view3d::Object object, pr::view3d::Colour colour, char const* name, int index);

	// Return one representative preset as ordinary caller-editable procedural surface parameters.
	VIEW3D_API pr::view3d::ProceduralSurface __stdcall View3D_ProceduralSurfacePreset(pr::view3d::EProceduralSurfacePreset preset);

	// Get, set, or clear the PBR procedural surface on a nugget. The getter returns false when no procedural surface is assigned.
	VIEW3D_API BOOL __stdcall View3D_ObjectNuggetProceduralSurfaceGet(pr::view3d::Object object, pr::view3d::ProceduralSurface& surface, char const* name, int index);
	VIEW3D_API void __stdcall View3D_ObjectNuggetProceduralSurfaceSet(pr::view3d::Object object, pr::view3d::ProceduralSurface const& surface, char const* name, int index);
	VIEW3D_API void __stdcall View3D_ObjectNuggetProceduralSurfaceClear(pr::view3d::Object object, char const* name, int index);

	// Set or remove the normal map of a nugget, promoting the nugget to a PBR material if needed. The map is sampled with the nugget's first
	// texture coordinate, and its red and green channels hold X and Y encoded as (n + 1) / 2, scaled by 'scale'. Tangent maps perturb the vertex
	// normal using a frame derived from the UVs. Model maps hold model-space normals with Z rebuilt as non-negative, so they replace the vertex
	// normal and suit height fields facing model +Z; the nugget must still have vertex normals. Set 'tex' and 'sam' to null to remove the map.
	VIEW3D_API void __stdcall View3D_ObjectNuggetNormalMapSet(pr::view3d::Object object, pr::view3d::Texture tex, pr::view3d::Sampler sam, pr::view3d::ENormalMapSpace space, float scale, char const* name, int index);

	// Set or remove the normal map of an object's instance material override, with the same map encoding as View3D_ObjectNuggetNormalMapSet.
	// Only this object changes; the shared model and other instances of it keep their materials. Without an existing override, the override
	// starts as a PBR copy of nugget 'index's material. The override replaces the material of every nugget of the object's model, and population
	// prototypes created from the object use it too.
	VIEW3D_API void __stdcall View3D_ObjectMaterialOverrideNormalMapSet(pr::view3d::Object object, pr::view3d::Texture tex, pr::view3d::Sampler sam, pr::view3d::ENormalMapSpace space, float scale, char const* name, int index);

	// Set or remove the tint mask of a nugget, promoting the nugget to a PBR material if needed. The mask is sampled with the nugget's first
	// texture coordinate. Its red channel is a linear weight that limits where the tint colours the surface: 1 = the base colour is multiplied
	// by the object and population instance tint colour; 0 = the base colour is kept. The tint alpha still applies everywhere. The nugget tint
	// (the material base colour) is not masked. Without a mask, the tint colours the whole surface. Set 'tex' and 'sam' to null to remove the mask.
	VIEW3D_API void __stdcall View3D_ObjectNuggetTintMaskSet(pr::view3d::Object object, pr::view3d::Texture tex, pr::view3d::Sampler sam, char const* name, int index);

	// Detail normals tilt a simple material's normals with world-projected layers of a tileable slope map. The map's red and green channels hold
	// the height slope along u and v, encoded as (slope + 1) / 2, and the blue channel holds (slope_u² + slope_v²) / 2, so mip filtering keeps
	// the roughness that minified slopes lose. Set 'tex' and 'sam' to null to remove detail normals. Layers are kept when the map
	// changes; a new assignment starts with no layers. Throws if the nugget material is not a simple material.
	VIEW3D_API void __stdcall View3D_ObjectNuggetDetailNormalsSet(pr::view3d::Object object, pr::view3d::Texture tex, pr::view3d::Sampler sam, char const* name, int index);

	// Draw nugget 'index' of 'object' with one indirect draw whose arguments the GPU writes at 'offset' bytes into 'buffer' (an ID3D12Resource),
	// or restore ordinary draws when 'buffer' is null. The record is a D3D12_DRAW_INDEXED_ARGUMENTS for nuggets with indices, otherwise a
	// D3D12_DRAW_ARGUMENTS. The caller writes it and keeps the buffer in the INDIRECT_ARGUMENT | NON_PIXEL_SHADER_RESOURCE state whenever the object
	// is drawn. 'bounds' (an ID3D12Resource) holds one float4 per instance index in [0, capacity): the object-space centre and radius of a sphere
	// containing that instance. The caller keeps it in the NON_PIXEL_SHADER_RESOURCE state whenever the object is drawn. 'capacity' is in
	// [1, 65535*64], and the record's instance count must not exceed it. Each view draws only the instances whose spheres it can see, as an unordered compacted list (see
	// pr/view3d-12/shaders/indirect_instances.hlsli). Such nuggets are drawn only by the forward render steps: they cast no shadows and are not ray
	// cast. The object keeps references to both buffers.
	VIEW3D_API void __stdcall View3D_ObjectNuggetDrawArgsSet(pr::view3d::Object object, void* buffer, int64_t offset, void* bounds, int capacity, int index);

	// Replace the detail-normal layers of every nugget of 'object' (not its children) that has detail normals. Layers are shared by the
	// material, so this is cheap enough to call every frame. 'count' must be in [0, 4]. 'base_slope_variance' (non-negative) is the mean square
	// slope of roughness finer than every layer; it blurs and weakens the environment reflection everywhere on the surface. Throws if the object
	// has no detail normals.
	VIEW3D_API void __stdcall View3D_ObjectDetailNormalLayersSet(pr::view3d::Object object, pr::view3d::DetailNormalLayer const* layers, int count, float base_slope_variance);

	// A detail strength map varies an object's detail-normal layers over its surface. Each texel holds (x, y, z, w): x scales the height of every
	// layer and y adds to the base slope variance, both finite and non-negative; (z, w) = (cos a, sin a) is a unit vector that turns the layers'
	// pattern by angle 'a' from their u direction toward their v direction, about the texel's centre. The texture coordinates of a world position
	// 'p' are u = dot(p.xyz, row_u.xyz) + row_u.w and v = dot(p.xyz, row_v.xyz) + row_v.w, where [0,1] spans the grid. Each fragment filters the
	// texels bilinearly, including the angle vectors, and positions outside the grid use the nearest edge texels. A new map is
	// (1, 0, 1, 0) everywhere with the identity XY projection. Release using View3D_DetailStrengthMapRelease. Returns null on failure.
	VIEW3D_API pr::view3d::DetailStrengthMap __stdcall View3D_DetailStrengthMapCreate(int width, int height);
	VIEW3D_API void __stdcall View3D_DetailStrengthMapRelease(pr::view3d::DetailStrengthMap map);

	// Set the world-space projection of the map. Both rows must be finite and their xyz parts must not be parallel.
	VIEW3D_API void __stdcall View3D_DetailStrengthMapProjectionSet(pr::view3d::DetailStrengthMap map, pr::view3d::Vec4 row_u, pr::view3d::Vec4 row_v);

	// Copy 'count' (= width * height) row-major texels to the GPU, visible from the next render.
	VIEW3D_API void __stdcall View3D_DetailStrengthMapUpload(pr::view3d::DetailStrengthMap map, pr::view3d::Vec4 const* texels, int count);

	// Show the height strength as colour, from blue at 0 through green at 1 to red at 2 or more, instead of the shaded surface.
	VIEW3D_API void __stdcall View3D_DetailStrengthMapVisualiseSet(pr::view3d::DetailStrengthMap map, BOOL visualise);

	// Use 'map' for every nugget of 'object' (not its children) that has detail normals. Null removes the map. Throws if the object has no
	// detail normals.
	VIEW3D_API void __stdcall View3D_ObjectDetailStrengthMapSet(pr::view3d::Object object, pr::view3d::DetailStrengthMap map);

	// Override stock-shader surface RGB after material/vertex/texture colour and before lighting. Packed RGB is an sRGB target, decoded at
	// shader upload; packed alpha is the linear UNORM8 blend weight (A/255), not opacity. Surface alpha and sorting remain unchanged.
	// Alpha zero (the default) disables the override. Custom pixel shaders must apply the colour_blend constant themselves.
	// Name follows LdrObject::Apply; getters return the first packed value/decoded weight, or zero if absent.
	VIEW3D_API pr::view3d::Colour __stdcall View3D_ObjectColourBlendGet(pr::view3d::Object object, char const* name);
	VIEW3D_API void __stdcall View3D_ObjectColourBlendSet(pr::view3d::Object object, pr::view3d::Colour blend, char const* name);

	// Get/Set the screen-door dither word of stock-shader draws of 'object'. Zero (the default) draws every pixel. Build words with
	// 'View3D_PopulationDitherWord'. Name follows LdrObject::Apply; the getter returns the first match, or zero if absent.
	VIEW3D_API uint32_t __stdcall View3D_ObjectDitherGet(pr::view3d::Object object, char const* name);
	VIEW3D_API void __stdcall View3D_ObjectDitherSet(pr::view3d::Object object, uint32_t dither, char const* name);

	// Get/Set an instance-owned multiplier of material emissive colour in the stock PBR and ray-traced shaders. One (the default) keeps the
	// authored emission and zero turns it off, so copies of one model can glow independently. Custom pixel shaders must apply the
	// emissive_scale constant themselves. Name follows LdrObject::Apply; the getter returns the first match, or one if absent.
	VIEW3D_API float __stdcall View3D_ObjectEmissiveScaleGet(pr::view3d::Object object, char const* name);
	VIEW3D_API void __stdcall View3D_ObjectEmissiveScaleSet(pr::view3d::Object object, float scale, char const* name);

	// Get/Set the soft depth of 'object': the view depth, in world units, over which its transparent fragments fade to zero alpha as they
	// approach the opaque scene behind them, so they do not show hard lines where they cut through opaque surfaces. Zero (the default) keeps
	// hard intersections. Applies to the stock transparent pixel shaders and to procedural families built with the stock entry-point macros.
	// 'depth' must be non-negative. Name follows LdrObject::Apply; the getter returns the first match, or zero if absent.
	VIEW3D_API float __stdcall View3D_ObjectSoftDepthGet(pr::view3d::Object object, char const* name);
	VIEW3D_API void __stdcall View3D_ObjectSoftDepthSet(pr::view3d::Object object, float depth, char const* name);

	// Resource batches ******************************

	// Begin a resource batch on the calling thread. Until the matching 'View3D_ResourceBatchEnd', resource creation calls made on this thread
	// (textures, samplers, shaders, objects, buffers, ...) submit their GPU work without waiting for it to finish, and share one command list.
	// 'View3D_ResourceBatchEnd' then waits once for all of it. Created resources can be used immediately. Batches nest; every begin needs one end.
	// Only one thread can have a batch open at a time.
	VIEW3D_API void __stdcall View3D_ResourceBatchBegin();
	VIEW3D_API void __stdcall View3D_ResourceBatchEnd();

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

	// Create an uninitialised RGBA8 sRGB cube map with 'face_size' pixels per face edge and a full mip chain. Use as a target for View3D_WindowEnvMapCapture.
	VIEW3D_API pr::view3d::CubeMap __stdcall View3D_CubeMapCreate(int face_size);

	// Create a texture sampler
	VIEW3D_API pr::view3d::Sampler __stdcall View3D_SamplerCreate(pr::view3d::SamplerOptions const& options);

	// Create one of the stock samplers
	VIEW3D_API pr::view3d::Sampler __stdcall View3D_SamplerCreateStock(pr::view3d::EStockSampler stock_sampler);

	// Create a procedural shader for the selected hardware stage. This is not an arbitrary shader binding API; other stages report unsupported.
	// Vertex: 'm_bytecode' is a procedural vertex shader for Forward, RayCast, or ShadowMap, used only on ProceduralVertexId objects. A ForwardRender
	//   vertex shader may also supply a forward pixel family.
	// Pixel: 'm_bytecode' must be null, and 'm_procedural' must be ForwardRender with a complete forward pixel family. The stock vertex shader is kept,
	//   so the shader may be used on objects with any vertex source.
	// The renderer copies bytecode and exactly 1024 constant bytes before returning. Stages not supplied keep their stock shaders.
	// Invalid or unsupported descriptors report through the public error callback and return null.
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

	// Replace the 'width' x 'height' pixel rectangle at ('x', 'y') in mip 0 of the 2D texture 'tex', then regenerate its lower mips.
	// 'data' holds tightly packed rows in the texture's pixel format, and 'data_size' must equal the size of the rectangle. The rectangle must lie
	// inside the texture. The update is submitted to the graphics queue, so frames rendered after this call see the new pixels.
	VIEW3D_API void __stdcall View3D_TextureUpdateRegion(pr::view3d::Texture tex, int x, int y, int width, int height, void const* data, size_t data_size);

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
	VIEW3D_API pr::view3d::Mat4x4 __stdcall View3D_ParseLdrTransform(char const* ldr_script, BOOL* is_affine);

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

	// Mark CPU and command-list GPU regions without requiring callers to link the optional PIX runtime.
	VIEW3D_API void __stdcall View3D_PixBeginCpuEvent(unsigned int colour, char const* name);
	VIEW3D_API void __stdcall View3D_PixEndCpuEvent();
	VIEW3D_API void __stdcall View3D_PixBeginGpuEvent(ID3D12GraphicsCommandList* command_list, unsigned int colour, char const* name);
	VIEW3D_API void __stdcall View3D_PixEndGpuEvent(ID3D12GraphicsCommandList* command_list);

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
			void operator()(WeatherMap p) noexcept { if (p) View3D_WeatherMapRelease(p); }
			void operator()(PopulationPrototype p) noexcept { if (p) View3D_PopulationPrototypeRelease(p); }
			void operator()(Population p) noexcept { if (p) View3D_PopulationRelease(p); }
			void operator()(DetailStrengthMap p) noexcept { if (p) View3D_DetailStrengthMapRelease(p); }
			void operator()(Window p) noexcept { if (p) View3D_WindowDestroy(p); }
		};
	}

	using ObjectPtr  = std::unique_ptr<rdr12::ldraw::LdrObject, impl::Deleter>;
	using GizmoPtr   = std::unique_ptr<rdr12::ldraw::LdrGizmo, impl::Deleter>;
	using TexturePtr = std::unique_ptr<rdr12::Texture2D, impl::Deleter>;
	using CubeMapPtr = std::unique_ptr<rdr12::TextureCube, impl::Deleter>;
	using SamplerPtr = std::unique_ptr<rdr12::Sampler, impl::Deleter>;
	using ShaderPtr  = std::unique_ptr<rdr12::Shader, impl::Deleter>;
	using WeatherMapPtr = std::unique_ptr<game_systems::WeatherMap, impl::Deleter>;
	using PopulationPrototypePtr = std::unique_ptr<rdr12::PopulationPrototype, impl::Deleter>;
	using PopulationPtr = std::unique_ptr<rdr12::Population, impl::Deleter>;
	using DetailStrengthMapPtr = std::unique_ptr<rdr12::materials::DetailStrengthMap, impl::Deleter>;
	using WindowPtr  = std::unique_ptr<rdr12::V3dWindow, impl::Deleter>;
}
