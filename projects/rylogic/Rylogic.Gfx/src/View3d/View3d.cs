//#define PR_VIEW3D_CREATE_STACKTRACE
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.Linq;
using System.Runtime.InteropServices;
using System.Threading;
using Rylogic.Common;
using Rylogic.Extn;
using Rylogic.Interop.Win32;
using Rylogic.Maths;
using Rylogic.Script;
using Rylogic.Utility;
using HContext = System.IntPtr;
using HCubeMap = System.IntPtr;
using HGizmo = System.IntPtr;
using HMODULE = System.IntPtr;
using HObject = System.IntPtr;
using HSampler = System.IntPtr;
using HTexture = System.IntPtr;
using HShader = System.IntPtr;
using HWindow = System.IntPtr;
using HWND = System.IntPtr;

namespace Rylogic.Gfx
{
	/// <summary>.NET wrapper for View3D.dll</summary>
	public sealed partial class View3d : IDisposable
	{
		// Notes:
		// - Each process should create a single View3d instance that is an isolated context.
		// - Ldr objects are created and owned by the context.

		#region Enumerations
		public enum EResult : int
		{
			Success,
			Failed,
		}
		public enum EFillMode : int
		{
			Default = 0,
			Points = 1,
			Wireframe = 2, // D3D11_FILL_WIREFRAME
			Solid = 3, // D3D11_FILL_SOLID
			SolidWire = 4,
		}
		public enum ECullMode : int
		{
			Default = 0,
			None = 1, // D3D11_CULL_NONE
			Front = 2, // D3D11_CULL_FRONT
			Back = 3, // D3D11_CULL_BACK
		}
		[Flags] public enum EGeom : int
		{
			Unknown = 0,
			Vert = 1 << 0,
			Colr = 1 << 1,
			Norm = 1 << 2,
			Tex0 = 1 << 3,
		}
		public enum ETopo : int
		{
			Undefined = 0,
			PointList,
			LineList,
			LineStrip,
			TriList,
			TriStrip,
			LineListAdj,
			LineStripAdj,
			TriListAdj,
			TriStripAdj,
		}
		[Flags] public enum ENuggetFlag : int
		{
			None = 0,

			/// <summary>Exclude this nugget when rendering a model</summary>
			Hidden = 1 << 0,

			/// <summary>Set if the geometry data for the nugget contains alpha colours</summary>
			GeometryHasAlpha = 1 << 1,

			/// <summary>Set if the tint colour contains alpha</summary>
			TintHasAlpha = 1 << 2,

			/// <summary>Set if the diffuse texture contains alpha (and we want alpha blending, not just thresholding)</summary>
			TexDiffuseHasAlpha = 1 << 3,

			/// <summary>Excluded from shadow map render steps</summary>
			ShadowCastExclude = 1 << 4,

			/// <summary>
			/// Can overlap with other nuggets.
			/// Set this flag to true if you want to add a nugget that overlaps the range
			/// of an existing nugget. For simple models, overlapping nugget ranges is
			/// usually an error, but in advanced cases it isn't.</summary>
			RangesCanOverlap = 1 << 5,
		}
		public enum ERenderStep :int
		{
			Invalid = 0,
			ForwardRender,
			GBuffer,
			DSLighting,
			ShadowMap,
			RayCast,
		}
		public enum EStockTexture : int
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
		}
		public enum EStockSampler : int
		{
			Invalid = 0,
			PointClamp,
			PointWrap,
			LinearClamp,
			LinearWrap,
			AnisotropicClamp,
			AnisotropicWrap,
		}
		public enum EStockShader : int
		{
			// Forward rendering shaders
			StandardVS = 0,
			StandardPS = 0,

			// Radial fade params:
			//  *Type {Spherical|Cylindrical}
			//  *Radius {min,max}
			//  *Centre {x,y,z} (optional, defaults to camera position)
			//  *Absolute (optional, default false) - True if 'radius' is absolute, false if 'radius' should be scaled by the focus distance
			RadialFadePS,

			// Point sprite params: *PointSize {w,h} *Depth {true|false}
			PointSpritesGS,

			// Thick line params: *LineWidth {width}
			ThickLineListGS,

			// Thick line params: *LineWidth {width}
			ThickLineStripGS,

			// Arrow params: *Size {size}
			ArrowHeadGS,
		}
		public enum ELight : int
		{
			Ambient,
			Directional,
			Point,
			Spot
		}
		public enum EAnimCommand : int
		{
			Reset, // Reset the 'time' value
			Play,  // Run continuously using 'time' as the step size, or real time if 'time' == 0
			Stop,  // Stop at the current time.
			Step,  // Step by 'time' (can be positive or negative)
		}
		public enum EGizmoMode :int
		{
			Translate,
			Rotate,
			Scale,
		}
		public enum EGizmoState : int // ELdrGizmoEvent 
		{
			StartManip,
			Moving,
			Commit,
			Revert,
		}
		[Flags] public enum EStockObject : int
		{
			None = 0,
			FocusPoint = 1 << 0,
			OriginPoint = 1 << 1,
			SelectionBox = 1 << 2,
			_flags_enum = 0,
		};
		[Flags] public enum ENavOp : int
		{
			None = 0,
			Translate = 1 << 0,
			Rotate = 1 << 1,
			Zoom = 1 << 2,
		}
		[Flags] public enum ECameraLockMask : int
		{
			None = 0,
			TransX = 1 << 0,
			TransY = 1 << 1,
			TransZ = 1 << 2,
			RotX = 1 << 3,
			RotY = 1 << 4,
			RotZ = 1 << 5,
			Zoom = 1 << 6,
			CameraRelative = 1 << 7,
			Translation = TransX | TransY | TransZ,
			Rotation = RotX | RotY | RotZ,
			All = (1 << 7) - 1, // Not including camera relative
		}
		[Flags] public enum EClipPlanes : int
		{
			None = 0,
			Near = 1 << 0,
			Far = 1 << 1,
			CameraRelative = 1 << 2,
			Both = Near | Far,
			_flags_enum = 0,
		}
		public enum EColourOp : int
		{
			Overwrite,
			Add,
			Subtract,
			Multiply,
			Lerp,
		}
		[Flags] public enum ELdrFlags : int
		{
			None = 0,

			/// <summary>The object is hidden</summary>
			Hidden = 1 << 0,

			/// <summary>The object is filled in wireframe mode</summary>
			Wireframe = 1 << 1,

			/// <summary>Render the object without testing against the depth buffer</summary>
			NoZTest = 1 << 2,

			/// <summary>Render the object without effecting the depth buffer</summary>
			NoZWrite = 1 << 3,

			/// <summary>The object has normals shown</summary>
			Normals = 1 << 4,

			/// <summary>The object to world transform is not an affine transform</summary>
			NonAffine = 1 << 5,

			/// <summary>Set when an object is selected. The meaning of 'selected' is up to the application</summary>
			Selected = 1 << 8,

			/// <summary>Doesn't contribute to the bounding box on an object.</summary>
			BBoxExclude = 1 << 9,

			/// <summary>Should not be included when determining the bounds of a scene.</summary>
			SceneBoundsExclude = 1 << 10,

			/// <summary>Ignored for hit test ray casts</summary>
			HitTestExclude = 1 << 11,

			/// <summary>Doesn't cast a shadow</summary>
			ShadowCastExclude = 1 << 12,
		}
		[Flags] public enum EBBoxFlags
		{
			None = 0,

			/// <summary>Include child objects when calculating the bounding box</summary>
			IncludeChildren = 1 << 0,
		}
		[Flags] public enum EUpdateObject :int // Flags for partial update of a model
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
		}
		public enum ESortGroup : int
		{
			Min = 0,               // The minimum sort group value
			PreOpaques = 63,       // 
			Default = 64,          // Make opaques the middle group
			Skybox,                // Sky-box after opaques
			PostOpaques,           // 
			PreAlpha = Default + 16, // Last group before the alpha groups
			AlphaBack,             // 
			AlphaFront,            // 
			PostAlpha,             // First group after the alpha groups
			Max = 127,             // The maximum sort group value
		}
		public enum ESceneBounds : int
		{
			All,
			Selected,
			Visible,
		}
		public enum ESourcesChangedReason : int
		{
			NewData,
			Reload,
			Removal,
		}
		public enum ESceneChanged : int
		{
			ObjectsAdded,
			ObjectsRemoved,
			GizmoAdded,
			GizmoRemoved,
		}
		[Flags] public enum ESnapMode : int
		{
			NoSnap = 0,
			Verts = 1 << 0,
			Edges = 1 << 1,
			Faces = 1 << 2,
			Perspective = 1 << 8,
			All = Faces | Edges | Verts,
			AllPerspective = All | Perspective,
		}
		public enum ESnapType : int
		{
			NoSnap,
			Vert,
			EdgeMiddle,
			FaceCentre,
			Edge,
			Face,
		}
		public enum HitTestRayId : int
		{
			None,
		}
		public enum EStreamingState : int
		{
			Disconnected,
			Listening,
			Connected,
		}
		[Flags] public enum ESettings : int
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
			Scene_FilllMode        = Scene | 1 << 2,
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
		}
		#endregion
		#region D3D Enumerations
		public enum EFormat : uint
		{
			DXGI_FORMAT_UNKNOWN = 0,
			DXGI_FORMAT_R32G32B32A32_TYPELESS = 1,
			DXGI_FORMAT_R32G32B32A32_FLOAT = 2,
			DXGI_FORMAT_R32G32B32A32_UINT = 3,
			DXGI_FORMAT_R32G32B32A32_SINT = 4,
			DXGI_FORMAT_R32G32B32_TYPELESS = 5,
			DXGI_FORMAT_R32G32B32_FLOAT = 6,
			DXGI_FORMAT_R32G32B32_UINT = 7,
			DXGI_FORMAT_R32G32B32_SINT = 8,
			DXGI_FORMAT_R16G16B16A16_TYPELESS = 9,
			DXGI_FORMAT_R16G16B16A16_FLOAT = 10,
			DXGI_FORMAT_R16G16B16A16_UNORM = 11,
			DXGI_FORMAT_R16G16B16A16_UINT = 12,
			DXGI_FORMAT_R16G16B16A16_SNORM = 13,
			DXGI_FORMAT_R16G16B16A16_SINT = 14,
			DXGI_FORMAT_R32G32_TYPELESS = 15,
			DXGI_FORMAT_R32G32_FLOAT = 16,
			DXGI_FORMAT_R32G32_UINT = 17,
			DXGI_FORMAT_R32G32_SINT = 18,
			DXGI_FORMAT_R32G8X24_TYPELESS = 19,
			DXGI_FORMAT_D32_FLOAT_S8X24_UINT = 20,
			DXGI_FORMAT_R32_FLOAT_X8X24_TYPELESS = 21,
			DXGI_FORMAT_X32_TYPELESS_G8X24_UINT = 22,
			DXGI_FORMAT_R10G10B10A2_TYPELESS = 23,
			DXGI_FORMAT_R10G10B10A2_UNORM = 24,
			DXGI_FORMAT_R10G10B10A2_UINT = 25,
			DXGI_FORMAT_R11G11B10_FLOAT = 26,
			DXGI_FORMAT_R8G8B8A8_TYPELESS = 27,
			DXGI_FORMAT_R8G8B8A8_UNORM = 28,
			DXGI_FORMAT_R8G8B8A8_UNORM_SRGB = 29,
			DXGI_FORMAT_R8G8B8A8_UINT = 30,
			DXGI_FORMAT_R8G8B8A8_SNORM = 31,
			DXGI_FORMAT_R8G8B8A8_SINT = 32,
			DXGI_FORMAT_R16G16_TYPELESS = 33,
			DXGI_FORMAT_R16G16_FLOAT = 34,
			DXGI_FORMAT_R16G16_UNORM = 35,
			DXGI_FORMAT_R16G16_UINT = 36,
			DXGI_FORMAT_R16G16_SNORM = 37,
			DXGI_FORMAT_R16G16_SINT = 38,
			DXGI_FORMAT_R32_TYPELESS = 39,
			DXGI_FORMAT_D32_FLOAT = 40,
			DXGI_FORMAT_R32_FLOAT = 41,
			DXGI_FORMAT_R32_UINT = 42,
			DXGI_FORMAT_R32_SINT = 43,
			DXGI_FORMAT_R24G8_TYPELESS = 44,
			DXGI_FORMAT_D24_UNORM_S8_UINT = 45,
			DXGI_FORMAT_R24_UNORM_X8_TYPELESS = 46,
			DXGI_FORMAT_X24_TYPELESS_G8_UINT = 47,
			DXGI_FORMAT_R8G8_TYPELESS = 48,
			DXGI_FORMAT_R8G8_UNORM = 49,
			DXGI_FORMAT_R8G8_UINT = 50,
			DXGI_FORMAT_R8G8_SNORM = 51,
			DXGI_FORMAT_R8G8_SINT = 52,
			DXGI_FORMAT_R16_TYPELESS = 53,
			DXGI_FORMAT_R16_FLOAT = 54,
			DXGI_FORMAT_D16_UNORM = 55,
			DXGI_FORMAT_R16_UNORM = 56,
			DXGI_FORMAT_R16_UINT = 57,
			DXGI_FORMAT_R16_SNORM = 58,
			DXGI_FORMAT_R16_SINT = 59,
			DXGI_FORMAT_R8_TYPELESS = 60,
			DXGI_FORMAT_R8_UNORM = 61,
			DXGI_FORMAT_R8_UINT = 62,
			DXGI_FORMAT_R8_SNORM = 63,
			DXGI_FORMAT_R8_SINT = 64,
			DXGI_FORMAT_A8_UNORM = 65,
			DXGI_FORMAT_R1_UNORM = 66,
			DXGI_FORMAT_R9G9B9E5_SHAREDEXP = 67,
			DXGI_FORMAT_R8G8_B8G8_UNORM = 68,
			DXGI_FORMAT_G8R8_G8B8_UNORM = 69,
			DXGI_FORMAT_BC1_TYPELESS = 70,
			DXGI_FORMAT_BC1_UNORM = 71,
			DXGI_FORMAT_BC1_UNORM_SRGB = 72,
			DXGI_FORMAT_BC2_TYPELESS = 73,
			DXGI_FORMAT_BC2_UNORM = 74,
			DXGI_FORMAT_BC2_UNORM_SRGB = 75,
			DXGI_FORMAT_BC3_TYPELESS = 76,
			DXGI_FORMAT_BC3_UNORM = 77,
			DXGI_FORMAT_BC3_UNORM_SRGB = 78,
			DXGI_FORMAT_BC4_TYPELESS = 79,
			DXGI_FORMAT_BC4_UNORM = 80,
			DXGI_FORMAT_BC4_SNORM = 81,
			DXGI_FORMAT_BC5_TYPELESS = 82,
			DXGI_FORMAT_BC5_UNORM = 83,
			DXGI_FORMAT_BC5_SNORM = 84,
			DXGI_FORMAT_B5G6R5_UNORM = 85,
			DXGI_FORMAT_B5G5R5A1_UNORM = 86,
			DXGI_FORMAT_B8G8R8A8_UNORM = 87,
			DXGI_FORMAT_B8G8R8X8_UNORM = 88,
			DXGI_FORMAT_R10G10B10_XR_BIAS_A2_UNORM = 89,
			DXGI_FORMAT_B8G8R8A8_TYPELESS = 90,
			DXGI_FORMAT_B8G8R8A8_UNORM_SRGB = 91,
			DXGI_FORMAT_B8G8R8X8_TYPELESS = 92,
			DXGI_FORMAT_B8G8R8X8_UNORM_SRGB = 93,
			DXGI_FORMAT_BC6H_TYPELESS = 94,
			DXGI_FORMAT_BC6H_UF16 = 95,
			DXGI_FORMAT_BC6H_SF16 = 96,
			DXGI_FORMAT_BC7_TYPELESS = 97,
			DXGI_FORMAT_BC7_UNORM = 98,
			DXGI_FORMAT_BC7_UNORM_SRGB = 99,
			DXGI_FORMAT_AYUV = 100,
			DXGI_FORMAT_Y410 = 101,
			DXGI_FORMAT_Y416 = 102,
			DXGI_FORMAT_NV12 = 103,
			DXGI_FORMAT_P010 = 104,
			DXGI_FORMAT_P016 = 105,
			DXGI_FORMAT_420_OPAQUE = 106,
			DXGI_FORMAT_YUY2 = 107,
			DXGI_FORMAT_Y210 = 108,
			DXGI_FORMAT_Y216 = 109,
			DXGI_FORMAT_NV11 = 110,
			DXGI_FORMAT_AI44 = 111,
			DXGI_FORMAT_IA44 = 112,
			DXGI_FORMAT_P8 = 113,
			DXGI_FORMAT_A8P8 = 114,
			DXGI_FORMAT_B4G4R4A4_UNORM = 115,
			DXGI_FORMAT_FORCE_UINT = 0xffffffff
		}
		[Flags] public enum EResFlags
		{
			None = 0,
			AllowRenderTarget = 0x1,
			AllowDepthStencil = 0x2,
			AllowUnorderedAccess = 0x4,
			DenyShaderResource = 0x8,
			AllowCrossAdapter = 0x10,
			AllowSimultaneousAccess = 0x20,
			VideoDecodeReferenceOnly = 0x40,
			VideoEncodeReferenceOnly = 0x80,
			RaytracingAccelerationStructure = 0x100
		}
		[Flags] public enum EResState : int
		{
			Common = 0,
			VertexAndConstantBuffer = 0x1,
			IndexBuffer = 0x2,
			RenderTarget = 0x4,
			UnorderedAccess = 0x8,
			DepthWrite = 0x10,
			DepthRead = 0x20,
			NonPixelShaderResource = 0x40,
			PixelShaderResource = 0x80,
			StreamOut = 0x100,
			IndirectArgument = 0x200,
			CopyDest = 0x400,
			CopySource = 0x800,
			ResolveDest = 0x1000,
			ResolveSource = 0x2000,
			RaytracingAccelerationStructure = 0x400000,
			ShadingRateSource = 0x1000000,
			GenericRead = (((((0x1 | 0x2) | 0x40) | 0x80) | 0x200) | 0x800),
			AllShaderResource = (0x40 | 0x80),
			Present = 0,
			Predication = 0x200,
			VideoDecodeRead = 0x10000,
			VideoDecodeWrite = 0x20000,
			VideoProcessRead = 0x40000,
			VideoProcessWrite = 0x80000,
			VideoEncodeRead = 0x200000,
			VideoEncodeWrite = 0x800000
		}
		public enum EFilter : int
		{
			Point             = 0,
			PointPointLinear  = 0x1,
			PointLinearPoint  = 0x4,
			PointLinearLinear = 0x5,
			LinearPointPoint  = 0x10,
			LinearPointLinear = 0x11,
			LinearLinearPoint = 0x14,
			Linear            = 0x15,
			Anisotropic       = 0x55,
		}
		public enum EAddrMode
		{
			Wrap = 1,
			Mirror = 2,
			Clamp = 3,
			Border = 4,
			MirrorOnce = 5,
		}
		#endregion

		#region Callbacks
		
		/// <summary>Report errors callback</summary>
		[StructLayout(LayoutKind.Sequential)]
		public struct ReportErrorCB
		{
			public delegate void FuncCB(IntPtr ctx, [MarshalAs(UnmanagedType.LPStr)] string msg, [MarshalAs(UnmanagedType.LPStr)] string filepath, int line, long pos);
			public IntPtr m_ctx;
			public FuncCB m_cb;

			public static ReportErrorCB Default()
			{
				static void ErrorSink(IntPtr ctx, string msg, string filepath, int line, long pos) => throw new Exception(msg);
				return new ReportErrorCB { m_cb = ErrorSink, m_ctx = IntPtr.Zero };
			}
		}

		/// <summary>Callback for progress updates during AddFile / Reload</summary>
		[StructLayout(LayoutKind.Sequential)]
		public struct ParsingProgressCB
		{
			public delegate void FuncCB(IntPtr ctx, ref Guid context_id, [MarshalAs(UnmanagedType.LPStr)] string filepath, long file_offset, long file_size, bool complete, ref bool cancel);
			public IntPtr m_ctx;
			public FuncCB m_cb;
		}

		/// <summary>Report settings changed callback</summary>
		[StructLayout(LayoutKind.Sequential)]
		public struct SettingsChangedCB
		{
			public delegate void FuncCB(IntPtr ctx, HWindow wnd, ESettings setting);
			public IntPtr m_ctx;
			public FuncCB m_cb;
		}

		/// <summary>Callback when the sources are reloaded</summary>
		[StructLayout(LayoutKind.Sequential)]
		public struct SourcesChangedCB
		{
			public delegate void FuncCB(IntPtr ctx, ESourcesChangedReason reason, IntPtr ids, int count, bool before);
			public IntPtr m_ctx;
			public FuncCB m_cb;
		}

		/// <summary>Predicate for selecting GUIDs</summary>
		[StructLayout(LayoutKind.Sequential)]
		public struct GuidPredCB
		{
			public delegate bool FuncCB(IntPtr ctx, ref Guid guid);
			public IntPtr m_ctx;
			public FuncCB m_cb;
		}

		/// <summary>Enumerate GUIDs callback</summary>
		[StructLayout(LayoutKind.Sequential)]
		public struct EnumGuidsCB
		{
			public delegate bool FuncCB(IntPtr ctx, ref Guid guid);
			public IntPtr m_ctx;
			public FuncCB m_cb;
		}

		/// <summary>Enumerate objects callback</summary>
		[StructLayout(LayoutKind.Sequential)]
		public struct EnumObjectsCB
		{
			public delegate bool FuncCB(IntPtr ctx, HObject obj);
			public IntPtr m_ctx;
			public FuncCB m_cb;
		}

		/// <summary>Callback for continuations after adding scripts/files</summary>
		[StructLayout(LayoutKind.Sequential)]
		public struct AddCompleteCB
		{
			public delegate void FuncCB(IntPtr ctx, ref Guid context_id, bool before);
			public IntPtr m_ctx;
			public FuncCB m_cb;
		}

		/// <summary>Callback for when the window is invalidated</summary>
		[StructLayout(LayoutKind.Sequential)]
		public struct InvalidatedCB
		{
			public delegate void FuncCB(IntPtr ctx, HWindow wnd);
			public IntPtr m_ctx;
			public FuncCB m_cb;
		}

		/// <summary>Called just prior to rendering</summary>
		[StructLayout(LayoutKind.Sequential)]
		public struct RenderingCB
		{
			public delegate void FuncCB(IntPtr ctx, HWindow wnd);
			public IntPtr m_ctx;
			public FuncCB m_cb;
		}

		/// <summary>Callback for when the collection of objects associated with a window changes</summary>
		[StructLayout(LayoutKind.Sequential)]
		public struct SceneChangedCB
		{
			public delegate void FuncCB(IntPtr ctx, HWindow wnd, ref SceneChanged args);
			public IntPtr m_ctx;
			public FuncCB m_cb;
		}

		/// <summary>Callback notification of animation commands</summary>
		[StructLayout(LayoutKind.Sequential)]
		public struct AnimationCB
		{
			public delegate void FuncCB(IntPtr ctx, HWindow wnd, EAnimCommand command, double clock);
			public IntPtr m_ctx;
			public FuncCB m_cb;
		}

		/// <summary>Callback with results of async hit tests</summary>
		[StructLayout(LayoutKind.Sequential)]
		public struct HitTestAsyncCB
		{
			public delegate void FuncCB(IntPtr ctx, HWindow window, [MarshalAs(UnmanagedType.LPArray, SizeParamIndex = 3)] HitTestResult[] results, int count);
			public IntPtr m_ctx;
			public FuncCB m_cb;
		}

		/// <summary>Callback notification of gizmo moved</summary>
		[StructLayout(LayoutKind.Sequential)]
		public struct GizmoMovedCB
		{
			public delegate void FuncCB(IntPtr ctx, HGizmo gizmo, EGizmoState state);
			public IntPtr m_ctx;
			public FuncCB m_cb;
		}

		/// <summary>Edit object callback</summary>
		[StructLayout(LayoutKind.Sequential)]
		private struct EditObjectCBInternal
		{
			public delegate VICount FuncCB(IntPtr ctx, int vcount, int icount, IntPtr vptr, IntPtr iptr, AddNuggetCBInternal out_nugget);
			public IntPtr m_ctx;
			public FuncCB m_cb;
		}

		/// <summary>Add nugget callback</summary>
		[StructLayout(LayoutKind.Sequential)]
		private struct AddNuggetCBInternal
		{
			public delegate void FuncCB(IntPtr ctx, ref Nugget nugget);
			public IntPtr m_ctx;
			public FuncCB m_cb;
		}

		#endregion

		#region Structures

		/// <summary></summary>
		[StructLayout(LayoutKind.Sequential)]
		public struct VICount
		{
			public int m_vcount;
			public int m_icount;
		};

		/// <summary></summary>
		[StructLayout(LayoutKind.Sequential)]
		public struct Vertex
		{
			public v4 m_pos;
			public v4 m_norm;
			public v2 m_uv;
			public uint m_col;
			public uint pad;

			public Vertex(v4 vert) { m_pos = vert; m_col = ~0U; m_norm = v4.Zero; m_uv = v2.Zero; pad = 0; }
			public Vertex(v4 vert, uint col) { m_pos = vert; m_col = col; m_norm = v4.Zero; m_uv = v2.Zero; pad = 0; }
			public Vertex(v4 vert, v4 norm, uint col, v2 tex) { m_pos = vert; m_col = col; m_norm = norm; m_uv = tex; pad = 0; }

			public override readonly string ToString() { return $"V:<{m_pos}> C:<{m_col:X8}>"; }
		}

		/// <summary></summary>
		[StructLayout(LayoutKind.Sequential)]
		public struct Nugget
		{
			[StructLayout(LayoutKind.Sequential)]
			public struct Shader(ERenderStep rdr_step, HShader shader)
			{
				public HShader m_shader = shader;
				public ERenderStep m_rdr_step = rdr_step;
				public int pad;
			}

			public ETopo m_topo;           // Model topology
			public EGeom m_geom;           // Model geometry type
			public int m_v0, m_v1;         // Vertex buffer range. Set to 0,0 to mean the whole buffer. Use 1,1 if you want to say 0-span
			public int m_i0, m_i1;         // Index buffer range. Set to 0,0 to mean the whole buffer. Use 1,1 if you want to say 0-span
			public HTexture m_tex_diffuse; // Diffuse texture
			public HSampler m_sam_diffuse; // Sampler for the diffuse texture
			[MarshalAs(UnmanagedType.ByValArray, SizeConst = 8)]
			public Shader[] m_shaders;     // Shader overrides for this nugget
			public ENuggetFlag m_nflags;   // Nugget flags (ENuggetFlag)
			public ECullMode m_cull_mode;  // Face culling mode
			public EFillMode m_fill_mode;  // Model fill mode
			public Colour32 m_tint;        // Tint colour
			public float m_rel_reflec;     // How reflective this nugget is relative to the over all model

			public Nugget(
				ETopo topo, EGeom geom,
				int v0 = 0, int v1 = 0,
				int i0 = 0, int i1 = 0,
				HTexture? tex_diffuse = null,
				HSampler? sam_diffuse = null,
				Shader[]? shaders = null,
				ENuggetFlag? flags = null,
				ECullMode? cull_mode = null,
				EFillMode? fill_mode = null,
				Colour32? tint = null,
				float? rel_reflec = null)
			{
				m_topo = topo;
				m_geom = geom;
				m_v0 = v0;
				m_v1 = v1;
				m_i0 = i0;
				m_i1 = i1;
				m_tex_diffuse = tex_diffuse ?? IntPtr.Zero;
				m_sam_diffuse = sam_diffuse ?? IntPtr.Zero;
				m_shaders = shaders ?? new Shader[8];
				m_nflags = flags ?? ENuggetFlag.None;
				m_cull_mode = cull_mode ?? ECullMode.Default;
				m_fill_mode = fill_mode ?? EFillMode.Default;
				m_tint = tint ?? Colour32.White;
				m_rel_reflec = rel_reflec ?? 1f;
			}
		}

		/// <summary>Light source properties</summary>
		[StructLayout(LayoutKind.Sequential)]
		public struct LightInfo
		{
			/// <summary>Position, only valid for point and spot lights</summary>
			public v4 Position;

			/// <summary>Direction, only valid for directional and spot lights</summary>
			public v4 Direction;

			/// <summary>The light source type. One of ambient, directional, point, spot</summary>
			public ELight Type;

			/// <summary>Ambient light colour</summary>
			public Colour32 AmbientColour;

			/// <summary>Main light colour</summary>
			public Colour32 DiffuseColour;

			/// <summary>Specular light colour</summary>
			public Colour32 SpecularColour;

			/// <summary>Specular power (controls specular spot size)</summary>
			public float SpecularPower;

			/// <summary>Light range</summary>
			public float Range;

			/// <summary>Intensity falloff per unit distance</summary>
			public float Falloff;

			/// <summary>Spot light inner angle 100% light (in radians)</summary>
			public float InnerAngle;

			/// <summary>Spot light outer angle 0% light (in radians)</summary>
			public float OuterAngle;

			/// <summary>Shadow cast range, 0 for off</summary>
			public float CastShadow;

			/// <summary>True if the light should move with the camera</summary>
			public bool CameraRelative;

			/// <summary>True if this light is on</summary>
			public bool On;

			/// <summary>Default light info</summary>
			public static LightInfo Default()
			{
				return new LightInfo
				{
					On = true,
					Type = ELight.Ambient,
					Position = v4.Origin,
					Direction = -v4.ZAxis,
					AmbientColour = 0xFF404040,
					DiffuseColour = 0xFF404040,
					SpecularColour = 0xFF808080,
					SpecularPower = 1000f,
					InnerAngle = Math_.TauBy4F,
					OuterAngle = Math_.TauBy4F,
					Range = 1000f,
					Falloff = 0f,
					CastShadow = 0f,
					CameraRelative = false,
				};
			}

			/// <summary>Return properties for an ambient light source</summary>
			public static LightInfo Ambient(Colour32? ambient = null)
			{
				var light = Default();
				light.Type = ELight.Ambient;
				light.AmbientColour = ambient ?? light.AmbientColour;
				return light;
			}

			/// <summary>Return properties for a directional light source</summary>
			public static LightInfo Directional(v4 direction, Colour32? ambient = null, Colour32? diffuse = null, Colour32? specular = null, float? spec_power = null, float? cast_shadow = null, bool camera_relative = false)
			{
				var light = Default();
				light.Type = ELight.Directional;
				light.Direction = Math_.Normalise(direction);
				light.AmbientColour = ambient ?? light.AmbientColour;
				light.DiffuseColour = diffuse ?? light.DiffuseColour;
				light.SpecularColour = specular ?? light.SpecularColour;
				light.SpecularPower = spec_power ?? light.SpecularPower;
				light.CastShadow = cast_shadow ?? light.CastShadow;
				light.CameraRelative = camera_relative;
				return light;
			}

			/// <summary>Return properties for a point light source</summary>
			public static LightInfo Point(v4 position, Colour32? ambient = null, Colour32? diffuse = null, Colour32? specular = null, float? spec_power = null, float? cast_shadow = null, bool camera_relative = false)
			{
				var light = Default();
				light.Type = ELight.Point;
				light.Position = position;
				light.AmbientColour = ambient ?? light.AmbientColour;
				light.DiffuseColour = diffuse ?? light.DiffuseColour;
				light.SpecularColour = specular ?? light.SpecularColour;
				light.SpecularPower = spec_power ?? light.SpecularPower;
				light.CastShadow = cast_shadow ?? light.CastShadow;
				light.CameraRelative = camera_relative;
				return light;
			}
		}

		/// <summary>Light source properties</summary>
		[StructLayout(LayoutKind.Sequential)]
		public struct MultiSamp(int? count = null, int? quality = null) // DXGI_SAMPLE_DESC
		{
			public int Count = count ?? 1;
			public int Quality = quality ?? 0;

			public static int BestQuality(int count, EFormat format)
			{
				return View3D_MSAAQuality(count, format);
			}
		}

		/// <summary></summary>
		[StructLayout(LayoutKind.Sequential)]
		public struct TextureOptions(
			m4x4? t2s = null, 
			EFormat? format = null, 
			int? mips = null, 
			EResFlags? usage = null, 
			EResState? state = null, 
			ClearValue? clear_value = null, 
			MultiSamp? msaa = null, 
			uint? colour_key = null, 
			bool? has_alpha = null, 
			string? dbg_name = null)
		{
			public m4x4 T2S = t2s ?? m4x4.Identity;
			public EFormat Format = format ?? EFormat.DXGI_FORMAT_R8G8B8A8_UNORM;
			public int Mips = mips ?? 0;
			public EResFlags Usage = usage ?? EResFlags.None;
			public EResState State = state ?? EResState.Common;
			public ClearValue ClearValue = clear_value ?? new ClearValue(format ?? EFormat.DXGI_FORMAT_R8G8B8A8_UNORM);
			public MultiSamp MultiSamp = msaa ?? new MultiSamp();
			public uint ColourKey = colour_key ?? 0U;
			public bool HasAlpha = has_alpha ?? false;
			public string DbgName = dbg_name ?? string.Empty;
		}

		/// <summary></summary>
		[StructLayout(LayoutKind.Sequential, CharSet = CharSet.Ansi)]
		public struct CubeMapOptions
		{
			public m4x4 m_cube2w;
			[MarshalAs(UnmanagedType.LPStr)] public string m_dbg_name;
		};

		/// <summary></summary>
		[StructLayout(LayoutKind.Sequential)]
		public struct SamplerOptions
		{
			public EFilter m_filter;
			public EAddrMode m_addrU;
			public EAddrMode m_addrV;
			public EAddrMode m_addrW;
			[MarshalAs(UnmanagedType.LPStr)] public string m_dbg_name;
		};

		/// <summary></summary>
		[StructLayout(LayoutKind.Sequential)]
		public struct ShaderOptions
		{
			// todo
		}

		/// <summary></summary>
		[StructLayout(LayoutKind.Sequential)]
		public struct WindowOptions
		{
			public ReportErrorCB ErrorCB;
			public Colour32 BackgroundColour;
			public bool AllowAltEnter;
			public bool GdiCompatibleBackBuffer;
			public bool XrSupport;
			public int Multisampling;
			public string DbgName;

			public static WindowOptions New(ReportErrorCB? error_cb = null, Colour32? background_colour = null, bool? allow_alt_enter = null, bool? xr_support = null, bool? gdi_compatible_bb = null, string? dbg_name = null)
			{
				return new()
				{
					ErrorCB = error_cb ?? ReportErrorCB.Default(),
					BackgroundColour = background_colour ?? Colour32.Gray,
					AllowAltEnter = allow_alt_enter ?? false,
					GdiCompatibleBackBuffer = gdi_compatible_bb ?? false,
					XrSupport = xr_support ?? false,
					Multisampling = (gdi_compatible_bb ?? false) ? 1 : 4,
					DbgName = dbg_name ?? string.Empty,
				};
			}
		}

		/// <summary>A ray description for hit testing in a 3d scene</summary>
		[StructLayout(LayoutKind.Sequential)]
		public struct HitTestRay
		{
			// The world space origin and direction of the ray (normalisation not required)
			public v4 m_ws_origin;
			public v4 m_ws_direction;

			// Snap mode and distance. If snap_mode includes ESnapMode::Perspective, then the snap distance scales with distance from the origin
			public ESnapMode m_snap_mode; // Snap behaviour
			public float m_snap_distance; // Snap distance: 'snap_dist = Perspectvie ? snap_distance * depth : snap_distance'

			// User provided id for the ray
			public HitTestRayId m_id;
			public int pad;

			public static HitTestRay New(v4 origin, v4 direction, ESnapMode snap_mode = ESnapMode.AllPerspective, float snap_distance = 0, HitTestRayId id = 0)
			{
				return new HitTestRay
				{
					m_ws_origin = origin,
					m_ws_direction = direction,
					m_snap_mode = snap_mode,
					m_snap_distance = snap_distance,
					m_id = id,
				};
			}
			public static HitTestRay Null(HitTestRayId id)
			{
				return new HitTestRay
				{
					m_ws_origin = v4.Origin,
					m_ws_direction = v4.Zero,
					m_snap_mode = ESnapMode.NoSnap,
					m_snap_distance = 0,
					m_id = id,
				};
			}
		}

		/// <summary>The result of a ray cast hit test in a 3d scene</summary>
		[StructLayout(LayoutKind.Sequential)]
		public struct HitTestResult
		{
			// The origin and direction of the cast ray (in world space)
			public v4 m_ws_ray_origin;
			public v4 m_ws_ray_direction;

			// The intercept point and surface normal (in world space)
			public v4 m_ws_intercept;
			public v4 m_ws_normal;

			// The object that was hit (or null)
			public readonly Object? HitObject => IsHit ? new Object(m_obj) : null;
			private readonly IntPtr m_obj;

			/// <summary>The distance from the ray origin to the hit point</summary>
			public float m_distance;

			// The index/id of the input ray
			public int m_ray_index;
			public int m_ray_id;

			/// <summary>How the hit point was snapped (if at all)</summary>
			public ESnapType m_snap_type;

			int pad0;
			int pad1;

			/// <summary>True if something was hit</summary>
			public readonly bool IsHit => m_obj != IntPtr.Zero;
		}

		/// <summary>The viewport volume in render target space (i.e. not normalised)</summary>
		[StructLayout(LayoutKind.Sequential)]
		public struct Viewport
		{
			// Notes:
			//  - The viewport is in "render target" space, so if the render target is a
			//    different size to the target window, 'Width/Height' do not equal the window size.
			//    The ScreenW/ScreenH fields are used to pass the size of the target window.
			public float X;        // (x,y,x+width,y+width) is in back buffer pixels, *NOT* window DIP.
			public float Y;        // Typically the back buffer is the same size as the true screen pixels
			public float Width;    // Typically the BB width
			public float Height;   // Typically the BB height
			public float DepthMin; // Typically 0.0f
			public float DepthMax; // Typically 1.0f
			public int ScreenW;    // The screen width in DIP
			public int ScreenH;    // The screen height in DIP

			public Viewport(float x, float y, float w_px, float h_px, int screen_w_dip, int screen_h_dip, float min, float max)
			{
				X = x;
				Y = y;
				Width = w_px;
				Height = h_px;
				ScreenW = screen_w_dip;
				ScreenH = screen_h_dip;
				DepthMin = min;
				DepthMax = max;
			}
			public readonly float Aspect => Width / Height;
			public readonly Size ToSize() => new((int)Math.Round(Width), (int)Math.Round(Height));
			public readonly SizeF ToSizeF() => new(Width, Height);
			public readonly Rectangle ToRect() => new((int)X, (int)Y, (int)Math.Round(Width), (int)Math.Round(Height));
			public readonly RectangleF ToRectF() => new(X, Y, Width, Height);
		}

		/// <summary>Include paths/sources for Ldr script #include resolving</summary>
		[StructLayout(LayoutKind.Sequential)]
		public struct Includes
		{
			/// <summary>
			/// Create an includes object.
			/// 'paths' should be null or a comma or semi colon separated list of directories
			/// Remember module == IntPtr.Zero means "this" module</summary>
			public Includes(string paths)
				: this(paths, null)
			{ }
			public Includes(HMODULE module, IEnumerable<string> paths)
				: this(paths, new[] { module })
			{ }
			public Includes(IEnumerable<string> paths, IEnumerable<HMODULE> modules)
				: this(string.Join(",", paths), [.. modules.Take(16)])
			{ }
			public Includes(string paths, HMODULE[]? modules)
			{
				m_include_paths = paths;
				m_modules = new HMODULE[16];
				m_module_count = 0;

				if (modules != null)
				{
					Array.Copy(modules, m_modules, modules.Length);
					m_module_count = modules.Length;
				}
			}

			/// <summary>A comma or semicolon separated list of search directories</summary>
			public string IncludePaths
			{
				readonly get => m_include_paths;
				set => m_include_paths = value;
			}
			[MarshalAs(UnmanagedType.LPStr)]
			public string m_include_paths;

			/// <summary>An array of binary modules that contain resources</summary>
			[MarshalAs(UnmanagedType.ByValArray, SizeConst = 16)]
			public readonly IntPtr[] m_modules;
			public readonly int m_module_count;
		}

		/// <summary></summary>
		[StructLayout(LayoutKind.Sequential)]
		public readonly struct SceneChanged
		{
			/// <summary>How the scene was changed</summary>
			public readonly ESceneChanged ChangeType;

			/// <summary>The context ids involved in the change</summary>
			public readonly Guid[] ContextIds => Marshal_.PtrToArray<Guid>(m_ctx_ids, m_count);
			private readonly IntPtr m_ctx_ids;
			private readonly int m_count;

			/// <summary>The object that changed (for single object changes only)</summary>
			public readonly Object? Object => m_object != IntPtr.Zero ? new Object(m_object) : null;
			private readonly HObject m_object;
		}

		/// <summary></summary>
		[StructLayout(LayoutKind.Sequential)]
		public struct AnimEvent
		{
			/// <summary>The state change type</summary>
			public EAnimCommand m_command;

			/// <summary>The current animation clock value</summary>
			public double m_clock;
		}

		/// <summary></summary>
		[StructLayout(LayoutKind.Sequential)]
		public struct ImageInfo
		{
			public ulong Width;
			public uint Height;
			public ushort Depth;
			public ushort Mips;
			public EFormat Format;
			public uint ImageFileFormat; // D3DXIMAGE_FILEFORMAT
		}

		/// <summary></summary>
		[StructLayout(LayoutKind.Sequential)]
		public struct BackBuffer
		{
			public IntPtr RenderTarget; // ID3D12Resource*
			public IntPtr DepthStencil; // ID3D12Resource*
			public Size Dim;
		}

		/// <summary>Information about a source</summary>
		public class SourceInfo
		{
			/// <summary>Friendly name for the source</summary>
			public string Name;

			/// <summary>The filepath associated with the source</summary>
			public string? FilePath;

			/// <summary>Context id for the source</summary>
			public Guid ContextId;

			/// <summary>The number of object in this source</summary>
			public int ObjectCount;

			/// <summary>True if the source is a text format, false if binary</summary>
			public bool TextFormat;

			internal SourceInfo(Interop interop)
			{
				Name = Marshal_.PtrToStringUTF8(interop.m_name);
				FilePath = Marshal_.PtrToStringUTF16(interop.m_filepath);
				ContextId = interop.m_context_id;
				ObjectCount = interop.m_object_count;
				TextFormat = interop.m_text_format != 0;
			}

			[StructLayout(LayoutKind.Sequential)]
			internal struct Interop
			{
				public IntPtr m_name; // char*
				public IntPtr m_filepath; // wchar_t*
				public Guid m_context_id;
				public int m_object_count;
				public int m_text_format;
			}
		}
		
		#endregion
		#region D3D Structures

		/// <summary></summary>
		[StructLayout(LayoutKind.Sequential)]
		public struct DepthStencilValue
		{
			public float Depth;
			public byte Stencil;
		}

		/// <summary></summary>
		[StructLayout(LayoutKind.Explicit)]
		public struct ClearValue(EFormat format, Colour128 colour = default)
		{
			[FieldOffset(0)] public EFormat Format = format;
			[FieldOffset(4)] public Colour128 Colour = colour;
			[FieldOffset(4)] public DepthStencilValue DepthStencil;
		}

		#endregion

		[Serializable]
		public class Exception :System.Exception
		{
			public EResult m_code = EResult.Success;
			public Exception() : this(EResult.Success) { }
			public Exception(EResult code) : this("", code) { }
			public Exception(string message) : this(message, EResult.Success) { }
			public Exception(string message, EResult code) : base(message) { m_code = code; }
			public Exception(string message, System.Exception innerException) : base(message, innerException) {}
		}

		private readonly List<Window> m_windows;            // Groups of objects to render
		private readonly HContext m_context;                // Unique id per Initialise call
		private readonly SynchronizationContext m_sync;     // Thread marshaller
		private readonly int m_thread_id;                   // The main thread id
		private ReportErrorCB m_error_cb;                   // Reference to callback
		private ParsingProgressCB m_parsing_progress_cb;    // Reference to callback
		private SourcesChangedCB m_sources_changed_cb;      // Reference to callback
		private List<AddCompleteCB> m_add_complete_cb; // Reference to callbacks

		#if PR_VIEW3D_CREATE_STACKTRACE
		private static List<StackTrace> m_create_stacktraces = new List<StackTrace>();
		#endif

		/// <summary>Create a reference to the View3d singleton instance for this process</summary>
		public static View3d Create()
		{
			#if PR_VIEW3D_CREATE_STACKTRACE
			m_create_stacktraces.Add(new StackTrace(true));
			#endif

			++m_ref_count;
			return m_singleton ??= new View3d();
		}
		private static View3d? m_singleton;
		private static int m_ref_count;

		/// <summary></summary>
		private View3d()
		{
			if (!ModuleLoaded)
				throw new Exception("View3d.dll has not been loaded");

			m_windows = new List<Window>();
			m_sync = SynchronizationContext.Current ?? new SynchronizationContext();
			m_thread_id = Environment.CurrentManagedThreadId;
			m_add_complete_cb = [];

			try
			{
				// Initialise view3d
				m_context = View3D_Initialise(m_error_cb = new ReportErrorCB { m_cb = HandleError });
				void HandleError(IntPtr ctx, string msg, string filepath, int line, long pos)
				{
					if (m_thread_id != Thread.CurrentThread.ManagedThreadId)
					{
						m_sync.Send(_ => HandleError(ctx, msg, filepath, line, pos), null);
						return;
					}

					LastError = new Exception(msg);
					Error?.Invoke(this, new ErrorEventArgs(msg, filepath, line, pos));
				}
				if (m_context == HContext.Zero)
					throw LastError ?? new Exception("Failed to initialised View3d");

				// Sign up for progress reports
				View3D_ParsingProgressCBSet(m_parsing_progress_cb = new ParsingProgressCB { m_cb = HandleParsingProgress }, true);
				void HandleParsingProgress(IntPtr ctx, ref Guid context_id, string filepath, long foffset, long fsize, bool complete, ref bool cancel)
				{
					var args = new ParsingProgressEventArgs(context_id, filepath, foffset, fsize, complete);
					ParsingProgress?.Invoke(this, args);
					cancel = args.Cancel;
				}

				// Sign up for notification of the sources changing
				View3D_SourcesChangedCBSet(m_sources_changed_cb = new SourcesChangedCB { m_cb = HandleSourcesChanged }, true);
				void HandleSourcesChanged(IntPtr ctx, ESourcesChangedReason reason, IntPtr ids, int count, bool before)
				{
					if (m_thread_id != Thread.CurrentThread.ManagedThreadId)
						m_sync.Post(_ => HandleSourcesChanged(ctx, reason, ids, count, before), null);
					else
						OnSourcesChanged?.Invoke(this, new SourcesChangedEventArgs(reason, ids, count, before));
				}
			}
			catch
			{
				Dispose();
				throw;
			}
		}
		public void Dispose()
		{
			Util.BreakIf(Util.IsGCFinalizerThread, "Disposing in the GCFinaliserThread");
			if (--m_ref_count != 0)
			{
				Util.BreakIf(m_ref_count < 0);
				return;
			}
			if (m_singleton != null)
			{
				View3D_SourcesChangedCBSet(m_sources_changed_cb, false);
				View3D_ParsingProgressCBSet(m_parsing_progress_cb, false);
				View3D_GlobalErrorCBSet(m_error_cb, false);

				while (m_windows.Count != 0)
					m_windows[0].Dispose();

				View3D_Shutdown(m_context);
				m_singleton = null;
			}
			GC.SuppressFinalize(this);
		}

		/// <summary>The last error reported from View3d</summary>
		public Exception? LastError { get; private set; }

		/// <summary>Event call on errors. Note: can be called in a background thread context</summary>
		public event EventHandler<ErrorEventArgs>? Error;

		/// <summary>Progress update when a file is being parsed</summary>
		public event EventHandler<ParsingProgressEventArgs>? ParsingProgress;

		/// <summary>Event notifying whenever sources are loaded/reloaded</summary>
		public event EventHandler<SourcesChangedEventArgs>? OnSourcesChanged;

		/// <summary>
		/// Add objects from an ldr file or string. This will create all objects declared in 'ldr_script'
		/// with context id 'context_id' if given, otherwise an id will be created.
		/// 'include_paths' is a list of include paths to use to resolve #include directives (or null).</summary>
		public Source LoadScriptFromString(string ldr_script, Guid? context_id = null, string[]? include_paths = null, Action<Guid, bool>? add_completed = null)
		{
			// Note: this method is asynchronous, it returns before objects have been added to the object manager
			// in view3d. The 'on_add' callback should be used to add objects to windows once they are available.
			var ctx = context_id ?? Guid.NewGuid();
			var includes = new Includes { m_include_paths = string.Join(",", include_paths ?? []) };
			ctx = View3D_LoadScriptFromString(ldr_script, ref ctx, ref includes, WrapAddCompletedCB(add_completed));
			return new Source(ctx, this);
		}

		/// <summary>
		/// Add objects from an ldr file. This will create all objects declared in 'ldr_script'
		/// with context id 'context_id' if given, otherwise an id will be created.
		/// 'include_paths' is a list of include paths to use to resolve #include directives (or null).</summary>
		public Source LoadScriptFromFile(string ldr_script_file, Guid? context_id = null, string[]? include_paths = null, Action<Guid, bool>? add_completed = null)
		{
			// Note: this method is asynchronous, it returns before objects have been added to the store in view3d.
			// The 'add_completed' callback should be used to add objects to windows once they are available.
			var ctx = context_id ?? View3D_ContextIdFromFilepath(ldr_script_file);
			var includes = new Includes { m_include_paths = string.Join(",", include_paths ?? []) };
			ctx = View3D_LoadScriptFromFile(ldr_script_file, ref ctx, ref includes, WrapAddCompletedCB(add_completed));
			return new Source(ctx, this);
		}

		/// <summary>Cancel an in-progress load operation. Thread-safe.</summary>
		public void CancelLoad(Guid context_id)
		{
			View3D_CancelLoad(ref context_id);
		}

		/// <summary>Wrap the 'add_completed' callback so that it doesn't get collected during an async load</summary>
		private AddCompleteCB WrapAddCompletedCB(Action<Guid, bool>? add_completed)
		{
			if (add_completed is null)
				return new AddCompleteCB();

			// Use a GCHandle as the context pointer to keep the delegate alive and provide a unique key for cleanup.
			// Marshal.GetFunctionPointerForDelegate doesn't work with generic delegate types like Action<Guid, bool>.
			var handle = GCHandle.Alloc(add_completed);
			var ctx_ptr = GCHandle.ToIntPtr(handle);
			var add_completed_cb = new AddCompleteCB { m_cb = HandleLoadScript, m_ctx = ctx_ptr };
			m_add_complete_cb.Add(add_completed_cb);
			return add_completed_cb;

			void HandleLoadScript(IntPtr ctx, ref Guid context_id, bool before)
			{
				if (!before)
				{
					m_add_complete_cb.RemoveAll(x => x.m_ctx == ctx);
					GCHandle.FromIntPtr(ctx).Free();
				}
				add_completed.Invoke(context_id, before);
			}
		}

		/// <summary>Force a reload of all script sources</summary>
		public void ReloadScriptSources()
		{
			View3D_ReloadScriptSources();
		}

		/// <summary>Poll for changed script source files, and reload any that have changed. (designed as a Timer.Tick handler)</summary>
		public void CheckForChangedSources(object? sender = null, EventArgs? args = null)
		{
			View3D_CheckForChangedSources();
		}

		/// <summary>Get/Enable/Disable streaming sources.</summary>
		public EStreamingState StreamingState => View3D_StreamingState();
		public void Streaming(bool enable, int port)
		{
			View3D_StreamingEnable(enable, port);
		}

		/// <summary>
		/// Delete all objects
		/// *WARNING* Careful with this function, make sure all C# references to View3D objects have been set to null
		/// otherwise, disposing them will result in memory corruption</summary>
		public void DeleteAllObjects()
		{
			View3D_DeleteAllObjects();
		}

		/// <summary>Delete all objects, filtered by 'context_ids'</summary>
		public void DeleteObjects(Func<Guid, bool> pred)
		{
			var pred_cb = new GuidPredCB { m_cb = (IntPtr ctx, ref Guid guid) => pred(guid) };
			View3D_DeleteById(pred_cb);
		}

		/// <summary>Release all objects not displayed in any windows (filtered by 'context_ids')</summary>
		public void DeleteUnused(GuidPredCB pred)
		{
			View3D_DeleteUnused(pred);
		}

		/// <summary>Return the context id for objects created from file 'filepath' (or null if 'filepath' is not an existing source)</summary>
		public static Guid ContextIdFromFilepath(string filepath)
		{
			return View3D_ContextIdFromFilepath(filepath);
		}

		/// <summary>Enumerate the sources in the store</summary>
		public void EnumSources(Action<Source> cb)
		{
			EnumSources(src => { cb(src); return true; });
		}
		public void EnumSources(Func<Source, bool> cb)
		{
			var enum_cb = new EnumGuidsCB { m_cb = (IntPtr ctx, ref Guid guid) => cb(new Source(guid, this)) };
			View3D_EnumSources(enum_cb);
		}

		/// <summary>Return info about a source given by context id</summary>
		public SourceInfo SourceInformation(Guid context_id)
		{
			return new SourceInfo(View3D_SourceInfo(ref context_id));
		}

		/// <summary>Return the example Ldr script</summary>
		public static string ExampleScript => View3D_ExampleScriptBStr();

		/// <summary>Template descriptions for auto complete of LDraw script</summary>
		public static string AutoCompleteTemplates => View3D_AutoCompleteTemplatesBStr();

		/// <summary>Return the address (form: keyword.keyword...) within a script at 'position'</summary>
		public static string AddressAt(string ldr_script, long position = -1)
		{
			// 'script' should start from a root level position.
			// 'position' should be relative to 'script'
			return View3D_ObjectAddressAt(ldr_script, position != -1 ? position : ldr_script.Length) ?? string.Empty;
		}

		#region DLL extern functions

		private const string Dll = "view3d-12";

		/// <summary>True if the view3d dll has been loaded</summary>
		public static bool ModuleLoaded => m_module != IntPtr.Zero;
		private static IntPtr m_module = IntPtr.Zero;

		/// <summary>The exception created if the module fails to load</summary>
		public static System.Exception? LoadError;

		/// <summary>Helper method for loading the view3d.dll from a platform specific path</summary>
		public static bool LoadDll(string dir = @".\lib\$(platform)\$(config)", bool throw_if_missing = true)
		{
			if (ModuleLoaded) return true;
			m_module = Win32.LoadDll(Dll + ".dll", out LoadError, dir, throw_if_missing);
			return m_module != IntPtr.Zero;
		}

		// Dll Context ****************************

		// Initialise calls are reference counted and must be matched with Shutdown calls
		// 'initialise_error_cb' is used to report dll initialisation errors only (i.e. it isn't stored)
		// Note: this function is not thread safe, avoid race calls
		[DllImport(Dll)] private static extern HContext View3D_Initialise(ReportErrorCB global_error_cb);
		[DllImport(Dll)] private static extern void View3D_Shutdown(HContext context);

		// This error callback is called for errors that are associated with the dll (rather than with a window).
		[DllImport(Dll)] private static extern void View3D_GlobalErrorCBSet(ReportErrorCB error_cb, bool add);

		// Set the callback for progress events when script sources are loaded or updated
		[DllImport(Dll)] private static extern void View3D_ParsingProgressCBSet(ParsingProgressCB progress_cb, bool add);

		// Set the callback that is called when the sources are reloaded
		[DllImport(Dll)] private static extern void View3D_SourcesChangedCBSet(SourcesChangedCB sources_changed_cb, bool add);

		// Return the context id for objects created from 'filepath' (if filepath is an existing source)
		[DllImport(Dll)] private static extern Guid View3D_ContextIdFromFilepath([MarshalAs(UnmanagedType.LPStr)] string filepath);

		// Data Sources ***************************

		// Add an ldr script source. This will create all objects with context id 'context_id' (if given, otherwise an id will be created). Concurrent calls are thread safe.
		[DllImport(Dll)] private static extern Guid View3D_LoadScriptFromString([MarshalAs(UnmanagedType.LPStr)] string ldr_script, ref Guid context_id, ref Includes includes, AddCompleteCB on_add_cb);
		[DllImport(Dll)] private static extern Guid View3D_LoadScriptFromFile([MarshalAs(UnmanagedType.LPStr)] string ldr_file, ref Guid context_id, ref Includes includes, AddCompleteCB on_add_cb);

		// Cancel an in-progress load operation
		[DllImport(Dll)] private static extern void View3D_CancelLoad(ref Guid context_id);

		// Enumerate all sources in the store
		[DllImport(Dll)] private static extern void View3D_EnumSources(EnumGuidsCB enum_guids_cb);

		// Reload objects from the source associated with 'context_id'
		[DllImport(Dll)] private static extern void View3D_SourceReload(ref Guid context_id);

		// Delete all objects and remove the source associated with 'context_id'
		[DllImport(Dll)] private static extern void View3D_SourceDelete(ref Guid context_id);

		// Get information about a source
		[DllImport(Dll)] private static extern SourceInfo.Interop View3D_SourceInfo(ref Guid context_id);

		// Get/Set the name of a source
		[DllImport(Dll, CharSet = CharSet.Unicode)][return: MarshalAs(UnmanagedType.BStr)] private static extern string View3D_SourceNameGetBStr(ref Guid context_id);
		[DllImport(Dll, CharSet = CharSet.Ansi)] private static extern void View3D_SourceNameSet(ref Guid context_id, [MarshalAs(UnmanagedType.LPStr)] string name);

		// Reload script sources. This will delete all objects associated with the script sources then reload the files creating new objects with the same context ids.
		[DllImport(Dll)] private static extern void View3D_ReloadScriptSources();

		// Delete all objects and object sources
		[DllImport(Dll)] private static extern void View3D_DeleteAllObjects();
		
		// Delete all objects matching (or not matching) a context id
		[DllImport(Dll)] private static extern void View3D_DeleteById(GuidPredCB pred);

		// Delete all objects not displayed in any windows
		[DllImport(Dll)] private static extern void View3D_DeleteUnused(GuidPredCB pred);

		// Poll for changed script sources and reload any that have changed
		[DllImport(Dll)] private static extern void View3D_CheckForChangedSources();
		
		// Enable/Disable streaming script sources.
		[DllImport(Dll)] private static extern EStreamingState View3D_StreamingState();
		[DllImport(Dll)] private static extern void View3D_StreamingEnable(bool enable, int port);

		// Windows ********************************

		// Create/Destroy a window
		[DllImport(Dll)] private static extern HWindow View3D_WindowCreate(HWND hwnd, ref WindowOptions opts);
		[DllImport(Dll)] private static extern void View3D_WindowDestroy(HWindow window);

		// Add/Remove a window error callback. Note: The callback function can be called in a worker thread context.
		[DllImport(Dll)] private static extern void View3D_WindowErrorCBSet(HWindow window, ReportErrorCB error_cb, bool add);

		// Get/Set the window settings (as ldr script string)
		[DllImport(Dll, CharSet = CharSet.Unicode)][return: MarshalAs(UnmanagedType.BStr)] private static extern string View3D_WindowSettingsGetBStr(HWindow window);
		[DllImport(Dll, CharSet = CharSet.Ansi)] private static extern void View3D_WindowSettingsSet(HWindow window, [MarshalAs(UnmanagedType.LPStr)] string settings);

		// Get/Set the dimensions of the render target. Note: Not equal to window size for non-96 dpi screens!
		// In set, if 'width' and 'height' are zero, the RT is resized to the associated window automatically.
		[DllImport(Dll)] private static extern Size View3D_WindowBackBufferSizeGet(HWindow window);
		[DllImport(Dll)] private static extern void View3D_WindowBackBufferSizeSet(HWindow window, Size size, bool force_recreate);

		// Get/Set the window viewport (and clipping area)
		[DllImport(Dll)] private static extern Viewport View3D_WindowViewportGet(HWindow window);
		[DllImport(Dll)] private static extern void View3D_WindowViewportSet(HWindow window, ref Viewport vp);

		// Set a notification handler for when a window setting changes
		[DllImport(Dll)] private static extern void View3D_WindowSettingsChangedCB(HWindow window, SettingsChangedCB settings_changed_cb, bool add);

		// Add/Remove a callback that is called when the collection of objects associated with 'window' changes
		[DllImport(Dll)] private static extern void View3D_WindowSceneChangedCB(HWindow window, SceneChangedCB scene_changed_cb, bool add);

		// Add/Remove a callback that is called just prior to rendering the window
		[DllImport(Dll)] private static extern void View3D_WindowRenderingCB(HWindow window, RenderingCB rendering_cb, bool add);

		// Add/Remove an object to/from a window
		[DllImport(Dll)] private static extern void View3D_WindowAddObject(HWindow window, HObject obj);
		[DllImport(Dll)] private static extern void View3D_WindowRemoveObject(HWindow window, HObject obj);

		// Add/Remove a gizmo to/from a window
		[DllImport(Dll)] private static extern void View3D_WindowAddGizmo(HWindow window, HGizmo giz);
		[DllImport(Dll)] private static extern void View3D_WindowRemoveGizmo(HWindow window, HGizmo giz);

		// Add/Remove objects by context id. This function can be used to add all objects either in, or not in 'context_ids'
		[DllImport(Dll)] private static extern void View3D_WindowAddObjectsById(HWindow window, GuidPredCB pred);
		[DllImport(Dll)] private static extern void View3D_WindowRemoveObjectsById(HWindow window, GuidPredCB pred);

		// Remove all objects 'window'
		[DllImport(Dll)] private static extern void View3D_WindowRemoveAllObjects(HWindow window);

		// Enumerate the sources associated with 'window'
		[DllImport(Dll)] private static extern void View3D_WindowEnumGuids(HWindow window, EnumGuidsCB enum_guids_cb);

		// Enumerate the objects associated with 'window'
		[DllImport(Dll)] private static extern void View3D_WindowEnumObjects(HWindow window, EnumObjectsCB enum_objects_cb);
		[DllImport(Dll)] private static extern void View3D_WindowEnumObjectsById(HWindow window, EnumObjectsCB enum_objects_cb, GuidPredCB pred);

		// Return true if 'object' is among 'window's objects
		[DllImport(Dll)] private static extern bool View3D_WindowHasObject(HWindow window, HObject obj, bool search_children);

		// Return the number of objects assigned to 'window'
		[DllImport(Dll)] private static extern int View3D_WindowObjectCount(HWindow window);

		// Return the bounds of a scene
		[DllImport(Dll)] private static extern BBox View3D_WindowSceneBounds(HWindow window, ESceneBounds bounds, int except_count, [MarshalAs(UnmanagedType.LPArray)] Guid[]? except);

		// Render the window
		[DllImport(Dll)] private static extern void View3D_WindowRender(HWindow window);

		// Wait for any previous frames to complete rendering within the GPU
		[DllImport(Dll)] private static extern void View3D_WindowGSyncWait(HWindow window);

		// Replace the swap chain buffers with 'targets'
		[DllImport(Dll)] private static extern void View3D_WindowCustomSwapChain(HWindow window, int count, [MarshalAs(UnmanagedType.LPArray)] HTexture[] targets);

		// Get the MSAA back buffer (render target + depth stencil)
		[DllImport(Dll)] private static extern BackBuffer View3D_WindowRenderTargetGet(HWindow window);

		// Signal the window is invalidated. This does not automatically trigger rendering. Use InvalidatedCB.
		[DllImport(Dll)] private static extern void View3D_WindowInvalidate(HWindow window, bool erase);
		[DllImport(Dll)] private static extern void View3D_WindowInvalidateRect(HWindow window, ref Win32.RECT rect, bool erase);

		// Register a callback for when the window is invalidated. This can be used to render in response to invalidation, rather than rendering on a polling cycle.
		[DllImport(Dll)] private static extern void View3D_WindowInvalidatedCB(HWindow window, InvalidatedCB invalidated_cb, bool add);

		// Clear the 'invalidated' state of the window.
		[DllImport(Dll)] private static extern void View3D_WindowValidate(HWindow window);

		// Get/Set the window background colour
		[DllImport(Dll)] private static extern uint View3D_WindowBackgroundColourGet(HWindow window);
		[DllImport(Dll)] private static extern void View3D_WindowBackgroundColourSet(HWindow window, uint argb);

		// Get/Set the fill mode for the window
		[DllImport(Dll)] private static extern EFillMode View3D_WindowFillModeGet(HWindow window);
		[DllImport(Dll)] private static extern void View3D_WindowFillModeSet(HWindow window, EFillMode mode);

		// Get/Set the cull mode for a faces in window
		[DllImport(Dll)] private static extern ECullMode View3D_WindowCullModeGet(HWindow window);
		[DllImport(Dll)] private static extern void View3D_WindowCullModeSet(HWindow window, ECullMode mode);

		// Get/Set the multi-sampling mode for a window
		[DllImport(Dll)] private static extern int View3D_MultiSamplingGet(HWindow window);
		[DllImport(Dll)] private static extern void View3D_MultiSamplingSet(HWindow window, int multisampling);

		// Control animation
		[DllImport(Dll)] private static extern void View3D_WindowAnimControl(HWindow window, EAnimCommand command, double time_s);

		// Set the callback for animation events
		[DllImport(Dll)] private static extern void View3D_WindowAnimEventCBSet(HWindow window, AnimationCB anim_cb, bool add);

		// Get/Set the animation time
		[DllImport(Dll)] private static extern bool View3D_WindowAnimating(HWindow window);
		[DllImport(Dll)] private static extern double View3D_WindowAnimTimeGet(HWindow window);
		[DllImport(Dll)] private static extern void View3D_WindowAnimTimeSet(HWindow window, double time_s);

		// Return the DPI of the monitor that 'window' is displayed on
		[DllImport(Dll)] private static extern v2 View3D_WindowDpiScale(HWindow window);

		// Set the global environment map for the window
		[DllImport(Dll)] private static extern void View3D_WindowEnvMapSet(HWindow window, CubeMap env_map);

		// Enable/Disable the depth buffer
		[DllImport(Dll)] private static extern bool View3D_DepthBufferEnabledGet(HWindow window);
		[DllImport(Dll)] private static extern void View3D_DepthBufferEnabledSet(HWindow window, bool enabled);

		// Cast a ray into the scene, returning information about what it hit.
		[DllImport(Dll)] private static extern void View3D_WindowHitTestObjects(HWindow window, IntPtr rays, IntPtr hits, int ray_count, [MarshalAs(UnmanagedType.LPArray, SizeParamIndex = 7)] HObject[] objects, int object_count);
		[DllImport(Dll)] private static extern void View3D_WindowHitTestByCtx(HWindow window, IntPtr rays, IntPtr hits, int ray_count, GuidPredCB pred);

		// Trigger execution of the async hit test rays. Submits GPU work and returns immediately.
		// Results are reported via the callback registered with View3D_HitTestAsyncCBSet.
		[DllImport(Dll)] private static extern void View3D_HitTestAsync(HWindow window);

		// Add/Remove a callback for receiving async hit test results
		[DllImport(Dll)] private static extern void View3D_HitTestAsyncCBSet(HWindow window, HitTestAsyncCB cb, bool add);

		// Add/Update/Remove an async hit test ray.
		// Returns 'HitTestRayId::None' if no more rays can be added.
		// Returns 'id' if the ray was updated/removed successfully, otherwise returns HitTestRayId::None.
		// Use 'ray == nullptr' to remove a ray.
		[DllImport(Dll)] private static extern HitTestRayId View3D_HitTestRayUpdate(HWindow window, HitTestRayId id, ref HitTestRay ray);
		[DllImport(Dll)] private static extern HitTestRayId View3D_HitTestRayUpdate(HWindow window, HitTestRayId id, IntPtr ray);

		// Camera *********************************

		// Position the camera and focus distance
		[DllImport(Dll)] private static extern void View3D_CameraPositionSet(HWindow window, v4 position, v4 lookat, v4 up);

		// Get/Set the current camera to world transform
		[DllImport(Dll)] private static extern m4x4 View3D_CameraToWorldGet(HWindow window);
		[DllImport(Dll)] private static extern void View3D_CameraToWorldSet(HWindow window, ref m4x4 c2w);

		// Move the camera to a position that can see the whole scene. Set 'dist' to 0 to preserve the FoV, or a distance to set the FoV
		[DllImport(Dll)] private static extern void View3D_ResetView(HWindow window, v4 forward, v4 up, float dist, bool preserve_aspect, bool commit);

		// Reset the camera to view a bbox. Set 'dist' to 0 to preserve the FoV, or a distance to set the FoV
		[DllImport(Dll)] private static extern void View3D_ResetViewBBox(HWindow window, BBox bbox, v4 forward, v4 up, float dist, bool preserve_aspect, bool commit);

		// Enable/Disable orthographic projection
		[DllImport(Dll)] private static extern bool View3D_CameraOrthographicGet(HWindow window);
		[DllImport(Dll)] private static extern void View3D_CameraOrthographicSet(HWindow window, bool on);

		// Get/Set the distance to the camera focus point
		[DllImport(Dll)] private static extern float View3D_CameraFocusDistanceGet(HWindow window);
		[DllImport(Dll)] private static extern void View3D_CameraFocusDistanceSet(HWindow window, float dist);

		// Get/Set the camera focus point position
		[DllImport(Dll)] private static extern v4 View3D_CameraFocusPointGet(HWindow window);
		[DllImport(Dll)] private static extern void View3D_CameraFocusPointSet(HWindow window, v4 position);

		// Get/Set bounds on the camera focus point position
		[DllImport(Dll)] private static extern BBox View3D_CameraFocusBoundsGet(HWindow window);
		[DllImport(Dll)] private static extern void View3D_CameraFocusBoundsSet(HWindow window, BBox bounds);

		// Get/Set the aspect ratio for the camera field of view
		[DllImport(Dll)] private static extern float View3D_CameraAspectGet(HWindow window);
		[DllImport(Dll)] private static extern void View3D_CameraAspectSet(HWindow window, float aspect);

		// Get/Set both the X and Y fields of view (i.e. set the aspect ratio). Null fov means don't change the current value.
		[DllImport(Dll)] private static extern v2 View3D_CameraFovGet(HWindow window);
		[DllImport(Dll)] private static extern void View3D_CameraFovSet(HWindow window, v2 fov);

		// Adjust the FocusDist, FovX, and FovY so that the average FOV equals 'fov'
		[DllImport(Dll)] private static extern void View3D_CameraBalanceFov(HWindow window, float fov);

		// Get/Set (using fov and focus distance) the size of the perpendicular area visible to the camera at 'dist' (in world space). Use 'focus_dist != 0' to set a specific focus distance
		[DllImport(Dll)] private static extern v2 View3D_CameraViewRectAtDistanceGet(HWindow window, float dist);
		[DllImport(Dll)] private static extern void View3D_CameraViewRectAtDistanceSet(HWindow window, v2 rect, float focus_dist);

		// Get/Set the near and far clip planes for the camera
		[DllImport(Dll)] private static extern v2 View3D_CameraClipPlanesGet(HWindow window, EClipPlanes flags);
		[DllImport(Dll)] private static extern void View3D_CameraClipPlanesSet(HWindow window, float near, float far, EClipPlanes flags);

		// Get/Set the scene camera lock mask
		[DllImport(Dll)] private static extern ECameraLockMask View3D_CameraLockMaskGet(HWindow window);
		[DllImport(Dll)] private static extern void View3D_CameraLockMaskSet(HWindow window, ECameraLockMask mask);
	
		// Get/Set the camera align axis
		[DllImport(Dll)] private static extern v4 View3D_CameraAlignAxisGet(HWindow window);
		[DllImport(Dll)] private static extern void View3D_CameraAlignAxisSet(HWindow window, v4 axis);
	
		// Reset to the default zoom
		[DllImport(Dll)] private static extern void View3D_CameraResetZoom(HWindow window);

		// Get/Set the FOV zoom
		[DllImport(Dll)] private static extern float View3D_CameraZoomGet(HWindow window);
		[DllImport(Dll)] private static extern void View3D_CameraZoomSet(HWindow window, float zoom);

		// Commit the current O2W position as the reference position
		[DllImport(Dll)] private static extern void View3D_CameraCommit(HWindow window);

		// Navigation *****************************

		// Direct movement of the camera
		[DllImport(Dll)] private static extern bool View3D_Navigate(HWindow window, float dx, float dy, float dz);

		// Move the scene camera using the mouse
		[DllImport(Dll)] private static extern bool View3D_MouseNavigate(HWindow window, v2 ss_point, ENavOp nav_op, bool nav_start_or_end);
		[DllImport(Dll)] private static extern bool View3D_MouseNavigateZ(HWindow window, v2 ss_point, float delta, bool along_ray);

		// Convert an MK_ macro to a default navigation operation
		[DllImport(Dll)] private static extern ENavOp View3D_MouseBtnToNavOp(EMouseBtns mk);

		// Convert lengths in pixels (X and Y) into lengths in normalised screen space
		[DllImport(Dll)] private static extern v2 View3D_PixelsToNSS(HWindow window, v2 pixels);

		// Convert a point between 'window' screen space and normalised screen space
		[DllImport(Dll)] private static extern v2 View3D_SSPointToNSSPoint(HWindow window, v2 screen);
		[DllImport(Dll)] private static extern v2 View3D_NSSPointToSSPoint(HWindow window, v2 nss_point);

		// Convert a point between world space and normalised screen space.
		// The x,y components of 'screen' should be in normalised screen space, i.e. (-1,-1)->(1,1)
		// The z component should be the world space distance from the camera
		[DllImport(Dll)] private static extern v4 View3D_NSSPointToWSPoint(HWindow window, v4 screen);
		[DllImport(Dll)] private static extern v4 View3D_WSPointToNSSPoint(HWindow window, v4 world);

		// Return a point and direction in world space corresponding to a normalised screen space point.
		// The x,y components of 'screen' should be in normalised screen space, i.e. (-1,-1)->(1,1)
		// The z component should be the world space distance from the camera
		[DllImport(Dll)] private static extern void View3D_NSSPointToWSRay(HWindow window, v4 screen, out v4 ws_point, out v4 ws_direction);

		// Lights *********************************

		// Get/Set the properties of the global light
		[DllImport(Dll)] private static extern LightInfo View3D_LightPropertiesGet(HWindow window);
		[DllImport(Dll)] private static extern void View3D_LightPropertiesSet(HWindow window, ref LightInfo light);
	
		// Set the global light source for a window
		[DllImport(Dll)] private static extern void View3D_LightSource(HWindow window, v4 position, v4 direction, bool camera_relative);

		// Objects ********************************

		// Notes:
		// 'name' parameter on object get/set functions:
		//   If 'name' is null, then the state of the root object is returned
		//   If 'name' begins with '#' then the remainder of the name is treated as a regular expression

		// Create an object from provided buffers
		[DllImport(Dll, CharSet = CharSet.Ansi)] private static extern HObject View3D_ObjectCreate([MarshalAs(UnmanagedType.LPStr)] string name, uint colour, int vcount, int icount, int ncount, IntPtr verts, IntPtr indices, IntPtr nuggets, ref Guid context_id);

		// Create an graphics object from ldraw script, either a string or a file 
		[DllImport(Dll, CharSet = CharSet.Ansi)] private static extern HObject View3D_ObjectCreateLdrA([MarshalAs(UnmanagedType.LPStr)] string ldr_script, bool file, ref Guid context_id, ref Includes includes);
		[DllImport(Dll, CharSet = CharSet.Unicode)] private static extern HObject View3D_ObjectCreateLdrW([MarshalAs(UnmanagedType.LPWStr)] string ldr_script, bool file, ref Guid context_id, ref Includes includes);

		// Load a p3d model file as a view3d object
		[DllImport(Dll, CharSet = CharSet.Ansi)] private static extern HObject View3D_ObjectCreateP3DFile([MarshalAs(UnmanagedType.LPStr)] string name, uint colour, [MarshalAs(UnmanagedType.LPStr)] string p3d_filepath, ref Guid context_id);

		// Load a p3d model in memory as a view3d object
		[DllImport(Dll, CharSet = CharSet.Ansi)] private static extern HObject View3D_ObjectCreateP3DStream([MarshalAs(UnmanagedType.LPStr)] string name, uint colour, int size, IntPtr p3d_data, ref Guid context_id);

		// Create an ldr object using a callback to populate the model data.
		[DllImport(Dll, CharSet = CharSet.Ansi)] private static extern HObject View3D_ObjectCreateWithCallback([MarshalAs(UnmanagedType.LPStr)] string name, uint colour, int vcount, int icount, int ncount, EditObjectCBInternal edit_cb, ref Guid context_id);
		[DllImport(Dll)] private static extern void View3D_ObjectEdit(HObject obj, EditObjectCBInternal edit_cb);

		// Replace the model and all child objects of 'obj' with the results of 'ldr_script'
		[DllImport(Dll)] private static extern void View3D_ObjectUpdate(HObject obj, [MarshalAs(UnmanagedType.LPWStr)] string ldr_script, EUpdateObject flags);

		// Delete an object, freeing its resources
		[DllImport(Dll)] private static extern void View3D_ObjectDelete(HObject obj);

		// Create an instance of 'obj'
		[DllImport(Dll)] private static extern HObject View3D_ObjectCreateInstance(HObject obj);

		// Return the context id that this object belongs to
		[DllImport(Dll)] private static extern Guid View3D_ObjectContextIdGet(HObject obj);

		// Return the root object of 'object'(possibly itself)
		[DllImport(Dll)] private static extern HObject View3D_ObjectGetRoot(HObject obj);

		// Return the immediate parent of 'object'
		[DllImport(Dll)] private static extern HObject View3D_ObjectGetParent(HObject obj);

		// Return a child object of 'object'
		[DllImport(Dll, CharSet = CharSet.Ansi)] private static extern HObject View3D_ObjectGetChildByName(HObject obj, [MarshalAs(UnmanagedType.LPStr)] string name);
		[DllImport(Dll)] private static extern HObject View3D_ObjectGetChildByIndex(HObject obj, int index);

		// Return the number of child objects of 'object'
		[DllImport(Dll)] private static extern int View3D_ObjectChildCount(HObject obj);

		// Enumerate the child objects of 'object'. (Not recursive)
		[DllImport(Dll)] private static extern void View3D_ObjectEnumChildren(HObject obj, EnumObjectsCB enum_objects_cb);

		// Get/Set the name of 'object'
		[DllImport(Dll, CharSet = CharSet.Unicode)][return: MarshalAs(UnmanagedType.BStr)] private static extern string View3D_ObjectNameGetBStr(HObject obj);
		[DllImport(Dll, CharSet = CharSet.Ansi)] private static extern void View3D_ObjectNameSet(HObject obj, [MarshalAs(UnmanagedType.LPStr)] string name);

		// Get the type of 'object'
		[DllImport(Dll, CharSet = CharSet.Unicode)][return: MarshalAs(UnmanagedType.BStr)] private static extern string View3D_ObjectTypeGetBStr(HObject obj);

		// Get/Set the current or base colour of an object(the first object to match 'name') (See LdrObject::Apply)
		[DllImport(Dll, CharSet = CharSet.Ansi)] private static extern uint View3D_ObjectColourGet(HObject obj, bool base_colour, [MarshalAs(UnmanagedType.LPStr)] string? name);
		[DllImport(Dll, CharSet = CharSet.Ansi)] private static extern void View3D_ObjectColourSet(HObject obj, bool base_colour, uint colour, [MarshalAs(UnmanagedType.LPStr)] string? name, EColourOp op, float op_value);

		// Reset the object colour back to its default
		[DllImport(Dll, CharSet = CharSet.Ansi)] private static extern void View3D_ObjectResetColour(HObject obj, [MarshalAs(UnmanagedType.LPStr)] string? name);

		// Get/Set the object's o2w transform
		[DllImport(Dll, CharSet = CharSet.Ansi)] private static extern m4x4 View3D_ObjectO2WGet(HObject obj, [MarshalAs(UnmanagedType.LPStr)] string? name);
		[DllImport(Dll, CharSet = CharSet.Ansi)] private static extern void View3D_ObjectO2WSet(HObject obj, ref m4x4 o2w, [MarshalAs(UnmanagedType.LPStr)] string? name);

		// Get/Set the object to parent transform for an object.
		// This is the object to world transform for objects without parents.
		// Note: In "*Box b { 1 1 1 *o2w{*pos{1 2 3}} }" setting this transform overwrites the "*o2w{*pos{1 2 3}}".
		[DllImport(Dll, CharSet = CharSet.Ansi)] private static extern m4x4 View3D_ObjectO2PGet(HObject obj, [MarshalAs(UnmanagedType.LPStr)] string? name);
		[DllImport(Dll, CharSet = CharSet.Ansi)] private static extern void View3D_ObjectO2PSet(HObject obj, ref m4x4 o2p, [MarshalAs(UnmanagedType.LPStr)] string? name);

		// Return the model space bounding box for 'object'
		[DllImport(Dll)] private static extern BBox View3D_ObjectBBoxMS(HObject obj, EBBoxFlags bbox_flags);

		// Get/Set the object visibility. See LdrObject::Apply for docs on the format of 'name'
		[DllImport(Dll, CharSet = CharSet.Ansi)] private static extern bool View3D_ObjectVisibilityGet(HObject obj, [MarshalAs(UnmanagedType.LPStr)] string? name);
		[DllImport(Dll, CharSet = CharSet.Ansi)] private static extern void View3D_ObjectVisibilitySet(HObject obj, bool visible, [MarshalAs(UnmanagedType.LPStr)] string? name);

		// Get/Set wireframe mode for an object (the first object to match 'name'). (See LdrObject::Apply)
		[DllImport(Dll, CharSet = CharSet.Ansi)] private static extern bool View3D_ObjectWireframeGet(HObject obj, [MarshalAs(UnmanagedType.LPStr)] string? name);
		[DllImport(Dll, CharSet = CharSet.Ansi)] private static extern void View3D_ObjectWireframeSet(HObject obj, bool wireframe, [MarshalAs(UnmanagedType.LPStr)] string? name);

		// Get/Set the object flags. See LdrObject::Apply for docs on the format of 'name'
		[DllImport(Dll, CharSet = CharSet.Ansi)] private static extern ELdrFlags View3D_ObjectFlagsGet(HObject obj, [MarshalAs(UnmanagedType.LPStr)] string? name);
		[DllImport(Dll, CharSet = CharSet.Ansi)] private static extern void View3D_ObjectFlagsSet(HObject obj, ELdrFlags flags, bool state, [MarshalAs(UnmanagedType.LPStr)] string? name);

		// Get/Set the reflectivity of an object (the first object to match 'name') (See LdrObject::Apply)
		[DllImport(Dll, CharSet = CharSet.Ansi)] private static extern float View3D_ObjectReflectivityGet(HObject obj, [MarshalAs(UnmanagedType.LPStr)] string? name);
		[DllImport(Dll, CharSet = CharSet.Ansi)] private static extern void View3D_ObjectReflectivitySet(HObject obj, float reflectivity, [MarshalAs(UnmanagedType.LPStr)] string? name);

		// Get/Set the sort group for the object or its children. (See LdrObject::Apply)
		[DllImport(Dll, CharSet = CharSet.Ansi)] private static extern ESortGroup View3D_ObjectSortGroupGet(HObject obj, [MarshalAs(UnmanagedType.LPStr)] string? name);
		[DllImport(Dll, CharSet = CharSet.Ansi)] private static extern void View3D_ObjectSortGroupSet(HObject obj, ESortGroup group, [MarshalAs(UnmanagedType.LPStr)] string? name);

		// Get/Set 'show normals' mode for an object (the first object to match 'name') (See LdrObject::Apply)
		[DllImport(Dll, CharSet = CharSet.Ansi)] private static extern bool View3D_ObjectNormalsGet(HObject obj, [MarshalAs(UnmanagedType.LPStr)] string? name);
		[DllImport(Dll, CharSet = CharSet.Ansi)] private static extern void View3D_ObjectNormalsSet(HObject obj, bool show, [MarshalAs(UnmanagedType.LPStr)] string? name);

		// Set the texture/sampler for all nuggets of 'object' or its children. (See LdrObject::Apply)
		[DllImport(Dll, CharSet = CharSet.Ansi)] private static extern void View3D_ObjectSetTexture(HObject obj, HTexture tex, [MarshalAs(UnmanagedType.LPStr)] string? name);
		[DllImport(Dll, CharSet = CharSet.Ansi)] private static extern void View3D_ObjectSetSampler(HObject obj, HSampler sam, [MarshalAs(UnmanagedType.LPStr)] string? name);

		// Get/Set the nugget flags on an object or its children (See LdrObject::Apply)
		[DllImport(Dll, CharSet = CharSet.Ansi)] private static extern ENuggetFlag View3D_ObjectNuggetFlagsGet(HObject obj, [MarshalAs(UnmanagedType.LPStr)] string? name, int index);
		[DllImport(Dll, CharSet = CharSet.Ansi)] private static extern void View3D_ObjectNuggetFlagsSet(HObject obj, ENuggetFlag flags, bool state, [MarshalAs(UnmanagedType.LPStr)] string? name, int index);

		// Get/Set the tint colour for a nugget within the model of an object or its children. (See LdrObject::Apply)
		[DllImport(Dll, CharSet = CharSet.Ansi)] private static extern Colour32 View3D_ObjectNuggetTintGet(HObject obj, [MarshalAs(UnmanagedType.LPStr)] string? name, int index);
		[DllImport(Dll, CharSet = CharSet.Ansi)] private static extern void View3D_ObjectNuggetTintSet(HObject obj, Colour32 colour, [MarshalAs(UnmanagedType.LPStr)] string? name, int index);

		// Materials ******************************

		// Create a texture from data in memory.
		// Set 'data' to nullptr to leave the texture uninitialised, if not null then data must point to width x height pixel data
		// of the size appropriate for the given format. 'e.g. uint32_t px_data[width * height] for D3DFMT_A8R8G8B8'
		// Note: careful with stride, 'data' is expected to have the appropriate stride for BytesPerPixel(format) * width
		[DllImport(Dll)] private static extern HTexture View3D_TextureCreate(int width, int height, IntPtr data, int data_size, ref TextureOptions options);

		// Create one of the stock textures
		[DllImport(Dll)] private static extern HTexture View3D_TextureCreateStock(EStockTexture stock_texture);

		// Load a texture from file, embedded resource, or stock assets. Specify width == 0, height == 0 to use the dimensions of the file
		[DllImport(Dll)] private static extern HTexture View3D_TextureCreateFromUri([MarshalAs(UnmanagedType.LPStr)] string resource, int width, int height, ref TextureOptions options);

		// Load a cube map from file, embedded resource, or stock assets. Specify width == 0, height == 0 to use the dimensions of the file
		[DllImport(Dll)] private static extern HCubeMap View3D_CubeMapCreateFromUri([MarshalAs(UnmanagedType.LPStr)] string resource, ref CubeMapOptions options);

		// Create a texture sampler
		[DllImport(Dll)] private static extern HSampler View3D_SamplerCreate(ref SamplerOptions options);

		// Create one of the stock samplers
		[DllImport(Dll)] private static extern HSampler View3D_SamplerCreateStock(EStockSampler stock_sampler);

		// Create a shader
		[DllImport(Dll)] private static extern HSampler View3D_ShaderCreate(ref ShaderOptions options);

		// Create one of the stock shaders
		[DllImport(Dll, CharSet = CharSet.Ansi)] private static extern HSampler View3D_ShaderCreateStock(EStockShader stock_shader, [MarshalAs(UnmanagedType.LPStr)] string config);

		// Read the properties of an existing texture
		[DllImport(Dll)] private static extern ImageInfo View3D_TextureGetInfo(HTexture tex);
		[DllImport(Dll)] private static extern EResult View3D_TextureGetInfoFromFile([MarshalAs(UnmanagedType.LPStr)] string tex_filepath, out ImageInfo info);

		// Release a reference to a resources
		[DllImport(Dll)] private static extern void View3D_TextureRelease(HTexture tex);
		[DllImport(Dll)] private static extern void View3D_CubeMapRelease(HCubeMap tex);
		[DllImport(Dll)] private static extern void View3D_SamplerRelease(HSampler sam);
		[DllImport(Dll)] private static extern void View3D_ShaderRelease(HShader shdr);

		// Resize this texture to 'size'
		[DllImport(Dll)] private static extern void View3D_TextureResize(HTexture tex, ulong width, uint height, ushort depth_or_array_len);

		// Return the ref count of 'tex'
		[DllImport(Dll)] private static extern ulong View3D_TextureRefCount(HTexture tex);

		// Get/Set the private data associated with 'guid' for 'tex'
		[DllImport(Dll)] private static extern void View3d_TexturePrivateDataGet(HTexture tex, Guid guid, ref uint size, IntPtr data);
		[DllImport(Dll)] private static extern void View3d_TexturePrivateDataSet(HTexture tex, Guid guid, uint size, IntPtr data);
		[DllImport(Dll)] private static extern void View3d_TexturePrivateDataIFSet(HTexture tex, Guid guid, IntPtr pointer);

		// Resolve a MSAA texture into a non-MSAA texture
		[DllImport(Dll)] private static extern void View3D_TextureResolveAA(HTexture dst, HTexture src);

		// Gizmos *********************************

		// Create the 3D manipulation gizmo
		[DllImport(Dll)] private static extern HGizmo View3D_GizmoCreate(Gizmo.EMode mode, ref m4x4 o2w);

		// Delete a 3D manipulation gizmo
		[DllImport(Dll)] private static extern void View3D_GizmoDelete(HGizmo gizmo);

		// Attach/Detach callbacks that are called when the gizmo moves
		[DllImport(Dll)] private static extern void View3D_GizmoMovedCBSet(HGizmo gizmo, GizmoMovedCB moved_cb, bool add);

		// Attach/Detach an object to the gizmo that will be moved as the gizmo moves
		[DllImport(Dll)] private static extern void View3D_GizmoAttach(HGizmo gizmo, HObject obj);
		[DllImport(Dll)] private static extern void View3D_GizmoDetach(HGizmo gizmo, HObject obj);

		// Get/Set the scale factor for the gizmo
		[DllImport(Dll)] private static extern float View3D_GizmoScaleGet(HGizmo gizmo);
		[DllImport(Dll)] private static extern void View3D_GizmoScaleSet(HGizmo gizmo, float scale);

		// Get/Set the current mode of the gizmo
		[DllImport(Dll)] private static extern Gizmo.EMode View3D_GizmoModeGet(HGizmo gizmo);
		[DllImport(Dll)] private static extern void View3D_GizmoModeSet(HGizmo gizmo, Gizmo.EMode mode);

		// Get/Set the object to world transform for the gizmo
		[DllImport(Dll)] private static extern m4x4 View3D_GizmoO2WGet(HGizmo gizmo);
		[DllImport(Dll)] private static extern void View3D_GizmoO2WSet(HGizmo gizmo, ref m4x4 o2w);

		// Get the offset transform that represents the difference between the gizmo's transform at the start of manipulation and now.
		[DllImport(Dll)] private static extern m4x4 View3D_GizmoOffsetGet(HGizmo gizmo);

		// Get/Set whether the gizmo is active to mouse interaction
		[DllImport(Dll)] private static extern bool View3D_GizmoEnabledGet(HGizmo gizmo);
		[DllImport(Dll)] private static extern void View3D_GizmoEnabledSet(HGizmo gizmo, bool enabled);

		// Returns true while manipulation is in progress
		[DllImport(Dll)] private static extern bool View3D_GizmoManipulating(HGizmo gizmo);

		// Diagnostics ****************************

		// Get/Set whether object bounding boxes are visible
		[DllImport(Dll)] private static extern bool View3D_DiagBBoxesVisibleGet(HWindow window);
		[DllImport(Dll)] private static extern void View3D_DiagBBoxesVisibleSet(HWindow window, bool visible);

		// Get/Set the length of the vertex normals
		[DllImport(Dll)] private static extern float View3D_DiagNormalsLengthGet(HWindow window);
		[DllImport(Dll)] private static extern void View3D_DiagNormalsLengthSet(HWindow window, float length);

		// Get/Set the colour of the vertex normals
		[DllImport(Dll)] private static extern Colour32 View3D_DiagNormalsColourGet(HWindow window);
		[DllImport(Dll)] private static extern void View3D_DiagNormalsColourSet(HWindow window, Colour32 colour);

		[DllImport(Dll)] private static extern v2 View3D_DiagFillModePointsSizeGet(HWindow window);
		[DllImport(Dll)] private static extern void View3D_DiagFillModePointsSizeSet(HWindow window, v2 size);

		// Miscellaneous **************************
		
		// Create a render target texture on a D3D9 device. Intended for WPF D3DImage
		[DllImport(Dll)] private static extern HTexture View3D_CreateDx9RenderTarget(HWND hwnd, uint width, uint height, ref TextureOptions options, out IntPtr shared_handle);

		// Create a Texture instance from a shared d3d resource (created on a different d3d device)
		[DllImport(Dll)] private static extern HTexture View3D_CreateTextureFromSharedResource(IntPtr shared_resource, ref TextureOptions options);

		// Return the supported MSAA quality for the given multi-sampling count
		[DllImport(Dll)] private static extern int View3D_MSAAQuality(int count, EFormat format);

		// Return true if the focus point is visible. Add/Remove the focus point to a window.
		[DllImport(Dll)] private static extern bool View3D_StockObjectVisibleGet(HWindow window, EStockObject stock_object);
		[DllImport(Dll)] private static extern void View3D_StockObjectVisibleSet(HWindow window, EStockObject stock_object, bool show);

		// Get/Set the size of the focus point
		[DllImport(Dll)] private static extern float View3D_FocusPointSizeGet(HWindow window);
		[DllImport(Dll)] private static extern void View3D_FocusPointSizeSet(HWindow window, float size);
		
		// Get/Set the size of the origin point
		[DllImport(Dll)] private static extern float View3D_OriginPointSizeGet(HWindow window);
		[DllImport(Dll)] private static extern void View3D_OriginPointSizeSet(HWindow window, float size);

		// Get/Set the position and size of the selection box
		[DllImport(Dll)] private static extern void View3D_SelectionBoxGet(HWindow window, out BBox box, out m4x4 o2w);
		[DllImport(Dll)] private static extern void View3D_SelectionBoxSet(HWindow window, ref BBox box, ref m4x4 o2w);

		// Set the selection box to encompass all selected objects
		[DllImport(Dll)] private static extern void View3D_SelectionBoxFitToSelected(HWindow window);

		// Create/Delete the demo scene in the given window
		[DllImport(Dll)] private static extern Guid View3D_DemoSceneCreateText(HWindow window);
		[DllImport(Dll)] private static extern Guid View3D_DemoSceneCreateBinary(HWindow window);
		[DllImport(Dll)] private static extern void View3D_DemoSceneDelete();
		
		// Show a window containing the demo script
		[DllImport(Dll)] private static extern void View3D_DemoScriptShow(HWindow window);

		// Return the example Ldr script as a BSTR
		[DllImport(Dll, CharSet = CharSet.Unicode)][return: MarshalAs(UnmanagedType.BStr)] private static extern string View3D_ExampleScriptBStr();

		// Return the auto complete templates as a BSTR
		[DllImport(Dll, CharSet = CharSet.Unicode)][return: MarshalAs(UnmanagedType.BStr)] private static extern string View3D_AutoCompleteTemplatesBStr();

		// Return the hierarchy "address" for a position in an ldr script file.
		[DllImport(Dll, CharSet = CharSet.Unicode)][return: MarshalAs(UnmanagedType.BStr)] private static extern string View3D_ObjectAddressAt([MarshalAs(UnmanagedType.LPWStr)] string ldr_script, long position);

		// Parse a transform description using the Ldr script syntax
		[DllImport(Dll, CharSet = CharSet.Ansi)] private static extern m4x4 View3D_ParseLdrTransform([MarshalAs(UnmanagedType.LPStr)] string ldr_script);

		// Handle standard keyboard shortcuts. 'key_code' should be a standard VK_ key code with modifiers included in the hi word. See 'EKeyCodes'
		[DllImport(Dll)] private static extern bool View3D_TranslateKey(HWindow window, EKeyCodes key_code, v2 ss_point);

		// Return the reference count of a COM interface
		[DllImport(Dll)] private static extern ulong View3D_RefCount(IntPtr pointer);

		// Tools **********************************

		// Show/Hide the object manager tool
		[DllImport(Dll)] private static extern bool View3D_ObjectManagerVisibleGet(HWindow window);
		[DllImport(Dll)] private static extern void View3D_ObjectManagerVisibleSet(HWindow window, bool show);

		// Show/Hide the script editor tool
		[DllImport(Dll)] private static extern bool View3D_ScriptEditorVisibleGet(HWindow window);
		[DllImport(Dll)] private static extern void View3D_ScriptEditorVisibleSet(HWindow window, bool show);

		// Show/Hide the measurement tool
		[DllImport(Dll)] private static extern bool View3D_MeasureToolVisibleGet(HWindow window);
		[DllImport(Dll)] private static extern void View3D_MeasureToolVisibleSet(HWindow window, bool show);

		// Show/Hide the angle measurement tool
		[DllImport(Dll)] private static extern bool View3D_AngleToolVisibleGet(HWindow window);
		[DllImport(Dll)] private static extern void View3D_AngleToolVisibleSet(HWindow window, bool show);

		// Show/Hide the lighting controls UI
		[DllImport(Dll)] private static extern void View3D_LightingControlsUI(HWindow window);

#if false

		// Camera
		[DllImport(Dll)]
		private static extern void View3D_CameraViewRectSet(HWindow window, float width, float height, float dist);
		[DllImport(Dll)]
		private static extern v2 View3D_ViewArea(HWindow window, float dist);

		// Materials
		[DllImport(Dll)]
		private static extern void View3D_TextureLoadSurface(HTexture tex, int level, [MarshalAs(UnmanagedType.LPWStr)] string tex_filepath, Rectangle[]? dst_rect, Rectangle[]? src_rect, EFilter filter, uint colour_key);
		[DllImport(Dll)]
		private static extern IntPtr View3D_TextureGetDC(HTexture tex, bool discard);
		[DllImport(Dll)]
		private static extern void View3D_TextureReleaseDC(HTexture tex);

		// Miscellaneous
		[DllImport(Dll)]
		private static extern void View3D_Flush();

		[DllImport(Dll)]
		private static extern HWND View3D_LdrEditorCreate(HWND parent);
		[DllImport(Dll)]
		private static extern void View3D_LdrEditorDestroy(HWND hwnd);
		[DllImport(Dll)]
		private static extern void View3D_LdrEditorCtrlInit(HWND scintilla_control, bool dark);
#endif
#endregion
	}
}
