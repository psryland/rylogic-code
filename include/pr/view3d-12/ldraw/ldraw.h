//********************************
// View3d
//  Copyright (c) Rylogic Ltd 2014
//********************************
#pragma once
#include "pr/view3d-12/forward.h"

namespace pr::rdr12::ldraw
{
	// Compile time hash
	inline constexpr int HashI(std::string_view str)
	{
		constexpr uint32_t FNV_offset_basis32 = 2166136261U;
		constexpr uint32_t FNV_prime32 = 16777619U;

		// Copied from pr/common/hash
		constexpr auto Mul32 = [](uint32_t a, uint32_t b) -> uint32_t
		{
			return uint32_t((uint64_t(a) * uint64_t(b)) & ~0U);
		};
		constexpr auto Lower = [](char c) -> char
		{
			return (c >= 'A' && c <= 'Z') ? c + ('a' - 'A') : c;
		};
		constexpr auto Hash32CT = [](uint32_t ch, uint32_t h) -> uint32_t
		{
			return Mul32(h ^ ch, FNV_prime32);
		};
		constexpr auto HashI32CT = [](std::string_view str, uint32_t h) -> uint32_t
		{
			auto b = str.data();
			auto e = b + str.size();
			for (; b != e; ++b) h = Hash32CT(static_cast<uint32_t>(Lower(*b)), h);
			return h;
		};
		return static_cast<int>(HashI32CT(str, FNV_offset_basis32));
	}

	// Keywords in ldraw script. This includes object types and field names because they need to have unique hashes
	enum class EKeyword : int
	{
		#define PR_LDRAW_KEYWORDS(x)\
		x(Accel                    )\
		x(Addr                     )\
		x(Align                    )\
		x(Alpha                    )\
		x(Ambient                  )\
		x(Anchor                   )\
		x(AngAccel                 )\
		x(AngVelocity              )\
		x(Animation                )\
		x(AnimSource               )\
		x(Arrow                    )\
		x(Aspect                   )\
		x(Axis                     )\
		x(AxisId                   )\
		x(BackColour               )\
		x(BakeTransform            )\
		x(BaseColour               )\
		x(BaseColourTexture        )\
		x(Billboard                )\
		x(Billboard3D              )\
		x(BinaryStream             )\
		x(Box                      )\
		x(BoxList                  )\
		x(CastShadow               )\
		x(Chart                    )\
		x(Circle                   )\
		x(Closed                   )\
		x(Colour                   )\
		x(Colours                  )\
		x(Commands                 )\
		x(Cone                     )\
		x(ConvexHull               )\
		x(CoordFrame               )\
		x(CornerRadius             )\
		x(CrossSection             )\
		x(CString                  )\
		x(Custom                   )\
		x(Cylinder                 )\
		x(Dashed                   )\
		x(Data                     )\
		x(DataPoints               )\
		x(Depth                    )\
		x(Diffuse                  )\
		x(Dim                      )\
		x(Direction                )\
		x(Divisions                )\
		x(Emissive                 )\
		x(EmissiveTexture          )\
		x(EndCaps                  )\
		x(Equation                 )\
		x(Euler                    )\
		x(Faces                    )\
		x(Facets                   )\
		x(Far                      )\
		x(FilePath                 )\
		x(Filter                   )\
		x(Font                     )\
		x(ForeColour               )\
		x(Format                   )\
		x(Fov                      )\
		x(FovX                     )\
		x(FovY                     )\
		x(Frame                    )\
		x(FrameRange               )\
		x(FrameRate                )\
		x(FrustumFA                )\
		x(FrustumWH                )\
		x(GenerateNormals          )\
		x(Grid                     )\
		x(Group                    )\
		x(GroupTint                )\
		x(Hidden                   )\
		x(HideWhenNotAnimating     )\
		x(Instance                 )\
		x(Intensity                )\
		x(Inverse                  )\
		x(Layers                   )\
		x(LeftHanded               )\
		x(LightSource              )\
		x(Line                     )\
		x(LineBox                  )\
		x(LineList                 )\
		x(Lines                    )\
		x(LineStrip                )\
		x(LookAt                   )\
		x(M3x3                     )\
		x(M4x4                     )\
		x(Material                 )\
		x(Mesh                     )\
		x(Metallic                 )\
		x(MetallicTexture          )\
		x(Model                    )\
		x(Montage                  )\
		x(Name                     )\
		x(Near                     )\
		x(NewLine                  )\
		x(NoMaterials              )\
		x(NonAffine                )\
		x(Normalise                )\
		x(Normals                  )\
		x(NormalTexture            )\
		x(NoRotation               )\
		x(NoTranslation            )\
		x(NoZTest                  )\
		x(NoZWrite                 )\
		x(O2W                      )\
		x(Orthographic             )\
		x(Orthonormalise           )\
		x(Padding                  )\
		x(Param                    )\
		x(Parametrics              )\
		x(Part                     )\
		x(Parts                    )\
		x(Period                   )\
		x(PerItemColour            )\
		x(PerItemParametrics       )\
		x(Pie                      )\
		x(Plane                    )\
		x(Point                    )\
		x(PointDepth               )\
		x(PointSize                )\
		x(PointStyle               )\
		x(Polygon                  )\
		x(Pos                      )\
		x(Position                 )\
		x(Quad                     )\
		x(Quat                     )\
		x(QuatPos                  )\
		x(Rand4x4                  )\
		x(RandColour               )\
		x(RandOri                  )\
		x(RandPos                  )\
		x(Range                    )\
		x(Rect                     )\
		x(Reflectivity             )\
		x(Resolution               )\
		x(Ribbon                   )\
		x(RootAnimation            )\
		x(Roughness                )\
		x(RoughnessTexture         )\
		x(Round                    )\
		x(Scale                    )\
		x(ScreenSpace              )\
		x(Series                   )\
		x(Size                     )\
		x(Smooth                   )\
		x(Solid                    )\
		x(Source                   )\
		x(Specular                 )\
		x(Sphere                   )\
		x(SphereList               )\
		x(Square                   )\
		x(Step                     )\
		x(Stretch                  )\
		x(Strikeout                )\
		x(Style                    )\
		x(Tetra                    )\
		x(TexCoords                )\
		x(Text                     )\
		x(TextLayout               )\
		x(TextStream               )\
		x(Texture                  )\
		x(TimeBias                 )\
		x(TimeRange                )\
		x(Transpose                )\
		x(Triangle                 )\
		x(TriList                  )\
		x(TriStrip                 )\
		x(Tube                     )\
		x(TwoSided                 )\
		x(Txfm                     )\
		x(Underline                )\
		x(Unknown                  )\
		x(Up                       )\
		x(Velocity                 )\
		x(Verts                    )\
		x(Video                    )\
		x(ViewPlaneZ               )\
		x(Wedges                   )\
		x(Weight                   )\
		x(Width                    )\
		x(Wireframe                )\
		x(XAxis                    )\
		x(XColumn                  )\
		x(YAxis                    )\
		x(ZAxis                    )
		// PR_LDRAW_KEYWORDS_END

		#define PR_LDRAW_ENUM_MEMBERS(name) name = HashI(#name),
		PR_LDRAW_KEYWORDS(PR_LDRAW_ENUM_MEMBERS)
		#undef PR_LDRAW_ENUM_MEMBERS
	};

	// An enum of just the object types
	enum class ELdrObject : int
	{
		#define PR_LDRAW_OBJECTS(x)\
		x(Box        )\
		x(BoxList    )\
		x(Chart      )\
		x(Circle     )\
		x(Cone       )\
		x(ConvexHull )\
		x(CoordFrame )\
		x(Custom     )\
		x(Cylinder   )\
		x(Equation   )\
		x(FrustumFA  )\
		x(FrustumWH  )\
		x(Grid       )\
		x(Group      )\
		x(Instance   )\
		x(LightSource)\
		x(Line       )\
		x(LineBox    )\
		x(Mesh       )\
		x(Model      )\
		x(Pie        )\
		x(Plane      )\
		x(Point      )\
		x(Polygon    )\
		x(Quad       )\
		x(Rect       )\
		x(Ribbon     )\
		x(Series     )\
		x(Sphere     )\
		x(SphereList )\
		x(Text       )\
		x(Triangle   )\
		x(Tube       )\
		x(Unknown    )
		// PR_LDRAW_OBJECTS_END
		
		#define PR_LDRAW_ENUM_MEMBERS(name) name = EKeyword::name,
		PR_LDRAW_OBJECTS(PR_LDRAW_ENUM_MEMBERS)
		#undef PR_LDRAW_ENUM_MEMBERS
	};

	// Ldraw script commands (for streaming)
	enum class ECommandId : int
	{
		// Notes: Commands apply to sources, the source has no knowledge of scenes that it's in.
		// However, the 'Render' command triggers a refresh of any scene that contains the source.
		#define PR_LDRAW_COMMANDS(x)\
		x(Invalid       )\
		x(Clear         ) /**/\
		x(ObjectToWorld ) /* <object-name> <o2w> */\
		x(ObjectColour  ) /* <object-name> <colour32> */\
		x(Render        ) /**/
		// PR_LDRAW_COMMANDS_END

		#define PR_LDRAW_ENUM_MEMBERS(name) name = HashI(#name),
		PR_LDRAW_COMMANDS(PR_LDRAW_ENUM_MEMBERS)
		#undef PR_LDRAW_ENUM_MEMBERS
	};

	// Point styles
	enum class EPointStyle : uint8_t
	{
		#define PR_LDRAW_POINT_STYLES(x)\
		x(Square)\
		x(Circle)\
		x(Triangle)\
		x(Star)\
		x(Annulus)

		#define PR_LDRAW_ENUM_MEMBERS(name) name,
		PR_LDRAW_POINT_STYLES(PR_LDRAW_ENUM_MEMBERS)
		#undef PR_LDRAW_ENUM_MEMBERS
	};

	// Line styles
	enum class ELineStyle : uint8_t
	{
		#define PR_LDRAW_LINE_STYLES(x)\
		x(LineSegments)\
		x(LineStrip)\
		x(Direction)\
		x(Bezier)\
		x(Hermite)\
		x(BSpline)\
		x(CatmullRom)
		
		#define PR_LDRAW_ENUM_MEMBERS(name) name,
		PR_LDRAW_LINE_STYLES(PR_LDRAW_ENUM_MEMBERS)
		#undef PR_LDRAW_ENUM_MEMBERS
	};

	// Arrow styles
	enum class EArrowType : uint8_t
	{
		#define PR_LDRAW_ARROW_TYPES(x)\
		x(Line        , 0)\
		x(Fwd         , 1 << 0)\
		x(Back        , 1 << 1)\
		x(FwdBack     , Fwd | Back)\
		x(_flags_enum , 0xFF)
		
		#define PR_LDRAW_ENUM_MEMBERS(name, value) name = value,
		PR_LDRAW_ARROW_TYPES(PR_LDRAW_ENUM_MEMBERS)
		#undef PR_LDRAW_ENUM_MEMBERS
	};

	// Camera fields
	enum class ECamField :int
	{
		None = 0,
		C2W = 1 << 0,
		Focus = 1 << 1,
		Align = 1 << 2,
		Aspect = 1 << 3,
		FovY = 1 << 4,
		Near = 1 << 5,
		Far = 1 << 6,
		Ortho = 1 << 7,
		_flags_enum = 0,
	};

	// Flags for partial update of a model
	enum class EUpdateObject :int
	{
		None = 0,
		Name = 1 << 0,
		Model = 1 << 1,
		Transform = 1 << 2,
		Children = 1 << 3,
		Colour = 1 << 4,
		GroupTint = 1 << 5,
		Reflectivity = 1 << 6,
		Flags = 1 << 7,
		Animation = 1 << 8,
		All = 0x1FF,
		_flags_enum = 0,
	};

	// Flags for extra behaviour of an object
	enum class ELdrFlags : int // sync with 'view3d-dll.h'
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

		// Hide animated models when the time is outside their animation time range
		HideWhenNotAnimating = 1 << 14,

		// Indicates invalidated flags that need to be refreshed
		Invalidated = 1 << 31,

		// Bitwise operators
		_flags_enum = 0,
	};

	// Flags for calculating bounding boxes
	enum class EBBoxFlags
	{
		None = 0,
		IncludeChildren = 1 << 0,
	};

	// Colour blend operations
	enum class EColourOp
	{
		Overwrite,
		Add,
		Subtract,
		Multiply,
		Lerp,
	};

	// Hard types for ldraw::Builder output
	struct textbuf :std::string
	{
		template <typename... Args> textbuf& append(Args... args)
		{
			std::string::append(std::forward<Args>(args)...);
			return *this;
		}
		template <typename T> friend std::basic_ostream<T>& operator << (std::basic_ostream<T>& out, textbuf const& ldr)
		{
			out.write(reinterpret_cast<T const*>(ldr.data()), ldr.size());
			return out;
		}
		friend std::ptrdiff_t ssize(textbuf const& buf)
		{
			return std::ssize(buf);
		}
	};
	struct bytebuf :std::vector<std::byte>
	{
		template <typename T> friend std::basic_ostream<T>& operator << (std::basic_ostream<T>& out, bytebuf const& ldr)
		{
			out.write(reinterpret_cast<T const*>(ldr.data()), ldr.size());
			return out;
		}
		friend std::ptrdiff_t ssize(bytebuf const& buf)
		{
			return std::ssize(buf);
		}
	};

	// Reflection
	PR_ENUM_REFLECTION1(EKeyword, PR_LDRAW_KEYWORDS);
	PR_ENUM_REFLECTION1(ELdrObject, PR_LDRAW_OBJECTS);
	PR_ENUM_REFLECTION1(ECommandId, PR_LDRAW_COMMANDS);
	PR_ENUM_REFLECTION1(EPointStyle, PR_LDRAW_POINT_STYLES);
	PR_ENUM_REFLECTION1(ELineStyle, PR_LDRAW_LINE_STYLES);
	PR_ENUM_REFLECTION2(EArrowType, PR_LDRAW_ARROW_TYPES);

	#undef PR_LDRAW_KEYWORDS
	#undef PR_LDRAW_COMMANDS
	#undef PR_LDRAW_ARROW_TYPES
	#undef PR_LDRAW_POINT_STYLES
	#undef PR_LDRAW_LINE_STYLES
}

namespace pr
{
	template <> struct is_string<rdr12::ldraw::textbuf> : std::true_type {};
	template <> struct string_traits<rdr12::ldraw::textbuf const> : string_traits<std::basic_string<char> const> {};
	template <> struct string_traits<rdr12::ldraw::textbuf> : string_traits<std::basic_string<char>> {};
	static_assert(pr::is_string_v<rdr12::ldraw::textbuf>);
}
