//************************************
// LDraw 
//  Copyright (c) Rylogic Ltd 2006
//************************************
// Simple LDraw script creation helpers
#pragma once
#include <concepts>
#include <type_traits>
#include <memory>
#include <vector>
#include <string>
#include <string_view>
#include <filesystem>
#include <ranges>
#include <optional>
#include <charconv>
#include <format>
#include <random>
#include <system_error>
#include <iostream>
#include <fstream>
// Only standard library headers

namespace pr::ldraw
{
	struct LdrBase;
	struct LdrPoint;
	struct LdrLine;
	struct LdrBox;
	struct LdrModel;
	struct LdrGroup;
	
	//struct LdrCoordFrame;
	//struct LdrTriangle;
	//struct LdrPlane;
	//struct LdrCircle;
	//struct LdrSphere;
	//struct LdrBox;
	//struct LdrCylinder;
	//struct LdrCone;
	//struct LdrFrustum;
	//struct LdrInstance;
	//struct LdrGroup;
	//struct LdrCommands;
	//struct LdrBinaryStream;
	//struct LdrTextStream;

	using ObjPtr = std::shared_ptr<LdrBase>;
	using textbuf = std::string;
	using bytebuf = std::vector<std::byte>;

	enum class ESaveFlags : uint32_t
	{
		None = 0,
		Binary = 1 << 0,
		Pretty = 1 << 1,
		Append = 1 << 2,
		NoThrowOnFailure = 1 << 8,
		_flags_enum = 0,
	};

	template <typename T> concept TVec2 = requires (T t)
	{
		{ t.x } -> std::convertible_to<float>;
		{ t.y } -> std::convertible_to<float>;
	};
	template <typename T> concept TVec3 = requires (T t)
	{
		{ t.x } -> std::convertible_to<float>;
		{ t.y } -> std::convertible_to<float>;
		{ t.z } -> std::convertible_to<float>;
	};
	template <typename T> concept TVec4 = requires (T t)
	{
		{ t.x } -> std::convertible_to<float>;
		{ t.y } -> std::convertible_to<float>;
		{ t.z } -> std::convertible_to<float>;
		{ t.w } -> std::convertible_to<float>;
	};
	template <typename T> concept TMat3 = requires (T t)
	{
		{ t.x } -> TVec3;
		{ t.y } -> TVec3;
		{ t.z } -> TVec3;
	};
	template <typename T> concept TMat4 = requires (T t)
	{
		{ t.x } -> TVec4;
		{ t.y } -> TVec4;
		{ t.z } -> TVec4;
		{ t.w } -> TVec4;
	};
	template <typename T> concept TColour32 = requires (T t)
	{
		{ t } -> std::convertible_to<uint32_t>;
	};
	template <typename T> concept TString = requires (T t)
	{
		t.data();
		t.size();
		t.reserve(0);
		t.push_back('c');
		{ t.append(0, 'c') } -> std::convertible_to<T&>;
	};

	// Enum strings
	struct EKeyword
	{
		char const* name = "";
		uint32_t value = 0;

		constexpr EKeyword() {}
		constexpr EKeyword(char const* name_, uint32_t value_)
		{
			name = name_;
			value = value_;
		}
		explicit constexpr operator bool() const { return value != 0; }
		constexpr bool operator==(EKeyword const& rhs) const { return value == rhs.value; }
		constexpr bool operator!=(EKeyword const& rhs) const { return value != rhs.value; }
	};
	struct EKeywords
	{
		#pragma region Keywords
		// AUTO-GENERATED-KEYWORDS-BEGIN
		inline static constexpr EKeyword Accel = {"*Accel", 3784776339};
		inline static constexpr EKeyword Addr = {"*Addr", 1087856498};
		inline static constexpr EKeyword Align = {"*Align", 1613521886};
		inline static constexpr EKeyword Alpha = {"*Alpha", 1569418667};
		inline static constexpr EKeyword Ambient = {"*Ambient", 479609067};
		inline static constexpr EKeyword Anchor = {"*Anchor", 1122880180};
		inline static constexpr EKeyword AngAccel = {"*AngAccel", 801436173};
		inline static constexpr EKeyword AngVelocity = {"*AngVelocity", 2226367268};
		inline static constexpr EKeyword AnimSource = {"*AnimSource", 4254838773};
		inline static constexpr EKeyword Animation = {"*Animation", 3779456605};
		inline static constexpr EKeyword Arrow = {"*Arrow", 2220107398};
		inline static constexpr EKeyword Aspect = {"*Aspect", 2929999953};
		inline static constexpr EKeyword Axis = {"*Axis", 1831579124};
		inline static constexpr EKeyword AxisId = {"*AxisId", 1558262649};
		inline static constexpr EKeyword BackColour = {"*BackColour", 1583172070};
		inline static constexpr EKeyword BakeTransform = {"*BakeTransform", 3697187404};
		inline static constexpr EKeyword Billboard = {"*Billboard", 3842439780};
		inline static constexpr EKeyword Billboard3D = {"*Billboard3D", 3185018435};
		inline static constexpr EKeyword BinaryStream = {"*BinaryStream", 1110191492};
		inline static constexpr EKeyword Box = {"*Box", 1892056626};
		inline static constexpr EKeyword BoxList = {"*BoxList", 282663022};
		inline static constexpr EKeyword Camera = {"*Camera", 2663290958};
		inline static constexpr EKeyword CastShadow = {"*CastShadow", 3890809582};
		inline static constexpr EKeyword Chart = {"*Chart", 1487494731};
		inline static constexpr EKeyword Circle = {"*Circle", 673280137};
		inline static constexpr EKeyword Closed = {"*Closed", 3958264005};
		inline static constexpr EKeyword Colour = {"*Colour", 33939709};
		inline static constexpr EKeyword Colours = {"*Colours", 3175120778};
		inline static constexpr EKeyword Commands = {"*Commands", 3062934995};
		inline static constexpr EKeyword Cone = {"*Cone", 3711978346};
		inline static constexpr EKeyword ConvexHull = {"*ConvexHull", 1990998937};
		inline static constexpr EKeyword CoordFrame = {"*CoordFrame", 3958982335};
		inline static constexpr EKeyword CornerRadius = {"*CornerRadius", 2396916310};
		inline static constexpr EKeyword CrossSection = {"*CrossSection", 3755993582};
		inline static constexpr EKeyword CString = {"*CString", 2719639595};
		inline static constexpr EKeyword Custom = {"*Custom", 542584942};
		inline static constexpr EKeyword Cylinder = {"*Cylinder", 2904473583};
		inline static constexpr EKeyword Dashed = {"*Dashed", 4029489596};
		inline static constexpr EKeyword Data = {"*Data", 3631407781};
		inline static constexpr EKeyword DataPoints = {"*DataPoints", 1656579750};
		inline static constexpr EKeyword Depth = {"*Depth", 4269121258};
		inline static constexpr EKeyword Diffuse = {"*Diffuse", 1416505917};
		inline static constexpr EKeyword Dim = {"*Dim", 3496118841};
		inline static constexpr EKeyword Direction = {"*Direction", 3748513642};
		inline static constexpr EKeyword Divisions = {"*Divisions", 555458703};
		inline static constexpr EKeyword Equation = {"*Equation", 2486886355};
		inline static constexpr EKeyword Euler = {"*Euler", 1180123250};
		inline static constexpr EKeyword Faces = {"*Faces", 455960701};
		inline static constexpr EKeyword Facets = {"*Facets", 3463018577};
		inline static constexpr EKeyword Far = {"*Far", 3170376174};
		inline static constexpr EKeyword FilePath = {"*FilePath", 1962937316};
		inline static constexpr EKeyword Filter = {"*Filter", 3353438327};
		inline static constexpr EKeyword Font = {"*Font", 659427984};
		inline static constexpr EKeyword ForeColour = {"*ForeColour", 3815865055};
		inline static constexpr EKeyword Format = {"*Format", 3114108242};
		inline static constexpr EKeyword Fov = {"*Fov", 2968750556};
		inline static constexpr EKeyword FovX = {"*FovX", 862039340};
		inline static constexpr EKeyword FovY = {"*FovY", 878816959};
		inline static constexpr EKeyword Frame = {"*Frame", 3523899814};
		inline static constexpr EKeyword FrameRange = {"*FrameRange", 1562558803};
		inline static constexpr EKeyword FrameRate = {"*FrameRate", 2601589476};
		inline static constexpr EKeyword FrustumFA = {"*FrustumFA", 3281884904};
		inline static constexpr EKeyword FrustumWH = {"*FrustumWH", 3334630522};
		inline static constexpr EKeyword GenerateNormals = {"*GenerateNormals", 750341558};
		inline static constexpr EKeyword Grid = {"*Grid", 2944866961};
		inline static constexpr EKeyword Group = {"*Group", 1605967500};
		inline static constexpr EKeyword GroupColour = {"*GroupColour", 2738848320};
		inline static constexpr EKeyword Hidden = {"*Hidden", 4128829753};
		inline static constexpr EKeyword HideWhenNotAnimating = {"*HideWhenNotAnimating", 2975106646};
		inline static constexpr EKeyword Instance = {"*Instance", 193386898};
		inline static constexpr EKeyword Inverse = {"*Inverse", 2986472067};
		inline static constexpr EKeyword Layers = {"*Layers", 2411172191};
		inline static constexpr EKeyword LeftHanded = {"*LeftHanded", 1992685208};
		inline static constexpr EKeyword LightSource = {"*LightSource", 2597226090};
		inline static constexpr EKeyword Line = {"*Line", 400234023};
		inline static constexpr EKeyword LineBox = {"*LineBox", 3297263992};
		inline static constexpr EKeyword LineList = {"*LineList", 419493935};
		inline static constexpr EKeyword Lines = {"*Lines", 3789825596};
		inline static constexpr EKeyword LineStrip = {"*LineStrip", 4082781759};
		inline static constexpr EKeyword LookAt = {"*LookAt", 3951693683};
		inline static constexpr EKeyword M3x3 = {"*M3x3", 1709156072};
		inline static constexpr EKeyword M4x4 = {"*M4x4", 3279345952};
		inline static constexpr EKeyword Mesh = {"*Mesh", 2701180604};
		inline static constexpr EKeyword Model = {"*Model", 2961925722};
		inline static constexpr EKeyword Montage = {"*Montage", 2939791094};
		inline static constexpr EKeyword Name = {"*Name", 2369371622};
		inline static constexpr EKeyword Near = {"*Near", 1425233679};
		inline static constexpr EKeyword NewLine = {"*NewLine", 4281549323};
		inline static constexpr EKeyword NonAffine = {"*NonAffine", 3876544483};
		inline static constexpr EKeyword NoMaterials = {"*NoMaterials", 762077060};
		inline static constexpr EKeyword Normalise = {"*Normalise", 4066511049};
		inline static constexpr EKeyword Normals = {"*Normals", 247908339};
		inline static constexpr EKeyword NoRootTranslation = {"*NoRootTranslation", 3287374065};
		inline static constexpr EKeyword NoRootRotation = {"*NoRootRotation", 3606635828};
		inline static constexpr EKeyword NoZTest = {"*NoZTest", 329427844};
		inline static constexpr EKeyword NoZWrite = {"*NoZWrite", 1339143375};
		inline static constexpr EKeyword O2W = {"*O2W", 2877203913};
		inline static constexpr EKeyword Orthographic = {"*Orthographic", 3824181163};
		inline static constexpr EKeyword Orthonormalise = {"*Orthonormalise", 2850748489};
		inline static constexpr EKeyword Padding = {"*Padding", 2157316278};
		inline static constexpr EKeyword Param = {"*Param", 1309554226};
		inline static constexpr EKeyword Parametrics = {"*Parametrics", 4148475404};
		inline static constexpr EKeyword Part = {"*Part", 2088252948};
		inline static constexpr EKeyword Parts = {"*Parts", 1480434725};
		inline static constexpr EKeyword Period = {"*Period", 2580104964};
		inline static constexpr EKeyword PerItemColour = {"*PerItemColour", 1734234667};
		inline static constexpr EKeyword PerItemParametrics = {"*PerItemParametrics", 2079701142};
		inline static constexpr EKeyword Pie = {"*Pie", 1782644405};
		inline static constexpr EKeyword Plane = {"*Plane", 3435855957};
		inline static constexpr EKeyword Point = {"*Point", 414084241};
		inline static constexpr EKeyword PointDepth = {"*PointDepth", 1069768758};
		inline static constexpr EKeyword PointSize = {"*PointSize", 375054368};
		inline static constexpr EKeyword PointStyle = {"*PointStyle", 2082693170};
		inline static constexpr EKeyword Polygon = {"*Polygon", 85768329};
		inline static constexpr EKeyword Pos = {"*Pos", 1412654217};
		inline static constexpr EKeyword Position = {"*Position", 2471448074};
		inline static constexpr EKeyword Quad = {"*Quad", 1738228046};
		inline static constexpr EKeyword Quat = {"*Quat", 1469786142};
		inline static constexpr EKeyword QuatPos = {"*QuatPos", 1842422916};
		inline static constexpr EKeyword Rand4x4 = {"*Rand4x4", 1326225514};
		inline static constexpr EKeyword RandColour = {"*RandColour", 1796074266};
		inline static constexpr EKeyword RandOri = {"*RandOri", 55550374};
		inline static constexpr EKeyword RandPos = {"*RandPos", 4225427356};
		inline static constexpr EKeyword Range = {"*Range", 4208725202};
		inline static constexpr EKeyword Rect = {"*Rect", 3940830471};
		inline static constexpr EKeyword Reflectivity = {"*Reflectivity", 3111471187};
		inline static constexpr EKeyword Resolution = {"*Resolution", 488725647};
		inline static constexpr EKeyword Ribbon = {"*Ribbon", 1119144745};
		inline static constexpr EKeyword RootAnimation = {"*RootAnimation", 464566237};
		inline static constexpr EKeyword Round = {"*Round", 1326178875};
		inline static constexpr EKeyword Scale = {"*Scale", 2190941297};
		inline static constexpr EKeyword ScreenSpace = {"*ScreenSpace", 3267318065};
		inline static constexpr EKeyword Series = {"*Series", 3703783856};
		inline static constexpr EKeyword Size = {"*Size", 597743964};
		inline static constexpr EKeyword Smooth = {"*Smooth", 24442543};
		inline static constexpr EKeyword Solid = {"*Solid", 2973793012};
		inline static constexpr EKeyword Source = {"*Source", 466561496};
		inline static constexpr EKeyword Specular = {"*Specular", 3195258592};
		inline static constexpr EKeyword Sphere = {"*Sphere", 2950268184};
		inline static constexpr EKeyword Square = {"*Square", 3031831110};
		inline static constexpr EKeyword Step = {"*Step", 3343129103};
		inline static constexpr EKeyword Stretch = {"*Stretch", 3542801962};
		inline static constexpr EKeyword Strikeout = {"*Strikeout", 3261692833};
		inline static constexpr EKeyword Style = {"*Style", 2888859350};
		inline static constexpr EKeyword Tetra = {"*Tetra", 1647597299};
		inline static constexpr EKeyword TexCoords = {"*TexCoords", 536531680};
		inline static constexpr EKeyword Text = {"*Text", 3185987134};
		inline static constexpr EKeyword TextLayout = {"*TextLayout", 2881593448};
		inline static constexpr EKeyword TextStream = {"*TextStream", 998584670};
		inline static constexpr EKeyword Texture = {"*Texture", 1013213428};
		inline static constexpr EKeyword TimeBias = {"*TimeBias", 2748914857};
		inline static constexpr EKeyword TimeRange = {"*TimeRange", 1138166793};
		inline static constexpr EKeyword Transpose = {"*Transpose", 3224470464};
		inline static constexpr EKeyword Triangle = {"*Triangle", 84037765};
		inline static constexpr EKeyword TriList = {"*TriList", 3668920810};
		inline static constexpr EKeyword TriStrip = {"*TriStrip", 1312470952};
		inline static constexpr EKeyword Tube = {"*Tube", 1747223167};
		inline static constexpr EKeyword Txfm = {"*Txfm", 2438414104};
		inline static constexpr EKeyword Underline = {"*Underline", 3850515583};
		inline static constexpr EKeyword Unknown = {"*Unknown", 2608177081};
		inline static constexpr EKeyword Up = {"*Up", 1128467232};
		inline static constexpr EKeyword Velocity = {"*Velocity", 846470194};
		inline static constexpr EKeyword Verts = {"*Verts", 3167497763};
		inline static constexpr EKeyword Video = {"*Video", 3472427884};
		inline static constexpr EKeyword ViewPlaneZ = {"*ViewPlaneZ", 458706800};
		inline static constexpr EKeyword Wedges = {"*Wedges", 451463732};
		inline static constexpr EKeyword Weight = {"*Weight", 1352703673};
		inline static constexpr EKeyword Width = {"*Width", 2508680735};
		inline static constexpr EKeyword Wireframe = {"*Wireframe", 305834533};
		inline static constexpr EKeyword XAxis = {"*XAxis", 3274667154};
		inline static constexpr EKeyword XColumn = {"*XColumn", 3953029077};
		inline static constexpr EKeyword YAxis = {"*YAxis", 1077811589};
		inline static constexpr EKeyword ZAxis = {"*ZAxis", 3837765916};
		// AUTO-GENERATED-KEYWORDS-END
		#pragma endregion
	};
	struct EPointStyle
	{
		char const* style;
		EPointStyle() : style(Square) {}
		EPointStyle(char const* style_) : style(style_) {}
		explicit operator bool() const { return style != Square; }

		inline static char const* Square = "Square";
		inline static char const* Circle = "Circle";
		inline static char const* Triangle = "Triangle";
		inline static char const* Star = "Star";
		inline static char const* Annulus = "Annulus";
	};
	struct ELineStyle
	{
		char const* style;
		ELineStyle() : style(LineSegments) {}
		ELineStyle(char const* style_) : style(style_) {}
		explicit operator bool() const { return style != LineSegments; }

		inline static char const* LineSegments = "LineSegments";
		inline static char const* LineStrip = "LineStrip";
		inline static char const* Direction = "Direction";
		inline static char const* BezierSpline = "BezierSpline";
		inline static char const* HermiteSpline = "HermiteSpline";
		inline static char const* BSplineSpline = "BSplineSpline";
		inline static char const* CatmullRom = "CatmullRom";
	};
	struct EArrowType
	{
		char const* type;
		EArrowType() : type(Line) {}
		EArrowType(char const* type_) : type(type_) {}
		explicit operator bool() const { return type != Line; }

		inline static char const* Line    = "Line";
		inline static char const* Fwd     = "Fwd";
		inline static char const* Back    = "Back";
		inline static char const* FwdBack = "FwdBack";
	};

	// Serializing helpers
	namespace seri
	{
		struct Vec2
		{
			float x, y;
			Vec2() : x(), y() {}
			Vec2(float x_, float y_) : x(x_), y(y_) {}
			Vec2(TVec2 auto v) :x(v.x), y(v.y) {}
			explicit operator bool() const { return x != 0 || y != 0; }
			friend auto operator <=> (Vec2 const& lhs, Vec2 const& rhs) = default;
		};
		struct Vec3
		{
			float x, y, z;
			Vec3() : x(), y(), z() {}
			Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
			Vec3(TVec3 auto v) :x(v.x), y(v.y), z(v.z) {}
			explicit operator bool() const { return x != 0 || y != 0 || z != 0; }
			friend auto operator <=> (Vec3 const& lhs, Vec3 const& rhs) = default;
		};
		struct Vec4
		{
			float x, y, z, w;
			Vec4() : x(), y(), z(), w() {}
			Vec4(float x_, float y_, float z_, float w_) : x(x_), y(y_), z(z_), w(w_) {}
			Vec4(TVec4 auto v) :x(v.x), y(v.y), z(v.z), w(v.w) {}
			explicit operator bool() const { return x != 0 || y != 0 || z != 0 || w != 0; }
			friend auto operator <=> (Vec4 const& lhs, Vec4 const& rhs) = default;
		};
		struct Mat3
		{
			Vec3 x, y, z;
			Mat3() : x(), y(), z() {}
			Mat3(Vec3 const& x_, Vec3 const& y_, Vec3 const& z_) : x(x_), y(y_), z(z_) {}
			Mat3(TMat3 auto v) :x(v.x), y(v.y), z(v.z) {}
			explicit operator bool() const { return x != Vec3{1, 0, 0} || y != Vec3(0,1,0) || z != Vec3(0,0,1); }
			friend auto operator <=> (Mat3 const& lhs, Mat3 const& rhs) = default;
		};
		struct Mat4
		{
			Vec4 x, y, z, w;
			Mat4() : x(), y(), z(), w() {}
			Mat4(Vec4 const& x_, Vec4 const& y_, Vec4 const& z_, Vec4 const& w_) : x(x_), y(y_), z(z_), w(w_) {}
			Mat4(TMat4 auto v) :x(v.x), y(v.y), z(v.z), w(v.w) {}
			explicit operator bool() const { return x != Vec4{1, 0, 0, 0} || y != Vec4(0,1,0,0) || z != Vec4(0,0,1,0) || w != Vec4(0,0,0,1); }
			friend auto operator <=> (Mat4 const& lhs, Mat4 const& rhs) = default;
		};
		struct Name
		{
			EKeyword m_kw;
			std::optional<std::string> m_name;
			Name() :m_kw(), m_name() {}
			Name(std::string_view str) :m_kw(), m_name(Sanitise(str)) {}
			Name(std::string const& str) :m_kw(), m_name(Sanitise(str)) {}
			Name(EKeyword kw, std::string_view str) :m_kw(kw), m_name(Sanitise(str)) {}
			template <int N> Name(char const (&str)[N]) :m_kw(), m_name(Sanitise(str)) {}
			static std::string Sanitise(std::string_view name)
			{
				std::string result(name);
				for (auto& ch : result) ch = std::isalnum(ch) ? ch : '_';
				if (!result.empty() && !std::isalpha(result[0])) result.insert(0, 1, '_');
				return result;
			}
			explicit operator bool() const { return m_name.has_value(); }
		};
		struct Colour
		{
			EKeyword m_kw;
			std::optional<uint32_t> m_colour;
			Colour() :m_kw(), m_colour() {}
			Colour(uint32_t argb) :m_kw(), m_colour(argb) {}
			Colour(TColour32 auto c) :m_kw(), m_colour(c) {}
			Colour(EKeyword kw, TColour32 auto c) :m_kw(kw), m_colour(c) {}
			explicit operator bool() const { return m_colour.has_value(); }
			static constexpr uint32_t Default = 0xFFFFFFFF;
		};
		struct Size
		{
			float m_size;
			Size() : m_size() {}
			Size(float size) : m_size(size) {}
			Size(int size) : m_size(float(size)) {}
			explicit operator bool() const { return m_size != 0; }
		};
		struct Size2
		{
			Vec2 m_size;
			Size2() : m_size() {}
			Size2(Vec2 size) : m_size(size) {}
			Size2(float sx, float_t sy) : m_size(sx, sy) {}
			explicit operator bool() const { return !!m_size; }
		};
		struct Scale
		{
			float m_scale;
			Scale() : m_scale(1) {}
			Scale(float scale) : m_scale(scale) {}
			explicit operator bool() const { return m_scale != 1; }
		};
		struct Scale2
		{
			Vec2 m_scale;
			Scale2() : m_scale(1, 1) {}
			Scale2(Vec2 scale) : m_scale(scale) {}
			Scale2(float sx, float sy) : m_scale(sx, sy) {}
			explicit operator bool() const { return !!m_scale; }
		};
		struct Scale3
		{
			Vec3 m_scale;
			Scale3() : m_scale(1, 1, 1) {}
			Scale3(Vec3 scale) :m_scale(scale) {}
			explicit operator bool() const { return !!m_scale; }
		};
		struct PerItemColour
		{
			std::optional<bool> m_active;
			PerItemColour() : m_active() {}
			PerItemColour(bool has_colours) : m_active(has_colours) {}
			explicit operator bool() const { return m_active.has_value(); }
		};
		struct Width
		{
			std::optional<float> m_width;
			Width() :m_width() {}
			Width(float w) :m_width(w) {}
			explicit operator bool() const { return m_width.has_value(); }
		};
		struct Depth
		{
			std::optional<bool> m_depth;
			Depth() :m_depth() {}
			Depth(bool d) : m_depth(d) {}
			explicit operator bool() const { return m_depth.has_value(); }
		};
		struct Hidden
		{
			std::optional<bool> m_hide;
			Hidden() :m_hide() {}
			Hidden(bool h) :m_hide(h) {}
			explicit operator bool() const { return m_hide.has_value(); }
		};
		struct Wireframe
		{
			std::optional<bool> m_wire;
			Wireframe() :m_wire() {}
			Wireframe(bool w) :m_wire(w) {}
			explicit operator bool() const { return m_wire.has_value(); }
		};
		struct Alpha
		{
			std::optional<bool> m_alpha;
			Alpha() :m_alpha() {}
			Alpha(bool a) : m_alpha(a) {}
			explicit operator bool() const { return m_alpha.has_value(); }
		};
		struct Reflectivity
		{
			std::optional<float> m_refl;
			Reflectivity() : m_refl() {}
			Reflectivity(float r) : m_refl(r) {}
			explicit operator bool() const { return m_refl.has_value(); }
		};
		struct Solid
		{
			std::optional<bool> m_solid;
			Solid() :m_solid() {}
			Solid(bool s) : m_solid(s) {}
			explicit operator bool() const { return m_solid.has_value(); }
		};
		struct Smooth
		{
			std::optional<bool> m_smooth;
			Smooth() :m_smooth() {}
			Smooth(bool s) : m_smooth(s) {}
			explicit operator bool() const { return m_smooth.has_value(); }
		};
		struct Dashed
		{
			std::optional<Vec2> m_dash;
			Dashed() : m_dash() {}
			Dashed(Vec2 dash) : m_dash(dash) {}
			explicit operator bool() const { return m_dash.has_value(); }
		};
		struct DataPoints
		{
			std::optional<Vec2> m_size;
			std::optional<Colour> m_colour;
			std::optional<EPointStyle> m_style;
			DataPoints() : m_size(), m_colour(), m_style() {}
			DataPoints(Vec2 size, Colour colour, EPointStyle style) : m_size(size), m_colour(colour), m_style(style) {}
			explicit operator bool() const { return m_size.has_value() || m_colour.has_value() || m_style.has_value(); }
		};
		struct LeftHanded
		{
			std::optional<bool> m_lh;
			LeftHanded() :m_lh() {}
			LeftHanded(bool lh) : m_lh(lh) {}
			explicit operator bool() const { return m_lh.has_value(); }
		};
		struct ScreenSpace
		{
			std::optional<bool> m_screen_space;
			ScreenSpace() :m_screen_space() {}
			ScreenSpace(bool screen_space) : m_screen_space(screen_space) {}
			explicit operator bool() const { return m_screen_space.has_value(); }
		};
		struct NoZTest
		{
			std::optional<bool> m_no_ztest;
			NoZTest() :m_no_ztest() {}
			NoZTest(bool no_ztest) : m_no_ztest(no_ztest) {}
			explicit operator bool() const { return m_no_ztest.has_value(); }
		};
		struct NoZWrite
		{
			std::optional<bool> m_no_zwrite;
			NoZWrite() :m_no_zwrite() {}
			NoZWrite(bool no_zwrite) : m_no_zwrite(no_zwrite) {}
			explicit operator bool() const { return m_no_zwrite.has_value(); }
		};
		struct AxisId
		{
			std::optional<int> m_id;
			AxisId() : m_id() {}
			AxisId(int id) : m_id(id) {}
			explicit operator bool() const { return m_id.has_value(); }

			inline static int const None = 0;
			inline static int const PosX = +1;
			inline static int const PosY = +2;
			inline static int const PosZ = +3;
			inline static int const NegX = -1;
			inline static int const NegY = -2;
			inline static int const NegZ = -3;
		};
		struct PointStyle
		{
			std::optional<EPointStyle> m_style;
			PointStyle() : m_style() {}
			PointStyle(EPointStyle style) : m_style(style) {}
			explicit operator bool() const { return m_style.has_value(); }
		};
		struct LineStyle
		{
			std::optional<ELineStyle> m_style;
			LineStyle() : m_style() {}
			LineStyle(ELineStyle style) : m_style(style) {}
			explicit operator bool() const { return m_style.has_value(); }
		};
		struct ArrowHeads
		{
			std::optional<EArrowType> m_type;
			float m_size;
			ArrowHeads() : m_type(), m_size() {}
			ArrowHeads(EArrowType type, float size = 10) : m_type(type), m_size(size) {}
			explicit operator bool() const { return m_type.has_value(); }
		};
		struct O2W
		{
			// todo: need a better way to do this, either a text form or a binary form, but not both.
			textbuf m_xform_txt = {};
			bytebuf m_xform_bin = {};

			O2W& o2w(Mat4 o2w);
			O2W& rot(Mat3 rot);
			O2W& align(Vec3 dir, AxisId axis = AxisId::PosZ);
			O2W& lookat(Vec3 pos);
			O2W& quat(Vec4 q);
			O2W& pos(Vec3 pos);
			O2W& pos(float x, float y, float z);
			O2W& scale(Vec3 scale);
			O2W& scale(float sx, float sy, float sz);
			O2W& scale(float s);
			O2W& euler(float pitch_deg, float yaw_deg, float roll_deg);
			O2W& rand(Vec3 centre, float radius);
			O2W& rand_pos(Vec3 centre, float radius);
			O2W& rand_ori();
			O2W& normalise();
			O2W& orthonormalise();
			O2W& transpose();
			O2W& inverse();
			O2W& non_affine();
			explicit operator bool() const
			{
				return !m_xform_txt.empty() || !m_xform_bin.empty();
			}
		};
		struct Texture
		{
			std::string m_tex;
			O2W m_t2s;

			Texture()
				: m_tex()
			{}
			Texture& filepath(std::string_view filepath);
			Texture& addr(std::string_view mode);
			Texture& addr(std::string_view modeU, std::string_view modeV);
			Texture& filter(std::string_view filter);
			Texture& Alpha(bool on = true);
			O2W& t2s()
			{
				return m_t2s;
			}
			explicit operator bool() const
			{
				return !m_tex.empty();
			}
		};
		struct RootAnimation
		{
			std::string m_anim;

			RootAnimation()
				: m_anim()
			{}
			RootAnimation& style(std::string_view style); // NoAnimation|Once|Repeat|Continuous|PingPong
			RootAnimation& period(float seconds);
			RootAnimation& velocity(Vec3 vel);
			RootAnimation& acceleration(Vec3 accel);
			RootAnimation& ang_velocity(Vec3 ang_vel);
			RootAnimation& ang_acceleration(Vec3 ang_accel);
			explicit operator bool() const
			{
				return !m_anim.empty();
			}
		};
		struct Animation
		{
			std::string m_anim;

			Animation()
				: m_anim()
			{}
			Animation& style(std::string_view style); // NoAnimation|Once|Repeat|Continuous|PingPong
			Animation& frame(int frame);
			Animation& frame_range(int start, int end);
			Animation& time_range(float start, float end);
			Animation& stretch(float speed_multiplier);
			Animation& no_translation();
			Animation& no_rotation();
			explicit operator bool() const
			{
				return !m_anim.empty();
			}			
		};
		struct Header
		{
			EKeyword m_kw;
			Name m_name;
			Colour m_colour;
		};
	}

	// LDraw object model
	struct LdrBase
	{
		std::vector<ObjPtr> m_children;
		seri::Name m_name;
		seri::Colour m_colour;
		seri::Colour m_group_colour;
		seri::Hidden m_hide;
		seri::Wireframe m_wire;
		seri::AxisId m_axis_id;
		seri::Solid m_solid;
		seri::Reflectivity m_refl;
		seri::LeftHanded m_left_handed;
		seri::ScreenSpace m_screen_space;
		seri::NoZTest m_no_ztest;
		seri::NoZWrite m_no_zwrite;
		seri::RootAnimation m_root_anim;
		seri::O2W m_o2w;
		
		LdrBase(seri::Name name = {}, seri::Colour colour = {})
			: m_name(name)
			, m_colour(colour)
		{}
		LdrBase(LdrBase&&) = default;
		LdrBase(LdrBase const&) = delete;
		LdrBase& operator=(LdrBase&&) = default;
		LdrBase& operator=(LdrBase const&) = delete;
		virtual ~LdrBase() = default;

		// Children
		LdrPoint& Point(seri::Name name = {}, seri::Colour colour = {});
		LdrLine& Line(seri::Name name = {}, seri::Colour colour = {});
		LdrBox& Box(seri::Name name = {}, seri::Colour colour = {});
		LdrModel& Model(seri::Name name = {}, seri::Colour colour = {});
		LdrGroup& Group(seri::Name name = {}, seri::Colour colour = {});

		// Object modifiers
		LdrBase& name(seri::Name name)
		{
			m_name = name;
			return *this;
		}
		LdrBase& colour(seri::Colour colour)
		{
			m_colour = colour;
			return *this;
		}
		LdrBase& rand_colour()
		{
			m_colour.m_colour = seri::Colour::Default;
			m_colour.m_kw = EKeywords::RandColour;
			return *this;
		}
		LdrBase& group_colour(seri::Colour colour)
		{
			m_group_colour = colour;
			m_group_colour.m_kw = EKeywords::GroupColour;
			return *this;
		}
		LdrBase& o2w(seri::Mat4 o2w)
		{
			m_o2w.o2w(o2w);
			return *this;
		}
		LdrBase& pos(seri::Vec3 pos)
		{
			m_o2w.pos(pos);
			return *this;
		}
		LdrBase& hide(bool hidden = true)
		{
			m_hide.m_hide = hidden;
			return *this;
		}
		LdrBase& wireframe(bool w = true)
		{
			m_wire.m_wire = w;
			return *this;
		}
		LdrBase& axis(seri::AxisId axis_id)
		{
			m_axis_id = axis_id;
			return *this;
		}
		LdrBase& solid(bool s = true)
		{
			m_solid.m_solid = s;
			return *this;
		}
		LdrBase& reflectivity(float r)
		{
			m_refl.m_refl = r;
			return *this;
		}
		LdrBase& left_handed(bool lh = true)
		{
			m_left_handed.m_lh = lh;
			return *this;
		}
		LdrBase& screen_space(bool ss = true)
		{
			m_screen_space.m_screen_space = ss;
			return *this;
		}
		LdrBase& no_ztest(bool nzt = true)
		{
			m_no_ztest.m_no_ztest = nzt;
			return *this;
		}
		LdrBase& no_zwrite(bool nzw = true)
		{
			m_no_zwrite.m_no_zwrite = nzw;
			return *this;
		}
		seri::RootAnimation& root_animation()
		{
			return m_root_anim;
		}
		seri::O2W& o2w()
		{
			return m_o2w;
		}

		// Copy all modifiers from another object
		LdrBase& modifiers(LdrBase const& rhs)
		{
			m_name = rhs.m_name;
			m_colour = rhs.m_colour;
			m_group_colour = rhs.m_group_colour;
			m_hide = rhs.m_hide;
			m_wire = rhs.m_wire;
			m_axis_id = rhs.m_axis_id;
			m_solid = rhs.m_solid;
			m_o2w = rhs.m_o2w;
			return *this;
		}

		// Serializing
		virtual void Write(textbuf& out) const
		{
			Append(out, m_group_colour, m_hide, m_wire, m_axis_id, m_solid, m_refl, m_left_handed, m_screen_space, m_no_ztest, m_no_zwrite, m_root_anim, m_o2w);
			for (auto const& child : m_children)
				child->Write(out);
		}
		virtual void Write(bytebuf& out) const
		{
			Append(out, m_group_colour, m_hide, m_wire, m_axis_id, m_solid, m_refl, m_left_handed, m_screen_space, m_no_ztest, m_no_zwrite, m_root_anim, m_o2w);
			for (auto const& child : m_children)
				child->Write(out);
		}

		// Append helpers (textbuf)
		template <typename T> struct no_overload;
		template <typename Type> static void Append(textbuf& out, Type)
		{
			no_overload<Type> missing_overload;
		}
		static void Append(textbuf& out, EKeyword kw)
		{
			out.append(kw.name);
		}
		static void Append(textbuf& out, std::string_view s)
		{
			if (s.empty()) return;
			if (*s.data() != '}' && *s.data() != ')' && s.front() != ' ' && !out.empty() && out.back() != '{')
			{
				out.append(1, ' ');
			}
			out.append(s);
		}
		static void Append(textbuf& out, std::string s)
		{
			Append(out, std::string_view{ s });
		}
		static void Append(textbuf& out, char const* s)
		{
			Append(out, std::string_view{ s });
		}
		static void Append(textbuf& out, bool b)
		{
			Append(out, b ? "true" : "false");
		}
		static void Append(textbuf& out, int i)
		{
			char ch[32];
			Append(out, conv(ch, i));
		}
		static void Append(textbuf& out, long i)
		{
			char ch[32];
			Append(out, conv(ch, i));
		}
		static void Append(textbuf& out, float f)
		{
			char ch[32];
			Append(out, conv(ch, f));
		}
		static void Append(textbuf& out, double f)
		{
			char ch[32];
			Append(out, conv(ch, f));
		}
		static void Append(textbuf& out, uint32_t u)
		{
			char ch[32];
			Append(out, conv(ch, u, 16));
		}
		static void Append(textbuf& out, seri::Vec2 v)
		{
			Append(out, v.x, v.y);
		}
		static void Append(textbuf& out, seri::Vec3 v)
		{
			Append(out, v.x, v.y, v.z);
		}
		static void Append(textbuf& out, seri::Vec4 v)
		{
			Append(out, v.x, v.y, v.z, v.w);
		}
		static void Append(textbuf& out, seri::Mat3 m)
		{
			Append(out, m.x, m.y, m.z);
		}
		static void Append(textbuf& out, seri::Mat4 m)
		{
			Append(out, m.x, m.y, m.z, m.w);
		}
		static void Append(textbuf& out, seri::Name n)
		{
			if (!n) return;
			if (!n.m_kw)
				Append(out, *n.m_name);
			else
				Append(out, n.m_kw, "{", *n.m_name, "}");
		}
		static void Append(textbuf& out, seri::Colour c)
		{
			if (!c) return;
			if (!c.m_kw)
				Append(out, *c.m_colour);
			else if (c.m_kw == EKeywords::RandColour)
				Append(out, c.m_kw, "{}");
			else
				Append(out, c.m_kw, "{", *c.m_colour, "}");
		}
		static void Append(textbuf& out, seri::Size s)
		{
			if (!s) return;
			Append(out, EKeywords::Size, "{", s.m_size, "}");
		}
		static void Append(textbuf& out, seri::Size2 s)
		{
			if (!s) return;
			Append(out, EKeywords::Size, "{", s.m_size, "}");
		}
		static void Append(textbuf& out, seri::Scale s)
		{
			if (!s) return;
			Append(out, EKeywords::Scale, "{", s.m_scale, "}");
		}
		static void Append(textbuf& out, seri::Scale2 s)
		{
			if (!s) return;
			Append(out, EKeywords::Scale, "{", s.m_scale, "}");
		}
		static void Append(textbuf& out, seri::Scale3 s)
		{
			if (!s) return;
			Append(out, EKeywords::Scale, "{", s.m_scale, "}");
		}
		static void Append(textbuf& out, seri::PerItemColour c)
		{
			if (!c) return;
			Append(out, EKeywords::PerItemColour, "{", *c.m_active, "}");
		}
		static void Append(textbuf& out, seri::Width w)
		{
			if (!w) return;
			Append(out, EKeywords::Width, "{", *w.m_width, "}");
		}
		static void Append(textbuf& out, seri::Depth d)
		{
			if (!d) return;
			Append(out, EKeywords::Depth, "{", *d.m_depth, "}");
		}
		static void Append(textbuf& out, seri::Hidden h)
		{
			if (!h) return;
			Append(out, EKeywords::Hidden, "{", *h.m_hide, "}");
		}
		static void Append(textbuf& out, seri::Wireframe w)
		{
			if (!w) return;
			Append(out, EKeywords::Wireframe, "{", *w.m_wire, "}");
		}
		static void Append(textbuf& out, seri::Alpha a)
		{
			if (!a) return;
			Append(out, EKeywords::Alpha, "{", *a.m_alpha, "}");
		}
		static void Append(textbuf& out, seri::Reflectivity r)
		{
			if (!r) return;
			Append(out, EKeywords::Reflectivity, "{", *r.m_refl, "}");
		}
		static void Append(textbuf& out, seri::Solid s)
		{
			if (!s) return;
			Append(out, EKeywords::Solid, "{", *s.m_solid, "}");
		}
		static void Append(textbuf& out, seri::Smooth s)
		{
			if (!s) return;
			Append(out, EKeywords::Smooth, "{", *s.m_smooth, "}");
		}
		static void Append(textbuf& out, seri::Dashed d)
		{
			if (!d) return;
			Append(out, EKeywords::Dashed, "{", *d.m_dash, "}");
		}
		static void Append(textbuf& out, seri::DataPoints dp)
		{
			if (!dp) return;
			Append(out, EKeywords::DataPoints, "{");
			if (dp.m_size) Append(out, EKeywords::Size, "{", *dp.m_size, "}");
			if (dp.m_style) Append(out, EKeywords::Style, "{", dp.m_style->style, "}");
			if (dp.m_colour) Append(out, EKeywords::Colour, "{", *dp.m_colour, "}");
			Append(out, "}");
		}
		static void Append(textbuf& out, seri::LeftHanded lh)
		{
			if (!lh) return;
			Append(out, EKeywords::LeftHanded, "{", *lh.m_lh, "}");
		}
		static void Append(textbuf& out, seri::ScreenSpace ss)
		{
			if (!ss) return;
			Append(out, EKeywords::ScreenSpace, "{", *ss.m_screen_space, "}");
		}
		static void Append(textbuf& out, seri::NoZTest nzt)
		{
			if (!nzt) return;
			Append(out, EKeywords::NoZTest, "{", *nzt.m_no_ztest, "}");
		}
		static void Append(textbuf& out, seri::NoZWrite nzw)
		{
			if (!nzw) return;
			Append(out, EKeywords::NoZWrite, "{", *nzw.m_no_zwrite, "}");
		}
		static void Append(textbuf& out, seri::AxisId a)
		{
			if (!a) return;
			Append(out, EKeywords::AxisId, "{", *a.m_id, "}");
		}
		static void Append(textbuf& out, seri::PointStyle p)
		{
			if (!p) return;
			Append(out, EKeywords::Style, "{", p.m_style->style, "}");
		}
		static void Append(textbuf& out, seri::LineStyle l)
		{
			if (!l) return;
			Append(out, EKeywords::Style, "{", l.m_style->style, "}");
		}
		static void Append(textbuf& out, seri::ArrowHeads a)
		{
			if (!a) return;
			Append(out, EKeywords::Arrow, "{", a.m_type->type, a.m_size, "}");
		}
		static void Append(textbuf& out, seri::O2W o2w)
		{
			if (!o2w) return;
			Append(out, "*O2W {", o2w.m_xform_txt, "}");
		}
		static void Append(textbuf& out, seri::Texture t)
		{
			if (!t) return;
			Append(out, "*Texture {", t.m_tex, t.m_t2s, "}");
		}
		static void Append(textbuf& out, seri::RootAnimation ra)
		{
			if (!ra) return;
			Append(out, "*RootAnimation {", ra.m_anim, "}");
		}
		static void Append(textbuf& out, seri::Animation a)
		{
			if (!a) return;
			Append(out, "*Animation {", a.m_anim, "}");
		}
		template <typename... Args> static void Append(textbuf& out, Args&&... args)
		{
			(Append(out, std::forward<Args>(args)), ...);
		}

		// Append Helper (bytebuf)
		template <typename Type> static void Append(bytebuf& out, Type)
		{
			no_overload<Type> missing_overload;
		}
		static auto Append(bytebuf& out, seri::Header hdr)
		{
			auto ofs = out.size();

			Append(out, hdr.m_kw);
			Append(out, int(0)); // Placeholder for section size
			if (hdr.m_name) Append(out, hdr.m_name);
			if (hdr.m_colour) Append(out, hdr.m_colour);

			// RAII object that updates the 'section size' field when it goes out of scope
			struct R
			{
				bytebuf& m_out;
				size_t m_start_ofs;
				R(bytebuf& out, size_t start_ofs) : m_out(out), m_start_ofs(start_ofs) {}
				R(R&&) = default;
				R(R const&) = delete;
				R& operator=(R&&) = default;
				R& operator=(R const&) = delete;
				~R()
				{
					auto end_ofs = m_out.size();
					auto section_size = static_cast<int>(end_ofs - m_start_ofs - sizeof(uint32_t) - sizeof(int));
					std::memcpy(m_out.data() + m_start_ofs + sizeof(uint32_t), &section_size, sizeof(section_size));
				}
			};
			return R{ out, ofs };
		}
		template <typename... Args> static auto Append(bytebuf& out, seri::Header hdr, Args&&... args)
		{
			auto s = Append(out, hdr);
			(Append(out, std::forward<Args>(args)), ...);
			return std::move(s);
		}
		static void Append(bytebuf& out, std::span<std::byte const> data, int64_t ofs = -1)
		{
			if (ofs == -1)
				out.append_range(data);
			else if (ofs + data.size() <= out.size())
				std::memcpy(out.data() + ofs, data.data(), data.size());
			else
				throw std::out_of_range("Append: offset out of range");
		}
		static void Append(bytebuf& out, EKeyword kw)
		{
			Append(out, { byte_ptr(&kw.value), sizeof(kw.value) });
		}
		static void Append(bytebuf& out, std::string_view s)
		{
			Append(out, { byte_ptr(s.data()), size_t(s.size()) });
		}
		static void Append(bytebuf& out, std::string s)
		{
			Append(out, std::string_view{ s });
		}
		static void Append(bytebuf& out, char const* s)
		{
			Append(out, std::string_view{ s });
		}
		static void Append(bytebuf& out, bool b)
		{
			Append(out, uint8_t(b ? 1 : 0));
		}
		static void Append(bytebuf& out, int8_t i)
		{
			Append(out, { byte_ptr(&i), sizeof(i) });
		}
		static void Append(bytebuf& out, int16_t i)
		{
			Append(out, { byte_ptr(&i), sizeof(i) });
		}
		static void Append(bytebuf& out, int32_t i)
		{
			Append(out, { byte_ptr(&i), sizeof(i) });
		}
		static void Append(bytebuf& out, int64_t i)
		{
			Append(out, { byte_ptr(&i), sizeof(i) });
		}
		static void Append(bytebuf& out, uint8_t u)
		{
			Append(out, { byte_ptr(&u), sizeof(u) });
		}
		static void Append(bytebuf& out, uint16_t u)
		{
			Append(out, { byte_ptr(&u), sizeof(u) });
		}
		static void Append(bytebuf& out, uint32_t u)
		{
			Append(out, { byte_ptr(&u), sizeof(u) });
		}
		static void Append(bytebuf& out, uint64_t u)
		{
			Append(out, { byte_ptr(&u), sizeof(u) });
		}
		static void Append(bytebuf& out, float f)
		{
			Append(out, { byte_ptr(&f), sizeof(f) });
		}
		static void Append(bytebuf& out, double f)
		{
			Append(out, { byte_ptr(&f), sizeof(f) });
		}
		static void Append(bytebuf& out, seri::Vec2 v)
		{
			Append(out, { byte_ptr(&v), sizeof(v) });
		}
		static void Append(bytebuf& out, seri::Vec3 v)
		{
			Append(out, { byte_ptr(&v), sizeof(v) });
		}
		static void Append(bytebuf& out, seri::Vec4 v)
		{
			Append(out, { byte_ptr(&v), sizeof(v) });
		}
		static void Append(bytebuf& out, seri::Mat3 m)
		{
			Append(out, { byte_ptr(&m), sizeof(m) });
		}
		static void Append(bytebuf& out, seri::Mat4 m)
		{
			Append(out, { byte_ptr(&m), sizeof(m) });
		}
		static void Append(bytebuf& out, seri::Name n)
		{
			if (!n) return;
			if (!n.m_kw)
			{
				Append(out, *n.m_name);
			}
			else
			{
				Append(out, seri::Header{ n.m_kw }, *n.m_name);
			}
		}
		static void Append(bytebuf& out, seri::Colour c)
		{
			if (!c) return;
			if (!c.m_kw)
			{
				Append(out, *c.m_colour);
			}
			else if (c.m_kw == EKeywords::RandColour)
			{
				Append(out, seri::Header{ c.m_kw });
			}
			else
			{
				Append(out, seri::Header{ c.m_kw }, *c.m_colour);
			}
		}
		static void Append(bytebuf& out, seri::Size sz)
		{
			if (!sz) return;
			auto s = Append(out, seri::Header{ EKeywords::Size });
			Append(out, sz.m_size);
		}
		static void Append(bytebuf& out, seri::Size2 sz)
		{
			if (!sz) return;
			auto s = Append(out, seri::Header{ EKeywords::Size });
			Append(out, sz.m_size);
		}
		static void Append(bytebuf& out, seri::Scale sc)
		{
			if (!sc) return;
			auto s = Append(out, seri::Header{ EKeywords::Scale });
			Append(out, sc.m_scale);
		}
		static void Append(bytebuf& out, seri::Scale2 sc)
		{
			if (!sc) return;
			auto s = Append(out, seri::Header{ EKeywords::Scale });
			Append(out, sc.m_scale);
		}
		static void Append(bytebuf& out, seri::Scale3 sc)
		{
			if (!sc) return;
			auto s = Append(out, seri::Header{ EKeywords::Scale });
			Append(out, sc.m_scale);
		}
		static void Append(bytebuf& out, seri::PerItemColour c)
		{
			if (!c) return;
			auto s = Append(out, seri::Header{ EKeywords::PerItemColour });
			Append(out, *c.m_active);
		}
		static void Append(bytebuf& out, seri::Width w)
		{
			if (!w) return;
			auto s = Append(out, seri::Header{ EKeywords::Width });
			Append(out, *w.m_width);
		}
		static void Append(bytebuf& out, seri::Depth d)
		{
			if (!d) return;
			auto s = Append(out, seri::Header{ EKeywords::Depth });
			Append(out, *d.m_depth);
		}
		static void Append(bytebuf& out, seri::Hidden h)
		{
			if (!h) return;
			auto s = Append(out, seri::Header{ EKeywords::Hidden });
			Append(out, *h.m_hide);
		}
		static void Append(bytebuf& out, seri::Wireframe w)
		{
			if (!w) return;
			auto s = Append(out, seri::Header{ EKeywords::Wireframe });
			Append(out, *w.m_wire);
		}
		static void Append(bytebuf& out, seri::Alpha a)
		{
			if (!a) return;
			auto s = Append(out, seri::Header{ EKeywords::Alpha });
			Append(out, *a.m_alpha);
		}
		static void Append(bytebuf& out, seri::Reflectivity r)
		{
			if (!r) return;
			auto s = Append(out, seri::Header{ EKeywords::Reflectivity });
			Append(out, *r.m_refl);
		}
		static void Append(bytebuf& out, seri::Solid so)
		{
			if (!so) return;
			auto s = Append(out, seri::Header{ EKeywords::Solid });
			Append(out, *so.m_solid);
		}
		static void Append(bytebuf& out, seri::Smooth sm)
		{
			if (!sm) return;
			auto s = Append(out, seri::Header{ EKeywords::Smooth });
			Append(out, *sm.m_smooth);
		}
		static void Append(bytebuf& out, seri::Dashed d)
		{
			if (!d) return;
			auto s = Append(out, seri::Header{ EKeywords::Dashed });
			Append(out, *d.m_dash);
		}
		static void Append(bytebuf& out, seri::DataPoints dp)
		{
			if (!dp) return;
			auto s = Append(out, seri::Header{ EKeywords::DataPoints });
			throw std::runtime_error("todo");
			//if (dp.m_size) Append(out, EKeywords::Size, "{", *dp.m_size, "}");
			//if (dp.m_style) Append(out, EKeywords::Style, "{", dp.m_style->style, "}");
			//if (dp.m_colour) Append(out, EKeywords::Colour, "{", *dp.m_colour, "}");
			
		}
		static void Append(bytebuf& out, seri::LeftHanded lh)
		{
			if (!lh) return;
			auto s = Append(out, seri::Header{ EKeywords::LeftHanded });
			Append(out, *lh.m_lh);
		}
		static void Append(bytebuf& out, seri::ScreenSpace ss)
		{
			if (!ss) return;
			auto s = Append(out, seri::Header{ EKeywords::ScreenSpace });
			Append(out, *ss.m_screen_space);
		}
		static void Append(bytebuf& out, seri::NoZTest nzt)
		{
			if (!nzt) return;
			auto s = Append(out, seri::Header{ EKeywords::NoZTest });
			Append(out, *nzt.m_no_ztest);
		}
		static void Append(bytebuf& out, seri::NoZWrite nzw)
		{
			if (!nzw) return;
			auto s = Append(out, seri::Header{ EKeywords::NoZWrite });
			Append(out, *nzw.m_no_zwrite);
		}
		static void Append(bytebuf& out, seri::AxisId a)
		{
			if (!a) return;
			auto s = Append(out, seri::Header{ EKeywords::AxisId });
			Append(out, *a.m_id);
		}
		static void Append(bytebuf& out, seri::PointStyle p)
		{
			if (!p) return;
			auto s = Append(out, seri::Header{ EKeywords::Style });
			Append(out, p.m_style->style);
		}
		static void Append(bytebuf& out, seri::LineStyle l)
		{
			if (!l) return;
			auto s = Append(out, seri::Header{ EKeywords::Style });
			Append(out, l.m_style->style);
		}
		static void Append(bytebuf& out, seri::ArrowHeads a)
		{
			if (!a) return;
			auto s = Append(out, seri::Header{ EKeywords::Arrow });
			Append(out, a.m_type->type, a.m_size);
		}
		static void Append(bytebuf& out, seri::O2W o2w)
		{
			if (!o2w) return;
			auto s = Append(out, seri::Header{ EKeywords::O2W });
			Append(out, std::span<std::byte const>{ o2w.m_xform_bin });
		}
		static void Append(bytebuf& out, seri::Texture t)
		{
			if (!t) return;
			auto s = Append(out, seri::Header{ EKeywords::Texture });
			Append(out, t.m_tex);
			Append(out, t.m_t2s);
		}
		static void Append(bytebuf& out, seri::RootAnimation ra)
		{
			if (!ra) return;
			auto s = Append(out, seri::Header{ EKeywords::RootAnimation });
			Append(out, ra.m_anim);
		}
		static void Append(bytebuf& out, seri::Animation a)
		{
			if (!a) return;
			auto s = Append(out, seri::Header{ EKeywords::Animation });
			Append(out, a.m_anim);
		}
		template <typename... Args> static void Append(bytebuf& out, Args&&... args)
		{
			(Append(out, std::forward<Args>(args)), ...);
		}

		// Convert values to string
		template <typename T, int N> static std::string_view conv(char(&buf)[N], T value)
		{
			auto [ptr, ec] = std::to_chars(&buf[0], &buf[0] + N, value);
			if (ec != std::errc{}) throw std::system_error(std::make_error_code(ec));
			return std::string_view{ &buf[0], static_cast<size_t>(ptr - &buf[0]) };
		}
		template <typename T, int N> static std::string_view conv(char(&buf)[N], T value, int base)
		{
			auto [ptr, ec] = std::to_chars(&buf[0], &buf[0] + N, value, base);
			if (ec != std::errc{}) throw std::system_error(std::make_error_code(ec));
			return std::string_view{ &buf[0], static_cast<size_t>(ptr - &buf[0]) };
		}

		// Convert to std::byte*
		template <typename T> static constexpr std::byte const* byte_ptr(T const* t)
		{
			return reinterpret_cast<std::byte const*>(t);
		}
		template <typename T> static constexpr std::byte* byte_ptr(T* t)
		{
			return reinterpret_cast<std::byte*>(t);
		}
	};
	struct LdrPoint :LdrBase
	{
		struct Point
		{
			seri::Vec3 pt = {};
			seri::Colour col = {};
		};
		std::vector<Point> m_points;
		seri::Size2 m_size;
		seri::Depth m_depth;
		seri::PointStyle m_style;
		seri::PerItemColour m_per_item_colour;
		seri::Texture m_tex;

		LdrPoint(seri::Name name, seri::Colour colour)
			: LdrBase(name, colour)
		{}

		LdrPoint& pt(seri::Vec3 point, seri::Colour colour = {})
		{
			m_points.push_back({ point, colour });
			if (colour) m_per_item_colour = true;
			return *this;
		}
		LdrPoint& pt(float x, float y, float z, seri::Colour colour = {})
		{
			return pt({ x, y, z }, colour);
		}
		LdrPoint& size(seri::Vec2 s)
		{
			m_size = s;
			return *this;
		}
		LdrPoint& size(float s) // Point size (in pixels if depth == false, in world space if depth == true)
		{
			return size({ s, s });
		}
		LdrPoint& depth(bool d = true) // Points have depth
		{
			m_depth = d;
			return *this;
		}
		LdrPoint& style(EPointStyle s) // Point style
		{
			m_style = s;
			return *this;
		}
		seri::Texture& texture() // Texture for point sprites
		{
			return m_tex;
		}

		virtual void Write(textbuf& out) const override
		{
			Append(out, EKeywords::Point, m_name, m_colour, "{");
			{
				Append(out, m_style, m_size, m_depth, m_per_item_colour);
				Append(out, EKeywords::Data, "{");
				{
					for (auto& point : m_points)
					{
						Append(out, point.pt);
						if (m_per_item_colour && *m_per_item_colour.m_active)
							Append(out, point.col ? *point.col.m_colour : seri::Colour::Default);
					}
				}
				Append(out, "}");
				Append(out, m_tex);
				LdrBase::Write(out);
			}
			Append(out, "}");
		}
		virtual void Write(bytebuf& out) const override
		{
			auto s = Append(out, seri::Header{ EKeywords::Point, m_name, m_colour });
			{
				Append(out, m_style, m_size, m_depth, m_per_item_colour);
				{
					auto sd = Append(out, seri::Header{ EKeywords::Data });
					{
						for (auto& point : m_points)
						{
							Append(out, point.pt);
							if (m_per_item_colour && *m_per_item_colour.m_active)
								Append(out, point.col ? *point.col.m_colour : seri::Colour::Default);
						}
					}
				}
				Append(out, m_tex);
				LdrBase::Write(out);
			}
		}
	};
	struct LdrLine : LdrBase
	{
		struct Ln { seri::Vec3 a, b; seri::Colour col; };
		struct Pt { seri::Vec3 a; seri::Colour col; };
		struct Block
		{
			std::vector<Ln> m_lines;
			std::vector<Pt> m_strip;
			seri::LineStyle m_style;
			seri::Smooth m_smooth;
			seri::Width m_width;
			seri::Dashed m_dashed;
			seri::ArrowHeads m_arrow;
			seri::DataPoints m_data_points;
			seri::PerItemColour m_per_item_colour;
			explicit operator bool() const { return !m_lines.empty() || !m_strip.empty(); }
		};

		std::vector<Block> m_blocks;
		Block m_current;

		LdrLine(seri::Name name, seri::Colour colour)
			: LdrBase(name, colour)
		{}

		LdrLine& style(ELineStyle sty)
		{
			m_current.m_style = sty;
			return *this;
		}
		LdrLine& per_item_colour(bool on = true)
		{
			m_current.m_per_item_colour = on;
			return *this;
		}
		LdrLine& smooth(bool smooth = true)
		{
			m_current.m_smooth = smooth;
			return *this;
		}
		LdrLine& width(seri::Width w)
		{
			m_current.m_width = w;
			return *this;
		}
		LdrLine& dashed(seri::Vec2 dash)
		{
			m_current.m_dashed = seri::Dashed(dash);
			return *this;
		}
		LdrLine& arrow(EArrowType style = EArrowType::Fwd, float size = 10.0f)
		{
			m_current.m_arrow = seri::ArrowHeads(style, size);
			return *this;
		}

		LdrLine& line(seri::Vec3 a, seri::Vec3 b, seri::Colour colour = {})
		{
			style(ELineStyle::LineSegments);
			m_current.m_lines.push_back({ a, b, colour });
			if (colour) m_current.m_per_item_colour = true;
			m_current.m_strip.clear();
			return *this;
		}
		LdrLine& lines(std::ranges::random_access_range auto verts, std::ranges::input_range auto indices)
		{
			auto iter = indices.begin();
			auto iend = indices.end();
			for (; iter != iend; )
			{
				auto v0 = verts[*iter++];
				auto v1 = verts[*iter++];
				line(v0, v1);
			}
			return *this;
		}
		LdrLine& strip(seri::Vec3 start, seri::Colour colour = {})
		{
			style(ELineStyle::LineStrip);
			m_current.m_strip.push_back({ start, colour });
			if (colour) m_current.m_per_item_colour = true;
			m_current.m_lines.clear();
			return *this;
		}
		LdrLine& line_to(seri::Vec3 pt, seri::Colour colour = {})
		{
			if (m_current.m_strip.empty()) strip(pt, colour);
			return strip(pt, colour);
		}

		LdrLine& new_block()
		{
			m_blocks.push_back(std::move(m_current));
			m_current.m_lines.clear();
			m_current.m_strip.clear();
			return *this;
		}

		virtual void Write(textbuf& out) const override
		{
			Append(out, EKeywords::Line, m_name, m_colour, "{");
			{
				auto WriteBlock = [](textbuf& out, Block const& block)
				{
					Append(out, block.m_style, block.m_smooth, block.m_width, block.m_dashed, block.m_arrow, block.m_data_points, block.m_per_item_colour);
					Append(out, EKeywords::Data, "{");
					{
						for (auto& ln : block.m_lines)
						{
							Append(out, ln.a, ln.b);
							if (block.m_per_item_colour)
								Append(out, ln.col.m_colour ? *ln.col.m_colour : seri::Colour::Default);
						}
						for (auto& pt : block.m_strip)
						{
							Append(out, pt.a);
							if (block.m_per_item_colour)
								Append(out, pt.col.m_colour ? *pt.col.m_colour : seri::Colour::Default);
						}
					}
					Append(out, "}");
				};

				for (auto& block : m_blocks)
					WriteBlock(out, block);
				if (m_current)
					WriteBlock(out, m_current);

				LdrBase::Write(out);
			}
			Append(out, "}");
		}
	};
	struct LdrBox : LdrBase
	{
		struct BoxData
		{
			seri::Vec3 m_whd = {};
			seri::Vec3 m_pos = {};
			seri::Colour m_col = {};
		};
		std::vector<BoxData> m_boxes;

		LdrBox(seri::Name name, seri::Colour colour)
			:LdrBase(name, colour)
		{}

		LdrBox& box(seri::Vec3 whd, seri::Vec3 pos = {}, seri::Colour col = {})
		{
			m_boxes.push_back(BoxData{ whd, pos, col });
			return *this;
		}
		LdrBox& box(float x, float y, float z, seri::Vec3 pos = {}, seri::Colour col = {})
		{
			return box(seri::Vec3(x, y, z), pos, col);
		}

		virtual void Write(textbuf& out) const override
		{
			auto single = m_boxes.size() == 1 && !m_boxes[0].m_pos && !m_boxes[0].m_col;
			auto per_item_colour = std::ranges::any_of(m_boxes, [](auto const& x) { return !!x.m_col; });

			Append(out, single ? EKeywords::Box : EKeywords::BoxList, m_name, m_colour, "{");
			{
				if (single)
				{
					Append(out, "*Data {", m_boxes[0].m_whd, "}");
				}
				else
				{
					if (per_item_colour)
						Append(out, "*PerItemColour", "{}");

					Append(out, "*Data {");
					for (auto const& box : m_boxes)
					{
						Append(out, box.m_whd, box.m_pos);
						if (per_item_colour)
							Append(out, box.m_col);
					}
					Append(out, "}");
				}
				LdrBase::Write(out);
			}
			Append(out, "}");
		}
	};
	struct LdrModel : LdrBase
	{
		std::filesystem::path m_filepath;
		seri::Animation m_anim;
		bool m_no_materials = {};

		LdrModel(seri::Name name, seri::Colour colour)
			: LdrBase(name, colour)
		{}

		LdrModel& filepath(std::filesystem::path filepath)
		{
			m_filepath = filepath;
			return *this;
		}
		seri::Animation& anim()
		{
			return m_anim;
		}
		LdrModel& no_materials(bool on = true)
		{
			m_no_materials = on;
			return *this;
		}

		// Write to 'out'
		virtual void Write(textbuf& out) const override
		{
			Append(out, EKeywords::Model, m_name, m_colour, "{");
			{
				Append(out, EKeywords::FilePath, std::format("{{\"{}\"}}", m_filepath.string()));
				if (m_anim) Append(out, m_anim);
				if (m_no_materials) Append(out, EKeywords::NoMaterials, "{}" );
				LdrBase::Write(out);
			}
			Append(out, "}");
		}
	};
	struct LdrGroup : LdrBase
	{
		LdrGroup(seri::Name name, seri::Colour colour)
			:LdrBase(name, colour)
		{}
		virtual void Write(textbuf& out) const override
		{
			Append(out, "*Group", m_name, m_colour, "{");
			LdrBase::Write(out);
			Append(out, "}");
		}
	};
	struct Builder : LdrBase
	{
		void Save(std::filesystem::path filepath, ESaveFlags flags = ESaveFlags::None) const
		{
			try
			{
				std::default_random_engine rng;
				std::uniform_int_distribution<uint64_t> dist(0);
				auto tmp_path = filepath.parent_path() / std::format("{}.tmp", dist(rng));

				auto binary = (int64_t(flags) & int64_t(ESaveFlags::Binary)) != 0;
				auto append = (int64_t(flags) & int64_t(ESaveFlags::Append)) != 0;
				auto pretty = (int64_t(flags) & int64_t(ESaveFlags::Pretty)) != 0;

				if (!std::filesystem::exists(tmp_path.parent_path()))
					std::filesystem::create_directories(tmp_path.parent_path());

				if (binary)
				{
					bytebuf out;
					Write(out);
					if (!out.empty())
					{
						std::filesystem::create_directories(tmp_path.parent_path());
						std::ofstream file(tmp_path, (append ? std::ios::app : std::ios::out) | std::ios::binary);
						file.write(reinterpret_cast<char const*>(out.data()), static_cast<std::streamsize>(out.size()));
						file.close();
					}
				}
				else
				{
					textbuf out;
					Write(out);
					if (pretty) out = FormatScript(out);
					if (!out.empty())
					{
						std::filesystem::create_directories(tmp_path.parent_path());
						std::ofstream file(tmp_path, append ? std::ios::app : std::ios::out);
						file.write(reinterpret_cast<char const*>(out.data()), static_cast<std::streamsize>(out.size()));
						file.close();
					}
				}

				auto outpath = filepath;
				if (outpath.has_extension() == false)
					outpath.replace_extension(binary ? ".bdr" : ".ldr");

				// Replace/rename
				std::filesystem::rename(tmp_path, outpath);
				// 'std::filesystem::rename' might not replace existing files on windows...
				//MoveFileExW(fpath.c_str(), outpath.c_str(), MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH);
			}
			catch (std::exception const& ex)
			{
				if ((int64_t(flags) & int64_t(ESaveFlags::NoThrowOnFailure)) == 0) throw;
				std::cerr << "LDraw save failed: " << ex.what();
			}
		}

		textbuf& ToString(textbuf& out, ESaveFlags flags = ESaveFlags::None) const
		{
			try
			{
				auto append = (int64_t(flags) & int64_t(ESaveFlags::Append)) != 0;
				auto pretty = (int64_t(flags) & int64_t(ESaveFlags::Pretty)) != 0;

				if (!append) out.resize(0);
				Write(out);
				if (pretty) out = FormatScript(out);
			}
			catch (std::exception const& ex)
			{
				if ((int64_t(flags) & int64_t(ESaveFlags::NoThrowOnFailure)) == 0) throw;
				std::cerr << "LDraw save failed: " << ex.what();
			}
			return out;
		}
		textbuf ToString(ESaveFlags flags = ESaveFlags::None) const
		{
			textbuf out;
			return ToString(out, flags);
		}

		bytebuf& ToBinary(bytebuf& out, ESaveFlags flags = ESaveFlags::None) const
		{
			try
			{
				auto append = (int64_t(flags) & int64_t(ESaveFlags::Append)) != 0;
				if (!append) out.resize(0);
				Write(out);
			}
			catch (std::exception const& ex)
			{
				if ((int64_t(flags) & int64_t(ESaveFlags::NoThrowOnFailure)) == 0) throw;
				std::cerr << "LDraw save failed: " << ex.what();
			}
			return out;
		}
		bytebuf ToBinary(ESaveFlags flags = ESaveFlags::None) const
		{
			bytebuf out;
			return ToBinary(out, flags);
		}

		// Pretty format Ldraw script
		auto FormatScript(TString auto& str) const
		{
			std::remove_reference_t<decltype(str)> out = {};
			out.reserve(str.size());

			constexpr int MaxShortLine = 80;
			std::optional<size_t> shortline;
			
			int indent = 0;
			for (auto c : str)
			{
				if (c == '{')
				{
					shortline = out.size(); // Record the position of the '{' character

					++indent;
					out.push_back(c);
					out.append(1, '\n').append(indent, '\t');
				}
				else if (c == '}')
				{
					--indent;
					out.append(1, '\n').append(indent, '\t');
					out.push_back(c);

					// Remove extra whitespace, turn "{ \n  data   data\t\t\n data \n  }" into "{data data data}"
					if (shortline)
					{
						auto end = out.size();
						for (size_t r = *shortline + 1, w = r;;) //read, write
						{
							for (; r != end && isspace(out[r]); ++r) {} // eat whitespace
							for (; r != end && !isspace(out[r]);) out[w++] = out[r++]; // copy non-whitespace
							for (; r != end && isspace(out[r]); ++r) {} // eat whitespace
							if (r == end) { out.resize(out.size() - (r - w)); break; }
							if (r + 1 != end) out[w++] = ' ';
						}
						shortline = {};
					}
				}
				else
				{
					if (!out.empty() && out.back() == '}')
						out.append(1, '\n').append(indent, '\t');
					if (!out.empty() && out.back() == '\t' && isspace(c))
						continue;

					out.push_back(c);
					if (shortline && (out.size() - *shortline > MaxShortLine || c == '{' || c == '*'))
						shortline = {};
				}
			}
			return out;
		}
	};

	// Implementation
	#pragma region Implementation
	inline LdrPoint& LdrBase::Point(seri::Name name, seri::Colour colour)
	{
		m_children.push_back(ObjPtr{ new LdrPoint(name, colour) });
		return *static_cast<LdrPoint*>(m_children.back().get());
	}
	inline LdrLine& LdrBase::Line(seri::Name name, seri::Colour colour)
	{
		m_children.push_back(ObjPtr{ new LdrLine(name, colour) });
		return *static_cast<LdrLine*>(m_children.back().get());
	}
	inline LdrBox& LdrBase::Box(seri::Name name, seri::Colour colour)
	{
		m_children.push_back(ObjPtr{ new LdrBox(name, colour) });
		return *static_cast<LdrBox*>(m_children.back().get());
	}
	inline LdrModel& LdrBase::Model(seri::Name name, seri::Colour colour)
	{
		m_children.push_back(ObjPtr{ new LdrModel(name, colour) });
		return *static_cast<LdrModel*>(m_children.back().get());
	}
	inline LdrGroup& LdrBase::Group(seri::Name name, seri::Colour colour)
	{
		m_children.push_back(ObjPtr{ new LdrGroup(name, colour) });
		return *static_cast<LdrGroup*>(m_children.back().get());
	}

	namespace seri
	{
		inline O2W& O2W::o2w(Mat4 o2w)
		{
			LdrBase::Append(m_xform_txt, "*M4x4 {", o2w, "}");
			auto s = LdrBase::Append(m_xform_bin, seri::Header{ EKeywords::M4x4 });
			LdrBase::Append(m_xform_bin, o2w);
			return *this;
		}
		inline O2W& O2W::rot(Mat3 rot)
		{
			LdrBase::Append(m_xform_txt, "*M3x3 {", rot, "}");
			return *this;
		}
		inline O2W& O2W::align(Vec3 dir, AxisId axis)
		{
			LdrBase::Append(m_xform_txt, "*Align {", *axis.m_id, dir, "}");
			return *this;
		}
		inline O2W& O2W::lookat(Vec3 pos)
		{
			LdrBase::Append(m_xform_txt, "*Lookat {", pos, "}");
			return *this;
		}
		inline O2W& O2W::quat(Vec4 q)
		{
			LdrBase::Append(m_xform_txt, "*Quat {", q, "}");
			return *this;
		}
		inline O2W& O2W::pos(Vec3 pos)
		{
			LdrBase::Append(m_xform_txt, "*Pos {", pos, "}");
			return *this;
		}
		inline O2W& O2W::pos(float x, float y, float z)
		{
			LdrBase::Append(m_xform_txt, "*Pos {", x, y, z, "}");
			return *this;
		}
		inline O2W& O2W::scale(Vec3 scale)
		{
			LdrBase::Append(m_xform_txt, "*Scale {", scale.x, scale.y, scale.z, "}");
			return *this;
		}
		inline O2W& O2W::scale(float sx, float sy, float sz)
		{
			LdrBase::Append(m_xform_txt, "*Scale {", sx, sy, sz, "}");
			return *this;
		}
		inline O2W& O2W::scale(float s)
		{
			LdrBase::Append(m_xform_txt, "*Scale {", s, s, s, "}");
			return *this;
		}
		inline O2W& O2W::euler(float pitch_deg, float yaw_deg, float roll_deg)
		{
			LdrBase::Append(m_xform_txt, "*Euler {", pitch_deg, yaw_deg, roll_deg, "}");
			return *this;
		}
		inline O2W& O2W::rand(Vec3 centre, float radius)
		{
			LdrBase::Append(m_xform_txt, "*Rand4x4 {", centre, radius, "}");
			return *this;
		}
		inline O2W& O2W::rand_pos(Vec3 centre, float radius)
		{
			LdrBase::Append(m_xform_txt, "*RandPos {", centre, radius, "}");
			return *this;
		}
		inline O2W& O2W::rand_ori()
		{
			LdrBase::Append(m_xform_txt, "*RandOri {}");
			return *this;
		}
		inline O2W& O2W::normalise()
		{
			LdrBase::Append(m_xform_txt, "*Normalise {}");
			return *this;
		}
		inline O2W& O2W::orthonormalise()
		{
			LdrBase::Append(m_xform_txt, "*Orthonormalise {}");
			return *this;
		}
		inline O2W& O2W::transpose()
		{
			LdrBase::Append(m_xform_txt, "*Transpose {}");
			return *this;
		}
		inline O2W& O2W::inverse()
		{
			LdrBase::Append(m_xform_txt, "*Inverse {}");
			return *this;
		}
		inline O2W& O2W::non_affine()
		{
			LdrBase::Append(m_xform_txt, "*NonAffine {}");
			return *this;
		}

		inline Texture& Texture::filepath(std::string_view filepath)
		{
			LdrBase::Append(m_tex, "*FilePath {", filepath, "}");
			return *this;
		}
		inline Texture& Texture::addr(std::string_view mode) // Wrap|Mirror|Clamp|Border|MirrorOnce
		{
			LdrBase::Append(m_tex, "*Addr {", mode, mode, "}");
			return *this;
		}
		inline Texture& Texture::addr(std::string_view modeU, std::string_view modeV)
		{
			LdrBase::Append(m_tex, "*Addr {", modeU, modeV, "}");
			return *this;
		}
		inline Texture& Texture::filter(std::string_view filter) // Point|Linear|Anisotropic
		{
			LdrBase::Append(m_tex, "*Filter {", filter, "}");
			return *this;
		}
		inline Texture& Texture::Alpha(bool on)
		{
			LdrBase::Append(m_tex, "*Alpha {", on, "}");
			return *this;
		}

		inline RootAnimation& RootAnimation::style(std::string_view style)
		{
			LdrBase::Append(m_anim, "*Style {", style, "}");
			return *this;
		}
		inline RootAnimation& RootAnimation::period(float seconds)
		{
			LdrBase::Append(m_anim, "*Period {", seconds, "}");
			return *this;
		}
		inline RootAnimation& RootAnimation::velocity(Vec3 vel)
		{
			LdrBase::Append(m_anim, "*Velocity {", vel, "}");
			return *this;
		}
		inline RootAnimation& RootAnimation::acceleration(Vec3 accel)
		{
			LdrBase::Append(m_anim, "*Accel {", accel, "}");
			return *this;
		}
		inline RootAnimation& RootAnimation::ang_velocity(Vec3 ang_vel)
		{
			LdrBase::Append(m_anim, "*AngVelocity {", ang_vel, "}");
			return *this;
		}
		inline RootAnimation& RootAnimation::ang_acceleration(Vec3 ang_accel)
		{
			LdrBase::Append(m_anim, "*AngAccel {", ang_accel, "}");
			return *this;
		}

		inline Animation& Animation::style(std::string_view style)
		{
			LdrBase::Append(m_anim, "*Style {", style, "}");
			return *this;
		}
		inline Animation& Animation::frame(int frame)
		{
			LdrBase::Append(m_anim, "*Frame {", frame, "}");
			return *this;
		}
		inline Animation& Animation::frame_range(int start, int end)
		{
			LdrBase::Append(m_anim, "*FrameRange {", start, end, "}");
			return *this;
		}
		inline Animation& Animation::time_range(float start, float end)
		{
			LdrBase::Append(m_anim, "*TimeRange {", start, end, "}");
			return *this;
		}
		inline Animation& Animation::stretch(float speed_multiplier)
		{
			LdrBase::Append(m_anim, "*Stretch {", speed_multiplier, "}");
			return *this;
		}
		inline Animation& Animation::no_translation()
		{
			LdrBase::Append(m_anim, "*NoRootTranslation {}");
			return *this;
		}
		inline Animation& Animation::no_rotation()
		{
			LdrBase::Append(m_anim, "*NoRootRotation {}");
			return *this;
		}
	}
	#pragma endregion
}

#if PR_UNITTESTS && !PR_UNITTESTS_VISUALISE
#include "pr/common/unittests.h"
namespace pr::ldraw
{
	PRUnitTestClass(LdrawBuilder)
	{
		PRUnitTestMethod(Point)
		{
			Builder builder;
			builder.Point("p", 0xFF00FF00)
				.style(EPointStyle::Star)
				.pt(1, 2, 3)
				.pt({ 2, 3, 4 }, 0xFFFF0000)
				.size({ 0.1f, 0.3f })
				.depth()
				.o2w().euler(10, 20, 30).pos({ -1,-1,-1 });
			auto ldr = builder.ToString(ESaveFlags::Pretty);
			PR_EXPECT(ldr ==
				"*Point p ff00ff00 {\n"
				"	*Style {Star}\n"
				"	*Size {0.1 0.3}\n"
				"	*Depth {true}\n"
				"	*PerItemColour {true}\n"
				"	*Data {1 2 3 ffffffff 2 3 4 ffff0000}\n"
				"	*O2W {\n"
				"		*Euler {10 20 30}\n"
				"		*Pos {-1 -1 -1}\n"
				"	}\n"
				"}");
			auto bdr = builder.ToBinary();

		}
		PRUnitTestMethod(Line)
		{
			Builder builder;
			builder.Line("l", 0xFF00FF00)
				.style(ELineStyle::LineStrip)
				.per_item_colour()
				.width(10)
				.dashed({ 0.2f, 0.4f })
				.arrow(EArrowType::Fwd, 5.0f)
				.line({ -1, -1, -1 }, { +1, +1, +1 }, 0xFFFF0000)
				.line({ -1, +1, -1 }, { +1, -1, +1 }, 0xFF0000FF)
				.new_block()
				.strip({ -1, -1, -1 })
				.line_to({ +1, -1, -1 })
				.line_to({ +1, +1, -1 })
				.line_to({ -1, +1, -1 });
			auto ldr = builder.ToString(ESaveFlags::Pretty);
			PR_EXPECT(ldr ==
				"*Line l ff00ff00 {\n"
				"	*Style {LineSegments}\n"
				"	*Width {10}\n"
				"	*Dashed {0.2 0.4}\n"
				"	*Arrow {Fwd 5}\n"
				"	*PerItemColour {true}\n"
				"	*Data {-1 -1 -1 1 1 1 ffff0000 -1 1 -1 1 -1 1 ff0000ff}\n"
				"	*Style {LineStrip}\n"
				"	*Width {10}\n"
				"	*Dashed {0.2 0.4}\n"
				"	*Arrow {Fwd 5}\n"
				"	*PerItemColour {true}\n"
				"	*Data {-1 -1 -1 ffffffff 1 -1 -1 ffffffff 1 1 -1 ffffffff -1 1 -1 ffffffff}\n"
				"}");
		}
		PRUnitTestMethod(Box)
		{
			Builder builder;
			builder.Box("b", 0xFF00FF00).box(1, 2, 3);
			auto ldr = builder.ToString(ESaveFlags::Pretty);
			PR_EXPECT(ldr ==
				"*Box b ff00ff00 {\n"
				"	*Data {1 2 3}\n"
				"}");
		}
		PRUnitTestMethod(Model)
		{
			Builder builder;
			builder.Model("m").filepath("my_model.fbx").no_materials().anim().frame(10);
			auto ldr = builder.ToString(ESaveFlags::Pretty);
			PR_EXPECT(ldr == 
				"*Model m {\n"
				"	*FilePath {\"my_model.fbx\"}\n"
				"	*Animation {\n"
				"		*Frame {10}\n"
				"	}\n"
				"	*NoMaterials {}\n"
				"}");
		}
		PRUnitTestMethod(Group)
		{
			Builder builder;
			auto& grp = builder.Group("g");
			grp.Box("b", 0xFF00FF00).box({ 1, 2, 3 }, { 1, 1, 1 }, 0xFF00FF00);
			auto ldr = builder.ToString(ESaveFlags::Pretty);
			PR_EXPECT(ldr ==
				"*Group g {\n"
				"	*BoxList b ff00ff00 {\n"
				"		*PerItemColour {}\n"
				"		*Data {1 2 3 1 1 1 ff00ff00}\n"
				"	}\n"
				"}");
		}
	};
}
#endif
