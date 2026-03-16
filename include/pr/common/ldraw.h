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
#include <variant>

// Only standard library headers

namespace pr::ldraw
{
	struct LdrBase;
	struct LdrGroup;
	struct LdrInstance;
	struct LdrText;
	struct LdrLightSource;
	struct LdrPoint;
	struct LdrLine;
	struct LdrLineBox;
	struct LdrGrid;
	struct LdrCoordFrame;
	struct LdrCircle;
	struct LdrPie;
	struct LdrRect;
	struct LdrPolygon;
	struct LdrTriangle;
	struct LdrQuad;
	struct LdrPlane;
	struct LdrRibbon;
	struct LdrBox;
	struct LdrFrustum;
	struct LdrSphere;
	struct LdrCylinder;
	struct LdrCone;
	struct LdrMesh;
	struct LdrConvexHull;
	struct LdrModel;
	struct LdrCommands;
	struct LdrBinaryStream;
	struct LdrTextStream;

	using ObjPtr = std::shared_ptr<LdrBase>;
	using textbuf = std::string;
	using bytebuf = std::vector<std::byte>;
	using outbuf = std::variant<textbuf, bytebuf>;

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

	// Serializing helpers
	namespace seri
	{
		//TODO: Use EKeywords not *Keyword...
		// Notes:
		//  - There's two methodologies here:
		//     (1) - create a specific type for each keyword and an overload of Append that knows how to write that type,
		//     (2) - create a type that writes to a sub-buffer, which is then written to the output.
		//   Both have pros/cons:
		//     - (1) buffers information until we know whether we're writting text or binary out. (2) Is required to write
		//        both text and binary because we don't know which will be needed.
		//     - (2) means we don't need types for super specific keywords that only occur in one place (e.g. O2W sub-keywords)
		//   (1) is the better choice, it just requires more code.
		struct Header;

		// Convert to std::byte*
		template <typename T> constexpr std::byte const* byte_ptr(T const* t)
		{
			return reinterpret_cast<std::byte const*>(t);
		}
		template <typename T> constexpr std::byte* byte_ptr(T* t)
		{
			return reinterpret_cast<std::byte*>(t);
		}

		// Convert values to string
		template <typename T, int N> std::string_view conv(char(&buf)[N], T value)
		{
			auto [ptr, ec] = std::to_chars(&buf[0], &buf[0] + N, value);
			if (ec != std::errc{}) throw std::system_error(std::make_error_code(ec));
			return std::string_view{ &buf[0], static_cast<size_t>(ptr - &buf[0]) };
		}
		template <typename T, int N> std::string_view conv(char(&buf)[N], T value, int base)
		{
			auto [ptr, ec] = std::to_chars(&buf[0], &buf[0] + N, value, base);
			if (ec != std::errc{}) throw std::system_error(std::make_error_code(ec));
			return std::string_view{ &buf[0], static_cast<size_t>(ptr - &buf[0]) };
		}
		template <typename T> std::string conv(T value)
		{
			if constexpr (requires (T t) { { ToString(t) } -> std::convertible_to<std::string>; })
				return ToString(value);
			else if constexpr (requires (T t) { { To<std::string>(t) } -> std::convertible_to<std::string>; })
				return To<std::string>(value);
			else if constexpr (std::convertible_to<T, std::string_view>)
				return std::string(std::string_view(value));
			else
				static_assert(false, "No conv() overload for type");
		}
		inline std::string conv(std::string_view value)
		{
			return std::string(value);
		}

		// Concept for types that can be converted to string via conv()
		template <typename T> concept TToString = requires (T t)
		{
			{ conv(t) } -> std::convertible_to<std::string>;
		};

		// Append helpers 
		template <typename T> struct no_overload;
		template <typename Type> static void Append(textbuf& out, Type) { no_overload<Type> missing_overload; }
		template <typename Type> static void Append(bytebuf& out, Type) { no_overload<Type> missing_overload; }
		template <typename... Args> void Append(textbuf& out, Args&&... args);
		template <typename... Args> void Append(bytebuf& out, Args&&... args);
		void Append(bytebuf& out, std::span<std::byte const> data, int64_t ofs = -1);

		inline void Append(bytebuf& out, std::string_view s)
		{
			Append(out, { byte_ptr(s.data()), size_t(s.size()) });
		}
		inline void Append(textbuf& out, std::string_view s)
		{
			if (s.empty()) return;
			if (*s.data() != '}' && *s.data() != ')' && s.front() != ' ' && !out.empty() && out.back() != '{')
			{
				out.append(1, ' ');
			}
			out.append(s);
		}
		inline void Append(bytebuf& out, std::string s)
		{
			Append(out, std::string_view{ s });
		}
		inline void Append(textbuf& out, std::string s)
		{
			Append(out, std::string_view{ s });
		}
		inline void Append(bytebuf& out, char const* s)
		{
			Append(out, std::string_view{ s });
		}
		inline void Append(textbuf& out, char const* s)
		{
			Append(out, std::string_view{ s });
		}
		inline void Append(textbuf& out, bool b)
		{
			Append(out, b ? "true" : "false");
		}
		inline void Append(bytebuf& out, int8_t i)
		{
			Append(out, { byte_ptr(&i), sizeof(i) });
		}
		inline void Append(bytebuf& out, int16_t i)
		{
			Append(out, { byte_ptr(&i), sizeof(i) });
		}
		inline void Append(bytebuf& out, int32_t i)
		{
			Append(out, { byte_ptr(&i), sizeof(i) });
		}
		inline void Append(bytebuf& out, int64_t i)
		{
			Append(out, { byte_ptr(&i), sizeof(i) });
		}
		inline void Append(bytebuf& out, uint8_t u)
		{
			Append(out, { byte_ptr(&u), sizeof(u) });
		}
		inline void Append(bytebuf& out, bool b)
		{
			Append(out, uint8_t(b ? 1 : 0));
		}
		inline void Append(bytebuf& out, uint16_t u)
		{
			Append(out, { byte_ptr(&u), sizeof(u) });
		}
		inline void Append(bytebuf& out, uint32_t u)
		{
			Append(out, { byte_ptr(&u), sizeof(u) });
		}
		inline void Append(bytebuf& out, uint64_t u)
		{
			Append(out, { byte_ptr(&u), sizeof(u) });
		}
		inline void Append(bytebuf& out, float f)
		{
			Append(out, { byte_ptr(&f), sizeof(f) });
		}
		inline void Append(bytebuf& out, double f)
		{
			Append(out, { byte_ptr(&f), sizeof(f) });
		}
		inline void Append(textbuf& out, int i)
		{
			char ch[32];
			Append(out, conv(ch, i));
		}
		inline void Append(textbuf& out, long i)
		{
			char ch[32];
			Append(out, conv(ch, i));
		}
		inline void Append(textbuf& out, float f)
		{
			char ch[32];
			Append(out, conv(ch, f));
		}
		inline void Append(textbuf& out, double f)
		{
			char ch[32];
			Append(out, conv(ch, f));
		}
		inline void Append(textbuf& out, uint32_t u)
		{
			char ch[32];
			Append(out, conv(ch, u, 16));
		}
		inline void Append(textbuf& out, EKeyword kw)
		{
			Append(out, std::string_view{ kw.name });
		}
		inline void Append(bytebuf& out, EKeyword kw)
		{
			Append(out, { byte_ptr(&kw.value), sizeof(kw.value) });
		}

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
			explicit operator bool() const
			{
				return m_name.has_value();
			}
			friend void Append(bytebuf& out, seri::Name n); // Defined after Header
			friend void Append(textbuf& out, seri::Name n)
			{
				if (!n) return;
				if (!n.m_kw)
					Append(out, *n.m_name);
				else
					Append(out, n.m_kw, "{", *n.m_name, "}");
			}
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
			friend void Append(bytebuf& out, seri::Colour c); // Defined after Header
			friend void Append(textbuf& out, seri::Colour c)
			{
				if (!c) return;
				if (!c.m_kw)
					Append(out, *c.m_colour);
				else if (c.m_kw == EKeywords::RandColour)
					Append(out, c.m_kw, "{}");
				else
					Append(out, c.m_kw, "{", *c.m_colour, "}");
			}
		};
		struct Header
		{
			EKeyword m_kw;
			Name m_name;
			Colour m_colour;

			friend auto Append(bytebuf& out, seri::Header hdr)
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
		};

		// Deferred definitions (these construct Header, which contains Name and Colour members)
		inline void Append(bytebuf& out, Name n)
		{
			if (!n) return;
			auto kw = n.m_kw ? n.m_kw : EKeywords::Name;
			Append(out, seri::Header{ kw }, *n.m_name);
		}
		inline void Append(bytebuf& out, Colour c)
		{
			if (!c) return;
			auto kw = c.m_kw ? c.m_kw : EKeywords::Colour;
			if (kw == EKeywords::RandColour)
				Append(out, seri::Header{ kw });
			else
				Append(out, seri::Header{ kw }, *c.m_colour);
		}

		// Variadic Append for binary header + payload
		template <typename... Args> auto Append(bytebuf& out, Header hdr, Args&&... args)
		{
			auto s = Append(out, hdr);
			(Append(out, std::forward<Args>(args)), ...);
			return std::move(s);
		}

		struct Vec2
		{
			float x, y;
			Vec2() : x(), y() {}
			Vec2(float x_, float y_) : x(x_), y(y_) {}
			Vec2(TVec2 auto v) :x(v.x), y(v.y) {}
			explicit operator bool() const { return x != 0 || y != 0; }
			friend auto operator <=> (Vec2 const& lhs, Vec2 const& rhs) = default;
			friend void Append(bytebuf& out, seri::Vec2 v)
			{
				Append(out, { byte_ptr(&v), sizeof(v) });
			}
			friend void Append(textbuf& out, seri::Vec2 v)
			{
				Append(out, v.x, v.y);
			}
		};
		struct Vec3
		{
			float x, y, z;
			Vec3() : x(), y(), z() {}
			Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
			Vec3(TVec3 auto v) :x(v.x), y(v.y), z(v.z) {}
			explicit operator bool() const { return x != 0 || y != 0 || z != 0; }
			friend auto operator <=> (Vec3 const& lhs, Vec3 const& rhs) = default;
			friend void Append(bytebuf& out, seri::Vec3 v)
			{
				Append(out, { byte_ptr(&v), sizeof(v) });
			}
			friend void Append(textbuf& out, seri::Vec3 v)
			{
				Append(out, v.x, v.y, v.z);
			}
		};
		struct Vec4
		{
			float x, y, z, w;
			Vec4() : x(), y(), z(), w() {}
			Vec4(float x_, float y_, float z_, float w_) : x(x_), y(y_), z(z_), w(w_) {}
			Vec4(TVec4 auto v) :x(v.x), y(v.y), z(v.z), w(v.w) {}
			explicit operator bool() const { return x != 0 || y != 0 || z != 0 || w != 0; }
			friend auto operator <=> (Vec4 const& lhs, Vec4 const& rhs) = default;
			friend float length(Vec4 v)
			{
				return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z + v.w * v.w);
			}
			friend Vec4 normalise(Vec4 v)
			{
				auto len = length(v);
				return len > 0 ? Vec4(v.x / len, v.y / len, v.z / len, v.w / len) : Vec4();
			}
			friend void Append(bytebuf& out, seri::Vec4 v)
			{
				Append(out, { byte_ptr(&v), sizeof(v) });
			}
			friend void Append(textbuf& out, seri::Vec4 v)
			{
				Append(out, v.x, v.y, v.z, v.w);
			}
		};
		struct Mat3
		{
			Vec3 x, y, z;
			Mat3() : x(), y(), z() {}
			Mat3(Vec3 const& x_, Vec3 const& y_, Vec3 const& z_) : x(x_), y(y_), z(z_) {}
			Mat3(TMat3 auto v) :x(v.x), y(v.y), z(v.z) {}
			explicit operator bool() const { return x != Vec3{1, 0, 0} || y != Vec3(0,1,0) || z != Vec3(0,0,1); }
			friend auto operator <=> (Mat3 const& lhs, Mat3 const& rhs) = default;
			friend void Append(textbuf& out, seri::Mat3 m)
			{
				Append(out, m.x, m.y, m.z);
			}
			friend void Append(bytebuf& out, seri::Mat3 m)
			{
				Append(out, { byte_ptr(&m), sizeof(m) });
			}
		};
		struct Mat4
		{
			Vec4 x, y, z, w;
			Mat4() : x(), y(), z(), w() {}
			Mat4(Vec4 const& x_, Vec4 const& y_, Vec4 const& z_, Vec4 const& w_) : x(x_), y(y_), z(z_), w(w_) {}
			Mat4(TMat4 auto v) :x(v.x), y(v.y), z(v.z), w(v.w) {}
			explicit operator bool() const { return x != Vec4{1, 0, 0, 0} || y != Vec4(0,1,0,0) || z != Vec4(0,0,1,0) || w != Vec4(0,0,0,1); }
			friend auto operator <=> (Mat4 const& lhs, Mat4 const& rhs) = default;
			friend void Append(bytebuf& out, seri::Mat4 m)
			{
				Append(out, { byte_ptr(&m), sizeof(m) });
			}
			friend void Append(textbuf& out, seri::Mat4 m)
			{
				Append(out, m.x, m.y, m.z, m.w);
			}
		};
		struct Size
		{
			float m_size;
			Size() : m_size() {}
			Size(float size) : m_size(size) {}
			Size(int size) : m_size(float(size)) {}
			explicit operator bool() const { return m_size != 0; }
			friend void Append(bytebuf& out, seri::Size sz)
			{
				if (!sz) return;
				auto s = Append(out, seri::Header{ EKeywords::Size });
				Append(out, sz.m_size);
			}
			friend void Append(textbuf& out, seri::Size s)
			{
				if (!s) return;
				Append(out, EKeywords::Size, "{", s.m_size, "}");
			}
		};
		struct Size2
		{
			Vec2 m_size;
			Size2() : m_size() {}
			Size2(Vec2 size) : m_size(size) {}
			Size2(float sx, float_t sy) : m_size(sx, sy) {}
			explicit operator bool() const { return !!m_size; }
			friend void Append(bytebuf& out, seri::Size2 sz)
			{
				if (!sz) return;
				auto s = Append(out, seri::Header{ EKeywords::Size });
				Append(out, sz.m_size);
			}
			friend void Append(textbuf& out, seri::Size2 s)
			{
				if (!s) return;
				Append(out, EKeywords::Size, "{", s.m_size, "}");
			}
		};
		struct Scale
		{
			float m_scale;
			Scale() : m_scale(1) {}
			Scale(float scale) : m_scale(scale) {}
			explicit operator bool() const { return m_scale != 1; }
			friend void Append(bytebuf& out, seri::Scale sc)
			{
				if (!sc) return;
				auto s = Append(out, seri::Header{ EKeywords::Scale });
				Append(out, sc.m_scale);
			}
			friend void Append(textbuf& out, seri::Scale s)
			{
				if (!s) return;
				Append(out, EKeywords::Scale, "{", s.m_scale, "}");
			}
		};
		struct Scale2
		{
			Vec2 m_scale;
			Scale2() : m_scale(1, 1) {}
			Scale2(Vec2 scale) : m_scale(scale) {}
			Scale2(float sx, float sy) : m_scale(sx, sy) {}
			explicit operator bool() const { return !!m_scale; }
			friend void Append(bytebuf& out, seri::Scale2 sc)
			{
				if (!sc) return;
				auto s = Append(out, seri::Header{ EKeywords::Scale });
				Append(out, sc.m_scale);
			}
			friend void Append(textbuf& out, seri::Scale2 s)
			{
				if (!s) return;
				Append(out, EKeywords::Scale, "{", s.m_scale, "}");
			}
		};
		struct Scale3
		{
			Vec3 m_scale;
			Scale3() : m_scale(1, 1, 1) {}
			Scale3(Vec3 scale) :m_scale(scale) {}
			explicit operator bool() const { return !!m_scale; }
			friend void Append(bytebuf& out, seri::Scale3 sc)
			{
				if (!sc) return;
				auto s = Append(out, seri::Header{ EKeywords::Scale });
				Append(out, sc.m_scale);
			}
			friend void Append(textbuf& out, seri::Scale3 s)
			{
				if (!s) return;
				Append(out, EKeywords::Scale, "{", s.m_scale, "}");
			}
		};
		struct PerItemColour
		{
			std::optional<bool> m_active;
			PerItemColour() : m_active() {}
			PerItemColour(bool has_colours) : m_active(has_colours) {}
			explicit operator bool() const { return m_active.has_value(); }
			friend void Append(bytebuf& out, seri::PerItemColour c)
			{
				if (!c) return;
				auto s = Append(out, seri::Header{ EKeywords::PerItemColour });
				Append(out, *c.m_active);
			}
			friend void Append(textbuf& out, seri::PerItemColour c)
			{
				if (!c) return;
				Append(out, EKeywords::PerItemColour, "{", *c.m_active, "}");
			}
		};
		struct Width
		{
			std::optional<float> m_width;
			Width() :m_width() {}
			Width(float w) :m_width(w) {}
			explicit operator bool() const { return m_width.has_value(); }
			friend void Append(bytebuf& out, seri::Width w)
			{
				if (!w) return;
				auto s = Append(out, seri::Header{ EKeywords::Width });
				Append(out, *w.m_width);
			}
			friend void Append(textbuf& out, seri::Width w)
			{
				if (!w) return;
				Append(out, EKeywords::Width, "{", *w.m_width, "}");
			}
		};
		struct Depth
		{
			std::optional<bool> m_depth;
			Depth() :m_depth() {}
			Depth(bool d) : m_depth(d) {}
			explicit operator bool() const { return m_depth.has_value(); }
			friend void Append(bytebuf& out, seri::Depth d)
			{
				if (!d) return;
				auto s = Append(out, seri::Header{ EKeywords::Depth });
				Append(out, *d.m_depth);
			}
			friend void Append(textbuf& out, seri::Depth d)
			{
				if (!d) return;
				Append(out, EKeywords::Depth, "{", *d.m_depth, "}");
			}
		};
		struct Hidden
		{
			std::optional<bool> m_hide;
			Hidden() :m_hide() {}
			Hidden(bool h) :m_hide(h) {}
			explicit operator bool() const { return m_hide.has_value(); }
			friend void Append(bytebuf& out, seri::Hidden h)
			{
				if (!h) return;
				auto s = Append(out, seri::Header{ EKeywords::Hidden });
				Append(out, *h.m_hide);
			}
			friend void Append(textbuf& out, seri::Hidden h)
			{
				if (!h) return;
				Append(out, EKeywords::Hidden, "{", *h.m_hide, "}");
			}
		};
		struct Wireframe
		{
			std::optional<bool> m_wire;
			Wireframe() :m_wire() {}
			Wireframe(bool w) :m_wire(w) {}
			explicit operator bool() const { return m_wire.has_value(); }
			friend void Append(bytebuf& out, seri::Wireframe w)
			{
				if (!w) return;
				auto s = Append(out, seri::Header{ EKeywords::Wireframe });
				Append(out, *w.m_wire);
			}
			friend void Append(textbuf& out, seri::Wireframe w)
			{
				if (!w) return;
				Append(out, EKeywords::Wireframe, "{", *w.m_wire, "}");
			}
		};
		struct Alpha
		{
			std::optional<bool> m_alpha;
			Alpha() :m_alpha() {}
			Alpha(bool a) : m_alpha(a) {}
			explicit operator bool() const { return m_alpha.has_value(); }
			friend void Append(bytebuf& out, seri::Alpha a)
			{
				if (!a) return;
				auto s = Append(out, seri::Header{ EKeywords::Alpha });
				Append(out, *a.m_alpha);
			}
			friend void Append(textbuf& out, seri::Alpha a)
			{
				if (!a) return;
				Append(out, EKeywords::Alpha, "{", *a.m_alpha, "}");
			}
		};
		struct Reflectivity
		{
			std::optional<float> m_refl;
			Reflectivity() : m_refl() {}
			Reflectivity(float r) : m_refl(r) {}
			explicit operator bool() const { return m_refl.has_value(); }
			friend void Append(bytebuf& out, seri::Reflectivity r)
			{
				if (!r) return;
				auto s = Append(out, seri::Header{ EKeywords::Reflectivity });
				Append(out, *r.m_refl);
			}
			friend void Append(textbuf& out, seri::Reflectivity r)
			{
				if (!r) return;
				Append(out, EKeywords::Reflectivity, "{", *r.m_refl, "}");
			}
		};
		struct Solid
		{
			std::optional<bool> m_solid;
			Solid() :m_solid() {}
			Solid(bool s) : m_solid(s) {}
			explicit operator bool() const { return m_solid.has_value(); }
			friend void Append(bytebuf& out, seri::Solid so)
			{
				if (!so) return;
				auto s = Append(out, seri::Header{ EKeywords::Solid });
				Append(out, *so.m_solid);
			}
			friend void Append(textbuf& out, seri::Solid s)
			{
				if (!s) return;
				Append(out, EKeywords::Solid, "{", *s.m_solid, "}");
			}
		};
		struct Smooth
		{
			std::optional<bool> m_smooth;
			Smooth() :m_smooth() {}
			Smooth(bool s) : m_smooth(s) {}
			explicit operator bool() const { return m_smooth.has_value(); }
			friend void Append(bytebuf& out, seri::Smooth sm)
			{
				if (!sm) return;
				auto s = Append(out, seri::Header{ EKeywords::Smooth });
				Append(out, *sm.m_smooth);
			}
			friend void Append(textbuf& out, seri::Smooth s)
			{
				if (!s) return;
				Append(out, EKeywords::Smooth, "{", *s.m_smooth, "}");
			}
		};
		struct Dashed
		{
			std::optional<Vec2> m_dash;
			Dashed() : m_dash() {}
			Dashed(Vec2 dash) : m_dash(dash) {}
			explicit operator bool() const { return m_dash.has_value(); }
			friend void Append(bytebuf& out, seri::Dashed d)
			{
				if (!d) return;
				auto s = Append(out, seri::Header{ EKeywords::Dashed });
				Append(out, *d.m_dash);
			}
			friend void Append(textbuf& out, seri::Dashed d)
			{
				if (!d) return;
				Append(out, EKeywords::Dashed, "{", *d.m_dash, "}");
			}
		};
		struct LeftHanded
		{
			std::optional<bool> m_lh;
			LeftHanded() :m_lh() {}
			LeftHanded(bool lh) : m_lh(lh) {}
			explicit operator bool() const { return m_lh.has_value(); }
			friend void Append(bytebuf& out, seri::LeftHanded lh)
			{
				if (!lh) return;
				auto s = Append(out, seri::Header{ EKeywords::LeftHanded });
				Append(out, *lh.m_lh);
			}
			friend void Append(textbuf& out, seri::LeftHanded lh)
			{
				if (!lh) return;
				Append(out, EKeywords::LeftHanded, "{", *lh.m_lh, "}");
			}
		};
		struct ScreenSpace
		{
			std::optional<bool> m_screen_space;
			ScreenSpace() :m_screen_space() {}
			ScreenSpace(bool screen_space) : m_screen_space(screen_space) {}
			explicit operator bool() const { return m_screen_space.has_value(); }
			friend void Append(bytebuf& out, seri::ScreenSpace ss)
			{
				if (!ss) return;
				auto s = Append(out, seri::Header{ EKeywords::ScreenSpace });
				Append(out, *ss.m_screen_space);
			}
			friend void Append(textbuf& out, seri::ScreenSpace ss)
			{
				if (!ss) return;
				Append(out, EKeywords::ScreenSpace, "{", *ss.m_screen_space, "}");
			}
		};
		struct NoZTest
		{
			std::optional<bool> m_no_ztest;
			NoZTest() :m_no_ztest() {}
			NoZTest(bool no_ztest) : m_no_ztest(no_ztest) {}
			explicit operator bool() const { return m_no_ztest.has_value(); }
			friend void Append(bytebuf& out, seri::NoZTest nzt)
			{
				if (!nzt) return;
				auto s = Append(out, seri::Header{ EKeywords::NoZTest });
				Append(out, *nzt.m_no_ztest);
			}
			friend void Append(textbuf& out, seri::NoZTest nzt)
			{
				if (!nzt) return;
				Append(out, EKeywords::NoZTest, "{", *nzt.m_no_ztest, "}");
			}
		};
		struct NoZWrite
		{
			std::optional<bool> m_no_zwrite;
			NoZWrite() :m_no_zwrite() {}
			NoZWrite(bool no_zwrite) : m_no_zwrite(no_zwrite) {}
			explicit operator bool() const { return m_no_zwrite.has_value(); }
			friend void Append(bytebuf& out, seri::NoZWrite nzw)
			{
				if (!nzw) return;
				auto s = Append(out, seri::Header{ EKeywords::NoZWrite });
				Append(out, *nzw.m_no_zwrite);
			}
			friend void Append(textbuf& out, seri::NoZWrite nzw)
			{
				if (!nzw) return;
				Append(out, EKeywords::NoZWrite, "{", *nzw.m_no_zwrite, "}");
			}
		};
		struct AxisId
		{
			inline static int const None = 0;
			inline static int const PosX = +1;
			inline static int const PosY = +2;
			inline static int const PosZ = +3;
			inline static int const NegX = -1;
			inline static int const NegY = -2;
			inline static int const NegZ = -3;

			std::optional<int> m_id;
			AxisId() : m_id() {}
			AxisId(int id) : m_id(id) {}
			explicit operator bool() const { return m_id.has_value(); }
			friend void Append(bytebuf& out, seri::AxisId a)
			{
				if (!a) return;
				auto s = Append(out, seri::Header{ EKeywords::AxisId });
				Append(out, *a.m_id);
			}
			friend void Append(textbuf& out, seri::AxisId a)
			{
				if (!a) return;
				Append(out, EKeywords::AxisId, "{", *a.m_id, "}");
			}
		};
		struct PointStyle
		{
			static constexpr std::string_view Styles[] = { "Square", "Circle", "Triangle", "Star", "Annulus" };
			std::optional<int> m_style;
			PointStyle() : m_style() {}
			PointStyle(TToString auto style) : m_style(static_cast<int>(std::distance(Styles, std::find(std::begin(Styles), std::end(Styles), conv(style))))) {}
			explicit operator bool() const { return m_style.has_value(); }
			friend void Append(bytebuf& out, seri::PointStyle p)
			{
				if (!p) return;
				auto s = Append(out, seri::Header{ EKeywords::Style });
				Append(out, *p.m_style);
			}
			friend void Append(textbuf& out, seri::PointStyle p)
			{
				if (!p) return;
				Append(out, EKeywords::Style, "{", Styles[*p.m_style], "}");
			}
		};
		struct LineStyle
		{
			static constexpr std::string_view Styles[] = { "LineSegments", "LineStrip", "Direction", "BezierSpline", "HermiteSpline", "BSplineSpline", "CatmullRom" };
			std::optional<int> m_style;
			LineStyle() : m_style() {}
			LineStyle(TToString auto style) : m_style(static_cast<int>(std::distance(Styles, std::find(std::begin(Styles), std::end(Styles), conv(style))))) {}
			explicit operator bool() const { return m_style.has_value(); }
			friend void Append(bytebuf& out, seri::LineStyle l)
			{
				if (!l) return;
				auto s = Append(out, seri::Header{ EKeywords::Style });
				Append(out, *l.m_style);
			}
			friend void Append(textbuf& out, seri::LineStyle l)
			{
				if (!l) return;
				Append(out, EKeywords::Style, "{", Styles[*l.m_style], "}");
			}
		};
		struct ArrowHeads
		{
			static constexpr std::string_view Styles[] = { "Line", "Fwd", "Back", "FwdBack" };
			std::optional<int> m_style;
			float m_size;
			ArrowHeads() : m_style(), m_size() {}
			ArrowHeads(TToString auto style, float size = 10) : m_style(static_cast<int>(std::distance(Styles, std::find(std::begin(Styles), std::end(Styles), conv(style))))), m_size(size) {}
			explicit operator bool() const { return m_style.has_value(); }
			friend void Append(bytebuf& out, seri::ArrowHeads a)
			{
				if (!a) return;
				auto s = Append(out, seri::Header{ EKeywords::Arrow });
				Append(out, *a.m_style, a.m_size);
			}
			friend void Append(textbuf& out, seri::ArrowHeads a)
			{
				if (!a) return;
				Append(out, EKeywords::Arrow, "{", Styles[*a.m_style], a.m_size, "}");
			}
		};
		struct DataPoints
		{
			Size2 m_size;
			Colour m_colour;
			PointStyle m_style;

			DataPoints() : m_size(), m_colour(), m_style() {}
			DataPoints(Vec2 size, Colour colour, PointStyle style) : m_size(size), m_colour(colour), m_style(style) {}
			explicit operator bool() const { return m_size || m_colour || m_style; }
			friend void Append(bytebuf& out, seri::DataPoints dp)
			{
				if (!dp) return;
				Append(out, seri::Header{ EKeywords::DataPoints }, dp.m_style, dp.m_size, dp.m_colour);
			}
			friend void Append(textbuf& out, seri::DataPoints dp)
			{
				if (!dp) return;
				Append(out, EKeywords::DataPoints, "{", dp.m_style, dp.m_size, dp.m_colour, "}");
			}
		};
		struct O2W
		{
			struct M4x4
			{
				Mat4 m;
				friend void Append(bytebuf& out, M4x4 const& v)
				{
					auto s = Append(out, seri::Header{ EKeywords::M4x4 });
					Append(out, v.m);
				}
				friend void Append(textbuf& out, M4x4 const& v)
				{
					Append(out, EKeywords::M4x4, "{", v.m, "}");
				}
			};
			struct M3x3
			{
				Mat3 rot;
				friend void Append(bytebuf& out, M3x3 const& v)
				{
					auto s = Append(out, seri::Header{ EKeywords::M3x3 });
					Append(out, v.rot);
				}
				friend void Append(textbuf& out, M3x3 const& v)
				{
					Append(out, EKeywords::M3x3, "{", v.rot, "}");
				}
			};
			struct Align_
			{
				Vec3 dir;
				AxisId axis;
				friend void Append(bytebuf& out, Align_ const& v)
				{
					auto s = Append(out, seri::Header{ EKeywords::Align });
					Append(out, v.axis);
					Append(out, v.dir);
				}
				friend void Append(textbuf& out, Align_ const& v)
				{
					Append(out, EKeywords::Align, "{", v.axis, v.dir, "}");
				}
			};
			struct LookAt_
			{
				Vec3 pos;
				friend void Append(bytebuf& out, LookAt_ const& v)
				{
					auto s = Append(out, seri::Header{ EKeywords::LookAt });
					Append(out, v.pos);
				}
				friend void Append(textbuf& out, LookAt_ const& v)
				{
					Append(out, EKeywords::LookAt, "{", v.pos, "}");
				}
			};
			struct Quat_
			{
				Vec4 q;
				friend void Append(bytebuf& out, Quat_ const& v)
				{
					auto s = Append(out, seri::Header{ EKeywords::Quat });
					Append(out, v.q);
				}
				friend void Append(textbuf& out, Quat_ const& v)
				{
					Append(out, EKeywords::Quat, "{", v.q, "}");
				}
			};
			struct Pos_
			{
				Vec3 pos;
				friend void Append(bytebuf& out, Pos_ const& v)
				{
					auto s = Append(out, seri::Header{ EKeywords::Pos });
					Append(out, v.pos);
				}
				friend void Append(textbuf& out, Pos_ const& v)
				{
					Append(out, EKeywords::Pos, "{", v.pos, "}");
				}
			};
			struct Scale_
			{
				Vec3 s;
				friend void Append(bytebuf& out, Scale_ const& v)
				{
					auto s = Append(out, seri::Header{ EKeywords::Scale });
					Append(out, v.s);
				}
				friend void Append(textbuf& out, Scale_ const& v)
				{
					Append(out, EKeywords::Scale, "{", v.s, "}");
				}
			};
			struct Euler_
			{
				float pitch_deg, yaw_deg, roll_deg;
				friend void Append(bytebuf& out, Euler_ const& v)
				{
					auto s = Append(out, seri::Header{ EKeywords::Euler });
					Append(out, v.pitch_deg, v.yaw_deg, v.roll_deg);
				}
				friend void Append(textbuf& out, Euler_ const& v)
				{
					Append(out, EKeywords::Euler, "{", v.pitch_deg, v.yaw_deg, v.roll_deg, "}");
				}
			};
			struct Rand4x4_
			{
				Vec3 centre;
				float radius;
				friend void Append(bytebuf& out, Rand4x4_ const& v)
				{
					auto s = Append(out, seri::Header{ EKeywords::Rand4x4 });
					Append(out, v.centre, v.radius);
				}
				friend void Append(textbuf& out, Rand4x4_ const& v)
				{
					Append(out, EKeywords::Rand4x4, "{", v.centre, v.radius, "}");
				}
			};
			struct RandPos_
			{
				Vec3 centre;
				float radius;
				friend void Append(bytebuf& out, RandPos_ const& v)
				{
					auto s = Append(out, seri::Header{ EKeywords::RandPos });
					Append(out, v.centre, v.radius);
				}
				friend void Append(textbuf& out, RandPos_ const& v)
				{
					Append(out, EKeywords::RandPos, "{", v.centre, v.radius, "}");
				}
			};
			struct RandOri_
			{
				friend void Append(bytebuf& out, RandOri_ const&)
				{
					auto s = Append(out, seri::Header{ EKeywords::RandOri });
				}
				friend void Append(textbuf& out, RandOri_ const&)
				{
					Append(out, EKeywords::RandOri, "{}");
				}
			};
			struct Normalise_
			{
				friend void Append(bytebuf& out, Normalise_ const&)
				{
					auto s = Append(out, seri::Header{ EKeywords::Normalise });
				}
				friend void Append(textbuf& out, Normalise_ const&)
				{
					Append(out, EKeywords::Normalise, "{}");
				}
			};
			struct Orthonormalise_
			{
				friend void Append(bytebuf& out, Orthonormalise_ const&)
				{
					auto s = Append(out, seri::Header{ EKeywords::Orthonormalise });
				}
				friend void Append(textbuf& out, Orthonormalise_ const&)
				{
					Append(out, EKeywords::Orthonormalise, "{}");
				}
			};
			struct Transpose_
			{
				friend void Append(bytebuf& out, Transpose_ const&)
				{
					auto s = Append(out, seri::Header{ EKeywords::Transpose });
				}
				friend void Append(textbuf& out, Transpose_ const&)
				{
					Append(out, EKeywords::Transpose, "{}");
				}
			};
			struct Inverse_
			{
				friend void Append(bytebuf& out, Inverse_ const&)
				{
					auto s = Append(out, seri::Header{ EKeywords::Inverse });
				}
				friend void Append(textbuf& out, Inverse_ const&)
				{
					Append(out, EKeywords::Inverse, "{}");
				}
			};
			struct NonAffine_
			{
				friend void Append(bytebuf& out, NonAffine_ const&)
				{
					auto s = Append(out, seri::Header{ EKeywords::NonAffine });
				}
				friend void Append(textbuf& out, NonAffine_ const&)
				{
					Append(out, EKeywords::NonAffine, "{}");
				}
			};

			using parts_t = std::variant<M4x4, M3x3, Align_, LookAt_, Quat_, Pos_, Scale_, Euler_, Rand4x4_, RandPos_, RandOri_, Normalise_, Orthonormalise_, Transpose_, Inverse_, NonAffine_>;
			std::vector<parts_t> m_parts;

			O2W() : m_parts() {}
			O2W(TMat4 auto o2w) : m_parts{ M4x4{ o2w } } {}
			O2W(TMat3 auto rot) : m_parts{ M3x3{ rot } } {}
			O2W& o2w(Mat4 o2w)
			{
				m_parts.push_back(M4x4{ o2w });
				return *this;
			}
			O2W& rot(Mat3 rot)
			{
				m_parts.push_back(M3x3{ rot });
				return *this;
			}
			O2W& align(Vec3 dir, AxisId axis)
			{
				m_parts.push_back(Align_{ dir, axis });
				return *this;
			}
			O2W& lookat(Vec3 pos)
			{
				m_parts.push_back(LookAt_{ pos });
				return *this;
			}
			O2W& quat(Vec4 q)
			{
				m_parts.push_back(Quat_{ q });
				return *this;
			}
			O2W& pos(Vec3 pos)
			{
				m_parts.push_back(Pos_{ pos });
				return *this;
			}
			O2W& pos(float x, float y, float z)
			{
				m_parts.push_back(Pos_{ {x, y, z} });
				return *this;
			}
			O2W& scale(Vec3 scale)
			{
				m_parts.push_back(Scale_{ scale });
				return *this;
			}
			O2W& scale(float sx, float sy, float sz)
			{
				m_parts.push_back(Scale_{ {sx, sy, sz} });
				return *this;
			}
			O2W& scale(float s)
			{
				m_parts.push_back(Scale_{ {s, s, s} });
				return *this;
			}
			O2W& euler(float pitch_deg, float yaw_deg, float roll_deg)
			{
				m_parts.push_back(Euler_{ pitch_deg, yaw_deg, roll_deg });
				return *this;
			}
			O2W& rand(Vec3 centre, float radius)
			{
				m_parts.push_back(Rand4x4_{ centre, radius });
				return *this;
			}
			O2W& rand_pos(Vec3 centre, float radius)
			{
				m_parts.push_back(RandPos_{ centre, radius });
				return *this;
			}
			O2W& rand_ori()
			{
				m_parts.push_back(RandOri_{});
				return *this;
			}
			O2W& normalise()
			{
				m_parts.push_back(Normalise_{});
				return *this;
			}
			O2W& orthonormalise()
			{
				m_parts.push_back(Orthonormalise_{});
				return *this;
			}
			O2W& transpose()
			{
				m_parts.push_back(Transpose_{});
				return *this;
			}
			O2W& inverse()
			{
				m_parts.push_back(Inverse_{});
				return *this;
			}
			O2W& non_affine()
			{
				m_parts.push_back(NonAffine_{});
				return *this;
			}

			explicit operator bool() const
			{
				return !m_parts.empty();
			}
			friend void Append(bytebuf& out, seri::O2W const& o2w)
			{
				if (!o2w) return;
				auto s = Append(out, seri::Header{ EKeywords::O2W });
				for (auto const& part : o2w.m_parts)
					std::visit([&out](auto const& p) { Append(out, p); }, part);
			}
			friend void Append(textbuf& out, seri::O2W const& o2w)
			{
				if (!o2w) return;
				Append(out, EKeywords::O2W, "{");
				for (auto const& part : o2w.m_parts)
					std::visit([&out](auto const& p) { Append(out, p); }, part);
				Append(out, "}");
			}
		};
		struct FilePath
		{
			std::string m_path;
			explicit operator bool() const
			{
				return !m_path.empty();
			}
			friend void Append(bytebuf& out, seri::FilePath f)
			{
				if (!f) return;
				auto s = Append(out, seri::Header{ EKeywords::FilePath });
				Append(out, f.m_path);
			}
			friend void Append(textbuf& out, seri::FilePath f)
			{
				if (!f) return;
				Append(out, EKeywords::FilePath, std::string("{\"") + f.m_path + "\"}");
			}
		};
		struct Texture
		{
			struct Addr_
			{
				std::string modeU, modeV;
				Addr_() : modeU("Wrap"), modeV("Wrap") {}
				Addr_(std::string u, std::string v) : modeU(u), modeV(v) {}
				Addr_(TToString auto u, TToString auto v) : Addr_(conv(u), conv(v)) {}
				friend void Append(bytebuf& out, Addr_ const& a)
				{
					auto s = Append(out, Header{ EKeywords::Addr });
					Append(out, a.modeU, a.modeV);
				}
				friend void Append(textbuf& out, Addr_ const& a)
				{
					Append(out, EKeywords::Addr, "{", a.modeU, a.modeV, "}");
				}
			};
			struct Filter_
			{
				std::string filter;
				Filter_() : filter("Point") {}
				Filter_(std::string f) : filter(f) {}
				Filter_(TToString auto f) : Filter_(conv(f)) {}
				friend void Append(bytebuf& out, Filter_ const& f)
				{
					auto s = Append(out, Header{ EKeywords::Filter });
					Append(out, f.filter);
				}
				friend void Append(textbuf& out, Filter_ const& f)
				{
					Append(out, EKeywords::Filter, "{", f.filter, "}");
				}
			};

			using parts_t = std::variant<FilePath, Addr_, Filter_, seri::Alpha, O2W>;
			std::vector<parts_t> m_parts;

			Texture& filepath(std::string_view filepath)
			{
				m_parts.push_back(FilePath{ std::string(filepath) });
				return *this;
			}
			Texture& addr(TToString auto modeU, TToString auto modeV) // Wrap|Mirror|Clamp|Border|MirrorOnce
			{
				m_parts.push_back(Addr_{ modeU, modeV });
				return *this;
			}
			Texture& addr(TToString auto mode)
			{
				return addr(mode, mode);
			}
			Texture& filter(TToString auto filter) // Point|Linear|Anisotropic
			{
				m_parts.push_back(Filter_{ filter });
				return *this;
			}
			Texture& alpha(bool on)
			{
				m_parts.push_back(seri::Alpha{ on });
				return *this;
			}
			Texture& t2s(std::invocable<O2W&> auto&& cb)
			{
				O2W o2w;
				cb(o2w);
				m_parts.push_back(std::move(o2w));
				return *this;
			}
			Texture& t2s(TMat4 auto o2w)
			{
				m_parts.push_back(O2W(o2w));
				return *this;
			}
			Texture& t2s(TMat3 auto o2w)
			{
				m_parts.push_back(O2W(o2w));
				return *this;
			}
			explicit operator bool() const
			{
				return !m_parts.empty();
			}
			friend void Append(bytebuf& out, Texture const& o2w)
			{
				if (!o2w) return;
				auto s = Append(out, Header{ EKeywords::Texture });
				for (auto const& part : o2w.m_parts)
					std::visit([&out](auto const& p) { Append(out, p); }, part);
			}
			friend void Append(textbuf& out, Texture const& o2w)
			{
				if (!o2w) return;
				Append(out, EKeywords::Texture, "{");
				for (auto const& part : o2w.m_parts)
					std::visit([&out](auto const& p) { Append(out, p); }, part);
				Append(out, "}");
			}
		};
		struct RootAnimation
		{
			struct Style_
			{
				std::string style; // NoAnimation|Once|Repeat|Continuous|PingPong
				friend void Append(bytebuf& out, Style_ const& v)
				{
					auto s = Append(out, Header{ EKeywords::Style });
					Append(out, v.style);
				}
				friend void Append(textbuf& out, Style_ const& v)
				{
					Append(out, EKeywords::Style, "{", v.style, "}");
				}
			};
			struct Period_
			{
				float seconds;
				friend void Append(bytebuf& out, Period_ const& v)
				{
					auto s = Append(out, Header{ EKeywords::Period });
					Append(out, v.seconds);
				}
				friend void Append(textbuf& out, Period_ const& v)
				{
					Append(out, EKeywords::Period, "{", v.seconds, "}");
				}
			};
			struct Velocity_
			{
				Vec3 vel;
				friend void Append(bytebuf& out, Velocity_ const& v)
				{
					auto s = Append(out, Header{ EKeywords::Velocity });
					Append(out, v.vel);
				}
				friend void Append(textbuf& out, Velocity_ const& v)
				{
					Append(out, EKeywords::Velocity, "{", v.vel, "}");
				}
			};
			struct Acceleration_
			{
				Vec3 accel;
				friend void Append(bytebuf& out, Acceleration_ const& v)
				{
					auto s = Append(out, Header{ EKeywords::Accel });
					Append(out, v.accel);
				}
				friend void Append(textbuf& out, Acceleration_ const& v)
				{
					Append(out, EKeywords::Accel, "{", v.accel, "}");
				}
			};
			struct AngVelocity_
			{
				Vec3 ang_vel;
				friend void Append(bytebuf& out, AngVelocity_ const& v)
				{
					auto s = Append(out, Header{ EKeywords::AngVelocity });
					Append(out, v.ang_vel);
				}
				friend void Append(textbuf& out, AngVelocity_ const& v)
				{
					Append(out, EKeywords::AngVelocity, "{", v.ang_vel, "}");
				}
			};
			struct AngAcceleration_
			{
				Vec3 ang_accel;
				friend void Append(bytebuf& out, AngAcceleration_ const& v)
				{
					auto s = Append(out, Header{ EKeywords::AngAccel });
					Append(out, v.ang_accel);
				}
				friend void Append(textbuf& out, AngAcceleration_ const& v)
				{
					Append(out, EKeywords::AngAccel, "{", v.ang_accel, "}");
				}
			};

			using parts_t = std::variant<Style_, Period_, Velocity_, Acceleration_, AngVelocity_, AngAcceleration_>;
			std::vector<parts_t> m_parts;

			RootAnimation& style(std::string_view style) // NoAnimation|Once|Repeat|Continuous|PingPong
			{
				m_parts.push_back(Style_{ std::string(style) });
				return *this;
			}
			RootAnimation& period(float seconds)
			{
				m_parts.push_back(Period_{ seconds });
				return *this;
			}
			RootAnimation& velocity(Vec3 vel)
			{
				m_parts.push_back(Velocity_{ vel });
				return *this;
			}
			RootAnimation& acceleration(Vec3 accel)
			{
				m_parts.push_back(Acceleration_{ accel });
				return *this;
			}
			RootAnimation& ang_velocity(Vec3 ang_vel)
			{
				m_parts.push_back(AngVelocity_{ ang_vel });
				return *this;
			}
			RootAnimation& ang_acceleration(Vec3 ang_accel)
			{
				m_parts.push_back(AngAcceleration_{ ang_accel });
				return *this;
			}

			explicit operator bool() const
			{
				return !m_parts.empty();
			}
			friend void Append(bytebuf& out, RootAnimation const& o2w)
			{
				if (!o2w) return;
				auto s = Append(out, seri::Header{ EKeywords::RootAnimation });
				for (auto const& part : o2w.m_parts)
					std::visit([&out](auto const& p) { Append(out, p); }, part);
			}
			friend void Append(textbuf& out, RootAnimation const& ra)
			{
				if (!ra) return;
				Append(out, EKeywords::RootAnimation, "{");
				for (auto const& part : ra.m_parts)
					std::visit([&out](auto const& p) { Append(out, p); }, part);
				Append(out, "}");
			}
		};
		struct Animation
		{
			struct Style_
			{
				std::string style;
				friend void Append(bytebuf& out, Style_ const& v)
				{
					auto s = Append(out, Header{ EKeywords::Style });
					Append(out, v.style);
				}
				friend void Append(textbuf& out, Style_ const& v)
				{
					Append(out, EKeywords::Style, "{", v.style, "}");
				}
			};
			struct Frame_
			{
				int frame;
				friend void Append(bytebuf& out, Frame_ const& v)
				{
					auto s = Append(out, Header{ EKeywords::Frame });
					Append(out, v.frame);
				}
				friend void Append(textbuf& out, Frame_ const& v)
				{
					Append(out, EKeywords::Frame, "{", v.frame, "}");
				}
			};
			struct FrameRange_
			{
				int start, end;
				friend void Append(bytebuf& out, FrameRange_ const& v)
				{
					auto s = Append(out, Header{ EKeywords::FrameRange });
					Append(out, v.start, v.end);
				}
				friend void Append(textbuf& out, FrameRange_ const& v)
				{
					Append(out, EKeywords::FrameRange, "{", v.start, v.end, "}");
				}
			};
			struct TimeRange_
			{
				float start, end;
				friend void Append(bytebuf& out, TimeRange_ const& v)
				{
					auto s = Append(out, Header{ EKeywords::TimeRange });
					Append(out, v.start, v.end);
				}
				friend void Append(textbuf& out, TimeRange_ const& v)
				{
					Append(out, EKeywords::TimeRange, "{", v.start, v.end, "}");
				}
			};
			struct Stretch_
			{
				float speed_multiplier;
				friend void Append(bytebuf& out, Stretch_ const& v)
				{
					auto s = Append(out, Header{ EKeywords::Stretch });
					Append(out, v.speed_multiplier);
				}
				friend void Append(textbuf& out, Stretch_ const& v)
				{
					Append(out, EKeywords::Stretch, "{", v.speed_multiplier, "}");
				}
			};
			struct NoRootTranslation_
			{
				friend void Append(bytebuf& out, NoRootTranslation_ const&)
				{
					auto s = Append(out, Header{ EKeywords::NoRootTranslation });
				}
				friend void Append(textbuf& out, NoRootTranslation_ const&)
				{
					Append(out, EKeywords::NoRootTranslation, "{}");
				}
			};
			struct NoRootRotation_
			{
				friend void Append(bytebuf& out, NoRootRotation_ const&)
				{
					auto s = Append(out, Header{ EKeywords::NoRootRotation });
				}
				friend void Append(textbuf& out, NoRootRotation_ const&)
				{
					Append(out, EKeywords::NoRootRotation, "{}");
				}
			};

			using parts_t = std::variant<Style_, Frame_, FrameRange_, TimeRange_, Stretch_, NoRootTranslation_, NoRootRotation_>;
			std::vector<parts_t> m_parts;

			Animation& style(std::string_view style)
			{
				m_parts.push_back(Style_{ std::string(style) });
				return *this;
			}
			Animation& frame(int frame)
			{
				m_parts.push_back(Frame_{ frame });
				return *this;
			}
			Animation& frame_range(int start, int end)
			{
				m_parts.push_back(FrameRange_{ start, end });
				return *this;
			}
			Animation& time_range(float start, float end)
			{
				m_parts.push_back(TimeRange_{ start, end });
				return *this;
			}
			Animation& stretch(float speed_multiplier)
			{
				m_parts.push_back(Stretch_{ speed_multiplier });
				return *this;
			}
			Animation& no_translation()
			{
				m_parts.push_back(NoRootTranslation_{});
				return *this;
			}
			Animation& no_rotation()
			{
				m_parts.push_back(NoRootRotation_{});
				return *this;
			}

			explicit operator bool() const
			{
				return !m_parts.empty();
			}
			friend void Append(bytebuf& out, Animation const& a)
			{
				if (!a) return;
				auto s = Append(out, Header{ EKeywords::Animation });
				for (auto const& part : a.m_parts)
					std::visit([&out](auto const& p) { Append(out, p); }, part);
			}
			friend void Append(textbuf& out, Animation const& a)
			{
				if (!a) return;
				Append(out, EKeywords::Animation, "{");
				for (auto const& part : a.m_parts)
					std::visit([&out](auto const& p) { Append(out, p); }, part);
				Append(out, "}");
			}
		};
		struct Facets
		{
			std::optional<int> m_facets;
			Facets() :m_facets() {}
			Facets(int f) :m_facets(f) {}
			explicit operator bool() const { return m_facets.has_value(); }
			friend void Append(bytebuf& out, seri::Facets f)
			{
				if (!f) return;
				auto s = Append(out, seri::Header{ EKeywords::Facets });
				Append(out, *f.m_facets);
			}
			friend void Append(textbuf& out, seri::Facets f)
			{
				if (!f) return;
				Append(out, EKeywords::Facets, "{", *f.m_facets, "}");
			}
		};
		struct CornerRadius
		{
			std::optional<float> m_radius;
			CornerRadius() :m_radius() {}
			CornerRadius(float r) :m_radius(r) {}
			explicit operator bool() const { return m_radius.has_value(); }
			friend void Append(bytebuf& out, seri::CornerRadius r)
			{
				if (!r) return;
				auto s = Append(out, seri::Header{ EKeywords::CornerRadius });
				Append(out, *r.m_radius);
			}
			friend void Append(textbuf& out, seri::CornerRadius r)
			{
				if (!r) return;
				Append(out, EKeywords::CornerRadius, "{", *r.m_radius, "}");
			}
		};
		struct Closed
		{
			std::optional<bool> m_closed;
			Closed() :m_closed() {}
			Closed(bool c) :m_closed(c) {}
			explicit operator bool() const { return m_closed.has_value(); }
			friend void Append(bytebuf& out, seri::Closed c)
			{
				if (!c) return;
				auto s = Append(out, seri::Header{ EKeywords::Closed });
				Append(out, *c.m_closed);
			}
			friend void Append(textbuf& out, seri::Closed c)
			{
				if (!c) return;
				Append(out, EKeywords::Closed, "{", *c.m_closed, "}");
			}
		};
		struct GenerateNormals
		{
			std::optional<float> m_angle;
			GenerateNormals() :m_angle() {}
			GenerateNormals(float angle) :m_angle(angle) {}
			explicit operator bool() const { return m_angle.has_value(); }
			friend void Append(bytebuf& out, seri::GenerateNormals g)
			{
				if (!g) return;
				auto s = Append(out, seri::Header{ EKeywords::GenerateNormals });
				Append(out, *g.m_angle);
			}
			friend void Append(textbuf& out, seri::GenerateNormals g)
			{
				if (!g) return;
				Append(out, EKeywords::GenerateNormals, "{", *g.m_angle, "}");
			}
		};
		struct Billboard
		{
			std::optional<bool> m_billboard;
			Billboard() :m_billboard() {}
			Billboard(bool b) :m_billboard(b) {}
			explicit operator bool() const { return m_billboard.has_value(); }
			friend void Append(bytebuf& out, seri::Billboard b)
			{
				if (!b) return;
				auto s = Append(out, seri::Header{ EKeywords::Billboard });
				Append(out, *b.m_billboard);
			}
			friend void Append(textbuf& out, seri::Billboard b)
			{
				if (!b) return;
				Append(out, EKeywords::Billboard, "{", *b.m_billboard, "}");
			}
		};
		struct Billboard3D
		{
			std::optional<bool> m_billboard;
			Billboard3D() :m_billboard() {}
			Billboard3D(bool b) :m_billboard(b) {}
			explicit operator bool() const { return m_billboard.has_value(); }
			friend void Append(bytebuf& out, seri::Billboard3D b)
			{
				if (!b) return;
				auto s = Append(out, seri::Header{ EKeywords::Billboard3D });
				Append(out, *b.m_billboard);
			}
			friend void Append(textbuf& out, seri::Billboard3D b)
			{
				if (!b) return;
				Append(out, EKeywords::Billboard3D, "{", *b.m_billboard, "}");
			}
		};
		struct BackColour
		{
			std::optional<uint32_t> m_colour;
			BackColour() :m_colour() {}
			BackColour(uint32_t c) :m_colour(c) {}
			explicit operator bool() const { return m_colour.has_value(); }
			friend void Append(bytebuf& out, seri::BackColour c)
			{
				if (!c) return;
				auto s = Append(out, seri::Header{ EKeywords::BackColour });
				Append(out, *c.m_colour);
			}
			friend void Append(textbuf& out, seri::BackColour c)
			{
				if (!c) return;
				Append(out, EKeywords::BackColour, "{", *c.m_colour, "}");
			}
		};
		struct Anchor
		{
			std::optional<Vec2> m_anchor;
			Anchor() :m_anchor() {}
			Anchor(Vec2 a) :m_anchor(a) {}
			Anchor(float x, float y) :m_anchor(Vec2{x, y}) {}
			explicit operator bool() const { return m_anchor.has_value(); }
			friend void Append(bytebuf& out, seri::Anchor a)
			{
				if (!a) return;
				auto s = Append(out, seri::Header{ EKeywords::Anchor });
				Append(out, *a.m_anchor);
			}
			friend void Append(textbuf& out, seri::Anchor a)
			{
				if (!a) return;
				Append(out, EKeywords::Anchor, "{", *a.m_anchor, "}");
			}
		};
		struct Padding
		{
			float m_left, m_top, m_right, m_bottom;
			bool m_set;
			Padding() :m_left(), m_top(), m_right(), m_bottom(), m_set(false) {}
			Padding(float l, float t, float r, float b) :m_left(l), m_top(t), m_right(r), m_bottom(b), m_set(true) {}
			explicit operator bool() const { return m_set; }
			friend void Append(bytebuf& out, seri::Padding p)
			{
				if (!p) return;
				auto s = Append(out, seri::Header{ EKeywords::Padding });
				Append(out, p.m_left, p.m_top, p.m_right, p.m_bottom);
			}
			friend void Append(textbuf& out, seri::Padding p)
			{
				if (!p) return;
				Append(out, EKeywords::Padding, "{", p.m_left, p.m_top, p.m_right, p.m_bottom, "}");
			}
		};
		struct VariableInt
		{
			int m_value;
			VariableInt() : m_value() {}
			VariableInt(int value) : m_value(value & 0x3FFFFFFF) {}
			friend void Append(bytebuf& out, seri::VariableInt vi)
			{
				// Variable sized int, write 6 bits at a time: xx444444 33333322 22221111 11000000
				uint8_t bits[5] = {}; int i = 5;
				for (int val = vi.m_value; val != 0 && i-- != 0; val >>= 6)
					bits[i] = static_cast<uint8_t>(0x80 | (val & 0b00111111));

				Append(out, { byte_ptr(&bits[0] + i), static_cast<size_t>(5 - i) });
			}
			friend void Append(textbuf& out, seri::VariableInt vi)
			{
				Append(out, vi.m_value);
			}
		};
		struct StringWithLength
		{
			std::string_view m_value;
			StringWithLength() : m_value() {}
			StringWithLength(std::string_view value) : m_value(value) {}
			friend void Append(bytebuf& out, seri::StringWithLength sl)
			{
				Append(out, VariableInt{ static_cast<int>(sl.m_value.size()) }); // length in bytes, not utf8 codes
				Append(out, sl.m_value);
			}
			friend void Append(textbuf& out, seri::StringWithLength sl)
			{
				Append(out, sl.m_value);
			}
		};

		// Append Helpers (implementation)
		inline void Append(bytebuf& out, std::span<std::byte const> data, int64_t ofs)
		{
			if (ofs == -1)
				out.append_range(data);
			else if (ofs + data.size() <= out.size())
				std::memcpy(out.data() + ofs, data.data(), data.size());
			else
				throw std::out_of_range("Append: offset out of range");
		}
		template <typename... Args> inline void Append(bytebuf& out, Args&&... args) { (Append(out, std::forward<Args>(args)), ...); }
		template <typename... Args> inline void Append(textbuf& out, Args&&... args) { (Append(out, std::forward<Args>(args)), ...); }
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
		LdrGroup& Group(seri::Name name = {}, seri::Colour colour = {});
		LdrInstance& Instance(seri::Name name = {}, seri::Colour colour = {});
		LdrText& Text(seri::Name name = {}, seri::Colour colour = {});
		LdrLightSource& LightSource(seri::Name name = {}, seri::Colour colour = {});
		LdrPoint& Point(seri::Name name = {}, seri::Colour colour = {});
		LdrLine& Line(seri::Name name = {}, seri::Colour colour = {});
		LdrLineBox& LineBox(seri::Name name = {}, seri::Colour colour = {});
		LdrGrid& Grid(seri::Name name = {}, seri::Colour colour = {});
		LdrCoordFrame& CoordFrame(seri::Name name = {}, seri::Colour colour = {});
		LdrCircle& Circle(seri::Name name = {}, seri::Colour colour = {});
		LdrPie& Pie(seri::Name name = {}, seri::Colour colour = {});
		LdrRect& Rect(seri::Name name = {}, seri::Colour colour = {});
		LdrPolygon& Polygon(seri::Name name = {}, seri::Colour colour = {});
		LdrTriangle& Triangle(seri::Name name = {}, seri::Colour colour = {});
		LdrQuad& Quad(seri::Name name = {}, seri::Colour colour = {});
		LdrPlane& Plane(seri::Name name = {}, seri::Colour colour = {});
		LdrRibbon& Ribbon(seri::Name name = {}, seri::Colour colour = {});
		LdrBox& Box(seri::Name name = {}, seri::Colour colour = {});
		LdrFrustum& Frustum(seri::Name name = {}, seri::Colour colour = {});
		LdrSphere& Sphere(seri::Name name = {}, seri::Colour colour = {});
		LdrCylinder& Cylinder(seri::Name name = {}, seri::Colour colour = {});
		LdrCone& Cone(seri::Name name = {}, seri::Colour colour = {});
		LdrMesh& Mesh(seri::Name name = {}, seri::Colour colour = {});
		LdrConvexHull& ConvexHull(seri::Name name = {}, seri::Colour colour = {});
		LdrModel& Model(seri::Name name = {}, seri::Colour colour = {});

		// Extension objects. Use: `builder._<LdrCustom>("name", 0xFFFFFFFF)`
		template <typename LdrCustom> requires std::is_base_of_v<LdrBase, LdrCustom>
		LdrCustom& Add(seri::Name name = {}, seri::Colour colour = {});

		// Wrap the current children in a new group.
		LdrBase& WrapAsGroup(seri::Name name = {}, seri::Colour colour = {});

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
		LdrBase& root_animation(std::invocable<seri::RootAnimation&> auto&& cb)
		{
			cb(m_root_anim);
			return *this;
		}
		seri::RootAnimation& root_animation()
		{
			return m_root_anim;
		}
		LdrBase& o2w(std::invocable<seri::O2W&> auto&& cb)
		{
			cb(m_o2w);
			return *this;
		}
		LdrBase& o2w(TMat4 auto o2w)
		{
			m_o2w.o2w(o2w);
			return *this;
		}
		seri::O2W& o2w()
		{
			return m_o2w;
		}
		LdrBase& euler(float pitch_deg, float yaw_deg, float roll_deg)
		{
			m_o2w.euler(pitch_deg, yaw_deg, roll_deg);
			return *this;
		}
		LdrBase& pos(TVec3 auto pos)
		{
			m_o2w.pos(pos);
			return *this;
		}
		LdrBase& pos(float x, float y, float z)
		{
			return pos(seri::Vec3{ x, y, z });
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
		LdrPoint& style(seri::PointStyle s) // Point style
		{
			m_style = s;
			return *this;
		}
		seri::Texture& texture() // Texture for point sprites
		{
			return m_tex;
		}
		LdrPoint& texture(std::invocable<seri::Texture&> auto&& cb)
		{
			cb(m_tex);
			return *this;
		}

		virtual void Write(textbuf& out) const override
		{
			using namespace seri;
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
			using namespace seri;
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

		LdrLine& style(seri::TToString auto sty)
		{
			m_current.m_style = seri::LineStyle{sty};
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
		LdrLine& arrow(seri::TToString auto style = "Fwd", float size = 10.0f)
		{
			m_current.m_arrow = seri::ArrowHeads(style, size);
			return *this;
		}

		LdrLine& line(seri::Vec3 a, seri::Vec3 b, seri::Colour colour = {})
		{
			// Don't overwrite style here, it could be direction or segments
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
			// Don't overwrite style here
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
			using namespace seri;
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
		virtual void Write(bytebuf& out) const override
		{
			using namespace seri;
			auto s = Append(out, seri::Header{ EKeywords::Line, m_name, m_colour });
			{
				auto WriteBlock = [](bytebuf& out, Block const& block)
				{
					Append(out, block.m_style, block.m_smooth, block.m_width, block.m_dashed, block.m_arrow, block.m_data_points, block.m_per_item_colour);
					{
						auto sd = Append(out, seri::Header{ EKeywords::Data });
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
					}
				};

				for (auto& block : m_blocks)
					WriteBlock(out, block);
				if (m_current)
					WriteBlock(out, m_current);

				LdrBase::Write(out);
			}
		}
	};
	struct LdrPlane : LdrBase
	{
		seri::Vec4 m_position;
		seri::Vec4 m_direction;
		seri::Vec2 m_wh;
		seri::Texture m_tex;
		seri::AxisId m_axis;

		LdrPlane(seri::Name name, seri::Colour colour)
			:LdrBase(name, colour)
		{}

		LdrPlane& plane(seri::Vec4 dir_and_dist)
		{
			o2w([=](seri::O2W& o2w)
			{
				o2w.pos(seri::Vec3{
					dir_and_dist.x * -dir_and_dist.w,
					dir_and_dist.y * -dir_and_dist.w,
					dir_and_dist.z * -dir_and_dist.w,
				});
				o2w.align(
					seri::Vec3{
						dir_and_dist.x,
						dir_and_dist.y,
						dir_and_dist.z,
					},
					seri::AxisId::PosZ
				);
			});
			return *this;
		}
		LdrPlane& wh(seri::Vec2 wh)
		{
			m_wh = wh;
			return *this;
		}
		LdrPlane& wh(float w, float h)
		{
			return wh({ w, h });
		}

		LdrPlane& texture(std::invocable<seri::Texture&> auto&& cb)
		{
			cb(m_tex);
			return *this;
		}
		seri::Texture& texture()
		{
			return m_tex;
		}
		LdrPlane& axis(seri::AxisId axis)
		{
			m_axis = axis;
			return *this;
		}

		virtual void Write(textbuf& out) const override
		{
			using namespace seri;
			Append(out, EKeywords::Plane, m_name, m_colour, "{");
			{
				Append(out, EKeywords::Data, "{", m_wh.x, m_wh.y, "}");
				Append(out, m_axis, m_tex);
				LdrBase::Write(out);
			}
			Append(out, "}");
		}
		virtual void Write(bytebuf& out) const override
		{
			using namespace seri;
			auto s = Append(out, seri::Header{ EKeywords::Plane, m_name, m_colour });
			{
				Append(out, seri::Header{ EKeywords::Data }, m_wh.x, m_wh.y);
				Append(out, m_axis, m_tex);
				LdrBase::Write(out);
			}
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
		LdrBox& box(float s)
		{
			return box(s, s, s);
		}

		virtual void Write(textbuf& out) const override
		{
			using namespace seri;
			auto single = m_boxes.size() == 1 && !m_boxes[0].m_pos && !m_boxes[0].m_col;
			auto per_item_colour = std::ranges::any_of(m_boxes, [](auto const& x) { return !!x.m_col; });

			Append(out, single ? EKeywords::Box : EKeywords::BoxList, m_name, m_colour, "{");
			{
				if (single)
				{
					Append(out, EKeywords::Data, "{", m_boxes[0].m_whd, "}");
				}
				else
				{
					if (per_item_colour)
						Append(out, EKeywords::PerItemColour, "{}");

					Append(out, EKeywords::Data, "{");
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
		virtual void Write(bytebuf& out) const override
		{
			using namespace seri;
			auto single = m_boxes.size() == 1 && !m_boxes[0].m_pos && !m_boxes[0].m_col;
			auto per_item_colour = std::ranges::any_of(m_boxes, [](auto const& x) { return !!x.m_col; });

			auto s = Append(out, seri::Header{ single ? EKeywords::Box : EKeywords::BoxList, m_name, m_colour });
			{
				if (single)
				{
					Append(out, seri::Header{ EKeywords::Data }, m_boxes[0].m_whd);
				}
				else
				{
					if (per_item_colour)
						Append(out, seri::Header{ EKeywords::PerItemColour });

					{
						auto sd = Append(out, seri::Header{ EKeywords::Data });
						for (auto const& box : m_boxes)
						{
							Append(out, box.m_whd, box.m_pos);
							if (per_item_colour)
								Append(out, box.m_col);
						}
					}
				}
				LdrBase::Write(out);
			}
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
			using namespace seri;
			Append(out, EKeywords::Model, m_name, m_colour, "{");
			{
				Append(out, EKeywords::FilePath, std::format("{{\"{}\"}}", m_filepath.string()));
				if (m_anim) Append(out, m_anim);
				if (m_no_materials) Append(out, EKeywords::NoMaterials, "{}" );
				LdrBase::Write(out);
			}
			Append(out, "}");
		}
		virtual void Write(bytebuf& out) const override
		{
			using namespace seri;
			auto s = Append(out, seri::Header{ EKeywords::Model, m_name, m_colour });
			{
				Append(out, seri::Header{ EKeywords::FilePath }, m_filepath.string());
				if (m_anim) Append(out, m_anim);
				if (m_no_materials) Append(out, seri::Header{ EKeywords::NoMaterials });
				LdrBase::Write(out);
			}
		}
	};
	struct LdrGroup : LdrBase
	{
		LdrGroup(seri::Name name, seri::Colour colour)
			:LdrBase(name, colour)
		{}
		virtual void Write(textbuf& out) const override
		{
			using namespace seri;
			Append(out, EKeywords::Group, m_name, m_colour, "{");
			LdrBase::Write(out);
			Append(out, "}");
		}
		virtual void Write(bytebuf& out) const override
		{
			using namespace seri;
			auto s = Append(out, seri::Header{ EKeywords::Group, m_name, m_colour });
			LdrBase::Write(out);
		}
	};
	struct LdrLineBox : LdrBase
	{
		seri::Vec3 m_dim = {};
		seri::Width m_width;
		seri::Dashed m_dashed;

		LdrLineBox(seri::Name name, seri::Colour colour)
		:LdrBase(name, colour)
		{}

		LdrLineBox& dim(seri::Vec3 d)
		{
			m_dim = d;
			return *this;
		}
		LdrLineBox& dim(float w, float h, float d)
		{
			m_dim = {w, h, d};
			return *this;
		}
		LdrLineBox& width(float w)
		{
			m_width = w;
			return *this;
		}
		LdrLineBox& dashed(seri::Vec2 d)
		{
			m_dashed = d;
			return *this;
		}

		virtual void Write(textbuf& out) const override
		{
			using namespace seri;
			Append(out, EKeywords::LineBox, m_name, m_colour, "{");
			{
				Append(out, EKeywords::Data, "{", m_dim, "}");
				Append(out, m_width, m_dashed);
				LdrBase::Write(out);
			}
			Append(out, "}");
		}
		virtual void Write(bytebuf& out) const override
		{
			using namespace seri;
			auto s = Append(out, seri::Header{ EKeywords::LineBox, m_name, m_colour });
			{
				Append(out, seri::Header{ EKeywords::Data }, m_dim);
				Append(out, m_width, m_dashed);
				LdrBase::Write(out);
			}
		}
	};
	struct LdrGrid : LdrBase
	{
		seri::Vec2 m_wh = {};
		int m_div_w = 0;
		int m_div_h = 0;
		seri::Width m_width;
		seri::Dashed m_dashed;

		LdrGrid(seri::Name name, seri::Colour colour)
		:LdrBase(name, colour)
		{}

		LdrGrid& wh(seri::Vec2 wh)
		{
			m_wh = wh;
			return *this;
		}
		LdrGrid& wh(float w, float h)
		{
			m_wh = {w, h};
			return *this;
		}
		LdrGrid& divisions(int w, int h)
		{
			m_div_w = w;
			m_div_h = h;
			return *this;
		}
		LdrGrid& width(float w)
		{
			m_width = w;
			return *this;
		}
		LdrGrid& dashed(seri::Vec2 d)
		{
			m_dashed = d;
			return *this;
		}

		virtual void Write(textbuf& out) const override
		{
			using namespace seri;
			Append(out, EKeywords::Grid, m_name, m_colour, "{");
			{
				Append(out, EKeywords::Data, "{", m_wh, "}");
				if (m_div_w != 0 || m_div_h != 0)
				Append(out, EKeywords::Divisions, "{", m_div_w, m_div_h, "}");
				Append(out, m_width, m_dashed);
				LdrBase::Write(out);
			}
			Append(out, "}");
		}
		virtual void Write(bytebuf& out) const override
		{
			using namespace seri;
			auto s = Append(out, seri::Header{ EKeywords::Grid, m_name, m_colour });
			{
				Append(out, seri::Header{ EKeywords::Data }, m_wh);
				if (m_div_w != 0 || m_div_h != 0)
				Append(out, seri::Header{ EKeywords::Divisions }, m_div_w, m_div_h);
				Append(out, m_width, m_dashed);
				LdrBase::Write(out);
			}
		}
	};
	struct LdrCoordFrame : LdrBase
	{
		seri::Scale m_scale;
		seri::Width m_width;

		LdrCoordFrame(seri::Name name, seri::Colour colour)
		:LdrBase(name, colour)
		{}

		LdrCoordFrame& scale(float s)
		{
			m_scale = s;
			return *this;
		}
		LdrCoordFrame& width(float w)
		{
			m_width = w;
			return *this;
		}

		virtual void Write(textbuf& out) const override
		{
			using namespace seri;
			Append(out, EKeywords::CoordFrame, m_name, m_colour, "{");
			{
				Append(out, m_scale, m_left_handed, m_width);
				LdrBase::Write(out);
			}
			Append(out, "}");
		}
		virtual void Write(bytebuf& out) const override
		{
			using namespace seri;
			auto s = Append(out, seri::Header{ EKeywords::CoordFrame, m_name, m_colour });
			{
				Append(out, m_scale, m_left_handed, m_width);
				LdrBase::Write(out);
			}
		}
	};
	struct LdrTriangle : LdrBase
	{
		struct Tri
		{
			seri::Vec3 a, b, c;
			seri::Colour col;
		};
		std::vector<Tri> m_tris;
		seri::PerItemColour m_per_item_colour;

		LdrTriangle(seri::Name name, seri::Colour colour)
		:LdrBase(name, colour)
		{}

		LdrTriangle& tri(seri::Vec3 a, seri::Vec3 b, seri::Vec3 c, seri::Colour colour = {})
		{
			m_tris.push_back({a, b, c, colour});
			if (colour) m_per_item_colour = true;
			return *this;
		}

		virtual void Write(textbuf& out) const override
		{
			using namespace seri;
			Append(out, EKeywords::Triangle, m_name, m_colour, "{");
			{
				Append(out, m_per_item_colour);
				Append(out, EKeywords::Data, "{");
				for (auto& t : m_tris)
				{
					Append(out, t.a, t.b, t.c);
					if (m_per_item_colour && *m_per_item_colour.m_active)
					Append(out, t.col ? *t.col.m_colour : seri::Colour::Default);
				}
				Append(out, "}");
				LdrBase::Write(out);
			}
			Append(out, "}");
		}
		virtual void Write(bytebuf& out) const override
		{
			using namespace seri;
			auto s = Append(out, seri::Header{ EKeywords::Triangle, m_name, m_colour });
			{
				Append(out, m_per_item_colour);
				{
					auto sd = Append(out, seri::Header{ EKeywords::Data });
					for (auto& t : m_tris)
					{
						Append(out, t.a, t.b, t.c);
						if (m_per_item_colour && *m_per_item_colour.m_active)
						Append(out, t.col ? *t.col.m_colour : seri::Colour::Default);
					}
				}
				LdrBase::Write(out);
			}
		}
	};
	struct LdrQuad : LdrBase
	{
		struct Qd
		{
			seri::Vec3 a, b, c, d;
			seri::Colour col;
		};
		std::vector<Qd> m_quads;
		seri::PerItemColour m_per_item_colour;
		seri::Texture m_tex;

		LdrQuad(seri::Name name, seri::Colour colour)
		:LdrBase(name, colour)
		{}

		LdrQuad& quad(seri::Vec3 a, seri::Vec3 b, seri::Vec3 c, seri::Vec3 d, seri::Colour colour = {})
		{
			m_quads.push_back({a, b, c, d, colour});
			if (colour) m_per_item_colour = true;
			return *this;
		}
		seri::Texture& texture()
		{
			return m_tex;
		}

		virtual void Write(textbuf& out) const override
		{
			using namespace seri;
			Append(out, EKeywords::Quad, m_name, m_colour, "{");
			{
				Append(out, m_per_item_colour);
				Append(out, EKeywords::Data, "{");
				for (auto& q : m_quads)
				{
					Append(out, q.a, q.b, q.c, q.d);
					if (m_per_item_colour && *m_per_item_colour.m_active)
					Append(out, q.col ? *q.col.m_colour : seri::Colour::Default);
				}
				Append(out, "}");
				Append(out, m_tex);
				LdrBase::Write(out);
			}
			Append(out, "}");
		}
		virtual void Write(bytebuf& out) const override
		{
			using namespace seri;
			auto s = Append(out, seri::Header{ EKeywords::Quad, m_name, m_colour });
			{
				Append(out, m_per_item_colour);
				{
					auto sd = Append(out, seri::Header{ EKeywords::Data });
					for (auto& q : m_quads)
					{
						Append(out, q.a, q.b, q.c, q.d);
						if (m_per_item_colour && *m_per_item_colour.m_active)
						Append(out, q.col ? *q.col.m_colour : seri::Colour::Default);
					}
				}
				Append(out, m_tex);
				LdrBase::Write(out);
			}
		}
	};
	struct LdrRibbon : LdrBase
	{
		struct Pt
		{
			seri::Vec3 pt;
			seri::Colour col;
		};
		std::vector<Pt> m_points;
		seri::Width m_width;
		seri::Smooth m_smooth;
		seri::PerItemColour m_per_item_colour;
		seri::Texture m_tex;

		LdrRibbon(seri::Name name, seri::Colour colour)
		:LdrBase(name, colour)
		{}

		LdrRibbon& pt(seri::Vec3 p, seri::Colour colour = {})
		{
			m_points.push_back({p, colour});
			if (colour) m_per_item_colour = true;
			return *this;
		}
		LdrRibbon& pt(float x, float y, float z, seri::Colour colour = {})
		{
			return pt({x, y, z}, colour);
		}
		LdrRibbon& width(float w)
		{
			m_width = w;
			return *this;
		}
		LdrRibbon& smooth(bool s = true)
		{
			m_smooth = s;
			return *this;
		}
		seri::Texture& texture()
		{
			return m_tex;
		}

		virtual void Write(textbuf& out) const override
		{
			using namespace seri;
			Append(out, EKeywords::Ribbon, m_name, m_colour, "{");
			{
				Append(out, m_width, m_smooth, m_per_item_colour);
				Append(out, EKeywords::Data, "{");
				for (auto& p : m_points)
				{
					Append(out, p.pt);
					if (m_per_item_colour && *m_per_item_colour.m_active)
					Append(out, p.col ? *p.col.m_colour : seri::Colour::Default);
				}
				Append(out, "}");
				Append(out, m_tex);
				LdrBase::Write(out);
			}
			Append(out, "}");
		}
		virtual void Write(bytebuf& out) const override
		{
			using namespace seri;
			auto s = Append(out, seri::Header{ EKeywords::Ribbon, m_name, m_colour });
			{
				Append(out, m_width, m_smooth, m_per_item_colour);
				{
					auto sd = Append(out, seri::Header{ EKeywords::Data });
					for (auto& p : m_points)
					{
						Append(out, p.pt);
						if (m_per_item_colour && *m_per_item_colour.m_active)
						Append(out, p.col ? *p.col.m_colour : seri::Colour::Default);
					}
				}
				Append(out, m_tex);
				LdrBase::Write(out);
			}
		}
	};
	struct LdrCircle : LdrBase
	{
		float m_radius = {};
		seri::Facets m_facets;

		LdrCircle(seri::Name name, seri::Colour colour)
		:LdrBase(name, colour)
		{}

		LdrCircle& radius(float r)
		{
			m_radius = r;
			return *this;
		}
		LdrCircle& facets(int f)
		{
			m_facets = f;
			return *this;
		}

		virtual void Write(textbuf& out) const override
		{
			using namespace seri;
			Append(out, EKeywords::Circle, m_name, m_colour, "{");
			{
				Append(out, EKeywords::Data, "{", m_radius, "}");
				Append(out, m_facets, m_axis_id, m_solid);
				LdrBase::Write(out);
			}
			Append(out, "}");
		}
		virtual void Write(bytebuf& out) const override
		{
			using namespace seri;
			auto s = Append(out, seri::Header{ EKeywords::Circle, m_name, m_colour });
			{
				Append(out, seri::Header{ EKeywords::Data }, m_radius);
				Append(out, m_facets, m_axis_id, m_solid);
				LdrBase::Write(out);
			}
		}
	};
	struct LdrPie : LdrBase
	{
		float m_angle0 = {};
		float m_angle1 = {};
		float m_inner_radius = {};
		float m_outer_radius = {};
		seri::Facets m_facets;
		bool m_scale_set = false;
		seri::Scale2 m_scale;

		LdrPie(seri::Name name, seri::Colour colour)
		:LdrBase(name, colour)
		{}

		LdrPie& angles(float a0, float a1)
		{
			m_angle0 = a0;
			m_angle1 = a1;
			return *this;
		}
		LdrPie& radii(float inner, float outer)
		{
			m_inner_radius = inner;
			m_outer_radius = outer;
			return *this;
		}
		LdrPie& facets(int f)
		{
			m_facets = f;
			return *this;
		}
		LdrPie& scale(float sx, float sy)
		{
			m_scale = seri::Scale2(sx, sy);
			m_scale_set = true;
			return *this;
		}

		virtual void Write(textbuf& out) const override
		{
			using namespace seri;
			Append(out, EKeywords::Pie, m_name, m_colour, "{");
			{
				Append(out, EKeywords::Data, "{", m_angle0, m_angle1, m_inner_radius, m_outer_radius, "}");
				Append(out, m_facets);
				if (m_scale_set) Append(out, m_scale);
				LdrBase::Write(out);
			}
			Append(out, "}");
		}
		virtual void Write(bytebuf& out) const override
		{
			using namespace seri;
			auto s = Append(out, seri::Header{ EKeywords::Pie, m_name, m_colour });
			{
				Append(out, seri::Header{ EKeywords::Data }, m_angle0, m_angle1, m_inner_radius, m_outer_radius);
				Append(out, m_facets);
				if (m_scale_set) Append(out, m_scale);
				LdrBase::Write(out);
			}
		}
	};
	struct LdrRect : LdrBase
	{
		seri::Vec2 m_wh = {};
		seri::CornerRadius m_corner_radius;
		seri::Facets m_facets;

		LdrRect(seri::Name name, seri::Colour colour)
		:LdrBase(name, colour)
		{}

		LdrRect& wh(seri::Vec2 wh)
		{
			m_wh = wh;
			return *this;
		}
		LdrRect& wh(float w, float h)
		{
			m_wh = {w, h};
			return *this;
		}
		LdrRect& corner_radius(float r)
		{
			m_corner_radius = r;
			return *this;
		}
		LdrRect& facets(int f)
		{
			m_facets = f;
			return *this;
		}

		virtual void Write(textbuf& out) const override
		{
			using namespace seri;
			Append(out, EKeywords::Rect, m_name, m_colour, "{");
			{
				Append(out, EKeywords::Data, "{", m_wh, "}");
				Append(out, m_corner_radius, m_facets, m_solid);
				LdrBase::Write(out);
			}
			Append(out, "}");
		}
		virtual void Write(bytebuf& out) const override
		{
			using namespace seri;
			auto s = Append(out, seri::Header{ EKeywords::Rect, m_name, m_colour });
			{
				Append(out, seri::Header{ EKeywords::Data }, m_wh);
				Append(out, m_corner_radius, m_facets, m_solid);
				LdrBase::Write(out);
			}
		}
	};
	struct LdrPolygon : LdrBase
	{
		struct Pt
		{
			seri::Vec2 pt;
			seri::Colour col;
		};
		std::vector<Pt> m_points;
		seri::PerItemColour m_per_item_colour;

		LdrPolygon(seri::Name name, seri::Colour colour)
		:LdrBase(name, colour)
		{}

		LdrPolygon& pt(seri::Vec2 p, seri::Colour colour = {})
		{
			m_points.push_back({p, colour});
			if (colour) m_per_item_colour = true;
			return *this;
		}
		LdrPolygon& pt(float x, float y, seri::Colour colour = {})
		{
			return pt({x, y}, colour);
		}

		virtual void Write(textbuf& out) const override
		{
			using namespace seri;
			Append(out, EKeywords::Polygon, m_name, m_colour, "{");
			{
				Append(out, m_per_item_colour);
				Append(out, EKeywords::Data, "{");
				for (auto& p : m_points)
				{
					Append(out, p.pt);
					if (m_per_item_colour && *m_per_item_colour.m_active)
					Append(out, p.col ? *p.col.m_colour : seri::Colour::Default);
				}
				Append(out, "}");
				LdrBase::Write(out);
			}
			Append(out, "}");
		}
		virtual void Write(bytebuf& out) const override
		{
			using namespace seri;
			auto s = Append(out, seri::Header{ EKeywords::Polygon, m_name, m_colour });
			{
				Append(out, m_per_item_colour);
				{
					auto sd = Append(out, seri::Header{ EKeywords::Data });
					for (auto& p : m_points)
					{
						Append(out, p.pt);
						if (m_per_item_colour && *m_per_item_colour.m_active)
						Append(out, p.col ? *p.col.m_colour : seri::Colour::Default);
					}
				}
				LdrBase::Write(out);
			}
		}
	};
	struct LdrSphere : LdrBase
	{
		seri::Vec3 m_radius = {};
		seri::Facets m_facets;

		LdrSphere(seri::Name name, seri::Colour colour)
		:LdrBase(name, colour)
		{}

		LdrSphere& radius(seri::Vec3 radius)
		{
			m_radius = radius;
			return *this;
		}
		LdrSphere& radius(seri::Vec4 r)
		{
			return radius({ r.x, r.y, r.z });
		}
		LdrSphere& radius(float rx, float ry, float rz)
		{
			return radius({rx, ry, rz});
		}
		LdrSphere& radius(float r)
		{
			return radius(r, r, r);
		}
		LdrSphere& facets(int f)
		{
			m_facets = f;
			return *this;
		}

		virtual void Write(textbuf& out) const override
		{
			using namespace seri;
			Append(out, EKeywords::Sphere, m_name, m_colour, "{");
			{
				if (m_radius.x == m_radius.y && m_radius.y == m_radius.z)
				Append(out, EKeywords::Data, "{", m_radius.x, "}");
				else
				Append(out, EKeywords::Data, "{", m_radius, "}");
				Append(out, m_facets);
				LdrBase::Write(out);
			}
			Append(out, "}");
		}
		virtual void Write(bytebuf& out) const override
		{
			using namespace seri;
			auto s = Append(out, seri::Header{ EKeywords::Sphere, m_name, m_colour });
			{
				if (m_radius.x == m_radius.y && m_radius.y == m_radius.z)
				Append(out, seri::Header{ EKeywords::Data }, m_radius.x);
				else
				Append(out, seri::Header{ EKeywords::Data }, m_radius);
				Append(out, m_facets);
				LdrBase::Write(out);
			}
		}
	};
	struct LdrCylinder : LdrBase
	{
		float m_height = {};
		float m_radius = {};
		float m_tip_radius = -1.0f;
		seri::Facets m_facets;
		bool m_scale_set = false;
		seri::Scale2 m_scale;

		LdrCylinder(seri::Name name, seri::Colour colour)
		:LdrBase(name, colour)
		{}

		LdrCylinder& hr(float h, float r)
		{
			m_height = h;
			m_radius = r;
			return *this;
		}
		LdrCylinder& height(float h)
		{
			m_height = h;
			return *this;
		}
		LdrCylinder& radius(float r)
		{
			m_radius = r;
			return *this;
		}
		LdrCylinder& tip_radius(float r)
		{
			m_tip_radius = r;
			return *this;
		}
		LdrCylinder& facets(int f)
		{
			m_facets = f;
			return *this;
		}
		LdrCylinder& scale(float sx, float sy)
		{
			m_scale = seri::Scale2(sx, sy);
			m_scale_set = true;
			return *this;
		}

		virtual void Write(textbuf& out) const override
		{
			using namespace seri;
			Append(out, EKeywords::Cylinder, m_name, m_colour, "{");
			{
				if (m_tip_radius >= 0)
				Append(out, EKeywords::Data, "{", m_height, m_radius, m_tip_radius, "}");
				else
				Append(out, EKeywords::Data, "{", m_height, m_radius, "}");
				Append(out, m_facets);
				if (m_scale_set) Append(out, m_scale);
				LdrBase::Write(out);
			}
			Append(out, "}");
		}
		virtual void Write(bytebuf& out) const override
		{
			using namespace seri;
			auto s = Append(out, seri::Header{ EKeywords::Cylinder, m_name, m_colour });
			{
				if (m_tip_radius >= 0)
				Append(out, seri::Header{ EKeywords::Data }, m_height, m_radius, m_tip_radius);
				else
				Append(out, seri::Header{ EKeywords::Data }, m_height, m_radius);
				Append(out, m_facets);
				if (m_scale_set) Append(out, m_scale);
				LdrBase::Write(out);
			}
		}
	};
	struct LdrCone : LdrBase
	{
		float m_angle = {};
		float m_near = {};
		float m_far = {};
		seri::Facets m_facets;
		bool m_scale_set = false;
		seri::Scale2 m_scale;

		LdrCone(seri::Name name, seri::Colour colour)
		:LdrBase(name, colour)
		{}

		LdrCone& angle(float a)
		{
			m_angle = a;
			return *this;
		}
		LdrCone& height(float h)
		{
			m_far = m_near + h;
			return *this;
		}
		LdrCone& near_dist(float n)
		{
			m_near = n;
			return *this;
		}
		LdrCone& far_dist(float f)
		{
			m_far = f;
			return *this;
		}
		LdrCone& facets(int f)
		{
			m_facets = f;
			return *this;
		}
		LdrCone& scale(float sx, float sy)
		{
			m_scale = seri::Scale2(sx, sy);
			m_scale_set = true;
			return *this;
		}

		virtual void Write(textbuf& out) const override
		{
			using namespace seri;
			Append(out, EKeywords::Cone, m_name, m_colour, "{");
			{
				Append(out, EKeywords::Data, "{", m_angle, m_near, m_far, "}");
				Append(out, m_facets);
				if (m_scale_set) Append(out, m_scale);
				LdrBase::Write(out);
			}
			Append(out, "}");
		}
		virtual void Write(bytebuf& out) const override
		{
			using namespace seri;
			auto s = Append(out, seri::Header{ EKeywords::Cone, m_name, m_colour });
			{
				Append(out, seri::Header{ EKeywords::Data }, m_angle, m_near, m_far);
				Append(out, m_facets);
				if (m_scale_set) Append(out, m_scale);
				LdrBase::Write(out);
			}
		}
	};
	struct LdrMesh : LdrBase
	{
		std::vector<seri::Vec3> m_verts;
		std::vector<seri::Vec3> m_normals;
		std::vector<uint32_t> m_colours;
		std::vector<seri::Vec2> m_tex_coords;
		std::vector<int> m_faces;
		std::vector<int> m_lines;
		std::vector<int> m_tetras;
		seri::GenerateNormals m_gen_normals;
		seri::Texture m_tex;

		LdrMesh(seri::Name name, seri::Colour colour)
		:LdrBase(name, colour)
		{}

		LdrMesh& vert(seri::Vec3 v)
		{
			m_verts.push_back(v);
			return *this;
		}
		LdrMesh& vert(float x, float y, float z)
		{
			return vert({x, y, z});
		}
		LdrMesh& normal(seri::Vec3 n)
		{
			m_normals.push_back(n);
			return *this;
		}
		LdrMesh& colour(uint32_t c)
		{
			m_colours.push_back(c);
			return *this;
		}
		LdrMesh& tex_coord(seri::Vec2 tc)
		{
			m_tex_coords.push_back(tc);
			return *this;
		}
		LdrMesh& face(int i0, int i1, int i2)
		{
			m_faces.push_back(i0);
			m_faces.push_back(i1);
			m_faces.push_back(i2);
			return *this;
		}
		LdrMesh& line(int i0, int i1)
		{
			m_lines.push_back(i0);
			m_lines.push_back(i1);
			return *this;
		}
		LdrMesh& tetra(int i0, int i1, int i2, int i3)
		{
			m_tetras.push_back(i0);
			m_tetras.push_back(i1);
			m_tetras.push_back(i2);
			m_tetras.push_back(i3);
			return *this;
		}
		LdrMesh& generate_normals(float angle = 0)
		{
			m_gen_normals = angle;
			return *this;
		}
		seri::Texture& texture()
		{
			return m_tex;
		}

		virtual void Write(textbuf& out) const override
		{
			using namespace seri;
			Append(out, EKeywords::Mesh, m_name, m_colour, "{");
			{
				if (!m_verts.empty())
				{
					Append(out, EKeywords::Verts, "{");
					for (auto& v : m_verts) Append(out, v);
					Append(out, "}");
				}
				if (!m_normals.empty())
				{
					Append(out, EKeywords::Normals, "{");
					for (auto& n : m_normals) Append(out, n);
					Append(out, "}");
				}
				if (!m_colours.empty())
				{
					Append(out, EKeywords::Colours, "{");
					for (auto c : m_colours) Append(out, c);
					Append(out, "}");
				}
				if (!m_tex_coords.empty())
				{
					Append(out, EKeywords::TexCoords, "{");
					for (auto& tc : m_tex_coords) Append(out, tc);
					Append(out, "}");
				}
				if (!m_faces.empty())
				{
					Append(out, EKeywords::Faces, "{");
					for (auto i : m_faces) Append(out, i);
					Append(out, "}");
				}
				if (!m_lines.empty())
				{
					Append(out, EKeywords::Lines, "{");
					for (auto i : m_lines) Append(out, i);
					Append(out, "}");
				}
				if (!m_tetras.empty())
				{
					Append(out, EKeywords::Tetra, "{");
					for (auto i : m_tetras) Append(out, i);
					Append(out, "}");
				}
				Append(out, m_gen_normals, m_tex);
				LdrBase::Write(out);
			}
			Append(out, "}");
		}
		virtual void Write(bytebuf& out) const override
		{
			using namespace seri;
			auto s = Append(out, seri::Header{ EKeywords::Mesh, m_name, m_colour });
			{
				if (!m_verts.empty())
				{
					auto sv = Append(out, seri::Header{ EKeywords::Verts });
					for (auto& v : m_verts) Append(out, v);
				}
				if (!m_normals.empty())
				{
					auto sn = Append(out, seri::Header{ EKeywords::Normals });
					for (auto& n : m_normals) Append(out, n);
				}
				if (!m_colours.empty())
				{
					auto sc = Append(out, seri::Header{ EKeywords::Colours });
					for (auto c : m_colours) Append(out, c);
				}
				if (!m_tex_coords.empty())
				{
					auto st = Append(out, seri::Header{ EKeywords::TexCoords });
					for (auto& tc : m_tex_coords) Append(out, tc);
				}
				if (!m_faces.empty())
				{
					auto sf = Append(out, seri::Header{ EKeywords::Faces });
					for (auto i : m_faces) Append(out, i);
				}
				if (!m_lines.empty())
				{
					auto sl = Append(out, seri::Header{ EKeywords::Lines });
					for (auto i : m_lines) Append(out, i);
				}
				if (!m_tetras.empty())
				{
					auto ste = Append(out, seri::Header{ EKeywords::Tetra });
					for (auto i : m_tetras) Append(out, i);
				}
				Append(out, m_gen_normals, m_tex);
				LdrBase::Write(out);
			}
		}
	};
	struct LdrConvexHull : LdrBase
	{
		std::vector<seri::Vec3> m_verts;
		seri::GenerateNormals m_gen_normals;

		LdrConvexHull(seri::Name name, seri::Colour colour)
		:LdrBase(name, colour)
		{}

		LdrConvexHull& vert(seri::Vec3 v)
		{
			m_verts.push_back(v);
			return *this;
		}
		LdrConvexHull& vert(float x, float y, float z)
		{
			return vert({x, y, z});
		}
		LdrConvexHull& generate_normals(float angle = 0)
		{
			m_gen_normals = angle;
			return *this;
		}

		virtual void Write(textbuf& out) const override
		{
			using namespace seri;
			Append(out, EKeywords::ConvexHull, m_name, m_colour, "{");
			{
				Append(out, EKeywords::Verts, "{");
				for (auto& v : m_verts) Append(out, v);
				Append(out, "}");
				Append(out, m_gen_normals);
				LdrBase::Write(out);
			}
			Append(out, "}");
		}
		virtual void Write(bytebuf& out) const override
		{
			using namespace seri;
			auto s = Append(out, seri::Header{ EKeywords::ConvexHull, m_name, m_colour });
			{
				{
					auto sv = Append(out, seri::Header{ EKeywords::Verts });
					for (auto& v : m_verts) Append(out, v);
				}
				Append(out, m_gen_normals);
				LdrBase::Write(out);
			}
		}
	};
	struct LdrFrustum : LdrBase
	{
		enum class EMode { WH, FA };
		EMode m_mode = EMode::WH;
		float m_width = {};
		float m_height = {};
		float m_fov_y = {};
		float m_aspect = 1.0f;
		float m_near = {};
		float m_far = {};
		seri::Facets m_facets;

		LdrFrustum(seri::Name name, seri::Colour colour)
		:LdrBase(name, colour)
		{}

		LdrFrustum& wh(float w, float h, float near_dist, float far_dist)
		{
			m_mode = EMode::WH;
			m_width = w;
			m_height = h;
			m_near = near_dist;
			m_far = far_dist;
			return *this;
		}
		LdrFrustum& fa(float fov_y, float aspect, float near_dist, float far_dist)
		{
			m_mode = EMode::FA;
			m_fov_y = fov_y;
			m_aspect = aspect;
			m_near = near_dist;
			m_far = far_dist;
			return *this;
		}
		LdrFrustum& facets(int f)
		{
			m_facets = f;
			return *this;
		}

		virtual void Write(textbuf& out) const override
		{
			using namespace seri;
			auto kw = m_mode == EMode::FA ? EKeywords::FrustumFA : EKeywords::FrustumWH;
			Append(out, kw, m_name, m_colour, "{");
			{
				if (m_mode == EMode::WH)
				Append(out, EKeywords::Data, "{", m_width, m_height, m_near, m_far, "}");
				else
				Append(out, EKeywords::Data, "{", m_fov_y, m_aspect, m_near, m_far, "}");
				Append(out, m_facets);
				LdrBase::Write(out);
			}
			Append(out, "}");
		}
		virtual void Write(bytebuf& out) const override
		{
			using namespace seri;
			auto kw = m_mode == EMode::FA ? EKeywords::FrustumFA : EKeywords::FrustumWH;
			auto s = Append(out, seri::Header{ kw, m_name, m_colour });
			{
				if (m_mode == EMode::WH)
				Append(out, seri::Header{ EKeywords::Data }, m_width, m_height, m_near, m_far);
				else
				Append(out, seri::Header{ EKeywords::Data }, m_fov_y, m_aspect, m_near, m_far);
				Append(out, m_facets);
				LdrBase::Write(out);
			}
		}
	};
	struct LdrInstance : LdrBase
	{
		std::string m_address;
		seri::Animation m_anim;

		LdrInstance(seri::Name name, seri::Colour colour)
		:LdrBase(name, colour)
		{}

		LdrInstance& address(std::string_view addr)
		{
			m_address = addr;
			return *this;
		}
		seri::Animation& anim()
		{
			return m_anim;
		}

		virtual void Write(textbuf& out) const override
		{
			using namespace seri;
			Append(out, EKeywords::Instance, m_name, m_colour, "{");
			{
				Append(out, EKeywords::Data, std::string("{\"") + m_address + "\"}");
				if (m_anim) Append(out, m_anim);
				LdrBase::Write(out);
			}
			Append(out, "}");
		}
		virtual void Write(bytebuf& out) const override
		{
			using namespace seri;
			auto s = Append(out, seri::Header{ EKeywords::Instance, m_name, m_colour });
			{
				Append(out, seri::Header{ EKeywords::Data }, m_address);
				if (m_anim) Append(out, m_anim);
				LdrBase::Write(out);
			}
		}
	};
	struct LdrText : LdrBase
	{
		std::string m_text;
		std::string m_font;
		seri::Billboard m_billboard;
		seri::BackColour m_back_colour;
		seri::Anchor m_anchor;
		seri::Padding m_padding;
		std::string m_format;

		LdrText(seri::Name name, seri::Colour colour)
		:LdrBase(name, colour)
		{}

		LdrText& text(std::string_view t)
		{
			m_text = t;
			return *this;
		}
		LdrText& font(std::string_view f)
		{
			m_font = f;
			return *this;
		}
		LdrText& billboard(bool b = true)
		{
			m_billboard = b;
			return *this;
		}
		LdrText& back_colour(uint32_t c)
		{
			m_back_colour = c;
			return *this;
		}
		LdrText& anchor(float x, float y)
		{
			m_anchor = seri::Anchor(x, y);
			return *this;
		}
		LdrText& padding(float l, float t, float r, float b)
		{
			m_padding = seri::Padding(l, t, r, b);
			return *this;
		}
		LdrText& format(std::string_view f)
		{
			m_format = f;
			return *this;
		}

		virtual void Write(textbuf& out) const override
		{
			using namespace seri;
			Append(out, EKeywords::Text, m_name, m_colour, "{");
			{
				Append(out, EKeywords::Data, std::string("{\"") + m_text + "\"}");
				if (!m_font.empty())
				Append(out, EKeywords::Font, std::string("{\"") + m_font + "\"}");
				if (!m_format.empty())
				Append(out, EKeywords::Format, std::string("{\"") + m_format + "\"}");
				Append(out, m_billboard, m_back_colour, m_anchor, m_padding);
				LdrBase::Write(out);
			}
			Append(out, "}");
		}
		virtual void Write(bytebuf& out) const override
		{
			using namespace seri;
			auto s = Append(out, seri::Header{ EKeywords::Text, m_name, m_colour });
			{
				Append(out, seri::Header{ EKeywords::Data }, m_text);
				if (!m_font.empty())
				Append(out, seri::Header{ EKeywords::Font }, m_font);
				if (!m_format.empty())
				Append(out, seri::Header{ EKeywords::Format }, m_format);
				Append(out, m_billboard, m_back_colour, m_anchor, m_padding);
				LdrBase::Write(out);
			}
		}
	};
	struct LdrLightSource : LdrBase
	{
		std::string m_style;
		std::optional<uint32_t> m_ambient;
		std::optional<uint32_t> m_diffuse;
		std::optional<uint32_t> m_specular;
		std::optional<float> m_range;
		std::optional<float> m_cone_angle;
		std::optional<bool> m_cast_shadow;

		LdrLightSource(seri::Name name, seri::Colour colour)
		:LdrBase(name, colour)
		{}

		LdrLightSource& style(std::string_view s)
		{
			m_style = s;
			return *this;
		}
		LdrLightSource& ambient(uint32_t c)
		{
			m_ambient = c;
			return *this;
		}
		LdrLightSource& diffuse(uint32_t c)
		{
			m_diffuse = c;
			return *this;
		}
		LdrLightSource& specular(uint32_t c)
		{
			m_specular = c;
			return *this;
		}
		LdrLightSource& range(float r)
		{
			m_range = r;
			return *this;
		}
		LdrLightSource& cone_angle(float angle)
		{
			m_cone_angle = angle;
			return *this;
		}
		LdrLightSource& cast_shadow(bool cs = true)
		{
			m_cast_shadow = cs;
			return *this;
		}

		virtual void Write(textbuf& out) const override
		{
			using namespace seri;
			Append(out, EKeywords::LightSource, m_name, m_colour, "{");
			{
				if (!m_style.empty())
				Append(out, EKeywords::Style, "{", m_style, "}");
				if (m_ambient)
				Append(out, EKeywords::Ambient, "{", *m_ambient, "}");
				if (m_diffuse)
				Append(out, EKeywords::Diffuse, "{", *m_diffuse, "}");
				if (m_specular)
				Append(out, EKeywords::Specular, "{", *m_specular, "}");
				if (m_range)
				Append(out, EKeywords::Range, "{", *m_range, "}");
				if (m_cone_angle)
				Append(out, EKeywords::Fov, "{", *m_cone_angle, "}");
				if (m_cast_shadow)
				Append(out, EKeywords::CastShadow, "{", *m_cast_shadow, "}");
				LdrBase::Write(out);
			}
			Append(out, "}");
		}
		virtual void Write(bytebuf& out) const override
		{
			using namespace seri;
			auto s = Append(out, seri::Header{ EKeywords::LightSource, m_name, m_colour });
			{
				if (!m_style.empty())
				Append(out, seri::Header{ EKeywords::Style }, m_style);
				if (m_ambient)
				Append(out, seri::Header{ EKeywords::Ambient }, *m_ambient);
				if (m_diffuse)
				Append(out, seri::Header{ EKeywords::Diffuse }, *m_diffuse);
				if (m_specular)
				Append(out, seri::Header{ EKeywords::Specular }, *m_specular);
				if (m_range)
				Append(out, seri::Header{ EKeywords::Range }, *m_range);
				if (m_cone_angle)
				Append(out, seri::Header{ EKeywords::Fov }, *m_cone_angle);
				if (m_cast_shadow)
				Append(out, seri::Header{ EKeywords::CastShadow }, *m_cast_shadow);
				LdrBase::Write(out);
			}
		}
	};
	struct LdrCommands :LdrBase
	{
		enum class ECmdId { AddToScene, CameraToWorld, CameraPosition, ObjectToWorld, Render };
		static constexpr std::string_view Cmds[] = { "AddToScene", "CameraToWorld", "CameraPosition", "ObjectToWorld", "Render" };
		using param_t = union param_t // Don't need type descrimination, the command implies the parameter types
		{
			seri::Mat4 mat4;
			seri::Vec4 vec4;
			seri::Vec2 vec2;
			seri::StringWithLength nstr;
			float f;
			int i;
			bool b;
		};
		using cmd_t = struct Cmd
		{
			ECmdId m_id;
			std::vector<param_t> m_params;
		};

		std::vector<cmd_t> m_cmds;

		// Add objects created by this script to scene 'scene_id'
		LdrCommands& add_to_scene(int scene_id)
		{
			m_cmds.push_back({ ECmdId::AddToScene, {{.i = scene_id}} });
			return *this;
		}

		// Apply a transform to an object with the given name
		LdrCommands& object_transform(std::string_view object_name, TMat4 auto&& o2w)
		{
			m_cmds.push_back({ ECmdId::ObjectToWorld, {{.nstr = object_name}, {.mat4 = o2w}} });
			return *this;
		}

		// Write to 'out'
		virtual void Write(bytebuf& out) const override
		{
			using namespace seri;
			auto s = Append(out, seri::Header{ EKeywords::Commands, m_name, m_colour });
			for (auto& cmd : m_cmds)
			{
				auto sd = Append(out, seri::Header{ EKeywords::Data });
				{
					Append(out, (int)cmd.m_id);
					switch (cmd.m_id)
					{
						case ECmdId::AddToScene:
						{
							Append(out, cmd.m_params[0].i);
							break;
						}
						case ECmdId::ObjectToWorld:
						{
							Append(out, cmd.m_params[0].nstr);
							Append(out, cmd.m_params[1].mat4);
							break;
						}
						default:
						{
							throw std::runtime_error("Unknown command id");
						}
					}
				}
			}
		}
		virtual void Write(textbuf& out) const override
		{
			using namespace seri;
			Append(out, EKeywords::Commands, m_name, m_colour, "{");
			for (auto& cmd : m_cmds)
			{
				Append(out, EKeywords::Data, "{");
				{
					Append(out, (int)cmd.m_id);
					switch (cmd.m_id)
					{
						case ECmdId::AddToScene:
						{
							Append(out, cmd.m_params[0].i);
							break;
						}
						case ECmdId::ObjectToWorld:
						{
							Append(out, cmd.m_params[0].nstr);
							Append(out, cmd.m_params[1].mat4);
							break;
						}
						default:
						{
							throw std::runtime_error("Unknown command id");
						}
					}
				}
				Append(out, "}");
			}
			Append(out, "}");
		}
	};
	struct LdrBinaryStream :LdrBase
	{
		// Write to 'out'
		virtual void Write(bytebuf& out) const override
		{
			using namespace seri;
			auto s = Append(out, seri::Header{ EKeywords::BinaryStream, m_name, m_colour });
		}
		virtual void Write(textbuf& out) const override
		{
			using namespace seri;
			Append(out, EKeywords::BinaryStream);
		}
	};
	struct LdrTextStream :LdrBase
	{
		// Write to 'out'
		virtual void Write(bytebuf& out) const override
		{
			using namespace seri;
			auto s = Append(out, seri::Header{ EKeywords::TextStream, m_name, m_colour });
		}
		virtual void Write(textbuf& out) const override
		{
			using namespace seri;
			Append(out, EKeywords::TextStream);
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
	inline LdrPlane& LdrBase::Plane(seri::Name name, seri::Colour colour)
	{
		m_children.push_back(ObjPtr{ new LdrPlane(name, colour) });
		return *static_cast<LdrPlane*>(m_children.back().get());
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
	inline LdrLineBox& LdrBase::LineBox(seri::Name name, seri::Colour colour)
	{
		m_children.push_back(ObjPtr{ new LdrLineBox(name, colour) });
		return *static_cast<LdrLineBox*>(m_children.back().get());
	}
	inline LdrGrid& LdrBase::Grid(seri::Name name, seri::Colour colour)
	{
		m_children.push_back(ObjPtr{ new LdrGrid(name, colour) });
		return *static_cast<LdrGrid*>(m_children.back().get());
	}
	inline LdrCoordFrame& LdrBase::CoordFrame(seri::Name name, seri::Colour colour)
	{
		m_children.push_back(ObjPtr{ new LdrCoordFrame(name, colour) });
		return *static_cast<LdrCoordFrame*>(m_children.back().get());
	}
	inline LdrTriangle& LdrBase::Triangle(seri::Name name, seri::Colour colour)
	{
		m_children.push_back(ObjPtr{ new LdrTriangle(name, colour) });
		return *static_cast<LdrTriangle*>(m_children.back().get());
	}
	inline LdrQuad& LdrBase::Quad(seri::Name name, seri::Colour colour)
	{
		m_children.push_back(ObjPtr{ new LdrQuad(name, colour) });
		return *static_cast<LdrQuad*>(m_children.back().get());
	}
	inline LdrRibbon& LdrBase::Ribbon(seri::Name name, seri::Colour colour)
	{
		m_children.push_back(ObjPtr{ new LdrRibbon(name, colour) });
		return *static_cast<LdrRibbon*>(m_children.back().get());
	}
	inline LdrCircle& LdrBase::Circle(seri::Name name, seri::Colour colour)
	{
		m_children.push_back(ObjPtr{ new LdrCircle(name, colour) });
		return *static_cast<LdrCircle*>(m_children.back().get());
	}
	inline LdrPie& LdrBase::Pie(seri::Name name, seri::Colour colour)
	{
		m_children.push_back(ObjPtr{ new LdrPie(name, colour) });
		return *static_cast<LdrPie*>(m_children.back().get());
	}
	inline LdrRect& LdrBase::Rect(seri::Name name, seri::Colour colour)
	{
		m_children.push_back(ObjPtr{ new LdrRect(name, colour) });
		return *static_cast<LdrRect*>(m_children.back().get());
	}
	inline LdrPolygon& LdrBase::Polygon(seri::Name name, seri::Colour colour)
	{
		m_children.push_back(ObjPtr{ new LdrPolygon(name, colour) });
		return *static_cast<LdrPolygon*>(m_children.back().get());
	}
	inline LdrSphere& LdrBase::Sphere(seri::Name name, seri::Colour colour)
	{
		m_children.push_back(ObjPtr{ new LdrSphere(name, colour) });
		return *static_cast<LdrSphere*>(m_children.back().get());
	}
	inline LdrCylinder& LdrBase::Cylinder(seri::Name name, seri::Colour colour)
	{
		m_children.push_back(ObjPtr{ new LdrCylinder(name, colour) });
		return *static_cast<LdrCylinder*>(m_children.back().get());
	}
	inline LdrCone& LdrBase::Cone(seri::Name name, seri::Colour colour)
	{
		m_children.push_back(ObjPtr{ new LdrCone(name, colour) });
		return *static_cast<LdrCone*>(m_children.back().get());
	}
	inline LdrMesh& LdrBase::Mesh(seri::Name name, seri::Colour colour)
	{
		m_children.push_back(ObjPtr{ new LdrMesh(name, colour) });
		return *static_cast<LdrMesh*>(m_children.back().get());
	}
	inline LdrConvexHull& LdrBase::ConvexHull(seri::Name name, seri::Colour colour)
	{
		m_children.push_back(ObjPtr{ new LdrConvexHull(name, colour) });
		return *static_cast<LdrConvexHull*>(m_children.back().get());
	}
	inline LdrFrustum& LdrBase::Frustum(seri::Name name, seri::Colour colour)
	{
		m_children.push_back(ObjPtr{ new LdrFrustum(name, colour) });
		return *static_cast<LdrFrustum*>(m_children.back().get());
	}
	inline LdrInstance& LdrBase::Instance(seri::Name name, seri::Colour colour)
	{
		m_children.push_back(ObjPtr{ new LdrInstance(name, colour) });
		return *static_cast<LdrInstance*>(m_children.back().get());
	}
	inline LdrText& LdrBase::Text(seri::Name name, seri::Colour colour)
	{
		m_children.push_back(ObjPtr{ new LdrText(name, colour) });
		return *static_cast<LdrText*>(m_children.back().get());
	}
	inline LdrLightSource& LdrBase::LightSource(seri::Name name, seri::Colour colour)
	{
		m_children.push_back(ObjPtr{ new LdrLightSource(name, colour) });
		return *static_cast<LdrLightSource*>(m_children.back().get());
	}

	// Extension objects. Use: `builder._<LdrCustom>("name", 0xFFFFFFFF)`
	template <typename LdrCustom> requires std::is_base_of_v<LdrBase, LdrCustom>
	inline LdrCustom& LdrBase::Add(seri::Name name, seri::Colour colour)
	{
		m_children.push_back(ObjPtr{ new LdrCustom(name, colour) });
		return *static_cast<LdrCustom*>(m_children.back().get());
	}

	// Wrap the current children in a new group.
	inline LdrBase& LdrBase::WrapAsGroup(seri::Name name, seri::Colour colour)
	{
		auto ptr = ObjPtr{ new LdrGroup(name, colour) };
		swap(m_children, ptr->m_children);
		m_children.emplace_back(ptr);
		return *this;
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
				.style("Star")
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
				.style("LineSegments")
				.per_item_colour()
				.width(10)
				.dashed({ 0.2f, 0.4f })
				.arrow("Fwd", 5.0f)
				.line({ -1, -1, -1 }, { +1, +1, +1 }, 0xFFFF0000)
				.line({ -1, +1, -1 }, { +1, -1, +1 }, 0xFF0000FF)
				.new_block()
				.style("LineStrip")
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
		PRUnitTestMethod(Plane)
		{
			Builder builder;
			builder.Plane("p", 0xFF00FF00)
				.plane({ 0, 1, 0, -1 })
				.wh({ 10, 20 })
				.texture([](seri::Texture& t) { t.filepath("my_texture.png").addr("Wrap").filter("Linear"); });
			auto ldr = builder.ToString(ESaveFlags::Pretty);
			PR_EXPECT(ldr ==
				"*Plane p ff00ff00 {\n"
				"	*Data {10 20}\n"
				"	*Texture {\n"
				"		*FilePath {\"my_texture.png\"}\n"
				"		*Addr {Wrap Wrap}\n"
				"		*Filter {Linear}\n"
				"	}\n"
				"	*O2W {\n"
				"		*Pos {0 1 0}\n"
				"		*Align {\n"
				"			*AxisId {3}\n"
				"			0 1 0\n"
				"		}\n"
				"	}\n"
				"}");
		}
		PRUnitTestMethod(Box)
		{
			Builder builder;
			builder.Box("b", 0xFF00FF00).box(1, 2, 3);
			auto ldr = builder.ToString();
			PR_EXPECT(ldr == "*Box b ff00ff00 {*Data {1 2 3}}");
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
		PRUnitTestMethod(LineBox)
		{
			Builder builder;
			builder.LineBox("lb", 0xFF00FF00).dim(1, 2, 3);
			auto ldr = builder.ToString();
			PR_EXPECT(ldr == "*LineBox lb ff00ff00 {*Data {1 2 3}}");
		}
		PRUnitTestMethod(Grid)
		{
			Builder builder;
			builder.Grid("g", 0xFF00FF00).wh(10, 20).divisions(5, 10);
			auto ldr = builder.ToString();
			PR_EXPECT(ldr == "*Grid g ff00ff00 {*Data {10 20} *Divisions {5 10}}");
		}
		PRUnitTestMethod(CoordFrame)
		{
			Builder builder;
			builder.CoordFrame("cf").scale(2);
			auto ldr = builder.ToString();
			PR_EXPECT(ldr == "*CoordFrame cf {*Scale {2}}");
		}
		PRUnitTestMethod(Triangle)
		{
			Builder builder;
			builder.Triangle("t", 0xFFFF0000).tri({ 0,0,0 }, { 1,0,0 }, { 0,1,0 });
			auto ldr = builder.ToString();
			PR_EXPECT(ldr == "*Triangle t ffff0000 {*Data {0 0 0 1 0 0 0 1 0}}");
		}
		PRUnitTestMethod(Quad)
		{
			Builder builder;
			builder.Quad("q", 0xFF0000FF).quad({ 0,0,0 }, { 1,0,0 }, { 1,1,0 }, { 0,1,0 });
			auto ldr = builder.ToString();
			PR_EXPECT(ldr == "*Quad q ff0000ff {*Data {0 0 0 1 0 0 1 1 0 0 1 0}}");
		}
		PRUnitTestMethod(Ribbon)
		{
			Builder builder;
			builder.Ribbon("r", 0xFF00FF00).width(2).pt(0, 0, 0).pt(1, 0, 0).pt(1, 1, 0);
			auto ldr = builder.ToString(ESaveFlags::Pretty);
			PR_EXPECT(ldr ==
			"*Ribbon r ff00ff00 {\n"
			"	*Width {2}\n"
			"	*Data {0 0 0 1 0 0 1 1 0}\n"
			"}");
		}
		PRUnitTestMethod(Circle)
		{
			Builder builder;
			builder.Circle("c", 0xFF00FF00).radius(5).facets(16);
			auto ldr = builder.ToString();
			PR_EXPECT(ldr == "*Circle c ff00ff00 {*Data {5} *Facets {16}}");
		}
		PRUnitTestMethod(Pie)
		{
			Builder builder;
			builder.Pie("p", 0xFF00FF00).angles(0, 90).radii(1, 5).facets(16);
			auto ldr = builder.ToString();
			PR_EXPECT(ldr == "*Pie p ff00ff00 {*Data {0 90 1 5} *Facets {16}}");
		}
		PRUnitTestMethod(Rect)
		{
			Builder builder;
			builder.Rect("r", 0xFF00FF00).wh(10, 20).corner_radius(2).facets(8);
			auto ldr = builder.ToString(ESaveFlags::Pretty);
			PR_EXPECT(ldr ==
			"*Rect r ff00ff00 {\n"
			"	*Data {10 20}\n"
			"	*CornerRadius {2}\n"
			"	*Facets {8}\n"
			"}");
		}
		PRUnitTestMethod(Polygon)
		{
			Builder builder;
			builder.Polygon("p", 0xFF00FF00).pt(0, 0).pt(1, 0).pt(1, 1).pt(0, 1);
			auto ldr = builder.ToString();
			PR_EXPECT(ldr == "*Polygon p ff00ff00 {*Data {0 0 1 0 1 1 0 1}}");
		}
		PRUnitTestMethod(Sphere)
		{
			Builder builder;
			builder.Sphere("s", 0xFF00FF00).radius(5).facets(16);
			auto ldr = builder.ToString();
			PR_EXPECT(ldr == "*Sphere s ff00ff00 {*Data {5} *Facets {16}}");
		}
		PRUnitTestMethod(Cylinder)
		{
			Builder builder;
			builder.Cylinder("c", 0xFF00FF00).height(10).radius(5).facets(16);
			auto ldr = builder.ToString();
			PR_EXPECT(ldr == "*Cylinder c ff00ff00 {*Data {10 5} *Facets {16}}");
		}
		PRUnitTestMethod(Cone)
		{
			Builder builder;
			builder.Cone("c", 0xFF00FF00).angle(45).near_dist(1).far_dist(10).facets(16);
			auto ldr = builder.ToString();
			PR_EXPECT(ldr == "*Cone c ff00ff00 {*Data {45 1 10} *Facets {16}}");
		}
		PRUnitTestMethod(Mesh)
		{
			Builder builder;
			builder.Mesh("m", 0xFF00FF00)
				.vert(0, 0, 0).vert(1, 0, 0).vert(0, 1, 0)
				.face(0, 1, 2);
			auto ldr = builder.ToString(ESaveFlags::Pretty);
			PR_EXPECT(ldr ==
			"*Mesh m ff00ff00 {\n"
			"	*Verts {0 0 0 1 0 0 0 1 0}\n"
			"	*Faces {0 1 2}\n"
			"}");
		}
		PRUnitTestMethod(ConvexHull)
		{
			Builder builder;
			builder.ConvexHull("ch", 0xFF00FF00)
				.vert(0, 0, 0).vert(1, 0, 0).vert(0, 1, 0).vert(0, 0, 1);
			auto ldr = builder.ToString(ESaveFlags::Pretty);
			PR_EXPECT(ldr ==
			"*ConvexHull ch ff00ff00 {\n"
			"	*Verts {0 0 0 1 0 0 0 1 0 0 0 1}\n"
			"}");
		}
		PRUnitTestMethod(Frustum)
		{
			Builder builder;
			builder.Frustum("f", 0xFF00FF00).wh(10, 20, 1, 100);
			auto ldr = builder.ToString();
			PR_EXPECT(ldr == "*FrustumWH f ff00ff00 {*Data {10 20 1 100}}");
		}
		PRUnitTestMethod(Instance)
		{
			Builder builder;
			builder.Instance("i", 0xFF00FF00).address("my_model");
			auto ldr = builder.ToString();
			PR_EXPECT(ldr == "*Instance i ff00ff00 {*Data {\"my_model\"}}");
		}
		PRUnitTestMethod(Text)
		{
			Builder builder;
			builder.Text("t", 0xFF00FF00).text("Hello");
			auto ldr = builder.ToString();
			PR_EXPECT(ldr == "*Text t ff00ff00 {*Data {\"Hello\"}}");
		}
		PRUnitTestMethod(LightSource)
		{
			Builder builder;
			builder.LightSource("ls", 0xFFFFFFFF).style("Directional").diffuse(0xFFFFFFFF).cast_shadow();
			auto ldr = builder.ToString(ESaveFlags::Pretty);
			PR_EXPECT(ldr ==
			"*LightSource ls ffffffff {\n"
			"	*Style {Directional}\n"
			"	*Diffuse {ffffffff}\n"
			"	*CastShadow {true}\n"
			"}");
		}
	};
}
#endif
