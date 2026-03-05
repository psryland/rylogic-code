//**********************************
// Script
//  Copyright (c) Rylogic Ltd 2015
//**********************************
#pragma once
#include "pr/maths/maths.h"
#include "pr/script/forward.h"
#include "pr/script/script_core.h"
#include "pr/script/filter.h"
#include "pr/script/includes.h"
#include "pr/script/macros.h"
#include "pr/script/fail_policy.h"

namespace pr::script
{
	struct ByteReader
	{
		// Notes:
		//  - Trying to approximate 'Reader' with binary.
		//  - Can't use sentinel characters. Each "section" must know how big it is
		//    so:
		//     *Keyword { ... }
		//    becomes:
		//      [KeywordHash (4bytes)][SectionSize (4bytes)] [...]
		//  - Need to track nested sections
		//  - File layout looks this:
		//      [SectionHeader] (keyword+size)
		//          [binary data..]
		//          [SectionHeader]
		//              [binary data..]
		//          [binary data..]
		
		using Src = std::span<std::byte const>;
		using Section = struct { int64_t beg, end; }; // byte offsets from 'Src.begin()'
		using SectionStack = pr::vector<Section>;

	private:

		Src const m_src;                   // The original byte span
		std::byte const* m_ptr;            // The pointer into 'm_src' that advances
		SectionStack m_sections;           // Stack of section ranges
		Includes m_def_includes;           // Default Include support for referencing other files
		MacroDB m_def_macros;              // Default Macro support
		IIncludeHandler* const m_includes; // Include provider
		IMacroHandler* const m_macros;     // Macros provider
		int m_last_keyword;                //

	public:
		ByteReader(Src data, IIncludeHandler* inc = nullptr, IMacroHandler* const mac = nullptr)
			: m_src(data)
			, m_ptr(data.data())
			, m_sections()
			, m_def_includes()
			, m_def_macros()
			, m_includes(inc ? inc : &m_def_includes)
			, m_macros(mac ? mac : &m_def_macros)
			, m_last_keyword()
			, ReportError(DefaultErrorHandler)
		{
		}

		// Allow override of error handling
		std::function<bool(EResult, Loc const&, std::string_view)> ReportError;
		static bool DefaultErrorHandler(EResult result, Loc const& loc, std::string_view msg)
		{
			throw ScriptException(result, loc, msg);
		}

		// Access the underlying source
		Src const& Source() const noexcept
		{
			return m_src;
		}

		// Position in the stream
		Loc Location() const
		{
			return Loc({}, m_ptr - m_src.data());
		}

		// Access the include handler
		IIncludeHandler& Includes() const noexcept
		{
			return *m_includes;
		}

		// Access the macro handler
		IMacroHandler& Macros() const noexcept
		{
			return *m_macros;
		}

		// Return true if the end of the source has been reached
		bool IsSourceEnd() const noexcept
		{
			return m_ptr == m_src.data() + m_src.size();
		}

		// True if the next 
		bool IsKeyword() const
		{
			// If pointing at the start of a section, then true
			return IsSectionStart();
		}

		// Returns true if the next byte is the start/end of a section
		bool IsSectionStart() const
		{
			throw std::runtime_error("not implemented");
			//return m_ptr.as<char>() == '{';
		}
		bool IsSectionEnd()
		{
			throw std::runtime_error("not implemented");
			//return m_ptr.as<char>() == '}';
		}

		// Return true if the next token is not a keyword, the section end, or the end of the source
		bool IsValue()
		{
			return !IsKeyword() && !IsSectionEnd() && !IsSourceEnd();
		}

		// Move to the start/end of a section and then one past it
		bool SectionStart()
		{
			throw std::runtime_error("not implemented");
			//if (IsSectionStart()) { m_ptr.read<char>(); return true; }
			//return ReportError(EResult::TokenNotFound, Location(), "expected '{'");
		}
		bool SectionEnd()
		{
			throw std::runtime_error("not implemented");
			//auto& src = m_pp;
			//if (IsSectionEnd()) { ++src; return true; }
			//return ReportError(EResult::TokenNotFound, Location(), "expected '}'");
		}

		// Read the next keyword from the stream
		template <typename Enum> bool NextKeywordH(Enum& kw)
		{
			(void)kw;
			throw std::runtime_error("not implemented");
			//if (m_ptr) return false;
			//kw = read<Enum>();
			//return true;
		}

		// The name of the last keyword read
		std::string_view LastKeyword() const
		{
			return "not_implemented";
		}

		// Read an identifier from the source. An identifier is one of (A-Z,a-z,'_') followed by (A-Z,a-z,'_',0-9) in a contiguous block
		template <typename StrType> StrType Identifier()
		{
			StrType word;
			return Identifier(word) ? word : StrType{};
		}
		template <typename StrType> StrType IdentifierS()
		{
			StrType word;
			return IdentifierS(word) ? word : StrType{};
		}
		template <typename StrType> bool Identifier(StrType& word)
		{
			(void)word;
			throw std::runtime_error("not implemented");
			//auto& src = m_pp;
			//str::Resize(word, 0);
			//return str::ExtractIdentifier(word, src, m_delim.c_str()) || ReportError(EResult::TokenNotFound, Location(), "{identifier} expected");
		}
		template <typename StrType> bool IdentifierS(StrType& word)
		{
			return SectionStart() && Identifier(word) && SectionEnd();
		}

		// Extract identifiers from the source separated by 'sep'
		template <typename StrType> bool Identifiers(char, StrType& word)
		{
			(void)word;
			throw std::runtime_error("not implemented");
			//auto& src = m_pp;
			//str::Resize(word, 0);
			//return str::ExtractIdentifier(word, src, m_delim.c_str()) || ReportError(EResult::TokenNotFound, Location(), "identifier expected");
		}
		template <typename StrType, typename... StrTypes> bool Identifiers(char sep, StrType& word, StrTypes&&... words)
		{
			(void)sep, word, words;
			throw std::runtime_error("not implemented");
			//auto& src = m_pp;
			//str::Resize(word, 0);
			//if (!str::ExtractIdentifier(word, src, m_delim.c_str())) return ReportError(EResult::TokenNotFound, Location(), "identifier expected");
			//if (*src == sep) ++src; else return ReportError(EResult::TokenNotFound, Location(), "identifier separator expected");
			//return Identifiers(sep, std::forward<StrTypes>(words)...);
		}
		template <typename StrType, typename... StrTypes> bool IdentifiersS(char sep, StrType& word, StrTypes&&... words)
		{
			return SectionStart() && Identifiers(sep,word,std::forward<StrTypes>(words)...) && SectionEnd();
		}

		// Extract a string with length <= 255
		template <typename StrType> StrType ShortString()
		{
			throw std::runtime_error("not implemented");
			//auto len = m_ptr.read<uint8_t>();
			//auto str = &m_ptr.read<char>(len);
			//return StrType(str, str + len);
		}
		template <typename StrType> StrType LongString()
		{
			throw std::runtime_error("not implemented");
			//auto len = m_ptr.read<int>();
			//auto str = &m_ptr.read<char>(len);
			//return StrType(str, str + len);
		}

		// Extract a bool from the source.
		bool Bool()
		{
			throw std::runtime_error("not implemented");
			//return m_ptr.read<uint8_t>() != 0;;
		}
		void Bool(std::span<bool> bools)
		{
			(void)bools;
			throw std::runtime_error("not implemented");
			//auto ptr = &m_ptr.read<uint8_t>(isize(bools));
			//memcpy(bools.data(), ptr, sizeof(uint8_t) * bools.size());
		}

		// Extract an integral type from the source.
		template <typename TInt> TInt Int(int = 0)
		{
			throw std::runtime_error("not implemented");
			//return m_ptr.read<TInt>();
		}
		template <typename TInt> TInt IntS(int = 0)
		{
			throw std::runtime_error("not implemented");
			//return m_ptr.read<TInt>();
		}
		template <typename TInt> void Int(std::span<TInt> ints, int = 0)
		{
			(void)ints;
			throw std::runtime_error("not implemented");
			//auto ptr = &m_ptr.read<TInt>(isize(ints));
			//memcpy(ints.data(), ptr, sizeof(TInt) * ints.size());
		}

		// Extract a real from the source.
		template <typename TReal> TReal Real()
		{
			throw std::runtime_error("not implemented");
			//return m_ptr.read<TReal>();
		}
		template <typename TReal> void Real(std::span<TReal> reals)
		{
			(void)reals;
			throw std::runtime_error("not implemented");
			//auto* ptr = &m_ptr.read<TReal>(reals.size());
			//memcpy(reals.data(), ptr, reals.size() * sizeof(TReal));
		}

		// Extract an enum value from the source.
		template <typename TEnum> TEnum EnumValue()
		{
			throw std::runtime_error("not implemented");
			//return static_cast<TEnum>(m_ptr.read<TEnum>());
		}
		template <typename TEnum> void EnumValue(std::span<TEnum> enums)
		{
			(void)enums;
			throw std::runtime_error("not implemented");
			//auto ptr = &m_ptr.read<TEnum>(isize(enums));
			//memcpy(enums.data(), ptr, enums.size() * sizeof(TEnum));
		}

		// Extract an enum identifier from the source.
		template <typename TEnum> TEnum Enum()
		{
			TEnum enum_;
			return Enum(enum_) ? enum_ : TEnum{};
		}
		template <typename TEnum> TEnum EnumS()
		{
			TEnum enum_;
			return EnumS(enum_) ? enum_ : TEnum{};
		}
		template <typename TEnum> bool Enum(TEnum& enum_)
		{
			throw std::runtime_error("not implemented");
			//auto& src = m_pp;
			//return str::ExtractEnum(enum_, src, m_delim.c_str()) || ReportError(EResult::TokenNotFound, Location(), "enum member string name expected");
		}
		template <typename TEnum> bool EnumS(TEnum& enum_)
		{
			return SectionStart() && Enum(enum_) && SectionEnd();
		}

		// Extract a 2D real vector from the source
		v2 Vector2()
		{
			throw std::runtime_error("not implemented");
			//return m_ptr.read<v2>();
		}
		void Vector2(std::span<v2> vectors)
		{
			(void)vectors;
			throw std::runtime_error("not implemented");
			//auto* ptr = &m_ptr.read<v2>(vectors.size());
			//memcpy(vectors.data(), ptr, vectors.size() * sizeof(v2));
		}

		// Extract a 2D integer vector from the source
		iv2 Vector2i()
		{
			throw std::runtime_error("not implemented");
			//return m_ptr.read<iv2>();
		}
		void Vector2i(std::span<iv2> vectors)
		{
			(void)vectors;
			throw std::runtime_error("not implemented");
			//auto* ptr = &m_ptr.read<iv2>(vectors.size());
			//memcpy(vectors.data(), ptr, vectors.size() * sizeof(iv2));
		}

		// Extract a 3D real vector from the source
		v4 Vector3(float w)
		{
			(void)w;
			throw std::runtime_error("not implemented");
			//return v4{ m_ptr.read<v3>(), w };
		}
		void Vector3(std::span<v4> vectors, float w)
		{
			(void)vectors,w;
			throw std::runtime_error("not implemented");
			//auto* ptr = &m_ptr.read<v3>(vectors.size());
			//for (auto& v : vectors)
			//	v = v4{ v, w };
		}

		// Extract a 3D integer vector from the source
		iv4 Vector3i(int w)
		{
			(void)w;
			throw std::runtime_error("not implemented");
			//auto v = m_ptr.read<iv3>();
			//return iv4{ v, w };
		}
		void Vector3i(std::span<iv4> vectors, int w)
		{
			(void)vectors, w;
			throw std::runtime_error("not implemented");
			//auto* ptr = &m_ptr.read<iv3>(vectors.size());
			//for (auto& v : vectors)
			//	v = iv4{ v, w };
		}

		// Extract a 4D real vector from the source
		v4 Vector4()
		{
			throw std::runtime_error("not implemented");
			//return m_ptr.read<v4>();
		}
		void Vector4(std::span<v4> vectors)
		{
			(void)vectors;
			throw std::runtime_error("not implemented");
			//auto* ptr = &m_ptr.read<v4>(vectors.size());
			//memcpy(vectors.data(), ptr, vectors.size() * sizeof(v4));
		}

		// Extract a 4D integer vector from the source
		iv4 Vector4i()
		{
			throw std::runtime_error("not implemented");
			//return m_ptr.read<iv4>();
		}
		void Vector4i(std::span<iv4> vectors)
		{
			(void)vectors;
			throw std::runtime_error("not implemented");
			//auto* ptr = &m_ptr.read<iv4>(vectors.size());
			//memcpy(vectors.data(), ptr, vectors.size() * sizeof(iv4));
		}

		// Extract a quaternion from the source
		quat Quaternion()
		{
			throw std::runtime_error("not implemented");
			//return m_ptr.read<quat>();
		}
		void Quaternion(std::span<quat> quaternions)
		{
			(void)quaternions;
			throw std::runtime_error("not implemented");
			//auto* ptr = &m_ptr.read<quat>(quaternions.size());
			//memcpy(quaternions.data(), ptr, quaternions.size() * sizeof(quat));
		}

		// Extract a 3x3 matrix from the source
		m3x4 Matrix3x3()
		{
			throw std::runtime_error("not implemented");
			//auto* ptr = &m_ptr.read<v3>(3);
			//return m3x4{ ptr[0].w0(), ptr[1].w0(), ptr[2].w0() };
		}
		void Matrix3x3(std::span<m3x4> transforms)
		{
			(void)transforms;
			throw std::runtime_error("not implemented");
			//auto* ptr = &m_ptr.read<v3>(3 * transforms.size());
			//for (auto& t : transforms)
			//	t = m3x4{ ptr[0].w0(), ptr[1].w0(), ptr[2].w0() };
		}

		// Extract a 4x4 matrix from the source
		m4x4 Matrix4x4()
		{
			throw std::runtime_error("not implemented");
			//return m_ptr.read<m4x4>();
		}
		void Matrix4x4(std::span<m4x4> transforms)
		{
			(void)transforms;
			throw std::runtime_error("not implemented");
			//auto* ptr = &m_ptr.read<m4x4>(transforms.size());
			//memcpy(transforms.data(), ptr, transforms.size() * sizeof(m4x4));
		}

		// Extract a transform description accumulatively. 'o2w' should be a valid initial transform.
		m4x4& Transform(m4x4& o2w)
		{
			assert(IsFinite(o2w) && "A valid 'o2w' must be passed to this function as it pre-multiplies the transform with the one read from the script");
			auto p2w = m4x4::Identity();
			auto affine = IsAffine(o2w);

			// Parse the transform
			for (ETransformKeyword kw; NextKeywordH(kw);)
			{
				if (kw == ETransformKeyword::NonAffine)
				{
					affine = false;
					continue;
				}
				if (kw == ETransformKeyword::M4x4)
				{
					auto m = Matrix4x4();
					if (affine && m.w.w != 1)
					{
						ReportError(EResult::UnknownValue, Location(), "Specify 'NonAffine' if M4x4 is intentionally non-affine.");
						break;
					}
					p2w = m * p2w;
					continue;
				}
				if (kw == ETransformKeyword::M3x3)
				{
					auto rot = Matrix3x3();
					p2w = m4x4{rot, v4::Origin()} * p2w;
					continue;
				}
				if (kw == ETransformKeyword::Pos)
				{
					auto pos = Vector3(1.0f);
					p2w = m4x4::Translation(pos) * p2w;
					continue;
				}
				if (kw == ETransformKeyword::Align)
				{
					auto axis_id = Int<int>();
					auto direction = Vector3(0.0f);

					v4 axis = AxisId(axis_id);
					if (axis == v4::Zero())
					{
						ReportError(EResult::UnknownValue, Location(), "axis_id must one of \xc2\xb1""1, \xc2\xb1""2, \xc2\xb1""3");
						break;
					}

					p2w = m4x4::Transform(axis, direction, v4::Origin()) * p2w;
					continue;
				}
				if (kw == ETransformKeyword::Quat)
				{
					quat q = Quaternion();
					p2w = m4x4::Transform(q, v4::Origin()) * p2w;
					continue;
				}
				if (kw == ETransformKeyword::QuatPos)
				{
					auto q = Quaternion();
					auto p = Vector3(1.0f);
					p2w = m4x4::Transform(q, p) * p2w;
					continue;
				}
				if (kw == ETransformKeyword::Rand4x4)
				{
					auto centre = Vector3(1.0f);
					auto radius = Real<float>();
					p2w = m4x4::Random(g_rng(), centre, radius) * p2w;
					continue;
				}
				if (kw == ETransformKeyword::RandPos)
				{
					auto centre = Vector3(1.0f);
					auto radius = Real<float>();
					p2w = m4x4::Translation(v4::Random(g_rng(), centre, radius, 1)) * p2w;
					continue;
				}
				if (kw == ETransformKeyword::RandOri)
				{
					auto m = m4x4(m3x4::Random(g_rng()), v4::Origin());
					p2w = m * p2w;
					continue;
				}
				if (kw == ETransformKeyword::Euler)
				{
					auto angles = Vector3(0.0f);
					p2w = m4x4::Transform(DegreesToRadians(angles.x), DegreesToRadians(angles.y), DegreesToRadians(angles.z), v4::Origin()) * p2w;
					continue;
				}
				if (kw == ETransformKeyword::Scale)
				{
					auto scale = Vector3(0.0f);
					p2w = m4x4::Scale(scale.x, scale.y, scale.z, v4::Origin()) * p2w;
					continue;
				}
				if (kw == ETransformKeyword::Transpose)
				{
					p2w = Transpose(p2w);
					continue;
				}
				if (kw == ETransformKeyword::Inverse)
				{
					p2w = IsOrthonormal(p2w) ? InvertAffine(p2w) : Invert(p2w);
					continue;
				}
				if (kw == ETransformKeyword::Normalise)
				{
					p2w.x = Normalise(p2w.x);
					p2w.y = Normalise(p2w.y);
					p2w.z = Normalise(p2w.z);
					continue;
				}
				if (kw == ETransformKeyword::Orthonormalise)
				{
					p2w = Orthonorm(p2w);
					continue;
				}
				ReportError(EResult::UnknownToken, Location(), std::format("{} is not a valid Transform keyword", ToString(static_cast<ETransformKeyword>(m_last_keyword))));
				break;
			}

			// Pre-multiply the object to world transform
			o2w = p2w * o2w;
			return o2w;
		}
	};
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::script
{
	PRUnitTest(ByteReaderTests)
	{
		enum class ETestKeyword :int
		{
			Identifer = pr::hash::HashICT("Identifier"),
			String = pr::hash::HashICT("String"),
		};

		{// basic extract methods
			byte_data src;
			src.push_back(ETestKeyword::Identifer);
			src.push_back("ident");
			
#if 0
			char kw[50];
			int hashed_kw = 0;
			std::string str;
			bool bval = false, barray[4];
			int ival = 0, iarray[4];
			unsigned int uival = 0;
			float fval = 0.0f, farray[4];
			pr::v4 vec = pr::v4::Zero();
			pr::quat q = pr::QuatIdentity;
			pr::m3x4 mat3;
			pr::m4x4 mat4;

			ByteReader reader(src);
			PR_EXPECT(reader.NextKeywordS(kw)); PR_EXPECT(std::string(kw) , "Identifier"                  );
			PR_EXPECT(reader.Identifier(str)         ,true); PR_EXPECT(str             , "ident"                       );
			PR_EXPECT(reader.NextKeywordS(kw)        ,true); PR_EXPECT(std::string(kw) , "String"                      );
			PR_EXPECT(reader.String(str)             ,true); PR_EXPECT(str             , "simple string"               );
			PR_EXPECT(reader.NextKeywordH(hashed_kw) ,true); PR_EXPECT(hashed_kw       , reader.HashKeyword(L"CString"));
			PR_EXPECT(reader.CString(str)            ,true); PR_EXPECT(str             , "C:\\Path\\Filename.txt"      );
			PR_EXPECT(reader.NextKeywordS(kw)        ,true); PR_EXPECT(std::string(kw) , "Bool"                        );
			PR_EXPECT(reader.Bool(bval)              ,true); PR_EXPECT(bval            , true                          );
			PR_EXPECT(reader.NextKeywordS(kw)        ,true); PR_EXPECT(std::string(kw) , "Intg"                        );
			PR_EXPECT(reader.Int(ival, 10)           ,true); PR_EXPECT(ival            , -23                           );
			PR_EXPECT(reader.NextKeywordS(kw)        ,true); PR_EXPECT(std::string(kw) , "Intg16"                      );
			PR_EXPECT(reader.Int(uival, 16)          ,true); PR_EXPECT(uival           , 0xABCDEF00                    );
			PR_EXPECT(reader.NextKeywordS(kw)        ,true); PR_EXPECT(std::string(kw) , "Real"                        );
			PR_EXPECT(reader.Real(fval)              ,true); PR_EXPECT(fval            , -2.3e+3                       );
			PR_EXPECT(reader.NextKeywordS(kw)        ,true); PR_EXPECT(std::string(kw) , "BoolArray"                   );
			PR_EXPECT(reader.Bool(barray, 4)         ,true);
			PR_EXPECT(barray[0], true );
			PR_EXPECT(barray[1], false);
			PR_EXPECT(barray[2], true );
			PR_EXPECT(barray[3], false);
			PR_EXPECT(reader.NextKeywordS(kw)   ,true); PR_EXPECT(std::string(kw) , "IntArray");
			PR_EXPECT(reader.Int(iarray, 4, 10) ,true);
			PR_EXPECT(iarray[0], -3);
			PR_EXPECT(iarray[1], +2);
			PR_EXPECT(iarray[2], +1);
			PR_EXPECT(iarray[3], -0);
			PR_EXPECT(reader.NextKeywordS(kw) ,true); PR_EXPECT(std::string(kw) , "RealArray");
			PR_EXPECT(reader.Real(farray, 4)  ,true);
			PR_EXPECT(farray[0], 2.3f    );
			PR_EXPECT(farray[1], -1.0e-1f);
			PR_EXPECT(farray[2], +2.0f   );
			PR_EXPECT(farray[3], -0.2f   );
			PR_EXPECT(reader.NextKeywordS(kw)            ,true); PR_EXPECT(std::string(kw) , "Vector3");
			PR_EXPECT(reader.Vector3(vec,-1.0f)          ,true); PR_EXPECT(pr::FEql(vec, pr::v4(1.0f, 2.0f, 3.0f,-1.0f)), true);
			PR_EXPECT(reader.NextKeywordS(kw)            ,true); PR_EXPECT(std::string(kw) , "Vector4");
			PR_EXPECT(reader.Vector4(vec)                ,true); PR_EXPECT(pr::FEql(vec, pr::v4(4.0f, 3.0f, 2.0f, 1.0f)), true);
			PR_EXPECT(reader.NextKeywordS(kw)            ,true); PR_EXPECT(std::string(kw) , "Quaternion");
			PR_EXPECT(reader.Quaternion(q)               ,true); PR_EXPECT(pr::FEql(q, pr::quat(0.0f, -1.0f, -2.0f, -3.0f)), true);
			PR_EXPECT(reader.NextKeywordS(kw)            ,true); PR_EXPECT(std::string(kw) , "M3x3");
			PR_EXPECT(reader.Matrix3x3(mat3)             ,true); PR_EXPECT(pr::FEql(mat3, pr::m3x4::Identity()), true);
			PR_EXPECT(reader.NextKeywordS(kw)            ,true); PR_EXPECT(std::string(kw) , "M4x4");
			PR_EXPECT(reader.Matrix4x4(mat4)             ,true); PR_EXPECT(pr::FEql(mat4, pr::m4x4::Identity()), true);
			PR_EXPECT(reader.NextKeywordS(kw)            ,true); PR_EXPECT(std::string(kw) , "Data");
			PR_EXPECT(reader.Data(kw, 16)                ,true); PR_EXPECT(std::string(kw) , "ABCDEFGHIJKLMNO");
			PR_EXPECT(reader.FindKeyword(L"Section") ,true); str.resize(0);
			PR_EXPECT(reader.Section(str, false)         ,true); PR_EXPECT(str, "*SubSection { *Data \n 23 \"With a }\\\"string\\\"{ in it\" }");
			PR_EXPECT(reader.FindKeyword(L"Section") ,true); str.resize(0);
			PR_EXPECT(reader.NextKeywordS(kw)            ,true); PR_EXPECT(std::string(kw) , "Token");
			PR_EXPECT(reader.Token(str)                  ,true); PR_EXPECT(str, "123token");
			PR_EXPECT(reader.NextKeywordS(kw)            ,true); PR_EXPECT(std::string(kw) , "LastThing");
			PR_EXPECT(!reader.IsKeyword()                ,true);
			PR_EXPECT(!reader.IsSectionStart()           ,true);
			PR_EXPECT(!reader.IsSectionEnd()             ,true);
			PR_EXPECT(reader.IsSourceEnd()               ,true);
		}
		{// Dot delimited identifiers
			char const* s =
				"A.B\n"
				"a.b.c\n"
				"A.B.C.D\n"
				;
			std::string s0,s1,s2,s3;

			Reader reader(s);
			reader.Identifiers('.',s0,s1);        PR_EXPECT(s0 == "A" && s1 == "B", true);
			reader.Identifiers('.',s0,s1,s2);     PR_EXPECT(s0 == "a" && s1 == "b" && s2 == "c", true);
			reader.Identifiers('.',s0,s1,s2,s3);  PR_EXPECT(s0 == "A" && s1 == "B" && s2 == "C" && s3 == "D", true);
		}
		{// AddressAt
			wchar_t const* str0 = L""
				L"*Group { *Width {1} *Smooth *Box\n" //33
				L"{\n" //35
				L"	*other {}\n" // 46
				L"	/* *something { */\n" //66
				L"	// *something {\n" //83
				L"	\"my { string\"\n" //98
				L"	*o2w { *pos {"; // 112
			{
				StringSrc src({ str0, 0 });
				PR_EXPECT(str::Equal(Reader::AddressAt(src), ""), true);
			}
			{
				StringSrc src({ str0, 18 });
				PR_EXPECT(str::Equal(Reader::AddressAt(src), "Group.Width"), true);
			}
			{
				StringSrc src({ str0, 19 });
				PR_EXPECT(str::Equal(Reader::AddressAt(src), "Group"), true);
			}
			{
				StringSrc src({ str0, 35 });
				PR_EXPECT(str::Equal(Reader::AddressAt(src), "Group.Box"), true);
			}
			{
				StringSrc src({ str0, 88 });
				PR_EXPECT(str::Equal(Reader::AddressAt(src), ""), true); // because partway through a literal string
			}
			{
				StringSrc src(str0);
				PR_EXPECT(str::Equal(Reader::AddressAt(src), "Group.Box.o2w.pos"), true);
			}

			auto const u8str1 = u8"*One { \"💩🍌\" \"💩🍌\" }";
			char const* str1 = reinterpret_cast<char const*>(&u8str1[0]);
			{
				StringSrc src(str1); src.Limit(6);
				PR_EXPECT(str::Equal(Reader::AddressAt(src), "One"), true);
			}
			{
				StringSrc src(str1); src.Limit(9);
				PR_EXPECT(str::Equal(Reader::AddressAt(src), ""), true);
			}
			{
				StringSrc src(str1); src.Limit(11);
				PR_EXPECT(str::Equal(Reader::AddressAt(src), "One"), true);
			}
			{
				StringSrc src(str1); src.Limit(14);
				PR_EXPECT(str::Equal(Reader::AddressAt(src), ""), true);
			}
			{
				StringSrc src(str1); src.Limit(16);
				PR_EXPECT(str::Equal(Reader::AddressAt(src), "One"), true);
			}
#endif
		}
	}
}
#endif