//***********************************************************
// PR AutoExp
// Copyright (c) Rylogic Ltd 2002
//***********************************************************

#include <atomic>
#include <windows.h>
#include <fstream>
#include <stdio.h>
#include <string.h>
#include <tchar.h>
#include <malloc.h>
#include <string>

#include "dbg_helper.h"
#include "reentry_guard.h"

#include "pr/common/fmt.h"
#include "pr/common/alloca.h"
#include "pr/common/cast.h"
#include "pr/macros/link.h"
#include "pr/maths/maths.h"
#include "pr/physics/physics.h"
#include "pr/lua/lua.h"
#include "lua/include/lstate.h"

using namespace pr;
#define ADDIN_API __declspec(dllexport)

// Helper rounding function
inline float R(float x)
{
	return
		isnan(x) ? x :
		x < -maths::tinyf ? x :
		x > +maths::tinyf ? x :
		x < 0 ? -0 : +0;
}
inline double R(double x)
{
	return 
		isnan(x) ? x :
		x < -maths::tinyd ? x :
		x > +maths::tinyd ? x :
		x < 0 ? -0 : +0;
}
inline int R(int x)
{
	return x;
}
inline int64_t R(int64_t x)
{
	return x;
}

extern "C"
{
	ADDIN_API HRESULT WINAPI AddIn_v2(DWORD, DbgHelper* pHelper, int, BOOL, char *pResult, size_t max, DWORD)
	{
		ReentryGuard guard;
		v2 vec;
		if (FAILED(pHelper->Read(vec))) return E_FAIL;
		_snprintf(pResult, max,
			"{%+g %+g} Len2=%g"
			,R(vec.x)
			,R(vec.y)
			,R(Length(vec))
		);
		return S_OK;
	}
	ADDIN_API HRESULT WINAPI AddIn_v3(DWORD, DbgHelper* pHelper, int, BOOL, char *pResult, size_t max, DWORD)
	{
		ReentryGuard guard;
		v3 vec;
		if (FAILED(pHelper->Read(vec))) return E_FAIL;
		_snprintf(pResult, max,
			"{%+g %+g %+g} Len3=%g"
			,R(vec.x)
			,R(vec.y)
			,R(vec.z)
			,R(Length(vec))
		);
		return S_OK;
	}
	ADDIN_API HRESULT WINAPI AddIn_v4(DWORD, DbgHelper* pHelper, int, BOOL, char *pResult, size_t max, DWORD)
	{
		ReentryGuard guard;
		v4 vec;
		if (FAILED(pHelper->Read(vec))) return E_FAIL;
		_snprintf(pResult, max,
			"{%+g %+g %+g %+g} Len3=%g Len4=%g"
			,R(vec.x)
			,R(vec.y)
			,R(vec.z)
			,R(vec.w)
			,R(Length(vec.xyz))
			,R(Length(vec))
		);
		return S_OK;
	}
	ADDIN_API HRESULT WINAPI AddIn_v8(DWORD, DbgHelper* pHelper, int, BOOL, char *pResult, size_t max, DWORD)
	{
		ReentryGuard guard;
		v8 vec;
		if (FAILED(pHelper->Read(vec))) return E_FAIL;
		_snprintf(pResult, max,
			"{{%+g %+g %+g}  {%+g %+g %+g}}"
			,R(vec.ang.x)
			,R(vec.ang.y)
			,R(vec.ang.z)
			,R(vec.lin.x)
			,R(vec.lin.y)
			,R(vec.lin.z)
		);
		return S_OK;
	}
	ADDIN_API HRESULT WINAPI AddIn_iv2(DWORD, DbgHelper* pHelper, int, BOOL, char *pResult, size_t max, DWORD)
	{
		ReentryGuard guard;
		iv2 vec;
		if (FAILED(pHelper->Read(vec))) return E_FAIL;
		_snprintf(pResult, max,
			"{%+d %+d} Len2=%g"
			,R(vec.x)
			,R(vec.y)
			,R(Length(vec))
		);
		return S_OK;
	}
	ADDIN_API HRESULT WINAPI AddIn_iv4(DWORD, DbgHelper* pHelper, int, BOOL, char *pResult, size_t max, DWORD)
	{
		ReentryGuard guard;
		iv4 vec;
		if (FAILED(pHelper->Read(vec))) return E_FAIL;
		_snprintf(pResult, max,
			"{%+d %+d %+d %+d} Len3=%g Len4=%g"
			,R(vec.x)
			,R(vec.y)
			,R(vec.z)
			,R(vec.w)
			,R(Length(vec.w0()))
			,R(Length(vec))
		);
		return S_OK;
	}
	ADDIN_API HRESULT WINAPI AddIn_i64v4(DWORD, DbgHelper* pHelper, int, BOOL, char *pResult, size_t max, DWORD)
	{
		ReentryGuard guard;
		int64_t vec[4];
		if (FAILED(pHelper->Read(vec))) return E_FAIL;
		
		auto len3 = Len(double(vec[0]), double(vec[1]), double(vec[2]));
		auto len4 = Len(double(vec[0]), double(vec[1]), double(vec[2]), double(vec[3]));
		_snprintf(pResult, max,
			"{%+lld %+lld %+lld %+lld} Len3=%g Len4=%g"
			,R(vec[0])
			,R(vec[1])
			,R(vec[2])
			,R(vec[3])
			,R(len3)
			,R(len4)
		);
		return S_OK;
	}
	ADDIN_API HRESULT WINAPI AddIn_m2x2(DWORD, DbgHelper* pHelper, int, BOOL, char *pResult, size_t max, DWORD)
	{
		ReentryGuard guard;
		m2x2 mat;
		if (FAILED(pHelper->Read(mat))) return E_FAIL;
		
		if (mat == m2x2::Identity())
			_snprintf(pResult, max, "identity");
		else if (mat == m2x2::Zero())
			_snprintf(pResult, max, "zero");
		else
		{
			auto ortho = Cross(Normalise(mat.x), Normalise(mat.y));
			auto det = Determinant(mat);
			_snprintf(pResult, max,
				"{%+g %+g} \n"
				"{%+g %+g} \n"
				"Len={%+g %+g} \n"
				"Orth=%g Det=%g \n"
				,R(mat.x.x) , R(mat.x.y)
				,R(mat.y.x) , R(mat.y.y)
				,R(Length(mat.x))
				,R(Length(mat.y))
				,ortho
				,det
			);
		}
		return S_OK;
	}
	ADDIN_API HRESULT WINAPI AddIn_m3x4(DWORD, DbgHelper* pHelper, int, BOOL, char *pResult, size_t max, DWORD)
	{
		ReentryGuard guard;
		m3x4 mat;
		if (FAILED(pHelper->Read(mat))) return E_FAIL;

		if (mat == m3x4::Identity())
			_snprintf(pResult, max, "identity");
		else if (mat == m3x4::Zero())
			_snprintf(pResult, max, "zero");
		else
		{
			auto ortho = Length(Cross(Normalise(mat.x), Normalise(mat.y)) - Normalise(mat.z));
			auto det = Determinant(mat);
			_snprintf(pResult, max,
				"{%+g %+g %+g} \n"
				"{%+g %+g %+g} \n"
				"{%+g %+g %+g} \n"
				"Len={%+g %+g %+g} \n"
				"Ortho=%g Det=%g \n"
				,R(mat.x.x) , R(mat.x.y) , R(mat.x.z)
				,R(mat.y.x) , R(mat.y.y) , R(mat.y.z)
				,R(mat.z.x) , R(mat.z.y) , R(mat.z.z)
				,R(Length(mat.x))
				,R(Length(mat.y))
				,R(Length(mat.z))
				,ortho
				,det
			);
		}
		return S_OK;
	}
	ADDIN_API HRESULT WINAPI AddIn_m4x4(DWORD, DbgHelper* pHelper, int, BOOL, char *pResult, size_t max, DWORD)
	{
		ReentryGuard guard;
		m4x4 mat;
		if (FAILED(pHelper->Read(mat))) return E_FAIL;

		if (mat == m4x4::Identity())
			_snprintf(pResult, max, "identity");
		else if (mat == m4x4::Zero())
			_snprintf(pResult, max, "zero");
		else
		{
			auto ortho = Length(Cross(Normalise(mat.x), Normalise(mat.y)) - Normalise(mat.z));
			auto det = Determinant(mat);
			_snprintf(pResult, max,
				"{%+g %+g %+g %+g} \n"
				"{%+g %+g %+g %+g} \n"
				"{%+g %+g %+g %+g} \n"
				"{%+g %+g %+g %+g} \n"
				"Len={%+g %+g %+g %+g} \n"
				"Orth=%g Det=%g \n"
				,R(mat.x.x) , R(mat.x.y) , R(mat.x.z) , R(mat.x.w)
				,R(mat.y.x) , R(mat.y.y) , R(mat.y.z) , R(mat.y.w)
				,R(mat.z.x) , R(mat.z.y) , R(mat.z.z) , R(mat.z.w)
				,R(mat.w.x) , R(mat.w.y) , R(mat.w.z) , R(mat.w.w)
				,R(Length(mat.x))
				,R(Length(mat.y))
				,R(Length(mat.z))
				,R(Length(mat.w))
				,R(ortho)
				,R(det)
			);
		}
		return S_OK;
	}
	ADDIN_API HRESULT WINAPI AddIn_m6x8(DWORD, DbgHelper* pHelper, int, BOOL, char *pResult, size_t max, DWORD)
	{
		ReentryGuard guard;
		m6x8 mat;
		if (FAILED(pHelper->Read(mat))) return E_FAIL;

		if (mat == m6x8::Identity())
			_snprintf(pResult, max, "identity");
		else if (mat == m6x8::Zero())
			_snprintf(pResult, max, "zero");
		else
		{
			_snprintf(pResult, max,
				"{%+g %+g %+g  %+g %+g %+g} \n"
				"{%+g %+g %+g  %+g %+g %+g} \n"
				"{%+g %+g %+g  %+g %+g %+g} \n"
				"{%+g %+g %+g  %+g %+g %+g} \n"
				"{%+g %+g %+g  %+g %+g %+g} \n"
				"{%+g %+g %+g  %+g %+g %+g} \n"
				,R(mat.m00.x.x) , R(mat.m00.x.y) , R(mat.m00.x.z) , R(mat.m10.x.x) , R(mat.m10.x.y) , R(mat.m10.x.z)
				,R(mat.m00.y.x) , R(mat.m00.y.y) , R(mat.m00.y.z) , R(mat.m10.y.x) , R(mat.m10.y.y) , R(mat.m10.y.z)
				,R(mat.m00.z.x) , R(mat.m00.z.y) , R(mat.m00.z.z) , R(mat.m10.z.x) , R(mat.m10.z.y) , R(mat.m10.z.z)
				,R(mat.m01.x.x) , R(mat.m01.x.y) , R(mat.m01.x.z) , R(mat.m11.x.x) , R(mat.m11.x.y) , R(mat.m11.x.z)
				,R(mat.m01.y.x) , R(mat.m01.y.y) , R(mat.m01.y.z) , R(mat.m11.y.x) , R(mat.m11.y.y) , R(mat.m11.y.z)
				,R(mat.m01.z.x) , R(mat.m01.z.y) , R(mat.m01.z.z) , R(mat.m11.z.x) , R(mat.m11.z.y) , R(mat.m11.z.z)
				);
		}
		return S_OK;
	}
	ADDIN_API HRESULT WINAPI AddIn_Quaternion(DWORD, DbgHelper* pHelper, int, BOOL, char *pResult, size_t max, DWORD)
	{
		ReentryGuard guard;
		quat q;
		if (FAILED(pHelper->Read(q))) return E_FAIL;

		auto [axis, angle] = AxisAngle(q);
		_snprintf(pResult, max,
			"%+g %+g %+g %+g Ang=%g° Len=%g"
			,R(q.x)
			,R(q.y)
			,R(q.z)
			,R(q.w)
			,R(RadiansToDegrees(angle))
			,R(Length(q))
		);
		return S_OK;
	}
	ADDIN_API HRESULT WINAPI AddIn_MatrixF(DWORD, DbgHelper* pHelper, int, BOOL, char *pResult, size_t max, DWORD)
	{
		ReentryGuard guard;
		struct Mat :Matrix<float>
		{
			using Matrix<float>::m_buf;
			using Matrix<float>::m_data;
			using Matrix<float>::m_vecs;
			using Matrix<float>::m_cmps;
		} mat;
		if (FAILED(pHelper->Read(mat))) return E_FAIL;
		std::vector<float> data(mat.m_vecs * mat.m_cmps);
		(void)pResult,max;
		return S_OK;
	}
	ADDIN_API HRESULT WINAPI AddIn_MatrixD(DWORD, DbgHelper* pHelper, int, BOOL, char *pResult, size_t max, DWORD)
	{
		ReentryGuard guard;
		Matrix<double> mat;
		if (FAILED(pHelper->Read(mat))) return E_FAIL;
		(void)pResult,max;
		return S_OK;
	}
	ADDIN_API HRESULT WINAPI AddIn_MAXMatrix3(DWORD, DbgHelper* pHelper, int, BOOL, char *pResult, size_t max, DWORD)
	{
		ReentryGuard guard;
		float mat[3][4];
		if (FAILED(pHelper->Read(mat))) return E_FAIL;
		_snprintf(pResult, max,
			"\r\n%3.3f\t%3.3f\t%3.3f"
			"\r\n%3.3f\t%3.3f\t%3.3f"
			"\r\n%3.3f\t%3.3f\t%3.3f"
			"\r\n%3.3f\t%3.3f\t%3.3f"
			,mat[0][0], mat[1][0], mat[2][0]
			,mat[0][1], mat[1][1], mat[2][1]
			,mat[0][2], mat[1][2], mat[2][2]
			,mat[0][3], mat[1][3], mat[2][3]
		);
		return S_OK;
	}
	ADDIN_API HRESULT WINAPI AddIn_MD5(DWORD, DbgHelper* pHelper, int, BOOL, char *pResult, size_t max, DWORD)
	{
		ReentryGuard guard;
		uint8_t md5[16];
		if (FAILED(pHelper->Read(md5))) return E_FAIL;
		_snprintf(pResult, max,
			"%0.2x%0.2x%0.2x%0.2x-"
			"%0.2x%0.2x%0.2x%0.2x-"
			"%0.2x%0.2x%0.2x%0.2x-"
			"%0.2x%0.2x%0.2x%0.2x"
			,md5[15], md5[14], md5[13], md5[12]
			,md5[11], md5[10], md5[ 9], md5[ 8]
			,md5[ 7], md5[ 6], md5[ 5], md5[ 4]
			,md5[ 3], md5[ 2], md5[ 1], md5[ 0]);
		return S_OK;
	}
	ADDIN_API HRESULT WINAPI AddIn_LargeInt(DWORD, DbgHelper* pHelper, int, BOOL, char *pResult, size_t max, DWORD)
	{
		(void)pHelper, pResult, max;
		//ReentryGuard guard;
		//LargeInt large_int;
		//if (FAILED(pHelper->Read(large_int))) return E_FAIL;

		//auto str = ToString(large_int);
		//_snprintf(pResult, max, "%s", str.c_str());
		return S_OK;
	}
	ADDIN_API HRESULT WINAPI AddIn_QuaternionAsMatrix(DWORD, DbgHelper* pHelper, int, BOOL, char *pResult, size_t max, DWORD)
	{
		ReentryGuard guard;
		quat q;
		if (FAILED(pHelper->Read(q))) return E_FAIL;

		auto mat = m3x4::Rotation(q);
		_snprintf(pResult, max,
			"{%+g %+g %+g} \n"
			"{%+g %+g %+g} \n"
			"{%+g %+g %+g} \n"
			,R(mat.x.x) , R(mat.y.x) , R(mat.z.x)
			,R(mat.x.y) , R(mat.y.y) , R(mat.z.y)
			,R(mat.x.z) , R(mat.y.z) , R(mat.z.z)
		);
		return S_OK;
	}
	ADDIN_API HRESULT WINAPI AddIn_PhShape(DWORD, DbgHelper* pHelper, int, BOOL, char *pResult, size_t max, DWORD)
	{
		using namespace ph;
		ReentryGuard guard;

		Shape base;
		if (FAILED(pHelper->Read(base))) return E_FAIL;
		switch (base.m_type)
		{
		case EShape_Sphere:
			{
				ShapeSphere shape;
				if (FAILED(pHelper->Read(shape))) return E_FAIL;
				_snprintf(pResult, max, "Sph(%d): r=%f", (int)shape.m_base.m_size, shape.m_radius);
			}break;
		case EShape_Cylinder:
			{
				ShapeCylinder shape;
				if (FAILED(pHelper->Read(shape))) return E_FAIL;
				_snprintf(pResult, max, "Cyl(%d): r=%f h=%f", (int)shape.m_base.m_size, shape.m_radius, shape.m_height);
			}break;
		case EShape_Box:
			{
				ShapeBox shape;
				if (FAILED(pHelper->Read(shape))) return E_FAIL;
				_snprintf(pResult, max, "Box(%d): w=%f h=%f d=%f", (int)shape.m_base.m_size, shape.m_radius.x, shape.m_radius.y, shape.m_radius.z);
			}break;
		case EShape_Polytope:
			{
				ShapePolytope shape;
				if (FAILED(pHelper->Read(shape))) return E_FAIL;
				_snprintf(pResult, max, "Poly(%d): v=%d f=%d", (int)shape.m_base.m_size, shape.m_vert_count, shape.m_face_count);
			}break;
		case EShape_Triangle:
			{
				ShapeTriangle shape;
				if (FAILED(pHelper->Read(shape))) return E_FAIL;
				_snprintf(pResult, max, "Tri(%d): <%3.3f,%3.3f,%3.3f> <%3.3f,%3.3f,%3.3f> <%3.3f,%3.3f,%3.3f>"
					,(int)shape.m_base.m_size
					,shape.m_v.x.x ,shape.m_v.x.y ,shape.m_v.x.z
					,shape.m_v.y.x ,shape.m_v.y.y ,shape.m_v.y.z
					,shape.m_v.z.x ,shape.m_v.z.y ,shape.m_v.z.z);
			}break;
		case EShape_Terrain:
			{
				ShapeTerrain shape;
				if (FAILED(pHelper->Read(shape))) return E_FAIL;
				_snprintf(pResult, max, "Terr(%d): ", (int)shape.m_base.m_size);
			}break;
		case EShape_Array:
			{
				ShapeArray shape;
				if (FAILED(pHelper->Read(shape))) return E_FAIL;
				_snprintf(pResult, max, "Array(%d): n=%d", (int)shape.m_base.m_size, (int)shape.m_num_shapes);
			}break;
		default:
			_snprintf(pResult, max, "Unknown Shape");
			break;
		}
		return S_OK;
	}
	ADDIN_API HRESULT WINAPI AddIn_LuaState(DWORD, DbgHelper* pHelper, int, BOOL, char *pResult, size_t max, DWORD)
	{
		ReentryGuard guard;

		lua_State lua_state;
		if (FAILED(pHelper->Read(lua_state)))
			return E_FAIL;

		std::vector<TValue> stack(lua_state.stacksize);
		if (lua_state.stacksize > 0)
		{
			DWORD got, addr = *(DWORD*)&lua_state.stack;
			if (FAILED(pHelper->ReadDebuggeeMemory(pHelper, addr, (DWORD)(stack.size()*sizeof(TValue)), &stack[0], &got))) return E_FAIL;
			if (got != stack.size()*sizeof(TValue)) return E_FAIL;

			lua_state.top   = &stack[lua_state.top - lua_state.stack];
			lua_state.base  = &stack[lua_state.base - lua_state.stack];
			lua_state.stack = &stack[0];
		}

		int count = lua_gettop(&lua_state);
		std::string s = FmtS("stack: %d", count);
		for (int i = 0; i != count && i != 10; ++i)
		{
			TValue* tv = lua_state.base + i;
			switch (tv->tt)
			{
			case LUA_TNONE:          s += "\n  none";          break;
			case LUA_TNIL:           s += "\n  nil";           break;
			case LUA_TBOOLEAN:       s += "\n  bool";          break;
			case LUA_TNUMBER:        s += "\n  number";        break;
			case LUA_TSTRING:        s += "\n  string";        break;
			case LUA_TTABLE:         s += "\n  table";         break;
			case LUA_TFUNCTION:      s += "\n  function";      break;
			case LUA_TUSERDATA:      s += "\n  userdata";      break;
			case LUA_TTHREAD:        s += "\n  thread";        break;
			case LUA_TLIGHTUSERDATA: s += "\n  lightuserdata"; break;
			default:                 s += "\n  unknown";       break;
			}
		}
		_snprintf(pResult, max, "%s", s.c_str());
		return S_OK;
	}

	// inherit std::basic_ios because basic_ios has a protected empty constructor
	struct stdbase_ios :std::basic_ios<char, std::char_traits<char> > {};

	// Show the state of a std::stringstream
	ADDIN_API HRESULT WINAPI AddIn_stdstringstream(DWORD, DbgHelper* pHelper, int, BOOL, char*pResult, size_t max, DWORD)
	{
		ReentryGuard guard;

		std::stringstream strm;
		size_t iofs = byte_ptr(static_cast<std::istream::_Myios*>(&strm)) - byte_ptr(&strm);
		size_t oofs = byte_ptr(static_cast<std::ostream::_Myios*>(&strm)) - byte_ptr(&strm);

		stdbase_ios ibase_ios, obase_ios;
		if (FAILED(pHelper->Read(ibase_ios, iofs))) return E_FAIL;
		if (FAILED(pHelper->Read(obase_ios, oofs))) return E_FAIL;

		std::stringstream s;
		s << "in:< ";
		if (ibase_ios.bad() ) s << "bad ";
		if (ibase_ios.eof() ) s << "eof ";
		if (ibase_ios.fail()) s << "fail ";
		if (ibase_ios.good()) s << "good ";
		s << "> out:< ";
		if (ibase_ios.bad() ) s << "bad ";
		if (ibase_ios.eof() ) s << "eof ";
		if (ibase_ios.fail()) s << "fail ";
		if (ibase_ios.good()) s << "good ";
		s << ">";
		_snprintf(pResult, max, "%s", s.str().c_str());
		return S_OK;
	}
	ADDIN_API HRESULT WINAPI AddIn_stdifstream(DWORD, DbgHelper* pHelper, int, BOOL, char*pResult, size_t max, DWORD)
	{
		ReentryGuard guard;

		std::ifstream strm;
		size_t ofs = byte_ptr(static_cast<std::ifstream::_Myios*>(&strm)) - byte_ptr(&strm);

		stdbase_ios base_ios;
		if (FAILED(pHelper->Read(base_ios, ofs))) return E_FAIL;

		std::stringstream s;
		s << "in:< ";
		if (base_ios.bad() ) s << "bad ";
		if (base_ios.eof() ) s << "eof ";
		if (base_ios.fail()) s << "fail ";
		if (base_ios.good()) s << "good ";
		s << ">";
		_snprintf(pResult, max, "%s", s.str().c_str());
		return S_OK;
	}
	ADDIN_API HRESULT WINAPI AddIn_stdofstream(DWORD, DbgHelper* pHelper, int, BOOL, char*pResult, size_t max, DWORD)
	{
		ReentryGuard guard;

		std::ofstream strm;
		size_t ofs = byte_ptr(static_cast<std::ofstream::_Myios*>(&strm)) - byte_ptr(&strm);

		stdbase_ios base_ios;
		if (FAILED(pHelper->Read(base_ios, ofs))) return E_FAIL;

		std::stringstream s;
		s << "out:< ";
		if (base_ios.bad() ) s << "bad ";
		if (base_ios.eof() ) s << "eof ";
		if (base_ios.fail()) s << "fail ";
		if (base_ios.good()) s << "good ";
		s << ">";
		_snprintf(pResult, max, "%s", s.str().c_str());
		return S_OK;
	}
}
