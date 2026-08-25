//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once

namespace pr::hlsl
{
	// Constant buffer registers
	enum ECBufReg
	{
		//  e.g. cbuffer MyCBuf :reg(b0)
		b0 = 0, b1, b2, b3, b4, b5, b6, b7, b8, b9, b10, b11, b12, b13, b14, b15 //...
	};

	// Shader Resource views / textures
	enum ESRVReg
	{
		// e.g. Texture2D<float4> tex[4] :reg(t0)
		t0 = 0, t1, t2, t3, t4, t5, t6, t7, t8, t9, t10, t11, t12, t13, t14, t15 //...
	};

	// Unordered access views
	enum EUAVReg
	{
		u0 = 0, u1, u2, u3, u4, u5, u6, u7, u8, u9, u10, u11, u12, u13, u14, u15, u16, u17, u18, u19 //...
	};

	// Sampler registers
	enum ESamReg
	{
		// e.g. SamplerState sam[1] :reg(s3)
		s0 = 0, s1, s2, s3, s4, s5, s6, s7, s8, s9, s10, s11, s12, s13, s14, s15 //...
	};

	// Base class to define a shader register type and value
	template <typename TShaderReg, TShaderReg rn, int sp> struct ShaderReg
	{
		inline static constexpr TShaderReg shader_register = rn;
		inline static constexpr UINT register_space = sp;
	};
}