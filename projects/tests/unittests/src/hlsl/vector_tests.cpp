//*********************************************
// HLSL VectorTests
//  Copyright (c) Rylogic Ltd 2025
//*********************************************
#include "pr/common/unittests.h"
#include "pr/math/math.h"
#include "pr/hlsl/interop.h"
#include "pr/hlsl/vector.hlsli"

namespace pr::hlsl::tests
{
	using namespace pr::math;

	PRUnitTestClass(VectorTests)
	{
		PRUnitTestMethod(AllZero)
		{
			PR_EXPECT(AllZero(float2(0, 0)));
			PR_EXPECT(!AllZero(float2(1, 0)));
			PR_EXPECT(!AllZero(float2(0, 1)));
			PR_EXPECT(!AllZero(float2(1, 1)));
			PR_EXPECT(AllZero(float3(0, 0, 0)));
			PR_EXPECT(!AllZero(float3(1, 0, 0)));
			PR_EXPECT(!AllZero(float3(0, 1, 0)));
			PR_EXPECT(!AllZero(float3(0, 0, 1)));
			PR_EXPECT(!AllZero(float3(1, 1, 1)));
			PR_EXPECT(AllZero(float4(0, 0, 0, 0)));
			PR_EXPECT(!AllZero(float4(1, 0, 0, 0)));
			PR_EXPECT(!AllZero(float4(0, 1, 0, 0)));
			PR_EXPECT(!AllZero(float4(0, 0, 1, 0)));
			PR_EXPECT(!AllZero(float4(0, 0, 0, 1)));
			PR_EXPECT(!AllZero(float4(1, 1, 1, 1)));
		}
		PRUnitTestMethod(AllZeroOrPositive)
		{
			PR_EXPECT(AllZeroOrPositive(float3(0, 0, 0)));
			PR_EXPECT(AllZeroOrPositive(float3(1, 0, 0)));
			PR_EXPECT(AllZeroOrPositive(float3(1, 2, 3)));
			PR_EXPECT(!AllZeroOrPositive(float3(-1, 0, 0)));
			PR_EXPECT(!AllZeroOrPositive(float3(0, -1, 0)));
		}
		PRUnitTestMethod(Triple)
		{
			// Triple product of orthogonal unit vectors = ±1
			auto x = float4(1, 0, 0, 0);
			auto y = float4(0, 1, 0, 0);
			auto z = float4(0, 0, 1, 0);
			PR_EXPECT(FEql(Triple(x, y, z), 1.0f));
			PR_EXPECT(FEql(Triple(y, x, z), -1.0f));
		}
		PRUnitTestMethod(CrossProductMatrix)
		{
			// CPM(r) * v should equal cross(r, v)
			auto r = float3(1, 2, 3);
			auto v = float3(4, -1, 2);
			auto cpm = CrossProductMatrix(r);
			auto result = mul(v, cpm);
			auto expected = cross(r, v);
			PR_EXPECT(FEql(result, expected));
		}
		PRUnitTestMethod(RotateCW_CCW)
		{
			auto v = float2(1, 0);
			auto cw = RotateCW(v);
			auto ccw = RotateCCW(v);

			// CW rotation of (1,0) = (0,-1)
			PR_EXPECT(FEql(cw.x, 0.0f));
			PR_EXPECT(FEql(cw.y, -1.0f));

			// CCW rotation of (1,0) = (0,1)
			PR_EXPECT(FEql(ccw.x, 0.0f));
			PR_EXPECT(FEql(ccw.y, 1.0f));
		}
		PRUnitTestMethod(Cross2D)
		{
			// Cross product of (1,0) x (0,1) = 1
			PR_EXPECT(FEql(Cross2D(float2(1, 0), float2(0, 1)), 1.0f));
			PR_EXPECT(FEql(Cross2D(float2(0, 1), float2(1, 0)), -1.0f));
		}
		PRUnitTestMethod(Perpendicular)
		{
			// Result should be perpendicular to input
			auto v = float3(1, 2, 3);
			auto p = Perpendicular(v);
			PR_EXPECT(FEqlAbsolute(dot(v, p), 0.0f, 1e-4f));
			PR_EXPECT(length(p) > 0.0f);

			// Test with axis-aligned vectors
			auto px = Perpendicular(float3(1, 0, 0));
			PR_EXPECT(FEqlAbsolute(dot(float3(1, 0, 0), px), 0.0f, 1e-4f));
		}
		PRUnitTestMethod(NormaliseOrZero)
		{
			// Normal vector
			auto v = NormaliseOrZero(float3(3, 0, 0));
			PR_EXPECT(FEql(v, float3(1, 0, 0)));

			// Zero vector should return zero
			auto z = NormaliseOrZero(float3(0, 0, 0));
			PR_EXPECT(FEql(z, float3(0, 0, 0)));
		}
		PRUnitTestMethod(Orthonormalise3x3)
		{
			// Start with a slightly skewed matrix
			auto m = float3x3(
				float3(1.01f, 0.02f, 0),
				float3(0, 0.01f, 1.01f),
				float3(0, -0.98f, 0.02f));

			auto o = Orthonormalise(m);

			// Compare HLSL vs CPU
			auto cpu_o = pr::math::Orthonorm(m);
			PR_EXPECT(FEql(o, cpu_o));

			// Check unit length rows
			PR_EXPECT(FEql(length(o[0]), 1.0f));
			PR_EXPECT(FEql(length(o[1]), 1.0f));
			PR_EXPECT(FEql(length(o[2]), 1.0f));

			// Check orthogonality
			PR_EXPECT(FEql(dot(o[0], o[1]), 0.0f));
			PR_EXPECT(FEql(dot(o[0], o[2]), 0.0f));
			PR_EXPECT(FEql(dot(o[1], o[2]), 0.0f));
		}
		PRUnitTestMethod(InvertOrthonormal)
		{
			// Build an orthonormal matrix with rotation and translation
			float a = 0.7f;
			auto m = float4x4(
				float4(cos(a), sin(a), 0, 0),
				float4(-sin(a), cos(a), 0, 0),
				float4(0, 0, 1, 0),
				float4(3, 5, 7, 1));

			auto inv = InvertOrthonormal(m);

			// Compare HLSL vs CPU
			auto cpu_inv = pr::math::InvertOrthonormal(m);
			PR_EXPECT(FEql(inv, cpu_inv));

			auto product = mul(m, inv);

			// M * M^-1 should be identity
			auto I = float4x4(
				float4(1, 0, 0, 0),
				float4(0, 1, 0, 0),
				float4(0, 0, 1, 0),
				float4(0, 0, 0, 1));
			PR_EXPECT(FEql(product, I));
		}
		PRUnitTestMethod(InvertAffine)
		{
			// Build an affine matrix with rotation, scale, and translation
			float a = 1.2f;
			auto m = float4x4(
				float4(2 * cos(a), 2 * sin(a), 0, 0),
				float4(-3 * sin(a), 3 * cos(a), 0, 0),
				float4(0, 0, 4, 0),
				float4(1, 2, 3, 1));

			auto inv = InvertAffine(m);

			// Compare HLSL vs CPU
			auto cpu_inv = pr::math::InvertAffine(m);
			PR_EXPECT(FEqlAbsolute(inv, cpu_inv, 1e-4f));

			auto product = mul(m, inv);

			auto I = float4x4(
				float4(1, 0, 0, 0),
				float4(0, 1, 0, 0),
				float4(0, 0, 1, 0),
				float4(0, 0, 0, 1));
			PR_EXPECT(FEqlAbsolute(product, I, 1e-4f));
		}
		PRUnitTestMethod(Invert3x3)
		{
			auto m = float3x3(
				float3(1, 2, 3),
				float3(0, 1, 4),
				float3(5, 6, 0));

			auto inv = Invert(m);

			// Compare HLSL vs CPU
			auto cpu_inv = pr::math::Invert(m);
			PR_EXPECT(FEql(inv, cpu_inv));

			auto product = mul(m, inv);

			auto I = float3x3(float3(1, 0, 0), float3(0, 1, 0), float3(0, 0, 1));
			PR_EXPECT(FEql(product, I));
		}
		PRUnitTestMethod(RotationVectorApprox)
		{
			// Small rotation around Z
			float a = 0.01f;
			auto from = float3x3(float3(1, 0, 0), float3(0, 1, 0), float3(0, 0, 1));
			auto to = float3x3(
				float3(cos(a), sin(a), 0),
				float3(-sin(a), cos(a), 0),
				float3(0, 0, 1));

			auto rv = RotationVectorApprox(from, to);

			// Compare HLSL vs CPU
			auto cpu_rv = pr::math::RotationVectorApprox(from, to);
			PR_EXPECT(FEql(rv.x, cpu_rv.x));
			PR_EXPECT(FEql(rv.y, cpu_rv.y));
			PR_EXPECT(FEql(rv.z, cpu_rv.z));

			// Should be approximately (0, 0, a)
			PR_EXPECT(FEqlAbsolute(rv.x, 0.0f, 1e-3f));
			PR_EXPECT(FEqlAbsolute(rv.y, 0.0f, 1e-3f));
			PR_EXPECT(FEqlAbsolute(rv.z, a, 1e-3f));
		}
	};
}
