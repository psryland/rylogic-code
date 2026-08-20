//*********************************************
// HLSL CoreTests
//  Copyright (c) Rylogic Ltd 2025
//*********************************************
#include "pr/common/unittests.h"
#include "pr/math/math.h"
#include "pr/hlsl/interop.h"
#include "pr/hlsl/core.hlsli"

namespace pr::hlsl::tests
{
	using namespace pr::math;

	PRUnitTestClass(CoreTests)
	{
		PRUnitTestMethod(VectorMulOrder, Quick)
		{
			auto a = float4(11, 20, 33, 14);
			auto a2b = float4x4(
				float4(16, 15, 14, 13),
				float4(12, 11, 10, 92),
				float4(85, 37, 16, 54),
				float4(42, 13, 42, 11));

			auto b = a2b * a;
			auto B = mul(a, a2b);
			PR_EXPECT(FEql(b, B));
		}
		PRUnitTestMethod(MatrixMulOrder, Quick)
		{
			auto a2b = float4x4(
				float4(11, 20, 33, 14),
				float4(25, 16, 72, 28),
				float4(39, 10, 11, 12),
				float4(13, 14, 15, 16));
			auto b2c = float4x4(
				float4(16, 15, 14, 13),
				float4(12, 11, 10, 92),
				float4(85, 37, 16, 54),
				float4(42, 13, 42, 11));

			auto a2c = b2c * a2b; // Rylogic uses row-major matrices and right to left matrix multiplication
			auto A2C = mul(a2b, b2c); // HLSL mul does left to right multiplication, so the order of the matrices is reversed
			PR_EXPECT(FEql(a2c, A2C));
		}
		PRUnitTestMethod(SignNZ, Quick)
		{
			PR_EXPECT(FEql(sign_nz(+2.0f), +1.0f));
			PR_EXPECT(FEql(sign_nz(-2.0f), -1.0f));
			PR_EXPECT(FEql(sign_nz(-0.0f), +1.0f)); // sign_nz(0) = +1, never zero
		}
		PRUnitTestMethod(LengthSq, Quick)
		{
			auto x2 = float2(3, 4);
			auto x3 = float3(1, 2, 3);
			auto x4 = float4(4, 3, 2, -1);
			PR_EXPECT(FEql(length_sq(x2), LengthSq(x2)));
			PR_EXPECT(FEql(length_sq(x3), LengthSq(x3)));
			PR_EXPECT(FEql(length_sq(x4), LengthSq(x4)));
		}
		PRUnitTestMethod(MinComponentIndex, Quick)
		{
			PR_EXPECT(min_component_index(float2(3, 1)) == 1);
			PR_EXPECT(min_component_index(float2(1, 3)) == 0);
			PR_EXPECT(min_component_index(float3(5, 2, 8)) == 1);
			PR_EXPECT(min_component_index(float3(2, 5, 1)) == 2);
			PR_EXPECT(min_component_index(float4(9, 3, 7, 1)) == 3);
			PR_EXPECT(min_component_index(float4(1, 3, 7, 9)) == 0);
		}
		PRUnitTestMethod(MaxComponentIndex, Quick)
		{
			PR_EXPECT(max_component_index(float2(3, 1)) == 0);
			PR_EXPECT(max_component_index(float2(1, 3)) == 1);
			PR_EXPECT(max_component_index(float3(5, 2, 8)) == 2);
			PR_EXPECT(max_component_index(float3(9, 5, 1)) == 0);
			PR_EXPECT(max_component_index(float4(1, 3, 7, 9)) == 3);
			PR_EXPECT(max_component_index(float4(9, 3, 7, 1)) == 0);
		}
		PRUnitTestMethod(FracParametric, Quick)
		{
			PR_EXPECT(FEql(Frac(0.0f, 0.5f, 1.0f), 0.5f));
			PR_EXPECT(FEql(Frac(0.0f, 0.0f, 1.0f), 0.0f));
			PR_EXPECT(FEql(Frac(0.0f, 1.0f, 1.0f), 1.0f));
			PR_EXPECT(FEql(Frac(10.0f, 15.0f, 20.0f), 0.5f));
		}
		PRUnitTestMethod(ISqrtValues, Quick)
		{
			PR_EXPECT(ISqrt(0) == 0);
			PR_EXPECT(ISqrt(1) == 1);
			PR_EXPECT(ISqrt(4) == 2);
			PR_EXPECT(ISqrt(9) == 3);
			PR_EXPECT(ISqrt(100) == 10);
			PR_EXPECT(ISqrt(99) == 10); // floor(sqrt(99)) = 9, but ISqrt rounds to nearest
			PR_EXPECT(ISqrt(-1) == 0);
		}
		PRUnitTestMethod(HashDeterministic, Quick)
		{
			auto h1 = Hash(42);
			auto h2 = Hash(42);
			PR_EXPECT(h1 == h2);

			// Different inputs should (very likely) produce different hashes
			auto h3 = Hash(43);
			PR_EXPECT(h1 != h3);
		}
		PRUnitTestMethod(HashChain, Quick)
		{
			// Chaining should produce different results
			auto h1 = Hash(1, Hash(0));
			auto h2 = Hash(0, Hash(1));
			PR_EXPECT(h1 != h2);
		}
		PRUnitTestMethod(AllSet, Quick)
		{
			PR_EXPECT(AllSet(0xAA, 0b00001010));
			PR_EXPECT(AllSet(0xAA, 0b10101010));
			PR_EXPECT(!AllSet(0xAA, 0b10101011));
			PR_EXPECT(!AllSet(0xAA, 0b00000011));
		}
		PRUnitTestMethod(AnySet, Quick)
		{
			PR_EXPECT(AnySet(0xAA, 0b00001111));
			PR_EXPECT(AnySet(0xAA, 0b10101010));
			PR_EXPECT(!AnySet(0xAA, 0b01010101));
			PR_EXPECT(!AnySet(0xAA, 0b00000001));
		}
		PRUnitTestMethod(SetFlag, Quick)
		{
			PR_EXPECT(SetFlag(0x00, 0x01, true) == 0x01);
			PR_EXPECT(SetFlag(0xFF, 0x01, false) == 0xFE);
			PR_EXPECT(SetFlag(0x0F, 0x10, true) == 0x1F);
		}
		PRUnitTestMethod(SqrCube, Quick)
		{
			PR_EXPECT(FEql(sqr(3.0f), 9.0f));
			PR_EXPECT(FEql(cube(2.0f), 8.0f));
			PR_EXPECT(FEql(signed_sqr(-3.0f), -9.0f));
			PR_EXPECT(FEql(signed_sqr(3.0f), 9.0f));
		}
	};
}
