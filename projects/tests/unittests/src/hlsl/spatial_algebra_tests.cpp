//*********************************************
// HLSL SpatialAlgebraTests
//  Copyright (c) Rylogic Ltd 2025
//*********************************************
#include "pr/common/unittests.h"
#include "pr/math/math.h"
#include "pr/hlsl/interop.h"
#include "pr/hlsl/spatial_algebra.hlsli"

namespace pr::hlsl::tests
{
	using namespace pr::math;

	PRUnitTestClass(SpatialAlgebraTests)
	{
		PRUnitTestMethod(RodriguesZeroRotation, Quick)
		{
			// Zero axis-angle => identity
			auto R = rodrigues_rotation(float3(0, 0, 0));
			auto I = float3x3(float3(1, 0, 0), float3(0, 1, 0), float3(0, 0, 1));
			PR_EXPECT(FEql(R, I));
		}
		PRUnitTestMethod(Rodrigues90Z, Quick)
		{
			// 90 degrees around Z
			float angle = 3.14159265f / 2.0f;
			auto R = rodrigues_rotation(float3(0, 0, angle));

			// R * (1,0,0) should be (0,1,0)
			auto v = mul(float3(1, 0, 0), R);
			PR_EXPECT(FEql(v, float3(0, 1, 0)));
		}
		PRUnitTestMethod(RodriguesSmallAngle, Quick)
		{
			// Very small rotation should still be orthonormal
			auto R = rodrigues_rotation(float3(1e-10f, 0, 0));

			// Check orthonormality: R * R^T = I
			auto RRt = mul(R, transpose(R));
			auto I = float3x3(float3(1, 0, 0), float3(0, 1, 0), float3(0, 0, 1));
			PR_EXPECT(FEql(RRt, I));
		}
		PRUnitTestMethod(RodriguesInverseProperty, Quick)
		{
			// R(v) * R(-v) = I
			auto axis = float3(0.3f, -0.7f, 0.5f);
			auto R = rodrigues_rotation(axis);
			auto Rinv = rodrigues_rotation(-axis);
			auto product = mul(R, Rinv);
			auto I = float3x3(float3(1, 0, 0), float3(0, 1, 0), float3(0, 0, 1));
			PR_EXPECT(FEql(product, I));
		}
		PRUnitTestMethod(BuildSymmetric, Quick)
		{
			auto m = build_symmetric_3x3(float3(1, 2, 3), float3(4, 5, 6));

			PR_EXPECT(FEql(m[0][0], 1.0f)); // diag.x
			PR_EXPECT(FEql(m[1][1], 2.0f)); // diag.y
			PR_EXPECT(FEql(m[2][2], 3.0f)); // diag.z
			PR_EXPECT(FEql(m[0][1], 4.0f)); // prod.x
			PR_EXPECT(FEql(m[1][0], 4.0f)); // symmetric
			PR_EXPECT(FEql(m[0][2], 5.0f)); // prod.y
			PR_EXPECT(FEql(m[2][0], 5.0f)); // symmetric
			PR_EXPECT(FEql(m[1][2], 6.0f)); // prod.z
			PR_EXPECT(FEql(m[2][1], 6.0f)); // symmetric
		}
		PRUnitTestMethod(RotateInertiaInvIdentity, Quick)
		{
			// Rotating by identity should not change the inertia
			auto I = float3x3(float3(1, 0, 0), float3(0, 1, 0), float3(0, 0, 1));
			auto iinv = float3x3(float3(0.5f, 0, 0), float3(0, 0.25f, 0), float3(0, 0, 0.1f));
			auto result = rotate_inertia_inv(iinv, I);
			PR_EXPECT(FEql(result, iinv));
		}
		PRUnitTestMethod(RotateInertiaInvSymmetric, Quick)
		{
			// Result should always be symmetric
			auto rot = rodrigues_rotation(float3(0.5f, 0.3f, 0.7f));
			auto iinv = build_symmetric_3x3(float3(2, 3, 5), float3(0.1f, 0.2f, 0.3f));
			auto result = rotate_inertia_inv(iinv, rot);

			// Check symmetry
			PR_EXPECT(FEql(result[0][1], result[1][0]));
			PR_EXPECT(FEql(result[0][2], result[2][0]));
			PR_EXPECT(FEql(result[1][2], result[2][1]));
		}
		PRUnitTestMethod(Orthonorm3x3, Quick)
		{
			// Start with a slightly non-orthonormal matrix
			auto m = float3x3(
				float3(1.01f, 0.02f, 0),
				float3(-0.01f, 0.99f, 0.03f),
				float3(0, -0.02f, 1.01f));

			auto o = orthonorm3x3(m);

			// Check orthonormality
			PR_EXPECT(FEql(length(o[0]), 1.0f));
			PR_EXPECT(FEql(length(o[1]), 1.0f));
			PR_EXPECT(FEql(length(o[2]), 1.0f));
			PR_EXPECT(FEql(dot(o[0], o[1]), 0.0f));
			PR_EXPECT(FEql(dot(o[0], o[2]), 0.0f));
			PR_EXPECT(FEql(dot(o[1], o[2]), 0.0f));

			// Check right-handedness: z = cross(x, y)
			auto z = cross(o[0], o[1]);
			PR_EXPECT(FEql(z, o[2]));
		}
		PRUnitTestMethod(SpatialDot, Quick)
		{
			auto ang_a = float3(1, 0, 0);
			auto lin_a = float3(0, 1, 0);
			auto ang_b = float3(2, 3, 0);
			auto lin_b = float3(0, 4, 5);

			auto result = spatial_dot(ang_a, lin_a, ang_b, lin_b);

			// dot(ang) = 1*2 + 0*3 + 0*0 = 2
			// dot(lin) = 0*0 + 1*4 + 0*5 = 4
			PR_EXPECT(FEql(result, 6.0f));
		}
	};
}
