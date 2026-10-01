//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"

namespace pr::rdr12
{
	// Return a scale-bounded normal transform for an affine placement; normalize each resulting nonzero normal before lighting.
	// Invertible placements match inverse transpose, including reflections. Rank-two placements retain surviving plane normals;
	// collapsed directions return zero because they have no defined surface normal.
	inline m4x4 NormalTransform(m4x4 const& placement)
	{
		// Each cofactor column is the cross product of the other two placement axes. Double intermediates keep products of finite
		// float scales from overflowing or underflowing. The arithmetic is written out in full because this runs for every drawn element.
		double const xx = placement.x.x, xy = placement.x.y, xz = placement.x.z;
		double const yx = placement.y.x, yy = placement.y.y, yz = placement.y.z;
		double const zx = placement.z.x, zy = placement.z.y, zz = placement.z.z;
		double const cofactors[3][3] = {
			{ yy * zz - yz * zy, yz * zx - yx * zz, yx * zy - yy * zx },
			{ zy * xz - zz * xy, zz * xx - zx * xz, zx * xy - zy * xx },
			{ xy * yz - xz * yy, xz * yx - xx * yz, xx * yy - xy * yx },
		};

		// Find the largest cofactor magnitude. The sum of magnitudes is not finite if any cofactor is infinite or NaN.
		auto largest = 0.0, total = 0.0;
		for (auto const& column : cofactors)
		{
			for (auto value : column)
			{
				auto const magnitude = value < 0 ? -value : value;
				largest = magnitude > largest ? magnitude : largest;
				total += magnitude;
			}
		}
		if (!std::isfinite(total))
			throw std::invalid_argument("Normal transformation requires a finite affine placement");

		// A common positive scale preserves directions; the determinant sign retains inverse-transpose orientation under reflection.
		auto result = m4x4::Zero();
		result.pos.w = 1;
		if (largest == 0)
			return result;

		auto const determinant = xx * cofactors[0][0] + xy * cofactors[0][1] + xz * cofactors[0][2];
		auto const sign = determinant < 0 ? -1.0 : 1.0;
		result.x = v4(float(sign * (cofactors[0][0] / largest)), float(sign * (cofactors[0][1] / largest)), float(sign * (cofactors[0][2] / largest)), 0);
		result.y = v4(float(sign * (cofactors[1][0] / largest)), float(sign * (cofactors[1][1] / largest)), float(sign * (cofactors[1][2] / largest)), 0);
		result.z = v4(float(sign * (cofactors[2][0] / largest)), float(sign * (cofactors[2][1] / largest)), float(sign * (cofactors[2][2] / largest)), 0);
		return result;
	}
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::rdr12::tests
{
	PRUnitTest(NormalTransformTests, Quick)
	{
		// Build an affine placement from its three axes
		auto placement = [](v4 x, v4 y, v4 z)
		{
			// Test placements have no translation because normal transforms ignore it
			return m4x4(x, y, z, v4::Origin());
		};

		// Rotations are their own normal transform
		{
			auto const rot = placement(v4(0, 1, 0, 0), v4(-1, 0, 0, 0), v4(0, 0, 1, 0));
			PR_EXPECT(FEql(NormalTransform(rot), rot));
			PR_EXPECT(FEql(NormalTransform(m4x4::Identity()), m4x4::Identity()));
		}

		// Non-uniform scale uses inverse scales, bounded so the largest component is one
		{
			auto const nt = NormalTransform(placement(v4(2, 0, 0, 0), v4(0, 4, 0, 0), v4(0, 0, 8, 0)));
			PR_EXPECT(FEql(nt, placement(v4(1, 0, 0, 0), v4(0, 0.5f, 0, 0), v4(0, 0, 0.25f, 0))));
		}

		// Reflections keep the inverse-transpose orientation
		{
			auto const nt = NormalTransform(placement(v4(-1, 0, 0, 0), v4(0, 1, 0, 0), v4(0, 0, 1, 0)));
			PR_EXPECT(FEql(nt, placement(v4(-1, 0, 0, 0), v4(0, 1, 0, 0), v4(0, 0, 1, 0))));
		}

		// Rank-two placements keep the surviving plane normal; fully collapsed placements give zero
		{
			auto const flat = NormalTransform(placement(v4(1, 0, 0, 0), v4(0, 1, 0, 0), v4(0, 0, 0, 0)));
			PR_EXPECT(FEql(flat, placement(v4(0, 0, 0, 0), v4(0, 0, 0, 0), v4(0, 0, 1, 0))));

			auto const line = NormalTransform(placement(v4(0, 0, 0, 0), v4(0, 0, 0, 0), v4(0, 0, 1, 0)));
			PR_EXPECT(FEql(line, placement(v4(0, 0, 0, 0), v4(0, 0, 0, 0), v4(0, 0, 0, 0))));
			PR_EXPECT(line.pos.w == 1.0f);
		}

		// Non-finite placements are rejected
		{
			auto const inf = std::numeric_limits<float>::infinity();
			auto const nan = std::numeric_limits<float>::quiet_NaN();
			PR_THROWS(NormalTransform(placement(v4(inf, 0, 0, 0), v4(0, 1, 0, 0), v4(0, 0, 1, 0))), std::invalid_argument);
			PR_THROWS(NormalTransform(placement(v4(1, 0, 0, 0), v4(0, nan, 0, 0), v4(0, 0, 1, 0))), std::invalid_argument);
		}
	}
}
#endif
