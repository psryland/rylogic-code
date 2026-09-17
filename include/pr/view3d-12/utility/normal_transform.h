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
		// Double intermediates keep products of finite float scales from overflowing or underflowing.
		auto cofactors = std::array<std::array<double, 3>, 3>{};
		auto largest = 0.0;
		for (auto column = 0; column != 3; ++column)
		{
			auto const& a = placement[(column + 1) % 3];
			auto const& b = placement[(column + 2) % 3];
			for (auto row = 0; row != 3; ++row)
			{
				auto const j = (row + 1) % 3;
				auto const k = (row + 2) % 3;
				auto const value = double(a[j]) * b[k] - double(a[k]) * b[j];
				if (!std::isfinite(value))
					throw std::invalid_argument("Normal transformation requires a finite affine placement");

				cofactors[column][row] = value;
				largest = std::max(largest, std::abs(value));
			}
		}

		// A common positive scale preserves directions; the determinant sign retains inverse-transpose orientation under reflection.
		auto result = m4x4::Zero();
		result.pos.w = 1;
		if (largest == 0)
			return result;

		auto const determinant = double(placement.x.x) * cofactors[0][0] + double(placement.x.y) * cofactors[0][1] + double(placement.x.z) * cofactors[0][2];
		auto const sign = determinant < 0 ? -1.0 : 1.0;
		for (auto column = 0; column != 3; ++column)
			for (auto row = 0; row != 3; ++row)
				result[column][row] = static_cast<float>(sign * (cofactors[column][row] / largest));

		return result;
	}
}
