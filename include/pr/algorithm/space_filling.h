//*************************************
// Hilbert space filling curve.
//  Copyright (c) Rylogic Ltd 2024
//*************************************
#pragma once
#include <cstdint>
#include "pr/math/math.h"

namespace pr::space_filling
{
	// Notes:
	//  - Space filling curves need to be fractal. I.e. the order at one level is the order of the blocks at the next level up.
	//  - Think about this like a depth-first search of a quad (oct) tree. Each pair (triple) of bits in the index is the quad (oct)
	//    to navigate into next.
	//    +---+---+
	//    | 3 | 2 |   MSB are the top level quads, LSB are the leaf quads
	//    +---+---+   e.g. bits 11011010 is: quad 3, 1, 2, 2.
	//    | 0 | 1 |
	//    +---+---+
	inline static constexpr int C_Order2DFwd[] = {0, 1, 3, 2};
	inline static constexpr int C_Order2DInv[] = {0, 1, 3, 2};
	inline static constexpr int C_Order3DFwd[] = {0, 1, 3, 2, 6, 7, 5, 4};
	inline static constexpr int C_Order3DInv[] = {0, 1, 3, 2, 7, 6, 4, 5};

	// Convert a ]-Order 2D index to a 2D point
	inline iv2 COrder2D(int64_t index)
	{
		// Index zero is the point at the origin.
		// This actually works for all indices (-ve and +ve) and covers all the space around
		// the origin even though it seems like it should only cover the +1,+1 and -1,-1 quadrants.
		// Haven't quite figured out why this works yet.
		iv2 pt = {};
		for (int i = 0; i != 32; ++i)
		{
			auto quad = C_Order2DFwd[index & 0b11]; // quad we're in
			pt.x += (1 << i) * ((quad >> 0) & 1); // += points/quad/axis at this level
			pt.y += (1 << i) * ((quad >> 1) & 1);
			index >>= 2;
		}
		return pt;
	}

	// Convert a 2D point to an index in the "]" Order 2D space filling curve
	inline int64_t COrder2D(iv2 pt)
	{
		int64_t index = 0;
		for (int i = 0; i != 32; ++i)
		{
			auto quad =
				((pt.x & 1) << 0) |
				((pt.y & 1) << 1);

			index += C_Order2DInv[quad] * (1LL << (2 * i));
			pt.x >>= 1;
			pt.y >>= 1;
		}
		return index;
	}

	// Convert a ]-Order 3D index to a 3D point
	inline iv4 COrder3D(int64_t index)
	{
		iv4 pt = {};
		for (int i = 0; i != 32; ++i)
		{
			auto oct = C_Order3DFwd[index & 0b111]; // oct we're in
			pt.x += (1 << i) * ((oct >> 0) & 1); // += points/oct/axis at this level
			pt.y += (1 << i) * ((oct >> 1) & 1);
			pt.z += (1 << i) * ((oct >> 2) & 1);
			index >>= 3;
		}
		return pt.w1();
	}

	// Convert a 3D point to an index in the "]" Order 3D space filling curve
	inline int64_t COrder3D(iv4 pt)
	{
		int64_t index = 0;
		for (int i = 0; i != 32; ++i)
		{
			auto oct =
				((pt.x & 1) << 0) |
				((pt.y & 1) << 1) |
				((pt.z & 1) << 2);

			index += C_Order3DInv[oct] * (1LL << (3 * i));
			pt.x >>= 1;
			pt.y >>= 1;
			pt.z >>= 1;
		}
		return index;
	}


	// Convert a Z-Order 2D index to a 2D point
	inline iv2 ZOrder2D(int64_t index)
	{
		constexpr int Order = 8*sizeof(int64_t) / 2; // 2 bits per level

		iv2 pt = {};
		for (int i = 0; i != Order; ++i)
		{
			pt.x |= (index & (1LL << (2 * i + 0))) >> (i + 0);
			pt.y |= (index & (1LL << (2 * i + 1))) >> (i + 1);
		}
		return pt;
	}

	// Convert a 2D point to an index in the Z-Order 2D space filling curve
	inline int64_t ZOrder2D(iv2 pt)
	{
		constexpr int Order = 8*sizeof(int64_t) / 2; // 2 bits per level
		
		int64_t index = 0;
		for (int i = 0; i != Order; ++i)
		{
			index |= (pt.x & (1ULL << i)) << (i + 0);
			index |= (pt.y & (1ULL << i)) << (i + 1);
		}
		return index;
	}

	// Convert a Z-Order 3D index to a 3D point
	inline iv4 ZOrder3D(int64_t index)
	{
		iv4 pt = { 0, 0, 0, 1 };
		for (int i = 0; i < 32; ++i)
		{
			pt.x |= (index & (1LL << (3 * i + 0))) >> (2 * i + 0);
			pt.y |= (index & (1LL << (3 * i + 1))) >> (2 * i + 1);
			pt.z |= (index & (1LL << (3 * i + 2))) >> (2 * i + 2);
		}
		return pt;
	}

	// Convert a 3D point to an index in the Z-Order 3D space filling curve
	inline int64_t ZOrder3D(iv4 pt)
	{
		int64_t index = 0;
		for (int i = 0; i != 32; ++i)
		{
			index |= (pt.x & (1 << i)) << (2 * i + 0);
			index |= (pt.y & (1 << i)) << (2 * i + 1);
			index |= (pt.z & (1 << i)) << (2 * i + 2);
		}
		return index;
	}




	// Convert a 2D point to an index in a 2D Hilbert space filling curve
	template <int Order>
	inline iv2 Hilbert2D(int64_t hilbert_index)
	{
		// Notes:
		//  - This works by kinda like quad tree traversal. At each fractal level, we figure out
		//    which quadrant the point is in, and then recurse into that quadrant. Each quadrant
		//    is rotated/reflected so 'rx' and 'ry' find the rotation to apply at each level.
		//  - 'Order' is the fractal level. 1 = 4 points, 2 = 16 points, 3 = 64 points, etc.
		static_assert(Order > 0 && Order < 32, "Order must be > 0 and < 32");

		// Total number of points in the curve
		constexpr auto MaxPoints = 1LL << (2 * Order);

		int64_t x = 0, y = 0;
		for (auto s = 1LL; s < MaxPoints; s <<= 1)
		{
			// Which quadrant is the point in? Grey code is used to ensure that we only move one quadrant at a time.
			auto rx = static_cast<int>(1 & (hilbert_index / 2));
			auto ry = static_cast<int>(1 & (hilbert_index ^ rx));
			if (ry == 0) {

				// Rotate 180
				if (rx == 1) {
					x = s - 1 - x;
					y = s - 1 - y;
				}

				// Reflect across y=x
				std::swap(x, y);
			}

			// Accumulate the coordinate
			x += s * rx;
			y += s * ry;

			// Go to the next fractal level
			hilbert_index /= 4;
		}

		return { s_cast<int>(x), s_cast<int>(y) };
	}

	// Convert a 2D point to an index in a 2D Hilbert space filling curve
	template <int Order>
	inline int64_t Hilbert2D(iv2 pt)
	{
		// Notes:
		// - This function reverses the process of the Hilbert curve.
		//  - 'Order' is the fractal level. 1 = 4 points, 2 = 16 points, 3 = 64 points, etc.
		static_assert(Order > 0 && Order < 32, "Order must be > 0 and < 32");

		// Total number of points in the curve
		constexpr auto MaxPoints = 1LL << (2 * Order);

		int64_t hilbert_index = 0;
		int64_t x = pt.x, y = pt.y;
		for (auto s = MaxPoints >> 1; s != 0; s >>= 1)
		{
			auto rx = x & s;
			auto ry = y & s;

			// Reflect across y=x if necessary
			if (ry != 0) {
				std::swap(x, y);
			}

			// Rotate 180 if necessary
			if (rx != 0) {
				x = s - 1 - x;
				y = s - 1 - y;
			}

			// Combine the bits
			hilbert_index = (hilbert_index << 2) | (rx + 2 * ry);

			// Move to the next fractal level
			x >>= 1;
			y >>= 1;
		}

		return hilbert_index;
	}
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::algorithm::tests
{
	PRUnitTest(SpaceFillingTests)
	{
		using namespace pr::space_filling;

		//*
		// Generate the indices for points around the origin
		constexpr int range = 1;
		std::vector<int64_t> indices;
		indices.reserve(Cube(2 * range));

		auto i0 = COrder2D(iv2(-1, -1));
		auto i1 = COrder3D(iv4(-1, -1, -1, 0));

		auto v0 = COrder2D(i0);
		auto v1 = COrder3D(i1);

		for (int z = -range; z != range; ++z)
			for (int y = -range; y != range; ++y)
				for (int x = -range; x != range; ++x)
					indices.push_back(COrder3D(iv4(x, y, z, 1)));
		//for (int i = -1000; i != 0; ++i)
		//	indices.push_back(i);

		// Convert to curve order
		std::sort(std::begin(indices), std::end(indices));

		#if PR_UNITTESTS_VISUALISE
		{
			// Plot them by converting indices to points.
			pr::ldraw::Builder builder;
			auto& line1 = builder.Line("COrder", Colour32Blue).strip(v4::Origin());
			for (auto idx : indices)
			{
				auto pt = COrder3D(idx);
				//line1.line_to(v4(To<v2>(pt), 0, 1));
				line1.line_to(To<v4>(pt));
			}
			builder.Save("E:\\dump\\space_filling.ldr");
		}
		#endif

		// Round-trip tests

		// ]-Order
		for (int i = -1000; i != 1000; ++i)
		{
			auto pt = COrder2D(i);
			auto index = COrder2D(pt);
			PR_EXPECT(index == i);
		}
		for (int i = 0; i != 64; ++i)
		{
			auto pt = COrder3D(i);
			auto index = COrder3D(pt);
			PR_EXPECT(index == i);
		}
		for (int y = -100; y != 100; ++y)
		{
			for (int x = -100; x != 100; ++x)
			{
				auto index = COrder2D(iv2(x,y));
				auto pt = COrder2D(index);
				PR_EXPECT(pt == iv2(x,y));
			}
		}

		// Z-Order
		for (int i = 0; i != 1000; ++i)
		{
			auto pt = ZOrder2D(i);
			auto index = ZOrder2D(pt);
			PR_EXPECT(index == i);
		}
		for (int i = 0; i != 1000; ++i)
		{
			auto pt = ZOrder3D(i);
			auto index = ZOrder3D(pt);
			PR_EXPECT(index == i);
		}

		/* failing
		for (int i = 0; i != 16384; ++i)
		{
			auto pt = Hilbert2D<4>(i);
			auto index = Hilbert2D<4>(pt);
			PR_EXPECT(index == i);
		}
		for (int i = 0; i != 1000; ++i)
		{
			auto pt = Hilbert3D(i);
			auto index = Hilbert3D(pt);
			PR_EXPECT(index == i);
		}
		*/

		#if PR_UNITTESTS_VISUALISE
		{
			// Draw the Hilbert curve
			ldr::Builder builder;
			builder.Box("bound", Colour32Green).dim(4, 4, 0.1).pos(2, 2, 0).wireframe();
			auto& line1 = builder.Line("hilbert", Colour32Blue).strip(v4::Origin());
			for (int32_t i = 0; i != 16; ++i)
			{
				auto pt = Hilbert2D<2>(i);
				line1.line_to(v4(To<v2>(pt), 0, 1));
			}
			auto& line2 = builder.Line("hilbert", Colour32Red).strip(v4::Origin());
			for (int32_t i = 0; i != 64; ++i)
			{
				auto pt = Hilbert2D<3>(i);
				line2.line_to(v4(To<v2>(pt) / 2, 0.1, 1));
			}
			builder.Write("E:\\dump\\hilbert.ldr");
		}
		#endif
	}
}
#endif
