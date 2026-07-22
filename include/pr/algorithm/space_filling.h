//*************************************
// Hilbert space filling curve.
//  Copyright (c) Rylogic Ltd 2024
//*************************************
#pragma once
#include <cstdint>
#include <cassert>
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

	// The 3D "]"-Order and Z-Order curves below interleave 3 bits per fractal level (one bit from
	// each of x, y, z) into a single 64-bit index. Only floor(64/3) = 21 fractal levels fit before
	// a per-level shift would reach or exceed the 64-bit width of the index, so each axis can only
	// contribute 21 bits to the curve. Each axis is therefore treated as a 21-bit two's-complement
	// value: encoding masks the axis down to its low C_Bits3D bits (the same bits used by the
	// existing, unchanged 2's-complement mapping for small magnitudes), and decoding sign-extends
	// bit (C_Bits3D - 1) back out. This preserves index 0 <-> the origin and every other existing
	// low-magnitude index <-> point mapping exactly, while adding support for the full 21-bit
	// range in both directions. All bit manipulation below uses uint64_t so shifts are always
	// well-defined; the signed int64_t/int32_t API types are only used at the function boundary.
	inline static constexpr int C_Bits3D = 8 * sizeof(int64_t) / 3; // 21 bits per axis (63 of the 64 index bits used)
	inline static constexpr int32_t C_Bias3D = int32_t{1} << (C_Bits3D - 1); // 2^20, magnitude of the representable range either side of zero
	inline static constexpr uint64_t C_Mask3D = (uint64_t{1} << C_Bits3D) - 1; // low C_Bits3D bits

	// Sign-extend the low C_Bits3D bits of 'v' (all other bits must be zero) to a signed 32-bit value.
	// 'v' is < 2^C_Bits3D, so the uint64_t -> int32_t conversion below always represents an in-range,
	// non-negative value - it is not a narrowing/wrapping conversion. The subsequent subtraction is
	// then plain int32_t arithmetic (no unsigned wraparound), keeping every step exactly representable.
	inline constexpr int32_t SignExtend3D(uint64_t v)
	{
		constexpr uint64_t sign_bit = uint64_t{1} << (C_Bits3D - 1);
		auto value = static_cast<int32_t>(v);
		return (v & sign_bit) == 0 ? value : value - (int32_t{1} << C_Bits3D);
	}

	// Convert a ]-Order 3D index to a 3D point.
	// Only the low (3 * C_Bits3D) bits of 'index' are meaningful; higher bits are ignored. The
	// reconstructed point's axes are each in the range [-C_Bias3D, C_Bias3D - 1].
	inline iv4 COrder3D(int64_t index)
	{
		auto idx = static_cast<uint64_t>(index);
		uint64_t x = 0, y = 0, z = 0;
		for (int i = 0; i != C_Bits3D; ++i)
		{
			auto oct = C_Order3DFwd[idx & 0b111]; // oct we're in
			x |= static_cast<uint64_t>((oct >> 0) & 1) << i; // set the bit/axis at this level
			y |= static_cast<uint64_t>((oct >> 1) & 1) << i;
			z |= static_cast<uint64_t>((oct >> 2) & 1) << i;
			idx >>= 3;
		}
		return iv4(SignExtend3D(x), SignExtend3D(y), SignExtend3D(z), 1);
	}

	// Convert a 3D point to an index in the "]" Order 3D space filling curve.
	// 'pt.x', 'pt.y', and 'pt.z' must each be in the range [-C_Bias3D, C_Bias3D - 1] (21 bits);
	// coordinates outside that range cannot be represented by this curve.
	inline int64_t COrder3D(iv4 pt)
	{
		assert(pt.x >= -C_Bias3D && pt.x < C_Bias3D && "COrder3D: x coordinate outside the representable 21-bit range");
		assert(pt.y >= -C_Bias3D && pt.y < C_Bias3D && "COrder3D: y coordinate outside the representable 21-bit range");
		assert(pt.z >= -C_Bias3D && pt.z < C_Bias3D && "COrder3D: z coordinate outside the representable 21-bit range");

		// Truncate each axis to its low C_Bits3D bits (the two's-complement representation of the value).
		uint64_t x = static_cast<uint32_t>(pt.x) & C_Mask3D;
		uint64_t y = static_cast<uint32_t>(pt.y) & C_Mask3D;
		uint64_t z = static_cast<uint32_t>(pt.z) & C_Mask3D;

		uint64_t index = 0;
		for (int i = 0; i != C_Bits3D; ++i)
		{
			auto oct =
				((x & 1) << 0) |
				((y & 1) << 1) |
				((z & 1) << 2);

			index |= static_cast<uint64_t>(C_Order3DInv[oct]) << (3 * i); // safe: 3*i <= 3*(C_Bits3D - 1) = 60 < 64
			x >>= 1;
			y >>= 1;
			z >>= 1;
		}
		return static_cast<int64_t>(index);
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

	// Convert a Z-Order 3D index to a 3D point.
	// Only the low (3 * C_Bits3D) bits of 'index' are meaningful; higher bits are ignored. The
	// reconstructed point's axes are each in the range [-C_Bias3D, C_Bias3D - 1].
	inline iv4 ZOrder3D(int64_t index)
	{
		auto idx = static_cast<uint64_t>(index);
		uint64_t x = 0, y = 0, z = 0;
		for (int i = 0; i != C_Bits3D; ++i)
		{
			x |= (idx & (uint64_t{1} << (3 * i + 0))) >> (2 * i + 0); // safe: 3*i <= 60, 2*i <= 40
			y |= (idx & (uint64_t{1} << (3 * i + 1))) >> (2 * i + 1);
			z |= (idx & (uint64_t{1} << (3 * i + 2))) >> (2 * i + 2);
		}
		return iv4(SignExtend3D(x), SignExtend3D(y), SignExtend3D(z), 1);
	}

	// Convert a 3D point to an index in the Z-Order 3D space filling curve.
	// 'pt.x', 'pt.y', and 'pt.z' must each be in the range [-C_Bias3D, C_Bias3D - 1] (21 bits);
	// coordinates outside that range cannot be represented by this curve.
	inline int64_t ZOrder3D(iv4 pt)
	{
		assert(pt.x >= -C_Bias3D && pt.x < C_Bias3D && "ZOrder3D: x coordinate outside the representable 21-bit range");
		assert(pt.y >= -C_Bias3D && pt.y < C_Bias3D && "ZOrder3D: y coordinate outside the representable 21-bit range");
		assert(pt.z >= -C_Bias3D && pt.z < C_Bias3D && "ZOrder3D: z coordinate outside the representable 21-bit range");

		// Truncate each axis to its low C_Bits3D bits (the two's-complement representation of the value).
		uint64_t x = static_cast<uint32_t>(pt.x) & C_Mask3D;
		uint64_t y = static_cast<uint32_t>(pt.y) & C_Mask3D;
		uint64_t z = static_cast<uint32_t>(pt.z) & C_Mask3D;

		uint64_t index = 0;
		for (int i = 0; i != C_Bits3D; ++i)
		{
			index |= (x & (uint64_t{1} << i)) << (2 * i + 0); // safe: 2*i <= 40 < 64
			index |= (y & (uint64_t{1} << i)) << (2 * i + 1);
			index |= (z & (uint64_t{1} << i)) << (2 * i + 2);
		}
		return static_cast<int64_t>(index);
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
		//  - This function reverses the process of the Hilbert curve.
		//  - 'Order' is the fractal level. 1 = 4 points, 2 = 16 points, 3 = 64 points, etc.
		static_assert(Order > 0 && Order < 32, "Order must be > 0 and < 32");

		// Total number of points in the curve
		constexpr auto MaxPoints = 1LL << (2 * Order);
		constexpr auto SideLength = 1LL << Order;
		assert(pt.x >= 0 && pt.x < SideLength && "Hilbert2D: x coordinate outside the curve domain");
		assert(pt.y >= 0 && pt.y < SideLength && "Hilbert2D: y coordinate outside the curve domain");

		// Reverse each index-to-point transform from the largest scale back to the leaf scale. The
		// existing index-to-point mapping performs Order extra zero-quadrant transforms after consuming
		// all index bits; reversing every transform preserves that established odd/even order orientation.
		uint64_t hilbert_index = 0;
		int64_t x = pt.x, y = pt.y;
		for (auto s = MaxPoints >> 1; s != 0; s >>= 1)
		{
			auto rx = (x & s) != 0 ? 1 : 0;
			auto ry = (y & s) != 0 ? 1 : 0;

			// Remove the quadrant offset before undoing the rotation/reflection at this scale.
			x -= s * rx;
			y -= s * ry;

			if (ry == 0)
			{
				// Reflect across y=x.
				std::swap(x, y);

				if (rx != 0)
				{
					// Rotate 180.
					x = s - 1 - x;
					y = s - 1 - y;
				}
			}

			// Only the lower Order scales encode index bits. Larger scales are the extra zero-quadrant
			// transforms that are part of the existing point mapping but do not consume Hilbert index bits.
			if (s < SideLength)
			{
				auto const digit = (rx << 1) | (ry ^ rx);
				auto const scale = static_cast<uint64_t>(s);
				hilbert_index |= static_cast<uint64_t>(digit) * scale * scale;
			}
		}

		return static_cast<int64_t>(hilbert_index);
	}
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::space_filling::tests
{
	PRUnitTestClass(SpaceFillingTests)
	{
		inline static constexpr bool CreateVisualisations = false;

		PRUnitTestMethod(COrderTest)
		{
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
			if constexpr (CreateVisualisations)
			{
				// Plot them by converting indices to points.
				ldraw::Builder builder;
				auto& line1 = builder.Line("COrder", 0xFFFF0000).strip(v4::Origin());
				for (auto idx : indices)
				{
					auto pt = COrder3D(idx);
					//line1.line_to(v4(To<v2>(pt), 0, 1));
					line1.line_to(To<v4>(pt));
				}
				builder.Save("E:\\dump\\space_filling.ldr");
			}
			#endif
		}
		PRUnitTestMethod(JOrderTest)
		{
			// Sign-extension boundary checks: verify SignExtend3D decodes the raw 21-bit patterns
			// at and either side of the sign boundary (bit C_Bits3D - 1) to the expected signed values.
			PR_EXPECT(SignExtend3D(0) == 0);
			PR_EXPECT(SignExtend3D(1) == 1);
			PR_EXPECT(SignExtend3D(C_Mask3D) == -1); // all C_Bits3D bits set -> -1
			PR_EXPECT(SignExtend3D(uint64_t{1} << (C_Bits3D - 1)) == -C_Bias3D); // sign bit only -> minimum representable value
			PR_EXPECT(SignExtend3D((uint64_t{1} << (C_Bits3D - 1)) - 1) == C_Bias3D - 1); // sign bit clear, all lower bits set -> maximum representable value

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

			// Exact-value checks: index 0 maps to the origin, and small positive per-axis offsets
			// map to the same small indices as the original (unbounded) implementation did for
			// in-range inputs - confirming the low-bit two's-complement mapping is unchanged.
			PR_EXPECT(COrder3D(iv4(0, 0, 0, 1)) == 0);
			PR_EXPECT(COrder3D(iv4(1, 0, 0, 1)) == 1);
			PR_EXPECT(COrder3D(iv4(0, 1, 0, 1)) == 3);
			PR_EXPECT(COrder3D(iv4(0, 0, 1, 1)) == 7);
			PR_EXPECT(All(COrder3D(int64_t(0)) == iv4(0, 0, 0, 1)));

			// Explicit negative-coordinate round trips (in addition to the exhaustive sweep below).
			for (auto pt : { iv4(-1, 0, 0, 1), iv4(0, -1, 0, 1), iv4(0, 0, -1, 1), iv4(-1, -1, -1, 1) })
			{
				auto index = COrder3D(pt);
				auto pt2 = COrder3D(index);
				PR_EXPECT(All(pt2 == pt));
			}
			for (int z = -20; z != 20; ++z)
			{
				for (int y = -20; y != 20; ++y)
				{
					for (int x = -20; x != 20; ++x)
					{
						auto pt = iv4(x, y, z, 1);
						auto index = COrder3D(pt);
						auto pt2 = COrder3D(index);
						PR_EXPECT(All(pt2 == pt));
					}
				}
			}
			for (int y = -100; y != 100; ++y)
			{
				for (int x = -100; x != 100; ++x)
				{
					auto index = COrder2D(iv2(x, y));
					auto pt = COrder2D(index);
					PR_EXPECT(All(pt == iv2(x, y)));
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

			// Exact-value checks: index 0 maps to the origin, and small positive per-axis offsets
			// map to the same small indices as the original (unbounded) implementation did for
			// in-range inputs - confirming the low-bit two's-complement mapping is unchanged.
			PR_EXPECT(ZOrder3D(iv4(0, 0, 0, 1)) == 0);
			PR_EXPECT(ZOrder3D(iv4(1, 0, 0, 1)) == 1);
			PR_EXPECT(ZOrder3D(iv4(0, 1, 0, 1)) == 2);
			PR_EXPECT(ZOrder3D(iv4(0, 0, 1, 1)) == 4);
			PR_EXPECT(All(ZOrder3D(int64_t(0)) == iv4(0, 0, 0, 1)));

			// Explicit negative-coordinate round trips (in addition to the exhaustive sweep below).
			for (auto pt : { iv4(-1, 0, 0, 1), iv4(0, -1, 0, 1), iv4(0, 0, -1, 1), iv4(-1, -1, -1, 1) })
			{
				auto index = ZOrder3D(pt);
				auto pt2 = ZOrder3D(index);
				PR_EXPECT(All(pt2 == pt));
			}
			for (int z = -20; z != 20; ++z)
			{
				for (int y = -20; y != 20; ++y)
				{
					for (int x = -20; x != 20; ++x)
					{
						auto pt = iv4(x, y, z, 1);
						auto index = ZOrder3D(pt);
						auto pt2 = ZOrder3D(index);
						PR_EXPECT(All(pt2 == pt));
					}
				}
			}

			// Boundary values for the 3D curves: the minimum/maximum representable per-axis coordinate
			// (C_Bias3D is the magnitude of the representable range either side of zero, see space_filling.h), plus values either side
			// of zero. This exercises the extremes of the 21-bit-per-axis domain without triggering the
			// out-of-range asserts, and confirms neither curve wraps or corrupts values near the boundary.
			int const bounds3d[] = { -C_Bias3D, -C_Bias3D + 1, -1, 0, 1, C_Bias3D - 2, C_Bias3D - 1 };
			for (auto bz : bounds3d)
			{
				for (auto by : bounds3d)
				{
					for (auto bx : bounds3d)
					{
						auto pt = iv4(bx, by, bz, 1);

						auto j_index = COrder3D(pt);
						auto j_pt = COrder3D(j_index);
						PR_EXPECT(All(j_pt == pt));

						auto z_index = ZOrder3D(pt);
						auto z_pt = ZOrder3D(z_index);
						PR_EXPECT(All(z_pt == pt));
					}
				}
			}

			// Hilbert2D: these exact index-to-point anchors were captured from the existing mapping before
			// changing the inverse. They document the established orientation, including the odd-order x/y
			// swap caused by the extra zero-quadrant transforms in the decoder.
			PR_EXPECT(All(Hilbert2D<1>(0) == iv2(0, 0)));
			PR_EXPECT(All(Hilbert2D<1>(1) == iv2(1, 0)));
			PR_EXPECT(All(Hilbert2D<1>(2) == iv2(1, 1)));
			PR_EXPECT(All(Hilbert2D<1>(3) == iv2(0, 1)));
			PR_EXPECT(All(Hilbert2D<2>(15) == iv2(3, 0)));
			PR_EXPECT(All(Hilbert2D<3>(63) == iv2(0, 7)));
			PR_EXPECT(All(Hilbert2D<4>(255) == iv2(15, 0)));
			PR_EXPECT(All(Hilbert2D<5>(1023) == iv2(0, 31)));
			PR_EXPECT(All(Hilbert2D<6>(4095) == iv2(63, 0)));

			auto hilbert2d_round_trip = []<int Order>()
			{
				constexpr auto SideLength = 1 << Order;
				constexpr auto MaxPoints = 1 << (2 * Order);
				std::vector<bool> seen(MaxPoints, false);

				// Every index must decode to one unique in-bounds point, and that point must encode back
				// to the same index.
				for (int64_t index = 0; index != MaxPoints; ++index)
				{
					auto pt = Hilbert2D<Order>(index);
					PR_EXPECT(pt.x >= 0 && pt.x < SideLength);
					PR_EXPECT(pt.y >= 0 && pt.y < SideLength);

					auto const offset = pt.x + SideLength * pt.y;
					PR_EXPECT(!seen[offset]);
					seen[offset] = true;

					auto index2 = Hilbert2D<Order>(pt);
					PR_EXPECT(index2 == index);
				}

				// Check the point-to-index-to-point direction too, including all boundary points in the
				// square domain for this order.
				for (int y = 0; y != SideLength; ++y)
				{
					for (int x = 0; x != SideLength; ++x)
					{
						auto pt = iv2(x, y);
						auto index = Hilbert2D<Order>(pt);
						auto pt2 = Hilbert2D<Order>(index);
						PR_EXPECT(All(pt2 == pt));
					}
				}
			};
			hilbert2d_round_trip.operator()<1>();
			hilbert2d_round_trip.operator()<2>();
			hilbert2d_round_trip.operator()<3>();
			hilbert2d_round_trip.operator()<4>();
			hilbert2d_round_trip.operator()<5>();
			hilbert2d_round_trip.operator()<6>();
			hilbert2d_round_trip.operator()<8>();

			#if PR_UNITTESTS_VISUALISE
			if constexpr (CreateVisualisations)
			{
				using namespace pr::ldraw;

				// Draw the Hilbert curve
				Builder builder;
				builder.Box("bound", 0xFF00FF00).box(4, 4, 0.1f).pos(2, 2, 0).wireframe();
				auto& line1 = builder.Line("hilbert", 0xFF0000FF).strip(v4::Origin());
				for (int32_t i = 0; i != 16; ++i)
				{
					auto pt = Hilbert2D<2>(i);
					line1.line_to(v4(pt.x * 1.0f, pt.y * 1.0f, 0, 1));
				}
				auto& line2 = builder.Line("hilbert", 0xFFFF0000).strip(v4::Origin());
				for (int32_t i = 0; i != 64; ++i)
				{
					auto pt = Hilbert2D<3>(i);
					line2.line_to(v4(pt.x * 0.5f, pt.y * 0.5f, 0.1f, 1));
				}
				builder.Save(temp_dir() / L"LDraw/hilbert.ldr");
			}
			#endif
		}
	};
}
#endif
