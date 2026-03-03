//*****************************************************************************
// Maths library
//  Copyright (c) Rylogic Ltd 2002
//*****************************************************************************
#pragma once
#include "pr/maths/maths.h"

namespace pr
{
	// Compress a normalised 3-vector into a v2 (almost) losslessly
	struct Norm64bit
	{
		static v2 Compress(v4 const& norm)
		{
			// Find the largest component
			auto abs_norm = Abs(norm);
			auto i = MaxElementIndex(abs_norm.xyz);

			// Save the sign and 'w' component of the dropped component
			auto s = 0;
			s |= norm[i] < 0.0f ? 0x2 : 0;
			s |= norm.w != 0.0f ? 0x1 : 0;

			v2 result;
			result.x = norm[(i + 1) % 3];
			result.y = norm[(i + 2) % 3];

			// Encode the index or the dropped component, the sign, and the 'w' value in the LSB
			reinterpret_cast<uint32_t&>(result.x) = SetBits(reinterpret_cast<uint32_t const&>(result.x), 0x3, i);
			reinterpret_cast<uint32_t&>(result.y) = SetBits(reinterpret_cast<uint32_t const&>(result.y), 0x3, s);

			return result;
		}
		static v4 Decompress(v2 const& packed_norm)
		{
			auto i = reinterpret_cast<uint32_t const&>(packed_norm.x) & 0x3;
			auto s = reinterpret_cast<uint32_t const&>(packed_norm.y) & 0x3;
			auto w = (s & 0x1) != 0 ? 1.0f : 0.0f;
			auto sign = (s & 0x2) != 0 ? -1.0f : +1.0f;

			v4 result;
			result[i] = sign * Sqrt(1.0f - LengthSq(packed_norm));
			result[(i + 1) % 3] = packed_norm.x;
			result[(i + 2) % 3] = packed_norm.y;
			result.w = w;
			return result;
		}
	};
	
	// Compress a normalised 3-vector into 32bits
	struct Norm32bit
	{
		// Best compression method (max error 0.0001)
		static uint32_t Compress(v4 const& normal)
		{
			auto sign_bits =
				(int(normal.x >= 0) << 2) |
				(int(normal.y >= 0) << 1) |
				(int(normal.z >= 0) << 0);

			// Project onto the plane x+y+z=1
			auto x = abs(normal.x);
			auto y = abs(normal.y);
			auto z = abs(normal.z);
			auto sum = x + y + z;
			x /= sum;
			y /= sum;
			z /= sum;

			auto iy = int((1.0f - y) * MaxYDiv);
			auto xz_scale = 1.0f - y;
			auto MaxXZDiv = 2 * iy;
			auto ixz = xz_scale > maths::tinyf ? int((z / xz_scale) * float(MaxXZDiv)) : 0;
			assert(ixz <= MaxXZDiv);

			auto index = iy * iy + ixz;
			assert((index & IndexMask) == index);

			return uint32_t((sign_bits << IndexBits) | index);
		}
		static v4 Decompress(uint32_t packed_normal)
		{
			auto sign_bits = packed_normal >> IndexBits;
			auto index     = packed_normal & IndexMask;
		
			auto iy  = static_cast<int>(floor(sqrt(static_cast<double>(index))));
			auto ixz = index - iy * iy;
			auto MaxXZDiv = 2 * iy;
			auto xz_scale = iy / MaxYDiv;
		
			float x, y, z;
			if (iy != 0)
			{
				auto xz_ratio = ixz / static_cast<float>(MaxXZDiv);
				x = (1.0f - xz_ratio) * xz_scale;
				y = (1.0f - xz_scale);
				z = (xz_ratio       ) * xz_scale;
			}
			else
			{
				x = 0.0f;
				y = 1.0f;
				z = 0.0f;
			}
		
			v4 direction;
			direction.x = (float((sign_bits >> 2) & 0x1) * 2.0f - 1.0f) * x;
			direction.y = (float((sign_bits >> 1) & 0x1) * 2.0f - 1.0f) * y;
			direction.z = (float((sign_bits >> 0) & 0x1) * 2.0f - 1.0f) * z;
			direction.w = 0;
			return Normalise(direction);
		}

		// An alternative compression method (max error 0.012)
		static uint32_t Compress2(v4 const& norm)
		{
			// Drop Z component
			// 1 bit: z sign
			// 16 bits: x (with LSB = 1 if negative)
			// 15 bits: y (with LSB = 1 if negative)
			// Overwriting the LSB with the sign seems to have better accuracy on average.
			// Probably because half of the time the LSB matches the sign anyway.

			auto abs_norm = Abs(norm);
			auto s = int(norm.z < 0);
			auto a = static_cast<uint32_t>(Clamp(abs_norm.x, 0.0f, 1.0f) * 0xFFFF);
			auto b = static_cast<uint32_t>(Clamp(abs_norm.y, 0.0f, 1.0f) * 0x7FFF);
			a = SetBits(a, 1, norm.x < 0);
			b = SetBits(b, 1, norm.y < 0);
			return
				(s << 31) |
				(a << 15) |
				(b << 0);
		}
		static v4 Decompress2(uint32_t packed_norm)
		{
			auto s = (packed_norm >> 31) & 0x1;
			auto a = (packed_norm >> 15) & 0xFFFF;
			auto b = (packed_norm >>  0) & 0x7FFF;

			v4 norm;
			norm.x = ((a & 1) ? -1.0f : 1.0f) * a / 0xFFFF;
			norm.y = ((b & 1) ? -1.0f : 1.0f) * b / 0x7FFF;
			norm.z = (s ? -1.0f : 1.0f) * Sqrt(Clamp(1.0f - Sqr(norm.x) - Sqr(norm.y), 0.0f, 1.0f));
			norm.w = 0;
			return norm;
		}

	private:

		static constexpr int IndexBits = 32 - 3;
		static constexpr int IndexMask = (1 << IndexBits) - 1;
		static constexpr float MaxYDiv = 23169.0f;  // = floor(sqrt(1 << IndexBits)) - 1
	};

	// Compress a normalised 3-vector into 16bits
	struct Norm16bit
	{
		// Best compression method (max error 0.029)
		static uint16_t Compress(v4 const& normal)
		{
			// ~0.7 degrees angular error measured empirically
			auto sign_bits =
				(int(normal.x >= 0) << 2) |
				(int(normal.y >= 0) << 1) |
				(int(normal.z >= 0) << 0);

			// Project onto the plane x+y+z=1
			auto x = abs(normal.x);
			auto y = abs(normal.y);
			auto z = abs(normal.z);
			auto sum = x + y + z;
			x /= sum;
			y /= sum;
			z /= sum;

			auto iy = int((1.0f - y) * MaxYDiv);
			auto xz_scale = 1.0f - y;
			auto MaxXZDiv = 2 * iy;
			auto ixz = xz_scale > maths::tinyf ? int((z / xz_scale) * float(MaxXZDiv)) : 0;
			assert(ixz <= MaxXZDiv);

			auto index = iy * iy + ixz;
			assert((index & IndexMask) == index);

			return uint16_t((sign_bits << IndexBits) | index);
		}
		static v4 Decompress(uint16_t packed_normal)
		{
			auto sign_bits = packed_normal >> IndexBits;
			auto index     = packed_normal & IndexMask;
		
			auto iy  = static_cast<int>(Floor(Sqrt(static_cast<float>(index))));
			auto ixz = index - iy * iy;
			auto MaxXZDiv = 2 * iy;
			auto xz_scale = iy / MaxYDiv;
		
			float x, y, z;
			if (iy != 0)
			{
				auto xz_ratio = ixz / static_cast<float>(MaxXZDiv);
				x = (1.0f - xz_ratio) * xz_scale;
				y = (1.0f - xz_scale);
				z = (xz_ratio       ) * xz_scale;
			}
			else
			{
				x = 0.0f;
				y = 1.0f;
				z = 0.0f;
			}
		
			v4 direction;
			direction.x = (float((sign_bits >> 2) & 0x1) * 2.0f - 1.0f) * x;
			direction.y = (float((sign_bits >> 1) & 0x1) * 2.0f - 1.0f) * y;
			direction.z = (float((sign_bits >> 0) & 0x1) * 2.0f - 1.0f) * z;
			direction.w = 0;
			return Normalise(direction);
		}

		// An alternative compression method (max error 0.04)
		static uint16_t Compress2(v4 const& vec)
		{
			auto tmp = vec;

			// input vector does not have to be unit length
			// assert( tmp.length() <= 1.001f );      
			uint16_t mVec = 0;

			if (tmp.x < 0) { mVec |= XSIGN_MASK; tmp.x = -tmp.x; }
			if (tmp.y < 0) { mVec |= YSIGN_MASK; tmp.y = -tmp.y; }
			if (tmp.z < 0) { mVec |= ZSIGN_MASK; tmp.z = -tmp.z; }

			// project the normal onto the plane that goes through
			// X0=(1,0,0),Y0=(0,1,0),Z0=(0,0,1).

			// on that plane we choose an (projective!) coordinate system
			// such that X0->(0,0), Y0->(126,0), Z0->(0,126),(0,0,0)->Infinity

			// a little slower... old pack was 4 multiplies and 2 adds. 
			// This is 2 multiplies, 2 adds, and a divide....
			float w = 126.0f / (tmp.x + tmp.y + tmp.z);
			long xbits = (long)(tmp.x * w);
			long ybits = (long)(tmp.y * w);

			assert(xbits < 127);
			assert(xbits >= 0);
			assert(ybits < 127);
			assert(ybits >= 0);

			// Now we can be sure that 0<=xp<=126, 0<=yp<=126, 0<=xp+yp<=126

			// however for the sampling we want to transform this triangle 
			// into a rectangle.
			if (xbits >= 64)
			{
				xbits = 127 - xbits;
				ybits = 127 - ybits;
			}

			// now we that have xp in the range (0,127) and yp in the 
			// range (0,63), we can pack all the bits together
			mVec |= (xbits << 7);
			mVec |= ybits;
			return mVec;
		}
		static v4 Decompress2(uint16_t mVec)
		{
			// if we do a straightforward backward transform
			// we will get points on the plane X0,Y0,Z0
			// however we need points on a sphere that goes through 
			// these points.
			// therefore we need to adjust x,y,z so that x^2+y^2+z^2=1

			// by normalizing the vector. We have already precalculated 
			// the amount by which we need to scale, so all we do is a 
			// table lookup and a multiplication
			v4 vec;

			// get the x and y bits
			long xbits = ((mVec & TOP_MASK) >> 7);
			long ybits = (mVec & BOTTOM_MASK);

			// map the numbers back to the triangle (0,0)-(0,126)-(126,0)
			if ((xbits + ybits) >= 127)
			{
				xbits = 127 - xbits;
				ybits = 127 - ybits;
			}

			// do the inverse transform and normalization
			// costs 3 extra multiplies and 2 subtracts. No big deal.         
			float uvadj = UVAdjustment(mVec & ~SIGN_MASK);
			vec.x = uvadj * (float)xbits;
			vec.y = uvadj * (float)ybits;
			vec.z = uvadj * (float)(126 - xbits - ybits);

			// set all the sign bits
			if (mVec & XSIGN_MASK) vec.x = -vec.x;
			if (mVec & YSIGN_MASK) vec.y = -vec.y;
			if (mVec & ZSIGN_MASK) vec.z = -vec.z;

			return vec;
		}

	private:

		static constexpr int IndexBits = 16 - 3;
		static constexpr int IndexMask = (1 << IndexBits) - 1;
		static constexpr float MaxYDiv = 89.0f;  // = floor(sqrt(1 << IndexBits)) - 1

		// upper 3 bits
		static constexpr int SIGN_MASK   = 0xe000;
		static constexpr int XSIGN_MASK  = 0x8000;
		static constexpr int YSIGN_MASK  = 0x4000;
		static constexpr int ZSIGN_MASK  = 0x2000;
		static constexpr int TOP_MASK    = 0x1f80; // middle 6 bits - xbits
		static constexpr int BOTTOM_MASK = 0x007f; // lower 7 bits - ybits

		static float UVAdjustment(uint16_t idx)
		{
			static float mUVAdjustment[0x2000];
			static bool initialised = []
			{
				for (int idx = 0; idx != 0x2000; idx++)
				{
					long xbits = idx >> 7;
					long ybits = idx & BOTTOM_MASK;

					// map the numbers back to the triangle (0,0)-(0,127)-(127,0)
					if ((xbits + ybits) >= 127)
					{
						xbits = 127 - xbits;
						ybits = 127 - ybits;
					}

					// convert to 3D vectors
					float x = (float)xbits;
					float y = (float)ybits;
					float z = (float)(126 - xbits - ybits);

					// calculate the amount of normalization required
					mUVAdjustment[idx] = 1.0f / sqrtf(y * y + z * z + x * x);
					assert(_finite(mUVAdjustment[idx]));
				}
				return true;
			}();
			return mUVAdjustment[idx];
		}
	};

	// Returns a direction in 5 bits. (Actually a number < 27)
	struct Norm5bit
	{
		// Notes:
		//  - This can be converted into 4 bits if sign information isn't needed
		//    using: if (idx > 13) idx = 26 - idx;
		//    Doing so, does not effect the Decompress() function
		static uint32_t Compress(v4 norm)
		{
			const float cos_67p5 = 0.382683f;
			uint32_t x = (norm.x > cos_67p5) - (norm.x < -cos_67p5) + 1;
			uint32_t y = (norm.y > cos_67p5) - (norm.y < -cos_67p5) + 1;
			uint32_t z = (norm.z > cos_67p5) - (norm.z < -cos_67p5) + 1;
			return x + y*3 + z*9;
		}
		static v4 Decompress(uint32_t idx, bool renorm = true)
		{
			int x = (idx % 3);
			int y = (idx % 9) / 3;
			int z = (idx / 9);
			return renorm
				? v4::Normal(x - 1.0f, y - 1.0f, z - 1.0f, 0.0f)
				: v4{x - 1.0f, y - 1.0f, z - 1.0f, 0};
		}
	};

	// Compress a quaternion into a uint32
	struct Quat32bit
	{
		// Compress a normalised quaternion into 32bits.
		// The algorithm and reasoning is as follows:
		// - We can drop one component because it can be recreated using the knowledge that the quat is normalised.
		//   Dropping the largest component gives the best accuracy at the cost of having to remember which component
		//   was dropped.
		// - Dropping the largest component means the magnitude of the next largest component can be no greater than
		//   1/root(2) (0.707106), the third largest no greater than 1/root(3) (0.577350), and 4th component no greater
		//   than 1/root(4) (0.5).
		// - Proportioning the bits used to store each component with these ratios gives the greatest accuracy.
		// - Since the 3rd and 4th components have roughly the same proportion, use an equal number of bits for these and store
		//   them in order so that the component position does not need to be stored.
		// - Reserve 4 bits for the index of the component that was dropped and the next largest component.
		// - Store the remaining components in 28bits using the ratio: 0.707106 : 0.577350 : 0.577350 -> 10 bits : 9 bits : 9 bits
		// - The final compressed format is:
		//      2bits: index of the dropped component
		//      2bits: index of the second largest component
		//      10bits: the compressed value of the second largest component
		//      9bits: the compressed value of the first component that isn't the dropped component or the second largest
		//      9bits: the compressed value of the second component that isn't the dropped component or the second largest
		// Approximate angular error ~0.27 degrees

		static constexpr uint32_t Mask1 = 0x3FF;       // = (1 << 10) - 1
		static constexpr uint32_t Mask2 = 0x1FF;       // = (1 <<  9) - 1
		static constexpr uint32_t Ofs1  = 0x1FF;
		static constexpr uint32_t Ofs2  = 0xFF;
		static constexpr float FScale1  = 723.3710f;   // = Scale1 / (2.0 * 0.707106)
		static constexpr float FScale2  = 442.5391f;   // = Scale2 / (2.0 * 0.577350)

		static uint32_t Compress(quat const& orientation)
		{
			quat ori = orientation;

			// Choose the largest component
			uint32_t largest1 = 0;
			if (Abs(ori[largest1]) < Abs(ori[1])) largest1 = 1;
			if (Abs(ori[largest1]) < Abs(ori[2])) largest1 = 2;
			if (Abs(ori[largest1]) < Abs(ori[3])) largest1 = 3;
			if (ori[largest1] < 0.0f) ori = -ori;    // Ensure the one we drop is positive
			ori[largest1] = 0.0f;

			// Choose the next largest component
			uint32_t largest2 = 0;
			if (Abs(ori[largest2]) < Abs(ori[1])) largest2 = 1;
			if (Abs(ori[largest2]) < Abs(ori[2])) largest2 = 2;
			if (Abs(ori[largest2]) < Abs(ori[3])) largest2 = 3;
			auto flargest2 = ori[largest2];
			ori[largest2] = 0.0f;

			// Compress the remaining three components
			auto the_big_one = static_cast<uint32_t>(static_cast<int>(flargest2 * FScale1) + Ofs1);
			assert((the_big_one & Mask1) == the_big_one);
			auto compressed_quat =
				(largest1    << 30) |   // index of the largest
				(largest2    << 28) |   // index of the second largest
				(the_big_one << 18);    // The compressed value of the second largest

			// Compress the remaining smaller two components
			int shift = 9;
			for (uint32_t i = 0; i != 4; ++i)
			{
				if (i == largest1 || i == largest2)
					continue;

				auto compressed_value = static_cast<uint32_t>((ori[i] * FScale2) + Ofs2);
				assert((compressed_value & Mask2) == compressed_value);
				compressed_quat |= compressed_value << shift;
				shift = 0;
			}
			return compressed_quat;
		}
		static quat DecompressQuat32(uint32_t compressed_orientation)
		{
			int largest1 = (compressed_orientation >> 30) & 0x3;
			int largest2 = (compressed_orientation >> 28) & 0x3;

			// Decompress the second largest component
			quat orientation;
			orientation[largest2] = (int((compressed_orientation >> 18) & Mask1) - Ofs1) / FScale1;

			// Accumulate the sum of squares
			auto sq_sum = orientation[largest2] * orientation[largest2];

			// Decompress the third and fourth components
			int shift = 9;
			for (int i = 0; i != 4; ++i)
			{
				if (i == largest1 || i == largest2)
					continue;

				orientation[i] = ((int(compressed_orientation >> shift) & Mask2) - Ofs2) / FScale2;
				shift = 0;

				sq_sum += orientation[i] * orientation[i];
			}

			// Construct the largest component from the unit length constraint
			orientation[largest1] = Sqrt(1.0f - sq_sum);
			return orientation;
		}
	};

	// Compress vectors with elements in the range [-1,+1]
	struct ComponentNorm
	{
		// Compress 'vec' into the lower 'max_bits' bits
		static uint64_t Compress2(v2 vec, int bits)
		{
			assert(Abs(vec.x) <= 1.0f && Abs(vec.y) <= 1.0f && "Only supports vectors with components in the range -1 to 1");

			// Divide bits evenly
			auto bx = bits / 2;
			auto by = bits - bx;

			auto x = static_cast<uint64_t>((vec.x + 1) * Scale(bx)) & Mask(bx);
			auto y = static_cast<uint64_t>((vec.y + 1) * Scale(by)) & Mask(by);
			return (x << by) | y;
		}
		static v2 Decompress2(uint64_t packed_vec, int bits)
		{
			// Divide bits evenly
			auto bx = bits / 2;
			auto by = bits - bx;

			v2 vec;
			vec.x = ((packed_vec >> by) & Mask(bx)) / Scale(bx) - 1.0f;
			vec.y = ((packed_vec >>  0) & Mask(by)) / Scale(by) - 1.0f;
			return vec;
		}

		// Compress 'vec' into the lower 'max_bits' bits
		static uint64_t Compress3(v4 vec, int bits)
		{
			assert(Abs(vec.x) <= 1.0f && Abs(vec.y) <= 1.0f && Abs(vec.z) <= 1.0f && "Only supports vectors with components in the range -1 to 1, and w as 0 or 1");
			assert((vec.w == 0.0f || vec.w == 1.0f) && "Only supports vectors with components in the range -1 to 1, and w as 0 or 1");

			// Use 1 bit for w and divide the remaining bits by 3
			auto bx = (bits - 1) / 3;
			auto by = (bits - 1 - bx) / 2;
			auto bz = (bits - 1 - bx - by);
			auto bw = 1;

			auto x = static_cast<uint64_t>((vec.x + 1.0f) * Scale(bx)) & Mask(bx);
			auto y = static_cast<uint64_t>((vec.y + 1.0f) * Scale(by)) & Mask(by);
			auto z = static_cast<uint64_t>((vec.z + 1.0f) * Scale(bz)) & Mask(bz);
			auto w = static_cast<uint64_t>(vec.w != 0.0f ? 1 : 0);
			return (x << (by+bz+bw)) | (y << (bz+bw)) | (z << (bw)) | (w);
		}
		static v4 Decompress3(uint64_t packed_vec, int bits)
		{
			// Use 1 bit for w and divide the remaining bits by 3
			auto bx = (bits - 1) / 3;
			auto by = (bits - 1 - bx) / 2;
			auto bz = (bits - 1 - bx - by);
			auto bw = 1;

			v4 vec;
			vec.x = ((packed_vec >> (by+bz+bw)) & Mask(bx)) / Scale(bx) - 1.0f;
			vec.y = ((packed_vec >> (bz+bw   )) & Mask(by)) / Scale(by) - 1.0f;
			vec.z = ((packed_vec >> (bw      )) & Mask(bz)) / Scale(bz) - 1.0f;
			vec.w = static_cast<float>(packed_vec & 0x1);
			return vec;
		}
		
		// Compress 'vec' into the lower 'max_bits' bits
		static uint64_t Compress4(v4 vec, int bits)
		{
			assert(Abs(vec.x) <= 1.0f && Abs(vec.y) <= 1.0f && Abs(vec.z) <= 1.0f && "Only supports vectors with components in the range -1 to 1, and w as 0 or 1");
			assert((vec.w == 0.0f || vec.w == 1.0f) && "Only supports vectors with components in the range -1 to 1, and w as 0 or 1");

			// Divide bits evenly
			auto bx = (bits) / 4;
			auto by = (bits - bx) / 3;
			auto bz = (bits - bx - by) / 2;
			auto bw = (bits - bx - by - bz);

			auto x = static_cast<uint64_t>((vec.x + 1.0f) * Scale(bx)) & Mask(bx);
			auto y = static_cast<uint64_t>((vec.y + 1.0f) * Scale(by)) & Mask(by);
			auto z = static_cast<uint64_t>((vec.z + 1.0f) * Scale(bz)) & Mask(bz);
			auto w = static_cast<uint64_t>((vec.w + 1.0f) * Scale(bw)) & Mask(bw);
			return (x << (by+bz+bw)) | (y << (bz+bw)) | (z << (bw)) | (w);
		}
		static v4 Decompress4(uint64_t packed_vec, int bits)
		{
			// Divide bits evenly
			auto bx = (bits) / 4;
			auto by = (bits - bx) / 3;
			auto bz = (bits - bx - by) / 2;
			auto bw = (bits - bx - by - bz);

			v4 vec;
			vec.x = ((packed_vec >> (by+bz+bw)) & Mask(bx)) / Scale(bx) - 1.0f;
			vec.y = ((packed_vec >> (bz+bw   )) & Mask(by)) / Scale(by) - 1.0f;
			vec.z = ((packed_vec >> (bw      )) & Mask(bz)) / Scale(bz) - 1.0f;
			vec.w = ((packed_vec >> (0       )) & Mask(bw)) / Scale(bw) - 1.0f;
			return vec;
		}

	private:
		static constexpr uint64_t Mask(int bits)
		{
			return (1ULL << bits) - 1;
		}
		static constexpr float Scale(int bits)
		{
			return (1ULL << bits) * 0.5f - 1.0f;
		}
	};
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::maths
{
	PRUnitTest(CompressionTests)
	{
		{ // Norm64bit
			float max_error = 0.0f;
			float const step = 0.05f;
			for (float z = -1.0f; z <= 1.0f; z += step)
			for (float y = -1.0f; y <= 1.0f; y += step)
			for (float x = -1.0f; x <= 1.0f; x += step)
			{
				auto in_ = v4::Normal(x, y, z, 0);
				auto enc = Norm64bit::Compress(in_);
				auto out_ = Norm64bit::Decompress(enc);
				max_error = std::max(Length(out_ - in_), max_error);
			}
			PR_EXPECT(max_error < 1e-6f);
		}
		{ // Norm32bit
			float max_error = 0.0f;
			float const step = 0.05f;
			for (float z = -1.0f; z <= 1.0f; z += step)
			for (float y = -1.0f; y <= 1.0f; y += step)
			for (float x = -1.0f; x <= 1.0f; x += step)
			{
				auto in_ = v4::Normal(x, y, z, 0);
				auto enc = Norm32bit::Compress(in_);
				auto out_ = Norm32bit::Decompress(enc);
				max_error = std::max(Length(out_ - in_), max_error);
				if (max_error > 0.01)
					max_error = max_error;
			}
			PR_EXPECT(max_error < 0.0002f); 
		}
		{ // Norm16bit
			float max_error = 0.0f;
			float const step = 0.05f;
			for (float z = -1.0f; z <= 1.0f; z += step)
			for (float y = -1.0f; y <= 1.0f; y += step)
			for (float x = -1.0f; x <= 1.0f; x += step)
			{
				auto in_ = v4::Normal(x, y, z, 0);
				auto enc = Norm16bit::Compress(in_);
				auto out_ = Norm16bit::Decompress(enc);
				max_error = std::max(Length(out_ - in_), max_error);
			}
			PR_EXPECT(max_error < 0.03f);
		}
		{ // Norm5bit
			float max_error = 0.0f;
			float const step = 0.05f;
			for (float z = -1.0f; z <= 1.0f; z += step)
			for (float y = -1.0f; y <= 1.0f; y += step)
			for (float x = -1.0f; x <= 1.0f; x += step)
			{
				v4   in_  = v4::Normal(x,y,z,0);
				auto enc  = Norm5bit::Compress(in_);
				v4   out_ = Norm5bit::Decompress(enc);
				max_error = std::max(Length(out_ - in_), max_error);
			}
			PR_EXPECT(max_error < 0.6f);
		}
	}
}
#endif
