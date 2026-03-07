//*****************************************************************************
// Maths library
//  Copyright (c) Rylogic Ltd 2002
//*****************************************************************************
#pragma once
#include "pr/math/core/forward.h"
#include "pr/math/types/vector4.h"
#include "pr/math/types/matrix3x4.h"
#include "pr/math/types/matrix4x4.h"
#include "pr/math/primitives/bsphere.h"
#include "pr/math/primitives/plane.h"

namespace pr::math
{
	template <ScalarType S>
	struct BoundingBox
	{
		using Vec4 = Vec4<S>;

		Vec4 m_centre;
		Vec4 m_radius;

		BoundingBox() = default;
		constexpr BoundingBox(Vec4 centre, Vec4 radius) noexcept
			: m_centre(centre)
			, m_radius(radius)
		{
			// Catch invalid BoundingBox radii (make an exception for the 'Reset' BoundingBox)
			pr_assert("Invalid bounding box" && (
				(radius.x >= +S(0) && radius.y >= +S(0) && radius.z >= +S(0)) ||
				(radius.x == -S(1) && radius.y == -S(1) && radius.z == -S(1))));
		}

		// Constants
		static constexpr BoundingBox const& Unit() noexcept
		{
			static auto s_unit = BoundingBox{ math::Origin<Vec4>(), S(0.5) * math::One<Vec4>().w0() };
			return s_unit;
		}
		static constexpr BoundingBox const& Reset() noexcept
		{
			static auto s_reset = BoundingBox{ math::Origin<Vec4>(), -math::One<Vec4>().w0() };
			return s_reset;
		}
		static constexpr BoundingBox const& Infinity() noexcept
		{
			static auto s_infinity = BoundingBox{ math::Origin<Vec4>(), math::Infinity<Vec4>().w0() };
			return s_infinity;
		}

		// Reset this BoundingBox to an invalid interval
		BoundingBox& reset() noexcept
		{
			m_centre = Origin<Vec4>();
			m_radius = Vec4{ S(-1), S(-1), S(-1), S(0) };
			return *this;
		}

		// Returns true if the BoundingBox is valid
		constexpr bool valid() const noexcept
		{
			// Note: An infinite bounding box should still be valid
			return m_radius.x >= 0 && m_radius.y >= 0 && m_radius.z >= 0 && IsFinite(m_centre);
		}

		// Returns true if this BoundingBox encloses a single point
		constexpr bool is_point() const noexcept
		{
			return m_radius == Zero<Vec4>();
		}

		// Returns true if all of the radii are non zero
		constexpr bool has_volume() const noexcept
		{
			return m_radius.x != 0 && m_radius.y != 0 && m_radius.z != 0;
		}

		// Set this BoundingBox to a unit cube centred on the origin
		BoundingBox& unit() noexcept
		{
			m_centre = Origin<Vec4>();
			m_radius = Vec4{ S(0.5), S(0.5), S(0.5), S(0) };
			return *this;
		}

		// The centre of the BoundingBox
		constexpr Vec4 Centre() const noexcept
		{
			// This method exists for uniformity with BSphere
			return m_centre;
		}

		// The radius on each axis of the BoundingBox
		constexpr Vec4 RadiusSq() const noexcept
		{
			return Sqr(Radius());
		}
		constexpr Vec4 Radius() const noexcept
		{
			return m_radius;
		}

		// Diagonal size of the BoundingBox
		constexpr S DiametreSq() const noexcept
		{
			return S(4) * LengthSq(m_radius);
		}
		double Diametre() const noexcept
		{
			return Sqrt(static_cast<double>(DiametreSq()));
		}

		// Return the lower corner (-x,-y,-z) of the bounding box
		constexpr S LowerX() const noexcept
		{
			return m_centre.x - m_radius.x;
		}
		constexpr S LowerY() const noexcept
		{
			return m_centre.y - m_radius.y;
		}
		constexpr S LowerZ() const noexcept
		{
			return m_centre.z - m_radius.z;
		}
		constexpr S Lower(int axis) const noexcept
		{
			return m_centre[axis] - m_radius[axis];
		}
		constexpr Vec4 Lower() const noexcept
		{
			return m_centre - m_radius;
		}

		// Return the upper corner (+x,+y,+z) of the bounding box
		constexpr S UpperX() const noexcept
		{
			return m_centre.x + m_radius.x;
		}
		constexpr S UpperY() const noexcept
		{
			return m_centre.y + m_radius.y;
		}
		constexpr S UpperZ() const noexcept
		{
			return m_centre.z + m_radius.z;
		}
		constexpr S Upper(int axis) const noexcept
		{
			return m_centre[axis] + m_radius[axis];
		}
		constexpr Vec4 Upper() const noexcept
		{
			return m_centre + m_radius;
		}

		// Return the size of the bounding box
		constexpr S SizeX() const noexcept
		{
			return S(2) * m_radius.x;
		}
		constexpr S SizeY() const noexcept
		{
			return S(2) * m_radius.y;
		}
		constexpr S SizeZ() const noexcept
		{
			return S(2) * m_radius.z;
		}
		constexpr S Size(int axis) const noexcept
		{
			return S(2) * m_radius[axis];
		}

		// Grows the BoundingBox to include 'rhs'. Note: prefer the free function versions.
		// There are two variations of 'Encompass':
		//   1) Grow = modifies the provided instance returning the point enclosed,
		//   2) Union = operates on a const BoundingBox returning a new BoundingBox that includes 'point'
		constexpr Vec4 Grow(Vec4 point) noexcept
		{
			pr_assert("BoundingBox Grow. Point must have w = 1" && point.w == 1.0f);

			auto fallback = [&]() constexpr
			{
				for (int i = 0; i != 3; ++i)
				{
					if (m_radius[i] < 0)
					{
						m_centre[i] = point[i];
						m_radius[i] = 0;
					}
					else
					{
						auto signed_dist = point[i] - m_centre[i];
						auto length = Abs(signed_dist);
						if (length > m_radius[i])
						{
							auto new_radius = (length + m_radius[i]) / 2;
							m_centre[i] += signed_dist * (new_radius - m_radius[i]) / length;
							m_radius[i] = new_radius;
						}
					}
				}
			};
			if consteval
			{
				fallback();
			}
			else
			{
				if constexpr (Vec4::IntrinsicF && std::same_as<S, float>)
				{
					__m128 const zero = _mm_set_ps1(+0.0f);
					__m128 const half = _mm_set_ps1(+0.5f);
					auto init = _mm_cmplt_ps(m_radius.vec, zero);                               // init = radius == -1 ? FFFF... : 0000... 
					auto lwr = _mm_sub_ps(m_centre.vec, m_radius.vec);                          // lwr  = centre - radius
					auto upr = _mm_add_ps(m_centre.vec, m_radius.vec);                          // upr  = centre + radius
					auto init_pt = _mm_and_ps(init, point.vec);                                 // init_pt = init & point
					lwr = _mm_or_ps(init_pt, _mm_andnot_ps(init, _mm_min_ps(lwr, point.vec))); // lwr  = init_pt | (~init & min(lwr, point))
					upr = _mm_or_ps(init_pt, _mm_andnot_ps(init, _mm_max_ps(upr, point.vec))); // upr  = init_pt | (~init & max(upr, point))
					m_centre.vec = _mm_mul_ps(_mm_add_ps(upr, lwr), half);                       // center = (upr + lwr) / 2;
					m_radius.vec = _mm_mul_ps(_mm_sub_ps(upr, lwr), half);                       // radius = (upr - lwr) / 2;
				}
				else
				{
					fallback();
				}
			}
			return point;
		}

		#pragma region Operators
		friend bool pr_vectorcall operator == (BoundingBox lhs, BoundingBox rhs)  { return memcmp(&lhs, &rhs, sizeof(lhs)) == 0; }
		friend bool pr_vectorcall operator != (BoundingBox lhs, BoundingBox rhs)  { return memcmp(&lhs, &rhs, sizeof(lhs)) != 0; }
		friend bool pr_vectorcall operator <  (BoundingBox lhs, BoundingBox rhs)  { return memcmp(&lhs, &rhs, sizeof(lhs)) <  0; }
		friend bool pr_vectorcall operator >  (BoundingBox lhs, BoundingBox rhs)  { return memcmp(&lhs, &rhs, sizeof(lhs)) >  0; }
		friend bool pr_vectorcall operator <= (BoundingBox lhs, BoundingBox rhs)  { return memcmp(&lhs, &rhs, sizeof(lhs)) <= 0; }
		friend bool pr_vectorcall operator >= (BoundingBox lhs, BoundingBox rhs)  { return memcmp(&lhs, &rhs, sizeof(lhs)) >= 0; }
		friend BoundingBox& pr_vectorcall operator += (BoundingBox& lhs, Vec4 offset) noexcept
		{
			lhs.m_centre += offset;
			return lhs;
		}
		friend BoundingBox& pr_vectorcall operator -= (BoundingBox& lhs, Vec4 offset) noexcept
		{
			lhs.m_centre -= offset;
			return lhs;
		}
		friend BoundingBox& pr_vectorcall operator *= (BoundingBox& lhs, S s) noexcept
		{
			lhs.m_radius *= s;
			return lhs;
		}
		friend BoundingBox& pr_vectorcall operator /= (BoundingBox& lhs, S s) noexcept
		{
			lhs *= (1.0f / s);
			return lhs;
		}
		friend BoundingBox pr_vectorcall operator + (BoundingBox lhs, Vec4 offset) noexcept
		{
			auto bb = lhs;
			return bb += offset;
		}
		friend BoundingBox pr_vectorcall operator - (BoundingBox lhs, Vec4 offset) noexcept
		{
			auto bb = lhs;
			return bb -= offset;
		}
		friend BoundingBox pr_vectorcall operator * (Mat4x4<S> const& m, BoundingBox rhs) noexcept
		{
			pr_assert("m4x4 * BoundingBox: Transform is not affine" && IsAffine(m));
			pr_assert("Transforming an invalid bounding box" && rhs.valid());

			BoundingBox bb(m.pos, Zero<Vec4>());
			auto mat = Transpose3x3(m);
			for (int i = 0; i != 3; ++i)
			{
				bb.m_centre[i] += Dot(    mat[i] , rhs.m_centre);
				bb.m_radius[i] += Dot(Abs(mat[i]), rhs.m_radius);
			}
			return bb;
		}
		friend BoundingBox pr_vectorcall operator * (Mat3x4<S> const& m, BoundingBox rhs) noexcept
		{
			pr_assert("Transforming an invalid bounding box" && rhs.valid());

			BoundingBox bb(Origin<Vec4>(), Zero<Vec4>());
			auto mat = Transpose(m);
			for (int i = 0; i != 3; ++i)
			{
				bb.m_centre[i] += Dot(    mat[i] , rhs.m_centre);
				bb.m_radius[i] += Dot(Abs(mat[i]), rhs.m_radius);
			}
			return bb;
		}
		#pragma endregion

		// Create a bounding box from lower/upper corners
		static BoundingBox Make(Vec4 lower, Vec4 upper) noexcept
		{
			return BoundingBox((upper + lower) * 0.5f, (upper - lower) * 0.5f);
		}

		// Create a bounding box from a collection of points
		static BoundingBox Make(std::ranges::input_range auto&& points) noexcept
		{
			auto bbox = Reset();
			for (auto& point : points) bbox.Grow(point);
			return bbox;
		}
	};
	static_assert(std::is_trivially_copyable_v<BoundingBox<float>>, "Should be a pod type");
	static_assert(std::alignment_of_v<BoundingBox<float>> == 16, "Should be 16 byte aligned");

	#pragma region Functions

	// Faces of the bounding box
	enum class EBoundingBoxPlane
	{
		Lx = 0,
		Ux = 1,
		Ly = 2,
		Uy = 3,
		Lz = 4,
		Uz = 5,
		NumberOf = 6
	};

	// Return a corner of the bounding box
	template <ScalarType S> constexpr Vec4<S> pr_vectorcall Corner(BoundingBox<S> BoundingBox, int corner) noexcept
	{
		pr_assert("Invalid corner index" && corner >= 0 && corner < 8);
		auto x = ((corner >> 0) & 0x1) * 2 - 1;
		auto y = ((corner >> 1) & 0x1) * 2 - 1;
		auto z = ((corner >> 2) & 0x1) * 2 - 1;
		return Vec4<S>(
			BoundingBox.m_centre.x + x * BoundingBox.m_radius.x,
			BoundingBox.m_centre.y + y * BoundingBox.m_radius.y,
			BoundingBox.m_centre.z + z * BoundingBox.m_radius.z,
			S(1));
	}

	// Return the corners of the bounding box
	template <ScalarType S> constexpr std::array<Vec4<S>,8> pr_vectorcall Corners(BoundingBox<S> bbox) noexcept
	{
		auto& c = bbox.m_centre;
		auto& r = bbox.m_radius;
		std::array<Vec4<S>, 8> corners;
		corners[0] = Vec4<S>{c.x - r.x, c.y - r.y, c.z - r.z, 1};
		corners[1] = Vec4<S>{c.x + r.x, c.y - r.y, c.z - r.z, 1};
		corners[2] = Vec4<S>{c.x - r.x, c.y + r.y, c.z - r.z, 1};
		corners[3] = Vec4<S>{c.x + r.x, c.y + r.y, c.z - r.z, 1};
		corners[4] = Vec4<S>{c.x - r.x, c.y - r.y, c.z + r.z, 1};
		corners[5] = Vec4<S>{c.x + r.x, c.y - r.y, c.z + r.z, 1};
		corners[6] = Vec4<S>{c.x - r.x, c.y + r.y, c.z + r.z, 1};
		corners[7] = Vec4<S>{c.x + r.x, c.y + r.y, c.z + r.z, 1};
		return corners;
	}

	// Return the volume of a bounding box
	template <ScalarType S> constexpr S pr_vectorcall Volume(BoundingBox<S> bbox) noexcept
	{
		return bbox.SizeX() * bbox.SizeY() * bbox.SizeZ();
	}

	// Include 'point' within 'bbox'.
	template <ScalarType S> [[nodiscard]] constexpr BoundingBox<S> pr_vectorcall Union(BoundingBox<S> bbox, Vec4<S> point) noexcept
	{
		// Const version returns lhs, non-const returns rhs!
		auto bb = bbox;
		bb.Grow(point);
		return bb;
	}
	template <ScalarType S> constexpr Vec4<S> pr_vectorcall Grow(BoundingBox<S>& bbox, Vec4<S> point) noexcept
	{
		return bbox.Grow(point);
	}

	// Include 'rhs' in 'lhs'
	template <ScalarType S> [[nodiscard]] constexpr BoundingBox<S> pr_vectorcall Union(BoundingBox<S> lhs, BoundingBox<S> rhs) noexcept
	{
		// Const version returns lhs, non-const returns rhs!
		// Don't treat !rhs.valid() as an error, it's the only way to grow an empty BoundingBox
		auto bb = lhs;
		if (!rhs.valid()) return bb;
		bb.Grow(rhs.m_centre + rhs.m_radius);
		bb.Grow(rhs.m_centre - rhs.m_radius);
		return bb;
	}
	template <ScalarType S> constexpr BoundingBox<S> pr_vectorcall Grow(BoundingBox<S>& lhs, BoundingBox<S> rhs) noexcept
	{
		// Const version returns lhs, non-const returns rhs!
		// Don't treat !rhs.valid() as an error, it's the only way to grow an empty BoundingBox
		if (!rhs.valid()) return rhs;
		lhs.Grow(rhs.m_centre + rhs.m_radius);
		lhs.Grow(rhs.m_centre - rhs.m_radius);
		return rhs;
	}

	// Returns the most extreme point in the direction of 'separating_axis'
	template <ScalarType S> constexpr Vec4<S> pr_vectorcall SupportPoint(BoundingBox<S> bbox, Vec4<S> separating_axis) noexcept
	{
		return bbox.m_centre + Sign(separating_axis, false) * bbox.m_radius;
	}

	// Return the planes of 'BoundingBox'. Returns inward facing planes
	template <ScalarType S> constexpr std::array<Plane3<S>,6> pr_vectorcall ToPlanes(BoundingBox<S> bbox) noexcept
	{
		return std::array<Plane3<S>, EBoundingBoxPlane::NumberOf> {
			Plane3<S>(+S(1), +S(0), +S(0), bbox.m_centre.x + bbox.m_radius.x),
			Plane3<S>(-S(1), +S(0), +S(0), bbox.m_centre.x + bbox.m_radius.x),
			Plane3<S>(+S(0), +S(1), +S(0), bbox.m_centre.y + bbox.m_radius.y),
			Plane3<S>(+S(0), -S(1), +S(0), bbox.m_centre.y + bbox.m_radius.y),
			Plane3<S>(+S(0), +S(0), +S(1), bbox.m_centre.z + bbox.m_radius.z),
			Plane3<S>(+S(0), +S(0), -S(1), bbox.m_centre.z + bbox.m_radius.z),
		};
	}

	// Return a plane corresponding to a side of the bounding box. Returns inward facing planes
	template <ScalarType S> constexpr Plane3<S> pr_vectorcall GetPlane(BoundingBox<S> bbox, EBoundingBoxPlane side) noexcept
	{
		return ToPlanes(bbox)[static_cast<int>(side)];
	}

	// Return a bounding sphere that bounds the bounding box
	template <ScalarType S> [[nodiscard]] BoundingSphere<S> pr_vectorcall GetBSphere(BoundingBox<S> bbox) noexcept
	{
		return BoundingSphere<S>(bbox.m_centre, Length(bbox.m_radius));
	}

	// Include 'rhs' in 'lhs'
	template <ScalarType S> [[nodiscard]] constexpr BoundingBox<S> Union(BoundingBox<S> lhs, BoundingSphere<S> rhs) noexcept
	{
		// Don't treat rhs.empty() as an error, it's the only way to grow an empty bsphere
		auto bb = lhs;
		if (!rhs.valid()) return bb;
		auto radius = Vec4<S>(rhs.Radius(), rhs.Radius(), rhs.Radius(), 0);
		bb.Grow(rhs.Centre() + radius);
		bb.Grow(rhs.Centre() - radius);
		return bb;
	}
	template <ScalarType S> constexpr BoundingSphere<S> Grow(BoundingBox<S>& lhs, BoundingSphere<S> rhs) noexcept
	{
		// Don't treat rhs.empty() as an error, it's the only way to grow an empty bsphere
		if (!rhs.valid()) return rhs;
		auto radius = Vec4<S>(rhs.Radius(), rhs.Radius(), rhs.Radius(), 0);
		lhs.Grow(rhs.Centre() + radius);
		lhs.Grow(rhs.Centre() - radius);
		return rhs;
	}

	// Returns true if 'point' is within the bounding volume
	template <ScalarType S> constexpr bool pr_vectorcall IsWithin(BoundingBox<S> bbox, Vec4<S> point, S tol = 0) noexcept
	{
		return
			Abs(point.x - bbox.m_centre.x) <= bbox.m_radius.x + tol &&
			Abs(point.y - bbox.m_centre.y) <= bbox.m_radius.y + tol &&
			Abs(point.z - bbox.m_centre.z) <= bbox.m_radius.z + tol;
	}

	// Returns true if 'test' is within the bounding volume
	template <ScalarType S> constexpr bool pr_vectorcall IsWithin(BoundingBox<S> bbox, BoundingSphere<S> test) noexcept
	{
		return
			Abs(test.m_centre.x - bbox.m_centre.x) <= (bbox.m_radius.x - test.m_radius.x) &&
			Abs(test.m_centre.y - bbox.m_centre.y) <= (bbox.m_radius.y - test.m_radius.y) &&
			Abs(test.m_centre.z - bbox.m_centre.z) <= (bbox.m_radius.z - test.m_radius.z);
	}

	// Returns true if 'bb' is within the bounding volume
	template <ScalarType S> constexpr bool pr_vectorcall IsWithin(BoundingBox<S> bb, BoundingBox<S> test, S tol = 0) noexcept
	{
		// True if 'test' is entirely within 'bb'
		return
			Abs(bb.m_centre.x - test.m_centre.x) <= (bb.m_radius.x - test.m_radius.x) + tol &&
			Abs(bb.m_centre.y - test.m_centre.y) <= (bb.m_radius.y - test.m_radius.y) + tol &&
			Abs(bb.m_centre.z - test.m_centre.z) <= (bb.m_radius.z - test.m_radius.z) + tol;
	}

	// Multiply the bounding box by a non-affine transform
	template <ScalarType S> constexpr BoundingBox<S> pr_vectorcall MulNonAffine(Mat4x4<S> const& m, BoundingBox<S> rhs) noexcept
	{
		pr_assert("Transforming an invalid bounding box" && rhs.valid());

		auto bb = BoundingBox<S>::Reset();
		for (auto& c : Corners(rhs))
		{
			auto cnr = m * c;
			bb.Grow(cnr / cnr.w);
		}
		return bb;
	}
	#pragma endregion
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::math
{
	PRUnitTestClass(BoundingBoxTests)
	{
		PRUnitTestMethod(Grow)
		{
			Vec4<float> pt[] =
			{
				{+1,+1,+1,1},
				{-1,+0,+1,1},
				{+1,+1,+1,1},
				{+0,-2,-1,1},
			};
	
			auto bbox = BoundingBox<float>::Reset();
			for (auto& p : pt)
				Grow(bbox, p);

			PR_EXPECT(bbox.Lower().x == -1.0f);
			PR_EXPECT(bbox.Lower().y == -2.0f);
			PR_EXPECT(bbox.Lower().z == -1.0f);
			PR_EXPECT(bbox.Lower().w == +1.0f);
			PR_EXPECT(bbox.Upper().x == +1.0f);
			PR_EXPECT(bbox.Upper().y == +1.0f);
			PR_EXPECT(bbox.Upper().z == +1.0f);
			PR_EXPECT(bbox.Upper().w == +1.0f);
		}
	};
}
#endif
