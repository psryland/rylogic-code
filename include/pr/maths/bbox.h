//*****************************************************************************
// Maths library
//  Copyright (c) Rylogic Ltd 2002
//*****************************************************************************
#pragma once

#include "pr/maths/forward.h"
#include "pr/maths/constants.h"
#include "pr/maths/vector4.h"
#include "pr/maths/matrix3x4.h"
#include "pr/maths/matrix4x4.h"
#include "pr/maths/plane.h"
#include "pr/maths/bsphere.h"

namespace pr
{
	struct BBox
	{
		enum class EPlane
		{
			Lx = 0,
			Ux = 1,
			Ly = 2,
			Uy = 3,
			Lz = 4,
			Uz = 5,
			NumberOf = 6
		};

		v4 m_centre;
		v4 m_radius;

		BBox() = default;
		constexpr BBox(v4 const& centre, v4 const& radius)
			:m_centre(centre)
			,m_radius(radius)
		{
			// Catch invalid bbox radii (make an exception for the 'Reset' bbox)
			pr_assert("Invalid bounding box" && (
				(radius.x >= 0.0f && radius.y >= 0.0f && radius.z >= 0.0f) ||
				(radius.x == -1.f && radius.y == -1.f && radius.z == -1.f)));
		}

		// Reset this bbox to an invalid interval
		BBox& reset()
		{
			m_centre = v4Origin;
			m_radius = v4(-1.0f, -1.0f, -1.0f, 0.0f);
			return *this;
		}

		// Returns true if the bbox is valid
		bool valid() const
		{
			return m_radius.x >= 0 && m_radius.y >= 0 && m_radius.z >= 0 &&
				IsFinite(LengthSq(m_radius)) && IsFinite(LengthSq(m_centre));
		}

		// Returns true if this bbox encloses a single point
		bool is_point() const
		{
			return m_radius == v4Zero;
		}

		// Returns true if all of the radii are non zero
		bool has_volume() const
		{
			return m_radius.x != 0 && m_radius.y != 0 && m_radius.z != 0;
		}

		// Set this bbox to a unit cube centred on the origin
		BBox& unit()
		{
			m_centre = v4Origin;
			m_radius = v4(0.5f, 0.5f, 0.5f, 0.0f);
			return *this;
		}

		// The centre of the bbox
		v4 Centre() const
		{
			// This method exists for uniformity with BSphere
			return m_centre;
		}

		// The radius on each axis of the bbox
		v4 RadiusSq() const
		{
			return Sqr(Radius());
		}
		v4 Radius() const
		{
			return m_radius;
		}

		// Diagonal size of the bbox
		float DiametreSq() const
		{
			return 4.0f * LengthSq(m_radius);
		}
		float Diametre() const
		{
			return Sqrt(DiametreSq());
		}

		// Return the lower corner (-x,-y,-z) of the bounding box
		float LowerX() const
		{
			return m_centre.x - m_radius.x;
		}
		float LowerY() const
		{
			return m_centre.y - m_radius.y;
		}
		float LowerZ() const
		{
			return m_centre.z - m_radius.z;
		}
		float Lower(int axis) const
		{
			return m_centre[axis] - m_radius[axis];
		}
		v4 Lower() const
		{
			return m_centre - m_radius;
		}

		// Return the upper corner (+x,+y,+z) of the bounding box
		float UpperX() const
		{
			return m_centre.x + m_radius.x;
		}
		float UpperY() const
		{
			return m_centre.y + m_radius.y;
		}
		float UpperZ() const
		{
			return m_centre.z + m_radius.z;
		}
		float Upper(int axis) const
		{
			return m_centre[axis] + m_radius[axis];
		}
		v4 Upper() const
		{
			return m_centre + m_radius;
		}

		// Return the size of the bounding box
		float SizeX() const
		{
			return 2.0f * m_radius.x;
		}
		float SizeY() const
		{
			return 2.0f * m_radius.y;
		}
		float SizeZ() const
		{
			return 2.0f * m_radius.z;
		}
		float Size(int axis) const
		{
			return 2.0f * m_radius[axis];
		}

		// Grows the bbox to include 'rhs'. Note: prefer the free function versions.
		// There are two variations of 'Encompass':
		//   1) Grow = modifies the provided instance returning the point enclosed,
		//   2) Union = operates on a const BBox returning a new BBox that includes 'point'
		v4 Grow(v4 point)
		{
			pr_assert("BBox Grow. Point must have w = 1" && point.w == 1.0f);
			pr_assert("'point' must be aligned to 16" && is_aligned<v4>(&point));

			#if PR_MATHS_USE_INTRINSICS
			__m128 const zero = _mm_set_ps1(+0.0f);
			__m128 const half = _mm_set_ps1(+0.5f);
			auto init = _mm_cmplt_ps(m_radius.vec, zero);                               // init = radius == -1 ? FFFF... : 0000... 
			auto lwr = _mm_sub_ps(m_centre.vec, m_radius.vec);                          // lwr  = centre - radius
			auto upr = _mm_add_ps(m_centre.vec, m_radius.vec);                          // upr  = centre + radius
			auto init_pt = _mm_and_ps(init, point.vec);                                 // init_pt = init & point
			lwr = _mm_or_ps(init_pt , _mm_andnot_ps(init, _mm_min_ps(lwr, point.vec))); // lwr  = init_pt | (~init & min(lwr, point))
			upr = _mm_or_ps(init_pt , _mm_andnot_ps(init, _mm_max_ps(upr, point.vec))); // upr  = init_pt | (~init & max(upr, point))
			m_centre.vec = _mm_mul_ps(_mm_add_ps(upr,lwr), half);                       // center = (upr + lwr) / 2;
			m_radius.vec = _mm_mul_ps(_mm_sub_ps(upr,lwr), half);                       // radius = (upr - lwr) / 2;
			#else
			for (int i = 0; i != 3; ++i)
			{
				if (m_radius[i] < 0.0f)
				{
					m_centre[i] = point[i];
					m_radius[i] = 0.0f;
				}
				else
				{
					float signed_dist = point[i] - m_centre[i];
					float length      = Abs(signed_dist);
					if (length > m_radius[i])
					{
						float new_radius = (length + m_radius[i]) / 2.0f;
						m_centre[i] += signed_dist * (new_radius - m_radius[i]) / length;
						m_radius[i] = new_radius;
					}
				}
			}
			#endif
			return point;
		}

		#pragma region Operators
		friend bool pr_vectorcall operator == (BBox lhs, BBox rhs)  { return memcmp(&lhs, &rhs, sizeof(lhs)) == 0; }
		friend bool pr_vectorcall operator != (BBox lhs, BBox rhs)  { return memcmp(&lhs, &rhs, sizeof(lhs)) != 0; }
		friend bool pr_vectorcall operator <  (BBox lhs, BBox rhs)  { return memcmp(&lhs, &rhs, sizeof(lhs)) <  0; }
		friend bool pr_vectorcall operator >  (BBox lhs, BBox rhs)  { return memcmp(&lhs, &rhs, sizeof(lhs)) >  0; }
		friend bool pr_vectorcall operator <= (BBox lhs, BBox rhs)  { return memcmp(&lhs, &rhs, sizeof(lhs)) <= 0; }
		friend bool pr_vectorcall operator >= (BBox lhs, BBox rhs)  { return memcmp(&lhs, &rhs, sizeof(lhs)) >= 0; }
		friend BBox& pr_vectorcall operator += (BBox& lhs, v4 offset)
		{
			lhs.m_centre += offset;
			return lhs;
		}
		friend BBox& pr_vectorcall operator -= (BBox& lhs, v4 offset)
		{
			lhs.m_centre -= offset;
			return lhs;
		}
		friend BBox& pr_vectorcall operator *= (BBox& lhs, float s)
		{
			lhs.m_radius *= s;
			return lhs;
		}
		friend BBox& pr_vectorcall operator /= (BBox& lhs, float s)
		{
			lhs *= (1.0f / s);
			return lhs;
		}
		friend BBox pr_vectorcall operator + (BBox lhs, v4 offset)
		{
			auto bb = lhs;
			return bb += offset;
		}
		friend BBox pr_vectorcall operator - (BBox lhs, v4 offset)
		{
			auto bb = lhs;
			return bb -= offset;
		}
		friend BBox pr_vectorcall operator * (m4x4 const& m, BBox rhs)
		{
			pr_assert("m4x4 * BBox: Transform is not affine" && IsAffine(m));
			pr_assert("Transforming an invalid bounding box" && rhs.valid());

			BBox bb(m.pos, v4Zero);
			m4x4 mat = Transpose3x3(m);
			for (int i = 0; i != 3; ++i)
			{
				bb.m_centre[i] += Dot4(    mat[i] , rhs.m_centre);
				bb.m_radius[i] += Dot4(Abs(mat[i]), rhs.m_radius);
			}
			return bb;
		}
		friend BBox pr_vectorcall operator * (m3x4 const& m, BBox rhs)
		{
			pr_assert("Transforming an invalid bounding box" && rhs.valid());

			BBox bb(v4Origin, v4Zero);
			auto mat = Transpose(m);
			for (int i = 0; i != 3; ++i)
			{
				bb.m_centre[i] += Dot4(    mat[i] , rhs.m_centre);
				bb.m_radius[i] += Dot4(Abs(mat[i]), rhs.m_radius);
			}
			return bb;
		}
		#pragma endregion

		// Constants
		static constexpr BBox Unit()
		{
			return { v4::Origin(), 0.5f * v4::One().w0() };
		}
		static constexpr BBox Reset()
		{
			return { v4::Origin(), -v4::One().w0() };
		}
		static constexpr BBox Infinity()
		{
			return { v4::Origin(), v4::Infinity().w0() };
		}

		// Create a bounding box from lower/upper corners
		static BBox Make(v4 const& lower, v4 const& upper)
		{
			return BBox((upper + lower) * 0.5f, (upper - lower) * 0.5f);
		}

		// Create a bounding box from a collection of verts
		template <typename VertCont> static BBox Make(VertCont const& verts)
		{
			auto bbox = Reset();
			for (auto& vert : verts) bbox.Grow(vert);
			return bbox;
		}

		// Create a bounding box from a list of verts
		template <typename Vert> static BBox Make(std::initializer_list<Vert> verts)
		{
			auto bbox = Reset();
			for (auto& vert : verts) pr::Grow(bbox, vert);
			return bbox;
		}
	};
	static_assert(std::is_trivially_copyable_v<BBox>, "Should be a pod type");
	static_assert(std::alignment_of_v<BBox> == 16, "Should be 16 byte aligned");

	#pragma region Constants
	[[deprecated]] static BBox const BBoxUnit  = BBox::Unit();
	[[deprecated]] static BBox const BBoxReset = BBox::Reset();
	#pragma endregion

	#pragma region Functions

	// Return a corner of the bounding box
	inline v4 pr_vectorcall Corner(BBox bbox, int corner)
	{
		pr_assert("Invalid corner index" && corner >= 0 && corner < 8);
		auto x = ((corner >> 0) & 0x1) * 2 - 1;
		auto y = ((corner >> 1) & 0x1) * 2 - 1;
		auto z = ((corner >> 2) & 0x1) * 2 - 1;
		return v4(
			bbox.m_centre.x + x * bbox.m_radius.x,
			bbox.m_centre.y + y * bbox.m_radius.y,
			bbox.m_centre.z + z * bbox.m_radius.z,
			1.0f);
	}

	// Return the corners of the bounding box
	inline std::array<v4,8> pr_vectorcall Corners(BBox bbox)
	{
		std::array<v4, 8> corners;
		auto& c = bbox.m_centre;
		auto& r = bbox.m_radius;
		corners[0] = v4{c.x - r.x, c.y - r.y, c.z - r.z, 1};
		corners[1] = v4{c.x + r.x, c.y - r.y, c.z - r.z, 1};
		corners[2] = v4{c.x - r.x, c.y + r.y, c.z - r.z, 1};
		corners[3] = v4{c.x + r.x, c.y + r.y, c.z - r.z, 1};
		corners[4] = v4{c.x - r.x, c.y - r.y, c.z + r.z, 1};
		corners[5] = v4{c.x + r.x, c.y - r.y, c.z + r.z, 1};
		corners[6] = v4{c.x - r.x, c.y + r.y, c.z + r.z, 1};
		corners[7] = v4{c.x + r.x, c.y + r.y, c.z + r.z, 1};
		return corners;
	}

	// Return the volume of a bounding box
	inline float pr_vectorcall Volume(BBox bbox)
	{
		return bbox.SizeX() * bbox.SizeY() * bbox.SizeZ();
	}

	// Returns the most extreme point in the direction of 'separating_axis'
	inline v4 pr_vectorcall SupportPoint(BBox bbox, v4 separating_axis)
	{
		return bbox.m_centre + Sign(separating_axis, false) * bbox.m_radius;
	}

	// Return a plane corresponding to a side of the bounding box. Returns inward facing planes
	inline Plane pr_vectorcall GetPlane(BBox bbox, BBox::EPlane side)
	{
		switch (side)
		{
		default: pr_assert(false && "Unknown side index"); return Plane();
		case BBox::EPlane::Lx: return plane::make( 1.0f,  0.0f,  0.0f, bbox.m_centre.x + bbox.m_radius.x);
		case BBox::EPlane::Ux: return plane::make(-1.0f,  0.0f,  0.0f, bbox.m_centre.x + bbox.m_radius.x);
		case BBox::EPlane::Ly: return plane::make( 0.0f,  1.0f,  0.0f, bbox.m_centre.y + bbox.m_radius.y);
		case BBox::EPlane::Uy: return plane::make( 0.0f, -1.0f,  0.0f, bbox.m_centre.y + bbox.m_radius.y);
		case BBox::EPlane::Lz: return plane::make( 0.0f,  0.0f,  1.0f, bbox.m_centre.z + bbox.m_radius.z);
		case BBox::EPlane::Uz: return plane::make( 0.0f,  0.0f, -1.0f, bbox.m_centre.z + bbox.m_radius.z);
		}
	}

	// Return a bounding sphere that bounds the bounding box
	inline BSphere pr_vectorcall GetBSphere(BBox bbox)
	{
		return BSphere(bbox.m_centre, Length(bbox.m_radius));
	}

	// Multiply the bounding box by a non-affine transform
	inline BBox pr_vectorcall MulNonAffine(m4x4 const& m, BBox rhs)
	{
		pr_assert("Transforming an invalid bounding box" && rhs.valid());

		auto bb = BBox::Reset();
		for (auto& c : Corners(rhs))
		{
			auto cnr = m * c;
			bb.Grow(cnr / cnr.w);
		}
		return bb;
	}

	// Include 'point' within 'bbox'.
	[[nodiscard]]
	inline BBox pr_vectorcall Union(BBox bbox, v4 point)
	{
		// Const version returns lhs, non-const returns rhs!
		BBox bb = bbox;
		bb.Grow(point);
		return bb;
	}
	inline v4 pr_vectorcall Grow(BBox& bbox, v4 point)
	{
		// Const version returns lhs, non-const returns rhs!
		return bbox.Grow(point);
	}

	// Include 'rhs' in 'lhs'
	[[nodiscard]]
	inline BBox pr_vectorcall Union(BBox lhs, BBox rhs)
	{
		// Const version returns lhs, non-const returns rhs!
		// Don't treat !rhs.valid() as an error, it's the only way to grow an empty bbox
		BBox bb = lhs;
		if (!rhs.valid()) return bb;
		bb.Grow(rhs.m_centre + rhs.m_radius);
		bb.Grow(rhs.m_centre - rhs.m_radius);
		return bb;
	}
	inline BBox pr_vectorcall Grow(BBox& lhs, BBox rhs)
	{
		// Const version returns lhs, non-const returns rhs!
		// Don't treat !rhs.valid() as an error, it's the only way to grow an empty bbox
		if (!rhs.valid()) return rhs;
		lhs.Grow(rhs.m_centre + rhs.m_radius);
		lhs.Grow(rhs.m_centre - rhs.m_radius);
		return rhs;
	}

	// Include 'rhs' in 'lhs'
	[[nodiscard]]
	inline BBox Union(BBox lhs, BSphere rhs)
	{
		// Don't treat rhs.empty() as an error, it's the only way to grow an empty bsphere
		BBox bb = lhs;
		if (!rhs.valid()) return bb;
		auto radius = v4(rhs.Radius(), rhs.Radius(), rhs.Radius(), 0);
		bb.Grow(rhs.Centre() + radius);
		bb.Grow(rhs.Centre() - radius);
		return bb;
	}
	inline BSphere Grow(BBox& lhs, BSphere rhs)
	{
		// Don't treat rhs.empty() as an error, it's the only way to grow an empty bsphere
		if (!rhs.valid()) return rhs;
		auto radius = v4(rhs.Radius(), rhs.Radius(), rhs.Radius(), 0);
		lhs.Grow(rhs.Centre() + radius);
		lhs.Grow(rhs.Centre() - radius);
		return rhs;
	}

	// Returns true if 'point' is within the bounding volume
	inline bool pr_vectorcall IsWithin(BBox bbox, v4 point, float tol = 0)
	{
		return
			Abs(point.x - bbox.m_centre.x) <= bbox.m_radius.x + tol &&
			Abs(point.y - bbox.m_centre.y) <= bbox.m_radius.y + tol &&
			Abs(point.z - bbox.m_centre.z) <= bbox.m_radius.z + tol;
	}

	// Returns true if 'test' is within the bounding volume
	inline bool pr_vectorcall IsWithin(BBox bbox, BBox test)
	{
		return
			Abs(test.m_centre.x - bbox.m_centre.x) <= (bbox.m_radius.x - test.m_radius.x) &&
			Abs(test.m_centre.y - bbox.m_centre.y) <= (bbox.m_radius.y - test.m_radius.y) &&
			Abs(test.m_centre.z - bbox.m_centre.z) <= (bbox.m_radius.z - test.m_radius.z);
	}

	#pragma endregion
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::maths
{
	PRUnitTest(BoundingBoxTests)
	{
		v4 pt[] =
		{
			{+1,+1,+1,1},
			{-1,+0,+1,1},
			{+1,+1,+1,1},
			{+0,-2,-1,1},
		};
		auto bbox = BBox::Reset();
		for (auto& p : pt) Grow(bbox, p);
		PR_EXPECT(bbox.Lower().x == -1.0f);
		PR_EXPECT(bbox.Lower().y == -2.0f);
		PR_EXPECT(bbox.Lower().z == -1.0f);
		PR_EXPECT(bbox.Lower().w == +1.0f);
		PR_EXPECT(bbox.Upper().x == +1.0f);
		PR_EXPECT(bbox.Upper().y == +1.0f);
		PR_EXPECT(bbox.Upper().z == +1.0f);
		PR_EXPECT(bbox.Upper().w == +1.0f);
	}
}
#endif