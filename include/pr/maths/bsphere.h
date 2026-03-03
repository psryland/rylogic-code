//*****************************************************************************
// Maths library
//  Copyright (c) Rylogic Ltd 2002
//*****************************************************************************
#pragma once

#include "pr/maths/forward.h"
#include "pr/maths/constants.h"
#include "pr/maths/vector4.h"
#include "pr/maths/matrix4x4.h"
#include "pr/maths/constants_vector.h"

namespace pr
{
	struct alignas(16) BSphere
	{
		v4 m_ctr_rad; // x,y,z = position, 'w' = radius

		BSphere() = default;
		constexpr BSphere(v4 const& centre, float radius)
			:m_ctr_rad(centre.xyz, radius)
		{}

		// Reset this bsphere to invalid
		BSphere& reset()
		{
			m_ctr_rad = -v4Origin;
			return *this;
		}

		// True if the bsphere is valid
		bool valid() const
		{
			return m_ctr_rad.w >= 0 && IsFinite(m_ctr_rad.w);
		}

		// Returns true if this bsphere does not bound anything
		bool is_point() const
		{
			return m_ctr_rad.w == 0;
		}

		// Set this bsphere to a unit sphere centred on the origin
		BSphere& unit()
		{
			m_ctr_rad = v4Origin;
			return *this;
		}

		// The centre of the bsphere
		v4 Centre() const
		{
			return m_ctr_rad.w1();
		}

		// The radius of the bsphere
		float RadiusSq() const
		{
			return Sqr(Radius());
		}
		float Radius() const
		{
			return m_ctr_rad.w;
		}

		// The diameter of the bsphere
		float DiametreSq() const
		{
			return Sqr(Diametre());
		}
		float Diametre() const
		{
			return 2.0f * m_ctr_rad.w;
		}

		// Include 'rhs' in 'lhs' and re-centre the centre point. Returns 'rhs'
		v4 pr_vectorcall Grow(v4 rhs)
		{
			if (Radius() < 0.0f)
			{
				// Centre on this point, since it's the first
				m_ctr_rad = v4{rhs.x, rhs.y, rhs.z, 0.0f};
			}
			else
			{
				// Only grow if outside the current bounds
				auto len_sq = LengthSq(rhs - Centre());
				if (len_sq > RadiusSq())
				{
					// Move the centre and increase the radius by the minimum
					// amount to include the existing bsphere and 'rhs'
					auto separation = Sqrt(len_sq);
					auto new_radius = (separation + Radius()) * 0.5f;
					m_ctr_rad += (rhs - Centre()) * ((new_radius - Radius()) / separation);
					m_ctr_rad.w = new_radius;
				}
			}
			return rhs;
		}
		BSphere pr_vectorcall Grow(BSphere rhs)
		{
			if (Radius() < 0.0f)
			{
				// If this is the first thing, just adopt 'rhs'
				m_ctr_rad = rhs.m_ctr_rad;
			}
			else
			{
				// Only grow if 'rhs' extends beyond the current radius
				auto separation = Length(rhs.Centre() - Centre());
				if (separation + rhs.Radius() > Radius())
				{
					// Move the centre and increase the radius by the minimum
					// amount to include the existing bsphere and 'rhs'
					auto new_radius = (separation + Radius() + rhs.Radius()) * 0.5f;
					m_ctr_rad += (rhs.Centre() - Centre()) * ((new_radius - Radius()) / separation);
					m_ctr_rad.w = new_radius;
				}
			}
			return rhs;
		}

		// Include 'rhs' within 'bsphere' without moving the centre point.
		v4 pr_vectorcall GrowLoose(v4 rhs)
		{
			if (m_ctr_rad.w < 0.0f)
			{
				m_ctr_rad = v4{rhs.x, rhs.y, rhs.z, 0.0f};
			}
			else
			{
				auto len_sq = LengthSq(rhs - Centre());
				if (len_sq > RadiusSq())
					m_ctr_rad.w = Sqrt(len_sq);
			}
			return rhs;
		}
		BSphere pr_vectorcall GrowLoose(BSphere rhs)
		{
			if (Radius() < 0.0f)
			{
				m_ctr_rad = rhs.m_ctr_rad;
			}
			else
			{
				auto new_radius = Length(rhs.Centre() - Centre()) + rhs.Radius();
				if (new_radius > Radius())
					m_ctr_rad.w = new_radius;
			}
			return rhs;
		}

		#pragma region Operators
		friend bool operator == (BSphere const& lhs, BSphere const& rhs)  { return memcmp(&lhs, &rhs, sizeof(lhs)) == 0; }
		friend bool operator != (BSphere const& lhs, BSphere const& rhs)  { return memcmp(&lhs, &rhs, sizeof(lhs)) != 0; }
		friend bool operator <  (BSphere const& lhs, BSphere const& rhs)  { return memcmp(&lhs, &rhs, sizeof(lhs)) <  0; }
		friend bool operator >  (BSphere const& lhs, BSphere const& rhs)  { return memcmp(&lhs, &rhs, sizeof(lhs)) >  0; }
		friend bool operator <= (BSphere const& lhs, BSphere const& rhs)  { return memcmp(&lhs, &rhs, sizeof(lhs)) <= 0; }
		friend bool operator >= (BSphere const& lhs, BSphere const& rhs)  { return memcmp(&lhs, &rhs, sizeof(lhs)) >= 0; }
		friend BSphere& pr_vectorcall operator += (BSphere& lhs, v4 offset)
		{
			pr_assert(offset.w == 0.0f);
			lhs.m_ctr_rad += offset;
			return lhs;
		}
		friend BSphere& pr_vectorcall operator -= (BSphere& lhs, v4 offset)
		{
			pr_assert(offset.w == 0.0f);
			lhs.m_ctr_rad -= offset;
			return lhs;
		}
		friend BSphere& pr_vectorcall operator *= (BSphere& lhs, float s)
		{
			lhs.m_ctr_rad.w *= s;
			return lhs;
		}
		friend BSphere& pr_vectorcall operator /= (BSphere& lhs, float s)
		{
			lhs.m_ctr_rad.w /= s;
			return lhs;
		}
		friend BSphere pr_vectorcall operator + (BSphere const& bsph, v4 offset)
		{
			auto bs = bsph;
			return bs += offset;
		}
		friend BSphere pr_vectorcall operator - (BSphere const& bsph, v4 offset)
		{
			auto bs = bsph;
			return bs -= offset;
		}
		friend BSphere pr_vectorcall operator * (BSphere const& bsph, float s)
		{
			auto bs = bsph;
			return bs *= s;
		}
		friend BSphere pr_vectorcall operator * (float s, BSphere const& bsph)
		{
			auto bs = bsph;
			return bs *= s;
		}
		friend BSphere pr_vectorcall operator * (m4x4 const& m, BSphere const& bsph)
		{
			return BSphere(m * bsph.Centre(), bsph.m_ctr_rad.w);
		}
		#pragma endregion

		// Constants
		static constexpr BSphere Zero()
		{
			return { v4::Zero(), 0.0f };
		}
		static constexpr BSphere Unit()
		{
			return { v4::Origin(), 1.0f };
		}
		static constexpr BSphere Reset()
		{
			return { v4::Origin(), -1.0f };
		}
	};
	static_assert(std::is_trivially_copyable_v<BSphere>, "Should be a pod type");
	static_assert(std::alignment_of_v<BSphere> == 16, "Should be 16 byte aligned");

	#pragma region Functions

	// The volume of the bsphere
	inline float Volume(BSphere const& bsph)
	{
		return 4.188790f * bsph.m_ctr_rad.w * bsph.m_ctr_rad.w * bsph.m_ctr_rad.w; // (2/3)*tau*r^3
	}

	// Returns the most extreme point in the direction of 'separating_axis'
	inline v4 pr_vectorcall SupportPoint(BSphere bsphere, v4 separating_axis)
	{
		return bsphere.m_ctr_rad.w1() + bsphere.m_ctr_rad.w * separating_axis;
	}

	// Include 'point' within 'bsphere' and re-centre the centre point.
	[[nodiscard]]
	inline BSphere pr_vectorcall Union(BSphere bsphere, v4 point)
	{
		auto bsph = bsphere;
		bsph.Grow(point);
		return bsph;
	}
	inline v4 pr_vectorcall Grow(BSphere& bsphere, v4 point)
	{
		return bsphere.Grow(point);
	}

	// Include 'rhs' in 'lhs' 
	[[nodiscard]]
	inline BSphere pr_vectorcall Union(BSphere lhs, BSphere rhs)
	{
		auto bsph = lhs;
		bsph.Grow(rhs);
		return bsph;
	}
	inline BSphere pr_vectorcall Grow(BSphere& lhs, BSphere rhs)
	{
		return lhs.Grow(rhs);
	}

	// Include 'point' within 'bsphere' without moving the centre point
	[[nodiscard]] 
	inline BSphere pr_vectorcall UnionLoose(BSphere bsphere, v4 point)
	{
		auto bsph = bsphere;
		bsph.GrowLoose(point);
		return bsph;
	}
	inline v4 pr_vectorcall GrowLoose(BSphere& bsphere, v4 point)
	{
		return bsphere.GrowLoose(point);
	}

	// Include 'rhs' in 'lhs' without moving the centre point
	[[nodiscard]] 
	inline BSphere pr_vectorcall UnionLoose(BSphere lhs, BSphere rhs)
	{
		auto bsph = lhs;
		bsph.GrowLoose(rhs);
		return bsph;
	}
	inline BSphere pr_vectorcall GrowLoose(BSphere& lhs, BSphere rhs)
	{
		return lhs.GrowLoose(rhs);
	}

	// Return true if 'point' is within the bounding sphere
	inline bool pr_vectorcall IsWithin(BSphere bsphere, v4 point, float tol = 0)
	{
		return LengthSq(point - bsphere.Centre()) <= bsphere.RadiusSq() + tol;
	}
	inline bool pr_vectorcall IsWithin(BSphere bsphere, BSphere test, float tol = 0)
	{
		return LengthSq(test.Centre() - bsphere.Centre()) <= Sqr(bsphere.Radius() - test.Radius() + tol);
	}

	// Returns true if 'lhs' and 'rhs' intersect
	inline bool pr_vectorcall IsIntersection(BSphere lhs, BSphere rhs)
	{
		return LengthSq(rhs.Centre() - lhs.Centre()) < Sqr(lhs.Radius() + rhs.Radius());
	}

	#pragma endregion
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::maths
{
	PRUnitTest(BoundingSphereTests)
	{
		v4 pt[] =
		{
			{0, 0, 0, 1},
			{1, 1, 1, 1},
			{0, 0, 1, 1},
			{0, 1, 0, 1},
			{0, 1, 1, 1},
			{1, 0, 0, 1},
			{1, 0, 1, 1},
			{1, 1, 0, 1},
		};
		auto bsph = BSphere::Reset();
		for (auto& p : pt)
			Grow(bsph, p);

		PR_EXPECT(FEql(bsph.Centre(), v4(0.5f, 0.5f, 0.5f, 1)));
		PR_EXPECT(FEql(bsph.Radius(), 0.8660254f));
	}
}
#endif