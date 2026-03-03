//*****************************************************************************
// Maths library
//  Copyright (c) Rylogic Ltd 2002
//*****************************************************************************
#pragma once
#include "pr/math_new/core/forward.h"
#include "pr/math_new/core/constants.h"
#include "pr/math_new/types/vector4.h"
#include "pr/math_new/types/matrix4x4.h"

namespace pr::math
{
	template <ScalarType S>
	struct BSphere
	{
		using Vec3 = Vec3<S>;
		using Vec4 = Vec4<S>;

		Vec4 m_ctr_rad; // x,y,z = position, 'w' = radius

		BSphere() = default;
		constexpr BSphere(Vec3 centre, S radius) noexcept
			:m_ctr_rad(centre, radius)
		{}
		constexpr BSphere(Vec4 centre, S radius) noexcept
			:BSphere(centre.xyz, radius)
		{}

		// Constants
		static constexpr BSphere const& Reset() noexcept
		{
			static auto s_reset = BSphere{ Vec3{0, 0, 0}, -1 };
			return s_reset;
		}
		static constexpr BSphere const& Unit() noexcept
		{
			static auto s_unit = BSphere{ Vec3{0, 0, 0}, 1 };
			return s_unit;
		}

		// Reset this bsphere to invalid
		BSphere& reset() noexcept
		{
			m_ctr_rad = Vec4{ 0,0,0,-1 };
			return *this;
		}

		// True if the bsphere is valid
		constexpr bool valid() const noexcept
		{
			return m_ctr_rad.w >= 0 && IsFinite(m_ctr_rad.w);
		}

		// Returns true if this bsphere does not bound anything
		constexpr bool is_point() const noexcept
		{
			return m_ctr_rad.w == 0;
		}

		// Set this bsphere to a unit sphere centred on the origin
		BSphere& unit() noexcept
		{
			m_ctr_rad = Vec4{ 0,0,0,1 };
			return *this;
		}

		// The centre of the bsphere
		constexpr Vec4 Centre() const noexcept
		{
			return m_ctr_rad.w1();
		}

		// The radius of the bsphere
		constexpr S RadiusSq() const noexcept
		{
			return Sqr(Radius());
		}
		constexpr S Radius() const noexcept
		{
			return m_ctr_rad.w;
		}

		// The diameter of the bsphere
		constexpr S DiametreSq() const noexcept
		{
			return Sqr(Diametre());
		}
		constexpr S Diametre() const noexcept
		{
			return S(2) * m_ctr_rad.w;
		}

		// Include 'rhs' in 'lhs' and re-centre the centre point. Returns 'rhs' for chaining
		Vec4 pr_vectorcall Grow(Vec4 rhs) noexcept
		{
			if (Radius() < 0)
			{
				// Centre on this point, since it's the first
				m_ctr_rad = Vec4{rhs.x, rhs.y, rhs.z, 0};
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
					auto new_radius = (separation + Radius()) / S(2);
					m_ctr_rad += (rhs - Centre()) * ((new_radius - Radius()) / separation);
					m_ctr_rad.w = new_radius;
				}
			}
			return rhs;
		}
		BSphere pr_vectorcall Grow(BSphere rhs) noexcept
		{
			if (Radius() < 0)
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
					auto new_radius = (separation + Radius() + rhs.Radius()) / S(2);
					m_ctr_rad += (rhs.Centre() - Centre()) * ((new_radius - Radius()) / separation);
					m_ctr_rad.w = new_radius;
				}
			}
			return rhs;
		}

		// Include 'rhs' within 'bsphere' without moving the centre point.
		Vec4 pr_vectorcall GrowLoose(Vec4 rhs) noexcept
		{
			if (m_ctr_rad.w < 0)
			{
				m_ctr_rad = Vec4{rhs.x, rhs.y, rhs.z, 0};
			}
			else
			{
				auto len_sq = LengthSq(rhs - Centre());
				if (len_sq > RadiusSq())
					m_ctr_rad.w = Sqrt(len_sq);
			}
			return rhs;
		}
		BSphere pr_vectorcall GrowLoose(BSphere rhs) noexcept
		{
			if (Radius() < 0)
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
		friend bool operator == (BSphere lhs, BSphere rhs) { return memcmp(&lhs, &rhs, sizeof(lhs)) == 0; }
		friend bool operator != (BSphere lhs, BSphere rhs) { return memcmp(&lhs, &rhs, sizeof(lhs)) != 0; }
		friend bool operator <  (BSphere lhs, BSphere rhs) { return memcmp(&lhs, &rhs, sizeof(lhs)) <  0; }
		friend bool operator >  (BSphere lhs, BSphere rhs) { return memcmp(&lhs, &rhs, sizeof(lhs)) >  0; }
		friend bool operator <= (BSphere lhs, BSphere rhs) { return memcmp(&lhs, &rhs, sizeof(lhs)) <= 0; }
		friend bool operator >= (BSphere lhs, BSphere rhs) { return memcmp(&lhs, &rhs, sizeof(lhs)) >= 0; }
		friend BSphere& pr_vectorcall operator += (BSphere& lhs, Vec4 offset) noexcept
		{
			pr_assert(offset.w == 0.0f);
			lhs.m_ctr_rad += offset;
			return lhs;
		}
		friend BSphere& pr_vectorcall operator -= (BSphere& lhs, Vec4 offset) noexcept
		{
			pr_assert(offset.w == 0.0f);
			lhs.m_ctr_rad -= offset;
			return lhs;
		}
		friend BSphere& pr_vectorcall operator *= (BSphere& lhs, S s) noexcept
		{
			lhs.m_ctr_rad.w *= s;
			return lhs;
		}
		friend BSphere& pr_vectorcall operator /= (BSphere& lhs, S s) noexcept
		{
			lhs.m_ctr_rad.w /= s;
			return lhs;
		}
		friend BSphere pr_vectorcall operator + (BSphere bsph, Vec4 offset) noexcept
		{
			auto bs = bsph;
			return bs += offset;
		}
		friend BSphere pr_vectorcall operator - (BSphere bsph, Vec4 offset) noexcept
		{
			auto bs = bsph;
			return bs -= offset;
		}
		friend BSphere pr_vectorcall operator * (BSphere bsph, S s) noexcept
		{
			auto bs = bsph;
			return bs *= s;
		}
		friend BSphere pr_vectorcall operator * (S s, BSphere bsph) noexcept
		{
			auto bs = bsph;
			return bs *= s;
		}
		friend BSphere pr_vectorcall operator * (Mat4x4<S> const& m, BSphere bsph) noexcept
		{
			return BSphere(m * bsph.Centre(), bsph.m_ctr_rad.w);
		}
		#pragma endregion
	};
	static_assert(std::is_trivially_copyable_v<BSphere<float>>, "Should be a pod type");
	static_assert(std::alignment_of_v<BSphere<float>> == 16, "Should be 16 byte aligned");

	#pragma region Functions

	// The volume of the bsphere
	template <ScalarType S> constexpr S Volume(BSphere<S> bsph) noexcept
	{
		return S(4.188790) * bsph.m_ctr_rad.w * bsph.m_ctr_rad.w * bsph.m_ctr_rad.w; // (2/3)*tau*r^3
	}

	// Returns the most extreme point in the direction of 'separating_axis'
	template <ScalarType S> constexpr Vec4<S> pr_vectorcall SupportPoint(BSphere<S> bsphere, Vec4<S> separating_axis) noexcept
	{
		return bsphere.m_ctr_rad.w1() + bsphere.m_ctr_rad.w * separating_axis;
	}

	// Include 'point' within 'bsphere' and re-centre the centre point.
	template <ScalarType S> [[nodiscard]] constexpr BSphere<S> pr_vectorcall Union(BSphere<S> bsphere, Vec4<S> point) noexcept
	{
		auto bsph = bsphere;
		bsph.Grow(point);
		return bsph;
	}
	template <ScalarType S> constexpr Vec4<S> pr_vectorcall Grow(BSphere<S>& bsphere, Vec4<S> point) noexcept
	{
		return bsphere.Grow(point);
	}

	// Include 'rhs' in 'lhs' 
	template <ScalarType S> [[nodiscard]] constexpr BSphere<S> pr_vectorcall Union(BSphere<S> lhs, BSphere<S> rhs) noexcept
	{
		auto bsph = lhs;
		bsph.Grow(rhs);
		return bsph;
	}
	template <ScalarType S> constexpr BSphere<S> pr_vectorcall Grow(BSphere<S>& lhs, BSphere<S> rhs) noexcept
	{
		return lhs.Grow(rhs);
	}

	// Include 'point' within 'bsphere' without moving the centre point
	template <ScalarType S> [[nodiscard]] constexpr BSphere<S> pr_vectorcall UnionLoose(BSphere<S> bsphere, Vec4<S> point) noexcept
	{
		auto bsph = bsphere;
		bsph.GrowLoose(point);
		return bsph;
	}
	template <ScalarType S> constexpr Vec4<S> pr_vectorcall GrowLoose(BSphere<S>& bsphere, Vec4<S> point) noexcept
	{
		return bsphere.GrowLoose(point);
	}

	// Include 'rhs' in 'lhs' without moving the centre point
	template <ScalarType S> [[nodiscard]] constexpr BSphere<S> pr_vectorcall UnionLoose(BSphere<S> lhs, BSphere<S> rhs) noexcept
	{
		auto bsph = lhs;
		bsph.GrowLoose(rhs);
		return bsph;
	}
	template <ScalarType S> constexpr BSphere<S> pr_vectorcall GrowLoose(BSphere<S>& lhs, BSphere<S> rhs) noexcept
	{
		return lhs.GrowLoose(rhs);
	}

	// Return true if 'point' is within the bounding sphere
	template <ScalarType S> [[nodiscard]] constexpr bool pr_vectorcall IsWithin(BSphere<S> bsphere, Vec4<S> point, S tol = 0) noexcept
	{
		return LengthSq(point - bsphere.Centre()) <= bsphere.RadiusSq() + tol;
	}
	template <ScalarType S> [[nodiscard]] constexpr bool pr_vectorcall IsWithin(BSphere<S> bsphere, BSphere<S> test, S tol = 0) noexcept
	{
		return LengthSq(test.Centre() - bsphere.Centre()) <= Sqr(bsphere.Radius() - test.Radius() + tol);
	}

	// Returns true if 'lhs' and 'rhs' intersect
	template <ScalarType S> [[nodiscard]] constexpr bool pr_vectorcall IsIntersection(BSphere<S> lhs, BSphere<S> rhs) noexcept
	{
		return LengthSq(rhs.Centre() - lhs.Centre()) < Sqr(lhs.Radius() + rhs.Radius());
	}

	#pragma endregion
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::math
{
	PRUnitTestClass(BoundingSphereTests)
	{
		PRUnitTestMethod(Grow, float)
		{
			Vec4<float> pt[] =
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
			auto bsph = BSphere<float>::Reset();
			for (auto& p : pt)
				Grow(bsph, p);

			PR_EXPECT(FEql(bsph.Centre(), Vec4<float>(0.5f, 0.5f, 0.5f, 1)));
			PR_EXPECT(FEql(bsph.Radius(), 0.8660254f));
		}
	};
}
#endif
