//*****************************************************************************
// Maths library
//  Copyright (c) Rylogic Ltd 2002
//*****************************************************************************
#pragma once
#include "pr/math/core/forward.h"
#include "pr/math/types/vector2.h"

namespace pr::math
{
	template <ScalarType S>
	struct Rectangle
	{
		using Vec2 = Vec2<S>;

		Vec2 m_min;
		Vec2 m_max;

		Rectangle() = default;
		constexpr Rectangle(Vec2 min, Vec2 max) noexcept
			:m_min(min)
			,m_max(max)
		{}
		constexpr Rectangle(S xmin, S ymin, S xmax, S ymax) noexcept
			:m_min(xmin, ymin)
			,m_max(xmax, ymax)
		{}
		template <ScalarType V> explicit constexpr Rectangle(Rectangle<V> rhs) noexcept
			:m_min(Vec2(rhs.m_min))
			,m_max(Vec2(rhs.m_max))
		{}

		// Constants
		static constexpr Rectangle<S> const& Zero() noexcept
		{
			static auto s_zero = Rectangle<S>{ math::Zero<Vec2>(), math::Zero<Vec2>() };
			return s_zero;
		}
		static constexpr Rectangle<S> const& Reset() noexcept
		{
			static auto s_reset = Rectangle<S>{ math::Max<Vec2>(), -math::Max<Vec2>() };
			return s_reset;
		}
		static constexpr Rectangle<S> const& Unit() noexcept
		{
			static auto s_unit = Rectangle<S>{ math::Zero<Vec2>(), math::One<Vec2>() };
			return s_unit;
		}

		// Reset this rectangle to an invalid interval
		constexpr Rectangle& reset() noexcept
		{
			m_min = +math::Max<Vec2>();
			m_max = -math::Max<Vec2>();
			return *this;
		}

		// Returns true if this bbox does not bound anything
		constexpr bool empty() const noexcept
		{
			return
				m_min.x > m_max.x ||
				m_min.y > m_max.y;
		}

		// The width and height of the rectangle
		constexpr Vec2 Size() const noexcept
		{
			return m_max - m_min;
		}

		// Get/Set the width of the rectangle. 'anchor' : -1 = anchor the left, 0 = anchor centre, 1 = anchor right
		constexpr S SizeX() const noexcept
		{
			return m_max.x - m_min.x;
		}
		constexpr void SizeX(S sz, int anchor) noexcept
		{
			switch (anchor)
			{
				case -1:
				{
					m_max = Vec2(m_min.x + sz, m_max.y);
					break;
				}
				case  0:
				{
					auto w0 = sz / S(2);
					auto w1 = sz - w0;
					auto c = (m_min.x + m_max.x) / S(2);
					m_min = Vec2(c - w0, m_min.y);
					m_max = Vec2(c + w1, m_max.y);
					break;
				}
				case +1:
				{
					m_min = Vec2(m_max.x - sz, m_min.y);
					break;
				}
			}
		}

		// Get/Set the height of the rectangle. 'anchor' : -1 = anchor the top, 0 = anchor centre, 1 = anchor bottom
		constexpr S SizeY() const noexcept
		{
			return m_max.y - m_min.y;
		}
		constexpr void SizeY(S sz, int anchor) noexcept
		{
			switch (anchor)
			{
				case -1:
				{
					m_max = Vec2(m_max.x, m_min.y + sz);
					break;
				}
				case  0:
				{
					auto h0 = sz / S(2);
					auto h1 = sz - h0;
					auto c = (m_min.y + m_max.y) / S(2);
					m_min = Vec2(m_min.x, c - h0);
					m_max = Vec2(m_max.x, c + h1);
					break;
				}
				case +1:
				{
					m_min = Vec2(m_min.x, m_max.y - sz);
					break;
				}
			}
		}

		// The left edge position (x value)
		constexpr S Left() const noexcept
		{
			return m_min.x;
		}

		// The top edge position (y value)
		constexpr S Top() const noexcept
		{
			return m_min.y;
		}

		// The right edge position (x value)
		constexpr S Right() const noexcept
		{
			return m_max.x;
		}

		// The bottom edge position (y value)
		constexpr S Bottom() const noexcept
		{
			return m_max.y;
		}

		// The centre position of the rectangle
		constexpr Vec2 Centre() const noexcept
		{
			return (m_min + m_max) / S(2);
		}

		// The diagonal length of the rectangle
		constexpr S DiametreSq() const noexcept
		{
			return LengthSq(m_max - m_min);
		}
		S Diametre() const noexcept
		{
			return Sqrt(DiametreSq());
		}

		// The area of the rectangle
		constexpr S Area() const noexcept
		{
			return SizeX() * SizeY();
		}

		// The aspect ratio (width/height)
		constexpr double Aspect() const noexcept
		{
			return static_cast<double>(SizeX()) / SizeY();
		}

		#pragma region Operators
		friend bool operator == (Rectangle lhs, Rectangle rhs) { return memcmp(&lhs, &rhs, sizeof(lhs)) == 0; }
		friend bool operator != (Rectangle lhs, Rectangle rhs) { return memcmp(&lhs, &rhs, sizeof(lhs)) != 0; }
		friend bool operator <  (Rectangle lhs, Rectangle rhs) { return memcmp(&lhs, &rhs, sizeof(lhs)) <  0; }
		friend bool operator >  (Rectangle lhs, Rectangle rhs) { return memcmp(&lhs, &rhs, sizeof(lhs)) >  0; }
		friend bool operator <= (Rectangle lhs, Rectangle rhs) { return memcmp(&lhs, &rhs, sizeof(lhs)) <= 0; }
		friend bool operator >= (Rectangle lhs, Rectangle rhs) { return memcmp(&lhs, &rhs, sizeof(lhs)) >= 0; }
		friend Rectangle& operator += (Rectangle& lhs, Vec2 offset) noexcept
		{
			lhs.m_min += offset;
			lhs.m_max += offset;
			return lhs;
		}
		friend Rectangle& operator -= (Rectangle& lhs, Vec2 offset) noexcept
		{
			lhs.m_min -= offset;
			lhs.m_max -= offset;
			return lhs;
		}
		friend Rectangle operator + (Rectangle lhs, Vec2 offset) noexcept
		{
			auto f = lhs;
			return f += offset;
		}
		friend Rectangle operator - (Rectangle lhs, Vec2 offset) noexcept
		{
			auto f = lhs;
			return f -= offset;
		}
		friend bool FEql(Rectangle lhs, Rectangle rhs) noexcept
		{
			return
				FEql(lhs.m_min, rhs.m_min) &&
				FEql(lhs.m_max, rhs.m_max);
		}
		#pragma endregion
	};

	using FRect = Rectangle<float>;
	using IRect = Rectangle<int>;

	#pragma region Functions

	// Returns 'rect' offset by 'dx,dy'
	template <ScalarType S> constexpr Rectangle<S> Shifted(Rectangle<S> rect, S dx, S dy) noexcept
	{
		return {
			rect.m_min.x + dx, rect.m_min.y + dy,
			rect.m_max.x + dx, rect.m_max.y + dy
		};
	}

	// Returns 'rect' inflated by the given values. Positive values increase the rect size, negative values decrease it
	template <ScalarType S> constexpr Rectangle<S> Inflated(Rectangle<S> rect, S dxmin, S dymin, S dxmax, S dymax) noexcept
	{
		return {
			rect.m_min.x - dxmin, rect.m_min.y - dymin,
			rect.m_max.x + dxmax, rect.m_max.y + dymax
		};
	}
	template <ScalarType S> constexpr Rectangle<S> Inflated(Rectangle<S> rect, S dx, S dy) noexcept
	{
		return Inflated(rect, dx, dy, dx, dy);
	}
	template <ScalarType S> constexpr Rectangle<S> Inflated(Rectangle<S> rect, S by) noexcept
	{
		return Inflated(rect, by, by);
	}

	// Returns 'rect' inflated by the given values, scaled by the current width/height of 'rect'
	template <ScalarType S> constexpr Rectangle<S> Scale(Rectangle<S> rect, S xmin, S ymin, S xmax, S ymax) noexcept
	{
		auto sx = rect.SizeX() / S(2);
		auto sy = rect.SizeY() / S(2);
		return Inflated(rect, sx * xmin, sy * ymin, sx * xmax, sy * ymax);
	}
	template <ScalarType S> constexpr Rectangle<S> Scale(Rectangle<S> rect, S dx, S dy) noexcept
	{
		return Scale(rect, dx, dy, dx, dy);
	}
	template <ScalarType S> constexpr Rectangle<S> Scale(Rectangle<S> rect, S by) noexcept
	{
		return Scale(rect, by, by);
	}

	// Include 'point' in 'frect'
	template <ScalarType S, VectorTypeN<2> Vec> constexpr Rectangle<S>& Grow(Rectangle<S>& rect, Vec point) noexcept
	{
		auto p = Vec2<S>{vec(point).x, vec(point).y};
		rect.m_min = Min(p, rect.m_min);
		rect.m_max = Max(p, rect.m_max);
		return rect;
	}
	template <ScalarType S, VectorTypeN<2> Vec> [[nodiscard]] constexpr Rectangle<S> Union(Rectangle<S> rect, Vec point) noexcept
	{
		auto r = rect;
		return Grow(r, point);
	}

	// Include 'rhs' in 'lhs'
	template <ScalarType S> constexpr Rectangle<S>& Grow(Rectangle<S>& lhs, Rectangle<S> rhs) noexcept
	{
		lhs.m_min = Min(lhs.m_min, rhs.m_min);
		lhs.m_max = Max(lhs.m_max, rhs.m_max);
		return lhs;
	}
	template <ScalarType S> [[nodiscard]] inline Rectangle<S> Union(Rectangle<S> lhs, Rectangle<S> rhs) noexcept
	{
		auto r = lhs;
		return Grow(r, rhs);
	}

	// Returns true if 'point' is within the bounding volume
	template <ScalarType S, VectorTypeN<2> Vec> inline bool IsWithin(Rectangle<S> rect, Vec point) noexcept
	{
		return
			vec(point).x >= rect.m_min.x && vec(point).x < rect.m_max.x &&
			vec(point).y >= rect.m_min.y && vec(point).y < rect.m_max.y;
	}

	// Returns true if 'lhs' and 'rhs' intersect
	template <ScalarType S> constexpr bool IsIntersection(Rectangle<S> lhs, Rectangle<S> rhs) noexcept
	{
		return
			!(lhs.m_max.x < rhs.m_min.x || lhs.m_min.x > rhs.m_max.x ||
			  lhs.m_max.y < rhs.m_min.y || lhs.m_min.y > rhs.m_max.y);
	}

	// Return 'point' scaled by the transform that maps 'rect' to the square (bottom left:-1,-1)->(top right:1,1) 
	// 'xsign' should be -1 if the rect origin is on the right, +1 if on the left
	// 'ysign' should be -1 if the rect origin is at the top, +1 if at the bottom
	// Inverse of 'ScalePoint'
	template <ScalarType S, VectorTypeN<2> Vec> inline Vec NormalisePoint(Rectangle<S> rect, Vec point, S xsign, S ysign) noexcept
	{
		return Vec{
			xsign * (S(2) * (vec(point).x - rect.m_min.x) / rect.SizeX() - S(1)),
			ysign * (S(2) * (vec(point).y - rect.m_min.y) / rect.SizeY() - S(1))
		};
	}

	// Scales a normalised 'point' by the transform that maps the square (bottom left:-1,-1)->(top right:1,1) to 'rect'
	// 'xsign' should be -1 if the rect origin is on the right, +1 if on the left
	// 'ysign' should be -1 if the rect origin is at the top, +1 if at the bottom
	// Inverse of 'NormalisedPoint'
	template <ScalarType S, VectorTypeN<2> Vec> inline Vec ScalePoint(Rectangle<S> rect, Vec point, S xsign, S ysign) noexcept
	{
		return Vec{
			rect.m_min.x + rect.SizeX() * (S(1) + xsign * vec(point).x) / S(2),
			rect.m_min.y + rect.SizeY() * (S(1) + ysign * vec(point).y) / S(2)
		};
	}

	#pragma endregion
}