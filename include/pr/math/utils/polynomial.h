//*****************************************************************************
// Maths library
//  Copyright (c) Rylogic Ltd 2002
//*****************************************************************************
#pragma once
#include "pr/math/core/forward.h"
#include "pr/math/core/traits.h"
#include "pr/math/core/functions.h"
#include "pr/math/types/matrix3x4.h"
#include "pr/math/types/matrix.h"

namespace pr::math
{
	enum { MaxRoots = 4 };
	struct Roots
	{
		int m_count;
		double m_root[MaxRoots];
			
		constexpr Roots() noexcept
			:m_count(0)
			,m_root()
		{}
		constexpr Roots(double a) noexcept
			:m_count(1)
			,m_root()
		{
			m_root[0] = a;
		}
		constexpr Roots(double a, double b) noexcept
			:m_count(2)
			,m_root()
		{
			m_root[0] = a;
			m_root[1] = b;
		}
		constexpr Roots(double a, double b, double c) noexcept
			:m_count(3)
			,m_root()
		{
			m_root[0] = a;
			m_root[1] = b;
			m_root[2] = c;
		}
		constexpr Roots(double a, double b, double c, double d) noexcept
			:m_count(4)
			,m_root()
		{
			m_root[0] = a;
			m_root[1] = b;
			m_root[2] = c;
			m_root[3] = d;
		}
		constexpr double operator[](int i) const noexcept
		{
			pr_assert(i >= 0 && i < m_count);
			return m_root[i];
		}
	};

	//' F(x) = Ax + B. a.k.a. Linear
	struct Monic
	{
		double A;
		double B;

		constexpr Monic(double a, double b) noexcept
			:A(a)
			,B(b)
		{}

		// Evaluate F(x) at 'x'
		constexpr double F(double x) const noexcept
		{
			return A*x + B;
		}

		// Evaluate dF(x)/dx at 'x'
		constexpr double dF(double x) const noexcept
		{
			(void)x;
			return A;
		}

		// Evaluate d²F(x)/dx at 'x'
		constexpr double ddF(double x) const noexcept
		{
			(void)x;
			return 0;
		}

		// Returns a linear approximation of a curve defined by evaluating F(x), dF(x)/dx at 'x'
		static constexpr Monic FromDerivatives(double x, double y, double dy) noexcept
		{
			//' y  = Ax + B
			//' y' = A

			//' A = dy
			auto A = dy;

			//' Ax + B = y
			auto B = y - A*x;

			return Monic(A,B);
		}

		// Create a Monic from 2 points
		template <VectorTypeN<2> Vec>
		static constexpr Monic FromPoints(Vec a, Vec b) noexcept
		{
			using vt = vector_traits<Vec>;
			using S = typename vt::element_t;

			auto dx = vec(b).x - vec(a).x;
			if (Abs(dx) < tiny<S>)
				return Monic(0, vec(a).y); // Vertical line - return horizontal through a.y
			
			auto A = (vec(b).y - vec(a).y) / dx;
			auto B = vec(a).y - A * vec(a).x;
			return Monic(A, B);
		}
	};

	//' F(x) = Ax² + Bx + C
	struct Quadratic
	{
		double A;
		double B;
		double C;

		constexpr Quadratic(double a, double b, double c) noexcept
			:A(a)
			,B(b)
			,C(c)
		{}

		// Evaluate F(x) at 'x'
		constexpr double F(double x) const noexcept
		{
			return (A*x + B)*x + C;
		}

		// Evaluate dF(x)/dx at 'x'
		constexpr double dF(double x) const noexcept
		{
			return 2*A*x + B;
		}

		// Evaluate d²F(x)/dx at 'x'
		constexpr double ddF(double x) const noexcept
		{
			(void)x;
			return 2*A;
		}

		// Returns a quadratic approximation of a curve defined by evaluating F(x), dF(x)/dx, and d²F(x)/dx at 'x'
		static constexpr Quadratic FromDerivatives(double x, double y, double dy, double ddy) noexcept
		{
			//' y  = Ax² + Bx + C
			//' y' = 2Ax + B
			//' y" = 2A

			//' 2A = ddy
			auto A = ddy/2;

			//' 2Ax + B = dy
			auto B = dy - 2*A*x;

			//' Ax² + Bx + C = y
			auto C = y - (A*x + B)*x;

			return Quadratic(A,B,C);
		}

		// Create a quadratic from 3 points
		template <VectorTypeN<2> Vec> static constexpr Quadratic FromPoints(Vec a, Vec b, Vec c) noexcept
		{
			//'   a.y = A(a.x)² + B(a.x) + C
			//'   b.y = A(b.x)² + B(b.x) + C
			//'   c.y = A(c.x)² + B(c.x) + C
			//' => y = Ax where
			//' A = |a.x² a.x 1| x = |A| y = |a.y|
			//'     |b.x² b.x 1|     |B|     |b.y|
			//'     |c.x² c.x 1|     |C|     |c.y|
			using vt = vector_traits<Vec>;
			using S = typename vt::element_t;

			auto M = Transpose(Mat3x4<S>(
				Vec4<S>(vec(a).x*vec(a).x, vec(a).x, S(1), S(0)),
				Vec4<S>(vec(b).x*vec(b).x, vec(b).x, S(1), S(0)),
				Vec4<S>(vec(c).x*vec(c).x, vec(c).x, S(1), S(0))));

			auto y = Vec4<S>(vec(a).y, vec(b).y, vec(c).y, 0);
			auto x = Invert(M) * y;

			return Quadratic(x.x, x.y, x.z);
		}
		template <VectorTypeN<2> Vec> static constexpr Quadratic FromPoints(std::span<Vec const, 3> pts) noexcept
		{
			return FromPoints(pts[0], pts[1], pts[2]);
		}
		static Quadratic FromPoints(double x0, double y0, double x1, double y1, double x2, double y2) noexcept
		{
			auto M = Matrix<double>(3, 3,
			{
				x0*x0, x0, 1.0,
				x1*x1, x1, 1.0,
				x2*x2, x2, 1.0,
			}).transpose();

			auto y = Matrix<double>(1, 3, {y0, y1, y2});
			auto x = Invert(M) * y;

			return Quadratic(x(0, 0), x(0, 1), x(0, 2));
		}
		static Quadratic FromPoints(double const* pts) noexcept
		{
			return FromPoints(pts[0], pts[1], pts[2], pts[3], pts[4], pts[5]);
		}
	};

	//' F(x) = Ax³ + Bx² + Cx + D
	struct Cubic
	{
		double A;
		double B;
		double C;
		double D;

		constexpr Cubic(double a, double b, double c, double d) noexcept
			:A(a)
			,B(b)
			,C(c)
			,D(d)
		{}

		// Evaluate F(x) at 'x'
		constexpr double F(double x) const noexcept
		{
			return ((A*x + B)*x + C)*x + D;
		}

		// Evaluate dF(x)/dx at 'x'
		constexpr double dF(double x) const noexcept
		{
			return (3*A*x + 2*B)*x + C;
		}

		// Evaluate d²F(x)/dx at 'x'
		constexpr double ddF(double x) const noexcept
		{
			return 6*A*x + 2*B;
		}

		// Create a cubic from 4 points
		template <VectorTypeN<2> Vec> static constexpr Cubic FromPoints(Vec a, Vec b, Vec c, Vec d) noexcept
		{
			using vt = vector_traits<Vec>;
			using S = typename vt::element_t;

			//' Aa.x³ + Ba.x² + Ca.x + D = a.y
			//' Ab.x³ + Bb.x² + Cb.x + D = a.y
			//' Ac.x³ + Bc.x² + Cc.x + D = a.y
			//' Ad.x³ + Bd.x² + Cd.x + D = a.y
			//' => Ax = y
			//' A = |a.x² a.x 1| x = |A| y = |a.y|
			//'     |b.x² b.x 1|     |B|     |b.y|
			//'     |c.x² c.x 1|     |C|     |c.y|
			auto M = Transpose(Mat4x4<S>(
				Vec4<S>(vec(a).x*vec(a).x*vec(a).x, vec(a).x*vec(a).x, vec(a).x, S(1)),
				Vec4<S>(vec(b).x*vec(b).x*vec(b).x, vec(b).x*vec(b).x, vec(b).x, S(1)),
				Vec4<S>(vec(c).x*vec(c).x*vec(c).x, vec(c).x*vec(c).x, vec(c).x, S(1)),
				Vec4<S>(vec(d).x*vec(d).x*vec(d).x, vec(d).x*vec(d).x, vec(d).x, S(1))));

			auto y = Vec4<S>(vec(a).y, vec(b).y, vec(c).y, vec(d).y);
			auto x = Invert(M) * y;

			return Cubic(x.x, x.y, x.z, x.w);
		}
	};

	//' F(x) = Ax^4 + Bx³ + Cx² + Dx + E
	struct Quartic
	{
		double A;
		double B;
		double C;
		double D;
		double E;

		constexpr Quartic(double a, double b, double c, double d, double e) noexcept
			:A(a)
			,B(b)
			,C(c)
			,D(d)
			,E(e)
		{}

		// Evaluate F(x) at 'x'
		constexpr double F(double x) const noexcept
		{
			return (((A*x + B)*x + C)*x + D)*x + E;
		}

		// Evaluate dF(x)/dx at 'x'
		constexpr double dF(double x) const noexcept
		{
			return ((4*A*x + 3*B)*x + 2*C)*x + D;
		}

		// Evaluate d²F(x)/dx at 'x'
		constexpr double ddF(double x) const noexcept
		{
			return (12*A*x + 6*B)*x + 2*C;
		}
	};

	// Calculate the real roots of this polynomial
	inline constexpr Roots FindRoots(Monic const& p) noexcept
	{
		if (p.A == 0) return Roots();
		return Roots(-p.B / p.A);
	}
	inline constexpr Roots FindRoots(Quadratic const& p) noexcept
	{
		// This method is numerically more stable than (-b +/- sqrt(b^2-4ac)) / 2a
		// (see numerical recipes, p184)
		auto discriminant = p.B * p.B - 4.0 * p.A * p.C;
		if (discriminant < 0)
			return Roots(); // No real roots

		discriminant = sqrt(discriminant);
		discriminant = p.B < 0
			? -0.5 * (p.B - discriminant)
			: -0.5 * (p.B + discriminant);

		// Check for zero discriminant to avoid division by zero
		if (Abs(discriminant) < tiny<double>)
		{
			// Repeated root case: x = -B / (2A)
			auto root = -p.B / (2 * p.A);
			return Roots(root, root);
		}

		return Roots
		(
			discriminant / p.A,
			p.C / discriminant
		);
	}
	inline constexpr Roots FindRoots(Cubic const& p) noexcept
	{
		// Check for degenerate cubic (A == 0)
		if (Abs(p.A) < tiny<double>)
			return FindRoots(Quadratic(p.B, p.C, p.D));
		
		// See http://www2.hawaii.edu/suremath/jrootsCubic.html for method
		auto a0 = p.D / p.A;
		auto a1 = p.C / p.A;
		auto a2 = p.B / p.A;

		auto q = (a1 / 3.0f) - (a2 * a2 / 9.0f);
		auto r = ((a1 * a2 - 3.0f * a0) / 6.0f) - (a2 * a2 * a2 / 27.0f);
		auto temp = (q * q * q) + (r * r);

		if (temp >= 0.0f)
		{
			temp = Sqrt(temp);
			auto real_s1 = r + temp;
			auto real_s2 = r - temp;

			// Preserve sign when taking cube root
			if (Abs(real_s1) > 0.0f)
			{
				auto sign1 = Sign(real_s1);
				real_s1 = sign1 * Cubert(Abs(real_s1));
			}
			if (Abs(real_s2) > 0.0f)
			{
				auto sign2 = Sign(real_s2);
				real_s2 = sign2 * Cubert(Abs(real_s2));
			}
			return Roots(real_s1 + real_s2 - a2 / 3.0f);
		}

		temp = Abs(temp);
		auto imaginary_s1 = Sqrt(temp);
		auto imaginary_s2 = -imaginary_s1;
		auto real_s1 = r;
		auto real_s2 = r;

		// cube root of s1 and s2
		// note: magnitude of s1 and s2 are equal
		auto magnitude = Cubert(Sqrt(imaginary_s1 * imaginary_s1 + real_s1 * real_s2));

		auto theta = atan2(imaginary_s1, real_s1) / 3.0;
		real_s1 = magnitude * cos(theta);
		imaginary_s1 = magnitude * sin(theta);

		theta = atan2(imaginary_s2, real_s2) / 3.0f;
		real_s2 = magnitude * cos(theta);
		imaginary_s2 = magnitude * sin(theta);

		double const root3_ovr_2 = 0.866025;
		return Roots
		(
			real_s1 + real_s2 - a2 / 3.0f,
			((real_s1 + real_s2) / -2.0f - a2 / 3.0f + (imaginary_s2 - imaginary_s1) * root3_ovr_2),
			((real_s1 + real_s2) / -2.0f - a2 / 3.0f - (imaginary_s2 - imaginary_s1) * root3_ovr_2)
		);
	}
	inline constexpr Roots FindRoots(Quartic const& quartic) noexcept
	{
		// See http://forum.swarthmore.edu/dr.math/problems/cowan2.5.27.98.html
		// Calculate depressed equation (x^4 coefft. = 1, x^3 coefft. = 0) by substituting x = y - b / 4a
		// See http://www.sosmath.com/algebra/factor/fac12/fac12.html
		Quartic depressed_eqn =
		{
			1.0f,
			0.0f,
			(quartic.C - (quartic.B * quartic.B * 3.0f / (8.0f * quartic.A))) / quartic.A,
			((quartic.D + (quartic.B * quartic.B * quartic.B / (8.0f * quartic.A * quartic.A))) - (quartic.B * quartic.C / (2.0f * quartic.A))) / quartic.A,
			(((quartic.E - (quartic.B * quartic.B * quartic.B * quartic.B * 3.0f / (256.0f * quartic.A * quartic.A * quartic.A))) + (quartic.B * quartic.B * quartic.C / (16.0f * quartic.A * quartic.A))) - (quartic.B * quartic.D / (4.0f * quartic.A))) / quartic.A
		};

		// Calculate coefficients. of resolvent cubic equation.
		Cubic res_cubic =
		{
			1.0f,
			2.0f * depressed_eqn.C,
			depressed_eqn.C * depressed_eqn.C - 4.0f * depressed_eqn.E,
			-depressed_eqn.D * depressed_eqn.D
		};

		auto res_cubic_roots = FindRoots(res_cubic);
		if (res_cubic_roots.m_count == 0)
			return Roots{ 0 };

		// Find a positive root
		int n = res_cubic_roots.m_count;
		while (res_cubic_roots.m_root[--n] < 0.0f)
		{
			if (n == 0)
				return Roots{ 0 };
		}
		auto h = Sqrt(res_cubic_roots.m_root[n]);
		auto j = (depressed_eqn.C + res_cubic_roots.m_root[n] - depressed_eqn.D / h) / 2.0f;

		auto roots = Roots{ 0 };
		if (h * h - 4.0f * j >= 0.0f)
		{
			Quadratic quad = { 1.0f, h, j };
			auto quad_roots = FindRoots(quad);
			roots.m_count += quad_roots.m_count;
			roots.m_root[roots.m_count - 1] = quad_roots.m_root[0] - (quartic.B / (quartic.A * 4.0f));
			roots.m_root[roots.m_count - 2] = quad_roots.m_root[1] - (quartic.B / (quartic.A * 4.0f));
		}

		h = -h;
		j = depressed_eqn.E / j;
		if (h * h - 4.0f * j >= 0.0f)
		{
			Quadratic quad = { 1.0f, h, j };
			auto quad_roots = FindRoots(quad);
			roots.m_count += quad_roots.m_count;
			roots.m_root[roots.m_count - 1] = quad_roots.m_root[0] - (quartic.B / (quartic.A * 4.0f));
			roots.m_root[roots.m_count - 2] = quad_roots.m_root[1] - (quartic.B / (quartic.A * 4.0f));
		}
		return roots;
	}

	// Return the X values of the maxima, minima, or inflection points
	inline constexpr Roots StationaryPoints(Monic const& p) noexcept
	{
		(void)p;
		return Roots();
	}
	inline constexpr Roots StationaryPoints(Quadratic const& p) noexcept
	{
		// Check for linear polynomial (A == 0)
		if (Abs(p.A) < tiny<double>)
			return Roots(); // Linear has no stationary points
		
		return Roots(-p.B / (2.0f * p.A));
	}
	inline constexpr Roots StationaryPoints(Cubic const& p) noexcept
	{
		// dF(x) = 3Ax² + 2Bx + C
		// dF(x) == 0 at the roots of dF(x)
		return FindRoots(Quadratic(3*p.A, 2*p.B, p.C));
	}
	inline constexpr Roots StationaryPoints(Quartic const& p) noexcept
	{
		// dF(x) = 4Ax³ + 3Bx² + 2Cx + D
		// dF(x) == 0 at the roots of dF(x)
		return FindRoots(Cubic(4*p.A, 3*p.B, 2*p.C, 1*p.D));
	}
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::math::tests
{
	PRUnitTestClass(PolynomialTests)
	{
		using Vec2 = Vec2<double>;
		PRUnitTestMethod(FromPoints)
		{
			Vec2 a(0.5, +0.3);
			Vec2 b(0.7, -0.2);
			Vec2 c(1.0, +0.6);

			auto q = Quadratic::FromPoints(a, b, c);
			PR_EXPECT(FEql(q.F(a.x), a.y));
			PR_EXPECT(FEql(q.F(b.x), b.y));
			PR_EXPECT(FEql(q.F(c.x), c.y));
		}
		PRUnitTestMethod(FromPoints2)
		{
			double a[] = { 0.5,  0.3 };
			double b[] = { 0.7, -0.2 };
			double c[] = { 1.0,  0.6 };

			auto q = Quadratic::FromPoints(a[0], a[1], b[0], b[1], c[0], c[1]);
			PR_EXPECT(FEql(q.F(a[0]), a[1]));
			PR_EXPECT(FEql(q.F(b[0]), b[1]));
			PR_EXPECT(FEql(q.F(c[0]), c[1]));
		}
	};
}
#endif

////   bezout
////   This code is based on MgcIntr2DElpElp.cpp written by David Eberly.  His
////  code along with many other excellent examples are avaiable at his site:
////   http://www.magic-software.com
//Intersection.bezout
// = function(e1, e2) {
//    var AB    = e1[0]*e2[1] - e2[0]*e1[1];
//    var AC    = e1[0]*e2[2] - e2[0]*e1[2];
//    var AD    = e1[0]*e2[3] - e2[0]*e1[3];
//    var AE    = e1[0]*e2[4] - e2[0]*e1[4];
//    var AF    = e1[0]*e2[5] - e2[0]*e1[5];
//    var BC    = e1[1]*e2[2] - e2[1]*e1[2];
//    var BE    = e1[1]*e2[4] - e2[1]*e1[4];
//    var BF    = e1[1]*e2[5] - e2[1]*e1[5];
//    var CD    = e1[2]*e2[3] - e2[2]*e1[3];
//    var DE    = e1[3]*e2[4] - e2[3]*e1[4];
//    var DF    = e1[3]*e2[5] - e2[3]*e1[5];
//    var BFpDE = BF + DE;
//    var BEmCD = BE - CD;
//
//    return new Polynomial(
//        AB*BC - AC*AC,
//        AB*BEmCD + AD*BC - 2*AC*AE,
//        AB*BFpDE + AD*BEmCD - AE*AE - 2*AC*AF,
//        AB*DF + AD*BFpDE - 2*AE*AF,
//        AD*DF - AF*AF
//    );
//};

//   intersectEllipseEllipse
//   This code is based on MgcIntr2DElpElp.cpp written by David Eberly.  His
//   code along with many other excellent examples are avaiable at his site:
//   http://www.magic-software.com
//   NOTE: Rotation will need to be added to this function
//Intersection.intersectEllipseEllipse = function(c1, rx1, ry1, c2, rx2, ry2)
//{
//    var a =
//	[
//        ry1*ry1, 0, rx1*rx1, -2*ry1*ry1*c1.x, -2*rx1*rx1*c1.y,
//        ry1*ry1*c1.x*c1.x + rx1*rx1*c1.y*c1.y - rx1*rx1*ry1*ry1
//    ];
//    var b =
//	[
//        ry2*ry2, 0, rx2*rx2, -2*ry2*ry2*c2.x, -2*rx2*rx2*c2.y,
//        ry2*ry2*c2.x*c2.x + rx2*rx2*c2.y*c2.y - rx2*rx2*ry2*ry2
//    ];
//
//    var yPoly   = Intersection.bezout(a, b);
//    var yRoots  = yPoly.getRoots();
//    var epsilon = 1e-3;
//    var norm0   = ( a[0]*a[0] + 2*a[1]*a[1] + a[2]*a[2] ) * epsilon;
//    var norm1   = ( b[0]*b[0] + 2*b[1]*b[1] + b[2]*b[2] ) * epsilon;
//    var result  = new Intersection("No Intersection");
//
//    for ( var y = 0; y < yRoots.length; y++ )
//	{
//        var xPoly = new Polynomial(
//            a[0],
//            a[3] + yRoots[y] * a[1],
//            a[5] + yRoots[y] * (a[4] + yRoots[y]*a[2])
//        );
//        var xRoots = xPoly.getRoots();
//
//        for ( var x = 0; x < xRoots.length; x++ ) {
//            var test =
//                ( a[0]*xRoots[x] + a[1]*yRoots[y] + a[3] ) * xRoots[x] +
//                ( a[2]*yRoots[y] + a[4] ) * yRoots[y] + a[5];
//            if ( Math.abs(test) < norm0 ) {
//                test =
//                    ( b[0]*xRoots[x] + b[1]*yRoots[y] + b[3] ) * xRoots[x] +
//                    ( b[2]*yRoots[y] + b[4] ) * yRoots[y] + b[5];
//                if ( Math.abs(test) < norm1 ) {
//                    result.appendPoint( new Point2D( xRoots[x], yRoots[y] ) );
//                }
//            }
//        }
//    }
//
//    if ( result.points.length > 0 ) result.status = "Intersection";
//
//    return result;
//}

	//// A polynomial of maximum order 'N'
	//// c0 + c1.x + c2.x^2 + c3.x^3 + ... + cN.x^N
	//template <uint32_t N>
	//struct Polynomial
	//{
	//	uint32_t	m_degree;		// The number of coefficients in use is m_degree + 1
	//	float	m_coefs[N+1];

	//	//.TOLERANCE=1e-6;
	//	//.ACCURACY=6;
	//	Polynomial()
	//	:m_degree(0)
	//	,m_coefs()
	//	{}
	//	Polynomial(float const* coefs, uint32_t num_coefs)
	//	{
	//		Set(coefs, num_coefs);
	//	}
	//	void Set(float const* coefs, uint32_t num_coefs)
	//	{
	//		pr_assert(num_coefs <= N && "This object cannot represent a polynomial of this order");
	//		m_degree = num_coefs - 1;
	//		float* in = m_coefs + m_degree;
	//		float const* out = coefs;
	//		while( num_coefs-- ) { *in-- = *out++; }
	//		//for( uint32_t i = m_degree; i >= 0; --i ) this.coefs.push(coefs[i]);
	//	}
	//	float Eval(float x) const
	//	{
	//		float result = 0.0f;
	//		for( uint32_t i = N-1; i >= 0; --i )
	//			result = result*x + m_coefs[i];
	//		return result;
	//	}
	//	Polynomial<N> Multiply(Polynomial<N> const& that)
	//	{
	//		var result=new Polynomial();
	//		for(var i=0;i<=this.getDegree()+that.getDegree();i++)
	//			result.coefs.push(0);

	//		for(var i=0;i<=this.getDegree();i++)
	//			for(var j=0;j<=that.getDegree();j++)
	//				result.coefs[i+j]+=m_coefs[i]*that.coefs[j];
	//		return result;
	//	}
	//	void DivideScalar(float scalar)
	//	{
	//		for( uint32_t i = 0; i != N; ++i )
	//			m_coefs[i] /= scalar;
	//	}
	//	void Simplify()
	//	{
	//		for( var i=this.getDegree(); i>=0; i--)
	//		{
	//			if(Math.abs(m_coefs[i])<=Polynomial.TOLERANCE)
	//				m_coefs.pop();
	//			else
	//				break;
	//		}
	//	};

	//	bisection(min,max)
	//	{
	//		var minValue=this.eval(min);
	//		var maxValue=this.eval(max);
	//		var result;
	//		if(Math.abs(minValue)<=Polynomial.TOLERANCE)
	//			result=min;
	//		else if(Math.abs(maxValue)<=Polynomial.TOLERANCE)
	//			result=max;
	//		else if(minValue*maxValue<=0)
	//		{
	//			var tmp1=Math.log(max-min);
	//			var tmp2=Math.log(10)*Polynomial.ACCURACY;
	//			var iters=Math.ceil((tmp1+tmp2)/Math.log(2));
	//			for(var i=0; i<iters; i++)
	//			{
	//				result=0.5*(min+max);
	//				var value=this.eval(result);
	//				if(Math.abs(value)<=Polynomial.TOLERANCE)
	//				{
	//					break;
	//				}
	//				if(value*minValue<0)
	//				{
	//					max=result;
	//					maxValue=value;
	//				}
	//				else
	//				{
	//					min=result;
	//					minValue=value;
	//				}
	//			}
	//		}
	//		return result;
	//	}

	//	void toString()
	//	{
	//		var coefs=new Array();
	//		var signs=new Array();
	//		for(var i=N-1; i>=0; i--)
	//		{
	//			var value=m_coefs[i];
	//			if(value!=0)
	//			{
	//				var sign=(value<0)?" - ":" + ";
	//				value=Math.abs(value);
	//				if(i>0)
	//					if(value==1)
	//						value="x";
	//					else
	//						value+="x";
	//				if(i>1)
	//					value+="^"+i;
	//				signs.push(sign);
	//				coefs.push(value);
	//			}
	//		}
	//		signs[0]=(signs[0]==" + ")?"":"-";
	//		var result="";
	//		for(var i=0; i<coefs.length; i++)
	//			result+=signs[i]+coefs[i];
	//		return result;
	//	}

	//	uint32_t Degree() const
	//	{
	//		return N-1;
	//	}
	//	Polynomial<N-1> Derivative() const
	//	{
	//		var derivative=new Polynomial();
	//		for(var i=1; i<N; i++)
	//		{
	//			derivative.coefs.push(i*m_coefs[i]);
	//		}
	//		return derivative;
	//	}
	//	Roots()
	//	{
	//		var result;
	//		this.simplify();
	//		switch(this.getDegree())
	//		{
	//		case 0:
	//			result=new Array();
	//			break;
	//		case 1:
	//			result=this.getLinearRoot();
	//			break;
	//		case 2:
	//			result=this.getQuadraticRoots();
	//			break;
	//		case 3:
	//			result=this.getCubicRoots();
	//			break;
	//		case 4:
	//			result=this.getQuarticRoots();
	//			break;
	//		default:
	//			result=new Array();
	//		}
	//		return result;
	//	}
	//	RootsInInterval(min,max)
	//	{
	//		var roots=new Array();
	//		var root;
	//		if(this.getDegree()==1)
	//		{
	//			root=this.bisection(min,max);
	//			if(root!=null)
	//				roots.push(root);
	//		}
	//		else
	//		{
	//			var deriv=this.getDerivative();
	//			var droots=deriv.getRootsInInterval(min,max);if(droots.length>0){root=this.bisection(min,droots[0]);if(root!=null)roots.push(root);for(i=0;i<=droots.length-2;i++){root=this.bisection(droots[i],droots[i+1]);if(root!=null)roots.push(root);}root=this.bisection(droots[droots.length-1],max);if(root!=null)roots.push(root);}else{root=this.bisection(min,max);if(root!=null)roots.push(root);}}return roots;};
	//	float	LinearRoot() const
	//	{
	//		float a = m_coefs[1];
	//		if( a != 0.0f ) result.push(-m_coefs[0]/a);
	//		return result;
	//	}
	//	float*	QuadraticRoots() const
	//	{
	//		var results=new Array();
	//		if(this.getDegree()==2)
	//		{
	//			var a=m_coefs[2];
	//			var b=m_coefs[1]/a;
	//			var c=m_coefs[0]/a;
	//			var d=b*b-4*c;
	//			if(d>0)
	//			{
	//				var e=Math.sqrt(d);
	//				results.push(0.5*(-b+e));
	//				results.push(0.5*(-b-e));
	//			}
	//			else if(d==0)
	//			{
	//				results.push(0.5*-b);
	//			}
	//		}
	//		return results;
	//	}
	//	float* CubicRoots() const
	//	{
	//		var results=new Array();
	//		if(this.getDegree()==3)
	//		{
	//			var c3=m_coefs[3];
	//			var c2=m_coefs[2]/c3;
	//			var c1=m_coefs[1]/c3;
	//			var c0=m_coefs[0]/c3;
	//			var a=(3*c1-c2*c2)/3;
	//			var b=(2*c2*c2*c2-9*c1*c2+27*c0)/27;
	//			var offset=c2/3;
	//			var discrim=b*b/4 + a*a*a/27;
	//			var halfB=b/2;
	//			if(Math.abs(discrim)<=Polynomial.TOLERANCE)disrim=0;
	//			if(discrim>0)
	//			{
	//				var e=Math.sqrt(discrim);
	//				var tmp;
	//				var root;
	//				tmp=-halfB+e;
	//				if(tmp>=0)root=Math.pow(tmp,1/3);
	//				else root=-Math.pow(-tmp,1/3);
	//				tmp=-halfB-e;
	//				if(tmp>=0)root+=Math.pow(tmp,1/3);
	//				else root-=Math.pow(-tmp,1/3);
	//				results.push(root-offset);
	//			}
	//			else if(discrim<0)
	//			{
	//				var distance=Math.sqrt(-a/3);
	//				var angle=Math.atan2(Math.sqrt(-discrim),-halfB)/3;
	//				var cos=Math.cos(angle);
	//				var sin=Math.sin(angle);
	//				var sqrt3=Math.sqrt(3);
	//				results.push(2*distance*cos-offset);
	//				results.push(-distance*(cos+sqrt3*sin)-offset);
	//				results.push(-distance*(cos-sqrt3*sin)-offset);
	//			}
	//			else
	//			{
	//				var tmp;
	//				if(halfB>=0)
	//					tmp=-Math.pow(halfB,1/3);
	//				else
	//					tmp=Math.pow(-halfB,1/3);
	//				results.push(2*tmp-offset);
	//				results.push(-tmp-offset);
	//			}
	//		}
	//		return results;
	//	}
	//
	//	float* QuarticRoots() const
	//	{
	//		var results=new Array();
	//		if(this.getDegree()==4)
	//		{
	//			var c4=m_coefs[4];
	//			var c3=m_coefs[3]/c4;
	//			var c2=m_coefs[2]/c4;
	//			var c1=m_coefs[1]/c4;
	//			var c0=m_coefs[0]/c4;
	//			var resolveRoots=new Polynomial(1,-c2,c3*c1-4*c0,-c3*c3*c0+4*c2*c0-c1*c1).getCubicRoots();
	//			var y=resolveRoots[0];
	//			var discrim=c3*c3/4-c2+y;
	//			if(Math.abs(discrim)<=Polynomial.TOLERANCE)
	//				discrim=0;
	//			if(discrim>0)
	//			{
	//				var e=Math.sqrt(discrim);
	//				var t1=3*c3*c3/4-e*e-2*c2;
	//				var t2=(4*c3*c2-8*c1-c3*c3*c3)/(4*e);
	//				var plus=t1+t2;
	//				var minus=t1-t2;
	//				if(Math.abs(plus)<=Polynomial.TOLERANCE)plus=0;
	//				if(Math.abs(minus)<=Polynomial.TOLERANCE)minus=0;
	//				if(plus>=0)
	//				{
	//					var f=Math.sqrt(plus);
	//					results.push(-c3/4 + (e+f)/2);
	//					results.push(-c3/4 + (e-f)/2);
	//				}
	//				if(minus>=0)
	//				{
	//					var f=Math.sqrt(minus);
	//					results.push(-c3/4 + (f-e)/2);
	//					results.push(-c3/4 - (f+e)/2);
	//				}
	//			}
	//			else if(discrim<0)
	//			{}
	//			else
	//			{
	//				var t2=y*y-4*c0;
	//				if(t2>=-Polynomial.TOLERANCE)
	//				{
	//					if(t2<0)t2=0;
	//					t2=2*Math.sqrt(t2);
	//					t1=3*c3*c3/4-2*c2;
	//					if(t1+t2>=Polynomial.TOLERANCE)
	//					{
	//						var d=Math.sqrt(t1+t2);
	//						results.push(-c3/4 + d/2);
	//						results.push(-c3/4 - d/2);
	//					}
	//					if(t1-t2>=Polynomial.TOLERANCE)
	//					{
	//						var d=Math.sqrt(t1-t2);
	//						results.push(-c3/4 + d/2);
	//						results.push(-c3/4 - d/2);
	//					}
	//				}
	//			}
	//		}
	//		return results;
	//	}
	//};
