using System;
using System.Collections.Generic;
using System.Diagnostics;

namespace Rylogic.Maths
{
	/// <summary>Function with first and second derivative</summary>
	public interface IPolynomial
	{
		/// <summary>Evaluate F(x) at 'x'</summary>
		double F(double x);

		/// <summary>Evaluate dF(x)/dx at 'x'</summary>
		double dF(double x);

		/// <summary>Evaluate d²F(x)/dx at 'x'</summary>
		double ddF(double x);

		/// <summary>Return the real roots of the polynomial</summary>
		double[] Roots { get; }

		/// <summary>Return the X values of the maxima, minima, or inflection points</summary>
		double[] StationaryPoints { get; }

		/// <summary>Return the values of X where Y = 'y'</summary>
		double[] Solve(double y);
	}

	/// <summary>'F(x) = Ax + B. a.k.a. Linear</summary>
	[DebuggerDisplay("A={A} B={B}")]
	public class Monic :IPolynomial
	{
		public double A;
		public double B;

		public Monic(double a, double b)
		{
			A = a;
			B = b;
		}

		/// <summary>Evaluate F(x) at 'x'</summary>
		public double F(double x)
		{
			return A*x + B;
		}

		/// <summary>Evaluate dF(x)/dx at 'x'</summary>
		public double dF(double x)
		{
			return A;
		}

		/// <summary>Evaluate d²F(x)/dx at 'x'</summary>
		public double ddF(double x)
		{
			return 0;
		}

		/// <summary>Evaluate the definite integral of the monic from 'x0' to 'x1'</summary>
		public double Integrate(double x0, double x1)
		{
			//' y(x) = Ax + B
			//' Intg-y(x) = Ax²/2 + Bx + C
			//' (A/2)(x1² - x0²) + (B/1)(x1 - x0)
			return (A/2.0)*(x1*x1 - x0*x0) + (B/1.0)*(x1 - x0);
		}

		/// <summary>Evaluate the indefinite integral with constant of integration 'C'</summary>
		public Quadratic Integrate(double C)
		{
			return new Quadratic(A/2.0, B/1.0, C);
		}

		/// <summary>Calculate the real roots of this polynomial</summary>
		public double[] Roots
		{
			get
			{
				if (A == 0) return new double[0];
				return new double[1] { -B / A };
			}
		}

		/// <summary>Return the X values of the maxima, minima, or inflection points</summary>
		public double[] StationaryPoints
		{
			get { return new double[0] {}; }
		}

		/// <summary>Return the values of X where Y = 'y'</summary>
		public double[] Solve(double y)
		{
			//' Ax + B - y == 0
			return new Monic(A, B - y).Roots;

		}

		/// <summary>Returns a linear approximation of a curve defined by evaluating F(x), dF(x)/dx at 'x'</summary>
		public static Monic FromDerivatives(double x, double y, double dy)
		{
			//' y  = Ax + B
			//' y' = A

			// A = dy
			var A = dy;

			// Ax + B = y
			var B = y - A*x;

			return new Monic(A,B);
		}

		/// <summary>Create a Monic from 2 points</summary>
		public static Monic FromPoints(v2 a, v2 b)
		{
			var A = (b.y - a.y) / (b.x - a.x);
			var B = a.y - A * a.x;
			return new Monic(A, B);
		}
		public static Monic FromPoints(double x0, double y0, double x1, double y1)
		{
			return FromPoints(
				new v2((float)x0, (float)y0),
				new v2((float)x1, (float)y1));
		}

		/// <summary>Create a best-fit monic for the given points</summary>
		public static Monic FromLinearRegression(IList<v2> points)
		{
			var corr = new Correlation();
			foreach (var pt in points)
				corr.Add(pt.x, pt.y);

			return corr.LinearRegression;
		}
	}

	/// <summary>'F(x) = Ax² + Bx + C'</summary>
	[DebuggerDisplay("A={A} B={B} C={C}")]
	public class Quadratic :IPolynomial
	{
		public double A;
		public double B;
		public double C;

		public Quadratic(double a, double b, double c)
		{
			A = a;
			B = b;
			C = c;
		}

		/// <summary>Evaluate F(x) at 'x'</summary>
		public double F(double x)
		{
			return (A*x + B)*x + C;
		}

		/// <summary>Evaluate dF(x)/dx at 'x'</summary>
		public double dF(double x)
		{
			return 2*A*x + B;
		}

		/// <summary>Evaluate d²F(x)/dx at 'x'</summary>
		public double ddF(double x)
		{
			return 2*A;
		}

		/// <summary>Evaluate the definite integral of the quadratic from 'x0' to 'x1'</summary>
		public double Integrate(double x0, double x1)
		{
			//' y(x) = Ax² + Bx + C
			//' Intg(y(x)) = Ax³/3 + Bx²/2 + Cx + D |
			//' Ax1³/3 + Bx1²/2 + Cx1 + D - Ax0³/3 - Bx0²/2 - Cx0 - D
			//' Ax1³/3 - Ax0³/3 + Bx1²/2 - Bx0²/2 + Cx1 - Cx0 + D - D 
			//' Ax1³/3 - Ax0³/3 + Bx1²/2 - Bx0²/2 + Cx1 - Cx0 + D - D 
			//' (A/3)(x1³ - x0³) + (B/2)(x1² - x0²) + C(x1 - x0)
			return (A/3.0)*(x1*x1*x1 - x0*x0*x0) + (B/2.0)*(x1*x1 - x0*x0) + (C/1.0)*(x1 - x0);
		}

		/// <summary>Evaluate the indefinite integral with constant of integration 'D'</summary>
		public Cubic Integrate(double D)
		{
			return new Cubic(A/3.0, B/2.0, C/1.0, D);
		}

		/// <summary>Calculate the real roots of this polynomial</summary>
		public double[] Roots
		{
			get
			{
				// This method is numerically more stable than (-b +/- sqrt(b^2-4ac)) / 2a
				// (see numerical recipes, p184)
				var discriminant = B * B - 4.0 * A * C;
				if (discriminant < 0)
					return new double[0]; // No real roots

				discriminant = Math.Sqrt(discriminant);
				discriminant = B < 0
					? -0.5 * (B - discriminant)
					: -0.5 * (B + discriminant);

				var roots = new double[2]
				{
					discriminant / A,
					C / discriminant
				};
				if (roots[0] > roots[1])
				{
					Math_.Swap(ref roots[0], ref roots[1]);
				}
				return roots;
			}
		}

		/// <summary>Return the X values of the maxima, minima, or inflection points</summary>
		public double[] StationaryPoints
		{
			get { return new double[1] { -B / (2*A) }; }
		}

		/// <summary>Return the values of X where Y = 'y'</summary>
		public double[] Solve(double y)
		{
			//' Ax² + Bx + C - y == 0
			return new Quadratic(A, B, C - y).Roots;
		}

		/// <summary>Returns a quadratic approximation of a curve defined by evaluating F(x), dF(x)/dx, and d²F(x)/dx at 'x'</summary>
		public static Quadratic FromDerivatives(double x, double y, double dy, double ddy)
		{
			//' y  = Ax² + Bx + C
			//' y' = 2Ax + B
			//' y" = 2A

			// 2A = ddy
			var A = ddy/2;

			// 2Ax + B = dy
			var B = dy - 2*A*x;

			// Ax² + Bx + C = y
			var C = y - (A*x - B)*x;

			return new Quadratic(A,B,C);
		}

		/// <summary>Create a quadratic from 3 points</summary>
		public static Quadratic FromPoints(v2 a, v2 b, v2 c)
		{
			//' Aa.x2 + Ba.x + C = a.y
			//' Ab.x2 + Bb.x + C = a.y
			//' Ac.x2 + Bc.x + C = a.y
			//' => Ax = y
			//' A = |a.x² a.x 1| x = |A| y = |a.y|
			//'     |b.x² b.x 1|     |B|     |b.y|
			//'     |c.x² c.x 1|     |C|     |c.y|
			var M = Math_.Transpose(new m3x4(
				new v4(a.x*a.x, a.x, 1, 0),
				new v4(b.x*b.x, b.x, 1, 0),
				new v4(c.x*c.x, c.x, 1, 0)));

			var y = new v4(a.y, b.y, c.y, 0);
			var x = Math_.Invert(M) * y;

			return new Quadratic(x.x, x.y, x.z);
		}
		public static Quadratic FromPoints(double x0, double y0, double x1, double y1, double x2, double y2)
		{
			return FromPoints(
				new v2((float)x0, (float)y0),
				new v2((float)x1, (float)y1),
				new v2((float)x2, (float)y2));
		}

		/// <summary>Create a best-fit quadratic for the given points</summary>
		public static Quadratic FromLinearRegression(IList<v2> points)
		{
			// Minimise F = Sum{(ax² + bx + c - y)²} where x,y are the points in 'points' (of length N)
			// The minimum of F is when dF/da == 0, dF/db == 0, and dF/dc == 0
			// Expand F and split the sum:
			//    Sum{(ax² + bx + c - y)(ax² + bx + c - y)}
			//  = Sum{(a²x^4 + abx³ + acx² - ayx² + abx³ + b²x² + bcx - byx + acx² + bcx + c² - cy - ax²y - bxy - cy + y²)}
			//  = a²Sum{x^4} + 2abSum{x³} + (b²+2ac)Sum{x²} - 2aSum{x²y} + 2bcSum{x} - 2bSum{yx} + Sum{c²} - 2cSum{y} + Sum{y²}
			// Let Sij mean Sum{x^i*y^j}, e.g S40 = Sum{x^4*y^0}. Note: S00 = Sum{X^0*y^0} = N
			//  = a²S40 + 2abS30 + (b²+2ac)S20 - 2aS21 + 2bcS10 - 2bS11 + c²S00 - 2cS01 + S02
			// Take partial derivatives w.r.t a,b,c:
			//  dF/da = 2aS40 + 2bS30 + 2cS20 - 2S21 == 0
			//  dF/db = 2aS30 + 2bS20 + 2cS10 - 2S11 == 0
			//  dF/dc = 2aS20 + 2bS10 + 2cS00 - 2S01 == 0
			// Solve the system of equations (divided through by 2)
			//         M         *  a  =   b
			//   [S40, S30, S20]   [a]   [S21]
			//   [S30, S20, S10] * [b] = [S11]
			//   [S20, S10, S00]   [c]   [S01]

			var M = m3x4.Zero;
			var b = v3.Zero;
			foreach (var pt in points)
			{
				var x = pt.x;
				var y = pt.y;
				var x2 = x * x;

				M.x.x += x2 * x2; // S40
				M.x.y += x2 * x;  // S30
				M.y.y += x2;      // S20
				M.y.z += x;       // S10

				b.x += x2*y;  // S21
				b.y += x*y;   // S11
				b.z += y;     // S01
			}
			M.x.z = M.y.y;
			M.z.x = M.y.y;
			M.y.x = M.x.y;
			M.z.y = M.y.z;
			M.z.z = points.Count;

			var a = Math_.Invert(M) * b;
			return new Quadratic(a.x, a.y, a.z);
		}

		#region Operators
		public static Quadratic operator + (Quadratic lhs)
		{
			return lhs;
		}
		public static Quadratic operator + (Quadratic lhs, Quadratic rhs)
		{
			return new Quadratic(lhs.A + rhs.A, lhs.B + rhs.B, lhs.C + rhs.C);
		}
		public static Quadratic operator + (Quadratic lhs, Monic rhs)
		{
			return new Quadratic(lhs.A, lhs.B + rhs.A, lhs.C + rhs.B);
		}
		public static Quadratic operator + (Monic lhs, Quadratic rhs)
		{
			return rhs + lhs;
		}
		public static Quadratic operator - (Quadratic lhs)
		{
			return new Quadratic(-lhs.A, -lhs.B, -lhs.C);
		}
		public static Quadratic operator - (Quadratic lhs, Quadratic rhs)
		{
			return new Quadratic(lhs.A - rhs.A, lhs.B - rhs.B, lhs.C - rhs.C);
		}
		public static Quadratic operator - (Quadratic lhs, Monic rhs)
		{
			return new Quadratic(lhs.A, lhs.B - rhs.A, lhs.C - rhs.B);
		}
		public static Quadratic operator - (Monic lhs, Quadratic rhs)
		{
			return -(rhs - lhs);
		}
		public static Quadratic operator * (double lhs, Quadratic rhs)
		{
			return new Quadratic(lhs * rhs.A, lhs * rhs.B, lhs * rhs.C);
		}
		public static Quadratic operator * (Quadratic lhs, double rhs)
		{
			return rhs * lhs;
		}
		public static Quadratic operator / (Quadratic lhs, double rhs)
		{
			return lhs * (1.0/rhs);
		}
		#endregion
	}

	/// <summary>'F(x) = Ax³ + Bx² + Cx + D'</summary>
	[DebuggerDisplay("A={A} B={B} C={C} D={D}")]
	public class Cubic :IPolynomial
	{
		public double A;
		public double B;
		public double C;
		public double D;

		public Cubic(double a, double b, double c, double d)
		{
			A = a;
			B = b;
			C = c;
			D = d;
		}

		/// <summary>Evaluate F(x) at 'x'</summary>
		public double F(double x)
		{
			return ((A*x + B)*x + C)*x + D;
		}

		/// <summary>Evaluate dF(x)/dx at 'x'</summary>
		public double dF(double x)
		{
			return (3*A*x + 2*B)*x + C;
		}

		/// <summary>Evaluate d²F(x)/dx at 'x'</summary>
		public double ddF(double x)
		{
			return 6*A*x + 2*B;
		}

		/// <summary>Calculate the real roots of this polynomial</summary>
		public double[] Roots
		{
			get
			{
				// See http://www2.hawaii.edu/suremath/jrootsCubic.html for method
				var a0 = D / A;
				var a1 = C / A;
				var a2 = B / A;

				var q = (a1 / 3) - (a2 * a2 / 9);
				var r = ((a1 * a2 - 3 * a0) / 6) - (a2 * a2 * a2 / 27);
				var temp = (q * q * q) + (r * r);

				if (temp >= 0)
				{
					temp = Math.Sqrt(temp);
					var real_s1 = r + temp;
					var real_s2 = r - temp;

					if (Math.Abs(real_s1) > 0)
					{
						real_s1 = Math_.CubeRoot(Math.Abs(real_s1));
						if (real_s1 < 0)
							real_s1 = -real_s1;
					}
					if (Math.Abs(real_s2) > 0)
					{
						real_s2 = Math_.CubeRoot(Math.Abs(real_s2));
						if (real_s2 < 0)
							real_s2 = -real_s2;
					}
					return new double[1] { real_s1 + real_s2 - a2 / 3 };
				}
				else
				{
					temp = Math.Abs(temp);
					var imaginary_s1 = Math.Sqrt(temp);
					var imaginary_s2 = -imaginary_s1;
					var real_s1 = r;
					var real_s2 = r;

					// cube root of s1 and s2
					// note: magnitude of s1 and s2 are equal
					var magnitude = Math_.CubeRoot(Math.Sqrt(imaginary_s1 * imaginary_s1 + real_s1 * real_s2));

					var theta = Math.Atan2(imaginary_s1, real_s1) / 3;
					real_s1 = magnitude * Math.Cos(theta);
					imaginary_s1 = magnitude * Math.Sin(theta);

					theta = Math.Atan2(imaginary_s2, real_s2) / 3;
					real_s2 = magnitude * Math.Cos(theta);
					imaginary_s2 = magnitude * Math.Sin(theta);

					const double Root3By2 = 0.86602540378443864676;
					return new double[3]
					{
						real_s1 + real_s2 - a2 / 3.0f,
						((real_s1 + real_s2)/-2 - a2/3 + (imaginary_s2 - imaginary_s1) * Root3By2),
						((real_s1 + real_s2)/-2 - a2/3 - (imaginary_s2 - imaginary_s1) * Root3By2)
					};
				}
			}
		}

		/// <summary>Return the X values of the maxima, minima, or inflection points</summary>
		public double[] StationaryPoints
		{
			get
			{
				// dF(x) = 3Ax² + 2Bx + C
				// dF(x) == 0 at the roots of dF(x)
				return new Quadratic(3*A, 2*B, C).Roots;
			}
		}

		/// <summary>Return the values of X where Y = 'y'</summary>
		public double[] Solve(double y)
		{
			//' Ax³ + Bx² + Cx + D - y == 0
			return new Cubic(A, B, C, D - y).Roots;
		}

		// Create a cubic from 4 points
		public static Cubic FromPoints(v2 a, v2 b, v2 c, v2 d)
		{
			//' Aa.x³ + Ba.x² + Ca.x + D = a.y
			//' Ab.x³ + Bb.x² + Cb.x + D = a.y
			//' Ac.x³ + Bc.x² + Cc.x + D = a.y
			//' Ad.x³ + Bd.x² + Cd.x + D = a.y
			//' => Ax = y
			//' A = |a.x² a.x 1| x = |A| y = |a.y|
			//'     |b.x² b.x 1|     |B|     |b.y|
			//'     |c.x² c.x 1|     |C|     |c.y|
			var M = Math_.Transpose(new m4x4(
				new v4(a.x*a.x*a.x, a.x*a.x, a.x, 1),
				new v4(b.x*b.x*b.x, b.x*b.x, b.x, 1),
				new v4(c.x*c.x*c.x, c.x*c.x, c.x, 1),
				new v4(d.x*d.x*d.x, d.x*d.x, d.x, 1)));

			var y = new v4(a.y, b.y, c.y, d.y);
			var x = Math_.Invert(M) * y;

			return new Cubic(x.x, x.y, x.z, x.w);
		}
	}

	/// <summary>Polynomial functions</summary>
	public static partial class Math_
	{
		/// <summary>Return the X values of the points if intersection with 'rhs'</summary>
		public static double[] Intersection(Quadratic lhs, Quadratic rhs)
		{
			return (lhs - rhs).Roots;
		}
		public static double[] Intersection(Quadratic lhs, Monic rhs)
		{
			return (lhs - rhs).Roots;
		}
		public static double[] Intersection(Monic lhs, Quadratic rhs)
		{
			return (lhs - rhs).Roots;
		}
		public static double[] Intersection(IPolynomial lhs, IPolynomial rhs)
		{
			if (lhs is Monic)
			{
				if (rhs is Monic    ) return Intersection((Monic)lhs, (Monic)rhs);
				if (rhs is Quadratic) return Intersection((Monic)lhs, (Quadratic)rhs);
				throw new NotSupportedException();
			}
			if (lhs is Quadratic)
			{
				if (rhs is Monic    ) return Intersection((Quadratic)lhs, (Monic)rhs);
				if (rhs is Quadratic) return Intersection((Quadratic)lhs, (Quadratic)rhs);
				throw new NotSupportedException();
			}
			throw new NotSupportedException();
		}
	}
}

#if PR_UNITTESTS
namespace Rylogic.UnitTests
{
	using Maths;

	[TestFixture] public class TestPolynomial
	{
		[Test] public void FromPoints()
		{
			var a = new v2(0.5f, 0.3f);
			var b = new v2(0.7f, -0.2f);
			var c = new v2(1.0f, 0.6f);

			var q = Quadratic.FromPoints(a,b,c);
			Assert.Equal(q.F(a.x), a.y, Math_.TinyF);
			Assert.Equal(q.F(b.x), b.y, Math_.TinyF);
			Assert.Equal(q.F(c.x), c.y, Math_.TinyF);
		}
		[Test] public void FromLinearRegression()
		{
			{
				var pts = new v2[]
				{
					new v2(0,15),
					new v2(1,13),
					new v2(2,10),
					new v2(3, 7),
					new v2(4, 4),
					new v2(5, 1),
				};
				var m = Monic.FromLinearRegression(pts);
				//Assert.True(Math_.FEql(m.A, 0.689393937587738));
				//Assert.True(Math_.FEql(m.B, -6.10151338577271));
			}
			{
				var pts = new v2[]
				{
					new v2(0,15),
					new v2(1,13),
					new v2(2,10),
					new v2(3, 7),
					new v2(4, 4),
					new v2(5, 1),
					new v2(6, 5),
					new v2(7, 8),
					new v2(8,13),
					new v2(9,19),
				};
				var q = Quadratic.FromLinearRegression(pts);
				Assert.True(Math_.FEql((float)q.A, 0.689394f));
				Assert.True(Math_.FEql((float)q.B, -6.10151672f));
				Assert.True(Math_.FEql((float)q.C, 17.3090973f));
			}
		}
	}
}
#endif