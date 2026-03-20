//***************************************************
// Math
//  Copyright (c) Rylogic Ltd 2008
//***************************************************

namespace Rylogic.Maths
{
	/// <summary>scalar functions</summary>
	public static partial class Math_
	{
		public const float TinyF = 1.00000007e-05f;
		public const float TinySqF = 1.00000015e-10f;
		public const float TinySqrtF = 3.16227786e-03f;

		public const double TinyD = 1.0000000000000002e-12;
		public const double TinySqD = 1.0000000000000003e-24;
		public const double TinySqrtD = 1.0000000000000002e-06;

		public const double Tau = 6.283185307179586476925286766559; // circle constant
		public const double InvTau = 1.0 / Tau;
		public const double TauBy2 = Tau / 2.0;
		public const double TauBy3 = Tau / 3.0;
		public const double TauBy4 = Tau / 4.0;
		public const double TauBy5 = Tau / 5.0;
		public const double TauBy6 = Tau / 6.0;
		public const double TauBy7 = Tau / 7.0;
		public const double TauBy8 = Tau / 8.0;
		public const double TauBy10 = Tau / 10.0;
		public const double TauBy16 = Tau / 16.0;
		public const double TauBy32 = Tau / 32.0;
		public const double TauBy360 = Tau / 360.0;
		public const double _360ByTau = 360.0 / Tau;
		public const double Root2 = 1.4142135623730950488016887242097;
		public const double Root3 = 1.7320508075688772935274463415059;
		public const double Root5 = 2.236067977499789696409173668731276;
		public const double InvRoot2 = 1.0 / 1.4142135623730950488016887242097;
		public const double InvRoot3 = 1.0 / 1.7320508075688772935274463415059;
		public const double GoldenRatio = 1.618033988749894848204586834; // "Golden Ratio" = (1 + sqrt(5))/2
		public const double GoldenAngle = 2.39996322972865332223; // (rad) "Golden Angle" = pi*(3-root(5)) = the angle the divides the circumference of a circle with ratio equal to the golden ratio
		public const double Phi = GoldenRatio;

		public const float TauF = (float)Tau;
		public const float InvTauF = (float)InvTau;
		public const float TauBy2F = (float)TauBy2;
		public const float TauBy3F = (float)TauBy3;
		public const float TauBy4F = (float)TauBy4;
		public const float TauBy5F = (float)TauBy5;
		public const float TauBy6F = (float)TauBy6;
		public const float TauBy7F = (float)TauBy7;
		public const float TauBy8F = (float)TauBy8;
		public const float TauBy10F = (float)TauBy10;
		public const float TauBy16F = (float)TauBy16;
		public const float TauBy32F = (float)TauBy32;
		public const float TauBy360F = (float)TauBy360;
		public const float _360ByTauF = (float)_360ByTau;
		public const float Root2F = (float)Root2;
		public const float Root3F = (float)Root3;
		public const float Root5F = (float)Root5;
		public const float InvRoot2F = (float)InvRoot2;
		public const float InvRoot3F = (float)InvRoot3;
		public const float GoldenRatioF = (float)GoldenRatio;
		public const float GoldenAngleF = (float)GoldenAngle;
		public const float PhiF = GoldenRatioF;

		// SI unit conversion
		public static double ident(double x) => x;
		public static double ms_to_s(double x) => x / 1000.0;
		public static double s_to_ms(double x) => x * 1000.0;
		public static double um_to_m(double x) => x / 1000000.0;
		public static double m_to_um(double x) => x * 1000000.0;
		public static double mm_to_m(double x) => x / 1000.0;
		public static double m_to_mm(double x) => x * 1000.0;
		public static double um_to_mm(double x) => x / 1000.0;
		public static double mm_to_um(double x) => x * 1000.0;
		public static double l00um_to_m(double x) => x / 10000.0;
		public static double m_to_100um(double x) => x * 10000.0;
		public static double l00um_to_mm(double x) => x / 10.0;
		public static double mm_to_100um(double x) => x * 10.0;
		public static double um_to_100um(double x) => x / 100.0;
		public static double l00um_to_um(double x) => x * 100.0;
		public static double inches_to_m(double x) => x / 39.3701;
		public static double m_to_inches(double x) => x * 39.3701;
	}
}
