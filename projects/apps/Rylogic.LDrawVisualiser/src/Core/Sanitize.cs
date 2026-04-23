namespace Rylogic.LDrawVisualiser.Core
{
	using Rylogic.Maths;

	/// <summary>
	/// Replaces non-finite (NaN, +/-Infinity) components in float-bearing values with 0.
	/// Used by the type readers to guarantee that values returned to scripts always
	/// produce a parseable LDraw stream — uninitialised native memory or partially-
	/// constructed objects often contain non-finite floats that would otherwise crash
	/// the LDraw script parser. Callers should treat the sanitised result as best-effort
	/// "displayable" data, not as semantically meaningful when garbage was the input.
	/// </summary>
	internal static class Sanitize
	{
		// Single component — net481 doesn't have float.IsFinite, so spell it out.
		private static float F(float x) => float.IsNaN(x) || float.IsInfinity(x) ? 0f : x;
		private static double F(double x) => double.IsNaN(x) || double.IsInfinity(x) ? 0.0 : x;

		public static float Finite(float x) => F(x);
		public static double Finite(double x) => F(x);

		public static v2 Finite(v2 v) => new(F(v.x), F(v.y));
		public static v3 Finite(v3 v) => new(F(v.x), F(v.y), F(v.z));
		public static v4 Finite(v4 v) => new(F(v.x), F(v.y), F(v.z), F(v.w));

		// A zero-magnitude quaternion is degenerate (any subsequent normalise / rotate
		// will divide by zero). Substitute Identity so the LDraw stream stays usable.
		public static Quat Finite(Quat q)
		{
			var s = new Quat(F(q.x), F(q.y), F(q.z), F(q.w));
			var mag_sq = s.x * s.x + s.y * s.y + s.z * s.z + s.w * s.w;
			return mag_sq > 0f ? s : Quat.Identity;
		}

		public static m2x2 Finite(m2x2 m) => new(Finite(m.x), Finite(m.y));
		public static m3x3 Finite(m3x3 m) => new(Finite(m.x), Finite(m.y), Finite(m.z));
		public static m4x4 Finite(m4x4 m) => new(Finite(m.x), Finite(m.y), Finite(m.z), Finite(m.w));

		public static BBox Finite(BBox b) => new(Finite(b.Centre), Finite(b.Radius));
	}
}
