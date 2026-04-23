namespace Rylogic.LDrawVisualiser.Core
{
	using System;
	using Rylogic.Maths;

	/// <summary>
	/// Sensible default values for proxy → typed-value conversions when no real value is
	/// available (debug symbol undefined, no debugger attached, no reader for the type, ...).
	/// Returning a default is preferable to letting the DLR throw RuntimeBinderException —
	/// scripts that gate optional rendering on null-checks (vars.foo != null) only protect
	/// the rendering, not the argument-binding path that would otherwise fail first.
	///
	/// Note on overload ambiguity: when a method has multiple overloads accepting types
	/// for which TryConvert succeeds (e.g. pos(v4) and pos(v3)), the DLR reports "best
	/// overloaded match has some invalid arguments" because it cannot pick between
	/// equally-applicable candidates. The script must disambiguate with an explicit cast
	/// at the call site, e.g. `.pos((v4)vars.seg_pt)`.
	/// </summary>
	internal static class Defaults
	{
		/// <summary>Best-effort default value for 'type'. Always succeeds.</summary>
		/// <param name="type">The target conversion type</param>
		/// <param name="result">Receives the default value (may be null for non-value reference types other than string)</param>
		/// <returns>True (always) — caller can wire straight into TryConvert.</returns>
		public static bool TryGet(Type type, out object? result)
		{
			// Common Rylogic.Maths types — return identity / zero so chained operations
			// (Inverse(), .pos, etc.) don't blow up on the result.
			if (type == typeof(string)) { result = string.Empty; return true; }
			if (type == typeof(m4x4)) { result = m4x4.Identity; return true; }
			if (type == typeof(m3x3)) { result = m3x3.Identity; return true; }
			if (type == typeof(m2x2)) { result = m2x2.Identity; return true; }
			if (type == typeof(Quat)) { result = Quat.Identity; return true; }
			if (type == typeof(v4)) { result = v4.Zero; return true; }
			if (type == typeof(v3)) { result = v3.Zero; return true; }
			if (type == typeof(v2)) { result = v2.Zero; return true; }

			// Any other value type: default(T) via Activator. Covers all primitives
			// (int, float, bool, ...) and any user value-type the script targets.
			if (type.IsValueType)
			{
				result = Activator.CreateInstance(type);
				return true;
			}

			// Reference types fall back to null. The DLR accepts null for any reference
			// parameter, so this still satisfies argument binding.
			result = null;
			return true;
		}
	}
}
