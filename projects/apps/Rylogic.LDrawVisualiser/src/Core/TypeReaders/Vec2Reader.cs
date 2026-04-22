namespace Rylogic.LDrawVisualiser.Core.TypeReaders;

using System.Text.RegularExpressions;
using EnvDTE;
using Rylogic.Maths;

/// <summary>
/// Reads a 2D float vector. Matches:
///  - Native: pr::math::Vec2&lt;float&gt; / pr::v2
///  - Managed: Rylogic.Maths.v2 / System.Numerics.Vector2
/// </summary>
public class Vec2Reader : ITypeReader
{
	private static readonly Regex NativePattern = new(@"pr::math::Vec2<float>|pr::v2\b");
	private static readonly Regex ManagedPattern = new(@"^(Rylogic\.Maths\.v2|System\.Numerics\.Vector2)$");

	/// <inheritdoc/>
	public bool CanRead(ELanguage lang, string type_name) => lang switch
	{
		ELanguage.Native => NativePattern.IsMatch(type_name),
		ELanguage.Managed => ManagedPattern.IsMatch(type_name),
		ELanguage.Unknown => false,
		_ => throw new System.ArgumentOutOfRangeException(nameof(lang), lang, "Unsupported language"),
	};

	/// <inheritdoc/>
	public object? Read(Debugger debugger, ELanguage lang, string type_name, string expr)
	{
		switch (lang)
		{
			case ELanguage.Native:
			{
				// Native pr::Vec2<float> is two contiguous floats (alignas 4).
				var data = DebugMemoryReader.ReadBytes(debugger, expr, 2 * sizeof(float));
				if (data == null) return null;
				return new v2(System.BitConverter.ToSingle(data, 0), System.BitConverter.ToSingle(data, 4));
			}
			case ELanguage.Managed:
			{
				// System.Numerics.Vector2 uses uppercase X/Y; Rylogic.Maths.v2 uses lowercase x/y.
				var (fx, fy) = type_name.StartsWith("System.Numerics") ? ("X", "Y") : ("x", "y");
				var x = ManagedFieldReader.ReadSingle(debugger, $"{expr}.{fx}");
				var y = ManagedFieldReader.ReadSingle(debugger, $"{expr}.{fy}");
				if (x == null || y == null) return null;
				return new v2(x.Value, y.Value);
			}
			default:
			{
				throw new System.ArgumentOutOfRangeException(nameof(lang), lang, "Unsupported language");
			}
		}
	}
}
