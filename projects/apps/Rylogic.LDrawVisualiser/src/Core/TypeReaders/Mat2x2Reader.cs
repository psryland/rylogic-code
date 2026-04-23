namespace Rylogic.LDrawVisualiser.Core.TypeReaders;

using System.Text.RegularExpressions;
using EnvDTE;
using Rylogic.Maths;

/// <summary>
/// Reads a 2x2 float matrix. Matches:
///  - Native: pr::math::Mat2x2&lt;float&gt; / pr::m2x2
///  - Managed: Rylogic.Maths.m2x2
/// </summary>
public class Mat2x2Reader : ITypeReader
{
	private static readonly Regex NativePattern = new(@"pr::math::Mat2x2<float>|pr::m2x2\b");
	private static readonly Regex ManagedPattern = new(@"^Rylogic\.Maths\.m2x2$");

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
				// Native pr::Mat2x2<float> is two contiguous Vec2<float> = 16 bytes.
				var data = DebugMemoryReader.ReadBytes(debugger, expr, 4 * sizeof(float));
				if (data == null) return null;
				var x = new v2(System.BitConverter.ToSingle(data, 0), System.BitConverter.ToSingle(data, 4));
				var y = new v2(System.BitConverter.ToSingle(data, 8), System.BitConverter.ToSingle(data, 12));
				return Sanitize.Finite(new m2x2(x, y));
			}
			case ELanguage.Managed:
			{
				// Rylogic.Maths.m2x2 has fields v2 x; v2 y;
				var x_x = ManagedFieldReader.ReadSingle(debugger, $"{expr}.x.x");
				var x_y = ManagedFieldReader.ReadSingle(debugger, $"{expr}.x.y");
				var y_x = ManagedFieldReader.ReadSingle(debugger, $"{expr}.y.x");
				var y_y = ManagedFieldReader.ReadSingle(debugger, $"{expr}.y.y");
				if (x_x == null || x_y == null || y_x == null || y_y == null) return null;
				return Sanitize.Finite(new m2x2(new v2(x_x.Value, x_y.Value), new v2(y_x.Value, y_y.Value)));
			}
			default:
			{
				throw new System.ArgumentOutOfRangeException(nameof(lang), lang, "Unsupported language");
			}
		}
	}
}
