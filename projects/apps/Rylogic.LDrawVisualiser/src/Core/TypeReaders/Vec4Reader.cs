namespace Rylogic.LDrawVisualiser.Core.TypeReaders;

using System.Text.RegularExpressions;
using EnvDTE;
using Rylogic.Maths;

/// <summary>
/// Reads a 4D float vector. Matches:
///  - Native: pr::math::Vec4&lt;float&gt; / pr::v4
///  - Managed: Rylogic.Maths.v4 / System.Numerics.Vector4
/// </summary>
public class Vec4Reader : ITypeReader
{
	private static readonly Regex NativePattern = new(@"pr::math::Vec4<float>|pr::v4\b");
	private static readonly Regex ManagedPattern = new(@"^(Rylogic\.Maths\.v4|System\.Numerics\.Vector4)$");

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
				// Native pr::Vec4<float> is alignas(16), four contiguous floats = 16 bytes.
				var data = DebugMemoryReader.ReadBytes(debugger, expr, 4 * sizeof(float));
				if (data == null) return null;
				var v = new float[4];
				for (var i = 0; i != 4; ++i)
					v[i] = System.BitConverter.ToSingle(data, i * sizeof(float));
				return new v4(v);
			}
			case ELanguage.Managed:
			{
				var (fx, fy, fz, fw) = type_name.StartsWith("System.Numerics") ? ("X", "Y", "Z", "W") : ("x", "y", "z", "w");
				var x = ManagedFieldReader.ReadSingle(debugger, $"{expr}.{fx}");
				var y = ManagedFieldReader.ReadSingle(debugger, $"{expr}.{fy}");
				var z = ManagedFieldReader.ReadSingle(debugger, $"{expr}.{fz}");
				var w = ManagedFieldReader.ReadSingle(debugger, $"{expr}.{fw}");
				if (x == null || y == null || z == null || w == null) return null;
				return new v4(x.Value, y.Value, z.Value, w.Value);
			}
			default:
			{
				throw new System.ArgumentOutOfRangeException(nameof(lang), lang, "Unsupported language");
			}
		}
	}
}
