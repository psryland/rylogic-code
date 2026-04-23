namespace Rylogic.LDrawVisualiser.Core.TypeReaders;

using System.Text.RegularExpressions;
using EnvDTE;
using Rylogic.Maths;

/// <summary>
/// Reads a 3x3 float matrix. Matches:
///  - Native: pr::math::Mat3x3&lt;float&gt; / pr::m3x3 (3 columns of v3 + padding float = 48 bytes)
///  - Managed: Rylogic.Maths.m3x3 (same blittable layout)
/// </summary>
public class Mat3x3Reader : ITypeReader
{
	private static readonly Regex NativePattern = new(@"pr::math::Mat3x3<float>|pr::m3x3\b");
	private static readonly Regex ManagedPattern = new(@"^Rylogic\.Maths\.m3x3$");

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
				// Layout: { Vec3 x; float xw; Vec3 y; float yw; Vec3 z; float zw; } = 48 bytes.
				// We use the v4 ctor to preserve the padding floats so round-trips are bit-exact.
				var data = DebugMemoryReader.ReadBytes(debugger, expr, 12 * sizeof(float));
				if (data == null) return null;
				float F(int i) => System.BitConverter.ToSingle(data, i * sizeof(float));
				return Sanitize.Finite(new m3x3(
					new v4(F(0), F(1), F(2), F(3)),
					new v4(F(4), F(5), F(6), F(7)),
					new v4(F(8), F(9), F(10), F(11))));
			}
			case ELanguage.Managed:
			{
				// Rylogic.Maths.m3x3 has fields: v3 x, float xw, v3 y, float yw, v3 z, float zw.
				var x = ReadV3(debugger, $"{expr}.x");
				var xw = ManagedFieldReader.ReadSingle(debugger, $"{expr}.xw");
				var y = ReadV3(debugger, $"{expr}.y");
				var yw = ManagedFieldReader.ReadSingle(debugger, $"{expr}.yw");
				var z = ReadV3(debugger, $"{expr}.z");
				var zw = ManagedFieldReader.ReadSingle(debugger, $"{expr}.zw");
				if (x == null || y == null || z == null || xw == null || yw == null || zw == null)
					return null;

				return Sanitize.Finite(new m3x3(
					new v4(x.Value, xw.Value),
					new v4(y.Value, yw.Value),
					new v4(z.Value, zw.Value)));
			}
			default:
			{
				throw new System.ArgumentOutOfRangeException(nameof(lang), lang, "Unsupported language");
			}
		}
	}

	private static v3? ReadV3(Debugger d, string expr)
	{
		var x = ManagedFieldReader.ReadSingle(d, $"{expr}.x");
		var y = ManagedFieldReader.ReadSingle(d, $"{expr}.y");
		var z = ManagedFieldReader.ReadSingle(d, $"{expr}.z");
		if (x == null || y == null || z == null) return null;
		return new v3(x.Value, y.Value, z.Value);
	}
}
