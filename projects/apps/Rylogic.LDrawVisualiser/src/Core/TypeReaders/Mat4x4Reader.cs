namespace Rylogic.LDrawVisualiser.Core.TypeReaders;

using System.Text.RegularExpressions;
using EnvDTE;
using Rylogic.Maths;

/// <summary>
/// Reads a 4x4 float matrix. Matches:
///  - Native: pr::math::Mat4x4&lt;float&gt; / pr::m4x4 (4 columns of v4 = 64 bytes)
///  - Managed: Rylogic.Maths.m4x4 (same blittable layout) and System.Numerics.Matrix4x4
///    (row-major M11..M44 fields)
/// </summary>
public class Mat4x4Reader : ITypeReader
{
	private static readonly Regex NativePattern = new(@"pr::math::Mat4x4<float>|pr::m4x4\b");
	private static readonly Regex ManagedPattern = new(@"^(Rylogic\.Maths\.m4x4|System\.Numerics\.Matrix4x4)$");

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
				var data = DebugMemoryReader.ReadBytes(debugger, expr, 16 * sizeof(float));
				if (data == null) return null;
				var f = new float[16];
				for (var i = 0; i != 16; ++i)
					f[i] = System.BitConverter.ToSingle(data, i * sizeof(float));
				return Sanitize.Finite(new m4x4(f));
			}
			case ELanguage.Managed:
			{
				if (type_name == "System.Numerics.Matrix4x4")
				{
					// System.Numerics.Matrix4x4 is row-major: Mij where i=row, j=column.
					// Convert to our column-major v4 columns.
					float? F(int r, int c) => ManagedFieldReader.ReadSingle(debugger, $"{expr}.M{r}{c}");
					var c1 = (F(1, 1), F(2, 1), F(3, 1), F(4, 1));
					var c2 = (F(1, 2), F(2, 2), F(3, 2), F(4, 2));
					var c3 = (F(1, 3), F(2, 3), F(3, 3), F(4, 3));
					var c4 = (F(1, 4), F(2, 4), F(3, 4), F(4, 4));
					if (!HasAll(c1) || !HasAll(c2) || !HasAll(c3) || !HasAll(c4))
						return null;
					return Sanitize.Finite(new m4x4(
						new v4(c1.Item1!.Value, c1.Item2!.Value, c1.Item3!.Value, c1.Item4!.Value),
						new v4(c2.Item1!.Value, c2.Item2!.Value, c2.Item3!.Value, c2.Item4!.Value),
						new v4(c3.Item1!.Value, c3.Item2!.Value, c3.Item3!.Value, c3.Item4!.Value),
						new v4(c4.Item1!.Value, c4.Item2!.Value, c4.Item3!.Value, c4.Item4!.Value)));
				}

				// Rylogic.Maths.m4x4: v4 x, y, z, w columns.
				var x = ReadV4(debugger, $"{expr}.x");
				var y = ReadV4(debugger, $"{expr}.y");
				var z = ReadV4(debugger, $"{expr}.z");
				var w = ReadV4(debugger, $"{expr}.w");
				if (x == null || y == null || z == null || w == null) return null;
				return Sanitize.Finite(new m4x4(x.Value, y.Value, z.Value, w.Value));
			}
			default:
			{
				throw new System.ArgumentOutOfRangeException(nameof(lang), lang, "Unsupported language");
			}
		}
	}

	private static bool HasAll((float?, float?, float?, float?) t) =>
		t.Item1.HasValue && t.Item2.HasValue && t.Item3.HasValue && t.Item4.HasValue;

	private static v4? ReadV4(Debugger d, string expr)
	{
		var x = ManagedFieldReader.ReadSingle(d, $"{expr}.x");
		var y = ManagedFieldReader.ReadSingle(d, $"{expr}.y");
		var z = ManagedFieldReader.ReadSingle(d, $"{expr}.z");
		var w = ManagedFieldReader.ReadSingle(d, $"{expr}.w");
		if (x == null || y == null || z == null || w == null) return null;
		return new v4(x.Value, y.Value, z.Value, w.Value);
	}
}
