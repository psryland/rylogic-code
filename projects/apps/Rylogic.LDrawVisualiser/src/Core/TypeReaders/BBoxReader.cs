namespace Rylogic.LDrawVisualiser.Core.TypeReaders;

using System.Text.RegularExpressions;
using EnvDTE;
using Rylogic.Maths;

/// <summary>
/// Reads an axis-aligned bounding box. Matches:
///  - Native: pr::math::BoundingBox&lt;float&gt; / pr::BBox (Vec4 m_centre + Vec4 m_radius = 32 bytes)
///  - Managed: Rylogic.Maths.BBox (v4 Centre + v4 Radius)
/// </summary>
public class BBoxReader : ITypeReader
{
	private static readonly Regex NativePattern = new(@"pr::math::BoundingBox<float>|pr::BBox\b");
	private static readonly Regex ManagedPattern = new(@"^Rylogic\.Maths\.BBox$");

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
				// Native pr::BoundingBox<float> is two contiguous alignas(16) Vec4 = 32 bytes.
				var data = DebugMemoryReader.ReadBytes(debugger, expr, 8 * sizeof(float));
				if (data == null) return null;
				float F(int i) => System.BitConverter.ToSingle(data, i * sizeof(float));
				return Sanitize.Finite(new BBox(
					new v4(F(0), F(1), F(2), F(3)),
					new v4(F(4), F(5), F(6), F(7))));
			}
			case ELanguage.Managed:
			{
				// Rylogic.Maths.BBox uses v4 Centre / v4 Radius (PascalCase).
				var c = ReadV4(debugger, $"{expr}.Centre");
				var r = ReadV4(debugger, $"{expr}.Radius");
				if (c == null || r == null) return null;
				return Sanitize.Finite(new BBox(c.Value, r.Value));
			}
			default:
			{
				throw new System.ArgumentOutOfRangeException(nameof(lang), lang, "Unsupported language");
			}
		}
	}

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
