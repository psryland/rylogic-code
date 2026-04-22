namespace Rylogic.LDrawVisualiser.Core.TypeReaders;

using System.Text.RegularExpressions;
using EnvDTE;
using Rylogic.Maths;

/// <summary>
/// Reads a 3D float vector. Matches:
///  - Native: pr::math::Vec3&lt;float&gt; / pr::v3
///  - Managed: Rylogic.Maths.v3 / System.Numerics.Vector3
/// </summary>
public class Vec3Reader : ITypeReader
{
	private static readonly Regex NativePattern = new(@"pr::math::Vec3<float>|pr::v3\b");
	private static readonly Regex ManagedPattern = new(@"^(Rylogic\.Maths\.v3|System\.Numerics\.Vector3)$");

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
				// Native pr::Vec3<float> is three contiguous floats (alignas 4).
				var data = DebugMemoryReader.ReadBytes(debugger, expr, 3 * sizeof(float));
				if (data == null) return null;
				return new v3(
					System.BitConverter.ToSingle(data, 0),
					System.BitConverter.ToSingle(data, 4),
					System.BitConverter.ToSingle(data, 8));
			}
			case ELanguage.Managed:
			{
				var (fx, fy, fz) = type_name.StartsWith("System.Numerics") ? ("X", "Y", "Z") : ("x", "y", "z");
				var x = ManagedFieldReader.ReadSingle(debugger, $"{expr}.{fx}");
				var y = ManagedFieldReader.ReadSingle(debugger, $"{expr}.{fy}");
				var z = ManagedFieldReader.ReadSingle(debugger, $"{expr}.{fz}");
				if (x == null || y == null || z == null) return null;
				return new v3(x.Value, y.Value, z.Value);
			}
			default:
			{
				throw new System.ArgumentOutOfRangeException(nameof(lang), lang, "Unsupported language");
			}
		}
	}
}
