namespace Rylogic.LDrawVisualiser.Core.TypeReaders;

using System.Text.RegularExpressions;
using EnvDTE;
using Rylogic.Maths;

/// <summary>
/// Reads a quaternion. Matches:
///  - Native: pr::math::Quat&lt;float&gt; / pr::quat
///  - Managed: Rylogic.Maths.Quat / System.Numerics.Quaternion
/// </summary>
public class QuatReader : ITypeReader
{
	private static readonly Regex NativePattern = new(@"pr::math::Quat<float>|pr::quat\b");
	private static readonly Regex ManagedPattern = new(@"^(Rylogic\.Maths\.Quat|System\.Numerics\.Quaternion)$");

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
				// Native pr::Quat<float> is alignas(16), four contiguous floats = 16 bytes.
				var data = DebugMemoryReader.ReadBytes(debugger, expr, 4 * sizeof(float));
				if (data == null) return null;
				return new Quat(
					System.BitConverter.ToSingle(data, 0),
					System.BitConverter.ToSingle(data, 4),
					System.BitConverter.ToSingle(data, 8),
					System.BitConverter.ToSingle(data, 12));
			}
			case ELanguage.Managed:
			{
				var (fx, fy, fz, fw) = type_name.StartsWith("System.Numerics") ? ("X", "Y", "Z", "W") : ("x", "y", "z", "w");
				var x = ManagedFieldReader.ReadSingle(debugger, $"{expr}.{fx}");
				var y = ManagedFieldReader.ReadSingle(debugger, $"{expr}.{fy}");
				var z = ManagedFieldReader.ReadSingle(debugger, $"{expr}.{fz}");
				var w = ManagedFieldReader.ReadSingle(debugger, $"{expr}.{fw}");
				if (x == null || y == null || z == null || w == null) return null;
				return new Quat(x.Value, y.Value, z.Value, w.Value);
			}
			default:
			{
				throw new System.ArgumentOutOfRangeException(nameof(lang), lang, "Unsupported language");
			}
		}
	}
}
