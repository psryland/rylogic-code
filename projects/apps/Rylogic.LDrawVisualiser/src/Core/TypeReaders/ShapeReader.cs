namespace Rylogic.LDrawVisualiser.Core.TypeReaders;

using System.Text.RegularExpressions;
using EnvDTE;

/// <summary>
/// Reads a pr::collision::Shape (or any derived shape) as a raw byte buffer.
/// Native-only: collision shapes are tightly-packed C++ structs that can only be
/// reliably extracted via ReadProcessMemory.
/// </summary>
public class ShapeReader : ITypeReader
{
	private static readonly Regex NativePattern = new(@"pr::collision::Shape");

	/// <inheritdoc/>
	public bool CanRead(ELanguage lang, string type_name) => lang switch
	{
		ELanguage.Native => NativePattern.IsMatch(type_name),
		ELanguage.Managed => false,
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
				return DebugMemoryReader.ReadShapeBytes(debugger, expr);
			}
			default:
			{
				throw new System.ArgumentOutOfRangeException(nameof(lang), lang, "ShapeReader supports native only");
			}
		}
	}
}
