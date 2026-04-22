namespace Rylogic.LDrawVisualiser.Core;

using EnvDTE;

/// <summary>
/// Reads a typed value from a debug expression. Implementations handle one logical
/// type (e.g. v4) and may match both native and managed type names.
/// </summary>
public interface ITypeReader
{
	/// <summary>Returns true if this reader can handle the given type in the given language</summary>
	bool CanRead(ELanguage lang, string type_name);

	/// <summary>Read the value at 'expr'. Returns null if the read fails or is unsupported</summary>
	object? Read(Debugger debugger, ELanguage lang, string type_name, string expr);
}
