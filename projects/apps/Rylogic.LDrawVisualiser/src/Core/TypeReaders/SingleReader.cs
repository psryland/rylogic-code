namespace Rylogic.LDrawVisualiser.Core.TypeReaders;

using System.Text.RegularExpressions;
using EnvDTE;

/// <summary>Reads a single-precision float (native 'float', managed 'float'/'Single')</summary>
public class SingleReader : ITypeReader
{
	private static readonly Regex NativePattern = new(@"^float$");
	private static readonly Regex ManagedPattern = new(@"^(float|System\.Single|Single)$");

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
		float? raw = lang switch
		{
			ELanguage.Native => DebugMemoryReader.ReadSingle(debugger, expr),
			ELanguage.Managed => ManagedFieldReader.ReadSingle(debugger, expr),
			_ => throw new System.ArgumentOutOfRangeException(nameof(lang), lang, "Unsupported language"),
		};
		return raw.HasValue ? Sanitize.Finite(raw.Value) : (object?)null;
	}
}
