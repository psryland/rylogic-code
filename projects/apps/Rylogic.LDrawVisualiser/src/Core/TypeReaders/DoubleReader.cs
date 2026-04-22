namespace Rylogic.LDrawVisualiser.Core.TypeReaders;

using System.Text.RegularExpressions;
using EnvDTE;

/// <summary>Reads a double-precision float (native 'double', managed 'double'/'Double')</summary>
public class DoubleReader : ITypeReader
{
	private static readonly Regex NativePattern = new(@"^double$");
	private static readonly Regex ManagedPattern = new(@"^(double|System\.Double|Double)$");

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
				return DebugMemoryReader.ReadDouble(debugger, expr);
			}
			case ELanguage.Managed:
			{
				return ManagedFieldReader.ReadDouble(debugger, expr);
			}
			default:
			{
				throw new System.ArgumentOutOfRangeException(nameof(lang), lang, "Unsupported language");
			}
		}
	}
}
