namespace Rylogic.LDrawVisualiser.Core.TypeReaders;

using System.Text.RegularExpressions;
using EnvDTE;

/// <summary>Reads a boolean (native 'bool', managed 'bool'/'Boolean')</summary>
public class BoolReader : ITypeReader
{
	private static readonly Regex NativePattern = new(@"^bool$");
	private static readonly Regex ManagedPattern = new(@"^(bool|System\.Boolean|Boolean)$");

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
				return DebugMemoryReader.ReadBool(debugger, expr);
			}
			case ELanguage.Managed:
			{
				return ManagedFieldReader.ReadBool(debugger, expr);
			}
			default:
			{
				throw new System.ArgumentOutOfRangeException(nameof(lang), lang, "Unsupported language");
			}
		}
	}
}
