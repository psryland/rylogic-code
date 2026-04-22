namespace Rylogic.LDrawVisualiser.Core.TypeReaders;

using System.Text.RegularExpressions;
using EnvDTE;

/// <summary>
/// Reads a managed string. Native string handling is intentionally not implemented —
/// std::string / std::wstring layouts vary between STL implementations and SSO states,
/// so they cannot reliably be reconstructed via RPM.
/// </summary>
public class StringReader : ITypeReader
{
	private static readonly Regex ManagedPattern = new(@"^(string|System\.String|String)$");

	/// <inheritdoc/>
	public bool CanRead(ELanguage lang, string type_name) => lang switch
	{
		ELanguage.Managed => ManagedPattern.IsMatch(type_name),
		ELanguage.Native => false,
		ELanguage.Unknown => false,
		_ => throw new System.ArgumentOutOfRangeException(nameof(lang), lang, "Unsupported language"),
	};

	/// <inheritdoc/>
	public object? Read(Debugger debugger, ELanguage lang, string type_name, string expr)
	{
		switch (lang)
		{
			case ELanguage.Managed:
			{
				return ManagedFieldReader.ReadString(debugger, expr);
			}
			default:
			{
				throw new System.ArgumentOutOfRangeException(nameof(lang), lang, "StringReader supports managed only");
			}
		}
	}
}
