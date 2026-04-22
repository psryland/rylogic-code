namespace Rylogic.LDrawVisualiser.Core;

using EnvDTE;
using Microsoft.VisualStudio.Shell;

/// <summary>The debugger language family that determines how expressions are evaluated</summary>
public enum ELanguage
{
	Unknown,
	Native,  // C / C++
	Managed, // C#, VB, F#, etc.
}

/// <summary>Helpers for detecting the active debug session language</summary>
public static class LanguageDetector
{
	/// <summary>Determine the language of the currently selected stack frame</summary>
	public static ELanguage Detect(Debugger debugger)
	{
		ThreadHelper.ThrowIfNotOnUIThread();

		try
		{
			// Debugger.CurrentStackFrame.Language returns strings like "C++", "C#", "Basic", "F#".
			// It can throw or be null when no frame is selected (not in break mode).
			var frame = debugger.CurrentStackFrame;
			var lang = frame?.Language;
			if (string.IsNullOrEmpty(lang))
				return ELanguage.Unknown;

			// Native is the only language that supports the address-of trick used by RPM helpers.
			if (lang == "C++" || lang == "C")
				return ELanguage.Native;

			return ELanguage.Managed;
		}
		catch
		{
			return ELanguage.Unknown;
		}
	}
}
