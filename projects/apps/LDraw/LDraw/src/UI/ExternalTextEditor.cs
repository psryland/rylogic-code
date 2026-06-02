using System;
using System.Diagnostics;
using System.IO;

namespace LDraw.UI
{
	internal static class ExternalTextEditor
	{
		public const string LegacyArgumentsPattern = "\"{file}\" --goto {line}";
		public const string VisualStudioCodeArgumentsPattern = "--reuse-window --goto \"{file}:{line}\"";

		/// <summary>Launch the configured external text editor at 'file' and 'line'</summary>
		public static void Launch(string editor_path, string arguments_pattern, string file, int line)
		{
			if (string.IsNullOrWhiteSpace(editor_path))
				throw new ArgumentException("An editor path is required", nameof(editor_path));

			arguments_pattern ??= string.Empty;
			line = Math.Max(1, line);
			if (UseVisualStudioCodeArguments(editor_path, arguments_pattern))
			{
				var start_info = new ProcessStartInfo
				{
					FileName = editor_path,
					UseShellExecute = false,
				};
				start_info.ArgumentList.Add("--reuse-window");
				start_info.ArgumentList.Add("--goto");
				start_info.ArgumentList.Add($"{file}:{line}");

				if (Process.Start(start_info) == null)
					throw new InvalidOperationException($"Failed to launch text editor '{editor_path}'");

				return;
			}

			var arguments = arguments_pattern
				.Replace("{file}", file)
				.Replace("{line}", line.ToString());

			if (Process.Start(new ProcessStartInfo
			{
				FileName = editor_path,
				Arguments = arguments,
				UseShellExecute = false,
			}) == null)
				throw new InvalidOperationException($"Failed to launch text editor '{editor_path}'");
		}

		/// <summary>True if 'editor_path' is Visual Studio Code</summary>
		private static bool IsVisualStudioCode(string editor_path)
		{
			return string.Equals(Path.GetFileName(editor_path), "Code.exe", StringComparison.OrdinalIgnoreCase);
		}

		/// <summary>True if legacy/default arguments should be replaced with VS Code's goto syntax</summary>
		private static bool UseVisualStudioCodeArguments(string editor_path, string arguments_pattern)
		{
			if (!IsVisualStudioCode(editor_path))
				return false;

			var pattern = arguments_pattern.Trim();
			return
				pattern.Length == 0 ||
				string.Equals(pattern, LegacyArgumentsPattern, StringComparison.OrdinalIgnoreCase) ||
				string.Equals(pattern, VisualStudioCodeArgumentsPattern, StringComparison.OrdinalIgnoreCase);
		}
	}
}
