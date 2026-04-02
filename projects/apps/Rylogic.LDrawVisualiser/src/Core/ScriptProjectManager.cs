using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using Rylogic.Utility;

namespace Rylogic.LDrawVisualiser.Core
{
	/// <summary>Info about a script file displayed in the scripts list</summary>
	public class ScriptInfo
	{
		public ScriptInfo(string filepath)
		{
			FilePath = filepath;
			Name = Path.GetFileNameWithoutExtension(filepath);
		}

		public string Name { get; }
		public string FilePath { get; }

		public override string ToString() => Name;
	}

	/// <summary>
	/// Manages the scripts directory, including generating a .csproj that references
	/// Rylogic.Gfx so that VS provides full C# IntelliSense when scripts are opened as editor tabs.
	/// </summary>
	public class ScriptProjectManager
	{
		private readonly LDrawVisualiserOptions m_options;

		public ScriptProjectManager(LDrawVisualiserOptions options)
		{
			m_options = options;
		}

		/// <summary>The scripts directory</summary>
		public string ScriptsDirectory => m_options.ScriptsDirectory;

		/// <summary>The path to the generated .csproj file</summary>
		public string ProjectFilePath => Path.Combine(ScriptsDirectory, "LDrawScripts.csproj");

		/// <summary>Ensure the scripts directory and project file exist</summary>
		public void EnsureProjectExists()
		{
			Directory.CreateDirectory(ScriptsDirectory);

			// Always regenerate the .csproj to pick up assembly reference changes
			GenerateProjectFile();

			// Ensure there's a global usings file
			EnsureGlobalUsings();
		}

		/// <summary>Get all script files in the directory</summary>
		public List<ScriptInfo> GetScripts()
		{
			if (!Directory.Exists(ScriptsDirectory))
				return new List<ScriptInfo>();

			return Directory.GetFiles(ScriptsDirectory, "*.cs")
				.Where(f => !Path.GetFileName(f).StartsWith("_")) // Skip helper files like _GlobalUsings.cs
				.OrderBy(f => Path.GetFileNameWithoutExtension(f))
				.Select(f => new ScriptInfo(f))
				.ToList();
		}

		/// <summary>Create a new script file with the default template</summary>
		public ScriptInfo CreateScript(string name)
		{
			EnsureProjectExists();
			var filepath = Path.Combine(ScriptsDirectory, $"{name}.cs");

			// Generate the script content as a complete valid C# file
			var content = GenerateScriptContent(name, m_options.DefaultScriptText);
			File.WriteAllText(filepath, content);

			return new ScriptInfo(filepath);
		}

		/// <summary>Delete a script file</summary>
		public void DeleteScript(ScriptInfo script)
		{
			if (File.Exists(script.FilePath))
				File.Delete(script.FilePath);
		}

		/// <summary>
		/// Read the script body from a file.
		/// Extracts just the body of the Generate method for compilation.
		/// </summary>
		public string ReadScriptBody(ScriptInfo script)
		{
			if (!File.Exists(script.FilePath))
				return m_options.DefaultScriptText;

			try
			{
				var content = File.ReadAllText(script.FilePath);
				return ExtractMethodBody(content) ?? content;
			}
			catch
			{
				return m_options.DefaultScriptText;
			}
		}

		/// <summary>Generate a complete C# file that VS can provide IntelliSense for</summary>
		public static string GenerateScriptContent(string name, string body)
		{
			// Indent the body
			var indented_body = string.Join("\n",
				body.Split('\n').Select(line => $"\t\t\t{line.TrimEnd()}"));

			return $@"using System;
using System.Linq;
using Rylogic.LDraw;
using Rylogic.Maths;

namespace Rylogic.LDrawVisualiser.Scripts
{{
	public static class {SanitizeIdentifier(name)}
	{{
		public static string Generate(dynamic vars)
		{{
{indented_body}
		}}
	}}
}}
";
		}

		/// <summary>Extract the body of the Generate method from a complete C# file</summary>
		private static string? ExtractMethodBody(string content)
		{
			// Find the Generate method signature
			var marker = "public static string Generate(dynamic vars)";
			var idx = content.IndexOf(marker, StringComparison.Ordinal);
			if (idx < 0) return null;

			// Find the opening brace of the method
			var brace_start = content.IndexOf('{', idx + marker.Length);
			if (brace_start < 0) return null;

			// Find the matching closing brace
			var depth = 1;
			var pos = brace_start + 1;
			while (pos < content.Length && depth > 0)
			{
				if (content[pos] == '{') depth++;
				else if (content[pos] == '}') depth--;
				pos++;
			}

			if (depth != 0) return null;

			// Extract and dedent the body
			var body = content.Substring(brace_start + 1, pos - brace_start - 2);
			return DedentBody(body);
		}

		/// <summary>Remove common leading whitespace from all lines</summary>
		private static string DedentBody(string text)
		{
			var lines = text.Split('\n')
				.Select(l => l.TrimEnd('\r'))
				.ToList();

			// Remove leading/trailing empty lines
			while (lines.Count > 0 && string.IsNullOrWhiteSpace(lines[0])) lines.RemoveAt(0);
			while (lines.Count > 0 && string.IsNullOrWhiteSpace(lines[lines.Count - 1])) lines.RemoveAt(lines.Count - 1);

			if (lines.Count == 0) return "";

			// Find minimum indentation
			var min_indent = lines
				.Where(l => !string.IsNullOrWhiteSpace(l))
				.Select(l => l.Length - l.TrimStart().Length)
				.DefaultIfEmpty(0)
				.Min();

			return string.Join("\n", lines.Select(l =>
				l.Length >= min_indent ? l.Substring(min_indent) : l.TrimStart()));
		}

		private static string SanitizeIdentifier(string name)
		{
			var sb = new StringBuilder();
			foreach (var ch in name)
			{
				if (char.IsLetterOrDigit(ch) || ch == '_')
					sb.Append(ch);
				else
					sb.Append('_');
			}
			if (sb.Length == 0 || char.IsDigit(sb[0]))
				sb.Insert(0, '_');
			return sb.ToString();
		}

		/// <summary>Generate the .csproj that provides IntelliSense context</summary>
		private void GenerateProjectFile()
		{
			// Find the Rylogic assembly paths from the extension directory
			var ext_dir = Path.GetDirectoryName(typeof(ScriptProjectManager).Assembly.Location) ?? "";
			var rylogic_core = Path.Combine(ext_dir, "Rylogic.Core.dll");
			var rylogic_gfx = Path.Combine(ext_dir, "Rylogic.Gfx.dll");

			var references = new StringBuilder();

			// Rylogic assemblies
			if (File.Exists(rylogic_core))
				references.AppendLine($"\t\t<Reference Include=\"Rylogic.Core\"><HintPath>{rylogic_core}</HintPath></Reference>");
			if (File.Exists(rylogic_gfx))
				references.AppendLine($"\t\t<Reference Include=\"Rylogic.Gfx\"><HintPath>{rylogic_gfx}</HintPath></Reference>");

			// User-specified additional assemblies
			foreach (var asm in m_options.Assemblies)
			{
				if (File.Exists(asm))
				{
					var name = Path.GetFileNameWithoutExtension(asm);
					references.AppendLine($"\t\t<Reference Include=\"{name}\"><HintPath>{asm}</HintPath></Reference>");
				}
			}

			var csproj = $@"<Project Sdk=""Microsoft.NET.Sdk"">
	<PropertyGroup>
		<TargetFramework>net481</TargetFramework>
		<LangVersion>latest</LangVersion>
		<Nullable>enable</Nullable>
		<OutputType>Library</OutputType>
		<!-- Prevent this project from building or interfering with the solution -->
		<BuildProjectReferences>false</BuildProjectReferences>
	</PropertyGroup>
	<ItemGroup>
		<PackageReference Include=""Microsoft.CSharp"" Version=""4.7.0"" />
	</ItemGroup>
	<ItemGroup>
{references}	</ItemGroup>
</Project>
";
			File.WriteAllText(ProjectFilePath, csproj);
		}

		/// <summary>Generate a _GlobalUsings.cs file for user-specified namespaces</summary>
		private void EnsureGlobalUsings()
		{
			var filepath = Path.Combine(ScriptsDirectory, "_GlobalUsings.cs");
			var namespaces = m_options.Namespaces
				.Split(new[] { '\n', '\r' }, StringSplitOptions.RemoveEmptyEntries)
				.Select(ns => ns.Trim())
				.Where(ns => ns.Length > 0);

			var sb = new StringBuilder();
			sb.AppendLine("// Auto-generated by LDraw Visualiser. Additional using directives from options.");
			foreach (var ns in namespaces)
				sb.AppendLine($"global using {ns};");

			File.WriteAllText(filepath, sb.ToString());
		}
	}
}
