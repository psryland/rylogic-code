using System;
using System.Collections.Generic;
using System.Collections.Immutable;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Threading.Tasks;
using Microsoft.CodeAnalysis;
using Microsoft.CodeAnalysis.CSharp;
using Microsoft.CodeAnalysis.Emit;

namespace Rylogic.LDrawVisualiser.Core
{
	/// <summary>
	/// Compiles user C# script into a callable delegate.
	/// The script body is wrapped in a function: string Generate(dynamic vars) { ... }
	/// Compilation only happens when the script text changes.
	/// </summary>
	public class ScriptCompiler
	{
		/// <summary>The last successfully compiled delegate</summary>
		public Func<dynamic, string>? CompiledScript { get; private set; }

		/// <summary>The script text that was last successfully compiled</summary>
		public string? LastCompiledText { get; private set; }

		/// <summary>Diagnostics from the most recent compilation attempt</summary>
		public IReadOnlyList<Diagnostic> Diagnostics { get; private set; } = [];

		/// <summary>True if the last compilation attempt had errors</summary>
		public bool HasErrors => Diagnostics.Any(d => d.Severity == DiagnosticSeverity.Error);

		/// <summary>
		/// Compile the user script. Returns true if compilation succeeded.
		/// The script body is the content of a method: string Generate(dynamic vars) { [script] }
		/// </summary>
		public bool Compile(string script_body)
		{
			// Don't recompile if unchanged
			if (script_body == LastCompiledText && CompiledScript != null)
				return true;

			try
			{
				var (success, diagnostics, assembly) = CompileAssembly(script_body);
				Diagnostics = diagnostics;

				if (!success || assembly == null)
					return false;

				// Load and extract the delegate
				var type = assembly.GetType("Rylogic.LDrawVisualiser.Generated.ScriptHost");
				var method = type?.GetMethod("Generate", BindingFlags.Public | BindingFlags.Static);
				if (method == null)
				{
					Diagnostics = [Diagnostic.Create(
						new DiagnosticDescriptor("LDVS001", "Internal Error", "Failed to find Generate method in compiled assembly", "Compiler", DiagnosticSeverity.Error, true),
						Location.None)];
					return false;
				}

				CompiledScript = (Func<dynamic, string>)Delegate.CreateDelegate(typeof(Func<dynamic, string>), method);
				LastCompiledText = script_body;
				return true;
			}
			catch (Exception ex)
			{
				Diagnostics = [Diagnostic.Create(
					new DiagnosticDescriptor("LDVS002", "Compilation Exception", ex.Message, "Compiler", DiagnosticSeverity.Error, true),
					Location.None)];
				return false;
			}
		}

		private static (bool success, IReadOnlyList<Diagnostic> diagnostics, Assembly? assembly) CompileAssembly(string script_body)
		{
			// Wrap the user script in a class with a static Generate method
			var source = $@"
using System;
using System.Linq;
using Rylogic.LDraw;
using Rylogic.Maths;

namespace Rylogic.LDrawVisualiser.Generated
{{
	public static class ScriptHost
	{{
		public static string Generate(dynamic vars)
		{{
			{script_body}
		}}
	}}
}}";

			var syntax_tree = CSharpSyntaxTree.ParseText(source);

			// Gather references
			var references = new List<MetadataReference>();

			// Core framework references
			var trusted_assemblies = AppDomain.CurrentDomain.GetAssemblies()
				.Where(a => !a.IsDynamic && !string.IsNullOrEmpty(a.Location))
				.Select(a => a.Location)
				.Distinct();

			foreach (var path in trusted_assemblies)
			{
				try { references.Add(MetadataReference.CreateFromFile(path)); }
				catch { /* Skip assemblies that can't be loaded */ }
			}

			// Add Rylogic assemblies from the extension directory
			var ext_dir = Path.GetDirectoryName(typeof(ScriptCompiler).Assembly.Location) ?? "";
			foreach (var dll in new[] { "Rylogic.Core.dll", "Rylogic.Gfx.dll" })
			{
				var dll_path = Path.Combine(ext_dir, dll);
				if (File.Exists(dll_path))
					references.Add(MetadataReference.CreateFromFile(dll_path));
			}

			// Also add Microsoft.CSharp for dynamic support
			var csharp_asm = typeof(Microsoft.CSharp.RuntimeBinder.Binder).Assembly.Location;
			if (!string.IsNullOrEmpty(csharp_asm))
				references.Add(MetadataReference.CreateFromFile(csharp_asm));

			var compilation = CSharpCompilation.Create(
				assemblyName: "LDrawVisualiserScript",
				syntaxTrees: [syntax_tree],
				references: references,
				options: new CSharpCompilationOptions(OutputKind.DynamicallyLinkedLibrary));

			using var ms = new MemoryStream();
			var emit_result = compilation.Emit(ms);

			// Map diagnostics back to user script (offset the line numbers)
			var mapped_diagnostics = emit_result.Diagnostics
				.Where(d => d.Severity >= DiagnosticSeverity.Warning)
				.Select(d => MapDiagnosticToUserScript(d))
				.ToImmutableArray();

			if (!emit_result.Success)
				return (false, mapped_diagnostics, null);

			ms.Seek(0, SeekOrigin.Begin);
			var assembly = Assembly.Load(ms.ToArray());
			return (true, mapped_diagnostics, assembly);
		}

		/// <summary>Adjust line numbers to account for the wrapper code</summary>
		private static Diagnostic MapDiagnosticToUserScript(Diagnostic diagnostic)
		{
			// The wrapper adds 12 lines before the user script body
			const int wrapper_line_offset = 12;

			if (diagnostic.Location.IsInSource)
			{
				var span = diagnostic.Location.GetLineSpan();
				var adjusted_line = Math.Max(1, span.StartLinePosition.Line - wrapper_line_offset + 1);
				var col = span.StartLinePosition.Character + 1;
				var message = $"({adjusted_line},{col}): {diagnostic.GetMessage()}";

				// Create a new descriptor with the pre-formatted message to avoid format arg issues
				var desc = new DiagnosticDescriptor(
					diagnostic.Id,
					diagnostic.Descriptor.Title,
					messageFormat: message,
					diagnostic.Descriptor.Category,
					diagnostic.Severity,
					isEnabledByDefault: true);

				return Diagnostic.Create(desc, Location.None);
			}
			return diagnostic;
		}
	}
}
