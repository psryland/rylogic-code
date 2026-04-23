using System;
using System.Collections.Generic;
using System.Collections.Immutable;
using System.IO;
using System.Linq;
using System.Reflection;
using Microsoft.CodeAnalysis;
using Microsoft.CodeAnalysis.CSharp;

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
		/// Additional using directives from the .csx file are included.
		/// 'nuget_refs' lets the script pull a specific (or latest-installed) version of
		/// a Rylogic library from the user's NuGet cache instead of the VSIX-bundled DLL.
		/// </summary>
		public bool Compile(string script_body, IEnumerable<string>? extra_usings = null, IEnumerable<NuGetReference>? nuget_refs = null)
		{
			// Don't recompile if unchanged
			if (script_body == LastCompiledText && CompiledScript != null)
				return true;

			try
			{
				var (success, diagnostics, assembly) = CompileAssembly(script_body, extra_usings, nuget_refs);
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

		private static (bool success, IReadOnlyList<Diagnostic> diagnostics, Assembly? assembly) CompileAssembly(string script_body, IEnumerable<string>? extra_usings, IEnumerable<NuGetReference>? nuget_refs)
		{
			// Build using directives
			var usings = new List<string>
			{
				"using System;",
				"using System.Linq;",
				"using Rylogic.LDraw;",
				"using Rylogic.Maths;",
			};
			if (extra_usings != null)
			{
				foreach (var u in extra_usings)
				{
					var directive = u.EndsWith(";") ? u : $"{u};";
					if (!usings.Contains(directive))
						usings.Add(directive);
				}
			}

			var usings_block = string.Join("\n", usings);
			var usings_line_count = usings.Count;

			// Wrap the user script in a class with a static Generate method
			var source = $@"{usings_block}

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

			// Gather references. Strategy:
			//   1. Resolve any '#r "nuget: Id, Ver"' directives against the user's NuGet
			//      cache first. These take precedence so the script binds to the metadata
			//      the user has actually installed locally.
			//   2. Pull in everything currently loaded by devenv (framework, VS shell, etc.)
			//      EXCEPT the simple-names we already resolved from NuGet — otherwise we'd
			//      have two metadata references for the same assembly and Roslyn would
			//      complain about ambiguous types.
			//   3. Fill in any remaining requested packages from the VSIX-bundled DLLs in
			//      the extension directory (so the script still works when the user hasn't
			//      restored the package locally).
			//   4. Always fall back to the bundled Rylogic.Core.dll / Rylogic.Gfx.dll for
			//      back-compat with scripts that don't list any '#r nuget:' lines.
			//
			// Caveat: at runtime the CLR resolves by simple name. If devenv has already
			// loaded Rylogic.Gfx (e.g. because the VSIX itself uses it) the script's calls
			// dispatch to that loaded version, not the cache one. New APIs added in a
			// newer cache version will throw MissingMethodException at call time. To pick
			// up runtime API changes you still need to rebuild + reinstall the VSIX and
			// restart VS.
			var references = new List<MetadataReference>();
			var added_simple_names = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
			var resolution_diagnostics = new List<Diagnostic>();

			void AddReference(string path, string simple_name)
			{
				if (added_simple_names.Contains(simple_name))
					return;
				try
				{
					references.Add(MetadataReference.CreateFromFile(path));
					added_simple_names.Add(simple_name);
				}
				catch
				{
					// Skip assemblies that can't be loaded (corrupt, locked, native, ...).
				}
			}

			// 1. NuGet-resolved packages
			var requested_ids = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
			if (nuget_refs != null)
			{
				foreach (var nref in nuget_refs)
				{
					requested_ids.Add(nref.Id);

					var resolved = NuGetResolver.Resolve(nref.Id, nref.Version);
					if (resolved != null)
					{
						AddReference(resolved, nref.Id);
					}
					else
					{
						// Informational diagnostic — the bundled fallback in step 3 may
						// still satisfy the reference. The visualiser surfaces these in
						// the error panel so the user knows why a package wasn't picked up.
						var msg = $"NuGet package '{nref}' not found in local cache ({NuGetResolver.CacheRoot}); attempting bundled fallback.";
						resolution_diagnostics.Add(Diagnostic.Create(
							new DiagnosticDescriptor("LDVS003", "NuGet Resolution", msg, "NuGet", DiagnosticSeverity.Warning, true),
							Location.None));
					}
				}
			}

			// 2. Currently-loaded assemblies (framework / VS shell / etc.) — but skip any
			//    simple-name we've resolved from NuGet so we don't get duplicate types.
			var trusted_assemblies = AppDomain.CurrentDomain.GetAssemblies()
				.Where(a => !a.IsDynamic && !string.IsNullOrEmpty(a.Location))
				.Select(a => new { a.Location, SimpleName = a.GetName().Name ?? "" })
				.Where(a => !added_simple_names.Contains(a.SimpleName))
				.GroupBy(a => a.SimpleName)
				.Select(g => g.First());

			foreach (var asm in trusted_assemblies)
				AddReference(asm.Location, asm.SimpleName);

			// 3. Bundled fallback for any explicitly-requested package not yet resolved.
			var ext_dir = Path.GetDirectoryName(typeof(ScriptCompiler).Assembly.Location) ?? "";
			foreach (var id in requested_ids)
			{
				if (added_simple_names.Contains(id))
					continue;
				var dll_path = Path.Combine(ext_dir, $"{id}.dll");
				if (File.Exists(dll_path))
					AddReference(dll_path, id);
			}

			// 4. Always add the bundled Rylogic core/gfx DLLs as a back-compat default
			//    for scripts that don't declare any #r nuget directives.
			foreach (var dll in new[] { "Rylogic.Core.dll", "Rylogic.Gfx.dll" })
			{
				var simple = Path.GetFileNameWithoutExtension(dll);
				if (added_simple_names.Contains(simple))
					continue;
				var dll_path = Path.Combine(ext_dir, dll);
				if (File.Exists(dll_path))
					AddReference(dll_path, simple);
			}

			// Microsoft.CSharp for dynamic support
			var csharp_asm = typeof(Microsoft.CSharp.RuntimeBinder.Binder).Assembly;
			if (!string.IsNullOrEmpty(csharp_asm.Location))
				AddReference(csharp_asm.Location, csharp_asm.GetName().Name ?? "Microsoft.CSharp");

			var compilation = CSharpCompilation.Create(
				assemblyName: "LDrawVisualiserScript",
				syntaxTrees: [syntax_tree],
				references: references,
				options: new CSharpCompilationOptions(OutputKind.DynamicallyLinkedLibrary));

			using var ms = new MemoryStream();
			var emit_result = compilation.Emit(ms);

			// Map diagnostics back to user script (offset the line numbers)
			// The wrapper adds: usings_line_count lines + 1 blank + 1 namespace + 1 { + 1 class + 1 { + 1 method + 1 { = usings_line_count + 7
			var wrapper_line_offset = usings_line_count + 7;
			var mapped_diagnostics = resolution_diagnostics
				.Concat(emit_result.Diagnostics
					.Where(d => d.Severity >= DiagnosticSeverity.Warning)
					.Select(d => MapDiagnosticToUserScript(d, wrapper_line_offset)))
				.ToImmutableArray();

			if (!emit_result.Success)
				return (false, mapped_diagnostics, null);

			ms.Seek(0, SeekOrigin.Begin);
			var assembly = Assembly.Load(ms.ToArray());
			return (true, mapped_diagnostics, assembly);
		}

		/// <summary>Adjust line numbers to account for the wrapper code</summary>
		private static Diagnostic MapDiagnosticToUserScript(Diagnostic diagnostic, int wrapper_line_offset)
		{
			if (diagnostic.Location.IsInSource)
			{
				var span = diagnostic.Location.GetLineSpan();
				var adjusted_line = Math.Max(1, span.StartLinePosition.Line - wrapper_line_offset + 1);
				var col = span.StartLinePosition.Character + 1;
				var severity = diagnostic.Severity == DiagnosticSeverity.Error ? "error" : "warning";
				var message = $"({adjusted_line},{col}): {severity} {diagnostic.Id}: {diagnostic.GetMessage()}";

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
