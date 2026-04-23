namespace Rylogic.LDrawVisualiser.Core
{
	using System;
	using System.Collections.Generic;
	using System.IO;
	using System.Linq;

	/// <summary>
	/// Resolves '#r "nuget: Id[, Version]"' references against the user's local NuGet
	/// global-packages cache. Lets a script bind against a specific (or latest) version
	/// of a package the user has restored elsewhere — e.g. by building Rylogic.Gfx and
	/// pushing it into the cache via 'dotnet nuget push' or an MSBuild restore step —
	/// without needing to rebuild the VSIX.
	///
	/// Resolution order:
	///   1. Cache root: $NUGET_PACKAGES if set, else %userprofile%\.nuget\packages.
	///   2. Package dir: &lt;root&gt;\&lt;id-lowercase&gt;\.
	///   3. Version: explicit match if 'version_spec' is a parseable version; otherwise
	///      (or for '*' / empty) the highest-sorted installed version.
	///   4. TFM: the most-compatible target framework folder containing &lt;id&gt;.dll.
	///      Preference list is biased toward .NET Framework 4.x because the visualiser
	///      runs in devenv (net481).
	///
	/// Caveat (documented at the call site too): Roslyn will compile against the metadata
	/// we hand it, but at runtime the CLR resolves by simple name and will reuse any
	/// already-loaded version. So if Rylogic.Gfx is already loaded into devenv (because
	/// the VSIX itself uses it), the script's calls dispatch to the loaded version, not
	/// the cache one. New APIs added in the cache version will fail with MissingMethodException.
	/// For runtime changes you still need to rebuild the VSIX and restart VS.
	/// </summary>
	internal static class NuGetResolver
	{
		// Preferred TFM folders, most-specific first. Biased toward net481 since that's
		// the runtime devenv loads our scripts under.
		private static readonly string[] s_tfm_preference = new[]
		{
			"net481", "net48", "net472", "net471", "net47",
			"net462", "net461", "net46", "net452", "net451", "net45",
			"netstandard2.0", "netstandard2.1",
			"netstandard1.6", "netstandard1.5", "netstandard1.4", "netstandard1.3",
		};

		/// <summary>The user's NuGet global packages folder.</summary>
		public static string CacheRoot
		{
			get
			{
				var env = Environment.GetEnvironmentVariable("NUGET_PACKAGES");
				if (!string.IsNullOrEmpty(env))
					return env!;

				return Path.Combine(
					Environment.GetFolderPath(Environment.SpecialFolder.UserProfile),
					".nuget", "packages");
			}
		}

		/// <summary>Resolve a single package reference to a DLL path. Returns null if
		/// no installed version matches the spec or no compatible TFM is present.</summary>
		/// <param name="id">Package id (case-insensitive)</param>
		/// <param name="version_spec">Explicit version (e.g. "1.2.3"), "*" / empty = highest</param>
		public static string? Resolve(string id, string? version_spec)
		{
			if (string.IsNullOrEmpty(id))
				return null;

			var pkg_dir = Path.Combine(CacheRoot, id.ToLowerInvariant());
			if (!Directory.Exists(pkg_dir))
				return null;

			var versions = Directory.GetDirectories(pkg_dir)
				.Select(d => new { Path = d, Name = Path.GetFileName(d) })
				.Where(v => !string.IsNullOrEmpty(v.Name))
				.ToList();
			if (versions.Count == 0)
				return null;

			// Pick the version directory: explicit name match, else highest-sorted by Version.
			string? chosen_dir = null;
			if (!string.IsNullOrEmpty(version_spec) && version_spec != "*")
			{
				chosen_dir = versions.FirstOrDefault(v => string.Equals(v.Name, version_spec, StringComparison.OrdinalIgnoreCase))?.Path;
			}
			if (chosen_dir == null)
			{
				// Sort by parsed Version where possible; fall back to lexical.
				chosen_dir = versions
					.Select(v => new { v.Path, Version = TryParseVersion(v.Name), Name = v.Name })
					.OrderByDescending(v => v.Version != null)
					.ThenByDescending(v => v.Version)
					.ThenByDescending(v => v.Name, StringComparer.OrdinalIgnoreCase)
					.First().Path;
			}

			// Find the best lib\<tfm>\ folder containing the DLL.
			var lib_dir = Path.Combine(chosen_dir, "lib");
			if (!Directory.Exists(lib_dir))
				return null;

			var dll_name = $"{id}.dll";
			foreach (var tfm in s_tfm_preference)
			{
				var candidate = Path.Combine(lib_dir, tfm, dll_name);
				if (File.Exists(candidate))
					return candidate;
			}

			// Fallback: any TFM subfolder that has the DLL — better to compile against
			// something than fail outright.
			var any = Directory.GetDirectories(lib_dir)
				.Select(tfm_dir => Path.Combine(tfm_dir, dll_name))
				.FirstOrDefault(File.Exists);
			return any;
		}

		// Parse strings like "1.2.3" or "1.2.3.4". Pre-release suffixes (e.g. "1.0-beta")
		// fall through to null and the caller orders them after parseable versions.
		private static Version? TryParseVersion(string s)
		{
			if (string.IsNullOrEmpty(s))
				return null;

			// Strip semver pre-release / metadata so System.Version can parse the numeric prefix.
			var dash = s.IndexOfAny(new[] { '-', '+' });
			var numeric = dash >= 0 ? s.Substring(0, dash) : s;
			return Version.TryParse(numeric, out var v) ? v : null;
		}
	}
}
