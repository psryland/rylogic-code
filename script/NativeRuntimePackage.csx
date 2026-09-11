#nullable enable

using System;
using System.ComponentModel;
using System.IO;
using System.IO.Compression;
using System.Linq;
using System.Runtime.InteropServices;
using System.Security.Cryptography;
using System.Text.RegularExpressions;
using System.Xml.Linq;
using IOPath = System.IO.Path;

// Stages and validates the exact runtime closure declared by native library projects.
public static class NativeRuntimePackage
{
	// The command line tools carried by the package so that a consuming repository can cook content without a
	// checkout of this one, and the runtime libraries those tools load from their own directory.
	public static readonly IReadOnlyList<string> PackagedToolNames = ["p3d"];
	public static readonly IReadOnlyList<string> PackagedToolLibraries = ["gltf.dll"];

	private const string IncludeDisposition = "Include";
	private const string ExcludeDisposition = "Exclude";
	private const uint LoadLibrarySearchDllLoadDir = 0x00000100;
	private const uint LoadLibrarySearchDefaultDirs = 0x00001000;
	private static readonly Regex m_dependency_pattern = new(@"^\s+(?<name>[^\\/:*?""<>|\s]+\.dll)\s*$", RegexOptions.Compiled | RegexOptions.IgnoreCase | RegexOptions.Multiline);

	// Removes generated project manifests before an aggregate build so omitted projects cannot be hidden by stale state.
	public static void ClearManifests(string workspace, IEnumerable<string> platforms, IEnumerable<string> configs)
	{
		foreach (var platform in platforms)
		{
			foreach (var config in configs)
			{
				foreach (var manifest_dir in new[]
				{
					RuntimeManifestDirectory(workspace, platform, config),
					LinkManifestDirectory(workspace, platform, config),
				})
				{
					if (Directory.Exists(manifest_dir))
						Directory.Delete(manifest_dir, recursive: true);
				}
			}
		}
	}

	// Stages the complete Rylogic.Native payload and proves that its runtime closure is loadable.
	public static string Stage(string workspace, string platform, string config, string output_dir, bool require_all_projects)
	{
		var runtime_assets = CollectRuntimeAssets(workspace, platform, config, require_all_projects);
		if (runtime_assets.Count == 0)
			throw new InvalidOperationException($"No declared native runtime assets are available for {platform}|{config}.");

		// Recreate a package-private staging directory so unrelated deployed files cannot leak into the archive.
		var staging_dir = PrepareStagingDirectory(workspace, output_dir);
		StageHeaders(workspace, staging_dir);
		StageProps(workspace, staging_dir);
		var runtime_dir = StageRuntimeAssets(platform, staging_dir, runtime_assets);
		StageLinkAssets(workspace, platform, config, staging_dir, runtime_assets, require_all_projects);
		StageTools(workspace, platform, config, staging_dir, runtime_dir);
		return staging_dir;
	}

	// Reports whether every included native project has a usable manifest so a local package selector can advance safely.
	public static bool HasCompleteManifestSet(string workspace, string platform, string config, out IReadOnlyList<string> unavailable_inputs)
	{
		var unavailable = new List<string>();
		var manifest_dir = RuntimeManifestDirectory(workspace, platform, config);

		// Require each package-owned project to have declared at least one available DLL from its latest build.
		foreach (var project in DiscoverRuntimeProjects(workspace).Where(x => x.Disposition == IncludeDisposition))
		{
			var manifest_path = IOPath.Combine(manifest_dir, $"{project.ProjectName}.txt");
			if (!File.Exists(manifest_path))
			{
				unavailable.Add($"missing manifest: {project.ProjectPath}");
				continue;
			}

			var source_paths = File.ReadLines(manifest_path)
				.Select(x => x.Trim())
				.Where(x => x.Length != 0 && string.Equals(IOPath.GetExtension(x), ".dll", StringComparison.OrdinalIgnoreCase))
				.ToList();
			if (source_paths.Count == 0)
			{
				unavailable.Add($"manifest declares no DLLs: {manifest_path}");
				continue;
			}

			foreach (var source_path in source_paths.Where(x => !File.Exists(x)))
				unavailable.Add($"missing runtime asset: {source_path}");
		}

		unavailable_inputs = unavailable;
		return unavailable.Count == 0;
	}

	// Reports whether every supported native link asset is available for one configuration.
	public static bool HasCompleteLinkAssetSet(string workspace, string platform, string config, out IReadOnlyList<string> unavailable_inputs)
	{
		var unavailable = new List<string>();
		try
		{
			foreach (var runtime_asset in CollectRuntimeAssets(workspace, platform, config, require_all_projects: true).Values)
			{
				if (!RequiresAdjacentImportLibrary(workspace, runtime_asset))
					continue;

				var import_library = IOPath.ChangeExtension(runtime_asset, ".imp");
				if (!File.Exists(import_library))
					unavailable.Add($"missing import library: {import_library}");
			}
		}
		catch (Exception ex)
		{
			unavailable.Add(ex.Message);
		}

		var manifest_dir = LinkManifestDirectory(workspace, platform, config);
		foreach (var project in DiscoverLinkProjects(workspace).Where(x => x.Disposition == IncludeDisposition))
		{
			var manifest_path = IOPath.Combine(manifest_dir, $"{project.ProjectName}.txt");
			if (!File.Exists(manifest_path))
			{
				unavailable.Add($"missing link manifest: {project.ProjectPath}");
				continue;
			}

			var source_paths = File.ReadLines(manifest_path).Select(x => x.Trim()).Where(x => x.Length != 0).ToList();
			if (source_paths.Count == 0)
			{
				unavailable.Add($"manifest declares no link assets: {manifest_path}");
				continue;
			}

			foreach (var source_path in source_paths.Where(x => !File.Exists(x)))
				unavailable.Add($"missing link asset: {source_path}");
		}

		unavailable_inputs = unavailable;
		return unavailable.Count == 0;
	}

	// True when every packaged tool has been built for one configuration. A package missing a tool is incomplete
	// in the same way as one missing a runtime library.
	public static bool HasCompleteToolSet(string workspace, string platform, string config, out IReadOnlyList<string> unavailable_tools)
	{
		var unavailable = new List<string>();
		foreach (var tool_name in PackagedToolNames)
		{
			var source_path = PackagedToolPath(workspace, tool_name, platform, config);
			if (!File.Exists(source_path))
				unavailable.Add($"missing tool: {source_path}");
		}

		unavailable_tools = unavailable;
		return unavailable.Count == 0;
	}

	// Returns the built location of a packaged tool. Applications use the project-local output convention rather
	// than the shared 'obj' tree that libraries use.
	public static string PackagedToolPath(string workspace, string tool_name, string platform, string config)
	{
		return IOPath.Combine(workspace, "projects", "tools", tool_name, "obj", platform, config, $"{tool_name}.exe");
	}

	// Confirms that the generated package contains exactly the staged build/runtime/tool payload.
	public static void ValidatePackage(string package_path, string staging_dir)
	{
		using var package = ZipFile.OpenRead(package_path);
		var expected = Directory.EnumerateFiles(staging_dir, "*", SearchOption.AllDirectories)
			.Select(x => IOPath.GetRelativePath(staging_dir, x).Replace('\\', '/'))
			.Where(x => !string.Equals(x, ".complete", StringComparison.OrdinalIgnoreCase))
			.ToHashSet(StringComparer.OrdinalIgnoreCase);
		var actual = package.Entries
			.Where(x => !x.FullName.EndsWith("/", StringComparison.Ordinal))
			.Select(x => x.FullName.Replace('\\', '/'))
			.Where(x => x.StartsWith("build/", StringComparison.OrdinalIgnoreCase) || x.StartsWith("runtimes/", StringComparison.OrdinalIgnoreCase) || x.StartsWith("tools/", StringComparison.OrdinalIgnoreCase))
			.ToHashSet(StringComparer.OrdinalIgnoreCase);

		if (!expected.SetEquals(actual))
		{
			var missing = expected.Except(actual, StringComparer.OrdinalIgnoreCase).OrderBy(x => x);
			var unexpected = actual.Except(expected, StringComparer.OrdinalIgnoreCase).OrderBy(x => x);
			throw new InvalidOperationException($"Rylogic.Native package inventory mismatch. Missing: [{string.Join(", ", missing)}]. Unexpected: [{string.Join(", ", unexpected)}].");
		}
	}

	// Discovers production DLL projects and requires each one to make an explicit release-package decision.
	private static List<RuntimeProject> DiscoverRuntimeProjects(string workspace)
	{
		var project_paths = Directory.EnumerateFiles(IOPath.Combine(workspace, "projects", "rylogic"), "*.vcxproj", SearchOption.AllDirectories)
			.Concat([
				IOPath.Combine(workspace, "sdk", "sqlite3", "sqlite.vcxproj"),
				IOPath.Combine(workspace, "sdk", "scintilla", "scintilla.vcxproj"),
			])
			.Distinct(StringComparer.OrdinalIgnoreCase);
		var projects = new List<RuntimeProject>();

		foreach (var project_path in project_paths)
		{
			var project = XDocument.Load(project_path);
			var is_dynamic_library = project.Descendants().Any(x => x.Name.LocalName == "ConfigurationType" && x.Value.Trim() == "DynamicLibrary");
			if (!is_dynamic_library)
				continue;

			var project_name = IOPath.GetFileNameWithoutExtension(project_path);
			var disposition = project.Descendants().LastOrDefault(x => x.Name.LocalName == "RylogicNativeRuntimePackage")?.Value.Trim();
			var reason = project.Descendants().LastOrDefault(x => x.Name.LocalName == "RylogicNativeRuntimePackageReason")?.Value.Trim();
			if (disposition is not (IncludeDisposition or ExcludeDisposition))
				throw new InvalidOperationException($"Dynamic library project must declare RylogicNativeRuntimePackage as Include or Exclude: {project_path}");
			if (disposition == ExcludeDisposition && string.IsNullOrWhiteSpace(reason))
				throw new InvalidOperationException($"Excluded native runtime project must declare RylogicNativeRuntimePackageReason: {project_path}");

			projects.Add(new RuntimeProject(project_path, project_name, disposition));
		}

		return projects;
	}

	// Discovers projects that explicitly contribute native link assets beyond DLL import libraries.
	private static List<LinkProject> DiscoverLinkProjects(string workspace)
	{
		var project_paths = Directory.EnumerateFiles(IOPath.Combine(workspace, "projects", "rylogic"), "*.vcxproj", SearchOption.AllDirectories)
			.Concat(Directory.EnumerateFiles(IOPath.Combine(workspace, "sdk"), "*.vcxproj", SearchOption.AllDirectories))
			.Distinct(StringComparer.OrdinalIgnoreCase);
		var projects = new List<LinkProject>();

		foreach (var project_path in project_paths)
		{
			var project = XDocument.Load(project_path);
			var project_name = IOPath.GetFileNameWithoutExtension(project_path);
			var disposition = project.Descendants().LastOrDefault(x => x.Name.LocalName == "RylogicNativeLinkPackage")?.Value.Trim();
			if (string.IsNullOrWhiteSpace(disposition))
				continue;
			if (disposition is not (IncludeDisposition or ExcludeDisposition))
				throw new InvalidOperationException($"Native link project must declare RylogicNativeLinkPackage as Include or Exclude: {project_path}");

			projects.Add(new LinkProject(project_path, project_name, disposition));
		}

		return projects;
	}

	// Rejects linked non-system dependencies that are absent from the staged runtime set.
	private static void ValidateDependencies(string staging_dir)
	{
		Tools.SetupVcEnvironment();
		var staged_names = Directory.EnumerateFiles(staging_dir, "*.dll")
			.Select(IOPath.GetFileName)
			.ToHashSet(StringComparer.OrdinalIgnoreCase);
		var unresolved = new List<string>();

		foreach (var dll_path in Directory.EnumerateFiles(staging_dir, "*.dll").OrderBy(x => x))
		{
			var (_, output) = Tools.Run(["dumpbin.exe", "/dependents", dll_path]);
			foreach (Match match in m_dependency_pattern.Matches(output))
			{
				var dependency = match.Groups["name"].Value;
				if (staged_names.Contains(dependency) || IsSystemDependency(dependency))
					continue;

				unresolved.Add($"{IOPath.GetFileName(dll_path)} -> {dependency}");
			}
		}

		if (unresolved.Count != 0)
			throw new InvalidOperationException($"Rylogic.Native has unresolved runtime dependencies:{Environment.NewLine}{string.Join(Environment.NewLine, unresolved.Select(x => $"  {x}"))}");
	}

	// Loads every staged DLL using package-local dependency resolution to catch missing dynamic prerequisites.
	private static void SmokeLoad(string staging_dir)
	{
		foreach (var dll_path in Directory.EnumerateFiles(staging_dir, "*.dll").OrderBy(x => x))
		{
			var module = NativeMethods.LoadLibraryEx(dll_path, IntPtr.Zero, LoadLibrarySearchDllLoadDir | LoadLibrarySearchDefaultDirs);
			if (module == IntPtr.Zero)
				throw new Win32Exception(Marshal.GetLastWin32Error(), $"Failed to load packaged native runtime '{dll_path}'");

			if (!NativeMethods.FreeLibrary(module))
				throw new Win32Exception(Marshal.GetLastWin32Error(), $"Failed to unload packaged native runtime '{dll_path}'");
		}
	}

	// Treats Windows system libraries as host prerequisites rather than package payload.
	private static bool IsSystemDependency(string dependency)
	{
		if (dependency.StartsWith("api-ms-win-", StringComparison.OrdinalIgnoreCase) || dependency.StartsWith("ext-ms-win-", StringComparison.OrdinalIgnoreCase))
			return true;

		return File.Exists(IOPath.Combine(Environment.SystemDirectory, dependency));
	}

	// Collects DLL payloads from project-owned runtime manifests.
	private static Dictionary<string, string> CollectRuntimeAssets(string workspace, string platform, string config, bool require_all_projects)
	{
		var projects = DiscoverRuntimeProjects(workspace);
		var manifest_dir = RuntimeManifestDirectory(workspace, platform, config);
		var missing_projects = new List<string>();
		var source_files = new Dictionary<string, string>(StringComparer.OrdinalIgnoreCase);

		foreach (var project in projects.Where(x => x.Disposition == IncludeDisposition))
		{
			var manifest_path = IOPath.Combine(manifest_dir, $"{project.ProjectName}.txt");
			if (!File.Exists(manifest_path))
			{
				missing_projects.Add(project.ProjectPath);
				continue;
			}

			foreach (var source_path in File.ReadLines(manifest_path).Select(x => x.Trim()).Where(x => x.Length != 0))
			{
				if (!string.Equals(IOPath.GetExtension(source_path), ".dll", StringComparison.OrdinalIgnoreCase))
					continue;
				if (!File.Exists(source_path))
				{
					if (require_all_projects)
						throw new FileNotFoundException($"Declared native runtime asset is missing: {source_path}", source_path);

					Console.WriteLine($"Skipping unavailable Debug runtime asset: {source_path}");
					continue;
				}

				var filename = IOPath.GetFileName(source_path);
				if (source_files.TryGetValue(filename, out var existing_path) && !FilesEqual(existing_path, source_path))
					throw new InvalidOperationException($"Native runtime assets collide at '{filename}': '{existing_path}' and '{source_path}'");

				source_files[filename] = source_path;
			}
		}

		if (require_all_projects && missing_projects.Count != 0)
			throw new InvalidOperationException($"Native runtime manifests are missing for:{Environment.NewLine}{string.Join(Environment.NewLine, missing_projects.Select(x => $"  {x}"))}{Environment.NewLine}Build AllNative before creating a Release package.");

		return source_files;
	}

	// Copies the public native header tree into the package layout expected by C++ consumers.
	private static void StageHeaders(string workspace, string staging_dir)
	{
		var source_dir = IOPath.Combine(workspace, "include", "pr");
		var target_dir = IOPath.Combine(staging_dir, "build", "native", "include", "pr");
		CopyDirectory(source_dir, target_dir);
	}

	// Copies the package-root props file that exposes include/lib/runtime locations without auto-linking.
	private static void StageProps(string workspace, string staging_dir)
	{
		var source_path = IOPath.Combine(workspace, "build", "Rylogic.Native.props");
		var target_path = IOPath.Combine(staging_dir, "build", "Rylogic.Native.props");
		Directory.CreateDirectory(IOPath.GetDirectoryName(target_path) ?? throw new InvalidOperationException("Unable to determine the props staging directory."));
		File.Copy(source_path, target_path, overwrite: true);
	}

	// Copies runtime DLLs into their NuGet runtime folder and validates that the staged closure is loadable.
	private static string StageRuntimeAssets(string platform, string staging_dir, IReadOnlyDictionary<string, string> runtime_assets)
	{
		var runtime_dir = IOPath.Combine(staging_dir, "runtimes", $"win-{platform}", "native");
		Directory.CreateDirectory(runtime_dir);
		foreach (var runtime_asset in runtime_assets.OrderBy(x => x.Key, StringComparer.OrdinalIgnoreCase))
			File.Copy(runtime_asset.Value, IOPath.Combine(runtime_dir, runtime_asset.Key), overwrite: true);

		ValidateDependencies(runtime_dir);
		SmokeLoad(runtime_dir);
		return runtime_dir;
	}

	// Copies import libraries and explicitly declared static archives into the configuration-specific consumer lib folder.
	private static void StageLinkAssets(string workspace, string platform, string config, string staging_dir, IReadOnlyDictionary<string, string> runtime_assets, bool require_all_projects)
	{
		var target_dir = IOPath.Combine(staging_dir, "build", "native", "lib", platform, config);
		Directory.CreateDirectory(target_dir);
		foreach (var runtime_asset in runtime_assets.Values.OrderBy(x => x, StringComparer.OrdinalIgnoreCase))
		{
			if (!RequiresAdjacentImportLibrary(workspace, runtime_asset))
				continue;

			var import_library = IOPath.ChangeExtension(runtime_asset, ".imp");
			if (!File.Exists(import_library))
			{
				if (require_all_projects)
					throw new FileNotFoundException($"Declared native import library is missing: {import_library}", import_library);
				continue;
			}

			File.Copy(import_library, IOPath.Combine(target_dir, IOPath.GetFileName(import_library)), overwrite: true);
		}

		var manifest_dir = LinkManifestDirectory(workspace, platform, config);
		foreach (var project in DiscoverLinkProjects(workspace).Where(x => x.Disposition == IncludeDisposition))
		{
			var manifest_path = IOPath.Combine(manifest_dir, $"{project.ProjectName}.txt");
			if (!File.Exists(manifest_path))
			{
				if (require_all_projects)
					throw new InvalidOperationException($"Native link manifest is missing: {project.ProjectPath}");
				continue;
			}

			foreach (var source_path in File.ReadLines(manifest_path).Select(x => x.Trim()).Where(x => x.Length != 0))
			{
				if (!File.Exists(source_path))
				{
					if (require_all_projects)
						throw new FileNotFoundException($"Declared native link asset is missing: {source_path}", source_path);
					continue;
				}

				File.Copy(source_path, IOPath.Combine(target_dir, IOPath.GetFileName(source_path)), overwrite: true);
			}
		}
	}

	// Stages command line tools with the runtime libraries they load from their own directory.
	private static void StageTools(string workspace, string platform, string config, string staging_dir, string runtime_dir)
	{
		var target_dir = IOPath.Combine(staging_dir, "tools", $"win-{platform}");
		Directory.CreateDirectory(target_dir);
		foreach (var library in PackagedToolLibraries)
		{
			var source_path = IOPath.Combine(runtime_dir, library);
			if (!File.Exists(source_path))
				throw new FileNotFoundException($"Tool dependency is not present in the staged runtime closure: {library}", source_path);

			File.Copy(source_path, IOPath.Combine(target_dir, library), overwrite: true);
		}

		foreach (var tool_name in PackagedToolNames)
		{
			var source_path = PackagedToolPath(workspace, tool_name, platform, config);
			if (!File.Exists(source_path))
				throw new FileNotFoundException($"Packaged tool has not been built for {platform}|{config}: {tool_name}", source_path);

			File.Copy(source_path, IOPath.Combine(target_dir, IOPath.GetFileName(source_path)), overwrite: true);
		}
	}

	// Recursively copies a directory tree into the staging layout.
	private static void CopyDirectory(string source_dir, string target_dir)
	{
		Directory.CreateDirectory(target_dir);
		foreach (var directory in Directory.EnumerateDirectories(source_dir, "*", SearchOption.AllDirectories))
			Directory.CreateDirectory(IOPath.Combine(target_dir, IOPath.GetRelativePath(source_dir, directory)));
		foreach (var file in Directory.EnumerateFiles(source_dir, "*", SearchOption.AllDirectories))
		{
			var relative = IOPath.GetRelativePath(source_dir, file);
			var destination = IOPath.Combine(target_dir, relative);
			Directory.CreateDirectory(IOPath.GetDirectoryName(destination) ?? throw new InvalidOperationException($"Unable to determine the destination directory for '{destination}'"));
			File.Copy(file, destination, overwrite: true);
		}
	}

	// Creates a clean staging directory only within the repository's generated NuGet area.
	private static string PrepareStagingDirectory(string workspace, string output_dir)
	{
		var allowed_root = IOPath.GetFullPath(IOPath.Combine(workspace, "obj", "nuget", "Rylogic.Native")) + IOPath.DirectorySeparatorChar;
		var staging_dir = IOPath.GetFullPath(output_dir);
		if (!staging_dir.StartsWith(allowed_root, StringComparison.OrdinalIgnoreCase))
			throw new InvalidOperationException($"Native package staging must remain under '{allowed_root}': {staging_dir}");

		if (Directory.Exists(staging_dir))
			Directory.Delete(staging_dir, recursive: true);

		Directory.CreateDirectory(staging_dir);
		return staging_dir;
	}

	// Returns the build-generated runtime-manifest directory for one native configuration.
	private static string RuntimeManifestDirectory(string workspace, string platform, string config)
	{
		return IOPath.Combine(workspace, "obj", "native-runtime-manifests", platform, config);
	}

	// Returns the build-generated native-link-manifest directory for one native configuration.
	private static string LinkManifestDirectory(string workspace, string platform, string config)
	{
		return IOPath.Combine(workspace, "obj", "native-link-manifests", platform, config);
	}

	// Treat third-party packaged DLLs as runtime-only payload because they do not participate in the supported
	// Rylogic import-library surface.
	private static bool RequiresAdjacentImportLibrary(string workspace, string runtime_asset)
	{
		var packages_dir = IOPath.GetFullPath(IOPath.Combine(workspace, "packages")) + IOPath.DirectorySeparatorChar;
		var source_path = IOPath.GetFullPath(runtime_asset);
		return !source_path.StartsWith(packages_dir, StringComparison.OrdinalIgnoreCase);
	}

	// Compares duplicate-named artifacts without trusting timestamps.
	private static bool FilesEqual(string lhs, string rhs)
	{
		if (string.Equals(IOPath.GetFullPath(lhs), IOPath.GetFullPath(rhs), StringComparison.OrdinalIgnoreCase))
			return true;

		var lhs_info = new FileInfo(lhs);
		var rhs_info = new FileInfo(rhs);
		if (lhs_info.Length != rhs_info.Length)
			return false;

		using var lhs_stream = File.OpenRead(lhs);
		using var rhs_stream = File.OpenRead(rhs);
		return SHA256.HashData(lhs_stream).SequenceEqual(SHA256.HashData(rhs_stream));
	}

	// Captures the package decision for one production DLL project.
	private sealed record RuntimeProject(string ProjectPath, string ProjectName, string Disposition);

	// Captures the package decision for one explicit native link-asset project.
	private sealed record LinkProject(string ProjectPath, string ProjectName, string Disposition);

	// Provides the Windows loader operations used by package smoke validation.
	private static class NativeMethods
	{
		[DllImport("kernel32.dll", EntryPoint = "LoadLibraryExW", CharSet = CharSet.Unicode, SetLastError = true)]
		public static extern IntPtr LoadLibraryEx(string file_name, IntPtr file, uint flags);

		[DllImport("kernel32.dll", SetLastError = true)]
		[return: MarshalAs(UnmanagedType.Bool)]
		public static extern bool FreeLibrary(IntPtr module);
	}
}
