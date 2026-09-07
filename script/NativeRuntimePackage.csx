#nullable enable

using System;
using System.ComponentModel;
using System.IO;
using System.IO.Compression;
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
				var manifest_dir = ManifestDirectory(workspace, platform, config);
				if (Directory.Exists(manifest_dir))
					Directory.Delete(manifest_dir, recursive: true);
			}
		}
	}

	// Stages the declared runtime assets and proves that their linked dependency closure is complete.
	public static string Stage(string workspace, string platform, string config, string output_dir, bool require_all_projects)
	{
		var projects = DiscoverRuntimeProjects(workspace);
		var manifest_dir = ManifestDirectory(workspace, platform, config);
		var missing_projects = new List<string>();
		var source_files = new Dictionary<string, string>(StringComparer.OrdinalIgnoreCase);

		// Collect only DLL build artifacts emitted by projects that explicitly opt into the runtime package.
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
		if (source_files.Count == 0)
			throw new InvalidOperationException($"No declared native runtime assets are available for {platform}|{config}.");

		// Recreate a package-private staging directory so unrelated deployed files cannot leak into the archive.
		var staging_dir = PrepareStagingDirectory(workspace, output_dir);
		foreach (var source_file in source_files.OrderBy(x => x.Key, StringComparer.OrdinalIgnoreCase))
			File.Copy(source_file.Value, IOPath.Combine(staging_dir, source_file.Key), overwrite: true);

		// Check both the static PE import graph and actual Windows loader behavior before packaging.
		ValidateDependencies(staging_dir);
		SmokeLoad(staging_dir);
		return staging_dir;
	}

	// Reports whether every included native project has a usable manifest so a local package selector can advance safely.
	public static bool HasCompleteManifestSet(string workspace, string platform, string config, out IReadOnlyList<string> unavailable_inputs)
	{
		var unavailable = new List<string>();
		var manifest_dir = ManifestDirectory(workspace, platform, config);

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

	// Stages the command line tools carried by the package, together with the runtime libraries they load.
	// Tools are applications rather than part of the runtime DLL closure, so they are staged separately and
	// are never smoke loaded.
	public static string StageTools(string workspace, string platform, string config, string output_dir, string runtime_staging_dir)
	{
		var staging_dir = PrepareStagingDirectory(workspace, output_dir);

		// A tool resolves its dynamically loaded libraries from its own directory, so each required library is
		// copied from the already-validated runtime closure rather than located independently.
		foreach (var library in PackagedToolLibraries)
		{
			var source_path = IOPath.Combine(runtime_staging_dir, library);
			if (!File.Exists(source_path))
				throw new FileNotFoundException($"Tool dependency is not present in the staged runtime closure: {library}", source_path);

			File.Copy(source_path, IOPath.Combine(staging_dir, library), overwrite: true);
		}

		foreach (var tool_name in PackagedToolNames)
		{
			var source_path = PackagedToolPath(workspace, tool_name, platform, config);
			if (!File.Exists(source_path))
				throw new FileNotFoundException($"Packaged tool has not been built for {platform}|{config}: {tool_name}", source_path);

			File.Copy(source_path, IOPath.Combine(staging_dir, IOPath.GetFileName(source_path)), overwrite: true);
		}

		return staging_dir;
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

	// Confirms that the generated package contains exactly the staged runtime DLLs.
	public static void ValidatePackage(string package_path, string staging_dir)
	{
		ValidatePackage(package_path, staging_dir, tools_staging_dir: null);
	}

	// Confirms that the generated package contains exactly the staged runtime DLLs and, when tools are carried,
	// exactly the staged tool payload.
	public static void ValidatePackage(string package_path, string staging_dir, string? tools_staging_dir)
	{
		using var package = ZipFile.OpenRead(package_path);
		ValidateFolder(package, staging_dir, "*.dll", "runtimes/win-x64/native/");
		if (tools_staging_dir is not null)
			ValidateFolder(package, tools_staging_dir, "*", "tools/win-x64/");
	}

	// Compares one package folder against the staging directory that produced it.
	private static void ValidateFolder(ZipArchive package, string staging_dir, string staging_pattern, string package_folder)
	{
		var expected = Directory.EnumerateFiles(staging_dir, staging_pattern)
			.Select(IOPath.GetFileName)
			.ToHashSet(StringComparer.OrdinalIgnoreCase);
		var actual = package.Entries
			.Where(x => x.FullName.StartsWith(package_folder, StringComparison.OrdinalIgnoreCase) && !x.FullName.EndsWith("/", StringComparison.Ordinal))
			.Select(x => IOPath.GetFileName(x.FullName))
			.ToHashSet(StringComparer.OrdinalIgnoreCase);

		if (!expected.SetEquals(actual))
		{
			var missing = expected.Except(actual, StringComparer.OrdinalIgnoreCase).OrderBy(x => x);
			var unexpected = actual.Except(expected, StringComparer.OrdinalIgnoreCase).OrderBy(x => x);
			throw new InvalidOperationException($"Rylogic.Native package inventory mismatch in '{package_folder}'. Missing: [{string.Join(", ", missing)}]. Unexpected: [{string.Join(", ", unexpected)}].");
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

	// Returns the build-generated project-manifest directory for one native configuration.
	private static string ManifestDirectory(string workspace, string platform, string config)
	{
		return IOPath.Combine(workspace, "obj", "native-runtime-manifests", platform, config);
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
