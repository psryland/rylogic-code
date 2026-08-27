#! "net10.0"
#r "System.Text.Json"
#r "nuget: Rylogic.Core, 2.0.2"
#load "UserVars.csx"
#load "Tools.csx"
#nullable enable

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.IO.Compression;
using System.Linq;
using System.Runtime;
using System.Security.Cryptography;
using System.Text.Json;
using System.Text.RegularExpressions;
using System.Xml;
using System.Xml.Linq;
using System.Threading;
using Rylogic.Extn;
using IOFile = System.IO.File;
using IOPath = System.IO.Path;

public class Nuget
{
	// Notes:
	//  - Assembly signing, packaging, and publishing are all separate steps.
	public record class File(string Path, string Target, bool Sign = false) { }
	public record class Dep(string AssemblyName, string Version, string? Framework = null) { }

	// Identifies a runtime framework required by package consumers for a target framework.
	public record class FrameworkRef(string Name, string Framework) { }

	// Describes one immutable archive in a validated public release.
	private sealed record ReleasePackageManifestEntry(string Id, string FileName, long SizeBytes, string Sha256);

	// Describes the exact archive set transferred between the build and publish jobs.
	private sealed record ReleasePackageManifest(int SchemaVersion, string Version, IReadOnlyList<ReleasePackageManifestEntry> Packages);

	public string PackageName = "";
	public string Version = "1.0.0";
	public string? Description;
	public string Tags = "";
	public List<File> Files = [];
	public List<Dep> Deps = [];
	public List<FrameworkRef> FrameworkRefs = [];

	// Validates a newly packed archive before it is signed or synchronized to shared package locations.
	public Action<string>? ValidateStagedPackage { get; set; }

	// The config-specific feed used to stage the package before synchronisation.
	public string PackageOutputPath = "";
	
	// Set automatically by 'Package()' to the canonical root feed package path.
	public string PackagePath = "";

	// Set automatically by 'Package()' to the staged config-specific package path.
	public string StagedPackagePath = "";

	// Create a NuGet package from the given project file.
	// Expects a file called 'package.nuspec' in the same directory as 'proj'. The package is first staged in the
	// config-specific feed under lib\packages\<configuration>, then synchronised to the canonical root feed and cache.
	public void Package()
	{
		// Create a nuspec file based on the template
		var ns = "http://schemas.microsoft.com/packaging/2013/05/nuspec.xsd";
		var xml_nuspec = XDocument.Load(Tools.Path([UserVars.Root, "build\\nuget\\rylogic.template.nuspec"])).Root ?? throw new Exception("Root element not found in nuspec template");;

		// Update metadata
		var xml_metadata = xml_nuspec.Element(XName.Get("metadata", ns)) ?? throw new Exception("metadata element not found in nuspec template");
		xml_metadata.Element(XName.Get("id", ns))?.SetValue(PackageName);
		xml_metadata.Element(XName.Get("version", ns))?.SetValue(Version);
		if (Description is not null)
			xml_metadata.Element(XName.Get("description", ns))?.SetValue(Description);
		xml_metadata.Element(XName.Get("tags", ns))?.SetValue(Tags);

		// Add each file to the spec
		var xml_files = xml_nuspec.Element(XName.Get("files", ns)) ?? throw new Exception("files element not found in nuspec template");
		foreach (var file in Files)
		{
			// Check the file exists
			if (!file.Path.Contains('?') && !file.Path.Contains('*') && !IOPath.Exists(file.Path))
				throw new Exception($"File missing for package {PackageName}: {file.Path}");

			// Sign if required
			if (file.Sign)
				Tools.SignAssembly(file.Path);

			// Add the file to the spec
			xml_files.Add2(new XElement(XName.Get("file", ns),
				new XAttribute("src", file.Path),
				new XAttribute("target", file.Target.Replace('\\','/'))
				));
		}

		// Add dependencies
		var xml_dependencies = xml_metadata.Element(XName.Get("dependencies", ns)) ?? throw new Exception("dependencies element not found in nuspec template");
		var has_framework_dependencies = Deps.Any(x => x.Framework != null);
		if (!has_framework_dependencies)
		{
			foreach (var dep in Deps)
			{
				xml_dependencies.Add2(new XElement(XName.Get("dependency", ns),
					new XAttribute("id", dep.AssemblyName),
					new XAttribute("version", dep.Version)
				));
			}
		}
		else
		{
			// NuGet forbids mixing dependency and group elements, so common dependencies must be repeated for every packaged target framework.
			var frameworks = Files
				.Select(file => file.Target.Replace('\\', '/').Trim('/').Split('/'))
				.Where(parts => parts.Length >= 2 && parts[0] == "lib")
				.Select(parts => parts[1])
				.Concat(Deps.Where(dep => dep.Framework != null).Select(dep => dep.Framework!))
				.Distinct(StringComparer.OrdinalIgnoreCase);
			foreach (var framework in frameworks)
			{
				var dependencies = Deps.Where(dep => dep.Framework == null || string.Equals(dep.Framework, framework, StringComparison.OrdinalIgnoreCase));
				xml_dependencies.Add2(new XElement(XName.Get("group", ns),
					new XAttribute("targetFramework", framework),
					dependencies.Select(dep => new XElement(XName.Get("dependency", ns),
						new XAttribute("id", dep.AssemblyName),
						new XAttribute("version", dep.Version)
					))
				));
			}
		}

		// Declare runtime frameworks that package consumers must select.
		if (FrameworkRefs.Count != 0)
		{
			var xml_framework_references = new XElement(XName.Get("frameworkReferences", ns));
			foreach (var group in FrameworkRefs.GroupBy(x => x.Framework))
			{
				xml_framework_references.Add2(new XElement(XName.Get("group", ns),
					new XAttribute("targetFramework", group.Key),
					group.Select(framework => new XElement(XName.Get("frameworkReference", ns),
						new XAttribute("name", framework.Name)
					))
				));
			}
			xml_dependencies.AddAfterSelf(xml_framework_references);
		}

		// Keep generated nuspec files under the repository object directory for diagnostics and cleanup.
		var objdir = Tools.Path([UserVars.Root, "obj", "nuget"], check_exists: false);
		Directory.CreateDirectory(objdir);

		// Stage into the caller's feed when supplied, otherwise use the stable Release feed.
		var package_output_path = string.IsNullOrWhiteSpace(PackageOutputPath)
			? Tools.Path([UserVars.Root, "lib\\packages\\release"], check_exists: false)
			: Tools.Path([PackageOutputPath], check_exists: false);
		Directory.CreateDirectory(package_output_path);

		// Write the nuspec to repo-local obj so packaging never depends on the machine temp directory.
		var nuspec = Tools.Path([objdir, $"{PackageName}.nuspec"], check_exists: false);
		xml_nuspec.Save(nuspec);

		// Build the NuGet package in a config-specific staging feed so Debug and Release builds never collide.
		Tools.Run([UserVars.Nuget, "pack", nuspec,
			"-OutputDirectory", package_output_path,
			"-Verbosity", "detailed",
		]);

		// Validate and sign the staged package before publishing it into the canonical feed and cache.
		StagedPackagePath = Tools.Path([package_output_path, $"{PackageName}.{Version}.nupkg"]);
		PackagePath = Tools.Path([UserVars.Root, $"lib\\packages\\{PackageName}.{Version}.nupkg"], check_exists: false);
		ValidateStagedPackage?.Invoke(StagedPackagePath);
		Tools.SignNugetPackage(StagedPackagePath);
		SyncPackageOutputs(StagedPackagePath, PackagePath);
	}

	// Prevents local long-lived credentials from bypassing the approved OIDC release workflow.
	public void Publish()
	{
		throw new NotSupportedException("Public Rylogic packages are published by .github/workflows/publish-nuget.yml from an approved version tag.");
	}

	// Validates the complete stable package set and writes its deterministic transfer manifest.
	public static void ValidateReleasePackageSet(string version, IReadOnlyCollection<Nuget> packages, string manifest_path)
	{
		if (!Regex.IsMatch(version, @"^\d+\.\d+\.\d+$"))
			throw new Exception($"Public package version '{version}' is not a stable three-part version");
		if (packages.Count == 0)
			throw new Exception("The public package set is empty");

		// Package IDs must be unique because they are the stable identity used by NuGet consumers.
		var duplicate_ids = packages
			.GroupBy(x => x.PackageName, StringComparer.OrdinalIgnoreCase)
			.Where(x => x.Count() != 1)
			.Select(x => x.Key)
			.ToArray();
		if (duplicate_ids.Length != 0)
			throw new Exception($"Duplicate public package IDs: {string.Join(", ", duplicate_ids)}");

		// The same-version staging directory must contain exactly the archives produced by these builders.
		var package_ids = packages.Select(x => x.PackageName).ToHashSet(StringComparer.OrdinalIgnoreCase);
		var expected_paths = packages
			.Select(x => Path.GetFullPath(x.StagedPackagePath))
			.ToHashSet(StringComparer.OrdinalIgnoreCase);
		var release_dir = Path.GetDirectoryName(expected_paths.First()) ?? throw new Exception("Unable to determine the release package directory");
		var actual_paths = Directory.EnumerateFiles(release_dir, $"*.{version}.nupkg", SearchOption.TopDirectoryOnly)
			.Select(Path.GetFullPath)
			.ToHashSet(StringComparer.OrdinalIgnoreCase);
		if (!expected_paths.SetEquals(actual_paths))
		{
			var missing = expected_paths.Except(actual_paths, StringComparer.OrdinalIgnoreCase).Select(Path.GetFileName);
			var unexpected = actual_paths.Except(expected_paths, StringComparer.OrdinalIgnoreCase).Select(Path.GetFileName);
			throw new Exception($"Release package inventory mismatch. Missing: [{string.Join(", ", missing)}]. Unexpected: [{string.Join(", ", unexpected)}]");
		}

		// Inspect the archives rather than trusting their filenames or the inputs used to create them.
		var entries = packages
			.OrderBy(x => x.PackageName, StringComparer.OrdinalIgnoreCase)
			.Select(package => ValidateReleasePackage(package, version, package_ids))
			.ToArray();

		// Persist the verified identities and hashes used to transfer the immutable set into the publish job.
		var manifest = new ReleasePackageManifest(1, version, entries);
		var options = new JsonSerializerOptions
		{
			PropertyNamingPolicy = JsonNamingPolicy.SnakeCaseLower,
			WriteIndented = true,
		};
		Directory.CreateDirectory(Path.GetDirectoryName(manifest_path) ?? throw new Exception($"Unable to determine the manifest directory for '{manifest_path}'"));
		IOFile.WriteAllText(manifest_path, JsonSerializer.Serialize(manifest, options) + Environment.NewLine);
		Console.WriteLine($"Validated {entries.Length} release packages and wrote '{manifest_path}'.");
	}

	// Verifies package identity, framework payloads, internal dependencies, and transfer hash.
	private static ReleasePackageManifestEntry ValidateReleasePackage(Nuget package, string version, IReadOnlySet<string> package_ids)
	{
		if (!IOFile.Exists(package.StagedPackagePath))
			throw new Exception($"Release package is missing: {package.StagedPackagePath}");

		// Match the archive filename to the package identity before opening untrusted package content.
		var expected_name = $"{package.PackageName}.{version}.nupkg";
		if (!string.Equals(Path.GetFileName(package.StagedPackagePath), expected_name, StringComparison.Ordinal))
			throw new Exception($"Release package filename '{Path.GetFileName(package.StagedPackagePath)}' does not match '{expected_name}'");

		// Require one release nuspec and reject paths that reveal Debug or development-version leakage.
		using var archive = ZipFile.OpenRead(package.StagedPackagePath);
		var nuspecs = archive.Entries.Where(x => x.FullName.EndsWith(".nuspec", StringComparison.OrdinalIgnoreCase)).ToArray();
		if (nuspecs.Length != 1)
			throw new Exception($"Package '{expected_name}' contains {nuspecs.Length} nuspec files");
		if (archive.Entries.Any(x => x.FullName.Contains("/Debug/", StringComparison.OrdinalIgnoreCase) || x.FullName.Contains("-dev.", StringComparison.OrdinalIgnoreCase)))
			throw new Exception($"Package '{expected_name}' contains Debug or development-version content");

		// Trust the embedded metadata only when it declares the builder's exact package identity.
		using var nuspec_stream = nuspecs[0].Open();
		var nuspec = XDocument.Load(nuspec_stream);
		var metadata = nuspec.Descendants().Single(x => x.Name.LocalName == "metadata");
		var archive_id = metadata.Elements().Single(x => x.Name.LocalName == "id").Value;
		var archive_version = metadata.Elements().Single(x => x.Name.LocalName == "version").Value;
		if (!string.Equals(archive_id, package.PackageName, StringComparison.Ordinal) || !string.Equals(archive_version, version, StringComparison.Ordinal))
			throw new Exception($"Package '{expected_name}' declares identity '{archive_id} {archive_version}'");

		// Every declared managed target must contain its corresponding assembly and no undeclared target assembly.
		var expected_frameworks = package.Files
			.Select(x => x.Target.Replace('\\', '/').Trim('/').Split('/'))
			.Where(x => x.Length >= 2 && x[0].Equals("lib", StringComparison.OrdinalIgnoreCase))
			.Select(x => x[1])
			.ToHashSet(StringComparer.OrdinalIgnoreCase);
		var actual_frameworks = archive.Entries
			.Select(x => x.FullName.Trim('/').Split('/'))
			.Where(x => x.Length == 3 && x[0].Equals("lib", StringComparison.OrdinalIgnoreCase) && x[2].Equals($"{package.PackageName}.dll", StringComparison.OrdinalIgnoreCase))
			.Select(x => x[1])
			.ToHashSet(StringComparer.OrdinalIgnoreCase);
		if (!expected_frameworks.SetEquals(actual_frameworks))
			throw new Exception($"Package '{expected_name}' framework payload mismatch. Expected [{string.Join(", ", expected_frameworks)}], found [{string.Join(", ", actual_frameworks)}]");

		// Internal dependencies must be part of this release and use its version as their minimum.
		foreach (var dependency in metadata.Descendants().Where(x => x.Name.LocalName == "dependency"))
		{
			var dependency_id = dependency.Attribute("id")?.Value ?? throw new Exception($"Package '{expected_name}' contains a dependency without an ID");
			if (!dependency_id.StartsWith("Rylogic.", StringComparison.OrdinalIgnoreCase))
				continue;
			if (!package_ids.Contains(dependency_id))
				throw new Exception($"Package '{expected_name}' depends on unpublished package '{dependency_id}'");

			// NuGet may normalize an inclusive open range to its equivalent bare minimum version.
			var dependency_version = dependency.Attribute("version")?.Value ?? string.Empty;
			if (!string.Equals(dependency_version, version, StringComparison.Ordinal) &&
				!string.Equals(dependency_version, $"[{version},)", StringComparison.Ordinal))
				throw new Exception($"Package '{expected_name}' depends on '{dependency_id}' using release-inconsistent version '{dependency_version}'");
		}

		// Hash the validated bytes so the publish job can detect any transfer-time mutation.
		using var package_stream = IOFile.OpenRead(package.StagedPackagePath);
		var hash = Convert.ToHexString(SHA256.HashData(package_stream)).ToLowerInvariant();
		return new ReleasePackageManifestEntry(package.PackageName, expected_name, package_stream.Length, hash);
	}

	// Synchronise a staged package into the canonical local feed and NuGet cache without leaving stale extracted
	// payload behind from a previous same-version archive.
	private void SyncPackageOutputs(string staged_nupkg, string feed_nupkg)
	{
		var cache_root = NormalizeDirectory(Tools.Path([Environment.GetFolderPath(Environment.SpecialFolder.UserProfile), ".nuget", "packages", PackageName.ToLowerInvariant(), Version], check_exists: false));
		var cache_nupkg_name = $"{PackageName.ToLowerInvariant()}.{Version}.nupkg";
		var cache_nupkg = Path.Combine(cache_root, cache_nupkg_name);
		var cache_metadata = Path.Combine(cache_root, ".nupkg.metadata");
		var cache_sha512 = cache_nupkg + ".sha512";
		var staged_nupkg_full = Path.GetFullPath(staged_nupkg);
		var feed_nupkg_full = Path.GetFullPath(feed_nupkg);
		var keep_files = new HashSet<string>(StringComparer.OrdinalIgnoreCase)
		{
			cache_nupkg,
			cache_sha512,
			cache_metadata,
		};
		var keep_dirs = new HashSet<string>(StringComparer.OrdinalIgnoreCase)
		{
			cache_root,
		};

		// Create every destination before acquiring the cross-process mutation lock.
		Directory.CreateDirectory(cache_root);
		Directory.CreateDirectory(Path.GetDirectoryName(feed_nupkg_full) ?? throw new InvalidOperationException($"Unable to determine the feed directory for '{feed_nupkg_full}'"));

		// Serialize writers for this package cache entry while allowing unrelated packages to proceed.
		using var mutex = new Mutex(false, MutexName(cache_root));
		var acquired = false;
		try
		{
			try
			{
				acquired = mutex.WaitOne(15000);
			}
			catch (AbandonedMutexException)
			{
				acquired = true;
			}

			// Never mutate the feed or cache unless this process owns the package lock.
			if (!acquired)
				throw new IOException($"Timed out waiting to synchronise package outputs for '{cache_root}'.");

			// Snapshot the staged archive once so a concurrent pack cannot make the feed, cache package, extracted payload, and hashes disagree.
			var package_bytes = Array.Empty<byte>();
			RetryLockedFile(() => package_bytes = IOFile.ReadAllBytes(staged_nupkg_full), staged_nupkg_full);

			// Remove NuGet's completion marker before mutation so concurrent restores never accept a partially refreshed cache entry.
			RetryLockedFile(() =>
			{
				ClearReadOnly(cache_metadata);
				IOFile.Delete(cache_metadata);
			}, cache_metadata);

			// Publish identical archive bytes to the repository feed and the global package cache.
			WritePackageAtomically(feed_nupkg_full, package_bytes);
			WritePackageAtomically(cache_nupkg, package_bytes);

			// Extract each archive entry while retaining an exact inventory for stale-payload removal.
			using (var package_stream = new MemoryStream(package_bytes, false))
			using (var archive = new ZipArchive(package_stream, ZipArchiveMode.Read))
			{
				foreach (var entry in archive.Entries)
				{
					var target_path = GetCachePath(cache_root, entry.FullName);

					// Directory entries establish empty package directories without producing payload files.
					if (string.IsNullOrEmpty(entry.Name))
					{
						AddDirectoryWithParents(keep_dirs, cache_root, target_path);
						Directory.CreateDirectory(target_path);
						continue;
					}

					// Every extracted file becomes part of the authoritative cache payload.
					keep_files.Add(target_path);

					// Materialize and retain parent directories before writing the file.
					var target_dir = Path.GetDirectoryName(target_path);
					if (!string.IsNullOrEmpty(target_dir))
					{
						AddDirectoryWithParents(keep_dirs, cache_root, target_dir);
						Directory.CreateDirectory(target_dir);
					}

					// Replace each payload file while tolerating short-lived reader locks.
					RetryLockedFile(() =>
					{
						ClearReadOnly(target_path);
						using var input = entry.Open();
						using var output = new FileStream(target_path, FileMode.Create, FileAccess.Write, FileShare.None);
						input.CopyTo(output);
					}, target_path);
				}
			}

			// Remove files left behind by an older archive of the same package version.
			RemoveStalePayload(cache_root, keep_files, keep_dirs, cache_nupkg, cache_metadata);

			// NuGet records the archive SHA-512 beside the cached package.
			string hash;
			using (var sha = SHA512.Create())
			{
				hash = Convert.ToBase64String(sha.ComputeHash(package_bytes));
			}

			// Write the completion metadata last so readers never accept a partially refreshed cache.
			RetryLockedFile(() => IOFile.WriteAllText(cache_sha512, hash), cache_sha512);
			var metadata = $@"{{""version"":2,""contentHash"":""{hash}"",""source"":null}}";
			RetryLockedFile(() => IOFile.WriteAllText(cache_metadata, metadata), cache_metadata);
		}
		finally
		{
			if (acquired)
				mutex.ReleaseMutex();
		}
	}

	// Replace a package archive through a same-directory temporary file so readers never observe partial bytes.
	private static void WritePackageAtomically(string destination_path, byte[] package_bytes)
	{
		var directory = Path.GetDirectoryName(destination_path) ?? throw new InvalidOperationException($"Unable to determine the package directory for '{destination_path}'.");
		var temporary_path = Path.Combine(directory, $".{Path.GetFileName(destination_path)}.{Guid.NewGuid():N}.tmp");
		try
		{
			RetryLockedFile(() => IOFile.WriteAllBytes(temporary_path, package_bytes), temporary_path);
			RetryLockedFile(() =>
			{
				if (IOFile.Exists(destination_path))
				{
					ClearReadOnly(destination_path);
					IOFile.Replace(temporary_path, destination_path, null, true);
				}
				else
				{
					IOFile.Move(temporary_path, destination_path);
				}
			}, destination_path);
		}
		finally
		{
			if (IOFile.Exists(temporary_path))
			{
				ClearReadOnly(temporary_path);
				IOFile.Delete(temporary_path);
			}
		}
	}

	// Remove payload files and directories that are not present in the newly staged archive.
	private static void RemoveStalePayload(string cache_root, HashSet<string> keep_files, HashSet<string> keep_dirs, string cache_nupkg, string cache_metadata)
	{
		var files_to_delete = new List<string>();
		foreach (var file in Directory.EnumerateFiles(cache_root, "*", SearchOption.AllDirectories))
		{
			if (keep_files.Contains(file))
				continue;

			// Preserve NuGet's package archive, checksum, and completion metadata.
			if (string.Equals(file, cache_nupkg, StringComparison.OrdinalIgnoreCase) ||
				string.Equals(file, cache_nupkg + ".sha512", StringComparison.OrdinalIgnoreCase) ||
				string.Equals(file, cache_metadata, StringComparison.OrdinalIgnoreCase))
			{
				continue;
			}

			// Queue payload removal until enumeration has completed.
			files_to_delete.Add(file);
		}

		// Delete stale files with the same bounded lock retry used during extraction.
		foreach (var file in files_to_delete)
		{
			RetryLockedFile(() =>
			{
				ClearReadOnly(file);
				IOFile.Delete(file);
			}, file);
		}

		// Collect stale directories separately so they can be removed from deepest to shallowest.
		var dirs_to_delete = new List<string>();
		foreach (var dir in Directory.EnumerateDirectories(cache_root, "*", SearchOption.AllDirectories))
		{
			if (keep_dirs.Contains(dir))
				continue;

			// Queue directory removal until enumeration has completed.
			dirs_to_delete.Add(dir);
		}

		// Delete children before parents so every queued directory is empty when removed.
		dirs_to_delete.Sort((lhs, rhs) => rhs.Length.CompareTo(lhs.Length));
		foreach (var dir in dirs_to_delete)
		{
			RetryLockedFile(() =>
			{
				ClearReadOnly(dir);
				Directory.Delete(dir, false);
			}, dir);
		}
	}

	// Retry only Windows sharing and lock violations until the configured timeout expires.
	private static void RetryLockedFile(Action action, string path)
	{
		var timer = Stopwatch.StartNew();
		var delay = 50;

		// Retry with bounded exponential backoff while another process holds a transient file lock.
		for (;;)
		{
			try
			{
				action();
				return;
			}
			catch (IOException ex) when (IsLockViolation(ex) && timer.ElapsedMilliseconds < 15000)
			{
				Thread.Sleep(delay);
				delay = Math.Min(delay * 2, 500);
			}
			catch (IOException ex) when (IsLockViolation(ex))
			{
				throw new IOException($"Timed out waiting to update '{path}'.", ex);
			}
		}
	}

	// Remove the read-only attribute to match MSBuild's previous OverwriteReadOnlyFiles behaviour.
	private static void ClearReadOnly(string path)
	{
		if (!IOFile.Exists(path) && !Directory.Exists(path))
			return;

		// Preserve every attribute except read-only so replacement and deletion can proceed.
		var attributes = IOFile.GetAttributes(path);
		if ((attributes & FileAttributes.ReadOnly) != 0)
		{
			IOFile.SetAttributes(path, attributes & ~FileAttributes.ReadOnly);
		}
	}

	// Recognise the Windows errors that indicate another process temporarily owns the destination file.
	private static bool IsLockViolation(IOException ex)
	{
		var error_code = ex.HResult & 0xFFFF;
		return error_code == 32 || error_code == 33;
	}

	// Resolve a package entry path relative to the cache root and reject any attempt to escape it.
	private static string GetCachePath(string cache_root, string entry_name)
	{
		var relative_path = entry_name.Replace('/', Path.DirectorySeparatorChar);
		var cache_root_prefix = cache_root + Path.DirectorySeparatorChar;
		var target_path = Path.GetFullPath(Path.Combine(cache_root, relative_path));
		if (!target_path.StartsWith(cache_root_prefix, StringComparison.OrdinalIgnoreCase))
		{
			throw new InvalidDataException($"Package entry '{entry_name}' is outside the cache directory.");
		}

		// Return only paths proven to remain beneath the package cache root.
		return target_path;
	}

	// Ensure a directory and all of its parents remain in the extracted payload when removing stale cache entries.
	private static void AddDirectoryWithParents(HashSet<string> keep_dirs, string cache_root, string directory_path)
	{
		for (var dir = Path.GetFullPath(directory_path).TrimEnd(Path.DirectorySeparatorChar); !string.Equals(dir, cache_root, StringComparison.OrdinalIgnoreCase); dir = Path.GetDirectoryName(dir) ?? cache_root)
		{
			keep_dirs.Add(dir);
		}

		// Retain the cache root itself even when the package has no nested directories.
		keep_dirs.Add(cache_root);
	}

	// Normalise a directory path once so later comparisons can use stable, separator-free values.
	private static string NormalizeDirectory(string path)
	{
		return Path.GetFullPath(path).TrimEnd(Path.DirectorySeparatorChar);
	}

	// Derive a stable cross-process mutex name without exposing path separators or length-sensitive cache paths.
	private static string MutexName(string cache_dir)
	{
		byte[] hash;
		using (var sha = SHA256.Create())
		{
			hash = sha.ComputeHash(System.Text.Encoding.UTF8.GetBytes(Path.GetFullPath(cache_dir).ToUpperInvariant()));
		}

		// Encode the path hash as a Windows-safe local mutex name.
		return "Local\\Rylogic.NuGetCache." + BitConverter.ToString(hash).Replace("-", string.Empty);
	}
}
