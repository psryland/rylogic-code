using System;
using System.IO;
using System.Security.Cryptography;
using System.Text;
using System.Text.RegularExpressions;
using System.Threading;
using Microsoft.Build.Framework;
using Microsoft.Build.Utilities;

// Publishes the newest immutable local package version through MSBuild props without allowing parallel builds to move the pointer backwards.
// Callers that opt in via CleanupOldDevVersions also have every older locally-cached '-dev.<timestamp>' build of the same package trimmed down to the one just published.
public sealed class PublishLocalPackageVersion : Task
{
	private static readonly Regex s_version_pattern = new(@"(?<version>\d+\.\d+\.\d+-dev\.\d+)", RegexOptions.CultureInvariant);

	// Matches only the exact "X.Y.Z-dev.<17-digit UTC timestamp>" shape this build system generates (yyyyMMddHHmmssfff).
	// Stable release folders, and anything hand-placed or from another tool, never match and are therefore never touched.
	private static readonly Regex s_owned_dev_version_pattern = new(@"^\d+\.\d+\.\d+-dev\.\d{17}$", RegexOptions.CultureInvariant);

	[Required]
	public string PackageId
	{
		get;
		set;
	} = string.Empty;

	[Required]
	public string PackageVersion
	{
		get;
		set;
	} = string.Empty;

	[Required]
	public string CentralVersionPropsPath
	{
		get;
		set;
	} = string.Empty;

	public string ProjectVersionPropsPath
	{
		get;
		set;
	} = string.Empty;

	public string ProjectVersionProperty
	{
		get;
		set;
	} = string.Empty;

	// Opt-in: once this build's version is confirmed the newest, delete every other locally-cached '-dev.<timestamp>' version of this exact package.
	// Left false for ordinary managed packages; the native Rylogic.Native development package opts in explicitly.
	public bool CleanupOldDevVersions
	{
		get;
		set;
	}

	// Root of the global NuGet cache to clean (the directory that directly contains one folder per lower-cased package id).
	// Defaults to the current user's cache when not supplied.
	public string NuGetCacheRoot
	{
		get;
		set;
	} = string.Empty;

	[Output]
	public bool Published
	{
		get;
		set;
	}

	// Atomically advance the package pointer when this build produced a newer timestamped revision.
	public override bool Execute()
	{
		Published = false;
		try
		{
			using var mutex = new Mutex(false, MutexName(CentralVersionPropsPath));
			var acquired = false;
			try
			{
				try
				{
					acquired = mutex.WaitOne(TimeSpan.FromSeconds(15));
				}
				catch (AbandonedMutexException)
				{
					acquired = true;
				}

				if (!acquired)
				{
					Log.LogError("Timed out waiting to publish local package version '{0}'.", CentralVersionPropsPath);
					return false;
				}

				// A slower parallel pack must not overwrite the pointer published by a newer build.
				var current_version = ReadVersion(CentralVersionPropsPath);
				if (current_version != null && string.CompareOrdinal(current_version, PackageVersion) >= 0)
					return true;

				var central_content = $"<Project><ItemGroup><PackageVersion Update=\"{PackageId}\" Version=\"{PackageVersion}\" /></ItemGroup></Project>{Environment.NewLine}";
				WriteAtomically(CentralVersionPropsPath, central_content);

				if (!string.IsNullOrWhiteSpace(ProjectVersionPropsPath) && !string.IsNullOrWhiteSpace(ProjectVersionProperty))
				{
					var project_content = $"<Project><PropertyGroup><{ProjectVersionProperty}>{PackageVersion}</{ProjectVersionProperty}></PropertyGroup></Project>{Environment.NewLine}";
					WriteAtomically(ProjectVersionPropsPath, project_content);
				}

				Published = true;

				// Still holding the lock guards this against a slower concurrent build re-ordering the pointer between the publish above and the cleanup below.
				if (CleanupOldDevVersions)
					CleanupOldCacheVersions();

				return true;
			}
			finally
			{
				if (acquired)
					mutex.ReleaseMutex();
			}
		}
		catch (Exception ex) when (ex is IOException or UnauthorizedAccessException or CryptographicException)
		{
			Log.LogError("Failed to publish local package version '{0}': {1}", PackageVersion, ex.Message);
			return false;
		}
	}

	// Remove every locally-cached '-dev.<timestamp>' version of PackageId that is strictly older than the version just published.
	// Stable releases, malformed folder names, and anything not matching this build system's owned dev-version shape are left untouched.
	private void CleanupOldCacheVersions()
	{
		var cache_root = string.IsNullOrWhiteSpace(NuGetCacheRoot)
			? Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.UserProfile), ".nuget", "packages")
			: NuGetCacheRoot;
		var package_cache_dir = Path.Combine(cache_root, PackageId.ToLowerInvariant());
		if (!Directory.Exists(package_cache_dir))
			return;

		foreach (var version_dir in Directory.EnumerateDirectories(package_cache_dir))
		{
			var version_name = Path.GetFileName(version_dir);
			if (!s_owned_dev_version_pattern.IsMatch(version_name))
				continue;

			// Never remove the version just published, or anything not strictly older than it (defensive against unexpected pointer drift).
			if (string.CompareOrdinal(version_name, PackageVersion) >= 0)
				continue;

			// A junction/symlink inside the version folder could otherwise cause a recursive delete to reach outside the NuGet cache; skip it and retry on the next successful publish.
			if (ContainsReparsePoint(version_dir))
			{
				Log.LogWarning("Skipped cleanup of old cached package version '{0}' because it contains a reparse point.", version_dir);
				continue;
			}

			TryDeleteVersionDirectory(version_dir);
		}
	}

	// Check a directory and its full subtree for reparse points (junctions/symlinks) before it is scheduled for deletion.
	private static bool ContainsReparsePoint(string root)
	{
		if ((new DirectoryInfo(root).Attributes & FileAttributes.ReparsePoint) != 0)
			return true;

		foreach (var directory in Directory.EnumerateDirectories(root, "*", SearchOption.AllDirectories))
		{
			if ((new DirectoryInfo(directory).Attributes & FileAttributes.ReparsePoint) != 0)
				return true;
		}

		return false;
	}

	// Best-effort delete of one superseded cache version. A file still held open (e.g. a native DLL loaded by a running process)
	// is retried briefly, then left in place with an actionable warning; the next successful publish retries the removal.
	private void TryDeleteVersionDirectory(string version_dir)
	{
		var started_at = DateTime.UtcNow;
		var delay_ms = 50;
		for (; ; )
		{
			try
			{
				Directory.Delete(version_dir, recursive: true);
				return;
			}
			catch (Exception ex) when (ex is IOException or UnauthorizedAccessException)
			{
				if ((DateTime.UtcNow - started_at).TotalMilliseconds >= 2000)
				{
					Log.LogWarning("Could not remove superseded cached package version '{0}' because it appears to be in use: {1}. It will be retried after the next successful publish.", version_dir, ex.Message);
					return;
				}

				Thread.Sleep(delay_ms);
				delay_ms = Math.Min(delay_ms * 2, 500);
			}
		}
	}

	// Read the timestamped package version already exposed by a generated props file.
	private static string ReadVersion(string path)
	{
		if (!File.Exists(path))
			return null;

		var match = s_version_pattern.Match(File.ReadAllText(path));
		return match.Success ? match.Groups["version"].Value : null;
	}

	// Replace one generated props file without exposing partial XML to concurrent restore evaluation.
	private static void WriteAtomically(string path, string content)
	{
		var directory = Path.GetDirectoryName(Path.GetFullPath(path)) ?? throw new InvalidOperationException($"No parent directory exists for '{path}'.");
		Directory.CreateDirectory(directory);

		var temporary_path = Path.Combine(directory, $".{Path.GetFileName(path)}.{Guid.NewGuid():N}.tmp");
		try
		{
			File.WriteAllText(temporary_path, content, new UTF8Encoding(false));
			if (File.Exists(path))
				File.Replace(temporary_path, path, null);
			else
				File.Move(temporary_path, path);
		}
		finally
		{
			File.Delete(temporary_path);
		}
	}

	// Derive a process-independent mutex name from the canonical props path.
	private static string MutexName(string path)
	{
		var normalized_path = Path.GetFullPath(path).ToUpperInvariant();
		using var sha256 = SHA256.Create();
		var hash = sha256.ComputeHash(Encoding.UTF8.GetBytes(normalized_path));
		return $@"Local\Rylogic.PackageVersion.{BitConverter.ToString(hash).Replace("-", string.Empty)}";
	}
}
