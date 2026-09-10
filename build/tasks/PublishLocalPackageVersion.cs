using System;
using System.Collections.Generic;
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

	// Parses "major.minor.patch" with an optional "-dev.<timestamp>" suffix so versions can be ordered numerically rather than lexically.
	private static readonly Regex s_semantic_version_pattern = new(@"^(?<major>\d+)\.(?<minor>\d+)\.(?<patch>\d+)(?:-dev\.(?<timestamp>\d+))?$", RegexOptions.CultureInvariant);

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
				if (current_version != null && CompareVersions(current_version, PackageVersion) >= 0)
				{
					// This build's own version did not advance the pointer, but its cache folder may have just been (re)written by a slow Sync step that
					// completed after a faster, newer build already published and cleaned up. Sweep again against the real pointer so that stray folder
					// does not survive indefinitely; the '>= 0' guard above still protects the true newest folder and everything at or above it.
					if (CleanupOldDevVersions)
						CleanupOldCacheVersions(current_version);

					return true;
				}

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
					CleanupOldCacheVersions(PackageVersion);

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

	// Remove every locally-cached '-dev.<timestamp>' version of PackageId that is strictly older than baseline_version (the real, currently-published pointer).
	// Stable releases, malformed folder names, and anything not matching this build system's owned dev-version shape are left untouched.
	private void CleanupOldCacheVersions(string baseline_version)
	{
		// Only ever clean up around a version in this build system's own owned dev-version shape; refuse to run destructive cleanup off an unexpected baseline.
		if (!s_owned_dev_version_pattern.IsMatch(baseline_version))
			return;

		var cache_root = string.IsNullOrWhiteSpace(NuGetCacheRoot)
			? Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.UserProfile), ".nuget", "packages")
			: NuGetCacheRoot;
		var package_cache_dir = Path.Combine(cache_root, PackageId.ToLowerInvariant());
		if (!Directory.Exists(package_cache_dir))
			return;

		// A linked package root could otherwise redirect the entire cleanup outside the intended cache; refuse to scan through it.
		if ((new DirectoryInfo(package_cache_dir).Attributes & FileAttributes.ReparsePoint) != 0)
		{
			Log.LogWarning("Skipped native package cache cleanup for '{0}' because the package cache directory itself is a reparse point.", package_cache_dir);
			return;
		}

		// A junction anywhere above the package cache directory (for example on the shared NuGet cache root) would make the directory's own
		// attributes look ordinary while still redirecting cleanup outside the intended tree; refuse to scan through it either.
		if (ContainsReparsePointAncestor(package_cache_dir, out var ancestor_reason))
		{
			Log.LogWarning("Skipped native package cache cleanup for '{0}' because {1}.", package_cache_dir, ancestor_reason);
			return;
		}

		foreach (var version_dir in Directory.EnumerateDirectories(package_cache_dir))
		{
			var version_name = Path.GetFileName(version_dir);
			if (!s_owned_dev_version_pattern.IsMatch(version_name))
				continue;

			// Never remove the currently-published version, or anything not strictly older than it.
			if (CompareVersions(version_name, baseline_version) >= 0)
				continue;

			TryDeleteVersionDirectory(version_dir);
		}
	}

	// Walk a directory and its full subtree exactly once, inspecting each entry's attributes before descending so a reparse point
	// (junction/symlink) is always detected and never itself followed. Returns the plain files and plain directories discovered, each
	// list in parent-before-child traversal order, so nothing later needs to re-enumerate the filesystem to act on the same tree.
	// Fails with a reason - and an inventory that must be discarded - the moment any reparse point is found anywhere, including the root.
	private static bool TryInventoryVersionDirectory(string root, out List<string> files, out List<string> directories, out string reason)
	{
		files = new List<string>();
		directories = new List<string>();

		if ((new DirectoryInfo(root).Attributes & FileAttributes.ReparsePoint) != 0)
		{
			reason = $"'{root}' is a reparse point";
			return false;
		}

		var pending = new Stack<string>();
		pending.Push(root);

		while (pending.Count != 0)
		{
			var dir = pending.Pop();

			foreach (var file in Directory.EnumerateFiles(dir))
			{
				if ((File.GetAttributes(file) & FileAttributes.ReparsePoint) != 0)
				{
					reason = $"'{file}' is a reparse point";
					return false;
				}

				files.Add(file);
			}

			foreach (var sub_dir in Directory.EnumerateDirectories(dir))
			{
				if ((new DirectoryInfo(sub_dir).Attributes & FileAttributes.ReparsePoint) != 0)
				{
					reason = $"'{sub_dir}' is a reparse point";
					return false;
				}

				// Record this directory before descending into it, so the list stays in parent-before-child order and a junction's
				// target is never enumerated.
				directories.Add(sub_dir);
				pending.Push(sub_dir);
			}
		}

		reason = null;
		return true;
	}

	// Walk every ancestor directory from the package cache directory up to the filesystem root, so a junction anywhere above it (for
	// example on the shared NuGet cache root itself) cannot make the package cache directory's own attributes look ordinary while still
	// redirecting cleanup outside the intended package cache tree.
	private static bool ContainsReparsePointAncestor(string path, out string reason)
	{
		var current = new DirectoryInfo(path).Parent;
		while (current != null)
		{
			if ((current.Attributes & FileAttributes.ReparsePoint) != 0)
			{
				reason = $"ancestor '{current.FullName}' is a reparse point";
				return true;
			}

			current = current.Parent;
		}

		reason = null;
		return false;
	}

	// Delete one superseded cache version using a single reparse-safe inventory taken under the cache-entry mutex, followed by held file
	// handles for the whole operation so a version still in use (e.g. a native DLL loaded by a running process) is left completely
	// untouched rather than partially deleted - unlike a preflight-then-release check, no other process can open a new reader or writer
	// on an inventoried file in the gap between checking and deleting. Holding a file's handle does not, however, stop something else
	// from adding a brand new entry to the directory; any such unexpected content simply makes the final non-recursive directory removal
	// fail (never descend into it), leaving that directory - and the retry on the next successful publish - to sort it out.
	// Serialises against SyncNuGetPackageOutputs writing into the same directory by reusing its exact per-cache-directory mutex naming scheme.
	private void TryDeleteVersionDirectory(string version_dir)
	{
		using var version_mutex = new Mutex(false, CacheEntryMutexName(version_dir));
		var acquired = false;
		var streams = new List<FileStream>();
		try
		{
			try
			{
				acquired = version_mutex.WaitOne(TimeSpan.FromSeconds(15));
			}
			catch (AbandonedMutexException)
			{
				acquired = true;
			}

			if (!acquired)
			{
				Log.LogWarning("Timed out waiting to remove superseded cached package version '{0}'; a concurrent sync may be in progress. It will be retried after the next successful publish.", version_dir);
				return;
			}

			// Another cleanup pass may already have removed this directory while this one waited for the lock.
			if (!Directory.Exists(version_dir))
				return;

			// Take the one and only inventory of this subtree now that the cache-entry mutex is held, closing the window between any
			// earlier scan and the point at which deletion actually begins. Everything from here on acts strictly on this exact file
			// and directory list - nothing is re-enumerated, so nothing can be re-traversed through a link introduced afterwards.
			if (!TryInventoryVersionDirectory(version_dir, out var files, out var directories, out var reparse_reason))
			{
				Log.LogWarning("Skipped cleanup of old cached package version '{0}' because {1}.", version_dir, reparse_reason);
				return;
			}

			// Acquire a handle on every inventoried file up front and hold all of them for the rest of this method. If any single
			// file is busy, everything acquired so far is released and the whole version is left byte-for-byte untouched.
			if (!TryAcquireVersionHandles(files, out streams, out var locked_path))
			{
				Log.LogWarning("Could not remove superseded cached package version '{0}' because '{1}' appears to be in use. It will be retried after the next successful publish.", version_dir, locked_path);
				return;
			}

			try
			{
				// Delete NuGet's completion marker first, while still holding its handle, so a version can never be observed as a
				// valid, complete cache entry once removal has begun. If an unexpected error interrupts what follows, the marker's
				// absence still guarantees NuGet cannot treat the remainder as a usable cache entry; it is retried on the next publish.
				var metadata_path = Path.Combine(version_dir, ".nupkg.metadata");
				if (File.Exists(metadata_path))
				{
					ClearReadOnly(metadata_path);
					File.Delete(metadata_path);
				}

				foreach (var file in files)
				{
					if (string.Equals(file, metadata_path, StringComparison.OrdinalIgnoreCase))
						continue;

					ClearReadOnly(file);
					File.Delete(file);
				}

				// Release every handle now that all inventoried files are deleted, before removing any directory.
				foreach (var stream in streams)
					stream.Dispose();
				streams.Clear();

				// Remove the inventoried directories in reverse of their parent-before-child discovery order (children before their
				// own parent), using only non-recursive deletes. A directory that unexpectedly contains something beyond what was
				// inventoried - for example a new file, or a link introduced after the scan above - fails here rather than being
				// descended into, and is retried on the next successful publish along with the rest of this version.
				for (var i = directories.Count - 1; i >= 0; --i)
					Directory.Delete(directories[i], recursive: false);

				Directory.Delete(version_dir, recursive: false);
			}
			catch (Exception ex) when (ex is IOException or UnauthorizedAccessException)
			{
				Log.LogWarning("Could not fully remove superseded cached package version '{0}' after removal began: {1}. It will be retried after the next successful publish.", version_dir, ex.Message);
			}
		}
		finally
		{
			foreach (var stream in streams)
				stream.Dispose();
			if (acquired)
				version_mutex.ReleaseMutex();
		}
	}

	// Open every file from an already-inventoried list with FileShare.Delete only (no Read or Write sharing), so no other process can
	// open a new reader or writer against any of them while the caller still holds these handles, yet this task can still delete them
	// itself. Returns false with everything already released if any single file cannot be opened this way. Does not itself enumerate
	// the filesystem, so it can never traverse anything beyond the exact files it is given.
	private static bool TryAcquireVersionHandles(List<string> files, out List<FileStream> streams, out string locked_path)
	{
		streams = new List<FileStream>();
		locked_path = null;

		foreach (var file in files)
		{
			try
			{
				streams.Add(new FileStream(file, FileMode.Open, FileAccess.Read, FileShare.Delete));
			}
			catch (Exception ex) when (ex is IOException or UnauthorizedAccessException)
			{
				locked_path = file;
				foreach (var stream in streams)
					stream.Dispose();
				streams.Clear();
				return false;
			}
		}

		return true;
	}

	// Remove the read-only attribute to match MSBuild's previous OverwriteReadOnlyFiles behavior.
	private static void ClearReadOnly(string path)
	{
		if (!File.Exists(path))
			return;

		var attributes = File.GetAttributes(path);
		if ((attributes & FileAttributes.ReadOnly) != 0)
			File.SetAttributes(path, attributes & ~FileAttributes.ReadOnly);
	}

	// Compare two locally-published package versions using numeric major.minor.patch ordering, then numeric dev-timestamp ordering, so a
	// version bump (e.g. 2.1.10 following 2.1.9) is never misordered by a plain lexical string comparison. Falls back to an ordinal
	// comparison for any version this build system did not itself generate.
	private static int CompareVersions(string lhs, string rhs)
	{
		var lhs_match = s_semantic_version_pattern.Match(lhs);
		var rhs_match = s_semantic_version_pattern.Match(rhs);
		if (!lhs_match.Success || !rhs_match.Success)
			return string.CompareOrdinal(lhs, rhs);

		try
		{
			var major_cmp = long.Parse(lhs_match.Groups["major"].Value).CompareTo(long.Parse(rhs_match.Groups["major"].Value));
			if (major_cmp != 0)
				return major_cmp;

			var minor_cmp = long.Parse(lhs_match.Groups["minor"].Value).CompareTo(long.Parse(rhs_match.Groups["minor"].Value));
			if (minor_cmp != 0)
				return minor_cmp;

			var patch_cmp = long.Parse(lhs_match.Groups["patch"].Value).CompareTo(long.Parse(rhs_match.Groups["patch"].Value));
			if (patch_cmp != 0)
				return patch_cmp;

			var lhs_timestamp = lhs_match.Groups["timestamp"].Success ? lhs_match.Groups["timestamp"].Value : string.Empty;
			var rhs_timestamp = rhs_match.Groups["timestamp"].Success ? rhs_match.Groups["timestamp"].Value : string.Empty;

			// A stable release (no '-dev' suffix) is always considered newer than any dev build of the same major.minor.patch.
			if (lhs_timestamp.Length == 0 && rhs_timestamp.Length == 0)
				return 0;
			if (lhs_timestamp.Length == 0)
				return 1;
			if (rhs_timestamp.Length == 0)
				return -1;

			// Equal-length numeric strings order identically under ordinal and numeric comparison; a length mismatch is decided by magnitude.
			return lhs_timestamp.Length == rhs_timestamp.Length
				? string.CompareOrdinal(lhs_timestamp, rhs_timestamp)
				: lhs_timestamp.Length.CompareTo(rhs_timestamp.Length);
		}
		catch (OverflowException)
		{
			return string.CompareOrdinal(lhs, rhs);
		}
	}

	// Reuse SyncNuGetPackageOutputs' exact mutex-naming scheme for one package version's cache directory, so a cleanup delete and a
	// concurrent sync into the same directory are always serialised against each other.
	private static string CacheEntryMutexName(string cache_dir)
	{
		byte[] hash;
		using (var sha256 = SHA256.Create())
		{
			hash = sha256.ComputeHash(Encoding.UTF8.GetBytes(Path.GetFullPath(cache_dir).ToUpperInvariant()));
		}

		return "Local\\Rylogic.NuGetCache." + BitConverter.ToString(hash).Replace("-", string.Empty);
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
