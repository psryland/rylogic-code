using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.IO.Compression;
using System.Security.Cryptography;
using System.Threading;
using Microsoft.Build.Framework;
using Microsoft.Build.Utilities;

// Syncs a locally built package into the canonical local feed and NuGet's global cache while tolerating transient locks from parallel Visual Studio builds.
public sealed class SyncNuGetPackageOutputs : Task
{
	[Required]
	public string NuPkgPath
	{
		get;
		set;
	} = string.Empty;

	[Required]
	public string FeedPkgPath
	{
		get;
		set;
	} = string.Empty;

	[Required]
	public string CacheDir
	{
		get;
		set;
	} = string.Empty;

	[Required]
	public string NuPkgName
	{
		get;
		set;
	} = string.Empty;

	[Required]
	public int RetryTimeoutMilliseconds
	{
		get;
		set;
	}

	[Output]
	public bool Synced
	{
		get;
		set;
	}

	// Sync the complete package before reporting success to the owning MSBuild target.
	public override bool Execute()
	{
		Synced = false;

		try
		{
			// Serialise writers across MSBuild processes so package files and metadata remain consistent.
			using (var mutex = new Mutex(false, MutexName(CacheDir)))
			{
				var acquired = false;
				try
				{
					try
					{
						acquired = mutex.WaitOne(RetryTimeoutMilliseconds);
					}
					catch (AbandonedMutexException)
					{
						acquired = true;
					}

					if (!acquired)
					{
						Log.LogError("Timed out waiting to sync package outputs for '{0}'.", CacheDir);
						return false;
					}

					SyncPackage();
				}
				finally
				{
					if (acquired)
					{
						mutex.ReleaseMutex();
					}
				}
			}
		}
		catch (IOException ex)
		{
			Log.LogError("Failed to sync package outputs for '{0}': {1}", CacheDir, ex.Message);
			return false;
		}
		catch (UnauthorizedAccessException ex)
		{
			Log.LogError("Failed to sync package outputs for '{0}': {1}", CacheDir, ex.Message);
			return false;
		}
		catch (InvalidDataException ex)
		{
			Log.LogError("Failed to read NuGet package '{0}': {1}", NuPkgPath, ex.Message);
			return false;
		}
		catch (CryptographicException ex)
		{
			Log.LogError("Failed to hash NuGet package '{0}': {1}", NuPkgPath, ex.Message);
			return false;
		}

		Synced = true;
		return true;
	}

	// Copy the staged package to the canonical feed and exact global-cache location, then refresh the extracted payload in place.
	private void SyncPackage()
	{
		var cache_root = NormalizeDirectory(CacheDir);
		var staged_nupkg = Path.GetFullPath(NuPkgPath);
		var feed_nupkg = Path.GetFullPath(FeedPkgPath);
		var cache_nupkg = Path.Combine(cache_root, NuPkgName);
		var cache_metadata = Path.Combine(cache_root, ".nupkg.metadata");
		var cache_sha512 = cache_nupkg + ".sha512";
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

		Directory.CreateDirectory(cache_root);
		Directory.CreateDirectory(Path.GetDirectoryName(feed_nupkg) ?? throw new InvalidOperationException($"Unable to determine the feed directory for '{feed_nupkg}'"));

		// Snapshot the staged archive once so a concurrent pack cannot make the feed, cache package, extracted payload, and hashes disagree.
		var package_bytes = Array.Empty<byte>();
		RetryLockedFile(
			() => package_bytes = File.ReadAllBytes(staged_nupkg),
			staged_nupkg);

		// Remove NuGet's completion marker before mutation so concurrent restores never accept a partially refreshed cache entry.
		RetryLockedFile(
			() =>
			{
				ClearReadOnly(cache_metadata);
				File.Delete(cache_metadata);
			},
			cache_metadata);

		// Publish complete archives atomically so concurrent restores see either the previous package or the new package.
		WritePackageAtomically(feed_nupkg, package_bytes);

		// Keep the cache package file itself exact so hash-based consumers observe the same payload as the local feed.
		WritePackageAtomically(cache_nupkg, package_bytes);

		// Refresh every package payload file without removing the cache directory used by concurrent readers.
		using (var package_stream = new MemoryStream(package_bytes, false))
		using (var archive = new ZipArchive(package_stream, ZipArchiveMode.Read))
		{
			foreach (var entry in archive.Entries)
			{
				var target_path = GetCachePath(cache_root, entry.FullName);

				if (string.IsNullOrEmpty(entry.Name))
				{
					AddDirectoryWithParents(keep_dirs, cache_root, target_path);
					Directory.CreateDirectory(target_path);
					continue;
				}

				keep_files.Add(target_path);

				var target_dir = Path.GetDirectoryName(target_path);
				if (!string.IsNullOrEmpty(target_dir))
				{
					AddDirectoryWithParents(keep_dirs, cache_root, target_dir);
					Directory.CreateDirectory(target_dir);
				}

				RetryLockedFile(
					() =>
					{
						ClearReadOnly(target_path);
						using (var input = entry.Open())
						using (var output = new FileStream(target_path, FileMode.Create, FileAccess.Write, FileShare.None))
						{
							input.CopyTo(output);
						}
					},
					target_path);
			}
		}

		// Remove payload files and directories that disappeared from the new archive while preserving the package sidecars.
		RemoveStalePayload(cache_root, keep_files, keep_dirs, cache_nupkg, cache_metadata);

		string hash;
		using (var sha = SHA512.Create())
		{
			hash = Convert.ToBase64String(sha.ComputeHash(package_bytes));
		}

		// Publish the package hash after the payload is fully in place so restore can see a coherent package record.
		RetryLockedFile(
			() => File.WriteAllText(cache_sha512, hash),
			cache_sha512);

		var metadata = $@"{{""version"":2,""contentHash"":""{hash}"",""source"":null}}";
		// Write the NuGet metadata last so readers only see the refreshed package once the payload and hash agree.
		RetryLockedFile(
			() => File.WriteAllText(cache_metadata, metadata),
			cache_metadata);
	}

	// Replace a package archive through a same-directory temporary file so readers never observe partial bytes.
	private void WritePackageAtomically(string destination_path, byte[] package_bytes)
	{
		var directory = Path.GetDirectoryName(destination_path) ?? throw new InvalidOperationException($"Unable to determine the package directory for '{destination_path}'.");
		var temporary_path = Path.Combine(directory, $".{Path.GetFileName(destination_path)}.{Guid.NewGuid():N}.tmp");
		try
		{
			RetryLockedFile(
				() => File.WriteAllBytes(temporary_path, package_bytes),
				temporary_path);
			RetryLockedFile(
				() =>
				{
					if (File.Exists(destination_path))
					{
						ClearReadOnly(destination_path);
						File.Replace(temporary_path, destination_path, null, true);
					}
					else
					{
						File.Move(temporary_path, destination_path);
					}
				},
				destination_path);
		}
		finally
		{
			if (File.Exists(temporary_path))
			{
				ClearReadOnly(temporary_path);
				File.Delete(temporary_path);
			}
		}
	}

	// Remove payload files and directories that are not present in the newly staged archive.
	private void RemoveStalePayload(string cache_root, HashSet<string> keep_files, HashSet<string> keep_dirs, string cache_nupkg, string cache_metadata)
	{
		var files_to_delete = new List<string>();
		foreach (var file in Directory.EnumerateFiles(cache_root, "*", SearchOption.AllDirectories))
		{
			if (keep_files.Contains(file))
			{
				continue;
			}
			if (string.Equals(file, cache_nupkg, StringComparison.OrdinalIgnoreCase) ||
				string.Equals(file, cache_nupkg + ".sha512", StringComparison.OrdinalIgnoreCase) ||
				string.Equals(file, cache_metadata, StringComparison.OrdinalIgnoreCase))
			{
				continue;
			}

			files_to_delete.Add(file);
		}

		foreach (var file in files_to_delete)
		{
			RetryLockedFile(
				() =>
				{
					ClearReadOnly(file);
					File.Delete(file);
				},
				file);
		}

		var dirs_to_delete = new List<string>();
		foreach (var dir in Directory.EnumerateDirectories(cache_root, "*", SearchOption.AllDirectories))
		{
			if (keep_dirs.Contains(dir))
			{
				continue;
			}

			dirs_to_delete.Add(dir);
		}

		dirs_to_delete.Sort((lhs, rhs) => rhs.Length.CompareTo(lhs.Length));
		foreach (var dir in dirs_to_delete)
		{
			RetryLockedFile(
				() =>
				{
					ClearReadOnly(dir);
					Directory.Delete(dir, false);
				},
				dir);
		}
	}

	// Retry only Windows sharing and lock violations until the configured timeout expires.
	private void RetryLockedFile(Action action, string path)
	{
		var timer = Stopwatch.StartNew();
		var delay = 50;

		for (;;)
		{
			try
			{
				action();
				return;
			}
			catch (IOException ex) when (IsLockViolation(ex) && timer.ElapsedMilliseconds < RetryTimeoutMilliseconds)
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

	// Remove the read-only attribute to match MSBuild's previous OverwriteReadOnlyFiles behavior.
	private static void ClearReadOnly(string path)
	{
		if (!File.Exists(path) && !Directory.Exists(path))
		{
			return;
		}

		var attributes = File.GetAttributes(path);
		if ((attributes & FileAttributes.ReadOnly) != 0)
		{
			File.SetAttributes(path, attributes & ~FileAttributes.ReadOnly);
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

		return target_path;
	}

	// Ensure a directory and all of its parents remain in the extracted payload when removing stale cache entries.
	private static void AddDirectoryWithParents(HashSet<string> keep_dirs, string cache_root, string directory_path)
	{
		for (var dir = Path.GetFullPath(directory_path).TrimEnd(Path.DirectorySeparatorChar); !string.Equals(dir, cache_root, StringComparison.OrdinalIgnoreCase); dir = Path.GetDirectoryName(dir) ?? cache_root)
		{
			keep_dirs.Add(dir);
		}

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

		return "Local\\Rylogic.NuGetCache." + BitConverter.ToString(hash).Replace("-", string.Empty);
	}
}
