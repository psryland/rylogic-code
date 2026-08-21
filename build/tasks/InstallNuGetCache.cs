using System;
using System.Diagnostics;
using System.IO;
using System.IO.Compression;
using System.Security.Cryptography;
using System.Threading;
using Microsoft.Build.Framework;
using Microsoft.Build.Utilities;

// Installs a package into NuGet's global cache while tolerating transient locks from parallel Visual Studio builds.
public sealed class InstallNuGetCache : Task
{
	[Required]
	public string NuPkgPath
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
	public bool Installed
	{
		get;
		set;
	}

	// Install the complete package before reporting success to the owning MSBuild target.
	public override bool Execute()
	{
		Installed = false;

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
						Log.LogError("Timed out waiting to update NuGet cache '{0}'.", CacheDir);
						return false;
					}

					InstallPackage();
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
			Log.LogError("Failed to update NuGet cache '{0}': {1}", CacheDir, ex.Message);
			return false;
		}
		catch (UnauthorizedAccessException ex)
		{
			Log.LogError("Failed to update NuGet cache '{0}': {1}", CacheDir, ex.Message);
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

		Installed = true;
		return true;
	}

	// Extract package payload and write NuGet's sidecar files, publishing the completion metadata last.
	private void InstallPackage()
	{
		Directory.CreateDirectory(CacheDir);

		// Refresh every package payload file without removing the cache directory used by concurrent readers.
		using (var package_stream = File.OpenRead(NuPkgPath))
		using (var archive = new ZipArchive(package_stream, ZipArchiveMode.Read))
		{
			foreach (var entry in archive.Entries)
			{
				var relative_path = entry.FullName.Replace('/', Path.DirectorySeparatorChar);
				var target_path = Path.GetFullPath(Path.Combine(CacheDir, relative_path));
				if (!target_path.StartsWith(Path.GetFullPath(CacheDir) + Path.DirectorySeparatorChar, StringComparison.OrdinalIgnoreCase))
				{
					throw new InvalidDataException($"Package entry '{entry.FullName}' is outside the cache directory.");
				}

				if (string.IsNullOrEmpty(entry.Name))
				{
					Directory.CreateDirectory(target_path);
					continue;
				}

				var target_dir = Path.GetDirectoryName(target_path);
				if (!string.IsNullOrEmpty(target_dir))
				{
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

		// Cache the original package before recording the hash that identifies this exact payload.
		var cached_package_path = Path.Combine(CacheDir, NuPkgName);
		RetryLockedFile(
			() =>
			{
				ClearReadOnly(cached_package_path);
				File.Copy(NuPkgPath, cached_package_path, true);
			},
			cached_package_path);

		string hash;
		using (var sha = SHA512.Create())
		{
			hash = Convert.ToBase64String(sha.ComputeHash(File.ReadAllBytes(NuPkgPath)));
		}

		RetryLockedFile(
			() => File.WriteAllText(cached_package_path + ".sha512", hash),
			cached_package_path + ".sha512");

		var metadata_path = Path.Combine(CacheDir, ".nupkg.metadata");
		var metadata = $@"{{""version"":2,""contentHash"":""{hash}"",""source"":null}}";
		RetryLockedFile(
			() => File.WriteAllText(metadata_path, metadata),
			metadata_path);
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
		if (!File.Exists(path))
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
