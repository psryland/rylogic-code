using System;
using System.IO;
using System.Security.Cryptography;
using System.Text;
using System.Text.RegularExpressions;
using System.Threading;
using Microsoft.Build.Framework;
using Microsoft.Build.Utilities;

// Publishes the newest immutable local package version through MSBuild props without allowing parallel builds to move the pointer backwards.
public sealed class PublishLocalPackageVersion : Task
{
	private static readonly Regex s_version_pattern = new(@"(?<version>\d+\.\d+\.\d+-dev\.\d+)", RegexOptions.CultureInvariant);

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
