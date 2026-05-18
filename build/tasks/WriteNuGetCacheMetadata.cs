using System;
using System.IO;
using System.Security.Cryptography;
using Microsoft.Build.Framework;
using Microsoft.Build.Utilities;

// MSBuild task for writing the metadata files NuGet expects in its global package cache.
public sealed class WriteNuGetCacheMetadata : Task
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

	// Compute the package content hash and write NuGet's sidecar cache files.
	public override bool Execute()
	{
		try
		{
			Directory.CreateDirectory(CacheDir);

			string hash;
			using (var sha = SHA512.Create())
			{
				hash = Convert.ToBase64String(sha.ComputeHash(File.ReadAllBytes(NuPkgPath)));
			}

			File.WriteAllText(Path.Combine(CacheDir, $"{NuPkgName}.sha512"), hash);
			File.WriteAllText(Path.Combine(CacheDir, ".nupkg.metadata"), $@"{{""version"":2,""contentHash"":""{hash}"",""source"":null}}");
		}
		catch (Exception ex)
		{
			Log.LogWarning("Failed to write NuGet cache metadata for '{0}': {1}", NuPkgPath, ex.Message);
		}

		return true;
	}
}
