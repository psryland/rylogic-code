#! "net9.0"
#nullable enable

// Write the sha512 hash and metadata files for a nuget package in the global cache.
// Called by the PurgeNuGetCache MSBuild target after Unzip and Copy have placed the package contents.
// Use: dotnet-script InstallNugetCache.csx -- <NuPkgPath> <CacheDir> <NuPkgName>
//
// Note: This script is intentionally self-contained (no #load Tools.csx) because it runs
// during the NuGet cache rebuild — Rylogic.Core may not be available in the cache yet.

using System;
using System.IO;
using System.Security.Cryptography;

var nupkg_path = Args[0];
var cache_dir = Args[1];
var nupkg_name = Args[2];

string hash;
using (var sha = SHA512.Create())
{
	hash = Convert.ToBase64String(sha.ComputeHash(File.ReadAllBytes(nupkg_path)));
}

File.WriteAllText(Path.Combine(cache_dir, $"{nupkg_name}.sha512"), hash);
File.WriteAllText(Path.Combine(cache_dir, ".nupkg.metadata"), $@"{{""version"":2,""contentHash"":""{hash}"",""source"":null}}");
