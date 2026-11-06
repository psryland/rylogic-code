using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text.RegularExpressions;

namespace Rylogic.Audio.Authoring;

/// <summary>
/// The project configuration that the authoring tools open. Every path is resolved relative to the configuration file's folder, and environment variables
/// such as '%LOCALAPPDATA%' are expanded. The project owns all of these locations; nothing here assumes a particular game.
/// </summary>
public sealed partial class AuthoringConfig
{
	/// <summary>The JSON shape of the configuration file.</summary>
	private sealed class FileData
	{
		public int Schema { get; set; }
		public string Catalog { get; set; } = string.Empty;
		public string Originals { get; set; } = string.Empty;
		public string Prep { get; set; } = string.Empty;
		public string Origins { get; set; } = string.Empty;
		public string Cache { get; set; } = string.Empty;
		public string? Fetched { get; set; }
		public string? Previews { get; set; }
		public List<LocalData>? Local { get; set; }
		public FreesoundData? Freesound { get; set; }
	}

	/// <summary>The JSON shape of one local source folder.</summary>
	private sealed class LocalData
	{
		public string Name { get; set; } = string.Empty;
		public string Path { get; set; } = string.Empty;
		public string Author { get; set; } = string.Empty;
		public string Licence { get; set; } = string.Empty;
		public string? Url { get; set; }
	}

	/// <summary>The JSON shape of the Freesound settings.</summary>
	private sealed class FreesoundData
	{
		public string? Credentials { get; set; }
	}

	/// <summary>The configuration file.</summary>
	public string ConfigPath { get; }

	/// <summary>The folder holding the configuration file, which relative paths are resolved against.</summary>
	public string Root { get; }

	/// <summary>The sound catalog. See <see cref="Authoring.Catalog"/>.</summary>
	public string CatalogPath { get; }

	/// <summary>The folder of original FLAC files and their sidecars.</summary>
	public string OriginalsFolder { get; }

	/// <summary>The folder of preparation files.</summary>
	public string PrepFolder { get; }

	/// <summary>The origins registry file.</summary>
	public string OriginsPath { get; }

	/// <summary>The disposable render cache folder.</summary>
	public string CacheFolder { get; }

	/// <summary>
	/// Where originals that can be downloaded again are kept, or null to keep them with the other originals. This folder need not be version controlled,
	/// because a missing original is fetched again and verified against its source ID (see <see cref="OriginalFetcher"/>).
	/// </summary>
	public string? FetchedFolder { get; }

	/// <summary>Where browse-quality previews are kept, or null if the project does not browse.</summary>
	public string? PreviewsFolder { get; }

	/// <summary>The folders of local recordings (generated, recorded or collected), in configuration order. Names are unique.</summary>
	public IReadOnlyList<LocalFolder> LocalFolders { get; }

	/// <summary>The per-user Freesound credentials file, or null if Freesound is not used. The file is outside version control.</summary>
	public string? FreesoundCredentialsPath { get; }

	private AuthoringConfig(string config_path, FileData data)
	{
		// Resolve every location once so later code works only with absolute paths
		ConfigPath = Path.GetFullPath(config_path);
		Root = Path.GetDirectoryName(ConfigPath)!;
		CatalogPath = Resolve(data.Catalog, "catalog")!;
		OriginalsFolder = Resolve(data.Originals, "originals")!;
		PrepFolder = Resolve(data.Prep, "prep")!;
		OriginsPath = Resolve(data.Origins, "origins")!;
		CacheFolder = Resolve(data.Cache, "cache")!;
		FetchedFolder = Resolve(data.Fetched, null);
		PreviewsFolder = Resolve(data.Previews, null);
		FreesoundCredentialsPath = Resolve(data.Freesound?.Credentials, null);

		// A local folder's name is the first part of its origin references, so it must be unique and contain no path separator
		var folders = new List<LocalFolder>();
		foreach (var local in data.Local ?? [])
		{
			if (!LocalName().IsMatch(local.Name))
				throw new InvalidDataException($"'{ConfigPath}' has a local folder named '{local.Name}'; names must be letters, digits, '_', '-' or '.'.");
			if (folders.Any(x => x.Name == local.Name))
				throw new InvalidDataException($"'{ConfigPath}' has more than one local folder named '{local.Name}'.");
			if (string.IsNullOrWhiteSpace(local.Author) || string.IsNullOrWhiteSpace(local.Licence))
				throw new InvalidDataException($"'{ConfigPath}' must set 'author' and 'licence' for local folder '{local.Name}'.");

			folders.Add(new LocalFolder(local.Name, Resolve(local.Path, $"local[{local.Name}].path")!, local.Author, local.Licence, local.Url));
		}
		LocalFolders = folders;
	}

	/// <summary>The local folder named 'name'. Throws <see cref="InvalidOperationException"/> if the project does not configure it.</summary>
	public LocalFolder LocalFolderOf(string name)
	{
		return LocalFolders.FirstOrDefault(x => x.Name == name) ?? throw new InvalidOperationException($"'{ConfigPath}' configures no local folder named '{name}'.");
	}

	/// <summary>The file of local origin 'origin' ('local:&lt;folder name&gt;/&lt;relative path&gt;'). Throws if the path would leave its folder.</summary>
	public string LocalPathOf(string origin)
	{
		// The first path part names the folder; the rest is relative to it
		var reference = OriginsFile.ReferenceOf(origin);
		var slash = reference.IndexOf('/');
		if (OriginsFile.KindOf(origin) != EOriginKind.Local || slash < 0)
			throw new InvalidDataException($"'{origin}' is not a local origin.");

		var folder = LocalFolderOf(reference[..slash]).Path.TrimEnd(Path.DirectorySeparatorChar) + Path.DirectorySeparatorChar;
		var path = Path.GetFullPath(Path.Combine(folder, reference[(slash + 1)..]));
		return path.StartsWith(folder, StringComparison.OrdinalIgnoreCase) ? path : throw new InvalidDataException($"'{origin}' leaves its local folder.");
	}

	/// <summary>Read the configuration at 'config_path'.</summary>
	public static AuthoringConfig Load(string config_path)
	{
		var data = AuthoringJson.Read<FileData>(config_path);
		if (data.Schema != 1)
			throw new InvalidDataException($"'{config_path}' must be a schema 1 authoring configuration.");

		return new AuthoringConfig(config_path, data);
	}

	/// <summary>Resolve a configured path. 'required_name' names a required setting; optional settings pass null.</summary>
	private string? Resolve(string? path, string? required_name)
	{
		// A missing required setting is reported by name so the project can fix its file
		if (string.IsNullOrWhiteSpace(path))
			return required_name != null ? throw new InvalidDataException($"'{ConfigPath}' must set '{required_name}'.") : null;

		return Path.GetFullPath(Path.Combine(Root, Environment.ExpandEnvironmentVariables(path)));
	}

	[GeneratedRegex("^[A-Za-z0-9_.-]+$")]
	private static partial Regex LocalName();
}

/// <summary>
/// A folder of local recordings. Every recording found under 'Path' (searched recursively) is an origin 'local:&lt;Name&gt;/&lt;relative path&gt;'
/// credited to 'Author' under 'Licence'. A new recording is assigned to the sound ID its subfolder names (see <see cref="Workspace.GuessSoundId"/>).
/// </summary>
public sealed record LocalFolder(string Name, string Path, string Author, string Licence, string? Url);
