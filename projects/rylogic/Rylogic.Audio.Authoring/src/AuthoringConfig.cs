using System;
using System.IO;

namespace Rylogic.Audio.Authoring;

/// <summary>
/// The project configuration that the authoring tools open. Every path is resolved relative to the configuration file's folder, and environment variables
/// such as '%LOCALAPPDATA%' are expanded. The project owns all of these locations; nothing here assumes a particular game.
/// </summary>
public sealed class AuthoringConfig
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
		public string? Generated { get; set; }
		public FreesoundData? Freesound { get; set; }
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

	/// <summary>Where generated sounds are found, or null if the project has none.</summary>
	public string? GeneratedFolder { get; }

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
		GeneratedFolder = Resolve(data.Generated, null);
		FreesoundCredentialsPath = Resolve(data.Freesound?.Credentials, null);
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
}
