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
		public CommandData? Apply { get; set; }
		public CommandData? Send { get; set; }
		public AssistantData? Assistant { get; set; }
		public CommandData? Editor { get; set; }
	}

	/// <summary>The JSON shape of the AI assistant settings.</summary>
	private sealed class AssistantData
	{
		public string? Model { get; set; }
		public string? Command { get; set; }
	}

	/// <summary>The model the AI assistant uses when the configuration names none. A small, cheap model is enough for searching and ranking.</summary>
	public const string DefaultAssistantModel = "claude-haiku-5.5";

	/// <summary>The JSON shape of a project command.</summary>
	private sealed class CommandData
	{
		public string Command { get; set; } = string.Empty;
		public List<string>? Arguments { get; set; }
		public string? Directory { get; set; }
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

	/// <summary>The command that makes the project use the current review results, such as an asset cook, or null if the project has none.</summary>
	public ProjectCommand? Apply { get; }

	/// <summary>
	/// The command that makes running instances of the project play the current preparation of one sound, or null if the project has none.
	/// Each argument has '{id}' replaced by the sound ID (see <see cref="ProjectCommand.ForSound"/>).
	/// </summary>
	public ProjectCommand? Send { get; }

	/// <summary>The model the AI assistant uses to find candidate sounds.</summary>
	public string AssistantModel { get; }

	/// <summary>The Copilot CLI executable the AI assistant runs, or null to find 'copilot' on PATH.</summary>
	public string? AssistantCommand { get; }

	/// <summary>
	/// The audio editor that opens a working copy of a recording, or null if there is none. Each argument has '{file}' replaced by the working copy's path
	/// (see <see cref="ProjectCommand.ForFile"/>). Without an 'editor' setting, Audacity is used if it is installed in a default location (see <see cref="DefaultEditorPaths"/>).
	/// </summary>
	public ProjectCommand? Editor { get; }

	/// <summary>Where the Audacity installers put Audacity, newest version first.</summary>
	public static IReadOnlyList<string> DefaultEditorPaths
	{
		get
		{
			// Audacity 4 installs beside Audacity 3 under a new folder and program name
			var program_files = Environment.GetFolderPath(Environment.SpecialFolder.ProgramFiles);
			return
			[
				Path.Combine(program_files, "Audacity 4", "bin", "Audacity4.exe"),
				Path.Combine(program_files, "Audacity", "Audacity.exe"),
			];
		}
	}

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

		// Project commands are run as given; only their working directory is a project path
		Apply = ReadCommand(data.Apply, "apply");
		Send = ReadCommand(data.Send, "send");
		AssistantModel = string.IsNullOrWhiteSpace(data.Assistant?.Model) ? DefaultAssistantModel : data.Assistant.Model;
		AssistantCommand = Resolve(data.Assistant?.Command, null);

		// An editor opens the working copy named by its arguments, which by default is its only argument
		Editor = ReadCommand(data.Editor, "editor") is ProjectCommand editor
			? (editor.Arguments.Count != 0 ? editor : editor with { Arguments = ["{file}"] })
			: DefaultEditorPaths.FirstOrDefault(File.Exists) is string installed ? new ProjectCommand(installed, ["{file}"], Root) : null;

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

	/// <summary>Read the optional command 'name', whose working directory defaults to the configuration folder.</summary>
	private ProjectCommand? ReadCommand(CommandData? data, string name)
	{
		// An absent command means the project does not support the action
		if (data == null)
			return null;
		if (string.IsNullOrWhiteSpace(data.Command))
			throw new InvalidDataException($"'{ConfigPath}' must set '{name}.command'.");

		return new ProjectCommand(data.Command, data.Arguments ?? [], Resolve(data.Directory, null) ?? Root);
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

/// <summary>
/// A project command that the authoring UI runs on request. 'Command' is a program name or path, 'Arguments' are passed to it unchanged,
/// and 'Directory' is the absolute working directory.
/// </summary>
public sealed record ProjectCommand(string Command, IReadOnlyList<string> Arguments, string Directory)
{
	/// <summary>This command with '{id}' in each argument replaced by 'sound_id'.</summary>
	public ProjectCommand ForSound(string sound_id)
	{
		// Only arguments are substituted, so the program and working directory stay fixed
		return this with { Arguments = Arguments.Select(x => x.Replace("{id}", sound_id, StringComparison.Ordinal)).ToList() };
	}

	/// <summary>This command with '{file}' in each argument replaced by 'path'.</summary>
	public ProjectCommand ForFile(string path)
	{
		// Only arguments are substituted, so the program and working directory stay fixed
		return this with { Arguments = Arguments.Select(x => x.Replace("{file}", path, StringComparison.Ordinal)).ToList() };
	}

	/// <summary>The program's name without its folder or extension, such as 'Audacity', for display.</summary>
	public string ProgramName => Path.GetFileNameWithoutExtension(Command);

	/// <summary>The command line as a user would type it, for display.</summary>
	public string Display => string.Join(' ', Arguments.Prepend(Command).Select(x => x.Contains(' ') ? $"\"{x}\"" : x));
}
