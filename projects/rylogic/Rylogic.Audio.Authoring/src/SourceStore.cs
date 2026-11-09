using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Security.Cryptography;
using System.Threading;
using System.Threading.Tasks;

namespace Rylogic.Audio.Authoring;

/// <summary>
/// The originals: one '&lt;source-id&gt;.&lt;type&gt;' audio file per source with a '&lt;source-id&gt;.json' provenance sidecar. A download in one of the
/// <see cref="StoredTypes"/> is kept byte-for-byte, so compressed recordings are not enlarged; any other type is converted to lossless FLAC. Originals are
/// written once on import and never modified, so every render can start from the untouched recording. Originals that can be downloaded again (see
/// <see cref="CanFetch"/>) are kept in the fetched folder, which need not be version controlled; all others are kept in the originals folder. A source is
/// looked up in both.
/// </summary>
public sealed class SourceStore
{
	private readonly Ffmpeg m_ffmpeg;
	private readonly Dictionary<string, (long length, DateTime modified, string sha)> m_hashes = [];

	/// <summary>Keep originals in 'folder', and those that can be fetched again in 'fetched_folder' (or in 'folder' when it is null).</summary>
	public SourceStore(string folder, string? fetched_folder, Ffmpeg ffmpeg)
	{
		Folder = folder;
		FetchedFolder = fetched_folder;
		m_ffmpeg = ffmpeg;
	}

	/// <summary>The folder of originals that exist nowhere else.</summary>
	public string Folder { get; }

	/// <summary>The folder of originals that can be fetched again, or null when they are kept in <see cref="Folder"/>.</summary>
	public string? FetchedFolder { get; }

	/// <summary>True when an original of 'kind' can be downloaded again, so it need not be kept with the project.</summary>
	public static bool CanFetch(EOriginKind kind)
	{
		switch (kind)
		{
			case EOriginKind.Freesound:
			{
				return true;
			}
			case EOriginKind.Local:
			case EOriginKind.HandMade:
			case EOriginKind.Edited:
			{
				return false;
			}
			default:
			{
				throw new ArgumentOutOfRangeException(nameof(kind), kind, "Unknown origin kind.");
			}
		}
	}

	/// <summary>The file types kept as received. FLAC is already lossless, and converting a lossy type would only enlarge it.</summary>
	public static IReadOnlyList<string> StoredTypes { get; } = [".flac", ".mp3", ".ogg", ".opus"];

	/// <summary>The stored audio file of 'source_id'. When it is missing, this is where a FLAC would be.</summary>
	public string OriginalPath(string source_id)
	{
		// The sidecar does not record the type, so the folder is searched for each type a source can be stored as
		var folder = FolderOf(source_id);
		foreach (var type in StoredTypes)
		{
			var path = Path.Combine(folder, $"{source_id}{type}");
			if (File.Exists(path))
				return path;
		}
		return Path.Combine(folder, $"{source_id}.flac");
	}

	/// <summary>The provenance sidecar of 'source_id'.</summary>
	public string SidecarPath(string source_id)
	{
		return Path.Combine(FolderOf(source_id), $"{source_id}.json");
	}

	/// <summary>The folder holding 'source_id': the one with its sidecar, or the originals folder when it is in neither.</summary>
	private string FolderOf(string source_id)
	{
		// The sidecar is written last on import, so it marks the folder holding a complete original
		if (FetchedFolder != null && !File.Exists(Path.Combine(Folder, $"{source_id}.json")) && File.Exists(Path.Combine(FetchedFolder, $"{source_id}.json")))
			return FetchedFolder;

		return Folder;
	}

	/// <summary>True when 'source_id' has been imported.</summary>
	public bool Contains(string source_id)
	{
		return File.Exists(OriginalPath(source_id)) && File.Exists(SidecarPath(source_id));
	}

	/// <summary>Read the sidecar of 'source_id'.</summary>
	public SourceSidecar Sidecar(string source_id)
	{
		return AuthoringJson.Read<SourceSidecar>(SidecarPath(source_id));
	}

	/// <summary>The source ID of a file: the first 16 hex digits of the SHA-256 of its bytes.</summary>
	public static string SourceIdOf(string sha256)
	{
		return sha256[..16];
	}

	/// <summary>
	/// Import 'file_path' as an original. 'provenance' supplies the kind, origin and licence fields; its hashes and ID are filled in here. Importing the same
	/// bytes again returns the existing source unchanged.
	/// </summary>
	public async Task<SourceSidecar> ImportAsync(string file_path, SourceSidecar provenance, CancellationToken cancel)
	{
		// The source is identified by the bytes as received, so the same download always maps to the same source
		var download_sha = await Sha256Async(file_path, cancel);
		var id = SourceIdOf(download_sha);
		if (Contains(id))
		{
			var existing = Sidecar(id);
			if (existing.DownloadSha256 != download_sha)
				throw new InvalidDataException($"Source '{id}' already exists for different bytes; the existing original is left unchanged.");

			return existing;
		}

		// A stored type is kept byte-for-byte; anything else is converted losslessly from its decoded form
		var folder = CanFetch(provenance.Kind) && FetchedFolder != null ? FetchedFolder : Folder;
		Directory.CreateDirectory(folder);
		var type = Path.GetExtension(file_path).ToLowerInvariant();
		string original_path;
		if (StoredTypes.Contains(type))
		{
			original_path = Path.Combine(folder, $"{id}{type}");
			var bytes = await File.ReadAllBytesAsync(file_path, cancel);
			AuthoringJson.WriteAtomic(original_path, bytes);
		}
		else
		{
			original_path = Path.Combine(folder, $"{id}.flac");
			await m_ffmpeg.ConvertToFlacAsync(file_path, original_path, cancel);
		}

		// The sidecar is written last, so a source only counts as imported once its audio file is complete
		var sidecar = new SourceSidecar
		{
			Id = id,
			Kind = provenance.Kind,
			Origin = provenance.Origin,
			Title = provenance.Title,
			Author = provenance.Author,
			Licence = provenance.Licence,
			Url = provenance.Url,
			EditOf = provenance.EditOf,
			DownloadName = provenance.DownloadName ?? Path.GetFileName(file_path),
			DownloadSha256 = download_sha,
			OriginalSha256 = await Sha256Async(original_path, cancel),
		};
		AuthoringJson.Write(Path.Combine(folder, $"{id}.json"), sidecar);
		return sidecar;
	}

	/// <summary>Remove the original 'source_id' and its sidecar. The caller makes sure that nothing uses it.</summary>
	public void Delete(string source_id)
	{
		// Find both files before deleting either, because the sidecar is what marks the folder that holds them.
		// The sidecar goes first, so a half-removed source no longer counts as imported.
		var original_path = OriginalPath(source_id);
		File.Delete(SidecarPath(source_id));
		File.Delete(original_path);
	}

	/// <summary>Throw if the stored audio file of 'source_id' no longer matches the hash recorded when it was imported.</summary>
	public async Task VerifyAsync(string source_id, CancellationToken cancel)
	{
		// A changed original would silently change every render, so it is an error
		if (!Contains(source_id))
			throw new FileNotFoundException($"Original '{source_id}' is missing.", OriginalPath(source_id));

		var expected = Sidecar(source_id).OriginalSha256;
		var actual = await OriginalSha256Async(source_id, cancel);
		if (!string.Equals(expected, actual, StringComparison.OrdinalIgnoreCase))
			throw new InvalidDataException($"Original '{source_id}' has changed since it was imported.");
	}

	/// <summary>The SHA-256 of the stored audio file of 'source_id', remembered until the file's size or time changes.</summary>
	public async Task<string> OriginalSha256Async(string source_id, CancellationToken cancel)
	{
		// Hashing a long recording is slow, so unchanged files are not hashed again
		var path = OriginalPath(source_id);
		var info = new FileInfo(path);
		if (!info.Exists)
			throw new FileNotFoundException($"Original '{source_id}' is missing.", path);

		lock (m_hashes)
		{
			if (m_hashes.TryGetValue(path, out var memo) && memo.length == info.Length && memo.modified == info.LastWriteTimeUtc)
				return memo.sha;
		}
		var sha = await Sha256Async(path, cancel);
		lock (m_hashes)
			m_hashes[path] = (info.Length, info.LastWriteTimeUtc, sha);

		return sha;
	}

	/// <summary>The lowercase hex SHA-256 of a file.</summary>
	public static async Task<string> Sha256Async(string path, CancellationToken cancel)
	{
		await using var stream = File.OpenRead(path);
		return Convert.ToHexStringLower(await SHA256.HashDataAsync(stream, cancel));
	}
}
