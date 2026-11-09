using System;
using System.IO;
using System.Threading;
using System.Threading.Tasks;

namespace Rylogic.Audio.Authoring;

/// <summary>
/// Brings original recordings into a <see cref="SourceStore"/> from where they came from: Freesound downloads and the project's local folders. An
/// original that is already stored is never fetched again, and a fetched original must have the source ID recorded for it, so a missing original can be
/// fetched again without changing what is rendered. Fetches run one at a time.
/// </summary>
public sealed class OriginalFetcher
{
	private readonly AuthoringConfig m_config;
	private readonly FreesoundClient m_freesound;
	private readonly SemaphoreSlim m_gate = new(1, 1);

	public OriginalFetcher(AuthoringConfig config, SourceStore sources, FreesoundClient freesound)
	{
		m_config = config;
		Sources = sources;
		m_freesound = freesound;
	}

	/// <summary>The store that fetched originals are imported into.</summary>
	public SourceStore Sources { get; }

	/// <summary>The provenance to record for an original of 'origin' described by 'record'.</summary>
	public static SourceSidecar ProvenanceOf(string origin, OriginRecord record)
	{
		return new SourceSidecar { Kind = OriginsFile.KindOf(origin), Origin = origin, Title = record.Title, Author = record.User, Licence = record.Licence, Url = record.Url, EditOf = record.EditOf };
	}

	/// <summary>Fetch the original of 'origin' and import it, returning its sidecar. Importing an original that is already stored returns the stored one.</summary>
	public async Task<SourceSidecar> ImportAsync(string origin, OriginRecord record, CancellationToken cancel)
	{
		// One fetch at a time keeps downloads from competing and from importing the same original twice
		await m_gate.WaitAsync(cancel);
		try
		{
			return await FetchAndImportAsync(origin, record, cancel);
		}
		finally
		{
			m_gate.Release();
		}
	}

	/// <summary>
	/// Make sure the original 'source_id' of 'origin' is stored, fetching it if it is missing. Throws <see cref="InvalidDataException"/> if the fetched
	/// recording is not 'source_id', because rendering a different recording would silently change the prepared sound.
	/// </summary>
	public async Task EnsureAsync(string origin, OriginRecord record, string source_id, CancellationToken cancel)
	{
		// A stored original needs no fetch, so it never waits behind another download
		if (Sources.Contains(source_id))
			return;

		// Check again after waiting, because an earlier fetch may have stored it
		await m_gate.WaitAsync(cancel);
		try
		{
			if (Sources.Contains(source_id))
				return;

			var sidecar = await FetchAndImportAsync(origin, record, cancel);
			if (sidecar.Id != source_id)
				throw new InvalidDataException($"The original of '{origin}' was fetched as source '{sidecar.Id}', but source '{source_id}' was expected. The recording has changed at its origin.");
		}
		finally
		{
			m_gate.Release();
		}
	}

	/// <summary>Fetch and import the original of 'origin'. Caller holds the gate.</summary>
	private async Task<SourceSidecar> FetchAndImportAsync(string origin, OriginRecord record, CancellationToken cancel)
	{
		var provenance = ProvenanceOf(origin, record);
		switch (provenance.Kind)
		{
			case EOriginKind.Freesound:
			{
				// Download beside the destination into a private folder, so another process fetching the same sound cannot collide, then remove the download
				var download_folder = Path.Combine(Sources.FetchedFolder ?? Sources.Folder, $".download-{Guid.NewGuid():N}");
				try
				{
					var path = await m_freesound.DownloadOriginalAsync(OriginsFile.ReferenceOf(origin), download_folder, cancel);
					return await Sources.ImportAsync(path, provenance, cancel);
				}
				finally
				{
					if (Directory.Exists(download_folder))
						Directory.Delete(download_folder, true);
				}
			}
			case EOriginKind.Local:
			{
				// Local files are already on this machine
				var path = m_config.LocalPathOf(origin);
				if (!File.Exists(path))
					throw new FileNotFoundException($"The local file '{path}' is missing.", path);

				return await Sources.ImportAsync(path, provenance, cancel);
			}
			case EOriginKind.HandMade:
			{
				throw new InvalidOperationException($"'{origin}' is hand-made and exists nowhere else; import its file into '{Sources.Folder}' instead.");
			}
			case EOriginKind.Edited:
			{
				throw new InvalidOperationException($"'{origin}' is an edit that exists nowhere else, and its original is missing from '{Sources.Folder}'.");
			}
			default:
			{
				throw new ArgumentOutOfRangeException(nameof(origin), origin, "Unknown origin kind.");
			}
		}
	}
}
