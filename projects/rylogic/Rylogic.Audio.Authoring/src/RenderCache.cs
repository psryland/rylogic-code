using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Security.Cryptography;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace Rylogic.Audio.Authoring;

/// <summary>
/// Renders recipes from originals into a disposable cache of 48 kHz float WAV files. A render is named by a key that covers everything that can change
/// its output, so a cached file is always correct for its key and the cache can be deleted at any time.
/// </summary>
public sealed class RenderCache
{
	private readonly SourceStore m_sources;
	private readonly Ffmpeg m_ffmpeg;

	public RenderCache(string folder, SourceStore sources, Ffmpeg ffmpeg)
	{
		Folder = folder;
		m_sources = sources;
		m_ffmpeg = ffmpeg;
	}

	/// <summary>The cache folder.</summary>
	public string Folder { get; }

	/// <summary>The path of the render with 'key'.</summary>
	public string PathOf(string key)
	{
		return Path.Combine(Folder, $"{key}.wav");
	}

	/// <summary>
	/// The cache key for rendering 'recipe' from 'source_id': a hash of the renderer version, the original's content, the enabled steps and the ffmpeg
	/// build that decodes the original.
	/// </summary>
	public async Task<string> KeyAsync(string source_id, IReadOnlyList<RecipeStep> recipe, CancellationToken cancel)
	{
		// Bypassed steps do not affect the output, so toggling bypass back restores the earlier key
		var original_sha = await m_sources.OriginalSha256Async(source_id, cancel);
		var steps = AuthoringJson.ToCanonicalString(recipe.Where(x => !x.Bypass).ToList());
		var text = string.Join('\n', RecipeRenderer.PreparationVersion, original_sha, steps, m_ffmpeg.Version);
		return Convert.ToHexStringLower(SHA256.HashData(Encoding.UTF8.GetBytes(text)));
	}

	/// <summary>Return the key and path of the render of 'recipe' from 'source_id', rendering it only if it is not already cached.</summary>
	public async Task<(string key, string path)> GetOrRenderAsync(string source_id, IReadOnlyList<RecipeStep> recipe, CancellationToken cancel)
	{
		// A cached render is complete, because renders are only ever written atomically
		var key = await KeyAsync(source_id, recipe, cancel);
		var path = PathOf(key);
		if (File.Exists(path))
			return (key, path);

		// Always render from the original, never from another render
		await m_sources.VerifyAsync(source_id, cancel);
		var original = await m_ffmpeg.DecodeAsync(m_sources.OriginalPath(source_id), cancel);
		var rendered = await Task.Run(() => RecipeRenderer.Render(original, recipe), cancel);
		cancel.ThrowIfCancellationRequested();
		AuthoringJson.WriteAtomic(path, WavFile.Encode(rendered));
		return (key, path);
	}
}
