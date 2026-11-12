using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Threading;
using System.Threading.Tasks;

namespace Rylogic.Audio.Authoring;

/// <summary>Runs ffmpeg for decoding and lossless conversion. ffmpeg is an authoring and cooking dependency only.</summary>
public sealed class Ffmpeg
{
	/// <summary>Use 'path' as ffmpeg, or find ffmpeg on PATH when 'path' is null.</summary>
	public Ffmpeg(string? path = null)
	{
		// An explicit path is authoritative so a build cannot silently pick another ffmpeg
		if (!string.IsNullOrWhiteSpace(path))
		{
			ExePath = Path.GetFullPath(path);
			if (!File.Exists(ExePath))
				throw new FileNotFoundException("The configured ffmpeg executable does not exist.", ExePath);
		}
		else
		{
			ExePath = FindOnPath(OperatingSystem.IsWindows() ? "ffmpeg.exe" : "ffmpeg")
				?? throw new FileNotFoundException("ffmpeg was not found. Put it on PATH or configure its location.");
		}

		// Package managers often install a link; the link's target is what runs
		ExePath = new FileInfo(ExePath).ResolveLinkTarget(returnFinalTarget: true)?.FullName ?? ExePath;
		m_version = new Lazy<string>(ReadVersion);
	}

	/// <summary>The ffmpeg executable.</summary>
	public string ExePath { get; }

	/// <summary>The first line of 'ffmpeg -version', which identifies the build. Part of every render cache key.</summary>
	public string Version => m_version.Value;
	private readonly Lazy<string> m_version;

	/// <summary>Decode the first audio stream of 'path' to 48 kHz float. Sources with more than two channels are mixed to stereo.</summary>
	public async Task<AudioBuffer> DecodeAsync(string path, CancellationToken cancel)
	{
		// Bit-exact mode and no metadata keep the output identical for identical input
		string[] arguments =
		[
			"-nostdin", "-hide_banner", "-loglevel", "error",
			"-i", path,
			"-map", "0:a:0", "-map_metadata", "-1",
			"-af", "aformat=channel_layouts=mono|stereo",
			"-ar", AudioBuffer.SampleRate.ToString(System.Globalization.CultureInfo.InvariantCulture),
			"-c:a", "pcm_f32le", "-fflags", "+bitexact", "-flags:a", "+bitexact",
			"-f", "wav", "-",
		];
		AudioBuffer? result = null;
		await RunAsync(arguments, stdout => result = WavFile.Decode(stdout), $"Decoding '{path}'", cancel);
		return result!;
	}

	/// <summary>Convert 'input_path' to lossless FLAC at 'output_path', keeping its sample rate and channels.</summary>
	public async Task ConvertToFlacAsync(string input_path, string output_path, CancellationToken cancel)
	{
		// Write to a temporary file so an interrupted conversion is never mistaken for an original
		var temporary_path = $"{output_path}.{Guid.NewGuid():N}.tmp.flac";
		try
		{
			string[] arguments =
			[
				"-nostdin", "-hide_banner", "-loglevel", "error", "-y",
				"-i", input_path,
				"-map", "0:a:0", "-map_metadata", "-1",
				"-c:a", "flac", "-compression_level", "12", "-fflags", "+bitexact", "-flags:a", "+bitexact",
				temporary_path,
			];
			await RunAsync(arguments, null, $"Converting '{input_path}' to FLAC", cancel);
			File.Move(temporary_path, output_path, overwrite: true);
		}
		finally
		{
			File.Delete(temporary_path);
		}
	}

	/// <summary>
	/// The descriptive tags of 'path', such as 'title' and 'artist', keyed by lowercase tag name. Tags that the file's format stores per stream, as Ogg
	/// does, are included. A file without tags gives an empty dictionary.
	/// </summary>
	public async Task<IReadOnlyDictionary<string, string>> ReadTagsAsync(string path, CancellationToken cancel)
	{
		// ffmpeg writes the tags in its plain 'ffmetadata' text form: a header line, then one 'key=value' line per tag
		var tags = new Dictionary<string, string>(StringComparer.Ordinal);
		foreach (var source in new[] { "0", "0:s:a:0" })
		{
			string[] arguments =
			[
				"-nostdin", "-hide_banner", "-loglevel", "error",
				"-i", path,
				"-map_metadata", source, "-f", "ffmetadata", "-",
			];
			var text = string.Empty;
			await RunAsync(arguments, stdout => text = new StreamReader(stdout).ReadToEnd(), $"Reading the tags of '{path}'", cancel);

			// The file's own tags come first, so a stream tag of the same name does not replace one
			foreach (var (key, value) in ParseMetadata(text))
				tags.TryAdd(key, value);
		}
		return tags;
	}

	/// <summary>The tags in ffmpeg's 'ffmetadata' text form, ignoring any chapter or stream sections.</summary>
	private static IEnumerable<(string key, string value)> ParseMetadata(string text)
	{
		// A backslash escapes the next character, including a line break that continues the value on the next line
		var line = new System.Text.StringBuilder();
		var lines = new List<string>();
		for (int i = 0; i != text.Length; ++i)
		{
			// Unescaped line breaks end a line
			var c = text[i];
			if (c == '\\' && i + 1 != text.Length)
			{
				line.Append(text[++i]);
			}
			else if (c == '\n')
			{
				lines.Add(line.ToString().TrimEnd('\r'));
				line.Clear();
			}
			else
			{
				line.Append(c);
			}
		}
		lines.Add(line.ToString());

		// Comments start with ';' or '#', and the first section header ends the file-level tags
		foreach (var entry in lines)
		{
			// Lines without '=' are blank or malformed
			if (entry.StartsWith('['))
				yield break;

			var equals = entry.IndexOf('=');
			if (entry.StartsWith(';') || entry.StartsWith('#') || equals <= 0)
				continue;

			var value = entry[(equals + 1)..].Trim();
			if (value.Length != 0)
				yield return (entry[..equals].Trim().ToLowerInvariant(), value);
		}
	}

	/// <summary>Run ffmpeg with 'arguments'. 'read_stdout', if given, consumes standard output while ffmpeg runs.</summary>
	private async Task RunAsync(IEnumerable<string> arguments, Action<Stream>? read_stdout, string what, CancellationToken cancel)
	{
		// Arguments are passed as a list so paths with spaces need no quoting
		var start_info = new ProcessStartInfo
		{
			FileName = ExePath,
			UseShellExecute = false,
			CreateNoWindow = true,
			RedirectStandardOutput = true,
			RedirectStandardError = true,
		};
		foreach (var argument in arguments)
			start_info.ArgumentList.Add(argument);

		using var process = Process.Start(start_info) ?? throw new InvalidOperationException("ffmpeg could not be started.");
		try
		{
			// Read both pipes concurrently so neither can fill and stall ffmpeg
			var stderr = process.StandardError.ReadToEndAsync(cancel);
			var stdout = Task.Run(() =>
			{
				// Without a reader, standard output is drained and discarded
				if (read_stdout != null)
					read_stdout(process.StandardOutput.BaseStream);
				else
					process.StandardOutput.BaseStream.CopyTo(Stream.Null);
			}, cancel);
			await process.WaitForExitAsync(cancel);
			await stdout;

			// ffmpeg explains failures on standard error, so its message is kept
			if (process.ExitCode != 0)
				throw new InvalidOperationException($"{what} failed (ffmpeg exit code {process.ExitCode}): {(await stderr).Trim()}");
		}
		catch (OperationCanceledException)
		{
			// A cancelled job must not leave ffmpeg running
			if (!process.HasExited)
				process.Kill(entireProcessTree: true);

			throw;
		}
	}

	/// <summary>Read the version line.</summary>
	private string ReadVersion()
	{
		// Only the first line names the build; the rest lists configuration that the build already implies
		var start_info = new ProcessStartInfo { FileName = ExePath, UseShellExecute = false, CreateNoWindow = true, RedirectStandardOutput = true };
		start_info.ArgumentList.Add("-version");
		using var process = Process.Start(start_info) ?? throw new InvalidOperationException("ffmpeg could not be started.");
		var line = process.StandardOutput.ReadLine() ?? string.Empty;
		process.StandardOutput.ReadToEnd();
		process.WaitForExit();
		return line.Trim();
	}

	/// <summary>Find 'file_name' in the folders on PATH.</summary>
	private static string? FindOnPath(string file_name)
	{
		// Search in PATH order, as the shell would
		foreach (var folder in (Environment.GetEnvironmentVariable("PATH") ?? string.Empty).Split(Path.PathSeparator, StringSplitOptions.RemoveEmptyEntries))
		{
			var candidate = Path.Combine(folder.Trim('"'), file_name);
			if (File.Exists(candidate))
				return candidate;
		}
		return null;
	}
}
