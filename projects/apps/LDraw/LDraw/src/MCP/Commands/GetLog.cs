using System;
using System.Linq;
using System.Threading.Tasks;
using Rylogic.Gui.WPF;
using Rylogic.Utility;

namespace LDraw.MCP;

/// <summary>Get-log command instance implementation</summary>
internal sealed partial class LDrawInstanceHost
{
	/// <summary>Return recent application-log entries, filtered by severity and source</summary>
	private Task<LDrawLogInfo> GetLogAsync(LDrawGetLogParams parameters)
	{
		// Snapshot the log on the UI thread. 'Log.Entries' is the app-global, in-memory log that LDraw writes
		// scene parse/load errors to (with file/line/offset via Model.ReportError), so it is the right surface
		// for diagnosing problems in a loaded scene. Reading it on the UI thread serialises with the writers,
		// matching the thread-affinity contract every other instance command relies on.
		return m_model.InvokeAsync(() =>
		{
			var min_level = ParseLogLevel(parameters.MinLevel, ELogLevel.Warn);
			var since_index = parameters.SinceIndex;
			var max_count = Math.Clamp(parameters.MaxCount <= 0 ? 200 : parameters.MaxCount, 1, 1000);
			var contains = parameters.Contains;
			var file_filter = parameters.File;

			// The newest entry's index is the cursor the caller passes next time to fetch only newer entries.
			// It is computed across all entries, independent of the level/text filters, so incremental polling
			// never skips or repeats an entry when the filters change between calls.
			var entries = Log.Entries;
			var next_index = entries.Count != 0 ? entries.Max(e => e.FPos) : Math.Max(since_index, 0);

			// Apply the cursor first (a cheap direct field), then the severity and text filters. The parsed
			// properties (Level/Message/File) each run the log-line regex, so they are only read for entries
			// that already passed the cursor check.
			var matches = entries
				.Where(e => e.FPos > since_index)
				.Where(e => (int)e.Level >= (int)min_level)
				.Where(e => string.IsNullOrEmpty(contains) || e.Message.Contains(contains, StringComparison.OrdinalIgnoreCase))
				.Where(e => string.IsNullOrEmpty(file_filter) || e.File.Contains(file_filter, StringComparison.OrdinalIgnoreCase))
				.ToList();

			// Keep the newest matches when truncating because the most recent entries best describe the last
			// action, but return them oldest-first so the sequence reads naturally.
			var truncated = matches.Count > max_count;
			var selected = truncated ? matches.Skip(matches.Count - max_count) : matches;

			return new LDrawLogInfo
			{
				Entries = [.. selected.Select(CreateLogEntryInfo)],
				Truncated = truncated,
				NextIndex = next_index,
			};
		}, TimeSpan.FromSeconds(5));
	}

	/// <summary>Create a serialisable DTO from a log entry</summary>
	private static LDrawLogEntry CreateLogEntryInfo(LogControl.LogEntry entry)
	{
		return new LDrawLogEntry
		{
			Index = entry.FPos,
			Level = entry.Level.ToString(),
			Message = entry.Message,
			File = entry.File,
			Line = entry.Line,
			Offset = entry.Offset,
			Elapsed = entry.ElapsedFormatted,
		};
	}

	/// <summary>Parse an ELogLevel name, falling back to 'fallback' for null, empty, or unrecognised values</summary>
	private static ELogLevel ParseLogLevel(string? level, ELogLevel fallback)
	{
		if (string.IsNullOrWhiteSpace(level))
			return fallback;

		return Enum.TryParse<ELogLevel>(level, ignoreCase: true, out var parsed) ? parsed : fallback;
	}
}
