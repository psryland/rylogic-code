#! "net10.0"
#load "UserVars.csx"
#load "Tools.csx"
#nullable enable

// Use: dotnet-script CodeSync.csx <dir1> [dir2] [...] [--tab-size N]

// Notes:
//  - This script finds blocks of code within comment sections like this
//    and ensures content within the blocks is identical across all files.
//     file1.h/.cpp
//     ...
//      PR_CODE_SYNC_BEGIN(unique_name, [source_of_truth])
//      PR_CODE_SYNC_END()
//     ...
//    This allows 'dependency free' files without the problems of code duplication.
//  - Blocks with the 'source_of_truth' tag are called 'TruthBlocks' and are considered to be the reference implementation.
//  - Blocks without the 'source_of_truth' tag are called 'RefBlocks' and get replaced by the reference implementation.
//    (if different, to prevent unnecessary file touching).
//  - PR_CODE_SYNC_BEGIN/PR_CODE_SYNC_END must be the only comment on the line.
//  - Blocks can be nested. Source of truth blocks containing nested source of truth blocks
//    have the PR_CODE_SYNC_BEGIN/END comments removed.
//  - Source of truth blocks can only contain nested source of truth blocks, not reference blocks
//  - Code within source of truth blocks is not modified.

using System;
using System.Text.RegularExpressions;
using System.Threading;
using Console = System.Console;

public class CodeSync
{
	// Regex for matching PR_CODE_SYNC_BEGIN lines.
	// Captures: 'name' = the unique block name, 'sot' = non-empty if source_of_truth
	private static readonly Regex BeginPattern = new(@"^(?<prefix>.*)PR_CODE_SYNC_BEGIN\(\s*(?<name>\w+)\s*(?:,\s*(?<sot>source_of_truth)\s*)?\)(?<suffix>.*)$");

	// Regex for matching PR_CODE_SYNC_END lines.
	private static readonly Regex EndPattern = new(@"PR_CODE_SYNC_END\(\)");

	// Patterns for matching files that should be searched for sync blocks
	private static readonly string[] FileExtensions = [".h", ".hpp", ".cpp", ".c", ".inl"];

	// Source of truth blocks. Mapping from unique name to the reference
	// implementation code (with PR_CODE_SYNC_BEGIN/END tags stripped).
	private Dictionary<string, TruthBlock> m_truths = [];

	// Errors collected during processing (reported at the end)
	private List<string> m_errors = [];

	// Tab size in spaces (1 tab == this many spaces)
	private int m_tab_size;

	// Whether to print diagnostic output
	private bool m_verbose;

	// A source of truth block. Content lines have indentation stored as a column
	// count (relative to the BEGIN line) plus the non-whitespace remainder.
	private record TruthLine(int IndentColumns, string Content);
	private record TruthBlock(string Name, List<TruthLine> Lines, string FilePath, int LineNumber);

	public CodeSync(int tab_size = 4, bool verbose = false)
	{
		m_tab_size = tab_size;
		m_verbose = verbose;
	}

	// Measure the column width of leading whitespace, where '\t' == m_tab_size spaces
	private int MeasureIndent(string line)
	{
		int cols = 0;
		for (int i = 0; i != line.Length; ++i)
		{
			if (line[i] == '\t') cols += m_tab_size;
			else if (line[i] == ' ') cols += 1;
			else break;
		}
		return cols;
	}

	// Return the leading whitespace characters of a line
	private static string GetIndentStr(string line)
	{
		int i = 0;
		while (i < line.Length && (line[i] == ' ' || line[i] == '\t'))
			++i;
		return line[..i];
	}

	// Detect whether a line uses tabs or spaces for indentation
	private static bool UsesTabs(string indent)
	{
		return indent.Length == 0 || indent.Contains('\t');
	}

	// Build a whitespace string for 'columns' using tabs or spaces
	private string MakeIndent(int columns, bool use_tabs)
	{
		if (use_tabs)
		{
			int tabs = columns / m_tab_size;
			int spaces = columns % m_tab_size;
			return new string('\t', tabs) + new string(' ', spaces);
		}
		return new string(' ', columns);
	}

	// Strip the non-whitespace content from a line, returning (indent_columns, content)
	private TruthLine DecomposeLineIndent(string line, int base_columns)
	{
		if (string.IsNullOrWhiteSpace(line))
			return new TruthLine(0, "");

		var indent_cols = MeasureIndent(line);
		var relative_cols = Math.Max(0, indent_cols - base_columns);
		var content = line[GetIndentStr(line).Length..];
		return new TruthLine(relative_cols, content);
	}

	// Reconstruct a line from a TruthLine with the given base indent
	private string ReconstructLine(TruthLine tl, int base_columns, bool use_tabs)
	{
		if (tl.Content.Length == 0)
			return "";

		return MakeIndent(base_columns + tl.IndentColumns, use_tabs) + tl.Content;
	}

	// Synchronise code blocks across multiple directory trees
	public void Run(IEnumerable<string> directories)
	{
		var dirs = directories.ToList();
		foreach (var dir in dirs)
		{
			if (!Directory.Exists(dir))
				throw new Exception($"Directory '{dir}' does not exist.");
		}

		if (m_verbose)
			Console.WriteLine($"CodeSync: Scanning {string.Join(", ", dirs.Select(d => $"'{d}'"))}...");

		// Find all the source of truth blocks in all directories
		foreach (var dir in dirs)
			FindTruthBlocks(dir);

		if (m_verbose)
			Console.WriteLine($"CodeSync: Found {m_truths.Count} truth block(s): {string.Join(", ", m_truths.Keys)}");

		// Now replace any ref blocks
		int replaced = 0;
		foreach (var dir in dirs)
			replaced += ReplaceRefBlocks(dir);

		if (replaced > 0 || m_verbose)
			Console.WriteLine($"CodeSync: Updated {replaced} file(s).");

		// Report any errors (ref blocks newer than their source of truth)
		if (m_errors.Count > 0)
		{
			foreach (var error in m_errors)
				Console.Error.WriteLine($"CodeSync error: {error}");

			throw new Exception($"CodeSync: {m_errors.Count} ref block(s) are newer than their source of truth. Update the source of truth first.");
		}
	}

	// Enumerate matching files in the directory tree
	private IEnumerable<string> EnumerateFiles(string root_directory)
	{
		return Directory.EnumerateFiles(root_directory, "*", SearchOption.AllDirectories)
			.Where(f => FileExtensions.Contains(Path.GetExtension(f).ToLowerInvariant()));
	}

	// Find all source of truth blocks in the directory tree
	private void FindTruthBlocks(string root_directory)
	{
		foreach (var filepath in EnumerateFiles(root_directory))
		{
			var lines = File.ReadAllLines(filepath);
			FindTruthBlocksInLines(lines, filepath, 0, lines.Length, is_truth_scope: false);
		}
	}

	// Scan lines [start, end) for truth blocks
	private void FindTruthBlocksInLines(string[] lines, string filepath, int start, int end, bool is_truth_scope)
	{
		for (int i = start; i < end; ++i)
		{
			var m = BeginPattern.Match(lines[i]);
			if (!m.Success)
			{
				// Check for orphaned END tags
				if (EndPattern.IsMatch(lines[i]))
					throw new Exception($"{filepath}({i + 1}): Unexpected PR_CODE_SYNC_END without matching BEGIN.");
				continue;
			}

			var name = m.Groups["name"].Value;
			var is_sot = m.Groups["sot"].Success;

			// Find the matching END tag, accounting for nesting
			int depth = 1;
			int content_start = i + 1;
			int content_end = -1;
			for (int j = content_start; j < end; ++j)
			{
				if (BeginPattern.IsMatch(lines[j]))
					++depth;
				if (EndPattern.IsMatch(lines[j]))
				{
					--depth;
					if (depth == 0)
					{
						content_end = j;
						break;
					}
				}
			}
			if (content_end < 0)
				throw new Exception($"{filepath}({i + 1}): PR_CODE_SYNC_BEGIN('{name}') has no matching PR_CODE_SYNC_END.");

			if (is_sot)
			{
				// Extract the truth content, stripping nested PR_CODE_SYNC_BEGIN/END lines
				// and storing indentation as column counts relative to the BEGIN line
				var content = ExtractTruthContent(lines, i, content_start, content_end);

				// Check for duplicate truth blocks
				if (m_truths.ContainsKey(name))
				{
					var existing = m_truths[name];
					throw new Exception(
						$"{filepath}({i + 1}): Duplicate source_of_truth block '{name}'. " +
						$"First defined at {existing.FilePath}({existing.LineNumber}).");
				}

				// Validate: truth blocks cannot contain ref blocks
				ValidateNoRefBlocks(lines, content_start, content_end, name, filepath);

				m_truths[name] = new TruthBlock(name, content, filepath, i + 1);

				// Recursively find nested truth blocks
				FindTruthBlocksInLines(lines, filepath, content_start, content_end, is_truth_scope: true);
			}
			else
			{
				// Ref block - if we're inside a truth scope, that's an error
				if (is_truth_scope)
					throw new Exception($"{filepath}({i + 1}): Ref block '{name}' found inside a source_of_truth block. Only nested source_of_truth blocks are allowed.");
			}

			// Skip past this block
			i = content_end;
		}
	}

	// Extract the content of a truth block, stripping nested PR_CODE_SYNC_BEGIN/END lines
	// and storing each line as (relative_indent_columns, content).
	private List<TruthLine> ExtractTruthContent(string[] lines, int begin_line, int content_start, int content_end)
	{
		var base_columns = MeasureIndent(lines[begin_line]);
		var result = new List<TruthLine>();
		for (int i = content_start; i != content_end; ++i)
		{
			// Skip lines that are PR_CODE_SYNC_BEGIN or PR_CODE_SYNC_END
			if (BeginPattern.IsMatch(lines[i]) || EndPattern.IsMatch(lines[i]))
				continue;

			result.Add(DecomposeLineIndent(lines[i], base_columns));
		}
		return result;
	}

	// Validate that a truth block does not contain any ref blocks
	private void ValidateNoRefBlocks(string[] lines, int start, int end, string truth_name, string filepath)
	{
		for (int i = start; i < end; ++i)
		{
			var m = BeginPattern.Match(lines[i]);
			if (m.Success && !m.Groups["sot"].Success)
				throw new Exception($"{filepath}({i + 1}): Ref block '{m.Groups["name"].Value}' found inside source_of_truth block '{truth_name}'.");
		}
	}

	// Replace all ref blocks with their corresponding truth content
	private int ReplaceRefBlocks(string root_directory)
	{
		int files_updated = 0;

		foreach (var filepath in EnumerateFiles(root_directory))
		{
			var lines = File.ReadAllLines(filepath);
			bool modified = false;

			// Process from the end so that line indices remain valid after insertions/deletions
			var ref_blocks = FindRefBlocks(lines, filepath);
			ref_blocks.Reverse(); // Process from bottom to top

			foreach (var (name, begin_line, content_start, content_end) in ref_blocks)
			{
				if (!m_truths.TryGetValue(name, out var truth))
					throw new Exception($"{filepath}({begin_line + 1}): Ref block '{name}' refers to unknown truth block.");

				// Determine the ref block's indent style
				var ref_indent_str = GetIndentStr(lines[begin_line]);
				var ref_base_columns = MeasureIndent(lines[begin_line]);
				var ref_uses_tabs = UsesTabs(ref_indent_str);

				// Reconstruct truth lines with the ref block's indentation style
				var indented_truth = truth.Lines.Select(tl => ReconstructLine(tl, ref_base_columns, ref_uses_tabs)).ToList();

				// Get the current ref block content
				var current = new List<string>();
				for (int i = content_start; i != content_end; ++i)
					current.Add(lines[i]);

				// Compare by content, ignoring whitespace differences
				bool same = current.Count == indented_truth.Count;
				if (same)
				{
					for (int i = 0; i != current.Count; ++i)
					{
						if (current[i].TrimStart() != indented_truth[i].TrimStart())
						{
							same = false;
							break;
						}
					}
				}

				if (!same)
				{
					// Safety check: if the ref file is newer than the truth file, the ref block
					// may have been modified directly instead of the source of truth. Report an
					// error rather than silently overwriting the ref block.
					var ref_modified = File.GetLastWriteTimeUtc(filepath);
					var truth_modified = File.GetLastWriteTimeUtc(truth.FilePath);
					if (ref_modified > truth_modified)
					{
						// Only flag as an error if the ref block is non-empty (i.e. someone edited it)
						if (current.Count > 0 && current.Any(l => l.Trim().Length > 0))
						{
							m_errors.Add($"{filepath}({begin_line + 1}): Ref block '{name}' code is newer than source of truth in {truth.FilePath}({truth.LineNumber}). Check source of truth implementation is up to date.");
							continue;
						}
					}

					// Build the new lines array
					var new_lines = new List<string>(lines.Length);
					for (int i = 0; i < content_start; ++i)
						new_lines.Add(lines[i]);
					new_lines.AddRange(indented_truth);
					for (int i = content_end; i < lines.Length; ++i)
						new_lines.Add(lines[i]);

					lines = new_lines.ToArray();
					modified = true;
					if (m_verbose)
						Console.WriteLine($"  Updated '{name}' in {filepath}");
				}
			}

			if (modified)
			{
				File.WriteAllLines(filepath, lines);
				++files_updated;
			}
		}

		return files_updated;
	}

	// Find all ref blocks in the file. Returns (name, begin_line, content_start, content_end).
	private List<(string name, int begin_line, int content_start, int content_end)> FindRefBlocks(string[] lines, string filepath)
	{
		var blocks = new List<(string, int, int, int)>();

		for (int i = 0; i < lines.Length; ++i)
		{
			var m = BeginPattern.Match(lines[i]);
			if (!m.Success)
				continue;

			var name = m.Groups["name"].Value;
			var is_sot = m.Groups["sot"].Success;

			// Find the matching END tag, accounting for nesting
			int depth = 1;
			int content_start = i + 1;
			int content_end = -1;
			for (int j = content_start; j < lines.Length; ++j)
			{
				if (BeginPattern.IsMatch(lines[j]))
					++depth;
				if (EndPattern.IsMatch(lines[j]))
				{
					--depth;
					if (depth == 0)
					{
						content_end = j;
						break;
					}
				}
			}
			if (content_end < 0)
				throw new Exception($"{filepath}({i + 1}): PR_CODE_SYNC_BEGIN('{name}') has no matching PR_CODE_SYNC_END.");

			if (!is_sot)
			{
				// Validate: ref blocks cannot contain nested blocks
				for (int j = content_start; j < content_end; ++j)
				{
					if (BeginPattern.IsMatch(lines[j]))
						throw new Exception($"{filepath}({j + 1}): Nested block found inside ref block '{name}'. Ref blocks cannot contain nested blocks.");
				}

				blocks.Add((name, i, content_start, content_end));
			}

			// Skip past this block
			i = content_end;
		}

		return blocks;
	}
}

try
{
	var directories = new List<string>();
	int tab_size = 4;
	string? stamp_path = null;
	int stamp_max_age_sec = 30;
	bool verbose = false;

	// Parse command line arguments
	for (int i = 0; i < Args.Count; ++i)
	{
		if (Args[i] == "--tab-size" && i + 1 < Args.Count)
			tab_size = int.Parse(Args[++i]);
		else if (Args[i] == "--stamp" && i + 1 < Args.Count)
			stamp_path = Args[++i];
		else if (Args[i] == "--stamp-max-age" && i + 1 < Args.Count)
			stamp_max_age_sec = int.Parse(Args[++i]);
		else if (Args[i] == "--verbose" || Args[i] == "-v")
			verbose = true;
		else
			directories.Add(Args[i]);
	}

	// Testing
	#if false
	{
		// Copy directory tree for testing
		string source_dir = @"E:\dump\CodeSync\Base";
		string dest_dir = @"E:\dump\CodeSync\Test";

		if (Directory.Exists(dest_dir))
			Directory.Delete(dest_dir, recursive: true);

		CopyDirectory(source_dir, dest_dir);

		void CopyDirectory(string src, string dst)
		{
			Directory.CreateDirectory(dst);
			foreach (var file in Directory.GetFiles(src))
				File.Copy(file, Path.Combine(dst, Path.GetFileName(file)), overwrite: true);
			foreach (var dir in Directory.GetDirectories(src))
				CopyDirectory(dir, Path.Combine(dst, Path.GetFileName(dir)));
		}

		directories = [dest_dir];
	}
	#endif

	if (directories.Count == 0)
	{
		Console.Error.WriteLine("Usage: dotnet-script CodeSync.csx <dir1> [dir2] [...] [--tab-size N] [--stamp <path>] [--stamp-max-age <seconds>] [--verbose|-v]");
		Environment.Exit(1);
		return;
	}

	// Normalise paths (handles trailing dots, slashes, etc.)
	directories = directories.Select(d => Path.GetFullPath(d)).ToList();

	// Use a named mutex to serialize parallel invocations (e.g. from MSBuild parallel builds)
	using var mutex = new Mutex(false, @"Global\RylogicCodeSync");
	mutex.WaitOne();
	try
	{
		// If a stamp file is specified, check if a recent stamp exists (another build already ran CodeSync)
		if (stamp_path != null && File.Exists(stamp_path))
		{
			var age = DateTime.Now - File.GetLastWriteTime(stamp_path);
			if (age.TotalSeconds < stamp_max_age_sec)
			{
				if (verbose)
					Console.WriteLine("CodeSync: Skipped (recent stamp exists).");
				return;
			}
		}

		new CodeSync(tab_size, verbose).Run(directories);

		// Write the stamp file after a successful run
		if (stamp_path != null)
		{
			var stamp_dir = Path.GetDirectoryName(stamp_path);
			if (stamp_dir != null && !Directory.Exists(stamp_dir))
				Directory.CreateDirectory(stamp_dir);
			File.WriteAllText(stamp_path, DateTime.Now.ToString("o"));
		}
	}
	finally
	{
		mutex.ReleaseMutex();
	}
}
catch (Exception ex)
{
	Console.Error.WriteLine($"CodeSync error: {ex.Message}");
	Environment.Exit(1);
}
