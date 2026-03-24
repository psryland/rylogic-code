#! "net10.0"
#r "System.Xml.Linq"
#load "UserVars.csx"
#nullable enable

using System;
using System.IO;
using System.Linq;
using System.Text;
using System.Text.RegularExpressions;
using System.Xml.Linq;

/// <summary>
/// Fixes up .vcxproj and .vcxproj.filters files:
///   1. Replaces relative paths (..\..\) with $(RylogicRoot) absolute paths
///   2. In .vcxproj.filters: merges ClInclude/ClCompile into a single ItemGroup, sorted alphabetically
///
/// Usage:
///   dotnet-script ProjFixUp.csx           # Apply changes
///   dotnet-script ProjFixUp.csx -- -n     # Dry-run (preview changes)
/// </summary>

var root = UserVars.Root;
if (!root.EndsWith('\\'))
	root += '\\';

var dry_run = Args.Contains("--dry-run") || Args.Contains("-n");

var vcxproj_files = Directory.EnumerateFiles(root, "*.vcxproj", SearchOption.AllDirectories).OrderBy(f => f).ToList();
var filters_files = Directory.EnumerateFiles(root, "*.vcxproj.filters", SearchOption.AllDirectories).OrderBy(f => f).ToList();

Console.WriteLine($"Repo root: {root}");
Console.WriteLine($"Found {vcxproj_files.Count} .vcxproj and {filters_files.Count} .vcxproj.filters files");
if (dry_run)
	Console.WriteLine("Dry-run mode — no files will be modified");
Console.WriteLine();

// Match Include attributes with relative paths starting with ".."
var include_regex = new Regex(@"(?<=Include="")(\.\.[\\\/][^""]+)(?="")");
var total_path_fixes = 0;
var total_files_modified = 0;

// --- .vcxproj files: path replacement only ---
foreach (var filepath in vcxproj_files)
{
	var (content, encoding) = ReadFile(filepath);
	var project_dir = Path.GetDirectoryName(filepath)!;
	var fixes = 0;

	var new_content = include_regex.Replace(content, match =>
	{
		var resolved = ResolvePath(project_dir, match.Value);
		if (resolved == null || !resolved.StartsWith(root, StringComparison.OrdinalIgnoreCase))
			return match.Value;

		fixes++;
		return $"$(RylogicRoot){resolved[root.Length..]}";
	});

	if (fixes > 0)
	{
		var rel = Path.GetRelativePath(root, filepath);
		total_path_fixes += fixes;
		total_files_modified++;

		if (dry_run)
			Console.WriteLine($"  [dry-run] {rel} ({fixes} path{Plural(fixes)} fixed)");
		else
		{
			File.WriteAllText(filepath, new_content, encoding);
			Console.WriteLine($"  Updated: {rel} ({fixes} path{Plural(fixes)} fixed)");
		}
	}
}

// --- .vcxproj.filters files: path replacement + sort/merge ClInclude and ClCompile ---
foreach (var filepath in filters_files)
{
	var (content, encoding) = ReadFile(filepath);
	var project_dir = Path.GetDirectoryName(filepath)!;
	var path_fixes = 0;

	// Phase 1: Replace relative paths in the raw text
	var fixed_content = include_regex.Replace(content, match =>
	{
		var resolved = ResolvePath(project_dir, match.Value);
		if (resolved == null || !resolved.StartsWith(root, StringComparison.OrdinalIgnoreCase))
			return match.Value;

		path_fixes++;
		return $"$(RylogicRoot){resolved[root.Length..]}";
	});
	total_path_fixes += path_fixes;

	// Phase 2: Parse XML, merge ClInclude/ClCompile into one sorted ItemGroup
	var doc = XDocument.Parse(fixed_content);
	var ns = doc.Root!.Name.Namespace;

	var source_items = doc.Root.Elements(ns + "ItemGroup")
		.SelectMany(ig => ig.Elements())
		.Where(e => e.Name.LocalName is "ClInclude" or "ClCompile")
		.ToList();

	var sorted_count = source_items.Count;
	if (sorted_count > 0)
	{
		// Sort all ClInclude/ClCompile by their Include path
		var sorted = source_items
			.OrderBy(e => e.Attribute("Include")?.Value ?? "", StringComparer.OrdinalIgnoreCase)
			.Select(e => new XElement(e))
			.ToList();

		// Remove the originals from all ItemGroups
		foreach (var ig in doc.Root.Elements(ns + "ItemGroup").ToList())
		{
			ig.Elements()
				.Where(e => e.Name.LocalName is "ClInclude" or "ClCompile")
				.Remove();

			// Drop now-empty ItemGroups
			if (!ig.HasElements)
				ig.Remove();
		}

		// Create a single merged ItemGroup with sorted items
		var merged = new XElement(ns + "ItemGroup");
		foreach (var item in sorted)
			merged.Add(item);

		// Insert after the Filter definitions ItemGroup
		var filter_group = doc.Root.Elements(ns + "ItemGroup")
			.LastOrDefault(ig => ig.Elements(ns + "Filter").Any());

		if (filter_group != null)
			filter_group.AddAfterSelf(merged);
		else
			doc.Root.AddFirst(merged);
	}

	if (path_fixes > 0 || sorted_count > 0)
	{
		var rel = Path.GetRelativePath(root, filepath);
		total_files_modified++;

		var parts = new List<string>();
		if (path_fixes > 0) parts.Add($"{path_fixes} path{Plural(path_fixes)} fixed");
		if (sorted_count > 0) parts.Add($"{sorted_count} items sorted");
		var summary = string.Join(", ", parts);

		if (dry_run)
		{
			Console.WriteLine($"  [dry-run] {rel} ({summary})");
		}
		else
		{
			using (var writer = new StreamWriter(filepath, false, encoding))
				doc.Save(writer);

			Console.WriteLine($"  Updated: {rel} ({summary})");
		}
	}
}

Console.WriteLine($"\nTotal: {total_files_modified} file{Plural(total_files_modified)} modified, {total_path_fixes} path{Plural(total_path_fixes)} fixed");

// --- Helper methods ---

static string Plural(int n) => n != 1 ? "s" : "";

/// <summary>Read a file, detecting encoding from BOM to preserve it on write-back.</summary>
static (string content, Encoding encoding) ReadFile(string filepath)
{
	Encoding encoding;
	string content;
	using (var reader = new StreamReader(filepath, detectEncodingFromByteOrderMarks: true))
	{
		content = reader.ReadToEnd();
		encoding = reader.CurrentEncoding;
	}
	return (content, encoding);
}

/// <summary>
/// Resolve a relative path against a base directory by walking '..' segments.
/// Safe for paths containing wildcards (unlike Path.GetFullPath).
/// </summary>
static string? ResolvePath(string base_dir, string relative_path)
{
	relative_path = relative_path.Replace('/', '\\');

	var parts = relative_path.Split('\\');
	var dir = base_dir.TrimEnd('\\').Split('\\').ToList();

	int i = 0;
	for (; i < parts.Length && parts[i] == ".."; i++)
	{
		if (dir.Count == 0)
			return null;
		dir.RemoveAt(dir.Count - 1);
	}

	var remaining = parts.Skip(i).ToArray();
	if (remaining.Length == 0)
		return null;

	return string.Join('\\', dir) + '\\' + string.Join('\\', remaining);
}
