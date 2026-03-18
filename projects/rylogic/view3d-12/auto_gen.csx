#! "net9.0"
// Auto generator for View3d.
// Use:
//  dotnet-script.exe auto_gen.csx $(RylogicRoot)
#r "System.IO"
#r "System.Text.RegularExpressions"
#r "nuget: Rylogic.Core, 2.0.0"
#nullable enable

using System;
using System.Text.RegularExpressions;
using Rylogic.Utility;

static Regex Pattern = new Regex(@"\s*x\(\s*(.*?)\s*\).*");
string Root = Args.Count != 0 ? Args[0] : "E:\\Rylogic\\Code";

// Do the auto gen
void AutoGen()
{
	// Embed 'ldraw_demo_scene.ldr' into 'ldraw_demo_scene.cpp'
	ReplaceSection(
		Path.Join(Root, "projects/rylogic/view3d-12/src/ldraw/ldraw_demo_scene.ldr"),
		Path.Join(Root, "projects/rylogic/view3d-12/src/ldraw/ldraw_demo_scene.cpp"),
		null,
		null,
		"// AUTO-GENERATED-DEMOSCENE-BEGIN",
		"// AUTO-GENERATED-DEMOSCENE-END",
		TransformToCppString);

	// Update 'LDraw.cs'
	ReplaceSection(
		Path.Join(Root, "include/pr/view3d-12/ldraw/ldraw.h"),
		Path.Join(Root, "projects/rylogic/Rylogic.Gfx/src/LDraw/LDraw.cs"),
		"#define PR_LDRAW_KEYWORDS(x)",
		"PR_LDRAW_KEYWORDS_END",
		"// AUTO-GENERATED-KEYWORDS-BEGIN",
		"// AUTO-GENERATED-KEYWORDS-END",
		TransformEnumLineToCSharp);
	ReplaceSection(
		Path.Join(Root, "include/pr/view3d-12/ldraw/ldraw.h"),
		Path.Join(Root, "projects/rylogic/Rylogic.Gfx/src/LDraw/LDraw.cs"),
		"#define PR_LDRAW_COMMANDS(x)",
		"PR_LDRAW_COMMANDS_END",
		"// AUTO-GENERATED-COMMANDS-BEGIN",
		"// AUTO-GENERATED-COMMANDS-END",
		TransformEnumLineToCSharp);

	// Update 'ldraw.py'
	ReplaceSection(
		Path.Join(Root, "include/pr/view3d-12/ldraw/ldraw.h"),
		Path.Join(Root, "projects/rylogic/py-rylogic/rylogic/ldraw/ldraw.py"),
		"#define PR_LDRAW_KEYWORDS(x)",
		"PR_LDRAW_KEYWORDS_END",
		"# AUTO-GENERATED-KEYWORDS-BEGIN",
		"# AUTO-GENERATED-KEYWORDS-END",
		TransformEnumLineToPython);
	ReplaceSection(
		Path.Join(Root, "include/pr/view3d-12/ldraw/ldraw.h"),
		Path.Join(Root, "projects/rylogic/py-rylogic/rylogic/ldraw/ldraw.py"),
		"#define PR_LDRAW_COMMANDS(x)",
		"PR_LDRAW_COMMANDS_END",
		"# AUTO-GENERATED-COMMANDS-BEGIN",
		"# AUTO-GENERATED-COMMANDS-END",
		TransformEnumLineToPython);

	// Update 'common/ldraw.h' keyword string constants
	ReplaceSection(
		Path.Join(Root, "include/pr/view3d-12/ldraw/ldraw.h"),
		Path.Join(Root, "include/pr/common/ldraw.h"),
		"#define PR_LDRAW_KEYWORDS(x)",
		"PR_LDRAW_KEYWORDS_END",
		"// AUTO-GENERATED-KEYWORDS-BEGIN",
		"// AUTO-GENERATED-KEYWORDS-END",
		TransformEnumLineToNameValue);
	ReplaceSection(
		Path.Join(Root, "include/pr/view3d-12/ldraw/ldraw.h"),
		Path.Join(Root, "include/pr/common/ldraw.h"),
		"#define PR_LDRAW_COMMANDS(x)",
		"PR_LDRAW_COMMANDS_END",
		"// AUTO-GENERATED-COMMANDS-BEGIN",
		"// AUTO-GENERATED-COMMANDS-END",
		TransformEnumLineToNameValueNoStar);

	// Update keywords in LDRTemplate.bt
	ReplaceSection(
		Path.Join(Root, "include/pr/view3d-12/ldraw/ldraw.h"),
		Path.Join(Root, "miscellaneous/010 templates/LDRTemplate.bt"),
		"#define PR_LDRAW_KEYWORDS(x)",
		"PR_LDRAW_KEYWORDS_END",
		"// AUTO-GENERATED-KEYWORDS-BEGIN",
		"// AUTO-GENERATED-KEYWORDS-END",
		TransformEnumLineTo010Template);
	ReplaceSection(
		Path.Join(Root, "include/pr/view3d-12/ldraw/ldraw.h"),
		Path.Join(Root, "miscellaneous/010 templates/LDRTemplate.bt"),
		"#define PR_LDRAW_COMMANDS(x)",
		"PR_LDRAW_COMMANDS_END",
		"// AUTO-GENERATED-COMMANDS-BEGIN",
		"// AUTO-GENERATED-COMMANDS-END",
		TransformEnumLineTo010Template);
}

// Convert from a line of text to a C++ string literal
string TransformToCppString(string line)
{
	line = line.Replace("\\", "\\\\");
	line = line.Replace("\"", "\\\"");
	return $"\"{line}\\n\"";
}

// Convert from C++ code macro to C# literal
string TransformEnumLineToCSharp(string line)
{
	var match = Pattern.Match(line.Trim());
	return match.Success ? $"{match.Groups[1].Value} = unchecked((int){HashI(match.Groups[1].Value)})," : line.Trim();
}

// Convert from C++ code macro to python
string TransformEnumLineToPython(string line)
{
	var match = Pattern.Match(line.Trim());
	return match.Success ? $"{match.Groups[1].Value} = {HashI(match.Groups[1].Value)}" : line.Trim();
}

// Convert from C++ code macro to keyword with name and hash value
string TransformEnumLineToNameValue(string line)
{
	var match = Pattern.Match(line.Trim());
	return match.Success ? $"inline static constexpr NameValue {match.Groups[1].Value} = {{\"*{match.Groups[1].Value}\", {HashI(match.Groups[1].Value)}}};" : line.Trim();
}

// Convert from C++ code macro to keyword with name and hash value
string TransformEnumLineToNameValueNoStar(string line)
{
	var match = Pattern.Match(line.Trim());
	return match.Success ? $"inline static constexpr NameValue {match.Groups[1].Value} = {{\"{match.Groups[1].Value}\", {HashI(match.Groups[1].Value)}}};" : line.Trim();
}

// Convert from C++ code macro to C++
string TransformEnumLineTo010Template(string line)
{
	var match = Pattern.Match(line.Trim());
	return match.Success ? $"{match.Groups[1].Value} = {HashI(match.Groups[1].Value)}," : line.Trim();
}

// Replace a block of lines within a file from lines from another file
// If 'src_tag_begin' and 'src_tag_end' are None, the entire file is used
// If 'dst_tag_begin' and 'dst_tag_end' are None, the entire file is replaced
// 'transform' has a signature of: "transform(line) -> line". Result should have no trailing newline or leading whitespace
void ReplaceSection(string src_file, string dst_file, string? src_tag_begin, string? src_tag_end, string? dst_tag_begin, string? dst_tag_end, Func<string, string> transform)
{
	if (!File.Exists(src_file))
		throw new FileNotFoundException($"Cannot read section in '{src_file}'.");
	if (!File.Exists(dst_file))
		throw new FileNotFoundException($"Cannot update section in '{dst_file}'.");

	var embed = (List<string>)[];
	var lines = (List<string>)[];
	var indent = "";

	// Find the lines to embed between the src tags
	var within_tags = src_tag_begin is null && src_tag_end is null;
	foreach (var line in File.ReadLines(src_file, Encoding.UTF8))
	{
		if (src_tag_begin is not null && line.Contains(src_tag_begin))
		{
			within_tags = true;
			continue;
		}
		if (src_tag_end is not null && line.Contains(src_tag_end))
		{
			within_tags = false;
			break;
		}
		if (within_tags)
		{
			embed.Add(line);
		}
	}

	// Read the 'dst' file and embed the lines between the dst tags
	within_tags = dst_tag_begin is null && dst_tag_end is null;
	foreach (var line in File.ReadLines(dst_file, Encoding.UTF8))
	{
		if (dst_tag_begin is not null && line.Contains(dst_tag_begin))
		{
			within_tags = true;
			indent = line[..line.IndexOf(dst_tag_begin)];
			lines.Add(line);
			lines.AddRange(embed.Select(l => $"{indent}{transform(l)}"));
			continue;
		}
		if (dst_tag_end is not null && line.Contains(dst_tag_end))
		{
			within_tags = false;
			lines.Add(line);
			continue;
		}
		if (!within_tags)
		{
			lines.Add(line);
		}
	}

	// Update the dst file
	using var file = File.CreateText(dst_file);
	foreach (var line in lines)
		file.WriteLine(line);
}

// Hash a string to a constant value
uint HashI(string s)
{
	const uint _FNV_offset_basis32 = 2166136261;
	const uint _FNV_prime32 = 16777619;
	uint h = _FNV_offset_basis32;
	foreach (char c in s.ToLower())
	{
		h = (uint)(0xFFFFFFFF & ((h ^ (int)c) * _FNV_prime32));
	}
	return h;
}

// Checks
if (HashI("Box") != 1892056626)
	throw new Exception("HashI() function is not working as expected");

AutoGen();
