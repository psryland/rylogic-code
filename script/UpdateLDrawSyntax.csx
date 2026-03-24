#! "net10.0"
#load "UserVars.csx"
#load "Tools.csx"
#nullable enable

// Use: dotnet-script .\UpdateLDrawSyntax

using System;
using System.Text.RegularExpressions;
using Console = System.Console;

public class UpdateLDrawSyntax
{
	// Update the LdrSyntaxRules.xshd file
	public static void UpdateSyntaxRules()
	{
		var ldraw_h = Tools.Path([UserVars.Root, "include\\pr\\view3d-12\\ldraw\\ldraw.h"]);
		var project_dir = Tools.Path([UserVars.Root, "projects\\apps\\LDraw"]);
		var rules_filepath = Tools.Path([project_dir, "res", "LdrSyntaxRules.xshd"]);
		UpdateELdrObjects(rules_filepath, ldraw_h);
		UpdateEKeywords(rules_filepath, ldraw_h);
	}

	// Update the ELdrObject keywords
	public static void UpdateELdrObjects(string rules_filepath, string ldraw_h)
	{
		// Read the ELdrObject enum values
		var pat = new Regex(@"#define PR_ENUM_LDRAW_OBJECTS\(x\)\\(.*?)ELdrObject", RegexOptions.Singleline);
		var section = Tools.Extract(ldraw_h, pat, by_line: false).Groups[1].Value;

		// Generate the ldraw object keywords
		var ldraw_objects = new StringBuilder();
		foreach (var line in section.Split('\n'))
		{
			var m = Regex.Match(line, @"\s*x\((.*?)\s*,.*");
			if (!m.Success)
				continue;

			var kw = m.Groups[1].Value;
			if (kw == "Unknown")
				continue;

			ldraw_objects.Append($"\t\t\t<Word>*{kw}</Word>\n");
		}

		var tag_beg = "\t\t\t<!-- ##AUTO-GENERATED## ELdrObject Begin -->\n";
		var tag_end = "\t\t\t<!-- ##AUTO-GENERATED## ELdrObject End -->\n";
		Tools.UpdateTaggedSection(rules_filepath, tag_beg, tag_end, ldraw_objects.ToString());
	}

	// Update the EKeyword keywords
	public static void UpdateEKeywords(string rules_filepath, string ldraw_h)
	{
		// Read the EKeyword enum values
		var pat = new Regex(@"#define PR_ENUM_LDRAW_KEYWORDS\(x\)\\(.*?)EKeyword", RegexOptions.Singleline);
		var section = Tools.Extract(ldraw_h, pat, by_line: false).Groups[1].Value;

		// Generate the ldr object keywords
		var keywords = new StringBuilder();
		foreach (var line in section.Split('\n'))
		{
			var m = Regex.Match(line, @"\s*x\((.*?)\s*,.*");
			if (!m.Success)
				continue;

			var kw = m.Groups[1].Value;
			keywords.Append($"\t\t\t<Word>*{kw}</Word>\n");
		}

		var tag_beg = "\t\t\t<!-- ##AUTO-GENERATED## EKeyword Begin -->\n";
		var tag_end = "\t\t\t<!-- ##AUTO-GENERATED## EKeyword End -->\n";
		Tools.UpdateTaggedSection(rules_filepath, tag_beg, tag_end, keywords.ToString());
	}

	// Update the EKeyword keywords
	public static void UpdateEPreprocessor(string rules_filepath, string ldraw_h)
	{
		// Read the EKeyword enum values
		var pat = new Regex(@"#define PR_ENUM_LDRAW_KEYWORDS\(x\)\\(.*?)EKeyword", RegexOptions.Singleline);
		var section = Tools.Extract(ldraw_h, pat, by_line: false).Groups[1].Value;

		// Generate the ldr object keywords
		var keywords = new StringBuilder();
		foreach (var line in section.Split('\n'))
		{
			var m = Regex.Match(line, @"\s*x\((.*?)\s*,.*");
			if (!m.Success)
				continue;

			var kw = m.Groups[1].Value;
			keywords.Append($"\t\t\t<Word>*{kw}</Word>\n");
		}

		var tag_beg = "\t\t\t<!-- ##AUTO-GENERATED## EKeyword Begin -->\n";
		var tag_end = "\t\t\t<!-- ##AUTO-GENERATED## EKeyword End -->\n";
		Tools.UpdateTaggedSection(rules_filepath, tag_beg, tag_end, keywords.ToString());
	}
}

try
{
	UpdateLDrawSyntax.UpdateSyntaxRules();
}
catch (Exception ex)
{
	Console.Error.WriteLine(ex.Message);
}
