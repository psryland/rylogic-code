using System.Globalization;
using System.Text.RegularExpressions;

namespace vsredirector;

// A single file request, optionally including a one-based text line.
internal sealed record Request(string FilePath, int? Line, bool Inspect)
{
	// Accept the devenv navigation command, not arbitrary IDE commands.
	public static Request Parse(string[] args)
	{
		string? file = null;
		int? line = null;
		var inspect = false;
		var edit = false;
		for (var i = 0; i != args.Length; ++i)
		{
			var arg = args[i];
			if (arg.Equals("--inspect", StringComparison.OrdinalIgnoreCase))
			{
				if (inspect) throw new ArgumentException("Duplicate --inspect.");
				inspect = true;
			}
			else if (arg.Equals("/edit", StringComparison.OrdinalIgnoreCase) || arg.Equals("-edit", StringComparison.OrdinalIgnoreCase))
			{
				if (edit) throw new ArgumentException("Duplicate /Edit.");
				edit = true;
			}
			else if (arg.Equals("/command", StringComparison.OrdinalIgnoreCase) || arg.Equals("-command", StringComparison.OrdinalIgnoreCase))
			{
				if (++i == args.Length || line != null)
					throw new ArgumentException("Supply one /Command \"Edit.Goto <positive line>\".");

				var match = Regex.Match(args[i], @"\AEdit\.Goto\s+([0-9]+)\z", RegexOptions.IgnoreCase | RegexOptions.CultureInvariant);
				if (!match.Success || !int.TryParse(match.Groups[1].Value, NumberStyles.None, CultureInfo.InvariantCulture, out var number) || number <= 0)
					throw new ArgumentException("Only /Command \"Edit.Goto <positive line>\" is supported.");

				line = number;
			}
			else if (arg.StartsWith('-') || arg.StartsWith('/'))
			{
				throw new ArgumentException($"Unknown option: {arg}. There is no devenv /line switch; use /Command \"Edit.Goto 42\".");
			}
			else
			{
				if (file != null || string.IsNullOrWhiteSpace(arg))
					throw new ArgumentException("Supply exactly one quoted file path.");

				file = Paths.Normalize(arg);
			}
		}
		if (file == null)
			throw new ArgumentException("Usage: VSRedirector.exe [--inspect] [/Edit] \"file\" [/Command \"Edit.Goto 42\"]");

		return new Request(file, line, inspect);
	}
}

// Windows full-path identity keeps independent clones distinct; it does not resolve filesystem aliases.
internal static class Paths
{
	public static string Normalize(string path)
	{
		return Path.GetFullPath(path);
	}
	public static bool Same(string left, string right)
	{
		return !string.IsNullOrEmpty(left) && !string.IsNullOrEmpty(right) && string.Equals(Normalize(left), Normalize(right), StringComparison.OrdinalIgnoreCase);
	}
}

// Facts from one completely inspected, running IDE; PID is identity, never an activity ranking.
internal sealed record Instance(int ProcessId, string Solution, bool ContainsFile, bool DocumentOpen, int WindowRank);

internal static class Routing
{
	// A null result means a genuine no-match. Window stacking order is the approved approximation, not historical activity.
	public static Instance? Select(IReadOnlyList<Instance> instances)
	{
		var matches = instances.Where(x => x.ContainsFile).ToArray();
		if (matches.Length != 0)
		{
			var open = matches.Where(x => x.DocumentOpen).ToArray();
			if (open.Length != 0) matches = open;
		}
		else
		{
			matches = instances.Where(x => x.DocumentOpen).ToArray();
		}
		if (matches.Length > 1 && matches.Any(x => x.WindowRank == int.MaxValue))
			throw new InvalidOperationException("A matching Visual Studio window is not visible in this desktop's stacking order. Make the IDE windows visible and retry.");

		return matches.OrderBy(x => x.WindowRank).FirstOrDefault();
	}
}

#if PR_UNITTESTS
internal static partial class Tests
{
	private static void TestRouting()
	{
		var solution = new Instance(1, @"C:\a\test.sln", true, false, 4);
		var loose = new Instance(2, "", false, true, 0);
		var solution_open = new Instance(3, @"C:\b\test.sln", true, true, 5);
		Check(Routing.Select([loose, solution]) == solution, "solution beats loose document");
		Check(Routing.Select([solution, solution_open, loose]) == solution_open, "open document breaks solution tie");
		Check(Routing.Select([loose]) == loose, "loose document reuse");
		Check(Routing.Select([]) == null, "empty discovery fallback");
		Check(Routing.Select([solution with { ContainsFile = false }]) == null, "unmatched fallback");
		Check(Routing.Select([solution, solution with { ProcessId = 4, WindowRank = 2 }])?.ProcessId == 4, "solution stacking-order tie");
		Check(Routing.Select([solution_open, solution_open with { ProcessId = 4, WindowRank = 2 }])?.ProcessId == 4, "open solution stacking-order tie");
		Check(Routing.Select([loose with { WindowRank = 10 }, loose with { ProcessId = 4 }])?.ProcessId == 4, "loose document stacking-order tie");
		Throws(() => Routing.Select([solution, solution with { WindowRank = int.MaxValue }]), "unavailable stacking order");
		Check(Paths.Same(@"C:\clone a\src\..\file.cs", @"c:\CLONE A\file.cs"), "full path normalization");
		Check(!Paths.Same(@"C:\clone a\file.cs", @"C:\clone b\file.cs"), "clone isolation");
		Check(!Paths.Same(@"C:\clone\file.cs", @"C:\clone\file.cs.bak"), "no prefix identity");
		var request = Request.Parse(["/Edit", @"C:\clone a\file.cs", "/Command", "Edit.Goto 42"]);
		Check(request.FilePath == @"C:\clone a\file.cs" && request.Line == 42, "devenv file and line");
		Check(Request.Parse(["-command", "edit.goto 1", @"C:\a.cs"]).Line == 1, "switch order and casing");
		Check(Request.Parse(["--inspect", @"C:\a.cs"]).Inspect, "read-only inspection");
		var markdown = @"E:\Copilot\copilot-skills\copilot-instructions.md";
		Check(Request.Parse([markdown]) == new Request(markdown, null, false), "reported Markdown path parses unchanged");
		Check(Request.Parse(["/Edit", markdown, "/Command", "Edit.Goto 1"]) == new Request(markdown, 1, false), "reported Markdown path with line navigation");
		foreach (var args in new string[][] {
			[], ["a.cs", "b.cs"], ["a.cs", "/line", "42"], ["a.cs", "/command"],
			["a.cs", "/command", "Edit.Goto 0"], ["a.cs", "/command", "Edit.Goto -1"],
			["a.cs", "/command", "Edit.Goto 2147483648"], ["a.cs", "/command", "File.Exit"],
			["a.cs", "/command", "Edit.Goto 1", "/command", "Edit.Goto 2"] })
			Throws(() => Request.Parse(args), "invalid argument rejection");
	}
}
#endif
