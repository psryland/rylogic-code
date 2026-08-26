#! "net10.0"
#r "System.Xml.Linq"
#load "UserVars.csx"
#load "Tools.csx"
#load "NativeRuntimePackage.csx"
#nullable enable

using System;
using IOPath = System.IO.Path;

// Stages a runtime-only Rylogic.Native payload for MSBuild-driven local development packages.
void Main(IList<string> args)
{
	var workspace = UserVars.Root;
	var platform = "x64";
	var config = "Debug";
	string? output_dir = null;
	var require_all_projects = false;

	for (var i = 0; i != args.Count;)
	{
		var arg = args[i++];
		switch (arg.ToLowerInvariant())
		{
			case "-workspace":
			{
				workspace = args[i++];
				break;
			}
			case "-platform":
			{
				platform = args[i++];
				break;
			}
			case "-config":
			{
				config = args[i++];
				break;
			}
			case "-output":
			{
				output_dir = args[i++];
				break;
			}
			case "-requireall":
			{
				require_all_projects = true;
				break;
			}
			default:
			{
				throw new ArgumentException($"Unknown command line argument: {arg}");
			}
		}
	}

	if (string.IsNullOrWhiteSpace(output_dir))
		throw new ArgumentException("Native runtime staging output path is required.");

	NativeRuntimePackage.Stage(workspace, platform, config, output_dir, require_all_projects);
}

Main(Args);
