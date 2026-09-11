#! "net10.0"
#r "System.Xml.Linq"
#load "UserVars.csx"
#load "Tools.csx"
#load "NativeRuntimePackage.csx"
#nullable enable

using System;
using IOPath = System.IO.Path;

// Stages the full Rylogic.Native payload for MSBuild-driven local development packages.
void Main(IList<string> args)
{
	var workspace = UserVars.Root;
	var platform = "x64";
	var config = "Debug";
	string? output_dir = null;
	var require_all_projects = false;
	var skip_if_incomplete = false;

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
			case "-skipincomplete":
			{
				skip_if_incomplete = true;
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
	if (skip_if_incomplete && !require_all_projects)
		throw new ArgumentException("-skipincomplete requires -requireall.");

	// Preserve the last complete local package when a partial native build cannot provide the full package closure.
	if (skip_if_incomplete && !NativeRuntimePackage.HasCompleteManifestSet(workspace, platform, config, out var unavailable_inputs))
	{
		Console.WriteLine($"Skipping local Rylogic.Native package because its {platform}|{config} manifest set is incomplete:");
		foreach (var unavailable_input in unavailable_inputs)
			Console.WriteLine($"  {unavailable_input}");
		Console.WriteLine("Build AllNative to publish a complete local native package.");
		return;
	}

	// Supported link assets must stay in sync with the staged runtime closure so consumers never restore headers without the matching libraries.
	if (skip_if_incomplete && !NativeRuntimePackage.HasCompleteLinkAssetSet(workspace, platform, config, out var unavailable_link_assets))
	{
		Console.WriteLine($"Skipping local Rylogic.Native package because its {platform}|{config} native link assets are incomplete:");
		foreach (var unavailable_link_asset in unavailable_link_assets)
			Console.WriteLine($"  {unavailable_link_asset}");
		Console.WriteLine("Build AllNative to publish a complete local native package.");
		return;
	}

	// A package missing a command line tool is incomplete in the same way as one missing a runtime library, so the
	// preceding package is preserved rather than publishing a payload a consumer cannot cook with.
	if (skip_if_incomplete && !NativeRuntimePackage.HasCompleteToolSet(workspace, platform, config, out var unavailable_tools))
	{
		Console.WriteLine($"Skipping local Rylogic.Native package because its {platform}|{config} tools are incomplete:");
		foreach (var unavailable_tool in unavailable_tools)
			Console.WriteLine($"  {unavailable_tool}");
		Console.WriteLine("Build AllNative to publish a complete local native package.");
		return;
	}

	// Mark only a fully staged and validated closure as eligible for publication by the calling MSBuild target.
	NativeRuntimePackage.Stage(workspace, platform, config, output_dir, require_all_projects);
	if (skip_if_incomplete)
		File.WriteAllText(IOPath.Combine(output_dir, ".complete"), string.Empty);
}

Main(Args);
