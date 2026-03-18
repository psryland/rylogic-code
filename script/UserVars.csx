#! "net9.0"
#r "System.Text.Json"
#r "nuget: Rylogic.Core, 2.0.2"
#nullable enable

using System;
using System.IO;
using System.Diagnostics;
using System.Text.Json;
using Rylogic.Common;
using Rylogic.Utility;

public class UserVars
{
	// Version History:
	//  1 - initial version
	//  2 - dynamic tool discovery (no hardcoded absolute paths)
	public static int Version => 2;

	/// <summary>Location of the root for the code library</summary>
	public static string Root => m_root ??= Path([Util.ThisDirectory(), ".."]);
	private static string? m_root;

	/// <summary>Location for trash/temp files</summary>
	public static string DumpDir => m_dump ??= Path([Root, "Dump"], check_exists: false);
	private static string? m_dump;

	/// <summary>The dotnet compiler</summary>
	public static string Dotnet => m_dotnet ??= FindOnPath("dotnet");
	private static string? m_dotnet;

	/// <summary>Visual Studio installation directory (discovered via vswhere)</summary>
	public static string VSDir => m_vs_dir ??= FindVSInstallation();
	private static string? m_vs_dir;

	/// <summary>MSBuild path. Derived from VSDir</summary>
	public static string MSBuild => Path([VSDir, "MSBuild\\Current\\Bin\\MSBuild.exe"]);

	/// <summary>The version of installed Visual Studio (discovered from vswhere)</summary>
	public static string VSVersion => m_vs_version ??= DetectVSVersion();
	private static string? m_vs_version;

	/// <summary>The VC tools version (latest installed under VSDir)</summary>
	public static string VCVersion => m_vc_version ??= DetectVCVersion();
	private static string? m_vc_version;

	/// <summary>The build system version. VS2013 == v120, VS2012 = v110, etc</summary>
	public static string PlatformToolset => m_platform_toolset ??= DerivePlatformToolset();
	private static string? m_platform_toolset;

	/// <summary>Power shell (discovered from PATH)</summary>
	public static string Pwsh => m_pwsh ??= FindOnPath("pwsh");
	private static string? m_pwsh;

	/// <summary>git path (discovered from PATH)</summary>
	public static string Git => m_git ??= FindOnPath("git");
	private static string? m_git;

	/// <summary>Azure Trusted Signing account name (from Azure portal). Empty if not configured.</summary>
	public static string AzureSignAccount
	{
		get => m_azure_sign_account ??= UserSecret("RylogicAzureSignAccount") ?? "";
		set => m_azure_sign_account = value;
	}
	private static string? m_azure_sign_account = null;

	/// <summary>Azure Trusted Signing certificate profile name. Empty if not configured.</summary>
	public static string AzureSignProfile
	{
		get => m_azure_sign_profile ??= UserSecret("RylogicAzureSignProfile") ?? "";
		set => m_azure_sign_profile = value;
	}
	private static string? m_azure_sign_profile = null;

	/// <summary>Nuget package manager</summary>
	public static string Nuget => Path([Root, "tools\\nuget\\nuget.exe"]);

	/// <summary>API Key for publishing nuget packages (regenerated every 6months)</summary>
	public static string NugetApiKey
	{
		get => m_nuget_api_key ??= UserSecret("RylogicNugetAPIKey") ?? throw new Exception("Nuget API Key no set");
		set => m_nuget_api_key = value;
	}
	private static string? m_nuget_api_key = null;

	/// <summary>The full path to the windows sdk</summary>
	public static string WinSDK => m_win_sdk ??= FindWinSDK();
	private static string? m_win_sdk;

	/// <summary>The windows sdk version (latest installed)</summary>
	public static string WinSDKVersion => m_win_sdk_version ??= DetectWinSDKVersion();
	private static string? m_win_sdk_version;

	/// <summary>The root of the .NET framework directory</summary>
	public static string NETFrameworkDir => Path(["C:\\Windows\\Microsoft.NET\\Framework", "v4.0.30319"]);

	/// <summary>Path helper</summary>
	public static string Path(IEnumerable<string> path_parts, bool check_exists = true, bool normalise = true)
	{
		var path = Path_.CombinePath(path_parts);
		if (string.IsNullOrEmpty(path))
			return string.Empty;

		if (check_exists && !Path_.PathExists(path))
			throw new Exception($"Path '{path}' does not exist.");

		if (normalise)
			path = Path_.Canonicalise(path);
		
		return path;
	}

	/// <summary>Prompt for input</summary>
	public static string Prompt(string message)
	{
		Console.Write(message);
		var input = Console.ReadLine();
		if (string.IsNullOrEmpty(input))
			throw new Exception($"No input provided for '{message}'.");

		return input;
	}

	/// <summary>Prompt for a file</summary>
	public static string Browse(string message, string filter, string default_name)
	{
		throw new NotImplementedException("Browse method is not implemented. Use OpenFileDialog in a WPF context.");
	}

	/// <summary>Try to read a user secret with the given name</summary>
	public static string? UserSecret(string name)
	{
		try
		{
			var secrets_filepath = Path([Environment.GetFolderPath(Environment.SpecialFolder.UserProfile), ".dotnet", "user-secrets", "secrets.json"], check_exists: false);
			if (File.Exists(secrets_filepath))
			{
				using var file = File.OpenText(secrets_filepath);
				using var json = JsonDocument.Parse(file.ReadToEnd());
				if (json.RootElement.TryGetProperty(name, out var secret))
					return secret.GetString();
			}
		}
		catch (Exception ex)
		{
			Debug.WriteLine($"Error reading user-secrets: {ex.Message}");
		}
		return null;
	}

	/// <summary>Find a tool on PATH using where.exe</summary>
	private static string FindOnPath(string tool_name)
	{
		try
		{
			var psi = new ProcessStartInfo("where.exe", tool_name)
			{
				RedirectStandardOutput = true,
				UseShellExecute = false,
				CreateNoWindow = true,
			};
			using var proc = Process.Start(psi);
			if (proc != null)
			{
				var output = proc.StandardOutput.ReadLine()?.Trim();
				proc.WaitForExit();
				if (!string.IsNullOrEmpty(output) && File.Exists(output))
					return output;
			}
		}
		catch { }
		throw new Exception($"Could not find '{tool_name}' on PATH. Install it or set the path in Script/UserVars.json.");
	}

	/// <summary>Find Visual Studio installation using vswhere.exe</summary>
	private static string FindVSInstallation()
	{
		var program_files = Environment.GetFolderPath(Environment.SpecialFolder.ProgramFilesX86);
		var vswhere = System.IO.Path.Combine(program_files, "Microsoft Visual Studio", "Installer", "vswhere.exe");
		if (File.Exists(vswhere))
		{
			try
			{
				var psi = new ProcessStartInfo(vswhere, "-latest -property installationPath")
				{
					RedirectStandardOutput = true,
					UseShellExecute = false,
					CreateNoWindow = true,
				};
				using var proc = Process.Start(psi);
				if (proc != null)
				{
					var output = proc.StandardOutput.ReadLine()?.Trim();
					proc.WaitForExit();
					if (!string.IsNullOrEmpty(output) && Directory.Exists(output))
						return output;
				}
			}
			catch { }
		}
		throw new Exception("Could not find Visual Studio via vswhere. Install VS or set VSDir in Script/UserVars.json.");
	}

	/// <summary>Detect VS version via vswhere</summary>
	private static string DetectVSVersion()
	{
		var program_files = Environment.GetFolderPath(Environment.SpecialFolder.ProgramFilesX86);
		var vswhere = System.IO.Path.Combine(program_files, "Microsoft Visual Studio", "Installer", "vswhere.exe");
		if (File.Exists(vswhere))
		{
			try
			{
				var psi = new ProcessStartInfo(vswhere, "-latest -property installationVersion")
				{
					RedirectStandardOutput = true,
					UseShellExecute = false,
					CreateNoWindow = true,
				};
				using var proc = Process.Start(psi);
				if (proc != null)
				{
					var output = proc.StandardOutput.ReadLine()?.Trim();
					proc.WaitForExit();
					if (!string.IsNullOrEmpty(output))
					{
						// Return major.0 (e.g., "18.0" from "18.4.11612.150")
						var parts = output.Split('.');
						if (parts.Length >= 1)
							return $"{parts[0]}.0";
					}
				}
			}
			catch { }
		}
		throw new Exception("Could not detect VS version. Set VSVersion in Script/UserVars.json.");
	}

	/// <summary>Detect the latest VC tools version installed under VSDir</summary>
	private static string DetectVCVersion()
	{
		var msvc_dir = System.IO.Path.Combine(VSDir, "VC", "Tools", "MSVC");
		if (Directory.Exists(msvc_dir))
		{
			var latest = Directory.GetDirectories(msvc_dir)
				.Select(System.IO.Path.GetFileName)
				.Where(n => !string.IsNullOrEmpty(n))
				.OrderByDescending(n => n)
				.FirstOrDefault();
			if (latest != null)
				return latest;
		}
		throw new Exception("Could not detect VC tools version. Set VCVersion in Script/UserVars.json.");
	}

	/// <summary>Derive the platform toolset from the VS major version</summary>
	private static string DerivePlatformToolset()
	{
		var major = VSVersion.Split('.')[0];
		return major switch
		{
			"18" => "v145",
			"17" => "v143",
			"16" => "v142",
			"15" => "v141",
			"14" => "v140",
			_ => throw new Exception($"Unknown VS major version '{major}'. Set PlatformToolset in Script/UserVars.json."),
		};
	}

	/// <summary>Find the Windows SDK installation directory</summary>
	private static string FindWinSDK()
	{
		string[] candidates = [
			System.IO.Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.ProgramFilesX86), "Windows Kits", "10"),
			@"C:\Program Files\Windows Kits\10",
		];
		foreach (var path in candidates)
		{
			if (Directory.Exists(path))
				return path;
		}
		throw new Exception("Could not find Windows SDK. Set WinSDK in Script/UserVars.json.");
	}

	/// <summary>Detect the latest Windows SDK version installed</summary>
	private static string DetectWinSDKVersion()
	{
		var include_dir = System.IO.Path.Combine(WinSDK, "Include");
		if (Directory.Exists(include_dir))
		{
			var latest = Directory.GetDirectories(include_dir)
				.Select(System.IO.Path.GetFileName)
				.Where(n => !string.IsNullOrEmpty(n) && n.StartsWith("10."))
				.OrderByDescending(n => n)
				.FirstOrDefault();
			if (latest != null)
				return latest;
		}
		throw new Exception("Could not detect Windows SDK version. Set WinSDKVersion in Script/UserVars.json.");
	}

	/// <summary>Load user provided defaults</summary>
	static UserVars()
	{
		try
		{
			// Load user provided defaults from UserVars.json (overrides auto-discovery)
			var uservars_json = Path_.CombinePath(Root, "Script/UserVars.json");
			if (Path_.FileExists(uservars_json))
			{
				using var file = File.OpenText(uservars_json);
				using var json = JsonDocument.Parse(file.ReadToEnd());

				foreach (var prop in json.RootElement.EnumerateObject())
				{
					switch (prop.Name)
					{
						case "DumpDir": m_dump = prop.Value.GetString(); break;
						case "VSDir": m_vs_dir = prop.Value.GetString(); break;
						case "VSVersion": m_vs_version = prop.Value.GetString(); break;
						case "VCVersion": m_vc_version = prop.Value.GetString(); break;
						case "PlatformToolset": m_platform_toolset = prop.Value.GetString(); break;
						case "Dotnet": m_dotnet = prop.Value.GetString(); break;
						case "Git": m_git = prop.Value.GetString(); break;
						case "Pwsh": m_pwsh = prop.Value.GetString(); break;
						case "WinSDK": m_win_sdk = prop.Value.GetString(); break;
						case "WinSDKVersion": m_win_sdk_version = prop.Value.GetString(); break;
						case "AzureSignAccount": m_azure_sign_account = prop.Value.GetString(); break;
						case "AzureSignProfile": m_azure_sign_profile = prop.Value.GetString(); break;
						default:
						{
							if (prop.Name == "") break; // Ignore empty names
							throw new Exception($"Unknown property '{prop.Name}' in {uservars_json}");
						}
					}
				}
			}
		}
		catch (Exception ex)
		{
			Console.Error.WriteLine($"Error loading user variables: {ex.Message}");
			System.Environment.Exit(1);
		}
	}
}
