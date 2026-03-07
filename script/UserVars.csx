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
	public static int Version => 1;

	/// <summary>Location of the root for the code library</summary>
	public static string Root => m_root ??= Path([Util.ThisDirectory(), ".."]);
	private static string? m_root;

	/// <summary>Location for trash/temp files</summary>
	public static string DumpDir => m_dump ??= Path([Root, "Dump"]);
	private static string? m_dump;

	/// <summary>The dotnet compiler</summary>
	public static string Dotnet => Path(["C:\\Program Files\\dotnet\\dotnet.exe"]);

	/// <summary>MSBuild path. Used by build scripts</summary>
	public static string MSBuild => Path([VSDir, "MSBuild\\Current\\Bin\\MSBuild.exe"]);

	/// <summary>The version of installed Visual Studio</summary>
	public static string VSVersion => "18.0";
	public static string VCVersion => "14.50.35717";

	/// <summary>Visual Studio installation directory</summary>
	public static string VSDir => m_vs_dir ??= Path(["C:\\Program Files\\Microsoft Visual Studio\\18\\Enterprise"]);
	private static string? m_vs_dir;
	//vs_devenv = Path(vs_dir, "Common7\\IDE\\devenv.exe")
	//vs_envvars = Path(vs_dir, "Common7\\Tools\\VsDevCmd.bat")

	/// <sumary>The build system version. VS2013 == v120, VS2012 = v110, etc
	public static string PlatformToolset => "v145";

	/// <summary>Power shell</summary>
	public static string Pwsh => Path(["C:\\Program Files\\PowerShell\\7-preview\\pwsh.exe"]);

	/// <summary>git path</summary>
	public static string Git => Path(["C:\\Program Files\\Git\\bin\\git.exe"]);

	/// <summary>Assembly sign tool</summary>
	public static string SignTool => Path([WinSDK, "bin", WinSDKVersion, "x64\\signtool.exe"]);
	public static string VsixSignTool => Path(["C:\\Program Files\\PackageManagement\\NuGet\\Packages\\Microsoft.VSSDK.VsixSignTool.16.2.29116.78\\tools\\vssdk\\vsixsigntool.exe"]);

	/// <summary>Code signing cert</summary>
	public static string CodeSignCert_Pfx => m_code_sign_cert_pfx ??= Browse("Code Signing Cert PFX: ", "PFX files (*.pfx)|*.pfx|All files (*.*)|*.*", "CodeSigningCert.pfx");
	private static string? m_code_sign_cert_pfx = null;

	/// <summary>Code signing cert thumbprint</summary>
	public static string CodeSignCert_Thumbprint => m_code_sign_cert_thumbprint ??= Prompt("Code Signing Cert Thumbprint: ");
	private static string? m_code_sign_cert_thumbprint = null;

	/// <summary>Code signing cert password</summary>
	public static string CodeSignCert_Pw
	{
		get => m_code_sign_cert_pw ??= UserSecret("RylogicCodeSigningCertPassword") ?? Prompt("Code Signing Cert Password: ");
		set => m_code_sign_cert_pw = value;
	}
	private static string? m_code_sign_cert_pw = null;

	/// <summary>Nuget package manager</summary>
	public static string Nuget => Path([Root, "tools\\nuget\\nuget.exe"]);

	/// <summary>API Key for publishing nuget packages (regenerated every 6months)</summary>
	public static string NugetApiKey
	{
		get => m_nuget_api_key ??= UserSecret("RylogicNugetAPIKey") ?? throw new Exception("Nuget API Key no set");
		set => m_nuget_api_key = value;
	}
	private static string? m_nuget_api_key = null;

	/// <summary>The full path to the windows sdk and version</summary>
	public static string WinSDKVersion => m_win_sdk_version ??= "10.0.26100.0";
	private static string? m_win_sdk_version;

	/// <summary>The full path to the windows sdk</summary>
	public static string WinSDK => m_win_sdk ??= Path(["C:\\Program Files (x86)\\Windows Kits\\10"]);
	private static string? m_win_sdk;

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

		// var file = new OpenFileDialog
		// {
		// 	Title = message,
		// 	Filter = filter,
		// 	FileName = default_name,
		// 	InitialDirectory = Environment.GetFolderPath(Environment.SpecialFolder.Desktop),
		// 	Multiselect = false,
		// 	CheckFileExists = true,
		// 	CheckPathExists = true,
		// };
		// if (file.ShowDialog() != true)
		// 	throw new Exception($"No file selected for '{message}'.");

		// return file.FileName;
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

	/// <summary>Load user provided defaults</summary>
	static UserVars()
	{
		try
		{
			// Load user provided defaults
			var uservars_json = Path_.CombinePath(Root, "Script/UserVars.json");
			if (Path_.FileExists(uservars_json))
			{
				using var file = File.OpenText(uservars_json);
				using var json = JsonDocument.Parse(file.ReadToEnd());

				foreach (var prop in json.RootElement.EnumerateObject())
				{
					switch (prop.Name)
					{
						case "DumpDir":
						{
							m_dump = prop.Value.GetString();
							break;
						}
						case "CodeSignCert_Pfx":
						{
							m_code_sign_cert_pfx = prop.Value.GetString();
							break;
						}
						case "CodeSignCert_Thumbprint":
						{
							// Get 'thumbprint' from the cert manager. Find your code signing cert (Rylogic Limited, Sectigo RSA Code Signing CA), and open it. Under 'details' find 'Thumbprint'.
							m_code_sign_cert_thumbprint = prop.Value.GetString();
							break;
						}
						case "CodeSignCert_Pw":
						{
							m_code_sign_cert_pw = prop.Value.GetString();
							break;
						}
						default:
						{
							if (prop.Name == "") break; // Ignore empty names
							throw new Exception($"Unknown property '{prop.Name}' in {uservars_json}");
						}
					}
				}
			}

			// Check for newer VCVersion
			if (VSDir is not null && VSVersion is not null)
			{
				var dir = Path([VSDir, "VC\\Tools\\MSVC"], check_exists: false);
				if (Directory.Exists(dir))
				{
					var vc_versions = Directory.EnumerateDirectories(dir)
						.Select(d => System.IO.Path.GetFileName(d))
						.Where(name => !string.IsNullOrEmpty(name))
						.OrderByDescending(name => name)
						.ToArray();
					
					// Check the latest version is being used
					if (vc_versions.Length != 0)
					{
						if (string.Compare(vc_versions[0], VCVersion, StringComparison.OrdinalIgnoreCase) > 0)
							Console.WriteLine($"Newer VC version available: {vc_versions[0]} (current: {VCVersion})");
					}
				}
			}

			// Check for newer WinSDK
			if (WinSDK is not null && WinSDKVersion is not null)
			{
				var dir = Path([WinSDK, "Include"], check_exists: false);
				if (Directory.Exists(dir))
				{
					var sdk_versions = Directory.EnumerateDirectories(dir)
						.Select(d => System.IO.Path.GetFileName(d))
						.Where(name => !string.IsNullOrEmpty(name) && name.StartsWith("10."))
						.OrderByDescending(name => name)
						.ToArray();
					
					// Check the latest version is being used
					if (sdk_versions.Length != 0)
					{
						if (string.Compare(sdk_versions[0], WinSDKVersion, StringComparison.OrdinalIgnoreCase) > 0)
							Console.WriteLine($"Newer Windows SDK version available: {sdk_versions[0]} (current: {WinSDKVersion})");
					}
				}
			}
		}
		catch (Exception ex)
		{
			Debug.WriteLine($"Error loading user variables: {ex.Message}");
			Debugger.Break();
			System.Environment.Exit(1);
		}
	}
}
