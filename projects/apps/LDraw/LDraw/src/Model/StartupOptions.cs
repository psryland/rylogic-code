using System;
using System.Collections.Generic;
using System.IO;
using Rylogic.Common;
using Rylogic.Utility;

namespace LDraw
{
	public class StartupOptions
	{
		// Notes:
		//  - Parsed command line options. See 'CmdLine' for the options that are supported.

		public StartupOptions(string[] args)
		{
			FilesToLoad = new List<string>();
			SettingsPath = null!;

			var exe_dir = Util.ResolveAppPath();
			if (!Path_.DirExists(exe_dir))
				throw new ArgumentException("Cannot determine the current executable directory");

			// Check the command line options
			for (int i = 0, iend = args.Length; i != iend; ++i)
			{
				var arg = args[i];
				var opt = arg.ToLowerInvariant();

				// No option character implies the file to load
				if (opt[0] != '-' && opt[0] != '/')
				{
					FilesToLoad.Add(arg);
					continue;
				}

				// Helper for comparing option strings
				bool IsOption(string option) => string.CompareOrdinal(opt, 0, option, 0, option.Length) == 0;

				// (order these by longest option first)
				if      (IsOption(CmdLine.McpLaunchNonce))
				{
					// '--mcp-launch-nonce=<guid>': the host passes this when it auto-launches LDraw. The value is the
					// nonce echoed back in the registration; its presence also forces MCP control on for this process.
					var eq = arg.IndexOf('=');
					McpLaunchNonce = eq >= 0 ? arg.Substring(eq + 1) : string.Empty;
				}
				else if (IsOption(CmdLine.SettingsPath ))
				{
					SettingsPath = arg.Length != CmdLine.SettingsPath.Length
						? arg.Substring(CmdLine.SettingsPath.Length)
						: i + 1 != iend ? args[++i] : throw new ArgumentException("Missing settings filepath for '-s'.");
				}
				else if (IsOption(CmdLine.Portable     )) { PortableMode = true; }
				else if (IsOption(CmdLine.ShowHelp     )) { ShowHelp = true; }
				else if (IsOption(CmdLine.ShowHelp2    )) { ShowHelp = true; }
				else throw new ArgumentException($"Unknown command line option '{arg}'.");
			}

			// Determine whether to run the app in portable mode
			PortableMode |= Path_.FileExists(Path_.CombinePath(exe_dir, "portable"));

			// Set the UserDataDir based on whether we're running in portable mode or not
			UserDataDir = Path.GetFullPath(PortableMode ? Path_.CombinePath(exe_dir, "UserData") : Util.ResolveAppDataPath("Rylogic", "LDraw"));
			Path_.CreateDirs(UserDataDir);

			// Check that we have write access to the user data directory
			if (!Path_.DirExists(UserDataDir) || (new DirectoryInfo(UserDataDir).Attributes & FileAttributes.ReadOnly) == FileAttributes.ReadOnly)
				throw new IOException($"The user data directory ('{UserDataDir}') is readonly.");

			// If a settings path has not been given, use the defaults
			SettingsPath ??= Path_.CombinePath(UserDataDir, "settings.xml");
		}

		/// <summary>The filepath to a file given on the command line</summary>
		public List<string> FilesToLoad { get; }

		/// <summary>True if we should run the app in portable mode</summary>
		public bool PortableMode { get; }

		/// <summary>A location that the app should read settings from and write to</summary>
		public string UserDataDir { get; }

		/// <summary>The filepath to the settings file to use</summary>
		public string SettingsPath { get; set; }

		/// <summary>
		/// The launch nonce supplied by the MCP host when it auto-launched this process, or empty.
		/// A non-empty value forces MCP control on regardless of the persisted AllowControl setting and is
		/// echoed back in the instance registration so the host can match the process it launched.</summary>
		public string McpLaunchNonce { get; set; } = string.Empty;

		/// <summary>True if the command line help options should be displayed</summary>
		public bool ShowHelp { get; }
	}
}
