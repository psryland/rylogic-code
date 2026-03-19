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
		//  - Parsed command line options

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
				var arg = args[i].ToLowerInvariant();

				// No option character implies the file to load
				if (arg[0] != '-' && arg[0] != '/')
				{
					FilesToLoad.Add(arg);
					continue;
				}

				// Helper for comparing option strings
				bool IsOption(string opt) => string.CompareOrdinal(arg, 0, opt, 0, opt.Length) == 0;

				// (order these by longest option first)
				if      (IsOption(CmdLine.SettingsPath )) { SettingsPath = arg.Substring(CmdLine.SettingsPath.Length); }
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

		/// <summary>True if the command line help options should be displayed</summary>
		public bool ShowHelp { get; }
	}
}
