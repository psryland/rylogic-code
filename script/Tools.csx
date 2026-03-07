#! "net9.0"
#r "System.Text.Json"
#r "nuget: Rylogic.Core, 2.0.2"
#r "System.Diagnostics.Process"
#load "UserVars.csx"
#nullable enable

using System;
using System.IO;
using System.Linq;
using System.Diagnostics;
using System.Reflection;
using System.Reflection.Metadata;
using System.Reflection.PortableExecutable;
using System.Runtime.InteropServices;
using System.Runtime.Versioning;
using System.Text.RegularExpressions;
using Rylogic.Extn;
using Console = System.Console;
using IOPath = System.IO.Path;

public class Tools
{
	/// <summary>Path helper</summary>
	public static string Path(IEnumerable<string> path_parts, bool check_exists = true, bool normalise = true)
	{
		return UserVars.Path(path_parts, check_exists, normalise);
	}

	// Ensure the directory 'dir' exists and is empty
	public static void CleanDir(string dir)
	{
		Console.WriteLine($"Cleaning deploy directory: {dir}");
		Directory.Delete(dir, true);
		Directory.CreateDirectory(dir);
	}

	// Deploy lib and/or dll files to the '/lib' folder
	public static void DeployLib(string target_path)
	{
		// 'Target' is the build target (i.e. in the obj/ directory)
		var m = Regex.Match(target_path, @"^(?<objdir>.*)[\\/](?<platform>.*?)[\\/](?<config>.*?)[\\/](?<target_name>.*?)$");
		if (!m.Success)
			throw new Exception($"Invalid $(TargetPath): {target_path}");

		var obj_dir = m.Groups["objdir"].Value;
		var platform = m.Groups["platform"].Value;
		var config = m.Groups["config"].Value;
		var target_name = IOPath.GetFileNameWithoutExtension(m.Groups["target_name"].Value);
		var target_dir = Tools.Path([obj_dir, platform, config]);

		// Get the destination directory: /lib/p/c/target_name.extn
		var dst_dir = Tools.Path([UserVars.Root, "lib", platform, config], check_exists: false);
		Directory.CreateDirectory(dst_dir);
	
		// Notes:
		//  - Watch out for pdb files overwriting projects with the same name.
		//  - The MSBuild system creates the '$(TargetName).pdb' file even if the project file sets the pdb name to something else.
		//  - Debugging will only load '$(TargetName).pdb' so trying to use '$(TargetName)$(TargetExt).pdb' doesn't work (sadly).
		//  - The only option is to use separate names for lib and dll projects :(
		//  - Use <project>-static.lib.
		//  - Don't make $(TargetName) == $(ProjectName), just set the name explicitly.
		var target_files = (string[])[
			Tools.Path([target_dir, $"{target_name}.lib"], check_exists: false),
			Tools.Path([target_dir, $"{target_name}.dll"], check_exists: false),
			Tools.Path([target_dir, $"{target_name}.imp"], check_exists: false),
			Tools.Path([target_dir, $"{target_name}.pdb"], check_exists: false),
		];

		// Copy the target files to the destination directories
		foreach (var filepath in target_files)
		{
			if (!IOPath.Exists(filepath)) continue;
			var dst_path = Tools.Path([dst_dir, IOPath.GetFileName(filepath)], check_exists: false);
			Tools.Copy(filepath, dst_path);
		}
	}

	//Executes a program and returns it's stdout/stderr as a string
	// 'checked' will raise an exception if the program returns a non-zero exit code
	// 'return_output' will return the output of the program as a string. If false, the output is printed to stdout
	// 'expected_return_code' is the expected return code. If the program returns a different code, an exception is raised
	// Returns (result,output)
	public static (bool Result, string Output) Run(IList<string> args, bool throw_on_error = true, bool return_output = true, int expected_return_code = 0, bool show_arguments = false, string? cwd = null, bool shell = false, Encoding? encoding = null)
	{
		if (args.Count == 0)
			throw new ArgumentException("args must contain at least one element (the program to run)");
		if (show_arguments)
			Console.WriteLine(string.Join(" ", args));

		try
		{
			var filename = args[0];
			var arguments = string.Join(" ", args.Skip(shell ? 0 : 1).Select(x => x.Contains(" ") ? $"\"{x}\"" : x));
			var psi = new ProcessStartInfo
			{
				FileName = filename,
				Arguments = arguments,
				RedirectStandardOutput = return_output,
				RedirectStandardError = return_output,
				UseShellExecute = shell, // Must be false to redirect output
				CreateNoWindow = false, // Show output in current console window
				WorkingDirectory = cwd ?? string.Empty,
			};
			if (return_output)
			{
				psi.StandardOutputEncoding = encoding ?? Encoding.UTF8;
				psi.StandardErrorEncoding = encoding ?? Encoding.UTF8;
			}

			using var proc = new Process { StartInfo = psi };
			proc.Start();

			var output = return_output ? proc.StandardOutput.ReadToEnd() + proc.StandardError.ReadToEnd() : "";
			proc.WaitForExit();

			var success = proc.ExitCode == expected_return_code;
			if (!success && throw_on_error)
				throw new Exception($"Process returned {proc.ExitCode}: {output}");

			return (success, output);
		}
		catch (Exception ex)
		{
			if (throw_on_error) throw;
			return (false, ex.Message);
		}
	}

	// Run a program in a separate console window
	// Returns the process for the caller to call wait() on,
	//  e.g.
	//    proc = Spawn(["cmd", "/C" ,"echo Hello"])
	//    proc.wait()
	public static Process Spawn(string prefix, IList<string> args, int expected_return_code = 0, bool same_window = false, bool show_window = true, bool show_arguments = false)
	{
		if (args.Count == 0)
			throw new ArgumentException("args must contain at least one element (the program to run)");
		if (show_arguments)
			Console.WriteLine(string.Join(" ", args));

		var filename = args[0];
		var arguments = string.Join(" ", args.Skip(1).Select(x => x.Contains(" ") ? $"\"{x}\"" : x));
		var psi = new ProcessStartInfo
		{
			FileName = filename,
			Arguments = arguments,
			RedirectStandardOutput = true,
			RedirectStandardError = true,
			UseShellExecute = false, // Required to open a new console
			CreateNoWindow = !show_window,
			WindowStyle = show_window ? ProcessWindowStyle.Normal : ProcessWindowStyle.Hidden,
		};

		var process = new Process { StartInfo = psi, EnableRaisingEvents = true };
		process.OutputDataReceived += (s, e) =>
		{
			if (e.Data == null) return;
			System.Console.WriteLine($"{prefix}{e.Data}");
		};
		process.ErrorDataReceived += (s, e) =>
		{
			if (e.Data == null) return;
			System.Console.Error.WriteLine($"{prefix}{e.Data}");
		};

		if (!process.Start())
			throw new Exception("Failed to start process");

		process.BeginOutputReadLine();
		process.BeginErrorReadLine();
		return process;
	}

	// Find the visual studio batch file for setting up a dev environment
	public static void SetupVcEnvironment()
	{
		// Already set up
		if (m_vc_env_setup)
			return;

		// Not a windows environment
		if (!RuntimeInformation.IsOSPlatform(OSPlatform.Windows))
			return;

		Console.WriteLine("Initializing VS Environment");

		// Check for newer versions
		{
			var msvc_version = Path([UserVars.VSDir, $"VC\\Tools\\MSVC"]);
			var available_versions = Directory.GetDirectories(msvc_version).Select(IOPath.GetFileName).OrderByDescending(x => x).ToList();
			if (available_versions.Count == 0 || !available_versions.Contains(UserVars.VCVersion))
				throw new Exception($"\n *** VC Version not Found : Latest = {available_versions[0]} *** \n");
			if (UserVars.VCVersion.CompareTo(available_versions[0]) < 0)
				Console.WriteLine($"\n *** Newer VC Version Available : Latest = {available_versions[0]} *** \n");
		}

		///* Just need this?
		{
			var path = Environment.GetEnvironmentVariable("PATH");
			var msbuild_tools = Path([UserVars.VSDir, "MSBuild\\Current\\Bin"]);
			var vc_tools = Path([UserVars.VSDir, $"VC\\Tools\\MSVC\\{UserVars.VCVersion}\\bin\\Hostx64\\x64"]);
			Environment.SetEnvironmentVariable("PATH", $"{msbuild_tools};{vc_tools};{path}");
		}
		//*/

		/* Previous method
		{
			// Find 'vswhere.exe' from the VS installer
			var program_files = Environment.GetEnvironmentVariable("ProgramFiles(x86)") ?? throw new Exception("ProgramFiles(x86) environment variable is not defined");
			var vswhere = Path([program_files, "Microsoft Visual Studio", "Installer", "vswhere.exe"]);

			// Get the VS install path
			var (_, vs_path) = Run([vswhere, "-latest", "-property", "installationPath"]);
			vs_path = vs_path.TrimEnd();

			// Get the VcVars batch file
			var vsvars_path = Path([vs_path, "VC", "Auxiliary", "Build", "vcvars64.bat"]);

			// Run the vcvars batch file, then dump the state of the environment variables
			var (_, env) = Run(["cmd", "/C", vsvars_path, "&", "set"]);

			// Add/Replace the environment variables in 'os.environ'
			foreach (var line in env.Split("\n", StringSplitOptions.RemoveEmptyEntries))
			{
				var m = Regex.Match(line, "^(.+?)=(.*)$");
				if (!m.Success)
					continue;

				var key = m.Groups[1].Value.Trim();
				var value = m.Groups[2].Value.Trim();
				Environment.SetEnvironmentVariable(key, value);
			}
		}
		//*/
		m_vc_env_setup = true;
	}
	private static bool m_vc_env_setup = false;

	// Invoke MSBuild on a solution or project file.
	// Solution file usage:
	//   sln_or_proj_file = "C:\path\mysolution.sln"
	//	projects = ["project_name","\"folder\proj_name:Rebuild\""]
	//	platforms = ["x64","x86","Any CPU"]
	//	configs = ["release","debug"]
	//	Tools.MSBuild(sln_or_proj_file, projects, platforms, configs, True, True)
	// Project file usage:
	//   sln_or_proj_file = "C:\path\myproject.csproj"
	//	projects = []
	//	platforms = ["x64","x86","AnyCPU"]
	//	configs = ["release","debug"]
	//	Tools.MSBuild(sln_or_proj_file, projects, platforms, configs, True, True)
	public static bool MSBuild(string sln_or_proj_file, IList<string>? projects = null, IList<string>? platforms = null, IList<string>? configs = null, bool parallel = true, bool same_window = true, IList<string>? additional_args = null)
	{
		SetupVcEnvironment();

		if (UserVars.MSBuild is null)
			throw new Exception("MSBuild path has not been set in UserVars");

		// Handle default options
		projects ??= [];
		platforms ??= [];
		configs ??= [];
		additional_args ??= [];

		// Build the arguments list
		List<string> args = [UserVars.MSBuild, sln_or_proj_file, "/m", "/verbosity:minimal", "/nologo", ..additional_args];

		// Set the targets to build
		// Targets should be the names as shown in the solution explorer (i.e. Folder\Project.Name)
		if (projects.Count != 0)
		{
			// Replace '.' in the project name with '_'
			projects = [..projects.Select(x => x.Replace(".", "_")), ];
			args.Add($"/t:{string.Join(';', projects)}");
		}

		// Set the platform/config
		List<Process> procs = [];
		bool errors = false;
		try
		{
			if (platforms.Count == 0 && configs.Count == 0)
			{
				Run(args);
			}
			else
			{
				int i = 0;
				foreach (var platform in platforms)
				{
					foreach (var config in configs)
					{
						List<string> args_ = [..args, $"/p:Configuration={config};Platform={platform}"];
						if (parallel)
						{
							var instance_id = ++i;
							Console.WriteLine($"{instance_id}> --- {string.Join(",", projects)} --- {platform}|{config} ---");
							procs.Add(Spawn($"{instance_id}>", args_, same_window: same_window));
						}
						else
						{
							Console.WriteLine($"--- {string.Join(",", projects)} --- {platform}|{config} ---");
							Run(args_, return_output: false, show_arguments: false);
						}
					}
				}
			}
		}
		catch (Exception ex)
		{
			Console.WriteLine($"Build Errors: {ex.Message}");
			errors = true;
		}
		finally
		{
			// Wait for all processes to finish, and check for error return codes
			foreach (var proc in procs)
			{
				proc.WaitForExit();
				errors |= proc.ExitCode != 0;
			}
		}

		return !errors;
	}

	// Sign an assembly
	public static void SignAssembly(string target)
	{
		if (UserVars.CodeSignCert_Pfx.Length == 0)
			return;

		Run([UserVars.SignTool, "sign", "/f", UserVars.CodeSignCert_Pfx, "/p", UserVars.CodeSignCert_Pw, "/fd", "SHA1", target]);
	}

	// Sign a VSIX extension package
	public static void SignVsix(string vsix_filepath, string algo)
	{
		// Use 'dotnet tool install -g OpenVsixSignTool' to install 'OpenVsixSignTool'
		// 'OpenVsixSignTool' is an open source (and better) version of 'VsixSignTool' (https://github.com/vcsjones/OpenOpcSignTool)
		// 'e1053e6fa1aeb7bd4ee453302116a129ca4112f9' is the thumbprint of the code signing certificate installed on the machine.
		// Run 'mmc' then add 'Certificates' to the console. Find your code signing cert (Rylogic Limited, Sectigo RSA Code Signing CA),
		// and open it. Under 'details' find 'Thumbprint'. The OpenVsixSignTool uses this hash value to find the cert in the store.
		// If the vsix supports VS versions less than 14.0, you need to use "-fd sha1" or the cert shows up as invalid. If the VSIX only
		// supports VS versions >= 14.0, use sha256 instead.
		Run(["openvsixsigntool", "sign", "--sha1", UserVars.CodeSignCert_Thumbprint, "-fd", algo, vsix_filepath]);
	}

	// Run the units tests in a .net assembly
	public static void UnitTest(string binary_filepath, bool managed, List<string>? deps = null, bool run_tests = true)
	{
		// Set this to false to disable running tests on compiling
		var RunTests = run_tests;
		//RunTests = false;
		if (!RunTests)
		{
			Console.WriteLine($"{binary_filepath}. **** Unit tests disabled ****");
			return;
		}

		if (!IOPath.Exists(binary_filepath))
		{
			Console.WriteLine($"{binary_filepath} assembly not found.   **** Unit tests skipped ****");
			return;
		}

		// Run unit tests in a managed assembly
		if (managed)
		{
			if (binary_filepath.EndsWith(".exe"))
			{
				var (res,outp) = Run([binary_filepath], throw_on_error: false, return_output: true);
				Console.WriteLine(outp);
				if (!res) throw new Exception("   **** Unit tests failed ****   ");
				return;
			}
			if (binary_filepath.EndsWith(".dll"))
			{
				var command =
					$"& {{\n" + 
					$"    Set-Location {UserVars.Root};\n" +
					$"    Add-Type -AssemblyName '{binary_filepath}';\n" + 
					$"    $result = [{IOPath.GetFileNameWithoutExtension(binary_filepath)}.Program]::Main();\n" + 
					$"    Exit $result;\n" + 
					$"}}";
				var (res,outp) = Run([UserVars.Pwsh, "-NonInteractive", "-NoProfile", "-NoLogo", "-Command", command]);
				Console.WriteLine(outp);
				if (!res) throw new Exception("   **** Unit tests failed ****   ");
				return;

				// If the assembly is '.NETCoreApp' then the assembly can actually be loaded in this process.
				#if false
				{
					// Load the assembly and call Program.Main()
					var ass = Assembly.LoadFile(binary_filepath);
					var ns = IOPath.GetFileNameWithoutExtension(binary_filepath);
					var type_program = ass.GetType($"{ns}.Program") ?? throw new Exception($"Type '{ns}.Program' not found in assembly '{binary_filepath}'");
					var mi_main = type_program.GetMethod("Main", BindingFlags.Public | BindingFlags.Static) ?? throw new Exception($"Program.Main method not found in assembly '{binary_filepath}'");
					var result = mi_main.Invoke(null, null);
					if (result is not int res || res != 0)
						throw new Exception("   **** Unit tests failed ****   ");
					return;
				}
				#endif
			}
		}
		else
		{
			if (binary_filepath.EndsWith(".exe"))
			{
				var (res,outp) = Run([binary_filepath], throw_on_error: false, return_output: true);
				Console.WriteLine(outp);
				if (!res) throw new Exception("   **** Unit tests failed ****   ");
				return;
			}
		}
	}

	// Extract data from a text file using a regex
	// Capture groups are defined like: (?P<name>.*) and accessed like: m.group("name")
	// Returns the regex match object for the first match or null
	// Encoding can be: ascii, utf-8, cp1250, etc
	public static Match Extract(string filepath, Regex pat, Encoding? encoding = null, bool by_line = true)
	{
		if (by_line)
		{
			foreach (var line in File.ReadAllLines(filepath, encoding: encoding ?? Encoding.UTF8))
			{
				var m = pat.Match(line);
				if (m.Success)
					return m;
			}
		}
		else
		{
			var s = File.ReadAllText(filepath);
			var m = pat.Match(s);
			if (m.Success)
				return m;
		}

		return Match.Empty;
	}

	// Extract data from a text file using a regex
	// Capture groups are defined like: (?P<name>.*) and accessed like: m.group("name")
	// Returns a collection of matches from within the file
	// Encoding can be: ascii, utf-8, cp1250, etc
	public static IEnumerable<Match> ExtractMany(string filepath, Regex pat, Encoding? encoding = null)
	{
		foreach (var line in File.ReadAllLines(filepath, encoding: encoding ?? Encoding.UTF8))
		{
			var m = pat.Match(line);
			if (m.Success)
				yield return m;
		}
	}

	// Modify a file, line-by-line, using regex
	// Capture groups are defined like: (?<name>.*) and accessed like: m.Groups["name"].Value
	public static void UpdateFileByLine(string filepath, Regex pat, string repl, bool all = false)
	{
		var do_replace = true;
		var text = File.ReadAllText(filepath);
		var updated = new StringBuilder(text.Length);
		foreach (var line in text.Split('\n'))
		{
			if (do_replace)
			{
				var m = pat.Match(line);
				if (m.Success)
				{
					updated.Append(pat.Replace(line, repl)).Append('\n');
					do_replace &= all;
					continue;
				}
			}

			updated.Append(line).Append('\n');
		}
		var newt = updated.ToString();
		if (text != newt)
		{
			File.WriteAllText(filepath + ".tmp", newt);
			File.Replace(filepath + ".tmp", filepath, null);
		}
	}

	// Modify a whole file using regex.
	public static void UpdateFile(string filepath, Regex pat, string repl)
	{
		var text = File.ReadAllText(filepath);
		var newt = pat.Replace(text, repl);
		if (text != newt)
		{
			File.WriteAllText(filepath + ".tmp", newt);
			File.Replace(filepath + ".tmp", filepath, null);
		}
	}

	// Replace a tagged section within a file
	public static void UpdateTaggedSection(string filepath, string tag_beg, string tag_end, string repl)
	{
		// Check the file exists
		if (!IOPath.Exists(filepath))
			throw new FileNotFoundException($"Cannot update tagged section in '{filepath}'.", filepath);

		// Create the string to replace with
		var section = $"{tag_beg}{repl}{tag_end}";

		// The tagged section pattern
		var pat = new Regex($"{tag_beg}(.*?){tag_end}", RegexOptions.Singleline);

		// Replace the tagged section in 'filepath'
		UpdateFile(filepath, pat, section);
	}

	// Copy 'src' to 'dst' optionally if 'src' is newer than 'dst'
	public static void Copy(string src, string dst, bool only_if_modified = true, bool show_unchanged = false, bool ignore_missing = false, bool quiet = false, bool full_paths = true, Regex? filter = null, bool follow_symlinks = true, string? indent = null)
	{
		bool src_is_dir = src.EndsWith("/") || src.EndsWith("\\");
		bool dst_is_dir = dst.EndsWith("/") || dst.EndsWith("\\");
		src = Path([src], check_exists: true);
		dst = Path([dst], check_exists: false);
		src_is_dir |= Directory.Exists(src);
		dst_is_dir |= Directory.Exists(dst) || src_is_dir;

		indent ??= string.Empty;

		// Find the source files/directories to copy (filenames only, not full paths)
		List<string> lst = [];
		if (src_is_dir)
		{
			var fnames = Directory.GetFileSystemEntries(src).Select(IOPath.GetFileName).Where(x => x is not null).Select(x => x!).ToList();
			lst.AddRange(fnames);
		}
		else if (File.Exists(src))
		{
			lst.Add(IOPath.GetFileName(src));
		}
		else if (src.Contains("*") || src.Contains("?"))
		{
			var dir = IOPath.GetDirectoryName(src) ?? Directory.GetCurrentDirectory();
			var pattern = IOPath.GetFileName(src);
			var fnames = Directory.GetFiles(dir, pattern).Select(IOPath.GetFileName).Where(x => x is not null).Select(x => x!).ToList();
			lst.AddRange(fnames);
		}
		else if (!ignore_missing)
		{
			throw new FileNotFoundException($"ERROR: {src} does not exist");
		}

		// If the 'src' represents multiple files, 'dst' must be a directory
		if (src_is_dir || lst.Count > 1)
		{
			if (!Directory.Exists(dst))
			{
				dst_is_dir = true;
			}
			else if (!dst_is_dir)
			{
				throw new FileNotFoundException($"ERROR: {dst} is not a valid directory");
			}
		}

		var srcdir = (src_is_dir ? src : IOPath.GetDirectoryName(src) ?? string.Empty).TrimEnd('/', '\\');
		var dstdir = (dst_is_dir ? dst : IOPath.GetDirectoryName(dst) ?? string.Empty).TrimEnd('/', '\\');

		// Ensure 'dstdir' exists
		if (!Directory.Exists(dstdir))
		{
			Directory.CreateDirectory(dstdir);

			// Copy directory attributes if needed
			if (src_is_dir && Directory.Exists(srcdir))
			{
				var srcInfo = new DirectoryInfo(srcdir);
				var dstInfo = new DirectoryInfo(dstdir);
				dstInfo.Attributes = srcInfo.Attributes;
			}

			if (!quiet)
			{
				if (full_paths)
					Console.WriteLine($"{indent}{srcdir}\\ --> {dstdir}\\");
				else
					Console.WriteLine($"{indent}{IOPath.GetFileName(srcdir)}\\ --> {IOPath.GetFileName(dstdir)}\\");
			}
				;
		}

		// Copy the source files/directories to 'dst'
		foreach (var src_item in lst)
		{
			var s = IOPath.Combine(srcdir, src_item);
			var d = dst_is_dir ? IOPath.Combine(dstdir, src_item) : dst;

			// Test for symlinks...
			if (File.Exists(s) && new FileInfo(s).Attributes.HasFlag(FileAttributes.ReparsePoint))
				throw new Exception("ERROR: This copy function doesn't consider symlinks yet. It's a todo...");

			// Call recursively for directory copies
			if (Directory.Exists(s))
			{
				Copy(s, d + "\\", only_if_modified, show_unchanged, ignore_missing, quiet, full_paths, filter, follow_symlinks, indent + indent);
			}
			else
			{
				// Copy if not excluded by the filter
				if (filter != null && !filter.IsMatch(s))
					continue;

				// Copy if modified or always based on the flag
				if (only_if_modified && !DiffContent(s, d))
				{
					if (!quiet && show_unchanged)
						Console.WriteLine($"{indent}{s} --> unchanged");

					continue;
				}

				if (!quiet)
				{
					if (full_paths)
						Console.WriteLine($"{indent}{s} --> {d}");
					else
						Console.WriteLine($"{indent}{IOPath.GetFileName(s)} --> {IOPath.GetFileName(d)}");
				}
				File.Copy(s, d, true);
			}
		}
	}

	// Compare the content of two files and return true if they are different, ignoring file timestamps
	public static bool DiffContent(string src, string dst, bool trace = false, int blocksize = 65536)
	{
		var sfound = File.Exists(src);
		var dfound = File.Exists(dst);

		if (!sfound)
		{
			if (trace) Console.WriteLine($"Content different, '{src}' not found");
			return true;
		}
		if (!dfound)
		{
			if (trace) Console.WriteLine($"Content different, '{dst}' not found");
			return true;
		}

		var src_info = new FileInfo(src);
		var dst_info = new FileInfo(dst);

		if (src_info.Length != dst_info.Length)
		{
			if (trace) Console.WriteLine($"Content different, '{src}' and '{dst}' have different sizes");
			return true;
		}

		using (var sha256 = System.Security.Cryptography.SHA256.Create())
		{
			byte[] srcHash, dstHash;
			using (var s = new FileStream(src, FileMode.Open, FileAccess.Read))
				srcHash = sha256.ComputeHash(s);
			using (var d = new FileStream(dst, FileMode.Open, FileAccess.Read))
				dstHash = sha256.ComputeHash(d);

			if (!srcHash.SequenceEqual(dstHash))
			{
				if (trace) Console.WriteLine($"Content different, '{src}' and '{dst}' have different hashes");
				return true;
			}
		}
		return false;
	}
}

// Allow methods within Tools to be invoked from the command line
//var args = Environment.GetCommandLineArgs().Skip(1).ToList();
//if (args.Count > 1 && args[0].EndsWith("Tools.csx") && typeof(Tools).GetMethod(args[1]) is MethodInfo mi)
//{
//	args = args[2..];
//
//	// Correct number of parameters given?
//	var pi = mi.GetParameters();
//	if (pi.Length != args.Count)
//		return;
//
//	// Convert strings to paramater types
//	var parameters = (List<object?>)[];
//	for (int i = 0; i != pi.Length; ++i)
//		parameters.Add(Convert.ChangeType(args[i], pi[i].ParameterType));
//
//	mi.Invoke(null, parameters.ToArray());
//}
