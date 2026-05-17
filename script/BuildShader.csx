#! "net10.0"
// Build shaders using dxc.exe
// Use:
//  dotnet-script.exe BuildShader.csx $(Fullpath) $(PlatformTarget) $(Configuration) [vs=<entry-points>] [ps=<entry-points>] [gs=<entry-points>] [cs=<entry-points>] [lib=<targets>] [obj] [dbg] [trace] [pp]
//
// Expected input is an hlsl file. Explicit entry points are supplied per shader stage.
// Entry point specs are semicolon separated, with either:
//  EntryPoint
//  output_symbol=EntryPoint
//
// Library targets are semicolon separated output symbols, with optional explicit profiles:
//  output_symbol
//  output_symbol=lib_6_6
//
// Add 'pp' to the command line for preprocessed output
// Add 'obj' to the command line for a 'compiled shader object' file
//  that can be used with the runtime shader support in the renderer.
#r "System.IO"
#r "System.Text.Json"
#r "nuget: Rylogic.Core, 2.1.1"
#load "UserVars.csx"
#nullable enable

using System.Diagnostics;
using System.IO;
using System.Text;
using System.Security.Cryptography;
using Rylogic.Common;

public class ShaderBuilder
{
	private class ShaderDesc
	{
		public string Name = string.Empty;
		public string FullPath = string.Empty;
		public string Profile = string.Empty;
		public string ShaderCode = string.Empty;
		public string Define = string.Empty;
		public string EntryPoint = string.Empty;
	}

	private string m_compiler;
	private bool m_pp;
	private bool m_obj;
	private bool m_trace;
	private bool m_dbg;

	public ShaderBuilder(bool pp = false, bool obj = false, bool trace = false, bool dbg = false)
	{
		// Get the full path to the compiler
		//m_compiler = UserVars.Path([UserVars.WinSDK, "bin", UserVars.WinSDKVersion, "x64", "fxc.exe"]); // old compiler < SM 6
		m_compiler = UserVars.Path([UserVars.WinSDK, "bin", UserVars.WinSDKVersion, "x64", "dxc.exe"]); // new compiler >= SM 6

		// Enable compiled shader objects in debug, for debugging and runtime shaders
		m_pp = pp;
		m_obj = obj | dbg;
		m_trace = trace;
		m_dbg = dbg;

		// Show the command line options
		Trace($"trace:{trace} debug:{dbg} obj:{obj} pp:{pp}");
	}

	// Build an HLSL shader
	// 'fullpath' - full path to the HLSL file
	// 'platform' - one of x86 or x64
	// 'config' - one of debug or release
	// Optional parameters:
	//  pp - produced pre-processed
	//  obj - produce compiled shader object files (automatically enabled in debug)
	//  trace - print debugging messages
	//  dbg - debugging
	public void Compile(string fullpath, string platform, string config, string shader_model, string vertex_entry_points, string pixel_entry_points, string geometry_entry_points, string compute_entry_points, string library_targets)
	{
		// Find the source and output directories
		fullpath = UserVars.Path([fullpath]);
		var fdir = Path.GetDirectoryName(fullpath) ?? throw new InvalidOperationException("Failed to get directory name");
		var fname = Path.GetFileName(fullpath) ?? throw new InvalidOperationException("Failed to get file name");
		var ftitle = Path.GetFileNameWithoutExtension(fullpath) ?? throw new InvalidOperationException("Failed to get file title");
		var extn  = Path.GetExtension(fullpath) ?? throw new InvalidOperationException("Failed to get file extension");
		Trace($"File: {fdir}/{ftitle}{extn}");

		// Determine the output directory
		var outdir = OutputRoot(fdir);
		outdir = UserVars.Path([outdir, "compiled", config], check_exists: false);
		Trace($"Output directory: {outdir}");
		Directory.CreateDirectory(outdir);

		var stamp = UserVars.Path([outdir, $"{ftitle}.built"], check_exists: false);
		var profile_model = shader_model.Replace(".", "_");

		// Careful with shader versions, if you bump up from 4_0 you'll need to change the minimum feature level in view3d.
		var shaders = (List<ShaderDesc>)[];
		shaders.AddRange(ShaderEntries(fullpath, profile_model, "vs", vertex_entry_points));
		shaders.AddRange(ShaderEntries(fullpath, profile_model, "ps", pixel_entry_points));
		shaders.AddRange(ShaderEntries(fullpath, profile_model, "gs", geometry_entry_points));
		shaders.AddRange(ShaderEntries(fullpath, profile_model, "cs", compute_entry_points));
		shaders.AddRange(LibraryTargets(fullpath, profile_model, library_targets));
		if (shaders.Count == 0)
			throw new Exception($"No shader targets specified for {fullpath}");

		File.Delete(stamp);
		foreach (var shdr in shaders)
			BuildShader(shdr, outdir, platform, config);

		// Create the .built file, so that VS's custom build tool can check for it's existence to determine when a build is needed.
		File.Create(stamp).Dispose();
	}

	// Return the root directory that owns generated shader outputs.
	private string OutputRoot(string fdir)
	{
		var hlsl_dir = FindAncestor(fdir, "hlsl");
		if (hlsl_dir != null)
			return hlsl_dir;

		var compute_dir = FindAncestor(fdir, "compute");
		if (compute_dir != null)
			return compute_dir;

		throw new Exception($"Shader file {fdir} is not within an 'hlsl' or 'compute' directory");
	}

	// Return the nearest ancestor directory with the given directory name.
	private string? FindAncestor(string dir, string name)
	{
		for (var path = dir; path != string.Empty; )
		{
			var dname = Path.GetFileName(path);
			if (string.Equals(dname, name, StringComparison.OrdinalIgnoreCase))
				return path;

			path = Path.GetDirectoryName(path) ?? string.Empty;
		}
		return null;
	}

	// Return shader entries for one shader stage.
	private IEnumerable<ShaderDesc> ShaderEntries(string fullpath, string profile_model, string shader_code, string entry_points)
	{
		foreach (var spec in TargetSpecs(entry_points))
		{
			var (name, entry_point) = ParseEntrySpec(spec, shader_code);
			yield return new ShaderDesc
			{
				Name = name,
				FullPath = fullpath,
				Profile = $"/T{shader_code}_{profile_model}",
				ShaderCode = shader_code,
				EntryPoint = entry_point,
			};
		}
	}

	// Return library targets.
	private IEnumerable<ShaderDesc> LibraryTargets(string fullpath, string profile_model, string library_targets)
	{
		foreach (var spec in TargetSpecs(library_targets))
		{
			var parts = spec.Split('=', 2, StringSplitOptions.TrimEntries);
			var name = parts[0];
			var profile = parts.Length == 2 ? parts[1] : $"lib_{profile_model}";
			if (name.Length == 0)
				throw new Exception($"Invalid library target '{spec}'");
			if (!profile.StartsWith("lib_", StringComparison.OrdinalIgnoreCase))
				throw new Exception($"Invalid library profile '{profile}' for target '{name}'. Expected a profile such as 'lib_6_6'.");

			yield return new ShaderDesc
			{
				Name = name,
				FullPath = fullpath,
				Profile = $"/T{profile}",
				ShaderCode = "lib",
			};
		}
	}

	// Split a semicolon separated shader-target list.
	private IEnumerable<string> TargetSpecs(string targets)
	{
		return targets
			.Split([';'], StringSplitOptions.RemoveEmptyEntries | StringSplitOptions.TrimEntries)
			.Distinct();
	}

	// Parse one stage entry point spec.
	private (string name, string entry_point) ParseEntrySpec(string spec, string shader_code)
	{
		var parts = spec.Split('=', 2, StringSplitOptions.TrimEntries);
		var entry_point = parts.Length == 2 ? parts[1] : parts[0];
		var name = parts.Length == 2 ? parts[0] : $"{ShaderName(entry_point, shader_code)}_{shader_code}";
		if (name.Length == 0 || entry_point.Length == 0)
			throw new Exception($"Invalid {shader_code} entry point spec '{spec}'");

		return (name, entry_point);
	}

	// Convert an HLSL entry point name into the generated shader symbol base name.
	private string ShaderName(string entry_point, string shader_code)
	{
		var name = entry_point;
		if (shader_code == "cs" && name.StartsWith("CS") && name.Length > 2 && char.IsUpper(name[2]))
			name = name[2..];

		var outp = new StringBuilder();
		for (int i = 0; i != name.Length; ++i)
		{
			var ch = name[i];
			if (char.IsUpper(ch))
			{
				if (i != 0)
				{
					var prev = name[i - 1];
					var next = i + 1 != name.Length ? name[i + 1] : '\0';
					if (char.IsLower(prev) || char.IsDigit(prev) || (char.IsUpper(prev) && char.IsLower(next)))
						outp.Append('_');
				}
				outp.Append(char.ToLowerInvariant(ch));
			}
			else if (ch == '-' || ch == ' ')
			{
				outp.Append('_');
			}
			else
			{
				outp.Append(ch);
			}
		}
		return outp.ToString();
	}

	// Compile the file as 'shader_type.Shader'
	private void BuildShader(ShaderDesc shdr, string outdir, string platform, string config)
	{
		Console.WriteLine($"Shader: {shdr.Name}");

		// Hash the full path to generate a unique temporary file name
		var hash = HashName(shdr.FullPath);

		// Create temporary filepaths so that we only overwrite
		// existing files if they've actually changed. Create temp
		// files for each unique platform/config to allow parallel build

		var temp_dir = UserVars.Path([UserVars.Root, "obj", "shaders", platform, config], check_exists: false);
		var filepath_h = UserVars.Path([temp_dir, $"{shdr.Name}-{hash}.h"], check_exists: false);
		var filepath_cso = UserVars.Path([temp_dir, $"{shdr.Name}-{hash}.cso"], check_exists: false);
		var filepath_pdb = UserVars.Path([temp_dir, $"{shdr.Name}-{hash}.pdb"], check_exists: false);

		// Delete any potentially left over temporary output
		Directory.CreateDirectory(temp_dir);
		if (File.Exists(filepath_h))   File.Delete(filepath_h);
		if (File.Exists(filepath_cso)) File.Delete(filepath_cso);

		// Set up the command line for DXC
		var args = (List<string>)[m_compiler, shdr.FullPath, shdr.Profile];

		// Set the variable name to the name of the shader
		args.Add($"/Vn{shdr.Name}");
		Trace($"Variable Name: {args.Last()}");

		// Set the entry point when building named-entry compute shaders.
		if (shdr.EntryPoint.Length != 0)
			args.Add($"/E{shdr.EntryPoint}");

		// Choose the output files to generate
		args.Add($"/Fh{filepath_h}");
		Trace($"Output: {args.Last()}");
		if (m_obj) args.Add($"/Fo{filepath_cso}");

		// Set include paths
		var includes = (List<string>)[
			$"/I{UserVars.Path([UserVars.Root, "include"])}",
			$"/I{UserVars.Path([UserVars.Root, "projects/rylogic"])}",
		];
		args.AddRange(includes);

		// Set defines
		args.Add("/DSHADER_BUILD");
		if (shdr.Define.Length != 0)
			args.Add($"/D{shdr.Define}");

		// Set other command line options
		args.AddRange(["/nologo", "/Gis", "/Ges", "/WX", "/Zpc", "/HV", "2021"]);

		// Debug build options
		// For some reason, the /Zi option causes the output to be different each time it's built using fxc
		if (m_dbg)
			args.AddRange(["/Gfp", "/Od", "/Zi", $"/Fd{filepath_pdb}"]);

		if (!m_pp)
		{
			// Build the shader using DXC
			Trace($"Running {Path.GetFileName(m_compiler)}...");
			var output = Run(args);
			Trace(output);

			var out_filepath_h   = UserVars.Path([outdir, $"{shdr.Name}.h"], check_exists: false);
			var out_filepath_cso = UserVars.Path([outdir, $"{shdr.Name}.cso"], check_exists: false);
			var out_filepath_pdb = UserVars.Path([outdir, $"{shdr.Name}.pdb"], check_exists: false);

			// Copy to target directory
			ReplaceIfModified(filepath_h, out_filepath_h);
			ReplaceIfModified(filepath_cso, out_filepath_cso);
			ReplaceIfModified(filepath_pdb, out_filepath_pdb);
		}
		else // Generate preprocessed output
		{
			throw new NotImplementedException("Preprocessing is not implemented yet.");
			// Delete existing pp output
			//var filepath_pp = UserVars.Path([outdir, $"{shdr.Name}.pp"]);
			//if (File.Exists(filepath_pp)) File.Delete(filepath_pp);

			// Pre process and clean
			//Run([m_compiler, shdr.FullPath, $"/P{filepath_pp}"] + includes + defines + options)
			//Run([os.path.join(UserVars.root, "bin", "textformatter.exe"), "-f", filepath_pp, "-newlines", "0", "1"])
		}
	}

	// Execute a process and return the output
	private string Run(List<string> args)
	{
		var process = new Process
		{
			StartInfo = new ProcessStartInfo
			{
				FileName = args[0],
				Arguments = string.Join(" ", args.Skip(1)),
				RedirectStandardOutput = true,
				UseShellExecute = false,
				CreateNoWindow = true,
			}
		};
		process.Start();

		var output = process.StandardOutput.ReadToEnd();
		process.WaitForExit();
		if (process.ExitCode != 0)
			throw new Exception($"Process failed with exit code {process.ExitCode}");

		return output;
	}

	// Replace 'dst' with 'src' if 'src' has been modified
	private void ReplaceIfModified(string src, string dst)
	{
		if (!File.Exists(src))
		{
			return;
		}
		if (!File.Exists(dst))
		{
			File.Move(src, dst);
			return;
		}
		else
		{
			var src_hash = CalculateFileHash(src);
			var dst_hash = CalculateFileHash(dst);
			if (src_hash == dst_hash)
				return;
		}
		try
		{
			File.Copy(src, dst, true);
			File.Delete(src);
		}
		catch (Exception ex)
		{
			throw new IOException($"Failed to replace {dst} with {src}: {ex.Message}", ex);
		}
	}

	// Trace output to the console
	private void Trace(string message)
	{
		if (!m_trace) return;
		Console.WriteLine(message);
	}

	// Compute MD5 hash as hex string
	private string HashName(string name)
	{
		using var md5 = MD5.Create();
		var input_bytes = Encoding.UTF8.GetBytes(name);
		var hash_bytes = md5.ComputeHash(input_bytes);
		return BitConverter.ToString(hash_bytes).Replace("-", "").ToLowerInvariant();
	}

	// Compute the hash of a file's contents
	private string CalculateFileHash(string filepath)
	{
		using var stream = File.OpenRead(filepath);
		using var md5 = MD5.Create();
		var bytes = md5.ComputeHash(stream);
		return BitConverter.ToString(bytes).Replace("-", "").ToLowerInvariant();
	}
}

var args =
	Args.ToArray();
	//(string[])[@"E:\Rylogic\Code\projects\rylogic\view3d-12\src\shaders\hlsl\forward\forward.hlsl", "x64", "debug", "vs=forward_vs=VSForward", "ps=forward_ps=PSForward", "obj", "dbg", "trace"];

if (args.Length < 3)
{
	Console.WriteLine("Usage: BuildShader <fullpath> <platform> <config> [model=<shader-model>] [vs=<entry-points>] [ps=<entry-points>] [gs=<entry-points>] [cs=<entry-points>] [lib=<targets>] [pp] [obj] [dbg] [trace]");
	Console.WriteLine("  fullpath   - Full path to the HLSL file");
	Console.WriteLine("  platform   - Target platform (x86 or x64)");
	Console.WriteLine("  config     - Build configuration (debug or release)");
	Console.WriteLine("  model      - Shader model version, e.g. 6.6");
	Console.WriteLine("  vs         - Semicolon-separated vertex shader entry point specs");
	Console.WriteLine("  ps         - Semicolon-separated pixel shader entry point specs");
	Console.WriteLine("  gs         - Semicolon-separated geometry shader entry point specs");
	Console.WriteLine("  cs         - Semicolon-separated compute shader entry point specs");
	Console.WriteLine("  lib        - Semicolon-separated library target specs");
	Console.WriteLine("  obj        - Produce compiled shader object files");
	Console.WriteLine("  dbg        - Debugging options enabled");
	Console.WriteLine("  trace      - Trace output enabled");
	Console.WriteLine("  pp         - Preprocess output only");
	return;
}
var fullpath = args[0];
var platform = args[1];
var config = args[2];
var shader_model = args[3..].FirstOrDefault(x => x.StartsWith("model=", StringComparison.OrdinalIgnoreCase))?.Split('=', 2)[1];
shader_model = string.IsNullOrWhiteSpace(shader_model) ? "6.6" : shader_model;
var vertex_entry_points = args[3..].FirstOrDefault(x => x.StartsWith("vs=", StringComparison.OrdinalIgnoreCase))?.Split('=', 2)[1] ?? string.Empty;
var pixel_entry_points = args[3..].FirstOrDefault(x => x.StartsWith("ps=", StringComparison.OrdinalIgnoreCase))?.Split('=', 2)[1] ?? string.Empty;
var geometry_entry_points = args[3..].FirstOrDefault(x => x.StartsWith("gs=", StringComparison.OrdinalIgnoreCase))?.Split('=', 2)[1] ?? string.Empty;
var compute_entry_points = args[3..].FirstOrDefault(x => x.StartsWith("cs=", StringComparison.OrdinalIgnoreCase))?.Split('=', 2)[1] ?? string.Empty;
var library_targets = args[3..].FirstOrDefault(x => x.StartsWith("lib=", StringComparison.OrdinalIgnoreCase))?.Split('=', 2)[1] ?? string.Empty;
var pp    = args[3..].Contains("pp");
var obj   = args[3..].Contains("obj");
var trace = args[3..].Contains("trace");
var dbg   = args[3..].Contains("dbg");

var builder = new ShaderBuilder(pp, obj, trace, dbg);
builder.Compile(fullpath, platform, config, shader_model, vertex_entry_points, pixel_entry_points, geometry_entry_points, compute_entry_points, library_targets);
