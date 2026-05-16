#! "net10.0"
// Build shaders using dxc.exe
// Use:
//  dotnet-script.exe BuildShader.csx $(Fullpath) $(PlatformTarget) $(Configuration) [obj] [dbg] [trace] [pp]
//
// Expected input is an hlsl file.
// The file is scanned for: PR_RDR_VSHADER_*, PR_RDR_PSHADER_*, PR_RDR_GSHADER_*, PR_RDR_CSHADER_*, PR_RDR_LSHADER_*, etc
// For each symbol found a compiled shader as header data is generated
// in the output directory.
//
// Add 'entries=<entry-points>' to build named compute shader entry points.
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
using System.Text.RegularExpressions;
using System.Security.Cryptography;
using Rylogic.Common;

public class ShaderBuilder
{
	private struct ShaderType
	{
		public string ShaderCode;
		public string Profile;
		public Regex Patn;
	}
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
	public void Compile(string fullpath, string platform, string config, string shader_model, string entry_points)
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
		var built = false;
		var profile_model = shader_model.Replace(".", "_");

		// Careful with shader versions, if you bump up from 4_0 you'll need to change the minimum feature level in view3d.
		var shader_types = (List<ShaderType>)[
			new ShaderType{ShaderCode = "vs", Profile = $"/Tvs_{profile_model}", Patn = new Regex("^#ifdef PR_RDR_VSHADER_(?<name>.*)$")},
			new ShaderType{ShaderCode = "ps", Profile = $"/Tps_{profile_model}", Patn = new Regex("^#ifdef PR_RDR_PSHADER_(?<name>.*)$")},
			new ShaderType{ShaderCode = "gs", Profile = $"/Tgs_{profile_model}", Patn = new Regex("^#ifdef PR_RDR_GSHADER_(?<name>.*)$")},
			new ShaderType{ShaderCode = "cs", Profile = $"/Tcs_{profile_model}", Patn = new Regex("^#ifdef PR_RDR_CSHADER_(?<name>.*)$")},
			new ShaderType{ShaderCode = "lib", Profile = $"/Tlib_{profile_model}", Patn = new Regex("^#ifdef PR_RDR_LSHADER_(?<name>.*)$")},
		];

		// Scan the file looking for instances of each shader type (there can be more than one)
		foreach (var shader_type in shader_types)
		{
			// Find any shaders of this type in the file
			var shaders = ExtractMany(fullpath, shader_type.Patn);

			// For each matching instance, build the shader
			foreach (var match in shaders)
			{
				var shdr = new ShaderDesc
				{
					Name = $"{match.Groups["name"].Value}_{shader_type.ShaderCode}",
					FullPath = fullpath,
					Profile = shader_type.Profile,
					ShaderCode = shader_type.ShaderCode,
					Define = $"PR_RDR_{shader_type.ShaderCode[0..1].ToUpper()}SHADER_{match.Groups["name"].Value}",
				};
				if (!built)
					File.Delete(stamp);

				BuildShader(shdr, outdir, platform, config);
				built = true;
			}
		}

		// Explicit entry point mode. Used by compute shaders that expose real named entry points instead of View3D's PR_RDR_* wrapper pattern.
		foreach (var entry_point in EntryPoints(entry_points))
		{
			var shdr = new ShaderDesc
			{
				Name = $"{ShaderName(entry_point, "cs")}_cs",
				FullPath = fullpath,
				Profile = $"/Tcs_{profile_model}",
				ShaderCode = "cs",
				EntryPoint = entry_point,
			};
			if (!built)
				File.Delete(stamp);

			BuildShader(shdr, outdir, platform, config);
			built = true;
		}

		// Create the .built file, so that VS's custom build tool can check for it's existence to determine when a build is needed.
		if (built)
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

	// Split the semicolon separated list of explicit entry points.
	private IEnumerable<string> EntryPoints(string entry_points)
	{
		return entry_points
			.Split([';'], StringSplitOptions.RemoveEmptyEntries | StringSplitOptions.TrimEntries)
			.Distinct();
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

	/// <summary> Extract data from a file using a regex pattern </summary>
	private List<Match> ExtractMany(string filepath, Regex patn, Encoding? encoding = null)
	{
		var matches = new List<Match>();
		foreach (var line in File.ReadLines(filepath, encoding ?? Encoding.UTF8))
		{
			var match = patn.Match(line);
			if (match.Success)
				matches.Add(match);
		}
		return matches;
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
	//(string[])[@"E:\Rylogic\Code\projects\rylogic\view3d-12\src\shaders\hlsl\forward\forward.hlsl", "x64", "debug", "obj", "dbg", "trace"];

if (args.Length < 3)
{
	Console.WriteLine("Usage: BuildShader <fullpath> <platform> <config> [model=<shader-model>] [entries=<entry-points>] [pp] [obj] [dbg] [trace]");
	Console.WriteLine("  fullpath   - Full path to the HLSL file");
	Console.WriteLine("  platform   - Target platform (x86 or x64)");
	Console.WriteLine("  config     - Build configuration (debug or release)");
	Console.WriteLine("  model      - Shader model version, e.g. 6.6");
	Console.WriteLine("  entries    - Semicolon-separated explicit compute shader entry points");
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
var entry_points = args[3..].FirstOrDefault(x => x.StartsWith("entries=", StringComparison.OrdinalIgnoreCase))?.Split('=', 2)[1] ?? string.Empty;
var pp    = args[3..].Contains("pp");
var obj   = args[3..].Contains("obj");
var trace = args[3..].Contains("trace");
var dbg   = args[3..].Contains("dbg");

var builder = new ShaderBuilder(pp, obj, trace, dbg);
builder.Compile(fullpath, platform, config, shader_model, entry_points);
