using System;
using System.ComponentModel;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Security.Cryptography;
using System.Text;
using System.Threading;
using Microsoft.Build.Framework;
using Microsoft.Build.Utilities;

// Installs a Release tool only when its deployed executable is absent; existing payloads are never refreshed.
public sealed class BootstrapRepositoryTool : Task
{
	[Required]
	public string RepositoryRoot { get; set; } = string.Empty;
	[Required]
	public string ToolName { get; set; } = string.Empty;
	[Required]
	public string DestinationDirectory { get; set; } = string.Empty;
	public string NativeMSBuild { get; set; } = string.Empty;

	public override bool Execute()
	{
		string work_dir = null;
		try
		{
			if (ToolName != "code_sync" && ToolName != "VSRedirector")
				throw new ArgumentException("Unknown repository tool: " + ToolName);

			var destination = Path.GetFullPath(DestinationDirectory);
			var executable = Path.Combine(destination, ToolName + ".exe");
			if (File.Exists(executable))
				return true;

			// Serialise bootstraps of the same tool, including requests from parallel consumers.
			using var hash = SHA256.Create();
			var identity = Path.GetFullPath(RepositoryRoot).ToUpperInvariant() + "|" + ToolName;
			var mutex_name = @"Local\RylogicToolBootstrap." + BitConverter.ToString(hash.ComputeHash(Encoding.UTF8.GetBytes(identity))).Replace("-", "");
			using var mutex = new Mutex(false, mutex_name);
			var acquired = false;
			try
			{
				try { acquired = mutex.WaitOne(TimeSpan.FromMinutes(10)); }
				catch (AbandonedMutexException) { acquired = true; }
				if (!acquired)
					throw new TimeoutException("Timed out waiting for " + ToolName + " bootstrap.");

				if (File.Exists(executable))
					return true;

				// Build away from ordinary build outputs, with every recursive tool hook disabled.
				work_dir = Path.Combine(Path.GetDirectoryName(destination), "." + ToolName + ".bootstrap-" + Guid.NewGuid().ToString("N"));
				var payload = Path.Combine(work_dir, "payload");
				Directory.CreateDirectory(payload);
				Log.LogMessage(MessageImportance.High, "Bootstrapping missing {0} in Release to {1}", ToolName, destination);
				BuildRelease(work_dir, payload);
				var required = ToolName == "VSRedirector"
					? new[] { "VSRedirector.exe", "VSRedirector.dll", "VSRedirector.deps.json", "VSRedirector.runtimeconfig.json" }
					: new[] { "code_sync.exe" };
				foreach (var file in required)
				{
					if (!File.Exists(Path.Combine(payload, file)))
						throw new IOException("Release build did not produce required tool file: " + file);
				}
				PublishMissing(payload, destination, ToolName + ".exe");
				Log.LogMessage(MessageImportance.High, "Deployed missing Release tool: {0}", executable);
				return true;
			}
			finally
			{
				if (acquired) mutex.ReleaseMutex();
			}
		}
		catch (Exception ex) when (ex is IOException or UnauthorizedAccessException or InvalidOperationException or ArgumentException or TimeoutException or Win32Exception or CryptographicException)
		{
			Log.LogError("Could not bootstrap {0}: {1}", ToolName, ex.Message);
			return false;
		}
		finally
		{
			// Only this invocation's uniquely named generated directory is eligible for cleanup.
			if (work_dir != null && Directory.Exists(work_dir))
			{
				try { Directory.Delete(work_dir, true); }
				catch (Exception ex) when (ex is IOException or UnauthorizedAccessException)
				{
					Log.LogWarning("Could not remove tool bootstrap scratch directory '{0}': {1}", work_dir, ex.Message);
				}
			}
		}
	}

	private void BuildRelease(string work_dir, string payload)
	{
		var project_dir = Path.Combine(RepositoryRoot, "projects", "tools", ToolName);
		var intermediate = Path.Combine(work_dir, "obj") + Path.DirectorySeparatorChar;
		if (ToolName == "VSRedirector")
		{
			// Read the tool's framework, not the consumer's (which may be native or target a different runtime).
			var project = Path.Combine(project_dir, "VSRedirector.csproj");
			var target_framework = Run("dotnet", new[] { "msbuild", project, "-getProperty:PrimaryTarget", "-nologo" }).Trim();
			if (string.IsNullOrWhiteSpace(target_framework))
				throw new InvalidOperationException("VSRedirector project did not define PrimaryTarget.");

			Run("dotnet", new[] {
				"publish", project, "-c", "Release", "-f", target_framework, "-o", payload,
				"--nologo", "-v", "minimal", "-p:BootstrapRepositoryTools=false", "-p:RunCodeSync=false",
				"-p:BaseIntermediateOutputPath=" + intermediate, "-p:MSBuildProjectExtensionsPath=" + intermediate,
				"-p:OutputPath=" + Path.Combine(work_dir, "build") + Path.DirectorySeparatorChar,
			});
		}
		else
		{
			var output = Path.Combine(work_dir, "build") + Path.DirectorySeparatorChar;
			var msbuild = NativeMSBuild;
			if (string.IsNullOrWhiteSpace(msbuild))
			{
				var vswhere = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.ProgramFilesX86), @"Microsoft Visual Studio\Installer\vswhere.exe");
				msbuild = Run(vswhere, new[] { "-latest", "-version", "[18.0,19.0)", "-products", "*", "-requires", "Microsoft.VisualStudio.Component.VC.Tools.x86.x64", "-find", @"MSBuild\Current\Bin\amd64\MSBuild.exe" }).Trim();
				if (!File.Exists(msbuild))
					throw new FileNotFoundException("VS 2026 MSBuild with C++ tools was not found. Install the C++ workload or set RepositoryToolMSBuild.");
			}
			Run(msbuild, new[] {
				Path.Combine(project_dir, "code_sync.vcxproj"), "/t:Build", "/p:Configuration=Release", "/p:Platform=x64", "/p:PlatformTarget=x64",
				"/p:BootstrapRepositoryTools=false", "/p:RunCodeSync=false", "/p:OutDir=" + output, "/p:IntDir=" + intermediate, "/nologo", "/v:minimal",
			});
			// code_sync uses the static C++ runtime and has no app-local runtime DLLs.
			File.Copy(Path.Combine(output, "code_sync.exe"), Path.Combine(payload, "code_sync.exe"));
		}
	}

	// The executable is the completion marker: publish it atomically, only after all companion files are ready.
	private static void PublishMissing(string payload, string destination, string executable_name)
	{
		if (!Directory.Exists(destination))
		{
			Directory.Move(payload, destination);
			return;
		}
		var executable = Path.Combine(destination, executable_name);
		if (File.Exists(executable))
			return;

		foreach (var source in Directory.GetFiles(payload, "*", SearchOption.AllDirectories))
		{
			var relative = source.Substring(payload.Length + 1);
			if (relative.Equals(executable_name, StringComparison.OrdinalIgnoreCase)) continue;
			var target = Path.Combine(destination, relative);
			Directory.CreateDirectory(Path.GetDirectoryName(target));
			File.Copy(source, target, true);
		}
		var temporary = Path.Combine(destination, "." + executable_name + "." + Guid.NewGuid().ToString("N") + ".tmp");
		try
		{
			File.Copy(Path.Combine(payload, executable_name), temporary);
			File.Move(temporary, executable);
		}
		finally
		{
			if (File.Exists(temporary)) File.Delete(temporary);
		}
	}

	private string Run(string executable, string[] arguments)
	{
		using var process = new Process
		{
			StartInfo = new ProcessStartInfo(executable, string.Join(" ", arguments.Select(Quote)))
			{
				UseShellExecute = false,
				CreateNoWindow = true,
				RedirectStandardOutput = true,
				RedirectStandardError = true,
			},
		};
		process.Start();
		var output = process.StandardOutput.ReadToEndAsync();
		var errors = process.StandardError.ReadToEndAsync();
		if (!process.WaitForExit(600000))
		{
			process.Kill();
			throw new TimeoutException("Tool bootstrap build exceeded ten minutes: " + executable);
		}
		var stdout = output.GetAwaiter().GetResult();
		var stderr = errors.GetAwaiter().GetResult();
		if (process.ExitCode != 0)
			throw new InvalidOperationException(executable + " exited " + process.ExitCode + Environment.NewLine + stdout + stderr);

		if (stdout.Length != 0) Log.LogMessage(MessageImportance.High, stdout.TrimEnd());
		if (stderr.Length != 0) Log.LogMessage(MessageImportance.High, stderr.TrimEnd());
		return stdout;
	}

	// MSBuild's desktop host also uses this task, so use Windows argument quoting without ProcessStartInfo.ArgumentList.
	private static string Quote(string argument)
	{
		var result = new StringBuilder("\"");
		var slashes = 0;
		foreach (var ch in argument)
		{
			if (ch == '\\')
			{
				++slashes;
				continue;
			}
			result.Append('\\', ch == '"' ? slashes * 2 + 1 : slashes);
			result.Append(ch);
			slashes = 0;
		}
		result.Append('\\', slashes * 2);
		return result.Append('"').ToString();
	}
}
