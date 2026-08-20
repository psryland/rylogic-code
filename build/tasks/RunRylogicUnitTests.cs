using System;
using System.ComponentModel;
using System.Diagnostics;
using System.IO;
using System.Text;
using Microsoft.Build.Framework;
using Microsoft.Build.Utilities;

// MSBuild task for running Rylogic unit-test entry points without invoking dotnet-script.
public sealed class RunRylogicUnitTests : Task
{
	[Required]
	public string TargetPath
	{
		get;
		set;
	} = string.Empty;

	[Required]
	public bool IsManaged
	{
		get;
		set;
	}

	[Required]
	public string ProjectRoot
	{
		get;
		set;
	} = string.Empty;

	public string PowerShellHost
	{
		get;
		set;
	} = string.Empty;

	// Optional command-line arguments for a native unit-test process.
	public string Arguments
	{
		get;
		set;
	} = string.Empty;

	// Execute the unit-test entry point and report failures without failing the build.
	public override bool Execute()
	{
		try
		{
			if (!File.Exists(TargetPath))
			{
				Log.LogMessage(MessageImportance.High, $"{TargetPath} assembly not found.   **** Unit tests skipped ****");
				return true;
			}

			var working_dir = Path.GetDirectoryName(TargetPath);
			if (string.IsNullOrEmpty(working_dir))
			{
				working_dir = Directory.GetCurrentDirectory();
			}

			var result = IsManaged
				? RunManagedTests()
				: RunProcess(TargetPath, Arguments, working_dir);

			LogOutput(result.Item2, MessageImportance.High);
			LogOutput(result.Item3, MessageImportance.High);
			if (result.Item1 != 0)
			{
				Log.LogWarning("   **** Unit tests failed ****   {0} exited with code {1}", Path.GetFileName(TargetPath), result.Item1);
			}
		}
		catch (Exception ex)
		{
			Log.LogWarning("   **** Unit tests failed ****   {0}", ex.Message);
		}

		return true;
	}

	// Run a managed test assembly via PowerShell so modern target-framework assemblies stay out of the MSBuild host process.
	private Tuple<int, string, string> RunManagedTests()
	{
		var type_name = Path.GetFileNameWithoutExtension(TargetPath) + ".Program";
		var script =
			"$ErrorActionPreference = 'Stop'\n" +
			"Set-Location -LiteralPath " + PowerShellString(ProjectRoot) + "\n" +
			"Add-Type -Path " + PowerShellString(TargetPath) + "\n" +
			"$result = [" + type_name + "]::Main()\n" +
			"if ($null -eq $result) { exit 0 }\n" +
			"exit [int]$result\n";
		var encoded = Convert.ToBase64String(Encoding.Unicode.GetBytes(script));
		var arguments = "-NonInteractive -NoProfile -NoLogo -EncodedCommand " + encoded;
		var host = string.IsNullOrWhiteSpace(PowerShellHost) ? "pwsh.exe" : PowerShellHost;

		try
		{
			return RunProcess(host, arguments, ProjectRoot);
		}
		catch (Win32Exception ex) when (ex.NativeErrorCode == 2 && !string.Equals(host, "powershell.exe", StringComparison.OrdinalIgnoreCase))
		{
			return RunProcess("powershell.exe", arguments, ProjectRoot);
		}
	}

	// Run a child process and capture its output so MSBuild reports it under the owning project.
	private static Tuple<int, string, string> RunProcess(string exe_path, string arguments, string working_dir)
	{
		var stdout = new StringBuilder();
		var stderr = new StringBuilder();

		using (var process = new Process())
		{
			process.StartInfo.FileName = exe_path;
			process.StartInfo.Arguments = arguments;
			process.StartInfo.WorkingDirectory = working_dir;
			process.StartInfo.UseShellExecute = false;
			process.StartInfo.RedirectStandardOutput = true;
			process.StartInfo.RedirectStandardError = true;
			process.StartInfo.CreateNoWindow = true;
			process.OutputDataReceived += (_, e) =>
			{
				if (e.Data == null)
				{
					return;
				}
				lock (stdout)
				{
					stdout.AppendLine(e.Data);
				}
			};
			process.ErrorDataReceived += (_, e) =>
			{
				if (e.Data == null)
				{
					return;
				}
				lock (stderr)
				{
					stderr.AppendLine(e.Data);
				}
			};

			process.Start();
			process.BeginOutputReadLine();
			process.BeginErrorReadLine();
			process.WaitForExit();
			return Tuple.Create(process.ExitCode, stdout.ToString(), stderr.ToString());
		}
	}

	// Write captured process output line-by-line so Visual Studio keeps normal build-output formatting.
	private void LogOutput(string text, MessageImportance importance)
	{
		if (string.IsNullOrEmpty(text))
		{
			return;
		}

		using (var reader = new StringReader(text))
		{
			for (var line = reader.ReadLine(); line != null; line = reader.ReadLine())
			{
				Log.LogMessage(importance, line);
			}
		}
	}

	// Escape a value for a single-quoted PowerShell string literal.
	private static string PowerShellString(string value)
	{
		return "'" + value.Replace("'", "''") + "'";
	}
}
