using System.Diagnostics;
using System.Text;
using System.Text.Json;

namespace vsredirector;

internal sealed record Installation(string Executable, Version Version);

internal static class Installations
{
	// Ask the installed Setup discovery tool for complete stable IDEs, excluding Build Tools.
	public static Installation Newest()
	{
		var tool = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.ProgramFilesX86), @"Microsoft Visual Studio\Installer\vswhere.exe");
		if (!File.Exists(tool))
			throw new FileNotFoundException("Visual Studio Installer's vswhere.exe is missing. Repair/install Visual Studio Installer.", tool);

		var info = new ProcessStartInfo(tool) { UseShellExecute = false, CreateNoWindow = true, RedirectStandardOutput = true, RedirectStandardError = true, StandardOutputEncoding = Encoding.UTF8 };
		foreach (var arg in new[] { "-products", "*", "-format", "json", "-utf8" })
			info.ArgumentList.Add(arg);

		using var process = Process.Start(info) ?? throw new InvalidOperationException("Could not start vswhere.");
		var output = process.StandardOutput.ReadToEndAsync();
		var error = process.StandardError.ReadToEndAsync();
		if (!process.WaitForExit(10000))
		{
			process.Kill();
			throw new TimeoutException("vswhere did not finish within 10 seconds. Check Visual Studio Installer.");
		}
		if (process.ExitCode != 0)
			throw new InvalidOperationException($"vswhere failed ({process.ExitCode}): {error.GetAwaiter().GetResult()}");

		var newest = Select(output.GetAwaiter().GetResult());
		if (!File.Exists(newest.Executable))
			throw new FileNotFoundException("The newest stable Visual Studio IDE executable is missing. Repair its installation.", newest.Executable);

		return newest;
	}

	// Parse numeric versions rather than comparing display names or version strings.
	internal static Installation Select(string json)
	{
		using var document = JsonDocument.Parse(json);
		var candidates = new List<Installation>();
		foreach (var item in document.RootElement.EnumerateArray())
		{
			if (item.GetProperty("isPrerelease").GetBoolean() || !item.GetProperty("isComplete").GetBoolean() || !item.GetProperty("isLaunchable").GetBoolean())
				continue;

			var product = item.GetProperty("productId").GetString();
			if (product is not ("Microsoft.VisualStudio.Product.Community" or "Microsoft.VisualStudio.Product.Professional" or "Microsoft.VisualStudio.Product.Enterprise"))
				continue;

			var path = item.GetProperty("productPath").GetString() ?? throw new InvalidOperationException("Missing IDE product path.");
			if (!Path.IsPathFullyQualified(path) || !Path.GetFileName(path).Equals("devenv.exe", StringComparison.OrdinalIgnoreCase))
				throw new InvalidOperationException($"Unexpected IDE executable: {path}");

			candidates.Add(new Installation(path, Version.Parse(item.GetProperty("installationVersion").GetString()!)));
		}
		return candidates.OrderByDescending(x => x.Version).ThenBy(x => x.Executable, StringComparer.OrdinalIgnoreCase).FirstOrDefault()
			?? throw new InvalidOperationException("No complete, launchable, stable Visual Studio IDE is installed (Preview and Build Tools are excluded).");
	}

	// Start an independent IDE without passing file paths or navigation commands to its command-line parser.
	internal static ProcessStartInfo LaunchInfo(Installation installation)
	{
		// Shell execution does not inherit the worker's result pipes; redirecting new standard streams alone does not prevent handle inheritance.
		return new ProcessStartInfo(installation.Executable) { UseShellExecute = true };
	}
}

#if PR_UNITTESTS
internal static partial class Tests
{
	private static void TestInstallations()
	{
		var records = new[]
		{
			new { isPrerelease = false, isComplete = true, isLaunchable = true, productId = "Microsoft.VisualStudio.Product.Enterprise", productPath = @"C:\old\devenv.exe", installationVersion = "18.9.1.0" },
			new { isPrerelease = false, isComplete = true, isLaunchable = true, productId = "Microsoft.VisualStudio.Product.Community", productPath = @"C:\new\devenv.exe", installationVersion = "18.10.1.0" },
			new { isPrerelease = true, isComplete = true, isLaunchable = true, productId = "Microsoft.VisualStudio.Product.Enterprise", productPath = @"C:\preview\devenv.exe", installationVersion = "19.0.0.0" },
			new { isPrerelease = false, isComplete = true, isLaunchable = true, productId = "Microsoft.VisualStudio.Product.BuildTools", productPath = @"C:\tools\LaunchDevCmd.bat", installationVersion = "20.0.0.0" },
			new { isPrerelease = false, isComplete = false, isLaunchable = true, productId = "Microsoft.VisualStudio.Product.Enterprise", productPath = @"C:\incomplete\devenv.exe", installationVersion = "21.0.0.0" },
		};
		var selected = Installations.Select(JsonSerializer.Serialize(records));
		Check(selected.Executable == @"C:\new\devenv.exe", "numeric stable IDE selection");
		Throws(() => Installations.Select(JsonSerializer.Serialize(records.Skip(2))), "no eligible installation");
		Throws(() => Installations.Select("[]"), "no installation");
		var info = Installations.LaunchInfo(selected);
		Check(info.FileName == selected.Executable && info.ArgumentList.Count == 0 && info.Arguments.Length == 0, "new IDE has no command-line file parsing or /Edit routing");
		Check(info.UseShellExecute && !info.RedirectStandardInput && !info.RedirectStandardOutput && !info.RedirectStandardError, "new IDE uses an independent shell launch");
	}
}
#endif
