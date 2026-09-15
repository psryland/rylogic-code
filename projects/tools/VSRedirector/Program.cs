using System.Diagnostics;
using System.Text.Json;

namespace vsredirector;

internal static class Program
{
	// Isolate potentially hung out-of-process COM calls so GitKraken receives a bounded, visible failure.
	[STAThread]
	public static int Main(string[] args)
	{
		var worker = args.Length != 0 && args[0] == "--worker";
		var quiet = worker || args.Contains("--inspect") || args.Contains("--self-test");
		try
		{
#if PR_UNITTESTS
			if (args.SequenceEqual(new[] { "--self-test" }))
			{
				Tests.Run();
				return 0;
			}
#endif
			if (worker) args = args[1..];
			var request = Request.Parse(args);
			if (!File.Exists(request.FilePath))
				throw new FileNotFoundException("The requested path must be an existing file, not a folder.", request.FilePath);

			if (!worker) return Supervise(args, request.Inspect);
			return Run(request);
		}
		catch (Exception ex)
		{
			var message = $"VSRedirector: {ex.Message}";
			if (ex.InnerException != null) message += $"\n{ex.InnerException.Message} (0x{ex.InnerException.HResult:X8})";
			Console.Error.WriteLine(message);
			if (!quiet) Native.MessageBoxW(nint.Zero, message, "VSRedirector", 0x10);
			return 1;
		}
	}

	private static int Supervise(string[] args, bool inspect)
	{
		// Use the apphost even when launched through 'dotnet VSRedirector.dll'.
		var executable = Path.Combine(AppContext.BaseDirectory, "VSRedirector.exe");
		var info = new ProcessStartInfo(executable) { UseShellExecute = false, CreateNoWindow = true, RedirectStandardOutput = true, RedirectStandardError = true };
		info.ArgumentList.Add("--worker");
		foreach (var arg in args) info.ArgumentList.Add(arg);
		using var process = Process.Start(info) ?? throw new InvalidOperationException("Could not start automation worker.");
		var output = process.StandardOutput.ReadToEndAsync();
		var error = process.StandardError.ReadToEndAsync();
		if (!process.WaitForExit(90000))
		{
			// Do not kill the process tree: any newly launched Visual Studio belongs to the user.
			process.Kill();
			throw new TimeoutException("Automation exceeded 90 seconds. The worker was stopped, not Visual Studio. Check busy/elevated IDEs and any newly started instance before retrying; a file operation may already have completed.");
		}
		if (!Task.WhenAll(output, error).Wait(TimeSpan.FromSeconds(1)))
			throw new TimeoutException("The automation worker exited but its result pipes did not close. Check the selected/new IDE before retrying.");

		Console.Write(output.GetAwaiter().GetResult());
		var errors = error.GetAwaiter().GetResult();
		Console.Error.Write(errors);
		if (process.ExitCode != 0 && !inspect)
			Native.MessageBoxW(nint.Zero, string.IsNullOrWhiteSpace(errors) ? $"Automation worker failed ({process.ExitCode})." : errors, "VSRedirector", 0x10);

		return process.ExitCode;
	}

	private static int Run(Request request)
	{
		using var visual_studio = new VisualStudio();
		var instances = visual_studio.Inspect(request);
		var selected = Routing.Select(instances);
		var installation = selected == null ? Installations.Newest() : null;
		if (request.Inspect)
		{
			Console.WriteLine(JsonSerializer.Serialize(new { Request = request, Instances = instances, SelectedProcessId = selected?.ProcessId, NewInstallation = installation }, new JsonSerializerOptions { WriteIndented = true }));
			return 0;
		}
		if (selected != null)
		{
			visual_studio.Open(selected.ProcessId, request);
			return 0;
		}

		// File.OpenFile starts a new IDE without passing a solution/project as a positional argument.
		using var process = Process.Start(Installations.LaunchInfo(installation!, request)) ?? throw new InvalidOperationException("Could not start Visual Studio.");
		try
		{
			process.StandardInput.Close();
			process.BeginOutputReadLine();
			process.BeginErrorReadLine();
			visual_studio.WaitForNew(process);
			visual_studio.Open(process.Id, request, require_empty_solution: true);
		}
		catch (Exception ex)
		{
			throw new InvalidOperationException($"Visual Studio PID {process.Id} was launched, but file/line navigation could not be confirmed. The instance was left running; inspect it before retrying. {ex.Message}", ex);
		}
		return 0;
	}
}

#if PR_UNITTESTS
internal static partial class Tests
{
	private static int s_count;

	public static void Run()
	{
		TestRouting();
		TestInstallations();
		TestNavigation();
		Check(!VisualStudio.IsRealProject("{66A26720-8FB5-11D2-AA7E-00C04F688DDE}"), "solution folder exclusion");
		Check(!VisualStudio.IsRealProject("{66A2671D-8FB5-11D2-AA7E-00C04F688DDE}"), "miscellaneous file exclusion");
		Check(!VisualStudio.IsRealProject("{67294A52-A4F0-11D2-AA88-00C04F688DDE}"), "unloaded project exclusion");
		Check(VisualStudio.IsRealProject("{FAE04EC0-301F-11D3-BF4B-00C04F79EFBC}"), "C# project");
		Check(!VisualStudio.HasLoadedSolution(true, ""), "loose document synthetic solution");
		Check(!VisualStudio.HasLoadedSolution(false, @"C:\a.sln"), "closed solution");
		Check(VisualStudio.HasLoadedSolution(true, @"C:\a.slnx"), "loaded solution");
		Console.WriteLine($"VSRedirector: {s_count} checks passed.");
	}
	private static void Check(bool condition, string name)
	{
		if (!condition) throw new InvalidOperationException($"Test failed: {name}");
		++s_count;
	}
	private static void Throws(Action action, string name)
	{
		try { action(); }
		catch (Exception ex) when (ex is ArgumentException or InvalidOperationException)
		{
			++s_count;
			return;
		}
		throw new InvalidOperationException($"Test failed: {name} did not throw.");
	}
}
#endif
