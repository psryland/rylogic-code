using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Net.Http;
using System.Text.Json.Nodes;
using System.Text.RegularExpressions;
using System.Threading;
using System.Threading.Tasks;
using Microsoft.AspNetCore.Builder;
using Microsoft.AspNetCore.Hosting;
using Microsoft.AspNetCore.Http;
using Microsoft.Extensions.DependencyInjection;
using Microsoft.Extensions.FileProviders;
using Microsoft.Extensions.Logging;

namespace Rylogic.Audio.Authoring.App;

/// <summary>
/// Serves the authoring UI for one project on the local machine.
/// Usage:
///   Rylogic.Audio.Authoring.App --config &lt;authoring.json&gt; [--port 8765]
/// </summary>
public static partial class Program
{
	public static async Task<int> Main(string[] args)
	{
		// Parse the command line; every option has a value
		string? config_path = null;
		var port = 8765;
		for (var i = 0; i != args.Length; ++i)
		{
			switch (args[i])
			{
				case "--config":
				{
					config_path = args[++i];
					break;
				}
				case "--port":
				{
					port = int.Parse(args[++i]);
					break;
				}
				default:
				{
					Console.Error.WriteLine($"Unknown argument '{args[i]}'.");
					return 2;
				}
			}
		}
		if (config_path == null)
		{
			Console.Error.WriteLine("Usage: Rylogic.Audio.Authoring.App --config <authoring.json> [--port 8765]");
			return 2;
		}

		// One HTTP client serves Freesound for the whole session
		var config = AuthoringConfig.Load(config_path);
		using var http = new HttpClient { Timeout = TimeSpan.FromMinutes(10) };
		var workspace = new Workspace(config, new Ffmpeg(), http);

		await Serve(workspace, port);
		return 0;
	}

	/// <summary>Run the web server until it is stopped.</summary>
	private static async Task Serve(Workspace workspace, int port)
	{
		// Bind to loopback only: the tool can read and write project files
		var builder = WebApplication.CreateSlimBuilder();
		builder.WebHost.UseUrls($"http://127.0.0.1:{port}");
		builder.Logging.SetMinimumLevel(LogLevel.Warning);
		var app = builder.Build();
		var web_root = Path.Combine(AppContext.BaseDirectory, "wwwroot");
		app.UseDefaultFiles(new DefaultFilesOptions { FileProvider = new PhysicalFileProvider(web_root) });
		app.UseStaticFiles(new StaticFileOptions { FileProvider = new PhysicalFileProvider(web_root) });

		// The full review state. Refreshing picks up catalog edits and new local files
		app.MapGet("/api/state", () =>
		{
			workspace.Refresh();
			return Results.Content(workspace.Snapshot().ToJsonString(), "application/json");
		});

		// Apply one review action, replying with the new state and a notice or error for the user
		app.MapPost("/api/state", async (HttpRequest request, CancellationToken cancel) =>
		{
			// Errors are shown in the UI rather than failing the request, so the state stays current
			var notice = (string?)null;
			var error = (string?)null;
			try
			{
				var action = await JsonNode.ParseAsync(request.Body, cancellationToken: cancel) as JsonObject ?? throw new InvalidOperationException("The request must be a JSON object.");
				notice = (string?)action["action"] == "add_freesound"
					? await workspace.AddFreesoundAsync((string?)action["id"], FreesoundNumber((string?)action["number"]), cancel)
					: workspace.Apply(action);
			}
			catch (Exception ex) when (ex is InvalidOperationException or InvalidDataException or HttpRequestException or System.Text.Json.JsonException or IOException)
			{
				error = ex.Message;
			}
			var reply = workspace.Snapshot();
			reply["notice"] = notice;
			reply["error"] = error;
			return Results.Content(reply.ToJsonString(), "application/json");
		});

		// Run the project's apply command, replying with the new state and the command's result
		var apply_lock = new SemaphoreSlim(1, 1);
		app.MapPost("/api/apply", async () =>
		{
			// One run at a time. The run is not tied to the request, so closing the page does not stop a half-finished command.
			var notice = (string?)null;
			var error = (string?)null;
			if (workspace.Config.Apply is not { } apply)
			{
				error = "This project has no apply command.";
			}
			else if (!await apply_lock.WaitAsync(0))
			{
				error = "Apply is already running.";
			}
			else
			{
				try
				{
					(notice, error) = await RunApply(apply);
				}
				finally
				{
					apply_lock.Release();
				}
			}
			var reply = workspace.Snapshot();
			reply["notice"] = notice;
			reply["error"] = error;
			return Results.Content(reply.ToJsonString(), "application/json");
		});

		// Freesound login: save the API credential, then send the user to Freesound to grant access
		app.MapPost("/api/freesound", async (HttpRequest request, CancellationToken cancel) =>
		{
			var body = await JsonNode.ParseAsync(request.Body, cancellationToken: cancel);
			var client_id = ((string?)body?["client_id"])?.Trim();
			var client_secret = ((string?)body?["client_secret"])?.Trim();
			if (string.IsNullOrEmpty(client_id) || string.IsNullOrEmpty(client_secret))
				return Results.BadRequest("Both the client ID and the client secret are needed.");

			workspace.Freesound.SetCredentials(client_id, client_secret);
			return Results.Ok();
		});
		app.MapGet("/freesound/connect", () => Results.Redirect(workspace.Freesound.AuthorizeUrl()));
		app.MapGet("/freesound/callback", async (string? code, CancellationToken cancel) =>
		{
			// Freesound returns here with a one-time code that is exchanged for tokens
			if (string.IsNullOrEmpty(code))
				return Results.BadRequest("Freesound did not supply an authorisation code.");

			await workspace.Freesound.ConnectAsync(code, cancel);
			return Results.Redirect("/");
		});

		// Audio files. Each route maps to one folder and refuses paths that leave it
		app.MapGet("/render/{key}.wav", (string key) =>
		{
			return HexId().IsMatch(key) ? AudioFile(workspace.Cache.PathOf(key)) : Results.NotFound();
		});
		app.MapGet("/original/{id}.flac", (string id) =>
		{
			return HexId().IsMatch(id) ? AudioFile(workspace.Sources.FlacPath(id)) : Results.NotFound();
		});
		app.MapGet("/local/{name}/{**path}", (string name, string path) =>
		{
			var folder = workspace.Config.LocalFolders.FirstOrDefault(x => x.Name == name);
			return AudioFile(Contained(folder?.Path, path));
		});
		app.MapGet("/reference/{**path}", (string path) =>
		{
			return AudioFile(Contained(workspace.Config.Root, path));
		});
		app.MapGet("/previews/{**path}", async (string path, CancellationToken cancel) =>
		{
			// Previews are a disposable cache, so a missing one is downloaded again
			var file = Contained(workspace.Config.PreviewsFolder, path);
			if (file != null && !File.Exists(file) && workspace.OriginOfPreview(path) is string origin && OriginsFile.KindOf(origin) == EOriginKind.Freesound)
			{
				var info = await workspace.Freesound.SoundInfoAsync(OriginsFile.ReferenceOf(origin), cancel);
				await workspace.Freesound.DownloadPreviewAsync(info.PreviewUrl, file, cancel);
			}
			return AudioFile(file);
		});

		Console.WriteLine($"Audio authoring for '{workspace.Config.ConfigPath}' at http://127.0.0.1:{port}/");
		await app.RunAsync();
	}

	/// <summary>Run the apply command to completion. Returns a notice on success, or an error with the end of the command's output on failure.</summary>
	private static async Task<(string? notice, string? error)> RunApply(ApplyCommand apply)
	{
		// Collect both output streams together, so a failure can show what the command printed last
		var info = new ProcessStartInfo(apply.Command)
		{
			WorkingDirectory = apply.Directory,
			RedirectStandardOutput = true,
			RedirectStandardError = true,
			UseShellExecute = false,
			CreateNoWindow = true,
		};
		foreach (var argument in apply.Arguments)
			info.ArgumentList.Add(argument);

		var lines = new List<string>();
		void Collect(object sender, DataReceivedEventArgs e)
		{
			// Output arrives on two threads
			if (string.IsNullOrWhiteSpace(e.Data))
				return;

			lock (lines)
				lines.Add(e.Data.Trim());
		}

		// A command that cannot start is reported like one that fails
		Console.WriteLine($"Apply: {apply.Display}");
		using var process = new Process { StartInfo = info };
		process.OutputDataReceived += Collect;
		process.ErrorDataReceived += Collect;
		try
		{
			process.Start();
		}
		catch (Win32Exception ex)
		{
			return (null, $"Apply could not run '{apply.Command}': {ex.Message}");
		}
		process.BeginOutputReadLine();
		process.BeginErrorReadLine();
		await process.WaitForExitAsync();

		// The last line usually summarises the run; a failure shows a few more lines for context
		lock (lines)
		{
			foreach (var line in lines)
				Console.WriteLine($"  {line}");

			return process.ExitCode == 0
				? ($"Applied. {lines.LastOrDefault()}", null)
				: (null, $"Apply failed (exit code {process.ExitCode}): {string.Join(" | ", lines.TakeLast(6))}");
		}
	}

	/// <summary>The sound number from a Freesound number or sound page address.</summary>
	private static string FreesoundNumber(string? text)
	{
		// Accept '483323' or 'https://freesound.org/people/x/sounds/483323/'
		var match = FreesoundNumberPattern().Match(text ?? string.Empty);
		return match.Success ? match.Groups[1].Value : throw new InvalidOperationException("Enter a Freesound sound number or sound page address.");
	}

	/// <summary>The full path of 'relative' inside 'folder', or null if there is no folder or the path leaves it.</summary>
	private static string? Contained(string? folder, string relative)
	{
		// Compare full paths so '..' segments cannot escape the folder
		if (folder == null)
			return null;

		var root = Path.GetFullPath(folder).TrimEnd(Path.DirectorySeparatorChar) + Path.DirectorySeparatorChar;
		var path = Path.GetFullPath(Path.Combine(root, relative));
		return path.StartsWith(root, StringComparison.OrdinalIgnoreCase) ? path : null;
	}

	/// <summary>Serve an audio file with range support, so the browser can seek.</summary>
	private static IResult AudioFile(string? path)
	{
		// The content type comes from the extension
		if (path == null || !File.Exists(path))
			return Results.NotFound();

		var type = Path.GetExtension(path).ToLowerInvariant() switch
		{
			".wav" => "audio/wav",
			".flac" => "audio/flac",
			".mp3" => "audio/mpeg",
			".ogg" => "audio/ogg",
			_ => "application/octet-stream",
		};
		return Results.File(path, type, enableRangeProcessing: true);
	}

	[GeneratedRegex("^[0-9a-f]{8,64}$")]
	private static partial Regex HexId();

	[GeneratedRegex(@"(?:^|/)(\d+)/?\s*$")]
	private static partial Regex FreesoundNumberPattern();
}
