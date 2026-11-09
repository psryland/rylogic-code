using System;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Text.Json.Nodes;
using System.Threading;
using System.Threading.Tasks;

namespace Rylogic.Audio.Authoring;

/// <summary>Whether an AI assistant can be used, and if not, what the user must do to enable it.</summary>
public sealed record AssistantStatus(bool Available, string Description);

/// <summary>An AI model that answers one self-contained prompt with a JSON object. It has no tools and sees nothing but the prompt.</summary>
public interface IAssistant
{
	/// <summary>Whether the assistant can be used now.</summary>
	AssistantStatus Status { get; }

	/// <summary>Answer 'prompt', which must ask for a single JSON object. Throws <see cref="InvalidOperationException"/> if no JSON object is returned.</summary>
	Task<JsonObject> AskAsync(string prompt, CancellationToken cancel);
}

/// <summary>An assistant run through the GitHub Copilot CLI, using the user's own Copilot sign-in.</summary>
public sealed class CopilotAssistant : IAssistant
{
	/// <summary>How to make the assistant available, shown while it is not.</summary>
	public const string EnableHint = "Install the GitHub Copilot CLI ('winget install GitHub.Copilot'), run 'copilot' once to sign in, then restart this tool.";

	/// <summary>The longest prompt passed on the command line, which Windows limits to 32767 characters in total.</summary>
	public const int MaxPrompt = 30000;

	private readonly string? m_exe;

	/// <summary>Use 'model' through the Copilot CLI at 'command', or the 'copilot' found on PATH when 'command' is null.</summary>
	public CopilotAssistant(string model, string? command)
	{
		// The executable is found once; installing it later needs a restart
		Model = model;
		m_exe = command ?? FindOnPath("copilot");
		Status = m_exe != null && File.Exists(m_exe)
			? new AssistantStatus(true, $"Copilot CLI with {model}")
			: new AssistantStatus(false, EnableHint);
	}

	/// <summary>The model that answers prompts.</summary>
	public string Model { get; }

	/// <inheritdoc/>
	public AssistantStatus Status { get; }

	/// <inheritdoc/>
	public async Task<JsonObject> AskAsync(string prompt, CancellationToken cancel)
	{
		// Run in an empty folder with no tools and no custom instructions, so the model sees only the prompt
		if (!Status.Available)
			throw new InvalidOperationException(EnableHint);
		if (prompt.Length > MaxPrompt)
			throw new ArgumentException($"The prompt is longer than {MaxPrompt} characters.", nameof(prompt));

		var folder = Directory.CreateTempSubdirectory("rylogic-assistant-");
		try
		{
			var info = new ProcessStartInfo(m_exe!)
			{
				WorkingDirectory = folder.FullName,
				RedirectStandardOutput = true,
				RedirectStandardError = true,
				UseShellExecute = false,
				CreateNoWindow = true,
			};
			foreach (var arg in new[] { "-p", prompt, "--model", Model, "-s", "--no-custom-instructions", "--disable-builtin-mcps", "--no-ask-user", "--no-color", "--no-auto-update", "--available-tools" })
				info.ArgumentList.Add(arg);

			// Read both streams while waiting, so a full pipe cannot stall the process. Cancelling stops the whole process tree.
			using var process = Process.Start(info) ?? throw new InvalidOperationException("The Copilot CLI could not be started.");
			var stdout = process.StandardOutput.ReadToEndAsync(cancel);
			var stderr = process.StandardError.ReadToEndAsync(cancel);
			try
			{
				await process.WaitForExitAsync(cancel);
			}
			catch (OperationCanceledException)
			{
				process.Kill(entireProcessTree: true);
				throw;
			}
			var output = await stdout;
			if (process.ExitCode != 0)
				throw new InvalidOperationException($"The Copilot CLI failed ({process.ExitCode}): {FirstLine(await stderr) ?? FirstLine(output) ?? "no message"}. If it is not signed in, run 'copilot' once to sign in.");

			return ParseReply(output);
		}
		finally
		{
			// The folder only isolates the run
			try
			{
				folder.Delete(recursive: true);
			}
			catch (IOException)
			{
			}
		}
	}

	/// <summary>The JSON object in a model reply, ignoring any text or code fence around it.</summary>
	public static JsonObject ParseReply(string reply)
	{
		// Models sometimes wrap the object in a code fence or add a sentence, so the outermost braces delimit it
		var start = reply.IndexOf('{');
		var end = reply.LastIndexOf('}');
		if (start < 0 || end < start)
			throw new InvalidOperationException($"The assistant did not reply with JSON: {FirstLine(reply) ?? "(empty)"}");

		try
		{
			return JsonNode.Parse(reply[start..(end + 1)]) as JsonObject ?? throw new InvalidOperationException("The assistant's reply is not a JSON object.");
		}
		catch (System.Text.Json.JsonException ex)
		{
			throw new InvalidOperationException($"The assistant's reply is not valid JSON: {ex.Message}");
		}
	}

	/// <summary>The full path of executable 'name' on PATH, or null.</summary>
	private static string? FindOnPath(string name)
	{
		// Try each PATH folder with each executable extension, as the shell would
		var extensions = (Environment.GetEnvironmentVariable("PATHEXT") ?? ".EXE").Split(';', StringSplitOptions.RemoveEmptyEntries);
		var folders = (Environment.GetEnvironmentVariable("PATH") ?? string.Empty).Split(Path.PathSeparator, StringSplitOptions.RemoveEmptyEntries);
		return folders
			.SelectMany(folder => extensions.Select(ext => Path.Combine(folder.Trim('"'), name + ext)))
			.FirstOrDefault(File.Exists);
	}

	/// <summary>The first non-blank line of 'text', or null.</summary>
	private static string? FirstLine(string text)
	{
		return text.Split('\n').Select(x => x.Trim()).FirstOrDefault(x => x.Length != 0);
	}
}
