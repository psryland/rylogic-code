using System;
using System.IO;
using System.IO.Pipes;
using System.Text.Json;
using System.Threading;
using System.Threading.Tasks;

namespace LDraw.MCP;

/// <summary>Client for querying a running LDraw instance over its local named pipe</summary>
internal sealed class InstancePipeClient
{
	/// <summary>Read the scene summary from 'registration'</summary>
	public async Task<LDrawSceneSummary> GetSceneSummaryAsync(InstanceRegistration registration)
	{
		// Match the server-side CurrentUserOnly flag so only same-user broker/instance pairs can communicate.
		using var cts = new CancellationTokenSource(TimeSpan.FromSeconds(2));
		using var pipe = new NamedPipeClientStream(".", registration.PipeName, PipeDirection.InOut, PipeOptions.Asynchronous | PipeOptions.CurrentUserOnly);
		await pipe.ConnectAsync(1000, cts.Token).ConfigureAwait(false);

		using var reader = new StreamReader(pipe, leaveOpen: true);
		using var writer = new StreamWriter(pipe, leaveOpen: true) { AutoFlush = true, NewLine = "\n" };

		// Keep the local IPC protocol deliberately simple: one request line, one response line.
		var request = new InstancePipeRequest { Command = InstancePipeCommands.GetSceneSummary };
		await writer.WriteLineAsync(JsonSerializer.Serialize(request, McpJson.Options)).ConfigureAwait(false);

		var line = await reader.ReadLineAsync(cts.Token).ConfigureAwait(false) ?? throw new IOException("No response from LDraw MCP instance pipe.");
		var response = JsonSerializer.Deserialize<InstancePipeResponse>(line, McpJson.Options) ?? throw new IOException("Invalid response from LDraw MCP instance pipe.");
		if (!response.Success)
			throw new IOException(response.Error.Length != 0 ? response.Error : "LDraw MCP instance pipe request failed.");
		if (response.SceneSummary == null)
			throw new IOException("LDraw MCP instance pipe did not return a scene summary.");

		return response.SceneSummary;
	}
}
