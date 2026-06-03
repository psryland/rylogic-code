using System;
using System.IO;
using System.IO.Pipes;
using System.Text.Json;
using System.Threading;
using System.Threading.Tasks;

namespace LDraw.MCP;

/// <summary>Client for querying a running LDraw instance over its local named pipe</summary>
internal sealed partial class InstancePipeClient
{
	private static readonly TimeSpan ReadTimeout = TimeSpan.FromSeconds(5);
	private static readonly TimeSpan WriteTimeout = TimeSpan.FromSeconds(30);

	/// <summary>Read the scene summary from 'registration'</summary>
	public async Task<LDrawSceneSummary> GetSceneSummaryAsync(InstanceRegistration registration)
	{
		return await SendAsync<LDrawSceneSummary>(registration, InstancePipeCommands.GetSceneSummary, null, ReadTimeout).ConfigureAwait(false);
	}

	/// <summary>Probe an instance's readiness and identity over its pipe</summary>
	public async Task<LDrawPingResult> PingAsync(InstanceRegistration registration)
	{
		return await SendAsync<LDrawPingResult>(registration, InstancePipeCommands.Ping, null, ReadTimeout).ConfigureAwait(false);
	}

	/// <summary>Read the scene list from 'registration'</summary>
	public async Task<LDrawSceneList> ListScenesAsync(InstanceRegistration registration, LDrawListScenesParams parameters)
	{
		return await SendAsync<LDrawSceneList>(registration, InstancePipeCommands.ListScenes, parameters, ReadTimeout).ConfigureAwait(false);
	}

	/// <summary>Read view settings from 'registration'</summary>
	public async Task<LDrawViewSettingsInfo> GetViewSettingsAsync(InstanceRegistration registration, LDrawViewSettingsParams parameters)
	{
		return await SendAsync<LDrawViewSettingsInfo>(registration, InstancePipeCommands.GetViewSettings, parameters, ReadTimeout).ConfigureAwait(false);
	}

	/// <summary>Read the object list from 'registration'</summary>
	public async Task<LDrawObjectList> ListObjectsAsync(InstanceRegistration registration, LDrawListObjectsParams parameters)
	{
		return await SendAsync<LDrawObjectList>(registration, InstancePipeCommands.ListObjects, parameters, ReadTimeout).ConfigureAwait(false);
	}

	/// <summary>Read the camera state from 'registration'</summary>
	public async Task<LDrawCameraInfo> GetCameraAsync(InstanceRegistration registration, LDrawCameraParams parameters)
	{
		return await SendAsync<LDrawCameraInfo>(registration, InstancePipeCommands.GetCamera, parameters, ReadTimeout).ConfigureAwait(false);
	}

	/// <summary>Set the camera state for 'registration'</summary>
	public async Task<LDrawCameraSetResult> SetCameraAsync(InstanceRegistration registration, LDrawSetCameraParams parameters)
	{
		return await SendAsync<LDrawCameraSetResult>(registration, InstancePipeCommands.SetCamera, parameters, WriteTimeout).ConfigureAwait(false);
	}

	/// <summary>Set chart axis ranges for 'registration'</summary>
	public async Task<LDrawAxisRangeResult> SetAxisRangesAsync(InstanceRegistration registration, LDrawSetAxisRangesParams parameters)
	{
		return await SendAsync<LDrawAxisRangeResult>(registration, InstancePipeCommands.SetAxisRanges, parameters, WriteTimeout).ConfigureAwait(false);
	}

	/// <summary>Replace a named overlay script in 'registration'</summary>
	public async Task<LDrawOverlayResult> OverlaySetScriptAsync(InstanceRegistration registration, LDrawOverlayScriptParams parameters)
	{
		return await SendAsync<LDrawOverlayResult>(registration, InstancePipeCommands.OverlaySetScript, parameters, WriteTimeout).ConfigureAwait(false);
	}

	/// <summary>Append to a named overlay script in 'registration'</summary>
	public async Task<LDrawOverlayResult> OverlayAppendScriptAsync(InstanceRegistration registration, LDrawOverlayScriptParams parameters)
	{
		return await SendAsync<LDrawOverlayResult>(registration, InstancePipeCommands.OverlayAppendScript, parameters, WriteTimeout).ConfigureAwait(false);
	}

	/// <summary>Clear one or more overlays from 'registration'</summary>
	public async Task<LDrawOverlayResult[]> OverlayClearAsync(InstanceRegistration registration, LDrawOverlayClearParams parameters)
	{
		return await SendAsync<LDrawOverlayResult[]>(registration, InstancePipeCommands.OverlayClear, parameters, WriteTimeout).ConfigureAwait(false);
	}

	/// <summary>Create or replace a generated chart in 'registration'</summary>
	public async Task<LDrawChartResult> ChartCreateAsync(InstanceRegistration registration, LDrawChartCreateParams parameters)
	{
		return await SendAsync<LDrawChartResult>(registration, InstancePipeCommands.ChartCreate, parameters, WriteTimeout).ConfigureAwait(false);
	}

	/// <summary>Replace generated chart data in 'registration'</summary>
	public async Task<LDrawChartResult> ChartUpdateDataAsync(InstanceRegistration registration, LDrawChartUpdateDataParams parameters)
	{
		return await SendAsync<LDrawChartResult>(registration, InstancePipeCommands.ChartUpdateData, parameters, WriteTimeout).ConfigureAwait(false);
	}

	/// <summary>Add a series to a generated chart in 'registration'</summary>
	public async Task<LDrawChartResult> ChartAddSeriesAsync(InstanceRegistration registration, LDrawChartAddSeriesParams parameters)
	{
		return await SendAsync<LDrawChartResult>(registration, InstancePipeCommands.ChartAddSeries, parameters, WriteTimeout).ConfigureAwait(false);
	}

	/// <summary>Set generated chart axis ranges in 'registration'</summary>
	public async Task<LDrawAxisRangeResult> ChartSetAxisRangesAsync(InstanceRegistration registration, LDrawChartSetAxisRangesParams parameters)
	{
		return await SendAsync<LDrawAxisRangeResult>(registration, InstancePipeCommands.ChartSetAxisRanges, parameters, WriteTimeout).ConfigureAwait(false);
	}

	/// <summary>Clear one or all generated charts from 'registration'</summary>
	public async Task<LDrawChartResult> ChartClearAsync(InstanceRegistration registration, LDrawChartClearParams parameters)
	{
		return await SendAsync<LDrawChartResult>(registration, InstancePipeCommands.ChartClear, parameters, WriteTimeout).ConfigureAwait(false);
	}

	/// <summary>Send a typed command to an instance pipe and return the typed response payload</summary>
	private static async Task<T> SendAsync<T>(InstanceRegistration registration, string command, object? parameters, TimeSpan timeout)
	{
		// Match the server-side CurrentUserOnly flag so only same-user broker/instance pairs can communicate.
		using var cts = new CancellationTokenSource(timeout);
		using var pipe = new NamedPipeClientStream(".", registration.PipeName, PipeDirection.InOut, PipeOptions.Asynchronous | PipeOptions.CurrentUserOnly);
		await pipe.ConnectAsync(1000, cts.Token).ConfigureAwait(false);

		using var reader = new StreamReader(pipe, leaveOpen: true);
		using var writer = new StreamWriter(pipe, leaveOpen: true) { AutoFlush = true, NewLine = "\n" };

		// Keep the local IPC protocol deliberately simple: one request line, one response line.
		var request = new InstancePipeRequest { Command = command };
		if (parameters != null)
			request.Parameters = JsonSerializer.SerializeToElement(parameters, McpJson.LineOptions);
		await writer.WriteLineAsync(JsonSerializer.Serialize(request, McpJson.LineOptions)).ConfigureAwait(false);

		var line = await reader.ReadLineAsync(cts.Token).ConfigureAwait(false) ?? throw new IOException("No response from LDraw MCP instance pipe.");
		var response = JsonSerializer.Deserialize<InstancePipeResponse>(line, McpJson.LineOptions) ?? throw new IOException("Invalid response from LDraw MCP instance pipe.");
		if (!response.Success)
			throw new IOException(response.Error.Length != 0 ? response.Error : "LDraw MCP instance pipe request failed.");
		if (response.Payload is not JsonElement payload)
			throw new IOException("LDraw MCP instance pipe did not return a response payload.");

		// Each pipe command has a single strongly typed payload, deserialized by the broker-side method that issued the command.
		return payload.Deserialize<T>(McpJson.LineOptions) ?? throw new IOException("LDraw MCP instance pipe returned an invalid response payload.");
	}
}
