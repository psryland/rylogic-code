using System;
using System.Diagnostics;
using System.IO;
using System.IO.Pipes;
using System.Linq;
using System.Text.Json;
using System.Threading;
using System.Threading.Tasks;
using Rylogic.Common;
using Rylogic.Utility;

namespace LDraw.MCP;

/// <summary>Hosts the local control pipe and registry entry for this LDraw process</summary>
internal sealed class LDrawInstanceHost :IDisposable
{
	private readonly Model m_model;
	private readonly InstanceRegistry m_registry;
	private readonly InstanceRegistration m_registration;
	private CancellationTokenSource? m_shutdown;
	private Task? m_server_task;
	private Timer? m_heartbeat;

	/// <summary>Create the per-process host that the broker can query</summary>
	public LDrawInstanceHost(Model model, InstanceRegistry registry)
	{
		m_model = model;
		m_registry = registry;

		// Use a fresh id for each process lifetime so stale registry files cannot alias a restarted process.
		using var process = Process.GetCurrentProcess();
		InstanceId = Guid.NewGuid().ToString("N");
		m_registration = new InstanceRegistration
		{
			InstanceId = InstanceId,
			ProcessId = process.Id,
			ProcessName = process.ProcessName,
			StartedUtc = DateTimeOffset.UtcNow,
			PipeName = $"LDraw.MCP.{InstanceId}",
			SettingsPath = model.StartupOptions.SettingsPath,
		};
	}

	/// <summary>The id for this running LDraw instance</summary>
	public string InstanceId { get; }

	/// <summary>Start the local instance host</summary>
	public void Start()
	{
		if (m_shutdown != null)
			return;

		m_shutdown = new CancellationTokenSource();
		m_registry.Write(m_registration);

		// The pipe accepts read-only requests from the broker; the heartbeat keeps the registry entry fresh for discovery.
		m_server_task = Task.Run(() => RunPipeServerAsync(m_shutdown.Token));
		m_heartbeat = new Timer(_ => TouchRegistration(), null, TimeSpan.FromSeconds(10), TimeSpan.FromSeconds(10));
	}

	/// <summary>Stop the local instance host</summary>
	public async Task StopAsync()
	{
		if (m_shutdown == null)
			return;

		m_heartbeat?.Dispose();
		m_heartbeat = null;

		m_shutdown.Cancel();
		try
		{
			// Wait for the accept loop to unwind so the registry file is removed after the pipe stops accepting clients.
			if (m_server_task != null)
				await m_server_task.ConfigureAwait(false);
		}
		catch (OperationCanceledException)
		{}
		finally
		{
			Util.Dispose(ref m_shutdown);
			m_server_task = null;
			m_registry.Delete(InstanceId);
		}
	}

	/// <summary>Dispose of this instance host</summary>
	public void Dispose()
	{
		StopAsync().GetAwaiter().GetResult();
	}

	/// <summary>Refresh this instance's registry file</summary>
	private void TouchRegistration()
	{
		try
		{
			m_registry.Write(m_registration);
		}
		catch (Exception ex)
		{
			Log.Write(ELogLevel.Warn, ex, "Failed to refresh the LDraw MCP instance registration.");
		}
	}

	/// <summary>Accept named-pipe connections until cancellation</summary>
	private async Task RunPipeServerAsync(CancellationToken cancellation_token)
	{
		for (; !cancellation_token.IsCancellationRequested;)
		{
			// CurrentUserOnly prevents other Windows users on the machine from querying private LDraw scene state through this back channel.
			var pipe = new NamedPipeServerStream(
				m_registration.PipeName,
				PipeDirection.InOut,
				NamedPipeServerStream.MaxAllowedServerInstances,
				PipeTransmissionMode.Byte,
				PipeOptions.Asynchronous | PipeOptions.CurrentUserOnly);

			try
			{
				await pipe.WaitForConnectionAsync(cancellation_token).ConfigureAwait(false);

				// Hand the connected pipe to a worker and immediately listen for the next broker request.
				_ = Task.Run(() => HandleConnectionAsync(pipe, cancellation_token), cancellation_token);
			}
			catch (OperationCanceledException)
			{
				pipe.Dispose();
				break;
			}
			catch (Exception ex)
			{
				pipe.Dispose();
				Log.Write(ELogLevel.Warn, ex, "LDraw MCP instance pipe accept failed.");
				await Task.Delay(TimeSpan.FromSeconds(1), cancellation_token).ConfigureAwait(false);
			}
		}
	}

	/// <summary>Handle one named-pipe request</summary>
	private async Task HandleConnectionAsync(NamedPipeServerStream pipe, CancellationToken cancellation_token)
	{
		await using var _ = pipe.ConfigureAwait(false);
		using var reader = new StreamReader(pipe, leaveOpen: true);
		using var writer = new StreamWriter(pipe, leaveOpen: true) { AutoFlush = true, NewLine = "\n" };

		var response = new InstancePipeResponse();
		try
		{
			// The pipe protocol is a single JSON request line followed by a single JSON response line.
			var line = await reader.ReadLineAsync(cancellation_token).ConfigureAwait(false) ?? throw new IOException("Empty LDraw MCP instance pipe request.");
			var request = JsonSerializer.Deserialize<InstancePipeRequest>(line, McpJson.LineOptions) ?? throw new IOException("Invalid LDraw MCP instance pipe request.");
			switch (request.Command)
			{
				case InstancePipeCommands.GetSceneSummary:
				{
					response.Success = true;
					response.SceneSummary = await CreateSceneSummaryAsync().ConfigureAwait(false);
					break;
				}
				default:
				{
					response.Success = false;
					response.Error = $"Unknown LDraw MCP instance pipe command '{request.Command}'.";
					break;
				}
			}
		}
		catch (Exception ex)
		{
			response.Success = false;
			response.Error = ex.Message;
			Log.Write(ELogLevel.Warn, ex, "LDraw MCP instance pipe request failed.");
		}

		await writer.WriteLineAsync(JsonSerializer.Serialize(response, McpJson.LineOptions)).ConfigureAwait(false);
	}

	/// <summary>Create a read-only summary of this LDraw instance</summary>
	private Task<LDrawSceneSummary> CreateSceneSummaryAsync()
	{
		// Sources and scenes are WPF/View3D objects, so capture their state on the UI thread with a bounded wait.
		return m_model.InvokeAsync(() =>
		{
			var summary = new LDrawSceneSummary
			{
				Instance = new McpInstanceInfo
				{
					InstanceId = m_registration.InstanceId,
					ProcessId = m_registration.ProcessId,
					ProcessName = m_registration.ProcessName,
					StartedUtc = m_registration.StartedUtc,
					LastSeenUtc = m_registration.LastSeenUtc,
					SettingsPath = m_registration.SettingsPath,
				},
				Sources =
				[
					..m_model.Sources.Select(source => new LDrawSourceSummary
					{
						ContextId = source.ContextId.ToString("D"),
						Name = source.Name,
						FilePath = source.FilePath,
						ObjectCount = source.ObjectCount,
						IsLoading = source.IsLoading,
						LoadFraction = source.LoadFraction,
						SelectedScenes = [..source.SelectedScenes.Select(scene => scene.SceneName)],
					}),
				],
				Scenes =
				[
					..m_model.Scenes.Select(scene =>
					{
						// The scene stores context ids in the native View3D window, not in the WPF SceneUI wrapper.
						var context_ids = new System.Collections.Generic.List<string>();
						scene.SceneView.Scene.Window.EnumGuids(context_id => context_ids.Add(context_id.ToString("D")));
						return new LDrawSceneInfo
						{
							Name = scene.SceneName,
							ObjectCount = scene.SceneView.Scene.Window.ObjectCount,
							ContextIds = context_ids,
						};
					}),
				],
			};
			return summary;
		}, TimeSpan.FromSeconds(2));
	}
}
