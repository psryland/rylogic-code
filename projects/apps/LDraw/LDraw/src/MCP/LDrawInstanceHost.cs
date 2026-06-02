using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.IO.Pipes;
using System.Linq;
using System.Text.Json;
using System.Threading;
using System.Threading.Tasks;
using LDraw.UI;
using Rylogic.Common;
using Rylogic.Gfx;
using Rylogic.Maths;
using Rylogic.Utility;

namespace LDraw.MCP;

/// <summary>Hosts the local control pipe and registry entry for this LDraw process</summary>
internal sealed partial class LDrawInstanceHost :IDisposable
{
	private readonly Model m_model;
	private readonly InstanceRegistry m_registry;
	private readonly InstanceRegistration m_registration;
	private readonly SemaphoreSlim m_overlay_gate;
	private readonly object m_overlay_lock = new();
	private readonly object m_chart_lock = new();
	private readonly object m_listen_pipe_lock = new();
	private readonly Dictionary<string, OverlayState> m_overlays;
	private readonly Dictionary<string, ChartState> m_charts;
	private NamedPipeServerStream? m_listen_pipe;
	private CancellationTokenSource? m_shutdown;
	private Task? m_server_task;
	private Timer? m_heartbeat;
	private bool m_published;

	/// <summary>Create the per-process host that the broker can query</summary>
	public LDrawInstanceHost(Model model, InstanceRegistry registry, string launch_nonce)
	{
		m_model = model;
		m_registry = registry;
		m_overlay_gate = new SemaphoreSlim(1, 1);
		m_overlays = new Dictionary<string, OverlayState>(StringComparer.OrdinalIgnoreCase);
		m_charts = new Dictionary<string, ChartState>(StringComparer.OrdinalIgnoreCase);

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
			LaunchNonce = launch_nonce,
			ProtocolVersion = McpProtocol.ProtocolVersion,
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

		// The pipe accepts read-only requests from the broker; the heartbeat keeps the registry entry fresh for discovery.
		// The registry entry is published from inside the server loop once the pipe is actually accepting connections,
		// so the host never discovers this instance before it can answer (see the registry-before-pipe-ready race).
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
		DisposeListenPipe();
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
		m_overlay_gate.Dispose();
	}

	/// <summary>Track the named pipe currently waiting for a client connection</summary>
	private void SetListenPipe(NamedPipeServerStream pipe)
	{
		lock (m_listen_pipe_lock)
		{
			m_listen_pipe = pipe;
		}
	}

	/// <summary>Stop tracking 'pipe' if it is still the current listener</summary>
	private void ClearListenPipe(NamedPipeServerStream pipe)
	{
		lock (m_listen_pipe_lock)
		{
			if (ReferenceEquals(m_listen_pipe, pipe))
				m_listen_pipe = null;
		}
	}

	/// <summary>Dispose the listening pipe to unblock a pending WaitForConnectionAsync during shutdown</summary>
	private void DisposeListenPipe()
	{
		NamedPipeServerStream? pipe;
		lock (m_listen_pipe_lock)
		{
			pipe = m_listen_pipe;
			m_listen_pipe = null;
		}

		pipe?.Dispose();
	}

	/// <summary>Refresh this instance's registry file once it has been published</summary>
	private void TouchRegistration()
	{
		// Do not publish from the heartbeat before the pipe is accepting; the server loop owns first publication.
		if (!m_published)
			return;

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
		try
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
					SetListenPipe(pipe);

					// Publish the registry entry only now that a listener exists and is about to accept, so the host
					// never selects this instance before its pipe can answer. Publication happens exactly once.
					if (!m_published)
					{
						m_registry.Write(m_registration);
						m_published = true;
					}

					await pipe.WaitForConnectionAsync(cancellation_token).ConfigureAwait(false);
					ClearListenPipe(pipe);

					// Hand the connected pipe to a worker and immediately listen for the next broker request.
					_ = Task.Run(() => HandleConnectionAsync(pipe, cancellation_token));
				}
				catch (OperationCanceledException)
				{
					ClearListenPipe(pipe);
					pipe.Dispose();
					break;
				}
				catch (Exception) when (cancellation_token.IsCancellationRequested)
				{
					ClearListenPipe(pipe);
					pipe.Dispose();
					break;
				}
				catch (Exception ex)
				{
					ClearListenPipe(pipe);
					pipe.Dispose();
					Log.Write(ELogLevel.Warn, ex, "LDraw MCP instance pipe accept failed.");
					await Task.Delay(TimeSpan.FromSeconds(1), cancellation_token).ConfigureAwait(false);
				}
			}
		}
		finally
		{
			// If the accept loop ever exits while still published (e.g. an unrecoverable fault rather than a
			// clean shutdown), unpublish so the host stops discovering an instance whose pipe is gone.
			if (m_published && !cancellation_token.IsCancellationRequested)
			{
				m_published = false;
				try { m_registry.Delete(InstanceId); } catch { /* best effort */ }
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
				case InstancePipeCommands.Ping:
				{
					response.Success = true;
					response.Payload = await PayloadAsync(PingAsync()).ConfigureAwait(false);
					break;
				}
				case InstancePipeCommands.GetSceneSummary:
				{
					response.Success = true;
					response.Payload = await PayloadAsync(CreateSceneSummaryAsync()).ConfigureAwait(false);
					break;
				}
				case InstancePipeCommands.ListScenes:
				{
					response.Success = true;
					response.Payload = await PayloadAsync(ListScenesAsync(Parameters<LDrawListScenesParams>(request))).ConfigureAwait(false);
					break;
				}
				case InstancePipeCommands.CreateScene:
				{
					response.Success = true;
					response.Payload = await PayloadAsync(CreateSceneAsync(Parameters<LDrawCreateSceneParams>(request))).ConfigureAwait(false);
					break;
				}
				case InstancePipeCommands.CloseScene:
				{
					response.Success = true;
					response.Payload = await PayloadAsync(CloseSceneAsync(Parameters<LDrawSceneParams>(request))).ConfigureAwait(false);
					break;
				}
				case InstancePipeCommands.SwitchScene:
				{
					response.Success = true;
					response.Payload = await PayloadAsync(SwitchSceneAsync(Parameters<LDrawSceneParams>(request))).ConfigureAwait(false);
					break;
				}
				case InstancePipeCommands.ClearScene:
				{
					response.Success = true;
					response.Payload = await PayloadAsync(ClearSceneAsync(Parameters<LDrawSceneParams>(request))).ConfigureAwait(false);
					break;
				}
				case InstancePipeCommands.RenameScene:
				{
					response.Success = true;
					response.Payload = await PayloadAsync(RenameSceneAsync(Parameters<LDrawRenameSceneParams>(request))).ConfigureAwait(false);
					break;
				}
				case InstancePipeCommands.ListSources:
				{
					response.Success = true;
					response.Payload = await PayloadAsync(ListSourcesAsync(Parameters<LDrawListSourcesParams>(request))).ConfigureAwait(false);
					break;
				}
				case InstancePipeCommands.LoadSource:
				{
					response.Success = true;
					response.Payload = await PayloadAsync(LoadSourceAsync(Parameters<LDrawLoadSourceParams>(request))).ConfigureAwait(false);
					break;
				}
				case InstancePipeCommands.ReloadSource:
				{
					response.Success = true;
					response.Payload = await PayloadAsync(ReloadSourceAsync(Parameters<LDrawSourceParams>(request))).ConfigureAwait(false);
					break;
				}
				case InstancePipeCommands.RemoveSource:
				{
					response.Success = true;
					response.Payload = await PayloadAsync(RemoveSourceAsync(Parameters<LDrawSourceParams>(request))).ConfigureAwait(false);
					break;
				}
				case InstancePipeCommands.SetSourceScenes:
				{
					response.Success = true;
					response.Payload = await PayloadAsync(SetSourceScenesAsync(Parameters<LDrawSetSourceScenesParams>(request))).ConfigureAwait(false);
					break;
				}
				case InstancePipeCommands.SetSceneSources:
				{
					response.Success = true;
					response.Payload = await PayloadAsync(SetSceneSourcesAsync(Parameters<LDrawSetSceneSourcesParams>(request))).ConfigureAwait(false);
					break;
				}
				case InstancePipeCommands.GetViewSettings:
				{
					response.Success = true;
					response.Payload = await PayloadAsync(GetViewSettingsAsync(Parameters<LDrawViewSettingsParams>(request))).ConfigureAwait(false);
					break;
				}
				case InstancePipeCommands.GetAnimation:
				{
					response.Success = true;
					response.Payload = await PayloadAsync(GetAnimationAsync(Parameters<LDrawAnimationParams>(request))).ConfigureAwait(false);
					break;
				}
				case InstancePipeCommands.ControlAnimation:
				{
					response.Success = true;
					response.Payload = await PayloadAsync(ControlAnimationAsync(Parameters<LDrawAnimationControlParams>(request))).ConfigureAwait(false);
					break;
				}
				case InstancePipeCommands.ListViewPresets:
				{
					response.Success = true;
					response.Payload = await PayloadAsync(ListViewPresetsAsync(Parameters<LDrawViewPresetParams>(request))).ConfigureAwait(false);
					break;
				}
				case InstancePipeCommands.SetViewPreset:
				{
					response.Success = true;
					response.Payload = await PayloadAsync(SetViewPresetAsync(Parameters<LDrawViewPresetParams>(request))).ConfigureAwait(false);
					break;
				}
				case InstancePipeCommands.ListSavedViews:
				{
					response.Success = true;
					response.Payload = await PayloadAsync(ListSavedViewsAsync(Parameters<LDrawSavedViewParams>(request))).ConfigureAwait(false);
					break;
				}
				case InstancePipeCommands.SaveView:
				{
					response.Success = true;
					response.Payload = await PayloadAsync(SaveViewAsync(Parameters<LDrawSavedViewParams>(request))).ConfigureAwait(false);
					break;
				}
				case InstancePipeCommands.ApplySavedView:
				{
					response.Success = true;
					response.Payload = await PayloadAsync(ApplySavedViewAsync(Parameters<LDrawSavedViewParams>(request))).ConfigureAwait(false);
					break;
				}
				case InstancePipeCommands.RemoveSavedView:
				{
					response.Success = true;
					response.Payload = await PayloadAsync(RemoveSavedViewAsync(Parameters<LDrawSavedViewParams>(request))).ConfigureAwait(false);
					break;
				}
				case InstancePipeCommands.GetStreamingState:
				{
					response.Success = true;
					response.Payload = await PayloadAsync(GetStreamingStateAsync()).ConfigureAwait(false);
					break;
				}
				case InstancePipeCommands.SetStreamingState:
				{
					response.Success = true;
					response.Payload = await PayloadAsync(SetStreamingStateAsync(Parameters<LDrawStreamingControlParams>(request))).ConfigureAwait(false);
					break;
				}
				case InstancePipeCommands.SetProjection:
				{
					response.Success = true;
					response.Payload = await PayloadAsync(SetProjectionAsync(Parameters<LDrawSetProjectionParams>(request))).ConfigureAwait(false);
					break;
				}
				case InstancePipeCommands.SetBackgroundColour:
				{
					response.Success = true;
					response.Payload = await PayloadAsync(SetBackgroundColourAsync(Parameters<LDrawSetBackgroundColourParams>(request))).ConfigureAwait(false);
					break;
				}
				case InstancePipeCommands.SetCameraAlignAxis:
				{
					response.Success = true;
					response.Payload = await PayloadAsync(SetCameraAlignAxisAsync(Parameters<LDrawSetCameraAlignAxisParams>(request))).ConfigureAwait(false);
					break;
				}
				case InstancePipeCommands.SetAxisRanges:
				{
					response.Success = true;
					response.Payload = await PayloadAsync(SetAxisRangesAsync(Parameters<LDrawSetAxisRangesParams>(request), "set_axis_ranges")).ConfigureAwait(false);
					break;
				}
				case InstancePipeCommands.GetDiagnosticModes:
				{
					response.Success = true;
					response.Payload = await PayloadAsync(GetDiagnosticModesAsync(Parameters<LDrawDiagnosticModesParams>(request))).ConfigureAwait(false);
					break;
				}
				case InstancePipeCommands.SetDiagnosticModes:
				{
					response.Success = true;
					response.Payload = await PayloadAsync(SetDiagnosticModesAsync(Parameters<LDrawSetDiagnosticModesParams>(request))).ConfigureAwait(false);
					break;
				}
				case InstancePipeCommands.SetRenderSettings:
				{
					response.Success = true;
					response.Payload = await PayloadAsync(SetRenderSettingsAsync(Parameters<LDrawSetRenderSettingsParams>(request))).ConfigureAwait(false);
					break;
				}
				case InstancePipeCommands.ListObjects:
				{
					response.Success = true;
					response.Payload = await PayloadAsync(ListObjectsAsync(Parameters<LDrawListObjectsParams>(request))).ConfigureAwait(false);
					break;
				}
				case InstancePipeCommands.GetCamera:
				{
					response.Success = true;
					response.Payload = await PayloadAsync(GetCameraAsync(Parameters<LDrawCameraParams>(request))).ConfigureAwait(false);
					break;
				}
				case InstancePipeCommands.SetCamera:
				{
					response.Success = true;
					response.Payload = await PayloadAsync(SetCameraAsync(Parameters<LDrawSetCameraParams>(request))).ConfigureAwait(false);
					break;
				}
				case InstancePipeCommands.FindObjects:
				{
					response.Success = true;
					response.Payload = await PayloadAsync(FindObjectsAsync(Parameters<LDrawObjectQueryParams>(request))).ConfigureAwait(false);
					break;
				}
				case InstancePipeCommands.GetObject:
				{
					response.Success = true;
					response.Payload = await PayloadAsync(GetObjectAsync(Parameters<LDrawGetObjectParams>(request))).ConfigureAwait(false);
					break;
				}
				case InstancePipeCommands.SelectObjects:
				{
					response.Success = true;
					response.Payload = await PayloadAsync(SelectObjectsAsync(Parameters<LDrawSelectObjectsParams>(request))).ConfigureAwait(false);
					break;
				}
				case InstancePipeCommands.SetObjectVisibility:
				{
					response.Success = true;
					response.Payload = await PayloadAsync(SetObjectVisibilityAsync(Parameters<LDrawSetObjectVisibilityParams>(request))).ConfigureAwait(false);
					break;
				}
				case InstancePipeCommands.SetObjectColour:
				{
					response.Success = true;
					response.Payload = await PayloadAsync(SetObjectColourAsync(Parameters<LDrawSetObjectColourParams>(request))).ConfigureAwait(false);
					break;
				}
				case InstancePipeCommands.GetObjectTransform:
				{
					response.Success = true;
					response.Payload = await PayloadAsync(GetObjectTransformAsync(Parameters<LDrawGetObjectTransformParams>(request))).ConfigureAwait(false);
					break;
				}
				case InstancePipeCommands.SetObjectTransform:
				{
					response.Success = true;
					response.Payload = await PayloadAsync(SetObjectTransformAsync(Parameters<LDrawSetObjectTransformParams>(request))).ConfigureAwait(false);
					break;
				}
				case InstancePipeCommands.GetObjectRenderState:
				{
					response.Success = true;
					response.Payload = await PayloadAsync(GetObjectRenderStateAsync(Parameters<LDrawGetObjectRenderStateParams>(request))).ConfigureAwait(false);
					break;
				}
				case InstancePipeCommands.SetObjectRenderState:
				{
					response.Success = true;
					response.Payload = await PayloadAsync(SetObjectRenderStateAsync(Parameters<LDrawSetObjectRenderStateParams>(request))).ConfigureAwait(false);
					break;
				}
				case InstancePipeCommands.HitTest:
				{
					response.Success = true;
					response.Payload = await PayloadAsync(HitTestAsync(Parameters<LDrawHitTestParams>(request))).ConfigureAwait(false);
					break;
				}
				case InstancePipeCommands.ShowNormals:
				{
					response.Success = true;
					response.Payload = await PayloadAsync(ShowNormalsAsync(Parameters<LDrawShowNormalsParams>(request))).ConfigureAwait(false);
					break;
				}
				case InstancePipeCommands.ShowObjectBounds:
				{
					response.Success = true;
					response.Payload = await PayloadAsync(ShowObjectBoundsAsync(Parameters<LDrawShowObjectBoundsParams>(request))).ConfigureAwait(false);
					break;
				}
				case InstancePipeCommands.GetSelection:
				{
					response.Success = true;
					response.Payload = await PayloadAsync(GetSelectionAsync(Parameters<LDrawGetSelectionParams>(request))).ConfigureAwait(false);
					break;
				}
				case InstancePipeCommands.FrameObject:
				{
					response.Success = true;
					response.Payload = await PayloadAsync(FrameObjectAsync(Parameters<LDrawFrameObjectParams>(request))).ConfigureAwait(false);
					break;
				}
				case InstancePipeCommands.FrameSelection:
				{
					response.Success = true;
					response.Payload = await PayloadAsync(FrameSelectionAsync(Parameters<LDrawFrameSelectionParams>(request))).ConfigureAwait(false);
					break;
				}
				case InstancePipeCommands.FrameBounds:
				{
					response.Success = true;
					response.Payload = await PayloadAsync(FrameBoundsAsync(Parameters<LDrawFrameBoundsParams>(request))).ConfigureAwait(false);
					break;
				}
				case InstancePipeCommands.CaptureScene:
				{
					response.Success = true;
					response.Payload = await PayloadAsync(CaptureSceneAsync(Parameters<LDrawCaptureSceneParams>(request))).ConfigureAwait(false);
					break;
				}
				case InstancePipeCommands.GetChartDisplayOptions:
				{
					response.Success = true;
					response.Payload = await PayloadAsync(GetChartDisplayOptionsAsync(Parameters<LDrawChartDisplayOptionsParams>(request))).ConfigureAwait(false);
					break;
				}
				case InstancePipeCommands.SetChartDisplayOptions:
				{
					response.Success = true;
					response.Payload = await PayloadAsync(SetChartDisplayOptionsAsync(Parameters<LDrawSetChartDisplayOptionsParams>(request))).ConfigureAwait(false);
					break;
				}
				case InstancePipeCommands.ChartCreate:
				{
					response.Success = true;
					response.Payload = await PayloadAsync(ChartCreateAsync(Parameters<LDrawChartCreateParams>(request))).ConfigureAwait(false);
					break;
				}
				case InstancePipeCommands.ChartUpdateData:
				{
					response.Success = true;
					response.Payload = await PayloadAsync(ChartUpdateDataAsync(Parameters<LDrawChartUpdateDataParams>(request))).ConfigureAwait(false);
					break;
				}
				case InstancePipeCommands.ChartAddSeries:
				{
					response.Success = true;
					response.Payload = await PayloadAsync(ChartAddSeriesAsync(Parameters<LDrawChartAddSeriesParams>(request))).ConfigureAwait(false);
					break;
				}
				case InstancePipeCommands.ChartSetAxisRanges:
				{
					response.Success = true;
					response.Payload = await PayloadAsync(ChartSetAxisRangesAsync(Parameters<LDrawChartSetAxisRangesParams>(request))).ConfigureAwait(false);
					break;
				}
				case InstancePipeCommands.ChartClear:
				{
					response.Success = true;
					response.Payload = await PayloadAsync(ChartClearAsync(Parameters<LDrawChartClearParams>(request))).ConfigureAwait(false);
					break;
				}
				case InstancePipeCommands.OverlaySetScript:
				{
					response.Success = true;
					response.Payload = await PayloadAsync(SetOverlayScriptAsync(Parameters<LDrawOverlayScriptParams>(request), append: false)).ConfigureAwait(false);
					break;
				}
				case InstancePipeCommands.OverlayAppendScript:
				{
					response.Success = true;
					response.Payload = await PayloadAsync(SetOverlayScriptAsync(Parameters<LDrawOverlayScriptParams>(request), append: true)).ConfigureAwait(false);
					break;
				}
				case InstancePipeCommands.OverlayClear:
				{
					response.Success = true;
					response.Payload = await PayloadAsync(ClearOverlayAsync(Parameters<LDrawOverlayClearParams>(request))).ConfigureAwait(false);
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

	/// <summary>Deserialize the typed parameters from a pipe request</summary>
	private static T Parameters<T>(InstancePipeRequest request)
		where T : new()
	{
		if (request.Parameters is not JsonElement parameters || parameters.ValueKind is JsonValueKind.Null or JsonValueKind.Undefined)
			return new T();

		return parameters.Deserialize<T>(McpJson.LineOptions) ?? new T();
	}

	/// <summary>Serialize a typed command result into a pipe response payload</summary>
	private static async Task<JsonElement> PayloadAsync<T>(Task<T> payload)
	{
		var value = await payload.ConfigureAwait(false);
		return JsonSerializer.SerializeToElement(value, McpJson.LineOptions);
	}

	/// <summary>Answer a readiness/identity ping from the host</summary>
	private Task<LDrawPingResult> PingAsync()
	{
		// The ping does not touch View3D, so it can answer immediately. The host uses it to confirm the pipe is
		// live, to match the launch nonce of an instance it auto-launched, and to detect a protocol mismatch.
		var version = typeof(LDrawInstanceHost).Assembly.GetName().Version?.ToString() ?? string.Empty;
		return Task.FromResult(new LDrawPingResult
		{
			InstanceId = InstanceId,
			LaunchNonce = m_registration.LaunchNonce,
			ProtocolVersion = McpProtocol.ProtocolVersion,
			AppVersion = version,
		});
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
					..m_model.Sources.Select(CreateSourceSummary),
				],
				Scenes =
				[
					..m_model.Scenes.Select(CreateSceneInfo),
				],
				Overlays = OverlayInfos(),
			};
			return summary;
		}, TimeSpan.FromSeconds(2));
	}

	/// <summary>Create a read-only source summary DTO</summary>
	private static LDrawSourceSummary CreateSourceSummary(Source source)
	{
		return new LDrawSourceSummary
		{
			ContextId = source.ContextId.ToString("D"),
			Name = source.Name,
			FilePath = source.FilePath,
			ObjectCount = source.ObjectCount,
			IsLoading = source.IsLoading,
			LoadFraction = source.LoadFraction,
			SelectedScenes = [..source.SelectedScenes.Select(scene => scene.SceneName)],
		};
	}

	/// <summary>Create a read-only scene summary DTO</summary>
	private LDrawSceneInfo CreateSceneInfo(SceneUI scene)
	{
		// The scene stores context ids in the native View3D window, not in the WPF SceneUI wrapper.
		var context_ids = SceneContextIds(scene);
		var context_id_set = context_ids.ToHashSet();
		var window = scene.SceneView.Scene.Window;
		var sources = m_model.Sources.Where(source => context_id_set.Contains(source.ContextId)).ToArray();
		return new LDrawSceneInfo
		{
			Name = scene.SceneName,
			IsActive = scene.DockControl.IsActiveContent,
			IsVisible = scene.DockControl.IsVisible,
			ObjectCount = window.ObjectCount,
			ContextIds = [..context_ids.Select(context_id => context_id.ToString("D"))],
			Sources = [..sources.Select(CreateSourceSummary)],
			Bounds = LDrawBoundsInfo.From(window.SceneBounds(View3d.ESceneBounds.Visible)),
			Camera = CreateCameraInfo(scene),
			CanCloseByMcp = m_model.Scenes.Count > 1 && sources.Length == 0,
		};
	}

	/// <summary>Return object summaries for the requested scene</summary>
	private Task<LDrawObjectList> ListObjectsAsync(LDrawListObjectsParams parameters)
	{
		return m_model.InvokeAsync(() =>
		{
			var scene = ResolveScene(parameters.SceneName);
			var max_count = Math.Clamp(parameters.MaxCount <= 0 ? 200 : parameters.MaxCount, 1, 1000);
			var context_id = ParseContextId(parameters.ContextId);
			var entries = EnumerateObjects(scene, context_id, parameters.IncludeChildren);
			var truncated = entries.Count > max_count;

			// List commands share the same object identity and hierarchy metadata as the richer query commands.
			var objects = CreateObjectInfos(entries.Take(max_count));

			return new LDrawObjectList
			{
				SceneName = scene.SceneName,
				Objects = objects,
				Truncated = truncated,
			};
		}, TimeSpan.FromSeconds(5));
	}

	/// <summary>Return the camera state for the requested scene</summary>
	private Task<LDrawCameraInfo> GetCameraAsync(LDrawCameraParams parameters)
	{
		return m_model.InvokeAsync(() =>
		{
			var scene = ResolveScene(parameters.SceneName);
			return CreateCameraInfo(scene);
		}, TimeSpan.FromSeconds(5));
	}

	/// <summary>Change the camera state for the requested scene</summary>
	private Task<LDrawCameraSetResult> SetCameraAsync(LDrawSetCameraParams parameters)
	{
		return m_model.InvokeAsync(() =>
		{
			var scene = ResolveScene(parameters.SceneName);
			var camera = scene.SceneView.Scene.Window.Camera;
			var action = "look_at";

			// Framing uses the current visible bounds, matching the normal user-facing "auto range" behaviour.
			if (parameters.FrameScene)
			{
				var bounds = scene.SceneView.Scene.Window.SceneBounds(View3d.ESceneBounds.Visible);
				if (bounds.IsValid)
					camera.ResetView(bounds);
				else
					camera.ResetView();
				action = "frame_scene";
			}
			else
			{
				if (parameters.Position == null || parameters.LookAt == null)
					throw new InvalidOperationException("Set-camera look-at mode requires 'position' and 'look_at'.");

				var up = parameters.Up?.ToV4(0f) ?? (camera.AlignAxis.LengthSq != 0f ? camera.AlignAxis : v4.ZAxis);
				camera.Lookat(parameters.Position.ToV4(1f), parameters.LookAt.ToV4(1f), up);
				camera.Commit();
			}

			scene.SceneView.Invalidate();
			return new LDrawCameraSetResult
			{
				SceneName = scene.SceneName,
				Action = action,
				Camera = CreateCameraInfo(scene),
			};
		}, TimeSpan.FromSeconds(5));
	}

	/// <summary>Replace or append raw script in a named transient overlay</summary>
	private async Task<LDrawOverlayResult> SetOverlayScriptAsync(LDrawOverlayScriptParams parameters, bool append)
	{
		await m_overlay_gate.WaitAsync().ConfigureAwait(false);
		try
		{
			var overlay_id = NormaliseOverlayId(parameters.OverlayId);
			var name = string.IsNullOrWhiteSpace(parameters.Name) ? $"MCP Overlay - {overlay_id}" : parameters.Name.Trim();
			var state = OverlayStateFor(overlay_id, name);
			var script = append ? $"{state.Script}{Environment.NewLine}{parameters.Script}" : parameters.Script;
			if (string.IsNullOrWhiteSpace(script))
				throw new InvalidOperationException("Overlay script cannot be empty.");

			var result = await LoadOverlayScriptAsync(state, name, script, parameters.SceneNames, parameters.ResetView, append ? "append" : "set").ConfigureAwait(false);
			lock (m_overlay_lock)
			{
				state.Name = name;
				state.Script = script;
				state.SceneNames = result.Overlay.SceneNames;
				m_overlays[overlay_id] = state;
			}
			return result;
		}
		finally
		{
			m_overlay_gate.Release();
		}
	}

	/// <summary>Clear one named transient overlay, or all overlays when no id is supplied</summary>
	private async Task<LDrawOverlayResult[]> ClearOverlayAsync(LDrawOverlayClearParams parameters)
	{
		await m_overlay_gate.WaitAsync().ConfigureAwait(false);
		try
		{
			var states = OverlayStatesForClear(parameters.OverlayId);
			if (states.Length == 0)
				return [];

			return await m_model.InvokeAsync(() =>
			{
				var results = new List<LDrawOverlayResult>(states.Length);
				foreach (var state in states)
				{
					// Remove both window membership and the underlying source so repeated overlay edits do not leave zombie sources in View3D.
					m_model.Clear(m_model.Scenes, state.ContextId);
					using var source = new View3d.Source(state.ContextId, m_model.View3d);
					source.Remove();
					View3d.ObjectManager.ExcludeCtxIds.Remove(state.ContextId);

					results.Add(new LDrawOverlayResult
					{
						Action = "clear",
						Overlay = state.ToInfo(),
						ObjectCount = 0,
					});
				}

				lock (m_overlay_lock)
				{
					foreach (var state in states)
						m_overlays.Remove(state.OverlayId);
				}
				RemoveChartStatesForOverlayIds(states.Select(state => state.OverlayId));
				return results.ToArray();
			}, TimeSpan.FromSeconds(10)).ConfigureAwait(false);
		}
		finally
		{
			m_overlay_gate.Release();
		}
	}

	/// <summary>Load an overlay script and wait until the objects have been added to the requested scenes</summary>
	private async Task<LDrawOverlayResult> LoadOverlayScriptAsync(OverlayState state, string name, string script, IReadOnlyList<string> scene_names, bool reset_view, string action)
	{
		var setup = await m_model.InvokeAsync(() =>
		{
			// Resolve scenes on the UI thread because scene collections and windows belong to WPF/View3D state.
			var target_scenes = ResolveScenes(scene_names);
			var all_scenes = m_model.Scenes.ToArray();
			View3d.ObjectManager.ExcludeCtxIds.Add(state.ContextId);
			return new OverlayLoadSetup(target_scenes, all_scenes);
		}, TimeSpan.FromSeconds(5)).ConfigureAwait(false);

		var completion = new TaskCompletionSource<LDrawOverlayResult>(TaskCreationOptions.RunContinuationsAsynchronously);
		void OnAdd(Guid context_id, bool before)
		{
			_ = m_model.InvokeAsync(() =>
			{
				if (before)
				{
					m_model.Clear(setup.AllScenes, context_id);
					return 0;
				}

				// Name the generated source and then add it only to the scenes requested by the MCP command.
				using var source = new View3d.Source(context_id, m_model.View3d);
				source.Name = name;
				m_model.AddObjects(setup.TargetScenes, context_id, reset_view);

				completion.TrySetResult(new LDrawOverlayResult
				{
					Action = action,
					Overlay = state.ToInfo(name, script, setup.TargetScenes.Select(x => x.SceneName)),
					ObjectCount = source.Info.ObjectCount,
				});
				return 0;
			}, TimeSpan.FromSeconds(10)).ContinueWith(task =>
			{
				if (task.Exception != null)
					completion.TrySetException(task.Exception.InnerException ?? task.Exception);
			}, TaskScheduler.Default);
		}

		try
		{
			// The native loader owns the parsing work and reports completion through OnAdd.
			await Task.Run(() => m_model.View3d.LoadScriptFromString(script, state.ContextId, add_completed: OnAdd)).ConfigureAwait(false);
			return await completion.Task.WaitAsync(TimeSpan.FromSeconds(30)).ConfigureAwait(false);
		}
		catch
		{
			if (!OverlayExists(state.OverlayId))
				View3d.ObjectManager.ExcludeCtxIds.Remove(state.ContextId);
			throw;
		}
	}

	/// <summary>Resolve a scene by name, defaulting to the first scene</summary>
	private SceneUI ResolveScene(string? scene_name)
	{
		if (m_model.Scenes.Count == 0)
			throw new InvalidOperationException("No LDraw scenes are open.");
		if (string.IsNullOrWhiteSpace(scene_name))
			return m_model.Scenes[0];

		return m_model.Scenes.FirstOrDefault(x => string.Equals(x.SceneName, scene_name, StringComparison.OrdinalIgnoreCase))
			?? throw new InvalidOperationException($"No LDraw scene named '{scene_name}' exists.");
	}

	/// <summary>Resolve scene names, defaulting to the first scene and allowing "*" for all scenes</summary>
	private SceneUI[] ResolveScenes(IReadOnlyList<string>? scene_names)
	{
		if (m_model.Scenes.Count == 0)
			throw new InvalidOperationException("No LDraw scenes are open.");
		if (scene_names == null || scene_names.Count == 0)
			return [m_model.Scenes[0]];
		if (scene_names.Count == 1 && scene_names[0] == "*")
			return [..m_model.Scenes];

		// Resolve each requested scene independently so the error message identifies the missing name.
		return [..scene_names.Select(ResolveScene)];
	}

	/// <summary>Parse an optional context id filter</summary>
	private static Guid? ParseContextId(string? context_id)
	{
		if (string.IsNullOrWhiteSpace(context_id))
			return null;
		if (!Guid.TryParse(context_id, out var id))
			throw new InvalidOperationException($"'{context_id}' is not a valid context id.");
		return id;
	}

	/// <summary>Create camera DTO data for a scene</summary>
	private static LDrawCameraInfo CreateCameraInfo(SceneUI scene)
	{
		var camera = scene.SceneView.Scene.Window.Camera;
		var o2w = camera.O2W;
		var fov = camera.Fov;
		return new LDrawCameraInfo
		{
			SceneName = scene.SceneName,
			Position = LDrawVector3.From(o2w.pos),
			FocusPoint = LDrawVector3.From(camera.FocusPoint),
			FocusDistance = camera.FocusDist,
			Up = LDrawVector3.From(o2w.y),
			Forward = LDrawVector3.From(-o2w.z),
			Fov = new LDrawVector3 { X = fov.x, Y = fov.y, Z = 0 },
			Orthographic = camera.Orthographic,
			CameraToWorld = [..o2w.ToArray().Select(x => (double)x)],
		};
	}

	/// <summary>Return the overlay infos visible to MCP clients</summary>
	private List<LDrawOverlayInfo> OverlayInfos()
	{
		lock (m_overlay_lock)
			return [..m_overlays.Values.Select(x => x.ToInfo())];
	}

	/// <summary>Return true if an overlay id is currently registered</summary>
	private bool OverlayExists(string overlay_id)
	{
		lock (m_overlay_lock)
			return m_overlays.ContainsKey(overlay_id);
	}

	/// <summary>Return an existing overlay state or create a new one</summary>
	private OverlayState OverlayStateFor(string overlay_id, string name)
	{
		lock (m_overlay_lock)
		{
			if (m_overlays.TryGetValue(overlay_id, out var state))
				return state;

			return new OverlayState(overlay_id, name, Guid.NewGuid());
		}
	}

	/// <summary>Return the overlay states targeted by a clear command</summary>
	private OverlayState[] OverlayStatesForClear(string? overlay_id)
	{
		lock (m_overlay_lock)
		{
			if (string.IsNullOrWhiteSpace(overlay_id))
				return [..m_overlays.Values];

			var id = NormaliseOverlayId(overlay_id);
			if (!m_overlays.TryGetValue(id, out var state))
				throw new InvalidOperationException($"No MCP overlay named '{id}' exists.");

			return [state];
		}
	}

	/// <summary>Normalise and validate a caller-visible overlay id</summary>
	private static string NormaliseOverlayId(string? overlay_id)
	{
		var id = string.IsNullOrWhiteSpace(overlay_id) ? "default" : overlay_id.Trim();
		if (id.Length > 64 || id.Any(char.IsControl))
			throw new InvalidOperationException("Overlay id must be 1-64 non-control characters.");
		return id;
	}

	/// <summary>State for one transient MCP overlay source</summary>
	private sealed class OverlayState
	{
		/// <summary>Create overlay state for a generated source context</summary>
		public OverlayState(string overlay_id, string name, Guid context_id)
		{
			OverlayId = overlay_id;
			Name = name;
			ContextId = context_id;
			Script = string.Empty;
			SceneNames = [];
		}

		/// <summary>Caller-visible overlay id</summary>
		public string OverlayId { get; }

		/// <summary>View3D source context id</summary>
		public Guid ContextId { get; }

		/// <summary>Display name for the generated source</summary>
		public string Name { get; set; }

		/// <summary>The raw script currently loaded for this overlay</summary>
		public string Script { get; set; }

		/// <summary>The scenes that currently show this overlay</summary>
		public List<string> SceneNames { get; set; }

		/// <summary>Create the public DTO for this overlay</summary>
		public LDrawOverlayInfo ToInfo()
		{
			return ToInfo(Name, Script, SceneNames);
		}

		/// <summary>Create the public DTO for this overlay using pending state</summary>
		public LDrawOverlayInfo ToInfo(string name, string script, IEnumerable<string> scene_names)
		{
			return new LDrawOverlayInfo
			{
				OverlayId = OverlayId,
				ContextId = ContextId.ToString("D"),
				Name = name,
				SceneNames = [..scene_names],
				ScriptLength = script.Length,
			};
		}
	}

	/// <summary>Scene sets captured before starting an overlay load</summary>
	private sealed record OverlayLoadSetup(SceneUI[] TargetScenes, SceneUI[] AllScenes);
}
