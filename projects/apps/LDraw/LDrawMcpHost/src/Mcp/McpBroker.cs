using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using LDraw.MCP.Host;
using ModelContextProtocol;
using ModelContextProtocol.Server;

namespace LDraw.MCP;

/// <summary>Routes MCP tool calls to registered LDraw instances</summary>
internal sealed partial class McpBroker
{
	private readonly InstanceRegistry m_registry;
	private readonly InstancePipeClient m_client;
	private readonly HostSettings m_settings;
	private readonly InstanceLauncher m_launcher;
	private readonly CancellationToken m_lifetime;
	private readonly SemaphoreSlim m_launch_gate;
	private string? m_last_active_id;
	private DateTimeOffset m_last_launch_failure_utc;

	// Brief back-off after a failed auto-launch so a bad exe path or a flapping LDraw cannot spawn a storm of processes.
	private static readonly TimeSpan LaunchCooldown = TimeSpan.FromSeconds(5);

	/// <summary>Create a broker for routing MCP requests to registered LDraw instances</summary>
	public McpBroker(InstanceRegistry registry, InstancePipeClient client, HostSettings settings, InstanceLauncher launcher, CancellationToken lifetime)
	{
		m_registry = registry;
		m_client = client;
		m_settings = settings;
		m_launcher = launcher;
		m_lifetime = lifetime;
		m_launch_gate = new SemaphoreSlim(1, 1);
		m_last_active_id = null;
		m_last_launch_failure_utc = DateTimeOffset.MinValue;
	}

	/// <summary>Return all live instances registered with the local broker</summary>
	public Task<McpInstanceInfo[]> ListInstancesAsync()
	{
		var instances = m_registry.LiveInstances()
			.Select(ToInfo)
			.ToArray();

		return Task.FromResult(instances);
	}

	/// <summary>Return a scene summary for 'instance_id', or the default instance when omitted</summary>
	public async Task<LDrawSceneSummary> GetSceneSummaryAsync(string? instance_id)
	{
		var registration = await ResolveInstanceAsync(instance_id).ConfigureAwait(false);
		var summary = await m_client.GetSceneSummaryAsync(registration).ConfigureAwait(false);
		summary.Instance = ToInfo(registration);
		return summary;
	}

	/// <summary>Return object summaries for a scene in 'instance_id'</summary>
	public async Task<LDrawObjectList> ListObjectsAsync(string? instance_id, LDrawListObjectsParams parameters)
	{
		var registration = await ResolveInstanceAsync(instance_id).ConfigureAwait(false);
		return await m_client.ListObjectsAsync(registration, parameters).ConfigureAwait(false);
	}

	/// <summary>Return camera state for a scene in 'instance_id'</summary>
	public async Task<LDrawCameraInfo> GetCameraAsync(string? instance_id, LDrawCameraParams parameters)
	{
		var registration = await ResolveInstanceAsync(instance_id).ConfigureAwait(false);
		return await m_client.GetCameraAsync(registration, parameters).ConfigureAwait(false);
	}

	/// <summary>Set camera state for a scene in 'instance_id'</summary>
	public async Task<LDrawCameraSetResult> SetCameraAsync(string? instance_id, LDrawSetCameraParams parameters)
	{
		var registration = await ResolveInstanceAsync(instance_id).ConfigureAwait(false);
		return await m_client.SetCameraAsync(registration, parameters).ConfigureAwait(false);
	}

	/// <summary>Replace a named overlay script in 'instance_id'</summary>
	public async Task<LDrawOverlayResult> OverlaySetScriptAsync(string? instance_id, LDrawOverlayScriptParams parameters)
	{
		var registration = await ResolveInstanceAsync(instance_id).ConfigureAwait(false);
		return await m_client.OverlaySetScriptAsync(registration, parameters).ConfigureAwait(false);
	}

	/// <summary>Append to a named overlay script in 'instance_id'</summary>
	public async Task<LDrawOverlayResult> OverlayAppendScriptAsync(string? instance_id, LDrawOverlayScriptParams parameters)
	{
		var registration = await ResolveInstanceAsync(instance_id).ConfigureAwait(false);
		return await m_client.OverlayAppendScriptAsync(registration, parameters).ConfigureAwait(false);
	}

	/// <summary>Clear one or more named overlays in 'instance_id'</summary>
	public async Task<LDrawOverlayResult[]> OverlayClearAsync(string? instance_id, LDrawOverlayClearParams parameters)
	{
		var registration = await ResolveInstanceAsync(instance_id).ConfigureAwait(false);
		return await m_client.OverlayClearAsync(registration, parameters).ConfigureAwait(false);
	}

	/// <summary>Find an instance registration by id, or pick (and optionally launch) a default target</summary>
	private async Task<InstanceRegistration> ResolveInstanceAsync(string? instance_id)
	{
		var instances = m_registry.LiveInstances();

		// An explicit id always wins and never triggers an auto-launch; the caller chose a specific LDraw process.
		if (!string.IsNullOrWhiteSpace(instance_id))
		{
			var match = instances.FirstOrDefault(x => string.Equals(x.InstanceId, instance_id, StringComparison.OrdinalIgnoreCase))
				?? throw new McpException($"No live LDraw instance with id '{instance_id}' is registered.");

			m_last_active_id = match.InstanceId;
			return match;
		}

		// With no id, route to the host-tracked last-active instance when it is still live, else a deterministic default.
		var chosen = SelectDefault(instances);
		if (chosen != null)
		{
			m_last_active_id = chosen.InstanceId;
			return chosen;
		}

		// No live instance: auto-launch one when enabled, otherwise return a clear, actionable error.
		if (!m_settings.AutoLaunch)
			throw new McpException("No live LDraw instance is registered and auto-launch is disabled. Start LDraw and try again.");

		var launched = await LaunchAndWaitAsync().ConfigureAwait(false);
		m_last_active_id = launched.InstanceId;
		return launched;
	}

	/// <summary>Pick the default instance for an omitted id: the only one, the sticky last-active, or the most-recently-seen</summary>
	private InstanceRegistration? SelectDefault(IReadOnlyList<InstanceRegistration> instances)
	{
		if (instances.Count == 0)
			return null;
		if (instances.Count == 1)
			return instances[0];

		// Prefer the instance the host last routed to, so repeated calls stick to one window when several are open.
		if (m_last_active_id != null)
		{
			var sticky = instances.FirstOrDefault(x => string.Equals(x.InstanceId, m_last_active_id, StringComparison.OrdinalIgnoreCase));
			if (sticky != null)
				return sticky;
		}

		return instances.OrderByDescending(x => x.LastSeenUtc).ThenByDescending(x => x.StartedUtc).First();
	}

	/// <summary>Single-flight launch of LDraw: concurrent callers await one launch, then share the ready instance</summary>
	private async Task<InstanceRegistration> LaunchAndWaitAsync()
	{
		await m_launch_gate.WaitAsync(m_lifetime).ConfigureAwait(false);
		try
		{
			// Another caller may have launched (or a user opened) an instance while we waited for the gate.
			var existing = SelectDefault(m_registry.LiveInstances());
			if (existing != null)
				return existing;

			if (DateTimeOffset.UtcNow - m_last_launch_failure_utc < LaunchCooldown)
				throw new McpException("LDraw is still starting or recently failed to start. Wait a moment and retry.");

			string nonce;
			try
			{
				nonce = m_launcher.Launch();
			}
			catch (Exception ex)
			{
				m_last_launch_failure_utc = DateTimeOffset.UtcNow;
				throw new McpException($"Failed to launch LDraw: {ex.Message}");
			}

			var ready = await WaitForReadyAsync(nonce).ConfigureAwait(false);
			if (ready == null)
			{
				m_last_launch_failure_utc = DateTimeOffset.UtcNow;
				throw new McpException("LDraw was launched but did not become ready in time. Retry shortly.");
			}

			return ready;
		}
		finally
		{
			m_launch_gate.Release();
		}
	}

	/// <summary>Poll the registry for the launched instance (matched by nonce) and confirm readiness with a pipe ping</summary>
	private async Task<InstanceRegistration?> WaitForReadyAsync(string nonce)
	{
		var timeout = TimeSpan.FromSeconds(Math.Clamp(m_settings.LaunchTimeoutSeconds, 5, 300));
		using var cts = CancellationTokenSource.CreateLinkedTokenSource(m_lifetime);
		cts.CancelAfter(timeout);

		try
		{
			while (!cts.IsCancellationRequested)
			{
				// Match by launch nonce so a concurrently-opened, unrelated LDraw is never mistaken for the one we launched.
				var candidate = m_registry.LiveInstances().FirstOrDefault(x => string.Equals(x.LaunchNonce, nonce, StringComparison.OrdinalIgnoreCase));
				if (candidate != null)
				{
					try
					{
						// The registry entry can appear before the pipe accepts connections; ping gates "usable", not just "registered".
						var pong = await m_client.PingAsync(candidate).ConfigureAwait(false);
						if (pong.ProtocolVersion == McpProtocol.ProtocolVersion)
							return candidate;
					}
					catch
					{
						// Pipe not accepting yet, or a transient read error; keep polling until the readiness timeout elapses.
					}
				}

				await Task.Delay(TimeSpan.FromMilliseconds(250), cts.Token).ConfigureAwait(false);
			}
		}
		catch (OperationCanceledException) when (!m_lifetime.IsCancellationRequested)
		{
			// Readiness timed out (not host shutdown); fall through and report a not-ready result to the caller.
		}

		return null;
	}

	/// <summary>Convert a registry entry into public MCP output</summary>
	private static McpInstanceInfo ToInfo(InstanceRegistration registration)
	{
		// The tray host owns the single HTTP endpoint, so no LDraw instance is itself a broker and none expose an endpoint.
		return new McpInstanceInfo
		{
			InstanceId = registration.InstanceId,
			ProcessId = registration.ProcessId,
			ProcessName = registration.ProcessName,
			StartedUtc = registration.StartedUtc,
			LastSeenUtc = registration.LastSeenUtc,
			IsBroker = false,
			Endpoint = string.Empty,
			SettingsPath = registration.SettingsPath,
		};
	}
}

/// <summary>MCP tool surface exposed to AI clients</summary>
[McpServerToolType]
internal sealed partial class LDrawTools
{
	private readonly McpBroker m_broker;

	/// <summary>Create the tool surface bound to a broker</summary>
	public LDrawTools(McpBroker broker)
	{
		m_broker = broker;
	}

	/// <summary>List the running LDraw instances visible to the local broker</summary>
	[McpServerTool(Name = "ldraw_list_instances", Title = "List LDraw instances", ReadOnly = true, Destructive = false, Idempotent = true, OpenWorld = false, UseStructuredContent = true)]
	[Description("Lists running LDraw instances registered with the local MCP broker.")]
	public Task<McpInstanceInfo[]> ListInstances()
	{
		return m_broker.ListInstancesAsync();
	}

	/// <summary>Return a read-only summary of one LDraw instance</summary>
	[McpServerTool(Name = "ldraw_get_scene_summary", Title = "Get LDraw scene summary", ReadOnly = true, Destructive = false, Idempotent = true, OpenWorld = false, UseStructuredContent = true)]
	[Description("Returns sources, scenes, and object counts for a running LDraw instance. Omit instance_id to use the default instance.")]
	public Task<LDrawSceneSummary> GetSceneSummary(
		[Description("The id of a running LDraw instance from ldraw_list_instances. Omit to target the default instance (most-recently-used, or auto-launched when none are running).")] string? instance_id = null)
	{
		return m_broker.GetSceneSummaryAsync(instance_id);
	}

	/// <summary>List objects in a running LDraw scene</summary>
	[McpServerTool(Name = "ldraw_list_objects", Title = "List LDraw objects", ReadOnly = true, Destructive = false, Idempotent = true, OpenWorld = false, UseStructuredContent = true)]
	[Description("Lists objects in a scene, optionally filtered by source context id. Omit scene_name to use the first scene.")]
	public Task<LDrawObjectList> ListObjects(
		[Description("The id of a running LDraw instance from ldraw_list_instances. Omit to target the default instance (most-recently-used, or auto-launched when none are running).")] string? instance_id = null,
		[Description("The scene name to query. Omit to use the first scene.")] string? scene_name = null,
		[Description("Optional source context id filter returned by ldraw_get_scene_summary.")] string? context_id = null,
		[Description("True to recursively include child objects.")] bool include_children = false,
		[Description("Maximum number of objects to return, clamped to 1..1000.")] int max_count = 200)
	{
		var parameters = new LDrawListObjectsParams
		{
			SceneName = scene_name,
			ContextId = context_id,
			IncludeChildren = include_children,
			MaxCount = max_count,
		};
		return m_broker.ListObjectsAsync(instance_id, parameters);
	}

	/// <summary>Return camera state for a running LDraw scene</summary>
	[McpServerTool(Name = "ldraw_get_camera", Title = "Get LDraw camera", ReadOnly = true, Destructive = false, Idempotent = true, OpenWorld = false, UseStructuredContent = true)]
	[Description("Returns camera position, focus point, projection, and camera-to-world matrix for a scene. Omit scene_name to use the first scene.")]
	public Task<LDrawCameraInfo> GetCamera(
		[Description("The id of a running LDraw instance from ldraw_list_instances. Omit to target the default instance (most-recently-used, or auto-launched when none are running).")] string? instance_id = null,
		[Description("The scene name to query. Omit to use the first scene.")] string? scene_name = null)
	{
		var parameters = new LDrawCameraParams
		{
			SceneName = scene_name,
		};
		return m_broker.GetCameraAsync(instance_id, parameters);
	}

	/// <summary>Set or frame the camera for a running LDraw scene</summary>
	[McpServerTool(Name = "ldraw_set_camera", Title = "Set LDraw camera", ReadOnly = false, Destructive = false, Idempotent = true, OpenWorld = false, UseStructuredContent = true)]
	[Description("Frames a scene or sets a look-at camera. Use frame_scene=true to frame visible objects, otherwise provide position_x/y/z and look_at_x/y/z.")]
	public Task<LDrawCameraSetResult> SetCamera(
		[Description("The id of a running LDraw instance from ldraw_list_instances. Omit to target the default instance (most-recently-used, or auto-launched when none are running).")] string? instance_id = null,
		[Description("The scene name to modify. Omit to use the first scene.")] string? scene_name = null,
		[Description("True to frame the scene instead of using look-at coordinates.")] bool frame_scene = false,
		[Description("Camera position X in world space.")] double? position_x = null,
		[Description("Camera position Y in world space.")] double? position_y = null,
		[Description("Camera position Z in world space.")] double? position_z = null,
		[Description("Look-at target X in world space.")] double? look_at_x = null,
		[Description("Look-at target Y in world space.")] double? look_at_y = null,
		[Description("Look-at target Z in world space.")] double? look_at_z = null,
		[Description("Camera up X direction. Optional.")] double? up_x = null,
		[Description("Camera up Y direction. Optional.")] double? up_y = null,
		[Description("Camera up Z direction. Optional.")] double? up_z = null)
	{
		var parameters = new LDrawSetCameraParams
		{
			SceneName = scene_name,
			FrameScene = frame_scene,
			Position = VectorOrNull(position_x, position_y, position_z),
			LookAt = VectorOrNull(look_at_x, look_at_y, look_at_z),
			Up = VectorOrNull(up_x, up_y, up_z),
		};
		return m_broker.SetCameraAsync(instance_id, parameters);
	}

	/// <summary>Replace a named transient MCP overlay with raw LDraw script</summary>
	[McpServerTool(Name = "ldraw_overlay_set_script", Title = "Set LDraw MCP overlay", ReadOnly = false, Destructive = false, Idempotent = true, OpenWorld = false, UseStructuredContent = true)]
	[Description("Replaces a named transient MCP overlay with raw .ldr script. User source documents are not modified.")]
	public Task<LDrawOverlayResult> OverlaySetScript(
		[Description("Raw .ldr script to load into the transient overlay.")] string script,
		[Description("The id of a running LDraw instance from ldraw_list_instances. Omit to target the default instance (most-recently-used, or auto-launched when none are running).")] string? instance_id = null,
		[Description("Overlay id to replace. Omit to use 'default'.")] string? overlay_id = null,
		[Description("Display name for the generated overlay source.")] string? name = null,
		[Description("Scene names to show the overlay in. Omit to use the first scene. Use '*' as the only value for all scenes.")] string[]? scene_names = null,
		[Description("True to frame the target scene after loading the overlay.")] bool reset_view = false)
	{
		var parameters = OverlayParameters(overlay_id, name, script, scene_names, reset_view);
		return m_broker.OverlaySetScriptAsync(instance_id, parameters);
	}

	/// <summary>Append raw LDraw script to a named transient MCP overlay</summary>
	[McpServerTool(Name = "ldraw_overlay_append_script", Title = "Append LDraw MCP overlay", ReadOnly = false, Destructive = false, Idempotent = false, OpenWorld = false, UseStructuredContent = true)]
	[Description("Appends raw .ldr script to a named transient MCP overlay. User source documents are not modified.")]
	public Task<LDrawOverlayResult> OverlayAppendScript(
		[Description("Raw .ldr script to append to the transient overlay.")] string script,
		[Description("The id of a running LDraw instance from ldraw_list_instances. Omit to target the default instance (most-recently-used, or auto-launched when none are running).")] string? instance_id = null,
		[Description("Overlay id to append to. Omit to use 'default'.")] string? overlay_id = null,
		[Description("Display name for the generated overlay source.")] string? name = null,
		[Description("Scene names to show the overlay in. Omit to use the first scene. Use '*' as the only value for all scenes.")] string[]? scene_names = null,
		[Description("True to frame the target scene after loading the overlay.")] bool reset_view = false)
	{
		var parameters = OverlayParameters(overlay_id, name, script, scene_names, reset_view);
		return m_broker.OverlayAppendScriptAsync(instance_id, parameters);
	}

	/// <summary>Clear one named transient MCP overlay, or all overlays</summary>
	[McpServerTool(Name = "ldraw_overlay_clear", Title = "Clear LDraw MCP overlay", ReadOnly = false, Destructive = false, Idempotent = false, OpenWorld = false, UseStructuredContent = true)]
	[Description("Clears a named transient MCP overlay, or all MCP overlays when overlay_id is omitted. User source documents are not modified.")]
	public Task<LDrawOverlayResult[]> OverlayClear(
		[Description("The id of a running LDraw instance from ldraw_list_instances. Omit to target the default instance (most-recently-used, or auto-launched when none are running).")] string? instance_id = null,
		[Description("Overlay id to clear. Omit to clear all MCP overlays.")] string? overlay_id = null)
	{
		var parameters = new LDrawOverlayClearParams
		{
			OverlayId = overlay_id,
		};
		return m_broker.OverlayClearAsync(instance_id, parameters);
	}

	/// <summary>Create an optional vector from nullable components</summary>
	private static LDrawVector3? VectorOrNull(double? x, double? y, double? z)
	{
		if (x == null && y == null && z == null)
			return null;
		if (x == null || y == null || z == null)
			throw new ArgumentException("Vector parameters require all three x, y, and z components.");

		return new LDrawVector3
		{
			X = x.Value,
			Y = y.Value,
			Z = z.Value,
		};
	}

	/// <summary>Create overlay parameters from MCP tool arguments</summary>
	private static LDrawOverlayScriptParams OverlayParameters(string? overlay_id, string? name, string script, string[]? scene_names, bool reset_view)
	{
		return new LDrawOverlayScriptParams
		{
			OverlayId = overlay_id,
			Name = name,
			Script = script,
			SceneNames = scene_names?.ToList() ?? [],
			ResetView = reset_view,
		};
	}
}
