using System;
using System.ComponentModel;
using System.Linq;
using System.Threading.Tasks;
using ModelContextProtocol.Server;

namespace LDraw.MCP;

/// <summary>Routes MCP tool calls to registered LDraw instances</summary>
internal sealed partial class McpBroker
{
	private readonly InstanceRegistry m_registry;
	private readonly InstancePipeClient m_client;
	private readonly string m_local_instance_id;
	private readonly Func<string> m_endpoint;

	/// <summary>Create a broker for routing MCP requests to registered LDraw instances</summary>
	public McpBroker(InstanceRegistry registry, InstancePipeClient client, string local_instance_id, Func<string> endpoint)
	{
		m_registry = registry;
		m_client = client;
		m_local_instance_id = local_instance_id;
		m_endpoint = endpoint;
	}

	/// <summary>Return all live instances registered with the local broker</summary>
	public Task<McpInstanceInfo[]> ListInstancesAsync()
	{
		var instances = m_registry.LiveInstances()
			.Select(ToInfo)
			.ToArray();

		return Task.FromResult(instances);
	}

	/// <summary>Return a scene summary for 'instance_id', or the broker instance when omitted</summary>
	public async Task<LDrawSceneSummary> GetSceneSummaryAsync(string? instance_id)
	{
		var registration = ResolveInstance(instance_id);
		var summary = await m_client.GetSceneSummaryAsync(registration).ConfigureAwait(false);
		summary.Instance = ToInfo(registration);
		return summary;
	}

	/// <summary>Return object summaries for a scene in 'instance_id'</summary>
	public async Task<LDrawObjectList> ListObjectsAsync(string? instance_id, LDrawListObjectsParams parameters)
	{
		var registration = ResolveInstance(instance_id);
		return await m_client.ListObjectsAsync(registration, parameters).ConfigureAwait(false);
	}

	/// <summary>Return camera state for a scene in 'instance_id'</summary>
	public async Task<LDrawCameraInfo> GetCameraAsync(string? instance_id, LDrawCameraParams parameters)
	{
		var registration = ResolveInstance(instance_id);
		return await m_client.GetCameraAsync(registration, parameters).ConfigureAwait(false);
	}

	/// <summary>Set camera state for a scene in 'instance_id'</summary>
	public async Task<LDrawCameraSetResult> SetCameraAsync(string? instance_id, LDrawSetCameraParams parameters)
	{
		var registration = ResolveInstance(instance_id);
		return await m_client.SetCameraAsync(registration, parameters).ConfigureAwait(false);
	}

	/// <summary>Replace a named overlay script in 'instance_id'</summary>
	public async Task<LDrawOverlayResult> OverlaySetScriptAsync(string? instance_id, LDrawOverlayScriptParams parameters)
	{
		var registration = ResolveInstance(instance_id);
		return await m_client.OverlaySetScriptAsync(registration, parameters).ConfigureAwait(false);
	}

	/// <summary>Append to a named overlay script in 'instance_id'</summary>
	public async Task<LDrawOverlayResult> OverlayAppendScriptAsync(string? instance_id, LDrawOverlayScriptParams parameters)
	{
		var registration = ResolveInstance(instance_id);
		return await m_client.OverlayAppendScriptAsync(registration, parameters).ConfigureAwait(false);
	}

	/// <summary>Clear one or more named overlays in 'instance_id'</summary>
	public async Task<LDrawOverlayResult[]> OverlayClearAsync(string? instance_id, LDrawOverlayClearParams parameters)
	{
		var registration = ResolveInstance(instance_id);
		return await m_client.OverlayClearAsync(registration, parameters).ConfigureAwait(false);
	}

	/// <summary>Find an instance registration by id</summary>
	private InstanceRegistration ResolveInstance(string? instance_id)
	{
		var instances = m_registry.LiveInstances();

		// Most clients only connect to one LDraw process, so omitted instance id targets the process that won broker election.
		var id = string.IsNullOrWhiteSpace(instance_id) ? m_local_instance_id : instance_id;
		return instances.FirstOrDefault(x => string.Equals(x.InstanceId, id, StringComparison.OrdinalIgnoreCase))
			?? throw new InvalidOperationException($"No live LDraw instance with id '{id}' is registered.");
	}

	/// <summary>Convert a registry entry into public MCP output</summary>
	private McpInstanceInfo ToInfo(InstanceRegistration registration)
	{
		// Only the process that owns the HTTP listener has an MCP endpoint. Other instances are reached through the broker.
		var is_broker = string.Equals(registration.InstanceId, m_local_instance_id, StringComparison.OrdinalIgnoreCase);
		return new McpInstanceInfo
		{
			InstanceId = registration.InstanceId,
			ProcessId = registration.ProcessId,
			ProcessName = registration.ProcessName,
			StartedUtc = registration.StartedUtc,
			LastSeenUtc = registration.LastSeenUtc,
			IsBroker = is_broker,
			Endpoint = is_broker ? m_endpoint() : string.Empty,
			SettingsPath = registration.SettingsPath,
		};
	}
}

/// <summary>MCP tool surface exposed to AI clients</summary>
[McpServerToolType]
internal sealed partial class LDrawTools
{
	private readonly McpBroker m_broker;

	/// <summary>Create the tool surface bound to a broker instance</summary>
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
	[Description("Returns sources, scenes, and object counts for a running LDraw instance. Omit instance_id to use the broker instance.")]
	public Task<LDrawSceneSummary> GetSceneSummary(
		[Description("The instance id returned by ldraw_list_instances. Omit to target the broker instance.")] string? instance_id = null)
	{
		return m_broker.GetSceneSummaryAsync(instance_id);
	}

	/// <summary>List objects in a running LDraw scene</summary>
	[McpServerTool(Name = "ldraw_list_objects", Title = "List LDraw objects", ReadOnly = true, Destructive = false, Idempotent = true, OpenWorld = false, UseStructuredContent = true)]
	[Description("Lists objects in a scene, optionally filtered by source context id. Omit scene_name to use the first scene.")]
	public Task<LDrawObjectList> ListObjects(
		[Description("The instance id returned by ldraw_list_instances. Omit to target the broker instance.")] string? instance_id = null,
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
		[Description("The instance id returned by ldraw_list_instances. Omit to target the broker instance.")] string? instance_id = null,
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
		[Description("The instance id returned by ldraw_list_instances. Omit to target the broker instance.")] string? instance_id = null,
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
		[Description("The instance id returned by ldraw_list_instances. Omit to target the broker instance.")] string? instance_id = null,
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
		[Description("The instance id returned by ldraw_list_instances. Omit to target the broker instance.")] string? instance_id = null,
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
		[Description("The instance id returned by ldraw_list_instances. Omit to target the broker instance.")] string? instance_id = null,
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
