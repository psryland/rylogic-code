using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Threading.Tasks;
using ModelContextProtocol.Server;

namespace LDraw.MCP;

/// <summary>Frame-object command broker routing</summary>
internal sealed partial class McpBroker
{
	/// <summary>Frame objects matching a query in 'instance_id'</summary>
	public async Task<LDrawFrameResult> FrameObjectAsync(string? instance_id, LDrawFrameObjectParams parameters)
	{
		var registration = ResolveInstance(instance_id);
		return await m_client.FrameObjectAsync(registration, parameters).ConfigureAwait(false);
	}
}

/// <summary>Frame-object command pipe client call</summary>
internal sealed partial class InstancePipeClient
{
	/// <summary>Frame objects matching 'parameters' in 'registration'</summary>
	public async Task<LDrawFrameResult> FrameObjectAsync(InstanceRegistration registration, LDrawFrameObjectParams parameters)
	{
		return await SendAsync<LDrawFrameResult>(registration, InstancePipeCommands.FrameObject, parameters, WriteTimeout).ConfigureAwait(false);
	}
}

/// <summary>Frame-object command instance implementation</summary>
internal sealed partial class LDrawInstanceHost
{
	/// <summary>Frame one or more matching objects in the requested scene</summary>
	private Task<LDrawFrameResult> FrameObjectAsync(LDrawFrameObjectParams parameters)
	{
		return m_model.InvokeAsync(() =>
		{
			var scene = ResolveScene(parameters.SceneName);
			var query = QueryObjects(scene, parameters);
			var targets = parameters.AllowMultiple ? query.Matches : new List<ObjectEntry> { ResolveSingleObject(query, "ldraw_frame_object") };
			if (targets.Count == 0)
				throw new InvalidOperationException("ldraw_frame_object did not match any objects.");
			if (parameters.AllowMultiple && query.Truncated)
				throw new InvalidOperationException("ldraw_frame_object matched more objects than max_count. Increase max_count or refine the query.");

			// Use the native selected-bounds calculation without permanently changing selection unless the caller explicitly asks for it.
			var original_selection = CaptureSelectionIds(scene);
			var action = parameters.Select ? "select_and_frame_object" : "frame_object";
			try
			{
				ApplySelection(scene, targets, "replace");
				var bounds = SelectedBounds(scene);
				if (!bounds.IsValid)
					throw new InvalidOperationException("Cannot frame object because the matched object bounds are not valid.");

				var window = scene.SceneView.Scene.Window;
				window.Camera.ResetView(bounds);
				if (!parameters.Select)
					RestoreSelection(scene, original_selection);

				scene.SceneView.Invalidate();
				return new LDrawFrameResult
				{
					SceneName = scene.SceneName,
					Action = action,
					Objects = CreateObjectInfos(targets),
					Bounds = LDrawBoundsInfo.From(bounds),
					Camera = CreateCameraInfo(scene),
				};
			}
			finally
			{
				if (!parameters.Select)
					RestoreSelection(scene, original_selection);
			}
		}, TimeSpan.FromSeconds(10));
	}
}

/// <summary>Frame-object command MCP tool surface</summary>
internal sealed partial class LDrawTools
{
	/// <summary>Frame an object in a running LDraw scene</summary>
	[McpServerTool(Name = "ldraw_frame_object", Title = "Frame LDraw object", ReadOnly = false, Destructive = false, Idempotent = true, OpenWorld = false, UseStructuredContent = true)]
	[Description("Frames one matching object without changing selection by default. Set select=true to replace selection with the framed object.")]
	public Task<LDrawFrameResult> FrameObject(
		[Description("The instance id returned by ldraw_list_instances. Omit to target the broker instance.")] string? instance_id = null,
		[Description("The scene name to modify. Omit to use the first scene.")] string? scene_name = null,
		[Description("Opaque object id returned by ldraw_find_objects or ldraw_list_objects.")] string? object_id = null,
		[Description("Object name filter.")] string? name = null,
		[Description("Object type filter, for example Model, Group, Line, or Box.")] string? type = null,
		[Description("Optional source context id returned by ldraw_get_scene_summary.")] string? context_id = null,
		[Description("Optional selected-state filter.")] bool? selected = null,
		[Description("Optional visible-state filter.")] bool? visible = null,
		[Description("True to recursively include child objects.")] bool include_children = true,
		[Description("Name/type matching mode: exact, contains, or regex.")] string match_mode = "contains",
		[Description("True for case-sensitive name/type matching.")] bool case_sensitive = false,
		[Description("Maximum number of candidates to inspect, clamped to 1..1000.")] int max_count = 20,
		[Description("True to replace selection with the framed object.")] bool select = false,
		[Description("True to allow framing multiple matching objects.")] bool allow_multiple = false)
	{
		var parameters = ObjectQueryParameters<LDrawFrameObjectParams>(scene_name, object_id, name, type, context_id, selected, visible, include_children, match_mode, case_sensitive, max_count);
		parameters.Select = select;
		parameters.AllowMultiple = allow_multiple;
		return m_broker.FrameObjectAsync(instance_id, parameters);
	}
}
