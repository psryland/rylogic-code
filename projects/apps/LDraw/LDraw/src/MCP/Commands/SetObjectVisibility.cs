using System;
using System.ComponentModel;
using System.Threading.Tasks;
using ModelContextProtocol.Server;

namespace LDraw.MCP;

/// <summary>Set-object-visibility command broker routing</summary>
internal sealed partial class McpBroker
{
	/// <summary>Set visibility for objects matching a query in 'instance_id'</summary>
	public async Task<LDrawObjectMutationResult> SetObjectVisibilityAsync(string? instance_id, LDrawSetObjectVisibilityParams parameters)
	{
		var registration = ResolveInstance(instance_id);
		return await m_client.SetObjectVisibilityAsync(registration, parameters).ConfigureAwait(false);
	}
}

/// <summary>Set-object-visibility command pipe client call</summary>
internal sealed partial class InstancePipeClient
{
	/// <summary>Set visibility for objects in 'registration'</summary>
	public async Task<LDrawObjectMutationResult> SetObjectVisibilityAsync(InstanceRegistration registration, LDrawSetObjectVisibilityParams parameters)
	{
		return await SendAsync<LDrawObjectMutationResult>(registration, InstancePipeCommands.SetObjectVisibility, parameters, WriteTimeout).ConfigureAwait(false);
	}
}

/// <summary>Set-object-visibility command instance implementation</summary>
internal sealed partial class LDrawInstanceHost
{
	/// <summary>Set visibility for matching objects in the requested scene</summary>
	private Task<LDrawObjectMutationResult> SetObjectVisibilityAsync(LDrawSetObjectVisibilityParams parameters)
	{
		return m_model.InvokeAsync(() =>
		{
			if (parameters.SetVisible == null)
				throw new InvalidOperationException("ldraw_set_object_visibility requires the visible parameter.");

			var scene = ResolveScene(parameters.SceneName);
			var query = QueryObjects(scene, parameters);
			var targets = ResolveMutationTargets(query, "ldraw_set_object_visibility");
			foreach (var entry in targets)
				entry.Object.Visible = parameters.SetVisible.Value;

			scene.SceneView.Invalidate();
			return new LDrawObjectMutationResult
			{
				SceneName = scene.SceneName,
				Action = parameters.SetVisible.Value ? "show_objects" : "hide_objects",
				Objects = CreateObjectInfos(targets),
			};
		}, TimeSpan.FromSeconds(10));
	}
}

/// <summary>Set-object-visibility command MCP tool surface</summary>
internal sealed partial class LDrawTools
{
	/// <summary>Show or hide objects in a running LDraw scene</summary>
	[McpServerTool(Name = "ldraw_set_object_visibility", Title = "Set LDraw object visibility", ReadOnly = false, Destructive = false, Idempotent = true, OpenWorld = false, UseStructuredContent = true)]
	[Description("Shows or hides objects matched by query. Visibility changes use existing View3D object state and do not edit user source files.")]
	public Task<LDrawObjectMutationResult> SetObjectVisibility(
		[Description("The visibility state to assign to matching objects.")] bool? visible,
		[Description("The instance id returned by ldraw_list_instances. Omit to target the broker instance.")] string? instance_id = null,
		[Description("The scene name to modify. Omit to use the first scene.")] string? scene_name = null,
		[Description("Opaque object id returned by ldraw_find_objects or ldraw_list_objects.")] string? object_id = null,
		[Description("Object name filter.")] string? name = null,
		[Description("Object type filter, for example Model, Group, Line, or Box.")] string? type = null,
		[Description("Optional source context id returned by ldraw_get_scene_summary or ldraw_list_scenes.")] string? context_id = null,
		[Description("Optional selected-state filter.")] bool? selected = null,
		[Description("Optional current visible-state filter.")] bool? current_visible = null,
		[Description("True to recursively include child objects when matching.")] bool include_children = true,
		[Description("Name/type matching mode: exact, contains, or regex.")] string match_mode = "contains",
		[Description("True for case-sensitive name/type matching.")] bool case_sensitive = false,
		[Description("Maximum number of matching objects to mutate, clamped to 1..1000. Throws if more matches exist.")] int max_count = 200)
	{
		var parameters = ObjectQueryParameters<LDrawSetObjectVisibilityParams>(scene_name, object_id, name, type, context_id, selected, current_visible, include_children, match_mode, case_sensitive, max_count);
		parameters.SetVisible = visible;
		return m_broker.SetObjectVisibilityAsync(instance_id, parameters);
	}
}
