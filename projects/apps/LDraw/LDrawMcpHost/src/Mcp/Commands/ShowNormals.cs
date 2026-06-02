using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Threading.Tasks;
using ModelContextProtocol.Server;

namespace LDraw.MCP;

/// <summary>Show-normals command broker routing</summary>
internal sealed partial class McpBroker
{
	/// <summary>Set normal visibility for objects matching a query in 'instance_id'</summary>
	public async Task<LDrawObjectMutationResult> ShowNormalsAsync(string? instance_id, LDrawShowNormalsParams parameters)
	{
		var registration = ResolveInstance(instance_id);
		return await m_client.ShowNormalsAsync(registration, parameters).ConfigureAwait(false);
	}
}

/// <summary>Show-normals command pipe client call</summary>
internal sealed partial class InstancePipeClient
{
	/// <summary>Set normal visibility for objects in 'registration'</summary>
	public async Task<LDrawObjectMutationResult> ShowNormalsAsync(InstanceRegistration registration, LDrawShowNormalsParams parameters)
	{
		return await SendAsync<LDrawObjectMutationResult>(registration, InstancePipeCommands.ShowNormals, parameters, WriteTimeout).ConfigureAwait(false);
	}
}

/// <summary>Show-normals command MCP tool surface</summary>
internal sealed partial class LDrawTools
{
	/// <summary>Show or hide normals for objects in a running LDraw scene</summary>
	[McpServerTool(Name = "ldraw_show_normals", Title = "Show LDraw object normals", ReadOnly = false, Destructive = false, Idempotent = true, OpenWorld = false, UseStructuredContent = true)]
	[Description("Shows or hides normals for objects matched by query. This changes existing View3D object diagnostic state and does not edit user source files.")]
	public Task<LDrawObjectMutationResult> ShowNormals(
		[Description("True to show normals; false to hide normals.")] bool show = true,
		[Description("The instance id returned by ldraw_list_instances. Omit to target the broker instance.")] string? instance_id = null,
		[Description("The scene name to modify. Omit to use the first scene.")] string? scene_name = null,
		[Description("Opaque object id returned by ldraw_find_objects or ldraw_list_objects.")] string? object_id = null,
		[Description("Object name filter.")] string? name = null,
		[Description("Object type filter, for example Model, Group, Line, or Box.")] string? type = null,
		[Description("Optional source context id returned by ldraw_get_scene_summary or ldraw_list_scenes.")] string? context_id = null,
		[Description("Optional selected-state filter.")] bool? selected = null,
		[Description("Optional visible-state filter.")] bool? visible = null,
		[Description("True to recursively include child objects when matching.")] bool include_children = true,
		[Description("Name/type matching mode: exact, contains, or regex.")] string match_mode = "contains",
		[Description("True for case-sensitive name/type matching.")] bool case_sensitive = false,
		[Description("Maximum number of matching objects to mutate, clamped to 1..1000. Throws if more matches exist.")] int max_count = 200,
		[Description("True to apply the normal visibility recursively to children of each matched object.")] bool recursive = true)
	{
		var parameters = ObjectQueryParameters<LDrawShowNormalsParams>(scene_name, object_id, name, type, context_id, selected, visible, include_children, match_mode, case_sensitive, max_count);
		parameters.SetShowNormals = show;
		parameters.Recursive = recursive;
		return m_broker.ShowNormalsAsync(instance_id, parameters);
	}
}
