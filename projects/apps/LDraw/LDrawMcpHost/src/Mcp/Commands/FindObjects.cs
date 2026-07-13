using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Threading.Tasks;
using ModelContextProtocol.Server;

namespace LDraw.MCP;

/// <summary>Find-object command broker routing</summary>
internal sealed partial class McpBroker
{
	/// <summary>Return objects matching a query in 'instance_id'</summary>
	public async Task<LDrawObjectList> FindObjectsAsync(string? instance_id, LDrawObjectQueryParams parameters)
	{
		var registration = await ResolveInstanceAsync(instance_id).ConfigureAwait(false);
		return await m_client.FindObjectsAsync(registration, parameters).ConfigureAwait(false);
	}
}

/// <summary>Find-object command pipe client call</summary>
internal sealed partial class InstancePipeClient
{
	/// <summary>Read objects matching 'parameters' from 'registration'</summary>
	public async Task<LDrawObjectList> FindObjectsAsync(InstanceRegistration registration, LDrawObjectQueryParams parameters)
	{
		return await SendAsync<LDrawObjectList>(registration, InstancePipeCommands.FindObjects, parameters, ReadTimeout).ConfigureAwait(false);
	}
}

/// <summary>Find-object command MCP tool surface</summary>
internal sealed partial class LDrawTools
{
	/// <summary>Find objects in a running LDraw scene</summary>
	[McpServerTool(Name = "ldraw_find_objects", Title = "Find LDraw objects", ReadOnly = true, Destructive = false, Idempotent = true, OpenWorld = false, UseStructuredContent = true)]
	[Description("Finds objects by object id, name, type, context id, selected state, or visible state. Omit scene_name to use the first scene.")]
	public Task<LDrawObjectList> FindObjects(
		[Description("The id of a running LDraw instance from ldraw_list_instances. Omit to target the default instance (most-recently-used, or auto-launched when none are running).")] string? instance_id = null,
		[Description("The scene name to query. Omit to use the first scene.")] string? scene_name = null,
		[Description("Opaque object id returned by ldraw_find_objects or ldraw_list_objects.")] string? object_id = null,
		[Description("Object name filter.")] string? name = null,
		[Description("Object type filter, for example Model, Group, Line, or Box.")] string? type = null,
		[Description("Optional source context id returned by ldraw_get_scene_summary.")] string? context_id = null,
		[Description("Optional selected-state filter.")] bool? selected = null,
		[Description("Optional visible-state filter.")] bool? visible = null,
		[Description("True to recursively include child objects.")] bool include_children = true,
		[Description("Name/type matching mode: exact, contains, or regex.")] string match_mode = "contains",
		[Description("True for case-sensitive name/type matching.")] bool case_sensitive = false,
		[Description("Maximum number of objects to return, clamped to 1..1000.")] int max_count = 200)
	{
		var parameters = ObjectQueryParameters<LDrawObjectQueryParams>(scene_name, object_id, name, type, context_id, selected, visible, include_children, match_mode, case_sensitive, max_count);
		return m_broker.FindObjectsAsync(instance_id, parameters);
	}
}
