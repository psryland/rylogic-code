using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Threading.Tasks;
using ModelContextProtocol.Server;

namespace LDraw.MCP;

/// <summary>Get-object command broker routing</summary>
internal sealed partial class McpBroker
{
	/// <summary>Return one object matching a query in 'instance_id'</summary>
	public async Task<LDrawObjectInfo> GetObjectAsync(string? instance_id, LDrawGetObjectParams parameters)
	{
		var registration = ResolveInstance(instance_id);
		return await m_client.GetObjectAsync(registration, parameters).ConfigureAwait(false);
	}
}

/// <summary>Get-object command pipe client call</summary>
internal sealed partial class InstancePipeClient
{
	/// <summary>Read one object matching 'parameters' from 'registration'</summary>
	public async Task<LDrawObjectInfo> GetObjectAsync(InstanceRegistration registration, LDrawGetObjectParams parameters)
	{
		return await SendAsync<LDrawObjectInfo>(registration, InstancePipeCommands.GetObject, parameters, ReadTimeout).ConfigureAwait(false);
	}
}

/// <summary>Get-object command MCP tool surface</summary>
internal sealed partial class LDrawTools
{
	/// <summary>Return one object from a running LDraw scene</summary>
	[McpServerTool(Name = "ldraw_get_object", Title = "Get LDraw object", ReadOnly = true, Destructive = false, Idempotent = true, OpenWorld = false, UseStructuredContent = true)]
	[Description("Returns exactly one object. Throws if the query matches no objects or multiple objects; use object_id to disambiguate.")]
	public Task<LDrawObjectInfo> GetObject(
		[Description("The instance id returned by ldraw_list_instances. Omit to target the broker instance.")] string? instance_id = null,
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
		[Description("Maximum number of candidates to inspect before reporting ambiguity, clamped to 1..1000.")] int max_count = 20)
	{
		var parameters = ObjectQueryParameters<LDrawGetObjectParams>(scene_name, object_id, name, type, context_id, selected, visible, include_children, match_mode, case_sensitive, max_count);
		return m_broker.GetObjectAsync(instance_id, parameters);
	}
}
