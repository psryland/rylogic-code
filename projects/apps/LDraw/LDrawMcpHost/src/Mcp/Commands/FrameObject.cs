using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Threading.Tasks;
using ModelContextProtocol.Server;

namespace LDraw.MCP;

/// <summary>Frame-object command broker routing</summary>
internal sealed partial class McpBroker
{
	/// <summary>Frame objects matching a query in 'instance_id'</summary>
	public async Task<LDrawFrameResult> FrameObjectAsync(string? instance_id, LDrawFrameObjectParams parameters)
	{
		var registration = await ResolveInstanceAsync(instance_id).ConfigureAwait(false);
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

/// <summary>Frame-object command MCP tool surface</summary>
internal sealed partial class LDrawTools
{
	/// <summary>Frame an object in a running LDraw scene</summary>
	[McpServerTool(Name = "ldraw_frame_object", Title = "Frame LDraw object", ReadOnly = false, Destructive = false, Idempotent = true, OpenWorld = false, UseStructuredContent = true)]
	[Description("Frames one matching object without changing selection by default. Set select=true to replace selection with the framed object.")]
	public Task<LDrawFrameResult> FrameObject(
		[Description("The id of a running LDraw instance from ldraw_list_instances. Omit to target the default instance (most-recently-used, or auto-launched when none are running).")] string? instance_id = null,
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
