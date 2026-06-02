using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Threading.Tasks;
using ModelContextProtocol.Server;

namespace LDraw.MCP;

/// <summary>Set-object-colour command broker routing</summary>
internal sealed partial class McpBroker
{
	/// <summary>Set colour for objects matching a query in 'instance_id'</summary>
	public async Task<LDrawObjectMutationResult> SetObjectColourAsync(string? instance_id, LDrawSetObjectColourParams parameters)
	{
		var registration = await ResolveInstanceAsync(instance_id).ConfigureAwait(false);
		return await m_client.SetObjectColourAsync(registration, parameters).ConfigureAwait(false);
	}
}

/// <summary>Set-object-colour command pipe client call</summary>
internal sealed partial class InstancePipeClient
{
	/// <summary>Set colour for objects in 'registration'</summary>
	public async Task<LDrawObjectMutationResult> SetObjectColourAsync(InstanceRegistration registration, LDrawSetObjectColourParams parameters)
	{
		return await SendAsync<LDrawObjectMutationResult>(registration, InstancePipeCommands.SetObjectColour, parameters, WriteTimeout).ConfigureAwait(false);
	}
}

/// <summary>Set-object-colour command MCP tool surface</summary>
internal sealed partial class LDrawTools
{
	/// <summary>Set or reset colours for objects in a running LDraw scene</summary>
	[McpServerTool(Name = "ldraw_set_object_colour", Title = "Set LDraw object colour", ReadOnly = false, Destructive = false, Idempotent = true, OpenWorld = false, UseStructuredContent = true)]
	[Description("Sets current object colour for debug highlighting, or resets current colour back to base colour. User source files are not edited.")]
	public Task<LDrawObjectMutationResult> SetObjectColour(
		[Description("Colour to apply, e.g. FFFF00FF, #FF00FF, or Red. Required unless reset is true.")] string? colour = null,
		[Description("The id of a running LDraw instance from ldraw_list_instances. Omit to target the default instance (most-recently-used, or auto-launched when none are running).")] string? instance_id = null,
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
		[Description("True to reset current colour back to base colour instead of applying colour.")] bool reset = false,
		[Description("True to modify base colour instead of current colour. Defaults false for temporary debug highlighting.")] bool base_colour = false,
		[Description("True to apply the colour operation recursively to children of each matched object.")] bool recursive = true)
	{
		var parameters = ObjectQueryParameters<LDrawSetObjectColourParams>(scene_name, object_id, name, type, context_id, selected, visible, include_children, match_mode, case_sensitive, max_count);
		parameters.Colour = colour;
		parameters.Reset = reset;
		parameters.BaseColour = base_colour;
		parameters.Recursive = recursive;
		return m_broker.SetObjectColourAsync(instance_id, parameters);
	}
}
