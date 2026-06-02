using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Threading.Tasks;
using ModelContextProtocol.Server;

namespace LDraw.MCP;

/// <summary>Object render-state command broker routing</summary>
internal sealed partial class McpBroker
{
	/// <summary>Return render-state data for an object in 'instance_id'</summary>
	public async Task<LDrawObjectInfo> GetObjectRenderStateAsync(string? instance_id, LDrawGetObjectRenderStateParams parameters)
	{
		var registration = ResolveInstance(instance_id);
		return await m_client.GetObjectRenderStateAsync(registration, parameters).ConfigureAwait(false);
	}

	/// <summary>Set render-state data for objects in 'instance_id'</summary>
	public async Task<LDrawObjectMutationResult> SetObjectRenderStateAsync(string? instance_id, LDrawSetObjectRenderStateParams parameters)
	{
		var registration = ResolveInstance(instance_id);
		return await m_client.SetObjectRenderStateAsync(registration, parameters).ConfigureAwait(false);
	}
}

/// <summary>Object render-state command pipe client calls</summary>
internal sealed partial class InstancePipeClient
{
	/// <summary>Return render-state data for an object in 'registration'</summary>
	public async Task<LDrawObjectInfo> GetObjectRenderStateAsync(InstanceRegistration registration, LDrawGetObjectRenderStateParams parameters)
	{
		return await SendAsync<LDrawObjectInfo>(registration, InstancePipeCommands.GetObjectRenderState, parameters, WriteTimeout).ConfigureAwait(false);
	}

	/// <summary>Set render-state data for objects in 'registration'</summary>
	public async Task<LDrawObjectMutationResult> SetObjectRenderStateAsync(InstanceRegistration registration, LDrawSetObjectRenderStateParams parameters)
	{
		return await SendAsync<LDrawObjectMutationResult>(registration, InstancePipeCommands.SetObjectRenderState, parameters, WriteTimeout).ConfigureAwait(false);
	}
}

/// <summary>Object render-state command MCP tool surface</summary>
internal sealed partial class LDrawTools
{
	/// <summary>Return render-state fields for a running LDraw object</summary>
	[McpServerTool(Name = "ldraw_get_object_render_state", Title = "Get LDraw object render state", ReadOnly = true, Destructive = false, Idempotent = true, OpenWorld = false, UseStructuredContent = true)]
	[Description("Returns visibility, colour, wireframe, normals, reflectivity, sort-group, object flags, and first-nugget state for exactly one object matched by query.")]
	public Task<LDrawObjectInfo> GetObjectRenderState(
		[Description("The instance id returned by ldraw_list_instances. Omit to target the broker instance.")] string? instance_id = null,
		[Description("The scene name to query. Omit to use the first scene.")] string? scene_name = null,
		[Description("Opaque object id returned by ldraw_find_objects or ldraw_list_objects.")] string? object_id = null,
		[Description("Object name filter.")] string? name = null,
		[Description("Object type filter, for example Model, Group, Line, or Box.")] string? type = null,
		[Description("Optional source context id returned by ldraw_get_scene_summary or ldraw_list_scenes.")] string? context_id = null,
		[Description("Optional selected-state filter.")] bool? selected = null,
		[Description("Optional visible-state filter.")] bool? visible = null,
		[Description("True to recursively include child objects when matching.")] bool include_children = true,
		[Description("Name/type matching mode: exact, contains, or regex.")] string match_mode = "contains",
		[Description("True for case-sensitive name/type matching.")] bool case_sensitive = false,
		[Description("Maximum number of matching objects to inspect, clamped to 1..1000. Throws unless exactly one object matches.")] int max_count = 200)
	{
		var parameters = ObjectQueryParameters<LDrawGetObjectRenderStateParams>(scene_name, object_id, name, type, context_id, selected, visible, include_children, match_mode, case_sensitive, max_count);
		return m_broker.GetObjectRenderStateAsync(instance_id, parameters);
	}

	/// <summary>Set render-state fields for running LDraw objects</summary>
	[McpServerTool(Name = "ldraw_set_object_render_state", Title = "Set LDraw object render state", ReadOnly = false, Destructive = false, Idempotent = true, OpenWorld = false, UseStructuredContent = true)]
	[Description("Sets runtime object render-state fields such as visibility, wireframe, normals, reflectivity, sort group, and selected flags. User source files are not edited.")]
	public Task<LDrawObjectMutationResult> SetObjectRenderState(
		[Description("Optional visibility state to assign.")] bool? visible_state = null,
		[Description("Optional wireframe state to assign.")] bool? wireframe = null,
		[Description("Optional per-object normal-display state to assign.")] bool? show_normals = null,
		[Description("Optional reflectivity value to assign.")] double? reflectivity = null,
		[Description("Optional sort group name, for example Default, AlphaBack, or AlphaFront.")] string? sort_group = null,
		[Description("Optional first-nugget tint colour, e.g. FFFFFFFF or Red.")] string? nugget_tint = null,
		[Description("Optional object SceneBoundsExclude flag state.")] bool? scene_bounds_excluded = null,
		[Description("Optional object HitTestExclude flag state.")] bool? hit_test_excluded = null,
		[Description("Optional object ShadowCastExclude flag state.")] bool? shadow_cast_excluded = null,
		[Description("Optional object NoZTest flag state.")] bool? no_z_test = null,
		[Description("Optional object NoZWrite flag state.")] bool? no_z_write = null,
		[Description("Optional first-nugget Hidden flag state.")] bool? nugget_hidden = null,
		[Description("Optional first-nugget AlphaBlend flag state.")] bool? nugget_alpha_blend = null,
		[Description("True to apply supported render-state setters recursively to child objects.")] bool recursive = true,
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
		var parameters = ObjectQueryParameters<LDrawSetObjectRenderStateParams>(scene_name, object_id, name, type, context_id, selected, current_visible, include_children, match_mode, case_sensitive, max_count);
		parameters.VisibleState = visible_state;
		parameters.Wireframe = wireframe;
		parameters.ShowNormals = show_normals;
		parameters.Reflectivity = reflectivity;
		parameters.SortGroup = sort_group;
		parameters.NuggetTint = nugget_tint;
		parameters.SceneBoundsExcluded = scene_bounds_excluded;
		parameters.HitTestExcluded = hit_test_excluded;
		parameters.ShadowCastExcluded = shadow_cast_excluded;
		parameters.NoZTest = no_z_test;
		parameters.NoZWrite = no_z_write;
		parameters.NuggetHidden = nugget_hidden;
		parameters.NuggetAlphaBlend = nugget_alpha_blend;
		parameters.Recursive = recursive;
		return m_broker.SetObjectRenderStateAsync(instance_id, parameters);
	}
}
