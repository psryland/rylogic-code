using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Threading.Tasks;
using ModelContextProtocol.Server;

namespace LDraw.MCP;

/// <summary>Show-object-bounds command broker routing</summary>
internal sealed partial class McpBroker
{
	/// <summary>Draw bounds for objects matching a query in 'instance_id'</summary>
	public async Task<LDrawBoundsOverlayResult> ShowObjectBoundsAsync(string? instance_id, LDrawShowObjectBoundsParams parameters)
	{
		var registration = await ResolveInstanceAsync(instance_id).ConfigureAwait(false);
		return await m_client.ShowObjectBoundsAsync(registration, parameters).ConfigureAwait(false);
	}
}

/// <summary>Show-object-bounds command pipe client call</summary>
internal sealed partial class InstancePipeClient
{
	/// <summary>Draw bounds for objects in 'registration'</summary>
	public async Task<LDrawBoundsOverlayResult> ShowObjectBoundsAsync(InstanceRegistration registration, LDrawShowObjectBoundsParams parameters)
	{
		return await SendAsync<LDrawBoundsOverlayResult>(registration, InstancePipeCommands.ShowObjectBounds, parameters, WriteTimeout).ConfigureAwait(false);
	}
}

/// <summary>Show-object-bounds command MCP tool surface</summary>
internal sealed partial class LDrawTools
{
	/// <summary>Draw bounds for objects in a running LDraw scene</summary>
	[McpServerTool(Name = "ldraw_show_object_bounds", Title = "Show LDraw object bounds", ReadOnly = false, Destructive = false, Idempotent = true, OpenWorld = false, UseStructuredContent = true)]
	[Description("Draws world-space AABBs for matching objects using a replaceable MCP overlay. Use ldraw_set_diagnostic_modes bboxes_visible for cheaper all-object native bounding boxes.")]
	public Task<LDrawBoundsOverlayResult> ShowObjectBounds(
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
		[Description("Maximum number of matching objects to draw, clamped to 1..1000. Throws if more matches exist.")] int max_count = 200,
		[Description("Overlay id to replace. Omit to use 'diagnostic_bounds'. Reusing the id replaces previous bounds instead of stacking overlays.")] string? overlay_id = null,
		[Description("Display name for the generated overlay source.")] string? overlay_name = null,
		[Description("Bounds colour, e.g. FF00FF00, #00FF00, or Green.")] string? colour = null,
		[Description("Line width in pixels. Must be positive.")] double line_width = 2.0)
	{
		var parameters = ObjectQueryParameters<LDrawShowObjectBoundsParams>(scene_name, object_id, name, type, context_id, selected, visible, include_children, match_mode, case_sensitive, max_count);
		parameters.OverlayId = overlay_id;
		parameters.OverlayName = overlay_name;
		parameters.Colour = colour;
		parameters.LineWidth = line_width;
		return m_broker.ShowObjectBoundsAsync(instance_id, parameters);
	}
}
