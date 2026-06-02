using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Threading.Tasks;
using ModelContextProtocol.Server;

namespace LDraw.MCP;

/// <summary>Set-diagnostic-modes command broker routing</summary>
internal sealed partial class McpBroker
{
	/// <summary>Set diagnostic modes for a scene in 'instance_id'</summary>
	public async Task<LDrawViewMutationResult> SetDiagnosticModesAsync(string? instance_id, LDrawSetDiagnosticModesParams parameters)
	{
		var registration = ResolveInstance(instance_id);
		return await m_client.SetDiagnosticModesAsync(registration, parameters).ConfigureAwait(false);
	}
}

/// <summary>Set-diagnostic-modes command pipe client call</summary>
internal sealed partial class InstancePipeClient
{
	/// <summary>Set diagnostic modes for a scene in 'registration'</summary>
	public async Task<LDrawViewMutationResult> SetDiagnosticModesAsync(InstanceRegistration registration, LDrawSetDiagnosticModesParams parameters)
	{
		return await SendAsync<LDrawViewMutationResult>(registration, InstancePipeCommands.SetDiagnosticModes, parameters, WriteTimeout).ConfigureAwait(false);
	}
}

/// <summary>Set-diagnostic-modes command MCP tool surface</summary>
internal sealed partial class LDrawTools
{
	/// <summary>Set diagnostic and rendering modes for a running LDraw scene</summary>
	[McpServerTool(Name = "ldraw_set_diagnostic_modes", Title = "Set LDraw diagnostic modes", ReadOnly = false, Destructive = false, Idempotent = true, OpenWorld = false, UseStructuredContent = true)]
	[Description("Sets any supplied diagnostic/rendering modes; omitted nullable values are left unchanged. Fill/cull/focus/origin modes persist through normal scene settings, while bboxes/normals/selection box/object info are runtime view diagnostics.")]
	public Task<LDrawViewMutationResult> SetDiagnosticModes(
		[Description("The instance id returned by ldraw_list_instances. Omit to target the broker instance.")] string? instance_id = null,
		[Description("The scene name to modify. Omit to use the first scene.")] string? scene_name = null,
		[Description("Show native diagnostic bounding boxes for all objects. For specific object bounds, use ldraw_show_object_bounds.")] bool? bboxes_visible = null,
		[Description("Show normals for all objects in the scene. For specific objects, use ldraw_show_normals.")] bool? show_normals = null,
		[Description("Diagnostic normal vector length. Must be positive.")] double? normals_length = null,
		[Description("Diagnostic normal colour, e.g. FFFF00FF, #FF00FF, or Red.")] string? normals_colour = null,
		[Description("Scene fill mode: Default, Points, Wireframe, Solid, or SolidWire.")] string? fill_mode = null,
		[Description("Scene cull mode: Default, None, Front, or Back.")] string? cull_mode = null,
		[Description("Uniform point size used when fill_mode is Points. Must be positive.")] double? fill_mode_points_size = null,
		[Description("Show the camera focus point marker.")] bool? focus_point_visible = null,
		[Description("Show the origin marker.")] bool? origin_point_visible = null,
		[Description("Show the selection bounding box.")] bool? selection_box_visible = null,
		[Description("Enable hover object information in the scene view.")] bool? object_info_enabled = null)
	{
		var parameters = new LDrawSetDiagnosticModesParams
		{
			SceneName = scene_name,
			BBoxesVisible = bboxes_visible,
			ShowNormals = show_normals,
			NormalsLength = normals_length,
			NormalsColour = normals_colour,
			FillMode = fill_mode,
			CullMode = cull_mode,
			FillModePointsSize = fill_mode_points_size,
			FocusPointVisible = focus_point_visible,
			OriginPointVisible = origin_point_visible,
			SelectionBoxVisible = selection_box_visible,
			ObjectInfoEnabled = object_info_enabled,
		};
		return m_broker.SetDiagnosticModesAsync(instance_id, parameters);
	}
}
