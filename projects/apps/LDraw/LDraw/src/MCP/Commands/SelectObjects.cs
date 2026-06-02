using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Threading.Tasks;
using LDraw.UI;
using ModelContextProtocol.Server;

namespace LDraw.MCP;

/// <summary>Select-objects command broker routing</summary>
internal sealed partial class McpBroker
{
	/// <summary>Change object selection in 'instance_id'</summary>
	public async Task<LDrawSelectionResult> SelectObjectsAsync(string? instance_id, LDrawSelectObjectsParams parameters)
	{
		var registration = ResolveInstance(instance_id);
		return await m_client.SelectObjectsAsync(registration, parameters).ConfigureAwait(false);
	}
}

/// <summary>Select-objects command pipe client call</summary>
internal sealed partial class InstancePipeClient
{
	/// <summary>Change object selection in 'registration'</summary>
	public async Task<LDrawSelectionResult> SelectObjectsAsync(InstanceRegistration registration, LDrawSelectObjectsParams parameters)
	{
		return await SendAsync<LDrawSelectionResult>(registration, InstancePipeCommands.SelectObjects, parameters, WriteTimeout).ConfigureAwait(false);
	}
}

/// <summary>Select-objects command instance implementation</summary>
internal sealed partial class LDrawInstanceHost
{
	/// <summary>Change selected object flags using the requested selection mode</summary>
	private Task<LDrawSelectionResult> SelectObjectsAsync(LDrawSelectObjectsParams parameters)
	{
		return m_model.InvokeAsync(() =>
		{
			var scene = ResolveScene(parameters.SceneName);
			var mode = NormaliseSelectionMode(parameters.Mode);
			var query = mode == "clear"
				? new ObjectQueryResult(scene, [], Truncated: false)
				: QueryObjects(scene, parameters);
			if (mode != "clear" && query.Matches.Count == 0)
				throw new InvalidOperationException("ldraw_select_objects did not match any objects.");

			// Selection is kept within MCP by using existing View3D selected flags; UI-level selection synchronisation remains a review item.
			ApplySelection(scene, query.Matches, mode);
			scene.SceneView.Invalidate();
			return CreateSelectionResult(scene, mode, parameters.IncludeChildren, parameters.MaxCount);
		}, TimeSpan.FromSeconds(10));
	}

	/// <summary>Apply a selection mutation to a scene</summary>
	private static void ApplySelection(SceneUI scene, List<ObjectEntry> targets, string mode)
	{
		switch (mode)
		{
			case "replace":
			{
				foreach (var entry in EnumerateObjects(scene, null, include_children: true))
					SetSelected(entry.Object, false);
				foreach (var entry in targets)
					SetSelected(entry.Object, true);
				break;
			}
			case "add":
			{
				foreach (var entry in targets)
					SetSelected(entry.Object, true);
				break;
			}
			case "remove":
			{
				foreach (var entry in targets)
					SetSelected(entry.Object, false);
				break;
			}
			case "toggle":
			{
				foreach (var entry in targets)
					SetSelected(entry.Object, !IsSelected(entry.Object));
				break;
			}
			case "clear":
			{
				foreach (var entry in EnumerateObjects(scene, null, include_children: true))
					SetSelected(entry.Object, false);
				break;
			}
			default:
			{
				throw new ArgumentOutOfRangeException(nameof(mode), mode, "Unknown selection mode.");
			}
		}
	}

	/// <summary>Create the public selection result for a scene</summary>
	private static LDrawSelectionResult CreateSelectionResult(SceneUI scene, string action, bool include_children, int max_count)
	{
		var selected = SelectedObjects(scene, include_children, max_count);
		var bounds = SelectedBounds(scene);
		return new LDrawSelectionResult
		{
			SceneName = scene.SceneName,
			Action = action,
			Objects = CreateObjectInfos(selected),
			Bounds = bounds.IsValid ? LDrawBoundsInfo.From(bounds) : null,
		};
	}

	/// <summary>Normalise and validate a selection mode string</summary>
	private static string NormaliseSelectionMode(string? mode)
	{
		var value = string.IsNullOrWhiteSpace(mode) ? "replace" : mode.Trim().ToLowerInvariant();
		switch (value)
		{
			case "replace":
			case "add":
			case "remove":
			case "toggle":
			case "clear":
			{
				return value;
			}
			default:
			{
				throw new ArgumentOutOfRangeException(nameof(mode), mode, "Selection mode must be replace, add, remove, toggle, or clear.");
			}
		}
	}
}

/// <summary>Select-objects command MCP tool surface</summary>
internal sealed partial class LDrawTools
{
	/// <summary>Select objects in a running LDraw scene</summary>
	[McpServerTool(Name = "ldraw_select_objects", Title = "Select LDraw objects", ReadOnly = false, Destructive = false, Idempotent = false, OpenWorld = false, UseStructuredContent = true)]
	[Description("Selects objects by query. Mode is replace, add, remove, toggle, or clear. Omit scene_name to use the first scene.")]
	public Task<LDrawSelectionResult> SelectObjects(
		[Description("Selection operation: replace, add, remove, toggle, or clear.")] string mode = "replace",
		[Description("The instance id returned by ldraw_list_instances. Omit to target the broker instance.")] string? instance_id = null,
		[Description("The scene name to modify. Omit to use the first scene.")] string? scene_name = null,
		[Description("Opaque object id returned by ldraw_find_objects or ldraw_list_objects.")] string? object_id = null,
		[Description("Object name filter. Not required when mode is clear.")] string? name = null,
		[Description("Object type filter, for example Model, Group, Line, or Box.")] string? type = null,
		[Description("Optional source context id returned by ldraw_get_scene_summary.")] string? context_id = null,
		[Description("Optional selected-state filter.")] bool? selected = null,
		[Description("Optional visible-state filter.")] bool? visible = null,
		[Description("True to recursively include child objects.")] bool include_children = true,
		[Description("Name/type matching mode: exact, contains, or regex.")] string match_mode = "contains",
		[Description("True for case-sensitive name/type matching.")] bool case_sensitive = false,
		[Description("Maximum number of matching objects to select, clamped to 1..1000.")] int max_count = 200)
	{
		var parameters = ObjectQueryParameters<LDrawSelectObjectsParams>(scene_name, object_id, name, type, context_id, selected, visible, include_children, match_mode, case_sensitive, max_count);
		parameters.Mode = mode;
		return m_broker.SelectObjectsAsync(instance_id, parameters);
	}
}
