using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Threading.Tasks;
using LDraw.UI;

namespace LDraw.MCP;

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
