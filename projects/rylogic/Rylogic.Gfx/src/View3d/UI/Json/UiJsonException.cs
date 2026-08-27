using System;

namespace Rylogic.Gfx.UI.Json;

/// <summary>
/// Thrown for any malformed, structurally invalid, or unresolvable View3DUI JSON document: parser errors, unsupported
/// schema versions, unknown control/layout/resource/template/style/state/primitive kinds, dangling id references, and
/// values that violate a closed-vocabulary or ABI-derived limit (for example exceeding UiTemplateDesc.MaxParts). Never
/// thrown for a problem the native ABI itself would reject; that remains View3dUiException's responsibility.
/// </summary>
public sealed class UiJsonException :Exception
{
	/// <summary>A JSON-Pointer-like dotted/bracketed path to the offending value, for example "tree[0].children[2].style_id".</summary>
	public string Path { get; }

	/// <summary>Describe one JSON document problem found at 'path'.</summary>
	public UiJsonException(string path, string message)
		: base($"{path}: {message}")
	{
		Path = path;
	}

	/// <summary>Describe one JSON document problem found at 'path', wrapping the lower-level cause.</summary>
	public UiJsonException(string path, string message, Exception inner)
		: base($"{path}: {message}", inner)
	{
		Path = path;
	}
}
