using System;
using System.Collections.Generic;
using System.Text.Json;
using System.Text.Json.Serialization;
using Rylogic.Gfx;
using Rylogic.Maths;

namespace LDraw.MCP;

/// <summary>Persisted registration data for one running LDraw instance</summary>
internal sealed class InstanceRegistration
{
	/// <summary>Version of the registry file schema</summary>
	public int SchemaVersion { get; set; } = 1;

	/// <summary>Unique id for this process lifetime</summary>
	public string InstanceId { get; set; } = string.Empty;

	/// <summary>Windows process id for liveness checks</summary>
	public int ProcessId { get; set; }

	/// <summary>Process name used to guard against PID reuse</summary>
	public string ProcessName { get; set; } = string.Empty;

	/// <summary>Time that this LDraw process registered itself</summary>
	public DateTimeOffset StartedUtc { get; set; }

	/// <summary>Last time this registration was refreshed</summary>
	public DateTimeOffset LastSeenUtc { get; set; }

	/// <summary>Name of the current-user pipe hosted by this process</summary>
	public string PipeName { get; set; } = string.Empty;

	/// <summary>Settings file used by this process</summary>
	public string SettingsPath { get; set; } = string.Empty;

	[JsonIgnore]
	/// <summary>Path to the registry file that was read or written</summary>
	public string FilePath { get; set; } = string.Empty;
}

/// <summary>Public MCP description of one running LDraw instance</summary>
public sealed class McpInstanceInfo
{
	/// <summary>Unique id for this process lifetime</summary>
	public string InstanceId { get; set; } = string.Empty;

	/// <summary>Windows process id for diagnostics</summary>
	public int ProcessId { get; set; }

	/// <summary>Process name for diagnostics</summary>
	public string ProcessName { get; set; } = string.Empty;

	/// <summary>Time that this process registered itself</summary>
	public DateTimeOffset StartedUtc { get; set; }

	/// <summary>Last time this process refreshed its registration</summary>
	public DateTimeOffset LastSeenUtc { get; set; }

	/// <summary>True if this instance owns the HTTP MCP listener</summary>
	public bool IsBroker { get; set; }

	/// <summary>The MCP endpoint when this instance is the broker</summary>
	public string Endpoint { get; set; } = string.Empty;

	/// <summary>Settings file used by this process</summary>
	public string SettingsPath { get; set; } = string.Empty;
}

/// <summary>Read-only summary of an LDraw instance</summary>
public sealed class LDrawSceneSummary
{
	/// <summary>The instance that produced this summary</summary>
	public McpInstanceInfo Instance { get; set; } = new();

	/// <summary>The LDraw sources loaded by this instance</summary>
	public List<LDrawSourceSummary> Sources { get; set; } = [];

	/// <summary>The scene views hosted by this instance</summary>
	public List<LDrawSceneInfo> Scenes { get; set; } = [];

	/// <summary>The transient MCP overlays owned by this instance</summary>
	public List<LDrawOverlayInfo> Overlays { get; set; } = [];

	/// <summary>The number of loaded sources</summary>
	public int SourceCount
	{
		get { return Sources.Count; }
	}

	/// <summary>The number of scene views</summary>
	public int SceneCount
	{
		get { return Scenes.Count; }
	}
}

/// <summary>Read-only summary of an LDraw source</summary>
public sealed class LDrawSourceSummary
{
	/// <summary>The View3D context id for this source</summary>
	public string ContextId { get; set; } = string.Empty;

	/// <summary>The display name of this source</summary>
	public string Name { get; set; } = string.Empty;

	/// <summary>The file path for file-backed sources</summary>
	public string FilePath { get; set; } = string.Empty;

	/// <summary>The number of objects in this source</summary>
	public int ObjectCount { get; set; }

	/// <summary>True while this source is loading</summary>
	public bool IsLoading { get; set; }

	/// <summary>Loading progress as a fraction from zero to one</summary>
	public double LoadFraction { get; set; }

	/// <summary>The scene names that currently display this source</summary>
	public List<string> SelectedScenes { get; set; } = [];
}

/// <summary>Read-only summary of an LDraw scene</summary>
public sealed class LDrawSceneInfo
{
	/// <summary>The scene view name</summary>
	public string Name { get; set; } = string.Empty;

	/// <summary>True if this scene is currently the active dock content</summary>
	public bool IsActive { get; set; }

	/// <summary>True if this scene is visible in the dock layout</summary>
	public bool IsVisible { get; set; }

	/// <summary>The number of objects visible in this scene</summary>
	public int ObjectCount { get; set; }

	/// <summary>The source context ids represented in this scene</summary>
	public List<string> ContextIds { get; set; } = [];

	/// <summary>The loaded user sources represented in this scene</summary>
	public List<LDrawSourceSummary> Sources { get; set; } = [];

	/// <summary>The visible bounds of this scene</summary>
	public LDrawBoundsInfo? Bounds { get; set; }

	/// <summary>The camera used by this scene</summary>
	public LDrawCameraInfo? Camera { get; set; }
}

/// <summary>Read-only list of scene views hosted by an LDraw instance</summary>
public sealed class LDrawSceneList
{
	/// <summary>The scenes returned by the query</summary>
	public List<LDrawSceneInfo> Scenes { get; set; } = [];

	/// <summary>The number of scenes returned</summary>
	public int Count
	{
		get { return Scenes.Count; }
	}
}

/// <summary>Three-component vector used by MCP payloads</summary>
public sealed class LDrawVector3
{
	/// <summary>X component</summary>
	public double X { get; set; }

	/// <summary>Y component</summary>
	public double Y { get; set; }

	/// <summary>Z component</summary>
	public double Z { get; set; }

	/// <summary>Create an MCP vector from a View3D vector</summary>
	public static LDrawVector3 From(v4 value)
	{
		return new LDrawVector3
		{
			X = value.x,
			Y = value.y,
			Z = value.z,
		};
	}

	/// <summary>Convert this MCP vector into a View3D point or direction</summary>
	public v4 ToV4(float w)
	{
		return new v4((float)X, (float)Y, (float)Z, w);
	}
}

/// <summary>Bounding-box data returned through MCP</summary>
public sealed class LDrawBoundsInfo
{
	/// <summary>True if this bounding box encloses valid finite geometry</summary>
	public bool IsValid { get; set; }

	/// <summary>Centre point of the bounding box</summary>
	public LDrawVector3 Centre { get; set; } = new();

	/// <summary>Half-size of the bounding box</summary>
	public LDrawVector3 Radius { get; set; } = new();

	/// <summary>Minimum corner of the bounding box</summary>
	public LDrawVector3 Min { get; set; } = new();

	/// <summary>Maximum corner of the bounding box</summary>
	public LDrawVector3 Max { get; set; } = new();

	/// <summary>Full size of the bounding box</summary>
	public LDrawVector3 Size { get; set; } = new();

	/// <summary>Create an MCP bounding-box DTO from a View3D bounding box</summary>
	public static LDrawBoundsInfo From(BBox bounds)
	{
		return new LDrawBoundsInfo
		{
			IsValid = bounds.IsValid,
			Centre = LDrawVector3.From(bounds.Centre),
			Radius = LDrawVector3.From(bounds.Radius),
			Min = LDrawVector3.From(bounds.Min),
			Max = LDrawVector3.From(bounds.Max),
			Size = new LDrawVector3
			{
				X = bounds.SizeX,
				Y = bounds.SizeY,
				Z = bounds.SizeZ,
			},
		};
	}
}

/// <summary>Camera data returned through MCP</summary>
public sealed class LDrawCameraInfo
{
	/// <summary>The scene that owns this camera</summary>
	public string SceneName { get; set; } = string.Empty;

	/// <summary>Camera position in world space</summary>
	public LDrawVector3 Position { get; set; } = new();

	/// <summary>Camera focus point in world space</summary>
	public LDrawVector3 FocusPoint { get; set; } = new();

	/// <summary>Distance from the camera to the focus point</summary>
	public double FocusDistance { get; set; }

	/// <summary>Camera up direction in world space</summary>
	public LDrawVector3 Up { get; set; } = new();

	/// <summary>Camera forward direction in world space</summary>
	public LDrawVector3 Forward { get; set; } = new();

	/// <summary>Horizontal and vertical field-of-view in radians</summary>
	public LDrawVector3 Fov { get; set; } = new();

	/// <summary>True when the camera is using orthographic projection</summary>
	public bool Orthographic { get; set; }

	/// <summary>The camera-to-world matrix in column-major order</summary>
	public double[] CameraToWorld { get; set; } = [];
}

/// <summary>Summary of one View3D object returned through MCP</summary>
public sealed class LDrawObjectInfo
{
	/// <summary>Opaque process-lifetime id for this object</summary>
	public string ObjectId { get; set; } = string.Empty;

	/// <summary>Object name from the LDraw/View3D object tree</summary>
	public string Name { get; set; } = string.Empty;

	/// <summary>LDraw object type</summary>
	public string Type { get; set; } = string.Empty;

	/// <summary>Context id for the source that owns this object</summary>
	public string ContextId { get; set; } = string.Empty;

	/// <summary>Opaque object id for the parent object, or empty for root objects</summary>
	public string ParentObjectId { get; set; } = string.Empty;

	/// <summary>Opaque object id for the root ancestor of this object</summary>
	public string RootObjectId { get; set; } = string.Empty;

	/// <summary>Stable path through the current object tree for diagnostics</summary>
	public string Path { get; set; } = string.Empty;

	/// <summary>Hierarchy depth relative to the enumerated root objects</summary>
	public int Depth { get; set; }

	/// <summary>True if this object is visible</summary>
	public bool Visible { get; set; }

	/// <summary>True if this object is currently selected in View3D</summary>
	public bool Selected { get; set; }

	/// <summary>Current object colour as AARRGGBB hex</summary>
	public string Colour { get; set; } = string.Empty;

	/// <summary>Base object colour as AARRGGBB hex</summary>
	public string BaseColour { get; set; } = string.Empty;

	/// <summary>The number of direct child objects</summary>
	public int ChildCount { get; set; }

	/// <summary>Model-space bounds for the object</summary>
	public LDrawBoundsInfo? ModelBounds { get; set; }

	/// <summary>Object-to-parent transform matrix in column-major order</summary>
	public double[] ObjectToParent { get; set; } = [];
}

/// <summary>Object-list result returned through MCP</summary>
public sealed class LDrawObjectList
{
	/// <summary>The scene that was queried</summary>
	public string SceneName { get; set; } = string.Empty;

	/// <summary>The objects returned by the query</summary>
	public List<LDrawObjectInfo> Objects { get; set; } = [];

	/// <summary>True if the object list was truncated by the requested limit</summary>
	public bool Truncated { get; set; }

	/// <summary>The number of objects returned</summary>
	public int Count
	{
		get { return Objects.Count; }
	}
}

/// <summary>One transient overlay owned by the MCP module</summary>
public sealed class LDrawOverlayInfo
{
	/// <summary>Caller-visible overlay id</summary>
	public string OverlayId { get; set; } = string.Empty;

	/// <summary>View3D context id for this overlay source</summary>
	public string ContextId { get; set; } = string.Empty;

	/// <summary>Display name assigned to the generated source</summary>
	public string Name { get; set; } = string.Empty;

	/// <summary>The scenes that currently show this overlay</summary>
	public List<string> SceneNames { get; set; } = [];

	/// <summary>The number of characters in the overlay script</summary>
	public int ScriptLength { get; set; }
}

/// <summary>Result from an overlay mutation command</summary>
public sealed class LDrawOverlayResult
{
	/// <summary>The operation that was performed</summary>
	public string Action { get; set; } = string.Empty;

	/// <summary>The overlay affected by the operation</summary>
	public LDrawOverlayInfo Overlay { get; set; } = new();

	/// <summary>The number of objects loaded from the overlay source</summary>
	public int ObjectCount { get; set; }
}

/// <summary>Result from a camera mutation command</summary>
public sealed class LDrawCameraSetResult
{
	/// <summary>The scene whose camera was changed</summary>
	public string SceneName { get; set; } = string.Empty;

	/// <summary>The operation that was performed</summary>
	public string Action { get; set; } = string.Empty;

	/// <summary>The resulting camera state</summary>
	public LDrawCameraInfo Camera { get; set; } = new();
}

/// <summary>Result from an object selection command</summary>
public sealed class LDrawSelectionResult
{
	/// <summary>The scene whose selection was read or changed</summary>
	public string SceneName { get; set; } = string.Empty;

	/// <summary>The operation that was performed</summary>
	public string Action { get; set; } = string.Empty;

	/// <summary>The selected objects after the operation</summary>
	public List<LDrawObjectInfo> Objects { get; set; } = [];

	/// <summary>Bounds of the selected objects, when any selected bounds are valid</summary>
	public LDrawBoundsInfo? Bounds { get; set; }

	/// <summary>The number of selected objects</summary>
	public int Count
	{
		get { return Objects.Count; }
	}
}

/// <summary>Result from an object or bounds framing command</summary>
public sealed class LDrawFrameResult
{
	/// <summary>The scene whose camera was changed</summary>
	public string SceneName { get; set; } = string.Empty;

	/// <summary>The operation that was performed</summary>
	public string Action { get; set; } = string.Empty;

	/// <summary>The objects used to determine the frame bounds</summary>
	public List<LDrawObjectInfo> Objects { get; set; } = [];

	/// <summary>The bounds used for camera framing</summary>
	public LDrawBoundsInfo? Bounds { get; set; }

	/// <summary>The resulting camera state</summary>
	public LDrawCameraInfo Camera { get; set; } = new();
}

/// <summary>Read-only view and rendering settings for a scene</summary>
public sealed class LDrawViewSettingsInfo
{
	/// <summary>The scene whose settings were queried</summary>
	public string SceneName { get; set; } = string.Empty;

	/// <summary>The scene camera</summary>
	public LDrawCameraInfo Camera { get; set; } = new();

	/// <summary>Current background colour as AARRGGBB hex</summary>
	public string BackgroundColour { get; set; } = string.Empty;

	/// <summary>True when the camera is using orthographic projection</summary>
	public bool Orthographic { get; set; }

	/// <summary>The camera align axis in world space</summary>
	public LDrawVector3 AlignAxis { get; set; } = new();

	/// <summary>The named align direction used by the view UI</summary>
	public string AlignDirection { get; set; } = string.Empty;

	/// <summary>The named view preset used by the view UI</summary>
	public string ViewPreset { get; set; } = string.Empty;

	/// <summary>Navigation mode name</summary>
	public string NavigationMode { get; set; } = string.Empty;

	/// <summary>Fill mode name</summary>
	public string FillMode { get; set; } = string.Empty;

	/// <summary>Cull mode name</summary>
	public string CullMode { get; set; } = string.Empty;

	/// <summary>True if anti-aliasing is enabled</summary>
	public bool Antialiasing { get; set; }

	/// <summary>True if the camera focus point marker is visible</summary>
	public bool FocusPointVisible { get; set; }

	/// <summary>True if the origin marker is visible</summary>
	public bool OriginPointVisible { get; set; }

	/// <summary>True if the selection bounding box is visible</summary>
	public bool SelectionBoxVisible { get; set; }

	/// <summary>True if hover object information is enabled</summary>
	public bool ObjectInfoEnabled { get; set; }

	/// <summary>True if diagnostic bounding boxes are visible</summary>
	public bool BBoxesVisible { get; set; }

	/// <summary>True if any object in the scene currently shows normals</summary>
	public bool ShowNormals { get; set; }

	/// <summary>Diagnostic normal vector length</summary>
	public double NormalsLength { get; set; }

	/// <summary>Diagnostic normal colour as AARRGGBB hex</summary>
	public string NormalsColour { get; set; } = string.Empty;

	/// <summary>True if ray tracing can be enabled for this scene</summary>
	public bool RayTracingAvailable { get; set; }

	/// <summary>True if ray tracing is enabled for this scene</summary>
	public bool RayTracingEnabled { get; set; }
}

/// <summary>Result from an object state mutation command</summary>
public sealed class LDrawObjectMutationResult
{
	/// <summary>The scene whose objects were changed</summary>
	public string SceneName { get; set; } = string.Empty;

	/// <summary>The operation that was performed</summary>
	public string Action { get; set; } = string.Empty;

	/// <summary>The objects after the operation</summary>
	public List<LDrawObjectInfo> Objects { get; set; } = [];

	/// <summary>The number of objects changed</summary>
	public int Count
	{
		get { return Objects.Count; }
	}
}

/// <summary>Parameters for scene-list requests</summary>
internal sealed class LDrawListScenesParams
{
	/// <summary>The scene to query, or all scenes when omitted</summary>
	public string? SceneName { get; set; }
}

/// <summary>Parameters for view-settings requests</summary>
internal sealed class LDrawViewSettingsParams
{
	/// <summary>The scene to query, or the first scene when omitted</summary>
	public string? SceneName { get; set; }
}

/// <summary>Parameters for object-list requests</summary>
internal sealed class LDrawListObjectsParams
{
	/// <summary>The scene to query, or the first scene when omitted</summary>
	public string? SceneName { get; set; }

	/// <summary>Optional source context id filter</summary>
	public string? ContextId { get; set; }

	/// <summary>True to recursively include child objects</summary>
	public bool IncludeChildren { get; set; }

	/// <summary>Maximum number of objects to return</summary>
	public int MaxCount { get; set; } = 200;
}

/// <summary>Shared parameters for object query commands</summary>
internal class LDrawObjectQueryParams
{
	/// <summary>The scene to query, or the first scene when omitted</summary>
	public string? SceneName { get; set; }

	/// <summary>Opaque object id returned by an earlier object query</summary>
	public string? ObjectId { get; set; }

	/// <summary>Object name filter</summary>
	public string? Name { get; set; }

	/// <summary>Object type filter</summary>
	public string? Type { get; set; }

	/// <summary>Optional source context id filter</summary>
	public string? ContextId { get; set; }

	/// <summary>Optional selected-state filter</summary>
	public bool? Selected { get; set; }

	/// <summary>Optional visible-state filter</summary>
	public bool? Visible { get; set; }

	/// <summary>True to recursively include child objects</summary>
	public bool IncludeChildren { get; set; } = true;

	/// <summary>Name/type matching mode: exact, contains, or regex</summary>
	public string MatchMode { get; set; } = "contains";

	/// <summary>True for case-sensitive name/type matching</summary>
	public bool CaseSensitive { get; set; }

	/// <summary>Maximum number of objects to return</summary>
	public int MaxCount { get; set; } = 200;
}

/// <summary>Parameters for single-object query commands</summary>
internal sealed class LDrawGetObjectParams : LDrawObjectQueryParams
{
}

/// <summary>Parameters for selection mutation commands</summary>
internal sealed class LDrawSelectObjectsParams : LDrawObjectQueryParams
{
	/// <summary>Selection operation: replace, add, remove, toggle, or clear</summary>
	public string Mode { get; set; } = "replace";
}

/// <summary>Parameters for object visibility mutation commands</summary>
internal sealed class LDrawSetObjectVisibilityParams : LDrawObjectQueryParams
{
	/// <summary>The visibility state to assign to matching objects</summary>
	public bool? SetVisible { get; set; }
}

/// <summary>Parameters for object colour mutation commands</summary>
internal sealed class LDrawSetObjectColourParams : LDrawObjectQueryParams
{
	/// <summary>Colour to apply, using any Colour32 parseable format</summary>
	public string? Colour { get; set; }

	/// <summary>True to reset current colour back to base colour instead of applying Colour</summary>
	public bool Reset { get; set; }

	/// <summary>True to modify base colour instead of current colour</summary>
	public bool BaseColour { get; set; }

	/// <summary>True to apply the colour operation recursively to children of each matched object</summary>
	public bool Recursive { get; set; } = true;
}

/// <summary>Parameters for selection read commands</summary>
internal sealed class LDrawGetSelectionParams
{
	/// <summary>The scene to query, or the first scene when omitted</summary>
	public string? SceneName { get; set; }

	/// <summary>True to recursively include child objects when enumerating selected descendants</summary>
	public bool IncludeChildren { get; set; } = true;

	/// <summary>Maximum number of selected objects to return</summary>
	public int MaxCount { get; set; } = 200;
}

/// <summary>Parameters for object framing commands</summary>
internal sealed class LDrawFrameObjectParams : LDrawObjectQueryParams
{
	/// <summary>True to replace the current selection with the framed objects</summary>
	public bool Select { get; set; }

	/// <summary>True to allow framing multiple query matches</summary>
	public bool AllowMultiple { get; set; }
}

/// <summary>Parameters for selection framing commands</summary>
internal sealed class LDrawFrameSelectionParams
{
	/// <summary>The scene to modify, or the first scene when omitted</summary>
	public string? SceneName { get; set; }

	/// <summary>Maximum number of selected objects to include in the response</summary>
	public int MaxCount { get; set; } = 200;
}

/// <summary>Parameters for explicit bounds framing commands</summary>
internal sealed class LDrawFrameBoundsParams
{
	/// <summary>The scene to modify, or the first scene when omitted</summary>
	public string? SceneName { get; set; }

	/// <summary>Minimum corner X, used with max_x/y/z</summary>
	public double? MinX { get; set; }

	/// <summary>Minimum corner Y, used with max_x/y/z</summary>
	public double? MinY { get; set; }

	/// <summary>Minimum corner Z, used with max_x/y/z</summary>
	public double? MinZ { get; set; }

	/// <summary>Maximum corner X, used with min_x/y/z</summary>
	public double? MaxX { get; set; }

	/// <summary>Maximum corner Y, used with min_x/y/z</summary>
	public double? MaxY { get; set; }

	/// <summary>Maximum corner Z, used with min_x/y/z</summary>
	public double? MaxZ { get; set; }

	/// <summary>Centre point X, used with radius_x/y/z</summary>
	public double? CentreX { get; set; }

	/// <summary>Centre point Y, used with radius_x/y/z</summary>
	public double? CentreY { get; set; }

	/// <summary>Centre point Z, used with radius_x/y/z</summary>
	public double? CentreZ { get; set; }

	/// <summary>Radius X, used with centre_x/y/z</summary>
	public double? RadiusX { get; set; }

	/// <summary>Radius Y, used with centre_x/y/z</summary>
	public double? RadiusY { get; set; }

	/// <summary>Radius Z, used with centre_x/y/z</summary>
	public double? RadiusZ { get; set; }
}

/// <summary>Parameters for camera read requests</summary>
internal sealed class LDrawCameraParams
{
	/// <summary>The scene to query, or the first scene when omitted</summary>
	public string? SceneName { get; set; }
}

/// <summary>Parameters for camera mutation requests</summary>
internal sealed class LDrawSetCameraParams
{
	/// <summary>The scene to modify, or the first scene when omitted</summary>
	public string? SceneName { get; set; }

	/// <summary>True to frame the visible scene bounds</summary>
	public bool FrameScene { get; set; }

	/// <summary>Camera position for look-at mode</summary>
	public LDrawVector3? Position { get; set; }

	/// <summary>Camera target for look-at mode</summary>
	public LDrawVector3? LookAt { get; set; }

	/// <summary>Camera up direction for look-at mode</summary>
	public LDrawVector3? Up { get; set; }
}

/// <summary>Parameters for overlay script mutation requests</summary>
internal sealed class LDrawOverlayScriptParams
{
	/// <summary>Caller-visible overlay id, defaults to "default"</summary>
	public string? OverlayId { get; set; }

	/// <summary>Display name assigned to the generated overlay source</summary>
	public string? Name { get; set; }

	/// <summary>Raw .ldr script to set or append</summary>
	public string Script { get; set; } = string.Empty;

	/// <summary>Names of scenes that should show this overlay; omitted means the first scene</summary>
	public List<string> SceneNames { get; set; } = [];

	/// <summary>True to frame the target scenes after loading the overlay</summary>
	public bool ResetView { get; set; }
}

/// <summary>Parameters for overlay clear requests</summary>
internal sealed class LDrawOverlayClearParams
{
	/// <summary>Caller-visible overlay id, or null to clear all overlays</summary>
	public string? OverlayId { get; set; }
}

/// <summary>Local named-pipe request between the broker and an LDraw instance</summary>
internal sealed class InstancePipeRequest
{
	/// <summary>Version of the pipe request schema</summary>
	public int SchemaVersion { get; set; } = 1;

	/// <summary>The command to execute</summary>
	public string Command { get; set; } = string.Empty;

	/// <summary>Command-specific parameters</summary>
	public JsonElement? Parameters { get; set; }
}

/// <summary>Local named-pipe response between the broker and an LDraw instance</summary>
internal sealed class InstancePipeResponse
{
	/// <summary>Version of the pipe response schema</summary>
	public int SchemaVersion { get; set; } = 1;

	/// <summary>True if the command completed successfully</summary>
	public bool Success { get; set; }

	/// <summary>Error message when Success is false</summary>
	public string Error { get; set; } = string.Empty;

	/// <summary>Command-specific response payload</summary>
	public JsonElement? Payload { get; set; }
}

/// <summary>Command names supported by the local instance pipe</summary>
internal static class InstancePipeCommands
{
	public const string GetSceneSummary = "get_scene_summary";
	public const string ListScenes = "list_scenes";
	public const string GetViewSettings = "get_view_settings";
	public const string ListObjects = "list_objects";
	public const string GetCamera = "get_camera";
	public const string SetCamera = "set_camera";
	public const string FindObjects = "find_objects";
	public const string GetObject = "get_object";
	public const string SelectObjects = "select_objects";
	public const string SetObjectVisibility = "set_object_visibility";
	public const string SetObjectColour = "set_object_colour";
	public const string GetSelection = "get_selection";
	public const string FrameObject = "frame_object";
	public const string FrameSelection = "frame_selection";
	public const string FrameBounds = "frame_bounds";
	public const string OverlaySetScript = "overlay_set_script";
	public const string OverlayAppendScript = "overlay_append_script";
	public const string OverlayClear = "overlay_clear";
}
