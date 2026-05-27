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

/// <summary>Read-only list of LDraw sources hosted by an instance</summary>
public sealed class LDrawSourceList
{
	/// <summary>The sources returned by the query</summary>
	public List<LDrawSourceSummary> Sources { get; set; } = [];

	/// <summary>The number of sources returned</summary>
	public int Count
	{
		get { return Sources.Count; }
	}
}

/// <summary>Result from a source lifecycle or scene-membership command</summary>
public sealed class LDrawSourceMutationResult
{
	/// <summary>The operation that was performed</summary>
	public string Action { get; set; } = string.Empty;

	/// <summary>The primary source affected by the operation, when there is one</summary>
	public LDrawSourceSummary? Source { get; set; }

	/// <summary>The affected sources after the operation</summary>
	public List<LDrawSourceSummary> Sources { get; set; } = [];

	/// <summary>The scenes after the operation</summary>
	public List<LDrawSceneInfo> Scenes { get; set; } = [];
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

	/// <summary>True if MCP can safely close this scene</summary>
	public bool CanCloseByMcp { get; set; }
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

/// <summary>Result from a scene lifecycle or source-membership command</summary>
public sealed class LDrawSceneMutationResult
{
	/// <summary>The operation that was performed</summary>
	public string Action { get; set; } = string.Empty;

	/// <summary>The primary scene affected by the operation</summary>
	public string SceneName { get; set; } = string.Empty;

	/// <summary>The affected scene after the operation, or the pre-close scene for close operations</summary>
	public LDrawSceneInfo? Scene { get; set; }

	/// <summary>The scenes after the operation</summary>
	public List<LDrawSceneInfo> Scenes { get; set; } = [];

	/// <summary>The MCP overlays whose scene membership changed</summary>
	public List<LDrawOverlayInfo> Overlays { get; set; } = [];
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

/// <summary>Chart axis range data returned through MCP</summary>
public sealed class LDrawAxisRangeInfo
{
	/// <summary>Minimum axis value</summary>
	public double Min { get; set; }

	/// <summary>Maximum axis value</summary>
	public double Max { get; set; }

	/// <summary>Axis span</summary>
	public double Span { get; set; }

	/// <summary>Create MCP axis range data from a ChartControl axis</summary>
	internal static LDrawAxisRangeInfo From(Rylogic.Gui.WPF.ChartControl.RangeData.Axis axis)
	{
		return new LDrawAxisRangeInfo
		{
			Min = axis.Min,
			Max = axis.Max,
			Span = axis.Span,
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

	/// <summary>Object-to-world transform matrix in column-major order</summary>
	public double[] ObjectToWorld { get; set; } = [];

	/// <summary>Current object reflectivity</summary>
	public double Reflectivity { get; set; }

	/// <summary>True if this object renders in wireframe mode</summary>
	public bool Wireframe { get; set; }

	/// <summary>True if this object renders normal vectors</summary>
	public bool ShowNormals { get; set; }

	/// <summary>Object render sort group name</summary>
	public string SortGroup { get; set; } = string.Empty;

	/// <summary>Object LDraw flags</summary>
	public string Flags { get; set; } = string.Empty;

	/// <summary>First nugget render flags</summary>
	public string NuggetFlags { get; set; } = string.Empty;

	/// <summary>First nugget tint colour as AARRGGBB hex</summary>
	public string NuggetTint { get; set; } = string.Empty;
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

/// <summary>Hit-test result returned through MCP</summary>
public sealed class LDrawHitTestInfo
{
	/// <summary>The scene that was hit tested</summary>
	public string SceneName { get; set; } = string.Empty;

	/// <summary>True if the ray hit an object</summary>
	public bool IsHit { get; set; }

	/// <summary>The cast ray origin in world space</summary>
	public LDrawVector3 RayOrigin { get; set; } = new();

	/// <summary>The cast ray direction in world space</summary>
	public LDrawVector3 RayDirection { get; set; } = new();

	/// <summary>The hit point in world space, or zero when no object was hit</summary>
	public LDrawVector3 Intercept { get; set; } = new();

	/// <summary>The hit normal in world space, or zero when no object was hit</summary>
	public LDrawVector3 Normal { get; set; } = new();

	/// <summary>Distance from RayOrigin to Intercept</summary>
	public double Distance { get; set; }

	/// <summary>Snap type applied to the hit point</summary>
	public string SnapType { get; set; } = string.Empty;

	/// <summary>The native ray index</summary>
	public int RayIndex { get; set; }

	/// <summary>The native ray id</summary>
	public int RayId { get; set; }

	/// <summary>The object hit by the ray, when IsHit is true</summary>
	public LDrawObjectInfo? Object { get; set; }
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

	/// <summary>The current chart X axis range</summary>
	public LDrawAxisRangeInfo XAxisRange { get; set; } = new();

	/// <summary>The current chart Y axis range</summary>
	public LDrawAxisRangeInfo YAxisRange { get; set; } = new();

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

	/// <summary>Uniform point size used when the fill mode is Points</summary>
	public double FillModePointsSize { get; set; }

	/// <summary>True if ray tracing can be enabled for this scene</summary>
	public bool RayTracingAvailable { get; set; }

	/// <summary>True if ray tracing is enabled for this scene</summary>
	public bool RayTracingEnabled { get; set; }

	/// <summary>World-space range used when casting shadows; zero disables shadow casting</summary>
	public double ShadowCastRange { get; set; }
}

/// <summary>Diagnostic and rendering modes for a scene</summary>
public sealed class LDrawDiagnosticModesInfo
{
	/// <summary>The scene whose diagnostic modes were queried</summary>
	public string SceneName { get; set; } = string.Empty;

	/// <summary>The current view settings containing the active diagnostic values</summary>
	public LDrawViewSettingsInfo ViewSettings { get; set; } = new();

	/// <summary>Fill mode names accepted by ldraw_set_diagnostic_modes</summary>
	public List<string> AvailableFillModes { get; set; } = [];

	/// <summary>Cull mode names accepted by ldraw_set_diagnostic_modes</summary>
	public List<string> AvailableCullModes { get; set; } = [];

	/// <summary>Modes that are existing runtime diagnostics rather than persisted scene profile values</summary>
	public List<string> SessionLocalModes { get; set; } = [];
}

/// <summary>Result from a view-setting mutation command</summary>
public sealed class LDrawViewMutationResult
{
	/// <summary>The scene whose view settings changed</summary>
	public string SceneName { get; set; } = string.Empty;

	/// <summary>The operation that was performed</summary>
	public string Action { get; set; } = string.Empty;

	/// <summary>The resulting view settings</summary>
	public LDrawViewSettingsInfo ViewSettings { get; set; } = new();
}

/// <summary>Scene capture result returned through MCP</summary>
public sealed class LDrawSceneCaptureResult
{
	/// <summary>The scene that was captured</summary>
	public string SceneName { get; set; } = string.Empty;

	/// <summary>The image file written by the command</summary>
	public string OutputPath { get; set; } = string.Empty;

	/// <summary>The captured image format inferred from the extension</summary>
	public string Format { get; set; } = string.Empty;

	/// <summary>Captured image width in pixels</summary>
	public int Width { get; set; }

	/// <summary>Captured image height in pixels</summary>
	public int Height { get; set; }

	/// <summary>Number of bytes written to OutputPath</summary>
	public long ByteCount { get; set; }
}

/// <summary>Chart display options for a scene</summary>
public sealed class LDrawChartDisplayOptionsInfo
{
	/// <summary>The scene whose chart display options were queried</summary>
	public string SceneName { get; set; } = string.Empty;

	/// <summary>True if chart axes are visible</summary>
	public bool ShowAxes { get; set; }

	/// <summary>True if X or Y grid lines are visible</summary>
	public bool ShowGridLines { get; set; }

	/// <summary>True if X-axis tick marks are drawn</summary>
	public bool XDrawTickMarks { get; set; }

	/// <summary>True if Y-axis tick marks are drawn</summary>
	public bool YDrawTickMarks { get; set; }

	/// <summary>True if X-axis tick labels are drawn</summary>
	public bool XDrawTickLabels { get; set; }

	/// <summary>True if Y-axis tick labels are drawn</summary>
	public bool YDrawTickLabels { get; set; }

	/// <summary>X-axis preferred spacing between ticks in pixels</summary>
	public double XPixelsPerTick { get; set; }

	/// <summary>Y-axis preferred spacing between ticks in pixels</summary>
	public double YPixelsPerTick { get; set; }

	/// <summary>X-axis line colour as AARRGGBB hex</summary>
	public string XAxisColour { get; set; } = string.Empty;

	/// <summary>Y-axis line colour as AARRGGBB hex</summary>
	public string YAxisColour { get; set; } = string.Empty;

	/// <summary>X-axis tick label colour as AARRGGBB hex</summary>
	public string XTickColour { get; set; } = string.Empty;

	/// <summary>Y-axis tick label colour as AARRGGBB hex</summary>
	public string YTickColour { get; set; } = string.Empty;
}

/// <summary>Result from a chart display option mutation command</summary>
public sealed class LDrawChartDisplayOptionsResult
{
	/// <summary>The operation that was performed</summary>
	public string Action { get; set; } = string.Empty;

	/// <summary>The resulting display options</summary>
	public LDrawChartDisplayOptionsInfo Options { get; set; } = new();
}

/// <summary>Read-only animation state for a scene</summary>
public sealed class LDrawAnimationInfo
{
	/// <summary>The scene whose animation state was queried</summary>
	public string SceneName { get; set; } = string.Empty;

	/// <summary>The current animation clock value in seconds</summary>
	public double TimeS { get; set; }

	/// <summary>True while the scene animation clock is running</summary>
	public bool Animating { get; set; }

	/// <summary>Animation commands accepted by ldraw_control_animation</summary>
	public List<string> AvailableCommands { get; set; } = [];
}

/// <summary>Result from changing scene animation state</summary>
public sealed class LDrawAnimationResult
{
	/// <summary>The operation that was performed</summary>
	public string Action { get; set; } = string.Empty;

	/// <summary>The resulting animation state</summary>
	public LDrawAnimationInfo Animation { get; set; } = new();
}

/// <summary>One named view preset accepted by the view-preset command</summary>
public sealed class LDrawViewPresetInfo
{
	/// <summary>The preset name</summary>
	public string Name { get; set; } = string.Empty;

	/// <summary>True when this preset is the scene's current named preset</summary>
	public bool IsCurrent { get; set; }
}

/// <summary>Read-only list of view presets for a scene</summary>
public sealed class LDrawViewPresetList
{
	/// <summary>The scene whose view presets were queried</summary>
	public string SceneName { get; set; } = string.Empty;

	/// <summary>The scene's current named view preset</summary>
	public string Current { get; set; } = string.Empty;

	/// <summary>The presets accepted by ldraw_set_view_preset</summary>
	public List<LDrawViewPresetInfo> Presets { get; set; } = [];

	/// <summary>The number of presets returned</summary>
	public int Count
	{
		get { return Presets.Count; }
	}
}

/// <summary>One saved camera view for a scene</summary>
public sealed class LDrawSavedViewInfo
{
	/// <summary>The saved view name</summary>
	public string Name { get; set; } = string.Empty;

	/// <summary>Distance from the camera to the focus point</summary>
	public double FocusDistance { get; set; }

	/// <summary>The saved camera align axis in world space</summary>
	public LDrawVector3 AlignAxis { get; set; } = new();

	/// <summary>Horizontal and vertical field-of-view in radians</summary>
	public LDrawVector3 Fov { get; set; } = new();

	/// <summary>True when the saved view uses orthographic projection</summary>
	public bool Orthographic { get; set; }

	/// <summary>The saved camera-to-world matrix in column-major order</summary>
	public double[] CameraToWorld { get; set; } = [];
}

/// <summary>Read-only saved-view list for a scene</summary>
public sealed class LDrawSavedViewList
{
	/// <summary>The scene whose saved views were queried</summary>
	public string SceneName { get; set; } = string.Empty;

	/// <summary>The currently selected saved view name, when one is selected</summary>
	public string CurrentName { get; set; } = string.Empty;

	/// <summary>The saved views in the scene</summary>
	public List<LDrawSavedViewInfo> Views { get; set; } = [];

	/// <summary>The number of saved views returned</summary>
	public int Count
	{
		get { return Views.Count; }
	}
}

/// <summary>Result from saving, applying, or removing a saved view</summary>
public sealed class LDrawSavedViewResult
{
	/// <summary>The scene whose saved views changed</summary>
	public string SceneName { get; set; } = string.Empty;

	/// <summary>The operation that was performed</summary>
	public string Action { get; set; } = string.Empty;

	/// <summary>The affected saved view, when one remains after the operation</summary>
	public LDrawSavedViewInfo? View { get; set; }

	/// <summary>The saved views after the operation</summary>
	public List<LDrawSavedViewInfo> Views { get; set; } = [];

	/// <summary>The resulting camera state for the scene</summary>
	public LDrawCameraInfo Camera { get; set; } = new();
}

/// <summary>Runtime streaming state for the View3D engine</summary>
public sealed class LDrawStreamingInfo
{
	/// <summary>The operation that produced this state</summary>
	public string Action { get; set; } = string.Empty;

	/// <summary>The current native streaming state</summary>
	public string State { get; set; } = string.Empty;

	/// <summary>True when streaming is listening or connected</summary>
	public bool IsActive { get; set; }

	/// <summary>The port currently requested for streaming while active, when known</summary>
	public int? Port { get; set; }

	/// <summary>The default port from the current LDraw profile</summary>
	public int DefaultPort { get; set; }
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

/// <summary>Result from drawing object bounds through an MCP overlay</summary>
public sealed class LDrawBoundsOverlayResult
{
	/// <summary>The scene that received the bounds overlay</summary>
	public string SceneName { get; set; } = string.Empty;

	/// <summary>The operation that was performed</summary>
	public string Action { get; set; } = string.Empty;

	/// <summary>The overlay source that contains the generated bounds</summary>
	public LDrawOverlayInfo Overlay { get; set; } = new();

	/// <summary>The objects whose bounds were drawn</summary>
	public List<LDrawObjectInfo> Objects { get; set; } = [];

	/// <summary>The world-space AABBs drawn by the overlay</summary>
	public List<LDrawBoundsInfo> Bounds { get; set; } = [];

	/// <summary>The number of bounds boxes drawn</summary>
	public int Count
	{
		get { return Bounds.Count; }
	}
}

/// <summary>Result from changing chart axis ranges</summary>
public sealed class LDrawAxisRangeResult
{
	/// <summary>The scene whose chart axes changed</summary>
	public string SceneName { get; set; } = string.Empty;

	/// <summary>The operation that was performed</summary>
	public string Action { get; set; } = string.Empty;

	/// <summary>The resulting X axis range</summary>
	public LDrawAxisRangeInfo XAxisRange { get; set; } = new();

	/// <summary>The resulting Y axis range</summary>
	public LDrawAxisRangeInfo YAxisRange { get; set; } = new();

	/// <summary>The resulting camera state after applying the axis range</summary>
	public LDrawCameraInfo Camera { get; set; } = new();
}

/// <summary>Series definition accepted by chart commands</summary>
public sealed class LDrawChartSeriesInput
{
	/// <summary>Series display name</summary>
	public string? Name { get; set; }

	/// <summary>Series colour, using any Colour32 parseable format</summary>
	public string? Colour { get; set; }

	/// <summary>X-axis expression, for example C0 or CI</summary>
	public string? XAxis { get; set; }

	/// <summary>Y-axis expression, for example C1 or abs(C2 - C1)</summary>
	public string? YAxis { get; set; }

	/// <summary>Optional line width</summary>
	public double? Width { get; set; }

	/// <summary>True to smooth the line</summary>
	public bool Smooth { get; set; }
}

/// <summary>Chart series data returned through MCP</summary>
public sealed class LDrawChartSeriesInfo
{
	/// <summary>Series display name</summary>
	public string Name { get; set; } = string.Empty;

	/// <summary>Series colour as AARRGGBB hex</summary>
	public string Colour { get; set; } = string.Empty;

	/// <summary>X-axis expression</summary>
	public string XAxis { get; set; } = string.Empty;

	/// <summary>Y-axis expression</summary>
	public string YAxis { get; set; } = string.Empty;

	/// <summary>Line width, when explicitly set</summary>
	public double? Width { get; set; }

	/// <summary>True if line smoothing is enabled</summary>
	public bool Smooth { get; set; }
}

/// <summary>Chart data owned by the MCP module</summary>
public sealed class LDrawChartInfo
{
	/// <summary>Caller-visible chart id</summary>
	public string ChartId { get; set; } = string.Empty;

	/// <summary>Chart display name</summary>
	public string Name { get; set; } = string.Empty;

	/// <summary>Chart colour as AARRGGBB hex</summary>
	public string Colour { get; set; } = string.Empty;

	/// <summary>Number of data columns</summary>
	public int Columns { get; set; }

	/// <summary>Number of data rows</summary>
	public int Rows { get; set; }

	/// <summary>Series displayed by the chart</summary>
	public List<LDrawChartSeriesInfo> Series { get; set; } = [];

	/// <summary>The scenes that currently show this chart overlay</summary>
	public List<string> SceneNames { get; set; } = [];

	/// <summary>The MCP overlay that contains the raw chart script</summary>
	public LDrawOverlayInfo Overlay { get; set; } = new();
}

/// <summary>Result from chart commands</summary>
public sealed class LDrawChartResult
{
	/// <summary>The operation that was performed</summary>
	public string Action { get; set; } = string.Empty;

	/// <summary>The chart affected by the operation</summary>
	public LDrawChartInfo? Chart { get; set; }

	/// <summary>The charts affected by the operation</summary>
	public List<LDrawChartInfo> Charts { get; set; } = [];

	/// <summary>The overlay affected by the operation, if any</summary>
	public LDrawOverlayInfo? Overlay { get; set; }

	/// <summary>The overlays affected by the operation</summary>
	public List<LDrawOverlayInfo> Overlays { get; set; } = [];
}

/// <summary>Parameters for scene-list requests</summary>
internal sealed class LDrawListScenesParams
{
	/// <summary>The scene to query, or all scenes when omitted</summary>
	public string? SceneName { get; set; }
}

/// <summary>Parameters for source-list requests</summary>
internal sealed class LDrawListSourcesParams
{
	/// <summary>Optional scene filter</summary>
	public string? SceneName { get; set; }
}

/// <summary>Parameters for loading file-backed sources</summary>
internal sealed class LDrawLoadSourceParams
{
	/// <summary>The LDraw script file to load</summary>
	public string? FilePath { get; set; }

	/// <summary>Scenes to show the source in, or the first scene when empty</summary>
	public List<string> SceneNames { get; set; } = [];

	/// <summary>True to show the source in every current scene</summary>
	public bool AllScenes { get; set; }
}

/// <summary>Parameters for source lifecycle requests</summary>
internal class LDrawSourceParams
{
	/// <summary>Source context ids, display names, or file paths to target</summary>
	public List<string> SourceIds { get; set; } = [];

	/// <summary>True to target every current user source</summary>
	public bool AllSources { get; set; }
}

/// <summary>Parameters for user-source scene membership requests</summary>
internal sealed class LDrawSetSourceScenesParams : LDrawSourceParams
{
	/// <summary>Membership operation: replace, add, remove, or clear</summary>
	public string Mode { get; set; } = "replace";

	/// <summary>Scenes whose user-source membership is modified, or the first scene when empty</summary>
	public List<string> SceneNames { get; set; } = [];

	/// <summary>True to target every current scene</summary>
	public bool AllScenes { get; set; }

	/// <summary>True to frame each scene after adding source objects</summary>
	public bool ResetView { get; set; }
}

/// <summary>Parameters for view-settings requests</summary>
internal sealed class LDrawViewSettingsParams
{
	/// <summary>The scene to query, or the first scene when omitted</summary>
	public string? SceneName { get; set; }
}

/// <summary>Parameters for animation-state requests</summary>
internal class LDrawAnimationParams
{
	/// <summary>The scene to query or modify, or the first scene when omitted</summary>
	public string? SceneName { get; set; }
}

/// <summary>Parameters for animation-control requests</summary>
internal sealed class LDrawAnimationControlParams : LDrawAnimationParams
{
	/// <summary>Animation command: reset, play, stop, or step</summary>
	public string? Command { get; set; }

	/// <summary>Animation command time value in seconds</summary>
	public double? TimeS { get; set; }
}

/// <summary>Parameters for view-preset requests</summary>
internal sealed class LDrawViewPresetParams
{
	/// <summary>The scene to query or modify, or the first scene when omitted</summary>
	public string? SceneName { get; set; }

	/// <summary>Named view preset to apply</summary>
	public string? ViewPreset { get; set; }
}

/// <summary>Parameters for saved-view requests</summary>
internal sealed class LDrawSavedViewParams
{
	/// <summary>The scene to query or modify, or the first scene when omitted</summary>
	public string? SceneName { get; set; }

	/// <summary>Saved view name</summary>
	public string? Name { get; set; }

	/// <summary>True to replace an existing saved view with the same name</summary>
	public bool Replace { get; set; }
}

/// <summary>Parameters for streaming control requests</summary>
internal sealed class LDrawStreamingControlParams
{
	/// <summary>True to enable streaming, false to disable it</summary>
	public bool Enable { get; set; }

	/// <summary>Optional runtime streaming port. Omit to use the current profile default.</summary>
	public int? Port { get; set; }
}

/// <summary>Parameters for projection mutation requests</summary>
internal sealed class LDrawSetProjectionParams
{
	/// <summary>The scene to modify, or the first scene when omitted</summary>
	public string? SceneName { get; set; }

	/// <summary>True for orthographic projection; false for perspective</summary>
	public bool Orthographic { get; set; }
}

/// <summary>Parameters for background-colour mutation requests</summary>
internal sealed class LDrawSetBackgroundColourParams
{
	/// <summary>The scene to modify, or the first scene when omitted</summary>
	public string? SceneName { get; set; }

	/// <summary>Colour to apply, using any Colour32 parseable format</summary>
	public string? Colour { get; set; }
}

/// <summary>Parameters for camera align-axis mutation requests</summary>
internal sealed class LDrawSetCameraAlignAxisParams
{
	/// <summary>The scene to modify, or the first scene when omitted</summary>
	public string? SceneName { get; set; }

	/// <summary>Named align direction: None, PosX, NegX, PosY, NegY, PosZ, or NegZ</summary>
	public string? AlignDirection { get; set; }
}

/// <summary>Parameters for chart-axis range mutation requests</summary>
internal class LDrawSetAxisRangesParams
{
	/// <summary>The scene to modify, or the first scene when omitted</summary>
	public string? SceneName { get; set; }

	/// <summary>Minimum X axis value</summary>
	public double? XMin { get; set; }

	/// <summary>Maximum X axis value</summary>
	public double? XMax { get; set; }

	/// <summary>Minimum Y axis value</summary>
	public double? YMin { get; set; }

	/// <summary>Maximum Y axis value</summary>
	public double? YMax { get; set; }
}

/// <summary>Parameters for diagnostic mode requests</summary>
internal class LDrawDiagnosticModesParams
{
	/// <summary>The scene to query or modify, or the first scene when omitted</summary>
	public string? SceneName { get; set; }
}

/// <summary>Parameters for diagnostic mode mutation requests</summary>
internal sealed class LDrawSetDiagnosticModesParams : LDrawDiagnosticModesParams
{
	/// <summary>Show native diagnostic bounding boxes for all objects</summary>
	public bool? BBoxesVisible { get; set; }

	/// <summary>Show normals for all objects in the scene</summary>
	public bool? ShowNormals { get; set; }

	/// <summary>Diagnostic normal vector length</summary>
	public double? NormalsLength { get; set; }

	/// <summary>Diagnostic normal colour as AARRGGBB hex or a Colour32 parseable value</summary>
	public string? NormalsColour { get; set; }

	/// <summary>Scene fill mode name</summary>
	public string? FillMode { get; set; }

	/// <summary>Scene cull mode name</summary>
	public string? CullMode { get; set; }

	/// <summary>Uniform point size used when FillMode is Points</summary>
	public double? FillModePointsSize { get; set; }

	/// <summary>Show the camera focus point marker</summary>
	public bool? FocusPointVisible { get; set; }

	/// <summary>Show the origin marker</summary>
	public bool? OriginPointVisible { get; set; }

	/// <summary>Show the selection bounding box</summary>
	public bool? SelectionBoxVisible { get; set; }

	/// <summary>Enable hover object information</summary>
	public bool? ObjectInfoEnabled { get; set; }
}

/// <summary>Parameters for render-setting mutation requests</summary>
internal sealed class LDrawSetRenderSettingsParams
{
	/// <summary>The scene to modify, or the first scene when omitted</summary>
	public string? SceneName { get; set; }

	/// <summary>True to enable multi-sample anti-aliasing for the scene view</summary>
	public bool? Antialiasing { get; set; }

	/// <summary>World-space range used when casting shadows; zero disables shadow casting</summary>
	public double? ShadowCastRange { get; set; }

	/// <summary>True to enable ray tracing when available</summary>
	public bool? RayTracingEnabled { get; set; }
}

/// <summary>Parameters for scene creation requests</summary>
internal sealed class LDrawCreateSceneParams
{
	/// <summary>The requested scene name, or a generated name when omitted</summary>
	public string? SceneName { get; set; }

	/// <summary>True to make the created scene active</summary>
	public bool Activate { get; set; } = true;

	/// <summary>MCP overlay ids to show in the new scene</summary>
	public List<string> OverlayIds { get; set; } = [];

	/// <summary>True to show all current MCP overlays in the new scene</summary>
	public bool AllOverlays { get; set; }

	/// <summary>True to frame the scene after adding overlay sources</summary>
	public bool ResetView { get; set; }
}

/// <summary>Parameters for scene-only mutation requests</summary>
internal sealed class LDrawSceneParams
{
	/// <summary>The scene to mutate</summary>
	public string? SceneName { get; set; }
}

/// <summary>Parameters for scene rename requests</summary>
internal sealed class LDrawRenameSceneParams
{
	/// <summary>The scene to rename</summary>
	public string? SceneName { get; set; }

	/// <summary>The new unique scene name</summary>
	public string? NewSceneName { get; set; }
}

/// <summary>Parameters for MCP overlay source membership requests</summary>
internal sealed class LDrawSetSceneSourcesParams
{
	/// <summary>The scene whose MCP overlay membership is modified</summary>
	public string? SceneName { get; set; }

	/// <summary>Membership operation: replace, add, remove, or clear</summary>
	public string Mode { get; set; } = "replace";

	/// <summary>MCP overlay ids to use for replace/add/remove operations</summary>
	public List<string> OverlayIds { get; set; } = [];

	/// <summary>True to target all current MCP overlays</summary>
	public bool AllOverlays { get; set; }

	/// <summary>True to frame the scene after adding overlay sources</summary>
	public bool ResetView { get; set; }
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

/// <summary>Parameters for object normal display commands</summary>
internal sealed class LDrawShowNormalsParams : LDrawObjectQueryParams
{
	/// <summary>The normal visibility state to assign to matching objects</summary>
	public bool SetShowNormals { get; set; } = true;

	/// <summary>True to apply the state recursively to matched object children</summary>
	public bool Recursive { get; set; } = true;
}

/// <summary>Parameters for object bounds overlay commands</summary>
internal sealed class LDrawShowObjectBoundsParams : LDrawObjectQueryParams
{
	/// <summary>Overlay id to replace with the generated bounds</summary>
	public string? OverlayId { get; set; }

	/// <summary>Display name for the generated overlay source</summary>
	public string? OverlayName { get; set; }

	/// <summary>Bounds colour, using any Colour32 parseable format</summary>
	public string? Colour { get; set; }

	/// <summary>Line width in pixels</summary>
	public double LineWidth { get; set; } = 2.0;
}

/// <summary>Parameters for object transform read commands</summary>
internal sealed class LDrawGetObjectTransformParams : LDrawObjectQueryParams
{
}

/// <summary>Parameters for object transform mutation commands</summary>
internal sealed class LDrawSetObjectTransformParams : LDrawObjectQueryParams
{
	/// <summary>Transform space: parent or world</summary>
	public string Space { get; set; } = "parent";

	/// <summary>Optional replacement matrix in column-major order</summary>
	public double[]? Matrix { get; set; }

	/// <summary>Optional replacement X position in the selected transform space</summary>
	public double? PositionX { get; set; }

	/// <summary>Optional replacement Y position in the selected transform space</summary>
	public double? PositionY { get; set; }

	/// <summary>Optional replacement Z position in the selected transform space</summary>
	public double? PositionZ { get; set; }

	/// <summary>Optional X translation delta in the selected transform space</summary>
	public double? DeltaX { get; set; }

	/// <summary>Optional Y translation delta in the selected transform space</summary>
	public double? DeltaY { get; set; }

	/// <summary>Optional Z translation delta in the selected transform space</summary>
	public double? DeltaZ { get; set; }
}

/// <summary>Parameters for object render-state read commands</summary>
internal sealed class LDrawGetObjectRenderStateParams : LDrawObjectQueryParams
{
}

/// <summary>Parameters for object render-state mutation commands</summary>
internal sealed class LDrawSetObjectRenderStateParams : LDrawObjectQueryParams
{
	/// <summary>Optional visibility state</summary>
	public bool? VisibleState { get; set; }

	/// <summary>Optional wireframe state</summary>
	public bool? Wireframe { get; set; }

	/// <summary>Optional per-object normal display state</summary>
	public bool? ShowNormals { get; set; }

	/// <summary>Optional object reflectivity value</summary>
	public double? Reflectivity { get; set; }

	/// <summary>Optional View3D sort group name</summary>
	public string? SortGroup { get; set; }

	/// <summary>Optional first-nugget tint colour</summary>
	public string? NuggetTint { get; set; }

	/// <summary>Optional object scene-bounds exclusion flag</summary>
	public bool? SceneBoundsExcluded { get; set; }

	/// <summary>Optional object hit-test exclusion flag</summary>
	public bool? HitTestExcluded { get; set; }

	/// <summary>Optional object shadow-cast exclusion flag</summary>
	public bool? ShadowCastExcluded { get; set; }

	/// <summary>Optional object no-depth-test flag</summary>
	public bool? NoZTest { get; set; }

	/// <summary>Optional object no-depth-write flag</summary>
	public bool? NoZWrite { get; set; }

	/// <summary>Optional first-nugget hidden flag</summary>
	public bool? NuggetHidden { get; set; }

	/// <summary>Optional first-nugget alpha-blend flag</summary>
	public bool? NuggetAlphaBlend { get; set; }

	/// <summary>True to apply recursive state setters to matched object children</summary>
	public bool Recursive { get; set; } = true;
}

/// <summary>Parameters for scene hit-test commands</summary>
internal sealed class LDrawHitTestParams : LDrawObjectQueryParams
{
	/// <summary>Screen-space X coordinate relative to the scene viewport</summary>
	public double? X { get; set; }

	/// <summary>Screen-space Y coordinate relative to the scene viewport</summary>
	public double? Y { get; set; }

	/// <summary>Snap mode: NoSnap, Verts, Edges, Faces, All, AllPerspective, or a flags combination</summary>
	public string? SnapMode { get; set; }

	/// <summary>Snap distance in screen-space pixels</summary>
	public double SnapDistance { get; set; }
}

/// <summary>Parameters for scene capture commands</summary>
internal sealed class LDrawCaptureSceneParams
{
	/// <summary>The scene to capture, or the first scene when omitted</summary>
	public string? SceneName { get; set; }

	/// <summary>Output image file path. The extension selects png, jpg, or bmp.</summary>
	public string? OutputPath { get; set; }

	/// <summary>True to overwrite an existing output file</summary>
	public bool Overwrite { get; set; }
}

/// <summary>Parameters for chart display option requests</summary>
internal class LDrawChartDisplayOptionsParams
{
	/// <summary>The scene to query or modify, or the first scene when omitted</summary>
	public string? SceneName { get; set; }
}

/// <summary>Parameters for chart display option mutation requests</summary>
internal sealed class LDrawSetChartDisplayOptionsParams : LDrawChartDisplayOptionsParams
{
	/// <summary>Show or hide chart axes</summary>
	public bool? ShowAxes { get; set; }

	/// <summary>Show or hide X and Y grid lines</summary>
	public bool? ShowGridLines { get; set; }

	/// <summary>Show or hide X and Y tick marks</summary>
	public bool? DrawTickMarks { get; set; }

	/// <summary>Show or hide X and Y tick labels</summary>
	public bool? DrawTickLabels { get; set; }

	/// <summary>Preferred X and Y tick spacing in pixels</summary>
	public double? PixelsPerTick { get; set; }

	/// <summary>Axis line colour applied to both axes</summary>
	public string? AxisColour { get; set; }

	/// <summary>Tick text colour applied to both axes</summary>
	public string? TickColour { get; set; }
}

/// <summary>Parameters for chart creation requests</summary>
internal sealed class LDrawChartCreateParams
{
	/// <summary>Caller-visible chart id, or null to use 'default'</summary>
	public string? ChartId { get; set; }

	/// <summary>Display name for the chart and generated overlay source</summary>
	public string? Name { get; set; }

	/// <summary>Chart colour, using any Colour32 parseable format</summary>
	public string? Colour { get; set; }

	/// <summary>Inline chart data rows</summary>
	public List<List<double>> DataRows { get; set; } = [];

	/// <summary>Series definitions</summary>
	public List<LDrawChartSeriesInput> Series { get; set; } = [];

	/// <summary>Scene names to show the chart in</summary>
	public List<string> SceneNames { get; set; } = [];

	/// <summary>True to frame the target scene after loading the chart overlay</summary>
	public bool ResetView { get; set; }
}

/// <summary>Parameters for chart data replacement requests</summary>
internal sealed class LDrawChartUpdateDataParams
{
	/// <summary>The chart id returned by ldraw_chart_create</summary>
	public string? ChartId { get; set; }

	/// <summary>Replacement inline chart data rows</summary>
	public List<List<double>> DataRows { get; set; } = [];

	/// <summary>True to frame the target scene after reloading the chart overlay</summary>
	public bool ResetView { get; set; }
}

/// <summary>Parameters for chart add-series requests</summary>
internal sealed class LDrawChartAddSeriesParams
{
	/// <summary>The chart id returned by ldraw_chart_create</summary>
	public string? ChartId { get; set; }

	/// <summary>The series to append</summary>
	public LDrawChartSeriesInput Series { get; set; } = new();

	/// <summary>True to frame the target scene after reloading the chart overlay</summary>
	public bool ResetView { get; set; }
}

/// <summary>Parameters for chart axis range requests</summary>
internal sealed class LDrawChartSetAxisRangesParams : LDrawSetAxisRangesParams
{
	/// <summary>The chart id returned by ldraw_chart_create</summary>
	public string? ChartId { get; set; }
}

/// <summary>Parameters for chart clear requests</summary>
internal sealed class LDrawChartClearParams
{
	/// <summary>The chart id to clear, or null to clear all MCP charts</summary>
	public string? ChartId { get; set; }
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
	public const string CreateScene = "create_scene";
	public const string CloseScene = "close_scene";
	public const string SwitchScene = "switch_scene";
	public const string ClearScene = "clear_scene";
	public const string RenameScene = "rename_scene";
	public const string ListSources = "list_sources";
	public const string LoadSource = "load_source";
	public const string ReloadSource = "reload_source";
	public const string RemoveSource = "remove_source";
	public const string SetSourceScenes = "set_source_scenes";
	public const string SetSceneSources = "set_scene_sources";
	public const string GetViewSettings = "get_view_settings";
	public const string GetAnimation = "get_animation";
	public const string ControlAnimation = "control_animation";
	public const string ListViewPresets = "list_view_presets";
	public const string SetViewPreset = "set_view_preset";
	public const string ListSavedViews = "list_saved_views";
	public const string SaveView = "save_view";
	public const string ApplySavedView = "apply_saved_view";
	public const string RemoveSavedView = "remove_saved_view";
	public const string GetStreamingState = "get_streaming_state";
	public const string SetStreamingState = "set_streaming_state";
	public const string SetProjection = "set_projection";
	public const string SetBackgroundColour = "set_background_colour";
	public const string SetCameraAlignAxis = "set_camera_align_axis";
	public const string SetAxisRanges = "set_axis_ranges";
	public const string GetDiagnosticModes = "get_diagnostic_modes";
	public const string SetDiagnosticModes = "set_diagnostic_modes";
	public const string SetRenderSettings = "set_render_settings";
	public const string ListObjects = "list_objects";
	public const string GetCamera = "get_camera";
	public const string SetCamera = "set_camera";
	public const string FindObjects = "find_objects";
	public const string GetObject = "get_object";
	public const string SelectObjects = "select_objects";
	public const string SetObjectVisibility = "set_object_visibility";
	public const string SetObjectColour = "set_object_colour";
	public const string GetObjectTransform = "get_object_transform";
	public const string SetObjectTransform = "set_object_transform";
	public const string GetObjectRenderState = "get_object_render_state";
	public const string SetObjectRenderState = "set_object_render_state";
	public const string HitTest = "hit_test";
	public const string ShowNormals = "show_normals";
	public const string ShowObjectBounds = "show_object_bounds";
	public const string GetSelection = "get_selection";
	public const string FrameObject = "frame_object";
	public const string FrameSelection = "frame_selection";
	public const string FrameBounds = "frame_bounds";
	public const string CaptureScene = "capture_scene";
	public const string GetChartDisplayOptions = "get_chart_display_options";
	public const string SetChartDisplayOptions = "set_chart_display_options";
	public const string ChartCreate = "chart_create";
	public const string ChartUpdateData = "chart_update_data";
	public const string ChartAddSeries = "chart_add_series";
	public const string ChartSetAxisRanges = "chart_set_axis_ranges";
	public const string ChartClear = "chart_clear";
	public const string OverlaySetScript = "overlay_set_script";
	public const string OverlayAppendScript = "overlay_append_script";
	public const string OverlayClear = "overlay_clear";
}
