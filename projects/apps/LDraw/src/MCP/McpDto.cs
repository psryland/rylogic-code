using System;
using System.Collections.Generic;
using System.Text.Json.Serialization;

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

	/// <summary>The number of objects visible in this scene</summary>
	public int ObjectCount { get; set; }

	/// <summary>The source context ids represented in this scene</summary>
	public List<string> ContextIds { get; set; } = [];
}

/// <summary>Local named-pipe request between the broker and an LDraw instance</summary>
internal sealed class InstancePipeRequest
{
	/// <summary>Version of the pipe request schema</summary>
	public int SchemaVersion { get; set; } = 1;

	/// <summary>The command to execute</summary>
	public string Command { get; set; } = string.Empty;
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

	/// <summary>The scene summary returned for get-scene-summary requests</summary>
	public LDrawSceneSummary? SceneSummary { get; set; }
}

/// <summary>Command names supported by the local instance pipe</summary>
internal static class InstancePipeCommands
{
	public const string GetSceneSummary = "get_scene_summary";
}
