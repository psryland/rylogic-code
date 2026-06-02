using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Threading.Tasks;
using ModelContextProtocol.Server;

namespace LDraw.MCP;

/// <summary>Source command broker routing</summary>
internal sealed partial class McpBroker
{
	/// <summary>List sources in 'instance_id'</summary>
	public async Task<LDrawSourceList> ListSourcesAsync(string? instance_id, LDrawListSourcesParams parameters)
	{
		var registration = ResolveInstance(instance_id);
		return await m_client.ListSourcesAsync(registration, parameters).ConfigureAwait(false);
	}

	/// <summary>Load a file source in 'instance_id'</summary>
	public async Task<LDrawSourceMutationResult> LoadSourceAsync(string? instance_id, LDrawLoadSourceParams parameters)
	{
		var registration = ResolveInstance(instance_id);
		return await m_client.LoadSourceAsync(registration, parameters).ConfigureAwait(false);
	}

	/// <summary>Reload sources in 'instance_id'</summary>
	public async Task<LDrawSourceMutationResult> ReloadSourceAsync(string? instance_id, LDrawSourceParams parameters)
	{
		var registration = ResolveInstance(instance_id);
		return await m_client.ReloadSourceAsync(registration, parameters).ConfigureAwait(false);
	}

	/// <summary>Remove sources from 'instance_id'</summary>
	public async Task<LDrawSourceMutationResult> RemoveSourceAsync(string? instance_id, LDrawSourceParams parameters)
	{
		var registration = ResolveInstance(instance_id);
		return await m_client.RemoveSourceAsync(registration, parameters).ConfigureAwait(false);
	}

	/// <summary>Set user-source scene membership in 'instance_id'</summary>
	public async Task<LDrawSourceMutationResult> SetSourceScenesAsync(string? instance_id, LDrawSetSourceScenesParams parameters)
	{
		var registration = ResolveInstance(instance_id);
		return await m_client.SetSourceScenesAsync(registration, parameters).ConfigureAwait(false);
	}
}

/// <summary>Source command pipe client calls</summary>
internal sealed partial class InstancePipeClient
{
	/// <summary>List sources in 'registration'</summary>
	public async Task<LDrawSourceList> ListSourcesAsync(InstanceRegistration registration, LDrawListSourcesParams parameters)
	{
		return await SendAsync<LDrawSourceList>(registration, InstancePipeCommands.ListSources, parameters, ReadTimeout).ConfigureAwait(false);
	}

	/// <summary>Load a file source in 'registration'</summary>
	public async Task<LDrawSourceMutationResult> LoadSourceAsync(InstanceRegistration registration, LDrawLoadSourceParams parameters)
	{
		return await SendAsync<LDrawSourceMutationResult>(registration, InstancePipeCommands.LoadSource, parameters, WriteTimeout).ConfigureAwait(false);
	}

	/// <summary>Reload sources in 'registration'</summary>
	public async Task<LDrawSourceMutationResult> ReloadSourceAsync(InstanceRegistration registration, LDrawSourceParams parameters)
	{
		return await SendAsync<LDrawSourceMutationResult>(registration, InstancePipeCommands.ReloadSource, parameters, WriteTimeout).ConfigureAwait(false);
	}

	/// <summary>Remove sources from 'registration'</summary>
	public async Task<LDrawSourceMutationResult> RemoveSourceAsync(InstanceRegistration registration, LDrawSourceParams parameters)
	{
		return await SendAsync<LDrawSourceMutationResult>(registration, InstancePipeCommands.RemoveSource, parameters, WriteTimeout).ConfigureAwait(false);
	}

	/// <summary>Set user-source scene membership in 'registration'</summary>
	public async Task<LDrawSourceMutationResult> SetSourceScenesAsync(InstanceRegistration registration, LDrawSetSourceScenesParams parameters)
	{
		return await SendAsync<LDrawSourceMutationResult>(registration, InstancePipeCommands.SetSourceScenes, parameters, WriteTimeout).ConfigureAwait(false);
	}
}

/// <summary>Source command MCP tool surface</summary>
internal sealed partial class LDrawTools
{
	/// <summary>List user-loaded LDraw sources</summary>
	[McpServerTool(Name = "ldraw_list_sources", Title = "List LDraw sources", ReadOnly = true, Destructive = false, Idempotent = true, OpenWorld = false, UseStructuredContent = true)]
	[Description("Lists user-loaded LDraw sources, optionally filtered to sources selected for one scene.")]
	public Task<LDrawSourceList> ListSources(
		[Description("The scene name to filter by. Omit to list all user-loaded sources.")] string? scene_name = null,
		[Description("The instance id returned by ldraw_list_instances. Omit to target the broker instance.")] string? instance_id = null)
	{
		var parameters = new LDrawListSourcesParams
		{
			SceneName = scene_name,
		};
		return m_broker.ListSourcesAsync(instance_id, parameters);
	}

	/// <summary>Load a file-backed LDraw source</summary>
	[McpServerTool(Name = "ldraw_load_source", Title = "Load LDraw source", ReadOnly = false, Destructive = false, Idempotent = false, OpenWorld = true, UseStructuredContent = true)]
	[Description("Synchronously loads a file-backed LDraw source and shows it in requested scenes. Scene names default to the first scene.")]
	public Task<LDrawSourceMutationResult> LoadSource(
		[Description("Path to the .ldr/.bdr source file to load.")] string file_path,
		[Description("Scene names to show the source in. Omit to use the first scene.")] string[]? scene_names = null,
		[Description("True to show the source in every current scene.")] bool all_scenes = false,
		[Description("The instance id returned by ldraw_list_instances. Omit to target the broker instance.")] string? instance_id = null)
	{
		var parameters = new LDrawLoadSourceParams
		{
			FilePath = file_path,
			SceneNames = scene_names?.ToList() ?? [],
			AllScenes = all_scenes,
		};
		return m_broker.LoadSourceAsync(instance_id, parameters);
	}

	/// <summary>Reload user-loaded LDraw sources</summary>
	[McpServerTool(Name = "ldraw_reload_source", Title = "Reload LDraw source", ReadOnly = false, Destructive = false, Idempotent = false, OpenWorld = true, UseStructuredContent = true)]
	[Description("Reloads one or more existing user-loaded sources from their backing files.")]
	public Task<LDrawSourceMutationResult> ReloadSource(
		[Description("Source context ids, display names, or file paths to reload. Required unless all_sources is true.")] string[]? source_ids = null,
		[Description("True to reload every current user-loaded source.")] bool all_sources = false,
		[Description("The instance id returned by ldraw_list_instances. Omit to target the broker instance.")] string? instance_id = null)
	{
		var parameters = new LDrawSourceParams
		{
			SourceIds = source_ids?.ToList() ?? [],
			AllSources = all_sources,
		};
		return m_broker.ReloadSourceAsync(instance_id, parameters);
	}

	/// <summary>Remove user-loaded LDraw sources</summary>
	[McpServerTool(Name = "ldraw_remove_source", Title = "Remove LDraw source", ReadOnly = false, Destructive = true, Idempotent = false, OpenWorld = false, UseStructuredContent = true)]
	[Description("Removes one or more existing user-loaded sources from the instance.")]
	public Task<LDrawSourceMutationResult> RemoveSource(
		[Description("Source context ids, display names, or file paths to remove. Required unless all_sources is true.")] string[]? source_ids = null,
		[Description("True to remove every current user-loaded source.")] bool all_sources = false,
		[Description("The instance id returned by ldraw_list_instances. Omit to target the broker instance.")] string? instance_id = null)
	{
		var parameters = new LDrawSourceParams
		{
			SourceIds = source_ids?.ToList() ?? [],
			AllSources = all_sources,
		};
		return m_broker.RemoveSourceAsync(instance_id, parameters);
	}

	/// <summary>Set user-loaded source scene membership</summary>
	[McpServerTool(Name = "ldraw_set_source_scenes", Title = "Set LDraw source scenes", ReadOnly = false, Destructive = false, Idempotent = true, OpenWorld = false, UseStructuredContent = true)]
	[Description("Changes which user-loaded sources are visible in scene views. MCP-owned overlay sources are never added or removed.")]
	public Task<LDrawSourceMutationResult> SetSourceScenes(
		[Description("Membership operation: replace, add, remove, or clear. Replace means exactly the requested user sources are visible in the target scenes.")] string mode = "replace",
		[Description("Source context ids, display names, or file paths to use for replace/add/remove operations. Required unless all_sources is true or mode is clear.")] string[]? source_ids = null,
		[Description("Scene names to modify. Omit to use the first scene.")] string[]? scene_names = null,
		[Description("True to target every current user-loaded source.")] bool all_sources = false,
		[Description("True to target every current scene.")] bool all_scenes = false,
		[Description("True to frame each scene after adding source objects.")] bool reset_view = false,
		[Description("The instance id returned by ldraw_list_instances. Omit to target the broker instance.")] string? instance_id = null)
	{
		var parameters = new LDrawSetSourceScenesParams
		{
			Mode = mode,
			SourceIds = source_ids?.ToList() ?? [],
			SceneNames = scene_names?.ToList() ?? [],
			AllSources = all_sources,
			AllScenes = all_scenes,
			ResetView = reset_view,
		};
		return m_broker.SetSourceScenesAsync(instance_id, parameters);
	}
}
