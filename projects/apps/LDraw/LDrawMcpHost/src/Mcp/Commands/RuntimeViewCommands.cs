using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Threading.Tasks;
using ModelContextProtocol.Server;

namespace LDraw.MCP;

/// <summary>Runtime view command broker routing</summary>
internal sealed partial class McpBroker
{
	/// <summary>Return animation state from 'instance_id'</summary>
	public async Task<LDrawAnimationInfo> GetAnimationAsync(string? instance_id, LDrawAnimationParams parameters)
	{
		var registration = await ResolveInstanceAsync(instance_id).ConfigureAwait(false);
		return await m_client.GetAnimationAsync(registration, parameters).ConfigureAwait(false);
	}

	/// <summary>Control animation state in 'instance_id'</summary>
	public async Task<LDrawAnimationResult> ControlAnimationAsync(string? instance_id, LDrawAnimationControlParams parameters)
	{
		var registration = await ResolveInstanceAsync(instance_id).ConfigureAwait(false);
		return await m_client.ControlAnimationAsync(registration, parameters).ConfigureAwait(false);
	}

	/// <summary>List view presets accepted by 'instance_id'</summary>
	public async Task<LDrawViewPresetList> ListViewPresetsAsync(string? instance_id, LDrawViewPresetParams parameters)
	{
		var registration = await ResolveInstanceAsync(instance_id).ConfigureAwait(false);
		return await m_client.ListViewPresetsAsync(registration, parameters).ConfigureAwait(false);
	}

	/// <summary>Set a view preset in 'instance_id'</summary>
	public async Task<LDrawViewMutationResult> SetViewPresetAsync(string? instance_id, LDrawViewPresetParams parameters)
	{
		var registration = await ResolveInstanceAsync(instance_id).ConfigureAwait(false);
		return await m_client.SetViewPresetAsync(registration, parameters).ConfigureAwait(false);
	}

	/// <summary>List saved views in 'instance_id'</summary>
	public async Task<LDrawSavedViewList> ListSavedViewsAsync(string? instance_id, LDrawSavedViewParams parameters)
	{
		var registration = await ResolveInstanceAsync(instance_id).ConfigureAwait(false);
		return await m_client.ListSavedViewsAsync(registration, parameters).ConfigureAwait(false);
	}

	/// <summary>Save the current view in 'instance_id'</summary>
	public async Task<LDrawSavedViewResult> SaveViewAsync(string? instance_id, LDrawSavedViewParams parameters)
	{
		var registration = await ResolveInstanceAsync(instance_id).ConfigureAwait(false);
		return await m_client.SaveViewAsync(registration, parameters).ConfigureAwait(false);
	}

	/// <summary>Apply a saved view in 'instance_id'</summary>
	public async Task<LDrawSavedViewResult> ApplySavedViewAsync(string? instance_id, LDrawSavedViewParams parameters)
	{
		var registration = await ResolveInstanceAsync(instance_id).ConfigureAwait(false);
		return await m_client.ApplySavedViewAsync(registration, parameters).ConfigureAwait(false);
	}

	/// <summary>Remove a saved view in 'instance_id'</summary>
	public async Task<LDrawSavedViewResult> RemoveSavedViewAsync(string? instance_id, LDrawSavedViewParams parameters)
	{
		var registration = await ResolveInstanceAsync(instance_id).ConfigureAwait(false);
		return await m_client.RemoveSavedViewAsync(registration, parameters).ConfigureAwait(false);
	}

	/// <summary>Return streaming state from 'instance_id'</summary>
	public async Task<LDrawStreamingInfo> GetStreamingStateAsync(string? instance_id)
	{
		var registration = await ResolveInstanceAsync(instance_id).ConfigureAwait(false);
		return await m_client.GetStreamingStateAsync(registration).ConfigureAwait(false);
	}

	/// <summary>Enable or disable streaming in 'instance_id'</summary>
	public async Task<LDrawStreamingInfo> SetStreamingStateAsync(string? instance_id, LDrawStreamingControlParams parameters)
	{
		var registration = await ResolveInstanceAsync(instance_id).ConfigureAwait(false);
		return await m_client.SetStreamingStateAsync(registration, parameters).ConfigureAwait(false);
	}
}

/// <summary>Runtime view command pipe client calls</summary>
internal sealed partial class InstancePipeClient
{
	/// <summary>Return animation state from 'registration'</summary>
	public async Task<LDrawAnimationInfo> GetAnimationAsync(InstanceRegistration registration, LDrawAnimationParams parameters)
	{
		return await SendAsync<LDrawAnimationInfo>(registration, InstancePipeCommands.GetAnimation, parameters, ReadTimeout).ConfigureAwait(false);
	}

	/// <summary>Control animation state in 'registration'</summary>
	public async Task<LDrawAnimationResult> ControlAnimationAsync(InstanceRegistration registration, LDrawAnimationControlParams parameters)
	{
		return await SendAsync<LDrawAnimationResult>(registration, InstancePipeCommands.ControlAnimation, parameters, WriteTimeout).ConfigureAwait(false);
	}

	/// <summary>List view presets accepted by 'registration'</summary>
	public async Task<LDrawViewPresetList> ListViewPresetsAsync(InstanceRegistration registration, LDrawViewPresetParams parameters)
	{
		return await SendAsync<LDrawViewPresetList>(registration, InstancePipeCommands.ListViewPresets, parameters, ReadTimeout).ConfigureAwait(false);
	}

	/// <summary>Set a view preset in 'registration'</summary>
	public async Task<LDrawViewMutationResult> SetViewPresetAsync(InstanceRegistration registration, LDrawViewPresetParams parameters)
	{
		return await SendAsync<LDrawViewMutationResult>(registration, InstancePipeCommands.SetViewPreset, parameters, WriteTimeout).ConfigureAwait(false);
	}

	/// <summary>List saved views in 'registration'</summary>
	public async Task<LDrawSavedViewList> ListSavedViewsAsync(InstanceRegistration registration, LDrawSavedViewParams parameters)
	{
		return await SendAsync<LDrawSavedViewList>(registration, InstancePipeCommands.ListSavedViews, parameters, ReadTimeout).ConfigureAwait(false);
	}

	/// <summary>Save the current view in 'registration'</summary>
	public async Task<LDrawSavedViewResult> SaveViewAsync(InstanceRegistration registration, LDrawSavedViewParams parameters)
	{
		return await SendAsync<LDrawSavedViewResult>(registration, InstancePipeCommands.SaveView, parameters, WriteTimeout).ConfigureAwait(false);
	}

	/// <summary>Apply a saved view in 'registration'</summary>
	public async Task<LDrawSavedViewResult> ApplySavedViewAsync(InstanceRegistration registration, LDrawSavedViewParams parameters)
	{
		return await SendAsync<LDrawSavedViewResult>(registration, InstancePipeCommands.ApplySavedView, parameters, WriteTimeout).ConfigureAwait(false);
	}

	/// <summary>Remove a saved view in 'registration'</summary>
	public async Task<LDrawSavedViewResult> RemoveSavedViewAsync(InstanceRegistration registration, LDrawSavedViewParams parameters)
	{
		return await SendAsync<LDrawSavedViewResult>(registration, InstancePipeCommands.RemoveSavedView, parameters, WriteTimeout).ConfigureAwait(false);
	}

	/// <summary>Return streaming state from 'registration'</summary>
	public async Task<LDrawStreamingInfo> GetStreamingStateAsync(InstanceRegistration registration)
	{
		return await SendAsync<LDrawStreamingInfo>(registration, InstancePipeCommands.GetStreamingState, null, ReadTimeout).ConfigureAwait(false);
	}

	/// <summary>Enable or disable streaming in 'registration'</summary>
	public async Task<LDrawStreamingInfo> SetStreamingStateAsync(InstanceRegistration registration, LDrawStreamingControlParams parameters)
	{
		return await SendAsync<LDrawStreamingInfo>(registration, InstancePipeCommands.SetStreamingState, parameters, WriteTimeout).ConfigureAwait(false);
	}
}

/// <summary>Runtime view command MCP tool surface</summary>
internal sealed partial class LDrawTools
{
	/// <summary>Return scene animation state</summary>
	[McpServerTool(Name = "ldraw_get_animation", Title = "Get LDraw animation state", ReadOnly = true, Destructive = false, Idempotent = true, OpenWorld = false, UseStructuredContent = true)]
	[Description("Returns animation time and running state for a scene. Omit scene_name to use the first scene.")]
	public Task<LDrawAnimationInfo> GetAnimation(
		[Description("The id of a running LDraw instance from ldraw_list_instances. Omit to target the default instance (most-recently-used, or auto-launched when none are running).")] string? instance_id = null,
		[Description("The scene name to query. Omit to use the first scene.")] string? scene_name = null)
	{
		var parameters = new LDrawAnimationParams
		{
			SceneName = scene_name,
		};
		return m_broker.GetAnimationAsync(instance_id, parameters);
	}

	/// <summary>Control scene animation</summary>
	[McpServerTool(Name = "ldraw_control_animation", Title = "Control LDraw animation", ReadOnly = false, Destructive = false, Idempotent = false, OpenWorld = false, UseStructuredContent = true)]
	[Description("Controls scene animation. Commands: reset, play, stop, or step. time_s is used as the play step, reset value, or step delta depending on the native command.")]
	public Task<LDrawAnimationResult> ControlAnimation(
		[Description("Animation command: reset, play, stop, or step.")] string command,
		[Description("Optional animation time value in seconds. For play, 0 or omitted uses real time.")] double? time_s = null,
		[Description("The id of a running LDraw instance from ldraw_list_instances. Omit to target the default instance (most-recently-used, or auto-launched when none are running).")] string? instance_id = null,
		[Description("The scene name to modify. Omit to use the first scene.")] string? scene_name = null)
	{
		var parameters = new LDrawAnimationControlParams
		{
			SceneName = scene_name,
			Command = command,
			TimeS = time_s,
		};
		return m_broker.ControlAnimationAsync(instance_id, parameters);
	}

	/// <summary>List scene view presets</summary>
	[McpServerTool(Name = "ldraw_list_view_presets", Title = "List LDraw view presets", ReadOnly = true, Destructive = false, Idempotent = true, OpenWorld = false, UseStructuredContent = true)]
	[Description("Lists named view presets accepted by ldraw_set_view_preset for a scene. Omit scene_name to use the first scene.")]
	public Task<LDrawViewPresetList> ListViewPresets(
		[Description("The id of a running LDraw instance from ldraw_list_instances. Omit to target the default instance (most-recently-used, or auto-launched when none are running).")] string? instance_id = null,
		[Description("The scene name to query. Omit to use the first scene.")] string? scene_name = null)
	{
		var parameters = new LDrawViewPresetParams
		{
			SceneName = scene_name,
		};
		return m_broker.ListViewPresetsAsync(instance_id, parameters);
	}

	/// <summary>Apply a named scene view preset</summary>
	[McpServerTool(Name = "ldraw_set_view_preset", Title = "Set LDraw view preset", ReadOnly = false, Destructive = false, Idempotent = true, OpenWorld = false, UseStructuredContent = true)]
	[Description("Applies a named view preset to a scene. Valid values: PosX, NegX, PosY, NegY, PosZ, NegZ, PosXYZ, or NegXYZ.")]
	public Task<LDrawViewMutationResult> SetViewPreset(
		[Description("Named view preset to apply: PosX, NegX, PosY, NegY, PosZ, NegZ, PosXYZ, or NegXYZ.")] string view_preset,
		[Description("The id of a running LDraw instance from ldraw_list_instances. Omit to target the default instance (most-recently-used, or auto-launched when none are running).")] string? instance_id = null,
		[Description("The scene name to modify. Omit to use the first scene.")] string? scene_name = null)
	{
		var parameters = new LDrawViewPresetParams
		{
			SceneName = scene_name,
			ViewPreset = view_preset,
		};
		return m_broker.SetViewPresetAsync(instance_id, parameters);
	}

	/// <summary>List saved camera views for a scene</summary>
	[McpServerTool(Name = "ldraw_list_saved_views", Title = "List LDraw saved views", ReadOnly = true, Destructive = false, Idempotent = true, OpenWorld = false, UseStructuredContent = true)]
	[Description("Lists saved camera views for a scene. Omit scene_name to use the first scene.")]
	public Task<LDrawSavedViewList> ListSavedViews(
		[Description("The id of a running LDraw instance from ldraw_list_instances. Omit to target the default instance (most-recently-used, or auto-launched when none are running).")] string? instance_id = null,
		[Description("The scene name to query. Omit to use the first scene.")] string? scene_name = null)
	{
		var parameters = new LDrawSavedViewParams
		{
			SceneName = scene_name,
		};
		return m_broker.ListSavedViewsAsync(instance_id, parameters);
	}

	/// <summary>Save the current camera view for a scene</summary>
	[McpServerTool(Name = "ldraw_save_view", Title = "Save LDraw view", ReadOnly = false, Destructive = false, Idempotent = false, OpenWorld = false, UseStructuredContent = true)]
	[Description("Saves the current camera view in a scene without opening the UI prompt. Omit scene_name to use the first scene.")]
	public Task<LDrawSavedViewResult> SaveView(
		[Description("Saved view name.")] string name,
		[Description("True to replace an existing saved view with the same name.")] bool replace = false,
		[Description("The id of a running LDraw instance from ldraw_list_instances. Omit to target the default instance (most-recently-used, or auto-launched when none are running).")] string? instance_id = null,
		[Description("The scene name to modify. Omit to use the first scene.")] string? scene_name = null)
	{
		var parameters = new LDrawSavedViewParams
		{
			SceneName = scene_name,
			Name = name,
			Replace = replace,
		};
		return m_broker.SaveViewAsync(instance_id, parameters);
	}

	/// <summary>Apply a saved camera view to a scene</summary>
	[McpServerTool(Name = "ldraw_apply_saved_view", Title = "Apply LDraw saved view", ReadOnly = false, Destructive = false, Idempotent = true, OpenWorld = false, UseStructuredContent = true)]
	[Description("Applies a saved camera view by name. Omit scene_name to use the first scene.")]
	public Task<LDrawSavedViewResult> ApplySavedView(
		[Description("Saved view name.")] string name,
		[Description("The id of a running LDraw instance from ldraw_list_instances. Omit to target the default instance (most-recently-used, or auto-launched when none are running).")] string? instance_id = null,
		[Description("The scene name to modify. Omit to use the first scene.")] string? scene_name = null)
	{
		var parameters = new LDrawSavedViewParams
		{
			SceneName = scene_name,
			Name = name,
		};
		return m_broker.ApplySavedViewAsync(instance_id, parameters);
	}

	/// <summary>Remove a saved camera view from a scene</summary>
	[McpServerTool(Name = "ldraw_remove_saved_view", Title = "Remove LDraw saved view", ReadOnly = false, Destructive = true, Idempotent = false, OpenWorld = false, UseStructuredContent = true)]
	[Description("Removes a saved camera view by name. Omit scene_name to use the first scene.")]
	public Task<LDrawSavedViewResult> RemoveSavedView(
		[Description("Saved view name.")] string name,
		[Description("The id of a running LDraw instance from ldraw_list_instances. Omit to target the default instance (most-recently-used, or auto-launched when none are running).")] string? instance_id = null,
		[Description("The scene name to modify. Omit to use the first scene.")] string? scene_name = null)
	{
		var parameters = new LDrawSavedViewParams
		{
			SceneName = scene_name,
			Name = name,
		};
		return m_broker.RemoveSavedViewAsync(instance_id, parameters);
	}

	/// <summary>Return View3D streaming state</summary>
	[McpServerTool(Name = "ldraw_get_streaming_state", Title = "Get LDraw streaming state", ReadOnly = true, Destructive = false, Idempotent = true, OpenWorld = false, UseStructuredContent = true)]
	[Description("Returns the View3D streaming state and the runtime port if streaming is active.")]
	public Task<LDrawStreamingInfo> GetStreamingState(
		[Description("The id of a running LDraw instance from ldraw_list_instances. Omit to target the default instance (most-recently-used, or auto-launched when none are running).")] string? instance_id = null)
	{
		return m_broker.GetStreamingStateAsync(instance_id);
	}

	/// <summary>Enable or disable View3D streaming</summary>
	[McpServerTool(Name = "ldraw_set_streaming_state", Title = "Set LDraw streaming state", ReadOnly = false, Destructive = false, Idempotent = true, OpenWorld = true, UseStructuredContent = true)]
	[Description("Enables or disables View3D streaming at runtime. The optional port does not update persistent profile settings.")]
	public Task<LDrawStreamingInfo> SetStreamingState(
		[Description("True to enable streaming, false to disable it.")] bool enable,
		[Description("Optional streaming port. Omit to use the current profile default.")] int? port = null,
		[Description("The id of a running LDraw instance from ldraw_list_instances. Omit to target the default instance (most-recently-used, or auto-launched when none are running).")] string? instance_id = null)
	{
		var parameters = new LDrawStreamingControlParams
		{
			Enable = enable,
			Port = port,
		};
		return m_broker.SetStreamingStateAsync(instance_id, parameters);
	}
}
