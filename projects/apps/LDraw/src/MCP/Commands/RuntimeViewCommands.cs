using System;
using System.ComponentModel;
using System.Linq;
using System.Threading.Tasks;
using ModelContextProtocol.Server;
using Rylogic.Gfx;

namespace LDraw.MCP;

/// <summary>Runtime view command broker routing</summary>
internal sealed partial class McpBroker
{
	/// <summary>Return animation state from 'instance_id'</summary>
	public async Task<LDrawAnimationInfo> GetAnimationAsync(string? instance_id, LDrawAnimationParams parameters)
	{
		var registration = ResolveInstance(instance_id);
		return await m_client.GetAnimationAsync(registration, parameters).ConfigureAwait(false);
	}

	/// <summary>Control animation state in 'instance_id'</summary>
	public async Task<LDrawAnimationResult> ControlAnimationAsync(string? instance_id, LDrawAnimationControlParams parameters)
	{
		var registration = ResolveInstance(instance_id);
		return await m_client.ControlAnimationAsync(registration, parameters).ConfigureAwait(false);
	}

	/// <summary>List view presets accepted by 'instance_id'</summary>
	public async Task<LDrawViewPresetList> ListViewPresetsAsync(string? instance_id, LDrawViewPresetParams parameters)
	{
		var registration = ResolveInstance(instance_id);
		return await m_client.ListViewPresetsAsync(registration, parameters).ConfigureAwait(false);
	}

	/// <summary>Set a view preset in 'instance_id'</summary>
	public async Task<LDrawViewMutationResult> SetViewPresetAsync(string? instance_id, LDrawViewPresetParams parameters)
	{
		var registration = ResolveInstance(instance_id);
		return await m_client.SetViewPresetAsync(registration, parameters).ConfigureAwait(false);
	}

	/// <summary>List saved views in 'instance_id'</summary>
	public async Task<LDrawSavedViewList> ListSavedViewsAsync(string? instance_id, LDrawSavedViewParams parameters)
	{
		var registration = ResolveInstance(instance_id);
		return await m_client.ListSavedViewsAsync(registration, parameters).ConfigureAwait(false);
	}

	/// <summary>Save the current view in 'instance_id'</summary>
	public async Task<LDrawSavedViewResult> SaveViewAsync(string? instance_id, LDrawSavedViewParams parameters)
	{
		var registration = ResolveInstance(instance_id);
		return await m_client.SaveViewAsync(registration, parameters).ConfigureAwait(false);
	}

	/// <summary>Apply a saved view in 'instance_id'</summary>
	public async Task<LDrawSavedViewResult> ApplySavedViewAsync(string? instance_id, LDrawSavedViewParams parameters)
	{
		var registration = ResolveInstance(instance_id);
		return await m_client.ApplySavedViewAsync(registration, parameters).ConfigureAwait(false);
	}

	/// <summary>Remove a saved view in 'instance_id'</summary>
	public async Task<LDrawSavedViewResult> RemoveSavedViewAsync(string? instance_id, LDrawSavedViewParams parameters)
	{
		var registration = ResolveInstance(instance_id);
		return await m_client.RemoveSavedViewAsync(registration, parameters).ConfigureAwait(false);
	}

	/// <summary>Return streaming state from 'instance_id'</summary>
	public async Task<LDrawStreamingInfo> GetStreamingStateAsync(string? instance_id)
	{
		var registration = ResolveInstance(instance_id);
		return await m_client.GetStreamingStateAsync(registration).ConfigureAwait(false);
	}

	/// <summary>Enable or disable streaming in 'instance_id'</summary>
	public async Task<LDrawStreamingInfo> SetStreamingStateAsync(string? instance_id, LDrawStreamingControlParams parameters)
	{
		var registration = ResolveInstance(instance_id);
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

/// <summary>Runtime view command instance implementation</summary>
internal sealed partial class LDrawInstanceHost
{
	private int? m_streaming_port;

	/// <summary>Return animation state for the requested scene</summary>
	private Task<LDrawAnimationInfo> GetAnimationAsync(LDrawAnimationParams parameters)
	{
		return m_model.InvokeAsync(() =>
		{
			var scene = ResolveScene(parameters.SceneName);
			return CreateAnimationInfo(scene);
		}, TimeSpan.FromSeconds(5));
	}

	/// <summary>Control animation in the requested scene</summary>
	private Task<LDrawAnimationResult> ControlAnimationAsync(LDrawAnimationControlParams parameters)
	{
		return m_model.InvokeAsync(() =>
		{
			var scene = ResolveScene(parameters.SceneName);
			var command = ParseAnimationCommand(parameters.Command);
			scene.SceneView.Scene.Window.AnimControl(command, parameters.TimeS ?? 0.0);
			scene.SceneView.Invalidate();
			return new LDrawAnimationResult
			{
				Action = $"animation_{command.ToString().ToLowerInvariant()}",
				Animation = CreateAnimationInfo(scene),
			};
		}, TimeSpan.FromSeconds(5));
	}

	/// <summary>List view presets that can be applied to the requested scene</summary>
	private Task<LDrawViewPresetList> ListViewPresetsAsync(LDrawViewPresetParams parameters)
	{
		return m_model.InvokeAsync(() =>
		{
			var scene = ResolveScene(parameters.SceneName);
			return CreateViewPresetList(scene);
		}, TimeSpan.FromSeconds(5));
	}

	/// <summary>Apply a named view preset to the requested scene</summary>
	private Task<LDrawViewMutationResult> SetViewPresetAsync(LDrawViewPresetParams parameters)
	{
		return m_model.InvokeAsync(() =>
		{
			var scene = ResolveScene(parameters.SceneName);
			var preset = ParseViewPreset(parameters.ViewPreset);
			scene.SceneState.ViewPreset = preset;
			scene.SceneView.Invalidate();
			return CreateViewMutationResult($"set_view_preset_{preset.ToString().ToLowerInvariant()}", scene);
		}, TimeSpan.FromSeconds(5));
	}

	/// <summary>List saved camera views for the requested scene</summary>
	private Task<LDrawSavedViewList> ListSavedViewsAsync(LDrawSavedViewParams parameters)
	{
		return m_model.InvokeAsync(() =>
		{
			var scene = ResolveScene(parameters.SceneName);
			return CreateSavedViewList(scene);
		}, TimeSpan.FromSeconds(5));
	}

	/// <summary>Save the requested scene's current camera as a named saved view</summary>
	private Task<LDrawSavedViewResult> SaveViewAsync(LDrawSavedViewParams parameters)
	{
		return m_model.InvokeAsync(() =>
		{
			var scene = ResolveScene(parameters.SceneName);
			var scene_view = scene.SceneView.Scene;
			var name = NormaliseSavedViewName(parameters.Name, "ldraw_save_view");
			var existing = FindSavedViews(scene, name);
			if (existing.Length != 0)
			{
				if (!parameters.Replace)
					throw new InvalidOperationException($"Saved view '{name}' already exists. Set replace=true to overwrite it.");
				if (existing.Length != 1)
					throw new InvalidOperationException($"Saved view '{name}' matches {existing.Length} views. Remove duplicates before replacing it.");

				scene_view.RemoveSavedViewItem(existing[0]);
			}

			var view = scene_view.SaveCurrentViewAs(name);
			return CreateSavedViewResult("save_view", scene, view);
		}, TimeSpan.FromSeconds(5));
	}

	/// <summary>Apply a named saved view to the requested scene</summary>
	private Task<LDrawSavedViewResult> ApplySavedViewAsync(LDrawSavedViewParams parameters)
	{
		return m_model.InvokeAsync(() =>
		{
			var scene = ResolveScene(parameters.SceneName);
			var view = ResolveSavedView(scene, parameters.Name, "ldraw_apply_saved_view");
			scene.SceneView.Scene.ApplySavedViewItem(view);
			return CreateSavedViewResult("apply_saved_view", scene, view);
		}, TimeSpan.FromSeconds(5));
	}

	/// <summary>Remove a named saved view from the requested scene</summary>
	private Task<LDrawSavedViewResult> RemoveSavedViewAsync(LDrawSavedViewParams parameters)
	{
		return m_model.InvokeAsync(() =>
		{
			var scene = ResolveScene(parameters.SceneName);
			var view = ResolveSavedView(scene, parameters.Name, "ldraw_remove_saved_view");
			var view_info = CreateSavedViewInfo(view);
			scene.SceneView.Scene.RemoveSavedViewItem(view);
			return CreateSavedViewResult("remove_saved_view", scene, view_info);
		}, TimeSpan.FromSeconds(5));
	}

	/// <summary>Return the current View3D streaming state</summary>
	private Task<LDrawStreamingInfo> GetStreamingStateAsync()
	{
		return m_model.InvokeAsync(() =>
		{
			return CreateStreamingInfo("get_streaming_state");
		}, TimeSpan.FromSeconds(5));
	}

	/// <summary>Enable or disable View3D streaming without changing persistent profile settings</summary>
	private Task<LDrawStreamingInfo> SetStreamingStateAsync(LDrawStreamingControlParams parameters)
	{
		return m_model.InvokeAsync(() =>
		{
			var port = ResolveStreamingPort(parameters.Port);
			m_model.View3d.Streaming(parameters.Enable, port);
			m_streaming_port = parameters.Enable ? port : null;
			return CreateStreamingInfo(parameters.Enable ? "enable_streaming" : "disable_streaming");
		}, TimeSpan.FromSeconds(5));
	}

	/// <summary>Create animation state data from the current scene window</summary>
	private static LDrawAnimationInfo CreateAnimationInfo(UI.SceneUI scene)
	{
		var window = scene.SceneView.Scene.Window;
		return new LDrawAnimationInfo
		{
			SceneName = scene.SceneName,
			TimeS = window.AnimTime,
			Animating = window.Animating,
			AvailableCommands = [..Enum.GetNames<View3d.EAnimCommand>().Select(x => x.ToLowerInvariant())],
		};
	}

	/// <summary>Create view-preset list data from the current scene state</summary>
	private static LDrawViewPresetList CreateViewPresetList(UI.SceneUI scene)
	{
		var current = scene.SceneView.Scene.ViewPreset;
		return new LDrawViewPresetList
		{
			SceneName = scene.SceneName,
			Current = current.ToString(),
			Presets =
			[
				..Enum.GetValues<EViewPreset>()
					.Where(IsApplicableViewPreset)
					.Select(preset => new LDrawViewPresetInfo
					{
						Name = preset.ToString(),
						IsCurrent = preset == current,
					})
			],
		};
	}

	/// <summary>Create saved-view list data from the current scene state</summary>
	private static LDrawSavedViewList CreateSavedViewList(UI.SceneUI scene)
	{
		var scene_view = scene.SceneView.Scene;
		return new LDrawSavedViewList
		{
			SceneName = scene.SceneName,
			CurrentName = scene_view.SavedViews.CurrentItem is Rylogic.Gui.WPF.SavedView view ? view.Name : string.Empty,
			Views = [..scene_view.SavedViewItems().Select(CreateSavedViewInfo)],
		};
	}

	/// <summary>Create saved-view mutation result data from the current scene state</summary>
	private static LDrawSavedViewResult CreateSavedViewResult(string action, UI.SceneUI scene, Rylogic.Gui.WPF.SavedView view)
	{
		return CreateSavedViewResult(action, scene, CreateSavedViewInfo(view));
	}

	/// <summary>Create saved-view mutation result data from the current scene state</summary>
	private static LDrawSavedViewResult CreateSavedViewResult(string action, UI.SceneUI scene, LDrawSavedViewInfo view)
	{
		return new LDrawSavedViewResult
		{
			SceneName = scene.SceneName,
			Action = action,
			View = view,
			Views = [..scene.SceneView.Scene.SavedViewItems().Select(CreateSavedViewInfo)],
			Camera = CreateCameraInfo(scene),
		};
	}

	/// <summary>Create saved-view DTO data</summary>
	private static LDrawSavedViewInfo CreateSavedViewInfo(Rylogic.Gui.WPF.SavedView view)
	{
		var fov = view.Fov;
		return new LDrawSavedViewInfo
		{
			Name = view.Name,
			FocusDistance = view.FocusDist,
			AlignAxis = LDrawVector3.From(view.AlignAxis),
			Fov = new LDrawVector3 { X = fov.x, Y = fov.y, Z = 0 },
			Orthographic = view.Orthographic,
			CameraToWorld = [..view.C2W.ToArray().Select(x => (double)x)],
		};
	}

	/// <summary>Create streaming DTO data from the current native state</summary>
	private LDrawStreamingInfo CreateStreamingInfo(string action)
	{
		var state = m_model.View3d.StreamingState;
		var active = StreamingStateIsActive(state);
		return new LDrawStreamingInfo
		{
			Action = action,
			State = state.ToString(),
			IsActive = active,
			Port = active ? m_streaming_port ?? m_model.Profile.StreamingPort : null,
			DefaultPort = m_model.Profile.StreamingPort,
		};
	}

	/// <summary>Parse an animation command name</summary>
	private static View3d.EAnimCommand ParseAnimationCommand(string? command)
	{
		if (string.IsNullOrWhiteSpace(command))
			throw new InvalidOperationException("ldraw_control_animation requires command.");
		if (!Enum.TryParse<View3d.EAnimCommand>(command.Trim(), ignoreCase: true, out var value))
			throw new InvalidOperationException("Animation command must be one of: reset, play, stop, or step.");

		return value;
	}

	/// <summary>Parse a named view preset accepted by the runtime command</summary>
	private static EViewPreset ParseViewPreset(string? view_preset)
	{
		if (string.IsNullOrWhiteSpace(view_preset))
			throw new InvalidOperationException("ldraw_set_view_preset requires view_preset.");
		if (!Enum.TryParse<EViewPreset>(view_preset.Trim(), ignoreCase: true, out var preset))
			throw new InvalidOperationException("View preset must be one of: PosX, NegX, PosY, NegY, PosZ, NegZ, PosXYZ, or NegXYZ.");

		switch (preset)
		{
			case EViewPreset.PosX:
			case EViewPreset.NegX:
			case EViewPreset.PosY:
			case EViewPreset.NegY:
			case EViewPreset.PosZ:
			case EViewPreset.NegZ:
			case EViewPreset.PosXYZ:
			case EViewPreset.NegXYZ:
			{
				return preset;
			}
			case EViewPreset.Current:
			{
				throw new InvalidOperationException("Current is a read-only view-preset sentinel. Use a direction preset such as PosX or PosXYZ.");
			}
			default:
			{
				throw new ArgumentOutOfRangeException(nameof(view_preset), view_preset, "Unknown view preset.");
			}
		}
	}

	/// <summary>Return true for view presets that can be applied explicitly</summary>
	private static bool IsApplicableViewPreset(EViewPreset preset)
	{
		switch (preset)
		{
			case EViewPreset.PosX:
			case EViewPreset.NegX:
			case EViewPreset.PosY:
			case EViewPreset.NegY:
			case EViewPreset.PosZ:
			case EViewPreset.NegZ:
			case EViewPreset.PosXYZ:
			case EViewPreset.NegXYZ:
			{
				return true;
			}
			case EViewPreset.Current:
			{
				return false;
			}
			default:
			{
				throw new ArgumentOutOfRangeException(nameof(preset), preset, "Unknown view preset.");
			}
		}
	}

	/// <summary>Resolve a unique saved view from the requested name</summary>
	private static Rylogic.Gui.WPF.SavedView ResolveSavedView(UI.SceneUI scene, string? name, string command_name)
	{
		var view_name = NormaliseSavedViewName(name, command_name);
		var matches = FindSavedViews(scene, view_name);
		if (matches.Length == 0)
			throw new InvalidOperationException($"{command_name} could not find saved view '{view_name}'.");
		if (matches.Length != 1)
			throw new InvalidOperationException($"{command_name} found {matches.Length} saved views named '{view_name}'. Remove duplicates before targeting it.");

		return matches[0];
	}

	/// <summary>Find saved views with a case-insensitive name match</summary>
	private static Rylogic.Gui.WPF.SavedView[] FindSavedViews(UI.SceneUI scene, string name)
	{
		return [..scene.SceneView.Scene.SavedViewItems().Where(view => string.Equals(view.Name, name, StringComparison.OrdinalIgnoreCase))];
	}

	/// <summary>Validate and normalize a saved view name</summary>
	private static string NormaliseSavedViewName(string? name, string command_name)
	{
		if (string.IsNullOrWhiteSpace(name))
			throw new InvalidOperationException($"{command_name} requires name.");

		return name.Trim();
	}

	/// <summary>Return true when the native streaming state is active</summary>
	private static bool StreamingStateIsActive(View3d.EStreamingState state)
	{
		switch (state)
		{
			case View3d.EStreamingState.Disconnected:
			{
				return false;
			}
			case View3d.EStreamingState.Listening:
			case View3d.EStreamingState.Connected:
			{
				return true;
			}
			default:
			{
				throw new ArgumentOutOfRangeException(nameof(state), state, "Unknown streaming state.");
			}
		}
	}

	/// <summary>Resolve a valid streaming port for a runtime streaming request</summary>
	private int ResolveStreamingPort(int? port)
	{
		var value = port ?? m_model.Profile.StreamingPort;
		if (value < 1 || value > 65535)
			throw new InvalidOperationException("Streaming port must be in the range 1..65535.");

		return value;
	}
}

/// <summary>Runtime view command MCP tool surface</summary>
internal sealed partial class LDrawTools
{
	/// <summary>Return scene animation state</summary>
	[McpServerTool(Name = "ldraw_get_animation", Title = "Get LDraw animation state", ReadOnly = true, Destructive = false, Idempotent = true, OpenWorld = false, UseStructuredContent = true)]
	[Description("Returns animation time and running state for a scene. Omit scene_name to use the first scene.")]
	public Task<LDrawAnimationInfo> GetAnimation(
		[Description("The instance id returned by ldraw_list_instances. Omit to target the broker instance.")] string? instance_id = null,
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
		[Description("The instance id returned by ldraw_list_instances. Omit to target the broker instance.")] string? instance_id = null,
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
		[Description("The instance id returned by ldraw_list_instances. Omit to target the broker instance.")] string? instance_id = null,
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
		[Description("The instance id returned by ldraw_list_instances. Omit to target the broker instance.")] string? instance_id = null,
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
		[Description("The instance id returned by ldraw_list_instances. Omit to target the broker instance.")] string? instance_id = null,
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
		[Description("The instance id returned by ldraw_list_instances. Omit to target the broker instance.")] string? instance_id = null,
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
		[Description("The instance id returned by ldraw_list_instances. Omit to target the broker instance.")] string? instance_id = null,
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
		[Description("The instance id returned by ldraw_list_instances. Omit to target the broker instance.")] string? instance_id = null,
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
		[Description("The instance id returned by ldraw_list_instances. Omit to target the broker instance.")] string? instance_id = null)
	{
		return m_broker.GetStreamingStateAsync(instance_id);
	}

	/// <summary>Enable or disable View3D streaming</summary>
	[McpServerTool(Name = "ldraw_set_streaming_state", Title = "Set LDraw streaming state", ReadOnly = false, Destructive = false, Idempotent = true, OpenWorld = true, UseStructuredContent = true)]
	[Description("Enables or disables View3D streaming at runtime. The optional port does not update persistent profile settings.")]
	public Task<LDrawStreamingInfo> SetStreamingState(
		[Description("True to enable streaming, false to disable it.")] bool enable,
		[Description("Optional streaming port. Omit to use the current profile default.")] int? port = null,
		[Description("The instance id returned by ldraw_list_instances. Omit to target the broker instance.")] string? instance_id = null)
	{
		var parameters = new LDrawStreamingControlParams
		{
			Enable = enable,
			Port = port,
		};
		return m_broker.SetStreamingStateAsync(instance_id, parameters);
	}
}
