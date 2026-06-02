using System;
using System.ComponentModel;
using System.Threading.Tasks;
using ModelContextProtocol.Server;

namespace LDraw.MCP;

/// <summary>Frame-selection command broker routing</summary>
internal sealed partial class McpBroker
{
	/// <summary>Frame selected objects in 'instance_id'</summary>
	public async Task<LDrawFrameResult> FrameSelectionAsync(string? instance_id, LDrawFrameSelectionParams parameters)
	{
		var registration = ResolveInstance(instance_id);
		return await m_client.FrameSelectionAsync(registration, parameters).ConfigureAwait(false);
	}
}

/// <summary>Frame-selection command pipe client call</summary>
internal sealed partial class InstancePipeClient
{
	/// <summary>Frame selected objects in 'registration'</summary>
	public async Task<LDrawFrameResult> FrameSelectionAsync(InstanceRegistration registration, LDrawFrameSelectionParams parameters)
	{
		return await SendAsync<LDrawFrameResult>(registration, InstancePipeCommands.FrameSelection, parameters, WriteTimeout).ConfigureAwait(false);
	}
}

/// <summary>Frame-selection command instance implementation</summary>
internal sealed partial class LDrawInstanceHost
{
	/// <summary>Frame the currently selected bounds in the requested scene</summary>
	private Task<LDrawFrameResult> FrameSelectionAsync(LDrawFrameSelectionParams parameters)
	{
		return m_model.InvokeAsync(() =>
		{
			var scene = ResolveScene(parameters.SceneName);
			var bounds = SelectedBounds(scene);
			if (!bounds.IsValid)
				throw new InvalidOperationException("Cannot frame selection because no selected object bounds are valid.");

			// The native selected-bounds path matches the existing user-facing Object Manager centre-view behaviour.
			var window = scene.SceneView.Scene.Window;
			window.Camera.ResetView(bounds);
			scene.SceneView.Invalidate();

			return new LDrawFrameResult
			{
				SceneName = scene.SceneName,
				Action = "frame_selection",
				Objects = CreateObjectInfos(SelectedObjects(scene, include_children: true, parameters.MaxCount)),
				Bounds = LDrawBoundsInfo.From(bounds),
				Camera = CreateCameraInfo(scene),
			};
		}, TimeSpan.FromSeconds(10));
	}
}

/// <summary>Frame-selection command MCP tool surface</summary>
internal sealed partial class LDrawTools
{
	/// <summary>Frame the currently selected objects in a running LDraw scene</summary>
	[McpServerTool(Name = "ldraw_frame_selection", Title = "Frame LDraw selection", ReadOnly = false, Destructive = false, Idempotent = true, OpenWorld = false, UseStructuredContent = true)]
	[Description("Frames the current selected objects. Fails if the selection has no valid bounds.")]
	public Task<LDrawFrameResult> FrameSelection(
		[Description("The instance id returned by ldraw_list_instances. Omit to target the broker instance.")] string? instance_id = null,
		[Description("The scene name to modify. Omit to use the first scene.")] string? scene_name = null,
		[Description("Maximum number of selected objects to include in the response, clamped to 1..1000.")] int max_count = 200)
	{
		var parameters = new LDrawFrameSelectionParams
		{
			SceneName = scene_name,
			MaxCount = max_count,
		};
		return m_broker.FrameSelectionAsync(instance_id, parameters);
	}
}
