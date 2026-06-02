using System;
using System.ComponentModel;
using System.Threading.Tasks;
using LDraw.UI;
using ModelContextProtocol.Server;

namespace LDraw.MCP;

/// <summary>Get-view-settings command broker routing</summary>
internal sealed partial class McpBroker
{
	/// <summary>Return view settings from 'instance_id'</summary>
	public async Task<LDrawViewSettingsInfo> GetViewSettingsAsync(string? instance_id, LDrawViewSettingsParams parameters)
	{
		var registration = ResolveInstance(instance_id);
		return await m_client.GetViewSettingsAsync(registration, parameters).ConfigureAwait(false);
	}
}

/// <summary>Get-view-settings command instance implementation</summary>
internal sealed partial class LDrawInstanceHost
{
	/// <summary>Return read-only view settings from the requested scene</summary>
	private Task<LDrawViewSettingsInfo> GetViewSettingsAsync(LDrawViewSettingsParams parameters)
	{
		return m_model.InvokeAsync(() =>
		{
			var scene = ResolveScene(parameters.SceneName);
			return CreateViewSettingsInfo(scene);
		}, TimeSpan.FromSeconds(5));
	}

	/// <summary>Create a read-only view settings DTO for 'scene'</summary>
	private static LDrawViewSettingsInfo CreateViewSettingsInfo(SceneUI scene)
	{
		var scene_view = scene.SceneView;
		var window = scene_view.Scene.Window;
		var camera = window.Camera;
		return new LDrawViewSettingsInfo
		{
			SceneName = scene.SceneName,
			Camera = CreateCameraInfo(scene),
			BackgroundColour = window.BackgroundColour.ToString(),
			Orthographic = camera.Orthographic,
			AlignAxis = LDrawVector3.From(camera.AlignAxis),
			AlignDirection = scene_view.AlignDirection.ToString(),
			ViewPreset = scene_view.ViewPreset.ToString(),
			NavigationMode = scene_view.NavigationMode.ToString(),
			XAxisRange = McpDtoConvert.AxisRange(scene_view.XAxis),
			YAxisRange = McpDtoConvert.AxisRange(scene_view.YAxis),
			FillMode = window.FillMode.ToString(),
			CullMode = window.CullMode.ToString(),
			Antialiasing = scene_view.Antialiasing,
			FocusPointVisible = scene_view.FocusPointVisible,
			OriginPointVisible = scene_view.OriginPointVisible,
			SelectionBoxVisible = scene_view.SelectionBoxVisible,
			ObjectInfoEnabled = scene_view.ObjectInfoEnabled,
			BBoxesVisible = window.Diag.BBoxesVisible,
			ShowNormals = scene_view.ShowNormals,
			NormalsLength = window.Diag.NormalsLength,
			NormalsColour = window.Diag.NormalsColour.ToString(),
			FillModePointsSize = window.Diag.FillModePointsSize.x,
			RayTracingAvailable = window.RayTracingAvailable,
			RayTracingEnabled = window.RayTracingEnabled,
			ShadowCastRange = scene_view.ShadowCastRange,
		};
	}
}

/// <summary>Get-view-settings command MCP tool surface</summary>
internal sealed partial class LDrawTools
{
	/// <summary>Return view and rendering settings from a running LDraw scene</summary>
	[McpServerTool(Name = "ldraw_get_view_settings", Title = "Get LDraw view settings", ReadOnly = true, Destructive = false, Idempotent = true, OpenWorld = false, UseStructuredContent = true)]
	[Description("Returns read-only view, camera, rendering, and diagnostic settings for a scene. Omit scene_name to use the first scene.")]
	public Task<LDrawViewSettingsInfo> GetViewSettings(
		[Description("The instance id returned by ldraw_list_instances. Omit to target the broker instance.")] string? instance_id = null,
		[Description("The scene name to query. Omit to use the first scene.")] string? scene_name = null)
	{
		var parameters = new LDrawViewSettingsParams
		{
			SceneName = scene_name,
		};
		return m_broker.GetViewSettingsAsync(instance_id, parameters);
	}
}
