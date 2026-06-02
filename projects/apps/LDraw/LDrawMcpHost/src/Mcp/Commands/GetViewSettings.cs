using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Threading.Tasks;
using ModelContextProtocol.Server;

namespace LDraw.MCP;

/// <summary>Get-view-settings command broker routing</summary>
internal sealed partial class McpBroker
{
	/// <summary>Return view settings from 'instance_id'</summary>
	public async Task<LDrawViewSettingsInfo> GetViewSettingsAsync(string? instance_id, LDrawViewSettingsParams parameters)
	{
		var registration = await ResolveInstanceAsync(instance_id).ConfigureAwait(false);
		return await m_client.GetViewSettingsAsync(registration, parameters).ConfigureAwait(false);
	}
}

/// <summary>Get-view-settings command MCP tool surface</summary>
internal sealed partial class LDrawTools
{
	/// <summary>Return view and rendering settings from a running LDraw scene</summary>
	[McpServerTool(Name = "ldraw_get_view_settings", Title = "Get LDraw view settings", ReadOnly = true, Destructive = false, Idempotent = true, OpenWorld = false, UseStructuredContent = true)]
	[Description("Returns read-only view, camera, rendering, and diagnostic settings for a scene. Omit scene_name to use the first scene.")]
	public Task<LDrawViewSettingsInfo> GetViewSettings(
		[Description("The id of a running LDraw instance from ldraw_list_instances. Omit to target the default instance (most-recently-used, or auto-launched when none are running).")] string? instance_id = null,
		[Description("The scene name to query. Omit to use the first scene.")] string? scene_name = null)
	{
		var parameters = new LDrawViewSettingsParams
		{
			SceneName = scene_name,
		};
		return m_broker.GetViewSettingsAsync(instance_id, parameters);
	}
}
