using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Threading.Tasks;
using ModelContextProtocol.Server;

namespace LDraw.MCP;

/// <summary>Get-diagnostic-modes command broker routing</summary>
internal sealed partial class McpBroker
{
	/// <summary>Return diagnostic modes from 'instance_id'</summary>
	public async Task<LDrawDiagnosticModesInfo> GetDiagnosticModesAsync(string? instance_id, LDrawDiagnosticModesParams parameters)
	{
		var registration = await ResolveInstanceAsync(instance_id).ConfigureAwait(false);
		return await m_client.GetDiagnosticModesAsync(registration, parameters).ConfigureAwait(false);
	}
}

/// <summary>Get-diagnostic-modes command pipe client call</summary>
internal sealed partial class InstancePipeClient
{
	/// <summary>Read diagnostic modes from 'registration'</summary>
	public async Task<LDrawDiagnosticModesInfo> GetDiagnosticModesAsync(InstanceRegistration registration, LDrawDiagnosticModesParams parameters)
	{
		return await SendAsync<LDrawDiagnosticModesInfo>(registration, InstancePipeCommands.GetDiagnosticModes, parameters, ReadTimeout).ConfigureAwait(false);
	}
}

/// <summary>Get-diagnostic-modes command MCP tool surface</summary>
internal sealed partial class LDrawTools
{
	/// <summary>Return diagnostic and rendering modes for a running LDraw scene</summary>
	[McpServerTool(Name = "ldraw_get_diagnostic_modes", Title = "Get LDraw diagnostic modes", ReadOnly = true, Destructive = false, Idempotent = true, OpenWorld = false, UseStructuredContent = true)]
	[Description("Returns diagnostic and rendering modes for a scene, including accepted fill/cull mode names. Omit scene_name to use the first scene.")]
	public Task<LDrawDiagnosticModesInfo> GetDiagnosticModes(
		[Description("The id of a running LDraw instance from ldraw_list_instances. Omit to target the default instance (most-recently-used, or auto-launched when none are running).")] string? instance_id = null,
		[Description("The scene name to query. Omit to use the first scene.")] string? scene_name = null)
	{
		var parameters = new LDrawDiagnosticModesParams
		{
			SceneName = scene_name,
		};
		return m_broker.GetDiagnosticModesAsync(instance_id, parameters);
	}
}
