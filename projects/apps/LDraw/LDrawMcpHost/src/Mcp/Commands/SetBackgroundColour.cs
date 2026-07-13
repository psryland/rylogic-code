using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Threading.Tasks;
using ModelContextProtocol.Server;

namespace LDraw.MCP;

/// <summary>Set-background-colour command broker routing</summary>
internal sealed partial class McpBroker
{
	/// <summary>Set background colour for a scene in 'instance_id'</summary>
	public async Task<LDrawViewMutationResult> SetBackgroundColourAsync(string? instance_id, LDrawSetBackgroundColourParams parameters)
	{
		var registration = await ResolveInstanceAsync(instance_id).ConfigureAwait(false);
		return await m_client.SetBackgroundColourAsync(registration, parameters).ConfigureAwait(false);
	}
}

/// <summary>Set-background-colour command pipe client call</summary>
internal sealed partial class InstancePipeClient
{
	/// <summary>Set background colour for a scene in 'registration'</summary>
	public async Task<LDrawViewMutationResult> SetBackgroundColourAsync(InstanceRegistration registration, LDrawSetBackgroundColourParams parameters)
	{
		return await SendAsync<LDrawViewMutationResult>(registration, InstancePipeCommands.SetBackgroundColour, parameters, WriteTimeout).ConfigureAwait(false);
	}
}

/// <summary>Set-background-colour command MCP tool surface</summary>
internal sealed partial class LDrawTools
{
	/// <summary>Set a scene background colour</summary>
	[McpServerTool(Name = "ldraw_set_background_colour", Title = "Set LDraw background colour", ReadOnly = false, Destructive = false, Idempotent = true, OpenWorld = false, UseStructuredContent = true)]
	[Description("Sets a scene background colour and persists the normal scene setting.")]
	public Task<LDrawViewMutationResult> SetBackgroundColour(
		[Description("Colour to apply, e.g. FF202020, #202020, or Black.")] string colour,
		[Description("The id of a running LDraw instance from ldraw_list_instances. Omit to target the default instance (most-recently-used, or auto-launched when none are running).")] string? instance_id = null,
		[Description("The scene name to modify. Omit to use the first scene.")] string? scene_name = null)
	{
		var parameters = new LDrawSetBackgroundColourParams
		{
			SceneName = scene_name,
			Colour = colour,
		};
		return m_broker.SetBackgroundColourAsync(instance_id, parameters);
	}
}
