using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Threading.Tasks;
using ModelContextProtocol.Server;

namespace LDraw.MCP;

/// <summary>Frame-bounds command broker routing</summary>
internal sealed partial class McpBroker
{
	/// <summary>Frame explicit bounds in 'instance_id'</summary>
	public async Task<LDrawFrameResult> FrameBoundsAsync(string? instance_id, LDrawFrameBoundsParams parameters)
	{
		var registration = ResolveInstance(instance_id);
		return await m_client.FrameBoundsAsync(registration, parameters).ConfigureAwait(false);
	}
}

/// <summary>Frame-bounds command pipe client call</summary>
internal sealed partial class InstancePipeClient
{
	/// <summary>Frame explicit bounds in 'registration'</summary>
	public async Task<LDrawFrameResult> FrameBoundsAsync(InstanceRegistration registration, LDrawFrameBoundsParams parameters)
	{
		return await SendAsync<LDrawFrameResult>(registration, InstancePipeCommands.FrameBounds, parameters, WriteTimeout).ConfigureAwait(false);
	}
}

/// <summary>Frame-bounds command MCP tool surface</summary>
internal sealed partial class LDrawTools
{
	/// <summary>Frame explicit bounds in a running LDraw scene</summary>
	[McpServerTool(Name = "ldraw_frame_bounds", Title = "Frame LDraw bounds", ReadOnly = false, Destructive = false, Idempotent = true, OpenWorld = false, UseStructuredContent = true)]
	[Description("Frames explicit bounds. Provide either min_x/y/z plus max_x/y/z, or centre_x/y/z plus radius_x/y/z.")]
	public Task<LDrawFrameResult> FrameBounds(
		[Description("The instance id returned by ldraw_list_instances. Omit to target the broker instance.")] string? instance_id = null,
		[Description("The scene name to modify. Omit to use the first scene.")] string? scene_name = null,
		[Description("Minimum corner X, used with max_x/y/z.")] double? min_x = null,
		[Description("Minimum corner Y, used with max_x/y/z.")] double? min_y = null,
		[Description("Minimum corner Z, used with max_x/y/z.")] double? min_z = null,
		[Description("Maximum corner X, used with min_x/y/z.")] double? max_x = null,
		[Description("Maximum corner Y, used with min_x/y/z.")] double? max_y = null,
		[Description("Maximum corner Z, used with min_x/y/z.")] double? max_z = null,
		[Description("Centre point X, used with radius_x/y/z.")] double? centre_x = null,
		[Description("Centre point Y, used with radius_x/y/z.")] double? centre_y = null,
		[Description("Centre point Z, used with radius_x/y/z.")] double? centre_z = null,
		[Description("Radius X, used with centre_x/y/z.")] double? radius_x = null,
		[Description("Radius Y, used with centre_x/y/z.")] double? radius_y = null,
		[Description("Radius Z, used with centre_x/y/z.")] double? radius_z = null)
	{
		var parameters = new LDrawFrameBoundsParams
		{
			SceneName = scene_name,
			MinX = min_x,
			MinY = min_y,
			MinZ = min_z,
			MaxX = max_x,
			MaxY = max_y,
			MaxZ = max_z,
			CentreX = centre_x,
			CentreY = centre_y,
			CentreZ = centre_z,
			RadiusX = radius_x,
			RadiusY = radius_y,
			RadiusZ = radius_z,
		};
		return m_broker.FrameBoundsAsync(instance_id, parameters);
	}
}
