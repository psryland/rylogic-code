using System;
using System.ComponentModel;
using System.Threading.Tasks;
using ModelContextProtocol.Server;
using Rylogic.Maths;

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

/// <summary>Frame-bounds command instance implementation</summary>
internal sealed partial class LDrawInstanceHost
{
	/// <summary>Frame explicit bounds in the requested scene</summary>
	private Task<LDrawFrameResult> FrameBoundsAsync(LDrawFrameBoundsParams parameters)
	{
		return m_model.InvokeAsync(() =>
		{
			var scene = ResolveScene(parameters.SceneName);
			var bounds = BoundsFromParameters(parameters);
			if (!bounds.IsValid)
				throw new InvalidOperationException("Cannot frame explicit bounds because the bounds are not valid.");

			// Explicit bounds are already in scene/world space, so the camera can frame them directly.
			var window = scene.SceneView.Scene.Window;
			window.Camera.ResetView(bounds);
			scene.SceneView.Invalidate();
			return new LDrawFrameResult
			{
				SceneName = scene.SceneName,
				Action = "frame_bounds",
				Bounds = LDrawBoundsInfo.From(bounds),
				Camera = CreateCameraInfo(scene),
			};
		}, TimeSpan.FromSeconds(10));
	}

	/// <summary>Create a bounding box from either min/max or centre/radius parameters</summary>
	private static BBox BoundsFromParameters(LDrawFrameBoundsParams parameters)
	{
		var min = VectorOrNull(parameters.MinX, parameters.MinY, parameters.MinZ, "min");
		var max = VectorOrNull(parameters.MaxX, parameters.MaxY, parameters.MaxZ, "max");
		var centre = VectorOrNull(parameters.CentreX, parameters.CentreY, parameters.CentreZ, "centre");
		var radius = VectorOrNull(parameters.RadiusX, parameters.RadiusY, parameters.RadiusZ, "radius");
		var has_min_max = min != null || max != null;
		var has_centre_radius = centre != null || radius != null;

		// Require exactly one complete representation so the tool contract stays easy for MCP clients to reason about.
		if (has_min_max == has_centre_radius)
			throw new InvalidOperationException("Frame bounds requires exactly one of min/max or centre/radius.");
		if (has_min_max)
		{
			if (min == null || max == null)
				throw new InvalidOperationException("Frame bounds using min/max requires all min_x/y/z and max_x/y/z values.");
			return BBox.From(min.Value, max.Value);
		}

		if (centre == null || radius == null)
			throw new InvalidOperationException("Frame bounds using centre/radius requires all centre_x/y/z and radius_x/y/z values.");
		if (radius.Value.x < 0f || radius.Value.y < 0f || radius.Value.z < 0f)
			throw new InvalidOperationException("Frame bounds radius components must be non-negative.");
		return new BBox(centre.Value, radius.Value);
	}

	/// <summary>Create an optional vector from nullable components and reject partial vectors</summary>
	private static v4? VectorOrNull(double? x, double? y, double? z, string name)
	{
		if (x == null && y == null && z == null)
			return null;
		if (x == null || y == null || z == null)
			throw new InvalidOperationException($"Vector '{name}' requires all three x, y, and z components.");

		return new v4((float)x.Value, (float)y.Value, (float)z.Value, name is "radius" ? 0f : 1f);
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
