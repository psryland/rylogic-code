using System;
using System.ComponentModel;
using System.Threading.Tasks;
using Rylogic.Maths;

namespace LDraw.MCP;

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
