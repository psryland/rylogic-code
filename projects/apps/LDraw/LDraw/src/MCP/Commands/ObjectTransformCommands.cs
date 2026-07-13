using System;
using System.ComponentModel;
using System.Linq;
using System.Threading.Tasks;
using Rylogic.Maths;

namespace LDraw.MCP;

/// <summary>Object-transform command instance implementation</summary>
internal sealed partial class LDrawInstanceHost
{
	/// <summary>Return transform data for one object in the requested scene</summary>
	private Task<LDrawObjectInfo> GetObjectTransformAsync(LDrawGetObjectTransformParams parameters)
	{
		return m_model.InvokeAsync(() =>
		{
			var scene = ResolveScene(parameters.SceneName);
			var query = QueryObjects(scene, parameters);
			var target = ResolveSingleObject(query, "ldraw_get_object_transform");
			return CreateObjectInfo(target);
		}, TimeSpan.FromSeconds(10));
	}

	/// <summary>Set parent-space or world-space transform data for matching objects in the requested scene</summary>
	private Task<LDrawObjectMutationResult> SetObjectTransformAsync(LDrawSetObjectTransformParams parameters)
	{
		return m_model.InvokeAsync(() =>
		{
			if (!HasTransformMutation(parameters))
				throw new InvalidOperationException("ldraw_set_object_transform requires matrix, position, or delta parameters.");

			var scene = ResolveScene(parameters.SceneName);
			var space = TransformSpace(parameters.Space);
			var query = QueryObjects(scene, parameters);
			var targets = ResolveMutationTargets(query, "ldraw_set_object_transform");
			foreach (var entry in targets)
			{
				var transform = parameters.Matrix != null
					? MatrixFrom(parameters.Matrix)
					: space switch
					{
						"parent" => entry.Object.O2PGet(null),
						"world" => entry.Object.O2WGet(null),
						_ => throw new ArgumentOutOfRangeException(nameof(parameters.Space), parameters.Space, "Unknown transform space."),
					};

				transform.pos.x = (float)(parameters.PositionX ?? transform.pos.x);
				transform.pos.y = (float)(parameters.PositionY ?? transform.pos.y);
				transform.pos.z = (float)(parameters.PositionZ ?? transform.pos.z);
				transform.pos.x += (float)(parameters.DeltaX ?? 0.0);
				transform.pos.y += (float)(parameters.DeltaY ?? 0.0);
				transform.pos.z += (float)(parameters.DeltaZ ?? 0.0);
				transform.pos.w = 1.0f;

				switch (space)
				{
					case "parent":
					{
						entry.Object.O2PSet(transform, null);
						break;
					}
					case "world":
					{
						entry.Object.O2WSet(transform, null);
						break;
					}
					default:
					{
						throw new ArgumentOutOfRangeException(nameof(parameters.Space), parameters.Space, "Unknown transform space.");
					}
				}
			}

			scene.SceneView.Invalidate();
			return new LDrawObjectMutationResult
			{
				SceneName = scene.SceneName,
				Action = $"set_object_transform_{space}",
				Objects = CreateObjectInfos(targets),
			};
		}, TimeSpan.FromSeconds(10));
	}

	/// <summary>Return true if any transform mutation parameter was supplied</summary>
	private static bool HasTransformMutation(LDrawSetObjectTransformParams parameters)
	{
		return
			parameters.Matrix != null ||
			parameters.PositionX != null ||
			parameters.PositionY != null ||
			parameters.PositionZ != null ||
			parameters.DeltaX != null ||
			parameters.DeltaY != null ||
			parameters.DeltaZ != null;
	}

	/// <summary>Normalise the requested transform space</summary>
	private static string TransformSpace(string? space)
	{
		var value = string.IsNullOrWhiteSpace(space) ? "parent" : space.Trim().ToLowerInvariant();
		return value switch
		{
			"parent" => "parent",
			"world" => "world",
			_ => throw new InvalidOperationException("Transform space must be 'parent' or 'world'."),
		};
	}

	/// <summary>Create a View3D matrix from a column-major array</summary>
	private static m4x4 MatrixFrom(double[] matrix)
	{
		if (matrix.Length != 16)
			throw new InvalidOperationException("Transform matrix must contain exactly 16 values in column-major order.");

		var values = matrix.Select(x => (float)x).ToArray();
		var result = new m4x4(values);
		if (result.pos.w != 1.0f)
			throw new InvalidOperationException("Transform matrix position column must have w = 1.");

		return result;
	}
}
