using System;
using System.ComponentModel;
using System.Linq;
using System.Threading.Tasks;
using ModelContextProtocol.Server;
using Rylogic.Maths;

namespace LDraw.MCP;

/// <summary>Object-transform command broker routing</summary>
internal sealed partial class McpBroker
{
	/// <summary>Return transform data for an object in 'instance_id'</summary>
	public async Task<LDrawObjectInfo> GetObjectTransformAsync(string? instance_id, LDrawGetObjectTransformParams parameters)
	{
		var registration = ResolveInstance(instance_id);
		return await m_client.GetObjectTransformAsync(registration, parameters).ConfigureAwait(false);
	}

	/// <summary>Set transform data for objects in 'instance_id'</summary>
	public async Task<LDrawObjectMutationResult> SetObjectTransformAsync(string? instance_id, LDrawSetObjectTransformParams parameters)
	{
		var registration = ResolveInstance(instance_id);
		return await m_client.SetObjectTransformAsync(registration, parameters).ConfigureAwait(false);
	}
}

/// <summary>Object-transform command pipe client calls</summary>
internal sealed partial class InstancePipeClient
{
	/// <summary>Return transform data for an object in 'registration'</summary>
	public async Task<LDrawObjectInfo> GetObjectTransformAsync(InstanceRegistration registration, LDrawGetObjectTransformParams parameters)
	{
		return await SendAsync<LDrawObjectInfo>(registration, InstancePipeCommands.GetObjectTransform, parameters, WriteTimeout).ConfigureAwait(false);
	}

	/// <summary>Set transform data for objects in 'registration'</summary>
	public async Task<LDrawObjectMutationResult> SetObjectTransformAsync(InstanceRegistration registration, LDrawSetObjectTransformParams parameters)
	{
		return await SendAsync<LDrawObjectMutationResult>(registration, InstancePipeCommands.SetObjectTransform, parameters, WriteTimeout).ConfigureAwait(false);
	}
}

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

/// <summary>Object-transform command MCP tool surface</summary>
internal sealed partial class LDrawTools
{
	/// <summary>Return object-to-parent and object-to-world transforms for a running LDraw object</summary>
	[McpServerTool(Name = "ldraw_get_object_transform", Title = "Get LDraw object transform", ReadOnly = true, Destructive = false, Idempotent = true, OpenWorld = false, UseStructuredContent = true)]
	[Description("Returns object-to-parent and object-to-world matrices for exactly one object matched by query. Matrices are column-major 16-value arrays.")]
	public Task<LDrawObjectInfo> GetObjectTransform(
		[Description("The instance id returned by ldraw_list_instances. Omit to target the broker instance.")] string? instance_id = null,
		[Description("The scene name to query. Omit to use the first scene.")] string? scene_name = null,
		[Description("Opaque object id returned by ldraw_find_objects or ldraw_list_objects.")] string? object_id = null,
		[Description("Object name filter.")] string? name = null,
		[Description("Object type filter, for example Model, Group, Line, or Box.")] string? type = null,
		[Description("Optional source context id returned by ldraw_get_scene_summary or ldraw_list_scenes.")] string? context_id = null,
		[Description("Optional selected-state filter.")] bool? selected = null,
		[Description("Optional visible-state filter.")] bool? visible = null,
		[Description("True to recursively include child objects when matching.")] bool include_children = true,
		[Description("Name/type matching mode: exact, contains, or regex.")] string match_mode = "contains",
		[Description("True for case-sensitive name/type matching.")] bool case_sensitive = false,
		[Description("Maximum number of matching objects to inspect, clamped to 1..1000. Throws unless exactly one object matches.")] int max_count = 200)
	{
		var parameters = ObjectQueryParameters<LDrawGetObjectTransformParams>(scene_name, object_id, name, type, context_id, selected, visible, include_children, match_mode, case_sensitive, max_count);
		return m_broker.GetObjectTransformAsync(instance_id, parameters);
	}

	/// <summary>Set object transforms in parent space or world space for running LDraw objects</summary>
	[McpServerTool(Name = "ldraw_set_object_transform", Title = "Set LDraw object transform", ReadOnly = false, Destructive = false, Idempotent = true, OpenWorld = false, UseStructuredContent = true)]
	[Description("Sets object transform matrix or translation in parent/world space for objects matched by query. User source files are not edited.")]
	public Task<LDrawObjectMutationResult> SetObjectTransform(
		[Description("Transform space to read or write: parent or world.")] string space = "parent",
		[Description("Optional replacement 16-value transform matrix in column-major order.")] double[]? matrix = null,
		[Description("Optional replacement X position in the selected space.")] double? position_x = null,
		[Description("Optional replacement Y position in the selected space.")] double? position_y = null,
		[Description("Optional replacement Z position in the selected space.")] double? position_z = null,
		[Description("Optional X translation delta in the selected space.")] double? delta_x = null,
		[Description("Optional Y translation delta in the selected space.")] double? delta_y = null,
		[Description("Optional Z translation delta in the selected space.")] double? delta_z = null,
		[Description("The instance id returned by ldraw_list_instances. Omit to target the broker instance.")] string? instance_id = null,
		[Description("The scene name to modify. Omit to use the first scene.")] string? scene_name = null,
		[Description("Opaque object id returned by ldraw_find_objects or ldraw_list_objects.")] string? object_id = null,
		[Description("Object name filter.")] string? name = null,
		[Description("Object type filter, for example Model, Group, Line, or Box.")] string? type = null,
		[Description("Optional source context id returned by ldraw_get_scene_summary or ldraw_list_scenes.")] string? context_id = null,
		[Description("Optional selected-state filter.")] bool? selected = null,
		[Description("Optional visible-state filter.")] bool? visible = null,
		[Description("True to recursively include child objects when matching.")] bool include_children = true,
		[Description("Name/type matching mode: exact, contains, or regex.")] string match_mode = "contains",
		[Description("True for case-sensitive name/type matching.")] bool case_sensitive = false,
		[Description("Maximum number of matching objects to mutate, clamped to 1..1000. Throws if more matches exist.")] int max_count = 200)
	{
		var parameters = ObjectQueryParameters<LDrawSetObjectTransformParams>(scene_name, object_id, name, type, context_id, selected, visible, include_children, match_mode, case_sensitive, max_count);
		parameters.Space = space;
		parameters.Matrix = matrix;
		parameters.PositionX = position_x;
		parameters.PositionY = position_y;
		parameters.PositionZ = position_z;
		parameters.DeltaX = delta_x;
		parameters.DeltaY = delta_y;
		parameters.DeltaZ = delta_z;
		return m_broker.SetObjectTransformAsync(instance_id, parameters);
	}
}
