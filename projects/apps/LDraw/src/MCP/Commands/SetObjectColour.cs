using System;
using System.ComponentModel;
using System.Threading.Tasks;
using ModelContextProtocol.Server;

namespace LDraw.MCP;

/// <summary>Set-object-colour command broker routing</summary>
internal sealed partial class McpBroker
{
	/// <summary>Set colour for objects matching a query in 'instance_id'</summary>
	public async Task<LDrawObjectMutationResult> SetObjectColourAsync(string? instance_id, LDrawSetObjectColourParams parameters)
	{
		var registration = ResolveInstance(instance_id);
		return await m_client.SetObjectColourAsync(registration, parameters).ConfigureAwait(false);
	}
}

/// <summary>Set-object-colour command pipe client call</summary>
internal sealed partial class InstancePipeClient
{
	/// <summary>Set colour for objects in 'registration'</summary>
	public async Task<LDrawObjectMutationResult> SetObjectColourAsync(InstanceRegistration registration, LDrawSetObjectColourParams parameters)
	{
		return await SendAsync<LDrawObjectMutationResult>(registration, InstancePipeCommands.SetObjectColour, parameters, WriteTimeout).ConfigureAwait(false);
	}
}

/// <summary>Set-object-colour command instance implementation</summary>
internal sealed partial class LDrawInstanceHost
{
	/// <summary>Set or reset colour for matching objects in the requested scene</summary>
	private Task<LDrawObjectMutationResult> SetObjectColourAsync(LDrawSetObjectColourParams parameters)
	{
		return m_model.InvokeAsync(() =>
		{
			if (!parameters.Reset && string.IsNullOrWhiteSpace(parameters.Colour))
				throw new InvalidOperationException("ldraw_set_object_colour requires colour unless reset is true.");

			var scene = ResolveScene(parameters.SceneName);
			var query = QueryObjects(scene, parameters);
			var targets = ResolveMutationTargets(query, "ldraw_set_object_colour");
			var name = parameters.Recursive ? string.Empty : null;
			if (parameters.Reset)
			{
				foreach (var entry in targets)
					entry.Object.ResetColour(name);
			}
			else
			{
				var colour = ParseColour(parameters.Colour!);
				foreach (var entry in targets)
					entry.Object.ColourSet(parameters.BaseColour, colour, name);
			}

			scene.SceneView.Invalidate();
			return new LDrawObjectMutationResult
			{
				SceneName = scene.SceneName,
				Action = parameters.Reset ? "reset_object_colour" : parameters.BaseColour ? "set_object_base_colour" : "set_object_colour",
				Objects = CreateObjectInfos(targets),
			};
		}, TimeSpan.FromSeconds(10));
	}
}

/// <summary>Set-object-colour command MCP tool surface</summary>
internal sealed partial class LDrawTools
{
	/// <summary>Set or reset colours for objects in a running LDraw scene</summary>
	[McpServerTool(Name = "ldraw_set_object_colour", Title = "Set LDraw object colour", ReadOnly = false, Destructive = false, Idempotent = true, OpenWorld = false, UseStructuredContent = true)]
	[Description("Sets current object colour for debug highlighting, or resets current colour back to base colour. User source files are not edited.")]
	public Task<LDrawObjectMutationResult> SetObjectColour(
		[Description("Colour to apply, e.g. FFFF00FF, #FF00FF, or Red. Required unless reset is true.")] string? colour = null,
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
		[Description("Maximum number of matching objects to mutate, clamped to 1..1000. Throws if more matches exist.")] int max_count = 200,
		[Description("True to reset current colour back to base colour instead of applying colour.")] bool reset = false,
		[Description("True to modify base colour instead of current colour. Defaults false for temporary debug highlighting.")] bool base_colour = false,
		[Description("True to apply the colour operation recursively to children of each matched object.")] bool recursive = true)
	{
		var parameters = ObjectQueryParameters<LDrawSetObjectColourParams>(scene_name, object_id, name, type, context_id, selected, visible, include_children, match_mode, case_sensitive, max_count);
		parameters.Colour = colour;
		parameters.Reset = reset;
		parameters.BaseColour = base_colour;
		parameters.Recursive = recursive;
		return m_broker.SetObjectColourAsync(instance_id, parameters);
	}
}
