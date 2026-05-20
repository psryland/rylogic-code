using System;
using System.ComponentModel;
using System.Linq;
using System.Threading.Tasks;
using ModelContextProtocol.Server;

namespace LDraw.MCP;

/// <summary>Routes MCP tool calls to registered LDraw instances</summary>
internal sealed class McpBroker
{
	private readonly InstanceRegistry m_registry;
	private readonly InstancePipeClient m_client;
	private readonly string m_local_instance_id;
	private readonly Func<string> m_endpoint;

	/// <summary>Create a broker for routing MCP requests to registered LDraw instances</summary>
	public McpBroker(InstanceRegistry registry, InstancePipeClient client, string local_instance_id, Func<string> endpoint)
	{
		m_registry = registry;
		m_client = client;
		m_local_instance_id = local_instance_id;
		m_endpoint = endpoint;
	}

	/// <summary>Return all live instances registered with the local broker</summary>
	public Task<McpInstanceInfo[]> ListInstancesAsync()
	{
		var instances = m_registry.LiveInstances()
			.Select(ToInfo)
			.ToArray();

		return Task.FromResult(instances);
	}

	/// <summary>Return a scene summary for 'instance_id', or the broker instance when omitted</summary>
	public async Task<LDrawSceneSummary> GetSceneSummaryAsync(string? instance_id)
	{
		var registration = ResolveInstance(instance_id);
		var summary = await m_client.GetSceneSummaryAsync(registration).ConfigureAwait(false);
		summary.Instance = ToInfo(registration);
		return summary;
	}

	/// <summary>Find an instance registration by id</summary>
	private InstanceRegistration ResolveInstance(string? instance_id)
	{
		var instances = m_registry.LiveInstances();

		// Most clients only connect to one LDraw process, so omitted instance id targets the process that won broker election.
		var id = string.IsNullOrWhiteSpace(instance_id) ? m_local_instance_id : instance_id;
		return instances.FirstOrDefault(x => string.Equals(x.InstanceId, id, StringComparison.OrdinalIgnoreCase))
			?? throw new InvalidOperationException($"No live LDraw instance with id '{id}' is registered.");
	}

	/// <summary>Convert a registry entry into public MCP output</summary>
	private McpInstanceInfo ToInfo(InstanceRegistration registration)
	{
		// Only the process that owns the HTTP listener has an MCP endpoint. Other instances are reached through the broker.
		var is_broker = string.Equals(registration.InstanceId, m_local_instance_id, StringComparison.OrdinalIgnoreCase);
		return new McpInstanceInfo
		{
			InstanceId = registration.InstanceId,
			ProcessId = registration.ProcessId,
			ProcessName = registration.ProcessName,
			StartedUtc = registration.StartedUtc,
			LastSeenUtc = registration.LastSeenUtc,
			IsBroker = is_broker,
			Endpoint = is_broker ? m_endpoint() : string.Empty,
			SettingsPath = registration.SettingsPath,
		};
	}
}

/// <summary>MCP tool surface exposed to AI clients</summary>
[McpServerToolType]
internal sealed class LDrawTools
{
	private readonly McpBroker m_broker;

	/// <summary>Create the tool surface bound to a broker instance</summary>
	public LDrawTools(McpBroker broker)
	{
		m_broker = broker;
	}

	/// <summary>List the running LDraw instances visible to the local broker</summary>
	[McpServerTool(Name = "ldraw_list_instances", Title = "List LDraw instances", ReadOnly = true, Destructive = false, Idempotent = true, OpenWorld = false, UseStructuredContent = true)]
	[Description("Lists running LDraw instances registered with the local MCP broker.")]
	public Task<McpInstanceInfo[]> ListInstances()
	{
		return m_broker.ListInstancesAsync();
	}

	/// <summary>Return a read-only summary of one LDraw instance</summary>
	[McpServerTool(Name = "ldraw_get_scene_summary", Title = "Get LDraw scene summary", ReadOnly = true, Destructive = false, Idempotent = true, OpenWorld = false, UseStructuredContent = true)]
	[Description("Returns sources, scenes, and object counts for a running LDraw instance. Omit instance_id to use the broker instance.")]
	public Task<LDrawSceneSummary> GetSceneSummary(
		[Description("The instance id returned by ldraw_list_instances. Omit to target the broker instance.")] string? instance_id = null)
	{
		return m_broker.GetSceneSummaryAsync(instance_id);
	}
}
