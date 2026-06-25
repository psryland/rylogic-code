using System;
using System.ComponentModel;
using System.Threading.Tasks;
using ModelContextProtocol.Server;

namespace LDraw.MCP;

/// <summary>Get-log command broker routing</summary>
internal sealed partial class McpBroker
{
	/// <summary>Return recent application-log entries from 'instance_id'</summary>
	public async Task<LDrawLogInfo> GetLogAsync(string? instance_id, LDrawGetLogParams parameters)
	{
		var registration = await ResolveInstanceAsync(instance_id).ConfigureAwait(false);
		return await m_client.GetLogAsync(registration, parameters).ConfigureAwait(false);
	}
}

/// <summary>Get-log command pipe client call</summary>
internal sealed partial class InstancePipeClient
{
	/// <summary>Read recent application-log entries from 'registration'</summary>
	public async Task<LDrawLogInfo> GetLogAsync(InstanceRegistration registration, LDrawGetLogParams parameters)
	{
		return await SendAsync<LDrawLogInfo>(registration, InstancePipeCommands.GetLog, parameters, ReadTimeout).ConfigureAwait(false);
	}
}

/// <summary>Get-log command MCP tool surface</summary>
internal sealed partial class LDrawTools
{
	/// <summary>Return recent entries from the LDraw application log</summary>
	[McpServerTool(Name = "ldraw_get_log", Title = "Get LDraw log", ReadOnly = true, Destructive = false, Idempotent = true, OpenWorld = false, UseStructuredContent = true)]
	[Description("Returns recent LDraw application log entries, including scene parse and load errors with their source file, line, and column offset. Use this to diagnose problems in a loaded scene. Defaults to warnings and errors. Poll incrementally by passing the previous result's next_index as since_index.")]
	public Task<LDrawLogInfo> GetLog(
		[Description("The id of a running LDraw instance from ldraw_list_instances. Omit to target the default instance (most-recently-used, or auto-launched when none are running).")] string? instance_id = null,
		[Description("Minimum severity to include: Debug, Info, Warn, Error, or Fatal. Omit for Warn (warnings and errors).")] string? min_level = null,
		[Description("Only return entries newer than this index. Pass the previous result's next_index to fetch only new entries. Omit or -1 for all.")] long since_index = -1,
		[Description("Maximum number of entries to return, clamped to 1..1000. The newest matching entries are kept.")] int max_count = 200,
		[Description("Optional case-insensitive substring filter on the message text.")] string? contains = null,
		[Description("Optional case-insensitive substring filter on the entry's source file path.")] string? file = null)
	{
		var parameters = new LDrawGetLogParams
		{
			MinLevel = min_level,
			SinceIndex = since_index,
			MaxCount = max_count,
			Contains = contains,
			File = file,
		};
		return m_broker.GetLogAsync(instance_id, parameters);
	}
}
