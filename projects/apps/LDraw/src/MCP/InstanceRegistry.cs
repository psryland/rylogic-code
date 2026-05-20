using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Text.Json;
using Rylogic.Common;
using Rylogic.Utility;

namespace LDraw.MCP;

/// <summary>Maintains the per-user registry of running LDraw instances</summary>
internal sealed class InstanceRegistry
{
	public InstanceRegistry(string user_data_dir)
	{
		Directory = Path_.CombinePath(user_data_dir, "MCP", "instances");
		Path_.CreateDirs(Directory);
	}

	/// <summary>The directory containing instance registry entries</summary>
	public string Directory { get; }

	/// <summary>Write or refresh a registry entry</summary>
	public void Write(InstanceRegistration registration)
	{
		// The registry file is the lightweight discovery mechanism shared by all LDraw processes in this user profile.
		registration.LastSeenUtc = DateTimeOffset.UtcNow;
		registration.FilePath = EntryFilePath(registration.InstanceId);
		File.WriteAllText(registration.FilePath, JsonSerializer.Serialize(registration, McpJson.Options));
	}

	/// <summary>Delete a registry entry</summary>
	public void Delete(string instance_id)
	{
		var filepath = EntryFilePath(instance_id);
		if (Path_.FileExists(filepath))
			File.Delete(filepath);
	}

	/// <summary>Return all live registered instances</summary>
	public IReadOnlyList<InstanceRegistration> LiveInstances()
	{
		var instances = new List<InstanceRegistration>();
		foreach (var filepath in DirectoryFiles())
		{
			// Clean stale or malformed entries opportunistically so discovery stays stable after crashes.
			var registration = Read(filepath);
			if (registration == null || !IsLive(registration))
			{
				DeleteFile(filepath);
				continue;
			}

			instances.Add(registration);
		}

		return instances.OrderBy(x => x.StartedUtc).ToArray();
	}

	/// <summary>Return the registry file path for 'instance_id'</summary>
	private string EntryFilePath(string instance_id)
	{
		return Path_.CombinePath(Directory, $"{instance_id}.json");
	}

	/// <summary>Enumerate the registry files</summary>
	private IEnumerable<string> DirectoryFiles()
	{
		if (!Path_.DirExists(Directory))
			yield break;

		foreach (var file in System.IO.Directory.EnumerateFiles(Directory, "*.json", SearchOption.TopDirectoryOnly))
			yield return file;
	}

	/// <summary>Read an instance registration file</summary>
	private static InstanceRegistration? Read(string filepath)
	{
		try
		{
			var registration = JsonSerializer.Deserialize<InstanceRegistration>(File.ReadAllText(filepath), McpJson.Options);
			if (registration != null)
				registration.FilePath = filepath;
			return registration;
		}
		catch (Exception ex)
		{
			Log.Write(ELogLevel.Warn, ex, "Invalid LDraw MCP instance registration.", filepath, 0, 0);
			return null;
		}
	}

	/// <summary>Return true if 'registration' refers to a live LDraw process</summary>
	private static bool IsLive(InstanceRegistration registration)
	{
		if (registration.SchemaVersion != 1)
			return false;
		if (registration.ProcessId <= 0 || registration.InstanceId.Length == 0 || registration.PipeName.Length == 0)
			return false;

		try
		{
			using var process = Process.GetProcessById(registration.ProcessId);

			// PIDs are recycled, so confirm the process name still matches the process that wrote the entry.
			return string.Equals(process.ProcessName, registration.ProcessName, StringComparison.OrdinalIgnoreCase);
		}
		catch
		{
			return false;
		}
	}

	/// <summary>Delete 'filepath' and log failures</summary>
	private static void DeleteFile(string filepath)
	{
		try
		{
			File.Delete(filepath);
		}
		catch (Exception ex)
		{
			Log.Write(ELogLevel.Warn, ex, "Failed to delete stale LDraw MCP instance registration.", filepath, 0, 0);
		}
	}
}
