using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Text.Json;
using Rylogic.Common;

namespace LDraw.MCP;

/// <summary>Maintains the per-user registry of running LDraw instances</summary>
public sealed class InstanceRegistry
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

		// Write to a temp file then move into place so a reader (the host polling for an auto-launched
		// instance, or a heartbeat overwrite) can never observe a half-written, parse-failing file.
		var temp = registration.FilePath + ".tmp";
		File.WriteAllText(temp, JsonSerializer.Serialize(registration, McpJson.Options));
		File.Move(temp, registration.FilePath, overwrite: true);
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
			// A null result means the file was missing or could not be parsed this pass. That can be a transient
			// read during an atomic overwrite, so skip it without deleting; a genuinely dead process is removed
			// only once its entry parses but fails the live-process check below.
			var registration = Read(filepath);
			if (registration == null)
				continue;
			if (!IsLive(registration))
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
			// Discovery is best-effort: a malformed entry is dropped rather than failing the whole listing.
			Trace.TraceWarning($"Invalid LDraw MCP instance registration '{filepath}': {ex.Message}");
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
			// A stale entry that cannot be deleted is harmless; it is re-evaluated on the next listing.
			Trace.TraceWarning($"Failed to delete stale LDraw MCP instance registration '{filepath}': {ex.Message}");
		}
	}
}
