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

	// The last successfully parsed registration per entry file, with the file write time it was parsed from. Opening a file is
	// slow and can fail briefly while a heartbeat replaces it, so entries are parsed again only when their write time changes.
	private readonly Dictionary<string, (DateTime write_time_utc, InstanceRegistration registration)> m_cache = new(StringComparer.OrdinalIgnoreCase);

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
		// Serialise listings because the cache is shared by concurrent callers.
		lock (m_cache)
		{
			var instances = new List<InstanceRegistration>();
			var present = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
			foreach (var file in DirectoryFiles())
			{
				// The directory listing provides write times without opening files. Parse an entry only when it is new or rewritten.
				// If the parse fails (usually a read during an atomic overwrite), keep the cached copy and retry on the next listing.
				var filepath = file.FullName;
				present.Add(filepath);
				var write_time_utc = file.LastWriteTimeUtc;
				var cached = m_cache.TryGetValue(filepath, out var entry);
				if (!cached || entry.write_time_utc != write_time_utc)
				{
					if (Read(filepath) is InstanceRegistration fresh)
						m_cache[filepath] = entry = (write_time_utc, fresh);
					else if (!cached)
						continue;
				}

				// A genuinely dead process is removed only by the live-process check.
				if (!IsLive(entry.registration))
				{
					m_cache.Remove(filepath);
					DeleteFile(filepath);
					continue;
				}

				instances.Add(entry.registration);
			}

			// Drop cache entries whose files have been deleted.
			foreach (var stale in m_cache.Keys.Where(x => !present.Contains(x)).ToArray())
				m_cache.Remove(stale);

			return instances.OrderBy(x => x.StartedUtc).ToArray();
		}
	}

	/// <summary>Return the registry file path for 'instance_id'</summary>
	private string EntryFilePath(string instance_id)
	{
		return Path_.CombinePath(Directory, $"{instance_id}.json");
	}

	/// <summary>Enumerate the registry files</summary>
	private IEnumerable<FileInfo> DirectoryFiles()
	{
		if (!Path_.DirExists(Directory))
			yield break;

		foreach (var file in new DirectoryInfo(Directory).EnumerateFiles("*.json", SearchOption.TopDirectoryOnly))
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
