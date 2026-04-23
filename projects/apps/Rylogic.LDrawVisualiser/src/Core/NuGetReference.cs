namespace Rylogic.LDrawVisualiser.Core
{
	/// <summary>A '#r "nuget: Id[, Version]"' reference parsed from a .csx script.
	/// Version may be null/empty or '*' to mean "highest installed version".</summary>
	public sealed class NuGetReference
	{
		public NuGetReference(string id, string? version)
		{
			Id = id;
			Version = version;
		}

		public string Id { get; }
		public string? Version { get; }

		public override string ToString() => Version == null ? Id : $"{Id}, {Version}";
	}
}
