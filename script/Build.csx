#! "net10.0"
#r "System.IO"
#r "System.Text.Json"
#r "nuget: Rylogic.Core, 2.1.0"
#load "UserVars.csx"
#load "Tools.csx"
#load "BuildInstaller.csx"
#load "Nuget.csx"
#load "NativeRuntimePackage.csx"
#nullable enable

using System;
using System.Diagnostics;
using System.Text.RegularExpressions;
using System.Runtime.InteropServices;
using System.Xml.Linq;
using Rylogic.Extn;
using Console = System.Console;
using IOPath = System.IO.Path;

// Available projects that can be built
public enum EProjects
{
	Sqlite3,            // = "Sqlite3";
	Scintilla,          // = "Scintilla";
	Audio,              // = "Audio";
	Fbx,                // = "Fbx";
	Gltf,               // = "Gltf";
	Imgui,              // = "Imgui";
	Compute,            // = "Compute";
	Physics,            // = "Physics";
	View3d,             // = "View3d";
	View3dUI,           // = "View3dUI";
	P3d,                // = "P3d";
	RylogicAudio,       // = "Rylogic.Audio";
	RylogicCore,        // = "Rylogic.Core";
	RylogicD3D12,       // = "Rylogic.D3D12";
	RylogicDB,          // = "Rylogic.DB";
	RylogicDirectShow,  // = "Rylogic.DirectShow";
	RylogicGfx,         // = "Rylogic.Gfx";
	RylogicGfxWPF,      // = "Rylogic.Gfx.WPF";
	RylogicGuiWPF,      // = "Rylogic.Gui.WPF";
	RylogicNet,         // = "Rylogic.Net";
	RylogicPhysics,     // = "Rylogic.Physics";
	RylogicScintilla,   // = "Rylogic.Scintilla";
	RylogicWindows,     // = "Rylogic.Windows";
	Csex,               // = "Csex";
	LDraw,              // = "LDraw";
	RyLogViewer,        // = "RyLogViewer";
	RylogicTextAligner, // = "Rylogic.TextAligner";
	AllNative,          // = "AllNative";
	AllManaged,         // = "AllManaged";
	AllRylogic,         // = "AllRylogic";
	All,                // = "All";
}

// Get the version number of the Rylogic library
public static string RylogicLibraryVersion
{
	get => Tools.Extract(Tools.Path([UserVars.Root, "Directory.Build.props"]), new Regex("<RylogicLibraryVersion>(?<Version>.*)</RylogicLibraryVersion>")).Groups["Version"].Value;
}

// Base class for all builders
public abstract class Common
{
	public string Workspace;
	public string RylogicSln;

	public Common(string workspace)
	{
		Workspace = workspace;
		RylogicSln = Tools.Path([Workspace, "Rylogic.sln"]);
	}
	public virtual void Clean() { }
	public virtual void Deploy() { }
	public virtual void Publish() { }

	// Adds the solution projects this builder needs to 'plan'. Builders never invoke MSBuild directly; the plan builds every selected project together.
	public virtual void Build(BuildPlan plan) { }
}

// Options parsed from the Build.csx command line.
public static class BuildOptions
{
	public static bool Restore { get; set; } = true;
	public static bool RunUnitTests { get; set; } = true;
	public static bool GeneratePackageOnBuild { get; set; } = true;
	public static bool Profile { get; set; } = false;
	public static string ProfileDir { get; set; } = string.Empty;
}

// Build phases, in execution order. Managed projects copy native DLLs from 'lib\<platform>\<config>' and Debug managed projects restore the
// local Rylogic.Native package that the native phase publishes, so every native project must finish before any managed project starts.
public enum EPhase
{
	Native,
	Managed,
}

// The set of solution projects to build, grouped by phase, platform, and configuration.
public sealed class BuildPlan
{
	// Managed projects are built through this solution platform. Rylogic.sln maps 'x64' and 'Any CPU' to the same project configurations,
	// so using one platform guarantees that each managed project is built at most once per configuration.
	public const string ManagedSolutionPlatform = "x64";

	private readonly Dictionary<(EPhase Phase, string Platform, string Config), SortedSet<string>> m_projects = [];

	// Adds native project files (full paths) for every combination of 'platforms' and 'configs'.
	public void AddNative(IEnumerable<string> platforms, IEnumerable<string> configs, params string[] project_files)
	{
		// Native outputs are platform-specific, so each requested platform is a separate build.
		foreach (var platform in platforms)
			Add(EPhase.Native, platform, configs, project_files);
	}

	// Adds managed project files (full paths) for every configuration in 'configs'.
	public void AddManaged(IEnumerable<string> configs, params string[] project_files)
	{
		// Managed projects are platform-neutral; see 'ManagedSolutionPlatform'.
		Add(EPhase.Managed, ManagedSolutionPlatform, configs, project_files);
	}

	// Returns the platform/config builds for 'phase'. Debug sorts last because the shared project.assets.json keeps the last restored configuration,
	// and only Debug builds reference the local Rylogic.Native package.
	public IReadOnlyList<(string Platform, string Config, IReadOnlyCollection<string> Projects)> Builds(EPhase phase)
	{
		// Order the builds so restores run deterministically with Debug last.
		return m_projects
			.Where(x => x.Key.Phase == phase)
			.OrderBy(x => string.Equals(x.Key.Config, "Debug", StringComparison.OrdinalIgnoreCase))
			.ThenBy(x => x.Key.Config, StringComparer.OrdinalIgnoreCase)
			.ThenBy(x => x.Key.Platform, StringComparer.OrdinalIgnoreCase)
			.Select(x => (x.Key.Platform, x.Key.Config, (IReadOnlyCollection<string>)x.Value))
			.ToList();
	}

	// Records each project once per phase, platform, and configuration so overlapping builder selections do not build twice.
	private void Add(EPhase phase, string platform, IEnumerable<string> configs, string[] project_files)
	{
		// Use the full path as the identity so different spellings of the same project are merged.
		foreach (var config in configs)
		{
			var key = (phase, platform, config);
			if (!m_projects.TryGetValue(key, out var projects))
				m_projects[key] = projects = new SortedSet<string>(StringComparer.OrdinalIgnoreCase);

			foreach (var project_file in project_files)
				projects.Add(IOPath.GetFullPath(project_file));
		}
	}
}

// Executes a BuildPlan with as few MSBuild invocations as the phase order allows.
// Each phase is one MSBuild process that builds every platform/config at once. It uses a generated solution filter (.slnf) for each
// platform/config, so solution configuration mappings and project references behave exactly as they do in Visual Studio.
public static class ParallelBuild
{
	// Builds every phase of 'plan' in order. Throws if any MSBuild invocation fails.
	public static void Run(string workspace, BuildPlan plan)
	{
		// Generated files and logs live in one place so agents and humans can find build diagnostics.
		var build_dir = Tools.Path([workspace, "obj", "build"], check_exists: false);
		Directory.CreateDirectory(build_dir);
		Tools.SetupVcEnvironment();

		// Phases run in order because each one consumes the outputs of the previous one.
		var total = Stopwatch.StartNew();
		foreach (var phase in Enum.GetValues<EPhase>())
		{
			var builds = plan.Builds(phase);
			if (builds.Count == 0)
				continue;

			RunPhase(workspace, build_dir, phase, builds);
		}
		Console.WriteLine($"Build complete in {total.Elapsed.TotalSeconds:F1}s. Logs: {build_dir}\n");
	}

	// Restores, then builds, all platform/config combinations of one phase.
	private static void RunPhase(string workspace, string build_dir, EPhase phase, IReadOnlyList<(string Platform, string Config, IReadOnlyCollection<string> Projects)> builds)
	{
		// Describe the work up front so long-running phases are easy to follow.
		var phase_name = phase.ToString().ToLowerInvariant();
		var solution = Tools.Path([workspace, "Rylogic.sln"]);
		var timer = Stopwatch.StartNew();
		Console.WriteLine($"=== {phase} phase: {string.Join(", ", builds.Select(x => $"{x.Platform}|{x.Config} ({x.Projects.Count} projects)"))} ===");

		// Write one solution filter per platform/config. A filter's Build target builds all of its projects in parallel.
		var filters = new List<(string Platform, string Config, string Filter)>();
		foreach (var (platform, config, projects) in builds)
		{
			var filter = IOPath.Combine(build_dir, $"{phase_name}-{SafeName(platform)}-{config}.slnf");
			WriteSolutionFilter(filter, solution, projects);
			filters.Add((platform, config, filter));
		}

		// Restore one configuration at a time because Debug and Release share each project's project.assets.json.
		if (BuildOptions.Restore)
		{
			foreach (var (platform, config, filter) in filters)
			{
				// Write a separate log for each restore so failures point at the right configuration.
				var log = IOPath.Combine(build_dir, $"{phase_name}-{SafeName(platform)}-{config}-restore");
				RunMSBuild(filter, ["-t:Restore", $"-p:Configuration={config}", $"-p:Platform={platform}"], log, $"{phase} restore {platform}|{config}");
			}
		}

		// Build every platform/config from one traversal project so a single MSBuild scheduler shares all cores across them.
		var traversal = IOPath.Combine(build_dir, $"{phase_name}.proj");
		WriteTraversalProject(traversal, filters);
		RunMSBuild(traversal, [], IOPath.Combine(build_dir, phase_name), $"{phase} build");
		Console.WriteLine($"=== {phase} phase complete in {timer.Elapsed.TotalSeconds:F1}s ===\n");
	}

	// Runs MSBuild on 'project' with the shared parallel options. 'log' is the log file path without an extension.
	private static void RunMSBuild(string project, IList<string> extra_args, string log, string description)
	{
		// MultiToolTask with a shared process limit stops each project's '/MP' compiler from multiplying the number of compiler processes.
		// Node reuse is off so that each run owns its worker nodes. Shared nodes can be shut down by other builds, and idle nodes keep output files locked.
		// The 64-bit compiler and linker avoid running out of address space when many large translation units build at once.
		var args = new List<string>
		{
			UserVars.MSBuild ?? throw new Exception("MSBuild path has not been set in UserVars"),
			project,
			"-m",
			"-nr:false",
			"-nologo",
			"-v:minimal",
			"-p:UseMultiToolTask=true",
			"-p:EnforceProcessCountAcrossBuilds=true",
			"-p:PreferredToolArchitecture=x64",
			$"-flp1:LogFile={log}.log;Verbosity=normal",
			$"-flp2:LogFile={log}.errors.log;ErrorsOnly",
		};
		if (!BuildOptions.RunUnitTests)
			args.Add("-p:RunUnitTests=false");
		if (!BuildOptions.GeneratePackageOnBuild)
			args.Add("-p:RylogicGeneratePackageOnBuild=false");

		// Profiling adds a binary log and per-target timing to the console summary.
		if (BuildOptions.Profile)
		{
			Directory.CreateDirectory(BuildOptions.ProfileDir);
			args.Add($"-bl:{IOPath.Combine(BuildOptions.ProfileDir, IOPath.GetFileName(log))}.binlog");
			args.Add("-clp:PerformanceSummary;Summary");
		}
		args.AddRange(extra_args);

		// Run MSBuild in this console so its live output is visible, then report the errors file on failure.
		var (success, _) = Tools.Run(args, throw_on_error: false, return_output: false);
		if (!success)
			throw new Exception($"{description} failed. Errors: {log}.errors.log  Full log: {log}.log");
	}

	// Writes a solution filter that selects 'projects' (full paths) from 'solution'.
	private static void WriteSolutionFilter(string filter, string solution, IReadOnlyCollection<string> projects)
	{
		// Solution filters use paths relative to the solution directory, and the solution path relative to the filter.
		var solution_dir = IOPath.GetDirectoryName(solution)!;
		var json = new System.Text.Json.Nodes.JsonObject
		{
			["solution"] = new System.Text.Json.Nodes.JsonObject
			{
				["path"] = IOPath.GetRelativePath(IOPath.GetDirectoryName(filter)!, solution),
				["projects"] = new System.Text.Json.Nodes.JsonArray(projects.Select(x => (System.Text.Json.Nodes.JsonNode?)IOPath.GetRelativePath(solution_dir, x)).ToArray()),
			},
		};
		WriteIfChanged(filter, json.ToJsonString(new System.Text.Json.JsonSerializerOptions { WriteIndented = true }));
	}

	// Writes a traversal project whose Build target builds every solution filter in parallel.
	private static void WriteTraversalProject(string traversal, IReadOnlyList<(string Platform, string Config, string Filter)> filters)
	{
		// Each filter becomes an item with its own Configuration and Platform, and the MSBuild task schedules all items together.
		var items = filters.Select(x => new XElement("SolutionFilter",
			new XAttribute("Include", x.Filter),
			new XAttribute("AdditionalProperties", $"Configuration={x.Config};Platform={x.Platform}")));
		var xml = new XDocument(
			new XElement("Project",
				new XElement("ItemGroup", items),
				new XElement("Target", new XAttribute("Name", "Build"),
					new XElement("MSBuild",
						new XAttribute("Projects", "@(SolutionFilter)"),
						new XAttribute("Targets", "Build"),
						new XAttribute("BuildInParallel", "true"),
						new XAttribute("StopOnFirstFailure", "false")))));
		WriteIfChanged(traversal, xml.ToString());
	}

	// Writes 'text' to 'path' only if the content differs, so unchanged generated files keep their timestamps.
	private static void WriteIfChanged(string path, string text)
	{
		// Skip the write when the file already holds the same content.
		if (File.Exists(path) && File.ReadAllText(path) == text)
			return;

		File.WriteAllText(path, text);
	}

	// Returns 'value' with characters that are not valid in file names replaced, e.g. "Any CPU" -> "Any_CPU".
	private static string SafeName(string value)
	{
		// Spaces are also replaced so generated paths never need quoting.
		foreach (var ch in IOPath.GetInvalidFileNameChars().Append(' '))
			value = value.Replace(ch, '_');

		return value;
	}
}

// Groups of projects
public abstract class Group : Common
{
	public List<Common> Items = [];

	public Group(string workspace)
		: base(workspace)
	{ }
	public override void Clean()
	{
		foreach (var item in Items)
			item.Clean();
	}
	public override void Build(BuildPlan plan)
	{
		// Each member adds its own projects; the plan removes duplicates.
		foreach (var item in Items)
			item.Build(plan);
	}
	public override void Deploy()
	{
		foreach (var item in Items)
			item.Deploy();
	}
	public override void Publish()
	{
		foreach (var item in Items)
			item.Publish();
	}
}

// Base class for native projects
public abstract class Native : Common
{
	public string ProjName;
	public string ProjDir;
	public string ProjFile;
	public string ObjDir;
	public IList<string> Platforms;
	public IList<string> Configs;

	public Native(string proj_name, string proj_dir, string workspace, List<string>? platforms, List<string>? configs)
		:base(workspace)
	{
		ProjName = proj_name;
		ProjDir = Tools.Path([workspace, proj_dir]);
		ProjFile = Tools.Path([ProjDir, $"{proj_name}.vcxproj"]);
		Platforms = platforms ?? ["x64"]; // "x86"
		Configs = configs ?? ["Release", "Debug"];
		ObjDir = Tools.Path([Workspace, "obj"], check_exists: false);
		Directory.CreateDirectory(ObjDir);
	}

	// Adds this builder's project file to 'plan'. Builders with more than one project override this.
	public override void Build(BuildPlan plan)
	{
		// Most native builders own exactly one project.
		plan.AddNative(Platforms, Configs, ProjFile);
	}

	// Returns the full path of the project file 'name'.vcxproj in this builder's project directory.
	protected string SiblingProject(string name)
	{
		// Library builders keep their static library and DLL projects side by side.
		return Tools.Path([ProjDir, $"{name}.vcxproj"]);
	}
}

// Base class for .NET projects
public abstract class Managed : Common
{
	public string ProjName;
	public string ProjDir;
	public string ProjFile;
	public IList<string> Frameworks;
	public IList<string> Configs;

	public Managed(string proj_name, string proj_dir, IList<string> frameworks, string workspace, IList<string>? configs)
		:base(workspace)
	{
		ProjName = proj_name;
		ProjDir = Tools.Path([workspace, proj_dir]);
		ProjFile = Tools.Path([ProjDir, $"{ProjName}.csproj"]);
		Frameworks = frameworks;
		Configs = configs ?? ["Release", "Debug"];
	}
	public override void Clean()
	{
		CleanDotNet(ProjDir);
	}

	// Adds this builder's project file to 'plan'. Builders with more than one project override this.
	public override void Build(BuildPlan plan)
	{
		// Most managed builders own exactly one project.
		plan.AddManaged(Configs, ProjFile);
	}

	// Clean the 'bin' and 'obj' directory of a dot net project
	public static void CleanDotNet(string proj_dir)
	{
		Tools.CleanDir(Tools.Path([proj_dir, "obj"], check_exists: false));
		Tools.CleanDir(Tools.Path([proj_dir, "bin"], check_exists: false));
	}
}

// Audio
public class Audio : Native
{
	public Audio(string workspace, List<string>? platforms, List<string>? configs)
		: base("audio", Tools.Path([workspace, $"projects\\rylogic\\audio"]), workspace, platforms, configs)
	{
	}
	public override void Clean()
	{
		Tools.CleanDir(Tools.Path([ObjDir, "audio"], check_exists: false));
		Tools.CleanDir(Tools.Path([ObjDir, "audio.dll"], check_exists: false));
	}
	public override void Build(BuildPlan plan)
	{
		// The static library and the DLL are separate projects.
		plan.AddNative(Platforms, Configs, SiblingProject("audio"), SiblingProject("audio.dll"));
	}
	public override void Deploy()
	{
		// To publish the nuget package, use AllNative`
		foreach (var p in Platforms)
		{
			foreach (var c in Configs)
			{
				Tools.DeployLib(Tools.Path([ObjDir, "audio", p, c, "audio-static.lib"]));
				Tools.DeployLib(Tools.Path([ObjDir, "audio.dll", p, c, "audio.dll"]));
			}
		}
	}
}

// Compute
public class Compute : Native
{
	public Compute(string workspace, List<string>? platforms, List<string>? configs)
		: base("compute", Tools.Path([workspace, $"projects\\rylogic\\compute"]), workspace, platforms, configs)
	{
	}
	public override void Clean()
	{
		Tools.CleanDir(Tools.Path([ObjDir, "compute"], check_exists: false));
	}
	public override void Deploy()
	{
		foreach (var p in Platforms)
		{
			foreach (var c in Configs)
			{
				Tools.DeployLib(Tools.Path([ObjDir, "compute", p, c, "compute-static.lib"]));
			}
		}
	}
}

// Physics
public class Physics : Native
{
	public Physics(string workspace, List<string>? platforms = null, List<string>? configs = null)
		: base("physics", Tools.Path([workspace, $"projects\\rylogic\\physics"]), workspace, platforms, configs)
	{
	}
	public override void Clean()
	{
		Tools.CleanDir(Tools.Path([ObjDir, "physics"], check_exists: false));
		Tools.CleanDir(Tools.Path([ObjDir, "physics.dll"], check_exists: false));
	}
	public override void Build(BuildPlan plan)
	{
		// The static library and the DLL are separate projects.
		plan.AddNative(Platforms, Configs, SiblingProject("physics"), SiblingProject("physics.dll"));
	}
	public override void Deploy()
	{
		// All configurations are retained locally; Rylogic.Native packages canonical Release runtime assets.
		foreach (var p in Platforms)
		{
			foreach (var c in Configs)
			{
				Tools.DeployLib(Tools.Path([ObjDir, "physics", p, c, "physics-static.lib"]));
				Tools.DeployLib(Tools.Path([ObjDir, "physics.dll", p, c, "physics.dll"]));
			}
		}
	}
}

// View3d
public class View3d : Native
{
	public View3d(string workspace, List<string>? platforms, List<string>? configs)
		: base("view3d-12", Tools.Path([workspace, $"projects\\rylogic\\view3d-12"]), workspace, platforms, configs)
	{
	}
	public override void Clean()
	{
		Tools.CleanDir(Tools.Path([ObjDir, "view3d-12"], check_exists: false));
		Tools.CleanDir(Tools.Path([ObjDir, "view3d-12.dll"], check_exists: false));
	}
	public override void Build(BuildPlan plan)
	{
		// The static library and the DLL are separate projects.
		plan.AddNative(Platforms, Configs, SiblingProject("view3d-12"), SiblingProject("view3d-12.dll"));
	}
	public override void Deploy()
	{
		// To publish the nuget package, use AllNative`
		foreach (var p in Platforms)
		{
			foreach (var c in Configs)
			{
				Tools.DeployLib(Tools.Path([ObjDir, "view3d-12", p, c, "view3d-12-static.lib"]));
				Tools.DeployLib(Tools.Path([ObjDir, "view3d-12.dll", p, c, "view3d-12.dll"]));
			}
		}
	}
}

// View3dUI
public class View3dUI : Native
{
	public View3dUI(string workspace, List<string>? platforms, List<string>? configs)
		: base("view3d-ui", Tools.Path([workspace, $"projects\\rylogic\\view3d-ui"]), workspace, platforms, configs)
	{
	}
	public override void Clean()
	{
		Tools.CleanDir(Tools.Path([ObjDir, "view3d-ui"], check_exists: false));
		Tools.CleanDir(Tools.Path([ObjDir, "view3d-ui.dll"], check_exists: false));
	}
	public override void Build(BuildPlan plan)
	{
		// The static library and the DLL are separate projects.
		plan.AddNative(Platforms, Configs, SiblingProject("view3d-ui"), SiblingProject("view3d-ui.dll"));
	}
	public override void Deploy()
	{
		foreach (var p in Platforms)
		{
			foreach (var c in Configs)
			{
				Tools.DeployLib(Tools.Path([ObjDir, "view3d-ui", p, c, "view3d-ui-static.lib"]));
				Tools.DeployLib(Tools.Path([ObjDir, "view3d-ui.dll", p, c, "view3d-ui.dll"]));
			}
		}
	}
}

// Fbx
public class Fbx : Native
{
	public Fbx(string workspace, List<string>? platforms = null, List<string>? configs = null)
		: base("fbx", Tools.Path([workspace, $"projects\\rylogic\\fbx"]), workspace, platforms, configs)
	{
	}
	public override void Clean()
	{
		Tools.CleanDir(Tools.Path([ObjDir, "fbx"], check_exists: false));
	}
	public override void Deploy()
	{
		foreach (var p in Platforms)
		{
			foreach (var c in Configs)
			{
				Tools.DeployLib(Tools.Path([ObjDir, "fbx", p, c, "fbx.dll"]));
			}
		}
	}
}

// Builds the dynamically loaded glTF scene importer.
public class Gltf : Native
{
	public Gltf(string workspace, List<string>? platforms = null, List<string>? configs = null)
		: base("gltf", Tools.Path([workspace, $"projects\\rylogic\\gltf"]), workspace, platforms, configs)
	{
	}
	public override void Clean()
	{
		Tools.CleanDir(Tools.Path([ObjDir, "gltf"], check_exists: false));
	}
	public override void Deploy()
	{
		foreach (var p in Platforms)
		{
			foreach (var c in Configs)
				Tools.DeployLib(Tools.Path([ObjDir, "gltf", p, c, "gltf.dll"]));
		}
	}
}

// Builds the dynamically loaded Dear ImGui integration used by View3D diagnostics.
public class Imgui : Native
{
	public Imgui(string workspace, List<string>? platforms = null, List<string>? configs = null)
		: base("imgui", Tools.Path([workspace, $"projects\\rylogic\\imgui"]), workspace, platforms, configs)
	{
	}
	public override void Clean()
	{
		Tools.CleanDir(Tools.Path([ObjDir, "imgui"], check_exists: false));
	}
	public override void Deploy()
	{
		foreach (var p in Platforms)
		{
			foreach (var c in Configs)
				Tools.DeployLib(Tools.Path([ObjDir, "imgui", p, c, "imgui.dll"]));
		}
	}
}

// Builds the native SQLite runtime consumed by Rylogic.DB.
public class Sqlite3 : Native
{
	public Sqlite3(string workspace, List<string>? platforms = null, List<string>? configs = null)
		: base("sqlite", Tools.Path([workspace, $"sdk\\sqlite3"]), workspace, platforms, configs)
	{
	}
	public override void Clean()
	{
		Tools.CleanDir(Tools.Path([ObjDir, "sqlite"], check_exists: false));
	}
	public override void Deploy()
	{
		foreach (var p in Platforms)
		{
			foreach (var c in Configs)
				Tools.DeployLib(Tools.Path([ObjDir, "sqlite", p, c, "sqlite3.dll"]));
		}
	}
}

// Builds the native editor control consumed by Rylogic.Scintilla.
public class Scintilla : Native
{
	public Scintilla(string workspace, List<string>? platforms = null, List<string>? configs = null)
		: base("scintilla", Tools.Path([workspace, $"sdk\\scintilla"]), workspace, platforms, configs)
	{
	}
	public override void Clean()
	{
		Tools.CleanDir(Tools.Path([ObjDir, "scintilla"], check_exists: false));
	}
	public override void Deploy()
	{
		foreach (var p in Platforms)
		{
			foreach (var c in Configs)
				Tools.DeployLib(Tools.Path([ObjDir, "scintilla", p, c, "scintilla.dll"]));
		}
	}
}

// Builds the p3d model conversion tool used to cook art into the P3D runtime format. It is a command
// line application rather than a runtime library, so it is delivered in the native package's tools
// folder instead of the runtime DLL closure.
public class P3d : Native
{
	public P3d(string workspace, List<string>? platforms = null, List<string>? configs = null)
		: base("p3d", Tools.Path([workspace, $"projects\\tools\\p3d"]), workspace, platforms, configs)
	{
	}

	// Applications use the project-local output convention rather than the shared 'obj' tree that libraries use.
	public string ExePath(string platform, string config)
	{
		return NativeRuntimePackage.PackagedToolPath(Workspace, ProjName, platform, config);
	}
	public override void Clean()
	{
		foreach (var p in Platforms)
		{
			foreach (var c in Configs)
				Tools.CleanDir(Tools.Path([ProjDir, "obj", p, c], check_exists: false));
		}
	}
	public override void Deploy()
	{
		// The tool loads 'gltf.dll' from its own directory at run time, so it is deployed alongside the
		// native libraries where that dependency has already been placed.
		foreach (var p in Platforms)
		{
			foreach (var c in Configs)
			{
				var src = ExePath(p, c);
				if (!File.Exists(src))
					throw new FileNotFoundException($"p3d tool was not built for {p}|{c}", src);

				var dst_dir = Tools.Path([UserVars.Root, "lib", p, c], check_exists: false);
				Directory.CreateDirectory(dst_dir);
				File.Copy(src, Tools.Path([dst_dir, "p3d.exe"], check_exists: false), overwrite: true);
			}
		}
	}
}

// Rylogic .NET assemblies
public abstract class RylogicAssembly : Managed
{
	public Nuget? Package = null;

	public RylogicAssembly(string proj_name, List<string> frameworks, string workspace, List<string>? platforms, List<string>? configs)
		:base(proj_name, Tools.Path([workspace, "projects\\rylogic", proj_name]), frameworks, workspace, configs)
	{
	}
	public override void Deploy()
	{
		Package = new Nuget()
		{
			PackageName = IOPath.GetFileNameWithoutExtension(ProjFile),
			Version = RylogicLibraryVersion,
			Tags = "rylogic csharp library",
		};
		Populate(Package);
		Package.Package();
	}
	public override void Publish()
	{
		if (Package is null) throw new Exception("Call Deploy before calling Publish");
		Package.Publish();
	}
	protected virtual void Populate(Nuget package)
	{
		foreach (var fw in Frameworks)
		{
			package.Files.Add(new Nuget.File(Tools.Path([ProjDir, $"bin\\Release\\{fw}\\{ProjName}.dll"]), $"lib/{FwToTarget(fw)}/", true));
			package.Files.Add(new Nuget.File(Tools.Path([ProjDir, $"bin\\Release\\{fw}\\{ProjName}.pdb"]), $"lib/{FwToTarget(fw)}/"));
		}
	}

	// Return the explicit NuGet TFM for the project's minimum supported Windows version.
	protected static string FwToTarget(string fw)
	{
		return fw.EndsWith("windows", StringComparison.Ordinal) ? $"{fw}7.0" : fw;
	}
}
public class RylogicAudio : RylogicAssembly
{
	public RylogicAudio(string workspace, List<string>? platforms = null, List<string>? configs = null)
		:base("Rylogic.Audio", ["net10.0-windows", "net481"], workspace, platforms, configs)
	{}
	protected override void Populate(Nuget package)
	{
		base.Populate(package);
		package.Description = "Managed ownership, playback, event, and diagnostics API for the Rylogic spatial audio engine.";
		package.Tags += " audio spatial-audio xaudio2 x3daudio ogg sound";
		package.Deps.Add(new Nuget.Dep("Rylogic.Core", $"[{RylogicLibraryVersion},)"));
		package.Deps.Add(new Nuget.Dep("Rylogic.Native", $"[{RylogicLibraryVersion},)"));
		package.Deps.Add(new Nuget.Dep("System.Memory", "[4.6.0,)", "net481"));
	}
}
public class RylogicCore : RylogicAssembly
{
	public RylogicCore(string workspace, List<string>? platforms = null, List<string>? configs = null)
		:base("Rylogic.Core", ["net10.0", "net10.0-windows", "net481"], workspace, platforms, configs)
	{}
}
public class RylogicD3D12 : RylogicAssembly
{
	public RylogicD3D12(string workspace, List<string>? platforms = null, List<string>? configs = null)
		:base("Rylogic.D3D12", ["net10.0-windows", "net481"], workspace, platforms, configs)
	{}
	protected override void Populate(Nuget package)
	{
		base.Populate(package);
		package.Description = "Lifetime-safe managed COM leases for sharing Direct3D 12 devices between independent Rylogic packages.";
		package.Tags += " d3d12 directx graphics interop";
	}
}
public class RylogicDB : RylogicAssembly
{
	public RylogicDB(string workspace, List<string>? platforms = null, List<string>? configs = null)
		:base("Rylogic.DB", ["net10.0-windows", "net481"], workspace, platforms, configs)
	{}
	protected override void Populate(Nuget package)
	{
		base.Populate(package);
		package.Deps.Add(new Nuget.Dep("Rylogic.Core", $"[{RylogicLibraryVersion},)"));
	}
}
public class RylogicDirectShow : RylogicAssembly
{
	public RylogicDirectShow(string workspace, List<string>? platforms = null, List<string>? configs = null)
		:base("Rylogic.DirectShow", ["net10.0-windows", "net481"], workspace, platforms, configs)
	{}
	protected override void Populate(Nuget package)
	{
		base.Populate(package);
		package.Deps.Add(new Nuget.Dep("Rylogic.Core", $"[{RylogicLibraryVersion},)"));
	}
}
public class RylogicGfx : RylogicAssembly
{
	public RylogicGfx(string workspace, List<string>? platforms = null, List<string>? configs = null)
		:base("Rylogic.Gfx", ["net10.0-windows", "net481"], workspace, platforms, configs)
	{}
	protected override void Populate(Nuget package)
	{
		base.Populate(package);
		package.Tags += " view3d";
		package.Deps.Add(new Nuget.Dep("Rylogic.Core", $"[{RylogicLibraryVersion},)"));
		package.Deps.Add(new Nuget.Dep("Rylogic.D3D12", $"[{RylogicLibraryVersion},)"));
		package.Deps.Add(new Nuget.Dep("Rylogic.Native", $"[{RylogicLibraryVersion},)"));
		package.Deps.Add(new Nuget.Dep("System.Drawing.Common", "[9.0.0,)", "net10.0-windows7.0"));
	}
}
public class RylogicGfxWPF : RylogicAssembly
{
	public RylogicGfxWPF(string workspace, List<string>? platforms = null, List<string>? configs = null)
		:base("Rylogic.Gfx.WPF", ["net10.0-windows", "net481"], workspace, platforms, configs)
	{}
	protected override void Populate(Nuget package)
	{
		base.Populate(package);
		package.Tags += " view3d wpf gui chart";
		package.Deps.Add(new Nuget.Dep("Rylogic.Gfx", $"[{RylogicLibraryVersion},)"));
		package.Deps.Add(new Nuget.Dep("Rylogic.Gui.WPF", $"[{RylogicLibraryVersion},)"));
		package.FrameworkRefs.Add(new Nuget.FrameworkRef("Microsoft.WindowsDesktop.App.WPF", "net10.0-windows7.0"));
	}
}
public class RylogicGuiWPF : RylogicAssembly
{
	public RylogicGuiWPF(string workspace, List<string>? platforms = null, List<string>? configs = null)
		:base("Rylogic.Gui.WPF", ["net10.0-windows", "net481"], workspace, platforms, configs)
	{}
	protected override void Populate(Nuget package)
	{
		base.Populate(package);
		package.Tags += " wpf gui";
		package.Deps.Add(new Nuget.Dep("Rylogic.Core", $"[{RylogicLibraryVersion},)"));
		package.Deps.Add(new Nuget.Dep("Rylogic.Windows", $"[{RylogicLibraryVersion},)"));
	}
}
public class RylogicNet : RylogicAssembly
{
	public RylogicNet(string workspace, List<string>? platforms = null, List<string>? configs = null)
		:base("Rylogic.Net", ["net10.0-windows", "net481"], workspace, platforms, configs)
	{}
	protected override void Populate(Nuget package)
	{
		base.Populate(package);
		package.Deps.Add(new Nuget.Dep("Rylogic.Core", $"[{RylogicLibraryVersion},)"));
	}
}
public class RylogicPhysics : RylogicAssembly
{
	public RylogicPhysics(string workspace, List<string>? platforms = null, List<string>? configs = null)
		:base("Rylogic.Physics", ["net10.0-windows", "net481"], workspace, platforms, configs)
	{}
	protected override void Populate(Nuget package)
	{
		base.Populate(package);
		package.Description = "Managed ownership, bulk stepping, snapshots, events, checkpoints, and diagnostics for the Rylogic D3D12 rigid-body physics engine.";
		package.Tags += " physics simulation rigid-body d3d12";
		package.Deps.Add(new Nuget.Dep("Rylogic.Core", $"[{RylogicLibraryVersion},)"));
		package.Deps.Add(new Nuget.Dep("Rylogic.D3D12", $"[{RylogicLibraryVersion},)"));
		package.Deps.Add(new Nuget.Dep("Rylogic.Native", $"[{RylogicLibraryVersion},)"));
		package.Deps.Add(new Nuget.Dep("System.Memory", "[4.6.0,)", "net481"));
	}
}
public class RylogicScintilla : RylogicAssembly
{
	public RylogicScintilla(string workspace, List<string>? platforms = null, List<string>? configs = null)
		:base("Rylogic.Scintilla", ["net10.0-windows", "net481"], workspace, platforms, configs)
	{}
	protected override void Populate(Nuget package)
	{
		base.Populate(package);
		package.Tags += " scintilla";
		package.Deps.Add(new Nuget.Dep("Rylogic.Core", $"[{RylogicLibraryVersion},)"));
		package.Deps.Add(new Nuget.Dep("Rylogic.Gui.WPF", $"[{RylogicLibraryVersion},)"));
		package.Deps.Add(new Nuget.Dep("Rylogic.Windows", $"[{RylogicLibraryVersion},)"));
	}
}
public class RylogicWindows : RylogicAssembly
{
	public RylogicWindows(string workspace, List<string>? platforms = null, List<string>? configs = null)
		:base("Rylogic.Windows", ["net10.0-windows", "net481"], workspace, platforms, configs)
	{}
	protected override void Populate(Nuget package)
	{
		base.Populate(package);
		package.Tags += " windows";
		package.Deps.Add(new Nuget.Dep("Rylogic.Core", $"[{RylogicLibraryVersion},)"));
	}
}
// Builds the complete public Rylogic package set and rejects unclassified managed assemblies.
public class AllRylogic : Group
{
	private readonly AllNative m_native;
	private readonly IReadOnlyList<RylogicAssembly> m_assemblies;
	private readonly IReadOnlyList<string> m_platforms;
	private readonly IReadOnlyList<string> m_configs;

	public AllRylogic(string workspace, List<string>? platforms = null, List<string>? configs = null)
		: base(workspace)
	{
		m_platforms = platforms ?? ["x64"];
		m_configs = configs ?? ["Release", "Debug"];
		m_native = new AllNative(workspace, platforms, configs);
		Items.Add(m_native);

		// Discover package builders so newly registered public assemblies join the aggregate release automatically.
		var assemblies = new List<RylogicAssembly>();
		foreach (var type in typeof(RylogicAssembly).Assembly.GetTypes().Where(t => t.IsClass && !t.IsAbstract && t.IsSubclassOf(typeof(RylogicAssembly))))
		{
			var instance = (RylogicAssembly?)Activator.CreateInstance(type, workspace, platforms, configs) ?? throw new Exception($"Failed to create instance of type {type}");
			assemblies.Add(instance);
			Items.Add(instance);
		}
		m_assemblies = assemblies;

		// Reject missing or contradictory package declarations before any build or deployment work begins.
		ValidatePackageCoverage();
	}

	// Packages the complete Release set and writes the validated manifest consumed by the publishing workflow.
	public override void Deploy()
	{
		base.Deploy();
		if (!m_platforms.Contains("x64", StringComparer.OrdinalIgnoreCase) || !m_configs.Contains("Release", StringComparer.OrdinalIgnoreCase))
			return;

		// Validate the exact aggregate package set only after every constituent deployment has completed.
		var packages = new List<Nuget>
		{
			m_native.Package ?? throw new Exception("Rylogic.Native was not packaged by the Release deployment"),
		};
		packages.AddRange(m_assemblies.Select(x => x.Package ?? throw new Exception($"{x.ProjName} was not packaged by the Release deployment")));
		Nuget.ValidateReleasePackageSet(
			RylogicLibraryVersion,
			packages,
			Tools.Path([Workspace, "lib\\packages\\release\\manifest.json"], check_exists: false));
	}

	// Requires every canonical managed assembly to be either represented by a package builder or explicitly marked non-packable.
	private void ValidatePackageCoverage()
	{
		var projects_root = Tools.Path([Workspace, "projects\\rylogic"]);
		var projects = Directory.EnumerateDirectories(projects_root)
			.Select(dir =>
			{
				var project_name = IOPath.GetFileName(dir) ?? throw new Exception($"Unable to determine the project name for '{dir}'");
				return (ProjectName: project_name, ProjectPath: IOPath.Combine(dir, $"{project_name}.csproj"));
			})
			.Where(x => File.Exists(x.ProjectPath))
			.ToDictionary(x => x.ProjectName, x => x.ProjectPath, StringComparer.OrdinalIgnoreCase);

		// Keep one authoritative builder per public package ID.
		foreach (var duplicate in m_assemblies.GroupBy(x => x.ProjName, StringComparer.OrdinalIgnoreCase).Where(x => x.Count() != 1))
			throw new Exception($"Multiple Rylogic package builders target '{duplicate.Key}'");

		// Prevent stale builders from silently targeting renamed or removed projects.
		var builders = m_assemblies.ToDictionary(x => x.ProjName, StringComparer.OrdinalIgnoreCase);
		foreach (var builder in builders.Keys.Except(projects.Keys, StringComparer.OrdinalIgnoreCase))
			throw new Exception($"Rylogic package builder '{builder}' has no canonical project at projects\\rylogic\\{builder}\\{builder}.csproj");

		// Require every canonical assembly to make its public-package intent explicit.
		foreach (var (project_name, project_path) in projects.OrderBy(x => x.Key, StringComparer.OrdinalIgnoreCase))
		{
			var project = XDocument.Load(project_path);
			var is_packable = project.Descendants()
				.Where(x => x.Name.LocalName == "IsPackable")
				.Select(x => x.Value.Trim())
				.LastOrDefault();

			// Builder presence and IsPackable must agree so either source of truth can expose configuration drift.
			if (builders.ContainsKey(project_name))
			{
				if (!string.Equals(is_packable, "true", StringComparison.OrdinalIgnoreCase))
					throw new Exception($"Public package project '{project_name}' must explicitly set <IsPackable>true</IsPackable>");
			}
			else if (!string.Equals(is_packable, "false", StringComparison.OrdinalIgnoreCase))
			{
				throw new Exception($"Managed assembly '{project_name}' is not classified for release. Add a RylogicAssembly builder or explicitly set <IsPackable>false</IsPackable>");
			}
		}
	}
}

// LDraw
public class LDraw : Managed
{
	public string DeployDir = string.Empty;
	public string? MsiPath = null;

	public LDraw(string workspace, List<string>? platforms = null, List<string>? configs = null)
		:base("LDraw", Tools.Path([workspace, "projects\\apps\\LDraw\\LDraw"]), ["net10.0-windows"], workspace, configs)
	{
		DeployDir = Tools.Path([UserVars.Root, "bin/LDraw"], check_exists: false);
	}

	public override void Build(BuildPlan plan)
	{
		// The MCP tray host is deployed alongside LDraw, so both are built together.
		plan.AddManaged(Configs, ProjFile, Tools.Path([ProjDir, "..", "LDrawMcpHost", "LDrawMcpHost.csproj"]));
	}

	public override void Deploy()
	{
		// Check versions
		var proj_file = Tools.Path([ProjDir, "LDraw.csproj"]);
		var version = Tools.Extract(proj_file, new Regex("<Version>(.*)</Version>")).Groups[1].Value;
		Console.WriteLine($"Deploying LDraw Version: {version}\n");

		// Ensure output directories exist and are empty
		Tools.CleanDir(DeployDir);

		// Declare the complete application payload separately from generated or harvested installer content.
		var file_list = (List<string>)[
			"LDraw.exe",
			"dxcompiler.dll",
			"dxil.dll",
			"ICSharpCode.AvalonEdit.dll",
			"LDraw.dll",
			"Rylogic.Core.dll",
			"Rylogic.Gfx.dll",
			"Rylogic.Gfx.WPF.dll",
			"Rylogic.Gui.WPF.dll",
			"Rylogic.Windows.dll",
			"LDraw.runtimeconfig.json",
		];
		var dir_list = (List<string>)[
			"lib",
		];

		// Copy build products to the output directory
		Console.WriteLine($"Deploying files to '{DeployDir}\\':\n");
		var target_dir = Tools.Path([ProjDir, "bin/Release", Frameworks[0]]);
		foreach (var file in file_list)
			Tools.Copy(Tools.Path([target_dir, file]), DeployDir, full_paths: false, indent: "    ");
		foreach (var dir in dir_list)
			Tools.Copy(Tools.Path([target_dir, dir]), Tools.Path([DeployDir, dir], check_exists: false), full_paths: false, indent: "    ");

		// Bundle the MCP tray host alongside LDraw so one install folder contains both exes. The host auto-launches its
		// sibling LDraw.exe, so they must be co-located. Copy the host's top-level build output into the harvest source
		// (target_dir, scanned by heat for *.dll + the runtimeconfig/deps regexes) and into the plain deploy dir for parity.
		// Shared assemblies (Rylogic.Core.dll etc.) are byte-identical here because both projects are produced by the
		// single solution build above, so overwriting LDraw's copies in target_dir is safe.
		var host_target_dir = Tools.Path([ProjDir, "..", "LDrawMcpHost", "bin/Release", Frameworks[0]]);
		Console.WriteLine($"Bundling MCP host from '{host_target_dir}\\':\n");
		foreach (var file in System.IO.Directory.GetFiles(host_target_dir))
		{
			Tools.Copy(file, target_dir, full_paths: false, indent: "    ");
			Tools.Copy(file, DeployDir, full_paths: false, indent: "    ");
		}

		// Build the installer (installer.wxs lives at the umbrella folder, one level above ProjDir)
		Console.WriteLine("Building installer...\n");
		var installer_wxs = Tools.Path([ProjDir, "..", "installer", "installer.wxs"]);
		MsiPath = BuildInstaller.Build("LDraw", version, installer_wxs, ProjDir, target_dir, Tools.Path([DeployDir, ".."]),
			[
				new HarvestPath("binaries", "INSTALLFOLDER", ".", false, [new Regex(@".*\.dll"), new Regex(@"LDraw\.runtimeconfig\.json"), new Regex(@"LDrawMcpHost\.runtimeconfig\.json"), new Regex(@"LDrawMcpHost\.deps\.json")]),
				new HarvestPath("lib_files", "lib", "lib", true),
			]);
		Console.WriteLine($"{MsiPath} created.\n");
	}

	public override void Publish()
	{
		//if (MsiPath is null)
		//	Deploy();

		//Console.WriteLine("\nPublishing to web site...");
		//Tools.Copy(MsiPath, Tools.Path([UserVars.WWWRoot, "files/ldraw", check_exists=False))
	}
}

// Builds the Csex command line tool. Build only; it has no deploy step.
public class Csex : Managed
{
	public Csex(string workspace, List<string>? platforms = null, List<string>? configs = null)
		:base("Csex", Tools.Path([workspace, "projects\\tools\\Csex"]), ["net10.0-windows"], workspace, configs)
	{
	}
}

// Builds the RyLogViewer application. Build only; it has no deploy step.
public class RyLogViewer : Managed
{
	public RyLogViewer(string workspace, List<string>? platforms = null, List<string>? configs = null)
		:base("RyLogViewer", Tools.Path([workspace, "projects\\apps\\RyLogViewer"]), ["net10.0-windows"], workspace, configs)
	{
	}
}

// Builds the Rylogic.TextAligner Visual Studio extensions. Build only; it has no deploy step.
public class RylogicTextAligner : Managed
{
	public RylogicTextAligner(string workspace, List<string>? platforms = null, List<string>? configs = null)
		:base("Rylogic.TextAligner.Shared", Tools.Path([workspace, "projects\\apps\\Rylogic.TextAligner\\Shared"]), ["net481"], workspace, configs)
	{
	}
	public override void Clean()
	{
		// Each Visual Studio version has its own project next to the shared one.
		foreach (var version in VersionProjects)
			CleanDotNet(IOPath.GetDirectoryName(version)!);
	}
	public override void Build(BuildPlan plan)
	{
		// Build the shared library and every Visual Studio version-specific extension.
		plan.AddManaged(Configs, [.. VersionProjects]);
	}

	// The shared project and the extension projects for each supported Visual Studio version.
	private IEnumerable<string> VersionProjects
	{
		get
		{
			// The version folders sit next to the 'Shared' folder.
			var root = IOPath.GetDirectoryName(ProjDir)!;
			foreach (var version in new[] { "Shared", "2022", "2019" })
				yield return Tools.Path([root, version, $"Rylogic.TextAligner.{version}.csproj"]);
		}
	}
}

// All managed projects
public class AllManaged : Group
{
	public AllManaged(string workspace, List<string>? platforms = null, List<string>? configs = null)
		: base(workspace)
	{
		// Discover managed builders so new ones join this group automatically.
		foreach (var type in typeof(Managed).Assembly.GetTypes().Where(t => t.IsClass && !t.IsAbstract && t.IsSubclassOf(typeof(Managed))))
		{
			var instance = (Common?)Activator.CreateInstance(type, workspace, platforms, configs) ?? throw new Exception($"Failed to create instance of type {type}");
			Items.Add(instance);
		}
	}
}

// All Native projects
public class AllNative : Group
{
	private readonly IReadOnlyList<string> m_platforms;
	private readonly IReadOnlyList<string> m_configs;

	public Nuget? Package { get; private set; }

	public AllNative(string workspace, List<string>? platforms = null, List<string>? configs = null)
		: base(workspace)
	{
		m_platforms = platforms ?? ["x64"];
		m_configs = configs ?? ["Release", "Debug"];
		foreach (var type in typeof(Native).Assembly.GetTypes().Where(t => t.IsClass && !t.IsAbstract && t.IsSubclassOf(typeof(Native))))
		{
			var instance = (Common?)Activator.CreateInstance(type, workspace, platforms, configs) ?? throw new Exception($"Failed to create instance of type {type}");
			Items.Add(instance);
		}
	}
	public override void Build(BuildPlan plan)
	{
		// A complete aggregate build must regenerate every included project's runtime declaration.
		NativeRuntimePackage.ClearManifests(Workspace, m_platforms, m_configs);
		base.Build(plan);

		// Publish the local Rylogic.Native development package once per build. The project references every packaged native project,
		// so MSBuild builds it after them. See docs/local-dev-packages.md.
		if (BuildOptions.GeneratePackageOnBuild && m_platforms.Contains("x64", StringComparer.OrdinalIgnoreCase) && m_configs.Contains("Debug", StringComparer.OrdinalIgnoreCase))
			plan.AddNative(["x64"], ["Debug"], Tools.Path([Workspace, "build", "nuget", "Rylogic.Native.Dev.vcxproj"]));
	}
	public override void Deploy()
	{
		base.Deploy();
		if (!m_platforms.Contains("x64", StringComparer.OrdinalIgnoreCase) || !m_configs.Contains("Release", StringComparer.OrdinalIgnoreCase))
			return;

		// Stable native packages use Release runtime DLLs but must support both native consumer ABIs.
		if (!m_configs.Contains("Debug", StringComparer.OrdinalIgnoreCase))
			throw new InvalidOperationException("Rylogic.Native stable packages require both x64 Debug and Release builds.");

		var staging_dir = NativeRuntimePackage.Stage(
			Workspace,
			"x64",
			"Release",
			Tools.Path([UserVars.Root, "obj\\nuget\\Rylogic.Native\\x64\\Release\\package"], check_exists: false),
			require_all_projects: true,
			link_configs: ["Debug", "Release"]);

		// Stage the native package in the release feed so the canonical root package and cache can be refreshed exactly.
		Package = new Nuget()
		{
			PackageName = "Rylogic.Native",
			Version = RylogicLibraryVersion,
			Description = "Native runtime assets, public headers, supported native link libraries, and command line cooking tools for Rylogic View3D, database, editor, rigid-body physics, and spatial audio packages.",
			Tags = "rylogic native library view3d physics d3d12 audio tools",
			PackageOutputPath = Tools.Path([UserVars.Root, "lib\\packages\\release"], check_exists: false),
			ValidateStagedPackage = package_path => NativeRuntimePackage.ValidatePackage(package_path, staging_dir),
		};
		Package.Files.AddRange([
			new Nuget.File(Tools.Path([staging_dir, "build\\**\\*"], check_exists: false), "build/"),
			new Nuget.File(Tools.Path([staging_dir, "runtimes\\**\\*"], check_exists: false), "runtimes/"),
			new Nuget.File(Tools.Path([staging_dir, "tools\\**\\*"], check_exists: false), "tools/"),
		]);
		Package.Package();
	}
	public override void Publish()
	{
		base.Publish();
		if (Package is null)
			throw new Exception("Call Deploy before calling Publish");

		// Delegate the policy decision so every local publish attempt is rejected consistently.
		Package.Publish();
	}
}

// All projects
public class All : Group
{
	public All(string workspace, List<string>? platforms = null, List<string>? configs = null)
		: base(workspace)
	{
		foreach (var type in typeof(Common).Assembly.GetTypes().Where(t => t.IsClass && !t.IsAbstract && t.IsSubclassOf(typeof(Common)) && !t.IsSubclassOf(typeof(Group))))
		{
			var instance = (Common?)Activator.CreateInstance(type, workspace, platforms, configs) ?? throw new Exception($"Failed to create instance of type {type}");
			Items.Add(instance);
		}
	}
}

// Ensure the workspace directories exist and SDK dependencies are fetched
void SetupWorkspace(string workspace)
{
	// Ensure required directories exist
	string[] required_dirs = [
		IOPath.Combine(workspace, "obj"),
		IOPath.Combine(workspace, "lib", "packages"),
		IOPath.Combine(workspace, "lib", "packages", "debug"),
		IOPath.Combine(workspace, "lib", "packages", "release"),
	];
	foreach (var dir in required_dirs)
	{
		Directory.CreateDirectory(dir);
	}

	// Pre-fetch SDK dependencies sequentially to avoid parallel dotnet-script conflicts.
	// Each entry is (marker_path, script_path) - if the marker doesn't exist, run the script.
	(string Marker, string Script)[] sdk_deps = [
		(IOPath.Combine(workspace, "sdk", "cgltf", "cgltf"), IOPath.Combine(workspace, "sdk", "cgltf", "_get.csx")),
		(IOPath.Combine(workspace, "sdk", "imgui", "imgui"), IOPath.Combine(workspace, "sdk", "imgui", "_get.csx")),
		(IOPath.Combine(workspace, "sdk", "ufbx", "ufbx"), IOPath.Combine(workspace, "sdk", "ufbx", "_get.csx")),
		(IOPath.Combine(workspace, "sdk", "openxr", "openxr"), IOPath.Combine(workspace, "sdk", "openxr", "_get.csx")),
	];

	// Track whether any download occurred so progress output stays concise.
	bool any_fetched = false;
	foreach (var (marker, script) in sdk_deps)
	{
		if (Directory.Exists(marker) || File.Exists(marker))
			continue;

		// Ignore optional dependencies whose repository does not provide a fetch script.
		if (!File.Exists(script))
			continue;

		// Print the section heading once before the first dependency is fetched.
		if (!any_fetched)
		{
			Console.WriteLine("Fetching SDK dependencies...");
			any_fetched = true;
		}

		// Fetch dependencies sequentially because concurrent dotnet-script restores share global state.
		var sdk_name = IOPath.GetFileName(IOPath.GetDirectoryName(script));
		Console.WriteLine($"  Fetching {sdk_name}...");
		Tools.Run(["dotnet-script", script], return_output: false);
	}
	if (any_fetched)
		Console.WriteLine("SDK dependencies ready.\n");

	// Restore NuGet packages for C++ projects (packages.config style).
	// This must happen before building because MSBuild /t:restore only handles PackageReference.
	if (!BuildOptions.Restore)
	{
		Console.WriteLine("Restoring NuGet packages skipped (-norestore).\n");
		return;
	}

	// Discover only repository project dependencies so managed PackageReference projects remain outside this legacy restore.
	var projects_dir = IOPath.Combine(workspace, "projects");
	var package_configs = Directory
		.EnumerateFiles(projects_dir, "packages.config", SearchOption.AllDirectories)
		.OrderBy(x => x, StringComparer.OrdinalIgnoreCase)
		.ToArray();
	if (package_configs.Length == 0)
	{
		Console.WriteLine("No packages.config dependencies found.\n");
		return;
	}

	// Ensure the repository-local NuGet client exists before restoring packages.config projects.
	var nuget_exe = IOPath.Combine(workspace, "tools", "nuget", "nuget.exe");
	if (!File.Exists(nuget_exe))
	{
		Console.WriteLine("Downloading nuget.exe...");
		var get_nuget = IOPath.Combine(workspace, "tools", "nuget", "_get.ps1");
		Tools.Run(["pwsh", "-NonInteractive", "-NoProfile", "-File", get_nuget], return_output: false);
	}
	if (!File.Exists(nuget_exe))
		throw new FileNotFoundException("NuGet client download completed without producing the expected executable.", nuget_exe);

	// Restore each legacy dependency file into the shared repository package directory without traversing managed projects.
	var packages_dir = IOPath.Combine(workspace, "packages");
	Console.WriteLine("Restoring NuGet packages (packages.config)...");
	foreach (var package_config in package_configs)
	{
		var relative_path = IOPath.GetRelativePath(workspace, package_config);
		Console.WriteLine($"  Restoring {relative_path}...");
		Tools.Run([nuget_exe, "restore", package_config, "-PackagesDirectory", packages_dir, "-Verbosity", "quiet", "-NonInteractive"], return_output: false);
	}
	Console.WriteLine("");
}

// Build script main function
void Main(IList<string> args)
{
	// Set defaults for command line options
	string workspace = UserVars.Root;
	List<EProjects> projects = [];
	List<string>? platforms = null;
	List<string>? configs = null;
	bool clean = false;
	bool build = false;
	bool deploy = false;
	bool publish = false;

	// Distinguish option values from the next command-line switch.
	bool IsDataArg(int i) => i != args.Count && args[i].StartsWith('-') == false;

	// Parse command line
	for (int i = 0; i != args.Count;)
	{
		var arg = args[i++].ToLowerInvariant();
		switch (arg)
		{
			case "-clean":
				{
					clean = true;
					break;
				}
			case "-build":
				{
					build = true;
					break;
				}
			case "-nobuild":
				{
					build = false;
					break;
				}
			case "-rebuild":
				{
					clean = true;
					build = true;
					break;
				}
			case "-deploy":
				{
					deploy = true;
					break;
				}
			case "-publish":
				{
					publish = true;
					break;
				}
			case "-norestore":
				{
					BuildOptions.Restore = false;
					break;
				}
			case "-notests":
			case "-nounittests":
				{
					BuildOptions.RunUnitTests = false;
					break;
				}
			case "-nopack":
			case "-nopackage":
			case "-nopackages":
				{
					BuildOptions.GeneratePackageOnBuild = false;
					break;
				}
			case "-profile":
				{
					BuildOptions.Profile = true;
					break;
				}
			case "-profiledir":
				{
					if (!IsDataArg(i)) throw new Exception("Profile output directory expected");
					BuildOptions.Profile = true;
					BuildOptions.ProfileDir = args[i++];
					break;
				}
			case "-cert_pfx":
				{
					if (!IsDataArg(i)) throw new Exception("Code signing certificate PFX path expected");
					UserVars.CodeSignCert_Pfx = args[i++];
					break;
				}
			case "-cert_pw":
				{
					if (!IsDataArg(i)) throw new Exception("Code signing certificate password expected");
					UserVars.CodeSignCert_Pw = args[i++];
					break;
				}
			case "-workspace":
				{
					if (!IsDataArg(i)) throw new Exception("Workspace argument missing");
					workspace = args[i++];
					break;
				}
			case "-project":
			case "-projects":
				{
					if (!IsDataArg(i))
					{
						Console.WriteLine(string.Join("\n", Enum<EProjects>.Values));
						return;
					}
					for (; IsDataArg(i);)
					{
						var proj_name = args[i++].Replace(".", "");
						var proj_enum = Enum<EProjects>.Parse(proj_name, ignore_case: true);
						projects.Add(proj_enum);
					}
					break;
				}
			case "-platform":
			case "-platforms":
				{
					if (!IsDataArg(i)) throw new Exception("Platform argument missing");
					platforms ??= [];
					for (; IsDataArg(i);)
					{
						var platform = args[i++];
						if (platform.ToLowerInvariant() == "x64") platform = "x64";
						if (platform.ToLowerInvariant() == "x86") platform = "x86";
						if (platform.ToLowerInvariant() == "any cpu") platform = "Any CPU";
						if (platform.ToLowerInvariant() == "anycpu") platform = "Any CPU";
						platforms.Add(platform);
					}
					break;
				}
			case "-config":
			case "-configs":
				{
					if (!IsDataArg(i)) throw new Exception("Config argument missing");
					configs ??= [];
					for (; IsDataArg(i); )
					{
						var config = args[i++];
						if (config.ToLowerInvariant() == "release") config = "Release";
						if (config.ToLowerInvariant() == "debug") config = "Debug";
						configs.Add(config);
					}
					break;
				}
			default:
				{
					throw new Exception($"Unknown command line argument: {args[i-1]}");
				}
		}
	}

	// Normalise parameters
	if (projects.Count == 0) projects.Add(EProjects.All);
	build |= !clean && !build && !deploy && !publish;
	deploy |= publish;

	// Profiles go to a fixed location unless the caller chose one.
	if (BuildOptions.Profile && string.IsNullOrEmpty(BuildOptions.ProfileDir))
		BuildOptions.ProfileDir = Tools.Path([workspace, "obj", "build-profile"], check_exists: false);

	// Ensure workspace is ready (directories, SDK dependencies)
	SetupWorkspace(workspace);

	// Instantiate every selected builder with the command-line platform and configuration scope.
	var builders = new List<Common>();
	foreach (var project in projects)
	{
		// Builder classes are named after the EProjects values.
		var builder_type = Type.GetType($"Submission#0+{project}") ?? throw new Exception($"Builder type '{project}' not found");
		var builder = (Common?)Activator.CreateInstance(builder_type, workspace, platforms, configs) ?? throw new Exception($"Failed to create the builder type for {project}");
		builders.Add(builder);
	}

	// Warn once if Azure signing is not configured for deploy/publish
	if (deploy && !Tools.SigningAvailable)
		Console.WriteLine("Warning: Azure Trusted Signing not configured. Artifacts will not be signed.");

	// Clean every selected project before anything is built.
	if (clean)
	{
		// Cleaning is file deletion only, so order does not matter.
		foreach (var builder in builders)
			builder.Clean();
	}

	// Collect the projects of every selected builder into one plan, then build them all in parallel.
	if (build)
	{
		// Builders only describe their projects; ParallelBuild owns all MSBuild invocations.
		var plan = new BuildPlan();
		foreach (var builder in builders)
			builder.Build(plan);

		ParallelBuild.Run(workspace, plan);
	}

	// Deploy the named project(s). Deploy steps package shared outputs, so they run one at a time.
	if (deploy)
	{
		// Run in command-line order so dependent packages see their dependencies first.
		foreach (var builder in builders)
			builder.Deploy();
	}

	// Publish the named project(s)
	if (publish)
	{
		// Publish only after every package has been created successfully.
		foreach (var builder in builders)
			builder.Publish();
	}

	// Report completion only after every requested project operation succeeds.
	Console.WriteLine($"\nComplete: {workspace}");
	return;
}

// Convert an unhandled build failure into a process exit code that CI can observe.
try
{
	// Testing
	List<string> args =
		//["-project", "View3d", "-build", "-deploy"]
		//["-project", "Rylogic.Core", "-build", "-deploy"]
		//["-project", "Rylogic.Gfx", "-deploy"]
		//["-project", "LDraw", "-deploy"];
		//["-project", "AllNative", "-build", "-deploy"]
		//["-project", "AllRylogic", "-build", "-deploy"]
		Args.ToList()
	;
	if (!args.SequenceEqual(Args))
	    Console.WriteLine("WARNING: Command line overridden for testing");

	// Execute the script with either the real arguments or the temporary local override above.
	Main(args);
}
catch (Exception ex)
{
	Console.WriteLine(ex.Message);
	Environment.ExitCode = 1;
}
