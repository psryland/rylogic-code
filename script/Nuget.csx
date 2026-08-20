#! "net10.0"
#r "System.Text.Json"
#r "nuget: Rylogic.Core, 2.0.2"
#load "UserVars.csx"
#load "Tools.csx"
#nullable enable

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Runtime;
using System.Text.RegularExpressions;
using System.Xml;
using System.Xml.Linq;
using Rylogic.Extn;
using IOPath = System.IO.Path;

public class Nuget
{
	// Notes:
	//  - Assembly signing, packaging, and publishing are all separate steps.
	public record class File(string Path, string Target, bool Sign = false) { }
	public record class Dep(string AssemblyName, string Version, string? Framework = null) { }

	// Identifies a runtime framework required by package consumers for a target framework.
	public record class FrameworkRef(string Name, string Framework) { }

	public string PackageName = "";
	public string Version = "1.0.0";
	public string? Description;
	public string Tags = "";
	public List<File> Files = [];
	public List<Dep> Deps = [];
	public List<FrameworkRef> FrameworkRefs = [];
	
	// Set automatically by 'Package()'
	public string PackagePath = "";

	// Create a Nuget package from the given project file
	// Expects a file called 'package.nuspec' in the same directory as 'proj'
	// The resulting package will be in UserVars.root\lib\packages
	// The full package path is returned
	public void Package()
	{
		// Create a nuspec file based on the template
		var ns = "http://schemas.microsoft.com/packaging/2013/05/nuspec.xsd";
		var xml_nuspec = XDocument.Load(Tools.Path([UserVars.Root, "build\\nuget\\rylogic.template.nuspec"])).Root ?? throw new Exception("Root element not found in nuspec template");;

		// Update metadata
		var xml_metadata = xml_nuspec.Element(XName.Get("metadata", ns)) ?? throw new Exception("metadata element not found in nuspec template");
		xml_metadata.Element(XName.Get("id", ns))?.SetValue(PackageName);
		xml_metadata.Element(XName.Get("version", ns))?.SetValue(Version);
		if (Description is not null)
			xml_metadata.Element(XName.Get("description", ns))?.SetValue(Description);

		xml_metadata.Element(XName.Get("tags", ns))?.SetValue(Tags);

		// Add each file to the spec
		var xml_files = xml_nuspec.Element(XName.Get("files", ns)) ?? throw new Exception("files element not found in nuspec template");
		foreach (var file in Files)
		{
			// Check the file exists
			if (!file.Path.Contains('?') && !file.Path.Contains('*') && !IOPath.Exists(file.Path))
				throw new Exception($"File missing for package {PackageName}: {file.Path}");

			// Sign if required
			if (file.Sign)
				Tools.SignAssembly(file.Path);

			// Add the file to the spec
			xml_files.Add2(new XElement(XName.Get("file", ns),
				new XAttribute("src", file.Path),
				new XAttribute("target", file.Target.Replace('\\','/'))
				));
		}

		// Add dependencies
		var xml_dependencies = xml_metadata.Element(XName.Get("dependencies", ns)) ?? throw new Exception("dependencies element not found in nuspec template");
		var has_framework_dependencies = Deps.Any(x => x.Framework != null);
		if (!has_framework_dependencies)
		{
			foreach (var dep in Deps)
			{
				xml_dependencies.Add2(new XElement(XName.Get("dependency", ns),
					new XAttribute("id", dep.AssemblyName),
					new XAttribute("version", dep.Version)
				));
			}
		}
		else
		{
			// NuGet forbids mixing dependency and group elements, so common dependencies must be repeated for every packaged target framework.
			var frameworks = Files
				.Select(file => file.Target.Replace('\\', '/').Trim('/').Split('/'))
				.Where(parts => parts.Length >= 2 && parts[0] == "lib")
				.Select(parts => parts[1])
				.Concat(Deps.Where(dep => dep.Framework != null).Select(dep => dep.Framework!))
				.Distinct(StringComparer.OrdinalIgnoreCase);
			foreach (var framework in frameworks)
			{
				var dependencies = Deps.Where(dep => dep.Framework == null || string.Equals(dep.Framework, framework, StringComparison.OrdinalIgnoreCase));
				xml_dependencies.Add2(new XElement(XName.Get("group", ns),
					new XAttribute("targetFramework", framework),
					dependencies.Select(dep => new XElement(XName.Get("dependency", ns),
						new XAttribute("id", dep.AssemblyName),
						new XAttribute("version", dep.Version)
					))
				));
			}
		}

		// Declare runtime frameworks that package consumers must select.
		if (FrameworkRefs.Count != 0)
		{
			var xml_framework_references = new XElement(XName.Get("frameworkReferences", ns));
			foreach (var group in FrameworkRefs.GroupBy(x => x.Framework))
			{
				xml_framework_references.Add2(new XElement(XName.Get("group", ns),
					new XAttribute("targetFramework", group.Key),
					group.Select(framework => new XElement(XName.Get("frameworkReference", ns),
						new XAttribute("name", framework.Name)
					))
				));
			}
			xml_dependencies.AddAfterSelf(xml_framework_references);
		}

		var objdir = Path.Combine(Path.GetTempPath(), "rylogic");
		Directory.CreateDirectory(objdir);

		// Write the nuspec to 'objdir'
		var nuspec = Tools.Path([objdir, $"{PackageName}.nuspec"], check_exists: false);
		xml_nuspec.Save(nuspec);

		// Build the Nuget package directly in the lib\packages folder
		Tools.Run([UserVars.Nuget, "pack", nuspec,
			"-OutputDirectory", Tools.Path([UserVars.Root, "lib\\packages"], check_exists: false),
			"-Verbosity", "detailed",
		]);

		// Sign the package
		PackagePath = Tools.Path([UserVars.Root, $"lib\\packages\\{PackageName}.{Version}.nupkg"]);
		Tools.SignNugetPackage(PackagePath);
	}

	// Push a nugget package to 'nuget.org'
	public void Publish()
	{
		if (PackagePath.Length == 0) throw new Exception("Call Package() first");
		Tools.Run([UserVars.Nuget, "push", PackagePath, UserVars.NugetApiKey, "-source", "https://api.nuget.org/v3/index.json"]);
	}
}
