#! "net9.0"
#r "System.Text.Json"
#r "nuget: Rylogic.Core, 2.0.2"
#load "UserVars.csx"
#load "Tools.csx"
#nullable enable
// Use:
//  BuildInstaller $(TargetDir)
//
// This script creates a '<projname>Installer.msi' using the WIX tool kit.
//
// There are a number of ways I could have used WIX. Here are the options and reasons for the current system:
// 1) Install the WIX tool kit as a visual studio extension
//    => maintaining a .wsx file for each project that an installer is needed for.
//    - Two projects per application, one to build the .msi and another to build the .exe
//    - Requires anyone who wants to build the project to also install it. Projects are unsupported otherwise
//    - Doesn't solve the problem of automatically adding doc files, updating version numbers, etc
//
// 2) Maintain a .wsx file for each project, build from the deploy.py script using WIX command line tools.
//    + Tools are in SDK, anyone with the code also has the tools to build the installer
//    - Requires manually adding files, and updating version numbers
//
// 3) Use a text template file to generate the .wsx file
//   => might as well use python, easier to debug/maintain
//
// 4) Use a script to generate the .wsx file, then build it
//   + One-click build of installer
//   + Automatically gets the correct files and version number
//   + Can be invoked from the deploy script
//   + Can be parametrized on the project name
//   + Can be debugged using VS
//   - It's gunna be a mofo of a script...

using System;
using System.IO;
using System.Text.RegularExpressions;
using System.Xml;
using System.Xml.Linq;
using System.Collections.Generic;
using System.Diagnostics;
using Rylogic.Extn;

public class HarvestPath
{
	public string group_id = null!;
	public string install_dir = null!;
	public string relative_dir = null!;
	public List<Regex> filters = [];
	public bool recursive = false;

	public HarvestPath(string group_id, string install_dir, string relative_dir, bool recursive = false, List<Regex>? filters = null)
	{
		this.group_id = group_id;
		this.install_dir = install_dir;
		this.relative_dir = relative_dir;
		this.recursive = recursive;
		this.filters = filters ?? [];
	}
}

public class BuildInstaller
{	
	// Build an installer .msi file
	// 'projname' is the name of the application (<projname>Installer_v<version>.msi is returned)
	// 'version' is the application version number
	// 'installer' is the full path to the main installer.wxs file
	// 'projdir' is the root folder of the project (passed as a define to the installer.wxs, used as a reference directory)
	// 'targetdir' is the staging directory from where files are taken for the installer
	// 'dstdir' is the output directory for the installer file
	// 'harvest' is a tuple list used to add files from a directory:
	//   [group_id, install_directory, targetdir_relative_directory, recursive, (optional) regex_filters...]
	//   'group_id' is the name of the corresponding 'ComponentGroupRef' element in the 'Feature' element of the installer.wxs
	//   'install_directory' is the id of the directory that will contain the harvested files
	//   'targetdir_relative_directory' is the relative path of where to harvest from.
	//   'recursive' should be True to search directories recursively
	//   'regex_filters' is used to match specific filenames to add to the group (no filters means add all)
	// Returns the installer full path
	public static string Build(string projname, string version, string installer, string projdir, string targetdir, string dstdir, List<HarvestPath> harvest)
	{
		// Notes:
		//  - The installer.wxs file should define the basic directory layout for the install plus
		//    the main executeables or files with additional shortcuts etc. Supplimental files (dlls etc)
		//    can be provided by the 'harvest' argument.
		//  - The 'group_id' is used to identify blocks of files that are included in the MSI by the
		//    'Feature' element in the installer.wxs

		// Check the installer file exists
		if (!File.Exists(installer))
			throw new Exception($"'{installer}' does not exist");

		// Ensure WiX tools are available
		Tools.Run([UserVars.Pwsh, Tools.Path([UserVars.Root, "tools\\wix\\_get.ps1"])]);

		// Create a temporary working directory for the 'wixobj' files
		var objdir = Path.Combine(Path.GetTempPath(), Path.GetRandomFileName());
		Directory.CreateDirectory(objdir);

		// The .msi filename
		var msi_filename = $"{projname}Installer_v{version}.msi";

		// Copy the main installer to the 'objdir' directory so that all the .wsx files are in the same folder
		Tools.Copy(installer, objdir, quiet: true);
		installer = Path.GetFileName(installer);

		// Collect all the .wsx files to build
		List<string> wsx_files = [installer];

		// Create .wsx fragment files for the harvest files and directories
		foreach (var h in harvest)
		{
			// The name of the wix file for this group
			var wxs_file = $"{h.group_id}.wxs";

			// Harvest files
			var search_path = Path.Combine(targetdir, h.relative_dir);
			var xml_root = HarvestDirectory(h.group_id, search_path, h.recursive, h.filters, h.install_dir);
			//Tools.WriteXml(xml_roo, os.path.join(UserVars.dumpdir, wxs_file))
			//print(f"HACK - Writing temp wix: {os.path.join(UserVars.dumpdir, wxs_file)}")

			// Save to a temporary file
			xml_root.Save(Path.Combine(objdir, wxs_file));

			// Collect the .wxs files
			wsx_files.Add(wxs_file);
		}

		// Compile all of the .wxs files to object files in 'objdir'
		var wix_candle = Tools.Path([UserVars.Root, @"tools\wiX\wix\candle.exe"]);
		Tools.Run((List<string>)[
			wix_candle,
			"-nologo",
			$"-dProjDir={projdir}",
			$"-dTargetDir={targetdir}",
			$"-dProductVersion={version}",
			"-arch", "x64",
			"-out", objdir + Path.DirectorySeparatorChar,
			..wsx_files.ConvertAll(f => Path.Combine(objdir, f)),
		]);

		// Create the .msi
		var wix_light = Tools.Path([UserVars.Root, @"tools\wiX\wix\light.exe"]);
		Tools.Run((List<string>)[
			wix_light,
			"-nologo",
			$"-dProjDir={projdir}",
			$"-dTargetDir={targetdir}",
			$"-dProductVersion={version}",
			"-ext", "WixUIExtension",
			"-ext", "WixUtilExtension",
			"-ext", "WixNetFxExtension",
			"-out", Path.GetFullPath(Path.Combine(objdir, msi_filename)),
			..wsx_files.ConvertAll(f => Path.Combine(objdir, Path.ChangeExtension(f, ".wixobj"))),
		]);

		// Copy the .msi file to the destination directory
		Tools.Copy(Path.Combine(objdir, msi_filename), dstdir, quiet: true);

		// Clean up
		Directory.Delete(objdir, true);
		return Path.GetFullPath(Path.Combine(dstdir, msi_filename));
	}

	// Generate a UUID suitable for an Id
	private static string Id() => Guid.NewGuid().ToString("N").ToUpper();

	/// <summary>Default namespace for WiX3</summary>
	private static string DefaultNS => XNamespace.Get("http://schemas.microsoft.com/wix/2006/wi").NamespaceName;

	// Create an XML tree of a WiX fragment by enumerating the files within a directory.
	// 'group_id' is the corresponding ComponentGroupRef for the feature
	// 'dir' is the directory to harvest
	// 'recursive' is true if directories should be harvested recursively
	// 'filters' is a collection of filters for the files to include
	// 'install_dir' is the directory Id in the main installer
	private static XElement HarvestDirectory(string group_id, string dir, bool recursive, List<Regex> filters, string install_dir)
	{
		var xml_root = new XElement(XName.Get("Wix", DefaultNS));

		// Add files to a fragment
		var xml_frag = new XElement(XName.Get("Fragment", DefaultNS));
		xml_root.Add(xml_frag);

		// The directory structure sub tree
		var xml_dr = new XElement(XName.Get("DirectoryRef", DefaultNS), new XAttribute("Id", install_dir));
		xml_frag.Add(xml_dr);

		// The component group sub tree
		var xml_cg = new XElement(XName.Get("ComponentGroup", DefaultNS), new XAttribute("Id", group_id));
		xml_frag.Add(xml_cg);

		// Walk the directory recursively
		void WalkDir(XElement xml_dr, XElement xml_cg, string directory)
		{
			foreach (var fname in Directory.GetFileSystemEntries(directory))
			{
				var path = Path.GetFullPath(fname);
				if (File.Exists(path))
				{
					if (filters.Count != 0 && !filters.Any(f => f.IsMatch(Path.GetFileName(path)))) continue;
					CreateFileComponent(xml_cg, path, true, xml_dr.Attribute("Id")?.Value);
				}
				if (Directory.Exists(path) && recursive)
				{
					var xml_sub = new XElement(XName.Get("Directory", DefaultNS),
						new XAttribute("Name", Path.GetFileName(path)),
						new XAttribute("Id", $"Dir_{Path.GetFileName(path)}_{Id()}"));
					xml_dr.Add(xml_sub);
					WalkDir(xml_sub, xml_cg, path);
				}
			}
		}

		WalkDir(xml_dr, xml_cg, dir);
		return xml_root;
	}

	// Create a component group in 'frag'
	private static void CreateComponentGroup(XElement elem, string id, string directory, IEnumerable<string> filepaths)
	{
		// Create the 'ComponentGroup' XML element
		var cg = elem.Add2(new XElement(XName.Get("ComponentGroup", DefaultNS),
			new XAttribute("Id", id),
			new XAttribute("Directory", directory)
		));

		// Add each component
		bool keypath = true;
		foreach (var filepath in filepaths)
		{
			CreateFileComponent(cg, filepath, keypath);
			keypath = false;
		}
	}

	// Add a single file component to 'elem'
	private static void CreateFileComponent(XElement elem, string filepath, bool keypath = true, string? directory_id = null)
	{
		// Get the file title from the full filepath
		var file = Path.GetFileNameWithoutExtension(filepath);
		file = Regex.Replace(file, @"[^0-9a-zA-Z_]", "_");
		var uid = Id();

		// Create the component to contain the file
		var cmp = elem.Add2(new XElement(XName.Get("Component", DefaultNS),
			new XAttribute("Id", $"Cmp_{file}_{uid}"),
			new XAttribute("Guid", "*")
		));
		if (directory_id != null)
			cmp.SetAttributeValue("Directory", directory_id);

		var fcp = cmp.Add2(new XElement(XName.Get("File", DefaultNS),
			new XAttribute("Id", $"{file}_{uid}"),
			new XAttribute("Source", Path.GetFullPath(filepath))
		));
		if (keypath)
			fcp.SetAttributeValue("KeyPath", "yes");
	}
}