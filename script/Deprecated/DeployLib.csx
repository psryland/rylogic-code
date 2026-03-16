#! "net9.0"
#load "UserVars.csx"
#load "Tools.csx"
#nullable enable

// Post Build Event for exporting library files to a directory
// Use:
//   dotnet-script DeployLib.csx $(TargetPath)
// 
// Note: 
//  pdb files are associated with the file name at the time they are build so it is
//  not possible to rename the lib and pdb. 

using System;
using System.IO;
using System.Text.RegularExpressions;
using Console = System.Console;

try
{
	List<string> args =
		//["E:\\Rylogic\\Code\\obj\\v145\\view3d-12\\x64\\Debug\\view3d-12-static.lib"]
		Args.ToList()
	;
	if (!args.SequenceEqual(Args))
	    Console.WriteLine("WARNING: Command line overridden for testing");

	var target_path = args.Count > 0 ? args[0] : throw new Exception("Target Path?");
	Tools.DeployLib(target_path);
}
catch (Exception ex)
{
	Console.Error.WriteLine(ex.Message);
}
