using System;
using System.Diagnostics;
using System.Reflection;
using Rylogic.UnitTests;

namespace Rylogic.Audio.Authoring;

/// <summary>Runs embedded authoring tests for Debug builds.</summary>
public static class Program
{
	/// <summary>Discover and run the tests embedded in this assembly.</summary>
	public static int Main()
	{
		// Report the process kind so a test log shows which runtime ran the tests
		var assembly = Assembly.GetExecutingAssembly();
		Debug.WriteLine($"{assembly.GetName().FullName} running as a {(Environment.Is64BitProcess ? "64bit" : "32bit")} process");
		return Environment.ExitCode = UnitTest.RunTests(assembly) ? 0 : 1;
	}
}
