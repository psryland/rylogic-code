using System;
using System.Diagnostics;
using System.Reflection;
using Rylogic.UnitTests;

namespace Rylogic.Audio;

/// <summary>Runs embedded Audio tests for Debug builds.</summary>
public static class Program
{
	/// <summary>Discover and run the tests embedded in this assembly.</summary>
	public static int Main()
	{
		var assembly = Assembly.GetExecutingAssembly();
		Debug.WriteLine($"{assembly.GetName().FullName} running as a {(Environment.Is64BitProcess ? "64bit" : "32bit")} process");
		return Environment.ExitCode = UnitTest.RunTests(assembly) ? 0 : 1;
	}
}
