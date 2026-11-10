//***************************************************
// UnitTest Runner
//  Copyright (c) Rylogic Ltd 2014
//***************************************************

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Runtime.Serialization;
using System.Runtime.Versioning;
using System.Text;
using System.Threading;
using Rylogic.Common;
using Rylogic.Extn;
using Rylogic.Utility;

namespace Rylogic.UnitTests
{
	// How to debug:
	//  - Change the OutputType to "WinExe" in the project file
	//  - Set the DLL as the startup project
	//  - Put a breakpoint in the test you care about
	//  - Hit F5.

	/// <summary>Properties controlling when a managed unit test should run</summary>
	[Flags]
	public enum EUnitTestFlags
	{
		None     = 0,
		Quick    = 1 << 0,
		Extended = 1 << 1,
		Stress   = 1 << 2,

		/// <summary>
		/// The fixture must not run at the same time as any other fixture (e.g. it uses global state or a shared device).
		/// Applies to the whole fixture, whether it is set on the fixture or on any of its tests.</summary>
		Serial   = 1 << 3,

		/// <summary>The groups that select whether a test runs</summary>
		RunGroups = Quick | Extended | Stress,
	}

	public class UnitTest
	{
#if PR_UNITTESTS
		public const bool TestsEnabled = true;
#else
		public const bool TestsEnabled = false;
#endif
		/// <summary>
		/// Loads a .NET assembly and runs the tests in its 'TestFixture' classes that have all of 'flags'. Returns true if all tests passed.
		/// Fixtures run on a pool of 'threads' worker threads (zero means one per processor), with each fixture's tests running in order on one thread.
		/// Fixtures with the 'Serial' flag run afterwards on the calling thread, with nothing else running. Everything runs on the calling thread
		/// when a debugger is attached, so that exceptions break where they are thrown. Diagnostic output is written to 'outstream', and the
		/// duration of each test is written to 'obj/unittests/(assembly name)/(output folder)/timings.csv' under the repository root.</summary>
		public static bool RunTests(Assembly ass, Stream? outstream = null, EUnitTestFlags flags = EUnitTestFlags.Quick, int threads = 0)
		{
			try
			{
				ass = ass ?? throw new ArgumentNullException("ass", "Assembly was null");
				outstream ??= Console.OpenStandardOutput();

				var banner =
					$"Unit Testing:  {ass.GetName().Name}.dll  " +
					$"Framework={ass.FindAttribute<TargetFrameworkAttribute>()?.FrameworkName ?? ass.ImageRuntimeVersion}  " +
					$"Config={ass.FindAttribute<AssemblyConfigurationAttribute>()?.Configuration ?? "???"}  " +
					//$"Platform={(Environment.Is64BitProcess ? "x64" : "x86")}  " +
					$"Version={ass.GetName().Version}";

				using var outp = new StreamWriter(outstream, Encoding.ASCII, 4096, true);
				outp.WriteLine(banner);
				var stopwatch = Stopwatch.StartNew();

				// Select the tests in each fixture. Flags are resolved before fixture construction so an excluded fixture cannot perform disruptive setup.
				var fixture_runs = new List<FixtureRun>();
				var fixture_count = 0;
				var skipped = 0;
				foreach (var fixture in FindTestFixtures(ass))
				{
					// A fixture is serial if any of its tests resolve to 'Serial' (which includes 'Serial' on the fixture itself).
					++fixture_count;
					var tests = fixture
						.FindMethodsWithAttribute<TestAttribute>()
						.Select(test => new TestCase(test, ResolveTestFlags(fixture, test)))
						.ToList();
					var selected = tests.Where(test => (test.Flags & flags) == flags).ToList();
					skipped += tests.Count - selected.Count;
					if (selected.Count == 0)
						continue;

					var serial = tests.Any(test => test.Flags.HasFlag(EUnitTestFlags.Serial));
					fixture_runs.Add(new FixtureRun(fixture, selected, serial));
				}

				// Start the fixtures that took longest last time first, so a slow fixture does not start near the end and extend the total run time.
				var report_path = TimingReportPath(ass);
				var previous = ReadFixtureDurations(report_path);
				var parallel_runs = fixture_runs
					.Where(run => !run.Serial)
					.OrderByDescending(run => previous.TryGetValue(run.Fixture.FullName ?? run.Fixture.Name, out var ms) ? ms : 0.0)
					.ToList();
				var serial_runs = fixture_runs.Where(run => run.Serial).ToList();

				// Write each piece of output in one locked write so output from fixtures running at the same time does not interleave.
				var output_lock = new object();
				void Report(string text)
				{
					lock (output_lock)
					{
						outp.Write(text);
						outp.Flush();
					}
				}

				// Run the parallel fixtures on worker threads that share the calling thread's apartment state (e.g. STA for WPF tests).
				var thread_count = Debugger.IsAttached ? 1 : Math.Min(threads > 0 ? threads : Environment.ProcessorCount, parallel_runs.Count);
				if (thread_count <= 1)
				{
					foreach (var run in parallel_runs)
						RunFixture(run, Report);
				}
				else
				{
					// Workers take the next fixture from a shared index until none remain.
					var next = -1;
					var apartment = Thread.CurrentThread.GetApartmentState();
					var workers = new List<Thread>();
					for (var i = 0; i != thread_count; ++i)
					{
						var worker = new Thread(() =>
						{
							// Claim fixtures one at a time.
							for (int index; (index = Interlocked.Increment(ref next)) < parallel_runs.Count;)
								RunFixture(parallel_runs[index], Report);
						})
						{
							IsBackground = true,
							Name = $"UnitTest worker {i}",
						};
						worker.SetApartmentState(apartment);
						worker.Start();
						workers.Add(worker);
					}
					foreach (var worker in workers)
						worker.Join();
				}

				// Run the serial fixtures with nothing else running.
				foreach (var run in serial_runs)
					RunFixture(run, Report);

				// Count the outcomes. Fixture-level failures (construction, set up, clean up) count as failed tests.
				var results = fixture_runs.SelectMany(run => run.Results).ToList();
				var passed = results.Count(result => result.Passed);
				var failed = results.Count(result => !result.Passed) + fixture_runs.Sum(run => run.FixtureFailures);
				var test_time_ms = results.Sum(result => result.DurationMS);

				// Write the timing report, slowest first, so it can be read top-down to find tests that do not belong in the 'Quick' group.
				var slowest = results.OrderByDescending(result => result.DurationMS).ToList();
				try
				{
					Directory.CreateDirectory(Path.GetDirectoryName(report_path)!);
					using var csv = new StreamWriter(report_path, false, Encoding.UTF8);
					csv.WriteLine("class,test,flags,result,duration_ms");
					foreach (var result in slowest)
						csv.WriteLine($"{result.Fixture},{result.Name},{FlagsText(result.Flags)},{(result.Passed ? "pass" : "fail")},{result.DurationMS.ToString("F3", CultureInfo.InvariantCulture)}");
				}
				catch (Exception ex)
				{
					outp.WriteLine($"Unit Testing:  Warning - failed to write timing report '{report_path}'. {ex.Message}");
				}

				// List the slowest tests to show which may not belong in the 'Quick' group.
				if (slowest.Count != 0)
				{
					outp.WriteLine("Unit Testing:  Slowest tests:");
					foreach (var result in slowest.Take(10))
						outp.WriteLine($"   {result.DurationMS,10:F1} ms  {result.Fixture}.{result.Name}");

					outp.WriteLine($"Unit Testing:  Timing report: {report_path}");
				}

				var timing = $"(taking {stopwatch.Elapsed.TotalMilliseconds:F0} ms, {test_time_ms:F0} ms of test time on {Math.Max(thread_count, 1)} thread(s))";
				if (fixture_count == 0 || (passed == 0 && failed == 0 && skipped == 0))
				{
					outp.WriteLine($"Unit Testing:  No unit tests found in assembly {ass.FullName}");
				}
				else if (failed > 0)
				{
					outp.WriteLine($"Unit Testing:  ERROR - {failed} Unit tests failed {timing}");
					if (skipped != 0)
						outp.WriteLine($"Unit Testing:  {skipped} Unit tests skipped by flag filter");
				}
				else
				{
					outp.WriteLine($"Unit Testing:  *** All {passed} unit tests passed *** {timing}");
					if (skipped != 0)
						outp.WriteLine($"Unit Testing:  {skipped} Unit tests skipped by flag filter");
				}
				outp.Flush();
				return failed == 0;
			}
			catch (Exception ex) when (!Debugger.IsAttached) // Don't catch while debugging
			{
				Console.WriteLine($"Unit Testing:  Unhandled exception running unit tests: {ex.Message}");
				return false;
			}
		}

		/// <summary>Run the selected tests of one fixture in order on the calling thread, recording the results in 'run'</summary>
		private static void RunFixture(FixtureRun run, Action<string> report)
		{
			// Format an exception, with source links, for the build output window.
			const string file_pattern = @" in " + Regex_.FullPathPattern + @":line\s(?<line>\d+)";
			static string Describe(Exception ex)
			{
				// Report the real exception rather than the reflection wrapper.
				while (ex is TargetInvocationException && ex.InnerException != null) ex = ex.InnerException;
				return $"{ex.MessageFull()}\n{(ex.StackTrace is string st ? Util.FormatForOutputWindow(st, file_pattern) : string.Empty)}\n";
			}
			var fixture = run.Fixture;

			// Create an instance of the unit test
			object inst;
			try { inst = Activator.CreateInstance(fixture) ?? throw new Exception($"Failed to create unit test {fixture.Name}"); }
			catch (Exception ex)
			{
				report($"Unit Testing:  {fixture.Name} - Failed to create an instance of test fixture\n{Describe(ex)}");
				++run.FixtureFailures;
				return;
			}

			// Call the fixture set up method
			var setup = fixture.FindMethodsWithAttribute<TestFixtureSetUpAttribute>().FirstOrDefault();
			if (setup != null)
			{
				try
				{
					setup.Invoke(inst, null);
				}
				catch (Exception ex)
				{
					report($"Unit Testing:  {fixture.Name} - Test fixture set up function threw\n{Describe(ex)}");
					++run.FixtureFailures;
					return;
				}
			}

			// Find the set up/clean up method to call before each unit test
			var test_setup = fixture.FindMethodsWithAttribute<SetUpAttribute>().FirstOrDefault();
			var test_clean = fixture.FindMethodsWithAttribute<TearDownAttribute>().FirstOrDefault();

			// Run the unit tests, timing each one including its set up and clean up.
			foreach (var test in run.Tests)
			{
				var passed = false;
				var stopwatch = Stopwatch.StartNew();
				try
				{
					// Call set up for the test
					if (test_setup != null)
						test_setup.Invoke(inst, null);

					// Exception Here?
					//  - Look at 'fixture' to see the text name.
					//  - Put a break point on the first line of that test.
					//  - Turn on first chance exceptions.

					// Run the test
					test.Method.Invoke(inst, null);

					// Call clean up for the test
					if (test_clean != null)
						test_clean.Invoke(inst, null);

					passed = true;
				}
				catch (Exception ex) when (!Debugger.IsAttached) // Don't catch while debugging
				{
					report($"\nUnit Testing:  Test {test.Method.Name} Failed\n{Describe(ex)}");
				}
				run.Results.Add(new TestResult(fixture.FullName ?? fixture.Name, test.Method.Name, test.Flags, passed, stopwatch.Elapsed.TotalMilliseconds));
			}

			// Call the fixture clean up method
			var cleanup = fixture.FindMethodsWithAttribute<TestFixtureTearDownAttribute>().FirstOrDefault();
			if (cleanup != null)
			{
				try
				{
					cleanup.Invoke(inst, null);
				}
				catch (Exception ex)
				{
					report($"Unit Testing:  {fixture.Name} - Test fixture clean up function threw\n{Describe(ex)}");
					++run.FixtureFailures;
				}
			}
		}

		/// <summary>The timing report for 'ass': 'obj/unittests/(assembly name)/(output folder)/timings.csv' under the repository root</summary>
		private static string TimingReportPath(Assembly ass)
		{
			// Each target framework builds into its own output folder and may be tested concurrently, so the folder name keeps their reports separate.
			var output_folder = Path.GetFileName(Path.GetDirectoryName(ass.Location)) is { Length: > 0 } folder ? folder : "unknown";
			return Path_.CombinePath(RepoRoot, "obj", "unittests", ass.GetName().Name ?? "unknown", output_folder, "timings.csv");
		}

		/// <summary>Read the total duration of each fixture from a previous timing report. Returns an empty map when there is no report.</summary>
		private static Dictionary<string, double> ReadFixtureDurations(string report_path)
		{
			// The report columns are 'class,test,flags,result,duration_ms', none of which contain commas.
			var durations = new Dictionary<string, double>();
			if (!File.Exists(report_path))
				return durations;

			foreach (var line in File.ReadLines(report_path).Skip(1))
			{
				// Accumulate each test's duration into its fixture total, skipping malformed lines.
				var columns = line.Split(',');
				if (columns.Length < 5 || !double.TryParse(columns[4], NumberStyles.Float, CultureInfo.InvariantCulture, out var ms))
					continue;

				durations[columns[0]] = durations.TryGetValue(columns[0], out var total) ? total + ms : ms;
			}
			return durations;
		}

		/// <summary>Format test flags as a pipe-separated expression</summary>
		private static string FlagsText(EUnitTestFlags flags)
		{
			return string.Join("|", Enum.GetValues(typeof(EUnitTestFlags)).Cast<EUnitTestFlags>().Where(flag => flag != EUnitTestFlags.RunGroups && flag != EUnitTestFlags.None && flags.HasFlag(flag)));
		}

		/// <summary>A test method with its resolved flags</summary>
		private sealed class TestCase
		{
			public TestCase(MethodInfo method, EUnitTestFlags flags)
			{
				Method = method;
				Flags = flags;
			}
			public MethodInfo Method { get; }
			public EUnitTestFlags Flags { get; }
		}

		/// <summary>The outcome of one executed test</summary>
		private sealed class TestResult
		{
			public TestResult(string fixture, string name, EUnitTestFlags flags, bool passed, double duration_ms)
			{
				Fixture = fixture;
				Name = name;
				Flags = flags;
				Passed = passed;
				DurationMS = duration_ms;
			}
			public string Fixture { get; }
			public string Name { get; }
			public EUnitTestFlags Flags { get; }
			public bool Passed { get; }
			public double DurationMS { get; }
		}

		/// <summary>The selected tests of one fixture and their results. Only the thread running the fixture modifies it.</summary>
		private sealed class FixtureRun
		{
			public FixtureRun(Type fixture, List<TestCase> tests, bool serial)
			{
				Fixture = fixture;
				Tests = tests;
				Serial = serial;
			}
			public Type Fixture { get; }
			public List<TestCase> Tests { get; }
			public bool Serial { get; }
			public List<TestResult> Results { get; } = new List<TestResult>();
			public int FixtureFailures { get; set; }
		}

		/// <summary>Returns the test fixture types in 'ass'</summary>
		private static IEnumerable<Type> FindTestFixtures(Assembly ass)
		{
			foreach (var type in ass.GetExportedTypes())
			{
				var attr = type.FindAttribute<TestFixtureAttribute>(false);
				if (attr == null)
					continue;

				yield return type;
			}
		}

		/// <summary>
		/// Resolve the run groups from a method override, then a fixture override, then the Quick default.
		/// 'Serial' from either the method or the fixture is always included.</summary>
		internal static EUnitTestFlags ResolveTestFlags(Type fixture, MethodInfo test)
		{
			// Run groups come from the most specific attribute that specifies any; 'Serial' alone does not change the run groups.
			var method_flags = test.FindAttribute<TestFlagsAttribute>(false)?.Flags ?? EUnitTestFlags.None;
			var fixture_flags = fixture.FindAttribute<TestFlagsAttribute>(false)?.Flags ?? EUnitTestFlags.None;
			var groups =
				(method_flags & EUnitTestFlags.RunGroups) != 0 ? method_flags & EUnitTestFlags.RunGroups :
				(fixture_flags & EUnitTestFlags.RunGroups) != 0 ? fixture_flags & EUnitTestFlags.RunGroups :
				EUnitTestFlags.Quick;
			return groups | ((method_flags | fixture_flags) & EUnitTestFlags.Serial);
		}

		/// <summary>Find the repository root (at runtime)</summary>
		public static string RepoRoot
		{
			get
			{
				// This is needed because build determinism prevents building absolute paths into the assembly.
				if (m_repo_root == null)
				{
					for (var dir = new DirectoryInfo(Directory.GetCurrentDirectory()); dir != null; dir = dir.Parent)
					{
						// System.Console.WriteLine(dir.FullName);
						if (!dir.GetFiles("Rylogic.sln").Any()) continue;
						m_repo_root = dir.FullName;
						break;
					}
				}
				return m_repo_root ?? throw new DirectoryNotFoundException("Repository root not found.");
			}
		}
		private static string? m_repo_root;

		/// <summary>The repo location containing unit test resources</summary>
		public static string ResourcePath
		{
			get
			{
				var path = Path_.CombinePath(RepoRoot, @"projects\tests\unittests\res");
				if (!Path_.DirExists(path)) throw new Exception($"Unit Testing: Resource path '{path}' is missing.");
				return path;
			}
		}

		/// <summary>The location of the compiled libraries</summary>
		public static string LibPath
		{
			get
			{
				var path = Path_.CombinePath(RepoRoot, @"lib");
				if (!Path_.DirExists(path)) throw new Exception($"Unit Testing: Library path '{path}' is missing.");
				return path;
			}
		}
	}

	/// <summary>Exceptions thrown by unit test failures</summary>
	public class UnitTestException :System.Exception
	{
		public UnitTestException() :base() {}
		public UnitTestException(string message) :base(message) {}
		public UnitTestException(string message, Exception innerException) :base(message, innerException) {}
	}

	/// <summary>Assert functions</summary>
	public static class Assert
	{
		/// <summary>Return a file and line link</summary>
		private static string VSLink
		{
			get
			{
				var st = new System.Diagnostics.StackTrace(true);
				var sf = st.GetFrame(2);
				var file = sf?.GetFileName() ?? string.Empty;
				var line = sf?.GetFileLineNumber() ?? 0;
				return $"{file}({line}) : ";
			}
		}

		public static void True(bool test)
		{
			if (test) return;
			throw new UnitTestException(VSLink + "result is not 'True'");
		}
		public static void False(bool test)
		{
			if (!test) return;
			throw new UnitTestException(VSLink + "result is not 'False'");
		}
		public static void Null(object? ptr)
		{
			if (ptr == null) return;
			throw new UnitTestException(VSLink + "reference is not Null");
		}
		public static void NotNull(object? ptr)
		{
			if (ptr != null) return;
			throw new UnitTestException(VSLink + "reference is Null");
		}

		/// <summary>Tests reference equality</summary>
		public static void AreSame(object? lhs, object? rhs)
		{
			if (ReferenceEquals(lhs, rhs)) return;
			throw new UnitTestException(VSLink + "references are not equal");
		}

		/// <summary>Tests value equality</summary>
		public static void Equal(object? expected, object? result)
		{
			if (Equals(expected, result)) return;
			throw new UnitTestException(VSLink + $"values are not equal\r\n  expected: {expected ?? "null"}\r\n    result: {result ?? "null"}");
		}

		/// <summary>Tests value equality</summary>
		public static void Equal<T>(IEnumerable<T>? expected, IEnumerable<T>? result)
		{
			if (ReferenceEquals(expected, result)) return;
			if (expected == null || result == null) throw new UnitTestException(VSLink + "sequences are not equal");
			if (expected.SequenceEqual(result)) return;
			throw new UnitTestException(VSLink + $"sequences are not equal\r\n  expected: {(expected != null ? string.Join(",", expected) : "null")}\r\n    result: {(result != null ? string.Join(",", result) : "null")}");
		}

		/// <summary>Tests value equality</summary>
		public static void Equal(string? expected, string? result)
		{
			if (expected == result) return;
			throw new UnitTestException(VSLink + $"sequences are not equal\r\n  expected: {expected ?? "null"}\r\n    result: {result ?? "null"}");
		}

		/// <summary>Tests value equality</summary>
		public static void Equal(double expected, double result, double tol)
		{
			if (Math.Abs(result - expected) < tol) return;
			throw new UnitTestException(VSLink + $"values are not equal\r\n  expected: {expected}\r\n    result: {result}\r\n  tol: {tol}");
		}
		public static void Equal(decimal expected, decimal result, decimal tol)
		{
			if (Math.Abs(result - expected) < tol) return;
			throw new UnitTestException(VSLink + $"values are not equal\r\n  expected: {expected}\r\n    result: {result}\r\n  tol: {tol}");
		}

		/// <summary>Tests value inequality</summary>
		public static void NotEqual(object? expected, object? result)
		{
			if (!Equals(expected, result)) return;
			throw new UnitTestException(VSLink + "values are equal");
		}

		public static void Throws(Type exception_type, Action action)
		{
			try
			{
				action();
				throw new UnitTestException(VSLink + "no exception was thrown");
			}
			catch (Exception ex)
			{
				if (exception_type.IsAssignableFrom(ex.GetType())) return;
				throw new UnitTestException(VSLink + "exception of the wrong type thrown", ex);
			}
		}
		public static void Throws<T>(Action action) where T:Exception
		{
			Throws(typeof(T), action);
		}
		public static void DoesNotThrow(Action action, string? msg = null)
		{
			try { action(); }
			catch (Exception ex)
			{
				throw new UnitTestException(VSLink + (msg ?? "exception was thrown"), ex);
			}
		}
	}

	/// <summary>Common base class for all UnitTest Attributes</summary>
	public abstract class AttributeBase :Attribute
	{}

	/// <summary>Overrides the default Quick execution properties for a test fixture or method</summary>
	[AttributeUsage(AttributeTargets.Class | AttributeTargets.Method, AllowMultiple=false, Inherited=true)]
	public sealed class TestFlagsAttribute :AttributeBase
	{
		/// <summary>Define the execution properties for a test fixture or method</summary>
		public TestFlagsAttribute(EUnitTestFlags flags)
		{
			const EUnitTestFlags all_flags = EUnitTestFlags.RunGroups | EUnitTestFlags.Serial;
			if (flags == EUnitTestFlags.None || (flags & ~all_flags) != 0)
				throw new ArgumentOutOfRangeException(nameof(flags), flags, "At least one recognised unit-test flag is required.");

			Flags = flags;
		}

		/// <summary>The execution properties declared for the target</summary>
		public EUnitTestFlags Flags { get; }
	}

	/// <summary>Marks a class as a unit test class</summary>
	[AttributeUsage(AttributeTargets.Class,AllowMultiple=true)]
	public class TestFixtureAttribute :AttributeBase {}

	/// <summary>Called once just after creation of a TestFixture class</summary>
	[AttributeUsage(AttributeTargets.Method)]
	public class TestFixtureSetUpAttribute :AttributeBase {}

	/// <summary>Called once after the last test in a TestFixture class has been executed</summary>
	[AttributeUsage(AttributeTargets.Method)]
	public class TestFixtureTearDownAttribute :AttributeBase {}

	/// <summary>Called just before a unit test in a test fixture is executed</summary>
	[AttributeUsage(AttributeTargets.Method)]
	public class SetUpAttribute :AttributeBase {}

	/// <summary>Called just after a unit test in a test fixture has executed</summary>
	[AttributeUsage(AttributeTargets.Method)]
	public class TearDownAttribute :AttributeBase {}

	/// <summary>Marks a method as a unit test method</summary>
	[AttributeUsage(AttributeTargets.Method)]
	public class TestAttribute :AttributeBase {}

	/// <summary>Tests managed unit-test flag resolution independently of test execution</summary>
	[TestFixture]
	public class TestUnitTestFlags
	{
		/// <summary>Prove absent flags default to Quick and explicit method metadata overrides fixture metadata</summary>
		[Test]
		public void ResolvesDefaultsAndOverrides()
		{
			var default_test = typeof(DefaultFixture).GetMethod(nameof(DefaultFixture.Default)) ?? throw new MissingMethodException();
			var extended_test = typeof(ExtendedFixture).GetMethod(nameof(ExtendedFixture.Extended)) ?? throw new MissingMethodException();
			var stress_test = typeof(ExtendedFixture).GetMethod(nameof(ExtendedFixture.Stress)) ?? throw new MissingMethodException();

			Assert.Equal(EUnitTestFlags.Quick, UnitTest.ResolveTestFlags(typeof(DefaultFixture), default_test));
			Assert.Equal(EUnitTestFlags.Extended, UnitTest.ResolveTestFlags(typeof(ExtendedFixture), extended_test));
			Assert.Equal(EUnitTestFlags.Extended | EUnitTestFlags.Stress, UnitTest.ResolveTestFlags(typeof(ExtendedFixture), stress_test));
		}

		/// <summary>Prove 'Serial' is merged from the fixture and the method without changing the run groups</summary>
		[Test]
		public void ResolvesSerial()
		{
			var fixture_serial = typeof(SerialFixture).GetMethod(nameof(SerialFixture.Inherited)) ?? throw new MissingMethodException();
			var method_serial = typeof(DefaultFixture).GetMethod(nameof(DefaultFixture.Serial)) ?? throw new MissingMethodException();
			var both = typeof(SerialFixture).GetMethod(nameof(SerialFixture.Extended)) ?? throw new MissingMethodException();

			Assert.Equal(EUnitTestFlags.Quick | EUnitTestFlags.Serial, UnitTest.ResolveTestFlags(typeof(SerialFixture), fixture_serial));
			Assert.Equal(EUnitTestFlags.Quick | EUnitTestFlags.Serial, UnitTest.ResolveTestFlags(typeof(DefaultFixture), method_serial));
			Assert.Equal(EUnitTestFlags.Extended | EUnitTestFlags.Serial, UnitTest.ResolveTestFlags(typeof(SerialFixture), both));
		}

		/// <summary>Fixture with no explicit execution metadata</summary>
		private sealed class DefaultFixture
		{
			/// <summary>Marker method that inherits the Quick default</summary>
			public void Default()
			{
			}

			/// <summary>Marker method that is serial but inherits the Quick default</summary>
			[TestFlags(EUnitTestFlags.Serial)]
			public void Serial()
			{
			}
		}

		/// <summary>Fixture that must run on its own</summary>
		[TestFlags(EUnitTestFlags.Serial)]
		private sealed class SerialFixture
		{
			/// <summary>Marker method that inherits the Quick default and the fixture's 'Serial' flag</summary>
			public void Inherited()
			{
			}

			/// <summary>Marker method that overrides the run group and keeps the fixture's 'Serial' flag</summary>
			[TestFlags(EUnitTestFlags.Extended)]
			public void Extended()
			{
			}
		}

		/// <summary>Fixture whose methods are excluded from normal builds unless explicitly overridden</summary>
		[TestFlags(EUnitTestFlags.Extended)]
		private sealed class ExtendedFixture
		{
			/// <summary>Marker method that inherits its fixture metadata</summary>
			public void Extended()
			{
			}

			/// <summary>Marker method that overrides its fixture metadata</summary>
			[TestFlags(EUnitTestFlags.Extended | EUnitTestFlags.Stress)]
			public void Stress()
			{
			}
		}
	}
}
