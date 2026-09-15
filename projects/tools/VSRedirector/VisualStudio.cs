using System.Diagnostics;
using System.Runtime.InteropServices;
using System.Runtime.InteropServices.ComTypes;

namespace vsredirector;

// All automation objects belong to the short-lived STA worker, never to a shared host.
internal sealed class ComScope : IDisposable
{
	private readonly List<object> m_objects = [];
	private readonly HashSet<object> m_seen = new(ReferenceEqualityComparer.Instance);

	public dynamic Own(object value)
	{
		if (Marshal.IsComObject(value) && m_seen.Add(value))
			m_objects.Add(value);

		return value;
	}
	public void Dispose()
	{
		for (var i = m_objects.Count; i != 0;)
		{
			var value = m_objects[--i];
			if (Marshal.IsComObject(value)) Marshal.FinalReleaseComObject(value);
		}
	}
}

// Read-only discovery is completed for every same-session IDE before routing or opening anything.
internal sealed class VisualStudio : IDisposable
{
	private readonly ComScope m_com = new();
	private readonly Dictionary<int, object> m_instances = [];
	private const string SolutionFolder = "{66A26720-8FB5-11D2-AA7E-00C04F688DDE}";
	private const string MiscellaneousFiles = "{66A2671D-8FB5-11D2-AA7E-00C04F688DDE}";
	private const string UnloadedProject = "{67294A52-A4F0-11D2-AA88-00C04F688DDE}";
	private const string TextView = "{7651A703-06E5-11D1-8EBD-00A0C90F26EA}";

	public void Dispose()
	{
		m_com.Dispose();
	}

	// Retry only server-busy/rejected calls, for a bounded interval. Other failures are not no-match results.
	internal static T Retry<T>(Func<T> action)
	{
		var timer = Stopwatch.StartNew();
		for (;;)
		{
			try { return action(); }
			catch (COMException ex) when ((ex.HResult == unchecked((int)0x80010001) || ex.HResult == unchecked((int)0x8001010A)) && timer.Elapsed < TimeSpan.FromSeconds(3))
			{
				Thread.Sleep(100);
			}
		}
	}

	// ROT absence is also checked against live processes, including elevated or not-yet-registered instances.
	public IReadOnlyList<Instance> Inspect(Request request)
	{
		var before = ProcessIds();
		BindRunning();
		var missing = before.Except(m_instances.Keys).ToArray();
		if (missing.Length != 0)
			throw new InvalidOperationException($"Cannot inspect Visual Studio PID(s) {string.Join(", ", missing)}. They may be starting, elevated, or inaccessible. Wait for startup/modal dialogs to finish and run GitKraken and VS at the same normal privilege level. No fallback instance will be launched.");

		var windows = WindowOrder();
		var facts = new List<Instance>();
		foreach (var pid in before)
		{
			try
			{
				facts.Add(Retry(() => InspectInstance(pid, request.FilePath, windows)));
			}
			catch (Exception ex) when (ex is Microsoft.CSharp.RuntimeBinder.RuntimeBinderException or InvalidCastException)
			{
				throw new InvalidOperationException($"VSRedirector could not bind a Visual Studio automation member in PID {pid}. This is an automation compatibility defect, not a busy/access diagnosis. No fallback instance will be launched.", ex);
			}
			catch (COMException ex)
			{
				var guidance = ex.HResult switch
				{
					unchecked((int)0x80010001) or unchecked((int)0x8001010A) => "The IDE is busy; finish loading projects and close modal dialogs.",
					unchecked((int)0x80070005) => "Access was denied; run GitKraken and VS at the same normal privilege level.",
					_ => "The IDE returned an automation error; this does not establish a busy/access problem.",
				};
				throw new InvalidOperationException($"Could not completely inspect Visual Studio PID {pid}. {guidance} No fallback instance will be launched.", ex);
			}
		}
		if (!before.SetEquals(ProcessIds()))
			throw new InvalidOperationException("Visual Studio instances changed during discovery. Retry once startup/shutdown finishes.");

		return facts;
	}

	private Instance InspectInstance(int pid, string file, IReadOnlyDictionary<nint, int> windows)
	{
		dynamic dte = m_instances[pid];
		dynamic solution = m_com.Own(dte.Solution);
		var solution_path = (string)solution.FullName;
		var contains = ContainsFile(solution, file);
		dynamic documents = m_com.Own(dte.Documents);
		var open = false;
		for (var i = 1; i <= (int)documents.Count && !open; ++i)
		{
			dynamic document = m_com.Own(documents.Item(i));
			open = Paths.Same((string)document.FullName, file);
		}
		dynamic main_window = m_com.Own(dte.MainWindow);
		var hwnd = (nint)(uint)(int)main_window.HWnd;
		Native.GetWindowThreadProcessId(hwnd, out var owner);
		if (owner != pid)
			throw new InvalidOperationException($"The automation window for PID {pid} changed during inspection. Retry.");

		var rank = windows.TryGetValue(hwnd, out var position) ? position : int.MaxValue;
		return new Instance(pid, solution_path, contains, open, rank);
	}

	internal bool ContainsFile(dynamic solution, string file)
	{
		var contains = false;
		if (HasLoadedSolution((bool)solution.IsOpen, (string)solution.FullName))
		{
			// FindProjectItem searches the loaded solution, including nested and linked items. Verify its full path and owner.
			dynamic? item = solution.FindProjectItem(file);
			if (item != null)
			{
				m_com.Own(item);
				dynamic project = m_com.Own(item.ContainingProject);
				contains = IsRealProject((string)project.Kind) && ItemMatches(item, file);
				if (!contains)
				{
					// A solution item can shadow the real project item returned by the first-match lookup.
					dynamic projects = m_com.Own(solution.Projects);
					for (var i = 1; i <= (int)projects.Count && !contains; ++i)
						contains = ProjectContains(m_com.Own(projects.Item(i)), file);
				}
			}
		}
		return contains;
	}

	internal static bool IsRealProject(string kind)
	{
		return !kind.Equals(SolutionFolder, StringComparison.OrdinalIgnoreCase)
			&& !kind.Equals(MiscellaneousFiles, StringComparison.OrdinalIgnoreCase)
			&& !kind.Equals(UnloadedProject, StringComparison.OrdinalIgnoreCase);
	}

	// Loose documents can have IsOpen == true but no actual solution file.
	internal static bool HasLoadedSolution(bool is_open, string path)
	{
		return is_open && !string.IsNullOrEmpty(path);
	}

	private bool ItemMatches(object item, string file)
	{
		// Managed project systems need the declared ProjectItem dispatch interface for this indexed property.
		var files = (IProjectItemFiles)item;
		for (int i = 1, count = files.FileCount; i <= count; ++i)
		{
			if (Paths.Same(files[(short)i], file)) return true;
		}
		return false;
	}

	private bool ProjectContains(dynamic project, string file)
	{
		var kind = (string)project.Kind;
		if (kind.Equals(UnloadedProject, StringComparison.OrdinalIgnoreCase) || kind.Equals(MiscellaneousFiles, StringComparison.OrdinalIgnoreCase))
			return false;

		dynamic? items = project.ProjectItems;
		return items != null && ItemsContain(m_com.Own(items), file, IsRealProject(kind));
	}

	private bool ItemsContain(dynamic items, string file, bool real_project)
	{
		for (var i = 1; i <= (int)items.Count; ++i)
		{
			dynamic item = m_com.Own(items.Item(i));
			if (real_project && ItemMatches(item, file)) return true;
			dynamic? subproject = item.SubProject;
			if (subproject != null && ProjectContains(m_com.Own(subproject), file)) return true;
			dynamic? children = item.ProjectItems;
			if (children != null && ItemsContain(m_com.Own(children), file, real_project)) return true;
		}
		return false;
	}

	// Reuse the exact ROT object. File and line navigation never go through devenv /Edit.
	public void Open(int pid, Request request, bool require_empty_solution = false)
	{
		dynamic dte = m_instances[pid];
		if (require_empty_solution)
		{
			dynamic solution = m_com.Own(dte.Solution);
			if (Retry(() => HasLoadedSolution((bool)solution.IsOpen, (string)solution.FullName)))
				throw new InvalidOperationException($"New Visual Studio PID {pid} unexpectedly loaded a solution (possibly startup restoration). It was left untouched. Disable automatic solution restoration before retrying.");
		}
		Navigate(dte, request, m_com);
	}

	// Both reused and newly created instances use the same document-specific navigation.
	internal static void Navigate(dynamic dte, Request request, ComScope com)
	{
		dynamic operations = com.Own(Retry(() => (object)dte.ItemOperations));
		dynamic window = com.Own(Retry(() => (object)operations.OpenFile(request.FilePath, request.Line != null ? TextView : "{00000000-0000-0000-0000-000000000000}")));
		Retry(() => { window.Activate(); return true; });
		if (request.Line != null)
		{
			// Navigate the returned document, not a possibly unrelated global ActiveDocument.
			dynamic document = com.Own(Retry(() => (object)window.Document));
			if (!Paths.Same((string)document.FullName, request.FilePath))
				throw new InvalidOperationException("Visual Studio did not open the requested document; line navigation was cancelled.");

			dynamic selection = com.Own(Retry(() => (object)document.Selection));
			dynamic text_document = com.Own(Retry(() => (object)selection.Parent));
			dynamic end_point = com.Own(Retry(() => (object)text_document.EndPoint));
			var last_line = Retry(() => (int)end_point.Line);
			if (request.Line.Value > last_line)
				throw new ArgumentOutOfRangeException(nameof(request.Line), $"Cannot navigate to line {request.Line.Value}: '{request.FilePath}' has {last_line} lines. The file is open; choose a line from 1 to {last_line}.");

			Retry(() => { selection.GotoLine(request.Line.Value, false); return true; });
		}
		dynamic main_window = com.Own(Retry(() => (object)dte.MainWindow));
		Retry(() => { main_window.Activate(); return true; });
	}

	// Bind only the process just launched; never adopt a different instance if startup fails.
	public void WaitForNew(Process process)
	{
		var timer = Stopwatch.StartNew();
		while (timer.Elapsed < TimeSpan.FromSeconds(60))
		{
			if (process.HasExited)
				throw new InvalidOperationException($"New Visual Studio PID {process.Id} exited before automation became available.");

			BindRunning(process.Id);
			if (m_instances.ContainsKey(process.Id)) return;
			Thread.Sleep(250);
		}
		throw new TimeoutException($"New Visual Studio PID {process.Id} did not register automation within 60 seconds. It was left running; check its startup before retrying.");
	}

	private void BindRunning(int? only_pid = null)
	{
		using var scope = new ComScope();
		Marshal.ThrowExceptionForHR(Native.GetRunningObjectTable(0, out var rot));
		scope.Own(rot);
		Marshal.ThrowExceptionForHR(Native.CreateBindCtx(0, out var context));
		scope.Own(context);
		rot.EnumRunning(out var enumerator);
		scope.Own(enumerator);
		var monikers = new IMoniker[1];
		while (enumerator.Next(1, monikers, nint.Zero) == 0)
		{
			using var moniker_scope = new ComScope();
			var moniker = monikers[0];
			moniker_scope.Own(moniker);
			moniker.GetDisplayName(context, null, out var name);
			if (!name.StartsWith("!VisualStudio.DTE.", StringComparison.OrdinalIgnoreCase)) continue;
			if (!int.TryParse(name[(name.LastIndexOf(':') + 1)..], out var pid))
				throw new InvalidOperationException($"Unrecognized Visual Studio ROT identity: {name}");

			if (only_pid != null && pid != only_pid || m_instances.ContainsKey(pid)) continue;
			try
			{
				Retry(() => { Marshal.ThrowExceptionForHR(rot.GetObject(moniker, out var value)); m_instances.Add(pid, m_com.Own(value)); return true; });
			}
			catch (COMException ex)
			{
				throw new InvalidOperationException($"Cannot access Visual Studio PID {pid}. Finish startup, close modal dialogs, and run GitKraken and VS at the same normal privilege level.", ex);
			}
		}
	}

	private static HashSet<int> ProcessIds()
	{
		using var self = Process.GetCurrentProcess();
		var result = new HashSet<int>();
		foreach (var process in Process.GetProcessesByName("devenv"))
		{
			using (process)
			{
				if (process.SessionId == self.SessionId) result.Add(process.Id);
			}
		}
		return result;
	}

	// Owned popups count with their main window. Z-order can change independently of activation history.
	private static Dictionary<nint, int> WindowOrder()
	{
		var result = new Dictionary<nint, int>();
		var visited = new HashSet<nint>();
		for (var hwnd = Native.GetTopWindow(nint.Zero); hwnd != nint.Zero; hwnd = Native.GetWindow(hwnd, 2))
		{
			if (!visited.Add(hwnd))
				throw new InvalidOperationException("Window stacking order changed during discovery. Retry.");

			if (!Native.IsWindowVisible(hwnd)) continue;
			var root = Native.GetAncestor(hwnd, 3);
			result.TryAdd(root, visited.Count);
		}
		return result;
	}
}

// The indexed-file subset of EnvDTE.ProjectItem, invoked through IDispatch rather than a partial vtable.
[ComImport, Guid("0B48100A-473E-433C-AB8F-66B9739AB620"), InterfaceType(ComInterfaceType.InterfaceIsIDispatch)]
internal interface IProjectItemFiles
{
	[DispId(11)]
	string this[short index]
	{
		[return: MarshalAs(UnmanagedType.BStr)]
		get;
	}
	[DispId(13)]
	short FileCount { get; }
}

#if PR_UNITTESTS
internal static partial class Tests
{
	private static void TestNavigation()
	{
		var file = @"C:\clone a\file.cs";
		var target = new FakeIde(file);
		using var com = new ComScope();
		VisualStudio.Navigate(target, new Request(file, 42, false), com);
		Check(target.ItemOperations.m_file == file, "navigation exact path");
		Check(target.ItemOperations.m_view == "{7651A703-06E5-11D1-8EBD-00A0C90F26EA}", "line navigation text view");
		Check(target.ItemOperations.m_window.Document.Selection.m_line == 42, "line reaches returned document");
		Check(target.ItemOperations.m_window.m_active && target.MainWindow.m_active, "activate target document and IDE");
		var plain = new FakeIde(file);
		VisualStudio.Navigate(plain, new Request(file, null, false), com);
		Check(plain.ItemOperations.m_window.Document.Selection.m_line == null, "plain open leaves line unchanged");
		Throws(() => VisualStudio.Navigate(new FakeIde(@"C:\wrong.cs"), new Request(file, 42, false), com), "do not navigate unrelated document");
		var short_file = new FakeIde(file);
		short_file.ItemOperations.m_window.Document.Selection.Parent.EndPoint.Line = 19;
		try
		{
			VisualStudio.Navigate(short_file, new Request(file, 42, false), com);
			throw new InvalidOperationException("Out-of-range navigation was accepted.");
		}
		catch (ArgumentOutOfRangeException ex)
		{
			Check(ex.Message.Contains("line 42") && ex.Message.Contains("19 lines"), "requested and available lines in error");
			Check(short_file.ItemOperations.m_window.Document.Selection.m_line == null, "out-of-range navigation does not move the selection");
		}
		var attempts = 0;
		Check(VisualStudio.Retry(() =>
		{
			if (++attempts == 1) throw new COMException("busy", unchecked((int)0x8001010A));
			return true;
		}) && attempts == 2, "busy automation call retries");
		attempts = 0;
		try
		{
			VisualStudio.Retry<bool>(() => { ++attempts; throw new COMException("access denied", unchecked((int)0x80070005)); });
			throw new InvalidOperationException("Access denied was swallowed.");
		}
		catch (COMException ex) when (ex.HResult == unchecked((int)0x80070005))
		{
			Check(attempts == 1, "access failure is not retried as a busy server");
		}

		// Exercise membership against DTE-shaped objects, including first-match shadowing by solution items.
		using var vs = new VisualStudio();
		var project = new FakeProject("{FAE04EC0-301F-11D3-BF4B-00C04F79EFBC}", new FakeCollection());
		var linked = new FakeItem(project, file);
		var solution = new FakeSolution(linked, new FakeCollection(project));
		Check(vs.ContainsFile(solution, file), "linked exact full path membership");
		Check(!vs.ContainsFile(solution, @"C:\clone b\file.cs"), "membership rejects different clone");
		solution.FullName = "";
		Check(!vs.ContainsFile(solution, file), "synthetic solution excluded despite matching item");
		solution.FullName = @"C:\other\solution.sln";
		var folder = new FakeProject(SolutionFolderKind, new FakeCollection());
		solution.m_match = new FakeItem(folder, file);
		Check(!vs.ContainsFile(solution, file), "solution item alone is not membership");
		project.ProjectItems = new FakeCollection(new FakeItem(project, @"C:\parent.cs") { ProjectItems = new FakeCollection(linked) });
		folder.ProjectItems = new FakeCollection(new FakeItem(folder, "") { SubProject = project });
		solution.Projects = new FakeCollection(folder);
		Check(vs.ContainsFile(solution, file), "nested project and dependent item behind solution item");
		var misc = new FakeProject("{66A2671D-8FB5-11D2-AA7E-00C04F688DDE}", new FakeCollection());
		solution.m_match = new FakeItem(misc, file);
		solution.Projects = new FakeCollection(misc);
		Check(!vs.ContainsFile(solution, file), "misc project excluded in a named solution");
	}

	private const string SolutionFolderKind = "{66A26720-8FB5-11D2-AA7E-00C04F688DDE}";
	internal sealed class FakeCollection(params object[] values)
	{
		public int Count
		{
			get { return values.Length; }
		}
		public object Item(int index)
		{
			return values[index - 1];
		}
	}
	internal sealed class FakeProject(string kind, FakeCollection items)
	{
		public string Kind { get; } = kind;
		public FakeCollection ProjectItems { get; set; } = items;
	}
	internal sealed class FakeItem(FakeProject project, string path) : IProjectItemFiles
	{
		public FakeProject ContainingProject { get; } = project;
		public short FileCount
		{
			get { return 1; }
		}
		string IProjectItemFiles.this[short index]
		{
			get
			{
				Check(index == 1, "indexed COM file property receives one-based Int16");
				return path;
			}
		}
		public FakeProject? SubProject { get; set; }
		public FakeCollection? ProjectItems { get; set; }
	}
	internal sealed class FakeSolution(FakeItem? match, FakeCollection projects)
	{
		public FakeItem? m_match = match;
		public bool IsOpen { get; } = true;
		public string FullName { get; set; } = @"C:\a.sln";
		public FakeCollection Projects { get; set; } = projects;
		public FakeItem? FindProjectItem(string file)
		{
			return m_match;
		}
	}
	internal sealed class FakeIde(string file)
	{
		public FakeOperations ItemOperations { get; } = new(file);
		public FakeWindow MainWindow { get; } = new("");
	}
	internal sealed class FakeOperations(string file)
	{
		public string? m_file;
		public string? m_view;
		public readonly FakeWindow m_window = new(file);
		public FakeWindow OpenFile(string path, string view)
		{
			m_file = path;
			m_view = view;
			return m_window;
		}
	}
	internal sealed class FakeWindow(string file)
	{
		public bool m_active;
		public FakeDocument Document { get; } = new(file);
		public void Activate()
		{
			m_active = true;
		}
	}
	internal sealed class FakeDocument(string file)
	{
		public string FullName { get; } = file;
		public FakeSelection Selection { get; } = new();
	}
	internal sealed class FakeSelection
	{
		public int? m_line;
		public FakeTextDocument Parent { get; } = new();
		public void GotoLine(int line, bool select)
		{
			m_line = line;
		}
	}
	internal sealed class FakeTextDocument
	{
		public FakeTextPoint EndPoint { get; } = new();
	}
	internal sealed class FakeTextPoint
	{
		public int Line { get; set; } = 100;
	}
}
#endif

internal static class Native
{
	[DllImport("ole32.dll")]
	internal static extern int GetRunningObjectTable(int reserved, out IRunningObjectTable table);
	[DllImport("ole32.dll")]
	internal static extern int CreateBindCtx(int reserved, out IBindCtx context);
	[DllImport("user32.dll")]
	internal static extern nint GetTopWindow(nint window);
	[DllImport("user32.dll")]
	internal static extern nint GetWindow(nint window, uint command);
	[DllImport("user32.dll")]
	internal static extern nint GetAncestor(nint window, uint flags);
	[DllImport("user32.dll")]
	[return: MarshalAs(UnmanagedType.Bool)]
	internal static extern bool IsWindowVisible(nint window);
	[DllImport("user32.dll")]
	internal static extern uint GetWindowThreadProcessId(nint window, out uint pid);
	[DllImport("user32.dll", CharSet = CharSet.Unicode)]
	internal static extern int MessageBoxW(nint owner, string text, string caption, uint type);
}
