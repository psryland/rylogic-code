using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices;
using System.Windows;
using Microsoft.VisualStudio.Shell;
using Rylogic.Common;
using Rylogic.Utility;

namespace Rylogic.LDrawVisualiser
{
	[ClassInterface(ClassInterfaceType.AutoDual)]
	[Guid("D4E5F6A1-B2C3-7890-DEFA-BC1234567890")]
	public sealed class LDrawVisualiserOptions : UIElementDialogPage, INotifyPropertyChanged
	{
		public LDrawVisualiserOptions() : base()
		{
			ResetSettings();
		}

		/// <summary>The location on disk where settings are saved</summary>
		public string SettingsFilepath => Util.ResolveAppDataPath("Rylogic", "VSExtension", "ldraw_visualiser_options.xml");

		/// <summary>The directory where named scripts are stored</summary>
		public string ScriptsDirectory => Util.ResolveAppDataPath("Rylogic", "VSExtension", "LDrawVisualiserScripts");

		/// <summary>Default script text shown when no saved script exists</summary>
		public string DefaultScriptText
		{
			get => m_default_script_text;
			set
			{
				if (m_default_script_text == value) return;
				m_default_script_text = value;
				NotifyPropertyChanged(nameof(DefaultScriptText));
			}
		}
		private string m_default_script_text = "";

		/// <summary>Default address for the LDraw streaming connection</summary>
		public string DefaultAddress
		{
			get => m_default_address;
			set
			{
				if (m_default_address == value) return;
				m_default_address = value;
				NotifyPropertyChanged(nameof(DefaultAddress));
			}
		}
		private string m_default_address = "localhost:1976";

		/// <summary>Whether auto-refresh on break is enabled by default</summary>
		public bool DefaultAutoRefresh
		{
			get => m_default_auto_refresh;
			set
			{
				if (m_default_auto_refresh == value) return;
				m_default_auto_refresh = value;
				NotifyPropertyChanged(nameof(DefaultAutoRefresh));
			}
		}
		private bool m_default_auto_refresh = true;

		/// <summary>The name of the last selected script (persisted across restarts)</summary>
		public string LastSelectedScript
		{
			get => m_last_selected_script;
			set
			{
				if (m_last_selected_script == value) return;
				m_last_selected_script = value;
				NotifyPropertyChanged(nameof(LastSelectedScript));
			}
		}
		private string m_last_selected_script = "Script";

		/// <summary>Assembly reference directives written to LDrawVisualiser.csx for IntelliSense</summary>
		public List<string> ReferenceAssemblies
		{
			get => m_reference_assemblies;
			set
			{
				m_reference_assemblies = value;
				NotifyPropertyChanged(nameof(ReferenceAssemblies));
			}
		}
		private List<string> m_reference_assemblies = new();

		/// <summary>Whether to automatically compile and run the script when the file is saved</summary>
		public bool AutoCompileOnSave
		{
			get => m_auto_compile_on_save;
			set
			{
				if (m_auto_compile_on_save == value) return;
				m_auto_compile_on_save = value;
				NotifyPropertyChanged(nameof(AutoCompileOnSave));
			}
		}
		private bool m_auto_compile_on_save = true;

		public override void ResetSettings()
		{
			DefaultScriptText =
@"#load ""LDrawVisualiser.csx""

using System;
using System.Linq;
using Rylogic.LDraw;
using Rylogic.Maths;

// Script body — this code runs inside: string Generate(dynamic vars) { ... }
// 'vars' is a dynamic proxy onto the current debugger frame. Member accesses turn
// into debug-expression evaluations, so 'vars.foo.bar' evaluates 'foo.bar' in the
// debuggee. Implicit conversions to known types (v4, m4x4, float, ...) are supported.

// --- Output window logging ---------------------------------------------------
// Print(...) and PrintLine(...) write to the 'LDraw Visualiser' pane in
// View → Output. Console.WriteLine / Debug.WriteLine do nothing here because the
// extension runs in-process inside devenv with no console attached.
vars.PrintLine(""Script run at "", DateTime.Now.ToString(""HH:mm:ss.fff""));

// --- Custom type readers -----------------------------------------------------
// Built-in readers cover scalars, v2/v3/v4, Quat, m2x2/m3x3/m4x4, BBox and
// pr::collision shapes (native + managed equivalents). To handle a custom type,
// register a reader for the exact debugger-reported type name. The lambda
// receives a DebugProxy positioned at the matched expression — chain into its
// fields the same way you would on 'vars'. Last registration wins, so you can
// override a built-in too.
vars.RegisterReader(""MyApp::Particle"", (Func<dynamic, object?>)(p =>
    new v4((float)p.pos.x, (float)p.pos.y, (float)p.pos.z, (float)p.mass)));

// Tip: put shared RegisterReader calls in a #load-ed common.csx file so multiple
// visualiser scripts can use the same custom readers.

// --- Build the LDraw scene ---------------------------------------------------
var b = new Builder();
b.Box(""obj1"").box(1, 1, 1).o2w(vars.o2w);
return b.ToString();";
			DefaultAddress = "localhost:1976";
			DefaultAutoRefresh = true;
			AutoCompileOnSave = true;
			LastSelectedScript = "Script";
			ReferenceAssemblies = new List<string>
			{
				"#r \"nuget: Rylogic.Core, *\"",
				"#r \"nuget: Rylogic.Gfx, *\"",
			};
		}

		public override void LoadSettingsFromStorage()
		{
			try
			{
				var filepath = SettingsFilepath;
				if (!Path_.FileExists(filepath))
					return;

				var root = System.Xml.Linq.XDocument.Load(filepath).Root;
				if (root == null) return;

				DefaultScriptText = (string?)root.Element(nameof(DefaultScriptText)) ?? DefaultScriptText;
				DefaultAddress = (string?)root.Element(nameof(DefaultAddress)) ?? DefaultAddress;
				LastSelectedScript = (string?)root.Element(nameof(LastSelectedScript)) ?? LastSelectedScript;
				if (bool.TryParse((string?)root.Element(nameof(DefaultAutoRefresh)), out var auto_refresh))
					DefaultAutoRefresh = auto_refresh;
				if (bool.TryParse((string?)root.Element(nameof(AutoCompileOnSave)), out var auto_compile))
					AutoCompileOnSave = auto_compile;

				var ref_asm_el = root.Element(nameof(ReferenceAssemblies));
				if (ref_asm_el != null)
					ReferenceAssemblies = ref_asm_el.Elements("Reference").Select(e => e.Value).Where(s => !string.IsNullOrWhiteSpace(s)).ToList();
			}
			catch (Exception)
			{
				// Don't allow anything to throw from here, otherwise VS locks up
			}
		}

		public override void SaveSettingsToStorage()
		{
			try
			{
				Directory.CreateDirectory(Path_.Directory(SettingsFilepath));
				var root = new System.Xml.Linq.XElement("root");
				root.Add(new System.Xml.Linq.XElement(nameof(DefaultScriptText), DefaultScriptText));
				root.Add(new System.Xml.Linq.XElement(nameof(DefaultAddress), DefaultAddress));
				root.Add(new System.Xml.Linq.XElement(nameof(DefaultAutoRefresh), DefaultAutoRefresh));
				root.Add(new System.Xml.Linq.XElement(nameof(AutoCompileOnSave), AutoCompileOnSave));
				root.Add(new System.Xml.Linq.XElement(nameof(LastSelectedScript), LastSelectedScript));
				root.Add(new System.Xml.Linq.XElement(nameof(ReferenceAssemblies),
					ReferenceAssemblies.Select(a => new System.Xml.Linq.XElement("Reference", a))));
				root.Save(SettingsFilepath);
			}
			catch (Exception)
			{
				// Don't allow anything to throw from here
			}
		}

		protected override void OnDeactivate(CancelEventArgs e)
		{
			SaveSettingsToStorage();

			// Regenerate LDrawVisualiser.csx so that IntelliSense picks up reference changes
			var pm = new Core.ScriptProjectManager(this);
			pm.GenerateInitScript();

			base.OnDeactivate(e);
		}

		protected override UIElement CreateChild()
		{
			return new LDrawVisualiserOptionsUI(this);
		}

		public event PropertyChangedEventHandler? PropertyChanged;
		private void NotifyPropertyChanged(string prop_name) => PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(prop_name));
	}
}
