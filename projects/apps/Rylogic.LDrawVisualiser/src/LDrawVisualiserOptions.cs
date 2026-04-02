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

		/// <summary>Additional assembly paths to reference when compiling scripts</summary>
		public List<string> Assemblies
		{
			get => m_assemblies;
			set
			{
				m_assemblies = value;
				NotifyPropertyChanged(nameof(Assemblies));
			}
		}
		private List<string> m_assemblies = new();

		/// <summary>Additional using namespaces (newline-delimited) to include in compiled scripts</summary>
		public string Namespaces
		{
			get => m_namespaces;
			set
			{
				if (m_namespaces == value) return;
				m_namespaces = value;
				NotifyPropertyChanged(nameof(Namespaces));
			}
		}
		private string m_namespaces = "";

		public override void ResetSettings()
		{
			DefaultScriptText = @"var b = new Builder();
// Access debug variables via 'vars', e.g.:
// b.Box(""obj1"").box(1,2,3).o2w(vars.my_object.m_o2w);
return b.ToString();";
			DefaultAddress = "localhost:1976";
			DefaultAutoRefresh = true;
			LastSelectedScript = "Script";
			Assemblies = new List<string>();
			Namespaces = "";
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
				Namespaces = (string?)root.Element(nameof(Namespaces)) ?? Namespaces;
				if (bool.TryParse((string?)root.Element(nameof(DefaultAutoRefresh)), out var auto_refresh))
					DefaultAutoRefresh = auto_refresh;

				var assemblies_el = root.Element(nameof(Assemblies));
				if (assemblies_el != null)
					Assemblies = assemblies_el.Elements("Assembly").Select(e => e.Value).Where(s => !string.IsNullOrWhiteSpace(s)).ToList();
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
				root.Add(new System.Xml.Linq.XElement(nameof(LastSelectedScript), LastSelectedScript));
				root.Add(new System.Xml.Linq.XElement(nameof(Namespaces), Namespaces));
				root.Add(new System.Xml.Linq.XElement(nameof(Assemblies),
					Assemblies.Select(a => new System.Xml.Linq.XElement("Assembly", a))));
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
