using System;
using System.ComponentModel;
using System.IO;
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

		/// <summary>Default script text shown when the tool window opens</summary>
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

		public override void ResetSettings()
		{
			DefaultScriptText = @"var b = new Builder();
// Access debug variables via 'vars', e.g.:
// b.Box(""obj1"").box(1,2,3).o2w(vars.my_object.m_o2w);
return b.ToString();";
			DefaultAddress = "localhost:1976";
			DefaultAutoRefresh = true;
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
				if (bool.TryParse((string?)root.Element(nameof(DefaultAutoRefresh)), out var auto_refresh))
					DefaultAutoRefresh = auto_refresh;
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
