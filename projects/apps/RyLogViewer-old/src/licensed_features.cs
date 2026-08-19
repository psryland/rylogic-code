using System;
using System.Collections.Generic;
using System.Windows.Forms;
using Rylogic.Extn;
using Rylogic.Gui.WinForms;

namespace RyLogViewer
{
	public sealed partial class Main
	{
		/// <summary>The collection of licensed features in use</summary>
		private readonly Dictionary<string, ILicensedFeature> m_licensed_features = new Dictionary<string, ILicensedFeature>();

		/// <summary>Call to begin or update a time limited feature</summary>
		public void UseLicensedFeature(string key, ILicensedFeature use)
		{
			if (Licence.Valid)
				return;

			// Wait till the main UI is visible
			if (!Visible)
				return;

			// See if the feature is already monitored, if so, just update the interface
			if (m_licensed_features.ContainsKey(key))
			{
				m_licensed_features[key] = use;
				return;
			}

			// If the feature isn't in use, ignore the request (this method is called recursively)
			if (!use.FeatureInUse)
				return;

			// Otherwise, prompt about the feature
			var dlg = new CrippleUI(use.FeatureDescription);
			if (dlg.ShowDialog(this) != DialogResult.OK)
			{
				// Not interested, close and go home
				use.CloseFeature();
				return;
			}

			// Otherwise, the user wants to use the feature
			m_licensed_features[key] = use;
			SynchronizationContext_.BeginInvokeDelayed(() =>
			{
				ILicensedFeature feat;
				if (!m_licensed_features.TryGetValue(key, out feat)) return;
				m_licensed_features.Remove(key);
				UseLicensedFeature(key, feat);
			}, TimeSpan.FromMinutes(FreeEditionLimits.FeatureEnableTimeInMinutes));
		}
	}

	/// <summary>Interface for controlling time limited features</summary>
	public interface ILicensedFeature
	{
		/// <summary>An html description of the licensed feature</summary>
		string FeatureDescription { get; }

		/// <summary>True if the licensed feature is still currently in use</summary>
		bool FeatureInUse { get; }

		/// <summary>Called to stop the use of the feature</summary>
		void CloseFeature();
	}

	/// <summary>Limits the number of highlighting patterns in use</summary>
	public class HighlightingCountLimiter :ILicensedFeature
	{
		private readonly Main m_main;
		public HighlightingCountLimiter(Main main)
		{
			m_main = main;
		}

		/// <summary>An html description of the licensed feature</summary>
		public string FeatureDescription
		{
			get { return Resources.cripple_highlighting; }
		}

		/// <summary>True if the licensed feature is still currently in use</summary>
		public virtual bool FeatureInUse
		{
			get { return m_main.Settings.Patterns.Highlights.Count > FreeEditionLimits.MaxHighlights; }
		}

		/// <summary>Called to stop the use of the feature</summary>
		public virtual void CloseFeature()
		{
			// Read the patterns from the settings to see if more than the max allowed are in use
			m_main.Settings.Patterns.Highlights.RemoveToEnd(FreeEditionLimits.MaxHighlights);
			m_main.Settings.Save();
		}
	}

	/// <summary>Limits the number of highlighting patterns in use</summary>
	public class FilteringCountLimiter :ILicensedFeature
	{
		private readonly Main m_main;
		public FilteringCountLimiter(Main main)
		{
			m_main = main;
		}

		/// <summary>An html description of the licensed feature</summary>
		public string FeatureDescription
		{
			get { return Resources.cripple_filtering; }
		}

		/// <summary>True if the licensed feature is still currently in use</summary>
		public virtual bool FeatureInUse
		{
			get { return m_main.Settings.Patterns.Filters.Count > FreeEditionLimits.MaxFilters; }
		}

		/// <summary>Called to stop the use of the feature</summary>
		public virtual void CloseFeature()
		{
			// Read the patterns from the settings to see if more than the max allowed are in use
			m_main.Settings.Patterns.Filters.RemoveToEnd(FreeEditionLimits.MaxFilters);
			m_main.Settings.Save();
		}
	}

	/// <summary>Limits the use of aggregate files</summary>
	public class AggregateFileLimiter :ILicensedFeature
	{
		private readonly Main m_main;
		private readonly AggregateFilesUI m_ui;
		public AggregateFileLimiter(Main main, AggregateFilesUI ui)
		{
			m_main = main;
			m_ui = ui;
		}

		/// <summary>An html description of the licensed feature</summary>
		public string FeatureDescription
		{
			get { return Resources.cripple_aggregate_files; }
		}

		/// <summary>True if the licensed feature is still currently in use</summary>
		public bool FeatureInUse
		{
			get
			{
				return
					m_main != null && m_main.Src is AggregateFile ||
					m_ui != null && m_ui.Visible;
			}
		}

		/// <summary>Called to stop the use of the feature</summary>
		public void CloseFeature()
		{
			if (m_main != null && m_main.Src is AggregateFile)
				m_main.Src = null;
			if (m_ui != null && m_ui.Visible)
				m_ui.Close();
		}
	}
}
