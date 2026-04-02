using System.Windows;
using System.Windows.Controls;
using Microsoft.Win32;

namespace Rylogic.LDrawVisualiser
{
	public partial class LDrawVisualiserOptionsUI : UserControl
	{
		private readonly LDrawVisualiserOptions m_options;

		public LDrawVisualiserOptionsUI(LDrawVisualiserOptions options)
		{
			InitializeComponent();
			m_options = options;

			// Bind to current values
			m_script_text.Text = m_options.DefaultScriptText;
			m_address_text.Text = m_options.DefaultAddress;
			m_auto_refresh_check.IsChecked = m_options.DefaultAutoRefresh;
			m_namespaces_text.Text = m_options.Namespaces;

			// Populate assemblies list
			foreach (var asm in m_options.Assemblies)
				m_assemblies_list.Items.Add(asm);

			// Update options on change
			m_script_text.TextChanged += (s, e) => m_options.DefaultScriptText = m_script_text.Text;
			m_address_text.TextChanged += (s, e) => m_options.DefaultAddress = m_address_text.Text;
			m_auto_refresh_check.Checked += (s, e) => m_options.DefaultAutoRefresh = true;
			m_auto_refresh_check.Unchecked += (s, e) => m_options.DefaultAutoRefresh = false;
			m_namespaces_text.TextChanged += (s, e) => m_options.Namespaces = m_namespaces_text.Text;
		}

		private void OnAddAssembly(object sender, RoutedEventArgs e)
		{
			var dlg = new OpenFileDialog
			{
				Title = "Select .NET Assembly",
				Filter = ".NET Assemblies (*.dll)|*.dll|All Files (*.*)|*.*",
				Multiselect = true,
			};
			if (dlg.ShowDialog() != true)
				return;

			foreach (var filepath in dlg.FileNames)
			{
				if (!m_options.Assemblies.Contains(filepath))
				{
					m_options.Assemblies.Add(filepath);
					m_assemblies_list.Items.Add(filepath);
				}
			}
		}

		private void OnRemoveAssembly(object sender, RoutedEventArgs e)
		{
			if (m_assemblies_list.SelectedItem is not string selected)
				return;

			m_options.Assemblies.Remove(selected);
			m_assemblies_list.Items.Remove(selected);
		}
	}
}
