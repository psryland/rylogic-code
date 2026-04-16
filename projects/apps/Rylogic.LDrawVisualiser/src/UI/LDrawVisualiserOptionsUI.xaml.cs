using System;
using System.Linq;
using System.Windows.Controls;

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
			m_auto_compile_on_save_check.IsChecked = m_options.AutoCompileOnSave;

			// Populate reference assemblies (one per line)
			m_reference_assemblies_text.Text = string.Join(Environment.NewLine, m_options.ReferenceAssemblies);

			// Update options on change
			m_script_text.TextChanged += (s, e) => m_options.DefaultScriptText = m_script_text.Text;
			m_address_text.TextChanged += (s, e) => m_options.DefaultAddress = m_address_text.Text;
			m_auto_refresh_check.Checked += (s, e) => m_options.DefaultAutoRefresh = true;
			m_auto_refresh_check.Unchecked += (s, e) => m_options.DefaultAutoRefresh = false;
			m_auto_compile_on_save_check.Checked += (s, e) => m_options.AutoCompileOnSave = true;
			m_auto_compile_on_save_check.Unchecked += (s, e) => m_options.AutoCompileOnSave = false;
			m_reference_assemblies_text.TextChanged += (s, e) =>
			{
				m_options.ReferenceAssemblies = m_reference_assemblies_text.Text
					.Split(new[] { '\r', '\n' }, StringSplitOptions.RemoveEmptyEntries)
					.Select(line => line.Trim())
					.Where(line => line.Length != 0)
					.ToList();
			};
		}
	}
}
