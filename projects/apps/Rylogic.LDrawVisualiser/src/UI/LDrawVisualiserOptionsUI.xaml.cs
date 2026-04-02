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

			// Update options on change
			m_script_text.TextChanged += (s, e) => m_options.DefaultScriptText = m_script_text.Text;
			m_address_text.TextChanged += (s, e) => m_options.DefaultAddress = m_address_text.Text;
			m_auto_refresh_check.Checked += (s, e) => m_options.DefaultAutoRefresh = true;
			m_auto_refresh_check.Unchecked += (s, e) => m_options.DefaultAutoRefresh = false;
		}
	}
}
