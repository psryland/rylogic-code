using System.Windows;
using System.Windows.Input;
using Microsoft.VisualStudio.PlatformUI;

namespace Rylogic.LDrawVisualiser
{
	public partial class RenameScriptDialog : DialogWindow
	{
		public RenameScriptDialog(string current_name)
		{
			InitializeComponent();
			m_name_box.Text = current_name;
			m_name_box.SelectAll();
			m_name_box.Focus();
		}

		/// <summary>The new name entered by the user (without extension)</summary>
		public string NewName => m_name_box.Text.Trim();

		private void OnOkClick(object sender, RoutedEventArgs e)
		{
			DialogResult = true;
		}

		private void OnKeyDown(object sender, KeyEventArgs e)
		{
			if (e.Key == Key.Enter && !string.IsNullOrWhiteSpace(m_name_box.Text))
			{
				DialogResult = true;
				e.Handled = true;
			}
		}
	}
}
