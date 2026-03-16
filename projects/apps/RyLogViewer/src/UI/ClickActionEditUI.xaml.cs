using System.Windows;
using Microsoft.Win32;
using Rylogic.Common;

namespace RyLogViewer
{
	public partial class ClickActionEditUI : Window
	{
		public ClickActionEditUI()
			: this(null)
		{ }
		public ClickActionEditUI(ClickAction? existing)
		{
			InitializeComponent();
			Result = null;

			if (existing != null)
			{
				m_type.SelectedIndex = existing.PatnType switch
				{
					EPattern.Wildcard => 1,
					EPattern.RegularExpression => 2,
					_ => 0,
				};
				m_match.Text = existing.Expr;
				m_exe.Text = existing.Executable;
				m_args.Text = existing.Arguments;
				m_working_dir.Text = existing.WorkingDirectory;
				m_active.IsChecked = existing.Active;
			}
			else
			{
				m_type.SelectedIndex = 2;
			}
		}

		/// <summary>The resulting action, set when the user clicks OK</summary>
		public ClickAction? Result { get; private set; }

		private void BrowseExe_Click(object sender, RoutedEventArgs e)
		{
			var fd = new OpenFileDialog
			{
				Title = "Select Executable",
				Filter = "Executables (*.exe)|*.exe|All Files (*.*)|*.*",
				CheckFileExists = true,
			};
			if (fd.ShowDialog(this) == true)
				m_exe.Text = fd.FileName;
		}

		private void BrowseDir_Click(object sender, RoutedEventArgs e)
		{
			var dlg = new OpenFolderDialog
			{
				Title = "Select Working Directory",
				InitialDirectory = m_working_dir.Text,
			};
			if (dlg.ShowDialog(this) == true)
				m_working_dir.Text = dlg.FolderName;
		}

		private void OK_Click(object sender, RoutedEventArgs e)
		{
			var patn_type = m_type.SelectedIndex switch
			{
				1 => EPattern.Wildcard,
				2 => EPattern.RegularExpression,
				_ => EPattern.Substring,
			};
			Result = new ClickAction(patn_type, m_match.Text)
			{
				Executable = m_exe.Text.Trim(),
				Arguments = m_args.Text.Trim(),
				WorkingDirectory = m_working_dir.Text.Trim(),
				Active = m_active.IsChecked == true,
			};
			DialogResult = true;
		}
	}
}
