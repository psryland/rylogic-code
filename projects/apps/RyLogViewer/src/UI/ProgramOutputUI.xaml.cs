using System;
using System.IO;
using System.Windows;
using Microsoft.Win32;
using Rylogic.Gui.WPF;

namespace RyLogViewer
{
	public partial class ProgramOutputUI : Window
	{
		private readonly LaunchApp[] m_history_items;

		public ProgramOutputUI(LaunchApp[] history)
		{
			InitializeComponent();
			m_history_items = history ?? Array.Empty<LaunchApp>();
			Result = null;

			// Populate history dropdown
			foreach (var item in m_history_items)
				m_history.Items.Add(item.Executable);
		}

		/// <summary>The launch configuration, set when the user clicks Launch</summary>
		public LaunchApp? Result { get; private set; }

		/// <summary>Browse for an executable</summary>
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

		/// <summary>Browse for a working directory</summary>
		private void BrowseWorkDir_Click(object sender, RoutedEventArgs e)
		{
			// Use OpenFolderDialog (available in .NET 8+ WPF)
			var dlg = new OpenFolderDialog
			{
				Title = "Select Working Directory",
				InitialDirectory = m_working_dir.Text,
			};
			if (dlg.ShowDialog(this) == true)
				m_working_dir.Text = dlg.FolderName;
		}

		/// <summary>Browse for an output file</summary>
		private void BrowseOutputFile_Click(object sender, RoutedEventArgs e)
		{
			var fd = new SaveFileDialog
			{
				Title = "Output File",
				Filter = "Log Files (*.log;*.txt)|*.log;*.txt|All Files (*.*)|*.*",
			};
			if (fd.ShowDialog(this) == true)
				m_output_file.Text = fd.FileName;
		}

		/// <summary>Select a history item</summary>
		private void History_SelectionChanged(object sender, System.Windows.Controls.SelectionChangedEventArgs e)
		{
			var idx = m_history.SelectedIndex;
			if (idx < 0 || idx >= m_history_items.Length) return;

			var item = m_history_items[idx];
			m_exe.Text = item.Executable;
			m_args.Text = item.Arguments;
			m_working_dir.Text = item.WorkingDirectory;
			m_output_file.Text = item.OutputFilepath;
			m_capture_stdout.IsChecked = item.CaptureStdout;
			m_capture_stderr.IsChecked = item.CaptureStderr;
			m_show_window.IsChecked = item.ShowWindow;
			m_append.IsChecked = item.AppendOutputFile;
		}

		/// <summary>Launch the configured process</summary>
		private void Launch_Click(object sender, RoutedEventArgs e)
		{
			if (string.IsNullOrWhiteSpace(m_exe.Text))
			{
				MessageBox.Show(this, "Please specify an executable.", "Program Output", MessageBoxButton.OK, MessageBoxImage.Warning);
				return;
			}

			Result = new LaunchApp
			{
				Executable = m_exe.Text.Trim(),
				Arguments = m_args.Text.Trim(),
				WorkingDirectory = m_working_dir.Text.Trim(),
				OutputFilepath = m_output_file.Text.Trim(),
				CaptureStdout = m_capture_stdout.IsChecked == true,
				CaptureStderr = m_capture_stderr.IsChecked == true,
				ShowWindow = m_show_window.IsChecked == true,
				AppendOutputFile = m_append.IsChecked == true,
			};
			DialogResult = true;
		}
	}
}
