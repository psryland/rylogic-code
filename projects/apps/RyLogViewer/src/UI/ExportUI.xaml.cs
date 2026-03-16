using System;
using System.IO;
using System.Text;
using System.Windows;
using Microsoft.Win32;

namespace RyLogViewer
{
	public partial class ExportUI : Window
	{
		private readonly Main m_main;
		private readonly Options.Settings m_settings;

		public ExportUI(Main main, Options.Settings settings)
		{
			InitializeComponent();
			m_main = main;
			m_settings = settings;

			// Pre-populate from settings
			m_output_file.Text = settings.ExportFilepath;
			m_row_delim.Text = @"\r\n";
			m_col_delim.Text = @",";
		}

		private void BrowseOutput_Click(object sender, RoutedEventArgs e)
		{
			var fd = new SaveFileDialog
			{
				Title = "Export To",
				Filter = "CSV Files (*.csv)|*.csv|Text Files (*.txt)|*.txt|All Files (*.*)|*.*",
			};
			if (fd.ShowDialog(this) == true)
				m_output_file.Text = fd.FileName;
		}

		private void Export_Click(object sender, RoutedEventArgs e)
		{
			var filepath = m_output_file.Text.Trim();
			if (string.IsNullOrEmpty(filepath))
			{
				MessageBox.Show(this, "Please specify an output file.", "Export", MessageBoxButton.OK, MessageBoxImage.Warning);
				return;
			}

			try
			{
				m_progress.Visibility = Visibility.Visible;
				m_progress.Maximum = m_main.Count;
				m_progress.Value = 0;

				// Unescape delimiters
				var row_delim = Unescape(m_row_delim.Text);
				var col_delim = Unescape(m_col_delim.Text);
				var apply_transforms = m_apply_transforms.IsChecked == true;

				using var writer = new StreamWriter(filepath, false, Encoding.UTF8);
				for (var i = 0; i != m_main.Count; ++i)
				{
					var line = m_main[i];
					var text = line.Value(0);

					// Apply transforms if requested
					if (apply_transforms)
					{
						foreach (var tx in m_main.Transforms)
							text = tx.Txfm(text);
					}

					writer.Write(text);
					writer.Write(row_delim);
					m_progress.Value = i + 1;
				}

				// Save the export filepath for next time
				m_settings.ExportFilepath = filepath;

				MessageBox.Show(this, $"Exported {m_main.Count} lines to {filepath}", "Export Complete", MessageBoxButton.OK, MessageBoxImage.Information);
				DialogResult = true;
			}
			catch (Exception ex)
			{
				MessageBox.Show(this, $"Export failed: {ex.Message}", "Export Error", MessageBoxButton.OK, MessageBoxImage.Error);
			}
			finally
			{
				m_progress.Visibility = Visibility.Collapsed;
			}
		}

		/// <summary>Unescape common C-style escape sequences</summary>
		private static string Unescape(string s)
		{
			return s
				.Replace(@"\r\n", "\r\n")
				.Replace(@"\n", "\n")
				.Replace(@"\r", "\r")
				.Replace(@"\t", "\t");
		}
	}
}
