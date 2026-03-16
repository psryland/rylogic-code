using System;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using Rylogic.Common;
using Rylogic.Gui.WPF;

namespace RyLogViewer
{
	/// <summary>Dockable panel for searching log content</summary>
	public class FindPanel : DockPanel, IDockable
	{
		private readonly Main m_main;
		private readonly TextBox m_search_box;
		private readonly ComboBox m_search_mode;
		private readonly CheckBox m_case_sensitive;
		private int m_last_found_index = -1;

		public FindPanel(Main main)
		{
			m_main = main;

			DockControl = new DockControl(this, "Find")
			{
				TabText = "Find",
			};

			// Search input row
			var input_panel = new DockPanel { Margin = new Thickness(4) };
			SetDock(input_panel, Dock.Top);
			Children.Add(input_panel);

			// Search mode dropdown
			m_search_mode = new ComboBox
			{
				Width = 100,
				Margin = new Thickness(0, 0, 4, 0),
			};
			m_search_mode.Items.Add("Substring");
			m_search_mode.Items.Add("Wildcard");
			m_search_mode.Items.Add("Regex");
			m_search_mode.SelectedIndex = 0;
			DockPanel.SetDock(m_search_mode, Dock.Left);
			input_panel.Children.Add(m_search_mode);

			// Search text
			m_search_box = new TextBox { Margin = new Thickness(0, 0, 4, 0) };
			m_search_box.KeyDown += (s, e) => { if (e.Key == Key.Enter) FindNext(); };
			input_panel.Children.Add(m_search_box);

			// Buttons row
			var btn_panel = new WrapPanel { Margin = new Thickness(4, 2, 4, 4) };
			SetDock(btn_panel, Dock.Top);
			Children.Add(btn_panel);

			var find_next_btn = new Button { Content = "Find Next", Padding = new Thickness(8, 2, 8, 2), Margin = new Thickness(0, 0, 4, 0) };
			find_next_btn.Click += (s, e) => FindNext();
			btn_panel.Children.Add(find_next_btn);

			var find_prev_btn = new Button { Content = "Find Prev", Padding = new Thickness(8, 2, 8, 2), Margin = new Thickness(0, 0, 4, 0) };
			find_prev_btn.Click += (s, e) => FindPrev();
			btn_panel.Children.Add(find_prev_btn);

			var bookmark_all_btn = new Button { Content = "Bookmark All", Padding = new Thickness(8, 2, 8, 2), Margin = new Thickness(0, 0, 4, 0) };
			bookmark_all_btn.Click += (s, e) => BookmarkAll();
			btn_panel.Children.Add(bookmark_all_btn);

			// Options row
			var opts_panel = new WrapPanel { Margin = new Thickness(4, 0, 4, 4) };
			SetDock(opts_panel, Dock.Top);
			Children.Add(opts_panel);

			m_case_sensitive = new CheckBox { Content = "Case Sensitive", Margin = new Thickness(0, 0, 8, 0) };
			opts_panel.Children.Add(m_case_sensitive);
		}

		/// <summary>IDockable implementation</summary>
		public DockControl DockControl { get; }

		/// <summary>Create a pattern from the current search settings</summary>
		private Pattern CreateSearchPattern()
		{
			var patn_type = m_search_mode.SelectedIndex switch
			{
				1 => EPattern.Wildcard,
				2 => EPattern.RegularExpression,
				_ => EPattern.Substring,
			};
			return new Pattern(patn_type, m_search_box.Text)
			{
				IgnoreCase = m_case_sensitive.IsChecked != true,
				Active = true,
			};
		}

		/// <summary>Find the next match after the current position</summary>
		public void FindNext()
		{
			if (string.IsNullOrEmpty(m_search_box.Text)) return;
			var pattern = CreateSearchPattern();
			if (!pattern.IsValid) return;

			var start = m_last_found_index + 1;
			for (var i = 0; i < m_main.Count; ++i)
			{
				var idx = (start + i) % m_main.Count;
				var text = m_main[idx].Value(0);
				if (!pattern.IsMatch(text)) continue;

				m_last_found_index = idx;
				FoundMatch?.Invoke(this, new FindMatchEventArgs(idx));
				return;
			}
		}

		/// <summary>Find the previous match before the current position</summary>
		public void FindPrev()
		{
			if (string.IsNullOrEmpty(m_search_box.Text)) return;
			var pattern = CreateSearchPattern();
			if (!pattern.IsValid) return;

			var start = m_last_found_index - 1;
			if (start < 0) start = m_main.Count - 1;
			for (var i = 0; i < m_main.Count; ++i)
			{
				var idx = (start - i + m_main.Count) % m_main.Count;
				var text = m_main[idx].Value(0);
				if (!pattern.IsMatch(text)) continue;

				m_last_found_index = idx;
				FoundMatch?.Invoke(this, new FindMatchEventArgs(idx));
				return;
			}
		}

		/// <summary>Bookmark all matching lines</summary>
		public void BookmarkAll()
		{
			if (string.IsNullOrEmpty(m_search_box.Text)) return;
			var pattern = CreateSearchPattern();
			if (!pattern.IsValid) return;

			for (var i = 0; i < m_main.Count; ++i)
			{
				var line = m_main[i];
				var text = line.Value(0);
				if (!pattern.IsMatch(text)) continue;
				m_main.Bookmarks.Toggle(i, line.FileByteRange.Beg, text);
			}
		}

		/// <summary>Raised when a match is found and the grid should scroll to it</summary>
		public event EventHandler<FindMatchEventArgs>? FoundMatch;
	}

	/// <summary>Event args for when a find match is located</summary>
	public class FindMatchEventArgs : EventArgs
	{
		public FindMatchEventArgs(int line_index)
		{
			LineIndex = line_index;
		}
		public int LineIndex { get; }
	}
}
