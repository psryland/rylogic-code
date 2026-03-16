using System;
using System.Windows;
using System.Windows.Controls;
using Rylogic.Gui.WPF;

namespace RyLogViewer
{
	/// <summary>Dockable panel for managing bookmarks</summary>
	public class BookmarksPanel : DockPanel, IDockable
	{
		private readonly Main m_main;
		private readonly ListBox m_list;

		public BookmarksPanel(Main main)
		{
			m_main = main;

			DockControl = new DockControl(this, "Bookmarks")
			{
				TabText = "Bookmarks",
			};

			// Toolbar
			var toolbar = new System.Windows.Controls.ToolBar();
			SetDock(toolbar, Dock.Top);
			Children.Add(toolbar);

			var prev_btn = new Button { Content = "◀ Prev", Padding = new Thickness(8, 2, 8, 2) };
			prev_btn.Click += (s, e) => NavigatePrev();
			toolbar.Items.Add(prev_btn);

			var next_btn = new Button { Content = "Next ▶", Padding = new Thickness(8, 2, 8, 2) };
			next_btn.Click += (s, e) => NavigateNext();
			toolbar.Items.Add(next_btn);

			toolbar.Items.Add(new Separator());

			var clear_btn = new Button { Content = "Clear All", Padding = new Thickness(8, 2, 8, 2) };
			clear_btn.Click += (s, e) => m_main.Bookmarks.Clear();
			toolbar.Items.Add(clear_btn);

			// Bookmark list
			m_list = new ListBox
			{
				ItemsSource = m_main.Bookmarks,
			};
			m_list.MouseDoubleClick += (s, e) =>
			{
				if (m_list.SelectedItem is Bookmark bm)
					NavigateToBookmark?.Invoke(this, new FindMatchEventArgs(bm.LineIndex));
			};
			Children.Add(m_list);
		}

		/// <summary>IDockable implementation</summary>
		public DockControl DockControl { get; }

		/// <summary>Navigate to the next bookmark</summary>
		private void NavigateNext()
		{
			var current = (m_list.SelectedItem as Bookmark)?.LineIndex ?? -1;
			var bm = m_main.Bookmarks.FindNext(current);
			if (bm != null)
			{
				m_list.SelectedItem = bm;
				NavigateToBookmark?.Invoke(this, new FindMatchEventArgs(bm.LineIndex));
			}
		}

		/// <summary>Navigate to the previous bookmark</summary>
		private void NavigatePrev()
		{
			var current = (m_list.SelectedItem as Bookmark)?.LineIndex ?? m_main.Count;
			var bm = m_main.Bookmarks.FindPrev(current);
			if (bm != null)
			{
				m_list.SelectedItem = bm;
				NavigateToBookmark?.Invoke(this, new FindMatchEventArgs(bm.LineIndex));
			}
		}

		/// <summary>Raised when the user wants to navigate to a bookmark</summary>
		public event EventHandler<FindMatchEventArgs>? NavigateToBookmark;
	}
}
