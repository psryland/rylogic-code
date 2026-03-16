using System.Windows;
using System.Windows.Controls;
using Rylogic.Common;
using Rylogic.Gui.WPF;

namespace RyLogViewer
{
	/// <summary>Dockable panel for managing filter patterns</summary>
	public class FiltersPanel : DockPanel, IDockable
	{
		private readonly Main m_main;
		private readonly ListBox m_list;

		public FiltersPanel(Main main)
		{
			m_main = main;

			DockControl = new DockControl(this, "Filters")
			{
				TabText = "Filters",
			};

			// Toolbar with add/remove buttons
			var toolbar = new System.Windows.Controls.ToolBar();
			SetDock(toolbar, Dock.Top);
			Children.Add(toolbar);

			var add_btn = new Button { Content = "Add", Padding = new Thickness(8, 2, 8, 2) };
			add_btn.Click += (s, e) => AddFilter();
			toolbar.Items.Add(add_btn);

			var remove_btn = new Button { Content = "Remove", Padding = new Thickness(8, 2, 8, 2) };
			remove_btn.Click += (s, e) => RemoveFilter();
			toolbar.Items.Add(remove_btn);

			var edit_btn = new Button { Content = "Edit", Padding = new Thickness(8, 2, 8, 2) };
			edit_btn.Click += (s, e) => EditFilter();
			toolbar.Items.Add(edit_btn);

			// List of filters
			m_list = new ListBox
			{
				ItemsSource = m_main.Filters,
				DisplayMemberPath = nameof(Filter.Expr),
			};
			Children.Add(m_list);
		}

		/// <summary>IDockable implementation</summary>
		public DockControl DockControl { get; }

		/// <summary>Add a new filter pattern</summary>
		private void AddFilter()
		{
			var ft = new Filter
			{
				Expr = "",
				PatnType = EPattern.Substring,
				Active = true,
				IfMatch = EIfMatch.Keep,
			};

			var dlg = new PatternEditorUI { Owner = Window.GetWindow(this) };
			dlg.Editor.NewPattern(ft);
			if (dlg.ShowDialog() == true)
			{
				var pat = dlg.Editor.Pattern;
				ft.Expr = pat.Expr;
				ft.PatnType = pat.PatnType;
				ft.IgnoreCase = pat.IgnoreCase;
				ft.WholeLine = pat.WholeLine;
				ft.Invert = pat.Invert;
				m_main.Filters.Add(ft);
			}
		}

		/// <summary>Edit the selected filter</summary>
		private void EditFilter()
		{
			if (m_list.SelectedItem is not Filter ft) return;

			var dlg = new PatternEditorUI { Owner = Window.GetWindow(this) };
			dlg.Editor.EditPattern(ft, clone: true);
			if (dlg.ShowDialog() == true)
			{
				var pat = dlg.Editor.Pattern;
				ft.Expr = pat.Expr;
				ft.PatnType = pat.PatnType;
				ft.IgnoreCase = pat.IgnoreCase;
				ft.WholeLine = pat.WholeLine;
				ft.Invert = pat.Invert;
				m_main.Filters.NotifyItemChanged();
			}
		}

		/// <summary>Remove the selected filter</summary>
		private void RemoveFilter()
		{
			if (m_list.SelectedItem is not Filter ft) return;
			m_main.Filters.Remove(ft);
		}
	}
}
