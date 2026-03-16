using System.Windows;
using System.Windows.Controls;
using Rylogic.Gui.WPF;

namespace RyLogViewer
{
	/// <summary>Dockable panel for managing click actions</summary>
	public class ActionsPanel : DockPanel, IDockable
	{
		private readonly Main m_main;
		private readonly ListBox m_list;

		public ActionsPanel(Main main)
		{
			m_main = main;

			DockControl = new DockControl(this, "Actions")
			{
				TabText = "Actions",
			};

			// Toolbar
			var toolbar = new System.Windows.Controls.ToolBar();
			SetDock(toolbar, Dock.Top);
			Children.Add(toolbar);

			var add_btn = new Button { Content = "Add", Padding = new Thickness(8, 2, 8, 2) };
			add_btn.Click += (s, e) => AddAction();
			toolbar.Items.Add(add_btn);

			var edit_btn = new Button { Content = "Edit", Padding = new Thickness(8, 2, 8, 2) };
			edit_btn.Click += (s, e) => EditAction();
			toolbar.Items.Add(edit_btn);

			var remove_btn = new Button { Content = "Remove", Padding = new Thickness(8, 2, 8, 2) };
			remove_btn.Click += (s, e) => RemoveAction();
			toolbar.Items.Add(remove_btn);

			// Action list
			m_list = new ListBox
			{
				ItemsSource = m_main.Actions,
			};
			Children.Add(m_list);
		}

		/// <summary>IDockable implementation</summary>
		public DockControl DockControl { get; }

		/// <summary>Add a new action</summary>
		private void AddAction()
		{
			var dlg = new ClickActionEditUI { Owner = Window.GetWindow(this) };
			if (dlg.ShowDialog() == true && dlg.Result != null)
				m_main.Actions.Add(dlg.Result);
		}

		/// <summary>Edit the selected action</summary>
		private void EditAction()
		{
			if (m_list.SelectedItem is not ClickAction action) return;
			var dlg = new ClickActionEditUI(action) { Owner = Window.GetWindow(this) };
			if (dlg.ShowDialog() == true && dlg.Result != null)
			{
				var idx = m_list.SelectedIndex;
				m_main.Actions[idx] = dlg.Result;
			}
		}

		/// <summary>Remove the selected action</summary>
		private void RemoveAction()
		{
			if (m_list.SelectedItem is not ClickAction action) return;
			m_main.Actions.Remove(action);
		}
	}
}
