using System;
using System.ComponentModel;
using System.Windows.Controls;
using Rylogic.Gfx;
using Rylogic.Gui.WPF;
using Rylogic.Utility;

namespace LDraw.UI
{
	public partial class ObjectManagerUI : UserControl, IDockable, IDisposable, INotifyPropertyChanged
	{
		public ObjectManagerUI()
		{
			InitializeComponent();
			DockControl = new DockControl(this, "ObjectManager")
			{
				ShowTitle = false,
				TabText = "Scene Manager",
				DestroyOnClose = false,
			};
			DataContext = this;
		}
		public void Dispose()
		{
			ManagerUI = null;
			DockControl = null!;
			GC.SuppressFinalize(this);
		}

		/// <summary>Provides support for the DockContainer</summary>
		public DockControl DockControl
		{
			get;
			private set
			{
				if (field == value) return;
				Util.Dispose(ref field!);
				field = value;
			}
		} = null!;

		/// <summary>The current View3dObjectManagerUI instance</summary>
		private View3dObjectManagerUI? ManagerUI
		{
			get;
			set
			{
				if (field == value) return;
				if (field != null)
				{
					m_root.Children.Remove(field);
					field.Dispose();
				}
				field = value;
				if (field != null)
				{
					m_root.Children.Add(field);
				}
			}
		}

		/// <summary>Update the object manager content for the given scene</summary>
		public void SetScene(SceneUI? scene)
		{
			if (scene?.SceneView?.Scene?.Window is View3d.Window window)
				ManagerUI = new View3dObjectManagerUI(window);
			else
				ManagerUI = null;
		}

		/// <inheritdoc/>
		public event PropertyChangedEventHandler? PropertyChanged;
		private void NotifyPropertyChanged(string prop_name) => PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(prop_name));
	}
}
