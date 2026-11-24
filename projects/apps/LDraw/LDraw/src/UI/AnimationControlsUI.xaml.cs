using System;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Threading;
using Rylogic.Gui.WPF.DockContainerDetail;
using Rylogic.Gui.WPF;
using Rylogic.Utility;

namespace LDraw.UI
{
	/// <summary>Dockable animation controls for the active scene</summary>
	public partial class AnimationControlsUI : UserControl, IDockable, IDisposable
	{
		public AnimationControlsUI()
		{
			// Create the dock control. The panel docks at the bottom by default.
			// The docked size is kept in pixels so that the controls do not grow or shrink when the window is resized.
			InitializeComponent();
			DockControl = new DockControl(this, "AnimationControls")
			{
				ShowTitle = false,
				TabText = "Animation",
				DestroyOnClose = false,
				DefaultDockLocation = new DockContainer.DockLocation(new[] { EDockSite.Bottom }),
				ResizeMode = EDockResizeMode.Absolute,
				FloatingSizeToContent = true,
			};
		}
		public void Dispose()
		{
			// Detach from the scene and release the dock control
			m_anim_controls.ViewWindow = null;
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

		/// <summary>The scene whose animation is controlled (null if none)</summary>
		public SceneUI? Scene { get; private set; }

		/// <summary>Resize the docked pane so that its height fits the animation controls, once the pane has been laid out</summary>
		public void FitDockedHeight()
		{
			// Wait for the layout pass so that the pane's title bar and tab strip have their final sizes
			Dispatcher.BeginInvoke(DispatcherPriority.Loaded, () =>
			{
				// Nothing to fit if the controls are not docked
				if (DockControl.DockPane is not DockPane pane)
					return;

				// Retry once the pane has been displayed
				if (!pane.IsLoaded || pane.ActualHeight == 0 || ActualHeight == 0)
				{
					pane.Loaded += OnPaneLoaded;
					return;
				}

				// The pane height is its chrome (title bar, tabs) plus the height the controls need at the current width
				var chrome = pane.ActualHeight - ActualHeight;
				m_anim_controls.Measure(new Size(ActualWidth, double.PositiveInfinity));
				pane.Height = chrome + m_anim_controls.DesiredSize.Height;
			});
			void OnPaneLoaded(object sender, RoutedEventArgs e)
			{
				// Fit the height now that the pane has a layout
				((DockPane)sender).Loaded -= OnPaneLoaded;
				FitDockedHeight();
			}
		}

		/// <summary>Control the animation of the given scene</summary>
		public void SetScene(SceneUI? scene)
		{
			// Only update when the scene changes, so that the control state is not reset unnecessarily
			if (ReferenceEquals(scene, Scene))
				return;

			Scene = scene;
			m_anim_controls.ViewWindow = scene?.SceneView?.Scene?.Window;
		}
	}
}
