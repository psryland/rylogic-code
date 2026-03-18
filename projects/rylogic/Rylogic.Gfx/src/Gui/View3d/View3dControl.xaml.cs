using System;
using System.ComponentModel;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Interop;
using System.Windows.Media;
using System.Windows.Threading;
using Rylogic.Extn;
using Rylogic.Gfx;
using Rylogic.Maths;
using Rylogic.Utility;
using Rylogic.Windows.Extn;

namespace Rylogic.Gui.WPF
{
	public partial class View3dControl :Image, IDisposable, INotifyPropertyChanged, IView3dCMenuContext
	{
		// Notes:
		//  - This control subclasses 'Image' because the D3DImage is an 'ImageSource'
		//  - View3dControl does not have a 'Settings' class, state changes are immediate
		//    and storing the state is left to the caller. (Unlike ChartControl).

		static View3dControl()
		{
			View3d.LoadDll(throw_if_missing: false);
		}
		public View3dControl()
		{
			InitializeComponent();
			Stretch = Stretch.Fill;
			StretchDirection = StretchDirection.Both;
			UseLayoutRounding = true;
			Focusable = true;

			if (DesignerProperties.GetIsInDesignMode(this))
				return;

			try
			{
				// Initialise View3d in off-screen only mode (i.e. no window handle)
				View3d = View3d.Create();
				Window = new View3d.Window(View3d, IntPtr.Zero);
				Window.Error += (s,a) => OnReportError(new ReportErrorEventArgs(a.Message));
				Camera.Lookat(new v4(0, 0, 10, 1), v4.Origin, v4.YAxis);
				Camera.ClipPlanes(0.01f, 1000f, View3d.EClipPlanes.CameraRelative);

				// Create a D3D11 off-screen render target image source
				Source = D3DImage = new Gfx.D3DImage(IntPtr.Zero, Window.BackBuffer.Dim, Window.DpiScale, 4);

				// Set defaults
				BackgroundColour = Window.BackgroundColour;
				DesiredPixelAspect = 1;
				ResolutionScale = 1;
				ClickTimeMS = 180;
				MouseNavigation = true;
				DefaultKeyboardShortcuts = true;
				ViewPreset = EViewPreset.Current;

				// Default context menu implementation
				if (ContextMenu != null && ContextMenu.DataContext == null)
					ContextMenu.DataContext = this;

				InitCommands();
				
				// Don't set 'DataContext = this'. Context should be whatever the containing
				// control's context is. E.g. ChartPanel.DataContext = ChartControl.DataContext
			}
			catch
			{
				Dispose();
				throw;
			}
		}
		public void Dispose()
		{
			Dispose(true);
			GC.SuppressFinalize(this);
		}
		protected virtual void Dispose(bool _)
		{
			Disposing?.Invoke(this, EventArgs.Empty);
			m_resize_timer?.Stop();
			Source = null;
			D3DImage = null!;
			Window = null!;
			View3d = null!;
		}
		protected override void OnRenderSizeChanged(SizeChangedInfo size_info)
		{
			base.OnRenderSizeChanged(size_info);

			var size_changed =
				Math.Abs(size_info.NewSize.Width - D3DImage.Width) > 1 ||
				Math.Abs(size_info.NewSize.Height - D3DImage.Height) > 1;

			if (size_changed)
			{
				// Defer render target recreation until resize stabilizes. During resize,
				// render at the current RT size and let WPF stretch the D3DImage to fill.
				if (m_resize_timer == null)
				{
					m_resize_timer = new DispatcherTimer { Interval = TimeSpan.FromMilliseconds(150) };
					m_resize_timer.Tick += HandleResizeCompleted;
				}
				m_resize_timer.Stop();
				m_resize_timer.Start();
			}

			Invalidate();
		}
		private void HandleResizeCompleted(object? sender, EventArgs e)
		{
			m_resize_timer?.Stop();
			m_resized = true;
			Invalidate();
		}
		private DispatcherTimer? m_resize_timer;
		protected override Size MeasureOverride(Size constraint)
		{
			var sz = base.MeasureOverride(constraint);

			// Scale up to pixels, round, then scale back to DIP
			var device_sz = sz.TransformToDevice(this);
			device_sz.Width  = Math.Round(device_sz.Width);
			device_sz.Height = Math.Round(device_sz.Height);
			return device_sz.TransformFromDevice(this);
		}
		protected override void OnMouseDown(MouseButtonEventArgs e)
		{
			Keyboard.Focus(this);
			base.OnMouseDown(e);
		}
		protected override void OnRender(DrawingContext dc)
		{
			// The control is rendering, update the source
			Render();
			base.OnRender(dc);
		}
		private bool m_resized;

		/// <summary>View3d context reference</summary>
		public View3d View3d
		{
			get;
			private set
			{
				if (field == value) return;
				Util.Dispose(ref field!);
				field = value;
			}
		} = null!;

		/// <summary>View3d window instance</summary>
		public View3d.Window Window
		{
			get;
			private set
			{
				if (Window == value) return;
				if (field != null)
				{
					field.OnInvalidated -= HandleInvalidated;
					field.OnSettingsChanged -= HandleSettingsChanged;
					Util.Dispose(ref field!);
				}
				field = value;
				if (field != null)
				{
					field.OnSettingsChanged += HandleSettingsChanged;
					field.OnInvalidated += HandleInvalidated;
				
					// We might have missed the first invalidate message
					m_render_pending = true;
				}

				// Handlers
				void HandleSettingsChanged(object? sender, View3d.SettingChangeEventArgs e)
				{
					if (ContextMenu?.DataContext is IView3dCMenu cmenu)
					{
						if (Bit.AllSet(e.Setting, View3d.ESettings.General))
						{
							if (Bit.AllSet(e.Setting, View3d.ESettings.General_OriginPointVisible))
								cmenu.NotifyPropertyChanged(nameof(IView3dCMenu.OriginPointVisible));
							if (Bit.AllSet(e.Setting, View3d.ESettings.General_FocusPointVisible))
								cmenu.NotifyPropertyChanged(nameof(IView3dCMenu.FocusPointVisible));
							if (Bit.AllSet(e.Setting, View3d.ESettings.General_SelectionBoxVisible))
								cmenu.NotifyPropertyChanged(nameof(IView3dCMenu.SelectionBoxVisible));
						}
						if (Bit.AllSet(e.Setting, View3d.ESettings.Scene))
						{
							if (Bit.AllSet(e.Setting, View3d.ESettings.Scene_BackgroundColour))
								cmenu.NotifyPropertyChanged(nameof(IView3dCMenu.BackgroundColour));
							if (Bit.AllSet(e.Setting, View3d.ESettings.Scene_Multisampling))
								cmenu.NotifyPropertyChanged(nameof(IView3dCMenu.Antialiasing));
							if (Bit.AllSet(e.Setting, View3d.ESettings.Scene_FillMode))
								cmenu.NotifyPropertyChanged(nameof(IView3dCMenu.FillMode));
							if (Bit.AllSet(e.Setting, View3d.ESettings.Scene_CullMode))
								cmenu.NotifyPropertyChanged(nameof(IView3dCMenu.CullMode));
						}
						if (Bit.AllSet(e.Setting, View3d.ESettings.Camera))
						{
							if (Bit.AllSet(e.Setting, View3d.ESettings.Camera_Orthographic))
								cmenu.NotifyPropertyChanged(nameof(IView3dCMenu.Orthographic));
							if (Bit.AllSet(e.Setting, View3d.ESettings.Camera_AlignAxis))
								cmenu.NotifyPropertyChanged(nameof(IView3dCMenu.AlignDirection));
						}
						if (Bit.AllSet(e.Setting, View3d.ESettings.Lighting))
						{
						}
						if (Bit.AllSet(e.Setting, View3d.ESettings.Diagnostics))
						{
							if (Bit.AllSet(e.Setting, View3d.ESettings.Diagnostics_BBoxesVisible))
								cmenu.NotifyPropertyChanged(nameof(IView3dCMenu.BBoxesVisible));
							if (Bit.AllSet(e.Setting, View3d.ESettings.Diagnostics_NormalsLength))
								cmenu.NotifyPropertyChanged(nameof(IView3dCMenu.NormalsLength));
							if (Bit.AllSet(e.Setting, View3d.ESettings.Diagnostics_NormalsColour))
								cmenu.NotifyPropertyChanged(nameof(IView3dCMenu.NormalsColour));
							if (Bit.AllSet(e.Setting, View3d.ESettings.Diagnostics_FillModePointsSize))
								cmenu.NotifyPropertyChanged(nameof(IView3dCMenu.FillModePointsSize));
						}
					}
					if (Bit.AllSet(e.Setting, View3d.ESettings.Camera_AlignAxis))
					{
						AlignDirection = AlignDirection_.FromAxis(Camera.AlignAxis);
						NotifyPropertyChanged(nameof(AlignDirection));
					}
					if (Bit.AllSet(e.Setting, View3d.ESettings.Scene_BackgroundColour))
					{
						BackgroundColour = Window.BackgroundColour;
						NotifyPropertyChanged(nameof(BackgroundColour));
					}
				}
				void HandleInvalidated(object? sender, EventArgs e)
				{
					if (m_render_pending) return;
					m_render_pending = true;
					InvalidateVisual();
				}
			}
		} = null!;

		/// <summary>The camera used to view the scene</summary>
		public View3d.Camera Camera => Window.Camera;

		/// <summary>The D3D render target texture</summary>
		public View3d.Texture? RenderTarget => D3DImage.FrontBuffer;

		/// <summary>An interop object providing an off-screen render target</summary>
		private Gfx.D3DImage D3DImage
		{
			get;
			set
			{
				if (D3DImage == value) return;
				if (field != null)
				{
					Loaded -= OnLoaded;
					Unloaded -= OnUnloaded;
					field.FrontBufferChanged -= HandleFrontBufferChanged;
					Util.Dispose(ref field!);
				}
				field = value;
				if (field != null)
				{
					field.FrontBufferChanged += HandleFrontBufferChanged;
					Unloaded += OnUnloaded;
					Loaded += OnLoaded;
				}

				void OnLoaded(object? sender, EventArgs arg)
				{
					// When the control is loaded, attach the main win.
					// Detach when unloaded (not reliable though)
					D3DImage.WindowOwner = System.Windows.Window.GetWindow(this).Hwnd();
					Invalidate();
				}
				void OnUnloaded(object? sender, EventArgs arg)
				{
					// When the control is unloaded, detach the window
					D3DImage.WindowOwner = IntPtr.Zero;
				}
				void HandleFrontBufferChanged(object? sender, EventArgs arg)
				{
					if (field.FrontBuffer != null)
					{
						var bb_size = field.RequiredBackBufferSize;
						Window.CustomSwapChain([field.FrontBuffer]); // This updates the MSAA buffer too
					}
					else
					{
						Window.CustomSwapChain([]);
					}

					OnRenderTargetChanged();
				}
			}
		} = null!;

		/// <summary>Trigger a redraw of the view3d scene</summary>
		public void Invalidate()
		{
			Window?.Invalidate();
		}

		/// <summary>The render target multi-sampling</summary>
		public int MultiSampling
		{
			get => D3DImage.MultiSampling;
			set
			{
				// Since, in WPF, we're rendering to an off-screen render target, multi-sampling
				// is done by changing the render target size, not the Dx11 multi-sampling settings.
				// This means the normal View3d.Window multi-sampling setting isn't changed.
				D3DImage.MultiSampling = value;
				NotifyPropertyChanged(nameof(MultiSampling));
				NotifyPropertyChanged(nameof(Antialiasing));
				if (ContextMenu?.DataContext is IView3dCMenu cmenu)
					cmenu.NotifyPropertyChanged(nameof(IView3dCMenu.Antialiasing));
			}
		}

		/// <summary>The time between mouse down->up that is considered a mouse click</summary>
		public int ClickTimeMS
		{
			get;
			set
			{
				if (field == value) return;
				field = value;
				NotifyPropertyChanged(nameof(ClickTimeMS));
			}
		}

		/// <summary>Enable/Disable default keyboard shortcuts</summary>
		public bool DefaultKeyboardShortcuts
		{
			get;
			set
			{
				if (DefaultKeyboardShortcuts == value)
					return;

				if (field)
				{
					KeyDown -= HandleKeyDown;
				}
				field = value;
				if (field)
				{
					KeyDown += HandleKeyDown;
				}
				
				NotifyPropertyChanged(nameof(DefaultKeyboardShortcuts));

				void HandleKeyDown(object sender, KeyEventArgs args)
				{
					var key = args.Key.ToKeyCode(include_modifier_keys: true);
					var mouse = Mouse.GetPosition(this);

					args.Handled = Window.TranslateKey(key, mouse.ToV2());
				}
			}
		}

		/// <summary>Hook up mouse navigation</summary>
		public bool MouseNavigation
		{
			get;
			set
			{
				if (MouseNavigation == value)
					return;

				if (field)
				{
					MouseDown -= OnMouseDown;
					MouseUp -= OnMouseUp;
					MouseMove -= OnMouseMove;
					MouseWheel -= OnMouseWheel;
				}
				field = value;
				if (field)
				{
					MouseWheel += OnMouseWheel;
					MouseMove += OnMouseMove;
					MouseUp += OnMouseUp;
					MouseDown += OnMouseDown;
				}

				NotifyPropertyChanged(nameof(MouseNavigation));
			}
		}

		/// <summary>The desired aspect ratio of pixels in the view</summary>
		public double DesiredPixelAspect
		{
			get;
			set
			{
				if (DesiredPixelAspect == value)
					return;

				if (double.IsNaN(value))
					throw new ArgumentException("The desired pixel aspect cannot be NaN");
				if (double.IsInfinity(value))
					throw new ArgumentException("The desired pixel aspect cannot be infinite");
				if (value == 0)
					throw new ArgumentException("The desired pixel aspect cannot be 0");

				field = value;
				NotifyPropertyChanged(nameof(DesiredPixelAspect));
			}
		}

		/// <summary>The current pixel aspect ratio</summary>
		public double ActualPixelAspect
		{
			get
			{
				var sz = RenderSize;
				return Camera.Aspect * sz.Height / sz.Width;
			}
			set
			{
				var sz = RenderSize;
				if (sz.Width != 0 && sz.Height != 0)
				{
					var aspect = (float)(value * sz.Width / sz.Height);
					if (!Math_.FEql(Camera.Aspect, aspect))
						Camera.Aspect = aspect;
				}
			}
		}

		/// <summary>The ratio of the back buffer resolution to the window resolution</summary>
		public double ResolutionScale
		{
			get;
			set
			{
				if (ResolutionScale == value)
					return;

				field = value;
				m_resized = true;
				Invalidate();
				NotifyPropertyChanged(nameof(ResolutionScale));
			}
		}

		/// <summary>Mouse navigation - public to allow users to forward mouse calls to us.</summary>
		public void OnMouseDown(object sender, MouseButtonEventArgs e)
		{
			if (Window == null) return;
			if (CaptureMouse())
			{
				Cursor = Cursors.SizeAll;
				m_mouse_down_at = Environment.TickCount;
				if (Window.MouseNavigate(e.GetPosition(this).ToPointI(), e.ToMouseBtns(), true))
					Invalidate();
			}
		}
		public void OnMouseUp(object sender, MouseButtonEventArgs e)
		{
			if (Window == null) return;
			if (IsMouseCaptured)
			{
				Cursor = Cursors.Arrow;
				if (Window.MouseNavigate(e.GetPosition(this).ToPointI(), e.ToMouseBtns(), View3d.ENavOp.None, true))
					Invalidate();

				ReleaseMouseCapture();
			}

			// Click detected
			if (Environment.TickCount - m_mouse_down_at < ClickTimeMS)
			{
				if (e.ChangedButton == MouseButton.Middle && e.MiddleButton == MouseButtonState.Released)
				{
					Camera.ResetZoom();
					Invalidate();
				}
			}
			else
			{
				e.Handled = true;
			}
		}
		public void OnMouseMove(object sender, MouseEventArgs e)
		{
			if (Window == null) return;
			if (IsMouseCaptured)
			{
				if (Window.MouseNavigate(e.GetPosition(this).ToPointI(), e.ToMouseBtns(), false))
					Invalidate();
			}
		}
		public void OnMouseWheel(object sender, MouseWheelEventArgs e)
		{
			if (Window == null) return;
			if (Window.MouseNavigateZ(e.GetPosition(this).ToPointI(), e.ToMouseBtns(), e.Delta, true))
				Invalidate();
		}
		private int m_mouse_down_at;

		/// <summary>Called whenever an error is generated in view3d</summary>
		public event EventHandler<ReportErrorEventArgs>? ReportError;
		protected virtual void OnReportError(ReportErrorEventArgs e)
		{
			ReportError?.Invoke(this, e);
		}

		/// <summary>Default handling of render target changes. Set the viewport and camera aspect</summary>
		public event EventHandler? RenderTargetChanged;
		protected virtual void OnRenderTargetChanged()
		{
			if (RenderTarget == null)
				return;

			// Set the viewport to match the render target size
			Window.Viewport = new View3d.Viewport(
				0, 0,
				RenderTarget.Info.Width,
				RenderTarget.Info.Height,
				(int)Math.Floor(RenderSize.Width != 0 ? RenderSize.Width : RenderTarget.Info.Width),
				(int)Math.Floor(RenderSize.Height != 0 ? RenderSize.Height : RenderTarget.Info.Height),
				0f, 1f);

			// Notify of a new render target. Don't directly subscribe to
			// D3DImage.FrontBufferChanged because that will leak references.
			RenderTargetChanged?.Invoke(this, EventArgs.Empty);
		}

		/// <summary>Render</summary>
		private void Render()
		{
			// Don't make this public, use 'Invalidate'
			if (!m_render_pending)
				return;

			// Clear the render pending flag on function return
			using var render_pending = Scope.Create(null, () => m_render_pending = false);

			// Ignore renders until we have a non-zero size, and the D3DImage has a render target
			if (!IsVisible || RenderSize == Size.Empty || D3DImage.FrontBuffer == null || !D3DImage.IsFrontBufferAvailable)
			{
				// 'Validate' the window so that future Invalidate() calls trigger the call back.
				Window?.Validate();
				return;
			}

			// If the size has changed, create a new front buffer of the correct size
			if (m_resized)
			{
				// The back-buffer updates whenever the front buffer changes
				Window.GSyncWait();
				D3DImage.SetRenderTargetSize(this, scale: ResolutionScale);
				m_resized = false;
			}

			// Set the camera aspect to achieve the desired pixel aspect
			ActualPixelAspect = DesiredPixelAspect;

			// Allow objects to be added/removed from the scene
			try { OnBuildScene(); }
			catch (OperationCanceledException) { }
			catch (Exception ex) { OnReportError(new ReportErrorEventArgs($"Error during build scene: {ex.Message}")); }

			// Render the scene into the render target texture
			Window.Render();
			Window.GSyncWait();

			// Update the "front buffer" in the D3DImage
			D3DImage.Flip();
			//D3DImage.Save("P:\\dump\\d3dimage.png");
		}
		private bool m_render_pending;

		/// <summary>Allow objects to be added/removed from the scene</summary>
		public event EventHandler<BuildSceneEventArgs>? BuildScene;
		protected virtual void OnBuildScene()
		{
			BuildScene?.Invoke(this, new BuildSceneEventArgs(Window));
		}

		/// <summary></summary>
		public event PropertyChangedEventHandler? PropertyChanged;
		public void NotifyPropertyChanged(string prop_name)
		{
			// Note: Notify is called from the SettingsChanged handler, not when the property is changed
			// because this catches all sources of a property changing, not just when it is explicitly set.
			PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(prop_name));
		}

		/// <summary>Raised when the control is disposed</summary>
		public event EventHandler? Disposing;

		#region EventArgs
		public class CustomContextMenuEventArgs : EventArgs
		{
			internal CustomContextMenuEventArgs(ContextMenu menu)
			{
				Menu = menu;
			}

			/// <summary>The context menu to be customised</summary>
			public ContextMenu Menu { get; }
		}
		public class ReportErrorEventArgs : EventArgs
		{
			internal ReportErrorEventArgs(string msg)
			{
				Message = msg;
			}

			/// <summary>Error message</summary>
			public string Message { get; }
		}
		public class BuildSceneEventArgs : EventArgs
		{
			public BuildSceneEventArgs(View3d.Window window)
			{
				Window = window;
			}

			/// <summary>The chart panel</summary>
			public View3d.Window Window { get; }

			/// <summary>Current camera position</summary>
			public View3d.Camera Camera => Window.Camera;
		}
		#endregion
	}
}
