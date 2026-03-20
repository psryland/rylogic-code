using System;
using System.Windows;
using System.Windows.Interop;
using Rylogic.Interop.Win32;
using Rylogic.Utility;

namespace Rylogic.Gui.WPF
{
	/// <summary>Anchor points on the pin target</summary>
	public enum EPin
	{
		TopLeft,
		TopCentre,
		TopRight,
		BottomLeft,
		BottomCentre,
		BottomRight,
		CentreLeft,
		Centre,
		CentreRight,
	}

	/// <summary>The window supports pinning</summary>
	public interface IPinnable
	{
		PinData PinState { get; }
	}

	/// <summary>Provides the functionality for pinning a window to another</summary>
	public sealed class PinData :IDisposable
	{
		// Notes:
		//  - Dispose is called when 'window' is closed 
		//
		// Usage:
		//  In your window class:
		//    ctor()
		//    {
		//        // Dispose is handled by 'PinData'
		//        PinState = new PinData(this, EPin.Centre);
		//    }
		//
		//    /// <summary>Support pinning this window</summary>
		//    private PinData PinState { get; }
		//

		public PinData(Window window, EPin pin_site, bool pinned)
		{
			PinSite = pin_site;
			PinWindow = window;
			PinTarget = window.Owner ?? Application.Current.MainWindow;
			Pinned = pinned;
		}
		public void Dispose()
		{
			Pinned = false;
			PinTarget = null;
			PinWindow = null!;
		}

		/// <summary>Pin or Unpin the window from the target</summary>
		public bool Pinned
		{
			get => m_pinned;
			set
			{
				// Note: this represents the requested state, not the actual state.
				// Actual "pinned-ness" depends on whether there is a PinTarget and PinOffset.
				// For this reason, there is no if (m_pinned == value) return check.
				m_pinned = value;
				PinOffset = null;

				// Set or Restore the startup location based on 'Pinned'
				if (PinWindow != null)
				{
					PinWindow.WindowStartupLocation = Pinned ? WindowStartupLocation.Manual : m_pin_window_startloc;
					if (Pinned) UpdateLocation();
				}

				// Check/Uncheck the menu option
				UpdatePinMenuCheckState();
			}
		}
		private bool m_pinned;

		/// <summary>The point on the target window that we move relative to</summary>
		public EPin PinSite
		{
			get;
			set
			{
				if (field == value) return;
				field = value;
			}
		}

		/// <summary>The window whose position is being controlled</summary>
		private Window PinWindow
		{
			get;
			set
			{
				if (field == value) return;
				if (field != null)
				{
					field.LayoutUpdated -= HandleLayoutUpdated;
					field.LocationChanged -= HandleMoved;
					field.SizeChanged -= HandleMoved;
					field.Closed -= HandleClosed;

					// Remove the menu item from the system menu
					PinOptionInSysMenu(false);

					field.WindowStartupLocation = m_pin_window_startloc;
					m_pin_window_handle = IntPtr.Zero;
				}
				field = value;
				if (field != null)
				{
					try { m_pin_window_handle = new WindowInteropHelper(field).EnsureHandle(); }
					catch
					{
						m_pin_window_handle = IntPtr.Zero;
						field = null!;
						return;
					}
					m_pin_window_startloc = field.WindowStartupLocation;
					field.WindowStartupLocation = Pinned ? WindowStartupLocation.Manual : m_pin_window_startloc;

					// Add a menu item to the system menu
					PinOptionInSysMenu(true);

					field.Closed += HandleClosed;
					field.SizeChanged += HandleMoved;
					field.LocationChanged += HandleMoved;
					field.LayoutUpdated += HandleLayoutUpdated;
				}

				// Handlers
				void HandleLayoutUpdated(object? sender, EventArgs e)
				{
					if (double.IsNaN(PinWindow.ActualWidth) || double.IsNaN(PinWindow.ActualHeight))
						return;

					// On first load, set the initial position based on the pin location
					PinOffset ??= PinSite switch
					{
						EPin.TopLeft => new Vector(-PinWindow.ActualWidth, 0),
						EPin.TopCentre => new Vector(-PinWindow.ActualWidth / 2, 0),
						EPin.TopRight => new Vector(0, 0),
						EPin.BottomLeft => new Vector(-PinWindow.ActualWidth, -PinWindow.ActualHeight),
						EPin.BottomCentre => new Vector(-PinWindow.ActualWidth / 2, -PinWindow.ActualHeight),
						EPin.BottomRight => new Vector(0, -PinWindow.ActualHeight),
						EPin.CentreLeft => new Vector(-PinWindow.ActualWidth, -PinWindow.ActualHeight / 2),
						EPin.Centre => new Vector(-PinWindow.ActualWidth / 2, -PinWindow.ActualHeight / 2),
						EPin.CentreRight => new Vector(0, -PinWindow.ActualHeight / 2),
						_ => throw new Exception($"Unknown pin location '{PinSite}'"),
					};
				}
				void HandleMoved(object? sender, EventArgs e)
				{
					// When the location of the controlled window changes, record the offset from
					// the target window, but only if it's not us setting the controlled window's location.
					if (!UpdatingLocation && PinWindow.IsLoaded)
						PinOffset = MeasureOffset();
				}
				void HandleClosed(object? sender, EventArgs e)
				{
					// Dispose when the controlled window closes
					Dispose();
				}
			}
		} = null!;
		private IntPtr m_pin_window_handle;
		private WindowStartupLocation m_pin_window_startloc;

		/// <summary>The window to position relative to (defaults to the Owner of PinWindow)</summary>
		public Window? PinTarget
		{
			get => m_pin_target;
			set
			{
				if (m_pin_target == value) return;
				if (m_pin_target != null)
				{
					m_pin_target.LocationChanged -= HandleMoved;
					m_pin_target.SizeChanged -= HandleMoved;
					m_pin_target_handle = IntPtr.Zero;
				}
				m_pin_target = value;
				if (m_pin_target != null)
				{
					m_pin_target_handle = new WindowInteropHelper(m_pin_target).EnsureHandle();
					m_pin_target.SizeChanged += HandleMoved;
					m_pin_target.LocationChanged += HandleMoved;
				}

				UpdatePinMenuCheckState();

				// Handler
				void HandleMoved(object? sender, EventArgs e)
				{
					// When the target window moves, update the position of the pinned window.
					if (Pinned)
						SignalUpdateLocation();
				}
			}
		}
		private Window? m_pin_target;
		private IntPtr m_pin_target_handle;

		/// <summary>The offset from the PinTarget to the desired location of the PinWindow</summary>
		public Vector? PinOffset
		{
			get;
			set
			{
				if (field == value) return;
				field = value;
				UpdatePinMenuCheckState();
				UpdateLocation();
			}
		}

		/// <summary>True if actually pinned</summary>
		private bool ActuallyPinned => Pinned && PinTarget != null && PinOffset != null;

		/// <summary>Measure the offset from the target window</summary>
		private Vector? MeasureOffset()
		{
			// Don't measure the offset until the window is loaded and in it's initial position
			if (PinWindow == null || !PinWindow.IsLoaded)
				return null;

			// The offset is unknown if the target window is not visible
			if (PinTarget == null || PinTarget.Visibility != Visibility.Visible)
				return null;

			// Target does not have measurement data yet
			if (double.IsNaN(PinTarget.Left) ||
				double.IsNaN(PinTarget.Top) ||
				double.IsNaN(PinTarget.ActualWidth) ||
				double.IsNaN(PinTarget.ActualHeight))
				return null;

			// Measure the offset from the reference point
			var rect = new Rect(PinTarget.Left, PinTarget.Top, PinTarget.ActualWidth, PinTarget.ActualHeight);
			var pt = new Point(PinWindow.Left, PinWindow.Top);
			return PinSite switch
			{
				EPin.TopLeft      => new Vector(pt.X - rect.Left                         , pt.Y - rect.Top),
				EPin.TopCentre    => new Vector(pt.X - (rect.Left + rect.Right) / 2      , pt.Y - rect.Top),
				EPin.TopRight     => new Vector(pt.X - rect.Right                        , pt.Y - rect.Top),
				EPin.BottomLeft   => new Vector(pt.X - rect.Left                         , pt.Y - rect.Bottom),
				EPin.BottomCentre => new Vector(pt.X - (rect.Left + rect.Right) / 2      , pt.Y - rect.Bottom),
				EPin.BottomRight  => new Vector(pt.X - rect.Right                        , pt.Y - rect.Bottom),
				EPin.CentreLeft   => new Vector(pt.X - rect.Left                         , pt.Y - (rect.Top + rect.Bottom) / 2),
				EPin.Centre       => new Vector(pt.X - (rect.Left + rect.Right) / 2      , pt.Y - (rect.Top + rect.Bottom) / 2),
				EPin.CentreRight  => new Vector(pt.X - rect.Right                        , pt.Y - (rect.Top + rect.Bottom) / 2),
				_                 => throw new Exception($"Unknown pin site '{PinSite}'"),
			};
		}

		/// <summary>Return the location for the PinWindow based on the current PinTarget position and PinOffset</summary>
		private Point? Location()
		{
			// The location is unknown if the target window is not visible
			if (PinTarget == null || PinTarget.Visibility != Visibility.Visible)
				return null;

			// If there is no pin offset, then the location is unknown
			PinOffset ??= MeasureOffset();
			if (PinOffset == null)
				return null;

			// Calculate the position
			var pin_offset = PinOffset.Value;
			var rect = PinTarget.WindowState != WindowState.Maximized || (PinWindow?.IsActive ?? true)
				? new Rect(PinTarget.Left, PinTarget.Top, PinTarget.ActualWidth, PinTarget.ActualHeight)
				: Gui_.MonitorRect(User32.MonitorFromWindow(m_pin_target_handle));

			var pt = PinSite switch
			{
				EPin.TopLeft      => new Point(rect.Left                    , rect.Top) + pin_offset,
				EPin.TopCentre    => new Point((rect.Left + rect.Right) / 2 , rect.Top) + pin_offset,
				EPin.TopRight     => new Point(rect.Right                   , rect.Top) + pin_offset,
				EPin.BottomLeft   => new Point(rect.Left                    , rect.Bottom) + pin_offset,
				EPin.BottomCentre => new Point((rect.Left + rect.Right) / 2 , rect.Bottom) + pin_offset,
				EPin.BottomRight  => new Point(rect.Right                   , rect.Bottom) + pin_offset,
				EPin.CentreLeft   => new Point(rect.Left                    , (rect.Top + rect.Bottom) / 2) + pin_offset,
				EPin.Centre       => new Point((rect.Left + rect.Right) / 2 , (rect.Top + rect.Bottom) / 2) + pin_offset,
				EPin.CentreRight  => new Point(rect.Right                   , (rect.Top + rect.Bottom) / 2) + pin_offset,
				_                 => throw new Exception($"Unknown pin location '{PinSite}'"),
			};

			// Keep the window on-screen (if not being dragged by the user)
			if (PinWindow?.IsActive == false)// || PinWindow?.IsLoaded == false)
			{
				if (PinTarget.WindowState == WindowState.Maximized)
				{
					var mon = User32.MonitorFromWindow(m_pin_target_handle);
					pt = Gui_.OnMonitor(mon, pt, PinWindow.RenderSize);
				}
				pt = Gui_.OnScreen(pt, PinWindow.RenderSize);
			}

			return pt;
		}

		/// <summary>BeginInvoke a location update</summary>
		private void SignalUpdateLocation()
		{
			if (m_location_update_pending) return;
			m_location_update_pending = true;
			PinWindow.Dispatcher.BeginInvoke(new Action(UpdateLocation));
		}
		private bool m_location_update_pending;

		/// <summary>Return the location for the PinWindow based on the current PinTarget position and PinOffset</summary>
		private void UpdateLocation()
		{
			m_location_update_pending = false;
			if (Location() is Point pt)
			{
				using var s = Scope.Create(() => UpdatingLocation = true, () => UpdatingLocation = false);
				PinWindow.SetLocation(pt);
			}
		}

		/// <summary>True when 'PinWindow' is being moved by us</summary>
		private bool UpdatingLocation { get; set; }

		/// <summary>Add or remove the 'Pin Window' menu option in the system menu</summary>
		private void PinOptionInSysMenu(bool add)
		{
			if (add)
			{
				// Get the win32 HWND for 'PinWindow'
				if (m_pin_window_handle == IntPtr.Zero)
					throw new Exception("The pinned window does not yet have a window handle");

				// Get the system menu handle for the window
				var sys_menu_handle = User32.GetSystemMenu(m_pin_window_handle, false);
				
				// Insert the 'Pin Window' option
				User32.InsertMenu(sys_menu_handle, 5, Win32.MF_BYPOSITION | Win32.MF_SEPARATOR, 0, string.Empty);
				User32.InsertMenu(sys_menu_handle, 6, Win32.MF_BYPOSITION | Win32.MF_STRING, MenuCmd_Pinned, "&Pin Window");

				// Set the checked state
				UpdatePinMenuCheckState();

				// Install a handler for the system menu item
				var src = HwndSource.FromHwnd(m_pin_window_handle);
				src.AddHook(HandlePinMenuOption);
			}
			else
			{
				// Remove the handler for the system menu item
				var src = HwndSource.FromHwnd(m_pin_window_handle);
				src.RemoveHook(HandlePinMenuOption);

				// Reset the system menu for the window
				User32.GetSystemMenu(m_pin_window_handle, true);
			}

			// Handler
			IntPtr HandlePinMenuOption(IntPtr hwnd, int msg, IntPtr wParam, IntPtr lParam, ref bool handled)
			{
				if (msg == Win32.WM_SYSCOMMAND && wParam.ToInt32() == MenuCmd_Pinned)
				{
					Pinned = !ActuallyPinned;
					handled = true;
				}
				return IntPtr.Zero;
			}
		}

		/// <summary>Update the check mark next to the Pin Window menu option</summary>
		private void UpdatePinMenuCheckState()
		{
			if (m_pin_window_handle == IntPtr.Zero) return;
			var sys_menu_handle = User32.GetSystemMenu(m_pin_window_handle, false);
			User32.CheckMenuItem(sys_menu_handle, MenuCmd_Pinned, Win32.MF_BYCOMMAND | (ActuallyPinned ? Win32.MF_CHECKED : Win32.MF_UNCHECKED));
		}

		/// <summary>The command id for the Pin menu item</summary>
		private const int MenuCmd_Pinned = 1000;
	}
}
