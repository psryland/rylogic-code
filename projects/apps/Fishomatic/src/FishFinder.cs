using System;
using System.ComponentModel;
using System.Drawing.Imaging;
using System.Windows.Input;
using System.Windows.Threading;
using Rylogic.Gui.WPF;
using Rylogic.Interop.Win32;
using Rylogic.Maths;
using Rylogic.Windows.Extn;
using Bitmap = System.Drawing.Bitmap;
using Color = System.Drawing.Color;
using Graphics = System.Drawing.Graphics;
using Point = System.Drawing.Point;
using Rectangle = System.Drawing.Rectangle;

namespace Fishomatic
{
	public class FishFinder :IDisposable, INotifyPropertyChanged
	{
		public FishFinder(Settings settings)
		{
			Settings = settings;
			State = EState.Idle;
			Ticker = true;
		}
		public void Dispose()
		{
			Run = false;
			Ticker = false;
		}

		/// <summary></summary>
		private Settings Settings { get; }

		/// <summary>State machine state</summary>
		public EState State
		{
			get => m_state_curr;
			set
			{
				if (State == value) return;
				m_state_curr = value;
				m_state_curr_when = Environment.TickCount;
				m_state_next_when = m_state_curr_when;
				NotifyPropertyChanged(nameof(State));
				NotifyPropertyChanged(nameof(Run));
			}
		}
		public EState StateNext
		{
			get => m_state_next;
		}
		private EState m_state_curr;
		private EState m_state_next;
		private int m_state_curr_when;
		private int m_state_next_when;

		/// <summary>Request a state machine state change after X seconds</summary>
		private void StateChange(EState next_state)
		{
			StateChange(next_state, TimeSpan.Zero);
		}
		private void StateChange(EState next_state, TimeSpan delay)
		{
			m_state_next = next_state;
			m_state_next_when = Environment.TickCount + (int)delay.TotalMilliseconds;
		}

		/// <summary>The string describing what's going on right now</summary>
		public string Status
		{
			get;
			private set
			{
				if (Status == value) return;
				field = value;
				NotifyPropertyChanged(nameof(Status));
			}
		} = string.Empty;

		/// <summary>Time until the next state change</summary>
		public TimeSpan Remaining => TimeSpan.FromMilliseconds(Math.Max(0, m_state_next_when - Environment.TickCount));

		/// <summary>Fractional time until state change</summary>
		public double ProgressFrac
		{
			get;
			private set
			{
				if (field == value) return;
				field = Math_.Clamp(value, 0.0, 1.0);
				NotifyPropertyChanged(nameof(ProgressFrac));

			}
		}

		/// <summary>Max bobber move distance</summary>
		public double MaxDelta
		{
			get;
			private set
			{
				if (field == value) return;
				field = value;
				NotifyPropertyChanged(nameof(MaxDelta));
			}
		}

		/// <summary>WoW main window</summary>
		public CWindow? TargetWnd
		{
			get
			{
				// The WoW Window can appear or disappear at any time
				if (m_wow != null)
				{
					if (!User32.IsWindow(m_wow) || m_wow.WindowRectangle.Width < 50 || m_wow.WindowRectangle.Height < 50)
						m_wow = null;
				}
				if (m_wow == null)
				{
					// Find the window by name, if there are multiple windows, choose the largest
					foreach (var win in Rylogic.Interop.Win32.Windows.GetWindowsByName(Settings.TargetWindowName, false))
					{
						// Find the largest window with WindowName as a title
						if (m_wow != null)
						{
							var r0 = m_wow.WindowRectangle;
							var r1 = win.WindowRectangle;
							if (r1.Width * r1.Height > r0.Width * r0.Height && r1.Width > 50 && r1.Height > 50)
								m_wow = win;
						}
						m_wow = win;
					}
				}
				return m_wow;
			}
		}
		private CWindow? m_wow = null!;

		/// <summary>Run the fish finder. This runs all the time, because colour sampling etc needs to work when not fishing</summary>
		private bool Ticker
		{
			get => m_ticker != null;
			set
			{
				if (Ticker == value) return;
				if (m_ticker != null)
				{
					m_ticker.Stop();
				}
				m_ticker = value ? new DispatcherTimer(TimeSpan.FromMilliseconds(100), DispatcherPriority.Normal, HandleTick, Dispatcher.CurrentDispatcher) : null;
				if (m_ticker != null)
				{
					m_ticker.Start();
				}

				// Handle the timer tick
				void HandleTick(object? sender, EventArgs e)
				{
					GoFish();
				}
			}
		}
		private DispatcherTimer? m_ticker;

		/// <summary>Run/Pause the fishing</summary>
		public bool Run
		{
			get => State != EState.Idle;
			set
			{
				if (Run == value) return;
				StateChange(value ? EState.Start : EState.Idle);
			}
		}

		/// <summary>Run the fishing state machine</summary>
		private void GoFish()
		{
			// Handle the WoW window appearing/disappearing
			if (TargetWnd is not CWindow wow)
				return;

			// If a state change is requested, wait for the delay time
			if (State != StateNext)
			{
				NotifyPropertyChanged(nameof(Remaining));
				if (Remaining != TimeSpan.Zero)
				{
					ProgressFrac = m_state_next_when > m_state_curr_when ? Math_.Frac(m_state_curr_when, Environment.TickCount, m_state_next_when) : 0.0;
					return;
				}

				State = StateNext;
			}

			// Target RGB colour
			var target_colour = new RGB(Settings.TargetColour);

			// Handle the state
			switch (State)
			{
				case EState.Idle:
				{
					// While idle (paused), allow the sample colour to be set
					var sampling = Win32.KeyDown(EKeyCodes.LControlKey) && User32.GetForegroundWindow() == wow.Hwnd;
					if (sampling)
					{
						// Read the colour of the pixel under the mouse
						var mouse_pt = User32.GetCursorPos().ToPoint().ToPointD();
						var search_area = Settings.SmallSearchArea(mouse_pt).ToRectI();
						m_small_grab = CaptureScreen(ref m_small_grab, search_area);
						Settings.TargetColour = m_small_grab.GetPixel(search_area.Width / 2, search_area.Height / 2);
					}
					Status = sampling ? "Sampling Target Colour" : "Idle";
					NotifyBobberUpdate(false);
					break;
				}
				case EState.Start:
				{
					m_start_time = Environment.TickCount;
					m_baubles_time = Environment.TickCount;
					NotifyBobberUpdate(false);
					StateChange(EState.Cast);
					break;
				}
				case EState.Cast:
				{
					// See if it's time to apply baubles
					if (TimeSpan.FromMilliseconds(Environment.TickCount - m_baubles_time).TotalMinutes > Settings.BaublesTimeMins)
					{
						StateChange(EState.ApplyBaubles);
						break;
					}

					// Otherwise, cast for another fish
					Status = "Casting";
					PressKey(wow, Settings.CastKey);
					m_cast_time = Environment.TickCount;
					StateChange(EState.LookForBobber, TimeSpan.FromSeconds(Settings.AfterCastWaitS));
					break;
				}
				case EState.LookForBobber:
				{
					// Find the bobber position.
					// Do a large search, then a small search because the detected position
					// can change a large amount between the searches.
					if (FindBobberPosition(target_colour, null) is Point pt0 &&
						FindBobberPosition(target_colour, pt0) is Point pt)
					{
						m_bobber_position = pt;
						m_found_time = Environment.TickCount;
						StateChange(EState.WatchBobber, TimeSpan.FromMilliseconds(100));
						MaxDelta = 0;
						break;
					}

					// Can't find the bobber?
					if (TimeSpan.FromMilliseconds(Environment.TickCount - m_cast_time) > TimeSpan.FromSeconds(Settings.AbortTimeS))
					{
						Status = "Bobber not found";
						StateChange(EState.Cast, TimeSpan.FromMilliseconds(500));
						break;
					}

					ProgressFrac = Math_.Frac(m_cast_time, Environment.TickCount, m_cast_time + Settings.MaxFishCycleS * 1000.0);
					Status = "Looking for bobber";
					break;
				}
				case EState.WatchBobber:
				{
					if (!(FindBobberPosition(target_colour, m_bobber_position) is Point pt))
					{
						// If we lose the bobber, assume it's bobbed, and click
						Status = $"Bobber lost, assumed bobbed";
						StateChange(EState.ClickBobber, TimeSpan.FromSeconds(Settings.ClickDelayS));
						break;
					}

					// Measure the movement away from 'm_bobber_position'
					var dist = (m_bobber_position.ToPointD() - pt.ToPointD()).Length;
					MaxDelta = Math.Max(MaxDelta, dist);

					// For the first second gauge the movement
					if (Settings.AutoThreshold && TimeSpan.FromMilliseconds(Environment.TickCount - m_found_time).TotalSeconds < Settings.SettlingTimeS)
					{
						Settings.MoveThreshold = MaxDelta + Settings.AutoThresholdMargin;
					}
					else if (dist > Settings.MoveThreshold)
					{
						Status = "Bobber movement detected";
						StateChange(EState.ClickBobber, TimeSpan.FromSeconds(Settings.ClickDelayS));
						break;
					}

					//Cursor.Position = pt;
					Status = $"Bobber @({pt.X},{pt.Y})";


					// Time Remaining
					if (TimeSpan.FromMilliseconds(Environment.TickCount - m_cast_time).TotalSeconds > Settings.MaxFishCycleS)
					{
						Status = "No bobber movement detected";
						StateChange(EState.Cast, TimeSpan.Zero);
						break;
					}

					ProgressFrac = Math_.Frac(m_cast_time, Environment.TickCount, m_cast_time + Settings.MaxFishCycleS * 1000.0);
					break;
				}
				case EState.ClickBobber:
				{
					Status = "Caught a fish!!";
					ClickBobber(wow, m_bobber_position, MouseButton.Right);
					StateChange(EState.Cast, TimeSpan.FromSeconds(Settings.AfterCatchWaitS));
					break;
				}
				case EState.ApplyBaubles:
				{
					Status = "Applying baubles...";
					PressKey(wow, Settings.BaublesKey);
					StateChange(EState.ApplyBaublesToPole, TimeSpan.FromMilliseconds(500));
					break;
				}
				case EState.ApplyBaublesToPole:
				{
					Status = "Applying baubles to Pole...";
					PressKey(wow, Settings.FishingPoleKey);
					StateChange(EState.ApplyBaublesWaiting, TimeSpan.FromMilliseconds(500));
					break;
				}
				case EState.ApplyBaublesWaiting:
				{
					Status = "Applying baubles to Pole, waiting...";
					StateChange(EState.ApplyBaublesDone, TimeSpan.FromSeconds(Settings.BaublesApplyWaitS));
					break;
				}
				case EState.ApplyBaublesDone:
				{
					Status = "Applying baubles to Pole, waiting... Done!";
					m_baubles_time = Environment.TickCount;
					StateChange(EState.Cast, TimeSpan.Zero);
					break;
				}
				default:
				{
					throw new Exception($"Unhandled state: {State}");
				}
			}
		}
		private Bitmap? m_small_grab;
		private Bitmap? m_large_grab;
		private Point m_bobber_position;
		private int m_baubles_time;
		private int m_found_time;
		private int m_start_time;
		private int m_cast_time;

		/// <summary>Search the search area for the bobber</summary>
		private Point? FindBobberPosition(RGB target, Point? position)
		{
			if (TargetWnd == null)
				return null;

			// Get the search area
			var search_rect = position != null
				? Settings.SmallSearchArea(position.Value.ToPointD()).ToRectI()
				: Settings.LargeSearchArea(TargetWnd.WindowRectangle.ToRectD()).ToRectI();

			// Sample the screen
			var bitmap = position != null
				? CaptureScreen(ref m_small_grab, search_rect)
				: CaptureScreen(ref m_large_grab, search_rect);
			if (bitmap == null)
				return null;

			// Find the nearest colour in the search area
			var pt = FindTargetColour(bitmap, target, out var distance);
			pt = new Point(
				pt.X + search_rect.X,
				pt.Y + search_rect.Y);

			return distance < Settings.ColourTolerence ? pt : null;
		}

		/// <summary>Return the point within a bitmap that matches 'target' the closest</summary>
		private Point FindTargetColour(Bitmap bitmap, RGB target, out double distance)
		{
			// Access the bits of the bitmap
			var rect = new Rectangle(0, 0, bitmap.Width, bitmap.Height);
			var bm = bitmap.LockBits(rect, ImageLockMode.ReadOnly, bitmap.PixelFormat);

			var pt = new Point(0, 0);
			distance = double.MaxValue;
			unsafe
			{
				//Bitmap test_bm = new Bitmap(rect.Width, rect.Height, bm.PixelFormat);
				var px = (RGB*)bm.Scan0.ToPointer();
				for (int y = 0; y != bitmap.Height; ++y)
				{
					var col = px + y * bitmap.Width;
					for (int x = 0; x != bitmap.Width; ++x, ++col)
					{
						//test_bm.SetPixel(x,y,Color.FromArgb(0xff, col->r, col->g, col->b));
						var distsq = RGB.DistanceSq(target, *col);
						if (distsq < distance)
						{
							pt.X = x;
							pt.Y = y;
							distance = distsq;
						}
					}
				}
				//test_bm.Save("D:/deleteme/screen_grap2.bmp");
			}
			bitmap.UnlockBits(bm);
			distance = Math.Sqrt(distance);
			return pt;
		}

		/// <summary>Capture an area of the screen. Return 'bm'</summary>
		private Bitmap CaptureScreen(ref Bitmap? bm, Rectangle area)
		{
			// Make sure the bitmap exists and is the right size
			if (bm == null || bm.Size != area.Size)
			{
				//Debug.Assert( .PrimaryScreen.BitsPerPixel == 32);
				bm = new Bitmap(area.Width, area.Height, PixelFormat.Format32bppRgb);
			}
			try
			{
				using var g = Graphics.FromImage(bm);
				g.CopyFromScreen(area.Location, new Point(0, 0), area.Size);
				//bm.Save(@"P:/dump/screen_grab.bmp");
			}
			catch {}
			return bm;
		}

		/// <summary>Send a keypress to the wow window</summary>
		private void PressKey(CWindow win, int key)
		{
			win.SendMessage(Win32.WM_KEYDOWN, key, 0x00080001);
			win.SendMessage(Win32.WM_KEYUP, key, 0x00080001);
		}

		/// <summary>Send a mouse click to the wow window</summary>
		private void ClickBobber(CWindow win, Point pos, MouseButton btn)
		{
			int wparam;
			uint down_msg, up_msg;

			switch (btn)
			{
				case MouseButton.Left:
				{
					wparam = Win32.MK_LBUTTON;
					down_msg = Win32.WM_LBUTTONDOWN;
					up_msg = Win32.WM_LBUTTONUP;
					break;
				}
				case MouseButton.Right:
				{
					wparam = Win32.MK_RBUTTON;
					down_msg = Win32.WM_RBUTTONDOWN;
					up_msg = Win32.WM_RBUTTONUP;
					break;
				}
				default:
				{
					return;
				}
			}

			//Cursor.Position = pos;
			var lparam = (int)Win32.PointToLParam(pos);
			win.SendMessage(down_msg, wparam, lparam);
			win.SendMessage(up_msg, wparam, lparam);
		}

		/// <summary></summary>
		private void NotifyBobberUpdate(bool visible)
		{
			NotifyBobberUpdate(visible, new Point(0, 0));
		}
		private void NotifyBobberUpdate(bool visible, Point position)
		{
			BobberUpdate?.Invoke(this, new BobberUpdateEventArgs(visible, position));
		}

		/// <summary></summary>
		public event EventHandler<BobberUpdateEventArgs>? BobberUpdate;
		public class BobberUpdateEventArgs :EventArgs
		{
			public BobberUpdateEventArgs(bool visible, Point position)
			{
				Visible = visible;
				Position = position;
			}

			/// <summary>True if the bobber is visible</summary>
			public bool Visible { get; }

			/// <summary>The window-relative position of the bobber</summary>
			public Point Position { get; }
		}

		/// <summary>Target colour</summary>
		private struct RGB
		{
			public byte b, g, r, a;

			public RGB(Color col)
			{
				b = col.B;
				g = col.G;
				r = col.R;
				a = col.A;
			}
			public static double DistanceSq(RGB lhs, RGB rhs)
			{
				var r = lhs.r - rhs.r;
				var g = lhs.g - rhs.g;
				var b = lhs.b - rhs.b;
				return r * r + g * g + b * b;
			}
		}

		/// <inheritdoc/>
		public event PropertyChangedEventHandler? PropertyChanged;
		public void NotifyPropertyChanged(string prop_name) => PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(prop_name));
	}
}
