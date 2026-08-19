using System;
using System.Diagnostics;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Windows.Forms;
using Rylogic.Extn;
using Rylogic.Interop.Win32;
using Rylogic.Maths;
using Rylogic.Utility;

namespace Rylogic.Gui.WinForms
{
	/// <summary>A tooltip form that doesn't suck</summary>
	public class HintBalloon :Form
	{
		private readonly object m_lock;
		private GraphicsPath m_path;
		private RichTextBox m_msg;
		private bool m_size_invalid;
		private int m_issue;

		/// <summary>Create a balloon tooltip form</summary>
		/// <param name="pin_to">Optional. Pin the balloon to a control and move with it</param>
		/// <param name="msg">The text on the hint balloon</param>
		/// <param name="target">The position of the balloon relative to the pinned control</param>
		/// <param name="show_delay">The time (in ms) before displaying the hint balloon</param>
		/// <param name="duration">How long the balloon is displayed for before auto hiding</param>
		/// <param name="fade_duration">The period to fade out the hint balloon</param>
		public HintBalloon(Control pin_to = null, string msg = null, Point? target = null, int show_delay = 500, int duration = 2000, int fade_duration = 500)
		{
			InitializeComponent();
			SetStyle(ControlStyles.Selectable           , false);
			SetStyle(ControlStyles.AllPaintingInWmPaint , true );
			SetStyle(ControlStyles.ResizeRedraw         , true );
			SetStyle(ControlStyles.UserPaint            , true );
			FormBorderStyle    = FormBorderStyle.None;
			StartPosition      = FormStartPosition.Manual;
			AutoSizeMode       = AutoSizeMode.GrowAndShrink;
			DoubleBuffered     = true;
			AutoResize         = true;
			ShowInTaskbar      = false;
			Visible            = false;
			m_lock             = new object();
			m_issue            = 0;
			ShowDelay          = show_delay;
			Duration           = duration;
			FadeDuration       = fade_duration;
			CornerRadius       = 10;
			TipBaseWidth       = 20;
			TipLength          = 20;
			PreferredTipCorner = ETipCorner.TopLeft;
			TipCorner          = ETipCorner.TopLeft;
			PinTo              = pin_to;
			Target             = target ?? pin_to?.ClientRectangle.Centre() ?? Point.Empty;
			Text               = msg ?? string.Empty;

			InvalidateSize();
			m_msg.GotFocus += (s,a) => Win32.HideCaret(m_msg.Handle);
		}
		protected override void Dispose(bool disposing)
		{
			Util.Dispose(ref components);
			base.Dispose(disposing);
		}
		protected override CreateParams CreateParams
		{
			get
			{
				var cp = base.CreateParams;
				cp.ClassStyle |= Win32.CS_DROPSHADOW;
				cp.Style &= ~Win32.WS_VISIBLE;
				cp.ExStyle |= Win32.WS_EX_NOACTIVATE;// | Win32.WS_EX_LAYERED | Win32.WS_EX_TRANSPARENT;
				return cp;
			}
		}
		protected override bool ShowWithoutActivation
		{
			get { return true; }
		}
		protected override void OnLayout(LayoutEventArgs levent)
		{
			using (this.SuspendLayout(false))
			{
				// Update the size of the balloon form
				if (m_size_invalid)
				{
					// Determine the best size for the balloon
					var size = PreferredTextBoxSize(Size.Empty);
					var margin = TipLength + CornerRadius;
					var balloon_size = new Size(size.Width + 2*margin, size.Height + 2*margin);
					var update_path = false;

					// Update the size of the form and the form's Region
					switch (AutoSizeMode)
					{
					case AutoSizeMode.GrowAndShrink:
						{
							if (Size != balloon_size)
							{
								Size = balloon_size;
								m_msg.Bounds = new Rectangle(margin, margin, size.Width, size.Height);
								update_path = true;
							}
							break;
						}
					case AutoSizeMode.GrowOnly:
						{
							if (Width < balloon_size.Width || Height < balloon_size.Height)
							{
								Width = Math.Max(Width, balloon_size.Width);
								Height = Math.Max(Height, balloon_size.Height);
								m_msg.Bounds = new Rectangle(margin, margin, Math.Max(m_msg.Width, size.Width), Math.Max(m_msg.Height, size.Height));
								update_path = true;
							}
							break;
						}
					}

					// Update the shape of the balloon form
					if (update_path || m_path == null)
					{
						m_path = GeneratePath(false);
						Region = new Region(GeneratePath(true));
					}

					m_size_invalid = false;
				}
			}
			base.OnLayout(levent);
		}
		protected override void OnPaint(PaintEventArgs e)
		{
			if (m_size_invalid)
				PerformLayout();

			base.OnPaint(e);

			// Draw the balloon and outline
			e.Graphics.SmoothingMode = SmoothingMode.AntiAlias;
			e.Graphics.FillRectangle(SystemBrushes.Info, ClientRectangle);
			e.Graphics.DrawPath(SystemPens.ActiveBorder, m_path);
		}
		public override Size GetPreferredSize(Size proposed_size)
		{
			// Determine the best size for the balloon
			var size = PreferredTextBoxSize(proposed_size);
			var margin = TipLength + CornerRadius;
			var balloon_size = new Size(size.Width + 2*margin, size.Height + 2*margin);
			return balloon_size;

		}

		/// <summary>Dynamically resize the balloon to fit the text</summary>
		public bool AutoResize
		{
			get;
			set;
		}

		/// <summary>The period to wait before displaying the hint (in milliseconds)</summary>
		public int ShowDelay { get; set; }

		/// <summary>How long to display the hint for (in milliseconds)</summary>
		public int Duration { get; set; }

		/// <summary>The time to take when fading out (in milliseconds)</summary>
		public int FadeDuration { get; set; }

		/// <summary>The text to display in the hint</summary>
		public override string Text
		{
			get { return m_msg?.Text ?? string.Empty; }
			set
			{
				if (Text == value) return;
				if (m_msg != null)
				{
					m_msg.Text = value;
					if (AutoResize) InvalidateSize();
					Invalidate();
				}
			}
		}

		/// <summary>The hint text in RTF</summary>
		public string TextRtf
		{
			get { return m_msg?.Rtf ?? string.Empty; }
			set
			{
				if (TextRtf == value) return;
				if (m_msg != null)
				{
					m_msg.Rtf = value;
					if (AutoResize) InvalidateSize();
					Invalidate();
				}
			}
		}

		/// <summary>The (optional) control that the balloon is pinned to</summary>
		public Control PinTo
		{
			get;
			set
			{
				if (field == value) return;
				field = value;
				Invalidate();
			}
		}

		/// <summary>The position to point the tool tip at (relative to the associated control)</summary>
		public Point Target
		{
			get;
			set
			{
				if (field == value) return;
				field = value;
				Invalidate();
			}
		}

		/// <summary>The radius of the corners of the balloon</summary>
		public int CornerRadius
		{
			get;
			set
			{
				if (field == value) return;
				field = value;
				InvalidateSize();
			}
		}

		/// <summary>The width of the fat end of the pointer</summary>
		public int TipBaseWidth
		{
			get;
			set
			{
				if (field == value) return;
				field = value;
				InvalidateSize();
			}
		}

		/// <summary>The vertical length of the tip</summary>
		public int TipLength
		{
			get;
			set
			{
				if (field == value) return;
				field = value;
				InvalidateSize();
			}
		}

		/// <summary>The preferred orientation of the balloon tip</summary>
		public ETipCorner PreferredTipCorner { get; set; }
		[Flags] public enum ETipCorner { TopLeft = 0, TopRight = 1, BottomLeft = 2, BottomRight = 3 }

		/// <summary>The current tip corner</summary>
		public ETipCorner TipCorner
		{
			get { return m_tip_corner; }
			private set
			{
				if (m_tip_corner == value) return;
				m_tip_corner = value;
				InvalidateSize();
			}
		}
		private ETipCorner m_tip_corner; // The corner that the tip currently points to

		/// <summary>Display the hint balloon</summary>
		public new void Show()
		{
			Show(PinTo);
		}
		public void Show(Control pin_to)
		{
			Show(pin_to, Target, Text);
		}
		public void Show(Control pin_to, Point target)
		{
			Show(pin_to, target, Text);
		}
		public void Show(Control pin_to, Point target, string msg)
		{
			Show(pin_to, target, msg, Duration);
		}
		public void Show(Control pin_to, Point target, string msg, int duration)
		{
			lock (m_lock)
			{
				Debug.Assert(msg != null && duration >= 0);
				var issue = ++m_issue;
				Target = target;
				Text = msg;
				Duration = duration;
				SynchronizationContext_.BeginInvokeDelayed(() => ShowHintInternal(issue, pin_to), TimeSpan.FromMilliseconds(ShowDelay));
			}
		}

		/// <summary>Make the hint balloon visible</summary>
		private void ShowHintInternal(int issue, Control pin_to)
		{
			lock (m_lock)
			{
				// Only display if the issue number is still valid
				if (issue != m_issue)
					return;

				if (PinTo != null)
				{
					PinTo.Move   -= DoPosition;
					PinTo.Resize -= DoPosition;
				}
				if (Owner != null)
				{
					Owner.Move       -= DoPosition;
					Owner.Resize     -= DoPosition;
					Owner.FormClosed -= DetachFromOwner;
				}

				Opacity = 1.0;
				PinTo   = pin_to;
				Owner   = pin_to != null ? pin_to.TopLevelControl as Form : null;
				TopMost = Owner == null;
				Location = pin_to != null ? pin_to.PointToScreen(Target) : Target;
				Win32.ShowWindow(Handle, Win32.SW_SHOWNOACTIVATE);

				if (PinTo != null)
				{
					PinTo.Move   += DoPosition;
					PinTo.Resize += DoPosition;
				}
				if (Owner != null)
				{
					Owner.Move       += DoPosition;
					Owner.Resize     += DoPosition;
					Owner.FormClosed += DetachFromOwner;
				}

				SynchronizationContext_.BeginInvokeDelayed(() => HideHintInternal(issue), TimeSpan.FromMilliseconds(Duration));
			}
		}

		/// <summary>Hide the hint balloon</summary>
		private void HideHintInternal(int issue)
		{
			lock (m_lock)
			{
				// Only hide if the issue number is still valid
				if (issue != m_issue || !Visible)
					return;

				// Test if the mouse is over the form, if so, wait another 2000
				var pt = PointToClient(Cursor.Position);
				var rect = ClientRectangle.Inflated(-TipLength,-TipLength);
				if (rect.Contains(pt))
				{
					Opacity = 1f;
					SynchronizationContext_.BeginInvokeDelayed(() => HideHintInternal(issue), TimeSpan.FromMilliseconds(2000));
					return;
				}

				// Do fading
				if (FadeDuration > 0)
				{
					Opacity -= Math.Min(0.1, Opacity);
					if (Opacity == 0)
						Visible = false;
					else
						SynchronizationContext_.BeginInvokeDelayed(() => HideHintInternal(issue), TimeSpan.FromMilliseconds(FadeDuration / 10));
					return;
				}

				// Done, hidden
				DetachFromOwner();
			}
		}

		/// <summary>Returns the target point in screen space</summary>
		private Point TargetInScreenSpace
		{
			get { return PinTo != null ? PinTo.PointToScreen(Target) : Target; }
		}

		/// <summary>Get the size of the text box containing the tip text</summary>
		private Size PreferredTextBoxSize(Size proposed_size)
		{
			const float ReflowAspectRatio = 5f;

			// Determine the best size for the balloon
			var size = proposed_size == Size.Empty ? proposed_size : new Size(640, 480);

			// Binary search for an aspect ratio ~= ReflowAspectRatio
			var initial_width = size.Width;
			for (float scale0 = 0.0f, scale1 = 1.0f;;)
			{
				var scale = (scale0 + scale1) / 2f;
				size = m_msg.GetPreferredSize(new Size((int)(initial_width * scale), 0));
				if      (size.Aspect() < ReflowAspectRatio) scale0 = scale;
				else if (size.Aspect() > ReflowAspectRatio) scale1 = scale;
				if (Math.Abs(scale1 - scale0) < 0.05f)
				{
					size = m_msg.GetPreferredSize(new Size((int)(initial_width * scale1), 0));
					break;
				}
			}

			return size;
		}

		/// <summary>Generates the boundary of the hint balloon</summary>
		private GraphicsPath GeneratePath(bool region_border)
		{
			var width  = Width  + (region_border ? 1 : 0);
			var height = Height + (region_border ? 1 : 0);
			var cr     = Math.Max(1, CornerRadius);

			GraphicsPath path;
			if (TipLength == 0 || TipBaseWidth == 0)
			{
				path = Gdi.RoundedRectanglePath(new RectangleF(0, 0, width, height), cr);
			}
			else
			{
				var tip_length = TipLength;
				var tip_width = Math_.Clamp(width - 2 * (tip_length + cr), Math.Min(5,TipBaseWidth), TipBaseWidth);

				// Find the corner to start from
				path = new GraphicsPath();
				switch (TipCorner)
				{
				default:
					Debug.Assert(false, "Unknown corner");
					break;
				case ETipCorner.TopLeft:
					path.AddLine(0, 0, tip_length + cr + tip_width, tip_length);
					path.AddArc(width - tip_length - cr, tip_length, cr, cr, 270f, 90f);
					path.AddArc(width - tip_length - cr, height - tip_length - cr, cr, cr, 0f, 90f);
					path.AddArc(tip_length, height - tip_length - cr, cr, cr, 90f, 90f);
					path.AddArc(tip_length, tip_length, cr, cr, 180f, 90f);
					path.AddLine(tip_length + cr, tip_length, 0, 0);
					break;
				case ETipCorner.TopRight:
					path.AddLine(width, 0, width - tip_length - cr, tip_length);
					path.AddArc(width - tip_length - cr, tip_length, cr, cr, 270f, 90f);
					path.AddArc(width - tip_length - cr, height - tip_length - cr, cr, cr, 0f, 90f);
					path.AddArc(tip_length, height - tip_length - cr, cr, cr, 90f, 90f);
					path.AddArc(tip_length, tip_length, cr, cr, 180f, 90f);
					path.AddLine(tip_length + cr, tip_length, width - tip_length - cr - tip_width, tip_length);
					path.AddLine(width - tip_length - cr - tip_width, tip_length, width, 0);
					break;
				case ETipCorner.BottomLeft:
					path.AddLine(0, height, tip_length + cr, height - tip_length);
					path.AddArc(tip_length, height - tip_length - cr, cr, cr, 90f, 90f);
					path.AddArc(tip_length, tip_length, cr, cr, 180f, 90f);
					path.AddArc(width - tip_length - cr, tip_length, cr, cr, 270f, 90f);
					path.AddArc(width - tip_length - cr, height - tip_length - cr, cr, cr, 0f, 90f);
					path.AddLine(width - tip_length - cr, height - tip_length, tip_length + cr + tip_width, height - tip_length);
					path.AddLine(tip_length + cr + tip_width, height - tip_length, 0, height);
					break;
				case ETipCorner.BottomRight:
					path.AddLine(width, height, width - tip_length - cr - tip_width, height - tip_length);
					path.AddArc(tip_length, height - tip_length - cr, cr, cr, 90f, 90f);
					path.AddArc(tip_length, tip_length, cr, cr, 180f, 90f);
					path.AddArc(width - tip_length - cr, tip_length, cr, cr, 270f, 90f);
					path.AddArc(width - tip_length - cr, height - tip_length - cr, cr, cr, 0f, 90f);
					path.AddLine(width - tip_length - cr, height - tip_length, width, height);
					break;
				}
			}
			return path;
		}

		/// <summary>Set the position of the hint balloon (and tip corner)</summary>
		private void DoPosition(object sender = null, EventArgs e = null)
		{
			// Set the position of the balloon given the target point relative to the screen.
			var pt = TargetInScreenSpace;
			var screen = Screen.FromPoint(pt);

			var loc = pt;
			int corner = (int)PreferredTipCorner;
			if ((corner & 1) != 0) loc.X -= Width;
			if ((corner & 2) != 0) loc.Y -= Height;
			if ((corner & 1) != 0 && loc.X          < screen.WorkingArea.Left  ) { loc.X += Width ; corner &= ~1; }
			if ((corner & 2) != 0 && loc.Y          < screen.WorkingArea.Top   ) { loc.Y += Height; corner &= ~2; }
			if ((corner & 1) == 0 && loc.X + Width  > screen.WorkingArea.Right ) { loc.X -= Width ; corner |=  1; }
			if ((corner & 2) == 0 && loc.Y + Height > screen.WorkingArea.Bottom) { loc.Y -= Height; corner |=  2; }

			Location = loc;
			TipCorner = (ETipCorner)corner;
		}

		/// <summary>Called when the owner is closed</summary>
		private void DetachFromOwner(object sender = null, EventArgs e = null)
		{
			// Immediately hide the balloon. This method should only be called
			// when it is still attached to the owner or from within a locked
			// section (the owner is only changed in a locked section). Therefore
			// we don't need to check the issue number.
			lock (m_lock)
			{
				Owner = null;
				Visible = false;
			}
		}

		/// <summary>Signal that the size of the hint balloon needs updating</summary>
		private void InvalidateSize()
		{
			m_size_invalid = true;
			Invalidate();
		}

		#region Windows Form Designer generated code
		/// <summary>Required designer variable.</summary>
		private System.ComponentModel.IContainer components = null;

		/// <summary>
		/// Required method for Designer support - do not modify
		/// the contents of this method with the code editor.</summary>
		private void InitializeComponent()
		{
			this.m_msg = new RichTextBox();
			this.SuspendLayout();
			//
			// m_msg
			//
			this.m_msg.BackColor = System.Drawing.SystemColors.Info;
			this.m_msg.BorderStyle = System.Windows.Forms.BorderStyle.None;
			this.m_msg.Location = new System.Drawing.Point(23, 28);
			this.m_msg.Margin = new System.Windows.Forms.Padding(0);
			this.m_msg.Name = "m_msg";
			this.m_msg.ReadOnly = true;
			this.m_msg.ScrollBars = System.Windows.Forms.RichTextBoxScrollBars.None;
			this.m_msg.Size = new System.Drawing.Size(154, 59);
			this.m_msg.TabIndex = 0;
			this.m_msg.TabStop = false;
			this.m_msg.Text = "hint text goes here";
			//
			// HintBalloon
			//
			this.AutoSizeMode = System.Windows.Forms.AutoSizeMode.GrowAndShrink;
			this.BackColor = System.Drawing.SystemColors.ActiveBorder;
			this.ClientSize = new System.Drawing.Size(200, 120);
			this.ControlBox = false;
			this.Controls.Add(this.m_msg);
			this.FormBorderStyle = System.Windows.Forms.FormBorderStyle.None;
			this.MaximizeBox = false;
			this.MinimizeBox = false;
			this.Name = "HintBalloon";
			this.ShowIcon = false;
			this.ShowInTaskbar = false;
			this.StartPosition = System.Windows.Forms.FormStartPosition.Manual;
			this.TransparencyKey = System.Drawing.Color.Fuchsia;
			this.ResumeLayout(false);
}
		#endregion
	}
}

#if PR_UNITTESTS
namespace Rylogic.UnitTests
{
	using Gui.WinForms;

	[TestFixture] public class TestTooltip
	{
		/*[Test] */public void Test()// Disabled cause the pop up is annoying
		{
			const string text =
				"Short Msg\r\n"+
				"Really long message goes here with lots of words\r\n"+
				"Including some new lines\r\n"+
				"..and really really really really really really really really really really really really really really really really really really really really really really really long sentences"+
				"";

			//var btns = MessageBoxButtons.OKCancel;
			//var icon = MessageBoxIcon.Question;
			//var line = string.Join(" "  , Enumerable.Range(0, 30).Select(x => "123456789"));
			//var msg = string.Join("\r\n", Enumerable.Range(0, 30).Select(x => line));

			new HintBalloon(target:new Point(500,500), msg:text).Show();
		}
	}
}
#endif