using System;
using System.ComponentModel;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;
using Rylogic.Gfx;
using Rylogic.Maths;

namespace Rylogic.Gui.WPF
{
	public partial class ColourWheel : UserControl, INotifyPropertyChanged
	{
		private const string ValueLabel = "V";
		private const string AlphaLabel = "A";
		private EParts m_selected_part;

		public ColourWheel()
		{
			InitializeComponent();
			Parts = EParts.All;
			Orientation = Orientation.Horizontal;
			SliderWidth = 20.0;
			SelectionIndicatorSize = 6.0;
			Colour = Colour32.White;
		}
		protected override void OnMouseDown(MouseButtonEventArgs e)
		{
			base.OnMouseDown(e);

			var pt = e.GetPosition(this);
			m_selected_part = PartHitTest(pt.X, pt.Y);
			if (m_selected_part == EParts.None)
				return;

			ColourSelection?.Invoke(this, e);
			switch (m_selected_part)
			{
			case EParts.Wheel: SelectHueAndSaturation(pt); break;
			case EParts.VSlider: SelectBrightness(pt); break;
			case EParts.ASlider: SelectAlpha(pt); break;
			}

			CaptureMouse();
		}
		protected override void OnMouseMove(MouseEventArgs e)
		{
			base.OnMouseMove(e);

			var pt = e.GetPosition(this);
			switch (m_selected_part)
			{
			case EParts.Wheel: SelectHueAndSaturation(pt); break;
			case EParts.VSlider: SelectBrightness(pt); break;
			case EParts.ASlider: SelectAlpha(pt); break;
			}
		}
		protected override void OnMouseUp(MouseButtonEventArgs e)
		{
			base.OnMouseUp(e);

			var pt = e.GetPosition(this);
			ReleaseMouseCapture();
			switch (m_selected_part)
			{
			case EParts.Wheel: SelectHueAndSaturation(pt); break;
			case EParts.VSlider: SelectBrightness(pt); break;
			case EParts.ASlider: SelectAlpha(pt); break;
			}

			ColourSelection?.Invoke(this, e);
			m_selected_part = EParts.None;
		}
		protected override void OnRenderSizeChanged(SizeChangedInfo sizeInfo)
		{
			base.OnRenderSizeChanged(sizeInfo);
			LayoutDimensions = null!;
		}
		protected override Size MeasureOverride(Size constraint)
		{
			if (double.IsInfinity(constraint.Width) || double.IsInfinity(constraint.Height))
				constraint = new Size(300, 200);

			var meas = CalcMeasurements(constraint);
			return new Size(meas.Width, meas.Height);
		}
		protected override void OnRender(DrawingContext gfx)
		{
			var size = new Size(ActualWidth, ActualHeight);
			if (size.IsEmpty)
			{
				base.OnRender(gfx);
				return;
			}

			var meas = LayoutDimensions;
			var vertical = Orientation == Orientation.Vertical;

			// Draw the wheel
			if ((Parts & EParts.Wheel) != 0)
			{
				var bm = WheelBitmap(meas.Centre, meas.Wheel.Size, meas.Radius);
				gfx.PushClip(new EllipseGeometry(meas.WheelInner));
				gfx.DrawImage(bm, meas.Wheel);
				gfx.Pop();

				// Draw the colour selection
				if ((Parts & EParts.ColourSelection) != 0)
					RenderColourSelection(gfx, WheelPoint(HSVColour));
			}

			// Draw the VSlider
			if ((Parts & EParts.VSlider) != 0)
			{
				gfx.DrawText(this.ToFormattedText(ValueLabel), meas.VLabel.TopLeft);
				if (!meas.VSlider.IsEmpty)
				{
					RenderSlider(gfx, vertical, meas.VSlider);

					// Draw the brightness selection
					if ((Parts & EParts.VSelection) != 0)
						RenderSliderSelector(gfx, vertical, meas.VSlider, HSVColour.V);
				}
			}

			// Draw the ASlider
			if ((Parts & EParts.ASlider) != 0)
			{
				gfx.DrawText(this.ToFormattedText(AlphaLabel), meas.ALabel.TopLeft);
				if (!meas.ASlider.IsEmpty)
				{
					RenderSlider(gfx, vertical, meas.ASlider);

					// Draw the alpha selection
					if ((Parts & EParts.ASelection) != 0)
						RenderSliderSelector(gfx, vertical, meas.ASlider, HSVColour.A);
				}
			}
		}

		/// <summary>
		/// An event raised whenever the act of selecting a colour begins or ends.
		/// Use the button state to tell. Occurs before the first colour change, or after the last colour change</summary>
		public event EventHandler<MouseButtonEventArgs>? ColourSelection;

		/// <summary>An event raised whenever the selected colour is changed</summary>
		public event EventHandler<ColourEventArgs>? ColourChanged;

		/// <summary></summary>
		public event PropertyChangedEventHandler? PropertyChanged;

		/// <summary>Control orientation</summary>
		public Orientation Orientation
		{
			get;
			set
			{
				if (field == value) return;
				field = value;
				LayoutDimensions = null!;
				PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(Orientation)));
			}
		}

		/// <summary>The parts of the control to draw</summary>
		public EParts Parts
		{
			get;
			set
			{
				if (field == value) return;
				field = value;
				LayoutDimensions = null!;
				PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(Parts)));
			}
		}

		/// <summary>The width of the slider bars</summary>
		public double SliderWidth
		{
			get;
			set
			{
				if (field == value) return;
				field = value;
				LayoutDimensions = null!;
				PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(SliderWidth)));
			}
		}

		/// <summary>The radius of the selector dot</summary>
		public double SelectionIndicatorSize
		{
			get;
			set
			{
				if (field == value) return;
				field = value;
				InvalidateVisual();
				PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(SelectionIndicatorSize)));
			}
		}

		/// <summary>The currently selected colour (RGB)</summary>
		public Colour32 Colour
		{
			get => (Colour32)GetValue(ColourProperty);
			set => SetValue(ColourProperty, value);
		}
		private void Colour_Changed()
		{
			ColourChanged?.Invoke(this, new ColourEventArgs(Colour));
			PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(nameof(Colour)));
			InvalidateVisual();
		}
		public static readonly DependencyProperty ColourProperty = Gui_.DPRegister<ColourWheel>(nameof(Colour), Colour32.White, Gui_.EDPFlags.TwoWay);

		/// <summary>The currently selected colour (HSV)</summary>
		public HSV HSVColour
		{
			get => HSV.FromColour32(Colour);
			set => Colour = value.ToColour32();
		}

		/// <summary>Wheel point based on the current dimensions</summary>
		private Point WheelPoint(HSV colour)
		{
			var meas = LayoutDimensions;
			return new Point(
				meas.Centre.X + (meas.Radius * colour.S * Math.Cos(Math_.Tau * colour.H)),
				meas.Centre.Y + (meas.Radius * colour.S * Math.Sin(Math_.Tau * colour.H)));
		}

		/// <summary>Set the hue and saturation based on point X,Y relative to the wheel centre</summary>
		private void SelectHueAndSaturation(Point pt)
		{
			var dim = LayoutDimensions;
			if (dim.Empty || dim.Radius == 0)
				return;

			HSVColour = HSV.FromRadial(
				(float)((pt.X - dim.Centre.X) / dim.Radius),
				(float)((pt.Y - dim.Centre.Y) / dim.Radius),
				HSVColour.V, HSVColour.A);
		}

		/// <summary>Set the brightness based on point x,y relative to the VSlider</summary>
		private void SelectBrightness(Point pt)
		{
			var dim = LayoutDimensions;
			if (dim.Empty || dim.VSlider.IsEmpty)
				return;

			var v = Orientation == Orientation.Vertical
				? Math_.Clamp(Math_.Frac(dim.VSlider.Left, pt.X, dim.VSlider.Right), 0f, 1f)
				: Math_.Clamp(Math_.Frac(dim.VSlider.Bottom, pt.Y, dim.VSlider.Top), 0f, 1f);

			HSVColour = HSV.FromAHSV(HSVColour.A, HSVColour.H, HSVColour.S, (float)v);
		}

		/// <summary>Set the alpha based on point x,y relative to the VSlider</summary>
		private void SelectAlpha(Point pt)
		{
			var dim = LayoutDimensions;
			if (dim.Empty || dim.ASlider.IsEmpty)
				return;

			var a = Orientation == Orientation.Vertical
				? Math_.Clamp(Math_.Frac(dim.ASlider.Left, pt.X, dim.ASlider.Right), 0f, 1f)
				: Math_.Clamp(Math_.Frac(dim.ASlider.Bottom, pt.Y, dim.ASlider.Top), 0f, 1f);

			HSVColour = HSV.FromAHSV((float)a, HSVColour.H, HSVColour.S, HSVColour.V);
		}

		/// <summary>Measure the control layout</summary>
		private Measurements LayoutDimensions
		{
			get => m_measurements ??= CalcMeasurements(new Size(ActualWidth, ActualHeight));
			set => m_measurements = value;
		}
		private Measurements? m_measurements;

		/// <summary>Returns the part hit at location x,y</summary>
		private EParts PartHitTest(double x, double y)
		{
			var dim = LayoutDimensions;
			if (dim.Empty) return EParts.None;
			if (dim.Wheel.Contains(x, y)) return EParts.Wheel;
			if (dim.VSlider.Contains(x, y)) return EParts.VSlider;
			if (dim.ASlider.Contains(x, y)) return EParts.ASlider;
			return EParts.None;
		}

		/// <summary>Get/Create the bitmap of the colour wheel</summary>
		private ImageSource WheelBitmap(Point centre, Size size, double radius)
		{
			if (size.IsEmpty)
				throw new Exception($"Invalid bitmap size");

			// Cached version already the right size? return it
			if (m_bm != null && m_bm.Width == size.Width && m_bm.Height == size.Height)
				return m_bm;

			// Allocate the bitmap, if needed
			var w = (int)Math.Ceiling(size.Width);
			var h = (int)Math.Ceiling(size.Height);
			using var bm = new System.Drawing.Bitmap(w, h, System.Drawing.Imaging.PixelFormat.Format32bppArgb);

			// Silly radius? ignore
			if (radius >= 10)
			{
				// Generate the points and colours for the wheel
				const int ColourCount = 60;
				var points = new System.Drawing.PointF[ColourCount];
				var colours = new System.Drawing.Color[ColourCount];
				for (var i = 0; i != ColourCount; ++i)
				{
					points[i] = new System.Drawing.PointF(
						(float)(centre.X + radius * Math.Cos(i * Math_.Tau / ColourCount)),
						(float)(centre.Y + radius * Math.Sin(i * Math_.Tau / ColourCount)));

					colours[i] = HSV.ToColour32(1f, (float)i / ColourCount, 1f, 1f);
				}

				// Generate a wheel bitmap
				using var gfx = System.Drawing.Graphics.FromImage(bm);
				using var pgb = new System.Drawing.Drawing2D.PathGradientBrush(points);
				gfx.CompositingQuality = System.Drawing.Drawing2D.CompositingQuality.HighQuality;

				// Set the various properties. Note the SurroundColors property, which contains an array of points, 
				// in a one-to-one relationship with the points that created the gradient.
				pgb.CenterColor = System.Drawing.Color.White;
				pgb.CenterPoint = new System.Drawing.PointF((float)centre.X, (float)centre.Y);
				pgb.SurroundColors = colours;

				var r = new System.Drawing.RectangleF((float)(centre.X - w * 0.5), (float)(centre.Y - h * 0.5), w, h);
				gfx.FillEllipse(pgb, r);
			}
			m_bm = bm.ToBitmapSource();
			return m_bm;
		}
		private ImageSource? m_bm; // Cached wheel bitmap

		/// <summary>Draw the selected colour indicator</summary>
		private void RenderColourSelection(DrawingContext gfx, Point pt)
		{
			var radius = SelectionIndicatorSize;
			gfx.DrawEllipse(Colour.ToMediaBrush(), new Pen(Brushes.Black, 1), pt, radius, radius);
		}

		/// <summary>Draw the slider body</summary>
		private void RenderSlider(DrawingContext gfx, bool vertical, Rect r)
		{
			var b = new LinearGradientBrush(Colors.White, Colors.Black,
				vertical ? new Point(0, 0.5) : new Point(0.5, 0),
				vertical ? new Point(1, 0.5) : new Point(0.5, 1));

			gfx.DrawRectangle(b, new Pen(Brushes.Black, 0.5), r);
		}

		/// <summary>Draw the level indicator on a slider</summary>
		private void RenderSliderSelector(DrawingContext gfx, bool vertical, Rect r, double value)
		{
			if (vertical)
			{
				var v = Math_.Lerp(r.Left, r.Right, value);
				gfx.DrawLine(new Pen(Brushes.Black, 1), new Point(v, r.Top), new Point(v, r.Bottom));
				gfx.DrawEllipse(null, new Pen(Brushes.Black, 1), new Point(v, r.Top - 2), 2, 2);
				gfx.DrawEllipse(null, new Pen(Brushes.Black, 1), new Point(v, r.Bottom + 2), 2, 2);
			}
			else
			{
				var v = Math_.Lerp(r.Bottom, r.Top, value);
				gfx.DrawLine(new Pen(Brushes.Black, 1), new Point(r.Left, v), new Point(r.Right, v));
				gfx.DrawEllipse(null, new Pen(Brushes.Black, 1), new Point(r.Left - 2, v), 2, 2);
				gfx.DrawEllipse(null, new Pen(Brushes.Black, 1), new Point(r.Right + 2, v), 2, 2);
			}
		}

		/// <summary>Returns the measurements of a ColourWheel that fits in the given width/height/orientation</summary>
		private Measurements CalcMeasurements(Size size)
		{
			var p = SelectionIndicatorSize;
			var drawv = (Parts & EParts.VSlider) != 0;
			var drawa = (Parts & EParts.ASlider) != 0;
			var vertical = Orientation == Orientation.Vertical;
			var slider_space = 0 + (drawv ? 2 + SliderWidth + 2 : 0) + (drawa ? 2 + SliderWidth + 2 : 0);
			var min = Math.Min(
				Math.Max(0, size.Width - (vertical ? 0 : slider_space)),
				Math.Max(0, size.Height - (vertical ? slider_space : 0)));

			var meas = new Measurements
			{
				Width = min,
				Height = min,
				Radius = Math.Max(0, min * 0.5 - p)
			};
			meas.Centre = new Point(p + Math.Ceiling(meas.Radius), p + Math.Ceiling(meas.Radius));
			meas.Wheel = new Rect(0, 0, meas.Centre.X * 2, meas.Centre.Y * 2);

			Rect LabelBounds(double x, double y, string label)
			{
				var d = new TextBlock { Text = label };
				d.Measure(Size_.Infinity);
				var sz = d.DesiredSize;
				return new Rect(
					(x + (vertical ? 0 : (SliderWidth - sz.Width) * 0.5)),
					(y + (vertical ? (SliderWidth - sz.Height) * 0.5 : 0)),
					sz.Width, sz.Height);
			}
			Rect SliderBounds(double x, double y)
			{
				return new Rect(x, y,
					Math.Max(0, vertical ? meas.Width - x : SliderWidth),
					Math.Max(0, vertical ? SliderWidth : meas.Height - y));
			}

			var ofs = (vertical ? meas.Wheel.Height : meas.Wheel.Width);
			if ((Parts & EParts.VSlider) != 0)
			{
				ofs += 2;
				meas.VLabel = LabelBounds(vertical ? 0 : ofs, vertical ? ofs : 0, ValueLabel);
				meas.VSlider = SliderBounds(vertical ? meas.VLabel.Width + 2 : ofs, vertical ? ofs : meas.VLabel.Height + 2);
				ofs += SliderWidth + 2;
			}
			if ((Parts & EParts.ASlider) != 0)
			{
				ofs += 2;
				meas.ALabel = LabelBounds(vertical ? 0 : ofs, vertical ? ofs : 0, AlphaLabel);
				meas.ASlider = SliderBounds(vertical ? meas.ALabel.Width + 2 : ofs, vertical ? ofs : meas.ALabel.Height + 2);
				ofs += SliderWidth + 2;
			}
			if (vertical)
				meas.Height = ofs;
			else
				meas.Width = ofs;

			return meas;
		}

		/// <summary>Parts of the control</summary>
		[Flags]
		public enum EParts
		{
			None = 0,
			Wheel = 1 << 0,
			VSlider = 1 << 1,
			ASlider = 1 << 2,
			ColourSelection = 1 << 3,
			VSelection = 1 << 4,
			ASelection = 1 << 5,
			All = Wheel | VSlider | ASlider | ColourSelection | VSelection | ASelection,
		}

		/// <summary>Layout measurement</summary>
		private class Measurements
		{
			/// <summary>True if the control has no area</summary>
			public bool Empty => Width == 0 || Height == 0;

			/// <summary>The area covered disk of the colour wheel</summary>
			public Rect WheelInner => new(Centre.X - Radius - 0.5, Centre.Y - Radius - 0.5, 2 * Radius + 1, 2 * Radius + 1);

			/// <summary>The total width required for the control</summary>
			public double Width;

			/// <summary>The total height required for the control</summary>
			public double Height;

			/// <summary>The radius of the colour wheel</summary>
			public double Radius;

			/// <summary>The centre of the colour wheel (clamped to a pixel)</summary>
			public Point Centre;

			/// <summary>The square area required for the wheel+indicator</summary>
			public Rect Wheel;

			/// <summary>The area of the 'V' label</summary>
			public Rect VLabel;

			/// <summary>The area covered by the VSlider</summary>
			public Rect VSlider;

			/// <summary>The area of the 'A' label</summary>
			public Rect ALabel;

			/// <summary>The area covered by the ASlider</summary>
			public Rect ASlider;
		}

		#region EventArgs
		public class ColourEventArgs(Colour32 colour) : EventArgs
		{
			/// <summary>The selected colour</summary>
			public Colour32 Colour { get; } = colour;
		}
		#endregion
	}
}
