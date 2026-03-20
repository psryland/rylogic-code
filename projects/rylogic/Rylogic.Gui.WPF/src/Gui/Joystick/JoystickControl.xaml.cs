using System;
using System.ComponentModel;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;
using Rylogic.Common;
using Rylogic.Maths;
using Rylogic.Utility;

namespace Rylogic.Gui.WPF
{
	public partial class JoystickControl :UserControl, INotifyPropertyChanged
	{
		// Notes:
		//  - "Selected = 1U << Sector"
		//  - The neutral position is: Sector == 0, Selected = 1U
		//  - The centre circle is Ring0
		//  - Ring1 has SectorCount sectors
		//  - The number of sectors doubles for each ring

		/// <summary>The native radius of the control</summary>
		private const double Radius = 128.0;
		private readonly ImageSource ThumbImage;

		public JoystickControl()
		{
			// Don't set default values for dependency properties because it overrides
			// the default value passed to 'DPRegister' when used in other controls.
			InitializeComponent();
			ThumbImage = (ImageSource)FindResource("thumb");
		}
		protected override void OnInitialized(EventArgs e)
		{
			base.OnInitialized(e);
			Dispatcher.BeginInvoke(new Action(UpdateGfx));
		}
		protected override Size MeasureOverride(Size constraint)
		{
			var desired = base.MeasureOverride(constraint);
			var sz = Math.Min(desired.Width, desired.Height);
			return sz != double.PositiveInfinity ? new Size(sz, sz) : new Size(256, 256);
		}

		/// <summary>Occurs when a sector is selected/deselected</summary>
		public event EventHandler<SectorsSelectedEventArgs>? SectorSelected;

		/// <summary>Get/Set the single selected sector. Only valid in SingleSelect mode</summary>
		public int Sector
		{
			get
			{
				if (!SingleSelect)
					throw new Exception("Joystick control must be in SingleSelect mode for 'Sector' to be valid");

				return BitmaskToSector(Selected);
			}
			set
			{
				if (!SingleSelect)
					throw new Exception("Joystick control must be in SingleSelect mode for 'Sector' to be valid");

				Selected ^= SectorToBitmask(value);
			}
		}

		/// <summary>Return the angle (in radians) of the selected sector [0,Tau]</summary>
		public double Angle
		{
			get
			{
				if (!SingleSelect) return 0;
				if (Selected == 1) return 0;
				if (RingCount == 0) return 0;

				var sector = Bit.Index(Selected >> 1);

				// Find the sector index and number of sectors within
				// the ring that the selected sector is in.
				var sector_count = SectorCount;
				for (; sector >= sector_count;)
				{
					sector -= sector_count;
					sector_count *= 2;
				}

				// Get the angle from the sector
				var ang = Math_.Tau + (Math_.Tau * sector / sector_count) + Math_.DegreesToRadians(NorthOffset);
				return ang % Math_.Tau;
			}
		}
		public double AngleDeg => Math_.RadiansToDegrees(Angle);

		/// <summary>Return the normalised deflection from the neutral position [0,1]</summary>
		public double Deflection
		{
			get
			{
				if (!SingleSelect) return 0;
				if (Selected == 1) return 0;
				if (RingCount == 0) return 0;

				var sector = Bit.Index(Selected >> 1);

				// Find the sector index and number of sectors within
				// the ring that the selected sector is in.
				var ring = 1.0;
				var sector_count = SectorCount;
				for (; sector >= sector_count;)
				{
					sector -= sector_count;
					sector_count *= 2;
					ring += 1.0;
				}

				return ring / RingCount;
			}
		}

		/// <summary>Returns the digital position of the joystick as a 4-bit mask: YYXX where 11=negative, 00=0, 01=positive</summary>
		public int DigitalPosition
		{
			get
			{
				if (!SingleSelect) return 0;
				if (Selected == 1) return 0;
				if (RingCount == 0) return 0;

				int x = 0, y = 0;
				if (Deflection > 0.5f)
				{
					var ang = Angle;
					if (ang > Math_.Tau * 1 / 8 && ang < Math_.Tau * 3 / 8) x = +1;
					if (ang > Math_.Tau * 5 / 8 && ang < Math_.Tau * 7 / 8) x = -1;
					if (ang < Math_.Tau * 1 / 8 || ang > Math_.Tau * 7 / 8) y = +1;
					if (ang > Math_.Tau * 3 / 8 && ang < Math_.Tau * 5 / 8) y = -1;
				}
				return ((y & 3) << 2) | ((x & 3) << 0);
			}
		}

		/// <summary>Get/Set the currently selected bits</summary>
		public ulong Selected
		{
			get => (ulong)GetValue(SelectedProperty);
			set
			{
				if (Selected == value) return;
				var new_value = value;

				// In single select mode, clicking the same sector toggles between
				// that sector and the centre sector. Expected use however is 'Selected ^= 1U << index'
				// so we need to handle this toggling for this case as well
				if (SingleSelect)
					new_value = (value & Selected) != 0 ? Bit.LowBit(value ^ Selected) : 1;

				SetValue(SelectedProperty, new_value);
			}
		}
		private void Selected_Changed(ulong new_value, ulong old_value)
		{
			SectorSelected?.Invoke(this, new SectorsSelectedEventArgs(new_value ^ old_value, new_value, SingleSelect ? (int?)Sector : null));
			NotifyPropertyChanged(nameof(Selected));
			if (SingleSelect)
			{
				NotifyPropertyChanged(nameof(Angle));
				NotifyPropertyChanged(nameof(AngleDeg));
				NotifyPropertyChanged(nameof(Deflection));
				NotifyPropertyChanged(nameof(DigitalPosition));
			}
			UpdateGfx();
		}
		public static readonly DependencyProperty SelectedProperty = Gui_.DPRegister<JoystickControl>(nameof(Selected), 1UL, Gui_.EDPFlags.TwoWay);

		/// <summary>The offset from 'North' for sector 1 (in degrees)</summary>
		public double NorthOffset
		{
			// Notes:
			//  - This rotates the position of the sectors, but does not change the 'Angle' value.
			//    'Angle == 0' is always north.
			//  - If you want to change the Angle, use a render transform. e.g.
			//       RenderTransform="0 -1 1 0 0 0"
			//       RenderTransformOrigin="0.5,0.5"
			get => (double)GetValue(NorthOffsetProperty);
			set => SetValue(NorthOffsetProperty, value);
		}
		private void NorthOffset_Changed()
		{
			UpdateGfx();
		}
		public static readonly DependencyProperty NorthOffsetProperty = Gui_.DPRegister<JoystickControl>(nameof(NorthOffset), 0.0, Gui_.EDPFlags.None);

		/// <summary>One or multiple sectors selected simultaneously</summary>
		public bool SingleSelect
		{
			get => (bool)GetValue(SingleSelectProperty);
			set => SetValue(SingleSelectProperty, value);
		}
		private void SingleSelect_Changed()
		{
			Selected = Bit.LowBit(Selected);
			UpdateGfx();
		}
		public static readonly DependencyProperty SingleSelectProperty = Gui_.DPRegister<JoystickControl>(nameof(SingleSelect), Boxed.False, Gui_.EDPFlags.None);

		/// <summary>The number of concentric rings</summary>
		public int RingCount
		{
			get => (int)GetValue(RingCountProperty);
			set => SetValue(RingCountProperty, value);
		}
		private void RingCount_Changed()
		{
			UpdateGfx();
		}
		public static readonly DependencyProperty RingCountProperty = Gui_.DPRegister<JoystickControl>(nameof(RingCount), 1, Gui_.EDPFlags.None);

		/// <summary>The number of sectors in the inner ring</summary>
		public int SectorCount
		{
			get => (int)GetValue(SectorCountProperty);
			set => SetValue(SectorCountProperty, value);
		}
		private void SectorCount_Changed()
		{
			UpdateGfx();
		}
		public static readonly DependencyProperty SectorCountProperty = Gui_.DPRegister<JoystickControl>(nameof(SectorCount), 8, Gui_.EDPFlags.None);

		/// <summary>True if the selected sectors cannot be changed via the UI</summary>
		public bool IsReadOnly
		{
			get => (bool)GetValue(IsReadOnlyProperty);
			set => SetValue(IsReadOnlyProperty, value);
		}
		public static readonly DependencyProperty IsReadOnlyProperty = Gui_.DPRegister<JoystickControl>(nameof(IsReadOnly), Boxed.False, Gui_.EDPFlags.None);

		/// <summary>The brush used to fill the non-selected sectors</summary>
		public Brush SectorBrush
		{
			get => (Brush)GetValue(SectorBrushProperty);
			set => SetValue(SectorBrushProperty, value);
		}
		private void SectorBrush_Changed()
		{
			UpdateGfx();
		}
		public static readonly DependencyProperty SectorBrushProperty = Gui_.DPRegister<JoystickControl>(nameof(SectorBrush), Brushes.WhiteSmoke, Gui_.EDPFlags.None);

		/// <summary>The brush used to fill the selected sectors</summary>
		public Brush SelectedSectorBrush
		{
			get => (Brush)GetValue(SelectedSectorBrushProperty);
			set => SetValue(SelectedSectorBrushProperty, value);
		}
		private void SelectedSectorBrush_Changed()
		{
			UpdateGfx();
		}
		public static readonly DependencyProperty SelectedSectorBrushProperty = Gui_.DPRegister<JoystickControl>(nameof(SelectedSectorBrush), Brushes.DarkGreen, Gui_.EDPFlags.None);

		/// <summary>The width of the sector outlines</summary>
		public double StrokeWidth
		{
			get => (double)GetValue(StrokeWidthProperty);
			set => SetValue(StrokeWidthProperty, value);
		}
		private void StrokeWidth_Changed()
		{
			UpdateGfx();
		}
		public static readonly DependencyProperty StrokeWidthProperty = Gui_.DPRegister<JoystickControl>(nameof(StrokeWidth), 1.0, Gui_.EDPFlags.None);

		/// <summary>The colour of the sector outlines</summary>
		public Brush StrokeColour
		{
			get => (Brush)GetValue(StrokeColourProperty);
			set => SetValue(StrokeColourProperty, value);
		}
		private void StrokeColour_Changed()
		{
			UpdateGfx();
		}
		public static readonly DependencyProperty StrokeColourProperty = Gui_.DPRegister<JoystickControl>(nameof(StrokeColour), Brushes.Black, Gui_.EDPFlags.None);

		/// <summary>The position of the thumb stick (in polar coords, i.e. [angle (radians), deflection (0->1)]). Null to hide</summary>
		public Point? ThumbPosition
		{
			get => (Point?)GetValue(ThumbPositionProperty);
			set => SetValue(ThumbPositionProperty, value);
		}
		private void ThumbPosition_Changed()
		{
			UpdateGfx();
		}
		public static readonly DependencyProperty ThumbPositionProperty = Gui_.DPRegister<JoystickControl>(nameof(ThumbPosition), null, Gui_.EDPFlags.TwoWay);

		/// <summary>Update the graphics for the control</summary>
		private void UpdateGfx()
		{
			// The scale is somewhat arbitrary, but pen widths are absolute, so
			// drawing as a unit circle doesn't work because the pen width is 1.
			// We can draw with (0,0) at the centre however.

			var centre = new Point(0, 0);
			var dr = Radius / (1 + 2*RingCount);
			var selected = Selected >> 1;

			var drawing = new DrawingGroup();

			// Fill the background
			if (SectorBrush != Brushes.Transparent)
			{
				var bkgd = new GeometryDrawing(SectorBrush, new Pen(Brushes.Black, 0.0), new EllipseGeometry(centre, Radius, Radius));
				drawing.Children.Add(bkgd);
			}

			// Create the sectors in order from inner to outer
			var path = new PathGeometry();
			for (int r = 0; r != RingCount; ++r)
			{
				// Each ring doubles the number of sectors from the inner ring
				var sector_count = SectorCount << r;
				var ang = Math_.Tau / sector_count;
				var ofs = Math_.DegreesToRadians(NorthOffset);
				var rad0 = dr * (1 + 2*r);
				var rad1 = dr * (3 + 2*r);

				for (int s = 0; s != sector_count; ++s)
				{
					var s0 = Math.Sin(ang * (s - 0.5) + ofs);
					var c0 = Math.Cos(ang * (s - 0.5) + ofs);
					var s1 = Math.Sin(ang * (s + 0.5) + ofs);
					var c1 = Math.Cos(ang * (s + 0.5) + ofs);
					var a = centre + rad0 * new Vector(s0, -c0);
					var b = centre + rad1 * new Vector(s0, -c0);
					var c = centre + rad1 * new Vector(s1, -c1);
					var d = centre + rad0 * new Vector(s1, -c1);

					// Create non-filled wedges for each sector except those that are selected
					var fig = new PathFigure { IsFilled = Bit.AllSet(selected, 1U), IsClosed = true, StartPoint = a };
					if (sector_count != 1)
					{
						fig.Segments.Add(new LineSegment(b, true));
						fig.Segments.Add(new ArcSegment(c, new Size(rad1, rad1), 0, false, SweepDirection.Clockwise, true));
						fig.Segments.Add(new LineSegment(d, true));
						fig.Segments.Add(new ArcSegment(a, new Size(rad0, rad0), 0, false, SweepDirection.Counterclockwise, true));
					}
					else
					{
						// Need to special case drawing a complete circle because 'ArcSegment' can't represent it.
						// see: http://www.charlespetzold.com/blog/2008/01/Mathematics-of-ArcSegment.html
						var e = centre + rad1 * new Vector(0, -1);
						var f = centre + rad0 * new Vector(0, -1);
						fig.Segments.Add(new LineSegment(b, true));
						fig.Segments.Add(new ArcSegment(e, new Size(rad1, rad1), 0, false, SweepDirection.Clockwise, true));
						fig.Segments.Add(new ArcSegment(c, new Size(rad1, rad1), 0, false, SweepDirection.Clockwise, true));
						fig.Segments.Add(new LineSegment(d, true));
						fig.Segments.Add(new ArcSegment(f, new Size(rad0, rad0), 0, false, SweepDirection.Counterclockwise, true));
						fig.Segments.Add(new ArcSegment(a, new Size(rad0, rad0), 0, false, SweepDirection.Counterclockwise, true));

					}
					path.Figures.Add(fig);
					selected >>= 1;
				}
			}

			// Create the neutral sector
			{
				var a = centre + dr * new Vector(0, -1);
				var b = centre + dr * new Vector(0, +1);
				var fig = new PathFigure { IsFilled = Bit.AllSet(Selected, 1U), IsClosed = true, StartPoint = a };
				fig.Segments.Add(new ArcSegment(b, new Size(dr, dr), 0, true, SweepDirection.Clockwise, true));
				fig.Segments.Add(new ArcSegment(a, new Size(dr, dr), 0, true, SweepDirection.Clockwise, true));
				path.Figures.Add(fig);
			}
			
			// Turn the geometry into a drawing
			var sector_drawing = new GeometryDrawing(SelectedSectorBrush, new Pen(StrokeColour, StrokeWidth), path).Freeze2();
			drawing.Children.Add(sector_drawing);

			// If the thumb position in not null, add the thumb graphic
			if (ThumbPosition is Point thumb)
			{
				var thumb_radius = 0.25 * Radius;
				var max_deflection = Radius - thumb_radius;

				// Convert from polar coords to rect
				var rect = new Rect(
					max_deflection * thumb.Y * Math.Cos(thumb.X) - thumb_radius,
					max_deflection * thumb.Y * Math.Sin(thumb.X) - thumb_radius,
					2 * thumb_radius, 2 * thumb_radius);

				var thumb_bitmap = new ImageDrawing(ThumbImage, rect);
				drawing.Children.Add(thumb_bitmap);
			}

			m_image.Source = new DrawingImage(drawing);
		}

		/// <summary>Return the sector under 'client_pt' (where 'client_pt' is in client space)</summary>
		private int? SectorFromClientPoint(Point client_pt)
		{
			// The joystick graphic is a square within the client area of the control.
			var sz = Math.Min(ActualWidth, ActualHeight) * 0.5;
			var unit_pt = new Point(
				(client_pt.X - 0.5 * ActualWidth) / sz,
				(0.5 * ActualHeight - client_pt.Y) / sz);
			return SectorFromUnitPoint(unit_pt);
		}

		/// <summary>Return the sector under 'unit_pt' (where 'unit_pt' have components on [-1,+1])</summary>
		private int? SectorFromUnitPoint(Point unit_pt)
		{
			var centre = new Point(0, 0);
			var pt = Radius * (unit_pt - centre);
			var dr = Radius / (1 + 2 * RingCount);

			// Determine the ring that 'pt' is in.
			var rad = pt.Length;
			if (rad < dr) return 0;
			if (rad >= Radius) return null;
			var ring = (int)Math_.Clamp(0.5 * (rad / dr - 1), 0, RingCount - 1);

			// Determine the sector within the ring that 'pt' is in.
			var sect = 0;
			for (int r = 0; r != ring; ++r)
				sect += SectorCount << r;

			// Find the sector corresponding to the radial angle
			var sector_count = SectorCount << ring;
			var ang = Math.Atan2(pt.X, pt.Y) + Math_.Tau; // 'ang' in [tau/2, 3*tau/2]
			ang -= Math_.DegreesToRadians(NorthOffset);   // Adjust for north offset
			
			// Map the angle to a sector index
			sect += 1 + (int)(0.5f + sector_count * ang / Math_.Tau) % sector_count;
			return sect;
		}

		/// <summary>Handle mouse clicks</summary>
		private void HandleMouseDown(object sender, MouseButtonEventArgs e)
		{
			if (IsReadOnly)
				return;

			// Determine the selected sector from 'pt'
			var pt = e.GetPosition(this);
			if (SectorFromClientPoint(pt) is int sector)
			{
				Selected ^= SectorToBitmask(sector);
			}

			UpdateGfx();
		}

		/// <summary></summary>
		public event PropertyChangedEventHandler? PropertyChanged;
		private void NotifyPropertyChanged(string prop_name)
		{
			PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(prop_name));
		}

		/// <summary>Convert a joystick sector value to a selection bitmask</summary>
		public static ulong SectorToBitmask(int sector)
		{
			return 1UL << sector;
		}

		/// <summary>Convert a joystick selection to a sector index</summary>
		public static int BitmaskToSector(ulong selected)
		{
			if (Bit.CountBits(selected) > 1)
				throw new Exception("Bitmask must contain 0 or 1 bits only");

			return Bit.LowBitIndex(selected);
		}

		#region EventArgs
		public class SectorsSelectedEventArgs :EventArgs
		{
			public SectorsSelectedEventArgs(ulong changed_bits, ulong selected, int? sector)
			{
				Bits = changed_bits;
				Selected = selected;
				Sector = sector;
			}

			/// <summary>The bits that changed</summary>
			public ulong Bits { get; }

			/// <summary>The bits currently selected</summary>
			public ulong Selected { get; }

			/// <summary>The single selected sector (Valid only in SingleSelect mode)</summary>
			public int? Sector { get; }
		}
		#endregion
	}
}
