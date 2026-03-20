using System;
using System.ComponentModel;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;
using Rylogic.Maths;
using Rylogic.Utility;

namespace Rylogic.Gui.WPF
{
	public partial class BitArray :UserControl, INotifyPropertyChanged
	{
		// Notes:
		//  - "Selected = 1U << Sector"

		public BitArray()
		{
			// Don't set default values for dependency properties because it overrides
			// the default value passed to 'DPRegister' when used in other controls.
			InitializeComponent();
		}
		protected override void OnInitialized(EventArgs e)
		{
			base.OnInitialized(e);
			Dispatcher.BeginInvoke(new Action(UpdateGfx));
		}
		protected override Size MeasureOverride(Size constraint)
		{
			var W = Orientation == Orientation.Horizontal ? BitCount : 1;
			var H = Orientation == Orientation.Horizontal ? 1 : BitCount;
			var sz = new Size(W * (CellW + Spacing), H * (CellH + Spacing));
			var scale = 1.0;

			var desired = base.MeasureOverride(constraint);
			if (Orientation == Orientation.Horizontal && desired.Width != double.PositiveInfinity)
				scale = desired.Width / sz.Width;
			if (Orientation == Orientation.Vertical && desired.Height != double.PositiveInfinity)
				scale = desired.Height / sz.Height;

			return new Size(scale * sz.Width, scale * sz.Height);
		}

		/// <summary>Occurs when a bit is selected/deselected</summary>
		public event EventHandler<BitSelectedEventArgs>? BitSelected;

		/// <summary>Get/Set the single selected bit. Only valid in SingleSelect mode</summary>
		public int Bit
		{
			get
			{
				if (!SingleSelect)
					throw new Exception($"{nameof(BitArray)} control must be in SingleSelect mode for '{nameof(Bit)}' to be valid");

				return BitmaskToBit(Selected);
			}
			set
			{
				if (!SingleSelect)
					throw new Exception($"{nameof(BitArray)} control must be in SingleSelect mode for '{nameof(Bit)}' to be valid");

				Selected ^= BitToBitmask(value);
			}
		}

		/// <summary>The orientation of the bit array</summary>
		public Orientation Orientation
		{
			get => (Orientation)GetValue(MyPropertyProperty);
			set => SetValue(MyPropertyProperty, value);
		}
		private void Orientation_Changed()
		{
			UpdateGfx();
		}
		public static readonly DependencyProperty MyPropertyProperty = Gui_.DPRegister<BitArray>(nameof(Orientation), Orientation.Horizontal, Gui_.EDPFlags.None);

		/// <summary>Get/Set the currently selected bits</summary>
		public ulong Selected
		{
			get => (ulong)GetValue(SelectedProperty);
			set
			{
				if (Selected == value) return;
				var new_value = value;

				if (SingleSelect)
					new_value = Common.Bit.LowBit(value ^ Selected);

				SetValue(SelectedProperty, new_value);
			}
		}
		private void Selected_Changed(ulong new_value, ulong old_value)
		{
			BitSelected?.Invoke(this, new BitSelectedEventArgs(new_value ^ old_value, new_value, SingleSelect ? (int?)Bit : null));
			NotifyPropertyChanged(nameof(Selected));
			UpdateGfx();
		}
		public static readonly DependencyProperty SelectedProperty = Gui_.DPRegister<BitArray>(nameof(Selected), 1UL, Gui_.EDPFlags.TwoWay);

		/// <summary>One or multiple bits selected simultaneously</summary>
		public bool SingleSelect
		{
			get => (bool)GetValue(SingleSelectProperty);
			set => SetValue(SingleSelectProperty, value);
		}
		private void SingleSelect_Changed()
		{
			// Remove extra bits if single selecting
			SetValue(SelectedProperty, Common.Bit.LowBit(Selected));
			UpdateGfx();
		}
		public static readonly DependencyProperty SingleSelectProperty = Gui_.DPRegister<BitArray>(nameof(SingleSelect), Boxed.False, Gui_.EDPFlags.None);

		/// <summary>The number of bits displayed</summary>
		public int BitCount
		{
			get => (int)GetValue(BitCountProperty);
			set => SetValue(BitCountProperty, value);
		}
		private void BitCount_Changed()
		{
			if (BitCount < 1 || BitCount > 64)
				throw new Exception($"BitCount={BitCount} is out of range");

			UpdateGfx();
		}
		public static readonly DependencyProperty BitCountProperty = Gui_.DPRegister<BitArray>(nameof(BitCount), 8, Gui_.EDPFlags.None);

		/// <summary>True if the LSB is displayed on the left</summary>
		public bool LSBIsLeft
		{
			get => (bool)GetValue(LSBIsLeftProperty);
			set => SetValue(LSBIsLeftProperty, value);
		}
		private void LSBIsLeft_Changed()
		{
			UpdateGfx();
		}
		public static readonly DependencyProperty LSBIsLeftProperty = Gui_.DPRegister<BitArray>(nameof(LSBIsLeft), Boxed.False, Gui_.EDPFlags.None);

		/// <summary>The gap between bit cells</summary>
		public double Spacing
		{
			get => (double)GetValue(SpacingProperty);
			set => SetValue(SpacingProperty, value);
		}
		private void Spacing_Changed()
		{
			UpdateGfx();
		}
		public static readonly DependencyProperty SpacingProperty = Gui_.DPRegister<BitArray>(nameof(Spacing), 0.0, Gui_.EDPFlags.None);

		/// <summary>True if the selected bits cannot be changed via the UI</summary>
		public bool IsReadOnly
		{
			get => (bool)GetValue(IsReadOnlyProperty);
			set => SetValue(IsReadOnlyProperty, value);
		}
		public static readonly DependencyProperty IsReadOnlyProperty = Gui_.DPRegister<BitArray>(nameof(IsReadOnly), Boxed.False, Gui_.EDPFlags.None);

		/// <summary>The brush used to fill the selected sectors</summary>
		public Brush SelectedBitBrush
		{
			get => (Brush)GetValue(SelectedBitBrushProperty);
			set => SetValue(SelectedBitBrushProperty, value);
		}
		private void SelectedBitBrush_Changed()
		{
			UpdateGfx();
		}
		public static readonly DependencyProperty SelectedBitBrushProperty = Gui_.DPRegister<BitArray>(nameof(SelectedBitBrush), Brushes.DarkGreen, Gui_.EDPFlags.None);

		/// <summary>The width of the BitArray outlines</summary>
		public double StrokeWidth
		{
			get => (double)GetValue(StrokeWidthProperty);
			set => SetValue(StrokeWidthProperty, value);
		}
		private void StrokeWidth_Changed()
		{
			UpdateGfx();
		}
		public static readonly DependencyProperty StrokeWidthProperty = Gui_.DPRegister<BitArray>(nameof(StrokeWidth), 1.0, Gui_.EDPFlags.None);

		/// <summary>The colour of the BitArray outlines</summary>
		public Brush StrokeColour
		{
			get => (Brush)GetValue(StrokeColourProperty);
			set => SetValue(StrokeColourProperty, value);
		}
		private void StrokeColour_Changed()
		{
			UpdateGfx();
		}
		public static readonly DependencyProperty StrokeColourProperty = Gui_.DPRegister<BitArray>(nameof(StrokeColour), Brushes.Black, Gui_.EDPFlags.None);

		/// <summary>The background colour</summary>
		public Brush CellBackground
		{
			get => (Brush)GetValue(CellBackgroundProperty);
			set => SetValue(CellBackgroundProperty, value);
		}
		private void CellBackground_Changed()
		{
			UpdateGfx();
		}
		public static readonly DependencyProperty CellBackgroundProperty = Gui_.DPRegister<BitArray>(nameof(CellBackground), Brushes.Transparent, Gui_.EDPFlags.None);

		/// <summary>Update the graphics for the control</summary>
		private void UpdateGfx()
		{
			// The scale is somewhat arbitrary, but pen widths are absolute, so
			// drawing as a unit circle doesn't work because the pen width is 1.
			// We can draw with (0,0) at the centre however.
			var W = Orientation == Orientation.Horizontal ? BitCount : 1;
			var H = Orientation == Orientation.Horizontal ? 1 : BitCount;

			var drawing = new DrawingGroup();

			// Fill the background (draw even if transparent because it affects the control size)
			{
				var rect = new Rect(0, 0, W * (CellW + Spacing), H * (CellH + Spacing));
				var bkgd = new GeometryDrawing(Background, new Pen(Brushes.Black, 0.0), new RectangleGeometry(rect));
				drawing.Children.Add(bkgd.Freeze2());
			}

			// Create the cells
			{
				var path = new PathGeometry();
				for (var h = 0; h != H; ++h)
				{
					for (var w = 0; w != W; ++w)
					{
						var fig = new PathFigure { IsFilled = true, IsClosed = true, };
						var x = Spacing/2 + w * (CellW + Spacing);
						var y = Spacing/2 + h * (CellH + Spacing);
						fig.StartPoint =                 new Point(x + 0    , y + 0);
						fig.Segments.Add(new LineSegment(new Point(x + CellW, y + 0), true));
						fig.Segments.Add(new LineSegment(new Point(x + CellW, y + CellH), true));
						fig.Segments.Add(new LineSegment(new Point(x + 0    , y + CellH), true));
						path.Figures.Add(fig);
					}
				}
				var cell = new GeometryDrawing(CellBackground, new Pen(StrokeColour, StrokeWidth), path);
				drawing.Children.Add(cell.Freeze2());
			}

			// Create the filled bits
			{
				var path = new PathGeometry();
				var selected = LSBIsLeft ? Selected : Common.Bit.ReverseBits(Selected) >> (64-BitCount);
				for (var h = 0; h != H; ++h)
				{
					for (var w = 0; w != W; ++w)
					{
						if (Common.Bit.AllSet(selected, 1U))
						{
							var fig = new PathFigure { IsFilled = true, IsClosed = true, };
							var x = Spacing/2 + w * (CellW + Spacing);
							var y = Spacing/2 + h * (CellH + Spacing);

							fig.StartPoint =                 new Point(x + 0.1 * CellW, y + 0.1 * CellH);
							fig.Segments.Add(new LineSegment(new Point(x + 0.9 * CellW, y + 0.1 * CellH), true));
							fig.Segments.Add(new LineSegment(new Point(x + 0.9 * CellW, y + 0.9 * CellH), true));
							fig.Segments.Add(new LineSegment(new Point(x + 0.1 * CellW, y + 0.9 * CellH), true));
							path.Figures.Add(fig);
						}
						selected >>= 1;
					}
				}
				var bit = new GeometryDrawing(SelectedBitBrush, new Pen(Brushes.Transparent, StrokeWidth), path);
				drawing.Children.Add(bit.Freeze2());
			}

			// Turn the geometry into a drawing
			PART_Image.Source = new DrawingImage(drawing);
		}

		/// <summary>Return the bit under 'client_pt' (where 'client_pt' is in image client space)</summary>
		private int? BitFromImagePoint(Point image_pt)
		{
			var W = Orientation == Orientation.Horizontal ? BitCount : 1;
			var H = Orientation == Orientation.Horizontal ? 1 : BitCount;
			var FracX = 0.5 * Spacing / (CellW + Spacing);
			var FracY = 0.5 * Spacing / (CellH + Spacing);

			var pt = new Point(
				W * image_pt.X / PART_Image.ActualWidth,
				H * image_pt.Y / PART_Image.ActualHeight);

			var bit = (int)Math.Truncate(Orientation == Orientation.Horizontal ? pt.X : pt.Y);

			pt.X = pt.X - Math.Truncate(pt.X);
			pt.Y = pt.Y - Math.Truncate(pt.Y);
			if (pt.X < FracX || pt.X > 1.0 - FracX ||
				pt.Y < FracY || pt.Y > 1.0 - FracY)
				return null;

			return LSBIsLeft ? bit : BitCount - bit - 1;
		}

		/// <summary>Handle mouse clicks</summary>
		private void HandleMouseDown(object sender, MouseButtonEventArgs e)
		{
			if (IsReadOnly)
				return;

			// Determine the selected sector from 'pt'
			var pt = e.GetPosition(PART_Image);
			if (BitFromImagePoint(pt) is int bit)
			{
				Selected ^= BitToBitmask(bit);
			}

			UpdateGfx();
		}

		/// <inheritdoc/>
		public event PropertyChangedEventHandler? PropertyChanged;
		private void NotifyPropertyChanged(string prop_name) => PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(prop_name));

		/// <summary>Convert a BitArray bit value to a selection bitmask</summary>
		public static ulong BitToBitmask(int bit)
		{
			return 1UL << bit;
		}

		/// <summary>Convert a BitArray selection to a bit index</summary>
		public static int BitmaskToBit(ulong selected)
		{
			if (Common.Bit.CountBits(selected) > 1)
				throw new Exception("Bitmask must contain 0 or 1 bits only");

			return Common.Bit.LowBitIndex(selected);
		}

		private const double CellW = 100;
		private const double CellH = 100;

		#region EventArgs
		public class BitSelectedEventArgs :EventArgs
		{
			public BitSelectedEventArgs(ulong changed_bits, ulong selected, int? bit)
			{
				Bits = changed_bits;
				Selected = selected;
				Bit = bit;
			}

			/// <summary>The bits that changed</summary>
			public ulong Bits { get; }

			/// <summary>The bits currently selected</summary>
			public ulong Selected { get; }

			/// <summary>The single selected bit (Valid only in SingleSelect mode)</summary>
			public int? Bit { get; }
		}
		#endregion
	}
}
