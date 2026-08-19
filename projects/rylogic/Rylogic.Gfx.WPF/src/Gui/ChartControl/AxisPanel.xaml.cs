using System;
using System.ComponentModel;
using System.Linq;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Shapes;
using Rylogic.Windows.Extn;

namespace Rylogic.Gui.WPF.ChartDetail
{
	public sealed partial class AxisPanel :Canvas, IDisposable, INotifyPropertyChanged, IChartAxisCMenu, IChartAxisCMenuContext
	{
		// Notes:
		//  - Represents the tick marks and tick labels of an axis.
		//  - This component is intended to be able to go on any size of the graph.

		public AxisPanel()
		{
			InitializeComponent();

			// Commands
			ToggleScrollLock = Command.Create(this, ToggleScrollLockInternal);
			ToggleZoomLock = Command.Create(this, ToggleZoomLockInternal);
		}
		protected override void OnRenderSizeChanged(SizeChangedInfo sizeInfo)
		{
			base.OnRenderSizeChanged(sizeInfo);
			Axis?.Gfx.Invalidate();

			// Also redraw the axis labels/tick marks since the tick spacing
			// depends on the pixel dimensions (via GridLines → PixelsPerTick).
			Invalidate();
		}
		public void Dispose()
		{
			Axis = null;
		}

		/// <summary>Font for tick marks</summary>
		public FontFamily FontFamily
		{
			get => (FontFamily)GetValue(FontFamilyProperty);
			set => SetValue(FontFamilyProperty, value);
		}
		public static readonly DependencyProperty FontFamilyProperty = Gui_.DPRegister<AxisPanel>(nameof(FontFamily), new FontFamily("tahoma"), Gui_.EDPFlags.None);

		/// <summary>Font style</summary>
		public FontStyle FontStyle
		{
			get => (FontStyle)GetValue(FontStyleProperty);
			set => SetValue(FontStyleProperty, value);
		}
		public static readonly DependencyProperty FontStyleProperty = Gui_.DPRegister<AxisPanel>(nameof(FontStyle), FontStyles.Normal, Gui_.EDPFlags.None);

		/// <summary>Font weight</summary>
		public FontWeight FontWeight
		{
			get => (FontWeight)GetValue(FontWeightProperty);
			set => SetValue(FontWeightProperty, value);
		}
		public static readonly DependencyProperty FontWeightProperty = Gui_.DPRegister<AxisPanel>(nameof(FontWeight), FontWeights.Normal, Gui_.EDPFlags.None);

		/// <summary>Font stretch</summary>
		public FontStretch FontStretch
		{
			get => (FontStretch)GetValue(FontStretchProperty);
			set => SetValue(FontStretchProperty, value);
		}
		public static readonly DependencyProperty FontStretchProperty = Gui_.DPRegister<AxisPanel>(nameof(FontStretch), FontStretches.Normal, Gui_.EDPFlags.None);

		/// <summary>Font size for tick labels</summary>
		public double FontSize
		{
			get => (double)GetValue(FontSizeProperty);
			set => SetValue(FontSizeProperty, value);
		}
		public static readonly DependencyProperty FontSizeProperty = Gui_.DPRegister<AxisPanel>(nameof(FontSize), 10.0, Gui_.EDPFlags.None);

		/// <summary>Font for tick labels</summary>
		public Typeface Typeface => new(FontFamily, FontStyle, FontWeight, FontStretch);

		/// <summary>The axis represented by this visual</summary>
		public ChartControl.RangeData.Axis? Axis
		{
			get => (ChartControl.RangeData.Axis?)GetValue(AxisProperty);
			set => SetValue(AxisProperty, value);
		}
		private void Axis_Changed(ChartControl.RangeData.Axis new_value, ChartControl.RangeData.Axis old_value)
		{
			if (old_value != null)
			{
				Options.PropertyChanged -= HandleOptionChanged;
				old_value.Scroll -= HandleMoved;
				old_value.Zoomed -= HandleMoved;
			}
			if (new_value != null)
			{
				new_value.Zoomed += HandleMoved;
				new_value.Scroll += HandleMoved;
				Options.PropertyChanged += HandleOptionChanged;
			}

			// Invalidate cached values
			AxisSize = 0;

			// Notify properties changed
			NotifyPropertyChanged(nameof(Options));
			UpdateGraphics();

			// Handlers
			void HandleMoved(object? sender, EventArgs e)
			{
				Invalidate();
			}
			void HandleOptionChanged(object? sender, PropertyChangedEventArgs e)
			{
				switch (e.PropertyName)
				{
					case nameof(ChartControl.OptionsData.Axis.Side):
						AxisSize = 0;
						UpdateGraphics();
						break;
					case nameof(ChartControl.OptionsData.Axis.AxisColour):
					case nameof(ChartControl.OptionsData.Axis.AxisThickness):
						UpdateGraphics();
						break;
					case nameof(ChartControl.OptionsData.Axis.DrawTickLabels):
					case nameof(ChartControl.OptionsData.Axis.DrawTickMarks):
						AxisSize = 0;
						NotifyPropertyChanged(nameof(AxisSize));
						UpdateGraphics();
						break;
					case nameof(ChartControl.OptionsData.Axis.TickTextTemplate):
						AxisSize = 0;
						NotifyPropertyChanged(nameof(AxisSize));
						UpdateGraphics();
						break;
				}
			}
		}
		public static readonly DependencyProperty AxisProperty = Gui_.DPRegister<AxisPanel>(nameof(Axis), null, Gui_.EDPFlags.None);

		/// <summary>The axis options</summary>
		public ChartControl.OptionsData.Axis Options => Axis?.Options ?? new ChartControl.OptionsData.Axis();

		/// <summary>The size of the axis area</summary>
		public double AxisSize
		{
			get
			{
				if (m_axis_size == null)
				{
					// Assume no axis to start with
					m_axis_size = 0;

					// Add space for tick marks
					if (Options.DrawTickMarks)
						m_axis_size += Options.TickLength;

					// Add space for tick labels
					if (Options.DrawTickLabels)
					{
						var measure = new TextBlock
						{
							Text = Options.TickTextTemplate,
							TextWrapping = TextWrapping.Wrap,
							RenderTransform = Options.LabelTransform,
							RenderTransformOrigin = Options.LabelTransformOrigin,
						};
						measure.Typeface(Typeface, FontSize);
						measure.Measure(Size_.Infinity);
						m_axis_size +=
							Options.Side == Dock.Left || Options.Side == Dock.Right ? measure.DesiredSize.Width :
							Options.Side == Dock.Top || Options.Side == Dock.Bottom ? measure.DesiredSize.Height :
							0;
					}
				}
				return m_axis_size.Value;
			}
			private set
			{
				m_axis_size = null;
				NotifyPropertyChanged(nameof(AxisSize));
			}
		}
		private double? m_axis_size;

		/// <summary>True if scrolling is allowed on this axis</summary>
		public bool AllowScroll
		{
			get => Axis?.AllowScroll ?? true;
			set
			{
				if (AllowScroll == value) return;
				if (Axis != null) Axis.AllowScroll = value;
				NotifyPropertyChanged(nameof(AllowScroll));
			}
		}

		/// <summary>True if zooming is allowed on this axis</summary>
		public bool AllowZoom
		{
			get => Axis?.AllowZoom ?? true;
			set
			{
				if (AllowZoom == value) return;
				if (Axis != null) Axis.AllowZoom = value;
				NotifyPropertyChanged(nameof(AllowZoom));
			}
		}

		/// <summary>Update the tick marks and labels</summary>
		public void Invalidate()
		{
			if (m_update_graphics_pending) return;
			m_update_graphics_pending = true;

			var weakthis = new WeakReference(this);
			Dispatcher.BeginInvoke(() =>
			{
				if (weakthis.Target is AxisPanel ap)
					ap.UpdateGraphics();
			});
		}
		private void UpdateGraphics()
		{
			if (Axis == null)
				return;

			// Preserve any graphics added by third parties
			const string Prefix = "PART_AxisPanel";
			var preserved = Children.Cast<FrameworkElement>().Where(x => !x.Name.StartsWith(Prefix)).ToList();

			// Reset the child controls
			Children.Clear();

			// Get the positions for the tick marks
			Axis.GridLines(out var min, out var max, out var step);

			// Add a line for the Chart axis
			var axis_line = Children.Add2(new Line { Name = $"{Prefix}_ChartAxis" });
			axis_line.Stroke = new SolidColorBrush(Options.AxisColour.ToMediaColor());
			axis_line.StrokeThickness = Options.AxisThickness;

			var bsh = new SolidColorBrush(Options.TickColour.ToMediaColor());
			switch (Options.Side)
			{
				case Dock.Left: // Left-side Y-Axis
				{
					// Position the axis line on the right side of this canvas
					axis_line.X1 = ActualWidth;
					axis_line.X2 = ActualWidth;
					axis_line.Y1 = 0;
					axis_line.Y2 = ActualHeight;

					// Add tick marks and labels
					for (var y = min; y < max; y += step)
					{
						var X = ActualWidth;
						var Y = ActualHeight - y * ActualHeight / Axis.Span;
						if (Options.DrawTickMarks)
						{
							Children.Add(new Line { Name = $"{Prefix}_TickMark", Stroke = bsh, X1 = X, Y1 = Y, X2 = X - Options.TickLength, Y2 = Y });
						}
						if (Options.DrawTickLabels)
						{
							var s = Axis.TickText(y + Axis.Min, step);
							var tb = new TextBlock
							{
								Name = $"{Prefix}_TickLabel",
								Text = s,
								Foreground = bsh,
								RenderTransform = Options.LabelTransform,
								RenderTransformOrigin = Options.LabelTransformOrigin,
							};
							var lbl = Children.Add2(tb);
							lbl.Typeface(Typeface, FontSize);
							lbl.Measure(Size_.Infinity);
							Canvas.SetLeft(lbl, X - lbl.DesiredSize.Width - Options.TickLength);
							Canvas.SetTop(lbl, Y - lbl.DesiredSize.Height / 2);
						}
					}
					break;
				}
				case Dock.Bottom: // Bottom-side X-Axis
				{
					// Position the axis line on the top side of this canvas
					axis_line.X1 = 0;
					axis_line.X2 = ActualWidth;
					axis_line.Y1 = 0;
					axis_line.Y2 = 0;

					// Add tick marks and labels
					for (var x = min; x < max; x += step)
					{
						var Y = 0;
						var X = x * ActualWidth / Axis.Span;
						if (Options.DrawTickMarks)
						{
							Children.Add(new Line { Name = $"{Prefix}_TickMark", Stroke = bsh, X1 = X, Y1 = Y, X2 = X, Y2 = Y + Options.TickLength });
						}
						if (Options.DrawTickLabels)
						{
							var s = Axis.TickText(x + Axis.Min, step);
							var tb = new TextBlock
							{
								Name = $"{Prefix}_TickLabel",
								Text = s,
								Foreground = bsh,
								RenderTransform = Options.LabelTransform,
								RenderTransformOrigin = Options.LabelTransformOrigin,
							};
							var lbl = Children.Add2(tb);
							lbl.Typeface(Typeface, FontSize);
							lbl.Measure(Size_.Infinity);
							Canvas.SetLeft(lbl, X - lbl.DesiredSize.Width / 2);
							Canvas.SetTop(lbl, Options.TickLength);
						}
					}
					break;
				}
				case Dock.Right: // Right-side Y-Axis
				{
					// Position the axis line on the left side of this canvas
					axis_line.X1 = 0;
					axis_line.X2 = 0;
					axis_line.Y1 = 0;
					axis_line.Y2 = ActualHeight;

					// Add tick marks and labels
					for (var y = min; y < max; y += step)
					{
						var X = 0;
						var Y = ActualHeight - y * ActualHeight / Axis.Span;
						if (Options.DrawTickMarks)
						{
							Children.Add(new Line { Name = $"{Prefix}_TickMark", Stroke = bsh, X1 = X, Y1 = Y, X2 = X + Options.TickLength, Y2 = Y });
						}
						if (Options.DrawTickLabels)
						{
							var s = Axis.TickText(y + Axis.Min, step);
							var tb = new TextBlock
							{
								Name = $"{Prefix}_TickLabel",
								Text = s,
								Foreground = bsh,
								RenderTransform = Options.LabelTransform,
								RenderTransformOrigin = Options.LabelTransformOrigin,
							};
							var lbl = Children.Add2(tb);
							lbl.Typeface(Typeface, FontSize);
							lbl.Measure(Size_.Infinity);
							Canvas.SetLeft(lbl, X + Options.TickLength + 1);
							Canvas.SetTop(lbl, Y - lbl.DesiredSize.Height / 2);
						}
					}
					break;
				}
				case Dock.Top: // Top-side X-Axis
				{
					// Position the axis line on the bottom side of this canvas
					axis_line.X1 = 0;
					axis_line.X2 = ActualWidth;
					axis_line.Y1 = ActualHeight;
					axis_line.Y2 = ActualHeight;

					// Add tick marks and labels
					for (var x = min; x < max; x += step)
					{
						var Y = ActualHeight;
						var X = x * ActualWidth / Axis.Span;
						if (Options.DrawTickMarks)
						{
							Children.Add(new Line { Name = $"{Prefix}_TickMark", Stroke = bsh, X1 = X, Y1 = Y, X2 = X, Y2 = Y - Options.TickLength });
						}
						if (Options.DrawTickLabels)
						{
							var s = Axis.TickText(x + Axis.Min, step);
							var tb = new TextBlock
							{
								Name = $"{Prefix}_TickLabel",
								Text = s,
								Foreground = bsh,
								RenderTransform = Options.LabelTransform,
								RenderTransformOrigin = Options.LabelTransformOrigin,
							};
							var lbl = Children.Add2(tb);
							lbl.Typeface(Typeface, FontSize);
							lbl.Measure(Size_.Infinity);
							Canvas.SetLeft(lbl, X - lbl.DesiredSize.Width / 2);
							Canvas.SetTop(lbl, Y - Options.TickLength - lbl.DesiredSize.Height);
						}
					}
					break;
				}
			}

			// Restore the preserved controls
			foreach (var c in preserved)
				Children.Add(c);

			m_update_graphics_pending = false;
		}
		private bool m_update_graphics_pending;

		/// <summary>Toggle the scroll locked state of the axis</summary>
		public ICommand ToggleScrollLock { get; }
		private void ToggleScrollLockInternal()
		{
			AllowScroll = !AllowScroll;
		}

		/// <summary>Toggle the scroll locked state of the axis</summary>
		public ICommand ToggleZoomLock { get; }
		private void ToggleZoomLockInternal()
		{
			AllowZoom = !AllowZoom;
		}

		///<inheritdoc/>
		public IChartAxisCMenu ChartAxisCMenuContext => this;

		/// <summary></summary>
		public event PropertyChangedEventHandler? PropertyChanged;
		public void NotifyPropertyChanged(string prop_name) => PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(prop_name));
	}
}
