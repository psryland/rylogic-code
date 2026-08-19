using System;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Shapes;
using Rylogic.Gfx;
using Rylogic.Maths;
using Rylogic.Windows.Extn;

namespace Rylogic.Gui.WPF
{
	public partial class ChartControl
	{
		/// <summary>Helper object for drawing a horizontal and vertical line on the chart, with accompanying labels on the axes</summary>
		public sealed class CrossHair :IDisposable
		{
			public CrossHair(ChartControl chart)
			{
				Chart = chart;

				var line_colour = Chart.Scene.BackgroundColour.LerpRGB(Chart.Scene.BackgroundColour.InvertBW(), 0.5);
				var bkgd_colour = Chart.Scene.BackgroundColour.InvertBW(0xFF333333, 0xFFCCCCCC);
				var text_colour = Chart.Scene.BackgroundColour;

				LineV = new Line
				{
					Stroke = line_colour.ToMediaBrush(),
					StrokeThickness = 0.5,
					StrokeDashArray = ELineStyles.Dashed.ToStrokeDashArray(),
					StrokeStartLineCap = PenLineCap.Flat,
					StrokeEndLineCap = PenLineCap.Flat,
					IsHitTestVisible = false,
				};
				LineH = new Line
				{
					Stroke = line_colour.ToMediaBrush(),
					StrokeThickness = 0.5,
					StrokeDashArray = ELineStyles.Dashed.ToStrokeDashArray(),
					StrokeStartLineCap = PenLineCap.Flat,
					StrokeEndLineCap = PenLineCap.Flat,
					IsHitTestVisible = false,
				};
				LabelX = new TextBlock
				{
					Padding = new Thickness(4,2,4,2),
					TextAlignment = TextAlignment.Center,
					Background = bkgd_colour.ToMediaBrush(),
					Foreground = text_colour.ToMediaBrush(),
					IsHitTestVisible = false,
					MinWidth = 50,
				};
				LabelY = new TextBlock
				{
					Padding = new Thickness(4, 2, 4, 2),
					TextAlignment = TextAlignment.Center,
					Background = bkgd_colour.ToMediaBrush(),
					Foreground = text_colour.ToMediaBrush(),
					IsHitTestVisible = false,
					MinWidth = 50,
				};

				LabelX.Typeface(Chart.XAxisPanel.Typeface, Chart.YAxisPanel.FontSize);
				LabelY.Typeface(Chart.YAxisPanel.Typeface, Chart.YAxisPanel.FontSize);

				Chart.Overlay.Adopt(LineV);
				Chart.Overlay.Adopt(LineH);
				Chart.XAxisPanel.Adopt(LabelX);
				Chart.YAxisPanel.Adopt(LabelY);
			}
			public void Dispose()
			{
				LineV.Detach();
				LineH.Detach();
				LabelX.Detach();
				LabelY.Detach();
			}

			/// <summary>The owning chart</summary>
			private ChartControl Chart { get; }

			/// <summary>The vertical part of the cross hair</summary>
			public Line LineV { get; }

			/// <summary>The horizontal part of the cross hair</summary>
			public Line LineH { get; }

			/// <summary>A text label for the cross hair candle/time</summary>
			public TextBlock LabelX { get; }

			/// <summary>A text label for the cross hair price</summary>
			public TextBlock LabelY { get; }

			/// <summary></summary>
			private View3d.Camera Camera => Chart.Camera;

			/// <summary></summary>
			private OptionsData Options => Chart.Options;

			/// <summary>Set the chart position of the cross hair</summary>
			public void PositionCrossHair(v2 scene_point)
			{
				var chart_point = Chart.SceneToChart(scene_point);
				var bounds = Chart.SceneBounds;

				LineV.X1 = scene_point.x;
				LineV.X2 = scene_point.x;
				LineV.Y1 = 0;
				LineV.Y2 = bounds.Height;

				LineH.X1 = 0;
				LineH.X2 = bounds.Width;
				LineH.Y1 = scene_point.y;
				LineH.Y2 = scene_point.y;

				LabelX.Text = Chart.XAxis.TickText(chart_point.x);
				switch (Chart.XAxis.Options.Side)
				{
					case Dock.Top:
					{
						var axis_point = Gui_.MapPoint(Chart.Scene, Chart.XAxisPanel, scene_point.ToPointD());
						Canvas.SetLeft(LabelX, axis_point.X - LabelX.RenderSize.Width / 2);
						Canvas.SetTop(LabelX, Chart.XAxisPanel.Height - LabelX.RenderSize.Height - Chart.XAxis.Options.TickLength);
						break;
					}
					case Dock.Bottom:
					{
						var axis_point = Gui_.MapPoint(Chart.Scene, Chart.XAxisPanel, scene_point.ToPointD());
						Canvas.SetLeft(LabelX, axis_point.X - LabelX.RenderSize.Width / 2);
						Canvas.SetTop(LabelX, Chart.XAxis.Options.TickLength);
						break;
					}
					default:
					{
						throw new Exception("CrossHair label: Unexpected side for the X axis");
					}
				}

				LabelY.Text = Chart.YAxis.TickText(chart_point.y);
				switch (Chart.YAxis.Options.Side)
				{
					case Dock.Left:
					{
						var axis_point = Gui_.MapPoint(Chart.Scene, Chart.YAxisPanel, scene_point.ToPointD());
						Canvas.SetLeft(LabelY, Chart.YAxisPanel.Width - LabelY.RenderSize.Width - Chart.YAxis.Options.TickLength);
						Canvas.SetTop(LabelY, axis_point.Y - LabelY.RenderSize.Height / 2);
						break;
					}
					case Dock.Right:
					{
						var axis_point = Gui_.MapPoint(Chart.Scene, Chart.YAxisPanel, scene_point.ToPointD());
						Canvas.SetLeft(LabelY, Chart.YAxis.Options.TickLength);
						Canvas.SetTop(LabelY, axis_point.Y - LabelY.RenderSize.Height / 2);
						break;
					}
					default:
					{
						throw new Exception("CrossHair label: Unexpected side for the Y axis");
					}
				}
			}
		}
	}
}