using System;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Shapes;
using Rylogic.Maths;
using Rylogic.Windows.Extn;

namespace Rylogic.Gui.WPF
{
	public partial class ChartControl
	{
		public sealed class TapeMeasure :IDisposable
		{
			private readonly bool m_xhair_enabled;
			public TapeMeasure(ChartControl chart)
			{
				Chart = chart;
				Chart.MouseOperations[MouseButton.Left] = new DrawTape(this);

				m_xhair_enabled = Chart.ShowCrossHair;
				Chart.ShowCrossHair = true;

				// A slightly darker or lighter shade of the background colour
				var stroke = Chart.Options.BackgroundColour.LerpRGB(Chart.Options.BackgroundColour.InvertBW(), 0.3);
				var bkgd = Chart.Options.BackgroundColour.LerpRGB(Chart.Options.BackgroundColour.InvertBW(), 0.1);
				var fill = bkgd.Alpha(0.5);
				Area = new Rectangle
				{
					Stroke = stroke.ToMediaBrush(),
					Fill = fill.ToMediaBrush(),
					StrokeThickness = 0.8,
					IsHitTestVisible = false,
				};
				DeltaX = new Line
				{
					Stroke = stroke.ToMediaBrush(),
					StrokeDashArray = ELineStyles.Dashed.ToStrokeDashArray(),
					StrokeEndLineCap = PenLineCap.Triangle,
					StrokeThickness = 0.8,
					IsHitTestVisible = false,
				};
				DeltaY = new Line
				{
					Stroke = stroke.ToMediaBrush(),
					StrokeDashArray = ELineStyles.Dashed.ToStrokeDashArray(),
					StrokeEndLineCap = PenLineCap.Triangle,
					StrokeThickness = 0.8,
					IsHitTestVisible = false,
				};
				DeltaD = new Line
				{
					Stroke = stroke.ToMediaBrush(),
					StrokeDashArray = ELineStyles.Dashed.ToStrokeDashArray(),
					StrokeEndLineCap = PenLineCap.Triangle,
					StrokeThickness = 1.0,
					IsHitTestVisible = false,
				};
				LabelX = new TextBlock
				{
					FontSize = 10,
					Foreground = fill.InvertBW().ToMediaBrush(),
					Background = bkgd.ToMediaBrush(),
					IsHitTestVisible = false,
					Margin = new Thickness(3),
				};
				LabelY = new TextBlock
				{
					FontSize = 10,
					Foreground = fill.InvertBW().ToMediaBrush(),
					Background = bkgd.ToMediaBrush(),
					IsHitTestVisible = false,
					Margin = new Thickness(3),
				};
				LabelD = new TextBlock
				{
					FontSize = 10,
					Foreground = fill.InvertBW().ToMediaBrush(),
					Background = bkgd.ToMediaBrush(),
					IsHitTestVisible = false,
					Margin = new Thickness(3),
				};
			}
			public void Dispose()
			{
				if (Chart != null)
					Chart.ShowCrossHair = m_xhair_enabled;

				Detach();
				Chart = null!;
			}

			/// <summary>The owning chart</summary>
			private ChartControl Chart
			{
				get;
				set
				{
					if (field == value) return;
					if (field != null)
					{
						field.ChartMoved -= HandleMoved;
					}
					field = value;
					if (field != null)
					{
						field.ChartMoved += HandleMoved;
					}

					// Handlers
					void HandleMoved(object? sender, ChartMovedEventArgs e)
					{
						if (Beg != null && End != null)
							UpdateGfx();
					}
				}
			} = null!;

			/// <summary>The grab point of the tape measure</summary>
			private v4? Beg
			{
				get;
				set
				{
					if (field == value) return;
					field = value;
					if (field == null)
					{
						End = null;
						Detach();
					}
					else
					{
						if (End == null)
							End = value;

						Adopt();
						UpdateGfx();
					}
				}
			}

			/// <summary>The drop point of the tape measure</summary>
			private v4? End
			{
				get;
				set
				{
					if (field == value) return;
					field = value;
					if (field == null)
					{
						Beg = null;
						Detach();
					}
					else
					{
						if (Beg == null)
							Beg = value;

						Adopt();
						UpdateGfx();
					}
				}
			}

			/// <summary>The measurement area</summary>
			private Rectangle Area { get; }

			/// <summary>The delta X graphics</summary>
			private Line DeltaX { get; }

			/// <summary>The delta Y graphics</summary>
			private Line DeltaY { get; }

			/// <summary>The delta diagonal graphics</summary>
			private Line DeltaD { get; }

			/// <summary>Text description of the measured X value</summary>
			private TextBlock LabelX { get; }

			/// <summary>Text description of the measured X value</summary>
			private TextBlock LabelY { get; }

			/// <summary>Text description of the measured diagonal value</summary>
			private TextBlock LabelD { get; }

			/// <summary>Add the tape measure graphics</summary>
			private void Adopt()
			{
				Chart.Overlay.Adopt(Area);
				Chart.Overlay.Adopt(DeltaX);
				Chart.Overlay.Adopt(DeltaY);
				Chart.Overlay.Adopt(DeltaD);
				Chart.Overlay.Adopt(LabelX);
				Chart.Overlay.Adopt(LabelY);
				Chart.Overlay.Adopt(LabelD);
			}

			/// <summary>Remove the tape measure graphics</summary>
			private void Detach()
			{
				Area.Detach();
				DeltaX.Detach();
				DeltaY.Detach();
				DeltaD.Detach();
				LabelX.Detach();
				LabelY.Detach();
				LabelD.Detach();
			}

			/// <summary></summary>
			private void UpdateGfx()
			{
				if (Beg == null || End == null)
					throw new Exception("UpdateGfx should only be called when both ends are valid");

				var beg = Chart.ChartToScene(Beg.Value);
				var end = Chart.ChartToScene(End.Value);

				Area.Width = Math.Abs(end.x - beg.x);
				Area.Height = Math.Abs(end.y - beg.y);
				Canvas.SetLeft(Area, Math.Min(beg.x, end.x));
				Canvas.SetTop(Area, Math.Min(beg.y, end.y));

				DeltaX.X1 = beg.x;
				DeltaX.Y1 = (beg.y + end.y) / 2;
				DeltaX.X2 = end.x;
				DeltaX.Y2 = (beg.y + end.y) / 2;

				DeltaY.X1 = (beg.x + end.x) / 2; 
				DeltaY.Y1 = beg.y;
				DeltaY.X2 = (beg.x + end.x) / 2;
				DeltaY.Y2 = end.y;

				DeltaD.X1 = beg.x;
				DeltaD.Y1 = beg.y;
				DeltaD.X2 = end.x;
				DeltaD.Y2 = end.y;

				var text = Chart.TapeMeasureStringFormat(Beg.Value, End.Value);

				LabelX.Text = text.LabelX ?? string.Empty;
				LabelY.Text = text.LabelY ?? string.Empty;
				LabelD.Text = text.LabelD ?? string.Empty;
				LabelX.Measure(Size_.Infinity);
				LabelY.Measure(Size_.Infinity);
				LabelD.Measure(Size_.Infinity);

				Canvas.SetLeft(LabelX, end.x > beg.x ? end.x : end.x - LabelX.DesiredSize.Width);
				Canvas.SetTop(LabelX, (beg.y + end.y - LabelX.DesiredSize.Height) / 2);

				Canvas.SetLeft(LabelY, (beg.x + end.x - LabelY.DesiredSize.Width) / 2);
				Canvas.SetTop(LabelY, end.y > beg.y ? end.y : end.y - LabelY.DesiredSize.Height);

				Canvas.SetLeft(LabelD, end.x > beg.x ? end.x + 20 : end.x - LabelD.DesiredSize.Width);
				Canvas.SetTop(LabelD, end.y > beg.y ? end.y + 20 : end.y - LabelY.DesiredSize.Height);
			}

			/// <summary></summary>
			private class DrawTape :MouseOp
			{
				private readonly TapeMeasure m_owner;
				public DrawTape(TapeMeasure owner)
					:base(owner.Chart, allow_cancel: true)
				{
					m_owner = owner;
				}
				public override void MouseDown(MouseButtonEventArgs? e)
				{
					if (e == null) throw new Exception("This mouse op should start on mouse down");
					m_owner.Beg = GrabCS;
					base.MouseDown(e);
				}
				public override void MouseMove(MouseEventArgs e)
				{
					base.MouseMove(e);
					m_owner.End = DropCS;
				}
				public override void MouseWheel(MouseWheelEventArgs e)
				{
					base.MouseWheel(e);
					e.Handled = false;
				}
			}

			/// <summary></summary>
			public class LabelText
			{
				// Notes:
				//  - Leave properties as null to hide the associated label.

				/// <summary>Text to display for the horizontal measurement</summary>
				public string? LabelX { get; set; }

				/// <summary>Text to display for the vertical measurement</summary>
				public string? LabelY { get; set; }

				/// <summary>Text to display for the diagonal measurement</summary>
				public string? LabelD { get; set; }
			}
		}
	}
}
