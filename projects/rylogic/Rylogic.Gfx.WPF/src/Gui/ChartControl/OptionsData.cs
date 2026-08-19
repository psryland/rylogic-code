using System.ComponentModel;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Xml.Linq;
using Rylogic.Common;
using Rylogic.Gfx;
using Rylogic.Maths;
using Rylogic.Utility;
using Matrix = System.Drawing.Drawing2D.Matrix;

namespace Rylogic.Gui.WPF
{
	public partial class ChartControl
	{
		[TypeConverter(typeof(TyConv))]
		public class OptionsData :SettingsSet<OptionsData>
		{
			public OptionsData()
			{
				NavigationMode = ENavMode.Chart2D;
				AllowSelection = false;
				AllowElementDragging = false;
				LockAspect = null;
				BackgroundColour = Colour32.White;
				SelectionColour = 0x8060A0E0;
				SceneBorderColour = Colour32.Transparent;
				SceneBorderThickness = 0.0;
				GridZOffset = 0.001;
				CrossHairZOffset = 0.005;
				FocusPointVisible = false;
				OriginPointVisible = false;
				ShowAxes = true;
				Antialiasing = true;
				ShadowCastRange = 0.0;
				FillMode = View3d.EFillMode.Solid;
				CullMode = View3d.ECullMode.Back;
				Orthographic = false;
				AreaSelectMode = EAreaSelectMode.Zoom;
				AreaSelectRequiresShiftKey = false;
				MinSelectionDistance = 10.0;
				MinDragPixelDistance = 5.0;
				MouseCentredZoom = true;
				ResetForward = -v4.ZAxis;
				ResetUp = +v4.YAxis;
				XAxis = new Axis { Side = Dock.Bottom };
				YAxis = new Axis { Side = Dock.Left };
			}
			public OptionsData(XElement node)
				: base(node)
			{}
			public OptionsData(OptionsData rhs)
				: base(rhs)
			{ }

			/// <summary>The control method used for shifting the camera</summary>
			public ENavMode NavigationMode
			{
				get => get<ENavMode>(nameof(NavigationMode));
				set => set(nameof(NavigationMode), value);
			}

			/// <summary>True if users are allowed to select elements on the diagram</summary>
			public bool AllowSelection
			{
				get => get<bool>(nameof(AllowSelection));
				set => set(nameof(AllowSelection), value);
			}

			/// <summary>True if users are allowed to select elements on the diagram</summary>
			public bool AllowElementDragging
			{
				get => get<bool>(nameof(AllowElementDragging));
				set => set(nameof(AllowElementDragging), value);
			}

			/// <summary>Lock the aspect ratio for the chart (null means unlocked)</summary>
			public double? LockAspect
			{
				get => get<double?>(nameof(LockAspect));
				set => set(nameof(LockAspect), value);
			}

			/// <summary>The chart area background colour</summary>
			public Colour32 BackgroundColour
			{
				get => get<Colour32>(nameof(BackgroundColour));
				set => set(nameof(BackgroundColour), value);
			}

			/// <summary>Area selection colour</summary>
			public Colour32 SelectionColour
			{
				get => get<Colour32>(nameof(SelectionColour));
				set => set(nameof(SelectionColour), value);
			}

			/// <summary>Chart area border colour</summary>
			public Colour32 SceneBorderColour
			{
				get => get<Colour32>(nameof(SceneBorderColour));
				set => set(nameof(SceneBorderColour), value);
			}

			/// <summary>Chart area border thickness</summary>
			public double SceneBorderThickness
			{
				get => get<double>(nameof(SceneBorderThickness));
				set => set(nameof(SceneBorderThickness), value);
			}

			/// <summary>The offset from the origin for the grid, in the forward direction of the camera (focus distance relative)</summary>
			public double GridZOffset
			{
				get => get<double>(nameof(GridZOffset));
				set => set(nameof(GridZOffset), value);
			}

			/// <summary>The depth position of the cross hair (focus distance relative)</summary>
			public double CrossHairZOffset
			{
				get => get<double>(nameof(CrossHairZOffset));
				set => set(nameof(CrossHairZOffset), value);
			}

			/// <summary>True if the focus is visible</summary>
			public bool FocusPointVisible
			{
				get => get<bool>(nameof(FocusPointVisible));
				set => set(nameof(FocusPointVisible), value);
			}

			/// <summary>True if the origin is visible</summary>
			public bool OriginPointVisible
			{
				get => get<bool>(nameof(OriginPointVisible));
				set => set(nameof(OriginPointVisible), value);
			}

			/// <summary>Show/Hide the chart axes</summary>
			public bool ShowAxes
			{
				get => get<bool>(nameof(ShowAxes));
				set => set(nameof(ShowAxes), value);
			}

			/// <summary>Show hide both X and Y Axis grid lines</summary>
			public bool ShowGridLines
			{
				get => XAxis.ShowGridLines || YAxis.ShowGridLines;
				set
				{
					if (value == ShowGridLines) return;
					XAxis.ShowGridLines = value;
					YAxis.ShowGridLines = value;
					NotifyPropertyChanged(nameof(ShowGridLines));
				}
			}

			/// <summary>Enable/Disable multi-sampling in the view3d view. Can only be changed before the view is created</summary>
			public bool Antialiasing
			{
				get => get<bool>(nameof(Antialiasing));
				set => set(nameof(Antialiasing), value);
			}

			/// <summary>How far to cast shadows. 0 = off</summary>
			public double ShadowCastRange
			{
				get => get<double>(nameof(ShadowCastRange));
				set => set(nameof(ShadowCastRange), value);
			}

			/// <summary>Fill mode, solid, wire, or both</summary>
			public View3d.EFillMode FillMode
			{
				get => get<View3d.EFillMode>(nameof(FillMode));
				set => set(nameof(FillMode), value);
			}

			/// <summary>Fill mode, solid, wire, or both</summary>
			public View3d.ECullMode CullMode
			{
				get => get<View3d.ECullMode>(nameof(CullMode));
				set => set(nameof(CullMode), value);
			}

			/// <summary>Get/Set orthographic camera projection</summary>
			public bool Orthographic
			{
				get => get<bool>(nameof(Orthographic));
				set => set(nameof(Orthographic), value);
			}

			/// <summary>The default behaviour of area select mode</summary>
			public EAreaSelectMode AreaSelectMode
			{
				get => get<EAreaSelectMode>(nameof(AreaSelectMode));
				set => set(nameof(AreaSelectMode), value);
			}

			/// <summary>Require the shift key to area select</summary>
			public bool AreaSelectRequiresShiftKey
			{
				get => get<bool>(nameof(AreaSelectRequiresShiftKey));
				set => set(nameof(AreaSelectRequiresShiftKey), value);
			}

			/// <summary>How close a click has to be for selection to occur (in client space)</summary>
			public double MinSelectionDistance
			{
				get => get<double>(nameof(MinSelectionDistance));
				set => set(nameof(MinSelectionDistance), value);
			}
			public double MinSelectionDistanceSq => MinSelectionDistance * MinSelectionDistance;

			/// <summary>Minimum distance in pixels before the chart starts dragging</summary>
			public double MinDragPixelDistance
			{
				get => get<double>(nameof(MinDragPixelDistance));
				set => set(nameof(MinDragPixelDistance), value);
			}

			/// <summary>True if the camera should move along a ray cast through the mouse point</summary>
			public bool MouseCentredZoom
			{
				get => get<bool>(nameof(MouseCentredZoom));
				set => set(nameof(MouseCentredZoom), value);
			}

			/// <summary>The forward direction of the camera when reset</summary>
			public v4 ResetForward
			{
				get => get<v4>(nameof(ResetForward));
				set => set(nameof(ResetForward), value);
			}

			/// <summary>The up direction of the camera when reset</summary>
			public v4 ResetUp
			{
				get => get<v4>(nameof(ResetUp));
				set => set(nameof(ResetUp), value);
			}

			/// <summary>XAxis rendering options</summary>
			public Axis XAxis
			{
				get => get<Axis>(nameof(XAxis));
				private set => set(nameof(XAxis), value);
			}

			/// <summary>YAxis rendering options</summary>
			public Axis YAxis
			{
				get => get<Axis>(nameof(YAxis));
				private set => set(nameof(YAxis), value);
			}

			[TypeConverter(typeof(TyConv))]
			public class Axis :SettingsSet<Axis>
			{
				public Axis()
				{
					Side = Dock.Left;
					AxisColour = Colour32.Black;
					LabelColour = Colour32.Black;
					TickColour = Colour32.Black;
					DrawTickMarks = true;
					DrawTickLabels = true;
					TickLength = 5;
					MinTickSize = 30;
					LabelTransform = Transform.Identity;
					LabelTransformOrigin = new Point();
					AxisThickness = 2f;
					PixelsPerTick = 30.0;
					ShowGridLines = true;
					TickTextTemplate = "-XXXXX.XX";
				}
				public Axis(XElement node)
					: base(node)
				{}
				public Axis(Axis rhs)
					:base(rhs)
				{ }

				/// <summary>The side of the chart that this axis is on</summary>
				public Dock Side
				{
					get => get<Dock>(nameof(Side));
					set => set(nameof(Side), value);
				}

				/// <summary>The colour of the main axes</summary>
				public Colour32 AxisColour
				{
					get => get<Colour32>(nameof(AxisColour));
					set => set(nameof(AxisColour), value);
				}

				/// <summary>The colour of the label text</summary>
				public Colour32 LabelColour
				{
					get => get<Colour32>(nameof(LabelColour));
					set => set(nameof(LabelColour), value);
				}

				/// <summary>The colour of the tick text</summary>
				public Colour32 TickColour
				{
					get => get<Colour32>(nameof(TickColour));
					set => set(nameof(TickColour), value);
				}

				/// <summary>True if tick marks should be drawn</summary>
				public bool DrawTickMarks
				{
					get => get<bool>(nameof(DrawTickMarks));
					set => set(nameof(DrawTickMarks), value);
				}

				/// <summary>True if tick labels should be drawn</summary>
				public bool DrawTickLabels
				{
					get => get<bool>(nameof(DrawTickLabels));
					set => set(nameof(DrawTickLabels), value);
				}

				/// <summary>The length of the tick marks</summary>
				public double TickLength
				{
					get => get<double>(nameof(TickLength));
					set => set(nameof(TickLength), value);
				}

				/// <summary>The minimum space reserved for tick marks and labels</summary>
				public double MinTickSize
				{
					get => get<double>(nameof(MinTickSize));
					set => set(nameof(MinTickSize), value);
				}

				/// <summary>Offset transform from default label position</summary>
				public Transform LabelTransform
				{
					get => get<Transform>(nameof(LabelTransform));
					set => set(nameof(LabelTransform), value);
				}

				/// <summary>Offset transform from default label position</summary>
				public Point LabelTransformOrigin
				{
					get => get<Point>(nameof(LabelTransformOrigin));
					set => set(nameof(LabelTransformOrigin), value);
				}
				
				/// <summary>The thickness of the axis line</summary>
				public double AxisThickness
				{
					get => get<double>(nameof(AxisThickness));
					set => set(nameof(AxisThickness), value);
				}

				/// <summary>The preferred number of pixels between each grid line</summary>
				public double PixelsPerTick
				{
					get => get<double>(nameof(PixelsPerTick));
					set => set(nameof(PixelsPerTick), value);
				}

				/// <summary>Show grid lines for this axis. (This settings is overruled by the main chart options)</summary>
				public bool ShowGridLines
				{
					get => get<bool>(nameof(ShowGridLines));
					set => set(nameof(ShowGridLines), value);
				}

				/// <summary>Example text used to measure the tick text size for this axis</summary>
				public string TickTextTemplate
				{
					get => get<string>(nameof(TickTextTemplate));
					set => set(nameof(TickTextTemplate), value);
				}

				private class TyConv :GenericTypeConverter<Axis> { }
			}

			private class TyConv :GenericTypeConverter<OptionsData> { }
		}
	}
}
