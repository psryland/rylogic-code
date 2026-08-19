using System.Collections.Generic;
using System.Linq;
using System.Windows.Input;
using Rylogic.Common;
using Rylogic.Gfx;
using Rylogic.Maths;

namespace Rylogic.Gui.WPF
{
	public partial class ChartControl
	{
		/// <summary>Results collection for a hit test</summary>
		public class HitTestResult
		{
			public HitTestResult(EZone zone, v2 scene_point, v4 chart_point, ModifierKeys modifier_keys, EMouseBtns mouse_btns, IEnumerable<Hit> hits, View3d.Camera cam)
			{
				Zone = zone;
				ScenePoint = scene_point;
				ChartPoint = chart_point;
				Hits = hits.ToList();
				ModifierKeys = modifier_keys;
				MouseBtns = mouse_btns;
				Camera = cam;
			}

			/// <summary>The zone on the chart that was hit</summary>
			public EZone Zone { get; }

			/// <summary>The client scene space location of where the hit test was performed</summary>
			public v2 ScenePoint { get; }

			/// <summary>The chart space location of where the hit test was performed</summary>
			public v4 ChartPoint { get; }

			/// <summary>The collection of hit objects</summary>
			public List<Hit> Hits { get; }

			/// <summary>Keys held down during the hit test</summary>
			public ModifierKeys ModifierKeys { get; }

			/// <summary>The mouse button state during the hit test</summary>
			public EMouseBtns MouseBtns { get; }

			/// <summary>The camera position when the hit test was performed (needed for chart to screen space conversion)</summary>
			public View3d.Camera Camera { get; }

			/// <summary></summary>
			public class Hit
			{
				/// <summary></summary>
				/// <param name="elem">The element that was hit</param>
				/// <param name="elem_point">Where the element was hit (in element space)</param>
				/// <param name="context">Context to provide with the hit result</param>
				public Hit(Element elem, v4 elem_point, object? context)
				{
					Element = elem;
					Point = elem_point;
					Location = elem.O2W;
					Context = context;
				}

				/// <summary>The element that was hit</summary>
				public Element Element { get; }

				/// <summary>Where on the element it was hit (in element space, i.e. typically world space, but element relative)</summary>
				public v4 Point { get; }

				/// <summary>The element's chart location at the time it was hit</summary>
				public m4x4 Location { get; }

				/// <summary>Optional context information collected during the hit test</summary>
				public object? Context { get; }

				/// <summary></summary>
				public override string ToString() => Element.ToString() ?? string.Empty;
			}
		}
	}
}
