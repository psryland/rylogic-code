using System;
using System.Drawing;
using Rylogic.Gfx;
using Rylogic.Maths;
using Rylogic.Utility;

namespace Rylogic.Gui.WPF
{
	public partial class ChartControl
	{
		public sealed class GridLines :IDisposable
		{
			private readonly RangeData.Axis m_axis;
			public GridLines(RangeData.Axis axis)
			{
				m_axis = axis;

				// The grid lines model is being updated all the time, we don't want to reallocate
				// for each update. The step sizes used for the axis mean there is a maximum number 
				// of lines before a different step size is used, based on PixelsPerTick etc. Working
				// out what it is a hard though. Have a reasonable guess and handle it if we guessed
				// to small.
				MaxLines = 100;
			}
			public void Dispose()
			{
				Gfx = null;
			}

			/// <summary>The view 3D model</summary>
			private View3d.Object? Gfx
			{
				get;
				set
				{
					if (field == value) return;
					Util.Dispose(ref field);
					field = value;
				}
			}

			/// <summary>Debugging name for this graphics object</summary>
			public string Name =>
				m_axis.AxisType == EAxis.XAxis ? "xaxis_grid" :
				m_axis.AxisType == EAxis.YAxis ? "yaxis_grid" :
				string.Empty;

			/// <summary>Add graphics for this axis to the scene</summary>
			public void UpdateScene(View3d.Window window)
			{
				// If grid lines are not shown, destroy the view3d object
				if (!m_axis.Options.ShowGridLines)
				{
					Gfx = null;
					return;
				}

				// Get the number and spacing of the grid lines
				m_axis.GridLines(out var min, out var max, out var step);

				// Ensure the model is up to date
				if (Gfx == null || m_invalidated)
				{
					// Ensure the grid lines model is big enough
					var num_lines = (int)(2 + (max - min) / step);
					if (num_lines > MaxLines)
						MaxLines = num_lines + 10;

					// Ensure the model exists and is up to date
					if (Gfx == null)
						Gfx = CreateGfx();
					else
						Gfx.Edit(UpdateGfxCB);
				}

				// Position the model instance so that they line up with the axis tick marks.
				// Grid lines are modelled from the bottom left corner.
				var cam = window.Camera;
				var wh = cam.ViewArea(cam.FocusDist);
				var z = (float)(cam.FocusDist * m_axis.Chart.Options.GridZOffset);
				var pos =
					m_axis.AxisType == EAxis.XAxis ? new v4((float)(wh.x / 2 - min), wh.y / 2, z, 0) :
					m_axis.AxisType == EAxis.YAxis ? new v4(wh.x / 2, (float)(wh.y / 2 - min), z, 0) :
					throw new Exception("Unknown axis type");

				var o2w = cam.O2W;
				o2w.pos = cam.FocusPoint - o2w * pos;

				Gfx.O2WSet(o2w);
				window.AddObject(Gfx);
			}

			/// <summary>Signal that the graphics need updating</summary>
			public void Invalidate()
			{
				m_invalidated = true;
			}
			private bool m_invalidated;

			/// <summary>The maximum number of grid lines</summary>
			private int MaxLines
			{
				get;
				set
				{
					// If the max number of grid lines changes we'll need to reallocate the model buffer
					if (field == value) return;
					field = value;
					Gfx = null;
				}
			}

			/// <summary>Create a dynamic view3d model for the axis graphics</summary>
			private View3d.Object CreateGfx()
			{
				var vcount = MaxLines * 2;
				var icount = MaxLines * 2;
				var ncount = 1;
				var gfx = new View3d.Object(Name, 0xFFFFFFFF, vcount, icount, ncount, UpdateGfxCB, CtxId);
				gfx.FlagsSet(View3d.ELdrFlags.SceneBoundsExclude | View3d.ELdrFlags.NoZWrite | View3d.ELdrFlags.ShadowCastExclude, true);
				return gfx;
			}

			/// <summary>Update the grid lines model</summary>
			private View3d.VICount UpdateGfxCB(Span<View3d.Vertex> verts, Span<ushort> indices, View3d.Object.AddNuggetCB out_nugget)
			{
				// Create a model for the grid lines
				// Need to allow for one step in either direction because we only create the grid lines
				// model when scaling and we can translate by a max of one step in either direction.
				m_axis.GridLines(out var min, out var max, out var step);
				var num_lines = (int)(2 + (max - min) / step);
				if (num_lines > MaxLines)
					throw new Exception("Grid lines dynamic model is too small");

				// Create the grid lines at the origin, they get positioned as the camera moves
				var v = 0;
				var i = 0;

				// Choose a suitable grid colour
				var colour = m_axis.Chart.Scene.BackgroundColour;
				colour = colour.LerpRGB(colour.InvertBW(), 0.15);

				// Grid verts
				if (m_axis.AxisType == EAxis.XAxis)
				{
					var x = 0f; var y0 = 0f; var y1 = (float)m_axis.Chart.YAxis.Span;
					for (int l = 0; l != num_lines; ++l)
					{
						verts[v++] = new View3d.Vertex(new v4(x, y0, 0f, 1f), colour);
						verts[v++] = new View3d.Vertex(new v4(x, y1, 0f, 1f), colour);
						x += (float)step;
					}
				}
				if (m_axis.AxisType == EAxis.YAxis)
				{
					var y = 0f; var x0 = 0f; var x1 = (float)m_axis.Chart.XAxis.Span;
					for (int l = 0; l != num_lines; ++l)
					{
						verts[v++] = new View3d.Vertex(new v4(x0, y, 0f, 1f), colour);
						verts[v++] = new View3d.Vertex(new v4(x1, y, 0f, 1f), colour);
						y += (float)step;
					}
				}

				// Grid indices
				for (int l = 0; l != num_lines; ++l)
				{
					indices[i] = (ushort)i++;
					indices[i] = (ushort)i++;
				}

				// Grid nugget
				var nugget = new View3d.Nugget(View3d.ETopo.LineList, View3d.EGeom.Vert | View3d.EGeom.Colr, flags: View3d.ENuggetFlag.ShadowCastExclude);
				out_nugget(ref nugget);
				
				m_invalidated = false;
				return new() { m_vcount = num_lines * 2, m_icount = num_lines * 2 };
			}
		}
	}
}
