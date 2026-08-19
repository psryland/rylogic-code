using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Windows;
using System.Windows.Controls;
using Rylogic.Gfx;
using Rylogic.Maths;

namespace Rylogic.Gui.WPF.ChartDetail
{
	public class ChartPanel : View3dControl, IChartCMenuContext
	{
		public ChartPanel()
		{
			if (DesignerProperties.GetIsInDesignMode(this))
				return;

			try
			{
				MouseNavigation = false;
				DefaultKeyboardShortcuts = false;
				Window.FocusPointVisible = false;
				Window.OriginPointVisible = false;
				Window.LightProperties = View3d.LightInfo.Directional(-v4.ZAxis, camera_relative: true);
			}
			catch
			{
				Dispose();
				throw;
			}
		}
		protected override void OnRenderTargetChanged()
		{
			base.OnRenderTargetChanged();
			if (Chart == null)
				return;

			// If the aspect is locked, resizing the render target zooms in/out.
			// Otherwise, change the camera position so that the range is preserved.
			if (Chart.LockAspect)
				Chart.SetRangeFromCamera();
			else
				Chart.SetCameraFromRange();

			Invalidate();
		}

		/// <inheritdoc/>
		public new IView3dCMenu View3dCMenuContext => Chart;

		/// <inheritdoc/>
		public IChartCMenu ChartCMenuContext => Chart;

		/// <summary>The containing chart control</summary>
		internal ChartControl Chart
		{
			get;
			set
			{
				if (field == value) return;
				field = value;
				DataContext = field;
			}
		} = null!;

		/// <summary>Add an object to the scene</summary>
		public void AddObject(View3d.Object obj)
		{
			Window.AddObject(obj);
		}
		public void AddObjects(IEnumerable<View3d.Object> objects)
		{
			Window.AddObjects(objects);
		}
		public void AddObjects(Guid context_id)
		{
			Window.AddObjects(context_id);
		}
		public void AddObjects(Func<Guid, bool> context_pred)
		{
			Window.AddObjects(context_pred);
		}

		/// <summary>Remove an object from the scene</summary>
		public void RemoveObject(View3d.Object obj)
		{
			Window.RemoveObject(obj);
		}
		public void RemoveObjects(IEnumerable<View3d.Object> objects)
		{
			Window.RemoveObjects(objects);
		}
		public void RemoveObjects(Func<Guid, bool> context_pred)
		{
			Window.RemoveObjects(context_pred);
		}
		public void RemoveAllObjects()
		{
			Window.RemoveAllObjects();
		}

		/// <summary>Prepare the scene for rendering</summary>
		protected override void OnBuildScene()
		{
			if (Chart == null)
				return;

			// Update axis graphics
			Chart.Range.UpdateScene(Window);

			// Raise the BuildScene event
			// Do this before calling UpdateScene on each element so that last minute
			// changes to the elements can be made. E.g. setting visibility, invalidating gfx, etc
			base.OnBuildScene();

			// Add/Remove all chart elements
			foreach (var elem in Chart.Elements)
				elem.UpdateScene();
		}
	}
}
