using System;
using Rylogic.Common;
using Rylogic.Gfx;
using Rylogic.Maths;
using Rylogic.Utility;

namespace Rylogic.Gui.WPF
{
	[Flags]
	public enum ELinkAxes
	{
		None = 0,
		XAxis = 1 << 0,
		YAxis = 1 << 1,
		All = XAxis | YAxis,
	}

	public class ChartLink :ViewLink
	{
		// Notes:
		//  - Links the camera or axis range of two charts without adding a GC references.
		//  - ChartControls do not use these directly, they are a helper for higher level code.

		public ChartLink(ChartControl source, ChartControl target, ELinkCameras cam = ELinkCameras.None, ELinkAxes axis = ELinkAxes.None)
			:base(source.Scene, target.Scene, cam)
		{
			Source = new WeakReference<ChartControl>(source);
			Target = new WeakReference<ChartControl>(target);
			AxisLink = axis;

			// Watch for axis range changes
			source.ChartMoved += WeakRef.MakeWeak<ChartControl.ChartMovedEventArgs>(OnSourceChartMoved, h => source.ChartMoved -= h);
		}

		/// <summary>The chart that is the source of navigation</summary>
		public new WeakReference<ChartControl> Source { get; }

		/// <summary>The chart whose navigation mirrors that of 'Source'</summary>
		public new WeakReference<ChartControl> Target { get; }

		/// <inheritdoc/>
		public override ELinkCameras CamLink
		{
			get
			{
				var cam = base.CamLink;
				if (AxisLink.HasFlag(ELinkAxes.XAxis)) cam &= ~ELinkCameras.LeftRight;
				if (AxisLink.HasFlag(ELinkAxes.YAxis)) cam &= ~ELinkCameras.UpDown;
				return cam;
			}
			set =>base.CamLink = value;
		}

		/// <summary>Mask of mirrored axis range</summary>
		public virtual ELinkAxes AxisLink { get; set; }

		/// <summary>Handle the axis range of the source chart changing</summary>
		protected virtual void OnSourceChartMoved(object? sender, ChartControl.ChartMovedEventArgs e)
		{
			if (!(sender is ChartControl source)) return;
			if (!Target.TryGetTarget(out var target))
				return;

			var update = false;
			if (AxisLink.HasFlag(ELinkAxes.XAxis))
			{
				target.XAxis.Range = source.XAxis.Range;
				update = true;
			}
			if (AxisLink.HasFlag(ELinkAxes.YAxis))
			{
				target.YAxis.Range = source.YAxis.Range;
				update = true;
			}
			if (update)
			{
				target.SetCameraFromRange();
				target.Invalidate();
			}
		}

		/// <inheritdoc />
		protected override void OnSourceNavigation(object? sender, View3d.MouseNavigateEventArgs e)
		{
			base.OnSourceNavigation(sender, e);
			if (!Target.TryGetTarget(out var target))
				return;

			target.SetRangeFromCamera();
			target.Invalidate();
		}
	}
}
