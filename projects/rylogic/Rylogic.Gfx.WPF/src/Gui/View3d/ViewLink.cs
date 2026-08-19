using System;
using Rylogic.Common;
using Rylogic.Gfx;
using Rylogic.Maths;
using Rylogic.Utility;

namespace Rylogic.Gui.WPF
{
	[Flags]
	public enum ELinkCameras
	{
		None = 0,
		LeftRight = 1 << 0,
		UpDown = 1 << 1,
		InOut = 1 << 2,
		Rotate = 1 << 3,
		All = LeftRight | UpDown | InOut | Rotate,
	}

	public class ViewLink
	{
		// Notes:
		//  - Links the camera of two scenes without adding GC references
		//  - View3dControls do not use these directly, they are a helper for higher level code.

		public ViewLink(View3dControl source, View3dControl target, ELinkCameras cam = ELinkCameras.None)
		{
			Source = new WeakReference<View3dControl>(source);
			Target = new WeakReference<View3dControl>(target);
			CamLink = cam;

			// Watch for navigation events on 'source' and apply them to 'target'
			source.Window.MouseNavigating += WeakRef.MakeWeak<View3d.MouseNavigateEventArgs>(OnSourceNavigation, h => source.Window.MouseNavigating -= h);
		}

		/// <summary>The scene that is the source of navigation</summary>
		public WeakReference<View3dControl> Source { get; }

		/// <summary>The scene whose navigation mirrors that of 'Source'</summary>
		public WeakReference<View3dControl> Target { get; }

		/// <summary>Mask of mirrored camera movements</summary>
		public virtual ELinkCameras CamLink { get; set; }

		/// <summary>Handle a navigation event from 'Source'</summary>
		protected virtual void OnSourceNavigation(object? sender, View3d.MouseNavigateEventArgs e)
		{
			if (!Target.TryGetTarget(out var target))
				return;

			// Temporarily use the camera lock to limit motion
			using var mask = Scope.Create(() => target.Camera.LockMask, m => target.Camera.LockMask = m);
		
			// Set the lock mask to use based on flags
			var lock_mask = View3d.ECameraLockMask.All;
			if (CamLink.HasFlag(ELinkCameras.LeftRight) /*&& !AxisLink.HasFlag(ELinkAxes.XAxis)*/) lock_mask = Bit.SetBits(lock_mask, View3d.ECameraLockMask.TransX, false);
			if (CamLink.HasFlag(ELinkCameras.UpDown   ) /*&& !AxisLink.HasFlag(ELinkAxes.YAxis)*/) lock_mask = Bit.SetBits(lock_mask, View3d.ECameraLockMask.TransY, false);
			if (CamLink.HasFlag(ELinkCameras.InOut    )) lock_mask = Bit.SetBits(lock_mask, View3d.ECameraLockMask.TransZ | View3d.ECameraLockMask.Zoom, false);
			if (CamLink.HasFlag(ELinkCameras.Rotate   )) lock_mask = Bit.SetBits(lock_mask, View3d.ECameraLockMask.RotX | View3d.ECameraLockMask.RotY | View3d.ECameraLockMask.RotZ, false);
			target.Camera.LockMask = lock_mask;

			// Replicate the navigation command
			if (!e.ZNavigation)
				target.Window.MouseNavigate(e.Point, e.Btns, e.NavOp, e.NavBegOrEnd);
			else
				target.Window.MouseNavigateZ(e.Point, e.Btns, e.Delta, e.AlongRay);

			// Invalidate to refresh
			target.Invalidate();
		}
	}
}
