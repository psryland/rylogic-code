using System;
using Rylogic.Attrib;

namespace Rylogic.Gui.WPF
{
	public partial class ChartControl
	{
		/// <summary>Axis type</summary>
		[Flags]
		public enum EAxis
		{
			None = 0,
			XAxis = 1 << 0,
			YAxis = 1 << 1,
			Both = XAxis | YAxis,
		}

		/// <summary>Named parts of the chart</summary>
		[Flags]
		public enum EZone
		{
			None,
			Chart = 1 << 0,
			XAxis = 1 << 1,
			YAxis = 1 << 2,
			Title = 1 << 3,
		}

		// Prefer camera matrix operations for navigation, then call
		// SetRangeFromCamera to update the X/Y axis since these work
		// for 2D or 3D.

		/// <summary>Navigation methods for moving the camera</summary>
		public enum ENavMode
		{
			[Desc("2D Chart")]
			Chart2D,

			[Desc("3D Scene")]
			Scene3D,
		}

		/// <summary>Flags for how a chart was scrolled</summary>
		[Flags]
		public enum EMoveType
		{
			None = 0,
			XZoomed = 1 << 0,
			YZoomed = 1 << 1,
			XScrolled = 1 << 2,
			YScrolled = 1 << 3,
		}

		/// <summary>Chart change types</summary>
		public enum EChangeType
		{
			/// <summary>
			/// Raised after elements in the chart have been moved, resized, or had their content changed.
			/// This event will be raised in addition to the more detailed modification events below</summary>
			Edited,

			/// <summary>
			/// Elements are about to be deleted from the chart by the user.
			/// Setting 'Cancel' for this event will abort the deletion.</summary>
			RemovingElements,
		}

		/// <summary>Actions used during dragging</summary>
		public enum EDragState
		{
			Start,
			Dragging,
			Commit,
			Cancel,
		}
	}
}