using System;
using System.ComponentModel;
using System.Diagnostics;
using System.Linq;
using System.Text;
using Rylogic.Attrib;
using Rylogic.Container;
using Rylogic.Extn;
using Rylogic.Maths;
using Rylogic.Utility;

namespace Rylogic.Gfx
{
	public sealed class Measurement : IDisposable, INotifyPropertyChanged
	{
		// Notes:
		//  - This class provides the functionality for a measurement tool.
		//  - UI frameworks need to provide binding wrappers.
		//
		// General usage:
		//  - User clicks on 'Start' or 'End' toggle button.
		//  - MouseMove over the scene moves a hotspot around.
		//  - When a mouse click is detected, the Hit point for start/end is updated,
		//    and the toggle button returns to unchecked.
		//  - If the check button is unchecked, the Hit point is not set
		//  - User can still pan/rotate the scene with mouse drag operations

		public Measurement(View3d.Window window)
		{
			ReferenceFrame = EReferenceFrame.WorldSpace;
			SnapMode = View3d.ESnapMode.Verts | View3d.ESnapMode.Edges | View3d.ESnapMode.Faces;
			CtxId = Guid.NewGuid();
			View3d.ObjectManager.ExcludeCtxIds.Add(CtxId);
			SnapDistance = 5f; // pixels
			BegSpotColour = Colour32.Aqua;
			EndSpotColour = Colour32.Salmon;
			Results = new BindingDict<EQuantity, Result>(Enum<EQuantity>.Values.ToDictionary(x => x, x => new Result(x, "---")));

			Window = window;
		}
		public void Dispose()
		{
			GfxHotSpot0 = null;
			GfxHotSpot1 = null;
			GfxMeasure = null;
			View3d.ObjectManager.ExcludeCtxIds.Remove(CtxId);
			Window = null!;
		}

		/// <summary>The 3D scene to do the measuring in</summary>
		public View3d.Window Window
		{
			get => field;
			set
			{
				if (field == value) return;
				if (field != null!)
				{
					field.OnRendering -= HandleRendering;
					field.OnSceneChanged -= HandleSceneChanged;
				}
				field = value;
				if (field != null!)
				{
					field.OnSceneChanged += HandleSceneChanged;
					field.OnRendering += HandleRendering;
				}

				// Handlers
				void HandleSceneChanged(object? sender, View3d.SceneChangedEventArgs e)
				{
					if (e.ChangeType == View3d.ESceneChanged.ObjectsRemoved)
					{
						if (Hit0.Obj != null && !Window.HasObject(Hit0.Obj, true))
							Hit0.Obj = null;
						if (Hit1.Obj != null && !Window.HasObject(Hit1.Obj, true))
							Hit1.Obj = null;
					}
				}
				void HandleRendering(object? sender, EventArgs e)
				{
					// Add the graphics for the measurement
					if (GfxMeasure != null)
					{
						if (MeasurementValid)
							Window.AddObject(GfxMeasure);
						else
							Window.RemoveObject(GfxMeasure);
					}

					// Add the graphics for the hotspots
					if (GfxHotSpot0 != null)
					{
						if (Hit0.IsValid)
						{
							GfxHotSpot0.Colour = BegSpotColour;
							GfxHotSpot0.O2P = m4x4.Translation(Hit0.PointWS);
							Window.AddObject(GfxHotSpot0);
						}
						else
						{
							Window.RemoveObject(GfxHotSpot0);
						}
					}
					if (GfxHotSpot1 != null)
					{
						if (Hit1.IsValid)
						{
							GfxHotSpot1.Colour = EndSpotColour;
							GfxHotSpot1.O2P = m4x4.Translation(Hit1.PointWS);
							Window.AddObject(GfxHotSpot1);
						}
						else
						{
							Window.RemoveObject(GfxHotSpot1);
						}
					}
				}
			}
		}

		/// <summary>The context Id to use for the measurement graphics</summary>
		public Guid CtxId { get; }

		/// <summary>The context Id predicate to use</summary>
		public Func<Guid, bool> ContextPredicate { get; set; } = x => true;

		/// <summary>The colour of the starting hot spot</summary>
		public Colour32 BegSpotColour
		{
			get => field;
			set
			{
				if (BegSpotColour == value) return;
				field = value;
				GfxHotSpot0 = null;
				Window?.Invalidate();
				NotifyPropertyChanged(nameof(BegSpotColour));
			}
		}

		/// <summary>The colour of the ending hot spot</summary>
		public Colour32 EndSpotColour
		{
			get => field;
			set
			{
				if (EndSpotColour == value) return;
				field = value;
				GfxHotSpot1 = null;
				Window?.Invalidate();
				NotifyPropertyChanged(nameof(EndSpotColour));
			}
		}
		
		/// <summary>The snap distance (in pixels)</summary>
		public double SnapDistance
		{
			get => field;
			set
			{
				if (SnapDistance == value) return;
				field = value;
				Window?.Invalidate();
				NotifyPropertyChanged(nameof(SnapDistance));
			}
		}

		/// <summary>The snap-to flags</summary>
		public View3d.ESnapMode SnapMode
		{
			get => field;
			set
			{
				if (SnapMode == value) return;
				field = value;
				Window?.Invalidate();
				NotifyPropertyChanged(nameof(SnapMode));
			}
		}

		/// <summary>The reference frame to display the measurements in</summary>
		public EReferenceFrame ReferenceFrame
		{
			get => field;
			set
			{
				if (ReferenceFrame == value) return;
				field = value;
				InvalidateGfxMeasure();
				UpdateResults();
				Window?.Invalidate();
				NotifyPropertyChanged(nameof(ReferenceFrame));
			}
		}

		/// <summary>The starting point in reference space</summary>
		public v4 BegPoint
		{
			get => Math_.InvertAffine(RefSpaceToWorld) * Hit0.PointWS;
			set
			{
				Hit0.PointWS = RefSpaceToWorld * value;
				Hit0.SnapType = View3d.ESnapType.NoSnap;
				Hit0.Obj = null;
				NotifyPropertyChanged(nameof(Hit0));
			}
		}
		public v4 EndPoint
		{
			get => Math_.InvertAffine(RefSpaceToWorld) * Hit1.PointWS;
			set
			{
				Hit1.PointWS = RefSpaceToWorld * value;
				Hit1.SnapType = View3d.ESnapType.NoSnap;
				Hit1.Obj = null;
				NotifyPropertyChanged(nameof(Hit1));
			}
		}

		/// <summary>The start point to measure from</summary>
		public Hit Hit0
		{
			get => field;
			set
			{
				if (Hit0 == value) return;
				field = value;
				UpdateResults();
				InvalidateGfxMeasure();
			}
		} = new();

		/// <summary>The start point to measure from</summary>
		public Hit Hit1
		{
			get => field;
			set
			{
				if (field == value) return;
				field = value;
				UpdateResults();
				InvalidateGfxMeasure();
			}
		} = new();

		/// <summary>Get/Set the Hit point that is being set</summary>
		public Hit? ActiveHit
		{
			get => field;
			set
			{
				if (ActiveHit == value) return;
				Debug.Assert(value == null || value == Hit0 || value == Hit1, "ActiveHit must be either Hit0 or Hit1");
				field = value;
				NotifyPropertyChanged(nameof(ActiveHit));
			}
		}

		/// <summary>True when we can measure between 'Hit0' and 'Hit1'</summary>
		public bool MeasurementValid => Hit0.IsValid && Hit1.IsValid;

		/// <summary>A transform from reference frame space to world space</summary>
		public m4x4 RefSpaceToWorld
		{
			get
			{
				switch (ReferenceFrame)
				{
					case EReferenceFrame.WorldSpace:
					{
						return m4x4.Identity;
					}
					case EReferenceFrame.Object1Space:
					{
						if (!Hit0.IsValid || Hit0.Obj == null) return m4x4.Identity;
						return Math_.Orthonormalise(Hit0.Obj.O2P);
					}
					case EReferenceFrame.Object2Space:
					{
						if (!Hit1.IsValid || Hit1.Obj == null) return m4x4.Identity;
						return Math_.Orthonormalise(Hit1.Obj.O2P);
					}
					default:
					{
						throw new Exception($"Unknown reference frame: {ReferenceFrame}");
					}
				}
			}
		}

		/// <summary>Cast a ray into the scene to find the intercept</summary>
		public void UpdateActiveHitPosition(v2 point_cs)
		{
			if (ActiveHit == null)
				return;

			// Perform a hit test to update the position of the active hit
			var dist_nss = Window.Camera.PixelsToNSS(new v2((float)SnapDistance, (float)SnapDistance));
			var ray = Window.Camera.RaySS(point_cs, SnapMode | View3d.ESnapMode.Perspective, Math.Max(dist_nss.x, dist_nss.y));
			var result = Window.HitTest(ray, x => x != CtxId && ContextPredicate(x));

			// Update the current hit point
			ActiveHit.PointWS = result.m_ws_intercept;
			ActiveHit.SnapType = result.m_snap_type;
			ActiveHit.Obj = result.HitObject;

			if (ActiveHit == Hit0)
				NotifyPropertyChanged(nameof(Hit0));
			if (ActiveHit == Hit1)
				NotifyPropertyChanged(nameof(Hit1));

			// Invalidate
			UpdateResults();
			InvalidateGfxMeasure();
			Window?.Invalidate();
		}

		/// <summary>Mouse handler helpers. 'point_cs' is the client space mouse position</summary>
		public void MouseDown(v2 point_cs)
		{
			if (ActiveHit == null) return;
			m_mouse_down_at = point_cs;
		}
		public void MouseMove(v2 point_cs)
		{
			if (ActiveHit == null) return;
			UpdateActiveHitPosition(point_cs);
		}
		public void MouseUp(v2 point_cs, double click_threshold = 5.0)
		{
			// Determine if this was a click (not a drag) by comparing mouse-down and mouse-up positions
			var is_click = m_mouse_down_at != null && (point_cs - m_mouse_down_at.Value).Length <= click_threshold;
			m_mouse_down_at = null;
			if (!is_click) return;
			if (ActiveHit == null) return;

			// Lock the hit position and advance to the next point
			if (ActiveHit == Hit0)
				ActiveHit = Hit1;
			else
				ActiveHit = null;
		}
		private v2? m_mouse_down_at;

		/// <summary>Graphics for the hotspot that follows the mouse around</summary>
		private View3d.Object? GfxHotSpot0
		{
			get
			{
				if (field == null)
				{
					var ldr = new LDraw.Builder();
					ldr.Point("hotspot0", BegSpotColour).pt(new v3(0, 0, 0)).size(20).style(LDraw.EPointStyle.Circle).ztest(false).zwrite(false);
					field = new View3d.Object(ldr.ToBinary().GetBuffer(), CtxId) { Flags = View3d.ELdrFlags.HitTestExclude | View3d.ELdrFlags.SceneBoundsExclude | View3d.ELdrFlags.ShadowCastExclude };
				}
				return field;
			}
			set
			{
				if (field == value) return;
				Util.Dispose(ref field);
				field = value;
			}
		}

		/// <summary>Graphics for the hotspot that follows the mouse around</summary>
		private View3d.Object? GfxHotSpot1
		{
			get
			{
				if (field == null)
				{
					var ldr = new LDraw.Builder();
					ldr.Point("hotspot1", EndSpotColour).pt(new v3(0, 0, 0)).size(20).style(LDraw.EPointStyle.Circle).ztest(false).zwrite(false);
					field = new View3d.Object(ldr.ToBinary().GetBuffer(), CtxId) { Flags = View3d.ELdrFlags.HitTestExclude | View3d.ELdrFlags.SceneBoundsExclude | View3d.ELdrFlags.ShadowCastExclude };
				}
				return field;
			}
			set
			{
				if (field == value) return;
				Util.Dispose(ref field);
				field = value;
			}
		}

		/// <summary>Measurement graphics</summary>
		private View3d.Object? GfxMeasure
		{
			get
			{
				if (field == null && MeasurementValid)
				{
					var r2w = RefSpaceToWorld;
					var w2r = Math_.InvertAffine(r2w);
					var pt0 = w2r * Hit0.PointWS;
					var pt1 = w2r * Hit1.PointWS;
					var dist_x = Math.Abs(pt1.x - pt0.x);
					var dist_y = Math.Abs(pt1.y - pt0.y);
					var dist_z = Math.Abs(pt1.z - pt0.z);
					var dist = (pt1 - pt0).Length;

					var ldr = new LDraw.Builder();
					var grp = ldr.Group("Measurement");
					grp.font().colour(0xFFFFFFFF);
					if (dist_x != 0)
						grp.Line("dist_x", 0xFFFF0000).line(pt0.xyz, new v3(pt1.x, pt0.y, pt0.z))
							.Text("lbl_x").text($"{dist_x}").billboard().back_colour(0xFF800000).ztest(false).pos((pt0.x + pt1.x) / 2, pt0.y, pt0.z);
					if (dist_y != 0)
						grp.Line("dist_y", 0xFF00FF00).line(new v3(pt1.x, pt0.y, pt0.z), new v3(pt1.x, pt1.y, pt0.z))
							.Text("lbl_y").text($"{dist_y}").billboard().back_colour(0xFF006000).ztest(false).pos(pt1.x, (pt0.y + pt1.y) / 2, pt0.z);
					if (dist_z != 0)
						grp.Line("dist_z", 0xFF0000FF).line(new v3(pt1.x, pt1.y, pt0.z), pt1.xyz)
							.Text("lbl_z").text($"{dist_z}").billboard().back_colour(0xFF000080).ztest(false).pos(pt1.x, pt1.y, (pt0.z + pt1.z) / 2);
					if (dist != 0)
						grp.Line("dist", 0xFF000000).line(pt0.xyz, pt1.xyz)
							.Text("lbl_d").text($"{dist}").billboard().back_colour(0xFF000000).ztest(false).pos((pt0 + pt1) / 2);
					grp.o2w(r2w);

					field = new View3d.Object(ldr.ToBinary().GetBuffer(), CtxId) { Flags = View3d.ELdrFlags.HitTestExclude | View3d.ELdrFlags.SceneBoundsExclude | View3d.ELdrFlags.ShadowCastExclude };
				}
				return field;
			}
			set
			{
				if (field == value) return;
				Util.Dispose(ref field);
				field = value;
			}
		}

		/// <summary>Update the measurement graphics</summary>
		private void InvalidateGfxMeasure(object? sender = null, EventArgs? args = null)
		{
			GfxMeasure = null;
		}

		/// <summary>Property changed</summary>
		public event PropertyChangedEventHandler? PropertyChanged;
		public void NotifyPropertyChanged(string prop_name) => PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(prop_name));

		/// <summary>The measurement results</summary>
		public BindingDict<EQuantity, Result> Results { get; }

		/// <summary>Populate the results collection</summary>
		private void UpdateResults()
		{
			if (MeasurementValid)
			{
				// Convert the points into the selected space
				var w2rf = Math_.InvertAffine(RefSpaceToWorld);
				var pt0 = w2rf * Hit0.PointWS;
				var pt1 = w2rf * Hit1.PointWS;

				Results[EQuantity.Distance] = new Result(EQuantity.Distance, (pt1 - pt0).Length.ToString());
				Results[EQuantity.DistanceX] = new Result(EQuantity.DistanceX, Math.Abs(pt1.x - pt0.x).ToString());
				Results[EQuantity.DistanceY] = new Result(EQuantity.DistanceY, Math.Abs(pt1.y - pt0.y).ToString());
				Results[EQuantity.DistanceZ] = new Result(EQuantity.DistanceZ, Math.Abs(pt1.z - pt0.z).ToString());
				Results[EQuantity.AngleXY] = new Result(EQuantity.AngleXY, Math_.RadiansToDegrees(Math.Atan2(Math.Abs(pt1.y - pt0.y), Math.Abs(pt1.x - pt0.x))).ToString());
				Results[EQuantity.AngleXZ] = new Result(EQuantity.AngleXZ, Math_.RadiansToDegrees(Math.Atan2(Math.Abs(pt1.z - pt0.z), Math.Abs(pt1.x - pt0.x))).ToString());
				Results[EQuantity.AngleYZ] = new Result(EQuantity.AngleYZ, Math_.RadiansToDegrees(Math.Atan2(Math.Abs(pt1.z - pt0.z), Math.Abs(pt1.y - pt0.y))).ToString());
			}
			else
			{
				foreach (var q in Enum<EQuantity>.Values)
					Results[q] = new Result(q, "---");
			}
		}

		/// <summary>The end point of a measurement</summary>
		public class Hit
		{
			public Hit()
			{
				PointWS = v4.Origin;
				SnapType = View3d.ESnapType.NoSnap;
				Obj = null;
			}

			/// <summary>The point in world space of the start of the measurement</summary>
			public v4 PointWS;

			/// <summary>The object that the measurement point is on</summary>
			public View3d.Object? Obj;

			/// <summary>The type of point snap applied</summary>
			public View3d.ESnapType SnapType;

			/// <summary>True if this hit point is on a known object</summary>
			public bool IsValid => Obj != null;
		}

		/// <summary>The single result of a measurement</summary>
		public class Result
		{
			public Result(EQuantity quantity, string value)
			{
				Quantity = quantity;
				Value = value;
			}

			/// <summary>The name of the measured quantity</summary>
			public EQuantity Quantity { get; }

			/// <summary>String name for the quantity</summary>
			public string QuantityName => Quantity.Desc() ?? string.Empty;

			/// <summary>The result of the measurement</summary>
			public string Value { get; set; }
		}

		/// <summary>Types of measured quantities</summary>
		public enum EQuantity
		{
			[Desc("Distance")] Distance,
			[Desc("X Distance")] DistanceX,
			[Desc("Y Distance")] DistanceY,
			[Desc("Z Distance")] DistanceZ,
			[Desc("ATan(Y/X)")] AngleXY,
			[Desc("ATan(Z/X)")] AngleXZ,
			[Desc("ATan(Z/Y)")] AngleYZ,
		}

		/// <summary>Spaces that measurements can be made in</summary>
		public enum EReferenceFrame
		{
			[Desc("World Space")] WorldSpace,
			[Desc("Start Object Space")] Object1Space,
			[Desc("End Object Space")] Object2Space,
		}
	}
}
