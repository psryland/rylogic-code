using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;
using Rylogic.Extn;
using Rylogic.Windows.Extn;

namespace Rylogic.Gui.WPF.DockContainerDetail
{
	/// <summary>Handles all docking operations</summary>
	internal class DragHandler : Window
	{
		// Notes:
		// This works by displaying an invisible modal window while the drag operation is in process.
		// The modal window ensures the rest of the UI is disabled for the duration of the drag.
		// The modal window spawns other non-interactive windows that provide the overlays for the drop targets.
		private static readonly Vector DraggeeOfs = new(-50, -30);
		private static readonly Rect DraggeeRect = new(0, 0, 150, 150);
		private readonly TabButton m_ghost_button;
		private Point m_ss_start_pt;

		public DragHandler(DockControl draggee, Point ss_start_pt)
			: this((object)draggee, ss_start_pt)
		{ }
		public DragHandler(DockPane draggee, Point ss_start_pt)
			: this((object)draggee, ss_start_pt)
		{ }
		private DragHandler(object draggee, Point ss_start_pt)
		{
			if (draggee == null)
				throw new Exception("Only panes and content should be being dragged");

			WindowStartupLocation = WindowStartupLocation.Manual;
			Visibility = Visibility.Collapsed;
			Focusable = false;
			ShowInTaskbar = false;

			var dc = draggee as DockControl;
			var dp = draggee as DockPane;
			var tree_host = dc?.TreeHost ?? dp?.TreeHost ?? throw new Exception("Dragged object must have an initial tree host");
			var container = dc?.DockContainer ?? dp?.DockContainer ?? throw new Exception("Dragged object must belong to a DockContainer");
			var item_name = dc?.TabText ?? dp?.CaptionText;
			var item_icon = dc?.TabIcon ?? dp?.VisibleContent?.TabIcon;

			m_ss_start_pt = ss_start_pt;
			m_ghost_button = new TabButton(item_name, item_icon) { Opacity = 0.5 };
			DockContainer = container;
			DraggedFromTree = tree_host;
			DraggedItem = draggee;
			TreeHost = null;
			DropAddress = Array.Empty<EDockSite>();
			DropIndex = null;

			// Hide all auto hide panels, since they are not valid drop targets
			foreach (var panel in DockContainer.AutoHidePanels)
				panel.PoppedOut = false;

			// If dragging a single item, hide the tab from it's pane
			if (dc?.TabButton is TabButton tb)
				tb.Visibility = Visibility.Collapsed;

			// Create an invisible modal dialog
			Title = "Drop Handler";
			ShowInTaskbar = false;
			WindowStyle = WindowStyle.None;
			ResizeMode = ResizeMode.NoResize;
			Left = 0;
			Top = 0;
			Width = 0;
			Height = 0;
		}
		protected override void OnSourceInitialized(EventArgs e)
		{
			base.OnSourceInitialized(e);

			// Convert (x,y) to device units in screen space
			Point ScreenPoint(double x, double y) => DockContainer.PointToScreen(new Point(x, y)).TransformFromDevice(DockContainer);

			// Set the positions of the edge docking indicators
			var distance_from_edge = 15;
			var loc_ghost = Point.Add(m_ss_start_pt, DraggeeOfs);
			var loc_top = ScreenPoint((DockContainer.ActualWidth - DimensionsFor(EIndicator.dock_site_top).Width) / 2, distance_from_edge);
			var loc_left = ScreenPoint(distance_from_edge, (DockContainer.ActualHeight - DimensionsFor(EIndicator.dock_site_left).Height) / 2);
			var loc_right = ScreenPoint(DockContainer.ActualWidth - distance_from_edge - DimensionsFor(EIndicator.dock_site_right).Width, (DockContainer.ActualHeight - DimensionsFor(EIndicator.dock_site_right).Height) / 2);
			var loc_bottom = ScreenPoint((DockContainer.ActualWidth - DimensionsFor(EIndicator.dock_site_bottom).Width) / 2, DockContainer.ActualHeight - distance_from_edge - DimensionsFor(EIndicator.dock_site_bottom).Height);

			// Create the semi-transparent non-modal window for the dragged item
			Ghost = new GhostPane(this, loc_ghost) { Visibility = Visibility.Visible };
			Ghost.Show();

			// Create the dock site indicators
			IndTop = new Indicator(this, EIndicator.dock_site_top, DockContainer.AutoHidePanels[EDockSite.Top].Root.DockPane(EDockSite.Centre), loc_top) { Visibility = Visibility.Visible };
			IndLeft = new Indicator(this, EIndicator.dock_site_left, DockContainer.AutoHidePanels[EDockSite.Left].Root.DockPane(EDockSite.Centre), loc_left) { Visibility = Visibility.Visible };
			IndRight = new Indicator(this, EIndicator.dock_site_right, DockContainer.AutoHidePanels[EDockSite.Right].Root.DockPane(EDockSite.Centre), loc_right) { Visibility = Visibility.Visible };
			IndBottom = new Indicator(this, EIndicator.dock_site_bottom, DockContainer.AutoHidePanels[EDockSite.Bottom].Root.DockPane(EDockSite.Centre), loc_bottom) { Visibility = Visibility.Visible };
			IndCrossLg = new Indicator(this, EIndicator.dock_site_cross_lg) { Visibility = Visibility.Collapsed };
			IndCrossSm = new Indicator(this, EIndicator.dock_site_cross_sm) { Visibility = Visibility.Collapsed };

			// The dock handler deals with the mouse
			CaptureMouse();
		}
		protected override void OnMouseMove(MouseEventArgs e)
		{
			base.OnMouseMove(e);
			HitTestDropLocations(PointToScreen(e.GetPosition(this)));
		}
		protected override void OnMouseUp(MouseButtonEventArgs e)
		{
			base.OnMouseUp(e);
			HitTestDropLocations(PointToScreen(e.GetPosition(this)));
			if (DialogResult == null)
				DialogResult = true;
		}
		protected override void OnLostMouseCapture(MouseEventArgs e)
		{
			base.OnLostMouseCapture(e);
			if (DialogResult == null)
				DialogResult = false;
		}
		protected override void OnKeyDown(KeyEventArgs e)
		{
			base.OnKeyDown(e);
			if (e.Key == Key.Escape)
			{
				DialogResult = false;
				Close();
			}
		}
		protected override void OnClosed(EventArgs e)
		{
			ReleaseMouseCapture();
			DockContainer.Focus();

			var dc = DraggedItem as DockControl;
			var dp = DraggedItem as DockPane;

			// Tidy up tab buttons
			if (dc?.TabButton is TabButton tb)
				tb.Visibility = Visibility.Visible;
			if (m_ghost_button.TabStrip != null)
				m_ghost_button.TabStrip.Buttons.Remove(m_ghost_button);

			// Commit the move on successful close
			if (DialogResult == true)
			{
				// Preserve the active content
				var active = DockContainer.ActiveContent;

				// No address means float in a new floating window
				if (DropAddress.Length == 0)
				{
					// Float the dragged content
					if (dc != null)
					{
						dc.IsFloating = true;
					}

					// Or, float the dragged dock pane
					if (dp != null)
					{
						dc = dp.VisibleContent;
						dp.IsFloating = true;
					}

					// Set the location of the floating window to the last position of the ghost
					if (dc?.TreeHost is FloatingWindow fw)
					{
						fw.Left = Ghost.Left;
						fw.Top = Ghost.Top;
					}
				}

				// Otherwise dock the dragged item at the dock address
				else if (TreeHost != null)
				{
					// Ensure a pane exists at the drop address
					var target = TreeHost.Root.DockPane(DropAddress);
					var index = DropIndex ?? target.TabStrip.Buttons.Count;
					var content =
						dc != null ? new[] { dc } :
						dp != null ? dp.AllContent.ToArray() :
						throw new Exception();

					// Dock the dragged item(s)
					foreach (var c in content)
					{
						// If we're dropping the item within the same pane, the index needs to be
						// adjusted if the original index is less than the new index.
						if (c.DockPane == target && c.TabButton != null && c.DockPane.TabStrip.Buttons.IndexOf(c.TabButton) < index)
							--index;

						// Even if 'dc.DockPane == target' remove and re-add 'c' because we might be
						// changing the order
						c.DockPane = null;
						target.AllContent.Insert(index++, c);
					}

					// Refit floating windows whose content changed
					(TreeHost as FloatingWindow)?.FitToContent();
					if (DraggedFromTree != TreeHost)
						(DraggedFromTree as FloatingWindow)?.FitToContent();
				}

				// Restore the active content
				DockContainer.ActiveContent = active;
				Debug.Assert(DockContainer.ValidateTree());
			}

			Ghost = null!;
			IndCrossLg = null!;
			IndCrossSm = null!;
			IndLeft = null!;
			IndTop = null!;
			IndRight = null!;
			IndBottom = null!;
			base.OnClosed(e);
		}

		/// <summary>The dock container that created this drop handler</summary>
		private DockContainer DockContainer { get; }

		/// <summary>The tree host that the item is being dragged from</summary>
		private ITreeHost DraggedFromTree { get; }

		/// <summary>The item being dragged (either a DockPane or IDockable)</summary>
		private object DraggedItem { get; }

		/// <summary>The tree to drop into</summary>
		private ITreeHost? TreeHost { get; set; }

		/// <summary>Where to dock the dropped item. If empty drop in a new floating window</summary>
		private EDockSite[] DropAddress { get; set; }

		/// <summary>The index position of where to drop within a pane</summary>
		private int? DropIndex { get; set; }

		/// <summary>A form used as graphics to show dragged items</summary>
		private GhostPane Ghost
		{
			get;
			set
			{
				if (field == value) return;
				field?.Close();
				field = value;
			}
		} = null!;

		/// <summary>The cross of dock site locations displayed within the centre of a pane</summary>
		private Indicator IndCrossLg
		{
			get;
			set
			{
				if (field == value) return;
				field?.Close();
				field = value;
			}
		} = null!;

		/// <summary>The small cross of dock site locations displayed within the centre of a pane</summary>
		private Indicator IndCrossSm
		{
			get;
			set
			{
				if (field == value) return;
				field?.Close();
				field = value;
			}
		} = null!;

		/// <summary>The left edge dock site indicator</summary>
		private Indicator IndLeft
		{
			get;
			set
			{
				if (field == value) return;
				field?.Close();
				field = value;
			}
		} = null!;

		/// <summary>The top edge dock site indicator</summary>
		private Indicator IndTop
		{
			get;
			set
			{
				if (field == value) return;
				field?.Close();
				field = value;
			}
		} = null!;

		/// <summary>The left edge dock site indicator</summary>
		private Indicator IndRight
		{
			get;
			set
			{
				if (field == value) return;
				field?.Close();
				field = value;
			}
		} = null!;

		/// <summary>The left edge dock site indicator</summary>
		private Indicator IndBottom
		{
			get;
			set
			{
				if (field == value) return;
				field?.Close();
				field = value;
			}
		} = null!;

		/// <summary>Enumerate all indicators</summary>
		private IEnumerable<Indicator> Indicators
		{
			get
			{
				yield return IndCrossLg;
				yield return IndCrossSm;
				yield return IndLeft;
				yield return IndTop;
				yield return IndRight;
				yield return IndBottom;
			}
		}

		/// <summary>Test the location 'screen_pt' as a possible drop location, and update the ghost and indicators</summary>
		private void HitTestDropLocations(Point screen_pt)
		{
			var pane = (DockPane?)null;
			var snap_to = (EDropSite?)null;
			var index = (int?)null;
			var over_indicator = false;

			// Look for an indicator that the mouse is over. Do this before updating the indicator
			// positions because if the mouse is over an indicator, we don't want to move it.
			foreach (var ind in Indicators)
			{
				if (!ind.IsVisible)
					continue;

				// Get the mouse point in indicator space and test if it's within the indicator regions
				var pt = ind.PointFromScreen(screen_pt);
				snap_to = ind.CheckSnapTo(pt);
				if (snap_to == null)
					continue;

				// The mouse is over an indicator, get the associated pane
				over_indicator = true;
				pane = ind.DockPane;
				break;
			}

			// Look for the dock pane under the mouse
			if (pane == null)
			{
				// Get all the tree hosts from owning DockContainer and check for mouse over a dock pane.
				// Careful though, have to check it's a dock pane that belongs to 'DockContainer'
				var tree_hosts = DockContainer.AllTreeHosts.Except(DraggedFromTree).Prepend(DraggedFromTree);
				foreach (var tree in tree_hosts)
				{
					var hit = VisualTreeHelper.HitTest(tree.Root, tree.Root.PointFromScreen(screen_pt));
					if (hit?.VisualHit is not DependencyObject dp ||
						dp.FindVisualParent<DockPane>(pred: x => ReferenceEquals(x.DockContainer, DockContainer), root: tree.Root) is not DockPane p)
						continue;

					pane = p;
					break;
				}
			}

			// Check whether the mouse is over the tab strip of the dock pane
			if (pane != null && !over_indicator)
			{
				var hit = VisualTreeHelper.HitTest(pane.TabStrip, pane.TabStrip.PointFromScreen(screen_pt));
				if (hit?.VisualHit is DependencyObject dp)
				{
					if (dp.FindVisualParent<TabButton>(root: pane.TabStrip) is TabButton tab)
					{
						snap_to = EDropSite.PaneCentre;
						index = pane.TabStrip.Buttons.IndexOf(tab);
					}
					else if (dp.FindVisualParent<TabStrip>(root: pane.TabStrip) is TabStrip ts)
					{
						snap_to = EDropSite.PaneCentre;
						index = pane.TabStrip.Buttons.Except(m_ghost_button).Count();
					}
				}
			}

			// Check whether the mouse is over the title bar of the dock pane
			if (pane != null && !over_indicator)
			{
				var hit = VisualTreeHelper.HitTest(pane.TitleBar, pane.TitleBar.PointFromScreen(screen_pt));
				if (hit?.VisualHit is DependencyObject dp && dp.FindVisualParent<Panel>(root: pane.TitleBar) != null)
					snap_to = EDropSite.PaneCentre;
			}

			// Determine the drop address
			if (pane == null || snap_to == null)
			{
				// No pane or snap-to means float in a new window
				DropAddress = Array.Empty<EDockSite>();
				TreeHost = null;
			}
			else
			{
				var snap = snap_to.Value;
				var ds = (EDockSite)(snap & EDropSite.DockSiteMask);
				if (ds == EDockSite.Centre)
				{
					// All EDockSite.Centre's dock to 'pane'
					DropAddress = pane.DockAddress;
				}
				else if (snap.HasFlag(EDropSite.Pane))
				{
					// Dock to a site within the current pane
					DropAddress = pane.DockAddress.Append(ds).ToArray();
				}
				else if (snap.HasFlag(EDropSite.Branch))
				{
					// Snap to a site in the branch that owns 'pane'
					var branch = pane.ParentBranch ?? throw new Exception("Pane has no parent branch");
					var address = branch.DockAddress.ToList();

					// While the child at 'ds' is a branch, append 'EDockSite.Centre's to the address
					for (; branch.Descendants[ds].Item is Branch b; branch = b, ds = EDockSite.Centre) { address.Add(ds); }
					address.Add(ds);

					DropAddress = address.ToArray();
				}
				else if (snap.HasFlag(EDropSite.Root))
				{
					// Snap to an auto hide dock site
					var branch = pane.RootBranch ?? throw new Exception("Pane has no root branch");
					var address = branch.DockAddress.ToList();

					// Auto hide panels always dock to the centre pane
					ds = EDockSite.Centre;

					// While the child at 'ds' is a branch, append 'EDockSite.Centre's to the address
					for (; branch.Descendants[ds].Item is Branch b; branch = b, ds = EDockSite.Centre) { address.Add(ds); }
					address.Add(ds);

					DropAddress = address.ToArray();
				}

				// Drop in the same tree as 'pane'
				TreeHost = pane.TreeHost;
				DropIndex = index;
			}

			// Ensure the drop target is in the foreground
			if (pane != null)
				GetWindow(pane).BringIntoView();

			// Update the position of the ghost
			PositionGhost(screen_pt);

			// If the mouse isn't over an indicator, update the positions of the indicators
			if (!over_indicator && pane != null)
				PositionCrossIndicator(pane);
		}

		/// <summary>Position the ghost window at the current drop address</summary>
		public void PositionGhost(Point screen_pt)
		{
			// Bounds in screen space covering the whole pane.
			// Clip in window space, clipping out the title, tab strip etc
			Rect bounds;
			Geometry clip;

			// No address means floating
			if (TreeHost == null || DropAddress.Length == 0)
			{
				bounds = new Rect(Point.Add(screen_pt, DraggeeOfs), DraggeeRect.Size);
				clip = new RectangleGeometry(DraggeeRect);
			}
			else
			{
				// Navigate as far down the tree as we can for the drop address.
				// The last dock site in the address should be a null child or a dock pane.
				// The second to last dock site should be a branch or a pane.
				var branch = TreeHost.Root;
				var ds = DropAddress.GetEnumerator();
				for (var more = ds.MoveNext(); more; more = ds.MoveNext())
				{
					if (!(branch.Descendants[(EDockSite)ds.Current!].Item is Branch b)) break;
					branch = b;
				}
				var target = branch.Descendants[(EDockSite)ds.Current!];
				if (target.Item is Branch)
					throw new Exception("The address ends at a branch node, not a pane or null-child");

				// Remove the ghost tab from it's current tabstrip
				if (m_ghost_button.TabStrip != null)
					m_ghost_button.TabStrip.Buttons.Remove(m_ghost_button);

				// If there is a pane at the target position then snap to fill the pane or some child area within the pane.
				if (target.Item is DockPane pane && pane.IsVisible)
				{
					// If there is one more dock site in the address, then the drop target is a child area within the content area of 'pane'.
					if (ds.MoveNext())
					{
						var rect = DockContainer.DockSiteBounds((EDockSite)ds.Current, pane.Centre.RenderArea(pane), EDockMask.None, DockSizeData.Halves);
						bounds = pane.RectToScreen(pane.RenderArea());
						clip = new RectangleGeometry(rect);
					}
					// Otherwise, the target area is the pane itself.
					else
					{
						// Add the ghost tab to the pane's tabstrip
						var idx = DropIndex ?? pane.TabStrip.Buttons.Count;
						pane.TabStrip.Buttons.Insert(idx, m_ghost_button);
						m_ghost_button.InvalidateArrange();

						// Get the pane area
						var rect = pane.RenderArea();
						bounds = pane.RectToScreen(rect);

						// Limit the shape of the ghost to the pane content area
						clip = new RectangleGeometry(rect);
						if (pane.TitleBar.IsVisible)
							clip = Geometry.Combine(clip, new RectangleGeometry(pane.TitleBar.RenderArea(pane)), GeometryCombineMode.Exclude, Transform.Identity);
						if (pane.TabStrip.IsVisible)
							clip = Geometry.Combine(clip, new RectangleGeometry(pane.TabStrip.RenderArea(pane)), GeometryCombineMode.Exclude, Transform.Identity);
						if (m_ghost_button.IsVisible)
							clip = Geometry.Combine(clip, new RectangleGeometry(m_ghost_button.RenderArea(pane)), GeometryCombineMode.Union, Transform.Identity);
					}
				}

				// Otherwise, the target position is an empty leaf of 'branch' or the target pane
				// is not visible. In either case, the user cannot see the drop location.
				else
				{
					var docksite = DropAddress.Back();

					// If the branch belongs to an auto hide panel, then the available area is
					// calculated from the main dock container. Otherwise, we expect the branch
					// to be visible and the available area is calculated from the branch's render area.
					if (branch.TreeHost is AutoHidePanel ahp)
					{
						var area = DockContainer.RenderArea();
						var rect = DockContainer.DockSiteBounds(ahp.DockSite, area, branch.DockedMask, branch.DockSize);
						bounds = DockContainer.RectToScreen(rect);
					}
					else if (branch.IsVisible)
					{
						var area = branch.RenderArea();
						var rect = DockContainer.DockSiteBounds(docksite, area, branch.DockedMask, branch.DockSize);
						bounds = branch.RectToScreen(rect);
					}
					else
					{
						throw new Exception("Cannot determine the area within which the dropped item will be added");
					}

					// Calculate the area that the dropped item would occupy within 'area'
					clip = new RectangleGeometry(Ghost.RectFromScreen(bounds));
				}
			}

			var dpi_scale = 96.0 / this.Dpi();

			// Update the bounds and region of the ghost
			Ghost.Clip = clip;
			Ghost.Left = bounds.X * dpi_scale;
			Ghost.Top = bounds.Y * dpi_scale;
			Ghost.Width = bounds.Width;
			Ghost.Height = bounds.Height;
		}

		/// <summary>Update the positions of the indicators. 'pane' is the pane under the mouse, or null</summary>
		private void PositionCrossIndicator(DockPane pane)
		{
			// Update the pane associated with the indicator
			IndCrossLg.DockPane = pane;
			IndCrossSm.DockPane = pane;

			// Update the position of the cross indicators
			// Auto hide panels are not valid drop targets, there are special indicators for dropping onto an auto hide pane.
			if (pane == null)
			{
				IndCrossLg.Visibility = Visibility.Collapsed;
				IndCrossSm.Visibility = Visibility.Collapsed;
				return;
			}

			var dpi_scale = 96.0 / this.Dpi();

			// Display the large cross indicator when over the Centre site
			// or the small cross indicator when over an edge site
			if (pane.DockSite == EDockSite.Centre)
			{
				var pt = Point.Subtract(pane.Centre.RenderArea(pane).Centre(), new Vector(IndCrossLg.Width * 0.5, IndCrossLg.Height * 0.5));
				pt = pane.PointToScreen(pt);
				IndCrossLg.Left = pt.X * dpi_scale;
				IndCrossLg.Top = pt.Y * dpi_scale;

				IndCrossLg.Visibility = Visibility.Visible;
				IndCrossSm.Visibility = Visibility.Collapsed;
			}
			else
			{
				var pt = Point.Subtract(pane.Centre.RenderArea(pane).Centre(), new Vector(IndCrossSm.Width * 0.5, IndCrossSm.Height * 0.5));
				pt = pane.PointToScreen(pt);
				IndCrossSm.Left = pt.X * dpi_scale;
				IndCrossSm.Top = pt.Y * dpi_scale;

				IndCrossLg.Visibility = Visibility.Collapsed;
				IndCrossSm.Visibility = Visibility.Visible;
			}
		}

		/// <summary>Return the hotspots for the given indicator</summary>
		private static Size DimensionsFor(EIndicator indy)
		{
			// These are the expected dimensions of the indicators in WPF virtual pixels
			// On high DPI screens the bitmaps will be scaled to these sizes.
			return indy switch
			{
				EIndicator.dock_site_cross_lg => new Size(128, 128),
				EIndicator.dock_site_cross_sm => new Size(64, 64),
				EIndicator.dock_site_left => new Size(25, 25),
				EIndicator.dock_site_top => new Size(25, 25),
				EIndicator.dock_site_right => new Size(25, 25),
				EIndicator.dock_site_bottom => new Size(25, 25),
				_ => throw new Exception("Unknown indicator type"),
			};
		}

		/// <summary>Return the hotspots for the given indicator</summary>
		private static Hotspot[] HotSpotsFor(EIndicator indy)
		{
			// Set the hotspot locations
			switch (indy)
			{
				default:
				{
					throw new Exception("Unknown indicator type");
				}
				case EIndicator.dock_site_cross_lg:
				{
					double a = 32, b = 50, c = 77, d = 95, cx = c - b;
					return
					[
						new(new RectangleGeometry(new Rect(b,b,cx,cx)), EDropSite.PaneCentre),
						new(Geometry_.MakePolygon(true, a,a, b,b, b,c, a,d), EDropSite.PaneLeft),
						new(Geometry_.MakePolygon(true, d,d, c,c, c,b, d,a), EDropSite.PaneRight),
						new(Geometry_.MakePolygon(true, a,a, d,a, c,b, b,b), EDropSite.PaneTop),
						new(Geometry_.MakePolygon(true, d,d, a,d, b,c, c,c), EDropSite.PaneBottom),
						new(new RectangleGeometry(new Rect(0,a,32,64)), EDropSite.BranchLeft),
						new(new RectangleGeometry(new Rect(d,a,32,64)), EDropSite.BranchRight),
						new(new RectangleGeometry(new Rect(a,0,64,32)), EDropSite.BranchTop),
						new(new RectangleGeometry(new Rect(a,d,64,32)), EDropSite.BranchBottom),
					];
				}
				case EIndicator.dock_site_cross_sm:
				{
					double a = 0, b = 18, c = 45, d = 63, cx = c - b;
					return
					[
						new(new RectangleGeometry(new Rect(b,b,cx,cx)), EDropSite.PaneCentre),
						new(Geometry_.MakePolygon(true, a,a, b,b, b,c, a,d), EDropSite.PaneLeft),
						new(Geometry_.MakePolygon(true, d,d, c,c, c,b, d,a), EDropSite.PaneRight),
						new(Geometry_.MakePolygon(true, a,a, d,a, c,b, b,b), EDropSite.PaneTop),
						new(Geometry_.MakePolygon(true, d,d, a,d, b,c, c,c), EDropSite.PaneBottom),
					];
				}
				case EIndicator.dock_site_left:
				{
					return [new Hotspot(new RectangleGeometry(new Rect(0, 0, 25, 25)), EDropSite.RootLeft)];
				}
				case EIndicator.dock_site_top:
				{
					return [new Hotspot(new RectangleGeometry(new Rect(0, 0, 25, 25)), EDropSite.RootTop)];
				}
				case EIndicator.dock_site_right:
				{
					return [new Hotspot(new RectangleGeometry(new Rect(0, 0, 25, 25)), EDropSite.RootRight)];
				}
				case EIndicator.dock_site_bottom:
				{
					return [new Hotspot(new RectangleGeometry(new Rect(0, 0, 25, 25)), EDropSite.RootBottom)];
				}
			}
		}

		/// <summary>Return the clipping region for the given indicator</summary>
		private static Geometry ClipFor(EIndicator indy)
		{
			switch (indy)
			{
				default:
				{
					throw new Exception("Unknown indicator type");
				}
				case EIndicator.dock_site_cross_lg:
				{
					var clip = Geometry.Empty;
					clip = Geometry.Combine(clip, new RectangleGeometry(new Rect(33, 43, 62, 42)), GeometryCombineMode.Union, Transform.Identity);
					clip = Geometry.Combine(clip, new RectangleGeometry(new Rect(43, 33, 42, 62)), GeometryCombineMode.Union, Transform.Identity);
					clip = Geometry.Combine(clip, new RectangleGeometry(new Rect(0, 39, 25, 50)), GeometryCombineMode.Union, Transform.Identity);
					clip = Geometry.Combine(clip, new RectangleGeometry(new Rect(103, 39, 25, 50)), GeometryCombineMode.Union, Transform.Identity);
					clip = Geometry.Combine(clip, new RectangleGeometry(new Rect(39, 0, 50, 25)), GeometryCombineMode.Union, Transform.Identity);
					clip = Geometry.Combine(clip, new RectangleGeometry(new Rect(39, 103, 50, 25)), GeometryCombineMode.Union, Transform.Identity);
					return clip;
				}
				case EIndicator.dock_site_cross_sm:
				{
					var clip = Geometry.Empty;
					clip = Geometry.Combine(clip, new RectangleGeometry(new Rect(1, 11, 62, 42)), GeometryCombineMode.Union, Transform.Identity);
					clip = Geometry.Combine(clip, new RectangleGeometry(new Rect(11, 1, 42, 62)), GeometryCombineMode.Union, Transform.Identity);
					return clip;
				}
				case EIndicator.dock_site_left:
				{
					return new RectangleGeometry(new Rect(0, 0, 25, 25));
				}
				case EIndicator.dock_site_top:
				{
					return new RectangleGeometry(new Rect(0, 0, 25, 25));
				}
				case EIndicator.dock_site_right:
				{
					return new RectangleGeometry(new Rect(0, 0, 25, 25));
				}
				case EIndicator.dock_site_bottom:
				{
					return new RectangleGeometry(new Rect(0, 0, 25, 25));
				}
			}
		}

		/// <summary>Base class for windows used as indicator graphics</summary>
		private class GhostBase : Window
		{
			public GhostBase(DragHandler owner)
			{
				Owner = owner;
				ShowInTaskbar = false;
				ShowActivated = false;
				AllowsTransparency = true;
				WindowStartupLocation = WindowStartupLocation.Manual;
				WindowStyle = WindowStyle.None;
				ResizeMode = ResizeMode.NoResize;
				Visibility = Visibility.Collapsed;
				Background = Brushes.Transparent;// Black;
				Opacity = 0.5f;
			}
			public DockContainer DockContainer => ((DragHandler)Owner).DockContainer;
		}

		/// <summary>A window that acts as a graphic while a pane or content is being dragged</summary>
		[DebuggerDisplay("{Name}")]
		private class GhostPane : GhostBase
		{
			public GhostPane(DragHandler owner, Point loc)
				: base(owner)
			{
				Name = "Ghost";
				Background = SystemColors.ActiveCaptionBrush;
				Left = loc.X;
				Top = loc.Y;
				Width = DraggeeRect.Width;
				Height = DraggeeRect.Height;
			}
		}

		/// <summary>A form that displays a docking site indicator</summary>
		[DebuggerDisplay("{Name}")]
		private class Indicator : GhostBase
		{
			private readonly Hotspot[] m_spots;
			public Indicator(DragHandler owner, EIndicator indy, DockPane? pane = null, Point? loc = null)
				: base(owner)
			{
				Name = indy.ToString();
				DockPane = pane;

				if (loc != null)
				{
					Left = loc.Value.X;
					Top = loc.Value.Y;
				}
				var sz = DimensionsFor(indy);
				Width = sz.Width;
				Height = sz.Height;

				// Add an image UI element
				var src = (ImageSource)DockContainer.Resources[indy.ToString()];
				var img = new Image { Source = src, Width = sz.Width, Height = sz.Height, Stretch = Stretch.None };
				Content = img;

				// Add the hotspots for this indicator
				m_spots = HotSpotsFor(indy);

				// Set the clip for the window
				Clip = ClipFor(indy);
			}

			/// <summary>Test 'pt' against the hotspots on this indicator</summary>
			public EDropSite? CheckSnapTo(Point pt)
			{
				return m_spots.FirstOrDefault(x => x.Area.FillContains(pt))?.DropSite;
			}

			/// <summary>The dock pane associated with this indicator</summary>
			public DockPane? DockPane { get; set; }
		}

		/// <summary>A region within the indicate that corresponds to a dock site location</summary>
		private class Hotspot
		{
			public Hotspot(Geometry area, EDropSite ds)
			{
				Area = area;
				DropSite = ds;
			}

			/// <summary>The area in the indicator that maps to 'DockSite'</summary>
			public Geometry Area;

			/// <summary>The drop site that this hot spot corresponds to</summary>
			public EDropSite DropSite;
		}

		/// <summary>Indicator types</summary>
		private enum EIndicator
		{
			dock_site_cross_lg,
			dock_site_cross_sm,
			dock_site_left,
			dock_site_top,
			dock_site_right,
			dock_site_bottom,
		}

		/// <summary>Places where panes/content can be dropped</summary>
		[Flags]
		private enum EDropSite
		{
			// Dock sites within the current pane
			Pane = 1 << 16,
			PaneCentre = Pane | EDockSite.Centre,
			PaneLeft = Pane | EDockSite.Left,
			PaneTop = Pane | EDockSite.Top,
			PaneRight = Pane | EDockSite.Right,
			PaneBottom = Pane | EDockSite.Bottom,

			// Dock sites within the current branch
			Branch = 1 << 17,
			BranchCentre = Branch | EDockSite.Centre,
			BranchLeft = Branch | EDockSite.Left,
			BranchTop = Branch | EDockSite.Top,
			BranchRight = Branch | EDockSite.Right,
			BranchBottom = Branch | EDockSite.Bottom,

			// Dock sites in the root level branch
			Root = 1 << 18,
			RootCentre = Root | EDockSite.Centre,
			RootLeft = Root | EDockSite.Left,
			RootTop = Root | EDockSite.Top,
			RootRight = Root | EDockSite.Right,
			RootBottom = Root | EDockSite.Bottom,

			// A mask the gets the EDockSite bits
			DockSiteMask = 0xff,
		}
	}
}
