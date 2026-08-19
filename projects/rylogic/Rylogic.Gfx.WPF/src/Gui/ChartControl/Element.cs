using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics;
using System.Windows;
using System.Windows.Input;
using System.Xml.Linq;
using Rylogic.Common;
using Rylogic.Extn;
using Rylogic.Gfx;
using Rylogic.Maths;
using Rylogic.Utility;

namespace Rylogic.Gui.WPF
{
	public partial class ChartControl
	{
		/// <summary>Base class for anything on a chart</summary>
		[DebuggerDisplay("{Name} {Id} {GetType().Name}")]
		public abstract class Element :IDisposable, IChartLegendItem, INotifyPropertyChanged
		{
			// Notes:
			//  - Graphics should be added/removed from the scene in 'UpdateSceneCore' because an element can
			//    exist before and after the scene (and therefore, window) exists. Don't be tempted to use
			//    the View3d.Object.Visible property instead.

			/// <summary>Buffers for creating the chart graphics</summary>
			protected List<View3d.Vertex> m_vbuf;
			protected List<View3d.Nugget> m_nbuf;
			protected List<ushort> m_ibuf;

			protected Element(Guid id, string? name = null, m4x4? position = null)
			{
				View3d = View3d.Create();
				m_vbuf = new List<View3d.Vertex>();
				m_nbuf = new List<View3d.Nugget>();
				m_ibuf = new List<ushort>();

				Id = id;
				m_name = name ?? string.Empty;
				m_colour = Colour32.Black;
				m_chart = null;
				m_o2w = position ?? m4x4.Identity;
				m_selected = false;
				m_hovered = false;
				m_visible = true;
				m_visible_to_find_range = true;
				m_screen_space = false;
				m_enabled = true;
				IsInvalidated = true;
				UserData = new Dictionary<Guid, object>();
			}
			protected Element(XElement node)
				: this(Guid.Empty)
			{
				// Note: Bounds, size, etc are not stored, the implementation
				// of the element provides those (typically in UpdateGfx)
				Id = node.Element(nameof(Id)).As<Guid>(Id);
				Name = node.Element(nameof(Name)).As<string>(Name);
				O2W = node.Element(nameof(O2W)).As<m4x4>(O2W);
			}
			public void Dispose()
			{
				Dispose(true);
				GC.SuppressFinalize(this);
			}
			protected virtual void Dispose(bool _)
			{
				Chart = null;
				Invalidated = null;
				PositionChanged = null;
				DataChanged = null;
				SelectedChanged = null;
				View3d = null!;
			}

			/// <summary>View3d context reference (needed because Elements can out-live the chart)</summary>
			private View3d View3d
			{
				get;
				set
				{
					if (field == value) return;
					Util.Dispose(ref field!);
					field = value;
				}
			} = null!;

			/// <summary>Non-null when the element has been added to a chart. Not virtual, override 'SetChartCore' instead</summary>
			public ChartControl? Chart
			{
				get => m_chart;
				set => SetChartInternal(value, true);
			}
			private ChartControl? m_chart;

			/// <summary>Assign the chart for this element</summary>
			internal void SetChartInternal(ChartControl? chart, bool update)
			{
				// An Element can be added to a chart by assigning to the Chart property
				// or by adding it to the Elements collection. It can be removed by setting
				// this property to null, by removing it from the Elements collection, or
				// by Disposing the element. Note: the chart does not own the elements, 
				// elements should only be disposed by the caller.
				if (m_chart == chart) return;

				// Detach from the old chart
				if (m_chart != null && update)
				{
					InvalidateChart();
					RemoveFromScene();
					m_chart.Elements.Remove(this);
				}

				// Assign to the new chart
				SetChartCore(chart);

				// Attach to the new chart
				if (m_chart != null && update)
				{
					if (m_chart.Elements.Contains(this))
						throw new Exception("Element is already in the Chart's Elements collection");

					m_chart.Elements.Add(this);
					UpdateScene();
					InvalidateChart();
				}
				Debug.Assert(CheckConsistency());
			}

			/// <summary>Add or remove this element from 'chart'</summary>
			protected virtual void SetChartCore(ChartControl? chart)
			{
				// Note: don't suspend events on Chart.Elements.
				// User code maybe watching for ListChanging events.

				// Remove this element from any selected collection on the outgoing chart
				// This also clears the 'selected' state for the element
				Selected = false;

				// Remove any graphics from the previous chart
				if (m_chart?.Scene != null)
					m_chart.Scene.Window.RemoveObjects(x => x == Id);

				// Set the new chart
				m_chart = chart;
			}

			/// <summary>Unique id for this element</summary>
			public Guid Id { get; }

			/// <summary>Debugging name for the element</summary>
			public string Name
			{
				get => m_name;
				set
				{
					if (Name == value) return;
					m_name = value;
					NotifyPropertyChanged(nameof(Name));
				}
			}
			private string m_name;

			/// <summary>Default identity colour</summary>
			public virtual Colour32 Colour
			{
				get => m_colour;
				set
				{
					if (Colour == value) return;
					m_colour = value;
					NotifyPropertyChanged(nameof(Colour));
				}
			}
			private Colour32 m_colour;

			/// <summary>Export to XML</summary>
			public virtual XElement ToXml(XElement node)
			{
				node.Add2(nameof(Id), Id, false);
				node.Add2(nameof(O2W), O2W, false);
				node.Add2(nameof(Name), Name, false);
				return node;
			}

			/// <summary>Import from XML. Used to update the state of this element without having to delete/recreate it</summary>
			protected virtual void FromXml(XElement node)
			{
				O2W = node.Element(nameof(O2W)).As<m4x4>(O2W);
				Name = node.Element(nameof(Name)).As<string>(Name);
			}

			/// <summary>Replace the contents of this element with data from 'node'</summary>
			internal void Update(XElement node)
			{
				// Don't validate the TypeAttribute as users may have sub-classed the element
				using (SuspendEvents())
					FromXml(node);

				Invalidate();
			}

			/// <summary>RAII object for suspending events on this element</summary>
			public Scope SuspendEvents()
			{
				return Scope.Create(
					() =>
					{
						return new IDisposable[]
						{
							//Invalidated.Suspend(),
							//DataChanged.Suspend(),
							//SelectedChanged.Suspend(),
							//PositionChanged.Suspend(),
							//SizeChanged.Suspend(),
						};
					},
					arr =>
					{
						Util.DisposeAll(arr);
					});
			}

			/// <summary>Raised whenever a property of this Element changes</summary>
			public event PropertyChangedEventHandler? PropertyChanged;
			protected void NotifyPropertyChanged(string prop_name)
			{
				PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(prop_name));
			}
			protected void SetProp<T>(ref T prop, T value, string name, bool invalidate_graphics, bool invalidate_chart)
			{
				if (Equals(prop, value)) return;
				prop = value;
				NotifyPropertyChanged(name);
				if (invalidate_graphics) Invalidate();
				if (invalidate_chart) InvalidateChart();
			}

			/// <summary>Raised whenever the element needs to be redrawn</summary>
			public event EventHandler? Invalidated;
			protected virtual void NotifyInvalidated()
			{
				IsInvalidated = true;
				InvalidateChart();
				Invalidated?.Invoke(this, EventArgs.Empty);
			}

			/// <summary>Raised whenever data associated with the element changes</summary>
			public event EventHandler? DataChanged;
			protected virtual void NotifyDataChanged()
			{
				// Raise data changed on this element, and propagate
				// the event to the containing chart as well.
				DataChanged?.Invoke(this, EventArgs.Empty);
				NotifyChartChanged();
			}

			/// <summary>Raised whenever the element is moved</summary>
			public event EventHandler? PositionChanged;
			protected virtual void NotifyPositionChanged()
			{
				PositionChanged?.Invoke(this, EventArgs.Empty);
				NotifyChartChanged();
			}

			/// <summary>Raised whenever the element changes size</summary>
			public event EventHandler? SizeChanged;
			protected virtual void NotifySizeChanged()
			{
				SizeChanged?.Invoke(this, EventArgs.Empty);
				NotifyChartChanged();
			}

			/// <summary>Raised whenever the element is selected or deselected</summary>
			public event EventHandler? SelectedChanged;
			protected virtual void NotifySelectedChanged()
			{
				SelectedChanged?.Invoke(this, EventArgs.Empty);
			}

			/// <summary>Raised whenever the element is hovered over with the mouse</summary>
			public event EventHandler? HoveredChanged;
			protected virtual void NotifyHoveredChanged()
			{
				HoveredChanged?.Invoke(this, EventArgs.Empty);
			}

			/// <summary>Signal that the chart needs laying out</summary>
			protected virtual void NotifyChartChanged()
			{
				if (Chart == null) return;
				Chart.OnChartChanged(new ChartChangedEventArgs(EChangeType.Edited));
			}

			/// <summary>Call 'Invalidate' on the containing chart</summary>
			protected virtual void InvalidateChart()
			{
				if (Chart == null) return;
				Chart.Invalidate();
			}

			/// <summary>Indicate that the graphics for this element needs to be recreated or modified</summary>
			public void Invalidate()
			{
				if (IsInvalidated) return;
				NotifyInvalidated();
			}

			/// <summary>True if this element has been invalidated. Call 'UpdateGfx' to clear the invalidated state</summary>
			public bool IsInvalidated { get; private set; }

			/// <summary>Set the selected state of this element</summary>
			internal void SetSelectedInternal(bool selected, bool update_selected_collection)
			{
				// This allows the chart to set the selected state without
				// adding/removing from the chart's 'Selected' collection.
				if (m_selected == selected) return;
				if (m_selected && update_selected_collection)
				{
					Chart?.Selected.Remove(this);
				}

				SetProp(ref m_selected, selected, nameof(Selected), true, true);

				if (m_selected && update_selected_collection)
				{
					// Selection state is changed by assigning to this property or by
					// addition/removal from the chart's 'Selected' collection.
					Chart?.Selected.Add(this);
				}

				// Notify observers about the selection change
				NotifySelectedChanged();
				Invalidate();
			}

			/// <summary>Set the selected state of this element</summary>
			internal void SetHoveredInternal(bool hovered, bool update_hovered_collection)
			{
				// This allows the chart to set the hovered state without
				// adding/removing from the chart's 'Hovered' collection.
				if (m_hovered == hovered) return;
				if (m_hovered && update_hovered_collection)
				{
					Chart?.Hovered.Remove(this);
				}

				SetProp(ref m_hovered, hovered, nameof(Hovered), true, true);

				if (m_hovered && update_hovered_collection)
				{
					// Hovered state is changed by assigning to this property or by
					// addition/removal from the chart's 'Hovered' collection.
					Chart?.Hovered.Add(this);
				}

				// Notify observers about the hovered change
				NotifyHoveredChanged();
				Invalidate();
			}

			/// <summary>Get/Set the selected state</summary>
			public bool Selected
			{
				get => m_selected;
				set => SetSelectedInternal(value, true);
			}
			private bool m_selected;

			/// <summary>Get/Set the hovered state</summary>
			public bool Hovered
			{
				get => m_hovered;
				internal set => SetHoveredInternal(value, true);
			}
			private bool m_hovered;

			/// <summary>Get/Set whether this element is visible in the chart</summary>
			public bool Visible
			{
				// Make an element visible/invisible, doesn't invalidate it, only the chart that is displaying it.
				get => m_visible;
				set
				{
					if (Visible == value) return;
					SetProp(ref m_visible, value, nameof(Visible), false, true);
					if (Visible)
						UpdateScene();
					else
						RemoveFromScene();
				}
			}
			private bool m_visible;

			/// <summary>Get/Set whether this element is enabled</summary>
			public bool Enabled
			{
				// Changing the enabled data of an element should result in the graphics changing in some way that implies 'Enabled/Disabled'
				get => m_enabled;
				set => SetProp(ref m_enabled, value, nameof(Enabled), true, true);
			}
			private bool m_enabled;

			/// <summary>True if this element is included when finding the range of data in the chart</summary>
			public bool VisibleToFindRange
			{
				// Making an element visible/invisible doesn't invalidate it, only the chart that is displaying it.
				get => m_visible_to_find_range;
				set => SetProp(ref m_visible_to_find_range, value, nameof(VisibleToFindRange), false, true);
			}
			private bool m_visible_to_find_range;

			/// <summary>True if this is a screen-space element (i.e. Position is in normalised screen space, not chart space)</summary>
			public bool ScreenSpace
			{
				get => m_screen_space;
				protected set => SetProp(ref m_screen_space, value, nameof(ScreenSpace), false, true);
			}
			private bool m_screen_space;

			/// <summary>Allow users to bind arbitrary data to the chart element</summary>
			public IDictionary<Guid, object> UserData { get; }

			/// <summary>Send this element to the bottom of the Z-order</summary>
			public void SendToBack()
			{
				// Z order is determined by position in the Elements collection
				if (Chart == null)
					return;

				// Save the chart pointer because removing the element will remove
				// it from the chart causing 'Chart' to become null before Insert is called.
				var chart = Chart;
				using (chart.Elements.SuspendEvents())
				{
					chart.Elements.Remove(this);
					chart.Elements.Insert(0, this);
				}
				chart.UpdateElementZOrder();
				InvalidateChart();
			}

			/// <summary>Bring this element to top of the stack.</summary>
			public void BringToFront()
			{
				// Z order is determined by position in the Elements collection
				if (Chart == null)
					return;

				// Save the chart pointer because removing the element will remove
				// it from the Chart causing 'Chart' to become null before Insert is called
				var chart = Chart;
				using (chart.Elements.SuspendEvents())
				{
					chart.Elements.Remove(this);
					chart.Elements.Add(this);
				}
				chart.UpdateElementZOrder();
				InvalidateChart();
			}

			/// <summary>The element to chart transform</summary>
			public m4x4 O2W
			{
				get => m_o2w;
				set
				{
					if (Math_.FEql(m_o2w, value)) return;
					SetO2W(value);
				}
			}
			private m4x4 m_o2w;

			/// <summary>Internal set position and raise event</summary>
			protected virtual void SetO2W(m4x4 o2w)
			{
				m_o2w = o2w;
				NotifyPositionChanged();
				NotifyPropertyChanged(nameof(O2W));
			}

			/// <summary>Get/Set the XY position of the element</summary>
			public v2 PositionXY
			{
				get => O2W.pos.xy;
				set
				{
					var o2p = O2W;
					O2W = new m4x4(o2p.rot, new v4(value, o2p.pos.z, o2p.pos.w));
				}
			}

			/// <summary>Get/Set the z position of the element</summary>
			public float PositionZ
			{
				get { return O2W.pos.z; }
				set
				{
					var o2p = O2W;
					o2p.pos.z = value;
					O2W = o2p;
				}
			}

			/// <summary>BBox for the element in chart space</summary>
			public virtual BBox Bounds => new(O2W.pos, v4.Zero);

			/// <summary>Get/Set the centre point of the element (in chart space)</summary>
			public v4 Centre
			{
				get => Bounds.Centre;
				set => O2W = new m4x4(O2W.rot, value + (O2W.pos - Centre));
			}

			/// <summary>Perform a hit test on this object. Returns null for no hit. 'scene_point' is typically used for screen-space hit tolerances</summary>
			public virtual HitTestResult.Hit? HitTest(v4 chart_point, v2 scene_point, ModifierKeys modifier_keys, EMouseBtns mouse_btns, View3d.Camera cam)
			{
				return null;
			}

			/// <summary>Handle a click event on this element</summary>
			internal void HandleClickedInternal(ChartClickedEventArgs args) => HandleClicked(args);
			protected virtual void HandleClicked(ChartClickedEventArgs args)
			{}

			/// <summary>Handle a drag event on this element</summary>
			internal void HandleDraggedInternal(ChartDraggedEventArgs args) => HandleDragged(args);
			protected virtual void HandleDragged(ChartDraggedEventArgs args)
			{}

			/// <summary>Translate this element from the drag start position</summary>
			public void DragTranslate(v4 delta, EDragState state)
			{
				var p = DragStartPosition;
				p.pos += delta;
				O2W = p;

				if (state == EDragState.Commit)
					DragStartPosition = O2W;
				if (state == EDragState.Cancel)
					O2W = DragStartPosition;
			}

			/// <summary>Position recorded at the time dragging starts</summary>
			public m4x4 DragStartPosition { get; internal set; }

			/// <summary>Update the graphics and object transforms associated with this element</summary>
			public void UpdateGfx(object? sender = null, EventArgs? args = null)
			{
				// Protect against reentrancy
				if (m_updating_gfx != 0 || !IsInvalidated) return;
				using var updating = Scope.Create(() => ++m_updating_gfx, () => --m_updating_gfx);

				UpdateGfxCore();
				IsInvalidated = false;
			}
			protected virtual void UpdateGfxCore()
			{
				// Recreate invalidated graphics models
			}
			private int m_updating_gfx;

			/// <summary>Add the graphics associated with this element to the scene</summary>
			internal void UpdateScene()
			{
				if (IsInvalidated)
					UpdateGfx();
				if (Chart == null || !Visible)
					return;

				UpdateSceneCore(Chart.Scene.Window, Chart.Scene.Camera);
			}
			protected virtual void UpdateSceneCore(View3d.Window window, View3d.Camera camera)
			{
				// Update graphics transforms
				// Add graphics to 'window'
				// Don't worry about Visible, it should already be handled
			}

			/// <summary>Remove the graphics associated with this element from the scene</summary>
			internal void RemoveFromScene()
			{
				if (Chart == null) return;
				RemoveFromSceneCore(Chart.Scene.Window);
			}
			protected virtual void RemoveFromSceneCore(View3d.Window window)
			{
				// Remove graphics from 'window'
				// Remember you can remove by context id if needed
			}

			/// <summary>Check the self consistency of this element</summary>
			public virtual bool CheckConsistency()
			{
				return true;
			}
		}
	}
}
