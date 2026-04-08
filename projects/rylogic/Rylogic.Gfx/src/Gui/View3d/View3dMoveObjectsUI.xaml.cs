using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Shapes;
using Rylogic.Gfx;
using Rylogic.Maths;

namespace Rylogic.Gui.WPF
{
	public partial class View3dMoveObjectsUI : Window, INotifyPropertyChanged
	{
		private readonly View3d.Window m_window;

		// Touch pad state
		private Canvas? m_active_pad;
		private Point m_drag_start;
		private float m_pos_x_at_drag_start;
		private float m_pos_y_at_drag_start;
		private float m_pos_z_at_drag_start;
		private double m_sensitivity = 10.0;
		private Ellipse? m_crosshair;

		public View3dMoveObjectsUI(Window? owner, IEnumerable<View3d.Object> objects, View3d.Window window)
		{
			InitializeComponent();
			Owner = owner;
			if (owner != null) Icon = owner.Icon;

			Objects = objects.ToList();
			m_window = window;

			Accept = Command.Create(this, AcceptInternal);

			DataContext = this;

			// Initialise sensitivity from first object's bounding box
			UpdateSensitivity();

			// Initialise position from the first object
			UpdatePositionFromFirstObject();

			// Draw touch pad visuals after layout
			Loaded += (s, e) =>
			{
				DrawTouchPadVisuals(m_pad_xy, "X", "Y");
				DrawTouchPadVisuals(m_pad_yz, "Y", "Z");
				DrawTouchPadVisuals(m_pad_zx, "Z", "X");
			};
		}

		/// <summary>The objects being moved</summary>
		public IReadOnlyList<View3d.Object> Objects { get; }

		/// <summary>Display string for the objects being moved</summary>
		public string ObjectNames
		{
			get
			{
				if (Objects.Count == 0) return "(none)";
				if (Objects.Count == 1) return Objects[0].Name;
				return $"{Objects[0].Name}, ... ({Objects.Count} objects)";
			}
		}

		/// <summary>True when updating position from object (suppresses ApplyPosition feedback)</summary>
		private bool m_updating;

		/// <summary>X component of the position</summary>
		public float PositionX
		{
			get;
			set
			{
				if (field == value) return;
				field = value;
				NotifyPropertyChanged(nameof(PositionX));
				if (!m_updating) ApplyPosition();
			}
		}

		/// <summary>Y component of the position</summary>
		public float PositionY
		{
			get;
			set
			{
				if (field == value) return;
				field = value;
				NotifyPropertyChanged(nameof(PositionY));
				if (!m_updating) ApplyPosition();
			}
		}

		/// <summary>Z component of the position</summary>
		public float PositionZ
		{
			get;
			set
			{
				if (field == value) return;
				field = value;
				NotifyPropertyChanged(nameof(PositionZ));
				if (!m_updating) ApplyPosition();
			}
		}

		/// <summary>True if editing in world space</summary>
		public bool IsWorldSpace
		{
			get;
			set
			{
				if (field == value) return;
				field = value;
				NotifyPropertyChanged(nameof(IsWorldSpace));
				NotifyPropertyChanged(nameof(IsParentSpace));

				// Re-read position in the new coordinate space
				UpdatePositionFromFirstObject();
			}
		} = true;

		/// <summary>True if editing in parent space (inverse of IsWorldSpace, for radio button binding)</summary>
		public bool IsParentSpace
		{
			get => !IsWorldSpace;
			set => IsWorldSpace = !value;
		}

		/// <summary>Accept/close button</summary>
		public Command Accept { get; }
		private void AcceptInternal()
		{
			Close();
		}

		/// <summary>Read position from the first object and populate text boxes</summary>
		private void UpdatePositionFromFirstObject()
		{
			if (Objects.Count == 0) return;
			var obj = Objects[0];

			m_updating = true;
			try
			{
				var pos = IsWorldSpace ? obj.O2WGet().pos : obj.O2PGet().pos;
				PositionX = pos.x;
				PositionY = pos.y;
				PositionZ = pos.z;
			}
			finally
			{
				m_updating = false;
			}
		}

		/// <summary>Set sensitivity based on the bounding box of the first object</summary>
		private void UpdateSensitivity()
		{
			if (Objects.Count == 0) { m_sensitivity = 10.0; return; }

			var bbox = Objects[0].BBoxMS(View3d.EBBoxFlags.IncludeChildren);
			m_sensitivity = bbox.IsValid ? 10.0 * bbox.Diametre : 10.0;
			if (m_sensitivity < 0.001) m_sensitivity = 10.0;
		}

		/// <summary>Apply the current position to all objects</summary>
		private void ApplyPosition()
		{
			if (Objects.Count == 0) return;

			var new_pos = new v4(PositionX, PositionY, PositionZ, 1f);

			foreach (var obj in Objects)
			{
				if (IsWorldSpace)
				{
					// Set position in world space — O2WSet internally computes the correct O2P
					var o2w = obj.O2WGet();
					o2w.pos = new_pos;
					obj.O2WSet(o2w);
				}
				else
				{
					// Set position directly in parent space
					var o2p = obj.O2PGet();
					o2p.pos = new_pos;
					obj.O2PSet(o2p);
				}
			}

			m_window.Invalidate();
		}

		#region Touch Pad

		/// <summary>Draw axis lines, center dot, and labels on a touch pad canvas</summary>
		private void DrawTouchPadVisuals(Canvas pad, string h_axis, string v_axis)
		{
			var w = pad.ActualWidth;
			var h = pad.ActualHeight;
			var cx = w / 2;
			var cy = h / 2;

			// Horizontal axis line
			var h_line = new Line
			{
				X1 = 0, Y1 = cy, X2 = w, Y2 = cy,
				Stroke = Brushes.DimGray,
				StrokeThickness = 0.5,
				IsHitTestVisible = false,
			};
			Canvas.SetZIndex(h_line, -1);
			pad.Children.Add(h_line);

			// Vertical axis line
			var v_line = new Line
			{
				X1 = cx, Y1 = 0, X2 = cx, Y2 = h,
				Stroke = Brushes.DimGray,
				StrokeThickness = 0.5,
				IsHitTestVisible = false,
			};
			Canvas.SetZIndex(v_line, -1);
			pad.Children.Add(v_line);

			// Center dot
			var dot = new Ellipse
			{
				Width = 5,
				Height = 5,
				Fill = Brushes.DimGray,
				IsHitTestVisible = false,
			};
			Canvas.SetLeft(dot, cx - 2.5);
			Canvas.SetTop(dot, cy - 2.5);
			pad.Children.Add(dot);

			// Horizontal axis label (right side)
			var h_label = new TextBlock
			{
				Text = h_axis,
				FontSize = 10,
				Foreground = Brushes.DimGray,
				IsHitTestVisible = false,
			};
			Canvas.SetRight(h_label, 3);
			Canvas.SetTop(h_label, cy - 8);
			pad.Children.Add(h_label);

			// Vertical axis label (top)
			var v_label = new TextBlock
			{
				Text = v_axis,
				FontSize = 10,
				Foreground = Brushes.DimGray,
				IsHitTestVisible = false,
			};
			Canvas.SetLeft(v_label, cx + 3);
			Canvas.SetTop(v_label, 2);
			pad.Children.Add(v_label);

			// Pad title (top-left)
			var title = new TextBlock
			{
				Text = $"{h_axis}{v_axis}",
				FontSize = 9,
				Foreground = Brushes.Gray,
				IsHitTestVisible = false,
			};
			Canvas.SetLeft(title, 3);
			Canvas.SetTop(title, 2);
			pad.Children.Add(title);
		}

		/// <summary>Begin a touch pad drag</summary>
		private void OnTouchPadMouseDown(object sender, MouseButtonEventArgs e)
		{
			if (sender is not Canvas pad) return;

			m_active_pad = pad;
			m_drag_start = e.GetPosition(pad);
			m_pos_x_at_drag_start = PositionX;
			m_pos_y_at_drag_start = PositionY;
			m_pos_z_at_drag_start = PositionZ;

			// Show crosshair
			m_crosshair = new Ellipse
			{
				Width = 7,
				Height = 7,
				Fill = Brushes.Red,
				IsHitTestVisible = false,
			};
			Canvas.SetLeft(m_crosshair, m_drag_start.X - 3.5);
			Canvas.SetTop(m_crosshair, m_drag_start.Y - 3.5);
			pad.Children.Add(m_crosshair);

			pad.CaptureMouse();
			e.Handled = true;
		}

		/// <summary>Update position during touch pad drag</summary>
		private void OnTouchPadMouseMove(object sender, MouseEventArgs e)
		{
			if (m_active_pad == null || sender is not Canvas pad || pad != m_active_pad) return;
			if (e.LeftButton != MouseButtonState.Pressed) return;

			var pos = e.GetPosition(pad);
			var dx = (pos.X - m_drag_start.X) / pad.ActualWidth;
			var dy = -(pos.Y - m_drag_start.Y) / pad.ActualHeight; // Y is inverted (screen Y goes down, world Y goes up)

			var scale = (float)m_sensitivity;
			var tag = pad.Tag as string ?? "";

			m_updating = true;
			try
			{
				switch (tag)
				{
					case "XY":
						PositionX = m_pos_x_at_drag_start + (float)dx * scale;
						PositionY = m_pos_y_at_drag_start + (float)dy * scale;
						break;
					case "YZ":
						PositionY = m_pos_y_at_drag_start + (float)dx * scale;
						PositionZ = m_pos_z_at_drag_start + (float)dy * scale;
						break;
					case "ZX":
						PositionZ = m_pos_z_at_drag_start + (float)dx * scale;
						PositionX = m_pos_x_at_drag_start + (float)dy * scale;
						break;
				}
			}
			finally
			{
				m_updating = false;
			}

			ApplyPosition();

			// Move crosshair
			if (m_crosshair != null)
			{
				Canvas.SetLeft(m_crosshair, pos.X - 3.5);
				Canvas.SetTop(m_crosshair, pos.Y - 3.5);
			}

			e.Handled = true;
		}

		/// <summary>End a touch pad drag</summary>
		private void OnTouchPadMouseUp(object sender, MouseButtonEventArgs e)
		{
			if (m_active_pad == null) return;

			// Remove crosshair
			if (m_crosshair != null)
			{
				m_active_pad.Children.Remove(m_crosshair);
				m_crosshair = null;
			}

			m_active_pad.ReleaseMouseCapture();
			m_active_pad = null;
			e.Handled = true;
		}

		/// <summary>Adjust touch pad sensitivity via mouse wheel</summary>
		private void OnTouchPadMouseWheel(object sender, MouseWheelEventArgs e)
		{
			if (e.Delta > 0)
				m_sensitivity *= Math.Sqrt(2.0);
			else
				m_sensitivity /= Math.Sqrt(2.0);

			// Clamp to reasonable range
			m_sensitivity = Math.Max(0.001, Math.Min(m_sensitivity, 1e8));
			e.Handled = true;
		}

		#endregion

		/// <inheritdoc/>
		public event PropertyChangedEventHandler? PropertyChanged;
		private void NotifyPropertyChanged(string prop_name) => PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(prop_name));
	}
}
