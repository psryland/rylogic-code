using System;
using System.Collections;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using Rylogic.Gui.WPF.ChartDetail;
using Rylogic.Utility;

namespace Rylogic.Gui.WPF
{
	public partial class ChartLegend :UserControl, INotifyPropertyChanged
	{
		// Notes:
		//  - Binding the ChartLegend is tricky. It works like this:
		//    - ChartControl as a 'Legend' property that can be set to any FrameworkElement.
		//    - Because this element is a child of the ChartControl (via Overlay), it's DataContext is the ChartControl.
		//    - If you want to set the Legend's DataContext to something else, you probably need to use 'DataContextRef'
		//    - If a 'ChartLegend' instance is assigned to the ChartControl's 'Legend' property, then the chart will
		//      initialise the 'ItemsSource' with the chart's 'Elements' collection (unless already set).
		//  - The simple usage is:
		//      <view3d:ChartControl
		//      	>
		//      	<view3d:ChartControl.Legend>
		//      		<view3d:ChartLegend/>
		//      	</view3d:ChartControl.Legend>
		//      </view3d:ChartControl>
		//
		//     ... or you can make the legend a static resource:
		//     <ResourceDictionary>
		//         <gui:DataContextRef x:Key="Ctx" Ctx="{Binding .}"/>
		//         <view3d:ChartLegend x:Key="MyLegend" ItemsSource="{Binding Ctx.MyLegendItems, Source={StaticResource Ctx}}" />
		//     </ResourceDictionary>
		//     ...
		//     <view3d:ChartControl
		//         Legend="{StaticResource MyLegend}"
		//         />
	
		public ChartLegend()
		{
			InitializeComponent();
			ToggleVisibility = Command.Create(this, ToggleVisibilityInternal);
			ItemActivated = Command.Create(this, ItemActivatedInternal);
			// Don't set DataContext, it's inherited from: ChartControl -> Canvas(overlay) -> ChartLegend
		}
		protected override void OnVisualParentChanged(DependencyObject oldParent)
		{
			if (oldParent is FrameworkElement old)
			{
				old.SizeChanged -= HandleParentSizeChanged;
			}
			base.OnVisualParentChanged(oldParent);
			if (Parent is FrameworkElement nue)
			{
				nue.SizeChanged += HandleParentSizeChanged;
			}

			void HandleParentSizeChanged(object? sender, SizeChangedEventArgs e)
			{
				PositionBasedOnAlignment();
			}
		}
		protected override void OnRenderSizeChanged(SizeChangedInfo sizeInfo)
		{
			base.OnRenderSizeChanged(sizeInfo);
			PositionBasedOnAlignment();
		}
		protected override void OnPropertyChanged(DependencyPropertyChangedEventArgs e)
		{
			base.OnPropertyChanged(e);
			if (e.Property == HorizontalAlignmentProperty ||
				e.Property == VerticalAlignmentProperty)
				PositionBasedOnAlignment();
		}

		/// <summary>True if the legend is expanded</summary>
		public bool Expanded
		{
			get => (bool)GetValue(ExpandedProperty);
			set => SetValue(ExpandedProperty, value);
		}
		private void Expanded_Changed()
		{
			NotifyPropertyChanged(nameof(Expanded));
		}
		public static readonly DependencyProperty ExpandedProperty = Gui_.DPRegister<ChartLegend>(nameof(Expanded), Boxed.True, Gui_.EDPFlags.TwoWay|Gui_.EDPFlags.AffectsMeasure|Gui_.EDPFlags.AffectsArrange);

		/// <summary>True if items in the legend can be reordered</summary>
		public bool CanUsersReorderItems
		{
			get => (bool)GetValue(CanUsersReorderItemsProperty);
			set => SetValue(CanUsersReorderItemsProperty, value);
		}
		public static readonly DependencyProperty CanUsersReorderItemsProperty = Gui_.DPRegister<ChartLegend>(nameof(CanUsersReorderItems), Boxed.False, Gui_.EDPFlags.None);

		/// <summary>The elements to display</summary>
		public IEnumerable ItemsSource
		{
			get => (IEnumerable)GetValue(ItemsSourceProperty);
			set => SetValue(ItemsSourceProperty, value);
		}
		public static readonly DependencyProperty ItemsSourceProperty = Gui_.DPRegister<ChartLegend>(nameof(ItemsSource), null, Gui_.EDPFlags.None);

		/// <summary>Legend dock location</summary>
		public bool Floating
		{
			get => (bool)GetValue(FloatingProperty);
			set => SetValue(FloatingProperty, value);
		}
		private void Floating_Changed()
		{
			NotifyPropertyChanged(nameof(Floating));
		}
		public static readonly DependencyProperty FloatingProperty = Gui_.DPRegister<ChartLegend>(nameof(Floating), true, Gui_.EDPFlags.None);

		/// <summary>The selected element</summary>
		public IChartLegendItem? SelectedItem
		{
			get;
			set
			{
				if (field == value) return;
				field = value;
				NotifyPropertyChanged(nameof(SelectedItem));
			}
		}

		/// <summary>Show/Hide the current item</summary>
		public Command ToggleVisibility { get; }
		private void ToggleVisibilityInternal()
		{
			if (SelectedItem == null)
				return;

			SelectedItem.Visible = !SelectedItem.Visible;
		}

		/// <summary></summary>
		public Command ItemActivated { get; }
		private void ItemActivatedInternal()
		{
		}

		/// <summary>Position the legend based on the alignment settings</summary>
		public void PositionBasedOnAlignment()
		{
			if (Gui_.FindVisualParent<Canvas>(this) is not Canvas canvas)
				return;
			if (!IsMeasureValid || !IsArrangeValid)
				return;

			// Current and target position for the legend based on H/V alignment
			var pt0 = new Point(Canvas.GetLeft(this), Canvas.GetTop(this));
			var pt1 = new Point(
				HorizontalAlignment switch
				{
					HorizontalAlignment.Left => 0,
					HorizontalAlignment.Center => (canvas.ActualWidth - ActualWidth) / 2,
					HorizontalAlignment.Stretch => (canvas.ActualWidth - ActualWidth) / 2,
					HorizontalAlignment.Right => canvas.ActualWidth - ActualWidth,
					_ => throw new Exception("Unknown horizontal alignment"),
				},
				VerticalAlignment switch
				{
					VerticalAlignment.Top => 0,
					VerticalAlignment.Center => (canvas.ActualHeight - ActualHeight) / 2,
					VerticalAlignment.Stretch => (canvas.ActualHeight - ActualHeight) / 2,
					VerticalAlignment.Bottom => canvas.ActualHeight - ActualHeight,
					_ => throw new Exception("Unknown vertical alignment"),
				});

			// If near a dock position, snap there
			const double SnapDistanceSq = 25 * 25;
			var docked = (pt1 - pt0).LengthSquared < SnapDistanceSq;

			if (Floating && !docked && m_last_position is Point last_pt)
			{
				// Last point is dock location last time, so (pt1 - last_pt) is the delta to apply
				Canvas.SetLeft(this, pt0.X + (pt1.X - last_pt.X));
				Canvas.SetTop (this, pt0.Y + (pt1.Y - last_pt.Y));
			}
			else
			{
				Canvas.SetLeft(this, pt1.X);
				Canvas.SetTop(this, pt1.Y);
			}

			m_last_position = pt1;
		}
		private Point? m_last_position;

		/// <summary>Handle dragging of the legend</summary>
		private void HandleDrag(object? sender, MouseButtonEventArgs e)
		{
			if (sender is not FrameworkElement fe)
				return;
			if (!Floating)
				return;
			if (Gui_.FindVisualParent<Canvas>(this) is not Canvas canvas)
				return;

			Util.Dispose(ref m_mouse_capture);
			if (e.LeftButton == MouseButtonState.Pressed)
			{
				m_mouse_offset = e.GetPosition(this);
				m_mouse_capture = Scope.Create(
					() =>
					{
						fe.CaptureMouse();
						fe.MouseMove += DoDrag;
						e.Handled = true;
					},
					() =>
					{
						fe.MouseMove -= DoDrag;
						fe.ReleaseMouseCapture();
						e.Handled = true;
					});
			}

			// Handle dragging
			void DoDrag(object? sender, MouseEventArgs e)
			{
				var pt = e.GetPosition(canvas);
				Canvas.SetLeft(this, pt.X - m_mouse_offset.X);
				Canvas.SetTop(this, pt.Y - m_mouse_offset.Y);
				e.Handled = true;
			}
		}
		private IDisposable? m_mouse_capture;
		private Point m_mouse_offset;

		/// <inheritdoc/>
		public event PropertyChangedEventHandler? PropertyChanged;
		private void NotifyPropertyChanged(string prop_name) => PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(prop_name));
	}
}
