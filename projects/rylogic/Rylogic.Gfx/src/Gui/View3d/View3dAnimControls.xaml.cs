using System;
using System.ComponentModel;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Controls.Primitives;
using System.Windows.Input;
using Rylogic.Gfx;
using Rylogic.Maths;
using Rylogic.Utility;

namespace Rylogic.Gui.WPF
{
	public partial class View3dAnimControls : Grid, INotifyPropertyChanged
	{
		// Notes:
		//  - There is conflicting control of the main slider thumb position. While an animation is playing
		//    we want the thumb to represent the current animation clock. While not playing, the thumb becomes
		//    authoritative, setting the animation clock when moved.

		public View3dAnimControls()
		{
			InitializeComponent();
			Unloaded += HandleUnloaded;
			Reset = Command.Create(this, ResetInternal);
			Play = Command.Create(this, PlayInternal);
			Pause = Command.Create(this, PauseInternal);
			StepBack = Command.Create(this, StepBackInternal);
			StepForward = Command.Create(this, StepForwardInternal);
			DataContext = this;
		}

		/// <summary>The window in which to control animations</summary>
		public View3d.Window? ViewWindow
		{
			get => (View3d.Window)GetValue(ViewWindowProperty);
			set => SetValue(ViewWindowProperty, value);
		}
		private void ViewWindow_Changed(View3d.Window? new_value, View3d.Window? old_value)
		{
			if (old_value != null)
			{
				old_value.OnAnimationEvent -= HandleAnimationEvent;
			}
			if (new_value != null)
			{
				new_value.OnAnimationEvent += HandleAnimationEvent;
			}
			if (ThumbDragging)
			{
				ThumbDragging = false;
			}
			m_slider_frame = NativeFrame;
			NotifyPropertyChanged(nameof(ViewWindow));
			NotifyPropertyChanged(nameof(Animating));
			NotifyPropertyChanged(nameof(AnimClock));
			NotifyPropertyChanged(nameof(Frame));
			NotifyPropertyChanged(nameof(SliderFrame));

			// Handlers
			void HandleAnimationEvent(object? sender, View3d.AnimationEventArgs e)
			{
				Util.BreakIf(!Util.IsMainThread);
				NotifyPropertyChanged(nameof(Animating));
				if (!ThumbDragging)
				{
					m_slider_frame = NativeFrame;
					NotifyPropertyChanged(nameof(AnimClock));
					NotifyPropertyChanged(nameof(Frame));
					NotifyPropertyChanged(nameof(SliderFrame));
				}
			}
		}
		public static readonly DependencyProperty ViewWindowProperty = Gui_.DPRegister<View3dAnimControls>(nameof(ViewWindow), null, Gui_.EDPFlags.None);

		/// <summary>Mouse wheel causes the slider to move</summary>
		private void Slider_PreviewMouseWheel(object sender, MouseWheelEventArgs e)
		{
			if (ViewWindow == null)
				return;

			var modifiers = Keyboard.Modifiers;
			var shift_down = modifiers.HasFlag(ModifierKeys.Shift);
			var control_down = modifiers.HasFlag(ModifierKeys.Control);
			var frame_delta =
				shift_down && control_down ? 0.01 :
				shift_down ? 0.1 :
				1.0;

			var step = frame_delta / FrameRate;
			AnimClock += e.Delta > 0 ? +step : -step;

			// Mark as handled to prevent bubbling
			e.Handled = true;
		}

		/// <summary>Animation time</summary>
		public double AnimClock
		{
			get => ThumbDragging ? SliderFrame / FrameRate : NativeClock;
			set
			{
				if (AnimClock == value || ViewWindow == null) return;
				SeekToClock(value);
				NotifyPropertyChanged(nameof(AnimClock));
				NotifyPropertyChanged(nameof(Frame));
				NotifyPropertyChanged(nameof(SliderFrame));
			}
		}

		/// <summary>The size of each step per second. Set to 0 for real time</summary>
		public double StepSize
		{
			get => m_step_size;
			set
			{
				if (StepSize == value) return;
				m_step_size = value;
				if (Animating) Play.Execute();
				NotifyPropertyChanged(nameof(StepSize));
			}
		}

		/// <summary>The assumed frame rate of all animation</summary>
		public double FrameRate
		{
			get => m_frame_rate;
			set
			{
				if (FrameRate == value) return;
				m_frame_rate = value;
				NotifyPropertyChanged(nameof(FrameRate));
				NotifyPropertyChanged(nameof(Frame));
				NotifyPropertyChanged(nameof(SliderFrame));
			}
		}

		/// <summary>True if the animation is running</summary>
		public bool Animating => ViewWindow?.Animating ?? false;

		/// <summary>If true, animations play relative to their start frame</summary>
		public bool RelativeTime
		{
			get => m_relative_time;
			set
			{
				if (RelativeTime == value) return;
				m_relative_time = value;
				NotifyPropertyChanged(nameof(RelativeTime));
			}
		}

		/// <summary>The start frame</summary>
		public int Frame0
		{
			get => m_frame0;
			set
			{
				if (Frame0 == value) return;
				m_frame0 = value;
				NotifyPropertyChanged(nameof(Frame0));
			}
		}

		/// <summary>The current frame</summary>
		public double Frame
		{
			get => ThumbDragging ? SliderFrame : NativeFrame;
			set
			{
				if (Frame == value) return;
				AnimClock = value / FrameRate;
			}
		}

		/// <summary>The value used by the frame slider thumb.</summary>
		public double SliderFrame
		{
			get => ThumbDragging ? m_slider_frame : Frame;
			set
			{
				if (SliderFrame == value) return;

				if (ThumbDragging)
				{
					// Mouse-position sampling owns the value while scrubbing; reject any WPF thumb-delta writes that slip through.
					NotifyPropertyChanged(nameof(SliderFrame));
					return;
				}

				Frame = value;
			}
		}

		/// <summary>The end frame</summary>
		public int FrameN
		{
			get => m_frameN;
			set
			{
				if (FrameN == value) return;
				m_frameN = value;
				NotifyPropertyChanged(nameof(FrameN));
			}
		}

		/// <summary>True while the thumb is being dragged by the user</summary>
		private bool ThumbDragging
		{
			get => m_thumb_dragging;
			set
			{
				if (ThumbDragging == value) return;
				m_thumb_dragging = value;
			}
		}
		private double m_slider_frame;
		private double m_step_size;
		private double m_frame_rate = 24.0;
		private int m_frame0;
		private int m_frameN = 100;
		private bool m_thumb_dragging;
		private bool m_relative_time;

		/// <summary>Cancel the local scrub session when this control is no longer visible.</summary>
		private void HandleUnloaded(object sender, RoutedEventArgs e)
		{
			ThumbDragging = false;
		}

		/// <summary>Reset the anim clock to 0</summary>
		public Command Reset { get; }
		private void ResetInternal()
		{
			if (ViewWindow == null) return;
			ViewWindow.AnimControl(View3d.EAnimCommand.Reset);
		}

		/// <summary>Start the animation</summary>
		public Command Play { get; }
		private void PlayInternal()
		{
			if (ViewWindow == null) return;
			ViewWindow.AnimControl(View3d.EAnimCommand.Play, StepSize);
		}

		/// <summary>Stop the animation</summary>
		public Command Pause { get; }
		private void PauseInternal()
		{
			if (ViewWindow == null) return;
			ViewWindow.AnimControl(View3d.EAnimCommand.Stop);
		}

		/// <summary>Step the animation one frame backwards</summary>
		public Command StepBack { get; }
		private void StepBackInternal()
		{
			if (ViewWindow == null) return;
			ViewWindow.AnimControl(View3d.EAnimCommand.Step, -StepSize);
		}

		/// <summary>Step the animation one frame forwards</summary>
		public Command StepForward { get; }
		private void StepForwardInternal()
		{
			if (ViewWindow == null) return;
			ViewWindow.AnimControl(View3d.EAnimCommand.Step, +StepSize);
		}

		/// <summary>Enter scrub mode before the slider can update its value from the mouse down event.</summary>
		private void FrameSlider_PreviewMouseLeftButtonDown(object sender, MouseButtonEventArgs e)
		{
			if (sender is not Slider slider)
				return;

			if (!BeginSliderScrub(slider))
				return;

			UpdateSliderScrub(slider, e.GetPosition(slider));
			slider.Focus();
			slider.CaptureMouse();
			e.Handled = true;
		}

		/// <summary>Update the scrub value from absolute mouse position instead of WPF thumb drag deltas.</summary>
		private void FrameSlider_PreviewMouseMove(object sender, MouseEventArgs e)
		{
			if (!ThumbDragging)
				return;

			if (sender is Slider slider && e.LeftButton == MouseButtonState.Pressed)
			{
				UpdateSliderScrub(slider, e.GetPosition(slider));
			}
			else
			{
				CompleteSliderScrub();
			}
			e.Handled = true;
		}

		/// <summary>Leave scrub mode after the slider has processed the final mouse up value.</summary>
		private void FrameSlider_PreviewMouseLeftButtonUp(object sender, MouseButtonEventArgs e)
		{
			if (!ThumbDragging)
				return;

			if (sender is Slider slider)
			{
				UpdateSliderScrub(slider, e.GetPosition(slider));
				CompleteSliderScrub();
				if (slider.IsMouseCaptured)
					slider.ReleaseMouseCapture();
			}
			else
			{
				CompleteSliderScrub();
			}
			e.Handled = true;
		}

		/// <summary>Commit the scrub value if mouse capture is cancelled after the button is released.</summary>
		private void FrameSlider_LostMouseCapture(object sender, MouseEventArgs e)
		{
			CompleteSliderScrub();
		}

		/// <summary>Start scrubbing with the slider's current value as the local preview value.</summary>
		private bool BeginSliderScrub(Slider slider)
		{
			if (ThumbDragging || ViewWindow == null)
				return false;

			// Seed directly from the current control value so the first drag update cannot snap back to a stale animation event.
			var was_animating = Animating;
			m_slider_frame = slider.Value;
			ThumbDragging = true;
			if (was_animating)
				ViewWindow.AnimControl(View3d.EAnimCommand.Stop);

			NotifyPropertyChanged(nameof(Animating));
			NotifyPropertyChanged(nameof(AnimClock));
			NotifyPropertyChanged(nameof(Frame));
			NotifyPropertyChanged(nameof(SliderFrame));
			return true;
		}

		/// <summary>Update the local scrub value from a mouse position over the slider.</summary>
		private void UpdateSliderScrub(Slider slider, Point point)
		{
			if (!ThumbDragging)
				return;

			SetSliderFrame(FrameFromSliderPoint(slider, point));
		}

		/// <summary>Convert an absolute mouse point over the slider into a frame value.</summary>
		private double FrameFromSliderPoint(Slider slider, Point point)
		{
			var minimum = slider.Minimum;
			var maximum = slider.Maximum;
			if (maximum <= minimum)
				return minimum;

			var track = slider.Template.FindName("PART_Track", slider) as Track;
			var track_x = 0.0;
			var track_width = slider.ActualWidth;
			var thumb_width = 0.0;
			if (track != null)
			{
				track_x = track.TranslatePoint(new Point(), slider).X;
				track_width = track.ActualWidth;
				thumb_width = track.Thumb?.ActualWidth ?? 0.0;
			}

			var span = Math.Max(track_width - thumb_width, 1.0);
			var frac = (point.X - track_x - thumb_width / 2.0) / span;
			frac = Math_.Clamp(frac, 0.0, 1.0);
			if (slider.IsDirectionReversed)
				frac = 1.0 - frac;

			return minimum + frac * (maximum - minimum);
		}

		/// <summary>Set the local slider frame while the mouse owns scrub control.</summary>
		private void SetSliderFrame(double frame)
		{
			if (m_slider_frame == frame)
				return;

			m_slider_frame = frame;
			NotifyPropertyChanged(nameof(AnimClock));
			NotifyPropertyChanged(nameof(Frame));
			NotifyPropertyChanged(nameof(SliderFrame));
			SeekToFrame(frame);
		}

		/// <summary>Commit the latest local preview value back to the native animation clock.</summary>
		private void CompleteSliderScrub()
		{
			if (!ThumbDragging)
				return;

			var frame = SliderFrame;
			SeekToFrame(frame);
			m_slider_frame = NativeFrame;
			ThumbDragging = false;
			NotifyPropertyChanged(nameof(AnimClock));
			NotifyPropertyChanged(nameof(Frame));
			NotifyPropertyChanged(nameof(SliderFrame));
		}

		/// <summary>Seek the animation clock to 'frame'.</summary>
		private void SeekToFrame(double frame)
		{
			SeekToClock(frame / FrameRate);
		}

		/// <summary>Seek the animation clock to 'clock'.</summary>
		private void SeekToClock(double clock)
		{
			if (ViewWindow == null)
				return;

			ViewWindow.AnimTime = Math.Max(clock, 0);
			ViewWindow.AnimControl(View3d.EAnimCommand.Step);
		}

		/// <summary>The native animation clock, without scrubbing preview state.</summary>
		private double NativeClock
		{
			get => ViewWindow?.AnimTime ?? 0.0;
		}

		/// <summary>The native animation frame, without scrubbing preview state.</summary>
		private double NativeFrame
		{
			get => NativeClock * FrameRate;
		}

		/// <inheritdoc/>
		public event PropertyChangedEventHandler? PropertyChanged;

		/// <summary>Notify subscribers that property 'prop_name' has changed.</summary>
		private void NotifyPropertyChanged(string prop_name)
		{
			PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(prop_name));
		}
	}
}
