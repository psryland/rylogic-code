using System;
using System.ComponentModel;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using Rylogic.Gfx;
using Rylogic.Utility;

namespace Rylogic.Gui.WPF
{
	public partial class View3dAnimControls :Grid, INotifyPropertyChanged
	{
		public View3dAnimControls()
		{
			InitializeComponent();
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
			NotifyPropertyChanged(nameof(ViewWindow));

			// Handlers
			void HandleAnimationEvent(object? sender, View3d.AnimationEventArgs e)
			{
				Util.BreakIf(!Util.IsMainThread);
				NotifyPropertyChanged(nameof(Animating));
				NotifyPropertyChanged(nameof(AnimClock));
				NotifyPropertyChanged(nameof(Frame));
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
			get => ViewWindow?.AnimTime ?? 0.0;
			set
			{
				if (AnimClock == value || ViewWindow == null) return;
				ViewWindow.AnimTime = Math.Max(value, 0);
				ViewWindow.AnimControl(View3d.EAnimCommand.Step);
				NotifyPropertyChanged(nameof(AnimClock));
				NotifyPropertyChanged(nameof(Frame));
			}
		}

		/// <summary>The size of each step per second. Set to 0 for real time</summary>
		public double StepSize
		{
			get;
			set
			{
				if (StepSize == value) return;
				field = value;
				if (Animating) Play.Execute();
				NotifyPropertyChanged(nameof(StepSize));
			}
		} = 0;

		/// <summary>The assumed frame rate of all animation</summary>
		public double FrameRate
		{
			get;
			set
			{
				if (FrameRate == value) return;
				field = value;
				NotifyPropertyChanged(nameof(FrameRate));
				NotifyPropertyChanged(nameof(Frame));
			}
		} = 24.0;

		/// <summary>True if the animation is running</summary>
		public bool Animating => ViewWindow?.Animating ?? false;

		/// <summary>If true, animations play relative to their start frame</summary>
		public bool RelativeTime
		{
			get;
			set
			{
				if (RelativeTime == value) return;
				field = value;
				NotifyPropertyChanged(nameof(RelativeTime));
			}
		}

		/// <summary>The start frame</summary>
		public int Frame0
		{
			get;
			set
			{
				if (Frame0 == value) return;
				field = value;
				NotifyPropertyChanged(nameof(Frame0));
			}
		} = 0;

		/// <summary>The current frame</summary>
		public double Frame
		{
			get => AnimClock * FrameRate;
			set
			{
				if (Frame == value) return;
				AnimClock = value / FrameRate;
			}
		}

		/// <summary>The end frame</summary>
		public int FrameN
		{
			get;
			set
			{
				if (FrameN == value) return;
				field = value;
				NotifyPropertyChanged(nameof(FrameN));
			}
		} = 100;

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

		/// <inheritdoc/>
		public event PropertyChangedEventHandler? PropertyChanged;
		private void NotifyPropertyChanged(string prop_name) => PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(prop_name));
	}
}
