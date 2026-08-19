using System;
using System.ComponentModel;
using System.Windows;
using Rylogic.Common;
using Rylogic.Gfx;
using Rylogic.Gui.WPF;

namespace Fishomatic
{
	public class Settings :SettingsBase<Settings>
	{
		public Settings()
		{
			TargetWindowName = "World of Warcraft";
			AlwaysOnTop = true;
			MoveCursor = true;
			AutoThreshold = true;
			AutoThresholdMargin = 3.0;
			CastKey = '7';
			MoveThreshold = 9.0;
			SettlingTimeS = 2.5;
			SearchArea = new Thickness(8);
			SmallSearchSize = new Size(50, 50);
			TargetColour = 0xFF962C1E;
			ColourTolerence = 50;
			ClickDelayS = 0.25;
			AfterCastWaitS = 3.0;
			AfterCatchWaitS = 3.0;
			MaxFishCycleS = 17.0;
			AbortTimeS = 8.0;
			BaublesKey = '9';
			FishingPoleKey = '0';
			BaublesTimeMins = 11.0;
			BaublesApplyWaitS = 6.0;

			AutoSaveOnChanges = true;
		}
		public Settings(string filepath)
			: base(filepath)
		{
			AutoSaveOnChanges = true;
		}

		/// <summary></summary>
		[Description("The window title to search for")]
		public string TargetWindowName
		{
			get => get<string>(nameof(TargetWindowName));
			set => set(nameof(TargetWindowName), value);
		}

		/// <summary></summary>
		[Description("Fishomatic window always on top")]
		public bool AlwaysOnTop
		{
			get => get<bool>(nameof(AlwaysOnTop));
			set => set(nameof(AlwaysOnTop), value);
		}

		/// <summary></summary>
		[Description("Move the cursor to the click position")]
		public bool MoveCursor
		{
			get => get<bool>(nameof(MoveCursor));
			set => set(nameof(MoveCursor), value);
		}

		/// <summary></summary>
		[Description("The key to press to cast")]
		public char CastKey
		{
			get => get<char>(nameof(CastKey));
			set => set(nameof(CastKey), value);
		}

		/// <summary></summary>
		[Description("True if the threshold is set automatically after the settling time")]
		public bool AutoThreshold
		{
			get => get<bool>(nameof(AutoThreshold));
			set => set(nameof(AutoThreshold), value);
		}

		/// <summary></summary>
		[Description("The margin above the settled movement level that triggers a catch")]
		public double AutoThresholdMargin
		{
			get => get<double>(nameof(AutoThresholdMargin));
			set => set(nameof(AutoThresholdMargin), value);
		}

		/// <summary></summary>
		[Description("How far the bobber moves before clicking to catch fish")]
		public double MoveThreshold
		{
			get => get<double>(nameof(MoveThreshold));
			set => set(nameof(MoveThreshold), value);
		}

		/// <summary></summary>
		[Description("The time to wait before determining the move threshold")]
		public double SettlingTimeS
		{
			get => get<double>(nameof(SettlingTimeS));
			set => set(nameof(SettlingTimeS), value);
		}

		/// <summary></summary>
		[Description("The padding, relative to the target window, in which to search for the bobber")]
		public Thickness SearchArea
		{
			get => get<Thickness>(nameof(SearchArea));
			set => set(nameof(SearchArea), value);
		}

		/// <summary></summary>
		[Description("The size of the area to search while tracking the bobber")]
		public Size SmallSearchSize
		{
			get => get<Size>(nameof(SmallSearchSize));
			set => set(nameof(SmallSearchSize), value);
		}

		/// <summary></summary>
		[Description("The colour to search for")]
		public Colour32 TargetColour
		{
			get => get<Colour32>(nameof(TargetColour));
			set => set(nameof(TargetColour), value);
		}

		/// <summary></summary>
		[Description("Tolerance when searching for the target colour")]
		public int ColourTolerence
		{
			get => get<int>(nameof(ColourTolerence));
			set => set(nameof(ColourTolerence), value);
		}

		/// <summary></summary>
		[Description("Length of time to wait between detecting the bobber's moved and clicking (s)")]
		public double ClickDelayS
		{
			get => get<double>(nameof(ClickDelayS));
			set => set(nameof(ClickDelayS), value);
		}

		/// <summary></summary>
		[Description("Length of time to wait after casting before looking for the bobber (ms)")]
		public double AfterCastWaitS
		{
			get => get<double>(nameof(AfterCastWaitS));
			set => set(nameof(AfterCastWaitS), value);
		}

		/// <summary></summary>
		[Description("Length of time to wait after catch a fish before casting again")]
		public double AfterCatchWaitS
		{
			get => get<double>(nameof(AfterCatchWaitS));
			set => set(nameof(AfterCatchWaitS), value);
		}

		/// <summary></summary>
		[Description("The maximum length of time the fishing process can take")]
		public double MaxFishCycleS
		{
			get => get<double>(nameof(MaxFishCycleS));
			set => set(nameof(MaxFishCycleS), value);
		}

		/// <summary></summary>
		[Description("The length of time to wait before deciding the bobber can't be found")]
		public double AbortTimeS
		{
			get => get<double>(nameof(AbortTimeS));
			set => set(nameof(AbortTimeS), value);
		}

		/// <summary></summary>
		[Description("The key to press to apply baubles to a fishing pole")]
		public char BaublesKey
		{
			get => get<char>(nameof(BaublesKey));
			set => set(nameof(BaublesKey), value);
		}

		/// <summary></summary>
		[Description("The key to press to select your fishing pole when applying baubles")]
		public char FishingPoleKey
		{
			get => get<char>(nameof(FishingPoleKey));
			set => set(nameof(FishingPoleKey), value);
		}

		/// <summary></summary>
		[Description("The time to wait (in minutes) between reapplication of baubles")]
		public double BaublesTimeMins
		{
			get => get<double>(nameof(BaublesTimeMins));
			set => set(nameof(BaublesTimeMins), value);
		}

		/// <summary></summary>
		[Description("The time to wait while baubles are being applied")]
		public double BaublesApplyWaitS
		{
			get => get<double>(nameof(BaublesApplyWaitS));
			set => set(nameof(BaublesApplyWaitS), value);
		}

		/// <summary>A rect for the small search area, centred on 'position'</summary>
		public Rect SmallSearchArea(Point position)
		{
			return new Rect(
				position.X - SmallSearchSize.Width / 2,
				position.Y - SmallSearchSize.Height / 2,
				SmallSearchSize.Width,
				SmallSearchSize.Height);
		}

		/// <summary>A rect for the large search area, relative to 'window_rect'</summary>
		public Rect LargeSearchArea(Rect window_rect)
		{
			return window_rect.ShrinkBy(SearchArea);
		}
	}
}
