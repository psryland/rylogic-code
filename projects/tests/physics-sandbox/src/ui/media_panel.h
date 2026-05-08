#pragma once
#include "src/forward.h"

namespace physics_sandbox
{
	// A bottom-docked panel with centred media controls for the physics simulation.
	// Contains Play/Pause/Reset buttons and a time-scale slider for slow-motion control.
	// Exposes Play, Pause, and Reset as event handlers that the parent wires up.
	struct MediaPanel : Panel
	{
		Button m_btn_play;
		Button m_btn_pause;
		Button m_btn_step;
		Button m_btn_reset;
		Button m_chk_allow_sleeping;
		Label  m_lbl_speed;  // Shows current time scale, e.g. "Speed: 1.00x"
		HWND   m_slider;     // Win32 trackbar for time scale (no wingui wrapper exists)

		// Slider range: 1..200, mapping to 0.01x..2.00x (position / 100).
		// Default position 100 = real-time (1.00x).
		static constexpr int SliderMin = 1;
		static constexpr int SliderMax = 200;
		static constexpr int SliderDefault = 100;

		// Events fired when buttons are clicked
		pr::EventHandler<MediaPanel&, pr::gui::EmptyArgs const&> OnPlay;
		pr::EventHandler<MediaPanel&, pr::gui::EmptyArgs const&> OnPause;
		pr::EventHandler<MediaPanel&, pr::gui::EmptyArgs const&> OnStep;
		pr::EventHandler<MediaPanel&, pr::gui::EmptyArgs const&> OnReset;
		pr::EventHandler<MediaPanel&, pr::gui::EmptyArgs const&> OnAllowSleepingChanged;

		MediaPanel(Panel::Params<> p = Panel::Params<>());
		~MediaPanel();

		// Read/set the current time scale from the slider.
		// Range [0.01, 2.0], where 1.0 = real-time.
		float TimeScale() const;
		void TimeScale(float scale);

		// Get/set whether the engine is allowed to put bodies to sleep.
		bool AllowSleeping() const;
		void AllowSleeping(bool allow_sleeping);

		// Update the speed label to reflect the current slider position.
		void UpdateSpeedLabel();
	};
}
