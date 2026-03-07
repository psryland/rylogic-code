#pragma once
#include "src/forward.h"

namespace physics_sandbox
{
	// A bottom-docked panel with centred media controls for the physics simulation.
	// Exposes Play, Pause, and Reset as event handlers that the parent wires up.
	struct MediaPanel : Panel
	{
		Button m_btn_play;
		Button m_btn_pause;
		Button m_btn_reset;

		// Events fired when buttons are clicked
		pr::EventHandler<MediaPanel&, pr::gui::EmptyArgs const&> OnPlay;
		pr::EventHandler<MediaPanel&, pr::gui::EmptyArgs const&> OnPause;
		pr::EventHandler<MediaPanel&, pr::gui::EmptyArgs const&> OnReset;

		MediaPanel(Panel::Params<> p = Panel::Params<>())
			: Panel(p.dock(EDock::Bottom).wh(Fill, 36).padding(4))
			, m_btn_play(Button::Params<>().parent(this_).text(L"\u25B6 Play").xy(0, 4).wh(80, 26))
			, m_btn_pause(Button::Params<>().parent(this_).text(L"\u23F8 Pause").xy(84, 4).wh(80, 26))
			, m_btn_reset(Button::Params<>().parent(this_).text(L"\u23EE Reset").xy(168, 4).wh(80, 26))
		{
			// Centre the buttons when the panel is resized
			WindowPosChange += [&](Control&, WindowPosEventArgs const& args)
			{
				if (args.m_before) return;
				auto cx = args.m_wp->cx;
				auto total_width = 80 + 4 + 80 + 4 + 80; // 3 buttons + gaps
				auto left = (cx - total_width) / 2;

				// Reposition buttons to stay centred
				::SetWindowPos(m_btn_play, nullptr, left, 4, 0, 0, SWP_NOSIZE | SWP_NOZORDER);
				::SetWindowPos(m_btn_pause, nullptr, left + 84, 4, 0, 0, SWP_NOSIZE | SWP_NOZORDER);
				::SetWindowPos(m_btn_reset, nullptr, left + 168, 4, 0, 0, SWP_NOSIZE | SWP_NOZORDER);
			};

			// Wire button clicks to events
			m_btn_play.Click += [&](Button&, pr::gui::EmptyArgs const&)
			{
				OnPlay(*this, pr::gui::EmptyArgs{});
			};
			m_btn_pause.Click += [&](Button&, pr::gui::EmptyArgs const&)
			{
				OnPause(*this, pr::gui::EmptyArgs{});
			};
			m_btn_reset.Click += [&](Button&, pr::gui::EmptyArgs const&)
			{
				OnReset(*this, pr::gui::EmptyArgs{});
			};
		}
	};
}