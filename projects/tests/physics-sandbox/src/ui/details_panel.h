#pragma once
#include "src/forward.h"
#include "src/scene/scene.h"

namespace physics_sandbox
{
	// A right-side panel that displays the properties of each rigid body in the scene.
	// Hidden by default; the View menu or viewport's D shortcut shows it. Scene changes request a refresh,
	// but formatting is deferred until the panel is visible and the simulation is paused.
	struct DetailsPanel : Panel
	{
		Button m_btn_pin;    // Pin/unpin toggle button
		TextBox m_text;
		std::wstring m_last_text;
		bool m_pinned;       // true = panel is visible and pinned open
		bool m_refresh_needed = true;

		DetailsPanel(Panel::Params<> p = Panel::Params<>());

		// Update the displayed properties from the current scene state.
		// Format only when visible and invalidated; unchanged output preserves the text control.
		void Update(Scene const& scene);

		// Request a refresh after displayed scene data changes, without inspecting the scene.
		void InvalidateValues();

		// Whether visible values need refreshing.
		bool NeedsUpdate() const;

		// Toggle the panel between pinned (visible) and hidden states
		void TogglePin();
	};
}
