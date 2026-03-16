#pragma once
#include "src/forward.h"
#include "src/scene/scene.h"

namespace physics_sandbox
{
	// A right-side panel that displays the properties of each rigid body in the scene.
	// The panel can be pinned open or hidden. When hidden, a small tab appears on the
	// right edge that can be clicked to show the panel. Only updates when visible.
	struct DetailsPanel : Panel
	{
		Button m_btn_pin;    // Pin/unpin toggle button
		TextBox m_text;
		std::wstring m_last_text;
		bool m_pinned;       // true = panel is visible and pinned open

		DetailsPanel(Panel::Params<> p = Panel::Params<>());

		// Update the displayed properties from the current scene state.
		// Only updates the text control when the panel is visible and pinned.
		void Update(Scene const& scene);

		// Toggle the panel between pinned (visible) and hidden states
		void TogglePin();
	};
}
