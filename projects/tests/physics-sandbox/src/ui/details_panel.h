#pragma once
#include "src/forward.h"
#include "src/scene/scene.h"

namespace physics_sandbox
{
	// A right-side panel that displays the properties of each rigid body in the scene.
	// Currently a stub that shows name, position, velocity, mass, and KE in a static text control.
	// Future: tree view, editable properties, selection sync with 3D viewport.
	struct DetailsPanel : Panel
	{
		TextBox m_text;
		std::wstring m_last_text;

		DetailsPanel(Panel::Params<> p = Panel::Params<>())
			: Panel(p.dock(EDock::Right).wh(250, Fill).padding(4))
			, m_text(TextBox::Params<>()
				.parent(this_)
				.dock(EDock::Fill)
				.style('+', ES_MULTILINE | ES_READONLY | WS_VSCROLL)
				.text(L"(no scene loaded)"))
		{
			// Use a fixed-width font for alignment
			auto font = ::CreateFontW(-12, 0, 0, 0, FW_NORMAL, 0, 0, 0,
				DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS,
				CLEARTYPE_QUALITY, FIXED_PITCH | FF_MODERN, L"Consolas");
			::SendMessageW(m_text, WM_SETFONT, reinterpret_cast<WPARAM>(font), TRUE);
		}

		// Update the displayed properties from the current scene state.
		// Called once per render frame. Only updates the text control when the
		// content has actually changed to avoid flicker from constant WM_SETTEXT repaints.
		void Update(Scene const& scene)
		{
			std::wstringstream ss;
			ss << L"Scenario: " << pr::Widen(ScenarioName(scene.m_scenario)) << L"\r\n";
			ss << L"Time: " << std::fixed << std::setprecision(3) << scene.m_clock << L"s\r\n";
			ss << L"Collisions: " << scene.m_diag.count << L"\r\n";
			ss << L"\r\n";

			for (int i = 0; i != scene.m_body_count; ++i)
			{
				auto const& body = scene.m_body[i];
				auto vel = body.VelocityWS();
				auto pos = body.O2W().pos;

				ss << L"--- Body " << i << L" ---\r\n";
				ss << L"  Pos:   " << std::fixed << std::setprecision(2)
					<< pos.x << L", " << pos.y << L", " << pos.z << L"\r\n";
				ss << L"  Vel:   "
					<< vel.lin.x << L", " << vel.lin.y << L", " << vel.lin.z << L"\r\n";
				ss << L"  AngV:  "
					<< vel.ang.x << L", " << vel.ang.y << L", " << vel.ang.z << L"\r\n";
				ss << L"  Mass:  " << std::setprecision(1) << body.Mass() << L"\r\n";
				ss << L"  KE:    " << std::setprecision(4) << body.KineticEnergy() << L"\r\n";
				ss << L"\r\n";
			}

			// Only update the text control when the content has changed to prevent
			// WM_SETTEXT → WM_PAINT flicker cycles every frame.
			auto new_text = ss.str();
			if (new_text != m_last_text)
			{
				m_last_text = std::move(new_text);
				m_text.Text(m_last_text.c_str());
			}
		}
	};
}
