#include "src/forward.h"
#include "src/ui/details_panel.h"

namespace physics_sandbox
{
	DetailsPanel::DetailsPanel(Panel::Params<> p)
		: Panel(p.dock(EDock::Right).wh(250, Fill).padding(4).visible(false))
		, m_btn_pin(Button::Params<>()
			.parent(this_)
			.text(L"\U0001F4CC") // 📌 pushpin emoji
			.xy(0, 0)
			.wh(28, 22)
			.style('+', BS_FLAT))
		, m_text(TextBox::Params<>()
			.parent(this_)
			.dock(EDock::Fill)
			.style('+', ES_MULTILINE | ES_READONLY | WS_VSCROLL)
			.text(L"(no scene loaded)"))
		, m_pinned(false)
	{
		// Use a fixed-width font for alignment
		auto font = ::CreateFontW(-12, 0, 0, 0, FW_NORMAL, 0, 0, 0,
			DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS,
			CLEARTYPE_QUALITY, FIXED_PITCH | FF_MODERN, L"Consolas");
		::SendMessageW(m_text, WM_SETFONT, reinterpret_cast<WPARAM>(font), TRUE);

		// Position the pin button in the top-right corner of the panel
		WindowPosChange += [&](Control&, WindowPosEventArgs const& args)
		{
			if (args.m_before) return;
			auto cx = args.m_wp->cx;
			::SetWindowPos(m_btn_pin, HWND_TOP, cx - 32, 2, 0, 0, SWP_NOSIZE | SWP_NOZORDER);
		};

		// Toggle pin state when the button is clicked
		m_btn_pin.Click += [&](Button&, pr::gui::EmptyArgs const&)
		{
			TogglePin();
		};
	}

	void DetailsPanel::TogglePin()
	{
		m_pinned = !m_pinned;
		InvalidateValues();
		Visible(m_pinned);

		// Update button text to reflect state
		m_btn_pin.Text(m_pinned ? L"\U0001F4CC" : L"\U0001F4CB"); // 📌 pinned, 📋 unpinned

		// Trigger parent re-layout so the 3D viewport fills the freed/reclaimed space.
		// Sending WM_SIZE with the current size causes the docking layout to recalculate.
		auto parent = Parent();
		if (parent.ctrl() != nullptr)
		{
			RECT rc;
			::GetClientRect(*parent.ctrl(), &rc);
			::SendMessageW(*parent.ctrl(), WM_SIZE, SIZE_RESTORED, MAKELPARAM(rc.right, rc.bottom));
		}
	}

	#if PR_UNITTESTS
	namespace physics_sandbox::tests
	{
		// Exercise real controls under an invisible parent so refresh and visibility checks need no interactive window.
		PRUnitTestClass(DetailsPanelTests)
		{
			// Hidden and unchanged panels must not format; requested visible refreshes preserve the user's scroll position.
			PRUnitTestMethod(DormantUntilShownAndInvalidated, Quick)
			{
				// Keep window state local to this test and include enough bodies to exercise a scrolling details document.
				auto parent = Form(Form::Params<>().wh(640, 480));
				auto panel = DetailsPanel(DetailsPanel::Params<>().parent(&parent));
				parent.CreateHandle();
				auto shape = collision::ShapeBox(v4(1, 1, 1, 0));
				auto scene = Scene(nullptr);
				for (auto index = 0; index != 1000; ++index)
					scene.m_body.emplace_back(nullptr, &shape.m_base, m4x4::Identity(), physics::Inertia::Box(v4(0.5f, 0.5f, 0.5f, 0), 1));

				// Hidden updates retain the pending request without touching the formatted output.
				PR_EXPECT(!panel.m_pinned && !panel.Visible() && !panel.NeedsUpdate());
				panel.Update(scene);
				PR_EXPECT(panel.m_last_text.empty() && panel.m_refresh_needed);
				auto const style = panel.cp().m_style & ~WS_VISIBLE;
				panel.TogglePin();
				PR_EXPECT(panel.Visible() && panel.NeedsUpdate());
				panel.Update(scene);
				PR_EXPECT(!panel.NeedsUpdate() && panel.m_last_text.find(L"--- Body 999 ---") != std::wstring::npos);
				auto const text = panel.m_last_text;

				// Mutation without caller invalidation must not trigger formatting on a later render.
				scene.m_clock = 1;
				panel.Update(scene);
				PR_EXPECT(panel.m_last_text == text);
				::SendMessageW(panel.m_text, EM_LINESCROLL, 0, 5);
				auto const first_line = ::SendMessageW(panel.m_text, EM_GETFIRSTVISIBLELINE, 0, 0);
				PR_EXPECT(first_line > 0);
				panel.InvalidateValues();
				panel.Update(scene);
				PR_EXPECT(panel.m_last_text.find(L"Time: 1.000s") != std::wstring::npos);
				PR_EXPECT(::SendMessageW(panel.m_text, EM_GETFIRSTVISIBLELINE, 0, 0) == first_line);

				// Hide/show must preserve non-visibility style bits and refresh data changed while dormant.
				panel.TogglePin();
				PR_EXPECT(!panel.Visible() && !AllSet(panel.cp().m_style, WS_VISIBLE) && (panel.cp().m_style & ~WS_VISIBLE) == style);
				auto const hidden_text = panel.m_last_text;
				scene.m_clock = 2;
				panel.InvalidateValues();
				panel.Update(scene);
				PR_EXPECT(panel.m_last_text == hidden_text && !panel.NeedsUpdate());
				panel.TogglePin();
				panel.Update(scene);
				PR_EXPECT(panel.m_last_text.find(L"Time: 2.000s") != std::wstring::npos && !panel.NeedsUpdate());
				PR_EXPECT((panel.cp().m_style & ~WS_VISIBLE) == style);
			}
		};
	}
	#endif

	// Update the displayed properties from the current scene state.
	// Only updates when the panel is pinned (visible) to avoid wasted work.
	// The caller invalidates values at scene-change boundaries rather than polling their formatted text.
	void DetailsPanel::Update(Scene const& scene)
	{
		// Neither hidden panels nor unchanged paused scenes need formatting.
		if (!NeedsUpdate())
			return;

		std::wstringstream ss;
		ss << L"Scenario: " << pr::Widen(ScenarioName(scene.m_current_scenario)) << L"\r\n";
		ss << L"Time: " << std::fixed << std::setprecision(3) << scene.m_clock << L"s\r\n";
		ss << L"Collisions: " << scene.m_diag.count << L"\r\n";
		ss << L"\r\n";

		for (int i = 0; i != std::ssize(scene.m_body); ++i)
		{
			auto const& body = scene.m_body[i];
			auto vel = body.VelocityWS();
			auto pos = body.O2W().pos;

			ss << L"--- Body " << i << L" ---\r\n";
			ss << L"  Pos:   " << std::fixed << std::setprecision(2) << pos.x << L", " << pos.y << L", " << pos.z << L"\r\n";
			ss << L"  State: " << (body.Sleeping() ? L"Sleeping" : L"Awake") << L"\r\n";
			ss << L"  Vel:   " << vel.lin.x << L", " << vel.lin.y << L", " << vel.lin.z << L"\r\n";
			ss << L"  AngV:  " << vel.ang.x << L", " << vel.ang.y << L", " << vel.ang.z << L"\r\n";
			ss << L"  Mass:  " << std::setprecision(1) << body.Mass() << L"\r\n";
			ss << L"  KE:    " << std::setprecision(4) << body.KineticEnergy() << L"\r\n";
			ss << L"\r\n";
		}

		// Only update the text control when the content has changed to prevent
		// WM_SETTEXT → WM_PAINT flicker cycles every frame.
		auto new_text = ss.str();
		if (new_text != m_last_text)
		{
			// Preserve the scroll position across text updates so the user's
			// scroll context isn't lost when the text refreshes.
			auto first_visible = static_cast<int>(::SendMessageW(m_text, EM_GETFIRSTVISIBLELINE, 0, 0));

			m_last_text = std::move(new_text);
			m_text.Text(m_last_text.c_str());

			// Restore scroll position — scroll back to the previously visible line
			auto new_first = static_cast<int>(::SendMessageW(m_text, EM_GETFIRSTVISIBLELINE, 0, 0));
			::SendMessageW(m_text, EM_LINESCROLL, 0, first_visible - new_first);
		}
		m_refresh_needed = false;
	}

	void DetailsPanel::InvalidateValues()
	{
		m_refresh_needed = true;
	}

	bool DetailsPanel::NeedsUpdate() const
	{
		return m_pinned && m_refresh_needed;
	}
}
