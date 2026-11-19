#include "src/forward.h"
#include "src/ui/media_panel.h"
#if PR_UNITTESTS
#include "src/ui/details_panel.h"
#endif

namespace physics_sandbox
{
	MediaPanel::MediaPanel(Panel::Params<> p)
		: Panel(p.dock(EDock::Bottom).wh(Fill, 36).padding(4))
		, m_btn_play(Button::Params<>().parent(this_).text(L"\u25B6 Play").xy(0, 4).wh(80, 26))
		, m_btn_pause(Button::Params<>().parent(this_).text(L"\u23F8 Pause").xy(84, 4).wh(80, 26))
		, m_btn_step(Button::Params<>().parent(this_).text(L"\u23ED Step").xy(168, 4).wh(80, 26))
		, m_btn_reset(Button::Params<>().parent(this_).text(L"\u23EE Reset").xy(252, 4).wh(80, 26))
		, m_chk_allow_sleeping(Button::Params<>().parent(this_).text(L"Allow Sleeping").xy(560, 4).wh(130, 26).chk_box())
		, m_lbl_speed(Label::Params<>().parent(this_).text(L"Speed: 1.00x").xy(280, 4).wh(100, 26).style('+', SS_CENTERIMAGE))
		, m_slider(nullptr)
		, m_allow_sleeping(true)
		, m_window_pos_change_id()
	{
		// Ensure the trackbar common control class is registered.
		// The default InitCtrls() only loads ICC_STANDARD_CLASSES which doesn't
		// include the trackbar. ICC_BAR_CLASSES covers toolbar, statusbar, trackbar.
		INITCOMMONCONTROLSEX icc = { sizeof(icc), ICC_BAR_CLASSES };
		::InitCommonControlsEx(&icc);

		// Wire button clicks to events
		m_btn_play.Click += [&](Button&, pr::gui::EmptyArgs const&)
		{
			OnPlay(*this, pr::gui::EmptyArgs{});
		};
		m_btn_pause.Click += [&](Button&, pr::gui::EmptyArgs const&)
		{
			OnPause(*this, pr::gui::EmptyArgs{});
		};
		m_btn_step.Click += [&](Button&, pr::gui::EmptyArgs const&)
		{
			OnStep(*this, pr::gui::EmptyArgs{});
		};
		m_btn_reset.Click += [&](Button&, pr::gui::EmptyArgs const&)
		{
			OnReset(*this, pr::gui::EmptyArgs{});
		};
		m_chk_allow_sleeping.CheckedChanged += [&](Button&, pr::gui::EmptyArgs const&)
		{
			m_allow_sleeping = m_chk_allow_sleeping.Checked();
			OnAllowSleepingChanged(*this, pr::gui::EmptyArgs{});
		};

		// Centre the button group and slider group when the panel is resized.
		// Layout: [Play][Pause][Step][Reset] --- gap --- [Speed: 1.00x][====slider====] --- gap --- [Allow Sleeping]
		// All controls share the same vertical centre (y=4, h=26 → midpoint at 17px).
		// The trackbar is created lazily here because wingui defers HWND creation —
		// m_hwnd is null during the constructor body, so CreateWindowEx would fail.
		// Attach to the base last, so no later throwing constructor work can leave a callback referring to destroyed members.
		m_window_pos_change_id = WindowPosChange += [&](Control&, WindowPosEventArgs const& args)
		{
			if (args.m_before) return;

			// Create the trackbar on first resize (when our HWND is valid)
			if (m_slider == nullptr && m_hwnd != nullptr)
			{
				m_slider = ::CreateWindowExW(
					0, TRACKBAR_CLASSW, L"",
					WS_CHILD | WS_VISIBLE | TBS_HORZ | TBS_NOTICKS,
					390, 4, 150, 26,
					m_hwnd, nullptr, nullptr, nullptr);
				::SendMessage(m_slider, TBM_SETRANGE, TRUE, MAKELPARAM(SliderMin, SliderMax));
				::SendMessage(m_slider, TBM_SETPOS, TRUE, SliderDefault);
			}
			if (m_chk_allow_sleeping != nullptr && m_chk_allow_sleeping.Checked() != m_allow_sleeping)
				m_chk_allow_sleeping.Checked(m_allow_sleeping);

			auto cx = args.m_wp->cx;

			// Button group: 4 buttons × 80px with 4px gaps between them
			auto btn_group_w = 80 + 4 + 80 + 4 + 80 + 4 + 80; // = 332
			// Slider group: speed label (100px) + gap (8px) + slider (150px)
			auto slider_group_w = 100 + 8 + 150; // = 258
			// Sleep group: checkbox label and box.
			auto sleep_group_w = 130;
			// Total width with gaps between groups
			auto total_w = btn_group_w + 24 + slider_group_w + 16 + sleep_group_w;
			auto left = (cx - total_w) / 2;

			// All controls vertically centred at y=4, h=26
			::SetWindowPos(m_btn_play, nullptr, left, 4, 0, 0, SWP_NOSIZE | SWP_NOZORDER);
			::SetWindowPos(m_btn_pause, nullptr, left + 84, 4, 0, 0, SWP_NOSIZE | SWP_NOZORDER);
			::SetWindowPos(m_btn_step, nullptr, left + 168, 4, 0, 0, SWP_NOSIZE | SWP_NOZORDER);
			::SetWindowPos(m_btn_reset, nullptr, left + 252, 4, 0, 0, SWP_NOSIZE | SWP_NOZORDER);

			// Speed label and slider to the right of the buttons
			auto slider_left = left + btn_group_w + 24;
			::SetWindowPos(m_lbl_speed, nullptr, slider_left, 4, 0, 0, SWP_NOSIZE | SWP_NOZORDER);
			if (m_slider)
				::SetWindowPos(m_slider, nullptr, slider_left + 108, 4, 0, 0, SWP_NOSIZE | SWP_NOZORDER);

			::SetWindowPos(m_chk_allow_sleeping, nullptr, slider_left + slider_group_w + 16, 4, 0, 0, SWP_NOSIZE | SWP_NOZORDER);
		};
	}

	MediaPanel::~MediaPanel()
	{
		// Base Control teardown reparents the HWND after these button members have been destroyed.
		WindowPosChange -= m_window_pos_change_id;

		// The native trackbar is not owned by a wingui member.
		if (m_slider)
			::DestroyWindow(m_slider);
	}

	// Read the current time scale from the slider.
	// Returns a value in the range [0.01, 2.0], where 1.0 = real-time.
	float MediaPanel::TimeScale() const
	{
		if (!m_slider)
			return 1.0f;

		auto pos = static_cast<int>(::SendMessage(m_slider, TBM_GETPOS, 0, 0));
		return pos / 100.0f;
	}

	// Set the slider position programmatically (e.g. for keyboard shortcuts).
	void MediaPanel::TimeScale(float scale)
	{
		if (!m_slider)
			return;

		auto pos = Clamp(static_cast<int>(scale * 100.0f), SliderMin, SliderMax);
		::SendMessage(m_slider, TBM_SETPOS, TRUE, pos);
	}

	bool MediaPanel::AllowSleeping() const
	{
		return m_chk_allow_sleeping != nullptr ? m_chk_allow_sleeping.Checked() : m_allow_sleeping;
	}

	void MediaPanel::AllowSleeping(bool allow_sleeping)
	{
		m_allow_sleeping = allow_sleeping;
		if (m_chk_allow_sleeping != nullptr)
			m_chk_allow_sleeping.Checked(allow_sleeping);
	}

	// Update the speed label to reflect the current slider position.
	void MediaPanel::UpdateSpeedLabel()
	{
		auto scale = TimeScale();
		wchar_t buf[32];
		swprintf_s(buf, L"Speed: %.2fx", scale);
		m_lbl_speed.Text(buf);
	}

	#if PR_UNITTESTS
	namespace tests
	{
		// Windowed buffers use client dimensions without requiring a physical output or fullscreen display mode.
		PRUnitTestClass(WindowedStartupTests, Serial)
		{
			// Default and explicit dimensions must work with an adapter description containing no outputs.
			PRUnitTestMethod(NoOutputDefaultAndExplicitSize, Quick)
			{
				// No adapter enumeration or GPU is needed to describe windowed buffers.
				auto renderer_settings = rdr12::RdrSettings(GetModuleHandle(nullptr));
				auto settings = rdr12::WndSettings(nullptr, true, renderer_settings);
				PR_EXPECT(settings.m_output.ptr == nullptr);
				PR_EXPECT(settings.m_mode.Width == 1024 && settings.m_mode.Height == 768);
				PR_EXPECT(settings.m_mode.Format == DXGI_FORMAT_B8G8R8A8_UNORM);
				PR_EXPECT(settings.m_mode.RefreshRate.Numerator == 0 && settings.m_mode.RefreshRate.Denominator == 1);

				// Explicit dimensions are not rounded to an enumerated monitor mode, including DXGI's zero-size request.
				settings.Size(iv2(317, 191));
				PR_EXPECT(settings.m_mode.Width == 317 && settings.m_mode.Height == 191);
				settings.Size(619, 283);
				PR_EXPECT(settings.m_mode.Width == 619 && settings.m_mode.Height == 283);
				settings.Size(0, 0);
				PR_EXPECT(settings.m_mode.Width == 0 && settings.m_mode.Height == 0);
				settings.DefaultMode();
				PR_EXPECT(settings.m_mode.Width == 1024 && settings.m_mode.Height == 768);
			}

			// A real hidden HWND must supply its client size both during construction and on a later size request.
			PRUnitTestMethod(NoOutputUsesHwndClientSize, Quick)
			{
				// Keep the window invisible; use the existing Size(HWND) DPI convention as the independent size oracle.
				auto parent = Form(Form::Params<>().text(L"[Articulation challenger] Client size regression").wh(419, 271).visible(false));
				auto hwnd = HWND(parent.CreateHandle());
				auto renderer_settings = rdr12::RdrSettings(GetModuleHandle(nullptr));
				auto settings = rdr12::WndSettings(hwnd, true, renderer_settings);
				RECT client = {};
				Check(GetClientRect(hwnd, &client));
				auto dpi = GetDpiForWindow(hwnd);
				PR_EXPECT(settings.m_output.ptr == nullptr);
				PR_EXPECT(settings.m_mode.Width == (client.right - client.left) * dpi / 96);
				PR_EXPECT(settings.m_mode.Height == (client.bottom - client.top) * dpi / 96);

				// Resizing the owned HWND must replace the dimensions without consulting an output.
				Check(SetWindowPos(hwnd, nullptr, 0, 0, 527, 349, SWP_NOMOVE | SWP_NOZORDER | SWP_NOACTIVATE));
				Check(GetClientRect(hwnd, &client));
				settings.Size(hwnd);
				PR_EXPECT(settings.m_mode.Width == (client.right - client.left) * dpi / 96);
				PR_EXPECT(settings.m_mode.Height == (client.bottom - client.top) * dpi / 96);
			}

			// Automatic fullscreen mode selection requires an output and leaves the previous mode unchanged on failure.
			PRUnitTestMethod(FullscreenRequiresOutput, Quick)
			{
				// The constructor must reject missing output explicitly, before querying monitor details.
				auto renderer_settings = rdr12::RdrSettings(GetModuleHandle(nullptr));
				auto rejected = false;
				try
				{
					// Request only the descriptor; never enter fullscreen or alter desktop state.
					auto settings = rdr12::WndSettings(nullptr, false, renderer_settings);
				}
				catch (std::runtime_error const& ex)
				{
					// Distinguish the owning contract error from an incidental failed monitor API.
					rejected = std::string_view(ex.what()).find("output") != std::string_view::npos;
				}
				PR_EXPECT(rejected);

				// Cover selection after the public settings have been changed to fullscreen as well.
				auto settings = rdr12::WndSettings(nullptr, true, renderer_settings);
				auto previous = settings.m_mode;
				settings.m_windowed = FALSE;
				PR_THROWS(settings.DefaultMode(), std::runtime_error);
				PR_THROWS(settings.Size(640, 480), std::runtime_error);
				PR_EXPECT(memcmp(&settings.m_mode, &previous, sizeof(previous)) == 0);
			}

			// Window construction must accept windowed VSync without inventing a monitor refresh rate.
			PRUnitTestMethod(NoOutputWindowedVsyncSwapChain, Quick)
			{
				// Use the same default renderer selection as View3DPanelStatic, with a hidden, session-tagged owned HWND.
				auto parent = Form(Form::Params<>().text(L"[Articulation challenger] Swap chain regression").wh(160, 120).visible(false));
				auto hwnd = HWND(parent.CreateHandle());
				auto renderer_settings = rdr12::RdrSettings(GetModuleHandle(nullptr));
				auto settings = rdr12::WndSettings(hwnd, true, renderer_settings);
				settings.m_vsync = 1;
				PR_EXPECT(settings.m_output.ptr == nullptr && settings.m_mode.RefreshRate.Numerator == 0);

				// Exercise the actual linked Window implementation, not just an independently compiled settings header.
				auto renderer = rdr12::Renderer(renderer_settings);
				auto window = rdr12::Window(renderer, settings);
				auto desc = DXGI_SWAP_CHAIN_DESC1{};
				Check(window.m_swap_chain->GetDesc1(&desc));
				auto fullscreen = TRUE;
				Check(window.m_swap_chain->GetFullscreenState(&fullscreen, nullptr));
				PR_EXPECT(!fullscreen);
				PR_EXPECT(desc.Width == settings.m_mode.Width && desc.Height == settings.m_mode.Height);
				PR_EXPECT(desc.Format == DXGI_FORMAT_B8G8R8A8_UNORM);
			}
		};

		// Observe the real panel subscriptions at destruction without executing a stale delegate in a red control.
		PRUnitTestClass(PanelTeardownTests, Serial)
		{
			// A thrown owner constructor destroys its completed panel member, as in SandboxUI startup unwinding.
			struct OwnerConstructionFailed {};

			// Check both normal destruction and owner-constructor unwinding through actual synchronous Win32 callbacks.
			template <typename TPanel, typename Child>
			void CheckTeardown(bool unwind, Child child)
			{
				// External state outlives every panel and never reads a destroyed C++ member.
				auto parent = Form(Form::Params<>().text(L"[Articulation challenger] Panel teardown regression").xy(41, 53).wh(800, 480).visible(false));
				auto live_calls = 0;
				auto late_calls = 0;
				auto base_late_calls = 0;
				HWND child_hwnd = nullptr;
				auto observe = [&](TPanel& panel)
				{
					// Create all children before replacing the one owned layout delegate with a lifetime-observing wrapper.
					parent.CreateHandle();
					child_hwnd = child(panel);
					PR_EXPECT(IsWindow(child_hwnd) != FALSE);
					PR_EXPECT(panel.WindowPosChange.count() == 1);
					auto& handler = panel.WindowPosChange.m_handlers.front();
					auto original = std::move(handler.m_delegate);
					handler.m_delegate = [&, original = std::move(original)](Control& sender, WindowPosEventArgs const& args)
					{
						// Record a stale invocation but do not deliberately dereference destroyed members in the old-path process.
						if (!IsWindow(child_hwnd))
						{
							// This is a failing observation, not production recovery or a replacement for unsubscribe.
							++late_calls;
							return;
						}
						++live_calls;
						original(sender, args);
					};

					// This independent observer must survive: framework teardown events must not be disabled wholesale.
					panel.WindowPosChange += [&](Control&, WindowPosEventArgs const&)
					{
						// Query only the saved native handle after the derived control members have gone.
						if (!IsWindow(child_hwnd))
							++base_late_calls;
					};
					Check(SetWindowPos(panel, nullptr, 0, 0, 743, 61, SWP_NOMOVE | SWP_NOZORDER | SWP_NOACTIVATE));
				};

				// The panel is fully constructed before a simulated failure of its containing owner.
				struct Owner
				{
					TPanel m_panel;

					// Install observation while the members are live, then optionally unwind this constructor.
					Owner(Form& parent, decltype(observe)& setup, bool unwind)
						: m_panel(Panel::Params<>().parent(&parent).visible(false))
					{
						// Failure after setup exercises automatic member destruction, not an explicit test-only destructor call.
						setup(m_panel);
						if (unwind)
							throw OwnerConstructionFailed{};
					}
				};
				auto caught = false;
				try
				{
					// Destruction occurs here normally, or during construction unwinding before the catch.
					auto owner = Owner(parent, observe, unwind);
				}
				catch (OwnerConstructionFailed const&)
				{
					// Catch only the deliberate owner failure; unexpected GUI failures must fail the test.
					caught = true;
				}

				// Require useful live layout and real late base callbacks, but no invocation of the owned layout after child destruction.
				log_stream() << "unwind=" << unwind << " live=" << live_calls << " late_owned=" << late_calls << " late_base=" << base_late_calls << '\n';
				PR_EXPECT(caught == unwind);
				PR_EXPECT(live_calls > 0);
				PR_EXPECT(base_late_calls > 0);
				PR_EXPECT(late_calls == 0);
				PR_EXPECT(IsWindow(child_hwnd) == FALSE);
			}

			// Normal MediaPanel teardown must retire its own layout subscription.
			PRUnitTestMethod(MediaPanelNormalDestruction, Quick)
			{
				// The checkbox is the member reached by the captured crash stack.
				CheckTeardown<MediaPanel>(false, [](MediaPanel& panel) { return HWND(panel.m_chk_allow_sleeping); });
			}

			// An enclosing constructor failure must retire the same MediaPanel subscription.
			PRUnitTestMethod(MediaPanelConstructorUnwind, Quick)
			{
				// Match the completed-member destruction path observed during SandboxUI construction.
				CheckTeardown<MediaPanel>(true, [](MediaPanel& panel) { return HWND(panel.m_chk_allow_sleeping); });
			}

			// The directly related DetailsPanel layout subscription has the same ownership boundary.
			PRUnitTestMethod(DetailsPanelNormalDestruction, Quick)
			{
				// Its layout callback accesses the pin button rather than the MediaPanel checkbox.
				CheckTeardown<DetailsPanel>(false, [](DetailsPanel& panel) { return HWND(panel.m_btn_pin); });
			}

			// An enclosing constructor failure must also detach DetailsPanel layout.
			PRUnitTestMethod(DetailsPanelConstructorUnwind, Quick)
			{
				// Leave unrelated observers active while the containing owner unwinds.
				CheckTeardown<DetailsPanel>(true, [](DetailsPanel& panel) { return HWND(panel.m_btn_pin); });
			}
		};
	}
	#endif
}
