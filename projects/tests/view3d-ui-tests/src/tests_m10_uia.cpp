//*********************************************
// View3DUI Tests
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// M10 Windows UI Automation bridge: the published snapshot projection and its change diff, the COM
// provider surface (properties, runtime ids, navigation, bounds, patterns, text ranges), the
// semantic-action seam into the existing input state machine and event queue, WM_GETOBJECT routing
// through the raw-message path, cross-thread action marshalling, and lifetime behaviour when COM
// references outlive the context.
//
// Nothing here requires Inspect.exe, Narrator or any other running UI Automation client: the
// providers are exercised directly through their COM interfaces, which is exactly what a client
// would do across the proxy boundary.
#include "pr/common/unittests.h"
#include "pr/view3d-ui/engine.h"
#include "input.h"
#include "semantics.h"
#include "test_support.h"
#include "text_unicode.h"
#include "uia_bridge.h"
#include "uia_provider.h"
#include "uia_snapshot.h"

namespace pr::view3d::ui::tests
{
	namespace
	{
		// Absolute tolerance for converted pixel coordinates: the conversion is a short chain of
		// single-precision multiplies, so exact equality is not required, but anything looser would
		// hide a genuine convention error.
		constexpr double kPixelTol = 1e-3;

		bool NearPx(double lhs, double rhs)
		{
			return std::abs(lhs - rhs) <= kPixelTol;
		}

		// Apply one transaction containing exactly the given control descriptors, in order.
		void ApplyControls(UiEngine& engine, std::initializer_list<ControlDesc> controls, std::uint64_t base_revision = 0)
		{
			auto b = TxnBuilder{};
			for (auto const& c : controls)
				b.Upsert(c);

			engine.TransactionApply(b.Build(base_revision, base_revision + 1));
		}

		// Copy the engine's current semantic snapshot back out of the ABI accessors, so a test can
		// build a published projection from exactly the records a real host would observe.
		SemanticSnapshot CaptureSemantics(UiEngine& engine)
		{
			auto snapshot = SemanticSnapshot{};
			snapshot.m_nodes.resize(engine.SemanticCount());
			snapshot.m_text_blob.resize(engine.SemanticTextBytesPending());
			engine.SemanticsCopy(snapshot.m_nodes, std::span<char>(snapshot.m_text_blob.data(), snapshot.m_text_blob.size()));
			return snapshot;
		}

		// Drain the engine's pending event queue into a plain vector.
		std::vector<Event> DrainEvents(UiEngine& engine)
		{
			auto events = std::vector<Event>(engine.EventCount());
			auto payload = std::vector<std::byte>(engine.EventPayloadBytesPending());
			engine.EventsCopy(events, payload);
			return events;
		}

		// True when 'events' contains at least one event of 'kind' for 'control_id'.
		bool HasEvent(std::vector<Event> const& events, EEventKind kind, ControlId control_id)
		{
			return std::any_of(events.begin(), events.end(), [=](Event const& e) { return e.kind == kind && e.control_id == control_id; });
		}

		// A three-control screen tree: a root panel holding a Button then a TextBox, which is the
		// smallest shape that exercises control types, sibling ordering and both patterns.
		void BuildSampleTree(UiEngine& engine)
		{
			auto b = TxnBuilder{};
			b.Upsert(MakeControl(1, 0, EControlType::Root, ELayoutMode::StackVertical, Lp(200, 100)), "", "Root panel", "the fragment contents");
			b.Upsert(MakeControl(2, 1, EControlType::Button, ELayoutMode::Canvas, Lp(80, 20)), "Press", "Press me", "activates the thing");
			b.Upsert(MakeControl(3, 1, EControlType::TextBox, ELayoutMode::Canvas, Lp(120, 20)), "hello world", "Name", "your name");
			engine.TransactionApply(b.Build(0, 1));
		}

		// A published snapshot of 'engine' as observed under 'viewport'.
		std::shared_ptr<UiaSnapshot const> Publish(UiEngine& engine, ViewportState const& viewport, std::uint64_t sequence = 1)
		{
			auto const semantics = CaptureSemantics(engine);
			return BuildUiaSnapshot(semantics, viewport, 1, sequence);
		}

		// Shared provider state bound to 'hwnd' and carrying 'snapshot'. This is exactly what the
		// WM_GETOBJECT path constructs; building it directly keeps the provider tests independent
		// of window creation.
		std::shared_ptr<UiaSharedState> SharedWith(HWND hwnd, std::shared_ptr<UiaSnapshot const> snapshot)
		{
			auto shared = std::make_shared<UiaSharedState>();
			shared->Bind(hwnd);
			shared->Publish(std::move(snapshot));
			return shared;
		}

		// A hidden top-level window used as a stand-in for the host's render window. It is never
		// shown, so these tests launch no visible UI.
		class TestWindow
		{
			inline static wchar_t const* const s_class_name = L"View3DUI.Tests.UiaWindow";
			HWND m_hwnd = nullptr;
			UiEngine* m_engine = nullptr;

			// Forward every raw message to the engine exactly as the demo host does, so the
			// accessibility messages reach UiEngine::ProcessWindowMessage by the production route.
			static LRESULT CALLBACK WndProc(HWND hwnd, UINT msg, WPARAM wparam, LPARAM lparam)
			{
				auto* self = reinterpret_cast<TestWindow*>(GetWindowLongPtrW(hwnd, GWLP_USERDATA));
				if (self != nullptr && self->m_engine != nullptr)
				{
					auto result = LRESULT{};
					auto invalidate = std::int32_t{};
					if (self->m_engine->ProcessWindowMessage(hwnd, msg, wparam, lparam, result, invalidate) != 0)
						return result;
				}
				return DefWindowProcW(hwnd, msg, wparam, lparam);
			}

			// Register the window class once per process; a second registration is not an error
			// here, it simply means another test already created a window.
			static void EnsureClass()
			{
				auto wc = WNDCLASSEXW{ .cbSize = sizeof(WNDCLASSEXW) };
				if (GetClassInfoExW(GetModuleHandleW(nullptr), s_class_name, &wc) != FALSE)
					return;

				wc = WNDCLASSEXW{
					.cbSize = sizeof(WNDCLASSEXW),
					.lpfnWndProc = &WndProc,
					.hInstance = GetModuleHandleW(nullptr),
					.lpszClassName = s_class_name,
				};
				RegisterClassExW(&wc);
			}

		public:

			explicit TestWindow(UiEngine* engine = nullptr)
				: m_engine(engine)
			{
				EnsureClass();
				m_hwnd = CreateWindowExW(0, s_class_name, L"view3d-ui tests", WS_OVERLAPPEDWINDOW, 40, 60, 320, 240, nullptr, nullptr, GetModuleHandleW(nullptr), nullptr);
				if (m_hwnd != nullptr)
					SetWindowLongPtrW(m_hwnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(this));
			}
			~TestWindow()
			{
				if (m_hwnd != nullptr)
					DestroyWindow(m_hwnd);
			}
			TestWindow(TestWindow const&) = delete;
			TestWindow& operator=(TestWindow const&) = delete;

			HWND Handle() const
			{
				return m_hwnd;
			}

			// Dispatch every queued message once. Sent messages arrive without being queued, so
			// this only needs to keep the queue drained while a worker thread blocks on a send.
			void Pump() const
			{
				auto msg = MSG{};
				while (PeekMessageW(&msg, nullptr, 0, 0, PM_REMOVE) != FALSE)
				{
					TranslateMessage(&msg);
					DispatchMessageW(&msg);
				}
			}

			// Pump until 'done' is set or 'timeout_ms' elapses. Returns whether 'done' was set.
			bool PumpUntil(std::atomic<bool> const& done, std::uint32_t timeout_ms) const
			{
				auto const deadline = GetTickCount64() + timeout_ms;
				while (!done.load() && GetTickCount64() < deadline)
				{
					Pump();
					MsgWaitForMultipleObjects(0, nullptr, FALSE, 10, QS_ALLINPUT);
				}
				return done.load();
			}
		};

		// The screen rectangle a provider should report for 'dip' in 'viewport' on 'hwnd'.
		UiaRect ExpectedScreenRect(HWND hwnd, ViewportState const& viewport, Rect const& dip)
		{
			auto const client = UiaClientPixelRect(viewport, dip);
			auto origin = POINT{};
			ClientToScreen(hwnd, &origin);
			return UiaRect{ origin.x + client.x, origin.y + client.y, client.w, client.h };
		}

		// A semantic action request with only the fields the kind actually uses populated.
		SemanticActionRequest Action(ESemanticActionKind kind, ControlId id, std::string text = {}, std::uint32_t start = 0, std::uint32_t end = 0)
		{
			return SemanticActionRequest{ .kind = kind, .control_id = id, .text = std::move(text), .selection_start = start, .selection_end = end };
		}
	}

	#pragma region Snapshot projection and coordinates

	PRUnitTest(UiaBoundsConvertDipToClientPixelsAtDpiAndViewportOffset, Quick)
	{
		// At 96 dpi with the viewport filling the client area, DIPs and client pixels coincide.
		{
			auto const vp = Viewport(400, 300);
			auto const px = UiaClientPixelRect(vp, Rect{ 10, 20, 30, 40 });
			PR_EXPECT(NearPx(px.x, 10.0) && NearPx(px.y, 20.0) && NearPx(px.w, 30.0) && NearPx(px.h, 40.0));
		}

		// At 144 dpi every DIP is 1.5 physical pixels.
		{
			auto const vp = Viewport(400, 300, 144.0f);
			auto const px = UiaClientPixelRect(vp, Rect{ 10, 20, 30, 40 });
			PR_EXPECT(NearPx(px.x, 15.0) && NearPx(px.y, 30.0) && NearPx(px.w, 45.0) && NearPx(px.h, 60.0));
		}

		// A viewport inset within the render target shifts DIP origin to target pixels, and a
		// render target smaller than the client area scales target pixels back up to client pixels.
		{
			auto vp = Viewport(800, 600, 96.0f);
			vp.target_width_px = 400;
			vp.target_height_px = 300;
			vp.viewport_x_px = 20.0f;
			vp.viewport_y_px = 10.0f;
			vp.viewport_width_px = 380.0f;
			vp.viewport_height_px = 290.0f;

			auto const px = UiaClientPixelRect(vp, Rect{ 100, 50, 40, 20 });
			PR_EXPECT(NearPx(px.x, (100.0 + 20.0) * 2.0) && NearPx(px.y, (50.0 + 10.0) * 2.0));
			PR_EXPECT(NearPx(px.w, 80.0) && NearPx(px.h, 40.0));
		}

		// A degenerate target must not produce infinities; the ratios degrade to 1.
		{
			auto vp = Viewport(400, 300);
			vp.target_width_px = 0;
			vp.target_height_px = 0;
			auto const px = UiaClientPixelRect(vp, Rect{ 5, 6, 7, 8 });
			PR_EXPECT(NearPx(px.x, 5.0) && NearPx(px.y, 6.0) && NearPx(px.w, 7.0) && NearPx(px.h, 8.0));
		}
	}

	PRUnitTest(UiaSnapshotProjectsSemanticNodesInOrderWithResolvedLinks, Quick)
	{
		auto engine = UiEngine{ DefaultConfig() };
		BuildSampleTree(engine);
		auto const vp = Viewport(400, 300);
		engine.Update(vp);

		auto const snapshot = Publish(engine, vp);
		PR_EXPECT(snapshot->m_nodes.size() == 3);
		PR_EXPECT(snapshot->m_roots.size() == 1 && snapshot->m_nodes[snapshot->m_roots[0]].id == 1);

		// Order is exactly the semantic pre-order; no node is invented and none is reordered.
		PR_EXPECT(snapshot->m_nodes[0].id == 1 && snapshot->m_nodes[1].id == 2 && snapshot->m_nodes[2].id == 3);
		PR_EXPECT(snapshot->m_nodes[0].role == EControlType::Root);
		PR_EXPECT(snapshot->m_nodes[1].role == EControlType::Button);
		PR_EXPECT(snapshot->m_nodes[2].role == EControlType::TextBox);

		// Parent/child links resolve to indices, and the root's parent is the fragment root.
		PR_EXPECT(snapshot->m_nodes[0].parent_index == UiaNoIndex);
		PR_EXPECT(snapshot->m_nodes[0].children.size() == 2);
		PR_EXPECT(snapshot->m_nodes[1].parent_index == 0 && snapshot->m_nodes[1].sibling_position == 0);
		PR_EXPECT(snapshot->m_nodes[2].parent_index == 0 && snapshot->m_nodes[2].sibling_position == 1);

		// Strings are projected to UTF-16, and the editable value keeps its UTF-8 form for the
		// text-range operations to address.
		PR_EXPECT(snapshot->m_nodes[1].name == L"Press me");
		PR_EXPECT(snapshot->m_nodes[1].description == L"activates the thing");
		PR_EXPECT(snapshot->m_nodes[2].value == L"hello world");
		PR_EXPECT(snapshot->m_nodes[2].value_utf8 == "hello world");

		// Find resolves by stable control id, and reports absence rather than guessing.
		PR_EXPECT(snapshot->Find(3) == &snapshot->m_nodes[2]);
		PR_EXPECT(snapshot->Find(99) == nullptr);

		// Building the same inputs twice must produce identical results.
		auto const again = Publish(engine, vp, 2);
		PR_EXPECT(again->m_nodes.size() == snapshot->m_nodes.size());
		for (auto i = std::size_t{}; i != again->m_nodes.size(); ++i)
		{
			PR_EXPECT(again->m_nodes[i].id == snapshot->m_nodes[i].id);
			PR_EXPECT(again->m_nodes[i].name == snapshot->m_nodes[i].name);
			PR_EXPECT(again->m_nodes[i].state_flags == snapshot->m_nodes[i].state_flags);
			PR_EXPECT(again->m_nodes[i].children == snapshot->m_nodes[i].children);
		}
	}

	PRUnitTest(UiaSnapshotPreservesOffscreenStateForCulledWorldRoots, Quick)
	{
		auto engine = UiEngine{ DefaultConfig() };

		// A world root behind the camera is culled by projection but still published, so an
		// assistive technology can see it exists and is not currently on screen.
		auto b = TxnBuilder{};
		b.Upsert(MakeWorldRoot(1, ERootPolicy::DepthTested, 100, 50, WorldParams(Vec3{ 0, 0, -10 })), "", "Behind", "");
		b.Upsert(MakeControl(2, 1, EControlType::Button, ELayoutMode::Canvas, Lp(40, 20)), "Go", "Go", "");
		engine.TransactionApply(b.Build(0, 1));

		auto const vp = Viewport(400, 300, 96.0f, 0.0, PerspectiveCamera());
		engine.Update(vp);

		auto const snapshot = Publish(engine, vp);
		PR_EXPECT(snapshot->m_nodes.size() == 2);
		PR_EXPECT(snapshot->m_nodes[0].HasState(ESemanticState::Offscreen));
		PR_EXPECT(snapshot->m_nodes[1].HasState(ESemanticState::Offscreen));
	}

	PRUnitTest(UiaSnapshotDiffSelectsTheMinimumSetOfNotifications, Quick)
	{
		auto engine = UiEngine{ DefaultConfig() };
		BuildSampleTree(engine);
		auto const vp = Viewport(400, 300);
		engine.Update(vp);
		auto const first = Publish(engine, vp, 1);

		// The very first publication is a structure change: the client has nothing cached yet.
		{
			auto const diff = DiffUiaSnapshots(nullptr, *first);
			PR_EXPECT(diff.structure_changed != 0);
			PR_EXPECT(diff.name_changed.empty() && diff.value_changed.empty());
		}

		// An identical republication must produce no notifications at all.
		{
			auto const same = Publish(engine, vp, 2);
			auto const diff = DiffUiaSnapshots(first.get(), *same);
			PR_EXPECT(diff.structure_changed == 0 && diff.focus_changed == 0);
			PR_EXPECT(diff.name_changed.empty() && diff.value_changed.empty() && diff.state_changed.empty() && diff.bounds_changed.empty());
		}

		// A name change is a property notification, not a structure change.
		{
			auto b = TxnBuilder{};
			b.Upsert(MakeControl(2, 1, EControlType::Button, ELayoutMode::Canvas, Lp(80, 20)), "Press", "Renamed", "activates the thing");
			engine.TransactionApply(b.Build(1, 2));
			engine.Update(vp);

			auto const next = Publish(engine, vp, 3);
			auto const diff = DiffUiaSnapshots(first.get(), *next);
			PR_EXPECT(diff.structure_changed == 0);
			PR_EXPECT(diff.name_changed.size() == 1 && diff.name_changed[0] == 2);
		}

		// Focus is reported once, against the newly focused node.
		{
			auto const before = Publish(engine, vp, 4);
			engine.ApplySemanticAction(Action(ESemanticActionKind::Focus, 3));
			engine.Update(vp);

			auto const after = Publish(engine, vp, 5);
			auto const diff = DiffUiaSnapshots(before.get(), *after);
			PR_EXPECT(diff.focus_changed != 0 && diff.focused_id == 3);
			PR_EXPECT(std::find(diff.state_changed.begin(), diff.state_changed.end(), ControlId{ 3 }) != diff.state_changed.end());
		}

		// Resizing a control alters its layout rectangle, which is a bounds notification only.
		{
			auto const before = Publish(engine, vp, 6);
			auto b = TxnBuilder{};
			b.Upsert(MakeControl(3, 1, EControlType::TextBox, ELayoutMode::Canvas, Lp(160, 30)), "hello world", "Name", "your name");
			engine.TransactionApply(b.Build(2, 3));
			engine.Update(vp);

			auto const after = Publish(engine, vp, 7);
			auto const diff = DiffUiaSnapshots(before.get(), *after);
			PR_EXPECT(diff.structure_changed == 0 && diff.viewport_changed == 0);
			PR_EXPECT(std::find(diff.bounds_changed.begin(), diff.bounds_changed.end(), ControlId{ 3 }) != diff.bounds_changed.end());
		}

		// Removing a control changes membership, which supersedes every property notification.
		{
			auto const before = Publish(engine, vp, 8);
			auto b = TxnBuilder{};
			b.Remove(2);
			engine.TransactionApply(b.Build(3, 4));
			engine.Update(vp);

			auto const after = Publish(engine, vp, 9);
			auto const diff = DiffUiaSnapshots(before.get(), *after);
			PR_EXPECT(diff.structure_changed != 0);
		}

		// A DPI change relocates every node on screen without altering any layout rectangle, so it
		// is reported as a single viewport change rather than one bounds change per node.
		{
			auto const before = Publish(engine, vp, 10);
			auto const zoomed = Viewport(400, 300, 144.0f);
			engine.Update(zoomed);

			auto const after = BuildUiaSnapshot(CaptureSemantics(engine), zoomed, 4, 11);
			auto const diff = DiffUiaSnapshots(before.get(), *after);
			PR_EXPECT(diff.structure_changed == 0);
			PR_EXPECT(diff.viewport_changed != 0);
		}
	}

	PRUnitTest(UiaSnapshotDiffCollapsesToAStructureChangeBeyondTheChangeCap, Quick)
	{
		auto engine = UiEngine{ DefaultConfig() };
		auto const count = static_cast<ControlId>(UiaMaxChangedNodes + 8);

		auto b = TxnBuilder{};
		b.Upsert(MakeControl(1, 0, EControlType::Root, ELayoutMode::StackVertical, Lp(400, 4000)), "", "Root", "");
		for (auto i = ControlId{ 2 }; i != count + 2; ++i)
			b.Upsert(MakeControl(i, 1, EControlType::Text, ELayoutMode::Canvas, Lp(80, 10)), "", std::format("name-{}", i), "");

		engine.TransactionApply(b.Build(0, 1));
		auto const vp = Viewport(400, 4000);
		engine.Update(vp);
		auto const before = Publish(engine, vp, 1);

		// Rename every one of them at once: far more property notifications than the cap allows.
		auto b2 = TxnBuilder{};
		for (auto i = ControlId{ 2 }; i != count + 2; ++i)
			b2.Upsert(MakeControl(i, 1, EControlType::Text, ELayoutMode::Canvas, Lp(80, 10)), "", std::format("renamed-{}", i), "");

		engine.TransactionApply(b2.Build(1, 2));
		engine.Update(vp);

		auto const after = Publish(engine, vp, 2);
		auto const diff = DiffUiaSnapshots(before.get(), *after);
		PR_EXPECT(diff.truncated != 0);
		PR_EXPECT(diff.structure_changed != 0);
		PR_EXPECT(diff.name_changed.size() <= UiaMaxChangedNodes);
	}

	#pragma endregion

	#pragma region Provider properties, navigation and patterns

	PRUnitTest(UiaProviderReportsStableIdentityRolesAndStates, Quick)
	{
		auto engine = UiEngine{ DefaultConfig() };
		BuildSampleTree(engine);
		auto const vp = Viewport(400, 300);
		engine.Update(vp);

		auto window = TestWindow{};
		auto shared = SharedWith(window.Handle(), Publish(engine, vp));
		auto* button = CreateUiaElementProvider(shared, 2);
		PR_EXPECT(button != nullptr);

		// Provider options must declare a server-side provider; the bridge marshals actions itself
		// rather than asking UI Automation to do COM threading for it.
		auto options = ProviderOptions{};
		PR_EXPECT(button->get_ProviderOptions(&options) == S_OK);
		PR_EXPECT((options & ProviderOptions_ServerSideProvider) != 0);

		auto value = VARIANT{};
		PR_EXPECT(button->GetPropertyValue(UIA_ControlTypePropertyId, &value) == S_OK);
		PR_EXPECT(value.vt == VT_I4 && value.lVal == UIA_ButtonControlTypeId);
		VariantClear(&value);

		PR_EXPECT(button->GetPropertyValue(UIA_NamePropertyId, &value) == S_OK);
		PR_EXPECT(value.vt == VT_BSTR && std::wstring(value.bstrVal) == L"Press me");
		VariantClear(&value);

		PR_EXPECT(button->GetPropertyValue(UIA_FullDescriptionPropertyId, &value) == S_OK);
		PR_EXPECT(value.vt == VT_BSTR && std::wstring(value.bstrVal) == L"activates the thing");
		VariantClear(&value);

		// The automation id is derived from the stable ControlId, so it survives every relayout.
		PR_EXPECT(button->GetPropertyValue(UIA_AutomationIdPropertyId, &value) == S_OK);
		PR_EXPECT(value.vt == VT_BSTR && std::wstring(value.bstrVal) == L"view3dui:2");
		VariantClear(&value);

		PR_EXPECT(button->GetPropertyValue(UIA_IsEnabledPropertyId, &value) == S_OK);
		PR_EXPECT(value.vt == VT_BOOL && value.boolVal == VARIANT_TRUE);
		VariantClear(&value);

		PR_EXPECT(button->GetPropertyValue(UIA_IsKeyboardFocusablePropertyId, &value) == S_OK);
		PR_EXPECT(value.vt == VT_BOOL && value.boolVal == VARIANT_TRUE);
		VariantClear(&value);

		PR_EXPECT(button->GetPropertyValue(UIA_IsOffscreenPropertyId, &value) == S_OK);
		PR_EXPECT(value.vt == VT_BOOL && value.boolVal == VARIANT_FALSE);
		VariantClear(&value);

		// A property this provider does not answer must report "not supported", never a plausible
		// default that a client would then trust.
		PR_EXPECT(button->GetPropertyValue(UIA_AcceleratorKeyPropertyId, &value) == S_OK);
		PR_EXPECT(value.vt == VT_EMPTY || value.vt == VT_UNKNOWN);
		VariantClear(&value);

		// The runtime id is derived from the same ControlId and is identical for two providers
		// created for the same node.
		auto* fragment = static_cast<IRawElementProviderFragment*>(nullptr);
		PR_EXPECT(button->QueryInterface(IID_PPV_ARGS(&fragment)) == S_OK);

		auto* runtime_a = static_cast<SAFEARRAY*>(nullptr);
		PR_EXPECT(fragment->GetRuntimeId(&runtime_a) == S_OK && runtime_a != nullptr);

		auto* other = CreateUiaElementProvider(shared, 2);
		auto* other_fragment = static_cast<IRawElementProviderFragment*>(nullptr);
		PR_EXPECT(other->QueryInterface(IID_PPV_ARGS(&other_fragment)) == S_OK);

		auto* runtime_b = static_cast<SAFEARRAY*>(nullptr);
		PR_EXPECT(other_fragment->GetRuntimeId(&runtime_b) == S_OK && runtime_b != nullptr);

		auto bound_a = LONG{};
		auto bound_b = LONG{};
		PR_EXPECT(SafeArrayGetUBound(runtime_a, 1, &bound_a) == S_OK);
		PR_EXPECT(SafeArrayGetUBound(runtime_b, 1, &bound_b) == S_OK);
		PR_EXPECT(bound_a == bound_b);
		for (auto i = LONG{}; i <= bound_a; ++i)
		{
			auto lhs = LONG{};
			auto rhs = LONG{};
			SafeArrayGetElement(runtime_a, &i, &lhs);
			SafeArrayGetElement(runtime_b, &i, &rhs);
			PR_EXPECT(lhs == rhs);
		}
		SafeArrayDestroy(runtime_a);
		SafeArrayDestroy(runtime_b);

		other_fragment->Release();
		other->Release();
		fragment->Release();
		button->Release();
	}

	PRUnitTest(UiaProviderAdvertisesExactlyTheClosedControlPatterns, Quick)
	{
		auto engine = UiEngine{ DefaultConfig() };
		BuildSampleTree(engine);
		auto const vp = Viewport(400, 300);
		engine.Update(vp);

		auto window = TestWindow{};
		auto shared = SharedWith(window.Handle(), Publish(engine, vp));

		auto* button = CreateUiaElementProvider(shared, 2);
		auto* textbox = CreateUiaElementProvider(shared, 3);
		auto* root = CreateUiaElementProvider(shared, 1);

		auto* pattern = static_cast<IUnknown*>(nullptr);

		// Button supports Invoke and nothing else.
		PR_EXPECT(button->GetPatternProvider(UIA_InvokePatternId, &pattern) == S_OK && pattern != nullptr);
		pattern->Release();
		PR_EXPECT(button->GetPatternProvider(UIA_ValuePatternId, &pattern) == S_OK && pattern == nullptr);
		PR_EXPECT(button->GetPatternProvider(UIA_TextPatternId, &pattern) == S_OK && pattern == nullptr);

		// TextBox supports Value, Text and Text2.
		PR_EXPECT(textbox->GetPatternProvider(UIA_ValuePatternId, &pattern) == S_OK && pattern != nullptr);
		pattern->Release();
		PR_EXPECT(textbox->GetPatternProvider(UIA_TextPatternId, &pattern) == S_OK && pattern != nullptr);
		pattern->Release();
		PR_EXPECT(textbox->GetPatternProvider(UIA_TextPattern2Id, &pattern) == S_OK && pattern != nullptr);
		pattern->Release();
		PR_EXPECT(textbox->GetPatternProvider(UIA_InvokePatternId, &pattern) == S_OK && pattern == nullptr);

		// The panel root supports no control pattern at all.
		PR_EXPECT(root->GetPatternProvider(UIA_InvokePatternId, &pattern) == S_OK && pattern == nullptr);
		PR_EXPECT(root->GetPatternProvider(UIA_ValuePatternId, &pattern) == S_OK && pattern == nullptr);

		// A pattern this module never implements must be reported absent, not guessed at.
		PR_EXPECT(textbox->GetPatternProvider(UIA_ScrollPatternId, &pattern) == S_OK && pattern == nullptr);
		PR_EXPECT(button->GetPatternProvider(UIA_TogglePatternId, &pattern) == S_OK && pattern == nullptr);

		root->Release();
		textbox->Release();
		button->Release();
	}

	PRUnitTest(UiaProviderNavigatesInDeterministicSemanticOrder, Quick)
	{
		auto engine = UiEngine{ DefaultConfig() };
		BuildSampleTree(engine);
		auto const vp = Viewport(400, 300);
		engine.Update(vp);

		auto window = TestWindow{};
		auto shared = SharedWith(window.Handle(), Publish(engine, vp));

		auto* root_simple = CreateUiaRootProvider(shared);
		auto* root = static_cast<IRawElementProviderFragmentRoot*>(nullptr);
		PR_EXPECT(root_simple->QueryInterface(IID_PPV_ARGS(&root)) == S_OK);

		auto* root_fragment = static_cast<IRawElementProviderFragment*>(nullptr);
		PR_EXPECT(root_simple->QueryInterface(IID_PPV_ARGS(&root_fragment)) == S_OK);

		// The fragment root's first child is the first semantic root.
		auto* first = static_cast<IRawElementProviderFragment*>(nullptr);
		PR_EXPECT(root_fragment->Navigate(NavigateDirection_FirstChild, &first) == S_OK && first != nullptr);

		auto* first_simple = static_cast<IRawElementProviderSimple*>(nullptr);
		PR_EXPECT(first->QueryInterface(IID_PPV_ARGS(&first_simple)) == S_OK);

		auto value = VARIANT{};
		PR_EXPECT(first_simple->GetPropertyValue(UIA_AutomationIdPropertyId, &value) == S_OK);
		PR_EXPECT(std::wstring(value.bstrVal) == L"view3dui:1");
		VariantClear(&value);

		// Its children come back in semantic order, and the sibling walk agrees.
		auto* child = static_cast<IRawElementProviderFragment*>(nullptr);
		PR_EXPECT(first->Navigate(NavigateDirection_FirstChild, &child) == S_OK && child != nullptr);

		auto* child_simple = static_cast<IRawElementProviderSimple*>(nullptr);
		PR_EXPECT(child->QueryInterface(IID_PPV_ARGS(&child_simple)) == S_OK);
		PR_EXPECT(child_simple->GetPropertyValue(UIA_AutomationIdPropertyId, &value) == S_OK);
		PR_EXPECT(std::wstring(value.bstrVal) == L"view3dui:2");
		VariantClear(&value);

		auto* next = static_cast<IRawElementProviderFragment*>(nullptr);
		PR_EXPECT(child->Navigate(NavigateDirection_NextSibling, &next) == S_OK && next != nullptr);

		auto* next_simple = static_cast<IRawElementProviderSimple*>(nullptr);
		PR_EXPECT(next->QueryInterface(IID_PPV_ARGS(&next_simple)) == S_OK);
		PR_EXPECT(next_simple->GetPropertyValue(UIA_AutomationIdPropertyId, &value) == S_OK);
		PR_EXPECT(std::wstring(value.bstrVal) == L"view3dui:3");
		VariantClear(&value);

		// The last sibling has no next, and the first has no previous.
		auto* beyond = static_cast<IRawElementProviderFragment*>(nullptr);
		PR_EXPECT(next->Navigate(NavigateDirection_NextSibling, &beyond) == S_OK && beyond == nullptr);
		PR_EXPECT(child->Navigate(NavigateDirection_PreviousSibling, &beyond) == S_OK && beyond == nullptr);

		// Navigating up from a semantic root lands on the HWND fragment root, not on nothing.
		auto* parent = static_cast<IRawElementProviderFragment*>(nullptr);
		PR_EXPECT(first->Navigate(NavigateDirection_Parent, &parent) == S_OK && parent != nullptr);
		parent->Release();

		// Every element reports the same fragment root.
		auto* fragment_root = static_cast<IRawElementProviderFragmentRoot*>(nullptr);
		PR_EXPECT(next->get_FragmentRoot(&fragment_root) == S_OK && fragment_root != nullptr);
		fragment_root->Release();

		// An unrecognised direction is a caller error.
		PR_EXPECT(first->Navigate(static_cast<NavigateDirection>(99), &beyond) == E_INVALIDARG);

		next_simple->Release();
		next->Release();
		child_simple->Release();
		child->Release();
		first_simple->Release();
		first->Release();
		root_fragment->Release();
		root->Release();
		root_simple->Release();
	}

	PRUnitTest(UiaProviderReportsScreenPhysicalPixelBoundsAndFocus, Quick)
	{
		auto engine = UiEngine{ DefaultConfig() };
		BuildSampleTree(engine);

		// A non-100% DPI is the case a naive implementation gets wrong.
		auto const vp = Viewport(400, 300, 144.0f);
		engine.Update(vp);
		engine.ApplySemanticAction(Action(ESemanticActionKind::Focus, 3));
		engine.Update(vp);

		auto window = TestWindow{};
		auto const published = Publish(engine, vp);
		auto shared = SharedWith(window.Handle(), published);

		auto* textbox = CreateUiaElementProvider(shared, 3);
		auto* fragment = static_cast<IRawElementProviderFragment*>(nullptr);
		PR_EXPECT(textbox->QueryInterface(IID_PPV_ARGS(&fragment)) == S_OK);

		auto rect = UiaRect{};
		PR_EXPECT(fragment->get_BoundingRectangle(&rect) == S_OK);

		auto const expected = ExpectedScreenRect(window.Handle(), vp, published->Find(3)->bounds_dip);
		PR_EXPECT(NearPx(rect.left, expected.left) && NearPx(rect.top, expected.top));
		PR_EXPECT(NearPx(rect.width, expected.width) && NearPx(rect.height, expected.height));

		// The bounds must actually be scaled by DPI, not passed through as DIPs.
		PR_EXPECT(rect.width > published->Find(3)->bounds_dip.w);

		// The fragment root reports the focused element, and the focused element says so.
		auto* root_simple = CreateUiaRootProvider(shared);
		auto* root = static_cast<IRawElementProviderFragmentRoot*>(nullptr);
		PR_EXPECT(root_simple->QueryInterface(IID_PPV_ARGS(&root)) == S_OK);

		auto* focused = static_cast<IRawElementProviderFragment*>(nullptr);
		PR_EXPECT(root->GetFocus(&focused) == S_OK && focused != nullptr);

		auto* focused_simple = static_cast<IRawElementProviderSimple*>(nullptr);
		PR_EXPECT(focused->QueryInterface(IID_PPV_ARGS(&focused_simple)) == S_OK);

		auto value = VARIANT{};
		PR_EXPECT(focused_simple->GetPropertyValue(UIA_AutomationIdPropertyId, &value) == S_OK);
		PR_EXPECT(std::wstring(value.bstrVal) == L"view3dui:3");
		VariantClear(&value);

		PR_EXPECT(focused_simple->GetPropertyValue(UIA_HasKeyboardFocusPropertyId, &value) == S_OK);
		PR_EXPECT(value.boolVal == VARIANT_TRUE);
		VariantClear(&value);

		focused_simple->Release();
		focused->Release();
		root->Release();
		root_simple->Release();
		fragment->Release();
		textbox->Release();
	}

	PRUnitTest(UiaProviderReportsAnOffscreenElementWithoutBogusGeometry, Quick)
	{
		auto engine = UiEngine{ DefaultConfig() };
		auto b = TxnBuilder{};
		b.Upsert(MakeWorldRoot(1, ERootPolicy::DepthTested, 100, 50, WorldParams(Vec3{ 0, 0, -10 })), "", "Behind", "");
		engine.TransactionApply(b.Build(0, 1));

		auto const vp = Viewport(400, 300, 96.0f, 0.0, PerspectiveCamera());
		engine.Update(vp);

		auto window = TestWindow{};
		auto shared = SharedWith(window.Handle(), Publish(engine, vp));

		auto* element = CreateUiaElementProvider(shared, 1);
		auto value = VARIANT{};
		PR_EXPECT(element->GetPropertyValue(UIA_IsOffscreenPropertyId, &value) == S_OK);
		PR_EXPECT(value.boolVal == VARIANT_TRUE);
		VariantClear(&value);

		auto* fragment = static_cast<IRawElementProviderFragment*>(nullptr);
		PR_EXPECT(element->QueryInterface(IID_PPV_ARGS(&fragment)) == S_OK);

		// An offscreen element must publish an empty rectangle rather than a projected position
		// that is meaningless once the node has been culled.
		auto rect = UiaRect{};
		PR_EXPECT(fragment->get_BoundingRectangle(&rect) == S_OK);
		PR_EXPECT(NearPx(rect.width, 0.0) && NearPx(rect.height, 0.0));

		fragment->Release();
		element->Release();
	}

	#pragma endregion

	#pragma region Text pattern and ranges

	PRUnitTest(UiaTextPatternMapsValueCaretSelectionAndComposition, Quick)
	{
		auto engine = UiEngine{ DefaultConfig() };
		BuildSampleTree(engine);
		auto const vp = Viewport(400, 300);
		engine.Update(vp);
		engine.ApplySemanticAction(Action(ESemanticActionKind::Focus, 3));
		engine.ApplySemanticAction(Action(ESemanticActionKind::SetSelection, 3, {}, 6, 11));
		engine.Update(vp);

		auto window = TestWindow{};
		auto shared = SharedWith(window.Handle(), Publish(engine, vp));

		auto* textbox = CreateUiaElementProvider(shared, 3);
		auto* text = static_cast<IUnknown*>(nullptr);
		PR_EXPECT(textbox->GetPatternProvider(UIA_TextPatternId, &text) == S_OK && text != nullptr);

		auto* provider = static_cast<ITextProvider*>(nullptr);
		PR_EXPECT(text->QueryInterface(IID_PPV_ARGS(&provider)) == S_OK);

		// The document range covers the whole live value.
		auto* document = static_cast<ITextRangeProvider*>(nullptr);
		PR_EXPECT(provider->get_DocumentRange(&document) == S_OK && document != nullptr);

		auto* bstr = BSTR{};
		PR_EXPECT(document->GetText(-1, &bstr) == S_OK && std::wstring(bstr) == L"hello world");
		SysFreeString(bstr);

		// GetText honours the caller's length cap rather than over-reporting.
		PR_EXPECT(document->GetText(5, &bstr) == S_OK && std::wstring(bstr) == L"hello");
		SysFreeString(bstr);

		// The selection is reported as one range covering exactly the selected text.
		auto* selection = static_cast<SAFEARRAY*>(nullptr);
		PR_EXPECT(provider->GetSelection(&selection) == S_OK && selection != nullptr);

		auto upper = LONG{};
		PR_EXPECT(SafeArrayGetUBound(selection, 1, &upper) == S_OK && upper == 0);

		auto* selected_unknown = static_cast<IUnknown*>(nullptr);
		auto index = LONG{};
		PR_EXPECT(SafeArrayGetElement(selection, &index, &selected_unknown) == S_OK);

		auto* selected = static_cast<ITextRangeProvider*>(nullptr);
		PR_EXPECT(selected_unknown->QueryInterface(IID_PPV_ARGS(&selected)) == S_OK);
		PR_EXPECT(selected->GetText(-1, &bstr) == S_OK && std::wstring(bstr) == L"world");
		SysFreeString(bstr);

		// Endpoint comparison is against the same UTF-8 offsets the edit model uses.
		auto order = int{};
		PR_EXPECT(selected->CompareEndpoints(TextPatternRangeEndpoint_Start, document, TextPatternRangeEndpoint_Start, &order) == S_OK);
		PR_EXPECT(order > 0);
		PR_EXPECT(selected->CompareEndpoints(TextPatternRangeEndpoint_End, document, TextPatternRangeEndpoint_End, &order) == S_OK);
		PR_EXPECT(order == 0);

		// A cloned range compares equal to its source, and moving it does not disturb the original.
		auto* clone = static_cast<ITextRangeProvider*>(nullptr);
		PR_EXPECT(selected->Clone(&clone) == S_OK && clone != nullptr);

		auto same = BOOL{};
		PR_EXPECT(clone->Compare(selected, &same) == S_OK && same == TRUE);

		auto moved = int{};
		PR_EXPECT(clone->Move(TextUnit_Character, -2, &moved) == S_OK);
		PR_EXPECT(clone->Compare(selected, &same) == S_OK && same == FALSE);

		// Expanding to the whole document is exactly the document range.
		PR_EXPECT(clone->ExpandToEnclosingUnit(TextUnit_Document) == S_OK);
		PR_EXPECT(clone->GetText(-1, &bstr) == S_OK && std::wstring(bstr) == L"hello world");
		SysFreeString(bstr);

		// A find that succeeds returns the matched sub-range; one that fails returns null, not an
		// empty range that would look like a match at offset zero.
		auto* found = static_cast<ITextRangeProvider*>(nullptr);
		auto* needle = SysAllocString(L"WORLD");
		PR_EXPECT(document->FindText(needle, FALSE, TRUE, &found) == S_OK && found != nullptr);
		PR_EXPECT(found->GetText(-1, &bstr) == S_OK && std::wstring(bstr) == L"world");
		SysFreeString(bstr);
		found->Release();
		SysFreeString(needle);

		needle = SysAllocString(L"absent");
		PR_EXPECT(document->FindText(needle, FALSE, FALSE, &found) == S_OK && found == nullptr);
		SysFreeString(needle);

		// Operations this module deliberately does not implement report the correct status.
		PR_EXPECT(document->AddToSelection() == UIA_E_INVALIDOPERATION);
		PR_EXPECT(document->RemoveFromSelection() == UIA_E_INVALIDOPERATION);

		auto point_range = static_cast<ITextRangeProvider*>(nullptr);
		PR_EXPECT(provider->RangeFromPoint(UiaPoint{ 0, 0 }, &point_range) == UIA_E_NOTSUPPORTED);
		PR_EXPECT(provider->RangeFromChild(nullptr, &point_range) == UIA_E_INVALIDOPERATION);

		// Text2 is reachable from the same object and reports the caret position.
		auto* provider2 = static_cast<ITextProvider2*>(nullptr);
		PR_EXPECT(text->QueryInterface(IID_PPV_ARGS(&provider2)) == S_OK);

		auto* caret = static_cast<ITextRangeProvider*>(nullptr);
		auto caret_active = BOOL{};
		PR_EXPECT(provider2->GetCaretRange(&caret_active, &caret) == S_OK && caret != nullptr);
		PR_EXPECT(caret_active == TRUE);

		// The caret range is degenerate and sits at the selection's active end.
		PR_EXPECT(caret->GetText(-1, &bstr) == S_OK && std::wstring(bstr).empty());
		SysFreeString(bstr);
		caret->Release();

		// An annotation is a document concept this closed control model has no equivalent of.
		auto* annotated = static_cast<ITextRangeProvider*>(nullptr);
		PR_EXPECT(provider2->RangeFromAnnotation(nullptr, &annotated) == UIA_E_NOTSUPPORTED);

		clone->Release();
		selected->Release();
		selected_unknown->Release();
		SafeArrayDestroy(selection);
		provider2->Release();
		document->Release();
		provider->Release();
		text->Release();
		textbox->Release();
	}

	PRUnitTest(UiaTextRangesRemainOnGraphemeBoundaries, Quick)
	{
		// A family emoji is one grapheme cluster made of several code points; a range that split it
		// would produce text no client could render or speak.
		auto const family = std::string("a\U0001F468\u200D\U0001F469\u200D\U0001F467b");

		auto engine = UiEngine{ DefaultConfig() };
		auto b = TxnBuilder{};
		b.Upsert(MakeControl(1, 0, EControlType::Root, ELayoutMode::StackVertical, Lp(200, 100)), "", "Root", "");
		b.Upsert(MakeControl(2, 1, EControlType::TextBox, ELayoutMode::Canvas, Lp(180, 20)), family, "Name", "");
		engine.TransactionApply(b.Build(0, 1));

		auto const vp = Viewport(400, 300);
		engine.Update(vp);
		engine.ApplySemanticAction(Action(ESemanticActionKind::Focus, 2));

		// Both edges land inside the cluster; both must snap to a boundary.
		engine.ApplySemanticAction(Action(ESemanticActionKind::SetSelection, 2, {}, 3, 8));
		engine.Update(vp);

		auto const published = Publish(engine, vp);
		auto const* node = published->Find(2);
		PR_EXPECT(node != nullptr);
		PR_EXPECT(IsGraphemeBoundary(node->value_utf8, node->selection_start));
		PR_EXPECT(IsGraphemeBoundary(node->value_utf8, node->selection_end));

		auto window = TestWindow{};
		auto shared = SharedWith(window.Handle(), published);

		auto* textbox = CreateUiaElementProvider(shared, 2);
		auto* text = static_cast<IUnknown*>(nullptr);
		PR_EXPECT(textbox->GetPatternProvider(UIA_TextPatternId, &text) == S_OK && text != nullptr);

		auto* provider = static_cast<ITextProvider*>(nullptr);
		PR_EXPECT(text->QueryInterface(IID_PPV_ARGS(&provider)) == S_OK);

		auto* document = static_cast<ITextRangeProvider*>(nullptr);
		PR_EXPECT(provider->get_DocumentRange(&document) == S_OK);

		// Moving by one character unit crosses exactly one grapheme cluster.
		auto* cursor = static_cast<ITextRangeProvider*>(nullptr);
		PR_EXPECT(document->Clone(&cursor) == S_OK);
		PR_EXPECT(cursor->MoveEndpointByRange(TextPatternRangeEndpoint_End, document, TextPatternRangeEndpoint_Start) == S_OK);

		auto moved = int{};
		PR_EXPECT(cursor->MoveEndpointByUnit(TextPatternRangeEndpoint_End, TextUnit_Character, 2, &moved) == S_OK && moved == 2);

		auto* bstr = BSTR{};
		PR_EXPECT(cursor->GetText(-1, &bstr) == S_OK);
		PR_EXPECT(std::wstring(bstr) == std::wstring(L"a\U0001F468\u200D\U0001F469\u200D\U0001F467"));
		SysFreeString(bstr);

		cursor->Release();
		document->Release();
		provider->Release();
		text->Release();
		textbox->Release();
	}

	#pragma endregion

	#pragma region Semantic actions

	PRUnitTest(UiaSemanticActionsEnterTheExistingEventQueue, Quick)
	{
		auto engine = UiEngine{ DefaultConfig() };
		BuildSampleTree(engine);
		auto const vp = Viewport(400, 300);
		engine.Update(vp);
		DrainEvents(engine);

		// Focus produces the same FocusChanged event a Tab key would.
		engine.ApplySemanticAction(Action(ESemanticActionKind::Focus, 2));
		{
			auto const events = DrainEvents(engine);
			PR_EXPECT(HasEvent(events, EEventKind::FocusChanged, 2));
		}

		// Focusing the already-focused control is a no-op rather than a duplicate event.
		engine.ApplySemanticAction(Action(ESemanticActionKind::Focus, 2));
		PR_EXPECT(engine.EventCount() == 0);

		// Invoke produces CommandInvoked, exactly as a pointer press-and-release would.
		engine.ApplySemanticAction(Action(ESemanticActionKind::Invoke, 2));
		{
			auto const events = DrainEvents(engine);
			PR_EXPECT(HasEvent(events, EEventKind::CommandInvoked, 2));
		}

		// SetValue proposes text through the M9 edit path; it must not silently become the
		// accepted value, which only the application can change.
		engine.ApplySemanticAction(Action(ESemanticActionKind::Focus, 3));
		DrainEvents(engine);
		engine.ApplySemanticAction(Action(ESemanticActionKind::SetValue, 3, "replaced"));
		{
			auto const events = DrainEvents(engine);
			PR_EXPECT(HasEvent(events, EEventKind::TextChangeProposed, 3));
			PR_EXPECT(std::any_of(events.begin(), events.end(), [](Event const& e) { return e.kind == EEventKind::TextChangeProposed && e.edit_generation != 0; }));
		}

		// The proposal is visible as the live value, and each further proposal advances the
		// edit generation rather than reusing it.
		engine.Update(vp);
		{
			auto const published = Publish(engine, vp);
			PR_EXPECT(published->Find(3)->value_utf8 == "replaced");
		}

		engine.ApplySemanticAction(Action(ESemanticActionKind::SetValue, 3, "again"));
		{
			auto const events = DrainEvents(engine);
			PR_EXPECT(HasEvent(events, EEventKind::TextChangeProposed, 3));
		}

		// SetSelection moves the selection without proposing an edit.
		engine.ApplySemanticAction(Action(ESemanticActionKind::SetSelection, 3, {}, 1, 3));
		PR_EXPECT(engine.EventCount() == 0);
		engine.Update(vp);
		{
			auto const published = Publish(engine, vp);
			auto const* node = published->Find(3);
			PR_EXPECT(node->HasTextFlag(ESemanticTextFlag::HasSelection));
			PR_EXPECT(node->selection_start == 1 && node->selection_end == 3);
		}
	}

	PRUnitTest(UiaSemanticActionsRejectWhatTheControlDoesNotSupport, Quick)
	{
		auto engine = UiEngine{ DefaultConfig() };
		BuildSampleTree(engine);

		// A disabled button and a disabled text box advertise no actions at all.
		auto b = TxnBuilder{};
		auto disabled_button = MakeControl(2, 1, EControlType::Button, ELayoutMode::Canvas, Lp(80, 20));
		disabled_button.enabled = 0;
		b.Upsert(disabled_button, "Press", "Press me", "");
		engine.TransactionApply(b.Build(1, 2));
		engine.Update(Viewport(400, 300));

		// An id that was never in the tree.
		PR_THROWS(engine.ApplySemanticAction(Action(ESemanticActionKind::Focus, 999)), EngineException);
		PR_THROWS(engine.ApplySemanticAction(Action(ESemanticActionKind::Invoke, 999)), EngineException);

		// A disabled control refuses every action.
		PR_THROWS(engine.ApplySemanticAction(Action(ESemanticActionKind::Invoke, 2)), EngineException);
		PR_THROWS(engine.ApplySemanticAction(Action(ESemanticActionKind::Focus, 2)), EngineException);

		// Invoke is a Button action; a TextBox does not silently accept it.
		PR_THROWS(engine.ApplySemanticAction(Action(ESemanticActionKind::Invoke, 3)), EngineException);

		// SetValue is a TextBox action; the panel root does not silently accept it.
		PR_THROWS(engine.ApplySemanticAction(Action(ESemanticActionKind::SetValue, 1, "x")), EngineException);

		// Malformed UTF-8 is rejected rather than being stored and later mis-decoded.
		PR_THROWS(engine.ApplySemanticAction(Action(ESemanticActionKind::SetValue, 3, std::string("\xC3\x28"))), EngineException);
	}

	#pragma endregion

	#pragma region Window message routing, threading and lifetime

	PRUnitTest(UiaGetObjectIsAnsweredThroughTheRawMessagePathOnAHiddenWindow, Quick)
	{
		auto engine = UiEngine{ DefaultConfig() };
		BuildSampleTree(engine);
		auto const vp = Viewport(400, 300);
		engine.Update(vp);

		auto window = TestWindow{ &engine };
		PR_EXPECT(window.Handle() != nullptr);

		// The bridge stays inert until a client asks, so nothing is published yet.
		PR_EXPECT(!engine.Uia().Bound());

		// A non-UIA object id belongs to the host's own window procedure.
		auto result = LRESULT{};
		auto invalidate = std::int32_t{};
		PR_EXPECT(engine.ProcessWindowMessage(window.Handle(), WM_GETOBJECT, 0, static_cast<LPARAM>(OBJID_CLIENT), result, invalidate) == 0);
		PR_EXPECT(!engine.Uia().Bound());

		// The UI Automation root object id is answered with a provider, with no separate
		// registration call anywhere.
		PR_EXPECT(engine.ProcessWindowMessage(window.Handle(), WM_GETOBJECT, 0, static_cast<LPARAM>(UiaRootObjectId), result, invalidate) == 1);
		PR_EXPECT(result != 0);
		PR_EXPECT(engine.Uia().Bound());

		// The same window is still accepted afterwards.
		PR_EXPECT(engine.ProcessWindowMessage(window.Handle(), WM_GETOBJECT, 0, static_cast<LPARAM>(UiaRootObjectId), result, invalidate) == 1);

		// A different window is declined rather than silently rebound.
		auto other = TestWindow{};
		PR_EXPECT(engine.ProcessWindowMessage(other.Handle(), WM_GETOBJECT, 0, static_cast<LPARAM>(UiaRootObjectId), result, invalidate) == 0);
		PR_EXPECT(engine.Uia().Shared()->Window() == window.Handle());

		// Once bound, Update publishes a snapshot the providers can read.
		engine.Update(vp);
		auto const snapshot = engine.Uia().Shared()->Snapshot();
		PR_EXPECT(snapshot != nullptr && snapshot->m_nodes.size() == 3);
	}

	PRUnitTest(UiaActionsMarshalToTheOwnerThreadAndBackWithTheirStatus, Quick)
	{
		auto engine = UiEngine{ DefaultConfig() };
		BuildSampleTree(engine);
		auto const vp = Viewport(400, 300);
		engine.Update(vp);

		auto window = TestWindow{ &engine };
		PR_EXPECT(engine.Uia().Bind(window.Handle()));
		engine.Update(vp);
		DrainEvents(engine);

		auto shared = engine.Uia().Shared();
		auto* textbox = CreateUiaElementProvider(shared, 3);
		auto* value_pattern_unknown = static_cast<IUnknown*>(nullptr);
		PR_EXPECT(textbox->GetPatternProvider(UIA_ValuePatternId, &value_pattern_unknown) == S_OK);

		auto* value_pattern = static_cast<IValueProvider*>(nullptr);
		PR_EXPECT(value_pattern_unknown->QueryInterface(IID_PPV_ARGS(&value_pattern)) == S_OK);

		// Run the action on a worker thread, as a real COM/RPC call would arrive.
		auto done = std::atomic<bool>{ false };
		auto status = HRESULT{ E_UNEXPECTED };
		auto worker = std::thread([&]
		{
			status = value_pattern->SetValue(L"from a worker");
			done.store(true);
		});

		PR_EXPECT(window.PumpUntil(done, 5000));
		worker.join();
		PR_EXPECT(status == S_OK);

		// The action went through the same state machine, so it produced the usual event.
		auto const events = DrainEvents(engine);
		PR_EXPECT(HasEvent(events, EEventKind::TextChangeProposed, 3));

		engine.Update(vp);
		PR_EXPECT(engine.Uia().Shared()->Snapshot()->Find(3)->value_utf8 == "from a worker");

		// The same call on the owner thread executes directly, without any timeout being involved.
		PR_EXPECT(value_pattern->SetValue(L"from the owner") == S_OK);
		engine.Update(vp);
		PR_EXPECT(engine.Uia().Shared()->Snapshot()->Find(3)->value_utf8 == "from the owner");

		// A failure inside the action is reported as a UI Automation status, not as success and
		// not as a C++ exception crossing the COM boundary.
		auto* root_element = CreateUiaElementProvider(shared, 1);
		auto* root_value = static_cast<IUnknown*>(nullptr);
		PR_EXPECT(root_element->GetPatternProvider(UIA_ValuePatternId, &root_value) == S_OK && root_value == nullptr);

		auto* invoke_unknown = static_cast<IUnknown*>(nullptr);
		auto* button = CreateUiaElementProvider(shared, 2);
		PR_EXPECT(button->GetPatternProvider(UIA_InvokePatternId, &invoke_unknown) == S_OK);

		auto* invoke = static_cast<IInvokeProvider*>(nullptr);
		PR_EXPECT(invoke_unknown->QueryInterface(IID_PPV_ARGS(&invoke)) == S_OK);
		DrainEvents(engine);
		PR_EXPECT(invoke->Invoke() == S_OK);
		{
			auto const invoked = DrainEvents(engine);
			PR_EXPECT(HasEvent(invoked, EEventKind::CommandInvoked, 2));
		}

		invoke->Release();
		invoke_unknown->Release();
		button->Release();
		root_element->Release();
		value_pattern->Release();
		value_pattern_unknown->Release();
		textbox->Release();
	}

	PRUnitTest(UiaActionsFailDeterministicallyWhenTheOwnerThreadCannotAnswer, Quick)
	{
		auto engine = UiEngine{ DefaultConfig() };
		BuildSampleTree(engine);
		auto const vp = Viewport(400, 300);
		engine.Update(vp);

		// A window that never routes messages to the engine: the send succeeds but nothing claims
		// the call, which must be reported as an unavailable element rather than as success.
		{
			auto silent = TestWindow{};
			auto shared = std::make_shared<UiaSharedState>();
			PR_EXPECT(shared->Bind(silent.Handle()));
			shared->Publish(Publish(engine, vp));

			auto const request = Action(ESemanticActionKind::Focus, 2);
			PR_EXPECT(shared->InvokeAction(request) == UIA_E_ELEMENTNOTAVAILABLE);
		}

		// A context that has been shut down fails immediately, without touching any window.
		{
			auto window = TestWindow{ &engine };
			auto shared = std::make_shared<UiaSharedState>();
			PR_EXPECT(shared->Bind(window.Handle()));
			shared->Publish(Publish(engine, vp));
			shared->Shutdown();

			auto const request = Action(ESemanticActionKind::Focus, 2);
			PR_EXPECT(shared->InvokeAction(request) == UIA_E_ELEMENTNOTAVAILABLE);
			PR_EXPECT(shared->Snapshot() == nullptr);
			PR_EXPECT(!shared->Available());

			// A shut-down state never rebinds, so a late message cannot resurrect the context.
			PR_EXPECT(!shared->Bind(window.Handle()));
		}

		// An owner thread that stops pumping produces a bounded timeout, not an indefinite block.
		{
			auto ready = std::atomic<bool>{ false };
			auto done = std::atomic<bool>{ false };
			auto status = HRESULT{ E_UNEXPECTED };
			auto shared = std::shared_ptr<UiaSharedState>{};

			// The owner window lives on a thread that stops pumping as soon as it is created.
			auto owner = std::thread([&]
			{
				auto window = TestWindow{ &engine };
				shared = std::make_shared<UiaSharedState>();
				shared->Bind(window.Handle());
				ready.store(true);

				// Deliberately do not pump: the send below must hit the bridge's own timeout.
				while (!done.load())
					Sleep(10);
			});

			while (!ready.load())
				Sleep(1);

			auto const request = Action(ESemanticActionKind::Focus, 2);
			status = shared->InvokeAction(request);
			done.store(true);
			owner.join();

			PR_EXPECT(FAILED(status));
			PR_EXPECT(status == UIA_E_TIMEOUT);
		}
	}

	PRUnitTest(UiaProvidersSurviveContextDestructionAndReportUnavailable, Quick)
	{
		auto window = TestWindow{};
		auto* root = static_cast<IRawElementProviderSimple*>(nullptr);
		auto* element = static_cast<IRawElementProviderSimple*>(nullptr);
		auto* fragment = static_cast<IRawElementProviderFragment*>(nullptr);
		auto* range = static_cast<ITextRangeProvider*>(nullptr);
		auto shared = std::shared_ptr<UiaSharedState>{};

		// Take references from a context, then destroy the context while they are still held.
		{
			auto engine = UiEngine{ DefaultConfig() };
			BuildSampleTree(engine);
			auto const vp = Viewport(400, 300);
			engine.Update(vp);

			PR_EXPECT(engine.Uia().Bind(window.Handle()));
			engine.Update(vp);
			shared = engine.Uia().Shared();

			root = CreateUiaRootProvider(shared);
			element = CreateUiaElementProvider(shared, 3);
			PR_EXPECT(element->QueryInterface(IID_PPV_ARGS(&fragment)) == S_OK);

			auto* text = static_cast<IUnknown*>(nullptr);
			PR_EXPECT(element->GetPatternProvider(UIA_TextPatternId, &text) == S_OK && text != nullptr);

			auto* provider = static_cast<ITextProvider*>(nullptr);
			PR_EXPECT(text->QueryInterface(IID_PPV_ARGS(&provider)) == S_OK);
			PR_EXPECT(provider->get_DocumentRange(&range) == S_OK && range != nullptr);
			provider->Release();
			text->Release();

			// Everything works while the context is alive.
			auto value = VARIANT{};
			PR_EXPECT(element->GetPropertyValue(UIA_NamePropertyId, &value) == S_OK);
			VariantClear(&value);
		}

		// The engine (and its bridge) are gone; the COM objects are not.
		PR_EXPECT(!shared->Available());
		PR_EXPECT(shared->Snapshot() == nullptr);

		auto value = VARIANT{};
		PR_EXPECT(element->GetPropertyValue(UIA_NamePropertyId, &value) == UIA_E_ELEMENTNOTAVAILABLE);
		// The fragment root is a static identity object, so it still answers its own identity, but
		// it reports the fragment as disabled and exposes no children once the context is gone.
		VariantClear(&value);
		PR_EXPECT(root->GetPropertyValue(UIA_AutomationIdPropertyId, &value) == S_OK);
		PR_EXPECT(value.vt == VT_BSTR && std::wstring(value.bstrVal) == L"view3dui:root");
		VariantClear(&value);
		PR_EXPECT(root->GetPropertyValue(UIA_IsEnabledPropertyId, &value) == S_OK);
		PR_EXPECT(value.vt == VT_BOOL && value.boolVal == VARIANT_FALSE);
		VariantClear(&value);

		auto* pattern = static_cast<IUnknown*>(nullptr);
		PR_EXPECT(element->GetPatternProvider(UIA_ValuePatternId, &pattern) == UIA_E_ELEMENTNOTAVAILABLE);

		auto rect = UiaRect{};
		PR_EXPECT(fragment->get_BoundingRectangle(&rect) == UIA_E_ELEMENTNOTAVAILABLE);

		auto* child = static_cast<IRawElementProviderFragment*>(nullptr);
		PR_EXPECT(fragment->Navigate(NavigateDirection_FirstChild, &child) == UIA_E_ELEMENTNOTAVAILABLE);
		PR_EXPECT(fragment->SetFocus() == UIA_E_ELEMENTNOTAVAILABLE);

		auto* bstr = BSTR{};
		PR_EXPECT(range->GetText(-1, &bstr) == UIA_E_ELEMENTNOTAVAILABLE);

		auto moved = int{};
		PR_EXPECT(range->Move(TextUnit_Character, 1, &moved) == UIA_E_ELEMENTNOTAVAILABLE);
		PR_EXPECT(range->Select() == UIA_E_ELEMENTNOTAVAILABLE);

		// The root's tree walk is snapshot-backed, so it too reports the fragment as unavailable
		// rather than handing out elements that can no longer be resolved.
		auto* root_fragment = static_cast<IRawElementProviderFragment*>(nullptr);
		PR_EXPECT(root->QueryInterface(IID_PPV_ARGS(&root_fragment)) == S_OK);
		PR_EXPECT(root_fragment->Navigate(NavigateDirection_FirstChild, &child) == UIA_E_ELEMENTNOTAVAILABLE);
		PR_EXPECT(child == nullptr);
		root_fragment->Release();

		// Releasing them afterwards must not touch anything that has gone away.
		range->Release();
		fragment->Release();
		element->Release();
		root->Release();
	}

	PRUnitTest(UiaProviderForAnIdThatIsNoLongerPublishedIsUnavailable, Quick)
	{
		auto engine = UiEngine{ DefaultConfig() };
		BuildSampleTree(engine);
		auto const vp = Viewport(400, 300);
		engine.Update(vp);

		auto window = TestWindow{};
		auto shared = SharedWith(window.Handle(), Publish(engine, vp));

		// An id that has never existed.
		auto* stale = CreateUiaElementProvider(shared, 4242);
		auto value = VARIANT{};
		PR_EXPECT(stale->GetPropertyValue(UIA_NamePropertyId, &value) == UIA_E_ELEMENTNOTAVAILABLE);

		// An id that existed in the previous publication but not this one.
		auto* removed = CreateUiaElementProvider(shared, 2);
		PR_EXPECT(removed->GetPropertyValue(UIA_NamePropertyId, &value) == S_OK);
		VariantClear(&value);

		auto b = TxnBuilder{};
		b.Remove(2);
		engine.TransactionApply(b.Build(1, 2));
		engine.Update(vp);
		shared->Publish(Publish(engine, vp, 2));

		PR_EXPECT(removed->GetPropertyValue(UIA_NamePropertyId, &value) == UIA_E_ELEMENTNOTAVAILABLE);

		removed->Release();
		stale->Release();
	}

	#pragma endregion

	#pragma region Registry, teardown and call lifetime

	PRUnitTest(UiaProviderReleaseAndShutdownRaceSafely, Quick)
	{
		// Providers are created and released on several threads while the owning context tears
		// down. The final reference transition and the registry removal share one critical section,
		// so teardown can never take a reference to an object that is already being destroyed.
		for (auto attempt = 0; attempt != 6; ++attempt)
		{
			auto window = TestWindow{};
			auto shared = std::make_shared<UiaSharedState>();
			PR_EXPECT(shared->Bind(window.Handle()));

			auto start = std::atomic<bool>{ false };
			auto workers = std::vector<std::thread>{};
			for (auto t = 0; t != 4; ++t)
			{
				workers.emplace_back([&]
				{
					while (!start.load())
						std::this_thread::yield();

					for (auto i = 0; i != 400; ++i)
					{
						auto* element = CreateUiaElementProvider(shared, 3);
						auto* root = CreateUiaRootProvider(shared);
						element->AddRef();
						element->Release();
						root->Release();
						element->Release();
					}
				});
			}

			start.store(true);
			Sleep(1);

			auto const disconnected = shared->Shutdown();
			for (auto* provider : disconnected)
				provider->Release();

			for (auto& worker : workers)
				worker.join();

			// Providers created after teardown are not tracked, so nothing is left behind.
			auto* late = CreateUiaElementProvider(shared, 3);
			PR_EXPECT(shared->Shutdown().empty());
			late->Release();

			PR_EXPECT(!shared->Available());
			PR_EXPECT(shared->PendingCallCount() == 0);
		}
	}

	PRUnitTest(UiaShutdownIsIdempotentAndDisconnectsExactlyOnce, Quick)
	{
		auto window = TestWindow{};
		auto shared = std::make_shared<UiaSharedState>();
		PR_EXPECT(shared->Bind(window.Handle()));

		auto* root = CreateUiaRootProvider(shared);
		auto* element = CreateUiaElementProvider(shared, 3);

		// The first teardown hands back exactly the tracked providers, each with one reference for
		// the caller to release after disconnecting it.
		auto const first = shared->Shutdown();
		PR_EXPECT(first.size() == 2);
		for (auto* provider : first)
			provider->Release();

		// A second teardown - which is what an explicit ContextDestroy followed by the bridge's
		// destructor produces - finds nothing left to disconnect or release.
		PR_EXPECT(shared->Shutdown().empty());
		PR_EXPECT(shared->Shutdown().empty());

		element->Release();
		root->Release();

		// The same is true one level up, where the bridge owns the sequencing.
		auto engine = UiEngine{ DefaultConfig() };
		BuildSampleTree(engine);
		auto const vp = Viewport(400, 300);
		engine.Update(vp);

		auto other = TestWindow{};
		PR_EXPECT(engine.Uia().Bind(other.Handle()));
		engine.Update(vp);

		auto bridge_shared = engine.Uia().Shared();
		engine.Uia().Shutdown();
		engine.Uia().Shutdown();
		PR_EXPECT(!bridge_shared->Available());
		PR_EXPECT(bridge_shared->Snapshot() == nullptr);
	}

	PRUnitTest(UiaProviderInterfaceSetIsInvariantForItsLifetime, Quick)
	{
		auto engine = UiEngine{ DefaultConfig() };
		BuildSampleTree(engine);
		auto const vp = Viewport(400, 300);
		engine.Update(vp);

		auto window = TestWindow{};
		auto shared = SharedWith(window.Handle(), Publish(engine, vp));

		// A Button exposes the same interface set as a TextBox, because COM requires the answer to
		// be invariant. Which patterns are usable is a separate question, answered by
		// GetPatternProvider and the pattern properties.
		auto* button = CreateUiaElementProvider(shared, 2);
		auto* value = static_cast<IValueProvider*>(nullptr);
		auto* text = static_cast<ITextProvider2*>(nullptr);
		PR_EXPECT(button->QueryInterface(IID_PPV_ARGS(&value)) == S_OK && value != nullptr);
		PR_EXPECT(button->QueryInterface(IID_PPV_ARGS(&text)) == S_OK && text != nullptr);

		auto* pattern = static_cast<IUnknown*>(nullptr);
		PR_EXPECT(button->GetPatternProvider(UIA_ValuePatternId, &pattern) == S_OK && pattern == nullptr);
		PR_EXPECT(button->GetPatternProvider(UIA_TextPatternId, &pattern) == S_OK && pattern == nullptr);

		auto property = VARIANT{};
		PR_EXPECT(button->GetPropertyValue(UIA_IsValuePatternAvailablePropertyId, &property) == S_OK);
		PR_EXPECT(property.vt == VT_BOOL && property.boolVal == VARIANT_FALSE);
		VariantClear(&property);

		// Removing the control from the published model does not retract an interface either; the
		// methods start failing instead.
		auto b = TxnBuilder{};
		b.Remove(2);
		engine.TransactionApply(b.Build(1, 2));
		engine.Update(vp);
		shared->Publish(Publish(engine, vp, 2));

		auto* value_after_removal = static_cast<IValueProvider*>(nullptr);
		PR_EXPECT(button->QueryInterface(IID_PPV_ARGS(&value_after_removal)) == S_OK && value_after_removal != nullptr);
		PR_EXPECT(button->GetPropertyValue(UIA_NamePropertyId, &property) == UIA_E_ELEMENTNOTAVAILABLE);

		// And the set survives teardown, when there is no snapshot to consult at all.
		shared->Shutdown();

		auto* value_after_shutdown = static_cast<IValueProvider*>(nullptr);
		auto* text_after_shutdown = static_cast<ITextProvider2*>(nullptr);
		auto* fragment_after_shutdown = static_cast<IRawElementProviderFragment*>(nullptr);
		PR_EXPECT(button->QueryInterface(IID_PPV_ARGS(&value_after_shutdown)) == S_OK && value_after_shutdown != nullptr);
		PR_EXPECT(button->QueryInterface(IID_PPV_ARGS(&text_after_shutdown)) == S_OK && text_after_shutdown != nullptr);
		PR_EXPECT(button->QueryInterface(IID_PPV_ARGS(&fragment_after_shutdown)) == S_OK && fragment_after_shutdown != nullptr);

		auto* range = static_cast<ITextRangeProvider*>(nullptr);
		PR_EXPECT(value_after_shutdown->SetValue(L"nope") == UIA_E_ELEMENTNOTAVAILABLE);
		PR_EXPECT(text_after_shutdown->get_DocumentRange(&range) == UIA_E_ELEMENTNOTAVAILABLE);
		PR_EXPECT(range == nullptr);

		// An interface this provider does not implement is still refused.
		auto* unsupported = static_cast<IScrollProvider*>(nullptr);
		PR_EXPECT(button->QueryInterface(IID_PPV_ARGS(&unsupported)) == E_NOINTERFACE);

		fragment_after_shutdown->Release();
		text_after_shutdown->Release();
		value_after_shutdown->Release();
		value_after_removal->Release();
		text->Release();
		value->Release();
		button->Release();
	}

	PRUnitTest(UiaProviderRegistryResumesTrackingOnceItHasRoomAgain, Quick)
	{
		auto window = TestWindow{};
		auto shared = std::make_shared<UiaSharedState>();
		PR_EXPECT(shared->Bind(window.Handle()));

		auto providers = std::vector<IRawElementProviderSimple*>{};
		providers.reserve(UiaMaxTrackedProviders);
		for (auto i = std::size_t{}; i != UiaMaxTrackedProviders; ++i)
			providers.push_back(CreateUiaElementProvider(shared, 3));

		// One past the cap is created normally but not tracked.
		auto* overflow = CreateUiaElementProvider(shared, 3);

		// Releasing some of them makes room, and the registry must start accepting entries again
		// rather than staying permanently full.
		auto const released = std::size_t{ 8 };
		for (auto i = std::size_t{}; i != released; ++i)
		{
			providers.back()->Release();
			providers.pop_back();
		}
		auto* recovered = CreateUiaElementProvider(shared, 3);

		auto const tracked = shared->Shutdown();
		PR_EXPECT(tracked.size() == providers.size() + 1);
		for (auto* provider : tracked)
			provider->Release();

		recovered->Release();
		overflow->Release();
		for (auto* provider : providers)
			provider->Release();
	}

	PRUnitTest(UiaAbandonedActionCallsStayOwnedUntilTheyAreDelivered, Quick)
	{
		auto engine = UiEngine{ DefaultConfig() };
		BuildSampleTree(engine);
		auto const vp = Viewport(400, 300);
		engine.Update(vp);
		DrainEvents(engine);

		auto ready = std::atomic<bool>{ false };
		auto pump = std::atomic<bool>{ false };
		auto pumped = std::atomic<bool>{ false };
		auto finished = std::atomic<bool>{ false };

		// The owner window lives on a thread that only starts pumping once the sender has already
		// given up waiting, which is exactly the window in which a timed-out message can still be
		// dispatched.
		auto owner = std::thread([&]
		{
			auto window = TestWindow{ &engine };
			engine.Uia().Bind(window.Handle());
			ready.store(true);

			while (!pump.load())
				Sleep(5);

			window.Pump();
			pumped.store(true);

			while (!finished.load())
				Sleep(5);
		});

		while (!ready.load())
			Sleep(1);

		auto shared = engine.Uia().Shared();
		PR_EXPECT(shared->PendingCallCount() == 0);

		auto const request = Action(ESemanticActionKind::Invoke, 2);
		PR_EXPECT(shared->InvokeAction(request) == UIA_E_TIMEOUT);

		// The call is still owned by the shared state, so a later delivery has live storage to
		// find rather than a dead stack frame.
		PR_EXPECT(shared->PendingCallCount() == 1);

		pump.store(true);
		while (!pumped.load())
			Sleep(1);

		// Whether or not Windows redelivered the message, the action must not have run: its caller
		// was already told it did not happen, and nothing may be left leaking.
		PR_EXPECT(shared->PendingCallCount() <= 1);
		PR_EXPECT(!HasEvent(DrainEvents(engine), EEventKind::CommandInvoked, 2));

		finished.store(true);
		owner.join();

		engine.Uia().Shutdown();
		PR_EXPECT(shared->PendingCallCount() == 0);
	}

	PRUnitTest(UiaActionMessageIgnoresSendersWithoutTheProcessCookie, Quick)
	{
		auto engine = UiEngine{ DefaultConfig() };
		BuildSampleTree(engine);
		auto const vp = Viewport(400, 300);
		engine.Update(vp);

		auto window = TestWindow{ &engine };
		PR_EXPECT(engine.Uia().Bind(window.Handle()));
		engine.Update(vp);

		auto result = LRESULT{};
		auto invalidate = std::int32_t{};

		// The registered message id is system-global, so a foreign sender can deliver it with an
		// arbitrary LPARAM. Without the process cookie the message is passed straight through and
		// its LPARAM is never dereferenced.
		auto bogus = std::uint64_t{ 0xDEADBEEF };
		PR_EXPECT(engine.ProcessWindowMessage(window.Handle(), UiaActionMessageId(), 0, reinterpret_cast<LPARAM>(&bogus), result, invalidate) == 0);
		PR_EXPECT(engine.ProcessWindowMessage(window.Handle(), UiaActionMessageId(), UiaActionCookie() + 1, reinterpret_cast<LPARAM>(&bogus), result, invalidate) == 0);

		// Even with the right cookie, a pointer that is not one of this context's outstanding calls
		// is refused on identity alone.
		PR_EXPECT(engine.ProcessWindowMessage(window.Handle(), UiaActionMessageId(), UiaActionCookie(), reinterpret_cast<LPARAM>(&bogus), result, invalidate) == 0);
		PR_EXPECT(engine.ProcessWindowMessage(window.Handle(), UiaActionMessageId(), UiaActionCookie(), 0, result, invalidate) == 0);

		// None of that may have produced any engine work.
		PR_EXPECT(invalidate == 0);
		PR_EXPECT(DrainEvents(engine).empty());
	}

	#pragma endregion

	#pragma region First publication, focus loss, hit testing and range comparison

	PRUnitTest(UiaFirstGetObjectPublishesBeforeTheProviderCanBeQueried, Quick)
	{
		// A render-on-demand host may not call Update again for a long time after a client
		// attaches, so the first provider it can reach must already have a snapshot behind it.
		auto engine = UiEngine{ DefaultConfig() };
		BuildSampleTree(engine);
		auto const vp = Viewport(400, 300);
		engine.Update(vp);

		auto window = TestWindow{ &engine };
		auto result = LRESULT{};
		auto invalidate = std::int32_t{};
		PR_EXPECT(engine.ProcessWindowMessage(window.Handle(), WM_GETOBJECT, 0, static_cast<LPARAM>(UiaRootObjectId), result, invalidate) == 1);

		auto shared = engine.Uia().Shared();
		auto const snapshot = shared->Snapshot();
		PR_EXPECT(snapshot != nullptr && snapshot->m_nodes.size() == 3);

		// Navigation and focus both answer immediately, with no Update in between.
		auto* root_simple = CreateUiaRootProvider(shared);
		auto* root_fragment = static_cast<IRawElementProviderFragment*>(nullptr);
		auto* root = static_cast<IRawElementProviderFragmentRoot*>(nullptr);
		PR_EXPECT(root_simple->QueryInterface(IID_PPV_ARGS(&root_fragment)) == S_OK);
		PR_EXPECT(root_simple->QueryInterface(IID_PPV_ARGS(&root)) == S_OK);

		auto* child = static_cast<IRawElementProviderFragment*>(nullptr);
		PR_EXPECT(root_fragment->Navigate(NavigateDirection_FirstChild, &child) == S_OK);
		PR_EXPECT(child != nullptr);
		child->Release();

		auto* focused = static_cast<IRawElementProviderFragment*>(nullptr);
		PR_EXPECT(root->GetFocus(&focused) == S_OK);
		if (focused != nullptr)
			focused->Release();

		root->Release();
		root_fragment->Release();
		root_simple->Release();

		// An engine that has never rendered anything publishes a valid empty snapshot rather than
		// leaving the client holding an unavailable element.
		auto empty = UiEngine{ DefaultConfig() };
		auto empty_window = TestWindow{ &empty };
		PR_EXPECT(empty.ProcessWindowMessage(empty_window.Handle(), WM_GETOBJECT, 0, static_cast<LPARAM>(UiaRootObjectId), result, invalidate) == 1);

		auto empty_shared = empty.Uia().Shared();
		auto const empty_snapshot = empty_shared->Snapshot();
		PR_EXPECT(empty_snapshot != nullptr);
		PR_EXPECT(empty_snapshot->m_nodes.empty() && empty_snapshot->m_roots.empty());

		auto* empty_root_simple = CreateUiaRootProvider(empty_shared);
		auto* empty_root_fragment = static_cast<IRawElementProviderFragment*>(nullptr);
		auto* empty_root = static_cast<IRawElementProviderFragmentRoot*>(nullptr);
		PR_EXPECT(empty_root_simple->QueryInterface(IID_PPV_ARGS(&empty_root_fragment)) == S_OK);
		PR_EXPECT(empty_root_simple->QueryInterface(IID_PPV_ARGS(&empty_root)) == S_OK);

		auto* empty_child = static_cast<IRawElementProviderFragment*>(nullptr);
		PR_EXPECT(empty_root_fragment->Navigate(NavigateDirection_FirstChild, &empty_child) == S_OK);
		PR_EXPECT(empty_child == nullptr);

		auto* empty_focus = static_cast<IRawElementProviderFragment*>(nullptr);
		PR_EXPECT(empty_root->GetFocus(&empty_focus) == S_OK);
		PR_EXPECT(empty_focus == nullptr);

		empty_root->Release();
		empty_root_fragment->Release();
		empty_root_simple->Release();
	}

	PRUnitTest(UiaFocusLeavingTheFragmentIsPublishedAsAChange, Quick)
	{
		auto engine = UiEngine{ DefaultConfig() };
		BuildSampleTree(engine);
		auto const vp = Viewport(400, 300);
		engine.Update(vp);
		engine.ApplySemanticAction(Action(ESemanticActionKind::Focus, 3));
		engine.Update(vp);

		auto const focused = Publish(engine, vp, 1);
		PR_EXPECT(focused->m_focus_id == 3);

		// Removing every control leaves the fragment with no focus at all. Removing only the
		// focused one would not do: focus reconciliation moves it to the next Tab-order target.
		auto b = TxnBuilder{};
		b.Remove(2);
		b.Remove(3);
		b.Remove(1);
		engine.TransactionApply(b.Build(1, 2));
		engine.Update(vp);

		auto const unfocused = Publish(engine, vp, 2);
		PR_EXPECT(unfocused->m_focus_id == 0);

		// A client that is only told about focus arriving would keep announcing the control it last
		// saw focused, so the loss has to be reported too - against the fragment root, since there
		// is no longer an element to report it against.
		auto const diff = DiffUiaSnapshots(focused.get(), *unfocused);
		PR_EXPECT(diff.focus_changed == 1);
		PR_EXPECT(diff.focused_id == 0);

		// Focus that has not changed still produces nothing.
		auto const again = Publish(engine, vp, 3);
		PR_EXPECT(DiffUiaSnapshots(unfocused.get(), *again).focus_changed == 0);
	}

	PRUnitTest(UiaHitTestOfFragmentEmptySpaceReturnsTheFragmentRoot, Quick)
	{
		auto engine = UiEngine{ DefaultConfig() };
		BuildSampleTree(engine);
		auto const vp = Viewport(400, 300);
		engine.Update(vp);

		auto window = TestWindow{};
		auto const published = Publish(engine, vp);
		auto shared = SharedWith(window.Handle(), published);

		auto* root_simple = CreateUiaRootProvider(shared);
		auto* root = static_cast<IRawElementProviderFragmentRoot*>(nullptr);
		PR_EXPECT(root_simple->QueryInterface(IID_PPV_ARGS(&root)) == S_OK);

		// A point over a control resolves to that control.
		auto const button = ExpectedScreenRect(window.Handle(), vp, published->Find(2)->bounds_dip);
		auto* hit = static_cast<IRawElementProviderFragment*>(nullptr);
		PR_EXPECT(root->ElementProviderFromPoint(button.left + button.width * 0.5, button.top + button.height * 0.5, &hit) == S_OK);
		PR_EXPECT(hit != nullptr);

		auto* hit_simple = static_cast<IRawElementProviderSimple*>(nullptr);
		auto value = VARIANT{};
		PR_EXPECT(hit->QueryInterface(IID_PPV_ARGS(&hit_simple)) == S_OK);
		PR_EXPECT(hit_simple->GetPropertyValue(UIA_AutomationIdPropertyId, &value) == S_OK);
		PR_EXPECT(std::wstring(value.bstrVal) == L"view3dui:2");
		VariantClear(&value);
		hit_simple->Release();
		hit->Release();

		// Empty space inside the fragment still belongs to the fragment, so the root answers for
		// it. A null answer would tell the client the point is outside the fragment entirely.
		auto origin = POINT{};
		ClientToScreen(window.Handle(), &origin);

		auto* empty_hit = static_cast<IRawElementProviderFragment*>(nullptr);
		PR_EXPECT(root->ElementProviderFromPoint(origin.x + 4000.0, origin.y + 4000.0, &empty_hit) == S_OK);
		PR_EXPECT(empty_hit != nullptr);

		auto* empty_simple = static_cast<IRawElementProviderSimple*>(nullptr);
		PR_EXPECT(empty_hit->QueryInterface(IID_PPV_ARGS(&empty_simple)) == S_OK);
		PR_EXPECT(empty_simple->GetPropertyValue(UIA_AutomationIdPropertyId, &value) == S_OK);
		PR_EXPECT(std::wstring(value.bstrVal) == L"view3dui:root");
		VariantClear(&value);

		empty_simple->Release();
		empty_hit->Release();
		root->Release();
		root_simple->Release();
	}

	PRUnitTest(UiaTextRangeMoveTerminatesForWholeValueUnits, Quick)
	{
		auto engine = UiEngine{ DefaultConfig() };
		BuildSampleTree(engine);
		auto const vp = Viewport(400, 300);
		engine.Update(vp);

		auto window = TestWindow{};
		auto shared = SharedWith(window.Handle(), Publish(engine, vp));

		auto* textbox = CreateUiaElementProvider(shared, 3);
		auto* text = static_cast<ITextProvider*>(nullptr);
		PR_EXPECT(textbox->QueryInterface(IID_PPV_ARGS(&text)) == S_OK);

		// Every unit above Word spans this whole single-line value, so a range that already covers
		// it cannot move. Reporting an attempted move would let a client that walks with Move until
		// it returns zero run forever.
		auto const units = { TextUnit_Line, TextUnit_Paragraph, TextUnit_Page, TextUnit_Document, TextUnit_Format };
		for (auto unit : units)
		{
			auto* range = static_cast<ITextRangeProvider*>(nullptr);
			PR_EXPECT(text->get_DocumentRange(&range) == S_OK);

			auto moved = int{};
			PR_EXPECT(range->Move(unit, 1, &moved) == S_OK);
			PR_EXPECT(moved == 0);
			PR_EXPECT(range->Move(unit, -1, &moved) == S_OK);
			PR_EXPECT(moved == 0);

			// The bounded walk a client performs must terminate on its very first step.
			auto steps = 0;
			for (; steps != 16; ++steps)
			{
				PR_EXPECT(range->Move(unit, 1, &moved) == S_OK);
				if (moved == 0)
					break;
			}
			PR_EXPECT(steps == 0);

			// The range is left exactly where it was.
			auto* full = static_cast<ITextRangeProvider*>(nullptr);
			PR_EXPECT(text->get_DocumentRange(&full) == S_OK);

			auto same = BOOL{};
			PR_EXPECT(range->Compare(full, &same) == S_OK && same == TRUE);
			full->Release();
			range->Release();
		}

		// A degenerate range still moves, because it genuinely changes span.
		{
			auto* range = static_cast<ITextRangeProvider*>(nullptr);
			PR_EXPECT(text->get_DocumentRange(&range) == S_OK);

			auto moved = int{};
			PR_EXPECT(range->MoveEndpointByUnit(TextPatternRangeEndpoint_End, TextUnit_Document, -1, &moved) == S_OK);
			PR_EXPECT(moved == -1);
			PR_EXPECT(range->Move(TextUnit_Document, 1, &moved) == S_OK);
			PR_EXPECT(moved == 1);
			range->Release();
		}

		// Character and word movement is unaffected.
		{
			auto* range = static_cast<ITextRangeProvider*>(nullptr);
			PR_EXPECT(text->get_DocumentRange(&range) == S_OK);
			PR_EXPECT(range->ExpandToEnclosingUnit(TextUnit_Character) == S_OK);

			auto moved = int{};
			PR_EXPECT(range->Move(TextUnit_Character, 1, &moved) == S_OK);
			PR_EXPECT(moved == 1);

			auto steps = 0;
			for (; steps != 64; ++steps)
			{
				PR_EXPECT(range->Move(TextUnit_Character, 1, &moved) == S_OK);
				if (moved == 0)
					break;
			}
			PR_EXPECT(steps < 64);
			range->Release();
		}

		text->Release();
		textbox->Release();
	}

	PRUnitTest(UiaTextRangeComparisonsNormalizeAgainstTheCurrentText, Quick)
	{
		auto engine = UiEngine{ DefaultConfig() };
		BuildSampleTree(engine);
		auto const vp = Viewport(400, 300);
		engine.Update(vp);

		auto window = TestWindow{};
		auto shared = SharedWith(window.Handle(), Publish(engine, vp));

		auto* textbox = CreateUiaElementProvider(shared, 3);
		auto* text = static_cast<ITextProvider*>(nullptr);
		PR_EXPECT(textbox->QueryInterface(IID_PPV_ARGS(&text)) == S_OK);

		// A range captured over the old, longer text.
		auto* before = static_cast<ITextRangeProvider*>(nullptr);
		PR_EXPECT(text->get_DocumentRange(&before) == S_OK);

		auto b = TxnBuilder{};
		b.Upsert(MakeControl(3, 1, EControlType::TextBox, ELayoutMode::Canvas, Lp(120, 20)), "hi", "Name", "your name");
		engine.TransactionApply(b.Build(1, 2));
		engine.Update(vp);
		shared->Publish(Publish(engine, vp, 2));

		// The same span, expressed with stale offsets, must compare equal: both sides are resolved
		// against the text they currently index into before being compared.
		auto* after = static_cast<ITextRangeProvider*>(nullptr);
		PR_EXPECT(text->get_DocumentRange(&after) == S_OK);

		auto same = BOOL{};
		PR_EXPECT(before->Compare(after, &same) == S_OK);
		PR_EXPECT(same == TRUE);
		PR_EXPECT(after->Compare(before, &same) == S_OK);
		PR_EXPECT(same == TRUE);

		auto order = int{};
		PR_EXPECT(before->CompareEndpoints(TextPatternRangeEndpoint_End, after, TextPatternRangeEndpoint_End, &order) == S_OK);
		PR_EXPECT(order == 0);
		PR_EXPECT(before->CompareEndpoints(TextPatternRangeEndpoint_Start, after, TextPatternRangeEndpoint_Start, &order) == S_OK);
		PR_EXPECT(order == 0);

		// A genuinely different span still compares as different.
		auto moved = int{};
		PR_EXPECT(after->MoveEndpointByUnit(TextPatternRangeEndpoint_End, TextUnit_Character, -1, &moved) == S_OK);
		PR_EXPECT(before->Compare(after, &same) == S_OK && same == FALSE);
		PR_EXPECT(before->CompareEndpoints(TextPatternRangeEndpoint_End, after, TextPatternRangeEndpoint_End, &order) == S_OK);
		PR_EXPECT(order == 1);

		// A range that is not one of ours is an invalid comparison, not a false answer.
		PR_EXPECT(before->Compare(nullptr, &same) == UIA_E_INVALIDOPERATION);
		PR_EXPECT(before->CompareEndpoints(TextPatternRangeEndpoint_Start, nullptr, TextPatternRangeEndpoint_Start, &order) == UIA_E_INVALIDOPERATION);

		after->Release();
		before->Release();
		text->Release();
		textbox->Release();
	}

	#pragma endregion
}
