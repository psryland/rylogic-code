//*********************************************
// View3DUI
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "win32_input.h"
#include "text_unicode.h"

#pragma comment(lib, "imm32.lib")

namespace pr::view3d::ui
{
	namespace
	{
		std::uint32_t CurrentModifiers()
		{
			std::uint32_t mods = 0;
			if ((GetKeyState(VK_SHIFT) & 0x8000) != 0)
				mods |= static_cast<std::uint32_t>(EInputModifier::Shift);
			if ((GetKeyState(VK_CONTROL) & 0x8000) != 0)
				mods |= static_cast<std::uint32_t>(EInputModifier::Ctrl);
			if ((GetKeyState(VK_MENU) & 0x8000) != 0)
				mods |= static_cast<std::uint32_t>(EInputModifier::Alt);
			return mods;
		}

		std::uint32_t MouseWParamToButtonMask(WPARAM wparam)
		{
			std::uint32_t mask = 0;
			if ((wparam & MK_LBUTTON) != 0)
				mask |= static_cast<std::uint32_t>(EPointerButtonMask::Left);
			if ((wparam & MK_RBUTTON) != 0)
				mask |= static_cast<std::uint32_t>(EPointerButtonMask::Right);
			if ((wparam & MK_MBUTTON) != 0)
				mask |= static_cast<std::uint32_t>(EPointerButtonMask::Middle);
			if ((wparam & MK_XBUTTON1) != 0)
				mask |= static_cast<std::uint32_t>(EPointerButtonMask::X1);
			if ((wparam & MK_XBUTTON2) != 0)
				mask |= static_cast<std::uint32_t>(EPointerButtonMask::X2);
			return mask;
		}

		// Convert physical client pixel coordinates into DIPs (section 7.4): client px -> render
		// target px using the explicit client/target size ratio and the UI's viewport offset
		// within the target, then render target px -> DIPs using 96/dpi. In the common case
		// (target size == client size, viewport at the origin) this reduces to client_px*96/dpi.
		Vec2 ClientPixelsToDip(ViewportState const& viewport, float client_x_px, float client_y_px)
		{
			auto ratio_x = viewport.client_width_px != 0 ? static_cast<float>(viewport.target_width_px) / static_cast<float>(viewport.client_width_px) : 1.0f;
			auto ratio_y = viewport.client_height_px != 0 ? static_cast<float>(viewport.target_height_px) / static_cast<float>(viewport.client_height_px) : 1.0f;
			auto target_x = client_x_px * ratio_x - viewport.viewport_x_px;
			auto target_y = client_y_px * ratio_y - viewport.viewport_y_px;
			auto dpi_scale = 96.0f / viewport.dpi;
			return Vec2{ target_x * dpi_scale, target_y * dpi_scale };
		}

		NormalizedInput MakeBase(EInputKind kind, double time_ms)
		{
			NormalizedInput input{};
			input.header.size = sizeof(NormalizedInput);
			input.header.version = VIEW3D_UI_STRUCT_VERSION;
			input.kind = kind;
			input.button = EPointerButton::None;
			input.time_ms = time_ms;
			return input;
		}

		// A message this module does not participate in: the caller must pass it to the rest of
		// its window procedure unmodified.
		TranslatedMessage MakeUnrecognised()
		{
			return TranslatedMessage{ .recognised = 0, .handled = 0, .result = 0, .place_ime_windows = 0, .inputs = {} };
		}

		// A message that produces exactly one normalized record and no text payload.
		TranslatedMessage MakeSingle(NormalizedInput const& input)
		{
			TranslatedMessage out{ .recognised = 1, .handled = 0, .result = 0, .place_ime_windows = 0, .inputs = {} };
			out.inputs.push_back(TranslatedInput{ .input = input, .text = {}, .caret = 0, .selection_start = 0, .selection_end = 0 });
			return out;
		}

		TranslatedMessage MakePointerMessage(EInputKind kind, EPointerButton button, WPARAM wparam, LPARAM lparam, ViewportState const& viewport, double time_ms)
		{
			// LOWORD/HIWORD of lParam are physical client-relative pixel coordinates for every
			// mouse message except WM_MOUSEWHEEL (handled separately, screen-relative); casting to
			// 'short' first sign-extends correctly for coordinates outside the client area while a
			// pointer capture is active (section 7.3: "capture continues receiving pointer messages
			// outside UI bounds").
			auto client_x = static_cast<float>(static_cast<short>(LOWORD(lparam)));
			auto client_y = static_cast<float>(static_cast<short>(HIWORD(lparam)));
			auto dip = ClientPixelsToDip(viewport, client_x, client_y);

			auto input = MakeBase(kind, time_ms);
			input.pointer_x = dip.x;
			input.pointer_y = dip.y;
			input.button = button;
			input.button_mask = MouseWParamToButtonMask(wparam);
			input.modifiers = CurrentModifiers();
			return MakeSingle(input);
		}

		// The UTF-8 form of one of the IME's composition strings, or nullopt when the IME reports
		// none. GCS_* strings are UTF-16 and are only valid for the duration of the message being
		// handled, so they are copied out immediately.
		std::optional<std::string> ImeStringUtf8(HIMC himc, DWORD index)
		{
			auto const bytes = ImmGetCompositionStringW(himc, index, nullptr, 0);
			if (bytes < 0)
				return std::nullopt;

			std::wstring wide(static_cast<std::size_t>(bytes) / sizeof(wchar_t), L'\0');
			if (bytes > 0 && ImmGetCompositionStringW(himc, index, wide.data(), static_cast<DWORD>(bytes)) < 0)
				return std::nullopt;

			std::string utf8;
			if (!Utf16ToUtf8(wide, utf8))
				return std::nullopt; // an IME reporting ill-formed UTF-16 is discarded rather than substituted

			return utf8;
		}

		// The UTF-8 byte offset corresponding to a UTF-16 code-unit index into the same text. Each
		// code point is counted as the one or two UTF-16 units it would occupy, so the mapping is
		// exact for astral characters; an index past the end clamps to the end of the string.
		std::uint32_t Utf16IndexToUtf8Offset(std::string_view utf8, int utf16_index)
		{
			if (utf16_index <= 0)
				return 0;

			auto units = 0;
			auto offset = std::uint32_t{};
			while (offset < utf8.size())
			{
				if (units >= utf16_index)
					break;

				char32_t cp;
				std::uint32_t length;
				if (!Utf8Decode(utf8, offset, cp, length))
					break;

				units += cp >= 0x10000u ? 2 : 1;
				offset += length;
			}
			return offset;
		}

		// The IME's cursor position within the composition string, converted from a UTF-16 code
		// unit index to the UTF-8 byte offset the edit model uses. An out-of-range index (some IMEs
		// report one past the end) clamps to the end of the string.
		std::uint32_t ImeCursorUtf8Offset(HIMC himc, std::string_view composition_utf8)
		{
			return Utf16IndexToUtf8Offset(composition_utf8, ImmGetCompositionStringW(himc, GCS_CURSORPOS, nullptr, 0));
		}

		// The IME's active conversion clause as UTF-8 offsets into 'composition_utf8', or nullopt
		// when the IME reports no target clause. GCS_COMPATTR carries one attribute byte per UTF-16
		// code unit; the run marked ATTR_TARGET_CONVERTED/ATTR_TARGET_NOTCONVERTED is the segment
		// the user is currently converting, which the edit model shows as the selection so the
		// active clause is visually distinct from the rest of the composition. GCS_COMPCLAUSE, when
		// present, gives the authoritative clause boundaries, so the attribute run is widened to
		// the clause containing it rather than trusted byte-for-byte.
		std::optional<std::pair<std::uint32_t, std::uint32_t>> ImeTargetClauseUtf8(HIMC himc, std::string_view composition_utf8)
		{
			auto const attr_bytes = ImmGetCompositionStringW(himc, GCS_COMPATTR, nullptr, 0);
			if (attr_bytes <= 0)
				return std::nullopt;

			std::vector<std::uint8_t> attrs(static_cast<std::size_t>(attr_bytes), 0);
			if (ImmGetCompositionStringW(himc, GCS_COMPATTR, attrs.data(), static_cast<DWORD>(attr_bytes)) < 0)
				return std::nullopt;

			auto is_target = [](std::uint8_t a)
			{
				return a == ATTR_TARGET_CONVERTED || a == ATTR_TARGET_NOTCONVERTED;
			};

			auto first = std::size_t(0);
			while (first != attrs.size() && !is_target(attrs[first]))
				++first;

			if (first == attrs.size())
				return std::nullopt;

			auto last = first;
			while (last != attrs.size() && is_target(attrs[last]))
				++last;

			auto start_u16 = static_cast<int>(first);
			auto end_u16 = static_cast<int>(last);

			// Snap outwards to the enclosing clause when the IME published a clause table, so a
			// clause whose attributes are only partially marked still selects as a whole segment.
			auto const clause_bytes = ImmGetCompositionStringW(himc, GCS_COMPCLAUSE, nullptr, 0);
			if (clause_bytes > 0 && (static_cast<std::size_t>(clause_bytes) % sizeof(DWORD)) == 0)
			{
				std::vector<DWORD> clauses(static_cast<std::size_t>(clause_bytes) / sizeof(DWORD), 0);
				if (ImmGetCompositionStringW(himc, GCS_COMPCLAUSE, clauses.data(), static_cast<DWORD>(clause_bytes)) >= 0)
				{
					for (auto i = std::size_t(0); i + 1 < clauses.size(); ++i)
					{
						auto const lo = static_cast<int>(clauses[i]);
						auto const hi = static_cast<int>(clauses[i + 1]);
						if (lo <= start_u16 && start_u16 < hi)
						{
							start_u16 = std::min(start_u16, lo);
							end_u16 = std::max(end_u16, hi);
							break;
						}
					}
				}
			}

			auto const start = Utf16IndexToUtf8Offset(composition_utf8, start_u16);
			auto const end = Utf16IndexToUtf8Offset(composition_utf8, end_u16);
			if (end <= start)
				return std::nullopt;

			return std::make_pair(start, end);
		}
	}

	TranslatedMessage TranslateWindowMessage(HWND hwnd, UINT msg, WPARAM wparam, LPARAM lparam, ViewportState const& viewport, double time_ms, bool editable_focus, Win32InputTranslatorState& translator_state)
	{
		switch (msg)
		{
			case WM_MOUSEMOVE:
			{
				return MakePointerMessage(EInputKind::PointerMove, EPointerButton::None, wparam, lparam, viewport, time_ms);
			}
			case WM_LBUTTONDOWN:
			{
				return MakePointerMessage(EInputKind::PointerButtonDown, EPointerButton::Left, wparam, lparam, viewport, time_ms);
			}
			case WM_LBUTTONUP:
			{
				return MakePointerMessage(EInputKind::PointerButtonUp, EPointerButton::Left, wparam, lparam, viewport, time_ms);
			}
			case WM_RBUTTONDOWN:
			{
				return MakePointerMessage(EInputKind::PointerButtonDown, EPointerButton::Right, wparam, lparam, viewport, time_ms);
			}
			case WM_RBUTTONUP:
			{
				return MakePointerMessage(EInputKind::PointerButtonUp, EPointerButton::Right, wparam, lparam, viewport, time_ms);
			}
			case WM_MBUTTONDOWN:
			{
				return MakePointerMessage(EInputKind::PointerButtonDown, EPointerButton::Middle, wparam, lparam, viewport, time_ms);
			}
			case WM_MBUTTONUP:
			{
				return MakePointerMessage(EInputKind::PointerButtonUp, EPointerButton::Middle, wparam, lparam, viewport, time_ms);
			}
			case WM_MOUSEWHEEL:
			{
				// WM_MOUSEWHEEL delivers screen-relative coordinates (unlike every other mouse
				// message), so they must be mapped to client space before the usual DIP conversion.
				POINT pt{ static_cast<LONG>(static_cast<short>(LOWORD(lparam))), static_cast<LONG>(static_cast<short>(HIWORD(lparam))) };
				ScreenToClient(hwnd, &pt);
				auto dip = ClientPixelsToDip(viewport, static_cast<float>(pt.x), static_cast<float>(pt.y));

				auto input = MakeBase(EInputKind::PointerWheel, time_ms);
				input.pointer_x = dip.x;
				input.pointer_y = dip.y;
				input.button_mask = MouseWParamToButtonMask(wparam);
				input.modifiers = CurrentModifiers();
				input.wheel_delta = static_cast<float>(static_cast<short>(HIWORD(wparam))) / static_cast<float>(WHEEL_DELTA);
				return MakeSingle(input);
			}
			case WM_KEYDOWN:
			case WM_SYSKEYDOWN:
			{
				auto input = MakeBase(EInputKind::KeyDown, time_ms);
				input.vk = static_cast<std::int32_t>(wparam);
				input.modifiers = CurrentModifiers();
				return MakeSingle(input);
			}
			case WM_KEYUP:
			case WM_SYSKEYUP:
			{
				auto input = MakeBase(EInputKind::KeyUp, time_ms);
				input.vk = static_cast<std::int32_t>(wparam);
				input.modifiers = CurrentModifiers();
				return MakeSingle(input);
			}
			case WM_DEADCHAR:
			case WM_SYSDEADCHAR:
			{
				// A dead key produces no text of its own: Windows delivers the composed character
				// in a following WM_CHAR (or, if the next key cannot combine, two WM_CHARs). The
				// message is consumed so the accent is not inserted literally, and the flag is
				// recorded purely so the next WM_CHAR can be recognised as its result.
				translator_state.pending_dead_key = 1;
				return TranslatedMessage{ .recognised = 1, .handled = 1, .result = 0, .place_ime_windows = 0, .inputs = {} };
			}
			case WM_CHAR:
			{
				// WM_CHAR delivers UTF-16 code units one at a time; a surrogate pair arrives as two
				// consecutive messages, so the high half must be buffered until its low half
				// arrives before a single UTF-32 codepoint can be produced.
				translator_state.pending_dead_key = 0;
				auto unit = static_cast<std::uint16_t>(wparam);
				if (unit >= 0xD800u && unit <= 0xDBFFu)
				{
					// A second high surrogate replaces the first: the earlier one can never be
					// completed now, and keeping it would pair it with the wrong low half.
					translator_state.pending_high_surrogate = unit;
					return TranslatedMessage{ .recognised = 1, .handled = 1, .result = 0, .place_ime_windows = 0, .inputs = {} };
				}

				// Whatever this unit turns out to be, it ends any pending pair: either it completes
				// it, or it orphans it. The orphan is dropped rather than emitted, because a lone
				// surrogate is not a scalar value and cannot be encoded as UTF-8 at all.
				auto const pending_high = translator_state.pending_high_surrogate;
				translator_state.pending_high_surrogate = 0;

				auto codepoint = std::uint32_t{ unit };
				if (unit >= 0xDC00u && unit <= 0xDFFFu)
				{
					// An unpaired low surrogate is equally unencodable, so it is discarded too
					// rather than inserted as U+DCxx.
					if (pending_high == 0)
						return TranslatedMessage{ .recognised = 1, .handled = 1, .result = 0, .place_ime_windows = 0, .inputs = {} };

					codepoint = 0x10000u + ((static_cast<std::uint32_t>(pending_high) - 0xD800u) << 10) + (static_cast<std::uint32_t>(unit) - 0xDC00u);
				}

				auto input = MakeBase(EInputKind::Char, time_ms);
				input.char_code = codepoint;
				input.modifiers = CurrentModifiers();
				return MakeSingle(input);
			}
			case WM_UNICHAR:
			{
				// WM_UNICHAR carries a full UTF-32 scalar value rather than a UTF-16 unit. Windows
				// probes support by sending UNICHAR_NOCHAR first and requires TRUE in reply, which
				// is what makes the window advertise itself as Unicode-capable; answering anything
				// else silently downgrades every subsequent character to the ANSI code page.
				if (wparam == UNICODE_NOCHAR)
					return TranslatedMessage{ .recognised = 1, .handled = 1, .result = TRUE, .place_ime_windows = 0, .inputs = {} };

				auto const codepoint = static_cast<std::uint32_t>(wparam);
				if (codepoint > 0x10FFFFu || (codepoint >= 0xD800u && codepoint <= 0xDFFFu))
					return TranslatedMessage{ .recognised = 1, .handled = 1, .result = 0, .place_ime_windows = 0, .inputs = {} };

				auto input = MakeBase(EInputKind::Char, time_ms);
				input.char_code = codepoint;
				input.modifiers = CurrentModifiers();
				auto out = MakeSingle(input);
				out.handled = 1;
				return out;
			}
			case WM_IME_STARTCOMPOSITION:
			{
				// The IME is opening a composition. With no editable target there is nothing to
				// compose into, so the message is left to the default window procedure rather than
				// reported as consumed UI input; the IME then behaves as it would for any other
				// non-editable window.
				translator_state.ime_composition_open = 1;
				if (!editable_focus)
					return TranslatedMessage{ .recognised = 1, .handled = 0, .result = 0, .place_ime_windows = 0, .inputs = {} };

				// The composition is drawn by this module, so the default IME window must never
				// see this message or it will draw its own composition string on top.
				translator_state.composition_active = 1;
				auto input = MakeBase(EInputKind::CompositionStart, time_ms);
				input.modifiers = CurrentModifiers();
				auto out = MakeSingle(input);
				out.handled = 1;
				out.place_ime_windows = 1;
				return out;
			}
			case WM_IME_COMPOSITION:
			{
				if (!editable_focus)
					return TranslatedMessage{ .recognised = 1, .handled = 0, .result = 0, .place_ime_windows = 0, .inputs = {} };

				TranslatedMessage out{ .recognised = 1, .handled = 1, .result = 0, .place_ime_windows = 1, .inputs = {} };

				auto himc = ImmGetContext(hwnd);
				if (himc == nullptr)
					return out;

				// The engine may have cancelled the composition on its own - a click elsewhere, or
				// Tab - while the IME kept converting. Rather than rejecting the follow-up message,
				// a fresh CompositionStart is prepended so the records stay well-ordered and the
				// state machine resynchronises onto whatever the IME is now producing.
				auto reopen = [&]()
				{
					if (translator_state.composition_active != 0)
						return;

					auto start = MakeBase(EInputKind::CompositionStart, time_ms);
					start.modifiers = CurrentModifiers();
					out.inputs.push_back(TranslatedInput{ .input = start, .text = {}, .caret = 0, .selection_start = 0, .selection_end = 0 });
					translator_state.composition_active = 1;
				};

				// A single message can carry both a finished result string and the start of the
				// next composition, so the commit is emitted before the update and the order is
				// preserved by the caller.
				if ((lparam & GCS_RESULTSTR) != 0)
				{
					if (auto result = ImeStringUtf8(himc, GCS_RESULTSTR); result.has_value())
					{
						reopen();
						auto input = MakeBase(EInputKind::CompositionCommit, time_ms);
						input.modifiers = CurrentModifiers();
						out.inputs.push_back(TranslatedInput{ .input = input, .text = std::move(*result), .caret = 0, .selection_start = 0, .selection_end = 0 });
						translator_state.composition_active = 0;
					}
				}
				if ((lparam & GCS_COMPSTR) != 0)
				{
					if (auto composition = ImeStringUtf8(himc, GCS_COMPSTR); composition.has_value())
					{
						if (!composition->empty())
							reopen();

						if (translator_state.composition_active != 0)
						{
							// The caret is the IME's insertion point; the selection is its active
							// conversion clause when it publishes one, so the segment being
							// converted is highlighted rather than the whole composition.
							auto const caret = ImeCursorUtf8Offset(himc, *composition);
							auto const clause = ImeTargetClauseUtf8(himc, *composition);
							auto const sel_start = clause.has_value() ? clause->first : caret;
							auto const sel_end = clause.has_value() ? clause->second : caret;

							auto input = MakeBase(EInputKind::CompositionUpdate, time_ms);
							input.modifiers = CurrentModifiers();
							out.inputs.push_back(TranslatedInput{ .input = input, .text = std::move(*composition), .caret = caret, .selection_start = sel_start, .selection_end = sel_end });
						}
					}
				}

				ImmReleaseContext(hwnd, himc);
				return out;
			}
			case WM_IME_ENDCOMPOSITION:
			{
				// The IME ends a composition after both a commit and an abandonment. Only the
				// latter still has a live composition here, so only that case cancels; a commit
				// already cleared the flag when its result string was translated.
				translator_state.ime_composition_open = 0;
				if (!editable_focus)
					return TranslatedMessage{ .recognised = 1, .handled = 0, .result = 0, .place_ime_windows = 0, .inputs = {} };

				TranslatedMessage out{ .recognised = 1, .handled = 1, .result = 0, .place_ime_windows = 0, .inputs = {} };
				if (translator_state.composition_active != 0)
				{
					auto input = MakeBase(EInputKind::CompositionCancel, time_ms);
					input.modifiers = CurrentModifiers();
					out.inputs.push_back(TranslatedInput{ .input = input, .text = {}, .caret = 0, .selection_start = 0, .selection_end = 0 });
					translator_state.composition_active = 0;
				}
				return out;
			}
			case WM_IME_CHAR:
			{
				// Some IMEs send the committed characters again as WM_IME_CHAR after the result
				// string has already been delivered by WM_IME_COMPOSITION. The message is consumed
				// without producing records so the text is not inserted twice, and so that letting
				// it reach the default window procedure cannot turn it into a WM_CHAR either.
				return TranslatedMessage{ .recognised = 1, .handled = 1, .result = 0, .place_ime_windows = 0, .inputs = {} };
			}
			case WM_IME_SETCONTEXT:
			{
				// This module draws the composition string itself, so the IME's own composition
				// window is suppressed by clearing its bit before the default handler configures
				// the UI. The remaining bits (the candidate windows) are left alone, since the IME
				// still owns candidate presentation; Win32PlaceImeWindows only moves them.
				auto const flags = static_cast<LPARAM>(lparam & ~static_cast<LPARAM>(ISC_SHOWUICOMPOSITIONWINDOW));
				return TranslatedMessage{
					.recognised = 1,
					.handled = 1,
					.result = DefWindowProcW(hwnd, msg, wparam, flags),
					.place_ime_windows = 0,
					.inputs = {},
				};
			}
			case WM_SETFOCUS:
			{
				return MakeSingle(MakeBase(EInputKind::FocusGained, time_ms));
			}
			case WM_KILLFOCUS:
			{
				// Focus is leaving the window, so any composition dies with it; the state machine
				// cancels its own composition when it processes the FocusLost record.
				translator_state.composition_active = 0;
				translator_state.ime_composition_open = 0;
				translator_state.pending_high_surrogate = 0;
				translator_state.pending_dead_key = 0;
				return MakeSingle(MakeBase(EInputKind::FocusLost, time_ms));
			}
			default:
			{
				return MakeUnrecognised();
			}
		}
	}

	void Win32CancelImeComposition(HWND hwnd)
	{
		auto himc = ImmGetContext(hwnd);
		if (himc == nullptr)
			return;

		// CPS_CANCEL discards the composition string outright; CPS_COMPLETE would commit text the
		// engine has already decided to abandon.
		ImmNotifyIME(himc, NI_COMPOSITIONSTR, CPS_CANCEL, 0);
		ImmReleaseContext(hwnd, himc);
	}

	void Win32ApplyCaptureTransition(HWND hwnd, ControlId previous_captured_id, ControlId current_captured_id)
	{
		if (previous_captured_id == current_captured_id)
			return;

		if (current_captured_id != 0)
			SetCapture(hwnd);
		else
			ReleaseCapture();
	}

	void Win32PlaceImeWindows(HWND hwnd, ViewportState const& viewport, Rect const& caret_dip)
	{
		auto himc = ImmGetContext(hwnd);
		if (himc == nullptr)
			return;

		// Invert the client-pixels-to-DIP mapping the translator applies to pointer positions, so
		// the IME's windows land on the caret the user is actually looking at.
		auto const dpi_scale = viewport.dpi / 96.0f;
		auto const ratio_x = viewport.target_width_px != 0 ? static_cast<float>(viewport.client_width_px) / static_cast<float>(viewport.target_width_px) : 1.0f;
		auto const ratio_y = viewport.target_height_px != 0 ? static_cast<float>(viewport.client_height_px) / static_cast<float>(viewport.target_height_px) : 1.0f;
		auto const to_client_x = [&](float dip) { return static_cast<LONG>((dip * dpi_scale + viewport.viewport_x_px) * ratio_x); };
		auto const to_client_y = [&](float dip) { return static_cast<LONG>((dip * dpi_scale + viewport.viewport_y_px) * ratio_y); };

		auto const left = to_client_x(caret_dip.x);
		auto const top = to_client_y(caret_dip.y);
		auto const right = to_client_x(caret_dip.x + caret_dip.w);
		auto const bottom = to_client_y(caret_dip.y + caret_dip.h);

		// CFS_POINT anchors the composition window at the caret; the candidate list is given the
		// caret's full rectangle as its exclusion zone so it never covers the text being typed.
		COMPOSITIONFORM composition{ .dwStyle = CFS_POINT, .ptCurrentPos = POINT{ left, top }, .rcArea = RECT{} };
		ImmSetCompositionWindow(himc, &composition);

		CANDIDATEFORM candidate{ .dwIndex = 0, .dwStyle = CFS_EXCLUDE, .ptCurrentPos = POINT{ left, bottom }, .rcArea = RECT{ left, top, right, bottom } };
		ImmSetCandidateWindow(himc, &candidate);

		ImmReleaseContext(hwnd, himc);
	}
}
