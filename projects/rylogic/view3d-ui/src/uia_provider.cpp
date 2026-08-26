//*********************************************
// View3DUI
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// COM provider implementations for the Windows UI Automation bridge; see uia_provider.h for the
// public surface and uia_bridge.h for the threading contract.
//
// Every object here reads only an immutable published snapshot and performs every mutation through
// UiaSharedState::InvokeAction, which marshals to the owner thread. None of them holds a UiEngine
// or an HWND of its own, so all of them remain safe to call after the context has been destroyed.
#include "uia_provider.h"
#include "uia_bridge.h"
#include "text_unicode.h"
#include "pr/view3d-ui/engine.h"

// UiaHostProviderFromHwnd and UiaGetReservedNotSupportedValue live in uiautomationcore.lib. The
// dependency is declared next to the code that needs it, matching dwrite.lib in text_shaper.cpp and
// imm32.lib in win32_input.cpp, so no project file has to know about it.
#pragma comment(lib, "uiautomationcore.lib")

namespace pr::view3d::ui
{
	namespace
	{
		// Run one COM entry point, converting every failure into an HRESULT. C++ exceptions must
		// never cross a COM vtable, so this is a sanctioned no-throw ABI boundary: each arm maps to
		// a specific status, and no arm can turn a failure into a success-shaped result.
		template <typename Fn> HRESULT ComGuard(Fn fn) noexcept
		{
			try
			{
				return fn();
			}
			catch (EngineException const& ex)
			{
				return UiaStatusFromEngineStatus(ex.Status());
			}
			catch (std::bad_alloc const&)
			{
				return E_OUTOFMEMORY;
			}
			catch (std::length_error const&)
			{
				return E_OUTOFMEMORY;
			}
			catch (std::exception const&)
			{
				return E_FAIL;
			}
			catch (...)
			{
				return E_FAIL;
			}
		}

		// A VARIANT the client must treat as "this provider has no opinion", which makes UI
		// Automation fall back to its own default for the property.
		VARIANT EmptyVariant()
		{
			auto value = VARIANT{};
			VariantInit(&value);
			return value;
		}

		// A VT_BOOL VARIANT.
		VARIANT BoolVariant(bool state)
		{
			auto value = EmptyVariant();
			value.vt = VT_BOOL;
			value.boolVal = state ? VARIANT_TRUE : VARIANT_FALSE;
			return value;
		}

		// A VT_I4 VARIANT.
		VARIANT I4Variant(std::int32_t number)
		{
			auto value = EmptyVariant();
			value.vt = VT_I4;
			value.lVal = number;
			return value;
		}

		// A VT_BSTR VARIANT, degrading to VT_EMPTY when the string cannot be allocated; a property
		// read has no way to report failure, and an absent property is the truthful degradation.
		VARIANT BstrVariant(std::wstring const& text)
		{
			auto const bstr = SysAllocStringLen(text.c_str(), static_cast<UINT>(text.size()));
			if (bstr == nullptr)
				return EmptyVariant();

			auto value = EmptyVariant();
			value.vt = VT_BSTR;
			value.bstrVal = bstr;
			return value;
		}

		// Copy 'text' into a caller-owned BSTR out-parameter.
		HRESULT AssignBstr(std::wstring const& text, BSTR* out)
		{
			if (out == nullptr)
				return E_POINTER;

			*out = SysAllocStringLen(text.c_str(), static_cast<UINT>(text.size()));
			return *out != nullptr ? S_OK : E_OUTOFMEMORY;
		}

		// The UI Automation control type for one semantic role. Root and Panel are containers with
		// no interactive behaviour of their own, so they map to Pane and Group respectively.
		std::int32_t ControlTypeOf(EControlType role)
		{
			switch (role)
			{
				case EControlType::Root: { return UIA_PaneControlTypeId; }
				case EControlType::Panel: { return UIA_GroupControlTypeId; }
				case EControlType::Text: { return UIA_TextControlTypeId; }
				case EControlType::TextBox: { return UIA_EditControlTypeId; }
				case EControlType::Button: { return UIA_ButtonControlTypeId; }
				default: throw EngineException(EStatus::InvalidArgument, std::format("unknown control type {}", static_cast<std::int32_t>(role)));
			}
		}

		// True when the node should appear in a client's content view. Containers are control-view
		// only so a screen reader is not made to announce structural scaffolding as content.
		bool IsContentElement(EControlType role)
		{
			switch (role)
			{
				case EControlType::Root:
				case EControlType::Panel: { return false; }
				case EControlType::Text:
				case EControlType::TextBox:
				case EControlType::Button: { return true; }
				default: throw EngineException(EStatus::InvalidArgument, std::format("unknown control type {}", static_cast<std::int32_t>(role)));
			}
		}

		// The stable automation id of a node. Derived only from ControlId, which the retained model
		// guarantees is stable for the lifetime of the control, so it survives every relayout.
		std::wstring AutomationIdOf(ControlId id)
		{
			return std::format(L"view3dui:{}", id);
		}

		// Build the runtime id of a node: the reserved prefix that scopes the id to this fragment's
		// HWND, followed by the 64-bit ControlId split into two 32-bit halves.
		HRESULT MakeRuntimeId(ControlId id, SAFEARRAY** out)
		{
			if (out == nullptr)
				return E_POINTER;

			*out = nullptr;
			auto* array = SafeArrayCreateVector(VT_I4, 0, 3);
			if (array == nullptr)
				return E_OUTOFMEMORY;

			std::int32_t const parts[3] = {
				UiaAppendRuntimeId,
				static_cast<std::int32_t>(static_cast<std::uint32_t>(id >> 32)),
				static_cast<std::int32_t>(static_cast<std::uint32_t>(id & 0xFFFFFFFFu)),
			};
			for (auto i = LONG{}; i != 3; ++i)
			{
				auto const hr = SafeArrayPutElement(array, &i, const_cast<std::int32_t*>(&parts[i]));
				if (FAILED(hr))
				{
					SafeArrayDestroy(array);
					return hr;
				}
			}

			*out = array;
			return S_OK;
		}

		// An empty SAFEARRAY of the given element type, used wherever "no results" must still be a
		// well-formed array rather than a null the client has to special-case.
		SAFEARRAY* EmptyArray(VARTYPE element_type)
		{
			return SafeArrayCreateVector(element_type, 0, 0);
		}

		// Screen-space physical-pixel bounds of 'node'. Offscreen nodes (including every descendant
		// of a culled world root) report an all-zero rectangle, which is UI Automation's encoding
		// for "this element has no on-screen location".
		UiaRect NodeScreenRect(UiaSharedState const& shared, UiaSnapshot const& snapshot, UiaNode const& node)
		{
			auto rect = UiaRect{ 0.0, 0.0, 0.0, 0.0 };
			if (node.HasState(ESemanticState::Offscreen) || !node.HasState(ESemanticState::Visible))
				return rect;

			auto const hwnd = shared.Window();
			if (hwnd == nullptr || IsWindow(hwnd) == FALSE)
				return rect;

			// Client pixels are relative to the window; UI Automation wants desktop pixels, so the
			// client origin is added last. ClientToScreen is safe to call for another thread's
			// window because it only reads shared window state.
			auto const client = UiaClientPixelRect(snapshot.m_viewport, node.bounds_dip);
			auto origin = POINT{ 0, 0 };
			if (ClientToScreen(hwnd, &origin) == FALSE)
				return rect;

			rect.left = static_cast<double>(client.x) + origin.x;
			rect.top = static_cast<double>(client.y) + origin.y;
			rect.width = static_cast<double>(client.w);
			rect.height = static_cast<double>(client.h);
			return rect;
		}

		// ASCII-only case folding, used by ITextRangeProvider::FindText's ignore-case mode. Folding
		// the full repertoire would need case tables this module does not carry, so the supported
		// scope is stated here rather than approximated.
		std::string FoldAscii(std::string_view text)
		{
			auto folded = std::string(text);
			for (auto& ch : folded)
			{
				if (ch >= 'A' && ch <= 'Z')
					ch = static_cast<char>(ch - 'A' + 'a');
			}
			return folded;
		}

		class UiaElementProvider;

		// Create a provider for one semantic node, as an owned fragment pointer.
		IRawElementProviderFragment* MakeFragment(std::shared_ptr<UiaSharedState> const& shared, ControlId id);

		// Create the fragment root, as an owned fragment-root pointer.
		IRawElementProviderFragmentRoot* MakeFragmentRoot(std::shared_ptr<UiaSharedState> const& shared);

		// Create the fragment root, as an owned fragment pointer. IRawElementProviderFragmentRoot
		// does not derive from IRawElementProviderFragment, so navigating to the root needs its own
		// helper rather than a cast of the one above.
		IRawElementProviderFragment* MakeRootFragment(std::shared_ptr<UiaSharedState> const& shared);

		// Reference counting and shared-state access common to every provider in this file.
		class UiaProviderBase
		{
		protected:

			std::atomic<ULONG> m_ref;
			std::shared_ptr<UiaSharedState> m_shared;

			explicit UiaProviderBase(std::shared_ptr<UiaSharedState> shared)
				: m_ref(1)
				, m_shared(std::move(shared))
			{}
			virtual ~UiaProviderBase() = default;

			// The published snapshot, or a StaleHandle failure when the context has gone away or
			// has not published yet. Callers map that status to UIA_E_ELEMENTNOTAVAILABLE.
			std::shared_ptr<UiaSnapshot const> RequireSnapshot() const
			{
				auto snapshot = m_shared->Snapshot();
				if (snapshot == nullptr)
					throw EngineException(EStatus::StaleHandle, "the UI context is no longer available");

				return snapshot;
			}
		};

		// A text range over one TextBox's value text, addressed in UTF-8 byte offsets so it shares
		// the M9 edit model's units exactly. Offsets are re-validated against the current snapshot
		// on every use because the text they index into can change between calls.
		class UiaTextRangeProvider
			: public UiaProviderBase
			, public ITextRangeProvider
		{
			ControlId m_id;
			std::uint32_t m_start;
			std::uint32_t m_end;

			// The node this range belongs to, or a StaleHandle failure when it has gone away or is
			// no longer an editable control.
			UiaNode const& RequireNode(std::shared_ptr<UiaSnapshot const> const& snapshot) const
			{
				auto const* node = snapshot->Find(m_id);
				if (node == nullptr || node->role != EControlType::TextBox)
					throw EngineException(EStatus::StaleHandle, "the text control is no longer available");

				return *node;
			}

			// Re-clamp the stored endpoints against 'text' and snap them onto grapheme boundaries,
			// so a range captured before an edit can never index into the middle of a cluster.
			void Normalize(std::string_view text, std::uint32_t& start, std::uint32_t& end) const
			{
				auto const size = static_cast<std::uint32_t>(text.size());
				start = ClampToGraphemeBoundary(text, std::min(m_start, size));
				end = ClampToGraphemeBoundary(text, std::min(m_end, size));
				if (start > end)
					std::swap(start, end);
			}

			// Move 'offset' by 'count' units of 'unit', returning how many units were actually
			// moved. Character units are grapheme clusters; every larger unit degrades to the whole
			// value because a TextBox holds exactly one line with uniform formatting.
			std::int32_t MoveOffset(std::string_view text, TextUnit unit, std::int32_t count, std::uint32_t& offset) const
			{
				auto const size = static_cast<std::uint32_t>(text.size());
				auto moved = std::int32_t{};
				switch (unit)
				{
					case TextUnit_Character:
					{
						for (; moved != count && count > 0; ++moved)
						{
							auto const next = NextGraphemeBoundary(text, offset);
							if (next == offset)
								break;

							offset = next;
						}
						for (; moved != count && count < 0; --moved)
						{
							auto const prev = PrevGraphemeBoundary(text, offset);
							if (prev == offset)
								break;

							offset = prev;
						}
						return moved;
					}
					case TextUnit_Word:
					{
						for (; moved != count && count > 0; ++moved)
						{
							auto const next = NextWordBoundary(text, offset);
							if (next == offset)
								break;

							offset = next;
						}
						for (; moved != count && count < 0; --moved)
						{
							auto const prev = PrevWordBoundary(text, offset);
							if (prev == offset)
								break;

							offset = prev;
						}
						return moved;
					}
					case TextUnit_Format:
					case TextUnit_Line:
					case TextUnit_Paragraph:
					case TextUnit_Page:
					case TextUnit_Document:
					{
						// One unit exists, so at most one move is possible in either direction.
						if (count > 0 && offset != size)
						{
							offset = size;
							return 1;
						}
						if (count < 0 && offset != 0)
						{
							offset = 0;
							return -1;
						}
						return 0;
					}
					// TextUnit arrives from the client, so an unrecognised value is a caller error
					// rather than a missing case in this module.
					default: throw EngineException(EStatus::UnsupportedFeature, "unsupported text unit");
				}
			}

		public:

			UiaTextRangeProvider(std::shared_ptr<UiaSharedState> shared, ControlId id, std::uint32_t start, std::uint32_t end)
				: UiaProviderBase(std::move(shared))
				, m_id(id)
				, m_start(start)
				, m_end(end)
			{}

			HRESULT STDMETHODCALLTYPE QueryInterface(REFIID riid, void** obj) override
			{
				if (obj == nullptr)
					return E_POINTER;

				*obj = nullptr;
				if (riid == __uuidof(IUnknown) || riid == __uuidof(ITextRangeProvider))
					*obj = static_cast<ITextRangeProvider*>(this);
				else
					return E_NOINTERFACE;

				AddRef();
				return S_OK;
			}
			ULONG STDMETHODCALLTYPE AddRef() override
			{
				return m_ref.fetch_add(1, std::memory_order_relaxed) + 1;
			}
			ULONG STDMETHODCALLTYPE Release() override
			{
				// A range is never entered in the shared registry, so the null key means "just make
				// the final transition"; routing every provider through one path keeps the release
				// rule uniform.
				auto remaining = ULONG{};
				if (m_shared->ReleaseProvider(nullptr, m_ref, remaining))
					delete this;

				return remaining;
			}

			HRESULT STDMETHODCALLTYPE Clone(ITextRangeProvider** ret) override
			{
				return ComGuard([&]() -> HRESULT
				{
					if (ret == nullptr)
						return E_POINTER;

					*ret = new UiaTextRangeProvider(m_shared, m_id, m_start, m_end);
					return S_OK;
				});
			}
			HRESULT STDMETHODCALLTYPE Compare(ITextRangeProvider* other, BOOL* ret) override
			{
				return ComGuard([&]() -> HRESULT
				{
					if (ret == nullptr)
						return E_POINTER;

					*ret = FALSE;
					auto const* peer = dynamic_cast<UiaTextRangeProvider*>(other);
					if (peer == nullptr || peer->m_shared != m_shared)
						return UIA_E_INVALIDOPERATION;

					// Ranges over different controls are simply not equal; only a foreign range is
					// an invalid comparison.
					if (peer->m_id != m_id)
						return S_OK;

					// Both sides are normalized against the current text before comparing, so two
					// ranges that address the same span answer the same way regardless of the
					// offsets they were created with or of edits made since.
					auto const snapshot = RequireSnapshot();
					auto const& node = RequireNode(snapshot);
					auto const text = std::string_view(node.value_utf8);
					auto lhs_start = std::uint32_t{};
					auto lhs_end = std::uint32_t{};
					Normalize(text, lhs_start, lhs_end);
					auto rhs_start = std::uint32_t{};
					auto rhs_end = std::uint32_t{};
					peer->Normalize(text, rhs_start, rhs_end);

					*ret = lhs_start == rhs_start && lhs_end == rhs_end ? TRUE : FALSE;
					return S_OK;
				});
			}
			HRESULT STDMETHODCALLTYPE CompareEndpoints(TextPatternRangeEndpoint endpoint, ITextRangeProvider* other, TextPatternRangeEndpoint other_endpoint, int* ret) override
			{
				return ComGuard([&]() -> HRESULT
				{
					if (ret == nullptr)
						return E_POINTER;

					*ret = 0;
					auto const* peer = dynamic_cast<UiaTextRangeProvider*>(other);
					if (peer == nullptr || peer->m_shared != m_shared || peer->m_id != m_id)
						return UIA_E_INVALIDOPERATION;

					// Endpoints are only comparable once both ranges have been resolved against the
					// text they currently index into; raw stored offsets can be stale or mid-cluster.
					auto const snapshot = RequireSnapshot();
					auto const& node = RequireNode(snapshot);
					auto const text = std::string_view(node.value_utf8);
					auto lhs_start = std::uint32_t{};
					auto lhs_end = std::uint32_t{};
					Normalize(text, lhs_start, lhs_end);
					auto rhs_start = std::uint32_t{};
					auto rhs_end = std::uint32_t{};
					peer->Normalize(text, rhs_start, rhs_end);

					auto const lhs = endpoint == TextPatternRangeEndpoint_Start ? lhs_start : lhs_end;
					auto const rhs = other_endpoint == TextPatternRangeEndpoint_Start ? rhs_start : rhs_end;
					*ret = lhs < rhs ? -1 : lhs > rhs ? 1 : 0;
					return S_OK;
				});
			}
			HRESULT STDMETHODCALLTYPE ExpandToEnclosingUnit(TextUnit unit) override
			{
				return ComGuard([&]() -> HRESULT
				{
					auto const snapshot = RequireSnapshot();
					auto const& node = RequireNode(snapshot);
					auto const text = std::string_view(node.value_utf8);
					auto start = std::uint32_t{};
					auto end = std::uint32_t{};
					Normalize(text, start, end);

					switch (unit)
					{
						case TextUnit_Character:
						{
							// A degenerate range grows to cover the cluster it sits before; a
							// non-degenerate one already spans whole clusters after Normalize.
							if (start == end)
								end = NextGraphemeBoundary(text, start);

							break;
						}
						case TextUnit_Word:
						{
							// Stepping one cluster forward first puts the offset inside the word it
							// sits at the start of, so the backward search returns that word's start
							// rather than the previous word's.
							start = ClampToGraphemeBoundary(text, PrevWordBoundary(text, NextGraphemeBoundary(text, start)));
							end = NextWordBoundary(text, start);
							break;
						}
						case TextUnit_Format:
						case TextUnit_Line:
						case TextUnit_Paragraph:
						case TextUnit_Page:
						case TextUnit_Document:
						{
							start = 0;
							end = static_cast<std::uint32_t>(text.size());
							break;
						}
						default: return UIA_E_NOTSUPPORTED;
					}

					m_start = start;
					m_end = end;
					return S_OK;
				});
			}
			HRESULT STDMETHODCALLTYPE FindAttribute(TEXTATTRIBUTEID, VARIANT, BOOL, ITextRangeProvider** ret) override
			{
				return ComGuard([&]() -> HRESULT
				{
					if (ret == nullptr)
						return E_POINTER;

					// No attribute varies within a single-format value, so no sub-range can ever be
					// the answer. A null result with S_OK is UI Automation's "not found".
					*ret = nullptr;
					return S_OK;
				});
			}
			HRESULT STDMETHODCALLTYPE GetAttributeValue(TEXTATTRIBUTEID attribute, VARIANT* ret) override
			{
				return ComGuard([&]() -> HRESULT
				{
					if (ret == nullptr)
						return E_POINTER;

					auto const snapshot = RequireSnapshot();
					auto const& node = RequireNode(snapshot);
					if (attribute == UIA_IsReadOnlyAttributeId)
					{
						*ret = BoolVariant(!node.HasState(ESemanticState::Enabled));
						return S_OK;
					}

					// Everything else is genuinely unknown to this model; the reserved value says
					// so explicitly rather than inventing a plausible default.
					auto* not_supported = static_cast<IUnknown*>(nullptr);
					auto const hr = UiaGetReservedNotSupportedValue(&not_supported);
					if (FAILED(hr))
						return hr;

					*ret = EmptyVariant();
					ret->vt = VT_UNKNOWN;
					ret->punkVal = not_supported;
					return S_OK;
				});
			}
			HRESULT STDMETHODCALLTYPE GetBoundingRectangles(SAFEARRAY** ret) override
			{
				return ComGuard([&]() -> HRESULT
				{
					if (ret == nullptr)
						return E_POINTER;

					*ret = nullptr;
					auto const snapshot = RequireSnapshot();
					auto const& node = RequireNode(snapshot);
					auto start = std::uint32_t{};
					auto end = std::uint32_t{};
					Normalize(node.value_utf8, start, end);

					// Per-glyph rectangles would need shaped run metrics the published snapshot
					// deliberately does not carry, so a non-degenerate range reports the control's
					// own rectangle: coarser than ideal, but never wrong about where the text is.
					if (start == end)
					{
						*ret = EmptyArray(VT_R8);
						return *ret != nullptr ? S_OK : E_OUTOFMEMORY;
					}

					auto const bounds = NodeScreenRect(*m_shared, *snapshot, node);
					auto* array = SafeArrayCreateVector(VT_R8, 0, 4);
					if (array == nullptr)
						return E_OUTOFMEMORY;

					double const values[4] = { bounds.left, bounds.top, bounds.width, bounds.height };
					for (auto i = LONG{}; i != 4; ++i)
					{
						auto const hr = SafeArrayPutElement(array, &i, const_cast<double*>(&values[i]));
						if (FAILED(hr))
						{
							SafeArrayDestroy(array);
							return hr;
						}
					}

					*ret = array;
					return S_OK;
				});
			}
			HRESULT STDMETHODCALLTYPE GetEnclosingElement(IRawElementProviderSimple** ret) override
			{
				return ComGuard([&]() -> HRESULT
				{
					if (ret == nullptr)
						return E_POINTER;

					auto const snapshot = RequireSnapshot();
					RequireNode(snapshot);
					*ret = CreateUiaElementProvider(m_shared, m_id);
					return *ret != nullptr ? S_OK : E_OUTOFMEMORY;
				});
			}
			HRESULT STDMETHODCALLTYPE GetText(int max_length, BSTR* ret) override
			{
				return ComGuard([&]() -> HRESULT
				{
					if (ret == nullptr)
						return E_POINTER;

					auto const snapshot = RequireSnapshot();
					auto const& node = RequireNode(snapshot);
					auto start = std::uint32_t{};
					auto end = std::uint32_t{};
					Normalize(node.value_utf8, start, end);

					// A negative or absent limit means "no limit"; a positive one bounds the result
					// in UTF-16 code units, which is the unit the caller's buffer is measured in.
					auto wide = std::wstring{};
					if (!Utf8ToUtf16(std::string_view(node.value_utf8).substr(start, end - start), wide))
						return E_FAIL;
					if (max_length >= 0 && wide.size() > static_cast<std::size_t>(max_length))
						wide.resize(static_cast<std::size_t>(max_length));

					return AssignBstr(wide, ret);
				});
			}
			HRESULT STDMETHODCALLTYPE Move(TextUnit unit, int count, int* ret) override
			{
				return ComGuard([&]() -> HRESULT
				{
					if (ret == nullptr)
						return E_POINTER;

					*ret = 0;
					auto const snapshot = RequireSnapshot();
					auto const& node = RequireNode(snapshot);
					auto const text = std::string_view(node.value_utf8);
					auto start = std::uint32_t{};
					auto end = std::uint32_t{};
					Normalize(text, start, end);

					// A non-degenerate range collapses to its start, moves, then re-expands, which
					// is the behaviour clients rely on for "move to the next word/character".
					auto const was_degenerate = start == end;
					auto offset = start;
					auto moved = MoveOffset(text, unit, count, offset);
					m_start = offset;
					m_end = offset;
					if (!was_degenerate)
					{
						auto const hr = ExpandToEnclosingUnit(unit);
						if (FAILED(hr))
							return hr;

						// Every unit above Word spans this whole single-line value, so re-expanding
						// can land back on exactly the span the range already covered. Reporting an
						// attempted move there would let a client that walks the text with Move
						// until it returns zero run forever, so a range that did not change reports
						// no movement.
						if (m_start == start && m_end == end)
							moved = 0;
					}

					*ret = moved;
					return S_OK;
				});
			}
			HRESULT STDMETHODCALLTYPE MoveEndpointByUnit(TextPatternRangeEndpoint endpoint, TextUnit unit, int count, int* ret) override
			{
				return ComGuard([&]() -> HRESULT
				{
					if (ret == nullptr)
						return E_POINTER;

					*ret = 0;
					auto const snapshot = RequireSnapshot();
					auto const& node = RequireNode(snapshot);
					auto const text = std::string_view(node.value_utf8);
					auto start = std::uint32_t{};
					auto end = std::uint32_t{};
					Normalize(text, start, end);

					auto& moving = endpoint == TextPatternRangeEndpoint_Start ? start : end;
					auto const moved = MoveOffset(text, unit, count, moving);

					// Crossing the other endpoint collapses the range onto the moved endpoint,
					// which is what the pattern requires rather than an inverted range.
					if (start > end)
					{
						start = moving;
						end = moving;
					}

					m_start = start;
					m_end = end;
					*ret = moved;
					return S_OK;
				});
			}
			HRESULT STDMETHODCALLTYPE MoveEndpointByRange(TextPatternRangeEndpoint endpoint, ITextRangeProvider* other, TextPatternRangeEndpoint other_endpoint) override
			{
				return ComGuard([&]() -> HRESULT
				{
					auto const* peer = dynamic_cast<UiaTextRangeProvider*>(other);
					if (peer == nullptr || peer->m_id != m_id)
						return UIA_E_INVALIDOPERATION;

					auto const target = other_endpoint == TextPatternRangeEndpoint_Start ? peer->m_start : peer->m_end;
					auto start = endpoint == TextPatternRangeEndpoint_Start ? target : m_start;
					auto end = endpoint == TextPatternRangeEndpoint_End ? target : m_end;
					if (start > end)
					{
						start = target;
						end = target;
					}

					m_start = start;
					m_end = end;
					return S_OK;
				});
			}
			HRESULT STDMETHODCALLTYPE Select() override
			{
				return ComGuard([&]() -> HRESULT
				{
					auto const snapshot = RequireSnapshot();
					auto const& node = RequireNode(snapshot);
					auto start = std::uint32_t{};
					auto end = std::uint32_t{};
					Normalize(node.value_utf8, start, end);

					auto const request = SemanticActionRequest{
						.kind = ESemanticActionKind::SetSelection,
						.control_id = m_id,
						.text = {},
						.selection_start = start,
						.selection_end = end,
					};
					return m_shared->InvokeAction(request);
				});
			}
			HRESULT STDMETHODCALLTYPE AddToSelection() override
			{
				// The edit model has exactly one selection, so adding to it is not merely
				// unimplemented, it is undefined for this control.
				return UIA_E_INVALIDOPERATION;
			}
			HRESULT STDMETHODCALLTYPE RemoveFromSelection() override
			{
				return UIA_E_INVALIDOPERATION;
			}
			HRESULT STDMETHODCALLTYPE ScrollIntoView(BOOL) override
			{
				// A TextBox renders its whole value with no viewport of its own, so every range is
				// already in view and there is nothing truthful left to do.
				return S_OK;
			}
			HRESULT STDMETHODCALLTYPE GetChildren(SAFEARRAY** ret) override
			{
				return ComGuard([&]() -> HRESULT
				{
					if (ret == nullptr)
						return E_POINTER;

					*ret = EmptyArray(VT_UNKNOWN);
					return *ret != nullptr ? S_OK : E_OUTOFMEMORY;
				});
			}

			HRESULT STDMETHODCALLTYPE FindText(BSTR text, BOOL backward, BOOL ignore_case, ITextRangeProvider** ret) override
			{
				return ComGuard([&]() -> HRESULT
				{
					if (ret == nullptr)
						return E_POINTER;

					*ret = nullptr;
					auto const snapshot = RequireSnapshot();
					auto const& node = RequireNode(snapshot);
					auto start = std::uint32_t{};
					auto end = std::uint32_t{};
					Normalize(node.value_utf8, start, end);

					auto needle = std::string{};
					if (text != nullptr && !Utf16ToUtf8(std::wstring_view(text, SysStringLen(text)), needle))
						return E_INVALIDARG;
					if (needle.empty())
						return S_OK;

					// Case-insensitive search folds ASCII only; the full-repertoire case tables are
					// outside this module's scope, so anything beyond ASCII matches case-sensitively.
					auto const haystack = std::string_view(node.value_utf8).substr(start, end - start);
					auto const folded_haystack = ignore_case != FALSE ? FoldAscii(haystack) : std::string(haystack);
					auto const folded_needle = ignore_case != FALSE ? FoldAscii(needle) : needle;
					auto const found = backward != FALSE ? folded_haystack.rfind(folded_needle) : folded_haystack.find(folded_needle);
					if (found == std::string::npos)
						return S_OK;

					// A match that does not begin and end on cluster boundaries would produce a
					// range no other operation could address, so it is reported as no match at all.
					auto const match_start = start + static_cast<std::uint32_t>(found);
					auto const match_end = match_start + static_cast<std::uint32_t>(folded_needle.size());
					if (!IsGraphemeBoundary(node.value_utf8, match_start) || !IsGraphemeBoundary(node.value_utf8, match_end))
						return S_OK;

					*ret = new UiaTextRangeProvider(m_shared, m_id, match_start, match_end);
					return S_OK;
				});
			}
		};

		// The provider for one semantic node.
		class UiaElementProvider
			: public UiaProviderBase
			, public IRawElementProviderSimple
			, public IRawElementProviderFragment
			, public IInvokeProvider
			, public IValueProvider
			, public ITextProvider2
		{
			ControlId m_id;

			// The published node this provider represents, or a StaleHandle failure when the id no
			// longer exists. This is the single place a stale client reference is rejected.
			UiaNode const& RequireNode(std::shared_ptr<UiaSnapshot const> const& snapshot) const
			{
				auto const* node = snapshot->Find(m_id);
				if (node == nullptr)
					throw EngineException(EStatus::StaleHandle, "the control is no longer available");

				return *node;
			}

		public:

			UiaElementProvider(std::shared_ptr<UiaSharedState> shared, ControlId id)
				: UiaProviderBase(std::move(shared))
				, m_id(id)
			{
				m_shared->RegisterProvider(static_cast<IRawElementProviderSimple*>(this));
			}
			~UiaElementProvider() override = default;

			HRESULT STDMETHODCALLTYPE QueryInterface(REFIID riid, void** obj) override
			{
				if (obj == nullptr)
					return E_POINTER;

				// COM requires the answer to be invariant for the object's lifetime, so the set is
				// fixed by the type rather than by the role the current snapshot gives this control.
				// Which patterns are actually usable is reported by GetPatternProvider and the
				// pattern properties, and a control that has gone away fails its methods with
				// UIA_E_ELEMENTNOTAVAILABLE rather than retracting an interface.
				*obj = nullptr;
				if (riid == __uuidof(IUnknown) || riid == __uuidof(IRawElementProviderSimple))
					*obj = static_cast<IRawElementProviderSimple*>(this);
				else if (riid == __uuidof(IRawElementProviderFragment))
					*obj = static_cast<IRawElementProviderFragment*>(this);
				else if (riid == __uuidof(IInvokeProvider))
					*obj = static_cast<IInvokeProvider*>(this);
				else if (riid == __uuidof(IValueProvider))
					*obj = static_cast<IValueProvider*>(this);
				else if (riid == __uuidof(ITextProvider))
					*obj = static_cast<ITextProvider*>(static_cast<ITextProvider2*>(this));
				else if (riid == __uuidof(ITextProvider2))
					*obj = static_cast<ITextProvider2*>(this);
				else
					return E_NOINTERFACE;

				AddRef();
				return S_OK;
			}
			ULONG STDMETHODCALLTYPE AddRef() override
			{
				return m_ref.fetch_add(1, std::memory_order_relaxed) + 1;
			}
			ULONG STDMETHODCALLTYPE Release() override
			{
				// The registry entry is dropped in the same critical section as the count reaching
				// zero, so teardown can never take a reference to an object that is already dying.
				auto remaining = ULONG{};
				if (m_shared->ReleaseProvider(static_cast<IRawElementProviderSimple*>(this), m_ref, remaining))
					delete this;

				return remaining;
			}

			HRESULT STDMETHODCALLTYPE get_ProviderOptions(ProviderOptions* ret) override
			{
				if (ret == nullptr)
					return E_POINTER;

				// Server-side only: this provider marshals its own mutations to the owner thread,
				// so it must not additionally be wrapped in UI Automation's COM threading model.
				*ret = ProviderOptions_ServerSideProvider;
				return S_OK;
			}
			HRESULT STDMETHODCALLTYPE GetPatternProvider(PATTERNID pattern, IUnknown** ret) override
			{
				return ComGuard([&]() -> HRESULT
				{
					if (ret == nullptr)
						return E_POINTER;

					*ret = nullptr;
					auto const snapshot = RequireSnapshot();
					auto const& node = RequireNode(snapshot);

					// Patterns are advertised from the node's own supported-action set, so a control
					// can never claim a pattern the owner-thread action path would then reject.
					auto const invoke = pattern == UIA_InvokePatternId && node.role == EControlType::Button && node.HasAction(ESemanticAction::Invoke);
					auto const value = pattern == UIA_ValuePatternId && node.role == EControlType::TextBox;
					auto const text = (pattern == UIA_TextPatternId || pattern == UIA_TextPattern2Id) && node.role == EControlType::TextBox;
					if (!invoke && !value && !text)
						return S_OK;

					AddRef();
					if (invoke)
						*ret = static_cast<IInvokeProvider*>(this);
					else if (value)
						*ret = static_cast<IValueProvider*>(this);
					else
						*ret = static_cast<ITextProvider2*>(this);

					return S_OK;
				});
			}
			HRESULT STDMETHODCALLTYPE GetPropertyValue(PROPERTYID property, VARIANT* ret) override
			{
				return ComGuard([&]() -> HRESULT
				{
					if (ret == nullptr)
						return E_POINTER;

					*ret = EmptyVariant();
					auto const snapshot = RequireSnapshot();
					auto const& node = RequireNode(snapshot);

					// Only properties this model actually knows are answered; everything else stays
					// VT_EMPTY so UI Automation supplies its own default rather than a fabrication.
					switch (property)
					{
						case UIA_AutomationIdPropertyId: { *ret = BstrVariant(AutomationIdOf(node.id)); break; }
						case UIA_NamePropertyId: { *ret = BstrVariant(node.name); break; }
						case UIA_HelpTextPropertyId:
						case UIA_FullDescriptionPropertyId: { *ret = BstrVariant(node.description); break; }
						case UIA_ControlTypePropertyId: { *ret = I4Variant(ControlTypeOf(node.role)); break; }
						case UIA_FrameworkIdPropertyId: { *ret = BstrVariant(L"View3DUI"); break; }
						case UIA_ProviderDescriptionPropertyId: { *ret = BstrVariant(L"View3DUI semantic provider"); break; }
						case UIA_IsEnabledPropertyId: { *ret = BoolVariant(node.HasState(ESemanticState::Enabled)); break; }
						case UIA_IsOffscreenPropertyId: { *ret = BoolVariant(node.HasState(ESemanticState::Offscreen) || !node.HasState(ESemanticState::Visible)); break; }
						case UIA_HasKeyboardFocusPropertyId: { *ret = BoolVariant(node.HasState(ESemanticState::Focused)); break; }
						case UIA_IsKeyboardFocusablePropertyId: { *ret = BoolVariant(node.HasState(ESemanticState::Focusable)); break; }
						case UIA_SelectionItemIsSelectedPropertyId: { *ret = BoolVariant(node.HasState(ESemanticState::Selected)); break; }
						case UIA_IsDataValidForFormPropertyId: { *ret = BoolVariant(!node.HasState(ESemanticState::Invalid)); break; }
						case UIA_IsContentElementPropertyId: { *ret = BoolVariant(IsContentElement(node.role)); break; }
						case UIA_IsControlElementPropertyId: { *ret = BoolVariant(true); break; }
						case UIA_ValueValuePropertyId: { *ret = BstrVariant(node.value); break; }
						case UIA_ValueIsReadOnlyPropertyId: { *ret = BoolVariant(!node.HasState(ESemanticState::Enabled)); break; }
						case UIA_IsInvokePatternAvailablePropertyId: { *ret = BoolVariant(node.role == EControlType::Button && node.HasAction(ESemanticAction::Invoke)); break; }
						case UIA_IsValuePatternAvailablePropertyId: { *ret = BoolVariant(node.role == EControlType::TextBox); break; }
						case UIA_IsTextPatternAvailablePropertyId:
						case UIA_IsTextPattern2AvailablePropertyId: { *ret = BoolVariant(node.role == EControlType::TextBox); break; }
						default: break;
					}

					return S_OK;
				});
			}
			HRESULT STDMETHODCALLTYPE get_HostRawElementProvider(IRawElementProviderSimple** ret) override
			{
				if (ret == nullptr)
					return E_POINTER;

				// Only the fragment root is hosted by the window; a child element that claimed a
				// host provider would be reported twice in the tree.
				*ret = nullptr;
				return S_OK;
			}

			HRESULT STDMETHODCALLTYPE Navigate(NavigateDirection direction, IRawElementProviderFragment** ret) override
			{
				return ComGuard([&]() -> HRESULT
				{
					if (ret == nullptr)
						return E_POINTER;

					*ret = nullptr;
					auto const snapshot = RequireSnapshot();
					auto const& node = RequireNode(snapshot);

					// Siblings come from the parent's child list, or from the snapshot's root list
					// for a semantic root; both are already in deterministic pre-order.
					auto const& siblings = node.parent_index != UiaNoIndex ? snapshot->m_nodes[node.parent_index].children : snapshot->m_roots;
					auto const position = node.sibling_position;

					auto target = UiaNoIndex;
					switch (direction)
					{
						case NavigateDirection_Parent:
						{
							if (node.parent_index != UiaNoIndex)
								target = node.parent_index;
							else
							{
								*ret = MakeRootFragment(m_shared);
								return *ret != nullptr ? S_OK : E_OUTOFMEMORY;
							}
							break;
						}
						case NavigateDirection_FirstChild: { target = !node.children.empty() ? node.children.front() : UiaNoIndex; break; }
						case NavigateDirection_LastChild: { target = !node.children.empty() ? node.children.back() : UiaNoIndex; break; }
						case NavigateDirection_NextSibling: { target = position + 1 < siblings.size() ? siblings[position + 1] : UiaNoIndex; break; }
						case NavigateDirection_PreviousSibling: { target = position != 0 && position <= siblings.size() ? siblings[position - 1] : UiaNoIndex; break; }
						// NavigateDirection arrives from the client, so an unrecognised value is a
						// caller error rather than a missing case here.
						default: return E_INVALIDARG;
					}

					if (target == UiaNoIndex)
						return S_OK;

					*ret = MakeFragment(m_shared, snapshot->m_nodes[target].id);
					return *ret != nullptr ? S_OK : E_OUTOFMEMORY;
				});
			}
			HRESULT STDMETHODCALLTYPE GetRuntimeId(SAFEARRAY** ret) override
			{
				return ComGuard([&]() -> HRESULT
				{
					auto const snapshot = RequireSnapshot();
					RequireNode(snapshot);
					return MakeRuntimeId(m_id, ret);
				});
			}
			HRESULT STDMETHODCALLTYPE get_BoundingRectangle(UiaRect* ret) override
			{
				return ComGuard([&]() -> HRESULT
				{
					if (ret == nullptr)
						return E_POINTER;

					auto const snapshot = RequireSnapshot();
					auto const& node = RequireNode(snapshot);
					*ret = NodeScreenRect(*m_shared, *snapshot, node);
					return S_OK;
				});
			}
			HRESULT STDMETHODCALLTYPE GetEmbeddedFragmentRoots(SAFEARRAY** ret) override
			{
				if (ret == nullptr)
					return E_POINTER;

				// The whole fragment lives in one HWND; there is nothing else to embed.
				*ret = nullptr;
				return S_OK;
			}
			HRESULT STDMETHODCALLTYPE SetFocus() override
			{
				return ComGuard([&]() -> HRESULT
				{
					auto const request = SemanticActionRequest{
						.kind = ESemanticActionKind::Focus,
						.control_id = m_id,
						.text = {},
						.selection_start = 0,
						.selection_end = 0,
					};
					return m_shared->InvokeAction(request);
				});
			}
			HRESULT STDMETHODCALLTYPE get_FragmentRoot(IRawElementProviderFragmentRoot** ret) override
			{
				return ComGuard([&]() -> HRESULT
				{
					if (ret == nullptr)
						return E_POINTER;

					*ret = MakeFragmentRoot(m_shared);
					return *ret != nullptr ? S_OK : E_OUTOFMEMORY;
				});
			}

			HRESULT STDMETHODCALLTYPE Invoke() override
			{
				return ComGuard([&]() -> HRESULT
				{
					auto const request = SemanticActionRequest{
						.kind = ESemanticActionKind::Invoke,
						.control_id = m_id,
						.text = {},
						.selection_start = 0,
						.selection_end = 0,
					};
					return m_shared->InvokeAction(request);
				});
			}

			HRESULT STDMETHODCALLTYPE SetValue(LPCWSTR value) override
			{
				return ComGuard([&]() -> HRESULT
				{
					auto text = std::string{};
					if (value != nullptr && !Utf16ToUtf8(value, text))
						return E_INVALIDARG;

					auto const request = SemanticActionRequest{
						.kind = ESemanticActionKind::SetValue,
						.control_id = m_id,
						.text = std::move(text),
						.selection_start = 0,
						.selection_end = 0,
					};
					return m_shared->InvokeAction(request);
				});
			}
			HRESULT STDMETHODCALLTYPE get_Value(BSTR* ret) override
			{
				return ComGuard([&]() -> HRESULT
				{
					auto const snapshot = RequireSnapshot();
					auto const& node = RequireNode(snapshot);
					return AssignBstr(node.value, ret);
				});
			}
			HRESULT STDMETHODCALLTYPE get_IsReadOnly(BOOL* ret) override
			{
				return ComGuard([&]() -> HRESULT
				{
					if (ret == nullptr)
						return E_POINTER;

					auto const snapshot = RequireSnapshot();
					auto const& node = RequireNode(snapshot);
					*ret = node.HasState(ESemanticState::Enabled) ? FALSE : TRUE;
					return S_OK;
				});
			}

			HRESULT STDMETHODCALLTYPE GetSelection(SAFEARRAY** ret) override
			{
				return ComGuard([&]() -> HRESULT
				{
					if (ret == nullptr)
						return E_POINTER;

					*ret = nullptr;
					auto const snapshot = RequireSnapshot();
					auto const& node = RequireNode(snapshot);

					// A caret with no selection is reported as a degenerate range, which is how a
					// screen reader learns where the insertion point is.
					auto const start = node.HasTextFlag(ESemanticTextFlag::HasSelection) ? node.selection_start : node.caret;
					auto const end = node.HasTextFlag(ESemanticTextFlag::HasSelection) ? node.selection_end : node.caret;

					auto* array = SafeArrayCreateVector(VT_UNKNOWN, 0, 1);
					if (array == nullptr)
						return E_OUTOFMEMORY;

					auto* range = static_cast<ITextRangeProvider*>(new UiaTextRangeProvider(m_shared, m_id, start, end));
					auto index = LONG{};
					auto const hr = SafeArrayPutElement(array, &index, range);
					range->Release();
					if (FAILED(hr))
					{
						SafeArrayDestroy(array);
						return hr;
					}

					*ret = array;
					return S_OK;
				});
			}
			HRESULT STDMETHODCALLTYPE GetVisibleRanges(SAFEARRAY** ret) override
			{
				return ComGuard([&]() -> HRESULT
				{
					if (ret == nullptr)
						return E_POINTER;

					*ret = nullptr;
					auto const snapshot = RequireSnapshot();
					auto const& node = RequireNode(snapshot);

					auto* array = SafeArrayCreateVector(VT_UNKNOWN, 0, 1);
					if (array == nullptr)
						return E_OUTOFMEMORY;

					auto* range = static_cast<ITextRangeProvider*>(new UiaTextRangeProvider(m_shared, m_id, 0, static_cast<std::uint32_t>(node.value_utf8.size())));
					auto index = LONG{};
					auto const hr = SafeArrayPutElement(array, &index, range);
					range->Release();
					if (FAILED(hr))
					{
						SafeArrayDestroy(array);
						return hr;
					}

					*ret = array;
					return S_OK;
				});
			}
			HRESULT STDMETHODCALLTYPE RangeFromChild(IRawElementProviderSimple*, ITextRangeProvider** ret) override
			{
				if (ret != nullptr)
					*ret = nullptr;

				// A TextBox publishes no child elements, so no child can enclose a range.
				return UIA_E_INVALIDOPERATION;
			}
			HRESULT STDMETHODCALLTYPE RangeFromPoint(UiaPoint, ITextRangeProvider** ret) override
			{
				if (ret != nullptr)
					*ret = nullptr;

				// Mapping a point to a text offset needs shaped glyph metrics, which the published
				// snapshot deliberately does not carry; guessing would put the caret in the wrong
				// cluster, so the operation is declined instead.
				return UIA_E_NOTSUPPORTED;
			}
			HRESULT STDMETHODCALLTYPE get_DocumentRange(ITextRangeProvider** ret) override
			{
				return ComGuard([&]() -> HRESULT
				{
					if (ret == nullptr)
						return E_POINTER;

					auto const snapshot = RequireSnapshot();
					auto const& node = RequireNode(snapshot);
					*ret = new UiaTextRangeProvider(m_shared, m_id, 0, static_cast<std::uint32_t>(node.value_utf8.size()));
					return S_OK;
				});
			}
			HRESULT STDMETHODCALLTYPE get_SupportedTextSelection(SupportedTextSelection* ret) override
			{
				if (ret == nullptr)
					return E_POINTER;

				*ret = SupportedTextSelection_Single;
				return S_OK;
			}
			HRESULT STDMETHODCALLTYPE GetCaretRange(BOOL* is_active, ITextRangeProvider** ret) override
			{
				return ComGuard([&]() -> HRESULT
				{
					if (is_active == nullptr || ret == nullptr)
						return E_POINTER;

					auto const snapshot = RequireSnapshot();
					auto const& node = RequireNode(snapshot);
					*is_active = node.HasState(ESemanticState::Focused) ? TRUE : FALSE;
					*ret = new UiaTextRangeProvider(m_shared, m_id, node.caret, node.caret);
					return S_OK;
				});
			}
			HRESULT STDMETHODCALLTYPE RangeFromAnnotation(IRawElementProviderSimple*, ITextRangeProvider** ret) override
			{
				if (ret != nullptr)
					*ret = nullptr;

				// This model has no annotation elements to anchor a range to.
				return UIA_E_NOTSUPPORTED;
			}
		};

		// The HWND fragment root. It owns no semantics of its own: its children are the semantic
		// roots, and its bounds and name come from the window, so it adds no element to the model.
		class UiaRootProvider
			: public UiaProviderBase
			, public IRawElementProviderSimple
			, public IRawElementProviderFragment
			, public IRawElementProviderFragmentRoot
		{
		public:

			explicit UiaRootProvider(std::shared_ptr<UiaSharedState> shared)
				: UiaProviderBase(std::move(shared))
			{
				m_shared->RegisterProvider(static_cast<IRawElementProviderSimple*>(this));
			}
			~UiaRootProvider() override = default;

			HRESULT STDMETHODCALLTYPE QueryInterface(REFIID riid, void** obj) override
			{
				if (obj == nullptr)
					return E_POINTER;

				*obj = nullptr;
				if (riid == __uuidof(IUnknown) || riid == __uuidof(IRawElementProviderSimple))
					*obj = static_cast<IRawElementProviderSimple*>(this);
				else if (riid == __uuidof(IRawElementProviderFragment))
					*obj = static_cast<IRawElementProviderFragment*>(this);
				else if (riid == __uuidof(IRawElementProviderFragmentRoot))
					*obj = static_cast<IRawElementProviderFragmentRoot*>(this);
				else
					return E_NOINTERFACE;

				AddRef();
				return S_OK;
			}
			ULONG STDMETHODCALLTYPE AddRef() override
			{
				return m_ref.fetch_add(1, std::memory_order_relaxed) + 1;
			}
			ULONG STDMETHODCALLTYPE Release() override
			{
				// The registry entry is dropped in the same critical section as the count reaching
				// zero, so teardown can never take a reference to an object that is already dying.
				auto remaining = ULONG{};
				if (m_shared->ReleaseProvider(static_cast<IRawElementProviderSimple*>(this), m_ref, remaining))
					delete this;

				return remaining;
			}

			HRESULT STDMETHODCALLTYPE get_ProviderOptions(ProviderOptions* ret) override
			{
				if (ret == nullptr)
					return E_POINTER;

				*ret = ProviderOptions_ServerSideProvider;
				return S_OK;
			}
			HRESULT STDMETHODCALLTYPE GetPatternProvider(PATTERNID, IUnknown** ret) override
			{
				if (ret == nullptr)
					return E_POINTER;

				// The root is a container; every interactive pattern belongs to a semantic node.
				*ret = nullptr;
				return S_OK;
			}
			HRESULT STDMETHODCALLTYPE GetPropertyValue(PROPERTYID property, VARIANT* ret) override
			{
				return ComGuard([&]() -> HRESULT
				{
					if (ret == nullptr)
						return E_POINTER;

					*ret = EmptyVariant();
					switch (property)
					{
						case UIA_AutomationIdPropertyId: { *ret = BstrVariant(L"view3dui:root"); break; }
						case UIA_ControlTypePropertyId: { *ret = I4Variant(UIA_PaneControlTypeId); break; }
						case UIA_FrameworkIdPropertyId: { *ret = BstrVariant(L"View3DUI"); break; }
						case UIA_ProviderDescriptionPropertyId: { *ret = BstrVariant(L"View3DUI fragment root"); break; }
						case UIA_IsEnabledPropertyId: { *ret = BoolVariant(m_shared->Available()); break; }
						case UIA_IsControlElementPropertyId: { *ret = BoolVariant(true); break; }
						case UIA_IsContentElementPropertyId: { *ret = BoolVariant(false); break; }
						case UIA_IsKeyboardFocusablePropertyId: { *ret = BoolVariant(false); break; }
						default: break;
					}

					return S_OK;
				});
			}
			HRESULT STDMETHODCALLTYPE get_HostRawElementProvider(IRawElementProviderSimple** ret) override
			{
				return ComGuard([&]() -> HRESULT
				{
					if (ret == nullptr)
						return E_POINTER;

					*ret = nullptr;
					auto const hwnd = m_shared->Window();
					if (hwnd == nullptr || IsWindow(hwnd) == FALSE)
						return static_cast<HRESULT>(UIA_E_ELEMENTNOTAVAILABLE);

					// The host provider supplies the window-level properties (bounds, native window
					// handle, process id) so this fragment never has to duplicate them.
					return UiaHostProviderFromHwnd(hwnd, ret);
				});
			}

			HRESULT STDMETHODCALLTYPE Navigate(NavigateDirection direction, IRawElementProviderFragment** ret) override
			{
				return ComGuard([&]() -> HRESULT
				{
					if (ret == nullptr)
						return E_POINTER;

					*ret = nullptr;
					auto const snapshot = RequireSnapshot();

					auto target = UiaNoIndex;
					switch (direction)
					{
						case NavigateDirection_Parent: { return S_OK; }
						case NavigateDirection_FirstChild: { target = !snapshot->m_roots.empty() ? snapshot->m_roots.front() : UiaNoIndex; break; }
						case NavigateDirection_LastChild: { target = !snapshot->m_roots.empty() ? snapshot->m_roots.back() : UiaNoIndex; break; }
						case NavigateDirection_NextSibling:
						case NavigateDirection_PreviousSibling: { return S_OK; }
						default: return E_INVALIDARG;
					}

					if (target == UiaNoIndex)
						return S_OK;

					*ret = MakeFragment(m_shared, snapshot->m_nodes[target].id);
					return *ret != nullptr ? S_OK : E_OUTOFMEMORY;
				});
			}
			HRESULT STDMETHODCALLTYPE GetRuntimeId(SAFEARRAY** ret) override
			{
				if (ret == nullptr)
					return E_POINTER;

				// A null runtime id tells UI Automation to derive one from the host window, which
				// keeps the root's identity stable across every republication.
				*ret = nullptr;
				return S_OK;
			}
			HRESULT STDMETHODCALLTYPE get_BoundingRectangle(UiaRect* ret) override
			{
				if (ret == nullptr)
					return E_POINTER;

				// An all-zero rectangle defers to the host provider's window rectangle.
				*ret = UiaRect{ 0.0, 0.0, 0.0, 0.0 };
				return S_OK;
			}
			HRESULT STDMETHODCALLTYPE GetEmbeddedFragmentRoots(SAFEARRAY** ret) override
			{
				if (ret == nullptr)
					return E_POINTER;

				*ret = nullptr;
				return S_OK;
			}
			HRESULT STDMETHODCALLTYPE SetFocus() override
			{
				// Focus belongs to a control, never to the container; the window itself is focused
				// by the host, not by this fragment.
				return S_OK;
			}
			HRESULT STDMETHODCALLTYPE get_FragmentRoot(IRawElementProviderFragmentRoot** ret) override
			{
				if (ret == nullptr)
					return E_POINTER;

				AddRef();
				*ret = static_cast<IRawElementProviderFragmentRoot*>(this);
				return S_OK;
			}

			HRESULT STDMETHODCALLTYPE ElementProviderFromPoint(double x, double y, IRawElementProviderFragment** ret) override
			{
				return ComGuard([&]() -> HRESULT
				{
					if (ret == nullptr)
						return E_POINTER;

					*ret = nullptr;
					auto const snapshot = RequireSnapshot();
					auto const hwnd = m_shared->Window();
					if (hwnd == nullptr || IsWindow(hwnd) == FALSE)
						return static_cast<HRESULT>(UIA_E_ELEMENTNOTAVAILABLE);

					auto origin = POINT{ 0, 0 };
					if (ClientToScreen(hwnd, &origin) == FALSE)
						return static_cast<HRESULT>(UIA_E_ELEMENTNOTAVAILABLE);

					// Later nodes in the pre-order walk are drawn over earlier ones, so scanning
					// backwards finds the topmost hit deterministically.
					auto const client_x = static_cast<float>(x - origin.x);
					auto const client_y = static_cast<float>(y - origin.y);
					for (auto i = snapshot->m_nodes.size(); i-- != 0;)
					{
						auto const& node = snapshot->m_nodes[i];
						if (node.HasState(ESemanticState::Offscreen) || !node.HasState(ESemanticState::Visible))
							continue;

						auto const rect = UiaClientPixelRect(snapshot->m_viewport, node.bounds_dip);
						if (client_x < rect.x || client_y < rect.y || client_x >= rect.x + rect.w || client_y >= rect.y + rect.h)
							continue;

						*ret = MakeFragment(m_shared, node.id);
						return *ret != nullptr ? S_OK : E_OUTOFMEMORY;
					}

					// Empty space inside the fragment still belongs to this fragment, so the root
					// answers for it. Returning null instead would tell the client the point is
					// outside the fragment entirely, which is what makes hit-testing fall through to
					// the wrong element.
					*ret = MakeRootFragment(m_shared);
					return *ret != nullptr ? S_OK : E_OUTOFMEMORY;
				});
			}
			HRESULT STDMETHODCALLTYPE GetFocus(IRawElementProviderFragment** ret) override
			{
				return ComGuard([&]() -> HRESULT
				{
					if (ret == nullptr)
						return E_POINTER;

					*ret = nullptr;
					auto const snapshot = RequireSnapshot();
					if (snapshot->m_focus_id == 0)
						return S_OK;

					*ret = MakeFragment(m_shared, snapshot->m_focus_id);
					return *ret != nullptr ? S_OK : E_OUTOFMEMORY;
				});
			}
		};

		IRawElementProviderFragment* MakeFragment(std::shared_ptr<UiaSharedState> const& shared, ControlId id)
		{
			return static_cast<IRawElementProviderFragment*>(new UiaElementProvider(shared, id));
		}

		IRawElementProviderFragmentRoot* MakeFragmentRoot(std::shared_ptr<UiaSharedState> const& shared)
		{
			return static_cast<IRawElementProviderFragmentRoot*>(new UiaRootProvider(shared));
		}

		IRawElementProviderFragment* MakeRootFragment(std::shared_ptr<UiaSharedState> const& shared)
		{
			return static_cast<IRawElementProviderFragment*>(new UiaRootProvider(shared));
		}
	}

	IRawElementProviderSimple* CreateUiaRootProvider(std::shared_ptr<UiaSharedState> shared)
	{
		return static_cast<IRawElementProviderSimple*>(new UiaRootProvider(std::move(shared)));
	}

	IRawElementProviderSimple* CreateUiaElementProvider(std::shared_ptr<UiaSharedState> shared, ControlId id)
	{
		return static_cast<IRawElementProviderSimple*>(new UiaElementProvider(std::move(shared), id));
	}
}
