//*********************************************
// View3DUI
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Owner-thread half of the UI Automation bridge; see uia_bridge.h for the threading contract every
// rule below implements.
#include "uia_bridge.h"
#include "uia_provider.h"
#include "pr/view3d-ui/engine.h"

// UiaRaiseAutomationEvent, UiaReturnRawElementProvider, UiaDisconnectProvider and
// UiaClientsAreListening live in uiautomationcore.lib. The dependency is declared next to the code
// that needs it, matching dwrite.lib in text_shaper.cpp and imm32.lib in win32_input.cpp.
#pragma comment(lib, "uiautomationcore.lib")

namespace pr::view3d::ui
{
	namespace
	{
		// A VT_BSTR VARIANT, degrading to VT_EMPTY when the string cannot be allocated. An event
		// payload has no way to report failure, and an absent old/new value is the truthful
		// degradation: the client re-reads the property instead.
		VARIANT EventBstr(std::wstring const& text)
		{
			auto value = VARIANT{};
			VariantInit(&value);

			auto const bstr = SysAllocStringLen(text.c_str(), static_cast<UINT>(text.size()));
			if (bstr == nullptr)
				return value;

			value.vt = VT_BSTR;
			value.bstrVal = bstr;
			return value;
		}

		// A VT_BOOL VARIANT.
		VARIANT EventBool(bool state)
		{
			auto value = VARIANT{};
			VariantInit(&value);
			value.vt = VT_BOOL;
			value.boolVal = state ? VARIANT_TRUE : VARIANT_FALSE;
			return value;
		}

		// Raise one property-changed notification and release both payloads. Failures are ignored
		// because a client that cannot be notified will re-read the property on its next poll;
		// there is no recovery an owner-thread update could usefully perform.
		void RaisePropertyChanged(IRawElementProviderSimple* provider, PROPERTYID property, VARIANT old_value, VARIANT new_value)
		{
			UiaRaiseAutomationPropertyChangedEvent(provider, property, old_value, new_value);
			VariantClear(&old_value);
			VariantClear(&new_value);
		}

		// Raise the boolean state notifications that actually differ between two publications of
		// the same node. Only properties this provider answers are considered, so the set is small
		// and fixed rather than growing with the state bitmask.
		void RaiseStateChanges(IRawElementProviderSimple* provider, UiaNode const& before, UiaNode const& after)
		{
			struct StateProperty
			{
				PROPERTYID property;
				ESemanticState flag;
				std::int32_t inverted; // True when the property reports the absence of the flag.
			};
			static constexpr StateProperty properties[] = {
				{ UIA_IsEnabledPropertyId, ESemanticState::Enabled, 0 },
				{ UIA_IsOffscreenPropertyId, ESemanticState::Offscreen, 0 },
				{ UIA_HasKeyboardFocusPropertyId, ESemanticState::Focused, 0 },
				{ UIA_IsKeyboardFocusablePropertyId, ESemanticState::Focusable, 0 },
				{ UIA_SelectionItemIsSelectedPropertyId, ESemanticState::Selected, 0 },
				{ UIA_IsDataValidForFormPropertyId, ESemanticState::Invalid, 1 },
			};

			for (auto const& entry : properties)
			{
				auto const was = before.HasState(entry.flag);
				auto const now = after.HasState(entry.flag);
				if (was == now)
					continue;

				RaisePropertyChanged(provider, entry.property, EventBool(entry.inverted != 0 ? !was : was), EventBool(entry.inverted != 0 ? !now : now));
			}
		}

		// Raise the notifications 'diff' calls for. Runs on the owner thread only, does no work at
		// all when no client is listening, and is bounded by UiaMaxChangedNodes by construction.
		void RaiseUiaEvents(std::shared_ptr<UiaSharedState> const& shared, UiaSnapshot const* previous, UiaSnapshot const& current, UiaSnapshotDiff const& diff)
		{
			if (UiaClientsAreListening() == FALSE)
				return;

			// A membership, parent or order change invalidates the client's whole cached subtree,
			// which supersedes any individual property notification.
			if (diff.structure_changed != 0)
			{
				auto* root = CreateUiaRootProvider(shared);
				if (root != nullptr)
				{
					UiaRaiseStructureChangedEvent(root, StructureChangeType_ChildrenInvalidated, nullptr, 0);
					root->Release();
				}
			}

			// Focus arriving is reported against the control that gained it; focus leaving the
			// fragment entirely has no control to report against, so the fragment root stands in.
			// Without that a client keeps announcing the control it last saw focused.
			if (diff.focus_changed != 0)
			{
				auto* element = diff.focused_id != 0
					? CreateUiaElementProvider(shared, diff.focused_id)
					: CreateUiaRootProvider(shared);
				if (element != nullptr)
				{
					UiaRaiseAutomationEvent(element, UIA_AutomationFocusChangedEventId);
					element->Release();
				}
			}

			// Any geometry change is reported once, against the fragment root, rather than as a
			// bounding-rectangle notification per node; a client re-reads the rectangles it cares
			// about, and the event count stays independent of how many nodes moved. A viewport or
			// DPI change moves everything on screen without altering any layout rectangle, so it
			// produces the same single notification.
			if (!diff.bounds_changed.empty() || diff.viewport_changed != 0)
			{
				auto* root = CreateUiaRootProvider(shared);
				if (root != nullptr)
				{
					UiaRaiseAutomationEvent(root, UIA_LayoutInvalidatedEventId);
					root->Release();
				}
			}

			// Per-node property notifications need the previous value, so they only exist when a
			// previous publication does.
			if (previous == nullptr)
				return;

			for (auto const id : diff.name_changed)
			{
				auto const* before = previous->Find(id);
				auto const* after = current.Find(id);
				if (before == nullptr || after == nullptr)
					continue;

				auto* element = CreateUiaElementProvider(shared, id);
				if (element == nullptr)
					continue;

				if (before->name != after->name)
					RaisePropertyChanged(element, UIA_NamePropertyId, EventBstr(before->name), EventBstr(after->name));
				if (before->description != after->description)
					RaisePropertyChanged(element, UIA_FullDescriptionPropertyId, EventBstr(before->description), EventBstr(after->description));

				element->Release();
			}

			for (auto const id : diff.value_changed)
			{
				auto const* before = previous->Find(id);
				auto const* after = current.Find(id);
				if (before == nullptr || after == nullptr)
					continue;

				auto* element = CreateUiaElementProvider(shared, id);
				if (element == nullptr)
					continue;

				RaisePropertyChanged(element, UIA_ValueValuePropertyId, EventBstr(before->value), EventBstr(after->value));
				UiaRaiseAutomationEvent(element, UIA_Text_TextChangedEventId);
				element->Release();
			}

			for (auto const id : diff.state_changed)
			{
				auto const* before = previous->Find(id);
				auto const* after = current.Find(id);
				if (before == nullptr || after == nullptr)
					continue;

				auto* element = CreateUiaElementProvider(shared, id);
				if (element == nullptr)
					continue;

				RaiseStateChanges(element, *before, *after);
				element->Release();
			}
		}
	}

	UINT UiaActionMessageId()
	{
		// Registered once per process and cached; RegisterWindowMessageW returns the same value for
		// the same string, so every context in the process shares this id and the per-call target
		// cookie is what distinguishes them.
		static UINT const id = RegisterWindowMessageW(L"Rylogic.View3DUI.UiaSemanticAction");
		return id;
	}

	WPARAM UiaActionCookie()
	{
		// The address of a module-private object: distinct from any value another process could
		// guess, and distinct between two loaded copies of this module, while identical for the
		// sender and receiver of any one call because both live in the same copy.
		static char const anchor = 0;
		return reinterpret_cast<WPARAM>(&anchor);
	}

	UiaActionCall::UiaActionCall(UiaSharedState* target, SemanticActionRequest const& request)
		: m_refs(1)
		, m_target(target)
		, m_request(request)
		, m_handled(0)
		, m_abandoned(0)
		, m_result(static_cast<HRESULT>(UIA_E_ELEMENTNOTAVAILABLE))
	{}

	void UiaActionCall::AddRef()
	{
		m_refs.fetch_add(1, std::memory_order_relaxed);
	}

	void UiaActionCall::Release()
	{
		if (m_refs.fetch_sub(1, std::memory_order_acq_rel) == 1)
			delete this;
	}

	UiaSharedState* UiaActionCall::Target() const
	{
		return m_target;
	}

	SemanticActionRequest const& UiaActionCall::Request() const
	{
		return m_request;
	}

	bool UiaActionCall::Abandoned() const
	{
		return m_abandoned.load(std::memory_order_acquire) != 0;
	}

	void UiaActionCall::Abandon()
	{
		m_abandoned.store(1, std::memory_order_release);
	}

	void UiaActionCall::Complete(HRESULT result)
	{
		m_result.store(result, std::memory_order_relaxed);
		m_handled.store(1, std::memory_order_release);
	}

	HRESULT UiaActionCall::Result() const
	{
		return m_result.load(std::memory_order_relaxed);
	}

	bool UiaActionCall::Handled() const
	{
		return m_handled.load(std::memory_order_acquire) != 0;
	}

	HRESULT UiaStatusFromEngineStatus(EStatus status)
	{
		switch (status)
		{
			case EStatus::Success: { return S_OK; }
			case EStatus::InvalidArgument:
			case EStatus::InvalidStruct:
			case EStatus::SchemaMismatch:
			case EStatus::UnknownType:
			case EStatus::UnknownResource: { return E_INVALIDARG; }
			case EStatus::AbiMismatch:
			case EStatus::InvalidHandle:
			case EStatus::StaleHandle:
			case EStatus::StaleRevision:
			case EStatus::InvalidTree:
			case EStatus::DeviceLost: { return UIA_E_ELEMENTNOTAVAILABLE; }
			case EStatus::WrongThread: { return RPC_E_WRONG_THREAD; }
			case EStatus::UnsupportedFeature: { return UIA_E_NOTSUPPORTED; }
			case EStatus::BufferTooSmall:
			case EStatus::QueueOverflow:
			case EStatus::ResourceLimit: { return E_OUTOFMEMORY; }
			case EStatus::MissingAsset:
			case EStatus::ResourceInUse:
			case EStatus::InternalError: { return E_FAIL; }
			default: throw EngineException(EStatus::InternalError, std::format("unknown engine status {}", static_cast<std::int32_t>(status)));
		}
	}

	bool UiaSharedState::Bind(HWND hwnd)
	{
		auto lock = std::lock_guard(m_mutex);

		// A context that has been shut down never becomes available again, so a late WM_GETOBJECT
		// arriving during teardown cannot resurrect it.
		if (m_shutdown != 0 || hwnd == nullptr)
			return false;

		// Once bound, only the same window is accepted; another window asking for a provider is
		// rejected so the host's own window procedure can answer it.
		if (m_available != 0)
			return m_hwnd == hwnd;

		m_hwnd = hwnd;
		m_owner_thread_id = GetCurrentThreadId();
		m_available = 1;
		return true;
	}

	HWND UiaSharedState::Window() const
	{
		auto lock = std::lock_guard(m_mutex);
		return m_hwnd;
	}

	bool UiaSharedState::Available() const
	{
		auto lock = std::lock_guard(m_mutex);
		return m_available != 0;
	}

	void UiaSharedState::Publish(std::shared_ptr<UiaSnapshot const> snapshot)
	{
		auto lock = std::lock_guard(m_mutex);
		if (m_available == 0)
			return;

		m_snapshot = std::move(snapshot);
	}

	std::shared_ptr<UiaSnapshot const> UiaSharedState::Snapshot() const
	{
		auto lock = std::lock_guard(m_mutex);
		return m_available != 0 ? m_snapshot : nullptr;
	}

	HRESULT UiaSharedState::InvokeAction(SemanticActionRequest const& request)
	{
		// Decrement the in-flight count however this function exits, so a failed or timed-out send
		// cannot permanently block Shutdown from disconnecting providers.
		struct InFlightGuard
		{
			UiaSharedState* m_state;
			~InFlightGuard()
			{
				auto lock = std::lock_guard(m_state->m_mutex);
				--m_state->m_actions_in_flight;
			}
		};

		auto hwnd = HWND{};
		auto* call = static_cast<UiaActionCall*>(nullptr);
		{
			auto lock = std::lock_guard(m_mutex);
			if (m_available == 0 || m_hwnd == nullptr)
				return UIA_E_ELEMENTNOTAVAILABLE;

			// An owner thread that has already failed to collect this many calls is not pumping at
			// all, so refusing immediately keeps the pending list bounded and answers the client
			// with the same result the wait would have produced.
			if (m_pending_calls.size() >= UiaMaxPendingActionCalls)
				return UIA_E_TIMEOUT;

			// Created with the pending list's reference; the second is this sender's, so the call
			// survives whichever of the two finishes with it last.
			call = new UiaActionCall(this, request);
			call->AddRef();
			m_pending_calls.push_back(call);

			hwnd = m_hwnd;
			++m_actions_in_flight;
		}
		auto const guard = InFlightGuard{ this };

		// SendMessageTimeout delivers directly (no timeout involved) when this already is the owner
		// thread, so there is one code path for both cases. SMTO_ABORTIFHUNG bounds the wait against
		// a hung owner thread, and the explicit timeout bounds it against a merely busy one. The
		// mutex is deliberately not held: the owner thread takes it while publishing.
		auto answer = DWORD_PTR{};
		auto const sent = SendMessageTimeoutW(hwnd, UiaActionMessageId(), UiaActionCookie(), reinterpret_cast<LPARAM>(call), SMTO_ABORTIFHUNG | SMTO_ERRORONEXIT, UiaActionTimeoutMs, &answer);

		// Giving up on the wait does not cancel the send; the owner thread may still dispatch this
		// message later. Abandoning the call - rather than retiring it - leaves the pending list
		// owning live storage for that delivery to find, and tells the receiver not to run an action
		// whose caller has already been told it did not happen.
		auto const abandoned = sent == 0;
		{
			auto lock = std::lock_guard(m_mutex);
			if (abandoned)
			{
				call->Abandon();
			}
			else
			{
				auto const it = std::find(m_pending_calls.begin(), m_pending_calls.end(), call);
				if (it != m_pending_calls.end())
				{
					m_pending_calls.erase(it);
					call->Release();
				}
			}
		}

		// The sender's own reference is still held here, so reading the outcome is safe whether or
		// not the receiver has finished with the call.
		auto const result =
			abandoned ? (GetLastError() == ERROR_TIMEOUT ? static_cast<HRESULT>(UIA_E_TIMEOUT) : static_cast<HRESULT>(UIA_E_ELEMENTNOTAVAILABLE)) :
			!call->Handled() ? static_cast<HRESULT>(UIA_E_ELEMENTNOTAVAILABLE) : // Delivered but unclaimed: the host never routed it to View3DUI_ProcessWindowMessage.
			call->Result();

		call->Release();
		return result;
	}

	bool UiaSharedState::ExecuteMarshalledCall(UiaActionCall* call, std::function<HRESULT(SemanticActionRequest const&)> const& executor)
	{
		// Membership of the pending list, tested by pointer alone, is what makes this safe: the
		// registered message id is process-global, so both another context's call and a stale
		// pointer from a call that has already been retired can legitimately reach this window
		// procedure. Neither is dereferenced.
		auto owned = static_cast<UiaActionCall*>(nullptr);
		auto available = false;
		{
			auto lock = std::lock_guard(m_mutex);
			auto const it = std::find(m_pending_calls.begin(), m_pending_calls.end(), call);
			if (it == m_pending_calls.end())
				return false;

			// Hold the call across the executor independently of the sender, which may return at any
			// moment once it has timed out.
			owned = *it;
			owned->AddRef();
			available = m_available != 0;

			// A late delivery of an abandoned call retires it here: its storage is freed and the
			// action is deliberately not run.
			if (owned->Abandoned())
			{
				m_pending_calls.erase(it);
				owned->Release();
			}
		}

		// Release this function's own reference however it exits.
		struct CallGuard
		{
			UiaActionCall* m_call;
			~CallGuard() { m_call->Release(); }
		};
		auto const call_guard = CallGuard{ owned };

		if (owned->Abandoned())
			return true;

		if (!available)
		{
			owned->Complete(UIA_E_ELEMENTNOTAVAILABLE);
			return true;
		}

		// The executor runs engine code that publishes a new snapshot, which takes this mutex, so
		// it must be called with the lock released.
		owned->Complete(executor(owned->Request()));
		return true;
	}

	void UiaSharedState::RegisterProvider(IRawElementProviderSimple* provider)
	{
		auto lock = std::lock_guard(m_mutex);

		// After shutdown there is nothing left to disconnect from, and every provider call already
		// fails, so tracking a newly created provider would only keep a dead pointer around.
		if (m_shutdown != 0)
			return;

		if (m_providers.size() >= UiaMaxTrackedProviders)
		{
			// Past the cap the registry stops growing. Correctness is unaffected because every
			// provider re-checks availability on entry; only eager disconnection is skipped.
			m_registry_full = 1;
			return;
		}

		m_providers.push_back(provider);
	}

	bool UiaSharedState::ReleaseProvider(IRawElementProviderSimple* provider, std::atomic<ULONG>& refs, ULONG& remaining)
	{
		auto lock = std::lock_guard(m_mutex);
		remaining = refs.fetch_sub(1, std::memory_order_acq_rel) - 1;
		if (remaining != 0)
			return false;

		// The count reaching zero and the registry losing the pointer happen in one critical
		// section, so Shutdown - which AddRefs everything it finds - can never see an object whose
		// final release is already under way.
		if (provider != nullptr)
		{
			auto const it = std::find(m_providers.begin(), m_providers.end(), provider);
			if (it != m_providers.end())
				m_providers.erase(it);
		}

		// The registry can accept entries again once it has room, so a burst of short-lived
		// providers does not permanently disable eager disconnection.
		if (m_registry_full != 0 && m_providers.size() < UiaMaxTrackedProviders)
			m_registry_full = 0;

		return true;
	}

	std::size_t UiaSharedState::PendingCallCount() const
	{
		auto lock = std::lock_guard(m_mutex);
		return m_pending_calls.size();
	}

	std::vector<IRawElementProviderSimple*> UiaSharedState::Shutdown()
	{
		auto providers = std::vector<IRawElementProviderSimple*>{};
		auto calls = std::vector<UiaActionCall*>{};
		{
			auto lock = std::lock_guard(m_mutex);

			// Teardown happens exactly once however it is reached, so an explicit ContextDestroy
			// followed by the bridge's destructor cannot disconnect or release twice.
			if (m_shutdown != 0)
				return {};

			m_shutdown = 1;
			m_available = 0;
			m_hwnd = nullptr;
			m_owner_thread_id = 0;
			m_snapshot.reset();

			// Abandoned calls are owned solely by this list, so taking it here is what frees them.
			// A call a sender is still waiting on survives on that sender's own reference.
			calls.swap(m_pending_calls);

			// UiaDisconnectProvider blocks until in-flight calls to the provider return. A call that
			// is already marshalling is blocked in SendMessageTimeout on this very thread, so
			// disconnecting now would stall teardown for the whole timeout. Clearing availability
			// already makes every later provider call fail with UIA_E_ELEMENTNOTAVAILABLE, which is
			// the same thing a client observes from a disconnected provider, so skipping the
			// disconnect is safe.
			if (m_actions_in_flight == 0)
			{
				// Each provider is kept alive across the disconnect by this reference; the caller
				// releases it once UiaDisconnectProvider has returned. The registry is emptied at
				// the same time so nothing can be disconnected twice.
				providers = std::move(m_providers);
				for (auto* provider : providers)
					provider->AddRef();
			}

			m_providers.clear();
			m_registry_full = 0;
		}

		for (auto* call : calls)
			call->Release();

		return providers;
	}

	UiaSharedState::~UiaSharedState()
	{
		// The shared state outlives every provider by construction - each holds a shared_ptr to it -
		// so only abandoned action calls can still be owned here, and only when the state is
		// destroyed without an explicit Shutdown.
		for (auto* call : m_pending_calls)
			call->Release();
	}

	UiaBridge::UiaBridge()
		: m_shared(std::make_shared<UiaSharedState>())
	{}

	UiaBridge::~UiaBridge()
	{
		// A moved-from bridge has no state left to tear down.
		if (m_shared != nullptr)
			Shutdown();
	}

	bool UiaBridge::Bound() const
	{
		return m_shared != nullptr && m_shared->Available();
	}

	bool UiaBridge::Bind(HWND hwnd)
	{
		return m_shared != nullptr && m_shared->Bind(hwnd);
	}

	std::shared_ptr<UiaSharedState> const& UiaBridge::Shared() const
	{
		return m_shared;
	}

	std::int32_t UiaBridge::HandleGetObject(HWND hwnd, WPARAM wparam, LPARAM lparam, SemanticSnapshot const& semantics, ViewportState const& viewport, std::uint64_t revision, LRESULT& result)
	{
		// Only the UI Automation root object is ours; MSAA and every other object id belongs to the
		// host's window procedure and must be passed through untouched.
		if (m_shared == nullptr || static_cast<long>(lparam) != static_cast<long>(UiaRootObjectId))
			return 0;

		// Binding on first use is what removes the need for a separate registration export; a
		// different window afterwards is declined rather than rebound.
		if (!m_shared->Bind(hwnd))
			return 0;

		// Publish before the provider exists, so the first element the client can reach already has
		// an immutable snapshot behind it. A host that renders on demand may not call Update again
		// for a long time, and an engine with nothing in it still publishes a valid empty snapshot
		// rather than leaving the client to see an unavailable element.
		Publish(semantics, viewport, revision);

		auto* provider = CreateUiaRootProvider(m_shared);
		if (provider == nullptr)
			return 0;

		result = UiaReturnRawElementProvider(hwnd, wparam, lparam, provider);
		provider->Release();
		return 1;
	}

	std::int32_t UiaBridge::HandleActionMessage(HWND hwnd, WPARAM wparam, LPARAM lparam, std::function<InputResult(SemanticActionRequest const&)> const& executor, std::int32_t& invalidate)
	{
		if (m_shared == nullptr || lparam == 0)
			return 0;

		// The registered message id is system-global, so an arbitrary sender can deliver this
		// message with an arbitrary LPARAM. The cookie is checked first, and the pending-list lookup
		// inside ExecuteMarshalledCall then confirms the pointer is one of this context's own live
		// calls; nothing dereferences the LPARAM until both have passed.
		if (wparam != UiaActionCookie())
			return 0;

		// The window must be the one this context is bound to; another context in this process
		// answers for its own window.
		if (hwnd != m_shared->Window())
			return 0;

		auto* call = reinterpret_cast<UiaActionCall*>(lparam);
		auto const run = [&](SemanticActionRequest const& request) -> HRESULT
		{
			// This is the boundary between engine code, which reports failure by throwing, and COM,
			// which must not see an exception; every failure maps to a specific status here.
			try
			{
				auto const result = executor(request);
				if (result.invalidate)
					invalidate = 1;

				return S_OK;
			}
			catch (EngineException const& ex)
			{
				return UiaStatusFromEngineStatus(ex.Status());
			}
			catch (std::bad_alloc const&)
			{
				return E_OUTOFMEMORY;
			}
			catch (std::exception const&)
			{
				return E_FAIL;
			}
		};

		return m_shared->ExecuteMarshalledCall(call, run) ? 1 : 0;
	}

	void UiaBridge::Publish(SemanticSnapshot const& semantics, ViewportState const& viewport, std::uint64_t revision) noexcept
	{
		// Nothing is published until a client has asked for a provider, so a context no assistive
		// technology has ever touched pays nothing per frame.
		if (m_shared == nullptr || !m_shared->Available())
			return;

		// Publication runs inside Update and must never be able to reject a frame, so this is a
		// no-throw boundary: an allocation failure simply leaves the previous publication in place
		// and the client re-reads it, which is stale but never wrong about identity or structure.
		try
		{
			auto snapshot = BuildUiaSnapshot(semantics, viewport, revision, m_publish_sequence + 1);
			auto const diff = DiffUiaSnapshots(m_published.get(), *snapshot);
			m_shared->Publish(snapshot);

			auto const previous = std::move(m_published);
			m_published = std::move(snapshot);
			++m_publish_sequence;

			RaiseUiaEvents(m_shared, previous.get(), *m_published, diff);
		}
		catch (std::bad_alloc const&)
		{
		}
		catch (std::exception const&)
		{
		}
	}

	void UiaBridge::Shutdown() noexcept
	{
		if (m_shared == nullptr)
			return;

		// Teardown must not be able to throw out of a destructor or an ABI export, and there is no
		// recovery available: the context is going away regardless of what fails here.
		try
		{
			auto const providers = m_shared->Shutdown();
			m_published.reset();

			// Disconnecting tells any client holding a reference to release it now rather than on
			// its next failed call. It is skipped entirely when no client is listening, because
			// there is then nobody to notify.
			if (UiaClientsAreListening() != FALSE)
			{
				for (auto* provider : providers)
					UiaDisconnectProvider(provider);
			}

			for (auto* provider : providers)
				provider->Release();
		}
		catch (std::exception const&)
		{
		}
	}
}
