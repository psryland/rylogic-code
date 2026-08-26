//*********************************************
// View3DUI
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// The owner-thread half of the Windows UI Automation bridge (implementation-plan.md M10): it owns
// the shared state COM providers read, answers WM_GETOBJECT from the raw-message path, executes
// marshalled semantic actions, and raises the minimum set of automation notifications.
//
// Threading contract, in one place because every rule below depends on the others:
//  - UiEngine is single-writer and owner-thread-only. No UI Automation entry point ever touches it.
//  - After each Update the owner thread publishes an immutable UiaSnapshot into UiaSharedState.
//    Provider property/navigation/text-range reads take a shared_ptr copy under a mutex and then
//    read the immutable value with no lock held at all.
//  - Mutating actions are marshalled synchronously to the owner thread's HWND with a process-wide
//    RegisterWindowMessageW message, bounded by SendMessageTimeout. A call already on the owner
//    thread is delivered by SendMessage, which dispatches the window procedure directly.
//  - A timed-out SendMessageTimeout unblocks the sender but leaves the message in the receiver's
//    queue, so the call itself is heap allocated and reference counted and is owned by the shared
//    state's pending list, never by the sender's stack. Late delivery therefore always finds live
//    storage, and teardown always frees it.
//  - No COM object retains a UiEngine or HWND pointer of its own; everything reachable from a
//    provider is either immutable or guarded by UiaSharedState::m_mutex, and every field the owner
//    thread owns is cleared by Shutdown before the engine goes away.
//  - A provider's final reference release and its removal from the shared registry happen together
//    under that same mutex, so teardown can never observe a provider that is already dying.
#pragma once
#include "pr/view3d-ui/forward.h"
#include "pr/view3d-ui/types.h"
#include "input.h"
#include "uia_snapshot.h"

namespace pr::view3d::ui
{
	// Bounded wait for one cross-thread semantic action. A client whose action cannot be delivered
	// within this window is told UIA_E_TIMEOUT rather than being blocked on an owner thread that
	// may be busy rendering or stopped in a debugger.
	inline constexpr std::uint32_t UiaActionTimeoutMs = 2000;

	// Upper bound on how many live provider objects one context tracks for eager disconnection.
	// Past this the bridge stops recording them; correctness is unaffected because every provider
	// re-checks availability on entry, only the eager disconnect optimization is skipped.
	inline constexpr std::size_t UiaMaxTrackedProviders = 4096;

	// Upper bound on how many marshalled action calls one context may have outstanding. Each entry
	// is one sender either waiting for the owner thread or having given up on a message that may
	// still be delivered. An owner thread that has accumulated this many undelivered calls is not
	// answering at all, so further actions fail immediately instead of growing the list.
	inline constexpr std::size_t UiaMaxPendingActionCalls = 64;

	// The process-wide registered message used to marshal a semantic action to the owner thread.
	// RegisterWindowMessageW guarantees the value cannot collide with another component's private
	// message; the per-call target cookie below guarantees it cannot be mistaken for another
	// context's call even within this process.
	UINT UiaActionMessageId();

	// The WPARAM every marshalled action carries. The registered message id is system-global, so a
	// foreign window could legitimately send this message with an arbitrary LPARAM; this cookie is
	// the address of a per-module object, which is neither guessable from outside the process nor
	// shared with another loaded copy of this module. A message without it is passed on untouched
	// and its LPARAM is never dereferenced.
	WPARAM UiaActionCookie();

	class UiaSharedState;

	// One in-flight marshalled semantic action.
	//
	// SendMessageTimeout can return before the receiving thread has finished - or even started -
	// processing the message, so this storage must outlive the sender's wait. It is heap allocated
	// and reference counted: the shared state's pending list holds one reference and the sender
	// holds another, and whichever is released last frees it. The request is held by value because
	// the caller's request object is gone once the sender stops waiting.
	class UiaActionCall
	{
		std::atomic<ULONG> m_refs;
		UiaSharedState* m_target;         // Cookie: which context this call is addressed to.
		SemanticActionRequest m_request;
		std::atomic<std::int32_t> m_handled;   // Set by the owner thread; still 0 means the host never routed the message.
		std::atomic<std::int32_t> m_abandoned; // Set when the sender has stopped waiting for a result.
		std::atomic<HRESULT> m_result;

		~UiaActionCall() = default;

	public:

		UiaActionCall(UiaSharedState* target, SemanticActionRequest const& request);
		UiaActionCall(UiaActionCall const&) = delete;
		UiaActionCall& operator=(UiaActionCall const&) = delete;

		// Reference counting. The call is created with a single reference owned by the caller,
		// which hands it to the pending list and then takes its own.
		void AddRef();
		void Release();

		// The context this call is addressed to, and the action it asks for.
		UiaSharedState* Target() const;
		SemanticActionRequest const& Request() const;

		// True once the sender has stopped waiting. A late delivery must not run the action,
		// because the client was already told the action did not happen.
		bool Abandoned() const;
		void Abandon();

		// Record the outcome of executing this call on the owner thread.
		void Complete(HRESULT result);

		// The outcome, and whether the owner thread ever saw the call at all.
		HRESULT Result() const;
		bool Handled() const;
	};

	// State shared between the owner thread and every live COM provider. Held by shared_ptr so a
	// provider that outlives its context keeps the state (and therefore its own availability flag)
	// alive without keeping the engine alive.
	class UiaSharedState
	{
		mutable std::mutex m_mutex;
		std::shared_ptr<UiaSnapshot const> m_snapshot;
		std::vector<IRawElementProviderSimple*> m_providers;  // Weak; each provider removes itself as its last reference goes.
		std::vector<UiaActionCall*> m_pending_calls;          // Owning; one reference per entry.
		HWND m_hwnd = nullptr;
		std::uint32_t m_owner_thread_id = 0;
		std::uint32_t m_actions_in_flight = 0;
		std::int32_t m_available = 0;
		std::int32_t m_shutdown = 0;
		std::int32_t m_registry_full = 0;

	public:
		UiaSharedState() = default;
		UiaSharedState(UiaSharedState const&) = delete;
		UiaSharedState& operator=(UiaSharedState const&) = delete;
		~UiaSharedState();

		// Bind this state to the owner thread's window. Returns false when it is already bound to a
		// different window, which is how a second HWND asking for a provider is rejected.
		bool Bind(HWND hwnd);

		// The bound window, or null once unavailable. Only used to convert client coordinates to
		// screen coordinates and to address the marshalling message.
		HWND Window() const;

		// True while the owning context is alive and bound.
		bool Available() const;

		// Replace the published snapshot. Owner thread only.
		void Publish(std::shared_ptr<UiaSnapshot const> snapshot);

		// The currently published snapshot, or null when none has been published or the context is
		// gone. Callable from any thread.
		std::shared_ptr<UiaSnapshot const> Snapshot() const;

		// Request one semantic action, executing it on the owner thread. Returns S_OK, or
		// UIA_E_ELEMENTNOTAVAILABLE when the context is gone or the host never routed the message,
		// or UIA_E_TIMEOUT when the owner thread did not answer within UiaActionTimeoutMs, or the
		// mapped failure the action itself produced. Callable from any thread.
		HRESULT InvokeAction(SemanticActionRequest const& request);

		// Execute one marshalled call after verifying it is one of this state's own outstanding
		// calls. Returns false when the pointer is not ours - a foreign message, or a call that has
		// already been retired - in which case the message must be left alone and the pointer must
		// not be dereferenced.
		bool ExecuteMarshalledCall(UiaActionCall* call, std::function<HRESULT(SemanticActionRequest const&)> const& executor);

		// Track a live provider for eager disconnection. Refused after shutdown, because there is
		// then nothing left to disconnect from and the provider already fails every call.
		void RegisterProvider(IRawElementProviderSimple* provider);

		// Drop one reference to a provider, removing it from the registry in the same critical
		// section as the reference reaching zero. Returns true when the caller owns the last
		// reference and must now delete the object. This atomicity is what stops Shutdown
		// resurrecting a provider whose final Release is already under way. A null key means the
		// object was never registered and only needs the reference transition.
		bool ReleaseProvider(IRawElementProviderSimple* provider, std::atomic<ULONG>& refs, ULONG& remaining);

		// How many marshalled calls are still outstanding, including ones a timed-out sender has
		// abandoned. Exposed so the module's own tests can prove late delivery frees its storage.
		std::size_t PendingCallCount() const;

		// Mark the context unavailable and drop everything the owner thread owns, freeing every
		// outstanding action call. Every subsequent provider call fails with
		// UIA_E_ELEMENTNOTAVAILABLE. Returns the providers that should be disconnected, each with
		// one reference taken for the caller, or an empty list when disconnection would risk
		// blocking against a call that is already marshalling. Idempotent: a second call returns an
		// empty list and changes nothing. Callable from any thread.
		std::vector<IRawElementProviderSimple*> Shutdown();
	};

	// The owner-thread bridge object, one per UI context, owned by UiEngine.
	class UiaBridge
	{
		std::shared_ptr<UiaSharedState> m_shared;
		std::shared_ptr<UiaSnapshot const> m_published; // Owner-thread copy of the last publication, kept only for diffing.
		std::uint64_t m_publish_sequence = 0;

	public:
		UiaBridge();
		~UiaBridge();
		UiaBridge(UiaBridge&&) noexcept = default;
		UiaBridge& operator=(UiaBridge&&) noexcept = default;
		UiaBridge(UiaBridge const&) = delete;
		UiaBridge& operator=(UiaBridge const&) = delete;

		// True once a window has asked for (and been given) a provider.
		bool Bound() const;

		// Bind to 'hwnd' without a WM_GETOBJECT round trip, so a headless test can publish and
		// diff snapshots exactly as a real client would cause. Returns false for a second window.
		bool Bind(HWND hwnd);

		// Answer WM_GETOBJECT for UiaRootObjectId by returning the fragment root for this context.
		// The committed semantics are published as part of binding, before the provider can be
		// handed out, so the very first element a client sees already has an immutable snapshot to
		// read - which is what a render-on-demand host that has not called Update since the client
		// attached depends on. An engine with nothing in it publishes a valid empty snapshot.
		// Returns 0 (leaving 'result' untouched) for any other object id, or when 'hwnd' is not the
		// window this bridge is bound to, so the host's window procedure handles it unmodified.
		std::int32_t HandleGetObject(HWND hwnd, WPARAM wparam, LPARAM lparam, SemanticSnapshot const& semantics, ViewportState const& viewport, std::uint64_t revision, LRESULT& result);

		// Execute one marshalled semantic action addressed to this context, mapping the executor's
		// EStatus failures onto UI Automation HRESULTs. Returns 0 when the message is not one of
		// this context's own calls - a foreign sender, or another context in this process - in
		// which case the message must be passed on unmodified and its LPARAM never dereferenced.
		std::int32_t HandleActionMessage(HWND hwnd, WPARAM wparam, LPARAM lparam, std::function<InputResult(SemanticActionRequest const&)> const& executor, std::int32_t& invalidate);

		// The state every provider for this context shares. Exposed so the provider factories - and
		// the module's own tests - can create providers without a WM_GETOBJECT round trip. Never
		// null for a bridge that has not been moved from.
		std::shared_ptr<UiaSharedState> const& Shared() const;

		// Publish the semantics of the update that just completed and raise the notifications the
		// difference from the previous publication implies. No-op until bound, so a context no
		// accessibility client has ever asked about pays nothing. Owner thread only, no-throw.
		void Publish(SemanticSnapshot const& semantics, ViewportState const& viewport, std::uint64_t revision) noexcept;

		// Mark the context unavailable and disconnect providers where that is safe. Callable from
		// any thread - View3DUI_UiContextAbandon deliberately allows a non-owner thread to
		// surrender a context - no-throw, and idempotent, so an explicit ContextDestroy followed by
		// this bridge's destructor disconnects and releases exactly once.
		void Shutdown() noexcept;
	};

	// Map one engine status onto the UI Automation/COM status a provider must report for it. Every
	// EStatus is mapped explicitly so a new status cannot silently acquire a success-shaped result.
	HRESULT UiaStatusFromEngineStatus(EStatus status);
}
