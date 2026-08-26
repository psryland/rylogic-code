//*********************************************
// View3DUI
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Internal (non-ABI) native engine: one instance per UI context. Owns the retained control tree,
// deterministic layout, style/transition resolution, the normalized input state machine, the
// bounded event queue, and the semantic snapshot builder. The C ABI in projects/rylogic/view3d-ui
// /dll is a thin, per-context, owner-thread-checked wrapper around this class.
#pragma once
#include "pr/view3d-ui/forward.h"
#include "pr/view3d-ui/types.h"
#include "pr/view3d-ui/draw_packet.h"

namespace pr::view3d::ui
{
	// Internal collaborators of UiEngine, declared here only so the private members below can be
	// expressed. Their definitions stay inside the module (src/input.h, src/text_shaper.h) so this
	// header remains dependency-minimal.
	struct InputResult;
	struct InputTextRecord;
	struct SemanticActionRequest;
	class TextShaper;
	class UiaBridge;

	// Thrown by every validating/rejecting operation in this module. The DLL layer catches this
	// exactly (no broad catch(...) is used elsewhere) and translates m_status/what() into the
	// stable EStatus/last-error ABI contract.
	class EngineException : public std::runtime_error
	{
		EStatus m_status;

	public:
		EngineException(EStatus status, std::string_view message)
			: std::runtime_error(std::string(message))
			, m_status(status)
		{}
		EStatus Status() const
		{
			return m_status;
		}
	};

	// Returns a Config populated with bounded, deterministic default capacities (section 9.2).
	Config DefaultConfig();

	// Throws EngineException(InvalidStruct) or EngineException(SchemaMismatch) if 'header' does not describe a
	// valid instance of a T-sized record at VIEW3D_UI_STRUCT_VERSION. Used for every headered
	// struct the ABI receives, both singular parameters and transaction array elements.
	template <typename T>
	void ValidateHeader(StructHeader const& header, char const* what)
	{
		if (header.size < sizeof(T))
			throw EngineException(EStatus::InvalidStruct, std::format("{}: struct size {} is smaller than the minimum required {}", what, header.size, sizeof(T)));
		if (header.version != VIEW3D_UI_STRUCT_VERSION)
			throw EngineException(EStatus::SchemaMismatch, std::format("{}: struct version {} does not match the supported version {}", what, header.version, VIEW3D_UI_STRUCT_VERSION));
	}

	// One owner-thread UI context: retained tree, layout, input, events, semantics, draw packets.
	// Move-only, single-writer; the DLL layer serializes all access to one instance to its owning
	// thread (section 8.2) so this class itself performs no internal locking.
	class UiEngine
	{
		struct Impl;
		std::unique_ptr<Impl> m_impl;

	public:
		explicit UiEngine(Config const& config);
		~UiEngine();
		UiEngine(UiEngine&&) noexcept;
		UiEngine& operator=(UiEngine&&) noexcept;
		UiEngine(UiEngine const&) = delete;
		UiEngine& operator=(UiEngine const&) = delete;

		// Validate and atomically apply one delta transaction (section 5.3). Throws EngineException on
		// any rejection; the previously accepted tree is left completely unmodified.
		void TransactionApply(Transaction const& txn);

		// Advance host time, recompute layout/style-transitions/semantics/draw-packets from the
		// currently accepted tree (section 7.4/9.3). Must be called at least once before hit
		// testing or semantics reflect a non-empty tree.
		void Update(ViewportState const& viewport);

		// Translate one raw Win32 message into the same normalized input state machine used by
		// InputInject (section 7.1). '*consumed'/'*invalidate' are 0/1. Returns the same value as
		// '*consumed' for convenience.
		std::int32_t ProcessWindowMessage(HWND hwnd, UINT msg, WPARAM wparam, LPARAM lparam, LRESULT& result, std::int32_t& invalidate);

		// Inject one already-normalized input record, identical in effect to a translated raw
		// message (section 7.2, used by deterministic tests).
		void InputInject(NormalizedInput const& input);

		// Inject one normalized input record that carries text: EInputKind::TextInput,
		// CompositionUpdate or CompositionCommit. 'payload' is borrowed for the duration of the
		// call. This is what lets a test drive a complete IME composition lifecycle - start,
		// update, commit or cancel - through exactly the same state machine the raw WM_IME_*
		// messages use, with no IME installed. Throws EngineException for text that is not
		// well-formed UTF-8, offsets that are not on code-point boundaries, a record that breaks
		// the composition ordering, or a composition whose control is no longer the composing one.
		void InputInjectText(NormalizedInput const& input, InputTextPayload const& payload);

		// The caret rectangle of 'control_id' in DIPs within the render target, measured from the
		// same shaped layout the renderer draws. Sets 'out_valid' to 0 (and zeroes 'out_caret_dip')
		// when the control is not a focused, laid-out, editable control.
		void CaretGeometry(ControlId control_id, Rect& out_caret_dip, std::int32_t& out_valid);

		// Pending event queue accessors (section 5.4). Count/PayloadBytes report the true pending
		// size regardless of the caller's buffer; Copy drains the entire queue and clears it.
		std::uint32_t EventCount() const;
		std::uint32_t EventPayloadBytesPending() const;
		void EventsCopy(std::span<Event> events, std::span<std::byte> payload_blob);

		// Semantic snapshot accessors (section 5.5), recomputed by the most recent Update() call.
		std::uint32_t SemanticCount() const;
		std::uint32_t SemanticTextBytesPending() const;
		void SemanticsCopy(std::span<SemanticNode> nodes, std::span<char> text_blob);

		// Bounded runtime counters and the last rejected-transaction failure category.
		Diagnostics DiagnosticsGet() const;

		// Internal-only accessor for the immutable renderer-neutral draw packet built by the most
		// recent Update() call. Not part of the public ABI; consumed natively by this module's own
		// host_bridge/renderer (dll/host_bridge.cpp, dll/renderer.cpp) during the FinalOverlay pass.
		DrawPacket const& DrawPackets() const;

		// Internal-only hook for the DLL's renderer to surface a rendering-side fault (currently
		// only EStatus::DeviceLost) through the same Diagnostics::last_failure_status field used
		// by rejected transactions/input, without inventing a second status surface. Bounded,
		// no-throw; not part of the public ABI.
		void NotifyRendererStatus(EStatus status);

		// Apply one semantic action - the closed set an accessibility client can request - through
		// exactly the same owner-thread control logic, focus rules and typed event queue that
		// normalized input uses, so a UI Automation client and a keystroke are indistinguishable
		// downstream. No application or managed code is called. Throws EngineException for an
		// unknown control, malformed text, or an action the target does not currently advertise.
		// Internal-only; not part of the public ABI.
		InputResult ApplySemanticAction(SemanticActionRequest const& request);

		// Internal-only accessor for this context's UI Automation bridge. Not part of the public
		// ABI; used by the DLL layer to mark the bridge unavailable during context teardown.
		UiaBridge& Uia();

	private:
		// The single place every input record - translated or injected - reaches the state
		// machine, so failure bookkeeping (queue-overflow counting, last-failure status) and the
		// text hit-test context are applied identically to both paths.
		InputResult DispatchNormalizedInput(NormalizedInput const& input, InputTextRecord const* text_payload);

		// The lazily-created text shaper, or null when DirectWrite is unavailable on this machine.
		// A null result deliberately degrades text measurement rather than failing input.
		TextShaper* TextShaperOrNull();
	};
}
