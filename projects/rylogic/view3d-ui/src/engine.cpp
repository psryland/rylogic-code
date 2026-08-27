//*********************************************
// View3DUI
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Native engine implementation: owns the retained tree, layout cache, style resolver, input state
// machine, event queue, semantics/draw-packet caches for one owner-thread UI context. This is the
// single place that sequences those subsystems together (implementation-plan.md section 4.2); the
// C ABI in projects/rylogic/view3d-ui/dll only adds handle/thread validation around this class.
#include "pr/view3d-ui/engine.h"
#include "tree.h"
#include "layout.h"
#include "style.h"
#include "input.h"
#include "win32_input.h"
#include "events.h"
#include "semantics.h"
#include "draw_packet_builder.h"
#include "text_shaper.h"
#include "text_layout.h"
#include "uia_bridge.h"

namespace pr::view3d::ui
{
	Config DefaultConfig()
	{
		// Bounded, deterministic defaults sized for the M2 demo vertical slice (section 9.2): a
		// few hundred controls with headroom, not an unbounded/production-scale document.
		return Config{
			StructHeader{ sizeof(Config), VIEW3D_UI_STRUCT_VERSION },
			/* max_controls */ 4096,
			/* max_roots */ 64,
			/* max_tree_depth */ 64,
			/* max_operations_per_transaction */ 4096,
			/* max_blob_bytes */ 1u << 20,
			/* max_templates */ 256,
			/* max_styles */ 256,
			/* max_resources */ 256,
			/* max_glyph_cache_bytes */ 1u << 24,
			/* max_glyph_cache_pages */ 16,
			/* max_generated_vertices */ 1u << 16,
			/* max_generated_indices */ 1u << 18,
			/* max_queued_events */ 1024,
			/* max_semantic_records */ 4096,
			/* max_transition_duration_ms */ 2000.0f,
		};
	}

	// Internal engine state; see the per-member comments below for how each subsystem's bounded
	// memory relates to Config and to the accepted tree (pruned after every TransactionApply).
	struct UiEngine::Impl
	{
		Config m_config;
		TreeModel m_tree;
		std::unordered_map<ControlId, Rect> m_layout;
		std::unordered_map<ControlId, RootPlacement> m_placements; // One per root, from the same Update() as m_layout
		StyleResolver m_styles;
		InputState m_input;
		EventQueue m_events;
		Win32InputTranslatorState m_win32_translator;
		SemanticSnapshot m_semantics;
		DrawPacket m_draw_packet;
		ViewportState m_viewport{};
		double m_time_ms = 0.0;
		std::uint64_t m_visual_sequence = 0;
		std::uint64_t m_rejected_revision_attempts = 0;
		std::uint32_t m_event_overflow_count = 0;
		EStatus m_last_failure_status = EStatus::Success;

		// Created on first use by TextHitContextOf, because a headless context (every ABI/logic
		// unit test) never asks a text question and must not pay for a DirectWrite factory - and,
		// more importantly, must not fail to construct on a machine where DirectWrite is absent.
		// Kept here rather than in the DLL's Renderer so pointer caret placement works identically
		// whether or not the context was ever attached to a window.
		std::unique_ptr<TextShaper> m_text_shaper;

		// The Windows UI Automation bridge for this context. It stays inert - publishing nothing
		// and costing nothing per frame - until an accessibility client sends the window a
		// WM_GETOBJECT, so a context nothing is observing pays only for this member.
		UiaBridge m_uia;

		explicit Impl(Config const& config)
			: m_config(config)
			, m_events(config.max_queued_events)
		{}
	};

	namespace
	{
		// The set of control ids currently accepted in 'tree', used to bound-prune every
		// per-control runtime cache (input focus/hover/capture/text-edits, style transitions)
		// after a transaction changes the tree shape (section 8: no unbounded growth).
		std::unordered_set<ControlId> LiveControlIds(TreeModel const& tree)
		{
			std::unordered_set<ControlId> live_ids;
			live_ids.reserve(tree.m_controls.size());
			for (auto const& [id, node] : tree.m_controls)
				live_ids.insert(id);

			return live_ids;
		}

		// Turn one InputTextPayload into the borrowed record the state machine consumes, after
		// checking the fields the ABI itself cannot express. A null pointer with a non-zero length
		// is rejected here rather than being read.
		InputTextRecord BorrowTextPayload(InputTextPayload const& payload)
		{
			if (payload.text_length != 0 && payload.text_utf8 == nullptr)
				throw EngineException(EStatus::InvalidArgument, "InputTextPayload: null text with non-zero length");

			return InputTextRecord{
				.text = std::string_view(payload.text_utf8 != nullptr ? payload.text_utf8 : "", payload.text_length),
				.caret = payload.caret,
				.selection_start = payload.selection_start,
				.selection_end = payload.selection_end,
			};
		}
	}

	UiEngine::UiEngine(Config const& config)
		: m_impl(std::make_unique<Impl>(config))
	{}
	UiEngine::~UiEngine() = default;
	UiEngine::UiEngine(UiEngine&&) noexcept = default;
	UiEngine& UiEngine::operator=(UiEngine&&) noexcept = default;

	void UiEngine::TransactionApply(Transaction const& txn)
	{
		ValidateHeader<Transaction>(txn.header, "Transaction");

		// Snapshot the pre-transaction Tab order so a focus target removed/hidden/disabled by
		// this transaction can be reconciled relative to its former position (section 7.5).
		auto old_tab_order = ComputeTabOrder(m_impl->m_tree);

		TreeModel next;
		try
		{
			next = m_impl->m_tree.Apply(txn, m_impl->m_config);
		}
		catch (EngineException const& ex)
		{
			m_impl->m_rejected_revision_attempts++;
			m_impl->m_last_failure_status = ex.Status();
			throw; // the previously accepted tree is left completely unmodified
		}

		// The transaction is accepted from this point; commit it before any further bookkeeping
		// so a later failure (e.g. an event-queue overflow while reconciling focus) can never
		// roll back an already-valid revision.
		m_impl->m_tree = std::move(next);

		auto live_ids = LiveControlIds(m_impl->m_tree);
		m_impl->m_styles.Prune(live_ids);

		try
		{
			ReconcileFocusAfterTransaction(m_impl->m_tree, old_tab_order, m_impl->m_input, m_impl->m_events, m_impl->m_tree.m_revision);
		}
		catch (EngineException const& ex)
		{
			if (ex.Status() == EStatus::QueueOverflow)
				m_impl->m_event_overflow_count++;
			m_impl->m_last_failure_status = ex.Status();
			throw;
		}

		// Reconcile every TextBox's local pending-edit state against the newly-accepted descriptor
		// text (acknowledge/normalize-or-reject/preserve) before pruning entries for controls this
		// transaction removed; this never emits events or throws, so it cannot roll back the commit.
		ReconcileTextEditsAfterTransaction(m_impl->m_tree, m_impl->m_input);
		m_impl->m_input.Prune(live_ids);
	}

	void UiEngine::Update(ViewportState const& viewport)
	{
		ValidateHeader<ViewportState>(viewport.header, "ViewportState");

		// A camera the host claims is valid must be usable for deterministic projection; rejecting
		// it here keeps ProjectWorldRoot a total function that only ever culls, never throws.
		ValidateCameraState(viewport.camera);

		// Recompute layout/semantics into locals first so a bounds failure (below) can never leave
		// m_impl's published snapshot inconsistent with itself: SemanticsCopy/SemanticCount promise
		// to reflect the most recent *successful* Update() call, and semantics.m_nodes.size() must
		// never be observable above max_semantic_records even transiently.
		auto placements = ComputeRootPlacements(m_impl->m_tree, viewport);
		auto layout = ComputeLayout(m_impl->m_tree, placements);
		auto semantics = BuildSemanticSnapshot(m_impl->m_tree, layout, placements, m_impl->m_input, m_impl->m_tree.m_revision);
		if (semantics.m_nodes.size() > m_impl->m_config.max_semantic_records)
			throw EngineException(EStatus::ResourceLimit, std::format("semantic snapshot has {} records, exceeding max_semantic_records {}", semantics.m_nodes.size(), m_impl->m_config.max_semantic_records));

		// Every prior step succeeded; commit the whole observation (viewport/time/layout/semantics/
		// draw-packet) together so every accessor reflects one consistent host-time snapshot (section 9.3).
		m_impl->m_viewport = viewport;
		m_impl->m_time_ms = viewport.time_ms;
		m_impl->m_layout = std::move(layout);
		m_impl->m_placements = std::move(placements);
		m_impl->m_semantics = std::move(semantics);
		m_impl->m_visual_sequence++;
		m_impl->m_draw_packet = BuildDrawPacket(m_impl->m_tree, m_impl->m_layout, m_impl->m_placements, m_impl->m_styles, m_impl->m_input, m_impl->m_tree.m_revision, m_impl->m_visual_sequence, m_impl->m_time_ms, viewport.dpi);

		// Republish the accessibility projection from the committed snapshot, on the owner thread,
		// after everything else has succeeded. This is a no-throw step so a UI Automation failure
		// can never reject a frame the engine has already accepted.
		m_impl->m_uia.Publish(m_impl->m_semantics, m_impl->m_viewport, m_impl->m_tree.m_revision);
	}

	std::int32_t UiEngine::ProcessWindowMessage(HWND hwnd, UINT msg, WPARAM wparam, LPARAM lparam, LRESULT& result, std::int32_t& invalidate)
	{
		result = 0;
		invalidate = 0;

		// Accessibility messages are answered before any input translation, because neither is part
		// of the input vocabulary and both must work regardless of focus or composition state. This
		// is what removes the need for a separate registration entry point: the host already
		// forwards every raw message here.
		if (msg == WM_GETOBJECT)
		{
			return m_impl->m_uia.HandleGetObject(hwnd, wparam, lparam, m_impl->m_semantics, m_impl->m_viewport, m_impl->m_tree.m_revision, result);
		}
		if (msg == UiaActionMessageId())
		{
			// A semantic action marshalled from a COM worker thread. It runs here, on the owner
			// thread, through exactly the same control logic as translated input.
			auto const run = [this](SemanticActionRequest const& request) { return ApplySemanticAction(request); };
			return m_impl->m_uia.HandleActionMessage(hwnd, wparam, lparam, run, invalidate);
		}

		// The translator keeps its own view of whether a composition is running, but the state
		// machine is authoritative: focus can move, a control can leave the tree, and the
		// application can cancel an edit, all without the IME being told. Resynchronising here
		// means a follow-up IME message can never be translated against a stale assumption and
		// then rejected as an out-of-order record.
		auto const editable_focus = HasEditableFocus(m_impl->m_tree, m_impl->m_input);
		if (m_impl->m_input.m_composing_id != 0 && !editable_focus)
			CancelActiveComposition(m_impl->m_input);

		m_impl->m_win32_translator.composition_active = m_impl->m_input.m_composing_id != 0 ? 1 : 0;
		auto const was_composing = m_impl->m_input.m_composing_id != 0;

		auto translated = TranslateWindowMessage(hwnd, msg, wparam, lparam, m_impl->m_viewport, m_impl->m_time_ms, editable_focus, m_impl->m_win32_translator);
		if (translated.recognised == 0)
			return 0; // this message is not part of the UI input vocabulary; the host's own window procedure handles it unmodified

		auto previous_captured_id = m_impl->m_input.m_captured_id;

		// One message can imply several records - a WM_IME_COMPOSITION carrying both a result
		// string and a new composition string is the motivating case - so they are applied in the
		// order the translator produced them and their outcomes combined.
		auto consumed = false;
		auto invalidated = false;
		for (auto const& record : translated.inputs)
		{
			auto const payload = InputTextRecord{
				.text = record.text,
				.caret = record.caret,
				.selection_start = record.selection_start,
				.selection_end = record.selection_end,
			};
			auto const carries_text = InputKindCarriesText(record.input.kind);
			auto const step = DispatchNormalizedInput(record.input, carries_text ? &payload : nullptr);
			consumed = consumed || step.consumed;
			invalidated = invalidated || step.invalidate;
		}

		// The translator's flag follows whatever the records actually did, so the next message is
		// translated against the state machine's real composition state.
		m_impl->m_win32_translator.composition_active = m_impl->m_input.m_composing_id != 0 ? 1 : 0;

		// A click elsewhere, Tab, or focus loss abandons the composition on this side. The OS IME
		// still believes it is converting, so it is told to discard its composition string too;
		// otherwise the next keystroke would resume a conversion nothing is displaying.
		if (was_composing && m_impl->m_input.m_composing_id == 0 && m_impl->m_win32_translator.ime_composition_open != 0 && translated.place_ime_windows == 0)
			Win32CancelImeComposition(hwnd);

		// Real OS capture is only ever touched here, once the state machine's own capture target
		// has actually changed, never by translation or deterministic injection (section 7.2).
		Win32ApplyCaptureTransition(hwnd, previous_captured_id, m_impl->m_input.m_captured_id);

		// Likewise the IME's own windows: the translator asks for placement, and the real IMM call
		// happens here, against the caret the state machine has just settled on.
		if (translated.place_ime_windows != 0 && m_impl->m_input.m_focus_id != 0)
		{
			Rect caret_dip{};
			std::int32_t caret_valid = 0;
			CaretGeometry(m_impl->m_input.m_focus_id, caret_dip, caret_valid);
			if (caret_valid != 0)
				Win32PlaceImeWindows(hwnd, m_impl->m_viewport, caret_dip);
		}

		if (translated.handled != 0)
		{
			result = translated.result;
			invalidate = invalidated ? 1 : 0;
			return 1;
		}

		invalidate = invalidated ? 1 : 0;
		return consumed ? 1 : 0;
	}

	void UiEngine::InputInject(NormalizedInput const& input)
	{
		ValidateHeader<NormalizedInput>(input.header, "NormalizedInput");

		// A text-carrying kind has no meaning without its text, and silently treating it as empty
		// would turn a caller mistake into a plausible-looking edit.
		if (InputKindCarriesText(input.kind))
			throw EngineException(EStatus::InvalidArgument, "NormalizedInput: this input kind requires InputInjectText");

		DispatchNormalizedInput(input, nullptr);
	}

	void UiEngine::InputInjectText(NormalizedInput const& input, InputTextPayload const& payload)
	{
		ValidateHeader<NormalizedInput>(input.header, "NormalizedInput");
		ValidateHeader<InputTextPayload>(payload.header, "InputTextPayload");

		if (!InputKindCarriesText(input.kind))
			throw EngineException(EStatus::InvalidArgument, "InputTextPayload: this input kind carries no text");

		auto const record = BorrowTextPayload(payload);
		DispatchNormalizedInput(input, &record);
	}

	InputResult UiEngine::DispatchNormalizedInput(NormalizedInput const& input, InputTextRecord const* text_payload)
	{
		try
		{
			auto const hit_context = TextHitContext{ .shaper = TextShaperOrNull(), .placements = &m_impl->m_placements };
			return ProcessNormalizedInput(m_impl->m_tree, m_impl->m_layout, input, text_payload, hit_context, m_impl->m_input, m_impl->m_events, m_impl->m_tree.m_revision);
		}
		catch (EngineException const& ex)
		{
			if (ex.Status() == EStatus::QueueOverflow)
				m_impl->m_event_overflow_count++;

			m_impl->m_last_failure_status = ex.Status();
			throw;
		}
	}

	InputResult UiEngine::ApplySemanticAction(SemanticActionRequest const& request)
	{
		// Failure bookkeeping is applied exactly as it is for translated and injected input, so a
		// rejected accessibility action is visible through the same Diagnostics surface.
		try
		{
			return pr::view3d::ui::ApplySemanticAction(m_impl->m_tree, request, m_impl->m_input, m_impl->m_events, m_impl->m_tree.m_revision);
		}
		catch (EngineException const& ex)
		{
			if (ex.Status() == EStatus::QueueOverflow)
				m_impl->m_event_overflow_count++;

			m_impl->m_last_failure_status = ex.Status();
			throw;
		}
	}

	UiaBridge& UiEngine::Uia()
	{
		return m_impl->m_uia;
	}

	TextShaper* UiEngine::TextShaperOrNull()
	{
		// Text measurement is only ever needed once a control actually holds text, so the
		// DirectWrite factory is created on first demand. A machine without DirectWrite degrades
		// to end-of-text caret placement rather than failing every pointer press.
		if (m_impl->m_text_shaper != nullptr)
			return m_impl->m_text_shaper.get();

		try
		{
			m_impl->m_text_shaper = std::make_unique<TextShaper>();
		}
		catch (EngineException const& ex)
		{
			if (ex.Status() != EStatus::InternalError)
				throw;

			m_impl->m_last_failure_status = ex.Status();
			return nullptr;
		}

		return m_impl->m_text_shaper.get();
	}

	void UiEngine::CaretGeometry(ControlId control_id, Rect& out_caret_dip, std::int32_t& out_valid)
	{
		out_caret_dip = Rect{ 0.0f, 0.0f, 0.0f, 0.0f };
		out_valid = 0;

		auto const node_it = m_impl->m_tree.m_controls.find(control_id);
		if (node_it == m_impl->m_tree.m_controls.end() || node_it->second.desc.type != EControlType::TextBox)
			return;

		// A caret only exists where text can be entered, and an IMM candidate window must never be
		// anchored to a control the user is not typing into.
		auto const& node = node_it->second;
		if (node.desc.enabled == 0 || m_impl->m_input.m_focus_id != control_id)
			return;

		auto const layout_it = m_impl->m_layout.find(control_id);
		if (layout_it == m_impl->m_layout.end())
			return;

		auto* const shaper = TextShaperOrNull();
		if (shaper == nullptr)
			return;

		// Measured from exactly the string the renderer draws - the pending edit with any active
		// composition spliced in - so the candidate window tracks the composition as it grows. A
		// focused control that has not been edited yet still has a caret, at the start of the
		// application's own text.
		auto const edit_it = m_impl->m_input.m_text_edits.find(control_id);
		auto const has_edit = edit_it != m_impl->m_input.m_text_edits.end() && edit_it->second.initialized != 0;
		auto const display = has_edit ? DisplayTextOf(edit_it->second) : node.text;
		auto const caret_offset = has_edit ? DisplayRangesOf(edit_it->second).caret : 0u;
		auto const scale = ControlScale(m_impl->m_tree, &m_impl->m_placements, control_id);
		auto const font = ResolveControlFont(m_impl->m_tree, node.desc.font_resource_id);
		auto const placement = TextPlacementFor(node.desc.type);

		// The caret rectangle must be the one the renderer paints, so it is measured from the same
		// shaped layout and placed with the same vertical formula rather than an approximation.
		// CaretAt already answers the empty-string case from the resolved font's metrics.
		auto const measured = shaper->CaretAt(font.family, font.size * scale, display, caret_offset);

		float ascent_dip = 0.0f, descent_dip = 0.0f;
		shaper->Metrics(font.family, font.size * scale, ascent_dip, descent_dip);

		out_caret_dip = Rect{
			layout_it->second.x + placement.inset_dip * scale + measured.x,
			TextOriginYDip(layout_it->second.y, layout_it->second.h, ascent_dip, descent_dip) + measured.y,
			1.0f,
			measured.height,
		};
		out_valid = 1;
	}

	std::uint32_t UiEngine::EventCount() const
	{
		return m_impl->m_events.Count();
	}

	std::uint32_t UiEngine::EventPayloadBytesPending() const
	{
		return m_impl->m_events.PayloadBytesPending();
	}

	void UiEngine::EventsCopy(std::span<Event> events, std::span<std::byte> payload_blob)
	{
		m_impl->m_events.Copy(events, payload_blob);
	}

	std::uint32_t UiEngine::SemanticCount() const
	{
		return static_cast<std::uint32_t>(m_impl->m_semantics.m_nodes.size());
	}

	std::uint32_t UiEngine::SemanticTextBytesPending() const
	{
		return static_cast<std::uint32_t>(m_impl->m_semantics.m_text_blob.size());
	}

	void UiEngine::SemanticsCopy(std::span<SemanticNode> nodes, std::span<char> text_blob)
	{
		auto const& snapshot = m_impl->m_semantics;
		if (nodes.size() < snapshot.m_nodes.size())
			throw EngineException(EStatus::BufferTooSmall, std::format("SemanticsCopy: caller buffer holds {} nodes but {} are pending", nodes.size(), snapshot.m_nodes.size()));
		if (text_blob.size() < snapshot.m_text_blob.size())
			throw EngineException(EStatus::BufferTooSmall, std::format("SemanticsCopy: caller text buffer holds {} bytes but {} are pending", text_blob.size(), snapshot.m_text_blob.size()));

		// Semantics is a persistent read-only snapshot of the most recent Update() call (unlike
		// the event queue), so repeated queries against unchanged state return identical data.
		std::copy(snapshot.m_nodes.begin(), snapshot.m_nodes.end(), nodes.begin());
		if (!snapshot.m_text_blob.empty())
			std::memcpy(text_blob.data(), snapshot.m_text_blob.data(), snapshot.m_text_blob.size());
	}

	Diagnostics UiEngine::DiagnosticsGet() const
	{
		return Diagnostics{
			StructHeader{ sizeof(Diagnostics), VIEW3D_UI_STRUCT_VERSION },
			m_impl->m_tree.m_revision,
			m_impl->m_rejected_revision_attempts,
			static_cast<std::uint32_t>(m_impl->m_tree.m_controls.size()),
			m_impl->m_events.Count(),
			m_impl->m_event_overflow_count,
			static_cast<std::uint32_t>(m_impl->m_semantics.m_nodes.size()),
			m_impl->m_last_failure_status,
			0,
		};
	}

	DrawPacket const& UiEngine::DrawPackets() const
	{
		return m_impl->m_draw_packet;
	}

	void UiEngine::NotifyRendererStatus(EStatus status)
	{
		m_impl->m_last_failure_status = status;
	}
}
