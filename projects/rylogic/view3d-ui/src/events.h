//*********************************************
// View3DUI
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Bounded, contiguous, coalescing event queue (implementation-plan.md section 5.4). One capacity
// slot is always reserved for a coalescible QueueOverflow marker event, so the application is
// always able to observe that an overflow happened even while every content slot is occupied.
#pragma once
#include "pr/view3d-ui/forward.h"
#include "pr/view3d-ui/types.h"

namespace pr::view3d::ui
{
	// One buffered event awaiting delivery to the application via View3DUI_EventsCopy.
	struct QueuedEvent
	{
		ControlId control_id;
		EEventKind kind;
		std::uint64_t accepted_revision;
		std::uint64_t sequence;
		std::uint32_t edit_generation;
		std::string payload; // UTF-8 bytes; empty for kinds with no payload (e.g. CommandInvoked)
	};

	// Returns true for kinds where only the most recent value matters (section 5.4): repeated
	// pushes for the same coalescing key replace the queued instance rather than growing the queue.
	// CommandInvoked is never coalesced; every activation is preserved and delivered.
	bool IsCoalescible(EEventKind kind);

	class EventQueue
	{
		std::uint32_t m_capacity;    // includes the one slot reserved for the overflow marker
		std::uint64_t m_next_sequence = 1;
		std::vector<QueuedEvent> m_items;
		std::int32_t m_overflow_marker_present = 0;
		QueuedEvent m_overflow_marker{};

	public:
		explicit EventQueue(std::uint32_t capacity);

		// Enqueue one event, coalescing it with any existing same-key entry. Returns true if the
		// event (or its coalesced replacement) is now queued; returns false only when a
		// non-coalescible event could not be queued and the queue is now in the overflow state
		// (section 5.4: CommandInvoked is never silently dropped - the caller must surface this as
		// EStatus::QueueOverflow and must not commit the activation that produced it).
		bool Push(ControlId control_id, EEventKind kind, std::uint64_t accepted_revision, std::uint32_t edit_generation, std::string payload);

		std::uint32_t Count() const;
		std::uint32_t PayloadBytesPending() const;
		bool OverflowActive() const;

		// Copy every currently queued event (including any pending overflow marker) into 'out'/
		// 'payload_out', then clear the queue. The caller must ensure both buffers are already
		// sized to Count()/PayloadBytesPending(); this call performs no partial/truncated drain.
		void Copy(std::span<Event> out, std::span<std::byte> payload_out);
	};
}
