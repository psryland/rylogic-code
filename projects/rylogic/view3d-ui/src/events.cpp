//*********************************************
// View3DUI
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "events.h"
#include "pr/view3d-ui/engine.h"

namespace pr::view3d::ui
{
	bool IsCoalescible(EEventKind kind)
	{
		switch (kind)
		{
			case EEventKind::FocusChanged:
			case EEventKind::TextChangeProposed:
			case EEventKind::PointerCaptureChanged:
			case EEventKind::Diagnostic:
			case EEventKind::QueueOverflow:
			{
				return true;
			}
			case EEventKind::CommandInvoked:
			{
				return false;
			}
			case EEventKind::Count:
			default:
			{
				throw EngineException(EStatus::UnknownType, std::format("IsCoalescible: unknown EEventKind {}", static_cast<int>(kind)));
			}
		}
	}

	namespace
	{
		// FocusChanged/PointerCaptureChanged/Diagnostic are context-wide singletons (coalescing
		// key is the kind alone); TextChangeProposed coalesces per control_id, since independent
		// text boxes each have their own outstanding proposal.
		bool IsSingletonCoalesced(EEventKind kind)
		{
			return kind == EEventKind::FocusChanged || kind == EEventKind::PointerCaptureChanged || kind == EEventKind::Diagnostic;
		}
	}

	EventQueue::EventQueue(std::uint32_t capacity)
		: m_capacity(capacity)
	{
		if (capacity < 2)
			throw EngineException(EStatus::InvalidArgument, std::format("EventQueue: capacity {} must be at least 2 (one content slot plus the reserved overflow marker slot)", capacity));
	}

	bool EventQueue::Push(ControlId control_id, EEventKind kind, std::uint64_t accepted_revision, std::uint32_t edit_generation, std::string payload)
	{
		auto content_capacity = m_capacity - 1;
		auto coalescible = IsCoalescible(kind);

		if (coalescible)
		{
			auto match = std::find_if(m_items.begin(), m_items.end(), [&](QueuedEvent const& e)
			{
				return e.kind == kind && (IsSingletonCoalesced(kind) || e.control_id == control_id);
			});
			if (match != m_items.end())
				m_items.erase(match); // evaporate the superseded entry; a fresh one is pushed below
		}

		if (m_items.size() >= content_capacity)
		{
			// Try to make room by evicting the oldest coalescible entry, regardless of its kind.
			auto victim = std::find_if(m_items.begin(), m_items.end(), [](QueuedEvent const& e) { return IsCoalescible(e.kind); });
			if (victim != m_items.end())
			{
				// The same-key replacement above already removed its superseded entry, so reaching
				// this branch means a distinct event is being discarded to make room. Preserve the
				// new latest-value event, but also publish the reserved overflow marker so the
				// application knows another control/key needs reconciliation.
				m_items.erase(victim);
				m_overflow_marker_present = 1;
				m_overflow_marker = QueuedEvent{ 0, EEventKind::QueueOverflow, accepted_revision, m_next_sequence++, 0, {} };
			}
			else
			{
				// No coalescible entry to evict: raise (or refresh) the overflow marker and reject.
				m_overflow_marker_present = 1;
				m_overflow_marker = QueuedEvent{ 0, EEventKind::QueueOverflow, accepted_revision, m_next_sequence++, 0, {} };
				return false;
			}
		}

		m_items.push_back(QueuedEvent{ control_id, kind, accepted_revision, m_next_sequence++, edit_generation, std::move(payload) });
		return true;
	}

	std::uint32_t EventQueue::Count() const
	{
		return static_cast<std::uint32_t>(m_items.size()) + (m_overflow_marker_present != 0 ? 1U : 0U);
	}

	std::uint32_t EventQueue::PayloadBytesPending() const
	{
		std::uint32_t total = 0;
		for (auto const& item : m_items)
			total += static_cast<std::uint32_t>(item.payload.size());

		return total;
	}

	bool EventQueue::OverflowActive() const
	{
		return m_overflow_marker_present != 0;
	}

	void EventQueue::Copy(std::span<Event> out, std::span<std::byte> payload_out)
	{
		auto const total = Count();
		if (out.size() < total)
			throw EngineException(EStatus::BufferTooSmall, std::format("EventQueue::Copy: caller buffer holds {} events but {} are pending", out.size(), total));

		auto write_index = std::size_t{};
		auto payload_cursor = std::uint32_t{};
		auto write_one = [&](QueuedEvent const& item)
		{
			if (payload_cursor + item.payload.size() > payload_out.size())
				throw EngineException(EStatus::BufferTooSmall, std::format("EventQueue::Copy: caller payload buffer of {} bytes is too small", payload_out.size()));

			if (!item.payload.empty())
				std::memcpy(payload_out.data() + payload_cursor, item.payload.data(), item.payload.size());

			out[write_index++] = Event{
				StructHeader{ sizeof(Event), VIEW3D_UI_STRUCT_VERSION },
				item.control_id,
				item.kind,
				item.accepted_revision,
				item.sequence,
				payload_cursor,
				static_cast<std::uint32_t>(item.payload.size()),
				item.edit_generation,
				0,
			};
			payload_cursor += static_cast<std::uint32_t>(item.payload.size());
		};

		if (m_overflow_marker_present != 0)
			write_one(m_overflow_marker);
		for (auto const& item : m_items)
			write_one(item);

		m_items.clear();
		m_overflow_marker_present = 0;
	}
}
