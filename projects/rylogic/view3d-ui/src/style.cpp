//*********************************************
// View3DUI
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "style.h"
#include "pr/view3d-ui/engine.h"

namespace pr::view3d::ui
{
	namespace
	{
		// Linear interpolation of one scalar/colour field between the previous and target visuals.
		float Lerp(float a, float b, float t)
		{
			return a + (b - a) * t;
		}
		Colour Lerp(Colour const& a, Colour const& b, float t)
		{
			return Colour{ Lerp(a.r, b.r, t), Lerp(a.g, b.g, t), Lerp(a.b, b.b, t), Lerp(a.a, b.a, t) };
		}

		// Smoothstep easing; Linear passes 't' through unchanged.
		float ApplyEasing(EEasing easing, float t)
		{
			switch (easing)
			{
				case EEasing::Linear: return t;
				case EEasing::EaseInOut: return t * t * (3.0f - 2.0f * t);
				case EEasing::Count:
				default:
				{
					throw EngineException(EStatus::UnknownType, std::format("ApplyEasing: unknown EEasing {}", static_cast<int>(easing)));
				}
			}
		}
	}

	EStateChannel ResolveActiveChannel(ControlNode const& node, ControlId hover_id, ControlId pressed_id, ControlId focus_id)
	{
		if (node.desc.enabled == 0)
			return EStateChannel::Disabled;
		if (pressed_id == node.desc.id)
			return EStateChannel::Pressed;
		if (node.desc.validation_state == EValidationState::Invalid)
			return EStateChannel::Invalid;
		if (focus_id == node.desc.id)
			return EStateChannel::Focused;
		if (hover_id == node.desc.id)
			return EStateChannel::Hover;
		if (node.desc.selected != 0)
			return EStateChannel::Selected;

		return EStateChannel::Normal;
	}

	StyleVisual StyleResolver::Resolve(ControlNode const& node, StyleRecord const& style, ControlId hover_id, ControlId pressed_id, ControlId focus_id, double time_ms)
	{
		auto& runtime = m_runtime[node.desc.id]; // bounded by the caller's live-control count; pruned each Update()

		// Visibility/ValueChanged both fire for exactly one Update() call, on top of whichever base
		// channel ResolveActiveChannel reports, so compute them before updating the history fields
		// they depend on.
		auto const visibility_entering = runtime.was_visible == 0;
		auto const value_changed = runtime.seen_before != 0 && node.desc.value_sequence != runtime.last_value_sequence;
		runtime.was_visible = 1; // Resolve is only ever called for a currently-visible control (see MarkInvisible)
		runtime.last_value_sequence = node.desc.value_sequence;

		// Fold the momentary Visibility/ValueChanged transitions into the base channel: a durable
		// interaction/validation state (Disabled/Pressed/Invalid/Focused) is never masked by them,
		// but the remaining, more passive channels (Hover/Selected/Normal) are.
		auto const base_channel = ResolveActiveChannel(node, hover_id, pressed_id, focus_id);
		EStateChannel active_channel;
		switch (base_channel)
		{
			case EStateChannel::Disabled:
			case EStateChannel::Pressed:
			case EStateChannel::Invalid:
			case EStateChannel::Focused:
			{
				active_channel = base_channel;
				break;
			}
			case EStateChannel::Hover:
			case EStateChannel::Selected:
			case EStateChannel::Normal:
			{
				active_channel = visibility_entering ? EStateChannel::Visibility : value_changed ? EStateChannel::ValueChanged : base_channel;
				break;
			}
			case EStateChannel::Visibility:
			case EStateChannel::ValueChanged:
			case EStateChannel::Count:
			default:
			{
				// ResolveActiveChannel never returns these; reaching here is an invariant violation.
				throw EngineException(EStatus::InternalError, std::format("StyleResolver::Resolve: unexpected base channel {}", static_cast<int>(base_channel)));
			}
		}

		if (runtime.seen_before == 0)
		{
			// First observation of this control: adopt the resolved channel with no interpolation.
			runtime.active_channel = active_channel;
			runtime.previous_channel = active_channel;
			runtime.transition_start_ms = time_ms;
			runtime.seen_before = 1;
		}
		else if (active_channel != runtime.active_channel)
		{
			runtime.previous_channel = runtime.active_channel;
			runtime.active_channel = active_channel;
			runtime.transition_start_ms = time_ms;
		}

		auto const& target = style.desc.visuals[static_cast<std::size_t>(runtime.active_channel)];
		auto const& transition = style.desc.transitions[static_cast<std::size_t>(runtime.active_channel)];
		if (transition.duration_ms <= 0.0f || runtime.previous_channel == runtime.active_channel)
			return target;

		auto elapsed_ms = static_cast<float>(time_ms - runtime.transition_start_ms);
		auto t = std::clamp(elapsed_ms / transition.duration_ms, 0.0f, 1.0f);
		if (t >= 1.0f)
		{
			runtime.previous_channel = runtime.active_channel; // transition finished; stop blending
			return target;
		}

		auto const& from = style.desc.visuals[static_cast<std::size_t>(runtime.previous_channel)];
		auto eased_t = ApplyEasing(transition.easing, t);
		return StyleVisual{
			Lerp(from.fill, target.fill, eased_t),
			Lerp(from.border_colour, target.border_colour, eased_t),
			Lerp(from.border_thickness, target.border_thickness, eased_t),
			Lerp(from.corner_radius, target.corner_radius, eased_t),
			Lerp(from.opacity, target.opacity, eased_t),
		};
	}

	void StyleResolver::MarkInvisible(ControlId id)
	{
		m_runtime[id].was_visible = 0;
	}

	void StyleResolver::Prune(std::unordered_set<ControlId> const& live_ids)
	{
		for (auto it = m_runtime.begin(); it != m_runtime.end();)
		{
			if (live_ids.contains(it->first))
				++it;
			else
				it = m_runtime.erase(it);
		}
	}
}
