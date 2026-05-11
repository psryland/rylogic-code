//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/physics/forward.h"

namespace pr::physics
{
	struct ContactPrioritySettings
	{
		float m_depth_bias = 60.0f;
		float m_priority_decay = 0.85f;
		float m_min_edge_weight = 1e-5f;
		int m_priority_iterations = 16;
	};

	struct ContactPriorityBody
	{
		v4 m_position_ws = v4::Origin();
		v4 m_velocity_ws = v4::Zero();
		float m_inv_mass = 0.0f;
	};

	struct ContactPriorityContact
	{
		int m_body_idx_a = -1;
		int m_body_idx_b = -1;
		v4 m_axis_ws = v4{1, 0, 0, 0};
		v4 m_point_ws = v4::Origin();
		float m_depth = 0.0f;
		float m_collision_time = 0.0f;
	};

	struct ContactPriorityEdge
	{
		int m_contact_idx_from = -1;
		int m_contact_idx_to = -1;
		int m_body_idx = -1;
		float m_weight = 0.0f;
	};

	struct ContactPriorityResult
	{
		std::vector<ContactPriorityEdge> m_edges;
		std::vector<float> m_contact_priority;
		std::vector<float> m_body_priority;
		float m_total_help_weight = 0.0f;
		float m_satisfied_help_weight = 0.0f;
		float m_lost_help_weight = 0.0f;
		float m_score = 1.0f;
	};

	inline v4 ContactPriorityAxis(ContactPriorityContact const& contact)
	{
		return Normalise(contact.m_axis_ws.w0(), v4::Zero());
	}

	inline void ValidateContactPriorityInput(std::span<ContactPriorityBody const> bodies, std::span<ContactPriorityContact const> contacts)
	{
		for (int contact_idx = 0; contact_idx != isize(contacts); ++contact_idx)
		{
			auto const& contact = contacts[contact_idx];
			if (contact.m_body_idx_a < 0 || contact.m_body_idx_a >= isize(bodies))
				throw std::invalid_argument("Contact priority contact has invalid body index A");
			if (contact.m_body_idx_b < 0 || contact.m_body_idx_b >= isize(bodies))
				throw std::invalid_argument("Contact priority contact has invalid body index B");
			if (contact.m_body_idx_a == contact.m_body_idx_b)
				throw std::invalid_argument("Contact priority contact references the same body twice");
		}
	}

	inline float ContactPriorityNormalDemand(ContactPrioritySettings const& settings, std::span<ContactPriorityBody const> bodies, ContactPriorityContact const& contact)
	{
		auto const axis = ContactPriorityAxis(contact);
		auto const& body_a = bodies[contact.m_body_idx_a];
		auto const& body_b = bodies[contact.m_body_idx_b];
		auto const normal_speed = Dot3(body_b.m_velocity_ws - body_a.m_velocity_ws, axis);
		auto const closing_speed = std::max(0.0f, -normal_speed);
		auto const depth_speed = settings.m_depth_bias * std::max(0.0f, contact.m_depth);
		return closing_speed + depth_speed;
	}

	inline int ContactPrioritySharedBody(ContactPriorityContact const& lhs, ContactPriorityContact const& rhs, std::span<ContactPriorityBody const> bodies)
	{
		auto const body_idx = std::array{
			lhs.m_body_idx_a == rhs.m_body_idx_a ? lhs.m_body_idx_a : -1,
			lhs.m_body_idx_a == rhs.m_body_idx_b ? lhs.m_body_idx_a : -1,
			lhs.m_body_idx_b == rhs.m_body_idx_a ? lhs.m_body_idx_b : -1,
			lhs.m_body_idx_b == rhs.m_body_idx_b ? lhs.m_body_idx_b : -1,
		};

		for (auto idx : body_idx)
		{
			if (idx >= 0 && bodies[idx].m_inv_mass > 0.0f)
				return idx;
		}
		return -1;
	}

	inline v4 ContactPriorityDeltaVelocity(ContactPriorityContact const& contact, std::span<ContactPriorityBody const> bodies, int body_idx, float impulse)
	{
		auto const axis = ContactPriorityAxis(contact);
		auto const& body = bodies[body_idx];
		if (body_idx == contact.m_body_idx_a)
			return -body.m_inv_mass * impulse * axis;
		if (body_idx == contact.m_body_idx_b)
			return +body.m_inv_mass * impulse * axis;

		throw std::invalid_argument("Contact priority delta requested for a body not in the contact");
	}

	inline float ContactPriorityInfluence(ContactPriorityContact const& contact, int shared_body_idx, v4 delta_velocity_ws)
	{
		auto const axis = ContactPriorityAxis(contact);
		auto delta_relative_velocity = v4::Zero();
		if (shared_body_idx == contact.m_body_idx_a)
			delta_relative_velocity = -delta_velocity_ws;
		else if (shared_body_idx == contact.m_body_idx_b)
			delta_relative_velocity = +delta_velocity_ws;
		else
			throw std::invalid_argument("Contact priority influence requested for a body not in the contact");

		// Positive values mean the upstream impulse makes this contact more closing/active, which is the dependency needed for same-frame shock propagation.
		return std::max(0.0f, -Dot3(delta_relative_velocity, axis));
	}

	inline std::vector<ContactPriorityEdge> BuildContactPriorityEdges(ContactPrioritySettings const& settings, std::span<ContactPriorityBody const> bodies, std::span<ContactPriorityContact const> contacts)
	{
		auto edges = std::vector<ContactPriorityEdge>{};
		for (int src = 0; src != isize(contacts); ++src)
		{
			for (int dst = 0; dst != isize(contacts); ++dst)
			{
				if (src == dst)
					continue;

				auto const shared_body_idx = ContactPrioritySharedBody(contacts[src], contacts[dst], bodies);
				if (shared_body_idx < 0)
					continue;

				auto const delta_velocity = ContactPriorityDeltaVelocity(contacts[src], bodies, shared_body_idx, 1.0f);
				auto const influence = ContactPriorityInfluence(contacts[dst], shared_body_idx, delta_velocity);
				if (influence <= settings.m_min_edge_weight)
					continue;

				edges.push_back(ContactPriorityEdge{
					.m_contact_idx_from = src,
					.m_contact_idx_to = dst,
					.m_body_idx = shared_body_idx,
					.m_weight = influence,
				});
			}
		}
		return edges;
	}

	inline std::vector<float> ContactPriorities(ContactPrioritySettings const& settings, std::span<ContactPriorityBody const> bodies, std::span<ContactPriorityContact const> contacts, std::span<ContactPriorityEdge const> edges)
	{
		auto priority = std::vector<float>(contacts.size(), 0.0f);
		for (int contact_idx = 0; contact_idx != isize(contacts); ++contact_idx)
			priority[contact_idx] = ContactPriorityNormalDemand(settings, bodies, contacts[contact_idx]);

		for (int iter = 0, iend = std::max(0, settings.m_priority_iterations); iter != iend; ++iter)
		{
			auto changed = false;
			auto next_priority = priority;
			for (auto const& edge : edges)
			{
				auto const coupling = std::min(edge.m_weight, 1.0f);
				auto const propagated = settings.m_priority_decay * coupling * priority[edge.m_contact_idx_from];
				if (propagated > next_priority[edge.m_contact_idx_to])
				{
					next_priority[edge.m_contact_idx_to] = propagated;
					changed = true;
				}
			}

			priority = std::move(next_priority);
			if (!changed)
				break;
		}
		return priority;
	}

	inline std::vector<float> BodyPriorities(std::span<ContactPriorityBody const> bodies, std::span<ContactPriorityContact const> contacts, std::span<float const> contact_priority)
	{
		auto body_priority = std::vector<float>(bodies.size(), 0.0f);
		for (int contact_idx = 0; contact_idx != isize(contacts); ++contact_idx)
		{
			auto const priority = contact_priority[contact_idx];
			auto const& contact = contacts[contact_idx];
			body_priority[contact.m_body_idx_a] = std::max(body_priority[contact.m_body_idx_a], priority);
			body_priority[contact.m_body_idx_b] = std::max(body_priority[contact.m_body_idx_b], priority);
		}
		return body_priority;
	}

	inline ContactPriorityResult EvaluateContactPriority(ContactPrioritySettings const& settings, std::span<ContactPriorityBody const> bodies, std::span<ContactPriorityContact const> contacts, std::span<int const> contact_order = {})
	{
		ValidateContactPriorityInput(bodies, contacts);

		auto result = ContactPriorityResult{};
		result.m_edges = BuildContactPriorityEdges(settings, bodies, contacts);
		result.m_contact_priority = ContactPriorities(settings, bodies, contacts, result.m_edges);
		result.m_body_priority = BodyPriorities(bodies, contacts, result.m_contact_priority);

		auto position = std::vector<int>(contacts.size(), 0);
		if (contact_order.empty())
		{
			for (int contact_idx = 0; contact_idx != isize(contacts); ++contact_idx)
				position[contact_idx] = contact_idx;
		}
		else
		{
			if (contact_order.size() != contacts.size())
				throw std::invalid_argument("Contact priority order length does not match contact count");

			for (int order_idx = 0; order_idx != isize(contact_order); ++order_idx)
			{
				auto const contact_idx = contact_order[order_idx];
				if (contact_idx < 0 || contact_idx >= isize(contacts))
					throw std::invalid_argument("Contact priority order contains an invalid contact index");

				position[contact_idx] = order_idx;
			}
		}

		for (auto const& edge : result.m_edges)
		{
			auto const priority_from = result.m_contact_priority[edge.m_contact_idx_from];
			auto const priority_to = result.m_contact_priority[edge.m_contact_idx_to];
			if (priority_from <= priority_to + settings.m_min_edge_weight)
				continue;

			auto const weight = edge.m_weight * priority_from;
			result.m_total_help_weight += weight;
			if (position[edge.m_contact_idx_from] <= position[edge.m_contact_idx_to])
				result.m_satisfied_help_weight += weight;
			else
				result.m_lost_help_weight += weight;
		}

		result.m_score = result.m_total_help_weight > 0.0f
			? result.m_satisfied_help_weight / result.m_total_help_weight
			: 1.0f;
		return result;
	}

	inline ContactPriorityResult EvaluateContactPriority(std::span<ContactPriorityBody const> bodies, std::span<ContactPriorityContact const> contacts, std::span<int const> contact_order = {})
	{
		return EvaluateContactPriority(ContactPrioritySettings{}, bodies, contacts, contact_order);
	}

	inline std::vector<int> ContactPriorityOrder(ContactPrioritySettings const& settings, std::span<ContactPriorityBody const> bodies, std::span<ContactPriorityContact const> contacts)
	{
		ValidateContactPriorityInput(bodies, contacts);

		auto edges = BuildContactPriorityEdges(settings, bodies, contacts);
		auto priority = ContactPriorities(settings, bodies, contacts, edges);
		auto order = std::vector<int>(contacts.size(), 0);
		std::iota(order.begin(), order.end(), 0);
		std::stable_sort(order.begin(), order.end(), [&](int lhs, int rhs)
		{
			if (priority[lhs] != priority[rhs])
				return priority[lhs] > priority[rhs];
			if (contacts[lhs].m_collision_time != contacts[rhs].m_collision_time)
				return contacts[lhs].m_collision_time < contacts[rhs].m_collision_time;
			return lhs < rhs;
		});
		return order;
	}

	inline std::vector<int> ContactPriorityOrder(std::span<ContactPriorityBody const> bodies, std::span<ContactPriorityContact const> contacts)
	{
		return ContactPriorityOrder(ContactPrioritySettings{}, bodies, contacts);
	}
}
