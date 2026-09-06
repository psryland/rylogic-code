//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "pr/physics/constraint/constraint_set.h"

namespace pr::physics
{
	namespace
	{
		// True when a scalar is non-negative and either finite or positive infinity.
		bool IsNonNegativeLimit(float value)
		{
			return !std::isnan(value) && value >= 0.0f;
		}

		// Validate one endpoint-local constraint frame.
		void ValidateFrame(BodyFrame const& frame)
		{
			switch (frame.m_body.m_type)
			{
				case EConstraintBodyType::World:
				{
					if (frame.m_body.m_body_id || frame.m_body.m_articulation_id || frame.m_body.m_link)
						throw std::invalid_argument("World constraint endpoint cannot carry an object identity");
					break;
				}
				case EConstraintBodyType::Rigid:
				{
					if (!frame.m_body.m_body_id)
						throw std::invalid_argument("Rigid constraint endpoint requires a valid body identity");
					if (frame.m_body.m_articulation_id || frame.m_body.m_link)
						throw std::invalid_argument("Rigid constraint endpoint cannot carry an articulation identity");
					break;
				}
				case EConstraintBodyType::ArticulationLink:
				{
					if (!frame.m_body.m_articulation_id || !frame.m_body.m_link)
						throw std::invalid_argument("Articulation-link constraint endpoint requires valid articulation and link identities");
					if (frame.m_body.m_body_id)
						throw std::invalid_argument("Articulation-link constraint endpoint cannot carry a rigid-body identity");
					break;
				}
				default:
				{
					throw std::invalid_argument("Unknown constraint endpoint type");
				}
			}

			if (!IsFinite(frame.m_constraint_to_body) || !IsAffine(frame.m_constraint_to_body) || !IsOrthonormal(frame.m_constraint_to_body))
				throw std::invalid_argument("Constraint frame must be a finite orthonormal affine transform");
		}

		// Validate one D6 axis descriptor and every value that can enter solver arithmetic.
		void ValidateAxis(ConstraintAxisDesc const& axis)
		{
			switch (axis.m_mode)
			{
				case EConstraintAxisMode::Free:
				{
					break;
				}
				case EConstraintAxisMode::Locked:
				{
					break;
				}
				case EConstraintAxisMode::Limited:
				{
					if (std::isnan(axis.m_limits.m_beg) || std::isnan(axis.m_limits.m_end) || axis.m_limits.m_beg > axis.m_limits.m_end)
						throw std::invalid_argument("Limited constraint axis requires an ordered position interval");
					break;
				}
				case EConstraintAxisMode::Driven:
				{
					break;
				}
				default:
				{
					throw std::invalid_argument("Unknown constraint axis mode");
				}
			}

			if (!IsFinite(axis.m_target_position) ||
				!IsFinite(axis.m_target_velocity) ||
				!IsFinite(axis.m_stiffness) || axis.m_stiffness < 0.0f ||
				!IsFinite(axis.m_damping) || axis.m_damping < 0.0f ||
				!IsNonNegativeLimit(axis.m_max_force))
				throw std::invalid_argument("Constraint axis contains an invalid target, stiffness, damping, or force limit");
		}

		// Increment a generation while preserving zero as the invalid generation.
		void AdvanceGeneration(uint32_t& generation)
		{
			++generation;
			if (generation == 0)
				++generation;
		}
	}

	// Construct an empty constraint collection without allocating descriptor storage.
	ConstraintSet::ConstraintSet()
		: m_slots()
		, m_free_slots()
		, m_count()
		, m_topology_revision()
		, m_parameter_revision()
	{
	}

	// Return a live slot or reject an invalid or stale handle.
	ConstraintSet::Slot& ConstraintSet::Require(ConstraintHandle handle)
	{
		if (!Contains(handle))
			throw std::invalid_argument("Constraint handle is invalid, removed, or stale");

		return m_slots[handle.m_index];
	}

	// Return a live slot or reject an invalid or stale handle.
	ConstraintSet::Slot const& ConstraintSet::Require(ConstraintHandle handle) const
	{
		if (!Contains(handle))
			throw std::invalid_argument("Constraint handle is invalid, removed, or stale");

		return m_slots[handle.m_index];
	}

	// Validate a descriptor before it can enter persistent storage.
	void ConstraintSet::Validate(D6ConstraintDesc const& desc)
	{
		ValidateFrame(desc.m_frame_a);
		ValidateFrame(desc.m_frame_b);
		if (desc.m_frame_a.m_body.IsWorld() && desc.m_frame_b.m_body.IsWorld())
			throw std::invalid_argument("A constraint cannot connect world space to itself");
		if (!desc.m_frame_a.m_body.IsWorld() && desc.m_frame_a.m_body == desc.m_frame_b.m_body)
			throw std::invalid_argument("A constraint cannot connect an endpoint to itself");

		for (auto const& axis : desc.m_linear)
			ValidateAxis(axis);
		for (auto const& axis : desc.m_angular)
			ValidateAxis(axis);

		if (!IsNonNegativeLimit(desc.m_break_force) || !IsNonNegativeLimit(desc.m_break_torque))
			throw std::invalid_argument("Constraint break force and torque must be non-negative");
	}

	// Add a general D6 constraint and return its generational handle.
	ConstraintHandle ConstraintSet::Add(D6ConstraintDesc const& desc)
	{
		Validate(desc);

		auto index = uint32_t{};
		if (!m_free_slots.empty())
		{
			index = m_free_slots.back();
			m_free_slots.pop_back();
		}
		else
		{
			if (m_slots.size() >= std::numeric_limits<uint32_t>::max())
				throw std::length_error("Constraint slot index capacity exhausted");

			index = s_cast<uint32_t>(m_slots.size());
			m_slots.push_back(Slot{});
		}

		auto& slot = m_slots[index];
		slot.m_desc = desc;
		slot.m_occupied = true;
		slot.m_dirty = true;
		slot.m_broken = false;
		++m_count;
		++m_topology_revision;
		++m_parameter_revision;
		return ConstraintHandle{
			.m_index = index,
			.m_generation = slot.m_generation,
		};
	}

	// Add a ball-and-socket constraint.
	ConstraintHandle ConstraintSet::Add(BallSocketConstraintDesc const& desc)
	{
		return Add(ToD6(desc));
	}

	// Add a hinge constraint.
	ConstraintHandle ConstraintSet::Add(HingeConstraintDesc const& desc)
	{
		return Add(ToD6(desc));
	}

	// Add a slider constraint.
	ConstraintHandle ConstraintSet::Add(SliderConstraintDesc const& desc)
	{
		return Add(ToD6(desc));
	}

	// Add a weld constraint.
	ConstraintHandle ConstraintSet::Add(WeldConstraintDesc const& desc)
	{
		return Add(ToD6(desc));
	}

	// Replace a persistent descriptor while preserving its handle.
	void ConstraintSet::Update(ConstraintHandle handle, D6ConstraintDesc const& desc)
	{
		Validate(desc);
		auto& slot = Require(handle);
		auto const topology_changed = slot.m_desc.m_frame_a.m_body != desc.m_frame_a.m_body || slot.m_desc.m_frame_b.m_body != desc.m_frame_b.m_body;
		slot.m_desc = desc;
		slot.m_dirty = true;
		m_topology_revision += topology_changed ? 1 : 0;
		++m_parameter_revision;
	}

	// Enable or disable a persistent descriptor without changing topology.
	void ConstraintSet::SetEnabled(ConstraintHandle handle, bool enabled)
	{
		auto& slot = Require(handle);
		if (slot.m_desc.m_enabled == enabled)
			return;

		// Only an actual disabled-to-enabled transition is a repair signal; repeated SetEnabled(true) calls cannot silently reconnect a broken joint.
		auto const re_enabled = !slot.m_desc.m_enabled && enabled;
		slot.m_desc.m_enabled = enabled;
		if (re_enabled)
			slot.m_broken = false;
		slot.m_dirty = true;
		++m_parameter_revision;
	}

	// Explicitly repair a broken constraint while preserving its descriptor and stable handle.
	void ConstraintSet::Repair(ConstraintHandle handle)
	{
		auto& slot = Require(handle);
		if (!slot.m_broken)
			return;

		slot.m_broken = false;
		++m_parameter_revision;
	}

	// Remove a persistent descriptor and invalidate every handle to its generation.
	void ConstraintSet::Remove(ConstraintHandle handle)
	{
		auto& slot = Require(handle);
		slot.m_desc = D6ConstraintDesc{};
		slot.m_desc.m_enabled = false;
		slot.m_occupied = false;
		slot.m_dirty = true;
		slot.m_broken = false;
		AdvanceGeneration(slot.m_generation);
		m_free_slots.push_back(handle.m_index);
		--m_count;
		++m_topology_revision;
		++m_parameter_revision;
	}

	// Return a persistent descriptor or reject an invalid or stale handle.
	D6ConstraintDesc const& ConstraintSet::Get(ConstraintHandle handle) const
	{
		return Require(handle).m_desc;
	}

	// True when a handle names a currently occupied slot with the same generation.
	bool ConstraintSet::Contains(ConstraintHandle handle) const
	{
		return
			handle.m_index < m_slots.size() &&
			handle.m_generation != 0 &&
			m_slots[handle.m_index].m_occupied &&
			m_slots[handle.m_index].m_generation == handle.m_generation;
	}

	// True when overload detection has disabled the live constraint until an explicit repair or disable/enable transition.
	bool ConstraintSet::IsBroken(ConstraintHandle handle) const
	{
		return Require(handle).m_broken;
	}

	// Return the number of live constraints.
	size_t ConstraintSet::Count() const
	{
		return m_count;
	}

	// Return the number of allocated descriptor slots, including reusable holes.
	size_t ConstraintSet::CapacitySlots() const
	{
		return m_slots.size();
	}

	// Return merged half-open ranges containing every dirty slot, including removed tombstones.
	std::vector<ConstraintDirtyRange> ConstraintSet::DirtyRanges() const
	{
		auto ranges = std::vector<ConstraintDirtyRange>{};
		for (uint32_t index = 0; index != m_slots.size();)
		{
			if (!m_slots[index].m_dirty)
			{
				++index;
				continue;
			}

			auto const begin = index;
			while (index != m_slots.size() && m_slots[index].m_dirty)
				++index;
			ranges.push_back(ConstraintDirtyRange{
				.m_begin = begin,
				.m_end = index,
			});
		}
		return ranges;
	}

	// Mark every descriptor slot clean after its current state has been uploaded.
	void ConstraintSet::ClearDirty()
	{
		for (auto& slot : m_slots)
			slot.m_dirty = false;
	}

	// Return the revision changed by endpoint additions, removals, or replacements.
	uint64_t ConstraintSet::TopologyRevision() const
	{
		return m_topology_revision;
	}

	// Return the revision changed by any descriptor parameter update.
	uint64_t ConstraintSet::ParameterRevision() const
	{
		return m_parameter_revision;
	}

	// Latch one validated GPU overload result and return whether it generated a new edge event.
	bool ConstraintSet::MarkBroken(uint32_t slot_index, uint32_t generation) const
	{
		auto const handle = ConstraintHandle{
			.m_index = slot_index,
			.m_generation = generation,
		};
		auto& slot = Require(handle);
		if (slot.m_broken)
			return false;

		slot.m_broken = true;
		++m_parameter_revision;
		return true;
	}
}
