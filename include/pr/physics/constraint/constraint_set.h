//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/physics/constraint/constraint_desc.h"

namespace pr::physics
{
	// A half-open range of persistent descriptor slots requiring a GPU upload.
	struct ConstraintDirtyRange
	{
		uint32_t m_begin = 0;
		uint32_t m_end = 0;
	};

	// Owns persistent constraint descriptors while referring to caller-owned bodies only through stable identities.
	class ConstraintSet
	{
	private:

		// Storage for one reusable generational handle slot.
		struct Slot
		{
			D6ConstraintDesc m_desc = {};
			uint32_t m_generation = 1;
			bool m_occupied = false;
			bool m_dirty = false;
		};

		std::vector<Slot> m_slots;
		std::vector<uint32_t> m_free_slots;
		size_t m_count;
		uint64_t m_topology_revision;
		uint64_t m_parameter_revision;

		// Return a live slot or reject an invalid or stale handle.
		Slot& Require(ConstraintHandle handle);

		// Return a live slot or reject an invalid or stale handle.
		Slot const& Require(ConstraintHandle handle) const;

		// Validate a descriptor before it can enter persistent storage.
		static void Validate(D6ConstraintDesc const& desc);

		friend CompiledConstraintSet CompileConstraints(ConstraintSet const& constraints, BodyRemap const& remap);
		friend GpuConstraintUpload PackGpuConstraints(ConstraintSet const& constraints, BodyRemap const& remap);
		friend bool HasCoupledConstraintWork(ConstraintSet const& constraints);
		friend void WakeCoupledConstraintArticulations(ConstraintSet const& constraints, std::span<Articulation*> articulations);

	public:

		// Construct an empty constraint collection without allocating descriptor storage.
		ConstraintSet();

		// Add a general D6 constraint and return its generational handle.
		ConstraintHandle Add(D6ConstraintDesc const& desc);

		// Add a ball-and-socket constraint.
		ConstraintHandle Add(BallSocketConstraintDesc const& desc);

		// Add a hinge constraint.
		ConstraintHandle Add(HingeConstraintDesc const& desc);

		// Add a slider constraint.
		ConstraintHandle Add(SliderConstraintDesc const& desc);

		// Add a weld constraint.
		ConstraintHandle Add(WeldConstraintDesc const& desc);

		// Replace a persistent descriptor while preserving its handle.
		void Update(ConstraintHandle handle, D6ConstraintDesc const& desc);

		// Enable or disable a persistent descriptor without changing topology.
		void SetEnabled(ConstraintHandle handle, bool enabled);

		// Remove a persistent descriptor and invalidate every handle to its generation.
		void Remove(ConstraintHandle handle);

		// Return a persistent descriptor or reject an invalid or stale handle.
		D6ConstraintDesc const& Get(ConstraintHandle handle) const;

		// True when a handle names a currently occupied slot with the same generation.
		bool Contains(ConstraintHandle handle) const;

		// Return the number of live constraints.
		size_t Count() const;

		// Return the number of allocated descriptor slots, including reusable holes.
		size_t CapacitySlots() const;

		// Return merged half-open ranges containing every dirty slot, including removed tombstones.
		std::vector<ConstraintDirtyRange> DirtyRanges() const;

		// Mark every descriptor slot clean after its current state has been uploaded.
		void ClearDirty();

		// Return the revision changed by endpoint additions, removals, or replacements.
		uint64_t TopologyRevision() const;

		// Return the revision changed by any descriptor parameter update.
		uint64_t ParameterRevision() const;
	};
}
