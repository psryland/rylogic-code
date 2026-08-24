//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/physics/articulation/articulation_desc.h"

namespace pr::physics
{
	namespace detail
	{
		struct ArticulationState;
		struct ArticulationBuilderState;
	}

	class ArticulationBuilder;

	// Owns one reduced-coordinate tree, its generalized state, and derived link kinematics.
	class Articulation
	{
		std::unique_ptr<detail::ArticulationState> m_state;

		friend class ArticulationBuilder;

		// Construct a validated articulation from a consumed builder state.
		explicit Articulation(std::unique_ptr<detail::ArticulationState> state);

	public:

		// Destroy the owned articulation state.
		~Articulation();

		// Move an articulation without changing its stable identity or link handles.
		Articulation(Articulation&& rhs) noexcept;

		// Move an articulation without changing its stable identity or link handles.
		Articulation& operator=(Articulation&& rhs) noexcept;

		Articulation(Articulation const&) = delete;
		Articulation& operator=(Articulation const&) = delete;

		// Return the stable identity used by future link constraint endpoints.
		ArticulationId Id() const;

		// Return whether the root is fixed or contributes a six-velocity floating base.
		EArticulationRootType RootType() const;

		// Return the root link handle.
		LinkHandle Root() const;

		// Return the number of physical links in the tree.
		int LinkCount() const;

		// Return the generalized velocity dimension, including six root velocities for a floating base.
		int DofCount() const;

		// Return the number of reduced coordinates owned by a non-root link joint.
		int JointDofCount(LinkHandle link) const;

		// Return the parent link, or an invalid handle for the root.
		LinkHandle Parent(LinkHandle link) const;

		// Return the current world transform of a link after lazily refreshing kinematics.
		m4x4 const& LinkToWorld(LinkHandle link) const;

		// Return the current link-frame spatial velocity after lazily refreshing kinematics.
		v8motion LinkVelocity(LinkHandle link) const;

		// Return the root world transform.
		m4x4 const& RootToWorld() const;

		// Replace the root world transform and invalidate derived link kinematics.
		void RootToWorld(m4x4 const& root_to_world);

		// Return the floating root's link-frame spatial velocity, or zero for a fixed root.
		v8motion RootVelocity() const;

		// Replace the floating root's link-frame spatial velocity.
		void RootVelocity(v8motion velocity);

		// Return the current reduced positions for one non-root joint.
		std::span<float const> JointPosition(LinkHandle link) const;

		// Replace all reduced positions for one non-root joint.
		void JointPosition(LinkHandle link, std::span<float const> position);

		// Return the current reduced velocities for one non-root joint.
		std::span<float const> JointVelocity(LinkHandle link) const;

		// Replace all reduced velocities for one non-root joint.
		void JointVelocity(LinkHandle link, std::span<float const> velocity);

		// Return the current applied generalized forces for one non-root joint.
		std::span<float const> JointForce(LinkHandle link) const;

		// Replace all applied generalized forces for one non-root joint.
		void JointForce(LinkHandle link, std::span<float const> force);

		// Refresh all world transforms, motion subspaces, bias accelerations, and link velocities in parent-before-child order.
		void UpdateKinematics();
	};

	// Builds a topologically sorted articulation while rejecting stale or cross-builder link handles.
	class ArticulationBuilder
	{
		std::unique_ptr<detail::ArticulationBuilderState> m_state;

	public:

		// Begin an empty articulation topology.
		ArticulationBuilder();

		// Destroy an unconsumed builder state.
		~ArticulationBuilder();

		// Move a builder without changing handles already returned from it.
		ArticulationBuilder(ArticulationBuilder&& rhs) noexcept;

		// Move a builder without changing handles already returned from it.
		ArticulationBuilder& operator=(ArticulationBuilder&& rhs) noexcept;

		ArticulationBuilder(ArticulationBuilder const&) = delete;
		ArticulationBuilder& operator=(ArticulationBuilder const&) = delete;

		// Add the single world-anchored root link.
		LinkHandle AddFixedRoot(ArticulationLinkDesc const& link, m4x4 const& link_to_world = m4x4::Identity());

		// Add the single six-velocity floating root link.
		LinkHandle AddFloatingRoot(ArticulationLinkDesc const& link, m4x4 const& link_to_world = m4x4::Identity(), v8motion velocity = {});

		// Add a physical child link connected to an existing parent by a zero-to-six-DOF reduced joint.
		LinkHandle AddLink(LinkHandle parent, ArticulationJointDesc const& joint, ArticulationLinkDesc const& link);

		// Validate and consume the topology into a movable articulation.
		Articulation Build();
	};
}
