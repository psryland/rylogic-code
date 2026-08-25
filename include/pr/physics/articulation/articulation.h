//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/physics/articulation/articulation_desc.h"

namespace pr::physics
{
	class Articulation;

	namespace detail
	{
		using SpatialMobility = Mat6x8<float, Force, Motion>;

		struct ArticulationState;
		struct ArticulationBuilderState;
		struct ArticulationIntegrationOutput;

		// Validate a detached GPU integration result without changing its articulation.
		void ValidateArticulationIntegrationOutput(Articulation const& articulation, ArticulationIntegrationOutput const& output);

		// Commit one prevalidated GPU integration result and reconstruct its midpoint link accelerations.
		void CommitArticulationIntegrationOutput(Articulation& articulation, ArticulationIntegrationOutput const& output);

		// Compute every exact self-link impulse mobility with one linear-time tree factorization and recurrence.
		void ComputeArticulationLinkMobilities(Articulation const& articulation, std::span<SpatialMobility> mobilities);

		// Evaluate one batched impulse ABA without committing its generalized or link-velocity response.
		void ComputeArticulationImpulseResponse(Articulation& articulation, std::span<ArticulationImpulse const> impulses, std::span<float> generalized_delta, std::span<v8motion> link_velocity_delta);

		// Integrate a detached generalized pseudo velocity into articulation coordinates without changing physical momentum.
		void ApplyArticulationPositionCorrection(Articulation& articulation, std::span<float const> generalized_velocity, float timestep);
	}

	class ArticulationBuilder;

	// Owns one reduced-coordinate tree, its generalized state, and derived link kinematics.
	class Articulation
	{
		std::unique_ptr<detail::ArticulationState> m_state;

		friend class ArticulationBuilder;
		friend struct Engine;
		friend void detail::ValidateArticulationIntegrationOutput(Articulation const& articulation, detail::ArticulationIntegrationOutput const& output);
		friend void detail::CommitArticulationIntegrationOutput(Articulation& articulation, detail::ArticulationIntegrationOutput const& output);
		friend void detail::ComputeArticulationLinkMobilities(Articulation const& articulation, std::span<detail::SpatialMobility> mobilities);
		friend void detail::ComputeArticulationImpulseResponse(Articulation& articulation, std::span<ArticulationImpulse const> impulses, std::span<float> generalized_delta, std::span<v8motion> link_velocity_delta);
		friend void detail::ApplyArticulationPositionCorrection(Articulation& articulation, std::span<float const> generalized_velocity, float timestep);

		// Construct a validated articulation from a consumed builder state.
		explicit Articulation(std::unique_ptr<detail::ArticulationState> state);

		// Advance the complete-tree sleep timer after one accepted frame and sleep only when every link remains below threshold.
		void UpdateSleeping(float elapsed_seconds, float linear_velocity_threshold, float angular_velocity_threshold, float sleep_delay_s);

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

		// Return a link handle by stable topological order.
		LinkHandle LinkAt(int link_index) const;

		// Return immutable mass and collision-proxy data for a link.
		ArticulationLinkDesc const& LinkDescription(LinkHandle link) const;

		// Return immutable reduced-joint topology for a non-root link.
		ArticulationJointDesc const& JointDescription(LinkHandle link) const;

		// Return the current world transform of a link after lazily refreshing kinematics.
		m4x4 const& LinkToWorld(LinkHandle link) const;

		// Return the current link-frame spatial velocity after lazily refreshing kinematics.
		v8motion LinkVelocity(LinkHandle link) const;

		// Return the most recently solved link-frame spatial acceleration.
		v8motion LinkAcceleration(LinkHandle link) const;

		// Return the articulation's total world-space momentum about the world origin.
		v8force MomentumWS() const;

		// Return the sum of the physical links' kinetic energies.
		float KineticEnergy() const;

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

		// Return the floating root's applied generalized wrench, or zero for a fixed root.
		v8force RootForce() const;

		// Replace the floating root's applied generalized wrench.
		void RootForce(v8force force);

		// Return the external link-frame wrench applied at a link origin.
		v8force ExternalForce(LinkHandle link) const;

		// Replace the external link-frame wrench applied at a link origin.
		void ExternalForce(LinkHandle link, v8force force);

		// Accumulate an external link-frame wrench at a link origin.
		void ApplyExternalForce(LinkHandle link, v8force force);

		// Return the world-space gravity field sampled by one link.
		v4 GravityWS(LinkHandle link) const;

		// Set the world-space gravity field evaluated at one link during each dynamics solve.
		void GravityWS(LinkHandle link, v4 gravity);

		// Return whether the complete articulation tree is asleep.
		bool Sleeping() const;

		// Put the complete articulation tree to sleep or wake it immediately.
		void Sleeping(bool sleeping);

		// Put the complete articulation tree to sleep and discard all generalized motion and transient loads.
		void Sleep();

		// Wake the complete articulation tree and restart its inactivity timer.
		void Wake();

		// Return whether the complete articulation tree is immune to automatic sleeping.
		bool NeverSleep() const;

		// Enable or disable automatic sleeping for the complete tree; enabling immunity also wakes it.
		void NeverSleep(bool never_sleep);

		// Clear every applied generalized force and external link wrench.
		void ClearForces();

		// Return the most recently solved floating-root spatial acceleration, or zero for a fixed root.
		v8motion RootAcceleration() const;

		// Return the most recently solved reduced accelerations for one non-root joint.
		std::span<float const> JointAcceleration(LinkHandle link) const;

		// Refresh all world transforms, motion subspaces, bias accelerations, and link velocities in parent-before-child order.
		void UpdateKinematics();

		// Solve unconstrained generalized and link accelerations with Featherstone's force ABA.
		void ForwardDynamics();

		// Advance unconstrained state with a bounded implicit-midpoint solve and consume the applied forces.
		void Integrate(float elapsed_seconds);

		// Apply one link-frame impulse through the complete tree response.
		void ApplyImpulse(LinkHandle link, v8force impulse);

		// Accumulate link-frame impulses and apply them through one complete tree response.
		void ApplyImpulses(std::span<ArticulationImpulse const> impulses);
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
