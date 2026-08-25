//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "pr/physics/articulation/articulation.h"
#include "src/articulation/articulation_internal.h"

namespace pr::physics
{
	namespace
	{
		std::atomic<uint64_t> g_next_articulation_id = 1;
		std::atomic<uint32_t> g_next_link_generation = 1;

		// Return a nonzero stable articulation identity.
		ArticulationId NewArticulationId()
		{
			auto value = g_next_articulation_id.fetch_add(1, std::memory_order_relaxed);
			if (value == 0)
				throw std::overflow_error("Articulation identity space exhausted");

			return ArticulationId{value};
		}

		// Return a nonzero generation unique to this builder lifetime.
		uint32_t NewLinkGeneration()
		{
			auto generation = g_next_link_generation.fetch_add(1, std::memory_order_relaxed);
			if (generation == 0)
				throw std::overflow_error("Articulation link generation space exhausted");

			return generation;
		}

		// Reject a moved-from or consumed builder before it can mutate shared state.
		detail::ArticulationBuilderState& CheckedBuilder(std::unique_ptr<detail::ArticulationBuilderState> const& state)
		{
			if (!state || state->m_consumed || !state->m_articulation)
				throw std::logic_error("Articulation builder has already been consumed");

			return *state;
		}

		// Reject a moved-from articulation before accessing its implementation state.
		detail::ArticulationState& CheckedState(std::unique_ptr<detail::ArticulationState> const& state)
		{
			if (!state)
				throw std::logic_error("Articulation has been moved from");

			return *state;
		}

		// Require finite physical inertia for every moving link.
		void ValidateMovingInertia(Inertia const& inertia)
		{
			if (!inertia.Check() || inertia.Mass() <= ZeroMass || inertia.Mass() >= InfiniteMass)
				throw std::invalid_argument("Moving articulation links require finite positive inertia");
		}

		// Validate link-local geometry and inertia before topology state is allocated.
		void ValidateLink(ArticulationLinkDesc const& link, bool moving)
		{
			detail::ValidateArticulationTransform(link.m_shape_to_link, "Articulation shape-to-link transform");
			if (moving)
				ValidateMovingInertia(link.m_inertia);
			else if (!link.m_inertia.Check())
				throw std::invalid_argument("Fixed articulation root inertia is invalid");
		}

		// Validate the bounded ordered-axis joint representation.
		void ValidateJoint(ArticulationJointDesc const& joint)
		{
			if (joint.m_dof_count < 0 || joint.m_dof_count > isize(joint.m_axes))
				throw std::invalid_argument("Articulation joint DOF count must be between zero and six");

			detail::ValidateArticulationTransform(joint.m_joint_to_parent, "Articulation parent joint frame");
			detail::ValidateArticulationTransform(joint.m_joint_to_child, "Articulation child joint frame");

			// Every active scalar screw must use a finite unit direction and finite initial state.
			for (int axis_index = 0; axis_index != joint.m_dof_count; ++axis_index)
			{
				auto const& axis = joint.m_axes[axis_index];
				switch (axis.m_type)
				{
					case EArticulationAxisType::Revolute:
					case EArticulationAxisType::Prismatic:
					{
						break;
					}
					default:
					{
						throw std::invalid_argument("Articulation joint axis type is invalid");
					}
				}

				if (!IsFinite(axis.m_axis) || axis.m_axis.w != 0.0f || Abs(LengthSq(axis.m_axis.xyz) - 1.0f) > 1.0e-5f)
					throw std::invalid_argument("Articulation joint axes must be finite unit vectors");
				if (!IsFinite(joint.m_initial_position[axis_index]) || !IsFinite(joint.m_initial_velocity[axis_index]))
					throw std::invalid_argument("Articulation initial joint state must be finite");
			}
		}

		// Append the unique root with its final stable handle and generalized root state.
		LinkHandle AddRoot(detail::ArticulationBuilderState& builder, EArticulationRootType root_type, ArticulationLinkDesc const& link, m4x4 const& link_to_world, v8motion velocity)
		{
			auto& state = *builder.m_articulation;
			if (!state.m_links.empty())
				throw std::logic_error("An articulation can contain exactly one root");

			detail::ValidateArticulationTransform(link_to_world, "Articulation root transform");
			ValidateLink(link, root_type == EArticulationRootType::Floating);
			if (!IsFinite(velocity.ang) || !IsFinite(velocity.lin))
				throw std::invalid_argument("Articulation root velocity must be finite");
			if (root_type == EArticulationRootType::Fixed && velocity != v8motion{})
				throw std::invalid_argument("A fixed articulation root cannot have velocity");

			// Floating roots reserve the first six generalized velocity entries in angular-then-linear order.
			state.m_root_type = root_type;
			if (root_type == EArticulationRootType::Floating)
			{
				state.m_velocity.resize(6);
				state.m_force.resize(6);
				state.m_acceleration.resize(6);
				state.m_response.resize(6);
				detail::StoreSpatialMotion(state.m_velocity, 0, velocity);
			}

			auto handle = LinkHandle{
				.m_index = 0,
				.m_generation = state.m_link_generation,
			};
			state.m_links.push_back(detail::ArticulationLinkState{
				.m_handle = handle,
				.m_parent_index = -1,
				.m_position_offset = 0,
				.m_velocity_offset = 0,
				.m_link = link,
				.m_link_to_world = link_to_world,
			});
			state.m_kinematics_dirty = true;
			return handle;
		}
	}

	namespace detail
	{
		// Return a validated link state or throw without indexing caller-controlled handle data.
		ArticulationLinkState& CheckedLink(ArticulationState& state, LinkHandle link)
		{
			if (!link || link.m_generation != state.m_link_generation || link.m_index >= state.m_links.size())
				throw std::out_of_range("Articulation link handle is stale or belongs to another articulation");

			auto& result = state.m_links[link.m_index];
			if (result.m_handle != link)
				throw std::out_of_range("Articulation link handle generation does not match its slot");

			return result;
		}

		// Return a validated link state or throw without indexing caller-controlled handle data.
		ArticulationLinkState const& CheckedLink(ArticulationState const& state, LinkHandle link)
		{
			return CheckedLink(const_cast<ArticulationState&>(state), link);
		}

		// Validate a finite rigid transform used by articulation topology or state.
		void ValidateArticulationTransform(m4x4 const& transform, char const* name)
		{
			if (!IsFinite(transform.x) || !IsFinite(transform.y) || !IsFinite(transform.z) || !IsFinite(transform.w) || !IsAffine(transform) || !IsOrthonormal(transform.rot, 1.0e-4f))
				throw std::invalid_argument(std::format("{} must be a finite rigid transform", name));
		}

		// Load six contiguous generalized values as one padded spatial motion vector.
		v8motion LoadSpatialMotion(std::span<float const> values, int offset)
		{
			if (offset < 0 || offset + 6 > isize(values))
				throw std::out_of_range("Spatial motion range exceeds generalized storage");

			return v8motion{
				values[offset + 0], values[offset + 1], values[offset + 2],
				values[offset + 3], values[offset + 4], values[offset + 5],
			};
		}

		// Store one padded spatial motion vector into six contiguous generalized values.
		void StoreSpatialMotion(std::span<float> values, int offset, v8motion motion)
		{
			if (offset < 0 || offset + 6 > isize(values))
				throw std::out_of_range("Spatial motion range exceeds generalized storage");

			values[offset + 0] = motion.ang.x;
			values[offset + 1] = motion.ang.y;
			values[offset + 2] = motion.ang.z;
			values[offset + 3] = motion.lin.x;
			values[offset + 4] = motion.lin.y;
			values[offset + 5] = motion.lin.z;
		}
	}

	// Construct a validated articulation from a consumed builder state.
	Articulation::Articulation(std::unique_ptr<detail::ArticulationState> state)
		:m_state(std::move(state))
	{
	}

	// Destroy the owned articulation state.
	Articulation::~Articulation() = default;

	// Move an articulation without changing its stable identity or link handles.
	Articulation::Articulation(Articulation&& rhs) noexcept = default;

	// Move an articulation without changing its stable identity or link handles.
	Articulation& Articulation::operator=(Articulation&& rhs) noexcept = default;

	// Return the stable identity used by future link constraint endpoints.
	ArticulationId Articulation::Id() const
	{
		return CheckedState(m_state).m_id;
	}

	// Return whether the root is fixed or contributes a six-velocity floating base.
	EArticulationRootType Articulation::RootType() const
	{
		return CheckedState(m_state).m_root_type;
	}

	// Return the root link handle.
	LinkHandle Articulation::Root() const
	{
		auto const& state = CheckedState(m_state);
		return state.m_links.front().m_handle;
	}

	// Return the number of physical links in the tree.
	int Articulation::LinkCount() const
	{
		return isize(CheckedState(m_state).m_links);
	}

	// Return the generalized velocity dimension, including six root velocities for a floating base.
	int Articulation::DofCount() const
	{
		return isize(CheckedState(m_state).m_velocity);
	}

	// Return the number of reduced coordinates owned by a non-root link joint.
	int Articulation::JointDofCount(LinkHandle link) const
	{
		auto const& state = CheckedState(m_state);
		return detail::CheckedLink(state, link).m_joint.m_dof_count;
	}

	// Return the parent link, or an invalid handle for the root.
	LinkHandle Articulation::Parent(LinkHandle link) const
	{
		auto const& state = CheckedState(m_state);
		auto const& link_state = detail::CheckedLink(state, link);
		return link_state.m_parent_index == -1
			? LinkHandle{}
			: state.m_links[link_state.m_parent_index].m_handle;
	}

	// Return a link handle by stable topological order.
	LinkHandle Articulation::LinkAt(int link_index) const
	{
		auto const& state = CheckedState(m_state);
		if (link_index < 0 || link_index >= isize(state.m_links))
			throw std::out_of_range("Articulation link index is out of range");

		return state.m_links[link_index].m_handle;
	}

	// Return immutable mass and collision-proxy data for a link.
	ArticulationLinkDesc const& Articulation::LinkDescription(LinkHandle link) const
	{
		auto const& state = CheckedState(m_state);
		return detail::CheckedLink(state, link).m_link;
	}

	// Return immutable reduced-joint topology for a non-root link.
	ArticulationJointDesc const& Articulation::JointDescription(LinkHandle link) const
	{
		auto const& state = CheckedState(m_state);
		auto const& link_state = detail::CheckedLink(state, link);
		if (link_state.m_parent_index == -1)
			throw std::invalid_argument("The articulation root does not have a reduced joint");

		return link_state.m_joint;
	}

	// Return the current world transform of a link after lazily refreshing kinematics.
	m4x4 const& Articulation::LinkToWorld(LinkHandle link) const
	{
		auto& state = CheckedState(m_state);
		if (state.m_kinematics_dirty)
			const_cast<Articulation*>(this)->UpdateKinematics();

		return detail::CheckedLink(state, link).m_link_to_world;
	}

	// Return the current link-frame spatial velocity after lazily refreshing kinematics.
	v8motion Articulation::LinkVelocity(LinkHandle link) const
	{
		auto& state = CheckedState(m_state);
		if (state.m_kinematics_dirty)
			const_cast<Articulation*>(this)->UpdateKinematics();

		return detail::CheckedLink(state, link).m_link_velocity;
	}

	// Return the root world transform.
	m4x4 const& Articulation::RootToWorld() const
	{
		auto const& state = CheckedState(m_state);
		return state.m_links.front().m_link_to_world;
	}

	// Replace the root world transform and invalidate derived link kinematics.
	void Articulation::RootToWorld(m4x4 const& root_to_world)
	{
		auto& state = CheckedState(m_state);
		detail::ValidateArticulationTransform(root_to_world, "Articulation root transform");
		Wake();
		state.m_links.front().m_link_to_world = root_to_world;
		state.m_kinematics_dirty = true;
	}

	// Return the floating root's link-frame spatial velocity, or zero for a fixed root.
	v8motion Articulation::RootVelocity() const
	{
		auto const& state = CheckedState(m_state);
		switch (state.m_root_type)
		{
			case EArticulationRootType::Fixed:
			{
				return {};
			}
			case EArticulationRootType::Floating:
			{
				return detail::LoadSpatialMotion(state.m_velocity, 0);
			}
			default:
			{
				throw std::runtime_error("Articulation root type is invalid");
			}
		}
	}

	// Replace the floating root's link-frame spatial velocity.
	void Articulation::RootVelocity(v8motion velocity)
	{
		auto& state = CheckedState(m_state);
		if (state.m_root_type != EArticulationRootType::Floating)
			throw std::logic_error("A fixed articulation root cannot have velocity");
		if (!IsFinite(velocity.ang) || !IsFinite(velocity.lin))
			throw std::invalid_argument("Articulation root velocity must be finite");

		Wake();
		detail::StoreSpatialMotion(state.m_velocity, 0, velocity);
		state.m_kinematics_dirty = true;
	}

	// Return the current reduced positions for one non-root joint.
	std::span<float const> Articulation::JointPosition(LinkHandle link) const
	{
		auto const& state = CheckedState(m_state);
		auto const& link_state = detail::CheckedLink(state, link);
		return std::span<float const>{state.m_position}.subspan(link_state.m_position_offset, link_state.m_joint.m_dof_count);
	}

	// Replace all reduced positions for one non-root joint.
	void Articulation::JointPosition(LinkHandle link, std::span<float const> position)
	{
		auto& state = CheckedState(m_state);
		auto const& link_state = detail::CheckedLink(state, link);
		if (isize(position) != link_state.m_joint.m_dof_count)
			throw std::invalid_argument("Joint position dimension does not match its articulation joint");
		if (!std::ranges::all_of(position, [](float value) { return IsFinite(value); }))
			throw std::invalid_argument("Joint positions must be finite");

		Wake();
		std::ranges::copy(position, state.m_position.begin() + link_state.m_position_offset);
		state.m_kinematics_dirty = true;
	}

	// Return the current reduced velocities for one non-root joint.
	std::span<float const> Articulation::JointVelocity(LinkHandle link) const
	{
		auto const& state = CheckedState(m_state);
		auto const& link_state = detail::CheckedLink(state, link);
		return std::span<float const>{state.m_velocity}.subspan(link_state.m_velocity_offset, link_state.m_joint.m_dof_count);
	}

	// Replace all reduced velocities for one non-root joint.
	void Articulation::JointVelocity(LinkHandle link, std::span<float const> velocity)
	{
		auto& state = CheckedState(m_state);
		auto const& link_state = detail::CheckedLink(state, link);
		if (isize(velocity) != link_state.m_joint.m_dof_count)
			throw std::invalid_argument("Joint velocity dimension does not match its articulation joint");
		if (!std::ranges::all_of(velocity, [](float value) { return IsFinite(value); }))
			throw std::invalid_argument("Joint velocities must be finite");

		Wake();
		std::ranges::copy(velocity, state.m_velocity.begin() + link_state.m_velocity_offset);
		state.m_kinematics_dirty = true;
	}

	// Return the current applied generalized forces for one non-root joint.
	std::span<float const> Articulation::JointForce(LinkHandle link) const
	{
		auto const& state = CheckedState(m_state);
		auto const& link_state = detail::CheckedLink(state, link);
		return std::span<float const>{state.m_force}.subspan(link_state.m_velocity_offset, link_state.m_joint.m_dof_count);
	}

	// Replace all applied generalized forces for one non-root joint.
	void Articulation::JointForce(LinkHandle link, std::span<float const> force)
	{
		auto& state = CheckedState(m_state);
		auto const& link_state = detail::CheckedLink(state, link);
		if (isize(force) != link_state.m_joint.m_dof_count)
			throw std::invalid_argument("Joint force dimension does not match its articulation joint");
		if (!std::ranges::all_of(force, [](float value) { return IsFinite(value); }))
			throw std::invalid_argument("Joint forces must be finite");

		if (std::ranges::any_of(force, [](float value) { return value != 0.0f; }))
			Wake();

		std::ranges::copy(force, state.m_force.begin() + link_state.m_velocity_offset);
	}

	// Return the floating root's applied generalized wrench, or zero for a fixed root.
	v8force Articulation::RootForce() const
	{
		auto const& state = CheckedState(m_state);
		switch (state.m_root_type)
		{
			case EArticulationRootType::Fixed:
			{
				return {};
			}
			case EArticulationRootType::Floating:
			{
				auto const motion = detail::LoadSpatialMotion(state.m_force, 0);
				return v8force{motion.ang, motion.lin};
			}
			default:
			{
				throw std::runtime_error("Articulation root type is invalid");
			}
		}
	}

	// Replace the floating root's applied generalized wrench.
	void Articulation::RootForce(v8force force)
	{
		auto& state = CheckedState(m_state);
		switch (state.m_root_type)
		{
			case EArticulationRootType::Fixed:
			{
				throw std::logic_error("A fixed articulation root cannot have a generalized wrench");
			}
			case EArticulationRootType::Floating:
			{
				break;
			}
			default:
			{
				throw std::runtime_error("Articulation root type is invalid");
			}
		}
		if (!IsFinite(force.ang) || !IsFinite(force.lin))
			throw std::invalid_argument("Articulation root force must be finite");

		if (detail::HasNonZeroComponent(force))
			Wake();

		detail::StoreSpatialMotion(state.m_force, 0, v8motion{force.ang, force.lin});
	}

	// Return the external link-frame wrench applied at a link origin.
	v8force Articulation::ExternalForce(LinkHandle link) const
	{
		auto const& state = CheckedState(m_state);
		return detail::CheckedLink(state, link).m_external_force;
	}

	// Replace the external link-frame wrench applied at a link origin.
	void Articulation::ExternalForce(LinkHandle link, v8force force)
	{
		auto& state = CheckedState(m_state);
		if (!IsFinite(force.ang) || !IsFinite(force.lin))
			throw std::invalid_argument("Articulation external force must be finite");

		if (detail::HasNonZeroComponent(force))
			Wake();

		detail::CheckedLink(state, link).m_external_force = force;
	}

	// Accumulate an external link-frame wrench at a link origin.
	void Articulation::ApplyExternalForce(LinkHandle link, v8force force)
	{
		auto& state = CheckedState(m_state);
		if (!IsFinite(force.ang) || !IsFinite(force.lin))
			throw std::invalid_argument("Articulation external force must be finite");

		if (detail::HasNonZeroComponent(force))
			Wake();

		detail::CheckedLink(state, link).m_external_force += force;
	}

	// Return the world-space gravity field sampled by one link.
	v4 Articulation::GravityWS(LinkHandle link) const
	{
		auto const& state = CheckedState(m_state);
		return detail::CheckedLink(state, link).m_gravity_ws;
	}

	// Set the world-space gravity field evaluated at one link during each dynamics solve.
	void Articulation::GravityWS(LinkHandle link, v4 gravity)
	{
		auto& state = CheckedState(m_state);
		if (!IsFinite(gravity) || gravity.w != 0.0f)
			throw std::invalid_argument("Articulation gravity must be a finite direction vector");

		auto& link_state = detail::CheckedLink(state, link);
		if (Any(link_state.m_gravity_ws != gravity))
			Wake();

		link_state.m_gravity_ws = gravity;
	}

	// Return whether the complete articulation tree is asleep.
	bool Articulation::Sleeping() const
	{
		return CheckedState(m_state).m_sleeping;
	}

	// Put the complete articulation tree to sleep or wake it immediately.
	void Articulation::Sleeping(bool sleeping)
	{
		if (sleeping)
			Sleep();
		else
			Wake();
	}

	// Put the complete articulation tree to sleep and discard all generalized motion and transient loads.
	void Articulation::Sleep()
	{
		auto& state = CheckedState(m_state);
		assert(!state.m_never_sleep);
		if (state.m_never_sleep)
			return;

		// A sleeping articulation owns one indivisible state, so no link can retain motion or a transient load independently.
		state.m_sleeping = true;
		state.m_sleep_timer_s = 0.0f;
		state.m_sleep_activity = false;
		std::ranges::fill(state.m_velocity, 0.0f);
		std::ranges::fill(state.m_acceleration, 0.0f);
		std::ranges::fill(state.m_response, 0.0f);
		std::ranges::fill(state.m_velocity_start, 0.0f);
		std::ranges::fill(state.m_velocity_midpoint, 0.0f);
		std::ranges::fill(state.m_acceleration_start, 0.0f);
		ClearForces();
		for (auto& link_state : state.m_links)
		{
			link_state.m_link_acceleration = {};
			link_state.m_link_acceleration_start = {};
			link_state.m_response_acceleration = {};
			link_state.m_response_impulse = {};
		}
		state.m_kinematics_dirty = true;
	}

	// Wake the complete articulation tree and restart its inactivity timer.
	void Articulation::Wake()
	{
		auto& state = CheckedState(m_state);
		state.m_sleeping = false;
		state.m_sleep_timer_s = 0.0f;
		state.m_sleep_activity = true;
	}

	// Return whether the complete articulation tree is immune to automatic sleeping.
	bool Articulation::NeverSleep() const
	{
		return CheckedState(m_state).m_never_sleep;
	}

	// Enable or disable automatic sleeping for the complete tree; enabling immunity also wakes it.
	void Articulation::NeverSleep(bool never_sleep)
	{
		auto& state = CheckedState(m_state);
		if (never_sleep)
			Wake();

		state.m_never_sleep = never_sleep;
	}

	// Advance the complete-tree sleep timer after one accepted frame and sleep only when every link remains below threshold.
	void Articulation::UpdateSleeping(float elapsed_seconds, float linear_velocity_threshold, float angular_velocity_threshold, float sleep_delay_s)
	{
		auto& state = CheckedState(m_state);
		if (state.m_sleeping)
			return;
		if (state.m_never_sleep)
		{
			state.m_sleep_timer_s = 0.0f;
			state.m_sleep_activity = false;
			return;
		}
		if (state.m_sleep_activity)
		{
			state.m_sleep_timer_s = 0.0f;
			state.m_sleep_activity = false;
			return;
		}
		if (state.m_kinematics_dirty)
			UpdateKinematics();

		// Any moving link resets the one tree-owned timer; links never enter or leave sleep independently.
		auto const linear_threshold_sq = Sqr(linear_velocity_threshold);
		auto const angular_threshold_sq = Sqr(angular_velocity_threshold);
		for (auto const& link_state : state.m_links)
		{
			if (LengthSq(link_state.m_link_velocity.lin) > linear_threshold_sq ||
				LengthSq(link_state.m_link_velocity.ang) > angular_threshold_sq)
			{
				state.m_sleep_timer_s = 0.0f;
				return;
			}
		}

		state.m_sleep_timer_s += elapsed_seconds;
		if (state.m_sleep_timer_s >= sleep_delay_s)
			Sleep();
	}

	// Clear every applied generalized force and external link wrench.
	void Articulation::ClearForces()
	{
		auto& state = CheckedState(m_state);
		std::ranges::fill(state.m_force, 0.0f);
		for (auto& link : state.m_links)
			link.m_external_force = {};
	}

	// Return the most recently solved floating-root spatial acceleration, or zero for a fixed root.
	v8motion Articulation::RootAcceleration() const
	{
		auto const& state = CheckedState(m_state);
		switch (state.m_root_type)
		{
			case EArticulationRootType::Fixed:
			{
				return {};
			}
			case EArticulationRootType::Floating:
			{
				return detail::LoadSpatialMotion(state.m_acceleration, 0);
			}
			default:
			{
				throw std::runtime_error("Articulation root type is invalid");
			}
		}
	}

	// Return the most recently solved reduced accelerations for one non-root joint.
	std::span<float const> Articulation::JointAcceleration(LinkHandle link) const
	{
		auto const& state = CheckedState(m_state);
		auto const& link_state = detail::CheckedLink(state, link);
		return std::span<float const>{state.m_acceleration}.subspan(link_state.m_velocity_offset, link_state.m_joint.m_dof_count);
	}

	// Begin an empty articulation topology.
	ArticulationBuilder::ArticulationBuilder()
		:m_state(std::make_unique<detail::ArticulationBuilderState>())
	{
		m_state->m_articulation = std::make_unique<detail::ArticulationState>();
		m_state->m_articulation->m_id = NewArticulationId();
		m_state->m_articulation->m_link_generation = NewLinkGeneration();
	}

	// Destroy an unconsumed builder state.
	ArticulationBuilder::~ArticulationBuilder() = default;

	// Move a builder without changing handles already returned from it.
	ArticulationBuilder::ArticulationBuilder(ArticulationBuilder&& rhs) noexcept = default;

	// Move a builder without changing handles already returned from it.
	ArticulationBuilder& ArticulationBuilder::operator=(ArticulationBuilder&& rhs) noexcept = default;

	// Add the single world-anchored root link.
	LinkHandle ArticulationBuilder::AddFixedRoot(ArticulationLinkDesc const& link, m4x4 const& link_to_world)
	{
		return AddRoot(CheckedBuilder(m_state), EArticulationRootType::Fixed, link, link_to_world, {});
	}

	// Add the single six-velocity floating root link.
	LinkHandle ArticulationBuilder::AddFloatingRoot(ArticulationLinkDesc const& link, m4x4 const& link_to_world, v8motion velocity)
	{
		return AddRoot(CheckedBuilder(m_state), EArticulationRootType::Floating, link, link_to_world, velocity);
	}

	// Add a physical child link connected to an existing parent by a zero-to-six-DOF reduced joint.
	LinkHandle ArticulationBuilder::AddLink(LinkHandle parent, ArticulationJointDesc const& joint, ArticulationLinkDesc const& link)
	{
		auto& builder = CheckedBuilder(m_state);
		auto& state = *builder.m_articulation;
		if (state.m_links.empty())
			throw std::logic_error("Add an articulation root before adding child links");

		auto const& parent_state = detail::CheckedLink(state, parent);
		ValidateJoint(joint);
		ValidateLink(link, true);

		// Append generalized state contiguously so every joint is a directly addressable dirty range.
		auto position_offset = isize(state.m_position);
		auto velocity_offset = isize(state.m_velocity);
		for (int axis_index = 0; axis_index != joint.m_dof_count; ++axis_index)
		{
			state.m_position.push_back(joint.m_initial_position[axis_index]);
			state.m_velocity.push_back(joint.m_initial_velocity[axis_index]);
			state.m_force.push_back(0.0f);
			state.m_acceleration.push_back(0.0f);
			state.m_response.push_back(0.0f);
		}

		auto handle = LinkHandle{
			.m_index = static_cast<uint32_t>(state.m_links.size()),
			.m_generation = state.m_link_generation,
		};
		state.m_links.push_back(detail::ArticulationLinkState{
			.m_handle = handle,
			.m_parent_index = static_cast<int>(parent_state.m_handle.m_index),
			.m_position_offset = position_offset,
			.m_velocity_offset = velocity_offset,
			.m_link = link,
			.m_joint = joint,
		});
		state.m_kinematics_dirty = true;
		return handle;
	}

	// Validate and consume the topology into a movable articulation.
	Articulation ArticulationBuilder::Build()
	{
		auto& builder = CheckedBuilder(m_state);
		if (builder.m_articulation->m_links.empty())
			throw std::logic_error("An articulation requires one fixed or floating root");

		builder.m_consumed = true;
		auto articulation = Articulation{std::move(builder.m_articulation)};

		// Reserve fixed-size integration scratch once so stepping performs no generalized-state allocation.
		auto& state = *articulation.m_state;
		state.m_position_start.resize(state.m_position.size());
		state.m_velocity_start.resize(state.m_velocity.size());
		state.m_velocity_midpoint.resize(state.m_velocity.size());
		state.m_acceleration_start.resize(state.m_acceleration.size());
		articulation.UpdateKinematics();
		return articulation;
	}
}
