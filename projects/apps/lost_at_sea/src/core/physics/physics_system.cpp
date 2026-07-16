//************************************
// Lost at Sea
//  Copyright (c) Rylogic Ltd 2025
//************************************
#include "src/forward.h"
#include "src/core/physics/physics_system.h"
#include "src/world/water/water_buoyancy_adapter.h"

namespace las
{
	namespace
	{
		// Return the physical seawater configuration used by Lost at Sea buoyancy.
		physics::GpuBuoyancy::Config BuoyancyConfig()
		{
			auto config = physics::GpuBuoyancy::Config{};
			config.m_fluid_density = PhysicsSystem::SeawaterDensityKgM3;
			config.m_enable_diagnostics = true;
			return config;
		}

		// Throw if a body descriptor does not contain one complete aligned shape and a physical density.
		void ValidateBodyDesc(PhysicsSystem::BodyDesc const& desc)
		{
			if (desc.m_shape_data.empty())
				throw std::runtime_error("Physics bodies require collision shape data");

			auto const* shape = desc.m_shape_data.begin<Shape>();
			if (!is_aligned(shape) || shape->m_size <= 0 || shape->m_size > isize(desc.m_shape_data))
				throw std::runtime_error("Physics body collision shape data is incomplete or misaligned");

			if (!std::isfinite(desc.m_density_kg_m3) || desc.m_density_kg_m3 <= 0.0f)
				throw std::runtime_error("Physics bodies require a positive finite average density");
		}
	}

	// Return an invalid body handle sentinel.
	PhysicsSystem::BodyHandle PhysicsSystem::BodyHandle::Invalid()
	{
		return BodyHandle{ -1, -1 };
	}

	// Return true if this handle could refer to a registered body.
	bool PhysicsSystem::BodyHandle::IsValid() const
	{
		return m_index >= 0 && m_generation >= 0;
	}

	// Construct an invalid body snapshot.
	PhysicsSystem::BodySnapshot::BodySnapshot()
		:m_handle(BodyHandle::Invalid())
		,m_o2w(m4x4::Identity())
		,m_step_index(-1)
		,m_valid()
	{}

	// Create an owned buoyancy hull registration.
	PhysicsSystem::BuoyancyHullRegistration::BuoyancyHullRegistration(PhysicsSystem& physics, BodyHandle handle)
		:m_physics(&physics)
		,m_handle(handle)
	{
	}

	// Construct an empty buoyancy hull registration.
	PhysicsSystem::BuoyancyHullRegistration::BuoyancyHullRegistration()
		:m_physics()
		,m_handle(BodyHandle::Invalid())
	{
	}

	// Move a buoyancy hull registration without releasing it.
	PhysicsSystem::BuoyancyHullRegistration::BuoyancyHullRegistration(BuoyancyHullRegistration&& rhs) noexcept
		:m_physics(std::exchange(rhs.m_physics, nullptr))
		,m_handle(std::exchange(rhs.m_handle, BodyHandle::Invalid()))
	{
	}

	// Release the current registration and then take ownership of another one.
	PhysicsSystem::BuoyancyHullRegistration& PhysicsSystem::BuoyancyHullRegistration::operator=(BuoyancyHullRegistration&& rhs) noexcept
	{
		if (this != &rhs)
		{
			Reset();
			m_physics = std::exchange(rhs.m_physics, nullptr);
			m_handle = std::exchange(rhs.m_handle, BodyHandle::Invalid());
		}
		return *this;
	}

	// Release the buoyancy hull registration when ownership leaves scope.
	PhysicsSystem::BuoyancyHullRegistration::~BuoyancyHullRegistration()
	{
		Reset();
	}

	// Release the buoyancy hull registration if one is owned.
	void PhysicsSystem::BuoyancyHullRegistration::Reset() noexcept
	{
		auto* physics = std::exchange(m_physics, nullptr);
		auto handle = std::exchange(m_handle, BodyHandle::Invalid());
		if (physics != nullptr)
		{
			physics->ReleaseBuoyancyHull(handle);
		}
	}

	// Return true if this object owns a buoyancy hull registration.
	bool PhysicsSystem::BuoyancyHullRegistration::IsValid() const
	{
		return m_physics != nullptr && m_handle.IsValid();
	}

	// Construct the LAS physics facade on the simulation owner thread.
	PhysicsSystem::PhysicsSystem(Renderer& rdr)
		:m_owner_thread_id(std::this_thread::get_id())
		,m_engine(physics::EngineConfig{}, nullptr, rdr.D3DDevice())
		,m_gpu_buoyancy(std::make_unique<physics::GpuBuoyancy>(rdr.D3DDevice(), m_engine,
			BuoyancyConfig(),
			[this](int body_slot_index) { return BodySlotStepIndex(body_slot_index); },
			[this](int body_slot_index) { return BodySlotState(body_slot_index); },
			water::BuoyancyAdapter::Extension())
		)
		,m_body_slots()
		,m_free_slots()
		,m_step_bodies()
		,m_snapshot_mutex()
		,m_snapshots()
		,m_config_mutex()
		,m_gravity_ws(v4{0, 0, -9.81f, 0})
		,m_step_pending()
		,m_engine_step_pending()
	{
		// Renderer does not expose a shared compute shader cache yet. Passing nullptr keeps the physics engine on its normal uncached compiler
		// path while still sharing the renderer's D3D device.
		PublishSnapshots();
	}

	// Destroy the LAS physics facade after all in-flight work has been completed by the owner.
	PhysicsSystem::~PhysicsSystem()
	{
		PR_ASSERT(PR_DBG, !m_step_pending, "PhysicsSystem destroyed with a pending physics step");
	}

	// Create a physics-owned body and publish its initial snapshot.
	PhysicsSystem::BodyHandle PhysicsSystem::CreateBody(BodyDesc desc)
	{
		CheckOwnerThread();
		CheckNoStepPending("PhysicsSystem::CreateBody");
		ValidateBodyDesc(desc);

		auto shape_data = std::move(desc.m_shape_data);
		auto* shape = shape_data.begin<Shape>();
		auto body = std::make_unique<RigidBody>(shape, desc.m_o2w);
		body->Shape(shape, desc.m_density_kg_m3, true);
		body->NeverSleep(desc.m_never_sleep);

		if (!m_free_slots.empty())
		{
			auto const slot_index = m_free_slots.back();
			m_free_slots.pop_back();

			auto& slot = m_body_slots[slot_index];
			slot.m_buoyancy_hull.Reset();
			slot.m_shape_data = std::move(shape_data);
			slot.m_body = std::move(body);
			slot.m_step_index = -1;
			auto handle = BodyHandle{ slot_index, slot.m_generation };
			PublishSnapshots();
			return handle;
		}

		auto const slot_index = static_cast<int>(m_body_slots.size());
		m_body_slots.push_back(BodySlot{
			.m_shape_data = std::move(shape_data),
			.m_body = std::move(body),
			.m_generation = 0,
			.m_step_index = -1,
			.m_buoyancy_hull = {},
		});
		auto handle = BodyHandle{ slot_index, 0 };
		PublishSnapshots();
		return handle;
	}

	// Destroy a physics-owned body and invalidate its published snapshot.
	void PhysicsSystem::DestroyBody(BodyHandle handle)
	{
		CheckOwnerThread();
		CheckNoStepPending("PhysicsSystem::DestroyBody");
		if (!handle.IsValid())
		{
			return;
		}

		auto& slot = Slot(handle);
		ReleaseBuoyancyHull(handle);
		slot.m_body.reset();
		slot.m_shape_data.clear();
		slot.m_step_index = -1;
		++slot.m_generation;
		m_free_slots.push_back(handle.m_index);
		PublishSnapshots();
	}

	// Return true if 'handle' currently refers to a registered body.
	bool PhysicsSystem::IsValid(BodyHandle handle) const
	{
		if (!handle.IsValid())
		{
			return false;
		}

		auto lock = std::lock_guard<std::mutex>(m_snapshot_mutex);
		return handle.m_index < static_cast<int>(m_snapshots.size()) &&
			m_snapshots[handle.m_index].m_valid &&
			m_snapshots[handle.m_index].m_handle.m_generation == handle.m_generation;
	}

	// Return the compact body index used by the most recent/current Engine::Step() call.
	int PhysicsSystem::StepIndex(BodyHandle handle) const
	{
		if (std::this_thread::get_id() == m_owner_thread_id)
		{
			return Slot(handle).m_step_index;
		}

		return Snapshot(handle).m_step_index;
	}

	// Return the latest published body transform and step metadata.
	PhysicsSystem::BodySnapshot PhysicsSystem::Snapshot(BodyHandle handle) const
	{
		if (!handle.IsValid())
		{
			throw std::runtime_error("Invalid PhysicsSystem body handle");
		}

		auto lock = std::lock_guard<std::mutex>(m_snapshot_mutex);
		if (handle.m_index >= static_cast<int>(m_snapshots.size()))
		{
			throw std::runtime_error("Invalid PhysicsSystem body handle");
		}

		auto const& snapshot = m_snapshots[handle.m_index];
		if (!snapshot.m_valid || snapshot.m_handle.m_generation != handle.m_generation)
		{
			throw std::runtime_error("Invalid PhysicsSystem body handle");
		}

		return snapshot;
	}

	// Return a body's collision shape while on the simulation owner thread.
	Shape const& PhysicsSystem::BodyShape(BodyHandle handle) const
	{
		return Slot(handle).m_body->Shape();
	}

	// Recalculate a body's mass properties from its unchanged shape and a new average density.
	void PhysicsSystem::SetBodyDensity(BodyHandle handle, float density_kg_m3)
	{
		CheckOwnerThread();
		CheckNoStepPending("PhysicsSystem::SetBodyDensity");
		if (!std::isfinite(density_kg_m3) || density_kg_m3 <= 0.0f)
			throw std::runtime_error("Physics body density must be positive and finite");

		auto& body = *Slot(handle).m_body;

		// Preserve motion rather than momentum so a live tuning change does not create an artificial velocity impulse.
		auto velocity_ws = body.VelocityWS();
		auto mass_properties = CalcMassProperties(body.Shape(), density_kg_m3);
		body.SetMassProperties(physics::Inertia{mass_properties}, mass_properties.m_centre_of_mass);
		body.VelocityWS(velocity_ws);
	}

	// Return the latest GPU buoyancy diagnostic record for a body.
	physics::GpuBuoyancy::Diagnostics PhysicsSystem::LatestBuoyancyDiagnostics(BodyHandle handle) const
	{
		if (!IsValid(handle))
			return physics::GpuBuoyancy::Diagnostics{};

		return m_gpu_buoyancy->LatestDiagnostics(handle.m_index, handle.m_generation);
	}

	// Register a physics body's collision shape for buoyancy.
	PhysicsSystem::BuoyancyHullRegistration PhysicsSystem::RegisterBuoyancyHull(BodyHandle handle)
	{
		CheckOwnerThread();
		CheckNoStepPending("PhysicsSystem::RegisterBuoyancyHull");

		auto& slot = Slot(handle);
		if (slot.m_buoyancy_hull)
		{
			throw std::runtime_error("A buoyancy hull is already registered for this body");
		}

		slot.m_buoyancy_hull = m_gpu_buoyancy->RegisterCompositeHull(*slot.m_body, handle.m_index, handle.m_generation);
		return BuoyancyHullRegistration{ *this, handle };
	}

	// Release a buoyancy hull registration during RAII cleanup.
	void PhysicsSystem::ReleaseBuoyancyHull(BodyHandle handle) noexcept
	{
		if (!handle.IsValid())
		{
			return;
		}

		if (std::this_thread::get_id() != m_owner_thread_id)
		{
			PR_ASSERT(PR_DBG, false, "PhysicsSystem buoyancy hull registration released from the wrong thread");
			std::terminate();
		}
		if (m_step_pending)
		{
			PR_ASSERT(PR_DBG, false, "PhysicsSystem buoyancy hull registration released while a physics step is pending");
			std::terminate();
		}

		if (handle.m_index < 0 || handle.m_index >= static_cast<int>(m_body_slots.size()))
		{
			return;
		}

		auto& slot = m_body_slots[handle.m_index];
		if (slot.m_generation != handle.m_generation)
		{
			return;
		}

		slot.m_buoyancy_hull.Reset();
	}

	// Submit GPU physics work against the matching immutable water snapshot without waiting for completion.
	void PhysicsSystem::BeginStep(float dt, double time_s, water::Snapshot const& water_snapshot)
	{
		CheckOwnerThread();
		CheckNoStepPending("PhysicsSystem::BeginStep");

		// Copy the field before Engine::BeginStep invokes external-force generators, ensuring every buoyancy dispatch in this step sees this exact value snapshot.
		water::BuoyancyAdapter::SetField(*m_gpu_buoyancy, water_snapshot, time_s);

		m_step_pending = true;
		m_engine_step_pending = false;

		try
		{
			BuildStepBodyList();
			if (!m_step_bodies.empty())
			{
				m_engine.BeginStep(dt, std::span{ m_step_bodies }, time_s);
				m_engine_step_pending = true;
			}
		}
		catch (...)
		{
			m_step_pending = false;
			m_engine_step_pending = false;
			throw;
		}
	}

	// Wait for pending GPU physics work and publish immutable snapshots, returning an exception on failure.
	[[nodiscard]] std::exception_ptr PhysicsSystem::CompleteStep() noexcept
	{
		try
		{
			CheckOwnerThread();
			if (!m_step_pending)
			{
				throw std::runtime_error("PhysicsSystem::CompleteStep called without a pending step");
			}

			if (m_engine_step_pending)
			{
				m_engine.CompleteStep();
				m_engine_step_pending = false;
				m_gpu_buoyancy->CompleteStep();
			}

			PublishSnapshots();
			m_step_pending = false;
			return {};
		}
		catch (...)
		{
			return std::current_exception();
		}
	}

	// Step all registered rigid bodies synchronously against the matching immutable water snapshot.
	void PhysicsSystem::Step(float dt, double time_s, water::Snapshot const& water_snapshot)
	{
		BeginStep(dt, time_s, water_snapshot);
		if (auto ex = CompleteStep())
		{
			std::rethrow_exception(ex);
		}
	}

	// Set the world-space gravity applied to registered bodies before each step.
	void PhysicsSystem::GravityWS(v4 gravity_ws)
	{
		CheckOwnerThread();
		CheckNoStepPending("PhysicsSystem::GravityWS");

		auto lock = std::lock_guard<std::mutex>(m_config_mutex);
		m_gravity_ws = gravity_ws;
	}

	// Return the world-space gravity applied before each step.
	v4 PhysicsSystem::GravityWS() const
	{
		auto lock = std::lock_guard<std::mutex>(m_config_mutex);
		return m_gravity_ws;
	}

	// Throw if mutable physics state is touched from a thread other than the owner thread.
	void PhysicsSystem::CheckOwnerThread() const
	{
		if (std::this_thread::get_id() != m_owner_thread_id)
		{
			throw std::runtime_error("PhysicsSystem mutable state accessed from the wrong thread");
		}
	}

	// Throw if an operation would mutate state while a step is in flight.
	void PhysicsSystem::CheckNoStepPending(char const* operation) const
	{
		if (m_step_pending)
		{
			throw std::runtime_error(std::string(operation) + " cannot run while a physics step is pending");
		}
	}

	// Return true if 'handle' refers to a live body slot on the owner thread.
	bool PhysicsSystem::IsValidOnOwnerThread(BodyHandle handle) const
	{
		CheckOwnerThread();
		return handle.IsValid() &&
			handle.m_index < static_cast<int>(m_body_slots.size()) &&
			m_body_slots[handle.m_index].m_generation == handle.m_generation &&
			m_body_slots[handle.m_index].m_body != nullptr;
	}

	// Return the live body slot for 'handle' on the owner thread.
	PhysicsSystem::BodySlot& PhysicsSystem::Slot(BodyHandle handle)
	{
		CheckOwnerThread();
		if (!IsValidOnOwnerThread(handle))
		{
			throw std::runtime_error("Invalid PhysicsSystem body handle");
		}

		return m_body_slots[handle.m_index];
	}

	// Return the live body slot for 'handle' on the owner thread.
	PhysicsSystem::BodySlot const& PhysicsSystem::Slot(BodyHandle handle) const
	{
		CheckOwnerThread();
		if (!IsValidOnOwnerThread(handle))
		{
			throw std::runtime_error("Invalid PhysicsSystem body handle");
		}

		return m_body_slots[handle.m_index];
	}

	// Return the compact body index used by the current physics step for a body slot.
	int PhysicsSystem::BodySlotStepIndex(int body_slot_index) const
	{
		CheckOwnerThread();
		if (body_slot_index < 0 || body_slot_index >= static_cast<int>(m_body_slots.size()))
		{
			return -1;
		}

		auto const& slot = m_body_slots[body_slot_index];
		return slot.m_body != nullptr ? slot.m_step_index : -1;
	}

	// Return the live body state used by owner-thread diagnostic systems.
	physics::GpuBuoyancy::BodyState PhysicsSystem::BodySlotState(int body_slot_index) const
	{
		CheckOwnerThread();
		if (body_slot_index < 0 || body_slot_index >= static_cast<int>(m_body_slots.size()))
			return physics::GpuBuoyancy::BodyState{};

		auto const& slot = m_body_slots[body_slot_index];
		if (slot.m_body == nullptr)
			return physics::GpuBuoyancy::BodyState{};

		auto state = physics::GpuBuoyancy::BodyState{};
		state.m_o2w = slot.m_body->O2W();
		state.m_centre_of_mass_os = slot.m_body->CentreOfMassOS();
		state.m_ws_gravity = slot.m_body->GravityWS();
		state.m_valid = true;
		return state;
	}

	// Rebuild the compact body list submitted to the physics engine.
	void PhysicsSystem::BuildStepBodyList()
	{
		CheckOwnerThread();

		auto const gravity_ws = GravityWS();
		m_step_bodies.clear();
		for (auto& slot : m_body_slots)
		{
			slot.m_step_index = -1;
			if (slot.m_body == nullptr)
			{
				continue;
			}

			slot.m_body->GravityWS(gravity_ws);
			slot.m_step_index = static_cast<int>(m_step_bodies.size());
			m_step_bodies.push_back(slot.m_body.get());
		}
	}

	// Publish body state for render and camera readers.
	void PhysicsSystem::PublishSnapshots()
	{
		CheckOwnerThread();

		auto lock = std::lock_guard<std::mutex>(m_snapshot_mutex);
		m_snapshots.resize(m_body_slots.size());
		for (int slot_index = 0; slot_index != static_cast<int>(m_body_slots.size()); ++slot_index)
		{
			auto const& slot = m_body_slots[slot_index];
			auto& snapshot = m_snapshots[slot_index];
			snapshot.m_handle = BodyHandle{ slot_index, slot.m_generation };
			snapshot.m_step_index = slot.m_step_index;
			snapshot.m_valid = slot.m_body != nullptr;
			snapshot.m_o2w = snapshot.m_valid ? slot.m_body->O2W() : m4x4::Identity();
		}
	}
}
