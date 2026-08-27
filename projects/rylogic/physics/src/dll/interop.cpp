//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2016
//*********************************************
#include "pr/physics/physics.h"
#include "pr/physics/physics-dll.h"
#include "pr/collision/shape_array.h"
#include "pr/collision/shape_polytope.h"
#include "pr/container/byte_data.h"
#include "physics/src/compute/physics_types.h"
#include "physics/src/dll/context.h"
#include "physics/src/dll/interop.h"

namespace
{
	// Keep the implementation's wire-record vocabulary local while the public API uses concise names in pr::physics.
	using PhysicsEngineHandle = pr::physics::EngineHandle;
	using PhysicsShapeHandle = pr::physics::ShapeHandle;
	using PhysicsBodyHandle = pr::physics::BodyHandle;
	using PhysicsStatus = pr::physics::EStatus;
	using PhysicsStructId = pr::physics::EStructId;
	using PhysicsMotionType = pr::physics::EMotionType;
	using PhysicsMassMode = pr::physics::EMassMode;
	using PhysicsBodyCommandType = pr::physics::ECommand;
	using PhysicsEventType = pr::physics::EEvent;
	using PhysicsBodyFlags = pr::physics::EBodyFlags;
	using PhysicsStructHeader = pr::physics::StructHeader;
	using PhysicsVector4 = pr::physics::Vector4;
	using PhysicsMatrix4 = pr::physics::Matrix4;
	using PhysicsSpatialVector = pr::physics::SpatialVector;
	using PhysicsInertia = pr::physics::InertiaProperties;
	using PhysicsEngineConfig = pr::physics::Config;
	using PhysicsMaterial = pr::physics::MaterialProperties;
	using PhysicsShapeCommon = pr::physics::ShapeCommon;
	using PhysicsSphereShape = pr::physics::SphereShape;
	using PhysicsBoxShape = pr::physics::BoxShape;
	using PhysicsLineShape = pr::physics::LineShape;
	using PhysicsTriangleShape = pr::physics::TriangleShape;
	using PhysicsBodyDesc = pr::physics::BodyDesc;
	using PhysicsBodyState = pr::physics::BodyState;
	using PhysicsBodyCommand = pr::physics::BodyCommand;
	using PhysicsBodySnapshot = pr::physics::BodySnapshot;
	using PhysicsEvent = pr::physics::Event;
	using PhysicsStepProfile = pr::physics::StepProfile;
	using PhysicsDiagnostics = pr::physics::Diagnostics;
}

namespace pr::physics
{
	thread_local std::string g_last_error;

		// Converts expected ABI failures into status codes without allowing exceptions across the C boundary.
		struct ApiException : std::runtime_error
		{
			PhysicsStatus m_status;

			ApiException(PhysicsStatus status, std::string message)
				: std::runtime_error(std::move(message))
				, m_status(status)
			{}
		};

		// A generation slot retains its generation after object destruction so stale handles remain detectable.
		template <typename T>
		struct ObjectSlot
		{
			std::uint32_t m_generation = 1;
			std::unique_ptr<T> m_object;
		};

		struct ShapeRecord
		{
			byte_data<16> m_data;

			// Child handles retained by a compound shape, in declaration order. A compound owns a private copy
			// of each child blob, but retaining the handles keeps the caller's child identities alive and
			// meaningful for as long as the compound that was built from them exists.
			std::vector<PhysicsShapeHandle> m_children;

			std::uint32_t m_body_refs = 0;
			std::uint32_t m_compound_refs = 0;

			collision::Shape* Shape()
			{
				return m_data.begin<collision::Shape>();
			}
			collision::Shape const* Shape() const
			{
				return m_data.begin<collision::Shape>();
			}
		};

		struct BodyRecord
		{
			std::unique_ptr<RigidBody> m_body;
			PhysicsShapeHandle m_shape = {};
			std::uint64_t m_user_tag = {};
			PhysicsMotionType m_motion_type = PhysicsMotionType::Dynamic;
			bool m_enabled = true;
			bool m_sleep_before_step = false;
		};

		// Translate an engine-side contact child index into the ABI's child identity.
		// The engine indexes convex leaves from zero for every body, so a body with a primitive root would
		// otherwise be indistinguishable from the first child of a compound.
		std::uint32_t ContactChildId(RigidBody const* body, int child_id)
		{
			if (body == nullptr || body->Shape().m_type != collision::EShape::Array || child_id < 0)
				return PHYSICS_NO_CHILD;

			return static_cast<std::uint32_t>(child_id);
		}

		struct EngineRecord
		{
			// Guards this engine's state. It is separate from the process-wide registry lock so that a long
			// operation on one engine, notably the GPU wait inside CompleteStep, does not block calls made
			// against unrelated engines on their own simulation threads. Recursive because ABI helpers call
			// each other while already holding it.
			std::recursive_mutex m_lock;

			// Set under m_lock when the engine is destroyed or abandoned. A caller that pinned this record
			// just before destruction keeps the memory alive, so this flag is what makes its handle
			// observably stale rather than silently operating on a destroyed engine.
			bool m_retired;

			std::uint16_t m_cookie;
			DWORD m_owner_thread_id;
			std::vector<ObjectSlot<ShapeRecord>> m_shapes;
			std::vector<ObjectSlot<BodyRecord>> m_bodies;
			std::unordered_map<RigidBody const*, PhysicsBodyHandle> m_body_handles;
			std::vector<RigidBody*> m_step_bodies;
			std::vector<PhysicsEvent> m_events;
			std::uint64_t m_submitted_step;
			std::uint64_t m_completed_step;
			std::unique_ptr<Engine> m_engine;

			EngineRecord(std::uint16_t cookie, EngineConfig const& config, ID3D12Device4* external_device)
				: m_lock()
				, m_retired(false)
				, m_cookie(cookie)
				, m_owner_thread_id(GetCurrentThreadId())
				, m_shapes()
				, m_bodies()
				, m_body_handles()
				, m_step_bodies()
				, m_events()
				, m_submitted_step()
				, m_completed_step()
				, m_engine(new Engine(config, nullptr, external_device))
			{
				// Contacts are buffered while CompleteStep unpacks native results; no managed callback occurs from stepping.
				m_engine->Collisions += [this](Engine&, std::span<RbContact const> contacts)
				{
					for (auto const& contact : contacts)
					{
						auto iter_a = m_body_handles.find(contact.m_objA);
						auto iter_b = m_body_handles.find(contact.m_objB);
						if (iter_a == m_body_handles.end() || iter_b == m_body_handles.end())
							continue;

						auto evt = PhysicsEvent{
							.header = {sizeof(PhysicsEvent), PHYSICS_STRUCT_VERSION},
							.type = PhysicsEventType::Contact,
							.point_count = static_cast<std::uint32_t>(contact.Count()),
							.body_a = iter_a->second,
							.body_b = iter_b->second,
							.normal = {},
							.points = {},
							.depth = contact.m_depth,
							.material_a = contact.m_mat_idA,
							.material_b = contact.m_mat_idB,
							.child_a = ContactChildId(contact.m_objA, contact.m_child_idA),
							.child_b = ContactChildId(contact.m_objB, contact.m_child_idB),
						};

						// Contact geometry is reported in world space so snapshots and events share one coordinate frame.
						auto const& a2w = contact.m_objA->O2W();
						auto normal = (a2w.rot * contact.m_axis).w0();
						memcpy(&evt.normal, &normal, sizeof(normal));
						for (auto i = 0; i != contact.Count(); ++i)
						{
							auto point = a2w * contact.m_manifold[i];
							memcpy(&evt.points[i], &point, sizeof(point));
						}
						m_events.push_back(evt);
					}
				};

			}
		};

		struct EngineSlot
		{
			std::uint32_t m_generation = 1;

			// Shared rather than unique so a call in progress can pin the record and let the process-wide
			// registry lock go while it works.
			std::shared_ptr<EngineRecord> m_record;
		};

		// Engine slots use a process-level generation because engine handles are not scoped by another object.
		std::uint64_t MakeEngineHandle(std::size_t index, std::uint32_t generation)
		{
			return (std::uint64_t{generation} << 32) | (std::uint64_t{index} + 1);
		}
		std::size_t EngineIndex(PhysicsEngineHandle handle)
		{
			auto index = static_cast<std::uint32_t>(handle);
			if (index == 0)
				throw ApiException(PhysicsStatus::InvalidHandle, "Engine handle is null");

			return static_cast<std::size_t>(index - 1);
		}
		std::uint32_t EngineGeneration(PhysicsEngineHandle handle)
		{
			return static_cast<std::uint32_t>(handle >> 32);
		}

		// Child handles encode an engine cookie as well as slot generation to reject cross-engine use.
		std::uint64_t MakeChildHandle(std::uint16_t cookie, std::size_t index, std::uint32_t generation)
		{
			if (index >= 0xFFFFFFU)
				throw ApiException(PhysicsStatus::InternalError, "Physics handle capacity exhausted");

			return
				(std::uint64_t{cookie} << 48) |
				(std::uint64_t{generation & 0xFFFFFFU} << 24) |
				(std::uint64_t{index} + 1);
		}
		std::uint16_t ChildCookie(std::uint64_t handle)
		{
			return static_cast<std::uint16_t>(handle >> 48);
		}
		std::uint32_t ChildGeneration(std::uint64_t handle)
		{
			return static_cast<std::uint32_t>((handle >> 24) & 0xFFFFFFU);
		}
		std::size_t ChildIndex(std::uint64_t handle)
		{
			auto index = static_cast<std::uint32_t>(handle & 0xFFFFFFU);
			if (index == 0)
				throw ApiException(PhysicsStatus::InvalidHandle, "Physics object handle is null");

			return static_cast<std::size_t>(index - 1);
		}
		void AdvanceGeneration(std::uint32_t& generation)
		{
			generation = (generation + 1) & 0xFFFFFFU;
			if (generation == 0)
				generation = 1;
		}

		// Invoke an ABI implementation with complete C++ exception containment.
		template <typename Func>
		PhysicsStatus ApiCall(Func&& func) noexcept
		{
			try
			{
				g_last_error.clear();
				func();
				return PhysicsStatus::Success;
			}
			catch (ApiException const& ex)
			{
				g_last_error = ex.what();
				return ex.m_status;
			}
			catch (compute::DeviceRemovedException const& ex)
			{
				g_last_error = ex.what();
				return PhysicsStatus::DeviceRemoved;
			}
			catch (std::exception const& ex)
			{
				g_last_error = ex.what();
				return PhysicsStatus::InternalError;
			}
			catch (...)
			{
				g_last_error = "Unknown native physics failure";
				return PhysicsStatus::InternalError;
			}
		}

		// Validate the common size/version prefix while allowing future callers to append fields.
		template <typename T>
		T const& RequireStruct(T const* value)
		{
			if (value == nullptr)
				throw ApiException(PhysicsStatus::InvalidArgument, "Required structure pointer is null");
			if (value->header.version != PHYSICS_STRUCT_VERSION || value->header.size < sizeof(T))
				throw ApiException(PhysicsStatus::InvalidStruct, "Structure size or version is incompatible");

			return *value;
		}
		template <typename T>
		T& RequireOutputStruct(T* value)
		{
			if (value == nullptr)
				throw ApiException(PhysicsStatus::InvalidArgument, "Required output structure pointer is null");
			if (value->header.version != PHYSICS_STRUCT_VERSION || value->header.size < sizeof(T))
				throw ApiException(PhysicsStatus::InvalidStruct, "Output structure size or version is incompatible");

			return *value;
		}

		// Take a strong reference to the engine identified by 'handle'.
		// The process-wide registry lock is held only for the handle validation itself, so no caller ever
		// waits on the registry while another engine performs GPU work.
		std::shared_ptr<EngineRecord> PinEngine(PhysicsEngineHandle handle)
		{
			auto dll = PinDll();
			LockGuard lock(dll->m_mutex);
			auto& state = *dll->m_interop;
			auto index = EngineIndex(handle);
			if (index >= state.m_engines.size() || !state.m_engines[index])
				throw ApiException(PhysicsStatus::InvalidHandle, "Engine handle is invalid");

			auto& slot = *state.m_engines[index];
			if (slot.m_generation != EngineGeneration(handle) || !slot.m_record)
				throw ApiException(PhysicsStatus::StaleHandle, "Engine handle is stale");

			return slot.m_record;
		}

		// Clear an engine's registry slot so its handles become stale.
		// Separated from record teardown so the caller can retire the slot while holding only the engine
		// lock, and let the record itself be released after both locks are dropped.
		void RetireEngineSlot(PhysicsEngineHandle handle)
		{
			auto dll = PinDll();
			LockGuard lock(dll->m_mutex);
			auto& state = *dll->m_interop;
			auto index = EngineIndex(handle);
			if (index >= state.m_engines.size() || !state.m_engines[index])
				return;

			auto& slot = *state.m_engines[index];
			slot.m_record.reset();
			++slot.m_generation;
			if (slot.m_generation == 0)
				slot.m_generation = 1;
		}

		// Claim a checkpoint's engine identity so restored object handles keep working.
		// Cookies must be unique across live engines, so the uniqueness test and the assignment are made
		// together under the registry lock rather than sampling other engines' state unsynchronised.
		void ClaimEngineCookie(EngineRecord& engine, std::uint16_t cookie)
		{
			auto dll = PinDll();
			LockGuard lock(dll->m_mutex);
			for (auto const& slot : dll->m_interop->m_engines)
			{
				if (slot->m_record && slot->m_record.get() != &engine && slot->m_record->m_cookie == cookie)
					throw ApiException(PhysicsStatus::InvalidArgument, "Checkpoint engine identity is already active");
			}

			engine.m_cookie = cookie;
		}

		// Scoped access to one engine's state for the duration of an ABI call.
		// Ownership is pinned first and the engine lock is taken only after the registry lock has been
		// released, so the two are never held in that order and destruction cannot pull the record out from
		// under a concurrent call.
		struct EngineScope
		{
			// Declaration order matters: the lock is released before the pin is dropped, so the mutex is
			// never destroyed while still held.
			std::shared_ptr<EngineRecord> m_record;
			std::unique_lock<std::recursive_mutex> m_lock;

			explicit EngineScope(PhysicsEngineHandle handle)
				: m_record(PinEngine(handle))
				, m_lock(m_record->m_lock)
			{
				if (m_record->m_retired)
					throw ApiException(PhysicsStatus::StaleHandle, "Engine handle is stale");
			}
			EngineScope(EngineScope&&) = delete;
			EngineScope(EngineScope const&) = delete;
			EngineScope& operator=(EngineScope&&) = delete;
			EngineScope& operator=(EngineScope const&) = delete;

			EngineRecord& operator*() const
			{
				return *m_record;
			}
			EngineRecord* operator->() const
			{
				return m_record.get();
			}
		};

		void RequireOwner(EngineRecord const& engine)
		{
			if (engine.m_owner_thread_id != GetCurrentThreadId())
				throw ApiException(PhysicsStatus::WrongThread, "Mutable physics operation called from a non-owner OS thread");
		}
		void RequireIdle(EngineRecord const& engine)
		{
			if (engine.m_submitted_step != engine.m_completed_step)
				throw ApiException(PhysicsStatus::StepPending, "Physics operation is not valid while a step is pending");
		}

		template <typename T>
		std::pair<T&, std::size_t> RequireChild(EngineRecord& engine, std::uint64_t handle, std::vector<ObjectSlot<T>>& slots)
		{
			if (ChildCookie(handle) != engine.m_cookie)
				throw ApiException(PhysicsStatus::InvalidHandle, "Object belongs to a different physics engine");

			auto index = ChildIndex(handle);
			if (index >= slots.size())
				throw ApiException(PhysicsStatus::InvalidHandle, "Physics object handle is invalid");

			auto& slot = slots[index];
			if ((slot.m_generation & 0xFFFFFFU) != ChildGeneration(handle) || !slot.m_object)
				throw ApiException(PhysicsStatus::StaleHandle, "Physics object handle is stale");

			return {*slot.m_object, index};
		}
		ShapeRecord& RequireShape(EngineRecord& engine, PhysicsShapeHandle handle)
		{
			return RequireChild(engine, handle, engine.m_shapes).first;
		}
		BodyRecord& RequireBody(EngineRecord& engine, PhysicsBodyHandle handle)
		{
			return RequireChild(engine, handle, engine.m_bodies).first;
		}

		// Allocate from an inactive generation slot before growing the registry.
		template <typename T>
		std::pair<ObjectSlot<T>&, std::size_t> AllocateSlot(std::vector<ObjectSlot<T>>& slots)
		{
			for (auto i = std::size_t{}; i != slots.size(); ++i)
			{
				if (!slots[i].m_object)
					return {slots[i], i};
			}

			slots.emplace_back();
			return {slots.back(), slots.size() - 1};
		}

		m4x4 ToNative(PhysicsMatrix4 const& value)
		{
			static_assert(sizeof(value) == sizeof(m4x4));
			auto result = m4x4{};
			memcpy(&result, &value, sizeof(result));
			return result;
		}
		v4 ToNative(PhysicsVector4 const& value)
		{
			static_assert(sizeof(value) == sizeof(v4));
			auto result = v4{};
			memcpy(&result, &value, sizeof(result));
			return result;
		}
		v8force ToForce(PhysicsSpatialVector const& value)
		{
			return v8force{ToNative(value.angular), ToNative(value.linear)};
		}
		v8motion ToMotion(PhysicsSpatialVector const& value)
		{
			return v8motion{ToNative(value.angular), ToNative(value.linear)};
		}
		Inertia ToNative(PhysicsInertia const& value)
		{
			return Inertia{
				ToNative(value.diagonal),
				ToNative(value.products),
				value.centre_of_mass_and_mass.w,
				ToNative(value.centre_of_mass_and_mass).w0()};
		}
		template <typename TNative, typename TAbi>
		void CopyLayout(TAbi& dst, TNative const& src)
		{
			static_assert(sizeof(TNative) == sizeof(TAbi));
			memcpy(&dst, &src, sizeof(dst));
		}

		EngineConfig ToNative(PhysicsEngineConfig const& value)
		{
			auto config = EngineConfig{};
			config.max_collision_pairs = value.max_collision_pairs;
			config.sleeping_enabled = value.sleeping_enabled != 0;
			config.sleep_velocity_threshold_lin = value.sleep_velocity_threshold_linear;
			config.sleep_velocity_threshold_ang = value.sleep_velocity_threshold_angular;
			config.sleep_delay_s = value.sleep_delay_seconds;
			config.solver_iterations = value.solver_iterations;
			config.push_out_iterations = value.push_out_iterations;
			config.broadphase_aabb_margin = value.broadphase_aabb_margin;
			config.contact_sort_propagation_scale = value.contact_sort_propagation_scale;
			config.contact_sort_shock_iterations = value.contact_sort_shock_iterations;
			config.contact_sort_shock_alignment = value.contact_sort_shock_alignment;
			config.contact_sort_shock_min_strength = value.contact_sort_shock_min_strength;
			config.contact_sort_shock_decay = value.contact_sort_shock_decay;
			config.penetration_slop = value.penetration_slop;
			config.velocity_baumgarte = value.velocity_baumgarte;
			config.position_slop = value.position_slop;
			config.position_baumgarte = value.position_baumgarte;
			config.contact_slop_scale = value.contact_slop_scale;
			config.support_contact_slop_scale = value.support_contact_slop_scale;
			config.warm_start_scale = value.warm_start_scale;
			config.deep_penetration_threshold = value.deep_penetration_threshold;
			config.deep_penetration_range = value.deep_penetration_range;
			config.deep_penetration_baumgarte_min = value.deep_penetration_baumgarte_min;
			config.deep_penetration_baumgarte_max = value.deep_penetration_baumgarte_max;
			config.selective_refresh_passes = value.selective_refresh_passes;
			config.selective_refresh_max_pairs = value.selective_refresh_max_pairs;
			config.selective_refresh_body_limit = value.selective_refresh_body_limit;
			config.selective_refresh_contact_limit = value.selective_refresh_contact_limit;
			config.selective_refresh_solver_iterations = value.selective_refresh_solver_iterations;
			config.selective_refresh_position_iterations = value.selective_refresh_position_iterations;
			config.selective_refresh_bias_scale = value.selective_refresh_bias_scale;
			config.selective_refresh_restitution_scale = value.selective_refresh_restitution_scale;
			config.selective_refresh_adaptive_body_limit = value.selective_refresh_adaptive_body_limit;
			config.selective_refresh_adaptive_solver_iterations = value.selective_refresh_adaptive_solver_iterations;
			config.selective_refresh_support_only = value.selective_refresh_support_only != 0;
			config.selective_refresh_resolve_support_only = value.selective_refresh_resolve_support_only != 0;
			config.selective_refresh_depth_slop = value.selective_refresh_depth_slop;
			config.selective_refresh_support_depth_slop = value.selective_refresh_support_depth_slop;
			config.selective_refresh_closing_speed_slop = value.selective_refresh_closing_speed_slop;
			config.selective_refresh_support_alignment = value.selective_refresh_support_alignment;
			config.selective_refresh_aabb_margin = value.selective_refresh_aabb_margin;
			return config;
		}

		void FromNative(PhysicsEngineConfig& value, EngineConfig const& config)
		{
			value = {};
			value.header = {sizeof(value), PHYSICS_STRUCT_VERSION};
			value.max_collision_pairs = config.max_collision_pairs;
			value.sleeping_enabled = config.sleeping_enabled;
			value.sleep_velocity_threshold_linear = config.sleep_velocity_threshold_lin;
			value.sleep_velocity_threshold_angular = config.sleep_velocity_threshold_ang;
			value.sleep_delay_seconds = config.sleep_delay_s;
			value.solver_iterations = config.solver_iterations;
			value.push_out_iterations = config.push_out_iterations;
			value.broadphase_aabb_margin = config.broadphase_aabb_margin;
			value.contact_sort_propagation_scale = config.contact_sort_propagation_scale;
			value.contact_sort_shock_iterations = config.contact_sort_shock_iterations;
			value.contact_sort_shock_alignment = config.contact_sort_shock_alignment;
			value.contact_sort_shock_min_strength = config.contact_sort_shock_min_strength;
			value.contact_sort_shock_decay = config.contact_sort_shock_decay;
			value.penetration_slop = config.penetration_slop;
			value.velocity_baumgarte = config.velocity_baumgarte;
			value.position_slop = config.position_slop;
			value.position_baumgarte = config.position_baumgarte;
			value.contact_slop_scale = config.contact_slop_scale;
			value.support_contact_slop_scale = config.support_contact_slop_scale;
			value.warm_start_scale = config.warm_start_scale;
			value.deep_penetration_threshold = config.deep_penetration_threshold;
			value.deep_penetration_range = config.deep_penetration_range;
			value.deep_penetration_baumgarte_min = config.deep_penetration_baumgarte_min;
			value.deep_penetration_baumgarte_max = config.deep_penetration_baumgarte_max;
			value.selective_refresh_passes = config.selective_refresh_passes;
			value.selective_refresh_max_pairs = config.selective_refresh_max_pairs;
			value.selective_refresh_body_limit = config.selective_refresh_body_limit;
			value.selective_refresh_contact_limit = config.selective_refresh_contact_limit;
			value.selective_refresh_solver_iterations = config.selective_refresh_solver_iterations;
			value.selective_refresh_position_iterations = config.selective_refresh_position_iterations;
			value.selective_refresh_bias_scale = config.selective_refresh_bias_scale;
			value.selective_refresh_restitution_scale = config.selective_refresh_restitution_scale;
			value.selective_refresh_adaptive_body_limit = config.selective_refresh_adaptive_body_limit;
			value.selective_refresh_adaptive_solver_iterations = config.selective_refresh_adaptive_solver_iterations;
			value.selective_refresh_support_only = config.selective_refresh_support_only;
			value.selective_refresh_resolve_support_only = config.selective_refresh_resolve_support_only;
			value.selective_refresh_depth_slop = config.selective_refresh_depth_slop;
			value.selective_refresh_support_depth_slop = config.selective_refresh_support_depth_slop;
			value.selective_refresh_closing_speed_slop = config.selective_refresh_closing_speed_slop;
			value.selective_refresh_support_alignment = config.selective_refresh_support_alignment;
			value.selective_refresh_aabb_margin = config.selective_refresh_aabb_margin;
		}

		// Require a material slot that can be indexed safely by both CPU and GPU material arrays.
		void RequireMaterialId(std::int32_t material_id)
		{
			if (material_id < 0 || material_id >= Material::MaxMaterialId)
				throw ApiException(PhysicsStatus::InvalidArgument, "Material ID is out of range");
		}

		PhysicsShapeCommon const& RequireShapeCommon(PhysicsShapeCommon const* common)
		{
			auto const& value = RequireStruct(common);
			RequireMaterialId(value.material_id);
			return value;
		}

		template <typename T>
		T const& RequireShapeDesc(T const* value)
		{
			if (value == nullptr)
				throw ApiException(PhysicsStatus::InvalidArgument, "Required shape description pointer is null");
			if (value->common.header.version != PHYSICS_STRUCT_VERSION || value->common.header.size < sizeof(T))
				throw ApiException(PhysicsStatus::InvalidStruct, "Shape description size or version is incompatible");

			RequireMaterialId(value->common.material_id);
			return *value;
		}

		// Store a complete location-independent shape blob in a generation slot.
		PhysicsShapeHandle StoreShape(EngineRecord& engine, byte_data<16> data)
		{
			auto [slot, index] = AllocateSlot(engine.m_shapes);
			slot.m_object = std::make_unique<ShapeRecord>();
			slot.m_object->m_data = std::move(data);
			return MakeChildHandle(engine.m_cookie, index, slot.m_generation);
		}

		template <collision::ShapeType TShape>
		PhysicsShapeHandle StoreShape(EngineRecord& engine, TShape const& shape)
		{
			auto data = byte_data<16>{};
			data.push_back({byte_ptr(&shape), static_cast<std::size_t>(shape.m_base.m_size)});
			return StoreShape(engine, std::move(data));
		}

		// Return the number of convex leaves that 'shape' contributes to a collision model.
		// Compounds contribute the leaves of their whole subtree because nested arrays are flattened.
		int CountShapeLeaves(collision::Shape const& shape)
		{
			if (shape.m_type != collision::EShape::Array)
				return 1;

			auto count = 0;
			auto const& arr = collision::shape_cast<collision::ShapeArray>(shape);
			for (auto const* child = arr.begin(), *end = arr.end(); child != end; child = collision::next(child))
				count += CountShapeLeaves(*child);

			return count;
		}

		// Compose 'p2r' through a copied shape subtree so it reads correctly under a new parent.
		// A shape-to-root transform is absolute within its owning root rather than relative to its immediate
		// parent, so every node in the subtree has to be rebased, not just the node being attached.
		void RebaseShapeTree(collision::Shape& shape, m4x4 const& p2r)
		{
			shape.m_s2r = p2r * shape.m_s2r;
			if (shape.m_type != collision::EShape::Array)
				return;

			auto& arr = collision::shape_cast<collision::ShapeArray>(shape);
			for (auto* child = arr.begin(), *end = arr.end(); child != end; child = collision::next(child))
				RebaseShapeTree(*child, p2r);
		}

		PhysicsBodyFlags BodyFlags(BodyRecord const& record)
		{
			auto flags = PhysicsBodyFlags::None;
			if (record.m_enabled) flags = static_cast<PhysicsBodyFlags>(static_cast<std::uint32_t>(flags) | static_cast<std::uint32_t>(PhysicsBodyFlags::Enabled));
			if (record.m_body->Sleeping()) flags = static_cast<PhysicsBodyFlags>(static_cast<std::uint32_t>(flags) | static_cast<std::uint32_t>(PhysicsBodyFlags::Sleeping));
			if (record.m_body->NeverSleep()) flags = static_cast<PhysicsBodyFlags>(static_cast<std::uint32_t>(flags) | static_cast<std::uint32_t>(PhysicsBodyFlags::NeverSleep));
			return flags;
		}
		bool HasFlag(PhysicsBodyFlags flags, PhysicsBodyFlags bit)
		{
			return (static_cast<std::uint32_t>(flags) & static_cast<std::uint32_t>(bit)) != 0;
		}

		// Fill a complete body state without exposing the native body's address.
		void FillBodyState(PhysicsBodyState& state, PhysicsBodyHandle handle, BodyRecord const& record)
		{
			state = {};
			state.header = {sizeof(state), PHYSICS_STRUCT_VERSION};
			state.body = handle;
			state.shape = record.m_shape;
			CopyLayout(state.object_to_world, record.m_body->O2W());
			CopyLayout(state.inertia, record.m_body->InertiaOS());
			CopyLayout(state.momentum, record.m_body->MomentumWS());
			CopyLayout(state.velocity, record.m_body->VelocityWS());
			CopyLayout(state.force, record.m_body->ForceWS());
			CopyLayout(state.gravity, record.m_body->GravityWS());
			state.user_tag = record.m_user_tag;
			state.motion_type = record.m_motion_type;
			state.flags = BodyFlags(record);
		}

		void ApplyBodyState(BodyRecord& record, PhysicsBodyState const& state)
		{
			record.m_body->O2W(ToNative(state.object_to_world));
			record.m_body->SetMassProperties(ToNative(state.inertia), ToNative(state.inertia.centre_of_mass_and_mass).w0());
			record.m_body->MomentumWS(ToForce(state.momentum));
			record.m_body->ForceWS(ToForce(state.force));
			record.m_body->GravityWS(ToNative(state.gravity));
			record.m_body->NeverSleep(HasFlag(state.flags, PhysicsBodyFlags::NeverSleep));
			record.m_body->Sleeping(HasFlag(state.flags, PhysicsBodyFlags::Sleeping));
			record.m_enabled = HasFlag(state.flags, PhysicsBodyFlags::Enabled);
			record.m_user_tag = state.user_tag;
			record.m_motion_type = state.motion_type;
		}

		// Validate every command before applying any command so a failed batch has no partial effect.
		void ValidateCommands(EngineRecord& engine, PhysicsBodyCommand const* commands, std::uint32_t count)
		{
			if (count != 0 && commands == nullptr)
				throw ApiException(PhysicsStatus::InvalidArgument, "Command buffer is null");

			for (auto i = std::uint32_t{}; i != count; ++i)
			{
				RequireStruct(&commands[i]);
				RequireBody(engine, commands[i].body);
				switch (commands[i].type)
				{
					case PhysicsBodyCommandType::SetTransform:
					case PhysicsBodyCommandType::SetVelocity:
					case PhysicsBodyCommandType::SetMomentum:
					case PhysicsBodyCommandType::SetForce:
					case PhysicsBodyCommandType::ApplyImpulse:
					case PhysicsBodyCommandType::SetGravity:
					case PhysicsBodyCommandType::SetKinematicTransform:
					case PhysicsBodyCommandType::SetEnabled:
					case PhysicsBodyCommandType::Wake:
					case PhysicsBodyCommandType::Sleep:
						break;
					case PhysicsBodyCommandType::ApplyForce:
					{
						if (commands[i].at.w != 0.0f)
							throw ApiException(PhysicsStatus::InvalidArgument, "Force application point must be an offset from the model origin");

						break;
					}
					default:
						throw ApiException(PhysicsStatus::InvalidArgument, "Unknown body command type");
				}
			}
		}

		void ApplyCommands(EngineRecord& engine, PhysicsBodyCommand const* commands, std::uint32_t count)
		{
			ValidateCommands(engine, commands, count);
			for (auto i = std::uint32_t{}; i != count; ++i)
			{
				auto const& command = commands[i];
				auto& record = RequireBody(engine, command.body);
				auto& body = *record.m_body;
				switch (command.type)
				{
					case PhysicsBodyCommandType::SetTransform:
					{
						body.O2W(ToNative(command.transform));
						break;
					}
					case PhysicsBodyCommandType::SetVelocity:
					{
						body.VelocityWS(ToMotion(command.value));
						break;
					}
					case PhysicsBodyCommandType::SetMomentum:
					{
						body.MomentumWS(ToForce(command.value));
						break;
					}
					case PhysicsBodyCommandType::SetForce:
					{
						body.ZeroForces();
						body.ApplyForceWS(ToForce(command.value));
						break;
					}
					case PhysicsBodyCommandType::ApplyForce:
					{
						body.ApplyForceWS(ToNative(command.value.linear), ToNative(command.value.angular), ToNative(command.at));
						break;
					}
					case PhysicsBodyCommandType::ApplyImpulse:
					{
						auto impulse = ToForce(command.value);
						body.MomentumWS(body.MomentumWS() + impulse);
						break;
					}
					case PhysicsBodyCommandType::SetGravity:
					{
						body.GravityWS(ToNative(command.value.linear));
						break;
					}
					case PhysicsBodyCommandType::SetKinematicTransform:
					{
						if (record.m_motion_type != PhysicsMotionType::Kinematic)
							throw ApiException(PhysicsStatus::InvalidArgument, "Kinematic transform command requires a kinematic body");

						body.O2W(ToNative(command.transform));
						break;
					}
					case PhysicsBodyCommandType::SetEnabled:
					{
						record.m_enabled = command.flags != 0;
						break;
					}
					case PhysicsBodyCommandType::Wake:
					{
						body.Wake();
						break;
					}
					case PhysicsBodyCommandType::Sleep:
					{
						body.Sleep();
						break;
					}
					default:
					{
						throw ApiException(PhysicsStatus::InvalidArgument, "Unknown body command type");
					}
				}
			}
		}

		void BeginStep(EngineRecord& engine, float elapsed_seconds, double absolute_time_seconds, PhysicsBodyCommand const* commands, std::uint32_t command_count)
		{
			RequireOwner(engine);
			RequireIdle(engine);
			if (!(elapsed_seconds > 0.0f) || !std::isfinite(elapsed_seconds))
				throw ApiException(PhysicsStatus::InvalidArgument, "Step duration must be finite and positive");

			ApplyCommands(engine, commands, command_count);

			// Stable slot ordering makes replay checksums and event ordering deterministic.
			engine.m_step_bodies.clear();
			engine.m_events.clear();
			for (auto& slot : engine.m_bodies)
			{
				if (!slot.m_object || !slot.m_object->m_enabled)
					continue;

				slot.m_object->m_sleep_before_step = slot.m_object->m_body->Sleeping();
				engine.m_step_bodies.push_back(slot.m_object->m_body.get());
			}

			engine.m_engine->BeginStep(elapsed_seconds, engine.m_step_bodies, absolute_time_seconds);
			++engine.m_submitted_step;
		}

		void CompleteStep(EngineRecord& engine)
		{
			RequireOwner(engine);
			if (engine.m_submitted_step == engine.m_completed_step)
				throw ApiException(PhysicsStatus::NoStepPending, "CompleteStep called without a pending step");

			engine.m_engine->CompleteStep();

			// Lifecycle events are derived only after native readback has produced the completed state.
			for (auto i = std::size_t{}; i != engine.m_bodies.size(); ++i)
			{
				auto& slot = engine.m_bodies[i];
				if (!slot.m_object || !slot.m_object->m_enabled)
					continue;

				auto sleeping = slot.m_object->m_body->Sleeping();
				if (sleeping == slot.m_object->m_sleep_before_step)
					continue;

				engine.m_events.push_back(PhysicsEvent{
					.header = {sizeof(PhysicsEvent), PHYSICS_STRUCT_VERSION},
					.type = sleeping ? PhysicsEventType::Sleep : PhysicsEventType::Wake,
					.point_count = 0,
					.body_a = MakeChildHandle(engine.m_cookie, i, slot.m_generation),
					.child_a = PHYSICS_NO_CHILD,
					.child_b = PHYSICS_NO_CHILD,
				});
			}
			engine.m_completed_step = engine.m_submitted_step;
		}

		// Clear terminal GPU work before destroying children whose addresses remain in the pending-step body list.
		void PrepareChildDestroy(EngineRecord& engine)
		{
			if (engine.m_submitted_step == engine.m_completed_step)
				return;

			auto reason = engine.m_engine->Device()->GetDeviceRemovedReason();
			if (!FAILED(reason))
				throw ApiException(PhysicsStatus::StepPending, "The engine has a pending simulation step");

			engine.m_engine->AbandonStep();
			engine.m_completed_step = engine.m_submitted_step;
		}

		// Bring the engine to an idle state so that complete, self-consistent results can be serialised.
		// A pending step is drained rather than refused, because the caller has no way to describe a
		// half-submitted step and would otherwise have to complete the step itself just to save state.
		void DrainPendingStep(EngineRecord& engine)
		{
			RequireOwner(engine);
			if (engine.m_submitted_step == engine.m_completed_step)
				return;

			CompleteStep(engine);
		}

		PhysicsBodySnapshot MakeSnapshot(PhysicsBodyHandle handle, BodyRecord const& record)
		{
			auto snapshot = PhysicsBodySnapshot{};
			snapshot.header = {sizeof(snapshot), PHYSICS_STRUCT_VERSION};
			snapshot.body = handle;
			snapshot.shape = record.m_shape;
			CopyLayout(snapshot.object_to_world, record.m_body->O2W());
			CopyLayout(snapshot.momentum, record.m_body->MomentumWS());
			CopyLayout(snapshot.velocity, record.m_body->VelocityWS());
			snapshot.user_tag = record.m_user_tag;
			snapshot.motion_type = record.m_motion_type;
			snapshot.flags = BodyFlags(record);
			return snapshot;
		}

		std::uint64_t HashBytes(std::uint64_t hash, void const* data, std::size_t size)
		{
			auto bytes = static_cast<std::uint8_t const*>(data);
			for (auto i = std::size_t{}; i != size; ++i)
			{
				hash ^= bytes[i];
				hash *= 1099511628211ULL;
			}
			return hash;
		}

		std::uint64_t StateChecksum(EngineRecord const& engine)
		{
			auto hash = 14695981039346656037ULL;
			for (auto i = std::size_t{}; i != engine.m_bodies.size(); ++i)
			{
				auto const& slot = engine.m_bodies[i];
				if (!slot.m_object || !slot.m_object->m_enabled)
					continue;

				auto snapshot = MakeSnapshot(MakeChildHandle(engine.m_cookie, i, slot.m_generation), *slot.m_object);
				hash = HashBytes(hash, &snapshot.body, sizeof(snapshot) - offsetof(PhysicsBodySnapshot, body));
			}
			return hash;
		}

		inline constexpr std::uint64_t CheckpointMagic = 0x3154504B43595052ULL; // "RYPCKPT1"

		struct CheckpointHeader
		{
			std::uint64_t m_magic;
			std::uint32_t m_version;
			std::uint32_t m_header_size;
			std::uint64_t m_total_size;
			std::uint64_t m_payload_checksum;
			std::uint16_t m_engine_cookie;
			std::uint16_t m_reserved0;
			std::uint32_t m_shape_count;
			std::uint32_t m_body_count;
			std::uint32_t m_reserved1;
			std::uint64_t m_submitted_step;
			std::uint64_t m_completed_step;
			PhysicsEngineConfig m_config;
		};

		struct CheckpointShape
		{
			PhysicsShapeHandle m_handle;
			std::uint32_t m_size;
			std::uint32_t m_child_count;
		};

		struct CheckpointBody
		{
			PhysicsBodyState m_state;
		};

		std::uint64_t CheckpointRequiredSize(EngineRecord const& engine)
		{
			auto size = std::uint64_t{sizeof(CheckpointHeader)} +
				std::uint64_t{Material::MaxMaterialId} * sizeof(PhysicsMaterial);
			for (auto const& slot : engine.m_shapes)
			{
				if (slot.m_object)
					size += sizeof(CheckpointShape) +
						slot.m_object->m_data.size() +
						std::uint64_t{slot.m_object->m_children.size()} * sizeof(PhysicsShapeHandle);
			}
			for (auto const& slot : engine.m_bodies)
			{
				if (slot.m_object)
					size += sizeof(CheckpointBody);
			}
			return size;
		}

		template <typename T>
		void WriteCheckpointValue(std::byte*& output, T const& value)
		{
			memcpy(output, &value, sizeof(value));
			output += sizeof(value);
		}

		template <typename T>
		T ReadCheckpointValue(std::byte const*& input, std::byte const* end)
		{
			if (static_cast<std::size_t>(end - input) < sizeof(T))
				throw ApiException(PhysicsStatus::InvalidArgument, "Checkpoint is truncated");

			auto value = T{};
			memcpy(&value, input, sizeof(value));
			input += sizeof(value);
			return value;
		}

		// Validate a location-independent shape blob before adopting it into the live registry.
		void ValidateCheckpointShape(std::span<std::byte const> data)
		{
			if (data.size() < sizeof(collision::Shape) || (data.size() & 0xFU) != 0)
				throw ApiException(PhysicsStatus::InvalidArgument, "Checkpoint contains an invalid shape size");

			// Validate every nested record before allowing collision code to walk the location-independent blob.
			auto validate = [&](auto const& self, std::span<std::byte const> bytes) -> std::size_t
			{
				if (bytes.size() < sizeof(collision::Shape))
					throw ApiException(PhysicsStatus::InvalidArgument, "Checkpoint shape is truncated");

				auto const& shape = *reinterpret_cast<collision::Shape const*>(bytes.data());
				if (shape.m_size < static_cast<int>(sizeof(collision::Shape)) ||
					(shape.m_size & 0xFU) != 0 ||
					static_cast<std::size_t>(shape.m_size) > bytes.size())
					throw ApiException(PhysicsStatus::InvalidArgument, "Checkpoint contains an invalid nested shape size");
				if (shape.m_type < collision::EShape::Sphere || shape.m_type >= collision::EShape::NumberOf)
					throw ApiException(PhysicsStatus::InvalidArgument, "Checkpoint contains an unknown shape type");
				RequireMaterialId(shape.m_material_id);

				auto minimum_size = std::size_t{};
				switch (shape.m_type)
				{
					case collision::EShape::Sphere: { minimum_size = sizeof(collision::ShapeSphere); break; }
					case collision::EShape::Box: { minimum_size = sizeof(collision::ShapeBox); break; }
					case collision::EShape::Line: { minimum_size = sizeof(collision::ShapeLine); break; }
					case collision::EShape::Triangle: { minimum_size = sizeof(collision::ShapeTriangle); break; }
					case collision::EShape::Polytope: { minimum_size = sizeof(collision::ShapePolytope); break; }
					case collision::EShape::Array: { minimum_size = sizeof(collision::ShapeArray); break; }
					default: { throw ApiException(PhysicsStatus::InvalidArgument, "Checkpoint contains an unsupported shape type"); }
				}
				if (static_cast<std::size_t>(shape.m_size) < minimum_size)
					throw ApiException(PhysicsStatus::InvalidArgument, "Checkpoint shape record is smaller than its type");

				if (shape.m_type == collision::EShape::Array)
				{
					if (shape.m_size < static_cast<int>(sizeof(collision::ShapeArray)))
						throw ApiException(PhysicsStatus::InvalidArgument, "Checkpoint compound header is truncated");

					auto const& array = reinterpret_cast<collision::ShapeArray const&>(shape);
					if (array.m_num_shapes < 0)
						throw ApiException(PhysicsStatus::InvalidArgument, "Checkpoint compound child count is invalid");

					auto offset = sizeof(collision::ShapeArray);
					for (auto i = 0; i != array.m_num_shapes; ++i)
					{
						if (offset >= static_cast<std::size_t>(shape.m_size))
							throw ApiException(PhysicsStatus::InvalidArgument, "Checkpoint compound child data is truncated");

						offset += self(self, bytes.subspan(offset, static_cast<std::size_t>(shape.m_size) - offset));
					}
					if (offset != static_cast<std::size_t>(shape.m_size))
						throw ApiException(PhysicsStatus::InvalidArgument, "Checkpoint compound size does not match its children");
				}

				return static_cast<std::size_t>(shape.m_size);
			};

			if (validate(validate, data) != data.size())
				throw ApiException(PhysicsStatus::InvalidArgument, "Checkpoint shape size does not match its blob");
		}
	InteropState::InteropState()
		: m_engines()
		, m_next_cookie(1)
	{}
	InteropState::~InteropState() = default;
	bool InteropState::HasLiveEngines() const
	{
		return std::ranges::any_of(m_engines, [](auto const& slot)
		{
			return slot->m_record != nullptr;
		});
	}
}

using namespace pr::physics;

extern "C"
{
	std::uint32_t __stdcall Physics_ApiVersion()
	{
		return PHYSICS_API_VERSION;
	}

	PhysicsStatus __stdcall Physics_StructSize(PhysicsStructId struct_id, std::uint32_t* size)
	{
		return pr::physics::ApiCall([&]
		{
			if (size == nullptr)
				throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Size output pointer is null");

			switch (struct_id)
			{
				case PhysicsStructId::EngineConfig: *size = sizeof(PhysicsEngineConfig); break;
				case PhysicsStructId::ShapeCommon: *size = sizeof(PhysicsShapeCommon); break;
				case PhysicsStructId::SphereShape: *size = sizeof(PhysicsSphereShape); break;
				case PhysicsStructId::BoxShape: *size = sizeof(PhysicsBoxShape); break;
				case PhysicsStructId::LineShape: *size = sizeof(PhysicsLineShape); break;
				case PhysicsStructId::TriangleShape: *size = sizeof(PhysicsTriangleShape); break;
				case PhysicsStructId::BodyDesc: *size = sizeof(PhysicsBodyDesc); break;
				case PhysicsStructId::BodyState: *size = sizeof(PhysicsBodyState); break;
				case PhysicsStructId::BodyCommand: *size = sizeof(PhysicsBodyCommand); break;
				case PhysicsStructId::BodySnapshot: *size = sizeof(PhysicsBodySnapshot); break;
				case PhysicsStructId::Event: *size = sizeof(PhysicsEvent); break;
				case PhysicsStructId::Diagnostics: *size = sizeof(PhysicsDiagnostics); break;
				case PhysicsStructId::Material: *size = sizeof(PhysicsMaterial); break;
				default: throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Unknown physics structure identifier");
			}
		});
	}

	PhysicsStatus __stdcall Physics_LastError(char* buffer, std::uint32_t capacity, std::uint32_t* required)
	{
		try
		{
			if (required == nullptr)
				return PhysicsStatus::InvalidArgument;

			auto size = static_cast<std::uint32_t>(pr::physics::g_last_error.size() + 1);
			*required = size;
			if (buffer == nullptr || capacity < size)
				return PhysicsStatus::BufferTooSmall;

			memcpy(buffer, pr::physics::g_last_error.c_str(), size);
			return PhysicsStatus::Success;
		}
		catch (...)
		{
			return PhysicsStatus::InternalError;
		}
	}

	PhysicsStatus __stdcall Physics_EngineCreate(pr::physics::DllHandle context, PhysicsEngineConfig const* config, void* external_d3d12_device, PhysicsEngineHandle* engine)
	{
		return pr::physics::ApiCall([&]
		{
			auto dll = pr::physics::PinDll();
			pr::physics::LockGuard lock(dll->m_mutex);
			if (!dll->m_inits.contains(context))
				throw pr::physics::ApiException(PhysicsStatus::InvalidHandle, "DLL context handle is invalid");
			if (engine == nullptr)
				throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Engine output pointer is null");

			auto native_config = config != nullptr
				? pr::physics::ToNative(pr::physics::RequireStruct(config))
				: pr::physics::EngineConfig{};

			auto& state = *dll->m_interop;
			auto index = std::size_t{};
			for (; index != state.m_engines.size(); ++index)
			{
				if (!state.m_engines[index]->m_record)
					break;
			}
			if (index == state.m_engines.size())
				state.m_engines.push_back(std::make_unique<pr::physics::EngineSlot>());

			auto cookie = static_cast<std::uint16_t>(state.m_next_cookie++);
			if (cookie == 0)
				cookie = static_cast<std::uint16_t>(state.m_next_cookie++);

			auto& slot = *state.m_engines[index];
			slot.m_record = std::make_unique<pr::physics::EngineRecord>(cookie, native_config, static_cast<ID3D12Device4*>(external_d3d12_device));
			*engine = pr::physics::MakeEngineHandle(index, slot.m_generation);
		});
	}

	PhysicsStatus __stdcall Physics_EngineDestroy(PhysicsEngineHandle engine)
	{
		return pr::physics::ApiCall([&]
		{
			auto scope = pr::physics::EngineScope(engine);
			auto& record = *scope;
			pr::physics::RequireOwner(record);

			// Release pending CPU references without resetting command state after device removal; otherwise finish the step normally.
			auto failure = std::exception_ptr{};
			try
			{
				if (record.m_submitted_step != record.m_completed_step)
				{
					if (FAILED(record.m_engine->Device()->GetDeviceRemovedReason()))
					{
						record.m_engine->AbandonStep();
						record.m_completed_step = record.m_submitted_step;
					}
					else
					{
						pr::physics::CompleteStep(record);
					}
				}
			}
			catch (...)
			{
				failure = std::current_exception();
			}

			// Retire under the engine lock so a caller that pinned this record concurrently observes a stale
			// handle. The record itself is released when the scope unwinds, after the engine lock is dropped.
			record.m_retired = true;
			pr::physics::RetireEngineSlot(engine);

			if (failure)
				std::rethrow_exception(failure);
		});
	}

	void __stdcall Physics_EngineAbandon(PhysicsEngineHandle engine)
	{
		// Finalizer-only cleanup bypasses the public owner-thread rule while still draining pending GPU work.
		try
		{
			auto dll = pr::physics::TryPinDll();
			if (!dll)
				return;

			// Pin the record under the registry lock, then drain with only the engine lock held so an
			// abandoned engine's GPU wait does not stall other engines.
			std::shared_ptr<pr::physics::EngineRecord> record;
			{
				pr::physics::LockGuard lock(dll->m_mutex);
				auto index = pr::physics::EngineIndex(engine);
				if (index >= dll->m_interop->m_engines.size())
					return;

				auto& slot = *dll->m_interop->m_engines[index];
				if (!slot.m_record || slot.m_generation != pr::physics::EngineGeneration(engine))
					return;

				record = slot.m_record;
			}

			std::unique_lock<std::recursive_mutex> lock(record->m_lock);
			if (record->m_retired)
				return;

			if (record->m_submitted_step != record->m_completed_step)
			{
				// Finalizer cleanup can run on any thread, so it waits without resetting thread-affine
				// command recording state or unpacking results into managed-owned bodies.
				try
				{
					record->m_engine->AbandonStep();
					record->m_completed_step = record->m_submitted_step;
				}
				catch (...)
				{}
			}

			record->m_retired = true;
			pr::physics::RetireEngineSlot(engine);
		}
		catch (...)
		{}
	}

	PhysicsStatus __stdcall Physics_EngineDeviceLeaseAcquire(PhysicsEngineHandle engine, void** d3d12_device)
	{
		return pr::physics::ApiCall([&]
		{
			auto scope = pr::physics::EngineScope(engine);
			auto& record = *scope;
			if (d3d12_device == nullptr)
				throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Device output pointer is null");

			auto device = record.m_engine->Device();
			device->AddRef();
			*d3d12_device = device;
		});
	}

	PhysicsStatus __stdcall Physics_EngineConfigGet(PhysicsEngineHandle engine, PhysicsEngineConfig* config)
	{
		return pr::physics::ApiCall([&]
		{
			auto scope = pr::physics::EngineScope(engine);
			auto& record = *scope;
			if (config == nullptr)
				throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Configuration output pointer is null");

			pr::physics::FromNative(*config, record.m_engine->Config());
		});
	}

	PhysicsStatus __stdcall Physics_EngineConfigSet(PhysicsEngineHandle engine, PhysicsEngineConfig const* config)
	{
		return pr::physics::ApiCall([&]
		{
			auto scope = pr::physics::EngineScope(engine);
			auto& record = *scope;
			pr::physics::RequireOwner(record);
			pr::physics::RequireIdle(record);
			record.m_engine->Config(pr::physics::ToNative(pr::physics::RequireStruct(config)));
		});
	}

	PhysicsStatus __stdcall Physics_MaterialGet(PhysicsEngineHandle engine, std::int32_t material_id, PhysicsMaterial* material)
	{
		return pr::physics::ApiCall([&]
		{
			auto scope = pr::physics::EngineScope(engine);
			auto& record = *scope;
			if (material == nullptr)
				throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Material output pointer is null");

			pr::physics::RequireMaterialId(material_id);
			auto native = record.m_engine->Material(material_id);
			*material = PhysicsMaterial{
				.header = {sizeof(PhysicsMaterial), PHYSICS_STRUCT_VERSION},
				.id = native.m_id,
				.static_friction = native.m_friction_static,
				.normal_elasticity = native.m_elasticity_norm,
				.tangential_elasticity = native.m_elasticity_tang,
				.torsional_elasticity = native.m_elasticity_tors,
				.density = native.m_density,
			};
		});
	}

	PhysicsStatus __stdcall Physics_MaterialSet(PhysicsEngineHandle engine, PhysicsMaterial const* material)
	{
		return pr::physics::ApiCall([&]
		{
			auto scope = pr::physics::EngineScope(engine);
			auto& record = *scope;
			pr::physics::RequireOwner(record);
			pr::physics::RequireIdle(record);
			auto const& value = pr::physics::RequireStruct(material);
			pr::physics::RequireMaterialId(value.id);
			record.m_engine->Material(pr::physics::Material{
				value.id,
				value.static_friction,
				value.normal_elasticity,
				value.tangential_elasticity,
				value.torsional_elasticity,
				value.density});
		});
	}

	PhysicsStatus __stdcall Physics_ShapeCreateSphere(PhysicsEngineHandle engine, PhysicsSphereShape const* desc, PhysicsShapeHandle* shape)
	{
		return pr::physics::ApiCall([&]
		{
			auto scope = pr::physics::EngineScope(engine);
			auto& record = *scope;
			pr::physics::RequireOwner(record);
			pr::physics::RequireIdle(record);
			auto const& value = pr::physics::RequireShapeDesc(desc);
			if (shape == nullptr || !(value.radius > 0.0f))
				throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Sphere output pointer or radius is invalid");

			auto native = pr::collision::ShapeSphere{
				value.radius,
				pr::physics::ToNative(value.common.shape_to_root),
				value.hollow != 0,
				value.common.material_id,
				static_cast<pr::collision::Shape::EFlags>(value.common.flags)};
			*shape = pr::physics::StoreShape(record, native);
		});
	}

	PhysicsStatus __stdcall Physics_ShapeCreateBox(PhysicsEngineHandle engine, PhysicsBoxShape const* desc, PhysicsShapeHandle* shape)
	{
		return pr::physics::ApiCall([&]
		{
			auto scope = pr::physics::EngineScope(engine);
			auto& record = *scope;
			pr::physics::RequireOwner(record);
			pr::physics::RequireIdle(record);
			auto const& value = pr::physics::RequireShapeDesc(desc);
			if (shape == nullptr || value.dimensions.x <= 0 || value.dimensions.y <= 0 || value.dimensions.z <= 0)
				throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Box output pointer or dimensions are invalid");

			auto dimensions = pr::physics::ToNative(value.dimensions).w0();
			auto native = pr::collision::ShapeBox{
				dimensions,
				pr::physics::ToNative(value.common.shape_to_root),
				value.common.material_id,
				static_cast<pr::collision::Shape::EFlags>(value.common.flags)};
			*shape = pr::physics::StoreShape(record, native);
		});
	}

	PhysicsStatus __stdcall Physics_ShapeCreateLine(PhysicsEngineHandle engine, PhysicsLineShape const* desc, PhysicsShapeHandle* shape)
	{
		return pr::physics::ApiCall([&]
		{
			auto scope = pr::physics::EngineScope(engine);
			auto& record = *scope;
			pr::physics::RequireOwner(record);
			pr::physics::RequireIdle(record);
			auto const& value = pr::physics::RequireShapeDesc(desc);
			if (shape == nullptr || !(value.length > 0.0f) || value.radius < 0.0f)
				throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Line output pointer, length, or radius is invalid");

			auto native = pr::collision::ShapeLine{
				value.length,
				value.radius,
				pr::physics::ToNative(value.common.shape_to_root),
				value.common.material_id,
				static_cast<pr::collision::Shape::EFlags>(value.common.flags)};
			*shape = pr::physics::StoreShape(record, native);
		});
	}

	PhysicsStatus __stdcall Physics_ShapeCreateTriangle(PhysicsEngineHandle engine, PhysicsTriangleShape const* desc, PhysicsShapeHandle* shape)
	{
		return pr::physics::ApiCall([&]
		{
			auto scope = pr::physics::EngineScope(engine);
			auto& record = *scope;
			pr::physics::RequireOwner(record);
			pr::physics::RequireIdle(record);
			auto const& value = pr::physics::RequireShapeDesc(desc);
			if (shape == nullptr)
				throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Triangle output pointer is null");

			auto native = pr::collision::ShapeTriangle{
				pr::physics::ToNative(value.a).w1(),
				pr::physics::ToNative(value.b).w1(),
				pr::physics::ToNative(value.c).w1(),
				pr::physics::ToNative(value.common.shape_to_root),
				value.common.material_id,
				static_cast<pr::collision::Shape::EFlags>(value.common.flags)};
			*shape = pr::physics::StoreShape(record, native);
		});
	}

	PhysicsStatus __stdcall Physics_ShapeCreatePolytope(PhysicsEngineHandle engine, PhysicsShapeCommon const* common, PhysicsVector4 const* points, std::uint32_t point_count, PhysicsShapeHandle* shape)
	{
		return pr::physics::ApiCall([&]
		{
			auto scope = pr::physics::EngineScope(engine);
			auto& record = *scope;
			pr::physics::RequireOwner(record);
			pr::physics::RequireIdle(record);
			auto const& value = pr::physics::RequireShapeCommon(common);
			if (shape == nullptr || points == nullptr || point_count < 4)
				throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "A polytope requires an output pointer and at least four points");

			auto vertices = std::vector<pr::v4>{};
			vertices.reserve(point_count);
			for (auto i = std::uint32_t{}; i != point_count; ++i)
				vertices.push_back(pr::physics::ToNative(points[i]).w1());

			auto data = pr::collision::BuildPolytopeFromPoints(
				vertices,
				pr::physics::ToNative(value.shape_to_root),
				value.material_id,
				static_cast<pr::collision::Shape::EFlags>(value.flags));
			*shape = pr::physics::StoreShape(record, std::move(data));
		});
	}

	PhysicsStatus __stdcall Physics_ShapeCreateCompound(PhysicsEngineHandle engine, PhysicsShapeCommon const* common, PhysicsShapeHandle const* children, std::uint32_t child_count, PhysicsShapeHandle* shape)
	{
		return pr::physics::ApiCall([&]
		{
			auto scope = pr::physics::EngineScope(engine);
			auto& record = *scope;
			pr::physics::RequireOwner(record);
			pr::physics::RequireIdle(record);
			auto const& value = pr::physics::RequireShapeCommon(common);
			if (shape == nullptr || children == nullptr || child_count == 0)
				throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "A compound requires an output pointer and at least one child shape");

			// Resolve and measure every child before touching the registry so that a rejected handle leaves
			// no partially built shape behind. Cross-engine and stale handles are rejected by the shared
			// child-handle validation, and a repeated handle is a legitimate request for a repeated child.
			auto child_records = std::vector<pr::physics::ShapeRecord*>{};
			child_records.reserve(child_count);
			auto leaf_count = 0;
			for (auto i = std::uint32_t{}; i != child_count; ++i)
			{
				auto* child = &pr::physics::RequireChild(record, children[i], record.m_shapes).first;
				leaf_count += pr::physics::CountShapeLeaves(*child->Shape());
				child_records.push_back(child);
			}

			// The GPU packer identifies a child proxy by a bounded index, so refuse an oversized compound at
			// creation rather than letting it fail later when a body first adopts it.
			if (leaf_count > pr::physics::MaxCompoundChildren)
				throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, std::format("A compound shape may contain at most {} convex children, but this one contains {}", pr::physics::MaxCompoundChildren, leaf_count));

			// Copy each child into one location-independent blob. Declaration order is preserved because it is
			// the child identity reported back through contacts, and each child keeps its own transform,
			// material, and flags; only the frame it is expressed in changes.
			auto p2r = pr::physics::ToNative(value.shape_to_root);
			auto data = pr::byte_data<16>{};
			data.push_back<pr::collision::ShapeArray>();
			for (auto* child : child_records)
			{
				auto offset = data.size();
				data.append(child->m_data);
				pr::physics::RebaseShapeTree(data.at_byte_ofs<pr::collision::Shape>(offset), p2r);
			}

			// 'Complete' derives the blob size and bounding box, so the root transform has to be in place first.
			auto& arr = data.at_byte_ofs<pr::collision::ShapeArray>(0);
			arr.m_base.m_s2r = p2r;
			arr.m_base.m_material_id = value.material_id;
			arr.m_base.m_flags = static_cast<pr::collision::Shape::EFlags>(value.flags);
			arr.Complete(child_count);

			// Packed shape data is cached by shape address, so drop the cache before a new record can reuse the
			// address of a shape that was destroyed earlier.
			record.m_engine->ResetCaches();

			auto handle = pr::physics::StoreShape(record, std::move(data));
			auto& compound = pr::physics::RequireShape(record, handle);
			compound.m_children.assign(children, children + child_count);
			for (auto* child : child_records)
				++child->m_compound_refs;

			*shape = handle;
		});
	}

	PhysicsStatus __stdcall Physics_ShapeDestroy(PhysicsEngineHandle engine, PhysicsShapeHandle shape)
	{
		return pr::physics::ApiCall([&]
		{
			auto scope = pr::physics::EngineScope(engine);
			auto& record = *scope;
			pr::physics::RequireOwner(record);
			pr::physics::PrepareChildDestroy(record);
			auto [shape_record, index] = pr::physics::RequireChild(record, shape, record.m_shapes);
			if (shape_record.m_body_refs != 0)
				throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Shape is still referenced by one or more rigid bodies");
			if (shape_record.m_compound_refs != 0)
				throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Shape is still referenced by one or more compound shapes");

			// Releasing a compound releases the children it retained. The retention is only a lifetime link;
			// the compound already owns a private copy of each child's collision data.
			for (auto child : shape_record.m_children)
				--pr::physics::RequireShape(record, child).m_compound_refs;

			record.m_engine->ResetCaches();
			auto& slot = record.m_shapes[index];
			slot.m_object.reset();
			pr::physics::AdvanceGeneration(slot.m_generation);
		});
	}

	PhysicsStatus __stdcall Physics_BodyCreate(PhysicsEngineHandle engine, PhysicsBodyDesc const* desc, PhysicsBodyHandle* body)
	{
		return pr::physics::ApiCall([&]
		{
			auto scope = pr::physics::EngineScope(engine);
			auto& record = *scope;
			pr::physics::RequireOwner(record);
			pr::physics::RequireIdle(record);
			auto const& value = pr::physics::RequireStruct(desc);
			if (body == nullptr)
				throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Body output pointer is null");

			auto& shape_record = pr::physics::RequireShape(record, value.shape);
			auto body_record = std::make_unique<pr::physics::BodyRecord>();
			body_record->m_body = std::make_unique<pr::physics::RigidBody>(
				shape_record.Shape(),
				pr::physics::ToNative(value.object_to_world),
				value.motion_type == PhysicsMotionType::Dynamic && value.mass_mode == PhysicsMassMode::ExplicitInertia
					? pr::physics::ToNative(value.inertia)
					: pr::physics::Inertia::Infinite());
			if (value.motion_type == PhysicsMotionType::Dynamic)
			{
				switch (value.mass_mode)
				{
					case PhysicsMassMode::ExplicitInertia:
					{
						break;
					}
					case PhysicsMassMode::Mass:
					{
						if (!(value.mass_or_density > 0.0f))
							throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Body mass must be positive");

						body_record->m_body->Shape(shape_record.Shape(), value.mass_or_density, false);
						break;
					}
					case PhysicsMassMode::Density:
					{
						if (!(value.mass_or_density > 0.0f))
							throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Body density must be positive");

						body_record->m_body->Shape(shape_record.Shape(), value.mass_or_density, true);
						break;
					}
					default:
					{
						throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Unknown body mass mode");
					}
				}
			}
			body_record->m_body->MomentumWS(pr::physics::ToForce(value.momentum));
			body_record->m_body->GravityWS(pr::physics::ToNative(value.gravity));
			body_record->m_body->NeverSleep(pr::physics::HasFlag(value.flags, PhysicsBodyFlags::NeverSleep));
			body_record->m_body->Sleeping(pr::physics::HasFlag(value.flags, PhysicsBodyFlags::Sleeping));
			body_record->m_shape = value.shape;
			body_record->m_user_tag = value.user_tag;
			body_record->m_motion_type = value.motion_type;
			body_record->m_enabled = pr::physics::HasFlag(value.flags, PhysicsBodyFlags::Enabled);

			auto [slot, index] = pr::physics::AllocateSlot(record.m_bodies);
			slot.m_object = std::move(body_record);
			*body = pr::physics::MakeChildHandle(record.m_cookie, index, slot.m_generation);
			record.m_body_handles.emplace(slot.m_object->m_body.get(), *body);
			++shape_record.m_body_refs;
		});
	}

	PhysicsStatus __stdcall Physics_BodyDestroy(PhysicsEngineHandle engine, PhysicsBodyHandle body)
	{
		return pr::physics::ApiCall([&]
		{
			auto scope = pr::physics::EngineScope(engine);
			auto& record = *scope;
			pr::physics::RequireOwner(record);
			pr::physics::PrepareChildDestroy(record);
			auto [body_record, index] = pr::physics::RequireChild(record, body, record.m_bodies);

			record.m_body_handles.erase(body_record.m_body.get());
			--pr::physics::RequireShape(record, body_record.m_shape).m_body_refs;
			auto& slot = record.m_bodies[index];
			slot.m_object.reset();
			pr::physics::AdvanceGeneration(slot.m_generation);
		});
	}

	PhysicsStatus __stdcall Physics_BodyStateGet(PhysicsEngineHandle engine, PhysicsBodyHandle body, PhysicsBodyState* state)
	{
		return pr::physics::ApiCall([&]
		{
			auto scope = pr::physics::EngineScope(engine);
			auto& record = *scope;
			pr::physics::RequireIdle(record);
			if (state == nullptr)
				throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Body state output pointer is null");

			pr::physics::FillBodyState(*state, body, pr::physics::RequireBody(record, body));
		});
	}

	PhysicsStatus __stdcall Physics_BodyStateSet(PhysicsEngineHandle engine, PhysicsBodyHandle body, PhysicsBodyState const* state)
	{
		return pr::physics::ApiCall([&]
		{
			auto scope = pr::physics::EngineScope(engine);
			auto& record = *scope;
			pr::physics::RequireOwner(record);
			pr::physics::RequireIdle(record);
			auto const& value = pr::physics::RequireStruct(state);
			auto& body_record = pr::physics::RequireBody(record, body);
			if (value.body != 0 && value.body != body)
				throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Body state identifies a different body");
			if (value.shape != body_record.m_shape)
				throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Body shape cannot be changed through BodyStateSet");

			pr::physics::ApplyBodyState(body_record, value);
		});
	}

	PhysicsStatus __stdcall Physics_CommandsApply(PhysicsEngineHandle engine, PhysicsBodyCommand const* commands, std::uint32_t command_count)
	{
		return pr::physics::ApiCall([&]
		{
			auto scope = pr::physics::EngineScope(engine);
			auto& record = *scope;
			pr::physics::RequireOwner(record);
			pr::physics::RequireIdle(record);
			pr::physics::ApplyCommands(record, commands, command_count);
		});
	}

	PhysicsStatus __stdcall Physics_BeginStep(PhysicsEngineHandle engine, float elapsed_seconds, double absolute_time_seconds, PhysicsBodyCommand const* commands, std::uint32_t command_count)
	{
		return pr::physics::ApiCall([&]
		{
			auto scope = pr::physics::EngineScope(engine);
			pr::physics::BeginStep(*scope, elapsed_seconds, absolute_time_seconds, commands, command_count);
		});
	}

	PhysicsStatus __stdcall Physics_CompleteStep(PhysicsEngineHandle engine)
	{
		return pr::physics::ApiCall([&]
		{
			auto scope = pr::physics::EngineScope(engine);
			pr::physics::CompleteStep(*scope);
		});
	}

	PhysicsStatus __stdcall Physics_Step(PhysicsEngineHandle engine, float elapsed_seconds, double absolute_time_seconds, PhysicsBodyCommand const* commands, std::uint32_t command_count)
	{
		return pr::physics::ApiCall([&]
		{
			auto scope = pr::physics::EngineScope(engine);
			auto& record = *scope;
			pr::physics::BeginStep(record, elapsed_seconds, absolute_time_seconds, commands, command_count);
			pr::physics::CompleteStep(record);
		});
	}

	PhysicsStatus __stdcall Physics_SnapshotCopy(PhysicsEngineHandle engine, PhysicsBodySnapshot* snapshots, std::uint32_t capacity, std::uint32_t* required)
	{
		return pr::physics::ApiCall([&]
		{
			auto scope = pr::physics::EngineScope(engine);
			auto& record = *scope;
			pr::physics::RequireIdle(record);
			if (required == nullptr)
				throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Snapshot required-count pointer is null");

			auto count = std::uint32_t{};
			for (auto const& slot : record.m_bodies)
				count += slot.m_object && slot.m_object->m_enabled ? 1U : 0U;

			*required = count;
			if (capacity < count || (count != 0 && snapshots == nullptr))
				throw pr::physics::ApiException(PhysicsStatus::BufferTooSmall, "Snapshot buffer is too small");

			auto output = std::uint32_t{};
			for (auto i = std::size_t{}; i != record.m_bodies.size(); ++i)
			{
				auto const& slot = record.m_bodies[i];
				if (!slot.m_object || !slot.m_object->m_enabled)
					continue;

				snapshots[output++] = pr::physics::MakeSnapshot(
					pr::physics::MakeChildHandle(record.m_cookie, i, slot.m_generation),
					*slot.m_object);
			}
		});
	}

	PhysicsStatus __stdcall Physics_EventsCopy(PhysicsEngineHandle engine, PhysicsEvent* events, std::uint32_t capacity, std::uint32_t* required)
	{
		return pr::physics::ApiCall([&]
		{
			auto scope = pr::physics::EngineScope(engine);
			auto& record = *scope;
			pr::physics::RequireIdle(record);
			if (required == nullptr)
				throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Event required-count pointer is null");

			*required = static_cast<std::uint32_t>(record.m_events.size());
			if (capacity < record.m_events.size() || (!record.m_events.empty() && events == nullptr))
				throw pr::physics::ApiException(PhysicsStatus::BufferTooSmall, "Event buffer is too small");

			std::ranges::copy(record.m_events, events);
		});
	}

	PhysicsStatus __stdcall Physics_DiagnosticsGet(PhysicsEngineHandle engine, PhysicsDiagnostics* diagnostics)
	{
		return pr::physics::ApiCall([&]
		{
			auto scope = pr::physics::EngineScope(engine);
			auto& record = *scope;
			if (diagnostics == nullptr)
				throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Diagnostics output pointer is null");

			auto const& profile = record.m_engine->LastStepProfile();
			auto const& collisions = record.m_engine->LastCollisionStats();
			auto device_removed_reason = record.m_engine->Device()->GetDeviceRemovedReason();
			auto body_count = std::ranges::count_if(record.m_bodies, [](auto const& slot) { return slot.m_object != nullptr; });
			auto shape_count = std::ranges::count_if(record.m_shapes, [](auto const& slot) { return slot.m_object != nullptr; });

			*diagnostics = PhysicsDiagnostics{
				.header = {sizeof(PhysicsDiagnostics), PHYSICS_STRUCT_VERSION},
				.profile = {
					profile.m_new_frame_ms,
					profile.m_pack_ms,
					profile.m_upload_ms,
					profile.m_external_forces_ms,
					profile.m_integrate_ms,
					profile.m_sleepwake_ms,
					profile.m_broadphase_ms,
					profile.m_collide_ms,
					profile.m_resolve_ms,
					profile.m_selective_ms,
					profile.m_sleepupdate_ms,
					profile.m_readback_ms,
					profile.m_gpu_run_ms,
					profile.m_gpu_prepare_ms,
					profile.m_gpu_execute_ms,
					profile.m_gpu_wait_ms,
					profile.m_gpu_reset_ms,
					profile.m_unpack_ms,
				},
				.submitted_step = record.m_submitted_step,
				.completed_step = record.m_completed_step,
				.state_checksum = pr::physics::StateChecksum(record),
				.body_count = static_cast<std::int32_t>(body_count),
				.shape_count = static_cast<std::int32_t>(shape_count),
				.pair_count = collisions.m_pair_count,
				.contact_count = collisions.m_contact_count,
				.max_pairs = collisions.m_max_pairs,
				.max_contacts = collisions.m_max_contacts,
				.step_pending = record.m_submitted_step != record.m_completed_step,
				.device_removed_reason = device_removed_reason == S_OK ? 0 : device_removed_reason,
			};
		});
	}

	PhysicsStatus __stdcall Physics_CheckpointSize(PhysicsEngineHandle engine, std::uint64_t* required)
	{
		return pr::physics::ApiCall([&]
		{
			auto scope = pr::physics::EngineScope(engine);
			auto& record = *scope;
			pr::physics::DrainPendingStep(record);
			if (required == nullptr)
				throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Checkpoint size output pointer is null");

			*required = pr::physics::CheckpointRequiredSize(record);
		});
	}

	PhysicsStatus __stdcall Physics_CheckpointWrite(PhysicsEngineHandle engine, void* buffer, std::uint64_t capacity, std::uint64_t* written)
	{
		return pr::physics::ApiCall([&]
		{
			auto scope = pr::physics::EngineScope(engine);
			auto& record = *scope;
			pr::physics::RequireOwner(record);
			pr::physics::DrainPendingStep(record);
			if (written == nullptr)
				throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Checkpoint written-size output pointer is null");

			auto required = pr::physics::CheckpointRequiredSize(record);
			*written = required;
			if (buffer == nullptr || capacity < required)
				throw pr::physics::ApiException(PhysicsStatus::BufferTooSmall, "Checkpoint buffer is too small");

			auto shape_count = std::ranges::count_if(record.m_shapes, [](auto const& slot) { return slot.m_object != nullptr; });
			auto body_count = std::ranges::count_if(record.m_bodies, [](auto const& slot) { return slot.m_object != nullptr; });
			auto header = pr::physics::CheckpointHeader{
				.m_magic = pr::physics::CheckpointMagic,
				.m_version = PHYSICS_CHECKPOINT_VERSION,
				.m_header_size = sizeof(pr::physics::CheckpointHeader),
				.m_total_size = required,
				.m_payload_checksum = 0,
				.m_engine_cookie = record.m_cookie,
				.m_reserved0 = 0,
				.m_shape_count = static_cast<std::uint32_t>(shape_count),
				.m_body_count = static_cast<std::uint32_t>(body_count),
				.m_reserved1 = 0,
				.m_submitted_step = record.m_submitted_step,
				.m_completed_step = record.m_completed_step,
				.m_config = {},
			};
			pr::physics::FromNative(header.m_config, record.m_engine->Config());

			auto output = static_cast<std::byte*>(buffer);
			pr::physics::WriteCheckpointValue(output, header);

			// Materials and active objects are written in stable numeric order for reproducible checkpoints.
			for (auto id = 0; id != pr::physics::Material::MaxMaterialId; ++id)
			{
				auto native = record.m_engine->Material(id);
				auto material = PhysicsMaterial{
					.header = {sizeof(PhysicsMaterial), PHYSICS_STRUCT_VERSION},
					.id = native.m_id,
					.static_friction = native.m_friction_static,
					.normal_elasticity = native.m_elasticity_norm,
					.tangential_elasticity = native.m_elasticity_tang,
					.torsional_elasticity = native.m_elasticity_tors,
					.density = native.m_density,
				};
				pr::physics::WriteCheckpointValue(output, material);
			}
			for (auto i = std::size_t{}; i != record.m_shapes.size(); ++i)
			{
				auto const& slot = record.m_shapes[i];
				if (!slot.m_object)
					continue;

				auto shape_header = pr::physics::CheckpointShape{
					.m_handle = pr::physics::MakeChildHandle(record.m_cookie, i, slot.m_generation),
					.m_size = static_cast<std::uint32_t>(slot.m_object->m_data.size()),
					.m_child_count = static_cast<std::uint32_t>(slot.m_object->m_children.size()),
				};
				pr::physics::WriteCheckpointValue(output, shape_header);
				memcpy(output, slot.m_object->m_data.data(), slot.m_object->m_data.size());
				output += slot.m_object->m_data.size();
				for (auto child : slot.m_object->m_children)
					pr::physics::WriteCheckpointValue(output, child);
			}
			for (auto i = std::size_t{}; i != record.m_bodies.size(); ++i)
			{
				auto const& slot = record.m_bodies[i];
				if (!slot.m_object)
					continue;

				auto body_entry = pr::physics::CheckpointBody{};
				pr::physics::FillBodyState(
					body_entry.m_state,
					pr::physics::MakeChildHandle(record.m_cookie, i, slot.m_generation),
					*slot.m_object);
				pr::physics::WriteCheckpointValue(output, body_entry);
			}

			auto payload = static_cast<std::byte const*>(buffer) + sizeof(pr::physics::CheckpointHeader);
			header.m_payload_checksum = pr::physics::HashBytes(
				14695981039346656037ULL,
				payload,
				static_cast<std::size_t>(required - sizeof(pr::physics::CheckpointHeader)));
			memcpy(buffer, &header, sizeof(header));
		});
	}

	PhysicsStatus __stdcall Physics_CheckpointRead(PhysicsEngineHandle engine, void const* buffer, std::uint64_t size)
	{
		return pr::physics::ApiCall([&]
		{
			auto scope = pr::physics::EngineScope(engine);
			auto& record = *scope;
			pr::physics::RequireOwner(record);
			pr::physics::RequireIdle(record);
			if (buffer == nullptr || size < sizeof(pr::physics::CheckpointHeader))
				throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Checkpoint buffer is null or truncated");
			if (std::ranges::any_of(record.m_shapes, [](auto const& slot) { return slot.m_object != nullptr; }) ||
				std::ranges::any_of(record.m_bodies, [](auto const& slot) { return slot.m_object != nullptr; }))
				throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Checkpoint import requires an empty engine");

			auto input = static_cast<std::byte const*>(buffer);
			auto end = input + size;
			auto header = pr::physics::ReadCheckpointValue<pr::physics::CheckpointHeader>(input, end);
			if (header.m_magic != pr::physics::CheckpointMagic ||
				header.m_version != PHYSICS_CHECKPOINT_VERSION ||
				header.m_header_size != sizeof(pr::physics::CheckpointHeader))
				throw pr::physics::ApiException(PhysicsStatus::IncompatibleVersion, "Checkpoint format or version is incompatible");
			if (header.m_total_size != size)
				throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Checkpoint size does not match its header");

			auto checksum = pr::physics::HashBytes(
				14695981039346656037ULL,
				input,
				static_cast<std::size_t>(end - input));
			if (checksum != header.m_payload_checksum)
				throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Checkpoint checksum is invalid");

			// Parse and validate the complete checkpoint into temporary storage before changing live state.
			auto materials = std::array<PhysicsMaterial, pr::physics::Material::MaxMaterialId>{};
			for (auto id = std::size_t{}; id != materials.size(); ++id)
			{
				auto& material = materials[id];
				material = pr::physics::ReadCheckpointValue<PhysicsMaterial>(input, end);
				pr::physics::RequireStruct(&material);
				pr::physics::RequireMaterialId(material.id);
				if (material.id != static_cast<std::int32_t>(id))
					throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Checkpoint material slot identity is invalid");
			}

			struct PendingShape
			{
				PhysicsShapeHandle m_handle;
				pr::byte_data<16> m_data;
				std::vector<PhysicsShapeHandle> m_children;
			};
			auto shapes = std::vector<PendingShape>{};
			shapes.reserve(header.m_shape_count);
			auto max_shape_index = std::size_t{};
			for (auto count = std::uint32_t{}; count != header.m_shape_count; ++count)
			{
				auto entry = pr::physics::ReadCheckpointValue<pr::physics::CheckpointShape>(input, end);
				if (pr::physics::ChildCookie(entry.m_handle) != header.m_engine_cookie)
					throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Checkpoint shape cookie is invalid");
				if (static_cast<std::uint64_t>(end - input) < entry.m_size)
					throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Checkpoint shape is truncated");

				auto data = std::span<std::byte const>{input, entry.m_size};
				pr::physics::ValidateCheckpointShape(data);
				input += entry.m_size;

				auto children = std::vector<PhysicsShapeHandle>{};
				children.reserve(entry.m_child_count);
				for (auto child_index = std::uint32_t{}; child_index != entry.m_child_count; ++child_index)
					children.push_back(pr::physics::ReadCheckpointValue<PhysicsShapeHandle>(input, end));

				shapes.push_back(PendingShape{entry.m_handle, pr::byte_data<16>{data}, std::move(children)});
				max_shape_index = std::max(max_shape_index, pr::physics::ChildIndex(entry.m_handle));
			}

			// Compound dependencies must resolve to exact shape identities in the same checkpoint.
			for (auto const& shape : shapes)
			{
				for (auto child : shape.m_children)
				{
					if (child == shape.m_handle ||
						pr::physics::ChildCookie(child) != header.m_engine_cookie ||
						std::ranges::none_of(shapes, [=](auto const& candidate) { return candidate.m_handle == child; }))
						throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Checkpoint compound child identity is invalid");
				}
			}

			auto bodies = std::vector<PhysicsBodyState>{};
			bodies.reserve(header.m_body_count);
			auto max_body_index = std::size_t{};
			for (auto count = std::uint32_t{}; count != header.m_body_count; ++count)
			{
				auto entry = pr::physics::ReadCheckpointValue<pr::physics::CheckpointBody>(input, end);
				pr::physics::RequireStruct(&entry.m_state);
				if (pr::physics::ChildCookie(entry.m_state.body) != header.m_engine_cookie)
					throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Checkpoint body cookie is invalid");

				bodies.push_back(entry.m_state);
				max_body_index = std::max(max_body_index, pr::physics::ChildIndex(entry.m_state.body));
			}
			if (input != end)
				throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Checkpoint contains trailing data");

			// Every body and dependency must resolve before any live engine state is changed.
			for (auto i = std::size_t{}; i != shapes.size(); ++i)
			{
				if (std::ranges::any_of(std::span{shapes}.first(i), [&](auto const& candidate)
					{
						return pr::physics::ChildIndex(candidate.m_handle) == pr::physics::ChildIndex(shapes[i].m_handle);
					}))
					throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Checkpoint contains duplicate shape slots");
			}
			for (auto i = std::size_t{}; i != bodies.size(); ++i)
			{
				if (std::ranges::none_of(shapes, [&](auto const& shape) { return shape.m_handle == bodies[i].shape; }))
					throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Checkpoint body references an unknown shape");
				if (std::ranges::any_of(std::span{bodies}.first(i), [&](auto const& candidate)
					{
						return pr::physics::ChildIndex(candidate.body) == pr::physics::ChildIndex(bodies[i].body);
					}))
					throw pr::physics::ApiException(PhysicsStatus::InvalidArgument, "Checkpoint contains duplicate body slots");
			}

			pr::physics::ClaimEngineCookie(record, header.m_engine_cookie);

			// Materialise validated state using the checkpoint cookie so managed-visible object identities survive restart.
			record.m_engine->Config(pr::physics::ToNative(pr::physics::RequireStruct(&header.m_config)));
			for (auto const& value : materials)
			{
				record.m_engine->Material(pr::physics::Material{
					value.id,
					value.static_friction,
					value.normal_elasticity,
					value.tangential_elasticity,
					value.torsional_elasticity,
					value.density});
			}

			record.m_shapes.resize(shapes.empty() ? 0 : max_shape_index + 1);
			for (auto& value : shapes)
			{
				auto index = pr::physics::ChildIndex(value.m_handle);
				auto& slot = record.m_shapes[index];
				slot.m_generation = pr::physics::ChildGeneration(value.m_handle);
				slot.m_object = std::make_unique<pr::physics::ShapeRecord>();
				slot.m_object->m_data = std::move(value.m_data);
				slot.m_object->m_children = std::move(value.m_children);
			}
			for (auto const& slot : record.m_shapes)
			{
				if (!slot.m_object)
					continue;

				for (auto child : slot.m_object->m_children)
					++pr::physics::RequireShape(record, child).m_compound_refs;
			}

			record.m_bodies.resize(bodies.empty() ? 0 : max_body_index + 1);
			for (auto const& value : bodies)
			{
				auto index = pr::physics::ChildIndex(value.body);
				auto& shape_record = pr::physics::RequireShape(record, value.shape);
				auto& slot = record.m_bodies[index];
				slot.m_generation = pr::physics::ChildGeneration(value.body);
				slot.m_object = std::make_unique<pr::physics::BodyRecord>();
				slot.m_object->m_body = std::make_unique<pr::physics::RigidBody>(
					shape_record.Shape(),
					pr::physics::ToNative(value.object_to_world),
					pr::physics::ToNative(value.inertia));
				slot.m_object->m_shape = value.shape;
				pr::physics::ApplyBodyState(*slot.m_object, value);
				record.m_body_handles.emplace(slot.m_object->m_body.get(), value.body);
				++shape_record.m_body_refs;
			}
			record.m_submitted_step = header.m_submitted_step;
			record.m_completed_step = header.m_completed_step;
			record.m_events.clear();
			record.m_engine->ResetCaches();
		});
	}
}
