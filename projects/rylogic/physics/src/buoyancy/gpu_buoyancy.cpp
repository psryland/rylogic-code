//************************************
// Physics Engine
//  Copyright (c) Rylogic Ltd 2026
//************************************
#include "pr/physics/buoyancy/gpu_buoyancy.h"
#include "pr/physics/articulation/articulation.h"
#include "pr/physics/rigid_body/rigid_body.h"
#include "pr/physics/buoyancy/buoyancy_sampler.h"
#include "pr/compute/compute_pso.h"
#include "pr/compute/compute_step.h"
#include "pr/compute/shaders/shader_compiler.h"
#include "pr/compute/utility/root_signature.h"

#include "src/utility/gpu.h"

namespace pr::physics
{
	namespace
	{
		// Sampled-composite pass tunables. Surface sample counts are fixed at registration (see buoyancy::BuildSurfacePlan).
		// At most the reduce thread count of groups serve each hull (the reducer sums one partial per group on a single
		// thread group); larger hulls stream additional samples through the same threads.
		static constexpr int BuoyancySampleThreadCount = 256;
		static constexpr int BuoyancyReduceThreadCount = 128;

		struct CBufGpuBuoyancy
		{
			int m_hull_count;
			int m_groups_per_hull;
			int m_water_field_count;
			float m_time_s;
			float m_water_level;
			float m_water_max_height;
			float m_fluid_density;
			float m_linear_drag_coefficient;
			float m_angular_drag_coefficient;
			float m_quadratic_drag_coefficient;
			float m_tangential_drag_coefficient;
			float m_added_mass_coefficient;
			float m_radiation_damping;
			float m_time_step_s;
			int m_enable_diagnostics;
			float m_wave_resistance;
			float m_air_density;
		};
		static_assert(sizeof(CBufGpuBuoyancy) % sizeof(uint32_t) == 0);

		// Mirrors HLSL BuoyancySums; see gpu_buoyancy.hlsl for the meaning of each sum. The host only sizes the buffer.
		struct GpuBuoyancyPartial
		{
			float m_sums[49];
		};
		static_assert(sizeof(GpuBuoyancyPartial) == 196);

		struct GpuBuoyancyDiagnostic
		{
			v4 m_force_ws;
			v4 m_torque_ws;
			v4 m_centre_buoyancy_ws;
			float m_volume_m3;
			int m_valid;
			float m_pad0;
			float m_pad1;
		};
		static_assert(sizeof(GpuBuoyancyDiagnostic) == 64);

		// Per-hull header for the sampled-composite pass. Mirrors HLSL BuoySurfHeader. The body index is the body's STEP index
		// (into the engine body list) resolved at dispatch time. m_prim_base/m_prim_count index the concatenated primitive array
		// uploaded for this dispatch.
		struct GpuBuoySurfHeader
		{
			int m_body_index;
			int m_prim_base;
			int m_prim_count;
			int m_total_surface_samples;
			float m_eps;
			int m_pad0;
			int m_pad1;
			int m_pad2;
		};
		static_assert(sizeof(GpuBuoySurfHeader) == 32);

		// Per-primitive surface counts, compact patch range, and half of the largest cell height (for the dry early-out).
		// Mirrors HLSL BuoySurfPrimRecord.
		struct GpuBuoySurfPrimRecord
		{
			int m_count;
			int m_patch_begin;
			int m_patch_count;
			float m_half_cell_height;
		};
		static_assert(sizeof(GpuBuoySurfPrimRecord) == 16);


		// Create a default-heap UAV buffer used by the diagnostic compute passes.
		template <typename T>
		D3DPtr<ID3D12Resource> CreateDefaultUavBuffer(ID3D12Device* device, int count, std::string_view name)
		{
			if (count <= 0)
			{
				throw std::runtime_error("GPU buoyancy buffer capacity must be positive");
			}

			auto const desc = D3D12_RESOURCE_DESC{
				.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER,
				.Alignment = D3D12_DEFAULT_RESOURCE_PLACEMENT_ALIGNMENT,
				.Width = static_cast<UINT64>(count) * sizeof(T),
				.Height = 1,
				.DepthOrArraySize = 1,
				.MipLevels = 1,
				.Format = DXGI_FORMAT_UNKNOWN,
				.SampleDesc = DXGI_SAMPLE_DESC{ .Count = 1, .Quality = 0 },
				.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR,
				.Flags = D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS,
			};

			auto res = D3DPtr<ID3D12Resource>{};
			auto const& heap_props = ::pr::compute::HeapProps::Default();
			Check(device->CreateCommittedResource(
				&heap_props,
				D3D12_HEAP_FLAG_NONE,
				&desc,
				D3D12_RESOURCE_STATE_COMMON,
				nullptr,
				__uuidof(ID3D12Resource),
				reinterpret_cast<void**>(res.address_of())));

			::pr::compute::DefaultResState(res.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			::pr::compute::DebugName(res, name);
			return res;
		}

		// Compile a runtime compute shader entry point from the physics buoyancy shader source.
		std::vector<uint8_t> CompileBuoyancyShader(wchar_t const* entry_point)
		{
			auto resolver = ::pr::compute::shader_cache::ResourceSourceResolver{};
			auto compiler = ::pr::compute::ShaderCompiler{}
				.Source("src/buoyancy/gpu_buoyancy.hlsl", resolver)
				.HlslVersion(::pr::compute::EHlslVersion::Hlsl2021)
				.Define(L"SHADER_BUILD")
				.Optimise(true)
				.ShaderModel(L"cs_6_6")
				.EntryPoint(entry_point);
			return compiler.Compile();
		}

		// Create the sampled-composite surface-sample compute step. The root signature mirrors
		// the resource access order of CSBuoyancySurfaceSamples: constants (b0), the body
		// accumulator (u0), the water-field SRV (t1, for water height/velocity), the primitives SRV (t3) and
		// face planes SRV (t6, for the sibling cull), headers (t8), patches (t9), primitive records (t11), then the
		// partials UAV (u1). Root descriptors need not be contiguous.
		::pr::compute::ComputeStep CreateSurfaceStep(ID3D12Device* device)
		{
			auto step = ::pr::compute::ComputeStep{};
			step.m_sig = ::pr::compute::RootSig(::pr::compute::ERootSigFlags::ComputeOnly)
				.U32<CBufGpuBuoyancy>(hlsl::ECBufReg::b0)
				.UAV(hlsl::EUAVReg::u0)
				.SRV(hlsl::ESRVReg::t1)
				.SRV(hlsl::ESRVReg::t3)
				.SRV(hlsl::ESRVReg::t6)
				.SRV(hlsl::ESRVReg::t8)
				.SRV(hlsl::ESRVReg::t9)
				.SRV(hlsl::ESRVReg::t11)
				.UAV(hlsl::EUAVReg::u1)
				.Create(device, "Physics.GpuBuoyancy.Surface.RootSig");

			step.m_pso = ::pr::compute::ComputePSO(step.m_sig.get(), CompileBuoyancyShader(L"CSBuoyancySurfaceSamples")).Create(device, "Physics.GpuBuoyancy.Surface.PSO");
			return step;
		}

		// Create the sampled-composite surface-reduce compute step. The root signature mirrors the
		// resource access order of CSBuoyancySurfaceReduce: constants (b0), the body accumulator
		// (u0), the per-hull surface headers SRV (t8), the partials UAV (u1), then the diagnostics UAV (u2).
		// The reduce adds the hydrodynamic force/torque to the body and writes the diagnostic.
		::pr::compute::ComputeStep CreateSurfaceReduceStep(ID3D12Device* device)
		{
			auto step = ::pr::compute::ComputeStep{};
			step.m_sig = ::pr::compute::RootSig(::pr::compute::ERootSigFlags::ComputeOnly)
				.U32<CBufGpuBuoyancy>(hlsl::ECBufReg::b0)
				.UAV(hlsl::EUAVReg::u0)
				.SRV(hlsl::ESRVReg::t8)
				.UAV(hlsl::EUAVReg::u1)
				.UAV(hlsl::EUAVReg::u2)
				.Create(device, "Physics.GpuBuoyancy.SurfaceReduce.RootSig");

			step.m_pso = ::pr::compute::ComputePSO(step.m_sig.get(), CompileBuoyancyShader(L"CSBuoyancySurfaceReduce")).Create(device, "Physics.GpuBuoyancy.SurfaceReduce.PSO");
			return step;
		}
	}

	struct GpuBuoyancy::Impl
	{
		// Immutable geometry and sample plans shared by every body that references the same collision
		// shape. Collision shapes are immutable value blobs, so pointer identity is a stable cache key
		// while at least one body retains the shape.
		struct CompositeShape
		{
			buoyancy::CompositeHull m_hull;

			// Scale-relative inside-test slack for sibling culling.
			float m_eps = 0.0f;

			// Half of the largest surface cell height per primitive, and over the whole hull (see buoyancy::SurfaceCellHeight).
			// A surface cell is partly wet while its sample is less than this far above the surface, so a primitive or body is
			// only certainly dry when its lowest point is this far above the water.
			std::vector<float> m_half_cell_heights;
			float m_max_half_cell_height = 0.0f;

			// Registration-time body-space AABB enclosing every primitive in the hull. Used by the
			// host-side flat-water dry broadphase cull to conservatively test whether the whole body
			// is above the water line along its local up (-gravity) axis.
			BBox m_obb_os = BBox::Reset();

			// Feature-preserving surface patches are cached per primitive, not per emitted sample (see buoyancy::BuildSurfacePlan).
			std::vector<surface::Plan> m_surface_plans;
			int m_total_surface_samples = 0;
		};
		struct ShapeCacheKey
		{
			collision::Shape const* m_shape;
			float m_surface_spacing;

			friend bool operator ==(ShapeCacheKey const&, ShapeCacheKey const&) = default;
		};
		struct ShapeCacheHash
		{
			std::size_t operator()(ShapeCacheKey const& key) const noexcept
			{
				auto hash = std::hash<collision::Shape const*>{}(key.m_shape);
				return hash ^ (std::hash<float>{}(key.m_surface_spacing) + 0x9e3779b9U + (hash << 6) + (hash >> 2));
			}
		};
		struct CompositeSlot
		{
			// Per-target registration state. Expensive shape-derived data is shared; only identity,
			// lifetime, target resolution, and rigid-body wake state remain per target.
			int m_generation = -1;
			bool m_active = false;
			RigidBody* m_body = nullptr;
			Articulation* m_articulation = nullptr;
			LinkHandle m_link = {};
			bool m_prev_never_sleep = false;
			multicast::AutoSub m_shape_change_sub;
			std::shared_ptr<CompositeShape const> m_shape_data;
		};

		// Shares one tree-wide NeverSleep override among all buoyant links in the same articulation.
		struct ArticulationSleepOwner
		{
			Articulation* m_articulation;
			int m_registration_count;
			bool m_prev_never_sleep;
		};

		// A compact reference to a registered hull participating in the current dispatch. Instances live in
		// reusable CPU scratch storage, so collecting active hulls does not allocate once registration is complete.
		struct ActiveHull
		{
			int m_body_index;
			int m_body_generation;
			int m_body_step_index;
			int m_shape_index;
		};
		struct ActiveShape
		{
			std::shared_ptr<CompositeShape const> m_shape_data;
			int m_prim_base;
			int m_prim_count;
		};
		struct ActiveShapeBucket
		{
			CompositeShape const* m_shape_data;
			int m_shape_index;
		};
		struct ActiveHullStats
		{
			int m_max_surface_samples;
		};

		// Metadata retained until the matching diagnostic readback completes.
		struct PendingDiagnostic
		{
			int m_body_index;
			int m_body_generation;
		};

		// GPU locations of the transient upload blocks consumed by the sampled-composite passes.
		struct DispatchAddresses
		{
			GpuAddress m_water_field;
			GpuAddress m_face_planes;
			GpuAddress m_surface_headers;
			GpuAddress m_surface_primitives;
			GpuAddress m_surface_records;
			GpuAddress m_surface_patches;
		};

		// Device services and callbacks remain valid for the lifetime of this implementation.
		ID3D12Device* m_device;
		Engine* m_engine;
		StepIndexResolver m_step_index_resolver;
		BodyStateResolver m_body_state_resolver;
		bool m_supports_rigid_bodies;

		// Immutable compute pipelines and the engine subscription used to append buoyancy work to each GPU step.
		::pr::compute::ComputeStep m_surface_step;
		::pr::compute::ComputeStep m_surface_reduce_step;
		multicast::AutoSub m_external_force_sub;

		// Runtime settings and registration-owned hull data. Hull slots are indexed by stable body index.
		terrain::water::WaterField m_water_field;
		Air m_air;
		Config m_config;
		std::unordered_map<ShapeCacheKey, std::weak_ptr<CompositeShape const>, ShapeCacheHash> m_shape_cache;
		std::vector<CompositeSlot> m_composite_hulls;
		std::vector<ArticulationSleepOwner> m_articulation_sleep_owners;

		// Reusable host-side dispatch state. Capacity tracks m_composite_hulls and is established during
		// registration so the per-step Apply/DispatchComposite path only clears and repopulates storage.
		std::vector<ActiveHull> m_active_hulls;
		std::vector<ActiveShape> m_active_shapes;
		std::vector<ActiveShapeBucket> m_active_shape_lookup;
		std::vector<PendingDiagnostic> m_pending_diagnostics;
		::pr::compute::GpuReadbackBuffer::Allocation m_pending_readback;

		// Grow-only GPU outputs of the sample and reduce passes. They are recreated only when a
		// dispatch exceeds the largest hull/group population seen previously.
		D3DPtr<ID3D12Resource> m_r_partials;
		D3DPtr<ID3D12Resource> m_r_diagnostics;
		int m_partial_capacity;
		int m_diagnostic_capacity;

		// Latest completed diagnostic values published for callers. Readback completion writes under the same
		// mutex used by LatestDiagnostics because rendering and physics scheduling may run independently.
		mutable std::mutex m_diagnostics_mutex;
		std::vector<Diagnostics> m_diagnostics;

		// Construct and subscribe the buoyancy compute pass.
		Impl(ID3D12Device* device, Engine& engine, Config const& config, StepIndexResolver step_index_resolver, BodyStateResolver body_state_resolver, bool supports_rigid_bodies)
			:m_device(device)
			,m_engine(&engine)
			,m_step_index_resolver(std::move(step_index_resolver))
			,m_body_state_resolver(std::move(body_state_resolver))
			,m_supports_rigid_bodies(supports_rigid_bodies)
			,m_surface_step(CreateSurfaceStep(device))
			,m_surface_reduce_step(CreateSurfaceReduceStep(device))
			,m_external_force_sub(engine.ExternalForces += [this](Engine& sender, Engine::ExternalForceArgs const& args)
			{
				Apply(sender, args);
			})
			,m_water_field()
			,m_config()
			,m_shape_cache()
			,m_composite_hulls()
			,m_articulation_sleep_owners()
			,m_active_hulls()
			,m_active_shapes()
			,m_active_shape_lookup()
			,m_pending_diagnostics()
			,m_pending_readback()
			,m_r_partials()
			,m_r_diagnostics()
			,m_partial_capacity()
			,m_diagnostic_capacity()
			,m_diagnostics_mutex()
			,m_diagnostics()
		{
			if (m_device == nullptr)
			{
				throw std::runtime_error("GpuBuoyancy requires a D3D12 device");
			}
			if (!m_step_index_resolver)
			{
				throw std::runtime_error("GpuBuoyancy requires a body step-index resolver");
			}
			if (!m_body_state_resolver)
			{
				throw std::runtime_error("GpuBuoyancy requires a body-state resolver");
			}

			// Validate and apply the initial config through the same path used by SetConfig so all
			// invariants (finite values, non-negative density) are checked once at construction.
			SetConfig(config);
		}

		// Destroy the buoyancy pass after all physics GPU work has completed.
		~Impl() = default;

		// Grow registration storage and its per-step scratch capacity outside the dispatch hot path.
		void EnsureHostCapacity(int slot_count)
		{
			if (slot_count > static_cast<int>(m_composite_hulls.size()))
			{
				m_composite_hulls.resize(static_cast<std::size_t>(slot_count));
			}

			// Active and pending populations cannot exceed the number of registration slots. Reserving to the
			// slot vector's geometric capacity avoids both per-step allocation and repeated exact-size growth.
			auto const slot_capacity = m_composite_hulls.capacity();
			m_active_hulls.reserve(slot_capacity);
			m_active_shapes.reserve(slot_capacity);
			m_pending_diagnostics.reserve(slot_capacity);

			// Keep the open-addressed table at least half empty so shared-shape lookup remains
			// effectively constant time without allocating hash-map nodes during each dispatch.
			auto bucket_count = std::size_t{1};
			while (bucket_count < 2 * slot_capacity)
				bucket_count *= 2;

			if (bucket_count > m_active_shape_lookup.size())
				m_active_shape_lookup.resize(bucket_count);
		}

		// Return shared derived data for a collision shape, creating it once per shape pointer and
		// surface-spacing setting. Weak cache entries do not extend the source shape's lifetime.
		std::shared_ptr<CompositeShape const> GetOrCreateCompositeShape(collision::Shape const& shape)
		{
			auto const key = ShapeCacheKey{ &shape, m_config.m_surface_spacing };
			if (auto iter = m_shape_cache.find(key); iter != m_shape_cache.end())
			{
				if (auto existing = iter->second.lock())
					return existing;
			}

			auto flattened = buoyancy::FlattenShape(shape);
			if (flattened.Empty())
				throw std::runtime_error("Composite buoyancy hull contains no primitives");

			// Build the surface plans in the same primitive order as the flattened GPU descriptors, rejecting an unrepresentable
			// hull before registration. The largest cell of each primitive bounds how far above the water a wet sample can be.
			auto const prims = buoyancy::CollectPrimitives(shape);
			auto surface_plans = std::vector<surface::Plan>(prims.size());
			auto half_cell_heights = std::vector<float>(prims.size(), 0.0f);
			auto total_surface_samples = 0;
			for (std::size_t k = 0; k != prims.size(); ++k)
			{
				surface_plans[k] = buoyancy::BuildSurfacePlan(*prims[k], m_config.m_surface_spacing);
				if (surface_plans[k].m_count > surface::MaxSampleCount - total_surface_samples)
					throw std::runtime_error("Buoyancy surface hull exceeds representable sample count");

				for (uint32_t i = 0; i != surface_plans[k].m_count; ++i)
					half_cell_heights[k] = std::max(half_cell_heights[k], 0.5f * std::sqrt(surface::EmitSurfaceSample(surface_plans[k], i).m_darea));

				total_surface_samples += static_cast<int>(surface_plans[k].m_count);
			}

			auto const bbox = collision::CalcBBox(shape);
			auto const extent = MaxElement(bbox.m_radius.w0());

			auto data = std::make_shared<CompositeShape>();
			data->m_hull = std::move(flattened);
			data->m_eps = std::max(1e-6f, extent * 1e-5f);
			data->m_max_half_cell_height = std::ranges::max(half_cell_heights);
			data->m_half_cell_heights = std::move(half_cell_heights);
			data->m_obb_os = bbox;
			data->m_surface_plans = std::move(surface_plans);
			data->m_total_surface_samples = total_surface_samples;
			m_shape_cache[key] = data;
			return data;
		}

		// Refresh a live registration after its rigid body adopts a new collision shape.
		void HandleShapeChange(int body_index, int body_generation, ChangeEventArgs<collision::Shape const*> const& args)
		{
			if (args.m_before)
				return;

			auto& slot = m_composite_hulls[body_index];
			if (!slot.m_active || slot.m_generation != body_generation)
				return;

			try
			{
				if (args.m_value == nullptr)
					throw std::runtime_error("A buoyancy-registered rigid body must have a collision shape");

				slot.m_shape_data = GetOrCreateCompositeShape(*args.m_value);
			}
			catch (...)
			{
				slot.m_shape_data.reset();
				throw;
			}

			auto lock = std::lock_guard<std::mutex>(m_diagnostics_mutex);
			m_diagnostics[body_index] = Diagnostics{};
			m_diagnostics[body_index].m_body_index = body_index;
			m_diagnostics[body_index].m_body_generation = body_generation;
		}

		// Register a rigid body's collision shape as its buoyancy hull.
		void RegisterCompositeHull(RigidBody& body, int body_index, int body_generation)
		{
			if (!m_supports_rigid_bodies)
				throw std::runtime_error("GpuBuoyancy requires rigid-body resolvers before a rigid body can be registered");
			if (body_index < 0 || body_generation < 0)
				throw std::runtime_error("Invalid body handle for buoyancy hull registration");
			if (!body.HasShape())
				throw std::runtime_error("A buoyancy-registered rigid body must have a collision shape");

			// Composite slots are indexed by stable body index. Establish matching scratch capacity here so
			// collecting those slots during a physics step cannot grow a vector.
			EnsureHostCapacity(body_index + 1);

			auto& slot = m_composite_hulls[body_index];
			if (slot.m_active)
				throw std::runtime_error("A buoyancy hull is already registered for this body");

			auto shape_data = GetOrCreateCompositeShape(body.Shape());
			auto shape_change_sub = multicast::AutoSub(body.ShapeChange += [this, body_index, body_generation](RigidBody&, ChangeEventArgs<collision::Shape const*> const& args)
			{
				HandleShapeChange(body_index, body_generation, args);
			});

			{
				auto lock = std::lock_guard<std::mutex>(m_diagnostics_mutex);
				if (body_index >= static_cast<int>(m_diagnostics.size()))
					m_diagnostics.resize(static_cast<std::size_t>(body_index + 1));

				auto& diagnostic = m_diagnostics[body_index];
				diagnostic = Diagnostics{};
				diagnostic.m_body_index = body_index;
				diagnostic.m_body_generation = body_generation;
			}

			slot.m_generation = body_generation;
			slot.m_active = true;
			slot.m_shape_data = std::move(shape_data);
			slot.m_shape_change_sub = std::move(shape_change_sub);

			// Keep the body awake for the lifetime of the registration. Buoyancy is a continuous
			// environmental force: the engine bails out of the GPU pipeline (and therefore
			// ExternalForces) when every dynamic body is asleep, which would freeze a floater in
			// place even as waves pass under it. Remember the prior flag so unregister can restore it.
			slot.m_body = &body;
			slot.m_prev_never_sleep = body.NeverSleep();
			body.NeverSleep(true);
			body.Wake();
		}

		// Retain one tree-wide wake override until the last buoyant link registration is released.
		void AcquireArticulationSleepOwnership(Articulation& articulation)
		{
			for (auto& owner : m_articulation_sleep_owners)
			{
				if (owner.m_articulation != &articulation)
					continue;

				++owner.m_registration_count;
				return;
			}

			m_articulation_sleep_owners.push_back(ArticulationSleepOwner{
				.m_articulation = &articulation,
				.m_registration_count = 1,
				.m_prev_never_sleep = articulation.NeverSleep(),
			});
			articulation.NeverSleep(true);
		}

		// Restore the caller's tree-wide sleep policy after its final buoyant link is released.
		void ReleaseArticulationSleepOwnership(Articulation& articulation) noexcept
		{
			for (auto iter = m_articulation_sleep_owners.begin(); iter != m_articulation_sleep_owners.end(); ++iter)
			{
				if (iter->m_articulation != &articulation)
					continue;

				assert(iter->m_registration_count > 0);
				if (--iter->m_registration_count != 0)
					return;

				articulation.NeverSleep(iter->m_prev_never_sleep);
				m_articulation_sleep_owners.erase(iter);
				return;
			}

			assert("Articulation buoyancy sleep ownership was released without a matching registration" && false);
		}

		// Register an articulation link's immutable collision shape as its buoyancy hull.
		void RegisterCompositeHull(Articulation& articulation, LinkHandle link, int body_index, int body_generation)
		{
			if (body_index < 0 || body_generation < 0)
				throw std::runtime_error("Invalid target handle for articulation buoyancy hull registration");

			auto const& desc = articulation.LinkDescription(link);
			if (desc.m_shape == nullptr)
				throw std::runtime_error("A buoyancy-registered articulation link must have a collision shape");

			// Articulation links expose immutable geometry, so registration does not need a shape-change subscription.
			EnsureHostCapacity(body_index + 1);
			auto& slot = m_composite_hulls[body_index];
			if (slot.m_active)
				throw std::runtime_error("A buoyancy hull is already registered for this target");

			auto shape_data = GetOrCreateCompositeShape(*desc.m_shape);
			{
				auto lock = std::lock_guard<std::mutex>(m_diagnostics_mutex);
				if (body_index >= static_cast<int>(m_diagnostics.size()))
					m_diagnostics.resize(static_cast<std::size_t>(body_index + 1));

				auto& diagnostic = m_diagnostics[body_index];
				diagnostic = Diagnostics{};
				diagnostic.m_body_index = body_index;
				diagnostic.m_body_generation = body_generation;
			}

			AcquireArticulationSleepOwnership(articulation);
			slot.m_generation = body_generation;
			slot.m_active = true;
			slot.m_articulation = &articulation;
			slot.m_link = link;
			slot.m_shape_data = std::move(shape_data);
		}

		// Remove the buoyancy hull for a stable physics body index.
		void UnregisterHull(int body_index, int body_generation) noexcept
		{
			if (body_index < 0)
			{
				return;
			}

			// Composite hull slot.
			if (body_index < static_cast<int>(m_composite_hulls.size()))
			{
				auto& slot = m_composite_hulls[body_index];
				if (slot.m_active && slot.m_generation == body_generation)
				{
					if (slot.m_body != nullptr)
					{
						slot.m_body->NeverSleep(slot.m_prev_never_sleep);
					}
					if (slot.m_articulation != nullptr)
					{
						ReleaseArticulationSleepOwnership(*slot.m_articulation);
					}

					slot = CompositeSlot{};

					auto lock = std::lock_guard<std::mutex>(m_diagnostics_mutex);
					if (body_index < static_cast<int>(m_diagnostics.size()))
					{
						m_diagnostics[body_index] = Diagnostics{};
					}
				}
			}
		}

		// Return the latest diagnostic record for a registered hull.
		Diagnostics LatestDiagnostics(int body_index, int body_generation) const
		{
			if (!m_config.m_enable_diagnostics)
				return Diagnostics{};

			auto lock = std::lock_guard<std::mutex>(m_diagnostics_mutex);
			if (body_index < 0 || body_index >= static_cast<int>(m_diagnostics.size()))
			{
				return Diagnostics{};
			}

			auto diagnostic = m_diagnostics[body_index];
			if (diagnostic.m_body_generation != body_generation)
			{
				return Diagnostics{};
			}

			return diagnostic;
		}

		// Set the water field used by subsequent buoyancy force dispatches.
		void SetWaterField(terrain::water::WaterField const& water_field)
		{
			// The shaders read element amplitudes directly, so a wind sea's local amplitudes would be silently replaced by its bounds.
			if (water_field.WindSea() != nullptr)
				throw std::invalid_argument("GpuBuoyancy does not support water fields with a wind sea");

			m_water_field = water_field;
		}

		// Return the water field used by buoyancy force dispatches.
		terrain::water::WaterField const& GetWaterField() const
		{
			return m_water_field;
		}

		// Set the air above the water used by subsequent buoyancy force dispatches.
		void SetAir(Air const& air)
		{
			if (!std::isfinite(air.m_density) || air.m_density < 0.0f)
			{
				throw std::invalid_argument("GpuBuoyancy air density must be a finite, non-negative value");
			}

			m_air = air;
		}

		// Return the air used by buoyancy force dispatches.
		Air const& GetAir() const
		{
			return m_air;
		}

		// Return a float upper bound on every GPU-evaluated water height. The shader adds float element terms to a float level,
		// so the double bound is widened by one ulp plus a relative rounding allowance per element before it is used as a dry test.
		float WaterMaxHeight() const
		{
			auto const bound = m_water_field.MaxHeight();
			auto const rounding = 1e-6 * (std::abs(bound) + static_cast<double>(m_water_field.Elements().size()) * (bound - m_water_field.Level()));
			return std::nextafter(static_cast<float>(bound + rounding), std::numeric_limits<float>::infinity());
		}

		// Set the tunable buoyancy parameters used by subsequent dispatches.
		void SetConfig(Config const& config)
		{
			surface::ValidateSpacing(config.m_surface_spacing);
			if (!std::isfinite(config.m_fluid_density) || config.m_fluid_density < 0.0f)
			{
				throw std::runtime_error("GpuBuoyancy fluid density must be a finite, non-negative value");
			}
			if (!std::isfinite(config.m_linear_drag_time_constant_s))
			{
				throw std::runtime_error("GpuBuoyancy linear drag time-constant must be finite");
			}
			if (!std::isfinite(config.m_angular_drag_time_constant_s))
			{
				throw std::runtime_error("GpuBuoyancy angular drag time-constant must be finite");
			}
			if (!std::isfinite(config.m_quadratic_drag_coefficient) || config.m_quadratic_drag_coefficient < 0.0f)
			{
				throw std::runtime_error("GpuBuoyancy quadratic drag coefficient must be a finite, non-negative value");
			}
			if (!std::isfinite(config.m_tangential_drag_coefficient) || config.m_tangential_drag_coefficient < 0.0f)
			{
				throw std::runtime_error("GpuBuoyancy tangential drag coefficient must be a finite, non-negative value");
			}
			if (!std::isfinite(config.m_added_mass_coefficient) || config.m_added_mass_coefficient < 0.0f)
			{
				throw std::runtime_error("GpuBuoyancy added-mass coefficient must be a finite, non-negative value");
			}
			if (!std::isfinite(config.m_radiation_damping) || config.m_radiation_damping < 0.0f)
			{
				throw std::runtime_error("GpuBuoyancy radiation damping must be a finite, non-negative value");
			}
			if (!std::isfinite(config.m_wave_resistance) || config.m_wave_resistance < 0.0f)
			{
				throw std::runtime_error("GpuBuoyancy wave resistance must be a finite, non-negative value");
			}

			m_config = config;
		}

		// Return the tunable buoyancy parameters currently in effect.
		Config const& GetConfig() const
		{
			return m_config;
		}

		// Consume diagnostic readback data after the physics engine has completed its GPU step.
		void CompleteStep()
		{
			if (m_pending_diagnostics.empty())
			{
				m_pending_readback = {};
				return;
			}

			auto const diagnostic_count = static_cast<int>(m_pending_diagnostics.size());
			auto const diagnostics = std::span{ m_pending_readback.ptr<GpuBuoyancyDiagnostic>(), m_pending_diagnostics.size() };
			{
				auto lock = std::lock_guard<std::mutex>(m_diagnostics_mutex);
				for (int index = 0; index != diagnostic_count; ++index)
				{
					auto const& pending = m_pending_diagnostics[index];
					auto const body_index = pending.m_body_index;
					auto const body_generation = pending.m_body_generation;
					if (body_index < 0 || body_index >= static_cast<int>(m_diagnostics.size()))
					{
						throw std::runtime_error("GPU buoyancy diagnostic readback referred to an unknown body slot");
					}
					if (m_diagnostics[body_index].m_body_generation != body_generation)
					{
						throw std::runtime_error("GPU buoyancy diagnostic readback generation mismatch");
					}

					auto const& gpu_diag = diagnostics[index];
					auto& diag = m_diagnostics[body_index];
					diag.m_volume_m3 = gpu_diag.m_volume_m3;
					diag.m_force_ws = gpu_diag.m_force_ws;
					diag.m_torque_ws = gpu_diag.m_torque_ws;
					diag.m_centre_buoyancy_ws = gpu_diag.m_centre_buoyancy_ws;
					diag.m_valid = gpu_diag.m_valid != 0;
				}
			}

			m_pending_diagnostics.clear();
			m_pending_readback = {};
		}

		// Record the buoyancy force and diagnostic compute work into the active physics GPU job.
		void Apply(Engine&, Engine::ExternalForceArgs const& args)
		{
			m_pending_diagnostics.clear();
			m_pending_readback = {};

			// All buoyancy flows through the sampled-composite surface dispatch.
			DispatchComposite(args);
		}

		// Host-side dry broadphase cull. Returns true when the body's registration-time AABB lies entirely
		// above the water field's maximum height measured along the body's local up (-gravity) axis. The
		// AABB encloses every primitive and a surface cell is only wet while its sample is less than half a cell
		// height above the water, so every sample is guaranteed dry and the body contributes exactly zero buoyancy
		// and drag - bit-identical to the GPU readback (which fast-paths dry boxes/spheres and produces all-dry
		// samples for polytopes and triangles).
		bool IsFullyDry(CompositeShape const& shape_data, BodyState const& bs) const
		{
			// Need a valid body pose and a usable gravity direction to define "up".
			if (!bs.m_valid)
				return false;

			auto const g = bs.m_ws_gravity.w0();
			if (!IsFinite(g) || LengthSq(g) < 1e-12f)
				return false;

			auto const up = -Normalise(g);

			// Lowest extent of the world-space AABB along up: project the centre, then subtract the
			// support distance contributed by each rotated body axis scaled by its half-extent.
			auto const centre_ws = bs.m_o2w * shape_data.m_obb_os.m_centre;
			auto const r = shape_data.m_obb_os.m_radius;
			auto const extent_up =
				Abs(Dot3(bs.m_o2w.x, up)) * r.x +
				Abs(Dot3(bs.m_o2w.y, up)) * r.y +
				Abs(Dot3(bs.m_o2w.z, up)) * r.z;
			auto const lowest = Dot3(centre_ws, up) - extent_up;

			if (!IsFinite(lowest))
				return false;

			// Strict margin: the band [level, level+margin] still produces zero on the GPU (its
			// per-primitive dry fast-path / all-dry samples), so a positive margin is always safe and
			// avoids host/GPU float divergence right at the waterline. The half cell height covers surface
			// samples whose cells are partly wet while their centres are above the surface.
			auto const margin = std::max(shape_data.m_eps, 1e-4f) + shape_data.m_max_half_cell_height;
			return lowest > WaterMaxHeight() + margin;
		}

		// Resolve a registration to its hidden articulation proxy or ordinary rigid-body step index.
		int ResolveStepIndex(CompositeSlot const& slot, int body_index) const
		{
			if (slot.m_articulation != nullptr)
				return m_engine->ArticulationLinkStepIndex(slot.m_articulation->Id(), slot.m_link);

			return m_step_index_resolver(body_index);
		}

		// Return the current collision-shape-root pose and gravity needed by the host-side dry cull.
		BodyState ResolveBodyState(CompositeSlot const& slot, int body_index) const
		{
			if (slot.m_articulation == nullptr)
				return m_body_state_resolver(body_index);

			auto const& desc = slot.m_articulation->LinkDescription(slot.m_link);
			return BodyState{
				.m_o2w = slot.m_articulation->LinkToWorld(slot.m_link) * desc.m_shape_to_link,
				.m_centre_of_mass_os = (InvertOrthonormal(desc.m_shape_to_link) * desc.m_inertia.CoM().w1()).w0(),
				.m_ws_gravity = slot.m_articulation->GravityWS(slot.m_link),
				.m_valid = true,
			};
		}

		// Resolve registered hulls to the current engine body order and collect the compact dispatch population.
		ActiveHullStats CollectActiveHulls(Engine::ExternalForceArgs const& args)
		{
			m_active_hulls.clear();
			m_active_shapes.clear();
			for (auto& bucket : m_active_shape_lookup)
				bucket.m_shape_data = nullptr;

			assert(m_active_hulls.capacity() >= m_composite_hulls.size());
			assert(m_active_shapes.capacity() >= m_composite_hulls.size());

			auto stats = ActiveHullStats{};
			for (auto body_index = 0; body_index != static_cast<int>(m_composite_hulls.size()); ++body_index)
			{
				auto const& slot = m_composite_hulls[body_index];
				if (!slot.m_active || slot.m_shape_data == nullptr)
				{
					continue;
				}

				auto const body_step_index = ResolveStepIndex(slot, body_index);
				if (body_step_index < 0)
				{
					continue;
				}
				if (body_step_index >= args.m_body_count)
				{
					throw std::runtime_error("Buoyancy composite hull resolved to an invalid physics step body index");
				}

				// Dry broadphase cull: if the whole body sits above the highest possible water surface, skip
				// the GPU dispatch entirely and publish a zero diagnostic directly. This is result-preserving
				// (see IsFullyDry) so the readback is identical to dispatching the body.
				{
					auto const bs = ResolveBodyState(slot, body_index);
					if (IsFullyDry(*slot.m_shape_data, bs))
					{
						auto lock = std::lock_guard<std::mutex>(m_diagnostics_mutex);
						if (body_index < static_cast<int>(m_diagnostics.size()))
						{
							auto diag = Diagnostics{};
							diag.m_body_index = body_index;
							diag.m_body_generation = slot.m_generation;
							diag.m_valid = true;
							m_diagnostics[body_index] = diag;
						}
						continue;
					}
				}

				// Multiple bodies can share a collision shape, while representative workloads can also
				// contain thousands of unique shapes. Use allocation-free open addressing so both cases
				// resolve in expected constant time instead of scanning all shapes collected so far.
				auto const* shape_data = slot.m_shape_data.get();
				auto const bucket_mask = m_active_shape_lookup.size() - 1;
				auto bucket_index = (reinterpret_cast<std::uintptr_t>(shape_data) >> 4) & bucket_mask;
				while (m_active_shape_lookup[bucket_index].m_shape_data != nullptr &&
					m_active_shape_lookup[bucket_index].m_shape_data != shape_data)
				{
					bucket_index = (bucket_index + 1) & bucket_mask;
				}

				auto& bucket = m_active_shape_lookup[bucket_index];
				if (bucket.m_shape_data == nullptr)
				{
					bucket.m_shape_data = shape_data;
					bucket.m_shape_index = static_cast<int>(m_active_shapes.size());
					m_active_shapes.push_back(ActiveShape{
						.m_shape_data = slot.m_shape_data,
						.m_prim_base = 0,
						.m_prim_count = 0,
					});
				}
				auto const shape_index = bucket.m_shape_index;

				m_active_hulls.push_back(ActiveHull{
					.m_body_index = body_index,
					.m_body_generation = slot.m_generation,
					.m_body_step_index = body_step_index,
					.m_shape_index = shape_index,
				});
				stats.m_max_surface_samples = std::max(stats.m_max_surface_samples, slot.m_shape_data->m_total_surface_samples);
			}
			return stats;
		}

		// Record surface sampling followed by reduction into the ordinary body force accumulators and the implicit solve.
		void RecordSurfacePass(Engine::ExternalForceArgs const& args, CBufGpuBuoyancy const& cb, DispatchAddresses const& addresses)
		{
			// Evaluate all surface samples into one partial record per hull/threadgroup. Root parameter
			// order must match CreateSurfaceStep: b0, u0(bodies), t1(water field), t3(surf_prims),
			// t6(face_planes), t8(surf_headers), t9(patches), t11(surf_records),
			// u1(partials).
			args.m_job.m_cmd_list.SetPipelineState(m_surface_step.m_pso.get());
			args.m_job.m_cmd_list.SetComputeRootSignature(m_surface_step.m_sig.get());
			args.m_job.m_cmd_list.AddComputeRoot32BitConstants(cb);
			args.m_job.m_cmd_list.AddComputeRootUnorderedAccessView(args.m_bodies);
			args.m_job.m_cmd_list.AddComputeRootShaderResourceView(addresses.m_water_field);
			args.m_job.m_cmd_list.AddComputeRootShaderResourceView(addresses.m_surface_primitives);
			args.m_job.m_cmd_list.AddComputeRootShaderResourceView(addresses.m_face_planes);
			args.m_job.m_cmd_list.AddComputeRootShaderResourceView(addresses.m_surface_headers);
			args.m_job.m_cmd_list.AddComputeRootShaderResourceView(addresses.m_surface_patches);
			args.m_job.m_cmd_list.AddComputeRootShaderResourceView(addresses.m_surface_records);
			args.m_job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_partials);
			args.m_job.m_cmd_list.Dispatch(cb.m_groups_per_hull, cb.m_hull_count, 1);
			args.m_job.m_barriers.UAV(m_r_partials.get()).Commit();

			// Reduce the surface partials into the body and diagnostic. Root parameter order must match
			// CreateSurfaceReduceStep: b0, u0(bodies), t8(surf_headers), u1(partials), u2(diagnostics). Diagnostic
			// output is optional, but its dummy UAV remains bound to keep one root-signature layout.
			args.m_job.m_cmd_list.SetPipelineState(m_surface_reduce_step.m_pso.get());
			args.m_job.m_cmd_list.SetComputeRootSignature(m_surface_reduce_step.m_sig.get());
			args.m_job.m_cmd_list.AddComputeRoot32BitConstants(cb);
			args.m_job.m_cmd_list.AddComputeRootUnorderedAccessView(args.m_bodies);
			args.m_job.m_cmd_list.AddComputeRootShaderResourceView(addresses.m_surface_headers);
			args.m_job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_partials);
			args.m_job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_diagnostics);
			args.m_job.m_cmd_list.Dispatch(cb.m_hull_count, 1, 1);
			args.m_job.m_barriers.UAV(args.m_bodies);
			if (cb.m_enable_diagnostics != 0)
				args.m_job.m_barriers.UAV(m_r_diagnostics.get());

			args.m_job.m_barriers.Commit();
		}

		// Queue the final per-hull diagnostics for consumption after the physics GPU job completes.
		void RecordDiagnosticReadback(Engine::ExternalForceArgs const& args, int hull_count)
		{
			m_pending_readback = args.m_job.m_readback.template Alloc<GpuBuoyancyDiagnostic>(hull_count);
			args.m_job.m_barriers.Transition(m_r_diagnostics.get(), D3D12_RESOURCE_STATE_COPY_SOURCE).Commit();
			args.m_job.m_cmd_list.CopyBufferRegion(m_pending_readback, m_r_diagnostics.get(), 0);
			args.m_job.m_barriers.Transition(m_r_diagnostics.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS).Commit();
		}

		// Prepare transient inputs and record the surface pass and the diagnostic readback.
		void DispatchComposite(Engine::ExternalForceArgs const& args)
		{
			if (args.m_body_count == 0)
			{
				return;
			}

			// Phase 1: compact registrations into body-step order and establish the dispatch dimensions.
			auto const active_stats = CollectActiveHulls(args);
			if (m_active_hulls.empty())
			{
				return;
			}

			auto const hull_count = static_cast<int>(m_active_hulls.size());

			// Groups are laid out [hull0 groups][hull1 groups]..., with a UNIFORM group count per hull
			// (the kernel derives the hull from global_group_index / groups_per_hull). Size it for the
			// busiest hull, capped at one partial per reduce thread; additional samples stream through
			// the same threads so the registered count is never truncated.
			auto const groups_per_hull = std::clamp((active_stats.m_max_surface_samples + BuoyancySampleThreadCount - 1) / BuoyancySampleThreadCount, 1, BuoyancyReduceThreadCount);
			if (hull_count > 65535)
				throw std::runtime_error("Buoyancy hull population exceeds the dispatch dimension");

			EnsureGpuCapacity(
				args.m_job,
				hull_count * groups_per_hull,
				m_config.m_enable_diagnostics ? hull_count : 1);

			// Phase 2: sum geometry sizes across unique active collision shapes. Bodies sharing a shape
			// point at the same primitive block, so immutable data is uploaded only once per dispatch.
			// Each upload uses at least one element because empty arrays still need a valid address.
			auto total_prims = 0;
			auto total_face_planes = 0;
			auto total_patches = uint64_t{0};
			for (auto const& active_shape : m_active_shapes)
			{
				total_prims += static_cast<int>(active_shape.m_shape_data->m_hull.m_primitives.size());
				total_face_planes += static_cast<int>(active_shape.m_shape_data->m_hull.m_face_planes.size());
				for (auto const& plan : active_shape.m_shape_data->m_surface_plans)
					total_patches += plan.m_patches.size();
			}
			if (total_patches > static_cast<uint64_t>(std::numeric_limits<int>::max()) / sizeof(surface::SurfacePatch))
				throw std::runtime_error("Buoyancy surface patch upload exceeds addressable buffer size");

			auto upload_face_planes = args.m_job.m_upload.template Alloc<v4>(std::max(total_face_planes, 1));
			auto upload_surf_headers = args.m_job.m_upload.template Alloc<GpuBuoySurfHeader>(hull_count);
			auto upload_surf_prims = args.m_job.m_upload.template Alloc<buoyancy::GpuPrimitive>(std::max(total_prims, 1));
			auto upload_surf_records = args.m_job.m_upload.template Alloc<GpuBuoySurfPrimRecord>(std::max(total_prims, 1));
			auto upload_patches = args.m_job.m_upload.template Alloc<surface::SurfacePatch>(std::max(static_cast<int>(total_patches), 1));

			auto face_planes = upload_face_planes.ptr<v4>();
			auto surf_headers = upload_surf_headers.ptr<GpuBuoySurfHeader>();
			auto surf_prims = upload_surf_prims.ptr<buoyancy::GpuPrimitive>();
			auto surf_records = upload_surf_records.ptr<GpuBuoySurfPrimRecord>();
			auto patches = upload_patches.ptr<surface::SurfacePatch>();

			// Phase 3: concatenate each unique shape's geometry and sample records into shared buffers.
			// Face-vertex indices remain relative to each primitive's vertex block.
			auto prim_base = 0;
			auto face_base = 0;
			auto patch_base = 0;
			for (auto& active_shape : m_active_shapes)
			{
				auto const& shape_data = *active_shape.m_shape_data;
				auto const& hull = shape_data.m_hull;
				auto const prim_count = static_cast<int>(hull.m_primitives.size());
				active_shape.m_prim_base = prim_base;
				active_shape.m_prim_count = prim_count;

				for (auto i = 0; i != static_cast<int>(hull.m_face_planes.size()); ++i)
					face_planes[face_base + i] = hull.m_face_planes[i];

				for (auto k = 0; k != prim_count; ++k)
				{
					// Copy the primitive descriptor and shift its face offset into the shared face-plane buffer. m_vert_ofs is
					// left unchanged: the surface-vertex array is not bound, so its offset is unused here.
					auto p = hull.m_primitives[k];
					p.m_face_ofs += face_base;
					surf_prims[prim_base + k] = p;

					surf_records[prim_base + k] = GpuBuoySurfPrimRecord{
						.m_count = static_cast<int>(shape_data.m_surface_plans[k].m_count),
						.m_patch_begin = patch_base,
						.m_patch_count = static_cast<int>(shape_data.m_surface_plans[k].m_patches.size()),
						.m_half_cell_height = shape_data.m_half_cell_heights[k],
					};
					for (auto const& patch : shape_data.m_surface_plans[k].m_patches)
						patches[patch_base++] = patch;
				}

				prim_base += prim_count;
				face_base += static_cast<int>(hull.m_face_planes.size());
			}

			// Phase 4: emit per-body headers that reference the shared shape blocks. Body index and
			// generation remain distinct for each registration.
			assert(!m_config.m_enable_diagnostics || m_pending_diagnostics.capacity() >= m_composite_hulls.size());
			for (auto index = 0; index != hull_count; ++index)
			{
				auto const& active_hull = m_active_hulls[index];
				auto const& active_shape = m_active_shapes[active_hull.m_shape_index];
				auto const& shape_data = *active_shape.m_shape_data;

				surf_headers[index] = GpuBuoySurfHeader{
					.m_body_index = active_hull.m_body_step_index,
					.m_prim_base = active_shape.m_prim_base,
					.m_prim_count = active_shape.m_prim_count,
					.m_total_surface_samples = shape_data.m_total_surface_samples,
					.m_eps = shape_data.m_eps,
					.m_pad0 = 0,
					.m_pad1 = 0,
					.m_pad2 = 0,
				};
				if (m_config.m_enable_diagnostics)
				{
					m_pending_diagnostics.push_back(PendingDiagnostic{
						.m_body_index = active_hull.m_body_index,
						.m_body_generation = active_hull.m_body_generation,
					});
				}
			}

			// Phase 5: finish the transient inputs with the active water field. The sample kernel needs
			// t1 bound even for a flat field, so always allocate and zero at least one element.
			auto const water_elements = m_water_field.Elements();
			auto const water_field_count = static_cast<int>(water_elements.size());
			auto const water_field_stride = static_cast<int>(sizeof(terrain::water::WaterFieldElement));
			auto upload_water_field = args.m_job.m_upload.Alloc(std::max(water_field_count, 1) * water_field_stride, 16);
			memset(upload_water_field.ptr<std::byte>(), 0, static_cast<std::size_t>(upload_water_field.m_size));
			memcpy(upload_water_field.ptr<std::byte>(), water_elements.data(), water_elements.size_bytes());

			auto const addresses = DispatchAddresses{
				.m_water_field = upload_water_field,
				.m_face_planes = upload_face_planes,
				.m_surface_headers = upload_surf_headers,
				.m_surface_primitives = upload_surf_prims,
				.m_surface_records = upload_surf_records,
				.m_surface_patches = upload_patches,
			};

			// Translational and rotational damping act on the wet water columns with independent rho/tau coefficients.
			auto const linear_drag_coefficient = m_config.m_linear_drag_time_constant_s > 0.0f
				? m_config.m_fluid_density / m_config.m_linear_drag_time_constant_s
				: 0.0f;
			auto const angular_drag_coefficient = m_config.m_angular_drag_time_constant_s > 0.0f
				? m_config.m_fluid_density / m_config.m_angular_drag_time_constant_s
				: 0.0f;

			auto const cb = CBufGpuBuoyancy{
				.m_hull_count = hull_count,
				.m_groups_per_hull = groups_per_hull,
				.m_water_field_count = water_field_count,
				.m_time_s = static_cast<float>(args.m_time_s),
				.m_water_level = static_cast<float>(m_water_field.Level()),
				.m_water_max_height = WaterMaxHeight(),
				.m_fluid_density = m_config.m_fluid_density,
				.m_linear_drag_coefficient = linear_drag_coefficient,
				.m_angular_drag_coefficient = angular_drag_coefficient,
				.m_quadratic_drag_coefficient = m_config.m_quadratic_drag_coefficient,
				.m_tangential_drag_coefficient = m_config.m_tangential_drag_coefficient,
				.m_added_mass_coefficient = m_config.m_added_mass_coefficient,
				.m_radiation_damping = m_config.m_radiation_damping,
				.m_time_step_s = args.m_dt,
				.m_enable_diagnostics = m_config.m_enable_diagnostics ? 1 : 0,
				.m_wave_resistance = m_config.m_wave_resistance,
				.m_air_density = m_air.m_density,
			};

			// Phase 6: sample the wet surface and reduce it into each body's force/torque accumulators.
			RecordSurfacePass(args, cb, addresses);

			// Diagnostic publication is opt-in and only the final substep is observable after the frame's single wait.
			// Production force application does not allocate or record this diagnostic transfer.
			if (m_config.m_enable_diagnostics && args.m_substep_index + 1 == args.m_substep_count)
				RecordDiagnosticReadback(args, hull_count);
		}

		// Resize GPU buffers used by the buoyancy dispatches. 'partial_capacity' is the number of
		// per-threadgroup partial records the sample pass will write and 'diagnostic_capacity'
		// is the number of per-hull diagnostic records the reduce pass will write.
		void EnsureGpuCapacity(GpuJob& job, int partial_capacity, int diagnostic_capacity)
		{
			if (partial_capacity > m_partial_capacity)
			{
				m_r_partials = CreateDefaultUavBuffer<GpuBuoyancyPartial>(m_device, partial_capacity, "Physics.GpuBuoyancy.Partials");
				m_partial_capacity = partial_capacity;
			}
			if (diagnostic_capacity > m_diagnostic_capacity)
			{
				m_r_diagnostics = CreateDefaultUavBuffer<GpuBuoyancyDiagnostic>(m_device, diagnostic_capacity, "Physics.GpuBuoyancy.Diagnostics");
				m_diagnostic_capacity = diagnostic_capacity;
			}

			// New resources start in COMMON but are tracked with a UAV default state, relying on normal
			// D3D12 buffer promotion for first use. The dummy diagnostic UAV is not accessed in production.
			job.m_barriers.Transition(m_r_partials.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			if (m_config.m_enable_diagnostics)
				job.m_barriers.Transition(m_r_diagnostics.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);

			job.m_barriers.Commit();
		}
	};

	// Construct an invalid buoyancy diagnostic record.
	GpuBuoyancy::Diagnostics::Diagnostics()
		:m_body_index(-1)
		,m_body_generation(-1)
		,m_volume_m3()
		,m_force_ws(v4::Zero())
		,m_centre_buoyancy_ws(v4::Zero())
		,m_torque_ws(v4::Zero())
		,m_valid()
	{
	}

	// Construct an empty registration handle.
	GpuBuoyancy::Registration::Registration()
		:m_owner()
		,m_body_index(-1)
		,m_body_generation(-1)
	{
	}

	// Construct a registration handle for an already registered hull.
	GpuBuoyancy::Registration::Registration(GpuBuoyancy& owner, int body_index, int body_generation)
		:m_owner(&owner)
		,m_body_index(body_index)
		,m_body_generation(body_generation)
	{
	}

	// Move ownership of a buoyancy hull registration.
	GpuBuoyancy::Registration::Registration(Registration&& rhs) noexcept
		:Registration()
	{
		std::swap(m_owner, rhs.m_owner);
		std::swap(m_body_index, rhs.m_body_index);
		std::swap(m_body_generation, rhs.m_body_generation);
	}

	// Replace this handle with another registration, unregistering the current hull first.
	GpuBuoyancy::Registration& GpuBuoyancy::Registration::operator=(Registration&& rhs) noexcept
	{
		if (this == &rhs)
		{
			return *this;
		}

		Reset();
		std::swap(m_owner, rhs.m_owner);
		std::swap(m_body_index, rhs.m_body_index);
		std::swap(m_body_generation, rhs.m_body_generation);
		return *this;
	}

	// Unregister the hull owned by this handle.
	GpuBuoyancy::Registration::~Registration()
	{
		Reset();
	}

	// Release the current hull registration, if any.
	void GpuBuoyancy::Registration::Reset() noexcept
	{
		if (m_owner == nullptr)
		{
			return;
		}

		m_owner->UnregisterHull(m_body_index, m_body_generation);
		m_owner = nullptr;
		m_body_index = -1;
		m_body_generation = -1;
	}

	// Return true when this handle owns a registered hull.
	GpuBuoyancy::Registration::operator bool() const
	{
		return m_owner != nullptr;
	}

	// Construct and subscribe the diagnostic buoyancy pass to a physics engine.
	GpuBuoyancy::GpuBuoyancy(ID3D12Device* device, Engine& engine, Config const& config, StepIndexResolver step_index_resolver, BodyStateResolver body_state_resolver)
		:m_impl(std::make_unique<Impl>(device, engine, config, std::move(step_index_resolver), std::move(body_state_resolver), true))
	{
	}

	// Construct an articulation-only buoyancy pass that resolves link proxies through the engine.
	GpuBuoyancy::GpuBuoyancy(ID3D12Device* device, Engine& engine, Config const& config)
		:m_impl(std::make_unique<Impl>(
			device,
			engine,
			config,
			[](int)
			{
				return -1;
			},
			[](int)
			{
				return BodyState{};
			},
			false))
	{
	}

	// Destroy the buoyancy module after all owned registrations have been released.
	GpuBuoyancy::~GpuBuoyancy()
	{
		#if PR_DBG
		auto const active_composite = std::ranges::any_of(m_impl->m_composite_hulls, [](Impl::CompositeSlot const& slot)
		{
			return slot.m_active;
		});
		PR_ASSERT(PR_DBG, !active_composite, "GpuBuoyancy registrations must be destroyed before the GpuBuoyancy module");
		#endif
	}

	// Return the latest diagnostic record for a registered hull.
	GpuBuoyancy::Diagnostics GpuBuoyancy::LatestDiagnostics(int body_index, int body_generation) const
	{
		return m_impl->LatestDiagnostics(body_index, body_generation);
	}

	// Consume diagnostic readback data after the physics engine has completed its GPU step.
	void GpuBuoyancy::CompleteStep()
	{
		m_impl->CompleteStep();
	}

	// Set the water field used by subsequent buoyancy force dispatches.
	void GpuBuoyancy::SetWaterField(terrain::water::WaterField const& water_field)
	{
		m_impl->SetWaterField(water_field);
	}

	// Return the water field used by buoyancy force dispatches.
	terrain::water::WaterField const& GpuBuoyancy::GetWaterField() const
	{
		return m_impl->GetWaterField();
	}

	// Set the air above the water used by subsequent buoyancy force dispatches.
	void GpuBuoyancy::SetAir(Air const& air)
	{
		m_impl->SetAir(air);
	}

	// Return the air used by buoyancy force dispatches.
	Air const& GpuBuoyancy::GetAir() const
	{
		return m_impl->GetAir();
	}

	// Set the tunable buoyancy parameters used by subsequent dispatches.
	void GpuBuoyancy::SetConfig(Config const& config)
	{
		m_impl->SetConfig(config);
	}

	// Return the tunable buoyancy parameters currently in effect.
	GpuBuoyancy::Config const& GpuBuoyancy::GetConfig() const
	{
		return m_impl->GetConfig();
	}

	// Register a rigid body's collision shape as its buoyancy hull.
	GpuBuoyancy::Registration GpuBuoyancy::RegisterCompositeHull(RigidBody& body, int body_index, int body_generation)
	{
		m_impl->RegisterCompositeHull(body, body_index, body_generation);
		return Registration{*this, body_index, body_generation};
	}

	// Register one articulation link's immutable collision shape as a buoyancy hull.
	GpuBuoyancy::Registration GpuBuoyancy::RegisterCompositeHull(Articulation& articulation, LinkHandle link, int body_index, int body_generation)
	{
		m_impl->RegisterCompositeHull(articulation, link, body_index, body_generation);
		return Registration{*this, body_index, body_generation};
	}

	// Remove the buoyancy hull for a stable physics body index.
	void GpuBuoyancy::UnregisterHull(int body_index, int body_generation) noexcept
	{
		if (m_impl == nullptr)
		{
			return;
		}

		m_impl->UnregisterHull(body_index, body_generation);
	}
}
