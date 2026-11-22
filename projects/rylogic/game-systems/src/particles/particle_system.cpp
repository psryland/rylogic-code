//************************************
// Game Systems
//  Copyright (c) Rylogic Ltd 2026
//************************************
#include "pr/game-systems/forward.h"
#include "pr/game-systems/particles/particle_system.h"
#include "pr/game-systems/particles/particles.hlsli"
#include "pr/game-systems/particles/particle_sprite.hlsli"
#include "pr/game-systems/particles/particle_mesh.hlsli"
#include "pr/game-systems/particles/particle_environment.hlsli"
#include "pr/game-systems/particles/particle_stepped_sprite.hlsli"
#include "pr/game-systems/particles/particle_bird.hlsli"
#include "pr/game-systems/ocean/ocean_surface.h"
#include "pr/game-systems/ocean/ocean_surface.hlsli"

#ifdef NDEBUG
#define PR_GAME_SYSTEMS_SHADER_COMPILED_DIR(file) PR_STRINGISE(game-systems/src/shaders/hlsl/compiled/release/##file)
#else
#define PR_GAME_SYSTEMS_SHADER_COMPILED_DIR(file) PR_STRINGISE(game-systems/src/shaders/hlsl/compiled/debug/##file)
#endif

namespace pr::game_systems::particles
{
	// The particle kernels are recorded with the compute library's command list and resource helpers.
	using GpuUploadBuffer = ::pr::compute::GpuUploadBuffer;
	using GfxCmdList = ::pr::compute::GfxCmdList;
	using BarrierBatch = ::pr::compute::BarrierBatch;
	using ComputePSO = ::pr::compute::ComputePSO;
	using RootSig = ::pr::compute::RootSig;
	using ERootSigFlags = ::pr::compute::ERootSigFlags;
	using EUsage = ::pr::compute::EUsage;
	using ::pr::compute::DefaultResState;
	using ::pr::compute::DebugName;
	namespace pix = ::pr::compute::pix;

	namespace compiled
	{
		// The particle system's build-time shader bytecode
		#include PR_GAME_SYSTEMS_SHADER_COMPILED_DIR(particle_reset_cs.h)
		#include PR_GAME_SYSTEMS_SHADER_COMPILED_DIR(particle_finalize_cs.h)
		#include PR_GAME_SYSTEMS_SHADER_COMPILED_DIR(particle_sprite_emit_cs.h)
		#include PR_GAME_SYSTEMS_SHADER_COMPILED_DIR(particle_sprite_update_cs.h)
		#include PR_GAME_SYSTEMS_SHADER_COMPILED_DIR(particle_stepped_sprite_emit_cs.h)
		#include PR_GAME_SYSTEMS_SHADER_COMPILED_DIR(particle_stepped_sprite_update_cs.h)
		#include PR_GAME_SYSTEMS_SHADER_COMPILED_DIR(particle_sprite_vs.h)
		#include PR_GAME_SYSTEMS_SHADER_COMPILED_DIR(particle_mesh_vs.h)
		#include PR_GAME_SYSTEMS_SHADER_COMPILED_DIR(particle_debris_emit_cs.h)
		#include PR_GAME_SYSTEMS_SHADER_COMPILED_DIR(particle_debris_update_cs.h)
		#include PR_GAME_SYSTEMS_SHADER_COMPILED_DIR(particle_bird_emit_cs.h)
		#include PR_GAME_SYSTEMS_SHADER_COMPILED_DIR(particle_bird_update_cs.h)
		#include PR_GAME_SYSTEMS_SHADER_COMPILED_DIR(particle_bird_vs.h)
		#include PR_GAME_SYSTEMS_SHADER_COMPILED_DIR(particle_marker_emit_cs.h)
		#include PR_GAME_SYSTEMS_SHADER_COMPILED_DIR(particle_marker_update_cs.h)
		#include PR_GAME_SYSTEMS_SHADER_COMPILED_DIR(particle_spray_emit_cs.h)
		#include PR_GAME_SYSTEMS_SHADER_COMPILED_DIR(particle_spray_update_cs.h)
		#include PR_GAME_SYSTEMS_SHADER_COMPILED_DIR(particle_froth_emit_cs.h)
		#include PR_GAME_SYSTEMS_SHADER_COMPILED_DIR(particle_froth_update_cs.h)
	}

	namespace
	{
		// Root parameter indices of the signature shared by every particle kernel (see particle_kernels.hlsli for the registers).
		enum ERootParam
		{
			RootTick = 0,      // b0
			RootType = 1,      // b1
			RootCounters = 2,  // u0
			RootFree = 3,      // u1
			RootLive = 4,      // u2
			RootStates = 5,    // u3
			RootRecords = 6,   // u4
			RootRuntime = 7,   // u5
			RootArgs = 8,      // u6
			RootDrawArgs = 9,  // u7
			RootHeaders = 10,  // t0
			RootParams = 11,   // t1
			RootBounds = 12,   // u8
			RootEnvironment = 13, // b2
			RootPlanes = 14,      // t2
			RootSets = 15,        // t3
			RootWind = 16,        // t4
			RootOcean = 17,       // b3
			RootOceanBuffer = 18, // t5
		};

		// Limits of the environment. Normals must be unit length because the collision distances are measured along them.
		constexpr int MaxCollisionIterations = 64;
		constexpr float PlaneNormalTolerance = 1e-3f;

		// Limits of the type and batch options. The capacity is bounded so that the reset dispatch fits in one dimension of thread groups.
		// Type constants are bounded by the largest constant buffer view.
		constexpr int MaxCapacity = 65535 * static_cast<int>(ParticleGroupSize);
		constexpr int MaxEmitters = 65535;
		constexpr int MaxStride = 2048;
		constexpr int MaxTypes = 256;
		constexpr size_t MaxTypeConstantsSize = D3D12_REQ_CONSTANT_BUFFER_ELEMENT_COUNT * 16;

		// The kernels write the indirect argument records as words at these indices, so they must match D3D12's layouts.
		static_assert(ParticleArgsDispatchX == offsetof(D3D12_DISPATCH_ARGUMENTS, ThreadGroupCountX) / 4 && sizeof(D3D12_DISPATCH_ARGUMENTS) <= ParticleArgsCount * 4);
		static_assert(ParticleDrawIndexCount == offsetof(D3D12_DRAW_INDEXED_ARGUMENTS, IndexCountPerInstance) / 4);
		static_assert(ParticleDrawInstanceCount == offsetof(D3D12_DRAW_INDEXED_ARGUMENTS, InstanceCount) / 4);
		static_assert(sizeof(D3D12_DRAW_INDEXED_ARGUMENTS) == ParticleDrawArgsCount * 4);

		// Emitter ids pack the slot generation, the type, and the emitter slot. Generation zero is never used, so zero is never a valid id.
		constexpr int EmitterSlotBits = 24;
		constexpr int EmitterTypeShift = EmitterSlotBits;
		constexpr int EmitterGenerationShift = 32;

		// The stock sprite draws an octagon as a fan of triangles around the centre vertex. See particle_sprite.hlsli.
		constexpr auto SpriteIndices = []
		{
			// Each rim edge forms one triangle with the centre.
			auto indices = std::array<uint32_t, ParticleSpriteIndexCount>{};
			for (uint32_t i = 0; i != ParticleSpriteRimCount; ++i)
			{
				indices[3 * i + 0] = ParticleSpriteRimCount;
				indices[3 * i + 1] = i;
				indices[3 * i + 2] = (i + 1) % ParticleSpriteRimCount;
			}
			return indices;
		}();

		// The bird draws seven triangles of its logical vertices (see particle_bird_vs.hlsl): two per wing root, one per wing tip, and the tail.
		constexpr auto BirdIndices = []
		{
			// Each triangle is listed in both windings, so the bird is seen from above and below.
			constexpr uint32_t triangles[7][3] = { {0, 2, 3}, {0, 3, 1}, {2, 4, 3}, {0, 6, 5}, {0, 1, 6}, {5, 6, 7}, {1, 8, 9} };
			auto indices = std::array<uint32_t, ParticleBirdIndexCount>{};
			for (int i = 0; i != 7; ++i)
			{
				indices[6 * i + 0] = triangles[i][0];
				indices[6 * i + 1] = triangles[i][1];
				indices[6 * i + 2] = triangles[i][2];
				indices[6 * i + 3] = triangles[i][0];
				indices[6 * i + 4] = triangles[i][2];
				indices[6 * i + 5] = triangles[i][1];
			}
			return indices;
		}();

		// Throw when a COM operation fails.
		void CheckHr(HRESULT hr, char const* message)
		{
			if (FAILED(hr))
				throw std::runtime_error(message);
		}

		// Require 'stride' to be a whole number of 4-byte words within the supported range.
		void RequireStride(int stride, char const* message)
		{
			if (stride < 4 || stride > MaxStride || stride % 4 != 0)
				throw std::invalid_argument(message);
		}

		// Require compiled shader bytecode to be present.
		void RequireBytecode(pr::view3d::ShaderByteCode const& code, char const* message)
		{
			if (code.m_bytecode == nullptr || code.m_size == 0)
				throw std::invalid_argument(message);
		}

		// View a byte-code descriptor as a span.
		std::span<uint8_t const> Bytecode(pr::view3d::ShaderByteCode const& code)
		{
			return std::span<uint8_t const>(static_cast<uint8_t const*>(code.m_bytecode), code.m_size);
		}

		// Create a compute pipeline for a caller-supplied kernel, naming the kernel if D3D12 rejects it (e.g. it is not a compute shader, or does not fit 'sig').
		D3DPtr<ID3D12PipelineState> CreateKernel(ID3D12Device* device, ID3D12RootSignature* sig, pr::view3d::ShaderByteCode const& code, std::string const& name)
		{
			// Rethrow with the kernel's name, which the device's own error lacks.
			try
			{
				return ComputePSO(sig, Bytecode(code)).Create(device, name.c_str());
			}
			catch (std::exception const& ex)
			{
				throw std::invalid_argument(std::format("Particle kernel '{}' is invalid: {}", name, ex.what()));
			}
		}

		// Copy 'data' into a new constant-buffer allocation of the upload buffer.
		GpuAddress UploadConstants(GpuUploadBuffer& upload, void const* data, size_t size)
		{
			// Constant buffer views read whole 256-byte blocks, so the allocation is padded to a block.
			auto const block = static_cast<size_t>(D3D12_CONSTANT_BUFFER_DATA_PLACEMENT_ALIGNMENT);
			auto alloc = upload.Alloc(static_cast<int64_t>((size + block - 1) / block * block), D3D12_CONSTANT_BUFFER_DATA_PLACEMENT_ALIGNMENT);
			std::memcpy(alloc.m_mem + alloc.m_ofs, data, size);
			return GpuAddress(alloc);
		}
	}

	// One particle batch: the render records of its types' live particles and the draw arguments of its View3D object.
	struct ParticleBatch
	{
		std::string m_name;
		uint32_t m_capacity;
		int m_record_stride;
		int64_t m_pool_capacity; // Sum of the capacities of the batch's types, which must not exceed 'm_capacity'.

		// The GPU buffers (see particles.hlsli). All are read by the draw outside a step.
		D3DPtr<ID3D12Resource> m_records;
		D3DPtr<ID3D12Resource> m_draw_args;
		D3DPtr<ID3D12Resource> m_bounds;
		D3DPtr<ID3D12Resource> m_vertex_data; // The draw's optional per-vertex data. Null when it has none.
	};

	// One particle type: its pool of GPU buffers, its kernels, and the CPU copy of its emitter slots.
	struct ParticleType
	{
		// The fixed layout of the type.
		std::string m_name;
		ParticleBatch* m_batch;
		uint32_t m_capacity;
		int m_max_emitters;
		int m_params_stride;
		std::vector<std::byte> m_constants; // The type's kernel constants.
		D3DPtr<ID3D12PipelineState> m_emit;
		D3DPtr<ID3D12PipelineState> m_update;

		// The GPU pool (see particles.hlsli).
		D3DPtr<ID3D12Resource> m_counters;
		D3DPtr<ID3D12Resource> m_free;
		D3DPtr<ID3D12Resource> m_live;
		D3DPtr<ID3D12Resource> m_states;
		D3DPtr<ID3D12Resource> m_runtime;
		D3DPtr<ID3D12Resource> m_args;
		D3DPtr<ID3D12Resource> m_headers;
		D3DPtr<ID3D12Resource> m_params;

		// The CPU copy of the emitter slots. The GPU copy is updated from the dirty range in the next step.
		std::vector<ParticleEmitterHeader> m_emitter_headers;
		std::vector<std::byte> m_emitter_params;
		std::vector<uint8_t> m_in_use;
		int m_emitter_hwm;
		int m_dirty_lo;
		int m_dirty_hi;

		// The live list read by the next step.
		uint32_t m_cur;

		// Mark emitter slot 'slot' as changed.
		void Dirty(int slot)
		{
			// The upload covers one contiguous range of slots, which is small because emitters change rarely.
			m_dirty_lo = std::min(m_dirty_lo, slot);
			m_dirty_hi = std::max(m_dirty_hi, slot + 1);
		}
	};

	// The renderer-side particle system.
	class ParticleSystemImpl final : public ParticleSystem
	{
		Renderer& m_rdr;
		std::string m_name;
		ResourceFactory m_factory;
		D3DPtr<ID3D12RootSignature> m_sig;
		D3DPtr<ID3D12PipelineState> m_reset;
		D3DPtr<ID3D12PipelineState> m_finalize;
		std::vector<std::unique_ptr<ParticleBatch>> m_batches;
		std::vector<std::unique_ptr<ParticleType>> m_types;
		uint32_t m_tick;

		// The environment of stepped types. The CPU copies of the planes and wind are uploaded in the next step after they change.
		ParticleEnvironmentConstants m_env;
		std::vector<std::vector<ParticleCollisionPlane>> m_sets;
		std::vector<uint8_t> m_set_deleted;
		std::vector<v2> m_wind_data;
		bool m_sets_dirty;
		bool m_wind_dirty;
		D3DPtr<ID3D12Resource> m_planes;
		D3DPtr<ID3D12Resource> m_set_ranges;
		D3DPtr<ID3D12Resource> m_wind;
		int64_t m_planes_capacity;
		int64_t m_set_ranges_capacity;
		int64_t m_wind_capacity;

		// The bound water surface. Its constants are copied at each step, and its procedural buffer is shared, so particles sample the frame being drawn.
		// A one-element dummy buffer and zeroed constants stand in when there is no water.
		ocean::OceanSurface const* m_ocean;
		pr::view3d::Object m_ocean_object;
		ocean::OceanConstants m_ocean_constants;
		D3DPtr<ID3D12Resource> m_ocean_buffer;
		D3DPtr<ID3D12Resource> m_water_dummy;

	public:

		ParticleSystemImpl(Renderer& rdr, char const* name)
			: m_rdr(rdr)
			, m_name(name != nullptr ? name : "Particles")
			, m_factory(rdr, EResourceFactoryLifetime::Retained)
			, m_sig()
			, m_reset()
			, m_finalize()
			, m_batches()
			, m_types()
			, m_tick()
			, m_env(ParticleEnvironmentConstants{ .collision_iterations = 4 })
			, m_sets()
			, m_set_deleted()
			, m_wind_data()
			, m_sets_dirty()
			, m_wind_dirty()
			, m_planes()
			, m_set_ranges()
			, m_wind()
			, m_planes_capacity()
			, m_set_ranges_capacity()
			, m_wind_capacity()
			, m_ocean()
			, m_ocean_object()
			, m_ocean_constants()
			, m_ocean_buffer()
			, m_water_dummy()
		{
			// Every particle kernel shares one signature of root views, so the step binds each pool once per type.
			m_sig = RootSig(ERootSigFlags::ComputeOnly)
				.CBuf(hlsl::ECBufReg::b0)
				.CBuf(hlsl::ECBufReg::b1)
				.UAV(hlsl::EUAVReg::u0)
				.UAV(hlsl::EUAVReg::u1)
				.UAV(hlsl::EUAVReg::u2)
				.UAV(hlsl::EUAVReg::u3)
				.UAV(hlsl::EUAVReg::u4)
				.UAV(hlsl::EUAVReg::u5)
				.UAV(hlsl::EUAVReg::u6)
				.UAV(hlsl::EUAVReg::u7)
				.SRV(hlsl::ESRVReg::t0)
				.SRV(hlsl::ESRVReg::t1)
				.UAV(hlsl::EUAVReg::u8)
				.CBuf(hlsl::ECBufReg::b2)
				.SRV(hlsl::ESRVReg::t2)
				.SRV(hlsl::ESRVReg::t3)
				.SRV(hlsl::ESRVReg::t4)
				.CBuf(hlsl::ECBufReg::b3)
				.SRV(hlsl::ESRVReg::t5)
				.Create(m_rdr.d3d(), "Particles:Sig");
			m_reset = ComputePSO(m_sig.get(), compiled::particle_reset_cs).Create(m_rdr.d3d(), "Particles:Reset");
			m_finalize = ComputePSO(m_sig.get(), compiled::particle_finalize_cs).Create(m_rdr.d3d(), "Particles:Finalize");

			// The environment's buffers always exist, so every dispatch can bind them. A one-element buffer stands in for an empty table.
			m_planes = CreateEnvironmentBuffer(1, sizeof(ParticleCollisionPlane), {}, "Particles:Planes");
			m_set_ranges = CreateEnvironmentBuffer(1, sizeof(uint32_t) * 2, {}, "Particles:CollisionSets");
			m_wind = CreateEnvironmentBuffer(1, sizeof(v2), {}, "Particles:Wind");
			m_water_dummy = CreateEnvironmentBuffer(1, sizeof(uint32_t), {}, "Particles:WaterDummy");
			m_planes_capacity = 1;
			m_set_ranges_capacity = 1;
			m_wind_capacity = 1;
			m_factory.FlushToGpu(EGpuFlush::Async);
		}
		~ParticleSystemImpl()
		{
			// The GPU may still be running the last step, so release through the renderer's deferred release.
			for (auto& type : m_types)
			{
				for (auto const* res : { &type->m_counters, &type->m_free, &type->m_live, &type->m_states, &type->m_runtime, &type->m_args, &type->m_headers, &type->m_params })
					m_rdr.DeferRelease(*res);

				m_rdr.DeferRelease(type->m_emit);
				m_rdr.DeferRelease(type->m_update);
			}
			for (auto& batch : m_batches)
			{
				// The batches' objects keep their own references, so they can keep drawing the last step's records.
				m_rdr.DeferRelease(batch->m_records);
				m_rdr.DeferRelease(batch->m_draw_args);
				m_rdr.DeferRelease(batch->m_bounds);
				if (batch->m_vertex_data != nullptr)
					m_rdr.DeferRelease(batch->m_vertex_data);
			}
			m_rdr.DeferRelease(m_reset);
			m_rdr.DeferRelease(m_finalize);
			m_rdr.DeferRelease(m_planes);
			m_rdr.DeferRelease(m_set_ranges);
			m_rdr.DeferRelease(m_wind);
			m_rdr.DeferRelease(m_water_dummy);
			m_rdr.DeferRelease(m_ocean_buffer);
			m_rdr.DeferRelease(m_sig);
		}

		// Validate 'options' and add a particle batch.
		int BatchCreate(pr::view3d::ParticleBatchOptions const& options, char const* name, pr::view3d::Object& out_object) override
		{
			// Validate the options against the documented domain.
			if (options.m_struct_size != sizeof(pr::view3d::ParticleBatchOptions) || options.m_version != pr::view3d::ParticleBatchOptions::CurrentVersion)
				throw std::invalid_argument("Particle batch options have an unsupported size or version");
			if (options.m_capacity < 1 || options.m_capacity > MaxCapacity)
				throw std::invalid_argument("Particle batch capacity is out of range");

			RequireStride(options.m_record_stride, "Particle record stride must be a multiple of 4 bytes in [4, 2048]");
			if (static_cast<size_t>(options.m_capacity) * options.m_record_stride > pr::view3d::ProceduralBinding::MaxBufferSize)
				throw std::invalid_argument("Particle batch records exceed the procedural buffer size");
			if (options.m_constants == nullptr ? options.m_constants_size != 0 : (options.m_constants_size < 4 || options.m_constants_size > pr::view3d::ProceduralBinding::ConstantsSize || options.m_constants_size % 4 != 0))
				throw std::invalid_argument("Particle batch constants must be absent, or a multiple of 4 bytes within the procedural constants size");

			auto const& shaders = options.m_shaders;
			RequireBytecode(shaders.m_vertex, "Particle vertex shader is required");
			if (shaders.m_indices == nullptr || shaders.m_index_count < 3 || shaders.m_index_count % 3 != 0 || shaders.m_vertex_count < 1)
				throw std::invalid_argument("Particle indices must be a non-empty triangle list over a positive vertex count");
			if (std::any_of(shaders.m_indices, shaders.m_indices + shaders.m_index_count, [&](uint32_t i) { return i >= static_cast<uint32_t>(shaders.m_vertex_count); }))
				throw std::invalid_argument("Particle indices must be less than the vertex count");
			if (!AllSet(shaders.m_geom, pr::view3d::EGeom::Vert))
				throw std::invalid_argument("Particle vertex shader outputs must include positions");
			if (shaders.m_vertex_data != nullptr)
				RequireStride(shaders.m_vertex_stride, "Particle vertex data stride must be a multiple of 4 bytes in [4, 2048]");
			else if (shaders.m_vertex_stride != 0)
				throw std::invalid_argument("Particle vertex data stride must be zero when there is no vertex data");

			if (static_cast<size_t>(shaders.m_vertex_count) * shaders.m_vertex_stride > pr::view3d::ProceduralBinding::MaxBufferSize)
				throw std::invalid_argument("Particle vertex data exceeds the procedural buffer size");

			// Create the records, the draw arguments, and the record bounds. They rest in the states that the draw and the renderer's cull read them in. The instance count starts at
			// zero and is zeroed again at the start of every step and reset, before the batch's types add their records.
			auto batch_name = std::string(name != nullptr ? name : "ParticleBatch");
			auto batch = std::make_unique<ParticleBatch>();
			batch->m_name = batch_name;
			batch->m_capacity = static_cast<uint32_t>(options.m_capacity);
			batch->m_record_stride = options.m_record_stride;
			batch->m_pool_capacity = 0;
			auto const draw_args = std::array<uint32_t, ParticleDrawArgsCount>{ static_cast<uint32_t>(shaders.m_index_count), 0, 0, 0, 0 };
			auto const none = std::span<std::byte const>{};
			batch->m_records = m_factory.CreateResource(ResDesc::Buf(options.m_capacity, options.m_record_stride, none, 4).usage(EUsage::UnorderedAccess).def_state(D3D12_RESOURCE_STATE_ALL_SHADER_RESOURCE), batch_name + ":Records");
			batch->m_draw_args = m_factory.CreateResource(ResDesc::Buf<uint32_t>(ParticleDrawArgsCount, std::span<uint32_t const>(draw_args)).usage(EUsage::UnorderedAccess).def_state(D3D12_RESOURCE_STATE_INDIRECT_ARGUMENT | D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE), batch_name + ":DrawArgs");
			batch->m_bounds = m_factory.CreateResource(ResDesc::Buf(options.m_capacity, sizeof(v4), none, 16).usage(EUsage::UnorderedAccess).def_state(D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE), batch_name + ":Bounds");
			if (shaders.m_vertex_data != nullptr)
			{
				// The per-vertex data never changes, so it is copied to the GPU once and only read by the draw.
				auto const data = std::span<std::byte const>(static_cast<std::byte const*>(shaders.m_vertex_data), static_cast<size_t>(shaders.m_vertex_count) * shaders.m_vertex_stride);
				batch->m_vertex_data = m_factory.CreateResource(ResDesc::Buf(shaders.m_vertex_count, shaders.m_vertex_stride, data, 4).def_state(D3D12_RESOURCE_STATE_ALL_SHADER_RESOURCE), batch_name + ":VertexData");
			}
			m_factory.FlushToGpu(EGpuFlush::Async);

			// Create the batch's object: one instance per record, drawn by one indirect draw whose instance count the GPU writes.
			out_object = CreateObject(*batch, options, batch_name.c_str());
			m_batches.push_back(std::move(batch));
			return static_cast<int>(m_batches.size() - 1);
		}

		// Validate 'options' and add a particle type.
		int TypeCreate(pr::view3d::ParticleTypeOptions const& options, char const* name) override
		{
			// Validate the options against the documented domain.
			if (options.m_struct_size != sizeof(pr::view3d::ParticleTypeOptions) || options.m_version != pr::view3d::ParticleTypeOptions::CurrentVersion)
				throw std::invalid_argument("Particle type options have an unsupported size or version");
			if (std::ssize(m_types) == MaxTypes)
				throw std::invalid_argument("Too many particle types");
			if (options.m_batch < 0 || options.m_batch >= std::ssize(m_batches))
				throw std::invalid_argument("Particle type batch index is invalid");
			if (options.m_capacity < 1 || options.m_capacity > MaxCapacity)
				throw std::invalid_argument("Particle type capacity is out of range");
			if (options.m_max_emitters < 1 || options.m_max_emitters > MaxEmitters)
				throw std::invalid_argument("Particle type emitter count is out of range");

			// Every live particle of the batch's types needs a record, so their capacities must fit in the batch.
			auto& batch = *m_batches[options.m_batch];
			if (options.m_record_stride != batch.m_record_stride)
				throw std::invalid_argument("Particle type record stride must equal its batch's record stride");
			if (batch.m_pool_capacity + options.m_capacity > batch.m_capacity)
				throw std::invalid_argument("Particle type capacities exceed their batch's capacity");

			RequireStride(options.m_state_stride, "Particle state stride must be a multiple of 4 bytes in [4, 2048]");
			RequireStride(options.m_params_stride, "Particle emitter parameter stride must be a multiple of 4 bytes in [4, 2048]");
			if (options.m_constants == nullptr || options.m_constants_size < 4 || options.m_constants_size > MaxTypeConstantsSize || options.m_constants_size % 4 != 0)
				throw std::invalid_argument("Particle type constants must be a multiple of 4 bytes in [4, 65536]");

			auto const& shaders = options.m_shaders;
			RequireBytecode(shaders.m_emit, "Particle emit shader is required");
			RequireBytecode(shaders.m_update, "Particle update shader is required");

			// Record the layout and create the type's kernels.
			auto type_name = std::string(name != nullptr ? name : "ParticleType");
			auto type = std::make_unique<ParticleType>();
			type->m_name = type_name;
			type->m_batch = &batch;
			type->m_capacity = static_cast<uint32_t>(options.m_capacity);
			type->m_max_emitters = options.m_max_emitters;
			type->m_params_stride = options.m_params_stride;
			type->m_constants.assign(static_cast<std::byte const*>(options.m_constants), static_cast<std::byte const*>(options.m_constants) + options.m_constants_size);
			type->m_emit = CreateKernel(m_rdr.d3d(), m_sig.get(), shaders.m_emit, type_name + ":Emit");
			type->m_update = CreateKernel(m_rdr.d3d(), m_sig.get(), shaders.m_update, type_name + ":Update");
			type->m_emitter_headers.assign(options.m_max_emitters, ParticleEmitterHeader{});
			type->m_emitter_params.assign(static_cast<size_t>(options.m_max_emitters) * options.m_params_stride, std::byte{});
			type->m_in_use.assign(options.m_max_emitters, uint8_t{});
			type->m_emitter_hwm = 0;
			type->m_dirty_lo = options.m_max_emitters;
			type->m_dirty_hi = 0;
			type->m_cur = 0;

			// Create the pool. The args rest as indirect arguments for the update dispatch. The reset kernel fills the free stack and counters
			// before the first step.
			auto const capacity = static_cast<int64_t>(options.m_capacity);
			auto const max_emitters = static_cast<int64_t>(options.m_max_emitters);
			auto const dispatch_args = std::array<uint32_t, ParticleArgsCount>{ 0, 1, 1, 0 };
			auto const none = std::span<std::byte const>{};
			auto const uav = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;
			auto const srv = D3D12_RESOURCE_STATE_ALL_SHADER_RESOURCE;
			type->m_counters = m_factory.CreateResource(ResDesc::Buf(ParticleCounterCount, 4, none, 4).usage(EUsage::UnorderedAccess).def_state(uav), type_name + ":Counters");
			type->m_free = m_factory.CreateResource(ResDesc::Buf(capacity, 4, none, 4).usage(EUsage::UnorderedAccess).def_state(uav), type_name + ":Free");
			type->m_live = m_factory.CreateResource(ResDesc::Buf(2 * capacity, 4, none, 4).usage(EUsage::UnorderedAccess).def_state(uav), type_name + ":Live");
			type->m_states = m_factory.CreateResource(ResDesc::Buf(capacity, options.m_state_stride, none, 4).usage(EUsage::UnorderedAccess).def_state(uav), type_name + ":States");
			type->m_runtime = m_factory.CreateResource(ResDesc::Buf(max_emitters, sizeof(ParticleEmitterRuntime), none, 4).usage(EUsage::UnorderedAccess).def_state(uav), type_name + ":Runtime");
			type->m_args = m_factory.CreateResource(ResDesc::Buf<uint32_t>(ParticleArgsCount, std::span<uint32_t const>(dispatch_args)).usage(EUsage::UnorderedAccess).def_state(D3D12_RESOURCE_STATE_INDIRECT_ARGUMENT), type_name + ":Args");
			type->m_headers = m_factory.CreateResource(ResDesc::Buf(max_emitters, sizeof(ParticleEmitterHeader), none, 4).def_state(srv), type_name + ":Headers");
			type->m_params = m_factory.CreateResource(ResDesc::Buf(max_emitters, options.m_params_stride, none, 4).def_state(srv), type_name + ":Params");
			RecordReset(*type);

			// Upload every emitter slot in the first step, so the GPU copy starts as inactive slots rather than uninitialised memory.
			type->Dirty(0);
			type->Dirty(options.m_max_emitters - 1);
			m_factory.FlushToGpu(EGpuFlush::Async);

			// The pool starts empty, so the batch's records are unchanged until the next step.
			batch.m_pool_capacity += options.m_capacity;
			m_types.push_back(std::move(type));
			return static_cast<int>(m_types.size() - 1);
		}

		// Add an emitter of particles of 'type'.
		uint64_t EmitterCreate(int type_index, pr::view3d::ParticleEmitter const& emitter, std::span<std::byte const> params) override
		{
			// Take the lowest unused slot, so the emit dispatch covers as few slots as possible.
			auto& type = Type(type_index);
			ValidateEmitter(type, emitter, params);
			auto it = std::find(type.m_in_use.begin(), type.m_in_use.end(), uint8_t{});
			if (it == type.m_in_use.end())
				throw std::runtime_error("No free emitter slots for particle type " + type.m_name);

			// A new generation makes the GPU restart the slot's emission progress, and makes ids of earlier emitters in the slot stale.
			auto const slot = static_cast<int>(it - type.m_in_use.begin());
			auto& header = type.m_emitter_headers[slot];
			header.generation = std::max(header.generation + 1, 1u);
			header.burst_total = 0;
			*it = 1;
			type.m_emitter_hwm = std::max(type.m_emitter_hwm, slot + 1);
			Store(type, slot, emitter, params);
			return (static_cast<uint64_t>(header.generation) << EmitterGenerationShift) | (static_cast<uint64_t>(type_index) << EmitterTypeShift) | static_cast<uint64_t>(slot);
		}

		// Replace the settings and parameters of an emitter.
		void EmitterUpdate(uint64_t id, pr::view3d::ParticleEmitter const& settings, std::span<std::byte const> params) override
		{
			// Bursts already requested are kept.
			auto [type, slot] = Emitter(id);
			ValidateEmitter(*type, settings, params);
			Store(*type, slot, settings, params);
		}

		// Ask an emitter for extra particles.
		void EmitterBurst(uint64_t id, uint32_t count) override
		{
			// The GPU emits the difference between the requested total and what it has emitted, so bursts between steps accumulate.
			auto [type, slot] = Emitter(id);
			type->m_emitter_headers[slot].burst_total += count;
			type->Dirty(slot);
		}

		// Remove an emitter.
		void EmitterDelete(uint64_t id) override
		{
			// An inactive header stops emission. The generation is kept so the next emitter in the slot gets a new one.
			auto [type, slot] = Emitter(id);
			type->m_emitter_headers[slot].flags = 0;
			type->m_in_use[slot] = 0;
			type->Dirty(slot);

			// Shrink the emit dispatch past trailing unused slots.
			while (type->m_emitter_hwm != 0 && type->m_in_use[type->m_emitter_hwm - 1] == 0)
				--type->m_emitter_hwm;
		}

		// Advance every particle by 'dt' simulated seconds.
		void Step(double dt) override
		{
			// A paused step does no work, so paused particles stay exactly as they are.
			if (!std::isfinite(dt) || dt < 0.0)
				throw std::invalid_argument("Particle step must be finite and non-negative");
			if (dt == 0.0 || m_types.empty())
				return;

			{
				// The batches collect their types' records afresh each tick, so their record counts start at zero.
				auto& cmd_list = m_factory.CmdList();
				auto pix = pix::EventScope<ID3D12GraphicsCommandList>(cmd_list.get(), 0xFFFFA040, "Particles::Step");
				UploadEnvironment();
				UpdateWater();
				BeginBatches();
				for (auto& type : m_types)
					RecordTick(*type, static_cast<float>(dt));

				EndBatches();
			}
			++m_tick;
			m_factory.FlushToGpu(EGpuFlush::Async);
		}

		// Remove every particle and restart every emitter's emission.
		void Reset() override
		{
			// Requested bursts are discarded with the particles, because the GPU's record of emitted bursts restarts from zero.
			for (auto& type : m_types)
			{
				for (auto& header : type->m_emitter_headers)
					header.burst_total = 0;

				type->Dirty(0);
				type->Dirty(type->m_max_emitters - 1);
				RecordReset(*type);
			}

			// No particle is live, so no batch draws any record.
			BeginBatches();
			EndBatches();
			m_factory.FlushToGpu(EGpuFlush::Async);
		}

		// Read the pool usage of a type.
		pr::view3d::ParticleStats Stats(int type_index) override
		{
			// Copy the counters to a readback buffer and wait for them, which is acceptable for tests and diagnostics only.
			auto& type = Type(type_index);
			auto heap = D3D12_HEAP_PROPERTIES{ .Type = D3D12_HEAP_TYPE_READBACK };
			auto desc = D3D12_RESOURCE_DESC{
				.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER,
				.Alignment = 0,
				.Width = ParticleCounterCount * sizeof(uint32_t),
				.Height = 1,
				.DepthOrArraySize = 1,
				.MipLevels = 1,
				.Format = DXGI_FORMAT_UNKNOWN,
				.SampleDesc = { .Count = 1, .Quality = 0 },
				.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR,
				.Flags = D3D12_RESOURCE_FLAG_NONE,
			};
			auto readback = D3DPtr<ID3D12Resource>{};
			CheckHr(m_rdr.d3d()->CreateCommittedResource(&heap, D3D12_HEAP_FLAG_NONE, &desc, D3D12_RESOURCE_STATE_COPY_DEST, nullptr, __uuidof(ID3D12Resource), (void**)readback.address_of()), "Failed to create the particle stats readback buffer");
			DefaultResState(readback.get(), D3D12_RESOURCE_STATE_COPY_DEST);
			DebugName(readback, "Particles:Stats");
			{
				// The counters are read as a copy source, then returned to their resting state.
				auto& cmd_list = m_factory.CmdList();
				BarrierBatch(cmd_list).Transition(type.m_counters.get(), D3D12_RESOURCE_STATE_COPY_SOURCE).Commit();
				cmd_list.CopyBufferRegion(readback.get(), 0, type.m_counters.get(), 0, desc.Width);
				BarrierBatch(cmd_list).Transition(type.m_counters.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS).Commit();
			}
			m_factory.FlushToGpu(EGpuFlush::Block);

			// The live count is the list that the next step reads.
			auto counters = std::array<uint32_t, ParticleCounterCount>{};
			void* mapped = nullptr;
			CheckHr(readback->Map(0, nullptr, &mapped), "Failed to map the particle stats readback buffer");
			std::memcpy(counters.data(), mapped, sizeof(counters));
			auto empty = D3D12_RANGE{ 0, 0 };
			readback->Unmap(0, &empty);
			return pr::view3d::ParticleStats{
				.m_free = counters[ParticleCounterFree],
				.m_live = counters[ParticleCounterLive0 + type.m_cur],
				.m_requested = counters[ParticleCounterRequested],
				.m_admitted = counters[ParticleCounterAdmitted],
				.m_dropped = counters[ParticleCounterDropped],
				.m_contacts = counters[ParticleCounterContacts],
				.m_overlaps = counters[ParticleCounterOverlaps],
				.m_exhausted = counters[ParticleCounterExhausted],
			};
		}

		// Set the gravity, uniform wind and contact budget of the environment.
		void EnvironmentSet(pr::view3d::ParticleEnvironment const& environment) override
		{
			// Validate against the documented domain. The constants are uploaded with every dispatch, so they apply from the next step.
			auto const& g = environment.m_gravity;
			auto const& w = environment.m_wind;
			if (!std::isfinite(g.x) || !std::isfinite(g.y) || !std::isfinite(g.z) || !std::isfinite(w.x) || !std::isfinite(w.y) || !std::isfinite(w.z))
				throw std::invalid_argument("Particle gravity and wind must be finite");
			if (environment.m_collision_iterations < 0 || environment.m_collision_iterations > MaxCollisionIterations)
				throw std::invalid_argument("Particle collision iterations must be in [0, 64]");

			m_env.gravity = float3{ g.x, g.y, g.z };
			m_env.wind = float3{ w.x, w.y, w.z };
			m_env.collision_iterations = static_cast<uint32_t>(environment.m_collision_iterations);
		}

		// Set the wind grid.
		void WindGridSet(v2 origin, float cell_size, int cells_x, int cells_y, std::span<v2 const> wind) override
		{
			// Zero columns in either direction removes the grid.
			if (cells_x < 0 || cells_y < 0)
				throw std::invalid_argument("Particle wind grid dimensions must be non-negative");
			if (cells_x == 0 || cells_y == 0)
			{
				m_env.wind_cells = uint2{ 0, 0 };
				return;
			}

			// Validate the grid against the documented domain.
			if (!std::isfinite(origin.x) || !std::isfinite(origin.y) || !std::isfinite(cell_size) || cell_size <= 0.0f)
				throw std::invalid_argument("Particle wind grid origin must be finite and its cell size positive");
			if (std::ssize(wind) != static_cast<int64_t>(cells_x) * cells_y)
				throw std::invalid_argument("Particle wind grid data must have one velocity per column");
			if (std::any_of(wind.begin(), wind.end(), [](v2 const& v) { return !std::isfinite(v.x) || !std::isfinite(v.y); }))
				throw std::invalid_argument("Particle wind grid velocities must be finite");

			m_env.wind_origin = float2{ origin.x, origin.y };
			m_env.wind_cell_size = cell_size;
			m_env.wind_cells = uint2{ static_cast<uint32_t>(cells_x), static_cast<uint32_t>(cells_y) };
			m_wind_data.assign(wind.begin(), wind.end());
			m_wind_dirty = true;
		}

		// Add a collision set of planes, returning its index.
		int CollisionSetCreate(std::span<pr::view3d::ParticlePlane const> planes) override
		{
			// Reuse the lowest deleted index, so the set table stays small.
			auto converted = ConvertPlanes(planes);
			auto it = std::find(m_set_deleted.begin(), m_set_deleted.end(), uint8_t{ 1 });
			auto const set = static_cast<int>(it - m_set_deleted.begin());
			if (it == m_set_deleted.end())
			{
				m_sets.emplace_back();
				m_set_deleted.push_back(0);
			}
			m_sets[set] = std::move(converted);
			m_set_deleted[set] = 0;
			m_sets_dirty = true;
			return set;
		}

		// Replace the planes of a collision set.
		void CollisionSetUpdate(int set, std::span<pr::view3d::ParticlePlane const> planes) override
		{
			CollisionSet(set) = ConvertPlanes(planes);
			m_sets_dirty = true;
		}

		// Remove a collision set.
		void CollisionSetDelete(int set) override
		{
			// A deleted set has no planes until its index is reused.
			CollisionSet(set).clear();
			m_set_deleted[set] = 1;
			m_sets_dirty = true;
		}

		// Bind the water surface that types sample, or remove it when 'ocean' is null.
		void WaterSet(ocean::OceanSurface const* ocean, pr::view3d::Object object) override
		{
			// A surface needs its object, which holds the buffer that the samples read.
			if (ocean != nullptr && object == nullptr)
				throw std::invalid_argument("A particle water surface requires its View3D object");

			// Removing the water binds the dummy buffer and zeroed constants from the next step.
			m_ocean = ocean;
			m_ocean_object = ocean != nullptr ? object : nullptr;
			m_env.water = ocean != nullptr ? 1u : 0u;
			if (ocean == nullptr)
			{
				m_ocean_constants = {};
				m_rdr.DeferRelease(m_ocean_buffer);
				m_ocean_buffer = nullptr;
			}
		}

	private:

		// Return the planes of the live collision set 'set'.
		std::vector<ParticleCollisionPlane>& CollisionSet(int set)
		{
			if (set < 0 || set >= std::ssize(m_sets) || m_set_deleted[set] != 0)
				throw std::out_of_range("Invalid particle collision set index");

			return m_sets[set];
		}

		// Validate 'planes' and convert them to their GPU layout.
		static std::vector<ParticleCollisionPlane> ConvertPlanes(std::span<pr::view3d::ParticlePlane const> planes)
		{
			// Normals must be unit length, because the collision distances are measured along them.
			auto converted = std::vector<ParticleCollisionPlane>{};
			converted.reserve(planes.size());
			for (auto const& plane : planes)
			{
				auto const& p = plane.m_plane;
				if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) || !std::isfinite(p.w))
					throw std::invalid_argument("Particle collision planes must be finite");
				if (std::abs(p.x * p.x + p.y * p.y + p.z * p.z - 1.0f) > PlaneNormalTolerance)
					throw std::invalid_argument("Particle collision plane normals must be unit length");
				if (!(plane.m_restitution >= 0.0f && plane.m_restitution <= 1.0f) || !(plane.m_damping >= 0.0f && plane.m_damping <= 1.0f))
					throw std::invalid_argument("Particle collision plane restitution and damping must be in [0, 1]");

				converted.push_back(ParticleCollisionPlane{
					.plane = float4{ p.x, p.y, p.z, p.w },
					.restitution = plane.m_restitution,
					.damping = plane.m_damping,
				});
			}
			return converted;
		}

		// Create a shader-readable environment buffer of 'count' elements of 'stride' bytes, initialised from 'data' when it is not empty.
		D3DPtr<ID3D12Resource> CreateEnvironmentBuffer(int64_t count, int stride, std::span<std::byte const> data, char const* name)
		{
			return m_factory.CreateResource(ResDesc::Buf(count, stride, data, 4).def_state(D3D12_RESOURCE_STATE_ALL_SHADER_RESOURCE), name);
		}

		// Copy 'data' to the start of 'buffer', replacing the buffer with a larger one when it holds fewer than 'count' elements.
		void UploadEnvironmentBuffer(D3DPtr<ID3D12Resource>& buffer, int64_t& capacity, int64_t count, int stride, std::span<std::byte const> data, char const* name)
		{
			// A buffer that is too small is replaced, initialised with the data. The old one may still be read by the last step.
			if (count > capacity)
			{
				m_rdr.DeferRelease(buffer);
				buffer = CreateEnvironmentBuffer(count, stride, data, name);
				capacity = count;
				return;
			}

			// Otherwise the data is copied over the start of the buffer. Elements past 'count' are never read.
			auto& cmd_list = m_factory.CmdList();
			auto& upload = m_factory.UploadBuffer();
			auto alloc = upload.Alloc(static_cast<int64_t>(data.size()), 4);
			std::memcpy(alloc.m_mem + alloc.m_ofs, data.data(), data.size());
			BarrierBatch(cmd_list).Transition(buffer.get(), D3D12_RESOURCE_STATE_COPY_DEST).Commit();
			cmd_list.CopyBufferRegion(buffer.get(), 0, alloc);
			BarrierBatch(cmd_list).Transition(buffer.get(), D3D12_RESOURCE_STATE_ALL_SHADER_RESOURCE).Commit();
		}

		// Upload the collision planes and wind grid if they changed since the last step.
		void UploadEnvironment()
		{
			// Flatten the collision sets into one plane table and one (first, count) range per set.
			if (m_sets_dirty)
			{
				auto planes = std::vector<ParticleCollisionPlane>{};
				auto ranges = std::vector<std::array<uint32_t, 2>>{};
				ranges.reserve(m_sets.size());
				for (auto const& set : m_sets)
				{
					ranges.push_back({ static_cast<uint32_t>(planes.size()), static_cast<uint32_t>(set.size()) });
					planes.insert(planes.end(), set.begin(), set.end());
				}
				if (!planes.empty())
					UploadEnvironmentBuffer(m_planes, m_planes_capacity, std::ssize(planes), sizeof(ParticleCollisionPlane), std::as_bytes(std::span(planes)), "Particles:Planes");
				if (!ranges.empty())
					UploadEnvironmentBuffer(m_set_ranges, m_set_ranges_capacity, std::ssize(ranges), sizeof(uint32_t) * 2, std::as_bytes(std::span(ranges)), "Particles:CollisionSets");

				m_env.collision_set_count = static_cast<uint32_t>(ranges.size());
				m_sets_dirty = false;
			}

			// The grid dimensions in the constants describe the uploaded data.
			if (m_wind_dirty)
			{
				UploadEnvironmentBuffer(m_wind, m_wind_capacity, std::ssize(m_wind_data), sizeof(v2), std::as_bytes(std::span(m_wind_data)), "Particles:Wind");
				m_wind_dirty = false;
			}
		}

		// Copy the bound water surface's constants for this step, and share its current procedural buffer.
		void UpdateWater()
		{
			// Without water, the dummy buffer and zeroed constants are bound and the environment says there is no water.
			if (m_ocean == nullptr)
				return;

			// The constants describe the frame last passed to the ocean's Update. The buffer rests in a shader-readable state, so it needs no barrier.
			// The ocean may replace its buffer, so it is fetched each step. The old one may still be read by the last step.
			m_ocean_constants = m_ocean->Constants();
			auto* buffer = static_cast<ID3D12Resource*>(View3D_ObjectProceduralBufferGet(m_ocean_object, 0));
			if (buffer == nullptr)
				throw std::logic_error("The particle water surface's object has no procedural buffer");
			if (buffer != m_ocean_buffer.get())
			{
				m_rdr.DeferRelease(m_ocean_buffer);
				m_ocean_buffer = D3DPtr<ID3D12Resource>(buffer, true);
			}
		}

		// Return the type at 'type_index'.
		ParticleType& Type(int type_index)
		{
			if (type_index < 0 || type_index >= std::ssize(m_types))
				throw std::out_of_range("Invalid particle type index");

			return *m_types[type_index];
		}

		// Return the type and slot of a live emitter.
		std::pair<ParticleType*, int> Emitter(uint64_t id)
		{
			// The id must name a slot in use whose generation still matches.
			auto const generation = static_cast<uint32_t>(id >> EmitterGenerationShift);
			auto const type_index = static_cast<int>((id >> EmitterTypeShift) & 0xFF);
			auto const slot = static_cast<int>(id & ((1u << EmitterSlotBits) - 1));
			auto& type = Type(type_index);
			if (generation == 0 || slot >= type.m_max_emitters || type.m_in_use[slot] == 0 || type.m_emitter_headers[slot].generation != generation)
				throw std::invalid_argument("Stale or invalid particle emitter id");

			return { &type, slot };
		}

		// Require emitter settings and parameters to be valid for 'type'.
		static void ValidateEmitter(ParticleType const& type, pr::view3d::ParticleEmitter const& emitter, std::span<std::byte const> params)
		{
			if (!std::isfinite(emitter.m_rate) || emitter.m_rate < 0.0f)
				throw std::invalid_argument("Particle emitter rate must be finite and non-negative");
			if (std::ssize(params) != type.m_params_stride)
				throw std::invalid_argument("Particle emitter parameters must be the type's parameter stride in size");
		}

		// Store emitter settings and parameters in 'slot' for the next step.
		static void Store(ParticleType& type, int slot, pr::view3d::ParticleEmitter const& emitter, std::span<std::byte const> params)
		{
			auto& header = type.m_emitter_headers[slot];
			header.flags = emitter.m_active ? ParticleEmitterActive : 0u;
			header.rate = emitter.m_rate;
			header.seed = emitter.m_seed;
			std::memcpy(type.m_emitter_params.data() + static_cast<size_t>(slot) * type.m_params_stride, params.data(), params.size());
			type.Dirty(slot);
		}

		// Bind the pool of 'type' and the constants of one dispatch to the shared root signature.
		void Bind(GfxCmdList& cmd_list, ParticleType& type, ParticleTickConstants const& tick)
		{
			// Constants are copied per dispatch, so a type's buffers need no persistent constant buffer.
			auto& upload = m_factory.UploadBuffer();
			cmd_list.SetComputeRootSignature(m_sig.get());
			cmd_list.SetComputeRootConstantBufferView(RootTick, UploadConstants(upload, &tick, sizeof(tick)));
			cmd_list.SetComputeRootConstantBufferView(RootType, UploadConstants(upload, type.m_constants.data(), type.m_constants.size()));
			cmd_list.SetComputeRootUnorderedAccessView(RootCounters, type.m_counters);
			cmd_list.SetComputeRootUnorderedAccessView(RootFree, type.m_free);
			cmd_list.SetComputeRootUnorderedAccessView(RootLive, type.m_live);
			cmd_list.SetComputeRootUnorderedAccessView(RootStates, type.m_states);
			cmd_list.SetComputeRootUnorderedAccessView(RootRecords, type.m_batch->m_records);
			cmd_list.SetComputeRootUnorderedAccessView(RootRuntime, type.m_runtime);
			cmd_list.SetComputeRootUnorderedAccessView(RootArgs, type.m_args);
			cmd_list.SetComputeRootUnorderedAccessView(RootDrawArgs, type.m_batch->m_draw_args);
			cmd_list.SetComputeRootShaderResourceView(RootHeaders, type.m_headers);
			cmd_list.SetComputeRootShaderResourceView(RootParams, type.m_params);
			cmd_list.SetComputeRootUnorderedAccessView(RootBounds, type.m_batch->m_bounds);
			cmd_list.SetComputeRootConstantBufferView(RootEnvironment, UploadConstants(upload, &m_env, sizeof(m_env)));
			cmd_list.SetComputeRootShaderResourceView(RootPlanes, m_planes);
			cmd_list.SetComputeRootShaderResourceView(RootSets, m_set_ranges);
			cmd_list.SetComputeRootShaderResourceView(RootWind, m_wind);
			cmd_list.SetComputeRootConstantBufferView(RootOcean, UploadConstants(upload, &m_ocean_constants, sizeof(m_ocean_constants)));
			cmd_list.SetComputeRootShaderResourceView(RootOceanBuffer, m_ocean_buffer != nullptr ? m_ocean_buffer : m_water_dummy);
		}

		// Zero every batch's record count and make its buffers writable by the kernels.
		void BeginBatches()
		{
			// The instance count is zeroed by a copy, so no kernel needs to know which type of a batch runs first.
			auto& cmd_list = m_factory.CmdList();
			auto& upload = m_factory.UploadBuffer();
			auto zero = upload.Alloc(sizeof(uint32_t), 4);
			std::memset(zero.m_mem + zero.m_ofs, 0, sizeof(uint32_t));
			for (auto& batch : m_batches)
			{
				BarrierBatch(cmd_list).Transition(batch->m_draw_args.get(), D3D12_RESOURCE_STATE_COPY_DEST).Commit();
				cmd_list.CopyBufferRegion(batch->m_draw_args.get(), ParticleDrawInstanceCount * sizeof(uint32_t), zero);
				BarrierBatch(cmd_list)
					.Transition(batch->m_draw_args.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS)
					.Transition(batch->m_records.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS)
					.Transition(batch->m_bounds.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS)
					.Commit();
			}
		}

		// Return every batch's buffers to the states that the draw reads them in.
		void EndBatches()
		{
			// The kernels' writes are complete before the draw reads them.
			auto& cmd_list = m_factory.CmdList();
			for (auto& batch : m_batches)
			{
				BarrierBatch(cmd_list)
					.Transition(batch->m_draw_args.get(), D3D12_RESOURCE_STATE_INDIRECT_ARGUMENT | D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE)
					.Transition(batch->m_records.get(), D3D12_RESOURCE_STATE_ALL_SHADER_RESOURCE)
					.Transition(batch->m_bounds.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE)
					.Commit();
			}
		}

		// Record the reset of the pool of 'type': every slot free, no particle live, and every emitter slot's progress restarted.
		void RecordReset(ParticleType& type)
		{
			// The reset kernel covers the larger of the pool and the emitter slots with one thread each.
			auto& cmd_list = m_factory.CmdList();
			auto const tick = ParticleTickConstants{
				.dt = 0.0f,
				.cur = 0,
				.emitter_hwm = static_cast<uint32_t>(type.m_max_emitters),
				.capacity = type.m_capacity,
				.tick = m_tick,
			};
			auto const threads = std::max(type.m_capacity, static_cast<uint32_t>(type.m_max_emitters));
			BarrierBatch(cmd_list).Transition(type.m_args.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS).Commit();
			Bind(cmd_list, type, tick);
			cmd_list.SetPipelineState(m_reset.get());
			cmd_list.Dispatch(static_cast<int>((threads + ParticleGroupSize - 1) / ParticleGroupSize), 1, 1);
			BarrierBatch(cmd_list).UAV(nullptr).Transition(type.m_args.get(), D3D12_RESOURCE_STATE_INDIRECT_ARGUMENT).Commit();
			type.m_cur = 0;
		}

		// Record one tick of 'type': upload changed emitters, update and emit particles, then point the draw at the new live list.
		void RecordTick(ParticleType& type, float dt)
		{
			auto& cmd_list = m_factory.CmdList();
			auto pix = pix::EventScope<ID3D12GraphicsCommandList>(cmd_list.get(), 0xFFFFC080, type.m_name.c_str());

			// Upload the changed emitter slots. The range is contiguous, so it is two copies however many emitters changed.
			if (type.m_dirty_lo < type.m_dirty_hi)
			{
				auto const header_size = static_cast<int64_t>(sizeof(ParticleEmitterHeader));
				auto const params_size = static_cast<int64_t>(type.m_params_stride);
				auto const count = static_cast<int64_t>(type.m_dirty_hi - type.m_dirty_lo);
				auto& upload = m_factory.UploadBuffer();
				auto headers = upload.Alloc(count * header_size, 4);
				auto params = upload.Alloc(count * params_size, 4);
				std::memcpy(headers.m_mem + headers.m_ofs, type.m_emitter_headers.data() + type.m_dirty_lo, static_cast<size_t>(count * header_size));
				std::memcpy(params.m_mem + params.m_ofs, type.m_emitter_params.data() + type.m_dirty_lo * params_size, static_cast<size_t>(count * params_size));

				BarrierBatch(cmd_list).Transition(type.m_headers.get(), D3D12_RESOURCE_STATE_COPY_DEST).Transition(type.m_params.get(), D3D12_RESOURCE_STATE_COPY_DEST).Commit();
				cmd_list.CopyBufferRegion(type.m_headers.get(), static_cast<uint64_t>(type.m_dirty_lo * header_size), headers);
				cmd_list.CopyBufferRegion(type.m_params.get(), static_cast<uint64_t>(type.m_dirty_lo * params_size), params);
				BarrierBatch(cmd_list).Transition(type.m_headers.get(), D3D12_RESOURCE_STATE_ALL_SHADER_RESOURCE).Transition(type.m_params.get(), D3D12_RESOURCE_STATE_ALL_SHADER_RESOURCE).Commit();
				type.m_dirty_lo = type.m_max_emitters;
				type.m_dirty_hi = 0;
			}

			// Update the particles of the read list. The dispatch size was written by the previous tick's finalize.
			auto const tick = ParticleTickConstants{
				.dt = dt,
				.cur = type.m_cur,
				.emitter_hwm = static_cast<uint32_t>(type.m_emitter_hwm),
				.capacity = type.m_capacity,
				.tick = m_tick,
			};
			Bind(cmd_list, type, tick);
			cmd_list.SetPipelineState(type.m_update.get());
			cmd_list.ExecuteIndirect(m_rdr.DispatchSignature(), 1, type.m_args.get(), ParticleArgsDispatchX * sizeof(uint32_t));
			BarrierBatch(cmd_list).UAV(nullptr).Commit();

			// Emit after the update, so new particles take the slots that this tick's deaths freed. One group per emitter slot.
			if (type.m_emitter_hwm != 0)
			{
				cmd_list.SetPipelineState(type.m_emit.get());
				cmd_list.Dispatch(type.m_emitter_hwm, 1, 1);
				BarrierBatch(cmd_list).UAV(nullptr).Commit();
			}

			// Close the tick on the GPU, then return the update's dispatch arguments to the state that the next tick reads them in.
			BarrierBatch(cmd_list).Transition(type.m_args.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS).Commit();
			cmd_list.SetPipelineState(m_finalize.get());
			cmd_list.Dispatch(1, 1, 1);
			BarrierBatch(cmd_list).UAV(nullptr).Transition(type.m_args.get(), D3D12_RESOURCE_STATE_INDIRECT_ARGUMENT).Commit();
			type.m_cur = 1u - type.m_cur;
		}

		// Create the View3D object that draws the records of 'batch'.
		pr::view3d::Object CreateObject(ParticleBatch& batch, pr::view3d::ParticleBatchOptions const& options, char const* name)
		{
			// Procedural vertex constants have a fixed size, so the batch's constants are zero-padded to it. The shader copies them.
			auto constants = std::vector<std::byte>(pr::view3d::ProceduralBinding::ConstantsSize);
			if (options.m_constants != nullptr)
				std::memcpy(constants.data(), options.m_constants, options.m_constants_size);

			auto const& shaders = options.m_shaders;
			auto const shader_options = pr::view3d::ShaderOptions
			{
				.m_struct_size = sizeof(pr::view3d::ShaderOptions),
				.m_version = pr::view3d::ShaderOptions::CurrentVersion,
				.m_stage = pr::view3d::EShaderStage::Vertex,
				.m_bytecode = shaders.m_vertex.m_bytecode,
				.m_bytecode_size = shaders.m_vertex.m_size,
				.m_dbg_name = name,
				.m_procedural =
				{
					.m_rdr_step = pr::view3d::ERenderStep::ForwardRender,
					.m_constants = constants.data(),
					.m_constants_size = constants.size(),
				},
			};
			auto forward = pr::view3d::ShaderPtr(View3D_ShaderCreate(shader_options));
			if (forward == nullptr)
				throw std::invalid_argument(std::format("Particle batch '{}' has an invalid vertex shader", name));

			// Particles are seen from both sides and cast no shadows. The blend mode chooses how they combine with the scene.
			auto nflags = pr::view3d::ENuggetFlag::ShadowCastExclude;
			switch (options.m_blend)
			{
				case pr::view3d::EParticleBlend::Sorted:
				{
					nflags = nflags | pr::view3d::ENuggetFlag::GeometryHasAlpha;
					break;
				}
				case pr::view3d::EParticleBlend::Weighted:
				{
					nflags = nflags | pr::view3d::ENuggetFlag::GeometryHasAlpha | pr::view3d::ENuggetFlag::AlphaWeighted;
					break;
				}
				case pr::view3d::EParticleBlend::Opaque:
				{
					break;
				}
				default:
				{
					throw std::invalid_argument("Unknown particle blend mode");
				}
			}
			auto nugget = pr::view3d::Nugget{};
			nugget.m_topo = pr::view3d::ETopo::TriList;
			nugget.m_geom = shaders.m_geom;
			nugget.m_v0 = 0;
			nugget.m_v1 = 1;
			nugget.m_i0 = 0;
			nugget.m_i1 = shaders.m_index_count;
			nugget.m_shaders[0] = { forward.get(), pr::view3d::ERenderStep::ForwardRender, 0 };
			nugget.m_shaders[1] = { nullptr, pr::view3d::ERenderStep::Invalid, 0 };
			nugget.m_nflags = nflags;
			nugget.m_cull_mode = pr::view3d::ECullMode::None;
			nugget.m_fill_mode = pr::view3d::EFillMode::Default;
			nugget.m_tint = 0xFFFFFFFFU;
			nugget.m_rel_reflec = 1.0f;
			nugget.m_tex_diffuse = options.m_texture;
			nugget.m_sam_diffuse = options.m_sampler;

			// The vertex shader generates every vertex from its logical id and instance, so the object has one placeholder vertex.
			auto const options_create = pr::view3d::ObjectCreateOptions
			{
				.m_struct_size = sizeof(pr::view3d::ObjectCreateOptions),
				.m_version = pr::view3d::ObjectCreateOptions::CurrentVersion,
				.m_vertex_source = pr::view3d::EVertexSource::ProceduralVertexId,
				.m_vcount_logical = shaders.m_vertex_count,
				.m_bbox = options.m_bbox,
			};
			auto placeholder = pr::view3d::Vertex{};
			placeholder.pos = { 0.0f, 0.0f, 0.0f, 1.0f };
			auto context_id = GUID{};
			CheckHr(CoCreateGuid(&context_id), "Failed to allocate the particle object context");
			auto object = View3D_ObjectCreateU32(name, 0xFFFFFFFFU, 1, shaders.m_index_count, 1, &placeholder, shaders.m_indices, &nugget, options_create, context_id);
			if (object == nullptr)
				throw std::runtime_error("Failed to create the particle object");

			try
			{
				// The object keeps its own references to the records, the optional vertex data, the draw arguments, and the record bounds that the renderer culls with.
				View3D_ObjectProceduralBufferSet(object, 0, batch.m_records.get());
				if (batch.m_vertex_data != nullptr)
					View3D_ObjectProceduralBufferSet(object, 1, batch.m_vertex_data.get());

				View3D_ObjectNuggetDrawArgsSet(object, batch.m_draw_args.get(), 0, batch.m_bounds.get(), static_cast<int>(batch.m_capacity), 0);
			}
			catch (...)
			{
				// The caller never receives the object, so release it here.
				View3D_ObjectDelete(object);
				throw;
			}
			return object;
		}
	};

	// Create an empty particle system.
	std::unique_ptr<ParticleSystem> ParticleSystem::Create(Renderer& rdr, char const* name)
	{
		return std::make_unique<ParticleSystemImpl>(rdr, name);
	}

	// The simulation shaders of the stock sprite type.
	pr::view3d::ParticleSimShaders ParticleSystem::SpriteSimShaders()
	{
		return pr::view3d::ParticleSimShaders{
			.m_emit = { compiled::particle_sprite_emit_cs, sizeof(compiled::particle_sprite_emit_cs) },
			.m_update = { compiled::particle_sprite_update_cs, sizeof(compiled::particle_sprite_update_cs) },
		};
	}

	// The simulation shaders of the stock stepped sprite type.
	pr::view3d::ParticleSimShaders ParticleSystem::SteppedSpriteSimShaders()
	{
		return pr::view3d::ParticleSimShaders{
			.m_emit = { compiled::particle_stepped_sprite_emit_cs, sizeof(compiled::particle_stepped_sprite_emit_cs) },
			.m_update = { compiled::particle_stepped_sprite_update_cs, sizeof(compiled::particle_stepped_sprite_update_cs) },
		};
	}

	// The draw shaders of a batch of stock sprite records.
	pr::view3d::ParticleDrawShaders ParticleSystem::SpriteDrawShaders()
	{
		return pr::view3d::ParticleDrawShaders{
			.m_vertex = { compiled::particle_sprite_vs, sizeof(compiled::particle_sprite_vs) },
			.m_indices = SpriteIndices.data(),
			.m_index_count = static_cast<int>(SpriteIndices.size()),
			.m_vertex_count = static_cast<int>(ParticleSpriteVertexCount),
			.m_geom = pr::view3d::EGeom::Vert | pr::view3d::EGeom::Colr,
			.m_vertex_data = nullptr,
			.m_vertex_stride = 0,
		};
	}

	// The vertex shader and outputs of a batch of stock mesh records. The caller supplies the mesh.
	pr::view3d::ParticleDrawShaders ParticleSystem::MeshDrawShaders()
	{
		return pr::view3d::ParticleDrawShaders{
			.m_vertex = { compiled::particle_mesh_vs, sizeof(compiled::particle_mesh_vs) },
			.m_indices = nullptr,
			.m_index_count = 0,
			.m_vertex_count = 0,
			.m_geom = pr::view3d::EGeom::Vert | pr::view3d::EGeom::Colr | pr::view3d::EGeom::Norm | pr::view3d::EGeom::Tex0,
			.m_vertex_data = nullptr,
			.m_vertex_stride = sizeof(ParticleMeshVertex),
		};
	}

	// The simulation shaders of the stock debris type.
	pr::view3d::ParticleSimShaders ParticleSystem::DebrisSimShaders()
	{
		return pr::view3d::ParticleSimShaders{
			.m_emit = { compiled::particle_debris_emit_cs, sizeof(compiled::particle_debris_emit_cs) },
			.m_update = { compiled::particle_debris_update_cs, sizeof(compiled::particle_debris_update_cs) },
		};
	}

	// The simulation shaders of the stock bird type.
	pr::view3d::ParticleSimShaders ParticleSystem::BirdSimShaders()
	{
		return pr::view3d::ParticleSimShaders{
			.m_emit = { compiled::particle_bird_emit_cs, sizeof(compiled::particle_bird_emit_cs) },
			.m_update = { compiled::particle_bird_update_cs, sizeof(compiled::particle_bird_update_cs) },
		};
	}

	// The draw shaders of a batch of stock bird records.
	pr::view3d::ParticleDrawShaders ParticleSystem::BirdDrawShaders()
	{
		return pr::view3d::ParticleDrawShaders{
			.m_vertex = { compiled::particle_bird_vs, sizeof(compiled::particle_bird_vs) },
			.m_indices = BirdIndices.data(),
			.m_index_count = static_cast<int>(BirdIndices.size()),
			.m_vertex_count = static_cast<int>(ParticleBirdVertexCount),
			.m_geom = pr::view3d::EGeom::Vert | pr::view3d::EGeom::Colr,
			.m_vertex_data = nullptr,
			.m_vertex_stride = 0,
		};
	}

	// The simulation shaders of the stock marker type.
	pr::view3d::ParticleSimShaders ParticleSystem::MarkerSimShaders()
	{
		return pr::view3d::ParticleSimShaders{
			.m_emit = { compiled::particle_marker_emit_cs, sizeof(compiled::particle_marker_emit_cs) },
			.m_update = { compiled::particle_marker_update_cs, sizeof(compiled::particle_marker_update_cs) },
		};
	}

	// The simulation shaders of the stock spray type.
	pr::view3d::ParticleSimShaders ParticleSystem::SpraySimShaders()
	{
		return pr::view3d::ParticleSimShaders{
			.m_emit = { compiled::particle_spray_emit_cs, sizeof(compiled::particle_spray_emit_cs) },
			.m_update = { compiled::particle_spray_update_cs, sizeof(compiled::particle_spray_update_cs) },
		};
	}

	// The simulation shaders of the stock froth type.
	pr::view3d::ParticleSimShaders ParticleSystem::FrothSimShaders()
	{
		return pr::view3d::ParticleSimShaders{
			.m_emit = { compiled::particle_froth_emit_cs, sizeof(compiled::particle_froth_emit_cs) },
			.m_update = { compiled::particle_froth_update_cs, sizeof(compiled::particle_froth_update_cs) },
		};
	}
}
