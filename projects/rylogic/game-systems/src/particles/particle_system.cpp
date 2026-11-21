//************************************
// Game Systems
//  Copyright (c) Rylogic Ltd 2026
//************************************
#include "pr/game-systems/forward.h"
#include "pr/game-systems/particles/particle_system.h"
#include "pr/game-systems/particles/particles.hlsli"
#include "pr/game-systems/particles/particle_sprite.hlsli"

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
		#include PR_GAME_SYSTEMS_SHADER_COMPILED_DIR(particle_sprite_vs.h)
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
			RootHeaders = 9,   // t0
			RootParams = 10,   // t1
		};

		// Limits of the type options. The capacity is bounded so that the reset dispatch fits in one dimension of thread groups.
		constexpr int MaxCapacity = 65535 * static_cast<int>(ParticleGroupSize);
		constexpr int MaxEmitters = 65535;
		constexpr int MaxStride = 2048;
		constexpr int MaxTypes = 256;

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

	// One particle type: its pool of GPU buffers, its kernels, and the CPU copy of its emitter slots.
	struct ParticleType
	{
		// The fixed layout of the type.
		std::string m_name;
		uint32_t m_capacity;
		int m_max_emitters;
		int m_params_stride;
		std::vector<std::byte> m_constants; // The type's constants, zero-padded to ProceduralBinding::ConstantsSize.
		D3DPtr<ID3D12PipelineState> m_emit;
		D3DPtr<ID3D12PipelineState> m_update;

		// The GPU pool (see particles.hlsli).
		D3DPtr<ID3D12Resource> m_counters;
		D3DPtr<ID3D12Resource> m_free;
		D3DPtr<ID3D12Resource> m_live;
		D3DPtr<ID3D12Resource> m_states;
		D3DPtr<ID3D12Resource> m_records;
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
		std::vector<std::unique_ptr<ParticleType>> m_types;
		uint32_t m_tick;

	public:

		ParticleSystemImpl(Renderer& rdr, char const* name)
			: m_rdr(rdr)
			, m_name(name != nullptr ? name : "Particles")
			, m_factory(rdr, EResourceFactoryLifetime::Retained)
			, m_sig()
			, m_reset()
			, m_finalize()
			, m_types()
			, m_tick()
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
				.SRV(hlsl::ESRVReg::t0)
				.SRV(hlsl::ESRVReg::t1)
				.Create(m_rdr.d3d(), "Particles:Sig");
			m_reset = ComputePSO(m_sig.get(), compiled::particle_reset_cs).Create(m_rdr.d3d(), "Particles:Reset");
			m_finalize = ComputePSO(m_sig.get(), compiled::particle_finalize_cs).Create(m_rdr.d3d(), "Particles:Finalize");
		}
		~ParticleSystemImpl()
		{
			// The GPU may still be running the last step, so release through the renderer's deferred release.
			for (auto& type : m_types)
			{
				for (auto const* res : { &type->m_counters, &type->m_free, &type->m_live, &type->m_states, &type->m_records, &type->m_runtime, &type->m_args, &type->m_headers, &type->m_params })
					m_rdr.DeferRelease(*res);

				m_rdr.DeferRelease(type->m_emit);
				m_rdr.DeferRelease(type->m_update);
			}
			m_rdr.DeferRelease(m_reset);
			m_rdr.DeferRelease(m_finalize);
			m_rdr.DeferRelease(m_sig);
		}

		// Validate 'options' and add a particle type.
		int TypeCreate(pr::view3d::ParticleTypeOptions const& options, char const* name, pr::view3d::Object& out_object) override
		{
			// Validate the options against the documented domain.
			if (options.m_struct_size != sizeof(pr::view3d::ParticleTypeOptions) || options.m_version != pr::view3d::ParticleTypeOptions::CurrentVersion)
				throw std::invalid_argument("Particle type options have an unsupported size or version");
			if (std::ssize(m_types) == MaxTypes)
				throw std::invalid_argument("Too many particle types");
			if (options.m_capacity < 1 || options.m_capacity > MaxCapacity)
				throw std::invalid_argument("Particle type capacity is out of range");
			if (options.m_max_emitters < 1 || options.m_max_emitters > MaxEmitters)
				throw std::invalid_argument("Particle type emitter count is out of range");

			RequireStride(options.m_state_stride, "Particle state stride must be a multiple of 4 bytes in [4, 2048]");
			RequireStride(options.m_record_stride, "Particle record stride must be a multiple of 4 bytes in [4, 2048]");
			RequireStride(options.m_params_stride, "Particle emitter parameter stride must be a multiple of 4 bytes in [4, 2048]");
			if (options.m_constants == nullptr || options.m_constants_size < 4 || options.m_constants_size > pr::view3d::ProceduralBinding::ConstantsSize || options.m_constants_size % 4 != 0)
				throw std::invalid_argument("Particle type constants must be a multiple of 4 bytes within the procedural constants size");
			if (static_cast<size_t>(options.m_capacity) * options.m_record_stride > pr::view3d::ProceduralBinding::MaxBufferSize)
				throw std::invalid_argument("Particle type records exceed the procedural buffer size");

			auto const& shaders = options.m_shaders;
			RequireBytecode(shaders.m_emit, "Particle emit shader is required");
			RequireBytecode(shaders.m_update, "Particle update shader is required");
			RequireBytecode(shaders.m_vertex, "Particle vertex shader is required");
			if (shaders.m_indices == nullptr || shaders.m_index_count < 3 || shaders.m_index_count % 3 != 0 || shaders.m_vertex_count < 1)
				throw std::invalid_argument("Particle indices must be a non-empty triangle list over a positive vertex count");
			if (std::any_of(shaders.m_indices, shaders.m_indices + shaders.m_index_count, [&](uint32_t i) { return i >= static_cast<uint32_t>(shaders.m_vertex_count); }))
				throw std::invalid_argument("Particle indices must be less than the vertex count");

			// Record the layout and create the type's kernels.
			auto type_name = std::string(name != nullptr ? name : "ParticleType");
			auto type = std::make_unique<ParticleType>();
			type->m_name = type_name;
			type->m_capacity = static_cast<uint32_t>(options.m_capacity);
			type->m_max_emitters = options.m_max_emitters;
			type->m_params_stride = options.m_params_stride;
			// Procedural vertex constants have a fixed size, so the type's constants are zero-padded to it.
			type->m_constants.assign(static_cast<std::byte const*>(options.m_constants), static_cast<std::byte const*>(options.m_constants) + options.m_constants_size);
			type->m_constants.resize(pr::view3d::ProceduralBinding::ConstantsSize);
			type->m_emit = ComputePSO(m_sig.get(), Bytecode(shaders.m_emit)).Create(m_rdr.d3d(), (type_name + ":Emit").c_str());
			type->m_update = ComputePSO(m_sig.get(), Bytecode(shaders.m_update)).Create(m_rdr.d3d(), (type_name + ":Update").c_str());
			type->m_emitter_headers.assign(options.m_max_emitters, ParticleEmitterHeader{});
			type->m_emitter_params.assign(static_cast<size_t>(options.m_max_emitters) * options.m_params_stride, std::byte{});
			type->m_in_use.assign(options.m_max_emitters, uint8_t{});
			type->m_emitter_hwm = 0;
			type->m_dirty_lo = options.m_max_emitters;
			type->m_dirty_hi = 0;
			type->m_cur = 0;

			// Create the pool. Buffers rest in the states that their readers outside a step need: the draw reads the records as a shader
			// resource and the args as indirect arguments. The reset kernel fills the free stack and counters before the first step.
			auto const capacity = static_cast<int64_t>(options.m_capacity);
			auto const max_emitters = static_cast<int64_t>(options.m_max_emitters);
			auto const draw_args = std::array<uint32_t, ParticleArgsCount>{ static_cast<uint32_t>(shaders.m_index_count), 0, 0, 0, 0, 0, 1, 1 };
			auto const none = std::span<std::byte const>{};
			auto const uav = D3D12_RESOURCE_STATE_UNORDERED_ACCESS;
			auto const srv = D3D12_RESOURCE_STATE_ALL_SHADER_RESOURCE;
			type->m_counters = m_factory.CreateResource(ResDesc::Buf(ParticleCounterCount, 4, none, 4).usage(EUsage::UnorderedAccess).def_state(uav), type_name + ":Counters");
			type->m_free = m_factory.CreateResource(ResDesc::Buf(capacity, 4, none, 4).usage(EUsage::UnorderedAccess).def_state(uav), type_name + ":Free");
			type->m_live = m_factory.CreateResource(ResDesc::Buf(2 * capacity, 4, none, 4).usage(EUsage::UnorderedAccess).def_state(uav), type_name + ":Live");
			type->m_states = m_factory.CreateResource(ResDesc::Buf(capacity, options.m_state_stride, none, 4).usage(EUsage::UnorderedAccess).def_state(uav), type_name + ":States");
			type->m_records = m_factory.CreateResource(ResDesc::Buf(capacity, options.m_record_stride, none, 4).usage(EUsage::UnorderedAccess).def_state(srv), type_name + ":Records");
			type->m_runtime = m_factory.CreateResource(ResDesc::Buf(max_emitters, sizeof(ParticleEmitterRuntime), none, 4).usage(EUsage::UnorderedAccess).def_state(uav), type_name + ":Runtime");
			type->m_args = m_factory.CreateResource(ResDesc::Buf<uint32_t>(ParticleArgsCount, std::span<uint32_t const>(draw_args)).usage(EUsage::UnorderedAccess).def_state(D3D12_RESOURCE_STATE_INDIRECT_ARGUMENT), type_name + ":Args");
			type->m_headers = m_factory.CreateResource(ResDesc::Buf(max_emitters, sizeof(ParticleEmitterHeader), none, 4).def_state(srv), type_name + ":Headers");
			type->m_params = m_factory.CreateResource(ResDesc::Buf(max_emitters, options.m_params_stride, none, 4).def_state(srv), type_name + ":Params");
			RecordReset(*type);

			// Upload every emitter slot in the first step, so the GPU copy starts as inactive slots rather than uninitialised memory.
			type->Dirty(0);
			type->Dirty(options.m_max_emitters - 1);
			m_factory.FlushToGpu(EGpuFlush::Async);

			// Create the type's object: one instance per live particle, drawn by one indirect draw whose arguments the GPU writes.
			out_object = CreateObject(*type, options, type_name.c_str());
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
				// Record every type's tick in the factory's command list.
				auto& cmd_list = m_factory.CmdList();
				auto pix = pix::EventScope<ID3D12GraphicsCommandList>(cmd_list.get(), 0xFFFFA040, "Particles::Step");
				for (auto& type : m_types)
					RecordTick(*type, static_cast<float>(dt));
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
			};
		}

	private:

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
			cmd_list.SetComputeRootUnorderedAccessView(RootRecords, type.m_records);
			cmd_list.SetComputeRootUnorderedAccessView(RootRuntime, type.m_runtime);
			cmd_list.SetComputeRootUnorderedAccessView(RootArgs, type.m_args);
			cmd_list.SetComputeRootShaderResourceView(RootHeaders, type.m_headers);
			cmd_list.SetComputeRootShaderResourceView(RootParams, type.m_params);
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
			BarrierBatch(cmd_list).Transition(type.m_records.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS).Commit();
			Bind(cmd_list, type, tick);
			cmd_list.SetPipelineState(type.m_update.get());
			cmd_list.ExecuteIndirect(m_rdr.DispatchSignature(), 1, type.m_args.get(), ParticleArgsDispatchOffset);
			BarrierBatch(cmd_list).UAV(nullptr).Commit();

			// Emit after the update, so new particles take the slots that this tick's deaths freed. One group per emitter slot.
			if (type.m_emitter_hwm != 0)
			{
				cmd_list.SetPipelineState(type.m_emit.get());
				cmd_list.Dispatch(type.m_emitter_hwm, 1, 1);
				BarrierBatch(cmd_list).UAV(nullptr).Commit();
			}

			// Close the tick on the GPU, then return the draw's buffers to the states that the draw reads them in.
			BarrierBatch(cmd_list).Transition(type.m_args.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS).Commit();
			cmd_list.SetPipelineState(m_finalize.get());
			cmd_list.Dispatch(1, 1, 1);
			BarrierBatch(cmd_list)
				.UAV(nullptr)
				.Transition(type.m_args.get(), D3D12_RESOURCE_STATE_INDIRECT_ARGUMENT)
				.Transition(type.m_records.get(), D3D12_RESOURCE_STATE_ALL_SHADER_RESOURCE)
				.Commit();
			type.m_cur = 1u - type.m_cur;
		}

		// Create the View3D object that draws the particles of 'type'.
		pr::view3d::Object CreateObject(ParticleType& type, pr::view3d::ParticleTypeOptions const& options, char const* name)
		{
			// The vertex shader reads the type's constants as its procedural constants.
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
					.m_constants = type.m_constants.data(),
					.m_constants_size = type.m_constants.size(),
				},
			};
			auto forward = pr::view3d::ShaderPtr(View3D_ShaderCreate(shader_options));
			if (forward == nullptr)
				throw std::runtime_error("Failed to create the particle vertex shader");

			// Particles are translucent, seen from both sides, and cast no shadows.
			auto nugget = pr::view3d::Nugget{};
			nugget.m_topo = pr::view3d::ETopo::TriList;
			nugget.m_geom = pr::view3d::EGeom::Vert | pr::view3d::EGeom::Colr;
			nugget.m_v0 = 0;
			nugget.m_v1 = 1;
			nugget.m_i0 = 0;
			nugget.m_i1 = shaders.m_index_count;
			nugget.m_shaders[0] = { forward.get(), pr::view3d::ERenderStep::ForwardRender, 0 };
			nugget.m_shaders[1] = { nullptr, pr::view3d::ERenderStep::Invalid, 0 };
			nugget.m_nflags = pr::view3d::ENuggetFlag::GeometryHasAlpha | pr::view3d::ENuggetFlag::ShadowCastExclude;
			nugget.m_cull_mode = pr::view3d::ECullMode::None;
			nugget.m_fill_mode = pr::view3d::EFillMode::Default;
			nugget.m_tint = 0xFFFFFFFFU;
			nugget.m_rel_reflec = 1.0f;

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
				// The object keeps its own references to the records and the draw arguments.
				View3D_ObjectProceduralBufferSet(object, 0, type.m_records.get());
				View3D_ObjectNuggetDrawArgsSet(object, type.m_args.get(), 0, 0);
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

	// The shaders of the stock sprite type.
	pr::view3d::ParticleTypeShaders ParticleSystem::SpriteShaders()
	{
		return pr::view3d::ParticleTypeShaders{
			.m_emit = { compiled::particle_sprite_emit_cs, sizeof(compiled::particle_sprite_emit_cs) },
			.m_update = { compiled::particle_sprite_update_cs, sizeof(compiled::particle_sprite_update_cs) },
			.m_vertex = { compiled::particle_sprite_vs, sizeof(compiled::particle_sprite_vs) },
			.m_indices = SpriteIndices.data(),
			.m_index_count = static_cast<int>(SpriteIndices.size()),
			.m_vertex_count = static_cast<int>(ParticleSpriteVertexCount),
		};
	}
}
