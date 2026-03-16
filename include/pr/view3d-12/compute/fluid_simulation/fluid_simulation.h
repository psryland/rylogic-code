//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/compute/gpu_job.h"
#include "pr/view3d-12/compute/compute_pso.h"
#include "pr/view3d-12/compute/compute_step.h"
#include "pr/view3d-12/compute/spatial_partition/spatial_partition.h"
#include "pr/view3d-12/compute/particle_collision/particle_collision.h"
#include "pr/view3d-12/model/vertex_layout.h"
#include "pr/view3d-12/texture/texture_2d.h"
#include "pr/view3d-12/utility/pix.h"

namespace pr::rdr12::compute::fluid
{
	// Particle is designed to be compatible with Vert so that the
	// same buffer can be used for both particle and vertex data.
	struct Particle
	{
		v4 pos;
		v4 col;
		v4 pad0;
		v4 pad1;
		inline static constexpr wchar_t const* Layout =
			L"struct PositionType "
			L"{ "
			L"	float4 pos; "
			L"	float4 col; "
			L"	float4 pad0; "
			L"	float4 pad1; "
			L"}";
	};
	struct Dynamics
	{
		v4 vel;     // Current velocity
		v4 accel;   // Accumulated acceleration
		v4 surface; // Nearby boundary surface

		inline static constexpr wchar_t const* Layout =
			L"struct DynamicsType "
			L"{ "
			L"	float4 vel; "
			L"	float4 accel; "
			L"	float4 surface; "
			L"}";
	};
	static_assert(sizeof(Particle) == sizeof(Vert));
	static_assert(alignof(Particle) == alignof(Vert));
	static_assert(offsetof(Particle, pos) == offsetof(Vert, m_vert));
	static_assert(offsetof(Particle, col) == offsetof(Vert, m_diff));

	enum class EWarning : uint32_t
	{
		None = 0,
		TooManyNeighbours = 1 << 0,
		_flags_enum = 0,
	};

	template <D3D12_COMMAND_LIST_TYPE QueueType = D3D12_COMMAND_LIST_TYPE_DIRECT>
	struct FluidSimulation
	{
		// Notes:
		//  Hydrostatic pressure is: P = density * gravity * depth
		using Gpu = Gpu<QueueType>;
		using GpuJob = GpuJob<QueueType>;
		using SpatialPartition = compute::spatial_partition::SpatialPartition<QueueType>;
		using ParticleCollision = compute::particle_collision::ParticleCollision<QueueType>;
		using CollisionPrim = compute::particle_collision::Prim;

		// Runtime data
		struct ColourData
		{
			Colour Spectrum[4] = {    // The colour spectrum to use
				Colour(0xFF2D50AF),
				Colour(0xFFFF0000),
				Colour(0xFFFFFF00),
				Colour(0xFFFFFFFF),
			};
			v2 Range = { 0, 1 }; // The range to interpolate the spectrum over
			int Scheme = 1;      // 0 = None, 1 = Velocity, 2 = Accel, 3 = Density, 0x80000000 = Within Probe
			int Pad = 0;
		};
		struct ProbeData
		{
			v4 Position = { 0,0,0,1 };
			Colour Colour = pr::Colour(0xFFFFFF00);
			float Radius = 0.1f;
			float Force = 0.0f;
			int Highlight = 0;
		};
		struct MapData
		{
			m4x4 MapToWorld = m4x4::Identity(); // Transform from map space to world space (including scale)
			iv2 TexDim = { 1,1 };               // The dimensions of the map texture
			int Type = 0;                       // 0 = Pressure
		};
		struct ConfigData
		{
			struct
			{
				float Radius = 0.1f; // The radius of influence for each particle
				iv2 Pad      = {};
			} Particles;
			struct
			{
				//hack v4 Gravity = { 0, -9.8f, 0, 0 }; // The acceleration due to gravity
				v4 Gravity = { 0, -10.0f, 0, 0 }; // The acceleration due to gravity
				float ForceScale = 10.0f;        // The force scaling factor
				float ForceRange = 1.0f;         // Controls the range between particles
				float ForceBalance = 0.8f;       // The position of the transition from repulsive to attractive forces
				float ForceDip = 0.05f;          // The depth of the attractive force
				float Viscosity = 0.05f;         // The viscosity scaler
				float ThermalDiffusion = 0.01f;  // The thermal diffusion rate
				int Pad = 0;
			} Dyn;                          // The dynamics variables
			ColourData Colours;             // The colour data
			v4 CullPlane = { 0, 1, 0, +1 }; // The plane below which particles are culled
			int NumParticles = 0;           // The number of particles
			int ColouringScheme = 0;        // Colouring scheme - 0 = None, 1 = Velocity, 2 = Accel, 3 = Density
		};
		struct StepOutput
		{
			struct GpuOutput
			{
				int num_particles;
				EWarning warnings;
				float p0_energy;
			};

			GpuReadbackBuffer::Allocation m_particles;
			GpuReadbackBuffer::Allocation m_dynamics;
			GpuReadbackBuffer::Allocation m_cull_results;
			GpuReadbackBuffer::Allocation m_debug_results;
			int m_particle_buffer_size;

			// The number of particles in the particle buffer (returned from Cull)
			int ParticleCount() const
			{
				// If cull hasn't been run, the number initial number of particles is the count
				auto ptr = m_cull_results.ptr<GpuOutput>();
				return ptr ? ptr[0].num_particles : m_particle_buffer_size;
			}

			// Populate 'particles' with the particle data from the last step
			template <std::invocable<Particle const*, Dynamics const*> Out>
			void ReadParticles(int start, int count, Out out) const
			{
				if (start < 0 || start + count > m_particle_buffer_size)
					throw std::range_error("Invalid particle buffer range");
				if (count != 0 && (m_particles.ptr<Particle>() == nullptr || m_dynamics.ptr<Dynamics>() == nullptr))
					throw std::runtime_error("You must use 'readback' to access particle data in sys-memory");

				out(m_particles.ptr<Particle>() + start, m_dynamics.ptr<Dynamics>() + start);
			}

			// Return the GpuOutput object returned from the debugging call
			GpuOutput const& DebugResults() const
			{
				static const GpuOutput s_zero = {};
				auto ptr = m_debug_results.ptr<GpuOutput>();
				return ptr ? *ptr : s_zero;
			}
		};

		// Initialisation data
		struct Setup
		{
			int ParticleCapacity;                            // The maximum number of particles to expect (to set buffer sizes)
			ConfigData Config;                               // Runtime configuration for the fluid simulation
			std::span<Particle const> ParticleInitData = {}; // Initialisation data for the particles
			std::span<Dynamics const> DynamicsInitData = {}; // Initialisation data for the dynamics

			bool valid() const noexcept
			{
				return
					Config.NumParticles >= 0 && Config.NumParticles <= ParticleCapacity &&
					(ParticleInitData.empty() || isize(ParticleInitData) == Config.NumParticles) &&
					(DynamicsInitData.empty() || isize(DynamicsInitData) == Config.NumParticles);
			}
		};

		Gpu* m_gpu;                           // The gpu instance to use to run the compute shader
		ComputeStep m_cs_apply_forces;        // Calculate the forces acting on each particle position
		ComputeStep m_cs_apply_probe;         // Apply forces from the probe
		ComputeStep m_cs_cull_particles;      // Remove particles below a plane
		ComputeStep m_cs_colour;              // Apply colours to the particles
		ComputeStep m_cs_gen_map;             // Populate a texture with a map of a property
		ComputeStep m_cs_debugging;           // Debug testing
		D3DPtr<ID3D12Resource> m_r_particles; // The buffer of the particles (includes position/colour/norm(velocity))
		D3DPtr<ID3D12Resource> m_r_dynamics;  // The buffer of the particles (includes position/colour/norm(velocity))
		D3DPtr<ID3D12Resource> m_r_output;    // The buffer that receives the output of the compute shader
		ParticleCollision m_collision;        // The collision resolution for the fluid
		SpatialPartition m_spatial;           // Spatial partitioning of the particles
		int m_capacity;                       // The maximum number of particles
		int m_frame;                          // Frame counter

		// Runtime configurable data
		ConfigData Config; // The configuration data for the fluid simulation
		StepOutput Output; // Read back data from the last executed step

		explicit FluidSimulation(Gpu& gpu)
			: m_gpu(&gpu)
			, m_cs_apply_forces()
			, m_cs_apply_probe()
			, m_cs_colour()
			, m_cs_gen_map()
			, m_cs_debugging()
			, m_r_particles()
			, m_r_dynamics()
			, m_r_output()
			, m_collision(gpu, Particle::Layout, Dynamics::Layout)
			, m_spatial(gpu, Particle::Layout)
			, m_capacity()
			, m_frame()
			, Config()
			, Output()
		{
			CreateComputeSteps(Particle::Layout, Dynamics::Layout);
		}

		// Set the initial state of the simulation (spatial partition, colours, etc.)
		void Init(GpuJob& job, Setup const& fs_setup, ParticleCollision::Setup const& pc_setup, SpatialPartition::Setup const& sp_setup)
		{
			assert(fs_setup.valid());

			// Save the config
			Config = fs_setup.Config;

			// Create resource buffers
			CreateResourceBuffers(job, fs_setup);

			// Reset the collision primitives
			m_collision.Init(job, pc_setup);

			// Reset the spatial partitioning
			m_spatial.Init(job, sp_setup);

			// Make the particle buffer accessible in the compute shader
			ParticleBufferAsUAV(job, true);

			// Update the spatial partition
			m_spatial.Update(job, Config.NumParticles, m_r_particles, true);

			// Cull any particles that are initially out of bounds
			CullParticles(job);

			// Make the particle buffer a vertex buffer again
			ParticleBufferAsUAV(job, false);

			// Run the compute jobs
			job.Run();

			Config.NumParticles = Output.ParticleCount();
		}

		// Convert the particle buffer to a compute resource or a vertex buffer
		void ParticleBufferAsUAV(GpuJob& job, bool for_compute)
		{
			auto state = for_compute
				? D3D12_RESOURCE_STATE_UNORDERED_ACCESS
				: D3D12_RESOURCE_STATE_VERTEX_AND_CONSTANT_BUFFER;

			job.m_barriers.Transition(m_r_particles.get(), state);
		}

		// Advance the simulation forward in time by 'elapsed_s' seconds
		void Step(GpuJob& job, float elapsed_s, bool read_back)
		{
			++m_frame;
			Output = {};

			// No-op if there are no particles
			if (Config.NumParticles == 0)
				return;

			// Save the size of the particle buffer used in this step
			Output.m_particle_buffer_size = Config.NumParticles;

			// Make the particle vertex buffer accessible in the compute shader
			ParticleBufferAsUAV(job, true);

			// Apply forces to each particle
			ApplyForces(job, elapsed_s);

			// Integrate velocity and position (with collision)
			m_collision.Integrate(job, elapsed_s, Config.NumParticles, Config.Particles.Radius, m_r_particles, m_r_dynamics);
			m_collision.DetectBoundaries(job, Config.NumParticles, Config.Particles.Radius, m_r_particles, m_r_dynamics);
			m_collision.CullDeadParticles(job, Config.NumParticles, m_r_particles);

			// Update the spatial partitioning of the particles
			m_spatial.Update(job, Config.NumParticles, m_r_particles, false);

			// Read back the particle and dynamics buffer
			if (read_back)
				ReadParticles(job, 0, Output.m_particle_buffer_size, &Output.m_particles, &Output.m_dynamics);

			// Cull any particles that are initially out of bounds
			CullParticles(job);

			// Make the particle buffer a vertex buffer again
			ParticleBufferAsUAV(job, false);
		}

		// Apply forces from a probe
		void ApplyProbeForces(GpuJob& job, ProbeData const& probe)
		{
			if (Config.NumParticles == 0)
				return;

			// Make the particle vertex buffer accessible in the compute shader
			ParticleBufferAsUAV(job, true);

			// Apply the probe forces
			ApplyProbe(job, probe);

			// Make the particle buffer a vertex buffer again
			ParticleBufferAsUAV(job, false);
		}

		// Update the particle colours without stepping the simulation
		void UpdateColours(GpuJob& job, ColourData const& colours)
		{
			if (Config.NumParticles == 0)
				return;

			// Make the particle vertex buffer accessible in the compute shader
			ParticleBufferAsUAV(job, true);

			// Set particle colours
			ColourParticles(job, colours);

			// Make the particle buffer a vertex buffer again
			ParticleBufferAsUAV(job, false);
		}

		// Read the particle position buffer
		void ReadParticles(GpuJob& job, int start, int count, GpuReadbackBuffer::Allocation* particles, GpuReadbackBuffer::Allocation* dynamics)
		{
			if (start + count < Config.NumParticles)
				throw std::range_error("Invalid particle buffer range");
			if (count == 0)
				return;

			pix::BeginEvent(job.m_cmd_list.get(), 0xFF4988F2, "FluidSim::ReadParticles");

			if (particles)
			{
				auto previous_state = job.m_cmd_list.ResState(m_r_particles.get());
				job.m_barriers.Transition(m_r_particles.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
				job.m_barriers.Commit();
			
				// Allocate read back buffer space and read from the particle buffer
				auto buf = job.m_readback.Alloc(count * sizeof(Particle), alignof(Particle));
				job.m_cmd_list.CopyBufferRegion(buf, m_r_particles.get(), start * sizeof(Particle));
				*particles = std::move(buf);

				job.m_barriers.Transition(m_r_particles.get(), previous_state.Mip0State());
				job.m_barriers.Commit();
			}

			if (dynamics)
			{
				auto previous_state = job.m_cmd_list.ResState(m_r_dynamics.get());
				job.m_barriers.Transition(m_r_dynamics.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
				job.m_barriers.Commit();
			
				// Allocate read back buffer space and read from the dynamics buffer
				auto buf = job.m_readback.Alloc(count * sizeof(Dynamics), alignof(Dynamics));
				job.m_cmd_list.CopyBufferRegion(buf, m_r_dynamics.get(), start * sizeof(Dynamics));
				*dynamics = std::move(buf);

				job.m_barriers.Transition(m_r_dynamics.get(), previous_state.Mip0State());
				job.m_barriers.Commit();
			}

			pix::EndEvent(job.m_cmd_list.get());
		}

		// Write particles into the particle buffer
		void WriteParticles(GpuJob& job, int start, std::span<Particle const> particles, std::span<Particle const> dynamics)
		{
			if (particles.empty() && dynamics.empty())
				return;

			pix::BeginEvent(job.m_cmd_list.get(), 0xFF4988F2, "FluidSim::WriteParticles");

			if (!particles.empty())
			{
				auto previous_state = job.m_cmd_list.ResState(m_r_particles.get());
				job.m_barriers.Transition(m_r_particles.get(), D3D12_RESOURCE_STATE_COPY_DEST);
				job.m_barriers.Commit();

				// Allocate upload buffer space and copy from 'particles' into the upload buffer, then into the particle buffer
				auto buf = job.m_upload.Alloc(isize(particles) * sizeof(Particle), alignof(Particle));
				memcpy(buf.ptr<Particle>(), particles.data(), particles.size() * sizeof(Particle));
				job.m_cmd_list.CopyBufferRegion(m_r_particles.get(), start * sizeof(Particle), buf);

				job.m_barriers.Transition(m_r_particles.get(), previous_state.Mip0State());
				job.m_barriers.Commit();
			}

			if (!dynamics.empty())
			{
				auto previous_state = job.m_cmd_list.ResState(m_r_dynamics.get());
				job.m_barriers.Transition(m_r_dynamics.get(), D3D12_RESOURCE_STATE_COPY_DEST);
				job.m_barriers.Commit();

				// Allocate upload buffer space and copy from 'dynamics' into the upload buffer, then into the dynamics buffer
				auto buf = job.m_upload.Alloc(isize(dynamics) * sizeof(Dynamics), alignof(Dynamics));
				memcpy(buf.ptr<Dynamics>(), dynamics.data(), dynamics.size() * sizeof(Dynamics));
				job.m_cmd_list.CopyBufferRegion(m_r_dynamics.get(), start * sizeof(Dynamics), buf);

				job.m_barriers.Transition(m_r_dynamics.get(), previous_state.Mip0State());
				job.m_barriers.Commit();
			}

			pix::EndEvent(job.m_cmd_list.get());
		}

		// Create a map of some value over the map area
		void GenerateMap(GpuJob& job, Texture2DPtr tex_map, MapData const& map_data, ColourData const& colour_data)
		{
			DoGenerateMap(job, tex_map, map_data, colour_data);
		}

		// Run the debugging function
		void Debugging(GpuJob& job, float time_step, ProbeData const& probe)
		{
			if (Config.NumParticles == 0)
				return;

			// Make the particle vertex buffer accessible in the compute shader
			ParticleBufferAsUAV(job, true);

			// Apply the probe forces
			DoDebugging(job, time_step, probe);

			// Make the particle buffer a vertex buffer again
			ParticleBufferAsUAV(job, false);
		}

	private:

		static constexpr int ThreadGroupSize = 1024;
		using float4x4 = pr::m4x4;
		using float4 = pr::v4;
		using float3 = pr::v3;
		using float2 = pr::v2;
		using int2 = pr::iv2;

		struct EReg
		{
			inline static constexpr ECBufReg Fluid = ECBufReg::b0;
			inline static constexpr ECBufReg Probe = ECBufReg::b1;
			inline static constexpr ECBufReg Cull = ECBufReg::b0;
			inline static constexpr ECBufReg Colours = ECBufReg::b0;
			inline static constexpr ECBufReg Map = ECBufReg::b0;

			inline static constexpr EUAVReg Particles = EUAVReg::u0;
			inline static constexpr EUAVReg Dynamics = EUAVReg::u1;
			inline static constexpr EUAVReg Spatial = EUAVReg::u2;
			inline static constexpr EUAVReg IdxStart = EUAVReg::u3;
			inline static constexpr EUAVReg IdxCount = EUAVReg::u4;
			inline static constexpr EUAVReg Output = EUAVReg::u5;
			inline static constexpr EUAVReg TexMap = EUAVReg::u6;
		};

		struct cbFluidSim
		{
			float4 Gravity;         // The acceleration due to gravity

			int Dimensions;         // 2D or 3D simulation
			int NumParticles;       // The number of particles
			int CellCount;          // The number of grid cells in the spatial partition
			float GridScale;        // The scale factor for the spatial partition grid

			float ParticleRadius;   // The radius of influence for each particle
			float TimeStep;         // Leap-frog time step
			float ThermalDiffusion; // The thermal diffusion rate
			int RandomSeed;         // Seed value for the RNG

			float ForceScale;       // The force scaling factor
			float ForceRange;       // Controls the range between particles
			float ForceBalance;     // The position of the transition from repulsive to attractive forces
			float ForceDip;         // The depth of the attractive force

			float Viscosity;        // The viscosity scaler
			float3 pad;
		};
		struct cbProbeData
		{
			float4 Position;      // The position of the probe
			float Radius;         // The radius of the probe
			float Force;          // The force that the probe applies
			int NumParticles;     // The number of particles in the 'm_particles' buffer
		};
		struct cbCullData
		{
			int NumParticles;       // The number of particles
			int CellCount;          // The number of grid cells in the spatial partition
		};
		struct cbColourData
		{
			float4 Spectrum[4];     // The colour scale to use
			float2 Range;           // Scales [0,1] to the colour range
			int NumParticles;       // The number of particles in the 'm_particles' buffer
			int Scheme;             // 0 = None, 1 = Velocity, 2 = Accel, 3 = Density
		};
		struct cbMapData
		{
			float4x4 MapToWorld;    // Transform from map space to world space (including scale)
			float4 Spectrum[4];     // The colour scale to use
			float2 Range;           // Scales [0,1] to the colour range
			int2 TexDim;            // The dimensions of the map texture

			int Type;               // 0 = Pressure
			int Dimensions;         // 2D or 3D simulation
			int CellCount;          // The number of grid cells in the spatial partition
			float GridScale;        // The scale factor for the spatial partition grid

			float ForceScale;       // The force scaling factor
			float ForceRange;       // Controls the range between particles
			float ForceBalance;     // The position of the transition from repulsive to attractive forces
			float ForceDip;         // The depth of the attractive force

			float ParticleRadius;   // The particle radius
			float3 pad;
		};

		using OutputData = StepOutput::GpuOutput;

		// Create constant buffer data for the fluid simulation
		cbFluidSim FluidSimCBuf(float time_step) const
		{
			return cbFluidSim{
				.Gravity = Config.Dyn.Gravity,
				.Dimensions = m_collision.Config.SpatialDimensions,
				.NumParticles = Config.NumParticles,
				.CellCount = m_spatial.Config.CellCount,
				.GridScale = m_spatial.Config.GridScale,
				.ParticleRadius = Config.Particles.Radius,
				.TimeStep = time_step,
				.ThermalDiffusion = Config.Dyn.ThermalDiffusion,
				.RandomSeed = m_frame,
				.ForceScale = Config.Dyn.ForceScale,
				.ForceRange = Config.Dyn.ForceRange,
				.ForceBalance = Config.Dyn.ForceBalance,
				.ForceDip = Config.Dyn.ForceDip,
				.Viscosity = Config.Dyn.Viscosity,
				.pad = {},
			};
		};
		cbProbeData ProbeCBuf(ProbeData const& probe) const
		{
			return cbProbeData{
				.Position = probe.Position,
				.Radius = probe.Radius,
				.Force = probe.Force,
				.NumParticles = Config.NumParticles,
			};
		}
		cbCullData CullCBuf() const
		{
			return cbCullData{
				.NumParticles = Config.NumParticles,
				.CellCount = m_spatial.Config.CellCount,
			};
		}
		cbColourData ColoursCBuf(ColourData const& colours) const
		{
			return cbColourData{
				.Spectrum = {
					To<v4>(colours.Spectrum[0]),
					To<v4>(colours.Spectrum[1]),
					To<v4>(colours.Spectrum[2]),
					To<v4>(colours.Spectrum[3]),
				},
				.Range = colours.Range,
				.NumParticles = Config.NumParticles,
				.Scheme = colours.Scheme,
			};
		}
		cbMapData MapCBuf(MapData const& map_data, ColourData const& colours) const
		{
			return cbMapData{
				.MapToWorld = map_data.MapToWorld,
				.Spectrum = { 
					To<v4>(colours.Spectrum[0]),
					To<v4>(colours.Spectrum[1]),
					To<v4>(colours.Spectrum[2]),
					To<v4>(colours.Spectrum[3]),
				},
				.Range = colours.Range,
				.TexDim = map_data.TexDim,
				.Type = map_data.Type,
				.Dimensions = m_collision.Config.SpatialDimensions,
				.CellCount = m_spatial.Config.CellCount,
				.GridScale = m_spatial.Config.GridScale,
				.ForceScale = Config.Dyn.ForceScale,
				.ForceRange = Config.Dyn.ForceRange,
				.ForceBalance = Config.Dyn.ForceBalance,
				.ForceDip = Config.Dyn.ForceDip,
				.ParticleRadius = Config.Particles.Radius,
				.pad = {},
			};
		}

		// Compile the compute shaders
		void CreateComputeSteps(std::wstring_view position_layout, std::wstring_view dynamics_layout)
		{
			ShaderCompiler compiler = ShaderCompiler{}
				.Source(resource::Read<char>(L"FLUID_SIMULATION_HLSL", L"TEXT"))
				.Includes({ new ResourceIncludeHandler, true })
				.Define(L"POSITION_TYPE", position_layout)
				.Define(L"DYNAMICS_TYPE", dynamics_layout)
				.Define(L"THREAD_GROUP_SIZE", std::to_wstring(ThreadGroupSize))
				.ShaderModel(L"cs_6_6")
				.Optimise();

			// Apply Forces
			{
				auto bytecode = compiler.EntryPoint(L"ApplyForces").Compile();
				m_cs_apply_forces.m_sig = RootSig(ERootSigFlags::ComputeOnly)
					.U32<cbFluidSim>(EReg::Fluid)
					.UAV(EReg::Particles)
					.UAV(EReg::Dynamics)
					.UAV(EReg::Spatial)
					.UAV(EReg::IdxStart)
					.UAV(EReg::IdxCount)
					.UAV(EReg::Output)
					.Create(*m_gpu, "Fluid:ApplyForcesSig");
				m_cs_apply_forces.m_pso = ComputePSO(m_cs_apply_forces.m_sig.get(), bytecode)
					.Create(*m_gpu, "Fluid:ApplyForcesPSO");
			}

			// Apply Probe
			{
				auto bytecode = compiler.EntryPoint(L"ApplyProbe").Compile();
				m_cs_apply_probe.m_sig = RootSig(ERootSigFlags::ComputeOnly)
					.U32<cbProbeData>(EReg::Probe)
					.UAV(EReg::Particles)
					.UAV(EReg::Dynamics)
					.Create(*m_gpu, "Fluid:ApplyProbeSig");
				m_cs_apply_probe.m_pso = ComputePSO(m_cs_apply_probe.m_sig.get(), bytecode)
					.Create(*m_gpu, "Fluid:ApplyProbePSO");
			}

			// Cull particles
			{
				auto bytecode = compiler.EntryPoint(L"CullParticles").Compile();
				m_cs_cull_particles.m_sig = RootSig(ERootSigFlags::ComputeOnly)
					.U32<cbCullData>(EReg::Cull)
					.UAV(EReg::Particles)
					.UAV(EReg::Dynamics)
					.UAV(EReg::Spatial)
					.UAV(EReg::IdxStart)
					.UAV(EReg::IdxCount)
					.UAV(EReg::Output)
					.Create(*m_gpu, "Fluid:CullParticlesSig");
				m_cs_cull_particles.m_pso = ComputePSO(m_cs_cull_particles.m_sig.get(), bytecode)
					.Create(*m_gpu, "Fluid:CullParticlesPSO");
			}

			// Colour
			{
				auto bytecode = compiler.EntryPoint(L"ColourParticles").Compile();
				m_cs_colour.m_sig = RootSig(ERootSigFlags::ComputeOnly)
					.U32<cbColourData>(EReg::Colours)
					.UAV(EReg::Particles)
					.UAV(EReg::Dynamics)
					.Create(*m_gpu, "Fluid:ColourParticlesSig");
				m_cs_colour.m_pso = ComputePSO(m_cs_colour.m_sig.get(), bytecode)
					.Create(*m_gpu, "Fluid:ColourParticlesPSO");
			}

			// Generate Map
			{
				auto bytecode = compiler.EntryPoint(L"GenerateMap").Compile();
				m_cs_gen_map.m_sig = RootSig(ERootSigFlags::ComputeOnly)
					.CBuf(EReg::Map)
					.UAV(EReg::Particles)
					.UAV(EReg::Dynamics)
					.UAV(EReg::Spatial)
					.UAV(EReg::IdxStart)
					.UAV(EReg::IdxCount)
					.UAV(EReg::TexMap, 1)
					.Create(*m_gpu, "Fluid:GenerateMapSig");
				m_cs_gen_map.m_pso = ComputePSO(m_cs_gen_map.m_sig.get(), bytecode)
					.Create(*m_gpu, "Fluid:GenerateMapPSO");
			}

			// Debugging
			{
				auto bytecode = compiler.EntryPoint(L"Debugging").Compile();
				m_cs_debugging.m_sig = RootSig(ERootSigFlags::ComputeOnly)
					.U32<cbFluidSim>(EReg::Fluid)
					.U32<cbProbeData>(EReg::Probe)
					.UAV(EReg::Particles)
					.UAV(EReg::Dynamics)
					.UAV(EReg::Spatial)
					.UAV(EReg::IdxStart)
					.UAV(EReg::IdxCount)
					.UAV(EReg::Output)
					.Create(*m_gpu, "Fluid:DebuggingSig");
				m_cs_debugging.m_pso = ComputePSO(m_cs_debugging.m_sig.get(), bytecode)
					.Create(*m_gpu, "Fluid:DebuggingPSO");
			}
		}

		// Create the resource buffers
		void CreateResourceBuffers(GpuJob& job, Setup const& setup)
		{
			m_capacity = setup.ParticleCapacity;

			// Create the particle (vertex) buffer
			{
				ResDesc desc = ResDesc::VBuf<Particle>(setup.ParticleCapacity, setup.ParticleInitData).usage(EUsage::UnorderedAccess);
				m_r_particles = m_gpu->CreateResource(desc, job.m_cmd_list, "Fluid:ParticlePositions");
			}

			// Create the particle dynamics buffer
			{
				ResDesc desc = ResDesc::Buf<Dynamics>(setup.ParticleCapacity, setup.DynamicsInitData).def_state(D3D12_RESOURCE_STATE_UNORDERED_ACCESS).usage(EUsage::UnorderedAccess);
				m_r_dynamics = m_gpu->CreateResource(desc, job.m_cmd_list, "Fluid:ParticleDynamics");
			}

			// Create the output buffer
			{
				ResDesc desc = ResDesc::Buf<OutputData>(1, {}).def_state(D3D12_RESOURCE_STATE_UNORDERED_ACCESS).usage(EUsage::UnorderedAccess);
				m_r_output = m_gpu->CreateResource(desc, job.m_cmd_list, "Fluid:Output");
			}
		}

		// Apply forces to each particle
		void ApplyForces(GpuJob& job, float time_step)
		{
			pix::BeginEvent(job.m_cmd_list.get(), 0xFF3F75FF, "FluidSim::ApplyForces");

			auto cb_params = FluidSimCBuf(time_step);

			job.m_barriers.Commit();
		
			job.m_cmd_list.SetPipelineState(m_cs_apply_forces.m_pso.get());
			job.m_cmd_list.SetComputeRootSignature(m_cs_apply_forces.m_sig.get());
			job.m_cmd_list.AddComputeRoot32BitConstants(cb_params);
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_particles->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_dynamics->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_spatial.m_spatial->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_spatial.m_idx_start->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_spatial.m_idx_count->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_output->GetGPUVirtualAddress());
			job.m_cmd_list.Dispatch(DispatchCount({ cb_params.NumParticles, 1, 1 }, { ThreadGroupSize, 1, 1 }));
		
			job.m_barriers.UAV(m_r_particles.get());
			job.m_barriers.UAV(m_r_dynamics.get());
			job.m_barriers.UAV(m_r_output.get());

			pix::EndEvent(job.m_cmd_list.get());
		}

		// Apply probe forces to the particles
		void ApplyProbe(GpuJob& job, ProbeData const& probe)
		{
			pix::BeginEvent(job.m_cmd_list.get(), 0xFF4CFF4F, "FluidSim::ApplyProbe");

			auto cb_probe = ProbeCBuf(probe);

			job.m_barriers.Commit();

			job.m_cmd_list.SetPipelineState(m_cs_apply_probe.m_pso.get());
			job.m_cmd_list.SetComputeRootSignature(m_cs_apply_probe.m_sig.get());
			job.m_cmd_list.AddComputeRoot32BitConstants(cb_probe);
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_particles->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_dynamics->GetGPUVirtualAddress());
			job.m_cmd_list.Dispatch(DispatchCount({ cb_probe.NumParticles, 1, 1 }, { ThreadGroupSize, 1, 1 }));

			job.m_barriers.UAV(m_r_particles.get());
			job.m_barriers.UAV(m_r_dynamics.get());

			pix::EndEvent(job.m_cmd_list.get());
		}

		// Cull particles that fall out of the world
		void CullParticles(GpuJob& job)
		{
			pix::BeginEvent(job.m_cmd_list.get(), 0xFF993020, "FluidSim::CullParticles");

			auto cb_params = CullCBuf();

			job.m_barriers.Commit();

			job.m_cmd_list.SetPipelineState(m_cs_cull_particles.m_pso.get());
			job.m_cmd_list.SetComputeRootSignature(m_cs_cull_particles.m_sig.get());
			job.m_cmd_list.AddComputeRoot32BitConstants(cb_params);
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_particles->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_dynamics->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_spatial.m_spatial->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_spatial.m_idx_start->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_spatial.m_idx_count->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_output->GetGPUVirtualAddress());
			job.m_cmd_list.Dispatch(DispatchCount({ cb_params.NumParticles, 1, 1 }, { ThreadGroupSize, 1, 1 }));

			job.m_barriers.UAV(m_r_particles.get());
			job.m_barriers.UAV(m_r_dynamics.get());
			job.m_barriers.UAV(m_r_output.get());

			// Read back the number of particles
			job.m_barriers.Transition(m_r_output.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
			job.m_barriers.Commit();
			{
				auto buf = job.m_readback.template Alloc<OutputData>(1);
				job.m_cmd_list.CopyBufferRegion(buf, m_r_output.get(), 0);
				Output.m_cull_results = std::move(buf);
			}
			job.m_barriers.Transition(m_r_output.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);

			pix::EndEvent(job.m_cmd_list.get());
		}

		// Apply colours to the particles
		void ColourParticles(GpuJob& job, ColourData const& colours)
		{
			pix::BeginEvent(job.m_cmd_list.get(), 0xFFFB9BFF, "FluidSim::ColourParticles");

			auto cb_colours = ColoursCBuf(colours);

			job.m_barriers.Commit();

			job.m_cmd_list.SetPipelineState(m_cs_colour.m_pso.get());
			job.m_cmd_list.SetComputeRootSignature(m_cs_colour.m_sig.get());
			job.m_cmd_list.AddComputeRoot32BitConstants(cb_colours);
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_particles->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_dynamics->GetGPUVirtualAddress());
			job.m_cmd_list.Dispatch(DispatchCount({ cb_colours.NumParticles, 1, 1 }, { ThreadGroupSize, 1, 1 }));

			job.m_barriers.UAV(m_r_particles.get());

			pix::EndEvent(job.m_cmd_list.get());
		}

		// Create a map of some value over the map area
		void DoGenerateMap(GpuJob& job, Texture2DPtr tex_map, MapData const& map_data, ColourData const& colour_data)
		{
			pix::BeginEvent(job.m_cmd_list.get(), 0xFFF0FF56, "FluidSim::GenerateMap");

			auto cb_map = MapCBuf(map_data, colour_data);

			job.m_barriers.Transition(tex_map->m_res.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			job.m_barriers.Commit();

			job.m_cmd_list.SetPipelineState(m_cs_gen_map.m_pso.get());
			job.m_cmd_list.SetComputeRootSignature(m_cs_gen_map.m_sig.get());
			job.m_cmd_list.AddComputeRootConstantBufferView(job.m_upload.Add(cb_map, D3D12_CONSTANT_BUFFER_DATA_PLACEMENT_ALIGNMENT, true));
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_particles->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_dynamics->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_spatial.m_spatial->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_spatial.m_idx_start->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_spatial.m_idx_count->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootDescriptorTable(job.m_view_heap.Add(tex_map->m_uav));
			job.m_cmd_list.Dispatch(DispatchCount({ cb_map.TexDim, 1 }, { 32, 32, 1 }));

			job.m_barriers.UAV(tex_map->m_res.get());
			job.m_barriers.Transition(tex_map->m_res.get(), D3D12_RESOURCE_STATE_ALL_SHADER_RESOURCE);
			job.m_barriers.Commit();

			job.Run();

			pix::EndEvent(job.m_cmd_list.get());
		}

		// Run the debugging function
		void DoDebugging(GpuJob& job, float time_step, ProbeData const& probe)
		{
			pix::BeginEvent(job.m_cmd_list.get(), 0xFFF0FF56, "FluidSim::Debugging");

			auto cb_sim = FluidSimCBuf(time_step);
			auto cb_probe = ProbeCBuf(probe);

			job.m_barriers.Commit();

			job.m_cmd_list.SetPipelineState(m_cs_debugging.m_pso.get());
			job.m_cmd_list.SetComputeRootSignature(m_cs_debugging.m_sig.get());
			job.m_cmd_list.AddComputeRoot32BitConstants(cb_sim);
			job.m_cmd_list.AddComputeRoot32BitConstants(cb_probe);
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_particles->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_dynamics->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_spatial.m_spatial->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_spatial.m_idx_start->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_spatial.m_idx_count->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_r_output->GetGPUVirtualAddress());
			job.m_cmd_list.Dispatch(DispatchCount({ 1, 1, 1 }, { 1, 1, 1 }));

			job.m_barriers.UAV(m_r_output.get());

			// Read back the output
			job.m_barriers.Transition(m_r_output.get(), D3D12_RESOURCE_STATE_COPY_SOURCE);
			job.m_barriers.Commit();
			{
				auto buf = job.m_readback.template Alloc<OutputData>(1);
				job.m_cmd_list.CopyBufferRegion(buf, m_r_output.get(), 0);
				Output.m_debug_results = std::move(buf);
			}
			job.m_barriers.Transition(m_r_output.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS);

			pix::EndEvent(job.m_cmd_list.get());
		}
	};
}
