using System;
using System.Runtime.InteropServices;
using Rylogic.Maths;
#if PR_UNITTESTS
using Rylogic.UnitTests;
#endif

namespace Rylogic.Gfx;

public sealed partial class View3d
{
	/// <summary>How a particle batch blends with the scene. Matches 'view3d::EParticleBlend'.</summary>
	public enum EParticleBlend : int
	{
		/// <summary>Alpha fragments sorted per pixel with the scene's other transparent geometry. Suits sparse particles that must sort exactly.</summary>
		Sorted = 0,

		/// <summary>Weighted blended transparency. Suits dense, soft, overlapping particles such as smoke and dust.</summary>
		Weighted = 1,

		/// <summary>Opaque, depth-tested and depth-written. Suits solid particles such as chips and leaves.</summary>
		Opaque = 2,
	}

	/// <summary>The stock particle effects. Matches 'view3d::EParticlePreset'. See pr/game-systems/particles/particle_presets.h.</summary>
	public enum EParticlePreset : int
	{
		/// <summary>Flickering flames that rise and shrink.</summary>
		Fire = 0,

		/// <summary>Streaked sparks thrown up and falling under gravity.</summary>
		Sparks = 1,

		/// <summary>Slow grey puffs that rise, grow and drift with the wind.</summary>
		Smoke = 2,

		/// <summary>A burst of dust that billows out, rises a little and settles.</summary>
		Dust = 3,

		/// <summary>Streaked drops that fall with the wind and die where they land.</summary>
		Rain = 4,

		/// <summary>Flakes that drift down with the wind and die where they land.</summary>
		Snow = 5,

		/// <summary>Leaves that flutter and tumble down with the wind, and settle where they land.</summary>
		Leaves = 6,

		/// <summary>A distant flock of birds circling, flapping and gliding.</summary>
		Birds = 7,

		/// <summary>A burst of tumbling wood chips, such as from chopping a tree.</summary>
		WoodChips = 8,

		/// <summary>Pulses of an expanding ring, such as a beacon.</summary>
		Marker = 9,

		/// <summary>Droplets thrown up from breaking wave crests. Needs a bound water surface.</summary>
		Spray = 10,

		/// <summary>Foam puffs that swell up on breaking wave crests and ride the waves. Needs a bound water surface.</summary>
		Froth = 11,
	}

	/// <summary>The system-owned settings of a particle emitter. The caller's parameters are passed separately. Matches 'view3d::ParticleEmitter'.</summary>
	[StructLayout(LayoutKind.Sequential)]
	public struct ParticleEmitter
	{
		/// <summary>Particles per simulated second. Finite and non-negative.</summary>
		public float Rate;

		/// <summary>Random seed; emitters with equal seeds and parameters make equal particles.</summary>
		public uint Seed;

		// A Win32 BOOL.
		private int m_active;

		/// <summary>Create emitter settings.</summary>
		public ParticleEmitter(float rate, uint seed, bool active)
		{
			Rate = rate;
			Seed = seed;
			m_active = active ? 1 : 0;
		}

		/// <summary>Inactive emitters make no particles, and discard bursts requested while inactive.</summary>
		public bool Active
		{
			readonly get
			{
				return m_active != 0;
			}
			set
			{
				m_active = value ? 1 : 0;
			}
		}
	}

	/// <summary>
	/// The pool usage of a particle type after its last step. Counts of particles requested, admitted, and dropped, and of plane collisions,
	/// accumulate from the last reset. Matches 'view3d::ParticleStats'.
	/// </summary>
	[StructLayout(LayoutKind.Sequential)]
	public readonly struct ParticleStats
	{
		/// <summary>Free and live slots of the type's pool.</summary>
		public readonly uint Free;
		public readonly uint Live;

		/// <summary>Births asked for, admitted into the pool, and dropped because the pool was full.</summary>
		public readonly uint Requested;
		public readonly uint Admitted;
		public readonly uint Dropped;

		/// <summary>Plane contacts resolved.</summary>
		public readonly uint Contacts;

		/// <summary>Particles pushed out of planes that they started inside.</summary>
		public readonly uint Overlaps;

		/// <summary>Particles stopped because their contact budget was spent.</summary>
		public readonly uint Exhausted;
	}

	/// <summary>The environment that stepped particle types move in, in the space of the particles' positions. Matches 'view3d::ParticleEnvironment'.</summary>
	[StructLayout(LayoutKind.Sequential)]
	public struct ParticleEnvironment
	{
		/// <summary>xyz = Acceleration (m/s²) of every stepped particle. w unused. Finite.</summary>
		public v4 Gravity;

		/// <summary>xyz = Uniform wind velocity (m/s), added to the wind grid. w unused. Finite.</summary>
		public v4 Wind;

		/// <summary>Most plane contacts resolved per particle per step. In [0, 64].</summary>
		public int CollisionIterations;
	}

	/// <summary>A static plane that stepped particles collide with. Particles are allowed in the half-space dot(n, p) >= d. Matches 'view3d::ParticlePlane'.</summary>
	[StructLayout(LayoutKind.Sequential)]
	public struct ParticlePlane
	{
		/// <summary>xyz = Unit normal n, pointing into the allowed half-space. w = Offset d. Finite.</summary>
		public v4 Plane;

		/// <summary>Fraction of the normal velocity kept after a contact. In [0, 1].</summary>
		public float Restitution;

		/// <summary>Fraction of the tangential velocity removed by a contact. In [0, 1].</summary>
		public float Damping;

		/// <summary>Create a plane.</summary>
		public ParticlePlane(v4 plane, float restitution, float damping)
		{
			Plane = plane;
			Restitution = restitution;
			Damping = damping;
		}
	}

	/// <summary>
	/// The simulation shaders of a particle type: compiled compute shaders built on pr/game-systems/particles/particle_kernels.hlsli, with the
	/// entry points 'ParticleEmitCS' and 'ParticleUpdateCS'.
	/// </summary>
	public ref struct ParticleSimShaders
	{
		public ReadOnlySpan<byte> Emit;
		public ReadOnlySpan<byte> Update;
	}

	/// <summary>
	/// The drawing shaders of a particle batch, and the index list that its vertex shader draws for each render record. See 'view3d::ParticleDrawShaders'.
	/// </summary>
	public ref struct ParticleDrawShaders
	{
		/// <summary>Procedural forward vertex shader, reading one render record per instance.</summary>
		public ReadOnlySpan<byte> Vertex;

		/// <summary>Triangle list of logical vertex ids drawn for each record. A positive multiple of 3.</summary>
		public ReadOnlySpan<uint> Indices;

		/// <summary>One more than the largest logical vertex id in 'Indices'.</summary>
		public int VertexCount;

		/// <summary>The surface outputs that 'Vertex' generates. Must include Vert.</summary>
		public EGeom Geom;

		/// <summary>Optional data for each of the 'VertexCount' logical vertices, 'VertexStride' bytes each. Empty for none.</summary>
		public ReadOnlySpan<byte> VertexData;
		public int VertexStride;
	}

	/// <summary>Fixed inputs of a particle batch. The data is copied. See 'view3d::ParticleBatchOptions'.</summary>
	public ref struct ParticleBatchOptions
	{
		/// <summary>The batch's drawing shaders.</summary>
		public ParticleDrawShaders Shaders;

		/// <summary>Most records drawn at once. At least the sum of the capacities of the batch's types.</summary>
		public int Capacity;

		/// <summary>Bytes per render record.</summary>
		public int RecordStride;

		/// <summary>Optional constants seen by the vertex shader. Empty for none.</summary>
		public ReadOnlySpan<byte> Constants;

		/// <summary>How the batch blends with the scene.</summary>
		public EParticleBlend Blend;

		/// <summary>Bounds of every particle, in the object's space, for culling.</summary>
		public BBox Bounds;

		/// <summary>Optional diffuse texture, and its sampler (null for the default). The batch keeps its own reference.</summary>
		public Texture? Texture;
		public Sampler? Sampler;
	}

	/// <summary>Fixed inputs of a particle type. The data is copied. See 'view3d::ParticleTypeOptions'.</summary>
	public ref struct ParticleTypeOptions
	{
		/// <summary>The type's simulation shaders.</summary>
		public ParticleSimShaders Shaders;

		/// <summary>The batch that draws the type's particles.</summary>
		public int Batch;

		/// <summary>Most particles alive at once.</summary>
		public int Capacity;

		/// <summary>Most emitters alive at once.</summary>
		public int MaxEmitters;

		/// <summary>Bytes per particle state, per render record (equal to the batch's), and per emitter's parameters.</summary>
		public int StateStride;
		public int RecordStride;
		public int ParamsStride;

		/// <summary>The type's constants, seen by its kernels.</summary>
		public ReadOnlySpan<byte> Constants;
	}

	/// <summary>
	/// A ready-made particle effect: the settings of one batch, one type and its emitters. Every value refers to static native data, so a preset can be
	/// kept for the life of the process. Presets work in metres with z up. Their emitter parameters are placed at the origin: every parameter structure
	/// starts with a float4 whose xyz is the emitter's position (see <see cref="ParticleParams"/>).
	/// </summary>
	public readonly unsafe struct ParticlePreset
	{
		private readonly NativeParticlePreset m_native;

		/// <summary>Read the stock effect 'preset'.</summary>
		public ParticlePreset(EParticlePreset preset)
		{
			// The native data is static, so only the pointers are copied.
			if (!View3D_ParticlePresetGet(preset, out m_native))
				throw new ArgumentOutOfRangeException(nameof(preset), $"Unknown particle preset: {preset}.");
		}

		/// <summary>The type's simulation shaders.</summary>
		public ParticleSimShaders Sim
		{
			get
			{
				return m_native.m_sim.ToManaged();
			}
		}

		/// <summary>The batch's drawing shaders, with the effect's mesh where it has one.</summary>
		public ParticleDrawShaders Draw
		{
			get
			{
				return m_native.m_draw.ToManaged();
			}
		}

		/// <summary>How the batch blends with the scene.</summary>
		public EParticleBlend Blend
		{
			get
			{
				return m_native.m_blend;
			}
		}

		/// <summary>Bytes per render record, for the batch and the type.</summary>
		public int RecordStride
		{
			get
			{
				return m_native.m_record_stride;
			}
		}

		/// <summary>Bytes per particle state, for the type.</summary>
		public int StateStride
		{
			get
			{
				return m_native.m_state_stride;
			}
		}

		/// <summary>The type's constants.</summary>
		public ReadOnlySpan<byte> Constants
		{
			get
			{
				return new ReadOnlySpan<byte>(m_native.m_constants, checked((int)m_native.m_constants_size));
			}
		}

		/// <summary>The emitter parameters of the effect, at the origin. Copy them to change them.</summary>
		public ReadOnlySpan<byte> Params
		{
			get
			{
				return new ReadOnlySpan<byte>(m_native.m_params, m_native.m_params_stride);
			}
		}

		/// <summary>Suggested emitter rate (particles per second). Zero for effects that are bursts.</summary>
		public float Rate
		{
			get
			{
				return m_native.m_rate;
			}
		}

		/// <summary>Suggested burst size. Zero for continuous effects.</summary>
		public int Burst
		{
			get
			{
				return m_native.m_burst;
			}
		}

		/// <summary>
		/// The batch's texture, or empty for none: square, <see cref="TextureSize"/> texels a side, row by row from the top, in R8G8B8A8_UNORM
		/// (red in the lowest byte) with straight alpha.
		/// </summary>
		public ReadOnlySpan<uint> Texels
		{
			get
			{
				return m_native.m_texels != null ? new ReadOnlySpan<uint>(m_native.m_texels, m_native.m_texture_size * m_native.m_texture_size) : default;
			}
		}

		/// <summary>Texels per side of <see cref="Texels"/>.</summary>
		public int TextureSize
		{
			get
			{
				return m_native.m_texture_size;
			}
		}

		/// <summary>Byte offset of the int collision set in <see cref="Params"/>, or -1 for types that collide with nothing.</summary>
		public int CollisionSetOffset
		{
			get
			{
				return m_native.m_collision_set_offset;
			}
		}

		/// <summary>Create the preset's texture, or return null when it has none. The caller owns the texture.</summary>
		public Texture? CreateTexture(string name)
		{
			// The texels are static native data, so they can be passed without pinning. A full mip chain keeps distant sprites smooth.
			// The texture is only ever sampled, so it rests in the shader-resource state that drawing expects.
			if (m_native.m_texels == null)
				return null;

			var size = m_native.m_texture_size;
			return new Texture(size, size, (IntPtr)m_native.m_texels, size * size * sizeof(uint), new TextureOptions(format: EFormat.DXGI_FORMAT_R8G8B8A8_UNORM, mips: 0, state: EResState.AllShaderResource, has_alpha: true, dbg_name: name));
		}
	}

	/// <summary>
	/// The emitter parameters of the stock spray preset. Matches 'ParticleSprayEmitter' in pr/game-systems/particles/particle_spray.hlsli, which
	/// documents each field. Read and write it in emitter parameter bytes with MemoryMarshal.
	/// </summary>
	[StructLayout(LayoutKind.Sequential)]
	public struct ParticleSprayEmitter
	{
		/// <summary>xy = Centre of the disc of sea. w = Its radius (m).</summary>
		public v4 Area;

		/// <summary>x = Mean upward launch speed (m/s). y = Spread of launch velocities (m/s). z = Fraction of the wind added at launch. w = Drag rate (1/s).</summary>
		public v4 Launch;

		/// <summary>Range of droplet lifetimes (s).</summary>
		public v2 Lifetime;

		/// <summary>Diameter (m) at birth (x) and at death (y).</summary>
		public v2 Size;

		/// <summary>Colours (0xAARRGGBB) at birth and at death.</summary>
		public uint Colour0;
		public uint Colour1;

		/// <summary>Horizontal wind speeds (m/s) over which the chance that a birth makes a droplet rises from zero (x) to one (y).</summary>
		public v2 WindRange;

		/// <summary>Greatest height (m) above the surface that droplets start at.</summary>
		public float Lift;

		/// <summary>Scale of the environment's gravity.</summary>
		public float Gravity;

		private uint m_pad0;
		private uint m_pad1;
	}

	/// <summary>
	/// The emitter parameters of the stock froth preset. Matches 'ParticleFrothEmitter' in pr/game-systems/particles/particle_froth.hlsli, which
	/// documents each field. Read and write it in emitter parameter bytes with MemoryMarshal.
	/// </summary>
	[StructLayout(LayoutKind.Sequential)]
	public struct ParticleFrothEmitter
	{
		/// <summary>xy = Centre of the disc of sea. w = Its radius (m).</summary>
		public v4 Area;

		/// <summary>Range of puff lifetimes (s).</summary>
		public v2 Lifetime;

		/// <summary>Diameter (m) at birth (x) and at death (y).</summary>
		public v2 Size;

		/// <summary>Colours (0xAARRGGBB) at birth and at death.</summary>
		public uint Colour0;
		public uint Colour1;

		/// <summary>Crest foam, in (0, 1], that a point must have to form froth. Uses the ocean's own crest foam settings.</summary>
		public float FoamMin;

		/// <summary>Random points tried per birth. [1, 32].</summary>
		public uint Candidates;

		/// <summary>Fraction of the wind that puffs drift with.</summary>
		public float Drift;

		/// <summary>Spread of the horizontal drift velocities (m/s).</summary>
		public float Spread;

		/// <summary>Height (m) of a puff's centre above the surface at birth.</summary>
		public float Lift;

		/// <summary>Rate (m/s) at which a puff's height above the surface grows.</summary>
		public float Rise;

		/// <summary>Time (s) for a puff to fade once the crest foam under it has gone.</summary>
		public float Linger;

		/// <summary>Distance (m) from the camera at which puffs start to fade out.</summary>
		public float FadeNear;

		/// <summary>Distance (m) from the camera beyond which puffs are not drawn. Greater than <see cref="FadeNear"/>.</summary>
		public float FadeFar;

		/// <summary>Height of a puff along the surface normal as a fraction of its width, in (0, 1].</summary>
		public float Squash;
	}

	/// <summary>Helpers for the emitter parameter bytes of the stock particle types.</summary>
	public static class ParticleParams
	{
		/// <summary>Write the leading float4 of 'params': xyz = the emitter's position, w = the spread of births about it (its meaning depends on the type).</summary>
		public static void SetPosition(Span<byte> @params, v4 position_and_spread)
		{
			// Every stock parameter structure starts with this float4.
			MemoryMarshal.Cast<byte, v4>(@params)[0] = position_and_spread;
		}

		/// <summary>Read the leading float4 of 'params'. See <see cref="SetPosition"/>.</summary>
		public static v4 GetPosition(ReadOnlySpan<byte> @params)
		{
			// Every stock parameter structure starts with this float4.
			return MemoryMarshal.Read<v4>(@params);
		}

		/// <summary>Set the collision set of 'params' for 'preset', or do nothing for presets that collide with nothing. -1 names no set.</summary>
		public static void SetCollisionSet(Span<byte> @params, ParticlePreset preset, int collision_set)
		{
			// The preset knows where its parameter structure keeps the set.
			var offset = preset.CollisionSetOffset;
			if (offset < 0)
				return;

			MemoryMarshal.Cast<byte, int>(@params.Slice(offset, sizeof(int)))[0] = collision_set;
		}
	}

	/// <summary>
	/// A pool of GPU-resident particles. Each particle type is one pool; each batch is one View3D object that draws the particles of all of its
	/// types with one indirect draw. The CPU only uploads emitter changes; particles are born, updated and die on the GPU, in steps of simulated time,
	/// so they move correctly at any step rate and in slow motion. Use on the render owner thread while the View3d context is alive.
	/// See pr/game-systems/particles/particle_system.h.
	/// </summary>
	public sealed class ParticleSystem : IDisposable
	{
		/// <summary>Adopt a successfully created native system.</summary>
		private ParticleSystem(IntPtr handle)
		{
			Handle = handle;
		}

		/// <summary>The native handle.</summary>
		public IntPtr Handle { get; private set; }

		/// <summary>Create an empty particle system on the renderer.</summary>
		public static ParticleSystem Create(string name)
		{
			// The native side reports the reason for a failure through the error callback.
			if (string.IsNullOrEmpty(name))
				throw new ArgumentException("A particle system name is required.", nameof(name));

			var handle = View3D_ParticleSystemCreate(name);
			if (handle == IntPtr.Zero)
				throw new InvalidOperationException("Failed to create the particle system.");

			return new ParticleSystem(handle);
		}

		/// <summary>Release the native system. Batch objects remain valid and keep drawing the last step. Repeated disposal is harmless.</summary>
		public void Dispose()
		{
			View3D_ParticleSystemDestroy(Handle);
			Handle = IntPtr.Zero;
		}

		/// <summary>Set the gravity, uniform wind and contact budget that stepped particle types move in, from the next step.</summary>
		public void SetEnvironment(ParticleEnvironment environment)
		{
			// The native side validates the values.
			Check(View3D_ParticleEnvironmentSet(Live, ref environment), "The particle environment was rejected.");
		}

		/// <summary>
		/// Set the wind grid, from the next step: 'cells_x' by 'cells_y' columns of horizontal wind velocity, row-major, with the minimum corner at
		/// 'origin' in the XY plane and square cells of side 'cell_size'. The data is copied. Zero columns removes the grid.
		/// </summary>
		public unsafe void SetWindGrid(v2 origin, float cell_size, int cells_x, int cells_y, ReadOnlySpan<v2> wind)
		{
			// Only the managed side knows the span length.
			if (wind.Length != (long)cells_x * cells_y)
				throw new ArgumentException("One wind is required for each column of the grid.", nameof(wind));

			fixed (v2* data = wind)
				Check(View3D_ParticleWindGridSet(Live, origin, cell_size, cells_x, cells_y, data), "The particle wind grid was rejected.");
		}

		/// <summary>
		/// Add a collision set of planes, returning its index. Emitters of stepped types name the set that their particles collide with. Indices of
		/// deleted sets are reused, so emitters must stop naming a set before it is deleted.
		/// </summary>
		public unsafe int CreateCollisionSet(ReadOnlySpan<ParticlePlane> planes)
		{
			// The planes are copied.
			fixed (ParticlePlane* data = planes)
			{
				Check(View3D_ParticleCollisionSetCreate(Live, data, planes.Length, out var set), "The particle collision set was rejected.");
				return set;
			}
		}

		/// <summary>Replace the planes of a collision set, from the next step.</summary>
		public unsafe void UpdateCollisionSet(int set, ReadOnlySpan<ParticlePlane> planes)
		{
			// The planes are copied.
			fixed (ParticlePlane* data = planes)
				Check(View3D_ParticleCollisionSetUpdate(Live, set, data, planes.Length), "The particle collision set update was rejected.");
		}

		/// <summary>Remove a collision set. Particles that still name it collide with nothing.</summary>
		public void DeleteCollisionSet(int set)
		{
			// The native side rejects unknown sets.
			Check(View3D_ParticleCollisionSetDelete(Live, set), "Failed to delete the particle collision set.");
		}

		/// <summary>
		/// Bind the water surface that particle types such as spray and froth sample, from the next step. 'model' is the View3D object of 'ocean'.
		/// Both must outlive the binding, and the ocean must be updated before each step so particles see the frame being drawn. Null unbinds.
		/// </summary>
		public void SetWater(Ocean? ocean, Object? model)
		{
			// The model is only read while an ocean is bound.
			var ocean_handle = ocean?.Handle ?? IntPtr.Zero;
			if (ocean != null && ocean_handle == IntPtr.Zero)
				throw new ObjectDisposedException(nameof(Ocean));
			if (ocean != null && (model == null || model.Handle == IntPtr.Zero))
				throw new ArgumentException("The ocean's live model is required.", nameof(model));

			Check(View3D_ParticleWaterSet(Live, ocean_handle, ocean != null ? model!.Handle : IntPtr.Zero), "The particle water binding was rejected.");
		}

		/// <summary>
		/// Add a particle batch, returning its index and its View3D object. The caller owns the object, which outlives the system by drawing the
		/// particles of its last step.
		/// </summary>
		public unsafe int CreateBatch(ParticleBatchOptions options, string name, out Object model)
		{
			// Pin every span while the native side copies the data.
			var draw = options.Shaders;
			fixed (byte* vs = draw.Vertex)
			fixed (uint* indices = draw.Indices)
			fixed (byte* vertex_data = draw.VertexData)
			fixed (byte* constants = options.Constants)
			{
				var native = new NativeParticleBatchOptions
				{
					m_struct_size = sizeof(NativeParticleBatchOptions),
					m_version = NativeParticleBatchOptions.CurrentVersion,
					m_shaders = NativeParticleDrawShaders.From(draw, vs, indices, vertex_data),
					m_capacity = options.Capacity,
					m_record_stride = options.RecordStride,
					m_constants = constants,
					m_constants_size = (nuint)options.Constants.Length,
					m_blend = options.Blend,
					m_bbox_centre = options.Bounds.Centre,
					m_bbox_radius = options.Bounds.Radius,
					m_texture = options.Texture?.Handle ?? IntPtr.Zero,
					m_sampler = options.Sampler?.Handle ?? IntPtr.Zero,
				};
				Check(View3D_ParticleBatchCreate(Live, ref native, name, out var batch, out var obj), "The particle batch was rejected.");
				model = new Object(obj, true);
				return batch;
			}
		}

		/// <summary>Add a particle type drawn by the batch 'options.Batch', returning its index.</summary>
		public unsafe int CreateType(ParticleTypeOptions options, string name)
		{
			// Pin every span while the native side copies the data.
			fixed (byte* emit = options.Shaders.Emit)
			fixed (byte* update = options.Shaders.Update)
			fixed (byte* constants = options.Constants)
			{
				var native = new NativeParticleTypeOptions
				{
					m_struct_size = sizeof(NativeParticleTypeOptions),
					m_version = NativeParticleTypeOptions.CurrentVersion,
					m_shaders = new NativeParticleSimShaders
					{
						m_emit = new NativeShaderByteCode { m_bytecode = emit, m_size = (nuint)options.Shaders.Emit.Length },
						m_update = new NativeShaderByteCode { m_bytecode = update, m_size = (nuint)options.Shaders.Update.Length },
					},
					m_batch = options.Batch,
					m_capacity = options.Capacity,
					m_max_emitters = options.MaxEmitters,
					m_state_stride = options.StateStride,
					m_record_stride = options.RecordStride,
					m_params_stride = options.ParamsStride,
					m_constants = constants,
					m_constants_size = (nuint)options.Constants.Length,
				};
				Check(View3D_ParticleTypeCreate(Live, ref native, name, out var type), "The particle type was rejected.");
				return type;
			}
		}

		/// <summary>Add an emitter of particles of 'type', returning its id. 'params' is the type's parameter stride in bytes.</summary>
		public unsafe ulong CreateEmitter(int type, ParticleEmitter emitter, ReadOnlySpan<byte> @params)
		{
			// The parameters are copied.
			fixed (byte* data = @params)
			{
				Check(View3D_ParticleEmitterCreate(Live, type, ref emitter, data, (nuint)@params.Length, out var id), "The particle emitter was rejected.");
				return id;
			}
		}

		/// <summary>Replace the settings and parameters of an emitter, from the next step. Its particles already alive are unchanged.</summary>
		public unsafe void UpdateEmitter(ulong emitter, ParticleEmitter settings, ReadOnlySpan<byte> @params)
		{
			// The parameters are copied.
			fixed (byte* data = @params)
				Check(View3D_ParticleEmitterUpdate(Live, emitter, ref settings, data, (nuint)@params.Length), "The particle emitter update was rejected.");
		}

		/// <summary>Ask an active emitter for 'count' extra particles, born at the end of the next step.</summary>
		public void Burst(ulong emitter, uint count)
		{
			// The native side rejects unknown emitters.
			Check(View3D_ParticleEmitterBurst(Live, emitter, count), "The particle burst was rejected.");
		}

		/// <summary>Remove an emitter. Its particles already alive live out their lives.</summary>
		public void DeleteEmitter(ulong emitter)
		{
			// The native side rejects unknown emitters.
			Check(View3D_ParticleEmitterDelete(Live, emitter), "Failed to delete the particle emitter.");
		}

		/// <summary>Advance every particle by 'dt' simulated seconds (finite, non-negative). Zero does nothing, so a paused game draws its particles unchanged.</summary>
		public void Step(double dt)
		{
			// The native side validates 'dt'.
			Check(View3D_ParticleSystemStep(Live, dt), "The particle step failed.");
		}

		/// <summary>Remove every particle and restart every emitter's emission. Emitters are kept.</summary>
		public void Reset()
		{
			// Emitters, types and batches survive the reset.
			Check(View3D_ParticleSystemReset(Live), "The particle reset failed.");
		}

		/// <summary>Read the pool usage of 'type'. Waits for the GPU, so use it for tests and diagnostics only.</summary>
		public ParticleStats Stats(int type)
		{
			// The read blocks until the last step completes.
			Check(View3D_ParticleStats(Live, type, out var stats), "Failed to read the particle stats.");
			return stats;
		}

		/// <summary>Throw 'message' when a native call fails.</summary>
		private static void Check(bool ok, string message)
		{
			// The native error callback has already reported the details.
			if (!ok)
				throw new InvalidOperationException(message);
		}

		/// <summary>The handle, or throw if disposed.</summary>
		private IntPtr Live
		{
			get
			{
				if (Handle == IntPtr.Zero)
					throw new ObjectDisposedException(nameof(ParticleSystem));

				return Handle;
			}
		}
	}

	/// <summary>Mirrors 'view3d::ParticleSimShaders'.</summary>
	[StructLayout(LayoutKind.Sequential)]
	internal unsafe struct NativeParticleSimShaders
	{
		public NativeShaderByteCode m_emit;
		public NativeShaderByteCode m_update;

		/// <summary>Spans over the (static) bytecode.</summary>
		public readonly ParticleSimShaders ToManaged()
		{
			// The pointers refer to static native data.
			return new ParticleSimShaders
			{
				Emit = new ReadOnlySpan<byte>(m_emit.m_bytecode, checked((int)m_emit.m_size)),
				Update = new ReadOnlySpan<byte>(m_update.m_bytecode, checked((int)m_update.m_size)),
			};
		}
	}

	/// <summary>Mirrors 'view3d::ParticleDrawShaders'.</summary>
	[StructLayout(LayoutKind.Sequential)]
	internal unsafe struct NativeParticleDrawShaders
	{
		public NativeShaderByteCode m_vertex;
		public uint* m_indices;
		public int m_index_count;
		public int m_vertex_count;
		public EGeom m_geom;
		public byte* m_vertex_data;
		public int m_vertex_stride;

		/// <summary>The native form of 'draw', with its spans pinned at the given pointers.</summary>
		public static NativeParticleDrawShaders From(ParticleDrawShaders draw, byte* vs, uint* indices, byte* vertex_data)
		{
			// Empty vertex data is passed as null with a zero stride.
			return new NativeParticleDrawShaders
			{
				m_vertex = new NativeShaderByteCode { m_bytecode = vs, m_size = (nuint)draw.Vertex.Length },
				m_indices = indices,
				m_index_count = draw.Indices.Length,
				m_vertex_count = draw.VertexCount,
				m_geom = draw.Geom,
				m_vertex_data = draw.VertexData.IsEmpty ? null : vertex_data,
				m_vertex_stride = draw.VertexData.IsEmpty ? 0 : draw.VertexStride,
			};
		}

		/// <summary>Spans over the (static) data.</summary>
		public readonly ParticleDrawShaders ToManaged()
		{
			// The pointers refer to static native data.
			return new ParticleDrawShaders
			{
				Vertex = new ReadOnlySpan<byte>(m_vertex.m_bytecode, checked((int)m_vertex.m_size)),
				Indices = new ReadOnlySpan<uint>(m_indices, m_index_count),
				VertexCount = m_vertex_count,
				Geom = m_geom,
				VertexData = m_vertex_data != null ? new ReadOnlySpan<byte>(m_vertex_data, m_vertex_count * m_vertex_stride) : default,
				VertexStride = m_vertex_stride,
			};
		}
	}

	/// <summary>Mirrors 'view3d::ParticleBatchOptions'.</summary>
	[StructLayout(LayoutKind.Sequential)]
	internal unsafe struct NativeParticleBatchOptions
	{
		public const int CurrentVersion = 1;

		public int m_struct_size;
		public int m_version;
		public NativeParticleDrawShaders m_shaders;
		public int m_capacity;
		public int m_record_stride;
		public byte* m_constants;
		public nuint m_constants_size;
		public EParticleBlend m_blend;
		public v4 m_bbox_centre;
		public v4 m_bbox_radius;
		public IntPtr m_texture;
		public IntPtr m_sampler;
	}

	/// <summary>Mirrors 'view3d::ParticleTypeOptions'.</summary>
	[StructLayout(LayoutKind.Sequential)]
	internal unsafe struct NativeParticleTypeOptions
	{
		public const int CurrentVersion = 2;

		public int m_struct_size;
		public int m_version;
		public NativeParticleSimShaders m_shaders;
		public int m_batch;
		public int m_capacity;
		public int m_max_emitters;
		public int m_state_stride;
		public int m_record_stride;
		public int m_params_stride;
		public byte* m_constants;
		public nuint m_constants_size;
	}

	/// <summary>Mirrors 'view3d::ParticlePreset'.</summary>
	[StructLayout(LayoutKind.Sequential)]
	internal unsafe struct NativeParticlePreset
	{
		public NativeParticleSimShaders m_sim;
		public NativeParticleDrawShaders m_draw;
		public EParticleBlend m_blend;
		public int m_record_stride;
		public int m_state_stride;
		public int m_params_stride;
		public byte* m_constants;
		public nuint m_constants_size;
		public byte* m_params;
		public float m_rate;
		public int m_burst;
		public uint* m_texels;
		public int m_texture_size;
		public int m_collision_set_offset;
	}

	[DllImport(Dll, CharSet = CharSet.Ansi)]
	private static extern IntPtr View3D_ParticleSystemCreate([MarshalAs(UnmanagedType.LPStr)] string name);

	[DllImport(Dll)]
	private static extern void View3D_ParticleSystemDestroy(IntPtr system);

	[DllImport(Dll)]
	[return: MarshalAs(UnmanagedType.Bool)]
	private static extern bool View3D_ParticlePresetGet(EParticlePreset preset, out NativeParticlePreset out_preset);

	[DllImport(Dll)]
	[return: MarshalAs(UnmanagedType.Bool)]
	private static extern bool View3D_ParticleEnvironmentSet(IntPtr system, ref ParticleEnvironment environment);

	[DllImport(Dll)]
	[return: MarshalAs(UnmanagedType.Bool)]
	private static unsafe extern bool View3D_ParticleWindGridSet(IntPtr system, v2 origin, float cell_size, int cells_x, int cells_y, v2* wind);

	[DllImport(Dll)]
	[return: MarshalAs(UnmanagedType.Bool)]
	private static unsafe extern bool View3D_ParticleCollisionSetCreate(IntPtr system, ParticlePlane* planes, int count, out int out_set);

	[DllImport(Dll)]
	[return: MarshalAs(UnmanagedType.Bool)]
	private static unsafe extern bool View3D_ParticleCollisionSetUpdate(IntPtr system, int set, ParticlePlane* planes, int count);

	[DllImport(Dll)]
	[return: MarshalAs(UnmanagedType.Bool)]
	private static extern bool View3D_ParticleCollisionSetDelete(IntPtr system, int set);

	[DllImport(Dll)]
	[return: MarshalAs(UnmanagedType.Bool)]
	private static extern bool View3D_ParticleWaterSet(IntPtr system, IntPtr ocean, IntPtr obj);

	[DllImport(Dll, CharSet = CharSet.Ansi)]
	[return: MarshalAs(UnmanagedType.Bool)]
	private static extern bool View3D_ParticleBatchCreate(IntPtr system, ref NativeParticleBatchOptions options, [MarshalAs(UnmanagedType.LPStr)] string name, out int out_batch, out IntPtr out_object);

	[DllImport(Dll, CharSet = CharSet.Ansi)]
	[return: MarshalAs(UnmanagedType.Bool)]
	private static extern bool View3D_ParticleTypeCreate(IntPtr system, ref NativeParticleTypeOptions options, [MarshalAs(UnmanagedType.LPStr)] string name, out int out_type);

	[DllImport(Dll)]
	[return: MarshalAs(UnmanagedType.Bool)]
	private static unsafe extern bool View3D_ParticleEmitterCreate(IntPtr system, int type, ref ParticleEmitter emitter, byte* @params, nuint params_size, out ulong out_emitter);

	[DllImport(Dll)]
	[return: MarshalAs(UnmanagedType.Bool)]
	private static unsafe extern bool View3D_ParticleEmitterUpdate(IntPtr system, ulong emitter, ref ParticleEmitter settings, byte* @params, nuint params_size);

	[DllImport(Dll)]
	[return: MarshalAs(UnmanagedType.Bool)]
	private static extern bool View3D_ParticleEmitterBurst(IntPtr system, ulong emitter, uint count);

	[DllImport(Dll)]
	[return: MarshalAs(UnmanagedType.Bool)]
	private static extern bool View3D_ParticleEmitterDelete(IntPtr system, ulong emitter);

	[DllImport(Dll)]
	[return: MarshalAs(UnmanagedType.Bool)]
	private static extern bool View3D_ParticleSystemStep(IntPtr system, double dt);

	[DllImport(Dll)]
	[return: MarshalAs(UnmanagedType.Bool)]
	private static extern bool View3D_ParticleSystemReset(IntPtr system);

	[DllImport(Dll)]
	[return: MarshalAs(UnmanagedType.Bool)]
	private static extern bool View3D_ParticleStats(IntPtr system, int type, out ParticleStats out_stats);
}

#if PR_UNITTESTS
/// <summary>Validate the managed particle layouts against the native ABI independently of native DLL availability.</summary>
[TestFixture]
public class ParticleTests
{
	/// <summary>The structs match the native layouts.</summary>
	[Test]
	public void Layouts()
	{
		Assert.Equal(12, Marshal.SizeOf<View3d.ParticleEmitter>());
		Assert.Equal(32, Marshal.SizeOf<View3d.ParticleStats>());
		Assert.Equal(36, Marshal.SizeOf<View3d.ParticleEnvironment>());
		Assert.Equal(24, Marshal.SizeOf<View3d.ParticlePlane>());
		Assert.Equal(32, Marshal.SizeOf<View3d.NativeParticleSimShaders>());
		Assert.Equal(56, Marshal.SizeOf<View3d.NativeParticleDrawShaders>());
		Assert.Equal(144, Marshal.SizeOf<View3d.NativeParticleBatchOptions>());
		Assert.Equal(80, Marshal.SizeOf<View3d.NativeParticleTypeOptions>());
		Assert.Equal(152, Marshal.SizeOf<View3d.NativeParticlePreset>());
	}
}
#endif
