using System;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;
using Rylogic.Maths;
#if PR_UNITTESTS
using Rylogic.UnitTests;
#endif

namespace Rylogic.Gfx;

public sealed partial class View3d
{
	/// <summary>The result of a native atmosphere function. Matches 'view3d::EAtmosphereStatus'.</summary>
	public enum EAtmosphereStatus
	{
		Success = 0,
		InvalidArgument = 1,
		StepPending = 2,
		NoStepPending = 3,
		BufferTooSmall = 4,
		DeviceRemoved = 5,
		InternalError = 6,
		Timeout = 7,
	}

	/// <summary>Boundary condition of one outside face of an atmosphere domain.</summary>
	public enum EAtmosphereBoundary
	{
		/// <summary>A wall that air cannot cross. Only solid sides may have drag.</summary>
		Solid = 0,

		/// <summary>A side where outside air flows in and inside air leaves freely.</summary>
		Open = 1,
	}

	/// <summary>Indices of the six outside faces of an atmosphere domain, used by <see cref="AtmosphereOptions.Boundaries"/> and <see cref="AtmosphereOptions.WallDrag"/>.</summary>
	public enum EAtmosphereSide
	{
		XMin = 0,
		XMax = 1,
		YMin = 2,
		YMax = 3,
		ZMin = 4,
		ZMax = 5,
	}

	/// <summary>
	/// Creation options for an <see cref="Atmosphere"/>: a terrain-following air solver over 'CellCountX * CellCountY' square columns of 'CellCountZ'
	/// layers. Distances are metres, temperatures are kelvin, and rates are 1/s. Defaults match the native solver's defaults.
	/// See AtmosphereDesc in pr/view3d-12/view3d-dll.h for the meaning of each value.
	/// </summary>
	public sealed class AtmosphereOptions
	{
		/// <summary>Number of columns along X. Must be more than one.</summary>
		public int CellCountX { get; set; }

		/// <summary>Number of columns along Y. Must be more than one.</summary>
		public int CellCountY { get; set; }

		/// <summary>Number of layers in each column. Must be more than one and at most 32.</summary>
		public int CellCountZ { get; set; }

		/// <summary>World position of the domain's minimum corner. 'OriginZ' is also the flat floor height when <see cref="FloorHeights"/> is null.</summary>
		public float OriginX { get; set; }
		public float OriginY { get; set; }
		public float OriginZ { get; set; }

		/// <summary>Width of each square column.</summary>
		public float CellSize { get; set; }

		/// <summary>World height of the top of every column.</summary>
		public float LidZ { get; set; } = 1.0f;

		/// <summary>
		/// Thickness of the lowest layer in every column. The layers above it grow thicker with height so each column reaches the lid in
		/// <see cref="CellCountZ"/> layers. A column shorter than <see cref="CellCountZ"/> layers of this thickness uses uniform layers of this thickness.
		/// </summary>
		public float FirstLayerThickness { get; set; } = 1.0f;

		/// <summary>
		/// Null for a flat floor at <see cref="OriginZ"/>, or one floor height per column in row-major order (x fastest).
		/// A floor at or above <see cref="LidZ"/> makes the column solid. The heights are copied during creation.
		/// </summary>
		public float[]? FloorHeights { get; set; }

		/// <summary>
		/// Null for every column active, or one mask value per column in row-major order (x fastest). Zero removes the column from the solve and
		/// makes neighbouring active columns treat it as open outside air. Floor heights in inactive columns are ignored.
		/// </summary>
		public byte[]? ActiveColumns { get; set; }

		/// <summary>Boundary condition of each outside face, indexed by <see cref="EAtmosphereSide"/>.</summary>
		public EAtmosphereBoundary[] Boundaries { get; } = new EAtmosphereBoundary[6];

		/// <summary>Quadratic drag coefficient in [0, 1] of each outside face, indexed by <see cref="EAtmosphereSide"/>. Only solid faces may have drag.</summary>
		public float[] WallDrag { get; } = new float[6];

		/// <summary>The reference profile 'ReferenceTemperature + LapseRate * (z - OriginZ)', limited below by 'MinTemperature', that sets the initial air at rest.</summary>
		public float ReferenceTemperature { get; set; } = 288.0f;
		public float LapseRate { get; set; } = -0.0065f;
		public float MinTemperature { get; set; } = 180.0f;

		/// <summary>
		/// Temperature change in K/m of moving air per metre that it rises; the default is dry air under Earth gravity. The air at rest is stable when
		/// <see cref="LapseRate"/> is greater (less negative) than this. A world with exaggerated heights can scale both rates by the same factor.
		/// </summary>
		public float AdiabaticLapseRate { get; set; } = -0.00976f;

		/// <summary>Gravity in m/s², positive.</summary>
		public float Gravity { get; set; } = 9.80665f;

		/// <summary>Rate at which air at the floor relaxes towards the floor temperature.</summary>
		public float FloorExchangeRate { get; set; }

		/// <summary>Temperature that the air at the lid relaxes towards, and the relaxation rate.</summary>
		public float LidTemperature { get; set; } = 270.0f;
		public float LidRelaxationRate { get; set; }

		/// <summary>Pressure solve effort per step. Fewer passes than the defaults can become unstable over steep terrain.</summary>
		public int PressureVCycles { get; set; } = 2;
		public int PressurePreSmooth { get; set; } = 2;
		public int PressurePostSmooth { get; set; } = 2;
		public int PressureCoarseSmooth { get; set; } = 8;

		/// <summary>Width, in columns, of the band inside each open side where the wind is nudged towards the inflowing outside air. Keep the floor level across it.</summary>
		public int OpenEdgeBand { get; set; } = 8;

		/// <summary>Strength of the force that restores small swirls lost to numerical smoothing, 1/s. Zero disables it.</summary>
		public float VorticityConfinement { get; set; }

		/// <summary>Vertical eddy viscosity of the horizontal wind, m²/s. Zero disables it.</summary>
		public float VerticalViscosity { get; set; }

		/// <summary>Number of diagnostic tracer particles that follow the wind. Zero disables them.</summary>
		public int TracerCount { get; set; }

		/// <summary>Seed for the deterministic tracer start and respawn positions.</summary>
		public uint TracerSeed { get; set; }

		/// <summary>Seconds before a tracer respawns, so tracers do not collect in still air.</summary>
		public float TracerMaxAge { get; set; } = 20.0f;

		/// <summary>
		/// Relative tracer densities over the column fraction: 'TracerGroundDensity' at the floor falling linearly to 'TracerBreakDensity' at the
		/// fraction 'TracerBreakHeight' (in (0, 1]), then 'TracerUpperDensity' up to the lid. Only the ratios matter.
		/// </summary>
		public float TracerGroundDensity { get; set; } = 1.0f;
		public float TracerBreakDensity { get; set; } = 1.0f;
		public float TracerUpperDensity { get; set; } = 1.0f;
		public float TracerBreakHeight { get; set; } = 0.5f;
	}

	/// <summary>A sphere that heats air at 'm_heating_rate' (K/s) and relaxes it towards 'm_target_temperature' at 'm_relaxation_rate' (1/s).</summary>
	[StructLayout(LayoutKind.Sequential)]
	public readonly struct AtmosphereHeatSource
	{
		public readonly v4 m_centre;
		public readonly float m_radius;
		public readonly float m_heating_rate;
		public readonly float m_target_temperature;
		public readonly float m_relaxation_rate;

		/// <summary>Specify the world centre as a point (m, w = 1) and radius (m), heating rate (K/s), target temperature (K), and relaxation rate (1/s).</summary>
		public AtmosphereHeatSource(v4 centre, float radius, float heating_rate, float target_temperature, float relaxation_rate)
		{
			m_centre = centre;
			m_radius = radius;
			m_heating_rate = heating_rate;
			m_target_temperature = target_temperature;
			m_relaxation_rate = relaxation_rate;
		}
	}

	/// <summary>Air outside one column: its horizontal wind (m/s) and its temperature relative to the reference profile (K).</summary>
	[StructLayout(LayoutKind.Sequential)]
	public readonly struct AtmosphereOutsideAir
	{
		public readonly v2 m_wind;
		public readonly float m_temperature_offset;
		private readonly float m_reserved;

		/// <summary>Specify the horizontal wind (m/s) and the temperature offset from the reference profile (K).</summary>
		public AtmosphereOutsideAir(v2 wind, float temperature_offset)
		{
			m_wind = wind;
			m_temperature_offset = temperature_offset;
			m_reserved = 0;
		}
	}

	/// <summary>The air at one probe point after a step: velocity (m/s, w = 0) and temperature (K). Points outside the air of the domain have 'Inside' false and zero values.</summary>
	[StructLayout(LayoutKind.Sequential)]
	public readonly struct AtmosphereProbeSample
	{
		public readonly v4 m_velocity;
		public readonly float m_temperature;
		private readonly int m_inside;

		/// <summary>True when the point is in an air column, between its floor and the lid.</summary>
		public bool Inside
		{
			get
			{
				return m_inside != 0;
			}
		}
	}

	/// <summary>One atmosphere tracer particle: world position (m, w = 1), air temperature (K), age (s), and the air speed that last moved it (m/s).</summary>
	[StructLayout(LayoutKind.Sequential)]
	public readonly struct AtmosphereTracerParticle
	{
		public readonly v4 m_position;
		public readonly float m_temperature;
		public readonly float m_age;
		public readonly float m_speed;
		private readonly float m_reserved;
	}

	/// <summary>
	/// A held GPU tracer buffer from <see cref="Atmosphere.AcquireTracers"/>. 'Buffer' is a raw buffer of 'Count' <see cref="AtmosphereTracerParticle"/>
	/// records, 'Stride' (32) bytes apart. Pass 'Slot' to <see cref="Atmosphere.ReleaseTracers"/> when the GPU reads are submitted.
	/// </summary>
	public readonly struct AtmosphereTracerSlot
	{
		/// <summary>Adopt a held slot.</summary>
		internal AtmosphereTracerSlot(Rylogic.D3D12.ResourceLease buffer, int slot, int count, int stride)
		{
			Buffer = buffer;
			Slot = slot;
			Count = count;
			Stride = stride;
		}

		/// <summary>The held GPU buffer. The caller disposes it.</summary>
		public Rylogic.D3D12.ResourceLease Buffer { get; }

		/// <summary>The ring slot to pass to <see cref="Atmosphere.ReleaseTracers"/>.</summary>
		public int Slot { get; }

		/// <summary>The number of particle records in the buffer.</summary>
		public int Count { get; }

		/// <summary>The byte size of one particle record.</summary>
		public int Stride { get; }
	}

	/// <summary>The air at one cell centre: velocity (m/s, w = 0, averaged from the cell faces) and temperature (K).</summary>
	[StructLayout(LayoutKind.Sequential)]
	public readonly struct AtmosphereCellState
	{
		public readonly v4 m_velocity;
		public readonly float m_temperature;
	}

	/// <summary>Simple diagnostics of a completed atmosphere field.</summary>
	public readonly struct AtmosphereStats
	{
		/// <summary>Largest cell-centre air speed, m/s.</summary>
		public float MaxSpeed { get; }

		/// <summary>Largest and root-mean-square cell divergence after the pressure solve, 1/s. Small values mean the flow conserves mass.</summary>
		public float MaxDivergence { get; }
		public float RmsDivergence { get; }

		/// <summary>Mean temperature of the top layer, K.</summary>
		public float MeanTopTemperature { get; }

		/// <summary>Largest vertical air speed, m/s.</summary>
		public float PeakVerticalVelocity { get; }

		/// <summary>Copy the native diagnostics record.</summary>
		internal AtmosphereStats(in NativeAtmosphereStats stats)
		{
			MaxSpeed = stats.m_max_speed;
			MaxDivergence = stats.m_max_divergence;
			RmsDivergence = stats.m_rms_divergence;
			MeanTopTemperature = stats.m_mean_top_temperature;
			PeakVerticalVelocity = stats.m_peak_vertical_velocity;
		}
	}

	/// <summary>A failed native atmosphere call, carrying its status and the native error description.</summary>
	public sealed class AtmosphereException : System.Exception
	{
		public AtmosphereException(EAtmosphereStatus status, string message)
			: base(message)
		{
			Status = status;
		}

		/// <summary>The native status of the failed call.</summary>
		public EAtmosphereStatus Status { get; }
	}

	/// <summary>
	/// Owns one GPU air solver and its optional tracer particles. The atmosphere runs on the D3D12 device it was created with, using its own
	/// compute queue, and steps independently of any other system. Every member must run on the thread that created it.
	/// </summary>
	public sealed class Atmosphere : IDisposable
	{
		private readonly int m_owner_thread_id;
		private IntPtr m_handle;

		/// <summary>Adopt a newly-created native atmosphere and record its fixed grid shape.</summary>
		private Atmosphere(IntPtr handle, AtmosphereOptions options)
		{
			m_owner_thread_id = Environment.CurrentManagedThreadId;
			m_handle = handle;
			CellCountX = options.CellCountX;
			CellCountY = options.CellCountY;
			CellCountZ = options.CellCountZ;
			TracerCount = options.TracerCount;
		}

		/// <summary>
		/// Create an atmosphere on 'device'. Creation compiles the solver kernels (or loads them from 'shader_cache_dir' when it is not null) and blocks
		/// until the air is at rest on the reference profile, so it can take seconds; callers with a frame budget should create atmospheres outside their frame work.
		/// The calling thread becomes the owner thread.
		/// </summary>
		public static unsafe Atmosphere Create(Rylogic.D3D12.DeviceLease device, AtmosphereOptions options, string? shader_cache_dir = null)
		{
			// The native call copies the arrays and takes its own device reference, so they only need to stay pinned for the call.
			if (device == null)
				throw new ArgumentNullException(nameof(device));
			if (options == null)
				throw new ArgumentNullException(nameof(options));

			using var borrowed = device.Borrow();
			fixed (float* floor_ptr = options.FloorHeights)
			fixed (byte* active_ptr = options.ActiveColumns)
			{
				var desc = NativeAtmosphereDesc.From(options, floor_ptr, active_ptr);
				Check(View3D_AtmosphereCreate(borrowed.Handle, &desc, shader_cache_dir, out var handle));
				return new Atmosphere(handle, options);
			}
		}

		/// <summary>
		/// Compile every atmosphere GPU kernel into 'shader_cache_dir' using 'thread_count' worker threads, so later creation does not invoke the shader
		/// compiler. Blocks until done. 'progress' receives the completed and total kernel counts on the calling thread.
		/// Throws OperationCanceledException when 'cancel' stops the remaining work.
		/// </summary>
		public static void PrecompileShaders(string shader_cache_dir, int thread_count, Action<int, int>? progress = null, CancellationToken cancel = default)
		{
			// The callback stops native work on cancellation and never lets a managed exception cross the native boundary.
			var error = (System.Exception?)null;
			AtmosphereShaderPrecompileProgressFn callback = (_, completed, total) =>
			{
				// Report progress, then decide whether to continue.
				try
				{
					progress?.Invoke(completed, total);
				}
				catch (System.Exception ex)
				{
					error = ex;
				}
				return error == null && !cancel.IsCancellationRequested ? 1 : 0;
			};
			Check(View3D_AtmosphereShaderPrecompile(shader_cache_dir, thread_count, callback, IntPtr.Zero));
			GC.KeepAlive(callback);

			// Surface a callback failure or cancellation after native work has stopped.
			if (error != null)
				throw new AggregateException("Shader precompile progress callback failed.", error);

			cancel.ThrowIfCancellationRequested();
		}

		/// <summary>The fixed grid shape: columns along X and Y, and layers per column.</summary>
		public int CellCountX { get; }
		public int CellCountY { get; }
		public int CellCountZ { get; }

		/// <summary>The number of columns, and the length of floor-height and floor-temperature arrays.</summary>
		public int ColumnCount
		{
			get
			{
				return CellCountX * CellCountY;
			}
		}

		/// <summary>The number of cells, packed by layer, then row, then column (x fastest).</summary>
		public int CellCount
		{
			get
			{
				return ColumnCount * CellCountZ;
			}
		}

		/// <summary>The number of diagnostic tracer particles.</summary>
		public int TracerCount { get; }

		/// <summary>Return the packed index of the cell at column (x, y) and layer z.</summary>
		public int CellIndex(int x, int y, int z)
		{
			return (z * CellCountY + y) * CellCountX + x;
		}

		/// <summary>True after this wrapper no longer owns a native atmosphere.</summary>
		public bool IsDisposed
		{
			get
			{
				return m_handle == IntPtr.Zero;
			}
		}

		/// <summary>
		/// Submit one step of 'dt' seconds, and tracer advection when tracers exist, without waiting for the GPU. The heat sources are copied and
		/// apply to this step only; at most 64. Floor temperatures and outside air persist from <see cref="SetFloorTemperatures"/> and <see cref="SetOutsideAir"/>.
		/// The air is sampled at each of 'probes' (world points, w = 1, at most 64) once the step has updated it; read the samples with <see cref="CopyProbes"/> after the step finishes.
		/// When 'surface_wind_height' is positive, the horizontal wind at that height (m) above each column floor is also sampled; read it with <see cref="CopySurfaceWind"/>.
		/// Fails with <see cref="EAtmosphereStatus.StepPending"/> while a step is in flight.
		/// </summary>
		public unsafe void BeginStep(float dt, ReadOnlySpan<AtmosphereHeatSource> heat_sources = default, ReadOnlySpan<v4> probes = default, float surface_wind_height = 0f)
		{
			// The native call copies the step inputs, so they only need to stay pinned for the call.
			var handle = Handle;
			fixed (AtmosphereHeatSource* heat_ptr = heat_sources)
			fixed (v4* probe_ptr = probes)
			{
				var step = new NativeAtmosphereStepDesc
				{
					m_dt = dt,
					m_heat_source_count = heat_sources.Length,
					m_heat_sources = heat_ptr,
					m_probe_count = probes.Length,
					m_surface_wind_height = surface_wind_height,
					m_probes = probe_ptr,
				};
				Check(View3D_AtmosphereBeginStep(handle, &step));
			}
		}

		/// <summary>
		/// Copy the surface wind of the last finished step into 'wind': the horizontal wind (m/s) at the requested height above each column floor, one per
		/// column in row-major order. Inactive columns report their outside air's wind; solid columns report zero. Returns the column count, which is zero
		/// before the first finished step and after a step that did not sample the surface wind. Does not wait, and may be called while a step is in flight.
		/// </summary>
		public unsafe int CopySurfaceWind(Span<v2> wind)
		{
			// Copy into the caller's buffer, which must hold every column.
			var handle = Handle;
			fixed (v2* wind_ptr = wind)
			{
				Check(View3D_AtmosphereSurfaceWindCopy(handle, wind_ptr, checked((uint)wind.Length), out var required));
				return checked((int)required);
			}
		}

		/// <summary>
		/// Copy the probe samples of the last finished step into 'samples', in the order of that step's probes. Returns the sample count, which is zero
		/// before the first finished step and after a step without probes. Does not wait, and may be called while a step is in flight.
		/// </summary>
		public unsafe int CopyProbes(Span<AtmosphereProbeSample> samples)
		{
			// Copy into the caller's buffer, which must hold every sample.
			var handle = Handle;
			fixed (AtmosphereProbeSample* sample_ptr = samples)
			{
				Check(View3D_AtmosphereProbesCopy(handle, sample_ptr, checked((uint)samples.Length), out var required));
				return checked((int)required);
			}
		}

		/// <summary>
		/// Set the floor temperature under each column (K, one per column in <see cref="AtmosphereOptions.FloorHeights"/> order). The lowest layer relaxes
		/// toward it. Initially the reference temperature at each column's floor. The values are copied and used from the next step, so this may be called while a step is in flight.
		/// </summary>
		public unsafe void SetFloorTemperatures(ReadOnlySpan<float> floor_temperatures)
		{
			// The native call copies the values.
			var handle = Handle;
			fixed (float* ptr = floor_temperatures)
				Check(View3D_AtmosphereFloorTemperaturesSet(handle, ptr, floor_temperatures.Length));
		}

		/// <summary>
		/// Set the air outside the open faces, one entry per column in row-major order. Square sides use the edge-column entry, and active/inactive mask faces
		/// use the inactive column's entry. Initially calm air at the reference temperature. The values are copied and used from the next step, so this may be called while a step is in flight.
		/// </summary>
		public unsafe void SetOutsideAir(ReadOnlySpan<AtmosphereOutsideAir> outside_air)
		{
			// The native call copies the values.
			var handle = Handle;
			fixed (AtmosphereOutsideAir* ptr = outside_air)
				Check(View3D_AtmosphereOutsideAirSet(handle, ptr, outside_air.Length));
		}

		/// <summary>Finish the step in flight if the GPU has completed it, without waiting. Returns true when no step is in flight after the call.</summary>
		public bool PollStep()
		{
			// Ask the native atmosphere without blocking.
			Check(View3D_AtmospherePollStep(Handle, out var idle));
			return idle != 0;
		}

		/// <summary>Wait for and finish the step in flight. Fails with <see cref="EAtmosphereStatus.NoStepPending"/> when there is none.</summary>
		public void CompleteStep()
		{
			// Block until the GPU finishes the step.
			Check(View3D_AtmosphereCompleteStep(Handle));
		}

		/// <summary>Change the column floor heights (one per column, as in <see cref="AtmosphereOptions.FloorHeights"/>) and remap the air in changed columns. Blocks; requires no step in flight.</summary>
		public unsafe void SetFloors(ReadOnlySpan<float> floor_heights)
		{
			// The native call copies the heights.
			var handle = Handle;
			fixed (float* floor_ptr = floor_heights)
				Check(View3D_AtmosphereFloorsSet(handle, floor_ptr, floor_heights.Length));
		}

		/// <summary>
		/// Copy the tracer particles from the last finished step, or from creation, into 'particles', which needs room for <see cref="TracerCount"/>.
		/// Reads the GPU back, so it blocks and requires no step in flight. Prefer <see cref="AcquireTracers"/> for display. Returns the particle count.
		/// </summary>
		public unsafe int CopyTracers(Span<AtmosphereTracerParticle> particles)
		{
			// Copy into the caller's buffer, which must hold every tracer.
			var handle = Handle;
			fixed (AtmosphereTracerParticle* particle_ptr = particles)
			{
				Check(View3D_AtmosphereTracersCopy(handle, particle_ptr, checked((uint)particles.Length), out var required));
				return checked((int)required);
			}
		}

		/// <summary>
		/// Hold the GPU buffer of the tracer particles from the last finished step, or from creation, so later steps do not write it. At most two slots can be
		/// held at once, and a step fails while it has no free buffer to write. The caller disposes the returned buffer lease and calls <see cref="ReleaseTracers"/>
		/// once for each acquire. May be called while a step is in flight.
		/// </summary>
		public unsafe AtmosphereTracerSlot AcquireTracers()
		{
			// The native call returns an owned COM reference, which the lease adopts.
			var slot = default(NativeAtmosphereTracerSlot);
			Check(View3D_AtmosphereTracersAcquire(Handle, &slot));
			return new AtmosphereTracerSlot(new Rylogic.D3D12.ResourceLease(slot.m_resource), slot.m_slot, checked((int)slot.m_count), checked((int)slot.m_stride));
		}

		/// <summary>
		/// Release one hold on 'slot'. 'fence' reaches 'value' when the caller's GPU reads of the slot have finished; pass null when they already have.
		/// The step that next writes the slot waits for the fence on the GPU, so this never blocks.
		/// </summary>
		public void ReleaseTracers(int slot, Rylogic.D3D12.FenceLease? fence, ulong value)
		{
			// The fence is pinned only for the call; the atmosphere takes its own reference.
			var handle = Handle;
			if (fence == null)
			{
				Check(View3D_AtmosphereTracersRelease(handle, slot, IntPtr.Zero, 0));
				return;
			}
			using var borrowed = fence.Borrow();
			Check(View3D_AtmosphereTracersRelease(handle, slot, borrowed.Handle, value));
		}

		/// <summary>Read the whole field back from the GPU into 'cells', which needs room for <see cref="CellCount"/>, and return its diagnostics. Blocks; requires no step in flight.</summary>
		public unsafe AtmosphereStats CopyCellStates(Span<AtmosphereCellState> cells)
		{
			// Copy into the caller's buffer, which must hold every cell.
			var handle = Handle;
			var stats = default(NativeAtmosphereStats);
			fixed (AtmosphereCellState* cell_ptr = cells)
				Check(View3D_AtmosphereCellStatesCopy(handle, cell_ptr, checked((uint)cells.Length), out _, &stats));

			return new AtmosphereStats(stats);
		}

		/// <summary>Close with an infinite wait, retaining this atmosphere for retry if cleanup fails.</summary>
		public void Dispose()
		{
			// IDisposable opts into blocking; bounded teardown is available through TryDispose.
			if (!TryDispose(-1))
				throw new TimeoutException("An infinite atmosphere close unexpectedly timed out.");
		}

		/// <summary>Stop this atmosphere and close within timeout_ms milliseconds (-1 means infinite). False or an exception retains ownership for retry.</summary>
		public bool TryDispose(int timeout_ms)
		{
			// Match the .NET timeout convention at the managed input boundary.
			if (timeout_ms < -1)
				throw new ArgumentOutOfRangeException(nameof(timeout_ms));
			if (m_handle == IntPtr.Zero)
				return true;

			// Keep the same handle until the native atmosphere confirms that its GPU resources may be released.
			EnsureOwner();
			var status = View3D_AtmosphereDestroy(m_handle, unchecked((uint)timeout_ms));
			switch (status)
			{
				case EAtmosphereStatus.Success:
				{
					m_handle = IntPtr.Zero;
					return true;
				}
				case EAtmosphereStatus.Timeout:
				{
					return false;
				}
				default:
				{
					Check(status);
					throw new InvalidOperationException($"Unexpected atmosphere close status {status}.");
				}
			}
		}

		/// <summary>The native atmosphere, after checking the caller is on the owner thread.</summary>
		private IntPtr Handle
		{
			get
			{
				EnsureOwner();
				if (m_handle == IntPtr.Zero)
					throw new ObjectDisposedException(nameof(Atmosphere));

				return m_handle;
			}
		}

		/// <summary>Reject calls from threads other than the creating thread, because the native atmosphere is not thread safe.</summary>
		private void EnsureOwner()
		{
			if (Environment.CurrentManagedThreadId != m_owner_thread_id)
				throw new InvalidOperationException("Atmosphere calls must run on the atmosphere's creating thread.");
		}

		/// <summary>Throw an <see cref="AtmosphereException"/> carrying the native error description when 'status' is a failure.</summary>
		private static unsafe void Check(EAtmosphereStatus status)
		{
			// Success needs no description.
			if (status == EAtmosphereStatus.Success)
				return;

			// Read the thread's last error text, growing the buffer once if the first read truncated it.
			var buffer = new byte[256];
			for (;;)
			{
				uint required;
				fixed (byte* ptr = buffer)
					View3D_AtmosphereLastError(ptr, (uint)buffer.Length, &required);

				if (required <= buffer.Length)
				{
					var length = Math.Max(0, (int)required - 1);
					throw new AtmosphereException(status, $"{status}: {Encoding.UTF8.GetString(buffer, 0, length)}");
				}
				buffer = new byte[required];
			}
		}
	}

	[StructLayout(LayoutKind.Sequential)]
	internal unsafe struct NativeAtmosphereDesc
	{
		public fixed int m_cell_count[4];
		public v4 m_origin;
		public float m_dx;
		public float m_lid_z;
		public float m_first_layer_thickness;
		public float* m_floor_heights;
		public byte* m_active_columns;
		public fixed int m_boundaries[6];
		public fixed float m_wall_drag[6];
		public float m_reference_temperature;
		public float m_lapse_rate;
		public float m_adiabatic_lapse_rate;
		public float m_min_temperature;
		public float m_gravity;
		public float m_floor_exchange_rate;
		public float m_lid_temperature;
		public float m_lid_relaxation_rate;
		public int m_pressure_vcycles;
		public int m_pressure_pre_smooth;
		public int m_pressure_post_smooth;
		public int m_pressure_coarse_smooth;
		public int m_open_edge_band;
		public float m_vorticity_confinement;
		public float m_vertical_viscosity;
		public int m_tracer_count;
		public uint m_tracer_seed;
		public float m_tracer_max_age;
		public float m_tracer_ground_density;
		public float m_tracer_break_density;
		public float m_tracer_upper_density;
		public float m_tracer_break_height;

		/// <summary>Convert managed creation options into the exact native layout. Pinned arrays must stay pinned until creation returns.</summary>
		public static NativeAtmosphereDesc From(AtmosphereOptions options, float* floor_heights, byte* active_columns)
		{
			// Copy the scalar options.
			var result = new NativeAtmosphereDesc
			{
				m_origin = new v4(options.OriginX, options.OriginY, options.OriginZ, 1.0f),
				m_dx = options.CellSize,
				m_lid_z = options.LidZ,
				m_first_layer_thickness = options.FirstLayerThickness,
				m_floor_heights = floor_heights,
				m_active_columns = active_columns,
				m_reference_temperature = options.ReferenceTemperature,
				m_lapse_rate = options.LapseRate,
				m_adiabatic_lapse_rate = options.AdiabaticLapseRate,
				m_min_temperature = options.MinTemperature,
				m_gravity = options.Gravity,
				m_floor_exchange_rate = options.FloorExchangeRate,
				m_lid_temperature = options.LidTemperature,
				m_lid_relaxation_rate = options.LidRelaxationRate,
				m_pressure_vcycles = options.PressureVCycles,
				m_pressure_pre_smooth = options.PressurePreSmooth,
				m_pressure_post_smooth = options.PressurePostSmooth,
				m_pressure_coarse_smooth = options.PressureCoarseSmooth,
				m_open_edge_band = options.OpenEdgeBand,
				m_vorticity_confinement = options.VorticityConfinement,
				m_vertical_viscosity = options.VerticalViscosity,
				m_tracer_count = options.TracerCount,
				m_tracer_seed = options.TracerSeed,
				m_tracer_max_age = options.TracerMaxAge,
				m_tracer_ground_density = options.TracerGroundDensity,
				m_tracer_break_density = options.TracerBreakDensity,
				m_tracer_upper_density = options.TracerUpperDensity,
				m_tracer_break_height = options.TracerBreakHeight,
			};
			result.m_cell_count[0] = options.CellCountX;
			result.m_cell_count[1] = options.CellCountY;
			result.m_cell_count[2] = options.CellCountZ;

			// The side arrays are indexed by EAtmosphereSide in both the managed options and the native record.
			for (var i = 0; i != 6; ++i)
			{
				result.m_boundaries[i] = (int)options.Boundaries[i];
				result.m_wall_drag[i] = options.WallDrag[i];
			}
			return result;
		}
	}

	[StructLayout(LayoutKind.Sequential)]
	internal unsafe struct NativeAtmosphereStepDesc
	{
		public float m_dt;
		public int m_heat_source_count;
		public AtmosphereHeatSource* m_heat_sources;
		public int m_probe_count;
		public float m_surface_wind_height;
		public v4* m_probes;
	}

	[StructLayout(LayoutKind.Sequential)]
	internal struct NativeAtmosphereStats
	{
		public float m_max_speed;
		public float m_max_divergence;
		public float m_rms_divergence;
		public float m_mean_top_temperature;
		public float m_peak_vertical_velocity;
		public int m_reserved;
	}

	[StructLayout(LayoutKind.Sequential)]
	internal struct NativeAtmosphereTracerSlot
	{
		public IntPtr m_resource;
		public int m_slot;
		public uint m_count;
		public uint m_stride;
		public uint m_reserved;
	}

	[UnmanagedFunctionPointer(CallingConvention.StdCall)]
	private delegate int AtmosphereShaderPrecompileProgressFn(IntPtr ctx, int completed, int total);

	[DllImport(Dll)]
	private static unsafe extern EAtmosphereStatus View3D_AtmosphereLastError(byte* buffer, uint capacity, uint* required);

	[DllImport(Dll, CharSet = CharSet.Unicode)]
	private static unsafe extern EAtmosphereStatus View3D_AtmosphereCreate(IntPtr d3d12_device, NativeAtmosphereDesc* desc, [MarshalAs(UnmanagedType.LPWStr)] string? shader_cache_dir, out IntPtr atmosphere);

	[DllImport(Dll)]
	private static extern EAtmosphereStatus View3D_AtmosphereDestroy(IntPtr atmosphere, uint timeout_ms);

	[DllImport(Dll)]
	private static unsafe extern EAtmosphereStatus View3D_AtmosphereBeginStep(IntPtr atmosphere, NativeAtmosphereStepDesc* step);

	[DllImport(Dll)]
	private static extern EAtmosphereStatus View3D_AtmospherePollStep(IntPtr atmosphere, out int idle);

	[DllImport(Dll)]
	private static unsafe extern EAtmosphereStatus View3D_AtmosphereProbesCopy(IntPtr atmosphere, AtmosphereProbeSample* samples, uint capacity, out uint required);

	[DllImport(Dll)]
	private static unsafe extern EAtmosphereStatus View3D_AtmosphereSurfaceWindCopy(IntPtr atmosphere, v2* wind, uint capacity, out uint required);

	[DllImport(Dll)]
	private static extern EAtmosphereStatus View3D_AtmosphereCompleteStep(IntPtr atmosphere);

	[DllImport(Dll)]
	private static unsafe extern EAtmosphereStatus View3D_AtmosphereFloorsSet(IntPtr atmosphere, float* floor_heights, int count);

	[DllImport(Dll)]
	private static unsafe extern EAtmosphereStatus View3D_AtmosphereFloorTemperaturesSet(IntPtr atmosphere, float* floor_temperatures, int count);

	[DllImport(Dll)]
	private static unsafe extern EAtmosphereStatus View3D_AtmosphereOutsideAirSet(IntPtr atmosphere, AtmosphereOutsideAir* outside_air, int count);

	[DllImport(Dll)]
	private static unsafe extern EAtmosphereStatus View3D_AtmosphereTracersCopy(IntPtr atmosphere, AtmosphereTracerParticle* particles, uint capacity, out uint required);

	[DllImport(Dll)]
	private static unsafe extern EAtmosphereStatus View3D_AtmosphereTracersAcquire(IntPtr atmosphere, NativeAtmosphereTracerSlot* slot);

	[DllImport(Dll)]
	private static extern EAtmosphereStatus View3D_AtmosphereTracersRelease(IntPtr atmosphere, int slot, IntPtr fence, ulong value);

	[DllImport(Dll)]
	private static unsafe extern EAtmosphereStatus View3D_AtmosphereCellStatesCopy(IntPtr atmosphere, AtmosphereCellState* cells, uint capacity, out uint required, NativeAtmosphereStats* stats);

	[DllImport(Dll, CharSet = CharSet.Unicode)]
	private static extern EAtmosphereStatus View3D_AtmosphereShaderPrecompile([MarshalAs(UnmanagedType.LPWStr)] string shader_cache_dir, int thread_count, AtmosphereShaderPrecompileProgressFn progress, IntPtr ctx);
}

#if PR_UNITTESTS
/// <summary>Validate the managed atmosphere layouts against the native ABI independently of native DLL availability.</summary>
[TestFixture]
public class AtmosphereTests
{
	/// <summary>The structs match the native layouts in view3d-dll.h.</summary>
	[Test]
	public unsafe void Layouts()
	{
		Assert.Equal(200, sizeof(View3d.NativeAtmosphereDesc));
		Assert.Equal(48, Marshal.OffsetOf<View3d.NativeAtmosphereDesc>(nameof(View3d.NativeAtmosphereDesc.m_floor_heights)).ToInt32());
		Assert.Equal(56, Marshal.OffsetOf<View3d.NativeAtmosphereDesc>(nameof(View3d.NativeAtmosphereDesc.m_active_columns)).ToInt32());
		Assert.Equal(196, Marshal.OffsetOf<View3d.NativeAtmosphereDesc>(nameof(View3d.NativeAtmosphereDesc.m_tracer_break_height)).ToInt32());
		Assert.Equal(32, sizeof(View3d.NativeAtmosphereStepDesc));
		Assert.Equal(20, Marshal.OffsetOf<View3d.NativeAtmosphereStepDesc>(nameof(View3d.NativeAtmosphereStepDesc.m_surface_wind_height)).ToInt32());
		Assert.Equal(24, sizeof(View3d.NativeAtmosphereStats));
		Assert.Equal(24, sizeof(View3d.NativeAtmosphereTracerSlot));
		Assert.Equal(32, sizeof(View3d.AtmosphereHeatSource));
		Assert.Equal(16, sizeof(View3d.AtmosphereOutsideAir));
		Assert.Equal(24, sizeof(View3d.AtmosphereProbeSample));
		Assert.Equal(32, sizeof(View3d.AtmosphereTracerParticle));
		Assert.Equal(20, sizeof(View3d.AtmosphereCellState));
	}
}
#endif
