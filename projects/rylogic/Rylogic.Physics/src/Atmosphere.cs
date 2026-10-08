using System;
using Rylogic.Maths;

namespace Rylogic.Physics;

/// <summary>
/// Owns one engine-owned GPU air solver and its optional tracer particles. The atmosphere runs on its engine's device with its own compute queue
/// and steps independently of the engine step. Every member must run on the engine's owner thread.
/// The atmosphere is destroyed with its engine and is not part of engine checkpoints.
/// </summary>
public sealed class Atmosphere :IDisposable
{
	private ulong m_handle;

	/// <summary>Adopt a newly-created native atmosphere and record its fixed grid shape.</summary>
	internal Atmosphere(Engine engine, ulong handle, AtmosphereOptions options)
	{
		Engine = engine;
		CellCountX = options.CellCountX;
		CellCountY = options.CellCountY;
		CellCountZ = options.CellCountZ;
		TracerCount = options.TracerCount;
		m_handle = handle;
	}

	/// <summary>The engine that owns this atmosphere.</summary>
	internal Engine Engine { get; }

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
			return m_handle == 0;
		}
	}

	/// <summary>The stable native atmosphere identity.</summary>
	internal ulong Handle
	{
		get
		{
			if (m_handle == 0)
				throw new ObjectDisposedException(nameof(Atmosphere));

			return m_handle;
		}
	}

	/// <summary>
	/// Submit one step of 'dt' seconds, and tracer advection when tracers exist, without waiting for the GPU. The heat sources are copied and
	/// apply to this step only; at most 64. Floor temperatures and outside air persist from <see cref="SetFloorTemperatures"/> and <see cref="SetOutsideAir"/>.
	/// The air is sampled at each of 'probes' (world points, w = 1, at most 64) once the step has updated it; read the samples with <see cref="CopyProbes"/> after the step finishes.
	/// Fails with <see cref="EStatus.StepPending"/> while a step is in flight.
	/// </summary>
	public unsafe void BeginStep(float dt, ReadOnlySpan<AtmosphereHeatSource> heat_sources = default, ReadOnlySpan<v4> probes = default)
	{
		Engine.EnsureOwner();
		fixed (AtmosphereHeatSource* heat_ptr = heat_sources)
		fixed (v4* probe_ptr = probes)
		{
			var step = new Native.AtmosphereStepDesc
			{
				m_header = NativeHeader.Create<Native.AtmosphereStepDesc>(),
				m_dt = dt,
				m_heat_source_count = heat_sources.Length,
				m_heat_sources = heat_ptr,
				m_probe_count = probes.Length,
				m_probes = probe_ptr,
			};
			Native.Check(Native.Physics_AtmosphereBeginStep(Engine.Handle, Handle, &step));
		}
	}

	/// <summary>
	/// Copy the probe samples of the last finished step into 'samples', in the order of that step's probes. Returns the sample count, which is zero
	/// before the first finished step and after a step without probes. Does not wait, and may be called while a step is in flight.
	/// </summary>
	public unsafe int CopyProbes(Span<AtmosphereProbeSample> samples)
	{
		Engine.EnsureOwner();
		fixed (AtmosphereProbeSample* sample_ptr = samples)
		{
			Native.Check(Native.Physics_AtmosphereProbesCopy(Engine.Handle, Handle, sample_ptr, checked((uint)samples.Length), out var required));
			return checked((int)required);
		}
	}

	/// <summary>
	/// Set the floor temperature under each column (K, one per column in <see cref="AtmosphereOptions.FloorHeights"/> order). The lowest layer relaxes
	/// toward it. Initially the reference temperature at each column's floor. The values are copied and used from the next step, so this may be called while a step is in flight.
	/// </summary>
	public unsafe void SetFloorTemperatures(ReadOnlySpan<float> floor_temperatures)
	{
		Engine.EnsureOwner();
		fixed (float* ptr = floor_temperatures)
			Native.Check(Native.Physics_AtmosphereFloorTemperaturesSet(Engine.Handle, Handle, ptr, floor_temperatures.Length));
	}

	/// <summary>
	/// Set the air outside the open faces, one entry per column in row-major order. Square sides use the edge-column entry, and active/inactive mask faces
	/// use the inactive column's entry. Initially calm air at the reference temperature. The values are copied and used from the next step, so this may be called while a step is in flight.
	/// </summary>
	public unsafe void SetOutsideAir(ReadOnlySpan<AtmosphereOutsideAir> outside_air)
	{
		Engine.EnsureOwner();
		fixed (AtmosphereOutsideAir* ptr = outside_air)
			Native.Check(Native.Physics_AtmosphereOutsideAirSet(Engine.Handle, Handle, ptr, outside_air.Length));
	}

	/// <summary>Finish the step in flight if the GPU has completed it, without waiting. Returns true when no step is in flight after the call.</summary>
	public bool PollStep()
	{
		Engine.EnsureOwner();
		Native.Check(Native.Physics_AtmospherePollStep(Engine.Handle, Handle, out var idle));
		return idle != 0;
	}

	/// <summary>Wait for and finish the step in flight. Fails with <see cref="EStatus.NoStepPending"/> when there is none.</summary>
	public void CompleteStep()
	{
		Engine.EnsureOwner();
		Native.Check(Native.Physics_AtmosphereCompleteStep(Engine.Handle, Handle));
	}

	/// <summary>Change the column floor heights (one per column, as in <see cref="AtmosphereOptions.FloorHeights"/>) and remap the air in changed columns. Blocks; requires no step in flight.</summary>
	public unsafe void SetFloors(ReadOnlySpan<float> floor_heights)
	{
		Engine.EnsureOwner();
		fixed (float* floor_ptr = floor_heights)
			Native.Check(Native.Physics_AtmosphereFloorsSet(Engine.Handle, Handle, floor_ptr, floor_heights.Length));
	}

	/// <summary>
	/// Copy the tracer particles from the last finished step, or from creation, into 'particles', which needs room for <see cref="TracerCount"/>.
	/// Reads the GPU back, so it blocks and requires no step in flight. Prefer <see cref="AcquireTracers"/> for display. Returns the particle count.
	/// </summary>
	public unsafe int CopyTracers(Span<AtmosphereTracerParticle> particles)
	{
		Engine.EnsureOwner();
		fixed (AtmosphereTracerParticle* particle_ptr = particles)
		{
			Native.Check(Native.Physics_AtmosphereTracersCopy(Engine.Handle, Handle, particle_ptr, checked((uint)particles.Length), out var required));
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
		Engine.EnsureOwner();
		var slot = default(Native.AtmosphereTracerSlot);
		Native.Check(Native.Physics_AtmosphereTracersAcquire(Engine.Handle, Handle, &slot));
		return new AtmosphereTracerSlot(new Rylogic.D3D12.ResourceLease(slot.m_resource), slot.m_slot, checked((int)slot.m_count), checked((int)slot.m_stride));
	}

	/// <summary>
	/// Release one hold on 'slot'. 'fence' reaches 'value' when the caller's GPU reads of the slot have finished; pass null when they already have.
	/// The step that next writes the slot waits for the fence on the GPU, so this never blocks.
	/// </summary>
	public void ReleaseTracers(int slot, Rylogic.D3D12.FenceLease? fence, ulong value)
	{
		// The fence is pinned only for the call; the atmosphere takes its own reference.
		Engine.EnsureOwner();
		if (fence == null)
		{
			Native.Check(Native.Physics_AtmosphereTracersRelease(Engine.Handle, Handle, slot, IntPtr.Zero, 0));
			return;
		}
		using var borrowed = fence.Borrow();
		Native.Check(Native.Physics_AtmosphereTracersRelease(Engine.Handle, Handle, slot, borrowed.Handle, value));
	}

	/// <summary>Read the whole field back from the GPU into 'cells', which needs room for <see cref="CellCount"/>, and return its diagnostics. Blocks; requires no step in flight.</summary>
	public unsafe AtmosphereStats CopyCellStates(Span<AtmosphereCellState> cells)
	{
		Engine.EnsureOwner();
		var stats = new Native.AtmosphereStats
		{
			m_header = NativeHeader.Create<Native.AtmosphereStats>(),
		};
		fixed (AtmosphereCellState* cell_ptr = cells)
			Native.Check(Native.Physics_AtmosphereCellStatesCopy(Engine.Handle, Handle, cell_ptr, checked((uint)cells.Length), out _, &stats));

		return new AtmosphereStats(stats);
	}

	/// <summary>Destroy this atmosphere, abandoning any step in flight.</summary>
	public void Dispose()
	{
		if (m_handle == 0)
			return;

		Engine.EnsureOwner();
		Native.Check(Native.Physics_AtmosphereDestroy(Engine.Handle, m_handle));
		m_handle = 0;
		Engine.Remove(this);
		GC.SuppressFinalize(this);
	}

	/// <summary>Invalidate this wrapper after its owning engine destroys the native atmosphere.</summary>
	internal void ReleaseFromEngine()
	{
		if (m_handle == 0)
			return;

		m_handle = 0;
		Engine.Remove(this);
		GC.SuppressFinalize(this);
	}
}
