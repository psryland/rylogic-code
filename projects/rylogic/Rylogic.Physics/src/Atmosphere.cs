using System;

namespace Rylogic.Physics;

/// <summary>
/// Owns one engine-owned GPU air solver and its optional tracer particles. The atmosphere runs on its engine's device with its own compute queue
/// and steps independently of the engine step. Every member except <see cref="CopyTracers"/> must run on the engine's owner thread.
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

	/// <summary>The number of boundary columns, and the length of an outside-air array. See <see cref="BeginStep"/> for their order.</summary>
	public int BoundaryColumnCount
	{
		get
		{
			return 2 * (CellCountX + CellCountY);
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
	/// Submit one step of 'dt' seconds, and tracer advection when tracers exist, without waiting for the GPU. All inputs are copied.
	/// At most 64 heat sources. Empty 'floor_temperatures' uses 'uniform_floor_temperature' everywhere; otherwise give one per column.
	/// Empty 'outside_air' means calm outside air at the reference temperature; otherwise give one per boundary column: the x- side by y,
	/// the x+ side by y, the y- side by x, then the y+ side by x. Fails with <see cref="EStatus.StepPending"/> while a step is in flight.
	/// </summary>
	public unsafe void BeginStep(float dt, float uniform_floor_temperature = 288.0f, ReadOnlySpan<AtmosphereHeatSource> heat_sources = default, ReadOnlySpan<float> floor_temperatures = default, ReadOnlySpan<AtmosphereOutsideAir> outside_air = default)
	{
		Engine.EnsureOwner();
		fixed (AtmosphereHeatSource* heat_ptr = heat_sources)
		fixed (float* floor_ptr = floor_temperatures)
		fixed (AtmosphereOutsideAir* outside_ptr = outside_air)
		{
			var step = new Native.AtmosphereStepDesc
			{
				m_header = NativeHeader.Create<Native.AtmosphereStepDesc>(),
				m_dt = dt,
				m_uniform_floor_temperature = uniform_floor_temperature,
				m_heat_sources = heat_ptr,
				m_floor_temperatures = floor_ptr,
				m_outside_air = outside_ptr,
				m_heat_source_count = heat_sources.Length,
				m_floor_temperature_count = floor_temperatures.Length,
				m_outside_air_count = outside_air.Length,
			};
			Native.Check(Native.Physics_AtmosphereBeginStep(Engine.Handle, Handle, &step));
		}
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
	/// Any thread may call this; the copy is ordered against the owner thread finishing a step. Returns the particle count.
	/// </summary>
	public unsafe int CopyTracers(Span<AtmosphereTracerParticle> particles)
	{
		fixed (AtmosphereTracerParticle* particle_ptr = particles)
		{
			Native.Check(Native.Physics_AtmosphereTracersCopy(Engine.Handle, Handle, particle_ptr, checked((uint)particles.Length), out var required));
			return checked((int)required);
		}
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
