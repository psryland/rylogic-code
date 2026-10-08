using System;
using System.Runtime.InteropServices;
using Rylogic.Maths;

namespace Rylogic.Physics;

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
/// See AtmosphereDesc in pr/physics/physics-dll.h for the meaning of each value.
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
/// records, 'Stride' (32) bytes apart.
/// Pass 'Slot' to <see cref="Atmosphere.ReleaseTracers"/> when the GPU reads are submitted.
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
	internal AtmosphereStats(in Native.AtmosphereStats stats)
	{
		MaxSpeed = stats.m_max_speed;
		MaxDivergence = stats.m_max_divergence;
		RmsDivergence = stats.m_rms_divergence;
		MeanTopTemperature = stats.m_mean_top_temperature;
		PeakVerticalVelocity = stats.m_peak_vertical_velocity;
	}
}
