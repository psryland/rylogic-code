using System;
using System.Runtime.InteropServices;
using Rylogic.Maths;

namespace Rylogic.Physics;

/// <summary>
/// Water that applies buoyancy and drag to every dynamic body. The surface is a height field over world XY (+Z up): the still-water level plus
/// optional wave elements (see <see cref="Engine.SetWater"/>). Bodies that cannot reach the surface during a step cost no water work.
/// Density is kg/m³ and drag rates are 1/s at full submersion.
/// </summary>
public readonly struct WaterConfiguration
{
	/// <summary>Default ratio of the largest (crest to trough) wave height to the water depth.</summary>
	public const float DefaultBreakingRatio = 0.78f;

	/// <summary>Default crest sharpness of waves at the breaking height. Zero leaves crests unchanged.</summary>
	public const float DefaultShoreSharpness = 0.0f;

	public readonly double m_level;
	public readonly float m_density;
	public readonly float m_linear_drag_rate;
	public readonly float m_quadratic_drag_coefficient;
	public readonly float m_angular_drag_rate;
	public readonly float m_breaking_ratio;
	public readonly float m_shore_sharpness;
	public readonly float m_repeat_period;

	/// <summary>
	/// Specify the still-water level in metres and the water's density and drag. 'breaking_ratio' limits the total wave amplitude to half this
	/// fraction of the depth where terrain heights are known (see <see cref="Engine.SetWaterBathymetry"/>). 'shore_sharpness', in [0, 0.9], is the
	/// crest sharpness that waves reach at the breaking height; crests sharpen towards it as waves approach breaking. 'repeat_period' (s) is the
	/// period after which every wave element repeats, so the engine can wrap long clocks without losing precision; zero means the elements are not periodic.
	/// </summary>
	public WaterConfiguration(double level, float density = 1000.0f, float linear_drag_rate = 0.5f, float quadratic_drag_coefficient = 0.5f, float angular_drag_rate = 0.5f, float breaking_ratio = DefaultBreakingRatio, float shore_sharpness = DefaultShoreSharpness, float repeat_period = 0.0f)
	{
		m_level = level;
		m_density = density;
		m_linear_drag_rate = linear_drag_rate;
		m_quadratic_drag_coefficient = quadratic_drag_coefficient;
		m_angular_drag_rate = angular_drag_rate;
		m_breaking_ratio = breaking_ratio;
		m_shore_sharpness = shore_sharpness;
		m_repeat_period = repeat_period;
	}
}

/// <summary>
/// One water-surface element, with the native 64-byte layout shared by CPU sampling, GPU buoyancy, and renderers.
/// See pr/physics/terrain/water/water_field_types.hlsli for the meaning of each field for each element type.
/// </summary>
[StructLayout(LayoutKind.Sequential)]
public struct WaterFieldElement
{
	/// <summary>Stride of one element in bytes.</summary>
	public const int SizeInBytes = 64;

	/// <summary>Element types stored in 'm_info.x'.</summary>
	public const int TypeNone = 0;
	public const int TypeSineWave = 1;
	public const int TypeGerstnerWave = 2;
	public const int TypeRadialPacket = 3;

	public Vector4i m_info;
	public v4 m_position;
	public v4 m_wave;
	public v4 m_timing;
}

/// <summary>The fixed component layout of a <see cref="WaveSpectrum"/>; see WaveSpectrumDesc in physics-dll.h.</summary>
[StructLayout(LayoutKind.Sequential)]
public readonly struct WaveSpectrumLayout
{
	/// <summary>Gravity in m/s², positive. It sets each component's speed.</summary>
	public readonly float Gravity;

	/// <summary>Every component repeats exactly after this many seconds, positive.</summary>
	public readonly float RepeatPeriod;

	/// <summary>The shortest wavelength of the covered range, in metres (positive).</summary>
	public readonly float MinWavelength;

	/// <summary>The longest wavelength of the covered range, in metres (greater than <see cref="MinWavelength"/>).</summary>
	public readonly float MaxWavelength;

	/// <summary>The wavelength range is split into this many equal log-wavelength bands.</summary>
	public readonly int Bands;

	/// <summary>The number of components in each band, spread evenly around the full circle.</summary>
	public readonly int Directions;

	/// <summary>Describe a spectrum of 'bands' * 'directions' components (at most 96) between 'min_wavelength' and 'max_wavelength'.</summary>
	public WaveSpectrumLayout(float gravity, float repeat_period, float min_wavelength, float max_wavelength, int bands, int directions)
	{
		Gravity = gravity;
		RepeatPeriod = repeat_period;
		MinWavelength = min_wavelength;
		MaxWavelength = max_wavelength;
		Bands = bands;
		Directions = directions;
	}

	/// <summary>The number of components.</summary>
	public int ComponentCount
	{
		get
		{
			return Bands * Directions;
		}
	}
}

/// <summary>
/// The layout of a wind sea (see <see cref="Engine.SetWaterWindSea"/> and pr/physics/terrain/water/wind_sea.hlsli). Node (i, j) is at
/// Origin + (i, j) * CellSize and node n = j * Width + i holds LevelCount downwind wind vectors (m/s) at winds[n * LevelCount + l].
/// The levels are spread evenly from the first band to the last, and each band uses the wind interpolated between its two nearest levels.
/// The table holds BandCount * SpeedCount rows from <see cref="WaveSpectrum.SeaTable"/>, where row r describes a wind of r * SpeedStep m/s.
/// Width, Height and SpeedCount are at least two, and LevelCount is in [1, BandCount].
/// </summary>
public readonly struct WindSeaGrid
{
	public readonly v2 Origin;
	public readonly float CellSize;
	public readonly float SpeedStep;
	public readonly int Width;
	public readonly int Height;
	public readonly int BandCount;
	public readonly int SpeedCount;
	public readonly int LevelCount;

	/// <summary>Describe a wind-sea grid.</summary>
	public WindSeaGrid(v2 origin, float cell_size, float speed_step, int width, int height, int band_count, int speed_count, int level_count)
	{
		Origin = origin;
		CellSize = cell_size;
		SpeedStep = speed_step;
		Width = width;
		Height = height;
		BandCount = band_count;
		SpeedCount = speed_count;
		LevelCount = level_count;
	}
}

/// <summary>
/// A fixed set of wind-driven wave components spread around the full circle, whose local amplitudes follow the local wind (see
/// <see cref="WindSeaGrid"/>). The components depend only on the layout, so changes in wind speed or direction only change amplitudes and the
/// surface never jumps. Every component repeats after the layout's repeat period.
/// </summary>
public static unsafe class WaveSpectrum
{
	/// <summary>
	/// Write the wind-sea table of 'layout' for a wind blowing over 'fetch' metres of open water. 'rows' holds Bands * speed_count rows, where
	/// row r of band b is at [b * speed_count + r] and describes a steady wind of r * 'speed_step' m/s. Each row is (amplitude of an element
	/// travelling downwind (m), alignment exponent, crest sharpness, sum of alignment weights).
	/// </summary>
	public static void SeaTable(in WaveSpectrumLayout layout, float fetch, float speed_step, Span<v4> rows)
	{
		Native.EnsureLoaded();
		fixed (WaveSpectrumLayout* desc = &layout)
		fixed (v4* out_rows = rows)
			Native.Check(Native.Physics_WaveSpectrumSeaTable(desc, fetch, speed_step, out_rows, rows.Length));
	}

	/// <summary>Move 'values' towards 'targets' over 'dt' seconds with an exponential time constant of 'time_constant' seconds.</summary>
	public static void Relax(Span<float> values, ReadOnlySpan<float> targets, float dt, float time_constant)
	{
		if (values.Length != targets.Length)
			throw new ArgumentException("Relax values and targets must have the same length");

		Native.EnsureLoaded();
		fixed (float* vals = values)
		fixed (float* tgts = targets)
			Native.Check(Native.Physics_WaveSpectrumRelax(vals, tgts, values.Length, dt, time_constant));
	}

	/// <summary>
	/// Write a wind-sea Gerstner element, in component order, for each component of 'layout' with a wavelength of at least 'min_wavelength'
	/// whose band can have waves in 'table' (from <see cref="SeaTable"/>). Each element holds its band's largest table amplitude and crest
	/// sharpness; a wind sea supplies the local values. Steepness is zero. Returns the number of elements written.
	/// </summary>
	public static int Elements(in WaveSpectrumLayout layout, ReadOnlySpan<v4> table, float min_wavelength, Span<WaterFieldElement> elements)
	{
		Native.EnsureLoaded();
		var count = 0;
		fixed (WaveSpectrumLayout* desc = &layout)
		fixed (v4* rows = table)
		fixed (WaterFieldElement* out_elements = elements)
			Native.Check(Native.Physics_WaveSpectrumElements(desc, rows, table.Length, min_wavelength, out_elements, elements.Length, &count));

		return count;
	}
}