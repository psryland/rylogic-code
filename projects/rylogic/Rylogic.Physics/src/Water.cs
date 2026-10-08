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

	public readonly double m_level;
	public readonly float m_density;
	public readonly float m_linear_drag_rate;
	public readonly float m_quadratic_drag_coefficient;
	public readonly float m_angular_drag_rate;
	public readonly float m_breaking_ratio;
	public readonly float m_repeat_period;

	/// <summary>
	/// Specify the still-water level in metres and the water's density and drag. 'breaking_ratio' limits the total wave amplitude to half this
	/// fraction of the depth where terrain heights are known (see <see cref="Engine.SetWaterBathymetry"/>). 'repeat_period' (s) is the period after
	/// which every wave element repeats, so the engine can wrap long clocks without losing precision; zero means the elements are not periodic.
	/// </summary>
	public WaterConfiguration(double level, float density = 1000.0f, float linear_drag_rate = 0.5f, float quadratic_drag_coefficient = 0.5f, float angular_drag_rate = 0.5f, float breaking_ratio = DefaultBreakingRatio, float repeat_period = 0.0f)
	{
		m_level = level;
		m_density = density;
		m_linear_drag_rate = linear_drag_rate;
		m_quadratic_drag_coefficient = quadratic_drag_coefficient;
		m_angular_drag_rate = angular_drag_rate;
		m_breaking_ratio = breaking_ratio;
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

/// <summary>Wind conditions that drive a <see cref="WaveSpectrum"/>. Spectrum directions are relative to downwind, so no direction is needed.</summary>
public readonly struct WaveWeather
{
	/// <summary>Wind speed 10 m above the water, in m/s. Speeds below 0.1 m/s produce no waves.</summary>
	public readonly float WindSpeed;

	/// <summary>Distance over which the wind has blown across open water, in metres. Longer fetches produce longer, higher waves.</summary>
	public readonly float Fetch;

	/// <summary>Specify the wind speed (m/s) and the fetch (m).</summary>
	public WaveWeather(float wind_speed, float fetch)
	{
		WindSpeed = wind_speed;
		Fetch = fetch;
	}
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

	/// <summary>The number of components in each band, each travelling in a different direction relative to downwind.</summary>
	public readonly int Directions;

	/// <summary>Describe a spectrum of 'bands' * 'directions' components (at most 64) between 'min_wavelength' and 'max_wavelength'.</summary>
	public WaveSpectrumLayout(float gravity, float repeat_period, float min_wavelength, float max_wavelength, int bands, int directions)
	{
		Gravity = gravity;
		RepeatPeriod = repeat_period;
		MinWavelength = min_wavelength;
		MaxWavelength = max_wavelength;
		Bands = bands;
		Directions = directions;
	}

	/// <summary>The number of components, and the length of every amplitude array.</summary>
	public int ComponentCount
	{
		get
		{
			return Bands * Directions;
		}
	}
}

/// <summary>
/// A fixed set of wind-driven wave components whose amplitudes follow the weather. The components depend only on the layout, so changing the
/// wind strength only changes amplitudes and the surface never jumps. Component directions are relative to downwind and are rotated to the
/// actual wind direction by <see cref="Elements"/>. Every component repeats after the layout's repeat period.
/// </summary>
public static unsafe class WaveSpectrum
{
	/// <summary>Write the equilibrium amplitude (m) of every component of 'layout' for 'weather' into 'amplitudes'.</summary>
	public static void Targets(in WaveSpectrumLayout layout, WaveWeather weather, Span<float> amplitudes)
	{
		Native.EnsureLoaded();
		fixed (WaveSpectrumLayout* desc = &layout)
		fixed (float* amps = amplitudes)
			Native.Check(Native.Physics_WaveSpectrumTargets(desc, weather.WindSpeed, weather.Fetch, amps, amplitudes.Length));
	}

	/// <summary>Move 'amplitudes' towards 'targets' over 'dt' seconds with an exponential time constant of 'time_constant' seconds.</summary>
	public static void Relax(Span<float> amplitudes, ReadOnlySpan<float> targets, float dt, float time_constant)
	{
		if (amplitudes.Length != targets.Length)
			throw new ArgumentException("Wave amplitudes and targets must have the same length");

		Native.EnsureLoaded();
		fixed (float* amps = amplitudes)
		fixed (float* tgts = targets)
			Native.Check(Native.Physics_WaveSpectrumRelax(amps, tgts, amplitudes.Length, dt, time_constant));
	}

	/// <summary>
	/// Return the crest sharpness of wind-driven waves for 'wind_speed' (m/s, finite and non-negative), for use with <see cref="Elements"/>.
	/// Light winds (up to 4 m/s) make sine-shaped waves; stronger winds make narrower crests and flatter troughs, approaching 0.7 in storms.
	/// </summary>
	public static float CrestSharpness(float wind_speed)
	{
		Native.EnsureLoaded();
		var sharpness = 0f;
		Native.Check(Native.Physics_WaveSpectrumCrestSharpness(wind_speed, &sharpness));
		return sharpness;
	}

	/// <summary>
	/// Write Gerstner elements, in component order, for the components of 'layout' with a positive amplitude and a wavelength of at least
	/// 'min_wavelength'. 'heading' is the direction the wind blows towards, in radians anticlockwise from +X; component directions are rotated
	/// by it. 'sharpness' in [0, 0.9] narrows crests and flattens troughs (see <see cref="CrestSharpness"/>) and is stored in each element's
	/// timing.x. Steepness is zero. Returns the number of elements written.
	/// </summary>
	public static int Elements(in WaveSpectrumLayout layout, ReadOnlySpan<float> amplitudes, float heading, float sharpness, float min_wavelength, Span<WaterFieldElement> elements)
	{
		Native.EnsureLoaded();
		var count = 0;
		fixed (WaveSpectrumLayout* desc = &layout)
		fixed (float* amps = amplitudes)
		fixed (WaterFieldElement* out_elements = elements)
			Native.Check(Native.Physics_WaveSpectrumElements(desc, amps, amplitudes.Length, heading, sharpness, min_wavelength, out_elements, elements.Length, &count));

		return count;
	}
}