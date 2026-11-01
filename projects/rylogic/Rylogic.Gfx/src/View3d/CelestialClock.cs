using System.Runtime.InteropServices;
using Rylogic.Maths;
#if PR_UNITTESTS
using Rylogic.UnitTests;
#endif

namespace Rylogic.Gfx;

public sealed partial class View3d
{
	/// <summary>
	/// The sun, moon and star state for one moment, in the procedural sky's frame (+Z up, +Y north, +X east). Matches 'view3d::CelestialState'.
	/// The directions and rotation can be copied directly into <see cref="ProceduralSkySettings"/>.
	/// </summary>
	[StructLayout(LayoutKind.Sequential)]
	public struct CelestialState
	{
		/// <summary>Unit direction toward the sun.</summary>
		public v4 SunDirection;

		/// <summary>Unit direction toward the moon.</summary>
		public v4 MoonDirection;

		/// <summary>Unit direction from the moon toward the sun, which sets the moon's lit side.</summary>
		public v4 MoonLightDirection;

		/// <summary>Rotation from the sky frame to the star frame (+Z toward the north celestial pole, +X toward right ascension zero).</summary>
		public m4x4 SkyToStars;

		/// <summary>Fraction of the moon's visible face that is lit: 0 for a new moon, 1 for a full moon.</summary>
		public float LitFraction;
	}

	/// <summary>
	/// The moment seen by the native Sun–Earth–Moon model (see 'CelestialClock' in pr/view3d-12/scene/celestial_clock.h): the observer's latitude,
	/// the date and time, and the moon's age. The caller decides how fast time passes by calling <see cref="Advance"/>.
	/// The default moment is midnight at the start of 1 January, at a new moon, seen from the equator. Evaluating needs the View3D DLL but no context.
	/// </summary>
	public sealed class CelestialClock
	{
		/// <summary>Days in the model's year. Must match celestial_clock.h.</summary>
		public const double DaysPerYear = 365;

		/// <summary>Days from one new moon to the next. Must match celestial_clock.h.</summary>
		public const double SynodicMonthDays = 29.53;

		/// <summary>Day of the year, counted from zero at the start of 1 January, of the March equinox. Must match celestial_clock.h.</summary>
		public const double MarchEquinoxDay = 79;

		private double m_day;
		private double m_moon_age;

		/// <summary>Latitude of the observer, in degrees within [-90, 90]. Negative is the southern hemisphere.</summary>
		public float Latitude { get; set; }

		/// <summary>Days since midnight at the start of 1 January, in [0, <see cref="DaysPerYear"/>). The fraction is the time of day. Assigned values wrap.</summary>
		public double Day
		{
			get
			{
				// The value is always wrapped into one year.
				return m_day;
			}
			set
			{
				// Wrap so the year repeats.
				m_day = Wrap(value, DaysPerYear);
			}
		}

		/// <summary>Days since the last new moon, in [0, <see cref="SynodicMonthDays"/>). About 14.8 is full. Assigned values wrap.</summary>
		public double MoonAge
		{
			get
			{
				// The value is always wrapped into one month.
				return m_moon_age;
			}
			set
			{
				// Wrap so the phases repeat.
				m_moon_age = Wrap(value, SynodicMonthDays);
			}
		}

		/// <summary>Move time forward by 'days'. The moon ages by the same amount.</summary>
		public void Advance(double days)
		{
			// The date and the moon share one clock, so the moon's phase stays tied to the sun.
			Day += days;
			MoonAge += days;
		}

		/// <summary>The sun, moon and stars at the current moment.</summary>
		public CelestialState Evaluate()
		{
			// The native model owns the astronomy.
			View3D_CelestialEvaluate(Latitude, m_day, m_moon_age, out var state);
			return state;
		}

		/// <summary>Return 'value' wrapped into [0, 'period').</summary>
		private static double Wrap(double value, double period)
		{
			// Floor keeps negative values in range too.
			return value - period * System.Math.Floor(value / period);
		}
	}

	[DllImport(Dll)]
	private static extern void View3D_CelestialEvaluate(float latitude, double day, double moon_age, out CelestialState state);
}

#if PR_UNITTESTS
/// <summary>Validate the managed celestial clock independently of native DLL availability. The native model has its own tests.</summary>
[TestFixture]
public class CelestialClockTests
{
	/// <summary>The state matches the native layout, and the clock advances and wraps its date and moon age together.</summary>
	[Test]
	public void LayoutAndAdvance()
	{
		// The layout must match 'view3d::CelestialState'.
		Assert.Equal(116, Marshal.SizeOf<View3d.CelestialState>());
		Assert.Equal(48, Marshal.OffsetOf<View3d.CelestialState>(nameof(View3d.CelestialState.SkyToStars)).ToInt32());
		Assert.Equal(112, Marshal.OffsetOf<View3d.CelestialState>(nameof(View3d.CelestialState.LitFraction)).ToInt32());

		// Advancing moves the date and the moon together, and assigned values wrap.
		var clock = new View3d.CelestialClock { Day = View3d.CelestialClock.MarchEquinoxDay + 0.5, MoonAge = 0 };
		clock.Advance(0.5);
		Assert.Equal(View3d.CelestialClock.MarchEquinoxDay + 1, clock.Day, 1e-9);
		Assert.Equal(0.5, clock.MoonAge, 1e-9);
		clock.Day = -1;
		Assert.Equal(View3d.CelestialClock.DaysPerYear - 1, clock.Day, 1e-9);
	}
}
#endif
