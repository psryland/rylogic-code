using System;
using System.Runtime.InteropServices;
#if PR_UNITTESTS
using Rylogic.UnitTests;
#endif

namespace Rylogic.Gfx;

public sealed partial class View3d
{
	/// <summary>
	/// Whole-screen "looking through water" post effect: a colour tint, depth-based distance fog, and a moving distortion.
	/// The caller decides when the camera is submerged. The distortion animates only while frames are being rendered.
	/// </summary>
	[StructLayout(LayoutKind.Sequential)]
	public struct UnderwaterProps
	{
		private int m_enabled;
		private Colour32 m_tint;
		private Colour32 m_fog_colour;
		private float m_visibility;
		private float m_distortion_amplitude;
		private float m_distortion_frequency;
		private float m_distortion_speed;

		/// <summary>Create disabled settings with the native defaults.</summary>
		public UnderwaterProps()
		{
			m_enabled = 0;
			m_tint = new Colour32(0xFFA6D9F2);
			m_fog_colour = new Colour32(0xFF0A384D);
			m_visibility = 40.0f;
			m_distortion_amplitude = 0.002f;
			m_distortion_frequency = 6.0f;
			m_distortion_speed = 0.25f;
		}

		/// <summary>Whether the effect is applied.</summary>
		public bool Enabled
		{
			readonly get
			{
				return m_enabled != 0;
			}
			set
			{
				m_enabled = value ? 1 : 0;
			}
		}

		/// <summary>sRGB colour multiplied into the scene colour. Alpha is ignored.</summary>
		public Colour32 Tint
		{
			readonly get
			{
				return m_tint;
			}
			set
			{
				m_tint = value;
			}
		}

		/// <summary>sRGB colour that distant surfaces fade towards. Alpha is ignored.</summary>
		public Colour32 FogColour
		{
			readonly get
			{
				return m_fog_colour;
			}
			set
			{
				m_fog_colour = value;
			}
		}

		/// <summary>Distance (in world units) at which the fog hides 95% of a surface. Must be finite and greater than zero.</summary>
		public float Visibility
		{
			readonly get
			{
				return m_visibility;
			}
			set
			{
				m_visibility = value;
			}
		}

		/// <summary>Largest distortion offset, as a fraction of the viewport height. Zero disables the distortion.</summary>
		public float DistortionAmplitude
		{
			readonly get
			{
				return m_distortion_amplitude;
			}
			set
			{
				m_distortion_amplitude = value;
			}
		}

		/// <summary>Number of distortion ripples per viewport height. Must be finite and greater than zero.</summary>
		public float DistortionFrequency
		{
			readonly get
			{
				return m_distortion_frequency;
			}
			set
			{
				m_distortion_frequency = value;
			}
		}

		/// <summary>Distortion animation rate, in cycles per second. Must be finite and not negative.</summary>
		public float DistortionSpeed
		{
			readonly get
			{
				return m_distortion_speed;
			}
			set
			{
				m_distortion_speed = value;
			}
		}

		/// <summary>Disabled settings matching a newly created native scene.</summary>
		public static UnderwaterProps Default()
		{
			return new UnderwaterProps();
		}

		/// <summary>Reject invalid settings, even while the effect is disabled.</summary>
		public readonly void Validate()
		{
			if (float.IsNaN(m_visibility) || float.IsInfinity(m_visibility) || m_visibility <= 0)
				throw new ArgumentOutOfRangeException(nameof(Visibility), "Underwater visibility must be finite and greater than zero.");
			if (float.IsNaN(m_distortion_amplitude) || float.IsInfinity(m_distortion_amplitude) || m_distortion_amplitude < 0)
				throw new ArgumentOutOfRangeException(nameof(DistortionAmplitude), "Underwater distortion amplitude must be finite and not negative.");
			if (float.IsNaN(m_distortion_frequency) || float.IsInfinity(m_distortion_frequency) || m_distortion_frequency <= 0)
				throw new ArgumentOutOfRangeException(nameof(DistortionFrequency), "Underwater distortion frequency must be finite and greater than zero.");
			if (float.IsNaN(m_distortion_speed) || float.IsInfinity(m_distortion_speed) || m_distortion_speed < 0)
				throw new ArgumentOutOfRangeException(nameof(DistortionSpeed), "Underwater distortion speed must be finite and not negative.");
		}
	}

	[DllImport(Dll)]
	private static extern UnderwaterProps View3D_PostEffectUnderwaterGet(IntPtr window);

	[DllImport(Dll)]
	[return: MarshalAs(UnmanagedType.Bool)]
	private static extern bool View3D_PostEffectUnderwaterSet(IntPtr window, ref UnderwaterProps props);
}

#if PR_UNITTESTS
/// <summary>Validate the managed post effect contracts independently of native DLL availability.</summary>
[TestFixture]
public class PostEffectTests
{
	/// <summary>New views have the effect disabled, with a stable 28-byte ABI.</summary>
	[Test]
	public void UnderwaterDefaultAndLayout()
	{
		var props = View3d.UnderwaterProps.Default();
		Assert.False(props.Enabled);
		Assert.Equal(0xFFA6D9F2U, props.Tint.ARGB);
		Assert.Equal(0xFF0A384DU, props.FogColour.ARGB);
		Assert.Equal(40.0f, props.Visibility);
		Assert.Equal(0.002f, props.DistortionAmplitude);
		Assert.Equal(6.0f, props.DistortionFrequency);
		Assert.Equal(0.25f, props.DistortionSpeed);
		Assert.Equal(28, Marshal.SizeOf<View3d.UnderwaterProps>());
		Assert.Equal(4, Marshal.OffsetOf<View3d.UnderwaterProps>("m_tint").ToInt32());
		Assert.Equal(8, Marshal.OffsetOf<View3d.UnderwaterProps>("m_fog_colour").ToInt32());
		Assert.Equal(12, Marshal.OffsetOf<View3d.UnderwaterProps>("m_visibility").ToInt32());
		Assert.Equal(24, Marshal.OffsetOf<View3d.UnderwaterProps>("m_distortion_speed").ToInt32());
	}

	/// <summary>Invalid settings fail before reaching the native setter.</summary>
	[Test]
	public void UnderwaterInvalid()
	{
		var props = View3d.UnderwaterProps.Default();
		props.Validate();

		props.Visibility = 0;
		Assert.Throws<ArgumentOutOfRangeException>(() => props.Validate());
		props = View3d.UnderwaterProps.Default();
		props.DistortionAmplitude = -1;
		Assert.Throws<ArgumentOutOfRangeException>(() => props.Validate());
		props = View3d.UnderwaterProps.Default();
		props.DistortionFrequency = float.NaN;
		Assert.Throws<ArgumentOutOfRangeException>(() => props.Validate());
		props = View3d.UnderwaterProps.Default();
		props.DistortionSpeed = float.PositiveInfinity;
		Assert.Throws<ArgumentOutOfRangeException>(() => props.Validate());
	}
}
#endif
