using System;
using System.Runtime.InteropServices;
using Rylogic.Maths;
#if PR_UNITTESTS
using Rylogic.UnitTests;
#endif

namespace Rylogic.Gfx;

public sealed partial class View3d
{
	/// <summary>
	/// Whole-screen "looking through water" post effect: a colour tint, distance fog below the water surface, and a moving distortion.
	/// Without a Surface, the caller decides when the camera is submerged. With one, only pixels whose near-plane point is below the surface
	/// get the effect. The distortion animates only while frames are being rendered.
	/// </summary>
	[StructLayout(LayoutKind.Sequential)]
	public unsafe struct UnderwaterProps
	{
		/// <summary>Number of waterline offset samples along each edge of the near plane. See SetWaterlineOffset.</summary>
		public const int WaterlineGridSize = 8;

		private int m_enabled;
		private Colour32 m_tint;
		private Colour32 m_fog_colour;
		private float m_visibility;
		private float m_distortion_amplitude;
		private float m_distortion_frequency;
		private float m_distortion_speed;
		private v4 m_surface;
		private float m_fade_depth;
		private fixed float m_waterline_offsets[WaterlineGridSize * WaterlineGridSize];

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
			m_surface = v4.Zero;
			m_fade_depth = 0.0f;
			for (var i = 0; i != WaterlineGridSize * WaterlineGridSize; ++i)
				m_waterline_offsets[i] = 0.0f;
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

		/// <summary>sRGB colour that distant surfaces fade towards. Alpha is the fog strength, the largest fraction of the scene colour the fog replaces.</summary>
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

		/// <summary>Distance (in world units) at which the fog hides 95% of a surface. Must be greater than zero. Positive infinity disables the fog.</summary>
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

		/// <summary>
		/// World-space water surface plane: xyz is the normal pointing out of the water, and Dot(Surface, point) > 0 above the water.
		/// Pixels whose near-plane point is above this plane keep the scene colour, so a camera part-way through the surface shows a waterline,
		/// and nothing is drawn while the whole near plane is above it. Below the surface, fog applies only to the part of each view ray in water,
		/// so surfaces seen up through the water surface stay visible. Zero means there is no surface and the whole view is in water.
		/// </summary>
		public v4 Surface
		{
			readonly get
			{
				return m_surface;
			}
			set
			{
				m_surface = value;
			}
		}

		/// <summary>
		/// Depth below the Surface (in world units) over which the effect fades in. A pixel's effect strength rises smoothly from none where its
		/// near-plane point is at the surface to full where that point is this deep. Zero gives a sharp waterline. Must be finite and not negative.
		/// </summary>
		public float FadeDepth
		{
			readonly get
			{
				return m_fade_depth;
			}
			set
			{
				m_fade_depth = value;
			}
		}

		/// <summary>Disabled settings matching a newly created native scene.</summary>
		public static UnderwaterProps Default()
		{
			return new UnderwaterProps();
		}

		/// <summary>
		/// The real water surface's height above Surface, measured along its normal, at sample (i, j) of a WaterlineGridSize square grid of points
		/// on the camera's near plane. Sample (i, j) is at viewport-normalised position (i, j) / (WaterlineGridSize - 1), where (0, 0) is the
		/// top-left corner. The waterline follows the surface height interpolated between the samples, so it can follow waves that a plane cannot.
		/// Fog still uses the plane. All zeros means the surface is the plane. Values must be finite.
		/// </summary>
		public readonly float WaterlineOffset(int i, int j)
		{
			// Reject positions outside the grid rather than reading other fields.
			if (i < 0 || i >= WaterlineGridSize || j < 0 || j >= WaterlineGridSize)
				throw new ArgumentOutOfRangeException(nameof(i), "Waterline sample is outside the grid.");

			return m_waterline_offsets[j * WaterlineGridSize + i];
		}
		public void SetWaterlineOffset(int i, int j, float offset)
		{
			// Reject positions outside the grid rather than writing other fields.
			if (i < 0 || i >= WaterlineGridSize || j < 0 || j >= WaterlineGridSize)
				throw new ArgumentOutOfRangeException(nameof(i), "Waterline sample is outside the grid.");

			m_waterline_offsets[j * WaterlineGridSize + i] = offset;
		}

		/// <summary>Reject invalid settings, even while the effect is disabled.</summary>
		public readonly void Validate()
		{
			if (float.IsNaN(m_visibility) || m_visibility <= 0)
				throw new ArgumentOutOfRangeException(nameof(Visibility), "Underwater visibility must be greater than zero, or positive infinity for no fog.");
			if (float.IsNaN(m_distortion_amplitude) || float.IsInfinity(m_distortion_amplitude) || m_distortion_amplitude < 0)
				throw new ArgumentOutOfRangeException(nameof(DistortionAmplitude), "Underwater distortion amplitude must be finite and not negative.");
			if (float.IsNaN(m_distortion_frequency) || float.IsInfinity(m_distortion_frequency) || m_distortion_frequency <= 0)
				throw new ArgumentOutOfRangeException(nameof(DistortionFrequency), "Underwater distortion frequency must be finite and greater than zero.");
			if (float.IsNaN(m_distortion_speed) || float.IsInfinity(m_distortion_speed) || m_distortion_speed < 0)
				throw new ArgumentOutOfRangeException(nameof(DistortionSpeed), "Underwater distortion speed must be finite and not negative.");
			if (!Math_.IsFinite(m_surface) || (m_surface != v4.Zero && m_surface.w0.LengthSq == 0))
				throw new ArgumentOutOfRangeException(nameof(Surface), "Underwater surface must be finite, and either zero or have a non-zero normal.");
			if (float.IsNaN(m_fade_depth) || float.IsInfinity(m_fade_depth) || m_fade_depth < 0)
				throw new ArgumentOutOfRangeException(nameof(FadeDepth), "Underwater fade depth must be finite and not negative.");
			for (var i = 0; i != WaterlineGridSize * WaterlineGridSize; ++i)
			{
				// Each offset must be a finite height.
				if (float.IsNaN(m_waterline_offsets[i]) || float.IsInfinity(m_waterline_offsets[i]))
					throw new ArgumentOutOfRangeException(nameof(SetWaterlineOffset), "Underwater waterline offsets must be finite.");
			}
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
	/// <summary>New views have the effect disabled, with a stable 304-byte ABI.</summary>
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
		Assert.Equal(304, Marshal.SizeOf<View3d.UnderwaterProps>());
		Assert.Equal(4, Marshal.OffsetOf<View3d.UnderwaterProps>("m_tint").ToInt32());
		Assert.Equal(8, Marshal.OffsetOf<View3d.UnderwaterProps>("m_fog_colour").ToInt32());
		Assert.Equal(12, Marshal.OffsetOf<View3d.UnderwaterProps>("m_visibility").ToInt32());
		Assert.Equal(24, Marshal.OffsetOf<View3d.UnderwaterProps>("m_distortion_speed").ToInt32());
		Assert.Equal(28, Marshal.OffsetOf<View3d.UnderwaterProps>("m_surface").ToInt32());
		Assert.Equal(44, Marshal.OffsetOf<View3d.UnderwaterProps>("m_fade_depth").ToInt32());
		Assert.Equal(48, Marshal.OffsetOf<View3d.UnderwaterProps>("m_waterline_offsets").ToInt32());
		Assert.Equal(v4.Zero, props.Surface);
		Assert.Equal(0.0f, props.FadeDepth);
		Assert.Equal(0.0f, props.WaterlineOffset(7, 7));
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
		props = View3d.UnderwaterProps.Default();
		props.Surface = new v4(0, 0, 0, 1);
		Assert.Throws<ArgumentOutOfRangeException>(() => props.Validate());
		props.Surface = new v4(0, 0, 2, -10);
		props.Validate();
		props.FadeDepth = -1;
		Assert.Throws<ArgumentOutOfRangeException>(() => props.Validate());

		// Infinite visibility disables the fog; offsets must be finite and inside the grid.
		props = View3d.UnderwaterProps.Default();
		props.Visibility = float.PositiveInfinity;
		props.Validate();
		props.SetWaterlineOffset(3, 5, 1.5f);
		Assert.Equal(1.5f, props.WaterlineOffset(3, 5));
		props.Validate();
		props.SetWaterlineOffset(0, 7, float.NaN);
		Assert.Throws<ArgumentOutOfRangeException>(() => props.Validate());
		Assert.Throws<ArgumentOutOfRangeException>(() => props.SetWaterlineOffset(8, 0, 0));
	}
}
#endif
