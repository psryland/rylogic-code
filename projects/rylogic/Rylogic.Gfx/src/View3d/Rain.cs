using System;
using System.Runtime.InteropServices;
using Rylogic.Maths;
#if PR_UNITTESTS
using Rylogic.UnitTests;
#endif

namespace Rylogic.Gfx;

public sealed partial class View3d
{
	/// <summary>The state of the rain. The frame is any right-handed frame; the fall velocity decides which way is down. Matches 'view3d::RainSettings'.</summary>
	[StructLayout(LayoutKind.Sequential)]
	public struct RainSettings
	{
		/// <summary>The velocity of the air, in world units per second. The drops drift with it. Must be finite.</summary>
		public v4 Wind;

		/// <summary>The velocity of a falling drop in still air, in world units per second. Drops fall at between 80% and 120% of it. Finite and nonzero.</summary>
		public v4 FallVelocity;

		/// <summary>The velocity of the camera, in world units per second. Streaks show the drops' motion relative to the camera. Must be finite.</summary>
		public v4 CameraVelocity;

		/// <summary>The colour of a lit drop (linear rgb, finite, nonnegative). The caller lights the rain, e.g. with the sky's ambient light.</summary>
		public v4 Colour;

		/// <summary>How hard it rains, in [0,1]. 0 is no rain, small values are a light sparse shower, and 1 is a heavy downpour.</summary>
		public float Intensity;

		/// <summary>The most a single streak covers what is behind it, in [0,1].</summary>
		public float Opacity;

		/// <summary>The exposure time of the streaks, in seconds (> 0). Longer times give longer streaks.</summary>
		public float StreakTime;

		/// <summary>The width of a drop at full intensity, in world units (> 0).</summary>
		public float DropWidth;

		/// <summary>The caller's clock, in seconds (finite). Drops move by the change in time since the previous update, so a slower clock gives slow-motion rain.</summary>
		public double Time;

		/// <summary>Default settings: no rain yet, falling at 7 units per second down the Z axis, in still air.</summary>
		public static RainSettings Default
		{
			get
			{
				return new RainSettings
				{
					FallVelocity = new v4(0, 0, -7, 0),
					Colour = new v4(0.6f, 0.65f, 0.7f, 1f),
					Opacity = 0.25f,
					StreakTime = 0.02f,
					DropWidth = 0.0015f,
				};
			}
		}
	}

	/// <summary>
	/// Owns rain that fills all of space around whichever camera draws it, so it looks the same at any camera speed. Drops are drawn as streaks
	/// along their motion relative to the camera. Create, update and dispose on the render owner while the View3d context is alive.
	/// </summary>
	public sealed class Rain : IDisposable
	{
		private readonly Object m_object;

		/// <summary>Create the rain. Throws for invalid settings.</summary>
		public Rain(string name, RainSettings settings, Guid? context_id = null)
		{
			var id = context_id ?? Guid.NewGuid();
			var handle = View3D_ObjectCreateRain(name, ref settings, ref id);
			if (handle == IntPtr.Zero)
				throw new InvalidOperationException("Failed to create the rain.");

			m_object = new Object(handle, true);
		}

		/// <summary>The owned scene object; remove it from windows before disposal. Transforms are ignored.</summary>
		public Object Object
		{
			get
			{
				return m_object;
			}
		}

		/// <summary>
		/// Change the rain and move its drops by the change in <see cref="RainSettings.Time"/>. Throws <see cref="ArgumentException"/> for invalid
		/// settings, which leave the rain unchanged.
		/// </summary>
		public void Update(RainSettings settings)
		{
			// The native owner validates the settings, so a rejection means the caller broke the settings contract.
			if (m_object.Handle == IntPtr.Zero)
				throw new ObjectDisposedException(nameof(Rain));

			if (!View3D_ObjectUpdateRain(m_object.Handle, ref settings))
				throw new ArgumentException("The rain settings were rejected.");
		}

		/// <summary>Release the scene object and its native rain owner; repeated disposal is harmless.</summary>
		public void Dispose()
		{
			// The object owns the native rain, so releasing it releases both.
			m_object.Dispose();
		}
	}

	[DllImport(Dll, CharSet = CharSet.Ansi)]
	private static extern IntPtr View3D_ObjectCreateRain([MarshalAs(UnmanagedType.LPStr)] string name, ref RainSettings settings, ref Guid context_id);

	[DllImport(Dll)]
	[return: MarshalAs(UnmanagedType.Bool)]
	private static extern bool View3D_ObjectUpdateRain(IntPtr obj, ref RainSettings settings);
}

#if PR_UNITTESTS
/// <summary>Validate the managed rain contracts independently of native DLL availability.</summary>
[TestFixture]
public class RainTests
{
	/// <summary>The settings match the native layout.</summary>
	[Test]
	public void SettingsLayout()
	{
		Assert.Equal(88, Marshal.SizeOf<View3d.RainSettings>());
		Assert.Equal(48, Marshal.OffsetOf<View3d.RainSettings>(nameof(View3d.RainSettings.Colour)).ToInt32());
		Assert.Equal(64, Marshal.OffsetOf<View3d.RainSettings>(nameof(View3d.RainSettings.Intensity)).ToInt32());
		Assert.Equal(76, Marshal.OffsetOf<View3d.RainSettings>(nameof(View3d.RainSettings.DropWidth)).ToInt32());
		Assert.Equal(80, Marshal.OffsetOf<View3d.RainSettings>(nameof(View3d.RainSettings.Time)).ToInt32());
	}
}
#endif
