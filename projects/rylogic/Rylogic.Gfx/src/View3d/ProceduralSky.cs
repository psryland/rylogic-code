using System;
using System.Runtime.InteropServices;
using Rylogic.Maths;
#if PR_UNITTESTS
using Rylogic.UnitTests;
#endif

namespace Rylogic.Gfx;

public sealed partial class View3d
{
	/// <summary>Procedural sky state. The sky is Z-up. Matches 'view3d::ProceduralSkySettings'.</summary>
	[StructLayout(LayoutKind.Sequential)]
	public unsafe struct ProceduralSkySettings
	{
		/// <summary>The maximum number of lightning flashes the sky shows at once.</summary>
		public const int LightningMax = 4;

		/// <summary>Direction toward the sun. Must be finite and nonzero.</summary>
		public v4 SunDirection;

		/// <summary>Linear sun colour. Finite and nonnegative.</summary>
		public v4 SunColour;

		/// <summary>Sun intensity (0 = night, 1 = noon). Finite and nonnegative.</summary>
		public float SunIntensity;

		/// <summary>Default cloud cover in [0,1] (0 = clear, 0.5 = scattered white cloud, 1 = dark overcast), used where no weather map applies.</summary>
		public float CloudCover;

		/// <summary>Speed (>= 0) of the lowest cloud layer in world units per second. Higher layers move proportionally slower.</summary>
		public float WindSpeed;

		/// <summary>Azimuth of cloud travel in radians, measured from +X toward +Y.</summary>
		public float WindDirection;

		/// <summary>Absolute time in seconds. Clouds advance by the change in time between updates; time may not go backwards.</summary>
		public double Time;

		/// <summary>Bit mask of cloud layers to hide: bit i hides layer i (0 = low, 1 = mid, 2 = cirrus). Zero shows all layers.</summary>
		public uint HiddenCloudLayers;

		// Lightning flashes, four floats each. See Lightning.
		private fixed float m_lightning[LightningMax * 4];

		/// <summary>
		/// Strength of the low and mid clouds' shadows in [0,1] (0 = no cloud shadows). Clouds shade the scene's directional light that shines from
		/// the sky's sun, drifting with the clouds.
		/// </summary>
		public float CloudShadowStrength;

		/// <summary>
		/// Brightness of the moon, finite and nonnegative. 0 hides the moon, 1 is a full moon on a clear night. The moon is drawn additively, so it
		/// is naturally faint in a daytime sky. It also adds a dim moonlit glow to the night sky and hides the faintest stars.
		/// </summary>
		public float MoonIntensity;

		/// <summary>Direction toward the moon. Finite and nonzero when the moon is shown.</summary>
		public v4 MoonDirection;

		/// <summary>
		/// Direction from the moon toward the light that lights it, which sets the phase: MoonDirection is a new moon and -MoonDirection is a full moon.
		/// It is independent of the sun direction, so the moon's cycle may run separately from the day. Finite and nonzero when the moon is shown.
		/// </summary>
		public v4 MoonLightDirection;

		/// <summary>
		/// Rotation from the sky frame into the star frame (Z = north celestial pole, X = right ascension 0h). The stars, Milky Way and Magellanic Clouds
		/// are fixed in the star frame, so turning this sets the observer's latitude and wheels the stars through the night. Must be a rotation.
		/// <see cref="CelestialClock"/> can compute this, with matching sun and moon directions, from a simple Sun–Earth–Moon model.
		/// </summary>
		public m4x4 SkyToStars;

		/// <summary>
		/// Scale for the sky's natural light (sun, moon, stars, night air and cubemap background), finite and nonnegative; 1 is normal. Fork lightning and
		/// its glow in the cloud are not scaled, so lowering the exposure during a strike makes the bolt dazzle (see <see cref="ProceduralSky.Dazzle"/>).
		/// </summary>
		public float Exposure;

		/// <summary>
		/// The fork lightning clock in seconds, finite. Strikes start at its current value and each strike's light follows the time since it started.
		/// Use real time, not game time, so strikes look the same at any game speed.
		/// </summary>
		public double LightningTime;

		/// <summary>
		/// Lightning flash 'i' inside the cloud: xy = position in the sky frame, z = radius (> 0), w = finite brightness (>= 0, 1 is a strong flash,
		/// 0 is no flash). The caller decides when flashes happen and how they flicker; the sky shows each flash's current brightness.
		/// </summary>
		public readonly v4 Lightning(int i)
		{
			// Reject slots outside the array rather than reading other fields.
			if (i < 0 || i >= LightningMax)
				throw new ArgumentOutOfRangeException(nameof(i), "Lightning flash index is out of range.");

			return new v4(m_lightning[i * 4 + 0], m_lightning[i * 4 + 1], m_lightning[i * 4 + 2], m_lightning[i * 4 + 3]);
		}
		public void SetLightning(int i, v4 flash)
		{
			// Reject slots outside the array rather than writing other fields.
			if (i < 0 || i >= LightningMax)
				throw new ArgumentOutOfRangeException(nameof(i), "Lightning flash index is out of range.");

			m_lightning[i * 4 + 0] = flash.x;
			m_lightning[i * 4 + 1] = flash.y;
			m_lightning[i * 4 + 2] = flash.z;
			m_lightning[i * 4 + 3] = flash.w;
		}

		/// <summary>Default settings: a clear midday sky with no wind, no moon, and the stars seen from the north pole.</summary>
		public static ProceduralSkySettings Default
		{
			get
			{
				return new ProceduralSkySettings
				{
					SunDirection = new v4(0.5f, 0.3f, 0.8f, 0f),
					SunColour = new v4(1f, 0.95f, 0.85f, 1f),
					SunIntensity = 1f,
					CloudShadowStrength = 1f,
					MoonDirection = new v4(-0.5f, -0.3f, 0.5f, 0f),
					MoonLightDirection = new v4(0.5f, 0.3f, -0.5f, 0f),
					SkyToStars = m4x4.Identity,
					Exposure = 1f,
				};
			}
		}
	}

	/// <summary>The light a fork lightning strike sheds where its bolt touches the ground: the ground point in the scene frame (w = 1), and the flash brightness in (0,1].</summary>
	public readonly struct LightningGroundLight
	{
		/// <summary>Create a ground light at 'position' with flash 'brightness'.</summary>
		public LightningGroundLight(v4 position, float brightness)
		{
			Position = position;
			Brightness = brightness;
		}

		/// <summary>The ground point in the scene frame (w = 1).</summary>
		public v4 Position { get; }

		/// <summary>The flash brightness in (0,1].</summary>
		public float Brightness { get; }
	}

	/// <summary>Owns a Z-up GPU atmosphere. Create, update and dispose on the render owner while the View3d context is alive.</summary>
	public sealed class ProceduralSky : IDisposable
	{
		private readonly Object m_skybox;

		/// <summary>Create a visible atmosphere with clouds and stars and no textures.</summary>
		public ProceduralSky(string name, ProceduralSkySettings settings, Guid? context_id = null)
		{
			var id = context_id ?? Guid.NewGuid();
			var handle = View3D_ObjectCreateProceduralSky(name, ref settings, ref id);
			if (handle == IntPtr.Zero)
				throw new InvalidOperationException("Failed to create the procedural sky.");

			m_skybox = new Object(handle, true);
		}

		/// <summary>The owned scene object; remove it from windows before disposal. Transforms are ignored and no reflection map is generated.</summary>
		public Object Skybox
		{
			get
			{
				return m_skybox;
			}
		}

		/// <summary>Change the sun, clouds and time without rebuilding resources. Invalid parameters leave the previous sky unchanged.</summary>
		public void Update(ProceduralSkySettings settings)
		{
			if (m_skybox.Handle == IntPtr.Zero)
				throw new ObjectDisposedException(nameof(ProceduralSky));

			if (!View3D_ObjectUpdateProceduralSky(m_skybox.Handle, ref settings))
				throw new ArgumentException("The procedural sky parameters were rejected.");
		}

		/// <summary>Set the weather map that varies cloud cover across the sky, or null to use only the default cover. The sky keeps its own reference.</summary>
		public void Weather(WeatherMap? weather)
		{
			if (m_skybox.Handle == IntPtr.Zero)
				throw new ObjectDisposedException(nameof(ProceduralSky));

			if (weather != null && weather.Handle == IntPtr.Zero)
				throw new ObjectDisposedException(nameof(weather));

			if (!View3D_ObjectProceduralSkyWeatherSet(m_skybox.Handle, weather?.Handle ?? IntPtr.Zero))
				throw new ArgumentException("The weather map was rejected.");
		}

		/// <summary>
		/// Start a fork lightning strike at 'ground', a sky-frame point below the cloud base, at the current <see cref="ProceduralSkySettings.LightningTime"/>.
		/// The bolt grows from inside the cloud above the point, lights the cloud around it, and fades. The same 'seed' always grows the same bolt.
		/// </summary>
		public void Strike(v4 ground, uint seed)
		{
			if (m_skybox.Handle == IntPtr.Zero)
				throw new ObjectDisposedException(nameof(ProceduralSky));

			if (!View3D_ObjectProceduralSkyStrike(m_skybox.Handle, ground, seed))
				throw new ArgumentException("The lightning strike was rejected. The ground point must be finite and below the cloud base.");
		}

		/// <summary>
		/// How strongly the fork lightning strikes dazzle the camera of 'window' at 'lightning_time', in [0,1]. A strike dazzles fully while any part
		/// of its bolt is in view, and fades to 'away_scale' (in [0,1]) times as much once the bolt is well outside the view. The sky does not darken
		/// the scene itself: callers read this for the frame they are about to draw, then lower <see cref="ProceduralSkySettings.Exposure"/> and their
		/// own scene lighting by it.
		/// </summary>
		public float Dazzle(View3d.Window window, double lightning_time, float away_scale)
		{
			if (m_skybox.Handle == IntPtr.Zero)
				throw new ObjectDisposedException(nameof(ProceduralSky));

			if (!(away_scale >= 0 && away_scale <= 1))
				throw new ArgumentOutOfRangeException(nameof(away_scale), "The dazzle away scale must be in [0,1].");

			if (!View3D_ObjectProceduralSkyDazzle(m_skybox.Handle, window.Handle, lightning_time, away_scale, out var dazzle))
				throw new InvalidOperationException("Failed to read the lightning dazzle.");

			return dazzle;
		}

		/// <summary>
		/// The ground light of each fork lightning strike whose return strokes are lighting the ground at 'lightning_time'. The sky adds no lights to the
		/// scene: callers that want the light place their own at each ground point. Most of the time there are none.
		/// </summary>
		public LightningGroundLight[] GroundLights(double lightning_time)
		{
			if (m_skybox.Handle == IntPtr.Zero)
				throw new ObjectDisposedException(nameof(ProceduralSky));

			// Each packed entry is the ground point (xyz) and brightness (w).
			Span<v4> packed = stackalloc v4[StrikesMax];
			var count = View3D_ObjectProceduralSkyGroundLights(m_skybox.Handle, lightning_time, ref packed[0], packed.Length);
			if (count < 0)
				throw new InvalidOperationException("Failed to read the lightning ground lights.");

			if (count == 0)
				return [];

			// Positions are points, so their w is 1.
			var lights = new LightningGroundLight[count];
			for (var i = 0; i != count; ++i)
				lights[i] = new LightningGroundLight(new v4(packed[i].x, packed[i].y, packed[i].z, 1), packed[i].w);

			return lights;
		}

		/// <summary>The most fork lightning strikes a sky shows at once.</summary>
		public const int StrikesMax = 4;

		/// <summary>Blend a retained cubemap (weight 0) into the atmosphere (weight 1), using rotations from current scene directions into each background's frame.</summary>
		/// <remarks>Transforms must be finite rotations without scale or translation. The cubemap's own orientation is also respected.
		/// Null background requires weight 1. Invalid input leaves the previous state unchanged.</remarks>
		public void Blend(CubeMap? background, float weight, m4x4 world_to_sky, m4x4 world_to_background)
		{
			if (m_skybox.Handle == IntPtr.Zero)
				throw new ObjectDisposedException(nameof(ProceduralSky));

			if (background != null && background.Handle == IntPtr.Zero)
				throw new ObjectDisposedException(nameof(background));

			if (!View3D_ObjectBlendProceduralSky(m_skybox.Handle, background?.Handle ?? IntPtr.Zero, weight, ref world_to_sky, ref world_to_background))
				throw new ArgumentException("The sky blend parameters were rejected.");
		}

		/// <summary>Release the scene object and its native atmosphere owner; repeated disposal is harmless.</summary>
		public void Dispose()
		{
			m_skybox.Dispose();
		}
	}

	/// <summary>
	/// A grid of cloud cover values in [0,1] over a sky-frame XY area, anchored in world space. Brushes edit a CPU copy; call 'Upload' to show the edits.
	/// Over the outer 10% of the area, cover fades to the sky's default cover. Use on the render owner thread while the View3d context is alive.
	/// </summary>
	public sealed class WeatherMap : IDisposable
	{
		/// <summary>Create a 'width' x 'height' grid (each in [2,4096]) of clear sky over [area_min, area_max).</summary>
		public WeatherMap(int width, int height, v2 area_min, v2 area_max)
		{
			Handle = View3D_WeatherMapCreate(width, height, area_min, area_max);
			if (Handle == IntPtr.Zero)
				throw new InvalidOperationException("Failed to create the weather map.");
		}

		/// <summary>The native handle.</summary>
		public IntPtr Handle { get; private set; }

		/// <summary>Move the area the grid covers, e.g. to follow the camera. The grid contents are unchanged.</summary>
		public void Area(v2 area_min, v2 area_max)
		{
			View3D_WeatherMapAreaSet(Live, area_min, area_max);
		}

		/// <summary>Set every cell to 'cover'.</summary>
		public void Fill(float cover)
		{
			View3D_WeatherMapFill(Live, cover);
		}

		/// <summary>Blend toward 'cover' within a circle. Solid inside 40% of 'radius', fading to nothing at 'radius'.</summary>
		public void AddStormCell(v2 centre, float radius, float cover)
		{
			View3D_WeatherMapAddStormCell(Live, centre, radius, cover);
		}

		/// <summary>Blend toward 'cover' behind a front through 'point'. 'travel_direction' points from the covered side to the clear side; 'width' is the soft edge width.</summary>
		public void AddFront(v2 point, v2 travel_direction, float width, float cover)
		{
			View3D_WeatherMapAddFront(Live, point, travel_direction, width, cover);
		}

		/// <summary>Add smooth variation in [-amplitude, +amplitude] with feature size 'scale', clamped to [0,1].</summary>
		public void AddNoise(float scale, float amplitude, uint seed)
		{
			View3D_WeatherMapAddNoise(Live, scale, amplitude, seed);
		}

		/// <summary>The cover at 'position', faded to 'default_cover' toward the area edges, matching what the sky renders.</summary>
		public float CoverAt(v2 position, float default_cover)
		{
			return View3D_WeatherMapCoverAt(Live, position, default_cover);
		}

		/// <summary>Copy the CPU grid to the GPU so later frames show the edits.</summary>
		public void Upload()
		{
			View3D_WeatherMapUpload(Live);
		}

		/// <summary>Release this reference; skies that use the map keep their own. Repeated disposal is harmless.</summary>
		public void Dispose()
		{
			View3D_WeatherMapRelease(Handle);
			Handle = IntPtr.Zero;
		}

		/// <summary>The handle, or throw if disposed.</summary>
		private IntPtr Live
		{
			get
			{
				if (Handle == IntPtr.Zero)
					throw new ObjectDisposedException(nameof(WeatherMap));

				return Handle;
			}
		}
	}

	[DllImport(Dll, CharSet = CharSet.Ansi)]
	private static extern IntPtr View3D_ObjectCreateProceduralSky([MarshalAs(UnmanagedType.LPStr)] string name, ref ProceduralSkySettings settings, ref Guid context_id);

	[DllImport(Dll)]
	[return: MarshalAs(UnmanagedType.Bool)]
	private static extern bool View3D_ObjectUpdateProceduralSky(IntPtr obj, ref ProceduralSkySettings settings);

	[DllImport(Dll)]
	[return: MarshalAs(UnmanagedType.Bool)]
	private static extern bool View3D_ObjectProceduralSkyWeatherSet(IntPtr obj, IntPtr weather);

	[DllImport(Dll)]
	[return: MarshalAs(UnmanagedType.Bool)]
	private static extern bool View3D_ObjectProceduralSkyStrike(IntPtr obj, v4 ground, uint seed);

	[DllImport(Dll)]
	[return: MarshalAs(UnmanagedType.Bool)]
	private static extern bool View3D_ObjectProceduralSkyDazzle(IntPtr obj, IntPtr window, double lightning_time, float away_scale, out float dazzle);

	[DllImport(Dll)]
	private static extern int View3D_ObjectProceduralSkyGroundLights(IntPtr obj, double lightning_time, ref v4 lights, int capacity);

	// Configure background blending without transferring managed ownership of the source.
	[DllImport(Dll)]
	[return: MarshalAs(UnmanagedType.Bool)]
	private static extern bool View3D_ObjectBlendProceduralSky(IntPtr obj, IntPtr background, float weight, ref m4x4 world_to_sky, ref m4x4 world_to_background);

	[DllImport(Dll)]
	private static extern IntPtr View3D_WeatherMapCreate(int width, int height, v2 area_min, v2 area_max);

	[DllImport(Dll)]
	private static extern void View3D_WeatherMapRelease(IntPtr weather);

	[DllImport(Dll)]
	private static extern void View3D_WeatherMapAreaSet(IntPtr weather, v2 area_min, v2 area_max);

	[DllImport(Dll)]
	private static extern void View3D_WeatherMapFill(IntPtr weather, float cover);

	[DllImport(Dll)]
	private static extern void View3D_WeatherMapAddStormCell(IntPtr weather, v2 centre, float radius, float cover);

	[DllImport(Dll)]
	private static extern void View3D_WeatherMapAddFront(IntPtr weather, v2 point, v2 travel_direction, float width, float cover);

	[DllImport(Dll)]
	private static extern void View3D_WeatherMapAddNoise(IntPtr weather, float scale, float amplitude, uint seed);

	[DllImport(Dll)]
	private static extern float View3D_WeatherMapCoverAt(IntPtr weather, v2 position, float default_cover);

	[DllImport(Dll)]
	private static extern void View3D_WeatherMapUpload(IntPtr weather);
}

#if PR_UNITTESTS
/// <summary>Validate the managed procedural sky contracts independently of native DLL availability.</summary>
[TestFixture]
public class ProceduralSkyTests
{
	/// <summary>The settings match the native layout, and lightning flashes round trip through their slots.</summary>
	[Test]
	public void SettingsLayoutAndLightning()
	{
		var settings = View3d.ProceduralSkySettings.Default;
		Assert.Equal(240, Marshal.SizeOf<View3d.ProceduralSkySettings>());
		Assert.Equal(48, Marshal.OffsetOf<View3d.ProceduralSkySettings>(nameof(View3d.ProceduralSkySettings.Time)).ToInt32());
		Assert.Equal(56, Marshal.OffsetOf<View3d.ProceduralSkySettings>(nameof(View3d.ProceduralSkySettings.HiddenCloudLayers)).ToInt32());
		Assert.Equal(60, Marshal.OffsetOf<View3d.ProceduralSkySettings>("m_lightning").ToInt32());
		Assert.Equal(124, Marshal.OffsetOf<View3d.ProceduralSkySettings>(nameof(View3d.ProceduralSkySettings.CloudShadowStrength)).ToInt32());
		Assert.Equal(128, Marshal.OffsetOf<View3d.ProceduralSkySettings>(nameof(View3d.ProceduralSkySettings.MoonIntensity)).ToInt32());
		Assert.Equal(132, Marshal.OffsetOf<View3d.ProceduralSkySettings>(nameof(View3d.ProceduralSkySettings.MoonDirection)).ToInt32());
		Assert.Equal(148, Marshal.OffsetOf<View3d.ProceduralSkySettings>(nameof(View3d.ProceduralSkySettings.MoonLightDirection)).ToInt32());
		Assert.Equal(164, Marshal.OffsetOf<View3d.ProceduralSkySettings>(nameof(View3d.ProceduralSkySettings.SkyToStars)).ToInt32());
		Assert.Equal(228, Marshal.OffsetOf<View3d.ProceduralSkySettings>(nameof(View3d.ProceduralSkySettings.Exposure)).ToInt32());
		Assert.Equal(232, Marshal.OffsetOf<View3d.ProceduralSkySettings>(nameof(View3d.ProceduralSkySettings.LightningTime)).ToInt32());
		Assert.Equal(1f, settings.Exposure);
		Assert.Equal(1f, settings.CloudShadowStrength);
		Assert.Equal(0f, settings.MoonIntensity);
		Assert.Equal(m4x4.Identity, settings.SkyToStars);
		Assert.Equal(v4.Zero, settings.Lightning(3));

		settings.SetLightning(2, new v4(100, -200, 3000, 0.5f));
		Assert.Equal(new v4(100, -200, 3000, 0.5f), settings.Lightning(2));
		Assert.Throws<ArgumentOutOfRangeException>(() => settings.SetLightning(View3d.ProceduralSkySettings.LightningMax, v4.Zero));
	}
}
#endif