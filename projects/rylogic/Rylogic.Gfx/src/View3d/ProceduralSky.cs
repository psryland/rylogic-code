using System;
using System.Runtime.InteropServices;
using Rylogic.Maths;

namespace Rylogic.Gfx;

public sealed partial class View3d
{
	/// <summary>Procedural sky state. The sky is Z-up. Matches 'view3d::ProceduralSkySettings'.</summary>
	[StructLayout(LayoutKind.Sequential)]
	public struct ProceduralSkySettings
	{
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

		/// <summary>Default settings: a clear midday sky with no wind.</summary>
		public static ProceduralSkySettings Default
		{
			get
			{
				return new ProceduralSkySettings
				{
					SunDirection = new v4(0.5f, 0.3f, 0.8f, 0f),
					SunColour = new v4(1f, 0.95f, 0.85f, 1f),
					SunIntensity = 1f,
				};
			}
		}
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