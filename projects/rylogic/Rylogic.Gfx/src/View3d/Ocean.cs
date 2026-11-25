using System;
using System.Runtime.InteropServices;
#if PR_UNITTESTS
using Rylogic.UnitTests;
#endif

namespace Rylogic.Gfx;

public sealed partial class View3d
{
	/// <summary>
	/// The fixed layout of an ocean's wind sea: waves whose amplitude and crest sharpness follow the wind at each place. Wind node (i, j) is at
	/// (OriginX, OriginY) + CellMetres * (i, j) and holds LevelCount wind vectors; the table holds SpeedCount rows per band at multiples of
	/// SpeedStepMetresPerSecond. Matches 'view3d::OceanWindSeaGrid'. See pr/physics/terrain/water/wind_sea.hlsli for the full contract.
	/// </summary>
	[StructLayout(LayoutKind.Sequential)]
	public struct OceanWindSeaGrid
	{
		/// <summary>World position (m) of node (0, 0).</summary>
		public double OriginX;
		public double OriginY;

		/// <summary>Node spacing (m). Positive.</summary>
		public double CellMetres;

		/// <summary>Wind speed (m/s) between table rows. Positive.</summary>
		public double SpeedStepMetresPerSecond;

		/// <summary>Nodes along x and y, each at least 2.</summary>
		public int Width;
		public int Height;

		/// <summary>Wavelength bands, in [1, 5].</summary>
		public int BandCount;

		/// <summary>Table rows per band, at least 2.</summary>
		public int SpeedCount;

		/// <summary>Wind vectors per node, in [1, BandCount].</summary>
		public int LevelCount;

		private int m_reserved;

		/// <summary>Create a layout.</summary>
		public OceanWindSeaGrid(double origin_x, double origin_y, double cell_metres, double speed_step_mps, int width, int height, int band_count, int speed_count, int level_count)
		{
			OriginX = origin_x;
			OriginY = origin_y;
			CellMetres = cell_metres;
			SpeedStepMetresPerSecond = speed_step_mps;
			Width = width;
			Height = height;
			BandCount = band_count;
			SpeedCount = speed_count;
			LevelCount = level_count;
			m_reserved = 0;
		}

		/// <summary>The number of floats in the table: four per row.</summary>
		public readonly long TableLength
		{
			get
			{
				return 4L * BandCount * SpeedCount;
			}
		}

		/// <summary>The number of floats in the winds: an (x, y) pair per level per node.</summary>
		public readonly long WindsLength
		{
			get
			{
				return 2L * Width * Height * LevelCount;
			}
		}
	}

	/// <summary>Per-frame state of an <see cref="Ocean"/>. Matches 'view3d::OceanFrame'.</summary>
	[StructLayout(LayoutKind.Sequential)]
	public struct OceanFrame
	{
		/// <summary>Camera world position (m).</summary>
		public double CameraX;
		public double CameraY;
		public double CameraZ;

		/// <summary>The wave clock (s), already wrapped by the caller into the spectrum's repeat period.</summary>
		public double WaveTimeSeconds;

		/// <summary>The still-water level (m).</summary>
		public double LevelMetres;

		/// <summary>Waves fade to flat water at this distance (m) from the camera, so no wave maths runs beyond it.</summary>
		public double WaveRadiusMetres;

		/// <summary>Vertex spacing (m) at the camera.</summary>
		public double MinimumSpacingMetres;

		/// <summary>Growth of the vertex spacing with distance from the camera; 1 is uniform.</summary>
		public double WarpPower;

		/// <summary>Waves break where their height exceeds this fraction of the depth.</summary>
		public double BreakingRatio;

		/// <summary>Crest sharpness that waves reach as they near the breaking height, in [0, 0.9], so waves peak as they run into the shore.</summary>
		public double ShoreSharpness;

		/// <summary>
		/// How hard it rains on the water, in [0, 1]: the fraction of a downpour's drops that fall. Each drop spreads a ring near the camera, and a downpour
		/// churns the surface. The drops follow the wave clock, and their pattern continues smoothly when the clock wraps at a multiple of 1024 seconds.
		/// A zero intensity, ring radius, ring wavelength or ring slope draws no rings.
		/// </summary>
		public double RainIntensity;

		/// <summary>The largest radius (m) that a drop's ring spreads to, finite and non-negative.</summary>
		public double RainRingRadiusMetres;

		/// <summary>
		/// The wavelength (m) of a new ring's ripples, finite and non-negative. It doubles as the ring spreads. Rings show only where the pixels are a few
		/// times smaller than their wavelength, so longer ripples show further from the camera.
		/// </summary>
		public double RainRingWavelengthMetres;

		/// <summary>The steepest slope of a new ring's ripples, finite and non-negative. It falls to zero as the ring spreads.</summary>
		public double RainRingSlope;

		/// <summary>Surface tint; alpha is the opacity when viewed from directly above.</summary>
		public uint ColourArgb;

		/// <summary>Non-zero to grow and break waves in shallow water.</summary>
		public uint DepthEffects;
	}

	/// <summary>
	/// The white foam an <see cref="Ocean"/> draws, and the dark lip of the water at the waterline. All foam is one foam texture, revealed by a foam
	/// amount: shore foam where the water laps the ground, crest foam on breaking wave crests, and the trails that crests leave behind, which fade
	/// where they formed. Shore foam and the lip are drawn only within RadiusMetres of the camera. Every value is finite and non-negative; the shore
	/// foam depth, softness, crest threshold, tile size, lip height, radius, fade, crest range, trail time, and crest depth are positive. A zero
	/// strength turns that effect off. Matches 'view3d::OceanFoam'.
	/// </summary>
	[StructLayout(LayoutKind.Sequential)]
	public struct OceanFoam
	{
		/// <summary>Vertical water depth (m) at which the shore foam fades out. Shallower water foams more, up to full cover at the waterline.</summary>
		public double ShoreFoamDepthMetres;

		/// <summary>Blur of the foam's broken edge, as a fraction of the foam pattern's range. Small values give crisp patches.</summary>
		public double EdgeSoftness;

		/// <summary>Crest foam amount on the sharpest breaking crests; 0 turns crest foam and its trails off.</summary>
		public double CrestStrength;

		/// <summary>Crest steepness at which crest foam starts: the summed steepest-face slopes of the waves near their crests.</summary>
		public double CrestThreshold;

		/// <summary>Size (m) of one repeat of the foam texture, which holds 5 of the foam's largest holes across.</summary>
		public double TileMetres;

		/// <summary>Height (m) of the foam's relief, which tilts its normal.</summary>
		public double BumpMetres;

		/// <summary>Distance (m) from the camera beyond which there is no shore foam or lip.</summary>
		public double RadiusMetres;

		/// <summary>Distance (m) inside the radius over which the shore foam and lip fade out. A fade longer than the radius starts at the camera.</summary>
		public double FadeMetres;

		/// <summary>Vertical water depth (m) over which the water darkens towards the ground, like a rounded lip in shadow.</summary>
		public double LipHeightMetres;

		/// <summary>Darkness of the lip at the waterline: 0 for none, and 1 for black.</summary>
		public double LipStrength;

		/// <summary>Crest steepness above the threshold at which the crest foam reaches the crest strength.</summary>
		public double CrestRange;

		/// <summary>Time (s) for a fully covered trail behind a crest to fade away.</summary>
		public double TrailSeconds;

		/// <summary>Still-water depth (m) at which crest foam is gone. It fades in up to twice this depth, so shallow water near the shore shows only shore foam.</summary>
		public double CrestDepthMetres;

		/// <summary>Rough speed (m/s) at which the foam pattern wanders in place.</summary>
		public double ChurnMetresPerSecond;
	}

	/// <summary>The fixed inputs of <see cref="Ocean.Create"/>. World positions are metres, Z up, with the ocean's circular domain centred on the origin.</summary>
	public ref struct OceanCreateOptions
	{
		/// <summary>The wind-sea layout, fixed for the ocean's lifetime.</summary>
		public OceanWindSeaGrid WindSeaGrid;

		/// <summary>The initial wind sea. See <see cref="Ocean.SetWindSea"/>. Elements are required.</summary>
		public ReadOnlySpan<byte> Elements;
		public ReadOnlySpan<float> Table;
		public ReadOnlySpan<float> Winds;

		/// <summary>
		/// Ground heights at BathymetryWidth x BathymetryHeight nodes, row-major. Node (i, j) is at origin + cell * (i, j). Waves are corrected for
		/// the depth below the water level, and vertices over ground above the reach of the swash stay flat.
		/// </summary>
		public ReadOnlySpan<float> BathymetryHeights;
		public double BathymetryOriginX;
		public double BathymetryOriginY;
		public double BathymetryCellMetres;
		public int BathymetryWidth;
		public int BathymetryHeight;

		/// <summary>Lattice vertices per side (odd, in [3, 4097]). The triangle density is highest at the camera.</summary>
		public int VerticesPerSide;

		/// <summary>The surface stays inside this circle (m) about the origin.</summary>
		public double RadiusMetres;

		/// <summary>Half the vertical extent (m) of the culling bounds, about z = 0. It must cover every water level plus the highest wave.</summary>
		public double HalfHeightMetres;

		/// <summary>Height (m) of the domain floor. An opaque curtain hangs from the surface's edge down to this height.</summary>
		public double FloorHeightMetres;

		/// <summary>The detail map from <see cref="Ocean.BakeDetailMap"/>: DetailMapSize^2 RGBA8 pixels, row-major. Required.</summary>
		public ReadOnlySpan<uint> DetailMap;

		/// <summary>
		/// Optional replacements for the three forward pixel shaders (opaque, opaque reflection attributes, and alpha collect), for example to supply
		/// the ground height for the shore foam (see pr/game-systems/ocean/ocean_forward_ps.hlsli). Either all three or none are given.
		/// </summary>
		public ReadOnlySpan<byte> PixelOpaque;
		public ReadOnlySpan<byte> PixelOpaqueReflectionAttrs;
		public ReadOnlySpan<byte> PixelAlphaCollect;
	}

	/// <summary>
	/// An ocean surface generated on the GPU from vertex IDs. The lattice is centred on the camera, densest there, and reaches the far side of the
	/// domain; waves are evaluated only for wet vertices near the camera. Use on the render owner thread while the View3d context is alive.
	/// </summary>
	public sealed class Ocean : IDisposable
	{
		/// <summary>The size of one wave element in the shared physics water-field layout.</summary>
		public const int ElementSizeInBytes = 64;

		/// <summary>The most wave elements an ocean supports.</summary>
		public const int MaximumElementCount = 40;

		/// <summary>Texels per side of the detail map.</summary>
		public const int DetailMapSize = 512;

		/// <summary>The number of shore source parameters. See <see cref="SetShoreSource"/>.</summary>
		public const int ShoreParamCount = 12;

		/// <summary>Adopt a successfully created native ocean laid out by 'grid'.</summary>
		private Ocean(IntPtr handle, OceanWindSeaGrid grid)
		{
			Handle = handle;
			Grid = grid;
		}

		/// <summary>The native handle.</summary>
		public IntPtr Handle { get; private set; }

		/// <summary>The wind-sea layout, fixed for the ocean's lifetime.</summary>
		public OceanWindSeaGrid Grid { get; }

		/// <summary>
		/// Create an ocean and its View3D object. The caller owns both, and must exclude the object from hit tests, which it does not support, and
		/// from scene bounds, because it follows the camera. Disposing the ocean leaves the object valid but no longer updatable.
		/// </summary>
		public static unsafe Ocean Create(string name, OceanCreateOptions options, out Object model)
		{
			// The native side validates values; check what only the managed side knows.
			if (string.IsNullOrEmpty(name))
				throw new ArgumentException("An ocean name is required.", nameof(name));
			CheckWindSea(options.WindSeaGrid, options.Elements, options.Table, options.Winds);
			if (options.BathymetryHeights.Length != (long)options.BathymetryWidth * options.BathymetryHeight)
				throw new ArgumentException("The bathymetry heights do not match its dimensions.", nameof(options));
			if (options.DetailMap.Length != DetailMapSize * DetailMapSize)
				throw new ArgumentException("The detail map must hold DetailMapSize^2 pixels.", nameof(options));

			fixed (byte* element_data = options.Elements)
			fixed (float* table_data = options.Table)
			fixed (float* wind_data = options.Winds)
			fixed (float* heights = options.BathymetryHeights)
			fixed (uint* detail = options.DetailMap)
			fixed (byte* ps0 = options.PixelOpaque)
			fixed (byte* ps1 = options.PixelOpaqueReflectionAttrs)
			fixed (byte* ps2 = options.PixelAlphaCollect)
			{
				var native = new NativeOceanOptions
				{
					m_wind_sea_grid = options.WindSeaGrid,
					m_wind_sea = new NativeOceanWindSea { m_elements = element_data, m_table = table_data, m_winds = wind_data, m_element_count = options.Elements.Length / ElementSizeInBytes },
					m_bathymetry_heights = heights,
					m_bathymetry_origin_x_m = options.BathymetryOriginX,
					m_bathymetry_origin_y_m = options.BathymetryOriginY,
					m_bathymetry_cell_m = options.BathymetryCellMetres,
					m_bathymetry_width = options.BathymetryWidth,
					m_bathymetry_height = options.BathymetryHeight,
					m_vertices_per_side = options.VerticesPerSide,
					m_radius_m = options.RadiusMetres,
					m_half_height_m = options.HalfHeightMetres,
					m_floor_height_m = options.FloorHeightMetres,
					m_detail_map = detail,
					m_ps0 = new NativeShaderByteCode { m_bytecode = ps0, m_size = (nuint)options.PixelOpaque.Length },
					m_ps1 = new NativeShaderByteCode { m_bytecode = ps1, m_size = (nuint)options.PixelOpaqueReflectionAttrs.Length },
					m_ps2 = new NativeShaderByteCode { m_bytecode = ps2, m_size = (nuint)options.PixelAlphaCollect.Length },
				};
				var handle = View3D_OceanCreate(ref native, name, out var obj);
				if (handle == IntPtr.Zero)
					throw new InvalidOperationException("Failed to create the ocean.");

				model = new Object(obj, true);
				return new Ocean(handle, options.WindSeaGrid);
			}
		}

		/// <summary>
		/// Bake the detail map of tileable ripple slopes and foam: DetailMapSize^2 RGBA8 pixels, row-major. This is CPU work that needs no renderer,
		/// and it takes seconds, so it is meant for asset cooking.
		/// </summary>
		public static unsafe uint[] BakeDetailMap()
		{
			// The native side fills the whole array.
			var pixels = new uint[DetailMapSize * DetailMapSize];
			fixed (uint* data = pixels)
			{
				if (!View3D_OceanDetailMapBake(data, pixels.Length))
					throw new InvalidOperationException("Failed to bake the ocean detail map.");
			}
			return pixels;
		}

		/// <summary>
		/// Build the triangle list shared by oceans with 'vertices_per_side' vertices per side, so <see cref="Create"/> does not build it. The list is
		/// kept for the life of the process. Any thread may call this; it does not need a renderer.
		/// </summary>
		public static void PrepareLattice(int vertices_per_side)
		{
			// The native side validates the vertex count.
			if (!View3D_OceanLatticePrepare(vertices_per_side))
				throw new ArgumentException("The ocean lattice vertex count was rejected.", nameof(vertices_per_side));
		}

		/// <summary>Set the frame state of 'model', which must have been created with this ocean.</summary>
		public void Update(Object model, OceanFrame frame)
		{
			// The native side validates the frame before it reaches the shader.
			if (!View3D_OceanUpdate(Live, ModelHandle(model), ref frame))
				throw new ArgumentException("The ocean frame was rejected.", nameof(frame));
		}

		/// <summary>
		/// Replace the wave elements, table, and winds of 'model' (created with this ocean), laid out by <see cref="Grid"/>. The drawn surface
		/// and <see cref="HeightAt"/> use them straight away. 'elements' are 64-byte Gerstner waves in the physics water-field layout; wind-sea
		/// elements take their amplitude and crest sharpness from 'table' (four floats per row, see WindSeaTableRow) and the local wind in 'winds'.
		/// </summary>
		public unsafe void SetWindSea(Object model, ReadOnlySpan<byte> elements, ReadOnlySpan<float> table, ReadOnlySpan<float> winds)
		{
			// The native side validates the values.
			CheckWindSea(Grid, elements, table, winds);
			fixed (byte* element_data = elements)
			fixed (float* table_data = table)
			fixed (float* wind_data = winds)
			{
				var wind_sea = new NativeOceanWindSea { m_elements = element_data, m_table = table_data, m_winds = wind_data, m_element_count = elements.Length / ElementSizeInBytes };
				SetWindSea(model, ref wind_sea);
			}
		}

		/// <summary>Replace only the winds of 'model' (created with this ocean), keeping its elements and table. See <see cref="SetWindSea"/>.</summary>
		public unsafe void SetWinds(Object model, ReadOnlySpan<float> winds)
		{
			// Null elements keep the current elements and table.
			if (winds.Length != Grid.WindsLength)
				throw new ArgumentException("One (x, y) wind is required for each level of each wind-sea node.", nameof(winds));

			fixed (float* wind_data = winds)
			{
				var wind_sea = new NativeOceanWindSea { m_winds = wind_data };
				SetWindSea(model, ref wind_sea);
			}
		}

		/// <summary>
		/// Draw or hide the small-scale ripple normals, and the foam and the dark lip at the waterline. Hidden foam leaves no trails, and its trails
		/// start afresh when it is shown again. Both are shown on a new ocean. Takes effect from the next <see cref="Update"/>.
		/// </summary>
		public void SetEffects(bool ripples, bool foam)
		{
			// The native ocean adds the settings to each frame's constants.
			if (!View3D_OceanSetEffects(Live, ripples, foam))
				throw new InvalidOperationException("Failed to set the ocean effects.");
		}

		/// <summary>
		/// Draw the ripple strength of the local wind as colour instead of the shaded water: blue for calm, green for a fresh breeze (8 m/s), and red
		/// for twice its roughness or more. Off on a new ocean. Takes effect from the next <see cref="Update"/>.
		/// </summary>
		public void ShowRippleStrength(bool show)
		{
			// The native ocean adds the setting to each frame's constants.
			if (!View3D_OceanShowRippleStrength(Live, show))
				throw new InvalidOperationException("Failed to set the ocean ripple strength view.");
		}

		/// <summary>Set the foam drawn at the waterline and on breaking crests. Takes effect from the next <see cref="Update"/>.</summary>
		public void SetFoam(OceanFoam foam)
		{
			// The native ocean validates the values and keeps them for later frames.
			if (!View3D_OceanSetFoam(Live, ref foam))
				throw new ArgumentException("The ocean foam was rejected.", nameof(foam));
		}

		/// <summary>
		/// Bind the ground source that custom pixel shaders read for the shore foam and the waterline lip of 'model': a GPU buffer, bound at
		/// VIEW3D_PROCEDURAL_BUFFER1_REGISTER, and ShoreParamCount values copied into the 'shore_source' shader constants. The ocean shares the
		/// buffer, so the caller keeps its contents valid while frames that use it are in flight. A zero 'buffer' removes the source; the params
		/// are then ignored. Oceans start with no source. Takes effect from the next <see cref="Update"/>.
		/// </summary>
		public unsafe void SetShoreSource(Object model, IntPtr buffer, ReadOnlySpan<uint> shore_params)
		{
			// Params are required with a buffer.
			if (buffer != IntPtr.Zero && shore_params.Length != ShoreParamCount)
				throw new ArgumentException("ShoreParamCount shore parameters are required with a buffer.", nameof(shore_params));

			fixed (uint* data = shore_params)
			{
				if (!View3D_OceanShoreSourceSet(Live, ModelHandle(model), buffer, buffer != IntPtr.Zero ? data : null))
					throw new ArgumentException("The ocean shore source was rejected.");
			}
		}

		/// <summary>
		/// Return the height (m) of the drawn surface at the finite world position ('x', 'y') in the last frame passed to <see cref="Update"/>,
		/// which must have been called. Positions beyond the domain radius take the height at the domain edge.
		/// </summary>
		public double HeightAt(double x, double y)
		{
			// The native side evaluates the waves exactly as the vertex shader draws them.
			if (!View3D_OceanHeight(Live, x, y, out var height))
				throw new InvalidOperationException("Failed to sample the ocean height.");

			return height;
		}

		/// <summary>Release the native ocean. Its View3D object remains valid but can no longer be updated. Repeated disposal is harmless.</summary>
		public void Dispose()
		{
			View3D_OceanDestroy(Handle);
			Handle = IntPtr.Zero;
		}

		/// <summary>Pass one wind-sea update to the native ocean.</summary>
		private void SetWindSea(Object model, ref NativeOceanWindSea wind_sea)
		{
			// The native side validates the values against the ocean's grid.
			if (!View3D_OceanSetWindSea(Live, ModelHandle(model), ref wind_sea))
				throw new ArgumentException("The ocean wind sea was rejected.");
		}

		/// <summary>Check that the spans match the element layout and 'grid'. The native side checks the values.</summary>
		private static void CheckWindSea(OceanWindSeaGrid grid, ReadOnlySpan<byte> elements, ReadOnlySpan<float> table, ReadOnlySpan<float> winds)
		{
			// Only the managed side knows the span lengths.
			if (elements.Length % ElementSizeInBytes != 0)
				throw new ArgumentException("Wave elements must use the 64-byte water-field layout.", nameof(elements));
			if (table.Length != grid.TableLength)
				throw new ArgumentException("One four-float row is required for each speed of each wind-sea band.", nameof(table));
			if (winds.Length != grid.WindsLength)
				throw new ArgumentException("One (x, y) wind is required for each level of each wind-sea node.", nameof(winds));
		}

		/// <summary>The live object handle of 'model'.</summary>
		private static IntPtr ModelHandle(Object model)
		{
			// The object must be the live object created with the ocean.
			if (model == null)
				throw new ArgumentNullException(nameof(model));
			if (model.Handle == IntPtr.Zero)
				throw new ObjectDisposedException(nameof(model));

			return model.Handle;
		}

		/// <summary>The handle, or throw if disposed.</summary>
		private IntPtr Live
		{
			get
			{
				if (Handle == IntPtr.Zero)
					throw new ObjectDisposedException(nameof(Ocean));

				return Handle;
			}
		}
	}

	/// <summary>Mirrors 'view3d::ShaderByteCode'.</summary>
	[StructLayout(LayoutKind.Sequential)]
	internal unsafe struct NativeShaderByteCode
	{
		public byte* m_bytecode;
		public nuint m_size;
	}

	/// <summary>Mirrors 'view3d::OceanWindSea'.</summary>
	[StructLayout(LayoutKind.Sequential)]
	private unsafe struct NativeOceanWindSea
	{
		public byte* m_elements;
		public float* m_table;
		public float* m_winds;
		public int m_element_count;
		public int m_reserved;
	}

	/// <summary>Mirrors 'view3d::OceanOptions'.</summary>
	[StructLayout(LayoutKind.Sequential)]
	private unsafe struct NativeOceanOptions
	{
		public OceanWindSeaGrid m_wind_sea_grid;
		public NativeOceanWindSea m_wind_sea;
		public float* m_bathymetry_heights;
		public double m_bathymetry_origin_x_m;
		public double m_bathymetry_origin_y_m;
		public double m_bathymetry_cell_m;
		public int m_bathymetry_width;
		public int m_bathymetry_height;
		public int m_vertices_per_side;
		public double m_radius_m;
		public double m_half_height_m;
		public double m_floor_height_m;
		public uint* m_detail_map;
		public NativeShaderByteCode m_ps0;
		public NativeShaderByteCode m_ps1;
		public NativeShaderByteCode m_ps2;
	}

	[DllImport(Dll, CharSet = CharSet.Ansi)]
	private static extern IntPtr View3D_OceanCreate(ref NativeOceanOptions options, [MarshalAs(UnmanagedType.LPStr)] string name, out IntPtr out_object);

	[DllImport(Dll)]
	private static extern void View3D_OceanDestroy(IntPtr ocean);

	[DllImport(Dll)]
	[return: MarshalAs(UnmanagedType.Bool)]
	private static extern bool View3D_OceanLatticePrepare(int vertices_per_side);

	[DllImport(Dll)]
	[return: MarshalAs(UnmanagedType.Bool)]
	private static unsafe extern bool View3D_OceanDetailMapBake(uint* out_pixels, int pixel_count);

	[DllImport(Dll)]
	[return: MarshalAs(UnmanagedType.Bool)]
	private static extern bool View3D_OceanUpdate(IntPtr ocean, IntPtr obj, ref OceanFrame frame);

	[DllImport(Dll)]
	[return: MarshalAs(UnmanagedType.Bool)]
	private static extern bool View3D_OceanSetWindSea(IntPtr ocean, IntPtr obj, ref NativeOceanWindSea wind_sea);

	[DllImport(Dll)]
	[return: MarshalAs(UnmanagedType.Bool)]
	private static extern bool View3D_OceanSetEffects(IntPtr ocean, [MarshalAs(UnmanagedType.Bool)] bool ripples, [MarshalAs(UnmanagedType.Bool)] bool foam);

	[DllImport(Dll)]
	[return: MarshalAs(UnmanagedType.Bool)]
	private static extern bool View3D_OceanShowRippleStrength(IntPtr ocean, [MarshalAs(UnmanagedType.Bool)] bool show);

	[DllImport(Dll)]
	[return: MarshalAs(UnmanagedType.Bool)]
	private static extern bool View3D_OceanSetFoam(IntPtr ocean, ref OceanFoam foam);

	[DllImport(Dll)]
	[return: MarshalAs(UnmanagedType.Bool)]
	private static unsafe extern bool View3D_OceanShoreSourceSet(IntPtr ocean, IntPtr obj, IntPtr buffer, uint* shore_params);

	[DllImport(Dll)]
	[return: MarshalAs(UnmanagedType.Bool)]
	private static extern bool View3D_OceanHeight(IntPtr ocean, double x_m, double y_m, out double out_height_m);
}

#if PR_UNITTESTS
/// <summary>Validate the managed ocean layouts against the native ABI independently of native DLL availability.</summary>
[TestFixture]
public class OceanTests
{
	/// <summary>The structs match the native layouts.</summary>
	[Test]
	public void Layouts()
	{
		Assert.Equal(56, Marshal.SizeOf<View3d.OceanWindSeaGrid>());
		Assert.Equal(120, Marshal.SizeOf<View3d.OceanFrame>());
		Assert.Equal(112, Marshal.SizeOf<View3d.OceanFoam>());
	}
}
#endif
