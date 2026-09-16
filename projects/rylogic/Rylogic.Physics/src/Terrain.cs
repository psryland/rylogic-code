using System.Runtime.InteropServices;

namespace Rylogic.Physics;

/// <summary>One explicit terrain frequency band; roundness and weight gain apply to mountains.</summary>
[StructLayout(LayoutKind.Sequential)]
public readonly struct TerrainBand
{
	public readonly double m_amplitude, m_wavelength, m_lacunarity, m_persistence, m_roundness, m_weight_gain;
	public readonly int m_octaves;
	private readonly int m_reserved;

	/// <summary>Specify a frequency band in metres without relying on native configuration defaults.</summary>
	public TerrainBand(double amplitude, double wavelength, int octaves, double lacunarity, double persistence, double roundness = 0.2, double weight_gain = 1)
	{
		m_amplitude = amplitude;
		m_wavelength = wavelength;
		m_octaves = octaves;
		m_lacunarity = lacunarity;
		m_persistence = persistence;
		m_roundness = roundness;
		m_weight_gain = weight_gain;
		m_reserved = 0;
	}
}

/// <summary>Complete immutable baseline terrain settings copied into an engine; zero spacing selects its shared sampling default.</summary>
[StructLayout(LayoutKind.Sequential)]
public readonly struct TerrainConfiguration
{
	private readonly NativeHeader m_header;
	public readonly uint m_seed;
	public readonly int m_material_id;
	public readonly double m_supported_coordinate, m_sea_level_bias, m_uplift_height, m_mountain_base;
	public readonly TerrainBand m_regional_base, m_region_selector, m_region_uplift, m_domain_warp, m_plains, m_hills, m_mountains;
	public readonly float m_surface_spacing;
	private readonly uint m_reserved;

	/// <summary>Specify every baseline field so replacing a package cannot silently change a saved landscape.</summary>
	public TerrainConfiguration(uint seed, int material_id, double supported_coordinate, double sea_level_bias, double uplift_height, double mountain_base,
		TerrainBand regional_base, TerrainBand region_selector, TerrainBand region_uplift, TerrainBand domain_warp, TerrainBand plains, TerrainBand hills, TerrainBand mountains, float surface_spacing = 0)
	{
		m_header = NativeHeader.Create<TerrainConfiguration>();
		m_seed = seed;
		m_material_id = material_id;
		m_supported_coordinate = supported_coordinate;
		m_sea_level_bias = sea_level_bias;
		m_uplift_height = uplift_height;
		m_mountain_base = mountain_base;
		m_regional_base = regional_base;
		m_region_selector = region_selector;
		m_region_uplift = region_uplift;
		m_domain_warp = domain_warp;
		m_plains = plains;
		m_hills = hills;
		m_mountains = mountains;
		m_surface_spacing = surface_spacing;
		m_reserved = 0;
	}
}
