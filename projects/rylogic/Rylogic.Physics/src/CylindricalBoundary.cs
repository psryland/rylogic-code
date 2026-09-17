using System.Runtime.InteropServices;

namespace Rylogic.Physics;

/// <summary>Immutable inward-facing cylinder, unlimited in height. Discrete contacts use current geometry without speed or timestep restrictions.</summary>
[StructLayout(LayoutKind.Sequential)]
public readonly struct CylindricalBoundaryConfiguration
{
	private readonly NativeHeader m_header;
	public readonly double m_centre_x, m_centre_y, m_radius;
	public readonly int m_material_id;
	public readonly float m_surface_spacing;

	/// <summary>Specify cylinder geometry and surface-sample spacing in metres.</summary>
	public CylindricalBoundaryConfiguration(double centre_x, double centre_y, double radius, int material_id = 0, float surface_spacing = 0.05f)
	{
		m_header = NativeHeader.Create<CylindricalBoundaryConfiguration>();
		m_centre_x = centre_x;
		m_centre_y = centre_y;
		m_radius = radius;
		m_material_id = material_id;
		m_surface_spacing = surface_spacing;
	}
}
