using System.Runtime.InteropServices;

namespace Rylogic.Physics;

/// <summary>Immutable inward-facing cylinder, unlimited in height. Discrete contacts use current geometry without speed or timestep restrictions.</summary>
[StructLayout(LayoutKind.Sequential)]
public readonly struct CylindricalBoundaryConfiguration
{
	private readonly NativeHeader m_header;
	public readonly Vector2d m_centre;
	public readonly double m_radius;
	public readonly int m_material_id;
	public readonly float m_surface_spacing;

	/// <summary>Specify cylinder geometry and surface-sample spacing in metres.</summary>
	public CylindricalBoundaryConfiguration(Vector2d centre, double radius, int material_id = 0, float surface_spacing = 0.05f)
	{
		m_header = NativeHeader.Create<CylindricalBoundaryConfiguration>();
		m_centre = centre;
		m_radius = radius;
		m_material_id = material_id;
		m_surface_spacing = surface_spacing;
	}
}
