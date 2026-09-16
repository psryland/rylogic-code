using System.Runtime.InteropServices;

namespace Rylogic.Physics;

/// <summary>Immutable inward-facing cylinder, unlimited in height. Distances are metres; substep surface motion is bounded in XY only.</summary>
[StructLayout(LayoutKind.Sequential)]
public readonly struct CylindricalBoundaryConfiguration
{
	private readonly NativeHeader m_header;
	public readonly double m_centre_x, m_centre_y, m_radius;
	public readonly int m_material_id;
	public readonly float m_surface_spacing, m_max_substep_motion, m_max_penetration;

	/// <summary>Specify geometry and the discrete collision envelope. Max penetration is a gross rejection cutoff, not a spawn/restore allowance.</summary>
	public CylindricalBoundaryConfiguration(double centre_x, double centre_y, double radius, int material_id = 0, float surface_spacing = 0.05f, float max_substep_motion = 0.1f, float max_penetration = 0.25f)
	{
		m_header = NativeHeader.Create<CylindricalBoundaryConfiguration>();
		m_centre_x = centre_x;
		m_centre_y = centre_y;
		m_radius = radius;
		m_material_id = material_id;
		m_surface_spacing = surface_spacing;
		m_max_substep_motion = max_substep_motion;
		m_max_penetration = max_penetration;
	}
}
