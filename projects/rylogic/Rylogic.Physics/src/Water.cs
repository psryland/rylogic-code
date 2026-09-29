using System.Runtime.InteropServices;

namespace Rylogic.Physics;

/// <summary>
/// A flat water surface at a world-space height (+Z up) that applies buoyancy and drag to every dynamic body.
/// Bodies that cannot reach the surface during a step cost no water work. Density is kg/m³ and drag rates are 1/s at full submersion.
/// </summary>
[StructLayout(LayoutKind.Sequential)]
public readonly struct WaterConfiguration
{
	private readonly NativeHeader m_header;
	public readonly double m_level;
	public readonly float m_density;
	public readonly float m_linear_drag_rate;
	public readonly float m_quadratic_drag_coefficient;
	public readonly float m_angular_drag_rate;

	/// <summary>Specify the water level in metres and the water's density and drag.</summary>
	public WaterConfiguration(double level, float density = 1000.0f, float linear_drag_rate = 0.5f, float quadratic_drag_coefficient = 0.5f, float angular_drag_rate = 0.5f)
	{
		m_header = NativeHeader.Create<WaterConfiguration>();
		m_level = level;
		m_density = density;
		m_linear_drag_rate = linear_drag_rate;
		m_quadratic_drag_coefficient = quadratic_drag_coefficient;
		m_angular_drag_rate = angular_drag_rate;
	}
}
