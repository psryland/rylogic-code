using System.Runtime.InteropServices;

namespace Rylogic.Physics;

/// <summary>Two-component double-precision vector, matching Vector2d in physics-dll.h. Used for world coordinates that need more precision than float.</summary>
[StructLayout(LayoutKind.Sequential)]
public readonly struct Vector2d
{
	public readonly double x;
	public readonly double y;

	/// <summary>Specify both components.</summary>
	public Vector2d(double x, double y)
	{
		this.x = x;
		this.y = y;
	}
}

/// <summary>Four-component integer vector, matching Vector4i in physics-dll.h.</summary>
[StructLayout(LayoutKind.Sequential)]
public struct Vector4i
{
	public int x;
	public int y;
	public int z;
	public int w;

	/// <summary>Specify all four components.</summary>
	public Vector4i(int x, int y, int z, int w)
	{
		this.x = x;
		this.y = y;
		this.z = z;
		this.w = w;
	}
}
