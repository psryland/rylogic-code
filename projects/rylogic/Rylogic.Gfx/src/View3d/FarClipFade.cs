using System;
using System.Runtime.InteropServices;
#if PR_UNITTESTS
using Rylogic.UnitTests;
#endif

namespace Rylogic.Gfx;

public sealed partial class View3d
{
	/// <summary>Opt-in world opacity range, as fractions of camera-forward far depth. UI and sky are not faded.</summary>
	[StructLayout(LayoutKind.Sequential)]
	public struct FarClipFadeProps
	{
		private int m_enabled;
		private float m_start_fraction;
		private float m_end_fraction;

		/// <summary>Create disabled settings with the native default range.</summary>
		public FarClipFadeProps()
			: this(false)
		{
		}

		/// <summary>Create validated settings; the default range ends before the hardware far plane.</summary>
		public FarClipFadeProps(bool enabled, float start_fraction = 0.9f, float end_fraction = 0.99f)
		{
			m_enabled = enabled ? 1 : 0;
			m_start_fraction = start_fraction;
			m_end_fraction = end_fraction;
			Validate();
		}

		/// <summary>Whether world geometry fades near the far plane.</summary>
		public bool Enabled
		{
			get
			{
				return m_enabled != 0;
			}
		}

		/// <summary>Camera-forward depth divided by far depth where opacity starts decreasing.</summary>
		public float StartFraction
		{
			get
			{
				return m_start_fraction;
			}
		}

		/// <summary>Camera-forward depth divided by far depth where opacity reaches zero.</summary>
		public float EndFraction
		{
			get
			{
				return m_end_fraction;
			}
		}

		/// <summary>Disabled settings with a valid range, matching a newly created native scene.</summary>
		public static FarClipFadeProps Default()
		{
			return new FarClipFadeProps(false);
		}

		/// <summary>Reject non-finite or unordered ranges, even while the effect is disabled.</summary>
		public void Validate()
		{
			if (float.IsNaN(m_start_fraction) || float.IsInfinity(m_start_fraction) ||
				float.IsNaN(m_end_fraction) || float.IsInfinity(m_end_fraction) ||
				m_start_fraction < 0 || m_start_fraction >= m_end_fraction || m_end_fraction >= 1)
				throw new ArgumentOutOfRangeException(nameof(StartFraction), "Far clip fade requires finite 0 <= start < end < 1.");
		}
	}

	[DllImport(Dll)]
	private static extern FarClipFadeProps View3D_FarClipFadePropertiesGet(IntPtr window);

	[DllImport(Dll)]
	[return: MarshalAs(UnmanagedType.Bool)]
	private static extern bool View3D_FarClipFadePropertiesSet(IntPtr window, ref FarClipFadeProps props);
}

#if PR_UNITTESTS
/// <summary>Validate the managed option contract independently of native DLL availability.</summary>
[TestFixture]
public class FarClipFadeTests
{
	/// <summary>New views remain unchanged until explicitly enabled, with a stable 12-byte ABI.</summary>
	[Test]
	public void DefaultAndLayout()
	{
		var props = View3d.FarClipFadeProps.Default();
		Assert.False(props.Enabled);
		Assert.Equal(0.9f, props.StartFraction);
		Assert.Equal(0.99f, props.EndFraction);
		Assert.Equal(12, Marshal.SizeOf<View3d.FarClipFadeProps>());
		Assert.Equal(4, Marshal.OffsetOf<View3d.FarClipFadeProps>("m_start_fraction").ToInt32());
		Assert.Equal(8, Marshal.OffsetOf<View3d.FarClipFadeProps>("m_end_fraction").ToInt32());
		Assert.Equal(props.StartFraction, new View3d.FarClipFadeProps().StartFraction);
		Assert.Equal(props.EndFraction, new View3d.FarClipFadeProps().EndFraction);
		Assert.True(new View3d.FarClipFadeProps(true).Enabled);
	}

	/// <summary>Invalid ranges fail before reaching the native setter.</summary>
	[Test]
	public void InvalidRanges()
	{
		Assert.Throws<ArgumentOutOfRangeException>(() => new View3d.FarClipFadeProps(true, -1, 0.99f));
		Assert.Throws<ArgumentOutOfRangeException>(() => new View3d.FarClipFadeProps(true, 0.9f, 0.9f));
		Assert.Throws<ArgumentOutOfRangeException>(() => new View3d.FarClipFadeProps(false, 0.9f, 1));
		Assert.Throws<ArgumentOutOfRangeException>(() => new View3d.FarClipFadeProps(true, float.NaN, 0.99f));
		Assert.Throws<ArgumentOutOfRangeException>(() => new View3d.FarClipFadeProps(true, 0.9f, float.PositiveInfinity));
	}
}
#endif
