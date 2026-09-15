using System;
using System.Runtime.InteropServices;
using Rylogic.Maths;

namespace Rylogic.Gfx;

public sealed partial class View3d
{
	/// <summary>Owns a Z-up GPU atmosphere. Create, update and dispose on the render owner while the View3d context is alive.</summary>
	public sealed class ProceduralSky : IDisposable
	{
		private readonly Object m_skybox;

		/// <summary>Create a visible atmosphere without textures. Direction points toward the sun; colour and intensity are linear and nonnegative.</summary>
		public ProceduralSky(string name, v4 sun_direction, v4 sun_colour, float sun_intensity, Guid? context_id = null)
		{
			var id = context_id ?? Guid.NewGuid();
			var handle = View3D_ObjectCreateProceduralSky(name, sun_direction, sun_colour, sun_intensity, ref id);
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

		/// <summary>Change sun constants without rebuilding resources. Invalid parameters leave the previous sky unchanged.</summary>
		public void Update(v4 sun_direction, v4 sun_colour, float sun_intensity)
		{
			if (m_skybox.Handle == IntPtr.Zero)
				throw new ObjectDisposedException(nameof(ProceduralSky));

			if (!View3D_ObjectUpdateProceduralSky(m_skybox.Handle, sun_direction, sun_colour, sun_intensity))
				throw new ArgumentException("The procedural sky parameters were rejected.");
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

	[DllImport(Dll, CharSet = CharSet.Ansi)]
	private static extern IntPtr View3D_ObjectCreateProceduralSky([MarshalAs(UnmanagedType.LPStr)] string name, v4 sun_direction, v4 sun_colour, float sun_intensity, ref Guid context_id);

	[DllImport(Dll)]
	[return: MarshalAs(UnmanagedType.Bool)]
	private static extern bool View3D_ObjectUpdateProceduralSky(IntPtr obj, v4 sun_direction, v4 sun_colour, float sun_intensity);

	// Configure background blending without transferring managed ownership of the source.
	[DllImport(Dll)]
	[return: MarshalAs(UnmanagedType.Bool)]
	private static extern bool View3D_ObjectBlendProceduralSky(IntPtr obj, IntPtr background, float weight, ref m4x4 world_to_sky, ref m4x4 world_to_background);
}
