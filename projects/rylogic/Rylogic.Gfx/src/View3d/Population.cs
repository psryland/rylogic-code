using System;
using System.Runtime.InteropServices;
using Rylogic.Maths;
#if PR_UNITTESTS
using Rylogic.UnitTests;
#endif

namespace Rylogic.Gfx;

public sealed partial class View3d
{
	/// <summary>Flags of a population instance. Matches 'view3d::EPopulationInstanceFlags'.</summary>
	[Flags]
	public enum EPopulationInstanceFlags : uint
	{
		None = 0,

		/// <summary>Only enabled instances are drawn.</summary>
		Enabled = 1 << 0,
	}

	/// <summary>One placement of a population prototype. Matches 'view3d::PopulationInstance' (64 bytes).</summary>
	[StructLayout(LayoutKind.Sequential)]
	public struct PopulationInstance
	{
		/// <summary>Rows of the affine instance-to-world transform: world[i] = dot(row i, (p,1)).</summary>
		public v4 O2WRow0;
		public v4 O2WRow1;
		public v4 O2WRow2;

		/// <summary>Multiplies the vertex colour.</summary>
		public Colour32 Tint;

		/// <summary>See EPopulationInstanceFlags.</summary>
		public EPopulationInstanceFlags Flags;

		/// <summary>Caller payload index, interpreted only by caller shaders.</summary>
		public uint Payload;

		/// <summary>Must be zero.</summary>
		public uint Reserved;

		/// <summary>Create an instance record from an affine instance-to-world transform.</summary>
		public PopulationInstance(m4x4 o2w, Colour32 tint, EPopulationInstanceFlags flags = EPopulationInstanceFlags.Enabled, uint payload = 0)
		{
			// 'o2w' stores columns, so each row gathers one component of every column
			O2WRow0 = new v4(o2w.x.x, o2w.y.x, o2w.z.x, o2w.w.x);
			O2WRow1 = new v4(o2w.x.y, o2w.y.y, o2w.z.y, o2w.w.y);
			O2WRow2 = new v4(o2w.x.z, o2w.y.z, o2w.z.z, o2w.w.z);
			Tint = tint;
			Flags = flags;
			Payload = payload;
			Reserved = 0;
		}
	}

	/// <summary>One level of detail of a population prototype. Matches 'view3d::PopulationLod'.</summary>
	[StructLayout(LayoutKind.Sequential)]
	public struct PopulationLod
	{
		/// <summary>The object that supplies this LOD's model. Child objects are not used.</summary>
		public IntPtr Object;

		/// <summary>The minimum projected radius, in pixels, of the prototype bounding sphere at which this LOD is drawn.</summary>
		public float MinSize;

		/// <summary>The dithered transition band above 'MinSize', as a fraction of 'MinSize'. Zero is an abrupt switch.</summary>
		public float Transition;

		public PopulationLod(Object obj, float min_size, float transition = 0f)
		{
			Object = obj.Handle;
			MinSize = min_size;
			Transition = transition;
		}
	}

	/// <summary>
	/// The shared LOD models and draw distance of a population. The prototype keeps its own reference to the LOD models, so the source objects may be
	/// disposed afterwards. Use on the render owner thread while the View3d context is alive.
	/// </summary>
	public sealed class PopulationPrototype : IDisposable
	{
		/// <summary>
		/// Create a prototype from LODs ordered from most to least detailed. LOD thresholds must strictly decrease, and transition bands must not overlap.
		/// 'draw_distance' is the maximum camera distance of a drawn instance's bounding sphere; zero is unlimited.
		/// </summary>
		public PopulationPrototype(PopulationLod[] lods, float draw_distance = 0f)
		{
			Handle = View3D_PopulationPrototypeCreate(lods, lods.Length, draw_distance);
			if (Handle == IntPtr.Zero)
				throw new InvalidOperationException("Failed to create the population prototype.");
		}

		/// <summary>The native handle.</summary>
		public IntPtr Handle { get; private set; }

		/// <summary>Release this reference; populations that use the prototype keep their own. Repeated disposal is harmless.</summary>
		public void Dispose()
		{
			View3D_PopulationPrototypeRelease(Handle);
			Handle = IntPtr.Zero;
		}
	}

	/// <summary>
	/// Many placements of one prototype, drawn with GPU instancing and GPU culling. Instances live in a GPU buffer and are grouped into caller-owned blocks.
	/// Updates are copied during the call and shown from the next frame. Use on the render owner thread while the View3d context is alive.
	/// </summary>
	public sealed class Population : IDisposable
	{
		/// <summary>
		/// Create a population of 'prototype' with fixed capacities. 'payload_size' is the size in bytes of an optional caller payload buffer for custom
		/// shaders; zero means no payload.
		/// </summary>
		public Population(PopulationPrototype prototype, int instance_capacity, int block_capacity, int payload_size = 0)
		{
			Handle = View3D_PopulationCreate(prototype.Handle, instance_capacity, block_capacity, payload_size);
			if (Handle == IntPtr.Zero)
				throw new InvalidOperationException("Failed to create the population.");
		}

		/// <summary>The native handle.</summary>
		public IntPtr Handle { get; private set; }

		/// <summary>
		/// Define block 'index' as the instance slots [first, first + capacity), of which the first 'live_count' are drawn. 'bbox' is a world-space bound
		/// that must contain the bounding sphere of every live instance.
		/// </summary>
		public void BlockSet(int index, BBox bbox, int first, int capacity, int live_count)
		{
			View3D_PopulationBlockSet(Live, index, bbox, first, capacity, live_count);
		}

		/// <summary>Stop drawing block 'index'.</summary>
		public void BlockRemove(int index)
		{
			View3D_PopulationBlockRemove(Live, index);
		}

		/// <summary>Copy 'instances' into the slots starting at 'first'.</summary>
		public void InstancesUpdate(int first, ReadOnlySpan<PopulationInstance> instances)
		{
			unsafe
			{
				fixed (PopulationInstance* ptr = instances)
					View3D_PopulationInstancesUpdate(Live, first, ptr, instances.Length);
			}
		}

		/// <summary>Copy 'data' into the payload buffer at byte offset 'ofs'.</summary>
		public void PayloadUpdate(long ofs, ReadOnlySpan<byte> data)
		{
			unsafe
			{
				fixed (byte* ptr = data)
					View3D_PopulationPayloadUpdate(Live, ofs, ptr, data.Length);
			}
		}

		/// <summary>Release this reference; windows that show the population keep their own. Repeated disposal is harmless.</summary>
		public void Dispose()
		{
			View3D_PopulationRelease(Handle);
			Handle = IntPtr.Zero;
		}

		/// <summary>The handle, or throw if disposed.</summary>
		private IntPtr Live
		{
			get
			{
				if (Handle == IntPtr.Zero)
					throw new ObjectDisposedException(nameof(Population));

				return Handle;
			}
		}
	}

	[DllImport(Dll)]
	private static extern IntPtr View3D_PopulationPrototypeCreate([In] PopulationLod[] lods, int lod_count, float draw_distance);

	[DllImport(Dll)]
	private static extern void View3D_PopulationPrototypeRelease(IntPtr prototype);

	[DllImport(Dll)]
	private static extern IntPtr View3D_PopulationCreate(IntPtr prototype, int instance_capacity, int block_capacity, int payload_size);

	[DllImport(Dll)]
	private static extern void View3D_PopulationRelease(IntPtr population);

	[DllImport(Dll)]
	private static extern void View3D_PopulationBlockSet(IntPtr population, int index, BBox bbox, int first, int capacity, int live_count);

	[DllImport(Dll)]
	private static extern void View3D_PopulationBlockRemove(IntPtr population, int index);

	[DllImport(Dll)]
	private static extern unsafe void View3D_PopulationInstancesUpdate(IntPtr population, int first, PopulationInstance* instances, int count);

	[DllImport(Dll)]
	private static extern unsafe void View3D_PopulationPayloadUpdate(IntPtr population, long ofs, void* data, long size);

	[DllImport(Dll)]
	private static extern void View3D_WindowAddPopulation(IntPtr window, IntPtr population);

	[DllImport(Dll)]
	private static extern void View3D_WindowRemovePopulation(IntPtr window, IntPtr population);
}

#if PR_UNITTESTS
/// <summary>Validate the managed population contracts independently of native DLL availability.</summary>
[TestFixture]
public class PopulationTests
{
	/// <summary>The instance and LOD records match the native layouts, and instance rows are the rows of the transform.</summary>
	[Test]
	public void InstanceLayout()
	{
		// Layouts must match 'view3d::PopulationInstance' and 'view3d::PopulationLod'
		Assert.Equal(64, Marshal.SizeOf<View3d.PopulationInstance>());
		Assert.Equal(48, (int)Marshal.OffsetOf<View3d.PopulationInstance>(nameof(View3d.PopulationInstance.Tint)));
		Assert.Equal(16, Marshal.SizeOf<View3d.PopulationLod>());

		// A translation lands in the 'w' component of each row
		var instance = new View3d.PopulationInstance(m4x4.Translation(1, 2, 3), Colour32.Red);
		Assert.Equal(new v4(1, 0, 0, 1), instance.O2WRow0);
		Assert.Equal(new v4(0, 1, 0, 2), instance.O2WRow1);
		Assert.Equal(new v4(0, 0, 1, 3), instance.O2WRow2);
		Assert.Equal(View3d.EPopulationInstanceFlags.Enabled, instance.Flags);
	}
}
#endif
