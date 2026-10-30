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
	/// Diagnostics of a population. Matches 'view3d::PopulationStats'. The capacity, block, memory, and upload counters are current. The classification
	/// counters are the latest classification of each view that the GPU has completed, summed over 'Views' views, so they lag by the frames in flight.
	/// </summary>
	[StructLayout(LayoutKind.Sequential)]
	public unsafe struct PopulationStats
	{
		/// <summary>The number of instance slots.</summary>
		public long InstanceCapacity;

		/// <summary>The number of block slots.</summary>
		public long BlockCapacity;

		/// <summary>Blocks that are set and not removed.</summary>
		public long ActiveBlocks;

		/// <summary>The sum of the active blocks' capacities: the most instances that can be live.</summary>
		public long AdmittedInstances;

		/// <summary>The size of the instance, live count, and payload buffers in bytes.</summary>
		public long ResidentBytes;

		/// <summary>The size of all per-view classification buffers in bytes.</summary>
		public long ScratchBytes;

		/// <summary>The largest value 'ScratchBytes' has had.</summary>
		public long ScratchHighWater;

		/// <summary>The total bytes copied to the GPU by instance, block, and payload updates.</summary>
		public long UploadBytes;

		/// <summary>The total number of producer dispatches recorded.</summary>
		public long ProducerDispatches;

		/// <summary>The number of views with completed classification counters.</summary>
		public long Views;

		/// <summary>Active blocks skipped by the block test (frustum or draw distance).</summary>
		public long BlocksRejected;

		/// <summary>Visible blocks not drawn because a view exceeded 65535 visible blocks.</summary>
		public long BlocksOverflow;

		/// <summary>Live instance slots tested by the GPU.</summary>
		public long InstancesTested;

		/// <summary>Visible list entries written. An instance in a LOD transition writes two.</summary>
		public long EntriesEmitted;

		/// <summary>Visible list entries of each LOD. Use 'EntriesForLod'.</summary>
		public fixed long EntriesPerLod[8];

		/// <summary>Indirect draws: one per (LOD, nugget) pair.</summary>
		public long IndirectDraws;

		/// <summary>Visible blocks whose live count exceeded their capacity. The live count is clamped.</summary>
		public long LiveCountViolations;

		/// <summary>The visible list entries of LOD 'lod'.</summary>
		public readonly long EntriesForLod(int lod)
		{
			if (lod < 0 || lod >= 8)
				throw new ArgumentOutOfRangeException(nameof(lod));

			return EntriesPerLod[lod];
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
		/// shaders; zero means no payload, and it must be a multiple of 4.
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

		/// <summary>
		/// Generate population content on the GPU. Queues one dispatch of the compiled compute shader 'cs_bytecode', which sees a copy of 'constants'
		/// (1 to 65536 bytes) and writes instances, block live counts, and payload through the producer bindings in 'population.hlsli'. Dispatches and
		/// other updates run at the start of the next frame in the order they were queued. Block bounds still come from BlockSet.
		/// </summary>
		public unsafe void Produce<T>(ReadOnlySpan<byte> cs_bytecode, ReadOnlySpan<T> constants, int groups_x, int groups_y, int groups_z) where T : unmanaged
		{
			// The bytecode and constants are pinned only for the call; View3D copies them.
			fixed (byte* cs = cs_bytecode)
			fixed (T* cb = constants)
				View3D_PopulationProduce(Live, cs, checked((nuint)cs_bytecode.Length), cb, checked((nuint)(constants.Length * sizeof(T))), groups_x, groups_y, groups_z);
		}

		/// <summary>Read the diagnostics without waiting for the GPU. See PopulationStats.</summary>
		public PopulationStats Stats()
		{
			if (!View3D_PopulationStats(Live, out var stats))
				throw new InvalidOperationException("Failed to read the population diagnostics.");

			return stats;
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
	private static extern unsafe void View3D_PopulationProduce(IntPtr population, void* cs_bytecode, nuint cs_bytecode_size, void* constants, nuint constants_size, int groups_x, int groups_y, int groups_z);

	[DllImport(Dll)]
	private static extern bool View3D_PopulationStats(IntPtr population, out PopulationStats stats);

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
		Assert.Equal(24 * 8, Marshal.SizeOf<View3d.PopulationStats>());

		// A translation lands in the 'w' component of each row
		var instance = new View3d.PopulationInstance(m4x4.Translation(1, 2, 3), Colour32.Red);
		Assert.Equal(new v4(1, 0, 0, 1), instance.O2WRow0);
		Assert.Equal(new v4(0, 1, 0, 2), instance.O2WRow1);
		Assert.Equal(new v4(0, 0, 1, 3), instance.O2WRow2);
		Assert.Equal(View3d.EPopulationInstanceFlags.Enabled, instance.Flags);
	}
}
#endif
