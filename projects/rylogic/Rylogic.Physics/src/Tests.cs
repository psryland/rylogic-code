#if PR_UNITTESTS
using System;
using System.Runtime.InteropServices;
using System.Threading;
using Rylogic.Maths;
using Rylogic.UnitTests;

namespace Rylogic.Physics;

/// <summary>Validates managed/native layouts, ownership, stepping, and restart behavior.</summary>
[TestFixture]
public sealed class TestPhysics
{
	/// <summary>Keep managed blittable records aligned with the ABI-reported native sizes.</summary>
	[Test]
	public unsafe void Layouts()
	{
		Native.EnsureLoaded();
		Assert.Equal(Native.ApiVersion, Native.Physics_ApiVersion());
		Assert.Equal(8, Marshal.SizeOf<NativeHeader>());
		Assert.Equal(8, sizeof(ShapeHandle));
		Assert.Equal(8, sizeof(BodyHandle));
		Assert.Equal(32, sizeof(SpatialVector));
		Assert.Equal(48, sizeof(BodyInertia));
		AssertNativeSize(1, Marshal.SizeOf<Native.EngineConfig>());
		AssertNativeSize(2, Marshal.SizeOf<Native.ShapeCommon>());
		AssertNativeSize(3, Marshal.SizeOf<Native.SphereShape>());
		AssertNativeSize(4, Marshal.SizeOf<Native.BoxShape>());
		AssertNativeSize(5, Marshal.SizeOf<Native.LineShape>());
		AssertNativeSize(6, Marshal.SizeOf<Native.TriangleShape>());
		AssertNativeSize(7, Marshal.SizeOf<Native.BodyDesc>());
		AssertNativeSize(8, Marshal.SizeOf<Native.BodyState>());
		AssertNativeSize(9, sizeof(BodyCommand));
		AssertNativeSize(10, sizeof(BodySnapshot));
		AssertNativeSize(11, sizeof(PhysicsEvent));
		AssertNativeSize(12, sizeof(Diagnostics));
		AssertNativeSize(13, Marshal.SizeOf<Native.MaterialValue>());
	}

	/// <summary>Exercise typed identities, bulk stepping, snapshots, checkpoints, and dependency-ordered disposal.</summary>
	[Test]
	public void Lifecycle()
	{
		using var runtime = new Physics();
		var engine = runtime.CreateEngine();
		var shape = engine.CreateSphere(0.5f);
		var body = engine.CreateBody(shape, new BodyOptions
		{
			ObjectToWorld = m4x4.Translation(0.0f, 1.0f, 0.0f),
			MassOrDensity = 1.0f,
			UserTag = 42,
		});

		Span<BodyCommand> commands = stackalloc BodyCommand[1];
		commands[0] = new BodyCommand(
			body.Handle,
			EBodyCommand.ApplyForce,
			m4x4.Identity,
			new SpatialVector(v4.Zero, new v4(0.0f, 1.0f, 0.0f, 0.0f)),
			v4.Zero);
		engine.Step(1.0f / 60.0f, commands: commands);

		Span<BodySnapshot> snapshots = stackalloc BodySnapshot[1];
		Assert.Equal(1, engine.CopySnapshots(snapshots));
		Assert.Equal(body.Handle, snapshots[0].m_body);
		Assert.Equal(shape.Handle, snapshots[0].m_shape);
		Assert.Equal(42UL, snapshots[0].m_user_tag);

		var checkpoint = new byte[engine.CheckpointSize()];
		Assert.Equal(checkpoint.Length, engine.WriteCheckpoint(checkpoint));
		Assert.Equal(1UL, engine.GetDiagnostics().m_completed_step);

		var body_handle = body.Handle;
		body.Dispose();
		shape.Dispose();
		engine.Dispose();

		// Restart into an empty engine and reopen wrappers from immutable snapshot identities.
		using var restored = runtime.CreateEngine();
		restored.ReadCheckpoint(checkpoint);
		Span<BodySnapshot> restored_snapshots = stackalloc BodySnapshot[1];
		Assert.Equal(1, restored.CopySnapshots(restored_snapshots));
		Assert.Equal(body_handle, restored_snapshots[0].m_body);
		using var restored_body = restored.OpenBody(body_handle);
		Assert.Equal(42UL, restored_body.GetState().m_user_tag);
		restored_body.Dispose();
		restored.Dispose();

		checkpoint[checkpoint.Length - 1] ^= 0x5A;
		using var corrupt_target = runtime.CreateEngine();
		ExpectStatus(EStatus.InvalidArgument, () => corrupt_target.ReadCheckpoint(checkpoint));
	}

	/// <summary>Reject owner-thread mutation before a call crosses the managed/native boundary.</summary>
	[Test]
	public void ThreadAffinity()
	{
		using var runtime = new Physics();
		using var engine = runtime.CreateEngine();
		Exception? error = null;
		var thread = new Thread(() =>
		{
			try
			{
				engine.Step(1.0f / 60.0f);
			}
			catch (Exception ex)
			{
				error = ex;
			}
		});
		thread.Start();
		thread.Join();
		Assert.Equal(typeof(InvalidOperationException), error?.GetType());
	}

	/// <summary>Prove independent COM leases survive their producing engine and can seed another engine.</summary>
	[Test]
	public void DeviceOwnership()
	{
		using var runtime = new Physics();
		var first = runtime.CreateEngine();
		using var lease = first.AcquireDeviceLease();
		using var clone = lease.Clone();
		first.Dispose();
		Assert.False(lease.IsDisposed);
		Assert.False(clone.IsDisposed);

		using var second = runtime.CreateEngine(device: lease);
		second.Step(1.0f / 60.0f);
	}

	/// <summary>Exercise every public native shape constructor and compound child ownership.</summary>
	[Test]
	public void ShapeSurface()
	{
		using var runtime = new Physics();
		using var engine = runtime.CreateEngine();
		engine.SetMaterial(new Material(1, 0.7f, 0.1f, 0.0f, 0.0f, 500.0f));
		Assert.Equal(500.0f, engine.GetMaterial(1).m_density);
		using var sphere = engine.CreateSphere(0.5f, new ShapeOptions
		{
			ShapeToRoot = m4x4.Translation(-0.75f, 0.0f, 0.0f),
			MaterialId = 1,
		});
		using var box = engine.CreateBox(new v4(1.0f, 0.5f, 0.5f, 0.0f), new ShapeOptions
		{
			ShapeToRoot = m4x4.Translation(+0.75f, 0.0f, 0.0f),
		});
		using var line = engine.CreateLine(1.0f, 0.1f);
		using var triangle = engine.CreateTriangle(
			new v4(-1.0f, -1.0f, 0.0f, 1.0f),
			new v4(+1.0f, -1.0f, 0.0f, 1.0f),
			new v4(0.0f, +1.0f, 0.0f, 1.0f));
		var points = new[]
		{
			new v4(-0.5f, -0.5f, -0.5f, 1.0f),
			new v4(+0.5f, -0.5f, -0.5f, 1.0f),
			new v4(0.0f, +0.5f, -0.5f, 1.0f),
			new v4(0.0f, 0.0f, +0.5f, 1.0f),
		};
		using var polytope = engine.CreatePolytope(points);
		using var compound = engine.CreateCompound(new[] { sphere, box });
		using var body = engine.CreateBody(compound);

		Assert.Throws<PhysicsException>(() => sphere.Dispose());
		engine.Step(1.0f / 60.0f);
		Assert.Equal(EMotionType.Dynamic, body.GetState().m_motion_type);
	}

	/// <summary>Reject stale/cross-engine handles, invalid lifetime order, insufficient buffers, and split-step misuse.</summary>
	[Test]
	public void ValidationAndMisuse()
	{
		using var runtime = new Physics();
		using var first = runtime.CreateEngine();
		using var second = runtime.CreateEngine();
		using var shape = first.CreateBox(new v4(1.0f, 1.0f, 1.0f, 0.0f));
		var body = first.CreateBody(shape);
		Assert.Throws<PhysicsException>(() => shape.Dispose());

		var cross_engine = new[] { BodyCommand.Wake(body.Handle) };
		ExpectStatus(EStatus.InvalidHandle, () => second.ApplyCommands(cross_engine));

		first.BeginStep(1.0f / 60.0f);
		ExpectStatus(EStatus.StepPending, () => first.BeginStep(1.0f / 60.0f));
		first.CompleteStep();
		ExpectStatus(EStatus.NoStepPending, () => first.CompleteStep());

		var too_small = Array.Empty<BodySnapshot>();
		ExpectStatus(EStatus.BufferTooSmall, () => first.CopySnapshots(too_small));

		var invalid_force = new[]
		{
			new BodyCommand(body.Handle, EBodyCommand.ApplyForce, m4x4.Identity, SpatialVector.Zero, v4.Origin),
		};
		ExpectStatus(EStatus.InvalidArgument, () => first.ApplyCommands(invalid_force));

		var stale_handle = body.Handle;
		body.Dispose();
		var stale = new[] { BodyCommand.Wake(stale_handle) };
		ExpectStatus(EStatus.StaleHandle, () => first.ApplyCommands(stale));

		// Application points are world-oriented offsets from the model origin, not absolute homogeneous positions.
		Assert.Throws<ArgumentException>(() => BodyCommand.ApplyForce(stale_handle, SpatialVector.Zero, v4.Origin));
	}

	/// <summary>Compare one managed blittable record against native ABI discovery.</summary>
	private static void AssertNativeSize(int struct_id, int managed_size)
	{
		Native.Check(Native.Physics_StructSize(struct_id, out var native_size));
		Assert.Equal((uint)managed_size, native_size);
	}

	/// <summary>Require one native call to fail with a specific stable ABI status.</summary>
	private static void ExpectStatus(EStatus expected, Action action)
	{
		try
		{
			action();
			throw new Rylogic.UnitTests.UnitTestException($"Expected native status {expected}.");
		}
		catch (PhysicsException ex)
		{
			Assert.Equal(expected, ex.Status);
		}
	}
}
#endif
