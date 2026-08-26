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
		Assert.Equal(8, sizeof(ArticulationHandle));
		Assert.Equal(8, sizeof(PersistentConstraintHandle));
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
		AssertNativeSize(14, Marshal.SizeOf<Native.ArticulationDesc>());
		AssertNativeSize(15, Marshal.SizeOf<Native.ArticulationLink>());
		AssertNativeSize(16, sizeof(Native.ArticulationJoint));
		AssertNativeSize(17, Marshal.SizeOf<Native.ArticulationState>());
		AssertNativeSize(18, Marshal.SizeOf<Native.ArticulationLinkState>());
		AssertNativeSize(19, Marshal.SizeOf<Native.D6Constraint>());
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

	/// <summary>Exercise articulation state, persistent endpoint ownership, internal substeps, diagnostics, and stale generations.</summary>
	[Test]
	public unsafe void ArticulationAndConstraintLifecycle()
	{
		using var runtime = new Physics();
		using var engine = runtime.CreateEngine();
		using var root_shape = engine.CreateBox(new v4(0.25f, 0.25f, 0.25f, 0.0f));
		using var child_shape = engine.CreateBox(new v4(0.25f, 0.25f, 0.25f, 0.0f));
		var articulation = CreateTestArticulation(engine, root_shape, child_shape);
		var body = engine.CreateBody(root_shape, new BodyOptions
		{
			ObjectToWorld = m4x4.Translation(0.0f, 0.0f, 1.0f),
			Gravity = v4.Zero,
			MassOrDensity = 1.0f,
		});

		// Whole-tree and flattened joint state round-trip without exposing native storage.
		var state_before = articulation.GetState();
		Assert.Equal(0xA17CUL, state_before.UserTag);
		Assert.Equal(2, state_before.LinkCount);
		Assert.Equal(1, state_before.Positions.Length);
		state_before.Positions[0] = 0.25f;
		state_before.Velocities[0] = -0.5f;
		state_before.Forces[0] = 2.0f;
		state_before.RootForce = new SpatialVector(v4.Zero, new v4(0.0f, 0.0f, -9.8f, 0.0f));
		state_before.Flags |= EArticulationFlags.NeverSleep;
		articulation.SetState(state_before);
		var state_after = articulation.GetState();
		Assert.Equal(0.25f, state_after.Positions[0]);
		Assert.Equal(-0.5f, state_after.Velocities[0]);
		Assert.Equal(2.0f, state_after.Forces[0]);
		Assert.True((state_after.Flags & EArticulationFlags.NeverSleep) != 0);

		// Persistent link fields remain independent from the generalized coordinate streams.
		var link_force = new SpatialVector(new v4(0.0f, 1.0f, 0.0f, 0.0f), new v4(1.0f, 0.0f, 0.0f, 0.0f));
		articulation.SetLinkForce(1, link_force);
		articulation.ApplyLinkForce(1, link_force);
		articulation.SetLinkGravity(1, new v4(0.0f, 0.0f, -9.8f, 0.0f));
		var links = articulation.CopyLinks();
		Assert.Equal(2, links.Length);
		Assert.Equal(-1, links[0].m_parent_index);
		Assert.Equal(0, links[1].m_parent_index);
		Assert.Equal(2.0f, links[1].m_external_force.m_linear.x);
		Assert.Equal(-9.8f, links[1].m_gravity.z);

		// A persistent coupled endpoint retains both dynamics owners until its declaration is destroyed.
		var options = D6ConstraintOptions.Weld(
			ConstraintFrame.ForBody(body, m4x4.Identity),
			ConstraintFrame.ForLink(articulation, 1, m4x4.Identity));
		var constraint = engine.CreateConstraint(options);
		var constraint_state = constraint.GetState();
		Assert.False(constraint_state.Broken);
		Assert.Equal(EConstraintEndpoint.RigidBody, constraint_state.Options.FrameA.Type);
		Assert.Equal(EConstraintEndpoint.ArticulationLink, constraint_state.Options.FrameB.Type);
		ExpectStatus(EStatus.InvalidArgument, () => body.Dispose());
		ExpectStatus(EStatus.InvalidArgument, () => articulation.Dispose());

		// Mutable declarations preserve stable identity and participate in one frame-wide GPU transaction.
		constraint_state.Options.Angular[0] = ConstraintAxis.Driven(0.0f, 0.25f, 0.0f, 1.0f, 1000.0f);
		constraint.Update(constraint_state.Options);
		constraint.SetEnabled(false);
		constraint.SetEnabled(true);
		constraint.Repair();
		engine.Step(1.0f / 120.0f, substep_count: 2);
		var diagnostics = engine.GetDiagnostics();
		Assert.Equal(1, diagnostics.m_articulation_count);
		Assert.Equal(1, diagnostics.m_constraint_count);
		Assert.Equal(1, diagnostics.m_coupled.m_constraint_count);
		Assert.Equal(1, diagnostics.m_frame_output.m_readback_count);
		Assert.Equal(EStepFailure.None, diagnostics.m_failure.m_reason);
		ExpectStatus(EStatus.InvalidArgument, () => engine.CheckpointSize());

		// Destroyed generations reject direct ABI access after managed wrappers have relinquished ownership.
		var constraint_handle = constraint.Handle;
		constraint.Dispose();
		ExpectStatus(EStatus.StaleHandle, () => Native.Check(Native.Physics_ConstraintRepair(engine.Handle, constraint_handle.Value)));
		var articulation_handle = articulation.Handle;
		articulation.Dispose();
		uint link_count;
		ExpectStatus(EStatus.StaleHandle, () => Native.Check(Native.Physics_ArticulationLinksCopy(engine.Handle, articulation_handle.Value, null, 0, out link_count)));
		body.Dispose();
	}

	/// <summary>Marshal break events and nested diagnostics while retaining the one-readback frame contract.</summary>
	[Test]
	public void ConstraintBreakEventsAndDiagnostics()
	{
		using var runtime = new Physics();
		using var engine = runtime.CreateEngine();
		using var shape = engine.CreateBox(new v4(1.0f, 1.0f, 1.0f, 0.0f));
		using var body = engine.CreateBody(shape, new BodyOptions
		{
			Gravity = v4.Zero,
			MassOrDensity = 1.0f,
			Momentum = new SpatialVector(v4.Zero, new v4(10.0f, 0.0f, 0.0f, 0.0f)),
		});
		var options = D6ConstraintOptions.Weld(
			ConstraintFrame.World(m4x4.Identity),
			ConstraintFrame.ForBody(body, m4x4.Identity));
		options.BreakForce = 0.01f;
		using var constraint = engine.CreateConstraint(options);

		// A bounded break is edge-triggered and returned through the frame's existing packed readback.
		engine.Step(1.0f / 60.0f, substep_count: 2);
		var events = new PhysicsEvent[engine.EventCount()];
		Assert.Equal(events.Length, engine.CopyEvents(events));
		var break_event_index = -1;
		for (var i = 0; i != events.Length; ++i)
		{
			if (events[i].m_type == EPhysicsEvent.ConstraintBreak && events[i].m_constraint == constraint.Handle)
				break_event_index = i;
		}
		Assert.True(break_event_index >= 0);
		var break_event = events[break_event_index];
		Assert.True(break_event.m_break_force >= options.BreakForce);
		Assert.True(break_event.m_substep_index >= 0 && break_event.m_substep_index < 2);
		Assert.True(constraint.GetState().Broken);

		// Diagnostics expose stable feature counts and exactly one readback for all internal substeps.
		var diagnostics = engine.GetDiagnostics();
		Assert.Equal(1UL, diagnostics.m_submitted_step);
		Assert.Equal(1UL, diagnostics.m_completed_step);
		Assert.Equal(1, diagnostics.m_constraint_count);
		Assert.Equal(1, diagnostics.m_constraints.m_breakable_count);
		Assert.Equal(1, diagnostics.m_frame_output.m_readback_count);
		Assert.Equal(EStepFailure.None, diagnostics.m_failure.m_reason);
		constraint.Repair();
		Assert.False(constraint.GetState().Broken);
	}

	/// <summary>Invalidate managed dependants after native engine teardown in dependency order.</summary>
	[Test]
	public void EngineOwnsConstraintDependencies()
	{
		using var runtime = new Physics();
		var engine = runtime.CreateEngine();
		var root_shape = engine.CreateBox(new v4(0.25f, 0.25f, 0.25f, 0.0f));
		var child_shape = engine.CreateBox(new v4(0.25f, 0.25f, 0.25f, 0.0f));
		var articulation = CreateTestArticulation(engine, root_shape, child_shape);
		var body = engine.CreateBody(root_shape);
		var constraint = engine.CreateConstraint(new D6ConstraintOptions(
			ConstraintFrame.ForBody(body, m4x4.Identity),
			ConstraintFrame.ForLink(articulation, 1, m4x4.Identity)));

		// Native engine destruction owns the dependency graph; every managed identity becomes unusable afterward.
		engine.Dispose();
		Assert.True(constraint.IsDisposed);
		Assert.True(articulation.IsDisposed);
		Assert.True(body.IsDisposed);
		Assert.True(root_shape.IsDisposed);
		Assert.True(child_shape.IsDisposed);
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

	/// <summary>Create the canonical two-link floating articulation used by managed ABI lifecycle tests.</summary>
	private static Articulation CreateTestArticulation(Engine engine, Shape root_shape, Shape child_shape)
	{
		var inertia = new BodyInertia(
			new v4(1.0f, 1.0f, 1.0f, 0.0f),
			v4.Zero,
			v4.Zero,
			1.0f);
		var links = new[]
		{
			new ArticulationLinkOptions(root_shape, -1)
			{
				Inertia = inertia,
				Flags = EArticulationLinkFlags.CollideSelf,
			},
			new ArticulationLinkOptions(child_shape, 0)
			{
				Inertia = inertia,
				Flags = EArticulationLinkFlags.CollideSelf,
			},
		};
		var joints = new[]
		{
			new ArticulationJointOptions(
				m4x4.Translation(0.0f, 0.0f, 1.0f),
				m4x4.Identity,
				new ArticulationAxis(new v4(0.0f, 0.0f, 1.0f, 0.0f), EArticulationAxis.Revolute)),
		};
		return engine.CreateArticulation(
			new ArticulationOptions
			{
				RootToWorld = m4x4.Translation(0.0f, 0.0f, 2.0f),
				UserTag = 0xA17CUL,
				RootType = EArticulationRoot.Floating,
			},
			links,
			joints);
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
