#if PR_UNITTESTS
using System;
using System.Linq;
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
		AssertNativeSize(20, Marshal.SizeOf<TerrainConfiguration>());
		AssertNativeSize(21, Marshal.SizeOf<CylindricalBoundaryConfiguration>());
		AssertNativeSize(22, Marshal.SizeOf<Native.WaterDesc>());
		AssertNativeSize(23, Marshal.SizeOf<Native.WaterBathymetryDesc>());
		AssertNativeSize(24, sizeof(Native.AtmosphereDesc));
		AssertNativeSize(25, Marshal.SizeOf<Native.AtmosphereStepDesc>());
		AssertNativeSize(26, Marshal.SizeOf<Native.AtmosphereStats>());
		Assert.Equal(32, sizeof(AtmosphereHeatSource));
		Assert.Equal(16, sizeof(AtmosphereOutsideAir));
		Assert.Equal(24, sizeof(AtmosphereTracerParticle));
		Assert.Equal(16, sizeof(AtmosphereCellState));
		Assert.Equal(48, Marshal.OffsetOf<Native.AtmosphereDesc>(nameof(Native.AtmosphereDesc.m_floor_heights)).ToInt32());
		Assert.Equal(56, Marshal.OffsetOf<Native.AtmosphereDesc>(nameof(Native.AtmosphereDesc.m_active_columns)).ToInt32());
		Assert.Equal(192, Marshal.OffsetOf<Native.AtmosphereDesc>(nameof(Native.AtmosphereDesc.m_tracer_break_height)).ToInt32());
		Assert.Equal(WaterFieldElement.SizeInBytes, sizeof(WaterFieldElement));
		Assert.Equal(8, Marshal.OffsetOf<Native.WaterDesc>(nameof(Native.WaterDesc.m_level)).ToInt32());
		Assert.Equal(40, Marshal.OffsetOf<Native.WaterDesc>(nameof(Native.WaterDesc.m_elements)).ToInt32());
		Assert.Equal(40, Marshal.OffsetOf<Native.WaterBathymetryDesc>(nameof(Native.WaterBathymetryDesc.m_heights)).ToInt32());
		Assert.Equal(8, Marshal.OffsetOf<CylindricalBoundaryConfiguration>(nameof(CylindricalBoundaryConfiguration.m_centre_x)).ToInt32());
		Assert.Equal(32, Marshal.OffsetOf<CylindricalBoundaryConfiguration>(nameof(CylindricalBoundaryConfiguration.m_material_id)).ToInt32());
		Assert.Equal(36, Marshal.OffsetOf<CylindricalBoundaryConfiguration>(nameof(CylindricalBoundaryConfiguration.m_surface_spacing)).ToInt32());
	}

	/// <summary>The cylindrical boundary has a stable ABI, independent lifetime, and explicit checkpoint and mutation guards.</summary>
	[Test]
	public void CylindricalBoundary()
	{
		using var runtime = new Physics();
		using var engine = runtime.CreateEngine();
		var checkpoint = new byte[engine.CheckpointSize()];
		engine.WriteCheckpoint(checkpoint);
		engine.SetCylindricalBoundary(new CylindricalBoundaryConfiguration(0, 0, 4000));
		ExpectStatus(EStatus.InvalidArgument, () => engine.SetCylindricalBoundary(new CylindricalBoundaryConfiguration(0, 0, double.NaN)));
		ExpectStatus(EStatus.InvalidArgument, () => engine.ReadCheckpoint(checkpoint));
		using var shape = engine.CreateSphere(0.3f);
		using var body = engine.CreateBody(shape, new BodyOptions { ObjectToWorld = m4x4.Translation(3999.65f, 0, 1000), MassOrDensity = 1 });
		engine.BeginStep(1f / 240);
		ExpectStatus(EStatus.StepPending, () => engine.SetCylindricalBoundary(null));
		engine.CompleteStep();
		ExpectStatus(EStatus.InvalidArgument, () => engine.CheckpointSize());

		// Clearing unrelated terrain must not disable the independent wall or its checkpoint guard.
		engine.SetTerrain(null);
		ExpectStatus(EStatus.InvalidArgument, () => engine.CheckpointSize());
		engine.SetCylindricalBoundary(null);
		Assert.True(engine.CheckpointSize() > 0);
	}

	/// <summary>Water floats a light body at its analytic draft, rejects invalid and pending changes, and keeps checkpoints available.</summary>
	[Test]
	public void Water()
	{
		using var runtime = new Physics();
		using var engine = runtime.CreateEngine();
		ExpectStatus(EStatus.InvalidArgument, () => engine.SetWater(new WaterConfiguration(0, density: 0)));
		ExpectStatus(EStatus.InvalidArgument, () => engine.SetWater(new WaterConfiguration(double.NaN)));
		engine.SetWater(new WaterConfiguration(2, linear_drag_rate: 2));

		// A sphere of half the water's density floats with its centre on the surface.
		using var shape = engine.CreateSphere(0.5f);
		using var body = engine.CreateBody(shape, new BodyOptions { ObjectToWorld = m4x4.Translation(0, 0, 2.4f), Gravity = v4.Zero, MassMode = EMassMode.Density, MassOrDensity = 500 });
		var commands = new[] { BodyCommand.SetGravity(body.Handle, new v4(0, 0, -9.81f, 0)) };
		engine.BeginStep(1f / 60, commands: commands);
		ExpectStatus(EStatus.StepPending, () => engine.SetWater(null));
		engine.CompleteStep();
		for (var step = 0; step != 600; ++step)
			engine.Step(1f / 60, commands: commands);

		// Require the analytic draft within a small settling tolerance.
		var height = body.GetState().m_object_to_world.pos.z;
		if (Math.Abs(height - 2.0f) > 0.02f || float.IsNaN(height))
			throw new Exception($"Water sphere did not float at its draft: height={height}, velocity={body.GetState().m_velocity.m_linear}");

		// Water is environment configuration, so body checkpoints remain available.
		Assert.True(engine.CheckpointSize() > 0);

		// Removing the water restores free fall.
		engine.SetWater(null);
		for (var step = 0; step != 60; ++step)
			engine.Step(1f / 60, commands: commands);

		Assert.True(body.GetState().m_object_to_world.pos.z < 0);
	}

	/// <summary>Wind-driven waves build towards their targets, marshal as elements, and apply with terrain heights through the engine.</summary>
	[Test]
	public void Waves()
	{
		// Calm air makes no waves, and wind makes waves.
		var layout = new WaveSpectrumLayout(9.81f, 1024f, 0.5f, 256f, 16, 4);
		var targets = new float[layout.ComponentCount];
		WaveSpectrum.Targets(layout, new WaveWeather(0, 10_000), targets);
		Assert.True(Array.TrueForAll(targets, a => a == 0));
		WaveSpectrum.Targets(layout, new WaveWeather(10, 10_000), targets);
		Assert.True(Array.Exists(targets, a => a > 0));

		// Relaxation reaches the targets, and a minimum wavelength removes short components.
		var amplitudes = new float[layout.ComponentCount];
		WaveSpectrum.Relax(amplitudes, targets, 1e4f, 20f);
		Assert.Equal(targets[targets.Length - 1], amplitudes[amplitudes.Length - 1]);
		var elements = new WaterFieldElement[layout.ComponentCount];
		var all = WaveSpectrum.Elements(layout, amplitudes, 0, 0, 0, elements);
		var sharpness = WaveSpectrum.CrestSharpness(10);
		var long_only = WaveSpectrum.Elements(layout, amplitudes, 0, sharpness, 4, elements);
		Assert.True(all > long_only && long_only > 0);
		Assert.Equal(WaterFieldElement.TypeGerstnerWave, elements[0].m_type);
		Assert.True(elements[0].m_wave.y >= 4);

		// Light wind gives sine-shaped waves, and stronger wind sharpens the crests.
		Assert.Equal(0f, WaveSpectrum.CrestSharpness(4));
		Assert.True(sharpness > 0 && sharpness < WaveSpectrum.CrestSharpness(40));
		Assert.Equal(sharpness, elements[0].m_timing.x);
		ExpectStatus(EStatus.InvalidArgument, () => WaveSpectrum.Elements(layout, amplitudes, 0, 0.95f, 4, elements));

		// The engine accepts the waves with terrain heights, and rejects a grid that does not match its heights.
		using var runtime = new Physics();
		using var engine = runtime.CreateEngine();
		var heights = new float[] { -20, -20, 5, 5 };
		engine.SetWaterBathymetry(-100, -100, 200, 2, 2, heights);
		engine.SetWater(new WaterConfiguration(0, repeat_period: 1024), elements.AsSpan(0, long_only));
		Assert.Throws<ArgumentException>(() => engine.SetWaterBathymetry(0, 0, 1, 3, 3, heights));
		ExpectStatus(EStatus.InvalidArgument, () => engine.SetWater(new WaterConfiguration(0, breaking_ratio: 0), elements.AsSpan(0, long_only)));
		engine.ClearWaterBathymetry();
		engine.SetWater(null);
	}

	/// <summary>An atmosphere heats rising air, reports pending steps, copies tracers from any thread, and is released with its engine.</summary>
	[Test]
	public void AtmosphereLifecycle()
	{
		var runtime = new Physics();
		var engine = runtime.CreateEngine();
		var options = new AtmosphereOptions
		{
			CellCountX = 8,
			CellCountY = 8,
			CellCountZ = 4,
			CellSize = 1,
			LidZ = 4,
			TracerCount = 64,
			TracerSeed = 1,
		};
		options.ActiveColumns = Enumerable.Repeat((byte)1, options.CellCountX * options.CellCountY).ToArray();
		options.FloorHeights = new float[options.CellCountX * options.CellCountY];
		options.ActiveColumns[0] = 0;
		options.FloorHeights[0] = float.NaN;
		ExpectStatus(EStatus.InvalidArgument, () => engine.CreateAtmosphere(new AtmosphereOptions { CellCountX = 8, CellCountY = 8, CellCountZ = 4, CellSize = 0, LidZ = 4 }));
		var atmosphere = engine.CreateAtmosphere(options);
		Assert.Equal(256, atmosphere.CellCount);

		// A heat source at one cell warms its air and drives flow; the pending step blocks a second submission and field readback.
		var heat = new[] { new AtmosphereHeatSource(new v4(4, 4, 1, 1), 1.5f, 5, 320, 0) };
		ExpectStatus(EStatus.NoStepPending, () => atmosphere.CompleteStep());
		atmosphere.BeginStep(0.1f, heat_sources: heat);
		ExpectStatus(EStatus.StepPending, () => atmosphere.BeginStep(0.1f, heat_sources: heat));
		var cells = new AtmosphereCellState[atmosphere.CellCount];
		ExpectStatus(EStatus.StepPending, () => atmosphere.CopyCellStates(cells));
		atmosphere.CompleteStep();
		for (var step = 0; step != 20; ++step)
		{
			atmosphere.BeginStep(0.1f, heat_sources: heat);
			while (!atmosphere.PollStep())
				Thread.Yield();
		}
		var stats = atmosphere.CopyCellStates(cells);
		Assert.True(cells[atmosphere.CellIndex(4, 4, 0)].m_temperature > 288.5f);
		Assert.True(stats.MaxSpeed > 0);
		ExpectStatus(EStatus.BufferTooSmall, () => atmosphere.CopyCellStates(cells.AsSpan(1)));

		// Tracers are readable from a worker thread; mutation from a worker is rejected before reaching native code.
		var particles = new AtmosphereTracerParticle[atmosphere.TracerCount];
		var copied = 0;
		Exception? worker_failure = null;
		var worker = new Thread(() =>
		{
			try
			{
				copied = atmosphere.CopyTracers(particles);
				atmosphere.CompleteStep();
			}
			catch (Exception ex)
			{
				worker_failure = ex;
			}
		});
		worker.Start();
		worker.Join();
		Assert.Equal(64, copied);
		Assert.True(worker_failure is InvalidOperationException);
		Assert.True(Array.TrueForAll(particles, p => p.m_z >= 0 && p.m_z <= 4));
		Assert.True(Array.TrueForAll(particles, p => !(p.m_x < 1 && p.m_y < 1)));

		// Floors must match the column count; atmospheres block checkpoint restore and are released with their engine.
		ExpectStatus(EStatus.InvalidArgument, () => atmosphere.SetFloors(new float[3]));
		atmosphere.SetFloors(new float[atmosphere.ColumnCount]);
		Assert.Throws<InvalidOperationException>(() => engine.ReadCheckpoint(new byte[16]));
		atmosphere.BeginStep(0.1f);
		engine.Dispose();
		Assert.True(atmosphere.IsDisposed);
		runtime.Dispose();
	}

	/// <summary>Terrain supports a falling body, rejects pending mutation and incomplete checkpoints, and can be removed.</summary>
	[Test]
	public void Terrain()
	{
		using var runtime = new Physics();
		using var engine = runtime.CreateEngine();
		var band = new TerrainBand(0, 100, 1, 2, 0.5);

		// Disabled bands leave a fixed hill-family datum; cancel its normalized blend for a zero-height plane. A basin threshold above the unit range
		// keeps the flat depression field out of the basin shore band.
		var blend = (0.5 - 0.28) / (0.58 - 0.28);
		var datum = 35 / (2 - blend * blend * (3 - 2 * blend));
		engine.SetTerrain(new TerrainConfiguration(42, 0, 1e6, -datum, 0, 0, 0, 2, band, band, band, band, band, band, band, band));
		engine.SetMaterial(new Material(0, 0.5f, 0, 0, 0, 1));
		using var shape = engine.CreateSphere(0.5f);
		using var body = engine.CreateBody(shape, new BodyOptions { ObjectToWorld = m4x4.Translation(0, 0, 2), Gravity = v4.Zero, MassOrDensity = 1 });
		var commands = new[] { BodyCommand.SetGravity(body.Handle, new v4(0, 0, -9.81f, 0)) };

		// Mutation and serialization cannot invalidate or silently omit the active collision authority.
		engine.BeginStep(1f / 60, commands: commands);
		ExpectStatus(EStatus.StepPending, () => engine.SetTerrain(null));
		engine.CompleteStep();
		ExpectStatus(EStatus.InvalidArgument, () => engine.CheckpointSize());
		for (var step = 0; step != 180; ++step)
			engine.Step(1f / 60, commands: commands);

		// Require support near the sphere radius above the analytically zero-height plane.
		var height = body.GetState().m_object_to_world.pos.z;
		if (height <= 0.45f || height >= 0.55f || float.IsNaN(height))
			throw new Exception($"Terrain sphere did not settle: height={height}, velocity={body.GetState().m_velocity.m_linear}");

		// Removing the source must restore free fall rather than leave cached terrain contacts behind.
		engine.SetTerrain(null);
		for (var step = 0; step != 60; ++step)
			engine.Step(1f / 60, commands: commands);

		// Passing through the former ground rules out retained support after removal.
		Assert.True(body.GetState().m_object_to_world.pos.z < 0);
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
