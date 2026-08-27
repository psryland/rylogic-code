using System;
using System.Collections.Generic;
using System.Runtime.ExceptionServices;
using System.Runtime.InteropServices;
using Rylogic.D3D12;
using Rylogic.Maths;

namespace Rylogic.Physics;

/// <summary>Owns one native rigid-body world, its D3D12 compute resources, shapes, and bodies.</summary>
public sealed class Engine :IDisposable
{
	private readonly Physics m_runtime;
	private readonly uint m_owner_thread_id;
	private readonly HashSet<Shape> m_shapes;
	private readonly HashSet<RigidBody> m_bodies;
	private readonly HashSet<Articulation> m_articulations;
	private readonly HashSet<PersistentConstraint> m_constraints;
	private EngineSafeHandle? m_handle;

	/// <summary>Adopt a newly-created native engine.</summary>
	internal Engine(Physics runtime, ulong handle, uint owner_thread_id)
	{
		m_runtime = runtime;
		m_owner_thread_id = owner_thread_id;
		m_shapes = new HashSet<Shape>();
		m_bodies = new HashSet<RigidBody>();
		m_articulations = new HashSet<Articulation>();
		m_constraints = new HashSet<PersistentConstraint>();
		m_handle = new EngineSafeHandle(handle, runtime);
	}

	/// <summary>True after this engine has released its native world.</summary>
	public bool IsDisposed
	{
		get
		{
			return m_handle == null || m_handle.IsInvalid || m_handle.IsClosed;
		}
	}

	/// <summary>Acquire an independent COM lease to the D3D12 device used by this engine.</summary>
	public DeviceLease AcquireDeviceLease()
	{
		Native.Check(Native.Physics_EngineDeviceLeaseAcquire(Handle, out var device));
		return new DeviceLease(device);
	}

	/// <summary>Read the active engine configuration.</summary>
	public unsafe EngineOptions GetOptions()
	{
		var config = new Native.EngineConfig
		{
			m_header = NativeHeader.Create<Native.EngineConfig>(),
		};
		Native.Check(Native.Physics_EngineConfigGet(Handle, &config));
		return config.ToPublic();
	}

	/// <summary>Replace the active engine configuration outside a pending step.</summary>
	public unsafe void SetOptions(EngineOptions options)
	{
		EnsureOwner();
		if (options == null)
			throw new ArgumentNullException(nameof(options));

		var config = Native.EngineConfig.From(options);
		Native.Check(Native.Physics_EngineConfigSet(Handle, &config));
	}

	/// <summary>Create a sphere collision shape.</summary>
	public unsafe Shape CreateSphere(float radius, ShapeOptions? options = null, bool hollow = false)
	{
		EnsureOwner();
		var desc = new Native.SphereShape
		{
			m_common = Native.ShapeCommon.From<Native.SphereShape>(options),
			m_radius = radius,
			m_hollow = hollow ? 1 : 0,
		};
		Native.Check(Native.Physics_ShapeCreateSphere(Handle, &desc, out var handle));
		return AddShape(handle);
	}

	/// <summary>Create a box collision shape from full dimensions.</summary>
	public unsafe Shape CreateBox(v4 dimensions, ShapeOptions? options = null)
	{
		EnsureOwner();
		var desc = new Native.BoxShape
		{
			m_common = Native.ShapeCommon.From<Native.BoxShape>(options),
			m_dimensions = dimensions.w0,
		};
		Native.Check(Native.Physics_ShapeCreateBox(Handle, &desc, out var handle));
		return AddShape(handle);
	}

	/// <summary>Create a Z-aligned line-segment collision shape with optional radius.</summary>
	public unsafe Shape CreateLine(float length, float radius = 0.0f, ShapeOptions? options = null)
	{
		EnsureOwner();
		var desc = new Native.LineShape
		{
			m_common = Native.ShapeCommon.From<Native.LineShape>(options),
			m_length = length,
			m_radius = radius,
		};
		Native.Check(Native.Physics_ShapeCreateLine(Handle, &desc, out var handle));
		return AddShape(handle);
	}

	/// <summary>Create a triangle collision shape.</summary>
	public unsafe Shape CreateTriangle(v4 a, v4 b, v4 c, ShapeOptions? options = null)
	{
		EnsureOwner();
		var desc = new Native.TriangleShape
		{
			m_common = Native.ShapeCommon.From<Native.TriangleShape>(options),
			m_a = a.w1,
			m_b = b.w1,
			m_c = c.w1,
		};
		Native.Check(Native.Physics_ShapeCreateTriangle(Handle, &desc, out var handle));
		return AddShape(handle);
	}

	/// <summary>Create a convex polytope from a caller-owned point span.</summary>
	public unsafe Shape CreatePolytope(ReadOnlySpan<v4> points, ShapeOptions? options = null)
	{
		EnsureOwner();
		var common = Native.ShapeCommon.From<Native.ShapeCommon>(options);
		fixed (v4* point_ptr = points)
		{
			Native.Check(Native.Physics_ShapeCreatePolytope(Handle, &common, point_ptr, (uint)points.Length, out var handle));
			return AddShape(handle);
		}
	}

	/// <summary>Create a compound collision shape from engine-owned child shapes.</summary>
	public unsafe Shape CreateCompound(ReadOnlySpan<Shape> children, ShapeOptions? options = null)
	{
		EnsureOwner();
		var handles = new ShapeHandle[children.Length];
		for (var i = 0; i != children.Length; ++i)
		{
			if (!ReferenceEquals(children[i].Engine, this))
				throw new ArgumentException("All compound children must belong to this engine.", nameof(children));

			handles[i] = children[i].Handle;
		}

		var common = Native.ShapeCommon.From<Native.ShapeCommon>(options);
		fixed (ShapeHandle* child_ptr = handles)
		{
			Native.Check(Native.Physics_ShapeCreateCompound(Handle, &common, child_ptr, (uint)handles.Length, out var handle));
			return AddShape(handle);
		}
	}

	/// <summary>Create a rigid body from an engine-owned shape and initial state.</summary>
	public unsafe RigidBody CreateBody(Shape shape, BodyOptions? options = null)
	{
		EnsureOwner();
		if (!ReferenceEquals(shape.Engine, this))
			throw new ArgumentException("The body shape must belong to this engine.", nameof(shape));

		options ??= new BodyOptions();
		var desc = new Native.BodyDesc
		{
			m_header = NativeHeader.Create<Native.BodyDesc>(),
			m_shape = shape.Handle,
			m_object_to_world = options.ObjectToWorld,
			m_inertia = options.Inertia,
			m_momentum = options.Momentum,
			m_gravity = options.Gravity,
			m_user_tag = options.UserTag,
			m_motion_type = options.MotionType,
			m_mass_mode = options.MassMode,
			m_mass_or_density = options.MassOrDensity,
			m_flags = options.Flags,
		};
		Native.Check(Native.Physics_BodyCreate(Handle, &desc, out var handle));
		var body = new RigidBody(this, shape, new BodyHandle(handle));
		m_bodies.Add(body);
		return body;
	}

	/// <summary>Create a reduced-coordinate articulation from topologically ordered links and one joint per non-root link.</summary>
	public unsafe Articulation CreateArticulation(ArticulationOptions options, IReadOnlyList<ArticulationLinkOptions> links, IReadOnlyList<ArticulationJointOptions> joints)
	{
		EnsureOwner();
		if (options == null)
			throw new ArgumentNullException(nameof(options));
		if (links == null)
			throw new ArgumentNullException(nameof(links));
		if (joints == null)
			throw new ArgumentNullException(nameof(joints));
		if (links.Count == 0)
			throw new ArgumentException("An articulation requires at least one root link.", nameof(links));
		if (joints.Count != links.Count - 1)
			throw new ArgumentException("An articulation requires exactly one joint for each non-root link.", nameof(joints));

		var native_links = new Native.ArticulationLink[links.Count];
		var native_joints = new Native.ArticulationJoint[joints.Count];
		var shapes = new Shape[links.Count];

		// Reject malformed topology and cross-engine shapes before native storage acquires any references.
		for (var i = 0; i != links.Count; ++i)
		{
			var link = links[i] ?? throw new ArgumentException("Articulation link declarations cannot contain null entries.", nameof(links));
			if (!ReferenceEquals(link.Shape.Engine, this))
				throw new ArgumentException("Every articulation link shape must belong to this engine.", nameof(links));
			if (i == 0 && link.ParentIndex != -1)
				throw new ArgumentException("The first articulation link must be the root and have parent index -1.", nameof(links));
			if (i != 0 && (link.ParentIndex < 0 || link.ParentIndex >= i))
				throw new ArgumentException("Every non-root articulation link must reference an earlier parent link.", nameof(links));

			shapes[i] = link.Shape;
			native_links[i] = Native.ArticulationLink.From(link);
		}
		for (var i = 0; i != joints.Count; ++i)
			native_joints[i] = Native.ArticulationJoint.From(joints[i] ?? throw new ArgumentException("Articulation joint declarations cannot contain null entries.", nameof(joints)));

		var desc = Native.ArticulationDesc.From(options, links.Count);
		fixed (Native.ArticulationLink* link_ptr = native_links)
		fixed (Native.ArticulationJoint* joint_ptr = native_joints)
		{
			Native.Check(Native.Physics_ArticulationCreate(Handle, &desc, link_ptr, joint_ptr, out var handle));
			var articulation = new Articulation(this, new ArticulationHandle(handle), shapes);
			m_articulations.Add(articulation);
			return articulation;
		}
	}

	/// <summary>Create an engine-owned persistent D6 constraint between validated world, body, or articulation-link endpoints.</summary>
	public unsafe PersistentConstraint CreateConstraint(D6ConstraintOptions options)
	{
		EnsureOwner();
		if (options == null)
			throw new ArgumentNullException(nameof(options));

		ValidateConstraintEndpoints(options);
		var desc = Native.D6Constraint.From(options);
		Native.Check(Native.Physics_ConstraintCreateD6(Handle, &desc, out var handle));
		var constraint = new PersistentConstraint(this, new PersistentConstraintHandle(handle));
		m_constraints.Add(constraint);
		return constraint;
	}

	/// <summary>Open an object wrapper for a body identity recovered from a checkpoint snapshot.</summary>
	public unsafe RigidBody OpenBody(BodyHandle handle)
	{
		EnsureOwner();
		foreach (var existing in m_bodies)
		{
			if (existing.Handle == handle)
				return existing;
		}

		var state = new Native.BodyState
		{
			m_header = NativeHeader.Create<Native.BodyState>(),
		};
		Native.Check(Native.Physics_BodyStateGet(Handle, handle.Value, &state));

		Shape? shape = null;
		foreach (var existing in m_shapes)
		{
			if (existing.Handle != state.m_shape)
				continue;

			shape = existing;
			break;
		}
		shape ??= AddShape(state.m_shape.Value);

		var body = new RigidBody(this, shape, handle);
		m_bodies.Add(body);
		return body;
	}

	/// <summary>Get one material from the native material table.</summary>
	public unsafe Material GetMaterial(int material_id)
	{
		var material = new Native.MaterialValue();
		Native.Check(Native.Physics_MaterialGet(Handle, material_id, &material));
		return material.ToPublic();
	}

	/// <summary>Set one material on the owner thread.</summary>
	public unsafe void SetMaterial(Material material)
	{
		EnsureOwner();
		var value = Native.MaterialValue.From(material);
		Native.Check(Native.Physics_MaterialSet(Handle, &value));
	}

	/// <summary>Apply a validated command span without stepping.</summary>
	public unsafe void ApplyCommands(ReadOnlySpan<BodyCommand> commands)
	{
		EnsureOwner();
		fixed (BodyCommand* command_ptr = commands)
			Native.Check(Native.Physics_CommandsApply(Handle, command_ptr, (uint)commands.Length));
	}

	/// <summary>Submit one simulation step without waiting for GPU completion.</summary>
	public unsafe void BeginStep(float elapsed_seconds, double absolute_time_seconds = 0.0, ReadOnlySpan<BodyCommand> commands = default, int substep_count = 1)
	{
		EnsureOwner();
		if (substep_count <= 0)
			throw new ArgumentOutOfRangeException(nameof(substep_count));

		fixed (BodyCommand* command_ptr = commands)
			Native.Check(Native.Physics_BeginStepEx(Handle, elapsed_seconds, absolute_time_seconds, checked((uint)substep_count), command_ptr, (uint)commands.Length));
	}

	/// <summary>Wait for and unpack the pending simulation step.</summary>
	public void CompleteStep()
	{
		EnsureOwner();
		Native.Check(Native.Physics_CompleteStep(Handle));
	}

	/// <summary>Submit and complete one simulation step synchronously.</summary>
	public unsafe void Step(float elapsed_seconds, double absolute_time_seconds = 0.0, ReadOnlySpan<BodyCommand> commands = default, int substep_count = 1)
	{
		EnsureOwner();
		if (substep_count <= 0)
			throw new ArgumentOutOfRangeException(nameof(substep_count));

		fixed (BodyCommand* command_ptr = commands)
			Native.Check(Native.Physics_StepEx(Handle, elapsed_seconds, absolute_time_seconds, checked((uint)substep_count), command_ptr, (uint)commands.Length));
	}

	/// <summary>Return the number of body snapshots required for a complete copy.</summary>
	public unsafe int SnapshotCount()
	{
		var status = Native.Physics_SnapshotCopy(Handle, null, 0, out var required);
		if (status != EStatus.Success && status != EStatus.BufferTooSmall)
			Native.Check(status);

		return checked((int)required);
	}

	/// <summary>Copy all completed body snapshots without partial writes.</summary>
	public unsafe int CopySnapshots(Span<BodySnapshot> snapshots)
	{
		fixed (BodySnapshot* snapshot_ptr = snapshots)
		{
			Native.Check(Native.Physics_SnapshotCopy(Handle, snapshot_ptr, (uint)snapshots.Length, out var required));
			return checked((int)required);
		}
	}

	/// <summary>Return the number of events buffered by the most recently completed step.</summary>
	public unsafe int EventCount()
	{
		var status = Native.Physics_EventsCopy(Handle, null, 0, out var required);
		if (status != EStatus.Success && status != EStatus.BufferTooSmall)
			Native.Check(status);

		return checked((int)required);
	}

	/// <summary>Copy all buffered events without partial writes.</summary>
	public unsafe int CopyEvents(Span<PhysicsEvent> events)
	{
		fixed (PhysicsEvent* event_ptr = events)
		{
			Native.Check(Native.Physics_EventsCopy(Handle, event_ptr, (uint)events.Length, out var required));
			return checked((int)required);
		}
	}

	/// <summary>Read generic completion, capacity, timing, checksum, and device diagnostics.</summary>
	public unsafe Diagnostics GetDiagnostics()
	{
		var diagnostics = new Diagnostics();
		Native.Check(Native.Physics_DiagnosticsGet(Handle, &diagnostics));
		return diagnostics;
	}

	/// <summary>Return the exact byte count required for an opaque restart checkpoint.</summary>
	public int CheckpointSize()
	{
		Native.Check(Native.Physics_CheckpointSize(Handle, out var required));
		return checked((int)required);
	}

	/// <summary>Write a versioned checkpoint into a caller-owned buffer.</summary>
	public unsafe int WriteCheckpoint(Span<byte> destination)
	{
		EnsureOwner();
		fixed (byte* destination_ptr = destination)
		{
			Native.Check(Native.Physics_CheckpointWrite(Handle, destination_ptr, (ulong)destination.Length, out var written));
			return checked((int)written);
		}
	}

	/// <summary>Restore a checkpoint into an empty engine while preserving checkpoint object identities.</summary>
	public unsafe void ReadCheckpoint(ReadOnlySpan<byte> checkpoint)
	{
		EnsureOwner();
		if (m_shapes.Count != 0 || m_bodies.Count != 0 || m_articulations.Count != 0 || m_constraints.Count != 0)
			throw new InvalidOperationException("Checkpoint restore requires an engine with no managed object wrappers.");

		fixed (byte* checkpoint_ptr = checkpoint)
			Native.Check(Native.Physics_CheckpointRead(Handle, checkpoint_ptr, (ulong)checkpoint.Length));
	}

	/// <summary>Dispose bodies, shapes, pending GPU work, and the native device in dependency order.</summary>
	public void Dispose()
	{
		if (m_handle == null)
			return;

		EnsureOwner();
		var bodies = new RigidBody[m_bodies.Count];
		m_bodies.CopyTo(bodies);
		var articulations = new Articulation[m_articulations.Count];
		m_articulations.CopyTo(articulations);
		var constraints = new PersistentConstraint[m_constraints.Count];
		m_constraints.CopyTo(constraints);
		var shapes = new Shape[m_shapes.Count];
		m_shapes.CopyTo(shapes);

		// A terminal native cleanup failure still retires the engine, so preserve it while invalidating every managed identity exactly once.
		var destroy_status = Native.Physics_EngineDestroy(Handle);
		ExceptionDispatchInfo? destroy_failure = null;
		switch (destroy_status)
		{
			case EStatus.Success:
			{
				break;
			}
			case EStatus.DeviceRemoved:
			case EStatus.InternalError:
			{
				try
				{
					Native.Check(destroy_status);
				}
				catch (PhysicsException exception)
				{
					destroy_failure = ExceptionDispatchInfo.Capture(exception);
				}
				break;
			}
			default:
			{
				Native.Check(destroy_status);
				throw new InvalidOperationException($"Unexpected successful engine-destroy status {destroy_status}.");
			}
		}

		// Native destruction owns dependency ordering, including checkpoint-only objects.
		foreach (var constraint in constraints)
			constraint.ReleaseFromEngine();
		foreach (var articulation in articulations)
			articulation.ReleaseFromEngine();
		foreach (var body in bodies)
			body.ReleaseFromEngine();
		foreach (var shape in shapes)
			shape.ReleaseFromEngine();

		m_handle.MarkDestroyed();
		m_handle.Dispose();
		m_handle = null;
		m_runtime.Remove(this);
		GC.SuppressFinalize(this);
		destroy_failure?.Throw();
	}

	/// <summary>The current stable native engine identity.</summary>
	internal ulong Handle
	{
		get
		{
			var handle = m_handle ?? throw new ObjectDisposedException(nameof(Engine));
			return handle.Value;
		}
	}

	/// <summary>Throw if a mutable or lifetime operation runs on a non-owner OS thread.</summary>
	internal void EnsureOwner()
	{
		if (Native.GetCurrentThreadId() != m_owner_thread_id)
			throw new InvalidOperationException("Physics mutations and lifetime operations must run on the engine's creating OS thread.");
		if (IsDisposed)
			throw new ObjectDisposedException(nameof(Engine));
	}

	/// <summary>Remove a disposed shape from managed ownership tracking.</summary>
	internal void Remove(Shape shape)
	{
		m_shapes.Remove(shape);
	}

	/// <summary>Remove a disposed body from managed ownership tracking.</summary>
	internal void Remove(RigidBody body)
	{
		m_bodies.Remove(body);
	}

	/// <summary>Remove a disposed articulation from managed ownership tracking.</summary>
	internal void Remove(Articulation articulation)
	{
		m_articulations.Remove(articulation);
	}

	/// <summary>Remove a disposed persistent constraint from managed ownership tracking.</summary>
	internal void Remove(PersistentConstraint constraint)
	{
		m_constraints.Remove(constraint);
	}

	/// <summary>Resolve a stable body identity returned by native constraint state.</summary>
	internal RigidBody ResolveBody(BodyHandle handle)
	{
		foreach (var body in m_bodies)
		{
			if (body.Handle == handle)
				return body;
		}
		throw new InvalidOperationException("Native constraint state references a body without a managed owner.");
	}

	/// <summary>Resolve a stable articulation identity returned by native constraint state.</summary>
	internal Articulation ResolveArticulation(ArticulationHandle handle)
	{
		foreach (var articulation in m_articulations)
		{
			if (articulation.Handle == handle)
				return articulation;
		}
		throw new InvalidOperationException("Native constraint state references an articulation without a managed owner.");
	}

	/// <summary>Reject cross-engine constraint endpoints before native persistent storage changes.</summary>
	internal void ValidateConstraintEndpoints(D6ConstraintOptions options)
	{
		if (options.FrameA.OwnerEngine != null && !ReferenceEquals(options.FrameA.OwnerEngine, this))
			throw new ArgumentException("The first constraint endpoint belongs to a different engine.", nameof(options));
		if (options.FrameB.OwnerEngine != null && !ReferenceEquals(options.FrameB.OwnerEngine, this))
			throw new ArgumentException("The second constraint endpoint belongs to a different engine.", nameof(options));
	}

	/// <summary>Register a native shape handle as an owned object wrapper.</summary>
	private Shape AddShape(ulong handle)
	{
		var shape = new Shape(this, new ShapeHandle(handle));
		m_shapes.Add(shape);
		return shape;
	}

	/// <summary>Provides finalizer-safe cleanup without weakening public owner-thread disposal.</summary>
	private sealed class EngineSafeHandle :SafeHandle
	{
		private readonly Physics m_runtime;

		/// <summary>Adopt a native generation-aware engine handle.</summary>
		internal EngineSafeHandle(ulong handle, Physics runtime)
			: base(IntPtr.Zero, true)
		{
			if (IntPtr.Size != sizeof(ulong))
				throw new PlatformNotSupportedException("Rylogic.Physics requires a 64-bit process.");

			m_runtime = runtime;
			SetHandle(unchecked((IntPtr)(long)handle));
		}

		/// <inheritdoc/>
		public override bool IsInvalid
		{
			get
			{
				return handle == IntPtr.Zero;
			}
		}

		/// <summary>The typed unsigned engine identity.</summary>
		internal ulong Value
		{
			get
			{
				return unchecked((ulong)handle.ToInt64());
			}
		}

		/// <summary>Prevent finalizer cleanup after explicit owner-thread destruction succeeds.</summary>
		internal void MarkDestroyed()
		{
			SetHandleAsInvalid();
		}

		/// <inheritdoc/>
		protected override bool ReleaseHandle()
		{
			Native.Physics_EngineAbandon(Value);
			GC.KeepAlive(m_runtime);
			return true;
		}
	}
}
