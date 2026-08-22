using System;
using System.Runtime.InteropServices;
using Rylogic.Maths;

namespace Rylogic.Physics;

/// <summary>Identifies the lifetime and integration behaviour of a rigid body.</summary>
public enum EMotionType
{
	Static = 0,
	Dynamic = 1,
	Kinematic = 2,
}

/// <summary>Selects how a dynamic body's mass properties are established.</summary>
public enum EMassMode
{
	ExplicitInertia = 0,
	Mass = 1,
	Density = 2,
}

/// <summary>Identifies a bulk body mutation applied on the engine owner thread.</summary>
public enum EBodyCommand
{
	SetTransform = 0,
	SetVelocity = 1,
	SetMomentum = 2,
	SetForce = 3,
	ApplyForce = 4,
	ApplyImpulse = 5,
	SetGravity = 6,
	SetKinematicTransform = 7,
	SetEnabled = 8,
	Wake = 9,
	Sleep = 10,
}

/// <summary>Identifies a buffered event produced by a completed step.</summary>
public enum EPhysicsEvent
{
	Contact = 0,
	Wake = 1,
	Sleep = 2,
}

/// <summary>Flags represented in body state, commands, and snapshots.</summary>
[Flags]
public enum EBodyFlags :uint
{
	None = 0,
	Enabled = 1U << 0,
	Sleeping = 1U << 1,
	NeverSleep = 1U << 2,
}

/// <summary>An immutable generation-aware shape identity with no native pointer semantics.</summary>
[StructLayout(LayoutKind.Sequential)]
public readonly struct ShapeHandle :IEquatable<ShapeHandle>
{
	private readonly ulong m_value;

	/// <summary>Initialise a typed handle from its stable binary identity.</summary>
	internal ShapeHandle(ulong value)
	{
		m_value = value;
	}

	/// <summary>True when this handle identifies no shape.</summary>
	public bool IsNull
	{
		get
		{
			return m_value == 0;
		}
	}

	/// <summary>The fixed-width value passed to the native ABI.</summary>
	internal ulong Value
	{
		get
		{
			return m_value;
		}
	}

	/// <inheritdoc/>
	public bool Equals(ShapeHandle other)
	{
		return m_value == other.m_value;
	}

	/// <inheritdoc/>
	public override bool Equals(object? obj)
	{
		return obj is ShapeHandle other && Equals(other);
	}

	/// <inheritdoc/>
	public override int GetHashCode()
	{
		return m_value.GetHashCode();
	}

	/// <summary>Compare two typed identities.</summary>
	public static bool operator ==(ShapeHandle lhs, ShapeHandle rhs)
	{
		return lhs.Equals(rhs);
	}

	/// <summary>Compare two typed identities.</summary>
	public static bool operator !=(ShapeHandle lhs, ShapeHandle rhs)
	{
		return !lhs.Equals(rhs);
	}
}

/// <summary>An immutable generation-aware body identity with no native pointer semantics.</summary>
[StructLayout(LayoutKind.Sequential)]
public readonly struct BodyHandle :IEquatable<BodyHandle>
{
	private readonly ulong m_value;

	/// <summary>Initialise a typed handle from its stable binary identity.</summary>
	internal BodyHandle(ulong value)
	{
		m_value = value;
	}

	/// <summary>True when this handle identifies no body.</summary>
	public bool IsNull
	{
		get
		{
			return m_value == 0;
		}
	}

	/// <summary>The fixed-width value passed to the native ABI.</summary>
	internal ulong Value
	{
		get
		{
			return m_value;
		}
	}

	/// <inheritdoc/>
	public bool Equals(BodyHandle other)
	{
		return m_value == other.m_value;
	}

	/// <inheritdoc/>
	public override bool Equals(object? obj)
	{
		return obj is BodyHandle other && Equals(other);
	}

	/// <inheritdoc/>
	public override int GetHashCode()
	{
		return m_value.GetHashCode();
	}

	/// <summary>Compare two typed identities.</summary>
	public static bool operator ==(BodyHandle lhs, BodyHandle rhs)
	{
		return lhs.Equals(rhs);
	}

	/// <summary>Compare two typed identities.</summary>
	public static bool operator !=(BodyHandle lhs, BodyHandle rhs)
	{
		return !lhs.Equals(rhs);
	}
}

/// <summary>An angular and linear spatial vector represented in one coordinate frame.</summary>
[StructLayout(LayoutKind.Sequential)]
public readonly struct SpatialVector
{
	public readonly v4 m_angular;
	public readonly v4 m_linear;

	/// <summary>Create a spatial vector from angular and linear components.</summary>
	public SpatialVector(v4 angular, v4 linear)
	{
		m_angular = angular;
		m_linear = linear;
	}

	/// <summary>The zero spatial vector.</summary>
	public static SpatialVector Zero
	{
		get
		{
			return new SpatialVector(v4.Zero, v4.Zero);
		}
	}
}

/// <summary>Compact rigid-body inertia expressed at a centre of mass.</summary>
[StructLayout(LayoutKind.Sequential)]
public readonly struct BodyInertia
{
	public readonly v4 m_diagonal;
	public readonly v4 m_products;
	public readonly v4 m_centre_of_mass_and_mass;

	/// <summary>Create explicit unit-inertia terms, centre of mass, and mass.</summary>
	public BodyInertia(v4 diagonal, v4 products, v4 centre_of_mass, float mass)
	{
		m_diagonal = diagonal;
		m_products = products;
		m_centre_of_mass_and_mass = new v4(centre_of_mass.xyz, mass);
	}
}

/// <summary>Common transform, material, and collision flags for a shape.</summary>
public sealed class ShapeOptions
{
	/// <summary>Transform from this shape directly to the model root.</summary>
	public m4x4 ShapeToRoot { get; set; } = m4x4.Identity;

	/// <summary>Stable native material identifier.</summary>
	public int MaterialId { get; set; }

	/// <summary>Native collision flags reserved for engine-defined shape behaviour.</summary>
	public uint Flags { get; set; }
}

/// <summary>Configuration used when creating or updating an engine.</summary>
public sealed class EngineOptions
{
	public int MaxCollisionPairs { get; set; } = 65536;
	public bool SleepingEnabled { get; set; } = true;
	public float SleepVelocityThresholdLinear { get; set; } = 0.25f;
	public float SleepVelocityThresholdAngular { get; set; } = 1.50f;
	public float SleepDelaySeconds { get; set; } = 1.0f;
	public int SolverIterations { get; set; } = 8;
	public int PushOutIterations { get; set; } = 4;
	public float BroadphaseAabbMargin { get; set; } = 0.0001f;
	public float ContactSortPropagationScale { get; set; } = 1e-3f;
	public int ContactSortShockIterations { get; set; } = 4;
	public float ContactSortShockAlignment { get; set; } = 1e-5f;
	public float ContactSortShockMinStrength { get; set; } = 1e-5f;
	public float ContactSortShockDecay { get; set; } = 0.98f;
	public float PenetrationSlop { get; set; } = 0.005f;
	public float VelocityBaumgarte { get; set; } = 0.2f;
	public float PositionSlop { get; set; } = 0.005f;
	public float PositionBaumgarte { get; set; } = 0.2f;
	public float ContactSlopScale { get; set; } = 0.01f;
	public float SupportContactSlopScale { get; set; } = 0.005f;
	public float WarmStartScale { get; set; } = 0.90f;
	public float DeepPenetrationThreshold { get; set; } = 0.3f;
	public float DeepPenetrationRange { get; set; } = 0.4f;
	public float DeepPenetrationBaumgarteMin { get; set; } = 0.2f;
	public float DeepPenetrationBaumgarteMax { get; set; } = 0.8f;
	public int SelectiveRefreshPasses { get; set; } = 1;
	public int SelectiveRefreshMaxPairs { get; set; } = 512;
	public int SelectiveRefreshBodyLimit { get; set; } = 256;
	public int SelectiveRefreshContactLimit { get; set; } = 512;
	public int SelectiveRefreshSolverIterations { get; set; } = 12;
	public int SelectiveRefreshPositionIterations { get; set; } = 1;
	public float SelectiveRefreshBiasScale { get; set; } = 1.0f;
	public float SelectiveRefreshRestitutionScale { get; set; }
	public int SelectiveRefreshAdaptiveBodyLimit { get; set; } = 256;
	public int SelectiveRefreshAdaptiveSolverIterations { get; set; } = 48;
	public bool SelectiveRefreshSupportOnly { get; set; } = true;
	public bool SelectiveRefreshResolveSupportOnly { get; set; }
	public float SelectiveRefreshDepthSlop { get; set; } = 0.015f;
	public float SelectiveRefreshSupportDepthSlop { get; set; } = 0.002f;
	public float SelectiveRefreshClosingSpeedSlop { get; set; } = 0.02f;
	public float SelectiveRefreshSupportAlignment { get; set; } = 0.65f;
	public float SelectiveRefreshAabbMargin { get; set; } = 0.03f;
}

/// <summary>Material properties consumed by native contact resolution.</summary>
[StructLayout(LayoutKind.Sequential)]
public readonly struct Material
{
	public readonly int m_id;
	public readonly float m_static_friction;
	public readonly float m_normal_elasticity;
	public readonly float m_tangential_elasticity;
	public readonly float m_torsional_elasticity;
	public readonly float m_density;

	/// <summary>Create a material with stable identity and physical properties.</summary>
	public Material(int id, float static_friction, float normal_elasticity, float tangential_elasticity, float torsional_elasticity, float density)
	{
		m_id = id;
		m_static_friction = static_friction;
		m_normal_elasticity = normal_elasticity;
		m_tangential_elasticity = tangential_elasticity;
		m_torsional_elasticity = torsional_elasticity;
		m_density = density;
	}
}

/// <summary>Configuration and initial state for a new rigid body.</summary>
public sealed class BodyOptions
{
	public m4x4 ObjectToWorld { get; set; } = m4x4.Identity;
	public BodyInertia Inertia { get; set; }
	public SpatialVector Momentum { get; set; } = SpatialVector.Zero;
	public v4 Gravity { get; set; } = new v4(0, 0, -9.81f, 0);
	public ulong UserTag { get; set; }
	public EMotionType MotionType { get; set; } = EMotionType.Dynamic;
	public EMassMode MassMode { get; set; } = EMassMode.Density;
	public float MassOrDensity { get; set; } = 1000.0f;
	public EBodyFlags Flags { get; set; } = EBodyFlags.Enabled;
}

/// <summary>A fixed-layout bulk body mutation.</summary>
[StructLayout(LayoutKind.Sequential)]
public readonly struct BodyCommand
{
	private readonly NativeHeader m_header;
	public readonly BodyHandle m_body;
	public readonly EBodyCommand m_type;
	public readonly uint m_flags;
	public readonly m4x4 m_transform;
	public readonly SpatialVector m_value;
	public readonly v4 m_at;

	/// <summary>Create a command containing all possible fixed-layout payload fields.</summary>
	public BodyCommand(BodyHandle body, EBodyCommand type, m4x4 transform, SpatialVector value, v4 at, uint flags = 0)
	{
		m_header = NativeHeader.Create<BodyCommand>();
		m_body = body;
		m_type = type;
		m_flags = flags;
		m_transform = transform;
		m_value = value;
		m_at = at;
	}

	/// <summary>Create a world-transform replacement command.</summary>
	public static BodyCommand SetTransform(BodyHandle body, m4x4 object_to_world)
	{
		return new BodyCommand(body, EBodyCommand.SetTransform, object_to_world, SpatialVector.Zero, v4.Zero);
	}

	/// <summary>Create a kinematic world-transform replacement command.</summary>
	public static BodyCommand SetKinematicTransform(BodyHandle body, m4x4 object_to_world)
	{
		return new BodyCommand(body, EBodyCommand.SetKinematicTransform, object_to_world, SpatialVector.Zero, v4.Zero);
	}

	/// <summary>Create a world-space angular and linear velocity replacement command.</summary>
	public static BodyCommand SetVelocity(BodyHandle body, SpatialVector velocity)
	{
		return new BodyCommand(body, EBodyCommand.SetVelocity, m4x4.Identity, velocity, v4.Zero);
	}

	/// <summary>Create a world-space angular and linear momentum replacement command.</summary>
	public static BodyCommand SetMomentum(BodyHandle body, SpatialVector momentum)
	{
		return new BodyCommand(body, EBodyCommand.SetMomentum, m4x4.Identity, momentum, v4.Zero);
	}

	/// <summary>Create a world-space torque and force replacement command.</summary>
	public static BodyCommand SetForce(BodyHandle body, SpatialVector force)
	{
		return new BodyCommand(body, EBodyCommand.SetForce, m4x4.Identity, force, v4.Zero);
	}

	/// <summary>Create a world-space torque and force accumulation command at a world-oriented offset from the model origin.</summary>
	public static BodyCommand ApplyForce(BodyHandle body, SpatialVector force, v4 at)
	{
		if (at.w != 0.0f)
			throw new ArgumentException("The force application point must be an offset from the model origin.", nameof(at));

		return new BodyCommand(body, EBodyCommand.ApplyForce, m4x4.Identity, force, at);
	}

	/// <summary>Create a world-space angular and linear impulse command.</summary>
	public static BodyCommand ApplyImpulse(BodyHandle body, SpatialVector impulse)
	{
		return new BodyCommand(body, EBodyCommand.ApplyImpulse, m4x4.Identity, impulse, v4.Zero);
	}

	/// <summary>Create a gravity replacement command.</summary>
	public static BodyCommand SetGravity(BodyHandle body, v4 gravity)
	{
		return new BodyCommand(body, EBodyCommand.SetGravity, m4x4.Identity, new SpatialVector(v4.Zero, gravity), v4.Zero);
	}

	/// <summary>Create a body participation toggle command.</summary>
	public static BodyCommand SetEnabled(BodyHandle body, bool enabled)
	{
		return new BodyCommand(body, EBodyCommand.SetEnabled, m4x4.Identity, SpatialVector.Zero, v4.Zero, enabled ? 1U : 0U);
	}

	/// <summary>Create an explicit wake command.</summary>
	public static BodyCommand Wake(BodyHandle body)
	{
		return new BodyCommand(body, EBodyCommand.Wake, m4x4.Identity, SpatialVector.Zero, v4.Zero);
	}

	/// <summary>Create an explicit sleep command.</summary>
	public static BodyCommand Sleep(BodyHandle body)
	{
		return new BodyCommand(body, EBodyCommand.Sleep, m4x4.Identity, SpatialVector.Zero, v4.Zero);
	}
}

/// <summary>Caller-owned immutable state copied from one completed simulation step.</summary>
[StructLayout(LayoutKind.Sequential)]
public struct BodySnapshot
{
	private NativeHeader m_header;
	public BodyHandle m_body;
	public ShapeHandle m_shape;
	public m4x4 m_object_to_world;
	public SpatialVector m_momentum;
	public SpatialVector m_velocity;
	public ulong m_user_tag;
	public EMotionType m_motion_type;
	public EBodyFlags m_flags;
}

/// <summary>The complete mutable state of one rigid body outside a pending step.</summary>
public readonly struct RigidBodyState
{
	public readonly m4x4 m_object_to_world;
	public readonly BodyInertia m_inertia;
	public readonly SpatialVector m_momentum;
	public readonly SpatialVector m_velocity;
	public readonly SpatialVector m_force;
	public readonly v4 m_gravity;
	public readonly ulong m_user_tag;
	public readonly EMotionType m_motion_type;
	public readonly EBodyFlags m_flags;

	/// <summary>Create an immutable body-state value.</summary>
	public RigidBodyState(
		m4x4 object_to_world,
		BodyInertia inertia,
		SpatialVector momentum,
		SpatialVector velocity,
		SpatialVector force,
		v4 gravity,
		ulong user_tag,
		EMotionType motion_type,
		EBodyFlags flags)
	{
		m_object_to_world = object_to_world;
		m_inertia = inertia;
		m_momentum = momentum;
		m_velocity = velocity;
		m_force = force;
		m_gravity = gravity;
		m_user_tag = user_tag;
		m_motion_type = motion_type;
		m_flags = flags;
	}
}

/// <summary>A buffered contact or body lifecycle event copied after CompleteStep.</summary>
[StructLayout(LayoutKind.Sequential)]
public struct PhysicsEvent
{
	private NativeHeader m_header;
	public EPhysicsEvent m_type;
	public uint m_point_count;
	public BodyHandle m_body_a;
	public BodyHandle m_body_b;
	public v4 m_normal;
	public v4 m_point0;
	public v4 m_point1;
	public v4 m_point2;
	public v4 m_point3;
	public float m_depth;
	public int m_material_a;
	public int m_material_b;
	public uint m_child_a;
	public uint m_child_b;
}

/// <summary>Timing and capacity measurements for the most recently completed step.</summary>
[StructLayout(LayoutKind.Sequential)]
public struct StepProfile
{
	public double m_new_frame_ms;
	public double m_pack_ms;
	public double m_upload_ms;
	public double m_external_forces_ms;
	public double m_integrate_ms;
	public double m_sleep_wake_ms;
	public double m_broadphase_ms;
	public double m_collide_ms;
	public double m_resolve_ms;
	public double m_selective_ms;
	public double m_sleep_update_ms;
	public double m_readback_ms;
	public double m_gpu_run_ms;
	public double m_gpu_prepare_ms;
	public double m_gpu_execute_ms;
	public double m_gpu_wait_ms;
	public double m_gpu_reset_ms;
	public double m_unpack_ms;
}

/// <summary>Generic engine state, capacity, device, and deterministic replay diagnostics.</summary>
[StructLayout(LayoutKind.Sequential)]
public struct Diagnostics
{
	private NativeHeader m_header;
	public StepProfile m_profile;
	public ulong m_submitted_step;
	public ulong m_completed_step;
	public ulong m_state_checksum;
	public int m_body_count;
	public int m_shape_count;
	public int m_pair_count;
	public int m_contact_count;
	public int m_max_pairs;
	public int m_max_contacts;
	private int m_step_pending;
	public int m_device_removed_reason;

	/// <summary>True while a split step has submitted work that has not been completed.</summary>
	public bool StepPending
	{
		get
		{
			return m_step_pending != 0;
		}
	}
}

/// <summary>Base exception for failures reported by the native physics ABI.</summary>
public sealed class PhysicsException :Exception
{
	/// <summary>Create an exception for one native status and message.</summary>
	internal PhysicsException(EStatus status, string message)
		: base(message)
	{
		Status = status;
	}

	/// <summary>The stable native status code.</summary>
	public EStatus Status { get; }
}

[StructLayout(LayoutKind.Sequential)]
internal readonly struct NativeHeader
{
	internal readonly uint m_size;
	internal readonly uint m_version;

	internal NativeHeader(uint size)
	{
		m_size = size;
		m_version = Native.StructVersion;
	}

	internal static NativeHeader Create<T>()
	{
		return new NativeHeader(NativeSize<T>.Value);
	}
}

/// <summary>Caches one managed ABI record size so hot command construction performs no reflection.</summary>
internal static class NativeSize<T>
{
	internal static readonly uint Value = (uint)Marshal.SizeOf(typeof(T));
}

/// <summary>Stable result codes returned by the native Physics ABI.</summary>
public enum EStatus
{
	Success = 0,
	InvalidArgument = 1,
	InvalidStruct = 2,
	InvalidHandle = 3,
	StaleHandle = 4,
	WrongThread = 5,
	StepPending = 6,
	NoStepPending = 7,
	BufferTooSmall = 8,
	IncompatibleVersion = 9,
	DeviceRemoved = 10,
	InternalError = 11,
}
