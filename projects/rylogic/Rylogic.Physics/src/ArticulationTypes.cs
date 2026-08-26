using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using Rylogic.Maths;

namespace Rylogic.Physics;

/// <summary>Selects whether an articulation root is fixed to world or contributes a floating six-velocity base.</summary>
public enum EArticulationRoot
{
	Fixed = 0,
	Floating = 1,
}

/// <summary>Selects the scalar screw motion represented by one reduced articulation coordinate.</summary>
public enum EArticulationAxis
{
	Revolute = 0,
	Prismatic = 1,
}

/// <summary>Controls articulation participation and whole-tree sleeping.</summary>
[Flags]
public enum EArticulationFlags :uint
{
	None = 0,
	Enabled = 1U << 0,
	Sleeping = 1U << 1,
	NeverSleep = 1U << 2,
}

/// <summary>Controls immutable collision policy for one articulation link.</summary>
[Flags]
public enum EArticulationLinkFlags :uint
{
	None = 0,
	CollideParent = 1U << 0,
	CollideSelf = 1U << 1,
}

/// <summary>An immutable generation-aware articulation identity with no native pointer semantics.</summary>
[StructLayout(LayoutKind.Sequential)]
public readonly struct ArticulationHandle :IEquatable<ArticulationHandle>
{
	private readonly ulong m_value;

	/// <summary>Initialise a typed handle from its stable binary identity.</summary>
	internal ArticulationHandle(ulong value)
	{
		m_value = value;
	}

	/// <summary>True when this handle identifies no articulation.</summary>
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
	public bool Equals(ArticulationHandle other)
	{
		return m_value == other.m_value;
	}

	/// <inheritdoc/>
	public override bool Equals(object? obj)
	{
		return obj is ArticulationHandle other && Equals(other);
	}

	/// <inheritdoc/>
	public override int GetHashCode()
	{
		return m_value.GetHashCode();
	}

	/// <summary>Compare two typed identities.</summary>
	public static bool operator ==(ArticulationHandle lhs, ArticulationHandle rhs)
	{
		return lhs.Equals(rhs);
	}

	/// <summary>Compare two typed identities.</summary>
	public static bool operator !=(ArticulationHandle lhs, ArticulationHandle rhs)
	{
		return !lhs.Equals(rhs);
	}
}

/// <summary>Whole-tree transform, velocity, identity, and participation used when creating an articulation.</summary>
public sealed class ArticulationOptions
{
	/// <summary>Initial transform from the root link to world.</summary>
	public m4x4 RootToWorld { get; set; } = m4x4.Identity;

	/// <summary>Initial floating-root spatial velocity; fixed roots require zero velocity.</summary>
	public SpatialVector RootVelocity { get; set; } = SpatialVector.Zero;

	/// <summary>Caller-owned value preserved with articulation state.</summary>
	public ulong UserTag { get; set; }

	/// <summary>Whether the root is fixed or contributes a floating base.</summary>
	public EArticulationRoot RootType { get; set; } = EArticulationRoot.Fixed;

	/// <summary>Whole-tree participation and sleep policy.</summary>
	public EArticulationFlags Flags { get; set; } = EArticulationFlags.Enabled;
}

/// <summary>Immutable shape, mass, parent, and collision policy for one topologically ordered articulation link.</summary>
public sealed class ArticulationLinkOptions
{
	/// <summary>Create a link declaration for an engine-owned shape and an earlier parent index, or -1 for the root.</summary>
	public ArticulationLinkOptions(Shape shape, int parent_index)
	{
		Shape = shape ?? throw new ArgumentNullException(nameof(shape));
		ParentIndex = parent_index;
	}

	/// <summary>The collision shape retained for the articulation lifetime.</summary>
	public Shape Shape { get; }

	/// <summary>Mass properties expressed in the link frame.</summary>
	public BodyInertia Inertia { get; set; }

	/// <summary>Transform from the collision shape to the link frame.</summary>
	public m4x4 ShapeToLink { get; set; } = m4x4.Identity;

	/// <summary>Index of an earlier parent link, or -1 for the single root declaration.</summary>
	public int ParentIndex { get; }

	/// <summary>Immutable collision policy for this link.</summary>
	public EArticulationLinkFlags Flags { get; set; }
}

/// <summary>One ordered reduced coordinate in an articulation joint.</summary>
public readonly struct ArticulationAxis
{
	public readonly v4 m_axis;
	public readonly EArticulationAxis m_type;
	public readonly float m_initial_position;
	public readonly float m_initial_velocity;

	/// <summary>Create a normalized revolute or prismatic coordinate with initial scalar state.</summary>
	public ArticulationAxis(v4 axis, EArticulationAxis type, float initial_position = 0.0f, float initial_velocity = 0.0f)
	{
		m_axis = axis;
		m_type = type;
		m_initial_position = initial_position;
		m_initial_velocity = initial_velocity;
	}
}

/// <summary>Immutable joint frames and ordered reduced coordinates joining one non-root link to its parent.</summary>
public sealed class ArticulationJointOptions
{
	private readonly ArticulationAxis[] m_axes;

	/// <summary>Create a joint from parent/child frames and up to six ordered reduced coordinates.</summary>
	public ArticulationJointOptions(m4x4 joint_to_parent, m4x4 joint_to_child, params ArticulationAxis[] axes)
	{
		JointToParent = joint_to_parent;
		JointToChild = joint_to_child;
		m_axes = axes != null ? (ArticulationAxis[])axes.Clone() : throw new ArgumentNullException(nameof(axes));
		if (m_axes.Length > 6)
			throw new ArgumentException("An articulation joint supports at most six reduced coordinates.", nameof(axes));
	}

	/// <summary>Transform from the joint frame to the parent link.</summary>
	public m4x4 JointToParent { get; }

	/// <summary>Transform from the joint frame to the child link.</summary>
	public m4x4 JointToChild { get; }

	/// <summary>Reduced coordinates in their stable flattened state order.</summary>
	public IReadOnlyList<ArticulationAxis> Axes
	{
		get
		{
			return m_axes;
		}
	}
}

/// <summary>Mutable whole-tree state and flattened reduced-coordinate streams.</summary>
public sealed class ArticulationState
{
	/// <summary>Create one complete state value returned by an articulation.</summary>
	internal ArticulationState(ArticulationHandle articulation, m4x4 root_to_world, SpatialVector root_velocity, SpatialVector root_force, ulong user_tag, int link_count, EArticulationFlags flags, float[] positions, float[] velocities, float[] accelerations, float[] forces)
	{
		Articulation = articulation;
		RootToWorld = root_to_world;
		RootVelocity = root_velocity;
		RootForce = root_force;
		UserTag = user_tag;
		LinkCount = link_count;
		Flags = flags;
		Positions = positions;
		Velocities = velocities;
		Accelerations = accelerations;
		Forces = forces;
	}

	/// <summary>The stable articulation identity that owns this state.</summary>
	public ArticulationHandle Articulation { get; }

	/// <summary>Current root-link transform.</summary>
	public m4x4 RootToWorld { get; set; }

	/// <summary>Current floating-root spatial velocity.</summary>
	public SpatialVector RootVelocity { get; set; }

	/// <summary>Persistent floating-root external force.</summary>
	public SpatialVector RootForce { get; set; }

	/// <summary>Caller-owned value preserved with articulation state.</summary>
	public ulong UserTag { get; set; }

	/// <summary>Immutable number of links in the articulation topology.</summary>
	public int LinkCount { get; }

	/// <summary>Whole-tree participation and sleep policy.</summary>
	public EArticulationFlags Flags { get; set; }

	/// <summary>Flattened non-root joint coordinates in topology and axis declaration order.</summary>
	public float[] Positions { get; }

	/// <summary>Flattened non-root joint velocities in topology and axis declaration order.</summary>
	public float[] Velocities { get; }

	/// <summary>Most recently computed non-root joint accelerations.</summary>
	public float[] Accelerations { get; }

	/// <summary>Persistent non-root joint forces in topology and axis declaration order.</summary>
	public float[] Forces { get; }
}

/// <summary>Current derived kinematics and persistent external fields for one topologically indexed link.</summary>
public readonly struct ArticulationLinkState
{
	public readonly ArticulationHandle m_articulation;
	public readonly uint m_link_index;
	public readonly int m_parent_index;
	public readonly ShapeHandle m_shape;
	public readonly m4x4 m_link_to_world;
	public readonly SpatialVector m_velocity;
	public readonly SpatialVector m_acceleration;
	public readonly SpatialVector m_external_force;
	public readonly v4 m_gravity;

	/// <summary>Create a caller-owned link snapshot from one native output record.</summary>
	internal ArticulationLinkState(Native.ArticulationLinkState state)
	{
		m_articulation = state.m_articulation;
		m_link_index = state.m_link_index;
		m_parent_index = state.m_parent_index;
		m_shape = state.m_shape;
		m_link_to_world = state.m_link_to_world;
		m_velocity = state.m_velocity;
		m_acceleration = state.m_acceleration;
		m_external_force = state.m_external_force;
		m_gravity = state.m_gravity;
	}
}
