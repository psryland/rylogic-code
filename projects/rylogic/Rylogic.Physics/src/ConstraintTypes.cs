using System;
using System.Runtime.InteropServices;
using Rylogic.Maths;

namespace Rylogic.Physics;

/// <summary>Selects the dynamics owner addressed by one persistent-constraint endpoint.</summary>
public enum EConstraintEndpoint
{
	World = 0,
	RigidBody = 1,
	ArticulationLink = 2,
}

/// <summary>Selects how one translational or rotational D6 coordinate contributes to the projected solve.</summary>
public enum EConstraintMode
{
	Free = 0,
	Locked = 1,
	Limited = 2,
	Driven = 3,
}

/// <summary>Controls persistent-constraint participation and connected-object collision policy.</summary>
[Flags]
public enum EConstraintFlags :uint
{
	None = 0,
	Enabled = 1U << 0,
	CollideConnected = 1U << 1,
}

/// <summary>An immutable generation-aware persistent-constraint identity with no native pointer semantics.</summary>
[StructLayout(LayoutKind.Sequential)]
public readonly struct PersistentConstraintHandle :IEquatable<PersistentConstraintHandle>
{
	private readonly ulong m_value;

	/// <summary>Initialise a typed handle from its stable binary identity.</summary>
	internal PersistentConstraintHandle(ulong value)
	{
		m_value = value;
	}

	/// <summary>True when this handle identifies no persistent constraint.</summary>
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
	public bool Equals(PersistentConstraintHandle other)
	{
		return m_value == other.m_value;
	}

	/// <inheritdoc/>
	public override bool Equals(object? obj)
	{
		return obj is PersistentConstraintHandle other && Equals(other);
	}

	/// <inheritdoc/>
	public override int GetHashCode()
	{
		return m_value.GetHashCode();
	}

	/// <summary>Compare two typed identities.</summary>
	public static bool operator ==(PersistentConstraintHandle lhs, PersistentConstraintHandle rhs)
	{
		return lhs.Equals(rhs);
	}

	/// <summary>Compare two typed identities.</summary>
	public static bool operator !=(PersistentConstraintHandle lhs, PersistentConstraintHandle rhs)
	{
		return !lhs.Equals(rhs);
	}
}

/// <summary>One endpoint-local constraint frame retaining its managed dynamics owner.</summary>
public readonly struct ConstraintFrame
{
	private readonly object? m_owner;

	/// <summary>Create an endpoint after its type, owner, link, and frame have been validated by a factory.</summary>
	private ConstraintFrame(EConstraintEndpoint type, object? owner, uint link_index, m4x4 constraint_to_body)
	{
		Type = type;
		m_owner = owner;
		LinkIndex = link_index;
		ConstraintToBody = constraint_to_body;
	}

	/// <summary>The dynamics owner addressed by this endpoint.</summary>
	public EConstraintEndpoint Type { get; }

	/// <summary>The topological articulation link index, or zero for world and rigid-body endpoints.</summary>
	public uint LinkIndex { get; }

	/// <summary>Transform from the constraint frame to the endpoint body or link frame.</summary>
	public m4x4 ConstraintToBody { get; }

	/// <summary>Create a frame fixed in world space.</summary>
	public static ConstraintFrame World(m4x4 constraint_to_world)
	{
		return new ConstraintFrame(EConstraintEndpoint.World, null, 0, constraint_to_world);
	}

	/// <summary>Create a frame attached to an engine-owned rigid body.</summary>
	public static ConstraintFrame ForBody(RigidBody body, m4x4 constraint_to_body)
	{
		if (body == null)
			throw new ArgumentNullException(nameof(body));

		return new ConstraintFrame(EConstraintEndpoint.RigidBody, body, 0, constraint_to_body);
	}

	/// <summary>Create a frame attached to one topologically indexed articulation link.</summary>
	public static ConstraintFrame ForLink(Articulation articulation, int link_index, m4x4 constraint_to_link)
	{
		if (articulation == null)
			throw new ArgumentNullException(nameof(articulation));
		if (link_index < 0)
			throw new ArgumentOutOfRangeException(nameof(link_index));

		return new ConstraintFrame(EConstraintEndpoint.ArticulationLink, articulation, checked((uint)link_index), constraint_to_link);
	}

	/// <summary>The owning engine for non-world endpoints.</summary>
	internal Engine? OwnerEngine
	{
		get
		{
			switch (Type)
			{
				case EConstraintEndpoint.World:
					{
						return null;
					}
				case EConstraintEndpoint.RigidBody:
					{
						return ((RigidBody)m_owner!).Engine;
					}
				case EConstraintEndpoint.ArticulationLink:
					{
						return ((Articulation)m_owner!).Engine;
					}
				default:
					{
						throw new ArgumentOutOfRangeException(nameof(Type), Type, "Unknown constraint endpoint type.");
					}
			}
		}
	}

	/// <summary>The stable native object identity for non-world endpoints.</summary>
	internal ulong ObjectHandle
	{
		get
		{
			switch (Type)
			{
				case EConstraintEndpoint.World:
					{
						return 0;
					}
				case EConstraintEndpoint.RigidBody:
					{
						return ((RigidBody)m_owner!).Handle.Value;
					}
				case EConstraintEndpoint.ArticulationLink:
					{
						return ((Articulation)m_owner!).Handle.Value;
					}
				default:
					{
						throw new ArgumentOutOfRangeException(nameof(Type), Type, "Unknown constraint endpoint type.");
					}
			}
		}
	}
}

/// <summary>Scalar D6 coordinate configuration in force units for linear rows and torque units for angular rows.</summary>
public readonly struct ConstraintAxis
{
	public readonly EConstraintMode m_mode;
	public readonly float m_lower_limit;
	public readonly float m_upper_limit;
	public readonly float m_target_position;
	public readonly float m_target_velocity;
	public readonly float m_stiffness;
	public readonly float m_damping;
	public readonly float m_max_force;

	/// <summary>Create a complete scalar constraint row configuration.</summary>
	public ConstraintAxis(EConstraintMode mode, float lower_limit, float upper_limit, float target_position, float target_velocity, float stiffness, float damping, float max_force)
	{
		m_mode = mode;
		m_lower_limit = lower_limit;
		m_upper_limit = upper_limit;
		m_target_position = target_position;
		m_target_velocity = target_velocity;
		m_stiffness = stiffness;
		m_damping = damping;
		m_max_force = max_force;
	}

	/// <summary>An unconstrained coordinate with finite drive parameters and unbounded limits.</summary>
	public static ConstraintAxis Free
	{
		get
		{
			return new ConstraintAxis(EConstraintMode.Free, float.NegativeInfinity, float.PositiveInfinity, 0.0f, 0.0f, 0.0f, 0.0f, float.PositiveInfinity);
		}
	}

	/// <summary>Create a hard equality row with a bounded applied force or torque.</summary>
	public static ConstraintAxis Locked(float max_force = float.PositiveInfinity)
	{
		return new ConstraintAxis(EConstraintMode.Locked, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, max_force);
	}

	/// <summary>Create a coordinate constrained to an inclusive scalar interval.</summary>
	public static ConstraintAxis Limited(float lower_limit, float upper_limit, float max_force = float.PositiveInfinity)
	{
		return new ConstraintAxis(EConstraintMode.Limited, lower_limit, upper_limit, 0.0f, 0.0f, 0.0f, 0.0f, max_force);
	}

	/// <summary>Create a position/velocity drive with explicit stiffness, damping, and effort bound.</summary>
	public static ConstraintAxis Driven(float target_position, float target_velocity, float stiffness, float damping, float max_force)
	{
		return new ConstraintAxis(EConstraintMode.Driven, float.NegativeInfinity, float.PositiveInfinity, target_position, target_velocity, stiffness, damping, max_force);
	}
}

/// <summary>Complete mutable declaration for one engine-owned persistent six-degree-of-freedom constraint.</summary>
public sealed class D6ConstraintOptions
{
	private readonly ConstraintAxis[] m_linear;
	private readonly ConstraintAxis[] m_angular;

	/// <summary>Create a free D6 constraint between two stable endpoint frames.</summary>
	public D6ConstraintOptions(ConstraintFrame frame_a, ConstraintFrame frame_b)
	{
		FrameA = frame_a;
		FrameB = frame_b;
		m_linear = new[] { ConstraintAxis.Free, ConstraintAxis.Free, ConstraintAxis.Free };
		m_angular = new[] { ConstraintAxis.Free, ConstraintAxis.Free, ConstraintAxis.Free };
	}

	/// <summary>The first endpoint-local frame.</summary>
	public ConstraintFrame FrameA { get; }

	/// <summary>The second endpoint-local frame.</summary>
	public ConstraintFrame FrameB { get; }

	/// <summary>Three translational rows ordered by the constraint frame's X, Y, and Z axes.</summary>
	public ConstraintAxis[] Linear
	{
		get
		{
			return m_linear;
		}
	}

	/// <summary>Three rotational rows ordered by the constraint frame's X, Y, and Z axes.</summary>
	public ConstraintAxis[] Angular
	{
		get
		{
			return m_angular;
		}
	}

	/// <summary>Resultant force threshold that permanently latches the constraint broken.</summary>
	public float BreakForce { get; set; } = float.PositiveInfinity;

	/// <summary>Resultant torque threshold that permanently latches the constraint broken.</summary>
	public float BreakTorque { get; set; } = float.PositiveInfinity;

	/// <summary>Participation and connected-object collision policy.</summary>
	public EConstraintFlags Flags { get; set; } = EConstraintFlags.Enabled;

	/// <summary>Create a fully locked six-degree-of-freedom weld between two frames.</summary>
	public static D6ConstraintOptions Weld(ConstraintFrame frame_a, ConstraintFrame frame_b)
	{
		var result = new D6ConstraintOptions(frame_a, frame_b);
		for (var i = 0; i != 3; ++i)
		{
			result.Linear[i] = ConstraintAxis.Locked();
			result.Angular[i] = ConstraintAxis.Locked();
		}
		return result;
	}
}

/// <summary>Current D6 declaration and sticky overload state for a persistent constraint.</summary>
public readonly struct PersistentConstraintState
{
	/// <summary>Create a caller-owned state value from native constraint output.</summary>
	internal PersistentConstraintState(D6ConstraintOptions options, bool broken)
	{
		Options = options;
		Broken = broken;
	}

	/// <summary>The current complete D6 declaration.</summary>
	public D6ConstraintOptions Options { get; }

	/// <summary>True after a break threshold was exceeded and before explicit repair.</summary>
	public bool Broken { get; }
}
