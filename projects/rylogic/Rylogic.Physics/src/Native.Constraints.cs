using System;
using System.Runtime.InteropServices;
using Rylogic.Maths;

namespace Rylogic.Physics;

/// <summary>Persistent D6 constraint records and entry points in the versioned native Physics ABI.</summary>
internal static unsafe partial class Native
{
	[StructLayout(LayoutKind.Sequential)]
	internal struct ConstraintFrame
	{
		internal EConstraintEndpoint m_type;
		internal uint m_link_index;
		internal ulong m_object_handle;
		internal m4x4 m_constraint_to_body;

		/// <summary>Convert one validated managed endpoint into its exact native frame record.</summary>
		internal static ConstraintFrame From(global::Rylogic.Physics.ConstraintFrame frame)
		{
			return new ConstraintFrame
			{
				m_type = frame.Type,
				m_link_index = frame.LinkIndex,
				m_object_handle = frame.ObjectHandle,
				m_constraint_to_body = frame.ConstraintToBody,
			};
		}

		/// <summary>Resolve one native endpoint back to the owning managed object wrapper.</summary>
		internal global::Rylogic.Physics.ConstraintFrame ToPublic(Engine engine)
		{
			switch (m_type)
			{
				case EConstraintEndpoint.World:
				{
					return global::Rylogic.Physics.ConstraintFrame.World(m_constraint_to_body);
				}
				case EConstraintEndpoint.RigidBody:
				{
					return global::Rylogic.Physics.ConstraintFrame.ForBody(engine.ResolveBody(new BodyHandle(m_object_handle)), m_constraint_to_body);
				}
				case EConstraintEndpoint.ArticulationLink:
				{
					return global::Rylogic.Physics.ConstraintFrame.ForLink(engine.ResolveArticulation(new ArticulationHandle(m_object_handle)), checked((int)m_link_index), m_constraint_to_body);
				}
				default:
				{
					throw new ArgumentOutOfRangeException(nameof(m_type), m_type, "Unknown constraint endpoint type.");
				}
			}
		}
	}

	[StructLayout(LayoutKind.Sequential)]
	internal struct ConstraintAxis
	{
		internal EConstraintMode m_mode;
		internal float m_lower_limit;
		internal float m_upper_limit;
		internal float m_target_position;
		internal float m_target_velocity;
		internal float m_stiffness;
		internal float m_damping;
		internal float m_max_force;

		/// <summary>Convert one managed scalar row configuration to native force or torque units.</summary>
		internal static ConstraintAxis From(global::Rylogic.Physics.ConstraintAxis axis)
		{
			return new ConstraintAxis
			{
				m_mode = axis.m_mode,
				m_lower_limit = axis.m_lower_limit,
				m_upper_limit = axis.m_upper_limit,
				m_target_position = axis.m_target_position,
				m_target_velocity = axis.m_target_velocity,
				m_stiffness = axis.m_stiffness,
				m_damping = axis.m_damping,
				m_max_force = axis.m_max_force,
			};
		}

		/// <summary>Convert one native scalar row configuration to its caller-facing value.</summary>
		internal global::Rylogic.Physics.ConstraintAxis ToPublic()
		{
			return new global::Rylogic.Physics.ConstraintAxis(m_mode, m_lower_limit, m_upper_limit, m_target_position, m_target_velocity, m_stiffness, m_damping, m_max_force);
		}
	}

	[StructLayout(LayoutKind.Sequential)]
	internal struct D6Constraint
	{
		internal NativeHeader m_header;
		internal ConstraintFrame m_frame_a;
		internal ConstraintFrame m_frame_b;
		internal ConstraintAxis m_linear0;
		internal ConstraintAxis m_linear1;
		internal ConstraintAxis m_linear2;
		internal ConstraintAxis m_angular0;
		internal ConstraintAxis m_angular1;
		internal ConstraintAxis m_angular2;
		internal float m_break_force;
		internal float m_break_torque;
		internal EConstraintFlags m_flags;
		private uint m_reserved;

		/// <summary>Convert a complete managed D6 declaration into its exact native fixed layout.</summary>
		internal static D6Constraint From(D6ConstraintOptions options)
		{
			return new D6Constraint
			{
				m_header = NativeHeader.Create<D6Constraint>(),
				m_frame_a = ConstraintFrame.From(options.FrameA),
				m_frame_b = ConstraintFrame.From(options.FrameB),
				m_linear0 = ConstraintAxis.From(options.Linear[0]),
				m_linear1 = ConstraintAxis.From(options.Linear[1]),
				m_linear2 = ConstraintAxis.From(options.Linear[2]),
				m_angular0 = ConstraintAxis.From(options.Angular[0]),
				m_angular1 = ConstraintAxis.From(options.Angular[1]),
				m_angular2 = ConstraintAxis.From(options.Angular[2]),
				m_break_force = options.BreakForce,
				m_break_torque = options.BreakTorque,
				m_flags = options.Flags,
			};
		}

		/// <summary>Convert a native D6 declaration into an independently mutable managed option set.</summary>
		internal D6ConstraintOptions ToPublic(Engine engine)
		{
			var result = new D6ConstraintOptions(m_frame_a.ToPublic(engine), m_frame_b.ToPublic(engine))
			{
				BreakForce = m_break_force,
				BreakTorque = m_break_torque,
				Flags = m_flags,
			};
			result.Linear[0] = m_linear0.ToPublic();
			result.Linear[1] = m_linear1.ToPublic();
			result.Linear[2] = m_linear2.ToPublic();
			result.Angular[0] = m_angular0.ToPublic();
			result.Angular[1] = m_angular1.ToPublic();
			result.Angular[2] = m_angular2.ToPublic();
			return result;
		}
	}

	[DllImport(Dll)] internal static extern EStatus Physics_ConstraintCreateD6(ulong engine, D6Constraint* desc, out ulong constraint);
	[DllImport(Dll)] internal static extern EStatus Physics_ConstraintGetD6(ulong engine, ulong constraint, D6Constraint* desc, out int broken);
	[DllImport(Dll)] internal static extern EStatus Physics_ConstraintUpdateD6(ulong engine, ulong constraint, D6Constraint* desc);
	[DllImport(Dll)] internal static extern EStatus Physics_ConstraintSetEnabled(ulong engine, ulong constraint, int enabled);
	[DllImport(Dll)] internal static extern EStatus Physics_ConstraintRepair(ulong engine, ulong constraint);
	[DllImport(Dll)] internal static extern EStatus Physics_ConstraintDestroy(ulong engine, ulong constraint);
}
