using System;
using System.Runtime.InteropServices;
using Rylogic.Maths;

namespace Rylogic.Physics;

/// <summary>Articulation records and entry points in the versioned native Physics ABI.</summary>
internal static unsafe partial class Native
{
	[StructLayout(LayoutKind.Sequential)]
	internal struct ArticulationDesc
	{
		internal NativeHeader m_header;
		internal m4x4 m_root_to_world;
		internal SpatialVector m_root_velocity;
		internal ulong m_user_tag;
		internal uint m_link_count;
		internal EArticulationRoot m_root_type;
		internal EArticulationFlags m_flags;
		private uint m_reserved;

		/// <summary>Convert managed creation options into an exact native topology header.</summary>
		internal static ArticulationDesc From(ArticulationOptions options, int link_count)
		{
			return new ArticulationDesc
			{
				m_header = NativeHeader.Create<ArticulationDesc>(),
				m_root_to_world = options.RootToWorld,
				m_root_velocity = options.RootVelocity,
				m_user_tag = options.UserTag,
				m_link_count = checked((uint)link_count),
				m_root_type = options.RootType,
				m_flags = options.Flags,
			};
		}
	}

	[StructLayout(LayoutKind.Sequential)]
	internal struct ArticulationLink
	{
		internal NativeHeader m_header;
		internal ShapeHandle m_shape;
		internal BodyInertia m_inertia;
		internal m4x4 m_shape_to_link;
		internal int m_parent_index;
		internal EArticulationLinkFlags m_flags;

		/// <summary>Convert one managed link declaration into its exact native topology record.</summary>
		internal static ArticulationLink From(ArticulationLinkOptions options)
		{
			return new ArticulationLink
			{
				m_header = NativeHeader.Create<ArticulationLink>(),
				m_shape = options.Shape.Handle,
				m_inertia = options.Inertia,
				m_shape_to_link = options.ShapeToLink,
				m_parent_index = options.ParentIndex,
				m_flags = options.Flags,
			};
		}
	}

	[StructLayout(LayoutKind.Sequential)]
	internal struct ArticulationJoint
	{
		internal NativeHeader m_header;
		internal m4x4 m_joint_to_parent;
		internal m4x4 m_joint_to_child;
		internal fixed float m_axes[24];
		internal fixed float m_initial_positions[6];
		internal fixed float m_initial_velocities[6];
		internal fixed int m_axis_types[6];
		internal uint m_dof_count;
		private uint m_reserved;

		/// <summary>Pack up to six reduced coordinates into the fixed native joint record.</summary>
		internal static ArticulationJoint From(ArticulationJointOptions options)
		{
			if (options.Axes.Count > 6)
				throw new ArgumentException("An articulation joint supports at most six reduced coordinates.", nameof(options));

			var result = new ArticulationJoint
			{
				m_header = NativeHeader.Create<ArticulationJoint>(),
				m_joint_to_parent = options.JointToParent,
				m_joint_to_child = options.JointToChild,
				m_dof_count = checked((uint)options.Axes.Count),
			};

			// Preserve declaration order because every scalar state array uses this flattened joint-axis order.
			for (var i = 0; i != options.Axes.Count; ++i)
			{
				var axis = options.Axes[i];
				result.m_axes[4 * i + 0] = axis.m_axis.x;
				result.m_axes[4 * i + 1] = axis.m_axis.y;
				result.m_axes[4 * i + 2] = axis.m_axis.z;
				result.m_axes[4 * i + 3] = axis.m_axis.w;
				result.m_initial_positions[i] = axis.m_initial_position;
				result.m_initial_velocities[i] = axis.m_initial_velocity;
				result.m_axis_types[i] = (int)axis.m_type;
			}
			return result;
		}
	}

	[StructLayout(LayoutKind.Sequential)]
	internal struct ArticulationState
	{
		internal NativeHeader m_header;
		internal ArticulationHandle m_articulation;
		internal m4x4 m_root_to_world;
		internal SpatialVector m_root_velocity;
		internal SpatialVector m_root_force;
		internal ulong m_user_tag;
		internal uint m_link_count;
		internal uint m_joint_dof_count;
		internal EArticulationFlags m_flags;
		private uint m_reserved;
	}

	[StructLayout(LayoutKind.Sequential)]
	internal struct ArticulationLinkState
	{
		internal NativeHeader m_header;
		internal ArticulationHandle m_articulation;
		internal uint m_link_index;
		internal int m_parent_index;
		internal ShapeHandle m_shape;
		internal m4x4 m_link_to_world;
		internal SpatialVector m_velocity;
		internal SpatialVector m_acceleration;
		internal SpatialVector m_external_force;
		internal v4 m_gravity;
	}

	[DllImport(Dll)] internal static extern EStatus Physics_ArticulationCreate(ulong engine, ArticulationDesc* desc, ArticulationLink* links, ArticulationJoint* joints, out ulong articulation);
	[DllImport(Dll)] internal static extern EStatus Physics_ArticulationDestroy(ulong engine, ulong articulation);
	[DllImport(Dll)] internal static extern EStatus Physics_ArticulationStateGet(ulong engine, ulong articulation, ArticulationState* state, float* positions, float* velocities, float* accelerations, float* forces, uint scalar_capacity, out uint scalar_required);
	[DllImport(Dll)] internal static extern EStatus Physics_ArticulationStateSet(ulong engine, ulong articulation, ArticulationState* state, float* positions, float* velocities, float* forces, uint scalar_count);
	[DllImport(Dll)] internal static extern EStatus Physics_ArticulationLinksCopy(ulong engine, ulong articulation, ArticulationLinkState* links, uint capacity, out uint required);
	[DllImport(Dll)] internal static extern EStatus Physics_ArticulationLinkForceSet(ulong engine, ulong articulation, uint link_index, SpatialVector* force);
	[DllImport(Dll)] internal static extern EStatus Physics_ArticulationLinkForceApply(ulong engine, ulong articulation, uint link_index, SpatialVector* force);
	[DllImport(Dll)] internal static extern EStatus Physics_ArticulationLinkGravitySet(ulong engine, ulong articulation, uint link_index, v4* gravity);
}
