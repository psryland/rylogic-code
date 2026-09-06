using System;
using System.Collections.Generic;
using Rylogic.Maths;

namespace Rylogic.Physics;

/// <summary>Owns one generation-aware reduced-coordinate articulation within an engine.</summary>
public sealed class Articulation :IDisposable
{
	private ArticulationHandle m_handle;

	/// <summary>Adopt an articulation and retain its engine-owned link shapes.</summary>
	internal Articulation(Engine engine, ArticulationHandle handle, Shape[] shapes)
	{
		Engine = engine;
		Shapes = shapes;
		m_handle = handle;
	}

	/// <summary>The engine that owns this articulation.</summary>
	internal Engine Engine { get; }

	/// <summary>The topologically ordered collision shapes retained by the articulation.</summary>
	public IReadOnlyList<Shape> Shapes { get; }

	/// <summary>True after this wrapper no longer owns a native articulation slot.</summary>
	public bool IsDisposed
	{
		get
		{
			return m_handle.IsNull;
		}
	}

	/// <summary>The stable typed articulation identity used by constraints, events, and diagnostics.</summary>
	public ArticulationHandle Handle
	{
		get
		{
			if (m_handle.IsNull)
				throw new ObjectDisposedException(nameof(Articulation));

			return m_handle;
		}
	}

	/// <summary>Read whole-tree state and all flattened non-root joint scalars outside a pending step.</summary>
	public unsafe ArticulationState GetState()
	{
		var native = new Native.ArticulationState();
		var status = Native.Physics_ArticulationStateGet(Engine.Handle, Handle.Value, &native, null, null, null, null, 0, out var required);
		if (status != EStatus.Success && status != EStatus.BufferTooSmall)
			Native.Check(status);

		var positions = new float[required];
		var velocities = new float[required];
		var accelerations = new float[required];
		var forces = new float[required];

		// The native stream uses one scalar index for positions, velocities, accelerations, and persistent forces.
		fixed (float* position_ptr = positions)
		fixed (float* velocity_ptr = velocities)
		fixed (float* acceleration_ptr = accelerations)
		fixed (float* force_ptr = forces)
			Native.Check(Native.Physics_ArticulationStateGet(Engine.Handle, Handle.Value, &native, position_ptr, velocity_ptr, acceleration_ptr, force_ptr, required, out required));

		return new ArticulationState(
			native.m_articulation,
			native.m_root_to_world,
			native.m_root_velocity,
			native.m_root_force,
			native.m_user_tag,
			checked((int)native.m_link_count),
			native.m_flags,
			positions,
			velocities,
			accelerations,
			forces);
	}

	/// <summary>Replace whole-tree mutable state and flattened non-root joint state outside a pending step.</summary>
	public unsafe void SetState(ArticulationState state)
	{
		Engine.EnsureOwner();
		if (state == null)
			throw new ArgumentNullException(nameof(state));
		if (state.Articulation != Handle)
			throw new ArgumentException("The articulation state belongs to a different native articulation.", nameof(state));
		if (state.Positions.Length != state.Velocities.Length || state.Positions.Length != state.Forces.Length)
			throw new ArgumentException("Articulation position, velocity, and force arrays must have identical lengths.", nameof(state));

		var native = new Native.ArticulationState
		{
			m_header = NativeHeader.Create<Native.ArticulationState>(),
			m_articulation = Handle,
			m_root_to_world = state.RootToWorld,
			m_root_velocity = state.RootVelocity,
			m_root_force = state.RootForce,
			m_user_tag = state.UserTag,
			m_link_count = checked((uint)state.LinkCount),
			m_joint_dof_count = checked((uint)state.Positions.Length),
			m_flags = state.Flags,
		};

		// Accelerations are derived outputs, so only authoritative position, velocity, and force streams are submitted.
		fixed (float* position_ptr = state.Positions)
		fixed (float* velocity_ptr = state.Velocities)
		fixed (float* force_ptr = state.Forces)
			Native.Check(Native.Physics_ArticulationStateSet(Engine.Handle, Handle.Value, &native, position_ptr, velocity_ptr, force_ptr, native.m_joint_dof_count));
	}

	/// <summary>Copy all current link kinematics and external fields in stable topological order.</summary>
	public unsafe ArticulationLinkState[] CopyLinks()
	{
		var status = Native.Physics_ArticulationLinksCopy(Engine.Handle, Handle.Value, null, 0, out var required);
		if (status != EStatus.Success && status != EStatus.BufferTooSmall)
			Native.Check(status);

		var native = new Native.ArticulationLinkState[required];
		fixed (Native.ArticulationLinkState* link_ptr = native)
			Native.Check(Native.Physics_ArticulationLinksCopy(Engine.Handle, Handle.Value, link_ptr, required, out required));

		var result = new ArticulationLinkState[required];
		for (var i = 0; i != result.Length; ++i)
			result[i] = new ArticulationLinkState(native[i]);

		return result;
	}

	/// <summary>Replace the persistent spatial force applied to one topological link.</summary>
	public unsafe void SetLinkForce(int link_index, SpatialVector force)
	{
		Engine.EnsureOwner();
		Native.Check(Native.Physics_ArticulationLinkForceSet(Engine.Handle, Handle.Value, checked((uint)link_index), &force));
	}

	/// <summary>Accumulate a spatial force into one topological link's persistent field.</summary>
	public unsafe void ApplyLinkForce(int link_index, SpatialVector force)
	{
		Engine.EnsureOwner();
		Native.Check(Native.Physics_ArticulationLinkForceApply(Engine.Handle, Handle.Value, checked((uint)link_index), &force));
	}

	/// <summary>Replace gravity for one topological link.</summary>
	public unsafe void SetLinkGravity(int link_index, v4 gravity)
	{
		Engine.EnsureOwner();
		Native.Check(Native.Physics_ArticulationLinkGravitySet(Engine.Handle, Handle.Value, checked((uint)link_index), &gravity));
	}

	/// <summary>Destroy this articulation after all persistent constraints referencing its links have been destroyed.</summary>
	public void Dispose()
	{
		if (m_handle.IsNull)
			return;

		Engine.EnsureOwner();
		Native.Check(Native.Physics_ArticulationDestroy(Engine.Handle, m_handle.Value));
		m_handle = default;
		Engine.Remove(this);
		GC.SuppressFinalize(this);
	}

	/// <summary>Invalidate this wrapper after its owning engine destroys the native articulation registry.</summary>
	internal void ReleaseFromEngine()
	{
		if (m_handle.IsNull)
			return;

		m_handle = default;
		Engine.Remove(this);
		GC.SuppressFinalize(this);
	}
}
