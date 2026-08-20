using System;
using Rylogic.Maths;

namespace Rylogic.Physics;

/// <summary>Owns one generation-aware native rigid body and its associated shape reference.</summary>
public sealed class RigidBody :IDisposable
{
	private BodyHandle m_handle;

	/// <summary>Adopt a body created by the owning engine.</summary>
	internal RigidBody(Engine engine, Shape shape, BodyHandle handle)
	{
		Engine = engine;
		Shape = shape;
		m_handle = handle;
	}

	/// <summary>The engine that owns this body.</summary>
	internal Engine Engine { get; }

	/// <summary>The collision shape retained by this body.</summary>
	public Shape Shape { get; }

	/// <summary>True after this wrapper no longer owns a native body slot.</summary>
	public bool IsDisposed
	{
		get
		{
			return m_handle.IsNull;
		}
	}

	/// <summary>The stable typed body identity used by commands, snapshots, events, and checkpoints.</summary>
	public BodyHandle Handle
	{
		get
		{
			if (m_handle.IsNull)
				throw new ObjectDisposedException(nameof(RigidBody));

			return m_handle;
		}
	}

	/// <summary>Read the complete current body state outside a pending step.</summary>
	public unsafe RigidBodyState GetState()
	{
		var state = new Native.BodyState
		{
			m_header = NativeHeader.Create<Native.BodyState>(),
		};
		Native.Check(Native.Physics_BodyStateGet(Engine.Handle, m_handle.Value, &state));
		return new RigidBodyState(
			state.m_object_to_world,
			state.m_inertia,
			state.m_momentum,
			state.m_velocity,
			state.m_force,
			state.m_gravity,
			state.m_user_tag,
			state.m_motion_type,
			state.m_flags);
	}

	/// <summary>Replace the complete current body state outside a pending step.</summary>
	public unsafe void SetState(RigidBodyState state)
	{
		Engine.EnsureOwner();
		var native = new Native.BodyState
		{
			m_header = NativeHeader.Create<Native.BodyState>(),
			m_body = m_handle,
			m_shape = Shape.Handle,
			m_object_to_world = state.m_object_to_world,
			m_inertia = state.m_inertia,
			m_momentum = state.m_momentum,
			m_velocity = state.m_velocity,
			m_force = state.m_force,
			m_gravity = state.m_gravity,
			m_user_tag = state.m_user_tag,
			m_motion_type = state.m_motion_type,
			m_flags = state.m_flags,
		};
		Native.Check(Native.Physics_BodyStateSet(Engine.Handle, m_handle.Value, &native));
	}

	/// <summary>Destroy this body and release its native reference to the shape.</summary>
	public void Dispose()
	{
		if (m_handle.IsNull)
			return;

		Engine.EnsureOwner();
		Native.Check(Native.Physics_BodyDestroy(Engine.Handle, m_handle.Value));
		m_handle = default;
		Engine.Remove(this);
		GC.SuppressFinalize(this);
	}

	/// <summary>Invalidate this wrapper after its owning engine destroys the native body registry.</summary>
	internal void ReleaseFromEngine()
	{
		if (m_handle.IsNull)
			return;

		m_handle = default;
		Engine.Remove(this);
		GC.SuppressFinalize(this);
	}
}
