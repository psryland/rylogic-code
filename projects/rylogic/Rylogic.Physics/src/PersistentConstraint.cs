using System;

namespace Rylogic.Physics;

/// <summary>Owns one generation-aware persistent D6 constraint within an engine.</summary>
public sealed class PersistentConstraint :IDisposable
{
	private PersistentConstraintHandle m_handle;

	/// <summary>Adopt a persistent constraint created by the owning engine.</summary>
	internal PersistentConstraint(Engine engine, PersistentConstraintHandle handle)
	{
		Engine = engine;
		m_handle = handle;
	}

	/// <summary>The engine that owns this constraint.</summary>
	internal Engine Engine { get; }

	/// <summary>True after this wrapper no longer owns a native persistent-constraint slot.</summary>
	public bool IsDisposed
	{
		get
		{
			return m_handle.IsNull;
		}
	}

	/// <summary>The stable typed identity used by break events and diagnostics.</summary>
	public PersistentConstraintHandle Handle
	{
		get
		{
			if (m_handle.IsNull)
				throw new ObjectDisposedException(nameof(PersistentConstraint));

			return m_handle;
		}
	}

	/// <summary>Read the complete D6 declaration and sticky break state outside a pending step.</summary>
	public unsafe PersistentConstraintState GetState()
	{
		var native = new Native.D6Constraint
		{
			m_header = NativeHeader.Create<Native.D6Constraint>(),
		};
		Native.Check(Native.Physics_ConstraintGetD6(Engine.Handle, Handle.Value, &native, out var broken));
		return new PersistentConstraintState(native.ToPublic(Engine), broken != 0);
	}

	/// <summary>Atomically replace endpoint frames, scalar rows, thresholds, and flags while preserving identity.</summary>
	public unsafe void Update(D6ConstraintOptions options)
	{
		Engine.EnsureOwner();
		if (options == null)
			throw new ArgumentNullException(nameof(options));

		Engine.ValidateConstraintEndpoints(options);
		var native = Native.D6Constraint.From(options);
		Native.Check(Native.Physics_ConstraintUpdateD6(Engine.Handle, Handle.Value, &native));
	}

	/// <summary>Enable or disable solver participation without destroying the declaration.</summary>
	public void SetEnabled(bool enabled)
	{
		Engine.EnsureOwner();
		Native.Check(Native.Physics_ConstraintSetEnabled(Engine.Handle, Handle.Value, enabled ? 1 : 0));
	}

	/// <summary>Clear the sticky overload latch so an enabled constraint can participate again.</summary>
	public void Repair()
	{
		Engine.EnsureOwner();
		Native.Check(Native.Physics_ConstraintRepair(Engine.Handle, Handle.Value));
	}

	/// <summary>Destroy this declaration and release its native endpoint references.</summary>
	public void Dispose()
	{
		if (m_handle.IsNull)
			return;

		Engine.EnsureOwner();
		Native.Check(Native.Physics_ConstraintDestroy(Engine.Handle, m_handle.Value));
		m_handle = default;
		Engine.Remove(this);
		GC.SuppressFinalize(this);
	}

	/// <summary>Invalidate this wrapper after its owning engine destroys the native constraint registry.</summary>
	internal void ReleaseFromEngine()
	{
		if (m_handle.IsNull)
			return;

		m_handle = default;
		Engine.Remove(this);
		GC.SuppressFinalize(this);
	}
}
