using System;

namespace Rylogic.Physics;

/// <summary>
/// Collects descriptor replacements for persistent constraints so that 'Engine.UpdateConstraints' applies them in one native call.
/// Each 'Add' copies the options immediately, so callers may reuse one options object for every entry. The batch reuses its storage,
/// so a batch that is filled and applied every step does not allocate once it has grown to its working size.
/// </summary>
public sealed class ConstraintUpdateBatch
{
	private ulong[] m_handles;
	private Native.D6Constraint[] m_descs;
	private int m_count;

	/// <summary>Create an empty batch for constraints owned by 'engine'.</summary>
	public ConstraintUpdateBatch(Engine engine)
	{
		// Start with a small capacity that grows as needed
		Engine = engine ?? throw new ArgumentNullException(nameof(engine));
		m_handles = new ulong[16];
		m_descs = new Native.D6Constraint[16];
		m_count = 0;
	}

	/// <summary>The engine whose constraints this batch updates.</summary>
	public Engine Engine { get; }

	/// <summary>The number of pending updates.</summary>
	public int Count
	{
		get
		{
			return m_count;
		}
	}

	/// <summary>Queue a replacement descriptor for 'constraint'. A constraint added more than once takes its last descriptor.</summary>
	public void Add(PersistentConstraint constraint, D6ConstraintOptions options)
	{
		// Reject entries that the native batch would reject, so that errors point at the entry that caused them
		if (constraint == null)
			throw new ArgumentNullException(nameof(constraint));
		if (options == null)
			throw new ArgumentNullException(nameof(options));
		if (!ReferenceEquals(constraint.Engine, Engine))
			throw new ArgumentException("The constraint belongs to a different engine.", nameof(constraint));

		Engine.ValidateConstraintEndpoints(options);

		// Grow by doubling so that repeated filling settles at a fixed capacity
		if (m_count == m_handles.Length)
		{
			Array.Resize(ref m_handles, m_count * 2);
			Array.Resize(ref m_descs, m_count * 2);
		}

		// Convert now so that the caller can reuse 'options'
		m_handles[m_count] = constraint.Handle.Value;
		m_descs[m_count] = Native.D6Constraint.From(options);
		++m_count;
	}

	/// <summary>Discard all pending updates.</summary>
	public void Clear()
	{
		m_count = 0;
	}

	/// <summary>Apply the pending updates in one native call. All entries are checked before any constraint changes.</summary>
	internal unsafe void Apply()
	{
		// Pin the storage for the duration of the native call
		fixed (ulong* handles = m_handles)
		fixed (Native.D6Constraint* descs = m_descs)
		{
			Native.Check(Native.Physics_ConstraintUpdateD6Batch(Engine.Handle, handles, descs, (uint)m_count));
		}
	}
}
