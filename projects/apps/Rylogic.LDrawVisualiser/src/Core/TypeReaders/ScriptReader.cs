namespace Rylogic.LDrawVisualiser.Core.TypeReaders;

using System;
using System.Collections.Generic;
using EnvDTE;
using Rylogic.LDrawVisualiser.Core;

internal class ScriptReader : ITypeReader
{
	// Notes:
	// - Holds type readers that scripts have registered at runtime via vars.RegisterReader(...).
	// - Checked before the built-in readers so scripts can override built-in behaviour.
	// - Registrations are scoped to the lifetime of the owning TypeHandler (one script run).
	// - Lookup is O(1) on the type-name dictionary.
	private readonly Dictionary<string, Func<dynamic, object?>> m_readers = new();

	/// <summary>Register a reader for the exact debugger-reported type name 'type_name'</summary>
	public void Register(string type_name, Func<dynamic, object?> reader)
	{
		if (string.IsNullOrEmpty(type_name))
			throw new ArgumentException("Type name must be non-empty", nameof(type_name));
		if (reader == null)
			throw new ArgumentNullException(nameof(reader));

		// Last registration wins, so scripts can override an earlier registration cleanly.
		m_readers[type_name] = reader;
	}

	/// <inheritdoc/>
	public bool CanRead(ELanguage lang, string type_name) => m_readers.ContainsKey(type_name);

	/// <inheritdoc/>
	public object? Read(Debugger debugger, ELanguage lang, string type_name, string expr)
	{
		if (!m_readers.TryGetValue(type_name, out var fn))
			return null;

		// The lambda receives a DebugProxy positioned at 'expr' so it can chain field
		// accesses (v.x, v.y, ...). The proxy shares this TypeHandler instance and the
		// owning ExpressionCache so nested reads see script-registered readers, built-ins
		// and the last-known-value cache consistently.
		var proxy = new DebugProxy(debugger, expr, m_owner, m_owner.Cache);
		return fn(proxy);
	}

	// The TypeHandler that owns this registry. Set immediately after construction so
	// child DebugProxy instances created inside Read() share the same reader set.
	private TypeHandler m_owner = null!;
	internal void SetOwner(TypeHandler owner) => m_owner = owner;
}
