using System;
using System.Collections.Generic;
using Microsoft.Win32.SafeHandles;
using Rylogic.D3D12;

namespace Rylogic.Physics;

/// <summary>Owns a reference-counted native Physics DLL context and the engines created through it.</summary>
public sealed class Physics :IDisposable
{
	private readonly uint m_owner_thread_id;
	private readonly Native.ReportErrorFn m_error_fn;
	private readonly HashSet<Engine> m_engines;
	private ContextHandle? m_context;
	private string? m_initialisation_error;

	/// <summary>Initialise the native runtime on the current OS thread.</summary>
	public Physics()
	{
		if (IntPtr.Size != sizeof(ulong))
			throw new PlatformNotSupportedException("Rylogic.Physics requires a 64-bit process.");

		Native.EnsureLoaded();
		m_owner_thread_id = Native.GetCurrentThreadId();
		m_error_fn = HandleNativeError;
		m_engines = new HashSet<Engine>();

		// The reverse callback records errors and never allows managed exceptions to cross the native boundary.
		var callback = new Native.ReportErrorCallback
		{
			m_context = IntPtr.Zero,
			m_callback = m_error_fn,
		};
		var context = Native.Physics_Initialise(callback);
		if (context == IntPtr.Zero)
			throw new PhysicsException(EStatus.InternalError, m_initialisation_error ?? "Failed to initialise the native physics runtime.");

		m_context = new ContextHandle(context);
		if (Native.Physics_ApiVersion() != Native.ApiVersion)
		{
			Dispose();
			throw new PhysicsException(EStatus.IncompatibleVersion, "The managed and native Physics API versions do not match.");
		}
	}

	/// <summary>Create a physics engine using either its own D3D12 device or an externally leased device.</summary>
	public unsafe Engine CreateEngine(EngineOptions? options = null, DeviceLease? device = null)
	{
		EnsureOwner();
		var context = Context;
		var config = options != null ? Native.EngineConfig.From(options) : default;
		Native.EngineConfig* config_ptr = options != null ? &config : null;

		ulong handle;
		if (device == null)
		{
			Native.Check(Native.Physics_EngineCreate(context, config_ptr, IntPtr.Zero, out handle));
		}
		else
		{
			// Pin the lease only for creation; the native engine takes its own independent COM reference.
			using var borrowed = device.Borrow();
			Native.Check(Native.Physics_EngineCreate(context, config_ptr, borrowed.Handle, out handle));
		}

		var engine = new Engine(this, handle, m_owner_thread_id);
		m_engines.Add(engine);
		return engine;
	}

	/// <summary>Dispose all owned engines before releasing the DLL context.</summary>
	public void Dispose()
	{
		if (m_context == null)
			return;

		EnsureOwner();
		// Engine disposal mutates the set, so traverse a stable cold-path copy.
		var engines = new Engine[m_engines.Count];
		m_engines.CopyTo(engines);
		foreach (var engine in engines)
			engine.Dispose();

		m_context.Dispose();
		m_context = null;
		GC.SuppressFinalize(this);
	}

	/// <summary>Remove an engine after its native lifetime has ended.</summary>
	internal void Remove(Engine engine)
	{
		m_engines.Remove(engine);
	}

	/// <summary>Throw if a mutable or lifetime operation is attempted from a different OS thread.</summary>
	internal void EnsureOwner()
	{
		if (Native.GetCurrentThreadId() != m_owner_thread_id)
			throw new InvalidOperationException("Physics lifetime operations must run on the OS thread that created the runtime.");
	}

	/// <summary>The active native context token.</summary>
	private IntPtr Context
	{
		get
		{
			var context = m_context ?? throw new ObjectDisposedException(nameof(Physics));
			return context.DangerousGetHandle();
		}
	}

	/// <summary>Contain initialisation callback failures as text for the creating managed call.</summary>
	private void HandleNativeError(IntPtr context, string message, string filepath, int line, long position)
	{
		try
		{
			m_initialisation_error = message;
		}
		catch
		{
			// Reverse P/Invoke callbacks must never propagate a managed exception into native code.
		}
	}

	/// <summary>Releases one reference-counted native DLL context token.</summary>
	private sealed class ContextHandle :SafeHandleZeroOrMinusOneIsInvalid
	{
		/// <summary>Adopt a successful Physics_Initialise token.</summary>
		internal ContextHandle(IntPtr handle)
			: base(true)
		{
			SetHandle(handle);
		}

		/// <inheritdoc/>
		protected override bool ReleaseHandle()
		{
			Native.Physics_Shutdown(handle);
			return true;
		}
	}
}
