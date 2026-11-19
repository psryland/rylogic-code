using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Threading;
using Microsoft.Win32.SafeHandles;
using Rylogic.D3D12;

namespace Rylogic.Physics;

/// <summary>Owns a reference-counted native Physics DLL context and the engines created through it.</summary>
public sealed class Physics :IDisposable
{
	private readonly int m_owner_thread_id;
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
		// Check before creating a token: an incompatible DLL may not implement this version's cleanup entry points.
		if (Native.Physics_ApiVersion() != Native.ApiVersion)
			throw new PhysicsException(EStatus.IncompatibleVersion, "The managed and native Physics API versions do not match.");

		m_owner_thread_id = Environment.CurrentManagedThreadId;
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

		try
		{
			// Publish managed ownership only after the SafeHandle has been allocated.
			m_context = new ContextHandle(context);
		}
		catch
		{
			// A failed managed allocation must not strand an initialization token without an owner.
			Native.Physics_ContextAbandon(context);
			throw;
		}
	}

	/// <summary>
	/// Cache runtime-compiled GPU kernels in 'directory' for engines created afterwards; null disables caching.
	/// The cache is shared by every Physics instance in the process.
	/// </summary>
	public void SetShaderCacheDirectory(string? directory)
	{
		EnsureOwner();
		Native.Check(Native.Physics_ShaderCacheDirectorySet(Context, directory));
	}

	/// <summary>
	/// Compile every runtime GPU kernel into the current shader cache using 'thread_count' worker threads, so later engine creation does not
	/// invoke the shader compiler. Blocks until done. 'progress' receives the completed and total kernel counts on the calling thread.
	/// Requires a shader cache directory. Throws OperationCanceledException when 'cancel' stops the remaining work.
	/// </summary>
	public void PrecompileShaders(int thread_count, Action<int, int>? progress = null, CancellationToken cancel = default)
	{
		EnsureOwner();

		// The callback stops native work on cancellation and never lets a managed exception cross the native boundary.
		var error = (Exception?)null;
		Native.ShaderPrecompileProgressFn callback = (_, completed, total) =>
		{
			// Report progress, then decide whether to continue.
			try
			{
				progress?.Invoke(completed, total);
			}
			catch (Exception ex)
			{
				error = ex;
			}
			return error == null && !cancel.IsCancellationRequested ? 1 : 0;
		};
		Native.Check(Native.Physics_ShaderPrecompile(Context, thread_count, callback, IntPtr.Zero));
		GC.KeepAlive(callback);

		// Surface a callback failure or cancellation after native work has stopped.
		if (error != null)
			throw new AggregateException("Shader precompile progress callback failed.", error);

		cancel.ThrowIfCancellationRequested();
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

		Engine engine;
		try
		{
			// The wrapper's SafeHandle takes ownership only after its constructor succeeds.
			engine = new Engine(this, handle, m_owner_thread_id);
		}
		catch
		{
			// Preserve unfinished native work in the existing registry if managed adoption itself fails.
			Native.Physics_EngineAbandon(handle);
			throw;
		}
		m_engines.Add(engine);
		return engine;
	}

	/// <summary>Dispose all owned engines with infinite GPU waits. Failed cleanup leaves the runtime available for retry.</summary>
	public void Dispose()
	{
		// A retained abandoned engine requires an explicit collection decision, not an implicit finalizer wait.
		if (!TryDispose(-1))
			throw new PhysicsException(EStatus.StepPending, "Native cleanup remains pending. Call CollectAbandoned and retry disposal.");
	}

	/// <summary>Close owned engines within one millisecond budget (-1 means infinite). False or an exception leaves remaining ownership intact.</summary>
	public bool TryDispose(int timeout_ms)
	{
		// Validate the budget and establish its start before any child can consume it.
		Native.WaitTimeout(timeout_ms);
		if (m_context == null)
			return true;

		EnsureOwner();
		var timer = Stopwatch.StartNew();

		// Engine disposal mutates the set, so traverse a stable cold-path copy.
		var engines = new Engine[m_engines.Count];
		m_engines.CopyTo(engines);
		foreach (var engine in engines)
		{
			// One slow child must not grant later children a fresh timeout.
			var remaining = timeout_ms == -1 ? -1 : (int)Math.Max(0, timeout_ms - timer.ElapsedMilliseconds);
			if (!engine.TryDispose(remaining))
				return false;
		}

		// A refused final-context release retains its token and leaves CollectAbandoned available.
		var status = Native.Physics_Shutdown(Context);
		switch (status)
		{
			case EStatus.StepPending: { return false; }
			case EStatus.Success: { break; }
			default:
			{
				throw new PhysicsException(status, $"Native context shutdown failed with status {status}; ownership is retained.");
			}
		}
		m_context.SetHandleAsInvalid();
		m_context.Dispose();
		m_context = null;
		GC.SuppressFinalize(this);
		return true;
	}

	/// <summary>
	/// Retry process-wide abandoned engines within one millisecond budget (-1 means infinite).
	/// Returns retained owners plus busy engines that still need inspection; other failures throw. Busy engines are not waited on,
	/// even with an infinite GPU budget. Retry when their operations finish. No live caller-owned object is collected.
	/// </summary>
	public uint CollectAbandoned(int timeout_ms)
	{
		// Keep the runtime alive while native collection pins and closes its retained records.
		EnsureOwner();
		var status = Native.Physics_CollectAbandoned(Context, Native.WaitTimeout(timeout_ms), out var remaining);
		Native.CloseCompleted(status);
		GC.KeepAlive(this);
		return remaining;
	}

	/// <summary>Remove an engine after its native lifetime has ended.</summary>
	internal void Remove(Engine engine)
	{
		m_engines.Remove(engine);
	}

	/// <summary>Throw if a mutable or lifetime operation is attempted from a different OS thread.</summary>
	internal void EnsureOwner()
	{
		if (Environment.CurrentManagedThreadId != m_owner_thread_id)
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
			// The native process anchor retains any unfinished engines after this caller token is relinquished.
			Native.Physics_ContextAbandon(handle);
			return true;
		}
	}
}
