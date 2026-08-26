using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using Microsoft.Win32.SafeHandles;
using Rylogic.D3D12;
using Rylogic.Gfx;

namespace Rylogic.Gfx.UI;

/// <summary>Owns the process-level native View3DUI module and the UI contexts created through it.</summary>
public sealed class UiRuntime :IDisposable
{
	private readonly uint m_owner_thread_id;
	private readonly Native.ReportErrorFn m_error_fn;
	private readonly HashSet<UiContext> m_contexts;
	private RuntimeHandle? m_runtime;
	private string? m_initialisation_error;

	/// <summary>Initialise the native View3DUI runtime on the current OS thread and verify the managed/native ABI match.</summary>
	public UiRuntime()
	{
		if (IntPtr.Size != sizeof(ulong))
			throw new PlatformNotSupportedException("Rylogic.Gfx.UI requires a 64-bit process.");

		Native.EnsureLoaded();
		m_owner_thread_id = Native.GetCurrentThreadId();
		m_error_fn = HandleNativeError;
		m_contexts = new HashSet<UiContext>();

		// The reverse callback records initialisation failures and never allows a managed exception to cross the native boundary.
		var callback = new Native.ReportErrorCallback
		{
			m_ctx = IntPtr.Zero,
			m_cb = m_error_fn,
		};
		var runtime = Native.View3DUI_Initialise(callback);
		if (runtime == IntPtr.Zero)
			throw new View3dUiException(EStatus.InternalError, m_initialisation_error ?? "Failed to initialise the native View3DUI runtime.");

		m_runtime = new RuntimeHandle(runtime);
		if (Native.View3DUI_ApiVersion() != Native.ApiVersion)
		{
			Dispose();
			throw new View3dUiException(EStatus.AbiMismatch, "The managed and native View3DUI API versions do not match.");
		}

		VerifyStructLayout();
	}

	/// <summary>Create a UI context, optionally attaching its own D3D12 device lease and/or an existing View3D window for rendering.</summary>
	public unsafe UiContext CreateContext(UiConfig? config = null, DeviceLease? device = null, View3d.Window? window = null)
	{
		EnsureOwner();
		var runtime = Runtime;
		var native_config = config != null ? Native.Config.From(config) : default;
		Native.Config* config_ptr = config != null ? &native_config : null;
		var window_ptr = window?.Handle ?? IntPtr.Zero;

		ulong handle;
		if (device == null)
		{
			Native.Check(Native.View3DUI_ContextCreate(runtime, config_ptr, IntPtr.Zero, window_ptr, out handle));
		}
		else
		{
			// Pin the lease only for creation; the native context takes its own independent COM reference.
			using var borrowed = device.Borrow();
			Native.Check(Native.View3DUI_ContextCreate(runtime, config_ptr, borrowed.Handle, window_ptr, out handle));
		}

		var context = new UiContext(this, handle, m_owner_thread_id);
		m_contexts.Add(context);
		return context;
	}

	/// <summary>Dispose all owned contexts before releasing the process-level native runtime.</summary>
	public void Dispose()
	{
		if (m_runtime == null)
			return;

		EnsureOwner();
		// Context disposal mutates the set, so traverse a stable cold-path copy.
		var contexts = new UiContext[m_contexts.Count];
		m_contexts.CopyTo(contexts);
		foreach (var context in contexts)
			context.Dispose();

		m_runtime.Dispose();
		m_runtime = null;
		GC.SuppressFinalize(this);
	}

	/// <summary>Remove a context after its native lifetime has ended.</summary>
	internal void Remove(UiContext context)
	{
		m_contexts.Remove(context);
	}

	/// <summary>Throw if a mutable or lifetime operation is attempted from a different OS thread.</summary>
	internal void EnsureOwner()
	{
		if (Native.GetCurrentThreadId() != m_owner_thread_id)
			throw new InvalidOperationException("View3DUI runtime lifetime operations must run on the OS thread that created it.");
	}

	/// <summary>The active native runtime token.</summary>
	private IntPtr Runtime
	{
		get
		{
			var runtime = m_runtime ?? throw new ObjectDisposedException(nameof(UiRuntime));
			return runtime.DangerousGetHandle();
		}
	}

	/// <summary>
	/// Confirm every native wire structure discoverable through View3DUI_StructSize matches the size this managed layer
	/// assumes, catching an ABI drift between the loaded native DLL and this compiled managed assembly at startup rather
	/// than as a corrupted read/write deep inside a later call.
	/// </summary>
	private static void VerifyStructLayout()
	{
		VerifyStructSize(EStructId.Config, Marshal.SizeOf<Native.Config>());
		VerifyStructSize(EStructId.Transaction, Marshal.SizeOf<Native.Transaction>());
		VerifyStructSize(EStructId.Operation, Marshal.SizeOf<Native.Operation>());
		VerifyStructSize(EStructId.Control, Marshal.SizeOf<Native.ControlDesc>());
		VerifyStructSize(EStructId.Resource, Marshal.SizeOf<Native.ResourceDesc>());
		VerifyStructSize(EStructId.Style, Marshal.SizeOf<Native.StyleDesc>());
		VerifyStructSize(EStructId.Template, Marshal.SizeOf<Native.TemplateDesc>());
		VerifyStructSize(EStructId.NormalizedInput, Marshal.SizeOf<NormalizedInput>());
		VerifyStructSize(EStructId.ViewportState, Marshal.SizeOf<ViewportState>());
		VerifyStructSize(EStructId.Event, Marshal.SizeOf<Native.Event>());
		VerifyStructSize(EStructId.SemanticNode, Marshal.SizeOf<Native.SemanticNode>());
		VerifyStructSize(EStructId.Diagnostics, Marshal.SizeOf<UiDiagnostics>());
		VerifyStructSize(EStructId.HostBridgeVersion, Marshal.SizeOf<Native.HostBridgeVersion>());
		VerifyStructSize(EStructId.HostPassContext, Marshal.SizeOf<Native.HostPassContext>());
		VerifyStructSize(EStructId.InputTextPayload, Marshal.SizeOf<Native.InputTextPayload>());
	}

	/// <summary>Compare one native reported struct size against the managed mirror size for 'struct_id'.</summary>
	private static void VerifyStructSize(EStructId struct_id, int managed_size)
	{
		Native.Check(Native.View3DUI_StructSize(struct_id, out var native_size));
		if (native_size != managed_size)
			throw new View3dUiException(EStatus.AbiMismatch, $"View3DUI struct '{struct_id}' is {native_size} bytes natively but {managed_size} bytes in the managed mirror.");
	}

	/// <summary>Contain initialisation callback failures as text for the creating managed call.</summary>
	private void HandleNativeError(IntPtr ctx, string msg, string filepath, int line)
	{
		try
		{
			m_initialisation_error = msg;
		}
		catch
		{
			// Reverse P/Invoke callbacks must never propagate a managed exception into native code.
		}
	}

	/// <summary>Releases the process-level native View3DUI runtime token.</summary>
	private sealed class RuntimeHandle :SafeHandleZeroOrMinusOneIsInvalid
	{
		/// <summary>Adopt a successful View3DUI_Initialise token.</summary>
		internal RuntimeHandle(IntPtr handle)
			: base(true)
		{
			SetHandle(handle);
		}

		/// <inheritdoc/>
		protected override bool ReleaseHandle()
		{
			// ReleaseHandle can run on the GC finalizer thread for a forgotten (never-Disposed) runtime. View3DUI_Shutdown
			// enforces the owner-thread and "no live contexts" invariants and reports failure rather than silently
			// abandoning state, so a finalizer-thread call to it can legitimately return non-Success. View3DUI_ContextAbandon
			// is the thread-agnostic fallback the native API pairs with Shutdown for exactly this case (see its own doc
			// comment: "Release a forgotten runtime token ... from any thread"), so fall back to it to avoid leaking the
			// runtime token and its process-level context.
			if (Native.View3DUI_Shutdown(handle) != EStatus.Success)
				Native.View3DUI_ContextAbandon(handle);

			return true;
		}
	}
}
