using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using Microsoft.Win32.SafeHandles;

namespace Rylogic.Audio;

/// <summary>Owns the native Audio DLL context and the engines created through it.</summary>
public sealed class AudioRuntime :IDisposable
{
	private readonly uint m_owner_thread_id;
	private readonly HashSet<Engine> m_engines;
	private ContextHandle? m_context;
	private static readonly Native.ReportErrorFn m_error_fn = HandleNativeError;
	[ThreadStatic] private static string? m_initialisation_error;

	/// <summary>Initialise the native runtime on the current OS thread.</summary>
	public AudioRuntime()
	{
		if (IntPtr.Size != sizeof(ulong))
			throw new PlatformNotSupportedException("Rylogic.Audio requires a 64-bit process.");
		if (!IsWindows11OrGreater())
			throw new PlatformNotSupportedException("Rylogic.Audio requires Windows 11 or greater.");

		Native.EnsureLoaded();
		m_owner_thread_id = Native.GetCurrentThreadId();
		m_engines = new HashSet<Engine>();
		m_initialisation_error = null;

		// The reverse callback records errors and never allows managed exceptions to cross the native boundary.
		var callback = new Native.ReportErrorCallback
		{
			m_context = IntPtr.Zero,
			m_callback = m_error_fn,
		};
		var context = Native.Audio_Initialise(callback);
		if (context == IntPtr.Zero)
			throw new AudioException(EStatus.InternalError, m_initialisation_error ?? "Failed to initialise the native audio runtime.");

		m_context = new ContextHandle(context);
		if (Native.Audio_ApiVersion() != Native.ApiVersion)
		{
			Dispose();
			throw new AudioException(EStatus.InternalError, "The managed and native Audio API versions do not match.");
		}

		// Layout mismatches indicate a stale managed build against a newer native binary; fail fast rather than corrupt memory.
		try
		{
			ValidateStructLayouts();
		}
		catch
		{
			Dispose();
			throw;
		}
	}

	/// <summary>Create an audio engine. Pass null options to accept the native engine's defaults.</summary>
	public unsafe Engine CreateEngine(EngineOptions? options = null)
	{
		EnsureOwner();
		var context = Context;
		var config = options != null ? Native.Config.From(options) : default;
		Native.Config* config_ptr = options != null ? &config : null;

		Native.Check(Native.Audio_EngineCreate(context, config_ptr, out var handle));
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

		Native.Audio_Shutdown(m_context.DangerousGetHandle());
		m_context.MarkDestroyed();
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
			throw new InvalidOperationException("Audio lifetime operations must run on the OS thread that created the runtime.");
	}

	/// <summary>The active native context token.</summary>
	private IntPtr Context
	{
		get
		{
			var context = m_context ?? throw new ObjectDisposedException(nameof(AudioRuntime));
			return context.DangerousGetHandle();
		}
	}

	/// <summary>Contain initialisation callback failures as text for the creating managed call.</summary>
	private static void HandleNativeError(IntPtr context, string message, string filepath, int line)
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

	/// <summary>Compare every versioned managed record against the native ABI's reported size for the same identifier.</summary>
	private static void ValidateStructLayouts()
	{
		CheckStructSize(Native.EStructId.Config, NativeSize<Native.Config>.Value);
		CheckStructSize(Native.EStructId.ListenerState, NativeSize<ListenerState>.Value);
		CheckStructSize(Native.EStructId.EmitterState, NativeSize<EmitterState>.Value);
		CheckStructSize(Native.EStructId.VoiceDesc, NativeSize<Native.VoiceDesc>.Value);
		CheckStructSize(Native.EStructId.VoiceState, NativeSize<VoiceState>.Value);
		CheckStructSize(Native.EStructId.Event, NativeSize<Native.Event>.Value);
		CheckStructSize(Native.EStructId.Diagnostics, NativeSize<Diagnostics>.Value);
		CheckStructSize(Native.EStructId.StreamDesc, NativeSize<Native.StreamDesc>.Value);
		CheckStructSize(Native.EStructId.StreamState, NativeSize<StreamState>.Value);
	}

	/// <summary>Throw when one managed record's size disagrees with the native ABI's reported size.</summary>
	private static void CheckStructSize(Native.EStructId struct_id, uint managed_size)
	{
		Native.Check(Native.Audio_StructSize(struct_id, out var native_size));
		if (native_size != managed_size)
			throw new AudioException(EStatus.InvalidStruct, $"Managed/native layout mismatch for {struct_id}: managed={managed_size}, native={native_size}.");
	}

	/// <summary>
	/// True when the OS reports Windows 11 (build 22000) or greater. Queries RtlGetVersion directly rather than
	/// Environment.OSVersion, because .NET Framework processes without a matching application manifest can otherwise
	/// receive a compatibility-shimmed version number from the OS loader.
	/// </summary>
	private static bool IsWindows11OrGreater()
	{
		var info = new RtlOsVersionInfo { m_size = (uint)Marshal.SizeOf<RtlOsVersionInfo>() };
		if (RtlGetVersion(ref info) != 0)
			return false;

		return info.m_major_version > 10 || (info.m_major_version == 10 && info.m_build_number >= 22000);
	}

	[StructLayout(LayoutKind.Sequential, CharSet = CharSet.Unicode)]
	private struct RtlOsVersionInfo
	{
		internal uint m_size;
		internal uint m_major_version;
		internal uint m_minor_version;
		internal uint m_build_number;
		internal uint m_platform_id;
		[MarshalAs(UnmanagedType.ByValTStr, SizeConst = 128)]
		internal string m_service_pack;
	}

	[DllImport("ntdll.dll")]
	private static extern int RtlGetVersion(ref RtlOsVersionInfo version_info);

	/// <summary>Releases the native DLL context token.</summary>
	private sealed class ContextHandle :SafeHandleZeroOrMinusOneIsInvalid
	{
		/// <summary>Adopt a successful Audio_Initialise token.</summary>
		internal ContextHandle(IntPtr handle)
			: base(true)
		{
			SetHandle(handle);
		}

		/// <summary>Prevent finalizer cleanup after explicit dependency-ordered shutdown succeeds.</summary>
		internal void MarkDestroyed()
		{
			SetHandleAsInvalid();
		}

		/// <inheritdoc/>
		protected override bool ReleaseHandle()
		{
			Native.Audio_ContextAbandon(handle);
			return true;
		}
	}
}
