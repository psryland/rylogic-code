namespace Rylogic.Audio;

#if NET472
using System;
using System.Runtime.InteropServices;
using System.Threading;
using Rylogic.Common;
using Rylogic.Interop.Win32;
using Rylogic.Utility;
using HContext = System.IntPtr;


public sealed class Audio :IDisposable
{
	private readonly HContext m_context;
	private readonly SynchronizationContext m_sync_context; // Thread marshaller
	private readonly int m_thread_id;         // The main thread id
	private ReportErrorCB m_error_cb;         // Reference to callback

	public Audio()
	{
		if (!ModuleLoaded)
			throw new Exception("Audio.dll has not been loaded");

		m_sync_context = SynchronizationContext.Current ?? throw new InvalidOperationException("Audio requires a synchronization context for error callbacks.");
		m_thread_id = Thread.CurrentThread.ManagedThreadId;

		// Initialise audio
		var init_error = (string?)null;
		ReportErrorCB error_cb = (ctx, msg) => init_error = msg;
		m_context = Audio_Initialise(error_cb, IntPtr.Zero);
		if (m_context == HContext.Zero)
			throw new Exception(init_error ?? "Failed to initialised Audio");

		// Attach the global error handler
		m_error_cb = (ctx, msg) =>
		{
			if (m_thread_id != Thread.CurrentThread.ManagedThreadId)
				m_sync_context.Post(_ => m_error_cb(ctx, msg), null);
			else
				Error?.Invoke(this, new MessageEventArgs(msg));
		};
		Audio_GlobalErrorCBSet(m_error_cb, IntPtr.Zero, true);
	}
	public void Dispose()
	{
		Util.BreakIf(Util.IsGCFinalizerThread, "Disposing in the GC finaliser thread");

		// Unsubscribe
		Audio_GlobalErrorCBSet(m_error_cb, IntPtr.Zero, false);

		Audio_Shutdown(m_context);
	}

	/// <summary>Event call on errors. Note: can be called in a background thread context</summary>
	public event EventHandler<MessageEventArgs>? Error;

	/// <summary>Play a WAV file</summary>
	public void PlayFile(string filepath)
	{
		Audio_PlayFile(filepath);
	}

	/// <summary>Create a MIDI instrument wave bank</summary>
	public void WaveBankCreateMidiInstrument(string bank_name, string root_dir, string xwb_filepath, string xml_instrument_filepath)
	{
		Audio_WaveBankCreateMidiInstrument(bank_name, root_dir, xwb_filepath, xml_instrument_filepath);
	}

	#region DLL extern functions

	/// <summary>True if the view3d dll has been loaded</summary>
	private const string Dll = "audio";
	public static bool ModuleLoaded => m_module != IntPtr.Zero;
	private static IntPtr m_module = IntPtr.Zero;

	/// <summary>The exception created if the module fails to load</summary>
	public static System.Exception? LoadError;

	/// <summary>Helper method for loading the view3d.dll from a platform specific path</summary>
	public static void LoadDll(string dir = @".\lib\$(platform)\$(config)")
	{
		if (ModuleLoaded) return;
		m_module = Win32.LoadDll(Dll+".dll", out LoadError, dir);
	}

	/// <summary>Report errors callback</summary>
	public delegate void ReportErrorCB(IntPtr ctx, [MarshalAs(UnmanagedType.LPWStr)] string msg);

	// Initialise / shutdown the dll
	[DllImport(Dll)] private static extern HContext Audio_Initialise(ReportErrorCB initialise_error_cb, IntPtr ctx);
	[DllImport(Dll)] private static extern void     Audio_Shutdown(HContext context);
	[DllImport(Dll)] private static extern void     Audio_GlobalErrorCBSet(ReportErrorCB error_cb, IntPtr ctx, bool add);

	// Wave Banks
	[DllImport(Dll)] private static extern void     Audio_WaveBankCreateMidiInstrument([MarshalAs(UnmanagedType.LPStr)] string bank_name, [MarshalAs(UnmanagedType.LPWStr)] string root_dir, [MarshalAs(UnmanagedType.LPWStr)] string xwb_filepath, [MarshalAs(UnmanagedType.LPWStr)] string xml_instrument_filepath);

	// Misc
	[DllImport(Dll)] private static extern void     Audio_PlayFile([MarshalAs(UnmanagedType.LPWStr)] string filepath);

	#endregion
}
#endif