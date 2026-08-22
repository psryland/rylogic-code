using System;
using System.Buffers;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using Microsoft.Win32.SafeHandles;

namespace Rylogic.Audio;

/// <summary>Owns one native audio engine, its clips, voices, and streams.</summary>
public sealed class Engine :IDisposable
{
	private readonly AudioRuntime m_runtime;
	private readonly uint m_owner_thread_id;
	private readonly HashSet<Clip> m_clips;
	private readonly HashSet<Voice> m_voices;
	private readonly HashSet<AudioStream> m_streams;
	private EngineSafeHandle? m_handle;

	/// <summary>Adopt a newly-created native engine.</summary>
	internal Engine(AudioRuntime runtime, ulong handle, uint owner_thread_id)
	{
		m_runtime = runtime;
		m_owner_thread_id = owner_thread_id;
		m_clips = new HashSet<Clip>();
		m_voices = new HashSet<Voice>();
		m_streams = new HashSet<AudioStream>();
		m_handle = new EngineSafeHandle(handle, runtime);
	}

	/// <summary>True after this engine has released its native world.</summary>
	public bool IsDisposed
	{
		get
		{
			return m_handle == null || m_handle.IsInvalid || m_handle.IsClosed;
		}
	}

	/// <summary>Replace the position, orientation, and velocity of the engine's single rendered listener.</summary>
	public unsafe void SetListener(ListenerState listener)
	{
		EnsureOwner();
		Native.Check(Native.Audio_ListenerSet(Handle, &listener));
	}

	/// <summary>Replace the linear gain applied to every voice and stream routed through one mixer bus.</summary>
	public void SetBusGain(EAudioBus bus, float gain)
	{
		EnsureOwner();
		Native.Check(Native.Audio_BusGainSet(Handle, bus, gain));
	}

	/// <summary>Create a clip from a wave-encoded payload. The engine copies the payload; the clip owns no reference to it.</summary>
	public Clip CreateClip(AudioData data)
	{
		EnsureOwner();
		if (data == null)
			throw new ArgumentNullException(nameof(data));
		if (data.Encoding != EAudioEncoding.Wave)
			throw new ArgumentException("Clips require wave-encoded audio data.", nameof(data));

		return CreateClipCore(data.Span);
	}

	/// <summary>Create a clip directly from wave-encoded memory. The native engine copies the payload before this method returns.</summary>
	public Clip CreateClip(ReadOnlySpan<byte> wave_data)
	{
		return CreateClipCore(wave_data);
	}

	/// <summary>Create a voice that plays an engine-owned clip.</summary>
	public unsafe Voice CreateVoice(Clip clip, VoiceOptions? options = null)
	{
		EnsureOwner();
		if (clip == null)
			throw new ArgumentNullException(nameof(clip));
		if (!ReferenceEquals(clip.Engine, this))
			throw new ArgumentException("The clip must belong to this engine.", nameof(clip));

		options ??= new VoiceOptions();
		var desc = Native.VoiceDesc.From(clip.Handle, options);
		Native.Check(Native.Audio_VoiceCreate(Handle, &desc, out var handle));

		var voice = new Voice(this, clip, new VoiceHandle(handle));
		m_voices.Add(voice);
		clip.AddVoice(voice);
		return voice;
	}

	/// <summary>Create a stream that decodes an Ogg Vorbis payload. The engine copies the payload as it decodes it.</summary>
	public AudioStream CreateStream(AudioData data, StreamOptions? options = null)
	{
		EnsureOwner();
		if (data == null)
			throw new ArgumentNullException(nameof(data));
		if (data.Encoding != EAudioEncoding.OggVorbis)
			throw new ArgumentException("Streams require Ogg Vorbis-encoded audio data.", nameof(data));

		return CreateStreamCore(data.Span, options);
	}

	/// <summary>Create a stream directly from Ogg Vorbis memory. The native engine copies the payload before this method returns.</summary>
	public AudioStream CreateStream(ReadOnlySpan<byte> ogg_data, StreamOptions? options = null)
	{
		return CreateStreamCore(ogg_data, options);
	}

	/// <summary>Advance engine-owned bookkeeping such as voice virtualisation and event queueing.</summary>
	public void Update()
	{
		EnsureOwner();
		Native.Check(Native.Audio_EngineUpdate(Handle));
	}

	/// <summary>Return the number of events currently queued without consuming them.</summary>
	public unsafe int EventCount()
	{
		EnsureOwner();
		var status = Native.Audio_EventsCopy(Handle, null, 0, out var required);
		if (status != EStatus.Success && status != EStatus.BufferTooSmall)
			Native.Check(status);

		return checked((int)required);
	}

	/// <summary>Copy and consume every queued event. The destination must be large enough for <see cref="EventCount"/>.</summary>
	public unsafe int CopyEvents(Span<AudioEvent> events)
	{
		EnsureOwner();
		Native.Event[]? rented = null;
		Span<Native.Event> buffer = events.Length <= 128
			? stackalloc Native.Event[events.Length]
			: (rented = ArrayPool<Native.Event>.Shared.Rent(events.Length)).AsSpan(0, events.Length);
		try
		{
			fixed (Native.Event* buffer_ptr = buffer)
			{
				Native.Check(Native.Audio_EventsCopy(Handle, buffer_ptr, (uint)buffer.Length, out var required));
				for (var i = 0; i != required; ++i)
					events[i] = buffer[i].ToPublic();

				return checked((int)required);
			}
		}
		finally
		{
			if (rented != null)
				ArrayPool<Native.Event>.Shared.Return(rented);
		}
	}

	/// <summary>Cold convenience that allocates and returns every currently queued event.</summary>
	public AudioEvent[] DrainEvents()
	{
		var count = EventCount();
		if (count == 0)
			return Array.Empty<AudioEvent>();

		var events = new AudioEvent[count];
		CopyEvents(events);
		return events;
	}

	/// <summary>Read generic voice, event, and device diagnostics.</summary>
	public unsafe Diagnostics GetDiagnostics()
	{
		EnsureOwner();
		var diagnostics = new Diagnostics();
		Native.Check(Native.Audio_DiagnosticsGet(Handle, &diagnostics));
		return diagnostics;
	}

	/// <summary>Dispose voices, clips, streams, and the native engine in dependency order.</summary>
	public void Dispose()
	{
		if (m_handle == null)
			return;

		EnsureOwner();
		var voices = new Voice[m_voices.Count];
		m_voices.CopyTo(voices);
		var clips = new Clip[m_clips.Count];
		m_clips.CopyTo(clips);
		var streams = new AudioStream[m_streams.Count];
		m_streams.CopyTo(streams);

		// Voices must be invalidated before the clips they reference; native destruction owns the underlying device teardown.
		Native.Check(Native.Audio_EngineDestroy(Handle));
		foreach (var voice in voices)
			voice.ReleaseFromEngine();
		foreach (var clip in clips)
			clip.ReleaseFromEngine();
		foreach (var stream in streams)
			stream.ReleaseFromEngine();

		m_handle.MarkDestroyed();
		m_handle.Dispose();
		m_handle = null;
		m_runtime.Remove(this);
		GC.SuppressFinalize(this);
	}

	/// <summary>The current stable native engine identity.</summary>
	internal ulong Handle
	{
		get
		{
			var handle = m_handle ?? throw new ObjectDisposedException(nameof(Engine));
			return handle.Value;
		}
	}

	/// <summary>Throw if a query, mutation, or lifetime operation runs on a non-owner OS thread.</summary>
	internal void EnsureOwner()
	{
		if (Native.GetCurrentThreadId() != m_owner_thread_id)
			throw new InvalidOperationException("Audio mutations, queries, and lifetime operations must run on the engine's creating OS thread.");
		if (IsDisposed)
			throw new ObjectDisposedException(nameof(Engine));
	}

	/// <summary>Remove a disposed clip from managed ownership tracking.</summary>
	internal void Remove(Clip clip)
	{
		m_clips.Remove(clip);
	}

	/// <summary>Remove a disposed voice from managed ownership tracking.</summary>
	internal void Remove(Voice voice)
	{
		m_voices.Remove(voice);
	}

	/// <summary>Remove a disposed stream from managed ownership tracking.</summary>
	internal void Remove(AudioStream stream)
	{
		m_streams.Remove(stream);
	}

	/// <summary>Create one resident clip from caller-owned bytes without retaining the managed buffer.</summary>
	private unsafe Clip CreateClipCore(ReadOnlySpan<byte> wave_data)
	{
		EnsureOwner();
		if (wave_data.Length == 0)
			throw new ArgumentException("Wave data must not be empty.", nameof(wave_data));

		fixed (byte* ptr = wave_data)
		{
			Native.Check(Native.Audio_ClipCreateWave(Handle, ptr, (ulong)wave_data.Length, out var handle));
			var clip = new Clip(this, new ClipHandle(handle));
			m_clips.Add(clip);
			return clip;
		}
	}

	/// <summary>Create one decoded stream from caller-owned bytes without retaining the managed buffer.</summary>
	private unsafe AudioStream CreateStreamCore(ReadOnlySpan<byte> ogg_data, StreamOptions? options)
	{
		EnsureOwner();
		if (ogg_data.Length == 0)
			throw new ArgumentException("Ogg Vorbis data must not be empty.", nameof(ogg_data));

		options ??= new StreamOptions();
		var desc = Native.StreamDesc.From(options);
		fixed (byte* ptr = ogg_data)
		{
			Native.Check(Native.Audio_StreamCreateOgg(Handle, &desc, ptr, (ulong)ogg_data.Length, out var handle));
			var stream = new AudioStream(this, new StreamHandle(handle));
			m_streams.Add(stream);
			return stream;
		}
	}

	/// <summary>Provides finalizer-safe cleanup without weakening public owner-thread disposal.</summary>
	private sealed class EngineSafeHandle :SafeHandle
	{
		private readonly AudioRuntime m_runtime;

		/// <summary>Adopt a native generation-aware engine handle.</summary>
		internal EngineSafeHandle(ulong handle, AudioRuntime runtime)
			: base(IntPtr.Zero, true)
		{
			if (IntPtr.Size != sizeof(ulong))
				throw new PlatformNotSupportedException("Rylogic.Audio requires a 64-bit process.");

			m_runtime = runtime;
			SetHandle(unchecked((IntPtr)(long)handle));
		}

		/// <inheritdoc/>
		public override bool IsInvalid
		{
			get
			{
				return handle == IntPtr.Zero;
			}
		}

		/// <summary>The typed unsigned engine identity.</summary>
		internal ulong Value
		{
			get
			{
				return unchecked((ulong)handle.ToInt64());
			}
		}

		/// <summary>Prevent finalizer cleanup after explicit owner-thread destruction succeeds.</summary>
		internal void MarkDestroyed()
		{
			SetHandleAsInvalid();
		}

		/// <inheritdoc/>
		protected override bool ReleaseHandle()
		{
			Native.Audio_EngineAbandon(Value);
			GC.KeepAlive(m_runtime);
			return true;
		}
	}
}
