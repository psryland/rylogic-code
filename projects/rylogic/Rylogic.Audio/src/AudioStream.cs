using System;

namespace Rylogic.Audio;

/// <summary>Owns one generation-aware native decoded audio stream.</summary>
public sealed class AudioStream :IDisposable
{
	private StreamHandle m_handle;

	/// <summary>Adopt a stream created by the owning engine.</summary>
	internal AudioStream(Engine engine, StreamHandle handle)
	{
		Engine = engine;
		m_handle = handle;
	}

	/// <summary>The engine that owns this stream.</summary>
	internal Engine Engine { get; }

	/// <summary>True after this wrapper no longer owns a native stream slot.</summary>
	public bool IsDisposed
	{
		get
		{
			return m_handle.IsNull;
		}
	}

	/// <summary>The stable typed identity used by events and diagnostics.</summary>
	public StreamHandle Handle
	{
		get
		{
			if (m_handle.IsNull)
				throw new ObjectDisposedException(nameof(AudioStream));

			return m_handle;
		}
	}

	/// <summary>Start or resume playback.</summary>
	public void Play()
	{
		Engine.EnsureOwner();
		Native.Check(Native.Audio_StreamPlay(Engine.Handle, Handle.Value));
	}

	/// <summary>Pause playback while retaining the current decode position.</summary>
	public void Pause()
	{
		Engine.EnsureOwner();
		Native.Check(Native.Audio_StreamPause(Engine.Handle, Handle.Value));
	}

	/// <summary>Stop playback and reset the decode position.</summary>
	public void Stop()
	{
		Engine.EnsureOwner();
		Native.Check(Native.Audio_StreamStop(Engine.Handle, Handle.Value));
	}

	/// <summary>Seek to an explicit PCM sample position.</summary>
	public void Seek(ulong pcm_position)
	{
		Engine.EnsureOwner();
		Native.Check(Native.Audio_StreamSeek(Engine.Handle, Handle.Value, pcm_position));
	}

	/// <summary>Read the current transport and decode state.</summary>
	public unsafe StreamState GetState()
	{
		Engine.EnsureOwner();
		var state = new StreamState();
		Native.Check(Native.Audio_StreamStateGet(Engine.Handle, Handle.Value, &state));
		return state;
	}

	/// <summary>Destroy this stream.</summary>
	public void Dispose()
	{
		if (m_handle.IsNull)
			return;

		Engine.EnsureOwner();
		Native.Check(Native.Audio_StreamDestroy(Engine.Handle, m_handle.Value));
		m_handle = default;
		Engine.Remove(this);
		GC.SuppressFinalize(this);
	}

	/// <summary>Invalidate this wrapper after its owning engine destroys the native stream registry.</summary>
	internal void ReleaseFromEngine()
	{
		if (m_handle.IsNull)
			return;

		m_handle = default;
		Engine.Remove(this);
		GC.SuppressFinalize(this);
	}
}
