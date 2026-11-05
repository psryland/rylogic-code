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

	/// <summary>Move the stream's linear gain to 'gain' over 'ramp_seconds'. Zero applies without interpolation on the next <see cref="Engine.Update"/>.</summary>
	public void SetGain(float gain, float ramp_seconds = 0.0f)
	{
		Engine.EnsureOwner();
		Native.Check(Native.Audio_StreamGainSet(Engine.Handle, Handle.Value, gain, ramp_seconds));
	}

	/// <summary>Move the stream's pitch ratio to 'pitch' over 'ramp_seconds'. Zero applies without interpolation on the next <see cref="Engine.Update"/>.</summary>
	public void SetPitch(float pitch, float ramp_seconds = 0.0f)
	{
		Engine.EnsureOwner();
		Native.Check(Native.Audio_StreamPitchSet(Engine.Handle, Handle.Value, pitch, ramp_seconds));
	}

	/// <summary>
	/// Move the stereo width in [0,1] to 'width' over 'ramp_seconds'. Zero applies without interpolation on the next <see cref="Engine.Update"/>.
	/// Width affects only non-spatial stereo streams.
	/// </summary>
	public void SetWidth(float width, float ramp_seconds = 0.0f)
	{
		Engine.EnsureOwner();
		Native.Check(Native.Audio_StreamWidthSet(Engine.Handle, Handle.Value, width, ramp_seconds));
	}

	/// <summary>
	/// Move the stream's left/right balance in [-1,1] to 'pan' over 'ramp_seconds'. Zero applies without interpolation on the next <see cref="Engine.Update"/>.
	/// Pan affects only non-spatial stereo streams. See <see cref="VoiceOptions.Pan"/>.
	/// </summary>
	public void SetPan(float pan, float ramp_seconds = 0.0f)
	{
		Engine.EnsureOwner();
		Native.Check(Native.Audio_StreamPanSet(Engine.Handle, Handle.Value, pan, ramp_seconds));
	}

	/// <summary>Replace the spatial emitter properties, making this a positional stream. The stream must decode to a single channel.</summary>
	public unsafe void SetEmitter(EmitterState emitter)
	{
		Engine.EnsureOwner();
		Native.Check(Native.Audio_StreamEmitterSet(Engine.Handle, Handle.Value, &emitter));
	}

	/// <summary>Clear any spatial emitter properties, making this a non-positional stream.</summary>
	public void ClearEmitter()
	{
		Engine.EnsureOwner();
		Native.Check(Native.Audio_StreamEmitterClear(Engine.Handle, Handle.Value));
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
