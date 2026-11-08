using System;

namespace Rylogic.Audio;

/// <summary>Owns one generation-aware native voice bound to a clip.</summary>
public sealed class Voice :IDisposable
{
	private VoiceHandle m_handle;

	/// <summary>Adopt a voice created by the owning engine.</summary>
	internal Voice(Engine engine, Clip clip, VoiceHandle handle)
	{
		Engine = engine;
		Clip = clip;
		m_handle = handle;
	}

	/// <summary>The engine that owns this voice.</summary>
	internal Engine Engine { get; }

	/// <summary>The clip this voice plays.</summary>
	public Clip Clip { get; }

	/// <summary>True after this wrapper no longer owns a native voice slot.</summary>
	public bool IsDisposed
	{
		get
		{
			return m_handle.IsNull;
		}
	}

	/// <summary>The stable typed identity used by events and diagnostics.</summary>
	public VoiceHandle Handle
	{
		get
		{
			if (m_handle.IsNull)
				throw new ObjectDisposedException(nameof(Voice));

			return m_handle;
		}
	}

	/// <summary>Start or resume playback.</summary>
	public void Play()
	{
		Engine.EnsureOwner();
		Native.Check(Native.Audio_VoicePlay(Engine.Handle, Handle.Value));
	}

	/// <summary>Pause playback while retaining the current sample position.</summary>
	public void Pause()
	{
		Engine.EnsureOwner();
		Native.Check(Native.Audio_VoicePause(Engine.Handle, Handle.Value));
	}

	/// <summary>Stop playback and reset the sample position.</summary>
	public void Stop()
	{
		Engine.EnsureOwner();
		Native.Check(Native.Audio_VoiceStop(Engine.Handle, Handle.Value));
	}

	/// <summary>Read the current transport state.</summary>
	public unsafe VoiceState GetState()
	{
		Engine.EnsureOwner();
		var state = new VoiceState();
		Native.Check(Native.Audio_VoiceStateGet(Engine.Handle, Handle.Value, &state));
		return state;
	}

	/// <summary>Replace the spatial emitter properties, making this a positional voice.</summary>
	public unsafe void SetEmitter(EmitterState emitter)
	{
		Engine.EnsureOwner();
		Native.Check(Native.Audio_VoiceEmitterSet(Engine.Handle, Handle.Value, &emitter));
	}

	/// <summary>Clear any spatial emitter properties, making this a non-positional voice.</summary>
	public void ClearEmitter()
	{
		Engine.EnsureOwner();
		Native.Check(Native.Audio_VoiceEmitterClear(Engine.Handle, Handle.Value));
	}

	/// <summary>Move the playing voice's linear gain to 'gain' over 'ramp_seconds'. Zero applies without interpolation on the next <see cref="Engine.Update"/>.</summary>
	public void SetGain(float gain, float ramp_seconds = 0.0f)
	{
		Engine.EnsureOwner();
		Native.Check(Native.Audio_VoiceGainSet(Engine.Handle, Handle.Value, gain, ramp_seconds));
	}

	/// <summary>Move the playing voice's pitch ratio to 'pitch' over 'ramp_seconds'. Zero applies without interpolation on the next <see cref="Engine.Update"/>.</summary>
	public void SetPitch(float pitch, float ramp_seconds = 0.0f)
	{
		Engine.EnsureOwner();
		Native.Check(Native.Audio_VoicePitchSet(Engine.Handle, Handle.Value, pitch, ramp_seconds));
	}

	/// <summary>
	/// Move the playing voice's signed stereo width in [-1,1] to 'width' (see VoiceOptions.Width) over 'ramp_seconds'. Zero applies without interpolation on the next <see cref="Engine.Update"/>.
	/// Width affects only non-spatial stereo clips.
	/// </summary>
	public void SetWidth(float width, float ramp_seconds = 0.0f)
	{
		Engine.EnsureOwner();
		Native.Check(Native.Audio_VoiceWidthSet(Engine.Handle, Handle.Value, width, ramp_seconds));
	}

	/// <summary>
	/// Move the playing voice's left/right balance in [-1,1] to 'pan' over 'ramp_seconds'. Zero applies without interpolation on the next <see cref="Engine.Update"/>.
	/// Pan affects only non-spatial stereo clips. See <see cref="VoiceOptions.Pan"/>.
	/// </summary>
	public void SetPan(float pan, float ramp_seconds = 0.0f)
	{
		Engine.EnsureOwner();
		Native.Check(Native.Audio_VoicePanSet(Engine.Handle, Handle.Value, pan, ramp_seconds));
	}

	/// <summary>Destroy this voice and deregister it from the clip it played.</summary>
	public void Dispose()
	{
		if (m_handle.IsNull)
			return;

		Engine.EnsureOwner();
		Native.Check(Native.Audio_VoiceDestroy(Engine.Handle, m_handle.Value));
		m_handle = default;
		Clip.RemoveVoice(this);
		Engine.Remove(this);
		GC.SuppressFinalize(this);
	}

	/// <summary>Invalidate this wrapper after its owning engine destroys the native voice registry.</summary>
	internal void ReleaseFromEngine()
	{
		if (m_handle.IsNull)
			return;

		m_handle = default;
		Clip.RemoveVoice(this);
		Engine.Remove(this);
		GC.SuppressFinalize(this);
	}
}
