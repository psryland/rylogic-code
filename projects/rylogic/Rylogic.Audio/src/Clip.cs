using System;
using System.Collections.Generic;

namespace Rylogic.Audio;

/// <summary>Owns one generation-aware native audio clip and tracks the voices that reference it.</summary>
public sealed class Clip :IDisposable
{
	private readonly HashSet<Voice> m_voices;
	private ClipHandle m_handle;

	/// <summary>Adopt a clip created by the owning engine.</summary>
	internal Clip(Engine engine, ClipHandle handle)
	{
		Engine = engine;
		m_handle = handle;
		m_voices = new HashSet<Voice>();
	}

	/// <summary>The engine that owns this clip.</summary>
	internal Engine Engine { get; }

	/// <summary>True after this wrapper no longer owns a native clip slot.</summary>
	public bool IsDisposed
	{
		get
		{
			return m_handle.IsNull;
		}
	}

	/// <summary>The stable typed identity used to create voices that play this clip.</summary>
	public ClipHandle Handle
	{
		get
		{
			if (m_handle.IsNull)
				throw new ObjectDisposedException(nameof(Clip));

			return m_handle;
		}
	}

	/// <summary>The number of voices currently referencing this clip.</summary>
	public int VoiceCount
	{
		get
		{
			return m_voices.Count;
		}
	}

	/// <summary>Destroy this clip. Rejected while any voice created from it has not yet been disposed.</summary>
	public void Dispose()
	{
		if (m_handle.IsNull)
			return;

		Engine.EnsureOwner();
		// The engine owns dependency ordering only when it tears down everything at once; a single clip must reject
		// disposal while it is still in use so a live voice can never be left referencing a destroyed clip.
		if (m_voices.Count != 0)
			throw new InvalidOperationException("The clip cannot be disposed while voices still reference it.");

		Native.Check(Native.Audio_ClipDestroy(Engine.Handle, m_handle.Value));
		m_handle = default;
		Engine.Remove(this);
		GC.SuppressFinalize(this);
	}

	/// <summary>Invalidate this wrapper after its owning engine destroys the native clip registry.</summary>
	internal void ReleaseFromEngine()
	{
		if (m_handle.IsNull)
			return;

		m_handle = default;
		Engine.Remove(this);
		GC.SuppressFinalize(this);
	}

	/// <summary>Register a voice that now references this clip.</summary>
	internal void AddVoice(Voice voice)
	{
		m_voices.Add(voice);
	}

	/// <summary>Deregister a voice that no longer references this clip.</summary>
	internal void RemoveVoice(Voice voice)
	{
		m_voices.Remove(voice);
	}
}
