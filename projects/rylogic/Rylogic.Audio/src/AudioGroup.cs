using System;

namespace Rylogic.Audio;

/// <summary>
/// Owns one generation-aware native submix group. Voices, streams, and child groups routed through a group share its gain and
/// low-/high-pass filters. A group cannot be disposed while any child group, voice, or stream still routes through it.
/// </summary>
public sealed class AudioGroup :IDisposable
{
	private GroupHandle m_handle;

	/// <summary>Adopt a group created by the owning engine.</summary>
	internal AudioGroup(Engine engine, AudioGroup? parent, GroupHandle handle)
	{
		Engine = engine;
		Parent = parent;
		m_handle = handle;
	}

	/// <summary>The engine that owns this group.</summary>
	internal Engine Engine { get; }

	/// <summary>The group this group feeds, or null when it feeds the master output.</summary>
	public AudioGroup? Parent { get; }

	/// <summary>True after this wrapper no longer owns a native group slot.</summary>
	public bool IsDisposed
	{
		get
		{
			return m_handle.IsNull;
		}
	}

	/// <summary>The stable typed identity passed to the native ABI.</summary>
	public GroupHandle Handle
	{
		get
		{
			if (m_handle.IsNull)
				throw new ObjectDisposedException(nameof(AudioGroup));

			return m_handle;
		}
	}

	/// <summary>Move the group's linear gain to 'gain' over 'ramp_seconds' (0 applies immediately).</summary>
	public void SetGain(float gain, float ramp_seconds = 0.0f)
	{
		Engine.EnsureOwner();
		Native.Check(Native.Audio_GroupGainSet(Engine.Handle, Handle.Value, gain, ramp_seconds));
	}

	/// <summary>Move the group's cutoffs (Hz) to new values over 'ramp_seconds' (0 applies immediately). See <see cref="GroupOptions"/> for the open values.</summary>
	public void SetFilter(float low_pass_hz, float high_pass_hz, float ramp_seconds = 0.0f)
	{
		Engine.EnsureOwner();
		Native.Check(Native.Audio_GroupFilterSet(Engine.Handle, Handle.Value, low_pass_hz, high_pass_hz, ramp_seconds));
	}

	/// <summary>Destroy this group. Child groups, voices, and streams routed through it must be disposed first.</summary>
	public void Dispose()
	{
		if (m_handle.IsNull)
			return;

		Engine.EnsureOwner();
		Native.Check(Native.Audio_GroupDestroy(Engine.Handle, m_handle.Value));
		m_handle = default;
		Engine.Remove(this);
		GC.SuppressFinalize(this);
	}

	/// <summary>Invalidate this wrapper after its owning engine destroys the native group registry.</summary>
	internal void ReleaseFromEngine()
	{
		if (m_handle.IsNull)
			return;

		m_handle = default;
		Engine.Remove(this);
		GC.SuppressFinalize(this);
	}
}
