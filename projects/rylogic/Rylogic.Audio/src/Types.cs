using System;
using System.Runtime.InteropServices;
using Rylogic.Maths;

namespace Rylogic.Audio;

/// <summary>Stable result codes returned by the native Audio ABI.</summary>
public enum EStatus
{
	Success = 0,
	InvalidArgument = 1,
	InvalidStruct = 2,
	InvalidHandle = 3,
	StaleHandle = 4,
	WrongThread = 5,
	BufferTooSmall = 6,
	UnsupportedFormat = 7,
	ResourceLimit = 8,
	DeviceLost = 9,
	InternalError = 10,
}

/// <summary>Mixer bus used to group related voices and streams for bulk gain control.</summary>
public enum EAudioBus
{
	Effects = 0,
	Ambience = 1,
	Speech = 2,
	Music = 3,
	Interface = 4,
	Count = 5,
}

/// <summary>Current transport state of a voice or stream.</summary>
public enum EPlaybackState
{
	Stopped = 0,
	Playing = 1,
	Paused = 2,
}

/// <summary>Identifies a buffered runtime notification returned from the engine's owner-thread event queue.</summary>
public enum EAudioEvent
{
	VoiceEnded = 0,
	VoiceVirtualized = 1,
	VoiceRealized = 2,
	DeviceReset = 3,
	StreamStopped = 4,
	QueueOverflow = 5,
	StreamUnderrun = 6,
}

/// <summary>Identifies the encoding of an <see cref="AudioData"/> payload.</summary>
public enum EAudioEncoding
{
	Wave = 0,
	OggVorbis = 1,
}

/// <summary>An immutable generation-aware clip identity with no native pointer semantics.</summary>
[StructLayout(LayoutKind.Sequential)]
public readonly struct ClipHandle :IEquatable<ClipHandle>
{
	private readonly ulong m_value;

	/// <summary>Initialise a typed handle from its stable binary identity.</summary>
	internal ClipHandle(ulong value)
	{
		m_value = value;
	}

	/// <summary>True when this handle identifies no clip.</summary>
	public bool IsNull
	{
		get
		{
			return m_value == 0;
		}
	}

	/// <summary>The fixed-width value passed to the native ABI.</summary>
	internal ulong Value
	{
		get
		{
			return m_value;
		}
	}

	/// <inheritdoc/>
	public bool Equals(ClipHandle other)
	{
		return m_value == other.m_value;
	}

	/// <inheritdoc/>
	public override bool Equals(object? obj)
	{
		return obj is ClipHandle other && Equals(other);
	}

	/// <inheritdoc/>
	public override int GetHashCode()
	{
		return m_value.GetHashCode();
	}

	/// <summary>Compare two typed identities.</summary>
	public static bool operator ==(ClipHandle lhs, ClipHandle rhs)
	{
		return lhs.Equals(rhs);
	}

	/// <summary>Compare two typed identities.</summary>
	public static bool operator !=(ClipHandle lhs, ClipHandle rhs)
	{
		return !lhs.Equals(rhs);
	}
}

/// <summary>An immutable generation-aware voice identity with no native pointer semantics.</summary>
[StructLayout(LayoutKind.Sequential)]
public readonly struct VoiceHandle :IEquatable<VoiceHandle>
{
	private readonly ulong m_value;

	/// <summary>Initialise a typed handle from its stable binary identity.</summary>
	internal VoiceHandle(ulong value)
	{
		m_value = value;
	}

	/// <summary>True when this handle identifies no voice.</summary>
	public bool IsNull
	{
		get
		{
			return m_value == 0;
		}
	}

	/// <summary>The fixed-width value passed to the native ABI.</summary>
	internal ulong Value
	{
		get
		{
			return m_value;
		}
	}

	/// <inheritdoc/>
	public bool Equals(VoiceHandle other)
	{
		return m_value == other.m_value;
	}

	/// <inheritdoc/>
	public override bool Equals(object? obj)
	{
		return obj is VoiceHandle other && Equals(other);
	}

	/// <inheritdoc/>
	public override int GetHashCode()
	{
		return m_value.GetHashCode();
	}

	/// <summary>Compare two typed identities.</summary>
	public static bool operator ==(VoiceHandle lhs, VoiceHandle rhs)
	{
		return lhs.Equals(rhs);
	}

	/// <summary>Compare two typed identities.</summary>
	public static bool operator !=(VoiceHandle lhs, VoiceHandle rhs)
	{
		return !lhs.Equals(rhs);
	}
}

/// <summary>An immutable generation-aware stream identity with no native pointer semantics.</summary>
[StructLayout(LayoutKind.Sequential)]
public readonly struct StreamHandle :IEquatable<StreamHandle>
{
	private readonly ulong m_value;

	/// <summary>Initialise a typed handle from its stable binary identity.</summary>
	internal StreamHandle(ulong value)
	{
		m_value = value;
	}

	/// <summary>True when this handle identifies no stream.</summary>
	public bool IsNull
	{
		get
		{
			return m_value == 0;
		}
	}

	/// <summary>The fixed-width value passed to the native ABI.</summary>
	internal ulong Value
	{
		get
		{
			return m_value;
		}
	}

	/// <inheritdoc/>
	public bool Equals(StreamHandle other)
	{
		return m_value == other.m_value;
	}

	/// <inheritdoc/>
	public override bool Equals(object? obj)
	{
		return obj is StreamHandle other && Equals(other);
	}

	/// <inheritdoc/>
	public override int GetHashCode()
	{
		return m_value.GetHashCode();
	}

	/// <summary>Compare two typed identities.</summary>
	public static bool operator ==(StreamHandle lhs, StreamHandle rhs)
	{
		return lhs.Equals(rhs);
	}

	/// <summary>Compare two typed identities.</summary>
	public static bool operator !=(StreamHandle lhs, StreamHandle rhs)
	{
		return !lhs.Equals(rhs);
	}
}

/// <summary>Configurable engine capacities and physical constants. Pass null to accept the native engine's defaults.</summary>
public sealed class EngineOptions
{
	public uint MaxLogicalVoices { get; set; } = 1024;
	public uint MaxRenderedVoices { get; set; } = 128;
	public uint MaxStreams { get; set; } = 16;
	public uint EventCapacity { get; set; } = 4096;
	public uint SampleRate { get; set; }
	public uint ChannelCount { get; set; }
	public float SpeedOfSound { get; set; } = 343.0f;
	public float MaxEngineBufferMs { get; set; } = 40.0f;
}

/// <summary>
/// Position, orientation, and motion of the engine's single rendered listener. All vectors use a right-handed coordinate
/// system with distances in metres and velocities in metres per second.
/// </summary>
[StructLayout(LayoutKind.Sequential)]
public struct ListenerState
{
	private NativeHeader m_header;
	public v3 m_position;
	public v3 m_forward;
	public v3 m_up;
	public v3 m_velocity;

	/// <summary>Create a listener state with an explicit position, orientation, and velocity.</summary>
	public ListenerState(v3 position, v3 forward, v3 up, v3 velocity)
	{
		m_header = NativeHeader.Create<ListenerState>();
		m_position = position;
		m_forward = forward;
		m_up = up;
		m_velocity = velocity;
	}
}

/// <summary>
/// Spatial properties applied to a mono positional voice. All vectors use a right-handed coordinate system with distances
/// in metres and velocities in metres per second; the cone angles are in radians.
/// </summary>
[StructLayout(LayoutKind.Sequential)]
public struct EmitterState
{
	private NativeHeader m_header;
	public v3 m_position;
	public v3 m_forward;
	public v3 m_up;
	public v3 m_velocity;
	public float m_min_distance;
	public float m_max_distance;
	public float m_cone_inner_angle;
	public float m_cone_outer_angle;
	public float m_cone_outer_gain;
	public float m_doppler_scale;
	public float m_obstruction;
	public float m_occlusion;
	public float m_reverb_send;

	/// <summary>Create an emitter state with every spatial property specified explicitly.</summary>
	public EmitterState(v3 position, v3 forward, v3 up, v3 velocity, float min_distance, float max_distance, float cone_inner_angle, float cone_outer_angle, float cone_outer_gain, float doppler_scale, float obstruction, float occlusion, float reverb_send)
	{
		m_header = NativeHeader.Create<EmitterState>();
		m_position = position;
		m_forward = forward;
		m_up = up;
		m_velocity = velocity;
		m_min_distance = min_distance;
		m_max_distance = max_distance;
		m_cone_inner_angle = cone_inner_angle;
		m_cone_outer_angle = cone_outer_angle;
		m_cone_outer_gain = cone_outer_gain;
		m_doppler_scale = doppler_scale;
		m_obstruction = obstruction;
		m_occlusion = occlusion;
		m_reverb_send = reverb_send;
	}

	/// <summary>Create a non-directional emitter at a position, with no attenuation cone and no doppler, obstruction, or occlusion.</summary>
	public static EmitterState AtPosition(v3 position, float min_distance = 1.0f, float max_distance = 100.0f)
	{
		var full_circle = (float)(2.0 * Math.PI);
		return new EmitterState(position, new v3(0, 0, 1), new v3(0, 1, 0), v3.Zero, min_distance, max_distance, full_circle, full_circle, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f);
	}
}

/// <summary>Configuration applied when creating a voice bound to a clip.</summary>
public sealed class VoiceOptions
{
	/// <summary>A loop count that plays the clip indefinitely until explicitly stopped.</summary>
	public const uint InfiniteLoopCount = 0xFFFFFFFFU;

	public EAudioBus Bus { get; set; } = EAudioBus.Effects;
	public bool Spatial { get; set; }
	public uint LoopCount { get; set; }
	public uint Priority { get; set; } = 100;
	public float Volume { get; set; } = 1.0f;
	public float Pitch { get; set; } = 1.0f;
}

/// <summary>Configuration applied when creating a stream decoding an Ogg Vorbis payload.</summary>
public sealed class StreamOptions
{
	/// <summary>A loop count that plays the stream indefinitely until explicitly stopped.</summary>
	public const uint InfiniteLoopCount = 0xFFFFFFFFU;

	public EAudioBus Bus { get; set; } = EAudioBus.Effects;
	public uint LoopCount { get; set; }
	public uint Priority { get; set; } = 100;
	public float Volume { get; set; } = 1.0f;
	public float Pitch { get; set; } = 1.0f;
}

/// <summary>Caller-owned observable transport state for one voice.</summary>
[StructLayout(LayoutKind.Sequential)]
public struct VoiceState
{
	private NativeHeader m_header;
	public EPlaybackState m_playback;
	private int m_spatial;
	private int m_virtualized;
	private uint m_reserved;
	public ulong m_samples_played;

	/// <summary>True when the voice was created with spatial emitter properties.</summary>
	public bool Spatial
	{
		get
		{
			return m_spatial != 0;
		}
	}

	/// <summary>True while the voice has been silenced but is still tracked, due to rendered-voice pressure.</summary>
	public bool Virtualized
	{
		get
		{
			return m_virtualized != 0;
		}
	}
}

/// <summary>Caller-owned observable transport and decode state for one stream.</summary>
[StructLayout(LayoutKind.Sequential)]
public struct StreamState
{
	private NativeHeader m_header;
	public EPlaybackState m_playback;
	public uint m_channel_count;
	public uint m_sample_rate;
	public ulong m_pcm_position;
	public ulong m_pcm_total;
	public uint m_underrun_count;
}

/// <summary>A buffered runtime notification copied from the engine's owner-thread event queue.</summary>
public readonly struct AudioEvent
{
	public readonly EAudioEvent m_type;
	public readonly EStatus m_status;
	public readonly ulong m_sequence;
	public readonly VoiceHandle m_voice;
	public readonly StreamHandle m_stream;

	/// <summary>Create an event with its target already resolved to a typed voice or stream handle.</summary>
	internal AudioEvent(EAudioEvent type, EStatus status, ulong sequence, VoiceHandle voice, StreamHandle stream)
	{
		m_type = type;
		m_status = status;
		m_sequence = sequence;
		m_voice = voice;
		m_stream = stream;
	}
}

/// <summary>Bounded runtime counts useful for diagnostics and stress tests.</summary>
[StructLayout(LayoutKind.Sequential)]
public struct Diagnostics
{
	private NativeHeader m_header;
	public uint m_logical_voice_count;
	public uint m_rendered_voice_count;
	public uint m_playing_voice_count;
	public uint m_virtualized_voice_count;
	public uint m_queued_event_count;
	public uint m_event_overflow_count;
	public uint m_device_reset_count;
	public uint m_output_channel_count;
	public uint m_output_sample_rate;
	public float m_engine_buffer_ms;
}

/// <summary>Base exception for failures reported by the native Audio ABI.</summary>
public sealed class AudioException :Exception
{
	/// <summary>Create an exception for one native status and message.</summary>
	internal AudioException(EStatus status, string message)
		: base(message)
	{
		Status = status;
	}

	/// <summary>The stable native status code.</summary>
	public EStatus Status { get; }
}

[StructLayout(LayoutKind.Sequential)]
internal readonly struct NativeHeader
{
	internal readonly uint m_size;
	internal readonly uint m_version;

	internal NativeHeader(uint size)
	{
		m_size = size;
		m_version = Native.StructVersion;
	}

	internal static NativeHeader Create<T>()
	{
		return new NativeHeader(NativeSize<T>.Value);
	}
}

/// <summary>Caches one managed ABI record size so hot construction performs no reflection.</summary>
internal static class NativeSize<T>
{
	internal static readonly uint Value = (uint)Marshal.SizeOf(typeof(T));
}
