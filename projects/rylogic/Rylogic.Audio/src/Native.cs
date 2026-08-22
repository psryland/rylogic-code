using System;
using System.Runtime.InteropServices;
using System.Text;
using Rylogic.Interop.Win32;

namespace Rylogic.Audio;

/// <summary>Fixed-layout P/Invoke surface for the versioned native Audio ABI.</summary>
internal static unsafe class Native
{
	internal const string Dll = "audio";
	internal const uint ApiVersion = 0x00010000U;
	internal const uint StructVersion = 1U;
	private static IntPtr m_module;

	/// <summary>Load the configuration-appropriate native runtime before the first P/Invoke.</summary>
	internal static void EnsureLoaded()
	{
		if (m_module != IntPtr.Zero)
			return;

		m_module = Win32.LoadDll(Dll + ".dll", out var load_error);
		if (m_module == IntPtr.Zero)
			throw load_error ?? new DllNotFoundException($"Unable to load {Dll}.dll.");
	}

	[UnmanagedFunctionPointer(CallingConvention.StdCall)]
	internal delegate void ReportErrorFn(IntPtr context, [MarshalAs(UnmanagedType.LPUTF8Str)] string message, [MarshalAs(UnmanagedType.LPUTF8Str)] string filepath, int line);

	[StructLayout(LayoutKind.Sequential)]
	internal struct ReportErrorCallback
	{
		internal IntPtr m_context;
		internal ReportErrorFn m_callback;
	}

	/// <summary>Identifies a public structure available through native ABI size discovery.</summary>
	internal enum EStructId
	{
		Config = 1,
		ListenerState = 2,
		EmitterState = 3,
		VoiceDesc = 4,
		VoiceState = 5,
		Event = 6,
		Diagnostics = 7,
		StreamDesc = 8,
		StreamState = 9,
	}

	[StructLayout(LayoutKind.Sequential)]
	internal struct Config
	{
		internal NativeHeader m_header;
		internal uint m_max_logical_voices;
		internal uint m_max_rendered_voices;
		internal uint m_max_streams;
		internal uint m_event_capacity;
		internal uint m_sample_rate;
		internal uint m_channel_count;
		internal float m_speed_of_sound;
		internal float m_max_engine_buffer_ms;

		/// <summary>Convert caller-facing options into the exact ABI layout.</summary>
		internal static Config From(EngineOptions options)
		{
			return new Config
			{
				m_header = NativeHeader.Create<Config>(),
				m_max_logical_voices = options.MaxLogicalVoices,
				m_max_rendered_voices = options.MaxRenderedVoices,
				m_max_streams = options.MaxStreams,
				m_event_capacity = options.EventCapacity,
				m_sample_rate = options.SampleRate,
				m_channel_count = options.ChannelCount,
				m_speed_of_sound = options.SpeedOfSound,
				m_max_engine_buffer_ms = options.MaxEngineBufferMs,
			};
		}
	}

	[StructLayout(LayoutKind.Sequential)]
	internal struct VoiceDesc
	{
		internal NativeHeader m_header;
		internal ulong m_clip;
		internal EAudioBus m_bus;
		internal int m_spatial;
		internal uint m_loop_count;
		internal uint m_priority;
		internal float m_volume;
		internal float m_pitch;

		/// <summary>Convert a target clip and caller-facing options into the exact ABI layout.</summary>
		internal static VoiceDesc From(ClipHandle clip, VoiceOptions options)
		{
			return new VoiceDesc
			{
				m_header = NativeHeader.Create<VoiceDesc>(),
				m_clip = clip.Value,
				m_bus = options.Bus,
				m_spatial = options.Spatial ? 1 : 0,
				m_loop_count = options.LoopCount,
				m_priority = options.Priority,
				m_volume = options.Volume,
				m_pitch = options.Pitch,
			};
		}
	}

	[StructLayout(LayoutKind.Sequential)]
	internal struct StreamDesc
	{
		internal NativeHeader m_header;
		internal EAudioBus m_bus;
		internal uint m_loop_count;
		internal uint m_priority;
		internal float m_volume;
		internal float m_pitch;

		/// <summary>Convert caller-facing options into the exact ABI layout.</summary>
		internal static StreamDesc From(StreamOptions options)
		{
			return new StreamDesc
			{
				m_header = NativeHeader.Create<StreamDesc>(),
				m_bus = options.Bus,
				m_loop_count = options.LoopCount,
				m_priority = options.Priority,
				m_volume = options.Volume,
				m_pitch = options.Pitch,
			};
		}
	}

	[StructLayout(LayoutKind.Sequential)]
	internal struct Event
	{
		internal NativeHeader m_header;
		internal EAudioEvent m_type;
		internal EStatus m_status;
		internal ulong m_target;
		internal ulong m_sequence;

		/// <summary>Resolve the raw target into a typed voice or stream handle for the event's type.</summary>
		internal AudioEvent ToPublic()
		{
			switch (m_type)
			{
				case EAudioEvent.VoiceEnded:
				case EAudioEvent.VoiceVirtualized:
				case EAudioEvent.VoiceRealized:
				{
					return new AudioEvent(m_type, m_status, m_sequence, new VoiceHandle(m_target), default);
				}
				case EAudioEvent.StreamStopped:
				case EAudioEvent.StreamUnderrun:
				{
					return new AudioEvent(m_type, m_status, m_sequence, default, new StreamHandle(m_target));
				}
				case EAudioEvent.DeviceReset:
				case EAudioEvent.QueueOverflow:
				{
					return new AudioEvent(m_type, m_status, m_sequence, default, default);
				}
				default:
				{
					throw new ArgumentOutOfRangeException(nameof(m_type), m_type, "Unknown audio event type.");
				}
			}
		}
	}

	/// <summary>Throw a managed exception containing the native thread-local error message.</summary>
	internal static void Check(EStatus status)
	{
		if (status == EStatus.Success)
			return;

		uint required;
		Audio_LastError(null, 0, out required);
		var bytes = new byte[Math.Max(required, 1)];
		fixed (byte* ptr = bytes)
		{
			Audio_LastError(ptr, (uint)bytes.Length, out required);
		}
		var count = Array.IndexOf(bytes, (byte)0);
		if (count < 0)
			count = bytes.Length;

		var message = Encoding.UTF8.GetString(bytes, 0, count);
		throw new AudioException(status, string.IsNullOrWhiteSpace(message) ? $"Native audio call failed with status {status}." : message);
	}

	[DllImport(Dll)] internal static extern uint Audio_ApiVersion();
	[DllImport(Dll)] internal static extern IntPtr Audio_Initialise(ReportErrorCallback callback);
	[DllImport(Dll)] internal static extern void Audio_Shutdown(IntPtr context);
	[DllImport(Dll)] internal static extern void Audio_ContextAbandon(IntPtr context);
	[DllImport(Dll)] private static extern EStatus Audio_LastError(byte* buffer, uint capacity, out uint required);
	[DllImport(Dll)] internal static extern EStatus Audio_StructSize(EStructId struct_id, out uint size);

	[DllImport(Dll)] internal static extern EStatus Audio_EngineCreate(IntPtr context, Config* config, out ulong engine);
	[DllImport(Dll)] internal static extern EStatus Audio_EngineDestroy(ulong engine);
	[DllImport(Dll)] internal static extern void Audio_EngineAbandon(ulong engine);
	[DllImport(Dll)] internal static extern EStatus Audio_EngineUpdate(ulong engine);
	[DllImport(Dll)] internal static extern EStatus Audio_ListenerSet(ulong engine, ListenerState* listener);
	[DllImport(Dll)] internal static extern EStatus Audio_BusGainSet(ulong engine, EAudioBus bus, float gain);

	[DllImport(Dll)] internal static extern EStatus Audio_ClipCreateWave(ulong engine, byte* wave_file, ulong wave_file_size, out ulong clip);
	[DllImport(Dll)] internal static extern EStatus Audio_ClipDestroy(ulong engine, ulong clip);

	[DllImport(Dll)] internal static extern EStatus Audio_VoiceCreate(ulong engine, VoiceDesc* desc, out ulong voice);
	[DllImport(Dll)] internal static extern EStatus Audio_VoiceDestroy(ulong engine, ulong voice);
	[DllImport(Dll)] internal static extern EStatus Audio_VoicePlay(ulong engine, ulong voice);
	[DllImport(Dll)] internal static extern EStatus Audio_VoicePause(ulong engine, ulong voice);
	[DllImport(Dll)] internal static extern EStatus Audio_VoiceStop(ulong engine, ulong voice);
	[DllImport(Dll)] internal static extern EStatus Audio_VoiceStateGet(ulong engine, ulong voice, VoiceState* state);
	[DllImport(Dll)] internal static extern EStatus Audio_VoiceEmitterSet(ulong engine, ulong voice, EmitterState* emitter);
	[DllImport(Dll)] internal static extern EStatus Audio_VoiceEmitterClear(ulong engine, ulong voice);

	[DllImport(Dll)] internal static extern EStatus Audio_StreamCreateOgg(ulong engine, StreamDesc* desc, byte* ogg_file, ulong ogg_file_size, out ulong stream);
	[DllImport(Dll)] internal static extern EStatus Audio_StreamDestroy(ulong engine, ulong stream);
	[DllImport(Dll)] internal static extern EStatus Audio_StreamPlay(ulong engine, ulong stream);
	[DllImport(Dll)] internal static extern EStatus Audio_StreamPause(ulong engine, ulong stream);
	[DllImport(Dll)] internal static extern EStatus Audio_StreamStop(ulong engine, ulong stream);
	[DllImport(Dll)] internal static extern EStatus Audio_StreamSeek(ulong engine, ulong stream, ulong pcm_position);
	[DllImport(Dll)] internal static extern EStatus Audio_StreamStateGet(ulong engine, ulong stream, StreamState* state);

	[DllImport(Dll)] internal static extern EStatus Audio_EventsCopy(ulong engine, Event* events, uint capacity, out uint required);
	[DllImport(Dll)] internal static extern EStatus Audio_DiagnosticsGet(ulong engine, Diagnostics* diagnostics);

	[DllImport("kernel32.dll")] internal static extern uint GetCurrentThreadId();
}
