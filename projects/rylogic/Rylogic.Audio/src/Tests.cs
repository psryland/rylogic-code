#if PR_UNITTESTS
using System;
using System.IO;
using System.Runtime.InteropServices;
using System.Threading;
using Rylogic.Maths;
using Rylogic.UnitTests;

namespace Rylogic.Audio;

/// <summary>Validates managed/native layouts, ownership, playback, events, diagnostics, and data loading.</summary>
[TestFixture]
public sealed class TestAudio
{
	/// <summary>Keep managed blittable records aligned with the ABI-reported native sizes.</summary>
	[Test]
	public unsafe void Layouts()
	{
		// The native struct-size query requires a live context, so hold a runtime for the duration of the check.
		using var runtime = new AudioRuntime();
		Assert.Equal(Native.ApiVersion, Native.Audio_ApiVersion());
		Assert.Equal(8, Marshal.SizeOf<NativeHeader>());
		Assert.Equal(8, sizeof(ClipHandle));
		Assert.Equal(8, sizeof(VoiceHandle));
		Assert.Equal(8, sizeof(StreamHandle));
		AssertNativeSize(Native.EStructId.Config, Marshal.SizeOf<Native.Config>());
		AssertNativeSize(Native.EStructId.ListenerState, Marshal.SizeOf<ListenerState>());
		AssertNativeSize(Native.EStructId.EmitterState, Marshal.SizeOf<EmitterState>());
		AssertNativeSize(Native.EStructId.VoiceDesc, Marshal.SizeOf<Native.VoiceDesc>());
		AssertNativeSize(Native.EStructId.VoiceState, Marshal.SizeOf<VoiceState>());
		AssertNativeSize(Native.EStructId.Event, Marshal.SizeOf<Native.Event>());
		AssertNativeSize(Native.EStructId.Diagnostics, Marshal.SizeOf<Diagnostics>());
		AssertNativeSize(Native.EStructId.StreamDesc, Marshal.SizeOf<Native.StreamDesc>());
		AssertNativeSize(Native.EStructId.StreamState, Marshal.SizeOf<StreamState>());
	}

	/// <summary>Exercise multiple voices per clip, clip disposal rejection, and dependency-ordered engine disposal.</summary>
	[Test]
	public void Lifecycle()
	{
		using var runtime = new AudioRuntime();
		var engine = runtime.CreateEngine();
		var clip = engine.CreateClip(AudioData.FromWave(MakeWave()));

		var voice_a = engine.CreateVoice(clip);
		var voice_b = engine.CreateVoice(clip);
		Assert.Equal(2, clip.VoiceCount);

		voice_a.Play();
		Assert.Equal(EPlaybackState.Playing, voice_a.GetState().m_playback);
		voice_a.Pause();
		Assert.Equal(EPlaybackState.Paused, voice_a.GetState().m_playback);
		voice_a.Stop();
		Assert.Equal(EPlaybackState.Stopped, voice_a.GetState().m_playback);

		// A clip cannot be destroyed while any voice still references it.
		Assert.Throws<InvalidOperationException>(() => clip.Dispose());

		voice_a.Dispose();
		Assert.Equal(1, clip.VoiceCount);
		voice_b.Dispose();
		Assert.Equal(0, clip.VoiceCount);
		clip.Dispose();

		// Engine disposal invalidates every remaining child regardless of dependency order.
		var second_clip = engine.CreateClip(AudioData.FromWave(MakeWave()));
		var second_voice = engine.CreateVoice(second_clip);
		var stream_data = AudioData.Load(FixturePath);
		var stream = engine.CreateStream(stream_data);
		engine.Dispose();

		Assert.True(second_voice.IsDisposed);
		Assert.True(second_clip.IsDisposed);
		Assert.True(stream.IsDisposed);
		Assert.True(engine.IsDisposed);
	}

	/// <summary>Reject owner-thread queries and mutations before a call crosses the managed/native boundary.</summary>
	[Test]
	public void ThreadAffinity()
	{
		using var runtime = new AudioRuntime();
		using var engine = runtime.CreateEngine();
		Exception? error = null;
		var thread = new Thread(() =>
		{
			try
			{
				engine.Update();
			}
			catch (Exception ex)
			{
				error = ex;
			}
		});
		thread.Start();
		thread.Join();
		Assert.Equal(typeof(InvalidOperationException), error?.GetType());
	}

	/// <summary>Set the listener and a spatial emitter, and confirm the spatial flag is observable on the voice.</summary>
	[Test]
	public void ListenerAndEmitter()
	{
		using var runtime = new AudioRuntime();
		using var engine = runtime.CreateEngine();
		engine.SetListener(new ListenerState(v3.Zero, new v3(0, 0, 1), new v3(0, 1, 0), v3.Zero));
		engine.SetBusGain(EAudioBus.Effects, 0.5f);

		using var clip = engine.CreateClip(AudioData.FromWave(MakeWave()));
		using var voice = engine.CreateVoice(clip, new VoiceOptions { Spatial = true });
		voice.SetEmitter(EmitterState.AtPosition(new v3(1, 0, 0)));
		Assert.True(voice.GetState().Spatial);

		voice.ClearEmitter();
	}

	/// <summary>Play a short clip to completion and confirm the typed voice-ended event, count, copy, and drain.</summary>
	[Test]
	public void Events()
	{
		using var runtime = new AudioRuntime();
		using var engine = runtime.CreateEngine();
		using var clip = engine.CreateClip(AudioData.FromWave(MakeWave()));
		using var voice = engine.CreateVoice(clip);
		voice.Play();

		// Other native events (e.g. voice virtualisation) may be queued before the terminal event, so accumulate
		// across repeated bounded polls rather than assuming the first non-empty drain already contains it.
		var found = false;
		for (var attempt = 0; !found && attempt != 300; ++attempt)
		{
			engine.Update();
			foreach (var evt in engine.DrainEvents())
			{
				if (evt.m_type != EAudioEvent.VoiceEnded)
					continue;

				Assert.Equal(EStatus.Success, evt.m_status);
				Assert.Equal(voice.Handle, evt.m_voice);
				Assert.True(evt.m_stream.IsNull);
				found = true;
			}
			if (!found)
				Thread.Sleep(10);
		}
		Assert.True(found);
	}

	/// <summary>Expose logical/rendered voice pressure as typed managed virtualisation and realization events.</summary>
	[Test]
	public void VoiceVirtualisation()
	{
		using var runtime = new AudioRuntime();
		using var engine = runtime.CreateEngine(new EngineOptions
		{
			MaxLogicalVoices = 2,
			MaxRenderedVoices = 1,
			MaxStreams = 1,
			EventCapacity = 32,
		});
		using var clip = engine.CreateClip(MakeWave(sample_rate: 8000, seconds: 1.0));
		using var low = engine.CreateVoice(clip, new VoiceOptions
		{
			LoopCount = VoiceOptions.InfiniteLoopCount,
			Priority = 10,
			Volume = 0.0f,
		});
		using var high = engine.CreateVoice(clip, new VoiceOptions
		{
			LoopCount = VoiceOptions.InfiniteLoopCount,
			Priority = 100,
			Volume = 0.0f,
		});

		low.Play();
		high.Play();
		engine.Update();
		Assert.True(low.GetState().Virtualized);
		Assert.False(high.GetState().Virtualized);

		var events = engine.DrainEvents();
		Assert.True(Array.Exists(events, evt => evt.m_type == EAudioEvent.VoiceVirtualized && evt.m_voice == low.Handle));
		Assert.True(Array.Exists(events, evt => evt.m_type == EAudioEvent.VoiceRealized && evt.m_voice == high.Handle));

		high.Stop();
		engine.Update();
		Assert.False(low.GetState().Virtualized);
		Assert.Equal(1U, engine.GetDiagnostics().m_rendered_voice_count);
	}

	/// <summary>Read generic voice, event, and device diagnostics while a voice plays.</summary>
	[Test]
	public void Diagnostics()
	{
		using var runtime = new AudioRuntime();
		using var engine = runtime.CreateEngine();
		using var clip = engine.CreateClip(AudioData.FromWave(MakeWave()));
		using var voice = engine.CreateVoice(clip);
		voice.Play();

		var diagnostics = engine.GetDiagnostics();
		Assert.True(diagnostics.m_logical_voice_count >= 1);
		Assert.True(diagnostics.m_output_sample_rate > 0);

		voice.Stop();
	}

	/// <summary>Exercise Ogg lifecycle, gapless short looping, seek, natural completion, and malformed rejection.</summary>
	[Test]
	public void OggStreamLifecycle()
	{
		using var runtime = new AudioRuntime();
		using var engine = runtime.CreateEngine();
		var data = AudioData.Load(FixturePath);
		Assert.Equal(EAudioEncoding.OggVorbis, data.Encoding);

		using (var stream = engine.CreateStream(data))
		{
			Assert.Equal(EPlaybackState.Stopped, stream.GetState().m_playback);
			stream.Play();

			var completed = false;
			for (var attempt = 0; attempt != 300 && !completed; ++attempt)
			{
				engine.Update();
				foreach (var evt in engine.DrainEvents())
				{
					Assert.NotEqual(EAudioEvent.StreamUnderrun, evt.m_type);
					if (evt.m_type == EAudioEvent.StreamStopped && evt.m_stream == stream.Handle)
						completed = true;
				}
				if (!completed)
					Thread.Sleep(10);
			}
			Assert.True(completed);
			Assert.Equal(EPlaybackState.Stopped, stream.GetState().m_playback);
			Assert.Equal(0U, stream.GetState().m_underrun_count);
		}

		using (var stream = engine.CreateStream(data, new StreamOptions
		{
			LoopCount = StreamOptions.InfiniteLoopCount,
		}))
		{
			stream.Play();
			for (var attempt = 0; attempt != 100; ++attempt)
			{
				engine.Update();
				foreach (var evt in engine.DrainEvents())
					Assert.NotEqual(EAudioEvent.StreamUnderrun, evt.m_type);

				Thread.Sleep(5);
			}
			Assert.Equal(0U, stream.GetState().m_underrun_count);
			stream.Pause();
			Assert.Equal(EPlaybackState.Paused, stream.GetState().m_playback);
			stream.Seek(0);
			stream.Stop();
			Assert.Equal(EPlaybackState.Stopped, stream.GetState().m_playback);
		}

		// A buffer that merely starts with the Ogg page magic but has no valid page content is rejected by the decoder.
		var malformed = AudioData.FromOggVorbis(new byte[] { (byte)'O', (byte)'g', (byte)'g', (byte)'S', 0, 0, 0, 0 });
		ExpectStatus(EStatus.UnsupportedFormat, () => engine.CreateStream(malformed));
	}

	/// <summary>Exercise in-memory copying, magic-byte detection, synchronous and asynchronous loading, and cancellation.</summary>
	[Test]
	public void AudioDataLoading()
	{
		var wave_bytes = MakeWave();
		var expected = (byte[])wave_bytes.Clone();
		var data = AudioData.FromWave(wave_bytes);
		Assert.Equal(EAudioEncoding.Wave, data.Encoding);

		// Mutating the caller's buffer after construction must not affect the copied payload.
		wave_bytes[0] ^= 0xFF;
		Assert.Equal(expected.Length, data.Span.Length);
		Assert.True(data.Span.SequenceEqual(expected));

		// The direct span overload also relies on the native copy-on-create contract.
		using (var direct_runtime = new AudioRuntime())
		using (var direct_engine = direct_runtime.CreateEngine())
		{
			var direct_bytes = MakeWave();
			using var direct_clip = direct_engine.CreateClip(direct_bytes.AsSpan());
			Array.Clear(direct_bytes, 0, direct_bytes.Length);
			using var direct_voice = direct_engine.CreateVoice(direct_clip);
			direct_voice.Play();
			Assert.Equal(EPlaybackState.Playing, direct_voice.GetState().m_playback);
		}

		var temp_wave = Path.Combine(Path.GetTempPath(), Path.GetRandomFileName() + ".wav");
		var temp_ogg = Path.Combine(Path.GetTempPath(), Path.GetRandomFileName() + ".ogg");
		var temp_bin = Path.Combine(Path.GetTempPath(), Path.GetRandomFileName() + ".bin");
		try
		{
			File.WriteAllBytes(temp_wave, MakeWave());
			var loaded_wave = AudioData.Load(temp_wave);
			Assert.Equal(EAudioEncoding.Wave, loaded_wave.Encoding);

			File.Copy(FixturePath, temp_ogg, true);
			var loaded_ogg = AudioData.LoadAsync(temp_ogg).GetAwaiter().GetResult();
			Assert.Equal(EAudioEncoding.OggVorbis, loaded_ogg.Encoding);

			using var cancelled = new CancellationTokenSource();
			cancelled.Cancel();
			Assert.Throws<OperationCanceledException>(() => AudioData.LoadAsync(temp_ogg, cancelled.Token).GetAwaiter().GetResult());

			File.WriteAllBytes(temp_bin, new byte[] { 1, 2, 3, 4 });
			Assert.Throws<NotSupportedException>(() => AudioData.Load(temp_bin));
		}
		finally
		{
			File.Delete(temp_wave);
			File.Delete(temp_ogg);
			File.Delete(temp_bin);
		}

		// An encoding mismatch is rejected before any native call is made.
		using var runtime = new AudioRuntime();
		using var engine = runtime.CreateEngine();
		var ogg_data = AudioData.Load(FixturePath);
		Assert.Throws<ArgumentException>(() => engine.CreateClip(ogg_data));
		Assert.Throws<ArgumentException>(() => engine.CreateStream(data));
	}

	/// <summary>Reject null and cross-engine arguments, disposed-handle access, and an under-sized event destination.</summary>
	[Test]
	public void ValidationAndMisuse()
	{
		using var runtime = new AudioRuntime();
		using var first = runtime.CreateEngine();
		using var second = runtime.CreateEngine();

		Assert.Throws<ArgumentNullException>(() => first.CreateClip((AudioData)null!));
		Assert.Throws<ArgumentNullException>(() => first.CreateStream((AudioData)null!));

		var clip = first.CreateClip(AudioData.FromWave(MakeWave()));
		Assert.Throws<ArgumentNullException>(() => first.CreateVoice(null!));

		// A clip must belong to the engine used to create a voice from it.
		Assert.Throws<ArgumentException>(() => second.CreateVoice(clip));

		var voice = first.CreateVoice(clip);
		voice.Dispose();
		clip.Dispose();

		Assert.Throws<ObjectDisposedException>(() => { var _ = clip.Handle; });
		Assert.Throws<ObjectDisposedException>(() => { var _ = voice.Handle; });
		Assert.Throws<ObjectDisposedException>(() => voice.Play());

		var disposed_stream = first.CreateStream(AudioData.Load(FixturePath));
		disposed_stream.Dispose();
		Assert.Throws<ObjectDisposedException>(() => disposed_stream.GetState());

		// A destination shorter than the queued event count is rejected without a partial copy.
		var another_clip = first.CreateClip(AudioData.FromWave(MakeWave()));
		var another_voice = first.CreateVoice(another_clip);
		another_voice.Play();
		WaitUntilEventsQueued(first);
		ExpectStatus(EStatus.BufferTooSmall, () => first.CopyEvents(Array.Empty<AudioEvent>()));
		first.DrainEvents();

		another_voice.Dispose();
		another_clip.Dispose();
	}

	/// <summary>The full path to the small embedded Ogg Vorbis fixture copied alongside the test assembly.</summary>
	private static string FixturePath
	{
		get
		{
			// 'AppContext.BaseDirectory' reflects the hosting process (the post-build test runner loads this
			// assembly via 'Add-Type' into PowerShell), so resolve the fixture relative to this assembly's file location instead.
			var assembly_dir = Path.GetDirectoryName(typeof(TestAudio).Assembly.Location) ?? AppContext.BaseDirectory;
			return Path.Combine(assembly_dir, "testdata", "fixture.ogg");
		}
	}

	/// <summary>Build a short mono 16-bit PCM wave file so voice-ended tests complete quickly.</summary>
	private static byte[] MakeWave(int sample_rate = 8000, double seconds = 0.05)
	{
		const int channels = 1;
		const int bits_per_sample = 16;
		var sample_count = (int)(sample_rate * seconds);
		var block_align = channels * (bits_per_sample / 8);
		var data_size = sample_count * block_align;
		var byte_rate = sample_rate * block_align;

		using var stream = new MemoryStream();
		using var writer = new BinaryWriter(stream);
		writer.Write(new[] { 'R', 'I', 'F', 'F' });
		writer.Write(36 + data_size);
		writer.Write(new[] { 'W', 'A', 'V', 'E' });
		writer.Write(new[] { 'f', 'm', 't', ' ' });
		writer.Write(16);
		writer.Write((short)1);
		writer.Write((short)channels);
		writer.Write(sample_rate);
		writer.Write(byte_rate);
		writer.Write((short)block_align);
		writer.Write((short)bits_per_sample);
		writer.Write(new[] { 'd', 'a', 't', 'a' });
		writer.Write(data_size);
		for (var i = 0; i != sample_count; ++i)
		{
			var t = (double)i / sample_rate;
			var value = (short)(Math.Sin(2.0 * Math.PI * 440.0 * t) * short.MaxValue * 0.25);
			writer.Write(value);
		}
		writer.Flush();
		return stream.ToArray();
	}

	/// <summary>Poll the engine's owner-thread update until at least one event is queued, without consuming it.</summary>
	private static void WaitUntilEventsQueued(Engine engine, int max_attempts = 300, int delay_ms = 10)
	{
		for (var attempt = 0; attempt != max_attempts; ++attempt)
		{
			engine.Update();
			if (engine.EventCount() > 0)
				return;

			Thread.Sleep(delay_ms);
		}
		throw new UnitTestException("Timed out waiting for a queued audio event.");
	}

	/// <summary>Compare one managed blittable record against native ABI discovery.</summary>
	private static void AssertNativeSize(Native.EStructId struct_id, int managed_size)
	{
		Native.Check(Native.Audio_StructSize(struct_id, out var native_size));
		Assert.Equal((uint)managed_size, native_size);
	}

	/// <summary>Require one native call to fail with a specific stable ABI status.</summary>
	private static void ExpectStatus(EStatus expected, Action action)
	{
		try
		{
			action();
			throw new UnitTestException($"Expected native status {expected}.");
		}
		catch (AudioException ex)
		{
			Assert.Equal(expected, ex.Status);
		}
	}
}
#endif
