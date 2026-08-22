# Rylogic.Audio

Managed, allocation-conscious ownership and playback APIs for the Rylogic spatial audio engine.

`AudioRuntime` owns the native DLL context and the engines created through it. Each `Engine` owns all `Clip`, `Voice`, and
`AudioStream` objects created through it. Mutable and lifetime operations, and all native queries, must run on the OS thread
that created the engine. A `Clip` tracks the voices that reference it and rejects disposal while any of them remain live; a
`Voice` always references the `Clip` it plays.

`AudioData` holds an immutable copy of an encoded WAV or Ogg Vorbis payload. Loading from disk can run asynchronously, but
creating a `Clip` or `AudioStream` from that data always runs synchronously on the engine's owner thread, matching the
native engine's synchronous, copy-on-create contract.

The engine requires a 64-bit process running Windows 11 or greater.

The package depends on `Rylogic.Core` and `Rylogic.Native`. It does not depend on Rylogic.Gfx, View3D, D3D12, WPF, or
application-specific code.

## Basic use

```csharp
using var audio = new AudioRuntime();
using var engine = audio.CreateEngine();

var data = AudioData.Load("effects/impact.wav");
using var clip = engine.CreateClip(data);
using var voice = engine.CreateVoice(clip, new VoiceOptions
{
	Bus = EAudioBus.Effects,
	Spatial = true,
	Priority = 100,
});

engine.SetListener(new ListenerState(
	position: v3.Zero,
	forward: new v3(0, -1, 0),
	up: new v3(0, 0, 1),
	velocity: v3.Zero));
voice.SetEmitter(EmitterState.AtPosition(new v3(3, 0, 0)));
voice.Play();
```

Positions use a right-handed world space in metres, velocities use metres per second, and cone angles use radians. The
native backend performs its X3DAudio coordinate conversion internally.

## Owner thread and asynchronous loading

Every engine operation, including state queries and disposal, must run on the OS thread that created the engine. The wrapper
checks this before crossing the native boundary and does not capture or marshal through a `SynchronizationContext`.

`AudioData.LoadAsync` performs cancellable file I/O only, so it may run on any thread:

```csharp
var data = await AudioData.LoadAsync(path, cancellation_token).ConfigureAwait(false);

// Queue 'data' to the engine owner thread. Native finalization remains synchronous there.
using var stream = engine.CreateStream(data);
```

The engine copies WAV and Ogg inputs during `CreateClip`/`CreateStream`; managed source buffers may be reused immediately
after those methods return.

## Update and events

Call `Engine.Update` regularly on the owner thread. It applies spatial state, processes native completion flags, promotes
virtualized voices, refills streams, recovers device changes, and publishes buffered events. No managed callback runs from
an XAudio2 or decoder thread.

Use caller-owned storage in frame loops:

```csharp
engine.Update();
var count = engine.EventCount();
Span<AudioEvent> events = count <= 64 ? stackalloc AudioEvent[count] : new AudioEvent[count];
engine.CopyEvents(events);
```

`DrainEvents` is an allocating convenience for cold paths. Events distinguish voice and stream handles and report
realization, virtualization, completion, stream underruns, device resets, and queue overflow with stable sequence numbers.

## Deterministic disposal

Dispose voices before their clip, and dispose streams and engines on their owner thread. Disposing an engine invalidates all
remaining child wrappers in dependency order. SafeHandle abandonment is a finalizer-only fallback and does not replace
deterministic owner-thread disposal. That fallback applies to runtime and engine ownership; clip, voice, and stream wrappers
remain engine-owned and must be disposed explicitly when their individual capacity should be reclaimed before engine teardown.
