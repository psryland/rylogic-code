# Rylogic Audio

Rylogic Audio is a Windows 11 native spatial-audio library. The static C++ API is declared in
`include/pr/audio/audio.h`; `audio.dll` exposes the versioned C ABI in
`include/pr/audio/audio-dll.h` for native dynamic loading and a future managed wrapper.

## Runtime architecture

- XAudio2 2.9 owns one output graph per `Engine`.
- Each engine renders one listener/output mix. Independent rendered perspectives use separate engines.
- X3DAudio supplies speaker matrixing, distance attenuation, Doppler, source cones, and direct-path low-pass values.
- Effects, ambience, speech, music, and interface buses provide application-neutral mix categories.
- Spatial voices send a caller-controlled room contribution to a dedicated XAudio2 reverb effect bus.
- Ogg Vorbis streams are decoded from engine-owned compressed bytes on a bounded worker thread. XAudio2 callbacks only publish atomic completion/underrun state.

## Coordinates and acoustic inputs

Public positions use Rylogic's right-handed world space in metres. Velocities use metres per
second. Forward and up vectors must be finite and non-zero. The backend converts to X3DAudio's
left-handed convention exactly once.

Audio does not query rendering or collision geometry. Callers derive normalized obstruction,
occlusion, and reverb-send parameters from their own scene, physics, and environment models.
This keeps authoritative simulation acoustics independent from the rendered listener mix.

## Ownership and threading

`Engine`, `Clip`, `Voice`, and `Stream` handles are generation-aware. Child handles belong to
one engine and become stale after destruction. Creation copies WAV or Ogg input bytes into
engine-owned storage; callers do not retain asynchronous buffer-lifetime obligations.

Every mutable engine operation runs on the OS thread that created the engine. Wrong-thread calls
return `EStatus::WrongThread`. Runtime completion, underrun, virtualization, device-reset, and
queue-overflow notifications are drained through `Audio_EventsCopy`; no user callback runs on an
XAudio2 real-time thread.

## Capacity and latency

Default limits are 1,024 logical voices, 128 rendered XAudio2 voices, and 16 streams. Logical
voices beyond the rendered budget are virtualized. Higher-priority voices can reclaim physical
slots from lower-priority voices, and waiting voices are promoted in priority order.

Streaming targets at most 40 ms of engine-owned decoded buffering. Endpoint, Windows mixer,
driver, and audio-processing-object latency are outside that budget and must be measured on the
target machine.

## Device recovery

The default mastering voice uses Windows' virtual audio client. If XAudio2 reports a critical
device error, the engine recreates its graph without invalidating public handles. Resident voices
and in-memory seekable Ogg streams resume from recorded transport positions and emit a
`DeviceReset` event.

## Supported formats

- Resident clips: RIFF/WAVE PCM 8/16/24/32-bit and IEEE float32.
- Streaming: Ogg Vorbis decoded through the vendored Xiph implementation.
- Procedural utilities: note/frequency conversion and basic tone synthesis.

MP3, AAC, M4A, MIDI sequencing, SoundFont playback, Windows Spatial Sound, geometry tracing, and
convolution reverb are intentionally outside the current API. Media Foundation and Windows
Spatial Sound can be added later behind internal decoder/output-backend seams without changing
the public coordinate or ownership model.

## Testing

`projects/tests/unittests/src/audio_dll_tests.cpp` dynamically loads the source-built DLL and
checks ABI discovery, layouts, errors, thread affinity, handle generations, resident spatial
voices, voice priority/virtualization, malformed Ogg rejection, stream limits, playback,
completion events, seeking, and stale stream handles.

`projects/tests/view3d-12-test` is the manual spatial demonstrator. Its camera drives the listener;
one emitter is stationary and directional, and another orbits to demonstrate panning and Doppler.
Press `O` to toggle strong caller-supplied occlusion. Subjective localization, headphone/spatial
processing, loudness balance, and endpoint-switch quality require perceptual verification on the
actual output device.
