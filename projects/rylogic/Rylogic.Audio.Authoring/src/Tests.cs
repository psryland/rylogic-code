#if PR_UNITTESTS
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text.Json.Nodes;
using System.Threading;
using System.Threading.Tasks;
using Rylogic.UnitTests;

namespace Rylogic.Audio.Authoring;

/// <summary>Validates recipe rendering, originals, preparation files, the render cache and latest-result job delivery.</summary>
[TestFixture]
public sealed class TestAuthoring
{
	/// <summary>A stereo sine at 'frequency' Hz and amplitude 'amplitude' lasting 'seconds'.</summary>
	private static AudioBuffer Sine(double frequency, double amplitude, double seconds)
	{
		// Both channels carry the same tone, so mono and stereo processing can be compared
		var frames = (int)(seconds * AudioBuffer.SampleRate);
		var samples = new float[frames * 2];
		for (int i = 0; i != frames; ++i)
			samples[i * 2] = samples[i * 2 + 1] = (float)(amplitude * Math.Sin(2 * Math.PI * frequency * i / AudioBuffer.SampleRate));

		return new AudioBuffer(samples, 2);
	}

	/// <summary>A mono ramp whose sample 'i' is 'i', which makes frame positions visible in the output.</summary>
	private static AudioBuffer Ramp(int frames)
	{
		return new AudioBuffer(Enumerable.Range(0, frames).Select(i => (float)i).ToArray(), 1);
	}

	/// <summary>The RMS level of one channel.</summary>
	private static double Rms(AudioBuffer buffer, int skip_frames = 0)
	{
		// Skipping the start ignores a filter's settling time
		var sum = 0.0;
		var count = 0;
		for (int i = skip_frames * buffer.Channels; i < buffer.Samples.Length; i += buffer.Channels, ++count)
			sum += (double)buffer.Samples[i] * buffer.Samples[i];

		return Math.Sqrt(sum / count);
	}

	/// <summary>A fresh temporary folder that is deleted by the returned scope.</summary>
	private sealed class TempFolder : IDisposable
	{
		public string Path { get; } = System.IO.Path.Combine(System.IO.Path.GetTempPath(), $"rylogic-audio-authoring-{Guid.NewGuid():N}");

		public TempFolder()
		{
			Directory.CreateDirectory(Path);
		}

		public void Dispose()
		{
			Directory.Delete(Path, recursive: true);
		}
	}

	/// <summary>Rendering is a pure function of the original and the enabled steps (P-T1).</summary>
	[Test]
	public void RenderIsDeterministic()
	{
		// A recipe using every kind of step must give identical bytes every time and leave the original untouched
		var original = Sine(440, 0.5, 4);
		var before = WavFile.Encode(original);
		var recipe = new List<RecipeStep>
		{
			new() { Step = ERecipeStep.Trim, Start = 0.1, End = 3.9 },
			new() { Step = ERecipeStep.Cut, Start = 1, End = 1.5 },
			new() { Step = ERecipeStep.HighPass, Frequency = 80, Slope = 24 },
			new() { Step = ERecipeStep.LowPass, Frequency = 8000, Slope = 48 },
			new() { Step = ERecipeStep.Eq, Frequency = 2000, Db = -3, Q = 2 },
			new() { Step = ERecipeStep.Denoise, Start = 0, End = 0.5, Db = 12 },
			new() { Step = ERecipeStep.Loudness, Lufs = -20 },
			new() { Step = ERecipeStep.Gain, Db = -1 },
			new() { Step = ERecipeStep.FadeIn, Duration = 0.2, Curve = EFadeCurve.Linear },
			new() { Step = ERecipeStep.FadeOut, Duration = 0.2 },
			new() { Step = ERecipeStep.Loop, Duration = 0.5 },
		};
		var a = WavFile.Encode(RecipeRenderer.Render(original, recipe));
		var b = WavFile.Encode(RecipeRenderer.Render(original, recipe));
		Assert.True(a.AsSpan().SequenceEqual(b));
		Assert.True(before.AsSpan().SequenceEqual(WavFile.Encode(original)));

		// The WAV encoding round-trips exactly
		var decoded = WavFile.Decode(new MemoryStream(a));
		Assert.True(a.AsSpan().SequenceEqual(WavFile.Encode(decoded)));
	}

	/// <summary>Each step does what its documentation says.</summary>
	[Test]
	public void StepBehaviour()
	{
		// Trim keeps exactly the region, and a missing end keeps to the end
		var rate = AudioBuffer.SampleRate;
		var trimmed = RecipeRenderer.Render(Ramp(rate * 2), [new() { Step = ERecipeStep.Trim, Start = 0.5 }]);
		Assert.Equal(rate * 3 / 2, trimmed.Frames);
		Assert.Equal((float)(rate / 2), trimmed.Samples[0]);

		// Cut removes the region and a 10 ms crossfade overlaps the join
		var cut = RecipeRenderer.Render(Ramp(rate * 2), [new() { Step = ERecipeStep.Cut, Start = 0.5, End = 1.0 }]);
		Assert.Equal(rate * 3 / 2 - rate / 100, cut.Frames);
		Assert.Equal((float)(rate * 2 - 1), cut.Samples[^1]);

		// Gain scales by the decibel factor
		var gained = RecipeRenderer.Render(Sine(1000, 0.5, 1), [new() { Step = ERecipeStep.Gain, Db = -6.0206 }]);
		Assert.Equal(Rms(Sine(1000, 0.25, 1)), Rms(gained), 1e-4);

		// Fades start from silence and end in silence
		var faded = RecipeRenderer.Render(Sine(1000, 0.5, 1), [new() { Step = ERecipeStep.FadeIn, Duration = 0.1 }, new() { Step = ERecipeStep.FadeOut, Duration = 0.1 }]);
		Assert.Equal(0.0, faded.Samples[0], 1e-12);
		Assert.Equal(0.0, faded.Samples[^1], 1e-12);

		// A high-pass removes a tone well below its corner and keeps one well above it
		var low = RecipeRenderer.Render(Sine(50, 0.5, 1), [new() { Step = ERecipeStep.HighPass, Frequency = 1000, Slope = 24 }]);
		var high = RecipeRenderer.Render(Sine(8000, 0.5, 1), [new() { Step = ERecipeStep.HighPass, Frequency = 1000, Slope = 24 }]);
		Assert.True(Rms(low, rate / 10) < Rms(Sine(50, 0.5, 1)) * 0.01);
		Assert.Equal(Rms(Sine(8000, 0.5, 1)), Rms(high, rate / 10), 0.01);

		// Loudness matching reaches its target
		var matched = RecipeRenderer.Render(Sine(1000, 0.1, 3), [new() { Step = ERecipeStep.Loudness, Lufs = -18 }]);
		Assert.Equal(-18.0, LoudnessMeter.Integrated(matched), 0.01);

		// The loop step shortens the sound by its crossfade, and the end flows into the start as it did in the original
		var looped = RecipeRenderer.Render(Ramp(rate), [new() { Step = ERecipeStep.Loop, Duration = 0.25 }]);
		Assert.Equal(rate * 3 / 4, looped.Frames);
		Assert.Equal((float)(rate * 3 / 4), looped.Samples[0]);
		Assert.Equal((float)(rate * 3 / 4 - 1), looped.Samples[^1]);
	}

	/// <summary>Denoise turns down hiss by about its reduction where there is only hiss, leaves a sound clearly above the hiss unchanged, and keeps the length.</summary>
	[Test]
	public void DenoiseStep()
	{
		// Three seconds of quiet white hiss, the same on both channels, with a loud tone from 1 s to 2 s; the first 0.9 s is hiss only
		var rate = AudioBuffer.SampleRate;
		var random = new Random(1);
		var samples = new float[rate * 3 * 2];
		for (int i = 0; i != rate * 3; ++i)
		{
			// The tone is added over its one-second span
			var hiss = 0.01 * (random.NextDouble() * 2 - 1);
			var tone = i >= rate && i < rate * 2 ? 0.3 * Math.Sin(2 * Math.PI * 1000 * i / rate) : 0.0;
			samples[i * 2] = samples[i * 2 + 1] = (float)(hiss + tone);
		}
		var noisy = new AudioBuffer(samples, 2);
		var clean = RecipeRenderer.Render(noisy, [new() { Step = ERecipeStep.Denoise, Start = 0, End = 0.9, Db = 12 }]);

		// The RMS of the left channel between two times, in seconds
		static double RmsBetween(AudioBuffer buffer, double from, double to)
		{
			// Only the left channel is read; both channels match
			var sum = 0.0;
			var first = buffer.FrameAt(from);
			var last = buffer.FrameAt(to);
			for (int f = first; f != last; ++f)
				sum += (double)buffer.Samples[f * 2] * buffer.Samples[f * 2];

			return Math.Sqrt(sum / (last - first));
		}

		// The length and channels are kept, and shared gains keep identical channels identical
		Assert.Equal(noisy.Frames, clean.Frames);
		Assert.Equal(2, clean.Channels);
		Assert.Equal(clean.Samples[rate * 2], clean.Samples[rate * 2 + 1]);

		// Hiss alone, away from the tone's attack and release, drops by close to 12 dB (a factor of about 0.25)
		var hiss_ratio = RmsBetween(clean, 2.4, 2.9) / RmsBetween(noisy, 2.4, 2.9);
		Assert.True(hiss_ratio < 0.35);

		// The tone, far above the hiss, passes almost unchanged
		var tone_ratio = RmsBetween(clean, 1.2, 1.8) / RmsBetween(noisy, 1.2, 1.8);
		Assert.Equal(1.0, tone_ratio, 0.02);

		// A noise section shorter than one analysis window is rejected
		Assert.Throws<InvalidDataException>(() => RecipeRenderer.Render(noisy, [new() { Step = ERecipeStep.Denoise, Start = 0, End = 0.01, Db = 12 }]));
	}

	/// <summary>Bad parameters are reported with the failing step, and bypassed steps are ignored.</summary>
	[Test]
	public void Validation()
	{
		// The message names the step so the editor can point at it
		var ex = (InvalidDataException?)null;
		try
		{
			RecipeRenderer.Render(Sine(440, 0.5, 1), [new() { Step = ERecipeStep.Gain, Db = 0 }, new() { Step = ERecipeStep.LowPass, Frequency = 30000 }]);
		}
		catch (InvalidDataException e)
		{
			ex = e;
		}
		Assert.True(ex != null && ex.Message.StartsWith("Step 2 (LowPass)", StringComparison.Ordinal));
		Assert.Throws<InvalidDataException>(() => RecipeRenderer.Render(Sine(440, 0.5, 1), [new() { Step = ERecipeStep.Loop, Duration = 0.9 }]));
		Assert.Throws<InvalidDataException>(() => RecipeRenderer.Render(new AudioBuffer(new float[4800], 1), [new() { Step = ERecipeStep.Loudness, Lufs = -20 }]));

		// A bypassed bad step does not run
		var bypassed = RecipeRenderer.Render(Sine(440, 0.5, 1), [new() { Step = ERecipeStep.LowPass, Frequency = 30000, Bypass = true }]);
		Assert.Equal(AudioBuffer.SampleRate, bypassed.Frames);
	}

	/// <summary>A mix adds other originals at their own levels and start points, ends with the shortest part, and widens mono parts to stereo.</summary>
	[Test]
	public void MixStep()
	{
		// A mono ramp mixed with a quieter stereo tone that starts 0.25 s in: the tone is the shorter part once its start is skipped
		var rate = AudioBuffer.SampleRate;
		var tone = Sine(1000, 0.5, 1);
		var inputs = new Dictionary<string, AudioBuffer> { ["tone"] = tone };
		var step = new RecipeStep { Step = ERecipeStep.Mix, Db = 0, Inputs = [new MixInput { Origin = "local:tone.wav", Source = "tone", Start = 0.25, Db = -6.0206 }] };
		var mixed = RecipeRenderer.Render(Ramp(rate * 2), [step], inputs);
		Assert.Equal(2, mixed.Channels);
		Assert.Equal(rate * 3 / 4, mixed.Frames);

		// Each frame is the ramp on both channels plus half the tone from its start point
		var at = rate / 3;
		var expected = at + 0.5 * tone.Samples[(rate / 4 + at) * 2];
		Assert.Equal(expected, mixed.Samples[at * 2], 0.01);
		Assert.Equal(expected, mixed.Samples[at * 2 + 1], 0.01);

		// A mix needs an input, an imported original for each one, and a start point inside it
		Assert.Throws<InvalidDataException>(() => RecipeRenderer.Render(Ramp(rate), [new() { Step = ERecipeStep.Mix, Db = 0, Inputs = [] }]));
		Assert.Throws<InvalidDataException>(() => RecipeRenderer.Render(Ramp(rate), [new() { Step = ERecipeStep.Mix, Db = 0, Inputs = [new MixInput { Origin = "local:x.wav" }] }]));
		Assert.Throws<InvalidDataException>(() => RecipeRenderer.Render(Ramp(rate), [new() { Step = ERecipeStep.Mix, Db = 0, Inputs = [new MixInput { Origin = "local:tone.wav", Source = "tone", Start = 1 }] }], inputs));
	}

	/// <summary>Recipes stay bound to stable variant IDs through reordering, review and saving (P-T3).</summary>
	[Test]
	public void PrepFilesKeepVariantIdentity()
	{
		// Reorder and change status; each recipe stays with its variant and runtime order is list order
		using var temp = new TempFolder();
		var prep = new PrepFile { Id = "wind.test" };
		var a = prep.AddVariant("freesound:1", "aaaa", EVariantStatus.Accepted);
		var b = prep.AddVariant("freesound:2", "bbbb", EVariantStatus.Candidate);
		a.Recipe.Add(new RecipeStep { Step = ERecipeStep.Gain, Db = -3 });
		b.Recipe.Add(new RecipeStep { Step = ERecipeStep.Loop, Duration = 2 });
		prep.Variants.Reverse();
		b.Status = EVariantStatus.Accepted;
		prep.Save(temp.Path);

		var loaded = PrepFile.Load(temp.Path, "wind.test");
		Assert.Equal(new[] { "v2", "v1" }, loaded.Accepted.Select(x => x.Id));
		Assert.Equal(-3.0, loaded.Find("v1")!.Recipe[0].Db!.Value, 1e-12);
		Assert.Equal(ERecipeStep.Loop, loaded.Find("v2")!.Recipe[0].Step);

		// A removed variant's ID is never reused
		loaded.Variants.RemoveAll(x => x.Id == "v2");
		Assert.Equal("v3", loaded.AddVariant("freesound:3", null, EVariantStatus.Unheard).Id);

		// The file uses snake_case names and enum values
		var text = File.ReadAllText(PrepFile.PathFor(temp.Path, "wind.test"));
		Assert.True(text.Contains("\"next_variant\"") && text.Contains("\"accepted\"") && text.Contains("\"loop\""));
	}

	/// <summary>Originals are stored once and never modified; the cache key follows content and enabled steps only (P-T2, P-T4).</summary>
	[Test]
	public void OriginalsAndCache()
	{
		// Import a local recording; importing the same bytes again returns the same source
		using var temp = new TempFolder();
		var ffmpeg = new Ffmpeg();
		var download = Path.Combine(temp.Path, "tone.wav");
		File.WriteAllBytes(download, WavFile.Encode(Sine(440, 0.5, 2)));
		var sources = new SourceStore(Path.Combine(temp.Path, "originals"), null, ffmpeg);
		var provenance = new SourceSidecar { Kind = EOriginKind.Local, Origin = "local:gen/tone.wav" };
		var sidecar = sources.ImportAsync(download, provenance, CancellationToken.None).GetAwaiter().GetResult();
		Assert.Equal(sidecar.Id, sources.ImportAsync(download, provenance, CancellationToken.None).GetAwaiter().GetResult().Id);
		Assert.Equal(SourceStore.SourceIdOf(sidecar.DownloadSha256), sidecar.Id);

		// Rendering from the original leaves it unchanged, and a second request is a cache hit
		var cache = new RenderCache(Path.Combine(temp.Path, "cache"), sources, ffmpeg);
		var recipe = new List<RecipeStep> { new() { Step = ERecipeStep.Gain, Db = -6 } };
		var (key, path) = cache.GetOrRenderAsync(sidecar.Id, recipe, CancellationToken.None).GetAwaiter().GetResult();
		var written = File.GetLastWriteTimeUtc(path);
		Assert.Equal(path, cache.GetOrRenderAsync(sidecar.Id, recipe, CancellationToken.None).GetAwaiter().GetResult().path);
		Assert.Equal(written, File.GetLastWriteTimeUtc(path));
		sources.VerifyAsync(sidecar.Id, CancellationToken.None).GetAwaiter().GetResult();

		// The render is exactly the recipe applied to the decoded original
		var rendered = WavFile.Read(path);
		var expected = RecipeRenderer.Render(ffmpeg.DecodeAsync(sources.OriginalPath(sidecar.Id), CancellationToken.None).GetAwaiter().GetResult(), recipe);
		Assert.True(WavFile.Encode(expected).AsSpan().SequenceEqual(WavFile.Encode(rendered)));

		// Bypassed steps do not change the key; parameter changes do
		var with_bypass = recipe.Append(new RecipeStep { Step = ERecipeStep.Eq, Frequency = 500, Db = 6, Q = 1, Bypass = true }).ToList();
		Assert.Equal(key, cache.KeyAsync(sidecar.Id, with_bypass, CancellationToken.None).GetAwaiter().GetResult());
		Assert.NotEqual(key, cache.KeyAsync(sidecar.Id, [new() { Step = ERecipeStep.Gain, Db = -5 }], CancellationToken.None).GetAwaiter().GetResult());

		// A modified original is detected rather than silently rendered
		File.AppendAllText(sources.OriginalPath(sidecar.Id), "x");
		Assert.Throws<InvalidDataException>(() => sources.VerifyAsync(sidecar.Id, CancellationToken.None).GetAwaiter().GetResult());
	}

	/// <summary>A compressed download is stored as received rather than enlarged by conversion, and still renders.</summary>
	[Test]
	public void CompressedOriginalsAreKept()
	{
		// Encode a short tone as MP3, as a download would arrive
		using var temp = new TempFolder();
		var ffmpeg = new Ffmpeg();
		var wav = Path.Combine(temp.Path, "tone.wav");
		var mp3 = Path.Combine(temp.Path, "tone.mp3");
		File.WriteAllBytes(wav, WavFile.Encode(Sine(440, 0.5, 1)));
		using (var process = System.Diagnostics.Process.Start(ffmpeg.ExePath, ["-hide_banner", "-loglevel", "error", "-i", wav, "-c:a", "libmp3lame", "-b:a", "64k", mp3]))
		{
			process.WaitForExit();
			Assert.Equal(0, process.ExitCode);
		}

		// The stored original is the MP3 byte-for-byte, and renders from it are verified like any other original
		var sources = new SourceStore(Path.Combine(temp.Path, "originals"), null, ffmpeg);
		var sidecar = sources.ImportAsync(mp3, new SourceSidecar { Kind = EOriginKind.Local, Origin = "local:gen/tone.mp3" }, CancellationToken.None).GetAwaiter().GetResult();
		Assert.Equal(Path.Combine(temp.Path, "originals", $"{sidecar.Id}.mp3"), sources.OriginalPath(sidecar.Id));
		Assert.True(File.ReadAllBytes(mp3).AsSpan().SequenceEqual(File.ReadAllBytes(sources.OriginalPath(sidecar.Id))));
		Assert.Equal(sidecar.DownloadSha256, sidecar.OriginalSha256);
		var cache = new RenderCache(Path.Combine(temp.Path, "cache"), sources, ffmpeg);
		var path = cache.GetOrRenderAsync(sidecar.Id, [], CancellationToken.None).GetAwaiter().GetResult().path;
		Assert.True(WavFile.Read(path).Frames > 0);
	}

	/// <summary>Accepted variants resolve to their prepared files in order; unassigned sounds are absent and broken references fail (P-T11).</summary>
	[Test]
	public void PreparedSoundsResolve()
	{
		// A project with one imported original, one reference master and a catalog of three sounds
		using var temp = new TempFolder();
		File.WriteAllText(Path.Combine(temp.Path, "authoring.json"), """{"schema":1,"catalog":"sounds.json","originals":"originals","prep":"prep","origins":"origins.json","cache":"cache"}""");
		var config = AuthoringConfig.Load(Path.Combine(temp.Path, "authoring.json"));
		var ffmpeg = new Ffmpeg();
		var sources = new SourceStore(config.OriginalsFolder, null, ffmpeg);
		var cache = new RenderCache(config.CacheFolder, sources, ffmpeg);
		using var http = new System.Net.Http.HttpClient();
		var fetcher = new OriginalFetcher(config, sources, new FreesoundClient(http, null));
		var download = Path.Combine(temp.Path, "tone.wav");
		File.WriteAllBytes(download, WavFile.Encode(Sine(440, 0.5, 1)));
		var source = sources.ImportAsync(download, new SourceSidecar { Kind = EOriginKind.Local, Origin = "local:gen/tone.wav" }, CancellationToken.None).GetAwaiter().GetResult().Id;
		File.WriteAllBytes(Path.Combine(temp.Path, "master.flac"), [1]);
		string[] sound_ids = ["a", "b", "silent"];

		// Sound 'a' has a rejected variant between a recipe variant and a reference variant; 'b' has only a candidate; 'silent' has no file
		var a = new PrepFile { Id = "a" };
		var rendered = a.AddVariant("local:gen/tone.wav", source, EVariantStatus.Accepted);
		rendered.Recipe.Add(new RecipeStep { Step = ERecipeStep.Gain, Db = -3 });
		a.AddVariant("local:gen/other.wav", null, EVariantStatus.Rejected);
		a.AddVariant("hand_made:a.1", null, EVariantStatus.Accepted).ReferenceMaster = "master.flac";
		a.Save(config.PrepFolder);
		var b = new PrepFile { Id = "b" };
		b.AddVariant("local:gen/tone.wav", source, EVariantStatus.Candidate);
		b.Save(config.PrepFolder);

		// Only 'a' resolves, to its render and then its reference master
		var resolved = PreparedSounds.ResolveAsync(config, cache, fetcher, sound_ids, CancellationToken.None).GetAwaiter().GetResult();
		Assert.Equal(1, resolved.Count);
		Assert.Equal(cache.GetOrRenderAsync(source, rendered.Recipe, CancellationToken.None).GetAwaiter().GetResult().path, resolved["a"][0]);
		Assert.Equal(Path.Combine(temp.Path, "master.flac"), resolved["a"][1]);

		// A missing reference master, an accepted variant with no original, and a file for an unknown sound are all errors
		File.Delete(Path.Combine(temp.Path, "master.flac"));
		Assert.Throws<FileNotFoundException>(() => PreparedSounds.ResolveAsync(config, cache, fetcher, sound_ids, CancellationToken.None).GetAwaiter().GetResult());
		a.Variants[2].Status = EVariantStatus.Rejected;
		a.AddVariant("local:gen/missing.wav", null, EVariantStatus.Accepted);
		a.Save(config.PrepFolder);
		Assert.Throws<InvalidDataException>(() => PreparedSounds.ResolveAsync(config, cache, fetcher, sound_ids, CancellationToken.None).GetAwaiter().GetResult());
		Assert.Throws<InvalidDataException>(() => PreparedSounds.CheckDefined(config, ["b", "silent"]));
		PreparedSounds.CheckDefined(config, sound_ids);

		// Resolving a subset ignores the other sounds' files, so one sound can be prepared while another is broken
		Assert.Equal(0, PreparedSounds.ResolveAsync(config, cache, fetcher, ["b"], CancellationToken.None).GetAwaiter().GetResult().Count);
	}

	/// <summary>A missing original is fetched again and verified against its recorded source; stores find originals in either folder.</summary>
	[Test]
	public void OriginalsAreFetchedAgain()
	{
		// A project whose local recording has been imported and adopted by sound 'a'
		using var temp = new TempFolder();
		File.WriteAllText(Path.Combine(temp.Path, "authoring.json"), """{"schema":1,"catalog":"sounds.json","originals":"originals","fetched":"fetched","local":[{"name":"gen","path":"generated","author":"Generator","licence":"CC0"}],"prep":"prep","origins":"origins.json","cache":"cache"}""");
		var config = AuthoringConfig.Load(Path.Combine(temp.Path, "authoring.json"));
		var ffmpeg = new Ffmpeg();
		var sources = new SourceStore(config.OriginalsFolder, config.FetchedFolder, ffmpeg);
		var cache = new RenderCache(config.CacheFolder, sources, ffmpeg);
		using var http = new System.Net.Http.HttpClient();
		var fetcher = new OriginalFetcher(config, sources, new FreesoundClient(http, null));
		Directory.CreateDirectory(config.LocalFolderOf("gen").Path);
		var generated = Path.Combine(config.LocalFolderOf("gen").Path, "tone.wav");
		File.WriteAllBytes(generated, WavFile.Encode(Sine(440, 0.5, 1)));
		const string origin = "local:gen/tone.wav";
		var origins = new OriginsFile();
		origins.Origins[origin] = new OriginRecord { Title = "Tone" };
		var source = fetcher.ImportAsync(origin, origins.Origins[origin], CancellationToken.None).GetAwaiter().GetResult().Id;
		origins.Origins[origin].Source = source;
		AuthoringJson.Write(config.OriginsPath, origins);
		var a = new PrepFile { Id = "a" };
		a.AddVariant(origin, source, EVariantStatus.Accepted);
		a.Save(config.PrepFolder);

		// Originals that exist nowhere else stay in the originals folder; a store also finds an original in the fetched folder
		Assert.Equal(Path.Combine(config.OriginalsFolder, $"{source}.flac"), sources.OriginalPath(source));
		Directory.CreateDirectory(config.FetchedFolder!);
		foreach (var ext in new[] { "flac", "json" })
			File.Move(Path.Combine(config.OriginalsFolder, $"{source}.{ext}"), Path.Combine(config.FetchedFolder!, $"{source}.{ext}"));

		Assert.Equal(Path.Combine(config.FetchedFolder!, $"{source}.flac"), sources.OriginalPath(source));
		sources.VerifyAsync(source, CancellationToken.None).GetAwaiter().GetResult();

		// Resolving with the original missing fetches it again, and renders the same audio
		var expected = cache.GetOrRenderAsync(source, [], CancellationToken.None).GetAwaiter().GetResult().path;
		var expected_bytes = File.ReadAllBytes(expected);
		Directory.Delete(config.FetchedFolder!, true);
		Directory.Delete(config.CacheFolder, true);
		Assert.False(sources.Contains(source));
		var resolved = PreparedSounds.ResolveAsync(config, cache, fetcher, ["a"], CancellationToken.None).GetAwaiter().GetResult();
		Assert.True(sources.Contains(source));
		Assert.True(File.ReadAllBytes(resolved["a"][0]).AsSpan().SequenceEqual(expected_bytes));

		// A recording that has changed at its origin is not silently used in place of the recorded source
		File.Delete(sources.OriginalPath(source));
		File.Delete(sources.SidecarPath(source));
		File.WriteAllBytes(generated, WavFile.Encode(Sine(220, 0.5, 1)));
		Assert.Throws<InvalidDataException>(() => fetcher.EnsureAsync(origin, origins.Origins[origin], source, CancellationToken.None).GetAwaiter().GetResult());
	}
	/// <summary>A superseded job never delivers, even if it finishes after the newer job (P-T7).</summary>
	[Test]
	public void LatestJobsRejectStaleResults()
	{
		// The first job is held until the second has delivered, then released
		var jobs = new LatestJobs(4);
		var release_first = new TaskCompletionSource();
		var delivered = new List<string>();
		var first = jobs.Run("slot", "first", async cancel =>
		{
			// Ignore cancellation to model work that completes regardless
			await release_first.Task;
			return "first";
		}, delivered.Add, ex => delivered.Add("error"));
		Assert.Equal(1, jobs.Pending.Count);

		var second = jobs.Run("slot", "second", cancel => Task.FromResult("second"), delivered.Add, ex => delivered.Add("error"));
		Assert.Equal(EJobOutcome.Completed, second.GetAwaiter().GetResult());
		release_first.SetResult();
		Assert.Equal(EJobOutcome.Superseded, first.GetAwaiter().GetResult());
		Assert.Equal(new[] { "second" }, delivered);
		Assert.Equal(0, jobs.Pending.Count);

		// Failures are delivered to the newest job's error handler
		var failed = jobs.Run<string>("slot", "fails", cancel => throw new InvalidDataException("bad"), delivered.Add, ex => delivered.Add(ex.Message));
		Assert.Equal(EJobOutcome.Failed, failed.GetAwaiter().GetResult());
		Assert.Equal("bad", delivered[^1]);
	}

	/// <summary>Wait up to 'seconds' for 'condition', returning whether it became true.</summary>
	private static bool WaitFor(Func<bool> condition, double seconds)
	{
		// Background jobs publish on the thread pool, so poll
		var end = DateTime.UtcNow.AddSeconds(seconds);
		while (!condition())
		{
			if (DateTime.UtcNow > end)
				return false;

			Thread.Sleep(20);
		}
		return true;
	}

	/// <summary>Answers Freesound API requests only after the test opens a gate, to model a slow download.</summary>
	private sealed class GatedFreesound(TaskCompletionSource gate, byte[] original) : System.Net.Http.HttpMessageHandler
	{
		protected override async Task<System.Net.Http.HttpResponseMessage> SendAsync(System.Net.Http.HttpRequestMessage request, CancellationToken cancel)
		{
			// The file type comes first, then the original's bytes
			await gate.Task.WaitAsync(cancel);
			var url = request.RequestUri!.ToString();
			var content = url.Contains("fields=type", StringComparison.Ordinal)
				? new System.Net.Http.StringContent("{\"type\":\"wav\"}")
				: (System.Net.Http.HttpContent)new System.Net.Http.ByteArrayContent(original);
			return new System.Net.Http.HttpResponseMessage(System.Net.HttpStatusCode.OK) { Content = content };
		}
	}

	/// <summary>
	/// State reads and review actions stay responsive while an original downloads (P-T6). The import then binds the source and renders it, and a failed
	/// render keeps the last working render (P-T9).
	/// </summary>
	[Test]
	public void WorkspaceStaysResponsive()
	{
		// A project with one looping sound, one known Freesound origin and a connected login
		using var temp = new TempFolder();
		File.WriteAllText(Path.Combine(temp.Path, "sounds.json"), $"{{\"schema\":1,\"sections\":[{{\"name\":\"Wind\",\"sounds\":[{BedSound("wind.test")}]}}]}}");
		File.WriteAllText(Path.Combine(temp.Path, "authoring.json"), "{\"schema\":1,\"catalog\":\"sounds.json\",\"originals\":\"originals\",\"prep\":\"prep\",\"origins\":\"origins.json\",\"cache\":\"cache\",\"fetched\":\"fetched\",\"freesound\":{\"credentials\":\"freesound.json\"}}");
		File.WriteAllText(Path.Combine(temp.Path, "origins.json"), "{\"schema\":1,\"origins\":{\"freesound:1\":{\"title\":\"Test wind\",\"user\":\"someone\",\"licence\":\"CC0\"}}}");
		File.WriteAllText(Path.Combine(temp.Path, "freesound.json"), $"{{\"client_id\":\"id\",\"client_secret\":\"secret\",\"access_token\":\"token\",\"refresh_token\":\"refresh\",\"expires_at\":{DateTimeOffset.UtcNow.ToUnixTimeSeconds() + 3600}}}");
		var gate = new TaskCompletionSource(TaskCreationOptions.RunContinuationsAsynchronously);
		using var http = new System.Net.Http.HttpClient(new GatedFreesound(gate, WavFile.Encode(Sine(440, 0.5, 2))));
		var workspace = new Workspace(AuthoringConfig.Load(Path.Combine(temp.Path, "authoring.json")), new Ffmpeg(), http);

		// Accepting the new variant starts the download, which is held at the gate
		Assert.Equal("Assigned to wind.test.", workspace.Apply(new System.Text.Json.Nodes.JsonObject { ["action"] = "assign", ["id"] = "wind.test", ["origin"] = "freesound:1" }));
		workspace.Apply(new System.Text.Json.Nodes.JsonObject { ["action"] = "status", ["id"] = "wind.test", ["variant"] = "v1", ["status"] = "accepted" });
		Assert.True(WaitFor(() => workspace.Jobs.Pending.ContainsKey("import:freesound:1"), 5));

		// State and edits do not wait for the download
		var timer = System.Diagnostics.Stopwatch.StartNew();
		var state = workspace.Snapshot();
		workspace.Apply(new System.Text.Json.Nodes.JsonObject { ["action"] = "note", ["id"] = "wind.test", ["variant"] = "v1", ["note"] = "gusty" });
		Assert.True(timer.Elapsed < TimeSpan.FromSeconds(1));
		Assert.True(state["origins"]!["freesound:1"]!["import_pending"]!.GetValue<bool>());
		Assert.Equal("gusty", PrepFile.Load(workspace.Config.PrepFolder, "wind.test").Variants[0].Note);

		// Once released, the original is imported, bound to the variant and rendered
		gate.SetResult();
		string? RenderUrl() => (string?)workspace.Snapshot()["preps"]!["wind.test"]![0]!["render_url"];
		Assert.True(WaitFor(() => RenderUrl() != null, 30));
		var source = PrepFile.Load(workspace.Config.PrepFolder, "wind.test").Variants[0].Source;
		Assert.True(source != null && workspace.Sources.Contains(source));
		Assert.Equal(Path.Combine(temp.Path, "fetched", $"{source}.flac"), workspace.Sources.OriginalPath(source!));
		Assert.Equal(2, Directory.EnumerateFileSystemEntries(Path.Combine(temp.Path, "fetched")).Count());

		// A failing recipe reports its error and keeps the last working render
		var working = RenderUrl();
		workspace.Apply(new System.Text.Json.Nodes.JsonObject { ["action"] = "recipe", ["id"] = "wind.test", ["variant"] = "v1", ["recipe"] = System.Text.Json.Nodes.JsonNode.Parse("[{\"step\":\"low_pass\",\"frequency\":30000}]") });
		Assert.True(WaitFor(() => workspace.Snapshot()["preps"]!["wind.test"]![0]!["render_error"] != null, 30));
		Assert.Equal(working, RenderUrl());
		Assert.Equal(ERecipeStep.LowPass, PrepFile.Load(workspace.Config.PrepFolder, "wind.test").Variants[0].Recipe[0].Step);
	}

	/// <summary>Multiple original-time ranges become independent candidates and resolve as separate cooked sounds after acceptance.</summary>
	[Test]
	public void ClipRangesBecomeIndependentSounds()
	{
		// The source variant has processing and a reference master that must not be inherited by its clips
		using var temp = new TempFolder();
		File.WriteAllText(Path.Combine(temp.Path, "sounds.json"), $"{{\"schema\":1,\"sections\":[{{\"name\":\"Wind\",\"sounds\":[{BedSound("a")},{BedSound("b")}]}}]}}");
		File.WriteAllText(Path.Combine(temp.Path, "authoring.json"), """{"schema":1,"catalog":"sounds.json","originals":"originals","prep":"prep","origins":"origins.json","cache":"cache"}""");
		var config = AuthoringConfig.Load(Path.Combine(temp.Path, "authoring.json"));
		var ffmpeg = new Ffmpeg();
		var store = new SourceStore(config.OriginalsFolder, null, ffmpeg);
		var download = Path.Combine(temp.Path, "tone.wav");
		File.WriteAllBytes(download, WavFile.Encode(Sine(233.7, 0.1, 3)));
		var sidecar = store.ImportAsync(download, new SourceSidecar { Kind = EOriginKind.Imported, Origin = "imported:test", Author = "Author", Licence = "CC0" }, CancellationToken.None).GetAwaiter().GetResult();
		File.WriteAllText(config.OriginsPath, $"{{\"schema\":1,\"origins\":{{\"imported:test\":{{\"source\":\"{sidecar.Id}\",\"user\":\"Author\",\"licence\":\"CC0\"}}}}}}");
		var prep = new PrepFile { Id = "a" };
		var original = prep.AddVariant("imported:test", sidecar.Id, EVariantStatus.Candidate);
		original.Recipe.Add(new RecipeStep { Step = ERecipeStep.Gain, Db = -12 });
		original.ReferenceMaster = "master.flac";
		original.Finalised = true;
		prep.Save(config.PrepFolder);
		var original_bytes = File.ReadAllBytes(store.OriginalPath(sidecar.Id));
		using var http = new System.Net.Http.HttpClient();
		using var workspace = new Workspace(config, ffmpeg, http);

		// Two ranges target the same ID, and an overlapping range targets another ID; ordinary assignment must not deduplicate them
		workspace.Apply(JsonNode.Parse("""{"action":"clips","id":"a","variant":"v1","ranges":[{"id":"a","start":0.25,"end":0.75},{"id":"a","start":1,"end":1.75},{"id":"b","start":0.5,"end":1.5}]}""")!.AsObject());
		var a = PrepFile.Load(config.PrepFolder, "a");
		var b = PrepFile.Load(config.PrepFolder, "b");
		Assert.Equal(3, a.Variants.Count);
		Assert.Equal("v2", a.Variants[1].Id);
		Assert.Equal("v3", a.Variants[2].Id);
		Assert.Equal("v1", b.Variants[0].Id);
		Assert.Equal("master.flac", a.Variants[0].ReferenceMaster);
		Assert.Equal(-12.0, a.Variants[0].Recipe[0].Db);
		foreach (var clip in a.Variants.Skip(1).Concat(b.Variants))
		{
			// Every clip retains the original credit and starts a fresh, editable trim recipe
			Assert.Equal(sidecar.Id, clip.Source);
			Assert.Equal(original.Origin, clip.Origin);
			Assert.Equal(EVariantStatus.Candidate, clip.Status);
			Assert.False(clip.Finalised);
			Assert.Null(clip.ReferenceMaster);
			Assert.Equal(1, clip.Recipe.Count);
			Assert.Equal(ERecipeStep.Trim, clip.Recipe[0].Step);
		}
		JsonNode Clip(string id, int index) => workspace.Snapshot()["preps"]![id]![index]!;
		Assert.True(WaitFor(() => Clip("a", 1)["render_url"] != null && Clip("a", 2)["render_url"] != null && Clip("b", 0)["render_url"] != null, 30));
		var sibling_url = (string?)Clip("a", 2)["render_url"];
		Assert.Equal(0, PreparedSounds.ResolveAsync(config, workspace.Cache, workspace.Fetcher, ["a", "b"], CancellationToken.None).GetAwaiter().GetResult().Count);

		// Processing and accepting one clip does not alter its siblings; accepted clips cook to the exact original slices
		workspace.Apply(JsonNode.Parse("""{"action":"recipe","id":"a","variant":"v2","recipe":[{"step":"trim","start":0.25,"end":0.75},{"step":"gain","db":-6}]}""")!.AsObject());
		workspace.Apply(new JsonObject { ["action"] = "status", ["id"] = "a", ["variant"] = "v2", ["status"] = "accepted" });
		workspace.Apply(new JsonObject { ["action"] = "status", ["id"] = "a", ["variant"] = "v3", ["status"] = "accepted" });
		workspace.Apply(new JsonObject { ["action"] = "status", ["id"] = "b", ["variant"] = "v1", ["status"] = "accepted" });
		Assert.True(WaitFor(() => workspace.Jobs.Pending.Count == 0 && (string?)Clip("a", 0)["status"] == "accepted", 30));
		var resolved = PreparedSounds.ResolveAsync(config, workspace.Cache, workspace.Fetcher, ["a", "b"], CancellationToken.None).GetAwaiter().GetResult();
		Assert.Equal(2, resolved["a"].Count);
		Assert.Equal(1, resolved["b"].Count);
		var decoded = ffmpeg.DecodeAsync(store.OriginalPath(sidecar.Id), CancellationToken.None).GetAwaiter().GetResult();
		var accepted = PrepFile.Load(config.PrepFolder, "a").Accepted.Concat(PrepFile.Load(config.PrepFolder, "b").Accepted).ToArray();
		var paths = resolved["a"].Concat(resolved["b"]).ToArray();
		for (var i = 0; i != paths.Length; ++i)
		{
			// Compare actual samples as well as length, so clipping the processed source instead would fail
			var expected = RecipeRenderer.Render(decoded, accepted[i].Recipe);
			var actual = WavFile.Read(paths[i]);
			Assert.True(WavFile.Encode(expected).AsSpan().SequenceEqual(WavFile.Encode(actual)));
			Assert.Equal(new[] { 0.5, 0.75, 1.0 }[i], actual.Seconds);
		}
		Assert.Equal(sibling_url, (string?)Clip("a", 1)["render_url"]);
		Assert.True(original_bytes.AsSpan().SequenceEqual(File.ReadAllBytes(store.OriginalPath(sidecar.Id))));
	}

	/// <summary>Invalid clip batches leave all preparation files and next variant IDs unchanged.</summary>
	[Test]
	public void ClipRangesValidateBeforeAdding()
	{
		// A referenced variant avoids background rendering; invalid requests must fail before any render is queued
		using var temp = new TempFolder();
		File.WriteAllText(Path.Combine(temp.Path, "sounds.json"), $"{{\"schema\":1,\"sections\":[{{\"name\":\"Wind\",\"sounds\":[{BedSound("a")},{BedSound("b")}]}}]}}");
		File.WriteAllText(Path.Combine(temp.Path, "authoring.json"), """{"schema":1,"catalog":"sounds.json","originals":"originals","prep":"prep","origins":"origins.json","cache":"cache"}""");
		var config = AuthoringConfig.Load(Path.Combine(temp.Path, "authoring.json"));
		var prep = new PrepFile { Id = "a" };
		prep.AddVariant("imported:test", "source", EVariantStatus.Candidate).ReferenceMaster = "master.flac";
		prep.Save(config.PrepFolder);
		using var http = new System.Net.Http.HttpClient();
		using var workspace = new Workspace(config, new Ffmpeg(), http);
		var before = File.ReadAllText(PrepFile.PathFor(config.PrepFolder, "a"));

		// A valid first range must not be added when a later range or destination is invalid
		foreach (var bad in new[] { """{"id":"b","start":1,"end":1}""", """{"id":"b","start":2,"end":1}""", """{"id":"b","start":-1,"end":1}""", """{"id":"missing","start":0,"end":1}""", """{"id":"b","end":1}""", """{"id":"b","start":"bad","end":1}""" })
		{
			var action = JsonNode.Parse($$"""{"action":"clips","id":"a","variant":"v1","ranges":[{"id":"a","start":0,"end":1},{{bad}}]}""")!.AsObject();
			Assert.Throws<InvalidOperationException>(() => workspace.Apply(action));
			Assert.Equal(before, File.ReadAllText(PrepFile.PathFor(config.PrepFolder, "a")));
			Assert.False(File.Exists(PrepFile.PathFor(config.PrepFolder, "b")));
		}

		// Empty batches and missing originals are actionable errors too
		Assert.Throws<InvalidOperationException>(() => workspace.Apply(JsonNode.Parse("""{"action":"clips","id":"a","variant":"v1","ranges":[]}""")!.AsObject()));
		var missing = new PrepFile { Id = "b" };
		missing.AddVariant("imported:test", null, EVariantStatus.Candidate);
		missing.Save(config.PrepFolder);
		using var reopened = new Workspace(config, new Ffmpeg(), http);
		Assert.Throws<InvalidOperationException>(() => reopened.Apply(JsonNode.Parse("""{"action":"clips","id":"b","variant":"v1","ranges":[{"id":"a","start":0,"end":1}]}""")!.AsObject()));
		Assert.Equal(before, File.ReadAllText(PrepFile.PathFor(config.PrepFolder, "a")));
	}

	/// <summary>Local files are found recursively, credited to their folder, and assigned to the folder's sound ID or else the one their path suggests.</summary>
	[Test]
	public void LocalFoldersCreditAndAssign()
	{
		// Two local folders: one names its sound ID with a subfolder, the other relies on words in its paths
		using var temp = new TempFolder();
		File.WriteAllText(Path.Combine(temp.Path, "sounds.json"), $"{{\"schema\":1,\"sections\":[{{\"name\":\"Lab\",\"sounds\":[{BedSound("lab.radio")},{BedSound("mind.speech")}]}}]}}");
		File.WriteAllText(Path.Combine(temp.Path, "authoring.json"), "{\"schema\":1,\"catalog\":\"sounds.json\",\"originals\":\"originals\",\"prep\":\"prep\",\"origins\":\"origins.json\",\"cache\":\"cache\",\"local\":[{\"name\":\"jazz\",\"path\":\"jazz\",\"author\":\"Archive\",\"licence\":\"Public domain\",\"url\":\"https://example.org\"},{\"name\":\"gen\",\"path\":\"gen\",\"author\":\"Generator\",\"licence\":\"CC0\"}]}");
		Directory.CreateDirectory(Path.Combine(temp.Path, "jazz", "lab-radio", "disc 1"));
		Directory.CreateDirectory(Path.Combine(temp.Path, "gen", "speech"));
		File.WriteAllBytes(Path.Combine(temp.Path, "jazz", "lab-radio", "disc 1", "speech.mp3"), WavFile.Encode(Sine(440, 0.1, 1)));
		File.WriteAllBytes(Path.Combine(temp.Path, "gen", "speech", "speech-1.wav"), WavFile.Encode(Sine(440, 0.1, 1)));
		File.WriteAllText(Path.Combine(temp.Path, "gen", "notes.txt"), "not audio");
		var config = AuthoringConfig.Load(Path.Combine(temp.Path, "authoring.json"));
		using var http = new System.Net.Http.HttpClient();
		var workspace = new Workspace(config, new Ffmpeg(), http);
		workspace.Refresh();

		// Each audio file is an origin credited to its folder, and only audio files are found
		var origins = OriginsFile.Load(config.OriginsPath).Origins;
		Assert.Equal(2, origins.Count);
		Assert.Equal("Archive", origins["local:jazz/lab-radio/disc 1/speech.mp3"].User);
		Assert.Equal("Public domain", origins["local:jazz/lab-radio/disc 1/speech.mp3"].Licence);
		Assert.Equal("https://example.org", origins["local:jazz/lab-radio/disc 1/speech.mp3"].Url);
		Assert.Equal("CC0", origins["local:gen/speech/speech-1.wav"].Licence);

		// A folder named for a sound ID wins over words in the file name; otherwise the words are used
		Assert.Equal("local:jazz/lab-radio/disc 1/speech.mp3", PrepFile.Load(config.PrepFolder, "lab.radio").Variants.Single().Origin);
		Assert.Equal("local:gen/speech/speech-1.wav", PrepFile.Load(config.PrepFolder, "mind.speech").Variants.Single().Origin);

		// Origins resolve to their files and cannot leave their folder
		Assert.Equal(Path.Combine(temp.Path, "jazz", "lab-radio", "disc 1", "speech.mp3"), config.LocalPathOf("local:jazz/lab-radio/disc 1/speech.mp3"));
		Assert.Throws<InvalidDataException>(() => config.LocalPathOf("local:jazz/../gen/speech/speech-1.wav"));
		Assert.Throws<InvalidOperationException>(() => config.LocalPathOf("local:missing/track.mp3"));
	}

	/// <summary>A file added from disk is credited from its tags, copied into the originals and assigned; rejecting it removes it again.</summary>
	[Test]
	public void ImportedFilesAreCreditedAndForgotten()
	{
		// A project with one sound, and a tagged mp3 and an untagged wave file outside the project
		using var temp = new TempFolder();
		File.WriteAllText(Path.Combine(temp.Path, "sounds.json"), $"{{\"schema\":1,\"sections\":[{{\"name\":\"Wind\",\"sounds\":[{BedSound("wind.test")}]}}]}}");
		File.WriteAllText(Path.Combine(temp.Path, "authoring.json"), "{\"schema\":1,\"catalog\":\"sounds.json\",\"originals\":\"originals\",\"prep\":\"prep\",\"origins\":\"origins.json\",\"cache\":\"cache\"}");
		var ffmpeg = new Ffmpeg();
		var mp3 = Path.Combine(temp.Path, "gale.mp3");
		using (var process = System.Diagnostics.Process.Start(new System.Diagnostics.ProcessStartInfo(ffmpeg.ExePath, ["-v", "error", "-f", "lavfi", "-i", "sine=d=1", "-metadata", "title=Big Gale", "-metadata", "artist=Someone", "-metadata", "comment=From https://example.org/gale ok", mp3]) { UseShellExecute = false })!)
			process.WaitForExit();

		var wav = Path.Combine(temp.Path, "Breeze Take 2.wav");
		File.WriteAllBytes(wav, WavFile.Encode(Sine(440, 0.1, 1)));
		var config = AuthoringConfig.Load(Path.Combine(temp.Path, "authoring.json"));
		using var http = new System.Net.Http.HttpClient();
		using var workspace = new Workspace(config, ffmpeg, http);
		workspace.Refresh();

		// Tags fill the credit, finding a web address inside other text; without tags the title is the file name
		var tagged = workspace.SuggestCreditAsync(mp3, "gale.mp3", CancellationToken.None).Result;
		Assert.Equal("Big Gale", tagged.Title);
		Assert.Equal("Someone", tagged.Author);
		Assert.Equal("https://example.org/gale", tagged.Url);
		var untagged = workspace.SuggestCreditAsync(wav, "Breeze Take 2.wav", CancellationToken.None).Result;
		Assert.Equal("Breeze Take 2", untagged.Title);
		Assert.Null(untagged.Author);

		// Adding copies the file into the originals and assigns it to the sound as an unheard candidate with the given credit
		workspace.ImportFileAsync("wind.test", wav, "Breeze Take 2.wav", new FileCredit { Title = "Breeze", Author = "Me", Licence = "CC0" }, CancellationToken.None).Wait();
		var variant = PrepFile.Load(config.PrepFolder, "wind.test").Variants.Single();
		Assert.True(variant.Origin.StartsWith("imported:"));
		Assert.Equal(EVariantStatus.Unheard, variant.Status);
		var record = OriginsFile.Load(config.OriginsPath).Origins[variant.Origin];
		Assert.Equal("Breeze", record.Title);
		Assert.Equal("Me", record.User);
		Assert.True(File.Exists(Path.Combine(config.OriginalsFolder, $"{record.Source}.json")));

		// Rejecting the only use forgets the variant, the origin and the copied original
		workspace.Apply(new JsonObject { ["action"] = "status", ["id"] = "wind.test", ["variant"] = variant.Id, ["status"] = "rejected" });
		Assert.Equal(0, PrepFile.Load(config.PrepFolder, "wind.test").Variants.Count);
		Assert.False(OriginsFile.Load(config.OriginsPath).Origins.ContainsKey(variant.Origin));
		Assert.False(File.Exists(Path.Combine(config.OriginalsFolder, $"{record.Source}.json")));
	}

	/// <summary>Playback matches the game: opening a project renders the recipes the game uses, and editing a recipe replaces a reference master.</summary>
	[Test]
	public void RecipesPlayAsInTheGame()
	{
		// A project with one local recording for one looping sound
		using var temp = new TempFolder();
		File.WriteAllText(Path.Combine(temp.Path, "sounds.json"), $"{{\"schema\":1,\"sections\":[{{\"name\":\"Wind\",\"sounds\":[{BedSound("wind.test")}]}}]}}");
		File.WriteAllText(Path.Combine(temp.Path, "authoring.json"), "{\"schema\":1,\"catalog\":\"sounds.json\",\"originals\":\"originals\",\"prep\":\"prep\",\"origins\":\"origins.json\",\"cache\":\"cache\",\"local\":[{\"name\":\"rec\",\"path\":\"rec\",\"author\":\"Me\",\"licence\":\"CC0\"}]}");
		Directory.CreateDirectory(Path.Combine(temp.Path, "rec", "wind.test"));
		File.WriteAllBytes(Path.Combine(temp.Path, "rec", "wind.test", "gust.wav"), WavFile.Encode(Sine(440, 0.1, 1)));
		Directory.CreateDirectory(Path.Combine(temp.Path, "masters"));
		File.WriteAllBytes(Path.Combine(temp.Path, "masters", "wind.test.1.wav"), WavFile.Encode(Sine(220, 0.1, 1)));
		var config = AuthoringConfig.Load(Path.Combine(temp.Path, "authoring.json"));
		using var http = new System.Net.Http.HttpClient();
		JsonNode Variant(Workspace ws) => ws.Snapshot()["preps"]!["wind.test"]![0]!;

		// Accepting the recording imports and renders it
		var first = new Workspace(config, new Ffmpeg(), http);
		first.Refresh();
		first.Apply(new JsonObject { ["action"] = "status", ["id"] = "wind.test", ["variant"] = "v1", ["status"] = "accepted" });
		Assert.True(WaitFor(() => Variant(first)["render_url"] != null, 30));

		// A reopened project renders the recipe without any edit
		var second = new Workspace(config, new Ffmpeg(), http);
		Assert.True(WaitFor(() => Variant(second)["render_url"] != null, 30));

		// A variant with a reference master plays that file, and is not rendered on opening
		var prep = PrepFile.Load(config.PrepFolder, "wind.test");
		prep.Variants[0].ReferenceMaster = "masters/wind.test.1.wav";
		prep.Save(config.PrepFolder);
		var third = new Workspace(config, new Ffmpeg(), http);
		Assert.True(WaitFor(() => third.Jobs.Pending.Count == 0, 30));
		Assert.Null((string?)Variant(third)["render_url"]);
		Assert.NotNull((string?)Variant(third)["reference_url"]);

		// Editing the recipe makes the recipe what the game plays
		third.Apply(new JsonObject { ["action"] = "recipe", ["id"] = "wind.test", ["variant"] = "v1", ["recipe"] = JsonNode.Parse("[{\"step\":\"gain\",\"db\":-3}]") });
		Assert.Null(PrepFile.Load(config.PrepFolder, "wind.test").Variants[0].ReferenceMaster);
		Assert.True(WaitFor(() => Variant(third)["render_url"] != null, 30));
		Assert.Null((string?)Variant(third)["reference_url"]);
	}

	/// <summary>Picking a recording to mix in imports its original, renders the mix once it arrives, and keeps the recording in use for the game.</summary>
	[Test]
	public void MixInputsAreImportedAndInUse()
	{
		// A project with a long accepted recording and a shorter one that is only a candidate, so its original is not imported
		using var temp = new TempFolder();
		File.WriteAllText(Path.Combine(temp.Path, "sounds.json"), $"{{\"schema\":1,\"sections\":[{{\"name\":\"Wind\",\"sounds\":[{BedSound("wind.test")}]}}]}}");
		File.WriteAllText(Path.Combine(temp.Path, "authoring.json"), "{\"schema\":1,\"catalog\":\"sounds.json\",\"originals\":\"originals\",\"fetched\":\"fetched\",\"prep\":\"prep\",\"origins\":\"origins.json\",\"cache\":\"cache\",\"local\":[{\"name\":\"rec\",\"path\":\"rec\",\"author\":\"Me\",\"licence\":\"CC0\"}]}");
		Directory.CreateDirectory(Path.Combine(temp.Path, "rec", "wind.test"));
		File.WriteAllBytes(Path.Combine(temp.Path, "rec", "wind.test", "gust.wav"), WavFile.Encode(Sine(440, 0.1, 2)));
		File.WriteAllBytes(Path.Combine(temp.Path, "rec", "wind.test", "rain.wav"), WavFile.Encode(Sine(880, 0.1, 1)));
		var config = AuthoringConfig.Load(Path.Combine(temp.Path, "authoring.json"));
		using var http = new System.Net.Http.HttpClient();
		using var workspace = new Workspace(config, new Ffmpeg(), http);
		workspace.Refresh();
		var gust = PrepFile.Load(config.PrepFolder, "wind.test").Variants.Single(x => x.Origin.EndsWith("gust.wav"));
		var rain = PrepFile.Load(config.PrepFolder, "wind.test").Variants.Single(x => x.Origin.EndsWith("rain.wav"));
		workspace.Apply(new JsonObject { ["action"] = "status", ["id"] = "wind.test", ["variant"] = gust.Id, ["status"] = "accepted" });
		Assert.True(WaitFor(() => workspace.Jobs.Pending.Count == 0 && OriginsFile.Load(config.OriginsPath).Origins[gust.Origin].Source != null, 30));
		Assert.Null(OriginsFile.Load(config.OriginsPath).Origins[rain.Origin].Source);

		// Mixing in the candidate imports its original and binds it, and the render ends with the shorter recording
		var recipe = new JsonArray { new JsonObject { ["step"] = "mix", ["db"] = 0, ["inputs"] = new JsonArray { new JsonObject { ["origin"] = rain.Origin, ["start"] = 0, ["db"] = -3 } } } };
		workspace.Apply(new JsonObject { ["action"] = "recipe", ["id"] = "wind.test", ["variant"] = gust.Id, ["recipe"] = recipe });
		JsonNode Gust() => workspace.Snapshot()["preps"]!["wind.test"]!.AsArray().Single(x => (string?)x!["id"] == gust.Id)!;
		Assert.True(WaitFor(() => workspace.Jobs.Pending.Count == 0 && Gust()["render_url"] != null && Gust()["render_error"] == null, 30));
		var input = PrepFile.Load(config.PrepFolder, "wind.test").Variants.Single(x => x.Id == gust.Id).MixInputs.Single();
		Assert.Equal(OriginsFile.Load(config.OriginsPath).Origins[rain.Origin].Source, input.Source);
		var key = ((string)Gust()["render_url"]!).Split('/')[^1];
		Assert.Equal(AudioBuffer.SampleRate, WavFile.Read(Path.Combine(config.CacheFolder, key)).Frames);

		// The game's preparation fetches a mixed-in original that has gone missing, so the mix still cooks
		var ffmpeg = new Ffmpeg();
		var sources = new SourceStore(config.OriginalsFolder, config.FetchedFolder, ffmpeg);
		File.Delete(sources.OriginalPath(input.Source!));
		File.Delete(sources.SidecarPath(input.Source!));
		Directory.Delete(config.CacheFolder, recursive: true);
		var fetcher = new OriginalFetcher(config, sources, new FreesoundClient(http, null));
		var prepared = PreparedSounds.ResolveAsync(config, new RenderCache(config.CacheFolder, sources, ffmpeg), fetcher, ["wind.test"], CancellationToken.None).GetAwaiter().GetResult();
		Assert.True(sources.Contains(input.Source!));
		Assert.Equal(AudioBuffer.SampleRate, WavFile.Read(prepared["wind.test"].Single()).Frames);
	}

	/// <summary>
	/// A variant opened in an external editor takes each version the editor saves over its working copy as its new original, keeping its review state,
	/// recipe and credit, and only the latest edit is kept.
	/// </summary>
	[Test]
	public void EditInExternalEditor()
	{
		// A project with one local recording and an 'editor' that exits at once, so the test writes the edits itself
		using var temp = new TempFolder();
		File.WriteAllText(Path.Combine(temp.Path, "sounds.json"), $"{{\"schema\":1,\"sections\":[{{\"name\":\"Wind\",\"sounds\":[{BedSound("wind.test")}]}}]}}");
		File.WriteAllText(Path.Combine(temp.Path, "authoring.json"), "{\"schema\":1,\"catalog\":\"sounds.json\",\"originals\":\"originals\",\"prep\":\"prep\",\"origins\":\"origins.json\",\"cache\":\"cache\",\"local\":[{\"name\":\"rec\",\"path\":\"rec\",\"author\":\"Me\",\"licence\":\"CC0\",\"url\":\"https://example.org\"}],\"editor\":{\"command\":\"cmd\",\"arguments\":[\"/c\",\"rem\",\"{file}\"]}}");
		Directory.CreateDirectory(Path.Combine(temp.Path, "rec", "wind.test"));
		File.WriteAllBytes(Path.Combine(temp.Path, "rec", "wind.test", "gust.wav"), WavFile.Encode(Sine(440, 0.1, 1)));
		var config = AuthoringConfig.Load(Path.Combine(temp.Path, "authoring.json"));
		using var http = new System.Net.Http.HttpClient();
		using var workspace = new Workspace(config, new Ffmpeg(), http);
		var changed = new List<string>();
		workspace.GameChanged += id => { lock (changed) changed.Add(id); };
		JsonNode Variant() => workspace.Snapshot()["preps"]!["wind.test"]![0]!;
		JsonNode Origin(string origin) => workspace.Snapshot()["origins"]![origin]!;

		// Accept the recording with a recipe, then open it in the editor, which writes the working copy
		workspace.Refresh();
		workspace.Apply(new JsonObject { ["action"] = "status", ["id"] = "wind.test", ["variant"] = "v1", ["status"] = "accepted" });
		Assert.True(WaitFor(() => Variant()["render_url"] != null, 30));
		workspace.Apply(new JsonObject { ["action"] = "recipe", ["id"] = "wind.test", ["variant"] = "v1", ["recipe"] = JsonNode.Parse("[{\"step\":\"gain\",\"db\":-3}]") });
		workspace.Apply(new JsonObject { ["action"] = "open_editor", ["id"] = "wind.test", ["variant"] = "v1" });
		Assert.True(WaitFor(() => Variant()["edit"]?["path"] != null && Variant()["edit"]?["pending"]?.GetValue<bool>() == false, 30));
		Assert.Null((string?)Variant()["edit"]!["error"]);
		var working = (string)Variant()["edit"]!["path"]!;
		Assert.True(File.Exists(working));

		// Saving an edit over the working copy replaces the original, crediting the recording it was made from
		File.WriteAllBytes(working, WavFile.Encode(Sine(330, 0.1, 2)));
		Assert.True(WaitFor(() => ((string?)Variant()["origin"])?.StartsWith("edited:") == true && Variant()["render_pending"]?.GetValue<bool>() == false && Variant()["render_url"] != null, 30));
		var first_edit = (string)Variant()["origin"]!;
		var first_source = (string)Variant()["source"]!;
		Assert.Equal("local:rec/wind.test/gust.wav", (string?)Origin(first_edit)["edit_of"]);
		Assert.Equal("Me", (string?)Origin(first_edit)["user"]);
		Assert.Equal("CC0", (string?)Origin(first_edit)["licence"]);
		Assert.True(File.Exists(Path.Combine(config.OriginalsFolder, $"{first_source}.json")));
		Assert.Equal("accepted", (string?)Variant()["status"]);
		Assert.Equal("gain", (string?)Variant()["recipe"]![0]!["step"]);
		lock (changed)
			Assert.True(changed.Contains("wind.test"));

		// A second edit replaces the first, which nothing uses any more; the recording the edits were made from is kept
		File.WriteAllBytes(working, WavFile.Encode(Sine(550, 0.1, 3)));
		Assert.True(WaitFor(() => (string?)Variant()["origin"] != first_edit && ((string?)Variant()["origin"])?.StartsWith("edited:") == true, 30));
		var second_edit = (string)Variant()["origin"]!;
		Assert.Equal("local:rec/wind.test/gust.wav", (string?)Origin(second_edit)["edit_of"]);
		Assert.Null(workspace.Snapshot()["origins"]![first_edit]);
		Assert.True(!File.Exists(Path.Combine(config.OriginalsFolder, $"{first_source}.json")));
		Assert.NotNull(workspace.Snapshot()["origins"]!["local:rec/wind.test/gust.wav"]);
	}

	/// <summary>An assistant that gives fixed replies in order and records the prompts it was given.</summary>
	private sealed class ScriptedAssistant(params string[] replies) : IAssistant
	{
		private int m_next;

		public List<string> Prompts { get; } = [];

		public AssistantStatus Status { get; } = new(true, "scripted");

		public Task<JsonObject> AskAsync(string prompt, CancellationToken cancel)
		{
			// Each prompt receives the next reply
			Prompts.Add(prompt);
			return Task.FromResult(CopilotAssistant.ParseReply(replies[m_next++]));
		}
	}

	/// <summary>Answers Freesound search, sound page and preview requests with made-up sounds, and records the search requests.</summary>
	private sealed class FakeFreesound : System.Net.Http.HttpMessageHandler
	{
		public List<System.Net.Http.HttpRequestMessage> Searches { get; } = [];

		protected override Task<System.Net.Http.HttpResponseMessage> SendAsync(System.Net.Http.HttpRequestMessage request, CancellationToken cancel)
		{
			// Searches return five sounds with mixed licences; pages describe sound N as "Sound N" by "user", licensed CC-BY 4.0
			var url = request.RequestUri!.ToString();
			System.Net.Http.HttpContent content;
			if (url.Contains("/search/text/", StringComparison.Ordinal))
			{
				Searches.Add(request);
				string Result(int n, string licence) => $"{{\"id\":{n},\"name\":\"Sound {n}\",\"username\":\"user\",\"license\":\"{licence}\",\"duration\":12.5,\"tags\":[\"wind\",\"forest\"],\"description\":\"Wind in trees\"}}";
				var zero = "http://creativecommons.org/publicdomain/zero/1.0/";
				var by = "https://creativecommons.org/licenses/by/4.0/";
				var nc = "https://creativecommons.org/licenses/by-nc/4.0/";
				content = new System.Net.Http.StringContent($"{{\"results\":[{Result(1, zero)},{Result(2, by)},{Result(3, zero)},{Result(4, by)},{Result(5, nc)}]}}");
			}
			else if (System.Text.RegularExpressions.Regex.Match(url, @"/s/(\d+)/$") is { Success: true } page)
			{
				var n = page.Groups[1].Value;
				content = new System.Net.Http.StringContent($"<meta property=\"og:title\" content=\"Sound {n} by user\"><meta property=\"og:url\" content=\"https://freesound.org/people/user/sounds/{n}/\"><a href=\"https://creativecommons.org/licenses/by/4.0/\"></a>\"https://cdn.freesound.org/previews/0/{n}-lq.mp3\"");
			}
			else
			{
				content = new System.Net.Http.ByteArrayContent([1, 2, 3]);
			}
			return Task.FromResult(new System.Net.Http.HttpResponseMessage(System.Net.HttpStatusCode.OK) { Content = content });
		}
	}

	/// <summary>
	/// Finding candidates searches Freesound with the assistant's queries and adds the assistant's picks as unheard variants. Sounds the project already
	/// knows, licences other than CC0 and CC-BY, and picks that were not offered are never added, and the review history guides the assistant.
	/// </summary>
	[Test]
	public void FindCandidatesAddsNewSounds()
	{
		// A sound with one accepted and one rejected Freesound recording
		using var temp = new TempFolder();
		File.WriteAllText(Path.Combine(temp.Path, "sounds.json"), $"{{\"schema\":1,\"sections\":[{{\"name\":\"Wind\",\"sounds\":[{BedSound("wind.test")}]}}]}}");
		File.WriteAllText(Path.Combine(temp.Path, "authoring.json"), "{\"schema\":1,\"catalog\":\"sounds.json\",\"originals\":\"originals\",\"prep\":\"prep\",\"origins\":\"origins.json\",\"cache\":\"cache\",\"previews\":\"previews\",\"freesound\":{\"credentials\":\"freesound.json\"}}");
		File.WriteAllText(Path.Combine(temp.Path, "origins.json"), "{\"schema\":1,\"origins\":{\"freesound:1\":{\"title\":\"Liked wind\",\"user\":\"a\",\"licence\":\"CC0\"},\"freesound:2\":{\"title\":\"Hissy wind\",\"user\":\"b\",\"licence\":\"CC0\"}}}");
		File.WriteAllText(Path.Combine(temp.Path, "freesound.json"), "{\"client_id\":\"id\",\"client_secret\":\"secret\"}");
		var freesound = new FakeFreesound();
		using var http = new System.Net.Http.HttpClient(freesound);
		var assistant = new ScriptedAssistant("```json\n{\"queries\":[\"forest wind\",\"Forest Wind\",\" \",\"leaves\"]}\n```", "{\"picks\":[\"4\",1,\"99\",\"3\",\"5\",\"4\"]}");
		var workspace = new Workspace(AuthoringConfig.Load(Path.Combine(temp.Path, "authoring.json")), new Ffmpeg(), http, assistant);
		workspace.Apply(new JsonObject { ["action"] = "assign", ["id"] = "wind.test", ["origin"] = "freesound:1" });
		workspace.Apply(new JsonObject { ["action"] = "assign", ["id"] = "wind.test", ["origin"] = "freesound:2" });
		workspace.Apply(new JsonObject { ["action"] = "note", ["id"] = "wind.test", ["variant"] = "v2", ["note"] = "too hissy" });
		workspace.Apply(new JsonObject { ["action"] = "status", ["id"] = "wind.test", ["variant"] = "v1", ["status"] = "accepted" });
		workspace.Apply(new JsonObject { ["action"] = "status", ["id"] = "wind.test", ["variant"] = "v2", ["status"] = "rejected" });

		// Run the search to completion
		workspace.Apply(new JsonObject { ["action"] = "find_candidates", ["id"] = "wind.test" });
		Assert.True(WaitFor(() => workspace.Snapshot()["finds"]!["wind.test"]?["pending"]?.GetValue<bool>() == false, 30));
		var find = workspace.Snapshot()["finds"]!["wind.test"]!;
		Assert.Null((string?)find["error"]);
		Assert.Equal("Added 2 candidates for wind.test: Sound 4, Sound 3.", (string?)find["notice"]);

		// The catalog entry and the review history were given to the assistant, and only new CC0 or CC-BY sounds were offered to it
		Assert.True(assistant.Prompts[0].Contains("\"wind.test\"") && assistant.Prompts[0].Contains("Liked wind") && assistant.Prompts[0].Contains("too hissy"));
		Assert.True(assistant.Prompts[1].Contains("\"number\":\"3\"") && assistant.Prompts[1].Contains("\"number\":\"4\""));
		Assert.True(!assistant.Prompts[1].Contains("\"number\":\"1\"") && !assistant.Prompts[1].Contains("\"number\":\"5\""));

		// Each distinct query was searched once with the API key and a CC0 or CC-BY filter
		Assert.Equal(2, freesound.Searches.Count);
		Assert.Equal("Token secret", freesound.Searches[0].Headers.Authorization!.ToString());
		Assert.True(Uri.UnescapeDataString(freesound.Searches[0].RequestUri!.Query).Contains("license:(\"Creative Commons 0\" OR \"Attribution\")"));

		// The picks are new unheard variants, and the known recordings are unchanged
		var variants = PrepFile.Load(workspace.Config.PrepFolder, "wind.test").Variants;
		Assert.Equal(4, variants.Count);
		Assert.Equal("freesound:4", variants[2].Origin);
		Assert.Equal("freesound:3", variants[3].Origin);
		Assert.True(variants.Skip(2).All(x => x.Status == EVariantStatus.Unheard));
		Assert.Equal(EVariantStatus.Rejected, variants[1].Status);
		Assert.Equal("CC-BY 4.0", OriginsFile.Load(workspace.Config.OriginsPath).Origins["freesound:4"].Licence);
	}

	/// <summary>Finding candidates is refused without an assistant, and model replies are read even when wrapped in other text.</summary>
	[Test]
	public void FindCandidatesNeedsAssistant()
	{
		// A project with no assistant
		using var temp = new TempFolder();
		File.WriteAllText(Path.Combine(temp.Path, "sounds.json"), $"{{\"schema\":1,\"sections\":[{{\"name\":\"Wind\",\"sounds\":[{BedSound("wind.test")}]}}]}}");
		File.WriteAllText(Path.Combine(temp.Path, "authoring.json"), "{\"schema\":1,\"catalog\":\"sounds.json\",\"originals\":\"originals\",\"prep\":\"prep\",\"origins\":\"origins.json\",\"cache\":\"cache\"}");
		var config = AuthoringConfig.Load(Path.Combine(temp.Path, "authoring.json"));
		using var http = new System.Net.Http.HttpClient();
		var workspace = new Workspace(config, new Ffmpeg(), http);
		Assert.Equal(AuthoringConfig.DefaultAssistantModel, config.AssistantModel);
		Assert.True(!workspace.Snapshot()["assistant"]!["available"]!.GetValue<bool>());
		Assert.Throws<InvalidOperationException>(() => workspace.Apply(new JsonObject { ["action"] = "find_candidates", ["id"] = "wind.test" }));

		// Replies may be fenced or explained; text without an object is an error
		Assert.Equal(1, CopilotAssistant.ParseReply("Here you go:\n```json\n{\"a\":1}\n```")["a"]!.GetValue<int>());
		Assert.Throws<InvalidOperationException>(() => CopilotAssistant.ParseReply("Sorry, I can't help."));
	}

	/// <summary>Project commands are optional; their arguments are kept as given and their directory is a project path defaulting to the configuration folder.</summary>
	[Test]
	public void ApplyCommandConfig()
	{
		// One configuration without an apply command, one with a directory, and one without
		using var temp = new TempFolder();
		const string Common = "\"schema\":1,\"catalog\":\"sounds.json\",\"originals\":\"originals\",\"prep\":\"prep\",\"origins\":\"origins.json\",\"cache\":\"cache\"";
		var path = Path.Combine(temp.Path, "authoring.json");
		File.WriteAllText(path, $"{{{Common}}}");
		Assert.Null(AuthoringConfig.Load(path).Apply);

		File.WriteAllText(path, $"{{{Common},\"apply\":{{\"command\":\"dotnet\",\"arguments\":[\"msbuild\",\"a b.proj\"],\"directory\":\"..\"}}}}");
		var apply = AuthoringConfig.Load(path).Apply!;
		Assert.Equal("dotnet", apply.Command);
		Assert.Equal(new[] { "msbuild", "a b.proj" }, apply.Arguments);
		Assert.Equal(Path.GetDirectoryName(temp.Path), apply.Directory);
		Assert.Equal("dotnet msbuild \"a b.proj\"", apply.Display);

		File.WriteAllText(path, $"{{{Common},\"apply\":{{\"command\":\"cook.cmd\"}}}}");
		apply = AuthoringConfig.Load(path).Apply!;
		Assert.Equal(temp.Path, apply.Directory);
		Assert.Equal(0, apply.Arguments.Count);

		// A command is required once 'apply' is given
		File.WriteAllText(path, $"{{{Common},\"apply\":{{}}}}");
		Assert.Throws<InvalidDataException>(() => AuthoringConfig.Load(path));

		// The send command substitutes the sound ID into its arguments only
		File.WriteAllText(path, $"{{{Common},\"send\":{{\"command\":\"{{id}}.cmd\",\"arguments\":[\"--sound={{id}}\",\"x\"]}}}}");
		var send = AuthoringConfig.Load(path).Send!.ForSound("wind.gust");
		Assert.Equal("{id}.cmd", send.Command);
		Assert.Equal(new[] { "--sound=wind.gust", "x" }, send.Arguments);
		Assert.Null(AuthoringConfig.Load(path).Apply);
	}

	/// <summary>A catalog is written in one standard layout that reads back to the same sounds and the same text, with optional values only when set.</summary>
	[Test]
	public void CatalogFormatRoundTrips()
	{
		// Every kind of value, including a whole-number gain, 'active' and 'retrigger' objects, and text that needs escaping
		const string Text =
			"{\r\n\t\"schema\": 1,\r\n\t\"sections\": [\r\n" +
			"\t\t{\r\n\t\t\t\"name\": \"Wind\",\r\n\t\t\t\"sounds\": [\r\n" +
			"\t\t\t\t{\"id\": \"wind.calm\", \"placement\": \"bed\", \"loop\": true, \"channels\": \"stereo\", \"length_s\": [30, 60], \"load\": \"stream\", \"group\": \"wind\", \"gain\": 1.0, \"active\": {\"times\": [\"dawn\", \"day\"], \"peak\": \"day\"}, \"description\": \"Calm \\\"still\\\" air\"},\r\n" +
			"\t\t\t\t{\"id\": \"wind.gust\", \"placement\": \"positioned\", \"loop\": false, \"channels\": \"mono\", \"load\": \"memory\", \"range_m\": 26.5, \"group\": \"wind\", \"gain\": 0.08, \"retrigger\": {\"max_per_emitter\": 1, \"when_busy\": \"refuse\"}}\r\n" +
			"\t\t\t]\r\n\t\t},\r\n" +
			"\t\t{\r\n\t\t\t\"name\": \"Lab\",\r\n\t\t\t\"sounds\": [\r\n" +
			"\t\t\t\t{\"id\": \"lab.radio\", \"placement\": \"bed\", \"loop\": true, \"channels\": \"mono\", \"length_s\": [0.1, 0.25], \"load\": \"stream\", \"group\": \"lab\", \"gain\": 0.58}\r\n" +
			"\t\t\t]\r\n\t\t}\r\n\t]\r\n}\r\n";
		using var temp = new TempFolder();
		var path = Path.Combine(temp.Path, "sounds.json");
		File.WriteAllText(path, Text);

		// Reading then writing changes nothing
		var sounds = Catalog.Read(path);
		Assert.Equal(Text, Catalog.Format(sounds));
		Assert.Equal("Calm \"still\" air", sounds[0].Description);
		Assert.Equal(new LengthRange(0.1, 0.25), sounds[2].LengthS!.Value);
		Assert.Equal(26.5, sounds[1].RangeM!.Value);
		Catalog.Write(path, sounds);
		Assert.Equal(Text, File.ReadAllText(path));
	}

	/// <summary>Reading a catalog rejects anything the game would not accept, and a failed write leaves the file unchanged.</summary>
	[Test]
	public void CatalogRejectsInvalidSounds()
	{
		// Each catalog differs from a valid one in a single way
		using var temp = new TempFolder();
		var path = Path.Combine(temp.Path, "sounds.json");
		string Catalog1(string sounds) => $"{{\"schema\":1,\"sections\":[{{\"name\":\"s\",\"sounds\":[{sounds}]}}]}}";
		foreach (var bad in new[]
		{
			Catalog1(BedSound("x").Replace("}", ",\"len\":1}")),
			Catalog1(BedSound("x").Replace("\"bed\"", "\"positioned\"").Replace("}", ",\"range_m\":5}")),
			Catalog1(BedSound("x").Replace("}", ",\"range_m\":5}")),
			Catalog1($"{BedSound("x")},{BedSound("x")}"),
			Catalog1(BedSound("X")),
			Catalog1(BedSound("x")).Replace("\"schema\":1,", ""),
		})
		{
			File.WriteAllText(path, bad);
			Assert.Throws<InvalidDataException>(() => Catalog.Read(path));
		}

		// Writing an invalid list throws before the file is touched
		File.WriteAllText(path, Catalog1(BedSound("x")));
		var sounds = Catalog.Read(path);
		var before = File.ReadAllText(path);
		Assert.Throws<InvalidDataException>(() => Catalog.Write(path, [.. sounds, sounds[0]]));
		Assert.Equal(before, File.ReadAllText(path));
	}

	/// <summary>Sounds are added at the end of their section, edited in place, moved to the end of a new section, and removed with empty sections.</summary>
	[Test]
	public void CatalogAddReplaceRemove()
	{
		// Two sections of two sounds
		var a = Sound("a", "one");
		var b = Sound("b", "one");
		var c = Sound("c", "two");
		var d = Sound("d", "two");
		IReadOnlyList<CatalogSound> sounds = [a, b, c, d];

		// An added sound goes last in its section, and a new section goes last
		Assert.True(Catalog.Add(sounds, Sound("e", "one")).Select(x => x.Id).SequenceEqual(["a", "b", "e", "c", "d"]));
		Assert.True(Catalog.Add(sounds, Sound("e", "three")).Select(x => x.Id).SequenceEqual(["a", "b", "c", "d", "e"]));
		Assert.Throws<InvalidDataException>(() => Catalog.Add(sounds, Sound("a", "two")));

		// An edit within a section keeps its place, even when renamed; a new section moves it to that section's end
		Assert.True(Catalog.Replace(sounds, "a", a with { Id = "z", Gain = 0.5 }).Select(x => x.Id).SequenceEqual(["z", "b", "c", "d"]));
		Assert.True(Catalog.Replace(sounds, "a", a with { Section = "two" }).Select(x => x.Id).SequenceEqual(["b", "c", "d", "a"]));
		Assert.Throws<InvalidDataException>(() => Catalog.Replace(sounds, "q", a));
		Assert.Throws<InvalidDataException>(() => Catalog.Replace(sounds, "a", a with { Id = "b" }));

		// Removing the last sound of a section removes the section
		var removed = Catalog.Remove(Catalog.Remove(sounds, "c"), "d");
		Assert.True(removed.Select(x => x.Id).SequenceEqual(["a", "b"]));
		Assert.False(Catalog.Format(removed).Contains("\"two\""));
		Assert.Throws<InvalidDataException>(() => Catalog.Remove(sounds, "q"));
	}

	/// <summary>
	/// Sounds can be added, edited, renamed and deleted through the workspace. A rename takes the sound's preparation file and own reference masters with it,
	/// and a delete removes them, but a master that another sound uses is left alone.
	/// </summary>
	[Test]
	public void WorkspaceEditsSounds()
	{
		// A project whose sound 'a' has a variant with its own master and one shared with sound 'b'
		using var temp = new TempFolder();
		File.WriteAllText(Path.Combine(temp.Path, "sounds.json"), $"{{\"schema\":1,\"sections\":[{{\"name\":\"s\",\"sounds\":[{BedSound("a")},{BedSound("b")}]}}]}}");
		File.WriteAllText(Path.Combine(temp.Path, "authoring.json"), "{\"schema\":1,\"catalog\":\"sounds.json\",\"originals\":\"originals\",\"prep\":\"prep\",\"origins\":\"origins.json\",\"cache\":\"cache\"}");
		var config = AuthoringConfig.Load(Path.Combine(temp.Path, "authoring.json"));
		Directory.CreateDirectory(config.PrepFolder);
		Directory.CreateDirectory(Path.Combine(temp.Path, "masters"));
		File.WriteAllText(Path.Combine(temp.Path, "masters", "a.1.flac"), "own");
		File.WriteAllText(Path.Combine(temp.Path, "masters", "shared.flac"), "shared");
		var prep_a = new PrepFile { Id = "a" };
		prep_a.AddVariant("hand_made:a.1", null, EVariantStatus.Accepted).ReferenceMaster = "masters/a.1.flac";
		prep_a.AddVariant("hand_made:shared", null, EVariantStatus.Accepted).ReferenceMaster = "masters/shared.flac";
		prep_a.Save(config.PrepFolder);
		var prep_b = new PrepFile { Id = "b" };
		prep_b.AddVariant("hand_made:shared", null, EVariantStatus.Accepted).ReferenceMaster = "masters/shared.flac";
		prep_b.Save(config.PrepFolder);
		using var http = new System.Net.Http.HttpClient();
		var workspace = new Workspace(config, new Ffmpeg(), http);
		JsonObject Action(string action, string? id, string section, string? sound) => new()
		{
			["action"] = action,
			["id"] = id,
			["section"] = section,
			["sound"] = sound != null ? JsonNode.Parse(sound) : null,
		};

		// Adding needs a valid, unused ID; the new sound is last in its new section
		Assert.Throws<InvalidDataException>(() => workspace.Apply(Action("add_sound", null, "t", BedSound("a"))));
		Assert.Equal("Added c.", workspace.Apply(Action("add_sound", null, "t", BedSound("c"))));
		Assert.True(Catalog.Read(config.CatalogPath).Select(x => (x.Id, x.Section)).SequenceEqual([("a", "s"), ("b", "s"), ("c", "t")]));

		// An edit that keeps the ID changes only the catalog
		workspace.Apply(Action("edit_sound", "c", "t", BedSound("c").Replace("1.0", "0.25")));
		Assert.Equal(0.25, Catalog.Read(config.CatalogPath).Single(x => x.Id == "c").Gain);

		// A rename moves the preparation file and the sound's own master, but not the shared one
		Assert.Equal("Renamed a to z.", workspace.Apply(Action("edit_sound", "a", "s", BedSound("z"))));
		Assert.False(File.Exists(PrepFile.PathFor(config.PrepFolder, "a")));
		var prep_z = PrepFile.Load(config.PrepFolder, "z");
		Assert.True(prep_z.Variants.Select(x => x.ReferenceMaster).SequenceEqual(["masters/z.1.flac", "masters/shared.flac"]));
		Assert.Equal("own", File.ReadAllText(Path.Combine(temp.Path, "masters", "z.1.flac")));
		Assert.False(File.Exists(Path.Combine(temp.Path, "masters", "a.1.flac")));
		Assert.True(Catalog.Read(config.CatalogPath).Select(x => x.Id).SequenceEqual(["z", "b", "c"]));

		// A rename onto existing files changes nothing
		File.WriteAllText(Path.Combine(temp.Path, "masters", "y.1.flac"), "in the way");
		Assert.Throws<InvalidOperationException>(() => workspace.Apply(Action("edit_sound", "z", "s", BedSound("y"))));
		Assert.True(File.Exists(PrepFile.PathFor(config.PrepFolder, "z")));
		Assert.True(Catalog.Read(config.CatalogPath).Select(x => x.Id).SequenceEqual(["z", "b", "c"]));

		// A delete removes the sound, its preparation file and its own master, keeping the master that 'b' still uses
		Assert.Equal("Deleted z.", workspace.Apply(new JsonObject { ["action"] = "delete_sound", ["id"] = "z" }));
		Assert.False(File.Exists(PrepFile.PathFor(config.PrepFolder, "z")));
		Assert.False(File.Exists(Path.Combine(temp.Path, "masters", "z.1.flac")));
		Assert.True(File.Exists(Path.Combine(temp.Path, "masters", "shared.flac")));
		Assert.True(Catalog.Read(config.CatalogPath).Select(x => x.Id).SequenceEqual(["b", "c"]));
	}

	/// <summary>A minimal valid looping bed in catalog form.</summary>
	private static string BedSound(string id)
	{
		return $"{{\"id\":\"{id}\",\"placement\":\"bed\",\"loop\":true,\"channels\":\"stereo\",\"load\":\"stream\",\"group\":\"g\",\"gain\":1.0}}";
	}

	/// <summary>A minimal valid looping bed in 'section'.</summary>
	private static CatalogSound Sound(string id, string section)
	{
		return new CatalogSound { Id = id, Section = section, Placement = ESoundPlacement.Bed, Loop = true, Channels = ESoundChannels.Stereo, Load = ESoundLoad.Stream, Group = "g", Gain = 1.0 };
	}

	/// <summary>Local files are assigned to the sound ID their folder names, or else the one their path's words suggest.</summary>
	[Test]
	public void GuessSoundIds()
	{
		// A folder named for an ID wins over words, and the deepest such folder wins; plural words match singular ID parts, and fewer parts win ties
		var ids = new[] { "wind.forest.10", "wind.forest.still", "mind.speech", "birds.dawn", "lab.radio" };
		Assert.Equal("lab.radio", Workspace.GuessSoundId("Lab_Radio/mind speech.mp3", ids));
		Assert.Equal("wind.forest.10", Workspace.GuessSoundId("lab-radio/wind.forest.10/gust.wav", ids));
		Assert.Equal("mind.speech", Workspace.GuessSoundId("speech/speech-2.wav", ids));
		Assert.Equal("wind.forest.still", Workspace.GuessSoundId("winds/forest_still.wav", ids));
		Assert.Equal("birds.dawn", Workspace.GuessSoundId("bird.mp3", ids));
		Assert.Equal(null, Workspace.GuessSoundId("misc/broadcast.wav", ids));
	}

	/// <summary>A burst of edits makes one publish, and edits during a publish make a second one after it.</summary>
	[Test]
	public void AutoPublishCoalesces()
	{
		// The first publish is held open, so edits made during it can be checked
		var published = new List<IReadOnlyList<string>>();
		var release_first = new TaskCompletionSource();
		var second_done = new TaskCompletionSource();
		using var auto = new AutoPublish(TimeSpan.FromMilliseconds(50), async ids =>
		{
			// Record each publish; only the first waits
			var count = 0;
			lock (published)
			{
				published.Add(ids);
				count = published.Count;
			}
			if (count == 1)
				await release_first.Task;
			else
				second_done.SetResult();
		});

		// Repeated and interleaved edits publish each ID once, in first-edit order
		auto.Touch("b");
		auto.Touch("a");
		auto.Touch("b");
		var deadline = DateTime.UtcNow.AddSeconds(10);
		while (published.Count == 0 && DateTime.UtcNow < deadline)
			Thread.Sleep(10);

		Assert.Equal(1, published.Count);
		Assert.True(published[0].SequenceEqual(["b", "a"]));
		Assert.Equal(0, auto.Waiting.Count);

		// An edit during the publish waits for it, then publishes alone
		auto.Touch("c");
		Thread.Sleep(200);
		Assert.Equal(1, published.Count);
		Assert.True(auto.Waiting.SequenceEqual(["c"]));
		release_first.SetResult();
		Assert.True(second_done.Task.Wait(TimeSpan.FromSeconds(10)));
		Assert.True(published[1].SequenceEqual(["c"]));
	}

	/// <summary>Only review actions that change what the game plays trigger a publish.</summary>
	[Test]
	public void GameActionsChangeTheGame()
	{
		// Catalog edits change what the game plays; notes, finalised flags, assignment and rendering only change the review
		foreach (var action in new[] { "add_sound", "edit_sound", "delete_sound", "status", "move", "recipe" })
			Assert.True(Workspace.GameActions.Contains(action));

		foreach (var action in new[] { "note", "finalised", "assign", "useful", "prepare", "add_freesound" })
			Assert.False(Workspace.GameActions.Contains(action));
	}
}
#endif
