#if PR_UNITTESTS
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
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
		var expected = RecipeRenderer.Render(ffmpeg.DecodeAsync(sources.FlacPath(sidecar.Id), CancellationToken.None).GetAwaiter().GetResult(), recipe);
		Assert.True(WavFile.Encode(expected).AsSpan().SequenceEqual(WavFile.Encode(rendered)));

		// Bypassed steps do not change the key; parameter changes do
		var with_bypass = recipe.Append(new RecipeStep { Step = ERecipeStep.Eq, Frequency = 500, Db = 6, Q = 1, Bypass = true }).ToList();
		Assert.Equal(key, cache.KeyAsync(sidecar.Id, with_bypass, CancellationToken.None).GetAwaiter().GetResult());
		Assert.NotEqual(key, cache.KeyAsync(sidecar.Id, [new() { Step = ERecipeStep.Gain, Db = -5 }], CancellationToken.None).GetAwaiter().GetResult());

		// A modified original is detected rather than silently rendered
		File.AppendAllText(sources.FlacPath(sidecar.Id), "x");
		Assert.Throws<InvalidDataException>(() => sources.VerifyAsync(sidecar.Id, CancellationToken.None).GetAwaiter().GetResult());
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
		Assert.Throws<InvalidDataException>(() => PreparedSounds.ResolveAsync(config, cache, fetcher, ["b", "silent"], CancellationToken.None).GetAwaiter().GetResult());
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
		Assert.Equal(Path.Combine(config.OriginalsFolder, $"{source}.flac"), sources.FlacPath(source));
		Directory.CreateDirectory(config.FetchedFolder!);
		foreach (var ext in new[] { "flac", "json" })
			File.Move(Path.Combine(config.OriginalsFolder, $"{source}.{ext}"), Path.Combine(config.FetchedFolder!, $"{source}.{ext}"));

		Assert.Equal(Path.Combine(config.FetchedFolder!, $"{source}.flac"), sources.FlacPath(source));
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
		File.Delete(sources.FlacPath(source));
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
		File.WriteAllText(Path.Combine(temp.Path, "sounds.json"), "{\"sections\":[{\"name\":\"Wind\",\"sounds\":[{\"id\":\"wind.test\",\"loop\":true}]}]}");
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
		Assert.Equal(Path.Combine(temp.Path, "fetched", $"{source}.flac"), workspace.Sources.FlacPath(source!));
		Assert.Equal(2, Directory.EnumerateFileSystemEntries(Path.Combine(temp.Path, "fetched")).Count());

		// A failing recipe reports its error and keeps the last working render
		var working = RenderUrl();
		workspace.Apply(new System.Text.Json.Nodes.JsonObject { ["action"] = "recipe", ["id"] = "wind.test", ["variant"] = "v1", ["recipe"] = System.Text.Json.Nodes.JsonNode.Parse("[{\"step\":\"low_pass\",\"frequency\":30000}]") });
		Assert.True(WaitFor(() => workspace.Snapshot()["preps"]!["wind.test"]![0]!["render_error"] != null, 30));
		Assert.Equal(working, RenderUrl());
		Assert.Equal(ERecipeStep.LowPass, PrepFile.Load(workspace.Config.PrepFolder, "wind.test").Variants[0].Recipe[0].Step);
	}

	/// <summary>Local files are found recursively, credited to their folder, and assigned to the folder's sound ID or else the one their path suggests.</summary>
	[Test]
	public void LocalFoldersCreditAndAssign()
	{
		// Two local folders: one names its sound ID with a subfolder, the other relies on words in its paths
		using var temp = new TempFolder();
		File.WriteAllText(Path.Combine(temp.Path, "sounds.json"), "{\"sections\":[{\"name\":\"Lab\",\"sounds\":[{\"id\":\"lab.radio\"},{\"id\":\"mind.speech\"}]}]}");
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
}
#endif
