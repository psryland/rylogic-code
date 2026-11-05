# Rylogic.Audio.Authoring

Non-destructive preparation of game audio. Projects keep their original recordings untouched and describe the processing each sound variant needs as
an editable recipe. Every consumer renders from the original plus the recipe with the same deterministic renderer, so the authoring preview, live
updates and a project's asset cooker always produce the same audio.

The library is an authoring and cooking dependency only. Shipped games play cooked assets and never reference it. Decoding and lossless conversion use
ffmpeg, found on `PATH` or given explicitly to `Ffmpeg`.

## Project configuration

A project supplies an `authoring.json` file. Relative paths are resolved against the folder holding the file, and environment variables are expanded.

```json
{
	"schema": 1,
	"catalog": "sounds.json",
	"originals": "originals",
	"prep": "prep",
	"origins": "origins.json",
	"cache": "../../artifacts/sound-prep",
	"fetched": "../../artifacts/sound-fetched",
	"previews": "../../artifacts/sound-candidates",
	"generated": "../../artifacts/generated-sounds",
	"freesound": { "credentials": "%LOCALAPPDATA%/MyGame/freesound.json" }
}
```

- `catalog`: the project's sound definitions. The library reads only each sound's `id`, its section and description; every other property belongs to
  the project.
- `originals`: one lossless `<source-id>.flac` per recording with a `<source-id>.json` provenance sidecar. Keep this folder in version control.
- `fetched` (optional): the same, for recordings that can be downloaded again (Freesound). This folder need not be version controlled: a missing
  original is downloaded again when it is needed and must match its recorded source ID. Without it, these originals are kept in `originals`.
- `prep`: one `<sound-id>.json` per sound listing its variants in order, each with a stable variant ID, origin, source, review status and recipe. Keep
  this folder in version control.
- `origins`: the registry of every recording that has been browsed, and the source each has been imported as.
- `cache`: disposable renders, named by a hash of everything that can change them. Delete it at any time.

## Recipes

A recipe is an ordered list of built-in steps, each of which can be bypassed: `trim`, `cut`, `fade_in`, `fade_out`, `gain`, `loudness`, `high_pass`,
`low_pass`, `eq` and `loop`. `RecipeRenderer.Render` applies them to 48 kHz float audio. Rendering always starts from the original.

## Resolving prepared audio

`PreparedSounds.ResolveAsync` returns the prepared file of every accepted variant, in runtime order, rendering through the `RenderCache` as needed. A
sound with no accepted variants is absent: it is intentionally unassigned. A missing original is fetched again through the `OriginalFetcher`; a missing
reference, or an original that cannot be fetched as recorded, is an error.

```csharp
var config = AuthoringConfig.Load("assets/sounds/authoring.json");
var ffmpeg = new Ffmpeg();
var sources = new SourceStore(config.OriginalsFolder, config.FetchedFolder, ffmpeg);
var cache = new RenderCache(config.CacheFolder, sources, ffmpeg);
var fetcher = new OriginalFetcher(config, sources, new FreesoundClient(http, config.FreesoundCredentialsPath));
var prepared = await PreparedSounds.ResolveAsync(config, cache, fetcher, sound_ids, cancel);
```

The `Rylogic.Audio.Authoring.App` project serves the browser-based authoring tool for a configuration.
