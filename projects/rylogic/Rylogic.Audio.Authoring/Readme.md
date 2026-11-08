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
	"local": [
		{ "name": "generated", "path": "../../artifacts/generated-sounds", "author": "ElevenLabs", "licence": "generated" },
		{ "name": "archive", "path": "D:/Recordings/Archive", "author": "Example Archive", "licence": "Public domain", "url": "https://example.org" }
	],
	"freesound": { "credentials": "%LOCALAPPDATA%/MyGame/freesound.json" },
	"apply": { "command": "dotnet", "arguments": ["msbuild", "build/Build.proj", "-t:CookAssets"], "directory": "../.." },
	"send": { "command": "dotnet", "arguments": ["msbuild", "build/Build.proj", "-t:SendSound", "-p:SoundId={id}"], "directory": "../.." }
}
```

- `catalog`: the project's sound definitions. The library reads each sound's `id`, its section, description, `loop` flag and `load` (`memory` or
  `stream`); every other property belongs to the project. The only changes the library makes to this file are setting a sound's `loop` flag or `load`;
  each edits just the value, keeping the file's layout.
- `originals`: one `<source-id>.<type>` per recording with a `<source-id>.json` provenance sidecar. FLAC, MP3 and Ogg downloads are kept byte-for-byte;
  any other type is converted to lossless FLAC. Keep this folder in version control.
- `fetched` (optional): the same, for recordings that can be downloaded again (Freesound). This folder need not be version controlled: a missing
  original is downloaded again when it is needed and must match its recorded source ID. Without it, these originals are kept in `originals`.
- `prep`: one `<sound-id>.json` per sound listing its variants in order, each with a stable variant ID, origin, source, review status and recipe. Keep
  this folder in version control.
- `origins`: the registry of every recording that has been browsed, and the source each has been imported as.
- `cache`: disposable renders, named by a hash of everything that can change them. Delete it at any time.
- `local` (optional): folders of recordings on this machine (generated, recorded or collected), each searched recursively for `.wav`, `.mp3`, `.flac`
  and `.ogg` files. Every file is an origin `local:<name>/<relative path>` credited to the folder's `author`, `licence` and optional `url`. The authoring
  tool adds new files as unheard variants of the sound ID named by their subfolder (`lab-radio/`, `lab_radio/` or `lab.radio/` all name `lab.radio`;
  the deepest such folder wins), or else of the sound ID whose name parts best match the words in the file's path. Accepting one imports it into
  `originals`, because it cannot be downloaded again.
- `apply` (optional): a command that makes the project use the current review results, usually its asset cook. It runs automatically when edits are
  published (see below), and an Apply button runs it on demand, for changes made outside the tool. It runs `command` with `arguments` (passed unchanged)
  in `directory` (default: the configuration folder). The tool shows the command's last output line, or the end of its output if it fails.
- `send` (optional): a command that makes running instances of the project play the current preparation of one sound, so it can be heard in place
  without a full cook. It runs like `apply`, with `{id}` in each argument replaced by the sound ID.

When the project has either command, the tool publishes edits automatically. About 2 s after the last review action that changes what the project
plays (accepting or rejecting a variant, reordering, a recipe edit, adopting a recipe, and the catalog `loop` and `load` values), it runs `send` for each
edited sound and then `apply` once. Edits made during a publish wait for the next pause. Only one project command runs at a time.

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
