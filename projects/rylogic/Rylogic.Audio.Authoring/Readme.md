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
	"send": { "command": "dotnet", "arguments": ["msbuild", "build/Build.proj", "-t:SendSound", "-p:SoundId={id}"], "directory": "../.." },
	"assistant": { "model": "claude-haiku-5.5" },
	"editor": { "command": "C:/Program Files/Audacity 4/bin/Audacity4.exe", "arguments": ["{file}"] }
}
```

- `catalog`: the sound definitions, owned by the library. The file has `"schema": 1` and a list of named `sections`, each with its `sounds` in order.
  Each sound has `id`, `placement` (`bed` surrounds the listener; `positioned` comes from a point, must be `mono` and needs `range_m`), `loop`,
  `channels` (`mono` or `stereo`), `load` (`memory` or `stream`), `group` and `gain`, and optionally `length_s` (`[min, max]`), `priority` (a whole
  number from 0 to 100; higher wins when the game has too few voices), `active` and `retrigger` (objects passed through unchanged) and `description`. Unknown properties are rejected. The authoring tool adds, edits, renames and deletes sounds, writing the
  whole file in one standard layout with one line per sound. Renaming a sound renames its preparation file and the reference masters named
  `<id>.*` that only it uses; deleting a sound deletes them. Game code that names a renamed or deleted ID must be changed by hand.
- `originals`: one `<source-id>.<type>` per recording with a `<source-id>.json` provenance sidecar. FLAC, MP3, Ogg and Opus downloads are kept byte-for-byte;
  any other type is converted to lossless FLAC. Keep this folder in version control.
- `fetched` (optional): the same, for recordings that can be downloaded again (Freesound). This folder need not be version controlled: a missing
  original is downloaded again when it is needed and must match its recorded source ID. Without it, these originals are kept in `originals`.
- `prep`: one `<sound-id>.json` per sound listing its variants in order, each with a stable variant ID, origin, source, review status and recipe. Keep
  this folder in version control.
- `origins`: the registry of every recording that has been browsed, and the source each has been imported as.
- `cache`: disposable renders, named by a hash of everything that can change them. Delete it at any time.
- `local` (optional): folders of recordings on this machine (generated, recorded or collected), each searched recursively for `.wav`, `.mp3`, `.flac`,
  `.ogg` and `.opus` files. Every file is an origin `local:<name>/<relative path>` credited to the folder's `author`, `licence` and optional `url`. The authoring
  tool adds new files as unheard variants of the sound ID named by their subfolder (`lab-radio/`, `lab_radio/` or `lab.radio/` all name `lab.radio`;
  the deepest such folder wins), or else of the sound ID whose name parts best match the words in the file's path. Accepting one imports it into
  `originals`, because it cannot be downloaded again.
- `apply` (optional): a command that makes the project use the current review results, usually its asset cook. It runs automatically when edits are
  published (see below), and an Apply button runs it on demand, for changes made outside the tool. It runs `command` with `arguments` (passed unchanged)
  in `directory` (default: the configuration folder). The tool shows the command's last output line, or the end of its output if it fails.
- `send` (optional): a command that makes running instances of the project play the current preparation of one sound, so it can be heard in place
  without a full cook. It runs like `apply`, with `{id}` in each argument replaced by the sound ID.
- `assistant` (optional): the AI model behind "Find more candidates" (`model`, default `claude-haiku-5.5`), and optionally the Copilot CLI
  executable to run (`command`, default `copilot` on PATH).
- `editor` (optional): the audio editor behind "Edit in Audacity" (see below). It runs like `apply`, with `{file}` in each argument replaced by the
  path of the working copy; `arguments` defaults to `["{file}"]`. Without this setting, Audacity is used if it is installed in its default folder
  (Audacity 4 is preferred to Audacity 3).

When the project has either command, the tool publishes edits automatically. About 2 s after the last review action that changes what the project
plays (accepting or rejecting a variant, reordering, a recipe edit, an imported edit from the external editor, and adding, editing or deleting a sound), it runs `send` for each
edited sound and then `apply` once. Edits made during a publish wait for the next pause. Only one project command runs at a time.

## Finding candidates

"Find more candidates" asks an AI model, through the user's own GitHub Copilot CLI sign-in, to find new Freesound recordings for the selected
sound. The model sees only the sound's catalog entry and the titles and notes of its accepted and rejected variants. It writes up to 4 search
queries; the tool runs them through the Freesound API (which needs only the API credentials, not a connection), keeps CC0 and CC-BY sounds that the
project does not know yet, and asks the model to choose up to 5 of them. Each choice is added as an unheard variant. A search takes a minute or
two. To enable it, install the Copilot CLI (`winget install GitHub.Copilot`), run `copilot` once to sign in, and restart the tool.

## Editing in Audacity

A recipe cannot fix everything, such as a click in the middle of a recording. "Edit in Audacity" opens a WAV working copy of the variant's
unprocessed original (in `cache/edits`) in the configured editor. Each time the editor exports over that same file (in Audacity, File > Export
Audio; saving a project does not change the file), the tool imports the new version as an original of kind `edited:<source id>` and gives it to the
variant. The variant keeps its status, note and recipe, and the recipe now applies to the edited recording. The edit keeps the credit of the recording
it was made from, and records that recording as `edit_of`. Only the latest edit is kept: an earlier edit that no variant uses any more is deleted. Edits
are stored in `originals`, because they cannot be downloaded again.

## Recipes

A recipe is an ordered list of built-in steps, each of which can be bypassed: `trim`, `cut`, `fade_in`, `fade_out`, `gain`, `loudness`, `high_pass`,
`low_pass`, `eq`, `denoise`, `loop` and `mix`. `RecipeRenderer.Render` applies them to 48 kHz float audio. Rendering always starts from the
original. A variant may start with a reference master, an existing prepared file that stands in for the recipe. The first recipe edit replaces it, so
playback in the tool always matches the game; bypass every step to hear the original.

`denoise` removes steady hiss. Set `start` and `end` to a section, at least 43 ms long, that holds only the hiss, and `db` to how far to turn it down.
About 12 dB sounds natural; much more can make the remaining sound 'watery'. It suits distinct sounds such as calls, clicks and voices, with gaps
between them. Do not use it on textures that are themselves noise-like, such as wind, rain or surf, because it removes them too; a `low_pass` step
is better there.

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
