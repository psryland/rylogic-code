# Rylogic Audio authoring plan

A reusable, non-destructive pipeline for preparing game sounds: **select sources -> edit processing -> automatic preview -> optionally update a running
game**. It works like a source document with a modifier stack: the high-quality original is never changed, and the preparation needed to make the game
asset is saved as an editable recipe. Petri is the first consumer; its game-specific plan is `petri2\docs\plans\audio.md` (Preparation pipeline).

## Projects

- **`Rylogic.Audio.Authoring`** (class library): the data model, the recipe renderer, the render cache, the background job queue, and the revision-folder
  format. Game asset cookers reference this library so that preview, live updates and cooking share one implementation.
- **`Rylogic.Audio.Authoring.App`** (ASP.NET Core minimal API): serves the browser UI and the HTTP API used by it, runs Freesound search and download,
  and publishes live revisions. It is an authoring tool only and is never a dependency of a shipped game.
- Not included: a plugin framework, external plugin hosting, AI restoration, or a full audio editor.

## Project configuration

The app opens a configuration file supplied by the game project. It names:

- the sound-ID catalog (IDs and the generic definition properties to show; the game owns their values and meaning),
- the originals, preparation and cache folders, and optionally a fetched folder for originals that can be downloaded again,
- Freesound settings (the credentials themselves stay in a per-user file outside the repository),
- the live-link target: the folder holding running-instance descriptors and the name of the tool that adopts a revision.

Rylogic never hard-codes a game's folders or sound IDs.

## Data model

- **Source**: an untouched original stored as lossless FLAC at `<originals>/<source-id>.flac`. The source ID comes from the content (SHA-256 of the
  original download bytes). A sidecar `<source-id>.json` records the provenance: origin kind (Freesound, generated, hand-made), origin reference, title,
  author, licence, URL and download SHA-256. One source can be used by many sound IDs. Originals that can be downloaded again (Freesound) may be kept
  in a fetched folder outside version control instead: the app and the cook download a missing one on demand and require it to have its recorded source
  ID, so the rendered audio cannot change silently. Generated and hand-made originals exist nowhere else and always stay in `<originals>`.
- **Preparation file**: one version-controlled `<prep>/<sound-id>.json` per sound ID with an ordered list of variants.
- **Variant**: a stable variant ID (never reused), origin and source ID, review status (unheard, candidate, accepted, rejected), note, repair flag, and
  recipe. The runtime variant order is the order of accepted variants. Recipes bind to variant IDs, never to rank or file names.
- **Recipe**: an ordered list of steps, each with parameters and a bypass flag.
- **Origins registry**: `<origins>` lists every known origin (a Freesound number, a generated file or a hand-made file) with its provenance, and maps an
  origin to its source ID once the original has been fetched. Origins that were only browsed have no source.
- **Source ID**: the first 16 hexadecimal digits of the SHA-256 of the original download.
- **Variant order**: the array order in the preparation file. Accepting a variant moves it after the last accepted variant and *move* reorders accepted
  variants; other status changes never reorder.
- **Reference file**: a variant may name an existing prepared file (`reference_master`, relative to the configuration folder). It is used instead of the
  recipe's render until the variant is *adopted*, which lets a project migrate existing hand-made results without changing them. Adopting clears the
  reference and never modifies or deletes the file.
- **Resolution**: `PreparedSounds.ResolveAsync` turns preparation files into each sound ID's ordered prepared files, for both a cooker and the live link.
  A preparation file for an unknown sound ID, an accepted variant with neither source nor reference, a missing reference file, or a missing or changed
  original is an error. A sound ID with no accepted variant is absent; the project decides what absence means.

## Recipe steps (first version)

| Step | Parameters |
|---|---|
| `trim` | start, end |
| `cut` | start, end (removes a region and joins the ends) |
| `fade_in`, `fade_out` | duration, curve |
| `gain` | dB |
| `loudness` | target integrated loudness; measured once, then applied as a static gain |
| `high_pass`, `low_pass` | frequency, slope |
| `eq` | frequency, gain, Q |
| `loop` | crossfade length; the tail is crossfaded into the head so the result loops seamlessly |

Compression and limiting would be separate, explicit steps in a later version; nothing normalizes sound automatically.

## Rendering and cache

- Rendering always starts from the original and applies the enabled steps in order. ffmpeg decodes and is used where a step needs it; ffmpeg is an
  authoring and cooking dependency only.
- Results are written as 48 kHz intermediates in the project's cache folder, keyed by the source content hash, the canonical recipe, the preparation
  version and the ffmpeg version. The cache is disposable.
- The same original and recipe always render to identical bytes. Platform encoding is done afterwards by the game's cooker.

## Jobs and responsiveness

- Downloads, imports and renders run as background jobs. State requests never wait for a job.
- Each sound ID's jobs carry a generation number; a superseded job's result is discarded and never replaces a newer result.
- The UI shows pending work and actionable errors. Preview playback keeps working during slow jobs.

## UI

- Keeps the existing audition features: Freesound search, multiple accepted variants, ordering, A/B comparison, loop-seam listening, notes and repair
  flags, rejection history and provenance. Changing a variant's status does not move it in the candidate list.
- A recipe editor per variant: add, remove, reorder and bypass steps, with parameter controls. The processed preview re-renders automatically after a short
  pause in editing.
- Loudness matching for audition is a clearly labelled playback-only option. The default playback is the actual prepared gain.
- **Create clips from original** in a variant's recipe editor collects multiple start/end ranges in seconds, with a destination sound ID per range.
  Listen and seek in the original, then use **Set start** / **Set end**, or type timestamps. Ranges may overlap and may target the same or different
  existing sound IDs. Each range creates a separate candidate with a stable variant ID, the same source and provenance, and its own initial `trim`
  recipe. The original, the source variant, its processing and any reference master stay unchanged. Clips can be processed and reviewed independently;
  accepting them uses the existing preview, live-update and cook paths. No extra original copies or new preparation format are needed.

## Live link (development only)

- A **Live link** toggle sends successful edits, and source or variant changes, to one chosen running game instance. Instances are listed from the
  descriptors in the configured folder.
- A **revision** is an immutable folder holding `manifest.json` and payloads. It is written to a temporary folder and published by renaming. The manifest
  has a revision number and, for each sound ID it changes, the complete definition and the ordered variant payloads with their hashes, so they always agree.
- The app then calls the game's configured adopt tool with the revision path. Rylogic does not define how the game adopts it; Petri adopts on its audio
  owner thread, crossfading loops and letting playing one-shots finish.
- A failed render or adoption leaves the last working audio in place and is reported in the app.

## Delivery

1. Library and app, migration from the Python audition tool (originals, preparation files, existing masters kept as references until confirmed), recipe
   editing with automatic preview, and the game cooker using the library. The migration was a one-time app command, removed after it ran; new
   generated sources record the user and licence "generated". Freesound originals are fetched on demand rather than committed.
2. Revision folders and the Live link, proven by replacing a looping ambience bed.
3. Live updates for variant-list changes and the remaining sound types.

## Tests

- Repeatable, identical rendering from the same original and recipe; originals never modified.
- Recipes stay bound to variant IDs through reordering and status changes.
- Each step's effect, including a seamless loop seam.
- Cache keys and cache hits.
- State requests stay responsive during slow jobs; superseded results are discarded.
- Revisions appear atomically; incomplete or inconsistent revisions are rejected.
