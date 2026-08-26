# View3DUI Implementation Plan

**Status:** Reviewed and approved; implementation not started  
**Scope:** Project-agnostic retained UI for native View3D applications, followed by a WPF-free managed API in `Rylogic.Gfx`  
**Implementation state:** No View3DUI source code exists yet

## 1. Objective

Add View3DUI as a dynamically loaded native satellite for View3D-12. View3DUI supplies retained control behaviour, deterministic layout, input, semantics, and rendering while applications retain ownership of content, composition, styles, lookless templates, and durable application values.

The first production-shaped vertical slice proves the architecture in `view3d-12-test` with a screen-space panel containing:

- static instructional text;
- a number-only text box;
- an Update button;
- normal, hover, pressed, focused, disabled, incomplete-value, and invalid-value presentation;
- mouse and keyboard activation;
- focus transfer between UI and the existing View3D camera controls;
- an update of the existing `m_obj0` box to uniform dimensions from an accepted value such as `1.23`.

The mock-up is directional only. Existing View3D conventions and the declarative template/style model determine the implementation.

## 2. Repository Findings That Shape the Plan

### 2.1 Frame and render ordering

The current View3D-12 frame is defined by `include/pr/view3d-12/main/frame.h` and submitted by `projects/rylogic/view3d-12/src/main/window.cpp` in this order:

1. `Frame::m_prepare`
2. `Frame::m_main`
3. `Frame::m_resolve`
4. `Frame::m_post`
5. `Frame::m_present`

`Window::NewFrame()` records MSAA resolve/copy and `AlphaKBuffer::CopyOpaqueBuffer()` into `m_resolve`, then records `AlphaKBuffer::ResolveAlpha()` and the PRESENT transition into `m_present`. `RenderForward::Execute()` currently appends `ESortGroup::PostAlpha` scene overlays to the same `m_present` list, including transitions back from PRESENT to RENDER_TARGET. `RenderRayTracing::Execute()` is a second writer: reflection/caustic work uses `m_post`, while fullscreen ray-tracing diagnostic/shadow output appends to `m_present`.

This means:

- `m_post` is before transparent-scene alpha resolution and cannot host final UI;
- `m_present` is currently both a composite phase and a final-state phase;
- a true final UI stage needs a separate View3D-owned command list after alpha resolution, post-alpha scene overlays, and fullscreen ray-tracing diagnostic output, but before the one authoritative transition to PRESENT;
- every current and future back-buffer writer must be assigned to an explicit phase; moving only `RenderForward` is insufficient.

### 2.2 Colour, alpha, and depth resources

- `AlphaKBuffer::m_opaque_colour_1x` is the resolved opaque base.
- `m_alpha_colour`, `m_alpha_depth`, and `m_alpha_rt_attrs` retain sorted transparent layers.
- `BackBuffer::m_depth_stencil` is the MSAA scene depth resource and `BackBuffer::m_depth_srv` is its shader view.
- `BackBuffer::m_render_target` and `m_rtv` identify the final single-sample target.
- `RenderForward` uses the scene depth during alpha collection, resolves alpha into `bb_post`, and draws `PostAlpha` objects without depth.

Screen-space UI therefore does not need depth in the first slice. Future depth-tested and occlusion-faded world roots need explicit scene-stage hooks and read-only depth access supplied by View3D; they must not sample or transition depth through an ad hoc final-overlay API. `DepthTested` roots must record in a scene-adjacent stage rather than the final overlay. `OcclusionFaded` roots additionally require a new deterministic single-sample depth-resolve resource because the current depth SRV is the MSAA scene depth and no post-resolve depth resource exists.

### 2.3 D3D11-on-12, Direct2D, DirectWrite, and existing text

`Renderer::RdrState` already owns:

- `ID3D11On12Device`;
- `ID3D11DeviceContext`;
- `ID2D1Factory2`;
- `ID2D1Device`.

`D2D1Context` in `include/pr/view3d-12/texture/d2d_context.h` wraps a shared, simultaneous-access D3D12 texture and flushes the D3D11 context on destruction. `ModelGenerator::Text()` creates one off-screen BGRA texture per formatted text model, uses DirectWrite for layout, draws with Direct2D, and presents it on a View3D quad.

View3DUI should reuse the DirectWrite concepts, not direct D2D drawing to the swap chain:

- direct D2D drawing would require GDI-compatible swap-chain constraints that conflict with the test application's 8x MSAA setup;
- `D2D1Context` flushes D3D11-on-12 work to the graphics queue, which conflicts with the requirement that View3D own queue submission and command-list order;
- UI text changes frequently enough that one full texture/model per text change is not an appropriate retained cache boundary.

The proposed first renderer uses DirectWrite shaping/metrics plus CPU glyph alpha extraction (for example through `IDWriteGlyphRunAnalysis`) and a satellite-owned D3D12 glyph cache. Uploads are recorded through the View3D-owned prepare command list; UI quads are recorded through the View3D-owned final-overlay command list. Direct2D remains available for later evaluation, but it is not the first-slice queue integration.

### 2.4 Dynamic satellite patterns

The existing patterns are:

- `include/pr/geometry/fbx.h` and `gltf.h`: dependency-light client facades that load `fbx.dll`/`gltf.dll`, resolve a closed function table, and hide third-party types;
- `include/pr/view3d-12/imgui/imgui.h`: a client facade that loads `imgui.dll`, exposes an opaque context, forwards Win32 input, and lets the satellite record into an application-provided D3D12 command list;
- `projects/rylogic/{fbx,gltf,imgui}`: standalone dynamic-library projects;
- `build/targets/{fbx,gltf,imgui}.targets`: native dependency copy rules.

View3DUI follows the dynamic loading and opaque C ABI pattern, but not ImGui's immediate-mode API or unrestricted application-supplied command-list integration. View3DUI is attached through a narrow, versioned View3D host bridge that View3D invokes at defined render stages.

### 2.5 Native and managed ownership patterns

The current View3D DLL exposes raw opaque handles and reports errors through callbacks. The newer audio and physics subsystems provide the stronger model to follow:

- stable status values;
- API-version and native-structure-size discovery;
- `{size, version}` headers on extensible records;
- generation-aware handles;
- thread-local last-error text;
- explicit owner-thread checks;
- bounded event queues;
- dependency-ordered managed disposal;
- SafeHandle abandonment only as a finalizer fallback.

Physics is the direct device-sharing precedent: `Physics_EngineCreate()` accepts a borrowed external D3D12 device, the engine retains its own COM reference, and `Physics_EngineDeviceLeaseAcquire()` exposes an independently owned lease. View3D already provides `View3D_DeviceLeaseAcquire()`, and `Rylogic.D3D12.DeviceLease` is the managed lifetime-safe representation.

`Rylogic.Gfx` is already separate from `Rylogic.Gfx.WPF`; the View3DUI wrapper can therefore live in the existing assembly under `Rylogic.Gfx.UI` without acquiring a WPF dependency. `View3d.Window.Handle` and `Hwnd` already expose the native View3D window and HWND needed by the satellite without adding a managed rendering callback.

### 2.6 Existing input and test-application surfaces

`projects/tests/view3d-12-test/src/main.cpp` derives `Main` from `gui::Form` and currently handles:

- resize in `OnWindowPosChange`;
- scene navigation in `OnMouseButton`, `OnMouseMove`, and `OnMouseWheel`;
- key commands in `OnKey`;
- rendering in `Step`;
- window messages through the overridable `ProcessWindowMessage` path in `wingui.h`.

`WinGuiMsgLoop` calls message filters before Win32 `TranslateMessage`/`DispatchMessageW`. The production View3DUI path must receive raw HWND messages at `Main::ProcessWindowMessage` before `Form` converts them into higher-level mouse/key events. Filtering at the message-loop stage would prevent `TranslateMessage` from generating `WM_CHAR`. This preserves access to `WM_CHAR`, dead-key, cursor, capture, focus, clipboard, and IME semantics and allows consumed messages to bypass camera handlers. The current form has dialog behaviour disabled; enabling `IsDialogMessage` or accelerators later would need an earlier UI-aware message-filter rule for Tab/Enter/Space.

The test window is per-monitor-DPI-aware V2. `GetClientRect` therefore already returns physical client pixels, but the current `OnWindowPosChange` multiplies those dimensions by `dpi / 96` and uses outer-window dimensions for viewport screen size. The demonstration must correct this existing host mapping before using it as DPI evidence.

The exact demonstration object is:

- member: `Main::m_obj0`;
- creation: `View3D_ObjectCreateLdrA("*Box nice_box FF00FF00 { *Data {1.23 1.23 1.23} }", ...)`;
- scene attachment: `View3D_WindowAddObject(m_win3d, m_obj0)`;
- update surface: `View3D_ObjectUpdate(m_obj0, L"*Box nice_box FF00FF00 { *Data {d d d} }", view3d::EUpdateObject::Model)`;
- render loop: `Main::Step()` rotates the box about world Z at the origin, derives one directional audio emitter from the same transform at the local +X face,
  then calls `View3D_WindowRender(m_win3d)`.

`EUpdateObject::Model` swaps the model/material/PSO state but not object identity or transforms. `Context::UpdateObject` temporarily detaches the object while
mutating renderer resources, then restores exactly its prior window memberships. Keeping the same `m_obj0` therefore preserves scene attachment while its
per-frame transform remains a world-Z rotation at the origin. The audio emitter position and forward direction are transformed from the box's local +X face,
so resizing changes the emitter radius without introducing a second motion model.

## 3. Settled Product Contract

### 3.1 Retained and closed

- View3DUI retains a validated tree, resource table, layout result, semantic tree, interaction state, and render caches.
- Applications cannot add control types, subclass controls, install arbitrary drawing, register managed per-frame draw callbacks, or submit custom shaders.
- New demonstrated control behaviour is implemented in View3DUI and added to the versioned descriptor schema.
- Applications own content, control composition, resource definitions, styles, templates, and durable values.
- View3DUI owns standard interaction behaviour and transient state.

### 3.2 Identity and events

- `ControlId` is a non-zero stable 64-bit value selected by the application.
- Managed descriptors are copyable values; managed object identity is never observed by native code.
- Handlers are not stored on UI elements.
- Applications drain ordered typed events and dispatch centrally by `ControlId`.
- Every event carries the source control ID, event kind, accepted tree revision, monotonic event sequence, and a tagged payload.
- The native DLL never invokes managed application code during mutation, layout, update, or rendering.

### 3.3 Authoritative and transient state

Application-owned durable state includes:

- committed text;
- toggle and slider values;
- selection;
- visibility;
- enabled state;
- validation state and message;
- accepted command effects.

Native transient state includes:

- hover and pressed state;
- pointer capture;
- keyboard focus;
- caret and selection;
- IME composition;
- scroll offsets and inertial state;
- layout;
- glyph/geometry caches;
- bounded visual transitions;
- pending proposals awaiting reconciliation.

Interaction events propose text/value changes or invoke commands. The application accepts, rejects, or normalizes them in a later transaction revision.

## 4. Proposed Architecture

### 4.1 Projects and dependency direction

```text
Native View3D application
    |
    +-- include/pr/view3d-ui/view3d-ui.h          (RAII facade and eager DLL loader)
    |       |
    |       +-- view3d-ui.dll                    (retained UI, layout, input, semantics, rendering)
    |               |
    |               +-- owned ID3D12Device COM lease
    |               +-- private versioned View3D UI host bridge
    |                       |
    |                       +-- view3d-12.dll    (targets, barriers, command lists, queue, present)
    |
    +-- view3d-12.dll

Managed application (later phase)
    |
    +-- Rylogic.Gfx.UI namespace in Rylogic.Gfx  (typed value descriptors and ownership)
            |
            +-- View3d.Window and native window handle
            +-- Rylogic.D3D12 device lease
            +-- view3d-ui.dll via Rylogic.Native
```

View3DUI does not link against the View3D static library and does not take ownership of View3D targets or command infrastructure. At context creation, the caller supplies a borrowed `ID3D12Device*` obtained from `View3D_DeviceLeaseAcquire()`/`Rylogic.D3D12.DeviceLease`; the satellite immediately takes its own COM reference, following the physics external-device precedent. It also resolves a private host-bridge export from the already loaded `view3d-12.dll`, verifies the bridge ABI, and attaches exactly one UI context to a View3D window for the initial release.

### 4.2 Public native layers

1. **Wire types:** dependency-minimal, fixed-layout enums and records.
2. **C ABI:** versioned status-returning exports with opaque handles and caller-owned buffers.
3. **Native facade:** move-only RAII `UiRuntime` and `UiContext` wrappers that load the complete DLL function table eagerly, preserve the owner thread, and translate non-success status values into exceptions containing the DLL's last-error text.

The facade does not expose the private render-provider table or a generic drawing callback.

### 4.3 Private View3D host bridge

The bridge is a dedicated View3DUI integration contract, not a general plug-in render API. It provides:

- ABI version and structure-size discovery;
- attach/detach against a `pr::view3d::Window`;
- a View3D-owned prepare command list for pending UI resource uploads;
- a View3D-owned final-overlay command list and final single-sample RTV;
- render-target size, format, frame index, DPI, and viewport state;
- future typed world-stage pass records with the exact colour/depth resources permitted at each policy stage.

The provider may set its own pipeline state, root signature, viewport/scissor, vertex/index buffers, and descriptor heaps. It may not:

- transition host resources;
- close or execute command lists;
- signal fences;
- submit to a queue;
- present;
- retain command-list or host-resource pointers beyond the call.

View3D brackets the provider call with authoritative resource transitions and submits all work.

The provider must not re-enter public `View3D_*` exports while View3D is invoking it. The bridge passes all required state directly, so the integration does not depend on View3D's current recursive DLL mutex remaining recursive.

### 4.4 Frame split

Refactor the current combined present list into explicit phases:

1. `prepare`
2. scene `main`
3. opaque/MSAA `resolve`
4. scene `post`
5. `composite` — K-buffer alpha resolve, existing `PostAlpha` scene overlays, and current fullscreen ray-tracing diagnostic/shadow output in preserved scene-step order
6. `final_overlay` — screen-space View3DUI for the first slice
7. `present` — the single final transition to `D3D12_RESOURCE_STATE_PRESENT`

`Window::NewFrame()` resets all lists and records alpha resolve into the still-open composite list. `Scene::Render(frame)` appends `PostAlpha` and ray-tracing diagnostic work to composite. `V3dWindow::Render()` then invokes the attached View3DUI provider to record final-overlay work before `Window::Present(frame)` closes every phase, records/closes the final PRESENT transition, and submits the frame. No command may write the back buffer after the final PRESENT transition.

If `bb_post` is null, the final-overlay stage is unavailable and the provider is not invoked. An empty viewport short-circuits rendering as it does today; applications still call `View3DUI_Update` independently so time, reconciliation, and event drain are not coupled to whether a frame can be rendered.

Future world policies use additional typed hook points without changing the element model:

- `DepthTested`: scene-adjacent raster stage with scene depth test/write policy controlled by View3D;
- `OcclusionFaded`: post-resolve world-overlay stage with a new View3D-owned single-sample depth resolve and deterministic fade parameters;
- `Overlay`: post-alpha world-overlay stage before the screen-space final overlay, without depth testing.

Only the screen-space final-overlay path is enabled in the first vertical slice. World-root descriptors are rejected with `UnsupportedFeature` until their milestone is complete.

## 5. Wire Model and C ABI

### 5.1 ABI rules

- Win64 only for the first release.
- `extern "C"` and `__stdcall`.
- fixed-width integer fields and `int32_t` enum storage;
- no STL, exceptions, references, `bool`, C++ classes, or ownership-bearing COM pointers in public records;
- every extensible record begins with `{uint32_t size; uint32_t version;}`;
- all input arrays use pointer plus 32-bit count and are copied before the call returns;
- all returned variable data uses caller-owned buffers with required-capacity reporting;
- string payloads are UTF-8 byte slices on the ABI; raw Win32 text remains UTF-16 internally until normalized;
- `View3DUI_ApiVersion()` and `View3DUI_StructSize(EStructId, uint32_t*)` support fail-fast native/managed layout checks;
- opaque UI contexts use generation-aware 64-bit handles;
- status and last-error retrieval follow the audio ABI pattern.

### 5.2 Principal exports

Names are provisional but the responsibilities are fixed:

| Surface | Responsibility |
| --- | --- |
| `View3DUI_ApiVersion`, `View3DUI_StructSize(EStructId, ...)`, `View3DUI_LastError` | ABI/schema diagnosis |
| `View3DUI_Initialise`, `View3DUI_Shutdown`, `View3DUI_ContextAbandon` | Process-level module lifetime |
| `View3DUI_ContextCreate`, `View3DUI_ContextDestroy` | Attach/detach one owner-thread UI context to a View3D window |
| `View3DUI_TransactionApply` | Copy, validate, and atomically accept one complete revision transaction |
| `View3DUI_ProcessWindowMessage` | Process raw HWND messages and report consumed/result |
| `View3DUI_InputInject` | Inject normalized deterministic input through the same control-state machine |
| `View3DUI_Update` | Supply viewport state and explicit time, reconcile pending proposals, run layout/transitions, and prepare an immutable render snapshot |
| `View3DUI_EventCount`, `View3DUI_EventsCopy` | Inspect and drain typed events |
| `View3DUI_SemanticsCopy` | Copy a flat semantic snapshot plus UTF-8 text storage |
| `View3DUI_DiagnosticsGet` | Return bounded counts, revisions, overflow totals, cache use, and last failure category |

No public render export accepts an application command list.

`EStructId` covers at least Config, Transaction, Operation, Control, Resource, Style, Template, NormalizedInput, ViewportState, Event, SemanticNode, Diagnostics, HostBridge, and HostPass. Native and managed startup tests enumerate the complete set rather than checking a hand-selected subset.

### 5.3 Transaction format

One delta `Transaction` contains only changed records plus:

- header, `base_revision`, and `revision`;
- ordered operation records;
- control descriptors;
- child-order arrays;
- style descriptors;
- template descriptors;
- font/image/resource descriptors;
- a shared UTF-8/blob table referenced by offset and length.

Validation applies the delta to a staging clone/reference model of the last accepted revision. The transaction is one bulk interop call even when a text proposal changes TextBox text, validation state, and Button enabled state together; it does not resend the full tree and does not make per-property calls.

The operation vocabulary is closed:

- `Upsert`;
- `Remove`;
- `Reorder`;
- `ReplaceSubtree`.

Validation occurs against a staging model and checks:

- `base_revision == last_accepted_revision`;
- `revision == base_revision + 1`;
- non-zero and unique IDs;
- valid parents and exactly one root per root descriptor;
- no parent cycle;
- valid child order with no duplicates or missing members;
- known control, layout, visual, state, transition, and resource types;
- required template parts for each interactive control;
- valid references and blob ranges;
- finite dimensions, durations, transforms, and numeric values;
- bounded node, operation, string/blob, depth, and resource counts;
- supported root/render policies for the current implementation level.

Any failure rejects the complete transaction and preserves the last accepted tree, layout, semantics, resources, and render snapshot. A failed transaction does not advance the revision.

Contiguous revisions are proposed because they detect dropped managed updates and make event/reconciliation traces unambiguous.

### 5.4 Events

The initial typed event set is:

- `FocusChanged`;
- `TextChangeProposed`;
- `CommandInvoked`;
- `PointerCaptureChanged`;
- `QueueOverflow`;
- `Diagnostic`.

Each event contains:

- structure header;
- `ControlId`;
- event kind;
- accepted tree revision that produced it;
- monotonic sequence;
- tagged payload metadata;
- payload offset/length into the event-copy blob.

The context owns a fixed-capacity queue configured at creation. Events are classified:

- coalescible state observations such as focus/capture/diagnostic updates retain the latest value deterministically;
- `TextChangeProposed` retains the latest pending proposal per control because it contains the complete proposed text/edit generation;
- `CommandInvoked` is non-coalescible and is never silently dropped.

One slot is reserved for a coalesced overflow marker. If coalescing cannot make room for a non-coalescible event, the triggering input operation returns `QueueOverflow`, does not commit the activation, and puts input into an explicit overflow state until the application drains events and submits a reconciliation revision. No command is reported as successful and then lost.

### 5.5 Semantic snapshot

`SemanticsCopy` returns a deterministic pre-order flat array containing:

- ID and parent ID;
- role;
- accessible name and description;
- value text;
- enabled, visible, focused, focusable, selected/checked, invalid, and off-screen state;
- supported actions;
- computed screen-space DIP bounds;
- accepted revision and semantic snapshot sequence.

This is the single source for native tests and a later UI Automation provider. UI Automation must not require a parallel element model.

## 6. Element, Layout, Template, and Style Model

### 6.1 Closed control types

The first vertical slice implements:

- `Root`;
- `Panel`;
- `Text`;
- `TextBox`;
- `Button`.

Later demonstrated needs can add controls to the schema. Applications cannot register control classes.

### 6.2 Layout vocabulary

The complete initial vocabulary is:

- `Stack` with horizontal or vertical orientation;
- `Overlay`;
- `Scroll`;
- absolute `Canvas`.

The demonstration slice implements `Stack` and `Overlay`, the only two it exercises. `Scroll` and `Canvas` are completed before declaring the initial layout vocabulary generally available. No Grid, dock, flex, constraint, or custom layout is included.

All public dimensions, spacing, padding, borders, radii, font sizes, and screen bounds are DIPs. Physical pixels are derived from the explicit viewport DPI supplied to `Update`.

### 6.3 Lookless templates

Applications define templates from a closed visual vocabulary:

- solid/rounded box;
- border;
- text presenter;
- content presenter;
- clip;
- transform/opacity group;
- required named control parts.

The first slice requires built-in contracts for:

- Button content and focus outline parts;
- TextBox text, selection, caret, validation, and focus outline parts.

Templates are lookless in the sense that control behaviour targets named semantic parts and state channels, not hard-coded colours or geometry. Missing or duplicate required parts reject the transaction.

Applications can compose these primitives but cannot submit code, shader names, arbitrary geometry, or draw callbacks.

### 6.4 Styles and transitions

Style rules target declared state channels:

- normal;
- hover;
- pressed;
- focused;
- selected/checked;
- disabled;
- validation;
- visibility;
- value change.

Transitions interpolate a closed set of visual properties such as colour, opacity, border thickness, and bounded transform offsets/scales. Each transition has validated duration/easing and a fixed upper duration. Native transitions:

- are driven only by host-supplied time;
- cannot alter durable values;
- cannot issue commands;
- cannot create unbounded animation;
- resolve ties in descriptor order for deterministic output.

The first slice implements the channels used by TextBox and Button. The remaining listed channels stay in the schema and are completed in the style-completion milestone.

## 7. Input, Focus, Text, and Validation

### 7.1 Production message path

`View3DUI_ProcessWindowMessage` receives the original `HWND`, message, `WPARAM`, and `LPARAM` before `gui::Form` handling. It returns:

- whether the UI consumed the message;
- the `LRESULT` when consumed;
- whether the window should invalidate.

The native platform adapter handles:

- pointer position/buttons/wheel;
- `SetCapture`/`ReleaseCapture`;
- cursor selection;
- `WM_SETFOCUS`/`WM_KILLFOCUS`;
- key down/up and system keys;
- `WM_CHAR`/UTF-16 surrogate handling;
- dead-character messages;
- clipboard commands;
- IME start/composition/end messages.

The first slice implements mouse, focus, Tab/Shift+Tab, Enter/Space activation, editing keys, clipboard paste/copy/cut for plain text, and UTF-16 character input needed by the text box. Full IME candidate/composition presentation and complete international text editing are later, but the raw-message boundary and transient composition fields are present from the start.

### 7.2 Normalized injection

Raw messages translate into internal normalized records and then enter the same control-state machine as `View3DUI_InputInject`. Test injection supplies explicit:

- pointer position and button transitions;
- wheel delta;
- virtual key and modifier transitions;
- text/composition units;
- focus gained/lost;
- timestamps or the sequence position relative to explicit `Update` time.

Injection does not call Win32 capture, clipboard, or IME services; deterministic test doubles provide those platform results.

### 7.3 Routing and camera ownership

- UI hit testing and active pointer capture run before scene/camera handling.
- A consumed message does not reach `Main::OnMouse*` or `Main::OnKey`.
- Mouse down on no UI element clears UI keyboard focus and is returned as unconsumed, allowing the same message to begin camera interaction.
- Active UI pointer capture continues receiving pointer messages outside UI bounds until release/cancel.
- Losing HWND focus clears UI focus/capture and emits ordered focus/capture events.
- Tab, Enter, and Space are consumed only while UI focus makes them applicable; existing host shortcuts remain unchanged when UI has no focus.

### 7.4 Coordinate conversion

The host supplies all coordinate spaces explicitly:

- raw pointer coordinates are physical client pixels;
- `ViewportState` contains client size in pixels, render-target size in pixels, viewport rectangle in render-target pixels, and DPI;
- client pixels map to render-target pixels using the explicit size ratio, then render-target pixels map to DIPs with `96 / dpi`;
- the normal HWND swap-chain path uses equal client/render-target dimensions, so the ratio is one;
- custom/off-screen targets may use a non-one ratio but use the same formula.

For `view3d-12-test`, `OnWindowPosChange` must use `GetClientRect` physical width/height directly for the back buffer and viewport screen dimensions. It must remove the current extra DPI multiplication and must not use outer-window `WINDOWPOS` dimensions as client size.

### 7.5 Keyboard focus

- Focus order is deterministic document order after filtering visible, enabled, focusable controls.
- Tab advances; Shift+Tab reverses and wraps within the root.
- TextBox and Button are focusable in the demo.
- Button activates on mouse press/release inside, Space release while focused, and Enter while focused.
- Disabled or invisible controls cannot receive focus or activate.
- A revision that removes, hides, or disables the focused control moves focus to the next eligible control when possible, otherwise clears it.

### 7.6 Proposed numeric semantics for the demo

The test application, not View3DUI, owns numeric interpretation and durable validation state.

Proposed first-demo grammar:

- invariant decimal text using `.` as the decimal separator;
- editing permits incomplete forms `""`, `"."`, `"+"`, `"-"`, `"1."`, `"+1."`, and `"-1."`;
- an accepted Update value must parse to a finite number greater than zero;
- incomplete input presents a neutral/pending validation state and disables Update;
- syntactically invalid, non-finite, zero, or negative input presents an invalid state and disables Update;
- a valid value enables Update;
- Update preserves the entered text and changes all three box dimensions to the parsed value;
- no implicit clamping or locale conversion occurs.

The UI keeps a transient pending edit buffer so consecutive raw messages can compose before the application drains proposals. A later descriptor revision acknowledges, normalizes, or rejects the proposal and repositions the caret deterministically.

## 8. Ownership, Lifetime, and Threading

### 8.1 Lifetime order

1. Load/initialise View3D.
2. Eagerly load View3DUI and resolve its complete export table during process startup.
3. Create the View3D window and acquire a D3D12 device lease.
4. Create and attach a View3DUI context.
5. Apply transactions, input, update, drain events, query semantics, and render.
6. Destroy the View3DUI context, which detaches it from the View3D window and retires its GPU resources through View3D.
7. Release the context's owned D3D12 device COM reference and the caller's temporary/managed lease.
8. Shut down View3DUI after all UI contexts are gone.
9. Destroy the View3D window and shut down View3D.

The DLL remains loaded while any runtime/context exists. Runtime shutdown with live contexts returns `ResourceInUse`. `V3dWindow` host destruction invalidates an attached provider defensively, but correct clients must destroy UI first.

### 8.2 Owner thread

The OS thread that creates a View3DUI context owns:

- transaction application;
- raw and normalized input;
- update/layout;
- event drain;
- semantic snapshot;
- resource release;
- context destruction;
- render-provider invocation through `View3D_WindowRender`.

Every public operation checks the owner OS thread and returns `WrongThread`, including queries. Managed wrappers repeat the check before P/Invoke. No automatic `SynchronizationContext` dispatch is added.

### 8.3 GPU lifetime

- View3D owns command allocators, command lists, graphics queue, fences, buffering, targets, resource barriers, and presentation.
- View3DUI owns one AddRef'd `ID3D12Device` reference for each live context; it releases that reference only after its GPU resources are retired.
- View3DUI owns only its PSOs, root signatures, descriptor heaps, geometry buffers, glyph textures, and CPU caches.
- View3DUI schedules release against a View3D-provided frame/fence lifetime service; it never blocks the queue during normal mutation or rendering.
- Resize invalidates target-dependent state but preserves the accepted descriptor tree, interaction state where valid, and device-independent caches.
- Device removal is detected and reported as a deterministic fault in the first slice; transparent device recreation is deferred.

## 9. Diagnostics and Explicit Failures

### 9.1 Stable statuses

At minimum:

- `Success`;
- `InvalidArgument`;
- `InvalidStruct`;
- `AbiMismatch`;
- `SchemaMismatch`;
- `InvalidHandle`;
- `StaleHandle`;
- `WrongThread`;
- `StaleRevision`;
- `InvalidTree`;
- `UnknownType`;
- `UnknownResource`;
- `UnsupportedFeature`;
- `BufferTooSmall`;
- `QueueOverflow`;
- `ResourceLimit`;
- `MissingAsset`;
- `DeviceLost`;
- `ResourceInUse`;
- `InternalError`.

### 9.2 Bounded limits

Context configuration declares hard limits for:

- controls;
- roots;
- tree depth;
- operations per transaction;
- UTF-8/blob bytes;
- templates/styles/resources;
- glyph-cache bytes/pages;
- generated vertices/indices;
- queued events;
- semantic records;
- transition count and maximum duration.

Limits are validated before allocation where possible and surfaced in diagnostics. No failure silently substitutes an unknown control/resource, drops a tree revision, or reports success.

### 9.3 Determinism

Given identical:

- accepted descriptor/resource revisions;
- viewport and DPI;
- normalized input sequence;
- platform-service test results;
- explicit time;

the implementation produces identical:

- layout rectangles;
- focus/capture state;
- semantic snapshot order/content;
- event order and sequence values;
- transition samples;
- stable draw-item order.

GPU pixels may vary at rasterization edges across driver/font versions unless fonts are packaged. Structural render tests therefore assert draw packets/order and use attended pixel evidence for the demo.

## 10. Concrete Repository Changes

### 10.1 New native public and implementation files

| Likely path | Purpose |
| --- | --- |
| `include/pr/view3d-ui/forward.h` | Sole include boundary for foreign/STL/Win32/D3D/DirectWrite dependencies |
| `include/pr/view3d-ui/types.h` | Dependency-minimal public wire records and enums |
| `include/pr/view3d-ui/view3d-ui-dll.h` | Versioned C ABI declarations |
| `include/pr/view3d-ui/view3d-ui.h` | Eager dynamic loader and move-only native RAII facade |
| `include/pr/view3d-ui/...` | Internal sibling headers for transactions, tree, layout, input, controls, semantics, styles, text cache, renderer, and diagnostics |
| `projects/rylogic/view3d-ui/view3d-ui.vcxproj` | Static core library (`view3d-ui-static.lib`) |
| `projects/rylogic/view3d-ui/view3d-ui.dll.vcxproj` | Dynamic satellite and C ABI |
| `projects/rylogic/view3d-ui/src/...` | Native definitions, organised by the same concerns as the headers |
| `projects/rylogic/view3d-ui/*.vcxproj.filters` | Visual Studio filters |

All View3DUI headers except `forward.h` include only `forward.h` and same-module headers.

### 10.2 View3D host changes

| Existing/new path | Change |
| --- | --- |
| `include/pr/view3d-12/main/frame.h` | Split composite, final-overlay, and final-present command-list phases |
| `include/pr/view3d-12/main/window.h` | Own/reset/submit the new phase resources |
| `projects/rylogic/view3d-12/src/main/window.cpp` | Record alpha composite before final overlay and one final PRESENT transition |
| `projects/rylogic/view3d-12/src/render/render_forward.cpp` | Move `PostAlpha` draws to the composite list and stop transitioning to PRESENT |
| `projects/rylogic/view3d-12/src/ray_tracing/render_ray_tracing.cpp` and its diagnostic recorder | Move fullscreen diagnostic/shadow output to composite and preserve its order before View3DUI |
| `projects/rylogic/view3d-12/src/dll/v3d_window.h/.cpp` | Store one versioned UI provider attachment and invoke its typed stage |
| `projects/rylogic/view3d-12/src/dll/dll.cpp` | Implement private host-bridge attach/detach/version exports |
| `include/pr/view3d-12/view3d-ui-bridge.h` | Private, dependency-minimal host/provider ABI shared only by View3D and the satellite |
| `projects/rylogic/view3d-12/view3d-12*.vcxproj[.filters]` | Add bridge files |

The existing public `View3D_WindowRenderingCB` remains unchanged but is not used for View3DUI rendering.

### 10.3 Test projects and application

| Path | Change |
| --- | --- |
| `projects/tests/view3d-ui-tests/view3d-ui-tests.vcxproj[.filters]` | Focused native test runner for core logic, ABI, bridge, and deterministic render packets |
| `projects/tests/view3d-ui-tests/src/...` | Transaction, layout, input, focus, semantics, lifecycle, thread, DPI, queue, and render-order tests |
| `projects/tests/view3d-12-test/src/main.cpp` | Create/destroy UI in lifetime order, forward raw messages first, drain events, own numeric state, update `m_obj0`, and render |
| `projects/tests/view3d-12-test/view3d-12-test.vcxproj[.filters]` | Add the facade header/project reference and `view3d-ui.targets` copy rule |
| `Rylogic.sln` | Add View3DUI native, DLL, native-test, and later managed projects with dependencies/configurations |

### 10.4 Build, deployment, and package files

| Path | Change |
| --- | --- |
| `build/targets/view3d-ui.targets` | Copy the configuration-appropriate satellite beside native/managed consumers |
| `script/Build.csx` | Add the `View3dUI` native project; the existing `RylogicGfx` package entry carries the managed wrapper |
| `Directory.Packages.props` | Add only packages required by later managed JSON support, if any |
| `projects/rylogic/Rylogic.Gfx/src/View3d/UI/...` | Add `Rylogic.Gfx.UI` typed descriptors, builders, ownership, P/Invoke, JSON conversion, and tests |
| `projects/rylogic/Rylogic.Gfx/Readme.md` | Document managed ownership, revisions, event draining, threading, and JSON usage |
| `projects/rylogic/Rylogic.Gfx/Rylogic.Gfx.csproj` | Import `view3d-ui.targets`; retain existing WPF-free dependencies and target frameworks |

`Rylogic.Native` already packages all Release DLLs under `lib/x64/Release`, so deploying `view3d-ui.dll` there includes it in the native runtime package without a one-off package layout.

## 11. Milestones

### M0. Contract and ABI skeleton

**Depends on:** approved decisions 1-5 in section 14.

**Work**

- Add public wire types, stable status values, API/version/size discovery, last-error retrieval, runtime/context handles, and eager native facade loading.
- Define context creation around a borrowed external D3D12 device; the DLL takes its own COM reference and releases it after context GPU teardown.
- Add static layout assertions and a native ABI probe.
- Add empty View3DUI static/DLL/test projects, solution entries, filters, and Debug x64 build wiring.
- Define the private View3D host bridge version and provider records without implementing rendering.

**Exit criteria**

- Missing DLL, missing export, ABI mismatch, structure mismatch, double destroy, stale handle, and wrong-thread operations fail with the expected stable status/message.
- Every `EStructId` reports an exact native size and the external-device COM reference count is balanced across success and failed creation.
- No application callback or arbitrary render entry exists.
- Header compile probes prove the C ABI does not leak STL or View3D internal types.

**Validation**

```powershell
$msbuild = "C:\Program Files\Microsoft Visual Studio\18\Enterprise\MSBuild\Current\Bin\amd64\MSBuild.exe"
& $msbuild "projects\tests\view3d-ui-tests\view3d-ui-tests.vcxproj" /t:Rebuild /p:Configuration=Debug /p:Platform=x64 /nologo /verbosity:minimal
```

### M1. Atomic retained model, resources, semantics, and diagnostics

**Depends on:** M0.

**Work**

- Implement staging validation and atomic revision swap.
- Implement upsert/remove/reorder/replace-subtree.
- Implement control/resource limits, stable diagnostics, coalescible-event replacement, non-coalescible overflow faulting, and bounded event storage.
- Implement deterministic semantic snapshot from accepted descriptors before rendering.
- Add control types `Root`, `Panel`, `Text`, `TextBox`, and `Button`; initially no interaction.

**Exit criteria**

- Valid transactions advance exactly one revision.
- Invalid IDs, parentage, cycles, child order, references, types, blob ranges, capacities, and stale/non-contiguous revisions preserve the previous accepted snapshot byte-for-byte.
- Semantic pre-order is stable across repeated runs and independent of hash-table iteration.
- Queue overflow never silently loses an accepted command; coalescing, explicit rejection, overflow state, marker, and recovery revision are deterministic.

**Validation**

Build with the M0 command, then run the standalone console test executable:

```powershell
.\bin\x64\Debug\view3d-ui-tests.exe --filter Transaction
```

The project uses `PR_UNITTESTS=1` and the existing native unit-test framework, but remains a focused executable rather than adding GPU/DLL lifecycle tests to the broad harvested `projects\tests\unittests` target.

### M1.5. Render bridge and phase-order spike

**Depends on:** M0.

**Work**

- Split the existing composite/final-overlay/present phases before implementing the complete control renderer.
- Move `RenderForward` PostAlpha and `RenderRayTracing` fullscreen diagnostic/shadow writers to composite.
- Attach a minimal versioned provider that records one fixed opaque quad into the View3D-owned final-overlay list.
- Exercise 1x/8x MSAA, transparent-scene alpha, PostAlpha content, fullscreen ray-tracing diagnostics, null `bb_post`, empty viewport, and shutdown under the D3D12 debug layer with GPU-based validation.

**Exit criteria**

- Submission order is instrumented as `prepare → main → resolve → post → composite → final_overlay → present`.
- Exactly one final transition places the back buffer in PRESENT, and a debug assertion rejects any later back-buffer writer.
- The fixed quad is above alpha, PostAlpha, and ray-tracing diagnostic output.
- The provider records only into host command lists and cannot submit, transition, close, fence, or present.
- Attach/detach and leased-device teardown are debug-layer clean.

**Validation**

```powershell
$msbuild = "C:\Program Files\Microsoft Visual Studio\18\Enterprise\MSBuild\Current\Bin\amd64\MSBuild.exe"
& $msbuild "projects\rylogic\view3d-12\view3d-12.dll.vcxproj" /t:Rebuild /p:Configuration=Debug /p:Platform=x64 /nologo /verbosity:minimal
& $msbuild "projects\tests\view3d-ui-tests\view3d-ui-tests.vcxproj" /t:Rebuild /p:Configuration=Debug /p:Platform=x64 /nologo /verbosity:minimal
.\bin\x64\Debug\view3d-ui-tests.exe --filter RenderBridge
```

### M2. Layout, templates, styles, input, focus, and events

**Depends on:** M1 and M1.5.

**Work**

- Implement DIP-based `Overlay` and horizontal/vertical `Stack`.
- Implement required template parts and the first visual primitives.
- Implement state-based styles and bounded transitions for hover, pressed, focus, disabled, and validation.
- Implement raw message translation and normalized injection through one state machine.
- Implement hit testing, capture, Tab/Shift+Tab, Button mouse/Enter/Space behaviour, TextBox editing, clipboard basics, and proposal events.
- Implement explicit-time `Update` and immutable render snapshots without GPU recording.

**Exit criteria**

- Raw and injected equivalents produce identical focus, events, semantics, layout, and visual state.
- A mouse down outside UI clears focus and remains unconsumed.
- Captured pointer release outside behaves correctly.
- Focus recovery after removal/hide/disable is deterministic.
- Text proposals reconcile correctly against accepted revisions, including multiple queued characters.
- A test drives every implemented transition channel, verifies the accepted descriptor hash is unchanged, and verifies transition evaluation adds no command/proposal event. A Debug assertion enforces that the transition evaluator writes only visual snapshot fields.

**Validation**

```powershell
.\bin\x64\Debug\view3d-ui-tests.exe --filter "Layout|Input|Focus|Transition|Semantics"
```

Repeat deterministic scenarios with the same inputs/time and compare serialized snapshots.

### M3. Retained screen-space renderer

**Depends on:** M1.5 and M2.

**Work**

- Replace the spike quad with retained visual draw packets while preserving the proven bridge and phase order.
- Implement satellite PSO/root signature/buffers/descriptors and stable visual draw ordering.
- Implement DirectWrite shaping/metrics, CPU glyph coverage extraction, bounded glyph cache, and prepare-list uploads.
- Handle resize, DPI, minimized/zero-size targets, deterministic device-removal fault detection, and deferred resource release. Transparent device recovery is not part of this milestone.

**Exit criteria**

- Instrumented frame order remains `... resolve → post → alpha resolve/PostAlpha/ray-tracing diagnostic → View3DUI → PRESENT`.
- UI remains visible over transparent scene geometry, existing `PostAlpha` scene objects, and fullscreen ray-tracing diagnostics.
- The provider cannot execute/close the command list or transition/present the target.
- 1x and 8x MSAA paths render UI through the same final single-sample target.
- Resize and DPI changes recompute physical placement from unchanged DIP descriptors.
- D3D12 debug-layer run has no state, lifetime, descriptor, or live-object errors attributable to View3DUI.

**Validation**

```powershell
$msbuild = "C:\Program Files\Microsoft Visual Studio\18\Enterprise\MSBuild\Current\Bin\amd64\MSBuild.exe"
& $msbuild "projects\rylogic\view3d-12\view3d-12.dll.vcxproj" /t:Rebuild /p:Configuration=Debug /p:Platform=x64 /nologo /verbosity:minimal
& $msbuild "projects\tests\view3d-ui-tests\view3d-ui-tests.vcxproj" /t:Rebuild /p:Configuration=Debug /p:Platform=x64 /nologo /verbosity:minimal
.\bin\x64\Debug\view3d-ui-tests.exe --filter "Renderer|Text|Resize|Dpi|Lifecycle"
```

### M4. Native `view3d-12-test` demonstration

**Depends on:** M3.

**Work**

- Load `view3d-ui.dll` and resolve the complete facade function table in `WinMain` beside the existing startup loads, before constructing `Main`; create/attach the UI context after `m_win3d` exists.
- Apply one initial revision containing the screen root, semi-transparent panel, text, numeric TextBox, Update button, resources, template, and styles.
- Override `Main::ProcessWindowMessage` to offer raw messages to UI before `Form`.
- Add a separate monotonically advancing UI clock accumulated from every `Step(dt)` regardless of `EStepMode`; retain `m_time` as the independently pausable scene/simulation clock.
- Correct `OnWindowPosChange` so the per-monitor-aware physical client size is used directly for back-buffer and viewport dimensions.
- Drain proposals/commands in `Step`, update application-owned text/validation/enabled descriptors in the next revision, then call `View3DUI_Update` with UI time and the explicit client/render-target/viewport/DPI mapping before `View3D_WindowRender`.
- On valid Update activation, call `View3D_ObjectUpdate` on the existing `m_obj0` with
  `L"*Box nice_box FF00FF00 { *Data {d d d} }"` and `EUpdateObject::Model`.
- Destroy UI before `m_win3d`.

**Exit criteria**

- Tab and Shift+Tab move focus between TextBox and Button.
- `1.23` can be entered and accepted.
- Update activates by mouse, Enter, and Space when the Button is focused.
- `m_obj0` becomes a `1.23 × 1.23 × 1.23` box without replacing the View3D object handle or losing its current transform.
- Empty/incomplete text is distinguishable from invalid text and both disable Update.
- Clicking a non-UI part of the client clears UI focus and the same click reaches camera navigation.
- Hover, press, release, focus, disabled, and validation visuals are observable.
- Resize and a non-100% DPI attended run preserve DIP sizing and hit-test alignment.
- UI transitions animate while the existing scene step mode remains `Single`.
- With UI focus cleared, the existing Space single-step shortcut and other host keys still behave as before.

**Validation**

```powershell
$msbuild = "C:\Program Files\Microsoft Visual Studio\18\Enterprise\MSBuild\Current\Bin\amd64\MSBuild.exe"
& $msbuild "projects\tests\view3d-12-test\view3d-12-test.vcxproj" /t:Rebuild /p:Configuration=Debug /p:Platform=x64 /nologo /verbosity:minimal
```

Attended evidence records:

- one screenshot with valid `1.23` and enabled Update;
- one screenshot with incomplete or invalid input and validation presentation;
- a short checklist confirming mouse activation, keyboard activation, host shortcuts without UI focus, focus return to camera, animation while scene time is paused, resize, and DPI behaviour;
- a debug-layer-clean shutdown.

### M5. Native hardening, deployment, and initial-layout completion

**Depends on:** M4.

**Work**

- Implement `Scroll` and absolute `Canvas`.
- Complete bounded style channels reserved by the initial schema.
- Add stress tests for transaction limits, queue overflow, resize churn, repeated context/window lifecycles, and missing resources.
- Add `view3d-ui.targets`, `Build.csx` native entries, deploy rules, and package-content tests.
- Confirm runtime loads the app-local configuration-correct DLL rather than an older PATH copy.

**Exit criteria**

- The complete initial layout vocabulary is supported and no unapproved layout type is present.
- Repeated create/update/render/destroy cycles leave no live provider attachment or GPU object.
- Debug x64 deployment places `view3d-ui.dll` in `lib\x64\Debug` and the native dependency target copies it to `$(OutDir)lib\x64\Debug`, matching the existing `win32::LoadDll` search convention.
- Release deployment places it in `lib\x64\Release`; `Rylogic.Native` package inspection finds it under `runtimes/win-x64/native`.

**Validation**

```powershell
dotnet-script .\script\Build.csx -project View3dUI -platform x64 -config Debug -build -deploy -norestore -notests -nopack
dotnet-script .\script\Build.csx -project AllNative -platform x64 -config Release -build -deploy
```

Inspect the produced native package rather than publishing it.

### M6. WPF-free managed API in `Rylogic.Gfx`

**Depends on:** M5.

**Work**

- Add the `Rylogic.Gfx.UI` namespace to the existing `Rylogic.Gfx` assembly and its current target frameworks.
- Implement copyable typed descriptors, transaction builders, resources, event payloads, semantic records, diagnostics, and owner-thread runtime/context wrappers under that namespace.
- Validate API version and every ABI structure size at startup.
- Hold a strong `View3d.Window` reference for the UI context lifetime.
- Acquire a `Rylogic.D3D12.DeviceLease` from `View3d`, use `Rylogic.Gfx`'s existing friend-assembly access to borrow it only for `ContextCreate`, and let native View3DUI retain/release its own COM reference.
- Provide span/caller-buffer event and semantic APIs plus allocating cold-path conveniences.
- Keep all WPF references in downstream adapters, not this package.

**Exit criteria**

- `Rylogic.Gfx` retains its existing `Rylogic.D3D12`, `Rylogic.Core`, and `Rylogic.Native` dependencies with no `Rylogic.Gfx.WPF`, `Rylogic.Gui.WPF`, or WindowsDesktop WPF framework reference.
- The existing `Rylogic.Gfx` package contains the new API; no second managed assembly or package is produced.
- Managed tests cover layout parity records, ownership, wrong-thread use, ABI mismatch simulation, transaction rejection, event drain, semantics, and disposal.
- No managed delegate crosses into per-frame rendering.

**Validation**

```powershell
dotnet build "projects\rylogic\Rylogic.Gfx\Rylogic.Gfx.csproj" -c Debug
```

### M7. JSON conversion tooling

**Depends on:** M6.

**Work**

- Define a versioned JSON document schema for resources, templates, styles, and trees.
- Deserialize JSON into the exact same managed descriptor values used by runtime construction.
- Validate and report source locations in managed code before building the native transaction.
- Add canonical serialization and equivalence tests.

**Exit criteria**

- Equivalent JSON and runtime construction produce equivalent managed descriptors and identical native accepted snapshots.
- The native DLL has no JSON parser and receives no raw application JSON.
- Unknown schema versions/types/resources fail explicitly.

**Validation**

Run the M6 managed build/tests with JSON equivalence fixtures.

### M8. World-anchored roots

**Depends on:** all M5 exit criteria and signed-off M4 attended evidence.

**Work**

- Enable world root descriptors with constant-apparent-DIP and true-world-unit sizing.
- Implement per-subtree `Overlay`, `DepthTested`, and `OcclusionFaded` policies through typed View3D host stages.
- Add the View3D-owned single-sample depth-resolve resource required by `OcclusionFaded`; do not reinterpret the existing MSAA depth SRV as resolved depth.
- Define projection, clipping, behind-camera, near-plane, and semantic-bound rules.
- Add camera/depth/alpha ordering tests.

**Exit criteria**

- Each sizing/policy combination has deterministic layout, draw order, and semantics.
- DepthTested roots use View3D-owned depth state.
- OcclusionFaded roots read depth only through the host-provided view and use explicit bounded fade parameters.
- Screen roots remain in the true final-overlay phase.

### M9. Full text editing and IME breadth

**Depends on:** M6.

**Work**

- Complete grapheme-aware caret movement, word movement, selection, bidi, surrogate pairs, dead keys, IME composition/candidate positioning, and accessibility text ranges.
- Define font packaging/substitution policy needed for cross-machine pixel determinism.

**Exit criteria**

- Windows text/IME scenarios pass attended tests for the selected languages/input methods.
- Deterministic injected tests cover composition lifecycle without Win32 UI.
- Missing fonts follow the approved explicit resource policy.

### M10. Windows UI Automation bridge

**Depends on:** stable M6 semantics and M9 text semantics.

**Work**

- Implement an HWND UIA provider over semantic snapshots and normalized semantic actions.
- Marshal UIA requests to the owner thread without replacing or duplicating the element tree.

**Exit criteria**

- Inspect.exe/Narrator observe ID, role, name, description, value/state, actions, focus, and bounds.
- UIA action invocation enters the same control logic and event queue as normalized input.

## 12. Requirement-to-Milestone Traceability

| Requirement | Milestones |
| --- | --- |
| Dynamic native satellite and native facade | M0, M5 |
| WPF-free managed API | M6 |
| Retained, closed control system | M1, M2 |
| Stable IDs and copyable typed managed descriptors | M1, M6 |
| Central typed event drain, no element handlers | M1, M2, M6 |
| JSON/runtime construction convergence; no native JSON | M6, M7 |
| Atomic monotonic revision transactions | M1 |
| Durable app state vs transient native state | M1, M2, M4, M6 |
| Stack, Overlay, Scroll, Canvas only | M2, M5 |
| DIPs and bounded declarative transitions | M2, M3, M5 |
| Transitions cannot mutate values or invoke commands | M2 |
| Screen and world roots with three world policies | M1.5 stage seam, M8 implementation |
| View3D-owned targets/barriers/order/queue/present | M0 contract, M1.5 proof, M3 renderer |
| True final overlay after alpha/scene diagnostics | M1.5, M3 |
| Raw Win32 input first and normalized injection | M2, M4 |
| UI input blocks camera; outside click returns ownership | M2, M4 |
| Semantics and testable snapshots; UIA-compatible model | M1, M2, M10 |
| Owner-thread affinity and no managed render callbacks | M0-M6 |
| Startup dynamic loading; no live replacement | M0, M4 |
| Explicit bounded failures | M0, M1, M1.5, M3, M5, M6 |
| Explicit UI time independent of scene time and determinism | M2, M4-M6 |
| Semi-transparent panel/Text/TextBox/Button demo | M4 |
| Decimal input, validation, mouse/keyboard Update | M2, M4 |
| Existing box uniformly updated | M4 |
| Resize/DPI/lifecycle/package evidence | M3-M6 |
| Full text/IME breadth deferred without closing seam | M2 seam, M9 |

## 13. Non-goals

- Immediate-mode APIs.
- User-defined native or managed controls.
- Native control subclassing.
- Arbitrary draw callbacks, shader injection, or managed render callbacks.
- WPF controls, XAML rendering, or a WPF dependency in `Rylogic.Gfx`.
- Parsing application JSON in native code.
- Live DLL replacement or state migration.
- Transparent D3D12 device-loss recovery in the first slice; device removal is detected and faulted explicitly.
- A general View3D plug-in renderer interface.
- Grid, dock, flexbox, constraint, flow, or application-defined layout.
- Data binding, dependency properties, routed events, MVVM framework integration, or automatic application-state ownership.
- Full UI Automation in the first slice.
- Full multilingual editing/IME presentation in the first slice.
- World-root rendering in the demonstration.
- Pixel-perfect reproduction of the orientation mock-up.
- Replacing the existing `m_obj0` handle or introducing a separate demonstration box.
- Backward-compatibility adapters before a concrete compatibility need is approved.

## 14. Approved and Deferred Decisions

Review accepted decisions 1-5 below. Font packaging remains deliberately deferred until the managed/full-text milestones because it does not block the native vertical slice.

1. **Host integration:** Approve the dedicated private View3DUI provider bridge, explicit leased D3D12 device, and frame split proven by M1.5, rather than direct swap-chain D2D, a generic render callback, or application-owned command lists. **Recommended: approve.**
2. **Revision rule:** Approve contiguous `revision == base_revision + 1`, rather than accepting any greater monotonic revision. Contiguous revisions make lost updates explicit. **Recommended: approve.**
3. **Queue overflow:** Approve latest-value coalescing for state/text proposals, never silently dropping `CommandInvoked`, and explicit input fault/reconciliation when no capacity remains. Alternatives that drop newest/oldest commands are rejected because commands are not reconstructible state. **Recommended: approve coalescing plus explicit fault.**
4. **Numeric grammar:** Approve invariant `.` decimal input with finite `value > 0`, pending incomplete forms, no clamping, and disabled Update until valid. A locale-aware grammar would expand test and IME scope. **Recommended: approve invariant grammar for the demo.**
5. **Text renderer:** Approve DirectWrite shaping/metrics plus CPU glyph coverage and View3D-command-list uploads, rather than D3D11-on-12/Direct2D queue submission. **Recommended: approve.**
6. **Font determinism — deferred:** The native demo uses an exact system font. Before M6/M9, decide whether the packaged product ships an approved font asset or requires an exact installed family and reports `MissingAsset`.

## 15. Risks and Alternatives

| Risk | Mitigation/decision |
| --- | --- |
| Existing `m_present` ordering has forward and ray-tracing writers and is easy to regress | Move both to composite, use explicit phase names, one final PRESENT transition, assert no later writer, and validate under D3D12 debug/GPU validation |
| Cross-DLL D3D12 device lifetime is ambiguous | Reuse the View3D/physics device-lease pattern; native context owns one AddRef until all UI GPU resources retire |
| Cross-DLL callback survives unload incorrectly | Runtime shutdown rejects live contexts; facade keeps module loaded; context detaches before window destruction; lifecycle stress tests |
| Provider state leaks into later commands | Dedicated final-overlay command list and no later renderer draw work before the final transition |
| Provider re-enters View3D while its DLL lock is held | Bridge supplies all required data and explicitly forbids public `View3D_*` re-entry during recording |
| DirectWrite cache growth is unbounded | Fixed page/byte limits, deterministic eviction, diagnostics, and explicit `ResourceLimit` |
| Text proposals race application revisions | Event source revision/sequence plus transient pending edit generation and explicit reconciliation |
| Event overflow loses a command edge | Coalesce only reconstructible/latest-value events; reject and fault before committing an undeliverable command |
| Hash containers introduce nondeterministic order | Stable descriptor/document order and sorted ID tables at snapshot/draw boundaries |
| Wrong-thread calls currently assert in parts of View3D | View3DUI returns stable `WrongThread`; host bridge validates before invoking; tests do not rely on debug-only asserts |
| Raw Win32 handling can conflict with `gui::Form` | Offer every message to View3DUI first through `ProcessWindowMessage`; consumed messages stop before Form/camera handling |
| Test-host DPI math currently scales physical pixels twice | Correct `OnWindowPosChange`; carry explicit client/render/viewport/DPI values and test non-100% hit alignment |
| UI transitions freeze with the test app's paused scene clock | Maintain a separate monotonically advancing UI clock and prove animation while `EStepMode::Single` is active |
| Outside click could clear focus but lose camera action | Clear focus while returning the same no-hit mouse-down as unconsumed |
| UI resource failure after transaction validation | Keep last renderable snapshot until the new revision's resources are realizable; report explicit failure and do not present a success-shaped partial revision |
| Screen-only bridge could block world policies | Versioned provider table reserves scene-adjacent and post-alpha world stages; M8 adds the currently absent resolved-depth resource |
| Public templates become an arbitrary drawing language | Closed visual primitives/properties, required control parts, bounded counts, no shader/geometry/callback escape hatch |
| Native and managed layouts drift | API version, struct headers, struct-size discovery, static assertions, managed `Marshal.SizeOf` tests |
| Managed UI accidentally pulls WPF into `Rylogic.Gfx` | Keep the API in `Rylogic.Gfx.UI`, add no WindowsDesktop/WPF reference, and assert package dependencies during inspection |
| Box motion obscures dimension change | Keep the box centered at the origin with a simple world-Z rotation; derive the local +X audio emitter from the same transform and verify resize visually |

## 16. Review Gate

The plan is approved, but implementation remains paused until explicitly requested. M0 is the first implementation milestone; no later milestone should broaden the closed control or layout vocabulary without a demonstrated requirement and a plan update.

## 17. Critique Resolution

The read-only critique was performed against the draft and current repository by a separate `claude-opus-5` architecture reviewer. Material findings were resolved as follows:

| Finding | Resolution |
| --- | --- |
| Ray-tracing fullscreen diagnostics also append to `Frame::m_present` | **Accepted.** Added `render_ray_tracing.cpp`, moved all final scene writers into composite, and added a no-writer-after-PRESENT invariant. |
| Satellite D3D12 device ownership was undefined | **Accepted.** Reused the existing View3D/physics device-lease pattern and added native/managed COM lifetime tests and `Rylogic.D3D12` integration. |
| Demo transitions used the normally paused `m_time` | **Accepted.** Added an independent host UI clock and attended evidence with scene time paused. |
| Existing DPI resize logic double-scales physical client pixels | **Accepted.** Added the exact client-pixel/render-pixel/DIP contract and made correction of `OnWindowPosChange` part of M4. |
| Drop-newest could lose `CommandInvoked` | **Accepted.** Replaced it with latest-value coalescing plus explicit overflow fault before an undeliverable command is committed. |
| Full control work preceded proof of the risky render bridge | **Accepted.** Added M1.5 to prove phase ordering, provider recording, alpha/ray-tracing interaction, and device lifetime before M2. |
| World `OcclusionFaded` lacks a resolved depth resource | **Accepted.** Reserved distinct world stages now and made a new View3D-owned depth resolve explicit in M8. |
| Deployment should copy the DLL beside the executable | **Corrected.** The plan now follows the existing `$(OutDir)lib\<platform>\<config>` satellite search/copy convention instead. |
| Provider reentrancy relies on View3D's recursive mutex | **Rejected as a design dependency.** The provider is prohibited from calling public View3D exports during recording; the bridge must pass everything it needs. |
| First slice should transparently recover device loss | **Rejected as first-slice scope.** It detects and faults device removal explicitly; transparent recovery remains deferred. |
| Test and ABI details were underspecified | **Accepted.** Added a focused native `PR_UNITTESTS` executable, exact run commands, an enumerated struct-size contract, delta transaction semantics, and transition non-mutation tests. |
