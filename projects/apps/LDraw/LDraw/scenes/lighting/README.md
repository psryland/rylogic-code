# View3D-12 Lighting and Shadow Reference Scenes

These scenes check the multi-light and multi-shadow renderer in LDraw. Regenerate them with `python make_scenes.py`.

| File | Purpose |
|---|---|
| `b0_baseline.ldr` | Lit only by the window's global light (light 0). Turn on shadows in the lighting settings to check the directional shadow. |
| `l1_point_lights.ldr` | 16 coloured point lights over a grid of spheres. Checks the light loop and range cut-off. |
| `l2_spot_lights_alpha.ldr` | 8 spot lights plus a directional light, with transparent objects. Checks alpha (K-buffer) lighting. |
| `s1_spot_shadow.ldr` | A shadow-casting spot light over a box field. |
| `s2_point_shadow_room.ldr` | A shadow-casting point light inside a room with pillars. Every wall should show shadows with no seams. |
| `s3_many_shadow_lights.ldr` | 1 directional + 3 point shadow lights over about 2000 boxes. Checks single-pass shadow performance. |
| `s4_large_terrain.ldr` | A 1 km plane with small objects near the origin. Checks cascades, shimmer, and pixelation. |
| `s5_shadow_cache.ldr` | A static box field with one spinning caster. Checks that cached shadow views update when a caster moves. |

Light markers (small spheres) are added only for lights that do not cast shadows, because a marker around a shadow-casting light would block its light.

A skinned-character scene (G1) is not included because the repository has no skinned test asset.

## Phase 2 checks (shadow atlas)

- Every light with `*CastShadow` > 0 casts shadows, up to the scene's max shadow lights (default 4). Further shadow lights still light the scene without shadows.
- Point lights render six cube-face views. In `s2_point_shadow_room.ldr` check for seams where the faces meet on the walls and floor.
- `s3_many_shadow_lights.ldr` needs 1 + 3 x 6 = 19 views, all drawn in one pass using viewport arrays.
- The atlas size and max shadow lights are saved per scene in LDraw's settings and can be changed with MCP `set_render_settings` (`shadow_atlas_size`, `max_shadow_lights`).
  Smaller atlases should give lower-resolution shadows, and a max of 0 should turn all shadows off.
- The 'Shadows' slider in the context menu sets the shadow strength of the main light (light 0).

## Phase 3 checks (cascades, filtering, caching)

- Directional lights use cascades (default 3, each 1024 x 1024). In `s4_large_terrain.ldr`, rocks near the camera should have sharp shadows,
  and the far boxes (80 m away) should still cast shadows. Detail should reduce smoothly with distance.
- Orbit and move the camera slowly in `s4_large_terrain.ldr`. Shadow edges should not shimmer or crawl while the camera moves.
- Casters behind the camera, or outside the view, must still shade visible ground (look down at the ground next to a tall box behind the camera).
- Point and spot light shadow views shrink when the light covers less of the screen. Zoom out in `s1_spot_shadow.ldr`: shadows get softer but must not disappear.
- Shadow edges are filtered over 5 x 5 texels by default. Setting the filter to 7 should give softer edges.
- In `s5_shadow_cache.ldr`, the spinning bar's shadow must follow it with no lag or smearing, and the static box shadows must stay stable.
  Views that do not see the spinning bar are not re-rendered while the scene is still.
- MCP `set_render_settings` adds `shadow_cascades` (1-4), `shadow_distance` (0 fits the cascades to the casters), `shadow_cascade_split_blend`
  (0 = even spacing, 1 = logarithmic, default 0.5), and `shadow_filter_size` (5 or 7). These are saved per scene.
  With `shadow_distance` set to a small value, directional shadows should stop at roughly that distance from the camera.
