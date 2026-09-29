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

Light markers (small spheres) are added only for lights that do not cast shadows, because a marker around a shadow-casting light would block its light.

A skinned-character scene (G1) is not included because the repository has no skinned test asset.

## Phase 2 checks (shadow atlas)

- Every light with `*CastShadow` > 0 casts shadows, up to the scene's max shadow lights (default 4). Further shadow lights still light the scene without shadows.
- Point lights render six cube-face views. In `s2_point_shadow_room.ldr` check for seams where the faces meet on the walls and floor.
- `s3_many_shadow_lights.ldr` needs 1 + 3 x 6 = 19 views, all drawn in one pass using viewport arrays.
- The atlas size and max shadow lights are saved per scene in LDraw's settings and can be changed with MCP `set_render_settings` (`shadow_atlas_size`, `max_shadow_lights`).
  Smaller atlases should give lower-resolution shadows, and a max of 0 should turn all shadows off.
- The 'Shadows' slider in the context menu sets the shadow strength of the main light (light 0).
