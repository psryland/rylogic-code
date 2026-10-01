# Output dithering

The main render target stores 8 bits per channel. Smooth, dark gradients (for example, deep translucent water or a
sunset sky) can show visible bands where neighbouring pixels round to the same 8-bit value. Output dithering adds a
small, fixed per-pixel noise before each write, so the rounding error looks like fine grain instead of bands.

## Setting

`Window::m_dither_amount` is the noise amplitude in 8-bit sRGB steps. It is `0` (off) by default, so existing
applications and exact-colour tests are unchanged. `1` is a good value for removing banding.

- DLL: `View3D_DitherAmountGet/Set(window, amount)`.
- C#: `View3d.Window.DitherAmount`. Changing the value invalidates the window.

## Where noise is added

Noise is added in sRGB-encoded space, where 8-bit steps are evenly spaced, at each point where a colour is rounded to
8 bits:

| Write | Seed |
|-------|------|
| Forward pixel shader output (including PBR, reflection-attribute and radial-fade entries) and the procedural sky | 0 |
| Translucent K-buffer layer packing | 1 |
| K-buffer resolve (only pixels with a translucent contribution) | 2 |
| Underwater post effect | 3 |

Each write uses a different seed, so noise from consecutive writes is not correlated. The noise is triangular in
`[-amount, +amount]` steps and fixed per pixel, so a still image does not flicker. Reflection attributes are computed
from the undithered colour.

The helpers live in `src/shaders/hlsl/utility/colour_space.hlsli` (`DitherOffsetSrgb8`, `DitherSrgb8`). The forward
shaders read the amount from `CBufFrame::output.x`; the K-buffer resolve reads it from a root constant at `b0`; the
underwater pass reads it from its own constant buffer.

## K-buffer precision

Translucent K-buffer layers store their colour sRGB-encoded in RGBA8 (`PackSrgbRGBA8`/`UnpackSrgbRGBA8` in
`forward/kbuffer.hlsli`). sRGB encoding gives dark colours far more precision than linear storage, which matters
because dark translucent layers are where banding is most visible. This applies whether or not dithering is enabled.

## Validation

Build `projects\tests\view3d-fade-tests\view3d-fade-tests.vcxproj` (Debug/x64) and run
`obj\x64\Debug\view3d-fade-tests.exe --dither`. At 1x and 4x MSAA, it checks that dithering is off by default, that
noise stays within a few steps and keeps the mean colour for opaque and translucent surfaces, and that turning it off
restores the exact original image.
