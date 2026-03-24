# Spatial Algebra Tutorial Series

A progressive series of tutorials covering spatial algebra as used in the Rylogic physics engine.

## Prerequisites

- Comfortable with 3D vectors, dot products, cross products
- Familiar with 3×3 and 4×4 matrices, rotation matrices, affine transforms
- Basic physics: Newton's laws, force, torque, momentum

## Tutorials

| # | Title | Concepts |
|---|-------|----------|
| 01 | [Why Spatial Algebra?](01-why-spatial-algebra.md) | Motivation, the problem it solves, Plücker coordinates |
| 02 | [Motion Vectors](02-motion-vectors.md) | `v8motion`, velocity fields, angular + linear components |
| 03 | [Force Vectors](03-force-vectors.md) | `v8force`, wrenches, the dual relationship to motion |
| 04 | [The Dot Product and Work](04-dot-product-and-work.md) | `Dot(v8motion, v8force)`, power, why M·M is undefined |
| 05 | [Spatial Cross Products](05-spatial-cross-products.md) | Motion cross, force cross (`×*`), `Cross()`, `CPM()` |
| 06 | [Shifting Spatial Vectors](06-shifting-spatial-vectors.md) | `Shift()`, relocating vectors to new reference points |
| 07 | [Spatial Transforms](07-spatial-transforms.md) | Changing coordinate frames, `Transform<Motion>`, `Transform<Force>` |
| 08 | [Spatial Inertia](08-spatial-inertia.md) | `Inertia`, `InertiaInv`, the 6×6 mass matrix, `To6x6()` |
| 09 | [Equations of Motion](09-equations-of-motion.md) | Newton-Euler in spatial form, `Evolve()` |
| 10 | [Putting It Together](10-putting-it-together.md) | Worked example: the floating cube in Lost at Sea |

## Reference

- Featherstone, *Rigid Body Dynamics Algorithms* (RBDA) — the primary reference
- The Rylogic implementation lives in `include/pr/maths/spatial.h` and `include/pr/physics/`
- Type conventions: `v8motion` ∈ M⁶, `v8force` ∈ F⁶ (dual vector spaces)
