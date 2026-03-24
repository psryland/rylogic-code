# Tutorial 08 — Spatial Inertia

## From Mass and Inertia Tensor to a Single Matrix

In classical mechanics, a rigid body has three mass properties:
- **Mass** $m$ (scalar) — resistance to linear acceleration
- **Inertia tensor** $I_c$ (3×3 matrix) — resistance to angular acceleration about the CoM
- **Centre of mass** $\mathbf{c}$ (3-vector) — offset from the reference point

Spatial algebra rolls all three into a single 6×6 matrix that maps velocity to momentum:

$$\hat{I} = \begin{bmatrix} I_c - m\mathbf{c}^{\times}\mathbf{c}^{\times} & -m\mathbf{c}^{\times} \\ m\mathbf{c}^{\times} & m\mathbf{1} \end{bmatrix}$$

This matrix maps `v8motion` → `v8force`:

$$\hat{\mathbf{h}} = \hat{I}\hat{\mathbf{v}} \qquad \text{(momentum = inertia × velocity)}$$

## Understanding the Blocks

$$\hat{I} = \begin{bmatrix} I_o & -m\mathbf{c}^{\times} \\ m\mathbf{c}^{\times} & m\mathbf{1} \end{bmatrix}$$

where $I_o = I_c - m\mathbf{c}^{\times}\mathbf{c}^{\times}$ is the inertia about the reference point (via the parallel axis theorem).

| Block | Size | Maps | Physical meaning |
|-------|------|------|-----------------|
| $I_o$ (top-left) | 3×3 | ω → τ | Rotational inertia at the reference point |
| $-m\mathbf{c}^{\times}$ (top-right) | 3×3 | v → τ | Coupling: linear motion creates angular momentum |
| $m\mathbf{c}^{\times}$ (bottom-left) | 3×3 | ω → f | Coupling: rotation creates linear momentum |
| $m\mathbf{1}$ (bottom-right) | 3×3 | v → f | Mass (scalar × identity) |

**When CoM is at the origin** ($\mathbf{c} = 0$), the off-diagonal blocks vanish and the matrix is block-diagonal — angular and linear decouple completely.

## The Rylogic `Inertia` Struct

The `Inertia` struct stores the mass properties compactly (not as a full 6×6 matrix):

```cpp
struct Inertia
{
    v4 m_diagonal;       // Ixx, Iyy, Izz of the unit inertia at CoM
    v4 m_products;       // Ixy, Ixz, Iyz of the unit inertia at CoM
    v4 m_com_and_mass;   // (com.x, com.y, com.z, mass)
};
```

Key design choices:
- Stores the **unit inertia** ($I_c / m$), not the full inertia. This avoids numerical issues at extreme masses.
- The CoM offset is stored alongside the inertia so it can be parallel-axis-translated on demand.
- The full 6×6 matrix can be computed via `To6x6()`.

### Accessors

```cpp
float Mass() const;           // Mass of the body
float InvMass() const;        // 1/mass
v4 CoM() const;               // Centre of mass offset
m3x4 Ic3x3(float scale) const; // 3×3 unit inertia at CoM, multiplied by scale
m3x4 To3x3() const;           // 3×3 inertia at reference point (parallel-axis translated)
Mat6x8<float,Motion,Force> To6x6() const;  // Full 6×6 spatial inertia matrix
```

### `To6x6()` Implementation

```cpp
Mat6x8<float, Motion, Force> To6x6() const
{
    auto mcx = CPM(Mass() * CoM());
    return Mat6x8<float,Motion,Force>(
        Mass() * Ic3x3(1),  mcx,     // [Ic  , mcx ]
        -mcx,                m3x4::Scale(Mass()));  // [-mcx, m*1 ]
    //  m00                  m01
    //  m10                  m11
}
```

This matches the spatial inertia formula from the top of this tutorial.

## Inverse Inertia: `InertiaInv`

The inverse inertia maps `v8force` → `v8motion`:

$$\hat{\mathbf{v}} = \hat{I}^{-1}\hat{\mathbf{f}} \qquad \text{(velocity = inverse-inertia × force)}$$

The `InertiaInv` struct has the same compact storage:

```cpp
struct InertiaInv
{
    v4 m_diagonal;          // Inverse unit inertia diagonal
    v4 m_products;          // Inverse unit inertia off-diagonal
    v4 m_com_and_invmass;   // (com.x, com.y, com.z, 1/mass)
};
```

### Key operations

```cpp
// Inertia × velocity → momentum
v8force momentum = inertia * velocity;     // Inertia × v8motion → v8force

// Inverse inertia × force → acceleration
v8motion accel = inertia_inv * force;      // InertiaInv × v8force → v8motion
```

## Constructing Inertias for Common Shapes

The `Inertia` struct provides factory methods for common shapes:

```cpp
// Point mass (all inertia is zero at CoM)
auto I = Inertia::Point(mass, com_offset);

// Solid sphere: I = (2/5) * m * r²
auto I = Inertia::Sphere(radius, mass, com_offset);

// Solid box: I_xx = (1/3)(h_y² + h_z²), etc. (h = half-extents)
auto I = Inertia::Box(half_extents, mass, com_offset);

// Infinite mass / immovable
auto I = Inertia::Infinite();

// Invert: Inertia ↔ InertiaInv
auto I_inv = Invert(I);
auto I_back = Invert(I_inv);
```

## Rotating and Translating Inertia

The inertia changes when you change coordinate frame:

### Rotation

Rotating from frame $a$ to frame $b$ (Rylogic: `Rotate(inertia, a2b)`):

$$I_b = E \cdot I_a \cdot E^T$$

Only the 3×3 unit inertia is affected. Mass and CoM offset are unchanged.

### Parallel Axis Theorem (Translation)

Moving the reference point away from the CoM (Rylogic: `Translate(inertia, offset, direction)`):

$$I_o = I_c + m d^2$$

where $d$ is the perpendicular distance. The full formula for the 3×3 inertia:

For diagonal elements:
$$I_{xx} = I_{c,xx} + m(d_y^2 + d_z^2)$$

For off-diagonal elements:
$$I_{xy} = I_{c,xy} + m \cdot d_x \cdot d_y$$

Rylogic uses `ETranslateInertia` to specify the direction:
```cpp
enum class ETranslateInertia
{
    TowardCoM,    // The offset points toward the CoM
    AwayFromCoM,  // The offset points away from the CoM
};
```

### Combined Transform

`Transform(inertia, a2b, direction)` = rotate then translate:
```cpp
auto I_new = Transform(I, a2b, ETranslateInertia::AwayFromCoM);
// Equivalent to:
//   I_new = Rotate(I, a2b.rot);
//   I_new = Translate(I_new, a2b.pos, AwayFromCoM);
```

## Summary

| Concept | Type | Maps |
|---------|------|------|
| Spatial inertia | `Inertia` → `Mat6x8<Motion,Force>` | velocity → momentum |
| Inverse inertia | `InertiaInv` → `Mat6x8<Force,Motion>` | force → acceleration |
| At CoM | Block-diagonal (no coupling) | |
| Away from CoM | Off-diagonal coupling via $m\mathbf{c}^{\times}$ | |
| Rotate | `Rotate(I, rot)` | Change coordinate axes |
| Translate | `Translate(I, ofs, dir)` | Change reference point (parallel axis theorem) |

## What's Next

[Tutorial 09](09-equations-of-motion.md) puts it all together in Newton-Euler's equation of motion and walks through the `Evolve()` integrator.
