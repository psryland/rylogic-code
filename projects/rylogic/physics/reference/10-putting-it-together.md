# Tutorial 10 — Putting It Together: The Floating Cube

This tutorial walks through the complete physics pipeline for the floating cube in Lost at Sea, connecting every concept from the previous tutorials.

## Setup: Creating the Rigid Body

```cpp
// ship.cpp — Constructor
Ship::Ship(Renderer& rdr, Ocean const& ocean, v4 location)
    :m_col_shape(v4{1, 1, 1, 0})           // 1×1×1 box collision shape
    ,m_body(&m_col_shape, m4x4::Identity(),
        Inertia::Box(v4{0.5f, 0.5f, 0.5f, 0}, 100.0f))  // 100 kg box
    ,m_inst()
```

### What `Inertia::Box` Computes

For a solid box with half-extents $(h_x, h_y, h_z) = (0.5, 0.5, 0.5)$ and mass $m = 100$:

$$I_{xx} = \frac{1}{3}(h_y^2 + h_z^2) = \frac{1}{3}(0.25 + 0.25) = 0.1\overline{6}$$

The unit inertia (inertia / mass) is stored in `m_diagonal`:
```
m_diagonal = (0.1667, 0.1667, 0.1667, 0)  — Ixx, Iyy, Izz
m_products = (0, 0, 0, 0)                  — Ixy, Ixz, Iyz (zero for symmetric box)
m_com_and_mass = (0, 0, 0, 100)            — CoM at origin, mass = 100 kg
```

Since the CoM is at the origin (`m_os_com = v4{}`), the spatial inertia is **block-diagonal** (Tutorial 08) — no coupling between linear and angular motion. In the full 6×6 form:

$$\hat{I} = \begin{bmatrix} 16.67 \cdot \mathbf{1}_3 & 0 \\ 0 & 100 \cdot \mathbf{1}_3 \end{bmatrix}$$

The inverse inertia is simply the inverse of each block.

## Each Frame: Forces and Integration

### Step 1: Gravity

```cpp
auto gravity_force = v4{0, 0, -9.81f * 100.0f, 0};  // -981 N in Z
m_body.ApplyForceWS(gravity_force, v4Zero);
```

This creates a spatial force vector:

$$\hat{\mathbf{f}}_{gravity} = \begin{bmatrix} \boldsymbol{\tau} \\ \mathbf{f} \end{bmatrix} = \begin{bmatrix} 0 \\ 0 \\ 0 \\ 0 \\ 0 \\ -981 \end{bmatrix}$$

Since gravity acts at the CoM and the CoM is at the model origin, there's no torque. The `Shift` inside `ApplyForceWS` is a no-op here.

### Step 2: Buoyancy

```cpp
auto ws_com = m_body.CentreOfMassWS();
auto surface_z = ocean.HeightAt(ws_com.x, ws_com.y, sim_time);
auto bottom_z = ws_com.z - 0.5f;
auto submerged_height = std::clamp(surface_z - bottom_z, 0.0f, 1.0f);

if (submerged_height > 0)
{
    auto buoyancy = 1025.0f * 9.81f * submerged_height * 1.0f;
    auto buoyancy_force = v4{0, 0, buoyancy, 0};
    m_body.ApplyForceWS(buoyancy_force, v4Zero);
}
```

The buoyancy force depends on submersion:

| Submersion | Buoyancy (N) | Net vertical force (N) |
|------------|-------------|----------------------|
| 0.00 m | 0 | -981 (falling) |
| 0.05 m | 503 | -478 (slowing) |
| 0.098 m | 981 | 0 (**equilibrium**) |
| 0.50 m | 5028 | +4047 (rising fast) |
| 1.00 m | 10055 | +9074 (fully submerged, strong upward force) |

**Equilibrium depth**: $d = \frac{m}{ρ_{water}} = \frac{100}{1025} ≈ 0.098$ m. About 10% of the cube is submerged — physically correct for a 100 kg/m³ cube in 1025 kg/m³ water.

### Step 3: Drag

```cpp
auto velocity = m_body.VelocityWS();       // v8motion
auto drag_lin = -200.0f * velocity.lin;     // Linear drag
auto drag_ang = -50.0f * velocity.ang;      // Angular drag
m_body.ApplyForceWS(drag_lin, drag_ang);
```

Drag is applied as a damping force proportional to velocity. This:
- Prevents endless oscillation around the equilibrium
- Simulates water resistance
- Both the linear and angular components are damped

### Step 4: Evolve

```cpp
pr::physics::Evolve(m_body, dt);
```

What happens inside (see Tutorial 09 for details):

1. **Mid-step momentum**: $\hat{h}_{mid} = \hat{h}_0 + \hat{f} \cdot \frac{\Delta t}{2}$
2. **Mid-step velocity**: $\hat{v}_{mid} = \hat{I}^{-1}_{mid} \hat{h}_{mid}$
3. **Position update**: Rotation from $\hat{v}_{mid}.\text{ang} \cdot \Delta t$, translation from $\hat{v}_{mid}.\text{lin} \cdot \Delta t$
4. **Momentum update**: $\hat{h}_{new} = \hat{h}_0 + \hat{f} \cdot \Delta t$
5. **Reset forces**: all accumulated forces zeroed

### Step 5: Render

```cpp
void Ship::PrepareRender(v4)
{
    m_inst.m_i2w = m_body.O2W();   // Copy physics transform to graphics
}
```

## The Physics of Floating

### Why It Works

At each frame:
1. Gravity pushes the cube down (constant $-981$ N)
2. Buoyancy pushes it up (proportional to submersion)
3. Drag slows everything down
4. Evolve integrates forces → momentum → position

The cube settles at the depth where buoyancy equals gravity. The ocean surface moves (waves), so the buoyancy force changes each frame, making the cube bob with the waves.

### The Spatial Algebra at Work

Even though this simple example doesn't show much angular coupling (the forces act at the CoM, producing no torque), the spatial algebra framework is ready for it:

- **Offset forces**: If buoyancy were applied at the bottom face instead of the CoM, the `Shift()` inside `ApplyForceWS` would automatically compute the resulting torque. This would cause the cube to tilt.
- **Asymmetric shapes**: For a ship shape where the CoM isn't at the geometric centre, the spatial inertia's off-diagonal blocks would couple linear and angular motion automatically.
- **Gyroscopic effects**: A spinning body's $\hat{v} \times^{*} \hat{I}\hat{v}$ term (captured implicitly by the midpoint integration) would produce precession.

## Type Flow Summary

```
Gravity (v4)  ──→  ApplyForceWS()  ──→  m_ws_force (v8force)  ──→  Evolve()
Buoyancy (v4) ──→  ApplyForceWS()  ──→  m_ws_force (v8force)  ──→    │
Drag (v4)     ──→  ApplyForceWS()  ──→  m_ws_force (v8force)  ──→    │
                                                                       ↓
                                              m_ws_momentum (v8force)  +=  force * dt
                                                                       ↓
                                              InertiaInvWS() * momentum  →  velocity (v8motion)
                                                                       ↓
                                              velocity * dt  →  position/rotation change
                                                                       ↓
                                              m_o2w (m4x4)  ←──  updated transform
                                                                       ↓
                                              m_inst.m_i2w  ←──  copied for rendering
```

## Concepts Used

| Tutorial | Concept | Where it appears |
|----------|---------|-----------------|
| 01 | Dual vector spaces | `v8force` for forces/momentum, `v8motion` for velocity |
| 02 | Motion as a velocity field | `VelocityWS()` describes the whole-body velocity |
| 03 | Force vectors | `m_ws_force` accumulator, gravity/buoyancy as `v8force` |
| 04 | Dot product = energy | Debug KE check in Evolve |
| 05 | Cross products | Inside `Shift()` when applying forces at offset points |
| 06 | Shifting | `ApplyForceWS()` shifts forces to the model origin |
| 07 | Transforms | `InertiaInvWS()` rotates the object-space inertia to world space |
| 08 | Spatial inertia | `Inertia::Box()` constructs the 6×6 mass matrix |
| 09 | Equations of motion | `Evolve()` integrates momentum and updates position |

## What's Next

You now have the conceptual foundation for spatial algebra as used in the Rylogic physics engine. Future extensions:
- **Multi-point buoyancy**: Apply forces at different points on the hull → `Shift()` creates torques → body tilts realistically
- **Collision response**: Impulses (`v8force`) applied at contact points
- **Articulated bodies**: Joints connecting rigid bodies, using Featherstone's algorithm
- **Featherstone's RBDA book** for the full theoretical treatment
