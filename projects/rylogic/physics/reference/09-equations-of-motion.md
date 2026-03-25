# Tutorial 09 — Equations of Motion

## Newton-Euler in Spatial Form

The classical Newton-Euler equations (force = mass × acceleration, torque = inertia × angular acceleration) unify into a single spatial equation:

$$\hat{\mathbf{f}} = \hat{I}\hat{\mathbf{a}} + \hat{\mathbf{v}} \times^{*} \hat{I}\hat{\mathbf{v}}$$

where:
- $\hat{\mathbf{f}}$ = net spatial force (`v8force`)
- $\hat{I}$ = spatial inertia (`Inertia`)
- $\hat{\mathbf{a}}$ = spatial acceleration (`v8motion`)
- $\hat{\mathbf{v}}$ = spatial velocity (`v8motion`)
- $\hat{\mathbf{v}} \times^{*}$ = force-space cross product with velocity

### Breaking It Down

| Term | Meaning |
|------|---------|
| $\hat{I}\hat{\mathbf{a}}$ | Inertial resistance — the force needed to accelerate the body |
| $\hat{\mathbf{v}} \times^{*} \hat{I}\hat{\mathbf{v}}$ | Velocity-dependent forces — gyroscopic, Coriolis, centrifugal |

The velocity-dependent term is zero when the body isn't rotating ($\boldsymbol{\omega} = 0$) and vanishes for a body whose inertia is symmetric about the rotation axis (e.g., a uniform sphere).

### Solving for Acceleration

Rearranging:

$$\hat{\mathbf{a}} = \hat{I}^{-1}(\hat{\mathbf{f}} - \hat{\mathbf{v}} \times^{*} \hat{I}\hat{\mathbf{v}})$$

In code terms: `acceleration = inertia_inv * (force - Cross(velocity, inertia * velocity))`.

## Momentum Form

In practice, it's easier to work with **momentum** rather than velocity:

$$\hat{\mathbf{h}} = \hat{I}\hat{\mathbf{v}}$$

Newton's second law in momentum form:

$$\hat{\mathbf{f}} = \frac{d\hat{\mathbf{h}}}{dt}$$

Force equals the rate of change of momentum. For a discrete time step $\Delta t$:

$$\hat{\mathbf{h}}_{t+\Delta t} = \hat{\mathbf{h}}_t + \hat{\mathbf{f}} \cdot \Delta t$$

This is the approach Rylogic uses. The `RigidBody` stores **momentum** (`m_ws_momentum`), not velocity. Velocity is computed on demand:

```cpp
v8motion VelocityWS() const
{
    return InertiaInvWS() * MomentumWS();
}
```

### Why Momentum Instead of Velocity?

1. **Momentum is frame-independent** in the sense that it transforms simply and conserves naturally.
2. **The inertia changes** as the body rotates (world-space inertia depends on orientation). Storing velocity would require updating it when the inertia changes. Storing momentum avoids this — the momentum stays the same; only the velocity-from-momentum mapping changes.
3. **Forces integrate directly into momentum** ($\Delta\hat{\mathbf{h}} = \hat{\mathbf{f}} \cdot \Delta t$) without needing the inertia at all.

## The `Evolve()` Function

The integrator in `include/pr/physics/integrator/integrator.h` implements a **semi-implicit midpoint method**:

```cpp
void Evolve(RigidBody& rb, float elapsed_seconds)
```

### Step-by-Step Walkthrough

**1. Read current state**
```cpp
auto ws_force = rb.ForceWS();           // Net accumulated force (v8force)
auto ws_inertia_inv = rb.InertiaInvWS(); // Current inverse inertia (InertiaInv)
```

**2. Estimate mid-step momentum**

The force is assumed constant over the step, so the momentum at the midpoint ($t + \frac{\Delta t}{2}$) is:

$$\hat{\mathbf{h}}_{mid} = \hat{\mathbf{h}}_0 + \hat{\mathbf{f}} \cdot \frac{\Delta t}{2}$$

```cpp
auto ws_momentum = rb.MomentumWS() + ws_force * elapsed_seconds * 0.5f;
```

**3. Refine the inverse inertia at the midpoint**

The world-space inertia depends on orientation, which changes during the step due to angular velocity. Estimate the orientation change at the midpoint and re-derive the inertia:

```cpp
for (int i = 0; i != 1; ++i)
{
    auto ws_velocity = ws_inertia_inv * ws_momentum;
    auto dpos = ws_velocity * elapsed_seconds * 0.5f;
    auto do2w = m3x3::Rotation(dpos.ang);       // Estimated rotation at midpoint
    ws_inertia_inv = Rotate(ws_inertia_inv, do2w); // Inertia at midpoint orientation
}
```

This is the key insight: using the inertia at the *midpoint* rather than the *start* gives much better accuracy for spinning bodies.

**4. Compute velocity and position change**

Using the mid-step inertia and mid-step momentum:

$$\hat{\mathbf{v}}_{mid} = \hat{I}^{-1}_{mid} \hat{\mathbf{h}}_{mid}$$
$$\Delta\hat{\mathbf{p}} = \hat{\mathbf{v}}_{mid} \cdot \Delta t$$

```cpp
auto ws_velocity = ws_inertia_inv * ws_momentum;
auto dpos = ws_velocity * elapsed_seconds;
```

**5. Update position and orientation**

The position change has angular and linear parts:

```cpp
auto o2w = m4x4
{
    m3x3::Rotation(dpos.ang) * rb.O2W().rot,  // Incremental rotation
    dpos.lin                 + rb.O2W().pos    // Linear displacement
};
rb.O2W(o2w);
```

**6. Update momentum (full step)**

$$\hat{\mathbf{h}}_{new} = \hat{\mathbf{h}}_0 + \hat{\mathbf{f}} \cdot \Delta t$$

```cpp
rb.MomentumWS(rb.MomentumWS() + ws_force * elapsed_seconds);
```

**7. Reset forces**
```cpp
rb.ZeroForces();
```

Forces must be re-applied each frame. This is important: gravity, buoyancy, drag — all must be added every step.

**8. Orthonormalise**
```cpp
rb.O2W(Orthonorm(rb.O2W()));
```

Floating-point drift causes the rotation matrix to deviate from orthogonal. Periodic orthonormalisation corrects this.

## Where Is $\hat{\mathbf{v}} \times^{*} \hat{I}\hat{\mathbf{v}}$?

You might notice the Evolve function doesn't explicitly compute the velocity-dependent term ($\hat{\mathbf{v}} \times^{*} \hat{I}\hat{\mathbf{v}}$). That's because the momentum-based formulation absorbs it:

When working with momentum directly:
$$\frac{d\hat{\mathbf{h}}}{dt} = \hat{\mathbf{f}}_{ext}$$

The velocity-dependent terms arise naturally from the fact that $\hat{\mathbf{v}} = \hat{I}^{-1}\hat{\mathbf{h}}$ and $\hat{I}$ changes with orientation. By using the midpoint inertia, these effects are captured implicitly.

**Important**: The equation $\hat{\mathbf{f}} = \hat{I}\hat{\mathbf{a}} + \hat{\mathbf{v}} \times^{*} \hat{I}\hat{\mathbf{v}}$ is the **body-frame** formulation (Featherstone RBDA §2.5). In the **world frame**, where the integrator operates, the equivalent is simply $d\hat{\mathbf{h}}/dt = \hat{\mathbf{f}}$. The gyroscopic/Euler effects are embedded in the time-varying world-space inertia $\hat{I}_{WS}(t)$, not in an explicit correction term.

## Debug Energy Check

In debug builds, Evolve verifies energy conservation using the power formula:

```cpp
#if PR_DBG
auto ke_before = rb.KineticEnergy();
// ... compute ws_velocity (mid-step), ws_force ...
auto ke_change = Dot(ws_velocity, ws_force) * elapsed_seconds;
// ... evolve ...
auto ke_after = rb.KineticEnergy();
assert(FEqlRelative(ke_before + ke_change, ke_after, 0.1f * elapsed_seconds));
#endif
```

This catches integration errors: the kinetic energy after the step should equal the energy before plus the work done by external forces ($P = \hat{\mathbf{v}} \cdot \hat{\mathbf{f}}$). The tolerance accommodates the discrete approximation of the time-varying inertia, which introduces O($\Delta t^2$) error.

## Summary

| Concept | Formula | Rylogic |
|---------|---------|---------|
| Newton-Euler | $\hat{f} = \hat{I}\hat{a} + \hat{v} \times^{*} \hat{I}\hat{v}$ | Implicit in momentum integration |
| Momentum update | $\hat{h}_{new} = \hat{h}_0 + \hat{f} \cdot \Delta t$ | `MomentumWS() + force * dt` |
| Velocity from momentum | $\hat{v} = \hat{I}^{-1}\hat{h}$ | `InertiaInvWS() * MomentumWS()` |
| Position update | $\Delta p = \hat{v}_{mid} \cdot \Delta t$ | Rotation + translation applied to `O2W` |
| Integration method | Semi-implicit midpoint | Mid-step inertia refinement loop |

## What's Next

[Tutorial 10](10-putting-it-together.md) walks through a complete worked example: the floating cube in Lost at Sea, showing how gravity, buoyancy, and Evolve come together.
