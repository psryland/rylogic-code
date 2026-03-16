#!/usr/bin/env python3
"""
Generate stress test scenes for the physics sandbox.

Usage:
  python generate_scene.py [options] > output.json
  python generate_scene.py --count 1000 --output stress_test.json

Options:
  --count N         Number of bodies to generate (default: 100)
  --seed N          Random seed for reproducibility (default: random)
  --output FILE     Output file path (default: stdout)
  --box-pct N       Percentage of boxes (default: 40)
  --sphere-pct N    Percentage of spheres (default: 40)
  --polytope-pct N  Percentage of polytopes (default: 20)
  --size-min F      Minimum shape dimension (default: 0.3)
  --size-max F      Maximum shape dimension (default: 2.0)
  --mass-min F      Minimum body mass in kg (default: 1.0)
  --mass-max F      Maximum body mass in kg (default: 50.0)
  --spread F        Position spread radius (default: auto based on count)
  --height-min F    Minimum drop height (default: 5.0)
  --height-max F    Maximum drop height (default: 50.0)
  --gravity F       Gravity magnitude (default: -9.81)
  --elasticity F    Collision elasticity 0..1 (default: 0.8)
  --friction F      Static friction coefficient (default: 0.3)
  --velocity F      Max initial velocity magnitude (default: 0.0)
  --angular F       Max initial angular velocity magnitude (default: 0.0)
  --no-ground       Disable ground plane
"""

import argparse
import json
import math
import random
import sys


def random_unit_vector(rng):
    """Generate a random unit vector on the sphere."""
    z = rng.uniform(-1, 1)
    phi = rng.uniform(0, 2 * math.pi)
    r = math.sqrt(1 - z * z)
    return [r * math.cos(phi), r * math.sin(phi), z]


def random_polytope_vertices(rng, size_min, size_max):
    """Generate 6-12 random vertices for a convex polytope."""
    n = rng.randint(6, 12)
    scale = rng.uniform(size_min, size_max)
    verts = []
    for _ in range(n):
        v = random_unit_vector(rng)
        r = rng.uniform(0.3, 1.0) * scale
        verts.append([v[0] * r, v[1] * r, v[2] * r])
    return verts


def generate_body(rng, index, args):
    """Generate a single body definition."""
    # Choose shape type based on percentages
    roll = rng.uniform(0, 100)
    if roll < args.box_pct:
        shape_type = "box"
    elif roll < args.box_pct + args.sphere_pct:
        shape_type = "sphere"
    else:
        shape_type = "polytope"

    # Shape definition
    if shape_type == "box":
        dx = rng.uniform(args.size_min, args.size_max)
        dy = rng.uniform(args.size_min, args.size_max)
        dz = rng.uniform(args.size_min, args.size_max)
        shape = {"type": "box", "dimensions": [dx, dy, dz]}
    elif shape_type == "sphere":
        r = rng.uniform(args.size_min / 2, args.size_max / 2)
        shape = {"type": "sphere", "radius": r}
    else:
        verts = random_polytope_vertices(rng, args.size_min, args.size_max)
        shape = {"type": "polytope", "vertices": verts}

    # Position: spread horizontally, stagger vertically
    x = rng.uniform(-args.spread, args.spread)
    y = rng.uniform(-args.spread, args.spread)
    z = rng.uniform(args.height_min, args.height_max)
    position = [round(x, 3), round(y, 3), round(z, 3)]

    # Mass
    mass = round(rng.uniform(args.mass_min, args.mass_max), 2)

    body = {
        "name": f"{shape_type}_{index}",
        "shape": shape,
        "mass": mass,
        "position": position,
    }

    # Optional initial velocity
    if args.velocity > 0:
        v = random_unit_vector(rng)
        speed = rng.uniform(0, args.velocity)
        body["velocity"] = [round(v[0] * speed, 3), round(v[1] * speed, 3), round(v[2] * speed, 3)]

    # Optional initial angular velocity
    if args.angular > 0:
        w = random_unit_vector(rng)
        spin = rng.uniform(0, args.angular)
        body["angular_velocity"] = [round(w[0] * spin, 3), round(w[1] * spin, 3), round(w[2] * spin, 3)]

    return body


def main():
    parser = argparse.ArgumentParser(description="Generate stress test scenes for the physics sandbox")
    parser.add_argument("--count", type=int, default=100, help="Number of bodies (default: 100)")
    parser.add_argument("--seed", type=int, default=None, help="Random seed (default: random)")
    parser.add_argument("--output", type=str, default=None, help="Output file (default: stdout)")
    parser.add_argument("--box-pct", type=float, default=40, help="Box percentage (default: 40)")
    parser.add_argument("--sphere-pct", type=float, default=40, help="Sphere percentage (default: 40)")
    parser.add_argument("--polytope-pct", type=float, default=20, help="Polytope percentage (default: 20)")
    parser.add_argument("--size-min", type=float, default=0.3, help="Min shape size (default: 0.3)")
    parser.add_argument("--size-max", type=float, default=2.0, help="Max shape size (default: 2.0)")
    parser.add_argument("--mass-min", type=float, default=1.0, help="Min mass kg (default: 1.0)")
    parser.add_argument("--mass-max", type=float, default=50.0, help="Max mass kg (default: 50.0)")
    parser.add_argument("--spread", type=float, default=None, help="Position spread (default: auto)")
    parser.add_argument("--height-min", type=float, default=5.0, help="Min drop height (default: 5.0)")
    parser.add_argument("--height-max", type=float, default=50.0, help="Max drop height (default: 50.0)")
    parser.add_argument("--gravity", type=float, default=-9.81, help="Gravity Z (default: -9.81)")
    parser.add_argument("--elasticity", type=float, default=0.8, help="Elasticity (default: 0.8)")
    parser.add_argument("--friction", type=float, default=0.3, help="Friction (default: 0.3)")
    parser.add_argument("--velocity", type=float, default=0.0, help="Max initial velocity (default: 0)")
    parser.add_argument("--angular", type=float, default=0.0, help="Max initial angular vel (default: 0)")
    parser.add_argument("--no-ground", action="store_true", help="Disable ground plane")
    args = parser.parse_args()

    # Auto-calculate spread based on body count if not specified.
    # Aim for bodies to be ~3 shape-widths apart on average.
    if args.spread is None:
        avg_size = (args.size_min + args.size_max) / 2
        bodies_per_side = math.ceil(math.sqrt(args.count))
        args.spread = bodies_per_side * avg_size * 1.5

    rng = random.Random(args.seed)

    # Build scene
    scene = {
        "description": f"Stress test: {args.count} bodies ({args.box_pct:.0f}% box, {args.sphere_pct:.0f}% sphere, {args.polytope_pct:.0f}% polytope)",
        "gravity": [0, 0, args.gravity],
        "material": {
            "elasticity": args.elasticity,
            "friction": args.friction,
        },
        "bodies": [generate_body(rng, i, args) for i in range(args.count)],
    }

    if not args.no_ground:
        scene["ground_plane"] = {
            "height": 0.0,
            "texture": "#checker3",
        }

    output = json.dumps({"scene": scene}, indent=2)

    if args.output:
        with open(args.output, "w") as f:
            f.write(output)
        print(f"Generated {args.count} bodies → {args.output}", file=sys.stderr)
    else:
        print(output)


if __name__ == "__main__":
    main()
