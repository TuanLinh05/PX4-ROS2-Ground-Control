#!/usr/bin/env python3
"""
Trajectory Generation Utilities
Generates waypoint lists for various geometric shapes in NED frame.
"""

import math
from typing import List, Tuple

# Type alias
Waypoint = List[float]  # [x, y, z] in NED


def generate_trajectory(shape: str, params: dict) -> List[Waypoint]:
    """
    Generate trajectory waypoints for a given shape.
    
    Args:
        shape: One of 'square', 'circle', 'triangle', 'hexagon', 'octagon',
               'figure8', 'helix', 'star', 'custom'
        params: Dictionary with shape-specific parameters:
            - center_x, center_y: Center position (NED, meters)
            - size: Size/radius (meters)
            - altitude: Flight altitude (positive, will be negated for NED)
            - num_points: Number of waypoints to generate
    
    Returns:
        List of [x, y, z] waypoints in NED frame
    """
    cx = params.get('center_x', 0.0)
    cy = params.get('center_y', 0.0)
    alt = -abs(params.get('altitude', 3.0))  # NED: negative = up
    size = params.get('size', 3.0)
    n = max(3, int(params.get('num_points', 8)))

    generators = {
        'square': _gen_square,
        'circle': _gen_circle,
        'triangle': _gen_triangle,
        'hexagon': _gen_hexagon,
        'octagon': _gen_octagon,
        'figure8': _gen_figure8,
        'helix': _gen_helix,
        'star': _gen_star,
        'custom': _gen_custom,
    }

    gen_func = generators.get(shape, _gen_square)
    waypoints = gen_func(cx, cy, alt, size, n, params)

    # Always start with a hover waypoint at takeoff position
    takeoff = [cx, cy, alt]
    if waypoints and waypoints[0] != takeoff:
        waypoints.insert(0, takeoff)

    return waypoints


def _gen_square(cx, cy, alt, size, n, params) -> List[Waypoint]:
    """Generate square trajectory."""
    half = size / 2.0
    corners = [
        [cx - half, cy - half, alt],
        [cx + half, cy - half, alt],
        [cx + half, cy + half, alt],
        [cx - half, cy + half, alt],
    ]
    return _interpolate_polygon(corners, n)


def _gen_circle(cx, cy, alt, size, n, params) -> List[Waypoint]:
    """Generate circular trajectory."""
    waypoints = []
    for i in range(n):
        angle = 2.0 * math.pi * i / n
        x = cx + size * math.cos(angle)
        y = cy + size * math.sin(angle)
        waypoints.append([x, y, alt])
    # Close the loop
    waypoints.append(waypoints[0][:])
    return waypoints


def _gen_triangle(cx, cy, alt, size, n, params) -> List[Waypoint]:
    """Generate equilateral triangle trajectory."""
    corners = []
    for i in range(3):
        angle = 2.0 * math.pi * i / 3 - math.pi / 2  # Start from top
        x = cx + size * math.cos(angle)
        y = cy + size * math.sin(angle)
        corners.append([x, y, alt])
    return _interpolate_polygon(corners, n)


def _gen_hexagon(cx, cy, alt, size, n, params) -> List[Waypoint]:
    """Generate hexagon trajectory."""
    corners = []
    for i in range(6):
        angle = 2.0 * math.pi * i / 6
        x = cx + size * math.cos(angle)
        y = cy + size * math.sin(angle)
        corners.append([x, y, alt])
    return _interpolate_polygon(corners, n)


def _gen_octagon(cx, cy, alt, size, n, params) -> List[Waypoint]:
    """Generate octagon trajectory."""
    corners = []
    for i in range(8):
        angle = 2.0 * math.pi * i / 8
        x = cx + size * math.cos(angle)
        y = cy + size * math.sin(angle)
        corners.append([x, y, alt])
    return _interpolate_polygon(corners, n)


def _gen_figure8(cx, cy, alt, size, n, params) -> List[Waypoint]:
    """Generate figure-8 (lemniscate) trajectory."""
    waypoints = []
    for i in range(n):
        t = 2.0 * math.pi * i / n
        # Lemniscate of Bernoulli parametric form
        denom = 1.0 + math.sin(t) ** 2
        x = cx + size * math.cos(t) / denom
        y = cy + size * math.sin(t) * math.cos(t) / denom
        waypoints.append([x, y, alt])
    waypoints.append(waypoints[0][:])
    return waypoints


def _gen_helix(cx, cy, alt, size, n, params) -> List[Waypoint]:
    """Generate helical (spiral) trajectory — altitude changes."""
    alt_start = -abs(params.get('altitude', 3.0))
    alt_end = -abs(params.get('altitude_end', 6.0))

    waypoints = []
    for i in range(n):
        t = float(i) / max(1, n - 1)
        angle = 2.0 * math.pi * 2 * t  # 2 full rotations
        x = cx + size * math.cos(angle)
        y = cy + size * math.sin(angle)
        z = alt_start + (alt_end - alt_start) * t
        waypoints.append([x, y, z])
    return waypoints


def _gen_star(cx, cy, alt, size, n, params) -> List[Waypoint]:
    """Generate 5-pointed star trajectory."""
    points = max(5, int(params.get('star_points', 5)))
    corners = []
    for i in range(points * 2):
        angle = math.pi * i / points - math.pi / 2
        r = size if i % 2 == 0 else size * 0.4
        x = cx + r * math.cos(angle)
        y = cy + r * math.sin(angle)
        corners.append([x, y, alt])
    corners.append(corners[0][:])
    return corners


def _gen_custom(cx, cy, alt, size, n, params) -> List[Waypoint]:
    """Handle custom waypoints passed directly from the GUI canvas."""
    custom = params.get('custom_waypoints', [])
    if custom:
        return custom
    # Fallback to square
    return _gen_square(cx, cy, alt, size, n, params)


def _interpolate_polygon(corners: List[Waypoint], total_points: int) -> List[Waypoint]:
    """Interpolate between polygon corners to reach desired point count."""
    n_sides = len(corners)
    if total_points <= n_sides:
        result = corners[:total_points]
        result.append(corners[0][:])
        return result

    points_per_side = max(1, total_points // n_sides)
    waypoints = []

    for i in range(n_sides):
        p1 = corners[i]
        p2 = corners[(i + 1) % n_sides]

        for j in range(points_per_side):
            t = float(j) / points_per_side
            x = p1[0] + (p2[0] - p1[0]) * t
            y = p1[1] + (p2[1] - p1[1]) * t
            z = p1[2] + (p2[2] - p1[2]) * t
            waypoints.append([x, y, z])

    # Close loop
    waypoints.append(corners[0][:])
    return waypoints
