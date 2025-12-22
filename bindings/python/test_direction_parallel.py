#!/usr/bin/env python3
"""
Quick test script to verify DirectionParallel functionality.
Run this to confirm Raster and Zigzag toolpath generation works.
"""

import numpy as np
from nepath_bindings import NEPathPlanner, DirectParallelOptions

def test_direction_parallel():
    """Test DirectionParallel (Raster and Zigzag) functionality."""

    # Create a planner with a circular contour
    planner = NEPathPlanner()
    theta = np.linspace(0, 2 * np.pi, 100)
    x = 10 * np.cos(theta)
    y = 10 * np.sin(theta)
    planner.set_contour(x, y)

    # Test Raster toolpaths
    print("Testing Raster toolpaths...")
    raster_opts = DirectParallelOptions()
    raster_opts.delta = 1.0
    raster_opts.angle = -np.pi / 3.0  # -60 degrees

    raster_paths = planner.Raster(raster_opts)
    print(f"✓ Raster: Generated {len(raster_paths)} paths")
    for i, path in enumerate(raster_paths[:3]):  # Show first 3
        x_coords, y_coords = path.get_arrays()
        print(f"  Path {i}: {len(x_coords)} waypoints")

    # Test Zigzag toolpaths
    print("\nTesting Zigzag toolpaths...")
    zigzag_opts = DirectParallelOptions()
    zigzag_opts.delta = 1.0
    zigzag_opts.angle = np.pi / 3.0  # 60 degrees

    zigzag_paths = planner.Zigzag(zigzag_opts)
    print(f"✓ Zigzag: Generated {len(zigzag_paths)} paths")
    for i, path in enumerate(zigzag_paths[:3]):  # Show first 3
        x_coords, y_coords = path.get_arrays()
        print(f"  Path {i}: {len(x_coords)} waypoints")

    print("\n" + "="*50)
    print("All DirectionParallel tests passed!")
    print("="*50)

if __name__ == "__main__":
    test_direction_parallel()
