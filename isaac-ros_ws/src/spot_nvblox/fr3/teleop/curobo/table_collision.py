"""Pure helpers for composing a static table cuboid with an ESDF voxel world.

Table extents supplied from the current ZED1 mesh are provisional. Callers must
keep this overlay disabled until tabletop bounds are checked in the lab.
"""
from math import isfinite


def build_world_config_data(grid_size_m, voxel_size_m, table_bounds_m=None):
    """Return cuRobo v0.7 WorldConfig data with optional table bounds.

    `table_bounds_m` is [xmin, xmax, ymin, ymax, z_top, thickness] in the MPC
    base frame. Voxel ESDF remains the dynamic scene; cuboid adds a fixed table.
    """
    dims = [float(v) for v in grid_size_m]
    voxel_size = float(voxel_size_m)
    if len(dims) != 3 or not all(isfinite(v) and v > 0 for v in dims):
        raise ValueError("grid_size_m must contain three positive finite values")
    if not isfinite(voxel_size) or voxel_size <= 0:
        raise ValueError("voxel_size_m must be positive and finite")

    world = {
        "voxel": {
            "world_voxel": {
                "dims": dims,
                "pose": [0, 0, 0, 1, 0, 0, 0],
                "voxel_size": voxel_size,
            }
        }
    }
    if table_bounds_m is None:
        return world

    bounds = [float(v) for v in table_bounds_m]
    if len(bounds) != 6 or not all(isfinite(v) for v in bounds):
        raise ValueError("table bounds must be six finite values")
    xmin, xmax, ymin, ymax, z_top, thickness = bounds
    if xmin >= xmax or ymin >= ymax or thickness <= 0:
        raise ValueError("table bounds must have positive X/Y extents and thickness")
    world["cuboid"] = {
        "fr3_table": {
            "dims": [xmax - xmin, ymax - ymin, thickness],
            "pose": [
                0.5 * (xmin + xmax),
                0.5 * (ymin + ymax),
                z_top - 0.5 * thickness,
                1,
                0,
                0,
                0,
            ],
        }
    }
    return world
