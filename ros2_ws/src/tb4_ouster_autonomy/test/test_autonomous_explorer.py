"""Unit tests for autonomous_explorer frontier extraction and map saving."""

import os
import tempfile

from types import SimpleNamespace
import math
import yaml


from tb4_ouster_autonomy.mapping import (
    find_frontier_clusters,
    save_occupancy_grid_to_files,
)


def OccupancyGrid():
    return SimpleNamespace(info=SimpleNamespace(
        origin=SimpleNamespace(position=SimpleNamespace(x=0., y=0.),
                               orientation=SimpleNamespace(x=0., y=0., z=0., w=1.))
    ), data=[])


def _make_grid(width: int, height: int, data: list, resolution: float = 0.05) -> OccupancyGrid:
    grid = OccupancyGrid()
    grid.info.width = width
    grid.info.height = height
    grid.info.resolution = resolution
    grid.info.origin.position.x = -1.0
    grid.info.origin.position.y = -1.0
    grid.info.origin.orientation.w = 1.0
    grid.data = data
    return grid


def test_find_frontier_clusters_detects_boundary():
    """Verify frontier cluster extraction between free (0) and unknown (-1) regions."""
    w, h = 20, 20
    data = [-1] * (w * h)
    # Mark lower half (rows 2..10, cols 2..17) as free (0)
    for r in range(2, 11):
        for c in range(2, 18):
            data[r * w + c] = 0

    grid = _make_grid(w, h, data, resolution=0.10)
    clusters = find_frontier_clusters(grid, min_cluster_size=5, clearance_cells=1)
    assert len(clusters) >= 1
    total_cells = sum(c[2] for c in clusters)
    assert total_cells >= 10


def test_find_frontier_clusters_respects_obstacle_clearance():
    """Verify frontier cells adjacent to obstacles (>=50) are filtered out."""
    w, h = 15, 15
    data = [-1] * (w * h)
    for r in range(3, 8):
        for c in range(3, 12):
            data[r * w + c] = 0
    # Put an obstacle wall right along the frontier boundary at row 8
    for c in range(2, 13):
        data[8 * w + c] = 100

    grid = _make_grid(w, h, data, resolution=0.05)
    clusters_strict = find_frontier_clusters(grid, min_cluster_size=4, clearance_cells=6)
    assert len(clusters_strict) == 0


def test_save_occupancy_grid_to_files_writes_valid_pgm_and_yaml():
    """Verify deterministic .pgm and .yaml export for Ctrl+C/completion map saving."""
    w, h = 10, 8
    data = [-1] * (w * h)
    data[0] = 0
    data[1] = 100

    grid = _make_grid(w, h, data, resolution=0.05)
    with tempfile.TemporaryDirectory() as tmpdir:
        prefix = os.path.join(tmpdir, "test_arena")
        pgm_path, yaml_path = save_occupancy_grid_to_files(grid, prefix)
        assert os.path.isfile(pgm_path)
        assert os.path.isfile(yaml_path)

        with open(yaml_path, "r", encoding="utf-8") as yf:
            yaml_text = yf.read()
        assert yaml.safe_load(yaml_text)["image"] == "test_arena.pgm"
        assert yaml.safe_load(yaml_text)["resolution"] == 0.05

        with open(pgm_path, "rb") as pf:
            raw = pf.read()
        assert raw.startswith(b"P5\n10 8\n255\n")
        pixel_payload = raw.split(b"255\n", 1)[1]
        assert len(pixel_payload) == w * h


def test_rotated_map_origin_rotates_frontier_coordinates():
    data = [-1] * 100
    for r in range(2, 8):
        for c in range(2, 8):
            data[r*10+c] = 0
    grid = _make_grid(10, 10, data, 0.1)
    original = find_frontier_clusters(grid, min_cluster_size=1, clearance_cells=0)
    grid.info.origin.orientation.z = math.sin(math.pi/4)
    grid.info.origin.orientation.w = math.cos(math.pi/4)
    rotated = find_frontier_clusters(grid, min_cluster_size=1, clearance_cells=0)
    for (x, y, _), (rx, ry, _) in zip(original, rotated):
        assert abs(rx - (-1 - (y + 1))) < 1e-6
        assert abs(ry - (-1 + (x + 1))) < 1e-6
