"""Pure frontier extraction and map serialization; accepts OccupancyGrid-shaped data."""

from collections import deque
import math
import os
from typing import List, Tuple


def quaternion_to_yaw(q) -> float:
    """Convert geometry_msgs/Quaternion to yaw (radians)."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    """Convert yaw (radians) to (x, y, z, w) quaternion tuple."""
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def save_occupancy_grid_to_files(grid, map_prefix: str) -> Tuple[str, str]:
    """Save OccupancyGrid directly to standard Nav2 trinary .pgm and .yaml files.

    File IO may fail; callers must report errors and retain the previous map.
    """
    width = int(grid.info.width)
    height = int(grid.info.height)
    resolution = float(grid.info.resolution)
    origin_x = float(grid.info.origin.position.x)
    origin_y = float(grid.info.origin.position.y)
    origin_yaw = quaternion_to_yaw(grid.info.origin.orientation)

    if width <= 0 or height <= 0 or len(grid.data) != width * height or resolution <= 0:
        raise ValueError('Malformed OccupancyGrid')
    parent_dir = os.path.dirname(os.path.abspath(map_prefix))
    if parent_dir:
        os.makedirs(parent_dir, exist_ok=True)

    pgm_path = f"{map_prefix}.pgm"
    yaml_path = f"{map_prefix}.yaml"
    pgm_basename = os.path.basename(pgm_path)

    data = grid.data
    pixel_bytes = bytearray(width * height)

    # OccupancyGrid row 0 is bottom (origin_y); PGM row 0 is top
    for row in range(height):
        src_row = height - 1 - row
        src_offset = src_row * width
        dst_offset = row * width
        for col in range(width):
            val = data[src_offset + col]
            if val < 0:
                pixel_bytes[dst_offset + col] = 205  # Unknown (gray)
            elif val >= 65:
                pixel_bytes[dst_offset + col] = 0    # Occupied (black)
            elif val <= 25:
                pixel_bytes[dst_offset + col] = 254  # Free (white)
            else:
                pixel_bytes[dst_offset + col] = 205

    with open(pgm_path, "wb") as f:
        header = f"P5\n{width} {height}\n255\n".encode("ascii")
        f.write(header)
        f.write(pixel_bytes)

    yaml_content = (
        f"image: {pgm_basename!r}\n"
        f"mode: trinary\n"
        f"resolution: {resolution:.12g}\n"
        f"origin: [{origin_x:.12g}, {origin_y:.12g}, {origin_yaw:.12g}]\n"
        f"negate: 0\n"
        f"occupied_thresh: 0.65\n"
        f"free_thresh: 0.25\n"
    )
    with open(yaml_path, "w", encoding="utf-8") as f:
        f.write(yaml_content)

    return pgm_path, yaml_path


def find_frontier_clusters(
    grid,
    min_cluster_size: int = 6,
    clearance_cells: int = 5,
) -> List[Tuple[float, float, int]]:
    """Extract frontier cluster centroids (x_m, y_m, cell_count) from an OccupancyGrid.

    A frontier cell is a known free cell (0 <= v <= 25) that has at least one
    4-connected unknown neighbor (v == -1) and no occupied cell (v >= 50)
    within `clearance_cells` radius.
    """
    width = int(grid.info.width)
    height = int(grid.info.height)
    res = float(grid.info.resolution)
    ox = float(grid.info.origin.position.x)
    oy = float(grid.info.origin.position.y)
    origin_yaw = quaternion_to_yaw(grid.info.origin.orientation)
    data = grid.data

    if width <= 2 or height <= 2 or len(data) < width * height:
        return []

    # 1. Identify candidate frontier cells
    is_frontier = bytearray(width * height)
    for r in range(1, height - 1):
        row_off = r * width
        for c in range(1, width - 1):
            idx = row_off + c
            val = data[idx]
            if 0 <= val <= 25:
                # Check 4-neighbors for unknown (-1)
                if (
                    data[idx - 1] == -1
                    or data[idx + 1] == -1
                    or data[idx - width] == -1
                    or data[idx + width] == -1
                ):
                    is_frontier[idx] = 1

    # 2. Filter out frontier cells too close to obstacles
    if clearance_cells > 0:
        for r in range(height):
            row_off = r * width
            for c in range(width):
                if data[row_off + c] >= 50:
                    r_min = max(0, r - clearance_cells)
                    r_max = min(height - 1, r + clearance_cells)
                    c_min = max(0, c - clearance_cells)
                    c_max = min(width - 1, c + clearance_cells)
                    for nr in range(r_min, r_max + 1):
                        noff = nr * width
                        for nc in range(c_min, c_max + 1):
                            is_frontier[noff + nc] = 0

    # 3. 8-connected BFS clustering
    visited = bytearray(width * height)
    clusters: List[Tuple[float, float, int]] = []

    for r in range(1, height - 1):
        row_off = r * width
        for c in range(1, width - 1):
            start_idx = row_off + c
            if is_frontier[start_idx] and not visited[start_idx]:
                queue = deque([(r, c)])
                visited[start_idx] = 1
                cells = []
                while queue:
                    cr, cc = queue.popleft()
                    cells.append((cr, cc))
                    for dr in (-1, 0, 1):
                        for dc in (-1, 0, 1):
                            if dr == 0 and dc == 0:
                                continue
                            nr, nc = cr + dr, cc + dc
                            if 0 <= nr < height and 0 <= nc < width:
                                nidx = nr * width + nc
                                if is_frontier[nidx] and not visited[nidx]:
                                    visited[nidx] = 1
                                    queue.append((nr, nc))

                if len(cells) >= min_cluster_size:
                    # Pick the cluster cell closest to the cluster median (ensures goal lies on a valid free cell)
                    mean_r = sum(p[0] for p in cells) / len(cells)
                    mean_c = sum(p[1] for p in cells) / len(cells)
                    best_r, best_c = min(
                        cells,
                        key=lambda p: (p[0] - mean_r) ** 2 + (p[1] - mean_c) ** 2,
                    )
                    lx, ly = (best_c + 0.5) * res, (best_r + 0.5) * res
                    wx = ox + math.cos(origin_yaw)*lx - math.sin(origin_yaw)*ly
                    wy = oy + math.sin(origin_yaw)*lx + math.cos(origin_yaw)*ly
                    clusters.append((wx, wy, len(cells)))

    return clusters
