#!/usr/bin/env python3
"""Stage 3 research tool: how good is the SLAM map? Compare it with the reference map.

  ros2 run warehouse_runner compare_maps                       # latest SLAM map vs prebuilt map
  ros2 run warehouse_runner compare_maps --slam ~/ros2_ws/maps/slam_map.yaml --out ~/ros2_ws/maps

Both maps use world coordinates (SLAM's map frame starts where odom starts), so cells can be
compared directly. Metrics:
  wall_accuracy   share of SLAM wall cells within 10 cm of a real wall surface (precision)
  wall_coverage   share of real wall surfaces that SLAM found within 10 cm   (recall)
  mean_error_cm   average distance from a SLAM wall cell to the nearest real wall
  free_coverage   share of the reference free floor that SLAM marked as free
Writes a PNG overlay and a JSON file next to --out.
"""
import argparse
import json
from pathlib import Path

import numpy as np
import yaml
from PIL import Image
from ament_index_python.packages import get_package_share_directory


def load(path):
    path = Path(path).expanduser()
    meta = yaml.safe_load(path.read_text())
    img = np.array(Image.open(path.parent / meta['image']).convert('L'), dtype=float)
    p = (255 - img) / 255.0 if not meta.get('negate', 0) else img / 255.0
    grid = np.full(img.shape, -1, dtype=int)           # -1 unknown, 0 free, 1 occupied
    grid[p > meta['occupied_thresh']] = 1
    grid[p < meta['free_thresh']] = 0
    return grid, meta['resolution'], meta['origin'][:2]


def cells_to_xy(mask, res, origin):
    rows, cols = np.nonzero(mask)
    h = mask.shape[0]
    return np.stack([origin[0] + (cols + 0.5) * res, origin[1] + (h - rows - 0.5) * res], axis=1)


def sample(grid, res, origin, xy):
    """Value of grid at world points xy (unknown outside the grid)."""
    h, w = grid.shape
    cols = np.floor((xy[:, 0] - origin[0]) / res).astype(int)
    rows = h - 1 - np.floor((xy[:, 1] - origin[1]) / res).astype(int)
    out = np.full(len(xy), -1)
    ok = (cols >= 0) & (cols < w) & (rows >= 0) & (rows < h)
    out[ok] = grid[rows[ok], cols[ok]]
    return out


def reachable(free, res, origin, start):
    """Free cells connected to the start point (4-neighbour flood fill)."""
    from collections import deque
    h, w = free.shape
    c0 = int((start[0] - origin[0]) / res)
    r0 = h - 1 - int((start[1] - origin[1]) / res)
    seen = np.zeros_like(free)
    q = deque([(r0, c0)])
    seen[r0, c0] = True
    while q:
        r, c = q.popleft()
        for rr, cc in ((r + 1, c), (r - 1, c), (r, c + 1), (r, c - 1)):
            if 0 <= rr < h and 0 <= cc < w and free[rr, cc] and not seen[rr, cc]:
                seen[rr, cc] = True
                q.append((rr, cc))
    return seen


def nearest(a, b):
    """For each point in a, distance to the nearest point in b (chunked brute force)."""
    best = np.full(len(a), np.inf)
    for i in range(0, len(a), 2000):
        d = np.sqrt(((a[i:i + 2000, None, :] - b[None, :, :]) ** 2).sum(-1))
        best[i:i + 2000] = d.min(1)
    return best


def main():
    ref_default = get_package_share_directory('warehouse_runner') + '/maps/warehouse.yaml'
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--slam', default='~/ros2_ws/maps/slam_map.yaml')
    ap.add_argument('--ref', default=ref_default)
    ap.add_argument('--out', default='~/ros2_ws/maps')
    args = ap.parse_args()

    ref, rres, rorg = load(args.ref)
    slam, sres, sorg = load(args.slam)
    tol = 0.10

    # Only judge what the robot could possibly see: the free floor REACHABLE from HOME (flood fill),
    # and the wall SURFACES that touch it. The outside faces of the warehouse walls don't count.
    occ = ref == 1
    free = reachable(ref == 0, rres, rorg, (0.0, -3.8))
    touch = np.zeros_like(occ)
    touch[1:, :] |= free[:-1, :]; touch[:-1, :] |= free[1:, :]
    touch[:, 1:] |= free[:, :-1]; touch[:, :-1] |= free[:, 1:]
    ref_surface = cells_to_xy(occ & touch, rres, rorg)
    slam_walls = cells_to_xy(slam == 1, sres, sorg)

    d_slam = nearest(slam_walls, ref_surface)
    d_ref = nearest(ref_surface, slam_walls)
    ref_free_xy = cells_to_xy(free, rres, rorg)
    slam_at_free = sample(slam, sres, sorg, ref_free_xy)

    result = {
        'slam_map': str(Path(args.slam).expanduser()),
        'slam_size_m': [round(slam.shape[1] * sres, 2), round(slam.shape[0] * sres, 2)],
        'slam_wall_cells': int(len(slam_walls)),
        'reference_surface_cells': int(len(ref_surface)),
        'wall_accuracy': round(float((d_slam <= tol).mean()), 3),
        'wall_coverage': round(float((d_ref <= tol).mean()), 3),
        'mean_error_cm': round(float(d_slam.mean() * 100), 1),
        'free_coverage': round(float((slam_at_free == 0).mean()), 3),
        'free_marked_occupied': round(float((slam_at_free == 1).mean()), 4),
    }

    out = Path(args.out).expanduser()
    out.mkdir(parents=True, exist_ok=True)
    (out / 'map_comparison.json').write_text(json.dumps(result, indent=2))
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(1, 2, figsize=(11, 5.2))
        def show(a, grid, res, org, title):
            img = np.where(grid == 1, 0.1, np.where(grid == 0, 1.0, 0.75))
            h, w = grid.shape
            a.imshow(img, cmap='gray', vmin=0, vmax=1,
                     extent=[org[0], org[0] + w * res, org[1], org[1] + h * res])
            a.set_xlim(-5.5, 5.5); a.set_ylim(-5.5, 5.5); a.set_title(title); a.set_xlabel('x (m)')
        show(ax[0], slam, sres, sorg, 'SLAM map (slam_toolbox)')
        ax[1].scatter(ref_surface[:, 0], ref_surface[:, 1], s=2, c='#888', label='real wall surface')
        ax[1].scatter(slam_walls[:, 0], slam_walls[:, 1], s=2, c='#e33', label='SLAM wall cells')
        ax[1].set_aspect('equal'); ax[1].set_xlim(-5.5, 5.5); ax[1].set_ylim(-5.5, 5.5)
        ax[1].set_title(f"Overlay: accuracy {result['wall_accuracy']:.0%}, coverage {result['wall_coverage']:.0%}, "
                        f"error {result['mean_error_cm']} cm", fontsize=9)
        ax[1].legend(loc='lower right', fontsize=8, markerscale=4)
        fig.tight_layout(); fig.savefig(out / 'map_comparison.png', dpi=110)
        result['figure'] = str(out / 'map_comparison.png')
    except Exception as e:  # charts are optional
        result['figure_error'] = str(e)
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
