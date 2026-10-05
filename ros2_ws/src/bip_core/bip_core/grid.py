"""Occupancy grid built from axis-aligned rectangular obstacles."""
import math
import numpy as np


class OccupancyGrid:
    def __init__(self, x_min, x_max, y_min, y_max, resolution):
        self.x_min, self.x_max = float(x_min), float(x_max)
        self.y_min, self.y_max = float(y_min), float(y_max)
        self.res = float(resolution)
        self.w = int(round((self.x_max - self.x_min) / self.res))
        self.h = int(round((self.y_max - self.y_min) / self.res))
        self.grid = np.zeros((self.h, self.w), dtype=np.uint8)  # 0 free, 1 occupied

    # ---- coordinate transforms -------------------------------------------
    def world_to_cell(self, x, y):
        cx = int((x - self.x_min) / self.res)
        cy = int((y - self.y_min) / self.res)
        return cx, cy

    def cell_to_world(self, cx, cy):
        return (self.x_min + (cx + 0.5) * self.res,
                self.y_min + (cy + 0.5) * self.res)

    def in_bounds(self, cx, cy):
        return 0 <= cx < self.w and 0 <= cy < self.h

    # ---- obstacle handling -----------------------------------------------
    def add_rect(self, cx_world, cy_world, size_x, size_y):
        """Mark a rectangle centered at (cx_world, cy_world) as occupied."""
        x0, y0 = self.world_to_cell(cx_world - size_x / 2.0, cy_world - size_y / 2.0)
        x1, y1 = self.world_to_cell(cx_world + size_x / 2.0, cy_world + size_y / 2.0)
        x0, y0 = max(0, x0), max(0, y0)
        x1, y1 = min(self.w - 1, x1), min(self.h - 1, y1)
        if x1 >= x0 and y1 >= y0:
            self.grid[y0:y1 + 1, x0:x1 + 1] = 1

    def inflated(self, radius_m):
        """Return a copy inflated by radius_m (binary dilation, square kernel)."""
        r = max(0, int(math.ceil(radius_m / self.res)))
        if r == 0:
            return self.grid.copy()
        occ = self.grid.astype(bool)
        out = occ.copy()
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if dx * dx + dy * dy > r * r:
                    continue
                shifted = np.zeros_like(occ)
                ys = slice(max(0, dy), min(self.h, self.h + dy))
                yd = slice(max(0, -dy), min(self.h, self.h - dy))
                xs = slice(max(0, dx), min(self.w, self.w + dx))
                xd = slice(max(0, -dx), min(self.w, self.w - dx))
                shifted[yd, xd] = occ[ys, xs]
                out |= shifted
        return out.astype(np.uint8)


def grid_from_config(cfg):
    """Build a grid from a dict: {bounds: {...}, resolution, obstacles: [...]}"""
    b = cfg["bounds"]
    g = OccupancyGrid(b["x_min"], b["x_max"], b["y_min"], b["y_max"],
                      cfg.get("resolution", 0.1))
    for ob in cfg.get("obstacles", []):
        g.add_rect(ob["x"], ob["y"], ob["size_x"], ob["size_y"])
    return g
