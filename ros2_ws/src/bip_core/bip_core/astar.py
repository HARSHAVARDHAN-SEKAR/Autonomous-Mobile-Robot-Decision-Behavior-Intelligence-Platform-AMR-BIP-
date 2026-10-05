"""8-connected A* with octile heuristic + shortcut smoothing."""
import heapq
import math

SQRT2 = math.sqrt(2.0)
_NEIGH = [(-1, -1, SQRT2), (-1, 0, 1.0), (-1, 1, SQRT2),
          (0, -1, 1.0), (0, 1, 1.0),
          (1, -1, SQRT2), (1, 0, 1.0), (1, 1, SQRT2)]


def _octile(a, b):
    dx, dy = abs(a[0] - b[0]), abs(a[1] - b[1])
    return (dx + dy) + (SQRT2 - 2.0) * min(dx, dy)


def _nearest_free(occ, cell, max_r=8):
    """If cell is occupied (e.g. due to inflation), find nearest free cell."""
    h, w = occ.shape
    cx, cy = cell
    if 0 <= cx < w and 0 <= cy < h and not occ[cy, cx]:
        return cell
    for r in range(1, max_r + 1):
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if max(abs(dx), abs(dy)) != r:
                    continue
                nx, ny = cx + dx, cy + dy
                if 0 <= nx < w and 0 <= ny < h and not occ[ny, nx]:
                    return (nx, ny)
    return None


def plan(occ, start, goal):
    """A* on a boolean/uint8 occupancy array (h, w). start/goal = (cx, cy).
    Returns list of cells [(cx, cy), ...] or None."""
    h, w = occ.shape
    start = _nearest_free(occ, start)
    goal = _nearest_free(occ, goal)
    if start is None or goal is None:
        return None

    g = {start: 0.0}
    parent = {start: None}
    pq = [(_octile(start, goal), start)]
    closed = set()
    while pq:
        _, cur = heapq.heappop(pq)
        if cur in closed:
            continue
        if cur == goal:
            path = []
            while cur is not None:
                path.append(cur)
                cur = parent[cur]
            return path[::-1]
        closed.add(cur)
        cx, cy = cur
        for dx, dy, cost in _NEIGH:
            nx, ny = cx + dx, cy + dy
            if not (0 <= nx < w and 0 <= ny < h) or occ[ny, nx]:
                continue
            # forbid diagonal corner-cutting
            if dx and dy and (occ[cy, nx] or occ[ny, cx]):
                continue
            ng = g[cur] + cost
            n = (nx, ny)
            if ng < g.get(n, float("inf")):
                g[n] = ng
                parent[n] = cur
                heapq.heappush(pq, (ng + _octile(n, goal), n))
    return None


def line_free(occ, a, b):
    """Bresenham collision check between cells a and b."""
    x0, y0 = a
    x1, y1 = b
    dx, dy = abs(x1 - x0), abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    while True:
        if occ[y0, x0]:
            return False
        if (x0, y0) == (x1, y1):
            return True
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy


def smooth(occ, path):
    """Greedy shortcut smoothing: keep only necessary waypoints."""
    if not path or len(path) < 3:
        return path
    out = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and not line_free(occ, path[i], path[j]):
            j -= 1
        out.append(path[j])
        i = j
    return out
