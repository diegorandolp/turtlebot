"""
Planificador de exploracion por fronteras (Tarea A, auto-1). Sin dependencias del robot.

Entrada: el mapa de ocupacion del SLAM (uint8: 0 = pared, 127 = desconocido, ~255 = libre; fila = y,
columna = x, origen en el centro; ver task-a/slam-1/slam_core.py).

Idea: una "frontera" es una celda libre pegada a una zona desconocida. Ir a la frontera alcanzable
mas cercana revela mapa nuevo; repetir hasta ver el rostro (punto B) o hasta que no queden fronteras.
La ruta se calcula con Dijkstra sobre celdas libres, con las paredes "infladas" por el radio del
robot y un coste extra cerca de ellas (la ruta tiende a ir por el centro de los pasillos).
"""
import heapq
import math
from dataclasses import dataclass, field

import cv2
import numpy as np

OCC, UNK, FREE = 0, 1, 2
SQ2 = math.sqrt(2.0)
NEIGH = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
         (-1, -1, SQ2), (-1, 1, SQ2), (1, -1, SQ2), (1, 1, SQ2)]


@dataclass
class PlanResult:
    status: str                         # "ok" | "no_frontier" | "no_path"
    path: list = field(default_factory=list)       # [(x, y)] en m (marco del mapa), del robot a la meta
    goal: tuple | None = None           # (x, y) m
    frontier_xy: np.ndarray | None = None          # (k, 2) m de las celdas frontera validas (para dibujar)
    n_frontier_cells: int = 0
    ms: float = 0.0


class Planner:
    def __init__(self, size_m=20.0, pixels=500, cell=2, robot_radius=0.20, margin=0.05,
                 occ_thr=90, free_thr=150, min_frontier_cells=4, wall_pref_m=0.50, wall_pref_w=3.0,
                 gain_w=0.0, horizon_m=2.0, keep_bonus_m=1.0):
        self.res = size_m / pixels               # m por pixel del mapa
        self.n = pixels
        self.f = cell                            # pixeles del mapa por celda de planificacion
        self.cres = self.res * cell              # m por celda
        self.m = pixels // cell
        self.infl = max(1, int(math.ceil((robot_radius + margin) / self.cres)))
        self.occ_thr, self.free_thr = occ_thr, free_thr
        self.min_fr = min_frontier_cells
        self.pref_cells = wall_pref_m / self.cres
        self.pref_w = wall_pref_w
        self.gain_w, self.horizon, self.keep_bonus = gain_w, horizon_m / self.cres, keep_bonus_m / self.cres

    # ---- conversiones ----
    def to_cell(self, x, y):
        return (int((self.n / 2 + y / self.res) // self.f), int((self.n / 2 + x / self.res) // self.f))   # (fila, col)

    def to_xy(self, r, c):
        return ((c + 0.5) * self.f - self.n / 2) * self.res, ((r + 0.5) * self.f - self.n / 2) * self.res

    # ---- rejilla de planificacion ----
    def grid(self, img):
        """-> (occ, unk, free) booleanos de m x m, por bloques de f x f pixeles."""
        m, f = self.m, self.f
        v = img[:m * f, :m * f].reshape(m, f, m, f)
        occ = (v < self.occ_thr).any(axis=(1, 3))
        free_all = (v >= self.free_thr).all(axis=(1, 3))
        free = free_all & ~occ
        unk = ~occ & ~free
        return occ, unk, free

    def analyse(self, img, start_xy, bad_goals=()):
        occ, unk, free = self.grid(img)
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * self.infl + 1, 2 * self.infl + 1))
        occ_i = cv2.dilate(occ.astype(np.uint8), k).astype(bool)
        sr, sc = self.to_cell(*start_xy)
        rr, cc = np.ogrid[:self.m, :self.m]
        occ_i &= ~(((rr - sr) ** 2 + (cc - sc) ** 2) <= (0.30 / self.cres) ** 2)    # el robot siempre puede salir
        trav = (free | ((rr - sr) ** 2 + (cc - sc) ** 2 <= (0.30 / self.cres) ** 2)) & ~occ_i & ~occ
        near = cv2.distanceTransform((~occ).astype(np.uint8), cv2.DIST_L2, 3)       # celdas hasta una pared
        extra = np.clip((self.pref_cells - near) / max(self.pref_cells, 1e-6), 0, 1) * self.pref_w
        # frontera: transitable con vecino desconocido
        unk_d = cv2.dilate(unk.astype(np.uint8), np.ones((3, 3), np.uint8)).astype(bool)
        fr = trav & unk_d & free
        n, lab, stats, _ = cv2.connectedComponentsWithStats(fr.astype(np.uint8), connectivity=8)
        valid = np.zeros_like(fr)
        for i in range(1, n):
            if stats[i, cv2.CC_STAT_AREA] >= self.min_fr:
                valid |= lab == i
        for bx, by in bad_goals:                          # metas descartadas (inalcanzables / ya visitadas)
            r, c = self.to_cell(bx, by)
            r0, r1, c0, c1 = max(r - 3, 0), min(r + 4, self.m), max(c - 3, 0), min(c + 4, self.m)
            valid[r0:r1, c0:c1] = False
        return trav, extra, valid, (sr, sc)

    # ---- Dijkstra ----
    def _dijkstra(self, trav, extra, start, is_goal):
        m = self.m
        dist = np.full((m, m), np.inf)
        parent = {}
        dist[start] = 0.0
        pq = [(0.0, start)]
        while pq:
            d, (r, c) = heapq.heappop(pq)
            if d > dist[r, c]:
                continue
            if is_goal(r, c):
                path = [(r, c)]
                while path[-1] != start:
                    path.append(parent[path[-1]])
                return path[::-1]
            for dr, dc, w in NEIGH:
                nr, nc = r + dr, c + dc
                if 0 <= nr < m and 0 <= nc < m and trav[nr, nc]:
                    if dr and dc and not (trav[r, nc] and trav[nr, c]):      # no cortar esquinas
                        continue
                    nd = d + w * (1.0 + extra[nr, nc])
                    if nd < dist[nr, nc]:
                        dist[nr, nc] = nd
                        parent[(nr, nc)] = (r, c)
                        heapq.heappush(pq, (nd, (nr, nc)))
        return None

    def _best_frontier(self, trav, extra, start, valid, prev_goal):
        """Dijkstra hasta la frontera mas cercana y un poco mas alla (horizonte); entre los grupos de
        frontera alcanzados elige el de mejor `costo - ganancia`: un grupo grande revela mas mapa que
        uno diminuto un poco mas cerca. Devuelve la ruta o None."""
        m = self.m
        _, lab, stats, _ = cv2.connectedComponentsWithStats(valid.astype(np.uint8), connectivity=8)
        dist = np.full((m, m), np.inf)
        parent = {}
        dist[start] = 0.0
        pq = [(0.0, start)]
        first = None
        reached = {}                                         # grupo -> (costo, celda) de su celda mas cercana
        while pq:
            d, (r, c) = heapq.heappop(pq)
            if d > dist[r, c]:
                continue
            if first is not None and d > first + self.horizon:
                break
            if valid[r, c]:
                first = d if first is None else first
                reached.setdefault(lab[r, c], (d, (r, c)))
            for dr, dc, w in NEIGH:
                nr, nc = r + dr, c + dc
                if 0 <= nr < m and 0 <= nc < m and trav[nr, nc]:
                    if dr and dc and not (trav[r, nc] and trav[nr, c]):      # no cortar esquinas
                        continue
                    nd = d + w * (1.0 + extra[nr, nc])
                    if nd < dist[nr, nc]:
                        dist[nr, nc] = nd
                        parent[(nr, nc)] = (r, c)
                        heapq.heappush(pq, (nd, (nr, nc)))
        if not reached:
            return None
        pg = self.to_cell(*prev_goal) if prev_goal is not None else None
        best, best_score = None, math.inf
        for g, (d, cell) in reached.items():
            score = d - self.gain_w * stats[g, cv2.CC_STAT_AREA]      # costo y ganancia, ambos en celdas
            if pg is not None and abs(cell[0] - pg[0]) <= 2 and abs(cell[1] - pg[1]) <= 2:
                score -= self.keep_bonus                                     # histeresis: no cambiar de meta sin motivo
            if score < best_score:
                best, best_score = cell, score
        path = [best]
        while path[-1] != start:
            path.append(parent[path[-1]])
        return path[::-1]

    def _result(self, path, valid, t0, import_time):
        pts = [self.to_xy(r, c) for r, c in path]
        fr = np.argwhere(valid)
        fxy = np.array([self.to_xy(r, c) for r, c in fr]) if len(fr) else np.zeros((0, 2))
        return PlanResult("ok", pts, pts[-1], fxy, int(valid.sum()), (import_time() - t0) * 1000)

    def plan_explore(self, img, start_xy, prev_goal=None, bad_goals=()):
        """Ruta a la frontera alcanzable mas cercana (o a la anterior si sigue siendo frontera)."""
        import time
        t0 = time.perf_counter()
        trav, extra, valid, start = self.analyse(img, start_xy, bad_goals)
        if not valid.any():
            return PlanResult("no_frontier", ms=(time.perf_counter() - t0) * 1000)
        path = self._best_frontier(trav, extra, start, valid, prev_goal)
        if path is None:
            return PlanResult("no_path", frontier_xy=np.array([self.to_xy(r, c) for r, c in np.argwhere(valid)]),
                              n_frontier_cells=int(valid.sum()), ms=(time.perf_counter() - t0) * 1000)
        return self._result(path, valid, t0, time.perf_counter)

    def plan_to(self, img, start_xy, target_xy):
        """Ruta a un punto concreto (p. ej. volver al punto A). La meta se ajusta a la celda libre mas cercana."""
        import time
        t0 = time.perf_counter()
        trav, extra, _, start = self.analyse(img, start_xy)
        tr, tc = self.to_cell(*target_xy)
        path = self._dijkstra(trav, extra, start, lambda r, c: abs(r - tr) <= 2 and abs(c - tc) <= 2)
        if path is None:
            return PlanResult("no_path", ms=(time.perf_counter() - t0) * 1000)
        return self._result(path, np.zeros_like(trav), t0, time.perf_counter)
