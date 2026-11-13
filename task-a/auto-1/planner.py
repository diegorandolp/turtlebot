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
                 occ_thr=90, free_thr=150, min_frontier_cells=2, wall_pref_m=0.50, wall_pref_w=3.0,
                 frontera="cercana", rama_peso=1.0):
        self.res = size_m / pixels               # m por pixel del mapa
        self.n = pixels
        self.f = cell                            # pixeles del mapa por celda de planificacion
        self.cres = self.res * cell              # m por celda
        self.m = pixels // cell
        # redondeo al entero MAS CERCANO (no hacia arriba): con celdas de 8 cm, 0.25 m daba 4 celdas = 0.32 m por
        # lado y cerraba pasillos de ~0.73 m (laberinto); asi son 3 celdas = 0.24 m (robot real: 0.17 m de radio)
        self.infl = max(1, int(round((robot_radius + margin) / self.cres)))
        self.occ_thr, self.free_thr = occ_thr, free_thr
        self.min_fr = min_frontier_cells
        self.pref_cells = wall_pref_m / self.cres
        self.pref_w = wall_pref_w
        # eleccion de la siguiente frontera:
        #   "cercana": la alcanzable mas cercana (version original; en caminos con dos ramas hace "ping-pong")
        #   "rama":    camino + rama_peso * distancia a la ULTIMA META alcanzada: termina la rama en la que esta
        #              antes de cruzar a otra
        if frontera not in ("cercana", "rama"):
            raise ValueError(f"frontera desconocida: {frontera}")
        self.frontera, self.rama_peso = frontera, rama_peso

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
        # frontera: celda libre con un vecino desconocido EN CRUZ (no en diagonal, para no tomar por puerta lo
        # desconocido detras de una pared fina). NO se exige que la frontera sea transitable: en un pasillo estrecho
        # la puerta entera cae dentro del margen de seguridad de sus bordes y nunca seria un destino. La meta es la
        # celda transitable mas cercana a la frontera; desde ahi el LiDAR ya ve a traves de la puerta.
        cruz = np.array([[0, 1, 0], [1, 1, 1], [0, 1, 0]], np.uint8)
        unk_d = cv2.dilate(unk.astype(np.uint8), cruz).astype(bool)
        fr = unk_d & free
        n, lab, stats, _ = cv2.connectedComponentsWithStats(fr.astype(np.uint8), connectivity=8)
        fr_ok = np.zeros_like(fr)
        for i in range(1, n):
            if stats[i, cv2.CC_STAT_AREA] >= self.min_fr:
                fr_ok |= lab == i
        k2 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * self.infl + 3, 2 * self.infl + 3))
        valid = cv2.dilate(fr_ok.astype(np.uint8), k2).astype(bool) & trav        # metas: transitables junto a una frontera
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

    def _result(self, path, valid, t0, import_time):
        pts = [self.to_xy(r, c) for r, c in path]
        fr = np.argwhere(valid)
        fxy = np.array([self.to_xy(r, c) for r, c in fr]) if len(fr) else np.zeros((0, 2))
        return PlanResult("ok", pts, pts[-1], fxy, int(valid.sum()), (import_time() - t0) * 1000)

    def _flood(self, trav, extra, start):
        """Dijkstra completo desde `start` (sin parar en ninguna meta): distancias y padres de todas las celdas."""
        m = self.m
        dist = np.full((m, m), np.inf)
        parent = {}
        dist[start] = 0.0
        pq = [(0.0, start)]
        while pq:
            d, (r, c) = heapq.heappop(pq)
            if d > dist[r, c]:
                continue
            for dr, dc, w in NEIGH:
                nr, nc = r + dr, c + dc
                if 0 <= nr < m and 0 <= nc < m and trav[nr, nc]:
                    if dr and dc and not (trav[r, nc] and trav[nr, c]):
                        continue
                    nd = d + w * (1.0 + extra[nr, nc])
                    if nd < dist[nr, nc]:
                        dist[nr, nc] = nd
                        parent[(nr, nc)] = (r, c)
                        heapq.heappush(pq, (nd, (nr, nc)))
        return dist, parent

    def _path_rama(self, trav, extra, valid, start, anchor):
        """Frontera que minimiza camino + rama_peso * distancia a `anchor` (la ultima meta alcanzada)."""
        dist, parent = self._flood(trav, extra, start)
        cand = np.argwhere(valid & np.isfinite(dist))
        if not len(cand):
            return None
        ax, ay = anchor
        xs, ys = self.to_xy(cand[:, 0], cand[:, 1])
        score = dist[cand[:, 0], cand[:, 1]] * self.cres + self.rama_peso * np.hypot(xs - ax, ys - ay)
        goal = tuple(int(v) for v in cand[int(np.argmin(score))])
        path = [goal]
        while path[-1] != start:
            path.append(parent[path[-1]])
        return path[::-1]

    def plan_explore(self, img, start_xy, prev_goal=None, bad_goals=(), anchor=None):
        """Ruta a la siguiente frontera (o a la anterior si sigue siendo frontera). Con frontera="rama" y un
        `anchor` (ultima meta alcanzada), prefiere las fronteras de la misma zona; si no, la mas cercana."""
        import time
        t0 = time.perf_counter()
        trav, extra, valid, start = self.analyse(img, start_xy, bad_goals)
        if not valid.any():
            return PlanResult("no_frontier", ms=(time.perf_counter() - t0) * 1000)
        path = None
        if prev_goal is not None:                                   # histeresis: no cambiar de meta sin motivo
            gr, gc = self.to_cell(*prev_goal)
            near = valid[max(gr - 2, 0):gr + 3, max(gc - 2, 0):gc + 3]
            if near.any():
                path = self._dijkstra(trav, extra, start, lambda r, c: abs(r - gr) <= 2 and abs(c - gc) <= 2 and valid[r, c])
        if path is None and self.frontera == "rama" and anchor is not None:
            path = self._path_rama(trav, extra, valid, start, anchor)
        if path is None:
            path = self._dijkstra(trav, extra, start, lambda r, c: valid[r, c])
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
