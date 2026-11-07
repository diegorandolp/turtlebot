"""
Cobertura de la CAMARA (Tarea A, auto-1). Sin dependencias del robot.

El LiDAR ve 360 grados, pero la camara solo ve un cono hacia delante y a poca distancia. Saber que una
zona esta en el mapa NO significa que la camara la haya mirado. Este modulo lleva la cuenta de lo
que la camara ya vio y de lo que falta:

  - `seen`: rejilla (misma que la del planificador) con las celdas que cayeron dentro del cono de
    alguna foto. El cono se recorta con el LiDAR: no se "ve" a traves de una pared.
  - "superficies": celdas ocupadas pegadas a espacio libre (paredes, cajas, soportes). Ahi es donde
    puede estar la fotografia; tienen mucho mas peso que el espacio libre.
  - gain(): cuanto de lo pendiente veria la camara apuntando en cierta direccion (con el scan real).
  - plan_view(): el mejor punto de vista alcanzable para seguir cubriendo (next-best-view).

El rango del cono (`rng`) es corto a proposito: la foto de la camara es de 250x250 px y una cara
impresa pequena solo ocupa los ~24 px que pide el detector a poco mas de un metro.
"""
import math

import cv2
import numpy as np

HEADINGS = 8                      # rumbos candidatos (cada 45 grados)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Coverage:
    def __init__(self, planner, fov_deg=60.0, rng=1.5, w_surface=3.0, w_free=0.02, min_gain=6.0):
        self.pl = planner
        self.fov = math.radians(fov_deg)
        self.rng = rng
        self.ws, self.wf = w_surface, w_free
        self.min_gain = min_gain
        self.seen = np.zeros((planner.m, planner.m), bool)
        self.n_marks = 0
        self.bad_views = []                                 # (x, y) puntos de vista ya visitados
        self._t = np.arange(0.0, rng + planner.cres, planner.cres * 0.75)
        R = int(math.ceil(rng / planner.cres))
        self._R = R
        self._kern = [self._cone_kernel(k * 2 * math.pi / HEADINGS) for k in range(HEADINGS)]

    # ---------- geometria del cono ----------
    def _cone_kernel(self, heading):
        R = self._R
        dy, dx = np.mgrid[-R:R + 1, -R:R + 1]
        dist = np.hypot(dx, dy) * self.pl.cres
        ang = np.arctan2(dy, dx)
        diff = np.abs(np.arctan2(np.sin(ang - heading), np.cos(ang - heading)))
        return ((dist <= self.rng) & (diff <= self.fov / 2) & (dist > 0)).astype(np.float32)

    def _cells(self, px, py):
        c = ((self.pl.n / 2 + px / self.pl.res) // self.pl.f).astype(int)
        r = ((self.pl.n / 2 + py / self.pl.res) // self.pl.f).astype(int)
        ok = (c >= 0) & (c < self.pl.m) & (r >= 0) & (r < self.pl.m)
        return r, c, ok

    @staticmethod
    def _range_at(scan, a):
        """Distancia del LiDAR en los angulos `a` (rad, marco del robot). inf = sin lectura."""
        ang = np.arctan2(np.sin(scan.angles), np.cos(scan.angles))
        r = np.asarray(scan.ranges, dtype=float).copy()
        r[np.isnan(r) | (r < 0.12)] = np.inf
        d = np.abs(np.arctan2(np.sin(a[:, None] - ang[None, :]), np.cos(a[:, None] - ang[None, :])))
        return r[d.argmin(axis=1)]

    def _scan_cone(self, pos, scan, rel, n=31):
        """Celdas (r, c) que la camara veria con el scan real: del robot hasta la pared o hasta `rng`."""
        x, y, th = pos
        a_rel = rel + np.linspace(-self.fov / 2, self.fov / 2, n)
        d = np.minimum(self._range_at(scan, a_rel), self.rng)
        ang = th + a_rel
        t = self._t[None, :]
        px, py = x + np.cos(ang)[:, None] * t, y + np.sin(ang)[:, None] * t
        r, c, ok = self._cells(px, py)
        ok &= t <= (d[:, None] + self.pl.cres)                # incluye la celda de la pared
        return r[ok], c[ok]

    # ---------- marcar lo visto ----------
    def mark(self, pos, scan, rel=0.0):
        """Una foto tomada en `pos` mirando hacia `rel` (0 = al frente del robot)."""
        r, c = self._scan_cone(pos, scan, rel)
        self.seen[r, c] = True
        self.n_marks += 1

    # ---------- que falta ----------
    def weights(self, img):
        """-> (surface, W): superficies (bool) y peso de cada celda aun NO vista por la camara."""
        occ, unk, free = self.pl.grid(img)
        surface = occ & cv2.dilate(free.astype(np.uint8), np.ones((3, 3), np.uint8)).astype(bool)
        seen = cv2.dilate(self.seen.astype(np.uint8), np.ones((3, 3), np.uint8)).astype(bool)   # tolera 1 celda de redondeo
        W = (surface * self.ws + free * self.wf).astype(np.float32)
        W[seen] = 0.0
        return surface, W, occ

    def stats(self, img):
        """(superficies vistas, superficies totales) en el mapa actual."""
        surface, W, _ = self.weights(img)
        total = int(surface.sum())
        pending = int(((W >= self.ws * 0.99) & surface).sum())
        return total - pending, total

    def gain(self, pos, scan, rel, W):
        """Peso pendiente que veria la camara con rumbo `rel` desde `pos` (con el scan actual)."""
        r, c = self._scan_cone(pos, scan, rel)
        if r.size == 0:
            return 0.0
        flat = np.unique(r * self.pl.m + c)
        return float(W.ravel()[flat].sum())

    @staticmethod
    def heading(k):
        """Rumbo (rad, marco del mapa) del candidato k."""
        return k * 2 * math.pi / HEADINGS

    def best_heading(self, pos, scan, W, min_gain=None, exclude=()):
        """Indice k del rumbo con pendiente suficiente que exige girar menos, o None. `exclude`: ya hechos."""
        mg = self.min_gain if min_gain is None else min_gain
        best = None
        for k in range(HEADINGS):
            if k in exclude:
                continue
            rel = wrap(self.heading(k) - pos[2])
            if self.gain(pos, scan, rel, W) >= mg and (best is None or abs(rel) < best[0]):
                best = (abs(rel), k)
        return None if best is None else best[1]

    def local_gain(self, pos, scan, W):
        return max(self.gain(pos, scan, wrap(k * 2 * math.pi / HEADINGS - pos[2]), W) for k in range(HEADINGS))

    # ---------- next-best-view ----------
    def _map_gain(self, x, y, heading, occ, W, n=21):
        """Como gain() pero con el mapa (para puntos donde el robot aun no esta): rayos hasta la 1a celda ocupada."""
        a = heading + np.linspace(-self.fov / 2, self.fov / 2, n)
        t = self._t[None, :]
        px, py = x + np.cos(a)[:, None] * t, y + np.sin(a)[:, None] * t
        r, c, ok = self._cells(px, py)
        o = np.zeros(r.shape, bool)
        o[ok] = occ[r[ok], c[ok]]
        first = np.where(o.any(axis=1), o.argmax(axis=1), r.shape[1] - 1)
        ok &= np.arange(r.shape[1])[None, :] <= first[:, None]
        if not ok.any():
            return 0.0
        flat = np.unique(r[ok] * self.pl.m + c[ok])
        return float(W.ravel()[flat].sum())

    def plan_view(self, img, start_xy, top_k=14, stride=3):
        """Mejor punto de vista alcanzable. -> (x, y, rumbo, gain, path) o None si ya no queda nada que valga la pena."""
        trav, _, _, _ = self.pl.analyse(img, start_xy)
        surface, W, occ = self.weights(img)
        if W.sum() < self.min_gain:
            return None
        sr, sc = self.pl.to_cell(*start_xy)
        mask = np.zeros_like(trav)
        mask[::stride, ::stride] = True
        mask &= trav
        for bx, by in self.bad_views:                         # no repetir sitios ya visitados
            r, c = self.pl.to_cell(bx, by)
            k = int(0.5 / self.pl.cres)
            mask[max(r - k, 0):r + k + 1, max(c - k, 0):c + k + 1] = False
        if not mask.any():
            return None
        rr, cc = np.mgrid[:self.pl.m, :self.pl.m]
        dist = np.hypot(rr - sr, cc - sc) * self.pl.cres
        cands = []
        for k, K in enumerate(self._kern):
            g = cv2.filter2D(W, -1, K, borderType=cv2.BORDER_CONSTANT)
            score = np.where(mask, g / (1.0 + 0.5 * dist), 0.0)
            idx = np.argsort(score.ravel())[::-1][:top_k]
            for i in idx:
                if score.ravel()[i] > 0:
                    cands.append((float(score.ravel()[i]), int(i // self.pl.m), int(i % self.pl.m), k))
        cands.sort(reverse=True)
        verified = []
        for _, r, c, k in cands[:top_k * 3]:                   # se verifica con oclusiones reales del mapa
            x, y = self.pl.to_xy(r, c)
            h = k * 2 * math.pi / HEADINGS
            g = self._map_gain(x, y, h, occ, W)
            if g >= self.min_gain:
                verified.append((g / (1.0 + 0.5 * math.hypot(x - start_xy[0], y - start_xy[1])), x, y, h, g))
        verified.sort(reverse=True)
        for _, x, y, h, g in verified[:5]:                     # el primero con camino
            res = self.pl.plan_to(img, start_xy, (x, y))
            if res.status == "ok":
                return x, y, h, g, res.path
        return None
