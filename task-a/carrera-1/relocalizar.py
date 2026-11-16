"""
Volver EXACTO a A aunque la odometria se haya desviado (Tarea A, carrera-1).

Problema: el mapa usa la odometria de la Create 3. Con un error sistematico de 2-4 % (ruedas, giros), tras
10-20 m de ida y vuelta el robot cree que llego a A pero esta a 30-60 cm (falla con el criterio de 35 cm).

Idea: A es el unico sitio que el robot conoce de verdad, porque arranco ahi. Al empezar se guarda el scan del
LiDAR tomado en A (en el marco de A). Al volver, cuando el robot cree estar cerca de A, se alinea el scan actual
con ese scan de referencia (ICP punto a punto 2D). La alineacion dice donde esta A respecto al robot AHORA, sin
depender de la odometria acumulada, y se corrige el objetivo de la vuelta (`home`) en el marco del mapa.

Seguro por diseno: la correccion solo se acepta si la alineacion es buena (muchos puntos emparejados, error
medio pequeno) y razonable (no mueve A mas de `max_corr` m ni gira mas de `max_giro`). Si no, no cambia nada.
Coste: el vecino mas cercano sale de una tabla precalculada sobre el scan de A (transformada de distancia de
OpenCV, celdas de 2 cm), asi cada iteracion es O(N): ~2-4 ms por alineacion en una PC x86.
"""
import math

import cv2
import numpy as np


def _rot(th):
    c, s = math.cos(th), math.sin(th)
    return np.array([[c, -s], [s, c]])


def puntos_scan(scan, rmax=6.0, paso=1):
    """Puntos (N, 2) del scan en el marco del robot, sin lecturas invalidas ni muy lejanas."""
    x, y = scan.points()
    r = np.hypot(x, y)
    ok = np.isfinite(r) & (r > 0.15) & (r < rmax)
    return np.stack([x[ok], y[ok]], 1)[::paso]


class VecinoGrid:
    """Vecino mas cercano de cualquier punto a los puntos `ref`, con una tabla de celdas de `celda` m."""

    def __init__(self, ref, celda=0.02, margen=1.5):
        self.ref, self.c = ref, celda
        self.x0, self.y0 = ref.min(0) - margen
        w, h = ((ref.max(0) + margen - np.array([self.x0, self.y0])) / celda).astype(int) + 1
        img = np.ones((h, w), np.uint8)
        ij = ((ref - [self.x0, self.y0]) / celda).astype(int)
        img[ij[:, 1], ij[:, 0]] = 0
        _, lab = cv2.distanceTransformWithLabels(img, cv2.DIST_L2, 5, labelType=cv2.DIST_LABEL_PIXEL)
        # etiqueta de cada celda -> indice del punto de ref que la origino
        idx_por_lab = np.zeros(lab.max() + 1, int)
        idx_por_lab[lab[ij[:, 1], ij[:, 0]]] = np.arange(len(ref))
        self.idx = idx_por_lab[lab]
        self.h, self.w = h, w

    def __call__(self, p):
        """-> (indice del vecino en ref, distancia) para cada punto de p (N, 2)."""
        ij = ((p - [self.x0, self.y0]) / self.c).astype(int)
        ij[:, 0] = np.clip(ij[:, 0], 0, self.w - 1)
        ij[:, 1] = np.clip(ij[:, 1], 0, self.h - 1)
        j = self.idx[ij[:, 1], ij[:, 0]]
        return j, np.hypot(*(p - self.ref[j]).T)


def _vecinos_fuerza_bruta(ref):
    def f(p):
        d2 = ((p[:, None, :] - ref[None, :, :]) ** 2).sum(-1)
        j = d2.argmin(1)
        return j, np.sqrt(d2[np.arange(len(p)), j])
    return f


def icp(src, ref, init=(0.0, 0.0, 0.0), iters=30, d_max=(0.5, 0.25, 0.12), tol=1e-4, vecinos=None):
    """Alinea `src` (N, 2) sobre `ref` (M, 2). Devuelve (tx, ty, th, error_medio, fraccion_emparejada) tal que
    R(th) @ src + t ~ ref. `vecinos`: funcion p -> (indices, distancias) (VecinoGrid); por defecto fuerza bruta."""
    vecinos = vecinos or _vecinos_fuerza_bruta(ref)
    tx, ty, th = init
    n = len(src)
    if n < 20 or len(ref) < 20:
        return tx, ty, th, math.inf, 0.0
    err, frac = math.inf, 0.0
    for k in range(iters):
        dm = d_max[min(k * len(d_max) // iters, len(d_max) - 1)]          # umbral que se va cerrando
        p = src @ _rot(th).T + np.array([tx, ty])
        j, d = vecinos(p)
        sel = d < dm
        frac = float(sel.mean())
        if sel.sum() < 15:
            return tx, ty, th, math.inf, frac
        a, b = p[sel], ref[j[sel]]
        ca, cb = a.mean(0), b.mean(0)
        H = (a - ca).T @ (b - cb)
        dth = math.atan2(H[0, 1] - H[1, 0], H[0, 0] + H[1, 1])
        R = _rot(dth)
        dt = cb - R @ ca
        # componer: nueva = (R, dt) o (rot(th), t)
        t_new = R @ np.array([tx, ty]) + dt
        tx, ty, th = float(t_new[0]), float(t_new[1]), th + dth
        err = float(d[sel].mean())
        if abs(dth) < tol and np.hypot(*dt) < tol:
            break
    # error final con el umbral mas estricto
    p = src @ _rot(th).T + np.array([tx, ty])
    _, d = vecinos(p)
    sel = d < d_max[-1]
    return tx, ty, th, float(d[sel].mean()) if sel.any() else math.inf, float(sel.mean())


class Relocalizador:
    """Guarda el scan de A y, al volver, estima donde esta A de verdad.

    - `referencia(scan)`: llamar una vez con el robot quieto en A (pose (0, 0, 0) del mapa).
    - `corregir(scan, pos)`: pos = (x, y, yaw) estimada AHORA en el marco del mapa. Devuelve el nuevo `home`
      (x, y) en el marco del mapa, o None si la alineacion no es fiable.
    """

    def __init__(self, max_corr=0.8, max_giro=math.radians(30), err_max=0.06, frac_min=0.45, d_usar=2.0):
        self.ref = None
        self.max_corr, self.max_giro = max_corr, max_giro
        self.err_max, self.frac_min = err_max, frac_min
        self.d_usar = d_usar                 # solo se intenta si el robot cree estar a menos de esto de A
        self.ultimo = None                   # (tx, ty, th, err, frac) de la ultima alineacion aceptada
        self.n_ok = self.n_mal = 0

    def referencia(self, scan, pose=(0.0, 0.0, 0.0)):
        """Guarda el scan de A, llevado al marco del mapa con `pose` (casi (0, 0, 0): el robot esta en A)."""
        p = puntos_scan(scan)
        if len(p) >= 40:
            x, y, th = pose
            p = p @ _rot(th).T + np.array([x, y])
            self.ref = p[:: max(1, len(p) // 500)]
            self._vec = VecinoGrid(self.ref)
        return self.ref is not None

    def corregir(self, scan, pos, home=(0.0, 0.0)):
        """La pose del robot en el marco de A segun la odometria es simplemente `pos` (A = origen del mapa).
        ICP parte de ahi y la ajusta: (tx, ty, th) = pose real del robot en el marco de A."""
        if self.ref is None or scan is None:
            return None
        x, y, yaw = pos
        if math.hypot(x - home[0], y - home[1]) > self.d_usar:
            return None
        src = puntos_scan(scan)
        src = src[:: max(1, len(src) // 400)]
        tx, ty, th, err, frac = icp(src, self.ref, init=(x, y, yaw), vecinos=self._vec)
        dpos = math.hypot(tx - x, ty - y)
        dth = abs(math.atan2(math.sin(th - yaw), math.cos(th - yaw)))
        if not (err < self.err_max and frac > self.frac_min and dpos < self.max_corr and dth < self.max_giro):
            self.n_mal += 1
            return None
        self.n_ok += 1
        self.ultimo = (tx, ty, th, err, frac)
        # A (origen del marco de A) visto desde el robot: inv(pose real) aplicada a (0, 0)
        a_rob = _rot(th).T @ (np.zeros(2) - np.array([tx, ty]))
        # y llevado al marco del mapa con la pose estimada actual
        a_map = np.array([x, y]) + _rot(yaw) @ a_rob
        return float(a_map[0]), float(a_map[1])
