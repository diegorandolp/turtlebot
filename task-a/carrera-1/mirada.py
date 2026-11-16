"""
Ver la foto de B aunque este en una pared LATERAL (Tarea A, carrera-1).

Problema (sala `denso:3`): la foto esta en la pared de un pasillo angosto. El robot pasa dos veces a 30 cm de
ella sin verla: la camara (40 grados) mira hacia delante y la foto queda de costado. Ademas la cobertura de
auto-1 marcaba esa pared como "vista" al pasar, aunque la camara solo la veia de refilon (una cara vista a
mas de ~60 grados de frente no se detecta).

Dos piezas:
  CoberturaFrontal  la cobertura de auto-1, pero una superficie solo cuenta como vista si una cara pegada ahi
                    habria salido lo bastante grande para detectarla: ancho aparente
                    focal * cara_m * cos(incidencia) / distancia >= px_min (y incidencia <= `incidencia_max`).
                    Antes contaba una pared vista a 1.5 m y 60 grados, donde la cara mide ~12 px y no se detecta.
  MiradaLateral     durante la IDA, si a un costado (60-120 grados) hay superficie cercana sin mirar de frente,
                    para, gira hacia ella, espera una foto y sigue. Con limites para no perder tiempo: una cada
                    `cada_m` m recorridos y nunca mas de `max_por_min` por minuto.
"""
import math

import cv2
import numpy as np

from coverage import Coverage, wrap


class CoberturaFrontal(Coverage):
    def __init__(self, *a, incidencia_max=60.0, cara_m=0.11, px_min=20.0, ancho_img=250, **kw):
        super().__init__(*a, **kw)
        self.cos_inc = math.cos(math.radians(incidencia_max))
        focal = (ancho_img / 2) / math.tan(self.fov / 2)
        self.k_px = px_min / (focal * cara_m)            # hace falta cos(incidencia) / distancia >= k_px
        # superficies vistas de frente, aparte: Coverage.weights() dilata `seen` una celda y el espacio libre
        # justo delante de una pared (que si se ve) la tapaba como "vista"
        self.seen_sup = np.zeros_like(self.seen)
        self._normales = (None, None, None)          # (id del mapa, nx, ny)

    def _normal(self, img):
        if self._normales[0] is not id(img):
            occ, _, _ = self.pl.grid(img)
            g = cv2.GaussianBlur(occ.astype(np.float32), (5, 5), 1.0)
            gx = cv2.Sobel(g, cv2.CV_32F, 1, 0, ksize=3)       # d/d columna (x)
            gy = cv2.Sobel(g, cv2.CV_32F, 0, 1, ksize=3)       # d/d fila (y)
            n = np.hypot(gx, gy) + 1e-6
            self._normales = (id(img), -gx / n, -gy / n)       # apunta hacia el espacio libre
            self._occ = occ
        return self._normales[1], self._normales[2], self._occ

    def mark(self, pos, scan, rel=0.0, img=None, n=31):
        if img is None:
            return super().mark(pos, scan, rel)
        x, y, th = pos
        a_rel = rel + np.linspace(-self.fov / 2, self.fov / 2, n)
        d = np.minimum(self._range_at(scan, a_rel), self.rng)
        ang = th + a_rel
        t = self._t[None, :]
        px, py = x + np.cos(ang)[:, None] * t, y + np.sin(ang)[:, None] * t
        r, c, ok = self._cells(px, py)
        ok &= t <= (d[:, None] + self.pl.cres)
        nx, ny, occ = self._normal(img)
        dx = np.broadcast_to(np.cos(ang)[:, None], r.shape)
        dy = np.broadcast_to(np.sin(ang)[:, None], r.shape)
        rr, cc = np.where(ok, r, 0), np.where(ok, c, 0)
        es_sup = occ[rr, cc]
        cosi = -(dx * nx[rr, cc] + dy * ny[rr, cc])
        frontal = (cosi >= self.cos_inc) & (cosi >= self.k_px * np.broadcast_to(t, r.shape))
        libre = ok & ~es_sup
        sup = ok & es_sup & frontal                            # superficie: solo si la cara se veria
        self.seen[r[libre], c[libre]] = True
        self.seen_sup[r[sup], c[sup]] = True
        self.n_marks += 1

    def weights(self, img):
        occ, unk, free = self.pl.grid(img)
        k = np.ones((3, 3), np.uint8)
        surface = occ & cv2.dilate(free.astype(np.uint8), k).astype(bool)
        W = (surface * self.ws + free * self.wf).astype(np.float32)
        W[free & cv2.dilate(self.seen.astype(np.uint8), k).astype(bool)] = 0.0
        W[surface & cv2.dilate(self.seen_sup.astype(np.uint8), k).astype(bool)] = 0.0
        return surface, W, occ


class MiradaLateral:
    """Decide cuando mirar a un costado durante la IDA. paso() devuelve (v, w) mientras mira, o None."""

    def __init__(self, cov, gain_min=15.0, cerca=1.2, cada_m=0.6, max_por_min=6, chequeo_s=0.4, desde_s=0.0):
        self.cov = cov
        self.gain_min, self.cerca, self.cada_m = gain_min, cerca, cada_m
        self.max_por_min, self.chequeo_s = max_por_min, chequeo_s
        self.desde_s = desde_s        # s de carrera antes de empezar a mirar (las carreras rapidas no se tocan)
        self.t_inicio = None
        self.activa = None            # dict(obj=rumbo en el mapa, t0, fotos0, etapa)
        self.t_check = -1e9
        self.ult_xy = None
        self.hechas = []              # instantes
        self.n = 0

    def toca(self, now):
        if self.activa is None and self.t_inicio is not None and now - self.t_inicio < self.desde_s:
            return False
        return self.activa is not None or now - self.t_check >= self.chequeo_s

    def paso(self, now, pos, scan, mapa, fotos, wmax=1.8):
        x, y, th = pos
        if self.activa is not None:
            a = self.activa
            err = wrap(a["obj"] - th)
            if a["etapa"] == "girar":
                if abs(err) < 0.15 or now - a["t0"] > 3.0:
                    a["etapa"], a["t_quieto"], a["fotos0"] = "mirar", now, fotos
                    return 0.0, 0.0
                w = float(np.clip(2.2 * err, -wmax, wmax))
                return 0.0, math.copysign(max(abs(w), 0.5), w)
            # mirar: una foto tomada ya quieto en ese rumbo (o 1.2 s como mucho)
            if (fotos >= a["fotos0"] + 2 and now - a["t_quieto"] > 0.25) or now - a["t_quieto"] > 1.2:
                self.activa = None
                self.ult_xy = (x, y)
                return None
            return 0.0, 0.0
        if mapa is None or scan is None:
            return None                     # sin mapa en este paso: se revisa en el siguiente (que si lo pide)
        self.t_check = now
        if self.ult_xy is not None and math.hypot(x - self.ult_xy[0], y - self.ult_xy[1]) < self.cada_m:
            return None
        self.hechas = [t for t in self.hechas if now - t < 60.0]
        if len(self.hechas) >= self.max_por_min:
            return None
        _, W, _ = self.cov.weights(mapa)
        mejor = None
        for grados in (90, -90, 60, -60, 120, -120):
            rel = math.radians(grados)
            # solo superficies cercanas: con la cara a > `cerca` m de costado, ya se vera al avanzar
            g = self._gain_cerca(pos, scan, rel, W)
            if g >= self.gain_min and (mejor is None or g > mejor[0]):
                mejor = (g, rel)
        if mejor is None:
            return None
        self.activa = dict(obj=wrap(th + mejor[1]), t0=now, etapa="girar")
        self.hechas.append(now)
        self.n += 1
        return 0.0, 0.0

    def _gain_cerca(self, pos, scan, rel, W):
        rng = self.cov.rng
        self.cov.rng = min(rng, self.cerca)
        try:
            return self.cov.gain(pos, scan, rel, W)
        finally:
            self.cov.rng = rng
