"""
Registro de caras vistas (Tarea A, auto-1): DONDE esta cada foto en el mapa y si ya se reviso.

Cuando la camara detecta una cara:
  1. direccion: con el centro del recuadro en la imagen y el campo de vision de la camara;
  2. distancia: el LiDAR en esa misma direccion (la foto esta pegada a una pared: el LiDAR mide la pared);
  3. orientacion de la pared: recta ajustada a los puntos del LiDAR cerca de la foto (su normal apunta al
     robot). Con eso se calcula un PUNTO DE VISTA: a `d_vista` m delante de la foto, mirandola de frente.

Cada cara queda como "pendiente" hasta que el robot va a su punto de vista y el reconocedor dice quien es:
"revisada" (otra persona), "objetivo" (la buscada) o "inalcanzable".
"""
import math

import numpy as np


def estimar_cara(caja, pos, scan, fov_deg, ancho_img=250, ventana_deg=3.0):
    """caja = (x, y, w, h, ...) en la imagen; pos = (x, y, rumbo) del robot en el mapa.
    -> (fx, fy, normal) en el mapa (normal = angulo hacia el que mira la pared), o None si el LiDAR no da distancia."""
    x, y, th = pos
    cx = caja[0] + caja[2] / 2.0
    f = (ancho_img / 2.0) / math.tan(math.radians(fov_deg) / 2.0)
    rumbo = math.atan2(ancho_img / 2.0 - cx, f)                 # + = a la izquierda del robot
    a = np.asarray(scan.angles, float)
    r = np.asarray(scan.ranges, float)
    da = np.abs(np.arctan2(np.sin(a - rumbo), np.cos(a - rumbo)))
    sel = (da < math.radians(ventana_deg)) & np.isfinite(r)
    if not sel.any():
        return None
    dist = float(np.median(r[sel]))
    fx, fy = x + dist * math.cos(th + rumbo), y + dist * math.sin(th + rumbo)
    # normal de la pared: recta por los puntos del LiDAR a menos de 0.35 m de la foto
    ok = np.isfinite(r)
    px = x + r[ok] * np.cos(th + a[ok])
    py = y + r[ok] * np.sin(th + a[ok])
    cerca = np.hypot(px - fx, py - fy) < 0.35
    hacia_robot = math.atan2(y - fy, x - fx)
    if cerca.sum() >= 4:
        P = np.stack([px[cerca] - px[cerca].mean(), py[cerca] - py[cerca].mean()], 1)
        dx, dy = np.linalg.svd(P, full_matrices=False)[2][0]    # direccion de la pared
        n = math.atan2(dx, -dy)                                 # perpendicular
        if math.cos(n - hacia_robot) < 0:
            n += math.pi                                        # que apunte hacia el lado del robot
        normal = math.atan2(math.sin(n), math.cos(n))
    else:
        normal = hacia_robot
    return fx, fy, normal


class RegistroCaras:
    def __init__(self, fusion=0.5, d_vista=0.55):
        self.caras = []                     # dicts: x, y, normal, estado, vistas, nombre
        self.fusion, self.d_vista = fusion, d_vista
        self.actual = None

    def agregar(self, fx, fy, normal):
        """Nueva deteccion: si ya hay una cara a menos de `fusion` m, la actualiza (media); si no, la apunta."""
        for c in self.caras:
            if math.hypot(c["x"] - fx, c["y"] - fy) < self.fusion:
                k = c["vistas"]
                c["x"], c["y"] = (c["x"] * k + fx) / (k + 1), (c["y"] * k + fy) / (k + 1)
                s = math.sin(c["normal"]) * k + math.sin(normal)
                co = math.cos(c["normal"]) * k + math.cos(normal)
                c["normal"] = math.atan2(s, co)
                c["vistas"] = k + 1
                return c, False
        c = dict(x=fx, y=fy, normal=normal, estado="pendiente", vistas=1, nombre=None)
        self.caras.append(c)
        return c, True

    def siguiente(self, pos, now=0.0):
        """La cara pendiente mas cercana al robot (o None). Se saltan las que estan "en espera": se vieron de lejos
        y aun no hay ruta hasta su punto de vista (esa zona del mapa no se ha explorado)."""
        pend = [c for c in self.caras if c["estado"] == "pendiente" and c.get("espera", 0.0) <= now]
        if not pend:
            return None
        return min(pend, key=lambda c: math.hypot(c["x"] - pos[0], c["y"] - pos[1]))

    def punto_de_vista(self, c):
        """A `d_vista` m delante de la foto, mirandola de frente: (x, y, rumbo)."""
        return (c["x"] + self.d_vista * math.cos(c["normal"]), c["y"] + self.d_vista * math.sin(c["normal"]),
                math.atan2(math.sin(c["normal"] + math.pi), math.cos(c["normal"] + math.pi)))

    def pendientes(self):
        return sum(c["estado"] == "pendiente" for c in self.caras)
