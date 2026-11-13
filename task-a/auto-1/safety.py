"""
Seguridad por LiDAR de auto-1 (sin E/S, se prueba sin robot). Todo en el marco del robot: x adelante,
y izquierda, metros. Complementa (no sustituye) el freno del puente (0.28 m al frente).

  scan_points   puntos del scan sin el ruido del propio robot
  arc_clearance cuanto puede avanzar con (v, w) antes de tocar algo; mira el arco que recorreria el
                cuerpo, no un sector fijo (asi ve las esquinas y los marcos de puerta al girar)
  side_clearance holgura lateral por delante (para ir mas despacio en sitios estrechos)
  rear_clear    se puede retroceder un poco
  paint_live    pinta el scan actual en una copia del mapa: un obstaculo nuevo (o un mapa desviado) bloquea
                la ruta de inmediato, sin esperar a que el SLAM lo integre
"""
import math

import numpy as np

BODY_R = 0.17        # radio del Create 3
CLEAR_R = 0.21       # radio de choque con margen
MIN_RANGE = 0.12     # por debajo es el propio robot


def scan_points(scan):
    """(n, 2) puntos validos del scan. Vacio si no hay scan."""
    if scan is None:
        return np.zeros((0, 2))
    x, y = scan.points()
    pts = np.column_stack([x, y])
    return pts[np.hypot(pts[:, 0], pts[:, 1]) >= MIN_RANGE] if len(pts) else pts


def arc_clearance(pts, v, w, horizon=0.8, r=CLEAR_R, step=0.04):
    """Metros que puede recorrer con (v>0, w) antes de que un punto quede a menos de `r` del centro.
    `horizon` si no choca dentro de ese tramo. Ignora lo que ya esta dentro de `r` y no se acerca (pared de costado)."""
    if len(pts) == 0 or v <= 0:
        return horizon
    s = np.arange(step, horizon + 1e-9, step)
    k = w / v                                           # curvatura
    if abs(k) < 1e-3:
        cx, cy = s, np.zeros_like(s)
    else:
        th = k * s
        cx, cy = np.sin(th) / k, (1.0 - np.cos(th)) / k
    d = np.hypot(pts[None, :, 0] - cx[:, None], pts[None, :, 1] - cy[:, None])
    d0 = np.hypot(pts[:, 0], pts[:, 1])[None, :]
    hit = ((d < r) & (d < d0 - 0.005)).any(axis=1)      # dentro del radio Y acercandose
    return float(s[hit.argmax()]) if hit.any() else horizon


def side_clearance(pts, ahead=0.6, width=0.6):
    """Distancia lateral minima a los puntos de los costados, por delante del robot."""
    if len(pts) == 0:
        return math.inf
    m = (pts[:, 0] > 0.0) & (pts[:, 0] < ahead) & (np.abs(pts[:, 1]) < width)
    return float(np.abs(pts[m, 1]).min()) if m.any() else math.inf


def rear_clear(pts, dist=0.40):
    if len(pts) == 0:
        return True
    m = (pts[:, 0] < 0.0) & (np.abs(pts[:, 1]) < CLEAR_R)
    return not m.any() or float((-pts[m, 0]).min()) > dist


def paint_live(img, pts, pos, res, n, max_r=1.5):
    """Copia del mapa con el scan actual pintado como pared (0). pos = (x, y, th) en el marco del mapa."""
    if len(pts) == 0:
        return img
    near = np.hypot(pts[:, 0], pts[:, 1]) <= max_r
    p = pts[near]
    if len(p) == 0:
        return img
    x, y, th = pos
    c, s = math.cos(th), math.sin(th)
    wx, wy = x + c * p[:, 0] - s * p[:, 1], y + s * p[:, 0] + c * p[:, 1]
    col, row = (n / 2 + wx / res).astype(int), (n / 2 + wy / res).astype(int)
    ok = (col >= 0) & (col < n) & (row >= 0) & (row < n)
    out = img.copy()
    out[row[ok], col[ok]] = 0
    return out
