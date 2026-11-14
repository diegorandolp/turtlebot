"""Lectura del codigo QR (Tarea A, laberinto-1). Prueba varias variantes de la foto porque el QR impreso
se ve pequeno, con poco contraste o inclinado."""
import cv2

_det = None
_aruco = None


def leer_qr(img):
    """-> texto del QR o None. Prueba: imagen tal cual, gris con contraste, y ampliada x2 / x3 si es pequena."""
    global _det, _aruco
    if img is None or img.size == 0:
        return None
    if _det is None:
        _det = cv2.QRCodeDetector()
        _aruco = cv2.QRCodeDetectorAruco() if hasattr(cv2, "QRCodeDetectorAruco") else None
    gris = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img
    variantes = [img, cv2.equalizeHist(gris)]
    if max(img.shape[:2]) < 900:
        variantes += [cv2.resize(gris, None, fx=k, fy=k, interpolation=cv2.INTER_CUBIC) for k in (2, 3)]
    for v in variantes:
        for d in (_det, _aruco):
            if d is None:
                continue
            try:
                txt, _, _ = d.detectAndDecode(v)
            except cv2.error:
                continue
            if txt:
                return txt
    return None


import math


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class FaseQR:
    """Con el robot frente a la foto de la persona, busca y lee el QR de debajo. Controla el robot (v, w) durante
    esta fase. Sin E/S: el bucle le pasa una foto nueva (o None) y la pose.

    Secuencia (cada paso lee cada foto nueva; termina en cuanto el QR se decodifica):
      1. quieto `t_quieto` s     2. gira +giro   3. gira -giro   4. vuelve al rumbo inicial
      5. avanza despacio hasta `d_min` del frente (el QR sale mas grande)   6. quieto `t_final` s -> falla.
    """

    def __init__(self, leer=leer_qr, t_quieto=3.0, giro_deg=9.0, t_giro=2.0, v=0.05, d_min=0.42, t_avance=8.0,
                 t_final=3.0, wmax=0.4):
        self.leer, self.t_quieto, self.giro = leer, t_quieto, math.radians(giro_deg)
        self.t_giro, self.v, self.d_min, self.t_avance, self.t_final, self.wmax = t_giro, v, d_min, t_avance, t_final, wmax
        self.texto, self.terminada, self.paso, self.intentos = None, False, 0, 0
        self._t0 = self._th0 = None

    @property
    def ok(self):
        return self.texto is not None

    def step(self, now, frame, pos, scan, front):
        """-> (v, w). `front` = distancia libre al frente (m)."""
        if self.terminada:
            return 0.0, 0.0
        th = pos[2]
        if self._t0 is None:
            self._t0, self._th0 = now, th
        if frame is not None:
            self.intentos += 1
            t = self.leer(frame)
            if t:
                self.texto, self.terminada = t, True
                return 0.0, 0.0
        dt = now - self._t0
        w = lambda objetivo: float(max(-self.wmax, min(self.wmax, 1.8 * _wrap(objetivo - th))))
        if self.paso == 0:                                     # quieto
            if dt > self.t_quieto:
                self._sig(now)
            return 0.0, 0.0
        if self.paso in (1, 2, 3):                              # +giro, -giro, rumbo inicial
            objetivo = self._th0 + {1: self.giro, 2: -self.giro, 3: 0.0}[self.paso]
            if abs(_wrap(objetivo - th)) < 0.04 and dt > self.t_giro or dt > self.t_giro + 4.0:
                self._sig(now)
            return 0.0, w(objetivo)
        if self.paso == 4:                                      # acercarse
            if front <= self.d_min or dt > self.t_avance:
                self._sig(now)
                return 0.0, 0.0
            return self.v, w(self._th0)
        if dt > self.t_final:                                   # paso 5: ultimo intento, luego se rinde
            self.terminada = True
        return 0.0, 0.0

    def _sig(self, now):
        self.paso += 1
        self._t0 = now
