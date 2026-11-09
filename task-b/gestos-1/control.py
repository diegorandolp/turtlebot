"""
Del gesto confirmado a la velocidad del robot (lógica pura, sin cámara ni robot: se prueba sola).

Comportamiento:
  - SEÑALAR (índice) en una dirección: el robot deja de avanzar y GIRA en el sitio hacia ese lado, relativo
    a hacia donde mira: derecha/izquierda 90°, diagonales de arriba 45°, de abajo 135°, abajo 180°.
    Al terminar el giro se queda QUIETO.
  - ✌️ V: AVANZA recto lento. 🤟 3 dedos: AVANZA rápido. Si está girando, primero termina el giro.
    Mientras avanza, cambiar entre V y 3 dedos solo cambia la velocidad.
  - STOP: se para en el acto (aunque esté girando) y no hace nada hasta la próxima señal.
  - Mantener la misma señal no repite el giro. Bajar la mano y volver a señalar sí.
  - Sin mano (NINGUNO): sigue haciendo lo que estaba haciendo.

Mientras avanza mantiene el rumbo con la odometría (corrige pequeñas desviaciones).
"""
import math

from gestos import CENTROS, LENTO, RAPIDO, STOP, es_senal

PARADO, GIRANDO, AVANZANDO = "PARADO", "GIRANDO", "AVANZANDO"

# Giro de cada señal (grados, + = izquierda / antihorario): el ángulo de la señal respecto a "arriba"
GIROS = {g: (c - 90.0 + 180.0) % 360.0 - 180.0 for g, c in CENTROS.items()}
GIROS["ABA"] = 180.0


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Control:
    def __init__(self, v_lento=0.07, v_rapido=0.15, w_max=0.6, w_min=0.15, tol_deg=3.0, k_giro=1.5, k_rumbo=1.5):
        self.v = {LENTO: v_lento, RAPIDO: v_rapido}
        self.w_max, self.w_min = w_max, w_min
        self.tol = math.radians(tol_deg)
        self.k_giro, self.k_rumbo = k_giro, k_rumbo
        self.estado = PARADO
        self.rumbo = None          # yaw objetivo (rad, odometría)
        self.rumbo_ok = False      # el rumbo sirve de base para el próximo giro/avance (no tras un STOP)
        self.vel = 0.0             # velocidad de avance pedida (m/s); 0 = quedarse quieto tras girar
        self.ultimo = None         # último gesto confirmado visto
        self.giro_deg = 0.0        # último giro pedido (para mostrarlo)

    def parar(self):
        self.estado, self.vel, self.rumbo_ok = PARADO, 0.0, False

    def _base(self, yaw):
        """Rumbo desde el que contar: el objetivo anterior (no acumula error) o, tras un STOP, donde está."""
        return self.rumbo if self.rumbo_ok and self.rumbo is not None else yaw

    def _evento(self, gesto, yaw):
        if gesto == STOP:
            self.parar()
        elif gesto in self.v:
            self.vel = self.v[gesto]
            if self.estado == PARADO:
                self.rumbo, self.rumbo_ok, self.estado = self._base(yaw), True, AVANZANDO
        elif es_senal(gesto):
            self.giro_deg = GIROS[gesto]
            self.rumbo = wrap(self._base(yaw) + math.radians(self.giro_deg))
            self.rumbo_ok = True
            self.vel = 0.0
            self.estado = GIRANDO

    def paso(self, gesto, yaw):
        """Llamar en cada cuadro con el gesto confirmado y el yaw actual (rad). Devuelve (v, w)."""
        if gesto != self.ultimo:
            self.ultimo = gesto
            if yaw is not None or gesto == STOP:
                self._evento(gesto, yaw)

        if self.estado == PARADO or yaw is None:
            return 0.0, 0.0
        err = wrap(self.rumbo - yaw)
        if self.estado == GIRANDO:
            if abs(err) > self.tol:
                w = max(-self.w_max, min(self.w_max, self.k_giro * err))
                if abs(w) < self.w_min:
                    w = math.copysign(self.w_min, err)
                return 0.0, w
            if self.vel <= 0:
                self.estado = PARADO                    # quieto, pero conserva el rumbo como base
                return 0.0, 0.0
            self.estado = AVANZANDO
        w = max(-0.5, min(0.5, self.k_rumbo * err))
        return self.vel, w

    def texto(self):
        if self.estado == PARADO:
            return "PARADO"
        if self.estado == GIRANDO:
            lado = "izq" if self.giro_deg > 0 else "der"
            return "DANDO LA VUELTA" if abs(self.giro_deg) == 180 else f"GIRANDO {abs(self.giro_deg):.0f} {lado}"
        return f"AVANZANDO {'rapido' if self.vel >= self.v[RAPIDO] else 'lento'} {self.vel:.2f} m/s"
