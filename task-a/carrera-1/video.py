"""
Video del recorrido (Tarea A, carrera-1): graba la ventana de carrera.py (camara del robot, mapa,
vista real desde arriba en el simulador y la barra de estado) en un MP4.

Pensado para que NUNCA moleste a la carrera:
  - el tiempo del video es el de la carrera (simulado en --sim, el reloj real con el robot): un video
    de una carrera de 50 s dura 50 s aunque el simulador corra mas rapido o mas lento que el reloj;
  - escribe en un hilo aparte. Con el robot real, si el disco se atrasa se pierden cuadros, nunca se
    frena el control; en el simulador espera (el video sale completo y el resultado no cambia);
  - cualquier error al grabar se avisa una vez y apaga la grabacion: la carrera sigue.

Codecs: OpenCV escribe MPEG-4 ('mp4v', viene en opencv-python). Si hay `ffmpeg` en el PATH se pasa
a H.264 (se abre en WhatsApp, PowerPoint, el navegador y el telefono). Si 'mp4v' no abre, MJPG en .avi.

Uso:
    rec = Grabador("corrida.mp4", fps=10)
    while ...:
        if rec.toca(t):                 # t en segundos (de la carrera)
            rec.agregar(cuadro_bgr, t)
    rec.cerrar()                        # -> ruta final (o None si no se grabo nada)
"""
import queue
import shutil
import subprocess
import sys
import threading
from pathlib import Path

import cv2
import numpy as np


class Grabador:
    def __init__(self, ruta, fps=10.0, bloquear=False, h264=True, cola=90, ancho_max=1600):
        self.ruta = Path(ruta)
        self.fps = float(fps)
        self.bloquear = bloquear                  # True en el simulador: no perder cuadros
        self.h264 = h264
        self.ancho_max = ancho_max
        self.q = queue.Queue(maxsize=cola)
        self.hilo = None
        self.writer = None
        self.tam = None
        self.real = None                          # ruta donde OpenCV escribe de verdad
        self.t0 = None                            # tiempo del primer cuadro
        self.n_slot = 0                           # cuadros ya puestos en el video (cada uno dura 1/fps)
        self.escritos = 0
        self.perdidos = 0
        self.error = None
        self.activo = True
        self._ultimo = None

    # ---------------------------------------------------------------- API

    def toca(self, t):
        """True si en el tiempo t ya hace falta un cuadro nuevo (asi no se compone la ventana de mas)."""
        if not self.activo:
            return False
        return self.t0 is None or t - self.t0 >= self.n_slot / self.fps - 1e-9

    def agregar(self, cuadro, t):
        """Agrega el cuadro en el tiempo t. Si el reloj salto (una pausa larga del bucle), repite el cuadro
        para que el video siga en tiempo real; si todavia no toca, no hace nada."""
        if not self.activo or cuadro is None:
            return
        try:
            if self.t0 is None:
                self.t0 = t
            n = int((t - self.t0) * self.fps + 1e-9) + 1 - self.n_slot     # cuantos cuadros cubre este instante
            if n <= 0:
                return
            n = min(n, int(5 * self.fps))                                     # un salto enorme no infla el video
            cuadro = self._normalizar(cuadro)
            self._ultimo = cuadro
            self.n_slot += n
            self._encolar(cuadro, n)
        except Exception as e:                                               # nunca tumba la carrera
            self._fallar(e)

    def sostener(self, segundos):
        """Repite el ultimo cuadro (para que el final se alcance a leer)."""
        if self.activo and self._ultimo is not None:
            n = int(segundos * self.fps)
            self.n_slot += n
            self._encolar(self._ultimo, n)

    def cerrar(self):
        """Termina de escribir y devuelve la ruta del video (o None)."""
        if self.hilo is not None:
            try:
                self.q.put(None, timeout=10)
            except queue.Full:
                pass
            self.hilo.join(timeout=60)
        if self.writer is not None:
            self.writer.release()
            self.writer = None
        if self.escritos == 0 or self.real is None or not self.real.exists():
            return None
        final = self.real
        if self.h264 and self.real.suffix == ".mp4":
            final = self._a_h264(self.real) or self.real
        if self.perdidos:
            print(f"[video] {self.perdidos} cuadros perdidos (el disco no daba abasto)", file=sys.stderr)
        return final

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.cerrar()

    # ---------------------------------------------------------------- interno

    def _normalizar(self, img):
        if img.ndim == 2:
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        if self.tam is None:                                                 # el primer cuadro fija el tamano
            h, w = img.shape[:2]
            if w > self.ancho_max:
                h, w = int(h * self.ancho_max / w), self.ancho_max
            self.tam = (w - w % 2, h - h % 2)                                 # H.264 pide lados pares
        if (img.shape[1], img.shape[0]) != self.tam:                          # la ventana cambio de tamano
            img = _encajar(img, self.tam)
        return np.ascontiguousarray(img)

    def _encolar(self, cuadro, n):
        if self.hilo is None:
            self.hilo = threading.Thread(target=self._escribir, name="grabador", daemon=True)
            self.hilo.start()
        try:
            if self.bloquear:
                self.q.put((cuadro, n), timeout=30)
            else:
                self.q.put_nowait((cuadro, n))
        except queue.Full:
            self.perdidos += n

    def _abrir(self):
        self.ruta.parent.mkdir(parents=True, exist_ok=True)
        intentos = [(self.ruta.with_suffix(".mp4"), "mp4v"), (self.ruta.with_suffix(".avi"), "MJPG")]
        for ruta, codec in intentos:
            w = cv2.VideoWriter(str(ruta), cv2.VideoWriter_fourcc(*codec), self.fps, self.tam)
            if w.isOpened():
                self.writer, self.real = w, ruta
                return
            w.release()
        raise RuntimeError(f"no se pudo abrir el video {self.ruta} (ni mp4v ni MJPG)")

    def _escribir(self):
        while True:
            item = self.q.get()
            if item is None:
                return
            if not self.activo:
                continue
            cuadro, n = item
            try:
                if self.writer is None:
                    self._abrir()
                for _ in range(n):
                    self.writer.write(cuadro)
                    self.escritos += 1
            except Exception as e:
                self._fallar(e)

    def _fallar(self, e):
        if self.error is None:
            print(f"[video] se apaga la grabacion: {type(e).__name__}: {e}", file=sys.stderr)
        self.error = f"{type(e).__name__}: {e}"
        self.activo = False

    @staticmethod
    def _a_h264(src):
        ff = shutil.which("ffmpeg")
        if ff is None:
            return None
        tmp = src.with_name(src.stem + ".h264tmp.mp4")
        cmd = [ff, "-y", "-loglevel", "error", "-i", str(src), "-c:v", "libx264", "-preset", "veryfast",
               "-crf", "23", "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(tmp)]
        try:
            r = subprocess.run(cmd, capture_output=True, timeout=600)
            if r.returncode == 0 and tmp.exists() and tmp.stat().st_size > 0:
                tmp.replace(src)
                return src
        except Exception:
            pass
        if tmp.exists():
            tmp.unlink()
        return None


def _encajar(img, tam):
    """Escala y rellena `img` para que mida exactamente tam = (ancho, alto), sin deformarla."""
    W, H = tam
    h, w = img.shape[:2]
    s = min(W / w, H / h)
    nw, nh = max(1, int(w * s)), max(1, int(h * s))
    out = np.zeros((H, W, 3), np.uint8)
    r = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_AREA if s < 1 else cv2.INTER_LINEAR)
    y, x = (H - nh) // 2, (W - nw) // 2
    out[y:y + nh, x:x + nw] = r
    return out


def ruta_auto(carpeta, nombre):
    """corridas/videos/<fecha>_<nombre>.mp4"""
    import time
    seguro = "".join(c if c.isalnum() or c in "-_" else "-" for c in nombre)
    return Path(carpeta) / (time.strftime("%Y%m%d_%H%M%S") + "_" + seguro + ".mp4")
