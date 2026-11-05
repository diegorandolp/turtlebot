"""
Deteccion de rostro en la camara del robot (Tarea A, auto-1).

OpenCV 5 ya no trae las cascadas Haar, asi que se usa YuNet (cv2.FaceDetectorYN) con el modelo
modelos/face_detection_yunet_2023mar.onnx (opencv_zoo, ~230 KB, ~30 ms por foto en CPU).
Aqui solo se DETECTA que hay una cara (para parar en B); reconocer a QUIEN es otra etapa.
"""
from pathlib import Path

import cv2
import numpy as np

MODEL = Path(__file__).resolve().parent / "modelos" / "face_detection_yunet_2023mar.onnx"


class FaceDetector:
    def __init__(self, model=MODEL, score=0.8, min_width=24):
        if not Path(model).exists():
            raise FileNotFoundError(f"Falta el modelo {model} (ver README de auto-1)")
        self.det = cv2.FaceDetectorYN.create(str(model), "", (250, 250), score, 0.3, 500)
        self.min_width = min_width

    def faces(self, img):
        """Lista de (x, y, w, h, score) de las caras de la imagen BGR."""
        h, w = img.shape[:2]
        self.det.setInputSize((w, h))
        _, f = self.det.detect(img)
        if f is None:
            return []
        return [(*map(float, r[:4]), float(r[-1])) for r in f if r[2] >= self.min_width]

    def __call__(self, img):
        return len(self.faces(img)) > 0


class FaceWatcher:
    """Pide fotos cada `period` s y exige `need` detecciones seguidas para confirmar.

    update() devuelve "none" | "candidate" (hay una cara: conviene parar) | "confirmed".
    """

    def __init__(self, period=1.0, need=2, retry_s=0.4, error_backoff=4.0):
        self.period, self.need, self.retry_s, self.backoff = period, need, retry_s, error_backoff
        self.hits = 0
        self.next_t = 0.0
        self.errors = 0
        self.photos = 0              # fotos recibidas
        self.last_error = ""         # ultimo fallo de la camara (para mostrarlo)
        self.last_seen = False       # la ultima foto tenia cara

    def update(self, now, take_photo, detect):
        if now < self.next_t:
            return self._status()
        try:
            img = take_photo()
            seen = bool(detect(img))
        except Exception as e:                             # camara apagada (dock), timeout, etc.
            self.errors += 1
            self.last_error = f"{type(e).__name__}: {e}"[:70]
            self.hits = 0
            self.next_t = now + self.backoff
            return "none"
        self.photos += 1
        self.last_seen = seen
        self.hits = self.hits + 1 if seen else 0
        self.next_t = now + (self.retry_s if self.hits else self.period)
        return self._status()

    def _status(self):
        if self.hits >= self.need:
            return "confirmed"
        return "candidate" if self.hits else "none"


class ThreadedWatcher:
    """FaceWatcher en un hilo propio: las fotos (hasta 4 s si la camara falla) nunca frenan el bucle de
    control ni la lectura de la tecla de parada. `status` es el ultimo resultado; `active` lo pausa."""

    def __init__(self, watcher, take_photo, detect):
        import threading
        import time
        self._w, self._photo, self._detect, self._time = watcher, take_photo, detect, time
        self.status = "none"
        self.active = False
        self._run = True
        self._t = threading.Thread(target=self._loop, daemon=True)
        self._t.start()

    def _loop(self):
        while self._run:
            if self.active:
                self.status = self._w.update(self._time.monotonic(), self._photo, self._detect)
            else:
                self.status = "none"
                self._w.hits = 0
            self._time.sleep(0.05)

    def close(self):
        self._run = False
