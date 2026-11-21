"""Detector de la mano (MediaPipe HandLandmarker) y dibujo de los puntos. Lo usan los dos programas."""
import sys
import threading
from pathlib import Path

import cv2
import mediapipe as mp
from mediapipe.tasks import python as mp_python
from mediapipe.tasks.python import vision

MODELO = Path(__file__).resolve().parent / "modelos" / "hand_landmarker.task"
URL_MODELO = ("https://storage.googleapis.com/mediapipe-models/hand_landmarker/"
              "hand_landmarker/float16/1/hand_landmarker.task")

CONEXIONES = [(0, 1), (1, 2), (2, 3), (3, 4), (0, 5), (5, 6), (6, 7), (7, 8), (5, 9), (9, 10), (10, 11),
              (11, 12), (9, 13), (13, 14), (14, 15), (15, 16), (13, 17), (0, 17), (17, 18), (18, 19), (19, 20)]


def crear_detector(video=True):
    if not MODELO.exists():
        sys.exit(f"Falta el modelo {MODELO}\nDescárgalo de:\n  {URL_MODELO}")
    opts = vision.HandLandmarkerOptions(
        base_options=mp_python.BaseOptions(model_asset_path=str(MODELO)),
        running_mode=vision.RunningMode.VIDEO if video else vision.RunningMode.IMAGE,
        num_hands=1,
        min_hand_detection_confidence=0.6,
        min_hand_presence_confidence=0.5,
        min_tracking_confidence=0.5,
    )
    return vision.HandLandmarker.create_from_options(opts)


def detectar(det, bgr, t_ms=None):
    """Devuelve los 21 puntos (x, y) normalizados de la primera mano, o None."""
    img = mp.Image(image_format=mp.ImageFormat.SRGB, data=cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
    res = det.detect_for_video(img, t_ms) if t_ms is not None else det.detect(img)
    if not res.hand_landmarks:
        return None
    return [(p.x, p.y) for p in res.hand_landmarks[0]]


class Camara:
    """Webcam leída en un hilo aparte que se queda solo con el cuadro MÁS RECIENTE: así el programa nunca
    procesa cuadros viejos acumulados en el búfer (eso es lo que da el "lag" cuando el bucle va más lento
    que la cámara). Misma interfaz que cv2.VideoCapture: read() y release()."""

    def __init__(self, cap):
        self.cap = cap
        self.lock = threading.Lock()
        self.nuevo = threading.Condition(self.lock)
        self.frame, self.ok, self.n, self.leido = None, True, 0, 0
        self._fin = False
        self.hilo = threading.Thread(target=self._leer, daemon=True)
        self.hilo.start()

    def _leer(self):
        while not self._fin:
            ok, frame = self.cap.read()
            with self.lock:
                self.ok, self.frame = ok, frame
                self.n += 1
                self.nuevo.notify_all()
            if not ok:
                break

    def read(self, timeout=2.0):
        """El cuadro más reciente que aún no se haya entregado (espera uno nuevo si hace falta)."""
        with self.lock:
            if not self.nuevo.wait_for(lambda: self.n != self.leido or not self.ok, timeout):
                return False, None
            self.leido = self.n
            return self.ok, self.frame

    def release(self):
        self._fin = True
        self.hilo.join(timeout=1.0)
        self.cap.release()


def abrir_camara(idx, ancho=640, alto=480, fps=30):
    """Abre la webcam a 640x480 (MediaPipe no gana nada con más y es más rápido) y la lee en un hilo."""
    cap = None
    if sys.platform == "win32":                       # DirectShow abre rápido y acepta MJPG en Windows
        cap = cv2.VideoCapture(idx, cv2.CAP_DSHOW)
        if not cap.isOpened():
            cap = None
    if cap is None:
        cap = cv2.VideoCapture(idx)
    if not cap.isOpened():
        sys.exit(f"No pude abrir la cámara {idx} (¿estás en WSL? usa el Python de Windows: python.exe ...)")
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, ancho)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, alto)
    cap.set(cv2.CAP_PROP_FPS, fps)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return Camara(cap)


def dibujar_mano(img, pts):
    if not pts:
        return
    h, w = img.shape[:2]
    px = [(int(x * w), int(y * h)) for x, y in pts]
    for a, b in CONEXIONES:
        cv2.line(img, px[a], px[b], (0, 255, 0), 2)
    for i, p in enumerate(px):
        punta = i in (8, 12)
        cv2.circle(img, p, 6 if punta else 3, (0, 0, 255) if punta else (255, 255, 255), -1)


def texto(img, s, org, escala=0.55, color=(255, 255, 255), grosor=2):
    cv2.putText(img, s, org, cv2.FONT_HERSHEY_SIMPLEX, escala, (0, 0, 0), grosor + 2)
    cv2.putText(img, s, org, cv2.FONT_HERSHEY_SIMPLEX, escala, color, grosor)
