"""Detector de la mano (MediaPipe HandLandmarker) y dibujo de los puntos. Lo usan los dos programas."""
import sys
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


def abrir_camara(idx):
    cap = cv2.VideoCapture(idx)
    if not cap.isOpened() and sys.platform == "win32":
        cap = cv2.VideoCapture(idx, cv2.CAP_DSHOW)
    if not cap.isOpened():
        sys.exit(f"No pude abrir la cámara {idx} (¿estás en WSL? usa el Python de Windows: python.exe ...)")
    return cap


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
