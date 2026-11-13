"""
Reconocimiento facial (Tarea A): QUIEN es la cara, no solo si hay una.

  1. YuNet (task-a/auto-1/face.py) localiza la cara y 5 puntos (ojos, nariz, boca).
  2. SFace (cv2.FaceRecognizerSF, opencv_zoo) alinea la cara con esos puntos y la convierte en un
     vector de 128 numeros ("embedding"). Dos fotos de la misma persona dan vectores parecidos.
  3. Se compara con la galeria (data/rostros/<Nombre>/*.jpg|heic) por similitud coseno: gana la
     persona mas parecida; si ni la mejor pasa el umbral, es "desconocido".

Umbral coseno 0.363: el recomendado por OpenCV para SFace (similitud >= 0.363 -> misma persona).
"""
import sys
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "auto-1"))
from face import FaceDetector                      # noqa: E402

SFACE = HERE / "modelos" / "face_recognition_sface_2021dec.onnx"
COS_THR = 0.363
EXTS = (".jpg", ".jpeg", ".png", ".heic")


def load_image(path, max_side=640):
    """Lee jpg/png/heic respetando la orientacion EXIF del movil; reduce a `max_side` px. -> BGR."""
    from PIL import Image, ImageOps
    if str(path).lower().endswith(".heic"):
        import pillow_heif
        pillow_heif.register_heif_opener()
    im = ImageOps.exif_transpose(Image.open(path)).convert("RGB")
    img = cv2.cvtColor(np.asarray(im), cv2.COLOR_RGB2BGR)
    s = max_side / max(img.shape[:2])
    return cv2.resize(img, None, fx=s, fy=s, interpolation=cv2.INTER_AREA) if s < 1 else img


class Recognizer:
    def __init__(self, score=0.6, min_width=10, thr=COS_THR):
        self.det = FaceDetector(score=score, min_width=min_width)
        self.rec = cv2.FaceRecognizerSF.create(str(SFACE), "")
        self.thr = thr
        self.names, self.feats = [], None          # galeria

    def embed(self, img):
        """-> (vector 128 normalizado, recuadro) de la cara mas grande, o (None, None) si no hay cara."""
        h, w = img.shape[:2]
        self.det.det.setInputSize((w, h))
        _, faces = self.det.det.detect(img)
        if faces is None or len(faces) == 0:
            return None, None
        f = max(faces, key=lambda r: r[2] * r[3])
        feat = self.rec.feature(self.rec.alignCrop(img, f)).flatten()
        return feat / np.linalg.norm(feat), f[:4]

    def add_gallery(self, root, skip=()):
        """Carga data/rostros/<Nombre>/<fotos>. Devuelve las fotos donde no encontro cara."""
        names, feats, sin_cara = [], [], []
        for p in sorted(Path(root).glob("*/*")):
            if p.suffix.lower() not in EXTS or str(p) in skip:
                continue
            feat, _ = self.embed(load_image(p))
            if feat is None:
                sin_cara.append(str(p))
                continue
            names.append(p.parent.name)
            feats.append(feat)
        self.names, self.feats = names, np.array(feats)
        return sin_cara

    def identify(self, img):
        """-> (nombre o 'desconocido' o None si no hay cara, similitud, ranking [(nombre, sim)])."""
        feat, _ = self.embed(img)
        if feat is None:
            return None, 0.0, []
        sims = self.feats @ feat
        best = {}
        for n, s in zip(self.names, sims):          # por persona, su foto mas parecida
            best[n] = max(best.get(n, -1.0), float(s))
        rank = sorted(best.items(), key=lambda t: -t[1])
        name, sim = rank[0]
        return (name if sim >= self.thr else "desconocido"), sim, rank
