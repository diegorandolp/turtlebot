"""
Reconocimiento facial comun a las Tareas A y B: quien es la persona de la foto en el punto B.

Cadena (todo con OpenCV: cv2.FaceDetectorYN y cv2.dnn; no hace falta onnxruntime ni insightface):
  1. YuNet encuentra las caras y 5 puntos de cada una (ojos, nariz, comisuras de la boca).
  2. Con esos puntos la cara se alinea a 112x112 (la plantilla con la que se entreno ArcFace).
  3. ArcFace (MobileFaceNet entrenado en WebFace600K, 13 MB) la convierte en un vector de 512
     numeros. Se suma el de la imagen espejada: el vector queda mas estable.
  4. Similitud coseno contra la galeria (los vectores de las fotos de datos/rostros, ver galeria.py).
     Conjunto cerrado: la respuesta siempre es la persona mas parecida.
  5. Votacion: cada foto suma evidencia (peso segun el tamano de la cara) y se decide cuando el
     lider saca suficiente ventaja, o al llegar al maximo de fotos.

Uso:
    from reconocimiento import Reconocedor, Votacion
    rec = Reconocedor()                       # carga modelos y galeria
    r = rec.mejor(img_bgr)                    # la cara mas grande, o None
    if r: print(r.nombre, r.sim, r.margen)
"""
import math
from dataclasses import dataclass, field
from pathlib import Path

import cv2
import numpy as np

AQUI = Path(__file__).resolve().parent
MODELOS = AQUI / "modelos"
ROSTROS = AQUI.parent / "datos" / "rostros"
YUNET = MODELOS / "yunet_2023mar.onnx"
EMBEBEDORES = {
    "mbf": MODELOS / "arcface_mbf.onnx",                                     # 13 MB, en el repo
    "r50": Path.home() / ".insightface" / "models" / "buffalo_l" / "w600k_r50.onnx",   # 174 MB, opcional
}

# Donde caen los 5 puntos en la cara alineada de 112x112 (plantilla estandar de ArcFace/InsightFace).
# YuNet da los puntos en el mismo orden (el ojo que se ve a la izquierda de la imagen primero).
ARCFACE_5P = np.array([[38.2946, 51.6963], [73.5318, 51.5014], [56.0252, 71.7366],
                       [41.5493, 92.3655], [70.7299, 92.2041]], np.float32)


def galeria_path(modelo):
    return MODELOS / f"galeria_{modelo}.npz"


def modelo_auto():
    """'r50' si esta instalado (mejor con caras chicas, 174 MB fuera del repo), si no 'mbf' (en el repo)."""
    return "r50" if EMBEBEDORES["r50"].exists() and galeria_path("r50").exists() else "mbf"


def nombre_bonito(carpeta):
    """'Alvaro_García' -> 'Alvaro García'."""
    return carpeta.replace("_", " ").strip()


@dataclass
class Cara:
    caja: tuple             # (x, y, w, h) en px de la imagen original
    puntos: np.ndarray      # (5, 2) ojos, nariz, comisuras
    score: float            # confianza del detector (0..1)

    @property
    def ancho(self):
        return float(self.caja[2])

    @property
    def centro(self):
        x, y, w, h = self.caja
        return x + w / 2.0, y + h / 2.0


class Detector:
    """YuNet. Las imagenes chicas (la camara del robot da 250x250) se amplian antes de detectar:
    YuNet pierde las caras de menos de ~20 px, y ampliando x2 recupera las de ~12-20 px."""

    def __init__(self, modelo=YUNET, umbral=0.6, nms=0.3, min_lado=500, max_lado=800):
        if not Path(modelo).exists():
            raise FileNotFoundError(f"Falta el modelo {modelo}")
        self.det = cv2.FaceDetectorYN.create(str(modelo), "", (320, 320), umbral, nms, 50)
        self.min_lado, self.max_lado = min_lado, max_lado

    def __call__(self, img):
        """Lista de Cara, de la mas grande a la mas chica."""
        h, w = img.shape[:2]
        s = 1.0
        if max(h, w) > self.max_lado:
            s = self.max_lado / max(h, w)
        elif max(h, w) < self.min_lado:
            s = min(2.0, self.min_lado / max(h, w))
        im = img if s == 1.0 else cv2.resize(img, None, fx=s, fy=s,
                                              interpolation=cv2.INTER_AREA if s < 1 else cv2.INTER_CUBIC)
        self.det.setInputSize((im.shape[1], im.shape[0]))
        _, f = self.det.detect(im)
        if f is None:
            return []
        caras = [Cara(tuple(float(v) / s for v in r[:4]), r[4:14].reshape(5, 2).astype(np.float32) / s, float(r[14]))
                 for r in f]
        return sorted(caras, key=lambda c: -c.ancho)


def alinear(img, cara, size=112):
    """Recorte 112x112 de la cara, girado y escalado para que los 5 puntos caigan en la plantilla."""
    M, _ = cv2.estimateAffinePartial2D(cara.puntos, ARCFACE_5P * (size / 112.0), method=cv2.LMEDS)
    if M is None:                                    # puntos degenerados: recorte simple de la caja
        x, y, w, h = cara.caja
        M = np.array([[size / w, 0, -x * size / w], [0, size / h, -y * size / h]], np.float32)
    return cv2.warpAffine(img, M, (size, size), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)


class Embebedor:
    """ArcFace con cv2.dnn: cara alineada 112x112 BGR -> vector 512-D de norma 1."""

    def __init__(self, modelo="mbf"):
        path = Path(EMBEBEDORES.get(modelo, modelo))
        if not path.exists():
            raise FileNotFoundError(f"Falta el modelo de reconocimiento {path}")
        self.modelo = modelo
        self.net = cv2.dnn.readNetFromONNX(str(path))

    def __call__(self, caras112):
        out = []
        for im in caras112:                          # de a una: algunos ONNX fijan el lote en 1
            blob = cv2.dnn.blobFromImages([im, cv2.flip(im, 1)], 1.0 / 127.5, (112, 112),
                                          (127.5, 127.5, 127.5), swapRB=True)
            self.net.setInput(blob)
            y = self.net.forward()
            e = y[0] + y[1]                          # original + espejo
            out.append(e / (np.linalg.norm(e) + 1e-9))
        return np.array(out, np.float32).reshape(len(out), -1)


@dataclass
class Galeria:
    nombres: list               # una entrada por persona
    emb: np.ndarray             # (n, d) vectores de norma 1
    etiqueta: np.ndarray        # (n,) indice en `nombres` de cada vector
    fuente: list = field(default_factory=list)      # archivo de origen de cada vector
    modelo: str = "mbf"

    def guardar(self, path):
        np.savez_compressed(path, nombres=np.array(self.nombres), emb=self.emb.astype(np.float32),
                            etiqueta=self.etiqueta.astype(np.int16), fuente=np.array(self.fuente),
                            modelo=np.array(self.modelo))

    @classmethod
    def cargar(cls, path):
        z = np.load(path, allow_pickle=False)
        return cls([str(n) for n in z["nombres"]], z["emb"].astype(np.float32), z["etiqueta"].astype(int),
                   [str(f) for f in z["fuente"]], str(z["modelo"]))

    def similitudes(self, e, excluir=None):
        """Similitud de un vector con cada PERSONA: la de su foto mas parecida (vecino mas cercano).
        `excluir`: mascara (n,) de vectores a ignorar (para la evaluacion dejando una foto fuera)."""
        s = self.emb @ e
        if excluir is not None:
            s = np.where(excluir, -1.0, s)
        out = np.full(len(self.nombres), -1.0, np.float32)
        np.maximum.at(out, self.etiqueta, s)
        return out


@dataclass
class Resultado:
    cara: Cara
    sims: np.ndarray            # similitud con cada persona
    nombre: str
    sim: float                  # similitud del mas parecido
    margen: float               # ventaja sobre el segundo


class Reconocedor:
    def __init__(self, modelo="mbf", galeria=None, detector=None, excluir=()):
        """modelo: 'mbf' | 'r50' | 'auto'. `excluir`: archivos de la galeria a ignorar ('Carpeta/foto.jpg');
        el simulador pone en B una foto de la galeria y la excluye: una foto que el reconocedor nunca vio."""
        modelo = modelo_auto() if modelo == "auto" else modelo
        self.modelo = modelo
        self.det = detector or Detector()
        self.emb = Embebedor(modelo)
        path = galeria or galeria_path(modelo)
        if not Path(path).exists():
            raise FileNotFoundError(f"Falta la galeria {path}: python -m reconocimiento.galeria --modelo {modelo}")
        self.gal = Galeria.cargar(path)
        if self.gal.modelo != modelo:
            raise ValueError(f"La galeria {path} es del modelo {self.gal.modelo}, no de {modelo}")
        self._excluir = np.array([f.split("#")[0] in set(excluir) for f in self.gal.fuente]) if excluir else None

    @property
    def nombres(self):
        return self.gal.nombres

    def resultado(self, img, cara, excluir=None):
        e = self.emb([alinear(img, cara)])[0]
        sims = self.gal.similitudes(e, self._excluir if excluir is None else excluir)
        orden = np.argsort(-sims)
        return Resultado(cara, sims, self.gal.nombres[orden[0]], float(sims[orden[0]]),
                         float(sims[orden[0]] - sims[orden[1]]) if len(orden) > 1 else 1.0)

    def analizar(self, img, max_caras=3):
        return [self.resultado(img, c) for c in self.det(img)[:max_caras]]

    def mejor(self, img, caras=None):
        """La cara mas grande de la imagen, reconocida; None si no hay caras."""
        caras = self.det(img) if caras is None else caras
        return self.resultado(img, caras[0]) if caras else None


def peso_por_tamano(ancho_px, chico=14.0, bueno=56.0):
    """Cuanto cuenta una foto en la votacion segun el ancho de la cara (px de la imagen original)."""
    return float(np.clip((ancho_px - chico) / (bueno - chico), 0.15, 1.0))


class Votacion:
    """Junta la evidencia de varias fotos de la MISMA cara. Conjunto cerrado: siempre hay un lider.

    Cada foto aporta log-probabilidades softmax(sims / temperatura), multiplicadas por su peso.
    decidido() se cumple con `min_fotos` y probabilidad >= `prob`, o al juntar `max_fotos`.
    """

    def __init__(self, nombres, temperatura=0.04, min_fotos=2, max_fotos=8, prob=0.97):
        self.nombres = list(nombres)
        self.T, self.min_fotos, self.max_fotos, self.prob_min = temperatura, min_fotos, max_fotos, prob
        self.reiniciar()

    def reiniciar(self):
        self.logp = np.zeros(len(self.nombres))
        self.n = 0
        self.peso = 0.0

    def agregar(self, sims, peso=1.0):
        z = np.asarray(sims, float) / self.T
        z -= z.max()
        self.logp += peso * (z - math.log(np.exp(z).sum()))
        self.n += 1
        self.peso += peso

    def probabilidades(self):
        z = self.logp - self.logp.max()
        p = np.exp(z)
        return p / p.sum()

    def lider(self):
        p = self.probabilidades()
        i = int(np.argmax(p))
        return self.nombres[i], float(p[i])

    def decidido(self):
        if self.n >= self.max_fotos:
            return True
        return self.n >= self.min_fotos and self.peso >= 1.0 and self.lider()[1] >= self.prob_min

    def top(self, k=3):
        p = self.probabilidades()
        return [(self.nombres[i], float(p[i])) for i in np.argsort(-p)[:k]]
