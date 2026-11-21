"""
Mientras la persona conduce con gestos, este hilo mira por la CÁMARA DEL ROBOT: busca una cara, la reconoce
(reconocimiento/, votando entre varias fotos) y lee el QR que esté junto a ella (task-a/carrera-1/qr.py).

    v = Vigia(robot, hd=False); v.start()
    ...
    v.persona()            # None hasta que la votación decide; luego 'Nombre Apellido'
    v.qr()                 # None hasta leer el QR (se busca DESPUÉS de reconocer a la persona, sin límite)
    Vigia(robot, buscado="Chiara Lazaro")   # solo vale esa persona: otras caras se descartan y sigue buscando
    v.reiniciar()          # buscar de nuevo (otra persona)
    v.detener()

No mueve el robot: solo pide fotos (robot.photo) en segundo plano, así no frena el bucle de los gestos.
"""
import sys
import threading
import unicodedata
import time
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "task-a" / "carrera-1"))

from reconocimiento import Reconocedor, Votacion, nombre_bonito, peso_por_tamano   # noqa: E402


def normalizar(nombre):
    """'Álvaro_garcía' -> 'alvaro garcia' (para comparar nombres sin tildes, mayúsculas ni guiones bajos)."""
    s = unicodedata.normalize("NFKD", nombre.replace("_", " ").replace("-", " "))
    return " ".join("".join(c for c in s if not unicodedata.combining(c)).lower().split())


@dataclass
class Hallazgo:
    nombre: str
    prob: float
    qr: str | None
    foto: object          # imagen BGR de la última foto con la cara (para mostrarla)


class Vigia(threading.Thread):
    def __init__(self, robot, hd=False, modelo="auto", olvido=4.0, pausa=0.05, buscado=None,
                 calidad=70):
        """olvido: s sin ver cara -> se borra la votación (era otra persona o pasó de largo)."""
        super().__init__(daemon=True)
        self.robot, self.hd, self.olvido, self.pausa = robot, hd, olvido, pausa
        self.calidad = calidad       # JPEG más liviano = llega antes por Wi-Fi (la cara se reconoce igual)
        self.rec = Reconocedor(modelo)
        try:
            from qr import buscar_qrs
            self._qrs = buscar_qrs
        except Exception as e:                                       # sin QR se sigue reconociendo caras
            print("aviso: no se pudo cargar el lector de QR:", e)
            self._qrs = None
        self.buscado = None
        if buscado:
            por_nombre = {normalizar(n): nombre_bonito(n) for n in self.rec.nombres}
            if normalizar(buscado) not in por_nombre:
                raise ValueError(f"'{buscado}' no está en la galería. Personas: " + ", ".join(sorted(por_nombre.values())))
            self.buscado = por_nombre[normalizar(buscado)]
        self.descartado = None       # última persona reconocida que no era la buscada
        self.voto = Votacion(self.rec.nombres)
        self.lock = threading.Lock()
        self._fin = threading.Event()
        self._persona = None         # 'Nombre Apellido' reconocido
        self._qr = None              # texto del QR leído
        self._t_cara = 0.0           # última vez que se vio una cara
        self.ultima = None          # (img, cajas, texto) para dibujar
        self.estado = self._buscando()
        self.error = None
        self.demora = None           # s que tardó la última foto en llegar
        self._foto = None            # (img, t) más nueva sin procesar
        self.nueva = threading.Condition()

    def persona(self):
        with self.lock:
            return self._persona

    def qr(self):
        with self.lock:
            return self._qr

    def terminado(self):
        return self.persona() is not None and self.qr() is not None

    def reiniciar(self):
        with self.lock:
            self.voto.reiniciar()
            self._persona, self._qr = None, None
            self.estado = self._buscando()

    def _buscando(self):
        return f"buscando a {self.buscado}" if self.buscado else "buscando cara"

    def detener(self):
        self._fin.set()

    def _traer(self):
        """Hilo 1: pide fotos al robot sin parar y guarda solo la MÁS NUEVA (no espera a que se procese)."""
        while not self._fin.is_set():
            if self.terminado():
                time.sleep(0.2)
                continue
            t = time.monotonic()
            try:
                img = self.robot.photo(quality=self.calidad, **({"hd": True} if self.hd else {}))
            except Exception as e:                                   # foto perdida o cámara arrancando
                self.error = "camara del robot arrancando / sin imagen" if "imagen" in str(e) or "Timeout" in type(e).__name__ else str(e)
                time.sleep(0.2)
                continue
            self.error = None
            ahora = time.monotonic()
            self.demora = ahora - t                                  # pedido -> foto en la PC
            with self.nueva:
                self._foto = (img, ahora)
                self.nueva.notify()
            time.sleep(self.pausa)

    def run(self):
        """Hilo 2: procesa (cara + QR) la foto más nueva que haya; las que llegan mientras tanto se saltan."""
        threading.Thread(target=self._traer, daemon=True).start()
        while not self._fin.is_set():
            with self.nueva:
                self.nueva.wait_for(lambda: self._foto is not None or self._fin.is_set(), 0.5)
                foto, self._foto = self._foto, None
            if foto is None or self.terminado():
                continue
            self._procesar(*foto)

    def _cara_de(self, img, caras, persona, max_caras=6):
        """Índice de la cara de `persona` entre `caras` (la que la reconoce como su mejor opción y con más
        similitud), o None si ninguna es ella. Con varias fotos juntas, así se sabe cuál es la buscada."""
        mejor, sim_mejor = None, -1.0
        for i, c in enumerate(caras[:max_caras]):
            r = self.rec.resultado(img, c)
            if nombre_bonito(r.nombre) == persona and r.sim > sim_mejor:
                mejor, sim_mejor = i, r.sim
        return mejor

    def _leer_qr(self, img, caras, persona):
        """Solo el QR que está DEBAJO de la cara de `persona` (qr.py lo asigna a su cara); los QR de las otras
        fotos se ignoran. Sin la cara de la persona en la imagen no se acepta ningún QR."""
        if self._qrs is None or not caras:
            return None
        i = self._cara_de(img, caras, persona)
        if i is None:
            return None
        cajas = [c.caja for c in caras]
        propios = [q for q in self._qrs(img, cajas) if q.cara == i]
        return propios[0].texto if propios else None

    def _procesar(self, img, ahora):
        caras = self.rec.det(img)
        cajas = [c.caja for c in caras]

        if self.persona() is not None:                             # 2.ª fase: ya se sabe quién es -> el QR
            qr = self._leer_qr(img, caras, self.persona())
            with self.lock:
                self.ultima = (img, cajas, self._persona)
                if qr:
                    self._qr = qr
                    self.estado = f"QR reconocido: {qr}"
                else:
                    self.estado = f"Persona: {self._persona} reconocida - buscando el QR (acerca el robot)"
            return

        with self.lock:                                            # 1.ª fase: ¿quién es?
            if not caras:
                if self.voto.n and ahora - self._t_cara > self.olvido:
                    self.voto.reiniciar()
                    self.estado = self._buscando()
                self.ultima = (img, cajas, None)
                return
            self._t_cara = ahora
            k = 0
            if self.buscado and len(caras) > 1:                    # varias fotos: votar con la de la buscada
                k = self._cara_de(img, caras, self.buscado) or 0
            r = self.rec.resultado(img, caras[k])
            self.voto.agregar(r.sims, peso_por_tamano(caras[k].ancho))
            nombre, p = self.voto.lider()
            self.ultima = (img, cajas, nombre_bonito(nombre))
            self.estado = f"cara: {nombre_bonito(nombre)} {p * 100:.0f}% ({self.voto.n} fotos)"
            if not self.voto.decidido():
                return
            if self.buscado and nombre_bonito(nombre) != self.buscado:
                self.descartado = nombre_bonito(nombre)              # no es: olvidar y seguir buscando
                self.voto.reiniciar()
                self.estado = f"{self.descartado} no es {self.buscado}: sigo buscando"
                return
            self._persona = nombre_bonito(nombre)
            self.estado = f"Persona: {self._persona} reconocida - buscando el QR"
        qr = self._leer_qr(img, caras, self._persona)              # quizá ya se lee en esta misma foto
        if qr:
            with self.lock:
                self._qr = qr
                self.estado = f"QR reconocido: {qr}"
