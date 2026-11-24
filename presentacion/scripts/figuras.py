"""Figuras de la presentacion, sacadas del propio repositorio (nada inventado):

  * el circuito `tubo:3` del simulador (la meta del 21/11: bandera y 4 fotos) y la trayectoria de una carrera real del
    simulador: `carrera.py --sim --mundo tubo:3 --turbo --go --no-gui --mudo --traza` -> figuras/datos/tubo_traza.json
  * el LiDAR simulado en un punto de esa carrera y los 114 arcos de la ventana dinamica (navegador.arcos y
    navegador.ventana_dinamica, el mismo codigo que corre en el robot)
  * la camara simulada (raycaster) para el fondo de la portada
  * las capturas reales de la Tarea B (fotos/*.png): webcam con la mano y la camara del robot con la foto + QR
  * el reconocimiento facial (YuNet + ArcFace mbf) sobre la camara real del robot
  * el banco de plantillas del detector de flechas (flechas.banco_derecha / banco_negativo)
  * cuadros del video de esa carrera (camara del robot con la senal, la bandera, las fotos y el QR; los del informe
    van enteros a "Informe Proyecto 2/fotos_task_a/")

Corre en la distrobox con el entorno del proyecto (numpy + OpenCV), desde la raiz del repositorio:

  distrobox enter ros-jazzy -- bash -lc 'source ~/.venvs/tb4/bin/activate; python presentacion/scripts/figuras.py'

Las figuras de datos (barras, curvas) se dibujan con pgfplots en el propio .tex, con los numeros de los README.
"""
import json
import math
import sys
from pathlib import Path

import cv2
import numpy as np

AQUI = Path(__file__).resolve().parent
PRES = AQUI.parent
REPO = PRES.parent
OUT = PRES / "figuras"
GEN = OUT / "gen"
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "task-a" / "carrera-1"))

from mundos import construir  # noqa: E402
from simulador import Geometria, Camara, LIDAR_Z  # noqa: E402
import navegador as nav  # noqa: E402
import flechas as fl  # noqa: E402

MUNDO = "tubo:3"            # L de 3.3 x 3.2 m: bandera al fondo, 4 fotos (2 y 2), una senal, cajas (meta del 21/11)
TRAZA = OUT / "datos" / "tubo_traza.json"
VIDEO = Path(sys.argv[1]) if len(sys.argv) > 1 else None   # opcional: video de la misma carrera (--video)


def f(v):
    return f"{v:.3f}".rstrip("0").rstrip(".")


# ------------------------------------------------------------------------------------------ circuito
def trayectoria(d):
    """[(t, estado, x, y)]: la ruta VERDADERA del robot simulado (scripts/carrera_con_ruta.py) con el estado del
    navegador en ese instante (de la traza)."""
    traza = d["traza"]
    out, i = [], 0
    for t, x, y, _ in d["ruta"]:
        while i + 1 < len(traza) and traza[i + 1][0] <= t:
            i += 1
        out.append((t, traza[i][1], x, y))
    return out


def poli(pts):
    return " -- ".join(f"({f(a)},{f(b)})" for a, b in pts)


def circuito(m, d):
    """TikZ (coordenadas en m): paredes, senales, fotos (la buscada resaltada), A y la trayectoria por estado."""
    L = ["% generado por scripts/figuras.py: mundo " + MUNDO + " y la ruta verdadera de figuras/datos/tubo_traza.json"]
    for p in m.paredes:
        if p.alto < LIDAR_Z:
            continue
        L.append(f"\\draw[pared] ({f(p.a[0])},{f(p.a[1])}) -- ({f(p.b[0])},{f(p.b[1])});")
    for k in m.carteles:
        if k.tipo.startswith("flecha"):
            cx, cy = (k.a[0] + k.b[0]) / 2, (k.a[1] + k.b[1]) / 2
            L.append(f"\\node[senal] (senal) at ({f(cx)},{f(cy)}) {{}};")
    for k in m.carteles:                                         # la bandera a cuadros del fondo: la meta
        if k.tipo == "bandera":
            cx, cy = (k.a[0] + k.b[0]) / 2, (k.a[1] + k.b[1]) / 2
            L.append(f"\\node[bandera] (bandera) at ({f(cx)},{f(cy)}) {{}};")
    for nombre, _, (rx, ry) in m.rostros:
        st = "buscada" if nombre == m.persona else "foto"
        L.append(f"\\node[{st}] at ({f(rx)},{f(ry)}) {{}};")
    est_tikz = {"IDA": "tIda", "META": "tFila", "FILA": "tFila", "ACERCAR": "tFila", "RECONOCER": "tQr",
                "LEER_QR": "tQr", "VOLVER": "tVuelta", "HECHO": "tVuelta", "EXPLORAR": "tIda", "PAUSA": "tIda"}
    tramo, actual = [], None
    for t, est, x, y in trayectoria(d):
        e = est_tikz.get(est, "tIda")
        if e != actual and tramo:
            L.append(f"\\draw[{actual}] " + poli(tramo[::3] + tramo[-1:]) + ";")
            tramo = tramo[-1:]
        actual = e
        tramo.append((x, y))
    if len(tramo) > 1:
        L.append(f"\\draw[{actual}] " + poli(tramo[::3] + tramo[-1:]) + ";")
    x0, y0, _ = m.inicio
    L.append(f"\\node[puntoA] (A) at ({f(x0)},{f(y0)}) {{}};")
    xs = [c for p in m.paredes for c in (p.a[0], p.b[0])]
    ys = [c for p in m.paredes for c in (p.a[1], p.b[1])]
    L.append(f"% caja: x {min(xs):.2f}..{max(xs):.2f}  y {min(ys):.2f}..{max(ys):.2f}")
    (GEN / "circuito.tex").write_text("\n".join(L) + "\n")
    print("[ok] circuito.tex", f"{max(xs) - min(xs):.2f} x {max(ys) - min(ys):.2f} m")


# ------------------------------------------------------------------- LiDAR + ventana dinamica
def scan_en(m, pose, rmax=3.0):
    """El LiDAR simulado (720 rayos, sin ruido) en el marco del robot: x adelante, y a la izquierda."""
    x, y, yaw = pose
    ang = np.linspace(-math.pi, math.pi, 720, endpoint=False)
    r = Geometria(m).lidar(x, y, ang + yaw)
    ok = np.isfinite(r) & (r < rmax)
    return (r * np.cos(ang))[ok], (r * np.sin(ang))[ok]


def lidar(m, pose):
    px, py = scan_en(m, pose)
    (GEN / "lidar.tex").write_text(
        f"% generado por scripts/figuras.py: {MUNDO}, pose {pose}\n"
        "% \\lidarpuntos{tamano de cada punto}: el scan (720 rayos, < 3 m) en el marco del robot (x adelante)\n"
        "\\newcommand{\\lidarpuntos}[1]{\\path plot[only marks, mark=*, mark size=#1, "
        "mark options={fill=cInk!85, draw=none}] coordinates {" + poli(zip(px, py)).replace(" -- ", " ") + "};}\n")
    print(f"[ok] lidar.tex: {len(px)} puntos desde {pose}")


def arcos(m, pose, h_rel_deg):
    """Los arcos de navegador.arcos y el elegido por navegador.ventana_dinamica (el codigo de la carrera)."""
    px, py = scan_en(m, pose, rmax=1.5)
    h_rel = math.radians(h_rel_deg)
    v, w, ax, ay, th, c, s = nav.arcos(0.30, 1.8)
    elegido = nav.ventana_dinamica(px.astype(np.float32), py.astype(np.float32), h_rel, 0.30, 1.8)
    # la misma prueba que ventana_dinamica: distancia minima a lo largo del arco y la regla del freno del puente
    dx, dy = px[None, None, :] - ax[..., None], py[None, None, :] - ay[..., None]
    dmin = np.sqrt((dx * dx + dy * dy).min(axis=(1, 2)))
    delante = c[..., None] * dx + s[..., None] * dy
    lado = c[..., None] * dy - s[..., None] * dx
    frena = ((delante > 0) & (np.abs(lado) < 0.20) & (delante < nav.FRENO)).any(axis=(1, 2)) & (v > 0)
    seguro = (dmin >= 0.20) & ~frena
    L = [f"% generado por scripts/figuras.py: {MUNDO}, pose {pose}, rumbo deseado {h_rel_deg} grados"]
    L.append("\\path plot[only marks, mark=*, mark size=0.55pt, mark options={fill=cInk!85, draw=none}] coordinates {"
             + poli(zip(px, py)).replace(" -- ", " ") + "};")
    n_seg = n_mal = 0
    for k in range(len(v)):
        if v[k] <= 1e-6:
            continue
        estilo = "arcoOk" if seguro[k] else "arcoMal"
        n_seg += bool(seguro[k])
        n_mal += not seguro[k]
        L.append(f"\\draw[{estilo}] " + poli([(0.0, 0.0)] + list(zip(ax[k], ay[k]))) + ";")
    if elegido is not None:
        k = int(np.argmin((v - elegido[0]) ** 2 + (w - elegido[1]) ** 2))
        L.append("\\draw[arcoElegido] " + poli([(0.0, 0.0)] + list(zip(ax[k], ay[k]))) + ";")
    L.append(f"\\def\\hrel{{{h_rel_deg}}}")
    (GEN / "arcos.tex").write_text("\n".join(L) + "\n")
    print(f"[ok] arcos.tex: arcos seguros {n_seg}, descartados {n_mal}, elegido {elegido}")


# ----------------------------------------------------------------------------- portada
def portada(m, pose):
    """La camara simulada (mas ancha y grande que la del robot) mirando a lo largo del tubo de las fotos."""
    geo = Geometria(m)
    x, y, yaw = pose
    cam = Camara(geo, ancho=1600, alto=1000, hfov=80.0, capas=8, semilla=m.semilla)
    img = cam.render(x, y, yaw)
    img = cv2.GaussianBlur(img, (0, 0), 0.8)
    img = np.clip(img, 0, 255).astype(np.uint8)
    cv2.imwrite(str(GEN / "portada_fondo.jpg"), img, [cv2.IMWRITE_JPEG_QUALITY, 88])
    print("[ok] portada_fondo.jpg desde", pose)


# -------------------------------------------------------------- capturas reales de la Tarea B
GESTOS = [("girar_derecha", "der"), ("girar_izquierda", "izq"), ("dar_la_vuelta", "vuelta"),
          ("avanzar_lento_v", "lento"), ("avanzar_rapido_puno", "rapido"), ("stop_mano_abierta", "stop")]


def capturas():
    for archivo, corto in GESTOS:
        img = cv2.imread(str(REPO / "fotos" / f"{archivo}.png"))
        web = img[57 + 32:57 + 480, 35:35 + 590]          # la webcam, sin la fila de texto de arriba ni el recuadro
        cv2.imwrite(str(GEN / f"gesto_{corto}.jpg"), web, [cv2.IMWRITE_JPEG_QUALITY, 85])
    img = cv2.imread(str(REPO / "fotos" / "avanzar_lento_v.png"))
    robot = img[56:56 + 202, 954:954 + 200]              # la camara del robot (preview), con la caja de la cara
    cv2.imwrite(str(GEN / "robot_cara_qr.png"), robot)
    print("[ok] capturas de la Tarea B")


# ------------------------------------------------------------------- reconocimiento facial
def sin_dibujos(img):
    """Quita lo que la ventana de la Tarea B dibuja encima de la foto del robot (caja y nombre en verde puro)."""
    b, g, r = [img[..., i].astype(int) for i in range(3)]
    mask = ((g > 150) & (g - r > 80) & (g - b > 80)).astype(np.uint8) * 255
    mask = cv2.dilate(mask, np.ones((3, 3), np.uint8))
    return cv2.inpaint(img, mask, 3, cv2.INPAINT_TELEA)


def reconocimiento():
    from reconocimiento.rostros import Reconocedor, alinear
    robot = sin_dibujos(cv2.imread(str(REPO / "fotos" / "avanzar_lento_v.png"))[56:56 + 202, 954:954 + 200])
    cv2.imwrite(str(GEN / "robot_limpia.png"), robot)
    rec = Reconocedor("mbf", espejo=False)
    caras = rec.det(robot)
    if not caras:
        print("[!] sin cara en la captura")
        return
    c = caras[0]
    r = rec.resultado(robot, c)
    ali = alinear(robot, c)
    cv2.imwrite(str(GEN / "cara_alineada.png"), cv2.resize(ali, (224, 224), interpolation=cv2.INTER_NEAREST))
    # la deteccion: caja y 5 puntos, sobre la imagen ampliada x3 (para que se vean)
    S = 3
    big = cv2.resize(robot, None, fx=S, fy=S, interpolation=cv2.INTER_CUBIC)
    x, y, w, h = c.caja
    cv2.rectangle(big, (int(x * S), int(y * S)), (int((x + w) * S), int((y + h) * S)), (200, 95, 79), 2, cv2.LINE_AA)
    for (u, v) in c.puntos:
        cv2.circle(big, (int(u * S), int(v * S)), 4, (255, 191, 0), -1, cv2.LINE_AA)   # celeste (cLight)
    x0, y0 = max(0, int((x - 1.6 * w) * S)), max(0, int((y - 1.0 * h) * S))
    x1, y1 = min(big.shape[1], int((x + 2.6 * w) * S)), min(big.shape[0], int((y + 2.2 * h) * S))
    cv2.imwrite(str(GEN / "cara_yunet.png"), big[y0:y1, x0:x1])
    # el vector de 512 numeros, como una tira de colores
    e = rec.emb([ali])[0]
    v = e[:128]
    tira = ((v - v.min()) / (v.max() - v.min() + 1e-9) * 255).astype(np.uint8)       # de menor a mayor valor
    # del celeste (cLight) al morado (cAcc), en BGR: solo los dos colores de la presentacion
    k = (tira.astype(np.float32) / 255.0).reshape(1, -1, 1)
    tira = ((1 - k) * np.array([255, 191, 0], np.float32) + k * np.array([200, 95, 79], np.float32)).astype(np.uint8)
    cv2.imwrite(str(GEN / "embedding.png"), cv2.resize(tira, (512, 28), interpolation=cv2.INTER_NEAREST))
    orden = np.argsort(-r.sims)[:5]
    vot = __import__("reconocimiento.rostros", fromlist=["Votacion"]).Votacion(rec.nombres)
    vot.agregar(r.sims, 1.0)
    p = vot.probabilidades()
    with open(GEN / "similitudes.dat", "w") as fh:
        fh.write("i nombre sim p\n")
        for i, k in enumerate(orden):
            fh.write(f"{i} {{{rec.nombres[k]}}} {r.sims[k]:.3f} {p[k]:.3f}\n")
    print(f"[ok] reconocimiento: cara {c.ancho:.0f} px -> {r.nombre} (sim {r.sim:.2f}, margen {r.margen:.2f}),"
          f" p de una foto = {p[orden[0]]:.2f}")


# ---------------------------------------------------------------------- banco de flechas
def banco_flechas():
    der, _ = fl.banco_derecha()
    neg = fl.banco_negativo()
    sel = [der[i] for i in (1, 4, 7, 9, 13, 16, 21, 22, 24, 26)]
    tiles = []
    for m in sel:
        t = np.full((fl.S, fl.S, 3), 255, np.uint8)
        t[m > 0] = (200, 95, 79)                          # cAcc en BGR
        tiles.append(t)
    for m in (neg[0], neg[1], neg[6], neg[8], neg[-1]):
        t = np.full((fl.S, fl.S, 3), 255, np.uint8)
        t[m > 0] = (150, 138, 138)
        tiles.append(t)
    pad = lambda t: cv2.copyMakeBorder(t, 10, 10, 10, 10, cv2.BORDER_CONSTANT, value=(255, 255, 255))  # noqa: E731
    fila1 = np.hstack([pad(t) for t in tiles[:5]])
    fila2 = np.hstack([pad(t) for t in tiles[5:10]])
    fila3 = np.hstack([pad(t) for t in tiles[10:15]])
    cv2.imwrite(str(GEN / "flechas_banco.png"), np.vstack([fila1, fila2, fila3]))
    print(f"[ok] flechas_banco.png ({len(der)} plantillas a la derecha, {len(neg)} formas que no son flecha)")


# ------------------------------------------------------------------ cuadros del video
# (segundo, nombre, para): del video de la carrera simulada tubo:3 --turbo. La camara se recorta sin la barra de arriba
# (BUSCO/QR) para la presentacion; el cuadro entero (camara, mapa, circuito y estado) va al informe.
CUADROS = [(2.5, "sim_flecha", "ambos"),      # IDA: una senal de giro vista y confirmada
           (17.0, "sim_bandera", "ambos"),    # META: la bandera a cuadros del fondo, detectada
           (79.6, "sim_qr", "presentacion")]  # LEER_QR: la cara buscada y su QR
INFORME_FOTOS = REPO / "Informe Proyecto 2" / "fotos_task_a"


def cuadros_video():
    if VIDEO is None or not VIDEO.exists():
        print("[--] sin video: no se extraen cuadros (pasar la ruta del MP4 de la misma carrera)")
        return
    cap = cv2.VideoCapture(str(VIDEO))
    for seg, nombre, para in CUADROS:
        cap.set(cv2.CAP_PROP_POS_MSEC, seg * 1000)
        ok, img = cap.read()
        if not ok:
            print("[!] el video no llega a", seg, "s")
            continue
        if para in ("ambos", "presentacion"):
            cam = img[40:400, 0:400]
            filas = np.nonzero(cam.reshape(cam.shape[0], -1).max(axis=1) > 12)[0]    # la foto HD (16:9) deja negro abajo
            cv2.imwrite(str(GEN / f"{nombre}.png"), cam[:filas.max() + 1] if len(filas) else cam)
        if para in ("ambos", "informe") and INFORME_FOTOS.exists():
            cv2.imwrite(str(INFORME_FOTOS / f"{nombre}.png"), img)
    print("[ok] cuadros del video")


def main():
    GEN.mkdir(parents=True, exist_ok=True)
    m = construir(MUNDO)
    d = json.loads(TRAZA.read_text())
    circuito(m, d)
    lidar(m, (0.90, 0.0, 0.0))                      # en el primer pasillo, mirando a la senal; la caja a la izquierda
    arcos(m, (0.50, -0.10, 0.35), -20)               # apuntando a la caja, queriendo seguir por el pasillo
    portada(m, (2.40, -0.30, -math.pi / 2))          # desde la boca del tubo, mirando la bandera del fondo
    capturas()
    reconocimiento()
    banco_flechas()
    cuadros_video()


if __name__ == "__main__":
    main()
