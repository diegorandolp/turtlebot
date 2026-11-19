"""
Comprueba que el robot (Raspberry Pi 4B del TurtleBot 4 Lite) puede correr carrera-1 SIN la laptop. Se ejecuta
EN el robot (robot/desplegar_carrera.sh --verificar lo hace por SSH):

    cd ~/turtlebot && python3 task-a/carrera-1/verificar_pi.py

Revisa, sin mover el robot: Python / numpy / OpenCV (y si tiene el lector de QR de WeChat), que carguen YuNet,
ArcFace y la galeria, que el puente UDP responda en 127.0.0.1 (LiDAR y odometria llegando), memoria libre,
temperatura y cuanto tarda cada pieza en ESTA maquina (una carrera a ritmo normal debe quedar por debajo
de ~1 nucleo: la Raspberry tiene 4 y ROS + LiDAR + camara + puente ya usan parte).
"""
import os
import platform
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
for p in (HERE, HERE.parent / "auto-1", HERE.parent / "slam-1", ROOT):
    sys.path.insert(0, str(p))

fallos, avisos = [], []


def ok(msg):
    print(f"[ok]    {msg}")


def mal(msg):
    print(f"[FALLA] {msg}")
    fallos.append(msg)


def aviso(msg):
    print(f"[aviso] {msg}")
    avisos.append(msg)


print(f"Python {platform.python_version()} en {platform.machine()} ({platform.node()}), {os.cpu_count()} nucleos")
if platform.machine() not in ("aarch64", "armv7l"):
    aviso("esto no parece una Raspberry: los tiempos de abajo no son los del robot")

# ---- memoria y temperatura ----
try:
    mem = {l.split(":")[0]: int(l.split()[1]) for l in open("/proc/meminfo") if ":" in l}
    libre = mem.get("MemAvailable", 0) / 1024
    (ok if libre > 700 else mal)(f"memoria disponible: {libre:.0f} MB (la carrera usa ~300 MB; ROS + camara ~1 GB)")
except OSError:
    aviso("no pude leer /proc/meminfo")
try:
    t = int(open("/sys/class/thermal/thermal_zone0/temp").read()) / 1000
    (ok if t < 70 else aviso)(f"temperatura CPU: {t:.0f} C (a 80 C la Raspberry baja la frecuencia)")
except OSError:
    pass

# ---- librerias ----
try:
    import numpy as np
    ok(f"numpy {np.__version__}")
except ImportError:
    mal("falta numpy: robot/desplegar_carrera.sh --instalar")
    sys.exit(1)
try:
    import cv2
    ok(f"OpenCV {cv2.__version__}")
    if hasattr(cv2, "wechat_qrcode_WeChatQRCode"):
        ok("lector de QR de WeChat disponible (opencv-contrib)")
    else:
        aviso("sin WeChat (opencv-python a secas): el QR se lee desde mas cerca. Mejor opencv-contrib-python-headless")
except ImportError:
    mal("falta OpenCV: robot/desplegar_carrera.sh --instalar")
    sys.exit(1)
cv2.setNumThreads(1)

# ---- modelos ----
try:
    from reconocimiento import Reconocedor
    t0 = time.perf_counter()
    rec = Reconocedor(modelo="mbf")
    ok(f"YuNet + ArcFace mbf + galeria cargados en {time.perf_counter() - t0:.1f} s ({len(rec.nombres)} personas)")
except Exception as e:                                           # noqa: BLE001
    mal(f"no cargan los modelos: {e} (copiar reconocimiento/modelos/*.onnx y galeria_mbf.npz)")
    rec = None

# ---- piezas: ms en esta maquina ----
if rec is not None:
    img = np.full((250, 250, 3), 120, np.uint8)
    cara = cv2.imread(str(ROOT / "reconocimiento" / "prueba.jpg")) if (ROOT / "reconocimiento" / "prueba.jpg").exists() else None
    if cara is None:
        cara = img
    cara = cv2.resize(cara, (250, 250))
    rec.det(cara)                                                # calentar
    t0 = time.perf_counter()
    for _ in range(5):
        rec.det(cara)
    ms_det = (time.perf_counter() - t0) / 5 * 1000
    ok(f"detectar caras (YuNet, 250x250): {ms_det:.0f} ms")
    t0 = time.perf_counter()
    rec.analizar(cara)
    ok(f"detectar + reconocer la foto de prueba: {(time.perf_counter() - t0) * 1000:.0f} ms")
    try:
        from flechas import DetectorFlechas
        df = DetectorFlechas(forma=True)
        t0 = time.perf_counter()
        for _ in range(5):
            df(cara)
        ok(f"detectar flechas: {(time.perf_counter() - t0) / 5 * 1000:.0f} ms")
    except Exception as e:                                       # noqa: BLE001
        aviso(f"detector de flechas: {e}")
    try:
        from navegador import ventana_dinamica, ANGS
        px, py = 2.0 * np.cos(ANGS), 2.0 * np.sin(ANGS)
        t0 = time.perf_counter()
        for _ in range(10):
            ventana_dinamica(px, py, 0.0, 0.3, 1.8, 0.25, False)
        ms_vd = (time.perf_counter() - t0) / 10 * 1000
        ok(f"ventana dinamica: {ms_vd:.0f} ms (corre a 20 Hz: {ms_vd * 20 / 10:.0f} % de un nucleo)")
    except Exception as e:                                       # noqa: BLE001
        aviso(f"ventana dinamica: {e}")
    if ms_det > 120:
        aviso("la deteccion es lenta: usar --cam-periodo 0.3 y sin --hd (ver medir_cpu.py para el total)")

# ---- puente UDP ----
try:
    from tb4.client import TB4Client
    cl = TB4Client("127.0.0.1")
    cl.wait_ready(5.0)
    st = cl.status()
    if st is None:
        mal("el puente no responde en 127.0.0.1:5600 (robot/tb4_inicio.sh o robot/deploy_bridge.sh desde la PC)")
    else:
        ok(f"puente UDP responde: {st}")
        fin = time.monotonic() + 5.0                              # LiDAR (~7 Hz) y odometria (~20 Hz) llegan despues del STATUS
        while time.monotonic() < fin and (cl.scan() is None or cl.pose() is None):
            time.sleep(0.1)
        sc = cl.scan()
        (ok if sc is not None and len(sc.ranges) > 100 else mal)("LiDAR llegando por el puente" if sc else "sin LiDAR (robot en el dock? undock)")
        pose = cl.pose()
        (ok if pose is not None else mal)(f"odometria: {pose}")
        for hd in (False, True):                                  # sin camara la carrera va a ciegas
            try:
                img = cl.photo(timeout=4.0, hd=hd)
                ok(f"camara {'HD' if hd else 'preview'}: {img.shape[1]}x{img.shape[0]}")
            except Exception as e:                               # noqa: BLE001
                (aviso if hd else mal)(f"camara {'HD' if hd else 'preview'}: {e} -> task-a/camara/camara_rgb.sh --hd")
except Exception as e:                                           # noqa: BLE001
    mal(f"cliente UDP: {e}")

print()
if fallos:
    print(f"{len(fallos)} FALLA(S): arreglar antes de correr la carrera en el robot")
    sys.exit(1)
print(f"todo ok ({len(avisos)} aviso(s)). Siguiente: python3 task-a/carrera-1/medir_cpu.py y luego "
      "robot/desplegar_carrera.sh --correr \"Nombre Apellido\"")
