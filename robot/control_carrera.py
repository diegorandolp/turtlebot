"""
Mando de la Tarea A desde la PC, por UDP (a traves del router), SIN hacer computo aqui: la carrera corre EN el
robot (robot/carrera.service o desplegar_carrera.sh --correr). Este programa solo manda START y espera el
anuncio, que imprime en la terminal (y lo dice en voz alta si la PC tiene voz).

    python3 robot/control_carrera.py                 # START y esperar el anuncio (robot $TB4_IP o 192.168.0.104)
    python3 robot/control_carrera.py 192.168.0.105   # otra IP
    python3 robot/control_carrera.py --solo-esperar  # no dar la salida (la da el boton 1), solo esperar el anuncio
    python3 robot/control_carrera.py --luz           # prueba: anillo verde 2 s + nota (comprueba audio/luces)
"""
import argparse
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tb4.client import TB4Client  # noqa: E402
from reconocimiento.voz import anunciar  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("ip", nargs="?", default=os.environ.get("TB4_IP", "192.168.0.104"))
    ap.add_argument("--solo-esperar", action="store_true")
    ap.add_argument("--luz", action="store_true")
    ap.add_argument("--espera", type=float, default=240.0, help="s maximos esperando el anuncio")
    args = ap.parse_args()
    cl = TB4Client(args.ip)
    st = cl.wait_ready(5.0)
    print(f"[ok] puente en {args.ip}: bateria {st.battery:.0%}, clientes {st.n_clients}"
          f"{'' if st.n_clients >= 2 else '  (AVISO: solo yo: la carrera NO esta corriendo en el robot?)'}")
    if args.luz:
        print(cl.luz(0, 255, 0, 2000), cl.audio([(523, 150), (784, 300)]))
        return
    if not args.solo_esperar:
        cl.start()
        print("[START] enviado. Esperando el anuncio...")
    t0 = time.monotonic()
    while time.monotonic() - t0 < args.espera:
        for _, texto in cl.textos():
            print(f"[{time.monotonic() - t0:5.1f} s] {texto}", flush=True)
            if texto.startswith("ANUNCIO:"):
                linea = texto[8:].strip()
                print("\n" + "=" * (len(linea) + 8) + f"\n    {linea}\n" + "=" * (len(linea) + 8) + "\n", flush=True)
                anunciar(linea.replace("_", " ").replace("-", " "), imprimir=False)
                cl.close()
                return
        time.sleep(0.2)
    print("[fin] sin anuncio en el tiempo de espera")
    cl.close()


if __name__ == "__main__":
    main()
